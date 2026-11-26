"""Exercise the real goal client against a fake Nav2 in an isolated ROS domain.

No navigation stack, sensors, chassis or velocity publishers are started.
"""

import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from unittest.mock import patch

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


PACKAGE = Path(__file__).resolve().parents[1]
CLIENT = PACKAGE / 'scripts' / 'send_goal.py'
# Test participants use their own domain, never the caller's vehicle domain.
DOMAIN = 186


class FakeNav(Node):
    def __init__(self, context):
        super().__init__('fake_nav_for_goal_test', context=context)
        self.active = False
        self.trusted = False
        self.status = 'NORMAL'
        self.extrapolated = False
        self.publish_tf = False
        self.base_frame = 'gimbal_nav'
        self.reject = False
        self.abort = False
        self.hold = False
        self.cancelled = False
        self.goals = []
        self.display_goals = []
        self.interactive_goals = []
        self.display_sub = self.create_subscription(
            PoseStamped, '/navigation/goal_accepted', self.display_goals.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.interactive_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.interactive_goals.append, 10)
        self.state = self.create_publisher(
            String, '/backend_node/localization_state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = TransformBroadcaster(self)
        self.nav_map = self.create_publisher(OccupancyGrid, '/nav_map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.costmap = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', 1)
        self.service = self.create_service(GetState, '/bt_navigator/get_state', self.get_state)
        self.server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose', self.execute,
            goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.timer = self.create_timer(0.05, self.publish)

    def get_state(self, request, response):
        response.current_state.id = State.PRIMARY_STATE_ACTIVE if self.active else State.PRIMARY_STATE_INACTIVE
        return response

    def publish(self):
        self.state.publish(String(data=f'status={self.status};pose_trusted={str(self.trusted).lower()};'
                                        f'pose_extrapolated={str(self.extrapolated).lower()}'))
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = 1.
        grid.info.width = grid.info.height = 1
        grid.info.origin.orientation.w = 1.
        grid.data = [0]
        self.nav_map.publish(grid)
        self.costmap.publish(grid)
        if self.publish_tf:
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = self.base_frame
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.transform.rotation.w = 1.0
            self.tf.sendTransform(transform)

    def on_goal(self, request):
        self.goals.append(request)
        return GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT

    def execute(self, handle):
        deadline = time.monotonic() + 20.0
        while self.hold and time.monotonic() < deadline:
            if handle.is_cancel_requested:
                self.cancelled = True
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        result = NavigateToPose.Result()
        if self.abort:
            result.error_code = 42
            result.error_msg = 'fake planner failure'
            handle.abort()
        else:
            handle.succeed()
        return result


@pytest.fixture
def nav():
    context = Context()
    rclpy.init(context=context, domain_id=DOMAIN)
    node = FakeNav(context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    processes = []

    def client(*extra, timeout=10.0):
        env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
        process = subprocess.Popen(
            [sys.executable, str(CLIENT), '1.0', '1.5', '--yaw', str(math.pi / 2),
             '--wait-timeout', str(timeout), *extra],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        processes.append(process)
        return process

    yield node, client
    node.hold = False
    for process in processes:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.communicate(timeout=6)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
    executor.shutdown(timeout_sec=5.0)
    thread.join(timeout=5.0)
    node.server.destroy()
    node.destroy_node()
    context.shutdown()


def finish(process, expected):
    output, _ = process.communicate(timeout=20)
    assert process.returncode == expected, output
    return output


def wait_until(predicate):
    deadline = time.monotonic() + 12
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert predicate()


def test_waits_then_sends_exactly_one_map_goal(nav):
    node, client = nav
    process = client()
    time.sleep(1.0)
    assert not node.goals
    node.active = True
    time.sleep(0.7)
    assert not node.goals  # Untrusted localization must hold the goal.
    node.trusted = True
    time.sleep(0.7)
    assert not node.goals  # Missing map -> gimbal_nav must also hold it.
    node.publish_tf = True
    assert 'Navigation goal succeeded' in finish(process, 0)
    assert len(node.goals) == 1
    wait_until(lambda: len(node.display_goals) == 1)
    assert node.display_goals[0] == node.goals[0].pose
    assert not node.interactive_goals
    pose = node.goals[0].pose
    assert pose.header.frame_id == 'map'
    assert pose.header.stamp.sec > 0
    assert (pose.pose.position.x, pose.pose.position.y) == (1.0, 1.5)
    assert pose.pose.orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert pose.pose.orientation.w == pytest.approx(math.cos(math.pi / 4))


def test_goal_sender_accepts_an_explicit_reference_override(nav):
    node, client = nav
    node.base_frame = 'test_nav_reference'
    node.active = node.trusted = node.publish_tf = True
    output = finish(client('--ros-args', '-p', 'robot_base_frame:=test_nav_reference'), 0)
    assert 'Navigation goal succeeded' in output
    assert len(node.goals) == 1


def test_suspect_waits_for_extrapolated_pose(nav):
    node, client = nav
    node.active = node.publish_tf = True
    node.status = 'SUSPECT'
    assert 'no goal sent' in finish(client(timeout=3), 2)
    assert not node.goals
    node.extrapolated = True
    assert 'Navigation goal succeeded' in finish(client(), 0)
    assert len(node.goals) == 1


@pytest.mark.parametrize('missing', ['active', 'trusted', 'publish_tf', 'clock'])
def test_readiness_timeout_sends_nothing(nav, missing):
    node, client = nav
    node.active = node.trusted = node.publish_tf = True
    extra = []
    if missing == 'clock':
        extra = ['--ros-args', '-p', 'use_sim_time:=true']
    else:
        setattr(node, missing, False)
    output = finish(client(*extra, timeout=3), 2)
    assert 'no goal sent' in output
    assert not node.goals
    assert not node.display_goals


@pytest.mark.parametrize('mode,expected,log', [
    ('reject', 3, 'Navigation goal rejected'),
    ('abort', 4, 'error_code=42'),
])
def test_reports_goal_failure(nav, mode, expected, log):
    node, client = nav
    node.active = node.trusted = node.publish_tf = True
    setattr(node, mode, True)
    assert log in finish(client(), expected)
    assert len(node.goals) == 1
    if mode == 'reject':
        assert not node.display_goals
    else:
        wait_until(lambda: len(node.display_goals) == 1)
    assert not node.interactive_goals


def test_ctrl_c_cancels_only_the_sent_goal(nav):
    node, client = nav
    node.active = node.trusted = node.publish_tf = node.hold = True
    process = client()
    wait_until(lambda: len(node.goals) == 1)
    time.sleep(0.2)
    process.send_signal(signal.SIGINT)
    assert 'cancellation accepted' in finish(process, 130)
    wait_until(lambda: node.cancelled)


def test_launch_validates_goal_and_defaults_to_no_sender():
    spec = importlib.util.spec_from_file_location('navigation_launch', PACKAGE / 'launch' / 'navigation.launch.py')
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    values = dict(profile='real', use_sim_time='', terrain_dir=str(PACKAGE), params_file='',
                  require_trusted='true', cmd_vel_topic='/cmd_vel', rviz='false',
                  goal_x='', goal_y='', goal_yaw='0', goal_wait_timeout='120')
    with patch.object(launch, '_get', side_effect=lambda _, key: values[key]), \
            patch.object(launch, 'get_package_share_directory', return_value=str(PACKAGE)), \
            patch.object(Path, 'is_file', return_value=True):
        assert all('send_goal.py' not in str(action.node_executable) for action in launch.build(None))
        values.update(goal_x='1.0', goal_y='1.5')
        senders = [action for action in launch.build(None) if 'send_goal.py' in str(action.node_executable)]
        assert len(senders) == 1
        for key, value in [('goal_y', ''), ('goal_x', 'nan'), ('goal_yaw', 'inf'),
                           ('goal_wait_timeout', '0'), ('goal_x', 'bad')]:
            previous = values[key]
            values[key] = value
            with pytest.raises(RuntimeError):
                launch.build(None)
            values[key] = previous
