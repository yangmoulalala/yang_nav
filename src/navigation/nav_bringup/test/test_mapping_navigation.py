"""Run the mapping planners against a rolling map fed by synthetic clearing rays."""

import math
import os
import signal
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def test_mapping_observations_extend_single_goal_but_never_substitute_a_waypoint(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    domain = 191
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('mapping_navigation_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log = tmp_path / 'mapping_launch.log'
    with log.open('w') as output:
        process = subprocess.Popen(
            ['ros2', 'launch', 'nav_bringup', 'navigation.launch.py', 'profile:=sim',
             'mode:=mapping', 'use_sim_time:=false', 'cmd_vel_topic:=/mapping_test/unused'],
            stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
            env=dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOG_DIR=str(tmp_path)))
    try:
        broadcaster = TransformBroadcaster(node)
        transform = TransformStamped(header=Header(frame_id='map'), child_frame_id='gimbal_nav')
        transform.transform.rotation.w = 1.0
        clearing = node.create_publisher(PointCloud2, '/nav/clearing', qos_profile_sensor_data)
        hits = node.create_publisher(PointCloud2, '/nav/obstacles', qos_profile_sensor_data)
        odom = node.create_publisher(Odometry, '/localization/odometry', 10)
        radius = 1.8
        robot_x = 0.0
        latest_grid = None
        directions = [(math.cos(index * math.tau / 1080), math.sin(index * math.tau / 1080))
                      for index in range(1080)]

        def receive_grid(grid):
            nonlocal latest_grid
            latest_grid = grid

        def publish_observations():
            stamp = node.get_clock().now().to_msg()
            transform.header.stamp = stamp
            transform.transform.translation.x = robot_x
            broadcaster.sendTransform(transform)
            header = Header(stamp=stamp, frame_id='gimbal_nav')
            clearing.publish(create_cloud_xyz32(header, [(radius * dx, radius * dy, 0.) for dx, dy in directions]))
            hits.publish(create_cloud_xyz32(header, []))
            odometry = Odometry(header=Header(stamp=stamp, frame_id='map'), child_frame_id='gimbal_nav')
            odometry.pose.pose.position.x = robot_x
            odometry.pose.pose.orientation.w = 1.0
            odom.publish(odometry)

        node.create_subscription(OccupancyGrid, '/global_costmap/costmap', receive_grid,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_timer(0.05, publish_observations)
        single = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        multi = ActionClient(node, ComputePathThroughPoses, '/compute_path_through_poses')
        navigate = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        lifecycle = node.create_client(GetState, '/bt_navigator/get_state')

        def wait_for(predicate, timeout=15):
            deadline = time.monotonic() + timeout
            while process.poll() is None and time.monotonic() < deadline:
                if predicate():
                    return
                executor.spin_once(timeout_sec=0.1)
            raise AssertionError(log.read_text()[-12000:])

        def result(future):
            wait_for(future.done)
            return future.result()

        def is_active():
            return result(lifecycle.call_async(GetState.Request())).current_state.label == 'active'

        def cell(x, y):
            if latest_grid is None:
                return None
            info = latest_grid.info
            col = math.floor((x - info.origin.position.x) / info.resolution)
            row = math.floor((y - info.origin.position.y) / info.resolution)
            return latest_grid.data[row * info.width + col]

        def pose(x):
            target = PoseStamped(header=Header(frame_id='map'))
            target.pose.position.x = x
            target.pose.orientation.w = 1.0
            return target

        def run_action(client, goal):
            handle = result(client.send_goal_async(goal))
            assert handle.accepted, log.read_text()[-12000:]
            return result(handle.get_result_async())

        def plan(x, planner='GridBased'):
            return run_action(single, ComputePathToPose.Goal(goal=pose(x), planner_id=planner))

        wait_for(lambda: all(client.server_is_ready() for client in (single, multi, navigate))
                 and lifecycle.service_is_ready())
        wait_for(is_active)
        wait_for(lambda: cell(0., 0.) == 0 and cell(3.2, 0.) == -1)

        partial = plan(3.2)
        assert partial.status == GoalStatus.STATUS_SUCCEEDED, partial
        endpoint = partial.result.path.poses[-1].pose.position.x
        assert 0.5 < endpoint < 1.4
        assert all(cell(point.pose.position.x, point.pose.position.y) == 0 for point in partial.result.path.poses)
        assert plan(20.).result.path.poses[-1].pose.position.x == endpoint
        route = ComputePathThroughPoses.Goal(goals=[pose(0.8), pose(3.2)], planner_id='StrictGridBased')
        assert run_action(multi, route).status == GoalStatus.STATUS_ABORTED

        navigation_feedback = []
        handle = result(navigate.send_goal_async(NavigateToPose.Goal(pose=pose(3.2)),
                                                feedback_callback=navigation_feedback.append))
        assert handle.accepted
        navigation_result = handle.get_result_async()
        wait_for(lambda: any(sample.feedback.distance_remaining > 0.5 for sample in navigation_feedback))
        robot_x = endpoint - 0.10
        wait_for(lambda: any(0.03 < sample.feedback.distance_remaining < 0.15
                             for sample in navigation_feedback))
        assert not navigation_result.done()

        radius = 4.2
        wait_for(lambda: cell(3.7, 0.) == 0)
        navigation_feedback.clear()
        wait_for(lambda: any(sample.feedback.distance_remaining > 1.0 for sample in navigation_feedback))
        robot_x = 3.2
        assert result(navigation_result).status == GoalStatus.STATUS_SUCCEEDED

        robot_x = 0.0
        wait_for(lambda: cell(0., 0.) == 0)
        complete = plan(3.2)
        assert complete.status == GoalStatus.STATUS_SUCCEEDED, complete
        assert complete.result.path.poses[-1].pose.position.x == 3.2
        assert run_action(multi, route).status == GoalStatus.STATUS_SUCCEEDED
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
