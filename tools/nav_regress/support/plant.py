"""Deterministic omni plant driven by the final command topic."""
import copy
import json
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from localization_interfaces.msg import FrontendFrame
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster


def angle(v):
    return math.atan2(math.sin(v), math.cos(v))


class Plant(Node):
    def __init__(self, out, step_period=.02):
        super().__init__('nav_acceptance_plant')
        self.pose = np.array([-3., -2., 0.])
        self.command = Twist()
        self.command_time = 0.
        self.last_time = time.monotonic()
        self.observations = True
        self.trusted = True
        self.blocker = None
        self.phase = 'none'
        self.phase_history = []
        self.mode_history = []
        self.samples = []
        self.tf = TransformBroadcaster(self)
        self.frames = self.create_publisher(FrontendFrame, '/frontend/frame', qos_profile_sensor_data)
        self.odometry = self.create_publisher(Odometry, '/localization/odometry', 5)
        self.state = self.create_publisher(String, '/backend_node/localization_state',
                                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(String, '/nav_zone_state', self.on_phase,
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/nav_mode', lambda m: self.mode_history.append(m.data),
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.single = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.multi = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.create_timer(step_period, self.step)
        self.trace = (out / 'trace.jsonl').open('w')
        self.frame_tick = 0

    def on_command(self, cmd):
        self.command, self.command_time = cmd, time.monotonic()

    def on_phase(self, msg):
        self.phase = msg.data.split(' ')[0]
        self.phase_history.append(self.phase)

    def step(self):
        now = time.monotonic()
        dt = min(.05, now - self.last_time)
        self.last_time = now
        cmd = self.command if now - self.command_time < .3 else Twist()
        x, y, yaw = self.pose
        yaw_rate = cmd.angular.z + getattr(self, 'passive_yaw_rate', 0.0)
        mid = yaw + yaw_rate * dt / 2
        self.pose += [dt * (math.cos(mid) * cmd.linear.x - math.sin(mid) * cmd.linear.y),
                      dt * (math.sin(mid) * cmd.linear.x + math.cos(mid) * cmd.linear.y),
                      dt * yaw_rate]
        self.pose[2] = angle(self.pose[2])
        stamp = self.get_clock().now().to_msg()
        t = TransformStamped(header=Header(stamp=stamp, frame_id='map'), child_frame_id=getattr(self, 'base_frame', 'base_link'))
        t.transform.translation.x, t.transform.translation.y = map(float, self.pose[:2])
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.pose[2] / 2), math.cos(self.pose[2] / 2)
        self.tf.sendTransform(t)
        odom = Odometry(header=t.header, child_frame_id=getattr(self, 'base_frame', 'base_link'))
        odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, self.pose[:2])
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist = copy.deepcopy(cmd)
        odom.twist.twist.angular.z = yaw_rate
        self.odometry.publish(odom)
        self.state.publish(String(data='state=NORMAL;pose_trusted=' + str(self.trusted).lower()))
        self.frame_tick += 1
        if self.frame_tick % 3 == 0 and self.observations:
            # Floor observations in all azimuths, plus an optional small box.
            theta = np.linspace(-math.pi, math.pi, 144, endpoint=False)
            points = np.column_stack((3 * np.cos(theta), 3 * np.sin(theta), np.zeros(len(theta))))
            if self.blocker is not None:
                bx, by = self.blocker
                world = np.array([[bx + dx, by + dy, .15] for dx in (-.05, 0, .05)
                                  for dy in (-.05, 0, .05)])
                world[:, :2] -= self.pose[:2]
                cy, sy = math.cos(self.pose[2]), math.sin(self.pose[2])
                world[:, :2] = world[:, :2] @ np.array([[cy, -sy], [sy, cy]])
                points = np.vstack((points, world))
            frame = FrontendFrame()
            frame.odometry = odom
            frame.lidar_updated = True
            frame.cloud = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id=getattr(self, 'base_frame', 'base_link')), points)
            self.frames.publish(frame)
        sample = dict(time=now, pose=self.pose.tolist(),
                      cmd=[cmd.linear.x, cmd.linear.y, cmd.angular.z], phase=self.phase,
                      blocker=self.blocker)
        self.samples.append(sample)
        self.trace.write(json.dumps(sample) + '\n')

    def wait(self, predicate, timeout=30., message='condition timed out'):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)
            if predicate():
                return
        raise AssertionError(message + f'; pose={self.pose}, phase={self.phase}')

    def run_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)

    def pose_msg(self, pose):
        msg = PoseStamped(header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'))
        msg.pose.position.x, msg.pose.position.y = float(pose[0]), float(pose[1])
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        return msg

    def start(self, goal, through=False):
        client = self.multi if through else self.single
        self.wait(lambda: client.server_is_ready(), 15., 'action server unavailable')
        request = NavigateThroughPoses.Goal() if through else NavigateToPose.Goal()
        if through:
            request.poses = [self.pose_msg(p) for p in goal]
        else:
            request.pose = self.pose_msg(goal)
        future = client.send_goal_async(request)
        self.wait(future.done, 10., 'goal not acknowledged')
        handle = future.result()
        assert handle is not None and handle.accepted, 'goal rejected'
        return handle, handle.get_result_async()

    def finish(self, future, goal, timeout=45.):
        self.wait(future.done, timeout, 'navigation did not finish')
        result = future.result()
        assert result.status == GoalStatus.STATUS_SUCCEEDED, str(result)
        assert np.linalg.norm(self.pose[:2] - goal[:2]) <= .18, (self.pose, goal)
        assert abs(angle(self.pose[2] - goal[2])) <= .32, (self.pose, goal)

    def goal(self, target, timeout=45.):
        handle, future = self.start(target)
        self.finish(future, target, timeout)
        return handle

    def stopped(self):
        now = time.monotonic()
        recent = [s for s in self.samples if s['time'] >= now - .25]
        return len(recent) >= 5 and max(np.linalg.norm(s['cmd']) for s in recent) < 1e-6

    def cancel(self, handle, future):
        requested = time.monotonic()
        initial = [self.command.linear.x, self.command.linear.y, self.command.angular.z]
        cancel = handle.cancel_goal_async()
        self.wait(cancel.done, 5., 'cancel not acknowledged')
        self.wait(future.done, 5., 'canceled navigation still running')
        assert future.result().status == GoalStatus.STATUS_CANCELED
        # Production smoothing brakes at 2 m/s² and 4 rad/s². Check the
        # observed zero-command window within that bound instead of sampling
        # one fixed instant while the timer/DDS callbacks may be catching up.
        settle = max(.8, max(abs(initial[0]) / 2., abs(initial[1]) / 2., abs(initial[2]) / 4.) + .35)
        self.wait(self.stopped, settle, 'cancel did not reach a stable zero output window')
        print(f'CANCEL_STOP initial={initial} elapsed={time.monotonic()-requested:.3f}s', flush=True)
