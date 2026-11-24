#!/usr/bin/env python3
"""Wait for navigation readiness, send one map-frame goal and report its result."""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class GoalSender(Node):
    def __init__(self):
        super().__init__('send_navigation_goal')
        self.base_frame = self.declare_parameter('robot_base_frame', 'base_link').value
        self.require_trusted = self.declare_parameter('require_trusted', True).value
        self.trusted = False
        self.state_received_at = -math.inf
        self.state_sub = self.create_subscription(
            String, '/backend_node/localization_state', self.on_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Wait for the static costmap source and the first integrated local costmap,
        # so the planner does not reject the start cell as unknown/occupied right after bringup.
        self.nav_map_received = False
        self.costmap_received = False
        self.nav_map_sub = self.create_subscription(
            OccupancyGrid, '/nav_map', self.on_nav_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_display = self.create_publisher(
            PoseStamped, '/navigation/goal_accepted',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.goal_handle = None
        self.send_future = None

    def on_state(self, message):
        fields = dict(part.strip().split('=', 1) for part in message.data.split(';') if '=' in part)
        self.trusted = any(key.strip() == 'pose_trusted' and value.strip().lower() == 'true'
                           for key, value in fields.items())
        self.state_received_at = time.monotonic()

    def on_nav_map(self, message):
        self.nav_map_received = True

    def on_costmap(self, message):
        self.costmap_received = True

    def wait_ready(self, timeout):
        deadline = time.monotonic() + timeout
        state_future = None
        active = False
        next_query = 0.0
        reason = 'Nav2 lifecycle service'
        self.get_logger().info(f'Waiting for active Nav2, localization and map -> {self.base_frame} TF')
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if state_future is not None and state_future.done():
                active = state_future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
                state_future = None
            if state_future is None and now >= next_query and self.lifecycle.service_is_ready():
                state_future = self.lifecycle.call_async(GetState.Request())
                next_query = now + 0.5
            if not active or not self.action.server_is_ready():
                reason = 'active /bt_navigator and /navigate_to_pose'
            elif self.require_trusted and (not self.trusted or now - self.state_received_at > 2.0):
                reason = 'fresh pose_trusted=true localization state'
            elif not self.nav_map_received:
                reason = '/nav_map (static costmap source not yet published)'
            elif not self.costmap_received:
                reason = 'first /local_costmap/costmap update (static layer not yet integrated)'
            elif self.get_clock().now().nanoseconds == 0:
                reason = 'simulation /clock'
            elif not self.tf_buffer.can_transform('map', self.base_frame, Time()):
                reason = f'map -> {self.base_frame} TF'
            else:
                return True
        self.get_logger().error(f'Readiness timed out after {timeout:g}s; waiting for {reason}; no goal sent')
        return False

    def send(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.get_logger().info(f'Sending one goal: map x={x:g}m, y={y:g}m, yaw={yaw:g}rad')
        self.send_future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, self.send_future)
        self.goal_handle = self.send_future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return 3
        # Display accepted Action goals without publishing another Nav2 command.
        self.goal_display.publish(goal.pose)
        self.goal_display.wait_for_all_acked(timeout=Duration(seconds=1.0))
        self.get_logger().info('Navigation goal accepted; Ctrl-C cancels this goal')
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded')
            return 0
        self.get_logger().error(
            f'Navigation ended: status={result.status}, error_code={result.result.error_code}, '
            f'error_msg={result.result.error_msg}')
        return 4

    def cancel(self):
        # Ctrl-C can arrive while the server is still acknowledging the request.
        if self.goal_handle is None and self.send_future is not None:
            rclpy.spin_until_future_complete(self, self.send_future, timeout_sec=2.0)
            if self.send_future.done():
                self.goal_handle = self.send_future.result()
        if self.goal_handle is not None and self.goal_handle.accepted:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result().goals_canceling:
                self.get_logger().info('Navigation goal cancellation accepted')
            else:
                self.get_logger().warning('Goal cancellation not confirmed; check Nav2 goal state')


def main(argv=None):
    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float, help='goal x in map, metres')
    parser.add_argument('y', type=float, help='goal y in map, metres')
    parser.add_argument('--yaw', type=float, default=0.0, help='heading in radians, default 0 faces map +X')
    parser.add_argument('--wait-timeout', type=float, default=120.0, help='readiness timeout in seconds')
    args = parser.parse_args(remove_ros_args(argv)[1:])
    if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw, args.wait_timeout)):
        parser.error('coordinates, yaw and wait timeout must be finite')
    if args.wait_timeout <= 0:
        parser.error('--wait-timeout must be positive')
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = GoalSender()
    try:
        if not node.wait_ready(args.wait_timeout):
            return 2
        return node.send(args.x, args.y, args.yaw)
    except KeyboardInterrupt:
        node.cancel()
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
