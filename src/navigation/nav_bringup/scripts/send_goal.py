#!/usr/bin/env python3
"""Send a map-frame navigation goal, or bridge fresh chassis targets to Nav2."""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener


class GoalSender(Node):
    def __init__(self):
        super().__init__('send_navigation_goal')
        self.base_frame = self.declare_parameter('robot_base_frame', 'gimbal_nav').value
        self.require_trusted = self.declare_parameter('require_trusted', True).value
        self.mapping = self.declare_parameter('mapping', False).value
        self.trusted = False
        self.state_received_at = -math.inf
        self.state_sub = self.create_subscription(
            String, '/backend_node/localization_state', self.on_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Wait for the static costmap source and the first integrated local costmap,
        # so the planner does not reject the start cell as unknown/occupied right after bringup.
        self.nav_map_received = False
        self.costmap_received = False
        self.observation_stamp = None
        if self.mapping:
            self.source_sub = self.create_subscription(Header, '/nav/observation', self.on_observation, 5)
        else:
            self.source_sub = self.create_subscription(
                OccupancyGrid, '/nav_map', self.on_nav_map,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, ('/global_costmap/costmap' if self.mapping else '/local_costmap/costmap'), self.on_costmap,
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
        self.state_future = None
        self.next_state_query = 0.0
        self.active_until = 0.0

    def on_state(self, message):
        fields = dict(part.strip().split('=', 1) for part in message.data.split(';') if '=' in part)
        self.trusted = any(key.strip() == 'pose_trusted' and value.strip().lower() == 'true'
                           for key, value in fields.items())
        self.state_received_at = time.monotonic()

    def on_nav_map(self, message):
        self.nav_map_received = True

    def on_observation(self, message):
        self.observation_stamp = Time.from_msg(message.stamp)

    def on_costmap(self, message):
        self.costmap_received = True

    def wait_reason(self, now):
        if self.state_future is not None and self.state_future.done():
            response = self.state_future.result()
            self.active_until = (now + 2.0 if response.current_state.id == State.PRIMARY_STATE_ACTIVE else 0.0)
            self.state_future = None
        elif self.state_future is not None and now >= self.next_state_query + 1.5:
            self.lifecycle.remove_pending_request(self.state_future)
            self.state_future = None
        if not self.lifecycle.service_is_ready():
            self.active_until = 0.0
        elif self.state_future is None and now >= self.next_state_query:
            self.state_future = self.lifecycle.call_async(GetState.Request())
            self.next_state_query = now + 0.5
        if now >= self.active_until or not self.action.server_is_ready():
            return 'active /bt_navigator and /navigate_to_pose'
        if self.require_trusted and (not self.trusted or now - self.state_received_at > 2.0):
            return 'fresh pose_trusted=true localization state'
        if self.mapping and (self.observation_stamp is None or not (
                0 <= (self.get_clock().now() - self.observation_stamp).nanoseconds <= 500_000_000)):
            return 'fresh /nav/observation from live LiDAR'
        if not self.mapping and not self.nav_map_received:
            return '/nav_map (static costmap source not yet published)'
        if not self.costmap_received:
            return 'first costmap update'
        if self.get_clock().now().nanoseconds == 0:
            return 'simulation /clock'
        if not self.tf_buffer.can_transform('map', self.base_frame, Time()):
            return f'map -> {self.base_frame} TF'
        return None

    def wait_ready(self, timeout):
        deadline = time.monotonic() + timeout
        reason = 'Nav2 lifecycle service'
        self.get_logger().info(f'Waiting for active Nav2, localization and map -> {self.base_frame} TF')
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            reason = self.wait_reason(time.monotonic())
            if reason is None:
                return True
        self.get_logger().error(f'Readiness timed out after {timeout:g}s; waiting for {reason}; no goal sent')
        return False

    def make_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def send(self, x, y, yaw):
        goal = self.make_goal(x, y, yaw)
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
        pending = self.send_future
        if pending is not None:
            rclpy.spin_until_future_complete(self, pending, timeout_sec=2.0)
            if pending.done() and pending.result().accepted:
                self.goal_handle = pending.result()
        if self.goal_handle is not None and self.goal_handle.accepted:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result().goals_canceling:
                self.get_logger().info('Navigation goal cancellation accepted')
            else:
                self.get_logger().warning('Goal cancellation not confirmed; check Nav2 goal state')


class SerialGoalSender(GoalSender):
    def __init__(self):
        super().__init__()
        topic = self.declare_parameter('serial_target_topic', '/chassis/nav_target').value
        self.pending_target = None
        self.last_submitted_target = None
        self.target_sub = self.create_subscription(PointStamped, topic, self.on_target, 1)
        self.timer = self.create_timer(0.1, self.forward_target, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Serial goals enabled: {topic} -> /navigate_to_pose')

    def on_target(self, message):
        position = (message.point.x, message.point.y)
        if message.header.frame_id != 'map' or not all(map(math.isfinite, position)):
            return
        stamp = Time.from_msg(message.header.stamp)
        if self.fresh(stamp):
            self.pending_target = (position, stamp)

    def fresh(self, stamp):
        return stamp.nanoseconds > 0 and 0 <= (self.get_clock().now() - stamp).nanoseconds <= 500_000_000

    def forward_target(self):
        if self.pending_target is None or self.send_future is not None:
            return
        position, stamp = self.pending_target
        if not self.fresh(stamp) or (self.last_submitted_target is not None and
                                     math.dist(position, self.last_submitted_target) <= 0.01):
            self.pending_target = None
            return
        if self.wait_reason(time.monotonic()) is not None:
            return
        goal = self.make_goal(*position, yaw=0.0)
        self.pending_target = None
        self.last_submitted_target = position
        self.get_logger().info(f'Sending serial goal: map x={position[0]:g}m, y={position[1]:g}m')
        self.send_future = self.action.send_goal_async(goal)
        self.send_future.add_done_callback(lambda future: self.on_response(future, goal.pose))

    def on_response(self, future, pose):
        self.send_future = None
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Serial navigation goal rejected; waiting for a different target')
            return
        self.goal_handle = handle
        self.goal_display.publish(pose)
        self.get_logger().info('Serial navigation goal accepted')
        handle.get_result_async().add_done_callback(lambda result: self.on_result(result, handle))

    def on_result(self, future, handle):
        if handle is self.goal_handle:
            self.goal_handle = None
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Serial navigation goal succeeded')
        else:
            self.get_logger().info(
                f'Serial navigation ended: status={result.status}, error_code={result.result.error_code}, '
                f'error_msg={result.result.error_msg}')


def main(argv=None):
    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float, nargs='?', help='goal x in map, metres')
    parser.add_argument('y', type=float, nargs='?', help='goal y in map, metres')
    parser.add_argument('--listen-serial', action='store_true', help='forward chassis targets to Nav2')
    parser.add_argument('--yaw', type=float, default=0.0, help='heading in radians, default 0 faces map +X')
    parser.add_argument('--wait-timeout', type=float, default=120.0, help='readiness timeout in seconds')
    args = parser.parse_args(remove_ros_args(argv)[1:])
    if args.listen_serial:
        if args.x is not None or args.y is not None:
            parser.error('--listen-serial cannot be combined with goal coordinates')
    else:
        if args.x is None or args.y is None:
            parser.error('goal x and y are required unless --listen-serial is used')
        if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw, args.wait_timeout)):
            parser.error('coordinates, yaw and wait timeout must be finite')
        if args.wait_timeout <= 0:
            parser.error('--wait-timeout must be positive')
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = SerialGoalSender() if args.listen_serial else GoalSender()
    try:
        if args.listen_serial:
            rclpy.spin(node)
            return 0
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
