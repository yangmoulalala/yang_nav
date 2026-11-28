#!/usr/bin/env python3
"""Send a map-frame navigation goal, bridge chassis targets, or map chassis intents to landmarks."""

import argparse
import json
import math
from pathlib import Path
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
from std_msgs.msg import Bool, Header, String
from tf2_ros import Buffer, TransformException, TransformListener


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


def load_landmarks(path):
    """Named map-frame points from a terrain.json export or a flat {name: [x, y]} file.

    A4 only carries an intent name and a chassis-frame radar angle, so a landmark name is the
    only thing that can become a map goal. Nothing here falls back to A4 x/y or the map origin.
    """
    if not path:
        return {}
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError('landmark file must contain a JSON object')
    frame = data.get('frame') if 'landmarks' in data else data.get('frame', 'map')
    if frame != 'map':
        raise ValueError(f'landmark file frame must be map, not {frame}')
    raw = data.get('landmarks', data)
    if not isinstance(raw, dict):
        raise ValueError('landmarks must be an object of name -> [x, y]')
    points = {}
    for name, value in raw.items():
        if not isinstance(value, list) or len(value) != 2 or any(type(part) not in (int, float) for part in value):
            raise ValueError(f'landmark {name!r} must contain two numeric map coordinates')
        x, y = map(float, value)
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f'landmark {name!r} must be finite')
        points[str(name).strip().lower()] = (x, y)
    return points


class IntentGoalSender(GoalSender):
    """Navigate to the selected A4 landmark and return if pushed out after success."""

    def __init__(self, landmarks_file=''):
        super().__init__()
        self.intent_topic = self.declare_parameter('intent_topic', '/chassis/nav_intent').value
        self.landmarks_file = self.declare_parameter('landmarks_file', landmarks_file or '').value
        self.intent_timeout = self.declare_parameter('intent_timeout', 0.5).value
        self.xy_goal_tolerance = self.declare_parameter('xy_goal_tolerance', 0.15).value
        if not math.isfinite(self.xy_goal_tolerance) or self.xy_goal_tolerance <= 0:
            raise ValueError('xy_goal_tolerance must be positive and finite')
        try:
            self.landmarks = load_landmarks(self.landmarks_file)
        except (OSError, ValueError, TypeError) as exc:
            # Stay alive so each rejected intent is visible instead of the bridge dying at startup.
            self.landmarks = {}
            self.get_logger().error(f'cannot read landmarks_file {self.landmarks_file!r}: {exc}')
        self.enabled_topic = self.declare_parameter(
            'enabled_topic', '/navigation/serial_intents_enabled').value
        self.enabled = bool(self.declare_parameter('enabled', True).value)
        self.switch_generation = 0
        self.pending_intent = None
        self.pending_since = 0.0
        self.last_submitted_intent = None
        self.last_intent_succeeded = False
        self.missing_intent = None
        self.sub = self.create_subscription(String, self.intent_topic, self.on_intent, 10)
        # The WebUI owns the runtime switch; retained so a late bridge still sees the operator's choice.
        self.enabled_sub = self.create_subscription(
            Bool, self.enabled_topic, self.on_enabled,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(0.1, self.forward_intent,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'Intent goals enabled: {self.intent_topic} -> map landmarks '
            f'({", ".join(sorted(self.landmarks)) or "none"}) from {self.landmarks_file or "no file"}')

    def on_intent(self, message):
        name = (message.data or '').strip().lower()
        if not self.enabled:
            return
        self.pending_intent = name or None
        self.pending_since = time.monotonic()

    def on_enabled(self, message):
        enabled = bool(message.data)
        if enabled == self.enabled:
            return
        self.enabled = enabled
        self.switch_generation += 1
        self.pending_intent = None
        # Re-arm so the intent held by the chassis still becomes a goal after an operator toggle.
        self.last_submitted_intent = None
        self.last_intent_succeeded = False
        if not enabled and self.goal_handle is not None and self.goal_handle.accepted:
            self.goal_handle.cancel_goal_async()
        self.get_logger().info(f'Serial intent goals {"enabled" if enabled else "disabled"} by {self.enabled_topic}')

    def outside_goal_tolerance(self, x, y):
        try:
            transform = self.tf_buffer.lookup_transform('map', self.base_frame, Time())
        except TransformException:
            return False
        age = self.get_clock().now() - Time.from_msg(transform.header.stamp)
        if not 0 <= age.nanoseconds <= 500_000_000:
            return False
        position = transform.transform.translation
        distance = math.hypot(position.x - x, position.y - y)
        return math.isfinite(distance) and distance > self.xy_goal_tolerance

    def forward_intent(self):
        if not self.enabled:
            return
        now = time.monotonic()
        if self.pending_intent is None:
            return
        name = self.pending_intent
        if now - self.pending_since > self.intent_timeout:
            self.pending_intent = None  # Stale intent is not a goal request.
            return
        same_intent = name == self.last_submitted_intent
        if same_intent and not self.last_intent_succeeded:
            self.pending_intent = None
            return
        if name not in ('home', 'target') or name not in self.landmarks:
            self.pending_intent = None
            if self.missing_intent != name:
                self.missing_intent = name
                self.get_logger().error(
                    f'Intent {name!r} requires home or target with a map landmark in '
                    f'{self.landmarks_file or "no file"}; no goal sent.')
            return
        if self.send_future is not None:
            return
        if self.wait_reason(now) is not None:
            return
        x, y = self.landmarks[name]
        if same_intent and not self.outside_goal_tolerance(x, y):
            return
        goal = self.make_goal(x, y, yaw=0.0)
        self.pending_intent = None
        self.missing_intent = None
        self.last_submitted_intent = name
        self.last_intent_succeeded = False
        self.get_logger().info(f'Sending intent goal {name!r}: map x={x:g}m, y={y:g}m')
        self.send_future = self.action.send_goal_async(goal)
        generation = self.switch_generation
        self.send_future.add_done_callback(lambda future: self.on_response(future, name, goal.pose, generation))

    def on_response(self, future, name, pose, generation):
        self.send_future = None
        handle = future.result()
        if generation != self.switch_generation:
            if handle.accepted:
                handle.cancel_goal_async()
                self.get_logger().info(f'Cancelling intent goal {name!r} acknowledged after switch change')
            return
        if not handle.accepted:
            # Allow the same intent to retry once Nav2 is actually ready again.
            if self.last_submitted_intent == name:
                self.last_submitted_intent = None
            self.get_logger().error(f'Intent goal {name!r} rejected; waiting for Nav2 readiness')
            return
        self.goal_handle = handle
        self.goal_display.publish(pose)
        self.get_logger().info(f'Intent goal {name!r} accepted')
        handle.get_result_async().add_done_callback(lambda result: self.on_result(result, handle, name, generation))

    def on_result(self, future, handle, name, generation):
        result = future.result()
        succeeded = result.status == GoalStatus.STATUS_SUCCEEDED
        if handle is self.goal_handle:
            self.goal_handle = None
            if generation == self.switch_generation and name == self.last_submitted_intent:
                self.last_intent_succeeded = succeeded
        if succeeded:
            self.get_logger().info('Intent goal succeeded')
        else:
            self.get_logger().info(
                f'Intent goal ended: status={result.status}, error_code={result.result.error_code}, '
                f'error_msg={result.result.error_msg}')


def main(argv=None):
    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x', type=float, nargs='?', help='goal x in map, metres')
    parser.add_argument('y', type=float, nargs='?', help='goal y in map, metres')
    parser.add_argument('--listen-serial', action='store_true', help='forward chassis targets to Nav2')
    parser.add_argument('--listen-intents', action='store_true',
                        help='map A4 home/target intents to map landmarks as NavigateToPose goals')
    parser.add_argument('--landmarks-file', default='',
                        help='terrain.json (or {name: [x, y]}) holding the home/target map points')
    parser.add_argument('--yaw', type=float, default=0.0, help='heading in radians, default 0 faces map +X')
    parser.add_argument('--wait-timeout', type=float, default=120.0, help='readiness timeout in seconds')
    args = parser.parse_args(remove_ros_args(argv)[1:])
    listening = args.listen_serial or args.listen_intents
    if args.listen_serial and args.listen_intents:
        parser.error('--listen-serial and --listen-intents cannot be combined')
    if listening:
        if args.x is not None or args.y is not None:
            parser.error('listening modes cannot be combined with goal coordinates')
    else:
        if args.x is None or args.y is None:
            parser.error('goal x and y are required unless a listening mode is used')
        if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw, args.wait_timeout)):
            parser.error('coordinates, yaw and wait timeout must be finite')
        if args.wait_timeout <= 0:
            parser.error('--wait-timeout must be positive')
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        if args.listen_intents:
            node = IntentGoalSender(args.landmarks_file)
        elif args.listen_serial:
            node = SerialGoalSender()
        else:
            node = GoalSender()
        if listening:
            rclpy.spin(node)
            return 0
        if not node.wait_ready(args.wait_timeout):
            return 2
        return node.send(args.x, args.y, args.yaw)
    except KeyboardInterrupt:
        if node is not None:
            node.cancel()
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
