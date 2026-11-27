"""A3 navigation TX and A4 encoder/target RX on /dev/nav by default."""

import math
import time

from geometry_msgs.msg import PointStamped, Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rclpy.utilities import try_shutdown
import serial
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformException, TransformListener

from .navigation_state import NavigationState
from .protocol import FRAME_SIZE, FrameParser, encode_frame


class ChassisSerial(Node):
    def __init__(self):
        super().__init__('chassis_serial')
        port = self.declare_parameter('port', '/dev/nav').value
        baud = self.declare_parameter('baud_rate', 115200).value
        rate = self.declare_parameter('rate', 100.0).value
        command_timeout = self.declare_parameter('command_timeout', 0.3).value
        # map -> lio_odom arrives at LiDAR rate (10 Hz); 0.3 s left three frames of
        # margin and a single late frame paused TX and hard-stopped the MCU.
        self._pose_timeout = self.declare_parameter('pose_timeout', 0.45).value
        yaw_timeout = self.declare_parameter('yaw_timeout', 0.3).value
        write_timeout = self.declare_parameter('write_timeout', 0.005).value
        self._feedback_timeout = self.declare_parameter('feedback_timeout', 0.1).value
        self._world_frame = self.declare_parameter('world_frame', 'map').value
        self._base_frame = self.declare_parameter('base_frame', 'gimbal_nav').value
        if self.get_parameter('use_sim_time').value:
            raise ValueError('chassis_serial requires use_sim_time=false')
        if not port or baud <= 0:
            raise ValueError('serial port must be nonempty and baud_rate positive')
        if not all(math.isfinite(v) and v > 0 for v in
                   (rate, command_timeout, self._pose_timeout, yaw_timeout,
                    write_timeout, self._feedback_timeout)):
            raise ValueError('rate and timeouts must be finite and positive')
        if not all((self._world_frame, self._base_frame)):
            raise ValueError('frame names must be nonempty')
        if baud < FRAME_SIZE * 10 * rate:
            raise ValueError('baud_rate cannot carry 24-byte A3 8N1 frames at the requested rate')
        self._state = NavigationState(command_timeout, self._pose_timeout, yaw_timeout)
        self._failed = False
        self._last_active = None
        self._parser = FrameParser(headers=(0xA4,))
        self._last_feedback_stamp = None
        self._last_rx_byte_stamp = None
        self._feedback_online = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._target_pub = self.create_publisher(
            PointStamped, self.declare_parameter('target_topic', '/chassis/nav_target').value, 20)
        self._big_yaw_pub = self.create_publisher(
            Float32, self.declare_parameter('big_yaw_topic', '/chassis/big_yaw').value, 20)
        self._online_pub = self.create_publisher(
            Bool, self.declare_parameter('big_yaw_online_topic', '/chassis/big_yaw_online').value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._set_feedback_online(False)
        qos = QoSProfile(depth=1)  # Volatile: do not replay retained control inputs.
        self._cmd_sub = self.create_subscription(
            Twist, self.declare_parameter('cmd_vel_topic', '/cmd_vel').value, self._velocity, qos)
        self._yaw_sub = self.create_subscription(
            Float32, self.declare_parameter('yaw_topic', '/chassis/yaw_target').value,
            self._yaw, qos)
        self._hole_sub = self.create_subscription(
            Bool, self.declare_parameter('action_topic', '/chassis/pass_hole').value,
            lambda msg: self._state.update_hole(msg.data), qos)
        # Full duplex, 8N1, no flow control, nonblocking RX; one owner per port.
        self._serial = serial.Serial(
            port, baudrate=baud, timeout=0, write_timeout=write_timeout, exclusive=True)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(1.0 / rate, self._tick, clock=steady)
        self._rx_timer = self.create_timer(0.005, self._receive, clock=steady)
        self.get_logger().info(
            f'{port} at {baud} baud (8N1): A3 TX {rate} Hz; 15-byte A4 RX; '
            f'current x/y from {self._world_frame} -> {self._base_frame}; '
            'yaw is an explicit radar-relative turn (default 0); pass_hole is a level.')

    def _set_feedback_online(self, online):
        if online != self._feedback_online:
            self._feedback_online = online
            self._online_pub.publish(Bool(data=online))

    def _receive(self):
        if self._failed:
            return
        stamp = time.monotonic()
        if (self._last_rx_byte_stamp is not None
                and stamp - self._last_rx_byte_stamp > self._feedback_timeout):
            self._parser.reset()
            self._last_rx_byte_stamp = None
        try:
            data = self._serial.read(min(self._serial.in_waiting, 4096))
        except (serial.SerialException, OSError) as exc:
            self._fail()
            raise RuntimeError(f'chassis serial read failed: {exc}') from exc
        if data:
            self._last_rx_byte_stamp = stamp
            for message in self._parser.feed(data):
                self._last_feedback_stamp = stamp
                self._set_feedback_online(True)
                self._big_yaw_pub.publish(Float32(data=message.yaw))
                target = PointStamped()
                target.header.stamp = self.get_clock().now().to_msg()
                target.header.frame_id = self._world_frame
                target.point.x, target.point.y = message.x, message.y
                self._target_pub.publish(target)
        online = (self._last_feedback_stamp is not None
                  and stamp - self._last_feedback_stamp <= self._feedback_timeout)
        self._set_feedback_online(online)

    def _velocity(self, msg: Twist):
        try:
            if not math.isfinite(msg.angular.z):
                raise ValueError('angular.z must be finite')
            self._state.update_velocity(msg.linear.x, msg.linear.y, time.monotonic())
            if abs(msg.angular.z) > 1e-6:
                self.get_logger().warning(
                    'Ignoring angular.z (rad/s): use yaw_topic for a radar-relative turn (rad).',
                    throttle_duration_sec=2.0)
        except ValueError as exc:
            self._state.invalidate_velocity()
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)

    def _yaw(self, msg: Float32):
        try:
            self._state.update_yaw(msg.data, time.monotonic())
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)

    def _update_pose(self, stamp):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._world_frame, self._base_frame, Time())
            age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
            if not 0.0 <= age <= self._pose_timeout:
                raise ValueError('localization transform is stale or future-dated')
            p = transform.transform.translation
            self._state.update_position(p.x, p.y, stamp - age)
        except (TransformException, ValueError):
            self._state.pose_stamp = None

    def _write(self, command):
        frame = encode_frame(command)
        if self._serial.write(frame) != len(frame):
            raise serial.SerialTimeoutException('partial chassis frame write')

    def _fail(self):
        self._failed = True
        self._set_feedback_online(False)
        self._serial.close()

    def _tick(self):
        if self._failed:
            return
        stamp = time.monotonic()
        self._update_pose(stamp)
        active = self._state.ready(stamp)
        if active != self._last_active:
            if not active:
                self.get_logger().warning(
                    'Velocity or localization missing/stale: A3 TX paused; '
                    'MCU stops autonomous motion after its 100 ms offline timeout.')
            self._last_active = active
        if not active:
            # A zero-speed frame with flag=0 still requests the chassis spin.
            return
        try:
            self._write(self._state.command(stamp))
        except (serial.SerialException, OSError) as exc:
            self._fail()
            raise RuntimeError(f'chassis serial write failed: {exc}') from exc

    def close(self):
        self._timer.cancel()
        self._rx_timer.cancel()
        if rclpy.ok(context=self.context):
            self._set_feedback_online(False)
        # Let the MCU's 100 ms A3 watchdog stop autonomous motion.
        self._serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ChassisSerial()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        try_shutdown()
