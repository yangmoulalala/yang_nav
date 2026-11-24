"""Full-duplex chassis commands and measured big-yaw feedback on one port."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.utilities import try_shutdown
import serial
from std_msgs.msg import Bool, Float32

from .big_yaw import BigYawParser
from .command_state import CommandState
from .protocol import encode_frame
from .twist_adapter import twist_command


class ChassisSerial(Node):
    def __init__(self):
        super().__init__('chassis_serial')
        port = self.declare_parameter('port', '').value
        baud = self.declare_parameter('baud_rate', 0).value
        rate = self.declare_parameter('rate', 100.0).value
        command_timeout = self.declare_parameter('command_timeout', 0.3).value
        yaw_timeout = self.declare_parameter('yaw_timeout', 0.3).value
        self._yaw_mode = self.declare_parameter('yaw_mode', 'action_angle').value
        self._yaw_horizon = self.declare_parameter('yaw_horizon', 0.1).value
        write_timeout = self.declare_parameter('write_timeout', 0.005).value
        self._big_yaw_timeout = self.declare_parameter('big_yaw_timeout', 0.1).value
        if self.get_parameter('use_sim_time').value:
            raise ValueError('chassis_serial requires use_sim_time=false')
        if not port or baud <= 0:
            raise ValueError('set the actual serial port and baud_rate explicitly')
        if not all(math.isfinite(v) and v > 0 for v in
                   (rate, command_timeout, yaw_timeout, write_timeout, self._big_yaw_timeout)):
            raise ValueError('rate and timeouts must be finite and positive')
        if baud < 17 * 10 * rate:
            raise ValueError('baud_rate cannot carry 17-byte 8N1 frames at the requested rate')
        if baud < 8 * 10 * 200:
            raise ValueError('baud_rate cannot receive 8-byte 8N1 big-yaw frames at 200 Hz')
        if self._yaw_mode not in ('action_angle', 'relative_angle', 'twist'):
            raise ValueError('yaw_mode must be action_angle, relative_angle or twist')
        if self._yaw_mode == 'twist':
            twist_command(0.0, 0.0, 0.0, self._yaw_horizon)
        # 8N1, no flow control. Port/baud are not specified by the wire protocol.
        self._serial = serial.Serial(
            port, baudrate=baud, timeout=0, write_timeout=write_timeout, exclusive=True)
        self._state = CommandState(command_timeout, yaw_timeout,
                                   action_only_yaw=self._yaw_mode == 'action_angle')
        self._failed = False
        self._last_active = None
        self._big_yaw_parser = BigYawParser()
        self._last_big_yaw_stamp = None
        self._last_rx_byte_stamp = None
        self._big_yaw_online = None
        feedback_topic = self.declare_parameter('big_yaw_topic', '/chassis/big_yaw').value
        self._big_yaw_pub = self.create_publisher(Float32, feedback_topic, QoSProfile(depth=20))
        self._big_yaw_online_pub = self.create_publisher(
            Bool, self.declare_parameter('big_yaw_online_topic', '/chassis/big_yaw_online').value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._set_big_yaw_online(False)
        qos = QoSProfile(depth=1)  # reliable, volatile: never replay a latched action
        self._cmd_sub = self.create_subscription(
            Twist, self.declare_parameter('cmd_vel_topic', '/cmd_vel').value,
            self._velocity, qos)
        yaw_topic = self.declare_parameter('yaw_topic', '/chassis/yaw_target').value
        self._yaw_sub = None
        if self._yaw_mode in ('action_angle', 'relative_angle'):
            self._yaw_sub = self.create_subscription(Float32, yaw_topic, self._yaw, qos)
        self._action_sub = self.create_subscription(
            Bool, self.declare_parameter('action_topic', '/chassis/pass_hole').value,
            lambda msg: self._state.update_action(msg.data, time.monotonic()), qos)
        self._timer = self.create_timer(
            1.0 / rate, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._rx_timer = self.create_timer(
            0.005, self._receive, clock=Clock(clock_type=ClockType.STEADY_TIME))
        yaw_info = (
            f'yaw=angular.z * {self._yaw_horizon} s; tune against MCU angle response'
            if self._yaw_mode == 'twist' else ('yaw from explicit body-frame angle, only on action edge'
                  if self._yaw_mode == 'action_angle' else 'yaw from explicit body-relative angle input'))
        self.get_logger().info(f'{port} at {baud} baud (8N1); {yaw_info}.')
        self.get_logger().info(
            f'Big-yaw RX: 8-byte frames at 200 Hz -> {feedback_topic} (Float32, rad); '
            f'offline after {self._big_yaw_timeout} s without valid feedback.')

    def _set_big_yaw_online(self, online):
        if online != self._big_yaw_online:
            self._big_yaw_online = online
            self._big_yaw_online_pub.publish(Bool(data=online))

    def _receive(self):
        if self._failed:
            return
        stamp = time.monotonic()
        # Do not assemble a frame across a long silence / MCU restart.
        if (self._last_rx_byte_stamp is not None
                and stamp - self._last_rx_byte_stamp > self._big_yaw_timeout):
            self._big_yaw_parser.reset()
            self._last_rx_byte_stamp = None
        try:
            # Nonblocking and bounded: receive traffic must not starve command TX.
            data = self._serial.read(min(self._serial.in_waiting, 4096))
        except (serial.SerialException, OSError) as exc:
            self._failed = True
            self._set_big_yaw_online(False)
            self._serial.close()
            raise RuntimeError(f'chassis serial read failed: {exc}') from exc
        if data:
            self._last_rx_byte_stamp = stamp
            for angle in self._big_yaw_parser.feed(data):
                self._last_big_yaw_stamp = stamp
                self._set_big_yaw_online(True)
                self._big_yaw_pub.publish(Float32(data=angle))
        online = (self._last_big_yaw_stamp is not None
                  and stamp - self._last_big_yaw_stamp <= self._big_yaw_timeout)
        if self._big_yaw_online and not online:
            self.get_logger().warning('Big-yaw feedback stale: angle publication paused.')
        self._set_big_yaw_online(online)

    def _velocity(self, msg: Twist):
        try:
            stamp = time.monotonic()
            if self._yaw_mode == 'twist':
                command = twist_command(
                    msg.linear.x, msg.linear.y, msg.angular.z, self._yaw_horizon)
                self._state.update_velocity(command.vx, command.vy, stamp)
                self._state.update_yaw(command.yaw, stamp)
            elif self._yaw_mode == 'action_angle':
                if not math.isfinite(msg.angular.z):
                    raise ValueError('angular.z must be finite')
                self._state.update_velocity(msg.linear.x, msg.linear.y, stamp)
                if abs(msg.angular.z) > 1e-6:
                    self.get_logger().warning(
                        'Ignoring navigation angular.z: MCU accepts body-frame yaw only for actions.',
                        throttle_duration_sec=2.0)
            else:
                # Do not mix an explicit relative target with an ignored yaw rate.
                if not math.isfinite(msg.angular.z) or abs(msg.angular.z) > 1e-6:
                    raise ValueError(
                        'relative_angle mode requires angular.z=0; use twist mode '
                        'to adapt angular velocity')
                self._state.update_velocity(msg.linear.x, msg.linear.y, stamp)
        except ValueError as exc:
            self._state.invalidate_velocity()
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)

    def _yaw(self, msg: Float32):
        try:
            self._state.update_yaw(msg.data, time.monotonic())
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)

    def _write(self, command):
        frame = encode_frame(command)
        if self._serial.write(frame) != len(frame):
            raise serial.SerialTimeoutException('partial chassis frame write')
        self._state.sent(command)

    def _tick(self):
        if self._failed:
            return
        stamp = time.monotonic()
        command = self._state.command(stamp)
        active = self._state.ready(stamp)
        if active != self._last_active:
            if not active:
                self.get_logger().warning(
                    'Velocity/yaw input missing or stale: sending '
                    'vx=vy=yaw=0, action_flag=0.')
            self._last_active = active
        try:
            self._write(command)
        except (serial.SerialException, OSError) as exc:
            self._failed = True
            self._set_big_yaw_online(False)
            self._serial.close()
            # Do not reconnect and replay a possibly delivered action.
            raise RuntimeError(f'chassis serial write failed: {exc}') from exc

    def close(self):
        self._timer.cancel()
        self._rx_timer.cancel()
        if rclpy.ok(context=self.context):
            self._set_big_yaw_online(False)
        try:
            stop = self._state.stop_command()
            if self._serial.is_open and not self._failed:
                self._write(stop)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'final stop frame failed: {exc}')
        finally:
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
