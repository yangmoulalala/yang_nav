"""Transmit relative body yaw targets, with an optional Twist lookahead adapter."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.utilities import try_shutdown
import serial
from std_msgs.msg import Bool, Float32

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
        self._yaw_mode = self.declare_parameter('yaw_mode', 'twist').value
        self._yaw_horizon = self.declare_parameter('yaw_horizon', 0.1).value
        write_timeout = self.declare_parameter('write_timeout', 0.005).value
        if self.get_parameter('use_sim_time').value:
            raise ValueError('chassis_serial requires use_sim_time=false')
        if not port or baud <= 0:
            raise ValueError('set the actual serial port and baud_rate explicitly')
        if not all(math.isfinite(v) and v > 0 for v in
                   (rate, command_timeout, yaw_timeout, write_timeout)):
            raise ValueError('rate and timeouts must be finite and positive')
        if baud < 17 * 10 * rate:
            raise ValueError('baud_rate cannot carry 17-byte 8N1 frames at the requested rate')
        if self._yaw_mode not in ('relative_angle', 'twist'):
            raise ValueError('yaw_mode must be relative_angle or twist')
        if self._yaw_mode == 'twist':
            twist_command(0.0, 0.0, 0.0, self._yaw_horizon)
        # 8N1, no flow control. Port/baud are not specified by the wire protocol.
        self._serial = serial.Serial(
            port, baudrate=baud, timeout=0, write_timeout=write_timeout, exclusive=True)
        self._state = CommandState(command_timeout, yaw_timeout)
        self._failed = False
        self._last_active = None
        qos = QoSProfile(depth=1)  # reliable, volatile: never replay a latched action
        self._cmd_sub = self.create_subscription(
            Twist, self.declare_parameter('cmd_vel_topic', '/cmd_vel').value,
            self._velocity, qos)
        yaw_topic = self.declare_parameter('yaw_topic', '/chassis/yaw_target').value
        self._yaw_sub = None
        if self._yaw_mode == 'relative_angle':
            self._yaw_sub = self.create_subscription(Float32, yaw_topic, self._yaw, qos)
        self._action_sub = self.create_subscription(
            Bool, self.declare_parameter('action_topic', '/chassis/pass_hole').value,
            lambda msg: self._state.update_action(msg.data, time.monotonic()), qos)
        self._timer = self.create_timer(
            1.0 / rate, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        yaw_info = (
            f'yaw=angular.z * {self._yaw_horizon} s; tune against MCU angle response'
            if self._yaw_mode == 'twist' else 'yaw from explicit body-relative angle input')
        self.get_logger().info(f'{port} at {baud} baud (8N1); {yaw_info}.')

    def _velocity(self, msg: Twist):
        try:
            stamp = time.monotonic()
            if self._yaw_mode == 'twist':
                command = twist_command(
                    msg.linear.x, msg.linear.y, msg.angular.z, self._yaw_horizon)
                self._state.update_velocity(command.vx, command.vy, stamp)
                self._state.update_yaw(command.yaw, stamp)
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
            self._serial.close()
            # Do not reconnect and replay a possibly delivered action.
            raise RuntimeError(f'chassis serial write failed: {exc}') from exc

    def close(self):
        self._timer.cancel()
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
