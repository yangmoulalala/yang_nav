"""Ordinary MCU interface: turret-frame XY, watchdog and measured feedback.

Only the simulator reads the physical joint. Localization receives the same
rigid gimbal_nav subtree as hardware; feedback does not create a chassis TF.
"""
import math
import time

from chassis_comm.command_state import CommandState


class DriveState:
    def __init__(self, yaw_offset=0.0, command_timeout=0.3, feedback_timeout=0.1):
        if not math.isfinite(yaw_offset):
            raise ValueError('gimbal_yaw_offset must be finite')
        if not all(math.isfinite(v) and v > 0 for v in (command_timeout, feedback_timeout)):
            raise ValueError('timeouts must be finite and positive')
        self.commands = CommandState(command_timeout, command_timeout, action_only_yaw=True)
        self.yaw_offset = yaw_offset
        self.feedback_timeout = feedback_timeout
        self.angle = 0.0
        self.feedback_stamp = None

    def feedback(self, joint_angle, stamp):
        if not math.isfinite(joint_angle):
            self.feedback_stamp = None
            return
        # CAD yaw_joint rotates about chassis -Z. Simulator feedback is CCW +Z.
        self.angle = (-joint_angle + math.pi) % (2 * math.pi) - math.pi
        self.feedback_stamp = stamp

    def online(self, stamp):
        return (self.feedback_stamp is not None
                and 0 <= stamp - self.feedback_stamp <= self.feedback_timeout)

    def velocity(self, vx, vy, angular, stamp):
        if not math.isfinite(angular):
            self.commands.invalidate_velocity()
            raise ValueError('angular.z must be finite')
        self.commands.update_velocity(vx, vy, stamp)

    def output(self, stamp):
        if not self.online(stamp):
            return 0.0, 0.0
        command = self.commands.command(stamp)
        angle = self.angle + self.yaw_offset
        c, s = math.cos(angle), math.sin(angle)
        return c * command.vx - s * command.vy, s * command.vx + c * command.vy


def main(args=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float32, Float64

    class ChassisSim(Node):
        def __init__(self):
            super().__init__('chassis_sim')
            self.state = DriveState(
                self.declare_parameter('gimbal_yaw_offset', 0.0).value,
                self.declare_parameter('command_timeout', 0.3).value,
                self.declare_parameter('big_yaw_timeout', 0.1).value)
            self.drive = self.create_publisher(Twist, '/sim/chassis_cmd_vel', 1)
            self.target = self.create_publisher(Float64, '/sim/yaw_joint_target', 1)
            self.angle = self.create_publisher(Float32, '/chassis/big_yaw', 20)
            self.status = self.create_publisher(Bool, '/chassis/big_yaw_online', QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.was_online = False
            self.status.publish(Bool(data=False))
            self.create_subscription(Twist, '/cmd_vel', self.velocity, 1)
            self.create_subscription(JointState, '/sim/joint_states', self.feedback, 10)
            self.create_subscription(Float64, '/sim/turret_yaw', self.turn_turret, 1)
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(0.01, self.tick, clock=steady)
            self.create_timer(0.005, self.publish_feedback, clock=steady)
            self.get_logger().info('MCU simulation: gimbal_nav XY at 100 Hz; angular.z ignored; '
                                   'big-yaw feedback at 200 Hz; no physical serial port.')

        def velocity(self, msg):
            try:
                self.state.velocity(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())
                if abs(msg.angular.z) > 1e-6:
                    self.get_logger().warning('Ignoring navigation angular.z, as on hardware.',
                                              throttle_duration_sec=2.0)
            except ValueError as exc:
                self.get_logger().error(str(exc), throttle_duration_sec=2.0)

        def feedback(self, msg):
            if 'yaw_joint' in msg.name:
                index = msg.name.index('yaw_joint')
                if index < len(msg.position):
                    self.state.feedback(msg.position[index], time.monotonic())

        def turn_turret(self, msg):
            if math.isfinite(msg.data):
                self.target.publish(Float64(data=-msg.data))

        def publish_feedback(self):
            online = self.state.online(time.monotonic())
            if online != self.was_online:
                self.status.publish(Bool(data=online))
                self.was_online = online
            if online:
                self.angle.publish(Float32(data=self.state.angle))

        def tick(self):
            cmd = Twist()
            cmd.linear.x, cmd.linear.y = self.state.output(time.monotonic())
            self.drive.publish(cmd)

    rclpy.init(args=args)
    node = None
    try:
        node = ChassisSim()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.drive.publish(Twist())
            node.destroy_node()
        rclpy.try_shutdown()
