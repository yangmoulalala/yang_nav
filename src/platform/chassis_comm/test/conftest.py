"""Real ROS node + Linux PTY fixture. No hardware devices are opened."""

import math
import os
import pty
import time

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from tf2_ros import TransformBroadcaster

from chassis_comm.protocol import FrameParser
from chassis_comm.serial_node import ChassisSerial


@pytest.fixture
def link():
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    prefix = f'/minipc_test_{os.getpid()}'
    world, base = f'world_{os.getpid()}', f'radar_{os.getpid()}'
    args = ['--ros-args', '-r', f'/chassis/tx_velocity:={prefix}/tx_velocity']
    topics = {
        'cmd_vel_topic': '/velocity', 'action_topic': '/hole',
        'intent_topic': '/intent', 'yaw_topic': '/turn',
        'big_yaw_topic': '/angle', 'big_yaw_online_topic': '/online',
    }
    for name, value in {
        'port': os.ttyname(slave), 'baud_rate': 115200,
        'command_timeout': .15, 'pose_timeout': .15, 'feedback_timeout': .08,
        'yaw_timeout': .08,
        'world_frame': world, 'base_frame': base,
        **{name: prefix + suffix for name, suffix in topics.items()},
    }.items():
        args.extend(['-p', f'{name}:={value}'])
    rclpy.init(args=args, domain_id=199)
    node = peer = executor = None
    try:
        node = ChassisSerial()
        peer = Node(f'minipc_peer_{os.getpid()}')
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(peer)

        class Link:
            def __init__(self):
                self.master, self.node, self.peer = master, node, peer
                self.prefix, self.world, self.base = prefix, world, base
                self.commands = []
                self.parser = FrameParser(headers=(0xA3,))
                self.velocity = peer.create_publisher(Twist, prefix + '/velocity', 1)
                self.hole = peer.create_publisher(Bool, prefix + '/hole', 1)
                self.turn = peer.create_publisher(Float32, prefix + '/turn', 1)
                self.tf = TransformBroadcaster(peer)
                self.subscriptions = []

            def drain(self):
                while True:
                    try:
                        data = os.read(master, 4096)
                    except BlockingIOError:
                        return
                    if not data:
                        return
                    self.commands.extend(self.parser.feed(data))

            def spin(self):
                executor.spin_once(timeout_sec=.001)
                self.drain()

            def wait(self, predicate):
                deadline = time.monotonic() + 5.
                while not predicate():
                    assert time.monotonic() < deadline, 'ROS delivery timed out'
                    self.spin()

            def run(self, seconds, *, velocity=False, pose=False, hole=None, omega=.7, turn=None):
                begin = len(self.commands)
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    if velocity:
                        msg = Twist()
                        msg.linear.x, msg.linear.y, msg.angular.z = .5, -.25, omega
                        self.velocity.publish(msg)
                    if pose:
                        msg = TransformStamped()
                        msg.header.stamp = peer.get_clock().now().to_msg()
                        msg.header.frame_id, msg.child_frame_id = world, base
                        msg.transform.translation.x, msg.transform.translation.y = 2., -3.
                        msg.transform.rotation.z = math.sin(.75 / 2)
                        msg.transform.rotation.w = math.cos(.75 / 2)
                        self.tf.sendTransform(msg)
                    if hole is not None:
                        self.hole.publish(Bool(data=hole))
                    if turn is not None:
                        self.turn.publish(Float32(data=turn))
                    self.spin()
                return self.commands[begin:]

            def subscribe(self, cls, suffix, qos=20):
                messages = []
                sub = peer.create_subscription(cls, prefix + suffix, messages.append, qos)
                self.subscriptions.append(sub)
                self.wait(lambda: sub.get_publisher_count() == 1)
                return messages

        connection = Link()
        connection.wait(lambda: connection.velocity.get_subscription_count() == 1
                        and connection.hole.get_subscription_count() == 1
                        and connection.turn.get_subscription_count() == 1)
        yield connection
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.close()
            node.destroy_node()
        if peer is not None:
            peer.destroy_node()
        rclpy.try_shutdown()
        os.close(master)
        os.close(slave)
