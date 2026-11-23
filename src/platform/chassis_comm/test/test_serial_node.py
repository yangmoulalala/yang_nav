"""Run the actual ROS node against a Linux PTY, never a hardware device."""

import os
import pty
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

from chassis_comm.protocol import FrameParser
from chassis_comm.serial_node import ChassisSerial


@pytest.mark.parametrize('yaw_mode', ['relative_angle', 'twist'])
def test_ros_transmitter_over_pseudoterminal(monkeypatch, yaw_mode):
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    prefix = f'/chassis_test_{os.getpid()}/{yaw_mode}'
    args = ['--ros-args']
    for name, value in {
        'port': os.ttyname(slave), 'baud_rate': 115200,
        'command_timeout': 0.2, 'yaw_timeout': 0.2,
        'cmd_vel_topic': prefix + '/velocity', 'yaw_topic': prefix + '/yaw',
        'action_topic': prefix + '/action',
        **({'yaw_mode': yaw_mode} if yaw_mode == 'relative_angle' else {}),
    }.items():
        args.extend(['-p', f'{name}:={value}'])
    rclpy.init(args=args, domain_id=199)
    node = peer = executor = None
    try:
        node = ChassisSerial()
        peer = Node(f'chassis_test_peer_{os.getpid()}')
        velocity = peer.create_publisher(Twist, prefix + '/velocity', 1)
        yaw = peer.create_publisher(Float32, prefix + '/yaw', 1)
        action = peer.create_publisher(Bool, prefix + '/action', 1)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(peer)
        parser = FrameParser()
        received = []

        def drain():
            while True:
                try:
                    data = os.read(master, 4096)
                except BlockingIOError:
                    return
                if not data:
                    return
                received.extend(parser.feed(data))

        def run_for(seconds, *, send_yaw=True, send_velocity=True, high=False, omega=None):
            start = len(received)
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if send_velocity:
                    msg = Twist()
                    msg.linear.x, msg.linear.y = 0.5, -0.25
                    msg.angular.z = (0.75 if yaw_mode == 'twist' else 0.0) if omega is None else omega
                    velocity.publish(msg)
                if send_yaw:
                    yaw.publish(Float32(data=1.25))
                action.publish(Bool(data=high))
                executor.spin_once(timeout_sec=0.005)
                drain()
            return received[start:]

        deadline = time.monotonic() + 5.0
        while min(velocity.get_subscription_count(), action.get_subscription_count(),
                  yaw.get_subscription_count() if yaw_mode == 'relative_angle' else 1) < 1:
            assert time.monotonic() < deadline, 'ROS subscriptions did not match'
            executor.spin_once(timeout_sec=0.02)
        startup = run_for(0.1, send_yaw=False, send_velocity=False)
        assert startup and all(c.vx == c.vy == c.yaw == c.action_flag == 0 for c in startup)
        without_yaw = run_for(0.2, send_yaw=False)
        if yaw_mode == 'relative_angle':
            assert all(c.vx == c.vy == c.yaw == 0 for c in without_yaw)
        else:
            assert any(c.vx == 0.5 and c.yaw == pytest.approx(0.075) for c in without_yaw)
        active = run_for(0.25)
        expected_yaw = 0.075 if yaw_mode == 'twist' else 1.25
        assert any(c.vx == 0.5 and c.vy == -0.25 and c.yaw == pytest.approx(expected_yaw)
                   for c in active)
        triggered = run_for(0.25, high=True)
        assert sum(c.action_flag for c in triggered) == 1
        assert triggered[-1].action_flag == 0
        run_for(0.1, high=False)
        assert sum(c.action_flag for c in run_for(0.2, high=True)) == 1

        stale = run_for(0.4, send_yaw=False, send_velocity=False, high=True)
        assert len(stale) > 4
        assert all(c.vx == c.vy == c.yaw == c.action_flag == 0
                   for c in stale[-3:])
        resumed = run_for(0.25, high=True)
        assert any(c.vx == 0.5 for c in resumed)
        assert all(c.action_flag == 0 for c in resumed)
        if yaw_mode == 'relative_angle':
            rejected = run_for(0.15, omega=0.7)
            assert all(c.vx == c.vy == c.yaw == c.action_flag == 0 for c in rejected[-3:])
        else:
            clockwise = run_for(0.15, omega=-0.75)
            assert all(c.yaw == pytest.approx(-0.075) for c in clockwise[-3:])
            straight = run_for(0.15, omega=0.0)
            assert all(c.vx == 0.5 and c.yaw == 0 for c in straight[-3:])
        invalid = run_for(0.15, omega=float('nan'))
        assert all(c.vx == c.vy == c.yaw == c.action_flag == 0 for c in invalid[-3:])

        node.close()
        drain()
        assert received[-1].vx == received[-1].vy == received[-1].action_flag == 0
        assert received[-1].yaw == 0
        # A partial write terminates the connection; never commit/retry an action.
        node._serial.open()
        monkeypatch.setattr(node._serial, 'write', lambda data: 5)
        with pytest.raises(RuntimeError, match='partial chassis frame write'):
            node._tick()
        assert not node._serial.is_open
        assert node._failed
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
