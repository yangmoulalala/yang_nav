"""Exercise the full-duplex ROS node on a PTY, including 200 Hz feedback."""

import os
import pty
import struct
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import serial
from std_msgs.msg import Bool, Float32

from chassis_comm.protocol import FrameParser, crc16_modbus
from chassis_comm.serial_node import ChassisSerial


def frame(angle):
    payload = struct.pack('<f', angle)
    return b'\xa5\x5a' + payload + struct.pack('<H', crc16_modbus(payload))


def test_big_yaw_feedback_over_pseudoterminal(monkeypatch):
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    prefix = f'/big_yaw_test_{os.getpid()}'
    args = ['--ros-args']
    for name, value in {
        'port': os.ttyname(slave), 'baud_rate': 115200,
        'cmd_vel_topic': prefix + '/velocity', 'yaw_topic': prefix + '/target',
        'action_topic': prefix + '/action', 'big_yaw_topic': prefix + '/angle',
        'big_yaw_online_topic': prefix + '/online', 'big_yaw_timeout': 0.08,
    }.items():
        args.extend(['-p', f'{name}:={value}'])
    rclpy.init(args=args, domain_id=198)
    node = peer = executor = None
    try:
        node = ChassisSerial()
        peer = Node(f'big_yaw_peer_{os.getpid()}')
        angles, states, commands = [], [], []
        angle_sub = peer.create_subscription(
            Float32, prefix + '/angle', lambda msg: angles.append(msg.data), 20)
        online_sub = peer.create_subscription(
            Bool, prefix + '/online', lambda msg: states.append(msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(peer)
        tx_parser = FrameParser()

        def spin():
            executor.spin_once(timeout_sec=0.001)
            while True:
                try:
                    data = os.read(master, 4096)
                except BlockingIOError:
                    break
                if not data:
                    break
                commands.extend(tx_parser.feed(data))

        def run_for(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                spin()

        def wait_until(predicate):
            deadline = time.monotonic() + 5.0
            while not predicate():
                assert time.monotonic() < deadline, 'ROS feedback delivery timed out'
                spin()

        wait_until(lambda: angle_sub.get_publisher_count() == 1
                   and online_sub.get_publisher_count() == 1 and states)
        assert states == [False]  # retained initial offline state for late subscribers
        assert angles == []

        # Split header and payload, followed by multiple frames in one read.
        os.write(master, b'noise\xa5')
        run_for(0.01)
        os.write(master, frame(1.0)[1:4])
        run_for(0.01)
        assert angles == []
        os.write(master, frame(1.0)[4:] + frame(-0.5) + frame(0.0))
        wait_until(lambda: len(angles) == 3 and states[-1])
        assert angles == [1.0, -0.5, 0.0]

        # Corrupted / invalid-angle traffic never publishes or keeps feedback online.
        bad = bytearray(frame(2.0))
        bad[-1] ^= 0x80
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            os.write(master, bytes(bad) + frame(float('nan')) + frame(4.0))
            run_for(0.005)
        assert angles == [1.0, -0.5, 0.0]
        assert states[-1] is False

        # Deliver exactly 200 measured samples at 5 ms intervals while TX continues.
        begin_count, begin_tx = len(angles), len(commands)
        samples = [-1.0 + i / 200.0 for i in range(200)]
        begin = time.monotonic()
        for index, angle in enumerate(samples):
            while time.monotonic() < begin + index * 0.005:
                spin()
            os.write(master, frame(angle))
        wait_until(lambda: len(angles) == begin_count + len(samples))
        assert angles[begin_count:] == pytest.approx(samples)
        assert len(commands) - begin_tx >= 50
        assert all(c.vx == c.vy == c.yaw == c.action_flag == 0 for c in commands)
        assert states[-1] is True

        # Silence stops angle publication and invalidates an incomplete old frame.
        os.write(master, frame(1.25)[:5])
        run_for(0.16)
        count = len(angles)
        assert states[-1] is False
        os.write(master, frame(1.25)[5:])
        run_for(0.02)
        assert len(angles) == count
        os.write(master, frame(-1.25))
        wait_until(lambda: len(angles) == count + 1 and states[-1])
        assert angles[-1] == -1.25
        run_for(0.16)
        assert len(angles) == count + 1 and states[-1] is False

        # A read error closes the port and follows the existing no-replay policy.
        def failed_read(_size):
            raise serial.SerialException('injected read failure')

        monkeypatch.setattr(node._serial, 'read', failed_read)
        with pytest.raises(RuntimeError, match='chassis serial read failed'):
            node._receive()
        assert node._failed and not node._serial.is_open
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
