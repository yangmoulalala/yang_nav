"""A4 navigation feedback and concurrent A3 TX through real ROS and a PTY."""

import os
import time

import pytest
from geometry_msgs.msg import PointStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
import serial
from std_msgs.msg import Bool, Float32

from chassis_comm.protocol import Command, NavigationTarget, encode_frame


A2 = bytes.fromhex('A2 00 00 00 C8 41 00 00 00 3F 00 00 80 BE 00 00 40 3F 00 00 60 3F D5 DF')
A4 = bytes.fromhex('A4 DB 0F 49 3F 00 00 C0 3F 00 00 20 C0 41 20')


def test_navigation_feedback_over_pseudoterminal(link, monkeypatch):
    angles = link.subscribe(Float32, '/angle')
    targets = link.subscribe(PointStamped, '/target')
    states = link.subscribe(Bool, '/online', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    link.wait(lambda: bool(states))
    assert [msg.data for msg in states] == [False]
    os.write(link.master, b'noise\xaa\x55' + A4[:7])
    link.run(.01)
    assert not angles and not targets
    # Other frame types must be consumed without publishing navigation data.
    os.write(link.master, A4[7:] + A2 + encode_frame(Command(1, 0, 0)) + A4)
    link.wait(lambda: len(angles) == len(targets) == 2)
    assert [msg.data for msg in angles] == pytest.approx([.7853981634] * 2)
    assert targets[0].header.frame_id == link.world
    assert (targets[0].point.x, targets[0].point.y) == (1.5, -2.5)
    assert targets[0].header.stamp.sec > 0
    assert states[-1].data is True

    # Bad CRC and unrelated A2 traffic cannot refresh A4 online state.
    bad = bytearray(A4)
    bad[-1] ^= 0x80
    deadline = time.monotonic() + .13
    while time.monotonic() < deadline:
        os.write(link.master, bytes(bad) + A2)
        link.run(.005)
    link.wait(lambda: states[-1].data is False)
    assert len(angles) == len(targets) == 2

    # Exercise 200 Hz A4 traffic; the document does not prescribe its frequency.
    link.run(.1, velocity=True, pose=True)
    begin_rx, begin_tx = len(angles), len(link.commands)
    start = time.monotonic()
    for index in range(200):
        while time.monotonic() < start + index * .005:
            link.spin()
        os.write(link.master, encode_frame(NavigationTarget(index / 256., float(index), -2.5)))
        link.run(.001, velocity=True, pose=True)
    link.wait(lambda: len(angles) == len(targets) == begin_rx + 200)
    assert [q.data for q in angles[begin_rx:]] == pytest.approx([i / 256. for i in range(200)])
    assert [p.point.x for p in targets[begin_rx:]] == list(range(200))
    assert len(link.commands) - begin_tx >= 50
    assert all(c.yaw == 0 for c in link.commands)  # Encoder yaw is never a turn request.
    assert states[-1].data is True

    os.write(link.master, A4[:9])
    link.run(.14)
    assert states[-1].data is False
    count = len(angles)
    os.write(link.master, A4[9:])
    link.run(.01)
    assert len(angles) == count
    os.write(link.master, A4)
    link.wait(lambda: len(angles) == count + 1 and states[-1].data is True)

    def failed_read(_size):
        raise serial.SerialException('injected read failure')

    monkeypatch.setattr(link.node._serial, 'read', failed_read)
    with pytest.raises(RuntimeError, match='chassis serial read failed'):
        link.node._receive()
    assert link.node._failed and not link.node._serial.is_open
