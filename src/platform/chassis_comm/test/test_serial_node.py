"""Exercise A3 navigation and the actual TF subscription over a Linux PTY."""

import pytest

def test_ros_transmitter_over_pseudoterminal(link, monkeypatch):
    startup = link.run(.08)
    assert startup == []  # No A3: MCU stays offline instead of spinning in place.
    missing_pose = link.run(.08, velocity=True)
    assert missing_pose == []
    active = link.run(.25, velocity=True, pose=True)
    assert len(active) >= 10
    command = active[-1]
    assert (command.vx, command.vy, command.yaw, command.x, command.y) == pytest.approx(
        (.5, -.25, 0., 2., -3.))  # World TF yaw=.75 must not become a relative turn.
    assert command.is_through_hole == 0
    # True remains on every frame until explicitly cleared; angular.z is not yaw.
    high = link.run(.15, velocity=True, pose=True, hole=True)
    assert all(c.is_through_hole == 1 for c in high[-8:])
    assert all(c.yaw == 0 for c in high[-8:])
    low = link.run(.08, velocity=True, pose=True, hole=False)
    assert all(c.is_through_hole == 0 for c in low[-3:])
    link.run(.3, pose=True)
    assert link.run(.08, pose=True) == []  # No valid A3 to refresh the MCU watchdog.
    link.run(.1, velocity=True, pose=True)
    link.run(.3, velocity=True)
    assert link.run(.08, velocity=True) == []
    link.run(.1, velocity=True, pose=True)
    invalid = link.run(.08, velocity=True, pose=True, omega=float('nan'))
    assert len(invalid) <= 1  # At most an already queued frame before the callback.
    assert link.run(.08, velocity=True, pose=True, omega=float('nan')) == []
    link.run(.1, velocity=True, pose=True)
    count = len(link.commands)
    link.node.close()
    link.drain()
    assert len(link.commands) == count  # Shutdown must not send a spin request.
    # Partial writes terminate the connection instead of reconnecting/replaying.
    link.node._serial.open()
    monkeypatch.setattr(link.node._serial, 'write', lambda data: 5)
    with pytest.raises(RuntimeError, match='partial chassis frame write'):
        link.node._tick()
    assert link.node._failed and not link.node._serial.is_open


def test_repeated_lookup_of_old_tf_cannot_refresh_pose_watchdog(link):
    link.run(.2, velocity=True, pose=True)
    assert link.commands[-1].vx == .5
    count = len(link.commands)
    link.run(.3, velocity=True)
    assert 0 < len(link.commands) - count < 20
    assert link.run(.1, velocity=True) == []


def test_explicit_relative_turn_sign_expiry_and_invalid_input(link):
    positive = link.run(.15, velocity=True, pose=True, turn=1.5707963)
    assert all(c.yaw == pytest.approx(1.5707963) for c in positive[-5:])
    negative = link.run(.1, velocity=True, pose=True, turn=-1.5707963)
    assert all(c.yaw == pytest.approx(-1.5707963) for c in negative[-5:])
    held = link.run(.2, velocity=True, pose=True)
    assert all(c.yaw == 0 and c.vx == .5 for c in held[-5:])
    link.run(.05, velocity=True, pose=True, turn=1.)
    invalid = link.run(.08, velocity=True, pose=True, turn=float('nan'))
    assert all(c.yaw == 0 and c.vx == .5 for c in invalid[-3:])
