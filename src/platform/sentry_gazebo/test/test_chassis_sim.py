import math

import pytest

from sentry_gazebo.chassis_sim import DriveState


@pytest.mark.parametrize('turret', [-2.1, -.6, 0, .7, 2.8])
def test_command_is_executed_in_current_turret_axes(turret):
    state = DriveState(yaw_offset=.23)
    state.feedback(-turret, 10.)
    state.velocity(.25, -.1, 1.5, 10.)  # angular.z has no ordinary MCU actuator
    x, y = state.output(10.05)
    a = turret + .23
    assert (x, y) == pytest.approx((.25 * math.cos(a) + .1 * math.sin(a),
                                   .25 * math.sin(a) - .1 * math.cos(a)))


def test_watchdog_and_feedback_loss_stop_without_replaying_commands():
    state = DriveState()
    state.velocity(.2, .1, 0., 10.)
    assert state.output(10.) == (0., 0.)  # no invented zero-angle feedback
    state.feedback(0., 10.)
    assert state.output(10.05) == (.2, .1)
    assert state.output(10.11) == (0., 0.)  # missing measured angle
    state.feedback(0., 10.4)
    assert state.output(10.4) == (0., 0.)  # velocity also expired
    state.velocity(.2, .1, 0., 10.4)
    assert state.output(10.41) == (.2, .1)
    with pytest.raises(ValueError):
        state.velocity(float('nan'), .1, 0., 10.42)
    assert state.output(10.43) == (0., 0.)


def test_nonfinite_angular_input_invalidates_previous_velocity():
    state = DriveState()
    state.feedback(0., 1.)
    state.velocity(.2, 0., 0., 1.)
    with pytest.raises(ValueError):
        state.velocity(.2, 0., float('inf'), 1.01)
    assert state.output(1.02) == (0., 0.)
