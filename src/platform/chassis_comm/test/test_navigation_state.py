import pytest

from chassis_comm.navigation_state import NavigationState
from chassis_comm.protocol import Command


def active_state():
    state = NavigationState(.3, .3, .15)
    state.update_velocity(.5, -.25, 10.)
    state.update_position(2., -3., 10.)
    return state


def test_startup_requires_position_and_velocity_but_no_turn_input():
    state = NavigationState(.3, .3)
    assert not state.ready(10.)
    state.update_velocity(.5, 0, 10.)
    assert not state.ready(10.)
    state.update_position(2., -3., 10.)
    assert state.ready(10.)
    assert state.command(10.) == Command(.5, 0, 0, 0, 2., -3.)


def test_hole_is_a_level_and_never_consumed_by_sending():
    state = active_state()
    state.update_hole(True)
    for stamp in (10., 10.1, 10.2):
        assert state.command(stamp) == Command(.5, -.25, 0, 1, 2., -3.)
    state.update_hole(False)
    assert state.command(10.2).is_through_hole == 0


@pytest.mark.parametrize('angle', [1.5707963, -1.5707963, 0.])
def test_explicit_turn_is_independent_of_position_and_expires_to_hold(angle):
    state = active_state()
    state.update_yaw(angle, 10.)
    state.update_position(4., 5., 10.1)
    assert state.command(10.1) == Command(.5, -.25, angle, 0, 4., 5.)
    # Expired relative commands must not keep requesting an old turn.
    assert state.command(10.2) == Command(.5, -.25, 0, 0, 4., 5.)


def test_stale_velocity_or_position_blocks_transmission_readiness():
    state = active_state()
    state.update_position(4., 5., 10.4)
    assert not state.ready(10.4)
    state.update_velocity(.2, .1, 10.4)
    assert state.ready(10.4)
    state.update_velocity(.2, .1, 10.8)
    assert not state.ready(10.8)
    assert not state.ready(9.)


def test_invalid_turn_means_hold_without_stopping_valid_xy():
    state = active_state()
    state.update_yaw(1., 10.)
    with pytest.raises(ValueError):
        state.update_yaw(float('nan'), 10.1)
    assert state.command(10.1) == Command(.5, -.25, 0, 0, 2., -3.)


def test_invalid_velocity_or_position_invalidates_previous_command():
    state = active_state()
    with pytest.raises(ValueError):
        state.update_velocity(float('nan'), 0, 10.1)
    assert not state.ready(10.1)
    state.update_velocity(.5, 0, 10.1)
    with pytest.raises(ValueError):
        state.update_position(float('inf'), 0, 10.1)
    assert not state.ready(10.1)
