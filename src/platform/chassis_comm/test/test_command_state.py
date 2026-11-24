import pytest

from chassis_comm.command_state import CommandState
from chassis_comm.protocol import Command


def active_state():
    state = CommandState(0.3, 0.3)
    state.update_velocity(0.5, -0.25, 10.0)
    state.update_yaw(1.25, 10.0)
    state.sent(state.command(10.0))  # receiver first sees a low flag
    return state


def test_startup_sends_zero_relative_yaw_and_velocity():
    state = CommandState(0.3, 0.3)
    state.update_velocity(0.5, 0.0, 10.0)
    assert state.command(10.0) == Command(0, 0, 0, 0)
    assert state.stop_command() == Command(0, 0, 0, 0)


def test_action_is_one_high_frame_per_input_rising_edge():
    state = active_state()
    for i in range(3):
        state.update_action(True, 10.1)
        command = state.command(10.1)
        assert command.action_flag == (1 if i == 0 else 0)
        state.sent(command)
    state.update_action(False, 10.15)
    state.update_action(True, 10.2)
    assert state.command(10.2).action_flag == 1


def test_low_frame_is_required_between_high_frames():
    state = active_state()
    state.update_action(True, 10.1)
    state.sent(state.command(10.1))
    state.update_action(False, 10.12)
    state.update_action(True, 10.13)
    assert state.command(10.13).action_flag == 0


@pytest.mark.parametrize('stale_input', ['velocity', 'yaw', 'both'])
def test_watchdog_zeroes_relative_yaw_and_does_not_replay_action_on_resume(stale_input):
    state = active_state()
    state.update_action(True, 10.1)
    if stale_input == 'yaw':
        state.update_velocity(0.5, -0.25, 10.4)
    if stale_input == 'velocity':
        state.update_yaw(1.25, 10.4)
    assert state.command(10.4) == Command(0, 0, 0, 0)
    state.update_velocity(0.5, -0.25, 10.5)
    state.update_yaw(1.25, 10.5)
    state.update_action(True, 10.5)
    assert state.command(10.5) == Command(0.5, -0.25, 1.25, 0)


def test_action_received_while_stale_is_not_queued():
    state = active_state()
    state.update_action(True, 11.0)
    state.update_velocity(0.5, -0.25, 11.1)
    state.update_yaw(1.25, 11.1)
    assert state.command(11.1).action_flag == 0


def test_action_received_before_first_low_frame_is_not_queued():
    state = CommandState(0.3, 0.3)
    state.update_velocity(0.5, 0.0, 10.0)
    state.update_yaw(1.25, 10.0)
    state.update_action(True, 10.0)
    assert state.command(10.0).action_flag == 0


def test_invalid_inputs_stop_translation_and_rotation():
    state = active_state()
    with pytest.raises(ValueError):
        state.update_yaw(float('nan'), 10.1)
    assert state.command(10.1) == Command(0, 0, 0, 0)
    state.update_yaw(1.25, 10.1)
    with pytest.raises(ValueError):
        state.update_velocity(float('inf'), 0, 10.1)
    assert state.command(10.1) == Command(0, 0, 0, 0)


def test_shutdown_frame_zeroes_relative_target_angle():
    assert active_state().stop_command() == Command(0, 0, 0, 0)


def test_action_angle_navigation_needs_no_yaw_and_never_sends_navigation_yaw():
    state = CommandState(0.3, 0.3, action_only_yaw=True)
    state.update_velocity(.5, -.25, 10.)
    assert state.command(10.) == Command(.5, -.25, 0, 0)
    state.sent(state.command(10.))
    # An absent action angle rejects the trigger, without rejecting X/Y.
    state.update_action(True, 10.1)
    assert state.command(10.1) == Command(.5, -.25, 0, 0)
    state.update_action(False, 10.11)
    state.update_yaw(1.25, 10.12)
    assert state.command(10.12).yaw == 0
    state.update_action(True, 10.13)
    # Snapshot the explicit angle at the action edge, never convert an omega.
    state.update_yaw(-.5, 10.14)
    command = state.command(10.14)
    assert command == Command(.5, -.25, 1.25, 1)
    state.sent(command)
    assert state.command(10.15) == Command(.5, -.25, 0, 0)
    # Yaw expiry cannot stop ordinary navigation.
    state.update_velocity(.2, .1, 11.)
    assert state.command(11.) == Command(.2, .1, 0, 0)


def test_action_angle_watchdog_drops_action_and_never_replays_it():
    state = CommandState(.3, .3, action_only_yaw=True)
    state.update_velocity(.5, 0, 10.)
    state.sent(state.command(10.))
    state.update_yaw(.7, 10.)
    state.update_action(True, 10.1)
    assert state.command(10.4) == Command(0, 0, 0, 0)
    state.update_velocity(.5, 0, 10.5)
    state.update_yaw(.7, 10.5)
    state.update_action(True, 10.5)
    assert state.command(10.5) == Command(.5, 0, 0, 0)
