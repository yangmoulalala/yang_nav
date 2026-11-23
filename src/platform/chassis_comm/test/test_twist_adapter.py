import pytest

from chassis_comm.command_state import CommandState
from chassis_comm.protocol import Command
from chassis_comm.twist_adapter import twist_command


@pytest.mark.parametrize('omega', [-2.5, -0.5, 0.0, 0.5, 2.5])
def test_twist_uses_relative_target_over_configured_horizon(omega):
    command = twist_command(0.5, -0.25, omega, 0.2)
    assert command == Command(0.5, -0.25, omega * 0.2)


@pytest.mark.parametrize('horizon', [0.0, -0.1, float('nan'), float('inf')])
def test_horizon_must_be_finite_and_positive(horizon):
    with pytest.raises(ValueError, match='yaw_horizon'):
        twist_command(0, 0, 1, horizon)


@pytest.mark.parametrize('omega', [float('nan'), float('inf'), -float('inf'), 1e100])
def test_invalid_or_unrepresentable_turn_is_rejected(omega):
    with pytest.raises(ValueError):
        twist_command(0.5, -0.25, omega, 0.2)


def test_repeated_sends_do_not_integrate_relative_target_or_refresh_watchdog():
    state = CommandState(0.3, 0.3)
    command = twist_command(0.5, -0.25, 0.75, 0.2)
    state.update_velocity(command.vx, command.vy, 10.0)
    state.update_yaw(command.yaw, 10.0)
    for stamp in (10.0, 10.01, 10.05, 10.15, 10.25):
        assert state.command(stamp) == command
        state.sent(command)
    assert state.command(10.4) == Command(0, 0, 0, 0)
