"""Legacy bench helper; not used by the A3 hardware navigation bridge.

Convert angular velocity into a relative angle over an explicit horizon.

The fixed horizon describes a moving target, not a per-frame turn increment.
Actual angular velocity depends on the MCU's angle-loop response and needs tuning.
"""

import math

from .protocol import Command, encode_frame


def twist_command(vx: float, vy: float, angular_velocity: float, horizon: float) -> Command:
    if not math.isfinite(horizon) or horizon <= 0.0:
        raise ValueError('yaw_horizon must be finite and positive in twist mode')
    if not math.isfinite(angular_velocity):
        raise ValueError('angular.z must be finite')
    command = Command(vx, vy, angular_velocity * horizon)
    encode_frame(command)  # Check all fields before updating any command state.
    return command
