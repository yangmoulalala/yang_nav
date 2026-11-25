"""Legacy edge command state retained for simulation; hardware uses NavigationState."""

from .protocol import Command, encode_frame


class CommandState:
    def __init__(self, command_timeout: float, yaw_timeout: float, *, action_only_yaw=False):
        self.action_only_yaw = action_only_yaw
        self._action_yaw = 0.0
        self.command_timeout = command_timeout
        self.yaw_timeout = yaw_timeout
        self.velocity = (0.0, 0.0)
        self.velocity_stamp = None
        self.yaw = 0.0
        self.yaw_stamp = None
        self._input_high = False
        self._pending_action = False
        self._low_sent = False

    def update_velocity(self, vx: float, vy: float, stamp: float):
        try:
            encode_frame(Command(vx, vy, 0.0))
        except ValueError:
            self.invalidate_velocity()
            raise
        self.velocity = (vx, vy)
        self.velocity_stamp = stamp

    def invalidate_velocity(self):
        self.velocity_stamp = None
        self._pending_action = False

    def update_yaw(self, yaw: float, stamp: float):
        try:
            encode_frame(Command(0.0, 0.0, yaw))
        except ValueError:
            self.yaw_stamp = None
            self._pending_action = False
            raise
        self.yaw = yaw
        self.yaw_stamp = stamp

    def ready(self, stamp: float) -> bool:
        return (
            self.velocity_stamp is not None
            and 0.0 <= stamp - self.velocity_stamp <= self.command_timeout
            and (self.action_only_yaw or self.yaw_ready(stamp))
        )

    def yaw_ready(self, stamp: float) -> bool:
        return (self.yaw_stamp is not None
                and 0.0 <= stamp - self.yaw_stamp <= self.yaw_timeout)

    def update_action(self, high: bool, stamp: float):
        rising = high and not self._input_high
        self._input_high = high
        if not high:
            self._pending_action = False
        elif rising:
            # Do not queue a request across a watchdog stop or startup.
            self._pending_action = self.ready(stamp) and self._low_sent and self.yaw_ready(stamp)
            if self._pending_action:
                self._action_yaw = self.yaw

    def command(self, stamp: float) -> Command:
        if not self.ready(stamp):
            self._pending_action = False
            return self.stop_command()
        if self.action_only_yaw:
            if not self.yaw_ready(stamp):
                self._pending_action = False
            action = self._pending_action and self._low_sent
            return Command(*self.velocity, self._action_yaw if action else 0.0, int(action))
        return Command(*self.velocity, self.yaw, int(self._pending_action and self._low_sent))

    def sent(self, command: Command):
        """Commit only after a complete frame write succeeds."""
        self._low_sent = command.action_flag == 0
        if command.action_flag:
            self._pending_action = False

    def stop_command(self) -> Command:
        # yaw is relative to the current heading: never resend an old turn on stop.
        return Command(0.0, 0.0, 0.0, 0)
