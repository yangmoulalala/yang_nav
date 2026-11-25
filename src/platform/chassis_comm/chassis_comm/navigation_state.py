"""Fresh A3 navigation inputs; position and relative radar turn are independent."""

from .protocol import Command, encode_frame


class NavigationState:
    def __init__(self, command_timeout, pose_timeout, yaw_timeout=0.3):
        self.command_timeout = command_timeout
        self.pose_timeout = pose_timeout
        self.yaw_timeout = yaw_timeout
        self.velocity = (0.0, 0.0)
        self.velocity_stamp = None
        self.position = (0.0, 0.0)
        self.pose_stamp = None
        self.yaw = 0.0
        self.yaw_stamp = None
        self.is_through_hole = False

    @staticmethod
    def _fresh(stamp, now, timeout):
        return stamp is not None and 0.0 <= now - stamp <= timeout

    def update_velocity(self, vx, vy, stamp):
        try:
            encode_frame(Command(vx, vy, 0.0))
        except ValueError:
            self.invalidate_velocity()
            raise
        self.velocity, self.velocity_stamp = (vx, vy), stamp

    def invalidate_velocity(self):
        self.velocity_stamp = None

    def update_position(self, x, y, stamp):
        try:
            encode_frame(Command(0.0, 0.0, 0.0, 0, x, y))
        except ValueError:
            self.pose_stamp = None
            raise
        self.position, self.pose_stamp = (x, y), stamp

    def update_yaw(self, yaw, stamp):
        try:
            encode_frame(Command(0.0, 0.0, yaw))
        except ValueError:
            self.yaw_stamp = None
            raise
        self.yaw, self.yaw_stamp = yaw, stamp

    def update_hole(self, enabled):
        self.is_through_hole = bool(enabled)

    def ready(self, stamp):
        return (self._fresh(self.velocity_stamp, stamp, self.command_timeout)
                and self._fresh(self.pose_stamp, stamp, self.pose_timeout))

    def command(self, stamp):
        if not self.ready(stamp):
            # The hardware node checks ready() and pauses TX, allowing MCU offline stop.
            return Command(0.0, 0.0, 0.0)
        yaw = self.yaw if self._fresh(self.yaw_stamp, stamp, self.yaw_timeout) else 0.0
        return Command(*self.velocity, yaw, int(self.is_through_hole), *self.position)
