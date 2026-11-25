"""Keyboard teleop for the sentry simulation.

    ros2 run sentry_gazebo teleop
    ros2 run sentry_gazebo teleop --ros-args -p linear_speed:=1.0 -p angular_speed:=2.0

Publishes turret-frame XY on ``/cmd_vel`` through the ordinary MCU simulator.
``q``/``e`` use the simulation-only turret position interface; navigation
angular.z is always zero.

Keys
    w / s    forward / backward
    a / d    strafe left / right
    q / e    turn turret left / right
    space    stop
    + / -    faster / slower
    x        quit

Motion keys only act while held down.  A terminal reports no key releases, so
a key counts as released once its auto-repeat stops arriving: ``press_timeout``
after the first press (covers the auto-repeat delay), ``repeat_timeout`` after
a repeat.  The robot is then stopped.

When stdin is not a TTY (a pipe, a launch file) one key per line is read and
held for ``hold`` seconds, so the node can be scripted or used in a test:

    printf 'w\\nw\\na\\n' | ros2 run sentry_gazebo teleop
"""

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64
from rclpy.node import Node

from sentry_gazebo import arena_spec as A

# key -> (turret forward, turret left, turret rotation) unit multipliers
MOTION = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0),
    'd': (0.0, -1.0, 0.0),
    'q': (0.0, 0.0, 1.0),
    'e': (0.0, 0.0, -1.0),
}

HELP = """\
sentry teleop
    w / s    forward / backward
    a / d    strafe left / right
    q / e    turn turret left / right
    space    stop
    + / -    faster / slower
    x        quit
"""


class Teleop(Node):
    """Publish XY while held; use a separate target for independent turret yaw."""

    def __init__(self):
        super().__init__('sentry_teleop')
        self.topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.linear = float(self.declare_parameter('linear_speed', 0.25).value)
        self.angular = float(self.declare_parameter('angular_speed', 0.5).value)
        self.hold = float(self.declare_parameter('hold', 1.0).value)
        self.press_timeout = float(self.declare_parameter('press_timeout', 0.1).value)
        self.repeat_timeout = float(self.declare_parameter('repeat_timeout', 0.1).value)
        rate = float(self.declare_parameter('rate', 20.0).value)
        self.rate = rate
        self.turret_angle = 0.0
        self.turret = self.create_publisher(Float64, '/sim/turret_yaw', 1)

        # limits come from arena_spec so the node cannot ask for more than the
        # rules allow
        self.max_linear = A.MAX_LINEAR_VELOCITY
        self.max_angular = A.MAX_ANGULAR_VELOCITY

        self._pub = self.create_publisher(Twist, self.topic, 10)
        self._motion = (0.0, 0.0, 0.0)
        self.create_subscription(Float32, '/chassis/big_yaw', self.feedback, 1)
        self._cmd = Twist()
        self.create_timer(1.0 / rate, self._publish)

    # -- velocity -----------------------------------------------------------
    def _set_motion(self, motion):
        self._motion = motion
        fwd, left, _ = motion
        cmd = Twist()
        cmd.linear.x = fwd * self.linear
        cmd.linear.y = left * self.linear
        self._cmd = cmd

    def feedback(self, msg):
        if not self._motion[2]:
            self.turret_angle = msg.data

    def _publish(self):
        self._pub.publish(self._cmd)
        if self._motion[2]:
            self.turret_angle += self._motion[2] * self.angular / self.rate
            self.turret.publish(Float64(data=self.turret_angle))

    def stop(self):
        """Zero the command and publish it once, right away."""
        self._set_motion((0.0, 0.0, 0.0))
        try:
            self._pub.publish(self._cmd)
        except Exception:      # context already gone during teardown
            pass

    def apply(self, key):
        """Apply one key press.  Returns False when the node should quit."""
        key = key.lower()
        if key in MOTION:
            self._set_motion(MOTION[key])
            return True
        if key == ' ':
            self.stop()
            return True
        if key in ('+', '='):
            self.linear = min(self.linear * 1.5, self.max_linear)
            self.angular = min(self.angular * 1.5, self.max_angular)
        elif key == '-':
            self.linear = max(self.linear / 1.5, 0.05)
            self.angular = max(self.angular / 1.5, 0.1)
        elif key == 'x':
            self.stop()
            return False
        else:
            return True
        self.get_logger().info('speed %.2f m/s, %.2f rad/s' % (self.linear, self.angular))
        self._set_motion(self._motion)      # re-scale the held direction
        return True

    # -- input --------------------------------------------------------------
    def run(self):
        """Pump the executor and read keys until quit.  Single threaded on
        purpose: a second thread calling rclpy.try_shutdown() races with the
        timer callback and kills the publisher mid-publish."""
        if sys.stdin.isatty():
            self._tty_loop()
        else:
            self._line_loop()

    def _tty_loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        held = None         # motion key currently held down
        deadline = 0.0      # it counts as released if no repeat arrives by then
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
                ready, _, _ = select.select([sys.stdin], [], [], 0)
                if not ready:
                    if held is not None and time.monotonic() > deadline:
                        held = None
                        self.stop()
                    continue
                key = sys.stdin.read(1)
                if key == '\x03':           # Ctrl-C, raw in cbreak mode
                    break
                if not self.apply(key):
                    break
                key = key.lower()
                if key in MOTION:
                    # the first repeat only comes after the auto-repeat delay
                    wait = self.repeat_timeout if key == held else self.press_timeout
                    held = key
                    deadline = time.monotonic() + wait
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _line_loop(self):
        for line in sys.stdin:
            key = line.strip()[:1]
            if not key:
                continue
            if not self.apply(key):
                break
            # keep the executor (and therefore the publish timer) running for
            # the hold time instead of blocking on the next line
            end = time.monotonic() + self.hold
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
            self.stop()


def main(argv=None):
    rclpy.init(args=argv)
    node = Teleop()
    print(HELP, end='', flush=True)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
