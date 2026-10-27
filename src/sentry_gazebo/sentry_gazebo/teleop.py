"""Keyboard teleop for the sentry simulation.

    ros2 run sentry_gazebo teleop
    ros2 run sentry_gazebo teleop --ros-args -p linear_speed:=1.0 -p angular_speed:=2.0

Publishes ``geometry_msgs/Twist`` on ``/cmd_vel``; ``ros_gz_bridge`` forwards it
to the Gazebo planar velocity plugin (``plugins/sentry_velocity_control.py``).
The chassis is a four-wheel omni base, so ``q``/``e`` strafe sideways as well as
``w``/``s``/``a``/``d`` drive -- the plugin integrates all three body-frame
components.

Keys
    w / s    forward / backward
    a / d    turn left / right
    q / e    strafe left / right
    space    stop
    + / -    faster / slower
    x        quit

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
from rclpy.node import Node

from sentry_gazebo import arena_spec as A

# key -> (forward, left, turn-left) unit multipliers, in the body frame
MOTION = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'q': (0.0, 1.0, 0.0),
    'e': (0.0, -1.0, 0.0),
    'a': (0.0, 0.0, 1.0),
    'd': (0.0, 0.0, -1.0),
}

HELP = """\
sentry teleop
    w / s    forward / backward
    a / d    turn left / right
    q / e    strafe left / right
    space    stop
    + / -    faster / slower
    x        quit
"""


class Teleop(Node):
    """Holds one body-frame velocity and republishes it at ``rate`` Hz.

    Republishing matters: the Gazebo plugin expires a command after
    ``timeout`` (0.4 s), so a single latched message would stop the robot.
    """

    def __init__(self):
        super().__init__('sentry_teleop')
        self.topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.linear = float(self.declare_parameter('linear_speed', 0.5).value)
        self.angular = float(self.declare_parameter('angular_speed', 1.0).value)
        self.hold = float(self.declare_parameter('hold', 1.0).value)
        rate = float(self.declare_parameter('rate', 20.0).value)

        # limits come from arena_spec so the node cannot ask for more than the
        # plugin will apply
        self.max_linear = A.MAX_LINEAR_VELOCITY
        self.max_angular = A.MAX_ANGULAR_VELOCITY

        self._pub = self.create_publisher(Twist, self.topic, 10)
        self._motion = (0.0, 0.0, 0.0)
        self._cmd = Twist()
        self.create_timer(1.0 / rate, self._publish)

    # -- velocity -----------------------------------------------------------
    def _set_motion(self, motion):
        self._motion = motion
        fwd, left, turn = motion
        cmd = Twist()
        cmd.linear.x = fwd * self.linear
        cmd.linear.y = left * self.linear
        cmd.angular.z = turn * self.angular
        self._cmd = cmd

    def _publish(self):
        self._pub.publish(self._cmd)

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
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
                ready, _, _ = select.select([sys.stdin], [], [], 0)
                if not ready:
                    continue
                key = sys.stdin.read(1)
                if key == '\x03':           # Ctrl-C, raw in cbreak mode
                    break
                if not self.apply(key):
                    break
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
