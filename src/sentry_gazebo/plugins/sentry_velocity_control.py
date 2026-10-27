"""Gazebo Sim 8 (gz-sim 8.15 / Gazebo Harmonic) Python system.

Planar velocity drive for the RMUL 2027 sentry
----------------------------------------------
The chassis is driven by a planar velocity controller instead of contact
simulating the four non-planar wheels (see docs/robot.md).  Every simulation
step the model pose is advanced by the commanded *body-frame* velocity:

    subscribes  /cmd_vel   gz.msgs.Twist   (bridged from ROS by ros_gz_bridge)

That is the only thing this file does.  Everything else is a stock component:

    gz::sim::systems::OdometryPublisher     -> /odom + /tf_odom (Pose_V)
    gz::sim::systems::JointStatePublisher   -> /joint_states
    robot_state_publisher (ROS 2)           -> base_link -> yaw_link -> sensors
    ros_gz_bridge                           -> all of the above to ROS 2

Why not ``gz::sim::systems::VelocityControl``?  It writes a *world-frame*
velocity to the link, while ROS ``cmd_vel`` is body-frame; the planar
integrator below keeps the two semantics apart.

Loading
-------
gz-sim's ``PythonSystemLoader`` imports a *module* by name, so the SDF block is

    <plugin filename="gz-sim-python-system-loader-system"
            name="gz::sim::systems::PythonSystemLoader">
      <module_name>sentry_velocity_control</module_name>
    </plugin>

with this file installed into ``share/sentry_gazebo/plugins`` (that directory is
on ``GZ_SIM_SYSTEM_PLUGIN_PATH``, and ``sitecustomize.py`` next to it appends
it to ``sys.path``).

Parameters (all optional SDF child elements)
    <cmd_topic> /cmd_vel </cmd_topic>
    <max_linear_velocity> 3.0 </max_linear_velocity>
    <max_angular_velocity> 6.0 </max_angular_velocity>
    <timeout> 0.4 </timeout>              # cmd_vel watchdog [s]
"""
import math
import threading

from gz.sim8 import Link, Model
from gz.transport13 import Node as GzNode
from gz.msgs10.twist_pb2 import Twist


def _log(text):
    print('[sentry velocity_control] ' + text, flush=True)


def _sim_time(info):
    """(sec, nsec) of a gz.sim8.UpdateInfo (``sim_time`` is a timedelta in 8.15)."""
    t = info.sim_time
    if hasattr(t, 'total_seconds'):
        sec = t.days * 86400 + t.seconds
        return sec, t.microseconds * 1000
    return int(t.sec), int(t.nsec)


def _children(element):
    """Iterate the direct children of an sdformat14 Element (via its Element view)."""
    if hasattr(element, 'GetElement'):          # sdformat Plugin wrapper
        element = element.GetElement()
    child = element.get_first_element()
    while child is not None:
        yield child
        child = element.get_next_element(child)


class _Params:
    """Reads the plugin's SDF child elements.

    ``sdf`` is an ``sdformat14.Element``; 14.x walks children with
    ``get_first_element()`` / ``get_next_element()``.
    """

    def __init__(self, sdf):
        self.values = {}
        if sdf is None:
            return
        try:
            for child in _children(sdf):
                name = child.get_name()
                text = child.get_string('') if child.has_value() else ''
                if text:
                    self.values[name] = text
        except AttributeError:
            self.values = {}

    def get(self, key, default):
        return self.values.get(key, default)

    def f(self, key, default):
        try:
            return float(self.get(key, '%f' % default))
        except (TypeError, ValueError):
            return default


class _CmdSubscriber:
    """gz-transport ``gz.msgs.Twist`` subscription, drained once per step."""

    def __init__(self, topic):
        self.lock = threading.Lock()
        self.pending = None
        self.node = GzNode()
        self.ok = self.node.subscribe(Twist, topic, self._on_msg)
        if not self.ok:
            _log('cannot subscribe to gz topic %s' % topic)

    def _on_msg(self, msg):
        with self.lock:
            self.pending = (float(msg.linear.x), float(msg.linear.y),
                            float(msg.angular.z))
        return True

    def take(self):
        """Last command received since the previous call (``None`` if none)."""
        with self.lock:
            cmd, self.pending = self.pending, None
        return cmd


class VelocityControl:
    """gz-sim system: planar velocity integration of the whole model.

    The model moves as one rigid body, so the pose is read from and written to
    the canonical link (``gz.sim8.Model`` exposes no pose accessor in 8.15).
    """

    def __init__(self):
        self.ecm = None
        self.model = None
        self.link = None
        self.name = 'sentry'
        self.cmd_topic = '/cmd_vel'
        self.sub = None
        self.max_lin = 3.0
        self.max_ang = 6.0
        self.timeout = 0.4
        self.cmd = (0.0, 0.0, 0.0)
        self.last_cmd_t = -1e9
        self.last_t = None
        self.z0 = None
        # roll/pitch of the canonical link at spawn; only yaw changes while
        # driving (identity for a z-up URDF, non-zero for a tilted export).
        self.pose_rpy = (0.0, 0.0, 0.0)

    # -- lifecycle ----------------------------------------------------------
    def configure(self, entity, sdf, ecm, event_mgr):
        self.ecm = ecm
        self.model = Model(entity)
        self.name = self.model.name(ecm)
        self.link = Link(self.model.canonical_link(ecm))
        p = _Params(sdf)
        self.cmd_topic = p.get('cmd_topic', '/cmd_vel')
        self.max_lin = p.f('max_linear_velocity', 3.0)
        self.max_ang = p.f('max_angular_velocity', 6.0)
        self.timeout = p.f('timeout', 0.4)
        spawn = self.link.world_pose(ecm)
        self.pose_rpy = (spawn.roll(), spawn.pitch(), spawn.yaw())
        self.sub = _CmdSubscriber(self.cmd_topic)
        _log('configured for model %r (vmax %.1f m/s, wmax %.1f rad/s, cmd %s)'
             % (self.name, self.max_lin, self.max_ang, self.cmd_topic))
        return True

    # -- pose helpers -------------------------------------------------------
    def _pose(self):
        p = self.link.world_pose(self.ecm)
        return p.pos().x(), p.pos().y(), p.pos().z(), p.yaw()

    def _set_pose(self, x, y, z, yaw):
        """Write the canonical link pose back; only the yaw term changes.

        ``gz.math7.Pose3d.set(x, y, z, roll, pitch, yaw)`` is used because the
        quaternion returned by ``rot()`` is a copy and ``Quaterniond`` has no
        ``Set`` method in these bindings.
        """
        pose = self.link.world_pose(self.ecm)
        pose.set(float(x), float(y), float(z),
                 self.pose_rpy[0], self.pose_rpy[1], float(yaw))
        self.model.set_world_pose_cmd(self.ecm, pose)

    # -- system interface ---------------------------------------------------
    def pre_update(self, info, ecm):
        sec, nsec = _sim_time(info)
        t = sec + nsec * 1e-9
        dt = 0.0 if self.last_t is None else max(0.0, min(0.1, t - self.last_t))
        self.last_t = t

        cmd = self.sub.take() if self.sub is not None else None
        if cmd is not None:
            self.cmd = cmd
            self.last_cmd_t = t
        vx, vy, wz = self.cmd
        if t - self.last_cmd_t > self.timeout:
            vx, vy, wz = 0.0, 0.0, 0.0
        vx = max(-self.max_lin, min(self.max_lin, vx))
        vy = max(-self.max_lin, min(self.max_lin, vy))
        wz = max(-self.max_ang, min(self.max_ang, wz))

        x, y, z, yaw = self._pose()
        if dt > 0.0 and (vx or vy or wz):
            if self.z0 is None:
                # freeze the settled ride height on the first command
                self.z0 = z
            x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            yaw += wz * dt
            self._set_pose(x, y, self.z0, yaw)
        return True

    def reset(self, info, ecm):
        self.last_t = None
        self.cmd = (0.0, 0.0, 0.0)
        self.last_cmd_t = -1e9
        self.z0 = None
        return True


def get_system():
    return VelocityControl()
