"""Referee-system UWB position simulator (serial command 0x0203).

    ros2 run sentry_gazebo uwb_sim --ros-args -p use_sim_time:=true
    ros2 run sentry_gazebo uwb_sim --ros-args -p seed:=42

Only the RMUC field (``arena:=rmuc``) is simulated; RMUL has no referee UWB.

What the real robot gets
------------------------
0x0203 ``robot_pos_t {float x; float y; float angle;}``: the robot's own
position in metres, **1 Hz**, measured by the referee UWB tag and relayed by
the main-control module over the regular link.  The field frame (table 1-21):

* origin: the corner of the field perimeter next to the **red supply zone**,
* +X: along the long side, towards blue,
* +Y: along the short side (into the field),
* the same frame for both colours (it is never mirrored for blue).

``angle`` is the speed-module heading and is not simulated (unused), z does
not exist in 0x0203 and is published as 0.

What this node does
-------------------
1. Reads the ground-truth model pose straight from gz-transport (the
   ``PosePublisher`` system on the sentry model, see ``sdf_builder``).  It is
   *not* bridged to ROS, so nothing else can cheat with it.
2. Every ``1 / rate_hz`` s of sim time it builds one 0x0203 sample:

   * the **antenna** position, i.e. base_link + ``antenna_position_body``
     rotated by the true attitude (the tag is not at the body origin),
   * taken ``latency`` s in the past (UWB solve ~80 ms + regular link
     ~130 ms; the message is stamped at *reception*, like a serial driver),
   * transformed into the referee field frame,
   * plus noise: white Gaussian (``noise_std``, UWB spec +-10 cm), a slow
     Gauss-Markov bias (base-station survey error / multipath), occasional
     occlusion outliers (spec +-30 cm) and packet loss (regular link <1 %),
   * rounded to float32, as on the wire.
3. Publishes ``geometry_msgs/PoseStamped`` on ``/uwb_pose`` with
   ``frame_id = uwb``: exactly the input of ``backend_node`` (``uwb.frame``).
4. Publishes the own colour, ``red`` / ``blue`` (parameter ``team``), as
   ``std_msgs/String`` on ``/uwb_pose/team``: transient local, re-sent with
   every 1 Hz tick.  PoseStamped has no room for it and the backend
   subscribes ``/uwb_pose`` as PoseStamped, so it is a sibling topic.
"""

import bisect
import math
import random
import struct
import threading
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from sentry_gazebo import arena_spec as A


def _f32(v):
    """Round to IEEE float32, the wire type of robot_pos_t."""
    return struct.unpack('<f', struct.pack('<f', v))[0]


def _quat_rotate(q, v):
    """Rotate vector ``v`` by unit quaternion ``q = (w, x, y, z)``."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 q_vec x v ;  v' = v + w t + q_vec x t
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


class RefereeFrame:
    """World (gz) <-> referee field frame, a 2-D rigid motion plus optional mirror.

    ``origin`` is the referee origin in world coordinates, ``x_yaw`` the world
    heading of referee +X and ``y_sign`` = +1 when referee +Y is +X turned 90
    deg counter-clockwise (right-handed, z up) or -1 when it is clockwise
    (left-handed seen from above).
    """

    def __init__(self, origin_x, origin_y, x_yaw, y_sign):
        self.ox, self.oy = origin_x, origin_y
        c, s = math.cos(x_yaw), math.sin(x_yaw)
        self.ex = (c, s)
        self.ey = (-s * y_sign, c * y_sign)
        self.right_handed = y_sign > 0

    def from_world(self, x, y):
        dx, dy = x - self.ox, y - self.oy
        return (self.ex[0] * dx + self.ex[1] * dy,
                self.ey[0] * dx + self.ey[1] * dy)


class GaussMarkov:
    """First-order Gauss-Markov process: stationary std ``sigma``, time constant ``tau``."""

    def __init__(self, sigma, tau, rng):
        self.sigma, self.tau, self.rng = sigma, tau, rng
        self.value = rng.gauss(0.0, sigma) if sigma > 0 else 0.0

    def step(self, dt):
        if self.sigma <= 0:
            return 0.0
        if self.tau <= 0:                       # degenerate: fixed per-run offset
            return self.value
        a = math.exp(-max(dt, 0.0) / self.tau)
        self.value = a * self.value + math.sqrt(1.0 - a * a) * self.rng.gauss(0.0, self.sigma)
        return self.value


class TruthBuffer:
    """Thread-safe history of (t, position, quaternion) from gz-transport."""

    def __init__(self, horizon):
        self.horizon = horizon
        self._lock = threading.Lock()
        self._t = deque()
        self._s = deque()

    def add(self, t, pos, quat):
        with self._lock:
            if self._t and t < self._t[-1]:
                # sim time went backwards (world reset / second gz instance)
                self._t.clear()
                self._s.clear()
            elif self._t and t == self._t[-1]:
                return
            self._t.append(t)
            self._s.append((pos, quat))
            while self._t and self._t[0] < t - self.horizon:
                self._t.popleft()
                self._s.popleft()

    def latest_time(self):
        with self._lock:
            return self._t[-1] if self._t else None

    def sample(self, t, max_extrapolation):
        """Pose at ``t``: linear position interpolation, nearest attitude.

        Returns None when ``t`` is outside the buffer by more than
        ``max_extrapolation`` seconds.
        """
        with self._lock:
            if not self._t:
                return None
            ts = list(self._t)
            ss = list(self._s)
        if t <= ts[0]:
            return ss[0] if ts[0] - t <= max_extrapolation else None
        if t >= ts[-1]:
            return ss[-1] if t - ts[-1] <= max_extrapolation else None
        i = bisect.bisect_right(ts, t)
        t0, t1 = ts[i - 1], ts[i]
        (p0, q0), (p1, q1) = ss[i - 1], ss[i]
        a = (t - t0) / (t1 - t0)
        pos = tuple(p0[k] + a * (p1[k] - p0[k]) for k in range(3))
        return pos, (q0 if a < 0.5 else q1)


class UwbSim(Node):
    def __init__(self):
        super().__init__('uwb_sim')
        # the gz truth is stamped in sim time, so the 1 Hz schedule, the latency
        # look-up and the output stamps must all be sim time too
        if not self.get_parameter('use_sim_time').value:
            self.get_logger().info('forcing use_sim_time:=true (ground truth is in sim time)')
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        arena = p('arena', 'rmuc')
        if arena == 'rmul':
            raise ValueError('RMUL has no referee UWB: uwb_sim only runs on arena:=rmuc')
        if arena not in A.REFEREE_FRAMES:
            raise ValueError('unknown arena %r (choose from %s)' % (arena, ', '.join(A.REFEREE_FRAMES)))
        ref = A.REFEREE_FRAMES[arena]
        u = A.UWB_SIM

        self.topic = p('topic', A.TOPIC_UWB_POSE)
        self.team = str(p('team', A.RMUC_SPAWN_TEAM)).strip().lower()
        if self.team not in A.TEAMS:
            raise ValueError('team must be one of %s, got %r' % (', '.join(A.TEAMS), self.team))
        self.team_topic = p('team_topic', A.TOPIC_UWB_TEAM)
        self.field_length = float(ref.get('length', 0.0))
        self._team_checked = False
        self.frame_id = p('frame_id', A.UWB_FRAME)
        self.gz_topic = p('gz_pose_topic', A.GZ_TOPIC_ROBOT_POSE)
        self.rate = float(p('rate_hz', u['rate_hz']))

        # referee field frame, expressed in the gz world
        self.ref = RefereeFrame(float(p('referee_origin_x', ref['origin'][0])),
                                float(p('referee_origin_y', ref['origin'][1])),
                                math.radians(float(p('referee_x_axis_yaw_deg', ref['x_axis_yaw_deg']))),
                                1 if int(p('referee_y_axis_sign', ref['y_axis_sign'])) >= 0 else -1)

        self.antenna = tuple(float(v) for v in p('antenna_position_body', list(u['antenna_position_body'])))
        self.latency = float(p('latency_mean', u['latency_mean']))
        self.latency_jitter = float(p('latency_jitter_std', u['latency_jitter_std']))
        self.noise_std = float(p('noise_std', u['noise_std']))
        self.outlier_prob = float(p('outlier_prob', u['outlier_prob']))
        self.outlier_std = float(p('outlier_std', u['outlier_std']))
        self.drop_prob = float(p('drop_prob', u['drop_prob']))
        seed = int(p('seed', 0))
        self.rng = random.Random(seed if seed != 0 else None)
        bias_std = float(p('bias_std', u['bias_std']))
        bias_tau = float(p('bias_tau', u['bias_tau']))
        self.bias = (GaussMarkov(bias_std, bias_tau, self.rng), GaussMarkov(bias_std, bias_tau, self.rng))

        if self.rate <= 0:
            raise ValueError('rate_hz must be > 0')
        self.truth = TruthBuffer(horizon=max(3.0, 3.0 * (self.latency + 4 * self.latency_jitter)))
        self._last_t = None
        self._published = 0
        self._dropped = 0

        self.pub = self.create_publisher(PoseStamped, self.topic, 10)
        # latched: a late subscriber gets the colour at once, not after <= 1 s
        self.team_pub = self.create_publisher(
            String, self.team_topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self._team_msg = String(data=self.team)
        self.team_pub.publish(self._team_msg)
        self.get_logger().info('team: %s (on %s)' % (self.team, self.team_topic))
        self._gz_node = self._subscribe_truth()
        self.create_timer(1.0 / self.rate, self._tick)

        self.get_logger().info(
            'UWB (0x0203) sim: %s [%s] @ %.2f Hz, truth from gz %s; referee origin (%.3f, %.3f) '
            'in world, +X heading %.1f deg, %s; noise %.3f m, bias %.3f m/%.0f s, '
            'outliers %.1f%% x %.2f m, latency %.3f+-%.3f s, loss %.1f%%, antenna %s'
            % (self.topic, self.frame_id, self.rate, self.gz_topic, self.ref.ox, self.ref.oy,
               math.degrees(math.atan2(self.ref.ex[1], self.ref.ex[0])),
               'right-handed' if self.ref.right_handed else 'LEFT-handed',
               self.noise_std, bias_std, bias_tau, 100 * self.outlier_prob, self.outlier_std,
               self.latency, self.latency_jitter, 100 * self.drop_prob, self.antenna))
        if not self.ref.right_handed:
            self.get_logger().warn(
                'referee frame is left-handed (z up): no rigid T_map_uwb exists, so the backend '
                'cannot align it (uwb.zero_pose_red / zero_pose_blue).  Mirror y in the '
                'serial driver on the real robot as well, or set referee_y_axis_sign:=1 if the '
                'field frame is actually right-handed.')

    # -- ground truth ---------------------------------------------------------
    def _subscribe_truth(self):
        try:
            from gz.msgs10.pose_pb2 import Pose
            from gz.transport13 import Node as GzNode
        except ImportError as exc:      # python3-gz-transport13 / python3-gz-msgs10
            raise RuntimeError('gz-transport python bindings not available (%s); install '
                               'python3-gz-transport13' % exc) from exc

        def on_pose(msg):
            # PosePublisher stamps with sim time; with publish_model_pose and
            # no link poses the topic carries only the model's world pose
            # (= base_link: the root link sits at the model origin).
            t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec
            pos = (msg.position.x, msg.position.y, msg.position.z)
            q = (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
            if not all(math.isfinite(v) for v in pos + q):
                return
            self.truth.add(t, pos, q)

        node = GzNode()
        if not node.subscribe(Pose, self.gz_topic, on_pose):
            raise RuntimeError('cannot subscribe to gz topic %s' % self.gz_topic)
        return node

    # -- one 0x0203 frame -------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        t_now = now.nanoseconds * 1e-9
        if t_now <= 0.0:
            return
        self.team_pub.publish(self._team_msg)
        dt = 1.0 / self.rate if self._last_t is None else t_now - self._last_t
        if dt < 0:                         # sim time reset
            dt = 1.0 / self.rate
        self._last_t = t_now
        bx, by = self.bias[0].step(dt), self.bias[1].step(dt)   # evolve even if dropped

        latency = max(0.0, self.rng.gauss(self.latency, self.latency_jitter)) \
            if self.latency_jitter > 0 else max(0.0, self.latency)
        state = self.truth.sample(t_now - latency, max_extrapolation=0.1)
        if state is None:
            newest = self.truth.latest_time()
            self.get_logger().warn(
                'no ground truth around t=%.2f (newest gz pose %s) -- is %s published? '
                '(PosePublisher on the sentry model)'
                % (t_now - latency, 'none' if newest is None else '%.2f' % newest, self.gz_topic),
                throttle_duration_sec=5.0)
            return
        if self.rng.random() < self.drop_prob:
            self._dropped += 1
            return

        pos, q = state
        lever = _quat_rotate(q, self.antenna)
        xr, yr = self.ref.from_world(pos[0] + lever[0], pos[1] + lever[1])
        if not self._team_checked and self.field_length > 0:
            # the referee origin is on the red side: x_ref < L/2 is the red half
            self._team_checked = True
            half = 'red' if xr < self.field_length / 2.0 else 'blue'
            if half != self.team:
                self.get_logger().warn(
                    'team:=%s but the robot starts in the %s half (x_ref %.2f m): launch '
                    'with team:=%s so the spawn and the reported colour agree'
                    % (self.team, half, xr, self.team))
        sigma = self.outlier_std if self.rng.random() < self.outlier_prob else self.noise_std
        xr += bx + self.rng.gauss(0.0, sigma)
        yr += by + self.rng.gauss(0.0, sigma)

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()     # reception time, like the serial driver
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = _f32(xr)
        msg.pose.position.y = _f32(yr)
        msg.pose.position.z = 0.0           # 0x0203 has no z
        msg.pose.orientation.w = 1.0        # angle (speed-module heading) not simulated
        self.pub.publish(msg)
        self._published += 1


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = UwbSim()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
