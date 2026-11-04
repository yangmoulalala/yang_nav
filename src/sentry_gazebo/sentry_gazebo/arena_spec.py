"""Single source of truth for the RMUL 2027 sentry simulation.

Everything that describes the *scene* lives here, so that the Gazebo world
(:mod:`sentry_gazebo.sdf_builder`) and the documentation cannot drift apart.

Arena
-----
All numbers were measured from the official STEP tessellation
(``RMUL2027_3V3_V1.0.0.stp``, shipped as
``models/rmul_2027/meshes/step_textured.dae``).  The boxes below were validated
with a vertical ray cast on 42 921 sample columns: the top surface agrees with
the tessellation everywhere except on the 50 mm cell borders (the ray test
needs a strictly interior hit, hence the one-cell rim in the validation
report).

Coordinate frame: map frame of ``rmul_2027.pgm`` -- southwest inner corner at
(0, 0), +x east, +y north, finished main floor at z = 0.  The blue half of the
field is the red half rotated by 180 deg about the field centre (6, 4).

Robot
-----
The sensor mount poses are copied verbatim from
``sentry_description/urdf/sentry_robot.urdf`` -- nothing here overrides the
URDF:

* the MID-360 origin is at base_link (0.056, -0.164, 0.137), rolled 105 deg,
  i.e. 0.20 m above the floor,
* the Unitree L2 origin is at base_link (-0.043, 0.176, 0.163), i.e. 0.22 m
  above the floor.

Both were confirmed by forward kinematics of the URDF and by ray casting
against the 1.3 M-triangle chassis meshes.

LiDAR
-----
The point clouds come from Gazebo's built-in ``gpu_lidar`` sensors (ogre2 ray
casting, uniform angular grid).  All sampling parameters, mount poses, frames
and topic names live here so the SDF, the bridge config and the docs cannot
diverge.
"""

import math

PI = math.pi
D2R = math.pi / 180.0


# ---------------------------------------------------------------------------
# Robot
# ---------------------------------------------------------------------------
ROBOT_MODEL_NAME = 'sentry'

# The robot's pose in the world -- the *only* thing that places the model.
# ``sentry_robot.urdf`` is authored z-up (wheels spread over x-y, ``yaw_joint``
# about +z), so no re-orientation is needed.
ROBOT_SPAWN_XY_YAW = (1.25, 6.75, 0.0)   # scenario.json spawn, in the red supply zone
ROBOT_Z = 0.062          # wheel bottoms on the floor (joint z 0.003 - radius 0.065)

# RMUC 2027 full field (``arena:=rmuc``, model://rmuc_2027): converted from
# RMUC2027_V1.0.0.step, 28.0 x 16.0 m, centred on the world origin, lowest
# point at z = 0.  The field has no single flat main floor.  The spawn is the
# flat floor 2 m behind the 0.45 m bump at x = -7.5 (ramps x -8.5..-7.8 and
# -7.0..-6.3): x -10.7..-8.5 is flat at z 0.29 with a 10 mm strip at 0.30
# along y = 0 (vertical ray cast of field_collision.stl).
RMUC_SPAWN_XY_YAW = (-9.5, 0.0, 0.0)
RMUC_SPAWN_FLOOR_Z = 0.30

# Teams.  Both fields are point-symmetric: the blue half is the red half turned
# 180 deg about the field centre, so one team's spawn mirrors onto the other's.
TEAMS = ('red', 'blue')
# The team the spawns above belong to: ROBOT_SPAWN_XY_YAW is the RMUL red
# supply zone; RMUC_SPAWN_XY_YAW is on world -x, which is the BLUE half of
# RMUC (red is at world +x, see REFEREE_FRAMES).
ROBOT_SPAWN_TEAM = 'red'
RMUC_SPAWN_TEAM = 'blue'


def team_spawn(spawn, spawn_team, team, centre):
    """``spawn`` (x, y, yaw) of ``spawn_team`` moved to ``team``'s half.

    The other team's spawn is the same spot turned 180 deg about ``centre``.
    """
    if team not in TEAMS:
        raise ValueError('unknown team %r (choose from %s)' % (team, ', '.join(TEAMS)))
    if team == spawn_team:
        return tuple(spawn)
    x, y, yaw = spawn
    yaw = math.atan2(math.sin(yaw + PI), math.cos(yaw + PI))
    return (2.0 * centre[0] - x, 2.0 * centre[1] - y, yaw)

# The URDF's own orientation.  z-up already, so this stays identity; it is kept
# so a future y-up export only has to change this one value.
ROBOT_MODEL_RPY = (0.0, 0.0, 0.0)


# --- laser mounts -----------------------------------------------------------
# Only which links carry a LiDAR: their pose is the URDF joint chain, read in
# ``sdf_builder`` with ``Urdf.pose_of``.  No numbers live here, so a new CAD
# export cannot desynchronise the simulation.

# --- chassis / wheel geometry ----------------------------------------------
# Wheel joints and every link pose are read straight from sentry_robot.urdf
# (see ``sdf_builder.robot_model_xml``); only what the URDF does not carry --
# the simplified collision shapes -- lives here.
WHEEL_RADIUS = 0.065
WHEEL_LENGTH = 0.04      # wheel_front_left.stl is 4 cm thick along its axis

# Collision hull of the chassis: one box instead of the 1.29 M-triangle STL.
# x/y and the top follow the bounding box of base_link.stl after its URDF
# visual rotation (0.594 x 0.594 x 0.170, centred 0.027 above the link origin).
# The bottom is cut to CHASSIS_GROUND_CLEARANCE above the floor: the raw bbox
# reaches 4 mm above the floor, so the box snagged on the 10 mm centre plate,
# which the 65 mm wheels climb easily.
CHASSIS_GROUND_CLEARANCE = 0.05
_HULL_TOP = 0.027 + 0.170 / 2.0                    # above the link origin
_HULL_BOTTOM = CHASSIS_GROUND_CLEARANCE - ROBOT_Z   # link origin is ROBOT_Z up
CHASSIS_HULL_CENTRE = (0.0, 0.0, (_HULL_TOP + _HULL_BOTTOM) / 2.0)
CHASSIS_HULL_SIZE = (0.594, 0.594, _HULL_TOP - _HULL_BOTTOM)
CHASSIS_MASS = 18.0
TURRET_MASS = 4.0

# chassis speed limits used by teleop (m/s, rad/s) -- RMUL 3v3 rules: 3 m/s chassis
MAX_LINEAR_VELOCITY = 3.0
MAX_ANGULAR_VELOCITY = 6.0


def lidar(name, n_h, n_v, v_min_deg, v_max_deg, r_min, r_max,
          rate=10.0, stddev=0.01, topic=None):
    """One LiDAR: geometry/FOV shared by the SDF and the docs.

    ``topic`` is the *base* topic.  gz-sim expands it into two topics:

        <base>          gz.msgs.LaserScan        (one scan line, for viz)
        <base>/points   gz.msgs.PointCloudPacked (the 3-D cloud we bridge)

    The base name must not start with ``/``, otherwise gz derives no
    ``/points`` topic.  The resulting gz cloud topics are
    :data:`GZ_TOPIC_MID360_CLOUD` / :data:`GZ_TOPIC_L2_CLOUD`; frame ids are
    set by gz (``<model>/<link>/<sensor>``, see :data:`SENSOR_FRAMES`).
    """
    base = (topic or '%s/points' % name).rsplit('/', 1)[0]
    return {
        'name': name,
        'update_rate': rate,
        'topic': topic or '/%s/points' % name,
        # built-in IMU of the device (both MID-360 and L2 have one), mounted at
        # the LiDAR origin, so the frontend uses mid360_link as both frames.imu and
        # frames.lidar (identity extrinsic from TF)
        'imu_name': name.replace('_lidar', '_imu'),
        'imu_topic': '/%s/imu' % base,
        'imu_rate': 200.0,
        'samples_h': n_h,
        'h_min': -PI,
        'h_max': PI,
        'samples_v': n_v,
        'v_min': v_min_deg * D2R,
        'v_max': v_max_deg * D2R,
        'range_min': r_min,
        'range_max': r_max,
        'range_resolution': 0.005,
        'noise_stddev': stddev,
    }


# Livox MID-360 (vendor data sheet): 360 deg x 59 deg FOV (-7 .. +52 deg),
# 200 000 points/s, 0.1 .. 40 m.  gpu_lidar can only do a uniform grid, so the
# 720 x 40 = 28 800 points/frame is a 1.4x oversampled stand-in for the real
# 20 000 points/frame rosette; 720 columns = 0.5 deg, the figure used by the
# LXRobotics Gazebo-Harmonic write-up.  Raise n_h if you want denser clouds.
MID360_LIDAR = lidar('mid360_lidar', 720, 40, -7.0, 52.0, 0.1, 40.0,
                     topic='lidar_mid360/points')
# Unitree 4D LiDAR L2 (vendor manual): 360 deg x 96 deg FOV, 64 000 points/s,
# 0.05 .. 30 m, and a *two-axis single-beam* scan (5.55 Hz azimuth / 216 Hz
# elevation) -- not a 55-ring mechanical LiDAR.  160 x 40 = 6 400 points/frame
# matches the real point budget at 10 Hz, which matters: an SLAM front end
# tuned on a 10x denser simulation falls apart on the real device.
L2_LIDAR = lidar('l2_lidar', 160, 40, -48.0, 48.0, 0.05, 30.0,
                 topic='lidar_l2/points')

# link, parent link, lidar spec, visual box size, colour
ROBOT_MOUNTS = (
    ('mid360_link', 'yaw_link', MID360_LIDAR,
     (0.07, 0.05, 0.05), (0.10, 0.75, 0.35, 1.0)),
    ('l2_link', 'yaw_link', L2_LIDAR,
     (0.055, 0.045, 0.05), (0.15, 0.45, 0.95, 1.0)),
)

# ROS topics (bridged from Gazebo)
TOPIC_CMD_VEL = '/cmd_vel'


# ---------------------------------------------------------------------------
# Referee-system UWB (serial 0x0203), simulated by ``uwb_sim``
# ---------------------------------------------------------------------------
# Ground truth for the simulator only: the stock PosePublisher on the sentry
# model publishes its world pose on this gz topic.  It is deliberately *not*
# bridged to ROS -- the localization stack must not see a ground truth.
GZ_TOPIC_ROBOT_POSE = '/model/%s/pose' % ROBOT_MODEL_NAME
ROBOT_POSE_RATE = 100.0

# Output: what the referee serial driver would publish, the input of
# backend_node (``uwb_pose`` topic, ``uwb.frame``).
TOPIC_UWB_POSE = '/uwb_pose'
# own colour, 'red' / 'blue' (std_msgs/String, transient local)
TOPIC_UWB_TEAM = '/uwb_pose/team'
UWB_FRAME = 'uwb'

# Referee field frame (protocol table 1-21) in the gz world:
#   origin  inner corner of the perimeter next to the RED supply zone,
#   +X      along the long side, towards blue,
#   +Y      along the short side, towards the red landing pad;
#   the same frame for both colours.
# ``x_axis_yaw_deg`` is the world heading of referee +X, ``y_axis_sign`` = +1
# when +Y is +X turned 90 deg counter-clockwise seen from above (right-handed
# with z up), -1 when clockwise.
#
# Only the RMUC field has a referee UWB; RMUL is not simulated.
# RMUC (28 x 16 m centred on the world origin): red is at world +x, so the
# origin is the world corner (14, 8) and the referee frame is the world frame
# turned 180 deg (right-handed, a rigid T_map_uwb exists):
#     x_ref = 14 - x_world        y_ref = 8 - y_world
# The blue spawn (-9.5, 0) reads (23.5, 8), the red spawn (9.5, 0) reads (4.5, 8).
REFEREE_FRAMES = {
    'rmuc': {'origin': (14.0, 8.0), 'x_axis_yaw_deg': 180.0, 'y_axis_sign': 1,
             'length': 28.0},     # x_ref < length / 2 is the red half
}

# Error model of one 0x0203 sample (all overridable as uwb_sim parameters).
# Sources: RoboMaster UWB kit manual -- 50 Hz solution, ~80 ms output delay,
# +-10 cm standard error, ~+-30 cm when occluded; referee protocol -- 0x0203
# at 1 Hz on the regular link, ~130 ms latency (200 ms when congested),
# <1 % packet loss (3 % when congested).
UWB_SIM = {
    'rate_hz': 1.0,              # 0x0203 fixed rate
    'noise_std': 0.08,           # white, per axis [m]
    'bias_std': 0.05,            # slow Gauss-Markov bias, per axis [m]
    'bias_tau': 30.0,            # its correlation time [s]
    'outlier_prob': 0.03,        # chance a sample is an occlusion outlier
    'outlier_std': 0.30,         # outlier std, replaces noise_std [m]
    'latency_mean': 0.20,        # UWB solve ~80 ms + regular link ~130 ms [s]
    'latency_jitter_std': 0.03,  # [s]
    'drop_prob': 0.01,           # lost frames on the regular link
    # tag position in base_link; same numbers as backend uwb.antenna_position_body
    'antenna_position_body': (-0.245, 0.0, 0.0),
}

# gz topic carrying the packed cloud: gz-sim appends "/points" to <topic>
GZ_TOPIC_MID360_CLOUD = '/' + MID360_LIDAR['topic'] + '/points'
GZ_TOPIC_L2_CLOUD = '/' + L2_LIDAR['topic'] + '/points'

# gz sensor name -> link frame; the launch file publishes an identity
# static transform from the link to the scoped frame gz stamps on the cloud
# ("sentry/mid360_link/mid360_lidar"), which SDFormat 1.9 cannot rename.
SENSOR_FRAMES = {**{m[2]['name']: m[0] for m in ROBOT_MOUNTS},
                 **{m[2]['imu_name']: m[0] for m in ROBOT_MOUNTS}}


# ---------------------------------------------------------------------------
# Arena
# ---------------------------------------------------------------------------
FLOOR = (0.62, 0.62, 0.62, 1.0)
FLOOR_SUPPLY = (0.55, 0.58, 0.60, 1.0)
FLOOR_TUNNEL = (0.66, 0.66, 0.68, 1.0)
FLOOR_CENTRE = (0.60, 0.74, 0.86, 1.0)
RED = (0.72, 0.20, 0.20, 1.0)
BLUE = (0.20, 0.32, 0.72, 1.0)
METAL = (0.45, 0.46, 0.50, 1.0)
PARAPET = (0.55, 0.56, 0.60, 1.0)
PERIMETER = (0.26, 0.27, 0.30, 1.0)
CROWN = (0.72, 0.20, 0.20, 1.0)
WORKBENCH = (0.80, 0.62, 0.36, 1.0)
GROUND = (0.20, 0.21, 0.23, 1.0)

PERIMETER_MU = 0.8
FLOOR_MU = 0.8

# name, min corner, size, rgba -- every box also collides (static arena model)
ARENA_BOXES = (
    # --- base -------------------------------------------------------------
    ('ground', (-3.0, -3.0, -0.1), (18.0, 14.0, 0.06), GROUND),
    ('field', (0.0, 0.0, -0.1), (12.0, 8.0, 0.1), FLOOR),
    # --- floors / supply zones / tunnel floors ----------------------------
    ('supply_red', (0.0, 6.0, -0.002), (2.5, 2.0, 0.005), FLOOR_SUPPLY),
    ('supply_blue', (9.5, 0.0, -0.002), (2.5, 2.0, 0.005), FLOOR_SUPPLY),
    ('supply_chamfer_south_red', (0.0, 5.98, 0.0), (2.5, 0.02, 0.002), FLOOR),
    ('supply_chamfer_north_red', (0.0, 8.0, 0.0), (2.5, 0.02, 0.002), FLOOR),
    ('supply_chamfer_west_red', (-0.02, 6.0, 0.0), (0.02, 2.0, 0.002), FLOOR),
    ('supply_chamfer_east_red', (2.5, 6.0, 0.0), (0.02, 2.0, 0.002), FLOOR),
    ('supply_chamfer_south_blue', (9.5, 1.98, 0.0), (2.5, 0.02, 0.002), FLOOR),
    ('supply_chamfer_north_blue', (9.5, 4.0, 0.0), (2.5, 0.02, 0.002), FLOOR),
    ('supply_chamfer_west_blue', (9.48, 2.0, 0.0), (0.02, 2.0, 0.002), FLOOR),
    ('supply_chamfer_east_blue', (12.0, 2.0, 0.0), (0.02, 2.0, 0.002), FLOOR),
    ('tunnel_floor_red', (2.5, 7.06, -0.002), (2.0, 0.8, 0.003), FLOOR_TUNNEL),
    ('tunnel_floor_blue', (7.5, 0.14, -0.002), (2.0, 0.8, 0.003), FLOOR_TUNNEL),
    ('centre_plate', (4.5, 2.5, -0.005), (3.0, 3.0, 0.015), FLOOR_CENTRE),
    ('centre_logo', (5.385, 3.47, 0.01), (1.229, 1.032, 0.001), FLOOR_CENTRE),
    # --- highlands (200 mm) ------------------------------------------------
    ('highland_red', (2.5, 3.0, -0.1), (2.0, 4.06, 0.3), RED),
    ('highland_blue', (7.5, 0.94, -0.1), (2.0, 4.06, 0.3), BLUE),
    ('highland_control_red', (2.9, 4.2, 0.2), (1.2, 1.2, 0.0012), RED),
    ('highland_control_blue', (7.9, 2.6, 0.2), (1.2, 1.2, 0.0012), BLUE),
    ('highland_mark_red', (3.254, 4.588, 0.2), (0.492, 0.413, 0.0016), METAL),
    ('highland_mark_blue', (8.254, 2.999, 0.2), (0.492, 0.413, 0.0016), METAL),
    # parapets: 150 x 1800 x 200 on the far edge of each highland
    ('parapet_red', (4.35, 3.9, 0.2), (0.15, 1.8, 0.2), PARAPET),
    ('parapet_blue', (7.5, 2.3, 0.2), (0.15, 1.8, 0.2), PARAPET),
    # --- tunnel roofs (underside 250, top 270) + ramp on the field side ----
    ('tunnel_roof_red', (3.325, 6.98, 0.25), (0.35, 0.96, 0.02), METAL),
    ('tunnel_roof_blue', (8.325, 0.06, 0.25), (0.35, 0.96, 0.02), METAL),
    ('tunnel_ramp_red', (3.325, 6.92, 0.21), (0.35, 0.06, 0.04), METAL),
    ('tunnel_ramp_blue', (8.325, 1.02, 0.21), (0.35, 0.06, 0.04), METAL),
    ('tunnel_ramp_top_red', (3.325, 6.92, 0.233), (0.35, 0.02, 0.027), METAL),
    ('tunnel_ramp_top_blue', (8.325, 1.02, 0.233), (0.35, 0.02, 0.027), METAL),
    # --- workbenches: 750 x 450 top at 403 --------------------------------
    ('workbench_red', (0.0, 7.55, 0.003), (0.75, 0.45, 0.4), WORKBENCH),
    ('workbench_blue', (11.25, 0.0, 0.003), (0.75, 0.45, 0.4), WORKBENCH),
    # --- perimeter: 300 thick, wall top 400 + 20 crown --------------------
    ('perimeter_south', (-0.3, -0.3, -0.1), (12.6, 0.3, 0.5), PERIMETER),
    ('perimeter_north', (-0.3, 8.0, -0.1), (12.6, 0.3, 0.5), PERIMETER),
    ('perimeter_west', (-0.3, 0.0, -0.1), (0.3, 8.0, 0.5), PERIMETER),
    ('perimeter_east', (12.0, 0.0, -0.1), (0.3, 8.0, 0.5), PERIMETER),
    ('crown_south', (-0.3, -0.2, 0.4), (12.6, 0.2, 0.02), CROWN),
    ('crown_north', (-0.3, 8.0, 0.4), (12.6, 0.2, 0.02), CROWN),
    ('crown_west', (-0.2, 0.0, 0.4), (0.2, 8.0, 0.02), CROWN),
    ('crown_east', (12.0, 0.0, 0.4), (0.2, 8.0, 0.02), CROWN),
)

def box_center(entry):
    """Return (centre xyz, size xyz) for an ``ARENA_BOXES`` entry."""
    _name, mn, size, _rgba = entry
    return (mn[0] + size[0] / 2.0, mn[1] + size[1] / 2.0, mn[2] + size[2] / 2.0), size


def box_mu(name):
    """Friction coefficient of an arena box (currently the same for all)."""
    return PERIMETER_MU

