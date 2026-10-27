#!/usr/bin/env python3
"""URDF -> SDF world generation for the sentry simulation.

Why this exists
---------------
``ros_gz_sim create`` + ``sdformat_urdf`` is installed and would spawn the URDF
directly, but it takes the URDF ``<collision>`` meshes verbatim: 1.29 M
triangles for the chassis and 0.92 M for the turret, which makes the physics
step unusable.  It would also require ``<gazebo>`` sensor blocks inside
``sentry_description``, a package shared with the real robot.

So this module converts the URDF itself and keeps the URDF the single source of
truth for geometry:

* link poses are the URDF forward kinematics, joint origins and axes are copied
  from the URDF joints -- no hand-tuned numbers,
* collisions are primitives (one box for the chassis, one cylinder per wheel),
  visuals keep the URDF meshes,
* the four omni wheels (X layout, axles on the diagonals, coplanar contacts)
  are driven by the stock ``MecanumDrive`` system through real wheel/ground
  contacts, so the IMUs read physical accelerations (see ``omni_wheel``),
* the LiDARs are ``gpu_lidar`` sensors on the URDF mount links, each with
  an ``imu`` sensor at the same origin (both devices have a built-in IMU).

``arena_spec`` is the single source of truth for the arena and for the sensor
sampling parameters.

Output
------
``build()`` returns the complete world SDF (arena + robot inline, two
``gpu_lidar`` sensors).  It is written to the generated directory on every
launch by ``launch/prepare_world.py``.

Usage
-----
    python3 -m sentry_gazebo.sdf_builder \
        --urdf ../sentry_description/urdf/sentry_robot.urdf \
        --output /tmp/world.sdf
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from sentry_gazebo import arena_spec as A

# --------------------------------------------------------------------------- config
# URDF STLs are exposed as a Gazebo resource model named sentry_description
# (a symlink to share/sentry_description/meshes, created by prepare_world).
MESH_URI_PREFIX = 'model://sentry_description/meshes/'
# Official STEP tessellation, merged into this package as models/rmul_2027.
ARENA_MESH_URI = 'model://rmul_2027/meshes/step_textured.dae'

CHASSIS_RGBA = (0.53, 0.54, 0.58, 1.0)
TURRET_RGBA = (0.62, 0.62, 0.68, 1.0)
WHEEL_RGBA = (0.10, 0.10, 0.11, 1.0)

# robot-centric collision / inertia choices (see docs/robot.md)
CHASSIS_INERTIA = (0.32, 0.32, 0.42)     # ixx, iyy, izz [kg m^2]
TURRET_INERTIA = (0.05, 0.05, 0.05)
WHEEL_MASS = 0.5

# The real LiDARs do not see the robot's own meshes as returns (the vendor
# drivers / blind zone drop them), but gpu_lidar renders every visual.  Robot
# visuals carry this flag and the LiDARs mask it out.
ROBOT_VISIBILITY = 0x2
LIDAR_VISIBILITY_MASK = 0xFFFFFFFF & ~ROBOT_VISIBILITY


# --------------------------------------------------------------------------- helpers
def n(x):
    """Compact deterministic number formatting."""
    s = '%.6f' % float(x)
    s = s.rstrip('0').rstrip('.')
    return '0' if s in ('', '-0') else s


def vec(v):
    return ' '.join(n(x) for x in v)


def rpy_to_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_to_rpy(m):
    p = math.asin(max(-1.0, min(1.0, -m[2][0])))
    return math.atan2(m[2][1], m[2][2]), p, math.atan2(m[1][0], m[0][0])


def inverse_rpy(rpy):
    """ZYX Euler angles of ``R(rpy)^-1`` (for gz's ``<rpy_offset>``)."""
    m = rpy_to_mat(*rpy)
    return mat_to_rpy([[m[j][i] for j in range(3)] for i in range(3)])


def _xyz(text):
    return tuple(float(v) for v in text.split()) if text else (0.0, 0.0, 0.0)


# --------------------------------------------------------------------------- URDF model
class Link:
    def __init__(self, el):
        self.name = el.get('name')
        self.visual_mesh = None
        v = el.find('visual')
        # <visual><geometry><mesh .../></geometry></visual>
        m = v.find('geometry/mesh') if v is not None else None
        if m is not None:
            o = v.find('origin')
            scale = m.get('scale')
            self.visual_mesh = (m.get('filename'),
                                _xyz(o.get('xyz') if o is not None else None),
                                _xyz(o.get('rpy') if o is not None else None),
                                _xyz(scale) if scale else (1.0, 1.0, 1.0))


class Joint:
    def __init__(self, el):
        self.name = el.get('name')
        self.type = el.get('type')
        self.parent = el.find('parent').get('link')
        self.child = el.find('child').get('link')
        o = el.find('origin')
        self.xyz = _xyz(o.get('xyz') if o is not None else None)
        self.rpy = _xyz(o.get('rpy') if o is not None else None)
        self.axis = _xyz(el.find('axis').get('xyz')) if el.find('axis') is not None else (1.0, 0.0, 0.0)


class Urdf:
    """Parsed URDF plus forward kinematics for every link (root frame)."""

    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.path = Path(path)
        self.links = {el.get('name'): Link(el) for el in root.findall('link')}
        self.joints = [Joint(el) for el in root.findall('joint')]
        children = {j.child for j in self.joints}
        roots = [name for name in self.links if name not in children]
        if len(roots) != 1:
            raise RuntimeError('URDF must have exactly one root link, found %r' % roots)
        self.root = roots[0]
        self.parent_joint = {j.child: j for j in self.joints}
        self.pose = {self.root: ([0.0, 0.0, 0.0], [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])}
        pending = list(self.joints)
        while pending:
            progressed = False
            for j in list(pending):
                if j.parent in self.pose:
                    pxyz, pR = self.pose[j.parent]
                    t = mat_vec(pR, j.xyz)
                    self.pose[j.child] = ([pxyz[i] + t[i] for i in range(3)],
                                          mat_mul(pR, rpy_to_mat(*j.rpy)))
                    pending.remove(j)
                    progressed = True
            if not progressed:
                raise RuntimeError('kinematic loop or disconnected joint in the URDF')

    def chain(self, link):
        """Joints from the root down to ``link`` (root first)."""
        out = []
        while link in self.parent_joint:
            j = self.parent_joint[link]
            out.append(j)
            link = j.parent
        return list(reversed(out))

    def axis_of(self, joint):
        """``<axis>`` of a joint, exactly as the URDF declares it."""
        for j in self.joints:
            if j.name == joint:
                return j.axis
        raise KeyError(joint)

    def pose_of(self, link):
        xyz, R = self.pose[link]
        return tuple(xyz), mat_to_rpy(R)

    def subtree(self, link):
        todo = [link]
        out = []
        while todo:
            cur = todo.pop()
            out.append(cur)
            todo.extend(j.child for j in self.joints if j.parent == cur)
        return out

    def mesh_of(self, link):
        lk = self.links.get(link)
        return lk.visual_mesh if lk is not None else None


# --------------------------------------------------------------------------- SDF writer
class W:
    def __init__(self):
        self.lines = []

    def __call__(self, indent, text):
        self.lines.append(' ' * indent + text)

    def text(self):
        return '\n'.join(self.lines) + '\n'


def material_xml(w, indent, rgba, specular=0.12, roughness_comment=None):
    w(indent, '<material>')
    w(indent + 2, '<ambient>%s</ambient>' % vec([rgba[0] * 0.45, rgba[1] * 0.45, rgba[2] * 0.45, 1.0]))
    w(indent + 2, '<diffuse>%s</diffuse>' % vec(rgba))
    w(indent + 2, '<specular>%s %s %s 1</specular>' % (n(specular), n(specular), n(specular)))
    w(indent, '</material>')


def box_geom(w, indent, centre, size):
    w(indent, '<pose>%s 0 0 0</pose>' % vec(centre))
    w(indent, '<geometry><box><size>%s</size></box></geometry>' % vec(size))


def friction_xml(w, indent, mu):
    w(indent, '<surface><friction><ode><mu>%s</mu><mu2>%s</mu2></ode></friction></surface>' % (n(mu), n(mu)))


def box_xml(w, indent, centre, size, rgba, name, collision=False, mu=None, visibility=None):
    tag = 'collision' if collision else 'visual'
    w(indent, '<%s name="%s">' % (tag, name))
    box_geom(w, indent + 2, centre, size)
    if collision:
        if mu is not None:
            friction_xml(w, indent + 2, mu)
    else:
        material_xml(w, indent + 2, rgba)
        if visibility is not None:
            w(indent + 2, '<visibility_flags>%d</visibility_flags>' % visibility)
    w(indent, '</%s>' % tag)


def cylinder_xml(w, indent, radius, length, rgba, name, collision=False, mu=None):
    tag = 'collision' if collision else 'visual'
    w(indent, '<%s name="%s">' % (tag, name))
    w(indent + 2, '<geometry><cylinder><radius>%s</radius><length>%s</length></cylinder></geometry>'
      % (n(radius), n(length)))
    if collision:
        if mu is not None:
            friction_xml(w, indent + 2, mu)
    else:
        material_xml(w, indent + 2, rgba)
    w(indent, '</%s>' % tag)


def mesh_visual_xml(w, indent, mesh, rgba, name):
    """``<visual>`` for a URDF mesh, exactly as the URDF places it."""
    fn, oxyz, orpy, scale = mesh
    w(indent, '<visual name="%s">' % name)
    w(indent + 2, '<pose>%s %s</pose>' % (vec(oxyz), vec(orpy)))
    basename = Path(fn).name
    w(indent + 2, '<geometry><mesh><uri>%s%s</uri><scale>%s</scale></mesh></geometry>'
      % (MESH_URI_PREFIX, basename, vec(scale)))
    material_xml(w, indent + 2, rgba)
    w(indent + 2, '<visibility_flags>%d</visibility_flags>' % ROBOT_VISIBILITY)
    w(indent, '</visual>')


def inertial_xml(w, indent, mass, ixx, iyy, izz, pose=(0, 0, 0, 0, 0, 0)):
    w(indent, '<inertial>')
    w(indent + 2, '<pose>%s</pose>' % vec(pose))
    w(indent + 2, '<mass>%s</mass>' % n(mass))
    w(indent + 2, '<inertia>')
    w(indent + 4, '<ixx>%s</ixx><ixy>0</ixy><ixz>0</ixz>' % n(ixx))
    w(indent + 4, '<iyy>%s</iyy><iyz>0</iyz>' % n(iyy))
    w(indent + 4, '<izz>%s</izz>' % n(izz))
    w(indent + 2, '</inertia>')
    w(indent, '</inertial>')


def joint_xml(w, indent, joint, joint_type, effort=20.0, velocity=60.0, damping=0.0,
              limits=(-1e16, 1e16), axis=None):
    """``<joint>`` for a URDF joint: same parent/child, origin and axis.

    SDFormat has no ``continuous`` type, so those become ``revolute`` with wide
    limits (or the given ``limits``); the URDF stays the source of truth for
    origin and axis.
    """
    w(indent, '<joint name="%s" type="%s">' % (joint.name, joint_type))
    w(indent + 2, '<parent>%s</parent><child>%s</child>' % (joint.parent, joint.child))
    w(indent + 2, '<pose relative_to="%s">%s %s</pose>'
      % (joint.parent, vec(joint.xyz), vec(joint.rpy)))
    if joint_type != 'fixed':
        w(indent + 2, '<axis>')
        w(indent + 4, '<xyz>%s</xyz>' % vec(axis or joint.axis))
        w(indent + 4, '<limit><lower>%s</lower><upper>%s</upper>'
                      '<effort>%s</effort><velocity>%s</velocity></limit>'
          % (n(limits[0]), n(limits[1]), n(effort), n(velocity)))
        w(indent + 4, '<dynamics><damping>%s</damping></dynamics>' % n(damping))
        w(indent + 2, '</axis>')
    w(indent, '</joint>')


# --------------------------------------------------------------------------- sensors
def gpu_lidar_xml(w, indent, spec, parent, mount_xyz, mount_rpy):
    """``<sensor type="gpu_lidar">`` -- uniform angular grid, ogre2 ray casting.

    The sensor frame is the ``*_link`` body frame itself (+x out of the
    aperture, +y left, +z up), exactly like the URDF link frame, so the point
    cloud header frame matches the TF tree without extra rotations.
    """
    w(indent, '<sensor name="%s" type="gpu_lidar">' % spec['name'])
    w(indent + 2, '<pose relative_to="%s">%s %s</pose>' % (parent, vec(mount_xyz), vec(mount_rpy)))
    w(indent + 2, '<always_on>1</always_on>')
    w(indent + 2, '<update_rate>%s</update_rate>' % n(spec['update_rate']))
    w(indent + 2, '<visualize>false</visualize>')
    w(indent + 2, '<!-- gz publishes <topic> as LaserScan and <topic>/points as')
    w(indent + 2, '     PointCloudPacked; the bridge takes the latter -->')
    w(indent + 2, '<topic>%s</topic>' % spec['topic'])
    w(indent + 2, '<lidar>')
    w(indent + 4, '<scan>')
    w(indent + 6, '<horizontal>')
    w(indent + 8, '<samples>%d</samples>' % spec['samples_h'])
    w(indent + 8, '<resolution>1</resolution>')
    w(indent + 8, '<min_angle>%s</min_angle>' % n(spec['h_min']))
    w(indent + 8, '<max_angle>%s</max_angle>' % n(spec['h_max']))
    w(indent + 6, '</horizontal>')
    w(indent + 6, '<vertical>')
    w(indent + 8, '<samples>%d</samples>' % spec['samples_v'])
    w(indent + 8, '<resolution>1</resolution>')
    w(indent + 8, '<min_angle>%s</min_angle>' % n(spec['v_min']))
    w(indent + 8, '<max_angle>%s</max_angle>' % n(spec['v_max']))
    w(indent + 6, '</vertical>')
    w(indent + 4, '</scan>')
    if spec['noise_stddev'] > 0:
        w(indent + 4, '<noise>')
        w(indent + 6, '<type>gaussian</type>')
        w(indent + 6, '<mean>0</mean>')
        w(indent + 6, '<stddev>%s</stddev>' % n(spec['noise_stddev']))
        w(indent + 4, '</noise>')
    w(indent + 4, '<range>')
    w(indent + 6, '<min>%s</min>' % n(spec['range_min']))
    w(indent + 6, '<max>%s</max>' % n(spec['range_max']))
    w(indent + 6, '<resolution>%s</resolution>' % n(spec['range_resolution']))
    w(indent + 4, '</range>')
    w(indent + 4, '<visibility_mask>%d</visibility_mask>' % LIDAR_VISIBILITY_MASK)
    w(indent + 2, '</lidar>')
    w(indent, '</sensor>')


def imu_xml(w, indent, spec, parent):
    """``<sensor type="imu">`` at the LiDAR origin, like the device's built-in IMU.

    Noise follows the order of magnitude of a MEMS IMU (BMI088 class); the
    reading is specific force in m/s^2, gravity included.
    """
    w(indent, '<sensor name="%s" type="imu">' % spec['imu_name'])
    w(indent + 2, '<pose relative_to="%s">0 0 0 0 0 0</pose>' % parent)
    w(indent + 2, '<always_on>1</always_on>')
    w(indent + 2, '<update_rate>%s</update_rate>' % n(spec['imu_rate']))
    w(indent + 2, '<topic>%s</topic>' % spec['imu_topic'])
    w(indent + 2, '<imu>')
    for group, stddev in (('angular_velocity', 2e-3), ('linear_acceleration', 2e-2)):
        w(indent + 4, '<%s>' % group)
        for axis in 'xyz':
            w(indent + 6, '<%s><noise type="gaussian"><mean>0</mean><stddev>%s</stddev>'
                          '</noise></%s>' % (axis, n(stddev), axis))
        w(indent + 4, '</%s>' % group)
    w(indent + 2, '</imu>')
    w(indent, '</sensor>')


# --------------------------------------------------------------------------- arena
def arena_model_xml(w, indent, with_visual_mesh):
    """Static arena: official STEP mesh as visual + exact primitive collisions."""
    w(indent, '<model name="rmul_2027">')
    w(indent + 2, '<static>true</static>')
    w(indent + 2, '<pose>0 0 0 0 0 0</pose>')
    w(indent + 2, '<link name="arena">')
    inertial_xml(w, indent + 4, 1.0, 1.0, 1.0, 1.0)
    if with_visual_mesh:
        w(indent + 4, '<!-- official STEP tessellation: markings and textures (visual only) -->')
        w(indent + 4, '<visual name="official_step">')
        w(indent + 6, '<geometry><mesh><uri>%s</uri></mesh></geometry>' % ARENA_MESH_URI)
        w(indent + 4, '</visual>')
    else:
        for entry in A.ARENA_BOXES:
            name, _mn, _size, rgba = entry
            centre, size = A.box_center(entry)
            box_xml(w, indent + 4, centre, size, rgba, '%s_visual' % name)
    for entry in A.ARENA_BOXES:
        name, _mn, _size, _rgba = entry
        centre, size = A.box_center(entry)
        box_xml(w, indent + 4, centre, size, (0, 0, 0, 1), '%s_collision' % name,
                collision=True, mu=A.box_mu(name))
    w(indent + 2, '</link>')
    w(indent, '</model>')


# --------------------------------------------------------------------------- omni wheels
# gz MecanumDrive wheel speeds, as signs of (vx, vy, wz):
#   front_left = vx - vy - L*wz    front_right = vx + vy + L*wz
#   back_left  = vx + vy - L*wz    back_right  = vx - vy + L*wz
MECANUM_SLOTS = {'front_left': (1, -1, -1), 'front_right': (1, 1, 1),
                 'back_left': (1, 1, -1), 'back_right': (1, -1, 1)}


def omni_wheel(joint):
    """MecanumDrive slot, axis sign and rolling direction of one omni wheel.

    A wheel spinning +w about its axle ``a`` pushes the base along ``d = a x z``
    (its rollers leave ``a`` free), so its speed for a base twist is
    ``d . (v + wz z x p) / r``.  With the axles on the diagonals this is one
    mecanum slot divided by sqrt(2), hence ``wheel_radius = sqrt(2) r`` in the
    MecanumDrive block.  A wheel matching the negated slot pattern gets its
    joint axis flipped: only the joint sign changes, geometry stays the URDF's.
    """
    a = mat_vec(rpy_to_mat(*joint.rpy), joint.axis)
    norm = math.hypot(a[0], a[1])
    d = (a[1] / norm, -a[0] / norm)
    k = (d[0], d[1], d[1] * joint.xyz[0] - d[0] * joint.xyz[1])
    signs = tuple(1 if c > 1e-6 else -1 if c < -1e-6 else 0 for c in k)
    for slot, pattern in MECANUM_SLOTS.items():
        if signs == pattern:
            return slot, 1.0, d
        if signs == tuple(-s for s in pattern):
            return slot, -1.0, d
    raise RuntimeError('%s is not an X-layout omni wheel (signs %r)' % (joint.name, signs))


# --------------------------------------------------------------------------- robot
# The URDF is used as-is: every link carries the pose the URDF gives it and the
# model pose places the whole robot in the world (``ROBOT_SPAWN_XY_YAW`` /
# ``ROBOT_Z`` below).  Nothing here re-orients the robot: no helper frame, no
# extra rotation on the links, no rotation on the meshes.
def robot_model_xml(w, indent, urdf):
    """Sentry model: URDF links/visuals verbatim, simplified physics, two LiDARs."""
    root = urdf.root
    spawn = A.ROBOT_SPAWN_XY_YAW
    w(indent, '<model name="%s">' % A.ROBOT_MODEL_NAME)
    w(indent + 2, '<!-- spawn pose (world) -->')
    w(indent + 2, '<pose>%s %s %s %s %s %s</pose>'
      % (n(spawn[0]), n(spawn[1]), n(A.ROBOT_Z),
         n(A.ROBOT_MODEL_RPY[0]), n(A.ROBOT_MODEL_RPY[1]), n(spawn[2])))
    w(indent + 2, '<self_collide>false</self_collide>')

    # ---- chassis -----------------------------------------------------------
    w(indent + 2, '<link name="%s">' % root)
    inertial_xml(w, indent + 4, A.CHASSIS_MASS, *CHASSIS_INERTIA, pose=A.CHASSIS_HULL_CENTRE + (0.0, 0.0, 0.0))
    if urdf.mesh_of(root) is not None:
        # the URDF <visual> origin, used verbatim
        mesh_visual_xml(w, indent + 4, urdf.mesh_of(root), CHASSIS_RGBA, 'chassis_mesh')
    w(indent + 4, '<!-- collision hull: one box instead of the 1.29 M-triangle STL -->')
    box_xml(w, indent + 4, A.CHASSIS_HULL_CENTRE, A.CHASSIS_HULL_SIZE,
            (0, 0, 0, 1), 'chassis_collision', collision=True, mu=A.FLOOR_MU)
    w(indent + 2, '</link>')

    # ---- omni wheels, driven by MecanumDrive below -------------------------
    wheels = [j for j in urdf.joints if j.parent == root and j.child.startswith('wheel_')]
    slots = {}
    for j in wheels:
        slot, sign, roll = omni_wheel(j)
        slots.setdefault(slot, []).append(j.name)
        xyz, rpy = urdf.pose_of(j.child)
        w(indent + 2, '<link name="%s">' % j.child)
        w(indent + 4, '<pose relative_to="%s">%s %s</pose>' % (root, vec(xyz), vec(rpy)))
        inertial_xml(w, indent + 4, WHEEL_MASS, 6e-4, 6e-4, 1.2e-3)
        if urdf.mesh_of(j.child) is not None:
            mesh_visual_xml(w, indent + 4, urdf.mesh_of(j.child), WHEEL_RGBA, 'wheel_mesh')
        # one cylinder instead of the 109 k-triangle wheel STL; friction only
        # along the rolling direction (fdir1), none along the axle, like the
        # free rollers of an omni wheel
        w(indent + 4, '<collision name="wheel_collision">')
        w(indent + 6, '<geometry><cylinder><radius>%s</radius><length>%s</length></cylinder></geometry>'
          % (n(A.WHEEL_RADIUS), n(A.WHEEL_LENGTH)))
        w(indent + 6, '<surface><friction><ode><mu>1</mu><mu2>0</mu2>'
                      '<fdir1 gz:expressed_in="%s">%s 0</fdir1></ode></friction></surface>'
          % (root, vec(roll)))
        w(indent + 4, '</collision>')
        w(indent + 2, '</link>')
        joint_xml(w, indent + 2, j, 'revolute', effort=20, velocity=60, damping=0.02,
                  axis=tuple(sign * c for c in j.axis))
    if sorted(slots) != sorted(MECANUM_SLOTS):
        raise RuntimeError('omni wheels do not fill the four MecanumDrive slots: %r' % slots)

    # ---- turret ------------------------------------------------------------
    if 'yaw_link' in urdf.links:
        yxyz, yrpy = urdf.pose_of('yaw_link')
        w(indent + 2, '<link name="yaw_link">')
        w(indent + 4, '<pose relative_to="%s">%s %s</pose>' % (root, vec(yxyz), vec(yrpy)))
        inertial_xml(w, indent + 4, A.TURRET_MASS, *TURRET_INERTIA)
        if urdf.mesh_of('yaw_link') is not None:
            mesh_visual_xml(w, indent + 4, urdf.mesh_of('yaw_link'), TURRET_RGBA, 'turret_mesh')
        w(indent + 2, '</link>')
        # The turret is not actuated in simulation, so yaw_joint is fixed at 0:
        # a passive or limit-pinned joint drags/jitters the MID-360 and its IMU.
        # joint_state_publisher (launch file) reports the zero angle for TF.
        joint_xml(w, indent + 2, urdf.parent_joint['yaw_link'], 'fixed')

    # ---- LiDAR sensors -----------------------------------------------------
    # Link pose = URDF joint chain (Urdf.pose_of), joint = the URDF joint itself.
    for (link, parent, spec, size, rgba) in A.ROBOT_MOUNTS:
        mxyz, mrpy = urdf.pose_of(link)
        w(indent + 2, '<!-- %s: URDF joint %s, parent %s -->'
          % (link, urdf.parent_joint[link].name, parent))
        w(indent + 2, '<link name="%s">' % link)
        w(indent + 4, '<pose relative_to="%s">%s %s</pose>' % (root, vec(mxyz), vec(mrpy)))
        if urdf.mesh_of(link) is not None:
            mesh_visual_xml(w, indent + 4, urdf.mesh_of(link), rgba, 'housing')
        else:
            box_xml(w, indent + 4, (0, 0, 0), size, rgba, 'housing', visibility=ROBOT_VISIBILITY)
        gpu_lidar_xml(w, indent + 4, spec, link, (0, 0, 0), (0, 0, 0))
        imu_xml(w, indent + 4, spec, link)
        w(indent + 2, '</link>')
        joint_xml(w, indent + 2, urdf.parent_joint[link], 'fixed')

    # ---- drive / ground truth ----------------------------------------------
    # Stock MecanumDrive: body-frame /cmd_vel -> omni wheel joint velocities.
    w(indent + 2, '<plugin filename="gz-sim-mecanum-drive-system" '
                  'name="gz::sim::systems::MecanumDrive">')
    for slot in MECANUM_SLOTS:
        for name in slots[slot]:
            w(indent + 4, '<%s_joint>%s</%s_joint>' % (slot, name, slot))
    w(indent + 4, '<wheel_separation>%s</wheel_separation>'
      % n(2 * max(abs(j.xyz[1]) for j in wheels)))
    w(indent + 4, '<wheelbase>%s</wheelbase>' % n(2 * max(abs(j.xyz[0]) for j in wheels)))
    w(indent + 4, '<wheel_radius>%s</wheel_radius>' % n(math.sqrt(2) * A.WHEEL_RADIUS))
    w(indent + 4, '<topic>%s</topic>' % A.TOPIC_CMD_VEL)
    w(indent + 2, '</plugin>')

    # Stock gz system: ground-truth odometry (+ odom -> base_link TF).
    w(indent + 2, '<plugin filename="gz-sim-odometry-publisher-system" '
                  'name="gz::sim::systems::OdometryPublisher">')
    w(indent + 4, '<odom_topic>%s</odom_topic>' % A.TOPIC_ODOM)
    w(indent + 4, '<tf_topic>%s</tf_topic>' % A.GZ_TOPIC_ODOM_TF)
    w(indent + 4, '<odom_frame>odom</odom_frame>')
    w(indent + 4, '<robot_base_frame>base_link</robot_base_frame>')
    w(indent + 4, '<dimensions>3</dimensions>')
    w(indent + 4, '<odom_publish_frequency>50</odom_publish_frequency>')
    w(indent + 4, '<!-- cancel any model rotation so the TF base frame stays')
    w(indent + 4, '     upright (derived from ROBOT_MODEL_RPY) -->')
    w(indent + 4, '<rpy_offset>%s</rpy_offset>' % vec(inverse_rpy(A.ROBOT_MODEL_RPY)))
    w(indent + 2, '</plugin>')
    w(indent, '</model>')


# --------------------------------------------------------------------------- world
def world_xml(w, urdf, view=False, arena_mesh=True):
    """Full world SDF.

    ``view``         add PointCloud GUI panels for both LiDAR topics; the robot
                     itself is always rendered from the URDF meshes
    ``arena_mesh``   draw the official STEP tessellation instead of the boxes
    """
    w(0, '<?xml version="1.0" encoding="UTF-8"?>')
    w(0, '<!-- generated by sentry_gazebo/sdf_builder.py, do not edit -->')
    w(0, '<sdf version="1.9">')
    w(2, '<world name="rmul_2027">')

    w(4, '<physics name="sim" type="ignored">')
    w(6, '<max_step_size>0.004</max_step_size>')
    w(6, '<real_time_factor>1.0</real_time_factor>')
    w(6, '<real_time_update_rate>250</real_time_update_rate>')
    w(4, '</physics>')
    w(4, '<gravity>0 0 -9.81</gravity>')
    w(4, '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>')
    w(4, '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>')
    w(4, '<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>')
    w(4, '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">')
    w(6, '<render_engine>ogre2</render_engine>')
    w(6, '<disable_on_drained_battery>false</disable_on_drained_battery>')
    w(4, '</plugin>')
    w(4, '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>')

    w(4, '<scene>')
    w(6, '<ambient>0.65 0.65 0.65 1</ambient>')
    w(6, '<background>0.16 0.17 0.19 1</background>')
    w(6, '<shadows>true</shadows>')
    w(6, '<grid>false</grid>')
    w(4, '</scene>')

    w(4, '<light name="sun" type="directional">')
    w(6, '<pose>6 4 12 0 0 0</pose>')
    w(6, '<cast_shadows>true</cast_shadows>')
    w(6, '<diffuse>0.85 0.85 0.85 1</diffuse>')
    w(6, '<specular>0.08 0.08 0.08 1</specular>')
    w(6, '<direction>-0.35 0.2 -1</direction>')
    w(4, '</light>')
    w(4, '<light name="fill" type="point">')
    w(6, '<pose>6 4 7 0 0 0</pose>')
    w(6, '<diffuse>0.35 0.35 0.38 1</diffuse>')
    w(6, '<attenuation><range>40</range><constant>0.9</constant><linear>0.01</linear>'
         '<quadratic>0.0</quadratic></attenuation>')
    w(4, '</light>')

    # ---------------------------------------------------------------- GUI
    w(4, '<gui fullscreen="false">')
    w(6, '<plugin filename="MinimalScene" name="3D View">')
    w(8, '<engine>ogre2</engine>')
    w(8, '<scene>scene</scene>')
    w(8, '<!-- looking from the red spawn corner across the field -->')
    w(8, '<camera_pose>-1.6 -1.9 4.2 0 0.42 0.9</camera_pose>')
    w(6, '</plugin>')
    w(6, '<plugin filename="GzSceneManager" name="Scene Manager"/>')
    w(6, '<plugin filename="InteractiveViewControl" name="Interactive view control"/>')
    w(6, '<plugin filename="CameraTracking" name="Camera tracking"/>')
    w(6, '<plugin filename="SelectEntities" name="Select entities"/>')
    w(6, '<plugin filename="EntityTree" name="Entity tree"/>')
    w(6, '<plugin filename="ComponentInspector" name="Component inspector"/>')
    w(6, '<plugin filename="WorldControl" name="World control">')
    w(8, '<play_pause>true</play_pause>')
    w(8, '<step>true</step>')
    w(8, '<start_paused>false</start_paused>')
    w(6, '</plugin>')
    w(6, '<plugin filename="WorldStats" name="World stats">')
    w(8, '<sim_time>true</sim_time>')
    w(8, '<real_time_factor>true</real_time_factor>')
    w(6, '</plugin>')
    if view:
        # gz-gui's PointCloud plugin reads only <point_cloud_topic> and
        # <float_v_topic>.  With no float_v topic it compares an empty Float_V
        # against the cloud on every frame and logs "Float message and
        # pointcloud are not of the same size, visualization may not be
        # accurate" (PointCloud.cc, PublishMarkers).  Points are still drawn,
        # in the plugin's default colour; ROS-side data is unaffected.
        w(6, '<!-- live point clouds of both LiDARs (gz.msgs.PointCloudPacked) -->')
        for name, topic in (('MID-360', A.GZ_TOPIC_MID360_CLOUD),
                            ('Unitree L2', A.GZ_TOPIC_L2_CLOUD)):
            w(6, '<plugin filename="PointCloud" name="%s">' % name)
            w(8, '<point_cloud_topic>%s</point_cloud_topic>' % topic)
            w(6, '</plugin>')
    w(4, '</gui>')

    # arena and robot are inlined so the world file is self-contained apart
    # from the two mesh directories listed in GZ_SIM_RESOURCE_PATH
    arena_model_xml(w, 4, with_visual_mesh=arena_mesh)
    robot_model_xml(w, 4, urdf)
    w(2, '</world>')
    w(0, '</sdf>')


# --------------------------------------------------------------------------- CLI
def build(urdf_path, view=False, arena_mesh=True):
    """Return the complete world SDF for ``urdf_path``."""
    urdf = Urdf(urdf_path)
    w = W()
    world_xml(w, urdf, view=view, arena_mesh=arena_mesh)
    return w.text()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--urdf', required=True, help='sentry_robot.urdf (sensor mount source)')
    ap.add_argument('--output', help='write here instead of stdout')
    ap.add_argument('--view', action='store_true', help='add PointCloud GUI panels')
    ap.add_argument('--no-arena-mesh', action='store_true',
                    help='draw the arena from primitives instead of the STEP mesh')
    args = ap.parse_args(argv)

    text = build(args.urdf, view=args.view, arena_mesh=not args.no_arena_mesh)
    if args.output:
        Path(args.output).write_text(text, encoding='utf-8')
    else:
        sys.stdout.write(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
