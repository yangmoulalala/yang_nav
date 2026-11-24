"""Observable turret reference with arbitrary independent chassis rotation."""
import importlib.util
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from test_sensor_frames import mul, rpy, vec

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gimbal_description', ROOT/'gimbal_description.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GimbalReference(unittest.TestCase):
    def setUp(self):
        self.source = (ROOT/'urdf/sentry_robot.urdf').read_text()
        self.physical = ET.fromstring(self.source)
        self.real = ET.fromstring(module.gimbal_description(self.source))

    def test_real_tree_has_no_fabricated_chassis_or_dynamic_joints(self):
        links = {link.get('name') for link in self.real.findall('link')}
        self.assertNotIn('base_link', links)
        self.assertIn('gimbal_nav', links)
        self.assertIn('mid360_imu_link', links)
        children = {j.find('child').get('link') for j in self.real.findall('joint')}
        self.assertEqual(links - children, {'gimbal_nav'})
        for j in self.real.findall('joint'):
            self.assertEqual(j.get('type'), 'fixed')
            self.assertIn(j.find('parent').get('link'), links)

    def test_reference_obeys_turret_xy_for_independent_rotations(self):
        # Sensor pose comes from LIO. Remove its fixed installation rotation;
        # recover turret heading even while the chassis takes any other heading.
        mount = rpy(self.physical.find("joint[@name='yaw_joint']/origin").get('rpy'))
        lidar = rpy(self.physical.find("joint[@name='mid360_joint']/origin").get('rpy'))
        sensor = mul(mount, lidar)
        inverse = list(map(list, zip(*sensor)))
        for chassis in (-2.1, 0, 1.7):
            for relative in (-2.8, -.6, 0, 1.3):
                # CAD yaw_joint +Y is chassis -Z.
                turret = chassis - relative
                world_sensor = mul(rpy(f'0 0 {turret}'), sensor)
                world_nav = mul(world_sensor, inverse)
                command = [.4, -.2, 0]
                self.assertLess(math.dist(vec(world_nav, command),
                    vec(rpy(f'0 0 {turret}'), command)), 1e-10)
                # A true chassis-frame command would be wrong by relative.
                if relative:
                    self.assertGreater(math.dist(vec(world_nav, command),
                        vec(rpy(f'0 0 {chassis}'), command)), .1)

    def test_zero_offset_preserves_old_zero_joint_extrinsic(self):
        original = self.physical.find("joint[@name='yaw_joint']/origin")
        generated = self.real.find("joint[@name='gimbal_nav_mount']/origin")
        self.assertEqual(original.get('xyz'), generated.get('xyz'))
        self.assertLess(max(abs(a-b) for a,b in zip(
            map(float,original.get('rpy').split()),map(float,generated.get('rpy').split()))),1e-12)
        for name in ('mid360_joint','mid360_imu_joint','l2_joint'):
            self.assertEqual(ET.tostring(self.real.find(f"joint[@name='{name}']")),
                             ET.tostring(self.physical.find(f"joint[@name='{name}']")))

    def test_calibrated_offset_is_separate_from_joint_angle(self):
        root = ET.fromstring(module.gimbal_description(self.source, .23))
        mount = rpy(root.find("joint[@name='gimbal_nav_mount']/origin").get('rpy'))
        zero = rpy(self.real.find("joint[@name='gimbal_nav_mount']/origin").get('rpy'))
        self.assertLess(math.dist(vec(mount,[0,0,1]),
                                 vec(mul(rpy('0 0 -.23'),zero),[0,0,1])),1e-12)
        for invalid in (float('nan'),float('inf')):
            with self.assertRaises(ValueError): module.gimbal_description(self.source,invalid)


if __name__ == '__main__': unittest.main()
