"""Regression checks for the live MID-360 / CAD frame mix-up (2026-11-23)."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

def mul(a,b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def vec(a,v):
    return [sum(a[i][k]*v[k] for k in range(3)) for i in range(3)]

def rpy(values):
    r,p,y=map(float,values.split());cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]

class SensorFrames(unittest.TestCase):
    def setUp(self):
        self.robot=ET.parse(ROOT/'urdf/sentry_robot.urdf').getroot()
        self.yaw=self.robot.find("joint[@name='yaw_joint']")
        self.mount=self.robot.find("joint[@name='mid360_joint']")
        self.R_yaw=rpy(self.yaw.find('origin').get('rpy'))
        self.R_mount=rpy(self.mount.find('origin').get('rpy'))
        self.R_sensor=mul(self.R_yaw,self.R_mount)

    def test_live_gravity_is_up_in_chassis(self):
        # Raw measured specific force in the Livox IMU, not a synthesized pose.
        g=vec(self.R_sensor,[0.012422,-0.262540,0.956357])
        angle=math.degrees(math.atan2(math.hypot(g[0],g[1]),g[2]))
        self.assertLess(angle,2.0,'Livox data axes must not tip the chassis by 120 degrees')

    def test_installed_heading_and_roll(self):
        # Physical mounting: yaw 180 deg, then roll -15 deg about local X.
        # Gravity alone cannot distinguish this from the old yaw=0 mount.
        self.assertLess(math.dist(vec(self.R_sensor,[1,0,0]),[-1,0,0]),1e-6)
        expected_y=[0,-math.cos(math.pi/12),-math.sin(math.pi/12)]
        self.assertLess(math.dist(vec(self.R_sensor,[0,1,0]),expected_y),1e-6)

    def test_body_heading_from_level_sensor_pose(self):
        # A live IMU with roll=-15 deg and map yaw=0 implies a rear-facing
        # chassis (+/-180 deg), not a chassis with yaw=0.
        sensor_in_map=rpy(f'{-math.pi/12} 0 0')
        sensor_from_body=[list(row) for row in zip(*self.R_sensor)]
        body_in_map=mul(sensor_in_map,sensor_from_body)
        self.assertLess(math.dist(vec(body_in_map,[1,0,0]),[-1,0,0]),1e-6)
        self.assertLess(math.dist(vec(body_in_map,[0,0,1]),[0,0,1]),1e-6)

    def test_mesh_world_placement_is_preserved(self):
        old=mul(self.R_yaw,rpy('-2.879793 -1.570796 0'))
        for kind in ('visual','collision'):
            local=self.robot.find(f"link[@name='mid360_link']/{kind}/origin")
            current=mul(self.R_sensor,rpy(local.get('rpy')))
            self.assertLess(max(abs(current[i][j]-old[i][j]) for i in range(3) for j in range(3)),1e-6)
        self.assertEqual(self.mount.find('origin').get('xyz'),'0.163595 -0.131644 0.055760')

    def test_turret_axis_and_factory_imu_are_preserved(self):
        axis=vec(self.R_yaw,list(map(float,self.yaw.find('axis').get('xyz').split())))
        self.assertLess(math.dist(axis,[0,0,-1]),1e-6)
        imu=self.robot.find("joint[@name='mid360_imu_joint']")
        self.assertEqual(imu.find('parent').get('link'),'mid360_link')
        self.assertEqual(imu.find('origin').get('xyz'),'0.011 0.02329 -0.04412')
        self.assertEqual(imu.find('origin').get('rpy'),'0 0 0')

if __name__=='__main__':unittest.main()
