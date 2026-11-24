#!/usr/bin/env python3
"""Offline real-profile XY-only acceptance, isolated ROS domain, no hardware.

The fixture integrates final body X/Y. It supplies a fixed body heading, never
executes angular.z, and has no chassis serial node. Tests the installed plugin,
real overlay, position-only completion, lateral motion, and preemption.
"""
import argparse
import ast
import math
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from support.plant import Plant
from support.runtime import finish, navigation
from support.terrain import flat_fixture
from dynamic_accept import polygon_clearance


class TranslationPlant(Plant):
    def on_command(self, cmd):
        # Record a failure before feeding the actuator with only supported axes.
        if abs(cmd.angular.z) > 1e-9:
            self.unexecutable_yaw.append(cmd.angular.z)
        cmd.angular.z = 0.0
        super().on_command(cmd)


def clearance(samples, obstacle):
    params = yaml.safe_load((Path(get_package_share_directory('nav_bringup')) /
                             'params/real.yaml').read_text())['local_costmap']['local_costmap']['ros__parameters']
    padding = params['footprint_padding']
    polygon = [(x+math.copysign(padding,x), y+math.copysign(padding,y))
               for x,y in ast.literal_eval(params['footprint'])]
    bx,by = obstacle
    square = [(bx+dx,by+dy) for dx,dy in ((-.05,-.05),(.05,-.05),(.05,.05),(-.05,.05))]
    minimum = float('inf')
    for sample in samples:
        x,y,yaw = sample['pose']; c,s = math.cos(yaw), math.sin(yaw)
        actual = [(x+c*u-s*v,y+s*u+c*v) for u,v in polygon]
        minimum = min(minimum,polygon_clearance(actual,square))
    assert minimum >= -1e-6, f'gimbal swept footprint collision: {minimum}'
    return minimum


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default='/tmp/nav_translation_accept')
    args = ap.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '158')
    out = Path(args.out).resolve(); out.mkdir(parents=True, exist_ok=True)
    terrain = flat_fixture(out, 'XY-only chassis fixture', 100)
    rclpy.init(); plant = TranslationPlant(out); plant.unexecutable_yaw = []
    plant.base_frame = 'gimbal_nav'
    checks = []
    try:
        with navigation(out, terrain, plant, profile='real'):
            text = (out / 'navigation.log').read_text(errors='replace')
            assert 'TranslationMPPI: controls reference X/Y' in text, text[-4000:]
            checks.append('real profile loads translation MPPI and lifecycle activates')
            plant.pose[:] = [-2., 0., .7]
            plant.run_for(.5)
            for target in ([0., 0., -1.5], [0., 1.5, 2.5]):
                handle, future = plant.start(target)
                plant.wait(future.done, 40., 'XY goal did not complete')
                assert future.result().status == GoalStatus.STATUS_SUCCEEDED, future.result()
                assert np.linalg.norm(plant.pose[:2] - target[:2]) < .18, plant.pose
                assert abs(plant.pose[2] - .7) < 1e-9, plant.pose
            checks.append('forward and lateral goals succeed despite incompatible requested yaw')
            handle, future = plant.start([-2., 1.5, -2.])
            plant.run_for(.5)
            new_handle, new_result = plant.start([1., .5, 2.])
            plant.wait(new_result.done, 40., 'preempting XY goal did not complete')
            assert new_result.result().status == GoalStatus.STATUS_SUCCEEDED
            assert np.linalg.norm(plant.pose[:2] - [1., .5]) < .18, plant.pose
            checks.append('new goal preemption remains functional')
            # The turret may turn independently while the physical chassis yaw
            # is unobserved. The MCU integrates XY in the CURRENT turret axes,
            # with passive yaw measured by LIO but no yaw command actuator.
            plant.passive_yaw_rate = .35
            begin = len(plant.samples)
            _, rotating_result = plant.start([2., 1.5, -2.])
            plant.wait(rotating_result.done, 40., 'independently rotating turret goal failed')
            assert rotating_result.result().status == GoalStatus.STATUS_SUCCEEDED
            assert np.linalg.norm(plant.pose[:2] - [2., 1.5]) < .18, plant.pose
            assert abs(plant.pose[2] - .7) > .2, plant.pose
            assert all(abs(s['cmd'][2]) < 1e-9 for s in plant.samples[begin:])
            plant.passive_yaw_rate = 0.
            checks.append('XY navigation succeeds with independently rotating turret (0.35 rad/s)')
            # A persistent obstacle is observed before requesting a crossing.
            plant.blocker = (0., -.7)
            plant.run_for(.5)
            start_sample = len(plant.samples)
            handle, future = plant.start([-2., -2., -2.])
            # Real commissioning limits and the swept envelope make this route
            # longer/slower than the short unobstructed goals above.
            plant.wait(future.done, 80., 'obstacle detour did not complete')
            assert future.result().status == GoalStatus.STATUS_SUCCEEDED
            gap = clearance(plant.samples[start_sample:], plant.blocker)
            checks.append(f'XY-only obstacle detour preserves padded footprint clearance ({gap:.3f} m)')
            plant.blocker = None
            assert not plant.unexecutable_yaw, plant.unexecutable_yaw
            checks.append('all final navigation commands have angular.z=0')
        result = dict(passed=True, checks=checks, final_pose=plant.pose.tolist())
    except Exception as exc:
        result = dict(passed=False, checks=checks, error=str(exc))
    finally:
        plant.destroy_node(); rclpy.shutdown()
    return finish(out, result)


if __name__ == '__main__':
    raise SystemExit(main())
