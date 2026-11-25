#!/usr/bin/env python3
"""Shared hardware/simulation XY-only acceptance, isolated ROS domain, no hardware.

The fixture integrates final body X/Y. It supplies a fixed body heading, never
executes angular.z, and has no chassis serial node. Tests the installed plugin,
shared configuration, speed envelope, position-only completion, and preemption.
"""
import argparse
import os
from pathlib import Path
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from support.plant import Plant
from support.runtime import finish, navigation
from support.terrain import flat_fixture
from dynamic_accept import clearance


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default='/tmp/nav_framework_accept')
    ap.add_argument('--profile', choices=('sim', 'real'), default='sim')
    args = ap.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '158')
    out = Path(args.out).resolve(); out.mkdir(parents=True, exist_ok=True)
    terrain = flat_fixture(out, 'XY-only chassis fixture', 100)
    rclpy.init(); plant = Plant(out)
    checks = []
    try:
        with navigation(out, terrain, plant, profile=args.profile):
            text = (out / 'navigation.log').read_text(errors='replace')
            assert 'TranslationMPPI: controls reference X/Y' in text, text[-4000:]
            checks.append('shared navigation loads translation MPPI and lifecycle activates')
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
            # Allow time for a detour around the swept physical envelope.
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
