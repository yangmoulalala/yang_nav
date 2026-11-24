#!/usr/bin/env python3
"""Closed-loop acceptance of the real Nav2 stack against a deterministic omni plant.

Run in a dedicated ROS domain. The plant integrates ONLY final /cmd_vel and
publishes fixture TF/odometry/frontend frames; production localization is unchanged.
This checks software contracts, not wheel slip, pitch, sensor accuracy or Gazebo physics.
"""
import argparse
import math
import os
from pathlib import Path
import sys

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus

from support.plant import Plant, angle
from support.runtime import finish, navigation
from support.terrain import fixture


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default='/tmp/nav_framework_accept')
    args = ap.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '86')
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    terrain = fixture(out)
    rclpy.init()
    plant = Plant(out)
    checks = []

    def passed(name):
        checks.append(name)
        print('PASS ' + name, flush=True)

    try:
        with navigation(out, terrain, plant):
            passed('all lifecycle nodes activate with production safety gates enabled')
            plant.goal((-2., -2., .8))
            passed('NavigateToPose reaches position and requested heading on flat terrain')
            plant.goal((-3., 0., 0.))
            plant.phase_history.clear()
            begin = len(plant.samples)
            plant.goal((2.5, 0., 0.))
            for phase in ('align', 'traverse', 'exit'):
                assert phase in plant.phase_history, (phase, plant.phase_history)
            body = [s for s in plant.samples[begin:] if abs(s['pose'][0]) < .45 and abs(s['pose'][1]) < .5]
            assert body, 'planner avoided the tunnel; crossing was not tested'
            assert max(abs(s['pose'][1]) for s in body) <= .03, body
            assert max(abs(angle(s['pose'][2])) for s in body) <= math.radians(3), body
            passed('real planner/shaper/controller/BT cross tunnel within 3 cm and 3 degrees')
            plant.goal((0., 0., 0.))
            passed('reverse traversal stops at the exact in-zone goal without handing off to MPPI')
            plant.run_for(5.)  # exceed every old phase-history protection timeout
            start_log = len((out / 'navigation.log').read_text())
            before = plant.pose.copy()
            handle, future = plant.start((30., 0., 0.))
            # Production ZoneRecovery has retries=-1: an unreachable target
            # stays pending until canceled/preempted, while the robot stays still.
            plant.run_for(7.)
            assert not future.done(), 'unlimited recovery unexpectedly terminated'
            text = (out / 'navigation.log').read_text()[start_log:]
            assert '-> clear' in text and '-> spin' not in text and '-> backup' not in text, text[-6000:]
            assert np.linalg.norm(plant.pose - before) < .01
            plant.cancel(handle, future)
            passed('in-zone planning failure stays pending without motion and remains cancelable')
            plant.goal((-2.5, 0., 0.))
            passed('new goal resumes from inside tunnel and exits in reverse')
            handle, future = plant.start((2.5, 0., 0.))
            plant.wait(lambda: plant.pose[0] > -1.9, 20., 'robot never approached tunnel')
            plant.blocker = (-.4, 0.)
            block_begin = len(plant.samples)
            plant.run_for(4.)
            # A fresh plan may detour through the other opening; both safe waiting
            # and safe rerouting are valid responses to a blocked tunnel.
            for sample in plant.samples[block_begin:]:
                px, py, yaw = sample['pose']
                dx, dy = -.4-px, -py
                bx = math.cos(yaw)*dx + math.sin(yaw)*dy
                by = -math.sin(yaw)*dx + math.cos(yaw)*dy
                assert not (-.38 < bx < .46 and abs(by) < .38), sample
            plant.blocker = None
            plant.finish(future, (2.5, 0., 0.))
            passed('live obstacle prevents collision, safe wait or reroute resumes after clearance')
            handle, future = plant.start((-2.5, 0., 0.))
            plant.wait(lambda: plant.pose[0] < .2, 25., 'robot never entered cancellation window')
            plant.cancel(handle, future)
            passed('cancel inside a zone terminates action and sends zero velocity')
            plant.goal((2.5, 0., 0.))
            passed('fresh goal after cancellation resumes safely from the zone')
            old_handle, old_future = plant.start((-2.5, 0., 0.))
            plant.wait(lambda: plant.pose[0] < .25, 25., 'preemption did not reach zone')
            new_handle, new_future = plant.start((2.5, 0., 0.))
            plant.finish(new_future, (2.5, 0., 0.))
            plant.wait(old_future.done, 5., 'preempted goal remains active')
            assert old_future.result().status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED)
            passed('new target preempts an active crossing and completes with a fresh shaped path')
            handle, future = plant.start((3.5, -2., 0.))
            plant.run_for(1.)
            plant.observations = False
            plant.run_for(1.)
            assert plant.stopped(), 'sensor outage did not hold navigation output'
            plant.observations = True
            plant.run_for(.4)
            plant.trusted = False
            plant.run_for(.5)
            assert plant.stopped(), 'untrusted localization did not hold output'
            plant.trusted = True
            plant.finish(future, (3.5, -2., 0.))
            passed('observation loss and untrusted localization hold motion, then navigation resumes')
            plant.goal((-3., 2.5, 0.))
            plant.mode_history.clear()
            handle, future = plant.start([(-1.5, 2.5, 0.), (2.5, 2.5, 0.)], through=True)
            plant.finish(future, (2.5, 2.5, 0.), 60.)
            assert 'ramp' in plant.mode_history, plant.mode_history
            passed('NavigateThroughPoses uses shaped directional terrain and reaches final goal')
            plant.goal(tuple(plant.pose))
            passed('an already-reached target succeeds without entering recovery')
        result = dict(passed=True, checks=checks, model='deterministic omni kinematics; real Nav2 nodes')
    except Exception as exc:
        result = dict(passed=False, checks=checks, error=str(exc))
        print('FAIL ' + str(exc), flush=True)
    finally:
        plant.trace.close()
        plant.destroy_node()
        rclpy.shutdown()
    return finish(out, result)


if __name__ == '__main__':
    sys.exit(main())
