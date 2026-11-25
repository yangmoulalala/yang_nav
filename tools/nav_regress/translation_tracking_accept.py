#!/usr/bin/env python3
"""XY-only short-route tracking with measured response uncertainty, no hardware.

001/003/004 motion windows show cross-axis response and excess lateral speed.
This fixture applies recorded or synthetic responses plus 50 ms delay; it is
an uncertainty test, not a hardware calibration. Optional --costmap-json loads
an extracted OccupancyGrid obstacle snapshot (inflation is regenerated).
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav_terrain.terrain_export import ExportConfig, export
from nav_terrain.terrain_map import OBSTACLE, TerrainMap
from support.runtime import finish, navigation
from support.terrain import flat_fixture
from support.plant import Plant


class UncertainTranslationPlant(Plant):
    def __init__(self, out, scenario='site_a_003'):
        self.scenario = scenario
        self.pending = []
        super().__init__(out)

    def on_command(self, cmd):
        if abs(cmd.angular.z) > 1e-9:
            self.unexecutable_yaw.append(cmd.angular.z)
        actual = copy.deepcopy(cmd)
        x, y = cmd.linear.x, cmd.linear.y
        if self.scenario.startswith('rotation_'):
            angle = math.radians(20 if self.scenario == 'rotation_plus20' else -20)
            c, s = 1.1 * math.cos(angle), 1.1 * math.sin(angle)
            actual.linear.x = c * x - s * y
            actual.linear.y = s * x + c * y
        elif self.scenario == 'site_a_004':
            actual.linear.x = 1.066809 * x + .309086 * y
            actual.linear.y = -.299567 * x + 1.045185 * y
        else:
            actual.linear.x = .995 * x + .214 * y
            actual.linear.y = -.170 * x + 1.150 * y
        actual.angular.z = 0.
        self.pending.append((time.monotonic(), actual))

    def step(self):
        while self.pending and self.pending[0][0] <= time.monotonic() - .05:
            _, actual = self.pending.pop(0)
            super().on_command(actual)
        super().step()


def snapshot_fixture(out, source):
    msg = json.loads(source.read_text())
    info = msg['info']; origin = info['origin']['position']
    grid = np.asarray(msg['data']).reshape(info['height'], info['width'])
    terrain = TerrainMap.blank(info['width'], info['height'], info['resolution'],
                               (origin['x'], origin['y']))
    terrain.terrain[grid == 100] = OBSTACLE
    terrain.terrain[grid < 0] = 255
    terrain.elevation = np.zeros(grid.shape, np.float32)
    terrain.clearance = np.full(grid.shape, .25, np.float32)
    target = out / 'terrain'
    export(terrain, target, ExportConfig(speed_limits={'flat': 100}),
           source='recorded local costmap obstacle snapshot')
    return target


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default='/tmp/nav_translation_tracking_accept')
    ap.add_argument('--costmap-json', type=Path)
    ap.add_argument('--scenario', choices=('site_a_003', 'site_a_004', 'rotation_plus20', 'rotation_minus20'), default='site_a_003',
                    help='recorded response or synthetic +/-20 deg, 1.1 gain; unchanged acceptance limits')
    ap.add_argument('--trials', type=int, default=3)
    args = ap.parse_args()
    if args.trials < 1:
        ap.error('--trials must be positive')
    os.environ.setdefault('ROS_DOMAIN_ID', '158')
    out = Path(args.out).resolve(); out.mkdir(parents=True, exist_ok=True)
    terrain = (snapshot_fixture(out, args.costmap_json) if args.costmap_json else
               flat_fixture(out, 'XY response uncertainty', 100))
    rclpy.init(); plant = UncertainTranslationPlant(out, args.scenario)
    trials = []
    try:
        with navigation(out, terrain, plant, profile='real'):
            text = (out / 'navigation.log').read_text(errors='replace')
            assert 'Critic loaded : mppi::critics::TranslationVelocityCritic' in text
            for index in range(args.trials):
                if args.scenario != 'site_a_003':
                    start = np.array([.67, 1.274]); target = np.array([-1.049796, .839129])
                    heading = math.radians(-138.)
                else:
                    start = np.array([.61, 1.59]); target = np.array([-1.022, 1.329])
                    heading = math.radians(-143.)
                plant.pose[:] = [*start, heading]
                plant.run_for(.6)
                begin = len(plant.samples)
                _, future = plant.start([*target, -2.98])
                plant.wait(future.done, 40., 'uncertain XY response failed to reach goal')
                assert future.result().status == GoalStatus.STATUS_SUCCEEDED
                speed_at_success = math.hypot(plant.command.linear.x, plant.command.linear.y)
                plant.run_for(.6)
                samples = plant.samples[begin:]
                positions = np.array([s['pose'][:2] for s in samples])
                direction = target - start
                cross = direction[0] * (positions[:, 1] - start[1]) - direction[1] * (positions[:, 0] - start[0])
                max_error = float(np.max(np.abs(cross)) / np.linalg.norm(direction))
                final_error = float(np.linalg.norm(plant.pose[:2] - target))
                trial = dict(trial=index, max_cross_track_error=max_error,
                             final_error=final_error, speed_at_success=speed_at_success)
                trials.append(trial)
                print(json.dumps(trial), flush=True)
                assert speed_at_success <= .055, trial
                assert max_error < .20, trial
                assert final_error < .18, trial
                assert abs(plant.pose[2] - heading) < 1e-9
            assert not plant.unexecutable_yaw, plant.unexecutable_yaw
        result = dict(passed=True, scenario=args.scenario, trials=trials,
                      checks=['cross track < 0.20 m with XY coupling and delay',
                              'stopped translation at success and settled goal error < 0.18 m',
                              'fixed heading and zero ordinary navigation yaw'])
    except Exception as exc:
        result = dict(passed=False, scenario=args.scenario, trials=trials, error=str(exc))
    finally:
        plant.destroy_node(); rclpy.shutdown()
    return finish(out, result)


if __name__ == '__main__':
    raise SystemExit(main())
