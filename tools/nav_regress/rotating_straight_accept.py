#!/usr/bin/env python3
"""Check real-profile straight-line performance at recorded yaw rates, without hardware."""
import argparse
import math
import os
from pathlib import Path
import statistics
import time

import rclpy

from support.plant import Plant
from support.runtime import finish, navigation
from support.terrain import flat_fixture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '159')
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    terrain = flat_fixture(out, 'Clear straight-line run with passive yaw', 100)
    rclpy.init()
    plant = Plant(out, step_period=.01)
    trials = []
    try:
        with navigation(out, terrain, plant, profile='real'):
            subscribers = lambda topic: {endpoint.node_name for endpoint in
                                         plant.get_subscriptions_info_by_topic(topic)}
            plant.wait(lambda: 'controller_server' in subscribers('/cmd_vel'), 3.,
                       'controller is not receiving the gated velocity')
            assert 'controller_server' not in subscribers('/cmd_vel_nav'), 'raw velocity feedback loop'
            for rate in (3.15, -3.15):
                plant.pose[:] = [-1.5, 0., .7]
                plant.passive_yaw_rate = rate
                plant.run_for(.5)
                begin = len(plant.samples)
                started = time.monotonic()
                plant.goal([1.5, 0., -2.], timeout=18.)
                duration = time.monotonic() - started
                samples = plant.samples[begin:]
                cruise = [math.hypot(*sample['cmd'][:2]) for sample in samples
                          if -.65 < sample['pose'][0] < .4]
                trial = dict(yaw_rate=rate, duration=duration,
                             cruise_median=statistics.median(cruise),
                             cross_track=max(abs(sample['pose'][1]) for sample in samples),
                             peak=max(math.hypot(*sample['cmd'][:2]) for sample in samples))
                trials.append(trial)
                plant.wait(plant.stopped, 2., 'vehicle did not stop after success')
                assert trial['cruise_median'] > .65, trial
                assert trial['cross_track'] < .20, trial
                assert trial['peak'] <= 1.50 + 1e-6, trial
        result = dict(passed=True, trials=trials)
    except Exception as exc:
        result = dict(passed=False, trials=trials, error=str(exc))
    finally:
        plant.destroy_node()
        rclpy.shutdown()
    return finish(out, result)


if __name__ == '__main__':
    raise SystemExit(main())
