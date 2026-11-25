#!/usr/bin/env python3
"""Shared recovery contract: no spin, unlimited retries, protected zones and cancel."""
import argparse
import os
from pathlib import Path
import re

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from support.plant import Plant
from support.runtime import finish, navigation
from support.terrain import flat_fixture

LADDER = re.compile(r'ZoneRecovery: attempt \d+/(?:\d+|unlimited) -> (\w+)')


def ladder_lines(text):
    return LADDER.findall(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default='/tmp/nav_recovery_ladder')
    ap.add_argument('--timeout', type=float, default=30.)
    args = ap.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '87')
    out = Path(args.out).resolve(); out.mkdir(parents=True, exist_ok=True)
    terrain = flat_fixture(out, 'recovery contract fixture', 100)
    rclpy.init(); plant = Plant(out)
    zone = plant.create_publisher(String, '/nav_zone_state', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    checks = []
    try:
        with navigation(out, terrain, plant) as log:
            handle, future = plant.start((40., 0., 0.))
            plant.wait(lambda: 'backup' in ladder_lines(log.read_text()), args.timeout,
                       'no translation recovery outside zone')
            first = ladder_lines(log.read_text())
            assert first[0] == 'clear' and 'spin' not in first, first
            assert not future.done(), 'unlimited recovery terminated before cancel'
            plant.cancel(handle, future)
            checks.append('outside zone clears then backs up, never spins, and remains cancelable')
            mark = len(ladder_lines(log.read_text()))

            def publish_zone():
                stamp = plant.get_clock().now().nanoseconds * 1e-9
                zone.publish(String(data=f'traverse tunnel_1 @{stamp:.3f}'))

            timer = plant.create_timer(.1, publish_zone)
            plant.run_for(.5)
            before = plant.pose.copy()
            handle, future = plant.start((40., 0., 0.))
            plant.run_for(7.)
            second = ladder_lines(log.read_text())[mark:]
            assert second and set(second) == {'clear'}, second
            assert not future.done(), 'protected failure terminated before cancel'
            assert max(abs(plant.pose-before)) < .01, (before, plant.pose)
            plant.cancel(handle, future)
            timer.cancel()
            assert not plant.unexecutable_yaw, plant.unexecutable_yaw
            checks.append('protected zone retries only clear, holds position and remains cancelable')
        result = dict(passed=True, checks=checks)
    except Exception as exc:
        result = dict(passed=False, checks=checks, error=str(exc))
    finally:
        plant.trace.close(); plant.destroy_node(); rclpy.shutdown()
    return finish(out, result)


if __name__ == '__main__':
    raise SystemExit(main())
