#!/usr/bin/env python3
"""Exercise footprint-clearance hold on a rotating plant, never on a serial chassis."""
import argparse
import math
import os
from pathlib import Path
import re
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from support.plant import Plant
from support.runtime import finish, navigation
from support.terrain import flat_fixture


def logged_at(logfile, message):
    match = re.search(r'\[(\d+\.\d+)\].*' + re.escape(message), logfile.read_text())
    return float(match.group(1)) if match else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--persistent', action='store_true', help='keep the blocker through the hold timeout')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '169'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    terrain = flat_fixture(out, 'rotating XY with transient occupied footprint', 100)
    rclpy.init()
    plant = Plant(out, step_period=.01)
    raw = []
    plant.create_subscription(Twist, '/cmd_vel_nav', lambda msg: raw.append(
        (time.monotonic(), math.hypot(msg.linear.x, msg.linear.y))), 10)
    try:
        with navigation(out, terrain, plant, profile='real') as logfile:
            local_costmap = '/local_costmap/local_costmap'
            parameters = AsyncParameterClient(plant, local_costmap)
            updated = parameters.set_parameters([Parameter('obstacle_layer.footprint_clearing_enabled', value=False)])
            plant.wait(updated.done, 2., 'fixture could not configure footprint clearing')
            assert all(result.successful for result in updated.result().results), updated.result()
            plant.pose[:] = [-1.8, 0., .5]
            plant.passive_yaw_rate = 3.15
            plant.run_for(.7)
            goal = [1.8, 0., 0.]
            _, future = plant.start(goal)
            plant.wait(lambda: plant.pose[0] > -1.45, 8., 'plant did not start')
            plant.blocker = (plant.pose[0] + .05, plant.pose[1] + .35)
            hold_log = 'holding for confirmed footprint clearance'
            plant.wait(lambda: logged_at(logfile, hold_log) is not None,
                       4., 'fixture did not trigger ordinary escape hold')
            hold = time.monotonic()
            hold_at = logged_at(logfile, hold_log)
            plant.run_for(.25)
            if args.persistent:
                timeout_log = 'footprint clearance hold timed out; checking translation escape'
                plant.wait(lambda: logged_at(logfile, timeout_log) is not None,
                           1.5, 'persistent blocker did not time out')
                timeout_delay = logged_at(logfile, timeout_log) - hold_at
                assert .99 <= timeout_delay <= 1.25, timeout_delay
                plant.wait(lambda: 'blocked' in plant.phase_history, 1., 'timeout never handed off to recovery')
                occupied = [speed for stamp, speed in raw if stamp >= hold + .25]
                assert len(occupied) >= 8 and max(occupied) < 1e-6, occupied
                result = dict(passed=True, hold_timeout_s=timeout_delay,
                              zero_commands_during_occupied_window=len(occupied))
            else:
                plant.blocker = None
                removed = time.monotonic()
                plant.run_for(.2)
                clear_map = plant.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
                cleared = clear_map.call_async(ClearEntireCostmap.Request())
                plant.wait(cleared.done, .4, 'fixture could not remove blocker from local costmap')
                release_log = 'path prefix clear; returning to controller'
                plant.wait(lambda: logged_at(logfile, release_log) is not None,
                           3., 'did not leave overlap hold with yaw still spinning')
                release_delay = time.monotonic() - removed
                hold_duration = logged_at(logfile, release_log) - hold_at
                assert hold_duration >= .99, hold_duration
                plant.wait(lambda: any(stamp >= removed and speed > .05 for stamp, speed in raw),
                           2., 'no executable navigation command after release')
                plant.finish(future, goal)
                plant.wait(plant.stopped, 2., 'plant did not stop at the goal')
                result = dict(passed=True, release_after_obstacle_removed_s=release_delay)
            assert plant.passive_yaw_rate == 3.15 and not plant.unexecutable_yaw
    except Exception as exc:
        result = dict(passed=False, error=str(exc))
    finally:
        plant.trace.close()
        plant.destroy_node()
        rclpy.shutdown()
    return finish(out, result)


if __name__ == '__main__':
    raise SystemExit(main())
