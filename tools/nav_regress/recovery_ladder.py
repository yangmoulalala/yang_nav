#!/usr/bin/env python3
"""Drive the zone-aware recovery ladder through bt_navigator without Gazebo.

Two goals the fixture cannot plan (far outside the map) are sent in turn:

  1. no zone state  -> the ladder escalates clear -> spin -> back-up -> give up
  2. "in a zone"    -> /nav_zone_state reports traverse; the ladder may only clear, then gives up

The zone phase is published by this process exactly as the controller does it ("traverse tunnel_1
@<stamp s>", transient local). In the fixture nothing else publishes it: the controller only talks
on /nav_zone_state while a path is being followed, and planning fails here.

The ladder is read back from the bt_navigator log lines of nav_behaviors::ZoneRecovery.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from localization_interfaces.msg import FrontendFrame
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster

WS = Path(__file__).resolve().parents[2]
os.environ.setdefault('ROS_DOMAIN_ID', '87')
sys.path.insert(0, str(WS / 'src/nav_terrain/scripts'))
from terrain_map import TerrainMap
from terrain_export import build_parser, export

LADDER = re.compile(r'ZoneRecovery: attempt \d+/\d+ -> (\w+)')
GIVE_UP = re.compile(r'ZoneRecovery: navigation failed')


class Probe(Node):
    def __init__(self):
        super().__init__('recovery_ladder_probe')
        self.broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.
        self.broadcaster.sendTransform(t)
        self.frame_pub = self.create_publisher(FrontendFrame, '/frontend/frame', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/localization/odometry', 5)
        self.zone_pub = self.create_publisher(String, '/nav_zone_state',
                                             rclpy.qos.QoSProfile(depth=100, durability=1))
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.zone = None
        self.create_timer(.05, self.publish)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry(header=Header(stamp=stamp, frame_id='map'), child_frame_id='base_link')
        odom.pose.pose.orientation.w = 1.
        self.odom_pub.publish(odom)
        frame = FrontendFrame()
        frame.odometry = odom
        frame.lidar_updated = True
        frame.cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=stamp, frame_id='base_link'), [[2., 0., 0.], [2., 0., .25]])
        self.frame_pub.publish(frame)
        if self.zone:   # heartbeat, as the controller does (0.5 s)
            msg = String()
            msg.data = '%s @%.3f' % (self.zone, self.get_clock().now().nanoseconds * 1e-9)
            self.zone_pub.publish(msg)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)

    def run_goal(self, x, timeout):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.orientation.w = 1.
        send = self.client.send_goal_async(goal)
        end = time.monotonic() + 20
        while not send.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)
        handle = send.result()
        if handle is None or not handle.accepted:
            return None
        result = handle.get_result_async()
        end = time.monotonic() + timeout
        while not result.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)
        return result.result().status if result.done() else None


def ladder_lines(text):
    return [LADDER.search(ln).group(1) for ln in text.splitlines() if LADDER.search(ln)]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default='/tmp/nav_recovery_ladder')
    ap.add_argument('--timeout', type=float, default=90.)
    args = ap.parse_args()
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    terrain = out / 'terrain'
    m = TerrainMap.blank(200, 200, .05, (-5., -5.))
    m.elevation = np.zeros(m.terrain.shape, np.float32)
    m.clearance = np.full(m.terrain.shape, .25, np.float32)
    cfg = build_parser().parse_args(['fixture', '--out', str(terrain)])
    cfg.src, cfg.speed_limits = 'recovery ladder fixture', {'flat': 100}
    export(m, terrain, cfg)

    log_path = out / 'navigation.log'
    checks = []
    rclpy.init()
    probe = Probe()
    proc = None
    try:
        with log_path.open('w') as log:
            proc = subprocess.Popen([
                'ros2', 'launch', 'nav_bringup', 'navigation.launch.py',
                'terrain_dir:=' + str(terrain), 'use_sim_time:=false', 'require_trusted:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            end = time.monotonic() + 60
            while time.monotonic() < end:
                rclpy.spin_once(probe, timeout_sec=.05)
                if 'Managed nodes are active' in log_path.read_text(errors='replace'):
                    break
            else:
                raise AssertionError('navigation stack never activated')
            if not probe.client.wait_for_server(timeout_sec=15.):
                raise AssertionError('no navigate_to_pose action server')

            status = probe.run_goal(40.0, args.timeout)
            assert status is not None, 'goal never finished'
            first = ladder_lines(log_path.read_text(errors='replace'))
            assert first[:3] == ['clear', 'spin', 'backup'], first
            assert first[-1] == 'backup', first
            assert GIVE_UP.search(log_path.read_text(errors='replace')), 'no give-up after the retries'
            checks.append('ladder escalates clear -> spin -> back-up and gives up: ' + ' '.join(first))

            # inside a zone: the same failing goal, but the phase says traverse
            probe.zone = 'traverse tunnel_1'
            probe.spin_for(1.0)
            mark = len(ladder_lines(log_path.read_text(errors='replace')))
            status = probe.run_goal(40.0, args.timeout)
            assert status is not None, 'second goal never finished'
            second = ladder_lines(log_path.read_text(errors='replace'))[mark:]
            assert second, 'no recovery in the second goal'
            assert set(second) == {'clear'}, second
            checks.append('inside a zone only clears the costmaps: ' + ' '.join(second))
            probe.zone = None
        result = {'passed': True, 'checks': checks}
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
        return 0
    except Exception as exc:
        result = {'passed': False, 'checks': checks, 'error': str(exc)}
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
        return 1
    finally:
        if proc is not None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
