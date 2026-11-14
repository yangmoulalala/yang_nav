#!/usr/bin/env python3
"""Exercise the installed navigation stack without Gazebo or a real robot.

Uses an isolated ROS domain and temporary terrain package. Verifies lifecycle
activation, percentage speed limiting, filtered hits, costmap marking/clearing,
collision slowdown and observation-loss stopping. No localization substitution
is made in the normal navigation launch; this process supplies test data only.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from localization_interfaces.msg import FrontendFrame
from nav2_msgs.msg import Costmap, SpeedLimit
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS / 'src/nav_terrain/scripts'))
from terrain_map import TerrainMap
from terrain_export import build_parser, export


class Probe(Node):
    def __init__(self):
        super().__init__('navigation_framework_probe')
        self.broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.rotation.w = 1.
        self.broadcaster.sendTransform(transform)
        self.frame_pub = self.create_publisher(FrontendFrame, '/frontend/frame', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/localization/odometry', 5)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 5)
        self.outputs = []
        self.hit_count = None
        self.speed = None
        self.costmap = None
        self.create_subscription(Twist, '/cmd_vel', self.output, 10)
        self.create_subscription(PointCloud2, '/nav/obstacles', self.cloud, qos_profile_sensor_data)
        self.create_subscription(SpeedLimit, '/speed_limit', self.limit, 10)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw', self.cost,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.obstacle = False
        self.obstacle_distance = 1.2
        self.send_observations = True
        self.send_commands = False
        self.create_timer(.05, self.publish)

    def output(self, msg):
        self.outputs.append((time.monotonic(), math.hypot(msg.linear.x, msg.linear.y)))
        self.outputs = self.outputs[-200:]

    def cloud(self, msg):
        self.hit_count = msg.width * msg.height

    def limit(self, msg):
        self.speed = msg

    def cost(self, msg):
        self.costmap = msg

    def obstacle_cost(self):
        """Highest cost in the cells touching (1.2, 0).

        The probe obstacle sits exactly on a cell corner and the costmap
        resolution is not exactly 0.025 in float32, so a single truncated
        index can land on the neighbouring inflation cell instead of the
        marked one.
        """
        m = self.costmap
        if m is None:
            return None
        fx = (1.2 - m.metadata.origin.position.x) / m.metadata.resolution
        fy = (0.0 - m.metadata.origin.position.y) / m.metadata.resolution
        costs = []
        for x in (math.floor(fx), math.ceil(fx)):
            for y in (math.floor(fy), math.ceil(fy)):
                if 0 <= x < m.metadata.size_x and 0 <= y < m.metadata.size_y:
                    costs.append(m.data[y * m.metadata.size_x + x])
        return max(costs) if costs else None

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry(header=Header(stamp=stamp, frame_id='map'), child_frame_id='base_link')
        odom.pose.pose.orientation.w = 1.
        self.odom_pub.publish(odom)
        if self.send_observations:
            points = [[2., 0., 0.], [2., 0., .25]]  # ground and tunnel roof must be removed
            if self.obstacle:
                points.append([self.obstacle_distance, 0., .15])
            frame = FrontendFrame()
            frame.odometry = odom
            frame.lidar_updated = True
            frame.cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=stamp, frame_id='base_link'), points)
            self.frame_pub.publish(frame)
        if self.send_commands:
            cmd = Twist()
            cmd.linear.x = 1.2
            self.cmd_pub.publish(cmd)

    def wait_for(self, predicate, seconds=15.):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)
            if predicate():
                return
        raise AssertionError('condition timed out')

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', default='/tmp/nav_framework_smoke')
    args = ap.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '85')
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    terrain = out / 'terrain'
    m = TerrainMap.blank(200, 200, .05, (-5., -5.))
    m.elevation = np.zeros(m.terrain.shape, np.float32)
    m.clearance = np.full(m.terrain.shape, .25, np.float32)
    cfg = build_parser().parse_args(['fixture', '--out', str(terrain)])
    cfg.src, cfg.speed_limits = 'framework smoke fixture', {'flat': 50}
    export(m, terrain, cfg)
    checks = []
    rclpy.init()
    probe = Probe()
    process = None
    try:
        with (out / 'navigation.log').open('w') as log:
            process = subprocess.Popen([
                'ros2', 'launch', 'nav_bringup', 'navigation.launch.py',
                'terrain_dir:=' + str(terrain), 'use_sim_time:=false', 'require_trusted:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            probe.wait_for(lambda: 'Managed nodes are active' in
                           (out / 'navigation.log').read_text(errors='replace'), 45.)
            checks.append('all navigation lifecycle nodes active')
            # bt_navigator loads the tree on activate: a missing BT node (ZoneRecovery, ...) or a bad
            # port would fail here instead
            log_text = (out / 'navigation.log').read_text(errors='replace')
            for bad in ('Exception when loading BT', 'Error loading XML file'):
                assert bad not in log_text, bad
            checks.append('navigate_to_pose.xml loaded (ZoneRecovery ladder registered)')
            probe.wait_for(lambda: probe.hit_count == 0 and probe.speed is not None)
            assert probe.speed.percentage and abs(probe.speed.speed_limit - 50) < 1e-6
            checks.append('ground/ceiling filtering and SpeedFilter percentage mask')
            probe.send_commands = True
            probe.wait_for(lambda: len(probe.outputs) > 20 and probe.outputs[-1][1] > .7)
            probe.spin_for(.5)
            samples = [v for t, v in probe.outputs if t > time.monotonic() - .4]
            assert samples and max(samples) <= .751, samples
            checks.append('final velocity capped at 0.75 m/s for 50% mask')
            probe.obstacle = True
            probe.wait_for(lambda: probe.hit_count == 1 and probe.obstacle_cost() == 254)
            checks.append('live obstacle marks local costmap lethal')
            # Move the hit nearer to test the collision monitor independently of
            # planner/controller actions (the probe is supplying raw commands).
            probe.obstacle_distance = .55
            probe.spin_for(.8)
            samples = [v for t, v in probe.outputs if t > time.monotonic() - .3]
            assert samples and max(samples) < .4, samples
            checks.append('collision monitor slows a fresh command near a live obstacle')
            probe.obstacle = False
            probe.wait_for(lambda: probe.hit_count == 0 and probe.obstacle_cost() == 0)
            checks.append('observed free ground clears disappeared obstacle')
            probe.send_observations = False
            probe.spin_for(1.2)
            samples = [v for t, v in probe.outputs if t > time.monotonic() - .3]
            assert samples and max(samples) < 1e-6, samples
            checks.append('lost observation stops output despite fresh velocity commands')
        result = {'passed': True, 'checks': checks}
        print(json.dumps(result, indent=2), flush=True)
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        return 0
    except Exception as exc:
        result = {'passed': False, 'checks': checks, 'error': str(exc)}
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
        return 1
    finally:
        probe.send_commands = False
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=8)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
