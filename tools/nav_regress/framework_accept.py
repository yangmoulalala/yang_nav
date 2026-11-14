#!/usr/bin/env python3
"""Closed-loop acceptance of the real Nav2 stack against a deterministic omni plant.

Run in a dedicated ROS domain. The plant integrates ONLY final /cmd_vel and
publishes fixture TF/odometry/frontend frames; production localization is unchanged.
This checks software contracts, not wheel slip, pitch, sensor accuracy or Gazebo physics.
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
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from localization_interfaces.msg import FrontendFrame
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import TransformBroadcaster

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS / 'src/nav_terrain/scripts'))
from terrain_map import TerrainMap, TUNNEL, SLOPE, OBSTACLE
from terrain_export import build_parser, export


def angle(v):
    return math.atan2(math.sin(v), math.cos(v))


class Plant(Node):
    def __init__(self, out):
        super().__init__('nav_acceptance_plant')
        self.pose = np.array([-3., -2., 0.])
        self.command = Twist()
        self.command_time = 0.
        self.last_time = time.monotonic()
        self.observations = True
        self.trusted = True
        self.blocker = None
        self.phase = 'none'
        self.phase_history = []
        self.mode_history = []
        self.samples = []
        self.tf = TransformBroadcaster(self)
        self.frames = self.create_publisher(FrontendFrame, '/frontend/frame', qos_profile_sensor_data)
        self.odometry = self.create_publisher(Odometry, '/localization/odometry', 5)
        self.state = self.create_publisher(String, '/backend_node/localization_state',
                                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(String, '/nav_zone_state', self.on_phase,
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/nav_mode', lambda m: self.mode_history.append(m.data),
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.single = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.multi = ActionClient(self, NavigateThroughPoses, '/navigate_through_poses')
        self.create_timer(.02, self.step)
        self.trace = (out / 'trace.jsonl').open('w')
        self.frame_tick = 0

    def on_command(self, cmd):
        self.command, self.command_time = cmd, time.monotonic()

    def on_phase(self, msg):
        self.phase = msg.data.split(' ')[0]
        self.phase_history.append(self.phase)

    def step(self):
        now = time.monotonic()
        dt = min(.05, now - self.last_time)
        self.last_time = now
        cmd = self.command if now - self.command_time < .3 else Twist()
        x, y, yaw = self.pose
        mid = yaw + cmd.angular.z * dt / 2
        self.pose += [dt * (math.cos(mid) * cmd.linear.x - math.sin(mid) * cmd.linear.y),
                      dt * (math.sin(mid) * cmd.linear.x + math.cos(mid) * cmd.linear.y),
                      dt * cmd.angular.z]
        self.pose[2] = angle(self.pose[2])
        stamp = self.get_clock().now().to_msg()
        t = TransformStamped(header=Header(stamp=stamp, frame_id='map'), child_frame_id='base_link')
        t.transform.translation.x, t.transform.translation.y = map(float, self.pose[:2])
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.pose[2] / 2), math.cos(self.pose[2] / 2)
        self.tf.sendTransform(t)
        odom = Odometry(header=t.header, child_frame_id='base_link')
        odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, self.pose[:2])
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist = cmd
        self.odometry.publish(odom)
        self.state.publish(String(data='state=NORMAL;pose_trusted=' + str(self.trusted).lower()))
        self.frame_tick += 1
        if self.frame_tick % 3 == 0 and self.observations:
            # Floor observations in all azimuths, plus an optional small box.
            theta = np.linspace(-math.pi, math.pi, 144, endpoint=False)
            points = np.column_stack((3 * np.cos(theta), 3 * np.sin(theta), np.zeros(len(theta))))
            if self.blocker is not None:
                bx, by = self.blocker
                world = np.array([[bx + dx, by + dy, .15] for dx in (-.05, 0, .05)
                                  for dy in (-.05, 0, .05)])
                world[:, :2] -= self.pose[:2]
                cy, sy = math.cos(self.pose[2]), math.sin(self.pose[2])
                world[:, :2] = world[:, :2] @ np.array([[cy, -sy], [sy, cy]])
                points = np.vstack((points, world))
            frame = FrontendFrame()
            frame.odometry = odom
            frame.lidar_updated = True
            frame.cloud = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='base_link'), points)
            self.frames.publish(frame)
        sample = dict(time=now, pose=self.pose.tolist(),
                      cmd=[cmd.linear.x, cmd.linear.y, cmd.angular.z], phase=self.phase,
                      blocker=self.blocker)
        self.samples.append(sample)
        self.trace.write(json.dumps(sample) + '\n')

    def wait(self, predicate, timeout=30., message='condition timed out'):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)
            if predicate():
                return
        raise AssertionError(message + f'; pose={self.pose}, phase={self.phase}')

    def run_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)

    def pose_msg(self, pose):
        msg = PoseStamped(header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'))
        msg.pose.position.x, msg.pose.position.y = float(pose[0]), float(pose[1])
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        return msg

    def start(self, goal, through=False):
        client = self.multi if through else self.single
        self.wait(lambda: client.server_is_ready(), 15., 'action server unavailable')
        request = NavigateThroughPoses.Goal() if through else NavigateToPose.Goal()
        if through:
            request.poses = [self.pose_msg(p) for p in goal]
        else:
            request.pose = self.pose_msg(goal)
        future = client.send_goal_async(request)
        self.wait(future.done, 10., 'goal not acknowledged')
        handle = future.result()
        assert handle is not None and handle.accepted, 'goal rejected'
        return handle, handle.get_result_async()

    def finish(self, future, goal, timeout=45.):
        self.wait(future.done, timeout, 'navigation did not finish')
        result = future.result()
        assert result.status == GoalStatus.STATUS_SUCCEEDED, str(result)
        assert np.linalg.norm(self.pose[:2] - goal[:2]) <= .18, (self.pose, goal)
        assert abs(angle(self.pose[2] - goal[2])) <= .32, (self.pose, goal)

    def goal(self, target, timeout=45.):
        handle, future = self.start(target)
        self.finish(future, target, timeout)
        return handle

    def stopped(self):
        now = time.monotonic()
        recent = [s for s in self.samples if s['time'] >= now - .25]
        return len(recent) >= 5 and max(np.linalg.norm(s['cmd']) for s in recent) < 1e-6

    def cancel(self, handle, future):
        requested = time.monotonic()
        initial = [self.command.linear.x, self.command.linear.y, self.command.angular.z]
        cancel = handle.cancel_goal_async()
        self.wait(cancel.done, 5., 'cancel not acknowledged')
        self.wait(future.done, 5., 'canceled navigation still running')
        assert future.result().status == GoalStatus.STATUS_CANCELED
        # Production smoothing brakes at 2 m/s² and 4 rad/s². Check the
        # observed zero-command window within that bound instead of sampling
        # one fixed instant while the timer/DDS callbacks may be catching up.
        settle = max(.8, max(abs(initial[0]) / 2., abs(initial[1]) / 2., abs(initial[2]) / 4.) + .35)
        self.wait(self.stopped, settle, 'cancel did not reach a stable zero output window')
        print(f'CANCEL_STOP initial={initial} elapsed={time.monotonic()-requested:.3f}s', flush=True)


def fixture(out):
    terrain = out / 'terrain'
    m = TerrainMap.blank(280, 240, .05, (-7., -6.))
    m.elevation = np.zeros(m.terrain.shape, np.float32)
    m.clearance = np.full(m.terrain.shape, np.inf, np.float32)
    yy, xx = np.indices(m.terrain.shape)
    x, y = -7 + (xx + .5) * .05, -6 + (yy + .5) * .05
    tunnel = (np.abs(x) < .5) & (np.abs(y) < .5)
    # Only these two openings connect the left and right halves. A planner
    # detour must not accidentally turn a traversal check into an open-field run.
    wall = (np.abs(x) < .6) & (np.abs(y) >= .5) & (np.abs(y - 2.5) >= .7)
    slope = (np.abs(x) < .6) & (np.abs(y - 2.5) < .7)
    m.terrain[tunnel], m.terrain[wall], m.terrain[slope] = TUNNEL, OBSTACLE, SLOPE
    m.clearance[tunnel] = .25
    cfg = build_parser().parse_args(['fixture', '--out', str(terrain)])
    cfg.src, cfg.speed_limits, cfg.terrain_cost = 'closed-loop Nav2 fixture', {'tunnel': 35, 'slope': 60}, 0.
    export(m, terrain, cfg)
    return terrain


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
    checks, process = [], None

    def passed(name):
        checks.append(name)
        print('PASS ' + name, flush=True)

    try:
        with (out / 'navigation.log').open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'nav_bringup', 'navigation.launch.py',
                                        'terrain_dir:=' + str(terrain), 'use_sim_time:=false'],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            plant.wait(lambda: 'Managed nodes are active' in (out / 'navigation.log').read_text(),
                       50., 'navigation lifecycle failed')
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
            plant.wait(future.done, 35., 'unreachable in-zone goal never aborted')
            assert future.result().status == GoalStatus.STATUS_ABORTED
            text = (out / 'navigation.log').read_text()[start_log:]
            assert '-> clear' in text and '-> spin' not in text and '-> backup' not in text, text[-6000:]
            assert np.linalg.norm(plant.pose - before) < .01
            passed('in-zone planning failure after 5 s idle never spins or backs up')
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
        code = 0
    except Exception as exc:
        result, code = dict(passed=False, checks=checks, error=str(exc)), 1
        print('FAIL ' + str(exc), flush=True)
    finally:
        if process is not None:
            for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
                try:
                    os.killpg(process.pid, sig)
                    process.wait(timeout=timeout)
                    break
                except ProcessLookupError:
                    break
                except subprocess.TimeoutExpired:
                    continue
        plant.trace.close()
        plant.destroy_node()
        rclpy.shutdown()
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
