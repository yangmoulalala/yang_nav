#!/usr/bin/env python3
"""Send one NavigateToPose goal (world/field coords converted to map) and log progress.

    goal.py X Y [YAW_DEG] [--frame world|map] [--timeout S] [--nav-dir DIR]

World coordinates are the Gazebo field frame; they are converted with the registration
stored in <nav_dir>/terrain.json.  Prints the robot pose (map -> base_link) every second
and the final result; exit code 0 on success.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import terrain_path as tp  # noqa: E402


def world_to_map(x, y, yaw, reg):
    tx, ty, _ = reg['t_world_map']
    a = math.radians(reg['rpy_deg'][2])
    dx, dy = x - tx, y - ty
    return (math.cos(a) * dx + math.sin(a) * dy, -math.sin(a) * dx + math.cos(a) * dy, yaw - a)


def map_to_world(x, y, reg):
    tx, ty, _ = reg['t_world_map']
    a = math.radians(reg['rpy_deg'][2])
    return (math.cos(a) * x - math.sin(a) * y + tx, math.sin(a) * x + math.cos(a) * y + ty)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('yaw', type=float, nargs='?', default=0.0)
    ap.add_argument('--frame', choices=['world', 'map'], default='world')
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--nav-dir', default=str(tp.default_terrain_dir()))
    args = ap.parse_args()
    reg = json.load(open(os.path.join(args.nav_dir, 'terrain.json')))['registration']
    yaw = math.radians(args.yaw)
    gx, gy, gyaw = (world_to_map(args.x, args.y, yaw, reg) if args.frame == 'world'
                    else (args.x, args.y, yaw))

    rclpy.init()
    node = Node('nav_goal_test', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    tf = Buffer()
    TransformListener(tf, node)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=60.0):
        print('navigate_to_pose not available'); return 2
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
    print('goal map (%.2f, %.2f, %.0f deg)' % (gx, gy, math.degrees(gyaw)), flush=True)
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        print('goal rejected'); return 3
    res_fut = handle.get_result_async()
    t0 = time.monotonic(); last = 0.0
    while not res_fut.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        el = time.monotonic() - t0
        if el - last >= 1.0:
            last = el
            try:
                t = tf.lookup_transform('map', 'base_link', Time())
                p = t.transform.translation
                wx, wy = map_to_world(p.x, p.y, reg)
                print('t=%5.1f map (%6.2f, %6.2f, z %5.2f)  world (%6.2f, %6.2f)  dist %.2f'
                      % (el, p.x, p.y, p.z, wx, wy, math.hypot(gx - p.x, gy - p.y)), flush=True)
            except TransformException:
                pass
        if el > args.timeout:
            print('timeout, cancelling'); handle.cancel_goal_async(); time.sleep(1); return 4
    st = res_fut.result().status
    err = getattr(res_fut.result().result, 'error_code', 0)
    print('result status %d (%s) error_code %s, %.1f s' % (
        st, 'SUCCEEDED' if st == GoalStatus.STATUS_SUCCEEDED else 'FAILED', err, time.monotonic() - t0))
    return 0 if st == GoalStatus.STATUS_SUCCEEDED else 1


if __name__ == '__main__':
    sys.exit(main())
