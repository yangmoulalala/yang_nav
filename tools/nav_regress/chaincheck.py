#!/usr/bin/env python3
"""Plan every acceptance leg and check that it really goes through its tunnel (no simulator).

    chaincheck.py [--terrain-dir maps/sim] [--order chain|pair] [--no-staging] [--spawn X Y]

Needs planner_server and a map -> base_link TF (chaincheck.sh provides both).  The legs are the goals
of tools/nav_regress/tunnel_accept.py, in the same order.  With staging (default) a crossing is two
goals - the stage pose just beyond the near mouth, then the goal pose beyond the far one - so the leg
that has to cross is stage -> goal.

A leg whose path goes around the wall instead of through the opening still succeeds in Gazebo, but it
is not a crossing, and then a "12/12" count would be a lie.  This check is the cheap guard against
that: it plans each leg from the previous goal (or the robot spawn for the first one).

Path of every crossing leg goes to /tmp/chain_paths.txt (tools/nav_regress/showpath.py draws it on
nav_map).  Exit code 0 when every crossing leg goes through its tunnel.
"""
import argparse
import math
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
import terrain_path as tp  # noqa: E402
import tunnel_accept as ta  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/nav_terrain/scripts'))
import tunnel_zones as tz  # noqa: E402

R = ComputePathToPose.Result
NAMES = {R.NONE: 'ok', R.START_OCCUPIED: 'START_OCC', R.GOAL_OCCUPIED: 'GOAL_OCC', R.TIMEOUT: 'TIMEOUT',
         R.NO_VALID_PATH: 'NO_PATH', R.START_OUTSIDE_MAP: 'START_OFF', R.GOAL_OUTSIDE_MAP: 'GOAL_OFF'}
PATHS = Path('/tmp/chain_paths.txt')


def pose(node, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def plan(node, cli, s, g, yaw):
    """-> (points or None, short status)."""
    goal = ComputePathToPose.Goal()
    goal.goal = pose(node, g[0], g[1], yaw)
    goal.start = pose(node, s[0], s[1], yaw)
    goal.use_start, goal.planner_id = True, 'GridBased'
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return None, 'REJECTED'
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=20)
    r = rf.result()
    if r is None:
        return None, 'NO_RESULT'
    return [(p.pose.position.x, p.pose.position.y) for p in r.result.path.poses], \
        NAMES.get(r.result.error_code, str(r.result.error_code))


def crossed(t, pts):
    """Did the path pass the tunnel mouth-to-mouth inside the opening?"""
    neg = pos = False
    for x, y in pts:
        u, v = t.to_local(x, y)
        if abs(v) < 0.5 * t.width:
            neg |= u < -0.5 * t.depth
            pos |= u > 0.5 * t.depth
    return neg and pos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--terrain-dir', default=str(tp.default_terrain_dir()))
    ap.add_argument('--past', type=float, default=0.7)
    ap.add_argument('--min-clear', type=float, default=0.15)
    ap.add_argument('--order', choices=['chain', 'pair'], default='chain')
    ap.add_argument('--no-staging', dest='staging', action='store_false')
    ap.add_argument('--spawn', nargs=2, type=float, default=[0.49, -0.56],
                    help='map-frame robot pose at the start of the run')
    a = ap.parse_args()

    d = Path(a.terrain_dir)
    tunnels = {t.id: t for t in tz.load(d / 'zones.yaml', log=lambda m: None)}
    legs = ta.build_legs(d, a.past, a.min_clear, a.staging, a.order)

    rclpy.init()
    node = Node('chaincheck')
    cli = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    if not cli.wait_for_server(timeout_sec=30):
        print('compute_path_to_pose not available')
        return 2

    PATHS.write_text('')
    cur, bad = tuple(a.spawn), 0
    print('%-9s %-3s %-8s %7s %7s %-8s %5s %s'
          % ('tunnel', 'dir', 'leg', 'from x', 'from y', 'plan', 'cross', 'path'))
    for lg in legs:
        if a.staging:
            st, err = plan(node, cli, cur, (lg['sx'], lg['sy']), lg['yaw'])
            print('%-9s %-3s %-8s %7.2f %7.2f %-8s %5s %d pts'
                  % (lg['t'].id, 'fwd' if lg['dir'] > 0 else 'rev', 'approach', cur[0], cur[1],
                     err, '-', len(st or [])))
            if st is None:
                bad += 1
                continue
            cur = (lg['sx'], lg['sy'])
        pts, err = plan(node, cli, cur, (lg['x'], lg['y']), lg['yaw'])
        ok = crossed(lg['t'], pts) if pts else False
        extra = sorted(k for k, t in tunnels.items() if t is not lg['t'] and pts and crossed(t, pts))
        if not ok:
            bad += 1
        print('%-9s %-3s %-8s %7.2f %7.2f %-8s %5s %d pts%s'
              % (lg['t'].id, 'fwd' if lg['dir'] > 0 else 'rev', 'cross', cur[0], cur[1], err,
                 'yes' if ok else 'NO', len(pts or []), ('  +' + ','.join(extra)) if extra else ''))
        if pts:
            with PATHS.open('a') as f:
                f.write('# %s %s\n' % (lg['t'].id, 'fwd' if lg['dir'] > 0 else 'rev'))
                for x, y in pts:
                    f.write('%.3f %.3f\n' % (x, y))
        cur = (lg['x'], lg['y'])
    print('\n%d/%d crossing legs really go through their tunnel' % (len(legs) - bad, len(legs)))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
