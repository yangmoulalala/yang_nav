#!/usr/bin/env python3
"""Plan through every tunnel in both directions with the running planner_server and check the paths.

    plan_tunnels.py [--terrain-dir maps/sim] [--approach 1.5] [--planner GridBased] [--smoother zone_shaper]

With --smoother the plan goes through smooth_path first; max|v| and the heading error (hdg, worst
|yaw - locked axis heading| on the axis segment) are then those of the shaped path.

Needs planner_server (global costmap on /nav_map) and a map -> base_link TF (plan_tunnels.sh provides
both without the simulator).  Start and goal sit on the zone axis `approach` m before / after the
centre; when one lands in a wall the distance steps down (1.2, 1.0, 0.8 m).  Per tunnel and direction:
  through    the path crosses the tunnel mouth-to-mouth inside the opening (|v| < width/2)
  max|v|     largest offset from the zone axis while |u| <= depth/2 + 0.1
  ratio      path length / straight start-goal distance
  corridor   centre of the lethal-free gap in nav_map across the tunnel, relative to the zone axis
Exit code 0 when every path goes through.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, SmoothPath
from rclpy.action import ActionClient
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/nav_terrain/scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import terrain_path as tp  # noqa: E402
import tunnel_zones as tz  # noqa: E402

R = ComputePathToPose.Result
OCCUPIED = {R.START_OCCUPIED, R.GOAL_OCCUPIED}
FALLBACK = (1.2, 1.0, 0.8)


def read_map(d):
    import yaml
    meta = yaml.safe_load((d / 'nav_map.yaml').read_text())
    raw = (d / meta['image']).read_bytes()
    parts = raw.split(b'\n', 3)
    w, h = map(int, parts[1].split())
    img = np.frombuffer(parts[3], np.uint8).reshape(h, w)[::-1]
    return img, meta['resolution'], meta['origin'][:2]


def corridor_offset(t, img, res, origin):
    """Mean centre of the lethal-free gap across the tunnel, in m from the zone axis (None if open)."""
    h, w = img.shape
    mids = []
    for u in np.arange(-t.depth / 2, t.depth / 2 + 1e-6, res / 2):
        lo = hi = None
        for v in np.arange(0, 0.8, res / 4):
            for sgn in (-1, 1):
                x, y = t.to_map(u, sgn * v)
                c, r = int((x - origin[0]) / res), int((y - origin[1]) / res)
                lethal = not (0 <= r < h and 0 <= c < w) or img[r, c] == 0
                if lethal and sgn < 0 and lo is None:
                    lo = -v
                if lethal and sgn > 0 and hi is None:
                    hi = v
        if lo is not None and hi is not None:
            mids.append(((lo + hi) / 2, hi - lo))
    if not mids:
        return None
    a = np.array(mids)
    return float(a[:, 0].mean()), float(a[:, 1].min())


FP = [(0.39, 0.31), (0.39, -0.31), (-0.31, -0.31), (-0.31, 0.31)]   # padded footprint


def footprint_free(x, y, yaw, img, res, origin):
    h, w = img.shape
    c, s = math.cos(yaw), math.sin(yaw)
    for k in range(4):
        (x0, y0), (x1, y1) = FP[k], FP[(k + 1) % 4]
        for f in np.linspace(0, 1, 40):
            a, b = x0 + f * (x1 - x0), y0 + f * (y1 - y0)
            col = int((x + c * a - s * b - origin[0]) / res)
            row = int((y + s * a + c * b - origin[1]) / res)
            if not (0 <= row < h and 0 <= col < w) or img[row, col] == 0:
                return False
    return True


def free_pose(t, u, yaw, img, res, origin):
    """Axis point at u; if the footprint touches a wall there, the nearest lateral shift (<= 0.25 m) that
    clears it - a real robot never starts or stops inside a wall."""
    for v in [0.0] + [sg * k * 0.025 for k in range(1, 11) for sg in (1, -1)]:
        x, y = t.to_map(u, v)
        if footprint_free(x, y, yaw, img, res, origin):
            return (x, y)
    return t.to_map(u, 0.0)


def pose(node, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def plan(node, cli, s, g, yaw, planner):
    """-> (points or [], error code, or None if rejected / timed out)."""
    goal = ComputePathToPose.Goal()
    goal.goal, goal.start, goal.use_start, goal.planner_id = pose(node, *g, yaw), pose(node, *s, yaw), True, planner
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return None, None
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=15)
    r = rf.result()
    if r is None or r.status != 4 or r.result.error_code != 0:
        return None, None
    return [(p.pose.position.x, p.pose.position.y) for p in r.result.path.poses], r.result.error_code


def smooth(node, cli, pts, yaws, smoother):
    goal = SmoothPath.Goal()
    goal.smoother_id = smoother
    goal.path.header.frame_id = 'map'
    for (x, y), yaw in zip(pts, yaws):
        goal.path.poses.append(pose(node, x, y, yaw))
    goal.max_smoothing_duration.sec = 2
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    gh = fut.result()
    if gh is None or not gh.accepted:
        return None
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=10)
    r = rf.result()
    if r is None or r.status != 4 or r.result.error_code != 0:
        return None
    return [(p.pose.position.x, p.pose.position.y, 2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w))
            for p in r.result.path.poses]


def axis_heading_error(t, poses, margin=0.5):
    """Worst heading error vs the closer of yaw / yaw+pi on the straight axis (|u| < depth/2 + margin)."""
    worst = 0.0
    for x, y, yaw in poses:
        u, v = t.to_local(x, y)
        if abs(u) < t.depth / 2 + margin - 0.01 and abs(v) < t.width / 2:
            e = (yaw - t.yaw + math.pi / 2) % math.pi - math.pi / 2
            worst = max(worst, abs(e))
    return math.degrees(worst)


def analyse(t, pts, start, goal):
    uv = np.array([t.to_local(x, y) for x, y in pts])
    inside = np.abs(uv[:, 0]) <= t.depth / 2 + 0.1
    length = float(np.sum(np.hypot(*np.diff(np.array(pts), axis=0).T))) if len(pts) > 1 else 0.0
    straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
    mouth = t.depth / 2
    seen_neg = seen_pos = False
    for u, v in uv:
        if abs(v) < t.width / 2:
            seen_neg |= u < -mouth
            seen_pos |= u > mouth
    through = bool(inside.any()) and seen_neg and seen_pos and bool((np.abs(uv[inside, 1]) < t.width / 2).all())
    maxv = float(np.abs(uv[inside, 1]).max()) if inside.any() else float('nan')
    return through, maxv, length / straight if straight else float('nan'), int(inside.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--terrain-dir', default=str(tp.default_terrain_dir()))
    ap.add_argument('--approach', type=float, default=1.5)
    ap.add_argument('--planner', default='GridBased')
    ap.add_argument('--smoother', default='')
    a = ap.parse_args()
    d = Path(a.terrain_dir)
    tunnels = tz.load(d / 'zones.yaml')
    img, res, origin = read_map(d)
    approaches = [a.approach] + [x for x in FALLBACK if x < a.approach]

    rclpy.init()
    node = Node('plan_tunnels')
    cli = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    if not cli.wait_for_server(timeout_sec=30):
        print('compute_path_to_pose not available'); return 2
    scli = None
    if a.smoother:
        scli = ActionClient(node, SmoothPath, 'smooth_path')
        if not scli.wait_for_server(timeout_sec=30):
            print('smooth_path not available'); return 2
    ok_all = True
    print('%-9s %-4s %5s %-7s %7s %6s %5s %s' % ('tunnel', 'dir', 'appr', 'through', 'max|v|', 'ratio', 'n_in',
                                               'corridor (centre, min gap)'))
    for t in tunnels:
        corr = corridor_offset(t, img, res, origin)
        cs = 'open' if corr is None else '%+.3f m, %.3f m' % corr
        for sgn, name in ((1, 'fwd'), (-1, 'rev')):
            yaw = t.yaw if sgn > 0 else t.yaw + math.pi
            for appr in approaches:
                s = free_pose(t, -sgn * appr, yaw, img, res, origin)
                g = free_pose(t, sgn * appr, yaw, img, res, origin)
                pts, err = plan(node, cli, s, g, yaw, a.planner)
                if pts or err not in OCCUPIED:
                    break
            if pts is None:
                print('%-9s %-4s %5.1f rejected / timeout' % (t.id, name, appr)); ok_all = False; continue
            if not pts:
                print('%-9s %-4s %5.1f NO PATH (error %s)   %s' % (t.id, name, appr, err, cs)); ok_all = False; continue
            hdg = ''
            if scli is not None:
                yaws = [math.atan2(pts[min(i + 1, len(pts) - 1)][1] - pts[max(i - 1, 0)][1],
                                   pts[min(i + 1, len(pts) - 1)][0] - pts[max(i - 1, 0)][0]) for i in range(len(pts))]
                yaws[0] = yaws[-1] = yaw  # keep the requested start/goal orientations
                sp = smooth(node, scli, pts, yaws, a.smoother)
                if not sp:
                    print('%-9s %-4s %5.1f SMOOTH FAILED' % (t.id, name, appr)); ok_all = False; continue
                pts = [(x, y) for x, y, _ in sp]
                he = axis_heading_error(t, sp)
                hdg = ' hdg %.2f deg' % he
                ok_all &= he < 1.0
            through, maxv, ratio, n_in = analyse(t, pts, s, g)
            if scli is not None:
                ok_all &= maxv < 0.035   # max_body_offset 0.03 + sampling
            ok_all &= through
            print('%-9s %-4s %5.1f %-7s %7.3f %6.2f %5d %s' % (t.id, name, appr, 'yes' if through else 'NO',
                                                             maxv, ratio, n_in, cs) + hdg)
    node.destroy_node(); rclpy.shutdown()
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
