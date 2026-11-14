#!/usr/bin/env python3
"""Print the Gazebo acceptance goals for every tunnel, both directions (map frame).

    tunnel_accept.py [--terrain-dir maps/sim] [--staging|--no-staging] [--order chain|pair]
                     [--only ID] [--one-dir] [--min-clear 0.15]

One line per goal: `<x> <y> <yaw_deg>`, to be fed to run_nav.sh with GOAL_FRAME=map through
tools/nav_regress/run_tunnel_accept.sh.

Why two goals per crossing (`--staging`, default): a goal has to be somewhere the robot can stop.
On the tunnel axis that place does not exist - the approach channel of tunnel_1/2/5/7 is 0.7 m wide
and the padded footprint is 0.62 m across, so the best a pose on the axis can have is ~4 cm of
clearance.  MPPI refuses to finish there (CostCritic + footprint check), and the goal fails with
`Controller patience exceeded` although the robot drove the tunnel perfectly.  So each crossing is
driven as

    stage  the open spot just beyond the near mouth (>= --min-clear from every lethal cell)
    goal   the open spot just beyond the far mouth, closest to the tunnel axis

The stage pose puts the robot on the correct side of the tunnel and the shaper builds the lead-in
curve from wherever it stands; the goal pose is where the crossing ends.  Both are picked as the
open spot closest to their mouth, so a leg stays short (2-4 m) and the planner has no cheap way
around the tunnel.

`--order chain` (default) orders the legs so that each one starts on the opposite side of its
tunnel: 12 legs = 12 real crossings (checked by tools/nav_regress/chaincheck.py).  `--order pair`
keeps the two directions of a tunnel together, which does not guarantee that.  The chosen tunnel
and direction of each leg go to stderr.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/nav_terrain/scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import terrain_path as tp  # noqa: E402
import tunnel_zones as tz  # noqa: E402

FP = [(0.39, 0.31), (0.39, -0.31), (-0.31, -0.31), (-0.31, 0.31)]   # padded footprint
ANGLE_STEP = math.pi / 16
RADIUS_STEP = 0.10
LATERAL = [sg * k * 0.025 for k in range(1, 9) for sg in (1, -1)]   # <= 0.2 m


def read_map(d):
    import yaml
    meta = yaml.safe_load((d / 'nav_map.yaml').read_text())
    raw = (d / meta['image']).read_bytes()
    parts = raw.split(b'\n', 3)
    w, h = map(int, parts[1].split())
    img = np.frombuffer(parts[3], np.uint8).reshape(h, w)[::-1]
    return img, meta['resolution'], meta['origin'][:2]


def clearance_field(img, res):
    """Distance in metres from every cell centre to the nearest lethal (0) cell."""
    from scipy import ndimage
    lethal = img == 0
    return ndimage.distance_transform_edt(~lethal) * res


def clearance(x, y, clr, res, origin):
    h, w = clr.shape
    c, r = int((x - origin[0]) / res), int((y - origin[1]) / res)
    return clr[r, c] if (0 <= r < h and 0 <= c < w) else 0.0


def pose_clearance(x, y, yaw, clr, res, origin):
    """Smallest clearance over the padded footprint outline (m)."""
    c, s = math.cos(yaw), math.sin(yaw)
    worst = 1e9
    for k in range(4):
        (x0, y0), (x1, y1) = FP[k], FP[(k + 1) % 4]
        for f in np.linspace(0, 1, 24):
            a, b = x0 + f * (x1 - x0), y0 + f * (y1 - y0)
            worst = min(worst, clearance(x + c * a - s * b, y + s * a + c * b, clr, res, origin))
    return worst


def pick_open(t, sgn, r0, r1, yaw, clr, res, origin, min_clear, axis_weight=2.0):
    """Open spot just beyond the mouth `sgn * depth/2`, as close to the tunnel axis as it can be.

    The grid is fine in both directions (5 cm along the axis, 2.5 cm across) because the open spots
    near a mouth are narrow: tunnel_2 fwd has no clear pose on the axis at all, only from 0.13 m off
    it.  `axis_weight` (per metre of lateral offset) dominates the distance term - a goal 0.6 m off
    the axis but 1.2 m closer would send the robot around the wall instead of through the opening,
    which looks like a crossing in the logs but is not one (tools/nav_regress/chaincheck.py catches
    it).
    """
    u_mouth = sgn * 0.5 * t.depth
    best = None
    for du in np.arange(0.05, r1 + 1e-9, 0.05):
        for v in np.arange(-0.6, 0.601, 0.025):
            u = u_mouth + sgn * du
            x, y = t.to_map(u, v)
            c = pose_clearance(x, y, yaw, clr, res, origin)
            if c < min_clear:
                continue
            score = -axis_weight * abs(v) - math.hypot(du, v) + 0.05 * min(c, 0.5)
            if best is None or score > best[0]:
                best = (score, c, x, y, u, v)
    return best


def pick_axis(t, sgn, past, clr, res, origin, min_clear):
    """Fallback: a pose on the axis, stepping outwards until the footprint clears `min_clear`."""
    yaw = t.yaw if sgn > 0 else t.yaw + math.pi
    best = None
    u = 0.5 * t.depth + past
    while u <= 0.5 * t.depth + 2.6 + 1e-9:
        for v in [0.0] + LATERAL:
            x, y = t.to_map(sgn * u, v)
            c = pose_clearance(x, y, yaw, clr, res, origin)
            score = c - 0.05 * abs(v) - 0.02 * (u - 0.5 * t.depth - past)
            if best is None or score > best[0]:
                best = (score, c, x, y, sgn * u, v)
            if c >= min_clear:
                return c, x, y, sgn * u, v
        u += 0.05
    return best[1], best[2], best[3], best[4], best[5]


def pick(t, sgn, past, clr, res, origin, min_clear=0.15):
    """-> (clearance, x, y, u, v): an open pose past the `sgn` mouth."""
    yaw = t.yaw if sgn > 0 else t.yaw + math.pi
    got = pick_open(t, sgn, past, 2.6, yaw, clr, res, origin, min_clear)
    if got is not None:
        return got[1], got[2], got[3], got[4], got[5]
    return pick_axis(t, sgn, past, clr, res, origin, min_clear)


def order_chain(legs):
    """Greedy: prefer legs that start on the opposite side of their tunnel (a real crossing)."""
    left, out, cur = list(legs), [], None
    while left:
        best = None
        for lg in left:
            if cur is not None:
                u0, v0 = lg['t'].to_local(*cur)
                # the start must be on the near side, not in the mouth and not off the opening
                if lg['dir'] * u0 >= -0.5 * lg['t'].depth or abs(v0) > 0.5 * lg['t'].width:
                    continue
            d = 0.0 if cur is None else math.hypot(lg['x'] - cur[0], lg['y'] - cur[1])
            if best is None or d < best[0]:
                best = (d, lg)
        if best is None:      # cannot cross from here: take the nearest leg anyway
            lg = min(left, key=lambda g: math.hypot(g['x'] - cur[0], g['y'] - cur[1]) if cur else 0.0)
        else:
            lg = best[1]
        left.remove(lg)
        out.append(lg)
        cur = (lg['x'], lg['y'])
    return out


def build_legs(d, past=0.7, min_clear=0.15, staging=True, order='chain', only=None, one_dir=False, log=None):
    """-> list of legs, in drive order.  Each leg: tunnel, dir, goal pose, and (staging) stage pose."""
    img, res, origin = read_map(d)
    clr = clearance_field(img, res)
    legs = []
    for t in tz.load(d / 'zones.yaml', log=log or (lambda m: None)):
        if only is not None and t.id != only:
            continue
        for sgn in ((1,) if one_dir else (1, -1)):
            c, x, y, u, v = pick(t, sgn, past, clr, res, origin, min_clear)
            yaw = t.yaw if sgn > 0 else t.yaw + math.pi
            leg = {'t': t, 'dir': sgn, 'x': x, 'y': y, 'u': u, 'v': v, 'yaw': yaw, 'clr': c}
            if staging:
                c0, x0, y0, u0, v0 = pick(t, -sgn, past, clr, res, origin, min_clear)
                leg.update(sx=x0, sy=y0, su=u0, sv=v0, sclr=c0)
            legs.append(leg)
    if order == 'chain':
        legs = order_chain(legs)
    return legs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--terrain-dir', default=str(tp.default_terrain_dir()))
    ap.add_argument('--past', type=float, default=0.7, help='search radius around the mouth, m')
    ap.add_argument('--order', choices=['chain', 'pair'], default='chain')
    ap.add_argument('--only', default=None, help='one tunnel id, debugging')
    ap.add_argument('--one-dir', action='store_true', help='only the +yaw direction (debugging)')
    ap.add_argument('--manifest', help='write requested coverage for the strict result checker')
    ap.add_argument('--min-clear', type=float, default=0.15, help='footprint clearance at a goal, m')
    ap.add_argument('--no-staging', dest='staging', action='store_false',
                    help='one goal per crossing (keeps the old single-goal behaviour)')
    a = ap.parse_args()

    legs = build_legs(Path(a.terrain_dir), a.past, a.min_clear, a.staging, a.order, a.only, a.one_dir,
                      log=lambda m: print(m, file=sys.stderr))
    if not legs:
        print('tunnel_accept: no tunnel matches --only %s' % a.only, file=sys.stderr)
        return 1
    if a.manifest:
        import json
        Path(a.manifest).write_text(json.dumps({'terrain_dir': str(Path(a.terrain_dir).resolve()),
            'only': a.only, 'one_dir': a.one_dir}, indent=2) + '\n')
    prev = None
    for lg in legs:
        print('tunnel_accept: %-9s %s goal u=%+.2f (v %+.2f) clear %.2f m%s'
              % (lg['t'].id, 'fwd' if lg['dir'] > 0 else 'rev', lg['u'], lg['v'], lg['clr'],
                 ('  stage u=%+.2f (v %+.2f) clear %.2f m' % (lg['su'], lg['sv'], lg['sclr']))
                 if a.staging else ''), file=sys.stderr)
        # Goal heading = the zone axis for both directions: the omni chassis passes backwards instead of
        # turning round, and a 180 deg turn on the spot at a 0.15 m-clearance goal is impossible (MPPI
        # finds no collision-free rollout, PATIENCE_EXCEEDED) - that is a test artefact, not a tunnel result.
        yaw = math.degrees(lg['t'].yaw)
        if a.staging:
            # the previous leg ended on this stage spot already: no extra goal
            if prev is None or math.hypot(lg['sx'] - prev[0], lg['sy'] - prev[1]) > 0.3:
                print('%.3f %.3f %.1f' % (lg['sx'], lg['sy'], yaw))
        print('%.3f %.3f %.1f' % (lg['x'], lg['y'], yaw))
        prev = (lg['x'], lg['y'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
