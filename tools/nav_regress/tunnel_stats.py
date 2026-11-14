#!/usr/bin/env python3
"""Independent acceptance numbers for the tunnel crossings, from the gz ground truth.

    tunnel_stats.py [-o RUN_DIR] [--terrain-dir maps/sim] [--lat-tol 0.03] [--yaw-tol 3]

truth.log is `/model/sentry/pose` at 20 Hz (tools/nav_regress/gz_truth.py), converted into the map
frame with terrain.json, so the numbers do not come from the localization the controller used.
Per crossing (a run of samples inside the tunnel body + 0.3 m either side):

    max |lateral|    largest distance from the zone axis while |u| <= depth / 2   (accept <= 3 cm)
    max |heading|    largest angle between the robot's long axis and the zone axis, either direction
                     (omni drives through backwards when that avoids a 180 deg turn)  (accept <= 3 deg)
    duration         time from the near mouth plane to the far one

Exit code 0 when every crossing stays within both tolerances.
"""
import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/nav_terrain/scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import terrain_path as tp  # noqa: E402
import tunnel_zones as tz  # noqa: E402

BODY_MARGIN = 0.3     # samples this far outside the body still count for finding the run


def world_to_map(x, y, reg):
    tx, ty, _ = reg['t_world_map']
    a = math.radians(reg['rpy_deg'][2])
    dx, dy = x - tx, y - ty
    return math.cos(a) * dx + math.sin(a) * dy, -math.sin(a) * dx + math.cos(a) * dy


def read_truth(path):
    """-> list of (t, map_x, map_y, yaw_deg or None)."""
    rows = []
    for line in path.read_text().splitlines():
        p = line.split()
        if len(p) not in (4, 5):
            continue
        try:
            row = tuple(float(v) for v in p)
            if all(math.isfinite(v) for v in row):
                rows.append(row)
        except ValueError:
            pass
    return rows


def crossings(pts, t):
    """Runs of samples that really pass the tunnel: inside the body + margin, spanning both mouths.

    A leg that only drives past a mouth (the field is dense: tunnels 1 and 2 are 0.95 m apart) is
    reported separately and not judged.
    """
    inside = []
    for _, x, y, _yaw in pts:
        u, v = t.to_local(x, y)
        inside.append(abs(u) <= 0.5 * t.depth + BODY_MARGIN and abs(v) <= 0.5 * t.width + 0.1)
    runs, i = [], 0
    while i < len(inside):
        if not inside[i]:
            i += 1
            continue
        f = i
        while i + 1 < len(inside) and inside[i + 1]:
            i += 1
        runs.append((f, i))
        i += 1
    through, near = [], []
    for f, l in runs:
        us = [t.to_local(x, y)[0] for _, x, y, _ in pts[f:l + 1]]
        crossed = ((us[0] < -0.5 * t.depth and us[-1] > 0.5 * t.depth) or
                   (us[0] > 0.5 * t.depth and us[-1] < -0.5 * t.depth))
        (through if crossed else near).append((f, l))
    return through, near


def axis_error(yaw_deg, axis_deg):
    """Angle between the robot's long axis and the tunnel axis, ignoring which end is the front."""
    d = (yaw_deg - axis_deg) % 180.0
    return min(d, 180.0 - d)


def assess(pts, tunnels, lat_tol=.03, yaw_tol=3., one_dir=False):
    """Strict coverage: an empty run, missing yaw or absent direction cannot pass."""
    rows, errors, covered = [], [], set()
    for tunnel in tunnels:
        through, _ = crossings(pts, tunnel)
        for first, last in through:
            run = pts[first:last + 1]
            direction = 1 if tunnel.to_local(*run[-1][1:3])[0] > 0 else -1
            body = [p for p in run if abs(tunnel.to_local(*p[1:3])[0]) <= tunnel.depth / 2]
            lat = max((abs(tunnel.to_local(*p[1:3])[1]) for p in body), default=None)
            yaw = (max(axis_error(p[3], math.degrees(tunnel.yaw)) for p in body)
                   if body and all(p[3] is not None for p in body) else None)
            valid = len(body) >= 2 and lat is not None and yaw is not None
            valid = valid and lat <= lat_tol and yaw <= yaw_tol
            row = dict(tunnel=tunnel.id, direction=direction, samples=len(body),
                       lateral_m=lat, yaw_deg=yaw, duration_s=run[-1][0] - run[0][0], passed=valid)
            rows.append(row)
            if valid:
                covered.add((tunnel.id, direction))
            else:
                errors.append(f'{tunnel.id} direction {direction:+d}: tolerances or body samples invalid')
    for tunnel in tunnels:
        for direction in ((1,) if one_dir else (1, -1)):
            if (tunnel.id, direction) not in covered:
                errors.append(f'{tunnel.id} direction {direction:+d}: no accepted crossing')
    if not tunnels:
        errors.append('no matching tunnels')
    return {'passed': not errors, 'crossings': rows, 'errors': errors}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--out', default='/tmp/nav_m3c')
    ap.add_argument('--terrain-dir', default=str(tp.default_terrain_dir()))
    ap.add_argument('--lat-tol', type=float, default=0.03)
    ap.add_argument('--yaw-tol', type=float, default=3.0)
    ap.add_argument('--only', help='require this tunnel only')
    ap.add_argument('--one-dir', action='store_true', help='require positive-axis crossing only')
    ap.add_argument('--manifest', help='coverage manifest written by tunnel_accept.py')
    a = ap.parse_args()
    if a.manifest:
        manifest = json.loads(Path(a.manifest).read_text())
        a.terrain_dir, a.only, a.one_dir = manifest['terrain_dir'], manifest['only'], manifest['one_dir']

    out, d = Path(a.out), Path(a.terrain_dir)
    reg = json.loads((d / 'terrain.json').read_text())['registration']
    raw = read_truth(out / 'truth.log')
    pts = []
    for r in raw:
        mx, my = world_to_map(r[1], r[2], reg)
        pts.append((r[0], mx, my, r[4] - reg['rpy_deg'][2] if len(r) > 4 else None))
    if len(pts) < 10:
        print('truth.log has too few samples (%d)' % len(pts))
        return 2
    has_yaw = pts[0][3] is not None
    print('truth samples: %d over %.1f s, yaw %s'
          % (len(pts), pts[-1][0] - pts[0][0], 'yes' if has_yaw else 'MISSING (old 4-column trace)'))

    tunnels = [t for t in tz.load(d / 'zones.yaml', log=lambda m: None) if not a.only or t.id == a.only]
    result = assess(pts, tunnels, a.lat_tol, a.yaw_tol, a.one_dir)
    (out / 'tunnel_result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1



if __name__ == '__main__':
    sys.exit(main())
