#!/usr/bin/env python3
"""terrain_auto_direction: fill the direction channel of labelled regions painted without a direction.

    terrain_auto_direction.py maps/sim/terrain.msgpack [--dry-run]

Per 8-connected region of a directional label:
  rectangle axes  minimum-area rectangle around the region cells (robust for near-square tunnels)
  slope / steps   robust least-squares plane on the elevation -> uphill vector, snapped to the nearest
                  rectangle axis direction when within --snap-deg
  tunnel          passage axis = the rectangle axis whose two ends are open (not obstacle, not higher
                  than the surrounding floor + --wall-lift); a tunnel has no up, the angle is kept in
                  [-90, 90)
The previous file is kept as <map>.bak before saving.  Check the result with the editor arrows (a).
"""
import argparse
import math
import shutil
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent))
from terrain_map import DIRECTIONAL, LABELS, OBSTACLE, TUNNEL, TerrainMap, encode_angle  # noqa: E402


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rect_axis(x, y):
    """Angle of one side of the minimum-area rectangle around the points (the other side is +90 deg)."""
    import cv2
    if x.size < 3:
        return 0.0
    (_, _), (_, _), ang = cv2.minAreaRect(np.column_stack([x, y]).astype(np.float32))
    return math.radians(ang)


def fit_uphill(x, y, z):
    """-> (uphill angle, incline deg) from a plane fit with one outlier rejection pass."""
    A = np.column_stack([x - x.mean(), y - y.mean(), np.ones_like(x)])
    keep = np.ones(z.size, bool)
    for _ in range(2):
        coef = np.linalg.lstsq(A[keep], z[keep], rcond=None)[0]
        res = np.abs(A @ coef - z)
        mad = np.median(res[keep]) + 1e-4
        keep = res <= 3 * mad
        if keep.sum() < 3:
            break
    gx, gy = coef[0], coef[1]
    return math.atan2(gy, gx), math.degrees(math.atan(math.hypot(gx, gy)))


def end_scores(m, inside, cx, cy, t, floor, wall_lift, depth=(0.10, 0.35)):
    """Blocked fraction just beyond each end of the region along axis t."""
    rows, cols = np.nonzero(inside)
    x = m.origin[0] + (cols + 0.5) * m.resolution - cx
    y = m.origin[1] + (rows + 0.5) * m.resolution - cy
    d, n = np.array([math.cos(t), math.sin(t)]), np.array([-math.sin(t), math.cos(t)])
    u, v = x * d[0] + y * d[1], x * n[0] + y * n[1]
    span = v.max() - v.min()
    vs = np.linspace(v.min() + 0.25 * span, v.max() - 0.25 * span, 5)
    out = []
    for sign, u0 in ((1, u.max()), (-1, u.min())):
        blocked = []
        for s in np.arange(depth[0], depth[1] + 1e-9, m.resolution):
            for vv in vs:
                px, py = np.array([cx, cy]) + d * (u0 + sign * s) + n * vv
                c, r = m.cell_of(px, py)
                if not (0 <= c < m.width and 0 <= r < m.height):
                    blocked.append(1.0)
                    continue
                if inside[r, c]:
                    continue
                z = m.elevation[r, c] if m.elevation is not None else np.nan
                blocked.append(float(m.terrain[r, c] == OBSTACLE
                                     or (np.isfinite(z) and np.isfinite(floor) and z > floor + wall_lift)))
        out.append(float(np.mean(blocked)) if blocked else 1.0)
    return out


def solve(m, snap_deg=35.0, wall_lift=0.15, min_incline=2.0):
    """Fill m.direction in place; -> list of report dicts."""
    report = []
    res = m.resolution
    ring_r = max(1, int(round(0.5 / res)))
    for lab in sorted(DIRECTIONAL):
        comp, nreg = ndimage.label(m.terrain == lab, structure=np.ones((3, 3)))
        for k in range(1, nreg + 1):
            inside = comp == k
            rows, cols = np.nonzero(inside)
            x = m.origin[0] + (cols + 0.5) * res
            y = m.origin[1] + (rows + 0.5) * res
            cx, cy = float(x.mean()), float(y.mean())
            a0 = rect_axis(x, y)
            axes = [wrap(a0 + i * math.pi / 2) for i in range(4)]
            rep = {'id': '%s_%d' % (LABELS[lab], k), 'centre': (round(cx, 2), round(cy, 2)), 'cells': int(rows.size)}
            warn = []
            if lab == TUNNEL:
                ring = ndimage.binary_dilation(inside, iterations=ring_r) & ~inside
                ez = m.elevation[ring] if m.elevation is not None else np.array([])
                ez = ez[np.isfinite(ez)]
                floor = float(np.percentile(ez, 20)) if ez.size else float('nan')
                sc = [float(np.mean(end_scores(m, inside, cx, cy, axes[i], floor, wall_lift))) for i in (0, 1)]
                best = int(np.argmin(sc))
                ang = axes[best]
                if ang >= math.pi / 2 - 1e-6:
                    ang -= math.pi
                elif ang < -math.pi / 2 - 1e-6:
                    ang += math.pi
                rep.update(method='open ends', blocked='%.2f / %.2f' % (sc[best], sc[1 - best]))
                if sc[1 - best] - sc[best] < 0.2:
                    warn.append('axis ambiguous, check in the editor')
            else:
                z = m.elevation[rows, cols] if m.elevation is not None else np.full(rows.size, np.nan)
                ok = np.isfinite(z)
                if ok.sum() < 5:
                    report.append(dict(rep, warning='no elevation, direction left unchanged'))
                    continue
                up, inc = fit_uphill(x[ok], y[ok], z[ok])
                diffs = [abs(wrap(up - a)) for a in axes]
                i = int(np.argmin(diffs))
                ang = axes[i] if math.degrees(diffs[i]) <= snap_deg else up
                rep.update(method='plane fit', fit_deg=round(math.degrees(up)), incline_deg=round(inc, 1))
                if math.degrees(diffs[i]) > snap_deg:
                    warn.append('uphill not along a region side, kept the raw fit')
                if inc < min_incline:
                    warn.append('nearly flat (%.1f deg): is this really %s?' % (inc, LABELS[lab]))
                if min(rows.ptp(), cols.ptp()) < 2:
                    warn.append('only %d cell(s) wide' % (min(rows.ptp(), cols.ptp()) + 1))
            code = encode_angle(ang)
            m.direction[inside] = code
            rep.update(dir_deg=round(math.degrees(ang), 1), code=code)
            if warn:
                rep['warning'] = '; '.join(warn)
            report.append(rep)
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map')
    ap.add_argument('--snap-deg', type=float, default=35.0)
    ap.add_argument('--wall-lift', type=float, default=0.15, help='m above the local floor counts as wall')
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args(argv)
    m = TerrainMap.load(a.map)
    if m.elevation is None:
        sys.exit('map has no elevation; run cloud_to_height_map.py first')
    for r in solve(m, a.snap_deg, a.wall_lift):
        print('  '.join('%s=%s' % kv for kv in r.items()))
    if not a.dry_run:
        shutil.copy2(a.map, str(a.map) + '.bak')
        m.save(a.map)
        print('saved %s (backup %s.bak)' % (a.map, a.map))


if __name__ == '__main__':
    main()
