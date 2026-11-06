#!/usr/bin/env python3
"""Pick N random flat, free goals (map frame) near a start point from a terrain package.

    pick_flat_goals.py TERRAIN_DIR [--n 5] [--x 0 --y 0] [--rmin 1.0 --rmax 3.0] [--seed 1]

A cell qualifies when it is reachable (finite ground_z), within 3 cm of the ground height
at the start, and its whole 0.8 m neighbourhood is reachable and equally flat (no slopes,
steps or walls nearby).  Prints one 'X Y YAW_DEG' line per goal.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('terrain_dir')
    ap.add_argument('--n', type=int, default=5)
    ap.add_argument('--x', type=float, default=0.0)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--rmin', type=float, default=1.0)
    ap.add_argument('--rmax', type=float, default=3.0)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--margin', type=float, default=0.6,
                    help='flat, free neighbourhood radius (m); > circumscribed radius 0.49')
    ap.add_argument('--min-free', type=int, default=250, help='min nav_map pixel (255 = free)')
    ap.add_argument('--any-height', action='store_true',
                    help='accept flat cells at any height (default: same level as the start)')
    a = ap.parse_args()
    d = Path(a.terrain_dir)
    meta = json.loads((d / 'terrain.json').read_text())
    nx, ny = meta['size']
    res = meta['resolution']
    ox, oy = meta['origin']
    g = np.fromfile(d / 'ground_z.f32', '<f4').reshape(nx, ny).astype(float)
    sx, sy = int((a.x - ox) / res), int((a.y - oy) / res)
    z0 = np.nanmedian(g[sx - 4:sx + 5, sy - 4:sy + 5])
    if a.any_height:
        # locally flat at any height: ground gradient below ~3 deg (no ramps, no step edges)
        gx, gy = np.gradient(np.nan_to_num(g, nan=1e3), res)
        flat = np.isfinite(g) & (np.hypot(gx, gy) < 0.05)
    else:
        flat = np.isfinite(g) & (np.abs(np.nan_to_num(g, nan=1e9) - z0) < 0.03)
    # nav_map.pgm (scale mode, negate 0): 255 = free, 0 = lethal; keep near-free cells only
    raw = (d / 'nav_map.pgm').read_bytes()
    hdr = raw.split(b'\n', 3)
    w, h = map(int, hdr[1].split())
    img = np.frombuffer(raw[-w * h:], np.uint8).reshape(h, w)
    free_img = img[::-1, :].T >= a.min_free    # -> [ix, iy]
    flat &= free_img[:nx, :ny]
    k = int(a.margin / res)
    # cell ok when every cell within k is flat: erode with a box via cumulative sums
    f = flat.astype(np.int32)
    c = np.pad(f, ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    ok = np.zeros_like(flat)
    w = 2 * k + 1
    box = c[w:, w:] - c[:-w, w:] - c[w:, :-w] + c[:-w, :-w]
    ok[k:nx - k, k:ny - k] = box == w * w
    ix, iy = np.nonzero(ok)
    px, py = ox + (ix + 0.5) * res, oy + (iy + 0.5) * res
    r = np.hypot(px - a.x, py - a.y)
    sel = (r >= a.rmin) & (r <= a.rmax)
    px, py = px[sel], py[sel]
    if len(px) == 0:
        raise SystemExit('no flat cells in range')
    rng = np.random.default_rng(a.seed)
    picks = []
    for _ in range(2000):
        i = rng.integers(len(px))
        if all(math.hypot(px[i] - q[0], py[i] - q[1]) > 0.8 for q in picks):
            picks.append((px[i], py[i], float(rng.uniform(-180, 180))))
        if len(picks) == a.n:
            break
    for x, y, yaw in picks:
        print('%.2f %.2f %.0f' % (x, y, yaw))


if __name__ == '__main__':
    main()
