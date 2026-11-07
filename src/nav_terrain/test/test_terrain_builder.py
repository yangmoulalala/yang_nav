"""Synthetic scene: floor, a 0.25 m tunnel between two walls (floor partly hidden under the roof),
a 15 deg ramp up to a plateau. Run: python3 -m pytest src/nav_terrain/test -q"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import terrain_builder as TB  # noqa: E402

RAMP_DEG = 15.0
PLATEAU = math.tan(math.radians(RAMP_DEG)) * 1.0


def scene(rng):
    pts = []
    xs = np.arange(0.0, 6.0, 0.02); ys = np.arange(0.0, 3.0, 0.02)
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    X, Y = X.ravel(), Y.ravel()
    Zf = np.where(X < 3.0, 0.0, np.where(X < 4.0, (X - 3.0) * math.tan(math.radians(RAMP_DEG)), PLATEAU))
    wall = (X >= 0.8) & (X <= 1.55) & (((Y >= 0.6) & (Y < 1.0)) | ((Y > 1.75) & (Y <= 2.15)))
    hidden = (X >= 1.1) & (X <= 1.25) & (Y >= 1.0) & (Y <= 1.75)
    keep = ~wall & ~hidden
    pts.append(np.stack([X[keep], Y[keep], Zf[keep]], 1))
    for z in np.arange(0.0, 0.6, 0.02):
        pts.append(np.stack([X[wall], Y[wall], np.full(wall.sum(), z)], 1))
    roof = (X >= 1.0) & (X <= 1.35) & (Y >= 1.0) & (Y <= 1.75)
    for z in (0.25, 0.26, 0.27):
        pts.append(np.stack([X[roof], Y[roof], np.full(roof.sum(), z)], 1))
    P = np.vstack(pts)
    return P + rng.normal(0, 0.002, P.shape)


def build():
    cfg = TB.build_parser().parse_args(['--map', 'x', '--out', 'y', '--seed', '0.3', '0.3', '0.0'])
    return TB.build_from_points(scene(np.random.default_rng(0)), cfg, log=lambda *_: None), cfg


def test_tunnel_found_with_passage_along_x():
    T, _ = build()
    tunnels = [t for t in T['tunnels'] if t['area_m2'] > 0.1]
    assert len(tunnels) == 1, tunnels
    t = tunnels[0]
    assert abs(t['centre'][0] - 1.175) < 0.1 and abs(t['centre'][1] - 1.375) < 0.1
    yaw = t['passage_yaw_deg'] % 180.0
    assert min(yaw, 180.0 - yaw) < 10.0, t
    assert 0.23 < t['min_clearance'] < 0.27
    assert t['both_ends_open']
    assert 0.6 < t['corridor_width'] < 0.9, t


def test_tunnel_floor_reachable_and_walls_lethal():
    T, _ = build()
    i, j = TB.cell_of(1.17, 1.37, T['x0'], T['y0'], T['res'])
    assert T['mask'][i, j] and not T['lethal'][i, j]
    i, j = TB.cell_of(1.2, 0.8, T['x0'], T['y0'], T['res'])
    assert T['lethal'][i, j]


def test_ramp_and_plateau():
    T, _ = build()
    ramps = [r for r in T['ramps'] if r['area_m2'] > 0.5]
    assert len(ramps) == 1, T['ramps']
    r = ramps[0]
    assert abs(r['slope_median_deg'] - RAMP_DEG) < 2.0, r
    assert abs(r['uphill_yaw_deg']) < 10.0, r
    i, j = TB.cell_of(5.0, 1.5, T['x0'], T['y0'], T['res'])
    assert T['mask'][i, j] and abs(T['G'][i, j] - PLATEAU) < 0.02
