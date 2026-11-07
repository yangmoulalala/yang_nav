"""Tunnel whose floor the LiDAR never saw: unreachable without annotation, passable with it."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import terrain_builder as TB  # noqa: E402
import test_terrain_builder as TT  # noqa: E402


def blind_scene():
    P = TT.scene(np.random.default_rng(1))
    # drop every floor point under the roof: only the roof is seen
    under = (P[:, 0] >= 1.0) & (P[:, 0] <= 1.35) & (P[:, 1] >= 1.0) & (P[:, 1] <= 1.75) & (P[:, 2] < 0.1)
    return P[~under]


def build(manual_file=None):
    args = ['--map', 'x', '--out', 'y', '--seed', '0.3', '0.3', '0.0']
    if manual_file:
        args += ['--manual', str(manual_file)]
    cfg = TB.build_parser().parse_args(args)
    return TB.build_from_points(blind_scene(), cfg, log=lambda *_: None)


def test_blind_tunnel_blocked_without_annotation():
    T = build()
    i, j = TB.cell_of(1.17, 1.37, T['x0'], T['y0'], T['res'])
    assert not T['mask'][i, j] or T['lethal'][i, j]


def test_annotation_opens_tunnel(tmp_path):
    f = tmp_path / 'zones_manual.yaml'
    f.write_text('tunnels:\n  - id: t\n    centre: [1.175, 1.375]\n    yaw_deg: 0\n'
                 '    size: [0.35, 0.7]\n    floor_z: 0.0\n    roof_z: 0.25\n')
    T = build(f)
    i, j = TB.cell_of(1.17, 1.37, T['x0'], T['y0'], T['res'])
    assert T['mask'][i, j] and not T['lethal'][i, j]
    assert abs(T['CL'][i, j] - 0.25) < 0.01
    i2, j2 = TB.cell_of(1.7, 1.37, T['x0'], T['y0'], T['res'])
    assert T['mask'][i2, j2]     # reached through the tunnel side
    t = min(T['tunnels'], key=lambda t: abs(t['centre'][0] - 1.175) + abs(t['centre'][1] - 1.375))
    assert t['source'].startswith('manual')
