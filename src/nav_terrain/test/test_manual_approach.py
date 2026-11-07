"""Annotated tunnel floor higher than the surrounding floor, approach strips unobserved:
the fill must ramp (no artificial step) so entry and exit stay reachable."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import terrain_builder as TB  # noqa: E402
import test_terrain_builder as TT  # noqa: E402

FLOOR = 0.08   # annotated tunnel floor, 8 cm above the observed floor (> max_step 5 cm)


def scene():
    P = TT.scene(np.random.default_rng(2))
    y_in = (P[:, 1] >= 1.0) & (P[:, 1] <= 1.75)
    low = P[:, 2] < 0.1
    blind = y_in & low & (P[:, 0] >= 0.55) & (P[:, 0] <= 1.8)   # tunnel floor + both approach strips
    return P[~blind]


def build(tmp_path, approach):
    f = tmp_path / 'zones_manual.yaml'
    f.write_text('tunnels:\n  - id: t\n    centre: [1.175, 1.375]\n    yaw_deg: 0\n'
                 '    size: [0.35, 0.7]\n    floor_z: %.3f\n    roof_z: %.3f\n' % (FLOOR, FLOOR + 0.25))
    cfg = TB.build_parser().parse_args(['--map', 'x', '--out', 'y', '--seed', '0.3', '0.3', '0.0',
                                        '--manual', str(f), '--manual-approach-fill', str(approach)])
    return TB.build_from_points(scene(), cfg, log=lambda *_: None)


def test_ramped_approach_keeps_both_ends_open(tmp_path):
    T = build(tmp_path, 2.0)
    t = T['tunnels'][0]
    assert t['both_ends_open'], t
    assert t['reachable_frac'] > 0.9, t
    i, j = TB.cell_of(1.7, 1.37, T['x0'], T['y0'], T['res'])
    assert T['mask'][i, j] and not T['lethal'][i, j]
    assert 0.0 < T['G'][i, j] < FLOOR       # on the ramp, between outside floor and tunnel floor


def test_observed_cells_untouched(tmp_path):
    T = build(tmp_path, 2.0)
    i, j = TB.cell_of(1.2, 0.8, T['x0'], T['y0'], T['res'])   # side wall stays lethal
    assert T['lethal'][i, j]
    i, j = TB.cell_of(0.3, 1.37, T['x0'], T['y0'], T['res'])  # observed floor keeps its height
    assert abs(T['G'][i, j]) < 0.01
