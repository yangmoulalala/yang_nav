import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import terrain_auto_direction as ad  # noqa: E402
from terrain_map import OBSTACLE, SLOPE, TUNNEL, TerrainMap, encode_angle  # noqa: E402


def blank():
    m = TerrainMap.blank(80, 80, 0.05, (0.0, 0.0))
    m.elevation = np.zeros((80, 80), np.float32)
    m.direction[:] = 64
    return m


def test_slope_uphill_minus_x():
    m = blank()
    rr, cc = np.mgrid[30:50, 20:60]
    m.terrain[rr, cc] = SLOPE
    m.elevation[rr, cc] = 0.3 - 0.01 * (cc - 20)          # higher at low x
    rep = ad.solve(m)
    assert m.direction[40, 40] == encode_angle(math.pi)
    assert rep[0]['incline_deg'] > 5 and 'warning' not in rep[0]


def test_square_tunnel_axis_from_walls():
    m = blank()
    m.terrain[32:48, 31:49] = TUNNEL                      # 18 x 16 cells, nearly square
    m.elevation[32:48, 31:49] = 0.28                       # roof seen from above
    m.terrain[32:48, 25:31] = OBSTACLE                     # walls on both x sides
    m.terrain[32:48, 49:55] = OBSTACLE
    m.elevation[32:48, 25:31] = 0.5
    m.elevation[32:48, 49:55] = 0.5
    ad.solve(m)
    # passage along y; tunnels are bidirectional, stored in [-90, 90) -> -90 deg
    assert m.direction[40, 40] == encode_angle(-math.pi / 2)


def test_tunnel_angle_in_half_plane():
    m = blank()
    m.terrain[34:46, 30:50] = TUNNEL
    m.terrain[28:34, 30:50] = OBSTACLE                     # walls on both y sides -> passage along x
    m.terrain[46:52, 30:50] = OBSTACLE
    ad.solve(m)
    assert m.direction[40, 40] == 0
