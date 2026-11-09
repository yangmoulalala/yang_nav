import math
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import mesh_to_terrain as mt  # noqa: E402
from terrain_map import FLAT, OBSTACLE, SLOPE, TUNNEL, TerrainMap, encode_angle  # noqa: E402


def quad(p0, p1, p2, p3):
    """Two triangles, counter-clockwise seen from the side the normal points to."""
    return [[p0, p1, p2], [p0, p2, p3]]


def box(x0, x1, y0, y1, z0, z1):
    """Closed box with outward normals."""
    t = []
    t += quad((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))     # top, up
    t += quad((x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0))     # bottom, down
    t += quad((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1))
    t += quad((x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1))
    t += quad((x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1))
    t += quad((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1))
    return t


def field():
    """6 x 4 m floor; tunnel along x at x 3.5..4.5, y 2.6..3.4 (roof 0.30); ramp up along +y."""
    t = []
    t += quad((0, 0, 0), (6, 0, 0), (6, 4, 0), (0, 4, 0))
    t += box(3.5, 4.5, 2.0, 2.6, 0, 0.6)          # tunnel walls
    t += box(3.5, 4.5, 3.4, 4.0, 0, 0.6)
    t += box(3.5, 4.5, 2.6, 3.4, 0.30, 0.6)       # roof: clearance 0.30 < 0.245 + 0.10
    # ramp 1.0 m wide, rises 0.20 m over 1.0 m along +y (11.3 deg), then a plateau
    t += quad((1.0, 0.5, 0.0), (2.0, 0.5, 0.0), (2.0, 1.5, 0.2), (1.0, 1.5, 0.2))
    t += quad((1.0, 1.5, 0.2), (2.0, 1.5, 0.2), (2.0, 2.5, 0.2), (1.0, 2.5, 0.2))
    return np.array(t, float)


def run(m=None):
    cfg = mt.build_parser().parse_args(['--mesh', 'x', '--out', 'y'])
    res = 0.05
    nx, ny = int(6 / res), int(4 / res)
    return mt.build_auto(field(), 0.0, 0.0, res, nx, ny, [(0.5, 3.5, 0.0)], cfg, log=lambda *_: None)


def cell(x, y, res=0.05):
    return int(y / res), int(x / res)


def test_labels_from_mesh():
    a = run()
    t, d = a['terrain'], a['direction']
    assert t[cell(0.5, 3.5)] == FLAT
    assert t[cell(4.0, 3.0)] == TUNNEL
    assert d[cell(4.0, 3.0)] == 0                                # passage along x
    assert t[cell(4.0, 2.3)] == OBSTACLE                         # wall
    assert t[cell(1.5, 1.0)] == SLOPE
    assert d[cell(1.5, 1.0)] == encode_angle(math.pi / 2)        # uphill +y
    assert abs(a['ground'][cell(1.5, 1.0)] - 0.1) < 0.01
    assert abs(a['clearance'][cell(4.0, 3.0)] - 0.30) < 1e-6
    assert abs(a['elevation'][cell(4.0, 2.3)] - 0.6) < 1e-6      # unreachable: top surface


def test_manual_edits_survive_regeneration(tmp_path):
    a = run()
    m = TerrainMap.blank(120, 80, 0.05)
    m.merge_auto(a['terrain'], a['direction'])
    r, c = cell(0.5, 3.5)
    m.terrain[r, c], m.ensure_manual()[r, c] = OBSTACLE, True    # hand edit
    m.save(tmp_path / 'm.msgpack')
    n = TerrainMap.load(tmp_path / 'm.msgpack')
    assert n.manual[r, c] and n.auto_terrain[r, c] == FLAT
    b = run()
    b['terrain'][cell(1.5, 1.0)] = FLAT                          # automatic result changed
    n.merge_auto(b['terrain'], b['direction'])
    assert n.terrain[r, c] == OBSTACLE                           # manual kept
    assert n.terrain[cell(1.5, 1.0)] == FLAT                     # auto followed
    mask = np.zeros(n.terrain.shape, bool)
    mask[r, c] = True
    assert n.revert_to_auto(mask) == 1 and n.terrain[r, c] == FLAT and not n.manual[r, c]


def test_binary_stl_and_registration_roundtrip(tmp_path):
    tri = field()
    p = tmp_path / 'f.stl'
    with open(p, 'wb') as f:
        f.write(b'\0' * 80 + struct.pack('<I', len(tri)))
        for t in tri:
            f.write(struct.pack('<3f', 0, 0, 0) + t.astype('<f4').tobytes() + b'\0\0')
    assert np.allclose(mt.load_stl(p), tri, atol=1e-6)
    R = mt.rpy_matrix(0.01, -0.02, 0.3)
    assert np.allclose(mt.rpy_matrix(*mt.matrix_rpy(R)), R)
