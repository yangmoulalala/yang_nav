import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import terrain_export as tx  # noqa: E402
from terrain_map import (OBSTACLE, SLOPE, TUNNEL, TerrainMap, decode_angle, encode_angle,  # noqa: E402
                         packb, unpackb)


def test_msgpack_roundtrip():
    obj = {'width': 3, 'neg': -5, 'big': 70000, 'f': 0.25, 's': 'x' * 40, 'b': bytes(range(256)) * 300,
           'l': [1, None, True, False], 'nested': {'a': [1.5]}}
    assert unpackb(packb(obj)) == obj




def test_map_save_load(tmp_path):
    m = TerrainMap.blank(6, 4, 0.1, (-1.0, 2.0))
    m.terrain[2, 3] = TUNNEL
    m.direction[2, 3] = encode_angle(math.pi / 2)
    m.elevation = np.full((4, 6), np.nan, np.float32)
    m.elevation[0, 0] = 0.3
    m.save(tmp_path / 'm.msgpack')
    n = TerrainMap.load(tmp_path / 'm.msgpack')
    assert n.origin == (-1.0, 2.0) and n.terrain[2, 3] == TUNNEL
    assert abs(decode_angle(n.direction[2, 3]) - math.pi / 2) < 0.03
    assert n.elevation[0, 0] == pytest.approx(0.3) and np.isnan(n.elevation[1, 1])
    assert n.cell_of(*n.cell_center(3, 2)) == (3, 2)


def test_invalid_label_rejected():
    m = TerrainMap.blank(2, 2, 0.05)
    m.terrain[0, 0] = 42
    with pytest.raises(ValueError):
        m.to_dict()






def test_cost_inflation_profile():
    obs = np.zeros((1, 40), bool)
    obs[0, 0] = True
    cost = tx.inflate_cost(obs, 0.05, 0.10, 0.30, 24.0)
    assert cost[0, 0] == 255 and cost[0, 2] == 255                  # inside the full-cost radius
    assert cost[0, 4] == int(255 * math.exp(-24 * 0.10))             # 0.20 m
    assert cost[0, 7] == 0                                           # beyond cutoff


def test_direction_field_body_and_ring():
    t = np.zeros((21, 21), np.uint8)
    d = np.zeros_like(t)
    t[10, 10] = SLOPE
    d[10, 10] = encode_angle(math.pi / 2)
    t[0, 0] = OBSTACLE
    ang, mag, lab, overlap = tx.inflate_direction(t, d, 0.05, 0.10, 0.30, 24.0, 0.9)
    assert mag[10, 10] == 255 and lab[10, 10] == SLOPE
    assert mag[10, 11] == int(0.9 * 255) and lab[10, 11] == SLOPE   # ring capped below body
    assert ang[10, 11] == d[10, 10]
    assert mag[10, 18] == 0 and lab[10, 18] == 0                     # 0.40 m: beyond cutoff
    assert lab[0, 0] == OBSTACLE and mag[0, 0] == 0 and overlap == 0
    # label/magnitude invariant from map_utils
    assert np.array_equal(np.isin(lab, list(tx.DIRECTIONAL)), mag > 0)


def test_export_package_and_zones(tmp_path):
    m = TerrainMap.blank(80, 60, 0.05, (-2.0, -1.5))
    m.terrain[:, 0] = OBSTACLE
    m.terrain[20:28, 30:50] = TUNNEL                     # 1.0 m along x, 0.4 m across
    m.direction[20:28, 30:50] = encode_angle(0.0)
    cfg = tx.build_parser().parse_args([str(tmp_path / 'm.msgpack'), '--out', str(tmp_path / 'pkg')])
    cfg.speed_limits, cfg.src = {'tunnel': 35}, tmp_path / 'm.msgpack'
    meta, zones = tx.export(m, tmp_path / 'pkg', cfg)
    for f in ('nav_map.pgm', 'nav_map.yaml', 'speed_mask.pgm', 'direction_map.png', 'zones.yaml', 'terrain.json'):
        assert (tmp_path / 'pkg' / f).is_file()
    raw = (tmp_path / 'pkg' / 'nav_map.pgm').read_bytes()
    img = np.frombuffer(raw[-80 * 60:], np.uint8).reshape(60, 80)[::-1]
    assert img[30, 0] == 0                               # obstacle lethal
    assert 0 < img[30, 1] < 255 and img[30, 1] > 255 - 0.6 * 255 - 1   # band capped below lethal
    assert img[30, 70] == 255                            # free
    assert len(zones) == 1
    z = zones[0]
    assert z['type'] == 'tunnel' and z['yaw_deg'] == 0.0
    assert z['size'] == pytest.approx([1.0, 0.4])
    assert z['centre'] == pytest.approx([-2.0 + 2.0, -1.5 + 1.2])
    assert z['entry'][0] < z['centre'][0] < z['exit'][0]
    assert yaml_load(tmp_path / 'pkg' / 'nav_map.yaml')['origin'][:2] == [-2.0, -1.5]


def yaml_load(p):
    import yaml
    return yaml.safe_load(Path(p).read_text())
