import math
from pathlib import Path

import numpy as np
import pytest

from nav_terrain import terrain_export as tx
from nav_terrain.terrain_map import (FLAT, OBSTACLE, UNKNOWN, SLOPE, TUNNEL, TerrainMap, decode_angle, encode_angle)
from nav_terrain._msgpack import packb, unpackb


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
    cfg = tx.ExportConfig(speed_limits={'tunnel': 35})
    meta, zones = tx.export(m, tmp_path / 'pkg', cfg, source=tmp_path / 'm.msgpack')
    for f in ('nav_map.pgm', 'nav_map.yaml', 'speed_mask.pgm', 'direction_map.png', 'zones.yaml', 'terrain.json'):
        assert (tmp_path / 'pkg' / f).is_file()
    raw = (tmp_path / 'pkg' / 'nav_map.pgm').read_bytes()
    img = np.frombuffer(raw[-80 * 60:], np.uint8).reshape(60, 80)[::-1]
    assert img[30, 0] == 100                             # raw-mode lethal
    assert 0 < img[30, 1] <= cfg.max_band_cost             # band capped below lethal
    assert img[30, 70] == 0                               # raw-mode free
    assert len(zones) == 1
    z = zones[0]
    assert z['type'] == 'tunnel' and z['yaw_deg'] == 0.0
    assert z['size'] == pytest.approx([1.0, 0.4])
    assert z['centre'] == pytest.approx([-2.0 + 2.0, -1.5 + 1.2])
    assert z['entry'][0] < z['centre'][0] < z['exit'][0]
    assert yaml_load(tmp_path / 'pkg' / 'nav_map.yaml')['origin'][:2] == [-2.0, -1.5]
    assert yaml_load(tmp_path / 'pkg' / 'nav_map.yaml')['mode'] == 'raw'


def test_unknown_roundtrip_and_export_preserves_three_states(tmp_path):
    m = TerrainMap.blank(12, 6, .1)
    m.terrain[:, :3] = UNKNOWN
    m.terrain[:, -1] = OBSTACLE
    m.elevation = np.zeros(m.terrain.shape, np.float32)
    m.elevation[:, :3] = np.nan
    meta, _ = tx.export(m, tmp_path, tx.ExportConfig(), source='test')
    loaded = TerrainMap.load(tmp_path / 'terrain.msgpack')
    np.testing.assert_array_equal(loaded.terrain, m.terrain)
    raw = (tmp_path / 'nav_map.pgm').read_bytes()
    grid = np.frombuffer(raw[-72:], np.uint8).reshape(6, 12)[::-1]
    assert np.all(grid[:, :3] == 255)
    assert np.all(grid[:, -1] == 100)
    assert np.all(grid[:, 3:7] == 0)  # No cost halo around the unknown boundary.
    assert meta['label_cells']['unknown'] == 18
    assert meta['unknown_cells'] == 18
    assert meta['label_cells']['obstacle'] == 6
    with np.load(tmp_path / 'ground_z.npz') as z:
        assert np.isnan(z['ground_z'][:, :3]).all()
        assert np.isnan(z['ground_z'][:, -1]).all()


def test_unknown_does_not_acquire_direction_from_nearby_terrain():
    terrain = np.full((11, 11), UNKNOWN, np.uint8)
    direction = np.zeros_like(terrain)
    terrain[5, 5] = SLOPE
    terrain[5, 4] = FLAT
    angle, magnitude, labels, _ = tx.inflate_direction(terrain, direction, .05, .1, .3, 24, .9)
    unknown = terrain == UNKNOWN
    assert np.all(labels[unknown] == UNKNOWN)
    assert not magnitude[unknown].any()
    assert not angle[unknown].any()
    assert magnitude[5, 4] > 0


def test_legacy_missing_elevation_upgrades_without_erasing_observed_obstacles(tmp_path):
    m = TerrainMap.blank(4, 3, .1)
    m.elevation = np.zeros(m.terrain.shape, np.float32)
    m.terrain[0, 0] = OBSTACLE
    m.elevation[0, 0] = np.nan
    m.terrain[1, 1] = OBSTACLE  # An observed obstacle must survive the upgrade.
    meta, _ = tx.export(m, tmp_path, tx.ExportConfig(), source='legacy')
    upgraded = TerrainMap.load(tmp_path / 'terrain.msgpack')
    assert upgraded.terrain[0, 0] == UNKNOWN
    assert upgraded.terrain[1, 1] == OBSTACLE
    assert m.terrain[0, 0] == OBSTACLE  # Export does not mutate the caller's object.
    assert meta['unknown_cells'] == 1


def test_goal_picker_excludes_unknown_raw_pixels_and_supports_legacy_maps():
    import runpy
    picker = Path(__file__).resolve().parents[4] / 'tools/nav_regress/pick_flat_goals.py'
    free_cells = runpy.run_path(str(picker))['free_map_cells']
    assert free_cells(np.array([0, 1, 2, 100, 255]), 'raw', 250).tolist() == [True, True, False, False, False]
    assert free_cells(np.array([255, 250, 249, 0]), 'scale', 250).tolist() == [True, True, False, False]


def test_multi_direction_patch_splits_into_faces():
    # a 4x4 ring of SLOPE cells, each side facing its own uphill way: one label component, four faces
    m = TerrainMap.blank(6, 6, 0.05, (0.0, 0.0))
    m.terrain[1:5, 1:5] = SLOPE
    m.terrain[2:4, 2:4] = 0                              # hollow centre, so only the ring is slope
    m.direction[1, 1:5] = encode_angle(0.5 * math.pi)    # bottom row faces +y
    m.direction[4, 1:5] = encode_angle(1.5 * math.pi)    # top row faces -y
    m.direction[1:5, 1] = encode_angle(0.0)              # left column faces +x
    m.direction[1:5, 4] = encode_angle(math.pi)          # right column faces -x
    comp, n = tx.direction_components(m, SLOPE, 20.0)
    assert n == 4
    assert int((comp > 0).sum()) == 12                     # every slope cell keeps exactly one face
    assert sorted(np.bincount(comp.ravel())[1:].tolist()) == [2, 2, 4, 4]   # corners follow the columns
    # 180 deg disables the cut: back to a single component, no warning any more only because one face
    comp, n = tx.direction_components(m, SLOPE, 180.0)
    assert n == 1
    zones = tx.region_zones(m, 0.0, 20.0)
    assert len(zones) == 4
    assert all('warning' not in z for z in zones)          # every face has one clean direction
    for z in zones:                                        # the axis is one of the four faces, not a mean
        d = min(abs((z['yaw_deg'] - a + 180.0) % 360.0 - 180.0) for a in (0.0, 90.0, 180.0, -90.0))
        assert d < 2.0                                     # uint8 angle quantisation is 1.41 deg


def test_direction_split_disabled_keeps_old_behaviour():
    m = TerrainMap.blank(3, 3, 0.05, (0.0, 0.0))
    m.terrain[0:3, 0:3] = SLOPE
    m.direction[0, 0:3] = encode_angle(0.0)
    m.direction[1, 0:3] = encode_angle(0.5 * math.pi)
    m.direction[2, 0:3] = encode_angle(math.pi)
    assert len(tx.region_zones(m, 0.0, 20.0)) == 3
    assert len(tx.region_zones(m, 0.0, 180.0)) == 1


def test_bad_split_angle_rejected():
    m = TerrainMap.blank(2, 2, 0.05)
    for bad in (0.0, -1.0, 181.0):
        with pytest.raises(ValueError):
            tx.direction_components(m, SLOPE, bad)


def yaml_load(p):
    import yaml
    return yaml.safe_load(Path(p).read_text())
