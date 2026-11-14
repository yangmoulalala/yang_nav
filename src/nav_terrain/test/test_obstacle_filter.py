import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from obstacle_filter import FilterConfig, GroundFilter
from terrain_map import OBSTACLE, TerrainMap


def terrain():
    m = TerrainMap.blank(100, 80, 0.1, (-5., -4.))
    m.elevation = np.zeros(m.terrain.shape, np.float32)
    m.clearance = np.full(m.terrain.shape, np.inf, np.float32)
    return m


def test_ground_roof_self_and_invalid_returns_are_not_obstacles():
    m = terrain()
    m.clearance[:] = 0.25
    f = GroundFilter(m)
    points = np.array([[1, 0, 0], [2, 0, .15], [3, 0, .25], [.1, .1, .15], [np.nan, 0, 0]])
    hits, rays = f.select(points, points, (0, 0, .1), 0)
    assert hits.tolist() == [1]
    assert rays.tolist() == [1]  # stop the clearing ray at the obstacle, never at the roof


def test_slope_uses_local_ground_and_keeps_low_box():
    m = terrain()
    x = m.origin[0] + (np.arange(m.width) + .5) * m.resolution
    m.elevation[:] = .35 * x
    points = np.array([[1.05, 0, .35 * 1.05], [2.05, 0, .35 * 2.05 + .12]])
    f = GroundFilter(m)
    hits, _ = f.select(points, points, (0, 0, .1), 0)
    assert hits.tolist() == [1]


def test_known_terrain_riser_is_not_reported_as_a_dynamic_obstacle():
    m = terrain()
    # A 12 cm registered step: the lower cell can see either the upper
    # surface or the vertical riser, while a level-ground box remains dynamic.
    m.elevation[40:, :] = 0.0
    m.elevation[:40, :] = 0.12
    f = GroundFilter(m)
    points = np.array([[1.0, 0.0, 0.10], [2.0, 1.0, 0.10], [3.0, -0.05, 0.15]])
    hits, _ = f.select(points, points, (0, 0, 0.1), 0)
    assert hits.tolist() == [1]


def test_dynamic_obstacle_near_but_not_on_registered_riser_is_retained():
    m = terrain()
    m.elevation[:40, :] = .12
    f = GroundFilter(m)
    # The first return is on the mapped step boundary; the second is 4 cm
    # inside the lower cell and must remain an obstacle candidate.
    points = np.array([[1., 0., .10], [1., .04, .10]])
    hits, _ = f.select(points, points, (0, 0, .1), 0)
    assert hits.tolist() == [1]


def test_riser_filter_does_not_wrap_between_distant_map_edges():
    m = terrain()
    m.elevation[-10:, :] = .12
    f = GroundFilter(m)
    point = np.array([[1., -3.95, .10]])
    hits, _ = f.select(point, point, (0, 0, .1), 0)
    assert hits.tolist() == [0]


def test_moving_obstacle_clears_only_after_observed_ground():
    f = GroundFilter(terrain())
    blocked = np.array([[1., 0., .12], [2., 0., 0.]])
    hits, rays = f.select(blocked, blocked, (0, 0, .1), 0)
    assert hits.tolist() == [0] and rays.tolist() == [0]
    free = np.array([[1., 0., 0.], [2., 0., 0.]])
    hits, rays = f.select(free, free, (0, 0, .1), 0)
    assert not len(hits) and rays.tolist() == [1]
    roof = np.array([[2., 0., 1.]])
    hits, rays = f.select(roof, roof, (0, 0, .1), 0)
    assert not len(hits) and not len(rays)


def test_unknown_and_obstacle_cells_do_not_use_obstacle_top_as_ground():
    m = terrain()
    m.terrain[35:46, 55:66] = OBSTACLE
    m.elevation[35:46, 55:66] = .15
    f = GroundFilter(m)
    points = np.array([[1., 0., .15], [-5.01, 0., .15]])
    hits, _ = f.select(points, points, (0, 0, .1), 0)
    assert hits.tolist() == [0, 1]  # floor() matters just beyond the negative grid boundary


def test_height_and_clearance_roundtrip(tmp_path):
    m = terrain()
    m.clearance[40, 60] = .25
    path = tmp_path / 'terrain.msgpack'
    m.save(path)
    loaded = TerrainMap.load(path)
    np.testing.assert_equal(loaded.elevation, m.elevation)
    np.testing.assert_equal(loaded.clearance, m.clearance)


def test_missing_ground_and_bad_configuration_fail_startup():
    with pytest.raises(ValueError, match='elevation'):
        GroundFilter(TerrainMap.blank(3, 3, .1))
    with pytest.raises(ValueError):
        GroundFilter(terrain(), FilterConfig(min_height=.2, max_height=.1))


def test_range_and_empty_input():
    f = GroundFilter(terrain())
    p = np.array([[8, 0, .1], [1, 0, .1]])
    hits, _ = f.select(p, p, (0, 0, 0), 0)
    assert hits.tolist() == [1]
    hits, rays = f.select([], [], (0, 0, 0), 0)
    assert not len(hits) and not len(rays)


def test_ceiling_rejection_is_not_just_max_height_clipping():
    m = terrain()
    m.clearance[:] = .20
    f = GroundFilter(m)
    p = np.array([[1., 0., .20], [2., 0., .12]])
    hits, rays = f.select(p, p, (0, 0, .1), 0)
    assert hits.tolist() == [1]  # roof is inside the configured obstacle height band
    assert rays.tolist() == [1]
