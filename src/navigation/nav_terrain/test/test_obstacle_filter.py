
import numpy as np
import pytest

from nav_terrain.obstacle_filter import FilterConfig, GroundFilter, LiveGroundFilter
from nav_terrain.mapping_ground import ground_evidence
from nav_terrain.terrain_map import OBSTACLE, UNKNOWN, TerrainMap


def terrain():
    m = TerrainMap.blank(100, 80, 0.1, (-5., -4.))
    m.elevation = np.zeros(m.terrain.shape, np.float32)
    m.clearance = np.full(m.terrain.shape, np.inf, np.float32)
    return m


def test_mapping_ground_accepts_only_observed_uncontested_cells():
    points = np.array([[-0.24, 0.05, 1.01], [0.02, 0.05, 1.0], [0.04, 0.05, 1.15],
                       [0.22, 0.05, 2.0], [0.38, 0.05, 0.85], [0.39, 0.04, 1.0],
                       [np.nan, 0.0, 1.0], [5.0, 0.0, 1.0]])
    grid, origin = ground_evidence(points, [-0.02, 0.0, 1.062], 0.062, FilterConfig(), 0.15, 1.0)
    def cost(world_x):
        col, row = np.floor(([world_x, 0.05] - origin) / 0.15).astype(int)
        return grid[row, col]
    assert cost(-0.24) == 0
    assert all(cost(world_x) == -1 for world_x in (-0.5, 0.02, 0.22, 0.38))


def test_ground_roof_self_and_invalid_returns_are_not_obstacles():
    m = terrain()
    m.clearance[:] = 0.25
    f = GroundFilter(m)
    points = np.array([[1, 0, 0], [2, 0, .15], [3, 0, .25], [.1, .1, .15], [np.nan, 0, 0]])
    hits, rays = f.select(points, points, (0, 0, .1), 0)
    assert hits.tolist() == [1]
    assert rays.tolist() == [1]  # stop the clearing ray at the obstacle, never at the roof


def test_live_filter_needs_no_terrain_and_never_clears_behind_a_hit():
    points = np.array([[3, 0, 1.0], [1, 0, 1.15], [0, 2, 1.01],
                       [-2, 0, 1.5], [0.1, 0.1, 1.15], [np.nan, 0, 1.0]])
    body = points - [0, 0, 1.062]
    hits, rays = LiveGroundFilter().select(points, body, (0, 0, 1.2), 1.0)
    assert hits.tolist() == [1]
    assert set(rays) == {1, 2}


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


def test_unknown_terrain_uses_ground_fallback_without_hiding_obstacles():
    m = terrain()
    m.terrain[35:46, 55:66] = UNKNOWN
    m.elevation[35:46, 55:66] = .15  # A stale elevation is not observed traversable ground.
    f = GroundFilter(m, FilterConfig(fill_radius=0))
    points = np.array([[1., 0., .15], [2., 0., 0.]])
    hits, _ = f.select(points, points, (0, 0, .1), 0)
    assert hits.tolist() == [0]


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


def test_mapping_ground_carves_floor_along_observed_rays():
    angles = np.linspace(0.0, 2.0 * np.pi, 720, endpoint=False)
    points = np.stack([3.0 * np.cos(angles), 3.0 * np.sin(angles), np.full(angles.size, 1.0)], axis=1)
    grid, origin = ground_evidence(points, [0.0, 0.0, 1.062], 0.062, FilterConfig(), 0.05, 4.0)

    def cost(x, y=0.0):
        col, row = np.floor(([x, y] - origin) / 0.05).astype(int)
        return grid[row, col]

    # Everything between the robot and the ring is observed floor, not just the hits.
    assert all(cost(distance) == 0 for distance in (0.25, 0.8, 1.5, 2.0, 2.9))
    assert cost(3.5) == -1


def test_mapping_ground_stops_carving_at_the_nearest_obstacle():
    angles = np.linspace(0.0, 2.0 * np.pi, 720, endpoint=False)
    points = np.stack([3.0 * np.cos(angles), 3.0 * np.sin(angles), np.full(angles.size, 1.0)], axis=1)
    points = np.vstack([points, [[1.5, 0.0, 1.15]]])
    grid, origin = ground_evidence(points, [0.0, 0.0, 1.062], 0.062, FilterConfig(), 0.05, 4.0)

    def cost(x):
        col, row = np.floor(([x, 0.0] - origin) / 0.05).astype(int)
        return grid[row, col]

    assert cost(1.2) == 0      # floor in front of the obstacle stays observed
    assert cost(2.0) == -1     # nothing behind an obstacle is declared clear
    assert cost(-2.0) == 0     # the opposite ray is unaffected
