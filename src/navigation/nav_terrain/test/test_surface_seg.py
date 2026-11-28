import math
import json
import struct

import numpy as np
import pytest

from nav_terrain import registration, surface_project as sp
from nav_terrain import surface_seg as ss
from nav_terrain.obstacle_filter import GroundFilter
from nav_terrain.surface_seg._regions import _absorb_clutter
from nav_terrain.terrain_map import OBSTACLE, UNKNOWN, SLOPE, TUNNEL, UNDULATION, FLAT, TerrainMap, decode_angle, encode_angle


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
    t += box(3.5, 4.5, 2.6, 3.4, 0.30, 0.6)       # roof
    # ramp 1.0 m wide, rises 0.20 m over 1.0 m along +y (11.3 deg), then a plateau
    t += quad((1.0, 0.5, 0.0), (2.0, 0.5, 0.0), (2.0, 1.5, 0.2), (1.0, 1.5, 0.2))
    t += quad((1.0, 1.5, 0.2), (2.0, 1.5, 0.2), (2.0, 2.5, 0.2), (1.0, 2.5, 0.2))
    return np.array(t, float)


def sample_mesh(tri, n, rng):
    """n points uniformly on the mesh surface (synthetic prior cloud)."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    idx = rng.choice(len(tri), n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    return a[idx] + u[:, None] * (b[idx] - a[idx]) + v[:, None] * (c[idx] - a[idx])


def params():
    return ss.SegParams(resolution=0.05)


def proj():
    return sp.Project.from_mesh(field(), params(), 'field.stl', log=lambda *_: None)


def pid_at(p, x, y):
    c, r = p.surf.cell_of(x, y)
    return p.patch_at(c, r)


def test_patches_carry_geometry_only():
    p = proj()
    floor, ramp, top, tun = (pid_at(p, *xy) for xy in ((0.5, 3.5), (1.5, 1.0), (1.5, 2.0), (4.0, 3.0)))
    assert len({floor, ramp, top, tun}) == 4
    r = p.info(ramp)
    assert r['kind'] == 'slope' and abs(r['incline'] - 11.3) < 0.5 and abs(r['azimuth'] - 90) < 1
    assert abs(r['area'] - 1.0) < 0.1 and abs(r['length'] - 1.0) < 0.1
    t = p.info(top)
    assert t['kind'] == 'plane' and abs(t['z'] - 0.2) < 1e-3
    assert abs(p.info(tun)['clearance'] - 0.30) < 1e-6 and p.info(floor)['clearance'] == math.inf
    assert any(n == top and abs(dz - 0.2) < 0.03 for n, _, dz in p.neighbours(floor))
    assert not p.ann                                            # nothing guessed


def test_map_landmarks_archive_export_and_history(tmp_path):
    project = proj()
    with pytest.raises(ValueError):
        project.set_landmark('home', [0, 0])
    project.frame = 'map'
    col, row = project.surf.cell_of(1.5, 1.)
    for name, offset in [('home', 0), ('target', 1), ('match', 2)]:
        project.set_landmark(name, [col + offset, row])
        assert project.landmarks[name] == project.surf.centre(col + offset, row)
    expected = project.landmarks.copy()
    assert project.match_yaw_deg is None
    for invalid in (True, '90', math.nan, math.inf, 181):
        with pytest.raises(ValueError):
            project.set_match_yaw(invalid)
    project.set_match_yaw(0)
    assert project.undo() and project.match_yaw_deg is None
    assert project.redo() and project.match_yaw_deg == 0
    project.set_match_yaw(90)
    project.set_landmark('match', None)
    assert project.match_yaw_deg is None
    with pytest.raises(ValueError):
        project.set_match_yaw(-90)
    assert project.undo() and project.match_yaw_deg == 90
    project.set_landmark('target', None)
    assert 'target' not in project.landmarks
    assert project.undo() and project.landmarks == expected
    assert project.redo() and 'target' not in project.landmarks
    project.undo()
    with pytest.raises(ValueError):
        project.set_landmark('home', [-1, row])
    archive = tmp_path / 'test.surf.npz'
    project.save(archive)
    loaded = sp.Project.load(archive)
    assert loaded.landmarks == expected
    assert loaded.match_yaw_deg == 90
    output = tmp_path / 'terrain'
    loaded.export(output)
    points = {name: list(point) for name, point in expected.items()}
    assert json.loads((output / 'terrain.json').read_text())['landmarks'] == points
    assert TerrainMap.load(output / 'terrain.msgpack').extra['landmarks'] == points
    assert json.loads((output / 'terrain.json').read_text())['match_yaw_deg'] == 90
    assert TerrainMap.load(output / 'terrain.msgpack').extra['match_yaw_deg'] == 90


def test_annotate_extent_undo_resegment_save(tmp_path):
    p = proj()
    ramp, top, tun = (pid_at(p, *xy) for xy in ((1.5, 1.0), (1.5, 2.0), (4.0, 3.0)))
    p.set_ann([ramp], type=SLOPE)
    assert p.direction(ramp) == pytest.approx(90, abs=1)
    p.flip_dir([ramp])
    assert p.direction(ramp) == pytest.approx(270, abs=1)
    p.set_ann([tun], type=TUNNEL, passable=False)
    n0 = p.info(ramp)['n']
    p.grow(ramp, 2)
    assert p.info(ramp)['n'] > n0
    p.shrink(ramp, 2)
    p.undo(); p.undo()                                           # noqa: E702
    assert p.info(ramp)['n'] == n0
    p.redo()
    assert p.info(ramp)['n'] > n0
    p.reset_extent([ramp])
    assert p.info(ramp)['n'] == n0
    keep = p.merge([top, ramp])
    assert keep == ramp and p.info(top) is None and p.a(ramp)['type'] == SLOPE
    p.undo()
    assert p.info(top) is not None
    p.save(tmp_path / 'p.npz')
    q = sp.Project.load(tmp_path / 'p.npz')
    assert q.a(tun) == {'type': TUNNEL, 'passable': False, 'dir': None, 'rev': False} and (q.owner == p.owner).all()
    q.params.min_area = 0.06
    q.segment(log=lambda *_: None)                              # annotations follow the overlap
    assert q.a(pid_at(q, 1.5, 1.0))['type'] == SLOPE and q.a(pid_at(q, 4.0, 3.0))['passable'] is False


def test_export(tmp_path):
    p = proj()
    ramp, tun, floor = (pid_at(p, *xy) for xy in ((1.5, 1.0), (4.0, 3.0), (0.5, 3.5)))
    p.set_ann([floor], type=FLAT)
    p.set_ann([ramp], type=SLOPE)
    p.set_ann([tun], type=TUNNEL, passable=False)
    meta, zones = p.export(tmp_path / 'out')
    m = TerrainMap.load(tmp_path / 'out' / 'terrain.msgpack')
    c, r = m.cell_of(1.5, 1.0)
    assert m.terrain[r, c] == SLOPE and abs(decode_angle(m.direction[r, c]) - math.pi / 2) < 0.03
    c, r = m.cell_of(4.0, 3.0)
    assert m.terrain[r, c] == OBSTACLE                          # not passable
    assert meta['blocked_cells'] == p.info(tun)['n'] and meta['unannotated_cells'] > 0
    assert [z['type'] for z in zones] == ['slope']
    assert (tmp_path / 'out' / 'nav_map.yaml').is_file()


def test_cloud_registration_transfer():
    f = proj()
    ramp = pid_at(f, 1.5, 1.0)
    f.set_ann([ramp], type=SLOPE)
    f.set_ann([pid_at(f, 0.5, 3.5)], type=UNDULATION, dir=30.0)
    yaw, t = 0.3, np.array([0.4, -0.2, 0.0])
    R = registration.rot_z(yaw)
    pts_f = sample_mesh(field(), 120000, np.random.default_rng(1))
    c = sp.Project.from_cloud((pts_f - t) @ R, params(), 'c.pcd', log=lambda *_: None)   # p_f = R p_m + t
    c.register_to(f, [0.45, -0.15, 0.0], math.degrees(yaw) + 2.0, log=lambda *_: None)
    Rr, tr = registration.reg_matrix(c.reg)
    assert np.allclose(tr[:2], t[:2], atol=0.03) and abs(math.atan2(Rr[1, 0], Rr[0, 0]) - yaw) < math.radians(0.5)
    hole = c.surf.cell_of(*(R.T @ (np.array([1.5, 0.8, 0.0]) - t))[:2])
    c.surf.z[hole[1] - 2:hole[1] + 3, hole[0] - 2:hole[0] + 3] = np.nan     # scan shadow on the ramp
    assert c.adopt_from(f, log=lambda *_: None) == 2
    col, row = c.surf.cell_of(*(R.T @ (np.array([1.5, 0.8, 0.0]) - t))[:2])
    assert c.patch_at(col, row) == ramp and np.isfinite(c.surf.z[row, col])
    pm = R.T @ (np.array([1.5, 1.0, 0.0]) - t)
    assert pid_at(c, pm[0], pm[1]) == ramp                    # model patch, not a cloud fragment
    assert set(c.ids()) <= set(f.ids())
    a = c.a(pid_at(c, pm[0], pm[1]))
    assert a['type'] == SLOPE and abs(a['dir'] - (90 - math.degrees(yaw))) < 2
    pm = R.T @ (np.array([0.5, 3.5, 0.0]) - t)
    a = c.a(pid_at(c, pm[0], pm[1]))
    assert a['type'] == UNDULATION and abs(a['dir'] - (30 - math.degrees(yaw))) < 0.5   # manual direction, rotated by the registration


def test_cloud_registration_transfers_saved_map_annotations(tmp_path):
    points = sample_mesh(field(), 120000, np.random.default_rng(1))
    reference = sp.Project.from_cloud(points, params(), log=lambda *_: None)
    patch = pid_at(reference, 0.5, 3.5)
    reference.set_ann([patch], type=UNDULATION, dir=30.0, passable=False)
    archive = tmp_path / 'reference.surf.npz'
    reference.save(archive)
    reference = sp.Project.load(archive)

    translation = np.array([0.40, -0.20, 0.0])
    cloud = sp.Project.from_cloud(points - translation, params(), log=lambda *_: None)
    registration_result = cloud.register_to(reference, translation, 0.0, log=lambda *_: None)
    assert registration_result['reference_frame'] == 'map'
    assert registration_result['t_world_map'] == pytest.approx(translation, abs=0.03)

    col, row = cloud.surf.cell_of(0.5 - translation[0], 3.5 - translation[1])
    cloud.surf.z[row, col] = np.nan
    assert cloud.field_lookup(reference)[row, col] == patch
    assert cloud.adopt_from(reference, log=lambda *_: None) == 1
    col, row = cloud.surf.cell_of(0.5 - translation[0], 3.5 - translation[1])
    assert np.isfinite(cloud.surf.z[row, col]) and cloud.patch_at(col, row) == patch
    assert cloud.reference_fill[row, col]
    annotation = cloud.a(pid_at(cloud, 0.6 - translation[0], 3.5 - translation[1]))
    assert annotation['type'] == UNDULATION and not annotation['passable']
    assert annotation['dir'] == pytest.approx(30.0, abs=0.5)


def test_no_surface_observation_exports_as_unknown(tmp_path):
    p = proj()
    floor = pid_at(p, 0.5, 3.5)
    p.set_ann([floor], type=FLAT)
    c, r = p.surf.cell_of(0.5, 3.5)
    p.surf.z[r, c] = np.nan  # Even a previously annotated patch cannot invent coverage.
    m, report = p.terrain_map()
    assert m.terrain[r, c] == UNKNOWN
    assert report['unknown_cells'] == report['no_data_cells']
    assert report['unknown_cells'] == np.count_nonzero(~np.isfinite(p.surf.z))
    observed_unannotated = np.isfinite(p.surf.z) & (p.label_grids()[0] < 0)
    assert np.all(m.terrain[observed_unannotated] == OBSTACLE)
    p.export(tmp_path)
    assert TerrainMap.load(tmp_path / 'terrain.msgpack').terrain[r, c] == UNKNOWN


def test_adopt_extends_cropped_map_without_erasing_evidence(tmp_path):
    def project(heights, origin, resolution, labels):
        heights = np.array(heights, dtype=np.float32)
        surface = ss.Surface(heights, resolution, origin, np.full_like(heights, np.inf))
        result = sp.Project(surface, params(), 'map')
        result._replace_patches(np.asarray(labels, dtype=np.int32), {})
        result.grad = ss.cell_gradient(surface, result.params)
        return result

    reference = project(np.zeros((4, 6)), (1., -2.), .5, [[1, 1, 2, 2, 3, 4]] * 4)
    reference.set_ann([1], type=FLAT)
    reference.set_ann([2], type=TUNNEL, dir=120.)
    reference.set_ann([3], type=OBSTACLE, passable=False)
    reference.surf.clearance[:, 2:4] = .32
    reference.surf.z[1, 2] = np.nan
    current = project([[.20, .50], [np.nan, .20]], (.5, 1.), .25, [[1, 2], [0, 1]])
    current.set_ann([2], type=OBSTACLE, passable=False)
    current.surf.clearance[0, 1] = .08
    current.reg = dict(rpy_deg=[0, 0, 90], t_world_map=[3., -2., -.20])
    current.landmarks['home'] = (.625, 1.125)
    current.adopt_from(reference, log=lambda *_: None)
    assert current.shape[0] > 2 and current.shape[1] > 2
    def cell(x_value, y_value):
        col, row = current.surf.cell_of(x_value, y_value)
        return row, col
    observed = cell(.625, 1.125)
    conflict = cell(.875, 1.125)
    shadow = cell(.625, 1.375)
    tunnel = cell(1.625, .625)
    wall = cell(1.625, -.375)
    no_source = cell(.625, .875)
    outside_unset = cell(.125, -.875)
    grid, _ = current.terrain_map()
    assert current.landmarks['home'] == (.625, 1.125)
    assert not current.reference_fill[observed] and grid.terrain[observed] == FLAT
    assert not current.reference_fill[conflict] and grid.terrain[conflict] == FLAT
    assert grid.elevation[conflict] == pytest.approx(.20) and np.isinf(grid.clearance[conflict])
    assert current.reference_fill[shadow] and grid.elevation[shadow] == pytest.approx(.20)
    assert grid.terrain[tunnel] == TUNNEL and grid.clearance[tunnel] == pytest.approx(.32)
    assert current.a(2)['dir'] == pytest.approx(30.) and grid.terrain[wall] == OBSTACLE
    assert grid.terrain[no_source] == UNKNOWN
    assert outside_unset[0] < 0 or outside_unset[1] < 0 or grid.terrain[outside_unset] == UNKNOWN
    archive = tmp_path / 'adopted.surf.npz'
    current.save(archive)
    restored = sp.Project.load(archive)
    assert np.array_equal(restored.reference_fill, current.reference_fill)
    restored.export(tmp_path / 'out')
    exported = TerrainMap.load(tmp_path / 'out/terrain.msgpack')
    assert np.array_equal(exported.terrain, grid.terrain)
    assert exported.elevation[conflict] == pytest.approx(.20) and np.isinf(exported.clearance[conflict])
    raster = (tmp_path / 'out/nav_map.pgm').read_bytes().split(b'\n', 3)[3]
    static_cost = np.frombuffer(raster, dtype=np.uint8).reshape(exported.terrain.shape)[::-1]
    assert static_cost[conflict] < 100 and static_cost[wall] == 100
    live_points = np.array([[.875, 1.125, .20], [.875, 1.125, .35]])
    hits, _ = GroundFilter(exported).select(live_points, live_points, [0., 0., .20], .20)
    assert hits.tolist() == [1]
    assert exported.clearance[tunnel] == pytest.approx(.32)
    assert exported.extra['reference_filled_cells'] == current.reference_fill.sum()
    reference.surf.z[:, 4] = np.nan
    reference.ann.pop(1)
    assert restored.measured_surface().z[conflict] == pytest.approx(.50)
    assert restored.measured_surface().clearance[conflict] == pytest.approx(.08)
    restored.adopt_from(reference, log=lambda *_: None)
    revised = restored.terrain_map()[0]
    assert revised.terrain[wall] == UNKNOWN
    assert revised.terrain[conflict] == OBSTACLE and revised.elevation[conflict] == pytest.approx(.50)


def test_reference_back_projection_respects_tilt():
    surface = ss.Surface(np.full((4, 4), np.nan, np.float32), .1, (1., 1.), np.full((4, 4), np.inf))
    reference = ss.Surface(np.full((80, 80), .8, np.float32), .1, (-3., -3.), np.full((80, 80), np.inf))
    reg = dict(rpy_deg=[10., -8., 20.], t_world_map=[.1, .2, .3])
    projection = registration.FieldProjection(surface, reference, reg)
    assert projection.inside.all()
    surface.z.fill(2.5)
    with_obstacle = registration.FieldProjection(surface, reference, reg)
    assert np.array_equal(projection.field_indices, with_obstacle.field_indices)
    np.testing.assert_allclose(projection.map_height, with_obstacle.map_height)
    for row, col in np.ndindex(surface.z.shape):
        map_point = [*surface.centre(col, row), projection.map_height[row, col]]
        transformed = projection.rotation @ map_point + projection.translation
        assert transformed[2] == pytest.approx(.8)
        ref_col, ref_row = reference.cell_of(*transformed[:2])
        index = row * surface.z.shape[1] + col
        assert tuple(axis[index] for axis in projection.field_indices) == (ref_row, ref_col)


def test_vertical_view_two_direction_slope():
    # ridge: up along +x over x 2..3, down over x 3..4 (11.3 deg each side) on a floor
    t = quad((0, 0, 0), (6, 0, 0), (6, 4, 0), (0, 4, 0))
    t += quad((2, 1, 0), (3, 1, 0.2), (3, 3, 0.2), (2, 3, 0))
    t += quad((3, 1, 0.2), (4, 1, 0), (4, 3, 0), (3, 3, 0.2))
    p = sp.Project.from_mesh(np.array(t, float), params(), 'ridge.stl', log=lambda *_: None)
    a, b = pid_at(p, 2.5, 2.0), pid_at(p, 3.5, 2.0)
    assert a != b                                               # two faces seen from above
    assert abs(p.info(a)['azimuth'] - 0) < 0.5 and abs(p.info(b)['azimuth'] - 180) < 0.5
    assert abs(p.info(a)['rms']) < 1e-6
    k = p.merge([a, b])
    p.set_ann([k], type=SLOPE)
    assert sorted(round(d) for d in p.info(k)['dirs']) == [0, 180] and p.is_local(k)
    assert p.info(k)['kind'] == 'slope' and p.info(k)['multi'] and abs(p.info(k)['incline'] - 11.3) < 0.5
    assert len(p.arrows(k)) == 2

    def code(x, y):
        m, _ = p.terrain_map()
        c, r = m.cell_of(x, y)
        return int(m.direction[r, c])
    assert code(2.5, 2.0) == encode_angle(0.0) and code(3.5, 2.0) == encode_angle(math.pi)
    p.flip_dir([k])
    assert code(2.5, 2.0) == encode_angle(math.pi) and code(3.5, 2.0) == encode_angle(0.0)
    p.set_ann([k], dir=45.0)                                    # manual: one direction for the whole patch
    assert not p.is_local(k) and code(2.5, 2.0) == code(3.5, 2.0) == encode_angle(math.radians(45))


def test_cut_level_select_default_obstacle():
    p = proj()
    floor = pid_at(p, 0.5, 3.5)
    p.set_ann([floor], type=FLAT)
    a = p.surf.cell_of(3.0, -0.5)
    b = p.surf.cell_of(3.0, 4.5)
    ids = p.split_line(a, b, [floor])                          # cut the floor in two at x = 3
    assert len(ids) == 2 and pid_at(p, 2.5, 3.5) != pid_at(p, 5.5, 3.5)
    right = pid_at(p, 5.5, 3.5)
    assert p.a(right)['type'] == FLAT                           # pieces keep the annotation
    p.set_ann([right], passable=False)
    m, rep = p.terrain_map()
    c, r = m.cell_of(5.5, 3.5)
    assert m.terrain[r, c] == OBSTACLE
    c, r = m.cell_of(1.5, 1.0)
    assert m.terrain[r, c] == OBSTACLE and rep['unannotated_cells'] > 0   # not annotated = obstacle
    p.undo(); p.undo()                                          # noqa: E702
    assert pid_at(p, 2.5, 3.5) == pid_at(p, 5.5, 3.5)
    q = proj()
    lvl = q.flood_level([pid_at(q, 0.5, 3.5)])
    assert pid_at(q, 1.5, 1.0) not in lvl and pid_at(q, 1.5, 2.0) not in lvl   # stops at slope / plateau


def test_cloud_surface_floor_under_roof():
    pts = sample_mesh(field(), 200000, np.random.default_rng(2))
    s = ss.cloud_surface(pts, params(), log=lambda *_: None)
    c, r = s.cell_of(4.0, 3.0)
    assert abs(s.z[r, c]) < 0.01 and abs(s.clearance[r, c] - 0.30) < 0.03       # floor, not roof
    c, r = s.cell_of(1.5, 1.0)
    assert abs(s.z[r, c] - 0.10) < 0.02


def test_clutter_fragments_absorbed():
    lab = np.ones((10, 10), np.int64)
    lab[4, 4], lab[0, 0], lab[0, 1] = 2, 3, 4
    z = np.zeros((10, 10))
    z[0, 0] = z[0, 1] = 1.0
    out = _absorb_clutter(lab, z, 5, 0.06)
    assert out[4, 4] == 1                                       # speckle at floor height joins the floor
    assert out[0, 0] == out[0, 1] != 1                          # high fragments: one clutter patch


def test_resegment_does_not_move_annotations_without_overlap():
    heights = np.array([[0.0] * 4, [np.nan] * 4], np.float32)
    surface = ss.Surface(heights, 0.1, (0.0, 0.0), np.full_like(heights, np.inf))
    project = sp.Project(surface, params(), 'field')
    project.segment()
    project.set_ann(project.ids(), type=SLOPE)
    surface.z = heights[::-1].copy()
    assert project.segment() == 0
    assert not project.ann and not project.undo()
    assert np.array_equal(project.owner > 0, np.isfinite(surface.z))


def write_stl(path, tri):
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80 + struct.pack('<I', len(tri)))
        for t in tri:
            fh.write(struct.pack('<3f', 0, 0, 0) + t.astype('<f4').tobytes() + b'\0\0')


def test_editor_preview_from_stl(tmp_path):
    from nav_terrain.mesh_io import load_mesh
    from nav_terrain.studio_render import compose_map
    from nav_terrain.terrain_export import COLORS

    source = tmp_path / 'field.stl'
    write_stl(source, field())
    project = sp.Project.from_mesh(load_mesh(source), params())
    ramp = pid_at(project, 1.5, 1.0)
    assert ramp > 0
    options = dict(show_unset=True, opacity=0.72, edges=True, overlay=None, selected=[], stroke=None)
    before = compose_map(project, COLORS, **options)
    project.set_ann([ramp], type=SLOPE, dir=30.0)
    after = compose_map(project, COLORS, **options)
    assert after.shape == (*project.shape, 3)
    assert np.any(after[project.owner[::-1] == ramp] != before[project.owner[::-1] == ramp])
    assert project.arrows(ramp)[0][2] == pytest.approx(30.0)
