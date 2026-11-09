import math
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'scripts'))
import surface_project as sp  # noqa: E402
import surface_seg as ss  # noqa: E402
from terrain_map import OBSTACLE, SLOPE, TUNNEL, UNDULATION, FLAT, TerrainMap, decode_angle, encode_angle  # noqa: E402


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
    R = sp.rot_z(yaw)
    pts_f = sample_mesh(field(), 120000, np.random.default_rng(1))
    c = sp.Project.from_cloud((pts_f - t) @ R, params(), 'c.pcd', log=lambda *_: None)   # p_f = R p_m + t
    c.register_to(f, [0.45, -0.15, 0.0], math.degrees(yaw) + 2.0, log=lambda *_: None)
    Rr, tr = sp.reg_matrix(c.reg)
    assert np.allclose(tr[:2], t[:2], atol=0.03) and abs(math.atan2(Rr[1, 0], Rr[0, 0]) - yaw) < math.radians(0.5)
    assert c.adopt_from(f, log=lambda *_: None) == 2
    pm = R.T @ (np.array([1.5, 1.0, 0.0]) - t)
    assert pid_at(c, pm[0], pm[1]) == ramp                    # model patch, not a cloud fragment
    assert set(c.ids()) <= set(f.ids())
    a = c.a(pid_at(c, pm[0], pm[1]))
    assert a['type'] == SLOPE and abs(a['dir'] - (90 - math.degrees(yaw))) < 2
    pm = R.T @ (np.array([0.5, 3.5, 0.0]) - t)
    a = c.a(pid_at(c, pm[0], pm[1]))
    assert a['type'] == UNDULATION and abs(a['dir'] - (30 - math.degrees(yaw))) < 0.5   # manual direction, rotated by the registration


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
    out = ss._absorb_clutter(lab, z, 5, 0.06)
    assert out[4, 4] == 1                                       # speckle at floor height joins the floor
    assert out[0, 0] == out[0, 1] != 1                          # high fragments: one clutter patch


def write_stl(path, tri):
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80 + struct.pack('<I', len(tri)))
        for t in tri:
            fh.write(struct.pack('<3f', 0, 0, 0) + t.astype('<f4').tobytes() + b'\0\0')


def test_studio_gui(tmp_path, monkeypatch):
    pytest.importorskip('PyQt5')
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    import surface_studio as st
    st.make_app([])
    s = st.Studio(sync=True)
    s.resize(1300, 850)
    s.set_params(params())
    write_stl(tmp_path / 'f.stl', field())
    s.open_model(str(tmp_path / 'f.stl'))
    p = s.p
    ramp, top = pid_at(p, 1.5, 1.0), pid_at(p, 1.5, 2.0)
    s.click_cell(p.surf.cell_of(1.5, 1.0), Qt_mod(st))
    assert s.sel == [ramp]
    s.set_type(SLOPE)
    assert p.a(ramp)['type'] == SLOPE and 'Slope' not in s.info.text()
    s.click_cell(p.surf.cell_of(1.5, 2.0), Qt_mod(st, ctrl=True))
    assert s.sel == [ramp, top]
    s.merge_selected()
    assert p.info(top) is None and s.sel == [ramp]
    s.undo()
    s.select([top])
    s.set_passable(False)
    assert p.a(top)['passable'] is False
    s.brush_r.setValue(10)
    s.region_mode.setCurrentIndex(1)                            # brush: join the selected patch
    s.tool_brush.setChecked(True)
    c, r = p.surf.cell_of(0.5, 0.5)
    s.press_cell((c, r), Qt_mod(st))
    s.move_cell((c + 4, r))
    s.release_cell((c + 4, r))
    assert pid_at(p, 0.6, 0.5) == top
    floor = pid_at(p, 0.5, 3.5)
    s.region_mode.setCurrentIndex(0)                            # new patch, only inside the selection
    s.select([floor])
    s.tool_rect.setChecked(True)
    a, b = p.surf.cell_of(5.0, 0.2), p.surf.cell_of(5.8, 1.0)
    s.press_cell(a, Qt_mod(st))
    s.move_cell(b)
    s.release_cell(b)
    new = pid_at(p, 5.4, 0.6)
    assert new not in (floor, 0) and s.sel == [new] and pid_at(p, 4.0, 0.6) == floor
    s.select([floor])
    s.tool_lasso.setChecked(True)
    pts = [p.surf.cell_of(*xy) for xy in ((0.2, 2.0), (0.8, 2.0), (0.8, 2.6), (0.2, 2.6))]
    s.press_cell(pts[0], Qt_mod(st))
    for q in pts[1:]:
        s.move_cell(q)
    s.release_cell(pts[0])
    las = pid_at(p, 0.5, 2.3)
    assert las not in (floor, new, 0)
    s.region_mode.setCurrentIndex(2)                            # back to the automatic segmentation
    s.tool_rect.setChecked(True)
    a, b = p.surf.cell_of(0.1, 1.9), p.surf.cell_of(0.9, 2.7)
    s.press_cell(a, Qt_mod(st))
    s.release_cell(b)
    assert pid_at(p, 0.5, 2.3) == floor
    s.tool_select.setChecked(True)
    s.refresh_list()
    assert s.tree.topLevelItemCount() == len(p.ids())
    s.save_project(str(tmp_path / 'p.npz'))
    assert s.export_to(str(tmp_path / 'out'))
    s.show()
    assert not s.grab().isNull()
    s.dirty = False
    s.close()


def Qt_mod(st, ctrl=False):
    return st.Qt.ControlModifier if ctrl else st.Qt.NoModifier
