#!/usr/bin/env python3
"""mesh_to_terrain: field mesh (STL) -> automatic terrain labels + directions in the terrain msgpack.

    # reuse a known world<-map registration
    mesh_to_terrain.py --mesh src/sentry_gazebo/models/rmuc_2027/meshes/field_collision.stl \
        --registration-from maps/sim/nav/terrain.json --out maps/sim/terrain.msgpack
    # or register the prior map cloud onto the mesh (ICP from --guess)
    mesh_to_terrain.py --mesh ... --register maps/sim/map.pcd --out maps/sim/terrain.msgpack

Pipeline (mesh geometry, then HWSentryNav26 style labels):
  1 mesh -> map frame (registration), vertical rays through every cell: up-facing surfaces with the
    clearance to the ceiling above
  2 reachable ground = surfaces with clearance >= robot height + margin, connected to the seeds
    (spawns) with per-cell steps <= max_step; lowest reachable surface per column
  3 labels: OBSTACLE  unreachable, step edge, too steep
            TUNNEL    reachable but clearance < robot height + tunnel_margin
            SLOPE     incline in [slope_min, slope_max]
            FLAT      the rest; small TUNNEL / SLOPE specks are dropped
  4 direction per region (terrain_auto_direction): slope uphill, tunnel passage axis

Manual edits survive regeneration: the msgpack keeps auto_terrain / auto_direction and a manual mask.
Cells painted in terrain_label_editor are marked manual and keep their label; all other cells take the
new automatic result.  'u' in the editor reverts painted cells to the automatic label.
A map without the auto layers (hand-labelled with an older flow) is moved to <out>.legacy.msgpack first.
"""
import argparse
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage, sparse
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
import terrain_auto_direction as autodir  # noqa: E402
from terrain_map import FLAT, LABELS, OBSTACLE, SLOPE, TUNNEL, TerrainMap, read_pcd  # noqa: E402


# ----------------------------------------------------------------------------- io / registration
def load_stl(path):
    data = Path(path).read_bytes()
    n = int(np.frombuffer(data, np.uint32, 1, 80)[0])
    if 84 + 50 * n != len(data):
        raise ValueError('only binary STL is supported: %s' % path)
    rec = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return np.frombuffer(data, rec, n, 84)['v'].astype(np.float64)


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rot_z(yaw) @ ry @ rx


def matrix_rpy(R):
    return (math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


def sample_mesh(tri, n, rng):
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    idx = rng.choice(len(tri), n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    return a[idx] + u[:, None] * (b[idx] - a[idx]) + v[:, None] * (c[idx] - a[idx])


def register(tri, cloud, guess_xyz, guess_yaw, z_max):
    """Point-to-point ICP of the map cloud onto the mesh -> (R, t) map -> world, residual p50/p90, inliers."""
    rng = np.random.default_rng(0)
    surf = sample_mesh(tri, 800000, rng)
    tree = cKDTree(surf[surf[:, 2] < z_max + 0.5])
    R, t = rot_z(guess_yaw), np.asarray(guess_xyz, float)
    for it in range(80):
        q = cloud @ R.T + t
        d, j = tree.query(q)
        m = d < (0.5 if it < 20 else (0.2 if it < 45 else 0.08))
        a, b = q[m], tree.data[j[m]]
        ca, cb = a.mean(0), b.mean(0)
        u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
        dr = vt.T @ np.diag([1, 1, np.sign(np.linalg.det(vt.T @ u.T))]) @ u.T
        R, t = dr @ R, dr @ t + cb - dr @ ca
    d, _ = tree.query(cloud @ R.T + t)
    return R, t, np.percentile(d, [50, 90]), float((d < 0.08).mean())


def registration_from(path):
    """(R, t) map -> world from a terrain.json 'registration' block (t_world_map, rpy_deg)."""
    reg = json.loads(Path(path).read_text())
    reg = reg.get('registration', reg)
    if not reg:
        raise ValueError('%s has no registration' % path)
    return rpy_matrix(*[math.radians(v) for v in reg['rpy_deg']]), np.asarray(reg['t_world_map'], float)


# ----------------------------------------------------------------------------- mesh geometry
def raycast(tri, x0, y0, res, nx, ny, chunk=4_000_000):
    """All vertical-ray hits through cell centres: (cell = ix * ny + iy, z, up-facing), sorted by cell, z."""
    a = tri[:, 0]
    e1, e2 = tri[:, 1] - a, tri[:, 2] - a
    den = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
    lo, hi = tri[:, :, :2].min(1), tri[:, :, :2].max(1)
    ix0 = np.maximum(np.ceil((lo[:, 0] - x0) / res - 0.5), 0).astype(np.int64)
    ix1 = np.minimum(np.floor((hi[:, 0] - x0) / res - 0.5), nx - 1).astype(np.int64)
    iy0 = np.maximum(np.ceil((lo[:, 1] - y0) / res - 0.5), 0).astype(np.int64)
    iy1 = np.minimum(np.floor((hi[:, 1] - y0) / res - 0.5), ny - 1).astype(np.int64)
    cx, cy = ix1 - ix0 + 1, iy1 - iy0 + 1
    ids = np.nonzero((np.abs(den) > 1e-12) & (cx > 0) & (cy > 0))[0]
    if ids.size == 0:
        return np.zeros(0, np.int64), np.zeros(0), np.zeros(0, bool)
    counts = (cx * cy)[ids]
    cum = np.cumsum(counts)
    out_c, out_z, out_u = [], [], []
    start = 0
    while start < len(ids):
        base = cum[start - 1] if start else 0
        stop = max(int(np.searchsorted(cum, base + chunk, side='right')), start + 1)
        sel, cnt = ids[start:stop], counts[start:stop]
        t = np.repeat(sel, cnt)
        off = np.arange(cnt.sum()) - np.repeat(np.cumsum(cnt) - cnt, cnt)
        jx, jy = ix0[t] + off // cy[t], iy0[t] + off % cy[t]
        px, py = x0 + (jx + 0.5) * res - a[t, 0], y0 + (jy + 0.5) * res - a[t, 1]
        d = den[t]
        u = (px * e2[t, 1] - e2[t, 0] * py) / d
        v = (e1[t, 0] * py - px * e1[t, 1]) / d
        m = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
        t, u, v = t[m], u[m], v[m]
        out_c.append(jx[m] * ny + jy[m])
        out_z.append(a[t, 2] + u * e1[t, 2] + v * e2[t, 2])
        out_u.append(den[t] > 0)
        start = stop
    c, z, up = np.concatenate(out_c), np.concatenate(out_z), np.concatenate(out_u)
    order = np.lexsort((z, c))
    return c[order], z[order], up[order]


def surfaces(cells, zs, ups, ncell, layers, eps=0.005):
    """Free up-facing surfaces per column -> Z, CLEAR [layers, ncell] (NaN = none), TOP [ncell]."""
    Z = np.full((layers, ncell), np.nan)
    C = np.full((layers, ncell), np.nan)
    top = np.full(ncell, np.nan)
    if cells.size == 0:
        return Z, C, top
    bounds = np.flatnonzero(np.diff(cells)) + 1
    starts = np.concatenate([[0], bounds])
    ends = np.concatenate([bounds, [len(cells)]])
    top[cells[ends - 1]] = zs[ends - 1]
    for s, e in zip(starts, ends):
        z, up = zs[s:e], ups[s:e]
        found = []
        for i in range(len(z)):
            if not up[i]:
                continue
            j, dup = i + 1, False
            while j < len(z) and z[j] <= z[i] + eps:
                dup |= bool(up[j])
                j += 1
            if dup:
                continue
            if j == len(z):
                found.append((z[i], np.inf))
            elif not up[j]:
                found.append((z[i], z[j] - z[i]))
        for k, (zz, cc) in enumerate(found[-layers:]):
            Z[k, cells[s]] = zz
            C[k, cells[s]] = cc
    return Z, C, top


def lip_relief(Z, C, nx, ny, res, radius=0.10, same=0.02, cap=0.01):
    """Clearance as seen by the chassis on a thin raised lip (seam plate, entry strip).

    The body rides on the wheels spread over the footprint, so a few-mm strip one or two cells
    wide does not lift it by its full height.  Clearance is measured from min(Z, local mean of the
    same surface) instead of Z, relief capped at `cap`; it never lowers clearance.  In the RMUC
    tunnels a 5.6 mm strip left 0.2456 m (< 0.248) and cut b1/b2/c1/c2 in half.
    """
    r = max(1, int(round(radius / res)))
    out = C.copy()
    for k in range(Z.shape[0]):
        z = Z[k].reshape(nx, ny)
        zp = np.pad(z, r, constant_values=np.nan)
        s = np.zeros((nx, ny)); n = np.zeros((nx, ny))
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                zn = zp[r + dx:r + dx + nx, r + dy:r + dy + ny]
                with np.errstate(invalid='ignore'):
                    use = np.isfinite(zn) & (np.abs(zn - z) <= same)
                s += np.where(use, zn, 0.0); n += use
        with np.errstate(invalid='ignore', divide='ignore'):
            relief = np.clip(z - s / n, 0.0, cap)
        out[k] = C[k] + np.nan_to_num(relief).reshape(-1)
    return out


def reachable(Z, C, nx, ny, min_clear, max_step, seeds):
    """Surfaces connected to the seed cells (ix, iy, z) through clearance- and step-feasible neighbours."""
    L = Z.shape[0]
    walk = np.isfinite(Z) & (C >= min_clear)
    Zg, Wg = Z.reshape(L, nx, ny), walk.reshape(L, nx, ny)
    node = np.arange(L * nx * ny).reshape(L, nx, ny)
    rows, cols = [], []
    for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)):
        xs, xd = slice(0, nx - dx), slice(dx, nx)
        ys, yd = (slice(0, ny - dy), slice(dy, ny)) if dy >= 0 else (slice(-dy, ny), slice(0, ny + dy))
        lim = max_step * (math.sqrt(2) if dx and dy else 1)
        for k in range(L):
            for k2 in range(L):
                m = Wg[k][xs, ys] & Wg[k2][xd, yd] & (np.abs(Zg[k][xs, ys] - Zg[k2][xd, yd]) <= lim)
                rows.append(node[k][xs, ys][m])
                cols.append(node[k2][xd, yd][m])
    r, c = np.concatenate(rows), np.concatenate(cols)
    g = sparse.coo_matrix((np.ones(len(r), bool), (r, c)), shape=(L * nx * ny,) * 2)
    _, lab = connected_components(g, directed=False)
    lab = lab.reshape(L, nx * ny)
    comps = set()
    for ix, iy, z in seeds:
        if not (0 <= ix < nx and 0 <= iy < ny):
            print('seed cell (%d, %d) outside the grid, ignored' % (ix, iy))
            continue
        cell = ix * ny + iy
        cand = [(abs(Z[k, cell] - z), k) for k in range(L) if walk[k, cell]]
        if cand:
            comps.add(lab[min(cand)[1], cell])
        else:
            print('seed cell (%d, %d) has no walkable surface, ignored' % (ix, iy))
    if not comps:
        raise ValueError('no usable seed: pass --seed X Y Z on reachable ground')
    return walk & np.isin(lab, list(comps))


def plane_fit(G, mask, res, radius, max_rise=0.05, max_grade=0.5):
    """Windowed same-surface plane fit -> slope (deg), NaN where unsupported."""
    r = max(1, int(round(radius / res)))
    nx, ny = G.shape
    zc = np.where(mask, G, np.nan)
    pad = np.pad(zc, r, constant_values=np.nan)
    S = {k: np.zeros((nx, ny)) for k in ('1', 'x', 'y', 'z', 'xx', 'yy', 'xy', 'xz', 'yz')}
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            d = math.hypot(dx, dy) * res
            if d > radius + 1e-9:
                continue
            zn = pad[r + dx:r + dx + nx, r + dy:r + dy + ny]
            with np.errstate(invalid='ignore'):
                use = np.isfinite(zn) & (np.abs(zn - zc) <= max_rise + max_grade * d)
            dz = np.where(use, zn - np.nan_to_num(zc), 0.0)
            w = use.astype(float)
            X, Y = dx * res, dy * res
            S['1'] += w; S['x'] += w * X; S['y'] += w * Y; S['z'] += dz  # noqa: E702
            S['xx'] += w * X * X; S['yy'] += w * Y * Y; S['xy'] += w * X * Y  # noqa: E702
            S['xz'] += X * dz; S['yz'] += Y * dz  # noqa: E702
    with np.errstate(invalid='ignore', divide='ignore'):
        n1 = S['1']
        mx, my, mz = S['x'] / n1, S['y'] / n1, S['z'] / n1
        cxx, cyy, cxy = S['xx'] / n1 - mx * mx, S['yy'] / n1 - my * my, S['xy'] / n1 - mx * my
        cxz, cyz = S['xz'] / n1 - mx * mz, S['yz'] / n1 - my * mz
        det = cxx * cyy - cxy * cxy
        gx = (cxz * cyy - cyz * cxy) / det
        gy = (cyz * cxx - cxz * cxy) / det
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    bad = ~mask | (n1 < max(6, 0.15 * (2 * r + 1) ** 2)) | ~np.isfinite(slope) | (det < 1e-8)
    slope[bad] = np.nan
    return slope


def max_neighbour_step(G, mask):
    """Largest drop from each cell to a reachable neighbour (upper side of a step only)."""
    step = np.zeros_like(G)
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        s = np.roll(np.roll(G, dx, 0), dy, 1)
        sm = np.roll(np.roll(mask, dx, 0), dy, 1)
        step = np.maximum(step, np.where(mask & sm, G - s, 0.0))
    return step


def drop_small(mask, min_cells):
    lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
    if n == 0:
        return mask
    sizes = ndimage.sum(np.ones_like(lab), lab, np.arange(n + 1))
    return mask & (sizes >= min_cells)[lab]


def keep_inclined(mask, G, x0, y0, res, min_deg):
    """Keep regions whose whole-region plane is inclined >= min_deg (per-cell slope noise on a
    bumpy floor makes patchy regions that are flat as a whole)."""
    lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
    out = np.zeros_like(mask)
    for k in range(1, n + 1):
        ix, iy = np.nonzero(lab == k)
        z = G[ix, iy]
        ok = np.isfinite(z)
        if ok.sum() < 5:
            continue
        _, inc = autodir.fit_uphill(x0 + (ix[ok] + 0.5) * res, y0 + (iy[ok] + 0.5) * res, z[ok])
        if inc >= min_deg:
            out[ix, iy] = True
    return out


# ----------------------------------------------------------------------------- labels
def build_auto(tri_m, x0, y0, res, nx, ny, seeds_map, cfg, log=print):
    """Map-frame triangles -> dict of [ny, nx] grids: terrain, direction, elevation, ground, clearance."""
    cells, zs, ups = raycast(tri_m, x0, y0, res, nx, ny)
    Z, C, top = surfaces(cells, zs, ups, nx * ny, layers=4)
    C = lip_relief(Z, C, nx, ny, res)
    min_clear = cfg.robot_height + cfg.clearance_margin
    seeds = [(int(math.floor((x - x0) / res)), int(math.floor((y - y0) / res)), z) for x, y, z in seeds_map]
    reach = reachable(Z, C, nx, ny, min_clear, cfg.max_step, seeds)
    has = reach.any(0)
    k = np.argmin(np.where(reach, Z, np.inf), 0)
    idx = np.arange(nx * ny)
    G = np.where(has, Z[k, idx], np.nan).reshape(nx, ny)
    CL = np.where(has, np.minimum(C[k, idx], 99.0), np.nan).reshape(nx, ny)
    mask = has.reshape(nx, ny)
    slope = plane_fit(G, mask, res, cfg.fit_radius)
    step = max_neighbour_step(np.nan_to_num(G), mask)
    obstacle = ~mask | (step > cfg.max_step) | ~np.isfinite(slope) | (slope > cfg.slope_max)
    free = ~obstacle
    cell_area = res * res
    tunnel = drop_small(free & (CL < cfg.robot_height + cfg.tunnel_margin),
                        cfg.tunnel_min_area / cell_area)
    sl = free & ~tunnel & (np.nan_to_num(slope) >= cfg.slope_min)
    sl = drop_small(ndimage.binary_opening(sl), cfg.slope_min_area / cell_area)
    sl = keep_inclined(sl, G, x0, y0, res, cfg.slope_min)
    lab = np.full((nx, ny), FLAT, np.uint8)
    lab[obstacle] = OBSTACLE
    lab[tunnel] = TUNNEL
    lab[sl] = SLOPE
    elev = np.where(mask, G, top.reshape(nx, ny))
    auto = TerrainMap(lab.T.copy(), np.zeros((ny, nx), np.uint8), res, (x0, y0),
                      elevation=elev.T.astype(np.float32))
    report = autodir.solve(auto, cfg.snap_deg, cfg.wall_lift)
    log('reachable %.1f m^2, labels: %s' % (mask.sum() * cell_area,
        ', '.join('%s %d' % (LABELS[v], int((lab == v).sum())) for v in (FLAT, OBSTACLE, SLOPE, TUNNEL))))
    for r in report:
        log('  ' + '  '.join('%s=%s' % kv for kv in r.items()))
    return {'terrain': auto.terrain, 'direction': auto.direction, 'elevation': auto.elevation,
            'ground': G.T.astype(np.float32), 'clearance': CL.T.astype(np.float32), 'report': report}


# ----------------------------------------------------------------------------- cli
def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mesh', required=True, help='binary STL of the field (gz world frame)')
    ap.add_argument('--out', required=True, help='terrain msgpack to create / update')
    reg = ap.add_mutually_exclusive_group()
    reg.add_argument('--registration-from', help='terrain.json with registration {t_world_map, rpy_deg}')
    reg.add_argument('--register', help='prior map cloud (map.pcd) to ICP onto the mesh')
    ap.add_argument('--guess', type=float, nargs=4, default=[-9.45, -0.16, 0.50, 0.0],
                    metavar=('X', 'Y', 'Z', 'YAW_DEG'), help='ICP start: map origin in the world')
    ap.add_argument('--seed', type=float, nargs=3, action='append', metavar=('X', 'Y', 'Z'),
                    help='WORLD point on reachable ground (repeatable; default: both RMUC spawns)')
    ap.add_argument('--resolution', type=float, default=0.025, help='new maps only')
    ap.add_argument('--z-max', type=float, default=1.6, help='ignore geometry above (world z)')
    ap.add_argument('--robot-height', type=float, default=0.245)
    ap.add_argument('--clearance-margin', type=float, default=0.003)
    ap.add_argument('--tunnel-margin', type=float, default=0.10,
                    help='clearance below robot height + this is TUNNEL')
    ap.add_argument('--max-step', type=float, default=0.035, help='per-cell height jump, m')
    ap.add_argument('--slope-min', type=float, default=6.0, help='deg, SLOPE from here')
    ap.add_argument('--slope-max', type=float, default=24.0, help='deg, OBSTACLE above')
    ap.add_argument('--fit-radius', type=float, default=0.15)
    ap.add_argument('--slope-min-area', type=float, default=0.15, help='m^2')
    ap.add_argument('--tunnel-min-area', type=float, default=0.10, help='m^2')
    ap.add_argument('--snap-deg', type=float, default=35.0)
    ap.add_argument('--wall-lift', type=float, default=0.15)
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)
    out = Path(a.out)
    tri = load_stl(a.mesh)
    tri = tri[tri[:, :, 2].min(1) < a.z_max]
    print('mesh: %d triangles below z %.2f' % (len(tri), a.z_max))

    R, t, reg = np.eye(3), np.zeros(3), None
    if a.registration_from:
        R, t = registration_from(a.registration_from)
    elif a.register:
        cloud = read_pcd(a.register)
        cloud = cloud[np.isfinite(cloud).all(1)]
        R, t, pct, inl = register(tri, cloud[cloud[:, 2] < a.z_max], a.guess[:3], math.radians(a.guess[3]), a.z_max)
        print('registration residual p50 %.3f p90 %.3f, inliers %.0f %%' % (pct[0], pct[1], 100 * inl))
        if pct[0] > 0.06 or inl < 0.6:
            sys.exit('registration failed; pass --guess')
    reg = {'t_world_map': [float(v) for v in t], 'rpy_deg': [math.degrees(v) for v in matrix_rpy(R)]}
    print('world <- map: t %s rpy %s deg' % (np.round(t, 3).tolist(), np.round(reg['rpy_deg'], 3).tolist()))
    tri_m = ((tri.reshape(-1, 3) - t) @ R).reshape(-1, 3, 3)
    seeds_w = a.seed or [[-9.5, 0.0, 0.30], [9.5, 0.0, 0.30]]
    seeds_m = [tuple((np.asarray(s, float) - t) @ R) for s in seeds_w]

    m = TerrainMap.load(out) if out.is_file() else None
    if m is not None and m.auto_terrain is None:
        legacy = out.with_name(out.stem + '.legacy.msgpack')
        shutil.move(str(out), str(legacy))
        print('%s has no auto layers (older hand-labelled map): moved to %s' % (out, legacy))
        m = None
    if m is None:
        res = a.resolution
        lo, hi = tri_m.reshape(-1, 3).min(0), tri_m.reshape(-1, 3).max(0)
        x0 = math.floor(lo[0] / res) * res - 2 * res
        y0 = math.floor(lo[1] / res) * res - 2 * res
        nx = int(math.ceil((hi[0] - x0) / res)) + 2
        ny = int(math.ceil((hi[1] - y0) / res)) + 2
        m = TerrainMap.blank(nx, ny, res, (x0, y0))
    res, (x0, y0), nx, ny = m.resolution, m.origin, m.width, m.height
    print('grid %d x %d @ %.3f m, origin (%.3f, %.3f)' % (nx, ny, res, x0, y0))

    auto = build_auto(tri_m, x0, y0, res, nx, ny, seeds_m, a)
    m.merge_auto(auto['terrain'], auto['direction'])
    m.elevation = auto['elevation']
    m.extra['ground_z'] = auto['ground'].astype('<f4').tobytes()
    m.extra['clearance'] = auto['clearance'].astype('<f4').tobytes()
    m.extra['source'] = 'mesh_to_terrain %s' % a.mesh
    m.extra['registration'] = reg
    m.save(out)
    print('%s: %d manual cells kept, saved' % (out, int(m.manual.sum())))


if __name__ == '__main__':
    main()
