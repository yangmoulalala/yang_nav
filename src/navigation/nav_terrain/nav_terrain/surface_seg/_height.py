"""Build height and clearance grids in the mesh or point-cloud frame."""
import math

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from ._grid import Surface


def grid_bounds(points, resolution, margin):
    lo = np.floor((points[:, :2].min(0) - margin) / resolution) * resolution
    hi = points[:, :2].max(0) + margin
    width = int(math.ceil((hi[0] - lo[0]) / resolution))
    height = int(math.ceil((hi[1] - lo[1]) / resolution))
    return (float(lo[0]), float(lo[1])), width, height


def fill_small_holes(hm, resolution, fill_radius, hole_area, k=10):
    """NaN holes smaller than hole_area cells -> mean of the observed cells within fill_radius."""
    valid = np.isfinite(hm)
    labels, n = ndimage.label(~valid)
    if n == 0 or not valid.any():
        return hm, 0
    sizes = ndimage.sum(np.ones_like(labels), labels, np.arange(n + 1))
    small = (sizes < hole_area)[labels] & ~valid
    rows, cols = np.nonzero(small)
    if rows.size == 0:
        return hm, 0
    vr, vc = np.nonzero(valid)
    tree = cKDTree(np.column_stack([vc, vr]) * resolution)
    dist, idx = tree.query(np.column_stack([cols, rows]) * resolution, k=min(k, vr.size),
                           distance_upper_bound=fill_radius)
    dist = dist.reshape(rows.size, -1)
    idx = idx.reshape(rows.size, -1)
    good = np.isfinite(dist)
    vz = hm[vr, vc]
    z = np.where(good, vz[np.minimum(idx, vz.size - 1)], 0.0)
    cnt = good.sum(1)
    out = hm.copy()
    fill = cnt > 0
    out[rows[fill], cols[fill]] = (z.sum(1)[fill] / cnt[fill]).astype(np.float32)
    return out, int(fill.sum())


def bilateral(hm, d, sigma_color, sigma_space):
    """Edge-preserving smoothing of the observed cells (NaN stays NaN)."""
    import cv2
    valid = np.isfinite(hm)
    if not valid.any() or d <= 0:
        return hm
    tmp = np.where(valid, hm, np.nanmean(hm)).astype(np.float32)
    filt = cv2.bilateralFilter(tmp, d=d, sigmaColor=sigma_color, sigmaSpace=sigma_space)
    out = hm.copy()
    out[valid] = filt[valid]
    return out


def surfaces(cells, zs, ups, ncell, layers, eps=0.005):
    """Ray hits sorted by (cell, z) -> free up-facing surfaces per column: Z, CLEAR [layers, ncell]
    bottom-up (NaN = none, inf = open sky), TOP [ncell] = highest hit."""
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


def raycast_ids(tri, x0, y0, res, nx, ny, chunk=4_000_000):
    # vertical rays through the cell centres -> hit cell, z, up-facing, triangle index (sorted by cell, z)
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
        return np.zeros(0, np.int64), np.zeros(0), np.zeros(0, bool), np.zeros(0, np.int64)
    counts = (cx * cy)[ids]
    cum = np.cumsum(counts)
    out_c, out_z, out_u, out_t = [], [], [], []
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
        out_t.append(t)
        start = stop
    c, z, up, tt = (np.concatenate(v) for v in (out_c, out_z, out_u, out_t))
    order = np.lexsort((z, c))
    return c[order], z[order], up[order], tt[order]


def mesh_surface(tri, p, log=print):
    """Field triangles (M, 3, 3) -> Surface in the field frame."""
    tri = tri[tri[:, :, 2].min(1) < p.z_max]
    if not len(tri):
        raise ValueError('no triangles below z_max %.2f m' % p.z_max)
    res = p.resolution
    v = tri.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    x0 = math.floor(lo[0] / res) * res - res
    y0 = math.floor(lo[1] / res) * res - res
    nx = int(math.ceil((hi[0] - x0) / res)) + 1
    ny = int(math.ceil((hi[1] - y0) / res)) + 1
    log('高程栅格 %d × %d @ %.3f m，垂直射线求交 %d 个三角形' % (nx, ny, res, len(tri)))
    cells, zs, ups, tid = raycast_ids(tri, x0, y0, res, nx, ny)
    Z, C, top = surfaces(cells, zs, ups, nx * ny, layers=4)
    ok = np.isfinite(Z) & (C >= p.min_clear)
    has = ok.any(0)
    k = np.argmax(ok, 0)                       # layers are stored bottom-up: first free one = lowest
    idx = np.arange(nx * ny)
    zg = np.where(has, Z[k, idx], np.nan)
    z = np.where(has, zg, top)
    cl = np.where(has, C[k, idx], 0.0)
    # vertical view: the exact plane of the up-facing triangle each ground cell was taken from
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    gx, gy = np.full(nx * ny, np.nan), np.full(nx * ny, np.nan)
    with np.errstate(invalid='ignore'):
        m = ups & (np.abs(zs - zg[cells]) < 1e-9)
    cm, tm = cells[m], tid[m]
    _, first = np.unique(cm, return_index=True)
    cm, tm = cm[first], tm[first]
    nz = nrm[tm, 2]
    good = nz > 1e-12
    gx[cm[good]] = -nrm[tm[good], 0] / nz[good]
    gy[cm[good]] = -nrm[tm[good], 1] / nz[good]
    log('垂直视角：%.0f %% 的地面格子使用模型面的精确坡度' % (100.0 * np.isfinite(gx).sum() / max(int(has.sum()), 1)))

    def grid(a):
        return a.reshape(nx, ny).T.astype(np.float32)
    return Surface(grid(z), res, (x0, y0), grid(cl), grid(gx), grid(gy))


def cloud_surface(points, p, log=print):
    """Prior map cloud (N, 3) -> Surface in the map frame.

    Ground per cell = top of the lowest point cluster with at least min_clear empty space above it
    (the floor inside a tunnel, not its roof; the gap is the clearance), backed by a second point
    within 4 cm below (a single stray point under the floor is noise).  Small holes filled, light
    edge-preserving smoothing."""
    q = points[np.isfinite(points).all(1)]
    q = q[(q[:, 2] >= p.cloud_z_min) & (q[:, 2] <= p.cloud_z_max)]
    if not len(q):
        raise ValueError('点云在高度范围 [%.2f, %.2f] m 内没有点' % (p.cloud_z_min, p.cloud_z_max))
    res = max(p.resolution, p.cloud_res)
    origin, w, h = grid_bounds(q, res, 0.3)
    col = np.floor((q[:, 0] - origin[0]) / res).astype(np.int64)
    row = np.floor((q[:, 1] - origin[1]) / res).astype(np.int64)
    ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    key, z = (row * w + col)[ok], q[ok, 2].astype(np.float64)
    o = np.lexsort((z, key))
    key, z = key[o], z[o]
    same = np.r_[key[1:] == key[:-1], False]
    gap = np.where(same, np.r_[z[1:] - z[:-1], 0.0], np.inf)
    kz = key * 1000.0 + (z - z.min())
    support = np.arange(kz.size) - np.searchsorted(kz, kz - 0.04) + 1
    cnt = np.bincount(key, minlength=w * h)
    zg, cl = np.full(w * h, np.nan), np.full(w * h, np.inf)
    last = ~same
    zg[key[last]] = z[last]
    cand = np.nonzero((gap >= p.min_clear) & (support >= np.minimum(2, cnt[key])))[0]
    _, first = np.unique(key[cand], return_index=True)
    idx = cand[first]
    zg[key[idx]] = z[idx]
    cl[key[idx]] = gap[idx]
    hm = zg.reshape(h, w).astype(np.float32)
    hm, _ = fill_small_holes(hm, res, 0.3, int(0.5 / res ** 2))
    hm = bilateral(hm, 3, 0.03, 3.0)
    log('点云高程图 %d × %d @ %.3f m，%d 个点，有数据 %.1f %%，%d 格有顶棚'
        % (w, h, res, len(q), 100.0 * np.isfinite(hm).mean(), int((cl < p.roof_height).sum())))
    return Surface(hm, res, origin, cl.reshape(h, w).astype(np.float32))
