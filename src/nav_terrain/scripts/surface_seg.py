#!/usr/bin/env python3
"""surface_seg: field model / prior cloud -> height grid -> planar and sloped surface patches.

No terrain type is guessed here.  A patch only carries geometry (plane or slope, incline, uphill
azimuth, area, height, roughness, roof clearance, height drop to its neighbours); the operator sets
type, passability and direction in surface_studio.

Grid convention as terrain_map: arrays [row = y, col = x], origin = position of cell (0, 0)'s
lower-left corner, in the field frame (model) or the map frame (prior cloud).

Height grid
  model  vertical rays through every cell; ground = lowest up-facing surface with at least
         min_clear headroom (the floor inside a tunnel, not its roof); clearance = that headroom
  cloud  highest point per cell, small holes filled, bilateral filter (cloud_to_height_map); no
         clearance (a tunnel roof hides the floor below it - less precise than the model)

Segmentation (4-neighbour graph on the height grid)
  1 a link is cut where the height jumps by more than `jump` (step, wall) or where one side is
    covered (clearance < roof_height) and the other is not (tunnel mouth)
  2 per-cell gradient: model = exact plane of the mesh face seen from above (vertical view), cloud =
    differences over the uncut links smoothed over `smooth` m; incline < flat_deg = plane cell
  3 connected components over links joining two plane cells, or two slope cells whose normals differ
    by less than normal_deg
  4 patches smaller than min_area merge into the uncut neighbour they share most border with
  5 one least-squares plane per patch -> incline, uphill azimuth, rms residual
"""
import math
import sys
from dataclasses import asdict, dataclass, fields
from pathlib import Path

import numpy as np
from scipy import ndimage, sparse
from scipy.sparse.csgraph import connected_components

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mesh_to_terrain as mt  # noqa: E402
from cloud_to_height_map import bilateral, fill_small_holes, grid_bounds  # noqa: E402


@dataclass
class SegParams:
    resolution: float = 0.025     # m per cell
    z_max: float = 1.6            # model: ignore triangles starting above this height
    min_clear: float = 0.25       # model: ground = lowest surface with at least this headroom
    roof_height: float = 1.0      # headroom below this = covered (tunnel); patches split at cover changes
    cloud_z_min: float = -1.0     # cloud: crop
    cloud_z_max: float = 1.5
    jump: float = 0.04            # m, neighbour height jump that separates patches
    smooth: float = 0.075         # m, gradient smoothing radius
    flat_deg: float = 4.0         # incline below this = plane
    normal_deg: float = 6.0       # neighbouring slope cells join when their normals differ less
    min_area: float = 0.04        # m^2, smaller patches merge into a neighbour
    cloud_res: float = 0.05       # cloud: coarsest of this and `resolution` (a prior map has ~10 points / 5 cm cell)
    cloud_smooth: float = 0.2     # cloud: gradient smoothing radius (scan noise)
    clutter_dz: float = 0.06      # fragments still below min_area join a touching patch within this height,
                                  # else touching fragments form one clutter patch

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d):
        names = {f.name for f in fields(cls)}
        return cls(**{k: float(v) for k, v in (d or {}).items() if k in names})


@dataclass
class Surface:
    z: np.ndarray                 # float32 [H, W], NaN = no data
    resolution: float
    origin: tuple
    clearance: np.ndarray         # float32 [H, W], inf = open sky
    gx: np.ndarray = None         # model only: exact dz/dx, dz/dy of the face seen from above, NaN = unknown
    gy: np.ndarray = None

    def cell_of(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def centre(self, col, row):
        return (self.origin[0] + (col + 0.5) * self.resolution, self.origin[1] + (row + 0.5) * self.resolution)


# ----------------------------------------------------------------------------- height grids
def raycast_ids(tri, x0, y0, res, nx, ny, chunk=4_000_000):
    # as mesh_to_terrain.raycast, plus the triangle index of every hit
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
    Z, C, top = mt.surfaces(cells, zs, ups, nx * ny, layers=4)
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


# ----------------------------------------------------------------------------- segmentation
def _links(surf, p):
    z = surf.z
    valid = np.isfinite(z)
    cl = np.nan_to_num(surf.clearance.astype(np.float64), nan=np.inf, posinf=np.inf)
    cov = cl < p.roof_height
    zz = np.where(valid, z, 0.0).astype(np.float64)
    dx, dy = zz[:, 1:] - zz[:, :-1], zz[1:] - zz[:-1]
    both_x, both_y = valid[:, 1:] & valid[:, :-1], valid[1:] & valid[:-1]
    lx = both_x & (np.abs(dx) <= p.jump) & (cov[:, 1:] == cov[:, :-1])
    ly = both_y & (np.abs(dy) <= p.jump) & (cov[1:] == cov[:-1])
    return valid, zz, dx, dy, lx, ly, both_x, both_y


def _mean_diff(d, link, axis, shape):
    s, c = np.zeros(shape), np.zeros(shape)
    v = np.where(link, d, 0.0)
    if axis == 1:
        s[:, 1:] += v; s[:, :-1] += v; c[:, 1:] += link; c[:, :-1] += link  # noqa: E702
    else:
        s[1:] += v; s[:-1] += v; c[1:] += link; c[:-1] += link  # noqa: E702
    return s / np.maximum(c, 1), c > 0


def cell_gradient(surf, p, links=None):
    """Smoothed per-cell gradient (dz/dx, dz/dy) from the uncut links."""
    valid, zz, dx, dy, lx, ly, _, _ = links or _links(surf, p)
    res = surf.resolution
    gx, hx = _mean_diff(dx, lx, 1, zz.shape)
    gy, hy = _mean_diff(dy, ly, 0, zz.shape)
    gx, gy = gx / res, gy / res
    smooth = p.smooth if surf.gx is not None else max(p.smooth, p.cloud_smooth)
    r = max(1, int(round(smooth / res)))
    w = (valid & hx & hy).astype(np.float64)
    den = ndimage.uniform_filter(w, 2 * r + 1, mode='constant')
    ok = den > 1e-6
    sden = np.maximum(den, 1e-6)
    gxs = np.where(ok, ndimage.uniform_filter(gx * w, 2 * r + 1, mode='constant') / sden, gx)
    gys = np.where(ok, ndimage.uniform_filter(gy * w, 2 * r + 1, mode='constant') / sden, gy)
    if surf.gx is not None:                    # model: exact face slope wherever known, no smoothing
        ex = np.isfinite(surf.gx) & valid
        gxs, gys = np.where(ex, surf.gx, gxs), np.where(ex, surf.gy, gys)
    return gxs, gys


def _compact(lab):
    """Relabel 1..N by decreasing area, 0 stays 0."""
    u, inv, cnt = np.unique(lab.ravel(), return_inverse=True, return_counts=True)
    order = np.argsort(-cnt, kind='stable')
    nz = order[u[order] != 0]
    new = np.zeros(len(u), np.int64)
    new[nz] = np.arange(1, len(nz) + 1)
    return new[inv].reshape(lab.shape)


def _merge_small(lab, lx, ly, min_cells, rounds=12):
    for _ in range(rounds):
        L = int(lab.max()) + 1
        area = np.bincount(lab.ravel(), minlength=L)
        small = (area < min_cells) & (area > 0)
        small[0] = False
        if not small.any():
            break
        A = np.concatenate([lab[:, :-1][lx], lab[:-1][ly]])
        B = np.concatenate([lab[:, 1:][lx], lab[1:][ly]])
        a, b = np.concatenate([A, B]), np.concatenate([B, A])
        # merge only into a larger patch (ties: lower id) -> no cycles
        m = (a != b) & small[a] & ((area[b] > area[a]) | ((area[b] == area[a]) & (b < a)))
        if not m.any():
            break
        a, b = a[m], b[m]
        u, cnt = np.unique(a * L + b, return_counts=True)
        ua, ub = u // L, u % L
        order = np.lexsort((-area[ub], -cnt, ua))
        ua, ub = ua[order], ub[order]
        first = np.r_[True, ua[1:] != ua[:-1]]
        target = np.arange(L)
        target[ua[first]] = ub[first]
        while True:
            nt = target[target]
            if (nt == target).all():
                break
            target = nt
        lab = target[lab]
    return lab


def _absorb_clutter(lab, z, min_cells, dz_tol, rounds=6):
    # Fragments still below min_cells (scan noise at wall feet, speckles): join the touching big patch
    # with the closest height (mean |dz| over the border <= dz_tol); the rest: touching fragments
    # (8-connected) become one clutter patch each.
    zz = np.nan_to_num(np.asarray(z, np.float64))
    for _ in range(rounds):
        L = int(lab.max()) + 1
        area = np.bincount(lab.ravel(), minlength=L)
        small = (area < min_cells) & (area > 0)
        small[0] = False
        if not small.any():
            break
        A, B, D = [], [], []
        for sa, sb in _SHIFTS:
            a, b = lab[sa], lab[sb]
            m = (a != b) & (a > 0) & (b > 0)
            m &= small[a] & ~small[b]
            A.append(a[m]); B.append(b[m]); D.append(np.abs(zz[sb][m] - zz[sa][m]))  # noqa: E702
        a, b, d = (np.concatenate(v) for v in (A, B, D))
        if not a.size:
            break
        u, inv, cnt = np.unique(a.astype(np.int64) * L + b, return_inverse=True, return_counts=True)
        md = np.bincount(inv, d) / cnt
        ok = md <= dz_tol
        if not ok.any():
            break
        ua, ub, md = u[ok] // L, u[ok] % L, md[ok]
        order = np.lexsort((md, ua))
        ua, ub = ua[order], ub[order]
        first = np.r_[True, ua[1:] != ua[:-1]]
        target = np.arange(L)
        target[ua[first]] = ub[first]
        lab = target[lab]
    L = int(lab.max()) + 1
    area = np.bincount(lab.ravel(), minlength=L)
    small = (area < min_cells) & (area > 0)
    small[0] = False
    frag = small[lab]
    if frag.any():
        cl, _ = ndimage.label(frag, structure=np.ones((3, 3)))
        lab = np.where(frag, L + cl, lab)
    return lab


def segment(surf, p):
    """-> int32 [H, W] patch ids 1..N ordered by decreasing area, 0 = no data."""
    links = _links(surf, p)
    valid, _, _, _, lx, ly, _, _ = links
    H, W = valid.shape
    gx, gy = cell_gradient(surf, p, links)
    slope = np.degrees(np.arctan(np.hypot(gx, gy))) >= p.flat_deg
    n = np.stack([-gx, -gy, np.ones_like(gx)], -1)
    n /= np.linalg.norm(n, axis=-1, keepdims=True)
    cmin = math.cos(math.radians(p.normal_deg))
    jx = lx & (slope[:, 1:] == slope[:, :-1]) & (~slope[:, 1:] | ((n[:, 1:] * n[:, :-1]).sum(-1) >= cmin))
    jy = ly & (slope[1:] == slope[:-1]) & (~slope[1:] | ((n[1:] * n[:-1]).sum(-1) >= cmin))
    idx = np.arange(H * W).reshape(H, W)
    a = np.concatenate([idx[:, :-1][jx], idx[:-1][jy]])
    b = np.concatenate([idx[:, 1:][jx], idx[1:][jy]])
    g = sparse.coo_matrix((np.ones(a.size, np.int8), (a, b)), shape=(H * W, H * W))
    _, comp = connected_components(g, directed=False)
    lab = comp.reshape(H, W).astype(np.int64) + 1
    lab[~valid] = 0
    min_cells = p.min_area / surf.resolution ** 2
    lab = _merge_small(_compact(lab), lx, ly, min_cells)
    lab = _absorb_clutter(_compact(lab), surf.z, min_cells, p.clutter_dz)
    return _compact(lab).astype(np.int32)


# ----------------------------------------------------------------------------- patch geometry
def patch_stats(surf, lab, p):
    """Per-patch geometry, arrays indexed by patch id (entry 0 unused).

    n, area, cx, cy, z (mean), zmin, zmax, incline (deg), azimuth (deg, uphill, 0 = +x, ccw),
    rms (m, plane-fit residual), axis (deg, long axis of the footprint, 0..180),
    clearance (m, minimum headroom), step_dir (deg, mean direction lower -> higher across the
    jumps on its border), step_w (strength of step_dir, 0 = no jump border)
    """
    res = surf.resolution
    L = int(lab.max()) + 1
    rr, cc = np.nonzero(lab > 0)
    l = lab[rr, cc]
    z = surf.z[rr, cc].astype(np.float64)
    x = surf.origin[0] + (cc + 0.5) * res
    y = surf.origin[1] + (rr + 0.5) * res

    def bc(w):
        return np.bincount(l, w, minlength=L)

    n = np.bincount(l, minlength=L).astype(np.float64)
    nn = np.maximum(n, 1)
    cx, cy, cz = bc(x) / nn, bc(y) / nn, bc(z) / nn
    X, Y, Z = x - cx[l], y - cy[l], z - cz[l]
    sxx, syy, sxy, sxz, syz = bc(X * X), bc(Y * Y), bc(X * Y), bc(X * Z), bc(Y * Z)
    det = sxx * syy - sxy * sxy
    good = det > 1e-12
    sdet = np.where(good, det, 1.0)
    ga = np.where(good, (sxz * syy - syz * sxy) / sdet, 0.0)
    gb = np.where(good, (syz * sxx - sxz * sxy) / sdet, 0.0)
    rms = np.sqrt(bc((Z - ga[l] * X - gb[l] * Y) ** 2) / nn)
    zmin, zmax = np.full(L, np.inf), np.full(L, -np.inf)
    np.minimum.at(zmin, l, z)
    np.maximum.at(zmax, l, z)
    clear = np.full(L, np.inf)
    np.minimum.at(clear, l, np.nan_to_num(surf.clearance[rr, cc].astype(np.float64), nan=np.inf, posinf=np.inf))
    # direction lower -> higher across the jumps on the patch border, accumulated on both sides
    valid, zz, dx, dy, _, _, both_x, both_y = _links(surf, p)
    ux, uy = np.zeros(L), np.zeros(L)
    for A, B, d, ex, ey in ((lab[:, :-1], lab[:, 1:], dx, 1.0, 0.0), (lab[:-1], lab[1:], dy, 0.0, 1.0)):
        m = (both_x if ex else both_y) & (np.abs(d) > p.jump) & (A > 0) & (B > 0)
        s, a_, b_ = np.sign(d[m]), A[m], B[m]
        o = a_ != b_               # a jump inside one patch (brushed step band) counts once
        for side, sw in ((a_, s), (b_[o], s[o])):
            ux += np.bincount(side, ex * sw, minlength=L)
            uy += np.bincount(side, ey * sw, minlength=L)
    return {
        'n': n, 'area': n * res * res, 'cx': cx, 'cy': cy, 'z': cz, 'zmin': zmin, 'zmax': zmax,
        'incline': np.degrees(np.arctan(np.hypot(ga, gb))), 'azimuth': np.degrees(np.arctan2(gb, ga)) % 360.0,
        'rms': rms, 'axis': (0.5 * np.degrees(np.arctan2(2 * sxy, sxx - syy))) % 180.0, 'clearance': clear,
        'step_dir': np.degrees(np.arctan2(uy, ux)) % 360.0, 'step_w': np.hypot(ux, uy) / nn,
    }


_SHIFTS = ((np.s_[:, :-1], np.s_[:, 1:]), (np.s_[:, 1:], np.s_[:, :-1]),
           (np.s_[:-1], np.s_[1:]), (np.s_[1:], np.s_[:-1]))


def neighbours(surf, lab, pid):
    """Patches touching `pid` -> [(id, border cells, mean height of theirs minus ours)], longest border first."""
    others, dzs = [], []
    z = surf.z
    for sa, sb in _SHIFTS:
        A, B = lab[sa], lab[sb]
        m = (A == pid) & (B != pid) & (B > 0)
        others.append(B[m])
        dzs.append((z[sb][m] - z[sa][m]).astype(np.float64))
    o, d = np.concatenate(others), np.concatenate(dzs)
    if not o.size:
        return []
    u, inv, cnt = np.unique(o, return_inverse=True, return_counts=True)
    mean = np.bincount(inv, d) / cnt
    order = np.argsort(-cnt)
    return [(int(u[i]), int(cnt[i]), float(mean[i])) for i in order]


def adjacency(surf, lab, tol):
    """Touching patch pairs (a < b) -> a, b, border cells, border cells with a height step <= tol."""
    A, B, D = [], [], []
    z = surf.z.astype(np.float64)
    for sa, sb in (_SHIFTS[0], _SHIFTS[2]):
        a, b = lab[sa], lab[sb]
        m = (a != b) & (a > 0) & (b > 0)
        A.append(a[m]); B.append(b[m]); D.append(z[sb][m] - z[sa][m])  # noqa: E702
    a, b, d = (np.concatenate(v) for v in (A, B, D))
    lo, hi = np.minimum(a, b).astype(np.int64), np.maximum(a, b).astype(np.int64)
    with np.errstate(invalid='ignore'):
        cont = (np.abs(d) <= tol).astype(np.float64)
    L = int(lab.max()) + 1
    u, inv, cnt = np.unique(lo * L + hi, return_inverse=True, return_counts=True)
    return u // L, u % L, cnt, np.bincount(inv, cont, minlength=len(u)).astype(np.int64)


def slope_groups(gx, gy, mask, res, origin, flat_deg, bin_deg=10.0, min_share=0.08):
    # Uphill directions inside one patch seen from above (a fortress hexagon ring or a ridge has several).
    # -> (groups [{'az', 'cx', 'cy', 'area'}] largest first, (rows, cols, group index per cell, -1 = flat))
    r, c = np.nonzero(mask)
    g1, g2 = gx[r, c].astype(np.float64), gy[r, c].astype(np.float64)
    inc = np.degrees(np.arctan(np.hypot(g1, g2)))
    s = np.isfinite(inc) & (inc >= flat_deg)
    asg = np.full(r.size, -1, np.int64)
    if not s.any():
        return [], (r, c, asg)
    az = np.degrees(np.arctan2(g2, g1)) % 360.0
    nb = int(round(360.0 / bin_deg))
    sidx = np.nonzero(s)[0]
    b = (az[sidx] // bin_deg).astype(np.int64) % nb
    hist = np.bincount(b, minlength=nb)
    occ = hist > max(1, 0.01 * sidx.size)
    if occ.all() or not occ.any():
        runs = [list(range(nb))]
    else:
        start = int(np.argmin(occ))            # an empty bin: contiguous occupied bins form one direction
        runs, cur = [], []
        for k in range(1, nb + 1):
            j = (start + k) % nb
            if occ[j]:
                cur.append(j)
            elif cur:
                runs.append(cur)
                cur = []
        if cur:
            runs.append(cur)
    groups = []
    for run in runs:
        sel = sidx[np.isin(b, run)]
        if sel.size < min_share * sidx.size:
            continue
        a = np.radians(az[sel])
        groups.append({'az': float(np.degrees(np.arctan2(np.sin(a).mean(), np.cos(a).mean())) % 360.0),
                       'cx': float(origin[0] + (c[sel].mean() + 0.5) * res),
                       'cy': float(origin[1] + (r[sel].mean() + 0.5) * res), 'area': float(sel.size * res * res),
                       'incline': float(inc[sel].mean())})
    groups.sort(key=lambda g: -g['area'])
    if groups:
        ga = np.radians([g['az'] for g in groups])
        d = np.abs((np.radians(az[sidx])[:, None] - ga[None, :] + np.pi) % (2 * np.pi) - np.pi)
        asg[sidx] = np.argmin(d, 1)
    return groups, (r, c, asg)


def surface_points(surf, p, wall_step=0.05, max_points=1_500_000):
    """Grid surface as points, plus vertical wall points along the jumps (ICP target without the mesh)."""
    res = surf.resolution
    rr, cc = np.nonzero(np.isfinite(surf.z))
    pts = [np.stack([surf.origin[0] + (cc + 0.5) * res, surf.origin[1] + (rr + 0.5) * res,
                     surf.z[rr, cc]], 1)]
    valid, zz, dx, dy, _, _, both_x, both_y = _links(surf, p)
    for d, both, ox, oy in ((dx, both_x, 1.0, 0.5), (dy, both_y, 0.5, 1.0)):
        r, c = np.nonzero(both & (np.abs(d) > p.jump))
        if not r.size:
            continue
        lo = np.minimum(zz[r, c], zz[r + (oy == 1.0), c + (ox == 1.0)])
        k = np.ceil(np.abs(d[r, c]) / wall_step).astype(np.int64)
        rep = np.repeat(np.arange(r.size), k)
        step = np.arange(k.sum()) - np.repeat(np.cumsum(k) - k, k)
        pts.append(np.stack([surf.origin[0] + (c[rep] + ox) * res, surf.origin[1] + (r[rep] + oy) * res,
                             lo[rep] + (step + 0.5) * wall_step], 1))
    pts = np.vstack(pts)
    if len(pts) > max_points:
        pts = pts[np.random.default_rng(0).choice(len(pts), max_points, replace=False)]
    return pts
