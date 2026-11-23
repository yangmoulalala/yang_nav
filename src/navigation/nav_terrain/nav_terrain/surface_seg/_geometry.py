"""Measure patch planes, shared borders, slope directions and registration points."""
import numpy as np

from ._grid import EDGES, NEIGHBOURS, height_steps


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
    ux, uy = np.zeros(L), np.zeros(L)
    for direction_sum, (near, far, d, jumps) in zip((ux, uy), height_steps(surf, p.jump)):
        A, B = lab[near], lab[far]
        m = jumps & (A > 0) & (B > 0)
        s, a_, b_ = np.sign(d[m]), A[m], B[m]
        o = a_ != b_               # a jump inside one patch (brushed step band) counts once
        for side, sw in ((a_, s), (b_[o], s[o])):
            direction_sum += np.bincount(side, sw, minlength=L)
    return {
        'n': n, 'area': n * res * res, 'cx': cx, 'cy': cy, 'z': cz, 'zmin': zmin, 'zmax': zmax,
        'incline': np.degrees(np.arctan(np.hypot(ga, gb))), 'azimuth': np.degrees(np.arctan2(gb, ga)) % 360.0,
        'rms': rms, 'axis': (0.5 * np.degrees(np.arctan2(2 * sxy, sxx - syy))) % 180.0, 'clearance': clear,
        'step_dir': np.degrees(np.arctan2(uy, ux)) % 360.0, 'step_w': np.hypot(ux, uy) / nn,
    }


def neighbours(surf, lab, pid):
    """Patches touching `pid` -> [(id, border cells, mean height of theirs minus ours)], longest border first."""
    others, dzs = [], []
    z = surf.z
    for sa, sb in NEIGHBOURS:
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
    for sa, sb in EDGES:
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
    runs = [np.arange(nb)]
    if occ.any() and not occ.all():
        start = int(np.argmin(occ))
        ordered = (np.arange(1, nb + 1) + start) % nb
        occupied = np.flatnonzero(occ[ordered])
        runs = np.split(ordered[occupied], np.flatnonzero(np.diff(occupied) > 1) + 1)
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
    for (near, far, d, jumps), (ox, oy) in zip(height_steps(surf, p.jump), ((1, 0.5), (0.5, 1))):
        r, c = np.nonzero(jumps)
        if not r.size:
            continue
        lo = np.minimum(surf.z[near][r, c], surf.z[far][r, c]).astype(np.float64)
        k = np.ceil(np.abs(d[r, c]) / wall_step).astype(np.int64)
        rep = np.repeat(np.arange(r.size), k)
        step = np.arange(k.sum()) - np.repeat(np.cumsum(k) - k, k)
        pts.append(np.stack([surf.origin[0] + (c[rep] + ox) * res, surf.origin[1] + (r[rep] + oy) * res,
                             lo[rep] + (step + 0.5) * wall_step], 1))
    pts = np.vstack(pts)
    if len(pts) > max_points:
        pts = pts[np.random.default_rng(0).choice(len(pts), max_points, replace=False)]
    return pts
