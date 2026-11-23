"""Connect co-planar cells and merge small fragments by shared border and height."""
import math

import numpy as np
from scipy import ndimage, sparse
from scipy.sparse.csgraph import connected_components

from ._grid import NEIGHBOURS, cell_gradient, gradient_links


def _compact(lab):
    """Relabel 1..N by decreasing area, 0 stays 0."""
    u, inv, cnt = np.unique(lab.ravel(), return_inverse=True, return_counts=True)
    order = np.argsort(-cnt, kind='stable')
    nz = order[u[order] != 0]
    new = np.zeros(len(u), np.int64)
    new[nz] = np.arange(1, len(nz) + 1)
    return new[inv].reshape(lab.shape)


def _patch_sizes(labels, min_cells):
    area = np.bincount(labels.ravel())
    small = (area > 0) & (area < min_cells)
    small[0] = False
    return area, small


def _merge_small(lab, lx, ly, min_cells, rounds=12):
    for _ in range(rounds):
        area, small = _patch_sizes(lab, min_cells)
        L = len(area)
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
        area, small = _patch_sizes(lab, min_cells)
        L = len(area)
        if not small.any():
            break
        A, B, D = [], [], []
        for sa, sb in NEIGHBOURS:
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
    area, small = _patch_sizes(lab, min_cells)
    L = len(area)
    frag = small[lab]
    if frag.any():
        cl, _ = ndimage.label(frag, structure=np.ones((3, 3)))
        lab = np.where(frag, L + cl, lab)
    return lab


def segment(surf, p):
    """-> int32 [H, W] patch ids 1..N ordered by decreasing area, 0 = no data."""
    labels, _ = segment_with_gradient(surf, p)
    return labels


def segment_with_gradient(surf, p):
    """Compute patch IDs and reuse the same gradients for annotation geometry."""
    links = gradient_links(surf, p)
    gradient = cell_gradient(surf, p, links)
    valid, ((_, lx), (_, ly)) = links
    H, W = valid.shape
    gx, gy = gradient
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
    return _compact(lab).astype(np.int32), gradient
