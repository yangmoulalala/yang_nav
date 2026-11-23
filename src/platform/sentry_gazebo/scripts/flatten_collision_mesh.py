#!/usr/bin/env python3
"""Flatten millimetre steps in a triangle-mesh field collision (binary STL).

The RMUC 2027 STEP models floor markings (white lines, zone marks) and plate
seams as solid slabs a few millimetres thick.  Converted verbatim into
``models/rmuc_2027/meshes/field_collision.stl`` they leave hundreds of
thousands of tiny vertical edges that the simulated omni wheels snag on,
although a real 65 mm wheel rolls over them unnoticed.

Method (vertex snapping, robust to unwelded vertices and T-junctions):

1. Every up-facing horizontal triangle votes its area for its height level
   (quantised to ``--quant``) in an x-y grid cell of ``--cell`` metres.
2. Each vertex below ``--zmax`` that does not touch a sloped face (ramps keep
   their exact geometry) looks at the levels within ``--max-step`` of its
   height in the 3x3 neighbouring cells and snaps to the level with the
   largest area, if that level outweighs its own.  So a 3 mm decal drops onto
   the floor around it, a 2 mm groove rises to it, and real steps taller than
   ``--max-step`` are untouched.
3. Step walls that collapsed to zero height, and degenerate triangles, are
   removed.

The visual mesh is untouched, so the markings are still drawn.

    python3 scripts/flatten_collision_mesh.py \
        models/rmuc_2027/meshes/field_collision.stl /tmp/field_collision_flat.stl

The original mesh is in git history (commit e14f451).
"""
import argparse
import collections
import sys

import numpy as np

STL_DTYPE = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])


def read_stl(path):
    d = open(path, 'rb').read()
    if d[:5] == b'solid' and b'facet' in d[:400]:
        raise SystemExit('ASCII STL not supported: %s' % path)
    nt = int.from_bytes(d[80:84], 'little')
    return np.frombuffer(d, STL_DTYPE, count=nt, offset=84)['v'].reshape(-1, 3, 3).astype(np.float64)


def write_stl(path, T):
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-30
    rec = np.zeros(len(T), STL_DTYPE)
    rec['n'] = n
    rec['v'] = T.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(b'flatten_collision_mesh.py'.ljust(80, b' '))
        f.write(np.uint32(len(T)).tobytes())
        f.write(rec.tobytes())


def weld(T, tol):
    key = np.round(T.reshape(-1, 3) / tol).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    V = np.zeros((len(uniq), 3))
    np.add.at(V, inv.ravel(), T.reshape(-1, 3))
    V /= np.bincount(inv.ravel())[:, None]
    return V, inv.reshape(-1, 3)


def tri_props(V, F):
    P = V[F]
    n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    a2 = np.linalg.norm(n, axis=1)
    return n[:, 2] / (a2 + 1e-30), a2 / 2.0, P[:, :, 2].min(1), P[:, :, 2].max(1)


def small_walls(V, F, a, zmax=None):
    nz, _, lo, hi = tri_props(V, F)
    m = (np.abs(nz) < 0.5) & (hi - lo > a.ztol) & (hi - lo <= a.max_step)
    return m & (lo < zmax) if zmax is not None else m


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('input')
    ap.add_argument('output')
    ap.add_argument('--max-step', type=float, default=0.0065, help='largest step flattened (m)')
    ap.add_argument('--zmax', type=float, default=1.2, help='only vertices below this height (m)')
    ap.add_argument('--cell', type=float, default=0.25, help='level-voting grid cell (m)')
    ap.add_argument('--quant', type=float, default=5e-4, help='level quantisation (m)')
    ap.add_argument('--weld', type=float, default=1e-5, help='vertex weld tolerance (m)')
    ap.add_argument('--ztol', type=float, default=2e-4, help='flatness tolerance (m)')
    ap.add_argument('--min-area', type=float, default=1e-9, help='drop triangles below this area (m^2)')
    a = ap.parse_args(argv)

    V, F = weld(read_stl(a.input), a.weld)
    nz, area, lo, hi = tri_props(V, F)
    print('input : %d triangles, %d vertices, %d small step walls below %.2f m'
          % (len(F), len(V), int(small_walls(V, F, a, a.zmax).sum()), a.zmax))

    # 1. area-weighted height levels per grid cell
    up = (nz > 0.99) & (hi - lo < a.ztol) & (hi < a.zmax + a.max_step)
    cen = V[F[up]].mean(1)
    cells = collections.defaultdict(collections.Counter)
    keys = np.floor(cen[:, :2] / a.cell).astype(int)
    zq = np.round(cen[:, 2] / a.quant).astype(int)
    for (i, j), q, ar in zip(keys.tolist(), zq.tolist(), area[up].tolist()):
        cells[(i, j)][q] += ar
    merged = {}

    def levels(i, j):
        if (i, j) not in merged:
            c = collections.Counter()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    c.update(cells.get((i + di, j + dj), {}))
            merged[(i, j)] = c
        return merged[(i, j)]

    # 2. snap vertices (not on ramps / sloped faces)
    sloped = np.zeros(len(V), bool)
    sloped[F[(np.abs(nz) >= 0.5) & (np.abs(nz) <= 0.99)].ravel()] = True
    cand = np.nonzero(~sloped & (V[:, 2] < a.zmax))[0]
    win = int(round(a.max_step / a.quant))
    ck = np.floor(V[cand, :2] / a.cell).astype(int).tolist()
    vq = np.round(V[cand, 2] / a.quant).astype(int).tolist()
    moved = 0
    newz = V[:, 2].copy()
    for v, (i, j), q in zip(cand.tolist(), ck, vq):
        L = levels(i, j)
        if not L:
            continue
        own = L.get(q, 0.0) + L.get(q - 1, 0.0) + L.get(q + 1, 0.0)
        best, bw = None, own
        for k in range(q - win, q + win + 1):
            w = L.get(k, 0.0)
            if abs(k - q) > 1 and w > bw:
                best, bw = k, w
        if best is not None:
            newz[v] = best * a.quant
            moved += 1
    V[:, 2] = newz
    print('snapped %d of %d candidate vertices' % (moved, len(cand)))

    # 3. drop collapsed walls and degenerate triangles
    nz, area, lo, hi = tri_props(V, F)
    dup = (F[:, 0] == F[:, 1]) | (F[:, 1] == F[:, 2]) | (F[:, 0] == F[:, 2])
    collapsed = (np.abs(nz) < 0.5) & (hi - lo < a.ztol)
    F = F[~dup & ~collapsed & (area >= a.min_area)]
    print('output: %d triangles, %d small step walls below %.2f m'
          % (len(F), int(small_walls(V, F, a, a.zmax).sum()), a.zmax))
    write_stl(a.output, V[F].astype(np.float32))
    print('wrote', a.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
