#!/usr/bin/env python3
r"""Official field CAD (STEP tessellation or STL) -> point cloud in the localization map frame.

Input mesh is in the field/world frame. With --registration-from (terrain.json, map->world), the
cloud is moved into the SLAM map frame for comparison with maps/<m>/map.pcd.
This writes a PCD file, not a backend map bundle with keyframes and descriptors.

    python3 tools/cad_map/cad_to_cloud.py --mesh /absolute/path/field_mesh.npz \
        --registration-from maps/sim/terrain/terrain.json --voxel 0.05 --out /tmp/cad/cad_map.pcd \
        --compare maps/sim/map.pcd
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from nav_terrain.mesh_io import load_mesh
from nav_terrain.point_cloud import read_pcd
from nav_terrain.registration import align_points, matrix_rpy, reg_matrix


def registration_from(path):
    reg = json.loads(Path(path).read_text())
    reg = reg.get('registration', reg)
    return reg_matrix(reg)


def sample_surface(tri, spacing, rng):
    """Area-uniform samples, ~1/spacing^2 points per m^2."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    n = int(area.sum() / spacing ** 2)
    idx = rng.choice(len(tri), n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    f = u + v > 1
    u[f], v[f] = 1 - u[f], 1 - v[f]
    return a[idx] + u[:, None] * (b[idx] - a[idx]) + v[:, None] * (c[idx] - a[idx])


def voxel_down(p, voxel):
    k = np.floor(p / voxel).astype(np.int64)
    _, inv = np.unique(k, axis=0, return_inverse=True)
    inv = inv.ravel()
    cnt = np.bincount(inv)
    return np.stack([np.bincount(inv, p[:, i]) / cnt for i in range(3)], 1)


def write_pcd(path, p):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = len(p)
    hdr = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
           'COUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (n, n))
    arr = np.zeros((n, 4), np.float32)
    arr[:, :3] = p
    path.write_bytes(hdr.encode() + arr.tobytes())


def stats(d):
    return 'p50 %.3f p90 %.3f inl<8cm %.1f%% inl<15cm %.1f%%' % (
        np.percentile(d, 50), np.percentile(d, 90), 100 * (d < .08).mean(), 100 * (d < .15).mean())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mesh', required=True, help='binary STL or mesh.npz (V, F) in field frame, metres')
    ap.add_argument('--registration-from', help='terrain.json with map->world registration')
    ap.add_argument('--spacing', type=float, default=0.02, help='surface sampling spacing, m')
    ap.add_argument('--voxel', type=float, default=0.05)
    ap.add_argument('--z-min', type=float, default=-0.5, help='crop in field frame')
    ap.add_argument('--z-max', type=float, default=4.0)
    ap.add_argument('--out', required=True)
    ap.add_argument('--compare', help='SLAM map.pcd (map frame) to check against')
    ap.add_argument('--refine-out', help='with --compare: write the CAD cloud after ICP onto the SLAM map')
    a = ap.parse_args()

    tri = load_mesh(a.mesh)
    zc = tri[:, :, 2].mean(1)
    tri = tri[(zc > a.z_min) & (zc < a.z_max)]
    pts = voxel_down(sample_surface(tri, a.spacing, np.random.default_rng(0)), a.voxel)
    print('mesh %d tris -> %d points (voxel %.2f)' % (len(tri), len(pts), a.voxel))
    if a.registration_from:
        R, t = registration_from(a.registration_from)  # map -> world
        pts = (pts - t) @ R                             # world -> map
    write_pcd(a.out, pts)
    print('wrote', a.out, 'bbox', pts.min(0).round(2), pts.max(0).round(2))

    if a.compare:
        slam = read_pcd(a.compare)
        tree = cKDTree(pts)
        d, _ = tree.query(slam)
        print('SLAM -> CAD with given registration :', stats(d))
        R2, t2 = align_points(slam, tree, np.eye(3), np.zeros(3))
        d2, _ = tree.query(slam @ R2.T + t2)
        rpy = np.degrees(matrix_rpy(R2))
        print('SLAM -> CAD after ICP refine         :', stats(d2), '| dt', t2.round(3), 'drpy', rpy.round(3))
        dr, _ = cKDTree(slam).query(pts)
        print('CAD  -> SLAM (CAD coverage by SLAM)  :', stats(dr))
        if a.refine_out:
            write_pcd(a.refine_out, (pts - t2) @ R2)  # CAD moved into the SLAM map frame
            print('wrote refined', a.refine_out)


if __name__ == '__main__':
    main()
