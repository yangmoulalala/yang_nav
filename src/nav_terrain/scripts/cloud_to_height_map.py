#!/usr/bin/env python3
"""cloud_to_height_map: prior point cloud -> top-down height map inside a terrain msgpack (HWSentryNav26 flow).

    cloud_to_height_map.py --map-dir maps/sim --out maps/sim/terrain.msgpack
    cloud_to_height_map.py --pcd maps/sim/map.pcd --out maps/sim/terrain.msgpack

Steps (as HWSentryNav26 utils/py/cloud/cloud_to_height_map.py):
  1 crop to [z_min, z_max], project to the grid, keep the highest point per cell
  2 fill only small holes (connected empty area < hole_area cells) with the mean of the nearest
    observed cells within fill_radius; large unobserved areas stay NaN
  3 edge-preserving bilateral filter over the observed cells
The height map is only a backdrop for terrain_label_editor; the labels drive navigation.

If --out already exists with the same grid, its labels are kept and only the elevation is replaced.
Otherwise a new all-FLAT map is created; --auto-obstacle-step optionally seeds OBSTACLE labels where the
height jumps by more than the given step between neighbouring cells (a starting point for hand labelling).
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
from terrain_map import OBSTACLE, TerrainMap, load_keyframe_cloud, read_pcd  # noqa: E402


def grid_bounds(points, resolution, margin):
    lo = np.floor((points[:, :2].min(0) - margin) / resolution) * resolution
    hi = points[:, :2].max(0) + margin
    width = int(math.ceil((hi[0] - lo[0]) / resolution))
    height = int(math.ceil((hi[1] - lo[1]) / resolution))
    return (float(lo[0]), float(lo[1])), width, height


def max_height_grid(points, origin, resolution, width, height):
    col = np.floor((points[:, 0] - origin[0]) / resolution).astype(np.int64)
    row = np.floor((points[:, 1] - origin[1]) / resolution).astype(np.int64)
    ok = (col >= 0) & (col < width) & (row >= 0) & (row < height)
    flat = np.full(width * height, -np.inf)
    np.maximum.at(flat, row[ok] * width + col[ok], points[ok, 2])
    hm = flat.reshape(height, width)
    hm[~np.isfinite(hm)] = np.nan
    return hm.astype(np.float32)


def fill_small_holes(hm, resolution, fill_radius, hole_area, k=10):
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
    import cv2
    valid = np.isfinite(hm)
    if not valid.any() or d <= 0:
        return hm
    tmp = np.where(valid, hm, np.nanmean(hm)).astype(np.float32)
    filt = cv2.bilateralFilter(tmp, d=d, sigmaColor=sigma_color, sigmaSpace=sigma_space)
    out = hm.copy()
    out[valid] = filt[valid]
    return out


def cloud_to_height_map(points, resolution=0.05, z_range=(-1.0, 1.0), margin=0.5, fill_radius=0.2,
                        hole_area=30, bilateral_d=3, sigma_color=0.15, sigma_space=3.0, bounds=None):
    """-> (height map float32 [H, W] NaN = no data, origin (x, y))."""
    p = points[np.isfinite(points).all(1)]
    p = p[(p[:, 2] >= z_range[0]) & (p[:, 2] <= z_range[1])]
    if p.shape[0] == 0:
        raise ValueError('no points inside z range %s' % (z_range,))
    origin, width, height = bounds if bounds else grid_bounds(p, resolution, margin)
    hm = max_height_grid(p, origin, resolution, width, height)
    hm, _ = fill_small_holes(hm, resolution, fill_radius, hole_area)
    hm = bilateral(hm, bilateral_d, sigma_color, sigma_space)
    return hm, origin


def auto_obstacles(hm, max_step):
    """Cells whose height differs from a 4-neighbour by more than max_step (the higher side of the jump)."""
    z = np.where(np.isfinite(hm), hm, -np.inf)
    obs = np.zeros(hm.shape, bool)
    for axis in (0, 1):
        for shift in (1, -1):
            nb = np.roll(z, shift, axis)
            edge = [slice(None)] * 2
            edge[axis] = 0 if shift == 1 else -1
            nb[tuple(edge)] = z[tuple(edge)]
            with np.errstate(invalid='ignore'):
                obs |= np.isfinite(z) & (z - nb > max_step)
    return obs


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--map-dir', help='backend map bundle (keyframes.txt + clouds/)')
    src.add_argument('--pcd', help='single map-frame point cloud')
    ap.add_argument('--out', required=True, help='terrain msgpack to create / update')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--z-min', type=float, default=-1.0)
    ap.add_argument('--z-max', type=float, default=1.0)
    ap.add_argument('--margin', type=float, default=0.5)
    ap.add_argument('--fill-radius', type=float, default=0.2)
    ap.add_argument('--hole-area', type=int, default=30, help='fill empty components smaller than this (cells)')
    ap.add_argument('--bilateral-d', type=int, default=3)
    ap.add_argument('--sigma-color', type=float, default=0.15)
    ap.add_argument('--sigma-space', type=float, default=3.0)
    ap.add_argument('--auto-obstacle-step', type=float, default=None,
                    help='new map only: label cells above a height jump > this (m) as OBSTACLE')
    a = ap.parse_args(argv)

    pts = load_keyframe_cloud(a.map_dir) if a.map_dir else read_pcd(a.pcd)
    out = Path(a.out)
    old = TerrainMap.load(out) if out.is_file() else None
    bounds = None
    if old is not None:
        if abs(old.resolution - a.resolution) > 1e-9:
            sys.exit('existing %s has resolution %.3f; pass --resolution %.3f or choose a new --out'
                     % (out, old.resolution, old.resolution))
        bounds = (old.origin, old.width, old.height)
    hm, origin = cloud_to_height_map(pts, a.resolution, (a.z_min, a.z_max), a.margin, a.fill_radius,
                                     a.hole_area, a.bilateral_d, a.sigma_color, a.sigma_space, bounds)
    if old is not None:
        old.elevation = hm
        m = old
        print('kept labels of %s, replaced elevation' % out)
    else:
        h, w = hm.shape
        m = TerrainMap.blank(w, h, a.resolution, origin)
        m.elevation = hm
        if a.auto_obstacle_step is not None:
            obs = auto_obstacles(hm, a.auto_obstacle_step)
            m.terrain[obs] = OBSTACLE
            print('auto obstacles: %d cells' % int(obs.sum()))
    m.save(out)
    print('%s: %d points, %dx%d cells @ %.3f m, origin (%.3f, %.3f), observed %.1f %%'
          % (out, pts.shape[0], m.width, m.height, m.resolution, m.origin[0], m.origin[1],
             100.0 * np.isfinite(hm).mean()))


if __name__ == '__main__':
    main()
