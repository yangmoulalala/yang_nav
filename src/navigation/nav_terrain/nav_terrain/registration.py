"""Rigid transforms and point-cloud registration in map and field frames."""
import math
from dataclasses import replace

import numpy as np
from scipy.spatial import cKDTree


def quaternion_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(r, p, y):
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rot_z(y) @ ry @ rx


def matrix_rpy(R):
    return (math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


def reg_matrix(reg):
    return rpy_matrix(*[math.radians(v) for v in reg['rpy_deg']]), np.asarray(reg['t_world_map'], float)


class FieldProjection:
    """Intersect map-frame verticals with the registered reference ground, not scanned objects."""

    def __init__(self, source, field, registration):
        self.rotation, self.translation = reg_matrix(registration)
        self.shape = source.z.shape
        rows, cols = np.ogrid[:self.shape[0], :self.shape[1]]
        x, y = source.centre(cols, rows)
        height, width = field.z.shape
        rotation = self.rotation
        if rotation[2, 2] <= 0.1 or not np.isfinite(self.translation).all():
            raise ValueError('配准姿态无法用于俯视地形，请重新配准')
        offsets = [rotation[axis, 0] * x + rotation[axis, 1] * y + self.translation[axis] for axis in range(3)]
        def locate(map_z):
            return [np.floor((offsets[axis] + rotation[axis, 2] * map_z - field.origin[axis]) /
                             field.resolution).astype(np.int64) for axis in (0, 1)]

        finite_heights = field.z[np.isfinite(field.z)]
        if not finite_heights.size:
            raise ValueError('参考工程没有有效高程')
        level = np.median(finite_heights)
        col, row = locate((level - offsets[2]) / rotation[2, 2])
        for _ in range(4):
            ref_z = field.z[row.clip(0, height - 1), col.clip(0, width - 1)]
            map_z = (np.where(np.isfinite(ref_z), ref_z, level) - offsets[2]) / rotation[2, 2]
            next_col, next_row = locate(map_z)
            stable = (col == next_col) & (row == next_row)
            if stable.all():
                break
            col, row = next_col, next_row
        self.inside = stable & (col >= 0) & (col < width) & (row >= 0) & (row < height)
        self.field_indices = row[self.inside], col[self.inside]
        self.map_height = (self.sample(field.z, np.nan) - offsets[2]) / rotation[2, 2]

    def sample(self, grid, fill):
        result = np.full(self.shape, fill)
        result[self.inside] = grid[self.field_indices]
        return result


def extend_to_reference(surface, reference, eligible, registration):
    """Pad on whole map cells without moving existing observations or world landmarks."""
    rows, cols = np.nonzero(eligible)
    if not rows.size:
        return surface, (slice(None), slice(None))
    rotation, translation = reg_matrix(registration)
    x, y = reference.centre(cols, rows)
    centres = (np.column_stack((x, y, reference.z[rows, cols])) - translation) @ rotation[:, :2]
    half_width = reference.resolution * np.abs(rotation[:2, :2]).sum(axis=0) / 2
    origin = np.asarray(surface.origin)
    resolution = surface.resolution
    lower = np.minimum(0, np.floor((centres.min(axis=0) - half_width - origin) / resolution + 1e-8)).astype(int)
    upper = np.maximum(surface.z.shape[::-1], np.ceil((centres.max(axis=0) + half_width - origin) / resolution - 1e-8)).astype(int)
    width, height = map(int, upper - lower)
    if width * height > max(4_000_000, surface.z.size):
        raise ValueError('合并后的地形网格过大，请检查配准平移和栅格分辨率')
    row, col = map(int, -lower[::-1])
    region = np.s_[row:row + surface.z.shape[0], col:col + surface.z.shape[1]]
    def padded(values, fill):
        if values is None:
            return None
        result = np.full((height, width), fill, dtype=values.dtype)
        result[region] = values
        return result
    return replace(surface, z=padded(surface.z, np.nan), clearance=padded(surface.clearance, np.inf),
                   gx=padded(surface.gx, np.nan), gy=padded(surface.gy, np.nan),
                   origin=tuple(origin + lower * resolution)), region


def align_points(source, tree, rotation, translation, iters=60, min_matches=3):
    """Iteratively fit source points to a target KD-tree from a rigid initial guess."""
    R, t = rotation, translation
    for it in range(iters):
        q = source @ R.T + t
        threshold = 0.5 if it < 15 else (0.2 if it < 35 else 0.08)
        d, j = tree.query(q, distance_upper_bound=threshold)
        m = np.isfinite(d)
        if m.sum() < min_matches:
            raise ValueError('ICP lost the overlap (%d matches); give a better start pose' % int(m.sum()))
        a, b = q[m], tree.data[j[m]]
        ca, cb = a.mean(0), b.mean(0)
        u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
        dr = vt.T @ np.diag([1, 1, np.sign(np.linalg.det(vt.T @ u.T))]) @ u.T
        R, t = dr @ R, dr @ t + cb - dr @ ca
    return R, t


def icp(target, source, guess_xyz, guess_yaw, iters=60):
    """Point-to-point ICP, source (map) onto target (field)."""
    tree = cKDTree(target)
    R, t = align_points(source, tree, rot_z(guess_yaw), np.asarray(guess_xyz, float), iters, 100)
    d, _ = tree.query(source @ R.T + t)
    return {'t_world_map': [float(v) for v in t], 'rpy_deg': [math.degrees(v) for v in matrix_rpy(R)],
            'residual_p50': float(np.percentile(d, 50)), 'residual_p90': float(np.percentile(d, 90)),
            'inlier_ratio_8cm': float((d < 0.08).mean())}
