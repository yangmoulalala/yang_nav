"""Rigid transforms and point-cloud registration in map and field frames."""
import math

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
    """Map-frame cell centres projected into a registered field grid at map z=0."""

    def __init__(self, source, field, registration):
        self.rotation, self.translation = reg_matrix(registration)
        self.shape = source.z.shape
        rows, cols = np.ogrid[:self.shape[0], :self.shape[1]]
        x, y = source.centre(cols, rows)
        def indices(axis):
            coord = self.rotation[axis, 0] * x + self.rotation[axis, 1] * y + self.translation[axis]
            return np.floor((coord - field.origin[axis]) / field.resolution).astype(np.int64)

        col, row = indices(0), indices(1)
        height, width = field.z.shape
        self.inside = (col >= 0) & (col < width) & (row >= 0) & (row < height)
        self.field_indices = row[self.inside], col[self.inside]

    def sample(self, grid, fill):
        result = np.full(self.shape, fill)
        result[self.inside] = grid[self.field_indices]
        return result


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
