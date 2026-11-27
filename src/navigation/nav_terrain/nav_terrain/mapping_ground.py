"""Observed free ground for mapping navigation (no ROS).

A ground return proves that the floor between the robot and that return is
clear; a nearer return that rises above the floor proves it is not. Carving the
mask in polar coordinates therefore turns a scattering of isolated ground hits
into the contiguous observed region the planner needs, instead of the sparse
uncertain bounds that force every plan onto the straight-line fallback.
"""
import numpy as np

UNKNOWN = -1
FREE = 0


def _spread(values, reduce, radius):
    """Reduce over a wrapping neighbourhood of the azimuth axis."""
    if radius <= 0:
        return values
    width = values.size
    padded = np.concatenate((values[-radius:], values, values[:radius]))
    return reduce(np.stack([padded[start:start + width] for start in range(2 * radius + 1)]), axis=0)


def ground_evidence(points, base, ground_offset, config, resolution, radius, ray_bins=None):
    if not np.isfinite(resolution) or resolution <= 0.0 or not np.isfinite(radius) or radius <= 0.0:
        raise ValueError('ground grid resolution and radius must be positive and finite')
    base = np.asarray(base, dtype=float)
    if base.shape != (3,) or not np.isfinite(base).all() or not np.isfinite(ground_offset):
        raise ValueError('ground grid reference must be finite')
    bins = getattr(config, 'ray_bins', 720) if ray_bins is None else ray_bins
    if bins < 8:
        raise ValueError('ray count must be at least 8')
    corner = np.floor((base[:2] - radius) / resolution).astype(np.int64)
    size = int(np.ceil(2 * radius / resolution)) + 1
    grid = np.full((size, size), UNKNOWN, dtype=np.int8)
    points = np.asarray(points).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1) &
                    (np.abs(points[:, :2] - base[:2]) <= radius).all(axis=1)]
    cells = np.floor(points[:, :2] / resolution).astype(np.int64) - corner
    height = points[:, 2] - (base[2] - ground_offset)
    floor = np.abs(height) <= config.ground_tolerance
    contested = ~floor & (height <= config.max_height)
    # Block the ray on a return that leaves the floor plane in either direction: a
    # step up is an obstacle and a step down is a drop, and neither is ground the
    # robot may drive across. Deviations inside the ground tolerance are noise.
    obstacle = (np.abs(height) >= config.min_height) & (height <= config.max_height)
    grid[cells[floor, 1], cells[floor, 0]] = FREE
    grid[cells[contested, 1], cells[contested, 0]] = UNKNOWN
    offset = points[:, :2] - base[:2]
    distance = np.linalg.norm(offset, axis=1)
    azimuth = np.floor((np.arctan2(offset[:, 1], offset[:, 0]) + np.pi)
                       * bins / (2 * np.pi)).astype(np.int64) % bins
    reached = np.full(bins, -np.inf)
    np.maximum.at(reached, azimuth[floor], distance[floor])
    blocked = np.full(bins, np.inf)
    np.minimum.at(blocked, azimuth[obstacle], distance[obstacle])
    # A cell can straddle two azimuth bins; widen by one bin each side so the
    # carved region stays contiguous instead of leaving single-cell holes.
    reached = _spread(reached, np.max, 1)
    blocked = _spread(blocked, np.min, 1)
    columns = np.arange(size)
    cell_x = ((corner[0] + columns + 0.5) * resolution - base[0])[None, :]
    cell_y = ((corner[1] + columns + 0.5) * resolution - base[1])[:, None]
    cell_range = np.hypot(cell_x, cell_y)
    cell_bin = np.floor((np.arctan2(cell_y, cell_x) + np.pi)
                        * bins / (2 * np.pi)).astype(np.int64) % bins
    observed = (cell_range <= reached[cell_bin]) & (cell_range < blocked[cell_bin])
    observed &= cell_range <= radius
    grid[observed] = FREE
    # A return above the floor keeps its own cell unknown even when the ray ahead
    # of it is clear: the cell is contested and must not be driven through.
    grid[cells[contested, 1], cells[contested, 0]] = UNKNOWN
    return grid, corner * resolution
