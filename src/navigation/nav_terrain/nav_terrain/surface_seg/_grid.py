"""Height-grid coordinates, segmentation parameters and four-neighbour topology."""
import math
from dataclasses import asdict, dataclass, fields

import numpy as np
from scipy import ndimage


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


EDGES = ((np.s_[:, :-1], np.s_[:, 1:]), (np.s_[:-1], np.s_[1:]))
NEIGHBOURS = tuple(pair for edge in EDGES for pair in (edge, edge[::-1]))


def height_steps(surf, threshold):
    """Yield finite height jumps, signed toward the next column, then the next row."""
    heights = surf.z.astype(np.float64)
    for near, far in EDGES:
        valid = np.isfinite(heights[near]) & np.isfinite(heights[far])
        delta = np.subtract(heights[far], heights[near], where=valid, out=np.zeros_like(heights[near]))
        yield near, far, delta, valid & (np.abs(delta) > threshold)


def gradient_links(surf, params):
    valid = np.isfinite(surf.z)
    heights = np.where(valid, surf.z, 0.0).astype(np.float64)
    covered = surf.clearance.astype(np.float64) < params.roof_height
    links = []
    for near, far in EDGES:
        delta = heights[far] - heights[near]
        joined = valid[near] & valid[far] & (covered[near] == covered[far]) & (np.abs(delta) <= params.jump)
        links.append((delta, joined))
    return valid, links


def cell_gradient(surf, p, links=None):
    """Smooth gradients along connected edges, preferring exact mesh slopes where available."""
    valid, edges = links if links is not None else gradient_links(surf, p)
    smooth = p.smooth if surf.gx is not None else max(p.smooth, p.cloud_smooth)
    window = 2 * max(1, round(smooth / surf.resolution)) + 1
    gradients = []
    supported = valid.copy()
    for pair, (delta, joined) in zip(EDGES, edges):
        total, count = np.zeros(valid.shape), np.zeros(valid.shape)
        values = np.where(joined, delta, 0.0)
        for side in pair:
            total[side] += values
            count[side] += joined
        gradients.append(total / np.maximum(count, 1) / surf.resolution)
        supported &= count > 0
    weight = supported.astype(np.float64)
    density = ndimage.uniform_filter(weight, window, mode='constant')
    def smooth_gradient(gradient):
        total = ndimage.uniform_filter(gradient * weight, window, mode='constant')
        return np.divide(total, density, where=density > 1e-6, out=gradient)

    result = tuple(map(smooth_gradient, gradients))
    if surf.gx is None:
        return result
    exact = valid & np.isfinite(surf.gx)
    return tuple(np.where(exact, mesh, grad) for mesh, grad in zip((surf.gx, surf.gy), result))
