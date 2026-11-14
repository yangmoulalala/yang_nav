#!/usr/bin/env python3
"""Ground-relative obstacle extraction and conservative clearing rays (no ROS).

Coordinates and heights are in the terrain package's map frame. Never treat the
top of an OBSTACLE-labelled cell as traversable ground. Unmapped cells use the
robot's current ground height; no distant height is extrapolated across steps.
"""
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from terrain_map import OBSTACLE, TerrainMap


@dataclass
class FilterConfig:
    min_height: float = 0.06
    max_height: float = 0.24
    ground_tolerance: float = 0.04
    ceiling_tolerance: float = 0.01
    max_range: float = 6.0
    ray_bins: int = 720
    fill_radius: float = 0.15
    terrain_edge_height: float = 0.08
    terrain_surface_tolerance: float = 0.04
    terrain_edge_xy_tolerance: float = 0.035

    def validate(self):
        values = (self.min_height, self.max_height, self.ground_tolerance,
                  self.ceiling_tolerance, self.max_range, self.fill_radius,
                  self.terrain_edge_height, self.terrain_surface_tolerance,
                  self.terrain_edge_xy_tolerance)
        if not all(np.isfinite(v) for v in values):
            raise ValueError('filter parameters must be finite')
        if not (0 <= self.ground_tolerance < self.min_height < self.max_height):
            raise ValueError('require 0 <= ground_tolerance < min_height < max_height')
        if (self.max_range <= 0 or self.fill_radius < 0 or self.ceiling_tolerance < 0 or
                self.terrain_edge_height < 0 or self.terrain_surface_tolerance < 0 or
                self.terrain_edge_xy_tolerance < 0 or self.ray_bins < 8):
            raise ValueError('invalid range, interpolation radius, ceiling tolerance or ray count')


class GroundFilter:
    def __init__(self, terrain: TerrainMap, config=None):
        terrain.validate()
        self.config = config or FilterConfig()
        self.config.validate()
        if terrain.elevation is None:
            raise ValueError('terrain.msgpack has no elevation; re-export the surface project')
        self.resolution, self.origin = terrain.resolution, np.asarray(terrain.origin)
        self.terrain = np.array(terrain.terrain, copy=True)
        self.ground = np.array(terrain.elevation, dtype=float, copy=True)
        self.ground[terrain.terrain == OBSTACLE] = np.nan
        known = np.isfinite(self.ground)
        if not known.any():
            raise ValueError('terrain package has no traversable ground heights')
        distance, nearest = ndimage.distance_transform_edt(~known, return_indices=True)
        fill = ~known & (distance * self.resolution <= self.config.fill_radius)
        self.ground[fill] = self.ground[tuple(nearest[:, fill])]
        self.clearance = (np.full(self.ground.shape, np.inf) if terrain.clearance is None
                          else np.asarray(terrain.clearance, dtype=float))
        # Riser returns can appear in the lower cell at a registered step. Use
        # only cardinal, in-map neighbors: np.roll would wrap the far map edge
        # onto the opposite edge and could erase unrelated dynamic obstacles.
        finite_ground = np.where(np.isfinite(self.ground), self.ground, -np.inf)
        h, w = finite_ground.shape
        self.edge_neighbors = []
        self.neighbor_ground = np.full(self.ground.shape, -np.inf, dtype=float)
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            neighbor = np.full(self.ground.shape, -np.inf, dtype=float)
            r0, r1 = max(0, -dr), min(h, h - dr)
            c0, c1 = max(0, -dc), min(w, w - dc)
            neighbor[r0:r1, c0:c1] = finite_ground[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
            self.edge_neighbors.append((dr, dc, neighbor))
            self.neighbor_ground = np.maximum(self.neighbor_ground, neighbor)
        self.terrain_edge_rise = np.where(
            np.isfinite(self.ground), np.maximum(self.neighbor_ground - self.ground, 0.0), 0.0)

    def sample(self, xy, fallback):
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        indices = np.floor((xy - self.origin) / self.resolution).astype(np.int64)
        cols, rows = indices.T
        valid = (rows >= 0) & (rows < self.ground.shape[0]) & (cols >= 0) & (cols < self.ground.shape[1])
        ground, ceiling = np.full(len(xy), fallback, dtype=float), np.full(len(xy), np.inf)
        ground[valid] = self.ground[rows[valid], cols[valid]]
        ceiling[valid] = self.clearance[rows[valid], cols[valid]]
        ground[~np.isfinite(ground)] = fallback
        ceiling[~np.isfinite(ceiling)] = np.inf
        return ground, ceiling

    def select(self, points_map, points_base, sensor_map, fallback_ground):
        """Return source indices for obstacle hits and clearing endpoints.

        A clearing ray ends at the nearest obstacle in its azimuth bin. Where
        there is no hit it ends at the farthest observed ground point. Ceiling,
        invalid and unseen returns never manufacture free space. Publish rays
        in the input sensor frame so Nav2 uses the actual sensor origin.
        """
        p = np.asarray(points_map, dtype=float).reshape(-1, 3)
        base = np.asarray(points_base, dtype=float).reshape(-1, 3)
        if p.shape != base.shape:
            raise ValueError('map and base point arrays must match')
        cfg = self.config
        delta = p[:, :2] - np.asarray(sensor_map)[:2]
        distance = np.linalg.norm(delta, axis=1)
        valid = np.isfinite(p).all(axis=1) & np.isfinite(base).all(axis=1)
        valid &= (distance > 0.05) & (distance <= cfg.max_range)
        # Include the padded chassis and turret, also on an inclined robot.
        self_return = ((base[:, 0] > -0.34) & (base[:, 0] < 0.42) & (np.abs(base[:, 1]) < 0.34))
        valid &= ~self_return
        ids = np.flatnonzero(valid)
        if not len(ids):
            return ids, ids.copy()
        ground, ceiling = self.sample(p[ids, :2], fallback_ground)
        height = p[ids, 2] - ground
        hits = (height >= cfg.min_height) & (height <= cfg.max_height)
        hits &= height < ceiling - cfg.ceiling_tolerance
        # Keep the conservative obstacle rule for explicitly labelled cells.
        # A registered riser may look like a point obstacle, but discard a
        # return only when its XY is close to a cardinal terrain boundary and
        # its height lies between that cell's ground and the known higher side.
        # Dynamic objects elsewhere in the lower cell remain visible.
        cols, rows = np.floor((p[ids, :2] - self.origin) / self.resolution).astype(np.int64).T
        in_grid = ((rows >= 0) & (rows < self.ground.shape[0]) &
                   (cols >= 0) & (cols < self.ground.shape[1]))
        terrain_return = np.zeros(len(ids), dtype=bool)
        if np.any(in_grid):
            inside = np.flatnonzero(in_grid)
            rr, cc = rows[inside], cols[inside]
            labels = self.terrain[rr, cc]
            local_ground = self.ground[rr, cc]
            traversable = labels != OBSTACLE
            x0 = self.origin[0] + cc * self.resolution
            y0 = self.origin[1] + rr * self.resolution
            local = p[ids[inside]]
            riser = np.zeros(len(inside), dtype=bool)
            for dr, dc, neighbor in self.edge_neighbors:
                higher = neighbor[rr, cc]
                edge = np.isfinite(higher) & (higher - local_ground >= cfg.terrain_edge_height)
                if dr < 0:
                    edge_distance = np.abs(local[:, 1] - y0)
                elif dr > 0:
                    edge_distance = np.abs(local[:, 1] - (y0 + self.resolution))
                elif dc < 0:
                    edge_distance = np.abs(local[:, 0] - x0)
                else:
                    edge_distance = np.abs(local[:, 0] - (x0 + self.resolution))
                between_surfaces = ((local[:, 2] >= local_ground + cfg.min_height) &
                                    (local[:, 2] <= higher + cfg.terrain_surface_tolerance))
                riser |= edge & (edge_distance <= cfg.terrain_edge_xy_tolerance) & between_surfaces
            terrain_return[inside] = traversable & riser
        hits &= ~terrain_return
        ground_return = np.abs(height) <= cfg.ground_tolerance
        bins = np.floor((np.arctan2(delta[ids, 1], delta[ids, 0]) + np.pi)
                        * cfg.ray_bins / (2 * np.pi)).astype(int) % cfg.ray_bins
        obstacle_ids = ids[hits]
        # Sort once, then reduce each observed bin. The farthest floor return
        # clears a disappearing obstacle only after free ground is observed.
        nearest_hit = np.full(cfg.ray_bins, np.inf)
        np.minimum.at(nearest_hit, bins[hits], distance[obstacle_ids])
        candidates = hits | (ground_return & (distance[ids] < nearest_hit[bins]))
        chosen = {}
        for j in np.flatnonzero(candidates)[np.argsort(distance[ids[candidates]])]:
            b = bins[j]
            if np.isfinite(nearest_hit[b]):
                if hits[j] and distance[ids[j]] <= nearest_hit[b]:
                    chosen[b] = ids[j]
            else:
                chosen[b] = ids[j]
        return obstacle_ids, np.asarray(list(chosen.values()), dtype=np.int64)
