"""terrain_map: shared data model of the labelled navigation map (HWSentryNav26 style).

The map is a msgpack dict with equally sized uint8 grids, row = y index (row 0 = min y), col = x index:
  width, height, resolution        grid geometry
  origin_x, origin_y               map-frame position of cell (0, 0)'s lower-left corner (extension)
  terrain                          uint8[H*W] semantic label (LABELS below)
  direction                        uint8[H*W] 0..255 -> 0..2pi, map-frame angle of the 'up / forward'
                                   direction of a directional label (ramp up, step up, tunnel axis)
  elevation (optional)             float32[H*W] top-down height map, NaN = no data (extension, display only)
Written by surface_studio (surface_project.terrain_map) and read by terrain_export.
"""
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from nav_terrain._msgpack import packb, unpackb

# ----------------------------------------------------------------------------- labels
FLAT, OBSTACLE, SLOPE, STEP_L1, STEP_L2, FLY_SLOPE, STEP_HIGH, TUNNEL, UNDULATION = range(9)
LABELS = {
    FLAT: 'flat', OBSTACLE: 'obstacle', SLOPE: 'slope', STEP_L1: 'step_l1', STEP_L2: 'step_l2',
    FLY_SLOPE: 'fly_slope', STEP_HIGH: 'step_high', TUNNEL: 'tunnel', UNDULATION: 'undulation',
}
LABEL_IDS = {v: k for k, v in LABELS.items()}
DIRECTIONAL = {SLOPE, STEP_L1, STEP_L2, FLY_SLOPE, STEP_HIGH, TUNNEL, UNDULATION}
FIRST_DIRECTIONAL = SLOPE
LABEL_COUNT = len(LABELS)


def is_directional(label):
    return int(label) in DIRECTIONAL


def encode_angle(rad):
    """map-frame angle (rad) -> uint8 direction code."""
    a = math.fmod(rad, 2 * math.pi)
    if a < 0:
        a += 2 * math.pi
    return max(0, min(255, int(round(a / (2 * math.pi) * 255.0))))


def decode_angle(code):
    return np.asarray(code, dtype=np.float64) / 255.0 * 2.0 * math.pi


@dataclass
class TerrainMap:
    terrain: np.ndarray                 # uint8 [H, W]
    direction: np.ndarray               # uint8 [H, W]
    resolution: float
    origin: tuple = (0.0, 0.0)
    elevation: Optional[np.ndarray] = None   # float32 [H, W], NaN = unknown
    extra: dict = field(default_factory=dict)
    clearance: Optional[np.ndarray] = None  # metres above ground, inf = open sky

    @property
    def height(self):
        return self.terrain.shape[0]

    @property
    def width(self):
        return self.terrain.shape[1]

    @classmethod
    def blank(cls, width, height, resolution, origin=(0.0, 0.0)):
        return cls(np.zeros((height, width), np.uint8), np.zeros((height, width), np.uint8),
                   float(resolution), (float(origin[0]), float(origin[1])))

    def cell_center(self, col, row):
        return (self.origin[0] + (col + 0.5) * self.resolution,
                self.origin[1] + (row + 0.5) * self.resolution)

    def cell_of(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def validate(self):
        if self.terrain.shape != self.direction.shape or self.terrain.ndim != 2:
            raise ValueError('terrain/direction shape mismatch')
        if not (math.isfinite(self.resolution) and self.resolution > 0):
            raise ValueError('resolution must be finite and positive')
        bad = self.terrain >= LABEL_COUNT
        if bad.any():
            r, c = np.argwhere(bad)[0]
            raise ValueError('invalid terrain label %d at row %d col %d' % (self.terrain[r, c], r, c))
        if self.elevation is not None and self.elevation.shape != self.terrain.shape:
            raise ValueError('elevation shape mismatch')
        if self.clearance is not None and self.clearance.shape != self.terrain.shape:
            raise ValueError('clearance shape mismatch')

    def to_dict(self):
        self.validate()
        d = dict(self.extra)
        d.update({
            'width': self.width, 'height': self.height, 'resolution': float(self.resolution),
            'origin_x': float(self.origin[0]), 'origin_y': float(self.origin[1]),
            'terrain': np.ascontiguousarray(self.terrain, np.uint8).tobytes(),
            'direction': np.ascontiguousarray(self.direction, np.uint8).tobytes(),
        })
        if self.elevation is not None:
            d['elevation'] = np.ascontiguousarray(self.elevation, '<f4').tobytes()
        if self.clearance is not None:
            d['clearance'] = np.ascontiguousarray(self.clearance, '<f4').tobytes()
        return d

    @classmethod
    def from_dict(cls, d):
        w, h = int(d['width']), int(d['height'])
        if w <= 0 or h <= 0:
            raise ValueError('map dimensions must be positive')
        n = w * h

        def grid(key, dt):
            raw = d[key]
            if isinstance(raw, list):     # tolerate arrays of ints
                raw = bytes(raw)
            a = np.frombuffer(raw, dt)
            if a.size != n:
                raise ValueError('%s size mismatch: expected %d, got %d' % (key, n, a.size))
            return a.reshape(h, w).copy()

        known = {'width', 'height', 'resolution', 'origin_x', 'origin_y', 'terrain', 'direction', 'elevation', 'clearance'}
        m = cls(grid('terrain', np.uint8), grid('direction', np.uint8), float(d.get('resolution', 0.05)),
                (float(d.get('origin_x', 0.0)), float(d.get('origin_y', 0.0))),
                grid('elevation', '<f4').astype(np.float32) if 'elevation' in d else None,
                {k: v for k, v in d.items() if k not in known})
        m.validate()
        if 'clearance' in d:
            m.clearance = grid('clearance', '<f4').astype(np.float32)
        return m

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + '.tmp')
        tmp.write_bytes(packb(self.to_dict()))
        tmp.replace(path)

    @classmethod
    def load(cls, path):
        return cls.from_dict(unpackb(Path(path).read_bytes()))
