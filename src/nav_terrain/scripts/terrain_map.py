#!/usr/bin/env python3
"""terrain_map: shared data model of the labelled navigation map (HWSentryNav26 style).

The map is a msgpack dict with equally sized uint8 grids, row = y index (row 0 = min y), col = x index:
  width, height, resolution        grid geometry
  origin_x, origin_y               map-frame position of cell (0, 0)'s lower-left corner (extension)
  terrain                          uint8[H*W] semantic label (LABELS below)
  direction                        uint8[H*W] 0..255 -> 0..2pi, map-frame angle of the 'up / forward'
                                   direction of a directional label (ramp up, step up, tunnel axis)
  elevation (optional)             float32[H*W] top-down height map, NaN = no data (extension, display only)
Files written by HWSentryNav26's terrain_label_editor (no origin / elevation) load with origin (0, 0).
"""
import math
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

# ----------------------------------------------------------------------------- labels
FLAT, OBSTACLE, SLOPE, STEP_L1, STEP_L2, FLY_SLOPE, STEP_HIGH, TUNNEL = range(8)
LABELS = {
    FLAT: 'flat', OBSTACLE: 'obstacle', SLOPE: 'slope', STEP_L1: 'step_l1', STEP_L2: 'step_l2',
    FLY_SLOPE: 'fly_slope', STEP_HIGH: 'step_high', TUNNEL: 'tunnel',
}
LABEL_IDS = {v: k for k, v in LABELS.items()}
DIRECTIONAL = {SLOPE, STEP_L1, STEP_L2, FLY_SLOPE, STEP_HIGH, TUNNEL}
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


# ----------------------------------------------------------------------------- minimal msgpack
# Subset needed for the map files: nil, bool, int, float, str, bin, array, map (str keys).
def _pack(o, out):
    if o is None:
        out.append(0xc0)
    elif o is True:
        out.append(0xc3)
    elif o is False:
        out.append(0xc2)
    elif isinstance(o, (int, np.integer)):
        o = int(o)
        if 0 <= o < 0x80:
            out.append(o)
        elif -32 <= o < 0:
            out.append(o & 0xff)
        elif 0 <= o <= 0xffffffff:
            out += b'\xce' + struct.pack('>I', o)
        elif 0 <= o:
            out += b'\xcf' + struct.pack('>Q', o)
        else:
            out += b'\xd3' + struct.pack('>q', o)
    elif isinstance(o, (float, np.floating)):
        out += b'\xcb' + struct.pack('>d', float(o))
    elif isinstance(o, str):
        b = o.encode()
        n = len(b)
        if n < 32:
            out.append(0xa0 | n)
        elif n < 0x100:
            out += b'\xd9' + struct.pack('>B', n)
        elif n < 0x10000:
            out += b'\xda' + struct.pack('>H', n)
        else:
            out += b'\xdb' + struct.pack('>I', n)
        out += b
    elif isinstance(o, (bytes, bytearray, memoryview)):
        b = bytes(o)
        n = len(b)
        if n < 0x100:
            out += b'\xc4' + struct.pack('>B', n)
        elif n < 0x10000:
            out += b'\xc5' + struct.pack('>H', n)
        else:
            out += b'\xc6' + struct.pack('>I', n)
        out += b
    elif isinstance(o, (list, tuple)):
        n = len(o)
        if n < 16:
            out.append(0x90 | n)
        elif n < 0x10000:
            out += b'\xdc' + struct.pack('>H', n)
        else:
            out += b'\xdd' + struct.pack('>I', n)
        for v in o:
            _pack(v, out)
    elif isinstance(o, dict):
        n = len(o)
        if n < 16:
            out.append(0x80 | n)
        elif n < 0x10000:
            out += b'\xde' + struct.pack('>H', n)
        else:
            out += b'\xdf' + struct.pack('>I', n)
        for k, v in o.items():
            _pack(k, out)
            _pack(v, out)
    else:
        raise TypeError('cannot msgpack %r' % type(o))


def packb(o):
    out = bytearray()
    _pack(o, out)
    return bytes(out)


_FIXED = {  # code: (struct fmt, size)
    0xca: ('>f', 4), 0xcb: ('>d', 8),
    0xcc: ('>B', 1), 0xcd: ('>H', 2), 0xce: ('>I', 4), 0xcf: ('>Q', 8),
    0xd0: ('>b', 1), 0xd1: ('>h', 2), 0xd2: ('>i', 4), 0xd3: ('>q', 8),
}


def _unpack(b, i):
    c = b[i]
    i += 1
    if c < 0x80:
        return c, i
    if c >= 0xe0:
        return c - 0x100, i
    if 0x80 <= c <= 0x8f:
        return _unpack_map(b, i, c & 0x0f)
    if 0x90 <= c <= 0x9f:
        return _unpack_arr(b, i, c & 0x0f)
    if 0xa0 <= c <= 0xbf:
        n = c & 0x1f
        return b[i:i + n].decode(), i + n
    if c == 0xc0:
        return None, i
    if c in (0xc2, 0xc3):
        return c == 0xc3, i
    if c in _FIXED:
        fmt, n = _FIXED[c]
        return struct.unpack_from(fmt, b, i)[0], i + n
    if c in (0xc4, 0xc5, 0xc6, 0xd9, 0xda, 0xdb):
        fmt, sz = {0xc4: ('>B', 1), 0xc5: ('>H', 2), 0xc6: ('>I', 4),
                   0xd9: ('>B', 1), 0xda: ('>H', 2), 0xdb: ('>I', 4)}[c]
        n = struct.unpack_from(fmt, b, i)[0]
        i += sz
        raw = bytes(b[i:i + n])
        return (raw if c <= 0xc6 else raw.decode()), i + n
    if c in (0xdc, 0xdd):
        fmt, sz = ('>H', 2) if c == 0xdc else ('>I', 4)
        return _unpack_arr(b, i + sz, struct.unpack_from(fmt, b, i)[0])
    if c in (0xde, 0xdf):
        fmt, sz = ('>H', 2) if c == 0xde else ('>I', 4)
        return _unpack_map(b, i + sz, struct.unpack_from(fmt, b, i)[0])
    raise ValueError('unsupported msgpack type 0x%02x at %d' % (c, i - 1))


def _unpack_arr(b, i, n):
    out = []
    for _ in range(n):
        v, i = _unpack(b, i)
        out.append(v)
    return out, i


def _unpack_map(b, i, n):
    out = {}
    for _ in range(n):
        k, i = _unpack(b, i)
        v, i = _unpack(b, i)
        out[k.decode() if isinstance(k, bytes) else k] = v
    return out, i


def unpackb(b):
    b = bytes(b)
    v, i = _unpack(b, 0)
    if i != len(b):
        raise ValueError('trailing bytes after msgpack object')
    return v


# ----------------------------------------------------------------------------- map model
@dataclass
class TerrainMap:
    terrain: np.ndarray                 # uint8 [H, W]
    direction: np.ndarray               # uint8 [H, W]
    resolution: float
    origin: tuple = (0.0, 0.0)
    elevation: Optional[np.ndarray] = None   # float32 [H, W], NaN = unknown
    extra: dict = field(default_factory=dict)
    # automatic layer (mesh_to_terrain) + mask of hand-edited cells; terrain/direction = merged result
    auto_terrain: Optional[np.ndarray] = None
    auto_direction: Optional[np.ndarray] = None
    manual: Optional[np.ndarray] = None      # bool [H, W]

    def ensure_manual(self):
        if self.manual is None:
            self.manual = np.zeros(self.terrain.shape, bool)
        return self.manual

    def merge_auto(self, auto_terrain, auto_direction):
        """Install a new automatic layer; hand-edited cells keep their label and direction."""
        man = self.ensure_manual()
        self.auto_terrain = np.asarray(auto_terrain, np.uint8).copy()
        self.auto_direction = np.asarray(auto_direction, np.uint8).copy()
        self.terrain = np.where(man, self.terrain, self.auto_terrain).astype(np.uint8)
        self.direction = np.where(man, self.direction, self.auto_direction).astype(np.uint8)

    def revert_to_auto(self, mask):
        """Drop hand edits inside mask (no-op without an automatic layer)."""
        if self.auto_terrain is None:
            return 0
        mask = mask & self.ensure_manual()
        self.terrain[mask] = self.auto_terrain[mask]
        self.direction[mask] = self.auto_direction[mask]
        self.manual[mask] = False
        return int(mask.sum())

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
        for name in ('elevation', 'auto_terrain', 'auto_direction', 'manual'):
            g = getattr(self, name)
            if g is not None and g.shape != self.terrain.shape:
                raise ValueError('%s shape mismatch' % name)

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
        if self.auto_terrain is not None:
            d['auto_terrain'] = np.ascontiguousarray(self.auto_terrain, np.uint8).tobytes()
            d['auto_direction'] = np.ascontiguousarray(self.auto_direction, np.uint8).tobytes()
        if self.manual is not None:
            d['manual'] = np.ascontiguousarray(self.manual, np.uint8).tobytes()
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

        known = {'width', 'height', 'resolution', 'origin_x', 'origin_y', 'terrain', 'direction', 'elevation',
                 'auto_terrain', 'auto_direction', 'manual'}
        has_auto = 'auto_terrain' in d and 'auto_direction' in d
        m = cls(grid('terrain', np.uint8), grid('direction', np.uint8), float(d.get('resolution', 0.05)),
                (float(d.get('origin_x', 0.0)), float(d.get('origin_y', 0.0))),
                grid('elevation', '<f4').astype(np.float32) if 'elevation' in d else None,
                {k: v for k, v in d.items() if k not in known},
                grid('auto_terrain', np.uint8) if has_auto else None,
                grid('auto_direction', np.uint8) if has_auto else None,
                grid('manual', np.uint8).astype(bool) if 'manual' in d else None)
        m.validate()
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


# ----------------------------------------------------------------------------- point clouds
def _lzf_decompress(src, out_len):
    out = bytearray(out_len)
    ip = op = 0
    n = len(src)
    while ip < n:
        ctrl = src[ip]
        ip += 1
        if ctrl < 32:
            ln = ctrl + 1
            out[op:op + ln] = src[ip:ip + ln]
            ip += ln
            op += ln
        else:
            ln = ctrl >> 5
            ref = op - ((ctrl & 0x1f) << 8) - 1
            if ln == 7:
                ln += src[ip]
                ip += 1
            ref -= src[ip]
            ip += 1
            ln += 2
            for _ in range(ln):
                out[op] = out[ref]
                op += 1
                ref += 1
    return bytes(out)


def read_pcd(path):
    """xyz (float64 N x 3) from an ascii / binary / binary_compressed PCD."""
    data = Path(path).read_bytes()
    end = data.index(b'DATA ')
    line_end = data.index(b'\n', end) + 1
    kind = data[end:line_end].split()[1].decode()
    hdr = {l.split()[0]: l.split()[1:] for l in data[:end].decode().splitlines() if l and not l.startswith('#')}
    fields, sizes, types = hdr['FIELDS'], [int(s) for s in hdr['SIZE']], hdr['TYPE']
    counts = [int(c) for c in hdr.get('COUNT', ['1'] * len(fields))]
    npts = int(hdr['POINTS'][0])
    if any(c != 1 for c in counts):
        raise ValueError('PCD fields with COUNT != 1 are not supported: %s' % path)
    dts = [np.dtype({'F': 'f', 'U': 'u', 'I': 'i'}[t] + str(s)) for t, s in zip(types, sizes)]
    if kind == 'ascii':
        arr = np.loadtxt(data[line_end:].decode().splitlines(), ndmin=2)
        idx = [fields.index(k) for k in 'xyz']
        return arr[:, idx].astype(np.float64)
    if kind == 'binary':
        arr = np.frombuffer(data, np.dtype(list(zip(fields, dts))), npts, line_end)
        return np.stack([arr['x'], arr['y'], arr['z']], 1).astype(np.float64)
    if kind == 'binary_compressed':
        csize, usize = struct.unpack_from('<II', data, line_end)
        raw = _lzf_decompress(data[line_end + 8:line_end + 8 + csize], usize)
        cols, off = {}, 0
        for f, dt in zip(fields, dts):
            cols[f] = np.frombuffer(raw, dt, npts, off)
            off += dt.itemsize * npts
        return np.stack([cols['x'], cols['y'], cols['z']], 1).astype(np.float64)
    raise ValueError('unsupported PCD DATA %s: %s' % (kind, path))


def _quat_mat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_keyframe_cloud(map_dir):
    """Backend map bundle (keyframes.txt: id stamp tx ty tz qx qy qz qw, clouds/NNNNNN.pcd) -> map-frame xyz."""
    map_dir = Path(map_dir)
    pts = []
    rows = [l.split() for l in (map_dir / 'keyframes.txt').read_text().splitlines() if l and not l.startswith('#')]
    for r in rows:
        f = map_dir / 'clouds' / ('%06d.pcd' % int(r[0]))
        if not f.is_file():
            continue
        p = read_pcd(f)
        p = p[np.isfinite(p).all(1)]
        t = np.array([float(v) for v in r[2:5]])
        pts.append(p @ _quat_mat(*[float(v) for v in r[5:9]]).T + t)
    if not pts:
        raise FileNotFoundError('no keyframe clouds under %s' % map_dir)
    return np.vstack(pts)
