"""Read PCD and assemble backend keyframes in map coordinates."""
import struct
from pathlib import Path

import numpy as np

from nav_terrain.registration import quaternion_matrix


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
        pts.append(p @ quaternion_matrix(*[float(v) for v in r[5:9]]).T + t)
    if not pts:
        raise FileNotFoundError('no keyframe clouds under %s' % map_dir)
    return np.vstack(pts)
