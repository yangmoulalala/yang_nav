"""MessagePack types used by the terrain grid format."""
import struct

import numpy as np


_LENGTHS = {'str': (0xa0, 32, 0xd9, 0xda, 0xdb), 'bin': (0, 0, 0xc4, 0xc5, 0xc6),
            'array': (0x90, 16, None, 0xdc, 0xdd), 'map': (0x80, 16, None, 0xde, 0xdf)}


def _pack_length(out, kind, length):
    fixed, limit, short, medium, long = _LENGTHS[kind]
    if length < limit:
        out.append(fixed + length)
    elif short is not None and length < 256:
        out.extend((short, length))
    elif length < 65536:
        out.extend(struct.pack('>BH', medium, length))
    else:
        out.extend(struct.pack('>BI', long, length))


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
    elif isinstance(o, (str, bytes, bytearray, memoryview)):
        text = isinstance(o, str)
        payload = o.encode() if text else bytes(o)
        _pack_length(out, 'str' if text else 'bin', len(payload))
        out.extend(payload)
    elif isinstance(o, (list, tuple)):
        _pack_length(out, 'array', len(o))
        for v in o:
            _pack(v, out)
    elif isinstance(o, dict):
        _pack_length(out, 'map', len(o))
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
