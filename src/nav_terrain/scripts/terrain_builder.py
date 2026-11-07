#!/usr/bin/env python3
"""terrain_builder: offline 2.5-D terrain package from the prior point-cloud map (no field mesh).

    terrain_builder.py --map maps/sim --out maps/sim/terrain [--registration-from maps/sim/nav]

Input : backend map bundle (keyframes.txt + clouds/NNNNNN.pcd, one sensor-frame cloud per keyframe).
Output (same layout as the old nav dir, so nav_bringup terrain_dir:=... works unchanged):
  nav_map.pgm/.yaml       traversability cost (scale mode: 255 free .. 0 lethal)
  speed_mask.pgm/.yaml    low-clearance speed limit (raw mode, percent)
  ground_z.f32 clearance.f32 slope.f32   float32 LE, index ix*ny+iy, NaN = not reachable
  zones.yaml              tunnels (low-clearance passages) and ramps, with approach poses
  terrain.json            grid metadata

Pipeline
  1 keyframe clouds -> map frame
  2 per cell: z layers (split where the gap > layer_gap); clearance = gap to the next layer
  3 fill unobserved floor: empty cells, and floor hidden under a roof the LiDAR saw from above
  4 reachability flood fill over the layers from the seeds (clearance and per-cell step limits)
  5 slope from finite differences between connected cells only (no blending across steps),
    roughness, step edges, cost image
  6 zones: tunnels = connected areas with clearance < tunnel_max_clearance, passage axis = the
    axis whose both ends open to the sky; ramps = connected areas with slope in [ramp_min, slope_max]
"""
import argparse
import json
import math
import struct
import sys
from collections import deque
from pathlib import Path

import numpy as np
from scipy import ndimage

N8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
OPEN = 99.0   # clearance of a surface with nothing above it


# ----------------------------------------------------------------------------- point cloud io
def lzf_decompress(src, out_len):
    out = bytearray(out_len)
    ip = op = 0
    n = len(src)
    while ip < n:
        ctrl = src[ip]; ip += 1
        if ctrl < 32:
            ln = ctrl + 1
            out[op:op + ln] = src[ip:ip + ln]
            ip += ln; op += ln
        else:
            ln = ctrl >> 5
            ref = op - ((ctrl & 0x1f) << 8) - 1
            if ln == 7:
                ln += src[ip]; ip += 1
            ref -= src[ip]; ip += 1
            ln += 2
            for _ in range(ln):
                out[op] = out[ref]; op += 1; ref += 1
    return bytes(out)


def read_pcd(path):
    data = Path(path).read_bytes()
    end = data.index(b'DATA ')
    line_end = data.index(b'\n', end) + 1
    kind = data[end:line_end].split()[1].decode()
    hdr = {l.split()[0]: l.split()[1:] for l in data[:end].decode().splitlines() if l and not l.startswith('#')}
    fields, sizes, types = hdr['FIELDS'], [int(s) for s in hdr['SIZE']], hdr['TYPE']
    npts = int(hdr['POINTS'][0])
    dts = [np.dtype({'F': 'f', 'U': 'u', 'I': 'i'}[t] + str(s)) for t, s in zip(types, sizes)]
    if kind == 'binary':
        arr = np.frombuffer(data, np.dtype(list(zip(fields, dts))), npts, line_end)
        return np.stack([arr['x'], arr['y'], arr['z']], 1).astype(np.float64)
    if kind == 'binary_compressed':
        csize, usize = struct.unpack_from('<II', data, line_end)
        raw = lzf_decompress(data[line_end + 8:line_end + 8 + csize], usize)
        cols, off = {}, 0
        for f, dt in zip(fields, dts):
            cols[f] = np.frombuffer(raw, dt, npts, off); off += dt.itemsize * npts
        return np.stack([cols['x'], cols['y'], cols['z']], 1).astype(np.float64)
    raise ValueError('unsupported PCD DATA %s: %s' % (kind, path))


def quat_mat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load_keyframe_cloud(map_dir):
    map_dir = Path(map_dir)
    pts, used = [], 0
    rows = [l.split() for l in (map_dir / 'keyframes.txt').read_text().splitlines() if l and not l.startswith('#')]
    for r in rows:
        f = map_dir / 'clouds' / ('%06d.pcd' % int(r[0]))
        if not f.is_file():
            continue
        p = read_pcd(f)
        p = p[np.isfinite(p).all(1)]
        t = np.array([float(v) for v in r[2:5]])
        pts.append(p @ quat_mat(*[float(v) for v in r[5:9]]).T + t)
        used += 1
    return np.vstack(pts), used


# ----------------------------------------------------------------------------- grid helpers
def neighbours(a):
    """8 shifted copies of a 2-D array (edges replicate the border instead of wrapping)."""
    p = np.pad(a, 1, mode='edge')
    nx, ny = a.shape
    return np.stack([p[1 + dx:1 + dx + nx, 1 + dy:1 + dy + ny] for dx, dy in N8])


def nconv(v, sigma):
    """NaN-aware gaussian smoothing (normalized convolution)."""
    w = np.isfinite(v).astype(float)
    num = ndimage.gaussian_filter(np.nan_to_num(v) * w, sigma)
    den = ndimage.gaussian_filter(w, sigma)
    with np.errstate(all='ignore'):
        return np.where(den > 0.2, num / den, np.nan)


def ramp01(v, lo, hi):
    return np.clip((v - lo) / max(hi - lo, 1e-9), 0.0, 1.0)


# ----------------------------------------------------------------------------- pipeline steps
def column_layers(P, x0, y0, res, nx, ny, layers, gap, min_pts):
    """Per cell, up to `layers` surfaces (z, clearance), lowest first. Returns Z, C as (L, nx, ny)."""
    ix = np.floor((P[:, 0] - x0) / res).astype(np.int64)
    iy = np.floor((P[:, 1] - y0) / res).astype(np.int64)
    ok = (ix >= 0) & (iy >= 0) & (ix < nx) & (iy < ny)
    cell = ix[ok] * ny + iy[ok]; z = P[ok, 2]
    order = np.lexsort((z, cell)); cell, z = cell[order], z[order]
    Z = np.full((layers, nx * ny), np.nan); C = np.full((layers, nx * ny), np.nan)
    cnt = np.zeros(nx * ny, np.int32)
    if len(cell) == 0:
        return Z.reshape(layers, nx, ny), C.reshape(layers, nx, ny), cnt.reshape(nx, ny)
    b = np.flatnonzero(np.diff(cell)) + 1
    for s, e in zip(np.concatenate([[0], b]), np.concatenate([b, [len(cell)]])):
        zz = z[s:e]; c = cell[s]; cnt[c] = e - s
        cut = np.flatnonzero(np.diff(zz) > gap) + 1
        segs = [g for g in np.split(zz, cut) if len(g) >= min_pts]
        surf = []
        for k, g in enumerate(segs):
            # a tall segment is a wall face: its top is the surface (unreachable from the floor)
            top = float(g[-1]) if g[-1] - g[0] > gap else float(np.median(g))
            nxt = segs[k + 1] if k + 1 < len(segs) else None
            clear = OPEN if nxt is None else float(np.median(nxt[:min(len(nxt), 3)]) - top)
            surf.append((top, clear))
        for k, (zk, ck) in enumerate(surf[:layers]):
            Z[k, c] = zk; C[k, c] = ck
    return Z.reshape(layers, nx, ny), C.reshape(layers, nx, ny), cnt.reshape(nx, ny)


def fill_floor(Z, C, iters, lift, roof_tol):
    """Fill floor the LiDAR did not see.

    * empty cell with >= 5 observed neighbours: lowest layer = neighbour median, clearance = neighbour min
    * cell whose lowest surface sits `lift` above the neighbours' floor and matches the neighbours'
      roof (floor + clearance): the floor under a roof was hidden, insert it below.
    """
    Z = Z.copy(); C = C.copy()
    filled = roofed = 0
    for _ in range(iters):
        z0, c0 = Z[0].copy(), C[0].copy()   # copies: Z/C are modified below
        nz = neighbours(z0)
        roofed_cell = np.isfinite(c0) & (c0 < OPEN - 1)
        roof = np.where(roofed_cell, z0 + c0, np.nan)
        nroof = neighbours(roof)
        nfloor = neighbours(np.where(roofed_cell, z0, np.nan))   # floor of neighbours that have a roof
        with np.errstate(all='ignore'):
            mz = np.nanmedian(nz, 0)
            mc = np.nanmin(neighbours(c0), 0)
            mroof = np.nanmedian(nroof, 0)
            mfloor = np.nanmedian(nfloor, 0)
        n = np.isfinite(nz).sum(0)
        hole = ~np.isfinite(z0) & (n >= 5)
        Z[0][hole] = mz[hole]; C[0][hole] = mc[hole]
        # lowest surface is a roof seen from above while roofed neighbours have floor under it
        under = (np.isfinite(z0) & ~roofed_cell & np.isfinite(mfloor) & np.isfinite(mroof)
                 & (z0 > mfloor + lift) & (np.abs(z0 - mroof) < roof_tol) & (np.isfinite(nroof).sum(0) >= 2))
        if under.any():
            Z[1:, under] = Z[:-1, under]; C[1:, under] = C[:-1, under]
            Z[0][under] = mfloor[under]
            C[0][under] = np.minimum(z0[under], mroof[under]) - mfloor[under]
        filled += int(hole.sum()); roofed += int(under.sum())
    return Z, C, filled, roofed


def load_manual(path):
    """Hand-annotated tunnels (zones_manual.yaml): floor the prior map never saw.

    tunnels:
      - id: blue_tunnel
        centre: [x, y]        # map frame
        yaw_deg: 0            # rectangle orientation (= passage direction)
        size: [length, width] # m, along / across yaw; keep the side walls outside
        floor_z: 0.047        # m, map frame
        roof_z: 0.297         # m, underside of the roof
    """
    import yaml
    data = yaml.safe_load(Path(path).read_text()) or {}
    out = []
    for t in data.get('tunnels', []):
        for k in ('centre', 'size', 'floor_z', 'roof_z'):
            if k not in t:
                raise ValueError('manual tunnel %s: missing %s' % (t.get('id', '?'), k))
        out.append(dict(id=t.get('id', 'manual_%d' % len(out)), centre=[float(v) for v in t['centre']],
                        yaw=math.radians(float(t.get('yaw_deg', 0.0))), size=[float(v) for v in t['size']],
                        floor_z=float(t['floor_z']), roof_z=float(t['roof_z'])))
    return out


def manual_cells(t, x0, y0, res, nx, ny):
    I, J = np.indices((nx, ny))
    X, Y = x0 + (I + 0.5) * res - t['centre'][0], y0 + (J + 0.5) * res - t['centre'][1]
    c, s = math.cos(t['yaw']), math.sin(t['yaw'])
    u, v = c * X + s * Y, -s * X + c * Y
    return (np.abs(u) <= t['size'][0] / 2) & (np.abs(v) <= t['size'][1] / 2)


def apply_manual(Z, C, manual, x0, y0, res, roof_tol=0.03, approach=0.0, observed=None, ramp_rise=0.15):
    """Inside each annotated rectangle: lowest layer = floor_z with clearance roof_z - floor_z; surfaces
    below the roof (roof underside seen from above, clutter) are dropped, higher ones kept."""
    L, nx, ny = Z.shape
    Z = Z.copy(); C = C.copy(); total = 0
    for t in manual:
        m = manual_cells(t, x0, y0, res, nx, ny)
        for i, j in zip(*np.nonzero(m)):
            keep = [(Z[k, i, j], C[k, i, j]) for k in range(L)
                    if np.isfinite(Z[k, i, j]) and Z[k, i, j] >= t['roof_z'] - roof_tol]
            layers = [(t['floor_z'], t['roof_z'] - t['floor_z'])] + keep
            Z[:, i, j] = np.nan; C[:, i, j] = np.nan
            for k, (zk, ck) in enumerate(layers[:L]):
                Z[k, i, j] = zk; C[k, i, j] = ck
        total += int(m.sum())
        if approach > 0:
            total += fill_approach(Z, C, t, m, x0, y0, res, approach, observed, ramp_rise)
    return Z, C, total


def fill_approach(Z, C, t, inside, x0, y0, res, max_len, observed, ramp_rise):
    """Approach floor beyond both tunnel ends that the prior map left as holes.

    Along each line parallel to the passage axis (out to max_len past the tunnel end), observed floor cells
    within ramp_rise of floor_z are anchors; unobserved cells between two anchors (the tunnel end counts as
    an anchor at floor_z) get the linearly interpolated height, so no artificial step appears. The walk
    stops at an observed surface outside ramp_rise (wall / step). Unobserved cells past the last anchor keep
    the last anchor height. Observed cells are never modified."""
    L, nx, ny = Z.shape
    if observed is None:
        observed = np.isfinite(Z[0])
    c, s = math.cos(t['yaw']), math.sin(t['yaw'])
    cx, cy = t['centre']
    half = t['size'][0] / 2
    step = res / 2
    done = set()
    for v in np.arange(-t['size'][1] / 2, t['size'][1] / 2 + 1e-9, step):
        for sg in (-1, 1):
            seq, seen = [], set()   # (u, i, j, z or None)
            u = half + step
            while u <= half + max_len:
                i, j = cell_of(cx + sg * u * c - v * s, cy + sg * u * s + v * c, x0, y0, res)
                if not (0 <= i < nx and 0 <= j < ny):
                    break
                if not inside[i, j] and (i, j) not in seen:
                    seen.add((i, j))
                    if observed[i, j]:
                        z = Z[0, i, j]
                        if not (np.isfinite(z) and abs(z - t['floor_z']) <= ramp_rise):
                            break
                        seq.append((u, i, j, float(z)))
                    else:
                        seq.append((u, i, j, None))
                u += step
            prev_u, prev_z = half, t['floor_z']
            for k, (u, i, j, z) in enumerate(seq):
                if z is not None:
                    prev_u, prev_z = u, z
                    continue
                if (i, j) in done:
                    continue
                nxt = next(((uu, zz) for uu, _, _, zz in seq[k + 1:] if zz is not None), None)
                if nxt:
                    f = (u - prev_u) / max(nxt[0] - prev_u, 1e-6)
                    zf = prev_z + f * (nxt[1] - prev_z)
                else:
                    zf = prev_z
                Z[1:, i, j] = np.nan; C[1:, i, j] = np.nan
                Z[0, i, j] = zf; C[0, i, j] = OPEN
                done.add((i, j))
    return len(done)


def manual_tunnels(manual, mask, lethal, CL, x0, y0, res, cfg):
    """zones.yaml entries for the annotated tunnels: approach poses plus what the built map says about them."""
    nx, ny = mask.shape
    out = []

    def free(x, y):
        i, j = cell_of(x, y, x0, y0, res)
        return bool(0 <= i < nx and 0 <= j < ny and mask[i, j] and not lethal[i, j])
    for t in manual:
        m = manual_cells(t, x0, y0, res, nx, ny)
        n = max(int(m.sum()), 1)
        cl = CL[m & mask]
        cl = cl[np.isfinite(cl)]
        cx, cy = t['centre']
        ax = (math.cos(t['yaw']), math.sin(t['yaw']))
        appr = t['size'][0] / 2 + cfg.approach_distance
        yaw = round(math.degrees(t['yaw']), 1)
        ends = []
        for sg in (-1, 1):
            ex, ey = cx + sg * appr * ax[0], cy + sg * appr * ax[1]
            ends.append({'x': round(ex, 3), 'y': round(ey, 3), 'yaw_deg': yaw, 'free': free(ex, ey)})
        nominal = t['roof_z'] - t['floor_z']
        out.append({
            'id': t['id'], 'source': 'manual:%s' % t['id'],
            'centre': [round(cx, 3), round(cy, 3)], 'passage_yaw_deg': yaw,
            'length': round(t['size'][0], 3), 'corridor_width': round(t['size'][1], 3),
            'area_m2': round(t['size'][0] * t['size'][1], 3),
            'floor_z': round(t['floor_z'], 3), 'roof_z': round(t['roof_z'], 3),
            'min_clearance': round(float(cl.min()) if len(cl) else nominal, 3),
            'median_clearance': round(float(np.median(cl)) if len(cl) else nominal, 3),
            'reachable_frac': round(float((m & mask).sum()) / n, 2),
            'free_frac': round(float((m & mask & ~lethal).sum()) / n, 2),
            'both_ends_open': bool(ends[0]['free'] and ends[1]['free']),
            'entry': ends[0], 'exit': ends[1],
        })
    return out


def spread_roof(CL, mask, cfg):
    """The LiDAR sees a low roof's underside in only a few floor cells; the rest of the floor under it
    reads as open sky. Give reachable 'open' cells the median low clearance of their neighbourhood when
    enough neighbours (≥ roof_min_frac) saw a roof below tunnel_max_clearance."""
    CL = CL.copy(); total = 0
    rad = cfg.roof_spread_radius
    size = 2 * rad + 1
    for _ in range(cfg.roof_spread_iters):
        low = mask & np.isfinite(CL) & (CL < cfg.tunnel_max_clearance)
        vals = np.where(low, CL, np.nan)
        frac = ndimage.uniform_filter(low.astype(float), size) / np.maximum(
            ndimage.uniform_filter(mask.astype(float), size), 1e-9)
        med = ndimage.generic_filter(vals, np.nanmedian, size=size, mode='constant', cval=np.nan) \
            if low.any() else vals
        take = mask & ~low & (frac >= cfg.roof_min_frac) & np.isfinite(med)
        if not take.any():
            break
        CL[take] = med[take]; total += int(take.sum())
    return CL, total


def reachable(Z, C, min_clear, max_step, seeds):
    """Flood fill over (layer, ix, iy). seeds: (ix, iy, z). Returns bool (L, nx, ny)."""
    L, nx, ny = Z.shape
    ok = np.isfinite(Z) & (np.nan_to_num(C, nan=0.0) >= min_clear)
    Zl = np.where(ok, Z, np.nan).tolist()
    R = np.zeros((L, nx, ny), bool)
    Rl = R.tolist()
    q = deque()
    for sx, sy, sz in seeds:
        best = None
        for rad in range(0, 8):
            for i in range(max(sx - rad, 0), min(sx + rad + 1, nx)):
                for j in range(max(sy - rad, 0), min(sy + rad + 1, ny)):
                    for k in range(L):
                        z = Zl[k][i][j]
                        if z == z and (best is None or abs(z - sz) < best[0]):
                            best = (abs(z - sz), k, i, j)
            if best is not None:
                break
        if best is not None and best[0] < 0.3:
            _, k, i, j = best
            Rl[k][i][j] = True; q.append((k, i, j))
    while q:
        k, i, j = q.popleft()
        z = Zl[k][i][j]
        for di, dj in N8:
            ii, jj = i + di, j + dj
            if ii < 0 or jj < 0 or ii >= nx or jj >= ny:
                continue
            for kk in range(L):
                zz = Zl[kk][ii][jj]
                if zz == zz and not Rl[kk][ii][jj] and abs(zz - z) <= max_step:
                    Rl[kk][ii][jj] = True; q.append((kk, ii, jj))
    return np.array(Rl, bool)


def slope_field(G, mask, res, max_step, sigma):
    """Gradient from finite differences between connected cells, smoothed. Returns gx, gy, slope_deg, rough_m."""
    Gm = np.where(mask, G, np.nan)

    def diff(axis):
        a = np.roll(Gm, -1, axis); b = np.roll(Gm, 1, axis)
        with np.errstate(all='ignore'):
            fa = np.isfinite(a) & (np.abs(a - Gm) <= max_step)
            fb = np.isfinite(b) & (np.abs(Gm - b) <= max_step)
            return np.where(fa & fb, (a - b) / (2 * res),
                            np.where(fa, (a - Gm) / res, np.where(fb, (Gm - b) / res, np.nan)))
    dx, dy = diff(0), diff(1)
    gx, gy = nconv(dx, sigma), nconv(dy, sigma)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    with np.errstate(all='ignore'):
        rough = res * np.sqrt(nconv((dx - gx) ** 2 + (dy - gy) ** 2, sigma))
    slope[~mask] = np.nan; rough[~mask] = np.nan
    return gx, gy, slope, rough


def region_axes(ix, iy):
    """Principal axes of a cell set: centre (cells), unit major u and minor v."""
    pts = np.stack([ix, iy], 1).astype(float)
    c = pts.mean(0)
    if len(pts) < 3:
        return c, np.array([1.0, 0.0]), np.array([0.0, 1.0])
    w, V = np.linalg.eigh(np.cov((pts - c).T))
    u = V[:, 1]; v = V[:, 0]
    return c, u, v


def cell_of(x, y, x0, y0, res):
    return int(math.floor((x - x0) / res)), int(math.floor((y - y0) / res))


def find_tunnels(mask, lethal, CL, x0, y0, res, cfg):
    nx, ny = mask.shape
    low = mask & np.isfinite(CL) & (CL < cfg.tunnel_max_clearance)
    lab, n = ndimage.label(low, structure=np.ones((3, 3)))
    out = []

    def free_open(x, y):
        i, j = cell_of(x, y, x0, y0, res)
        return 0 <= i < nx and 0 <= j < ny and mask[i, j] and not lethal[i, j] and CL[i, j] >= cfg.tunnel_max_clearance

    def run(cx, cy, d, limit=2.0):
        s = 0.0
        while s < limit:
            i, j = cell_of(cx + s * d[0], cy + s * d[1], x0, y0, res)
            if not (0 <= i < nx and 0 <= j < ny) or not mask[i, j] or lethal[i, j]:
                return s
            s += res / 2
        return limit

    for k in range(1, n + 1):
        ix, iy = np.nonzero(lab == k)
        area = len(ix) * res * res
        if area < cfg.tunnel_min_area:
            continue
        c, u, v = region_axes(ix, iy)
        cx, cy = x0 + (c[0] + 0.5) * res, y0 + (c[1] + 0.5) * res
        best = None
        for ax, perp in ((u, v), (v, u)):
            proj = (ix - c[0]) * ax[0] + (iy - c[1]) * ax[1]
            half = (proj.max() - proj.min()) * res / 2 + res
            ends = [free_open(cx + sg * (half + 0.15) * ax[0], cy + sg * (half + 0.15) * ax[1]) for sg in (-1, 1)]
            score = sum(ends)
            if best is None or score > best[0]:
                best = (score, ax, perp, half)
        score, ax, perp, half = best
        width = run(cx, cy, perp) + run(cx, cy, -perp)
        yaw = math.atan2(ax[1], ax[0])
        appr = half + cfg.approach_distance
        ent = (cx - appr * ax[0], cy - appr * ax[1])
        ext = (cx + appr * ax[0], cy + appr * ax[1])
        cl = CL[ix, iy]
        out.append({
            'id': 'tunnel_%d' % len(out),
            'centre': [round(cx, 3), round(cy, 3)],
            'passage_yaw_deg': round(math.degrees(yaw), 1),
            'length': round(2 * half, 3),
            'corridor_width': round(width, 3),
            'area_m2': round(area, 3),
            'min_clearance': round(float(np.nanmin(cl)), 3),
            'median_clearance': round(float(np.nanmedian(cl)), 3),
            'both_ends_open': bool(score == 2),
            'entry': {'x': round(ent[0], 3), 'y': round(ent[1], 3), 'yaw_deg': round(math.degrees(yaw), 1),
                      'free': bool(free_open(*ent))},
            'exit': {'x': round(ext[0], 3), 'y': round(ext[1], 3), 'yaw_deg': round(math.degrees(yaw), 1),
                     'free': bool(free_open(*ext))},
            'cells': [[int(a), int(b)] for a, b in zip(ix[::max(1, len(ix) // 400)], iy[::max(1, len(ix) // 400)])],
        })
    return out


def find_ramps(mask, G, gx, gy, slope, x0, y0, res, cfg):
    sl = np.where(mask, slope, np.nan)
    rm = np.isfinite(sl) & (sl >= cfg.ramp_min) & (sl <= cfg.slope_max)
    rm = ndimage.binary_opening(rm, iterations=1)
    lab, n = ndimage.label(rm, structure=np.ones((3, 3)))
    out = []
    for k in range(1, n + 1):
        ix, iy = np.nonzero(lab == k)
        area = len(ix) * res * res
        if area < cfg.ramp_min_area:
            continue
        mgx, mgy = float(np.nanmean(gx[ix, iy])), float(np.nanmean(gy[ix, iy]))
        up = math.atan2(mgy, mgx)
        ud = (math.cos(up), math.sin(up))
        z = G[ix, iy]
        lo, hi = np.nanpercentile(z, 20), np.nanpercentile(z, 80)
        foot = (ix[z <= lo].mean(), iy[z <= lo].mean()); top = (ix[z >= hi].mean(), iy[z >= hi].mean())
        proj = ix * ud[0] + iy * ud[1]; perp = -ix * ud[1] + iy * ud[0]
        s = slope[ix, iy]
        c = (x0 + (ix.mean() + 0.5) * res, y0 + (iy.mean() + 0.5) * res)
        fx, fy = x0 + (foot[0] + 0.5) * res, y0 + (foot[1] + 0.5) * res
        tx, ty = x0 + (top[0] + 0.5) * res, y0 + (top[1] + 0.5) * res
        out.append({
            'id': 'ramp_%d' % len(out),
            'centre': [round(c[0], 3), round(c[1], 3)],
            'uphill_yaw_deg': round(math.degrees(up), 1),
            'slope_median_deg': round(float(np.nanmedian(s)), 1),
            'slope_p90_deg': round(float(np.nanpercentile(s, 90)), 1),
            'rise': round(float(np.nanmax(z) - np.nanmin(z)), 3),
            'length': round(float(proj.max() - proj.min() + 1) * res, 3),
            'width': round(float(perp.max() - perp.min() + 1) * res, 3),
            'area_m2': round(area, 3),
            'foot': {'x': round(fx, 3), 'y': round(fy, 3), 'z': round(float(lo), 3)},
            'top': {'x': round(tx, 3), 'y': round(ty, 3), 'z': round(float(hi), 3)},
        })
    return out


# ----------------------------------------------------------------------------- build
def build_from_points(P, cfg, log=print):
    res = cfg.resolution
    P = P[(P[:, 2] > cfg.z_min) & (P[:, 2] < cfg.z_max)]
    lo, hi = P.min(0), P.max(0)
    x0 = math.floor(lo[0] / res) * res - 3 * res; y0 = math.floor(lo[1] / res) * res - 3 * res
    nx = int(math.ceil((hi[0] - x0) / res)) + 3; ny = int(math.ceil((hi[1] - y0) / res)) + 3
    log('points %d, grid %d x %d @ %.3f m, origin (%.2f, %.2f)' % (len(P), nx, ny, res, x0, y0))
    Z, C, cnt = column_layers(P, x0, y0, res, nx, ny, cfg.layers, cfg.layer_gap, cfg.min_points)
    log('observed cells %d, mean points/cell %.1f' % ((cnt > 0).sum(), cnt[cnt > 0].mean()))
    surfaced = np.isfinite(Z[0])   # cells with a real surface (single stray points do not count)
    Z, C, filled, roofed = fill_floor(Z, C, cfg.fill_iters, cfg.roof_lift, cfg.roof_tol)
    log('filled holes %d, floor inserted under roofs %d' % (filled, roofed))
    manual = load_manual(cfg.manual) if getattr(cfg, 'manual', None) else []
    Z, C, nman = apply_manual(Z, C, manual, x0, y0, res, approach=getattr(cfg, 'manual_approach_fill', 0.0),
                              observed=surfaced, ramp_rise=getattr(cfg, 'manual_ramp_rise', 0.15))
    if manual:
        log('manual tunnels %d, cells %d' % (len(manual), nman))

    min_clear = cfg.robot_height + cfg.clearance_margin
    seeds = [(*cell_of(s[0], s[1], x0, y0, res), s[2]) for s in (cfg.seed or [[0.0, 0.0, -0.20]])]
    R = reachable(Z, C, min_clear, cfg.max_step, seeds)
    has = R.any(0)
    Zr = np.where(R, Z, np.inf)
    k = np.argmin(Zr, 0)
    I, J = np.indices((nx, ny))
    G = np.where(has, Z[k, I, J], np.nan)
    CL = np.where(has, np.minimum(C[k, I, J], OPEN), np.nan)
    mask = has
    log('reachable %.1f m^2' % (mask.sum() * res * res))
    CL, spread = spread_roof(CL, mask, cfg)
    log('roof clearance spread to %d cells' % spread)

    gx, gy, slope, rough = slope_field(G, mask, res, cfg.max_step, cfg.fit_radius / res / 2)
    nb = neighbours(np.where(mask, G, np.inf))
    drop = np.where(mask, G - nb.min(0), 0.0)        # high side of a step edge
    lethal = ~mask | (drop > cfg.max_step) | ~np.isfinite(slope) | (slope > cfg.slope_max)
    tight = mask & (CL < cfg.robot_height + cfg.tight_margin)
    cost = np.maximum.reduce([
        cfg.slope_cost * ramp01(np.nan_to_num(slope), cfg.slope_free, cfg.slope_max),
        cfg.rough_cost * ramp01(np.nan_to_num(rough), cfg.rough_free, cfg.rough_max),
        np.where(tight, cfg.tight_cost, 0.0),
        np.where(ndimage.binary_dilation(drop > cfg.soft_step), cfg.soft_step_cost, 0.0),
    ])
    occ = np.where(lethal, 100.0, np.clip(cost, 0, 99))
    candidates = find_tunnels(mask, lethal, CL, x0, y0, res, cfg)
    tunnels = candidates
    for t in tunnels:   # mark detections that come from an annotation
        src = next((m for m in manual if math.hypot(m['centre'][0] - t['centre'][0],
                                                     m['centre'][1] - t['centre'][1]) < 0.4), None)
        t['source'] = 'manual:%s' % src['id'] if src else 'cloud'
    if manual:   # the annotated list is authoritative; auto detections are only review candidates
        tunnels = manual_tunnels(manual, mask, lethal, CL, x0, y0, res, cfg)
        log('auto tunnel candidates %d (not exported)' % len(candidates))
    ramps = find_ramps(mask & ~lethal, G, gx, gy, slope, x0, y0, res, cfg)
    log('tunnels %d, ramps %d' % (len(tunnels), len(ramps)))
    observed = ndimage.binary_dilation(cnt > 0, iterations=2)   # cells the prior map covers
    return dict(x0=x0, y0=y0, nx=nx, ny=ny, res=res, G=G, CL=CL, slope=slope, rough=rough, mask=mask,
                observed=observed,
                lethal=lethal, tight=tight, occ=occ, tunnels=tunnels, ramps=ramps, min_clear=min_clear)


# ----------------------------------------------------------------------------- output
def to_image(grid):
    return grid.T[::-1]   # row 0 = max y


def write_pgm(path, img):
    img = np.ascontiguousarray(img, np.uint8)
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())


def write_map_yaml(path, image, res, origin, mode='scale'):
    Path(path).write_text('image: %s\nmode: %s\nresolution: %.4f\norigin: [%.4f, %.4f, 0.0]\n'
                          'negate: 0\noccupied_thresh: 0.990\nfree_thresh: 0.001\n'
                          % (image, mode, res, origin[0], origin[1]))


def write_package(out, T, cfg, registration=None):
    import yaml
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    res, origin = T['res'], (T['x0'], T['y0'])
    write_pgm(out / 'nav_map.pgm', to_image(255 - np.rint(T['occ'] / 100.0 * 255)))
    write_map_yaml(out / 'nav_map.yaml', 'nav_map.pgm', res, origin)
    write_pgm(out / 'speed_mask.pgm', to_image(np.where(T['tight'] & ~T['lethal'], cfg.tight_speed, 0)))
    write_map_yaml(out / 'speed_mask.yaml', 'speed_mask.pgm', res, origin, mode='raw')
    T['G'].astype('<f4').tofile(out / 'ground_z.f32')
    T['CL'].astype('<f4').tofile(out / 'clearance.f32')
    T['slope'].astype('<f4').tofile(out / 'slope.f32')
    T['observed'].astype(np.uint8).tofile(out / 'observed.u8')
    zones = {'frame_id': 'map', 'robot_height': cfg.robot_height, 'tunnels': T['tunnels'], 'ramps': T['ramps']}
    zones = json.loads(json.dumps(zones, default=float))   # numpy scalars -> plain floats for yaml
    (out / 'zones.yaml').write_text(yaml.safe_dump(zones, sort_keys=False, default_flow_style=None, width=120))
    tight_zones = [{'x': t['centre'][0], 'y': t['centre'][1], 'min_clearance': t['min_clearance']} for t in T['tunnels']]
    meta = {'frame_id': 'map', 'resolution': res, 'origin': list(origin), 'size': [T['nx'], T['ny']],
            'layout': 'float32 little endian, index = ix * ny + iy, NaN = not reachable',
            'source': 'prior point cloud (keyframes)', 'robot_height': cfg.robot_height,
            'min_clearance': T['min_clear'], 'max_step': cfg.max_step, 'slope_max_deg': cfg.slope_max,
            'registration': registration, 'tight_zones': tight_zones,
            'args': {k: v for k, v in vars(cfg).items() if k not in ('map', 'out')}}
    (out / 'terrain.json').write_text(json.dumps(meta, indent=2))


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True, help='backend map bundle (keyframes.txt, clouds/)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--manual', help='zones_manual.yaml with hand-annotated tunnel floors (default: <map>/zones_manual.yaml if present)')
    ap.add_argument('--registration-from', help='terrain dir whose terrain.json registration (world<-map) to copy')
    ap.add_argument('--seed', type=float, nargs=3, action='append', metavar=('X', 'Y', 'Z'),
                    help='map-frame point on reachable ground (default: 0 0 -0.20, under the spawn sensor)')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--z-min', type=float, default=-1.5)
    ap.add_argument('--z-max', type=float, default=1.5)
    ap.add_argument('--layers', type=int, default=4)
    ap.add_argument('--layer-gap', type=float, default=0.06)
    ap.add_argument('--min-points', type=int, default=2)
    ap.add_argument('--fill-iters', type=int, default=3)
    ap.add_argument('--roof-lift', type=float, default=0.12, help='lowest surface this far above neighbours = roof')
    ap.add_argument('--roof-tol', type=float, default=0.06)
    ap.add_argument('--robot-height', type=float, default=0.245)
    ap.add_argument('--clearance-margin', type=float, default=-0.01,
                    help='walkable iff clearance >= height + margin (point noise ~1 cm)')
    ap.add_argument('--tight-margin', type=float, default=0.10)
    ap.add_argument('--tight-cost', type=float, default=30.0)
    ap.add_argument('--tight-speed', type=int, default=35)
    ap.add_argument('--max-step', type=float, default=0.05, help='per-cell height jump, m')
    ap.add_argument('--soft-step', type=float, default=0.02)
    ap.add_argument('--soft-step-cost', type=float, default=90.0)
    ap.add_argument('--slope-free', type=float, default=8.0)
    ap.add_argument('--slope-max', type=float, default=26.0,
                    help='lethal above, deg (the robot climbs the ~20-24 deg fortress ramps)')
    ap.add_argument('--slope-cost', type=float, default=70.0)
    ap.add_argument('--rough-free', type=float, default=0.010)
    ap.add_argument('--rough-max', type=float, default=0.040)
    ap.add_argument('--rough-cost', type=float, default=80.0)
    ap.add_argument('--fit-radius', type=float, default=0.20)
    ap.add_argument('--tunnel-max-clearance', type=float, default=0.35)
    ap.add_argument('--tunnel-min-area', type=float, default=0.10)
    ap.add_argument('--manual-approach-fill', type=float, default=2.0,
                    help='max m beyond both tunnel ends to fill unobserved floor (ramp to the first observed floor)')
    ap.add_argument('--manual-ramp-rise', type=float, default=0.15,
                    help='observed floor within this height of floor_z is ramped to; higher = wall, not joined')
    ap.add_argument('--roof-spread-radius', type=int, default=2, help='cells')
    ap.add_argument('--roof-spread-iters', type=int, default=4)
    ap.add_argument('--roof-min-frac', type=float, default=0.2)
    ap.add_argument('--approach-distance', type=float, default=0.6)
    ap.add_argument('--ramp-min', type=float, default=6.0)
    ap.add_argument('--ramp-min-area', type=float, default=0.30)
    return ap


def main(argv=None):
    cfg = build_parser().parse_args(argv)
    if not cfg.manual and (Path(cfg.map) / 'zones_manual.yaml').is_file():
        cfg.manual = str(Path(cfg.map) / 'zones_manual.yaml')
    if cfg.manual:
        print('manual annotations:', cfg.manual)
    P, used = load_keyframe_cloud(cfg.map)
    print('keyframes used %d' % used)
    T = build_from_points(P, cfg)
    reg = None
    if cfg.registration_from:
        f = Path(cfg.registration_from) / 'terrain.json'
        if f.is_file():
            reg = json.loads(f.read_text()).get('registration')
    write_package(cfg.out, T, cfg, reg)
    for t in T['tunnels']:
        print('  %s centre (%.2f, %.2f) yaw %.0f len %.2f width %.2f clear min %.3f open %s %s' % (
            t['id'], *t['centre'], t['passage_yaw_deg'], t['length'], t['corridor_width'], t['min_clearance'],
            t['both_ends_open'], t['source']))
    for r in T['ramps']:
        print('  %s centre (%.2f, %.2f) up %.0f slope %.1f (p90 %.1f) rise %.2f area %.2f' % (
            r['id'], *r['centre'], r['uphill_yaw_deg'], r['slope_median_deg'], r['slope_p90_deg'], r['rise'],
            r['area_m2']))
    print('wrote', cfg.out)


if __name__ == '__main__':
    main()
