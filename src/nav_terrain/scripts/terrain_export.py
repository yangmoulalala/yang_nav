#!/usr/bin/env python3
"""terrain_export: labelled terrain msgpack -> Nav2 terrain package (HWSentryNav26 map_server inflation).

    terrain_export.py maps/sim/terrain.msgpack --out maps/sim/terrain

Port of HWSentryNav26 map_server/src/utils.cpp, done offline:
  cost inflation    OBSTACLE cells lethal; full cost within full_cost_radius, exp(-decay * (d - full))
                    down to cutoff_radius (physical distances)
  direction field   every directional label spreads its unit direction with the same decay; body cells
                    keep magnitude 1, the inflated ring is capped at non_body_cap (<= 0.9) so the two stay
                    separable; overlaps resolve to the higher label id, angle = magnitude-weighted mean

Output (terrain_dir for nav_bringup):
  nav_map.pgm/.yaml         Nav2 static map, scale mode. OBSTACLE = lethal (pixel 0); the inflated band is
                            capped below lethal (--max-band-cost) so Nav2's own footprint check stays the
                            only hard limit next to walls and inside 0.25 m tunnels
  speed_mask.pgm/.yaml      raw-mode speed limit percent on directional terrain (0 = no limit)
  direction_map.png         3 channels: angle code / magnitude 0..255 / label (HWSentryNav26 layout)
  terrain.png               label colour preview
  zones.yaml                one entry per connected directional region (tunnel, slope, steps ...):
                            centre, yaw (up / forward), size, entry and exit approach poses.  A label
                            region is also cut where the direction field jumps by more than
                            --direction-split-deg, so each face of a multi-direction patch (fortress
                            hexagon ring, ridge) is its own zone instead of one box spanned over the ring
  terrain.json              grid metadata and the parameters used
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage, sparse
from scipy.sparse import csgraph

sys.path.insert(0, str(Path(__file__).resolve().parent))
from terrain_map import (DIRECTIONAL, FIRST_DIRECTIONAL, LABEL_COUNT, LABELS, OBSTACLE,  # noqa: E402
                         TerrainMap, decode_angle, is_directional)

COLORS = {0: (76, 175, 80), 1: (244, 67, 54), 2: (255, 152, 0), 3: (255, 235, 59), 4: (156, 39, 176),
          5: (0, 188, 212), 6: (0, 242, 255), 7: (63, 81, 181), 8: (233, 30, 99)}


def validate_inflation(full, cutoff, decay, cap=None):
    if not (0.0 <= full <= cutoff) or not math.isfinite(cutoff):
        raise ValueError('inflation radii require 0 <= full_cost_radius <= cutoff_radius')
    if not (math.isfinite(decay) and decay >= 0.0):
        raise ValueError('decay_rate must be finite and non-negative')
    if cap is not None and not (0.0 < cap <= 0.9):
        raise ValueError('non_body_cap must be in (0, 0.9]')


def decay_profile(dist_m, full, cutoff, decay):
    """1 inside full radius, exp decay to cutoff, 0 beyond."""
    v = np.exp(-decay * np.maximum(dist_m - full, 0.0))
    return np.where(dist_m <= cutoff, v, 0.0)


def inflate_cost(obstacle, resolution, full, cutoff, decay):
    """uint8 cost (255 = obstacle) from a boolean obstacle mask, as map_utils::inflate_cost_map."""
    validate_inflation(full, cutoff, decay)
    if not obstacle.any():
        return np.zeros(obstacle.shape, np.uint8)
    dist = ndimage.distance_transform_edt(~obstacle) * resolution
    cost = np.clip(255.0 * decay_profile(dist, full, cutoff, decay), 0, 255).astype(np.uint8)
    cost[obstacle] = 255
    return cost


def inflate_direction(terrain, direction, resolution, full, cutoff, decay, cap):
    """-> (angle u8, magnitude u8, label u8, overlap cell count), as map_utils::inflate_direction_field."""
    validate_inflation(full, cutoff, decay, cap)
    h, w = terrain.shape
    r = int(math.ceil(cutoff / resolution - 1e-9))
    offs = [(dy, dx, math.hypot(dx, dy) * resolution) for dy in range(-r, r + 1) for dx in range(-r, r + 1)]
    offs = [(dy, dx, float(decay_profile(np.array(d), full, cutoff, decay))) for dy, dx, d in offs
            if d <= cutoff]
    ang_src = decode_angle(direction)
    angle = np.zeros((h, w), np.uint8)
    mag = np.zeros((h, w), np.uint8)
    label = terrain.copy()
    support = np.zeros((h, w), np.uint8)
    won = np.zeros((h, w), np.uint8)
    for lab in range(FIRST_DIRECTIONAL, LABEL_COUNT):
        src = terrain == lab
        if not src.any():
            continue
        vx = np.where(src, np.cos(ang_src), 0.0)
        vy = np.where(src, np.sin(ang_src), 0.0)
        sx = np.zeros((h + 2 * r, w + 2 * r))
        sy = np.zeros_like(sx)
        mx = np.zeros_like(sx)
        srcf = src.astype(float)
        for dy, dx, m in offs:
            ys, xs = slice(r + dy, r + dy + h), slice(r + dx, r + dx + w)
            sx[ys, xs] += vx * m
            sy[ys, xs] += vy * m
            np.maximum(mx[ys, xs], srcf * m, out=mx[ys, xs])
        sx, sy, mx = sx[r:r + h, r:r + w], sy[r:r + h, r:r + w], mx[r:r + h, r:r + w]
        enc = np.clip(np.minimum(mx, cap) * 255.0, 0, 255).astype(np.uint8)
        has = enc > 0
        support[has] |= np.uint8(1 << (lab - FIRST_DIRECTIONAL))
        take = has & (terrain != OBSTACLE) & ~np.isin(terrain, list(DIRECTIONAL)) & (lab > won)
        a = np.arctan2(sy, sx)
        a = np.where(a < 0, a + 2 * math.pi, a)
        angle[take] = (a[take] / (2 * math.pi) * 255.0).astype(np.uint8)
        mag[take] = enc[take]
        label[take] = lab
        won[take] = lab
    body = np.isin(terrain, list(DIRECTIONAL))
    angle[body], mag[body], label[body] = direction[body], 255, terrain[body]
    obs = terrain == OBSTACLE
    angle[obs] = mag[obs] = 0
    label[obs] = OBSTACLE
    overlap = int(np.count_nonzero(support & (support - 1)))
    return angle, mag, label, overlap


# ----------------------------------------------------------------------------- zones
def direction_components(m, lab, split_deg):
    """8-connected components of one label, also cut where adjacent cells point further than split_deg
    apart.  A fortress ring is one label component but six faces: without the cut its zone box spans the
    whole ring and its axis is the meaningless mean of the six uphill directions."""
    if not (0.0 < split_deg <= 180.0):
        raise ValueError('direction split must be in (0, 180] degrees')
    mask = m.terrain == lab
    rows, cols = np.nonzero(mask)
    n = rows.size
    out = np.zeros(mask.shape, np.int32)
    if n == 0:
        return out, 0
    index = np.full(mask.shape, -1, np.int64)
    index[rows, cols] = np.arange(n)
    ang = decode_angle(m.direction[rows, cols])
    jump = math.radians(split_deg)
    h, w = mask.shape
    src, dst = [], []
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):        # 8-neighbourhood, each pair once
        r0, r1 = max(0, -dy), h - max(0, dy)
        c0, c1 = max(0, -dx), w - max(0, dx)
        a = index[r0:r1, c0:c1]
        b = index[r0 + dy:r1 + dy, c0 + dx:c1 + dx]
        ok = (a >= 0) & (b >= 0)
        if not ok.any():
            continue
        d = np.abs((ang[a[ok]] - ang[b[ok]] + math.pi) % (2 * math.pi) - math.pi)
        keep = d <= jump
        src.append(a[ok][keep])
        dst.append(b[ok][keep])
    if src and sum(s.size for s in src):
        s, t = np.concatenate(src), np.concatenate(dst)
        graph = sparse.coo_matrix((np.ones(s.size, np.int8), (s, t)), shape=(n, n))
        ncomp, comp = csgraph.connected_components(graph, directed=False)
    else:
        ncomp, comp = n, np.arange(n)
    out[rows, cols] = comp + 1
    return out, int(ncomp)


def region_zones(m, approach_dist, split_deg=20.0):
    """One zone per connected direction region of each directional label.

    Regions are 8-connected components of the label, cut where the direction field jumps by more than
    split_deg, so a multi-direction patch (fortress hexagon ring, ridge) yields one zone per face."""
    zones = []
    res = m.resolution
    for lab in sorted(DIRECTIONAL):
        comp, n = direction_components(m, lab, split_deg)
        for k in range(1, n + 1):
            rows, cols = np.nonzero(comp == k)
            codes = m.direction[rows, cols].astype(float) / 255.0 * 2 * math.pi
            yaw = math.atan2(np.sin(codes).mean(), np.cos(codes).mean())
            spread = 1.0 - math.hypot(np.sin(codes).mean(), np.cos(codes).mean())
            x = m.origin[0] + (cols + 0.5) * res
            y = m.origin[1] + (rows + 0.5) * res
            cx, cy = float(x.mean()), float(y.mean())
            u = (x - cx) * math.cos(yaw) + (y - cy) * math.sin(yaw)      # along direction
            v = -(x - cx) * math.sin(yaw) + (y - cy) * math.cos(yaw)     # across
            length = float(u.max() - u.min() + res)
            width = float(v.max() - v.min() + res)
            uc = float((u.max() + u.min()) / 2)
            vc = float((v.max() + v.min()) / 2)
            ccx = cx + uc * math.cos(yaw) - vc * math.sin(yaw)
            ccy = cy + uc * math.sin(yaw) + vc * math.cos(yaw)
            half = length / 2 + approach_dist
            zone = {
                'id': '%s_%d' % (LABELS[lab], k), 'type': LABELS[lab],
                'centre': [round(ccx, 3), round(ccy, 3)], 'yaw_deg': round(math.degrees(yaw), 1),
                'size': [round(length, 3), round(width, 3)], 'cells': int(rows.size),
                # Traversal policy is data, not a terrain-specific branch in the Nav2 plugins.
                # Tunnel/undulation axes are bidirectional; slopes and steps face their annotated +axis.
                'one_way_dir': 0 if LABELS[lab] in ('tunnel', 'undulation') else 1,
                'allow_flip': LABELS[lab] in ('tunnel', 'undulation'),
                'recentre': LABELS[lab] == 'tunnel',
                'entry': [round(ccx - half * math.cos(yaw), 3), round(ccy - half * math.sin(yaw), 3),
                          round(math.degrees(yaw), 1)],
                'exit': [round(ccx + half * math.cos(yaw), 3), round(ccy + half * math.sin(yaw), 3),
                         round(math.degrees(yaw), 1)],
            }
            if m.elevation is not None:
                z = m.elevation[rows, cols]
                z = z[np.isfinite(z)]
                if z.size:
                    zone['z_range'] = [round(float(z.min()), 3), round(float(z.max()), 3)]
            if spread > 0.05:
                zone['warning'] = 'direction varies inside the region (spread %.2f)' % spread
            zones.append(zone)
    return zones


# ----------------------------------------------------------------------------- writers
def write_pgm(path, img_rows_up):
    """Grid rows go up in y; image rows go down."""
    img = np.ascontiguousarray(img_rows_up[::-1], np.uint8)
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())


def write_map_yaml(path, image, res, origin, mode):
    Path(path).write_text('image: %s\nmode: %s\nresolution: %.4f\norigin: [%.4f, %.4f, 0.0]\n'
                          'negate: 0\noccupied_thresh: 0.990\nfree_thresh: 0.001\n'
                          % (image, mode, res, origin[0], origin[1]))


def write_png(path, rgb_rows_up):
    import cv2
    cv2.imwrite(str(path), np.ascontiguousarray(rgb_rows_up[::-1, :, ::-1]))


def export(m, out, cfg):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    m.save(out / 'terrain.msgpack')
    if m.elevation is not None:
        ground = m.elevation.copy()
        ground[m.terrain == OBSTACLE] = np.nan
        np.savez_compressed(out / 'ground_z.npz', ground_z=ground,
                            clearance=(m.clearance if m.clearance is not None
                                       else np.full(ground.shape, np.inf, np.float32)),
                            resolution=m.resolution, origin=np.asarray(m.origin))
    res = m.resolution
    obstacle = m.terrain == OBSTACLE
    cost = inflate_cost(obstacle, res, cfg.full_cost_radius, cfg.cutoff_radius, cfg.decay_rate)
    # Nav2 cost 0..100 percent: obstacle = 100 (lethal); band scaled to <= max_band_cost
    occ = np.where(obstacle, 100.0, cost.astype(float) / 255.0 * cfg.max_band_cost)
    ang, mag, lab, overlap = inflate_direction(m.terrain, m.direction, res, cfg.full_cost_radius,
                                               cfg.cutoff_radius, cfg.decay_rate, cfg.non_body_cap)
    body = np.isin(m.terrain, list(DIRECTIONAL))
    occ = np.where(body & ~obstacle, np.maximum(occ, cfg.terrain_cost), occ)
    write_pgm(out / 'nav_map.pgm', np.rint(255 - occ / 100.0 * 255).astype(np.uint8))
    write_map_yaml(out / 'nav_map.yaml', 'nav_map.pgm', res, m.origin, 'scale')
    speed = np.zeros(m.terrain.shape, np.uint8)
    for name, pct in cfg.speed_limits.items():
        speed[m.terrain == LABEL_IDS_LOCAL[name]] = pct
    write_pgm(out / 'speed_mask.pgm', speed)
    write_map_yaml(out / 'speed_mask.yaml', 'speed_mask.pgm', res, m.origin, 'raw')
    write_png(out / 'direction_map.png', np.dstack([ang, mag, lab]))
    preview = np.zeros(m.terrain.shape + (3,), np.uint8)
    for k, c in COLORS.items():
        preview[m.terrain == k] = c
    write_png(out / 'terrain.png', preview)
    zones = region_zones(m, cfg.approach_dist, cfg.direction_split_deg)
    import yaml
    (out / 'zones.yaml').write_text('# generated by terrain_export.py from %s\n' % cfg.src
                                    + yaml.safe_dump({'zones': zones}, sort_keys=False,
                                                     default_flow_style=None, width=120))
    meta = {
        'source': str(cfg.src), 'resolution': res, 'origin': list(m.origin), 'size': [m.width, m.height],
        'labels': {str(k): v for k, v in LABELS.items()},
        'label_cells': {LABELS[k]: int((m.terrain == k).sum()) for k in LABELS},
        'inflation': {'full_cost_radius': cfg.full_cost_radius, 'cutoff_radius': cfg.cutoff_radius,
                      'decay_rate': cfg.decay_rate, 'non_body_cap': cfg.non_body_cap,
                      'max_band_cost': cfg.max_band_cost, 'terrain_cost': cfg.terrain_cost},
        'speed_limits': cfg.speed_limits, 'direction_overlap_cells': overlap,
        # map -> field registration from surface_studio (tools/nav_regress/goal.py converts world goals with it)
        'registration': m.extra.get('registration'),
    }
    (out / 'terrain.json').write_text(json.dumps(meta, indent=2))
    return meta, zones


LABEL_IDS_LOCAL = {v: k for k, v in LABELS.items()}


def parse_speed(items):
    out = {}
    for it in items or []:
        name, pct = it.split('=')
        if name not in LABEL_IDS_LOCAL or not is_directional(LABEL_IDS_LOCAL[name]):
            raise argparse.ArgumentTypeError('unknown directional label %r' % name)
        out[name] = int(pct)
    return out


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map', help='terrain msgpack')
    ap.add_argument('--out', required=True, help='terrain_dir to write')
    ap.add_argument('--full-cost-radius', type=float, default=0.10)
    ap.add_argument('--cutoff-radius', type=float, default=0.30)
    ap.add_argument('--decay-rate', type=float, default=24.0, help='1/m')
    ap.add_argument('--non-body-cap', type=float, default=0.9)
    ap.add_argument('--max-band-cost', type=float, default=60.0,
                    help='Nav2 cost percent of the inflated band next to obstacles (100 = lethal)')
    ap.add_argument('--terrain-cost', type=float, default=10.0,
                    help='Nav2 cost percent on directional terrain bodies (prefer flat ground)')
    ap.add_argument('--speed', action='append', metavar='LABEL=PCT',
                    help='speed limit percent on a directional label, default tunnel=35 slope=60')
    ap.add_argument('--approach-dist', type=float, default=0.6, help='entry/exit pose distance beyond a zone')
    ap.add_argument('--direction-split-deg', type=float, default=20.0,
                    help='cut a label region where adjacent cells point further apart (multi-direction slope: '
                         'one zone per face). 180 disables the cut')
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)
    a.speed_limits = parse_speed(a.speed) if a.speed else {'tunnel': 35, 'slope': 60}
    a.src = Path(a.map)
    if not (0.0 <= a.max_band_cost < 100.0):
        sys.exit('--max-band-cost must be in [0, 100)')
    m = TerrainMap.load(a.map)
    meta, zones = export(m, a.out, a)
    print('%s -> %s: %dx%d @ %.3f m, %d zones, direction overlap %d cells'
          % (a.map, a.out, m.width, m.height, m.resolution, len(zones), meta['direction_overlap_cells']))
    for k, v in meta['label_cells'].items():
        if v:
            print('  %-10s %d cells' % (k, v))


if __name__ == '__main__':
    main()
