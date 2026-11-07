#!/usr/bin/env python3
"""Compare a cloud-built terrain package against a reference (mesh-built) one in the same map frame.

    compare_terrain.py NEW_DIR REF_DIR [--slope-ref-max 18] [--slope-new-max 26]

Traversable = nav_map pixel >= 3 (not lethal). Agreement is counted over cells traversable in either map,
raw and with 1-cell tolerance (a disagreement is forgiven if the other map has the same class within
one cell). Cells whose new-map slope lies between the two slope limits are reported separately (the
reference marks them lethal by design).
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy import ndimage


def load(d):
    d = Path(d)
    m = json.loads((d / 'terrain.json').read_text())
    nx, ny = m['size']
    raw = (d / 'nav_map.pgm').read_bytes()
    hdr = raw.split(b'\n', 3)
    w, h = map(int, hdr[1].split())
    img = np.frombuffer(raw[-w * h:], np.uint8).reshape(h, w)[::-1, :].T[:nx, :ny]
    g = np.fromfile(d / 'ground_z.f32', '<f4').reshape(nx, ny)
    c = np.fromfile(d / 'clearance.f32', '<f4').reshape(nx, ny)
    return m, img, g, c


def sample(m, arr, X, Y, fill):
    r = m['resolution']; ox, oy = m['origin']; nx, ny = m['size']
    ix = np.floor((X - ox) / r).astype(int); iy = np.floor((Y - oy) / r).astype(int)
    ok = (ix >= 0) & (iy >= 0) & (ix < nx) & (iy < ny)
    out = np.full(X.shape, fill, dtype=arr.dtype)
    out[ok] = arr[ix[ok], iy[ok]]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('new'); ap.add_argument('ref')
    ap.add_argument('--slope-ref-max', type=float, default=18.0)
    ap.add_argument('--slope-new-max', type=float, default=26.0)
    a = ap.parse_args()
    mn, imn, gn, cn = load(a.new)
    mr, imr, gr, cr = load(a.ref)
    r = mn['resolution']; ox, oy = mn['origin']; nx, ny = mn['size']
    I, J = np.indices((nx, ny))
    X, Y = ox + (I + 0.5) * r, oy + (J + 0.5) * r
    tn = imn >= 3
    tr = sample(mr, imr, X, Y, 0) >= 3
    sn = np.fromfile(Path(a.new) / 'slope.f32', '<f4').reshape(nx, ny) if (Path(a.new) / 'slope.f32').is_file() else np.zeros((nx, ny))
    steep = np.isfinite(sn) & (sn > a.slope_ref_max - 1) & (sn <= a.slope_new_max)
    dil = np.ones((3, 3), bool)
    tn_near = ndimage.binary_dilation(tn, dil); tr_near = ndimage.binary_dilation(tr, dil)
    ln_near = ndimage.binary_dilation(~tn, dil); lr_near = ndimage.binary_dilation(~tr, dil)
    obs_f = Path(a.new) / 'observed.u8'
    observed = np.fromfile(obs_f, np.uint8).reshape(nx, ny).astype(bool) if obs_f.is_file() else np.ones((nx, ny), bool)
    unobs = (tn | tr) & ~observed
    print('cells the prior map never observed (excluded): %.1f m^2' % (unobs.sum() * r * r))
    roi = (tn | tr) & ~steep & observed
    agree = (tn == tr) & roi
    tol_ok = agree | (roi & tn & ~tr & tr_near) | (roi & ~tn & tr & tn_near)
    area = r * r
    print('cells in ROI %d (%.1f m^2), steep cells excluded %d (%.1f m^2)' % (roi.sum(), roi.sum() * area, steep.sum(), steep.sum() * area))
    print('agreement raw %.2f %%, with 1-cell tolerance %.2f %%' % (100 * agree.sum() / roi.sum(), 100 * tol_ok.sum() / roi.sum()))
    fn = roi & ~tn & tr & ~tn_near
    fp = roi & tn & ~tr & ~tr_near
    print('beyond tolerance: new lethal / ref free %.2f m^2, new free / ref lethal %.2f m^2' % (fn.sum() * area, fp.sum() * area))
    for name, msk in (('new-lethal/ref-free', fn), ('new-free/ref-lethal', fp)):
        lab, n = ndimage.label(msk, structure=np.ones((3, 3)))
        sizes = ndimage.sum(msk, lab, range(1, n + 1))
        order = np.argsort(sizes)[::-1][:6]
        for k in order:
            ix, iy = np.nonzero(lab == k + 1)
            print('   %s blob %.2f m^2 at (%.2f, %.2f)' % (name, sizes[k] * area, ox + (ix.mean() + 0.5) * r, oy + (iy.mean() + 0.5) * r))
    both = tn & tr & np.isfinite(gn)
    gref = sample(mr, gr, X, Y, np.nan)
    dz = (gn - gref)[both & np.isfinite(gref)]
    print('ground z diff on common free cells: median %.3f m, p90 |dz| %.3f m' % (np.median(dz), np.percentile(np.abs(dz), 90)))
    zones = Path(a.new) / 'zones.yaml'
    ref_tight = [z for z in mr.get('tight_zones', []) if z.get('min_clearance', 1) < 0.30 and z.get('cells', 0) >= 300]
    if zones.is_file():
        import yaml
        Z = yaml.safe_load(zones.read_text())
        print('reference low-clearance zones (<0.30 m, >=300 cells @2.5 cm): %d' % len(ref_tight))
        for z in ref_tight:
            cx, cy = (z['x'][0] + z['x'][1]) / 2, (z['y'][0] + z['y'][1]) / 2
            best = min(Z['tunnels'], key=lambda t: math.hypot(t['centre'][0] - cx, t['centre'][1] - cy), default=None)
            d = math.hypot(best['centre'][0] - cx, best['centre'][1] - cy) if best else float('nan')
            print('   ref (%.2f, %.2f) clear %.3f -> nearest new %s at %.2f m (clear %.3f, open %s)' % (
                cx, cy, z['min_clearance'], best['id'] if best else None, d,
                best['min_clearance'] if best else float('nan'), best['both_ends_open'] if best else None))


if __name__ == '__main__':
    main()
