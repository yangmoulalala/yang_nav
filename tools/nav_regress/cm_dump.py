#!/usr/bin/env python3
"""Print a costmap snapshot (record_costmap.py output) as ASCII.

    cm_dump.py SNAPSHOT.npz [x y radius]

Without x/y the whole grid is printed at a reduced scale.  With x/y only the
window around that map point is shown, which is what you want when the shaper
reports a footprint collision at a coordinate.
"""
import sys

import numpy as np


def main():
    z = np.load(sys.argv[1])
    data = z['data']
    res = float(z['res']); ox = float(z['ox']); oy = float(z['oy'])
    w = int(z['w']); h = int(z['h'])
    img = data.reshape(h, w)
    print(f'res {res} origin ({ox:.2f},{oy:.2f}) {w}x{h}  lethal {(img==100).sum()}  unknown {(img==-1).sum()}')
    if len(sys.argv) > 3:
        px, py = float(sys.argv[2]), float(sys.argv[3])
        rad = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
        j0 = max(0, int((px - rad - ox) / res)); j1 = min(w, int((px + rad - ox) / res) + 1)
        i0 = max(0, int((py - rad - oy) / res)); i1 = min(h, int((py + rad - oy) / res) + 1)
        sub = img[i0:i1, j0:j1]
        step = max(1, sub.shape[0] // 44)
        sub = sub[::step, ::step]
        print(f'window x {px-rad:.2f}..{px+rad:.2f}  y {py-rad:.2f}..{py+rad:.2f}  step {step*res:.2f} m')
        for row in sub[::-1]:
            print(''.join('?' if v < 0 else ('.' if v == 0 else ('#' if v >= 100 else
                                                                  '+')) for v in row))
    else:
        step = max(1, h // 60)
        sub = img[::step, ::step]
        for row in sub[::-1]:
            print(''.join('?' if v < 0 else ('.' if v == 0 else ('#' if v >= 100 else
                                                                  '+')) for v in row))


if __name__ == '__main__':
    main()
