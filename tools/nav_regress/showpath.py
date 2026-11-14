#!/usr/bin/env python3
"""Draw a planned path from /tmp/chain_paths.txt on nav_map (ASCII).

    showpath.py "tunnel_5 fwd" [X0 X1 Y0 Y1] [--step N]

The path file is written by tools/nav_regress/chaincheck.py: blocks start with `# <tunnel> <dir>`,
one `x y` per line.  `o` marks the path, `T` a tunnel centre, `#` lethal, `+` inflated band.
"""
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/nav_regress'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/nav_terrain/scripts'))
import terrain_path as tp  # noqa: E402
import tunnel_accept as ta  # noqa: E402
import tunnel_zones as tz  # noqa: E402

CHARS = {0: '#', 255: '.', 254: ',', 253: ':', 251: ';', 241: '-', 230: '+', 209: '*', 205: 'o', 199: 'x',
         102: '~'}


def main():
    args = [a for a in sys.argv[1:]]
    step = 1
    if '--step' in args:
        i = args.index('--step')
        step = int(args[i + 1])
        del args[i:i + 2]
    want = args[0]
    nums = [float(v) for v in args[1:]]

    blocks, cur = {}, None
    for line in Path('/tmp/chain_paths.txt').read_text().splitlines():
        if line.startswith('#'):
            cur = line[2:].strip()
            blocks[cur] = []
        elif cur is not None:
            x, y = (float(v) for v in line.split())
            blocks[cur].append((x, y))
    pts = blocks[want]

    d = tp.default_terrain_dir()
    img, res, origin = ta.read_map(d)
    tun = tz.load(d / 'zones.yaml', log=lambda m: None)
    x0, x1, y0, y1 = nums if len(nums) == 4 else (min(p[0] for p in pts) - 0.5,
                                                  max(p[0] for p in pts) + 0.5,
                                                  min(p[1] for p in pts) - 0.5,
                                                  max(p[1] for p in pts) + 0.5)
    c0, c1 = int((x0 - origin[0]) / res), int((x1 - origin[0]) / res)
    r0, r1 = int((y0 - origin[1]) / res), int((y1 - origin[1]) / res)
    grid = [[CHARS.get(int(img[r, c]), '?') for c in range(c0, c1 + 1, step)]
            for r in range(r0, r1 + 1, step)]
    for t in tun:
        c, r = int((t.cx - origin[0]) / res) - c0, int((t.cy - origin[1]) / res) - r0
        if 0 <= r // step < len(grid) and 0 <= c // step < len(grid[0]):
            grid[r // step][c // step] = 'T'
    for x, y in pts:
        c, r = int((x - origin[0]) / res) - c0, int((y - origin[1]) / res) - r0
        if 0 <= r // step < len(grid) and 0 <= c // step < len(grid[0]):
            grid[r // step][c // step] = 'o'
    for i in range(len(grid) - 1, -1, -1):
        print('%6.2f %s' % (origin[1] + (r0 + i * step + 0.5) * res, ''.join(grid[i])))
    print('       ' + ''.join(
        str(int(round(origin[0] + (c0 + j * step) * res)) % 10)
        if abs((origin[0] + (c0 + j * step) * res) - round(origin[0] + (c0 + j * step) * res)) < step * res / 2
        else ' ' for j in range(len(grid[0]))))
    print('x %.2f..%.2f  y %.2f..%.2f, 1 char = %.2f m  (%s: %d pts, %s -> %s)'
          % (x0, x1, y0, y1, step * res, want, len(pts),
             '(%.2f,%.2f)' % pts[0], '(%.2f,%.2f)' % pts[-1]))


if __name__ == '__main__':
    main()
