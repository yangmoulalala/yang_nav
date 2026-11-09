#!/usr/bin/env python3
"""tunnel_zones: the tunnels navigation drives through, read from a terrain package's zones.yaml.

    tunnel_zones.py maps/sim/zones.yaml          print the tunnels kept and the fragments dropped

A zone of type tunnel (terrain_export.region_zones) carries
  centre   [x, y] map frame
  yaw_deg  passing direction (the robot drives along it, either way: tunnels are bidirectional)
  size     [depth along yaw, opening width across yaw]
Fragments smaller than min_cells (label noise in the prior cloud) are dropped with a warning.
The axis is used analytically (centre + yaw), never from the grid, so the map resolution does not
move the line the robot follows.
"""
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

MIN_CELLS = 50


@dataclass
class Tunnel:
    id: str
    cx: float
    cy: float
    yaw: float        # rad, passing direction
    depth: float      # m along yaw
    width: float      # m across yaw (opening)
    cells: int

    def to_local(self, x, y):
        """-> (u along the axis, v across it) relative to the centre."""
        dx, dy = x - self.cx, y - self.cy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def to_map(self, u, v):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.cx + c * u - s * v, self.cy + s * u + c * v


def load(path, min_cells=MIN_CELLS, log=print):
    zones = yaml.safe_load(Path(path).read_text()).get('zones') or []
    out = []
    for z in zones:
        if z.get('type') != 'tunnel':
            continue
        if int(z.get('cells', 0)) < min_cells:
            log('tunnel_zones: dropping %s (%d cells < %d, label fragment)' % (z['id'], z.get('cells', 0), min_cells))
            continue
        depth, width = (float(v) for v in z['size'])
        out.append(Tunnel(z['id'], float(z['centre'][0]), float(z['centre'][1]), math.radians(float(z['yaw_deg'])),
                          depth, width, int(z['cells'])))
    return out


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    for t in load(argv[0] if argv else 'zones.yaml'):
        print('%-10s centre (%.3f, %.3f) yaw %6.1f deg depth %.3f width %.3f cells %d'
              % (t.id, t.cx, t.cy, math.degrees(t.yaw), t.depth, t.width, t.cells))


if __name__ == '__main__':
    main()
