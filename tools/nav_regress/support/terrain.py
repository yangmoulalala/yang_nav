"""Generate terrain packages for stationary probes and closed-loop scenarios."""
import numpy as np

from nav_terrain.terrain_export import ExportConfig, export
from nav_terrain.terrain_map import OBSTACLE, SLOPE, TUNNEL, TerrainMap


def flat_fixture(out, description, speed):
    terrain = out / 'terrain'
    m = TerrainMap.blank(200, 200, .05, (-5., -5.))
    m.elevation = np.zeros(m.terrain.shape, np.float32)
    m.clearance = np.full(m.terrain.shape, .25, np.float32)
    export(m, terrain, ExportConfig(speed_limits={'flat': speed}), source=description)
    return terrain


def fixture(out):
    terrain = out / 'terrain'
    m = TerrainMap.blank(280, 240, .05, (-7., -6.))
    m.elevation = np.zeros(m.terrain.shape, np.float32)
    m.clearance = np.full(m.terrain.shape, np.inf, np.float32)
    yy, xx = np.indices(m.terrain.shape)
    x, y = -7 + (xx + .5) * .05, -6 + (yy + .5) * .05
    tunnel = (np.abs(x) < .5) & (np.abs(y) < .5)
    # Only these two openings connect the left and right halves. A planner
    # detour must not accidentally turn a traversal check into an open-field run.
    wall = (np.abs(x) < .6) & (np.abs(y) >= .5) & (np.abs(y - 2.5) >= .7)
    slope = (np.abs(x) < .6) & (np.abs(y - 2.5) < .7)
    m.terrain[tunnel], m.terrain[wall], m.terrain[slope] = TUNNEL, OBSTACLE, SLOPE
    m.clearance[tunnel] = .25
    export(m, terrain, ExportConfig(terrain_cost=0.), source='closed-loop Nav2 fixture')
    return terrain
