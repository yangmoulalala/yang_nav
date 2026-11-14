#!/usr/bin/env python3
"""Check navigation terrain data before running the stack; never modify annotations.

This checks data completeness and consistency, not absolute height accuracy or
physical traversability. Use plan_tunnels and physical tests for those contracts.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS / 'src/nav_terrain/scripts'))
from terrain_map import TerrainMap, OBSTACLE, TUNNEL


def check(directory, robot_height=.245):
    directory = Path(directory).resolve()
    errors, warnings = [], []
    report = dict(directory=str(directory), errors=errors, warnings=warnings)
    for name in ('nav_map.yaml', 'speed_mask.yaml', 'zones.yaml', 'terrain.msgpack'):
        if not (directory / name).is_file():
            errors.append('missing ' + name)
    if errors:
        return report
    terrain = TerrainMap.load(directory / 'terrain.msgpack')
    terrain.validate()
    free = terrain.terrain != OBSTACLE
    tunnel = terrain.terrain == TUNNEL
    report.update(size=[terrain.width, terrain.height], resolution=terrain.resolution,
                  traversable_cells=int(free.sum()), tunnel_cells=int(tunnel.sum()))
    if not free.any():
        errors.append('no traversable cells')
    if terrain.elevation is None:
        errors.append('missing elevation; ground-relative obstacle filtering cannot run')
    else:
        missing = int((free & ~np.isfinite(terrain.elevation)).sum())
        report['missing_traversable_ground_cells'] = missing
        if missing:
            errors.append(f'{missing} traversable cells have no finite ground height')
    if tunnel.any():
        if terrain.clearance is None:
            errors.append('missing clearance for tunnel-labelled terrain; verify ceiling source data and regenerate the export')
        else:
            clear = terrain.clearance[tunnel]
            report['finite_tunnel_clearance_cells'] = int(np.isfinite(clear).sum())
            if np.isnan(clear).any() or (clear <= 0).any():
                errors.append('invalid tunnel clearance values')
            if (clear < robot_height).any():
                errors.append('some tunnel clearance values are below robot height')
            if np.isinf(clear).any():
                warnings.append('some tunnel-labelled cells have open-sky clearance; verify the ceiling data')
    ground_file = directory / 'ground_z.npz'
    if ground_file.is_file() and terrain.elevation is not None:
        with np.load(ground_file) as data:
            grid = data['ground_z']
            if grid.shape != terrain.terrain.shape or not np.allclose(
                    grid[free], terrain.elevation[free], equal_nan=True):
                errors.append('ground_z.npz disagrees with terrain.msgpack')
    # Check the saved manual project without regenerating or replacing anything.
    projects = sorted(directory.parent.glob('*.surf.npz'))
    if len(projects) == 1:
        with np.load(projects[0], allow_pickle=False) as data:
            if data['z'].shape == terrain.terrain.shape and terrain.elevation is not None:
                report['source_project'] = str(projects[0])
                report['elevation_matches_source_project'] = bool(np.allclose(
                    data['z'], terrain.elevation, equal_nan=True))
                report['source_finite_tunnel_clearance_cells'] = int(
                    np.isfinite(data['clearance'][tunnel]).sum())
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('terrain_dir')
    ap.add_argument('--robot-height', type=float, default=.245)
    args = ap.parse_args()
    try:
        result = check(args.terrain_dir, args.robot_height)
    except (OSError, ValueError, KeyError) as exc:
        result = dict(errors=[str(exc)], warnings=[])
    result['passed'] = not result['errors']
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
