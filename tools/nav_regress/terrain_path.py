"""Locate the nav_terrain package (terrain_export.py output) belonging to a map directory.

A run directory keeps the localization prior map (keyframes.txt, clouds/) and may hold the terrain
package either beside it (`maps/sim`) or in its own export folder (`maps/sim/terrain`).
``resolve_terrain_dir`` returns whichever one actually contains nav_map.yaml.

    terrain_path.py [MAP_DIR]        print the terrain package directory (default maps/sim)
"""
import sys
from pathlib import Path


def resolve_terrain_dir(map_dir):
    """`maps/sim` -> `maps/sim/terrain` when the terrain package was exported into a subfolder."""
    d = Path(map_dir).expanduser()
    return d / 'terrain' if (d / 'terrain' / 'nav_map.yaml').is_file() else d


def default_map_dir():
    return Path.home() / 'yang_nav/maps/sim'


def default_terrain_dir():
    return resolve_terrain_dir(default_map_dir())


if __name__ == '__main__':
    print(resolve_terrain_dir(sys.argv[1] if len(sys.argv) > 1 else default_map_dir()))
