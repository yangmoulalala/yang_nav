#!/usr/bin/env python3
import os
from pathlib import Path
import sys


def main():
    python = Path.home() / '.venvs' / 'nav-terrain' / 'bin' / 'python'
    if sys.prefix == sys.base_prefix and python.is_file() and os.access(python, os.X_OK):
        os.execv(str(python), [str(python), __file__, *sys.argv[1:]])
    from nav_terrain.surface_studio import main as studio_main
    return studio_main()


if __name__ == "__main__":
    raise SystemExit(main())
