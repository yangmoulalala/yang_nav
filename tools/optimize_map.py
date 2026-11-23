#!/usr/bin/env python3
"""Run the package-owned map optimizer from a source checkout."""
from pathlib import Path
import runpy

if __name__ == '__main__':
    command = Path(__file__).resolve().parents[1] / 'src/mapping/offline_mapping_optimizer/scripts/optimize_map.py'
    runpy.run_path(str(command), run_name='__main__')
