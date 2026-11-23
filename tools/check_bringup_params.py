#!/usr/bin/env python3
"""Run the localization package's source-tree configuration check."""
from pathlib import Path
import runpy

if __name__ == '__main__':
    check = Path(__file__).resolve().parents[1] / 'src/localization/bringup/test/check_bringup_params.py'
    runpy.run_path(str(check), run_name='__main__')
