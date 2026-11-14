#!/bin/bash
set -e
cd /home/yang/yang_nav
report=reports/navigation/2026-11-13/manual_four_goals
build_dir=$(mktemp -d /tmp/nav_exit_suffix.XXXXXX)
g++ -std=c++17 -O2 -Isrc/nav_behaviors/include "$report/scripts/exit_suffix.cpp" src/nav_behaviors/src/zone_tracker.cpp src/nav_behaviors/src/path_shaper.cpp src/nav_behaviors/src/zone_state.cpp -o "$build_dir/exit_suffix"
"$build_dir/exit_suffix" | tee "$report/exit_suffix_result.txt"
