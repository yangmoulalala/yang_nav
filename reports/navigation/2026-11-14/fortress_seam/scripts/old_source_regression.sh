#!/bin/bash
set -eo pipefail
cd /home/yang/yang_nav
c++ -std=c++17 -O2 -Isrc/nav_behaviors/include -c /tmp/fortress_path_shaper_before.cpp -o /tmp/fortress_before.o
c++ build/nav_behaviors/CMakeFiles/test_zone_drive.dir/test/test_zone_drive.cpp.o /tmp/fortress_before.o -o /tmp/fortress_old_test build/nav_behaviors/gtest/libgtest_main.a build/nav_behaviors/gtest/libgtest.a build/nav_behaviors/libdirectional_zone_core.a -pthread -lyaml-cpp
set +e
/tmp/fortress_old_test --gtest_filter=Fortress.* > reports/navigation/2026-11-14/fortress_seam/old_source_regression.log 2>&1
status=$?
cat reports/navigation/2026-11-14/fortress_seam/old_source_regression.log
[ "$status" -eq 1 ]
