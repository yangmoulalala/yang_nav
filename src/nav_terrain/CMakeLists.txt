cmake_minimum_required(VERSION 3.8)
project(nav_terrain)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/terrain_builder.py scripts/compare_terrain.py scripts/terrain_viewer.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_terrain_builder test/test_terrain_builder.py)
  ament_add_pytest_test(test_manual_zones test/test_manual_zones.py)
  ament_add_pytest_test(test_manual_approach test/test_manual_approach.py)
endif()

ament_package()
