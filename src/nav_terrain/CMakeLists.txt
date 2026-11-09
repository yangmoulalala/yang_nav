cmake_minimum_required(VERSION 3.8)
project(nav_terrain)

find_package(ament_cmake REQUIRED)

# the scripts import each other from the same directory
install(PROGRAMS
          scripts/terrain_map.py
          scripts/terrain_export.py
          scripts/tunnel_zones.py
          scripts/mesh_io.py
          scripts/surface_seg.py
          scripts/surface_project.py
          scripts/surface_studio.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_terrain_export test/test_terrain_export.py)
  ament_add_pytest_test(test_surface_seg test/test_surface_seg.py)
endif()

ament_package()
