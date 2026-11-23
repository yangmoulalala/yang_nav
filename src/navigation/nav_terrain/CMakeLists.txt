cmake_minimum_required(VERSION 3.8)
project(nav_terrain)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
          scripts/terrain_export.py
          scripts/tunnel_zones.py
          scripts/surface_studio.py
          scripts/terrain_obstacles.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_terrain_export test/test_terrain_export.py)
  ament_add_pytest_test(test_surface_seg test/test_surface_seg.py)
  ament_add_pytest_test(test_obstacle_filter test/test_obstacle_filter.py)
endif()

ament_package()
