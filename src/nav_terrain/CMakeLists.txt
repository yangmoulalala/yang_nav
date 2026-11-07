cmake_minimum_required(VERSION 3.8)
project(nav_terrain)

find_package(ament_cmake REQUIRED)

# terrain_map.py / terrain_export.py are imported by the other scripts from the same directory
install(PROGRAMS
          scripts/terrain_map.py
          scripts/cloud_to_height_map.py
          scripts/terrain_label_editor.py
          scripts/terrain_export.py
          scripts/terrain_auto_direction.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_terrain_label test/test_terrain_label.py)
  ament_add_pytest_test(test_auto_direction test/test_auto_direction.py)
endif()

ament_package()
