cmake_minimum_required(VERSION 3.8)
project(nav_behaviors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(timed_obstacle_layer SHARED src/timed_obstacle_layer.cpp)
target_include_directories(timed_obstacle_layer PRIVATE include)
ament_target_dependencies(timed_obstacle_layer nav2_costmap_2d rclcpp pluginlib sensor_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

# Reusable zone geometry, policy parsing, tracking and phase state; no ROS dependencies.
add_library(directional_zone_core STATIC
  src/path_shaper.cpp
  src/zone_tracker.cpp
  src/zones_yaml.cpp
  src/recovery_policy.cpp
  src/zone_state.cpp)
target_include_directories(directional_zone_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(directional_zone_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(directional_zone_core ${YAML_CPP_LIBRARIES})

add_library(start_aware_planner SHARED src/start_aware_planner.cpp)
ament_target_dependencies(start_aware_planner rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util nav_msgs pluginlib)
target_include_directories(start_aware_planner PRIVATE include)

add_library(zone_path_shaper SHARED src/zone_path_shaper.cpp)
target_link_libraries(zone_path_shaper directional_zone_core ${YAML_CPP_LIBRARIES})
ament_target_dependencies(zone_path_shaper rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util nav_msgs
  pluginlib)

add_library(zone_aware_controller SHARED src/zone_aware_controller.cpp)
target_link_libraries(zone_aware_controller directional_zone_core)
ament_target_dependencies(zone_aware_controller rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util nav_msgs
  pluginlib std_msgs visualization_msgs)

# BT node implementations (no registration): shared by the plugin library and the tests.
add_library(zone_bt_core STATIC
  src/zone_phase_monitor.cpp
  src/zone_recovery.cpp
  src/zone_replan_paused.cpp)
target_link_libraries(zone_bt_core directional_zone_core)
set_target_properties(zone_bt_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(zone_bt_core rclcpp std_msgs behaviortree_cpp tf2_ros)

# BT nodes loaded by bt_navigator (plugin_lib_names): the path is kept inside a zone stretch, and
# recovery escalates without moving the robot inside one.
add_library(zone_bt_nodes SHARED src/zone_bt_nodes.cpp)
target_link_libraries(zone_bt_nodes zone_bt_core directional_zone_core)
# without this the BT_REGISTER_NODES macro expands to `static` and the library exports no plugin symbol
target_compile_definitions(zone_bt_nodes PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(zone_bt_nodes rclcpp std_msgs nav_msgs behaviortree_cpp tf2_ros tf2_geometry_msgs)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS timed_obstacle_layer start_aware_planner zone_path_shaper zone_aware_controller zone_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_timed_obstacle_layer test/test_timed_obstacle_layer.cpp)
  target_include_directories(test_timed_obstacle_layer PRIVATE include)
  target_link_libraries(test_timed_obstacle_layer timed_obstacle_layer)
  ament_target_dependencies(test_timed_obstacle_layer nav2_costmap_2d nav2_util sensor_msgs tf2_ros)
  ament_add_gtest(test_costmap_collision test/test_costmap_collision.cpp)
  target_link_libraries(test_costmap_collision directional_zone_core)
  ament_target_dependencies(test_costmap_collision nav2_costmap_2d)
  ament_add_gtest(test_path_shaper test/test_path_shaper.cpp)
  target_link_libraries(test_path_shaper directional_zone_core)
  ament_add_gtest(test_zone_path_handoff test/test_zone_path_handoff.cpp)
  ament_target_dependencies(test_zone_path_handoff nav_msgs behaviortree_cpp tf2_ros tf2_geometry_msgs)
  target_include_directories(test_zone_path_handoff PRIVATE include)
  ament_add_gtest(test_zone_tracker test/test_zone_tracker.cpp)
  target_link_libraries(test_zone_tracker directional_zone_core)
  ament_add_gtest(test_slope test/test_slope.cpp)
  target_link_libraries(test_slope directional_zone_core)
  ament_add_gtest(test_zone_state test/test_zone_state.cpp)
  target_link_libraries(test_zone_state directional_zone_core)
  ament_add_gtest(test_recovery_policy test/test_recovery_policy.cpp)
  target_link_libraries(test_recovery_policy directional_zone_core)
  ament_add_gtest(test_zone_recovery test/test_zone_recovery.cpp)
  target_link_libraries(test_zone_recovery zone_bt_core directional_zone_core)
  ament_target_dependencies(test_zone_recovery rclcpp std_msgs behaviortree_cpp)
  ament_add_gtest(test_zone_drive test/test_zone_drive.cpp)
  target_link_libraries(test_zone_drive directional_zone_core)
  # closed-loop test drives the zones.yaml navigation.launch.py loads (terrain_dir maps/sim/terrain), not a copy
  target_compile_definitions(test_zone_drive PRIVATE
    SIM_ZONES_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../../maps/sim/terrain/zones.yaml")
endif()

ament_package()
