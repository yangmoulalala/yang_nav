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

# geometry only, no ROS: unit tested offline
add_library(path_shaper_core STATIC src/path_shaper.cpp src/tunnel_tracker.cpp src/zones_yaml.cpp)
target_include_directories(path_shaper_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(path_shaper_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(path_shaper_core ${YAML_CPP_LIBRARIES})

add_library(zone_path_shaper SHARED src/zone_path_shaper.cpp)
target_link_libraries(zone_path_shaper path_shaper_core ${YAML_CPP_LIBRARIES})
ament_target_dependencies(zone_path_shaper rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util nav_msgs
  pluginlib)

add_library(zone_aware_controller SHARED src/zone_aware_controller.cpp)
target_link_libraries(zone_aware_controller path_shaper_core)
ament_target_dependencies(zone_aware_controller rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util nav_msgs
  pluginlib std_msgs)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS zone_path_shaper zone_aware_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_shaper test/test_path_shaper.cpp)
  target_link_libraries(test_path_shaper path_shaper_core)
  ament_add_gtest(test_tunnel_tracker test/test_tunnel_tracker.cpp)
  target_link_libraries(test_tunnel_tracker path_shaper_core)
endif()

ament_package()
