cmake_minimum_required(VERSION 3.8)
project(nav_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cmd_gate src/cmd_gate.cpp)
target_include_directories(cmd_gate PRIVATE include)
ament_target_dependencies(cmd_gate rclcpp geometry_msgs std_msgs)

install(TARGETS cmd_gate DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gate_logic test/test_gate_logic.cpp)
  target_include_directories(test_gate_logic PRIVATE include)
endif()

ament_package()
