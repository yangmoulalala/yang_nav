cmake_minimum_required(VERSION 3.8)
project(nav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch behavior_trees rviz DESTINATION share/${PROJECT_NAME})
install(FILES params/nav2.yaml DESTINATION share/${PROJECT_NAME}/params)
install(FILES params/nav2.yaml DESTINATION share/${PROJECT_NAME}/params/sim)

ament_package()
