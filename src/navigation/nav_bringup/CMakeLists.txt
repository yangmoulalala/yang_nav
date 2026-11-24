cmake_minimum_required(VERSION 3.8)
project(nav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch behavior_trees rviz DESTINATION share/${PROJECT_NAME})
install(FILES params/nav2.yaml params/real.yaml DESTINATION share/${PROJECT_NAME}/params)
install(FILES params/nav2.yaml DESTINATION share/${PROJECT_NAME}/params/sim)
install(PROGRAMS scripts/send_goal.py scripts/goal_visualizer.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_send_goal test/test_send_goal.py TIMEOUT 120)
  ament_add_pytest_test(test_real_reference test/test_real_reference.py TIMEOUT 60)
  ament_add_pytest_test(test_goal_visualizer test/test_goal_visualizer.py TIMEOUT 60)
endif()

ament_package()
