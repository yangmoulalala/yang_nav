cmake_minimum_required(VERSION 3.8)
project(chassis_comm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/chassis_serial DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_protocol test/test_protocol.py)
  ament_add_pytest_test(test_big_yaw test/test_big_yaw.py)
  ament_add_pytest_test(test_command_state test/test_command_state.py)
  ament_add_pytest_test(test_navigation_state test/test_navigation_state.py)
  ament_add_pytest_test(test_twist_adapter test/test_twist_adapter.py)
  ament_add_pytest_test(test_serial_node test/test_serial_node.py TIMEOUT 45)
  ament_add_pytest_test(test_feedback_serial test/test_feedback_serial.py TIMEOUT 45)
endif()

ament_package()
