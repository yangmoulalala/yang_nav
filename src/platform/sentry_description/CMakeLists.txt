cmake_minimum_required(VERSION 3.16)
project(sentry_description)

find_package(ament_cmake REQUIRED)

# Plain resource package: URDF + meshes, no build steps.
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(PROGRAMS gimbal_description.py DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME sensor_frame_conventions
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sensor_frames.py)
  add_test(NAME gimbal_reference
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gimbal_reference.py)
endif()

ament_package()
