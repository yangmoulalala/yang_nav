# 安装前后端统一启动入口。
cmake_minimum_required(VERSION 3.16)
project(bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.launch.py" PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME check_bringup_params
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/check_bringup_params.py)
endif()

ament_package()
