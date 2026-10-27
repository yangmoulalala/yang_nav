# 安装前后端统一启动入口。
cmake_minimum_required(VERSION 3.16)
project(bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.launch.py" PATTERN "__pycache__" EXCLUDE)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

ament_package()
