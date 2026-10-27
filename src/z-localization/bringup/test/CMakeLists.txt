find_package(ament_cmake_pytest REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
ament_add_pytest_test(bringup_composition_test test_composition.py TIMEOUT 30)
set_tests_properties(bringup_composition_test PROPERTIES LABELS "integration")

# 停车、运动和自动故障恢复分别启动真实前后端，为多次插桩初始化保留执行预算。
add_launch_test(test_reset_protocol.py TARGET bringup_reset_protocol_test TIMEOUT 240
    ARGS "config:=${PROJECT_SOURCE_DIR}/config/lio.yaml" LABELS "ros;integration")
set_tests_properties(bringup_reset_protocol_test PROPERTIES
    ENVIRONMENT "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs")

# pytest 和动态导入的 launch 文件不在源码树生成字节码。
get_property(package_tests DIRECTORY PROPERTY TESTS)
set_property(TEST ${package_tests} APPEND PROPERTY ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
