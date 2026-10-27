"""仿真定位入口：雷达融合 + ZLIO 前端 + 后端。

    ros2 launch sentry_gazebo sim.launch.py                    # 先启动仿真
    ros2 launch bringup sim.launch.py lidar:=single            # 仅 MID-360
    ros2 launch bringup sim.launch.py lidar:=dual rviz:=true   # MID-360 + L2

IMU 用 MID-360 内置 IMU（与雷达同原点），合并点云位于 mid360_link 系；
L2 到 MID-360 的外参只来自 tf2（URDF）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare('bringup')
    lidar = LaunchConfiguration('lidar')
    rviz = LaunchConfiguration('rviz')
    mode = LaunchConfiguration('mode')
    prior_map = LaunchConfiguration('prior_map')
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='dual', description='single 或 dual'),
        DeclareLaunchArgument('mode', default_value='mapping', description='mapping 或 localization'),
        DeclareLaunchArgument('prior_map', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='lidar_fusion', executable='merge_node', name='lidar_merge', output='screen',
            parameters=[{
                'primary_topic': '/lidar_mid360/points',
                'secondary_topic': PythonExpression(
                    ["'/lidar_l2/points' if '", lidar, "' == 'dual' else ''"]),
                'output_frame': 'mid360_link',
                'output_topic': '/lidar/points',
                'use_sim_time': True,
            }]),
        Node(
            package='frontend', executable='zlio_node', output='screen',
            parameters=[{
                'config_file': PathJoinSubstitution([bringup, 'config', 'sim_lio.yaml']),
                'input_type': 'pointcloud2',
                'lidar_topic': '/lidar/points',
                'imu_topic': '/lidar_mid360/imu',
                'imu_acceleration_scale': 1.0,
                'world_frame': 'lio_odom',
                'body_frame': 'imu',
                'coordinated_reset': True,
                'use_sim_time': True,
            }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('backend'), 'launch', 'backend.launch.py'])),
            launch_arguments={
                'config_file': PathJoinSubstitution([bringup, 'config', 'sim_backend.yaml']),
                'use_sim_time': 'true', 'mode': mode, 'prior_map': prior_map, 'rviz': rviz,
            }.items()),
    ])
