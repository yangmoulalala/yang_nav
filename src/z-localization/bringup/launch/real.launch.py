"""真车定位入口：雷达驱动 + URDF TF + config/real，管线见 localization.launch.py。

    ros2 launch bringup real.launch.py lidar:=single                       # 仅 MID-360
    ros2 launch bringup real.launch.py lidar:=dual driver_l2:=true l2_launch:=/abs/xxx.launch.py
    ros2 launch bringup real.launch.py mode:=localization                  # 用 map_dir 里的地图
    ros2 launch bringup real.launch.py mode:=localization prior_map:=/abs/map

节点参数只来自 bringup/config/real/：
  livox_driver.yaml   MID-360 驱动（frame_id 直接是 mid360_link）；IP 在 MID360_config.json
  robot_state.yaml    joint_state_publisher：云台 yaw_joint 取自 /chassis/joint_states，没收到就发 0
  lidar_fusion.yaml / frontend.yaml / backend.yaml   定位管线
外参只在 URDF：MID-360 内置 IMU 为 mid360_imu_link（Livox 出厂值）。
L2 驱动（unilidar_sdk2）是外部 launch，其 frame_id 不受本仓库控制，故保留 l2_frame 别名静态 TF。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# <ws>/install/bringup/share/bringup -> <ws>
WORKSPACE = Path(get_package_share_directory('bringup')).parents[3]

PIPELINE_ARGS = {
    'lidar': ('single', 'single 或 dual'),
    'mode': ('mapping', 'mapping 或 localization'),
    'prior_map': ('', '先验地图目录；localization 时留空则用 map_dir'),
    'map_dir': (str(WORKSPACE / 'maps' / 'real'), '地图保存目录（绝对路径）'),
    'rviz': ('false', '后端 RViz'),
}
SOURCE_ARGS = {
    # MID-360（livox_ros_driver2, xfer_format=0），话题待真车确认
    'driver_mid360': ('true', '是否启动 livox_ros_driver2'),
    'mid360_topic': ('/livox/lidar', ''),
    'imu_topic': ('/livox/imu', 'ZLIO 使用的 MID-360 内置 IMU'),
    # Unitree L2（unilidar_sdk2，外部驱动）
    'driver_l2': ('false', '是否 include unilidar_sdk2 的 launch'),
    'l2_launch': ('', 'unilidar_sdk2 launch 文件绝对路径'),
    'l2_topic': ('/unilidar/cloud', ''),
    'l2_frame': ('unilidar_lidar', 'L2 驱动点云 frame_id，用恒等静态 TF 挂到 l2_link'),
}


def generate_launch_description():
    bringup = FindPackageShare('bringup')
    config = PathJoinSubstitution([bringup, 'config', 'real'])
    arg = {key: LaunchConfiguration(key) for key in {**PIPELINE_ARGS, **SOURCE_ARGS}}
    urdf = PathJoinSubstitution([FindPackageShare('sentry_description'), 'urdf', 'sentry_robot.urdf'])
    description = {'robot_description': ParameterValue(Command(['cat ', urdf]), value_type=str)}
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v, description=d)
          for k, (v, d) in {**PIPELINE_ARGS, **SOURCE_ARGS}.items()),

        # ---- drivers -----------------------------------------------------
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen',
             condition=IfCondition(arg['driver_mid360']),
             parameters=[PathJoinSubstitution([config, 'livox_driver.yaml']),
                         {'user_config_path': PathJoinSubstitution([config, 'MID360_config.json'])}],
             remappings=[('/livox/lidar', arg['mid360_topic']), ('/livox/imu', arg['imu_topic'])]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arg['l2_launch']),
            condition=IfCondition(PythonExpression(
                ["'", arg['driver_l2'], "' == 'true' and '", arg['lidar'], "' == 'dual'"]))),

        # ---- TF: URDF + joint states + L2 frame alias -------------------
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[description]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher',
             parameters=[PathJoinSubstitution([config, 'robot_state.yaml']), description]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='alias_l2',
             condition=LaunchConfigurationEquals('lidar', 'dual'),
             arguments=['--frame-id', 'l2_link', '--child-frame-id', arg['l2_frame']]),

        # ---- shared pipeline --------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup, 'launch', 'localization.launch.py'])),
            launch_arguments={
                'config_dir': config,
                'use_sim_time': 'false',
                'primary_topic': arg['mid360_topic'],
                'secondary_topic': arg['l2_topic'],
                'imu_topic': arg['imu_topic'],
                **{k: arg[k] for k in PIPELINE_ARGS},
            }.items()),
    ])
