"""仿真定位入口：雷达融合 + ZLIO 前端 + 后端。

    ros2 launch sentry_gazebo sim.launch.py                    # 先启动仿真
    ros2 launch bringup sim.launch.py lidar:=single            # 仅 MID-360
    ros2 launch bringup sim.launch.py lidar:=dual rviz:=true   # MID-360 + L2
    ros2 launch bringup sim.launch.py mode:=localization       # 用 map_dir 里保存的地图定位

地图：建图时 ~/save_map 保存到 map_dir（默认 <工作空间>/maps/sim）；
localization 未给 prior_map 时从 map_dir 加载。

IMU 用 MID-360 内置 IMU（与雷达同原点），合并点云位于 mid360_link 系；
L2 到 MID-360 的外参只来自 tf2（URDF）。

TF：map → lio_odom（后端）→ base_link（base_tf，由 /odometry 与 URDF 外参换算）→ URDF 树。
ZLIO 不发 TF；它的 imu 系与 mid360_link 重合，用恒等静态 TF 挂进 URDF 树。
后端不发 map → body，整棵树只有 base_link 一个车体帧。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    map_dir = LaunchConfiguration('map_dir')
    # <ws>/install/bringup/share/bringup -> <ws>
    workspace = Path(get_package_share_directory('bringup')).parents[3]
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='dual', description='single 或 dual'),
        DeclareLaunchArgument('mode', default_value='mapping', description='mapping 或 localization'),
        DeclareLaunchArgument('prior_map', default_value='',
                              description='先验地图目录；localization 时留空则用 map_dir'),
        DeclareLaunchArgument('map_dir', default_value=str(workspace / 'maps' / 'sim'),
                              description='地图保存目录（绝对路径）'),
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
                # TF 由下面的 base_tf 发到 base_link，imu 已在 URDF 树上
                'publish_tf': False,
                'use_sim_time': True,
            }]),
        # 仿真 IMU 位于雷达原点（sim_lio.yaml 外参为单位阵），imu 系即 mid360_link
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='alias_imu',
            output='screen',
            arguments=['--frame-id', 'mid360_link', '--child-frame-id', 'imu'],
            parameters=[{'use_sim_time': True}]),
        Node(
            package='lidar_fusion', executable='base_tf', name='lio_base_tf', output='screen',
            parameters=[{'odometry_topic': '/odometry', 'base_frame': 'base_link',
                         'use_sim_time': True}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('backend'), 'launch', 'backend.launch.py'])),
            launch_arguments={
                'config_file': PathJoinSubstitution([bringup, 'config', 'sim_backend.yaml']),
                'use_sim_time': 'true', 'mode': mode, 'rviz': rviz, 'save_dir': map_dir,
                # 建图时不能把 map_dir 设为先验目录：后端拒绝保存到先验地图所在位置
                'prior_map': PythonExpression(
                    ["'", prior_map, "' or ('", map_dir, "' if '", mode,
                     "' == 'localization' else '')"]),
            }.items()),
    ])
