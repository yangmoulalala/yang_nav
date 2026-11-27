"""仿真定位入口：只给出仿真数据源与 config/sim，管线见 localization.launch.py。

    ros2 launch sentry_gazebo sim.launch.py                    # 先启动仿真
    ros2 launch bringup sim.launch.py lidar:=single            # 仅 MID-360
    ros2 launch bringup sim.launch.py lidar:=dual rviz:=true   # MID-360 + L2
    ros2 launch bringup sim.launch.py mode:=localization       # 用 map_dir 里保存的地图定位

节点参数来自 config/common 与 config/sim 的必要差异。
IMU 使用真车同一 mid360_imu_link 外参；Gazebo 输出单位为 m/s²。
刚性云台 URDF 与 robot_state_publisher 由 sentry_gazebo 启动。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# <ws>/install/bringup/share/bringup -> <ws>
WORKSPACE = Path(get_package_share_directory('bringup')).parents[3]

ARGS = {
    'lidar': ('single', 'single 或 dual'),
    'mode': ('mapping', 'mapping 或 localization'),
    'prior_map': ('', '先验地图目录；localization 时留空则用 map_dir'),
    'map_dir': (str(WORKSPACE / 'maps' / 'sim'), '地图保存目录（绝对路径）'),
    'save_interval': ('', '定时保存间隔（秒）；留空时有输出目录的建图默认 30，0 关闭'),
    'rviz': ('false', '后端 RViz'),
    'backend_overrides': ('', '后端 ROS 参数覆盖 YAML'),
}


def generate_launch_description():
    bringup = FindPackageShare('bringup')
    arg = {key: LaunchConfiguration(key) for key in ARGS}
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup, 'launch', 'localization.launch.py'])),
            launch_arguments={
                'config_dir': PathJoinSubstitution([bringup, 'config', 'sim']),
                'use_sim_time': 'true',
                'primary_topic': '/lidar_mid360/points',
                'secondary_topic': '/lidar_l2/points',
                'imu_topic': '/lidar_mid360/imu',
                **arg,
            }.items()),
    ])
