"""统一启动 ZLIO 前端、定位后端及可选的 Livox 驱动。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """声明公共入口参数，并分别传给前端和后端启动文件。"""
    frontend = FindPackageShare('frontend')
    backend = FindPackageShare('backend')
    defaults = {
        'frontend_config': PathJoinSubstitution([frontend, 'config', 'lio.yaml']),
        'backend_config': PathJoinSubstitution([backend, 'config', 'backend.yaml']),
        'driver': 'false',
        'input_type': 'livox',
        'lidar_topic': '/livox/lidar',
        'imu_topic': '/livox/imu',
        'imu_acceleration_scale': '9.81',
        'use_sim_time': 'false',
        'wait_for_start': 'false',
        'reset_mode': 'warm',
        # 运行期输入中断由后端协调前端换代。
        'coordinated_reset': 'true',
        'mode': 'config',
        'prior_map': '',
        'rviz': 'false'
    }
    args = {key: LaunchConfiguration(key) for key in defaults}
    return LaunchDescription(
        [
            *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('livox_ros_driver2'),
                        'launch_ROS2', 'msg_MID360_launch.py'
                    ])
                ),
                condition=IfCondition(args['driver'])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([frontend, 'launch', 'lio.launch.py'])
                ),
                launch_arguments={
                    'config': args['frontend_config'],
                    **{
                        key: args[key]
                        for key in (
                            'input_type',
                            'lidar_topic',
                            'imu_topic',
                            'imu_acceleration_scale',
                            'use_sim_time',
                            'wait_for_start',
                            'reset_mode',
                            'coordinated_reset'
                        )
                    }
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([backend, 'launch', 'backend.launch.py'])
                ),
                launch_arguments={
                    'config_file': args['backend_config'],
                    **{key: args[key] for key in ('use_sim_time', 'mode', 'prior_map', 'rviz')}
                }.items()
            )
        ]
    )
