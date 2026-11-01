"""独立启动 ZLIO 前端，可选启动前端可视化适配器及专属 RViz。

节点参数的唯一来源是 params_file；launch 只负责话题重映射与仿真时间。
imu ← lidar 外参须在启动时由外部 TF 提供。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """声明参数文件、仿真时间、输入话题重映射和可视化开关。"""
    share = FindPackageShare('frontend')
    params_file = LaunchConfiguration('params_file')
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    visualization = IfCondition(LaunchConfiguration('visualization'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([share, 'config', 'example.yaml']),
            description='ROS 参数文件，含 zlio 与 frontend_visualization 两节'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='使用 /clock 提供的仿真时间'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar',
                              description='重映射到节点输入 lidar 的话题'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu',
                              description='重映射到节点输入 imu 的话题'),
        DeclareLaunchArgument('visualization', default_value='false',
                              description='启动前端可视化节点与 RViz'),
        Node(
            package='frontend', executable='zlio_node', name='zlio', output='screen',
            parameters=[params_file, clock],
            remappings=[('lidar', LaunchConfiguration('lidar_topic')),
                        ('imu', LaunchConfiguration('imu_topic'))],
        ),
        Node(
            package='frontend', executable='zlio_visualizer', name='frontend_visualization',
            output='screen', parameters=[params_file, clock], condition=visualization,
        ),
        Node(
            package='rviz2', executable='rviz2', name='frontend_rviz', output='screen',
            arguments=['-d', PathJoinSubstitution([share, 'rviz', 'frontend.rviz'])],
            parameters=[clock], condition=visualization,
        ),
    ])
