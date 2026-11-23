"""启动雷达合并节点 lidar_merge。

    ros2 launch lidar_fusion merge.launch.py                       # 单雷达，示例参数
    ros2 launch lidar_fusion merge.launch.py dual:=true params_file:=/abs/lidar_fusion.yaml

节点参数只来自 params_file；本文件只额外给出 use_sim_time 与 dual（运行选择），
话题通过重映射接线。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('lidar_fusion'), 'config', 'example.yaml']),
            description='参数文件绝对路径'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('dual', default_value='false', description='是否合并副雷达'),
        DeclareLaunchArgument('primary_topic', default_value='/lidar_mid360/points'),
        DeclareLaunchArgument('secondary_topic', default_value='/lidar_l2/points'),
        DeclareLaunchArgument('output_topic', default_value='/lidar/points'),
        Node(
            package='lidar_fusion', executable='merge_node', name='lidar_merge',
            output='screen',
            parameters=[arg('params_file'), {
                'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool),
                'dual': ParameterValue(arg('dual'), value_type=bool),
            }],
            remappings=[('primary/points', arg('primary_topic')),
                        ('secondary/points', arg('secondary_topic')),
                        ('lidar/points', arg('output_topic'))]),
    ])
