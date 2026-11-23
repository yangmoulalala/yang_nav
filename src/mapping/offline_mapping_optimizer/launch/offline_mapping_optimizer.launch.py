"""Launch the batch optimizer; the workspace wrapper propagates its exit status directly."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('offline_mapping_optimizer'), 'config', 'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('data_path', description='poses.txt and frame_*.pcd directory'),
        DeclareLaunchArgument('output_path', description='New, separate result directory'),
        DeclareLaunchArgument('config', default_value=default_config),
        Node(package='offline_mapping_optimizer', executable='offline_mapping_optimizer_node',
             output='screen', parameters=[LaunchConfiguration('config'), {
                 'data_path': ParameterValue(LaunchConfiguration('data_path'), value_type=str),
                 'output_path': ParameterValue(LaunchConfiguration('output_path'), value_type=str),
             }]),
    ])
