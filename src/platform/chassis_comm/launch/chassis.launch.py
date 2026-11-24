"""Standalone hardware transmitter; both port and baud_rate are required."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', description='actual serial device path'),
        DeclareLaunchArgument('baud_rate', description='baud rate agreed with MCU'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('yaw_topic', default_value='/chassis/yaw_target'),
        DeclareLaunchArgument('rate', default_value='100.0', description='serial frame rate (Hz)'),
        DeclareLaunchArgument('yaw_mode', default_value='action_angle',
                              description='action_angle: XY navigation, explicit body yaw only on action; legacy relative_angle/twist'),
        DeclareLaunchArgument('yaw_horizon', default_value='0.1',
                              description='relative yaw lookahead time (s), independent of frame rate'),
        DeclareLaunchArgument('action_topic', default_value='/chassis/pass_hole'),
        Node(
            package='chassis_comm', executable='chassis_serial', output='screen',
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'baud_rate': ParameterValue(LaunchConfiguration('baud_rate'), value_type=int),
                'rate': ParameterValue(LaunchConfiguration('rate'), value_type=float),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'yaw_topic': LaunchConfiguration('yaw_topic'),
                'yaw_mode': LaunchConfiguration('yaw_mode'),
                'yaw_horizon': ParameterValue(LaunchConfiguration('yaw_horizon'), value_type=float),
                'action_topic': LaunchConfiguration('action_topic'),
                'use_sim_time': False,
            }]),
    ])
