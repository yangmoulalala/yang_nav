"""MiniPC navigation serial bridge, default /dev/nav at 115200 baud."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARGUMENTS = {
    'port': ('/dev/nav', str, 'navigation serial device path'),
    'baud_rate': ('115200', int, 'baud rate agreed with MCU'),
    'rate': ('100.0', float, 'A3 serial frame rate (Hz)'),
    'command_timeout': ('0.3', float, 'velocity input timeout (s)'),
    'pose_timeout': ('0.3', float, 'localization TF timeout (s)'),
    'yaw_timeout': ('0.3', float, 'relative turn input timeout (s); stale means hold heading'),
    'write_timeout': ('0.005', float, 'serial write timeout (s)'),
    'feedback_timeout': ('0.1', float, 'A4 online and incomplete RX frame timeout (s)'),
    'world_frame': ('map', str, 'shared A3 current pose / A4 target coordinate frame'),
    'base_frame': ('gimbal_nav', str, 'radar / big-yaw navigation frame'),
    'cmd_vel_topic': ('/cmd_vel', str, 'final gated radar-frame XY velocity'),
    'yaw_topic': ('/chassis/yaw_target', str, 'desired turn from current radar heading, Float32 rad'),
    'action_topic': ('/chassis/pass_hole', str, 'Bool hole mode level (not a rising edge)'),
    'target_topic': ('/chassis/nav_target', str, 'A4 target output, PointStamped'),
    'big_yaw_topic': ('/chassis/big_yaw', str, 'A4 measured chassis-to-radar angle, Float32 rad'),
    'big_yaw_online_topic': ('/chassis/big_yaw_online', str, 'A4 feedback online, retained Bool'),
}


def generate_launch_description():
    declarations = [DeclareLaunchArgument(
        name, description=description, **({} if default is None else {'default_value': default}))
        for name, (default, _, description) in ARGUMENTS.items()]
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
                  for name, (_, value_type, _) in ARGUMENTS.items()}
    return LaunchDescription([
        *declarations,
        Node(package='chassis_comm', executable='chassis_serial', output='screen',
             parameters=[parameters, {'use_sim_time': False}]),
    ])
