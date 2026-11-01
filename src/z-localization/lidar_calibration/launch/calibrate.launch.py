"""启动双雷达外参标定节点 calibrate。

    ros2 launch lidar_calibration calibrate.launch.py use_sim_time:=true      # 仿真，示例参数
    ros2 launch lidar_calibration calibrate.launch.py params_file:=/abs/lidar_calibration.yaml \\
        primary_topic:=/livox/lidar secondary_topic:=/unilidar/cloud imu_topic:=/livox/imu

节点参数只来自 params_file；本文件只额外给出 use_sim_time 与 output_file（运行选择），
话题通过重映射接线。标定完成（或失败）后节点自行退出。
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
                [FindPackageShare('lidar_calibration'), 'config', 'example.yaml']),
            description='参数文件绝对路径'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('output_file', default_value='/tmp/lidar_extrinsic.yaml',
                              description='标定结果输出文件'),
        DeclareLaunchArgument('primary_topic', default_value='/lidar_mid360/points',
                              description='主雷达点云'),
        DeclareLaunchArgument('secondary_topic', default_value='/lidar_l2/points',
                              description='副雷达点云'),
        DeclareLaunchArgument('imu_topic', default_value='/lidar_mid360/imu',
                              description='主雷达 IMU（静止判定）'),
        DeclareLaunchArgument('odometry_topic', default_value='/odometry',
                              description='里程计（位姿多样性检查）'),
        Node(
            package='lidar_calibration', executable='calibrate', name='calibrate',
            output='screen',
            parameters=[arg('params_file'), {
                'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool),
                'output_file': ParameterValue(arg('output_file'), value_type=str),
            }],
            remappings=[('primary/points', arg('primary_topic')),
                        ('secondary/points', arg('secondary_topic')),
                        ('imu', arg('imu_topic')),
                        ('odometry', arg('odometry_topic'))]),
    ])
