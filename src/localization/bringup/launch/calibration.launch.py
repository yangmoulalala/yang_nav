"""双雷达外参标定入口：按平台选择参数文件与话题，include lidar_calibration/calibrate.launch.py。

    ros2 launch bringup calibration.launch.py platform:=sim     # 仿真（需已运行 sentry_gazebo 与 bringup sim.launch.py lidar:=dual）
    ros2 launch bringup calibration.launch.py platform:=real    # 真车（需已运行 real.launch.py lidar:=dual）
    ros2 launch bringup calibration.launch.py platform:=real primary_topic:=/livox/lidar output_file:=/abs/out.yaml

节点参数只来自 bringup/config/<platform>/lidar_calibration.yaml。
本文件不启动驱动、URDF 或前端：TF（URDF）与 /odometry（ZLIO）必须已在运行。
话题参数留空时取平台默认值（见 TOPICS）。
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# 各平台的话题默认值与 use_sim_time。
TOPICS = ('primary_topic', 'secondary_topic', 'imu_topic', 'odometry_topic')
PLATFORMS = {
    'sim': {'use_sim_time': 'true',
            'primary_topic': '/lidar_mid360/points', 'secondary_topic': '/lidar_l2/points',
            'imu_topic': '/lidar_mid360/imu', 'odometry_topic': '/odometry'},
    'real': {'use_sim_time': 'false',
             'primary_topic': '/livox/lidar', 'secondary_topic': '/unilidar/cloud',
             'imu_topic': '/livox/imu', 'odometry_topic': '/odometry'},
}


def _include(context):
    platform = LaunchConfiguration('platform').perform(context)
    if platform not in PLATFORMS:
        raise RuntimeError(f"platform must be 'sim' or 'real', got '{platform}'")
    defaults = PLATFORMS[platform]
    topics = {k: LaunchConfiguration(k).perform(context) or defaults[k] for k in TOPICS}
    bringup = FindPackageShare('bringup')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('lidar_calibration'), 'launch', 'calibrate.launch.py'])),
        launch_arguments={
            'params_file': PathJoinSubstitution(
                [bringup, 'config', platform, 'lidar_calibration.yaml']),
            'use_sim_time': defaults['use_sim_time'],
            'output_file': LaunchConfiguration('output_file'),
            **topics,
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('platform', default_value='sim', choices=list(PLATFORMS),
                              description='sim 或 real'),
        DeclareLaunchArgument('output_file', default_value='/tmp/lidar_extrinsic.yaml',
                              description='标定结果输出文件'),
        *(DeclareLaunchArgument(k, default_value='',
                                description=f"留空取平台默认（sim: {PLATFORMS['sim'][k]}；"
                                            f"real: {PLATFORMS['real'][k]}）")
          for k in TOPICS),
        OpaqueFunction(function=_include),
    ])
