"""真车定位入口：雷达驱动 + URDF TF + 雷达融合 + ZLIO 前端 + 后端。

    ros2 launch bringup real.launch.py lidar:=single                       # 仅 MID-360
    ros2 launch bringup real.launch.py lidar:=dual                         # MID-360 + L2
    ros2 launch bringup real.launch.py mode:=localization prior_map:=/abs/map

与仿真入口（sim.launch.py）共用融合节点、ZLIO 与后端；差别只在数据来源：
  - MID-360：livox_ros_driver2 输出 PointCloud2（xfer_format=0），IMU 单位 g；
  - L2：宇树 unilidar_sdk2（需单独安装并启动，driver_l2:=true 时由本文件 include）；
  - 外参：URDF 由 robot_state_publisher 发布，驱动的 frame_id 用恒等静态 TF 挂到对应 link。

标注“待真车确认”的默认值（话题、frame、逐点时间字段）请上车后用
``ros2 topic echo --once <topic> --field fields`` 核对，改启动参数即可，无需改代码。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

DEFAULTS = {
    'lidar': ('single', 'single 或 dual'),
    'mode': ('mapping', 'mapping 或 localization'),
    'prior_map': ('', '先验地图目录（localization 时必填）'),
    'rviz': ('false', ''),
    # MID-360（livox_ros_driver2, xfer_format=0）
    'driver_mid360': ('true', '是否启动 livox_ros_driver2'),
    'mid360_topic': ('/livox/lidar', ''),
    'mid360_frame': ('livox_frame', '驱动点云 frame_id'),
    'imu_topic': ('/livox/imu', 'ZLIO 使用的 MID-360 内置 IMU'),
    'imu_frame': ('livox_frame', 'IMU frame_id'),
    # PointCloud2 'timestamp' 字段为绝对时间 ns（livox_ros_driver2 lddc.cpp），待真车确认
    'mid360_time_field': ('timestamp', ''),
    'mid360_time_scale': ('1e-9', ''),
    'mid360_time_absolute': ('true', ''),
    # Unitree L2（unilidar_sdk2），以下默认值待真车确认
    'driver_l2': ('false', '是否 include unilidar_sdk2 的 launch'),
    'l2_launch': ('', 'unilidar_sdk2 launch 文件绝对路径'),
    'l2_topic': ('/unilidar/cloud', ''),
    'l2_frame': ('unilidar_lidar', ''),
    'l2_time_field': ('time', ''),
    'l2_time_scale': ('1.0', ''),
    'l2_time_absolute': ('false', ''),
    # 云台关节：真车应由底盘通信节点发布 /joint_states；没有时用 joint_state_publisher 发零位
    'zero_joint_states': ('true', ''),
}


def generate_launch_description():
    bringup = FindPackageShare('bringup')
    arg = {key: LaunchConfiguration(key) for key in DEFAULTS}
    urdf = PathJoinSubstitution([FindPackageShare('sentry_description'), 'urdf', 'sentry_robot.urdf'])
    description = ParameterValue(Command(['cat ', urdf]), value_type=str)
    dual = LaunchConfigurationEquals('lidar', 'dual')
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in DEFAULTS.items()),

        # ---- drivers -----------------------------------------------------
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen',
             condition=IfCondition(arg['driver_mid360']),
             parameters=[{'xfer_format': 0, 'multi_topic': 0, 'data_src': 0,
                          'publish_freq': 10.0, 'output_data_type': 0,
                          'frame_id': arg['mid360_frame'],
                          'user_config_path': PathJoinSubstitution(
                              [FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json'])}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arg['l2_launch']),
            condition=IfCondition(PythonExpression(
                ["'", arg['driver_l2'], "' == 'true' and '", arg['lidar'], "' == 'dual'"]))),

        # ---- TF: URDF + driver frame aliases ----------------------------
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=IfCondition(arg['zero_joint_states']),
             parameters=[{'robot_description': description}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='alias_mid360',
             arguments=['--frame-id', 'mid360_link', '--child-frame-id', arg['mid360_frame']]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='alias_l2',
             condition=dual,
             arguments=['--frame-id', 'l2_link', '--child-frame-id', arg['l2_frame']]),

        # ---- shared pipeline (same as sim.launch.py) ---------------------
        Node(package='lidar_fusion', executable='merge_node', name='lidar_merge', output='screen',
             parameters=[{
                 'primary_topic': arg['mid360_topic'],
                 'primary_time_field': arg['mid360_time_field'],
                 'primary_time_scale': ParameterValue(arg['mid360_time_scale'], value_type=float),
                 'primary_time_absolute': ParameterValue(arg['mid360_time_absolute'], value_type=bool),
                 'secondary_topic': PythonExpression(
                     ["'", arg['l2_topic'], "' if '", arg['lidar'], "' == 'dual' else ''"]),
                 'secondary_time_field': arg['l2_time_field'],
                 'secondary_time_scale': ParameterValue(arg['l2_time_scale'], value_type=float),
                 'secondary_time_absolute': ParameterValue(arg['l2_time_absolute'], value_type=bool),
                 'output_frame': 'mid360_link',
                 'output_topic': '/lidar/points',
             }]),
        Node(package='frontend', executable='zlio_node', output='screen',
             parameters=[{
                 'config_file': PathJoinSubstitution([FindPackageShare('frontend'), 'config', 'lio.yaml']),
                 'input_type': 'pointcloud2',
                 'lidar_topic': '/lidar/points',
                 'imu_topic': arg['imu_topic'],
                 'imu_acceleration_scale': 9.81,
                 'world_frame': 'lio_odom',
                 'body_frame': 'imu',
                 'coordinated_reset': True,
             }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('backend'), 'launch', 'backend.launch.py'])),
            launch_arguments={
                'config_file': PathJoinSubstitution([bringup, 'config', 'real_backend.yaml']),
                'use_sim_time': 'false', 'mode': arg['mode'], 'prior_map': arg['prior_map'],
                'rviz': arg['rviz'],
            }.items()),
    ])
