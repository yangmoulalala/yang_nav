"""共用定位管线：lidar_fusion（点云合并）+ ZLIO 前端 + 后端。

由 sim.launch.py / real.launch.py include，一般不直接启动。本文件只 include 各模块自己的 launch，
并传入：
  - params_file：common 完整配置 + config_dir 平台差异 + single_lidar 差异；
  - 运行选择：use_sim_time、dual、mode、prior_map、save_dir、rviz（不在 yaml 中出现）；
  - 话题重映射。

数据流：
  primary_topic ─┐
                 ├─ lidar_merge ─ /lidar/points ─┐
  secondary_topic┘ (dual)                         ├─ zlio ─ /frontend/* ─ backend_node
  imu_topic ────────────────────────────────────┘

TF：map → lio_odom → gimbal_nav（两条动态边由后端同一消息发布）→ 刚性云台树。
外参全部来自 URDF：前端启动时查 frames.imu ← frames.lidar，后端查 sensor_frame ← body_frame。
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from bringup.configuration import parameter_files

ARGS = {
    'config_dir': ('', '平台差异目录 config/real 或 config/sim，绝对路径'),
    'use_sim_time': ('false', ''),
    'lidar': ('single', 'single 或 dual'),
    'mode': ('mapping', 'mapping 或 localization'),
    'prior_map': ('', '先验地图目录；localization 时留空则用 map_dir'),
    'map_dir': ('', '~/save_map 保存目录（绝对路径）'),
    'rviz': ('false', '后端 RViz'),
    'primary_topic': ('', '主雷达（MID-360）点云话题'),
    'secondary_topic': ('', '副雷达（L2）点云话题'),
    'imu_topic': ('', 'ZLIO 使用的 IMU 话题'),
}


def include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=arguments.items())


def build(context):
    arg = {key: LaunchConfiguration(key) for key in ARGS}
    lidar = arg['lidar'].perform(context)
    files = parameter_files(arg['config_dir'].perform(context),
                            modules=('lidar_fusion', 'frontend', 'backend'), lidar=lidar)

    def params(name):
        return files.name + '/' + name

    dual = 'true' if lidar == 'dual' else 'false'
    # 建图时不能把 map_dir 设为先验目录：后端拒绝保存到先验地图所在位置
    prior_map = arg['prior_map'].perform(context) or (
        arg['map_dir'].perform(context) if arg['mode'].perform(context) == 'localization' else '')
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda _context: files.cleanup())])),
        include('lidar_fusion', 'merge.launch.py', {
            'params_file': params('lidar_fusion.yaml'), 'use_sim_time': arg['use_sim_time'],
            'dual': dual, 'primary_topic': arg['primary_topic'],
            'secondary_topic': arg['secondary_topic'], 'output_topic': '/lidar/points'}),
        include('frontend', 'frontend.launch.py', {
            'params_file': params('frontend.yaml'), 'use_sim_time': arg['use_sim_time'],
            'lidar_topic': '/lidar/points', 'imu_topic': arg['imu_topic'],
            'visualization': 'false'}),
        include('backend', 'backend.launch.py', {
            'params_file': params('backend.yaml'), 'use_sim_time': arg['use_sim_time'],
            'mode': arg['mode'], 'prior_map': prior_map, 'save_dir': arg['map_dir'],
            'rviz': arg['rviz']}),
    ]


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()),
        OpaqueFunction(function=build),
    ])
