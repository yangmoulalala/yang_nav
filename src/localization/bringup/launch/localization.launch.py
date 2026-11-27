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
    'save_interval': ('', '建图期间周期保存地图的间隔（秒）；留空时有输出目录的建图默认 30，定位模式 0'),
    'rviz': ('false', '后端 RViz'),
    'backend_overrides': ('', '后端 ROS 参数覆盖 YAML'),
    'lidar_min_range': ('', '雷达近距软件过滤下限（米），留空使用配置默认值'),
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
    lidar_range = arg['lidar_min_range'].perform(context).strip()
    files = parameter_files(arg['config_dir'].perform(context),
                            modules=('lidar_fusion', 'frontend', 'backend'), lidar=lidar,
                            backend_overrides=arg['backend_overrides'].perform(context),
                            lidar_min_range=float(lidar_range) if lidar_range else None)

    def params(name):
        return files.name + '/' + name

    dual = 'true' if lidar == 'dual' else 'false'
    mode = arg['mode'].perform(context)
    save_dir = arg['map_dir'].perform(context)
    # 断电走不到退出保存，建图默认每 30 s 落盘一次；定位模式不保存运行轨迹。
    save_interval = arg['save_interval'].perform(context).strip() or (
        '30' if mode == 'mapping' and save_dir else '0')
    # 建图时不能把 map_dir 设为先验目录：后端拒绝保存到先验地图所在位置
    prior_map = arg['prior_map'].perform(context) or (
        save_dir if mode == 'localization' else '')
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
            'mode': mode, 'prior_map': prior_map, 'save_dir': save_dir,
            'save_interval': save_interval,
            'rviz': arg['rviz']}),
    ]


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()),
        OpaqueFunction(function=build),
    ])
