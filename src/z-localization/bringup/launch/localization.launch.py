"""共用定位管线：lidar_fusion（点云合并）+ ZLIO 前端 + 后端。

由 sim.launch.py / real.launch.py include，一般不直接启动。本文件只 include 各模块自己的 launch，
并传入：
  - params_file：取自 config_dir 下的 lidar_fusion.yaml / frontend.yaml / backend.yaml（节点参数唯一来源；
    lidar:=single 时后端改用 backend_single.yaml）；
  - 运行选择：use_sim_time、dual、mode、prior_map、save_dir、rviz（不在 yaml 中出现）；
  - 话题重映射。

数据流：
  primary_topic ─┐
                 ├─ lidar_merge ─ /lidar/points ─┐
  secondary_topic┘ (dual)                         ├─ zlio ─ /frontend/* ─ backend_node
  imu_topic ────────────────────────────────────┘

TF：map → lio_odom → base_link（两条动态边由后端同一消息发布）→ URDF 树。
外参全部来自 URDF：前端启动时查 frames.imu ← frames.lidar，后端查 sensor_frame ← body_frame。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

ARGS = {
    'config_dir': ('', '参数目录（含 lidar_fusion.yaml、frontend.yaml、backend.yaml），绝对路径'),
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


def generate_launch_description():
    arg = {key: LaunchConfiguration(key) for key in ARGS}

    def params(name):
        return PathJoinSubstitution([arg['config_dir'], name])

    dual = PythonExpression(["'true' if '", arg['lidar'], "' == 'dual' else 'false'"])
    # 单雷达用 backend_single.yaml（侧装单雷达专用取值），双雷达用 backend.yaml
    backend_file = PythonExpression([
        "'backend.yaml' if '", arg['lidar'], "' == 'dual' else 'backend_single.yaml'"])
    # 建图时不能把 map_dir 设为先验目录：后端拒绝保存到先验地图所在位置
    prior_map = PythonExpression([
        "'", arg['prior_map'], "' or ('", arg['map_dir'], "' if '", arg['mode'],
        "' == 'localization' else '')"])
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()),
        include('lidar_fusion', 'merge.launch.py', {
            'params_file': params('lidar_fusion.yaml'), 'use_sim_time': arg['use_sim_time'],
            'dual': dual, 'primary_topic': arg['primary_topic'],
            'secondary_topic': arg['secondary_topic'], 'output_topic': '/lidar/points'}),
        include('frontend', 'frontend.launch.py', {
            'params_file': params('frontend.yaml'), 'use_sim_time': arg['use_sim_time'],
            'lidar_topic': '/lidar/points', 'imu_topic': arg['imu_topic'],
            'visualization': 'false'}),
        include('backend', 'backend.launch.py', {
            'params_file': params(backend_file), 'use_sim_time': arg['use_sim_time'],
            'mode': arg['mode'], 'prior_map': prior_map, 'save_dir': arg['map_dir'],
            'rviz': arg['rviz']}),
    ])
