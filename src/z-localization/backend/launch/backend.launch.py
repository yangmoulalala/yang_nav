"""启动定位后端：参数文件提供全部调参项，launch 只提供运行模式与地图目录。

每个节点参数只有一个来源：
- params_file：除下列三项外的全部后端参数（ROS 参数文件格式，不支持继承或合并）；
- launch：prior_map.enabled、prior_map.directory、trajectory.save_directory 与 use_sim_time。
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODES = ('mapping', 'localization')


def launch_bool(value):
    """解析启动参数中的布尔值并拒绝非法输入。"""
    text = str(value).lower()
    if text not in ('true', 'false', '1', '0', 'yes', 'no', 'on', 'off'):
        raise ValueError(f'Invalid boolean: {value}')
    return text in ('true', '1', 'yes', 'on')


def resolve(args):
    """校验启动参数，返回 (params_file, launch 侧参数, use_sim_time, rviz, prior)；非法输入抛 ValueError。"""
    params_file = Path(args['params_file'])
    if not params_file.is_file():
        raise ValueError(f'params_file does not exist: {params_file}')
    mode = args['mode']
    if mode not in MODES:
        raise ValueError(f'mode must be mapping or localization, got {mode!r}')
    use_sim_time = launch_bool(args['use_sim_time'])
    rviz = launch_bool(args['rviz'])
    prior = mode == 'localization'
    prior_map = ''
    if prior:
        if not args['prior_map']:
            raise ValueError('mode:=localization requires prior_map:=/absolute/map_bundle')
        if not Path(args['prior_map']).is_dir():
            raise ValueError(f'prior_map does not exist: {args["prior_map"]}')
        prior_map = args['prior_map']
    overrides = {
        'prior_map.enabled': prior,
        'prior_map.directory': prior_map,
        'trajectory.save_directory': args['save_dir'],
        'use_sim_time': use_sim_time,
    }
    return str(params_file), overrides, use_sim_time, rviz, prior


def generate_launch_description():
    """声明启动参数并构造后端及可选 RViz 节点。"""
    share = get_package_share_directory('backend')
    defaults = {
        'params_file': os.path.join(share, 'config', 'example.yaml'),
        'use_sim_time': 'false',
        'mode': 'mapping',
        'prior_map': '',
        'save_dir': '',
        'rviz': 'false',
    }
    descriptions = {
        'params_file': '后端 ROS 参数文件（backend_node: ros__parameters:）',
        'use_sim_time': '后端与 RViz 是否使用 /clock',
        'mode': 'mapping：无先验建图；localization：加载 prior_map 定位',
        'prior_map': '先验地图包目录；localization 必填且须存在，mapping 忽略',
        'save_dir': '~/save_map 与退出保存的目标目录；留空则不可保存',
        'rviz': '是否启动 RViz；可视化话题开关只由参数文件的 visualization.* 决定',
    }

    def setup(context):
        """先完成参数校验，再创建后端与可选 RViz 动作。"""
        args = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        params_file, overrides, use_sim_time, rviz, prior = resolve(args)
        actions = [
            Node(
                package='backend',
                executable='backend_node',
                name='backend_node',
                output='screen',
                parameters=[params_file, overrides],
            )
        ]
        if rviz:
            actions.append(
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='backend_rviz',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=['-d', os.path.join(share, 'rviz', 'backend.rviz')],
                    remappings=[(
                        '/backend_node/rviz_map',
                        '/backend_node/prior_map' if prior else '/backend_node/runtime_map',
                    )],
                )
            )
        return actions

    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=value, description=descriptions[key])
          for key, value in defaults.items()),
        OpaqueFunction(function=setup),
    ])
