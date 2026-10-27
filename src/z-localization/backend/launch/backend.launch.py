"""启动定位后端，并应用配置、地图和可视化选项。"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 解析启动参数中的布尔值并拒绝非法输入。
def launch_bool(value):
    if str(value).lower() not in ('true', 'false', '1', '0', 'yes', 'no', 'on', 'off'):
        raise ValueError(f'Invalid boolean: {value}')
    return str(value).lower() in ('true', '1', 'yes', 'on')


# 递归合并配置映射，以覆盖值替换对应叶参数。
def merge(base, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


# 读取单父继承配置并检查循环引用。
def load_params(path, ancestors=()):
    path = Path(path).resolve()
    if path in ancestors:
        raise ValueError(f'Configuration inheritance cycle: {path}')
    with path.open(encoding='utf-8') as stream:
        params = yaml.safe_load(stream) or {}
    if not isinstance(params, dict):
        raise ValueError(f'Configuration must be a mapping: {path}')
    parent = params.pop('extends', None)
    if parent:
        params = merge(load_params(path.parent / parent, ancestors=(*ancestors, path)), params)
    return params


# 声明启动参数并构造节点启动描述。
def generate_launch_description():
    share = get_package_share_directory('backend')
    defaults = {
        'config_file': os.path.join(share, 'config', 'backend.yaml'),
        'use_sim_time': 'false',
        'rviz': 'false',
        'mode': 'config',
        'prior_map': ''
    }

    # 解析启动上下文，生成后端与可视化组件动作。
    def setup(context):
        args = {key: LaunchConfiguration(key).perform(context) for key in defaults}
        params = load_params(args['config_file'])
        if args['mode'] not in ('config', 'mapping', 'localization'):
            raise ValueError('mode must be config, mapping or localization')
        if args['mode'] != 'config':
            params.setdefault('prior_map', {})['enabled'] = args['mode'] == 'localization'
        if args['prior_map']:
            params.setdefault('prior_map', {})['directory'] = args['prior_map']
        prior = bool(params.get('prior_map', {}).get('enabled', False))
        if prior and (not Path(params['prior_map'].get('directory', '')).is_dir()):
            raise ValueError(
                'Prior map does not exist; set prior_map:=/absolute/map or mode:=mapping'
            )
        if launch_bool(args['rviz']):
            params.setdefault('visualization', {}).update(
                enabled=True,
                publish_prior_map=prior,
                publish_runtime_map=not prior,
                publish_map_axes=True,
                publish_paths=True,
                publish_debug_registration=True
            )
        actions = [
            Node(
                package='backend',
                executable='backend_node',
                name='backend_node',
                output='screen',
                parameters=[params, {'use_sim_time': launch_bool(args['use_sim_time'])}]
            )
        ]
        if launch_bool(args['rviz']):
            actions.append(
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='backend_rviz',
                    parameters=[{'use_sim_time': launch_bool(args['use_sim_time'])}],
                    arguments=['-d', os.path.join(share, 'rviz', 'backend.rviz')],
                    remappings=[
                        (
                            '/backend_node/rviz_map',
                            '/backend_node/prior_map' if prior else '/backend_node/runtime_map'
                        )
                    ]
                )
            )
        return actions

    return LaunchDescription(
        [
            *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
            OpaqueFunction(function=setup)
        ]
    )
