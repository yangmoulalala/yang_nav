"""验证后端 launch 的参数校验、单一参数来源及示例配置格式。"""

import importlib.util
from pathlib import Path

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location(
    'backend_launch', PACKAGE / 'launch/backend.launch.py')
backend_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(backend_launch)

LAUNCH_ONLY = {'prior_map.enabled', 'prior_map.directory', 'trajectory.save_directory'}


def flatten(tree, prefix=''):
    """将嵌套参数树展开为点分参数名。"""
    result = {}
    for key, value in tree.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            result.update(flatten(value, name + '.'))
        else:
            result[name] = value
    return result


@pytest.mark.parametrize('text, expected',
                         [('true', True), ('OFF', False), ('1', True), ('0', False)])
def test_boolean_values(text, expected):
    """核对启动布尔值的受支持文本形式。"""
    assert backend_launch.launch_bool(text) is expected


def test_invalid_boolean_is_not_silently_true():
    """未知布尔文本应显式报错。"""
    with pytest.raises(ValueError):
        backend_launch.launch_bool('maybe')


def test_inheritance_support_is_removed():
    """launch 不再提供 YAML 继承或合并入口。"""
    assert not hasattr(backend_launch, 'load_params')
    assert not hasattr(backend_launch, 'merge')
    assert 'extends' not in (PACKAGE / 'launch/backend.launch.py').read_text(encoding='utf-8')


def launch_actions(params_file, **overrides):
    """解析声明与覆盖参数，执行生产启动入口并返回生成的动作。"""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    context = LaunchContext()
    context.launch_configurations.update({'params_file': str(params_file), **overrides})
    description = backend_launch.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    setup = next(action for action in description.entities if isinstance(action, OpaqueFunction))
    return context, setup.execute(context)


@pytest.fixture
def params_file(tmp_path):
    """提供一个存在的参数文件；launch 只检查路径，内容由节点校验。"""
    path = tmp_path / 'backend.yaml'
    path.write_text('backend_node:\n  ros__parameters: {}\n', encoding='utf-8')
    return path


@pytest.mark.parametrize('overrides', [
    {'mode': 'unknown'}, {'mode': 'config'},
    {'mode': 'localization'}, {'mode': 'localization', 'prior_map': 'missing'},
    {'mode': 'mapping', 'rviz': 'perhaps'}, {'use_sim_time': 'sometimes'},
])
def test_invalid_arguments_fail_before_process_creation(tmp_path, params_file, overrides):
    """非法模式、缺失或不存在的先验地图、非法开关均在创建进程前失败。"""
    overrides = dict(overrides)
    if overrides.get('prior_map') == 'missing':
        overrides['prior_map'] = str(tmp_path / 'missing_map')
    with pytest.raises(ValueError):
        launch_actions(params_file, **overrides)


def test_missing_params_file_fails(tmp_path):
    """参数文件不存在时不创建任何进程。"""
    with pytest.raises(ValueError):
        launch_actions(tmp_path / 'absent.yaml')


def test_mapping_passes_params_file_and_launch_only_values(params_file):
    """建图模式只传参数文件和 launch 独占参数，且忽略 prior_map。"""
    context, actions = launch_actions(params_file, save_dir='/tmp/runtime', prior_map='/ignored')
    assert len(actions) == 1
    file_arg, overrides = backend_launch.resolve({
        'params_file': str(params_file), 'mode': 'mapping', 'prior_map': '/ignored',
        'save_dir': '/tmp/runtime', 'use_sim_time': 'false', 'rviz': 'false'})[:2]
    assert file_arg == str(params_file)
    assert overrides == {'prior_map.enabled': False, 'prior_map.directory': '',
                         'trajectory.save_directory': '/tmp/runtime', 'use_sim_time': False}


def test_localization_with_rviz_yields_backend_and_rviz(tmp_path, params_file):
    """先验定位启用 RViz 时生成后端与 RViz 两个节点，且不改写可视化参数。"""
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node

    context, actions = launch_actions(
        params_file, mode='localization', prior_map=str(tmp_path), rviz='true',
        use_sim_time='true')
    assert len(actions) == 2
    assert all(isinstance(action, Node) for action in actions)
    executables = [perform_substitutions(context, normalize_to_list_of_substitutions(
        action.node_executable)) for action in actions]
    assert executables == ['backend_node', 'rviz2']
    _, overrides = backend_launch.resolve({
        'params_file': str(params_file), 'mode': 'localization', 'prior_map': str(tmp_path),
        'save_dir': '', 'use_sim_time': 'true', 'rviz': 'true'})[:2]
    assert overrides == {'prior_map.enabled': True, 'prior_map.directory': str(tmp_path),
                         'trajectory.save_directory': '', 'use_sim_time': True}
    assert not any(key.startswith('visualization') for key in overrides)


def test_example_yaml_is_ros_format_without_launch_only_or_removed_keys():
    """示例配置为 ROS 参数文件格式，不含 launch 独占、话题及已删除的外参键。"""
    data = yaml.safe_load((PACKAGE / 'config/example.yaml').read_text(encoding='utf-8'))
    assert list(data) == ['backend_node']
    keys = set(flatten(data['backend_node']['ros__parameters']))
    assert 'extends' not in keys
    assert not keys & LAUNCH_ONLY
    assert not any(key.endswith('topic') for key in keys)
    assert not any(key.startswith('common.body.') for key in keys)
    assert {'common.body_frame', 'common.sensor_frame', 'common.observation_queue_depth'} <= keys
    assert not any(key.startswith('common.gprobe.') for key in keys)
