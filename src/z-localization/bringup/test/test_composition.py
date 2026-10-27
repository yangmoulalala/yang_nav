"""验证组合启动入口对子节点参数的声明与传递。"""

import importlib.util
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest

_spec = importlib.util.spec_from_file_location(
    'bringup_launch', Path(__file__).resolve().parents[1] / 'launch/bringup.launch.py')
bringup = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bringup)


@pytest.mark.parametrize('mode', ['mapping', 'localization'])
def test_overrides_reach_correct_package_without_starting_processes(mode):
    description = bringup.generate_launch_description()
    context = LaunchContext()
    overrides = {
        'frontend_config': '/tmp/frontend.yaml', 'backend_config': '/tmp/backend.yaml',
        'input_type': 'pointcloud2', 'lidar_topic': '/test/lidar', 'imu_topic': '/test/imu',
        'imu_acceleration_scale': '1.0', 'use_sim_time': 'true', 'wait_for_start': 'true',
        'reset_mode': 'cold', 'mode': mode, 'prior_map': '/tmp/map', 'rviz': 'false',
        'coordinated_reset': 'false',
    }
    context.launch_configurations.update(overrides)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            if action.name == 'coordinated_reset':
                assert perform_substitutions(context, action.default_value) == 'true'
            action.execute(context)
    children = [action for action in description.entities
                if isinstance(action, IncludeLaunchDescription)
                and (action.condition is None or action.condition.evaluate(context))]
    assert len(children) == 2
    frontend = {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                for name, value in children[0].launch_arguments}
    backend = {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
               for name, value in children[1].launch_arguments}
    assert frontend == {
        'config': overrides['frontend_config'],
        **{key: overrides[key] for key in ('input_type', 'lidar_topic', 'imu_topic',
                                         'imu_acceleration_scale', 'use_sim_time',
                                         'wait_for_start', 'reset_mode', 'coordinated_reset')},
    }
    assert backend == {'config_file': overrides['backend_config'],
                       **{key: overrides[key]
                          for key in ('use_sim_time', 'mode', 'prior_map', 'rviz')}}
    # 解析子 launch 的声明，保证组合入口传递的参数仍被目标包接受。
    for child, passed in zip(children, (frontend, backend)):
        child_description = child.launch_description_source.get_launch_description(context)
        declared = {argument.name for argument in child_description.get_launch_arguments()}
        assert set(passed) <= declared


@pytest.mark.parametrize('value, enabled', [(None, False), ('false', False), ('true', True)])
def test_livox_driver_is_optional_without_starting_processes(value, enabled):
    description = bringup.generate_launch_description()
    context = LaunchContext()
    if value is not None:
        context.launch_configurations['driver'] = value
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)

    driver = next(action for action in description.entities
                  if isinstance(action, IncludeLaunchDescription) and action.condition is not None)
    included = driver.visit(context)
    if not enabled:
        assert included is None
        return

    # 展开驱动启动文件，核对节点所属包和可执行程序。
    driver_description = next(action for action in included
                              if isinstance(action, LaunchDescription))
    nodes = [action for action in driver_description.entities if isinstance(action, Node)]
    assert len(nodes) == 1
    assert perform_substitutions(context, normalize_to_list_of_substitutions(
        nodes[0].node_package)) == 'livox_ros_driver2'
    assert perform_substitutions(context, normalize_to_list_of_substitutions(
        nodes[0].node_executable)) == 'livox_ros_driver2_node'
