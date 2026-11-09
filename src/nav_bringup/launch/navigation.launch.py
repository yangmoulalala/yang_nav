"""Sentry navigation v2 on top of z-localization.

    ros2 launch sentry_gazebo sim.launch.py arena:=rmuc uwb:=true
    ros2 launch bringup sim.launch.py lidar:=dual mode:=localization \
         prior_map:=$HOME/yang_nav/maps/sim
    ros2 launch nav_bringup navigation.launch.py terrain_dir:=$HOME/yang_nav/maps/sim

M0 nodes:
    odom_adapter      /odometry (imu)  -> /nav_odom (base_link)
    nav_map_server    <terrain_dir>/nav_map.yaml -> /nav_map  (static traversability cost)
    Nav2              planner (Smac2D), controller (MPPI Omni), smoother, behaviours,
                      bt_navigator, velocity_smoother
    cmd_gate          cmd_vel_smoothed -> /cmd_vel  (localization gate, watchdog, per-mode limits)

collision_monitor is added in M2, when terrain_node provides ground-free obstacles.

terrain_dir is a nav_terrain package, written by surface_studio (src/nav_terrain/terrain_studio.md):
    surface_studio.py <field model | prior map>   annotate patches, 导出 Nav2 -> terrain_dir
    terrain_export.py <terrain_dir>/terrain.msgpack --out <terrain_dir>   (re-export with other parameters)
Only nav_map.yaml is read here.  Live obstacles (terrain_node) arrive in M2.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIFECYCLE_NODES = ['nav_map_server', 'controller_server', 'smoother_server', 'planner_server',
                   'behavior_server', 'velocity_smoother', 'bt_navigator']


def _get(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def build(context):
    share = Path(get_package_share_directory('nav_bringup'))
    terrain_dir = Path(os.path.expanduser(_get(context, 'terrain_dir')))
    params = _get(context, 'params_file') or str(share / 'params' / 'sim' / 'nav2.yaml')
    sim = _get(context, 'use_sim_time').lower() in ('1', 'true', 'yes')
    common = {'use_sim_time': sim}
    bt_xml = str(share / 'behavior_trees' / 'navigate_m0.xml')

    def nav2(pkg, exe, name=None, extra=None, remap=None):
        return Node(package=pkg, executable=exe, name=name or exe, output='screen',
                    parameters=[params, dict(common, **(extra or {}))], remappings=remap or [])

    actions = [
        Node(package='nav_safety', executable='odom_adapter', name='odom_adapter', output='screen',
             parameters=[common], remappings=[('input', '/odometry'), ('output', '/nav_odom')]),
        nav2('nav2_map_server', 'map_server', 'nav_map_server',
             extra={'yaml_filename': str(terrain_dir / 'nav_map.yaml')}),
        nav2('nav2_controller', 'controller_server', remap=[('cmd_vel', 'cmd_vel_nav')]),
        nav2('nav2_smoother', 'smoother_server'),
        nav2('nav2_planner', 'planner_server'),
        nav2('nav2_behaviors', 'behavior_server', remap=[('cmd_vel', 'cmd_vel_nav')]),
        nav2('nav2_velocity_smoother', 'velocity_smoother', remap=[('cmd_vel', 'cmd_vel_nav')]),
        nav2('nav2_bt_navigator', 'bt_navigator', extra={
            'default_nav_to_pose_bt_xml': bt_xml}),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[dict(common, autostart=True, node_names=LIFECYCLE_NODES, bond_timeout=0.0)]),
        Node(package='nav_safety', executable='cmd_gate', name='cmd_gate', output='screen',
             parameters=[params, dict(common, require_trusted=(
                 _get(context, 'require_trusted').lower() != 'false'))],
             remappings=[('input', 'cmd_vel_smoothed'), ('output', '/cmd_vel')]),
    ]
    if _get(context, 'rviz').lower() == 'true':
        rviz = share / 'rviz' / 'navigation.rviz'
        actions.append(Node(package='rviz2', executable='rviz2', name='nav_rviz', output='screen',
                            arguments=['-d', str(rviz)] if rviz.is_file() else [],
                            parameters=[common]))
    return actions


def generate_launch_description():
    home = os.path.expanduser('~')
    return LaunchDescription([
        DeclareLaunchArgument('terrain_dir', default_value=os.path.join(home, 'yang_nav/maps/sim'),
                              description='offline terrain package for the prior map in use'),
        DeclareLaunchArgument('params_file', default_value='', description='Nav2 parameters'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('require_trusted', default_value='true',
                              description='stop unless localization reports pose_trusted=true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=build),
    ])
