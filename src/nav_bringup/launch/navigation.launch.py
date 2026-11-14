"""Sentry navigation v2 on top of z-localization.

    ros2 launch sentry_gazebo sim.launch.py arena:=rmuc uwb:=true
    ros2 launch bringup sim.launch.py lidar:=dual mode:=localization \
         prior_map:=$HOME/yang_nav/maps/sim
    ros2 launch nav_bringup navigation.launch.py terrain_dir:=$HOME/yang_nav/maps/sim/terrain

Navigation nodes:
    nav_map_server    <terrain_dir>/nav_map.yaml -> /nav_map  (static traversability cost)
    Nav2              planner (Smac2D), controller (MPPI Omni), smoother, behaviours,
                      bt_navigator, velocity_smoother
    cmd_gate          cmd_vel_collision_checked -> /cmd_vel (trust gate, watchdog, per-mode limits)

terrain_obstacles provides ground-relative obstacle clouds to the costmap and collision_monitor.
Velocity chain: controller/behaviors -> velocity_smoother -> collision_monitor -> cmd_gate.

terrain_dir is a nav_terrain package, written by surface_studio (src/nav_terrain/terrain_studio.md):
    surface_studio.py <field model | prior map>   annotate patches, 导出 Nav2 -> terrain_dir
    terrain_export.py <terrain_dir>/terrain.msgpack --out <terrain_dir>   (re-export with other parameters)
nav_map.yaml feeds the costmaps; zones.yaml feeds shaping, tracking and recovery protection.
Both NavigateToPose and NavigateThroughPoses use the custom zone-aware behavior trees.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

LIFECYCLE_NODES = ['nav_map_server', 'speed_mask_server', 'speed_filter_info_server',
                   'controller_server', 'smoother_server', 'planner_server',
                   'behavior_server', 'velocity_smoother', 'collision_monitor', 'bt_navigator']


def _get(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _terrain_dir(raw):
    """A terrain package may sit beside the prior map (maps/sim) or in its own folder (maps/sim/terrain)."""
    d = Path(os.path.expanduser(raw))
    return d / 'terrain' if (d / 'terrain' / 'nav_map.yaml').is_file() else d


def build(context):
    share = Path(get_package_share_directory('nav_bringup'))
    clock_arg = _get(context, 'use_sim_time').lower()
    profile = _get(context, 'profile') or ('real' if clock_arg in ('0', 'false', 'no') else 'sim')
    if profile not in ('sim', 'real'):
        raise RuntimeError('profile must be sim or real')
    sim = profile == 'sim' if not clock_arg else clock_arg in ('1', 'true', 'yes')
    if profile == 'real' and sim:
        raise RuntimeError('real profile requires use_sim_time=false')
    workspace = share.parents[3]
    terrain_dir = _terrain_dir(_get(context, 'terrain_dir') or str(workspace / 'maps' / profile))
    params = _get(context, 'params_file') or str(share / 'params' / 'sim' / 'nav2.yaml')
    common = {'use_sim_time': sim}
    bt_xml = str(share / 'behavior_trees' / 'navigate_to_pose.xml')
    bt_xml = ReplaceString(source_file=bt_xml,
                          replacements={'__NAV_ZONES_FILE__': str(terrain_dir / 'zones.yaml')})
    through_xml = ReplaceString(
        source_file=str(share / 'behavior_trees' / 'navigate_through_poses.xml'),
        replacements={'__NAV_ZONES_FILE__': str(terrain_dir / 'zones.yaml')})
    for required in ('nav_map.yaml', 'speed_mask.yaml', 'zones.yaml', 'terrain.msgpack'):
        if not (terrain_dir / required).is_file():
            raise RuntimeError(f'Terrain package is missing {required}: {terrain_dir}; re-export it first')

    def nav2(pkg, exe, name=None, extra=None, remap=None):
        return Node(package=pkg, executable=exe, name=name or exe, output='screen',
                    parameters=[params, dict(common, **(extra or {}))], remappings=remap or [])

    actions = [
        nav2('nav2_map_server', 'map_server', 'nav_map_server',
             extra={'yaml_filename': str(terrain_dir / 'nav_map.yaml')}),
        nav2('nav2_map_server', 'map_server', 'speed_mask_server',
             extra={'yaml_filename': str(terrain_dir / 'speed_mask.yaml')}),
        nav2('nav2_map_server', 'costmap_filter_info_server', 'speed_filter_info_server'),
        Node(package='nav_terrain', executable='terrain_obstacles.py', name='terrain_obstacles',
             output='screen', parameters=[params, dict(common,
                terrain_file=str(terrain_dir / 'terrain.msgpack'))]),
        nav2('nav2_controller', 'controller_server', remap=[('cmd_vel', 'cmd_vel_nav')], extra={
            'FollowPath.zone.zones_file': str(terrain_dir / 'zones.yaml')}),
        nav2('nav2_smoother', 'smoother_server', extra={
            'zone_shaper.zones_file': str(terrain_dir / 'zones.yaml')}),
        nav2('nav2_planner', 'planner_server'),
        nav2('nav2_behaviors', 'behavior_server', remap=[('cmd_vel', 'cmd_vel_nav')]),
        nav2('nav2_velocity_smoother', 'velocity_smoother', remap=[('cmd_vel', 'cmd_vel_nav')]),
        nav2('nav2_collision_monitor', 'collision_monitor'),
        nav2('nav2_bt_navigator', 'bt_navigator', extra={
            'default_nav_to_pose_bt_xml': bt_xml,
            'default_nav_through_poses_bt_xml': through_xml}),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[dict(common, autostart=True, node_names=LIFECYCLE_NODES, bond_timeout=0.0)]),
        Node(package='nav_safety', executable='cmd_gate', name='cmd_gate', output='screen',
             parameters=[params, dict(common, require_trusted=(
                 _get(context, 'require_trusted').lower() != 'false'))],
             remappings=[('input', 'cmd_vel_collision_checked'), ('output', _get(context, 'cmd_vel_topic'))]),
    ]
    if _get(context, 'rviz').lower() == 'true':
        rviz = share / 'rviz' / 'navigation.rviz'
        actions.append(Node(package='rviz2', executable='rviz2', name='nav_rviz', output='screen',
                            arguments=['-d', str(rviz)] if rviz.is_file() else [],
                            parameters=[common]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='', description='sim or real; inferred from use_sim_time if omitted'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel', description='final gated chassis Twist topic'),
        DeclareLaunchArgument('terrain_dir', default_value='',
                              description='offline terrain package for the prior map in use '
                                          '(maps/sim or an export folder such as maps/sim/terrain)'),
        DeclareLaunchArgument('params_file', default_value='', description='Nav2 parameters'),
        DeclareLaunchArgument('use_sim_time', default_value='', description='default true for sim, false for real'),
        DeclareLaunchArgument('require_trusted', default_value='true',
                              description='stop unless localization reports pose_trusted=true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=build),
    ])
