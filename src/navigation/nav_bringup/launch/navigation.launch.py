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

terrain_dir is a nav_terrain package exported by the WebUI terrain editor (docs/web-ui.md):
    terrain_export.py <terrain_dir>/terrain.msgpack --out <terrain_dir>   (re-export with other parameters)
nav_map.yaml feeds the costmaps; zones.yaml feeds shaping, tracking and recovery protection.
Both NavigateToPose and NavigateThroughPoses use the custom zone-aware behavior trees.
mode:=mapping uses rolling live costmaps instead of the offline terrain package.
"""
import math
import os
from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml

from ament_index_python.packages import get_package_share_directory
from bringup.configuration import merge
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

TERRAIN_NODES = ['nav_map_server', 'speed_mask_server', 'speed_filter_info_server']
LIFECYCLE_NODES = ['controller_server', 'smoother_server', 'planner_server',
                   'behavior_server', 'velocity_smoother', 'collision_monitor', 'bt_navigator']


def _get(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _terrain_dir(raw):
    """A terrain package may sit beside the prior map (maps/sim) or in its own folder (maps/sim/terrain)."""
    d = Path(os.path.expanduser(raw))
    return d / 'terrain' if (d / 'terrain' / 'nav_map.yaml').is_file() else d


def mapping_params(document):
    # The local window only has to cover the controller rollout; the global one also holds the
    # goals, so at 8 m every target further than 4 m fell outside the costmap and was rejected.
    for name, size in (('local_costmap', 8), ('global_costmap', 30)):
        costmap = document[name][name]['ros__parameters']
        costmap.update(rolling_window=True, track_unknown_space=True, width=size, height=size,
                       resolution=0.05, plugins=['obstacle_layer', 'inflation_layer'])
        costmap.pop('filters', None)
        costmap['obstacle_layer']['plugin'] = 'nav2_costmap_2d::ObstacleLayer'
    planners = document['planner_server']['ros__parameters']
    for name in ('GridBased', 'StrictGridBased'):
        planners[name].update(allow_unknown=False, approach_occupied_goal=False,
                              require_observed_path=True, tolerance=0.0,
                              approach_unknown_goal=(name == 'GridBased'))
    # Mapping keeps the pre-tune conservative envelope: the real profile raises the
    # driver limits for racing, not for building maps.
    follow = document['controller_server']['ros__parameters']['FollowPath']
    follow.update(vx_max=1.5, vx_min=-1.5, vy_max=1.5, ax_max=0.9, ax_min=-1.0,
                  ay_max=0.9, ay_min=-1.0)
    follow['TranslationVelocityCritic']['max_speed'] = 1.0
    gate = document['cmd_gate']['ros__parameters']
    for limits in gate['limits'].values():
        limits.update(max_linear=min(limits['max_linear'], 0.5), max_angular=0.0)
    gate.update(require_observation=True)
    return document


def navigation_params(source, real_overrides=None, mapping=False):
    document = yaml.safe_load(Path(source).read_text())
    if real_overrides is not None:
        document = merge(document, yaml.safe_load(real_overrides.read_text()))
    return mapping_params(document) if mapping else document


def velocity_smoother_executable(profile, document):
    follow = document.get('controller_server', {}).get('ros__parameters', {}).get('FollowPath', {})
    if profile == 'real' and follow.get('fixed_frame_translation') is True:
        return 'nav_behaviors', 'fixed_frame_velocity_smoother'
    return 'nav2_velocity_smoother', 'velocity_smoother'


def build(context):
    share = Path(get_package_share_directory('nav_bringup'))
    goal_x, goal_y = _get(context, 'goal_x'), _get(context, 'goal_y')
    if bool(goal_x) != bool(goal_y):
        raise RuntimeError('set both goal_x and goal_y to send a startup goal')
    if goal_x:
        values = [goal_x, goal_y, _get(context, 'goal_yaw'), _get(context, 'goal_wait_timeout')]
        try:
            numbers = [float(value) for value in values]
        except ValueError as exc:
            raise RuntimeError('startup goal coordinates, yaw and wait timeout must be numbers') from exc
        if not all(math.isfinite(value) for value in numbers) or numbers[3] <= 0:
            raise RuntimeError('startup goal values must be finite and goal_wait_timeout must be positive')
    clock_arg = _get(context, 'use_sim_time').lower()
    profile = _get(context, 'profile') or ('real' if clock_arg in ('0', 'false', 'no') else 'sim')
    if profile not in ('sim', 'real'):
        raise RuntimeError('profile must be sim or real')
    sim = profile == 'sim' if not clock_arg else clock_arg in ('1', 'true', 'yes')
    if profile == 'real' and sim:
        raise RuntimeError('real profile requires use_sim_time=false')
    mode = _get(context, 'mode')
    if mode not in ('mapping', 'localization'):
        raise RuntimeError('mode must be mapping or localization')
    mapping = mode == 'mapping'
    serial_goals = _get(context, 'serial_goals').lower() == 'true'
    if serial_goals and (profile != 'real' or goal_x):
        raise RuntimeError('serial_goals requires the real profile without a startup goal')
    terrain_arg = _get(context, 'terrain_dir')
    if mapping and terrain_arg:
        raise RuntimeError('mapping navigation does not use an offline terrain_dir')
    terrain_dir = None if mapping else _terrain_dir(terrain_arg or str(share.parents[3] / 'maps' / profile))
    custom_params = _get(context, 'params_file')
    params_dir = share / 'params'
    params = custom_params or str(params_dir / 'nav2.yaml')
    real_overrides = params_dir / 'nav2_real.yaml' if profile == 'real' and not custom_params else None
    document = navigation_params(params, real_overrides, mapping)
    smoother_package, smoother_executable = velocity_smoother_executable(profile, document)
    follow = document.get('controller_server', {}).get('ros__parameters', {}).get('FollowPath', {})
    curve_accel = 0.0
    reentry_timeout = 0.0
    if profile == 'real' and follow.get('fixed_frame_translation') and follow.get('path_departure_limit', 0.) > 0.:
        smoother = document['velocity_smoother']['ros__parameters']
        curve_accel = min(min(smoother['max_accel'][:2]), -max(smoother['max_decel'][:2]),
                          follow.get('ax_max', 0.), follow.get('ay_max', 0.),
                          -follow.get('ax_min', 0.), -follow.get('ay_min', 0.))
        reentry_timeout = max(0.5, smoother.get('velocity_timeout', 0.5)) + 0.1
    zone = follow.get('zone', {})
    clear_reentry = curve_accel > 0.0 and zone.get('body_heading_available') is False and zone.get('angular_control') is False
    require_trusted = mapping or _get(context, 'require_trusted').lower() != 'false'
    common = {'use_sim_time': sim}
    base_frame = 'gimbal_nav'
    zones = '' if mapping else str(terrain_dir / 'zones.yaml')
    replacements = {'__NAV_ZONES_FILE__': zones,
                    '__NAV_BACKUP__': 'false' if mapping else 'true',
                    '__NAV_RETRIES__': '1' if mapping else '-1',
                    '__NAV_PATH_MAX_AGE__': '1' if mapping else '10'}
    bt_xml = str(share / 'behavior_trees' / 'navigate_to_pose.xml')
    bt_xml = ReplaceString(source_file=bt_xml, replacements=replacements)
    through_xml = ReplaceString(
        source_file=str(share / 'behavior_trees' / 'navigate_through_poses.xml'),
        replacements=dict(replacements, __NAV_THROUGH_PLANNER__=(
            'StrictGridBased' if mapping else '{selected_planner}')))
    actions = []
    terrain_file = ''
    if mapping or real_overrides is not None:
        with NamedTemporaryFile(mode='w', prefix='nav_params_', suffix='.yaml', delete=False) as output:
            yaml.safe_dump(document, output)
            params = output.name
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=lambda _: Path(params).unlink(missing_ok=True))])))
    if not mapping:
        for required in ('nav_map.yaml', 'speed_mask.yaml', 'zones.yaml', 'terrain.msgpack'):
            if not (terrain_dir / required).is_file():
                raise RuntimeError(f'Terrain package is missing {required}: {terrain_dir}; re-export it first')
        terrain_file = str(terrain_dir / 'terrain.msgpack')

    def nav2(pkg, exe, name=None, extra=None, remap=None):
        return Node(package=pkg, executable=exe, name=name or exe, output='screen',
                    parameters=[params, dict(common, **(extra or {}))], remappings=remap or [])

    if not mapping:
        actions.extend([
            nav2('nav2_map_server', 'map_server', 'nav_map_server',
                 extra={'yaml_filename': str(terrain_dir / 'nav_map.yaml')}),
            nav2('nav2_map_server', 'map_server', 'speed_mask_server',
                 extra={'yaml_filename': str(terrain_dir / 'speed_mask.yaml')}),
            nav2('nav2_map_server', 'costmap_filter_info_server', 'speed_filter_info_server'),
        ])
    obstacle_params = dict(common, terrain_file=terrain_file, mapping=mapping)
    if mapping:
        obstacle_params.update(base_ground_offset=0.062, max_height=0.45)
    actions.extend([
        Node(package='nav_bringup', executable='goal_visualizer.py',
             name='navigation_goal_visualizer', output='screen', parameters=[common]),
        Node(package='nav_terrain', executable='terrain_obstacles.py', name='terrain_obstacles',
             output='screen', parameters=[params, obstacle_params]),
        nav2('nav2_controller', 'controller_server', remap=[('cmd_vel', 'cmd_vel_nav'),
             ('~/executed_velocity', '/cmd_vel')], extra={
            'FollowPath.zone.zones_file': zones, 'FollowPath.executed_velocity_topic': '~/executed_velocity',
            'FollowPath.reference_curve_accel': float(curve_accel), 'FollowPath.clear_space_reentry': clear_reentry,
            'FollowPath.reentry_timeout': float(reentry_timeout)}),
        nav2('nav2_smoother', 'smoother_server', extra={
            'zone_shaper.zones_file': zones}),
        nav2('nav2_planner', 'planner_server'),
        nav2('nav2_behaviors', 'behavior_server', remap=[('cmd_vel', 'cmd_vel_nav')]),
        nav2(smoother_package, smoother_executable, 'velocity_smoother',
             remap=[('cmd_vel', 'cmd_vel_nav')]),
        nav2('nav2_collision_monitor', 'collision_monitor'),
        nav2('nav2_bt_navigator', 'bt_navigator', extra={
            'default_nav_to_pose_bt_xml': bt_xml,
            'default_nav_through_poses_bt_xml': through_xml}),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[dict(common, autostart=True, bond_timeout=0.0,
                              node_names=([] if mapping else TERRAIN_NODES) + LIFECYCLE_NODES)]),
        Node(package='nav_safety', executable='cmd_gate', name='cmd_gate', output='screen',
             parameters=[params, dict(common, require_trusted=require_trusted)],
             remappings=[('input', 'cmd_vel_collision_checked'), ('output', _get(context, 'cmd_vel_topic'))]),
    ])
    if _get(context, 'rviz').lower() == 'true':
        rviz = share / 'rviz' / 'navigation.rviz'
        actions.append(Node(package='rviz2', executable='rviz2', name='nav_rviz', output='screen',
                            arguments=['-d', str(rviz)] if rviz.is_file() else [],
                            parameters=[common]))
    if serial_goals:
        actions.append(Node(
            package='nav_bringup', executable='send_goal.py', name='serial_navigation_goal',
            output='screen', arguments=['--listen-serial'],
            parameters=[dict(common, robot_base_frame=base_frame, require_trusted=True, mapping=mapping,
                             serial_target_topic=_get(context, 'serial_target_topic'))]))
    elif goal_x:
        actions.append(Node(
            package='nav_bringup', executable='send_goal.py', name='startup_navigation_goal',
            output='screen',
            arguments=[goal_x, goal_y, '--yaw', _get(context, 'goal_yaw'),
                       '--wait-timeout', _get(context, 'goal_wait_timeout')],
            parameters=[dict(common, robot_base_frame=base_frame, require_trusted=require_trusted,
                             mapping=mapping)]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='', description='sim or real; inferred from use_sim_time if omitted'),
        DeclareLaunchArgument('mode', default_value='localization',
                              description='localization (offline terrain) or mapping (live rolling costmaps)'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel', description='final gated chassis Twist topic'),
        DeclareLaunchArgument('terrain_dir', default_value='',
                              description='offline terrain package for the prior map in use '
                                          '(maps/sim or an export folder such as maps/sim/terrain)'),
        DeclareLaunchArgument('params_file', default_value='', description='full Nav2 parameters (replaces profile defaults)'),
        DeclareLaunchArgument('use_sim_time', default_value='', description='default true for sim, false for real'),
        DeclareLaunchArgument('require_trusted', default_value='true',
                              description='require trusted or odometry-extrapolated localization'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('serial_goals', default_value='false', choices=['true', 'false'],
                              description='forward live A4 goals in the real profile'),
        DeclareLaunchArgument('serial_target_topic', default_value='/chassis/nav_target',
                              description='PointStamped target input when serial_goals is enabled'),
        DeclareLaunchArgument('goal_x', default_value='', description='optional startup goal x in map, metres; set with goal_y'),
        DeclareLaunchArgument('goal_y', default_value='', description='optional startup goal y in map, metres; set with goal_x'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0', description='startup goal heading in radians, 0 faces map +X'),
        DeclareLaunchArgument('goal_wait_timeout', default_value='120.0', description='seconds to wait for navigation readiness before giving up'),
        OpaqueFunction(function=build),
    ])
