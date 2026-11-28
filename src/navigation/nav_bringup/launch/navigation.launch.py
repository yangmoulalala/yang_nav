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
import sys
from pathlib import Path
from tempfile import NamedTemporaryFile

import yaml

from ament_index_python.packages import get_package_share_directory
from bringup.configuration import merge
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
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
                       resolution=0.05, plugins=['mapping_ground', 'obstacle_layer', 'inflation_layer'])
        costmap['mapping_ground'] = dict(plugin='nav2_costmap_2d::StaticLayer',
                                         map_topic='/nav/mapping_ground', map_subscribe_transient_local=True,
                                         footprint_clearing_enabled=False)
        costmap.pop('filters', None)
        costmap['obstacle_layer'].update(plugin='nav_behaviors::MappingObstacleLayer',
                                          footprint_clearing_enabled=False)
    global_costmap = document['global_costmap']['global_costmap']['ros__parameters']
    global_costmap['update_frequency'] = max(10.0, global_costmap['update_frequency'])
    planners = document['planner_server']['ros__parameters']
    planners['costmap_update_timeout'] = max(1.0, planners.get('costmap_update_timeout', 1.0))
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
    if follow.get('fixed_frame_translation') is True:
        return 'nav_behaviors', 'fixed_frame_velocity_smoother'
    return 'nav2_velocity_smoother', 'velocity_smoother'


def navigation_summary_recorder(share):
    for workspace in share.parents:
        recorder = workspace / 'tools' / 'record_bag.py'
        if recorder.is_file():
            return ExecuteProcess(
                cmd=[sys.executable, '-u', str(recorder), '--summary', '--navigation-session'],
                cwd=str(workspace), name='navigation_summary_recorder', output='screen',
                sigterm_timeout='60', sigkill_timeout='5')
    return LogInfo(msg='Skipping automatic bag: tools/record_bag.py not found in the workspace.')


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
    landmarks_file = _get(context, 'landmarks_file') or ('' if mapping else str(terrain_dir / 'terrain.json'))
    serial_intents = _get(context, 'serial_intents').lower() or 'true'
    if serial_intents not in ('true', 'false'):
        raise RuntimeError('serial_intents must be true or false')
    # A4 only names a place (home/target), so the matching map landmark is the only goal source.
    # The bridge always runs for real localization; the WebUI switch only decides whether it acts.
    intents_bridge = profile == 'real' and not mapping and not goal_x and not serial_goals
    custom_params = _get(context, 'params_file')
    params_dir = share / 'params'
    params = custom_params or str(params_dir / 'nav2.yaml')
    real_overrides = params_dir / 'nav2_real.yaml' if profile == 'real' and not custom_params else None
    document = navigation_params(params, real_overrides, mapping)
    smoother_package, smoother_executable = velocity_smoother_executable(profile, document)
    follow = document.get('controller_server', {}).get('ros__parameters', {}).get('FollowPath', {})
    curve_accel = 0.0
    reentry_timeout = 0.0
    if follow.get('fixed_frame_translation') and follow.get('path_departure_limit', 0.) > 0.:
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
                    '__NAV_GLOBAL_RECOVERY__': ('Wait wait_duration="0.5"' if mapping else
                        'ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"'),
                    '__NAV_LOCAL_RECOVERY__': ('Wait wait_duration="0.5"' if mapping else
                        'ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"'),
                    '__NAV_BACKUP__': 'false' if mapping else 'true',
                    '__NAV_RETRIES__': '3' if mapping else '-1',
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
    if profile == 'real' and _get(context, 'auto_bag') == 'true':
        actions.append(navigation_summary_recorder(share))

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
             ('~/executed_velocity', _get(context, 'cmd_vel_topic'))], extra={
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
    if intents_bridge:
        actions.append(Node(
            package='nav_bringup', executable='send_goal.py', name='serial_intent_goal',
            output='screen',
            arguments=['--listen-intents', '--landmarks-file', landmarks_file],
            parameters=[dict(common, robot_base_frame=base_frame, require_trusted=True, mapping=mapping,
                             intent_topic=_get(context, 'serial_intent_topic'),
                             enabled_topic=_get(context, 'serial_intents_topic'),
                             enabled=serial_intents == 'true',
                             landmarks_file=landmarks_file)]))
    elif serial_goals:
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
        DeclareLaunchArgument('auto_bag', default_value='true', choices=['true', 'false'],
                              description='record a lightweight ROS bag for the real navigation session'),
        DeclareLaunchArgument('require_trusted', default_value='true',
                              description='require trusted or odometry-extrapolated localization'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('serial_intents', default_value='true', choices=['true', 'false'],
                              description='initial state of the WebUI 串口意图导航 switch; '
                                          'the bridge always starts on the real profile'),
        DeclareLaunchArgument('serial_intents_topic', default_value='/navigation/serial_intents_enabled',
                              description='retained Bool the WebUI publishes to enable intent goals'),
        DeclareLaunchArgument('serial_intent_topic', default_value='/chassis/nav_intent',
                              description='String intent topic (home or target) when serial_intents is enabled'),
        DeclareLaunchArgument('landmarks_file', default_value='',
                              description='annotated terrain.json with home/target map landmarks; '
                                          'defaults to <terrain_dir>/terrain.json'),
        DeclareLaunchArgument('serial_goals', default_value='false', choices=['true', 'false'],
                              description='legacy PointStamped input only, not A4 home/target intents'),
        DeclareLaunchArgument('serial_target_topic', default_value='/chassis/nav_target',
                              description='PointStamped target input when serial_goals is enabled'),
        DeclareLaunchArgument('goal_x', default_value='', description='optional startup goal x in map, metres; set with goal_y'),
        DeclareLaunchArgument('goal_y', default_value='', description='optional startup goal y in map, metres; set with goal_x'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0', description='startup goal heading in radians, 0 faces map +X'),
        DeclareLaunchArgument('goal_wait_timeout', default_value='120.0', description='seconds to wait for navigation readiness before giving up'),
        OpaqueFunction(function=build),
    ])
