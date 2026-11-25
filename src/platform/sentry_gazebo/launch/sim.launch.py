"""RMUL 2027 sentry simulation: Gazebo Harmonic + sensors + ROS 2 glue.

    ros2 launch sentry_gazebo sim.launch.py                  # headless
    ros2 launch sentry_gazebo sim.launch.py gui:=true        # + Gazebo GUI
    ros2 launch sentry_gazebo sim.launch.py gui:=true rviz:=true
    ros2 launch sentry_gazebo sim.launch.py gui:=true arena:=rmuc   # RMUC full field
    ros2 launch sentry_gazebo sim.launch.py arena:=rmuc team:=red   # red spawn

The physical model uses the full shared URDF. ROS exposes the same rigid
gimbal_nav sensor subtree and turret-frame XY interface as the real robot.
Physical joint state and actuator topics stay under /sim; ground truth is not
used by localization. See README.md for topics and simulator controls.
"""
import shlex
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sentry_gazebo import arena_spec as A, prepare_world, sdf_builder
from sentry_gazebo.launch_utils import gz_environment, generated_dir, urdf_path


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in ('1', 'true', 'yes', 'on')


def _str(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def build(context):
    share = Path(get_package_share_directory('sentry_gazebo'))
    gui = _bool(context, 'gui')
    rviz = _bool(context, 'rviz')
    gen = Path(_str(context, 'generated_dir') or generated_dir())
    urdf = Path(urdf_path())
    offset = float(_str(context, 'gimbal_yaw_offset'))
    generator = Path(get_package_share_directory('sentry_description')).parents[1] / \
        'lib/sentry_description/gimbal_description.py'
    description = subprocess.check_output([
        'python3', str(generator), str(urdf), f'--yaw-offset={offset}'], text=True)

    arena = _str(context, 'arena') or 'rmul'
    # own colour: spawn in that half (point-symmetric field); empty keeps the
    # arena's own spawn (rmul red, rmuc blue)
    team = _str(context, 'team').lower() or sdf_builder.ARENAS[arena]['spawn_team']
    world = prepare_world.generate(gen, urdf, view=gui, arena=arena, team=team)

    actions = []
    for key, value in gz_environment(gen).items():
        actions.append(SetEnvironmentVariable(key, value))

    # ---------------------------------------------------------------- Gazebo
    # -r      run immediately (do not start paused)
    # -s      server only, without -s the GUI opens as well
    # --headless-rendering keeps the gpu_lidar sensors rendering without a window
    gz_args = ['-r', '-v', '3']
    if not gui:
        gz_args += ['-s', '--headless-rendering']
    gz_args.append(str(world))
    ros_gz_sim_launch = Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py'
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_sim_launch)),
        launch_arguments={'gz_args': ' '.join(shlex.quote(a) for a in gz_args),
                          'on_exit_shutdown': 'true'}.items()))

    # ---------------------------------------------------------- ros gz bridge
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='gz_bridge', output='screen',
                        parameters=[{'config_file': str(share / 'config' / 'bridge.yaml'),
                                     'use_sim_time': True}]))

    # ------------------------------------------------------------------- TF
    # The same observable rigid tree as real.launch.py. No synthetic chassis TF.
    actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                        name='robot_state_publisher', output='screen',
                        parameters=[{'robot_description': description,
                                     'use_sim_time': True}]))
    if _bool(context, 'chassis'):
        actions.append(Node(package='sentry_gazebo', executable='chassis_sim', output='screen',
                            parameters=[{'use_sim_time': True, 'gimbal_yaw_offset': offset}]))

    # gz stamps sensor messages with its scoped name
    # ("sentry/mid360_link/mid360_lidar") and SDFormat 1.9 has no gz_frame_id to
    # override it, so alias each scoped frame onto its link with an identity.
    for sensor, (link, frame) in A.SENSOR_FRAMES.items():
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='alias_%s' % sensor, output='screen',
            arguments=['--frame-id', frame,
                       '--child-frame-id', '%s/%s/%s' % (A.ROBOT_MODEL_NAME, link, sensor)],
            parameters=[{'use_sim_time': True}]))

    # referee-system UWB (0x0203): 1 Hz noisy position in the referee field
    # frame on /uwb_pose, synthesised from the gz-only ground-truth pose
    # only on the RMUC field: RMUL has no referee UWB
    if _bool(context, 'uwb') and arena == 'rmuc':
        actions.append(Node(package='sentry_gazebo', executable='uwb_sim', name='uwb_sim',
                            output='screen',
                            parameters=[{'use_sim_time': True, 'arena': arena, 'team': team,
                                         'gimbal_yaw_offset': offset}]))

    if rviz:
        rviz_cfg = share / 'rviz' / 'sentry.rviz'
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            arguments=(['-d', str(rviz_cfg)] if rviz_cfg.is_file() else [])
                            + ['-f', 'map'],
                            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='open the Gazebo GUI as well'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='start RViz2 with the sentry layout'),
        DeclareLaunchArgument('gimbal_yaw_offset', default_value='0.0',
                              description='calibrated MCU +X offset from CAD turret zero (rad)'),
        DeclareLaunchArgument('chassis', default_value='true',
                              description='MCU interface simulator; false only for direct sensor regression'),
        DeclareLaunchArgument('arena', default_value='rmul', choices=['rmul', 'rmuc'],
                              description='rmul: RMUL 2027 3v3 field (model://rmul_2027); '
                                          'rmuc: RMUC 2027 full field (model://rmuc_2027)'),
        DeclareLaunchArgument('uwb', default_value='true',
                              description='run the referee UWB (0x0203) simulator on /uwb_pose '
                                          '(arena:=rmuc only; ignored on rmul)'),
        DeclareLaunchArgument('team', default_value='',
                              description='own colour red|blue: spawn in that half and report '
                                          'it on /uwb_pose/team (default: the arena spawn, '
                                          'rmul red / rmuc blue)'),
        DeclareLaunchArgument('generated_dir', default_value='',
                              description='where to write the generated world '
                                          '(default: $SENTRY_GAZEBO_GEN or '
                                          '/tmp/sentry_gazebo_generated)'),
        OpaqueFunction(function=build),
    ])
