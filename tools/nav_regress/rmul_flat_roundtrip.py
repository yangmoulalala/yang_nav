"""Isolated RMUL navigation bench: fixed LiDAR carrier, simulated wheels, no serial or joint commands."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from sentry_gazebo import arena_spec, prepare_world
from sentry_gazebo.launch_utils import gz_environment, urdf_path


ROOT = Path(__file__).resolve().parents[2]
HOME = (6.0, 1.5)
CENTER = (6.0, 4.0)
DRIVE = '/rmul_lab/wheels'
MODEL = 'navigation_cart'
TRUTH_TOLERANCE = 0.25


def field_to_map(point, field_origin, map_origin, heading):
    delta_x, delta_y = (value - origin for value, origin in zip(point, field_origin))
    return (map_origin[0] + math.cos(heading) * delta_x - math.sin(heading) * delta_y,
            map_origin[1] + math.sin(heading) * delta_x + math.cos(heading) * delta_y)


def fixture(folder, spawn_yaw):
    source = prepare_world.generate(folder, urdf_path(), arena='rmul')
    doc = ET.parse(source)
    robot = doc.find(f'.//world/model[@name="{arena_spec.ROBOT_MODEL_NAME}"]')
    robot.set('name', MODEL)
    robot.find('pose').text = f'{HOME[0]} {HOME[1]} {arena_spec.ROBOT_Z} 0 0 {spawn_yaw}'
    joint = robot.find('joint[@name="yaw_joint"]')
    joint.set('type', 'fixed')
    joint.remove(joint.find('axis'))
    for visual in robot.findall('link[@name="yaw_link"]/visual'):
        robot.find('link[@name="yaw_link"]').remove(visual)
    for plugin in robot.findall('plugin'):
        if plugin.get('filename') not in ('gz-sim-mecanum-drive-system', 'gz-sim-pose-publisher-system'):
            robot.remove(plugin)
    wheels = robot.find('plugin[@filename="gz-sim-mecanum-drive-system"]')
    wheels.find('topic').text = DRIVE
    doc.write(source)
    bridge_source = Path(get_package_share_directory('sentry_gazebo')) / 'config/bridge.yaml'
    bridge = [row for row in yaml.safe_load(bridge_source.read_text()) if
              row['ros_topic_name'] == '/clock' or row['ros_topic_name'].startswith('/lidar_')]
    bridge.append(dict(ros_topic_name=DRIVE, gz_topic_name=DRIVE, ros_type_name='geometry_msgs/msg/Twist',
                       gz_type_name='gz.msgs.Twist', direction='ROS_TO_GZ'))
    (folder / 'bridge.yaml').write_text(yaml.safe_dump(bridge))
    generator = Path(get_package_share_directory('sentry_description')).parents[1] / 'lib/sentry_description/gimbal_description.py'
    description = subprocess.check_output([sys.executable, str(generator), str(urdf_path())], text=True)
    (folder / 'rsp.yaml').write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
        'robot_description': description, 'use_sim_time': True}}}))
    return source


def measure(mode, out, registration, nav_start, known_dock, spawn_yaw):
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node as GzNode
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    from rclpy.time import Time
    from action_msgs.msg import GoalStatus

    sys.path.insert(0, str(ROOT / 'src/navigation/nav_bringup/scripts'))
    from send_goal import GoalSender

    trace = []
    gz_node = GzNode()

    def truth(message):
        if message.name == MODEL:
            trace.append((time.monotonic(), message.position.x, message.position.y))

    gz_node.subscribe(Pose, f'/model/{MODEL}/pose', truth)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', f'mapping:={str(mode == "mapping").lower()}'])
    sender = GoalSender()
    report = dict(mode=mode, world_start=HOME, world_center=CENTER, truth_tolerance_m=TRUTH_TOLERANCE,
                  known_dock=known_dock, spawn_yaw_deg=math.degrees(spawn_yaw), legs=[], passed=False)

    def estimated():
        transform = sender.tf_buffer.lookup_transform('map', sender.base_frame, Time()).transform
        rot = transform.rotation
        heading = math.atan2(2 * (rot.w * rot.z + rot.x * rot.y), 1 - 2 * (rot.y ** 2 + rot.z ** 2))
        return (transform.translation.x, transform.translation.y), heading

    try:
        deadline = time.monotonic() + 80
        while time.monotonic() < deadline:
            rclpy.spin_once(sender, timeout_sec=0.1)
            if trace and sender.trusted and sender.tf_buffer.can_transform('map', sender.base_frame, Time()):
                break
        else:
            raise RuntimeError(f'Initial localization/truth timeout: trusted={sender.trusted}, truth_samples={len(trace)}')
        nav_start()
        if not sender.wait_ready(70):
            raise RuntimeError('Navigation readiness timeout')
        runtime = {}
        for node, keys in {
            'controller_server': ['use_sim_time', 'FollowPath.fixed_frame_translation', 'FollowPath.vx_max',
                                  'FollowPath.ax_max', 'FollowPath.path_departure_limit', 'goal_checker.xy_goal_tolerance'],
            'velocity_smoother': ['use_sim_time', 'max_velocity', 'max_accel', 'max_decel'],
            'cmd_gate': ['use_sim_time', 'require_trusted', 'require_observation', 'limits.normal.max_linear'],
            'local_costmap/local_costmap': ['inflation_layer.inflation_radius'],
        }.items():
            client = sender.create_client(GetParameters, f'/{node}/get_parameters')
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError(f'Missing parameter service: {node}')
            pending = client.call_async(GetParameters.Request(names=keys))
            rclpy.spin_until_future_complete(sender, pending, timeout_sec=5)
            if not pending.done():
                raise RuntimeError(f'Parameter readback timeout: {node}')
            runtime[node] = dict(zip(keys, map(parameter_value_to_python, pending.result().values)))
        (out / 'effective.json').write_text(json.dumps(runtime, indent=2) + '\n')
        if not all(runtime['cmd_gate'][key] for key in ('use_sim_time', 'require_trusted', 'require_observation')):
            raise RuntimeError('Simulation clock or navigation safety gates not enabled')
        start_pose, start_heading = estimated()
        report.update(start_map=start_pose, start_heading_rad=start_heading, start_truth=trace[-1][1:])
        if math.dist(trace[-1][1:], HOME) > TRUTH_TOLERANCE:
            raise RuntimeError('Simulator did not start at the specified home')
        if mode == 'mapping':
            map_goals = [field_to_map(CENTER, HOME, start_pose, start_heading - spawn_yaw), start_pose]
        else:
            heading = -math.radians(registration['rpy_deg'][2])
            map_goals = [field_to_map(point, registration['t_world_map'][:2], (0, 0), heading) for point in (CENTER, HOME)]
            report['start_registration_error_m'] = math.dist(start_pose, map_goals[1])
            report['start_heading_error_deg'] = math.degrees(math.remainder(start_heading - heading - spawn_yaw, 2 * math.pi))
            if report['start_registration_error_m'] > TRUTH_TOLERANCE or abs(report['start_heading_error_deg']) > 15:
                raise RuntimeError('Prior-map pose disagrees with registered start; no goal sent')
        for target, map_goal in zip((CENTER, HOME), map_goals):
            if not sender.wait_ready(12):
                raise RuntimeError('Lost readiness between goals')
            leg = dict(field_goal=target, map_goal=map_goal, passed=False)
            report['legs'].append(leg)
            begin = time.monotonic()
            request = sender.action.send_goal_async(sender.make_goal(*map_goal, yaw=start_heading))
            rclpy.spin_until_future_complete(sender, request, timeout_sec=8)
            if not request.done() or not request.result().accepted:
                raise RuntimeError('Goal not accepted')
            sender.goal_handle = request.result()
            result = sender.goal_handle.get_result_async()
            rclpy.spin_until_future_complete(sender, result, timeout_sec=90)
            if not result.done():
                sender.cancel()
                raise RuntimeError('Goal timeout; cancellation requested')
            settled = time.monotonic() + 0.8
            while time.monotonic() < settled:
                rclpy.spin_once(sender, timeout_sec=0.05)
            if time.monotonic() - trace[-1][0] > 1.0:
                raise RuntimeError('Stale Gazebo truth')
            segment = [sample for sample in trace if sample[0] >= begin]
            if not segment:
                raise RuntimeError('Missing Gazebo leg samples')
            actual = segment[-1][1:]
            leg.update(action_status=result.result().status, error_code=result.result().result.error_code,
                       duration_wall_s=round(time.monotonic() - begin, 2), estimated_map=estimated()[0],
                       truth_world=actual, error_m=math.dist(actual, target), samples=len(segment),
                       flat_corridor=all(5.0 < sample[1] < 7.0 and 1.0 < sample[2] < 4.5 for sample in segment))
            leg['passed'] = (leg['action_status'] == GoalStatus.STATUS_SUCCEEDED and leg['error_code'] == 0 and
                             leg['error_m'] <= TRUTH_TOLERANCE and leg['flat_corridor'])
            print(json.dumps(leg), flush=True)
            if not leg['passed']:
                raise RuntimeError('Leg failed acceptance; stopping before issuing another goal')
        if mode == 'mapping':
            from std_srvs.srv import Trigger
            service = sender.create_client(Trigger, '/backend_node/save_map')
            if not service.wait_for_service(timeout_sec=4):
                raise RuntimeError('Missing map save service')
            saved = service.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(sender, saved, timeout_sec=15)
            if not saved.done() or not saved.result().success:
                raise RuntimeError('Failed to save the live map')
            bundle = out / 'map'
            sizes = {name: (bundle / name).stat().st_size for name in ('map.pcd', 'keyframes.txt', 'sc_descriptors.bin')}
            if not all(sizes.values()) or not any((bundle / 'clouds').glob('*.pcd')):
                raise RuntimeError('Saved live map is incomplete')
            report['map_bundle_bytes'] = sizes
        report['passed'] = True
    except Exception as exc:
        report['failure'] = str(exc)
    finally:
        gz_node.unsubscribe(f'/model/{MODEL}/pose')
        (out / 'truth.csv').write_text('monotonic_s,x_m,y_m\n' + ''.join(f'{stamp},{pos_x},{pos_y}\n' for stamp, pos_x, pos_y in trace))
        (out / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report), flush=True)
        sender.destroy_node()
        rclpy.try_shutdown()
    return report['passed']


def run(args):
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    domain = 143 if args.mode == 'mapping' else 144
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='',
                      GZ_PARTITION=f'flat_lab_{os.getpid()}', GZ_IP='127.0.0.1', ROS_LOG_DIR=str(out / 'ros'),
                      OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='4')
    (out / 'ros').mkdir()
    import rclpy
    rclpy.init()
    guard = rclpy.create_node('rmul_lab_preflight')
    try:
        time.sleep(1.2)
        if set(guard.get_node_names()) - {guard.get_name()}:
            raise RuntimeError(f'ROS domain {domain} is already in use; refusing to start')
    finally:
        guard.destroy_node()
        rclpy.shutdown()
    sys.path.insert(0, str(ROOT / 'tools'))
    from web_ui.settings import Settings
    settings = Settings(ROOT / 'log/web_ui/domain_0')
    settings.data['profile'] = 'real'
    settings.overrides('navigation', out / 'nav2.yaml')
    settings.overrides('localization', out / 'backend.yaml')
    overrides = yaml.safe_load((out / 'backend.yaml').read_text())
    backend = overrides['backend_node']['ros__parameters']
    backend['uwb'] = {'enabled': False}
    registration = None
    if args.mode == 'localization':
        info = json.loads((args.terrain / 'terrain.json').read_text())
        registration = info['registration']
        if 'RMUL' not in registration['reference'].upper() or Path(info['source']).resolve() != args.prior.resolve():
            raise ValueError('Expected the paired RMUL prior map and terrain registration')
        backend.setdefault('recovery', {})['global_fallback_candidates'] = 15
        if args.known_dock:
            heading = -math.radians(registration['rpy_deg'][2])
            dock_x, dock_y = field_to_map(HOME, registration['t_world_map'][:2], (0, 0), heading)
            backend['startup_hint'] = dict(enabled=True, force_until_matched=True, x=dock_x, y=dock_y,
                                           z=arena_spec.ROBOT_Z - registration['t_world_map'][2], yaw_deg=math.degrees(heading) + args.spawn_yaw)
    (out / 'backend.yaml').write_text(yaml.safe_dump(overrides))
    spawn_yaw = math.radians(args.spawn_yaw)
    source = fixture(out, spawn_yaw)
    os.environ.update(gz_environment(out))
    children = []

    def start(name, command):
        with (out / f'{name}.log').open('w') as stream:
            children.append(subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))

    loc = ['ros2', 'launch', 'bringup', 'sim.launch.py', f'mode:={args.mode}', f'lidar:={settings.active["lidar"]}',
           f'map_dir:={out}/map', f'backend_overrides:={out}/backend.yaml']
    nav = ['ros2', 'launch', 'nav_bringup', 'navigation.launch.py', 'profile:=sim', 'use_sim_time:=true',
           f'mode:={args.mode}', f'params_file:={out}/nav2.yaml', f'cmd_vel_topic:={DRIVE}']
    if args.mode == 'localization':
        loc.append(f'prior_map:={args.prior.resolve()}')
        nav.append(f'terrain_dir:={args.terrain.resolve()}')
    else:
        loc.append('save_interval:=5')
    try:
        start('gazebo', ['gz', 'sim', '-r', '-s', '--headless-rendering', str(source)])
        start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args', '-p', f'config_file:={out}/bridge.yaml', '-p', 'use_sim_time:=true'])
        start('rsp', ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args', '--params-file', str(out / 'rsp.yaml')])
        for sensor, (link, frame) in arena_spec.SENSOR_FRAMES.items():
            start(sensor, ['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', frame,
                           '--child-frame-id', f'{MODEL}/{link}/{sensor}', '--ros-args', '-r', f'__node:=alias_{sensor}', '-p', 'use_sim_time:=true'])
        start('localization', loc)
        return measure(args.mode, out, registration, lambda: start('navigation', nav), args.known_dock, spawn_yaw)
    finally:
        for sig, delay in ((signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 0)):
            for process in children:
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
            time.sleep(delay)
        for process in children:
            process.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('mapping', 'localization'))
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--known-dock', action='store_true', help='Use the registered start station as a prior for geometric matching (localization only)')
    parser.add_argument('--spawn-yaw', type=int, choices=(0, -90), default=0, help='Physical stand orientation in field degrees')
    parser.add_argument('--prior', type=Path, default=ROOT / 'maps/sim/new_session_optimized')
    parser.add_argument('--terrain', type=Path, default=ROOT / 'maps/sim/new_session_optimized/annotation-before-reference-picker_terrain')
    arguments = parser.parse_args()
    if arguments.known_dock and arguments.mode != 'localization':
        parser.error('--known-dock is only valid in localization mode')
    sys.exit(0 if run(arguments) else 1)
