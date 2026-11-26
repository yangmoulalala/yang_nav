from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import time
from contextlib import suppress

from ament_index_python.packages import PackageNotFoundError, get_package_prefix

from .settings import (WORKSPACE, bag_request, flag, map_output, number, path, prior_path,
                       terrain_path, workspace_path)


PROCESSES = ('gazebo', 'localization', 'navigation', 'bag', 'optimizer', 'goal')
OWNERS = {'gazebo': ('/gz_bridge',), 'localization': ('/backend_node', '/zlio', '/livox_lidar_publisher'),
          'navigation': ('/bt_navigator', '/cmd_gate'),
          'bag': ('/rosbag2_recorder',)}
LABELS = dict(gazebo='Gazebo', localization='定位 / 建图', navigation='Nav2',
              bag='录包', optimizer='地图优化', goal='网页导航目标')


class ManagedProcess:
    def __init__(self, command, log, parameters, output=None, environment=None):
        self.log = log
        self.parameters = parameters
        self.output = output
        self.stopping = False
        self.group_finished = False
        with log.open('ab') as stream:
            stream.write(f'\n[{datetime.now().isoformat(timespec="seconds")}] 进程启动\n'.encode())
            stream.flush()
            self.child = subprocess.Popen(command, cwd=WORKSPACE, stdin=subprocess.DEVNULL,
                                          env=dict(os.environ) | {'PYTHONUNBUFFERED': '1'} | (environment or {}),
                                          stdout=stream, stderr=subprocess.STDOUT,
                                          start_new_session=True)

    def status(self):
        code = self.child.poll()
        if code is None:
            return 'stopping' if self.stopping else 'running'
        return 'stopped' if code == 0 or (self.stopping and code in (130, -signal.SIGINT)) else 'failed'

    def stop(self, timeout, signal_group=False):
        if self.child.poll() is not None:
            if timeout:
                self.finish_group()
            return
        if not self.stopping:
            self.stopping = True
            if signal_group:
                with suppress(ProcessLookupError):
                    os.killpg(self.child.pid, signal.SIGINT)
            else:
                self.child.send_signal(signal.SIGINT)
        if not timeout:
            return
        try:
            self.child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            with suppress(ProcessLookupError):
                os.killpg(self.child.pid, signal.SIGTERM)
            try:
                self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                with suppress(ProcessLookupError):
                    os.killpg(self.child.pid, signal.SIGKILL)
                self.child.wait()
        self.finish_group()

    def finish_group(self):
        if self.group_finished:
            return
        self.group_finished = True
        with suppress(ProcessLookupError):
            os.killpg(self.child.pid, signal.SIGTERM)
            time.sleep(0.5)
            os.killpg(self.child.pid, signal.SIGKILL)


class Processes:
    def __init__(self, settings, ros, directory):
        self.settings = settings
        self.ros = ros
        self.directory = directory
        self.entries = {}

    def alive(self, name):
        process = self.entries.get(name)
        return process is not None and process.child.poll() is None

    def describe(self):
        nodes = self.ros.node_names()
        data = {}
        for name in PROCESSES:
            process = self.entries.get(name)
            if process and process.child.poll() is None:
                state = process.status()
            elif nodes.intersection(OWNERS.get(name, ())):
                state = 'external'
            else:
                state = process.status() if process else 'stopped'
            if name == 'optimizer' and state == 'failed' and process.stopping:
                state = 'cancelled'
            data[name] = dict(state=state, output=process.output if process else None,
                              code=process.child.poll() if process else None)
            if name == 'localization' and state == 'running':
                data[name]['mode'] = process.parameters['mode']
        return data

    def launch(self, name, command, parameters=None, output=None, environment=None):
        self.entries[name] = ManagedProcess(command, self.directory / f'{name}.log', parameters,
                                            output, environment)

    def verify_start(self, name):
        if self.alive(name):
            raise ValueError(f'{LABELS[name]} 已启动或正在关闭')
        previous = self.entries.get(name)
        if previous and name != 'bag':
            previous.finish_group()
        if self.ros.node_names().intersection(OWNERS.get(name, ())):
            raise ValueError(f'当前 ROS 域里已运行 {LABELS[name]}，请使用原终端管理该进程')

    def start(self, name, payload):
        self.verify_start(name)
        profile, config = self.settings.profile, self.settings.active
        simulation = profile == 'sim'
        if name == 'gazebo':
            if not simulation:
                raise ValueError('Gazebo 仅用于仿真配置')
            command = ['ros2', 'launch', 'sentry_gazebo', 'sim.launch.py',
                       f'arena:={config["arena"]}', f'gui:={str(config["gui"]).lower()}',
                       f'uwb:={str(config["uwb"]).lower()}',
                       f'generated_dir:={self.directory / "gazebo"}']
            if config['team'] != 'auto':
                command.append(f'team:={config["team"]}')
            domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
            self.launch(name, command, config, environment={'GZ_PARTITION': f'yang_nav_web_{domain}'})
        elif name == 'localization':
            command = ['ros2', 'launch', 'bringup', f'{profile}.launch.py',
                       f'mode:={config["mode"]}', f'lidar:={config["lidar"]}']
            if config['mode'] == 'mapping':
                folder = map_output(config['map_dir'])
                command.append(f'map_dir:={folder}')
            else:
                folder = prior_path(config['prior_map'])
                command.append(f'prior_map:={folder}')
            if not simulation:
                port = config['chassis_port']
                if not port.startswith('/dev/') or not Path(port).is_char_device():
                    raise ValueError(f'找不到底盘串口 {port or "/dev/nav"}')
                command.append(f'chassis_port:={port}')
                if config['lidar'] == 'dual' and config['l2_launch']:
                    l2 = path(config['l2_launch'])
                    if not l2.is_file() or not l2.name.endswith('.launch.py'):
                        raise ValueError('L2 驱动需要有效的 .launch.py 文件')
                    command.extend(('driver_l2:=true', f'l2_launch:={l2}'))
            if config['tuning']:
                params = self.directory / 'backend_overrides.yaml'
                self.settings.overrides(name, params)
                command.append(f'backend_overrides:={params}')
            self.launch(name, command, config, str(folder))
            return
        elif name == 'navigation':
            loc = self.entries.get('localization')
            if config['mode'] != 'localization' or (self.alive('localization') and
                                                    loc.parameters['mode'] != 'localization'):
                raise ValueError('Nav2 需要使用先验地图的匹配模式')
            if not self.alive('localization') and '/backend_node' not in self.ros.node_names():
                raise ValueError('请先启动定位')
            self.wait_for_localization()
            prior = self.entries['localization'].output if self.alive('localization') else config['prior_map']
            folder = terrain_path(config['terrain_dir'] or prior)
            command = ['ros2', 'launch', 'nav_bringup', 'navigation.launch.py',
                       f'profile:={profile}', f'terrain_dir:={folder}']
            if config['tuning']:
                params = self.directory / 'nav2.yaml'
                self.settings.overrides(name, params)
                command.append(f'params_file:={params}')
            self.launch(name, command, config, str(folder))
            return
        elif name == 'bag':
            bag_name, options = bag_request(payload)
            bag_name = bag_name or datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            output = WORKSPACE / 'bags' / 'recordings' / bag_name
            if output.exists():
                raise ValueError('录包目录已经存在，请更换名称')
            command = ['python3', 'tools/record_bag.py', '--output', str(output), *options]
            if simulation:
                command.append('--use-sim-time')
            self.launch(name, command, payload, str(output))
            return
        elif name == 'optimizer':
            source = prior_path(str(workspace_path(payload.get('source'), '输入地图')))
            if self.active_mapping() and source == Path(self.entries['localization'].output):
                raise ValueError('请保存并停止当前建图，再优化该地图')
            output = map_output(payload.get('output'))
            if output.is_relative_to(source) or source.is_relative_to(output):
                raise ValueError('优化输出与输入地图目录不能重叠')
            threads = number(payload.get('threads', 4), 1, 32, '线程数')
            if type(threads) is not int:
                raise ValueError('线程数需要整数')
            origin = payload.get('origin')
            if origin is not None and (not isinstance(origin, list) or len(origin) != 3):
                raise ValueError('雷达光心需要 X、Y、Z 三个坐标')
            command = ['bash', 'tools/optimize_map.sh', '--in', str(source), '--out', str(output),
                       '--overwrite', '--threads', str(threads)]
            if not flag(payload.get('raycasting', False), '射线动态过滤'):
                command.append('--no-raycasting')
            elif origin is not None:
                command.extend(['--lidar-origin', *(str(number(value, -5, 5, '雷达光心')) for value in origin)])
            if not flag(payload.get('mls', True), 'MLS 平滑'):
                command.append('--no-mls')
            try:
                prefix = Path(get_package_prefix('offline_mapping_optimizer'))
            except PackageNotFoundError as exc:
                raise ValueError('请先编译 offline_mapping_optimizer') from exc
            if not (prefix / 'lib/offline_mapping_optimizer/offline_mapping_optimizer_node').is_file():
                raise ValueError('请先编译 offline_mapping_optimizer')
            self.launch(name, command, payload, str(output))
            return
        else:
            raise ValueError('无效的进程')

    def wait_for_localization(self):
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            state = self.ros.snapshot()
            if state.get('localization', {}).get('pose_trusted') == 'true' and state.get('pose'):
                return
            process = self.entries.get('localization')
            if process and process.status() == 'failed':
                raise ValueError('定位进程已退出，请查看定位日志')
            time.sleep(0.25)
        raise ValueError('定位未得到可信位姿，Nav2 尚未启动。请查看定位日志。')

    def send_goal(self, payload):
        if self.active_mapping() or self.settings.active['mode'] == 'mapping':
            raise ValueError('建图时无法发送导航目标')
        if self.ros.snapshot().get('nav2') != 'active':
            raise ValueError('Nav2 尚未激活')
        x = number(payload.get('x'), -10000, 10000, 'X')
        y = number(payload.get('y'), -10000, 10000, 'Y')
        if self.alive('goal'):
            raise ValueError('已有网页目标正在执行，请先取消')
        executable = Path(get_package_prefix('nav_bringup')) / 'lib/nav_bringup/send_goal.py'
        clock = str(self.settings.profile == 'sim').lower()
        command = [str(executable), str(x), str(y), '--wait-timeout', '15',
                   '--ros-args', '-p', f'use_sim_time:={clock}']
        self.launch('goal', command)

    def active_mapping(self):
        return self.alive('localization') and self.entries['localization'].parameters['mode'] == 'mapping'

    def stop(self, name):
        process = self.entries.get(name)
        if not process:
            return
        if name == 'navigation':
            self.stop('goal')
        if name == 'gazebo' and self.alive('localization'):
            self.stop('localization')
        if name in ('gazebo', 'localization') and self.alive('navigation'):
            self.stop('navigation')
        if name == 'bag':
            process.stop(0)
        elif name == 'optimizer':
            process.stop(16, signal_group=True)
        else:
            process.stop(16)

    def start_stack(self):
        config = self.settings.active
        if self.alive('localization') and self.entries['localization'].parameters['mode'] != config['mode']:
            raise ValueError('定位仍在使用上一次的模式，请先重启定位')
        if config['mode'] == 'localization':
            prior_path(config['prior_map'])
            terrain_path(config['terrain_dir'] or config['prior_map'])
        targets = ['localization']
        if self.settings.profile == 'sim':
            targets.insert(0, 'gazebo')
        launched = []
        try:
            for name in targets:
                if self.alive(name) or self.ros.node_names().intersection(OWNERS[name]):
                    continue
                self.start(name, {})
                launched.append(name)
                time.sleep(0.25)
                if not self.alive(name):
                    raise ValueError(f'{LABELS[name]} 退出，请查看进程日志')
        except Exception:
            for name in reversed(launched):
                self.stop(name)
            raise
        if (config['mode'] == 'localization' and not self.alive('navigation') and
                not self.ros.node_names().intersection(OWNERS['navigation'])):
            self.start('navigation', {})

    def stop_stack(self):
        self.stop('bag')
        if self.alive('bag'):
            try:
                self.entries['bag'].child.wait(timeout=180)
            except subprocess.TimeoutExpired as exc:
                raise ValueError('录包仍在收尾，请查看日志，导航和定位保持运行') from exc
        for name in ('goal', 'navigation', 'localization', 'gazebo'):
            self.stop(name)

    def restart(self, name, payload):
        if name == 'bag':
            raise ValueError('停止录包并等待收尾，再用新的目录名开始录制')
        if name == 'optimizer':
            raise ValueError('请在离线优化页重新选定输出路径，再启动优化')
        if name == 'localization' and self.settings.active['mode'] == 'mapping':
            map_output(self.settings.active['map_dir'])
        restart_nav = name in ('gazebo', 'localization') and self.alive('navigation')
        restart_loc = name == 'gazebo' and self.alive('localization')
        if restart_loc:
            if self.active_mapping():
                raise ValueError('请先保存地图并关闭建图，再重启 Gazebo')
            self.stop('localization')
        self.stop(name)
        targets = [name] + (['localization'] if restart_loc else []) + (['navigation'] if restart_nav else [])
        for target in targets:
            deadline = time.monotonic() + 5
            while self.ros.node_names().intersection(OWNERS.get(target, ())) and time.monotonic() < deadline:
                time.sleep(0.15)
            self.start(target, payload)

    def log(self, name):
        file = self.directory / f'{name}.log'
        if not file.is_file():
            return ''
        with file.open('rb') as stream:
            stream.seek(-min(file.stat().st_size, 28000), os.SEEK_END)
            return stream.read(28000).decode('utf-8', errors='replace')
