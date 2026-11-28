import os
from pathlib import Path
import re
from tempfile import NamedTemporaryFile

from ament_index_python.packages import get_package_share_directory
from bringup.configuration import LIDAR_MIN_RANGE_BOUNDS, merge, resolve
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
TERRAIN_FILES = ('nav_map.yaml', 'speed_mask.yaml', 'zones.yaml', 'terrain.msgpack')
NAV_PARAMS = Path(get_package_share_directory('nav_bringup')) / 'params' / 'nav2.yaml'
LOC_CONFIG = Path(get_package_share_directory('bringup')) / 'config'

TUNING = {
    'cruise': ('navigation', '普通地形速度上限', 'm/s', 0.1, 2.0, 0.05,
               'cmd_gate.ros__parameters.limits.normal.max_linear'),
    'goal_tolerance': ('navigation', '到达位置容差', 'm', 0.05, 0.5, 0.01,
                       'controller_server.ros__parameters.goal_checker.xy_goal_tolerance'),
    'linear_accel': ('navigation', '平移加减速度上限', 'm/s²', 0.2, 3.0, 0.1,
                     'controller_server.ros__parameters.FollowPath.ax_max'),
    'velocity_lookahead': ('navigation', '速度参考前瞻距离', 'm', 0.5, 2.5, 0.05,
                           'controller_server.ros__parameters.FollowPath.TranslationVelocityCritic.lookahead'),
    'progress_timeout': ('navigation', '无进展判定时间', 's', 3.0, 30.0, 0.5,
                         'controller_server.ros__parameters.progress_checker.movement_time_allowance'),
    'path_cost_weight': ('navigation', '全局规划障碍代价权重', '倍', 1.0, 8.0, 0.1,
                          'planner_server.ros__parameters.GridBased.cost_travel_multiplier'),
    'local_inflation': ('navigation', '局部代价地图膨胀半径', 'm', 0.55, 1.5, 0.05,
                        'local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius'),
    'global_inflation': ('navigation', '全局代价地图膨胀半径', 'm', 0.55, 2.0, 0.05,
                         'global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius'),
    'local_cost_decay': ('navigation', '局部膨胀代价衰减系数', '1/m', 0.5, 12.0, 0.1,
                         'local_costmap.local_costmap.ros__parameters.inflation_layer.cost_scaling_factor'),
    'global_cost_decay': ('navigation', '全局膨胀代价衰减系数', '1/m', 0.5, 12.0, 0.1,
                          'global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor'),
    'lidar_min_range': ('localization', '雷达近距软件过滤', 'm', *LIDAR_MIN_RANGE_BOUNDS, 0.01, None),
    'correction_period': ('localization', '跟踪校正周期', 's', 0.2, 3.0, 0.05,
                          'backend_node.ros__parameters.tracking.correction_period'),
    'target_radius': ('localization', '跟踪子图半径', 'm', 2.0, 20.0, 0.5,
                      'backend_node.ros__parameters.tracking.target_radius'),
    'target_keyframes': ('localization', '跟踪子图关键帧数', '帧', 3, 30, 1,
                         'backend_node.ros__parameters.tracking.target_keyframes'),
    'tracking_window': ('localization', '跟踪点云累计时间', 's', 0.1, 1.5, 0.05,
                         'backend_node.ros__parameters.tracking.source.max_time'),
    'initial_search_radius': ('localization', '开机定位搜索半径', 'm', 0.5, 10.0, 0.1,
                              'backend_node.ros__parameters.recovery.initial_search_radius'),
    'recovery_candidates': ('localization', '重定位验证候选数', '个', 1, 40, 1,
                            'backend_node.ros__parameters.recovery.max_candidates'),
    'recovery_confirmations': ('localization', '重定位连续确认次数', '次', 2, 8, 1,
                               'backend_node.ros__parameters.recovery.confirmations'),
    'keyframe_interval': ('localization', '建图关键帧最大间隔', 's', 0.1, 3.0, 0.05,
                          'backend_node.ros__parameters.trajectory.keyframe.max_interval'),
    'loop_radius': ('localization', '建图回环子图半径', 'm', 3.0, 25.0, 0.5,
                    'backend_node.ros__parameters.loop.target_radius'),
}


def number(value, minimum, maximum, name):
    if type(value) not in (int, float) or not minimum <= value <= maximum:
        raise ValueError(f'{name}必须在 {minimum} 到 {maximum} 之间')
    return value


def text(value, name, limit=400):
    if not isinstance(value, str) or len(value) > limit or any(ord(char) < 32 for char in value):
        raise ValueError(f'{name}格式无效')
    return value.strip()


def choice(value, allowed, name):
    if value not in allowed:
        raise ValueError(f'{name}的可选值为 {", ".join(allowed)}')
    return value


def flag(value, name):
    if type(value) is not bool:
        raise ValueError(f'{name}需要布尔值')
    return value


def path(value):
    return (WORKSPACE / Path(value).expanduser()).resolve()


def workspace_path(value, label):
    raw = text(value, label)
    target = path(raw)
    if not raw or not target.is_relative_to(WORKSPACE) or target == WORKSPACE:
        raise ValueError(f'{label}须位于项目工作区内')
    return target


def map_output(value):
    raw = text(value, '输出路径')
    selected = WORKSPACE / Path(raw).expanduser()
    target = selected.resolve()
    root = WORKSPACE / 'maps'
    if not target.is_relative_to(root) or target == root:
        raise ValueError('输出须位于工作区 maps/ 下')
    if selected.is_symlink() or (target.exists() and not target.is_dir()):
        raise ValueError('输出路径须为实际目录')
    return target


def prior_path(value):
    folder = path(value)
    if not value or not (folder / 'keyframes.txt').is_file() or not (folder / 'clouds').is_dir():
        raise ValueError('请选择含 keyframes.txt 和 clouds/ 的先验地图目录')
    return folder


def terrain_path(value):
    folder = path(value)
    if not value:
        raise ValueError('请选择先验地图对应的地形目录')
    if (folder / 'terrain' / 'nav_map.yaml').is_file():
        folder = folder / 'terrain'
    missing = [name for name in TERRAIN_FILES if not (folder / name).is_file()]
    if missing:
        raise ValueError(f'地形目录 {folder} 缺少 {", ".join(missing)}')
    return folder


def candidates():
    maps, terrain = [], []
    for directory, subdirs, files in os.walk(WORKSPACE / 'maps'):
        folder = Path(directory).relative_to(WORKSPACE)
        subdirs[:] = sorted(name for name in subdirs if not name.startswith('.') and name != 'clouds')
        if len(folder.parts) >= 5:
            subdirs.clear()
        if 'keyframes.txt' in files and (WORKSPACE / folder / 'clouds').is_dir():
            maps.append(str(folder))
        if set(TERRAIN_FILES) <= set(files):
            terrain.append(str(folder))
    return {'maps': maps, 'terrain': terrain}


def defaults(profile):
    simulation = profile == 'sim'
    return dict(mode='localization', lidar='dual' if simulation else 'single',
                prior_map='maps/sim' if simulation else 'maps/real/site_a_001', terrain_dir='',
                map_dir=f'maps/{profile}/new_session', arena='rmuc', team='auto', gui=False, uwb=True,
                chassis_port='/dev/nav', favorites=[], l2_launch='', tuning={})


def yaml_write(destination, data):
    with NamedTemporaryFile(mode='w', dir=destination.parent, delete=False, encoding='utf-8') as stream:
        temporary = Path(stream.name)
    try:
        temporary.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding='utf-8')
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)


def yaml_value(data, key):
    for segment in key.split('.'):
        data = data[segment]
    return data


def yaml_set(data, key, value):
    *parts, name = key.split('.')
    for part in parts:
        data = data.setdefault(part, {})
    data[name] = value


def navigation_defaults(profile):
    data = yaml.safe_load(NAV_PARAMS.read_text())
    if profile == 'real':
        data = merge(data, yaml.safe_load(NAV_PARAMS.with_name('nav2_real.yaml').read_text()))
    return data


class Settings:
    def __init__(self, directory):
        self.file = directory / 'settings.yaml'
        self.data = {'profile': 'sim', 'sim': defaults('sim'), 'real': defaults('real')}
        if self.file.exists():
            saved = yaml.safe_load(self.file.read_text(encoding='utf-8'))
            self.data['profile'] = choice(saved['profile'], ('sim', 'real'), '平台')
            for profile in ('sim', 'real'):
                self.data[profile].update(saved[profile])
                self.data[profile].pop('serial_goals', None)

    @property
    def profile(self):
        return self.data['profile']

    @property
    def active(self):
        return self.data[self.profile]

    def setup(self, payload):
        profile = choice(payload.get('profile'), ('sim', 'real'), '平台')
        previous = self.data[profile]
        document = dict(previous)
        for name, allowed in (('mode', ('mapping', 'localization')), ('lidar', ('single', 'dual')),
                              ('arena', ('rmuc', 'rmul')), ('team', ('auto', 'red', 'blue'))):
            document[name] = choice(payload.get(name, previous[name]), allowed, name)
        for name in ('prior_map', 'terrain_dir', 'map_dir', 'chassis_port', 'l2_launch'):
            document[name] = text(payload.get(name, previous[name]), name)
        document['gui'] = flag(payload.get('gui', previous['gui']), 'Gazebo GUI')
        document['uwb'] = flag(payload.get('uwb', previous['uwb']), '仿真 UWB')
        document['tuning'] = previous['tuning']
        self.save(dict(self.data, profile=profile, **{profile: document}))

    def favorite(self, payload):
        profile = choice(payload.get('profile'), ('sim', 'real'), '平台')
        pair = {key: text(payload.get(key), key) for key in ('prior_map', 'terrain_dir')}
        items = list(self.data[profile]['favorites'])
        if flag(payload.get('remove'), '移除常用地形'):
            if pair in items:
                items.remove(pair)
            message = '已移除常用地形'
        else:
            prior_path(pair['prior_map'])
            terrain_path(pair['terrain_dir'] or pair['prior_map'])
            if pair not in items:
                if len(items) >= 50:
                    raise ValueError('每个平台最多保存 50 组常用地形')
                items.append(pair)
            message = '先验地图和配套地形已加入常用'
        self.save(dict(self.data, **{profile: dict(self.data[profile], favorites=items)}))
        return message

    def tune(self, payload):
        values = {}
        if set(payload) - TUNING.keys():
            raise ValueError('存在不支持的调参项')
        if self.profile != 'real' and 'lidar_min_range' in payload:
            raise ValueError('雷达近距软件过滤仅用于真车配置')
        for name, value in payload.items():
            _, label, _, minimum, maximum, _, _ = TUNING[name]
            value = number(value, minimum, maximum, label)
            if type(minimum) is int and int(value) != value:
                raise ValueError(f'{label}需要整数')
            values[name] = type(minimum)(value)
        self.save(dict(self.data, **{self.profile: dict(self.active, tuning=values)}))

    def save(self, document):
        yaml_write(self.file, document)
        self.data = document

    def fields(self, profile):
        nav = navigation_defaults(profile)
        lidar = self.data[profile]['lidar']
        loc = resolve(LOC_CONFIG / profile, 'backend', lidar)
        fields = []
        for name, (component, label, unit, minimum, maximum, step, key) in TUNING.items():
            if name == 'lidar_min_range':
                if profile != 'real':
                    continue
                frontend = resolve(LOC_CONFIG / profile, 'frontend', lidar)
                default = yaml_value(frontend, 'zlio.ros__parameters.sensor.minimum_range')
            else:
                default = yaml_value(nav if component == 'navigation' else loc, key)
            fields.append(dict(key=name, component=component, label=label, unit=unit, min=minimum,
                               max=maximum, step=step, default=default))
        return fields

    def overrides(self, component, destination):
        values = self.active['tuning']
        if component == 'navigation':
            data = navigation_defaults(self.profile)
        else:
            data = {'backend_node': {'ros__parameters': {}}}
        for name, value in values.items():
            group, _, _, _, _, _, key = TUNING[name]
            if group == component and key is not None:
                yaml_set(data, key, value)
        if 'cruise' in values and component == 'navigation':
            critic = data['controller_server']['ros__parameters']['FollowPath']['TranslationVelocityCritic']
            limit = values['cruise']
            critic['max_speed'] = min(critic['max_speed'], limit) if self.profile == 'real' else limit
        if 'linear_accel' in values and component == 'navigation':
            accel = values['linear_accel']
            controller = data['controller_server']['ros__parameters']['FollowPath']
            controller.update(ax_max=accel, ax_min=-accel, ay_max=accel, ay_min=-accel)
            smoother = data['velocity_smoother']['ros__parameters']
            smoother['max_accel'][:2] = [accel, accel]
            smoother['max_decel'][:2] = [-accel, -accel]
        if 'path_cost_weight' in values and component == 'navigation':
            data['planner_server']['ros__parameters']['StrictGridBased']['cost_travel_multiplier'] = values['path_cost_weight']
        yaml_write(destination, data)


def bag_request(payload):
    name = text(payload.get('name', ''), '录包目录名', 80)
    if name and (name in ('.', '..') or not re.fullmatch(r'[\w.-]+', name)):
        raise ValueError('录包目录名只能包含字母、数字、下划线、短横线和点')
    duration = number(payload.get('duration', 0), 0, 86400, '录制时长')
    note = text(payload.get('note', ''), '录包备注')
    topic_input = text(payload.get('topics', ''), '追加话题', 1500)
    topics = []
    for topic in topic_input.split():
        if not re.fullmatch(r'/(?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*', topic):
            raise ValueError(f'话题名称无效 {topic}')
        topics.extend(('--topic', topic))
    return name, ['--duration', str(duration), '--note', note, *topics,
                  *(['--all'] if flag(payload.get('all', False), '全部话题') else [])]
