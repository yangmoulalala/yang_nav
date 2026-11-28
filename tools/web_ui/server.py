import argparse
import fcntl
from functools import lru_cache, partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import threading
import traceback
from urllib.parse import parse_qs, urlsplit

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .processes import LABELS, OWNERS, PROCESSES, Processes
from .ros_state import RosState
from .settings import Settings, WORKSPACE, candidates, prior_path, terrain_path
from .terrain import (TerrainEditor, field_references, matches_prior, preview_archive,
                      render_match_reference, terrain_sources)


STATIC = Path(__file__).with_name('static')
ASSETS = {'/': ('index.html', 'text/html'), '/style.css': ('style.css', 'text/css'),
          '/app.js': ('app.js', 'text/javascript'), '/terrain.js': ('terrain.js', 'text/javascript')}


def finite(value):
    if isinstance(value, dict):
        return {key: finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class Console:
    def __init__(self, directory, ros, foxglove_port):
        self.settings = Settings(directory)
        self.processes = Processes(self.settings, ros, directory)
        self.terrain = TerrainEditor()
        self.ros = ros
        self.operation = threading.Lock()
        self.admission = threading.Lock()
        self.domain = os.environ.get('ROS_DOMAIN_ID', '0')
        self.foxglove_port = foxglove_port

    def snapshot(self):
        return dict(domain=self.domain, profile=self.settings.profile,
                    ros=self.ros.snapshot(), processes=self.processes.describe(),
                    matching_map=self.matching_map(),
                    foxglove_online='/foxglove_bridge' in self.ros.node_names(),
                    foxglove_port=self.foxglove_port)

    def matching_map(self):
        prior = self.preview_prior()
        if not prior:
            return None
        loc = self.processes.entries.get('localization')
        config = loc.parameters if self.processes.alive('localization') else self.settings.active
        nav = self.processes.entries.get('navigation')
        try:
            folder = (Path(nav.output) if self.processes.alive('navigation') and nav.output else
                      terrain_path(config['terrain_dir'] or str(prior)))
            meta = folder / 'terrain.json'
            stat = meta.stat()
            return read_landmarks(meta, prior, stat.st_ino, stat.st_mtime_ns)
        except (ValueError, OSError, TypeError):
            return None

    def preview_prior(self):
        if self.processes.alive('localization'):
            process = self.processes.entries['localization']
            return Path(process.output) if process.parameters['mode'] == 'localization' else None
        if self.settings.active['mode'] != 'localization' or '/backend_node' in self.ros.node_names():
            return None
        try:
            return prior_path(self.settings.active['prior_map'])
        except ValueError:
            return None

    def config(self):
        sources = terrain_sources()
        return {**self.settings.data, **candidates(), 'terrain_sources': sources,
                'terrain_references': field_references(sources, self.preview_prior()),
                'fields': {profile: self.settings.fields(profile) for profile in ('sim', 'real')}}

    def action(self, route, body):
        stop = route == '/api/stack/stop'
        with self.admission:
            if self.processes.cancel_start.is_set():
                raise ValueError('工作台正在关闭进程')
            if stop:
                self.processes.cancel_start.set()
            elif not self.operation.acquire(blocking=False):
                raise ValueError('有操作正在执行，请等待完成，或点击关闭全部')
        if stop:
            self.operation.acquire()
        try:
            return self.perform(route, body)
        finally:
            with self.admission:
                self.operation.release()
                if stop:
                    self.processes.cancel_start.clear()

    def perform(self, route, body):
        processes = self.processes
        if route == '/api/setup':
            profile = body.get('profile')
            if profile != self.settings.profile:
                active = any(processes.alive(name) for name in PROCESSES if name != 'optimizer')
                external = self.ros.node_names().intersection(node for names in OWNERS.values() for node in names)
                if active or external:
                    raise ValueError('请关闭当前 ROS 域的业务进程和录包后切换平台')
            self.settings.setup(body)
            return '配置已保存。重启相关进程后应用。'
        if route == '/api/tune':
            overrides = body.get('values')
            if body.get('profile') != self.settings.profile or not isinstance(overrides, dict):
                raise ValueError('请刷新页面并核对当前平台')
            self.settings.tune(overrides)
            return '参数已保存。重启相关进程后应用。'
        if route == '/api/favorite':
            return self.settings.favorite(body)
        if route == '/api/stack/start':
            processes.start_stack()
            return '启动请求已提交，可在工作台查看 ROS 节点是否就绪。'
        if route == '/api/stack/stop':
            processes.stop_stack()
            return '本工作台启动的进程已关闭。'
        if route == '/api/goal':
            processes.send_goal(body)
            return '目标已交给发送器，查看进程日志确认 Nav2 是否接受。'
        if route == '/api/cancel_goal':
            if processes.alive('goal'):
                processes.stop('goal')
                return '网页目标发送器已退出，查看日志确认取消结果。'
            return self.ros.call('cancel_goal')
        if route == '/api/save_map':
            if processes.alive('localization') and not processes.active_mapping():
                raise ValueError('保存地图只用于建图模式')
            return self.ros.call('save_map')
        if route == '/api/relocalize':
            if processes.active_mapping():
                raise ValueError('全局重定位需要先验地图匹配模式')
            if processes.alive('navigation') or {'/bt_navigator', '/cmd_gate'} & self.ros.node_names():
                raise ValueError('请先停止 Nav2 并确认车辆停稳，再做全局重定位')
            return self.ros.call('relocalize')
        prefix, _, action = route.rpartition('/')
        name = prefix.removeprefix('/api/process/')
        if not prefix.startswith('/api/process/') or name not in PROCESSES[:-1] or action not in ('start', 'stop', 'restart'):
            raise ValueError('未知操作')
        if action == 'start':
            processes.start(name, body)
        elif action == 'stop':
            processes.stop(name)
        else:
            processes.restart(name, body)
        return f'{LABELS[name]} 操作已提交'


@lru_cache(maxsize=32)
def read_landmarks(meta, prior, inode, modified):
    data = json.loads(meta.read_text(encoding='utf-8'))
    if not isinstance(data, dict) or not matches_prior(data.get('frame'), data.get('source'), prior):
        return None
    raw = data.get('landmarks', {})
    if not isinstance(raw, dict):
        return None
    points = {name: pair for name, pair in raw.items() if name in ('home', 'target', 'match') and
              isinstance(pair, list) and len(pair) == 2 and
              all(type(value) in (int, float) and math.isfinite(value) for value in pair)}
    return dict(terrain=str(meta.parent), prior=str(prior), landmarks=points, revision=f'{inode}-{modified}',
                grid=[data.get('resolution'), data.get('origin'), data.get('size')])


class Handler(BaseHTTPRequestHandler):
    def __init__(self, *args, console, **kwargs):
        self.console = console
        super().__init__(*args, **kwargs)

    def setup(self):
        super().setup()
        self.connection.settimeout(8)

    def respond(self, status, body, content_type='application/json', headers=()):
        if not isinstance(body, bytes):
            body = json.dumps(finite(body), ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', f'{content_type}; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy',
                         "default-src 'self'; object-src 'none'; frame-ancestors 'none'; base-uri 'none'")
        for key, value in headers:
            self.send_header(key, value)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        request = urlsplit(self.path)
        route = request.path
        if route in ASSETS:
            name, content_type = ASSETS[route]
            self.respond(200, (STATIC / name).read_bytes(), content_type)
        elif route == '/api/status':
            self.respond(200, self.console.snapshot())
        elif route == '/api/config':
            self.respond(200, self.console.config())
        elif route == '/api/match-reference':
            try:
                terrain = self.console.matching_map()
                revision = parse_qs(request.query).get('revision', [''])[0]
                if not terrain or 'match' not in terrain['landmarks'] or terrain['revision'] != revision:
                    raise ValueError('匹配参考点缺失或地图已经切换')
                png = render_match_reference(Path(terrain['terrain']), terrain['grid'], terrain['landmarks']['match'])
                if terrain != self.console.matching_map():
                    raise ValueError('地形已经更新，请重新选用')
                self.respond(200, png, 'image/png')
            except (OSError, ValueError) as exc:
                self.respond(400, {'error': str(exc)})
        elif route == '/api/terrain':
            self.respond(200, self.console.terrain.snapshot())
        elif route == '/api/terrain/project-preview':
            query = parse_qs(request.query)
            try:
                image, metadata = preview_archive(query.get('source', [''])[0], self.console.preview_prior())
                self.respond(200, image, 'image/png', [('X-Terrain-Preview', json.dumps(metadata))])
            except ValueError as exc:
                self.respond(400, {'error': str(exc)})
        elif route in ('/api/terrain/preview', '/api/terrain/owners', '/api/terrain/patch'):
            query = parse_qs(request.query)
            revision = query.get('revision', [''])[0]
            try:
                if route.endswith('/patch'):
                    self.respond(200, self.console.terrain.details(revision, int(query.get('id', ['0'])[0])))
                else:
                    body, mime = self.console.terrain.raster(route.rsplit('/', 1)[1], revision)
                    self.respond(200, body, mime)
            except ValueError as exc:
                self.respond(400, {'error': str(exc)})
        elif route.startswith('/api/log/') and route.removeprefix('/api/log/') in PROCESSES:
            self.respond(200, {'text': self.console.processes.log(route.removeprefix('/api/log/'))})
        else:
            self.respond(404, {'error': '页面不存在'})

    def do_POST(self):
        if (self.headers.get('X-Nav-Console') != '1' or
                self.headers.get_content_type() != 'application/json'):
            self.respond(403, {'error': '请从工作台页面提交操作'})
            return
        try:
            route = urlsplit(self.path).path
            terrain_operation = route.startswith('/api/terrain/')
            length = int(self.headers.get('Content-Length', 0))
            if not 0 < length <= (131072 if terrain_operation else 8192):
                raise ValueError('请求大小超出限制')
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError('无效的请求数据')
            if terrain_operation:
                result = self.console.terrain.action(route.removeprefix('/api/terrain/'), body)
            else:
                result = {'message': self.console.action(route, body)}
            self.respond(200, result)
        except (ValueError, OSError) as exc:
            self.respond(400, {'error': str(exc)})
        except Exception:
            traceback.print_exc()
            self.respond(500, {'error': '操作失败，请查看 WebUI 终端日志'})

    def log_request(self, code='-', size='-'):
        if self.command != 'GET' or str(code) not in ('200', '404'):
            super().log_request(code, size)


def port(raw):
    value = int(raw)
    if not 1 <= value <= 65535:
        raise argparse.ArgumentTypeError('端口必须在 1 到 65535 之间')
    return value


def main():
    parser = argparse.ArgumentParser(description='yang_nav ROS 2 Web 工作台')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', default=8080, type=port)
    parser.add_argument('--foxglove-port', default=8766, type=port)
    args = parser.parse_args()
    if args.port == args.foxglove_port:
        parser.error('网页与 Foxglove Bridge 需要不同端口')
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if not domain.isdecimal():
        parser.error('ROS_DOMAIN_ID 必须是数字')
    directory = WORKSPACE / 'log' / 'web_ui' / f'domain_{int(domain)}'
    directory.mkdir(parents=True, exist_ok=True)
    lock = (directory / 'console.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error(f'ROS 域 {domain} 已运行一个 WebUI')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    ros = RosState()
    executor = SingleThreadedExecutor()
    executor.add_node(ros)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    console = Console(directory, ros, args.foxglove_port)
    handler = partial(Handler, console=console)
    try:
        with ThreadingHTTPServer((args.host, args.port), handler) as http:
            def shutdown(_signum, _frame):
                threading.Thread(target=http.shutdown, daemon=True).start()

            signal.signal(signal.SIGTERM, shutdown)
            signal.signal(signal.SIGINT, shutdown)
            print(f'WebUI http://{args.host}:{args.port}  ·  ROS_DOMAIN_ID={domain}', flush=True)
            http.serve_forever()
    finally:
        try:
            console.processes.cancel_start.set()
            with console.operation:
                try:
                    console.processes.stop_stack()
                finally:
                    console.processes.stop('optimizer')
        finally:
            executor.shutdown()
            spinner.join()
            ros.destroy_node()
            rclpy.shutdown()
            lock.close()


if __name__ == '__main__':
    main()
