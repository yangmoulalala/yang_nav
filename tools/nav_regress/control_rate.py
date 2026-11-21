#!/usr/bin/env python3
"""Measure the real navigation command chain and PTY frame rate on a fixture.

No hardware is used. Nav2 drives framework_accept's kinematic plant; the chassis
transmitter writes to a PTY. Run in an isolated domain, default 198.
"""

import argparse
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.parameter_client import AsyncParameterClient
from geometry_msgs.msg import Twist

from chassis_comm.protocol import FrameParser
from framework_accept import Plant, fixture


TOPICS = ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel_collision_checked', '/cmd_vel')


class RatePlant(Plant):
    def __init__(self, out, master):
        super().__init__(out, step_period=.01)
        self.arrivals = {name: [] for name in (*TOPICS, 'serial_frames')}
        self.master = master
        self.parser = FrameParser()
        self.command_subscriptions = [self.create_subscription(
            Twist, name, lambda msg, topic=name: self.arrivals[topic].append(time.monotonic()),
            100) for name in TOPICS]
        self.serial_timer = self.create_timer(.002, self.read_serial)

    def read_serial(self):
        while True:
            try:
                data = os.read(self.master, 8192)
            except BlockingIOError:
                return
            if not data:
                return
            commands = self.parser.feed(data)
            self.arrivals['serial_frames'].extend([time.monotonic()] * len(commands))

    def parameters(self, node_name, names):
        client = AsyncParameterClient(self, node_name)
        self.wait(client.services_are_ready, 5., f'{node_name} parameters unavailable')
        future = client.get_parameters(names)
        self.wait(future.done, 5., f'{node_name} parameter read timed out')
        return {name: value.double_value for name, value in zip(names, future.result().values)}

    def rates(self, begin, end):
        result = {}
        for name, arrivals in self.arrivals.items():
            stamps = np.array([t for t in arrivals if begin <= t < end])
            gaps = np.diff(stamps)
            result[name] = dict(
                count=len(stamps), hz=len(stamps) / (end - begin),
                p95_gap_ms=float(np.percentile(gaps, 95) * 1000) if len(gaps) else None,
                max_gap_ms=float(max(gaps) * 1000) if len(gaps) else None)
        return result


def stop(process):
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default='/tmp/nav_control_rate')
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '198')
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    terrain = fixture(out)
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    rclpy.init()
    plant = RatePlant(out, master)
    processes, logs = [], []
    result = dict(passed=False, target_hz=100.0,
                  model='real Nav2 + deterministic omni plant + PTY, no hardware')
    try:
        for name, command in (
            ('navigation', ['ros2', 'launch', 'nav_bringup', 'navigation.launch.py',
                            'terrain_dir:=' + str(terrain), 'use_sim_time:=false']),
            ('chassis', ['ros2', 'run', 'chassis_comm', 'chassis_serial', '--ros-args',
                         '-p', f'port:={os.ttyname(slave)}', '-p', 'baud_rate:=115200']),
        ):
            log = (out / f'{name}.log').open('w')
            logs.append(log)
            processes.append(subprocess.Popen(
                command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        plant.wait(lambda: 'Managed nodes are active' in
                   (out / 'navigation.log').read_text(errors='replace'), 50.,
                   'navigation lifecycle failed')
        checks = {
            '/controller_server': ['controller_frequency'],
            '/behavior_server': ['cycle_frequency'],
            '/velocity_smoother': ['smoothing_frequency'],
            '/cmd_gate': ['rate'],
            '/chassis_serial': ['rate', 'yaw_horizon'],
            '/local_costmap/local_costmap': ['update_frequency'],
        }
        result['parameters'] = {node: plant.parameters(node, names)
                                for node, names in checks.items()}
        for node, values in result['parameters'].items():
            for name, value in values.items():
                expected = .1 if name == 'yaw_horizon' else 100.
                assert abs(value - expected) < 1e-9, (node, name, value, expected)
        _, future = plant.start((-3., 4.5, 0.))
        plant.run_for(.7)
        assert not future.done(), 'goal finished before the measurement window'
        begin = time.monotonic()
        plant.run_for(3.)
        end = time.monotonic()
        result['window_seconds'] = end - begin
        result['rates'] = plant.rates(begin, end)
        print(json.dumps(result['rates'], indent=2), flush=True)
        assert not future.done(), 'measurement window included an inactive controller'
        for topic, values in result['rates'].items():
            assert 90. <= values['hz'] <= 110., f'{topic}: {values}'
        plant.finish(future, (-3., 4.5, 0.))
        result['passed'] = True
        code = 0
    except Exception as exc:
        result['error'] = str(exc)
        code = 1
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
        plant.trace.close()
        plant.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
