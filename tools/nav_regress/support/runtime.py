"""Own the test process groups and persist a scenario's result."""
from contextlib import contextmanager
import json
import os
import signal
import subprocess
import time

import rclpy


@contextmanager
def logged_process(command, log_path):
    with log_path.open('w') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            yield process
        finally:
            for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=timeout)
                    break
                except subprocess.TimeoutExpired:
                    if sig == signal.SIGKILL:
                        raise
            process.wait()


@contextmanager
def navigation(out, terrain, probe, *, require_trusted=True, timeout=50):
    log_path = out / 'navigation.log'
    command = ['ros2', 'launch', 'nav_bringup', 'navigation.launch.py',
               f'terrain_dir:={terrain}', 'use_sim_time:=false',
               f'require_trusted:={str(require_trusted).lower()}']
    with logged_process(command, log_path) as process:
        deadline = time.monotonic() + timeout
        while 'Managed nodes are active' not in log_path.read_text(errors='replace'):
            if process.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError(f'Navigation failed to activate. See {log_path}')
            rclpy.spin_once(probe, timeout_sec=.05)
        yield log_path


def finish(out, result):
    text = json.dumps(result, indent=2)
    (out / 'result.json').write_text(text + '\n')
    print(text, flush=True)
    return 0 if result['passed'] else 1
