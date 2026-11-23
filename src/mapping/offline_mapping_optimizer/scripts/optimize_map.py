#!/usr/bin/env python3
"""Optimize a saved yang_nav map bundle and rebuild its localization descriptors."""
import argparse
import math
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--in', dest='source', type=Path, required=True,
                        help='saved native map bundle (keyframes.txt + clouds/)')
    parser.add_argument('--out', type=Path, required=True, help='new output map bundle')
    parser.add_argument('--config', type=Path, help='optimizer params.yaml override')
    parser.add_argument('--threads', type=int, help='override all three CPU thread counts')
    parser.add_argument('--lidar-origin', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='LiDAR optical origin in saved cloud coordinates, metres')
    parser.add_argument('--no-raycasting', action='store_true',
                        help='disable single-origin dynamic filtering (e.g. fused dual-LiDAR data)')
    parser.add_argument('--no-mls', action='store_true', help='disable global PCD MLS smoothing')
    args = parser.parse_args()
    source, output = args.source.expanduser().resolve(), args.out.expanduser().resolve()
    if not (source / 'keyframes.txt').is_file():
        parser.error(f'not a native map bundle: {source}')
    if output == source or source in output.parents:
        parser.error('--out must be outside the input bundle')
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        parser.error('--out must be absent or an empty directory')
    if args.threads is not None and args.threads < 1:
        parser.error('--threads must be positive')
    if args.lidar_origin and not all(math.isfinite(v) for v in args.lidar_origin):
        parser.error('--lidar-origin must be finite')
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    bridge = Path(get_package_prefix('backend')) / 'lib/backend/offline_optimizer_bundle'
    optimizer = (Path(get_package_prefix('offline_mapping_optimizer')) /
                 'lib/offline_mapping_optimizer/offline_mapping_optimizer_node')
    config = (args.config.expanduser().resolve() if args.config else
              Path(get_package_share_directory('offline_mapping_optimizer')) / 'config/params.yaml')
    if not bridge.is_file() or not optimizer.is_file() or not config.is_file():
        parser.error('missing executable/config; rebuild backend and offline_mapping_optimizer')
    output.parent.mkdir(parents=True, exist_ok=True)
    # An isolated domain keeps this batch node out of the running robot graph.
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('OFFLINE_ROS_DOMAIN_ID', '93'))
    with tempfile.TemporaryDirectory(prefix='.offline_opt_', dir=output.parent) as work:
        work = Path(work)
        log_path = work / 'optimizer.log'
        with log_path.open('w') as log:
            def run(command):
                command = [str(item) for item in command]
                print('+ ' + shlex.join(command), flush=True)
                log.write('+ ' + shlex.join(command) + '\n')
                log.flush()
                with subprocess.Popen(command, env=env, stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT, text=True) as process:
                    for line in process.stdout:
                        print(line, end='', flush=True)
                        log.write(line)
                    code = process.wait()
                log.flush()
                if code:
                    raise subprocess.CalledProcessError(code, command)
            try:
                run([bridge, '--in', source, '--out', work / 'input'])
                command = [optimizer, '--ros-args', '--params-file', config,
                           '-p', f'data_path:={work / "input"}',
                           '-p', f'output_path:={work / "results"}']
                if args.threads is not None:
                    for key in ('num_threads', 'local_vgicp_num_threads', 'global_vgicp_num_threads'):
                        command += ['-p', f'{key}:={args.threads}']
                if args.no_raycasting:
                    command += ['-p', 'enable_raycasting_filter:=false']
                if args.lidar_origin:
                    command += ['-p', f'lidar_origin:={args.lidar_origin}']
                if args.no_mls:
                    command += ['-p', 'enable_mls_smoothing:=false']
                run(command)
                run([bridge, '--in', work / 'input', '--results', work / 'results', '--out', output])
            except (subprocess.CalledProcessError, KeyboardInterrupt):
                failure = output.parent / (output.name + '.optimization-failed.log')
                failure.write_text(log_path.read_text())
                print(f'Optimization interrupted/failed; log: {failure}', flush=True)
                return 1
        (output / 'optimizer.log').write_text(log_path.read_text())
    print(f'Optimized map bundle: {output}\nGlobal cloud: {output / "map.pcd"}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
