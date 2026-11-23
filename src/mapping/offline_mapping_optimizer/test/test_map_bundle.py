"""Process-level acceptance fixtures; source ROS/workspace before running pytest."""
import hashlib
import math
from pathlib import Path
import struct
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory

import pytest
import yaml

BRIDGE = Path(get_package_prefix('backend')) / 'lib/backend/offline_optimizer_bundle'
CONFIG = Path(get_package_share_directory('offline_mapping_optimizer')) / 'config/params.yaml'


def write_cloud(path, points):
    header = ('VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n'
              f'COUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\n'
              f'POINTS {len(points)}\nDATA binary\n')
    path.write_bytes(header.encode() + b''.join(struct.pack('<ffff', *p) for p in points))


def fingerprint(directory):
    return {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in directory.rglob('*') if p.is_file()}


def invoke(*args, code=0):
    result = subprocess.run(['ros2', 'run', 'offline_mapping_optimizer', 'optimize_map.py', *map(str, args)],
                            text=True, capture_output=True, timeout=120)
    assert result.returncode == code, result.stdout + result.stderr
    return result


def config(tmp_path, **overrides):
    content = yaml.safe_load(CONFIG.read_text())
    content['/**']['ros__parameters'].update(overrides)
    path = tmp_path / 'params.yaml'
    path.write_text(yaml.safe_dump(content))
    return path


def test_two_stage_vgicp_corrects_relative_drift(tmp_path):
    source = tmp_path / 'drift'
    (source / 'clouds').mkdir(parents=True)
    # Three nonparallel walls with an asymmetric distribution of points.
    world = ([(4.03, -.92 + .17*y, .13 + .17*z, 10.) for y in range(16) for z in range(12)] +
             [(-.93 + .17*x, 3.07, .13 + .17*z, 20.) for x in range(20) for z in range(12)] +
             [(-.93 + .17*x, -.92 + .17*y, -.07, 30.) for x in range(20) for y in range(16)])
    rows = ['# backend_map 1']
    for i in range(9):
        true_x = .15*i
        points = [(x - true_x, y, z, intensity) for x, y, z, intensity in world]
        write_cloud(source / 'clouds' / f'{i:06d}.pcd', points)
        yaw = .004*i
        rows.append(f'{i} {10+i} {.175*i} 0 0 0 0 {math.sin(yaw/2)} {math.cos(yaw/2)}')
    (source / 'keyframes.txt').write_text('\n'.join(rows) + '\n')
    before = fingerprint(source)
    output = tmp_path / 'optimized'
    params = config(tmp_path, local_submap_size=3, global_min_loop_overlap=.3,
                    global_voxel_resolution=.15, local_voxel_resolution=.15)
    result = invoke('--in', source, '--out', output, '--config', params,
                    '--threads', '2', '--no-raycasting', '--no-mls')
    assert 'Local stage: 3 submaps' in result.stdout
    assert 'Global graph: nodes=3' in result.stdout
    poses = [[float(v) for v in row.split()] for row in (output / 'keyframes.txt').read_text().splitlines()
             if not row.startswith('#')]
    relative_x = poses[-1][2] - poses[0][2]
    assert abs(relative_x - 1.2) < .05, relative_x  # initial error: 0.20 m
    assert fingerprint(source) == before
    assert len(list((output / 'clouds').glob('*.pcd'))) == 9
    assert (output / 'optimizer.log').is_file()
    # Published bundles must load through the actual backend, including XYZ/I validation.
    subprocess.run([str(BRIDGE),
                    '--in', str(output), '--out', str(tmp_path / 'reload')], check=True)


@pytest.mark.parametrize('origin', [(0., 0., 0.), (1.1, 2., .7)])
def test_raycasting_removes_transient_geometry_from_local_clouds(tmp_path, origin):
    source = tmp_path / 'moving_object'
    (source / 'clouds').mkdir(parents=True)
    rows = ['# backend_map 1']
    wall = [(6.05, .05+.2*y, .05+.2*z, 42.) for y in range(-15, 16) for z in range(-15, 16)]
    for i in range(6):
        rows.append(f'{i} {i+1} 0 0 0 0 0 0 1')
        points = wall + ([(2.05, .05, .05, 99.)] if i == 0 else [])
        points = [(x+origin[0], y+origin[1], z+origin[2], intensity)
                  for x, y, z, intensity in points]
        write_cloud(source / 'clouds' / f'{i:06d}.pcd', points)
    (source / 'keyframes.txt').write_text('\n'.join(rows) + '\n')
    output = tmp_path / 'cleaned'
    params = config(tmp_path, enable_factor_graph_optimization=False,
                    enable_statistical_outlier_removal=False, voxel_resolution=.2)
    invoke('--in', source, '--out', output, '--config', params, '--threads', '2', '--no-mls',
           '--lidar-origin', *origin)
    report = (output / 'optimization.txt').read_text()
    first = next(line for line in report.splitlines() if not line.startswith('#')).split()
    assert first[2:] == ['961', '962']
    assert '# removed_points 1/5767' in report
    # Backend reload/re-export reads compressed native PCD and emits an independent snapshot.
    snapshot = tmp_path / 'reload'
    subprocess.run([str(BRIDGE),
                    '--in', str(output), '--out', str(snapshot)], check=True)
    invoke('--in', source, '--out', output, code=2)
    invoke('--in', source, '--out', source / 'child', code=2)


def test_invalid_optimizer_config_does_not_publish_bundle(tmp_path):
    source = tmp_path / 'source'
    (source / 'clouds').mkdir(parents=True)
    write_cloud(source / 'clouds/000000.pcd', [(float(i), 0., 0., 1.) for i in range(20)])
    (source / 'keyframes.txt').write_text('# backend_map 1\n0 1 0 0 0 0 0 0 1\n')
    output = tmp_path / 'invalid'
    params = config(tmp_path, voxel_resolution=0.)
    invoke('--in', source, '--out', output, '--config', params, code=1)
    assert not output.exists()
    assert (tmp_path / 'invalid.optimization-failed.log').is_file()
