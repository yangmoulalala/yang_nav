#!/usr/bin/env python3
"""Validate resolved common/platform/single-LiDAR parameters and TF ownership."""
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'bringup'))
from bringup.configuration import MODULES, merge, parameter_files, resolve

CONFIG = ROOT / 'bringup' / 'config'
# Only sensor representation, sampling and map/UWB coordinates differ.
PLATFORM_KEYS = {
    'frontend': {
        'zlio/input.imu_acceleration_scale', 'zlio/input.maximum_imu_interval',
        'zlio/sensor.maximum_range',
        'zlio/initialization.minimum_directional_support', 'zlio/filter.gyro_variance',
    },
    'backend': {
        'backend_node/registration.max_iterations',
        'backend_node/registration.convergence_translation',
        'backend_node/registration.convergence_rotation_deg',
        'backend_node/uwb.enabled', 'backend_node/uwb.time_offset_seconds',
        *(f'backend_node/uwb.zero_pose_{team}.{axis}' for team in ('red', 'blue')
          for axis in ('x', 'y', 'yaw_deg')),
    },
    'lidar_fusion': {
        'lidar_merge/primary.time_field', 'lidar_merge/primary.time_absolute',
        'lidar_merge/secondary.time_field', 'lidar_merge/secondary.time_scale',
    },
    'lidar_calibration': {'calibrate/frames'},
}


def flat(data):
    out = {}

    def walk(prefix, value):
        if isinstance(value, dict):
            for key, item in value.items():
                walk(f'{prefix}.{key}', item)
        else:
            out[prefix] = value

    for node, section in data.items():
        for key, value in section['ros__parameters'].items():
            walk(f'{node}/{key}', value)
    return out


def main():
    for module in MODULES:
        example = flat(yaml.safe_load((ROOT / module / 'config/example.yaml').read_text()))
        for platform in ('real', 'sim'):
            for lidar in (('single', 'dual') if module == 'backend' else ('dual',)):
                values = flat(resolve(CONFIG / platform, module, lidar))
                if values.keys() != example.keys():
                    raise ValueError(f'{platform}/{module}/{lidar}: missing {example.keys()-values.keys()}, '
                                     f'extra {values.keys()-example.keys()}')
                if module == 'frontend':
                    assert values['zlio/frames.base'] == 'gimbal_nav'
                    assert values['zlio/frames.imu'] == 'mid360_imu_link'
                    assert values['zlio/output.publish_tf'] is False
                    assert values['zlio/sensor.minimum_range'] == 0.3
                elif module == 'lidar_fusion':
                    assert values['lidar_merge/min_range'] == 0.3
                elif module == 'backend':
                    assert values['backend_node/common.body_frame'] == 'gimbal_nav'
                    assert values['backend_node/common.sensor_frame'] == 'mid360_imu_link'
                    assert values['backend_node/common.publish_odom_tf'] is True
                print(f'OK resolved {platform}/{module}/{lidar}: {len(values)} keys')
        real = flat(resolve(CONFIG / 'real', module))
        sim = flat(resolve(CONFIG / 'sim', module))
        difference = {key for key in real if real[key] != sim[key]}
        assert difference == PLATFORM_KEYS[module], (module, difference ^ PLATFORM_KEYS[module])
        # Unchanged keys in an override would recreate duplicate tuning.
        overlay = yaml.safe_load((CONFIG / 'sim' / 'overrides.yaml').read_text())[module]
        assert set(flat(overlay)) == difference, f'redundant sim override in {module}'
    for platform in ('real', 'sim'):
        dual = flat(resolve(CONFIG / platform, 'backend'))
        single = flat(resolve(CONFIG / platform, 'backend', 'single'))
        differences = {key for key in dual if single[key] != dual[key]}
        expected = {'backend_node/common.scan_context.similarity_normalization',
                    'backend_node/tracking.source.max_frames',
                    'backend_node/tracking.source.max_time'}
        if platform == 'sim':
            expected.add('backend_node/registration.voxel_resolution')
        assert differences == expected, (platform, differences)
    try:
        merge({'known': 1}, {'typo': 2})
    except ValueError:
        pass
    else:
        raise AssertionError('unknown override keys must be rejected')
    for name in ('livox_driver.yaml',):
        flat(yaml.safe_load((CONFIG / 'real' / name).read_text()))
    with parameter_files(CONFIG / 'real', lidar='single', lidar_min_range=0.25) as directory:
        frontend = flat(yaml.safe_load((Path(directory) / 'frontend.yaml').read_text()))
        fusion = flat(yaml.safe_load((Path(directory) / 'lidar_fusion.yaml').read_text()))
        assert frontend['zlio/sensor.minimum_range'] == fusion['lidar_merge/min_range'] == 0.25
    for invalid in (0.19, 0.51, float('nan'), float('inf')):
        try:
            parameter_files(CONFIG / 'real', lidar_min_range=invalid)
        except ValueError:
            pass
        else:
            raise AssertionError(f'invalid lidar_min_range accepted: {invalid}')
    print('OK shared TF, platform overrides, single-LiDAR overrides and unknown-key rejection')
    return 0


if __name__ == '__main__':
    sys.exit(main())
