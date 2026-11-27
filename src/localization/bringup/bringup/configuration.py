"""Resolve common tuning plus small platform and single-LiDAR overrides."""
import copy
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml


MODULES = ('frontend', 'backend', 'lidar_fusion', 'lidar_calibration')
LIDAR_MIN_RANGE_BOUNDS = (0.2, 0.5)


def merge(base, override, prefix=''):
    """Merge known keys only: a misspelled override must fail before launch."""
    result = copy.deepcopy(base)
    for key, value in override.items():
        name = f'{prefix}.{key}' if prefix else key
        if key not in result:
            raise ValueError(f'unknown configuration key: {name}')
        if isinstance(result[key], dict):
            if not isinstance(value, dict):
                raise ValueError(f'expected a mapping at {name}')
            result[key] = merge(result[key], value, name)
        else:
            result[key] = copy.deepcopy(value)
    return result


def resolve(config_dir, module, lidar='dual'):
    """config_dir is config/real or config/sim; every result is a full ROS YAML."""
    directory = Path(config_dir)
    if directory.name not in ('real', 'sim') or module not in MODULES:
        raise ValueError(f'unsupported configuration: {directory}/{module}')
    if lidar not in ('single', 'dual'):
        raise ValueError('lidar must be single or dual')
    filename = f'{module}.yaml'
    result = yaml.safe_load((directory.parent / 'common' / filename).read_text())
    path = directory / 'overrides.yaml'
    overrides = yaml.safe_load(path.read_text()) if path.is_file() else {}
    unknown = set(overrides) - set(MODULES)
    if unknown:
        raise ValueError(f'unknown modules in {path}: {unknown}')
    result = merge(result, overrides.get(module, {}))
    if module == 'backend' and lidar == 'single':
        path = directory.parent / 'single_lidar.yaml'
        single = yaml.safe_load(path.read_text())
        if set(single) - {'common', 'sim', 'real'}:
            raise ValueError(f'unknown platform in {path}')
        result = merge(result, single['common'])
        result = merge(result, single.get(directory.name, {}))
    return result


def parameter_files(config_dir, modules=MODULES, lidar='dual', backend_overrides='', lidar_min_range=None):
    """Materialize resolved files for the standalone module launch interfaces.

    The caller keeps the directory alive until launch shutdown, then cleans it.
    Source configuration stays in common/ and its small explicit overrides.
    """
    if lidar_min_range is not None:
        minimum, maximum = LIDAR_MIN_RANGE_BOUNDS
        if not minimum <= lidar_min_range <= maximum:
            raise ValueError(f'近距软件过滤须在 {minimum} 到 {maximum} 米之间')
    temporary = TemporaryDirectory(prefix='yang_nav_params_')
    try:
        for module in modules:
            path = Path(temporary.name) / f'{module}.yaml'
            config = resolve(config_dir, module, lidar)
            if lidar_min_range is not None:
                if module == 'frontend':
                    config['zlio']['ros__parameters']['sensor']['minimum_range'] = float(lidar_min_range)
                elif module == 'lidar_fusion':
                    config['lidar_merge']['ros__parameters']['min_range'] = float(lidar_min_range)
            if module == 'backend' and backend_overrides:
                config = merge(config, yaml.safe_load(Path(backend_overrides).read_text()))
            path.write_text(yaml.safe_dump(config, sort_keys=False))
    except Exception:
        temporary.cleanup()
        raise
    return temporary
