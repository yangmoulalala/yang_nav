#!/usr/bin/env python3
"""检查 bringup 参数文件与各模块 example 的键集合完全一致（只比较键，不比较值）。

    python3 tools/check_bringup_params.py          # 在工作空间根目录运行

bringup/config/<platform>/<module>.yaml 是完整系统中该模块参数的唯一来源；模块自带的
config/example.yaml 只用于单独启动。两者按节点名、参数名展开后必须是同一组键，
否则说明新增/改名的参数只改了一处。存在差异时以退出码 1 结束。
"""
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1] / 'src' / 'z-localization'
MODULES = {
    'lidar_fusion': ROOT / 'lidar_fusion' / 'config' / 'example.yaml',
    'frontend': ROOT / 'frontend' / 'config' / 'example.yaml',
    'backend': ROOT / 'backend' / 'config' / 'example.yaml',
    'lidar_calibration': ROOT / 'lidar_calibration' / 'config' / 'example.yaml',
}
PLATFORMS = ('sim', 'real')
# 没有模块 example 可比的平台专属配置（第三方驱动等）：只检查存在且格式正确。
PLATFORM_ONLY = {'real': ('livox_driver.yaml', 'robot_state.yaml')}


def flat(path):
    """把 ROS 参数文件展开为 {'node/param.path'}。"""
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    out = set()

    def walk(prefix, value):
        if isinstance(value, dict):
            for k, v in value.items():
                walk(f'{prefix}.{k}', v)
        else:
            out.add(prefix)

    for node, section in data.items():
        params = (section or {}).get('ros__parameters')
        if not isinstance(params, dict):
            raise ValueError(f'{path}: section {node!r} has no ros__parameters mapping')
        for k, v in params.items():
            walk(f'{node}/{k}', v)
    return out


def main():
    failed = False
    for module, example in MODULES.items():
        if not example.is_file():
            print(f'MISSING {example}')
            failed = True
            continue
        reference = flat(example)
        for platform in PLATFORMS:
            target = ROOT / 'bringup' / 'config' / platform / f'{module}.yaml'
            if not target.is_file():
                print(f'MISSING {target}')
                failed = True
                continue
            actual = flat(target)
            missing, extra = sorted(reference - actual), sorted(actual - reference)
            if missing or extra:
                failed = True
                print(f'DIFF {target.relative_to(ROOT)} vs {example.relative_to(ROOT)}')
                for k in missing:
                    print(f'  - missing {k}')
                for k in extra:
                    print(f'  + extra   {k}')
            else:
                print(f'OK   {target.relative_to(ROOT)} ({len(actual)} keys)')
    for platform, names in PLATFORM_ONLY.items():
        for name in names:
            target = ROOT / 'bringup' / 'config' / platform / name
            try:
                print(f'OK   {target.relative_to(ROOT)} ({len(flat(target))} keys, no example)')
            except (OSError, ValueError) as error:
                print(f'BAD  {target}: {error}')
                failed = True
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
