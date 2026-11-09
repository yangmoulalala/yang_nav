#!/usr/bin/env python3
"""检查 bringup 参数文件：键集合与模块 example 一致，且各份完整配置之间只在白名单内取值不同。

    python3 tools/check_bringup_params.py          # 在工作空间根目录运行

参数文件不支持继承或合并，每份 bringup/config/<platform>/<module>.yaml 都是该模块参数的完整、唯一来源。
代价是共用参数分散在多份文件中，本脚本负责发现“只改了一份”：

1. 键集合：每份 bringup 文件与模块 config/example.yaml 按节点名、参数名展开后必须是同一组键；
2. 取值：VALUE_PAIRS 中每对文件只允许 ALLOWED 白名单内的键取值不同；白名单中取值已经相同的键
   也报错，保证白名单始终等于实际差异。

有意让某个参数在平台或雷达配置之间不同，就把它和原因加进 ALLOWED。存在问题时以退出码 1 结束。
"""
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1] / 'src' / 'z-localization'
CONFIG = ROOT / 'bringup' / 'config'
MODULES = {
    'lidar_fusion': ROOT / 'lidar_fusion' / 'config' / 'example.yaml',
    'frontend': ROOT / 'frontend' / 'config' / 'example.yaml',
    'backend': ROOT / 'backend' / 'config' / 'example.yaml',
    'lidar_calibration': ROOT / 'lidar_calibration' / 'config' / 'example.yaml',
}
PLATFORMS = ('sim', 'real')
# 同一模块按运行选择使用的多份完整配置（如单/双雷达后端），每份都须与 example 键集合一致。
VARIANTS = {'backend': ('backend.yaml', 'backend_single.yaml')}
# 没有模块 example 可比的平台专属配置（第三方驱动等）：只检查存在且格式正确。
PLATFORM_ONLY = {'real': ('livox_driver.yaml', 'robot_state.yaml')}

# 仅限于平台或雷达配置的取值差异。键为 'sim/x.yaml' 与 'real/x.yaml' 形式的文件对，值为 {参数名: 原因}。
SIM_IMU = 'Gazebo IMU 位于 mid360_link 原点；真车为 MID-360 内置 IMU mid360_imu_link'
SIM_CONVERGENCE = '仿真点云稀疏，真车沿用上游收敛判据'
BACKEND_PLATFORM = {
    'backend_node/common.sensor_frame': SIM_IMU,
    'backend_node/registration.max_iterations': SIM_CONVERGENCE,
    'backend_node/registration.convergence_translation': SIM_CONVERGENCE,
    'backend_node/registration.convergence_rotation_deg': SIM_CONVERGENCE,
    'backend_node/uwb.enabled': '仿真有 uwb_sim；真车 T_map_uwb 未标定，先关闭',
    'backend_node/uwb.time_offset_seconds': '仿真 uwb_sim 延迟 0.2 s；真车待标定',
    'backend_node/uwb.zero_pose_red.x': '仿真取 maps/sim 的 T_map_uwb；真车按地图包另设',
    'backend_node/uwb.zero_pose_red.y': '同上',
    'backend_node/uwb.zero_pose_red.yaw_deg': '同上',
    'backend_node/uwb.zero_pose_blue.x': '同上',
    'backend_node/uwb.zero_pose_blue.y': '同上',
    'backend_node/uwb.zero_pose_blue.yaw_deg': '同上',
}
SINGLE_LIDAR = {
    'backend_node/common.scan_context.similarity_normalization':
        '侧装单雷达单侧视野须用 union',
    'backend_node/tracking.source.max_frames': '侧装单雷达单帧点数不足，累积更多帧',
    'backend_node/tracking.source.max_time': '同上',
}
SIM_SINGLE_VOXEL = {
    'backend_node/registration.voxel_resolution':
        '仿真单雷达累积后 0.4 m 体素只剩约 136 点',
}
ALLOWED = {
    ('sim/backend.yaml', 'real/backend.yaml'): BACKEND_PLATFORM,
    ('sim/backend_single.yaml', 'real/backend_single.yaml'): {**BACKEND_PLATFORM, **SIM_SINGLE_VOXEL},
    ('real/backend.yaml', 'real/backend_single.yaml'): SINGLE_LIDAR,
    ('sim/backend.yaml', 'sim/backend_single.yaml'): {**SINGLE_LIDAR, **SIM_SINGLE_VOXEL},
    ('sim/frontend.yaml', 'real/frontend.yaml'): {
        'zlio/frames.imu': SIM_IMU,
        'zlio/input.imu_acceleration_scale': 'gz IMU 单位为 m/s²，MID-360 为 g',
        'zlio/sensor.minimum_range': '仿真雷达量程设置',
        'zlio/sensor.maximum_range': '仿真雷达量程设置',
        'zlio/initialization.minimum_directional_support': '仿真出生点静止视野几何单一',
        'zlio/input.maximum_imu_interval': 'WSL 下仿真+定位+导航同时运行时 IMU 回调偶发被饿住超过 0.1 s，仿真放宽到 0.3 s',
    },
    ('sim/lidar_fusion.yaml', 'real/lidar_fusion.yaml'): {
        'lidar_merge/primary.time_field': 'gpu_lidar 整帧同一时刻；真车用驱动逐点时间',
        'lidar_merge/primary.time_absolute': '同上',
        'lidar_merge/secondary.time_field': '同上',
        'lidar_merge/secondary.time_scale': '同上',
    },
    ('sim/lidar_calibration.yaml', 'real/lidar_calibration.yaml'): {
        'calibrate/frames': '仿真场地只有约 4 处几何足够丰富',
    },
}


def flat(path):
    """把 ROS 参数文件展开为 {'node/param.path': value}。"""
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    out = {}

    def walk(prefix, value):
        if isinstance(value, dict):
            for k, v in value.items():
                walk(f'{prefix}.{k}', v)
        else:
            out[prefix] = value

    for node, section in data.items():
        params = (section or {}).get('ros__parameters')
        if not isinstance(params, dict):
            raise ValueError(f'{path}: section {node!r} has no ros__parameters mapping')
        for k, v in params.items():
            walk(f'{node}/{k}', v)
    return out


def same(a, b):
    """按 ROS 语义比较取值：整数写法与等值浮点视为相同，布尔不与数值混同。"""
    if isinstance(a, bool) or isinstance(b, bool):
        return type(a) is type(b) and a == b
    return a == b


def check_keys():
    """每份 bringup 文件的键集合须与模块 example 一致。"""
    failed = False
    for module, example in MODULES.items():
        if not example.is_file():
            print(f'MISSING {example}')
            failed = True
            continue
        reference = set(flat(example))
        for platform, filename in ((p, f) for p in PLATFORMS
                                   for f in VARIANTS.get(module, (f'{module}.yaml',))):
            target = CONFIG / platform / filename
            if not target.is_file():
                print(f'MISSING {target}')
                failed = True
                continue
            actual = set(flat(target))
            missing, extra = sorted(reference - actual), sorted(actual - reference)
            if missing or extra:
                failed = True
                print(f'DIFF {target.relative_to(ROOT)} vs {example.relative_to(ROOT)}')
                for k in missing:
                    print(f'  - missing {k}')
                for k in extra:
                    print(f'  + extra   {k}')
            else:
                print(f'OK   keys   {target.relative_to(ROOT)} ({len(actual)} keys)')
    for platform, names in PLATFORM_ONLY.items():
        for name in names:
            target = CONFIG / platform / name
            try:
                print(f'OK   keys   {target.relative_to(ROOT)} ({len(flat(target))} keys, no example)')
            except (OSError, ValueError) as error:
                print(f'BAD  {target}: {error}')
                failed = True
    return failed


def check_values():
    """每对文件只允许白名单内的键取值不同，且白名单不得包含已经相同的键。"""
    failed = False
    for (left_name, right_name), allowed in ALLOWED.items():
        left_path, right_path = CONFIG / left_name, CONFIG / right_name
        if not (left_path.is_file() and right_path.is_file()):
            continue  # 缺失已在键集合检查中报告。
        left, right = flat(left_path), flat(right_path)
        shared = sorted(set(left) & set(right))
        differing = {k for k in shared if not same(left[k], right[k])}
        unexpected = sorted(differing - set(allowed))
        stale = sorted(set(allowed) - differing)
        label = f'{left_name} vs {right_name}'
        if not unexpected and not stale:
            print(f'OK   values {label} ({len(differing)} allowed differences)')
            continue
        failed = True
        print(f'DIFF values {label}')
        for k in unexpected:
            print(f'  ! {k}: {left[k]!r} vs {right[k]!r}'
                  ' (只改了一份？有意不同则加入 ALLOWED 并写明原因)')
        for k in stale:
            shown = repr(left.get(k, '<missing>'))
            print(f'  ? {k}: 两份均为 {shown}，从 ALLOWED 中删去该项')
    return failed


def main():
    failed = check_keys()
    failed = check_values() or failed
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
