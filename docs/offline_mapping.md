# 点云离线优化

`src/offline_mapping_optimizer` 移植自 HWSentryNav26，保留局部子图和全局子图两阶段 VGICP 因子图优化、回环约束、raycasting 动态物体去除及 PCL 后处理。后端工具 `offline_optimizer_bundle` 和工作区脚本 `tools/optimize_map.sh` 将它接入本项目的地图包流程。

## 安装与构建

在 WSL Ubuntu 24.04、项目根目录执行。已有定位后端依赖之外，还需 `gtsam_points 1.2.0`；安装脚本固定上游提交 `00f9fca52b8a24abce3d7ce04f290f135a7615e6`，使用 `/opt/ros/jazzy` 的 GTSAM 4.2，安装到 Git 忽略的 `.deps/offline_optimizer/install`。脚本不修改系统 GTSAM，不需要 sudo；需要 GitHub 网络和现有 Eigen、Boost、OpenMP 开发库。

```bash
cd /home/yang/yang_nav
bash tools/install_offline_optimizer_deps.sh
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j2 colcon build --base-paths src \
  --packages-up-to backend offline_mapping_optimizer \
  --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

CMake 自动发现项目内依赖目录。默认 CPU 路径，不要求 CUDA；`JOBS=2` 控制依赖编译并行数。删除 `.deps` 后需重新安装依赖并重建优化器。

## 优化已有地图包

先完成建图并成功调用 `/save_map`，操作见[真车建图](real_mapping.md)。输入必须包含 `keyframes.txt`、`clouds/000000.pcd` 等原始局部关键帧，以及建图时保存的描述子。单张合并 PCD 缺少观测视点和分帧信息，不能作为完整离线优化输入。

仿真单雷达关键帧位于 `mid360_link`，光学原点为零：

```bash
bash tools/optimize_map.sh \
  --in maps/sim-rmuc-single --out maps/sim-rmuc-single-optimized --threads 4
```

真车单 MID-360 的关键帧位于 `mid360_imu_link`。按当前 URDF，雷达光学原点在该坐标系为 `(-0.011, -0.02329, 0.04412)` 米：

```bash
bash tools/optimize_map.sh \
  --in maps/real/session_001 --out maps/real/session_001_optimized \
  --lidar-origin -0.011 -0.02329 0.04412 --threads 4
```

双雷达融合云具有多个射线原点，当前地图包没有逐点传感器来源；使用 `--no-raycasting`，保留位姿优化和全局点云后处理。不能将它当作已适配双雷达的动态去除算法：

```bash
bash tools/optimize_map.sh \
  --in maps/sim-rmuc-dual --out maps/sim-rmuc-dual-optimized \
  --no-raycasting --threads 4
```

脚本在输出的父目录建立临时快照，原生 clouds 与 `frame_*.pcd` 通过硬链接共用快照文件。优化和导入读取同一份快照；结束时自动清理临时文件。输出必须不存在或为空，且位于输入目录之外。成功后输出：

| 文件 | 用途 |
| --- | --- |
| `keyframes.txt` | 优化后的传感器位姿，保留时间戳 |
| `clouds/*.pcd` | 原始局部坐标与 intensity，应用动态去除掩码 |
| `sc_descriptors.bin` / `gprobe_descriptors.bin` | 按输入中存在的类型和配置重新生成 |
| `map.pcd` | 合并、动态去除、降采样、离群过滤、MLS 后的全局点云 |
| `optimized_poses.txt` | 优化器原始 SE(3) 位姿结果 |
| `optimization.txt` | 原编号到新编号、每帧保留点数及总移除量 |
| `optimizer_params.yaml` / `optimizer.log` | 实际参数和完整执行日志 |

局部关键帧应用动态去除，不应用 MLS 或全局离群过滤，以保留实测几何；所以重新拼接关键帧不会逐点复现平滑后的 `map.pcd`。若一帧全部被清空，会删除该帧并连续重编号，然后重建描述子；编号对应关系记录在 `optimization.txt`。全部被清空、参数非法、优化异常或结果写入失败时返回非零，不发布最终地图包；失败日志保存为输出旁的 `<输出名>.optimization-failed.log`。

优化后的目录可作为现有定位入口的 `map_dir`。用于导航时，须根据优化后的 `map.pcd` 重新核对或生成 terrain；已有地图的地形标注不会自动随非刚性轨迹修正更新。

## 参数与原始格式入口

默认参数位于 `src/offline_mapping_optimizer/config/params.yaml`。默认 4 个 CPU 线程、20 帧一个局部子图、5 cm 动态判定体素、5 cm 全局降采样。`--threads` 同时覆盖外层、局部 VGICP、全局 VGICP 线程数；`--no-mls` 关闭全局平滑。需要其他参数时复制 YAML，保留完整参数集合：

```bash
cp src/offline_mapping_optimizer/config/params.yaml /tmp/offline_params.yaml
# 编辑 /tmp/offline_params.yaml
bash tools/optimize_map.sh --in maps/your_map --out maps/your_map_optimized \
  --config /tmp/offline_params.yaml
```

内存和运行时间随帧数、局部帧对数量、体素分辨率增长。原算法的 `pass_through_threshold` 统计穿过体素的射线条数，并非独立关键帧数；密集射线也可能造成误删，应对比输出再用于定位。可关闭动态去除单独检查位姿修正。

仍支持 HWSentryNav26 的 `poses.txt + frame_*.pcd` 原始目录。直接启动只生成优化器结果，不重建本项目地图包，`output_path` 必须是输入之外的新目录：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch offline_mapping_optimizer offline_mapping_optimizer.launch.py \
  data_path:=/absolute/raw_frames output_path:=/absolute/new_results
```

## 验证

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTHONDONTWRITEBYTECODE=1 BACKEND_BINARY_DIR="$PWD/build/backend" \
  python3 -m pytest src/z-localization/backend/test/tools/test_map_tools.py \
  tools/test_offline_optimizer.py -v
```

验证包括实际 VGICP 局部/全局优化后的相对漂移、动态点从局部关键帧移除、两种描述子重建、地图包重载、输入保留、非法配置和掩码失败时不提交结果。验证使用合成数据，不代表真车地图质量或 CUDA 已验收。
