# offline_mapping_optimizer

从 [HWSentryNav26](https://github.com/Polyacetone/HWSentryNav26) 的 `utils/cpp/offline_mapping_optimizer` 移植，原始提交 `f5f941288197e14c867d711a4c4cd85bfd7a3194`，保留上游 MIT 许可（见 LICENSE）。原作者维护者：@Polyacetone。

保留两阶段 VGICP 因子图、子图间回环、raycasting、降采样、离群过滤与 MLS。适配点：

- 兼容 yang_nav 的 ROS 2 Jazzy / GTSAM 4.2，独立安装固定的 gtsam_points 1.2.0。
- 使用标准 ament_cmake 和显式依赖，移除上游未提供的 common_libs 依赖和编译器硬编码。
- 输入与输出目录分离，参数和点云检查，失败返回非零，位姿以 17 位精度输出。
- 导出逐关键帧保留索引，由 backend 的 offline_optimizer_bundle 恢复局部 XYZ/I、更新位姿和重建两类描述子。
- 可配置雷达在局部云坐标系中的光学原点；默认 CPU、4 线程、5 cm raycasting 体素。

## 安装和使用

```bash
bash tools/install_offline_optimizer_deps.sh
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to offline_mapping_optimizer --symlink-install
source install/setup.bash
bash tools/optimize_map.sh --in maps/source --out maps/optimized --threads 4
```

`tools/optimize_map.sh` 加载工作区环境，调用 `ros2 run offline_mapping_optimizer optimize_map.py`。`--in` 读取后端地图包，`--out` 指向尚不存在的目录。程序保留输入文件，失败日志写在输出目录旁。需要覆盖参数时用 `--config` 选择 YAML，选项见 `--help`。[默认参数](config/params.yaml)、[端到端测试](test/test_map_bundle.py)

Raycasting 使用局部点云坐标中的单一光学原点，`--lidar-origin X Y Z` 可覆盖。合并的双雷达点云包含不同的射线原点，当无法提供可靠的单一原点时可用 `--no-raycasting`。

原始格式 ROS 入口：

```bash
ros2 launch offline_mapping_optimizer offline_mapping_optimizer.launch.py \
  data_path:=/absolute/raw_frames output_path:=/absolute/new_results
```

默认未编译 CUDA，CPU 路径已验证。可选 `MAP_OPTIMIZER_USE_CUDA` 与 `MAP_OPTIMIZER_USE_ABSL` 保留供有相应依赖的机器使用；本次移植没有 GPU 验收。
