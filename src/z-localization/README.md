# Z-Localization

### 2026年东北大学T-DT实验室 RoboMaster超级对抗赛 哨兵定位开源

![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)
![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)
![Mapping & Localization](https://img.shields.io/badge/Mapping-Localization-536A8A)
![GPL-2.0-only](https://img.shields.io/badge/License-GPL--2.0--only-287A6B)

Z-Localization 是一套面向移动机器人的 **ROS 2 激光惯性定位栈**，支持里程计、点云建图、先验地图定位、
回环与重定位，也可接入 UWB 提供位置辅助。实机运行与 rosbag 回放使用同一套入口，输出位姿、速度、协方差和定位状态。

项目包含 ZLIO 前端、定位后端、统一启动入口，以及地图处理和车体外参标定工具。
ZLIO 可以单独作为里程计使用，后端也提供独立里程计与点云接口，方便适配已有的 LIO 前端及奥丁之眼等定位模块。

地图制作也在项目范围内：保存实测地图、清理动态残影、配合 CloudCompare 精修，再生成用于定位的先验地图包。

前后端的设计思路、尝试和取舍，整理在[设计说明](docs/design.md)中。

由于作者的代码质量实在有点拉胯，项目借助 AI 做过整理与优化，并经过人工审查。建议配合 AI 食用，发现问题欢迎提 Issue 或 PR。

如果这个项目对你有帮助，欢迎点个 Star ⭐，给作者一点更新动力（虽然大概率不会再更新了，除非有重大 bug，相信后人的智慧 (￣▽￣)ゞ）。

[环境与依赖](#环境与依赖) · [配置示例](#配置示例) · [无先验地图的实时定位](#无先验地图的实时定位) · [从 bag 建图到先验定位](#从-bag-建图到先验定位) · [更多文档](#更多文档) · [联系作者](#联系作者)

## 环境与依赖

| 类别 | 环境 / 依赖 |
| --- | --- |
| 系统与 ROS | Ubuntu 22.04、ROS 2 Humble |
| 编译与构建 | C++17、CMake ≥ 3.16、ament_cmake、colcon |
| C++ 库 | Eigen ≥ 3.3、GTSAM 4.2.x、PCL、yaml-cpp、nanoflann |
| 并行支持 | OpenMP C++、oneTBB、Threads |
| 驱动依赖 | Livox ROS Driver 2、Livox-SDK2、APR |
| 启动与可视化 | ROS 2 launch、PyYAML、RViz2 |

完整 ROS 包与测试依赖见[前端说明](src/frontend/README.md#环境与依赖)和[后端说明](src/backend/README.md#环境与依赖)。

## 配置示例

配置入口为 [config/lio.yaml](config/lio.yaml) 和 [config/backend.yaml](config/backend.yaml)。
仓库中的安装外参、场地路径和 UWB 设置来自作者的设备，不是通用标定结果。

> 参数确实不少，作者对此负全责 (；´д｀)ゞ。请务必认真看[前端调参指南](src/frontend/docs/parameters.md#调参建议)和[后端调参指南](src/backend/docs/parameters.md#调参建议)，尤其是传感器、外参与时间相关的部分。

bag 建图流程以 **Mid-360 + Livox CustomMsg、关闭 UWB、使用 Scan Context** 为例。
场地配置 `pcd/site.yaml`：

```yaml
extends: ../config/backend.yaml
uwb:
  enabled: false
trajectory:
  save_directory: pcd/site_raw
  save_global_pcd: true
  save_on_shutdown: false
```

## 无先验地图的实时定位

没有先验地图、又需要实时全局位姿时，使用 `mode:=mapping`，将场地配置中的 `uwb.enabled` 设为 `true` 并接入 UWB。
实机运行使用 `use_sim_time:=false`，需要一并启动 Mid-360 驱动时追加 `driver:=true`。

正式使用定位前，先启动程序并让机器人移动几米（默认需要至少 `3 m` 的水平位移跨度），
待定位状态进入 `NORMAL` 后再使用全局位姿。UWB 配置见[调参指南](src/backend/docs/parameters.md#uwb-参数)。

## 从 bag 建图到先验定位

**回放建图 → 保存地图 → 自动清理 → CloudCompare 精修 → 网格生成地图包与描述子 → 先验定位。**

以下命令在项目根目录执行，使用上面的 `pcd/site.yaml`。

### 1. 建图与回放

```bash
ros2 launch bringup bringup.launch.py \
  frontend_config:="$PWD/config/lio.yaml" \
  backend_config:="$PWD/pcd/site.yaml" \
  mode:=mapping use_sim_time:=true rviz:=true
```

另一个终端播放 bag：

```bash
ros2 bag play /absolute/path/to/bag --clock
```

从头重播前，重新启动定位程序。

### 2. 保存地图

回放结束后停止输入，待末尾观测和地图更新完成再保存：

```bash
ros2 service call /stop std_srvs/srv/Trigger "{}"

# 地图更新完成后执行
ros2 service call /backend_node/save_map std_srvs/srv/Trigger "{}"
```

返回 `success: true` 后再退出节点，结果在 `pcd/site_raw/`。
保留完整地图包，后续清理需要其中的关键帧和局部点云；同一目录再次保存会更新原有结果。

### 3. 自动清理动态残影

```bash
ros2 run backend clean_map_bundle \
  --in "$PWD/pcd/site_raw" \
  --out "$PWD/pcd/site_auto_clean" \
  --lidar-origin -0.011 -0.02329 0.04412
```

这里的雷达光心位置对应默认 Mid-360 外参，在 IMU 系中表达。
清理及后续生成工具的输出目录须不存在或为空。

过滤思路参考 [FreeDOM](https://github.com/LC-Robotics/FreeDOM)。按作者的实测体验，这一步非常有效，
能大幅减少手工清理的工作量，但结果仍不足以直接使用（尤其是在适应性训练阶段如此恶劣的环境下）。

### 4. CloudCompare 手工收尾

用 [CloudCompare](https://www.cloudcompare.org/) 打开 `pcd/site_auto_clean/map.pcd`，去掉剩余动态残影和噪点，
调平地图并调整世界系原点，另存为 `pcd/site_refined.pcd`。

### 5. 网格生成地图包与描述子

**雷达水平安装时，作者建议精修后重新按网格生成。** 按作者的使用经验，这样生成的描述子通常比建图时直接保存的质量更好。

下面以 X：`0–28 m`、Y：`0–15 m`、采样 Z 坐标 `0 m`、雷达离地高度 `0.35 m` 为例，范围和高度按调整后的地图填写。

```bash
ros2 run backend build_pcd_map_bundle \
  --input "$PWD/pcd/site_refined.pcd" \
  --out-bundle "$PWD/pcd/site_prior" \
  --x-min 0 --x-max 28 --y-min 0 --y-max 15 \
  --sensor-z 0.0 --bundle-step 1.5 \
  --map-voxel 0.15 --scan-voxel 0.4 \
  --descriptor-type scan_context --sc-lidar-height 0.35
```

`--sensor-z` 是地图中的采样高度坐标，`--sc-lidar-height` 是离地高度，两者不一定相等。
本例按水平姿态、固定高度生成；描述子配置须与定位时一致，其余选项见[工具参数](src/backend/docs/parameters.md#离线工具参数)。

### 6. 先验地图定位

```bash
ros2 launch bringup bringup.launch.py \
  frontend_config:="$PWD/config/lio.yaml" \
  backend_config:="$PWD/pcd/site.yaml" \
  mode:=localization prior_map:="$PWD/pcd/site_prior" \
  use_sim_time:=true rviz:=true
```

另一个终端回放验证：

```bash
ros2 bag play /absolute/path/to/bag --clock
```

实机运行时使用 `use_sim_time:=false`，追加 `driver:=true` 可一并启动 Livox Mid-360 驱动。
`driver` 默认 `false`，回放 bag 或已单独启动驱动时保持关闭；驱动使用 [MID360_config.json](src/livox_ros_driver2/config/MID360_config.json) 中的网络配置，输出 Livox CustomMsg。

## 更多文档

| 文档 | 内容 |
| --- | --- |
| [设计说明](docs/design.md) | 前后端设计思路、方案探索与取舍 |
| [前端说明](src/frontend/README.md) | 独立运行、输入格式与对外接口 |
| [前端参数与调参指南](src/frontend/docs/parameters.md) | 传感器、初始化、采样与地图参数 |
| [后端说明](src/backend/README.md) | 定位输出、服务、状态与地图工具 |
| [后端参数与调参指南](src/backend/docs/parameters.md) | 外参、UWB、场景参数与工具选项 |
| [接口说明](src/localization_interfaces/README.md) | 前后端消息与重置服务 |

## 联系作者

作者：mozihe

- 邮箱：[zhujunheng2005@gmail.com](mailto:zhujunheng2005@gmail.com)
- QQ：820288431

## 许可证

本项目采用 [GPL-2.0-only](LICENSE)。第三方组件保留各自许可证。
