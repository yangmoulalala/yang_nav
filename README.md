# yang_nav

ROS 2 Jazzy 哨兵机器人工作区：机器人描述与 Gazebo 仿真、单/双雷达定位、地形制作和 Nav2 导航。
环境为 Ubuntu 24.04；本机 WSL 路径为 `/home/yang/yang_nav`，Windows 入口为 `\\wsl.localhost\ubuntu24\home\yang\yang_nav`。
本文命令在 WSL 项目根目录执行。

## 从哪里开始

| 任务 | 入口 |
| --- | --- |
| 日常导航、发送目标、排查 | [导航使用指南](docs/navigation_usage.md) |
| 导航实现及当前完成边界 | [导航框架](docs/navigation.md)、[导航进度](docs/nav_plan.md) |
| 建图、定位、硬件依赖及传感器配置 | [定位系统](src/z-localization/README.md) |
| 真车建图与录包 | [真车操作](docs/real_mapping.md) |
| 仿真场地、机器人和遥控 | [Gazebo](src/sentry_gazebo/README.md)、[机器人描述](src/sentry_description/README.md) |
| 地形标注与导出 | [地形制作](src/nav_terrain/terrain_studio.md) |
| 检查、回归和诊断脚本 | [工具目录](tools/README.md) |
| 全部文档和验证记录 | [文档索引](docs/README.md)、[报告索引](reports/README.md) |

## 目录职责

```text
src/
  nav_behaviors/       区域规划、路径整形、控制器、BT 与代价图插件
  nav_bringup/         导航 launch、行为树、Nav2 参数与 RViz
  nav_safety/          最终速度命令门与安全判定
  nav_terrain/         地形标注、导出、地面投影与障碍过滤
  sentry_description/  URDF 与机器人网格，传感器外参唯一来源
  sentry_gazebo/       Gazebo 场地生成、传感器桥接、遥控及 UWB 仿真
  z-localization/      定位相关 ROS 包集合；本目录本身不是 ROS 包
docs/                 系统级操作与原理；history/ 保存历史审查
tools/                工作区级启动、地图转换、检查和回归入口
reports/              有日期的验证证据；COLCON_IGNORE 隔离构建发现
maps/                 本机地图及地形数据，Git 忽略
backup/               本机恢复快照，Git 忽略
build/ install/ log/   colcon 生成物，Git 忽略
```

`z-localization/` 下共有七个包：`frontend`、`backend`、`bringup`、`localization_interfaces`、
`lidar_fusion`、`lidar_calibration`、`livox_ros_driver2`。驱动中的第三方源码、许可及供应商说明属于依赖，
使用情况应按其构建入口判断。包内 `include/` 放公开 C++ 接口、`src/` 放实现、`apps/` 放 ROS 适配层、
`test/` 放测试；`launch/` 和 `config/` 或 `params/` 随包安装。运行入口读取安装目录中的配置，改源配置后须重建对应包。

## 构建与检查

先按[定位依赖](src/z-localization/README.md)及[导航准备](docs/navigation_usage.md#1-进入项目与准备环境)准备 ROS、Nav2、Gazebo、Livox-SDK2 等依赖。
从当前工作区构建全部 13 个包，限制并行以控制内存使用：

```bash
cd /home/yang/yang_nav
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j4 colcon build --base-paths src --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
python3 tools/check_bringup_params.py
```

需要运行测试时：

```bash
colcon test --base-paths src --return-code-on-test-failure
colcon test-result --verbose
```

colcon 结果目录可能保留已删除测试的旧 XML；统计当前测试数量时应对照当前 CMake 测试目标。
运动回归应使用独立 ROS domain，命令及验证范围见[导航回归](tools/nav_regress/README.md)和[定位仿真测试](tools/sim_tests/README.md)。

## 配置和状态约束

- 组合定位使用 `src/z-localization/bringup/config/{sim,real}/` 的完整参数文件；模块 `config/example.yaml` 用于独立运行。
- 组合启动关闭前端 `output.publish_tf`，开启后端 `common.publish_odom_tf`，由后端在同一消息中发布 `map → lio_odom → base_link`；不要重复发布同一条 TF 边。
- 导航需要有效地图、地形、观测、TF 和定位可信状态。已通过的单元测试或 fixture 回归不能代替真实地图、Gazebo 物理或真车验收，完成边界以[导航进度](docs/nav_plan.md)为准。
- 地图、构建产物和仍在采集的日志按各自入口维护；历史证据按日期归档到 `reports/`，有恢复用途的快照放 `backup/`。
