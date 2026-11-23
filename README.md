# yang_nav

Ubuntu 24.04 / ROS 2 Jazzy 机器人导航工作区。单雷达或双雷达与 IMU 提供定位，人工标注的地形提供静态通行约束，Nav2 根据实时障碍规划和控制，最终速度送往 Gazebo 或真车底盘。

[使用手册：仿真与真车启动、建图、录包、地图优化和地形标注](docs/user-guide.md)

## 目录与入口

| 目录 | 职责 | 阅读入口 |
| --- | --- | --- |
| `src/platform/` | 机器人 URDF、Gazebo 数据源、真车底盘通信 | [机器人描述](src/platform/sentry_description/README.md)、[仿真](src/platform/sentry_gazebo/README.md)、[底盘接口](src/platform/chassis_comm/README.md) |
| `src/localization/` | 雷达驱动与融合、外参标定、激光惯性前端、定位或建图后端、组合启动 | [定位入口](src/localization/README.md) |
| `src/navigation/` | 地形制作与观测、Nav2 插件、行为树、速度安全门 | [地形](src/navigation/nav_terrain/README.md)、[插件](src/navigation/nav_behaviors/README.md)、[导航启动](src/navigation/nav_bringup/launch/navigation.launch.py) |
| `src/mapping/` | 离线优化关键帧位姿、过滤动态点、重建地图包 | [离线优化器](src/mapping/offline_mapping_optimizer/README.md) |
| `tools/` | 跨包启动、录包、诊断、仿真及导航回归 | [工具索引](tools/README.md) |

跨包调用链、话题和数据格式见[系统架构](docs/architecture.md)。`maps/` 和 `bags/` 存放本机地图与录包，`.deps/` 存放工作区依赖，`build/`、`install/`、`log/` 由 colcon 生成。

## 构建与检查

在 WSL `ubuntu24` 的 `/home/yang/yang_nav` 执行。首次准备环境需要 ROS 2 Jazzy、Nav2、Gazebo Harmonic、Livox-SDK2 和地图优化依赖。离线优化器使用 GTSAM 4.2 与项目内固定版本的 gtsam_points。[定位依赖](src/localization/README.md)、[优化器依赖](src/mapping/offline_mapping_optimizer/README.md)

```bash
source /opt/ros/jazzy/setup.bash
bash tools/install_offline_optimizer_deps.sh  # 首次安装 gtsam_points
colcon list --base-paths src
MAKEFLAGS=-j4 CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build --base-paths src \
  --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
python3 tools/check_bringup_params.py
ROS_DOMAIN_ID=157 QT_QPA_PLATFORM=offscreen colcon test --base-paths src \
  --executor sequential --return-code-on-test-failure
colcon test-result --verbose
```

从旧目录布局迁移时先移开旧 `build/` 和 `install/` 再构建。CMake 缓存和 symlink 安装都记录了源码路径。构建发现范围固定为 `src/`，其他目录可能带有快照或同名上游包。

## 运行导航

仿真使用三个已加载 ROS 和工作区环境的终端，设相同的 `ROS_DOMAIN_ID`。下面的例子使用 RMUC 场地，要求 `maps/sim` 下有与出生位置、传感器配置匹配的先验地图及地形导出。

```bash
ros2 launch sentry_gazebo sim.launch.py arena:=rmuc uwb:=true               # 终端 1
ros2 launch bringup sim.launch.py lidar:=dual mode:=localization           # 终端 2
bash tools/start_navigation.sh terrain_dir:=maps/sim/terrain rviz:=true    # 终端 3
```

真车数据源由 `bringup real.launch.py` 启动，包含底盘串口节点。准备好车上驱动、外参、串口和对应的先验地图后使用 `profile:=real` 启动导航。只检查参数时用 `ros2 launch bringup real.launch.py --show-args` 或 `bash tools/start_navigation.sh --dry-run profile:=real`。

回归使用独立 ROS domain 和输出目录。软件闭环、场地物理与真车验收各有对应的输入和判定条件。[导航回归](tools/nav_regress/README.md)、[定位仿真](tools/sim_tests/README.md)
