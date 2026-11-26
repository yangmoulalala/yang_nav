# 系统架构与调用链

`src/` 下的四个目录按职责分组。colcon 递归发现其中的 15 个 ROS 包，launch 通过 ament 索引定位已安装的包。顶层目录名用于阅读源码，ROS 客户端使用 `package.xml` 中的包名。

## 定位与建图

```text
仿真 sentry_gazebo/sim.launch.py              真车 bringup/real.launch.py
  Gazebo → ROS bridge + /clock                  Livox 驱动 + 可选 L2 驱动
  robot_state_publisher                         robot_state_publisher + chassis_comm
                └───────── 雷达点云、IMU、URDF 外参 ─────────┘
bringup/{sim,real}.launch.py → localization.launch.py
  ├─ lidar_fusion/merge.launch.py      → /lidar/points
  ├─ frontend/frontend.launch.py       ← /lidar/points、IMU
  │    zlio_node                       → /frontend/frame、/frontend/odometry、/frontend/session
  └─ backend/backend.launch.py         ← 上述消息、运行模式、先验地图
       backend_node                    → /localization/odometry、/backend_node/localization_state
                                       → map → lio_odom → gimbal_nav
```

`localization_interfaces` 定义前后端消息及重置服务。组合启动时前端关闭 TF 发布，后端用同一时间戳、同一消息发布两条动态边。`gimbal_nav` 到各传感器的刚性外参由共用 URDF 提供。前端与后端的 C++ 核心在各自 `src/` 下，`apps/` 负责 ROS 消息、参数和发布。[组合入口](../src/localization/bringup/launch/localization.launch.py)、[接口](../src/localization/localization_interfaces/README.md)、[前端](../src/localization/frontend/README.md)、[后端](../src/localization/backend/README.md)

`mode:=mapping` 建图，保存关键帧位姿和点云。离线优化入口调用 `backend/offline_optimizer_bundle` 导出快照，再调用 `offline_mapping_optimizer_node` 优化，最后由同一个后端工具重建可加载的先验地图。`mode:=localization` 用 `prior_map` 或 `map_dir` 载入该地图。[离线优化器](../src/mapping/offline_mapping_optimizer/README.md)

## 地形、规划与速度输出

离线标注从 `Project.from_mesh()` 或 `Project.from_cloud()` 进入，依次生成高程栅格、面片和人工标注。`Project.export()` 将标注转换为 `TerrainMap`，再由 `terrain_export.export()` 生成下游文件。[模块与数据格式](../src/navigation/nav_terrain/README.md#3-处理流程)

```text
场地网格 / 先验点云 → WebUI 地形标注 → 人工标签与通行方向 → terrain_export
  ├─ nav_map.yaml、speed_mask.yaml → Nav2 地图服务、代价图及速度限制
  ├─ zones.yaml                    → 路径整形、区域跟踪、行为树恢复保护
  └─ terrain.msgpack + /frontend/frame + TF → terrain_obstacles → 障碍点与清除射线

NavigateToPose / NavigateThroughPoses → bt_navigator
  → planner_server → smoother_server → controller_server / behavior_server
  → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed
  → collision_monitor → /cmd_vel_collision_checked → cmd_gate → /cmd_vel
  → 仿真 chassis_sim → MecanumDrive，或真车 chassis_serial
```

`nav_bringup` 组合地图服务、Nav2 生命周期节点、地形节点和速度门。两个行为树共用插件，`StartAwarePlanner` 委托内部规划器，`ZonePathShaper` 整形路径，`ZoneAwareController` 使用区域跟踪器和内部 MPPI 控制器。`nav_behaviors/src/core` 负责算法，`src/plugins` 对接 Nav2，`src/bt` 实现行为树节点。[导航入口](../src/navigation/nav_bringup/launch/navigation.launch.py)、[行为树](../src/navigation/nav_bringup/behavior_trees)、[插件接口](../src/navigation/nav_behaviors/README.md)

速度门消费定位可信状态、观测健康状态与速度命令，检查新鲜度及限速后输出最终 `Twist`。定位状态和坐标变换由后端负责。真车底盘将最终命令编码为串口帧。[地形节点](../src/navigation/nav_terrain/nav_terrain/terrain_obstacles.py)、[安全门](../src/navigation/nav_safety/src/cmd_gate.cpp)、[底盘接口](../src/platform/chassis_comm/README.md)

## 配置和测试归属

| 变更 | 配置来源 | 对应检查 |
| --- | --- | --- |
| 雷达选择、定位参数、TF 归属 | `src/localization/bringup/config/common` 与平台/单雷达差异，模块示例供单独运行 | `python3 tools/check_bringup_params.py`、前后端各包的 `test/` |
| 传感器安装位姿 | `sentry_description/urdf/sentry_robot.urdf` | 场地生成、[定位仿真](../tools/sim_tests/README.md) |
| 地形格式、导出、障碍过滤 | `nav_terrain/nav_terrain` 和导出的地形目录 | `nav_terrain/test/`，`tools/nav_regress/terrain_check.py` |
| Nav2 控制、速度门、行为树 | `nav_bringup/params/nav2.yaml`、`nav_bringup/behavior_trees` | `nav_behaviors/test/`、`nav_safety/test/`、[闭环回归](../tools/nav_regress/README.md) |
| 串口编码与超时 | `chassis_comm` 及 `bringup/real.launch.py` 的 `chassis_*` 参数 | `chassis_comm/test/`、PTY 控制频率回归 |
| 地图优化 | `offline_mapping_optimizer/config/params.yaml` | 包内 `test/test_map_bundle.py` 用真实二进制写出并重新载入地图 |

运动学回归用最终 `/cmd_vel` 积分模拟底盘。2026-11-23 对本机 `maps/sim/terrain` 的复测显示，隧道标签缺少导出的 `clearance`，6 条有效隧道的双向严格整形通过 8/12，`tunnel_2` 和 `tunnel_5` 双向失败。场地坡道、净高、传感器误差与真车制动仍需物理验收。

历史测试记录及其场景条件可由 `git show ce8fac9:reports/README.md` 查阅。本机重构前的报告归档于 `backup/layout-before-20261123/reports/`。
