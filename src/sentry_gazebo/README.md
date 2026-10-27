# sentry_gazebo — RMUL 2027 哨兵仿真环境

用 Gazebo Sim 8（gz-sim，ROS Jazzy）搭建的 RMUL 2027 赛场仿真。机器人几何与传感器
安装位姿**全部从 `src/sentry_description/urdf/sentry_robot.urdf` 读取**，赛场几何与
雷达参数在 `sentry_gazebo/arena_spec.py`，`sentry_gazebo` 只负责把它们组装成一个
可运行的 world。

位姿、TF、关节状态、点云全部由 gz 内置 system 或标准 ROS 2 包提供；自定义代码只有
URDF→SDF 生成、平面速度插件和一个键盘遥控节点（`ros2 run sentry_gazebo teleop`）。

## 快速开始

```bash
cd /home/yang/yang_nav
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sentry_gazebo
source install/setup.bash

# GUI + 两路点云面板
ros2 launch sentry_gazebo sim.launch.py gui:=true

# 只跑 server（无 GUI），传感器仍由 ogre2 headless 渲染
ros2 launch sentry_gazebo sim.launch.py

# 额外打开 RViz 看两路点云 + TF
ros2 launch sentry_gazebo sim.launch.py gui:=true rviz:=true
```

启动参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `gui` | `false` | 是否打开 Gazebo GUI；`true` 时追加两路 PointCloud 面板 |
| `rviz` | `false` | 是否同时启动 RViz2（`rviz/sentry.rviz`） |
| `generated_dir` | `$SENTRY_GAZEBO_GEN` 或 `/tmp/sentry_gazebo_generated` | 生成的 world 输出目录 |

点云只由 Gazebo 内置的 `gpu_lidar` 产生（ogre2 光线投射），没有其他后端。

### 键盘遥控

另开一个终端：

```bash
ros2 run sentry_gazebo teleop
```

| 按键 | 动作 |
| --- | --- |
| `w` / `s` | 前进 / 后退 |
| `a` / `d` | 左转 / 右转 |
| `q` / `e` | 左平移 / 右平移（四轮全向底盘） |
| `space` | 停止 |
| `+` / `-` | 加速 / 减速 |
| `x` | 退出 |

参数：`-p linear_speed:=0.5 -p angular_speed:=1.0 -p cmd_vel_topic:=/cmd_vel`。
节点以 20 Hz 重复发布，因为速度插件对指令有 0.4 s 超时，单条消息不会让车一直走。

stdin 不是终端时（管道、launch）改为**每行一个按键、保持 `hold` 秒**，方便脚本化：

```bash
printf 'w\nw\na\n' | ros2 run sentry_gazebo teleop -p hold:=2.0
```

### GPU 加速（WSL 必读）

`gpu_lidar` 是 GPU 光线投射，渲染器选错卡会直接把仿真拖慢一个数量级。WSL 的
Mesa `d3d12` 驱动默认挑**枚举到的第一块适配器**，在核显 + 独显的机器上就是核显。
用 `glxinfo` 确认当前用的是哪块：

```bash
glxinfo -B | grep Device
# 好：Device: D3D12 (NVIDIA GeForce RTX 4060 Ti)
# 坏：Device: D3D12 (AMD Radeon(TM) Graphics)      <- 核显，仿真会很卡
```

如果是核显，在 `~/.bashrc` / `~/.zshrc` 里指定独显（按名字子串匹配，没有匹配项时
自动回退，换机器也不会坏）：

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

本机实测（`gui:=true`，同一 world，15 s 采样）：

| 适配器 | 仿真时间推进 | RTF |
| --- | --- | --- |
| AMD 核显（默认） | 6.3 s / 15 s | **0.42** |
| RTX 4060 Ti（上面的变量） | 15.0 s / 15 s | **1.00** |

非交互式调用（`wsl -e bash -c ...`、VS Code task、Windows 快捷方式）不读
`.bashrc`，需要写进 `/etc/profile.d/*.sh` 或在命令前显式加变量。

### 多个 Gazebo 实例会导致仿真时间乱跳

如果上一个 `gz sim` 没有正常退出（例如 launch 被强杀），残留的 server 仍在发布
`/clock` 和激光雷达话题。此时新启动的实例会同时收到两套时钟，`sim` 时间来回跳变，
话题上也会出现多个发布者。启动前确认没有残留：

```bash
pkill -9 -f 'gz sim'; pkill -9 -f parameter_bridge
```

手动复现同一环境：

```bash
. /tmp/sentry_gazebo_generated/env.sh     # 由 prepare_world.py 生成
gz sim -r -v 3 /tmp/sentry_gazebo_generated/rmul_2027.sdf
```

## 坐标系与外参

URDF 是 **z-up** 的，所以模型位姿里没有额外旋转（`ROBOT_MODEL_RPY = (0,0,0)`），
`odom → base_link` 是纯平移 + yaw。

TF 链完全由标准件产生：

| 变换 | 发布者 | 来源 |
| --- | --- | --- |
| `odom → base_link` | `ros_gz_bridge` | gz 内置 `OdometryPublisher` 的 `/tf_odom`（`Pose_V`） |
| `base_link → yaw_link → mid360_link / l2_link` | `robot_state_publisher` | URDF 正解 + `/joint_states` |
| `mid360_link → sentry/mid360_link/mid360_lidar` 等 | `tf2_ros static_transform_publisher` | 见下文 `frame_id` |

雷达外参（`tf2_echo base_link mid360_link` 等，即 URDF 关节链的正解结果）：

| 传感器 | 父 link | 平移 xyz (m) | 旋转 rpy (deg) |
| --- | --- | --- | --- |
| MID-360 | `yaw_link` | `0.0558, -0.1636, 0.1366` | `105, 0, 0` |
| Unitree L2 | `yaw_link` | `-0.0426, 0.1761, 0.1625` | `-90, 22.5, 0` |

这些数值不在 `arena_spec` 里手抄，`sdf_builder` 用 `Urdf.pose_of()` 从 URDF 关节链
现算，改 CAD 导出即可自动跟上。

> **关于两个雷达的扫描轴**：gz 的 `gpu_lidar` 以 link 的 **+z** 为极轴（射线在
> x–y 平面内绕 z 转，仰角相对该平面）。按当前 URDF 挂载姿态，`mid360_link` 的 +z
> 在 `base_link` 里是 `(0, -0.966, -0.259)`、`l2_link` 的 +z 是 `(0, 1, 0)`，都不是
> 竖直方向。这与观测到的近场自遮挡（`p50 ≈ 0.15 m`，约 57% 的射线有回波）一致，
> 属于模型侧问题，仿真如实复现，没有为了"好看"去改挂载或加偏移。

## 传感器模型

| | MID-360 | Unitree L2 |
| --- | --- | --- |
| 类型 | `gpu_lidar` | `gpu_lidar` |
| 水平 | 360°，720 采样 | 360°，160 采样 |
| 垂直 | 40 线，-7° … +52° | 40 线，-48° … +48° |
| 量程 | 0.1 – 40 m | 0.05 – 30 m |
| 更新率 | 10 Hz | 10 Hz |
| 噪声 | 高斯 σ = 0.01 m | 高斯 σ = 0.01 m |
| 每帧点数 | 28 800 | 6 400 |

L2 的采样刻意按真机 64 000 点/s 的点数预算设置：在 10 倍稠密的仿真上整定的 SLAM
前端换到真机会直接崩掉。需要更密的 MID-360 点云时，改
`arena_spec.MID360_LIDAR` 的第一个参数（例如 720 → 1800）即可。

## ROS 接口

全部映射写在 `config/bridge.yaml`。

| 话题 | 类型 | 方向 | 来源 |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | gz → ROS | gz |
| `/odom` | `nav_msgs/Odometry` | gz → ROS | `OdometryPublisher` |
| `/tf` | `tf2_msgs/TFMessage` | gz → ROS | `OdometryPublisher` 的 `/tf_odom` |
| `/joint_states` | `sensor_msgs/JointState` | gz → ROS | `JointStatePublisher`（`yaw_joint`） |
| `/lidar_mid360/points` | `sensor_msgs/PointCloud2` | gz → ROS | `gpu_lidar` |
| `/lidar_l2/points` | `sensor_msgs/PointCloud2` | gz → ROS | `gpu_lidar` |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → gz | 速度插件 |

gz 侧的原始话题名是 `lidar_mid360/points` 与 `lidar_l2/points`（`<topic>` 元素必须是
`<sensor>` 的直接子元素且**不带前导 `/`**，否则 gz 不派生点云话题）。gz 会派生两个话题：

```
lidar_mid360/points          gz.msgs.LaserScan          （单线，仅可视化用）
lidar_mid360/points/points   gz.msgs.PointCloudPacked   （桥接的就是这个）
```

`/odom` 直接来自 `OdometryPublisher`，协方差矩阵为零（仿真真值，没有加噪声）。

### 关于 `frame_id`

点云的 `header.frame_id` 由 gz 用 `模型名/link名/传感器名` 生成
（例如 `sentry/mid360_link/mid360_lidar`）。`ros_gz_bridge` 没有改写 `frame_id` 的配置项，
当前 `gz-sim8-sensors-system` 也没有实现 `gz_frame_id`，所以 `sim.launch.py` 为每个
传感器起一个 **identity 静态 TF**，把这些 frame 挂到对应的 link 上：

```
mid360_link -> sentry/mid360_link/mid360_lidar
l2_link     -> sentry/l2_link/l2_lidar
```

如果你更希望点云直接带 `mid360_link`，需要在 ROS 侧另加一个改写 `frame_id` 的节点。

## 机器人建模方式

- **物理**：平面运动学底盘。`base_link` + 4 个被动轮 joint + `yaw_joint`（云台），
  由 gz Python system 插件 `plugins/sentry_velocity_control.py` 直接设置模型世界位姿，
  不依赖摩擦/接触求解，因此 `physics` 是 `type="ignored"`。
- **碰撞**：`base_link` 用单个 box hull（`0.594 × 0.594 × 0.170`，中心 `z = 0.027`，
  即 `base_link.stl` 按其 URDF visual 旋转后的包围盒）代替 1.29 M 三角面的 STL，
  外加 4 个圆柱轮（r = 0.065，厚 0.04）。
- **视觉**：直接用 URDF 的 STL，URI 为 `model://sentry_description/meshes/*.stl`，通过
  `GZ_SIM_RESOURCE_PATH` 里的 `sentry_description` 模型 shim 解析（`prepare_world.py` 会把
  `meshes/` 软链到生成目录）。
- **SDF 位姿不叠加**：link 的 `<pose>` 承载安装位姿，对应的 `<joint><pose>` 恒为
  `0 0 0 0 0 0`。两处都写同一 mount 位姿会导致位姿叠加两次（早期曾因此出现 90° roll）。
- **不用 `ros_gz_sim create` 的原因**：`sdformat_urdf` 已安装，但它会原样使用 URDF 的
  `<collision>` 网格（车身 1.29 M 三角面、云台 0.92 M），物理步进会卡死；而且要把
  `<gazebo>` 传感器标签塞进真车共用的 `sentry_description`。所以由 `sdf_builder`
  自己转换，碰撞用基本体，视觉保留网格。

## 赛场几何

`arena_spec.ARENA_BOXES` 是 40 个轴对齐盒，从官方 STEP 细分网格与 `scenario.json`
派生，负责碰撞；视觉上叠加官方 `models/rmul_2027/meshes/step_textured.dae`。

## 自检（全部用标准工具）

```bash
ros2 topic hz /lidar_mid360/points     # 标称 10 Hz
ros2 topic hz /lidar_l2/points         # 标称 10 Hz
ros2 topic hz /odom                    # 标称 50 Hz
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link mid360_link
ros2 topic echo --once /clock
ros2 topic echo --once /joint_states
```

RTF 直接看 Gazebo GUI 右下角的 World stats（`real time factor`），或
`gz topic -e -t /clock` 观察仿真时间推进。

## 已知限制

1. **GUI 里的 `Float message and pointcloud are not of the same size` 警告**：
   gz-gui 的 `PointCloud` 插件只读 `<point_cloud_topic>` 和 `<float_v_topic>`。
   没配 float_v 时，它每帧拿**空的** `Float_V` 和点云比长度
   （`PointCloud.cc` 的 `PublishMarkers`），于是每帧报一次。只影响 GUI 的点着色，
   **ROS 话题上的点云数据完全不受影响**（`/lidar_*/points` 的 width、frame_id、
   坐标都正常）。要彻底消掉只能关掉这两个 GUI 面板。
2. **雷达自遮挡**：按当前 URDF 挂载姿态，两个雷达的 +z 扫描轴都不是竖直的，
   近场有大量车体回波（`p50 ≈ 0.15 m`，57% 射线有回波）。属模型侧问题，仿真如实
   复现，未做补偿。
3. **非有限点**：gz 未命中的 ray 返回 `inf`（约 43%），ROS 侧用
   `point_cloud2.read_points(..., skip_nans=True)` 过滤即可。
4. **`/odom` 协方差为零**：`OdometryPublisher` 的普通 odometry 话题不带协方差，
   Nav2 里若配置了协方差阈值需要放宽，或用真值当 ground truth。
5. `RMUL_2027` 包名触发 colcon 命名规范警告（大写），无害。
6. WSL 下无 GPU 直通时会回退到软件渲染（`MESA-EGL ... kms_swrast`），此时 RTF
   约 0.5，话题频率按 RTF 等比下降（实测 `/odom` 约 27 Hz、`/clock` 约 126 Hz、
   雷达约 2.6–4.6 Hz），属环境限制。

## 目录结构

```
sentry_gazebo/
├── sentry_gazebo/
│   ├── arena_spec.py      # 单一数据源：赛场盒体、机器人位姿、雷达参数、话题名
│   ├── sdf_builder.py     # URDF + arena_spec -> SDF world
│   └── teleop.py          # 键盘遥控节点（发布 /cmd_vel）
├── launch/
│   ├── sim.launch.py      # 主入口（gz + bridge + robot_state_publisher + 帧别名）
│   ├── prepare_world.py   # 每次启动重新生成 world + env.sh
│   └── launch_utils.py    # 路径/环境变量共享
├── plugins/
│   ├── sentry_velocity_control.py   # gz Python system 插件（平面速度）
│   └── sitecustomize.py
├── scripts/teleop         # ros2 run 入口（装到 lib/sentry_gazebo）
├── config/bridge.yaml     # 全部 ROS <-> gz 话题
├── rviz/sentry.rviz
└── models/rmul_2027/      # 官方 STEP 网格 + 贴图（仅视觉）
```
