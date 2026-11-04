# sentry_gazebo — RMUL 2027 哨兵仿真环境

用 Gazebo Sim 8（gz-sim，ROS Jazzy）搭建的 RMUL 2027 赛场仿真。机器人几何与传感器
安装位姿**全部从 `src/sentry_description/urdf/sentry_robot.urdf` 读取**，赛场几何与
雷达参数在 `sentry_gazebo/arena_spec.py`，`sentry_gazebo` 只负责把它们组装成一个
可运行的 world。

位姿、TF、关节状态、点云全部由 gz 内置 system 或标准 ROS 2 包提供；自定义代码只有
URDF→SDF 生成和一个键盘遥控节点（`ros2 run sentry_gazebo teleop`）；底盘由 gz 内置
`MecanumDrive` 通过真实轮地接触驱动，两台雷达各带一个内置 IMU。

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
| `uwb` | `true` | 是否启动裁判系统 UWB 模拟节点 `uwb_sim`（仅 `arena:=rmuc` 生效） |
| `team` | 空 | 己方颜色 `red` / `blue`：出生在己方半场，并由 `uwb_sim` 发布到 `/uwb_pose/team`；空 = 场地原出生点（RMUL 红方、RMUC 蓝方） |
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
| `a` / `d` | 左平移 / 右平移（四轮全向底盘） |
| `q` / `e` | 左转 / 右转 |
| `space` | 停止 |
| `+` / `-` | 加速 / 减速 |
| `x` | 退出 |

参数：`-p linear_speed:=0.5 -p angular_speed:=1.0 -p cmd_vel_topic:=/cmd_vel`。
节点以 20 Hz 重复发布。运动键**按住才动、松开即停**：终端收不到按键抬起事件，所以以键盘
自动重复停止为准——首次按下后 `press_timeout`（默认 0.6 s，覆盖自动重复的起始延迟）、
之后每次重复后 `repeat_timeout`（默认 0.15 s）内没有新的重复，就视为松开并发 0。
若松手后停得慢或按住时断断续续，可用 `-p press_timeout:=... -p repeat_timeout:=...` 按本机
键盘重复速率调整（`xset q` 可查看）。

`MecanumDrive` 没有指令超时，会一直执行最后一条指令：松键、按空格或 `x` 退出时 teleop
会发 0，但强杀 teleop 进程车会继续走。

stdin 不是终端时（管道、launch）改为**每行一个按键、保持 `hold` 秒**，方便脚本化：

```bash
printf 'w\nw\na\n' | ros2 run sentry_gazebo teleop --ros-args -p hold:=2.0
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

URDF 是 **z-up** 的，所以模型位姿里没有额外旋转（`ROBOT_MODEL_RPY = (0,0,0)`）。

机器人在世界中的位置**只由定位给出**，与真车一致：`map → lio_odom → base_link`（`bringup`）。
仿真不发布任何真值位姿（没有 `/odom` 话题，也没有 `odom` 帧），因为真车上没有。
不起定位时 TF 树只有 `base_link` 以下的 URDF 部分。

TF 链完全由标准件产生：

| 变换 | 发布者 | 来源 |
| --- | --- | --- |
| `map → lio_odom` | `backend_node` | 定位后端 |
| `lio_odom → base_link` | `zlio`（`frontend`） | ZLIO 里程计（`lio_odom → imu`）乘以按时间戳从 TF 查到的 `imu ← base_link` |
| `base_link → yaw_link → mid360_link / l2_link` | `robot_state_publisher` | URDF 正解 + `joint_state_publisher` 发布的零位 `/joint_states` |
| `mid360_link → mid360_imu_link` | `robot_state_publisher` | URDF 固定关节（真车 Livox 出厂 IMU 外参）；仿真 IMU 位于 `mid360_link` 原点，前端 `frames.imu` 直接用 `mid360_link` |
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
| 内置 IMU | gz `imu`，雷达原点，200 Hz | 同左 |

IMU 噪声按 MEMS 量级（陀螺 σ 2e-3 rad/s，加计 σ 2e-2 m/s²），输出比力（含重力，m/s²）。
车身视觉带 `visibility_flags`，雷达的 `visibility_mask` 将其屏蔽，不会打到自己车身
（真机驱动同样不出车体回波）。

L2 的采样刻意按真机 64 000 点/s 的点数预算设置：在 10 倍稠密的仿真上整定的 SLAM
前端换到真机会直接崩掉。需要更密的 MID-360 点云时，改
`arena_spec.MID360_LIDAR` 的第一个参数（例如 720 → 1800）即可。

## ROS 接口

映射写在 `config/bridge.yaml`。

| 话题 | 类型 | 方向 | 来源 |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | gz → ROS | gz |
| `/joint_states` | `sensor_msgs/JointState` | ROS | `joint_state_publisher`（全部关节零位，不经 bridge） |
| `/lidar_mid360/points` | `sensor_msgs/PointCloud2` | gz → ROS | `gpu_lidar` |
| `/lidar_l2/points` | `sensor_msgs/PointCloud2` | gz → ROS | `gpu_lidar` |
| `/lidar_mid360/imu` | `sensor_msgs/Imu` | gz → ROS | `imu` 传感器 |
| `/lidar_l2/imu` | `sensor_msgs/Imu` | gz → ROS | `imu` 传感器 |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → gz | `MecanumDrive` |
| `/uwb_pose` | `geometry_msgs/PoseStamped` | ROS | `uwb_sim`（裁判系统 0x0203 模拟，1 Hz，仅 RMUC，见下文） |
| `/uwb_pose/team` | `std_msgs/String` | ROS | `uwb_sim`：己方颜色 `red` / `blue`（transient local，仅 RMUC） |

gz 侧的原始话题名是 `lidar_mid360/points` 与 `lidar_l2/points`（`<topic>` 元素必须是
`<sensor>` 的直接子元素且**不带前导 `/`**，否则 gz 不派生点云话题）。gz 会派生两个话题：

```
lidar_mid360/points          gz.msgs.LaserScan          （单线，仅可视化用）
lidar_mid360/points/points   gz.msgs.PointCloudPacked   （桥接的就是这个）
```

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

## 裁判系统 UWB 模拟（0x0203）

`uwb_sim` 节点模拟裁判系统串口 0x0203“机器人位置数据”，**只在 RMUC 场地（`arena:=rmuc`）启动**，RMUL 不模拟；`uwb:=false` 可关闭，
也可单独运行：`ros2 run sentry_gazebo uwb_sim --ros-args -p use_sim_time:=true`。

| 项 | 值 |
| --- | --- |
| 话题 | `/uwb_pose`，`geometry_msgs/PoseStamped`，`frame_id: uwb`（即后端 `uwb_pose` / `uwb.frame` 的输入） |
| 频率 | 1 Hz（仿真时间），与 0x0203 一致 |
| 内容 | x、y 为裁判系统场地坐标（m，float32 精度）；z 恒为 0；朝向 `angle` 不模拟，姿态为单位四元数 |
| 时间戳 | **接收时刻**（与串口驱动一致），数据本身比时间戳旧约 0.2 s |

**真值来源**：机器人模型上的 gz `PosePublisher` 只发布到 gz 话题 `/model/sentry/pose`，**不进 bridge**，
ROS 侧仍然没有真值。

**坐标系**（协议表 1-21）：原点为红方补给站一侧的场地围挡交点，+X 沿长边指向蓝方，+Y 沿短边指向红方停机坪；红蓝方共用。
RMUC world 为 28 × 16 m、以世界原点为中心，红方在世界 +x 一侧，所以裁判坐标系是世界坐标系绕 z 转 180°（右手系）：

```
x_ref = 14 - x_world        y_ref = 8 - y_world        （蓝方出生点 (-9.5, 0) -> (23.5, 8)，红方出生点 (9.5, 0) -> (4.5, 8)）
```

原点、轴向可用参数 `referee_origin_x/y`、`referee_x_axis_yaw_deg`、`referee_y_axis_sign` 覆盖。

**己方颜色**：launch 参数 `team:=red|blue`。场地中心对称，红方出生点 = 蓝方出生点绕场地中心转 180°（RMUC：蓝方 `(-9.5, 0, 0)`，红方 `(9.5, 0, π)`，都朝向场地中央）。`uwb_sim` 把颜色发布到 `/uwb_pose/team`（`std_msgs/String`，`red`/`blue`，transient local，每秒随 UWB 重发）。`/uwb_pose` 本身仍是 `PoseStamped`（后端直接订阅它，不能加字段）。单独运行 `uwb_sim` 时用 `-p team:=red` 指定；首帧位置不在该颜色半场时会打印警告。

**误差模型**（默认值在 `arena_spec.UWB_SIM`，均可用同名参数覆盖）：

| 参数 | 默认 | 依据 |
| --- | --- | --- |
| `noise_std` | 0.08 m | 每轴白噪声；UWB 套装标称 ±10 cm |
| `bias_std` / `bias_tau` | 0.05 m / 30 s | 一阶 Gauss-Markov 慢变偏差（基站坐标误差、多径） |
| `outlier_prob` / `outlier_std` | 3 % / 0.30 m | 遮挡时 ±30 cm |
| `latency_mean` / `latency_jitter_std` | 0.20 s / 0.03 s | UWB 输出延时约 80 ms + 常规链路约 130 ms |
| `drop_prob` | 1 % | 常规链路丢包 <1 % |
| `antenna_position_body` | `[-0.245, 0, 0]` | 标签在 `base_link` 中的位置，与后端 `uwb.antenna_position_body` 一致；按真实姿态旋转（杆臂） |
| `seed` | 0 | 0 为随机，非 0 可复现 |

后端侧对应参数：`uwb.position_std_xy` 0.15（≈ 白噪声 + 偏差 + 离群的综合）、`uwb.time_offset_seconds` 约 `-0.2`、
`uwb.effective_rate_hz` 1。bringup 的 `config/sim/backend*.yaml` 里 `uwb.enabled` 为 `false`（RMUL 没有 UWB），
RMUC 上用 `ros2 launch bringup sim.launch.py uwb:=true` 打开（只覆盖 `uwb.enabled`）。仿真回归见 `tools/sim_tests/README.md`（`ARENA=rmuc UWB=true`）。

## 机器人建模方式

- **底盘**：X 型四全向轮（轮轴沿对角线，四轮接地点共面）。gz 内置 `MecanumDrive`
  把机体系 `/cmd_vel` 换算成轮速，靠轮地接触驱动，所以 IMU 读到的是真实加速度。
  全向轮与麦轮运动学等价、轮缘速度差 √2，故 `wheel_radius = √2 × 0.065`；轮子按几何位置
  （而非 URDF 名字）分到 front_left 等槽位，见 `sdf_builder.omni_wheel()`。
  之前的“直接写速度/位姿”插件会让 IMU 运动中读到 100+ m/s² 的假加速度，已删除。
- **云台**：仿真不驱动云台，`yaw_joint` 在 SDF 里是 fixed（被动关节会被底盘带着空转）。
- **碰撞**：`base_link` 用单个 box hull 代替 1.29 M 三角面的 STL。水平尺寸和顶面取
  `base_link.stl` 按其 URDF visual 旋转后的包围盒（`0.594 × 0.594`，顶面离地 0.174 m）；
  底面抬到离地 `CHASSIS_GROUND_CLEARANCE` = 30 mm（原包围盒底面只离地 4 mm，会卡在中央 10 mm 高的台面边上）。
  外加 4 个圆柱轮（r = 0.065，厚 0.04），轮子只在滚动方向有摩擦（`fdir1`，`mu2 = 0`），
  模拟全向轮的小辊子。贴墙时方形车体转不动，需先退开，与真车一致。
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
ros2 run tf2_ros tf2_echo map base_link        # 需同时运行 bringup 定位
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
2. **雷达近场点**：车身已对雷达隐藏，剩下的近场点主要是地面（MID-360 离地约 0.2 m、
   侧倾 105°）。定位侧 `lidar_fusion` 丢弃 0.3 m 内的点。
3. **非有限点**：gz 未命中的 ray 返回 `inf`（约 43%），ROS 侧用
   `point_cloud2.read_points(..., skip_nans=True)` 过滤即可。
4. WSL 下无 GPU 直通时会回退到软件渲染（`MESA-EGL ... kms_swrast`），此时 RTF
   约 0.5，话题频率按 RTF 等比下降（实测 `/clock` 约 126 Hz、
   雷达约 2.6–4.6 Hz），属环境限制。

## 目录结构

```
sentry_gazebo/
├── sentry_gazebo/
│   ├── arena_spec.py      # 单一数据源：赛场盒体、机器人位姿、雷达参数、话题名
│   ├── sdf_builder.py     # URDF + arena_spec -> SDF world
│   ├── teleop.py          # 键盘遥控节点（发布 /cmd_vel）
│   └── uwb_sim.py         # 裁判系统 UWB（0x0203）模拟节点（发布 /uwb_pose）
├── launch/
│   ├── sim.launch.py      # 主入口（gz + bridge + robot/joint_state_publisher + 帧别名）
│   ├── prepare_world.py   # 每次启动重新生成 world + env.sh
│   └── launch_utils.py    # 路径/环境变量共享
├── scripts/teleop, uwb_sim  # ros2 run 入口（装到 lib/sentry_gazebo）
├── config/bridge.yaml     # ROS <-> gz 话题
├── rviz/sentry.rviz
└── models/rmul_2027/      # 官方 STEP 网格 + 贴图（仅视觉）
```
