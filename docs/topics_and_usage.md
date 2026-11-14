# yang_nav 话题、模块与使用说明

本文对应 `ros2 topic list` 在**仿真 + 建图模式 + RViz** 下的输出（`ros2 launch sentry_gazebo sim.launch.py` 与 `ros2 launch bringup sim.launch.py rviz:=true`）。
各节点的完整接口表见 `src/z-localization/frontend/README.md` 与 `src/z-localization/backend/README.md`，参数见两包的 `docs/parameters.md`。

---

## 1. 系统总览

```
            Gazebo 仿真 (sentry_gazebo)              真车 (livox_ros_driver2 / unilidar_sdk2)
   /lidar_mid360/points  /lidar_l2/points  /lidar_mid360/imu      /livox/lidar  /unilidar/cloud  /livox/imu
            │                 │                  │                        │            │              │
            └──────┬──────────┘                  │                        └─────┬──────┘              │
                   ▼                             │                              ▼                     │
          lidar_fusion/merge_node  ──── /lidar/points ────┐            (同一个 merge_node)              │
                                                          ▼                                           │
                                         frontend/zlio_node (ZLIO 激光惯性里程计) ◀──────────── IMU ───┘
                     ┌──────────────────────┬─────────────┼───────────────┬───────────────────────┐
                 /odometry            /lidar_odometry  /frontend/frame /frontend/odometry  /frontend/session
                     │                                    └───────────────┴───────────────────────┤
                     ▼                                                                            ▼
          lidar_calibration                                                          backend/backend_node
        （标定，订阅 /odometry）                                          (全局定位 / 建图 / 回环 / 重定位)
                                                                                   │
            TF: map → lio_odom → base_link、/localization/odometry、~/localization_pose、~/localization_state、可视化话题 ...
```

TF 树（仿真与真车一致）：

```
map ──(backend_node)──▶ lio_odom ─(backend_node)─▶ base_link ──(robot_state_publisher, URDF)──▶ yaw_link ──▶ mid360_link ──▶ mid360_imu_link
                                                                                                      └─▶ l2_link
```

- 不再有手写的 `imu` 静态 TF：MID-360 内置 IMU 是 URDF 中的 `mid360_imu_link`（Livox 出厂外参）。
  前端 IMU 坐标系 `frames.imu` 仿真为 `mid360_link`（gz IMU 与雷达同原点），真车为 `mid360_imu_link`；下文“`imu` 系”均指它。
- 组合启动关闭前端 `output.publish_tf`，开启后端 `common.publish_odom_tf`。后端在一个 TFMessage 内、用同一代同一时间戳的位姿投影发布两条动态边；高频预测也走此路径。
  后端按输出时间戳换算车体外参，因此跟随云台运动。独立前端仍可开启 TF，但此时后端须关闭 `common.publish_odom_tf`。IMU 系在 URDF 里已有父帧，里程计接到树根 `base_link`。
- 前端启动时从 TF 查一次 `frames.imu ← frames.lidar`，后端按消息时间戳查询 `common.sensor_frame ← base_link`，跟踪云台 `yaw_joint`（容差 `common.body_extrinsic_tolerance`）。
- 后端不发布 `map → body`，树里只有 `base_link` 一个车体帧。

---

## 2. 模块（功能包）作用

| 包 | 可执行 / 节点 | 作用 |
| --- | --- | --- |
| `sentry_description` | — | 哨兵机器人 URDF 与网格（`urdf/sentry_robot.urdf`），提供传感器外参 |
| `sentry_gazebo` | `sim.launch.py`、`gz_bridge`、`robot_state_publisher`、`joint_state_publisher` | Gazebo Harmonic 仿真（RMUL 场地），桥接雷达/IMU/时钟/`cmd_vel`；**不提供真值位姿** |
| `livox_ros_driver2` | `livox_ros_driver2_node` | 真车 MID-360 驱动，输出 `/livox/lidar`、`/livox/imu` |
| `lidar_fusion` | `merge_node`（节点名 `lidar_merge`） | 把 MID-360 与可选 L2 点云按 tf2 外参合并到 `mid360_link`，输出带逐点时间 `offset_time` 的 `/lidar/points` |
| `frontend` (ZLIO) | `zlio_node`、`zlio_visualizer` | 激光惯性里程计：IMU 预测 + 点面配准滤波，输出高频里程计和去畸变观测帧；组合启动关闭其 TF 发布 |
| `backend` | `backend_node`、`body_calibration_node` | 全局定位后端：先验地图定位或在线建图、Scan Context 回环、位姿图优化、丢失恢复（局中恢复用裁判系统 UWB 作位置先验） |
| `localization_interfaces` | — | 前后端协议消息：`FrontendFrame`、`FrontendOdometry`、`FrontendSession`、`ResetFrontend.srv` |
| `lidar_calibration` | `calibrate` | 雷达/车体外参标定工具（订阅 `/odometry`） |
| `bringup` | `localization.launch.py`、`sim.launch.py`、`real.launch.py` | `localization.launch.py` 组装融合 + ZLIO + 后端；`sim`/`real` 只管数据源；`config/{sim,real}/{lidar_fusion,frontend,backend}.yaml` 为完整参数文件 |

---

## 3. 话题逐条说明

### 3.1 传感器与仿真输入

| 话题 | 类型 | 发布者 | 说明 |
| --- | --- | --- | --- |
| `/lidar_mid360/points` | `sensor_msgs/PointCloud2` | `gz_bridge` | 仿真 MID-360 点云，frame 为 gz 作用域名，由静态 TF 别名挂到 `mid360_link` |
| `/lidar_mid360/imu` | `sensor_msgs/Imu` | `gz_bridge` | 仿真 MID-360 内置 IMU，单位 m/s²（真车 Livox 为 g，`bringup/config/real/frontend.yaml` 中 `input.imu_acceleration_scale: 9.81`） |
| `/lidar_l2/points` | `sensor_msgs/PointCloud2` | `gz_bridge` | 仿真宇树 L2 点云，`lidar:=dual` 时参与融合 |
| `/lidar_l2/imu` | `sensor_msgs/Imu` | `gz_bridge` | L2 内置 IMU，当前流程未使用 |
| `/lidar/points` | `sensor_msgs/PointCloud2` | `lidar_merge` | 合并点云，`mid360_link` 系，字段 `x y z offset_time(uint32 ns)`，ZLIO 的雷达输入 |
| `/cmd_vel` | `geometry_msgs/Twist` | 导航 / 手动遥控（订阅者：Gazebo MecanumDrive） | 底盘速度指令 |
| `/clock` | `rosgraph_msgs/Clock` | `gz_bridge` | 仿真时间，所有节点 `use_sim_time:=true` |

### 3.2 机器人描述与 TF

| 话题 | 发布者 | 说明 |
| --- | --- | --- |
| `/robot_description` | `robot_state_publisher` | URDF 字符串（transient local），RViz RobotModel 使用 |
| `/joint_states` | `joint_state_publisher` | 关节角；仿真中云台固定，发零位。真车从 `/chassis/joint_states`（底盘通信节点）取云台角，没有时发 0 |
| `/tf` | `robot_state_publisher`、`backend_node` | 动态 TF：URDF 可动关节；后端成对发布 `map → lio_odom` 与 `lio_odom → base_link` |
| `/tf_static` | `robot_state_publisher`、各 `static_transform_publisher` | 固定外参（含 `mid360_link → mid360_imu_link`）与驱动/gz 传感器帧别名 |

### 3.3 前端 ZLIO（`zlio_node`）

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/odometry` | `nav_msgs/Odometry` | **随 IMU 更新的高频预测**位姿/速度/协方差，pose 在 `lio_odom`，twist 在 `imu` 系；标定工具订阅 |
| `/lidar_odometry` | `nav_msgs/Odometry` | 每次激光观测更新后的后验估计（频率≈雷达帧率） |
| `/frontend/frame` | `localization_interfaces/FrontendFrame` | 一次已提交观测：位姿 + 去畸变到 `imu` 系的点云 + 匹配数，含 `instance_id/epoch/sequence`；后端主输入 |
| `/frontend/odometry` | `localization_interfaces/FrontendOdometry` | 带进程身份和代次的高频预测，后端据此高频外推输出 |
| `/frontend/session` | `localization_interfaces/FrontendSession` | 进程身份、重置代次、`reset_required`（输入中断）与 `failed`（处理失败），transient local |

> 仅在 `frontend.launch.py` 启动 `zlio_visualizer` 时才会出现 `/frontend/cloud_registered`、`/frontend/cloud_map`、`/frontend/path`，本次列表中没有。

### 3.4 后端（`backend_node`）主输出

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/localization/odometry` | `nav_msgs/Odometry` | map 系的**车体**位姿 + 车体系速度 + 协方差（高频）；`child_frame_id` 为 `base_link` |
| `/backend_node/localization_pose` | `geometry_msgs/PoseWithCovarianceStamped` | map 系的**车体**位姿及协方差 |
| `/backend_node/localization_state` | `std_msgs/String` | 分号分隔 `key=value`：状态、`pose_trusted`、代次、输出版本、故障信息 |
| `/localization/lio_world_cloud` | `sensor_msgs/PointCloud2` | 变换到 map 系的实时点云（供导航/障碍物使用），由 `output.world_cloud` 控制 |

定位状态：

| 状态 | 含义 | 可信 |
| --- | --- | --- |
| `INITIALIZING` | 等待建立初始全局位姿 | 否 |
| `NORMAL` | 正常定位 | 是 |
| `OBS_DEGRADED` | 观测与预测短暂冲突，未达阈值 | 是 |
| `SUSPECT` | 连续冲突/速度异常，尝试恢复；输出由里程计前推（沿用最后可信 `map→lio_odom`），保留速度 | 否 |
| `LOST` | 定位丢失，等待重定位；输出冻结在最后可信位姿，速度清零 | 否 |

后端另订阅裁判系统 UWB：`/uwb_pose`（`geometry_msgs/PoseStamped`，只用 x、y）与 `/uwb_pose/team`（`std_msgs/String`，己方颜色，transient local），只在 `uwb.enabled` 时订阅，只用于 `SUSPECT`/`LOST` 恢复；没有新鲜 UWB 时不会自动恢复。仿真由 `sentry_gazebo` 的 `uwb_sim` 发布（仅 RMUC）。

### 3.5 后端可视化（`rviz:=true` 时启用）

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/backend_node/runtime_map` | `PointCloud2` | 建图模式下累计的运行地图（定位模式改为 `/backend_node/prior_map`） |
| `/backend_node/odom_path` | `nav_msgs/Path` | 前端原始里程计轨迹（已转到 map 系） |
| `/backend_node/optimized_path` | `nav_msgs/Path` | 位姿图优化后的轨迹；与 `odom_path` 对比可看出回环修正量 |
| `/backend_node/loop_edges` | `visualization_msgs/Marker` | 回环连线 |
| `/backend_node/map_axes` | `Marker` | 关键帧坐标轴 |
| `/backend_node/localization_status` | `Marker` | 状态文字标记 |
| `/backend_node/debug/aligned_source` | `PointCloud2` | 最近一次配准后的源点云 |
| `/backend_node/debug/target` | `PointCloud2` | 最近一次配准的目标点云；两者重合程度反映匹配质量 |

> **`*_array` 话题**（`loop_edges_array`、`map_axes_array`、`localization_status_array`）不是代码发布的：RViz2 的 Marker 显示项会同时订阅 `<topic>` 和 `<topic>_array`（MarkerArray），所以只要 RViz 打开就会出现，没有发布者，可忽略。

### 3.6 ROS 通用

| 话题 | 说明 |
| --- | --- |
| `/clicked_point` | RViz “Publish Point” 工具发布，当前无节点订阅 |
| `/parameter_events` | 参数变更事件 |
| `/rosout` | 所有节点日志 |

---

## 4. 常用话题（对接导航 / 上层模块）

### 4.1 定位输出给导航

| 用途 | 话题 / TF | 类型 | 坐标系 | 说明 |
| --- | --- | --- | --- | --- |
| **车体全局位姿（首选）** | TF `map → base_link` | tf2 | `map` | 由后端同一消息内的 `map → lio_odom` + `lio_odom → base_link` 组成；Nav2 的 `global_frame: map`、`robot_base_frame: base_link` 直接可用 |
| 车体全局位姿（话题） | `/backend_node/localization_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | 带协方差，适合不用 TF 的上层模块（决策、裁判系统通信） |
| 车体全局里程计 | `/localization/odometry` | `nav_msgs/Odometry` | pose 在 `map`，twist 在 `base_link` | 高频（随 IMU）；Nav2 的 `odom_topic` 直接用它 |
| 前端原始里程计 | `/odometry` | `nav_msgs/Odometry` | pose 在 `lio_odom`，twist 在 `imu` | 高频局部预测，重置可改变原点；标定工具使用，导航使用后端输出 |
| 定位是否可信 | `/backend_node/localization_state` | `std_msgs/String` | — | 解析 `pose_trusted=true/false`；为 false（`INITIALIZING`/`SUSPECT`/`LOST`）时导航应停车或降级 |
| 障碍物点云 | `/localization/lio_world_cloud` | `sensor_msgs/PointCloud2` | `map` | 当前帧去畸变点云，做 costmap 障碍层 / 地形分析输入 |
| 障碍物点云（传感器系） | `/lidar/points` | `sensor_msgs/PointCloud2` | `mid360_link` | 未去畸变的原始合并点云，可配合 TF 给 costmap 使用 |
| 静态地图点云 | `/backend_node/prior_map` | `sensor_msgs/PointCloud2` | `map` | 定位模式 + `rviz:=true` 时发布，transient local；可用于生成 2D 栅格地图 |

### 4.2 导航输入给底盘

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 车体（`base_link`）系速度指令；仿真由 Gazebo MecanumDrive 执行，真车由底盘通信节点转发给下位机 |
| `/joint_states` | `sensor_msgs/JointState` | 真车由底盘通信节点发布云台 yaw 角，否则 `base_link → mid360_link` 外参错误 |

### 4.3 Nav2 / 上层对接要点

- 全局帧用 `map`，车体帧用 `base_link`，TF 的里程计帧保持 `lio_odom`；`/localization/odometry` 作为 Nav2 的 `odom_topic`（pose 在 `map`、twist 在 `base_link`）。**不要**再让别的节点发布 `odom → base_link` 或 `map → odom`。
- 需要 map 系车体速度时：`/localization/odometry` 的 twist 在 `base_link`，乘 `T_map_base` 的旋转即可；或对 `map → base_link` 差分。
- 订阅 `localization_state` 判断可信性（`pose_trusted=true` 才能使用位姿）。
- 订阅 transient local 话题（`localization_state`、`prior_map`、可视化话题）时 QoS 也需设为 transient local 才能收到最近一次消息。

### 4.4 常用查看命令

```bash
ros2 topic echo /backend_node/localization_state            # 状态与可信标记
ros2 topic echo /backend_node/localization_pose --once      # 车体全局位姿
ros2 run tf2_ros tf2_echo map base_link                     # TF 位姿
ros2 topic hz /odometry                                     # 局部里程计频率（≈IMU 频率）
ros2 topic hz /localization/lio_world_cloud                 # 障碍物点云频率（≈雷达帧率）
```

---

## 5. 使用方法

### 5.1 编译

```bash
cd ~/yang_nav
colcon build --symlink-install
source install/setup.bash   # 每个新终端都要执行
```

### 5.2 仿真

```bash
# 终端 1：仿真（gui:=true 打开 Gazebo 界面）
ros2 launch sentry_gazebo sim.launch.py gui:=true

# 终端 2：定位栈 —— 建图模式（默认），双雷达，打开后端 RViz
ros2 launch bringup sim.launch.py lidar:=dual rviz:=true

# 仅用 MID-360
ros2 launch bringup sim.launch.py lidar:=single
```

`bringup sim.launch.py` 参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `lidar` | `dual` | `single` 仅 MID-360；`dual` 加 L2 |
| `mode` | `mapping` | `mapping` 在线建图；`localization` 使用先验地图 |
| `prior_map` | 空 | 先验地图目录；`localization` 时留空则使用 `map_dir` |
| `map_dir` | `<工作空间>/maps/sim`（真车 `maps/real`） | 地图保存目录，`save_map` 写到这里 |
| `rviz` | `false` | 打开后端 RViz 并启用可视化话题 |

让车动起来（另开终端）：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 发布 /cmd_vel
```

### 5.3 建图 → 保存 → 定位

```bash
# 1. 建图模式下开车绕场地一圈（尽量回到起点触发回环）
ros2 launch bringup sim.launch.py mode:=mapping rviz:=true

# 2. 保存地图包到 map_dir（默认 ~/yang_nav/maps/sim，目录不存在会自动创建）
ros2 service call /backend_node/save_map std_srvs/srv/Trigger

# 3. 用保存的地图定位（默认从 map_dir 加载）
ros2 launch bringup sim.launch.py mode:=localization rviz:=true

# 保存/加载其他目录
ros2 launch bringup sim.launch.py mode:=mapping map_dir:=$HOME/yang_nav/maps/rmul_v2
ros2 launch bringup sim.launch.py mode:=localization prior_map:=$HOME/yang_nav/maps/rmul_v2
```

> 地图默认放在工作空间 `maps/` 下（`<工作空间>` 由 `install/bringup` 的位置推出，即 `~/yang_nav`）。
> 同一目录再次保存会覆盖；想保留旧图请换 `map_dir`。定位模式下后端不会保存地图。

### 5.4 真车

```bash
# 仅 MID-360，建图（地图保存到 ~/yang_nav/maps/real）
ros2 launch bringup real.launch.py lidar:=single

# MID-360 + L2（需安装 unilidar_sdk2）
ros2 launch bringup real.launch.py lidar:=dual driver_l2:=true l2_launch:=/abs/path/unilidar.launch.py

# 先验地图定位（默认加载 maps/real）
ros2 launch bringup real.launch.py mode:=localization
```

真车话题名与仿真不同：MID-360 为 `/livox/lidar`、`/livox/imu`，L2 为 `/unilidar/cloud`。
标注“待真车确认”的参数上车后先核对：launch 参数 `mid360_topic`、`imu_topic`、`l2_topic`、`l2_frame`，以及 `bringup/config/real/lidar_fusion.yaml` 的逐点时间键 `primary.*`、`secondary.*`：

```bash
ros2 topic echo --once /livox/lidar --field fields
ros2 topic echo --once /livox/lidar --field header.frame_id
```

不一致时改 launch 参数或该 yaml 即可，无需改代码。驱动参数在 `bringup/config/real/livox_driver.yaml`，IP 在 `bringup/config/real/MID360_config.json`。
云台角由底盘发布到 `/chassis/joint_states`，`joint_state_publisher` 合并后输出 `/joint_states`，没收到的关节发 0。

### 5.5 服务

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `/backend_node/save_map` | `std_srvs/Trigger` | 保存运行地图包到 `map_dir`（建图模式） |
| `/backend_node/relocalize` | `std_srvs/Trigger` | `LOST` 时请求重定位（需要新鲜 UWB，在 UWB 位置附近搜索） |
| `/localization_reset` | `std_srvs/Trigger` | 前后端协调重置；**调用期间必须停车** |
| `/zlio/reset_internal` | `localization_interfaces/ResetFrontend` | 后端内部调用，不要手动调用 |
| `/localization_start` | `std_srvs/SetBool` | 前端 `wait_for_start:=true` 时，`data: true` 开始接收数据 |
| `/stop` | `std_srvs/Trigger` | 前端停止接收输入 |

```bash
ros2 service call /backend_node/relocalize std_srvs/srv/Trigger
ros2 service call /localization_reset std_srvs/srv/Trigger
```

### 5.6 车体外参标定

```bash
ros2 launch backend body_calibration.launch.py odom_frame:=lio_odom use_sim_time:=true
```

按提示原地旋转（偏航跨度 ≥180°），终端会输出标定结果。外参只在 URDF（`sentry_description/urdf/sentry_robot.urdf`）中维护，结果应写入 URDF 对应关节，而不是 yaml。

---

## 6. 常用调试命令

```bash
# 定位状态（最先看这个）
ros2 topic echo /backend_node/localization_state

# 各环节频率：雷达 → 融合 → 前端 → 后端
ros2 topic hz /lidar_mid360/points
ros2 topic hz /lidar/points
ros2 topic hz /lidar_odometry
ros2 topic hz /localization/odometry

# 查看 TF 链是否完整
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames          # 生成 frames.pdf

# 查看话题的发布者 / 订阅者与 QoS
ros2 topic info -v /frontend/frame

# 节点关系图
rqt_graph
```

### 排查顺序

| 现象 | 检查 |
| --- | --- |
| `/lidar/points` 无数据 | 原始雷达话题是否有数据；`lidar_merge` 日志中 tf2 查找是否失败（`robot_state_publisher` 是否在运行） |
| `/odometry` 无数据 | IMU 话题与 `imu_acceleration_scale` 是否正确；ZLIO 日志中初始化是否通过（仿真出生点几何单一时已调低 `minimum_directional_support`） |
| 状态一直 `INITIALIZING` | 定位模式：`prior_map` 路径和出生点是否在地图范围内；可调 `/backend_node/relocalize` |
| `SUSPECT`/`LOST` 频繁 | 对比 `debug/aligned_source` 与 `debug/target` 的重合度；检查 URDF 中传感器到 `base_link` 的外参与 `scan_context.lidar_height` |
| RViz 中车模不动/错位 | `ros2 run tf2_ros tf2_echo map base_link`；确认没有多个节点发布 IMU 坐标系或 `base_link` 的父帧 |
| 仿真时间相关警告 | 所有节点是否都 `use_sim_time:=true`，`/clock` 是否在发布 |
