# ZLIO 前端使用与机制说明

本文说明 `src/z-localization/frontend`（ROS 包 `frontend`，节点 `/zlio`）当前在 yang_nav 中的用法：它在系统里的位置、启动方式、初始化与运行流程、输入中断和重置机制、主要参数，以及仿真中已知的问题。

- 后端说明见 [backend_guide.md](backend_guide.md)，话题总表见 [topics_and_usage.md](topics_and_usage.md)。
- 全部参数见 `src/z-localization/frontend/docs/parameters.md`，接口细节见 `frontend/README.md`。

---

## 1. 前端做什么

ZLIO 是激光惯性里程计（LIO）：用 IMU 做高频运动预测，用 LiDAR 点云和在线平面地图做迭代滤波校正，输出**局部、连续、会缓慢漂移**的里程计。它不知道全局地图，全局位姿由后端负责。

```
/lidar_mid360/points ─┐
/lidar_l2/points ─────┴─▶ lidar_merge ──/lidar/points──┐
/lidar_mid360/imu（真车 /livox/imu）────────────────────┴─▶ /zlio ─┬─ /odometry          ─▶ 标定工具
                                                                   ├─ /tf                ─▶ lio_odom → base_link（frames.base）
                                                                   ├─ /lidar_odometry
                                                                   ├─ /frontend/frame    ─▶ backend_node（观测）
                                                                   ├─ /frontend/odometry ─▶ backend_node（高频预测）
                                                                   └─ /frontend/session  ─▶ backend_node（身份/代次/中断/失败）
```

| 输出 | 频率 | 内容 |
| --- | --- | --- |
| `/odometry` | ≈ IMU 频率 | 随每个 IMU 样本更新的**预测**位姿、速度、协方差。pose 在 `lio_odom`，twist 在 `imu` 系。连续不跳变，适合控制和局部规划 |
| `/lidar_odometry` | ≈ 激光更新频率 | 每次激光观测更新后的**后验**估计 |
| `/frontend/frame` | 同上 | 一次已提交观测：位姿 + 去畸变到该时刻 `imu` 系的点云 + 匹配数 + `lidar_updated`，带 `instance_id/epoch/sequence` |
| `/frontend/odometry` | ≈ IMU 频率 | `/odometry` 加上进程身份和代次，供后端区分新旧数据 |
| `/frontend/session` | 事件触发 | transient local；启动、重置、输入中断（`reset_required`）、处理失败（`failed`）时发布 |

在 bringup 中 ZLIO 直接发布 `lio_odom → base_link` TF（`output.publish_tf: true`、`frames.base: base_link`）：IMU 坐标系（`frames.imu`，仿真 `mid360_link`，真车 `mid360_imu_link`）在 URDF 树上已有父帧，直接发 `lio_odom → frames.imu` 会让它有两个父帧，所以 ZLIO 把里程计乘以按里程计时间戳从 TF 查到的 `frames.imu ← base_link`，接到树根 `base_link`，云台 `yaw_joint` 转动也会跟随。下文的“`imu` 系”均指 `frames.imu`。

---

## 2. 启动

### 2.1 随 bringup 启动（推荐）

`bringup` 总入口会同时拉起雷达融合、ZLIO 和后端（`sim.launch.py` / `real.launch.py` 只管数据源，共用管线在 `localization.launch.py`，参数目录为 `bringup/config/sim|real`）：

```bash
ros2 launch sentry_gazebo sim.launch.py            # 仿真
ros2 launch bringup sim.launch.py lidar:=dual

ros2 launch bringup real.launch.py lidar:=single   # 真车
```

bringup 的 `localization.launch.py` include `frontend/launch/frontend.launch.py`，参数文件为 `bringup/config/<platform>/frontend.yaml`（节点参数唯一来源），话题由 launch 重映射：

| 参数 / 键 | 仿真 | 真车 | 说明 |
| --- | --- | --- | --- |
| launch `params_file` | `bringup/config/sim/frontend.yaml` | `bringup/config/real/frontend.yaml` | 完整参数文件 |
| `input.type` | `pointcloud2` | `pointcloud2` | 两路雷达先由 `lidar_merge` 合并成 PointCloud2 |
| launch `lidar_topic` | `/lidar/points` | `/lidar/points` | 合并点云，`mid360_link` 系，逐点时间字段 `offset_time`（uint32 ns，相对帧头） |
| launch `imu_topic` | `/lidar_mid360/imu` | `/livox/imu` | MID-360 内置 IMU |
| `input.imu_acceleration_scale` | `1.0`（gz 输出 m/s²） | `9.81`（Livox 输出 g） | 填错会导致初始化失败或漂移 |
| `frames.world` / `frames.imu` / `frames.lidar` | `lio_odom` / `mid360_link` / `mid360_link` | `lio_odom` / `mid360_imu_link` / `mid360_link` | `world`、`imu` 与后端 `common.odom_frame` / `common.sensor_frame` 一致 |
| `session.coordinated_reset` | `true` | `true` | 输入中断由后端协调重置（第 4 节） |
| `frames.base` | `base_link` | `base_link` | TF 输出子坐标系（URDF 树根），见第 1 节 |
| `output.publish_tf` | `true` | `true` | 发布 `lio_odom → base_link`，见第 1 节 |
| `output.base_extrinsic_tolerance` | `0.2` | `0.2` | 按时间戳查不到 `frames.imu ← base_link` 时，最新变换允许的最大时差（s）；超过则沿用上次有效值并告警 |

`input.point_time_*` 为 `offset_time`、`1e-9`、相对时间，正好对应 `lidar_merge` 的输出格式。各雷达原始的时间字段约定在 `lidar_merge` 里处理（`bringup/config/<platform>/lidar_fusion.yaml` 的 `primary.*`、`secondary.*`）。

### 2.2 单独启动前端

frontend 包只有一个 launch：`frontend.launch.py`，参数为 `params_file`、`use_sim_time`、`lidar_topic`、`imu_topic`、`visualization`：

```bash
# 只启动里程计（默认 params_file 为包内 config/example.yaml，订阅 Livox CustomMsg：/livox/lidar、/livox/imu）
ros2 launch frontend frontend.launch.py params_file:=/abs/frontend.yaml

# 用本机器人的仿真参数单独跑前端（融合另起：ros2 launch lidar_fusion merge.launch.py ...）
ros2 launch frontend frontend.launch.py params_file:=$(ros2 pkg prefix bringup)/share/bringup/config/sim/frontend.yaml \
  use_sim_time:=true lidar_topic:=/lidar/points imu_topic:=/lidar_mid360/imu

# 里程计 + 可视化节点 + 前端 RViz
ros2 launch frontend frontend.launch.py params_file:=/abs/frontend.yaml visualization:=true
```

前端启动时从 TF 查一次 `frames.imu ← frames.lidar`，该变换须已由 URDF（`robot_state_publisher`）或手动 `static_transform_publisher` 发布。`config/example.yaml` 为 `frames.world=odom`、`frames.base=imu`（直接发布 `odom → imu`），与 bringup 不同；不要和 bringup 同时运行。可视化节点额外发布 `/frontend/cloud_registered`、`/frontend/cloud_map`、`/frontend/path`，只用于显示。

### 2.3 启动前检查

```bash
pgrep -af 'merge_node|zlio_node|backend_node'           # 启动前应为空
ros2 topic info /lidar/points                            # 启动后 Publisher count 必须为 1
```

曾出现上一次 launch 残留的 `merge_node`，导致 `/lidar/points` 两个发布者、ZLIO 收到重复帧并报 `IMU input interrupted`，见 7.1。

---

## 3. 运行流程

### 3.1 等待启动（可选）

`wait_for_start: true` 时，节点丢弃所有输入，直到收到 `/localization_start`（`SetBool`，`data: true`）。bringup 未开启，启动即开始接收。

### 3.2 初始化

初始化需要车辆**静止或缓慢运动、视野中有足够多的平面**：

1. 收集 `initialization.seed_imu_samples`（31）个 IMU 样本，粗估重力方向；
2. 积累 `window_frames`（20）帧扫描，逐帧粗配准建立临时地图（`initialization.registration`）；
3. 对窗口做“重去畸变 + 激光/IMU 联合优化”交替迭代，最多 `maximum_refinements`（15）轮（`initialization.optimization`），估计速度、重力、陀螺和加速度零偏；
4. 检查几何约束：有效平面数 ≥ `minimum_planes`（10），最弱方向的平面约束 ≥ `minimum_directional_support`；
5. 恢复初始协方差；失败且 `uncertainty.allow_configured_fallback: true` 时改用 `state_variance`。

任一步不满足就清空重来（同一代次）。仿真出生点在补给区角落，静止视野几何单一，所以 `bringup/config/sim/frontend.yaml` 把 `minimum_directional_support` 从 15 降到 5；真车仍为 15。

### 3.3 在线滤波

初始化完成后进入迭代滤波：

| 环节 | 说明 | 相关参数 |
| --- | --- | --- |
| IMU 预测 | 每个 IMU 样本传播状态，并立即发布 `/odometry`、`/frontend/odometry` | `filter.gyro_variance`、`accel_variance`、`*_bias_psd` |
| 观测调度 | 不固定按帧更新，而是根据运动外推误差决定激光更新间隔（0.03–0.12 s） | `window.*` |
| 自适应采样 | 按场景尺度调节体素大小，使代表点数落在 1000–4000 | `sampling.*` |
| 点面关联与更新 | 点到在线概率平面地图做关联，迭代更新，最多 5 次 | `filter.maximum_iterations`、`convergence_tolerance`、`association_threads` |
| 地图更新 | 将新点并入体素平面地图；**体素数超过 `map.maximum_roots` 会报错停止，不会自动淘汰** | `map.*` |
| 输出 | 发布 `/lidar_odometry` 和 `/frontend/frame`（点云按 `output_resolution` 0.10 m 降采样） | `output_resolution` |

`/frontend/frame.lidar_updated=false` 表示这次没有接受激光校正（例如匹配不足），只发布了 IMU 传播结果。

---

## 4. 输入中断、失败与重置

### 4.1 什么算输入中断

| 情况 | 处理 |
| --- | --- |
| 相邻 IMU 间隔 > `maximum_imu_interval`（0.1 s） | 输入中断 |
| IMU 时间回退 | 输入中断 |
| 等待 IMU 期间点云缓存耗尽 | 输入中断 |
| IMU 重复时间戳 | 直接丢弃，不算中断 |

- **还没输出过估计**（初始化中）：清空缓存，同一代次重新初始化。
- **已经在运行**：
  - `coordinated_reset: false`（独立运行默认）：前端自己换代冷启动；
  - `coordinated_reset: true`（bringup）：前端在 `/frontend/session` 置 `reset_required`，后端冻结输出、进入 `LOST`，再调用 `/zlio/reset_internal` 协调重置。

### 4.2 处理失败

数值错误、地图容量超限等普通处理失败：前端在 `/frontend/session` 置 `failed`，停止处理并等待显式重置，不会自动反复重启。后端收到后冻结输出。

### 4.3 重置方式

| 方式 | 行为 | 条件 |
| --- | --- | --- |
| 暖重置 `warm`（默认 `reset_mode`） | 位置、航向归零；保留速度、倾角和 IMU 零偏；状态和协方差换算到新里程计系，并给后端提供新旧坐标系的变换 `old_from_new` | 前端在运行、IMU 历史完整 |
| 冷启动 `cold` | 完全重新初始化（第 3.2 节） | 暖重置条件不满足时自动退化为冷启动 |

冷启动补不回缺测期间的运动：之后必须靠后端用先验地图或 UWB 重新确定全局位姿。**建图模式没有先验地图，冷启动后后端会一直保持 `LOST`。**

### 4.4 服务

| 服务 | 类型 | 说明 |
| --- | --- | --- |
| `/localization_start` | `std_srvs/SetBool` | `wait_for_start` 时传 `data: true` 开始；不支持 `false` |
| `/stop` | `std_srvs/Trigger` | 停止接收输入，处理完缓存后返回，进程继续运行；不会触发自动恢复 |
| `/zlio/reset_internal` | `localization_interfaces/ResetFrontend` | **由后端调用**，按请求者和事务号去重。手动重置请调用后端的 `/localization_reset`（需停车） |

---

## 5. 坐标与外参

| 项 | 约定 |
| --- | --- |
| LiDAR → IMU 外参 | 不在参数文件中。启动时从 TF 查一次 `frames.imu ← frames.lidar` 并固定使用；输入点云的 `frame_id` 不会逐帧触发 TF 转换 |
| 仿真 | 合并点云在 `mid360_link`，gz IMU 与雷达同原点：`frames.imu = frames.lidar = mid360_link`，外参为单位阵 |
| 真车 | `frames.imu = mid360_imu_link`（URDF 中的 Livox 出厂外参，平移 `[-0.011, -0.02329, 0.04412]`），`frames.lidar = mid360_link`。外参只在 URDF 一处维护 |
| IMU | 角速度 rad/s；加速度乘 `imu_acceleration_scale` 后为 m/s²（含重力）；不使用消息里的 orientation 和协方差 |
| 导出点云 | float32 XYZ，统一到观测结束时刻的 IMU 系，不含强度和时间字段 |

检查 IMU 单位：静止时 `ros2 topic echo --once <imu_topic> --field linear_acceleration`，换算后模长应接近 9.81。

---

## 6. 主要参数速查

仿真值来自 `bringup/config/sim/frontend.yaml`，真车值来自 `bringup/config/real/frontend.yaml`；未标注时两者相同。完整列表见 `frontend/docs/parameters.md`。

### 6.1 传感器 `sensor`

| 参数 | 仿真 / 真车 | 说明 |
| --- | --- | --- |
| `gravity` | 9.81 | 必填，范围 [9.6, 10.0] |
| `minimum_range` | 0.3 / 0.5 m | 近距回波过滤，用于去掉车体自身点 |
| `maximum_range` | 40 / 100 m | 远距回波过滤；RMUL 场地 12 m × 8 m，40 m 足够 |

### 6.2 初始化 `initialization`

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `window_frames` | 20 | 初始化使用的扫描帧数 |
| `seed_imu_samples` | 31 | 粗估重力的 IMU 样本数 |
| `minimum_directional_support` | 5（仿真）/ 15（真车） | 最弱方向平面约束；初始化总失败时可适当降低，但越低越容易在退化场景下启动 |
| `optimization.minimum_planes` | 10 | 最少有效平面数 |
| `accel_bias`、`accel_bias_std` | 0 / 0.3 m/s² | 加速度零偏先验 |
| `uncertainty.allow_configured_fallback` | true | 协方差恢复失败时用配置方差 |

### 6.3 滤波 `filter`

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `maximum_iterations` | 5 | 每次激光更新迭代上限 |
| `gyro_variance` / `accel_variance` | 0.25 / 0.5 | IMU 噪声；越大越不信任 IMU 预测 |
| `gyro_bias_psd` / `accel_bias_psd` | 1.33e-6 / 2.15e-5 | 零偏随机游走强度 |
| `association_threads` | 2 | 点面关联线程 |

### 6.4 地图、采样、调度

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `map.voxel_size` | 0.5 m | 在线平面地图根体素；近距多表面可减小 |
| `map.maximum_roots` | 1000000 | 超出即报错停止 |
| `map.range_sigma` / `angle_sigma_degrees` | 0.02 m / 0.05° | 雷达测距、测角标准差 |
| `sampling.minimum_points` / `maximum_points` | 1000 / 4000 | 目标代表点数；几何不足时增大，延迟高时减小 |
| `sampling.initial_size` / `minimum_size` / `maximum_size` | 0.25 / 0.02 / 1.0 m | 采样体素范围 |
| `window.minimum_interval` / `maximum_interval` | 0.03 / 0.12 s | 激光更新间隔范围 |
| `window.normalized_error` | 2.0 | 越小更新越频繁 |

### 6.5 节点参数（ROS）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `maximum_imu_interval` | 0.1 s | 超过即判输入中断 |
| `imu_history_seconds` / `maximum_imu_samples` | 30 s / 65536 | IMU 历史容量（暖重置需要） |
| `maximum_buffered_points` / `maximum_queue_events` | 1e6 / 8192 | 输入队列容量 |
| `output_resolution` | 0.10 m | `/frontend/frame` 点云降采样；设 0 会输出空点云，后端将无法工作 |
| `reset_mode` | warm | 见 4.3 |

所有参数只在启动时读取，修改后需重启节点。

---

## 7. 已知问题与注意事项

### 7.1 重复的雷达融合节点

上一次 launch 被强制关闭时，`merge_node` 可能残留。`/lidar/points` 出现两个发布者后，ZLIO 收到重复、乱序的帧，日志出现 `IMU input interrupted`，后端随之频繁 `SUSPECT`/`LOST`。启动前按 2.3 检查。（正常 Ctrl-C 退出时的报错已在 `f14c083` 修复。）

### 7.2 仿真性能

- 非交互 shell 必须设置 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`，否则 gpu_lidar 回退软件渲染。
- 开 Gazebo GUI + RViz 时实测 RTF 约 0.72；测定位时建议关 GUI。
- `ros2 topic hz` 对大点云严重低估频率，用 `tools/sim_tests/cloud_rate.py`。

### 7.3 前端感知不到“瞬移”

仿真绑架测试用 `set_pose` 瞬移车辆，不产生 IMU 运动，前端里程计不会有任何变化，只能靠后端发现。真实碰撞会有 IMU 冲击，情况不同。

### 7.4 精度参考（仿真，2026-10-27，RTF 1.0）

出生点南侧约 21 s 路线（平移、转向、侧移、停车），ZLIO 位姿对比 Gazebo 真值：

| 模式 | 最大位置误差 | 结束误差 | 最大航向误差 |
| --- | --- | --- | --- |
| single | 7.3 cm | 1.0 cm | 0.43° |
| dual | 1.9 cm | 0.4 cm | 0.16° |

只是短路线、各一次，说明链路正常，不代表全场长时间精度。长时间漂移由后端修正。

---

## 8. 排查流程

```bash
ros2 topic info /lidar/points              # 发布者必须为 1
ros2 topic hz /lidar_odometry              # ≈ 激光更新频率
ros2 topic echo --once /frontend/session   # reset_required / failed
ros2 topic echo --once /lidar_mid360/imu --field linear_acceleration   # 单位检查
```

| 现象 | 可能原因 | 检查 |
| --- | --- | --- |
| `/odometry` 一直没有输出 | 初始化未通过 | 车是否静止、视野是否有足够平面；`imu_acceleration_scale` 是否正确；仿真可看 `minimum_directional_support` |
| `IMU input interrupted` | IMU 断流、时间回退、重复节点 | `ros2 topic hz` IMU；按 2.3 检查重复节点；RTF 是否过低 |
| `/frontend/session` 的 `failed=true` | 处理错误（数值、地图容量） | 前端日志 ERROR；需要通过后端 `/localization_reset` 重置 |
| 静止时位姿漂移 | IMU 单位或零偏错误、外参错误 | `imu_acceleration_scale`、URDF 中 `frames.imu ← frames.lidar` 外参 |
| `lidar_updated=false` 频繁 | 点面匹配不足 | 场景退化、`sampling.minimum_points`、`minimum_range` 是否把有效点滤掉 |
| 转弯时轨迹扭曲 | 逐点时间错误，去畸变失效 | `point_time_*` / `lidar_merge` 时间字段配置 |
