# z-localization（yang_nav 移植版）

移植自上游 z-localization（东北大学 T-DT，ZLIO 前端 + 因子图后端，GPL-2.0-only），
并融合 navi_minco_bit 的双雷达标定思路，适配本机器人：**Livox MID-360（云台上）+
Unitree L2（云台上、朝后）**，可选单雷达或双雷达。仿真与真车各一个启动文件，融合、前端、后端完全共用。

```
MID-360 点云 ─┐
              ├─ lidar_fusion (message_filters 同步 + tf2 外参 + 逐点时间) ─ /lidar/points ─┐
L2 点云 ──────┘ (dual 时)                                                                   ├─ zlio_node ─ backend_node
MID-360 内置 IMU ───────────────────────────────────────────────────────────────────────────┘
```

外参**只存在 URDF 一处**（`sentry_description/urdf/sentry_robot.urdf`），融合节点与标定工具都经 tf2 读取。

## 包

| 包 | 来源 | 作用 |
| --- | --- | --- |
| `frontend` | 上游 | ZLIO 激光惯性里程计（`zlio_node`），发布 `lio_odom → base_link` |
| `backend` | 上游 | 因子图后端：建图、回环、先验地图定位；局中恢复（SUSPECT/LOST）用裁判系统 UWB 作位置先验 |
| `localization_interfaces` | 上游 | 前后端消息与重置服务 |
| `livox_ros_driver2` | 上游 | 真车 MID-360 驱动 |
| `lidar_fusion` | 新增 | 单/双雷达合并为一路带 `offset_time` 的 PointCloud2 |
| `lidar_calibration` | 新增，核心移植自 navi_minco_bit | L2 → MID-360 外参标定（PCL GICP） |
| `bringup` | 重写 | `localization.launch.py`（共用管线：融合 + 前端 + 后端）、`sim.launch.py` / `real.launch.py`（只管数据源）与 `config/{sim,real}/` |

对上游代码的改动：

- 编译修正：`backend/src/keyframe_map.cpp`（nanoflann 1.5 的 `ResultItem`）、`rclcpp::ServicesQoS()`、
  各 `cmake/Warnings.cmake` 中 `LOCALIZATION_WARNINGS_AS_ERRORS` 默认 `OFF`（GCC 13 对 Eigen 误报）。
- 参数：前端不再读自定义 `lio.yaml`，前后端都只读标准 ROS 参数文件，代码无默认值，缺失/未知参数启动即失败；
  话题改为固定相对名由 launch remap。
- 外参：前端 `sensor.body_from_lidar`、后端 `common.body.*` 删除，改为从 TF（URDF）查询；
  前端自己发布 `world → base`（替代原 `lidar_fusion/base_tf` 节点），后端按消息时间查车体外参。

## 依赖与编译（Ubuntu 24.04 / ROS 2 Jazzy）

```bash
sudo apt install ros-jazzy-gtsam libpcl-dev ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
  ros-jazzy-tf2-eigen ros-jazzy-tf2-sensor-msgs ros-jazzy-message-filters ros-jazzy-sensor-msgs-py \
  ros-jazzy-joint-state-publisher ros-jazzy-ament-cmake-auto ros-jazzy-rosidl-default-generators \
  libnanoflann-dev libtbb-dev libopenmpi-dev libyaml-cpp-dev libapr1-dev
# Livox-SDK2：从 github.com/Livox-SDK/Livox-SDK2 源码编译安装（GCC 13 需 -DCMAKE_CXX_FLAGS='-include cstdint'）

cd ~/yang_nav
# --base-paths src：根目录若检出了上游仓库（reference/，已 gitignore）会有同名包，须排除；限制并行避免 16 GB 内存下 OOM
MAKEFLAGS=-j6 colcon build --base-paths src --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

WSL 下两项系统设置（缺了 MID-360 点云会掉到 2–3 Hz）：

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA       # 让 gpu_lidar 用独显，写入 ~/.bashrc
# /etc/sysctl.d/60-ros2-dds.conf：net.core.rmem_max/wmem_max=67108864，rmem/wmem_default=8388608
```

## 仿真

```bash
ros2 launch sentry_gazebo sim.launch.py              # 终端 1（gui:=true 可看画面）
ros2 launch bringup sim.launch.py lidar:=dual        # 终端 2；lidar:=single 只用 MID-360
ros2 run sentry_gazebo teleop                        # 终端 3，键盘开车
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `lidar` | `dual`（真车 `single`） | `single` / `dual` |
| `mode` | `mapping` | `mapping` / `localization` |
| `prior_map` | 空 | 先验地图目录；`localization` 时留空则用 `map_dir` |
| `map_dir` | `<工作空间>/maps/sim`（真车 `maps/real`） | `~/save_map` 的保存目录，已加入 `.gitignore` |
| `rviz` | `false` | 后端 RViz |

配置：`bringup/config/sim/{lidar_fusion,frontend,backend}.yaml`（真车为 `bringup/config/real/`），各为完整参数文件，
见下文[参数规则](#参数规则)。`lidar:=single` 时后端改读同目录的 `backend_single.yaml`。

### 单雷达操作

MID-360 侧装（`mid360_joint` 俯仰约 90°），静止时只看到一侧，单雷达要转车补视野（仅仿真验证）：

1. **启动**：`zlio_node` 开始收数据后先静止约 2 s，再以约 0.3 rad/s 原地慢转，直到后端进入 NORMAL。静止时 ZLIO 初始化最弱方向支撑只有 2–4.8（门槛 5），会反复重来甚至几分钟不完成；转动后 3 s 内完成。
2. **建图**：先在出生点慢转一圈再正常跑图，其余路段不要原地转（全程转圈会把建图误差拉到约 27 cm）。地图在出生点缺少其他航向的关键帧时，启动慢转会让重定位锁到对称位置。
3. 真车可考虑转云台 `yaw_joint` 代替转底盘（未验证）。

`tools/sim_tests` 的 `earlyspin` / `spawnspin` 标志按上述流程驾驶。

单雷达与双雷达后端配置差异（`backend_single.yaml` 开头也有记录）：

| 键 | 双雷达 | 单雷达 | 原因 |
| --- | --- | --- | --- |
| `common.scan_context.similarity_normalization` | `overlap` | `union` | 单侧视野下 `overlap` 让所有候选描述子距离为 0；`union` 用在双雷达会使恢复变差 |
| `tracking.source.max_frames` / `max_time` | 5 / 0.5 s | 15 / 1.5 s | 单帧在 0.4 m 体素下只剩约 90 点，不够 `min_correspondences: 80` |
| `registration.voxel_resolution` | 0.4 | 0.25（仅仿真） | 累积后 0.4 m 体素只剩约 136 点；代价是绑架后错误恢复更严重，见[已知问题](#已知问题) |

仿真与真车配置不同的地方（均为 yaml 取值）：

| 文件 / 键 | 仿真 | 真车 | 原因 |
| --- | --- | --- | --- |
| `frontend.yaml` `frames.imu` / `frames.lidar` | `mid360_link` / `mid360_link` | `mid360_imu_link` / `mid360_link` | 仿真 IMU 与雷达同原点（外参为单位阵）；真车为 Livox 内置 IMU |
| `backend.yaml` `common.sensor_frame` | `mid360_link` | `mid360_imu_link` | 与前端 `frames.imu` 一致 |
| `frontend.yaml` `imu_acceleration_scale` | 1.0 | 9.81 | gz IMU 单位 m/s²（Livox 为 g） |
| `frontend.yaml` `initialization.minimum_directional_support` | 5 | 15 | 出生点补给区静止视野几何单一，15 时初始化报最弱方向支撑不足 |
| `frontend.yaml` `minimum_range` / `maximum_range` | 0.3 / 40 m | 0.5 / 100 m | 仿真场地小、无近距离噪声 |
| `lidar_fusion.yaml` `primary.*` / `secondary.*` 逐点时间 | 帧头时间 | 驱动时间字段 | gpu_lidar 一帧同一时刻渲染 |
| `backend*.yaml` `registration.convergence_translation` / `convergence_rotation_deg` | 0.02 m / 0.5° | 0.005 m / 0.1°（上游值） | 仿真中 5 mm / 0.1° 使配准常判未收敛、跟踪频繁 SUSPECT；真车未验证，保持上游 |
| `backend*.yaml` `registration.max_iterations` | 30 | 16 | 同上 |

LIO 的里程计系叫 `lio_odom`。仿真与真车 TF 相同，是一棵连通的树 `map → lio_odom → base_link → URDF 树`：
`lio_odom → base_link` 由 ZLIO 直接发布（`output.publish_tf: true`、`frames.base: base_link`）：里程计 `lio_odom → frames.imu`
乘以按里程计时间戳从 TF 查到的 `frames.imu ← base_link`，云台 `yaw_joint` 转动也会跟随。IMU 系在 URDF 里已有父帧，
所以里程计接在树根 `base_link` 上，而不是直接发 `lio_odom → frames.imu`。
不再有手写的 `imu` 静态 TF：MID-360 内置 IMU 是 URDF 中的 `mid360_imu_link`（Livox 出厂外参），
前端启动时从 TF 查一次 `frames.imu ← frames.lidar`（同在雷达上，固定）；后端按每条消息的时间戳查
`common.sensor_frame ← common.body_frame`（`base_link`），超过 `common.body_extrinsic_tolerance` 时沿用上次有效值并告警，
所以云台 `yaw_joint` 转动时车体位姿与 UWB 杆臂也跟随（启动位姿提示仍用启动时外参）。后端只发布 `map → lio_odom`，
不发 `map → base_link`。Gazebo 不发布真值 `/odom` / `odom` 帧。

### 仿真实测（2026-10-27，RTX 4060 Ti，RTF 1.0）

出生点南侧约 21 s 路线（平移、转向、侧移、停车），ZLIO 位姿对比 Gazebo 真值：

| 模式 | 合并点云 | 位移 | 最大位置误差 | 结束误差 | 最大航向误差 |
| --- | --- | --- | --- | --- | --- |
| single | 10 Hz | 1.13 m | 7.3 cm | 1.0 cm | 0.43° |
| dual | 10 Hz | 1.08 m | 1.9 cm | 0.4 cm | 0.16° |

只是短路线、各一次，说明链路工作正常，不代表全场长时间精度。最新回归结果见 [`tools/sim_tests/README.md`](../../tools/sim_tests/README.md)。

注意：后端 SUSPECT/LOST 时输出冻结在最后可信位姿，行驶中短暂 SUSPECT 也会带来 0.3–0.5 m 的瞬时误差，评估时应分开看可信样本。

## 真车

```bash
ros2 launch bringup real.launch.py lidar:=single
ros2 launch bringup real.launch.py lidar:=dual driver_l2:=true l2_launch:=/abs/path/to/unilidar_sdk2/launch/xxx.launch.py
ros2 launch bringup real.launch.py mode:=localization                 # 加载 ~/yang_nav/maps/real
ros2 launch bringup real.launch.py mode:=localization prior_map:=/abs/map
```

- MID-360：`livox_ros_driver2`，参数在 `bringup/config/real/livox_driver.yaml`（`xfer_format=0` PointCloud2，`frame_id: mid360_link`，无需别名 TF）。
  **雷达与主机 IP 改 `bringup/config/real/MID360_config.json`**，不要改驱动包里的那份。
- L2：宇树 `unilidar_sdk2`（需另行安装）；其 launch 不归本仓库管，故保留 `l2_frame` → `l2_link` 的恒等静态 TF。
- IMU：MID-360 内置 IMU，`imu_acceleration_scale=9.81`，外参是 URDF 中 `mid360_imu_link` 的 Livox 出厂值。
- TF：本文件启动 `robot_state_publisher`（URDF）和 `joint_state_publisher`（`bringup/config/real/robot_state.yaml`）。
  云台 `yaw_joint` 取自底盘发布的 `/chassis/joint_states`；该话题没有数据时，所有关节发 0。
- 后端：`bringup/config/real/backend.yaml`（`lidar:=single` 时为 `backend_single.yaml`，共用参数两份都要改）。
  真车 `uwb.enabled: false`（`uwb.zero_pose_*` 未标定），此时 SUSPECT/LOST 不会自动恢复；接入裁判系统 UWB 后在这里打开，并按
  `backend/docs/parameters.md` 的 `uwb` 小节配置。

**以下取值未在真车验证（本机无硬件），上车先核对**，改启动参数或 yaml 即可，不用改代码：

| 位置 | 参数 / 键 | 当前值 | 核对方法 |
| --- | --- | --- | --- |
| launch 参数 | `mid360_topic` / `imu_topic` | `/livox/lidar` / `/livox/imu` | `ros2 topic list` |
| `config/real/MID360_config.json` | 雷达 / 主机 IP | 驱动包原值 | 按车上网络修改 |
| 底盘节点 | `/chassis/joint_states` 中的 `yaw_joint` | 未接入时为 0 | `ros2 topic echo /joint_states` |
| `config/real/lidar_fusion.yaml` | `primary.time_field` / `primary.time_scale` / `primary.time_absolute` | `timestamp` / `1.0e-9` / `true` | `ros2 topic echo --once /livox/lidar --field fields`，看 `timestamp` 是绝对时间还是帧内偏移 |
| launch 参数 | `l2_topic` / `l2_frame` | `/unilidar/cloud` / `unilidar_lidar` | 同上 |
| `config/real/lidar_fusion.yaml` | `secondary.time_field` / `secondary.time_scale` / `secondary.time_absolute` | `time` / `1.0` / `false` | 同上 |

逐点时间写错会让 ZLIO 去畸变出错甚至发散，务必确认。

## 参数规则

- **参数只有一个来源**：每个节点参数都来自 ROS 参数文件，代码中没有默认值；缺失参数或未知参数都会使启动失败。
  参数文件不支持继承或合并。外参不是参数，只在 URDF 中维护。
- **模块 example 与 bringup 配置**：`lidar_fusion`、`frontend`、`backend`、`lidar_calibration` 各有 `config/example.yaml`，只用于单独启动该模块和测试；
  完整系统只读 `bringup/config/<platform>/{lidar_fusion,frontend,backend,lidar_calibration}.yaml`（`<platform>` 为 `sim` 或 `real`；单雷达后端为 `backend_single.yaml`），
  它们是完整副本，不引用 example。
- **键集合检查**：增删或改名参数后，在工作空间根目录运行 `python3 tools/check_bringup_params.py`，
  检查 bringup 各配置的键集合与对应模块 example 完全一致。
- **launch 参数与 yaml 的分工**：launch 只提供 `params_file`、`use_sim_time`、话题 remap 和运行时选择
  （`lidar`、`mode`、`prior_map`、`map_dir`、`rviz`，真车另有驱动与话题/frame 参数），这些不在 yaml 中出现；
  其余调参项只写 yaml。后端的 `prior_map.enabled`、`prior_map.directory`、`trajectory.save_directory`
  属于 launch 侧。

## 双雷达外参标定

```bash
# 仿真：先启动 sentry_gazebo sim.launch.py 与 bringup sim.launch.py lidar:=dual（提供 TF 与 /odometry）
ros2 launch bringup calibration.launch.py platform:=sim
# 真车：驱动、URDF TF 与前端须已在运行（如 real.launch.py lidar:=dual driver_l2:=true ...）
ros2 launch bringup calibration.launch.py platform:=real output_file:=/abs/lidar_extrinsic.yaml
```

| launch 参数 | 默认 | 说明 |
| --- | --- | --- |
| `platform` | `sim` | `sim` / `real`；选择 `bringup/config/<platform>/lidar_calibration.yaml`，仿真时 `use_sim_time=true` |
| `output_file` | `/tmp/lidar_extrinsic.yaml` | 结果文件 |
| `primary_topic` / `secondary_topic` / `imu_topic` / `odometry_topic` | 空（取平台默认） | 仿真 `/lidar_mid360/points`、`/lidar_l2/points`、`/lidar_mid360/imu`、`/odometry`；真车 `/livox/lidar`、`/unilidar/cloud`、`/livox/imu`、`/odometry` |

标定入口只启动标定节点，不启动驱动、URDF 或前端：初值、点云帧变换与结果换算都要 TF，位姿多样性检查要 ZLIO `/odometry`。
参数、质量门与输出格式详见 [lidar_calibration/README.md](lidar_calibration/README.md)。

流程：同步帧对 → 体素降采样 → PCL GICP（初值取 URDF，`initial_guess_from_tf`）→ 质量门 → 多帧中值剔除 + 加权平均。
质量门：只收静止帧（IMU 角速度与里程计线速度都低于阈值）；每帧须与已接受帧相距 ≥0.3 m 或 ≥20°
（ZLIO `/odometry`，`require_distinct_poses`）；重叠区法向信息矩阵最小特征值 ≥0.09（几何退化检测）；点到面 RMSE ≤0.05 m。

采集方法：把车停到**墙角、台阶旁等结构丰富处**，换位置和朝向，每处静止 2 s 以上，收满 `frames`（真车 6、仿真 4，至少 3）。
只看得到地面的位置会被判为 `degenerate_geometry` 拒收，这是故意的：单位置标定曾得出平移错 0.72 m 但帧间高度一致的结果。

输出 `output_file`，含 L2→MID-360 变换和可直接填回 URDF 的 `l2_joint` origin。
仿真实测（2026-10-31，4 帧，`tools/sim_tests` 的 `calib` 模式）：平移误差约 1 cm（标准差 1.4 cm），旋转误差 < 1°。

## 已知问题

- 后端偶尔打印 `invalid ZLIO observation frame/time/point layout`（一次运行 0–2 条），原因未查，未见对精度的影响。
- 仿真出生点附近结构少，标定在该处全部因 `degenerate_geometry` 拒收，需开到拐角采集。
- 绑架后恢复可能锁到对称位置或相似关键帧（场地中心对称）。修复前：双雷达 `FALSE TRUST` 0.6–14 s；单雷达（体素 0.25）12–17 s、结束时仍偏约 12 m。绑架恢复修复（已合入 main）加了严重冲突提前进 `SUSPECT`、恢复试用期、歧义检验和运动门（见 `docs/backend_guide.md` 4.5–4.6、7.3）：恢复期间车或云台转动时，双雷达 5/5 恢复、恢复后可信最大误差 ≤ 0.31 m；单雷达仍只有 1/5 干净恢复，常锁到约 99°/180° 的错误位姿。车不动时运动门不起作用，停车兜底可能提交错误解（两者已于 2026-11-08 删除）。`recovery.confirmations` 改为 3 无改善。2026-11-08 起开机初始化只在起点或 `startup_hint` 附近 2 m 内搜索，局中恢复只在 UWB 位置 2 m 内搜索并检查与 UWB 的偏差（`docs/backend_guide.md` 4.2），不做全局重定位；开机位置须在建图起点 2 m 内（或配置 `startup_hint`），没有 UWB 时 SUSPECT/LOST 不会自动恢复。
- 单雷达有一次绑架后已正确恢复（3 cm），行驶中又跑偏到 8.4 m，原因未查。
- 跟踪与恢复/重定位共用 `registration.voxel_resolution`：单雷达跟踪需要 0.25，恢复用 0.25 更易误匹配，暂无法分开设置。
- 单雷达跟踪时配准偶尔把航向拉偏 1–10°（预测航向是准的），推测侧装单雷达偏航约束弱，未单独验证。
- 单雷达先验定位起点偶尔偏约 22 cm（2 次），原因未查。
- ZLIO 初始化失败重来时会在转动中重新估计重力；纯原地慢转影响很小，人手推车可能偏差较大，未处理。
- SUSPECT/LOST 时后端输出冻结在最后可信位姿（`backend/src/pose_output.cpp`），行驶中误差随距离增长；是否改为里程计外推待定。
- 前端 IMU 采样中断（`front-end input discontinuity`）后，建图模式下后端停在 LOST 直到结束（仿真 2 次）。读代码确认的原因：`notifyFrontendDiscontinuity` → `isolateFrontendLocked` 把 `odometry_connected_` 置为 false，重置完成后 `commitFrontendResetBoundaryLocked` 在无先验地图时调用 `forceLost`，暖重置返回的 `old_from_new` 也不再用于衔接（`backend/src/reset_coordinator.cpp`）。这是有意设计：`old_from_new` 只对应中断前最后一次后验，补不回断流期间车的运动；若直接用它衔接，车在断流中移动过时会把错误的边写进地图，而状态仍报可信。需要继续建图时，停车后手动调用 `/localization_reset`。
- 真车路径只验证了编译、launch 展开与无驱动启动（外参、云台关节跟随）。

## 上游文档

地图制作（保存、清理、CloudCompare 精修、生成先验地图包）沿用上游流程，详见上游仓库 README 与 `docs/design.md`（本仓库未收录）；
本仓库内的地图工具列表见 [backend/README.md](backend/README.md#地图与标定工具)。
参数说明见 [frontend/docs/parameters.md](frontend/docs/parameters.md)、[backend/docs/parameters.md](backend/docs/parameters.md)（取值以本仓库 yaml 为准）。
