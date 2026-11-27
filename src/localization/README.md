# 定位与建图

定位算法移植自上游 z-localization（东北大学 T-DT，ZLIO 前端 + 因子图后端，GPL-2.0-only），
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
| `frontend` | 上游 | ZLIO 激光惯性里程计（`zlio_node`），输出观测与高频预测；组合启动关闭其 TF 发布 |
| `backend` | 上游 | 因子图后端：建图、回环、先验地图定位；组合启动成对发布两条动态 TF；局中用 UWB 或同代里程计预测作局部恢复先验 |
| `localization_interfaces` | 上游 | 前后端消息与重置服务 |
| `livox_ros_driver2` | 上游 | 真车 MID-360 驱动 |
| `lidar_fusion` | 新增 | 单/双雷达合并为一路带 `offset_time` 的 PointCloud2 |
| `lidar_calibration` | 新增，核心移植自 navi_minco_bit | L2 → MID-360 外参标定（PCL GICP） |
| `bringup` | 重写 | `localization.launch.py`（共用管线：融合 + 前端 + 后端）、`sim.launch.py` / `real.launch.py`（数据源、真车底盘通信）与 `config/{sim,real}/` |

## 依赖与编译（Ubuntu 24.04 / ROS 2 Jazzy）

```bash
sudo apt install ros-jazzy-gtsam libpcl-dev ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
  ros-jazzy-tf2-eigen ros-jazzy-tf2-sensor-msgs ros-jazzy-message-filters ros-jazzy-sensor-msgs-py \
  ros-jazzy-joint-state-publisher ros-jazzy-ament-cmake-auto ros-jazzy-rosidl-default-generators \
  libnanoflann-dev libtbb-dev libopenmpi-dev libyaml-cpp-dev libapr1-dev
# Livox-SDK2：从 github.com/Livox-SDK/Livox-SDK2 源码编译安装（GCC 13 需 -DCMAKE_CXX_FLAGS='-include cstdint'）

```

工作区构建命令见[项目首页](../../README.md)。WSL 上检查 GPU 渲染和 DDS 缓冲区设置。

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
| `lidar` | `single` | `single` / `dual` |
| `mode` | `mapping` | `mapping` / `localization` |
| `prior_map` | 空 | 先验地图目录；`localization` 时留空则用 `map_dir` |
| `map_dir` | `<工作空间>/maps/sim`（真车 `maps/real`） | `~/save_map` 的保存目录，已加入 `.gitignore` |
| `rviz` | `false` | 后端 RViz |

配置在 `bringup/config/common/` 共用；`config/sim/overrides.yaml` 仅保存 Gazebo 差异，`config/single_lidar.yaml` 保存单雷达差异。启动时合成为完整参数，详见[参数规则](#参数规则)。

### 单雷达操作

MID-360 的数据坐标系与 CAD 网格坐标系分别定义；`yaw_joint=0` 时数据系相对车体先 yaw 180°、再绕局部 X 轴 roll −15°（`Rz(π) · Rx(−π/12)`），不能单看 `mid360_joint` 的欧拉俯仰角判断侧装。单雷达转车补视野的仿真操作：

先保持静止等待初始化。视野结构不足时可通过仿真专用 `/sim/turret_yaw` 转动云台补充视野；普通 `/cmd_vel.angular.z` 与真车一样不驱动底盘旋转。
传感器专项回归 `tools/sim_tests` 显式关闭 MCU 接口并直接控制物理底盘，其原地旋转只用于定位算法测试。

单雷达与双雷达后端配置差异（共用 `config/single_lidar.yaml`）：

| 键 | 双雷达 | 单雷达 | 原因 |
| --- | --- | --- | --- |
| `common.scan_context.similarity_normalization` | `overlap` | `union` | 单侧视野下 `overlap` 让所有候选描述子距离为 0；`union` 用在双雷达会使恢复变差 |
| `tracking.source.max_frames` / `max_time` | 5 / 0.5 s | 15 / 1.5 s | 单帧在 0.4 m 体素下只剩约 90 点，不够 `min_correspondences: 80` |
| `registration.voxel_resolution` | 0.4 | 0.25（仅仿真） | 累积后 0.4 m 体素只剩约 136 点；代价是绑架后错误恢复更严重，见[已知问题](#已知问题) |

仿真与真车配置不同的地方（均为 yaml 取值）：

| 文件 / 键 | 仿真 | 真车 | 原因 |
| --- | --- | --- | --- |
| `frontend.yaml` `imu_acceleration_scale` | 1.0 | 9.81 | gz IMU 单位 m/s²（Livox 为 g） |
| `frontend.yaml` `initialization.minimum_directional_support` | 5 | 15 | 出生点补给区静止视野几何单一，15 时初始化报最弱方向支撑不足 |
| `frontend.yaml` `minimum_range` / `maximum_range` | 0.3 / 40 m | 0.5 / 100 m | 仿真场地小、无近距离噪声 |
| `lidar_fusion.yaml` `primary.*` / `secondary.*` 逐点时间 | 帧头时间 | 驱动时间字段 | gpu_lidar 一帧同一时刻渲染 |
| `backend*.yaml` `registration.convergence_translation` / `convergence_rotation_deg` | 0.02 m / 0.5° | 0.005 m / 0.1°（上游值） | 仿真中 5 mm / 0.1° 使配准常判未收敛、跟踪频繁 SUSPECT；真车未验证，保持上游 |
| `backend*.yaml` `registration.max_iterations` | 30 | 16 | 同上 |

LIO 里程计系为 `lio_odom`。真车与仿真统一使用 `map → lio_odom → gimbal_nav → yaw_link → 传感器`。
`gimbal_nav` 位于同轴旋转中心，采用云台的前/左/上方向，匹配下位机普通 X/Y 参考。
真实底盘 yaw 不可观测，真车不发布假的 `base_link` 或默认零关节状态；仿真的物理关节反馈也只供模拟器接口使用。
组合启动时前端 `output.publish_tf=false`，后端统一发布 map→lio_odom 和 lio_odom→配置的根帧，
同一时间戳、同一投影，保证重置时不会拼接不同代次。

真车 `sensor_frame ← body_frame` 为 `mid360_imu_link ← gimbal_nav` 的固定外参。
`gimbal_description.py` 从原 URDF 提取刚性云台子树，保留 MID-360、内置 IMU 和 L2 的安装外参；
`yaw_link` 是 CAD 轴向，不能直接当平面导航帧。`gimbal_yaw_offset` 默认为 0，仅在实测确认 MCU +X 与 CAD
云台零方向存在安装偏角后设置（rad，逆时针为正）；它不是云台相对底盘的关节角。
零偏角时，新外参与旧 yaw_joint=0 的外参完全相同，因此此修复不会凭空消除原先速度方向残差。
UWB 天线固定于底盘时，缺少关节反馈也无法用固定云台杆臂推算天线位置；当前真车 UWB 保持关闭。

## 真车

```bash
ros2 launch bringup real.launch.py lidar:=single
ros2 launch bringup real.launch.py lidar:=dual driver_l2:=true l2_launch:=/abs/path/to/unilidar_sdk2/launch/xxx.launch.py
ros2 launch bringup real.launch.py mode:=localization                 # 加载 ~/yang_nav/maps/real
ros2 launch bringup real.launch.py mode:=localization prior_map:=/abs/map
```

真车入口默认启动 `chassis_comm`。默认使用固定串口 `/dev/nav`、波特率 `115200`，启动命令无需传入串口参数；需要临时调整时可用 `chassis_port`、`chassis_baud_rate` 覆盖。底盘默认 100 Hz、`yaw_mode=action_angle`：普通导航使用云台方向参考的 X/Y 速度，动作上升沿使用显式底盘机体系相对 yaw 角度，参数详见[底盘接口](../platform/chassis_comm/README.md)。

- MID-360：`livox_ros_driver2`，参数在 `bringup/config/real/livox_driver.yaml`（`xfer_format=0` PointCloud2，`frame_id: mid360_link`，无需别名 TF）。
  **雷达与主机 IP 改 `bringup/config/real/MID360_config.json`**，不要改驱动包里的那份。
- L2：宇树 `unilidar_sdk2`（需另行安装）；其 launch 不归本仓库管，故保留 `l2_frame` → `l2_link` 的恒等静态 TF。
- IMU：MID-360 内置 IMU，`imu_acceleration_scale=9.81`，外参是 URDF 中 `mid360_imu_link` 的 Livox 出厂值。
- TF：启动 `robot_state_publisher`，根为 `gimbal_nav`，只发布可观测的固定云台子树；不启动默认零关节发布器。
  接通真实关节反馈后，应另行恢复有依据的物理底盘 TF；普通 X/Y 仍应保持云台参考。
- 后端：`bringup/config/common/backend.yaml`；单雷达仅叠加 `config/single_lidar.yaml`。
  真车默认 `uwb.enabled: false`，`SUSPECT`/`LOST` 在同代里程计预测附近尝试局部恢复；没有同代可信锚或漂移超出局部范围时，停稳并停止导航后可调用 `/backend_node/global_relocalize`。
  接入裁判系统 UWB 后可启用，并按 `backend/docs/parameters.md` 的 `uwb` 小节标定。

**以下取值未在真车验证（本机无硬件），上车先核对**，改启动参数或 yaml 即可，不用改代码：

| 位置 | 参数 / 键 | 当前值 | 核对方法 |
| --- | --- | --- | --- |
| launch 参数 | `mid360_topic` / `imu_topic` | `/livox/lidar` / `/livox/imu` | `ros2 topic list` |
| `config/real/MID360_config.json` | 雷达 / 主机 IP | 见当前 JSON | 按车上网络修改 |
| 云台参考 | `gimbal_yaw_offset` | 0（名义安装方向，非实测标定） | 核对 MCU +X/+Y 与雷达安装轴向；不要用关节零值替代 |
| `config/common/lidar_fusion.yaml` | `primary.time_field` / `primary.time_scale` / `primary.time_absolute` | `timestamp` / `1.0e-9` / `true` | `ros2 topic echo --once /livox/lidar --field fields`，看 `timestamp` 是绝对时间还是帧内偏移 |
| launch 参数 | `l2_topic` / `l2_frame` | `/unilidar/cloud` / `unilidar_lidar` | 同上 |
| `config/common/lidar_fusion.yaml` | `secondary.time_field` / `secondary.time_scale` / `secondary.time_absolute` | `time` / `1.0` / `false` | 同上 |

逐点时间写错会让 ZLIO 去畸变出错甚至发散，务必确认。

## 参数规则

- `bringup/config/common/` 是完整系统调参的唯一完整来源；模块 `config/example.yaml` 只供独立运行与接口检查。
- `config/sim/overrides.yaml` 仅保留数据单位、逐点时间、采样与 UWB 场地差异；`config/real/` 保留硬件驱动配置，可用 `overrides.yaml` 添加明确的硬件差异。
- 单雷达共用 `config/single_lidar.yaml`，仿真额外使用同文件 `sim` 节的体素设置，不再维护成套 backend_single 副本。
- 启动按“common → 平台 → 单雷达共用 → 单雷达平台”合成临时完整 YAML，退出后删除临时文件。覆盖中的未知键直接报错。
- 运行 `python3 tools/check_bringup_params.py`，检查每种平台/雷达组合与模块 example 的键集合、允许差异、TF 归属，以及覆盖中没有重复的共用值。
- launch 仅负责时间、输入话题、地图目录和运行选择；传感器安装外参只在 URDF 维护。

## 双雷达外参标定

```bash
# 仿真：先启动 sentry_gazebo sim.launch.py 与 bringup sim.launch.py lidar:=dual（提供 TF 与 /odometry）
ros2 launch bringup calibration.launch.py platform:=sim
# 真车：驱动、URDF TF 与前端须已在运行（如 real.launch.py lidar:=dual driver_l2:=true ...）
ros2 launch bringup calibration.launch.py platform:=real output_file:=/abs/lidar_extrinsic.yaml
```

| launch 参数 | 默认 | 说明 |
| --- | --- | --- |
| `platform` | `sim` | `sim` / `real`；选择 common 参数与平台标定差异，仿真时 `use_sim_time=true` |
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

单侧雷达在低结构或对称场景可能出现初始化缓慢、错误恢复和航向偏差。先验定位的开机搜索以建图起点或 `startup_hint` 为中心。局中恢复需要启用 UWB 并得到新鲜位置，半径和允许偏差见对应平台配置。前端断流跨代次时，建图模式会隔离失去连续性的里程计。停车后可按后端的重置流程继续建图。

真车联调时核对雷达逐点时间、主机 IP、云台反馈和底盘串口。仿真和离线测试的输入及判据见[定位回归](../../tools/sim_tests/README.md)。

## 模块参考

前端初始化与参数见[前端](frontend/README.md)和[前端参数](frontend/docs/parameters.md)。地图保存与重置服务见[后端](backend/README.md)，恢复状态与阈值见[后端参数](backend/docs/parameters.md)。
