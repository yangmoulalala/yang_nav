# z-localization（yang_nav 移植版）

移植自 `reference/z-localization`（东北大学 T-DT，ZLIO 前端 + 因子图后端，GPL-2.0-only），
并融合 `reference/navi_minco_bit` 的双雷达标定思路，适配本机器人：**Livox MID-360（云台上）+
Unitree L2（底盘下、朝后）**，可选单雷达或双雷达。仿真与真车各一个启动文件，融合、前端、后端完全共用。

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
| `frontend` | 上游，原样 | ZLIO 激光惯性里程计（`zlio_node`） |
| `backend` | 上游，两行编译修正 | 因子图后端：建图、回环、先验地图定位、UWB 辅助（保留，默认关闭） |
| `localization_interfaces` | 上游 | 前后端消息与重置服务 |
| `livox_ros_driver2` | 上游 | 真车 MID-360 驱动 |
| `lidar_fusion` | 新增 | 单/双雷达合并为一路带 `offset_time` 的 PointCloud2；`base_tf` 把 `/odometry` 换算为 `lio_odom → base_link` TF |
| `lidar_calibration` | 新增，核心移植自 navi_minco_bit | L2 → MID-360 外参标定（PCL GICP） |
| `bringup` | 重写 | `sim.launch.py`、`real.launch.py` 与配置 |

对上游代码的改动只有：`backend/src/keyframe_map.cpp`（nanoflann 1.5 的 `ResultItem`）、
`backend/apps/node_setup.cpp`（`rclcpp::ServicesQoS()` 替代已弃用的 `rmw_qos_profile`），
以及各 `cmake/Warnings.cmake` 中 `LOCALIZATION_WARNINGS_AS_ERRORS` 默认改为 `OFF`（GCC 13 对 Eigen 误报）。

## 依赖与编译（Ubuntu 24.04 / ROS 2 Jazzy）

```bash
sudo apt install ros-jazzy-gtsam libpcl-dev ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
  ros-jazzy-tf2-eigen ros-jazzy-tf2-sensor-msgs ros-jazzy-message-filters ros-jazzy-sensor-msgs-py \
  ros-jazzy-joint-state-publisher ros-jazzy-ament-cmake-auto ros-jazzy-rosidl-default-generators \
  libnanoflann-dev libtbb-dev libopenmpi-dev libyaml-cpp-dev libapr1-dev
# Livox-SDK2：从 github.com/Livox-SDK/Livox-SDK2 源码编译安装（GCC 13 需 -DCMAKE_CXX_FLAGS='-include cstdint'）

cd ~/yang_nav
# --base-paths src：reference/ 下有同名包，必须排除；限制并行避免 16 GB 内存下 OOM
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
| `lidar` | `dual` | `single` / `dual` |
| `mode` | `mapping` | `mapping` / `localization` |
| `prior_map` | 空 | 先验地图目录（`localization` 时必填） |
| `rviz` | `false` | 后端 RViz |

配置：`bringup/config/sim_lio.yaml`（ZLIO）、`bringup/config/sim_backend.yaml`（继承 `backend/config/backend.yaml`）。

与真车配置不同的地方：

| 项 | 仿真值 | 原因 |
| --- | --- | --- |
| `initialization.minimum_directional_support` | 5（真车 15） | 出生点补给区静止视野几何单一，15 时初始化报最弱方向支撑不足 |
| `body_from_lidar.translation` | 0 | 仿真 IMU 与雷达同原点 |
| `imu_acceleration_scale` | 1.0 | gz IMU 单位 m/s²（Livox 为 g） |
| 逐点时间 | 帧头时间 | gpu_lidar 一帧同一时刻渲染 |

LIO 的里程计系叫 `lio_odom`（与 Gazebo 真值 `odom` 区分）。仿真 TF 为 `map → lio_odom → base_link → URDF 树`：
`lio_odom → base_link` 由 `lidar_fusion` 的 `base_tf` 用 `/odometry` 与 URDF 外参算出，ZLIO 自身 TF 关闭，
`imu` 以恒等静态 TF 挂在 `mid360_link` 下。Gazebo 须以默认的 `ground_truth_tf:=false` 启动。

### 仿真实测（2026-10-27，RTX 4060 Ti，RTF 1.0）

出生点南侧约 21 s 路线（平移、转向、侧移、停车），ZLIO 位姿对比 Gazebo 真值：

| 模式 | 合并点云 | 位移 | 最大位置误差 | 结束误差 | 最大航向误差 |
| --- | --- | --- | --- | --- | --- |
| single | 10 Hz | 1.13 m | 7.3 cm | 1.0 cm | 0.43° |
| dual | 10 Hz | 1.08 m | 1.9 cm | 0.4 cm | 0.16° |

只是短路线、各一次，说明链路工作正常，不代表全场长时间精度。

## 真车

```bash
ros2 launch bringup real.launch.py lidar:=single
ros2 launch bringup real.launch.py lidar:=dual driver_l2:=true l2_launch:=/abs/path/to/unilidar_sdk2/launch/xxx.launch.py
ros2 launch bringup real.launch.py mode:=localization prior_map:=/abs/map
```

- MID-360：`livox_ros_driver2`，`xfer_format=0`（PointCloud2），网络配置在 `livox_ros_driver2/config/MID360_config.json`。
- L2：宇树 `unilidar_sdk2`（需另行安装）。
- IMU：MID-360 内置 IMU，`imu_acceleration_scale=9.81`，`body_from_lidar` 用 `frontend/config/lio.yaml` 的 Livox 出厂值。
- TF：本文件启动 `robot_state_publisher`（URDF），驱动的 `frame_id` 用恒等静态 TF 挂到 `mid360_link` / `l2_link`。
  云台 `yaw_joint` 若由底盘通信节点发布 `/joint_states`，设 `zero_joint_states:=false`。
- 后端：`bringup/config/real_backend.yaml`。接 UWB 时在这里打开 `uwb.enabled` 并按
  `backend/docs/parameters.md` 配置。

**以下默认值未在真车验证（本机无硬件），上车先核对**，改启动参数即可，不用改代码：

| 参数 | 默认 | 核对方法 |
| --- | --- | --- |
| `mid360_topic` / `mid360_frame` / `imu_topic` | `/livox/lidar` / `livox_frame` / `/livox/imu` | `ros2 topic list` |
| `mid360_time_field` / `_scale` / `_absolute` | `timestamp` / `1e-9` / `true` | `ros2 topic echo --once /livox/lidar --field fields`，看 `timestamp` 是绝对时间还是帧内偏移 |
| `l2_topic` / `l2_frame` | `/unilidar/cloud` / `unilidar_lidar` | 同上 |
| `l2_time_field` / `_scale` / `_absolute` | `time` / `1.0` / `false` | 同上 |

逐点时间写错会让 ZLIO 去畸变出错甚至发散，务必确认。

## 双雷达外参标定

```bash
ros2 run lidar_calibration calibrate --ros-args -p use_sim_time:=true   # 真车去掉 use_sim_time
```

流程：同步帧对 → 体素降采样 → PCL GICP（初值取 URDF）→ 质量门 → 多帧中值剔除 + 加权平均。
质量门：只收静止帧（IMU 判定）；每帧须与已接受帧相距 ≥0.3 m 或 ≥20°（`/odom`）；重叠区法向信息矩阵
最小特征值 ≥0.05（几何退化检测）；点到面 RMSE ≤0.05 m。

采集方法：把车停到**墙角、台阶旁等结构丰富处**，换位置和朝向，每处静止 2 s 以上，收满 `frames`（默认 6，至少 3）。
只看得到地面的位置会被判为 `degenerate_geometry` 拒收，这是故意的：单位置标定曾得出平移错 0.72 m 但帧间高度一致的结果。

输出 `/tmp/lidar_extrinsic.yaml`，含 L2→MID-360 变换和可直接填回 URDF 的 `l2_joint` origin。
仿真实测（3 帧）：平移误差 2.8 mm，旋转误差 0.078°。

## 已知问题

- 后端偶尔打印 `invalid ZLIO observation frame/time/point layout`（一次运行 0–2 条），原因未查，未见对精度的影响。
- 仿真出生点附近结构少，标定接受率低（实测约 5%）。
- 真车路径只验证了编译和 launch 展开。

## 上游文档

地图制作（保存、清理、CloudCompare 精修、生成先验地图包）沿用上游流程，见
`reference/z-localization/README.md`；参数说明见 `frontend/docs/parameters.md`、`backend/docs/parameters.md`，
设计说明见 `reference/z-localization/docs/design.md`。
