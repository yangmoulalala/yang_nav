# lidar_calibration

副雷达（Unitree L2）到主雷达（Livox MID-360）的外参标定，GICP 核心移植自 navi_minco_bit `dual_lidar_calibration`。
输入为任意 PointCloud2（不绑定 Livox），只用静止帧，不做逐点去畸变。初值与输出都经 tf2（URDF），
结果直接给出可填回 URDF 的关节 origin。本包不保存任何外参数值。

| 节点 | 可执行文件 | 作用 |
| --- | --- | --- |
| `calibrate` | `calibrate` | 采集多位姿静止帧对 → GICP → 质量门 → 多帧聚合 → 写出结果文件后退出 |

## 流程

同步帧对（ApproximateTime）→ 各自按 tf2 变换到 `primary_frame` / `secondary_frame`、剔除近距点、体素降采样
→ PCL GICP（初值见 `initial_guess_from_tf`）→ 质量门 → 满 `frames` 帧后平移逐轴中值剔除离群（>0.1 m 或 >0.05 rad），
剩余帧按 overlap/rmse 加权平均平移与四元数。一致帧少于 3 时报 `calibration failed` 并退出，不写文件。

## 采集方法

把车停到**墙角、台阶旁等结构丰富处**，换位置和朝向（间隔 ≥ `min_pose_translation` 或 ≥ `min_pose_rotation`），
每处静止 2 s 以上，直到收满 `frames` 帧。日志中每帧都会打印 `frame N/M accepted` 或 `frame rejected: <原因>`。

单个位置的点云往往只约束部分方向（地面 + 一面墙），GICP 会沿未约束方向滑走，而同一位置的多帧彼此一致，
离散度检查拦不住：单位置标定曾得出平移错 0.72 m 但帧间高度一致的结果。因此要求多位姿，并拒收几何退化的帧。

## 质量门

按顺序检查，任一不满足即丢弃该帧：

| 检查 | 参数 | 拒收原因 |
| --- | --- | --- |
| 静止：最后一次 IMU 角速度或里程计线速度超阈值后已过 0.5 s；未收到里程计前不采帧 | `stationary_gyro_threshold`、`stationary_speed_threshold` | （静默跳过） |
| 与上次配准间隔 | `min_frame_period` | （静默跳过） |
| 位姿多样性：与所有已接受帧的里程计位姿相距足够远 | `require_distinct_poses`、`min_pose_translation`、`min_pose_rotation` | （静默跳过；未收到里程计时告警 `waiting for odometry`） |
| 降采样后点数 | `min_inliers` | `too_few_points` |
| GICP 收敛 | | `not_converged` |
| 内点数 | `min_inliers` | `insufficient_inliers` |
| 重叠率（内点 / 副雷达点数） | `min_overlap_ratio` | `insufficient_overlap` |
| 几何退化：重叠区法向信息矩阵最小特征值 | `min_normal_support` | `degenerate_geometry` |
| 点到面 RMSE | `max_rmse` | `rmse_too_large` |

## 接口

话题均为节点内固定的相对名，由 launch 重映射到实际话题。

| 话题 | 方向 | 类型 | QoS | 说明 |
| --- | --- | --- | --- | --- |
| `primary/points` | 订阅 | `sensor_msgs/PointCloud2` | reliable，深度 5 | 主雷达点云，任意 frame |
| `secondary/points` | 订阅 | `sensor_msgs/PointCloud2` | reliable，深度 5 | 副雷达点云，任意 frame，与主雷达近似时间同步 |
| `imu` | 订阅 | `sensor_msgs/Imu` | SensorData | 主雷达 IMU，用角速度判定静止 |
| `odometry` | 订阅（始终） | `nav_msgs/Odometry` | 深度 10 | ZLIO `/odometry`：线速度用于静止判定；x、y、yaw 用于位姿多样性（`require_distinct_poses`） |

### TF 需求

- 主雷达点云 `frame_id` → `primary_frame`，副雷达点云 `frame_id` → `secondary_frame`（真车 MID-360 驱动直接用 `mid360_link`，L2 由 `real.launch.py` 的 `alias_l2` 静态 TF 提供）。
- `primary_frame` ← `secondary_frame`：仅 `initial_guess_from_tf=true` 时用作 GICP 初值（当前 URDF 外参）。
- `secondary_parent_frame` ← `primary_frame`：输出时把标定结果换算为 URDF 关节 origin。

均取最新 TF（`TimePointZero`）；TF 不全时告警 `waiting for TF` 并跳过该帧。云台 `yaw_joint` 需在采集期间保持不动。

## 参数

全部必填、节点内无默认值；缺失、类型错误或未知参数都会使启动失败（`calibrate failed: ...`，退出码 1）。
示例见 [config/example.yaml](config/example.yaml)（仿真取值）。

| 参数 | 类型 | 说明 |
| --- | --- | --- |
| `primary_frame` | string | 主雷达坐标系（配准目标） |
| `secondary_frame` | string | 副雷达坐标系（待标定） |
| `secondary_parent_frame` | string | URDF 中副雷达关节的父系，输出 `urdf_joint` 相对此系 |
| `initial_guess_from_tf` | bool | true：初值取 tf2（URDF）；false：取 `initial_xyz_rpy` |
| `initial_xyz_rpy` | double[6] | 手动初值 `[x, y, z, roll, pitch, yaw]`（m, rad，secondary → primary）；始终须为 6 个数，`initial_guess_from_tf=true` 时不使用 |
| `frames` | int, ≥3 | 需要接受的帧数（真车 6；仿真场地只有约 4 处几何足够丰富，取 4） |
| `min_frame_period` | double, s | 两次配准的最小间隔 |
| `sync_tolerance` | double, s | 双雷达同步最大时间差 |
| `stationary_gyro_threshold` | double, rad/s | 静止判定角速度阈值 |
| `stationary_speed_threshold` | double, m/s | 静止判定线速度阈值（里程计 twist，`odometry` 话题始终订阅） |
| `min_range` | double, m | 近距离剔除阈值 |
| `voxel_size` | double, m | 体素降采样边长 |
| `max_correspondence_distance` | double, m | GICP 与内点判定的最大对应距离 |
| `min_overlap_ratio` | double | 重叠率下限 |
| `min_inliers` | int, ≥1 | 点数与内点数下限 |
| `max_rmse` | double, m | 点到面 RMSE 上限 |
| `min_normal_support` | double | 法向信息矩阵最小特征值下限（三正交平面均匀时约 0.33）。当前 0.09：仿真中 support≈0.05 的帧沿最弱方向滑动 10–17 cm |
| `normal_k` | int, ≥3 | 法向估计近邻数 |
| `require_distinct_poses` | bool | 是否订阅 `odometry` 并要求位姿多样性（false 不推荐） |
| `min_pose_translation` | double, m | 位姿多样性平移间隔 |
| `min_pose_rotation` | double, rad | 位姿多样性航向间隔 |
| `output_file` | string | 结果文件路径；**只由 launch 给出**，不写入参数文件 |

## 输出

`output_file`（默认 `/tmp/lidar_extrinsic.yaml`）：

```yaml
# l2_link -> mid360_link (p_primary = T * p_secondary)
primary_frame: mid360_link
secondary_frame: l2_link
xyz: [x, y, z]                 # m
rpy: [roll, pitch, yaw]        # rad，URDF 固定轴约定
used_frames: 5
translation_stddev: ...        # m
rotation_stddev_rad: ...
min_normal_support: ...
# URDF <origin> of the joint yaw_link -> l2_link
urdf_joint:
  parent: yaw_link
  xyz: [...]
  rpy: [...]
```

`urdf_joint` 可直接填回 `sentry_description/urdf/sentry_robot.urdf` 中 `l2_joint` 的 `<origin>`。
每个被接受的帧在日志中打印 `t=[...]`（该帧平移）与 `weak=[...]`（主雷达系中最弱约束方向），用于判断平移离散来自哪个方向。

仿真实测（2026-10-31，4 帧）：平移误差约 1 cm（标准差 1.4 cm），旋转误差 < 1°。

## 启动

```bash
# 单独启动（默认 config/example.yaml）
ros2 launch lidar_calibration calibrate.launch.py use_sim_time:=true
ros2 launch lidar_calibration calibrate.launch.py params_file:=/abs/lidar_calibration.yaml \
    primary_topic:=/livox/lidar secondary_topic:=/unilidar/cloud imu_topic:=/livox/imu output_file:=/abs/out.yaml
```

launch 参数：`params_file`、`use_sim_time`（默认 false）、`output_file`（默认 `/tmp/lidar_extrinsic.yaml`）、
`primary_topic` / `secondary_topic` / `imu_topic` / `odometry_topic`（默认为仿真话题）。
完整系统用 `ros2 launch bringup calibration.launch.py platform:=sim|real`，它传入
`bringup/config/<platform>/lidar_calibration.yaml` 与平台话题，见 `src/z-localization/README.md`。
