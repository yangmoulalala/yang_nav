# ZLIO · 激光惯性里程计

![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros)
![LiDAR–IMU](https://img.shields.io/badge/Sensors-LiDAR%20%2B%20IMU-287A6B)
![Odometry](https://img.shields.io/badge/Localization-Odometry-536A8A)

ZLIO 是 [Z-Localization](../README.md) 的激光惯性前端，ROS 包名为 `frontend`。
它接收带逐点时间的 LiDAR 点云和 IMU 数据，输出局部里程计、去畸变点云，以及随 IMU 更新的高频位姿预测。

[项目结构](#项目结构) · [环境与依赖](#环境与依赖) · [快速开始](#快速开始) · [参数配置](#参数配置) · [对外接口](#对外接口) · [致谢](#致谢)

## 项目结构

```text
src/z-localization/frontend/
├── README.md
├── docs/parameters.md           参数列表与调参建议
├── include/zlio/                核心库公开接口
├── src/                         初始化、估计、地图、采样与输入处理
├── apps/                        ROS 节点、参数读取、消息转换与可视化
├── config/example.yaml          独立运行的示例 ROS 参数文件（MID-360）
├── launch/frontend.launch.py    里程计（可选可视化）启动入口
├── rviz/frontend.rviz           RViz 配置
├── cmake/                       CMake 导出配置
├── CMakeLists.txt
└── package.xml
```

全部节点参数来自标准 ROS 参数文件，且均为必填：缺失、类型错误或未知参数都会使节点启动失败。
[config/example.yaml](config/example.yaml) 仅用于独立运行；完整系统使用 `bringup/config/<platform>/frontend.yaml`。

## 环境与依赖

| 类别 | 依赖 |
| --- | --- |
| 系统环境 | Ubuntu 24.04、ROS 2 Jazzy |
| 编译与构建 | C++ ≥ 17、CMake ≥ 3.16、ament_cmake |
| C++ 库 | Eigen ≥ 3.3、nanoflann、Threads、OpenMP C++ |
| ROS 包 | rclcpp、sensor_msgs、nav_msgs、geometry_msgs、std_srvs、tf2、tf2_ros、launch、launch_ros、rviz2 |
| 工作区包 | localization_interfaces、livox_ros_driver2 |
| 驱动依赖 | Livox-SDK2、PCL、APR |

## 快速开始

启动前必须有外部节点发布 `frames.imu ← frames.lidar` 的 TF（雷达在 IMU 系中的位姿），节点在构造时查询一次，
未就绪时持续等待并告警。示例（MID-360 出厂外参）：

```bash
ros2 run tf2_ros static_transform_publisher --x -0.011 --y -0.02329 --z 0.04412 \
    --frame-id imu --child-frame-id livox_frame

# 里程计（默认使用 share/frontend/config/example.yaml）
ros2 launch frontend frontend.launch.py

# 自定义参数文件、输入话题，并启动可视化
ros2 launch frontend frontend.launch.py params_file:=/absolute/path/to/frontend.yaml \
    lidar_topic:=/livox/lidar imu_topic:=/livox/imu visualization:=true

# 直接运行节点
ros2 run frontend zlio_node --ros-args --params-file /absolute/path/to/frontend.yaml \
    -r lidar:=/livox/lidar -r imu:=/livox/imu
```

Launch 参数仅有 `params_file`、`use_sim_time`、`lidar_topic`、`imu_topic`（仅用于重映射）和 `visualization`。

## 参数配置

完整参数列表与调参建议见 [参数文档](docs/parameters.md)。

## 对外接口

### 话题

下表列出默认话题名。节点固定订阅相对话题 `lidar`、`imu`，由 launch 的 `lidar_topic`、`imu_topic` 或 `-r` 重映射；
相对名称也支持 ROS 命名空间。
QoS 中的数字表示 keep-last 队列深度；未标注 transient local 的话题使用 volatile，不向新订阅者补发历史消息。

| 话题 | 类型 | 方向 / 节点 | QoS | 内容 |
| --- | --- | --- | --- | --- |
| `lidar`（launch 默认重映射到 `/livox/lidar`） | `livox_ros_driver2/msg/CustomMsg` 或 `sensor_msgs/msg/PointCloud2` | `/zlio` 订阅 | best effort，200 | LiDAR 输入；消息类型由 `input.type` 决定 |
| `imu`（launch 默认重映射到 `/livox/imu`） | `sensor_msgs/msg/Imu` | `/zlio` 订阅 | best effort，4000 | IMU 角速度与加速度 |
| `/odometry` | `nav_msgs/msg/Odometry` | `/zlio` 发布；RViz 订阅 | reliable，200 | 随 IMU 更新的预测位姿、速度与协方差 |
| `/lidar_odometry` | `nav_msgs/msg/Odometry` | `/zlio` 发布 | reliable，100 | 每个激光观测结束时的状态估计 |
| `/frontend/frame` | `localization_interfaces/msg/FrontendFrame` | `/zlio` 发布；可视化节点订阅 | 发布 reliable，64；订阅 best effort，5 | 同一次观测的位姿、去畸变点云与更新状态 |
| `/frontend/odometry` | `localization_interfaces/msg/FrontendOdometry` | `/zlio` 发布 | reliable，200 | 带进程身份和代次的高频预测 |
| `/frontend/session` | `localization_interfaces/msg/FrontendSession` | `/zlio` 发布；可视化节点订阅 | reliable，1，transient local | 进程与重置代次、输入中断和处理失败通知 |
| `/tf` | `tf2_msgs/msg/TFMessage` | `/zlio` 发布并在启动时订阅；RViz 订阅 | tf2 动态广播器／监听器默认 QoS | 发布 `frames.world → frames.base`（示例 `frames.base=imu`，即 `odom → imu`；bringup 为 `lio_odom → base_link`），由 `output.publish_tf` 控制；组合 bringup 将它设为 `false`，改由后端同一消息发布两条动态边；`frames.base` 与 `frames.imu` 不同时还按里程计时间戳查询 `frames.imu ← frames.base` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | `/zlio` 启动时订阅；RViz 订阅 | tf2 静态监听器默认 QoS | 外部节点发布的静态坐标变换，须包含 `frames.imu ← frames.lidar` |
| `/frontend/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 可视化节点发布；RViz 按显示配置订阅 | 发布 reliable，1 | 当前观测的 odom 系点云 |
| `/frontend/cloud_map` | `sensor_msgs/msg/PointCloud2` | 可视化节点发布；RViz 订阅 | 发布 reliable，1，transient local | 用于显示的累计点云，保留量可配置 |
| `/frontend/path` | `nav_msgs/msg/Path` | 可视化节点发布；RViz 订阅 | 发布 reliable，1，transient local | 激光观测轨迹，保留长度可配置 |

ROS 通用话题：

| 话题 | 类型 | 方向 / 条件 |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | 使用仿真时间时订阅 |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | 节点参数事件发布；ROS 参数机制按需订阅 |
| `/rosout` | `rcl_interfaces/msg/Log` | 节点日志发布 |

### 服务

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `/localization_start` | `std_srvs/srv/SetBool` | 传入 `data=true` 后开始接收输入；重复调用不会重置，`data=false` 不支持 |
| `/stop` | `std_srvs/srv/Trigger` | 停止接收输入，处理完缓存中可用的数据后返回；节点进程继续运行 |
| `/zlio/reset_internal` | `localization_interfaces/srv/ResetFrontend` | 由后端协调器调用；按请求者和事务号去重，同一请求不会重复重置 |

暖重置将位置和航向归零，保留速度、倾角与 IMU 零偏，并将状态及协方差换算到新的里程计坐标系。
暖重置要求前端持续运行且 IMU 历史完整；尚未初始化、已经停止、处理失败或历史缺失时，会改为冷启动。
两种重置都保留输入的启用状态。请求与响应字段见 [ResetFrontend.srv](../localization_interfaces/srv/ResetFrontend.srv)。

相邻 IMU 采样的间隔上限由 `input.maximum_imu_interval` 设置（示例 `0.1 s`）。间隔过长、时间回退，或等待 IMU 时点云缓存耗尽，都会按输入中断处理；
重复时间戳则直接丢弃。尚未输出初始估计时，中断后会清空缓存并在同一代次重新初始化。
已经开始运行时，`session.coordinated_reset=false` 的独立前端自动换代冷启动；设置为 `true` 后，前端通过
`FrontendSession.reset_required` 通知后端，待后端冻结输出后再协调重置。统一 bringup 默认开启此模式。

主动调用 `/stop`、无效输入或程序错误不会触发上述自动恢复。普通处理失败通过 `FrontendSession.failed`
通知后端冻结输出，前端等待显式重置。冷启动无法补回缺测期间的运动，因此自动冷恢复后需要重新通过先验地图
确定全局位姿；已有轨迹、且无绝对定位来源的纯建图模式会保持 `LOST`。

### 消息与坐标约定

| 对象 | 约定 |
| --- | --- |
| IMU 输入 | 角速度为 rad/s，加速度乘换算倍率后为 m/s²，均在 IMU 系表达；时间须递增，不使用消息中的 orientation 和协方差 |
| PointCloud2 输入 | 小端，必须包含 `x/y/z` 与配置的逐点时间字段；支持 FLOAT32、FLOAT64、UINT32、INT32、UINT16、UINT8 标量 |
| Livox 输入 | `point_num` 须与点数组长度一致；逐点时间为 `timebase` 或 `header.stamp` 加纳秒 `offset_time`；保留 `tag & 0x30` 为 `0x00` 或 `0x10` 的回波 |
| 坐标系 | `frames.world` 为里程计父坐标系，`frames.imu` 为输出子坐标系（IMU 系），`frames.lidar` 为 LiDAR 坐标系，`frames.base` 为 TF 输出子坐标系 |
| 外参 | 启动时从 TF 查询 `frames.imu ← frames.lidar`（LiDAR → IMU），之后不再更新；变换须在启动时存在 |
| 输入 `frame_id` | 不触发 TF 转换；与 `frames.lidar` 不一致时只告警一次，点仍按 `frames.lidar` 解释 |
| 里程计 | pose 在 `frames.world`，twist 在 `frames.imu`；示例分别为 `odom`、`imu` |
| TF 输出 | `frames.world → frames.base` = `(frames.world → frames.imu)·(frames.imu ← frames.base)`；后者按每条里程计的时间戳从 TF 查询（跟随云台等可动关节），查不到时取最新变换，最新变换与时间戳相差超过 `output.base_extrinsic_tolerance` 则沿用上次有效值并告警，尚无有效值时不发 TF。`frames.base = frames.imu` 时直接发布里程计位姿。IMU 系在 URDF 中已有父帧时，`frames.base` 须填树根（如 `base_link`） |
| 导出点云 | float32 XYZ，统一到该次观测结束时的 IMU 坐标系；不保留原始强度、线号和时间字段 |
| 可视化点云 | 使用同一观测中的位姿转换到 `frames.world` |

`FrontendFrame` 的字段定义见 [FrontendFrame.msg](../localization_interfaces/msg/FrontendFrame.msg)：

| 字段 | 含义 |
| --- | --- |
| `instance_id`、`epoch`、`sequence` | 进程标识、重置代次，以及该代次内递增的观测序号 |
| `begin` | 本次新增点云的时间区间起点，终点为 `odometry.header.stamp` |
| `odometry`、`cloud` | 同一时刻的状态估计与去畸变点云；两者时间戳相同，点云位于里程计子坐标系 |
| `matches` | 最后一轮迭代中的有效点面匹配数 |
| `lidar_updated` | 本次是否接受了激光校正；为 `false` 时仍可能发布 IMU 传播结果 |

`FrontendOdometry` 包含 `instance_id`、`epoch` 和 `odometry`。`FrontendSession` 还通过
`reset_required`、`failed` 分别报告输入中断和处理失败。接收方应根据进程标识与重置代次区分新旧数据。

## 致谢

感谢以下开源项目与研究工作提供的算法思路和工程参考。

| 项目 / 研究工作 | 参考与启发 |
| --- | --- |
| [FAST-LIO / FAST-LIO2](https://github.com/hku-mars/FAST_LIO) | 高效激光惯性紧耦合、迭代滤波与直接点云配准的基础思路 |
| [Voxel-SLAM](https://github.com/hku-mars/Voxel-SLAM) | 多帧激光惯性初始化、重去畸变与联合优化 |
| [BALM / BALM 2.0](https://github.com/hku-mars/BALM) | 多帧平面统计与点云位姿联合优化 |
| [PV-LIO](https://github.com/HViktorTsoi/PV-LIO) / [VoxelMap](https://github.com/hku-mars/VoxelMap) | 概率体素平面地图、测量与平面不确定度建模，以及点面关联 |
| [Faster-LIO](https://github.com/gaoxiang12/faster-lio) | iVox 稀疏增量体素索引与高效邻域检索的思路 |
| [Eq-LIO](https://github.com/taoxiom/Eq-LIO) | 等变滤波与状态建模，尤其是导航状态和 IMU 零偏的耦合表达 |
| [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry) | 连续时间运动轨迹构建与点云去畸变 |
| [Point-LIO](https://github.com/hku-mars/Point-LIO) | 按逐点采样时间组织 LiDAR–IMU 数据，逐点或按点组进行状态传播与更新 |
| [AS-LIO](https://arxiv.org/abs/2408.11426) | 基于空间重叠调整滑动窗口与更新步长的思路，为早期时间自适应设计提供参考 |
| [Coco-LIC](https://arxiv.org/abs/2309.09808) / [ASPIRE-VINS](https://arxiv.org/abs/2608.12840) | 根据运动变化调整时间分辨率的思路，为当前观测间隔调度提供参考 |
| [GenZ-LIO](https://arxiv.org/abs/2603.16273) | 场景尺度感知的空间自适应采样、点数反馈调节与双分辨率体素采样 |
| [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO) | 体素内保留原始代表点、按中心距离进行降采样的思路 |

同时感谢 [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) 提供驱动与消息接口，
以及 ROS 2、Eigen、nanoflann 和 OpenMP 生态的维护者。

## 维护者

mozihe · [zhujunheng2005@gmail.com](mailto:zhujunheng2005@gmail.com)
