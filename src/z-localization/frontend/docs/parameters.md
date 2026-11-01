# 参数与调参建议

[返回 README](../README.md) · [参数列表](#参数列表) · [调参建议](#调参建议)

## 参数列表

### 参数来源与规则

节点参数的唯一来源是标准 ROS 参数文件（`--params-file` 或 launch 的 `params_file`）；代码中没有默认值，
launch 也不覆盖任何参数（`use_sim_time` 除外）。规则：

- **全部必填**：缺失参数会报 `missing required parameter '<name>'` 并退出。
- **拒绝未知参数**：未被读取的参数（如拼写错误）会报 `unknown parameters for node ...` 并退出；`use_sim_time`、
  `start_type_description_service` 与 `qos_overrides.*` 除外。
- **类型严格**：类型不符时报错；浮点参数可写整数（如 `1000`），浮点数组可写整数数组。
- 示例文件 [config/example.yaml](../config/example.yaml) 含 `zlio` 与 `frontend_visualization` 两节，仅用于独立运行；
  完整系统使用 `bringup/config/<platform>/frontend.yaml`。下表“示例值”均取自该文件。
- 参数仅在启动时读取，修改后需重启。

### 算法参数（节点 `/zlio`）

共 70 项。完整参数名为“小节名.键名”，如 `sensor.gravity`、`initialization.registration.voxel_size`。
所有数值须有限；“正数”指大于零，“非负数”指不小于零，数组长度固定。
旧版 `sensor.body_from_lidar.*` 已删除：LiDAR → IMU 外参在启动时从 TF `frames.imu ← frames.lidar` 读取，见[坐标系](#坐标系与外参)。

#### `sensor` · 传感器（3 项）

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `gravity` | 浮点，m/s² | `9.81` | 重力加速度大小；必填，范围 `[9.6, 10.0]` |
| `minimum_range` | 浮点，m | `0.5` | 丢弃距离 LiDAR 小于此值的回波；非负且 `< maximum_range` |
| `maximum_range` | 浮点，m | `100.0` | 丢弃距离 LiDAR 大于此值的回波；须大于 `minimum_range` |

#### `initialization` · 收集与采样（10 项）

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `accel_bias` | 3 元数组，m/s² | `[0, 0, 0]` | 加速度计三轴零偏的初始估计，也是初始化优化的先验值 |
| `accel_bias_std` | 3 元数组，m/s² | `[0.3, 0.3, 0.3]` | 上述零偏初值的各轴标准差；越小越信任初值，各项须为正数 |
| `window_frames` | 整数，帧 | `20` | 每次初始化联合优化使用的扫描帧数；须 `≥2` |
| `seed_imu_samples` | 整数，样本 | `31` | 用于初步估计重力方向的 IMU 样本数；收集足够样本后开始积累扫描，须 `>0` |
| `maximum_refinements` | 整数，轮 | `15` | 交替执行去畸变和窗口优化的最多轮数；须 `≥4` |
| `sample_size` | 浮点，m | `0.1` | 初始化扫描降采样的体素边长；正数 |
| `fallback_sample_size` | 浮点，m | `0.05` | 采样点数不足时改用的更细体素边长；正数且 `≤ sample_size` |
| `minimum_sampled_points` | 整数，点 | `1000` | 常规采样后少于此点数，就改用 `fallback_sample_size` 重新采样；须 `>0` |
| `minimum_directional_support` | 浮点，无量纲 | `15` | 初始化最弱方向所需的平面约束量；越大越难在几何单一的场景中启动，正数 |
| `relative_cost_tolerance` | 2 元数组，无量纲 | `[0.05, 0.01]` | 粗、细两阶段的相对代价变化阈值；低于阈值视为收敛，各项在 `(0,1)` |

#### `initialization.registration` · 启动粗配准（13 项）

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `voxel_size` | 浮点，m | `0.5` | 粗配准地图的降采样体素边长，每格取质心；正数 |
| `neighbors` | 整数，点 | `5` | 为一个点拟合局部平面时查找的近邻点数；须 `≥3` |
| `minimum_map_points` | 整数，点 | `100` | 地图至少达到此点数才进行粗配准；须 `≥ neighbors` |
| `maximum_iterations` | 整数，次 | `4` | 每帧粗配准的最大迭代次数；须 `≥1` |
| `plane_residual_tolerance` | 浮点，无量纲 | `0.1` | 平面方程 `A·n=-1` 的拟合残差上限；越小拟面越严格，正数，不是点面距离 |
| `prior_covariance_scale` | 浮点，无量纲 | `1000` | 粗配准时先验协方差的放大倍数；越大越减弱先验约束，正数 |
| `rotation_tolerance` | 浮点，rad | `0.00017452006980802793` | 旋转更新量小于此值时满足旋转收敛条件；正数，默认约 `0.01°` |
| `position_tolerance` | 浮点，m | `0.00015` | 位置更新量小于此值时满足位置收敛条件；正数 |
| `gyro_variance` | 浮点，rad²/s² | `0.1` | 粗配准 IMU 传播使用的角速度采样噪声方差；正数 |
| `accel_variance` | 浮点，m²/s⁴ | `1.0` | 粗配准 IMU 传播使用的加速度采样噪声方差；正数 |
| `bias_variance` | 浮点，各偏置变化率的平方单位 | `0.0001` | 粗配准中两类 IMU 零偏变化率的方差；正数，陀螺项单位 rad²/s⁴，加速度项 m²/s⁶ |
| `initial_state_variance` | 浮点，各状态的平方单位 | `0.0001` | 粗启动时旋转、位置、速度的各轴先验方差；也用于暖重置后的位置和航向，正数 |
| `initial_bias_variance` | 浮点，各偏置的平方单位 | `0.00001` | 粗启动时陀螺与加速度零偏的各轴先验方差；正数 |

#### `initialization.optimization` · 启动联合优化（14 项）

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `voxel_size` | 浮点，m | `1.0` | 将启动点云划分为平面拟合区域的根体素边长；正数 |
| `maximum_depth` | 整数，层 | `2` | 平面拟合不通过时最多细分多少层；范围 `[0,8]`，`0` 不细分 |
| `minimum_points` | 整数，点 | `5` | 体素内点数严格大于此值才尝试拟合平面；须 `≥3` |
| `minimum_planes` | 整数，面 | `10` | 初始化优化至少需要的有效平面数；须 `≥1` |
| `plane_variance` | 2 元数组，m² | `[0.02, 0.0025]` | 点到拟合平面的散布方差上限，依次为粗、细阶段；越小要求越平整，正数且细阶段不大于粗阶段 |
| `plane_eigenvalue_ratio` | 浮点，无量纲 | `0.25` | 点分布最小 / 最大特征值的上限，用于判断是否足够平整；范围 `(0,1)` |
| `plane_middle_ratio` | 浮点，无量纲 | `0.12` | 点分布最小 / 中间特征值的上限，用于排除线状点集；范围 `(0,1)` |
| `imu_weight` | 浮点，有效权重 | `0.0001` | IMU 残差相对点云几何残差的权重；越大越信任 IMU，正数 |
| `thread_count` | 整数，线程 | `5` | 平面优化可用的线程数，包含主线程；须 `≥1` |
| `maximum_iterations` | 整数，步 | `3` | 每轮联合优化最多执行的迭代步数；须 `≥1` |
| `gyro_variance` | 浮点，rad²/s² | `0.01` | 初始化 IMU 预积分使用的角速度采样噪声方差；正数 |
| `accel_variance` | 浮点，m²/s⁴ | `1.0` | 初始化 IMU 预积分使用的加速度采样噪声方差；正数 |
| `gyro_bias_psd` | 浮点，rad²/s³ | `0.0001` | 初始化中陀螺零偏随时间变化的噪声强度；越大允许变化越快，正数 |
| `accel_bias_psd` | 浮点，m²/s⁵ | `0.0001` | 初始化中加速度零偏随时间变化的噪声强度；越大允许变化越快，正数 |

#### `initialization.uncertainty` · 初始化不确定度（2 项）

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `allow_configured_fallback` | 布尔 | `true` | 几何与联合信息检查通过、但协方差计算失败时，是否改用 `state_variance`；关闭则重新初始化 |
| `state_variance` | 5 元数组，各量的平方单位 | `[1, 1, 1, 0.0001, 0.001]` | 备用初始方差，依次对应位置、旋转、速度、陀螺零偏、加速度零偏；每项用于三个轴，均须为正数 |

`state_variance` 的五项单位依次为 m²、rad²、m²/s²、rad²/s²、m²/s⁴。

#### `filter` · 在线滤波（7 项）

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `association_threads` | 整数，线程 | `2` | 点面匹配使用的线程数；`1` 单线程，`2` 允许主线程与一个工作线程并行 |
| `maximum_iterations` | 整数，次 | `5` | 每次激光更新的最大迭代次数；须 `≥1` |
| `convergence_tolerance` | 浮点，各导航分量的 SI 单位 | `0.001` | 各状态分量的更新量小于此值时满足收敛条件；使用各自 SI 单位，正数 |
| `gyro_variance` | 浮点，rad²/s² | `0.25` | 在线 IMU 传播使用的角速度采样噪声方差；正数 |
| `accel_variance` | 浮点，m²/s⁴ | `0.5` | 在线 IMU 传播使用的加速度采样噪声方差；正数 |
| `gyro_bias_psd` | 浮点，rad²/s³ | `0.00000133` | 在线估计中陀螺零偏随时间变化的噪声强度；越大允许变化越快，正数 |
| `accel_bias_psd` | 浮点，m²/s⁵ | `0.0000215` | 在线估计中加速度零偏随时间变化的噪声强度；越大允许变化越快，正数 |

#### `map` · 在线地图（4 项）

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `voxel_size` | 浮点，m | `0.5` | 在线平面地图的根体素边长；越小划分越细，正数 |
| `maximum_roots` | 整数，体素根 | `1000000` | 地图根体素数量上限；继续新增时若超限则报错停止，不自动淘汰旧体素，须 `>0` |
| `range_sigma` | 浮点，m | `0.02` | LiDAR 沿射线方向的测距标准差；用于计算点的不确定度，正数 |
| `angle_sigma_degrees` | 浮点，度 | `0.05` | LiDAR 射线方向的角度标准差；用于计算点的不确定度，正数，单位为度 |

#### `sampling` · 自适应采样（14 项）

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `initial_size` | 浮点，m | `0.25` | 开始跟踪或暖重置后的采样体素边长；正数 |
| `minimum_size` | 浮点，m | `0.02` | 自适应采样允许的最小体素边长；正数 |
| `maximum_size` | 浮点，m | `1.0` | 自适应采样允许的最大体素边长；正数 |
| `history_size` | 整数，观测 | `5` | 平滑场景距离时保留的历史观测数；每次取点距中位数，再对历史求平均，须 `>0` |
| `minimum_points` | 浮点，点 | `1000` | 小尺度场景下的目标采样点数下限；正数，非实际点数保证 |
| `maximum_points` | 浮点，点 | `4000` | 大尺度场景下的目标采样点数上限；正数，非实际点数硬限制 |
| `scale_threshold` | 浮点，m | `30.0` | 平滑后的点距中位数达到此值时，目标点数取 `maximum_points`；正数 |
| `exponent` | 浮点，无量纲 | `2.0` | 控制目标点数随场景距离增大的速度；越大越早接近点数上限，须 `>1` |
| `proportional_scale` | 浮点，无量纲 | `0.1` | 点数误差相对目标点数的调节尺度；越小，比例增益越快增大，正数 |
| `derivative_scale` | 浮点，无量纲 | `0.2` | 相邻观测点数误差变化的调节尺度；越小，微分增益越快增大，正数 |
| `proportional_minimum` | 浮点，m/点 | `0.000001` | 按当前点数误差调整体素边长的最小增益；正数 |
| `proportional_maximum` | 浮点，m/点 | `0.0001` | 按当前点数误差调整体素边长的最大增益；正数 |
| `derivative_minimum` | 浮点，m·s/点 | `0.000000001` | 按点数误差变化率调整体素边长的最小增益；正数 |
| `derivative_maximum` | 浮点，m·s/点 | `0.0000001` | 按点数误差变化率调整体素边长的最大增益；正数 |

#### `window` · 观测调度（3 项）

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `maximum_interval` | 浮点，s | `0.12` | 激光更新目标间隔的上限，也是点云采样历史的最长保留时长；正数 |
| `minimum_interval` | 浮点，s | `0.03` | 激光更新目标间隔的下限；至少 `1 ns`，且不超过 `maximum_interval` 的一半 |
| `normalized_error` | 浮点，无量纲 | `2.0` | 允许的运动外推误差，以点面噪声归一化；越小更新越频繁，正数 |

### 坐标系与外参

#### `frames` · 坐标系（4 项）

| 键 | 类型 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `frames.world` | 字符串 | `odom` | 里程计父坐标系；非空，改名不改变估计结果 |
| `frames.imu` | 字符串 | `imu` | IMU 坐标系，即里程计子坐标系与导出点云坐标系；非空 |
| `frames.lidar` | 字符串 | `livox_frame` | LiDAR 坐标系；非空，用于查询外参 |
| `frames.base` | 字符串 | `imu` | TF 输出的子坐标系；非空。与 `frames.imu` 相同时直接发布 `frames.world → frames.imu`；不同时发布 `frames.world → frames.base` = `(frames.world → frames.imu)·(frames.imu ← frames.base)`，后者按里程计时间戳从 TF 查询（云台转动也跟随）。IMU 系在 URDF 中已有父帧时须填树根（bringup 为 `base_link`），否则 IMU 系会有两个父帧 |

LiDAR → IMU 外参不再作为参数（旧版 `sensor.body_from_lidar.*` 已删除）。节点构造时从 TF 查询
`target=frames.imu`、`source=frames.lidar` 的最新变换，即雷达坐标系在 IMU 系中的位姿；未就绪时每 5 s 告警一次并持续等待，
取得后以 INFO 打印并校验（四元数模长与 1 相差不超过 `0.01`，随后归一化），运行中不再更新。该变换须由外部节点（如
`static_transform_publisher` 或 robot_state_publisher）发布。输入消息的 `frame_id` 与 `frames.lidar` 不一致时只告警一次，不做转换。

### 传输参数（节点 `/zlio`）

#### `input` · 传感器输入（11 项）

输入话题固定为相对名 `lidar`、`imu`，由 launch 的 `lidar_topic`、`imu_topic` 或 `-r` 重映射，不再作为参数。

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `input.type` | 字符串 | `livox` | LiDAR 消息格式：`livox` 对应 `CustomMsg`，`pointcloud2` 对应 `PointCloud2` |
| `input.point_time_field` | 字符串 | `offset_time` | PointCloud2 中每个点的采样时间字段名；Livox 输入不使用 |
| `input.point_time_scale` | 浮点 | `1.0e-9` | PointCloud2 时间字段换算为秒的倍率；纳秒 `1e-9`、微秒 `1e-6`、毫秒 `1e-3`、秒 `1.0`，须 `>0` |
| `input.point_time_absolute` | 布尔 | `false` | PointCloud2 时间字段是否为绝对时间；关闭时按相对 `header.stamp` 的偏移读取 |
| `input.livox_use_timebase` | 布尔 | `true` | Livox 逐点时间以 `timebase` 为起点；关闭则用 `header.stamp`，两者均加纳秒 `offset_time` |
| `input.imu_acceleration_scale` | 浮点 | `9.81` | 加速度单位换算倍率；g 取 `9.81`（MID-360），m/s² 取 `1.0`，须 `>0` |
| `input.imu_history_seconds` | 浮点，s | `30.0` | IMU 历史的最长保留时长，也限制输入时间差；至少 `1 ns` |
| `input.maximum_imu_interval` | 浮点，s | `0.1` | 相邻 IMU 采样允许的最大间隔；超出后按输入中断处理，至少 `1 ns` |
| `input.maximum_imu_samples` | 整数 | `65536` | IMU 历史最多保留的样本数；须 `≥2` |
| `input.maximum_buffered_points` | 整数 | `1000000` | 点数容量上限，分别作用于单包、ROS 待处理队列和核心点云缓存；须 `>0` |
| `input.maximum_queue_events` | 整数 | `8192` | ROS 待处理队列的消息数上限；一条 IMU 或一次扫描各计一个事件，须 `>0` |

#### `output` · 输出（3 项）

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `output.resolution` | 浮点，m | `0.10` | 导出去畸变点云的采样体素边长；允许 `0` 或 `≥1e-4 m`，`0` 输出空点云 |
| `output.publish_tf` | 布尔 | `true` | 是否发布 `frames.world → frames.base` 的高频预测 TF（随 `/odometry` 发布） |
| `output.base_extrinsic_tolerance` | 浮点，s | `0.2` | 按里程计时间戳查不到 `frames.imu ← frames.base` 时，最新变换与该时间戳允许的最大差；超过则沿用上次有效值并限频告警，尚无有效值时不发 TF。有限且 `≥0`；`frames.base = frames.imu` 时不使用 |

#### `session` · 会话与重置（3 项）

| 键 | 类型 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `session.wait_for_start` | 布尔 | `false` | 是否等待 `/localization_start` 的 `data=true` 请求后再接收输入；等待期间丢弃数据 |
| `session.reset_mode` | 字符串 | `warm` | `warm` 保留速度、倾角和 IMU 零偏，重设位置与航向；条件不满足时冷启动。`cold` 始终重新初始化 |
| `session.coordinated_reset` | 布尔 | `false` | 运行中输入中断后，是否交给后端协调重置；关闭则自行冷启动，统一 bringup 开启 |

`use_sim_time` 由 launch 设置（默认 `false`），不属于参数文件必填项。

### 可视化节点 `/frontend_visualization`（4 项）

| 键 | 类型 / 单位 | 示例值 | 说明 |
| --- | --- | --- | --- |
| `path_max_poses` | 整数 | `5000` | 显示轨迹最多保留的位姿数；超限删除最早位姿，须 `>0` |
| `map_voxel_size` | 浮点，m | `0.15` | 累计显示点云的体素边长；越小细节越多，正数 |
| `map_max_voxels` | 整数 | `200000` | 累计显示点云最多保留的体素数；超限删除最久未观测的体素，须 `>0` |
| `map_publish_period` | 浮点，s | `0.2` | 累计显示点云的最小发布间隔，按观测时间计算；范围 `[0,60]`，`0` 每个有效观测都发布 |

### Launch 参数（`frontend.launch.py`）

Launch 只负责参数文件选择、话题重映射与仿真时间，不覆盖其他节点参数。

| 参数 | 类型 | 默认值 | 含义 |
| --- | --- | --- | --- |
| `params_file` | 路径 | `<frontend share>/config/example.yaml` | ROS 参数文件，须含 `zlio`（及启用可视化时的 `frontend_visualization`）一节 |
| `use_sim_time` | 布尔 | `false` | 是否使用 `/clock` 提供的仿真时间 |
| `lidar_topic` | 字符串 | `/livox/lidar` | 重映射到节点输入 `lidar`；消息格式须与 `input.type` 一致 |
| `imu_topic` | 字符串 | `/livox/imu` | 重映射到节点输入 `imu` |
| `visualization` | 布尔 | `false` | 是否同时启动 `zlio_visualizer` 与 RViz（`frontend.rviz`） |

## 调参建议

先核对传感器配置，再根据场景调整采样和地图参数。其余参数通常可以保持示例值。

### 传感器参数

| 参数 | 范围 / 取值 | 作用 | 设置依据 |
| --- | --- | --- | --- |
| TF `frames.imu ← frames.lidar` | 单位四元数、有限三轴平移；无通用数值区间 | LiDAR 坐标到 IMU 坐标的旋转与平移，平移单位 m | 由外部 TF 发布厂商外参或离线标定结果，注意变换方向（父 `frames.imu`、子 `frames.lidar`）；启动日志打印所用外参 |
| `sensor.minimum_range`、`maximum_range` | `0 ≤ min < max`，落在传感器可靠测距区间 | 参与估计的原始回波距离范围，m | 根据可靠量程、自车遮挡和实际回波分布设置 |
| `input.point_time_field`、`point_time_scale`、`point_time_absolute`、`livox_use_timebase` | 秒／毫秒／微秒／纳秒倍率分别为 `1 / 1e-3 / 1e-6 / 1e-9`；绝对／相对约定取自数据源 | 每个回波的真实采样时刻与时间原点 | 核对驱动的逐点时间字段、单位和起点；见 [PointCloud2 解码](../apps/ros.cpp)、[Livox 解码](../apps/node.cpp) |
| `input.imu_acceleration_scale` | m/s² 输入取 `1.0`；g 输入通常取 `9.81` | 将原始加速度换算为 m/s² 的倍率 | 按设备单位填写，用静止时加速度模长接近 `9.81 m/s²` 检查；见 [输入处理](../apps/node.cpp) |
| `map.range_sigma` | `>0` m；来自传感器参数 | 径向测距标准差 | 按传感器测距标准差填写，注意不要将误差上限直接当作标准差 |
| `map.angle_sigma_degrees` | `>0` °；来自传感器参数 | 射线切向角度标准差 | 按传感器测角标准差填写，单位为度 |

### 场景参数

| 参数 | 范围 / 当前值 | 作用 | 设置建议 |
| --- | --- | --- | --- |
| `map.voxel_size` | `>0` m；当前 `0.5 m` | 在线平面地图的根体素边长 | 近距多表面场景可减小，远距稀疏场景可增大；比较有效匹配数与轨迹稳定性，见 [地图实现](../src/map.cpp) |
| `sampling.minimum_points`、`maximum_points` | 均 `>0`；当前目标为 `1000–4000` 点 | 自适应采样的目标代表点数量 | 几何覆盖不足时增加目标点数，处理延迟过大时降低；见 [自适应采样](../src/sampling.cpp) |
| `output.resolution` | `0` 或 `≥0.0001 m`；当前 `0.10 m` | 导出点云的体素分辨率，`0` 关闭导出 | 下游需要更多细节时减小，需要降低点云数据量时增大；设为 `0` 会关闭点云导出，见 [导出实现](../src/preparation.hpp) |

每次调整一组参数，用同一段数据比较匹配连续性、轨迹误差和处理耗时。
