# sentry_gazebo

Gazebo Harmonic 的 RMUL / RMUC 场地与车辆数据源。真车和仿真共用机器人 URDF、`gimbal_nav` 定位参考和 Nav2 纯平移控制，平台之间仅保留传感器数据格式、采样和场地参数差异。

## 启动

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sentry_gazebo sim.launch.py arena:=rmuc   # 终端 1；gui:=true 打开 GUI
ros2 launch bringup sim.launch.py lidar:=single      # 终端 2，建图
ros2 run sentry_gazebo teleop                        # 终端 3
```

已有与当前安装外参匹配的先验地图时，定位使用 `mode:=localization prior_map:=/abs/map`，导航使用 `bash tools/start_navigation.sh profile:=sim terrain_dir:=maps/sim/terrain`。外参改变后需重新生成仿真地图；旧地图和录包不会被自动覆盖。

| 参数 | 默认 | 用途 |
| --- | --- | --- |
| `arena` | `rmul` | `rmul` / `rmuc` 场地 |
| `gui` / `rviz` | `false` | Gazebo / RViz 窗口 |
| `team` | 空 | `red` / `blue`，空使用场地默认出生方 |
| `uwb` | `true` | 仅 RMUC 发布裁判系统 UWB |
| `gimbal_yaw_offset` | `0.0` | 与真车相同的 MCU +X 安装偏角，rad；不是关节角 |
| `chassis` | `true` | MCU 速度接口；仅传感器专项回归用 `false` 并直接控制物理底盘 |
| `generated_dir` | `$SENTRY_GAZEBO_GEN` 或 `/tmp/sentry_gazebo_generated` | 自动生成的 world 和资源环境 |

## 速度与云台

`/cmd_vel` 与真车普通接口一致：`linear.x/y` 以云台前/左方向为参考，`angular.z` 被忽略。`chassis_sim` 复用 `chassis_comm.CommandState` 的输入检查和 0.3 s 指令看门狗，以 100 Hz 把 XY 转到物理底盘方向，再通过私有 `/sim/chassis_cmd_vel` 驱动 Gazebo 内置 `MecanumDrive`。关节反馈超过 0.1 s 未更新也停车。

云台使用 Gazebo 内置 `JointPositionController` 独立驱动。仅供仿真的控制入口：

```bash
# 云台相对底盘 +1 rad，绕竖直 +Z 逆时针；普通导航不会发布此话题
ros2 topic pub --once /sim/turret_yaw std_msgs/msg/Float64 '{data: 1.0}'
```

键盘 `w/s/a/d` 前后左右平移，`q/e` 转动云台，空格停车，`+/-` 调速，`x` 退出。初始平移速度 0.25 m/s；此入口只供仿真操作。

`/chassis/big_yaw` 为约 200 Hz 的 Float32 实测仿真关节角（rad，范围 `[-π, π)`，仿真约定 CCW +Z）；`/chassis/big_yaw_online` 为 reliable + transient local 在线状态。失联时停止角度发布，不补零、不重复旧值。仿真角度约定不构成对实物 MCU 零位与正方向的标定。当前不模拟过洞动作和 MCU 动作完成反馈。

## 定位与传感器

ROS 的 TF 链与真车一致：

```text
map → lio_odom → gimbal_nav → yaw_link → mid360_link → mid360_imu_link
                                    └→ l2_link
```

`robot_state_publisher` 调用共用 `gimbal_description.py` 生成刚性云台子树。定位只用雷达/IMU，不接收底盘真值或默认零关节构造的底盘 TF。物理关节状态 `/sim/joint_states` 仅供模拟器接口及测试使用，不进入定位。

安装位姿只维护于 `sentry_description/urdf/sentry_robot.urdf`。MID-360 数据轴为名义 yaw 180°、roll −15°；内置 IMU 也使用 URDF 中 `(0.011, 0.02329, -0.04412)` m 的真实杆臂。Gazebo IMU 单位是 m/s²，所以平台差异仅把加速度倍率设为 1，真车为 9.81。

| 传感器 | MID-360 | Unitree L2 |
| --- | --- | --- |
| Gazebo 类型 | `gpu_lidar` | `gpu_lidar` |
| 水平 / 垂直采样 | 720 × 40 | 160 × 40 |
| 仰角 | −7° … +52° | −48° … +48° |
| 量程 | 0.1–40 m | 0.05–30 m |
| 点云 / IMU | 10 / 200 Hz | 10 / 200 Hz |
| 点云话题 | `/lidar_mid360/points` | `/lidar_l2/points` |
| IMU 话题 | `/lidar_mid360/imu` | `/lidar_l2/imu` |

Gazebo 使用作用域传感器 frame 名；launch 为它们发布到对应数据坐标系的恒等别名，MID-360 的 IMU 别名父帧为 `mid360_imu_link`。雷达点云是均匀光栅，未命中的射线可能含 inf，不能等同于真机逐点时序。IMU 陀螺噪声标准差 0.002 rad/s，加速度噪声 0.02 m/s²。车体视觉通过 visibility mask 从雷达回波排除。

## 物理模型与 UWB

四个全向轮采用 URDF 位置和轴向，Gazebo 内置 MecanumDrive 通过轮地接触驱动。轮子使用球形碰撞和单向摩擦，底盘使用简化盒体，云台使用受控旋转关节，视觉保留 CAD 网格。轮子有垂直弹簧阻尼，避免四轮刚性接触使单个对角方向失去牵引。这些近似不代表真车速度响应已标定。

RMUC 的 `/uwb_pose` 模拟裁判系统 0x0203，1 Hz、约 0.2 s 延迟，带白噪声、慢变偏差、离群值和丢包；`/uwb_pose/team` 发布己方颜色。场地坐标为 `x_ref=14-x_world`、`y_ref=8-y_world`。天线杆臂按 `gimbal_nav` 旋转，独立云台转动时也保持一致。模型真值只在模拟器内部使用，不桥接为 ROS 定位输入。真车 UWB 仍按其原有未标定状态关闭。

## 检查

```bash
python3 tools/check_bringup_params.py
ros2 topic hz /lidar_mid360/points
ros2 topic hz /chassis/big_yaw
ros2 run tf2_ros tf2_echo map gimbal_nav  # 需运行定位
ROS_DOMAIN_ID=168 python3 tools/nav_regress/framework_accept.py --out /tmp/nav_accept
```

WSL 有多块显卡时，非交互 shell 显式设置 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`，可用 `glxinfo -B` 检查渲染器。软件渲染会降低实时率与传感器频率。不要在同一 ROS domain/Gazebo partition 启动重复实例；通过原启动终端或其进程组停止旧实例。

源码职责：`arena_spec.py` 管场地与采样，`sdf_builder.py` 生成物理场景，`prepare_world.py` 管生成资源，`chassis_sim.py` 模拟 MCU 普通接口，`uwb_sim.py` 模拟裁判系统，`bridge.yaml` 维护 Gazebo/ROS 接线。共用定位参数见 `bringup/config/common/`，平台差异见 `bringup/config/sim/`。
