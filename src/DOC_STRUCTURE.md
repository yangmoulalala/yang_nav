# 文件结构与作用（sentry_description + sentry_gazebo）

两个包，职责只有一条线：

```
sentry_description/urdf/sentry_robot.urdf   ── 决定机器人是什么（形状 + 关节）
arena_spec 里的模型位姿                      ── 决定它在 world 里的位置和朝向
```

除此之外没有任何帧换算、没有辅助 link、没有 mesh 旋转补偿。

---

## 1. `src/sentry_description` — 机器人描述（真车共用，不依赖 Gazebo）

| 文件 | 作用 |
| --- | --- |
| `urdf/sentry_robot.urdf` | **唯一几何来源**。8 个 link / 7 个 joint，**z-up**（轮子分布在 `x–y` 平面、`yaw_joint` 绕竖直轴转、`z` 是"上"），可以直接进 Gazebo，不需要任何整体旋转。 |
| `meshes/*.stl` | 8 个 STL，和 URDF 里的 `<mesh filename>` 一一对应。 |
| `CMakeLists.txt` | 把 `urdf/` 和 `meshes/` 装到 `share/sentry_description/`。 |
| `package.xml` | 纯资源包，无 `gz`/`ros_gz` 依赖。 |

URDF 里各 link 的层级（改动前先对照）：

```
base_link
├── wheel_front_left   (xyz -0.208 -0.208 0.003, axis 0 0 -1)
├── wheel_front_right  (xyz  0.208 -0.208 0.003, axis 0 0 -1)
├── wheel_rear_left    (xyz -0.208  0.208 0.003, axis 0 0 -1)
├── wheel_rear_right   (xyz  0.208  0.208 0.003, axis 0 0 -1)
└── yaw_link           (xyz 0 0 0.005, axis 0 1 0)
    ├── mid360_link    (xyz  0.1636 -0.1316  0.0558)
    └── l2_link        (xyz -0.1761 -0.1575 -0.0426)
```

`base_link` 的 visual/collision 自带 `rpy="1.570796 0 0"`（STL 本身是另一套轴向），
仿真里原样使用，不做补偿。

---

## 2. `src/sentry_gazebo` — 仿真包（world + 传感器 + 桥接）

### 2.1 Python 模块 `sentry_gazebo/`

| 文件 | 作用 |
| --- | --- |
| `arena_spec.py` | **唯一数据源**。机器人位姿（`ROBOT_SPAWN_XY_YAW` / `ROBOT_Z` / `ROBOT_MODEL_RPY`）、碰撞体尺寸、雷达参数与挂载表 `ROBOT_MOUNTS`、赛场盒体 `ARENA_BOXES`、话题名。常量都在这里改。 |
| `sdf_builder.py` | **URDF + arena_spec → SDF world**。`Urdf` 类解析 URDF 并做 FK（`pose_of(link)` 直接给 root 系下的 xyz/rpy）；`robot_model_xml()` 写机器人（link 位姿与 joint origin/axis 全部来自 URDF）；`gpu_lidar_xml` 写传感器；`arena_model_xml()` 写赛场；`world_xml()` 拼整个 world。 |
| `teleop.py` | 包内**唯一的 ROS 节点**：键盘遥控，20 Hz 发布 `geometry_msgs/Twist` 到 `/cmd_vel`。终端下按住即走；stdin 不是终端时每行一个按键、保持 `hold` 秒。速度上限取自 `arena_spec`。 |

### 2.2 入口 / 生成

| 文件 | 作用 |
| --- | --- |
| `launch/sim.launch.py` | 主入口。参数 `gui` / `rviz` / `generated_dir` / `ground_truth_tf`；调 `prepare_world.generate()` 生成 world，然后起 gz、`ros_gz_bridge`、`robot_state_publisher`、`joint_state_publisher`（关节零位）、每个传感器一个 `static_transform_publisher`（帧别名），可选 RViz2。 |
| `launch/prepare_world.py` | **每次启动重新生成** `rmul_2027.sdf`、`env.sh` 到 `/tmp/sentry_gazebo_generated`，并建 `model://sentry_description/meshes` 软链。**生成物不落仓库**。 |
| `launch/launch_utils.py` | 路径解析（`generated_dir()`、`urdf_path()`）和 `gz_environment()`（`GZ_SIM_RESOURCE_PATH`）。 |

### 2.3 gz system（全部内置，写在 `sdf_builder.py` 里）

| system | 作用 |
| --- | --- |
| `MecanumDrive` | 订 gz `/cmd_vel`，驱动四个全向轮关节（轮槽位、轴向、`fdir1` 由 `omni_wheel()` 从 URDF 算出）。 |
| `OdometryPublisher` | 真值 `/odom` 与 `odom → base_link`（`/tf_odom`，默认不进 ROS TF，见 `ground_truth_tf`）。 |
| `Sensors` + `Imu` | 两个 `gpu_lidar` 与两个 `imu` 传感器。 |

不再有自写 gz 插件（原 `plugins/sentry_velocity_control.py` 已删除：它直接写速度/位姿，IMU 运动中读数错误）。

### 2.4 配置 / 界面 / 入口

| 文件 | 作用 |
| --- | --- |
| `config/bridge.yaml` | ros_gz 桥映射：clock、odom、`/cmd_vel`（ROS→gz）、两路点云、两路 IMU。`/tf_odom`→`/tf` 只在 `ground_truth_tf:=true` 时由 launch 另起桥。 |
| `rviz/sentry.rviz` | RViz 布局（Fixed Frame = `odom`）。 |
| `scripts/teleop` | `ros2 run sentry_gazebo teleop` 的入口。因为 colcon 把 ament_python 的 console_scripts 装到 `<prefix>/bin`（`ros2 run` 不查那里），所以按 `data_files` 装进 `lib/sentry_gazebo/`。 |

### 2.5 资源 `models/`

| 目录 | 作用 |
| --- | --- |
| `models/rmul_2027/meshes/step_textured.dae` | 赛场官方 STEP 细分网格，只作 visual 引用。 |
| `models/rmul_2027/materials/textures/official_top.png` | 上面那份网格的贴图。 |

### 2.6 `setup.py` / `package.xml` / `resource/`

`setup.py` 按数据文件安装 `launch/config/rviz/models`，并把 `scripts/teleop`
装到 `lib/sentry_gazebo/`——那是 `ros2 run` 的查找路径。**没有用 `entry_points`**：
colcon 会把 ament_python 的 console_scripts 放到 `<prefix>/bin`，`ros2 run` 找不到。

---

## 3. 运行时的数据流

```
prepare_world.py
  └─ sdf_builder.build(urdf)  ──> rmul_2027.sdf
                                      │
                              gz sim -r -v 3
                                      │
        ┌──────────────────┬──────────┴──────────┬────────────────────┐
        │                  │                     │                    │
   gpu_lidar ×2         imu ×2          OdometryPublisher       MecanumDrive
   (点云)            (雷达内置 IMU)      (/odom, /tf_odom)       (读 /cmd_vel)
        │                  │                     │                    │
        └──────────────────┴──────────┬──────────┴────────────────────┘
                                      │
                              ros_gz_bridge
                                      │
                  /lidar_*/points  /lidar_*/imu  /odom  /tf
                                      │
     joint_state_publisher (零位) ─> robot_state_publisher (URDF 正解)
                                      │
                      base_link -> yaw_link -> mid360_link / l2_link
```

`/cmd_vel` 反向走：`teleop`（或你自己的节点）→ ROS `/cmd_vel` → bridge → gz
`/cmd_vel` → `MecanumDrive`。

定位（`src/z-localization`，入口 `ros2 launch bringup sim.launch.py lidar:=single|dual`）订阅
`/lidar_*/points`、`/lidar_mid360/imu` 与 TF，详见 `src/z-localization/README.md`。

---

## 4. 决定机器人位姿的三个常量（`arena_spec.py`）

```python
ROBOT_SPAWN_XY_YAW = (1.25, 6.75, 0.0)   # world 里的 x, y, yaw
ROBOT_Z            = 0.062               # 轮子最低点落在 z=0
ROBOT_MODEL_RPY    = (0.0, 0.0, 0.0)     # URDF 已是 z-up；只为兼容未来导出保留
```

SDF 里对应的写法（`sdf_builder.robot_model_xml`）：

```xml
<pose> x y ROBOT_Z   ROBOT_MODEL_RPY[0] ROBOT_MODEL_RPY[1]  yaw </pose>
```

`ROBOT_MODEL_RPY` 同时喂给 gz `OdometryPublisher` 的 `<rpy_offset>`（取逆旋转，
由 `sdf_builder.inverse_rpy()` 算），保证 `odom → base_link` 的 base 帧永远竖直。
当前该值为 0，所以 `<rpy_offset>` 也是 0；若以后又导出 y-up 模型，只改这一个常量即可。

---

## 5. 自检

自定义的健康检查节点已删除，用标准工具即可：

```bash
ros2 topic hz /lidar_mid360/points
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link mid360_link
ros2 topic echo --once /joint_states
```

RTF 看 Gazebo GUI 的 World stats，或 `gz topic -e -t /clock`。
