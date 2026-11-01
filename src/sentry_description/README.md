# sentry_description

RMUL 2027 哨兵的**机器人描述**：只有 URDF 和网格，没有任何 Gazebo / 仿真依赖。
真车和仿真共用这一份，真车上跑 `robot_state_publisher` 时不需要装 Gazebo。

```
sentry_description/
├── urdf/sentry_robot.urdf
└── meshes/*.stl          # base_link, yaw_link, mid360_link, l2_link, 4 轮
```

## 坐标系与关节

```
base_link
├── wheel_{front,rear}_{left,right}   4 × continuous
└── yaw_link                          continuous（云台偏航，joint origin rpy = (-π/2, 0, -π/2)）
    ├── mid360_link                   fixed  xyz 0.164 -0.132 0.056
    │   └── mid360_imu_link           fixed  xyz 0.011 0.02329 -0.04412（Livox 出厂 IMU 外参，无网格）
    └── l2_link                       fixed  xyz -0.176 -0.157 -0.043
```

- 网格路径是相对路径 `meshes/*.stl`。仿真侧由 `sentry_gazebo` 换成
  `model://sentry_description/meshes/...`；RViz 的 RobotModel 不解析相对路径，要在 RViz 里
  显示网格需改成 `package://sentry_description/meshes/...`。
- 网格单位是毫米，`scale` 已设为 `0.001`。
- `yaw_joint` 的轴在 `yaw_link` 系里是 `+y`，经 joint origin 旋转后在 `base_link` 里是竖直的 `+z`。

## 真车用法

```bash
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat $(ros2 pkg prefix sentry_description)/share/sentry_description/urdf/sentry_robot.urdf)"
```

`bringup/launch/real.launch.py` 已经这样启动了 `robot_state_publisher`，一般不用手动运行。

## 注意

- URDF 里**没有 `<inertial>`**：几何来自 CAD 导出，质量/惯量由仿真侧
  （`sentry_gazebo/arena_spec.py` 的 `CHASSIS_MASS` 等）单独给出。
  真车上如果只用 `robot_state_publisher` 这不影响；如果要用
  `ros2_control` 做动力学，需要补 `<inertial>`。
- 仿真专用的东西（Gazebo `gpu_lidar` 参数、赛场、world、bridge）**不在这个包**，
  见 `sentry_gazebo`。
