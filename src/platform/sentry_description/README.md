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

- `mid360_link` 是 Livox 点云数据系，`mid360_imu_link` 与其同轴；不能直接把 CAD 网格局部轴当作数据轴。
  `yaw_joint=0` 时 `base_link ← mid360_link` 为 **先 yaw 180°，再绕局部 X 轴 roll −15°**，
  即 `Rz(π) · Rx(−π/12)`（URDF RPY 为 `[-15°, 0°, 180°]`）。2026-11-23
  实测 IMU 重力方向约 `[0.012, -0.263, 0.956] g`，前端 roll 约 −15.4°；原 CAD
  关节链却给出 +105°，导致输出车体系额外侧翻约 −120°。2026-11-24 根据实物安装方向补全 yaw 180°：
  `mid360_joint` 的 pitch 从 −90° 改为 +90°；visual/collision 局部 RPY 补偿为
  `[90°, 0°, 180°]`，保留网格的装配位置。重力只能验证倾角，不能验证 yaw，
  回归检查同时约束雷达 +X 指向车尾和车体位姿换算。平移沿用 CAD 值；该值是
  当前安装的名义轴约定修复，不是完整的六自由度外参精标定。更换安装方式须重新标定。
- 网格路径是相对路径 `meshes/*.stl`。仿真侧由 `sentry_gazebo` 换成
  `model://sentry_description/meshes/...`；RViz 的 RobotModel 不解析相对路径，要在 RViz 里
  显示网格需改成 `package://sentry_description/meshes/...`。
- 网格单位是毫米，`scale` 已设为 `0.001`。
- `yaw_joint` 的轴在 `yaw_link` 系里是 `+y`，经 joint origin 旋转后在 `base_link` 里是竖直的 `−z`（保留原 CAD 关节正方向）。

## 真车用法

`bringup/launch/real.launch.py` 调用 `gimbal_description.py` 从上述同一份 URDF 提取刚性云台子树：

```
gimbal_nav -> yaw_link -> mid360_link -> mid360_imu_link
                      -> l2_link
```

`gimbal_nav` 为前/左/上参考，原点同原底盘中心（同轴云台转轴下方 5 mm）。
下位机的 X/Y 按云台方向解释，因此关节反馈未接入时仍可使用雷达定位云台；不能把 `yaw_joint=0` 当作物理底盘朝向。
真车不发布默认零关节状态，不包含未观测的底盘、车轮子树。仿真继续使用完整 URDF 与真实关节反馈。
生成器直接复用传感器安装外参；名义偏角 0 时，传感器到 gimbal_nav 等于原 yaw_joint=0 时到 base_link 的变换。

`gimbal_yaw_offset` 是 MCU +X 相对 CAD 云台零方向的实测安装偏角（rad，CCW 为正）。它不是关节角，
不会随底盘相对云台转动而更新；默认 0，尚未进行速度轴向精标定。重力不能标定该平面偏角。
生成器不会自动读 `/chassis/joint_states`，接入真实关节反馈后普通导航仍需保持云台速度参考。
