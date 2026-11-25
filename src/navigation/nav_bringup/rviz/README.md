# Foxglove 导航布局（推荐）

导入本目录的 `navigation.foxglove.json`，连接 Foxglove WebSocket `ws://192.168.1.154:8765`。
使用 **布局 / Layout → 导入 / Import**，不是“打开数据文件”。
建议窗口至少 1600 × 900，日常使用深色主题。

```text
┌─────────────────────────── 70% ─────────────────────────────┬────── 30% ──────┐
│ 全局导航：map 固定，正交俯视                               │ 车旁避障        │
│                                                            │ 跟随车辆位置    │
│ 可通行地图 / 绿色整形路径 / 区域标记 / 金色车辆轮廓          │ 局部代价地图    │
│                                                            │ 红色障碍点      │
│                                                            ├─────────────────┤
│                                                            │ 定位状态        │
├────────────────────────────────┬───────────────────────────┼─────────────────┤
│ vx / vy 指令与定位估计（m/s）   │ wz 指令与估计（rad/s）    │ 模式与阶段      │
│ 最近 30 秒                     │ 最近 30 秒                │ 避障/限速/告警  │
└────────────────────────────────┴───────────────────────────┴─────────────────┘
```

左侧地图/曲线的高度为 73% / 27%，曲线左右宽度为 60% / 40%。
右侧上半屏显示约 8 m 局部代价地图，下方固定显示定位状态与导航详情标签页。
局部视图只跟随 `gimbal_nav` 的位置，地图方向稳定，便于与全局视图对照。

| 面板或图层 | 话题 | 默认状态 |
| --- | --- | --- |
| 全局可通行地图 | `/nav_map` | 显示，灰度地图 |
| 控制器实际输入路径 | `/plan_smoothed` | 显示，绿色粗线 |
| 原始规划路径 | `/plan` | 隐藏，蓝色细线，可按需对比 |
| 全局代价地图 | `/global_costmap/costmap` | 隐藏，需要检查膨胀代价时打开 |
| 局部代价地图 | `/local_costmap/costmap` | 仅局部视图显示 |
| 隧道、坡道区域 | `/nav_zones/markers` | 全局显示，保留消息内颜色 |
| 区域跟踪状态标记 | `/nav_zones/tracker` | 显示，保留消息内颜色 |
| 车辆轮廓 | `/local_costmap/published_footprint` | 显示，金色 |
| 障碍点 | `/nav/obstacles` | 局部显示，红色 |
| 最近目标 | `/navigation/goal_marker` | 全局显示，粉色箭头、圆点和坐标；保留最近目标 |
| 定位状态 | `/backend_node/localization_state.data` | 一直显示 |
| 模式/区域阶段 | `/nav_mode.data`、`/nav_zone_state.data` | 默认详情页 |
| 碰撞监控/限速 | `/collision_monitor_state`、`/speed_limit` | 详情标签页 |
| 告警 | `/rosout` | WARN 及以上；可查看 cmd_gate 的 holding 原因 |

底部曲线将最终 `/cmd_vel` 与 `/localization/odometry.twist.twist` 对照。
这里的“估计”是定位估计，不能代替底盘轮速反馈。
需要分析门控时，可在 Plot 设置中启用已预设但默认关闭的 `/cmd_vel_nav` 曲线；
`/cmd_vel_nav` 非零而最终 `/cmd_vel` 为零时，结合定位状态、碰撞监控和告警判断原因。
线速度、角速度使用独立坐标轴，避免混用 m/s 和 rad/s。

## 使用

1. 启动传感器、定位、Foxglove Bridge 和导航。
2. 连接 `ws://192.168.1.154:8765`，导入布局，确认 `map → lio_odom → gimbal_nav` 可用。
3. 检查定位状态 `pose_trusted=true`、地图与机器人位置正确后，再发送导航目标。
4. 使用 3D 面板的 **Publish pose / 发布位姿** 工具，在地图上拖出目标位置和朝向。
   布局已将其 `poseTopic` 设为 Nav2 的 `/goal_pose`，不是 ROS 1 的 `/move_base_simple/goal`。
   布局默认工具仍为发布点，需主动选择发布位姿后才会发送目标。

本项目也可通过 `tools/start_navigation.sh goal_x:=... goal_y:=...` 或
`ros2 run nav_bringup send_goal.py X Y --yaw YAW` 发送目标，实际接口为 `/navigate_to_pose`。
`/goal_pose` 是 Nav2 接收交互式目标的主题入口，Action 发送的目标不会自动发布到这里。
导航启动时会运行 `goal_visualizer.py`：接收 `/goal_pose` 的交互目标，以及
`send_goal.py` 在 Action 接受后发布的 `/navigation/goal_accepted`，在
`/navigation/goal_marker` 输出独立显示标记。箭头抬高 0.2 m，避免被地图遮住；
显示输出不会发送导航指令。标记使用 transient-local 并每秒刷新，浏览器后连接也能看到。

**Last goal 是最近收到的目标，包含已到达、取消或中止的历史目标，不表示任务仍在执行。**
重启导航后，收到新目标才会显示；其他自定义 Action 客户端如需显示目标，
应在目标被接受后将原始 `PoseStamped` 发布到 `/navigation/goal_accepted`（可靠、transient-local）。
升级后须重新导入布局；只调整 JSON 不会为旧导航进程启动目标显示节点。
仿真与真车可共用布局；Foxglove Bridge 应与 ROS 节点处于相同的 ROS domain。

不要在主视图同时打开先验大点云、实时点云和两张代价地图；需要查点云时逐项打开即可。
有些话题只有相应节点启动或开始导航后才发布；未启动导航时局部图、路径或阶段暂时无数据。
布局不会启动节点，也不会改变导航参数。
