# Foxglove 定位可视化

导入本目录的 `backend.foxglove.json`，连接 Foxglove WebSocket `ws://192.168.1.154:8765`。
这是布局文件，使用 Foxglove 的 **布局 / Layout → 导入 / Import**，不是“打开数据文件”。
建议窗口至少 1600 × 900；深色应用主题与布局中的深蓝灰背景更协调。

## 默认布局

```text
┌────────────────────────────── 74% ──────────────────────────┬────── 26% ──────┐
│ 三维总览 / 俯视轨迹 / 配准诊断                              │ 定位状态        │
│                                                            │ status          │
│ 固定 map，默认斜俯视                                       │ pose_trusted    │
│ 灰蓝地图 + 青色实时点云 + 绿/橙轨迹                         ├─────────────────┤
│                                                            │ 里程计 / 告警   │
├────────────────────────────────┬───────────────────────────┤                 │
│ 位置 X / Y / Z（m）             │ 车体 vx / vy（m/s）       │                 │
│ 最近 30 秒                     │ 最近 30 秒                │                 │
└────────────────────────────────┴───────────────────────────┴─────────────────┘
```

地图与图表的高度为 75% / 25%。右上状态一直可见，右下切换完整里程计与 WARN 以上日志。
速度来自定位估计，不代表轮速反馈；定位不可信时后端可能将速度清零。

| 内容 | 话题 | 样式 |
| --- | --- | --- |
| 先验地图 | `/backend_node/prior_map` | 半透明灰蓝，小点 |
| 运行地图 | `/backend_node/runtime_map` | 半透明蓝灰，小点 |
| 实时点云 | `/localization/lio_world_cloud` | 亮青色，较大点，无历史累积 |
| 优化轨迹 | `/backend_node/optimized_path` | 绿色，线宽 0.055 m |
| 里程计轨迹 | `/backend_node/odom_path` | 橙色，线宽 0.025 m |
| 当前位姿 | `/backend_node/localization_pose` | 金色箭头、淡金色协方差 |
| 配准目标 | `/backend_node/debug/target` | 诊断页：半透明蓝色 |
| 对齐源点云 | `/backend_node/debug/aligned_source` | 诊断页：粉色 |

`nav_msgs/Path` 使用 `type: line`、`lineWidth` 和 `gradient` 配色。
关键帧坐标轴和传感器坐标轴默认隐藏，可按需在 3D 设置中打开。
已有的额外话题条目会保留。先验定位与建图发布的地图不同，两种地图都已配置。

## 视图与排查

- **三维总览**：日常使用；固定 `map`，便于比较地图和轨迹。
- **俯视轨迹**：正交俯视；隐藏实时点云，适合看轨迹漂移、回环和 XY 偏移。
- **配准诊断**：跟随 `base_link` 的位置并保持地图方向；主地图与轨迹隐藏，只比较源和目标点云。

当前场地地图约为 15.4 × 21.7 m，初始视点位于 `map` 中约 `(0.7, -1.05)`。
换到其他地图后，可在 3D 面板调整视点并保存布局。

配准诊断需要后端参数 `visualization.publish_debug_registration: true`。
布局不会开启该发布器；若它关闭，对应诊断话题没有数据是正常现象。
如果定位是 `SUSPECT` 或 `LOST` 且 `output.world_cloud.publish_untrusted: false`，
实时点云也可能暂停。先看右上角 `status` 和 `pose_trusted`。

ROS TF 链应为 `map → lio_odom → base_link`。
本布局仅选择已存在的输出，不更改定位参数。
常规观察使用鼠标移动视角；需要发导航目标时使用导航布局，见
`src/navigation/nav_bringup/rviz/README.md`。
