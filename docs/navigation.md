# 导航框架

全向底盘的导航由 Nav2、区域路径插件、地形障碍过滤及最终命令门组成。本文说明当前实现；完成情况与未解决项集中在 [导航进度](nav_plan.md)。

## 数据与命令链

```text
定位：z-localization → TF / /localization/odometry / localization_state
地形：nav_map + speed_mask + terrain.msgpack + zones.yaml

StartAwarePlanner（Smac2D + 占用起点退出段）→ ZonePathShaper → ZoneAwareController（区域外 MPPI，区域内 ZoneTracker）
                                      ↓
behavior_server ───────────────→ cmd_vel_nav
                                      ↓
velocity_smoother → cmd_vel_smoothed → collision_monitor
                                      ↓
                         cmd_vel_collision_checked → cmd_gate → /cmd_vel

/frontend/frame → terrain_obstacles → /nav/obstacles → 代价地图及 collision_monitor
```

导航直接使用定位结果，不修改定位数据；`cmd_gate` 按已有可信状态、命令/观测新鲜度和速度上限决定最终输出。

障碍点云已经位于固定 map 坐标系。碰撞监控使用最新 map→base TF（`base_shift_correction=false`），避免每条命令等待估计器尚未发布的“当前时刻”变换；观测过期、变换失败停车和完整车体 approach 检查继续启用。

## 地形输入

默认目录为 `maps/sim/terrain`。`terrain_path.py` 也接受包含该子目录的 `maps/sim`。

| 文件 | 使用方 |
|---|---|
| `nav_map.yaml` 及其图像 | 静态可通行地图 |
| `speed_mask.yaml` 及其图像 | SpeedFilter 速度上限 |
| `terrain.msgpack` | 地面相对障碍过滤；净高用于顶棚剔除 |
| `zones.yaml` | 整形器、跟踪器、恢复区域保护 |

整形器和控制器必须读取同一份区域数据，并保持区域类型及链合并阈值一致。地图和手工标注不能为让测试通过而随意覆盖。当前真实地形包净高缺失，启动成功不代表地形检查通过。

## 区域路径与控制

隧道、坡道和其他方向地形共用整形器及跟踪器。语义类型选择 tunnel/ramp 参数组，区域策略定义轴线、方向、是否居中及是否允许车身翻转。邻近且轴线兼容的区域合为一个 stretch。

- 整形器生成入口、轴向通行段和出口；保留目标位置及朝向。路径可以从区域内开始、在区域内结束或完全位于区域内。
- APPROACH 混合 MPPI 与跟踪器；ALIGN/TRAVERSE 由跟踪器控制；EXIT 逐渐交回 MPPI。区域内目标在到达前保持跟踪器控制。
- ALIGN/TRAVERSE/EXIT 期间 BT 保留当前整形路径，APPROACH 允许重规划；新目标可以打断旧路径和恢复。
- `/nav_zone_state` 采用 `traverse tunnel_3 @123.4` 形式，时间戳与控制器时钟一致；`/nav_mode` 由控制器唯一发布，值为 normal/tunnel/ramp。
- `one_way_dir` 目前选择需要整形和跟踪的方向。独立坡道反向下坡交普通规划/MPPI，不能把这个字段理解为全局规划器禁止下坡。

碰撞检查使用完整 padded convex footprint，包含足迹内部占用格。路径段和制动轨迹采用半栅格步长的运动插值检查，未知和越界视为碰撞。`collision_tolerance` 必须为零，整形核心不存在跳过起点碰撞检查的开关。必须整形的路径失败会返回 Nav2 错误，不能静默把原路径交给区域跟踪器。

起点已经与已知障碍重叠或处于起点不可用的膨胀带时，导航先找保持实际朝向的平移退出段，再从空闲端点调用 Smac。退出段最长默认 1.5 m，控制速度默认 0.15 m/s；不清除或修改共享代价图。完全嵌入较厚障碍时允许退出起初接触的同一连通障碍，但车体与障碍的重叠面积不能增加，也不能进入起初未授权的障碍区域。这个例外只覆盖连续的退出前缀；一旦退出，普通完整车体碰撞检查继续生效。未知、越界、没有可退出通路和实时碰撞监控停车仍会阻止运动。

退出段受阻同样发布 `blocked`，允许行为树从当前姿态重规划。收到新提交的整形路径后重新建立退出目标及接触许可，不沿用上一条路径的失效退出状态；目标改变也会清除旧目标的跟踪锁定。

退出段由规划器生成、整形器验证并保留，控制器在实时局部地图上重新检查，先制动已有旋转/侧向惯性，再保持朝向低速退出。控制器为整个退出过程冻结初始障碍范围，更新地图或滚动地图不会把新障碍自动加入豁免。该过程不是旋转/倒车恢复，因此不解除 ZoneRecovery 的洞内保护。

若重规划后路径只剩洞外后缀，跟踪器仍重建 EXIT 窗口，并在车尾离开洞体前保持区域控制和洞体航向。新路径没有洞体采样点不能作为整车已经离洞的依据。

车身允许 0/π 翻转时，由整形器选择物理朝向并完成足迹检查；控制器沿用路径中的朝向，不在入口再次选择翻转，避免前后不对称的车体及出口转弯使已验证路径失效。区域控制器额外检查名义路径前方、当前速度及新命令的制动轨迹，包含侧移、旋转、倒行。

## 行为树与恢复

单目标使用 `navigate_to_pose.xml`，多目标使用 `navigate_through_poses.xml`。两者都执行规划、整形、区域路径保持及 `ZoneRecovery`；launch 将真实 zones 路径注入两个树文件。

规划、整形、执行使用不同路径黑板键。`CapturePathGoal` 记录规划对应的目标，`CommitZonePath` 只在整形成功且目标仍一致时提交执行路径；`PathForGoal` 和 `AwaitGoalPath` 阻止新目标沿用旧目标的路径。整形期间旧结果到达也不能标记为新目标的结果。

`ZoneReplanPaused` 只使用最新有效 phase；恢复另由 `ZoneRecovery` 保存近期禁止运动记录，不再提供旧的 `last_phase` 端口。

恢复阶梯：清图 → 自转 → 倒车 → 放弃。以下情况禁止运动恢复，只清图并最终返回失败：

- TF 显示机器人位于受约束区域及其外扩 0.55 m 内。
- 已配置区域而机器人 TF 缺失或过期。
- 近期报告 ALIGN/TRAVERSE/EXIT，或当前失败链已经触发区域禁止运动状态。

保护在同一失败链中锁存，phase 心跳超时不会自动解除；执行中的自转/倒车也会被保护条件打断。新目标中断旧导航或恢复并重置阶梯。

`SmoothPath` 的 Nav2 通用碰撞检查关闭，由插件完成带航向的完整足迹检查；这不表示跳过碰撞验证。

## 构建和启动

在 WSL 的项目根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav_behaviors nav_bringup nav_safety nav_terrain
source install/setup.bash
```

实际导航需要定位 TF、里程计和前端观测已经可用。启动示例：

```bash
ros2 launch nav_bringup navigation.launch.py terrain_dir:=maps/sim/terrain use_sim_time:=true
```

测试优先使用 [独立 domain 的回归工具](../tools/nav_regress/README.md)。运动学 fixture 不验证真实净高、俯仰、轮胎接触、打滑或制动能力。

## 文件维护

当前架构放在本文，进度放在 `nav_plan.md`，测试方法放在工具 README。审查过程记录放在 `docs/history/`，最终验证证据放在 `reports/navigation/<日期>/`。临时日志写到显式指定的 `/tmp` 输出目录；关闭自己启动的进程后可清理。清理前需要保留的失败诊断和最终结果应先归入证据目录。

2026-11-13 代码与文档一致性复核见 [清理记录](history/nav_code_cleanup_2026-11-13.md)。
