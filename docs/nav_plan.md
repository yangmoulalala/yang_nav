# 导航 v2 计划（2026-11-10 修订）

目标：RMUC 2027 场地全向哨兵，平地导航、六个 0.25 m 净高隧道（车高 0.245 m）、坡道（含约 20° 堡垒坡）全部可靠通过。

## 架构

```
定位 (z-localization) ── /localization/odometry, TF map→base_link, localization_state
        │
离线地形包 (nav_terrain → maps/<场景>/terrain): nav_map, speed_mask, ground_z, clearance, zones.yaml
        │
Nav2: Smac2D ─▶ ZonePathShaper (区域整形) ─▶ ZoneAwareController (MPPI + 区域通行状态机)
        │                                         └─ 发布 /nav_mode (唯一发布者)
velocity_smoother ─▶ cmd_gate (定位可信门、看门狗、按 /nav_mode 限速) ─▶ /cmd_vel
```

设计约定：

- **区域通行通用化**：隧道和坡道共用一套"沿区域轴线、锁定航向、速度曲线、入口检查"的整形器 + 跟踪器，按 `zones.yaml` 的区域类型取参数（`tunnel.*`、`slope.*`）。
- **通行状态机**：`APPROACH → ALIGN → TRAVERSE → EXIT → 交回 MPPI`。进入 ALIGN 后由跟踪器全权负责，内层 MPPI 不再参与，其失败不导致导航失败；只在 APPROACH 段和 EXIT 之后与 MPPI 线性混合。
- **区域内不重规划**：状态机在 ALIGN/TRAVERSE/EXIT 时，行为树暂停 `ComputePathToPose`（路径过期、IsPathValid 失败都不替换当前路径）。
- **`/nav_mode` 只由 ZoneAwareController 发布**（normal / tunnel / ramp / recovery），`cmd_gate` 只读。
- **不动 MPPI、Smac2D**：区域外表现正常，问题只在交接处。

## 里程碑

| 里程碑 | 内容 | 验收 | 状态 |
|---|---|---|---|
| M0 平地导航 | Nav2 + MPPI Omni + cmd_gate + nav_regress | 10 个平地目标 | ✅ 已提交 |
| M1 地形包 | surface_studio 手工标注 → terrain_export | 地形包可被 Nav2 加载 | ✅ 已提交 |
| M3a 隧道规划检查 | plan_tunnels（不开仿真） | 12/12 路径过隧道 | ✅ 已提交 |
| M3b 区域路径整形 | ZonePathShaper | 12/12 路径整形成功，11 个单测 | ✅ 已提交 |
| **M3c 隧道通行** | ① tunnel_tracker 改状态机，ALIGN 起不再混 MPPI；② 区域内暂停重规划（BT 条件节点读控制器状态）；③ 无 ROS 运动学闭环测试（全向运动学 + 位姿/执行噪声，6 隧道 × 正反向）；④ Gazebo 验收 | 闭环测试 12/12；Gazebo 6 隧道正反向 12/12，隧道体内横向误差 ≤ 3 cm、航向误差 ≤ 3° | ①②已提交（c1e5570，单测 5/5）；③④未做，Gazebo 仍报 tunnel_3 反向失败 |
| M3d 隧道提速 | 体内速度 0.6 → 目标值，收紧检查门参数 | 同 M3c 验收标准，耗时下降 | 未开始 |
| **M4 坡道** | 通行模块泛化到 `slope` 区域：方向约束（direction_map 代价层或整形器处理单向）、坡道速度曲线、`/nav_mode=ramp` | 所有标注坡道上/下（允许方向）通过；闭环测试 + Gazebo | 未开始 |
| M4b 限速地图 | Nav2 SpeedFilter 接 `speed_mask` | 限速区实测速度符合掩码 | 未开始 |
| **M2' 障碍层** | 点云按 `ground_z` 做高度过滤 → 局部代价地图障碍层（替代原"实时地形节点"） | Gazebo 中放置障碍物可绕行，不误报隧道顶/坡面 | 未开始 |
| M5 全量回归 | 平地 + 隧道 + 坡道 + 障碍混合目标序列，统计成功率与耗时 | 待定 | 未开始 |

顺序：M3c → M3d → M4 → M4b → M2' → M5。真车参数（`params/real`）暂缓。

## 测试分层

1. 单元测试（gtest / pytest）：几何、整形、状态机转移。
2. 运动学闭环测试（新增，无 ROS、秒级）：交接逻辑与参数在这里先调好。
3. 规划检查（`plan_tunnels`，无仿真）：整形后的路径可行。
4. Gazebo 验收（`tools/nav_regress`）：最终验收；定位问题记录在 `docs/localization_issues.md`，导航不对定位数据做过滤。
