# 定位问题记录（导航测试中发现）

> 2026-11-08：定位已删除 UWB，重定位只在附近 2 m 内搜索。下面记录里的 UWB 相关现象（UWB 恢复、UWB 校验、`uwb_zero_pose`）只作历史参考。

导航侧直接信任定位输出（TF `map -> lio_odom -> base_link`、`pose_trusted`），不做任何过滤或修正。
下面的问题都留到定位端处理。

环境：RMUC 2027 仿真（`arena:=rmuc uwb:=true gui:=false`），`bringup sim.launch.py lidar:=dual mode:=localization uwb:=true`，
先验地图 `maps/sim`。所有日志已从 `/tmp` 复制到 `~/yang_nav_issue_logs/<run>/`（`loc.log`、`truth.log` 为 gz 真值）。
时间戳为墙钟秒（2026-11-06）。

---

## L1 车实际移动时估计比真值多偏约 0.8 m，pose_trusted 仍为 true（已更正）

- 日志：`nav_m0_pass`，1793965409 - 1793965425
- 现象：车已到达目标（估计 map (-2.08, -4.24)，与真值一致），之后 2 s 内估计向南漂到 map (-1.78, -8.26)，
  约 4 m，超出地图边界。期间状态只在 NORMAL / OBS_DEGRADED 间切换，`pose_trusted` 一直为 true。
  导航按错误位姿控制，车真实往南开出约 1.7 m（真值 y -4.4 -> -6.05）。
- 之后才进入 SUSPECT（1793965416.6），靠 UWB + scan-to-map 恢复时修正 `delta_t=2.28 m`（1793965424.9）。
- 日志特征：`correction conflict ... all available global observations conflicted: prior scan-to-map NIS=21.3/6`。
- 更正（nav_m0_pass4 复查）：车确实被导航开出约 2.4 m（真值同向移动），不是纯定位跳变；定位只是额外多偏约 0.8 m（UWB 恢复修正 0.82 m）。
  导航侧原因：到点时大角度原地转身导致 MPPI 偏离目标，已在导航参数中处理。定位侧仍需关注：偏差 0.8 m 时 pose_trusted 仍为 true。

## L2 恢复时位姿跳变 1 m 以上

- 日志：`nav_m0_final`（1793959918、1793959928）、`nav_m0_pass`（1793965424）
- `recovered via UWB + scan-to-map` 时 `delta_t` 为 1.48 m、1.00 m、2.28 m。
- 说明 SUSPECT 之前发布的位姿已经偏了这么多，和 L1 是同一类问题。

## L3 平地行驶时周期性进入 SUSPECT / OBS_DEGRADED

- 几乎每轮都有，每次 5-20 s，`cmd_gate` 停车。位置集中在出生点西南（约 world (-10.5..-12.5, -2..-4.5)）。
- 原因字样：`observation model invalid: prior scan-to-map cannot construct a pose likelihood`、
  `correction conflict ... position-only evidence cannot confirm the pose while full-pose evidence conflicted`。

## L4 ZLIO 因 IMU 间隔超时重置

- `IMU input interrupted: IMU sampling gap exceeds the maximum interval`，多发生在 nav2 启动后 10-20 s 内（CPU 峰值），
  不开 RViz 也会发生（`nav_m0_norviz`：1793963181、1793963315）。仿真实时率全程约 1.0。
- 建议：评估 `bringup/config/sim` 中允许的最大 IMU 间隔是否过严。

## L5 ZLIO 重置后后端长期停在 LOST（tf_frozen=true）

- 日志：`nav_optfail3`、`nav_m0_norviz`、更早的 RViz 轮次。车在出生点静止，重置后 90 s 以上不恢复。
- 重定位每次报 `ambiguous result, deferring recovery`：正确候选（出生点 KF）与镜像候选
  （相距约 18.85 m、转角 179.9°）残差几乎相同；UWB 校验 `support_ok=false`（对应点约 42%）一直失败。
- 怀疑：`uwb_zero_pose=online` 需要已接受的 scan-to-map 位姿启动，重置后可能失效，形成死循环。
- 也会发生在启动阶段：`nav_m0_pass2`，后端就绪（1793966135.9）2.5 s 后 ZLIO 重置（1793966138.5），
  此前从未进入 NORMAL；之后约 280 s 内 `ambiguous` 853 次、UWB 校验失败 848 次，车始终静止在出生点。

## L6 镜像位姿被接受

- 日志：`nav_m0b`（1793950590 SUSPECT，约 8 s 后 `pose_trusted=true`）。
- 估计位姿为真值绕场地中心转 180° 的位置（真值 (-4.7, -4.4)，估计 (4.7, 4.3)）。
- 注意：该轮存在 3 个残留 `uwb_sim` 同时发 `/uwb_pose`（测试脚本 bug，已修），可能是诱因；
  清理后未再见镜像被接受，但 L5 的歧义仍在。

---

## L7 绑架恢复后行驶中自报可信但偏差大（2026-11-08 全量回归 `/tmp/zt_all2`）

- 配置：无 UWB，重定位只在 2 m 内搜索，绑架为离跑圈终点 1.5 m、航向 45°。四个定位场景全部恢复，未触发全局搜索。
- `single_localization`（RMUL 单雷达）：恢复后 3.8 cm，行驶中自报可信最大偏 2.97 m，`FALSE TRUST` 2.8 s。
- `rmuc_single_localization`：恢复后 3.2 cm，行驶中自报可信最大偏 1.17 m；`FALSE TRUST` 合计 28.7 s（含瞬移刚发生时）。
- `dual_localization`：行驶中自报可信最大偏 0.69 m（0.2 s）。`rmuc_dual_localization` 正常（最大 0.18 m）。
- 属定位端问题（主要是单雷达），尚未分析日志；与导航无关。

## 对导航测试的影响

M0 平地验收的失败中，由定位引起的：`nav_m0_pass` 目标 3、4（L1），`nav_m0_norviz`、`nav_optfail3` 全部（L5）。
回归脚本用 gz 真值对每个目标记录定位误差，只用于给测试结果归类，不进入导航控制链。
