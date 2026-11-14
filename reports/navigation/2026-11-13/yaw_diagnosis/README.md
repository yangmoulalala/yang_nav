# 洞口 yaw 晃动诊断（2026-11-13）

> 归档说明（2026-11-14）：本目录保留结论、脚本和精简证据。大型原始数据已转入本机 `backup/report-evidence-2026-11-14.tar.zst`，源码快照、编译产物和重复中间记录已清理；清单与恢复方法见 `reports/maintenance/2026-11-14/commit_cleanup/README.md`（路径均从仓库根目录解析）。下文的 before/after、trace 和输入文件路径按原实验记录保留，现存状态以清单为准。

结论：本轮复现的主要问题在导航输出和命令处理链。Gazebo 底盘能跟随最终 `/cmd_vel.angular.z`；不能把观察到的左右摆动归为底盘自身 yaw 执行振荡。导航控制器已输出反复换向的角速度，且经过碰撞监控后有明显延迟。另有整形失败后控制器仍换路、参考航向变化的问题。

本轮只增加诊断证据，没有修改正式导航、仿真、地图或定位源码/参数，没有提交 Git。实验使用独立 ROS domain/Gazebo partition，结束后仅关闭本轮创建的进程组。

## 输入与范围

- 已读取 Windows Claude 项目记忆：`--wsl-localhost-ubuntu24-home-yang-yang-nav/memory/`，按记忆使用 WSL 实际仓库 `/home/yang/yang_nav`。保留原有未提交修改及暂存状态。
- 原手动运行 `/tmp/nav_manual_gui`，约 21:39—21:51；原始 nav/truth/loc 日志副本见 `original_manual/`。
- 原日志未录制速度链，且 `FollowPath.zone.trace=false`，不能从旧日志精确追溯每次换向。本轮根据原日志前两个目标重现：map `(1.87,-1.70,0°)` → `(7.69,-3.24,0°)`。旧 RViz 目标日志未提供 yaw，复现取 0°，故不是原运行的逐指令确定性回放。
- 完整复现记录 `/cmd_vel_nav`、`/cmd_vel_smoothed`、`/cmd_vel_collision_checked`、`/cmd_vel`、区域 phase、参考 yaw、TF、定位里程计、Gazebo 真值及路径，见 `repro/chain_trace.jsonl`。

## 1. 独立底盘执行检查

只启动原 RMUC Gazebo 和 bridge，不启动导航、定位、平滑器或命令门；经 ROS `/cmd_vel` 到 Gazebo 的生产 bridge 连续发布恒定指令。使用模型 pose 的仿真时间计算 yaw 变化，避开 ±π 包角问题。

| 指令 rad/s | 稳态真值 rad/s |
|---|---|
| +0.3 | +0.299931 |
| -0.3 | -0.299977 |
| +0.6 | +0.600341 |
| -0.6 | -0.600392 |
| +1.0 | +1.000152 |
| -1.0 | -1.000077 |

每项旋转 5 仿真秒，去掉前 1 秒估计稳态。转速相对误差均小于 0.07%，去趋势 yaw 残差峰峰值 0.073°—0.125°，无自发往复振荡。±0.3 m/s 前后/侧向平移各 3 秒，yaw 总变化绝对值最大约 0.265°。

证据：`open_loop_trace.jsonl`、`open_loop_summary.json`、`physics_sim.log`。

此项证明空旷处基本转速执行正常，不排除碰墙、坡道轮地接触等特定工况扰动。复现中的最终指令对真值的比较进一步检查了洞口工况。

## 2. 原参数复现：延迟发生在底盘前面

- 导航控制器输出约 20 Hz，平滑器约 30 Hz。
- 碰撞监控 `/cmd_vel_collision_checked` 的接收间隔中位数 **202 ms（约 4.94 Hz）**。
- 命令门仍以 30 Hz 发布 `/cmd_vel`，但其中大量是重复的旧输入；最终话题 30 Hz 不代表控制信息以 30 Hz 更新。
- 角速度信号互相关估计：导航输出到真值约 **0.47—0.48 s**，最终 `/cmd_vel` 到真值约 **0.04—0.05 s**，最终命令与真值角速度相关系数约 **0.99**。具体值随统计窗口/插值设置略变。
- 相邻级信号估计：nav→smoothed 约 90 ms，smoothed→collision_checked 约 210 ms，collision_checked→final 约 90 ms。后两项包括低频消息采样保持及接收时刻的影响，不能当作每个节点严格的单消息执行时间。
- 参考固定为 -89.6° 时，真值仍出现约 -114° 过冲及随后回到约 -73°，同时导航输出正负转速交替；最终指令与实际转速贴合。这是有延迟的闭环反复纠偏，并非稳定最终指令下底盘自己摇摆。
- TF yaw 相对真值的信号拟合延迟约 190 ms，也占用反馈相位裕量。本轮不修改或过滤定位输出。

证据：`yaw_chain.png`、`repro_metrics.json`、`ab_metrics.json`。

## 3. 单参数诊断对照

只在 `/tmp/navdev/yaw_nav2_ab.yaml` 把 collision_monitor 的 `base_shift_correction` 从 true 改为 false；正式 YAML 保持原值。其余导航参数相同，仍启用碰撞监控、定位可信门和观测门。两次运行都有 `trace=true`，仅用于日志。

| 项目 | 原配置 | 临时 latest-TF 对照 |
|---|---|---|
| collision_checked 中位间隔 | 202 ms | 33 ms |
| smoothed→collision_checked 信号延迟 | 约 210 ms | 本次 10 ms 扫描分辨率下约 0 ms |
| collision_checked→final 信号延迟 | 约 90 ms | 约 20 ms |
| 洞口航向行为 | 多次大幅纠偏、换向 | 初次对齐明显收敛，入段日志 pose -90.1° / ref -89.6° |

Nav2 Jazzy 上游 `Source::getTransform()` 的 true 分支使用 `base_frame_id` 在 `curr_time` 的 TF；false 分支使用最新变换。当前配置 `odom_frame_id: map`、`transform_tolerance: 0.5`，等待当前 TF 是输出低频/延迟的强证据链；需后续通过本机精确版本及节点计时进一步确认各耗时。上游参考源码及来源说明保存在 `scripts/`。

对照只是单次诊断，两次定位/规划轨迹不完全相同，不能宣称完成性能验收。**不能直接把 false 作为正式修复**：它改变观测与运动补偿的时间语义，需要检验障碍投影、动态停车和失流保护。两次第二目标均超时取消，整体通行问题尚未解决。

## 4. 独立发现：整形失败后仍向控制器送新路径

原日志和复现均出现同类序列。复现可定位到 `repro/nav.log`：

- wall 1794578327.101：tunnel_5 entry margin 完整足迹碰撞，SmoothPath action 失败。
- 1794578327.132：`Passing new path to controller`，随后 tunnel_3 `done` 并切回 normal，实际仍在入口附近。
- 1794578329.192：成功整形后再次送路，参考 -89.6°；随后 1794578333.271 又发生整形失败和换路。
- 1794578334.322：tunnel_3 整形失败，控制器紧接着收到路径，报告 heading locked 90.0°、实际参考约 159.6°；与前面的 -89.6°发生明显变化。

BT 中 ComputePathToPose 的输出与 FollowPath 共用 `{path}`，SmoothPath 的输入和输出也共用该变量；IsPathValid 只检查路径碰撞，无法证明已通过 zone_shaper。**已确认失败后换路/区域状态变化；原始规划路径通过共享黑板泄漏是高优先级待验证机制**，应通过独立 raw/shaped path 和失败场景测试验证。不能再仅凭 shaper 抛异常宣称失败传播已闭环保证。

Tracker 的 `body_yaw` 主要用于报告，实际 yaw_ref 由当前路径 heading、flip 和 entry error 生成，故日志里的 “heading locked” 不等于实际参考 yaw 恒定。路径变化和同一区域方向/flip 重选可进一步放大摆动。

## 后续修复顺序

1. 修复并验证命令链对当前 TF 的等待及旧指令传播，使碰撞监控及时处理最新输入，保持碰撞和失流保护。
2. 修复 BT 的路径发布事务：只有整形成功才能替换 FollowPath 使用的路径；整形失败不得让未验证的新路径进入控制器。
3. 在以上问题消除后，复核入口 yaw_ref 连续性、重规划状态保存及 k_yaw 与实际延迟的匹配，再做洞口闭环回归。现在先调小底盘或 tracker 增益只会掩盖链路问题。

脚本和临时 YAML 保存在 `scripts/`，生产源码哈希见 `source_hashes.json`。`collision_source.cpp` / `collision_monitor_node.cpp` 来自 2026-11-13 读取的 https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_collision_monitor/src ，仅作机制参考，不冒称与本机二进制逐行相同。