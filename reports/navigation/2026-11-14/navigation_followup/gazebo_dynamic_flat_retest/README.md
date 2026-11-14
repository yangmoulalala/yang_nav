# Gazebo 动态实体平地补测（2026-11-14）

> 归档说明（2026-11-14）：本目录保留结论、脚本和精简证据。大型原始数据已转入本机 `backup/report-evidence-2026-11-14.tar.zst`，源码快照、编译产物和重复中间记录已清理；清单与恢复方法见 `reports/maintenance/2026-11-14/commit_cleanup/README.md`（路径均从仓库根目录解析）。下文的 before/after、trace 和输入文件路径按原实验记录保留，现存状态以清单为准。

本组补测接着动态障碍软件夹具测试，直接复用运行中的 RMUC Gazebo、双雷达、定位和完整 Nav2 栈。测试路线均在地图南侧平地，不穿越 `zones.yaml` 中标注的隧道、坡、台阶区域。Gazebo 障碍由 1 kg 方块实体 `nav_dynamic_obstacle_smoke` 提供，传感器点云走真实仿真 `/nav/obstacles` 流，目标通过 `/navigate_to_pose` action 下发。

## 被占目标

起点约为 map `(5.80, -5.99)`，目标为 `(7.10, -5.99)`，静态地图中心线路径净空至少 0.906 m。把方块放在目标附近后发目标，Nav2 接受了 action；10 秒状态采样仍为 `EXECUTING (2)`，行为树日志中 `KeepOccupiedGoalPending` 为 `RUNNING`，定位仍为 `NORMAL / pose_trusted=true`。`/nav/obstacles` 记录到 290 帧，单帧 27–126 点。障碍移开后，原 action 以 `SUCCEEDED / error_code 0` 结束，整次任务耗时 27.1 s。

这证明 live Gazebo 传感器、目标接受、pending 行为树分支和障碍清除后的原任务恢复连通。该轮没有把障碍精确移除时刻写入日志，因此不报告清障后的单独恢复延迟。自由方块在等待期间漂移约 0.25 m；10 秒采样时车距目标约 1.2 m，路径绕行幅度较大，所以该轮**不作为固定占用目标最小停车距离或接近效率的验收**。详细结果在 `occupied_goal_result.json`，证据在 `goal.log`、`status_pending.yaml`、`behavior_tree.yaml`、`obstacles.txt`、`plans.txt` 和 `localization_trace.jsonl`。

## 移动障碍横穿

从 map 约 `(7.08, -5.93)` 导航回 `(5.80, -5.99)`。方块从路径上的 `(6.40, -5.99)` 开始，再用 5 次 Gazebo `set_pose` 更新横向移到 `(6.40, -6.74)`，移动约 0.75 m、耗时约 2.2 s。Nav2 action 在障碍移动前后的采样均为 `EXECUTING (2)`，未被拒绝或取消；规划器更新 9 次，障碍云 96 帧（每帧 19–84 点），按同一时刻的 Gazebo 真值和定位 TF 对齐后，点云中的障碍中心从约 `(6.30, -5.88)` 移至 `(6.50, -6.75)`，最近返回点距估算实体中心 0.159 m。路径绕到障碍南侧后返回目标；action 在 12.0 s 以 `SUCCEEDED / error_code 0` 结束。末端定位约 `(5.793, -6.180)`，距请求目标约 0.190 m。

机器人与方块中心的最小观测间距为 0.603 m。该实体的实际碰撞几何没有在本轮导出，因此**不把中心距换算成车体 footprint 安全净空**。障碍运动由 Gazebo 服务离散设置位姿，约 0.34 m/s 的低速横移，不等同于带速度控制的人形行人模型，也不是动态人群或高速穿行验收。详见 `moving_crossing/result.json`、`goal.log`、`status_before_move.yaml`、`status_after_crossing.yaml`、`gazebo_poses.jsonl`、`obstacles.txt`、`plans.txt` 和 `localization_trace.jsonl`。

## 收尾状态

方块已恢复到原始世界暂存点附近 `(−3.43, 1.48)`，当前没有进行中的导航目标。定位状态为 `NORMAL / pose_trusted=true`；TF probe 在启动发现窗口先报告尚未建立 `map` 帧，随后稳定获得 `map → lio_odom → base_link`，没有发现持续 TF 中断。仿真栈保持运行，便于接续人工检查。
