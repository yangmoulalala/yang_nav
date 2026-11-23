# 导航回归与诊断工具

完整仿真入口 `run_nav.sh`、`run_tunnel_accept.sh` 和 `sim_rviz.sh` 支持 `LIDAR=single|dual`（默认 dual）。
单雷达会在启动 Nav2 前运行 `zt_driver.py ready ... earlyspin`，以慢转补足侧装 MID-360 初始化视野；
需要与先验地图匹配的出生点和航向。示例：

```bash
ROS_DOMAIN_ID=122 LIDAR=single OUT=/tmp/nav_single \
  bash tools/nav_regress/run_nav.sh '1.0 1.0 0'
```

坐标为格式示例，运行前选取当前地图上的可通行目标。`ready` 只初始化并停车，不跑圈、保存地图或绑架。
自动回归可用 `LOC_WAIT_ATTEMPTS`（默认 90）限制目标间等待可信定位的查询次数，
`MAX_LOC_RESTARTS`（默认 2，设 0 禁止自动重启）限制目标间定位重启。等待失败的目标记录为 skipped 并使运行失败。

已有定位与传感器时执行 `bash tools/start_navigation.sh rviz:=true`。位姿、观测、规划与速度之间的接线见[系统架构](../../docs/architecture.md)。

`support/plant.py` 提供按最终 `/cmd_vel` 积分的运动学底盘。`support/terrain.py` 生成测试地形。`support/runtime.py` 管理日志、进程组和结果文件，各场景单独维护验收判据。

在 WSL 项目根目录执行，先加载 ROS 和工作区环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 当前回归入口

| 工具 | 输入与输出 | 检查范围 |
|---|---|---|
| `terrain_check.py` | 地形目录 → JSON，失败退出非零 | 文件完整性、地面/净高数据及源项目一致性；不证明绝对高程或物理可通行 |
| `verify_nav_system.sh` | 已安装四包 → 终端结果 | 库、插件注册、两个 BT 与配置的静态接线检查；不启动运行验证 |
| `framework_smoke.py` | 自动生成 fixture → `result.json` / `navigation.log` | 生命周期、两个 BT 接线、障碍过滤、速度限制、collision_monitor 减速、障碍清除及观测断流停车 |
| `framework_accept.py` | 自动生成 fixture → `result.json` / `navigation.log` / `trace.jsonl` | 真实 Nav2 节点及最终 cmd_vel 的全向运动学闭环，包括单/多目标、区域通行、取消/抢占与安全门 |
| `control_rate.py` | 自动生成 fixture + PTY → `result.json` / 两个进程日志 | 核对安装后的 100 Hz 参数并测量控制器、平滑、碰撞监控、命令门与串口实际频率；确定性底盘，无硬件 |
| `dynamic_accept.py` | 自动生成 fixture → `result.json` / `navigation.log` / `trace.jsonl` | 横穿、挡路、占用目标接近等待、观测断流、障碍移除与新目标抢占；真实 Nav2 + 运动学底盘，不含 Gazebo 接触物理 |
| `recovery_ladder.py` | 两个不可达目标与测试 phase → `result.json` / `navigation.log` | 区域外恢复升级、区域 phase 内禁止运动恢复 |
| `plan_tunnels.sh` + `plan_tunnels.py` | 地形目录 → 终端统计与 planner/smoother 日志 | 当前地图各隧道双向规划；设置 SMOOTHER 才检查整形，action 失败计失败 |

```bash
python3 tools/nav_regress/terrain_check.py maps/sim/terrain
bash tools/nav_regress/verify_nav_system.sh
ROS_DOMAIN_ID=85 python3 tools/nav_regress/framework_smoke.py --out /tmp/nav_framework_smoke
ROS_DOMAIN_ID=86 python3 tools/nav_regress/framework_accept.py --out /tmp/nav_framework_accept
ROS_DOMAIN_ID=198 python3 tools/nav_regress/control_rate.py --out /tmp/nav_control_rate
ROS_DOMAIN_ID=87 python3 tools/nav_regress/recovery_ladder.py --out /tmp/nav_recovery_ladder
ROS_DOMAIN_ID=77 SMOOTHER=zone_shaper OUT=/tmp/nav_plans tools/nav_regress/plan_tunnels.sh maps/sim/terrain
```

这些工具默认使用各自的 ROS domain；已有 `ROS_DOMAIN_ID` 会覆盖默认值。避免在同一 domain 启动重复栈。运动学和冒烟工具自动生成地形并清理自己启动的进程，原始日志留在输出目录，方便失败诊断。

场地检查使用指定地图的数据。`terrain_check.py` 会在缺少隧道净空时返回非零，`plan_tunnels.sh` 单独报告实际规划及整形结果。

## Gazebo 与路径诊断

这些工具保留用于后续场地/物理验证。仅运行或生成目标不代表已经通过验收。

| 工具 | 用途 |
|---|---|
| `run_nav.sh` | 启动 Gazebo、定位、导航并运行给定 world/map 目标；输出到 OUT |
| `sim_rviz.sh` | 手动仿真导航与 RViz |
| `run_tunnel_accept.sh` | 生成隧道目标，调用仿真运行，再检查 ground-truth 穿越误差 |
| `tunnel_accept.py` | 生成目标与覆盖 manifest；不验证执行结果 |
| `tunnel_stats.py` | 从运行目录的 truth.log 与 manifest 检查双向覆盖和体内误差 |
| `gz_truth.py` | 将 Gazebo 位姿流转换为含航向的 ground-truth 轨迹 |
| `chaincheck.sh` / `chaincheck.py` | 检查多腿路径实际经过指定隧道；不代替整形或闭环检查 |
| `goal.py` | 向已有仿真栈发送单目标；默认 world 坐标，yaw 单位为度 |
| `pick_flat_goals.py` | 从地图选取平地测试目标 |
| `terrain_path.py` | 解析实际地形包路径，供其他脚本复用 |

仿真入口示例：

```bash
ROS_DOMAIN_ID=78 OUT=/tmp/nav_field tools/nav_regress/run_tunnel_accept.sh --order pair
python3 tools/nav_regress/tunnel_stats.py --out /tmp/nav_field --manifest /tmp/nav_field/tunnel_manifest.json
```

## 可选观测记录

`run_nav.sh` 可通过 LOC_REC、OBS_REC、CM_REC、PLAN_REC 启用以下记录器：

- `LOC_REC=true` 启动 `record_localization.py`：记录前端观测/预测、map→lio_odom 和 lio_odom→base_link TF、全局里程计、定位状态、最终速度和 Gazebo 真值，保留每条消息的仿真时间。只记录帧的标量元数据，不保存原始点云。`sim_rviz.sh` 默认启用；可用 `LOC_REC=false` 关闭。输出为 `localization_trace.jsonl`，目录中已有同名文件时另建带时间戳的文件，保留旧记录。

- `record_obs.py`：记录地面相对障碍过滤的观测。
- `record_costmap.py`：保存代价地图快照，`cm_dump.py` 用于离线查看。
- `record_plan.py`：保存规划路径。
- `showpath.py`：查看 `chaincheck.py` 生成的 `/tmp/chain_paths.txt`。

`record_localization.py` 在 TF 每行记录原始 `TFMessage` 的 `batch` 编号。成对发布的两条边必须按整个消息更新后再组合，不能把日志中的相邻两行误当成两个运行时 TF 状态。`record_costmap.py` 保留源时间戳与 frame。

```bash
python3 tools/nav_regress/analyze_tf_continuity.py /tmp/nav_field \
  --output /tmp/nav_field/continuity.json
```

该分析输出逐边跳变、按消息接收顺序组合的最大位置步长、超过 0.3 m 的组合跳变和局部图原点异常；它是接收侧诊断，不替代 tf2 插值查询或真值误差计算。

隔离仿真栈可在目标完成并保持运行（例如 `HOLD=180`）期间，验证一次静止前端重置：

```bash
ROS_DOMAIN_ID=78 python3 tools/nav_regress/reset_tf_accept.py \
  --output /tmp/nav_field/reset_accept.json
ROS_DOMAIN_ID=87 python3 tools/nav_regress/dynamic_accept.py --out /tmp/nav_dynamic
```

重置工具等待静止、可信状态，调用 `/localization_reset` 并检查代次改变、恢复和组合 TF 连续性；只用于测试 domain，不用于正在驾驶的真车。建图、定位、导航及原始传感器录包可用 `tools/record_bag.sh`。[录包入口](../README.md)

## 占用起点回归

`occupied_start.py --out /tmp/nav_occupied` 用真实 Nav2 和运动学底盘验证边角重叠、车心落入较厚静态障碍时的退出、到点和后续新目标；`--recorded-exit` 使用正式地图及 2026-11-13 手动日志的两个出口位姿 (4.52,-5.24)、(4.59,-5.27)。测试保持碰撞监控、定位可信门和观测门开启；只模拟代价图重叠，不模拟实体穿墙。使用独立 ROS_DOMAIN_ID，并在已加载 Jazzy/工作空间的终端运行。
