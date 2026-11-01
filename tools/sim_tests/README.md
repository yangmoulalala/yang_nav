# 定位仿真回归测试

在 Gazebo 仿真中用真值闭环驱动小车，记录 Gazebo 真值与 TF `map → base_link`，评估建图、回环、先验定位、绑架重定位、
原地旋转和双雷达标定，单雷达与双雷达均可测。

| 文件 | 作用 |
| --- | --- |
| `run_all.sh` | 全量回归：单/双雷达 ×（建图、定位+绑架、原地旋转）+ 双雷达标定，串行运行并打印汇总；单雷达场景自动加 `earlyspin`（建图另加 `spawnspin`） |
| `zt_run.sh` | 一键启动仿真 + `bringup sim.launch.py`，运行驱动脚本后清理进程组；环境变量 `LIDAR=single\|dual`（默认 dual），`CALIB=1` 时同时启动标定 |
| `zt_driver.py` | 按真值跟踪路线（补给区 → 中央 → 返回）；`mapping` 结束时调用 `save_map`；`localize` 跑完后用 `set_pose` 瞬移到 (6.0, 1.5, 90°) 并等待恢复；`spin` 原地慢转/快转；`calib` 依次停到 5 个拐角/中央点、每处 4 个朝向静止 6 s；第 4 个参数（`zt_run.sh` 第 6 个）为逗号分隔的标志，见下文[单雷达](#单雷达) |
| `zt_eval.py` | SE(2) 对齐真值与估计（参考样本几乎不动时改用首样本锚定），按阶段输出 RMSE / 最大误差 / 航向误差；`FALSE TRUST` 行统计定位自报可信但误差 > 0.5 m 的样本（错误恢复）；SUSPECT/LOST 时后端把输出冻结在最后可信位姿、误差随行驶距离增长，故每个阶段另列 `trusted` / `untrusted` 统计；`--align YAW_DEG TX TY` 用固定对齐（取建图那次的 alignment） |
| `cloud_rate.py` | 不反序列化点云地统计话题频率与 RTF（`ros2 topic hz` 对大点云会严重低估） |

## 用法

```bash
cd ~/yang_nav
# 全量回归（约 30–40 分钟），结果在 /tmp/zt_all/<场景>/，最后打印汇总
tools/sim_tests/run_all.sh /tmp/zt_all

# 单雷达建图（初始化期间慢转 + 出生点慢转一圈，见下文“单雷达”）
LIDAR=single tools/sim_tests/zt_run.sh mapping /tmp/zt/single ~/yang_nav/maps/sim_single 1 mapping earlyspin,spawnspin
# 单雷达先验定位 + 绑架
LIDAR=single tools/sim_tests/zt_run.sh localization /tmp/zt/single_loc ~/yang_nav/maps/sim_single 1 localize earlyspin

# 双雷达标定（结果写 /tmp/zt/calib/extrinsic.yaml）
CALIB=1 tools/sim_tests/zt_run.sh mapping /tmp/zt/calib /tmp/zt/calibmap 0

# 1. 建图 2 圈并保存地图
tools/sim_tests/zt_run.sh mapping /tmp/zt/mapping ~/yang_nav/maps/sim_test 2
python3 tools/sim_tests/zt_eval.py /tmp/zt/mapping/result.json
grep -c 'loop scan-to-map accepted' /tmp/zt/mapping/bringup.log

# 2. 用该地图定位 1 圈，然后绑架
tools/sim_tests/zt_run.sh localization /tmp/zt/loc ~/yang_nav/maps/sim_test 1
python3 tools/sim_tests/zt_eval.py /tmp/zt/loc/result.json
# 定位跑偏或绑架会把默认 SE(2) 对齐带偏：改用建图那次输出的 alignment
# python3 tools/sim_tests/zt_eval.py /tmp/zt/loc/result.json --align YAW_DEG TX TY
```

输出目录里有 `sim.log`、`bringup.log`、`driver.log` 和 `result.json`；标定场景另有 `calib.log` 与 `extrinsic.yaml`。

改动参数后先运行 `python3 tools/check_bringup_params.py`，确认 bringup 配置与各模块 example 的键一致。

## 单雷达

MID-360 侧装（URDF `mid360_joint` 俯仰约 90°），静止时只看到一侧，单雷达要转车补视野。`zt_driver.py` 的标志：

| 标志 | 作用 | 原因 |
| --- | --- | --- |
| `earlyspin` | 先静止 2 s，再以 0.3 rad/s 慢转直到后端定位（NORMAL/OBS_DEGRADED），然后转回航向 0 | 静止时 ZLIO 初始化最弱方向支撑只有 2–4.8（门槛 5），反复重来、可能几分钟不完成；转动后 ≤ 2.8 s 完成（6/6）。首次尝试的重力仍取自静止 IMU |
| `initspin` | ZLIO 初始化完成后才慢转 | 只补后端重定位视野，前端仍可能久等 |
| `spawnspin` | 建图时先在出生点慢转一圈再跑圈 | 侧装雷达各航向画面不同；地图在出生点缺其他航向的关键帧时，启动慢转会让重定位锁到对称位置（曾偏 5.5 m / 94°） |
| `spin` | 每个路点原地转一圈 | 单雷达建图不要用：建图最大误差升到约 27 cm |

单雷达后端读 `bringup/config/sim/backend_single.yaml`（launch 按 `lidar` 选择），与 `backend.yaml` 的差异写在该文件开头。

## 参考结果（2026-11-01，`single-lidar` 分支，RTX 4060 Ti，RTF 1.0）

误差来自 `zt_eval.py`，`trusted` 指定位自报 NORMAL/OBS_DEGRADED 的样本；定位场景用建图那次的固定对齐。每组只跑 1–5 次。

| 场景 | 双雷达 | 单雷达 |
| --- | --- | --- |
| 建图 | 2 圈：回环接受 17 / 拒绝 0；RMSE 0.9–1.8 cm，最大 4.0 cm，航向 ≤ 0.20° | 1 圈 + `earlyspin,spawnspin`：回环接受 9 / 拒绝 0；RMSE 1.8–3.4 cm，最大 9.4 cm |
| 前端初始化 | 第 1 次尝试成功 | `earlyspin`：6/6 成功，开始转动后 ≤ 2.8 s |
| 先验定位 1 圈 | 5 次：起点约 4 cm；跑圈 RMSE 2.5–3.2 cm，4 次最大 ≤ 6 cm（1 次末样本 26 cm）；跑圈中无 SUSPECT | 2 次：起点 2–4 cm；跑圈最大 17–19 cm（trusted 最大 11.5–18 cm），无 LOST |
| 绑架 | 能恢复，但 `FALSE TRUST` 0.6–13.7 s，可信状态下最大偏 7.3 m | 能恢复，但常定到航向反约 176° 的对称位置，`FALSE TRUST` 12–17 s，最大偏 10.5 m |
| 原地旋转（慢 + 快） | 最大 4.9 cm，航向 ≤ 0.28° | 最大 8.7 cm，航向 ≤ 0.75° |
| 双雷达标定 | 4 帧，平移误差约 1 cm（std 1.4 cm），旋转 < 1°（2026-10-31） | — |

未用当前配置复测：双雷达建图与原地旋转（收敛门槛放宽前测得）、单雷达建图（`backend_single.yaml` 引入前测得）、单雷达原地旋转（旧配置、未用 `earlyspin`）、双雷达标定。

旧参考值（2026-10-31）里先验定位“最大 8.0 cm”用当前 `zt_eval.py` 无法复现：同一次运行重算为最大 36.8 cm，大误差都在 SUSPECT 冻结输出期间。

## 注意

- 需要 `gz.transport13`、`gz.msgs10` Python 绑定（Gazebo Harmonic 自带）。
- 瞬移不产生 IMU 运动，前端感知不到，比真实碰撞更苛刻。
- 必须设置 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`（脚本已设置），否则雷达掉到约 3 Hz。
- 瞬移偶尔返回失败（`TELEPORT False`），该次绑架数据无效；曾有一次返回失败但位姿已跳变。
- Gazebo 关闭时段错误（exit 139）、`merge_node` 关闭时 exit 1 都发生在 SIGINT 之后，不影响结果。
