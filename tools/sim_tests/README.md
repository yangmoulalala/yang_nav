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
| `kidnapspin` | 绑架后后端处于 SUSPECT/LOST 时以 0.3 rad/s 原地慢转（单双雷达都可用） | 后端恢复的运动门需要视角变化；车不动时对称解会连续自洽。真车对应 SUSPECT 时转云台 |

单雷达后端读 `bringup/config/sim/backend_single.yaml`（launch 按 `lidar` 选择），与 `backend.yaml` 的差异写在该文件开头。

## 参考结果（2026-11-02，`single-lidar` 分支 `1357a28`，RTX 4060 Ti，RTF 1.0）

主数据来自一次全量 `run_all.sh`（每个场景 1 次），括号里是 2026-11-01 单项测试的结果。误差来自 `zt_eval.py`，`trusted` 指定位自报 NORMAL/OBS_DEGRADED 的样本。`run_all.sh` 汇总用默认 SE(2) 对齐；单雷达边转边初始化，地图系与真值差约 6° 航向，对齐会吸收这部分。

| 场景 | 双雷达 | 单雷达 |
| --- | --- | --- |
| 建图 2 圈 | 回环接受 18 / 拒绝 0；RMSE 1.0–2.2 cm，最大 4.9 cm，航向 ≤ 0.21°（11-01：最大 4.0 cm） | `earlyspin,spawnspin`：第 4.8 s 进入 NORMAL；回环接受 23；RMSE 1.7–2.5 cm，最大 4.9 cm，航向 ≤ 0.70°（11-01 只跑 1 圈：最大 9.4 cm） |
| 前端初始化 | 第 1 次尝试成功 | `earlyspin`：3 个单雷达场景都初始化成功（11-01：6/6，开始转动后 ≤ 2.8 s） |
| 先验定位 1 圈 | 起点 4.3 cm；跑圈 RMSE 2.9 cm，最大 5.9 cm，跑圈中无 SUSPECT（11-01 共 5 次：RMSE 2.5–3.2 cm，4 次最大 ≤ 6 cm） | 起点 2.9 cm；跑圈 RMSE 2.3 cm，trusted 最大 7.1 cm（11-01 共 2 次：最大 17–19 cm） |
| 绑架 | 能恢复，但 `FALSE TRUST` 11.8 s（54 个样本，瞬移后约 10 s 仍报可信），最大偏 7.0 m（11-01 共 5 次：0.6–13.7 s） | `FALSE TRUST` 11.7 s，最大偏 11.1 m；恢复后行驶结束时仍偏 13 m，**未恢复**（11-01 共 2 次：12–17 s，定到航向反约 176° 的对称位置） |
| 原地旋转（慢 + 快） | 最大 5.1 cm，航向 ≤ 0.23° | `earlyspin`：最大 4.6 cm，航向 ≤ 0.46° |
| 双雷达标定 | 接受 4 帧 / 拒绝 19；std 1.3 cm / 0.0055 rad；本次精度未确认（见下）。2026-10-31：平移误差约 1 cm，旋转 < 1° | — |

标定场景本次在第 2.35 s 因 IMU 采样中断（`front-end input discontinuity`）进入 LOST，整场没有恢复。标定节点只读前端 `/odometry`，仍输出了外参，但没有和 URDF 真值做误差对比，所以本次标定精度未确认。

旧参考值（2026-10-31）里先验定位“最大 8.0 cm”用当前 `zt_eval.py` 无法复现：同一次运行重算为最大 36.8 cm，大误差都在 SUSPECT 冻结输出期间。

## 绑架修复验证（2026-11-02，`kidnap-recovery-fix` 分支 `fabbcc5`）

上表的绑架行是修复前基线。修复内容见 `docs/backend_guide.md` 3.1、4.5–4.6、7.3。验证用 `/tmp/zt_all/map_*` 地图，单雷达加 `earlyspin`。

```bash
# 单场示例：绑架后慢转
LIDAR=single tools/sim_tests/zt_run.sh localization /tmp/zt/k ~/yang_nav/maps/sim_single 1 localize earlyspin,kidnapspin
LIDAR=dual   tools/sim_tests/zt_run.sh localization /tmp/zt/k ~/yang_nav/maps/sim_test 1 localize kidnapspin
```

| 场景 | 修复前 | `fabbcc5` |
| --- | --- | --- |
| 双雷达绑架 + `kidnapspin`（5 场） | `FALSE TRUST` 9.0 s；恢复后 trusted 最大 3.68 m | 5/5 恢复；`FALSE TRUST` 0–2.6 s（只在瞬移刚发生时）；恢复后 trusted 最大 0.09–0.31 m；最终 4–14 cm |
| 单雷达绑架 + `kidnapspin`（5 场） | 锁到 180° 对称位姿，结束时偏约 12 m | 只有 1/5 干净恢复（最终 2 cm）；其余锁到约 99°/180° 错误位姿（结束时偏 5–11 m），或恢复正确后行驶中跑偏到 8.4 m |
| 两种雷达绑架，车不动 | 同上 | 运动门不起作用：双雷达 2/3 停在不可信状态未恢复；停车兜底可能提交错误解（日志 `stationary limit`） |
| 全量 `run_all.sh`（不带 `kidnapspin`） | 见上表 | 建图最大 3.5 / 3.6 cm，原地旋转 4.0 / 5.5 cm，先验跑圈 trusted 最大 4.4 / 8.4 cm（单 / 双），标定与 URDF 差约 9.5 mm；绑架两种雷达仍锁错 |

新增日志关键字：`(severe N/2)`（严重冲突）、`during recovery probation`（试用期内升级）、`ambiguous result, deferring recovery`（歧义推迟）、`recovery candidate accepted after N consistent candidates, motion=..`（运动门通过）、`(stationary limit)`（停车兜底提交）。

## 注意

- 需要 `gz.transport13`、`gz.msgs10` Python 绑定（Gazebo Harmonic 自带）。
- 瞬移不产生 IMU 运动，前端感知不到，比真实碰撞更苛刻。
- 必须设置 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`（脚本已设置），否则雷达掉到约 3 Hz。
- 瞬移偶尔返回失败（`TELEPORT False`），但多数情况下位姿实际已跳变（11-02 验证中 4 次都是这样）。用 `kidnap` 阶段的误差（约 7 m）判断绑架是否发生，不要只看返回值。
- Gazebo 关闭时段错误（exit 139）、`merge_node` 关闭时 exit 1 都发生在 SIGINT 之后，不影响结果。
- 前端 IMU 采样中断后，建图模式下后端会停在 LOST 直到结束（已出现 2 次：单雷达原地旋转、双雷达标定），原因未查。
