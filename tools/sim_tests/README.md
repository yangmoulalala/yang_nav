# 定位仿真回归测试

在 Gazebo 仿真中用真值闭环驱动小车，记录 Gazebo 真值与 TF `map → base_link`，评估建图、回环、先验定位、绑架重定位、
原地旋转和双雷达标定，单雷达与双雷达均可测。

| 文件 | 作用 |
| --- | --- |
| `run_all.sh` | 全量回归：单/双雷达 ×（建图、定位+绑架、原地旋转）+ 双雷达标定，串行运行并打印汇总；单雷达场景自动加 `earlyspin`（建图另加 `spawnspin`） |
| `zt_run.sh` | 一键启动仿真 + `bringup sim.launch.py`，运行驱动脚本后清理进程组；环境变量 `LIDAR=single\|dual`（默认 dual），`CALIB=1` 时同时启动标定 |
| `zt_driver.py` | 按真值跟踪路线（补给区 → 中央 → 返回）；`mapping` 结束时调用 `save_map`；`localize` 跑完后用 `set_pose` 瞬移到跑圈终点 1.5 m 外、航向 45°（RMUL (1.25, 5.25)，RMUC (-9.5, -1.5)）并等待恢复（局中恢复在 UWB 位置 2 m 内搜索，只有 RMUC 有 UWB，RMUL 绑架后不会恢复）；`spin` 原地慢转/快转；`calib` 依次停到 5 个拐角/中央点、每处 4 个朝向静止 6 s；第 4 个参数（`zt_run.sh` 第 6 个）为逗号分隔的标志，见下文[单雷达](#单雷达) |
| `zt_eval.py` | SE(2) 对齐真值与估计（参考样本几乎不动时改用首样本锚定），按阶段输出 RMSE / 最大误差 / 航向误差；`FALSE TRUST` 行统计定位自报可信但误差 > 0.5 m 的样本（错误恢复）；SUSPECT 时后端由里程计前推、LOST 时冻结在最后可信位姿（误差随行驶距离增长），故每个阶段另列 `trusted` / `untrusted` 统计；`--align YAW_DEG TX TY` 用固定对齐（取建图那次的 alignment） |
| `cloud_rate.py` | 不反序列化点云地统计话题频率与 RTF（`ros2 topic hz` 对大点云会严重低估） |

## 用法

`run_all.sh` 接受新的或空的输出目录，保留已有结果。脚本默认使用 ROS domain 87 和单独的 Gazebo partition。需要与其他回归并行时，请显式设置独立的 `ROS_DOMAIN_ID` 和 `SENTRY_GAZEBO_GEN`。

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
`zt_run.sh` 传递驱动的退出码（包括超时）；初始化失败时驱动返回非零。
退出码 0 仍需结合 `zt_eval.py` 的误差、可信状态及绑架恢复结果判断验收，不能等同于定位精度通过。

改动参数后先运行 `python3 tools/check_bringup_params.py`，确认 bringup 配置与各模块 example 的键一致。

## 单雷达

MID-360 侧装（URDF `mid360_joint` 俯仰约 90°），静止时只看到一侧，单雷达要转车补视野。`zt_driver.py` 的标志：

| 标志 | 作用 | 原因 |
| --- | --- | --- |
| `earlyspin` | 先静止 2 s，再以 0.3 rad/s 慢转直到后端定位（NORMAL/OBS_DEGRADED），然后转回航向 0 | 静止时 ZLIO 初始化最弱方向支撑只有 2–4.8（门槛 5），反复重来、可能几分钟不完成；转动后 ≤ 2.8 s 完成（6/6）。首次尝试的重力仍取自静止 IMU |
| `initspin` | ZLIO 初始化完成后才慢转 | 只补后端重定位视野，前端仍可能久等 |
| `spawnspin` | 建图时先在出生点慢转一圈再跑圈 | 侧装雷达各航向画面不同；地图在出生点缺其他航向的关键帧时，启动慢转会让重定位锁到对称位置（曾偏 5.5 m / 94°） |
| `spin` | 每个路点原地转一圈 | 单雷达建图不要用：建图最大误差升到约 27 cm |
| `kidnapspin` | 绑架后后端处于 SUSPECT/LOST 时以 0.3 rad/s 原地慢转（单双雷达都可用） | 原用于给运动门提供视角变化（运动门已于 2026-11-08 删除）；现在只给恢复增加视野 |

单雷达后端读 `bringup/config/sim/backend_single.yaml`（launch 按 `lidar` 选择），与 `backend.yaml` 的差异写在该文件开头。

## 结果解释

每个场景的 `result.json` 记录驱动结果。评估时检查真值、后端可信状态、定位误差及绑架后的时间区间。RMUC 场地可提供模拟 UWB，RMUL 场地上的局中恢复会等待 UWB 观测。跨前端代次的输出、重置服务和对应参数见[定位系统](../../src/localization/README.md)。
