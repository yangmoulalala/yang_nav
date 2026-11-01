# 定位仿真回归测试

在 Gazebo 仿真中用真值闭环驱动小车，记录 Gazebo 真值与 TF `map → base_link`，评估建图、回环、先验定位、绑架重定位、
原地旋转和双雷达标定，单雷达与双雷达均可测。

| 文件 | 作用 |
| --- | --- |
| `run_all.sh` | 全量回归：单/双雷达 ×（建图、定位+绑架、原地旋转）+ 双雷达标定，串行运行并打印汇总 |
| `zt_run.sh` | 一键启动仿真 + `bringup sim.launch.py`，运行驱动脚本后清理进程组；环境变量 `LIDAR=single\|dual`（默认 dual），`CALIB=1` 时同时启动标定 |
| `zt_driver.py` | 按真值跟踪路线（补给区 → 中央 → 返回）；`mapping` 结束时调用 `save_map`；`localize` 跑完后用 `set_pose` 瞬移到 (6.0, 1.5, 90°) 并等待恢复；`spin` 原地慢转/快转；`calib` 依次停到 5 个拐角/中央点、每处 4 个朝向静止 6 s |
| `zt_eval.py` | SE(2) 对齐真值与估计（参考样本几乎不动时改用首样本锚定），按阶段输出 RMSE / 最大误差 / 航向误差；`FALSE TRUST` 行统计定位自报可信但误差 > 0.5 m 的样本（错误恢复） |
| `cloud_rate.py` | 不反序列化点云地统计话题频率与 RTF（`ros2 topic hz` 对大点云会严重低估） |

## 用法

```bash
cd ~/yang_nav
# 全量回归（约 30–40 分钟），结果在 /tmp/zt_all/<场景>/，最后打印汇总
tools/sim_tests/run_all.sh /tmp/zt_all

# 单雷达建图
LIDAR=single tools/sim_tests/zt_run.sh mapping /tmp/zt/single ~/yang_nav/maps/sim_single 2

# 双雷达标定（结果写 /tmp/zt/calib/extrinsic.yaml）
CALIB=1 tools/sim_tests/zt_run.sh mapping /tmp/zt/calib /tmp/zt/calibmap 0

# 1. 建图 2 圈并保存地图
tools/sim_tests/zt_run.sh mapping /tmp/zt/mapping ~/yang_nav/maps/sim_test 2
python3 tools/sim_tests/zt_eval.py /tmp/zt/mapping/result.json
grep -c 'loop scan-to-map accepted' /tmp/zt/mapping/bringup.log

# 2. 用该地图定位 1 圈，然后绑架
tools/sim_tests/zt_run.sh localization /tmp/zt/loc ~/yang_nav/maps/sim_test 1
python3 tools/sim_tests/zt_eval.py /tmp/zt/loc/result.json
```

输出目录里有 `sim.log`、`bringup.log`、`driver.log` 和 `result.json`；标定场景另有 `calib.log` 与 `extrinsic.yaml`。

改动参数后先运行 `python3 tools/check_bringup_params.py`，确认 bringup 配置与各模块 example 的键一致。

## 参考结果（2026-10-31，RTX 4060 Ti，RTF 1.0，双雷达，参数重构后）

| 场景 | 结果 |
| --- | --- |
| 建图 2 圈 | 回环 18 次全部接受；RMSE 1.5–1.9 cm，最大 4.7 cm，航向 ≤ 0.15°；`save_map` 成功 |
| 先验定位 1 圈 | RMSE 3.0 cm，最大 8.0 cm，航向 ≤ 0.85°；行驶中偶发 `SUSPECT`（observation model invalid）后约 0.4–1 s 恢复 |
| 原地旋转（慢 + 快） | 最大 4.4 cm，航向 ≤ 0.48° |
| 双雷达标定 | 4 帧，平移误差约 1 cm（std 1.4 cm），旋转 < 1°；采集过程定位 RMSE 4.8–6.3 cm |
| 绑架 | 最终恢复，但 `FALSE TRUST` 常见：曾接受错误位姿（3.5–7 m）并报可信 6–16 s（场地中心对称）；`recovery.confirmations: 3` 无改善 |
| 单雷达 | 前端在出生点初始化失败（最弱方向支撑不足），待查 |

## 注意

- 需要 `gz.transport13`、`gz.msgs10` Python 绑定（Gazebo Harmonic 自带）。
- 瞬移不产生 IMU 运动，前端感知不到，比真实碰撞更苛刻。
- 必须设置 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`（脚本已设置），否则雷达掉到约 3 Hz。
