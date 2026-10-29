# ZLIO 后端仿真回归测试

在 Gazebo 仿真中用真值闭环驱动小车，记录 Gazebo 真值与 TF `map → base_link`，评估建图、回环、先验定位和绑架重定位。

| 文件 | 作用 |
| --- | --- |
| `zt_run.sh` | 一键启动仿真 + `bringup sim.launch.py`，运行驱动脚本后清理进程组 |
| `zt_driver.py` | 按真值跟踪路线（补给区 → 中央 → 返回）；建图模式结束时调用 `save_map`；定位模式跑完后用 `set_pose` 瞬移到 (6.0, 1.5, 90°) 并等待恢复 |
| `zt_eval.py` | SE(2) 对齐真值与估计，按阶段输出 RMSE / 最大误差 / 航向误差，并列出状态变化和服务结果 |
| `cloud_rate.py` | 不反序列化点云地统计话题频率与 RTF（`ros2 topic hz` 对大点云会严重低估） |

## 用法

```bash
cd ~/yang_nav
# 1. 建图 2 圈并保存地图
tools/sim_tests/zt_run.sh mapping /tmp/zt/mapping ~/yang_nav/maps/sim_test 2
python3 tools/sim_tests/zt_eval.py /tmp/zt/mapping/result.json
grep -c 'loop scan-to-map accepted' /tmp/zt/mapping/bringup.log

# 2. 用该地图定位 1 圈，然后绑架
tools/sim_tests/zt_run.sh localization /tmp/zt/loc ~/yang_nav/maps/sim_test 1
python3 tools/sim_tests/zt_eval.py /tmp/zt/loc/result.json
```

输出目录里有 `sim.log`、`bringup.log`、`driver.log` 和 `result.json`。

## 参考结果（2026-10-29，RTX 4060 Ti，RTF 1.0，双雷达）

| 场景 | 结果 |
| --- | --- |
| 建图 2 圈 | 回环 18 次全部接受；RMSE 2.1 / 2.4 cm，最大 11.7 cm，航向 ≤ 0.21° |
| 先验定位 1 圈 | RMSE 3–5.6 cm，最大 11.6–28.5 cm；行驶中偶发 `SUSPECT`（observation model invalid）后约 0.2–0.4 s 恢复 |
| 绑架 | 最终恢复（误差 1.6–8 cm），但恢复过程中仍会接受错误位姿（场地中心对称，静止时同一错误匹配可通过连续确认） |

## 注意

- 需要 `gz.transport13`、`gz.msgs10` Python 绑定（Gazebo Harmonic 自带）。
- 瞬移不产生 IMU 运动，前端感知不到，比真实碰撞更苛刻。
- 必须设置 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`（脚本已设置），否则雷达掉到约 3 Hz。
