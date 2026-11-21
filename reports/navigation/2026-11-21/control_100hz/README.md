# 默认 yaw 前视时长与 100 Hz 控制链验证

日期：2026-11-21。环境：Ubuntu 24.04 / ROS 2 Jazzy，WSL ubuntu24。全部运行使用隔离 ROS domain、测试地形和确定性全向运动学底盘；串口连接 Linux PTY，不连接真车。

## 实现

- `chassis_comm` 默认 `yaw_mode=twist`、`yaw_horizon=0.1 s`，从最终 `/cmd_vel` 的 `angular.z` 计算当前机体系的相对目标角。发送周期为 0.01 s，前视时长独立于该周期；重复发送不累加角度。
- `controller_frequency`、`behavior_server.cycle_frequency`、`velocity_smoother.smoothing_frequency`、`cmd_gate.rate`、串口 `rate` 均为 100 Hz。局部代价图更新为 100 Hz，BT 周期原本就是 10 ms。碰撞监控随输入处理。
- MPPI 保留 `model_dt=0.05 s`、`time_steps=40`，预测范围保持 2 s；Jazzy 允许控制周期小于模型时间步，不开启控制序列平移。没有为提高控制频率缩短预测范围或减少轨迹采样数。
- 局部代价图用于 RViz 的发布仍为 20 Hz，全局代价图、重规划、传感器及定位频率独立于控制链。
- 串口写超时默认 5 ms；按 17 字节、8N1、指定发送频率检查波特率容量，100 Hz 至少需要 17000 bit/s。实际波特率仍须与下位机一致。

## 结果

构建 `nav_safety`、`chassis_comm`、`nav_bringup` 成功。通信 67 项 pytest 用例及安全门 9 项 gtest 用例通过；colcon 汇总分别另含 4 个、1 个包装条目。

`control_rate.py` 启动真实 Nav2、运动学底盘和默认参数的串口发送节点，读取各节点实际参数，然后在同一段持续导航窗口内统计命令消息和通过 CRC 解析的串口帧。窗口长度为 3.000051339 s。

| 环节 | 消息/帧数 | 实测 Hz | P95 接收间隔 ms | 最大接收间隔 ms |
|---|---:|---:|---:|---:|
| 控制器 `/cmd_vel_nav` | 300 | 99.9983 | 10.461 | 11.064 |
| 平滑 `/cmd_vel_smoothed` | 300 | 99.9983 | 10.232 | 11.110 |
| 碰撞监控 `/cmd_vel_collision_checked` | 300 | 99.9983 | 10.465 | 11.619 |
| 命令门 `/cmd_vel` | 300 | 99.9983 | 10.374 | 10.674 |
| PTY 解出的串口帧 | 300 | 99.9983 | 10.140 | 10.464 |

运行参数服务确认控制器、恢复行为、平滑、命令门、串口及局部地图更新均配置为 100 Hz，串口 `yaw_horizon=0.1 s`。该次目标正常到达。详见 [频率结果](rate_result.json)。

另有两套保留正式安全链的回归全部通过：

- 冒烟 8/8：生命周期、BT、地形过滤、速度限幅、障碍标记/清除、碰撞减速、观测断流停车，见 [冒烟结果](smoke_result.json)。
- 导航闭环 13/13：平地位置与朝向、隧道正反向、洞内目标、洞内恢复保护、障碍与续行、取消、抢占、定位/观测门及多目标坡道，见 [闭环结果](accept_result.json)。

## 复现

在工作区加载 ROS 和安装环境后运行：

```bash
MAKEFLAGS=-j4 colcon build --base-paths src \
  --packages-select chassis_comm nav_bringup nav_safety --symlink-install \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --base-paths src --packages-select chassis_comm nav_safety \
  --return-code-on-test-failure
ROS_DOMAIN_ID=198 python3 tools/nav_regress/control_rate.py --out /tmp/nav_control_rate
ROS_DOMAIN_ID=185 python3 tools/nav_regress/framework_smoke.py --out /tmp/nav_100hz_smoke
ROS_DOMAIN_ID=186 python3 tools/nav_regress/framework_accept.py --out /tmp/nav_100hz_accept
```

本次原始运行目录：`/tmp/nav_control_100hz_20261121_01`、`/tmp/nav_100hz_smoke_20261121`、`/tmp/nav_100hz_accept_20261121`。目录内有进程日志、fixture 和轨迹；摘要 JSON 保存在本报告目录。

## 范围与限制

频率数据来自接收端时间，串口为 PTY 读取，不是物理 UART 逻辑分析仪测量。统计窗口验证平均频率，接收间隔已有调度抖动，不能证明每个 10 ms 截止时间都满足；运行也未加入真实 Gazebo/定位/传感器负载。该次配置保留 MPPI 的控制周期小于模型时间步警告，fixture 启动日志另有 Smac inflation 提示，均未阻止激活、到点或上述回归。

运动学底盘执行最终 Twist，未模拟下位机相对角度闭环。`yaw = angular.z × 0.1 s` 仅定义前视目标，实际角速度响应和真车 100 Hz 稳定性需要实机确认。过洞机械动作自动触发阶段、真实地形/物理通行以及定位既有能力边界仍按导航进度管理。
