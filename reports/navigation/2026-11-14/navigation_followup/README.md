# 2026-11-14 仿真导航与真车准备后续

> 归档说明（2026-11-14）：本目录保留结论、脚本和精简证据。大型原始数据已转入本机 `backup/report-evidence-2026-11-14.tar.zst`，源码快照、编译产物和重复中间记录已清理；清单与恢复方法见 `reports/maintenance/2026-11-14/commit_cleanup/README.md`（路径均从仓库根目录解析）。下文的 before/after、trace 和输入文件路径按原实验记录保留，现存状态以清单为准。

本记录接续用户在仿真中反复观察到的隧道 yaw 摇摆、洞口停顿、离洞不走/定位漂移，以及连续目标绕行问题。此次同时完成动态障碍闭环测试、真车入口检查和手动建图录包准备。代码尚未提交；保留了工作区原有暂存与未提交修改。

## 结论

- 真实 Gazebo 定位与 Nav2 连续目标：最终三目标运行 **3/3 成功，返回码均为 0**，所有 ≥2 s 的平移静止采样均发生在目标 15 cm 位置容差内；洞口/中途路线没有此类停车。连续 TF 最大组合步长 6.67 cm，没有超过 30 cm 的组合跳变。
- 动态障碍回归：**6/6 通过**，覆盖横穿行人、固定挡路、占用目标安全接近并等待、观测中断制动、目标取消和抢占。使用真实 Nav2 节点与最终 `/cmd_vel` 的确定性全向运动学夹具，不是 Gazebo 碰撞物理验收。
- 导航软件框架：**13/13 通过**，包括隧道双向通行、区域内恢复限制、取消、抢占、多个目标及观测/定位安全门，同样不代表轮胎打滑、实车制动或真实传感器验收。
- 仿真静止定位重置跨代 TF：**通过**，epoch 0→1，重置时组合车体位移 1.23 cm，最大组合步长 3.02 cm。
- 真车建图入口与录包工具已检查；真实底盘驱动仍不在仓库，真车自主定位恢复也有无 UWB 限制。不能因此宣布真车运动或自主导航已经验收。

## 隧道停顿、航向变化和出口不动

不是单独一个 odometry 噪声问题。复测把相关现象拆成了规划几何、运动安全门和定位 TF 三类问题：

1. **隧道出口与相邻坡道之间有航向不连续。** 隧道 EXIT 判断与控制器使用的出口方向/过渡方向不一致；整形器也会跳过前一段完整 EXIT，导致相邻坡道搜索从坡体内部开始，把方向判成未知，再在边界重新锁向。现在统一使用出口 heading/blend/flip 判据，保留已规划路径在 zone 边界的进入航向，并从上一段完整直线退出余量之后开始生成下一段坡道入口。已加相邻隧道/坡道回归测试。
2. **旧出口后缀没有按车体扫掠检查重整。** 新规划段从洞口附近重接时，现在用与现有路径整形相同的完整车体轮廓和碰撞检查，重建向外退出路径。斜向通过时同时保留洞口进出口横向偏置，避免路径将车身挤向一侧墙。
3. **狭窄通道可行侧移被控制器抹掉。** 过去前进和侧移一起撞上角点后会连续缩放整条命令，最后车停住，即使单独横移可离开角点。现在在普通安全速度命令失败后，控制器尝试用同一碰撞/制动检查验证的侧向对齐命令。路径整形在 2.5 cm 栅格上细搜约 5 mm 横移步长，规划 padding 为 2.25 cm；局部/全局 costmap 和控制器仍按真实轮廓另加 1 cm padding，没有缩小物理 footprint。
4. **观测与碰撞监控存在时序余量问题。** 地形障碍观测经帧窗口与 TF 等待，最坏约 0.45 s；过去 0.5 s 源超时会让来源在有效/失效间抖动而停车。现在 source timeout 为 1 s，云数据已在 map frame，碰撞监控用最新 map→base TF；断流停车和完整 approach footprint 检查仍开启。
5. **重置时两条 TF 曾跨代拼接。** 独立进程分别发布 `map→lio_odom` 与 `lio_odom→base_link`，前端重置时接收方会暂时组合新旧代次。仅调整发布顺序不足以消除竞态。组合 bringup 改为后端在同一 `TFMessage`、同一时间戳发布两条变换，并限制旧 epoch/旧时间戳发布；前端配置关闭重复 TF。定位参考子图切换匹配失败时，后端还会在原搜索半径内用完全相同门限重试上一个参考子图。

### Gazebo 连续目标实测

使用 `tools/nav_regress/run_nav.sh`、RMUC 场地、双雷达仿真定位与真实 Nav2 栈，目标使用 **map frame**。最新记录 `final_gazebo_three_goals/`：

| 目标 (x, y, yaw) | 结果 | 时间 | 采样路径长度 | ≥2 s 静止区间 |
|---|---:|---:|---:|---:|
| (8.66, -3.61, 0°) | 成功，error_code 0 | 24.2 s | 12.97 m | 无 |
| (12.78, -3.86, 0°) | 成功，error_code 0 | 23.7 s | 7.78 m | 2 s 终点 settling，在目标 5.8 cm 内 |
| (8.31, -3.65, 0°) | 成功，error_code 0 | 19.7 s | 7.72 m | 无 |

静止检测依据约 1 Hz 的地图位姿采样、速度阈值 0.04 m/s 和持续 2 秒门槛，并按距目标的 15 cm 位置容差区分行程停止与终点 settling；不代表没有小于 2 秒的降速。最新定位记录包含 25,887 条配对 TF，最大接收侧组合步长 0.06673 m，未发现超过 0.3 m 的车体组合跳变。另一轮带 costmap 记录的物理路线保存 183 个 costmap 快照，没有原点跳变。

有一个中间运行在第二目标出现 **52.1 s** 静止、77.5 s 才完成；之后的横向对齐控制修复让连续三目标在约 20–24 s 完成，洞口及离开目标区域前没有 ≥2 s 的静止。最终第二目标抵达位置后有 2 s 平移 settling，同时控制器收敛终点航向，然后成功返回。更早的一轮第三目标因相邻 zone heading 突跳，整段扫掠 footprint 碰撞检查失败；后续修复继承边界 heading 并以相同静态路线重验成功。测试结果没有删去这些中间失败，比较证据在 `before_lateral_alignment/` 与 `continued_before_heading_join/`。

最初几轮 Gazebo action 虽返回成功状态，但后两目标还携带中途恢复留下的旧 `error_code=503`。`ZoneRecovery` 现在只在最终导航成功时清理已恢复的子 action 错误码；失败任务保留诊断码。单元测试和框架闭环通过；之后再次执行完整三目标 Gazebo 运行，三个目标均返回 `SUCCEEDED / error_code 0`。

## 目标二绕行与坡道方向

坡道不是全局单向封锁。`one_way_dir` 只决定哪个方向采用专门的上坡跟踪与路径整形；下坡仍交给正常全向规划/MPPI。静态代价图中从第二目标返回和从第三目标出发都找到南侧同一合法通道，规划长度分别为 **7.603 m、7.619 m、7.466 m**。所以第三目标能走这条坡，不说明它是“反向禁行”；出发位姿、航向、路径入口整形和动态障碍证据都会改变候选路径。

直穿目标二所在高度的一段路线会经过静态 lethal 格，合法的短路线绕过中央障碍并走 tunnel_2。此前长北绕行还会被短时障碍观测及整形拒绝后的重规划放大。现在只有候选路线与当前承诺路径仍相邻、路线超过两倍且多绕至少 5 m 时，才等待 1.5 s 以过滤短时障碍抖动；障碍持续存在时仍允许安全改道。该逻辑不会绕过 live collision 检查，也不保证真实地图任意路线最优。静态 route JSON 保存在 `static_routes/`。

## 动态障碍和被占目标

`dynamic_acceptance/result.json` 的六项检查全通过。最近的静态挡路与占用目标情况均保持正 clearance。占用目标的测试过程是：车辆接近被动态障碍标记的原目标，在安全距离处到达临时 approach waypoint，但 NavigateToPose 保持 pending；观测断流时输出停止，障碍消失后约 1.52 s 抵达原始目标。持续占用时可安全取消，新目标能抢占 pending 目标。静态墙上的永久占用目标仍然报错，不会被当成动态目标而盲目驶入。

复测最初揭示 approach waypoint 离障碍过近：控制器在容差范围内提前报告临时 waypoint 成功，轨迹侵入了约 2.6 mm 的 padded footprint，且反复重规划可能切换到障碍另一侧。现在规划候选点额外按 0.20 m 扩展 footprint 检查，为控制器终点容差留出空间，并保持仍安全的 waypoint，避免每次重规划翻边。六项动态测试随即通过，最近占用物到 pad 后车体间隔约 0.48 m。测试几何使用矩形车体和方形障碍逐姿态计算，不再使用外接圆近似。

## Gazebo 动态实体补测（平地）

在 RMUC Gazebo 完整导航栈中补做了方块实体测试：被占目标的 action 被接受并保持 EXECUTING，行为树进入 `KeepOccupiedGoalPending`，障碍移开后原 action 成功（error_code 0）；另一轮低速横向移动障碍路径中，96 帧 `/nav/obstacles` 点云持续跟踪实体，规划绕行后返回目标并成功（12.0 s，error_code 0）。这两轮补足真实 Gazebo 传感器到 Nav2 的闭环证据，但不是动态人群或高速行人验收；占用目标轮的方块发生漂移，且横穿轮未导出实体碰撞尺寸，因此不宣称已测定最小 footprint 净空。结果和限定见 `gazebo_dynamic_flat_retest/README.md`。

## 接收起点位于障碍中

`StartAwarePlanner` 在完整 costmap 中搜索能降低 footprint 接触/重叠的短平移退出路径，再交回正常 Smac 规划和 zone shaper；不清除、屏蔽或篡改障碍格，不把原地旋转或倒车塞进区域内的恢复路径。若没有碰撞递减的安全退出，规划仍会失败。新增 0.20 m approach clearance 仅影响占用**目标**的安全等待点，不放宽普通路径的碰撞检查。

## 真车适配检查

定位已有 `bringup/real.launch.py` 与 MID-360、双雷达融合、URDF 关节 TF、ZLIO 和后端建图/定位配置；导航 profile 选真车地图和真实时间，和仿真共用 Nav2 规划、控制、footprint 和安全门。`python3 tools/check_bringup_params.py` 通过，覆盖 sim/real 单/双雷达配置键、允许的参数差异和 paired TF 发布归属。

现有仿真/真车导航上限相同（MPPI x/y 1.5 m/s，普通最终限速 1.6 m/s、2.5 rad/s；洞参考 0.6 m/s、坡参考 1.4 m/s），车体 footprint 为 x=[-0.30,0.38]、y=[-0.30,0.30] m。真车 IMU frame、尺度、频率、噪声和 LiDAR 范围按硬件分别配置，不能照抄仿真传感器数值。

仓库**没有底盘通信/轮速驱动**。外部驱动必须把最终 `/cmd_vel` 的车体速度转换为轮速并实现通信超时停车；方向定义、轮廓、最大速度和制动距离都须实车确认。目前也没有验证真实 MID-360/L2 驱动的逐点时间与 frame。另一个明确边界是：real 关闭 UWB，而后端当前 SUSPECT/LOST 自动恢复需要新鲜 UWB 位置先验。mapping 模式建图不依赖先前地图定位，可供人工建图；真车自主运行遇到失定位后的无 UWB 自动恢复能力尚未验证，也未宣称完成。

## 手动建图录包

仓库新增 `tools/record_real_mapping.py` / `.sh`。在 WSL 仓库根目录先启动 recorder，再另开终端启动 real mapping；录包器不启动运动节点、不发布运动命令。启动命令与双雷达可选参数如下：

```bash
bash tools/record_real_mapping.sh --note 'MID360单雷达，人手操控建图，云台固定'

# 第二终端
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bringup real.launch.py lidar:=single mode:=mapping \\
  map_dir:="$PWD/maps/real/session_001" rviz:=true
```

人工建图完成后，保持建图节点运行并保存地图，然后停止录包器并等待它 finalize：

```bash
ros2 service call /save_map std_srvs/srv/Trigger '{}'
# 确认保存成功后，在 recorder 终端按 Ctrl-C，等到 Saved session
```

双雷达时需给 `real.launch.py` 提供真实 L2 驱动路径，示例和回放安全说明在 [真车建图指南](../../../../docs/real_mapping.md)。录包内容包括 raw lidar/IMU、融合观测、定位/TF/session、URDF 与关节、各级速度命令、ROS 日志、版本 diff、参数和开始/结束 graph 快照；MCAP 每 120 s 分段。开始 graph 用于记录先存在的驱动；结束快照用于补齐之后才启动的建图节点。

MCAP 隔离 domain 烟测通过：260 条 IMU、86 帧点云，能回读录包前发出的 `/tf_static`、URDF 和原始消息时间戳。该烟测仅验证记录器。实车使用前应检查输出 `bag_info.txt` 的 raw 传感器话题/计数；若底盘状态或硬件时间另有话题，通过重复 `--topic /actual/topic` 加入。

## 验证与证据索引

- `approach_clearance_tests.log`：nav_behaviors 10/10 测试目标通过。
- `framework_acceptance/result.json`：真实 Nav2 + 全向运动学底盘闭环 13/13。
- `dynamic_acceptance/result.json`：真实 Nav2 + 全向运动学底盘动态障碍 6/6。
- `final_gazebo_three_goals/`：最终三目标 Gazebo 日志、位姿和 gzip 定位 trace；初始无停顿对照轮另存 `gazebo_no_midroute_stall/`。
- `paired_tf_reset/`：真实 Gazebo 静止定位重置前后 TF 连续性结果。
- `static_routes/`：目标二正向、反向和第三目标静态路线。
- `bag_smoke_final/`：录包器 MCAP、开始/结束 ROS graph 与元数据烟测。
- `dynamic_failed_approx_clearance/`、`before_lateral_alignment/`：失败与修复前对比记录；保留用于解释迭代。

完整范围和残余验收项见 [导航进度](../../../../docs/nav_plan.md)、[定位问题记录](../../../../docs/localization_issues.md) 与 [录包说明](../../../../docs/real_mapping.md)。
