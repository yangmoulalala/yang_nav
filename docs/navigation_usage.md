# 导航使用指南

适用环境：WSL Ubuntu 24.04、ROS 2 Jazzy；实际项目位于 `/home/yang/yang_nav`。以下 Bash 命令均在 WSL 执行，除非单独标为 PowerShell。更新日期：2026-11-13。

本文包含日常启动、导航操作、排查和回归命令。实现见 [导航框架](navigation.md)，完成边界见 [导航进度](nav_plan.md)。当前主要软件链已验证，但真实地图整形仍为 8/12，缺净高、坡道物理通行和全场/真车验收尚未完成。

## 1. 进入项目与准备环境

Windows PowerShell 打开项目终端：

```powershell
wsl -d ubuntu24 --cd /home/yang/yang_nav
```

每个新开的 WSL 操作终端都需要加载环境；发送目标、查询状态的终端必须与启动终端使用同一个 `ROS_DOMAIN_ID`：

```bash
cd /home/yang/yang_nav
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=78
```

未设置 domain 时 ROS 使用 0；本文的手动仿真示例统一使用 78。回归工具使用其他 domain，见第 7 节。只有 `start_navigation.sh` 和仿真启动脚本会自行加载环境，普通 `ros2` 命令不会。

已有工作区更新导航代码后构建：

```bash
cd /home/yang/yang_nav
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav_behaviors nav_bringup nav_safety nav_terrain
source install/setup.bash
```

首次构建完整仿真环境时，需要同时构建仿真和定位依赖：

```bash
colcon build --packages-up-to nav_bringup bringup sentry_gazebo
source install/setup.bash
```

上述命令要求 ROS/Nav2、Gazebo 和定位依赖已安装；定位准备见 [话题和使用方式](topics_and_usage.md)。启动读取 `install/` 中的 launch、BT 和 YAML；修改源配置后应重建对应包，并重启导航使其生效。

## 2. 选择启动方式

| 场景 | 入口 | 启动范围 |
|---|---|---|
| 已有定位、TF、观测 | `tools/start_navigation.sh` | 导航、地形障碍过滤、命令门，可选 RViz |
| 手动完整仿真 | `tools/nav_regress/sim_rviz.sh` | Gazebo + 定位 + 导航 + RViz |
| 自动运行指定目标 | `tools/nav_regress/run_nav.sh` | 完整仿真，顺序执行目标并保存日志 |
| 只验证软件链 | 第 7 节回归工具 | 自动 fixture，不依赖当前真实地图完整可通行 |

### 2.1 已有定位时，仅启动导航

启动前应有 `map → lio_odom → base_link` TF、`/localization/odometry`、`/frontend/frame` 和定位可信状态；仿真还需要 `/clock`。导航 launch 自身不会启动 Gazebo、定位或传感器驱动。

```bash
# 前台运行；Ctrl-C 停止这次导航 launch
ROS_DOMAIN_ID=78 bash tools/start_navigation.sh rviz:=true

# 只打印将执行的命令，不启动节点
ROS_DOMAIN_ID=78 bash tools/start_navigation.sh --dry-run rviz:=true

# 指定地形和参数文件；相对路径从项目根目录解析
ROS_DOMAIN_ID=78 bash tools/start_navigation.sh \
  terrain_dir:=maps/sim/terrain \
  params_file:=src/nav_bringup/params/sim/nav2.yaml \
  use_sim_time:=true rviz:=true

# 查看脚本用法、底层 launch 参数
bash tools/start_navigation.sh --help
ros2 launch nav_bringup navigation.launch.py --show-args
```

启动脚本源码：[tools/start_navigation.sh](../tools/start_navigation.sh)。

该脚本根据自身位置找到项目，加载 ROS 与工作区环境，然后以 `exec` 前台运行 launch。脚本可从其他目录通过绝对路径调用；`--dry-run` 只检查环境并展示命令，不检查定位、地图语义或运行健康。

底层等价命令：

```bash
ros2 launch nav_bringup navigation.launch.py \
  terrain_dir:=/home/yang/yang_nav/maps/sim/terrain \
  use_sim_time:=true require_trusted:=true rviz:=true
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `terrain_dir` | 脚本：当前项目 `maps/sim/terrain`；原生 launch：`~/yang_nav/maps/sim` | 可传导出目录或其父目录；父目录中有 `terrain/nav_map.yaml` 时自动进入 `terrain` |
| `params_file` | 空，使用安装包的 `params/sim/nav2.yaml` | 参数文件路径 |
| `use_sim_time` | `true` | Gazebo/有时钟的回放用 true；实时时钟用 false |
| `require_trusted` | `true` | 定位未报告可信时命令门停车；日常使用保持 true |
| `rviz` | `false` | 打开导航 RViz |

地形目录必须包含 `nav_map.yaml`、`speed_mask.yaml`、`zones.yaml`、`terrain.msgpack`，以及两个 YAML 引用的图像。`terrain.json` 还用于现有目标坐标转换工具。文件存在不等于地形数据通过验收。

使用实时时钟的已有系统可传 `use_sim_time:=false`，但当前默认仍是仿真参数；这一个开关不能替代真车参数标定与验收。真车定位启动见 [定位指南](topics_and_usage.md)。

### 2.2 手动完整仿真：推荐日常调试入口

```bash
cd /home/yang/yang_nav
ROS_DOMAIN_ID=78 OUT=/tmp/nav_manual bash tools/nav_regress/sim_rviz.sh

# 需要同时显示 Gazebo 窗口时
ROS_DOMAIN_ID=78 GUI=true OUT=/tmp/nav_manual_gui bash tools/nav_regress/sim_rviz.sh
```

脚本启动 RMUC Gazebo、双雷达先验地图定位，等待定位可信和 TF，再启动导航及 RViz。默认无 Gazebo GUI，RViz 始终打开；无目标时默认保留 86400 秒，可用 `HOLD` 改时长。运行期间可在另一终端发目标，或在 RViz 使用 **Nav2 Goal**。

脚本启动前会检查现有导航/仿真进程，发现冲突就退出；即使 domain 不同也可能被这项进程检查拒绝。先在原终端正常停止冲突运行。Ctrl-C 时脚本清理本次创建的进程组。不要同时启动该脚本和另一套完整栈。

日志默认位于 `/tmp/nav_manual`：`sim.log`、`loc.log`、`nav.log`、`rviz.log`、`truth.log`。指定每次不同的 `OUT` 便于区分结果。

### 2.3 分终端启动，便于单独检查

三个终端分别加载第 1 节环境，使用相同 domain：

```bash
# 终端 A：Gazebo，按需打开 GUI
ros2 launch sentry_gazebo sim.launch.py arena:=rmuc uwb:=true gui:=false

# 终端 B：先验地图定位；prior_map 是定位地图目录
ros2 launch bringup sim.launch.py \
  lidar:=dual mode:=localization prior_map:=/home/yang/yang_nav/maps/sim

# 终端 C：确认可信状态和 TF 后再启动导航
ros2 topic echo --once /backend_node/localization_state
ros2 run tf2_ros tf2_echo map base_link
# 看到连续有效 TF 后 Ctrl-C 结束 tf2_echo，再运行：
bash tools/start_navigation.sh rviz:=true
```

此处定位的 `prior_map=maps/sim` 与导航的 `terrain_dir=maps/sim/terrain` 用途不同。不要互换两个参数。

## 3. 发送目标、取消与停止

### 3.1 RViz

固定坐标系为 `map`；使用 **Nav2 Goal** 在可通行位置按住并拖出目标航向。当前布局也有 `/goal_pose` 工具；日常优先使用 Nav2 的目标工具，以 action 反馈判断结果。定位由本项目定位链提供，不依靠 AMCL 的初始位姿流程。

### 3.2 命令行单目标

下面坐标仅是格式示例，发送前替换为当前地图内确认可通行的位置。ROS action 使用 `map` 坐标、米和四元数；航向为 0 时 `z=0,w=1`。

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}" \
  --feedback
```

已有仿真还可使用目标脚本：

```bash
# map 坐标，x/y 单位米，第三个位置参数 yaw 单位度
python3 tools/nav_regress/goal.py 1.0 1.0 90 \
  --frame map --nav-dir maps/sim/terrain --timeout 180

# Gazebo world/场地坐标；依赖 terrain.json 中的 registration
python3 tools/nav_regress/goal.py 1.0 1.0 90 \
  --frame world --nav-dir maps/sim/terrain --timeout 180
```

`goal.py` 默认 `--frame world`，固定 `use_sim_time=true`，即使指定 map 也会读取 `terrain.json`。它适用于已有仿真栈；实时时钟系统使用上面的 action 命令或 RViz。超时后的取消请求不能代替确认动作停止。

### 3.3 多目标

Jazzy 的 `NavigateThroughPoses` 接收 `PoseStamped[]`，会走项目的多目标 BT。以下为两个 map 点的格式示例：

```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [
    {header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}
  ]}" --feedback
```

### 3.4 取消当前导航与关闭

```bash
# 默认全零 goal_info 表示取消该 action server 上全部活动目标
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
ros2 service call /navigate_through_poses/_action/cancel_goal action_msgs/srv/CancelGoal "{}"

# 检查最终速度是否回零
ros2 topic echo /cmd_vel
```

取消后检查 action 结果和最终速度。仅在发送目标的终端按 Ctrl-C 可能只退出客户端，不代表取消服务端任务。关闭手动分终端栈时先取消目标并确认停车，再停止导航、定位、Gazebo；一键仿真脚本则在其终端 Ctrl-C。

## 4. 常用状态检查命令

以下命令大多持续输出，查看后用 Ctrl-C 结束。它们应在与导航相同的 domain 执行。

```bash
# 定位可信状态、仿真时钟、TF 和里程计
ros2 topic echo --once /backend_node/localization_state
ros2 topic hz /clock
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /localization/odometry

# 导航生命周期与 action
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /collision_monitor
ros2 action list -t
ros2 node list

# 实时观测及过滤结果
ros2 topic hz /frontend/frame
ros2 topic hz /nav/obstacles
ros2 topic info -v /nav/obstacles

# 区域模式、跟踪 phase 和速度限制
ros2 topic echo /nav_mode
ros2 topic echo /nav_zone_state
ros2 topic echo /speed_limit

# 沿命令链定位在哪一层停车
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel_collision_checked
ros2 topic echo /cmd_vel
```

生命周期节点应为 `active`。区域 phase 的 `align/traverse/exit` 会暂停重规划，`approach` 允许重规划；恢复区域内只清图是保护逻辑。完整语义见 [导航框架](navigation.md)。

## 5. 常见故障排查

| 现象 | 先检查什么 |
|---|---|
| 找不到 ROS 包或命令 | 是否加载 `/opt/ros/jazzy/setup.bash` 和当前项目 `install/setup.bash`；是否已构建 |
| 其他终端看不到节点 | `echo "$ROS_DOMAIN_ID"` 是否一致；当前终端是否加载工作区 |
| `use_sim_time=true` 后节点等待/不更新 | `/clock` 是否在发布，Gazebo 是否暂停；实时时钟系统应使用 false |
| 生命周期未激活 | `nav.log` 中首次错误、TF、里程计、地形文件和图像路径；先修首个失败节点 |
| 接受目标但最终速度一直为零 | 定位是否可信且新鲜，`/frontend/frame`、`/nav/obstacles` 是否持续更新；逐段看命令链 |
| 障碍消失但仍停车 | 观测是否持续、自由地面是否被重新观测，costmap 障碍是否实际清除 |
| 区域内不自转/不倒车恢复 | 这是 `ZoneRecovery` 的区域保护；检查物理区域位置和 phase，不要绕过保护 |
| 隧道路径整形失败 | 查看 smoother 报错和完整足迹碰撞；目前 tunnel_2/tunnel_5 双向仍有已知失败 |
| world 目标位置不对 | 检查 `--frame` 与 `terrain.json.registration`；map 点必须显式 `--frame map` |
| 修改配置却不生效 | 是否改的是实际使用的文件；源文件修改后重建包并重启，或显式传 `params_file` |
| 一键仿真提示已有栈 | 在原终端关闭冲突运行；不要用全局杀 ROS 进程的方式清理 |

`require_trusted=false` 也不会取消观测新鲜度与碰撞保护；排查时应先找出输入或命令链的问题。当前真实地形缺净高，不能用“成功启动”替代数据检查。

## 6. 自动目标与运行日志

下面的目标坐标仍需替换为可通行位置。每个带引号的参数是一个 `X Y YAW_DEG` 目标，默认 **world** 坐标：

```bash
ROS_DOMAIN_ID=78 OUT=/tmp/nav_run_demo RVIZ=true \
  bash tools/nav_regress/run_nav.sh "1.0 1.0 0" "2.0 1.0 90"

# 输入 map 坐标时显式指定；MAP 是定位地图，TERRAIN 是导航地形
ROS_DOMAIN_ID=78 GOAL_FRAME=map MAP=/home/yang/yang_nav/maps/sim \
  TERRAIN=/home/yang/yang_nav/maps/sim/terrain OUT=/tmp/nav_run_map \
  bash tools/nav_regress/run_nav.sh "1.0 1.0 0"
```

脚本执行后默认关闭本次栈；`HOLD=60` 可在结束后多保留 60 秒。`GOAL_TIMEOUT` 控制单目标超时，默认 180 秒。`MAP`、`TERRAIN` 使用绝对路径更便于从不同目录运行；相对路径受调用目录影响。

按需记录过滤障碍、代价地图及路径：

```bash
ROS_DOMAIN_ID=78 OUT=/tmp/nav_record \
  OBS_REC=/tmp/nav_record/obstacles.txt \
  CM_REC=/tmp/nav_record/costmaps \
  PLAN_REC=/tmp/nav_record/plans.txt \
  bash tools/nav_regress/sim_rviz.sh

# 另一终端查看日志
tail -f /tmp/nav_record/nav.log
```

`obstacles.txt`、`plans.txt` 是带注释表头的空格分隔文本，`costmaps/` 保存 NPZ 快照与 `meta.txt`。`truth.log` 是 Gazebo ground truth，`goals.log` 是目标结果。需要留档时保存对应结果、配置和必要失败日志；不要把所有重复运行日志加入源码仓库。

## 7. 构建后回归命令

先加载环境。以下工具自行启动测试栈，应单独运行；每项使用自己的 domain 和输出目录：

```bash
# 安装/插件/BT 静态接线
bash tools/nav_regress/verify_nav_system.sh

# 当前地形完整性：真实 maps/sim/terrain 目前会因缺净高而失败
python3 tools/nav_regress/terrain_check.py maps/sim/terrain

# 隔离 fixture 的软件验证
ROS_DOMAIN_ID=85 python3 tools/nav_regress/framework_smoke.py --out /tmp/nav_smoke
ROS_DOMAIN_ID=86 python3 tools/nav_regress/framework_accept.py --out /tmp/nav_accept
ROS_DOMAIN_ID=87 python3 tools/nav_regress/recovery_ladder.py --out /tmp/nav_recovery

# 真实地图各隧道双向规划 + zone_shaper；当前历史结果 8/12
ROS_DOMAIN_ID=77 SMOOTHER=zone_shaper OUT=/tmp/nav_plans \
  bash tools/nav_regress/plan_tunnels.sh maps/sim/terrain

# 当前三个有单测的导航包
colcon test --packages-select nav_behaviors nav_safety nav_terrain \
  --event-handlers console_direct+
```

fixture 工具主要结果保存在各自输出目录的 `result.json` 和 `navigation.log`；完整闭环还保存 `trace.jsonl`。当前 12 个测试目标共 108 个用例，勿用含旧 XML 的 colcon 汇总数字代替当前用例数。软件 fixture 通过不代表真实地图或物理通行通过。

更多诊断工具见 [回归工具说明](../tools/nav_regress/README.md)，本轮代码清理证据见 [验证记录](../reports/navigation/2026-11-13/code_cleanup/README.md)。
