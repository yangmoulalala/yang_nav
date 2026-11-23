# yang_nav 使用手册

已有配套先验地图和地形包，按第 1、2 节启动导航。第一次到新场地，按第 3 节建图，在开始走图前按第 4 节录包，保存地图后按第 5、6 节优化和标注，最后回到第 2 节用新地图定位。

| 手里的数据 | 用在哪里 |
| --- | --- |
| 后端地图目录，含 `keyframes.txt`、`clouds/` 和描述子 | `prior_map` 加载定位地图，也是离线优化器的输入 |
| `*.surf.npz` | 地形编辑器的可编辑工程，保存面片、标注与配准 |
| 导出的地形目录，含 `terrain.msgpack`、`nav_map.yaml`、`zones.yaml` 等 | Nav2 的 `terrain_dir`，须与先验地图使用同一 `map` 坐标系 |
| 录包会话中的 `bag/`，含 `metadata.yaml` | `ros2 bag info` 和 `ros2 bag play` 的输入 |

## 1. 准备终端

Windows PowerShell 中运行 `wsl -d ubuntu24 -- bash` 进入 Bash。以下命令在 WSL 中执行。**每个新终端**先运行这段，所有参与同一次运行的终端使用相同的 `ROS_DOMAIN_ID`。示例值 `42` 可替换为车上已配置的值。仿真与真车同时使用时分别选用不同的 domain。

```bash
cd /home/yang/yang_nav
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
```

若 `install/setup.bash` 不存在，先按项目首页安装依赖并编译。加载成功后，`ros2 pkg prefix bringup` 和 `ros2 pkg prefix nav_terrain` 应指向本工作区 `install/` 下的包。 [依赖与构建命令](../README.md#构建与检查)

## 2. 已有地图，启动定位与导航

### 仿真：三个终端

以下是本机 RMUC、双雷达和 `maps/sim` 的组合。换场地时同时核对出生点、雷达配置和地图。每个终端执行第 1 节后，再分别运行对应的一条命令。

```bash
# 终端 A：保持 Gazebo 运行，提供传感器、机器人模型与 /clock
ros2 launch sentry_gazebo sim.launch.py arena:=rmuc gui:=true uwb:=true
```

```bash
# 终端 B：加载后端关键帧地图
ros2 launch bringup sim.launch.py lidar:=dual mode:=localization prior_map:="$PWD/maps/sim"
```

```bash
# 终端 C：先查看定位状态，看到 pose_trusted=true 后启动 Nav2 与 RViz
ros2 topic echo --once --qos-durability transient_local /backend_node/localization_state
bash tools/start_navigation.sh profile:=sim terrain_dir:="$PWD/maps/sim/terrain" rviz:=true
```

在另一个已加载环境的终端运行 `ros2 lifecycle get /bt_navigator`。看到 `active` 后，在 RViz 的 `map` 固定坐标系中选择已标注的可通行区域，用 **Nav2 Goal** 设置终点和朝向。当前 `maps/sim/terrain` 有已知隧道净空缺失，普通导航联调请选平地区域。隧道验证见第 6 节。

### 真车：先启动硬件与定位，再启动 Nav2

先确认 MID-360 网络配置和点云话题、实际串口以及先验地图目录。真车入口会启动底盘通信，默认 `/dev/ttyACM1`、`115200`。示例使用单雷达启动，地图路径取自本机已有目录。使用前核对地图的采集配置，保持车辆静止并准备人工停车。 [真车硬件配置](../src/localization/README.md#真车)

```bash
# 终端 A：替换为本次场地的先验地图与实际设备路径
PRIOR="$PWD/maps/real/session_001_optimized"
ros2 launch bringup real.launch.py mode:=localization lidar:=single \
  prior_map:="$PRIOR" chassis_port:=/dev/ttyACM1
```

```bash
# 终端 B：TERRAIN 改为第 6 节已导出、检查过的真车地形目录
TERRAIN="$PWD/maps/real/session_001_optimized/terrain"
python3 tools/nav_regress/terrain_check.py "$TERRAIN" && \
  bash tools/start_navigation.sh profile:=real terrain_dir:="$TERRAIN" rviz:=true
```

若只想核对将要执行的命令，使用 `bash tools/start_navigation.sh --dry-run profile:=real`。真车使用系统时间。发目标前同样确认 `pose_trusted=true`、Nav2 为 `active`，且 RViz 中车体与地图位置一致。

## 3. 从零建图，并保存后端地图包

给本次采集起一个**尚未使用的名字**。在建图、录包和离线处理终端分别设置同一组变量。以下示例为真车，仿真时把 `PROFILE` 改为 `sim`。保留选定的 `RUN` 值，后续步骤按这个名字找到同一次采集。

```bash
PROFILE=real
RUN=site_a_001
RAW="$PWD/maps/$PROFILE/$RUN"
OPT="${RAW}_optimized"
BAG="$PWD/bags/${PROFILE}_mapping/$RUN"
```

**真车建图**：在终端 A 运行下列命令。若用双雷达，改为 `lidar:=dual`。L2 驱动已在同一 domain 发布时沿用该驱动，若需由本入口启动，则同时传 `driver_l2:=true` 和 `l2_launch:=实际的绝对路径`。根据 `ros2 topic list -t` 核对 `/livox/lidar`、`/livox/imu` 以及双雷达的 `/unilidar/cloud`。

```bash
ros2 launch bringup real.launch.py mode:=mapping lidar:=single \
  map_dir:="$RAW" chassis_port:=/dev/ttyACM1 rviz:=true
```

**仿真建图**：先按第 2 节启动 Gazebo。在建图终端设置 `PROFILE=sim` 及对应的 `RAW` 等变量，再运行下面第一条。开始走图前可按第 4 节开始录包。手动驾驶使用独立终端，保持 Nav2 关闭，以免两个节点同时向 `/cmd_vel` 发指令。

```bash
# 终端 B：建图
ros2 launch bringup sim.launch.py mode:=mapping lidar:=dual map_dir:="$RAW" rviz:=true
# 终端 C：仅仿真，键盘操作，空格停车，x 退出
ros2 run sentry_gazebo teleop
```

**保存**：确认 `/lidar/points` 和 `/frontend/odometry` 持续更新，走完路线、停车，保持建图节点运行。在另一个设置了同一 `RAW` 的终端执行下列命令。服务将地图写入启动时的 `map_dir`。等到 `success=True` 并确认输出文件后再关闭建图进程。

```bash
ros2 service call /backend_node/save_map std_srvs/srv/Trigger '{}'
ls -lh "$RAW/keyframes.txt" "$RAW/map.pcd"
find "$RAW/clouds" -maxdepth 1 -name '*.pcd' | wc -l
ls -lh "$RAW"/*descriptors*  # 描述子文件依后端配置而定
```

## 4. 录包、检查与回放

**真车手动建图录包**：先启动驱动和建图，再在独立终端设置第 3 节的变量。第一条预览话题，第二条实际开始录制。`--output` 指向新的会话目录，脚本会在其中创建 `bag/`。开始驾驶前确认所用雷达和 IMU 均已发布。

```bash
bash tools/record_real_mapping.sh --dry-run
bash tools/record_real_mapping.sh --output "$BAG" --note "$RUN 单雷达建图"
```

默认录制 MCAP，含原始雷达、IMU、融合点云、TF、前后端状态和速度话题。限时录制可加 `--duration 180`，每个重映射后的原始话题用一个 `--topic /实际话题名` 追加。停车并保存地图后，在**录包终端**按 Ctrl+C，保持建图节点运行，直到脚本打印 `Saved session`，以便它保存停止时的节点与参数快照。

```bash
ros2 bag info "$BAG/bag"
cat "$BAG/bag_info.txt"
```

核对时长、实际雷达与 IMU 话题的消息数。单雷达应有主雷达与 IMU，双雷达还应有副雷达。会话根目录另有 `session.json`、`recorder.log`、`qos.yaml` 和 `config/`，便于复现实验配置。

**仿真录包**：设置 `PROFILE=sim` 对应的 `BAG` 后，在 Gazebo 的 domain 中按仿真时钟录制。以下示例记录两路原始点云、IMU 和 TF，单雷达可省略 `/lidar_l2/points`。

```bash
mkdir -p "$(dirname "$BAG")"
ros2 bag record --use-sim-time -s mcap -o "$BAG" --topics \
  /clock /lidar_mid360/points /lidar_l2/points /lidar_mid360/imu /tf /tf_static /joint_states
# Ctrl+C 收尾后
ros2 bag info "$BAG"
```

**回看真车原始输入**：在与实车、仿真和底盘均隔离的空闲 domain 中，只回放所需传感器与 TF。下例的 `BAG` 指向真车会话。用另一个相同回放 domain 的终端可运行 `ros2 topic hz /livox/imu` 核对数据。只在实际录有 L2 时追加 `/unilidar/cloud`。

```bash
ROS_DOMAIN_ID=82 ros2 bag play "$BAG/bag" --clock 50 --rate 1.0 --topics \
  /livox/lidar /livox/imu /tf /tf_static /joint_states
```

## 5. 离线优化关键帧地图

在第 3 节的 `RAW` 已保存后运行，`OPT` 选用与 `RAW` 同级的新目录。以下命令使用 CPU 优化关键帧位姿、重建点云和定位描述子。若这份地图由双雷达融合，且无法给出可靠的单一光心，在同一条命令末尾加 `--no-raycasting`。 [优化器参数及依赖](../src/mapping/offline_mapping_optimizer/README.md)

```bash
bash tools/optimize_map.sh --in "$RAW" --out "$OPT" --threads 4
ls -lh "$OPT/keyframes.txt" "$OPT/map.pcd" "$OPT/optimizer.log"
```

成功后，确认建图和录包均已保存，关闭原来的 `mode:=mapping` 进程。在第 2 节对应的定位命令中设置 `mode:=localization prior_map:="$OPT"`，检查定位与地图是否对齐，再基于这份地图制作地形。仿真可保持原 Gazebo 运行。优化失败时查看输出目录旁的 `${OPT}.optimization-failed.log`（优化过程已开始时生成）。如果跳过优化，第 6 节将 `PRIOR` 设为 `"$RAW"` 即可。

## 6. 地形分析、标注和导出调参

**生成可编辑工程**：在带桌面显示的终端打开最终采用的地图目录。编辑器读取局部关键帧点云生成高程栅格，自动分割平面、斜面，并显示坡度、朝向、高差和净空。

```bash
PRIOR="$OPT"
TERRAIN="$PRIOR/terrain"
ros2 run nav_terrain surface_studio.py "$PRIOR"
```

1. 点选面片查看几何信息。若切分过碎，调整“分割参数”中的最小面片面积后重新分割。重新分割会替换人工边界，已做过编辑时先另存工程。
2. 人工设置类型、方向和可通过性。`1` 标平地，`2` 标坡道，`7` 标隧道，`X` 切换可通过性，`R` 翻转方向。未标注、无数据和禁止通过的格子都会导出为障碍。
3. 如有场地模型，可在点云工程载入参考模型，设置近似初始位姿后运行 ICP。检查模型边界叠加与点云是否一致，再选择“采用模型面片和标注”。导航使用地图坐标系下的点云工程导出。 [配准与标注说明](../src/navigation/nav_terrain/README.md)
4. `Ctrl+S` 保存为 `*.surf.npz`。`Ctrl+E` 选择 `$TERRAIN` 对应的实际目录导出 Nav2 文件。今后修改标注时直接打开保存的工程。

**检查当前导出**：读取报告的 `passed`、`errors`、`warnings` 和隧道净空统计。`--robot-height` 单位为米，示例 `0.245` 为工具默认值，按实车尺寸核对。遇到净空缺失应回查测量、点云或模型来源，然后重新导出。

```bash
python3 tools/nav_regress/terrain_check.py "$TERRAIN" --robot-height 0.245
```

**只调整导航地形参数**：从 `terrain.msgpack` 生成另一版目录，保留原始标注和导出。`--speed` 为百分比，显式给出本版全部需要限速的标签。`--direction-split-deg` 控制多向坡分成几个方向区域。比对两版 `zones.yaml`、`speed_mask.pgm` 和检查报告后，再决定 Nav2 的 `terrain_dir` 指向哪一版。

```bash
CANDIDATE="$PRIOR/terrain_v2"
ros2 run nav_terrain terrain_export.py "$TERRAIN/terrain.msgpack" --out "$CANDIDATE" \
  --direction-split-deg 20 --speed tunnel=35 --speed slope=60 --speed undulation=50
python3 tools/nav_regress/terrain_check.py "$CANDIDATE"
```

**验证有隧道的地图**：选一个空闲 domain 单独启动规划器和整形器，检查每条有效隧道的两个方向。日志写入 `OUT`，脚本以非零退出表示有检查失败。用本机 `maps/sim/terrain/terrain.msgpack` 重导出后，检查器仍报告隧道缺少 `clearance`。该图最近一次严格规划与整形检查为 8/12，`tunnel_2`、`tunnel_5` 双向失败。补齐源数据后应重新导出和验证。实车通行还需核对物理净空与制动条件。 [回归工具与判据](../tools/nav_regress/README.md)

```bash
ROS_DOMAIN_ID=77 SMOOTHER=zone_shaper OUT="/tmp/${RUN}_tunnels" \
  bash tools/nav_regress/plan_tunnels.sh "$CANDIDATE"
```

## 7. 常见问题与停机

| 现象 | 先检查什么 |
| --- | --- |
| `Package ... not found` | 当前终端从第 1 节重新加载环境，用 `ros2 pkg prefix 包名` 确认解析路径 |
| 等不到点云、状态或保存服务 | 各终端 `echo "$ROS_DOMAIN_ID"`，用 `ros2 topic list -t` 检查实际话题，`ros2 service list -t` 查 `/backend_node/save_map`。仿真另查 `ros2 topic hz /clock` |
| Nav2 启动报缺少地形文件 | `terrain_dir` 指到第 6 节的导出目录，检查 `nav_map.yaml`、`speed_mask.yaml`、`zones.yaml` 和 `terrain.msgpack` |
| 有目标但车不动 | 用 `ros2 lifecycle get /controller_server`、`ros2 run tf2_ros tf2_echo map base_link`、`ros2 topic hz /nav/observation` 排查控制器、TF 和观测，再核对后端 `pose_trusted` 与底盘串口 |
| 录包有目录但数据不齐 | 查看 `bag_info.txt` 的逐话题计数和 `recorder.log`。对照 `ros2 topic info -v /实际话题名` 检查驱动、重映射和 QoS |

结束导航时先取消目标并确认车辆停稳，再依次关闭 Nav2、定位和数据源。结束建图时先停车、保存地图、完成录包收尾，再关闭建图和驱动。仿真键盘遥控用空格停车后按 `x` 退出。
