# yang_nav 使用手册

已有地图时，按第 1、2 节启动，第 3 节发送目标。新场地按第 4～7 节完成建图、录包、优化和地形标注，再启动导航。

命令直接写地图路径，无需先定义 `PRIOR`、`TERRAIN` 等变量。真车示例使用车上已有的 `maps/real/site_a_001`；其地形文件直接放在地图目录内。换地图时，同时修改定位的 `prior_map` 和导航的 `terrain_dir`，两者须对应同一份地图。

## 1. 准备终端

在车上 Ubuntu 终端执行；远程连接为 `ssh potential@192.168.1.154`。每个新终端先运行：

```bash
cd ~/yang_nav
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

日常启动沿用车上现有的 ROS 环境，不必额外设置环境变量。首次安装或尚未编译时，见[依赖与构建](../README.md#构建与检查)。

## 2. 启动定位与导航

### 真车：两个终端

终端 A 启动 MID-360、底盘通信和地图定位。底盘默认 `/dev/nav`、`115200`：

```bash
ros2 launch bringup real.launch.py mode:=localization lidar:=single \
  prior_map:=/home/potential/yang_nav/maps/real/site_a_001
```

真车启动文件只对 MID-360 驱动进程启用 Fast DDS UDPv4，避免 DDS 共享内存端口检查阻塞同步发布、造成 IMU 断流。此设置不改变雷达网口参数，也不放宽前端断流保护。

终端 B 启动导航：

```bash
bash tools/start_navigation.sh profile:=real terrain_dir:=maps/real/site_a_001
```

有桌面显示时，末尾加 `rviz:=true`。只启动上述命令不会自行前往某个点；目标发送方式见第 3 节。已经运行的定位或导航不需要重复启动。

### 仿真：三个终端

仿真需要已安装 Gazebo，且 `maps/sim` 中已有配套先验地图和地形。分别执行：

```bash
# 终端 A：仿真与传感器
ros2 launch sentry_gazebo sim.launch.py arena:=rmuc gui:=true uwb:=true
```

```bash
# 终端 B：定位
ros2 launch bringup sim.launch.py lidar:=dual mode:=localization prior_map:="$PWD/maps/sim"
```

```bash
# 终端 C：导航
bash tools/start_navigation.sh profile:=sim terrain_dir:=maps/sim/terrain rviz:=true
```

## 3. 发送导航目标

**坐标说明**：下面的 `(1.0, 1.5)` 表示 `map` 地图坐标系中的 `x=1.0 米、y=1.5 米`，是地图上的绝对位置。`goal_yaw` / `--yaw` 单位为弧度，默认 `0`；真车与仿真的普通导航都只检查位置，不执行目标朝向，转向由独立动作接口处理。目标应落在本次地图标注的可通行区域。

车上当前 `maps/real/site_a_001/nav_map.pgm` 中，`(1, 1.5)` 所在栅格为障碍。以下保留这个坐标演示写法；实际导航请先核对并修正通行标注，或改为地图中的可通行坐标。

### 启动后自动前往 (1, 1.5)

先启动终端 A 的定位，把终端 B 的导航命令换成：

```bash
bash tools/start_navigation.sh profile:=real terrain_dir:=maps/real/site_a_001 \
  goal_x:=1.0 goal_y:=1.5 goal_yaw:=0.0
```

程序会等待 Nav2 激活、定位报告 `pose_trusted=true`，并确认 `map → gimbal_nav` 变换可用，然后向 `/navigate_to_pose` 发送一次目标。发送器会输出目标是否接受、是否成功到达。默认最多等待 120 秒，可用 `goal_wait_timeout:=180` 调整。等待超时后导航继续运行。

仿真时使用相同的目标参数，换成仿真配置：

```bash
bash tools/start_navigation.sh profile:=sim terrain_dir:=maps/sim/terrain rviz:=true \
  goal_x:=1.0 goal_y:=1.5 goal_yaw:=0.0
```

不传 `goal_x`、`goal_y` 时只启动导航。每次重新启动带目标参数的命令，都会执行一次自动发目标。

### 导航已启动，手动发送或换一个目标

另开一个已加载环境的终端，执行：

```bash
ros2 run nav_bringup send_goal.py 1.0 1.5 --yaw 0.0
```

这个命令也会等待就绪，并显示执行结果；运行期间按 Ctrl+C 会请求取消它发送的目标。仿真中在命令末尾加 `--ros-args -p use_sim_time:=true`。发送新目标会替换当前正在执行的目标。

也可以在 RViz 中把 Fixed Frame 设为 `map`，选择 **Nav2 Goal**，在地图上按住并拖动设置终点和朝向。

### 自己的程序如何调用

导航接口是 `/navigate_to_pose`，类型为 `nav2_msgs/action/NavigateToPose`。目标中的 `pose.header.frame_id` 填 `map`，位置填 `x/y`，朝向填四元数。`yaw=0` 对应 `z=0, w=1`。在定位可信且 Nav2 已激活后，也可直接调用：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.5}, orientation: {z: 0.0, w: 1.0}}}}' \
  --feedback
```

Python ActionClient 的完整实现见 [send_goal.py](../src/navigation/nav_bringup/scripts/send_goal.py)，包含等待就绪、发送、接收结果和取消。需要程序启动后自动导航时，可直接采用该实现，或使用上面的启动目标参数。

### 取消当前目标

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
```

该命令取消当前导航目标。确认车辆停稳后再关闭导航、定位和驱动。

## 4. 从零建图并保存

以下以新的采集目录 `site_a_002` 为例，每次采集换一个未使用的名字。

真车建图：

```bash
ros2 launch bringup real.launch.py mode:=mapping lidar:=single \
  map_dir:=/home/potential/yang_nav/maps/real/site_a_002 rviz:=true
```

双雷达改为 `lidar:=dual`，并确保 L2 驱动已运行；驱动和硬件设置见[真车配置](../src/localization/README.md#真车)。

仿真建图时先启动第 2 节的 Gazebo，再执行：

```bash
ros2 launch bringup sim.launch.py mode:=mapping lidar:=dual \
  map_dir:="$PWD/maps/sim/site_a_002" rviz:=true
```

仿真手动驾驶用 `ros2 run sentry_gazebo teleop`，空格停车，`x` 退出。手动建图时保持 Nav2 关闭。需要录包时，先按第 5 节开始录制，再驾驶走图。

走完路线后停车，保持建图进程运行，在另一终端保存地图：

```bash
ros2 service call /backend_node/save_map std_srvs/srv/Trigger '{}'
ls -lh maps/real/site_a_002/keyframes.txt maps/real/site_a_002/map.pcd
```

等到服务返回 `success=True`，并确认地图目录含 `keyframes.txt`、`clouds/`、`map.pcd` 和定位描述子后，再结束建图。仿真检查对应的 `maps/sim/site_a_002`。

## 5. 录包、检查与回放

录包适用于建图、定位、导航及传感器调试。在所需节点启动后，用同一 ROS domain 的独立终端录制；也可先启动录包，再启动业务节点。以下为建图示例：

```bash
bash tools/record_bag.sh --output bags/recordings/site_a_002 --note 'site_a_002 单雷达建图'
```

默认录制雷达、IMU、融合点云、TF、定位状态、导航路径、代价地图、障碍观测、导航模式、Action feedback/status 和速度链路。未发布的话题会在出现后自动录制。限时录制加 `--duration 180`，追加话题用 `--topic /实际话题名`，全部话题用 `--all`（含隐藏话题）。默认输出 `bags/recordings/<时间>`。

导航或定位时直接使用同一入口，例如：

```bash
bash tools/record_bag.sh --output bags/recordings/nav_site_a_001 --note 'site_a_001 导航问题复现'
bash tools/record_bag.sh --output bags/recordings/loc_site_a_001 --duration 180 --note '定位观测'
```

Action 目标和结果走服务，默认话题录制包含 feedback/status，不包含服务请求和响应。结束时先停车；建图场景还需保存地图。在录包终端按 Ctrl+C，保持本次导航、定位或建图节点运行，等待打印 `Saved session` 后再结束它们。

```bash
ros2 bag info bags/recordings/site_a_002/bag
```

检查时长和雷达、IMU 的消息数。会话根目录还包含 `session.json`、`recorder.log` 和配置快照。

仿真录包示例：

```bash
bash tools/record_bag.sh --use-sim-time --output bags/recordings/sim_site_a_002 --note '仿真观测'
```

`--use-sim-time` 同时录制 `/clock`，收到第一条时钟消息后才写入数据。可先用 `bash tools/record_bag.sh --dry-run` 查看默认话题。

录包时 `/livox/lidar` 的 QoS 随发布端协商；MID-360 驱动提供可靠传输时，录包端也用可靠订阅，避免 UDP 点云分片丢失后整帧缺失。

```bash
ros2 bag info bags/recordings/sim_site_a_002/bag
```

回看真车原始传感器数据时，先停止实车驱动、定位和导航，避免与回放话题重名：

```bash
ros2 bag play bags/recordings/site_a_002/bag --clock 50 --topics \
  /livox/lidar /livox/imu /tf /tf_static /joint_states
```

## 6. 离线优化地图

地图保存后，在 WebUI「离线地图优化」页选定原始地图和新输出目录，也可从终端运行以下命令。

```bash
bash tools/optimize_map.sh --in maps/real/site_a_002 --out maps/real/site_a_002_optimized --threads 4
```

输出包含优化后的关键帧位姿、点云和定位描述子。双雷达地图无法确定单一光心时，加 `--no-raycasting`。[优化参数及依赖](../src/mapping/offline_mapping_optimizer/README.md)

完成后，在第 2 节定位命令中把 `prior_map` 换成 `/home/potential/yang_nav/maps/real/site_a_002_optimized`，核对定位与地图对齐，再制作地形。跳过优化时直接用原始地图。

## 7. 地形标注和导出

在 WebUI「地形标注」页打开最终地图，例如 `maps/real/site_a_002_optimized`。

1. 点选面片查看坡度、方向、高程和净空。切分过碎时可调整分割参数。
2. 设置类型、通行性和方向。未标注和禁止通行的格子会导出为障碍，无观测格保留为未知。
3. 点击「保存工程」保存 `*.surf.npz`，再点击「导出 Nav2 地形」，例如导出到 `maps/real/site_a_002_optimized/terrain`。

检查导出结果：

```bash
python3 tools/nav_regress/terrain_check.py maps/real/site_a_002_optimized/terrain --robot-height 0.245
```

`--robot-height` 单位为米，按实车尺寸修改；净空缺失时回查点云或模型并重新导出。之后将第 2、3 节的 `terrain_dir` 改为 `maps/real/site_a_002_optimized/terrain`。

只调整导航地形参数时，可保留工程和原始导出，另导出一版：

```bash
ros2 run nav_terrain terrain_export.py maps/real/site_a_002_optimized/terrain/terrain.msgpack \
  --out maps/real/site_a_002_optimized/terrain_v2 \
  --direction-split-deg 20 --speed tunnel=35 --speed slope=60 --speed undulation=50
```

标注、模型配准和参数解释见[地形工具](../src/navigation/nav_terrain/README.md)。隧道双向规划验证见[导航回归工具](../tools/nav_regress/README.md)。

## 8. 常见问题

| 现象 | 检查方法 |
| --- | --- |
| `Package ... not found` | 按第 1 节加载当前工作区；尚未构建时先编译 |
| 启动报缺少地形文件 | `terrain_dir` 必须包含 `nav_map.yaml`、`speed_mask.yaml`、`zones.yaml` 和 `terrain.msgpack` |
| 自动目标一直等待 | 查 `ros2 lifecycle get /bt_navigator` 是否为 `active`，读取 `/backend_node/localization_state` 的 `pose_trusted` 和 `pose_extrapolated`，并用 `ros2 run tf2_ros tf2_echo map gimbal_nav` 检查导航参考变换 |
| 目标被拒绝或导航失败 | 看发送器和 Nav2 输出；确认目标在当前地图可通行区域且有足够空间 |
| 目标已接受但车不动 | 查定位状态、`ros2 topic hz /nav/observation`、底盘串口，以及导航进程日志中的停车原因 |
| 录包数据不齐 | 查 `bag_info.txt` 的逐话题计数和 `recorder.log` |

结束建图时先停车、保存地图、等待录包收尾，再关闭建图和驱动。


## 真车普通导航与串口协议

当前下位机普通模式执行云台方向参考的 X/Y 速度。`profile:=real` 使用仅控制平移的 MPPI，要求当前位置距路径终点不超过 0.15 m 且实测 X/Y 合速度不超过 0.03 m/s 才判定到达，普通目标的 yaw 不触发底盘转向；自动恢复不使用 Spin。控制器预测采用实测航向和被动角速度，不把 angular.z 当作可执行的普通控制通道。`profile:=sim` 共用控制器，保留原仿真速度参数。

真车导航串口默认 `/dev/nav`、115200 波特。A3 下行（24 字节）发送雷达系 X/Y 速度、持续过洞标志、相对雷达当前朝向的期望转角（rad），以及定位 TF 给出的当前 x/y（m）。普通平移默认 yaw=0；显式转角输入 `/chassis/yaw_target` 为 Float32，逆时针为正，超过默认 0.3 s 后恢复 yaw=0；不会把定位航向、编码器角或 angular.z 当作相对转角。`/chassis/pass_hole` Bool 为 true 时每帧保持过洞模式，false 为小陀螺模式。速度或定位 TF 过期时暂停 A3，触发板侧 100 ms 离线停车。A4 上行改为 15 字节，编码器实测角发布到 `/chassis/big_yaw`，在线状态发布到 `/chassis/big_yaw_online`，目标 x/y 发布到 `/chassis/nav_target`（PointStamped，map 系），供上层任务消费。导航节点只处理 A3/A4，不发布自瞄状态；编码器角不自动写入关节 TF。详情见 [底盘接口 README](../src/platform/chassis_comm/README.md)。

录包已包含相对转角输入、过洞模式、A4 目标点与编码器反馈话题，以便核对导航输出和板侧状态。修改控制与接口配置后需重启相关 ROS 进程，已有进程不会自动采用新配置。


真车与仿真定位、导航参考统一为 `gimbal_nav`，原点在底盘与云台的同轴旋转中心，轴向随云台。
雷达上的 LIO 可观测云台朝向，不需要猜测 yaw_joint。当前真车不发布默认零关节构造的底盘 TF。
代价地图、地形障碍过滤、BT 恢复与碰撞监视器均使用同一参考；未知底盘角度的碰撞范围用外切 16 边形覆盖，
内切圆半径 0.50 m，另加原有碰撞/规划留量（0.01/0.0225 m），因此部分窄通道会被拒绝。
该范围覆盖原配置矩形在任意平面转角下的轮廓；新增突出部件仍需重新测量。
启动参数 `gimbal_yaw_offset` 仅供实测 MCU 轴向安装偏角，不是 yaw_joint，也不填 bag 拟合的响应角。
`send_goal.py` 和 navigation.launch.py 的启动目标默认都使用 `gimbal_nav`。

004 的速度响应残差尚不能仅由缺少关节反馈解释。零安装偏角的新坐标变换与旧零关节变换等价，
不能把完成坐标重构当作已经修复实车偏航；还需核对 MCU 实际速度轴向、速度跟踪和时间延迟。

共享导航配置使用 20 Hz 控制，与 0.05 s 模型步长一致；速度平滑器和最终安全门仍为 100 Hz。
真车默认在共享 `nav2.yaml` 上叠加 `nav2_real.yaml`：普通地形跟踪参考上限 1.00 m/s，最终安全门按 `hypot(vx, vy)` 限幅至 1.50 m/s；百分比限速的 100% 对应 1.50 m/s。MPPI 与平滑器的 X/Y 每轴加速和减速幅值分别为 0.9、1.0 m/s²。局部方向看路径前方 0.10 m，参考速度大小看前方 2 m 并在终点附近降速；不发送预计会继续明显离开路径的速度。建图导航仍额外限速至 0.5 m/s；碰撞预测、碰撞监视与安全门看门狗不变。
这些真车初值参考 2026-11-26 23:33 的空场录包：目标执行期间，定位平移速度中位数 0.25 m/s、P95 为 0.63 m/s、峰值 1.34 m/s。短路线上观察到的峰值不是持续车速，也不是制动标定。`params_file:=...` 显式替换完整参数，不再叠加平台默认值。
20 Hz 来自此前真车低速调试时的计算预算，并非已测得的最佳频率；100 Hz 控制受 Jazzy MPPI 支持，但须结合计算耗时与传感器延迟验证。
这些软件验证不能据此判定 MCU 实际轴向已标定，也不能代替实车高速制动验证。
再次实车验证前应先停止车辆并取消旧目标，重启真车定位、导航以加载新参考系和插件，再发送新目标。

真车还启用有界的速度响应观察器：仅从稳定的最终速度命令与定位反馈更新当前进程内的运动预测。
它不保存硬件标定，也不写 yaw_joint；底盘输出关闭时不会用零速度反推轴向。
命令和响应都在运动参考系下使用；串口 A3 的 yaw 是相对雷达当前朝向的期望转角，普通平移默认 0。
