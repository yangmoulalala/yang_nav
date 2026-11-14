# 真车手动建图与录包

导航核心参数共用 `src/nav_bringup/params/sim/nav2.yaml`。`profile:=real` 选择真车地图与真实时钟，`profile:=sim` 选择仿真地图与 `/clock`。`use_sim_time:=false` 也会选择真车默认地图，避免误加载仿真场地。显式传入 `terrain_dir` 时以该目录为准。

## 当前适配结论

定位已有真车入口 `bringup/real.launch.py`：MID-360 驱动、单/双雷达融合、URDF 关节 TF、ZLIO、后端建图/定位。导航使用相同规划器、控制器、车体轮廓和安全链，最终输出 `geometry_msgs/msg/Twist`，默认 `/cmd_vel`，可用 `cmd_vel_topic:=/your/chassis/topic` 重映射。

仿真和真车的组合定位入口均由后端在同一个 TFMessage 内发布 `map→lio_odom`、`lio_odom→base_link`，两条边来自同一代、同一时间戳的状态投影，避免前端重置原点时短暂拼出错误车体位置。对应 `frontend.yaml` 的 `output.publish_tf=false` 与后端 `common.publish_odom_tf=true` 必须配套；`tools/check_bringup_params.py` 已检查这一约束。独立运行前端时仍可使用其自带 TF 发布。

仓库没有真车底盘通信驱动，不能仅凭仿真认为底盘已接通。实机需要外部驱动把最终 Twist 转为轮速，确认指令是车体系，x 向前、y 向左、z 角速度逆时针，单位 m/s 与 rad/s，并验证底盘自身通信超时停车。当前没有硬件运动验收。

另有定位能力边界：当前后端在局中 `SUSPECT/LOST` 恢复时要求新鲜 UWB 位置先验，而 real 配置关闭 UWB。因而“真车可以启动建图”不代表“无 UWB 的真车丢定位后能自动恢复”；这一兜底尚未适配验收，见 `docs/localization_issues.md` 的 L8。手动建图使用 mapping 模式，不依赖 prior-map 局中重定位；真车自主导航前仍需处理这项能力及硬件实测。

仿真和真车的导航速度参数相同：MPPI x/y 上限 1.5 m/s，最终正常限速 1.6 m/s、2.5 rad/s；洞跟踪参考 0.6 m/s，坡参考 1.4 m/s。车体轮廓为 x=[-0.30,0.38]、y=[-0.30,0.30] m，运行代价图及控制检查加 0.01 m padding；路径整形加 0.0225 m padding，多留半个 0.025 m 代价图格的规划余量，应对滚动局部图的重采样。真车是否能实现这些速度、制动能力和轮廓，需要按实际安装测量核对。

传感器参数应保留差异，不能机械复制仿真值：

| 项目 | 仿真 | 真车当前配置 |
|---|---|---|
| 时间 | `/clock` | 真实系统/传感器时间 |
| IMU frame | `mid360_link` | `mid360_imu_link`（URDF 外参） |
| 加速度倍率 | 1.0（m/s²） | 9.81（驱动原始 g） |
| 最大 IMU 间隔 | 0.3 s | 0.1 s |
| 运行滤波 gyro variance | 4e-6 | 0.25，保留硬件配置，等待实测噪声标定 |
| 测距范围 | 0.3–40 m | 0.5–100 m |
| 初始化最弱方向支撑 | 5 | 15 |
| MID-360 逐点时间 | 整帧时间 | `timestamp`，绝对 ns，乘 1e-9 |
| UWB 恢复 | 开启且有仿真标定 | 关闭，等待真车坐标/时间标定 |

已按仓库 Livox 驱动源码核对：PointCloud2 的 `timestamp` 存储绝对纳秒时间；IMU 原样输出传感器加速度。驱动的 IMU 消息 header 当前写 `livox_frame`，ZLIO 实际使用配置中的 `frames.imu` 查询外参，排错时不要把 raw IMU header 当作已标定的 TF。L2 驱动在仓库外，其逐点时间字段、单位、frame 仍需实机核对。

MID-360 当前配置主机 IP `192.168.0.50`，雷达 IP `192.168.0.121`，在 `src/z-localization/bringup/config/real/MID360_config.json`。云台 `yaw_joint` 从 `/chassis/joint_states` 获取；从未收到时 joint_state_publisher 会填 0，旋转云台建图前必须确认真实关节角已接入。真实地图需要自己建图并导出地形，不能直接用仿真 terrain。

## 人手操控建图

在 WSL/Linux 的仓库根目录执行。录包脚本只订阅与记录，不启动导航，也不发布运动命令。建议先开录包，再启动建图，从而保留初始化输入。

终端一：

```bash
bash tools/record_real_mapping.sh --note 'MID360单雷达，人手操控建图，云台固定'
```

脚本打印本次 session 路径；也可明确指定一个**尚不存在**的输出目录：

```bash
bash tools/record_real_mapping.sh --output "$PWD/bags/real_mapping/session_001" \
  --note '硬件版本、路线、复现现象写在这里'
```

终端二：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch bringup real.launch.py lidar:=single mode:=mapping \
  map_dir:="$PWD/maps/real/session_001" rviz:=true
```

双雷达需额外提供真实 L2 驱动：

```bash
ros2 launch bringup real.launch.py lidar:=dual mode:=mapping \
  driver_l2:=true l2_launch:=/absolute/path/to/l2.launch.py \
  map_dir:="$PWD/maps/real/session_001" rviz:=true
```

用现有遥控/人手操控底盘完成建图。先保留一段静止数据，等待前端完成初始化，再开始移动。建图结束后，保持建图进程运行，调用地图保存服务（当前默认 `/save_map`，可用 `ros2 service list -t` 核实）：

```bash
ros2 service call /save_map std_srvs/srv/Trigger '{}'
```

确认服务返回保存成功后，再在录包终端 Ctrl-C，等待打印 bag 信息及 `Saved session`，最后停止建图。录包不代替后端地图保存。

## 录制内容与排错

默认记录原始 MID-360/L2 点云、IMU、融合云、frontend frame/odom/session、后端 odom/状态/位姿、TF/static TF、URDF、关节角、命令、诊断、ROS 日志与参数事件。默认话题在 `tools/record_real_mapping.py` 的 `TOPICS` 中。若驱动/底盘话题改名，追加 `--topic /actual/lidar --topic /actual/imu --topic /chassis/status`；`--all` 会包含所有发现的话题和隐藏话题，适合有更多外部硬件数据的排错记录。

输出采用标准 rosbag2 MCAP，每 120 秒分段；支持 `--storage sqlite3`。每次还保存源码版本与 diff、关键配置/URDF/导航源码快照、开始和结束时的话题/服务/节点列表与可读取的节点实际参数（`start_graph` / `stop_graph`）、`session.json`、`recorder.log`、`bag_info.txt`。先录包再启动建图时，结束快照可补上开始时尚未启动的节点参数，因此要等 `Saved session` 后再停止建图。原始传感器使用 best_effort 兼容订阅，静态 TF/URDF/会话状态使用 transient_local，支持晚于驱动启动录包。真实录包不等待 `/clock`。

```bash
ros2 bag info bags/real_mapping/session_001/bag
```

`bag_info.txt` 中应存在实际使用的 raw LiDAR 和 IMU，且消息数持续增长。脚本记录的当前时间为 bag 接收时间，消息内原始传感器 header/逐点时间保持原样。`completed=true` 表示 bag 格式可读取，不能替代检查传感器是否真的发布。

回放排错时使用隔离 ROS domain，避免录下来的运动指令连接实车。只回放需要的输入，例如：

```bash
ROS_DOMAIN_ID=92 ros2 bag play bags/real_mapping/session_001/bag \
  --clock --topics /livox/lidar /livox/imu /unilidar/cloud /tf_static /joint_states
```

在同一隔离 domain 使用 `localization.launch.py`、`config_dir` 指向归档的 real 配置、`use_sim_time:=true`；回放原始传感器时让定位重新计算 odom/TF，避免同时回放保存的定位输出。需要变更 raw 话题时同步修改 launch 的 `primary_topic` / `secondary_topic` / `imu_topic`。不应启动实时雷达驱动。

完成真车地图及地形导出后，导航入口为：

```bash
bash tools/start_navigation.sh profile:=real \
  terrain_dir:="$PWD/maps/real/session_001/terrain" rviz:=true
```

软件录包自测已经在隔离 domain 完成：273 条 IMU、91 条点云、录包启动前发布的静态 TF 和 URDF 均可从 MCAP 回读。此结果不代表真车传感器或底盘已经验收。
