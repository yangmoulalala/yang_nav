# 真车底盘控制协议与通信入口

`chassis_comm` 实现上位机发送、协议编解码和接收流解析参考。当前接口依据 2026-11-21 提供的协议；`action_flag` 表示**执行过洞动作**。

## 固定 17 字节帧

偏移从 0 开始；所有多字节字段采用小端序，float32 为 IEEE 754。

| 字节偏移 | 字段 | 长度 | 类型 | 含义 |
|---|---|---|---|---|
| 0–1 | SOF | 2 | uint8[2] | `A5 5A` |
| 2–5 | vx | 4 | float32 | 机体系 X 方向线速度，m/s |
| 6–9 | vy | 4 | float32 | 机体系 Y 方向线速度，m/s |
| 10–13 | yaw | 4 | float32 | 当前机体系下的期望相对转角，rad |
| 14 | action_flag | 1 | uint8 | 0 无动作，1 请求过洞动作 |
| 15–16 | CRC16 | 2 | uint16 | CRC 低字节、高字节 |

CRC-16/MODBUS：初始值 `0xFFFF`，反射多项式 `0xA001`（常规表示 `0x8005`），无最终异或；只校验 `frame[2:15]` 的 13 字节，不包含 SOF 或 CRC。标准自检 `123456789` → `0x4B37`。

联调向量：`vx=1.0`、`vy=-2.0`、`yaw=0.5`、`action_flag=1`：

```text
A5 5A  00 00 80 3F  00 00 00 C0  00 00 00 3F  01  5B D6
```

该帧 CRC 为 `0xD65B`，线上按 `5B D6` 发送。全零负载的完整帧为 `A5 5A` + 13 个 `00` + `03 EB`。

接收端先搜索帧头，累积完整 17 字节，再验证 CRC，成功后才更新指令。帧头可能出现在浮点字节中，不能在已经收集候选帧时无条件重启收帧；CRC 失败后逐字节重新搜索。`FrameParser.feed()` 支持分包、粘包、噪声、丢字节后的重新同步，只返回校验通过且浮点有限、动作值有效的命令。该解析器用于参考和测试，仓库未修改下位机固件，也未定义下位机反馈报文。

## yaw 相对转角与 ROS 输入

已确认：`yaw` 是以**当前机体朝向为零点**的期望相对转角，单位 rad。`yaw=0` 表示相对当前朝向不再转动。该字段不是世界系绝对航向，也不携带机体当前姿态。

每帧给出当前机体系中的目标偏角，上位机不累加历次 yaw。下位机应基于当前实测朝向解释新偏角，不能继续累加到上一次目标上。若上游希望完成一次固定角度转动，应随朝向变化发送剩余角度；持续重发同一个非零偏角会持续请求当前朝向之外的目标，不能视为“一次转完后保持”。

| 输入话题 | 消息 | 当前处理 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 使用 `linear.x/y` 作为 vx/vy；`angular.z` 的处理由 yaw_mode 决定 |
| `/chassis/yaw_target` | `std_msgs/msg/Float32` | relative_angle 模式下的显式相对转角，rad |
| `/chassis/pass_hole` | `std_msgs/msg/Bool` | `false → true` 请求一次过洞动作 |

支持两种互斥模式，避免两路角度来源混用：

- `yaw_mode=twist`（默认，导航使用）：直接接收最终安全门输出的完整 `/cmd_vel`，按 `yaw = angular.z × yaw_horizon` 生成当前机体系相对目标角；不订阅 `/chassis/yaw_target`。`yaw_horizon` 默认 0.1 s，配置值必须有限且大于 0。
- `yaw_mode=relative_angle`（显式角度输入）：直接使用 `/chassis/yaw_target`，不积分、不包络；`/cmd_vel.angular.z` 必须为 0，非零或无效值使本次速度输入失效并输出全零指令。角度与线速度均须持续更新。

`yaw_horizon` 是前视时长 T（s），默认 0.1 s。例如 `angular.z=1 rad/s` 时输出 yaw=0.1 rad（约 5.73°）。每次新 Twist 都重新计算该相对偏角，串口定时发送不累加它，也不更新输入看门狗时间。100 Hz 的发送周期为 0.01 s，T 与该周期独立，修改发送频率不会改变同一 Twist 对应的偏角。

这是面向下位机角度环的前视目标适配，实际角速度仍取决于下位机角度闭环响应；T 需实机调试，不能将该公式视为已经验证的角速度跟踪能力。上位机使用 ROS 绕 +Z 逆时针为正的符号，需与底盘实现一致。节点不读取定位姿态或云台 `yaw_joint`。

动作输入采用 reliable、volatile、depth=1，避免晚加入节点重放缓存动作。发送节点先成功发出至少一帧 0，才接受新的动作上升沿；请求有效时发送一帧 1，随后恢复 0。持续发布 true 不重复触发；下次请求前需发布 false，并让线上至少发送一帧 0。true 至少保持一个发送周期，过短脉冲可能被后续 false 取消。

动作只在控制输入新鲜时接受（relative_angle 模式要求速度与显式角度，twist 模式要求有效 Twist）；启动期或断流期间的请求不排队，恢复输入后不会补发旧请求。发送失败直接退出，不自动重连重发动作。协议没有 ACK 或序号，因此无法确认下位机是否实际完成动作，单帧丢失也可能漏触发；下位机仍须按约定做上升沿检测。

现有 `/nav_mode=tunnel` 包含部分接近洞口的阶段，并非独立的机械动作请求。当前未将它自动映射为过洞动作；具体在哪个阶段触发、动作执行期间由谁控制运动，需要与下位机行为确认。

## 构建与启动

在 `/home/yang/yang_nav` 执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select chassis_comm --symlink-install \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --base-paths src --packages-select chassis_comm --return-code-on-test-failure
colcon test-result --test-result-base build/chassis_comm --verbose
```

单独启动通信节点时，串口设备和波特率必须显式提供；真车入口已默认配置 `/dev/ttyACM1`、`115200`。当前串口格式为 8N1、无流控，需与下位机设置一致。100 Hz × 17 字节 × 10 bit/字节 = 17000 bit/s，波特率不足时启动报错。下面两项环境变量应先设为实机约定值：

```bash
ros2 launch chassis_comm chassis.launch.py \
  port:="$CHASSIS_PORT" baud_rate:="$CHASSIS_BAUD"
```

以上命令默认采用 Nav2 Twist 输入、`yaw_horizon=0.1 s`、100 Hz 发送。需要直接发送上游算好的相对角度时：

```bash
ros2 launch chassis_comm chassis.launch.py \
  port:="$CHASSIS_PORT" baud_rate:="$CHASSIS_BAUD" \
  yaw_mode:=relative_angle
```

真车入口 `real.launch.py` 同时启动此节点，默认串口 `/dev/ttyACM1`、波特率 `115200`：

```bash
ros2 launch bringup real.launch.py mode:=localization
```

默认 `chassis_rate:=100.0`、`chassis_yaw_mode:=twist`、`chassis_yaw_horizon:=0.1`；还可配置 `chassis_cmd_vel_topic`、`chassis_yaw_topic`、`chassis_action_topic`。需要更换串口或波特率时，使用 `chassis_port`、`chassis_baud_rate` 覆盖。通过 real 入口启动后无需另开独立发送节点。

独立入口仍可单独启动；两种入口均使用真实单调时钟检查输入新鲜度，禁止 `use_sim_time=true`。可以用 launch 参数 `cmd_vel_topic`、`yaw_topic`、`action_topic` 更换输入；twist 模式的速度源应为最终安全门输出，默认 `/cmd_vel`。

节点参数可通过 `ros2 run chassis_comm chassis_serial --ros-args -p ...` 设置：

| 参数 | 默认值 | 含义 |
|---|---|---|
| port | 空，必须指定 | 串口设备路径 |
| baud_rate | 0，必须指定 | 下位机约定波特率 |
| rate | 100.0 | 连续发送频率，Hz；launch 同名参数可覆盖 |
| yaw_mode | `twist` | Nav2 前视适配，或 `relative_angle` 显式相对转角 |
| yaw_horizon | 0.1 | 前视时长，s；须按下位机角度环响应调试 |
| command_timeout | 0.3 | 速度输入超时，s |
| yaw_timeout | 0.3 | yaw 输入超时，s |
| write_timeout | 0.005 | 单次串口写超时，s |
| cmd_vel_topic | `/cmd_vel` | 线速度输入；twist 模式还使用 angular.z |
| yaw_topic | `/chassis/yaw_target` | relative_angle 模式下的相对转角输入 |
| action_topic | `/chassis/pass_hole` | 过洞动作输入 |

启动后未收到有效控制输入、输入超时或数据无效时，持续发送 `vx=vy=yaw=0`、`action_flag=0`，正常退出也尽力发一帧同样的全零指令。相对转角归零用于撤销旧转向请求，实际制动响应由下位机实现。拔线、进程崩溃或写失败时，停车必须由下位机通信看门狗保证；该协议还没有提供急停/使能字段。

## 验证范围

2026-11-21：相对转角适配后的包构建成功，4 个测试目标全部通过，分别含 41、10、14、2 项 pytest 用例，共 67 项。colcon 汇总另包含 4 个测试目标包装条目。

自动化覆盖标准 CRC 向量、固定报文字节、各单比特损坏、流解析重新同步、角度/速度输入超时、过洞触发不重复及不补发、正负角速度前视换算、前视时长校验，以及反复发送不累计偏角、不刷新看门狗。ROS 集成测试使用隔离 domain 199、唯一话题与 Linux PTY 伪串口，分别运行两种模式，读取实际发送的 17 字节帧，验证角度来源隔离、角速度归零、无效输入/断流/正常退出的全零帧，以及不完整写入后的连接关闭。

这些测试不连接真车，不证明下位机接收、角度闭环或机械过洞动作已验收。

100 Hz 默认配置已通过真实 Nav2 + 运动学底盘 + PTY 联调：控制器、平滑、碰撞监控、命令门和串口在同一 3 s 窗口各收到 300 条，约 100 Hz；冒烟 8/8、导航闭环 13/13 通过。测量口径、抖动和真车适用边界见[100 Hz 验证报告](../reports/navigation/2026-11-21/control_100hz/README.md)。
