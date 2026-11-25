# chassis_comm

真车导航通信使用 `/dev/nav`，默认 115200 波特、8N1、100 Hz 下行。协议以 [最新版 MiniPC 协议](docs/minipc_protocol.md) 的导航部分为准：单字节帧头、小端 float32、CRC16/MODBUS（初值 `0xFFFF`，多项式 `0xA001`，覆盖帧头和载荷，CRC 低字节先发）。导航节点只发送 A3、接收 A4，同一串口出现自瞄帧时消费整帧后忽略，不发布自瞄话题。

## 导航下行 A3（24 字节）

```
A3 | is_through_hole:u8 | vx:f32 | vy:f32 | yaw:f32 | x:f32 | y:f32 | crc:u16
 0          1              2        6        10       14      18      22
```

- `/cmd_vel.linear.x/y` 是雷达（大 yaw）坐标系的前/左速度，m/s，默认参考系为 `gimbal_nav`。速度直接透传，下位机按小 yaw 实测角转换到枪口系。
- `yaw` 是**从雷达当前 +X 到期望朝向的有向转角**，rad，俯视逆时针为正。0 保持当前朝向，+π/2 逆时针转 90°，−π/2 顺时针转 90°。显式输入为 `/chassis/yaw_target`（Float32），不是绝对地图航向，也不是底盘相对角；不从定位 TF 或上行编码器角复制，不把 `angular.z`（rad/s）直接作为角度。
- 普通平移不依赖转角输入：启动、未收到输入、输入非法或超过 `yaw_timeout`（默认 0.3 s）时发送 yaw=0。需要转向时，上层应根据剩余角度持续更新相对转角；保持非零输入代表每帧都相对当时朝向请求该转角，不是只转一次的增量命令。
- `x/y` 是定位当前坐标，m，来自最新 `world_frame → base_frame` TF，默认 `map → gimbal_nav`。定位航向不参与 A3.yaw。TF 新鲜度检查使用采样时间，重复查询旧变换不会刷新看门狗。
- `/chassis/pass_hole`（Bool）为持续状态：true 时每帧标志为 1，底盘停止小陀螺并随头；false 时标志为 0，下位机执行小陀螺。启动默认 false，保持最后一次收到的值。
- 速度或定位缺失/超过各自默认 0.3 s 时，暂停 A3，让板侧在最后一个有效导航帧后 100 ms 离线停车；上行接收继续。零速度、标志 0 仍请求小陀螺，不能作为整车停车指令。输入恢复后按当前模式继续发送。
- 节点退出、读写失败或部分写入时关闭端口，由板侧 100 ms 看门狗停车，不自动重连。速度超时后的总停车延迟约为 `command_timeout + 0.1 s`，另加调度延迟。

## 导航上行 A4（15 字节）

```
A4 | yaw:f32 | x:f32 | y:f32 | crc:u16
 0      1        5       9       13
```

上行 yaw 是**从底盘 +X 到雷达 +X 的编码器实测角**，rad、逆时针为正，两轴同向为 0。x/y 是目标点，须与 A3 的当前坐标使用同一世界坐标系。旧版 11 字节 A4 不再兼容。

| 默认话题 | ROS 类型 | 含义 |
| --- | --- | --- |
| `/chassis/big_yaw` | `std_msgs/msg/Float32` | 每个有效 A4 发布一次编码器角，rad |
| `/chassis/big_yaw_online` | `std_msgs/msg/Bool` | 启动 false，有效 A4 后 true；超过 `feedback_timeout`（默认 0.1 s）无有效 A4 后 false；仅变化时发布，reliable + transient_local |
| `/chassis/nav_target` | `geometry_msgs/msg/PointStamped` | 每个有效 A4 发布一次目标 x/y，z=0，frame_id 为 `world_frame`（默认 map）；时间戳为主机接收时间 |

角度和目标话题为 reliable + volatile、depth=20；失联不重复旧值、不补零。协议没有采样时间戳，未规定 A4 频率；接收端以 200 Hz 非阻塞轮询，支持半帧、粘包、噪声和坏帧后逐字节重新同步。跨静默超时的半帧丢弃，CRC 正确但含 NaN/Inf 的 A4 也会丢弃。其他类型的帧不刷新 A4 在线状态。

`nav_target` 只发布收到的目标，不自动提交 Nav2 action；上层任务可订阅并决定目标朝向与调度。编码器角也不会自动写入关节状态或 TF；CAD 关节轴向、安装零位的转换仍须按实物约定处理。历史 `big_yaw.py` 的 AA 55 解码器仅为旧台架保留，真车节点不使用它。

## 启动和联调

```bash
cd /home/yang/yang_nav
source /opt/ros/jazzy/setup.bash
colcon build --packages-select chassis_comm bringup --symlink-install
source install/setup.bash
ros2 launch bringup real.launch.py
# 独立导航串口节点也默认 /dev/nav、115200；只能一个节点打开该端口：
# ros2 launch chassis_comm chassis.launch.py
ros2 topic echo /chassis/big_yaw
ros2 topic echo /chassis/nav_target
ros2 topic echo /chassis/big_yaw_online --qos-durability transient_local
```

独立 launch 可用 `--show-args` 查看参数；整车参数统一加 `chassis_` 前缀，如 `chassis_yaw_topic:=/chassis/yaw_target`、`chassis_feedback_timeout:=0.1`。端口仍可覆盖，以支持 PTY 联调，但节点、独立 launch、整车入口的默认值均为 `/dev/nav`。`yaw_mode`、`yaw_horizon` 以及此前 A2 自瞄话题参数不属于当前导航节点。

`tools/record_bag.py` 录制相对转角输入、过洞模式、A4 目标点、编码器角和在线状态。测试使用新版导航固定参考帧逐字节验证，ROS + Linux PTY 验证转角语义、输入超时、A4 接收、帧过滤、并发收发和串口错误，不访问硬件：

```bash
colcon test --packages-select chassis_comm --event-handlers console_direct+
colcon test-result --test-result-base build/chassis_comm --verbose
```
