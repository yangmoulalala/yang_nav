# chassis_comm

真车串口使用 17 字节小端帧（A5 5A、三个 float32、动作字节、载荷 CRC-16/MODBUS）。默认模式 `action_angle` 与下位机约定一致：

- 普通导航：`/cmd_vel.linear.x/y` 是**云台方向参考系** X/Y 速度（`gimbal_nav`，+X 前、+Y 左；沿用现有 ROS FLU 轴约定，实物轴向须标定）（m/s），无需 yaw 输入；发送 yaw=0、action_flag=0。`angular.z` 不会转换为角度。
- 动作：先发布 `/chassis/yaw_target`（Float32，机体系目标角度，rad），再发送 `/chassis/pass_hole`（Bool）的上升沿。yaw 必须新鲜（默认 0.3 s），采用触发时的角度快照；一个上升沿只发送一帧 action_flag=1，其余帧 yaw=0、action_flag=0。连续动作需先发 false，且串口成功发过低标志帧。
- 速度超时停止发送有效运动，过期或缺失的动作 yaw 不影响普通 X/Y。动作不会在速度恢复后补发。串口写失败不自动重发动作。

整车 `bringup real.launch.py` 默认 `/dev/nav`、115200 波特、100 Hz、`chassis_yaw_mode:=action_angle`。独立启动需指定 port 和 baud_rate。仅查看参数用 `ros2 launch chassis_comm chassis.launch.py --show-args`。

兼容旧协议/台架时可显式选择 `relative_angle`（持续显式 yaw，拒绝非零 angular.z）或 `twist`（angular.z × yaw_horizon 转相对角度）；这两个模式不代表本车普通导航接口，整车不再默认使用它们。

同一串口全双工接收大 yaw 实测角；尚无 MCU 动作完成反馈接口，导航不会自动触发动作。角度零位、正方向和动作执行过程由双方协议约定，不能从导航角速度推导。

`protocol.py` 编解码与解析控制帧；`big_yaw.py` 解析大 yaw 反馈；`command_state.py` 管理输入新鲜度和动作边沿；`serial_node.py` 订阅 ROS 输入并读写串口。测试使用伪终端，不访问硬件。

## 大 yaw 实测角接收

下位机在同一串口发送 **8 字节、200 Hz** 反馈，大 yaw 离线时暂停发送：

```
A5 5A | float32 小端角度(rad) | CRC16/MODBUS 小端
 2 B             4 B                    2 B
```

CRC 初值 `0xFFFF`、多项式 `0xA001`，仅覆盖角度的 4 字节。角度是**大 yaw 相对底盘的实测角**，范围 `[-π, π)`；接受 float32 编码后的 `-π` 边界，拒绝 NaN、Inf、越界值和 CRC 错误。例：`1.0 rad` 为 `A5 5A 00 00 80 3F 21 F4`。

| 默认话题 | ROS 类型 | 行为 |
| --- | --- | --- |
| `/chassis/big_yaw` | `std_msgs/msg/Float32` | `data` 为 rad；每个有效帧发布一次，正常约 200 Hz；无有效帧时不发布，不补零、不重复旧角度 |
| `/chassis/big_yaw_online` | `std_msgs/msg/Bool` | 启动为 false；有效帧恢复为 true；超过 `big_yaw_timeout`（默认 0.1 s）无有效帧则 false；仅状态变化时发布，reliable + transient_local，晚加入者可读取当前状态 |

角度话题使用 reliable + volatile、depth=20，不缓存给晚加入者。接收采用非阻塞串口、200 Hz 独立轮询，支持半帧、粘包、噪声、丢字节和坏帧后的重新同步；一次读到多帧时逐帧发布。超时按主机单调时钟计算，协议没有采样时间戳；跨超时静默间隔的半帧会丢弃。串口读写异常均退出，不自动重连重放动作。

启动方式不变（只能有一个节点打开 `/dev/nav`）：

```bash
source /opt/ros/jazzy/setup.bash
source /home/potential/yang_nav/install/setup.bash
ros2 launch chassis_comm chassis.launch.py port:=/dev/nav baud_rate:=115200
# 整车入口同样自动启用接收；可用 chassis_big_yaw_timeout:=0.1 等参数覆盖
# ros2 launch bringup real.launch.py
ros2 topic echo /chassis/big_yaw
ros2 topic hz /chassis/big_yaw
ros2 topic echo /chassis/big_yaw_online --qos-durability transient_local
```

独立 launch 支持 `big_yaw_topic`、`big_yaw_online_topic`、`big_yaw_timeout`；整车入口对应 `chassis_` 前缀参数。`tools/record_bag.py` 默认录制这两个话题。

反馈只发布测量角度和在线状态；没有自动写入 `/chassis/joint_states` 或改变 TF。CAD `yaw_joint` 的正方向与协议角度零位/正方向的转换尚需实物标定，导航速度仍以 `gimbal_nav` 为参考。


X/Y 参考系与动作 yaw 参考系不同：串口桥直接透传云台系速度，动作 yaw 仍是底盘机体系相对角度。
`/chassis/joint_states` 未接通不妨碍从云台上的雷达/IMU 观测云台朝向；不能把缺失反馈的零值当成真实底盘朝向。
串口现已接收大 yaw 实测角；该反馈不代表 MCU 已执行速度或过洞动作，bag 的 `/cmd_vel` 仍只能证明上位机输出。
