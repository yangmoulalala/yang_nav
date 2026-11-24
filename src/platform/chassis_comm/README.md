# chassis_comm

真车串口使用 17 字节小端帧（A5 5A、三个 float32、动作字节、载荷 CRC-16/MODBUS）。默认模式 `action_angle` 与下位机约定一致：

- 普通导航：`/cmd_vel.linear.x/y` 是**云台方向参考系** X/Y 速度（`gimbal_nav`，+X 前、+Y 左；沿用现有 ROS FLU 轴约定，实物轴向须标定）（m/s），无需 yaw 输入；发送 yaw=0、action_flag=0。`angular.z` 不会转换为角度。
- 动作：先发布 `/chassis/yaw_target`（Float32，机体系目标角度，rad），再发送 `/chassis/pass_hole`（Bool）的上升沿。yaw 必须新鲜（默认 0.3 s），采用触发时的角度快照；一个上升沿只发送一帧 action_flag=1，其余帧 yaw=0、action_flag=0。连续动作需先发 false，且串口成功发过低标志帧。
- 速度超时停止发送有效运动，过期或缺失的动作 yaw 不影响普通 X/Y。动作不会在速度恢复后补发。串口写失败不自动重发动作。

整车 `bringup real.launch.py` 默认 `/dev/nav`、115200 波特、100 Hz、`chassis_yaw_mode:=action_angle`。独立启动需指定 port 和 baud_rate。仅查看参数用 `ros2 launch chassis_comm chassis.launch.py --show-args`。

兼容旧协议/台架时可显式选择 `relative_angle`（持续显式 yaw，拒绝非零 angular.z）或 `twist`（angular.z × yaw_horizon 转相对角度）；这两个模式不代表本车普通导航接口，整车不再默认使用它们。

发送端只负责写串口，尚无 MCU 动作完成反馈接口；导航不会自动触发动作。角度零位、正方向和动作执行过程由双方协议约定，不能从导航角速度推导。

`protocol.py` 编解码与解析帧；`command_state.py` 管理输入新鲜度和动作边沿；`serial_node.py` 订阅 ROS 输入并写串口。测试使用伪终端，不访问硬件。


X/Y 参考系与动作 yaw 参考系不同：串口桥直接透传云台系速度，动作 yaw 仍是底盘机体系相对角度。
`/chassis/joint_states` 未接通不妨碍从云台上的雷达/IMU 观测云台朝向；不能把缺失反馈的零值当成真实底盘朝向。
串口当前只有发送，没有 MCU 接收/执行回显；bag 的 `/cmd_vel` 只能证明上位机输出。
