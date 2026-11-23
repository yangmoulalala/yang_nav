# chassis_comm

`chassis_serial` 把速度门输出的 `geometry_msgs/Twist` 写入底盘串口。单独启动需指定 `port` 和 `baud_rate`。整车入口 `bringup real.launch.py` 默认使用固定串口 `/dev/nav`、115200 波特、100 Hz，无需在启动命令中指定串口；需要临时调整时可用 `chassis_*` 启动参数覆盖。只查看配置可运行 `ros2 launch chassis_comm chassis.launch.py --show-args`。

| 模块 | 接口与职责 |
| --- | --- |
| `protocol.py` | `Command`、`encode_frame`、`decode_frame`、`FrameParser` 处理 17 字节小端帧。帧头是 `A5 5A`，载荷为三个 float32 和一个动作字节，CRC-16/MODBUS 覆盖载荷 |
| `twist_adapter.py` | `twist_command` 把角速度乘以 `yaw_horizon` 转为相对航向角，默认前视 0.1 秒 |
| `command_state.py` | 管理速度和相对角度输入的新鲜度，处理显式动作上升沿 |
| `serial_node.py` | 订阅最终 `/cmd_vel`、显式航向与动作话题，按配置频率写串口 |

协议里的 yaw 单位为弧度，表示相对当前车体航向的角度。`relative_angle` 模式从 `/chassis/yaw_target` 取角度。测试覆盖编码、解析、超时、动作边沿和伪终端发送。[组合启动](../../localization/bringup/launch/real.launch.py)、[测试](test)
