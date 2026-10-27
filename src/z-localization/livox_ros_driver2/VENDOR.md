# Vendored: livox_ros_driver2

Livox 雷达 ROS2 驱动，为前端 LIO 提供点云 / IMU 输入。

## 来源

`github.com/Livox-SDK/livox_ros_driver2`

## 本地改动

- **config/MID360_config.json**：传感器网络 IP 改为本机部署地址
  （host `192.168.1.5 → 192.168.0.50`，lidar `192.168.1.12 → 192.168.0.121`）。

其余源码差异属上游双版本（ROS1/ROS2）分支的版本差，未与本项目集成改动区分，
不在此认领。

- ROS 包维护者及本地集成作者统一为 mozihe（zhujunheng2005@gmail.com）；上游源码版权与许可保留。
