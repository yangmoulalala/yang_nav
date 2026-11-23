# 工作区工具

在 Ubuntu 工作区根目录执行。ROS、Python 地形模块和 ROS 可执行文件由 `/opt/ros/jazzy/setup.bash` 与 `install/setup.bash` 加载。

| 入口 | 用途 |
| --- | --- |
| `bash tools/start_navigation.sh --dry-run` | 展示导航启动命令。去掉 `--dry-run` 后连接当前 domain 已运行的定位与传感器 |
| `bash tools/start_navigation.sh profile:=real terrain_dir:=maps/real/site_a_001 goal_x:=1.0 goal_y:=1.5` | Nav2 和定位就绪后，自动发送一次地图坐标 `(1, 1.5)` 的目标，朝向默认 0 弧度 |
| `ros2 run nav_bringup send_goal.py 1.0 1.5 --yaw 0.0` | 导航已启动时发送目标并等待结果；Ctrl+C 请求取消。仿真加 `--ros-args -p use_sim_time:=true` |
| `bash tools/record_bag.sh --dry-run` | 通用录包，支持建图、定位、导航和传感器调试。列出待录话题；录包可指定 `--output`、`--topic`、`--duration`，默认写到 `bags/recordings/<时间>` |
| `bash tools/optimize_map.sh --help` | 调用已安装的优化器。源码入口 `python3 tools/optimize_map.py` 使用同一实现。新地图输出目录须与输入分开。[参数和安装](../src/mapping/offline_mapping_optimizer/README.md) |
| `python3 tools/check_bringup_params.py` | 比对定位配置的键、仿真与真车差异以及 TF 所属节点 |
| [nav_regress/](nav_regress/README.md) | 导航接线检查、运动学闭环、Gazebo 场地验证及观测记录 |
| [sim_tests/](sim_tests/README.md) | 定位建图、先验定位、绑架恢复、旋转与双雷达标定 |
| [cad_map/](cad_map/README.md) | CAD 网格采样和点云配准比较 |

录包工具记录已有节点，默认包含传感器、定位、导航路径、代价地图、障碍观测、导航模式、Action feedback/status 和速度链路，保存 ROS graph、QoS 覆盖配置、源码配置和 Git 版本。无需按建图或导航切换模式，尚未发布的话题会在出现后自动录制。用 `--topic` 追加重映射或命名空间下的话题，`--all` 录制所有发现的话题（含隐藏话题），仿真加 `--use-sim-time` 并等待 `/clock`。Action 目标和结果使用服务传输，默认话题录制不包含这些服务请求/响应。Ctrl-C 后保持业务节点运行，等待 `Saved session` 再结束它们。回归工具使用独立 `ROS_DOMAIN_ID`、`--out` 或 `OUT`，场景日志与 `result.json` 同目录。
