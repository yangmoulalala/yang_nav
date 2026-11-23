# 工作区工具

在 WSL 工作区根目录执行。ROS、Python 地形模块和 ROS 可执行文件由 `/opt/ros/jazzy/setup.bash` 与 `install/setup.bash` 加载。

| 入口 | 用途 |
| --- | --- |
| `bash tools/start_navigation.sh --dry-run` | 展示导航启动命令。去掉 `--dry-run` 后连接当前 domain 已运行的定位与传感器 |
| `bash tools/record_real_mapping.sh --dry-run` | 列出待录话题。录包可指定 `--output`、`--topic`、`--duration`，默认写到 `bags/real_mapping/<时间>` |
| `bash tools/optimize_map.sh --help` | 调用已安装的优化器。源码入口 `python3 tools/optimize_map.py` 使用同一实现。新地图输出目录须与输入分开。[参数和安装](../src/mapping/offline_mapping_optimizer/README.md) |
| `python3 tools/check_bringup_params.py` | 比对定位配置的键、仿真与真车差异以及 TF 所属节点 |
| [nav_regress/](nav_regress/README.md) | 导航接线检查、运动学闭环、Gazebo 场地验证及观测记录 |
| [sim_tests/](sim_tests/README.md) | 定位建图、先验定位、绑架恢复、旋转与双雷达标定 |
| [cad_map/](cad_map/README.md) | CAD 网格采样和点云配准比较 |

录包工具记录已有节点，保存 ROS graph、话题 QoS、源码配置和 Git 版本。用 `--topic` 增加实际被重映射的硬件输入，Ctrl-C 后等待 `rosbag2` 写完元数据。回归工具使用独立 `ROS_DOMAIN_ID`、`--out` 或 `OUT`，场景日志与 `result.json` 同目录。
