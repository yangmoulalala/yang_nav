# 工作区工具

以下路径从项目根目录解析，命令在 WSL 中运行。功能包自己的 launch、配置与安装脚本位于 `src/<包>/`。

| 入口 | 用途 | 使用说明 |
| --- | --- | --- |
| `start_navigation.sh` | 在已有定位与传感器的环境中启动导航；支持 `--help`、`--dry-run` | [导航使用指南](../docs/navigation_usage.md) |
| `record_real_mapping.sh` / `record_real_mapping.py` | 真车建图录包；参数、话题与存储要求见操作文档 | [真车建图与录包](../docs/real_mapping.md) |
| `check_bringup_params.py` | 核对完整参数文件、平台差异白名单及组合定位的 TF 发布归属；有问题返回非零 | [定位参数规则](../src/z-localization/README.md#参数规则) |
| `nav_regress/` | 导航静态接线、fixture 冒烟与闭环、Gazebo 目标执行、日志记录和诊断 | [导航回归工具索引](nav_regress/README.md) |
| `sim_tests/` | 定位建图、先验定位、绑架、原地旋转和双雷达标定回归 | [定位仿真测试索引](sim_tests/README.md) |
| `cad_map/cad_to_cloud.py` | 从 CAD 网格生成 PCD，并可与 SLAM 点云比较；不生成后端地图包 | [CAD 点云工具](cad_map/README.md) |

普通 Python/ROS 工具需先加载 `/opt/ros/jazzy/setup.bash` 与 `install/setup.bash`。
导航仿真与定位回归会启动进程并写入地图或日志，先阅读对应索引并设置独立 `ROS_DOMAIN_ID` 和输出目录。
长期保留的证据放入 [reports/](../reports/README.md)，重复运行的临时输出使用显式 `OUT` 或工具的输出参数。
