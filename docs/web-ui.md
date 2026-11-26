# WebUI 导航工作台

在运行 ROS 2 的 Ubuntu 主机上编译工作区后启动。已有工作区升级时，重新编译 `backend`、`bringup` 和 `nav_terrain`。

```bash
cd ~/yang_nav
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select backend bringup nav_terrain --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
bash tools/start_web_ui.sh --port 8080
```

在受信任的局域网内，用浏览器访问 `http://主机局域网IP:8080` 即可直接使用工作台。用 `--host 127.0.0.1` 可限制到本机。WebUI 与业务节点使用同一个 `ROS_DOMAIN_ID`，未设置时为 `0`。

WSL 默认 NAT 环境下，Windows 本机可以访问 `http://localhost:8080`。其他设备通过 Windows 局域网 IP 访问时，需要将 TCP `8080` 转发到 WSL 的 IPv4 并放行主机防火墙。使用 Foxglove 时同样处理 TCP `8765`。WSL 开启镜像网络后，按主机防火墙配置放行所用端口即可。Ubuntu 真车直接使用车上的局域网地址。

在本机 WSL 环境已配置端口转发时，WSL IPv4 变化后需更新转发目标。可在管理员 PowerShell 中用 `wsl -d ubuntu24 -- hostname -I` 查看新地址，用 `netsh interface portproxy show v4tov4` 查看现有规则。

## 日常操作

1. 在「模式与地图」选择仿真或真车、单或双雷达，以及先验匹配或从零建图。虚拟场地、阵营和先验地图应相互对应。真车默认 `/dev/nav`，双雷达可以填写 L2 驱动 launch 路径，或沿用已运行的驱动。
2. 匹配模式选择先验地图和与之配套的地形。地形留空时从先验地图目录查找。建图时填写工作区 `maps/` 下的输出目录。保存地图成功后会替换已有目录的全部内容。
3. 点击「启动系统」。仿真会启动 Gazebo，匹配模式会启动定位，等位姿可信后再启动 Nav2。建图模式启动传感器及建图。也可以按进程分别启动、停止和重启。停止 Gazebo 会依次关闭网页启动的 Nav2 和定位，停止定位也会关闭网页启动的 Nav2。网页会标记同一 ROS 域里由别处启动的节点，这些节点由原终端管理。
4. 「运行概览」显示位姿及可信度、实测和指令速度、Nav2 生命周期状态、地形模式及底盘反馈。定位可信且 Nav2 激活后，可以提交或取消地图坐标目标。
5. 建图完成后，停稳，再点「保存地图」并等成功提示。全局重定位前先停止 Nav2 并确认停稳，再点「触发」。服务器从完整先验地图检索描述子，以新点云做几何验证，并按配置连续确认，整个过程无需 UWB。接受请求表示已排队。视野存在相似解时保持位姿未可信，可以到特征更明显的位置停稳后再试。恢复后查看定位状态和位姿，再启动 Nav2。

「参数调整」为仿真与真车分别保存参数。导航可调整速度和加减速度上限、速度前瞻、到达容差、无进展超时、规划代价权重和两级代价地图的膨胀参数。定位与建图可调整跟踪校正周期、子图半径和帧数、点云累计时间、开机搜索半径、重定位候选数和确认次数、建图关键帧间隔及回环子图半径。空值使用对应平台及雷达配置的默认值。保存后重启相应的 Nav2 或定位进程，使参数生效。参数覆盖和日志位于 `log/web_ui/domain_<ID>/`。

## 离线优化与地形标注

1. 保存地图并停止对应建图任务。在「离线地图优化」选择已保存的输入地图和 `maps/` 下的输出目录。「开始优化」后可查看日志或取消。优化成功会替换已有输出目录的全部内容。单雷达可启用射线过滤并填写保存点云坐标系中的光心。多光心的双雷达地图可沿用关闭射线过滤的默认选项。首次使用需安装并编译[离线优化器](../src/mapping/offline_mapping_optimizer/README.md)。
2. 将 STEP 场地文件放入工作区 `maps/field_models/`，在「地形标注 → 场地模型」选中文件并点「打开场地」。文件名在工作区内唯一时，也可仅输入文件名。`.step` 和 `.stp` 均受支持。STEP 原点留空时，`RMUL2027_3V3_V1.0.0` 使用与 Gazebo 一致的坐标，其他模型沿用 RMUC2027 的默认原点。首次加载大型文件可能需要几分钟。已保存的场地 `.surf.npz` 工程也可直接打开。
3. 在俯视图选中面片，设置类型、通行性和方向。可用画笔、矩形、套索和切割线修正范围。手动方向从当前坐标系正 X 轴开始逆时针计角。保存场地 `.surf.npz` 工程即可复用标注。
4. 切换到「建图点云」，打开已建图的目录、PCD 或点云 `.surf.npz` 工程。从离线优化页点「标注输出地图」也可带入输出路径。两个工程可来回切换，切换时保留各自的修改。
5. 在「点云配准到场地」用工程初值，或从已有地形的 `terrain.json` 读取初值。也可手动填写地图到场地的 X/Y/Z 平移和航向。运行 ICP 后，查看青色参考边界、残差及内点率，确认对齐再点「采用参考面片与标注」。
6. 保存建图标注工程可覆盖同名文件。在 `maps/` 下选择输出目录导出 Nav2 地形，成功后会替换已有目录的全部内容。未标注格和禁行格导出为障碍，无观测格保留为未知。之后在「模式与地图」将同一份地图和对应的导出地形设为先验地图和导航地形。

同一个 WebUI 实例共享正在打开的场地和建图工程。多台浏览器编辑时，可通过「读取当前工程」同步。重启服务后可通过路径重新打开已保存的工程。

### STEP 解析环境

新机器按下面的命令创建基于 Ubuntu 系统 Python 的环境，使 WebUI 能同时使用 ROS 2 Jazzy 的 Python 模块和 STEP 解析器。安装后重启 WebUI 服务。

```bash
sudo apt install python3-venv
python3 -m venv --system-site-packages ~/.venvs/nav-terrain
~/.venvs/nav-terrain/bin/python -m pip install cadquery-ocp
```

`tools/start_web_ui.sh` 优先使用 `YANG_NAV_WEB_PYTHON` 指定的解释器，其次使用已激活的 Python 虚拟环境，再尝试 `~/.venvs/nav-terrain/bin/python`。已有包含 `cadquery-ocp` 的环境时，也可以把该环境的解释器绝对路径配置到 systemd 服务的 `YANG_NAV_WEB_PYTHON`。

## 录包与 Foxglove

- 「开始录包」使用项目的 `record_bag.py`。可设时长、备注、追加话题或录制全部话题。仿真自动使用 `/clock`。录包目录位于 `bags/recordings/`。点击「停止录包」后等收尾，再关闭 ROS 节点。点击「关闭全部」也会先等待录包收尾。
- Foxglove Bridge 由 systemd 开机启动和故障重启。Ubuntu 主机已有对应服务时直接使用。首次部署可执行 `sudo apt install ros-jazzy-foxglove-bridge`，再以运行 ROS 的用户执行 `bash tools/install_foxglove_service.sh`。默认地址为 `ws://主机局域网IP:8765`，用于查看点云、TF、路径和代价地图。自定义端口时，安装脚本的端口参数和 WebUI 的 `--foxglove-port` 应一致。可用 `journalctl --user -u yang-nav-foxglove.service` 查看服务日志。
- 终端按 Ctrl+C 会退出 WebUI，并按顺序关闭由本次 WebUI 启动的进程。
