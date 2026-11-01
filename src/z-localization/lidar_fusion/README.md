# lidar_fusion

把单/双雷达合并成 ZLIO 需要的一路带逐点时间的点云。
外参只从 tf2（URDF / robot_state_publisher）读取，本包不保存任何外参数值。
`lio_odom → base_link` TF 由 ZLIO 前端直接发布（`frames.base`），见 [frontend/docs/parameters.md](../frontend/docs/parameters.md)。

| 节点 | 可执行文件 | 作用 |
| --- | --- | --- |
| `lidar_merge` | `merge_node` | 主雷达 + 可选副雷达 → 一路 PointCloud2（`x y z offset_time`） |

## 接口

话题均为节点内固定的相对名，由 launch 重映射到实际话题。

### lidar_merge

| 话题 | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `primary/points` | 订阅 | `sensor_msgs/PointCloud2` | 主雷达（MID-360），任意 frame |
| `secondary/points` | 订阅（仅 `dual=true`） | `sensor_msgs/PointCloud2` | 副雷达（L2），与主雷达近似时间同步 |
| `lidar/points` | 发布 | `sensor_msgs/PointCloud2` | 合并点云，frame 为 `output_frame`；`offset_time` 为 uint32 ns，相对帧头（取最早点时刻） |

TF 需求：各输入点云的 `frame_id` → `output_frame`。QoS：订阅、发布均为 reliable，深度 5。

## 参数

全部必填、节点内无默认值，缺失时启动报错退出。示例见 [config/example.yaml](config/example.yaml)。

| 节点 | 参数 | 类型 | 说明 |
| --- | --- | --- | --- |
| `lidar_merge` | `dual` | bool | 是否合并副雷达；**只由 launch 给出**，不写入参数文件 |
| `lidar_merge` | `primary.time_field` / `secondary.time_field` | string | 逐点时间字段；空串表示整帧取帧头时间 |
| `lidar_merge` | `primary.time_scale` / `secondary.time_scale` | double | 字段值乘此倍率得到秒 |
| `lidar_merge` | `primary.time_absolute` / `secondary.time_absolute` | bool | 字段是绝对时间（true）还是帧头偏移（false） |
| `lidar_merge` | `output_frame` | string | 合并点云坐标系 |
| `lidar_merge` | `min_range` | double, m | 近距离剔除阈值 |
| `lidar_merge` | `sync_slop` | double, s | 双雷达同步时间容差 |

`secondary.*` 单雷达时也必须给出（参数文件对单/双雷达保持同一套键）。

## 单独启动

```bash
ros2 launch lidar_fusion merge.launch.py dual:=true use_sim_time:=true \
    primary_topic:=/lidar_mid360/points secondary_topic:=/lidar_l2/points output_topic:=/lidar/points
```

`merge.launch.py` 接受 `params_file:=/abs/path.yaml`，默认用本包的 `config/example.yaml`。
完整系统由 `bringup` include 该 launch，并传入 `bringup/config/<platform>/lidar_fusion.yaml`。
