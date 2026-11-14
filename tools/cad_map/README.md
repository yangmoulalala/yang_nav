# CAD 网格转点云

`cad_to_cloud.py` 接收以米为单位、位于场地/world 坐标系中的二进制 STL 或 `mesh.npz`（数组 `V`、`F`），
按表面面积采样并体素化，输出 XYZ + intensity 的二进制 PCD。依赖 Python 3、NumPy 和 SciPy。
STEP 须先由其他工具三角化；本脚本不直接读取 STEP，也不生成包含关键帧和描述子的后端地图包。

在 WSL 项目根目录运行：

```bash
python3 tools/cad_map/cad_to_cloud.py --help
python3 tools/cad_map/cad_to_cloud.py \
  --mesh /absolute/path/field_mesh.npz \
  --registration-from maps/sim/terrain/terrain.json \
  --voxel 0.05 --out /tmp/cad/cad_map.pcd \
  --compare maps/sim/map.pcd
```

`--registration-from` 读取 `registration.rpy_deg` 和 `registration.t_world_map`（也接受这两项直接位于 JSON 根部），
其变换方向为 map → world；输出点云使用逆变换进入地图坐标系。未给出此参数时，输出仍在场地/world 系。
`--z-min`、`--z-max` 按三角形中心的场地高度筛选；`--spacing` 控制表面采样密度，`--voxel` 控制最终体素大小。

`--compare` 只打印与 SLAM 点云的近邻距离和 ICP 对齐统计；只有同时指定 `--refine-out /tmp/cad/refined.pcd`
才额外写出经 ICP 细调的 CAD 点云，原始 `--out` 文件保留。输出文件应使用独立路径，避免覆盖已有地图。
它不会修改地形标注或现有后端地图包。地图的正式保存与加载见[定位系统](../../src/z-localization/README.md)。
