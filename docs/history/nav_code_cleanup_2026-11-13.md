# 代码与文档一致性复核（2026-11-13）

范围：导航插件、命令门、导航 launch/BT/参数、回归工具及当前导航文档。依据当前 CMake、插件注册、两个行为树和代码调用关系核对；不将本次局部检查称为整个仓库绝对没有死代码。

## 清理项

| 内容 | 使用情况与处理 |
|---|---|
| `ZoneReplanPaused.last_phase` | 当前两个 BT 均未使用；移除旧恢复条件分支及端口，当前 phase 暂停逻辑保留 |
| `pausedRecently` / `have_pause` / `pause_stamp` | 只服务上述旧分支；移除重复状态，恢复使用的 `blockedRecently` 和锁存保护保留 |
| `parseZoneId` / `phaseMemory` | 无调用方；移除解析接口和只读包装 |
| `TrackerOutput.body_length` | 仅赋值、无人读取；移除字段及写入 |
| `Zone.have_z/z_min/z_max/climb` | 高程范围只读入、聚合，没有控制/整形消费方；移除内存副本、解析助手和无效测试设置。源地形及导出字段不改 |
| `ShaperParams.start_skip` | 生产适配器固定为零，纯几何调用默认仍可跳过起点附近碰撞；移除开关及跳过分支，统一检查所有候选起点 |
| 无用 include/import | 移除两个旧 footprint checker include 和五个 Python import |
| 注释 | 修正命令门 `/nav_mode` 发布方和 recovery 保留状态；补齐 phase 测试中不完整注释 |

保留了有明确用途的测试接口、命令行诊断工具和配置兼容性防护。例如 `collision_tolerance` 仍拒绝非零旧配置，不能因为当前配置为零就删掉这个防护。

## 验证

- 四包构建通过，三包当前 12 个测试目标全部通过：nav_behaviors 73、nav_safety 9、nav_terrain 26，共 **108/108**。
- 新增 `RobotStartCannotBypassCollisionChecks`，并强化已有入口路径用例，覆盖实际起点碰撞。
- 冒烟检查 8/8，恢复整栈 2/2 场景；结果见 [本轮证据](../../reports/navigation/2026-11-13/code_cleanup/README.md)。
- 旧端口、字段、接口已无源码引用；文档链接、Python/Bash 语法和 Git 空白检查通过。
- 本轮没有重新运行 13 项完整运动学闭环及真实地图 12 条路径，历史证据保留在原目录，不冒称重跑。

当前地图 8/12、缺净高、坡道物理验证、时钟回退和退出异常仍是待办，见 [导航进度](../nav_plan.md)。

## 边界和恢复

本轮未修改地图、定位、BT 执行节点和导航参数。`last_phase` 是删除的旧 BT 接口；自定义外部树如使用它，应移除端口并通过 `ZoneRecovery` 保护恢复，不能沿用历史暂停条件。

清理前快照在本机忽略目录 `backup/navigation-code-cleanup-2026-11-13.tar.gz`。原有未提交工作和暂存状态保留，未执行 Git 提交。
