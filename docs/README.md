# 项目文档

工作区目录与构建入口见[项目首页](../README.md)，脚本见[工具索引](../tools/README.md)，验证证据见[报告索引](../reports/README.md)。

导航相关文档按下面的职责维护，状态以当前计划及对应验证证据为准。

| 文档 | 用途 |
|---|---|
| [导航使用指南](navigation_usage.md) | 构建、启动脚本、目标操作、排查及常用命令 |
| [导航框架](navigation.md) | 当前架构、接口、安全约束及启动方式 |
| [导航进度](nav_plan.md) | 已验证结果、待办和完成边界 |
| [回归工具](../tools/nav_regress/README.md) | 复现命令、输入、输出及各检查能证明的范围 |
| [区域插件](../src/nav_behaviors/README.md) | 模块职责、区域策略和插件参数 |
| [地形制作](../src/nav_terrain/terrain_studio.md) | 手工标注与地形导出流程 |
| [审查记录](history/nav_audit_2026-11-12.md) | 2026-11-12 修复与问题发现的历史记录 |
| [验证证据](../reports/navigation/2026-11-12/README.md) | 历史结果、单测 XML 和必要失败诊断；最新状态见导航进度 |

原 `nav_m3c_design.md` 已撤下，由导航框架文档替代。历史材料中的完成结论不能覆盖当前进度。

定位相关文档：

- [前端指南](frontend_guide.md)
- [后端指南](backend_guide.md)
- [话题和使用方式](topics_and_usage.md)
- [定位问题记录](localization_issues.md)
- [定位数据流](zlocalization_flow.html)、[详细数据流](zlocalization_flow_detail.html)

最新维护记录：[2026-11-14 仓库结构与定位文档一致性复核](../reports/maintenance/2026-11-14/README.md)。

导航整理记录：[2026-11-13 文档与废弃文件整理验证](../reports/navigation/2026-11-13/README.md)。

代码维护记录：[2026-11-13 代码与文档一致性复核](history/nav_code_cleanup_2026-11-13.md)。
