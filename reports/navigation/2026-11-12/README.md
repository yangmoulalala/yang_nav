# 导航审查验证证据

> 归档说明（2026-11-14）：本目录保留结论、脚本和精简证据。大型原始数据已转入本机 `backup/report-evidence-2026-11-14.tar.zst`，源码快照、编译产物和重复中间记录已清理；清单与恢复方法见 `reports/maintenance/2026-11-14/commit_cleanup/README.md`（路径均从仓库根目录解析）。下文的 before/after、trace 和输入文件路径按原实验记录保留，现存状态以清单为准。

本目录保留 2026-11-12 测试、11-13 复核的最终证据。它是历史结果快照，不代表修改后的代码自动通过。当前状态见 [导航进度](../../../docs/nav_plan.md)。

| 文件 | 结果或用途 |
|---|---|
| [framework_accept.json](framework_accept.json) | 13/13 真实 Nav2 节点 + 确定性运动学闭环 |
| [framework_smoke.json](framework_smoke.json) | 8/8 冒烟，独立 domain 85 复跑 |
| [recovery_ladder.json](recovery_ladder.json) | 2/2 恢复场景，独立 domain 87 复跑 |
| [unit_tests.json](unit_tests.json) 与 `unit_*.xml` | 当前配置 107 个用例全部通过，原始单测 XML 可复核 |
| [tunnel_plans.txt](tunnel_plans.txt) | 真实地图 8/12，四条整形失败 |
| [terrain_check.json](terrain_check.json) | 真实地图检查失败，缺 clearance |
| [fixture_check.json](fixture_check.json) | 运动学测试地形数据检查通过 |
| [smoother_failure.log](smoother_failure.log) | 保留四条失败路径的整形诊断 |
| [shutdown_excerpt.log](shutdown_excerpt.log) | 保留 SIGINT 后 collision_monitor exit -11 的退出诊断 |

2026-11-13 已清理重复调试日志及临时 fixture，最后一轮结果先迁入本目录。清理前的导航源码、旧设计稿、废弃脚本与被删日志打包保存在本机忽略目录 `backup/navigation-cleanup-2026-11-13.tar.gz`，不作为当前运行输入。

原始 `/tmp/nav_audit_*` 路径属于已清理的历史输出，不能继续用作有效链接。测试结果中的绝对临时路径仅是运行元数据；复现命令见 [工具说明](../../../tools/nav_regress/README.md)。
