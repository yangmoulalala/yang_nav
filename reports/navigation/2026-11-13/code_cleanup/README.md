# 代码清理验证（2026-11-13）

> 归档说明（2026-11-14）：本目录保留结论、脚本和精简证据。大型原始数据已转入本机 `backup/report-evidence-2026-11-14.tar.zst`，源码快照、编译产物和重复中间记录已清理；清单与恢复方法见 `reports/maintenance/2026-11-14/commit_cleanup/README.md`（路径均从仓库根目录解析）。下文的 before/after、trace 和输入文件路径按原实验记录保留，现存状态以清单为准。

本轮范围及删除依据见 [一致性复核](../../../../docs/history/nav_code_cleanup_2026-11-13.md)。

| 检查 | 本轮结果 | 证据 |
|---|---|---|
| 四包构建 | 通过，nav_behaviors/nav_bringup/nav_safety/nav_terrain | [构建日志](build.log) |
| 当前单测 | 108/108，12 个目标；零失败、错误、跳过 | [汇总](unit_summary.json)、[原始 XML](unit_xml/) |
| 框架冒烟 | 8/8 | [结果](smoke.json) |
| 恢复阶梯 | 2/2 场景 | [结果](recovery.json) |

单测只统计当前 CMake 注册目标，避免旧 XML 或 colcon 重复汇总。nav_behaviors 73 个、nav_safety 9 个、nav_terrain 26 个。

本轮新增起点碰撞回归，强化入口路径碰撞断言；另完成 Python/Bash 语法、Markdown 本地链接及 Git 空白检查，见 [静态复核](static_checks.json)。

13 项完整运动学闭环、真实地图 12 条整形路径没有在本轮重跑；[历史证据](../../2026-11-12/README.md)保留。真实地图仍为 8/12，净高、坡道物理验证、时钟回退及退出异常仍待处理。
