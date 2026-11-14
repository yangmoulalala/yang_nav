# 提交整理与证据清理（2026-11-14）

按用户“有用的提交，没用的删掉”整理剩余工作区。定位修复已单独提交；启动、录包、CAD 工具、现用指南及历史结论保留到 Git。个人 Claude 设置仅保留本机，并从版本控制移除；`.claude/`、录包目录 `bags/` 及误生成的 `NUL/` 加入忽略规则。

## 保留与删除

- 报告原约 4.5 GiB。Git 收录结论、JSON 汇总、测试 XML、关键运行日志、分析脚本、误差图及压缩误差数组。最终四目标的完整路径四元数记录和最终 Nav2 框架 trace 也保留。
- 原始故障 trace、最终及关键失败补测的 trace/代价图、定位修复的有效二进制传感器输入保存在本机压缩包 `backup/report-evidence-2026-11-14.tar.zst`（约 1.06 GiB，Git 忽略）。压缩完整性及成员清单均已校验。每个文件的原路径、大小和 SHA256，以及压缩包 SHA256 见 [原始证据清单](archived_evidence.json)。
- 删除约 1.45 GiB 的重复源码快照、嵌套编译产物、无效录包冒烟副本、重复中间 trace 和代价图序列。逐文件清单见 [删除清单](removed_generated_files.txt)。报告中的 before/after 和原始数据路径保留其历史语义，不再表示这些文件均在 Git 中。
- `NUL/` 的四个文件与 `.git/hooks/` 正式 LFS 钩子逐字节相同，且无 hooksPath 指向该目录，因此删除重复目录；正式 Git 钩子保留。

## 本机原始证据恢复

先对照清单选取需要的路径；从仓库根目录使用临时目录恢复，避免覆盖当前文件。例如：

```bash
mkdir -p /tmp/yang_nav_evidence_restore
tar --zstd -xf backup/report-evidence-2026-11-14.tar.zst \
  -C /tmp/yang_nav_evidence_restore \
  reports/navigation/2026-11-14/manual_third_goal_stall/original/localization_trace.jsonl
```

该压缩包仅在本机存在，克隆 Git 仓库不会得到完整原始输入。Git 中的报告和汇总可独立阅读；精确重算原始数据需要该压缩包。被判定为重复/生成物而删除的文件不在压缩包内，可依据源码和工具重新生成，历史失败不能据此称为已经重新执行。

## 本次检查

- `check_bringup_params.py`：参数键、平台差异及四种 TF 发布归属均通过。
- 当前构建的 frontend 1/1、backend 19/19 CTest 目标通过。
- 自有 Python/Bash/XML/YAML 及文档链接检查通过；导航启动脚本 real profile dry-run、录包 dry-run、CAD 帮助命令通过。
- 本轮没有重跑 Gazebo 物理仿真或真车录包。历史测试结果保持各自日期与验证边界。
