# 验证报告与证据

此目录保存人工审查和回归证据。大型原始证据与生成物已按 [提交整理说明](maintenance/2026-11-14/commit_cleanup/README.md) 归档或清理；本机压缩包不随 Git 分发。`COLCON_IGNORE` 避免 colcon 扫描报告内复制的配置和源码。
每个日期目录中的 README 说明当时的代码、场景、结果与限制；历史报告不能覆盖当前代码的完成边界。

| 目录 | 内容 |
| --- | --- |
| [maintenance/2026-11-14](maintenance/2026-11-14/README.md) | 仓库结构、配置、源码入口与文档一致性维护；修正 TF 发布归属和 YAML 格式，删除重复 Git 钩子 |
| [navigation/2026-11-13](navigation/2026-11-13/README.md) | 导航文档及代码清理；同日其他子目录保存后续故障诊断与修复证据，分别阅读其 README |
| [navigation/2026-11-12](navigation/2026-11-12/README.md) | 导航框架、地形、单元测试及路径检查的历史结果 |

导航当前状态见 [docs/nav_plan.md](../docs/nav_plan.md)，日常回归命令见[工具索引](../tools/README.md)。
保留失败日志、测试 XML、结果 JSON 和复现所需输入的对应关系；移动或删除前检查文档引用及是否有进程仍在写入。
构建产物在根目录 `build/`、`install/`、`log/`，本机清理前快照在 Git 忽略的 `backup/`。
