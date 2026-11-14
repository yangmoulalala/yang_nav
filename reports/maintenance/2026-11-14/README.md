# 仓库结构与代码、文档一致性维护（2026-11-14）

本轮盘点整个 `src/` 的 13 个 ROS 包及 `docs/`、`tools/`，检查构建入口、配置语法、参数表和本地文档链接；人工复核定位 TF 发布、重置原点、UWB 恢复及参考子图切换的实现和说明。
核心发现是定位实现已更新，但多份现用指南和两张 HTML 流程图仍描述旧行为。本轮统一这些说明，并补齐工作区目录入口。

## 修正与清理

| 内容 | 源码或证据与处理 |
| --- | --- |
| TF 发布归属 | `Publisher::publishTransforms()`、sim/real 四份后端与两份前端 YAML 证明组合启动由后端从同一投影、时间戳、TFMessage 发布两条动态边；修正总览、前后端指南、模块 README、流程图、四处配置注释与发布器注释 |
| 缺失参数文档 | 补齐必填的 `common.publish_odom_tf`、独立前后端的互斥配置和高频输出说明；核对完整 example 的前后端参数键及表内默认值，修正三处过时分组计数 |
| 恢复与参考切换 | 按当前 `makeUwbRecoveryPriorLocked()`、`Search::tryPositionPrior()` 及 `TrackingMatcher::compute()` 补充同代 SUSPECT 连续航向优先、失败回退全圈，以及新参考失败时按相同门限重试原半径内旧参考；LOST/跨代次行为保留 |
| 原始里程计 | 删除“连续不跳变”跨重置也成立的暗示，明确重置可改变原点及当前导航消费后端输出 |
| Gazebo bridge YAML | `%YAML 1.2` 后缺少 `---`，标准 YAML 解析器会拒绝；补齐文档起始标记，六条桥接映射和值保持原样 |
| 目录入口 | 新增根 README、tools 索引、reports 索引与 CAD 工具说明，并接入 docs 索引；按实际 ROS 包边界解释目录职责 |
| CAD 示例 | 修正不存在的 `maps/sim/nav/terrain.json` 和本机外部项目示例路径，明确输入格式、坐标变换与仅输出 PCD 的边界；命令帮助保留示例换行 |
| 重复 Git 钩子 | 删除根 `NUL/` 下四个文件：逐字节与 `.git/hooks/` 的正式钩子相同，Git 未配置 hooksPath 指向 NUL；实际 Git LFS 钩子保留 |
| 旧测试产物 | 删除已无 CMake 目标的 `build/backend/test_results/backend/backend_uwb_factor_test.gtest.xml`；当前测试证据仅按现有目标收集，避免旧 XML 虚增数量 |
| 未完成报告链接 | 若导航后续 README 尚未生成，将对应现用文档链接指向已存在的后续证据目录，不补写或推断其他任务的验收结论 |

## 验证

- 当前包发现：`colcon list --base-paths src --names-only`，共 13 包；报告的 `COLCON_IGNORE` 保留。
- 全部自有 Python、Bash、XML/URDF、YAML、Markdown 本地链接与锚点、两个 HTML 文档中的内嵌 SVG 语法检查，见 `static_checks.json`。不检查外部网络链接或 GFM 渲染器差异，不将此称为完整 HTML 视觉验收。
- 当前非供应商 C++ `.cpp` 均能在 CMake 文本中找到构建引用；这是构建入口筛查，不证明所有函数都被运行时使用。
- 前后端 example 的参数键与文档表内默认值核对，见 `parameter_docs.json`；bringup 键集合、实际允许差异及四种组合的 TF 发布归属检查，见 `bringup_params.log`。
- 在已加载 Jazzy 和工作区环境下执行 backend 19、frontend 1、nav_safety 1、nav_terrain 3，共 **24/24 个当前 CTest 目标、202 个 XML 内用例通过，0 失败、0 错误、0 跳过**；分包数量见 `test_summary.json`，原始 XML 在 `test_xml/`。不把 CTest 外层条目和内部用例重复相加。
- 源码 `git diff --check` 通过；本轮未写 Git 索引，检查前后已暂存补丁相同，见 `verification.json`。

复核命令（在工作区根目录执行）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 reports/maintenance/2026-11-14/audit_project.py
python3 reports/maintenance/2026-11-14/audit_parameters.py
python3 tools/check_bringup_params.py
ctest --test-dir build/backend --output-on-failure -j2
ctest --test-dir build/frontend --output-on-failure
ctest --test-dir build/nav_safety --output-on-failure
ctest --test-dir build/nav_terrain --output-on-failure
```

## 范围和恢复

执行时工作区已有大量未提交代码与文档变更，并有其他 Gazebo/Nav2 回归正在运行。本轮仅改说明、注释、CAD 命令帮助和 YAML 文档标记；未改导航/定位算法、参数值、地图或足迹，也未重新构建替换正在运行的二进制。运行测试使用现有构建，定位二进制的构建时间晚于本轮开始前的行为改动；本轮 C++ 只改注释。

没有重跑全场 Gazebo/真车验收，也没有改变此前的净高、坡道、真实地图路径和定位恢复完成边界。为避免干扰仍在写入的证据，没有批量清除约 3.9 GB 的既有 reports 或搬动日志。第三方驱动、许可、可选测试、公开库接口及本机地图均按各自构建/数据用途保留。

删除项和修改清单见 `changes.json`。实施前版本及删除产物保存在本机 Git 忽略目录：

- `backup/project-cleanup-2026-11-14.tar.gz`：17 个初始修改文件及重复钩子。
- `backup/project-structure-2026-11-14.tar.gz`：原 docs 索引。
- `backup/project-cad-doc-2026-11-14.tar.gz`：原 CAD 脚本。
- `backup/project-recovery-doc-2026-11-14.tar.gz`：原定位回归索引。
- `backup/project-generated-cleanup-2026-11-14.tar.gz`：旧测试 XML 及链接调整前文档（若需要调整）。

恢复应按文件提取所需版本；工作区存在并行修改，不应整包覆盖当前项目。未执行 Git 提交。
