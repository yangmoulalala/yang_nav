import ast
import hashlib
import json
import shutil
import subprocess
import tarfile
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
import yaml

root = Path('/home/yang/yang_nav')
report = root/'reports/maintenance/2026-11-14'
scratch = Path('/mnt/c/Users/yang/Documents/Codex/2026-11-14/new-chat-8/work')

for script, output in [('audit_project.py', 'static_checks.json'),
                       ('audit_parameters.py', 'parameter_docs.json')]:
    shutil.copyfile(scratch/script, report/script)
    check = subprocess.run(['python3', str(report/script)], text=True, capture_output=True, cwd=root)
    (report/output).write_text(check.stdout)
    if check.returncode:
        raise RuntimeError(check.stdout + check.stderr)
params = subprocess.run(['python3', str(root/'tools/check_bringup_params.py')], text=True,
                        capture_output=True, cwd=root)
(report/'bringup_params.log').write_text(params.stdout + params.stderr)
if params.returncode:
    raise RuntimeError(params.stdout + params.stderr)

def from_archive(archive, name):
    with tarfile.open(root/'backup'/archive) as saved:
        return saved.extractfile(name).read().decode()

# Prove the only source-code edits are comments and help text.
cad_name = 'tools/cad_map/cad_to_cloud.py'
before_cad = ast.parse(from_archive('project-cad-doc-2026-11-14.tar.gz', cad_name))
after_cad = ast.parse((root/cad_name).read_text())
before_cad.body.pop(0)
after_cad.body.pop(0)
cad_code_unchanged = ast.dump(before_cad) == ast.dump(after_cad)
cpp_name = 'src/z-localization/backend/apps/publisher_state.cpp'
before_cpp = from_archive('project-cleanup-2026-11-14.tar.gz', cpp_name)
expected_cpp = before_cpp.replace('/// 发布允许推进的 map → odom TF 及同版本定位状态。',
                                  '/// 发布允许推进的 TF 投影及同版本定位状态；可选在同一消息中附带 odom → body。')
cpp_code_unchanged = expected_cpp == (root/cpp_name).read_text()
bridge_name = 'src/sentry_gazebo/config/bridge.yaml'
old_bridge = from_archive('project-cleanup-2026-11-14.tar.gz', bridge_name)
bridge = yaml.safe_load((root/bridge_name).read_text())
bridge_values_unchanged = yaml.safe_load(old_bridge.replace('%YAML 1.2\n', '')) == bridge
config_values_unchanged = True
for platform in ('sim', 'real'):
    for filename in ('backend.yaml', 'backend_single.yaml'):
        name = f'src/z-localization/bringup/config/{platform}/{filename}'
        config_values_unchanged &= yaml.safe_load(from_archive('project-cleanup-2026-11-14.tar.gz', name)) == yaml.safe_load((root/name).read_text())
with tarfile.open(root/'backup/project-generated-cleanup-2026-11-14.tar.gz') as saved:
    old_xml = saved.extractfile('build/backend/test_results/backend/backend_uwb_factor_test.gtest.xml').read()
    obsolete_xml_cases = len(list(ET.fromstring(old_xml).iter('testcase')))

cached_now = subprocess.check_output(['git', '-C', str(root), 'diff', '--cached', '--binary'])
index_unchanged = cached_now == (report/'before_index.patch').read_bytes()
whitespace = subprocess.run(['git', '-C', str(root), 'diff', '--check'], capture_output=True, text=True)
(report/'diff_check.log').write_text(whitespace.stdout + whitespace.stderr)

for package in ('backend', 'frontend', 'nav_safety', 'nav_terrain'):
    (report/'ctest_logs').mkdir(exist_ok=True)
    shutil.copyfile(root/f'build/{package}/Testing/Temporary/LastTest.log', report/f'ctest_logs/{package}.log')

readme = report/'README.md'
text = readme.read_text().replace('`runPositionPriorSearch()`', '`Search::tryPositionPrior()`')
text = text.replace('共 **24/24 个当前 CTest 目标通过**；实际用例数、失败/跳过数见',
                    '共 **24/24 个当前 CTest 目标、202 个 XML 内用例通过，0 失败、0 错误、0 跳过**；分包数量见')
readme.write_text(text)
checks = {
    'checked_at_utc': datetime.now(timezone.utc).isoformat(),
    'cad_executable_ast_unchanged': cad_code_unchanged,
    'cpp_exactly_one_comment_change': cpp_code_unchanged,
    'bridge_mapping_count': len(bridge),
    'bridge_values_unchanged': bridge_values_unchanged,
    'four_backend_config_values_unchanged': config_values_unchanged,
    'git_index_unchanged': index_unchanged,
    'git_diff_check_passed': whitespace.returncode == 0,
    'removed_obsolete_xml_testcases': obsolete_xml_cases,
    'cpp_without_cmake_mentions': json.loads((report/'static_checks.json').read_text())['cpp_without_cmake_mentions'],
}
(report/'verification.json').write_text(json.dumps(checks, indent=2) + '\n')
required = [cad_code_unchanged, cpp_code_unchanged, bridge_values_unchanged,
            config_values_unchanged, index_unchanged, whitespace.returncode == 0]
if not all(required):
    raise RuntimeError(json.dumps(checks))

changes = json.loads((report/'changes.json').read_text())
hashes = {}
for name in [*changes['modified'], *changes['created']]:
    hashes[name] = hashlib.sha256((root/name).read_bytes()).hexdigest()
(report/'after_sha256.json').write_text(json.dumps(hashes, ensure_ascii=False, indent=2) + '\n')
shutil.copyfile(scratch/'verify_and_deliver.py', report/'verify_and_deliver.py')

outputs = Path('/mnt/c/Users/yang/Documents/Codex/2026-11-14/new-chat-8/outputs')
outputs.mkdir(exist_ok=True)
deliverable = '''# yang_nav 项目维护报告 · 2026-11-14

已完成今天的结构、配置和文档一致性维护，检查项目 13 个 ROS 包及 docs、tools。

- 修正文档与实际 TF 发布、原点重置、SUSPECT 恢复航向及参考子图回退行为不一致的问题，补齐缺失参数说明，并同步两份 HTML 流程图和配置/源码注释。
- 补齐项目首页、工具入口、报告索引及 CAD 工具说明；修正 CAD 示例中的过时地形路径和输出范围描述。
- 修复 Gazebo bridge YAML 的文档起始标记。已核对六条桥接映射和四份后端配置的参数值均未改变。
- 删除 4 个无使用方的重复 Git 钩子文件及 1 个已撤销测试目标的旧 XML；正式 Git LFS 钩子保留。旧 XML 含 18 个历史用例，会污染当前用例汇总。

验证：24/24 个当前 CTest 目标、202 个 XML 内用例通过，0 失败、0 错误、0 跳过；前后端参数表和 bringup 配置检查通过；Python/Bash/XML/YAML、Markdown 链接与锚点、内嵌 SVG 语法及 Git 空白检查通过。

算法、配置参数值、地图和 Git 暂存区保持原状态。代码层面的两处编辑已分别通过 AST 和原文件比对确认仅涉及注释/命令帮助。执行时还有其他 Gazebo/Nav2 测试正在运行，本轮没有替换其二进制、清理其日志，或把其未完成结果记为通过。完整全场、坡道、净高和真车验收仍以项目当前进度文档为准。

详细清单、恢复快照位置、当前测试 XML、脚本及检查结果已保存到项目目录：

```text
/home/yang/yang_nav/reports/maintenance/2026-11-14/README.md
```

项目入口为 `/home/yang/yang_nav/README.md`。修改保持未提交，实施前版本位于项目 `backup/` 下本日的 `project-*.tar.gz` 快照中。
'''
(outputs/'yang_nav_维护报告_2026-11-14.md').write_text(deliverable)
print(json.dumps({'static': json.loads((report/'static_checks.json').read_text()),
                  'verification': checks, 'deliverable': str(outputs/'yang_nav_维护报告_2026-11-14.md')}, ensure_ascii=False, indent=2))
