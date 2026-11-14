from pathlib import Path
import hashlib, json, shutil, xml.etree.ElementTree as ET, difflib, re, math
root = Path('/home/yang/yang_nav')
out=root/'reports/navigation/2026-11-14/manual_third_goal_stall'
summary=[]
for source in sorted((root/'build/nav_behaviors/test_results/nav_behaviors').glob('*.gtest.xml')):
    tree=ET.parse(source).getroot()
    summary.append(dict(target=source.name,tests=int(tree.attrib.get('tests',0)),failures=int(tree.attrib.get('failures',0)),errors=int(tree.attrib.get('errors',0))))
    (out/'unit_xml').mkdir(exist_ok=True)
    shutil.copy2(source,out/'unit_xml'/source.name)
(out/'unit_summary.json').write_text(json.dumps(dict(targets=summary, tests=sum(r['tests'] for r in summary)),indent=2)+'\n')
files=['src/nav_behaviors/src/zone_tracker.cpp','src/nav_behaviors/src/zone_aware_controller.cpp','tools/nav_regress/record_plan.py','src/nav_behaviors/include/nav_behaviors/zone_tracker.hpp','src/nav_behaviors/test/test_zone_tracker.cpp','src/nav_behaviors/test/test_zone_drive.cpp','src/nav_behaviors/test/test_costmap_collision.cpp']
patch=[]
for name in files:
    before=(out/'before'/name).read_text().splitlines(keepends=True)
    after=(root/name).read_text().splitlines(keepends=True)
    patch.extend(difflib.unified_diff(before,after,fromfile='before/'+name,tofile='after/'+name))
    (out/'after'/name).parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(root/name,out/'after'/name)
(out/'review.patch').write_text(''.join(patch))
(root/'reports/navigation/2026-11-14/manual_third_goal_stall/original_sha256.txt').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+p.name+'\n' for p in sorted((out/'original').iterdir()) if p.is_file()))
for name,src in [('baseline_yaw0','/tmp/nav_third_baseline'),('fixed_yaw0','/tmp/nav_third_fixed'),('heading_only_failed_yaw90','/tmp/nav_third_fixed_yaw90'),('framework_before_alignment','/tmp/nav_third_framework'),('recovery_only_failed_yaw90','/tmp/nav_third_final_yaw90'),('final_yaw90','/tmp/nav_third_aligned_yaw90'),('framework_final','/tmp/nav_third_framework_final')]:
    dest=out/name
    if not dest.exists():shutil.copytree(src,dest)
print(json.dumps(dict(test_cases=sum(r['tests'] for r in summary),targets=len(summary))))





