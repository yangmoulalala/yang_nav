from pathlib import Path
import tarfile,difflib,hashlib,json,shutil,ast,xml.etree.ElementTree as ET
import yaml
r=Path('/home/yang/yang_nav');out=r/'reports/navigation/2026-11-13/navigation_fixes'
# Validate the saved runtime configuration differs from production only by trace.
a=yaml.safe_load((r/'src/nav_bringup/params/sim/nav2.yaml').read_text());b=yaml.safe_load((out/'scripts/trace_nav2.yaml').read_text());b['controller_server']['ros__parameters']['FollowPath']['zone']['trace']=False;assert a==b
v=json.loads((out/'validation.json').read_text());assert len(v['goals'])==4 and all(g[1]=='SUCCEEDED' and g[2]=='0' for g in v['goals']);assert all(t['failures']==0 and t['errors']==0 for t in v['unit_tests'])
xmlout=out/'unit_xml';xmlout.mkdir(exist_ok=True)
for f in (r/'build/nav_behaviors/test_results/nav_behaviors').glob('*.gtest.xml'):shutil.copy2(f,xmlout/f.name)
base={}
with tarfile.open(out/'before.tar.gz') as tar:
 for item in tar:
  if item.isfile():base[item.name]=tar.extractfile(item).read()
paths=set(base)
paths.update(str(p.relative_to(r)) for p in (r/'src/nav_behaviors').rglob('*') if p.is_file())
paths.add('tools/nav_regress/occupied_start.py')
patch=[];changes=[]
for path in sorted(paths):
 p=r/path;before=base.get(path,b'');after=p.read_bytes() if p.is_file() else b''
 if before==after:continue
 changes.append({'file':path,'before_sha256':hashlib.sha256(before).hexdigest() if before else None,'after_sha256':hashlib.sha256(after).hexdigest() if after else None})
 patch.extend(difflib.unified_diff(before.decode().splitlines(True),after.decode().splitlines(True),fromfile='before/'+path,tofile='after/'+path))
(out/'review.patch').write_text(''.join(patch));(out/'changes.json').write_text(json.dumps(changes,indent=2)+'\n')
for name in ['navigate_to_pose.xml','navigate_through_poses.xml']:ET.parse(r/'src/nav_bringup/behavior_trees'/name)
ast.parse((r/'tools/nav_regress/occupied_start.py').read_text())
print('Final verification:',v['unit_count'],'unit cases; four Gazebo goals succeeded;',len(changes),'files in scoped source diff')