from pathlib import Path
import json, shutil, re, collections, math
import numpy as np
ws=Path('/home/yang/yang_nav'); report=ws/'reports/navigation/2026-11-14/fortress_seam'
for run,dname in [('nav_fortress_centre1','fixed_yaw0'),('nav_fortress_centre90','fixed_yaw90')]:
 src=Path('/tmp')/run; dest=report/dname;dest.mkdir(exist_ok=True)
 for n in ('nav.log','goals.log','truth.log','plans.txt.jsonl'):shutil.copy2(src/n,dest/n)
 shutil.copy2(Path('/tmp')/(run+'_console.log'),dest/'console.log')
 nav=(src/'nav.log').read_text()
 start=float(re.search(r'\[([\d.]+)\] \[bt_navigator\]: Begin navigating[^\n]*to \(16.79, 0.61\)',nav).group(1))
 rows=[];truth=[];states=collections.Counter();epochs=set();start_sim=None
 for line in (src/'localization_trace.jsonl').open():
  r=json.loads(line)
  if r['wall']<start:continue
  if start_sim is None:start_sim=r['sim']
  if r['kind']=='localization_odom':
   p=r['data']['pose']; x,y,z,w=p[3:];yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
   rows.append([r['sim'],p[0],p[1],p[2],math.degrees(yaw)])
  if r['kind']=='truth':truth.append([r['sim']]+r['data']['pose'])
  if r['kind']=='/backend_node/localization_state':
   states[r['data'].split(';')[0]]+=1
   epochs.add(next(x for x in r['data'].split(';') if x.startswith('epoch=')))
 np.savetxt(dest/'last_goal_map.csv',np.array(rows)[::20],delimiter=',',header='sim,x,y,z,yaw_deg')
 stats=re.search(r'slope_8 done, in body max \|lateral\| ([\d.]+) m, max \|heading\| ([\d.]+) deg, min speed ([\d.]+)',nav)
 result=dict(goals=re.findall(r'result status[^\n]+',(dest/'goals.log').read_text()),last_goal_state_counts=states,epochs=sorted(epochs),start_sim=start_sim,final_map_pose=rows[-1],truth_start_z=truth[0][3],truth_end_z=truth[-1][3],tracker_body_lateral_max_m=float(stats[1]),tracker_body_yaw_max_deg=float(stats[2]),tracker_body_min_speed_mps=float(stats[3]))
 (dest/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
 print(dname,json.dumps(result,indent=2))
(report/'scripts').mkdir(exist_ok=True)
shutil.copy2('/tmp/fortress_yaws.py',report/'scripts/original_arrival_yaws.py')
shutil.copy2('/tmp/fortress_trace.py',report/'scripts/original_trace_summary.py')
shutil.copy2('/tmp/fortress_old_test.sh',report/'scripts/old_source_regression.sh')
