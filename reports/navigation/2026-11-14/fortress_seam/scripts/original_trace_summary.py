import json, math, collections, numpy as np
from pathlib import Path
src=Path('/tmp/nav_manual_gui/localization_trace_1794651945159297151.jsonl')
out=Path('/home/yang/yang_nav/reports/navigation/2026-11-14/fortress_seam')
rows=[]; states=[]; truth=[]; cmd=[]
for line in src.open():
 r=json.loads(line)
 if r['kind']=='localization_odom':
  p=r['data']['pose']; x,y,z,w=p[3:]; yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
  rows.append([r['sim'],r['wall'],p[0],p[1],p[2],math.degrees(yaw)])
 if r['kind']=='/backend_node/localization_state': states.append((r['sim'],r['data']))
 if r['kind']=='truth': truth.append([r['sim']]+r['data']['pose'])
 if r['kind']=='/cmd_vel': cmd.append([r['sim']]+r['data'])
a=np.array(rows); last=a[a[:,1]>=1794652121.518821559]; t0=last[0,0]
state_counts=collections.Counter(s.split(';')[0] for t,s in states if t>=t0)
tail=last[last[:,0]>=last[-1,0]-15]
r={'last_goal_map':[16.79,.61], 'last_goal_sim_start':t0,'duration_until_user_stop':float(last[-1,0]-t0),'last_map_pose':last[-1,2:].tolist(),'last_15s_xy_range':np.ptp(tail[:,2:4],axis=0).tolist(),'states':state_counts,'epochs':sorted(set(next(x for x in s.split(';') if x.startswith('epoch=')) for t,s in states if t>=t0)),'commands_last_15s_mean':np.array([c[1:] for c in cmd if c[0]>=last[-1,0]-15]).mean(axis=0).tolist(),'original_shaped_body_offset_m':-.45,'replan_body_offsets_m':[-.54,-.55,-.57], 'face_width_m':1.096,'padded_half_width_m':.31,'nominal_side_overhang_m':.45+.31-1.096/2}
(out/'original_summary.json').write_text(json.dumps(r,indent=2)+'\n')
np.savetxt(out/'original/last_goal_map.csv',last[::20],delimiter=',',header='sim,wall,x,y,z,yaw_deg')
print(json.dumps(r,indent=2))
