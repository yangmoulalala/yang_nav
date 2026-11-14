from pathlib import Path
import json,re,math,xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path('/home/yang/yang_nav');out=r/'reports/navigation/2026-11-13/navigation_fixes';p=out/'gazebo'
rows=[]
for line in (p/'chain_trace.jsonl').read_text().splitlines():
 try:rows.append(json.loads(line))
 except ValueError:pass
tr=[]
refs=[x for x in rows if x['kind']=='yaw_ref']
ref_t=np.array([x['t'] for x in refs]);ref_y=np.array([x['data'] for x in refs])
for row in rows:
 if row['kind']!='tracker_text':continue
 m=re.search(r'^slope_3\+tunnel_3 narrow (\w+).*yaw ([+\-\d.]+) deg',row['data'],re.S)
 if m:tr.append((row['t'],m[1],math.degrees(float(np.interp(row['t'],ref_t,ref_y))),float(m[2])))
start=tr[0][0];end=tr[-1][0]+.05
metrics={'timebase':'ROS /clock simulation time; wall clock had backward steps','zone_start_sim':start,'zone_duration_s':end-start}
for phase in ('approach','align','traverse','exit'):
 a=np.array([x[3] for x in tr if x[1]==phase]);metrics[phase]={'samples':len(a),'max_abs_error_deg':float(abs(a).max()),'p95_abs_error_deg':float(np.percentile(abs(a),95))} if len(a) else None
kinds=['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel_collision_checked','/cmd_vel'];data={}
for k in kinds:
 a=[x for x in rows if x['kind']==k and start-.5<=x['t']<=end+.5]
 t=np.array([x['t'] for x in a]);v=np.array([x['data'][2] for x in a]);data[k]=(t,v)
 wall_dt=np.diff([x['wall'] for x in a]);dt=wall_dt[wall_dt>0];metrics[k]={'median_interval_s':float(np.median(dt)),'p90_interval_s':float(np.percentile(dt,90)),'effective_hz':float(1/np.median(dt))}
for kin,kout in zip(kinds,kinds[1:]):
 grid=np.arange(start+.5,end-.1,.01);t,v=data[kin];u,z=data[kout];target=np.interp(grid,u,z)
 scores=[]
 for lag in np.arange(0,.601,.01):
  source=np.interp(grid-lag,t,v);scores.append((float(np.corrcoef(source,target)[0,1]),float(lag)))
 c,d=max(scores);metrics[kin+' -> '+kout]={'lag_s':d,'correlation':c}
a=[x for x in rows if x['kind']=='truth'];tt=np.array([x['data']['stamp'] for x in a]);yaw=np.unwrap([x['data']['pose'][3] for x in a]);sim=np.array([x['data']['stamp'] for x in a])
# Both commands and the truth derivative use simulation time, not stepped wall time.
sgrid=np.arange(sim[0]+.1,sim[-1]-.1,.02);rate=(np.interp(sgrid+.06,sim,yaw)-np.interp(sgrid-.06,sim,yaw))/.12;m=(sgrid>start+.5)&(sgrid<end-.1);g=sgrid[m];actual=rate[m]
for kind in ['/cmd_vel_nav','/cmd_vel']:
 t,v=data[kind];scores=[]
 for lag in np.arange(0,.601,.01):
  command=np.interp(g-lag,t,v);scores.append((float(np.corrcoef(command,actual)[0,1]),float(lag)))
 c,d=max(scores);metrics[kind+' -> truth']={'lag_s':d,'correlation':c}
console=(out/'gazebo_console.log').read_text();metrics['goals']=re.findall(r'result status (\d+) \((\w+)\) error_code (\d+), ([\d.]+) s',console)
metrics['unit_tests']=[]
for f in sorted((r/'build/nav_behaviors/test_results/nav_behaviors').glob('*.gtest.xml')):
 x=ET.parse(f).getroot();metrics['unit_tests'].append({'file':f.name,'tests':int(x.get('tests',0)),'failures':int(x.get('failures',0)),'errors':int(x.get('errors',0))})
metrics['unit_count']=sum(x['tests'] for x in metrics['unit_tests'])
for folder in ['occupied','recorded_exit','framework2']:
 metrics[folder]=json.loads((out/folder/'result.json').read_text())
(out/'validation.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n')
fig,ax=plt.subplots(2,1,figsize=(11,7),sharex=True)
reg=json.loads((r/'maps/sim/terrain/terrain.json').read_text())['registration'];ym=np.degrees(yaw)-reg['rpy_deg'][2]
ax[0].plot(tt-start,ym,label='Gazebo yaw (map registration)',lw=1.5)
a=[x for x in rows if x['kind']=='tf_pose'];ax[0].plot([x['t']-start for x in a],np.degrees(np.unwrap([x['data'][3] for x in a])),alpha=.8,label='Localization TF yaw')
ax[0].plot([x[0]-start for x in tr],[x[2] for x in tr],'--',label='Tracker reference')
for k in kinds:
 t,v=data[k];ax[1].plot(t-start,v,label=k,alpha=.8)
for a in ax:a.grid(alpha=.25);a.legend(fontsize=8)
ax[0].set_ylabel('Yaw (deg)');ax[0].set_ylim(min(x[2] for x in tr)-20,max(x[2] for x in tr)+40);ax[0].set_title('Fixed configuration: second goal through tunnel_3')
ax[1].set_ylabel('Angular velocity (rad/s)');ax[1].set_xlabel('Seconds since tracker entry');ax[1].set_xlim(0,end-start)
fig.tight_layout();fig.savefig(out/'fixed_yaw.png',dpi=150)
print(json.dumps({k:v for k,v in metrics.items() if k!='unit_tests'},ensure_ascii=False,indent=2))