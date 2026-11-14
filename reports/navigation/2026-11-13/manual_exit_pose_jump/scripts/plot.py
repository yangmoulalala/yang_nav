from pathlib import Path
import json,math,re,hashlib,datetime
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
out=Path(__file__).resolve().parents[1]
a=json.loads((out/'analysis.json').read_text())
truth=[]
for l in (out/'logs/truth.log').read_text().splitlines():
 try:
  v=list(map(float,l.split()))
  if len(v)==5 and v[0]>1794584298: truth.append(v)
 except ValueError: pass
rec=[r for r in a['recoveries'] if r['epoch']>=a['goal_epoch']]
for r in rec:
 rows=[v for v in truth if r['epoch']-.12<=v[0]<=r['epoch']+.12]
 r['truth_bracket']={'first_wall_epoch':rows[0][0],'last_wall_epoch':rows[-1][0], 'xyz_displacement_m':math.dist(rows[0][1:4],rows[-1][1:4]),'yaw_delta_deg':(rows[-1][4]-rows[0][4]+180)%360-180,'samples':len(rows)}
a['incident_recoveries']=rec
a['correction_definition']='Norm of 3D sensor pose translation and SO(3) rotation before/after recovery; not horizontal displacement, yaw, or absolute localization error.'
a['timing_limitation']='Wall timestamps only. loc.log has a backward wall-time step near epoch 1794584337.6. Do not derive speed, latency, or precise simulation durations from these files. Truth logger throttles on wall time and may skip samples across clock changes.'
a['files']={p.name:{'size_bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted((out/'logs').glob('*.log'))}
(out/'analysis.json').write_text(json.dumps(a,indent=2)+'\n')
base=1794584280
fig,axes=plt.subplots(3,1,figsize=(11.5,8.2),sharex=True,gridspec_kw={'height_ratios':[1.1,1,0.65]})
x=[v[0]-base for v in truth]
tx,ty=-9.415037765663683,-.22529290326302126
alpha=math.radians(-.46518737473210825)
mx=[math.cos(alpha)*(v[1]-tx)+math.sin(alpha)*(v[2]-ty) for v in truth]
my=[-math.sin(alpha)*(v[1]-tx)+math.cos(alpha)*(v[2]-ty) for v in truth]
axes[0].plot(x,mx,label='Truth map x (approx.)',lw=1.7)
axes[0].plot(x,my,label='Truth map y (approx.)',lw=1.7)
axes[0].plot(x,[v[3] for v in truth],label='Truth world z',lw=1)
axes[0].set_ylabel('Position (m)'); axes[0].legend(loc='lower right',ncol=3,fontsize=9)
axes[0].set_title('Post-tunnel localization recovery: manual single-goal run, 2026-11-13')
rx=[r['epoch']-base for r in rec]
axes[1].bar(rx,[r['translation_3d_m'] for r in rec],width=.3,color='#a53030',label='Committed 3D pose correction')
axes[1].scatter(rx,[r['truth_bracket']['xyz_displacement_m'] for r in rec],color='#147c44',s=30,zorder=3,label='Truth displacement in nearby ~0.18 s wall-time bracket')
for r in rec:
 axes[1].annotate(f"{r['translation_3d_m']:.2f}",(r['epoch']-base,r['translation_3d_m']),xytext=(0,5),textcoords='offset points',ha='center',fontsize=9)
axes[1].set_ylim(-.12,4.15);axes[1].set_ylabel('Translation (m)');axes[1].legend(loc='upper left',fontsize=9)
s=[{'epoch':1794584298,'state':'NORMAL'}]+[s for s in a['states'] if 1794584298<s['epoch']<1794584341]
s.append({'epoch':1794584341,'state':s[-1]['state']})
state={'NORMAL':0,'OBS_DEGRADED':1,'SUSPECT':2}
axes[2].step([v['epoch']-base for v in s],[state[v['state']] for v in s],where='post',color='#7951a1')
axes[2].set_yticks([0,1,2],['NORMAL','OBS_DEGRADED','SUSPECT'],fontsize=8);axes[2].set_ylim(-.3,2.3)
for ax in axes:
 ax.grid(alpha=.2)
 for r in rec: ax.axvline(r['epoch']-base,color='#a53030',alpha=.16,lw=.8)
axes[2].set_xlim(30,60.5)
axes[2].xaxis.set_major_formatter(FuncFormatter(lambda v,pos:f'23:38:{int(v):02d}' if v<60 else '23:39:00'))
axes[2].set_xlabel('Asia/Shanghai wall time (not simulation /clock)')
fig.text(.13,.015,'Recovery corrections are 3D sensor-pose norms, not planar error. Truth map x/y use terrain registration.\nWall-clock irregularity near 23:38:57 limits timing and derivative analysis; no TF/odom stream was recorded.',fontsize=9,color='#444')
fig.tight_layout(rect=[0,.055,1,1]);fig.savefig(out/'pose_jump_evidence.png',dpi=160);plt.close(fig)
print('Saved plot, enriched JSON and SHA-256 manifest; incident recoveries:',len(rec))
print('Stationary at largest recovery:',max(rec,key=lambda r:r['translation_3d_m']))