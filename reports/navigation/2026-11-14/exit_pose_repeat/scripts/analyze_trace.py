from pathlib import Path
import json,sys,collections
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

run=Path(sys.argv[1]);rows=[]
for line in (run/'localization_trace.jsonl').read_text().splitlines():
 try:rows.append(json.loads(line))
 except ValueError:pass

def series(kind,pred=lambda r:True):
 d={r['data']['stamp']:r['data']['pose'] for r in rows if r['kind']==kind and pred(r)}
 t=np.array(sorted(d));p=np.array([d[v] for v in t]);return t,p
truth_t,truth_p=series('truth')
raw_t,raw_p=series('tf',lambda r:r['data']['parent']=='lio_odom')
global_t,global_p=series('localization_odom')
firstcmd=next(r['sim'] for r in rows if r['kind']=='/cmd_vel' and np.linalg.norm(r['data'])>.01)
baseline=firstcmd-1.

def interpolate(t,p,ts):
 ts=np.clip(ts,t[0],t[-1]);xyz=np.array([np.interp(ts,t,p[:,i]) for i in range(3)]).T
 q=Slerp(t,Rotation.from_quat(p[:,3:]))(ts).as_quat();return np.c_[xyz,q]
def align(t,p):
 b=float(np.clip(baseline,max(t[0],truth_t[0]),min(t[-1],truth_t[-1])))
 a=interpolate(t,p,[b])[0];g=interpolate(truth_t,truth_p,[b])[0]
 ra=Rotation.from_quat(g[3:])*Rotation.from_quat(a[3:]).inv();ta=g[:3]-ra.apply(a[:3]);return ra,ta,b

def errors(t,p):
 mask=(t>=baseline)&(t>=truth_t[0])&(t<=truth_t[-1]);t,p=t[mask],p[mask]
 ra,ta,b=align(t,p);xyz=ra.apply(p[:,:3])+ta;rot=ra*Rotation.from_quat(p[:,3:]);gt=interpolate(truth_t,truth_p,t)
 e=xyz-gt[:,:3];angle=(Rotation.from_quat(gt[:,3:]).inv()*rot).magnitude()*180/np.pi
 yaw=(rot.as_euler('xyz')[:,2]-Rotation.from_quat(gt[:,3:]).as_euler('xyz')[:,2]+np.pi)%(2*np.pi)-np.pi
 return {'t':t,'xyz':xyz,'error':e,'norm':np.linalg.norm(e,axis=1),'angle':angle,'yaw':yaw*180/np.pi,'baseline':b}
lio=errors(raw_t,raw_p);glob=errors(global_t,global_p)
states=[];last=None
for r in rows:
 if r['kind']=='/backend_node/localization_state':
  s=r['data'].split(';')[0]
  if s!=last:states.append({'sim':r['sim'],'wall':r['wall'],'state':s,'text':r['data']});last=s
frames=[r['data'] for r in rows if r['kind']=='frontend_frame' and r['data']['stamp']>=baseline]
metrics={'run':run.name,'first_motion_sim':firstcmd,'alignment':'Separate fixed full-SE3 initial alignment at first_motion-1s; no fit to moving samples; interpolation by message simulation stamps','states':states,'frontend_epochs_during_motion':sorted(set(f['epoch'] for f in frames)),'samples':dict(collections.Counter(r['kind'] for r in rows)),'frontend':{'frames':len(frames),'lidar_rejected':sum(not f['lidar_updated'] for f in frames),'empty_clouds':sum(f['cloud_shape'][0]*f['cloud_shape'][1]==0 for f in frames),'matches_min':min(f['matches'] for f in frames),'matches_median':float(np.median([f['matches'] for f in frames]))}}
for name,e in [('lio',lio),('global',glob)]:
 i=int(np.argmax(e['norm']));metrics[name]={'max_position_error_m':float(e['norm'][i]),'time_max_sim':float(e['t'][i]),'error_xyz_at_max_m':e['error'][i].tolist(),'max_yaw_error_deg':float(np.max(np.abs(e['yaw']))),'final_error_m':float(e['norm'][-1]),'baseline_sim':e['baseline']}
 for threshold in [.1,.25,.5,1.,2.]:
  hit=np.flatnonzero(e['norm']>threshold)
  metrics[name][f'first_over_{threshold}m_sim']=float(e['t'][hit[0]]) if len(hit) else None
print(json.dumps(metrics,indent=2))
(run/'diagnosis.json').write_text(json.dumps(metrics,indent=2)+'\n')
np.savez_compressed(run/'aligned_errors.npz',**{k+'_'+a:v for k,e in [('lio',lio),('global',glob)] for a,v in e.items() if isinstance(v,np.ndarray)})
fig,axes=plt.subplots(4,1,figsize=(12,10),sharex=True)
for k,e,c in [('Frontend odometry',lio,'#d45030'),('Global localization',glob,'#2b67a0')]:
 axes[0].plot(e['t']-firstcmd,e['norm'],label=k,color=c,lw=1.5)
 axes[1].plot(e['t']-firstcmd,e['error'][:,2],label=k,color=c,lw=1.5)
 axes[2].plot(e['t']-firstcmd,e['yaw'],label=k,color=c,lw=1.5)
axes[0].set_ylabel('Position error (m)');axes[1].set_ylabel('Vertical error (m)');axes[2].set_ylabel('Yaw error (deg)')
axes[0].set_title(run.name+': odometry and localization versus Gazebo truth (fixed initial SE3 alignment)');axes[0].legend()
axes[3].plot([f['stamp']-firstcmd for f in frames],[f['matches'] for f in frames],color='#327850',lw=1,label='LiDAR matches')
reject=[f for f in frames if not f['lidar_updated']]
axes[3].scatter([f['stamp']-firstcmd for f in reject],[f['matches'] for f in reject],s=25,c='red',label='LiDAR update rejected')
axes[3].legend(fontsize=8);axes[3].set_ylabel('Match count');axes[3].set_xlabel('Seconds after first actual motion command (/clock)')
for s in states:
 if s['sim']>=baseline and 'SUSPECT' in s['state']:
  for ax in axes:ax.axvline(s['sim']-firstcmd,color='black',alpha=.35,ls='--')
for ax in axes:ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(run/'odometry_vs_global.png',dpi=150);plt.close(fig)