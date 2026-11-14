from pathlib import Path
import json, collections, re
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
p=Path('/home/yang/yang_nav/reports/navigation/2026-11-14/manual_repeat_0054'); logs=p/'logs'
rows=[json.loads(l) for l in (logs/'localization_trace.jsonl').read_text().splitlines()]
err=np.load(logs/'aligned_errors.npz')
def select(k,where=lambda r:True):return [r for r in rows if r['kind']==k and where(r)]
def data(k,where=lambda r:True):return [r['data'] for r in select(k,where)]
def poses(k,where=lambda r:True):
 d={r['data']['stamp']:r['data']['pose'] for r in select(k,where)};t=np.array(sorted(d));return t,np.array([d[x] for x in t])
def interp(t,x,ts):
 ts=np.clip(np.atleast_1d(ts),t[0],t[-1]);return np.c_[np.array([np.interp(ts,t,x[:,j]) for j in range(3)]).T,Slerp(t,R.from_quat(x[:,3:]))(ts).as_quat()]
def inv(a):
 r=R.from_quat(a[3:]).inv();return np.r_[r.apply(-a[:3]),r.as_quat()]
def mul(a,b):
 r=R.from_quat(a[3:]);return np.r_[a[:3]+r.apply(b[:3]),(r*R.from_quat(b[3:])).as_quat()]
truth_t,truth_p=poses('truth'); raw_t,raw_p=poses('tf',lambda r:r['data']['parent']=='lio_odom'); pred_t,pred_p=poses('frontend_prediction',lambda r:r['data']['epoch']==1)
rawdict=dict(zip(raw_t,raw_p));extra_t=[];extra=[]
for t,x in zip(pred_t,pred_p):
 if t in rawdict:extra_t.append(t);extra.append(mul(inv(x),rawdict[t]))
extra_t=np.array(extra_t);extra=np.array(extra)
frame_t,frame_p=poses('frontend_frame',lambda r:r['data']['epoch']==1)
extra_frame=interp(extra_t,extra,frame_t);base_frame=np.array([mul(a,b) for a,b in zip(frame_p,extra_frame)])
baseline=44.456
def pose_errors(t,x):
 a=interp(t,x,baseline)[0];g=interp(truth_t,truth_p,baseline)[0];alignment=mul(g,inv(a));mask=(t>=baseline)&(t<=truth_t[-1]);t=t[mask];xp=np.array([mul(alignment,a) for a in x[mask]]);gt=interp(truth_t,truth_p,t);e=xp[:,:3]-gt[:,:3];yaw=(R.from_quat(xp[:,3:]).as_euler('xyz')[:,2]-R.from_quat(gt[:,3:]).as_euler('xyz')[:,2]+np.pi)%(2*np.pi)-np.pi
 return t,np.linalg.norm(e,axis=1),np.rad2deg(yaw)
post_t,post_e,post_yaw=pose_errors(frame_t,base_frame)
lines=[]
def out(x):lines.append(x);print(x)
out('ALL TIMES BELOW ARE MESSAGE SIMULATION SECONDS, except explicitly identified log wall timestamps.')
# Validate data timing and extrinsic stability.
summary={}
for k,a in [('truth',select('truth')),('imu',select('imu')),('raw_tf',select('tf',lambda r:r['data']['parent']=='lio_odom')),('frontend_prediction',select('frontend_prediction')),('frontend_frame',select('frontend_frame'))]:
 a=[x for x in a if x['data']['stamp']>=baseline];ts=np.array([x['data']['stamp'] for x in a]);dt=np.diff(ts);lag=np.array([x['sim']-x['data']['stamp'] for x in a]);inc=np.diff(np.unique(ts))
 summary[k]={'count':len(a),'source_dt_min':float(dt.min()),'source_dt_max':float(dt.max()),'source_dt_p50':float(np.median(inc)),'nonpositive_source_dt':int(np.sum(dt<=0)),'receive_age_p50':float(np.median(lag)),'receive_age_max':float(np.max(lag)),'incident_90_100_max_gap':float(np.diff(ts[(ts>=90)&(ts<=100)]).max())}
 out(k+' timing '+json.dumps(summary[k]))
mask=(extra_t>=baseline);e0=interp(extra_t,extra,baseline)[0];tr=np.linalg.norm(extra[mask,:3]-e0[:3],axis=1);ang=(R.from_quat(e0[3:]).inv()*R.from_quat(extra[mask,3:])).magnitude()*180/np.pi
summary['imu_from_base']={'samples':int(mask.sum()),'max_translation_change_m':float(tr.max()),'max_rotation_change_deg':float(ang.max()),'baseline_pose':e0.tolist()};out('Derived prediction-to-base extrinsic '+json.dumps(summary['imu_from_base']))
summary['posterior']={'max_error_m':float(post_e.max()),'max_error_time':float(post_t[post_e.argmax()]),'max_yaw_error_deg':float(abs(post_yaw).max()),'final_error_m':float(post_e[-1])};out('Posterior converted to base '+json.dumps(summary['posterior']))
out('\nTIMELINE: sim | truth x y z | raw error m / yaw deg | posterior error m | global error m / yaw deg | matches/update, nearest frame stamp')
frames=data('frontend_frame',lambda r:r['data']['epoch']==1);ft=np.array([x['stamp'] for x in frames])
for t in [45.452,88.764,89.18,90,91.4,92,93,93.968,94.14,95.36,96,97.108,97.808,98.164,98.36,98.8,99.364,105,112.8,113.372,114.564,114.98,115.952,116.764,117.184,118.364,130]:
 g=interp(truth_t,truth_p,t)[0];f=frames[np.argmin(abs(ft-t))]
 vals=[]
 for k in ['lio','global']:
  vals.append((float(np.interp(t,err[k+'_t'],err[k+'_norm'])),float(np.interp(t,err[k+'_t'],err[k+'_yaw']))))
 out(f'{t:8.3f} | {g[0]:8.3f} {g[1]:8.3f} {g[2]:6.3f} | {vals[0][0]:7.3f} / {vals[0][1]:6.2f} | {np.interp(t,post_t,post_e):7.3f} | {vals[1][0]:7.3f} / {vals[1][1]:6.2f} | {f["matches"]} / {f["lidar_updated"]}, {f["stamp"]:.3f}')
out('\nREJECTED FRONTEND FRAMES')
for f in frames:
 if f['stamp']>=baseline and not f['lidar_updated']:out(json.dumps({k:f[k] for k in ['stamp','begin','sequence','matches','lidar_updated','cloud_shape']}))
out('\nIMU NORM / RATE IN INCIDENT 90--100s')
imus=data('imu',lambda r:90<=r['data']['stamp']<=100);acc=np.array([x['accel'] for x in imus]);gyro=np.array([x['gyro'] for x in imus]);out('accel norm min/median/max '+str(np.quantile(np.linalg.norm(acc,axis=1),[0,.5,1])));out('gyro norm min/median/max '+str(np.quantile(np.linalg.norm(gyro,axis=1),[0,.5,1])))
out('\nRAW TRANSFORM / POSTERIOR MATCH COVERAGE')
# Compare prediction at posterior stamps, allowing expected forecast displacement by interpolation.
post_pred=interp(raw_t,raw_p,frame_t);delta=np.linalg.norm(post_pred[:,:3]-base_frame[:,:3],axis=1);mask=(frame_t>=90)&(frame_t<=100)
out('same stamp posterior-vs-predicted-base translation min/median/max '+str(np.quantile(delta[mask],[0,.5,1])))
# Local vehicle displacement at recovery, aligned by recorder wall solely to identify event simulation time.
out('\nRECOVERY LOGS AND GROUND TRUTH MOVEMENT')
for n,l in enumerate((logs/'loc.log').read_text().splitlines(),1):
 m=re.search(r'\[(17945889\d+\.\d+)\].*localization state -> NORMAL \(recovered.*delta_t=([\d.]+)m, delta_r_deg=([\d.]+)',l)
 if not m:continue
 wall=float(m[1]);states=select('/backend_node/localization_state');s=min(states,key=lambda r:abs(r['wall']-wall));t=s['sim'];
 if abs(s['wall']-wall)>.2:continue
 g=interp(truth_t,truth_p,[t-.1,t+.1]);dist=np.linalg.norm(g[1,:3]-g[0,:3]);angle=(R.from_quat(g[0,3:]).inv()*R.from_quat(g[1,3:])).magnitude()*180/np.pi
 out(f'loc.log:{n} wall={wall:.3f}, receive_sim={t:.3f}, correction={m[2]}m/{m[3]}deg, truth +/-0.1s displacement={dist:.6f}m rotation={angle:.6f}deg')
(p/'timeline.txt').write_text('\n'.join(lines)+'\n');(p/'input_checks.json').write_text(json.dumps(summary,indent=2)+'\n')