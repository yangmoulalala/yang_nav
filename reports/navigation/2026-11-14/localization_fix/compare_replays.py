import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R,Slerp
p=Path(sys.argv[1]);rows=[json.loads(l) for l in (p/'localization_trace.jsonl').read_text().splitlines()]
def poses(kind,pred=lambda x:True):
 d={x['data']['stamp']:x['data']['pose'] for x in rows if x['kind']==kind and pred(x)};t=np.array(sorted(d));return t,np.array([d[x] for x in t])
def interp(t,x,ts):
 ts=np.clip(np.atleast_1d(ts),t[0],t[-1]);return np.c_[np.array([np.interp(ts,t,x[:,j]) for j in range(3)]).T,Slerp(t,R.from_quat(x[:,3:]))(ts).as_quat()]
def inv(a):
 r=R.from_quat(a[3:]).inv();return np.r_[r.apply(-a[:3]),r.as_quat()]
def mul(a,b):
 r=R.from_quat(a[3:]);return np.r_[a[:3]+r.apply(b[:3]),(r*R.from_quat(b[3:])).as_quat()]
truth_t,truth_p=poses('truth');raw_t,raw_p=poses('tf',lambda x:x['data']['parent']=='lio_odom');pred_t,pred_p=poses('frontend_prediction',lambda x:x['data']['epoch']==1)
baseline=next(x['sim'] for x in rows if x['kind']=='/cmd_vel' and np.linalg.norm(x['data'])>.01)-1
ib=mul(inv(pred_p[0]),interp(raw_t,raw_p,pred_t[0])[0]);output={}
for f in p.glob('replay_*.jsonl'):
 a=[json.loads(l) for l in f.read_text().splitlines() if json.loads(l)['epoch']==1];t=np.array([x['stamp'] for x in a]);xyz=np.array([mul(x['pose'],ib) for x in a]);b=float(np.clip(baseline,t[0],t[-1]));alignment=mul(interp(truth_t,truth_p,b)[0],inv(interp(t,xyz,b)[0]));xx=np.array([mul(alignment,x) for x in xyz]);g=interp(truth_t,truth_p,t);e=np.linalg.norm(xx[:,:3]-g[:,:3],axis=1);mask=t>=b;yu=(R.from_quat(xx[:,3:]).as_euler('xyz')[:,2]-R.from_quat(g[:,3:]).as_euler('xyz')[:,2]+np.pi)%(2*np.pi)-np.pi
 output[f.stem]={'frames':len(a),'max_position_error_m':float(max(e[mask])),'max_yaw_error_deg':float(max(abs(yu[mask]))*180/np.pi),'final_error_m':float(e[-1]),'rejected':sum(not x['updated'] for x in a)}
print(json.dumps(output,indent=2));(p/'replay_comparison.json').write_text(json.dumps(output,indent=2)+'\n')