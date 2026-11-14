import json,math
import numpy as np
from pathlib import Path
p=Path('/home/yang/yang_nav/reports/navigation/2026-11-13/yaw_diagnosis/repro')
rows=[]
for s in (p/'chain_trace.jsonl').read_text().splitlines():
    try:rows.append(json.loads(s))
    except ValueError:pass
truth=[r for r in rows if r['kind']=='truth']; t=np.array([r['data']['stamp'] for r in truth]); y=np.unwrap([r['data']['pose'][3] for r in truth])
ref=[r for r in rows if r['kind']=='yaw_ref']; start=ref[0]['t']
grid=np.arange(max(t[0],start-1),min(t[-1],start+27),.05); yi=np.interp(grid,t,y); rate=(np.interp(grid+.08,t,y)-np.interp(grid-.08,t,y))/.16
print('ref start',start,'truth end',t[-1]);print('time, ref_deg, tf_yaw_deg, truth_yaw_deg, nav_wz, smooth_wz, final_wz, actual_wz')
def values(kind,col=None):
    a=[r for r in rows if r['kind']==kind]; x=np.array([r['t'] for r in a]); v=np.array([r['data'] if col is None else r['data'][col] for r in a]);return x,v
kinds=['yaw_ref','tf_pose','/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel_collision_checked','/cmd_vel']
data={k:values(k,None if k=='yaw_ref' else -1) for k in kinds}
for x in np.arange(start,start+15,.5):
    if x>t[-1]:break
    vals=[np.interp(x,*data[k]) for k in kinds]; act=np.interp(x,grid,rate)
    print('%.2f %.1f %.1f %.1f %.2f %.2f %.2f %.2f'%(x,vals[0]*180/np.pi,vals[1]*180/np.pi,np.interp(x,t,y)*180/np.pi,vals[2],vals[3],vals[-1],act))
for kind in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel_collision_checked','/cmd_vel']:
    x,v=data[kind]; masks=(grid>start)&(grid<min(t[-1]-.7,start+25))
    best=None
    for lag in np.arange(-.15,.81,.01):
        cmd=np.interp(grid[masks]-lag,x,v); actual=rate[masks]; corr=np.corrcoef(cmd,actual)[0,1]; err=np.sqrt(np.mean((cmd-actual)**2))
        if best is None or corr>best[0]:best=(float(corr),float(lag),float(err))
    print('best corr lag rmse',kind,best)