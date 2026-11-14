import json
from pathlib import Path
import numpy as np
p=Path('/home/yang/yang_nav/reports/navigation/2026-11-13/yaw_diagnosis/repro')
r=[json.loads(s) for s in (p/'chain_trace.jsonl').read_text().splitlines()]
start=next(x['wall'] for x in r if x['kind']=='yaw_ref'); end=start+25
kinds=['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel_collision_checked','/cmd_vel']
data={}
for k in kinds:
    a=[x for x in r if x['kind']==k and start-2<x['wall']<end+2]
    t=np.array([x['wall']-start for x in a]); v=np.array([x['data'][2] for x in a]);data[k]=(t,v)
    print(k,'wall sample interval p50 p90',np.percentile(np.diff(t),[50,90]),'clock age p50',np.median([x['t'] for x in a]))
grid=np.arange(0,25,.02)
for kin,kout in zip(kinds,kinds[1:]):
    t,v=data[kin];u,z=data[kout];target=np.interp(grid,u,z)
    score=[]
    for lag in np.arange(-.05,.81,.01):
        source=np.interp(grid-lag,t,v);score.append((float(np.corrcoef(source,target)[0,1]),float(lag),float(np.sqrt(np.mean((source-target)**2)))))
    print(kin,kout,max(score))
a=[x for x in r if x['kind']=='tf_pose' and start-2<x['wall']<end+2]; b=[x for x in r if x['kind']=='truth' and start-3<x['wall']<end+3]
t=np.array([x['wall']-start for x in b]);v=np.unwrap([x['data']['pose'][3] for x in b]); u=np.array([x['wall']-start for x in a]);z=np.unwrap([x['data'][3] for x in a]);target=np.interp(grid,u,z)
score=[]
for lag in np.arange(0,.61,.01):
    pred=np.interp(grid-lag,t,v);score.append((float(np.std(np.angle(np.exp(1j*(target-pred))))),float(lag)))
print('truth -> tf yaw best residual std rad, lag',min(score))
(p/'chain_delays.json').write_text(json.dumps({'window_wall_start':start,'window_seconds':25,'note':'Estimates by cross-correlation of received streams, not instrumented node processing durations.'},indent=2))