import json,re,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=Path('/home/yang/yang_nav/reports/navigation/2026-11-13/yaw_diagnosis')
result={}
fig,axs=plt.subplots(2,1,figsize=(12,8),sharex=True)
for name in ['repro','ab_latest_tf']:
    p=out/name;rows=[json.loads(s) for s in (p/'chain_trace.jsonl').read_text().splitlines()]
    first=next(r for r in rows if r['kind']=='tracker_text' and r['data'].startswith('slope_3+tunnel_3'))
    start=first['wall'];sim=first['t'];end=min(start+25,rows[-1]['wall']-.5)
    kinds=['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel_collision_checked','/cmd_vel'];data={};metrics={'start_sim':sim,'start_wall':start,'duration_wall':end-start}
    for k in kinds:
        a=[r for r in rows if r['kind']==k and start-2<r['wall']<end+2];t=np.array([r['wall']-start for r in a]);v=np.array([r['data'][2] for r in a]);data[k]=(t,v)
        metrics[k]={'median_interval_s':float(np.median(np.diff(t))),'p90_interval_s':float(np.percentile(np.diff(t),90))}
    grid=np.arange(0,end-start,.02)
    for kin,kout in zip(kinds,kinds[1:]):
        t,v=data[kin];u,z=data[kout];target=np.interp(grid,u,z);score=[]
        for lag in np.arange(0,.81,.01):
            source=np.interp(grid-lag,t,v);score.append((float(np.corrcoef(source,target)[0,1]),float(lag),float(np.sqrt(np.mean((source-target)**2)))))
        best=max(score);metrics[kin+' -> '+kout]={'correlation':best[0],'best_lag_s':best[1],'rmse':best[2]}
    a=[r for r in rows if r['kind']=='truth'];t=np.array([r['data']['stamp'] for r in a]);y=np.unwrap([r['data']['pose'][3] for r in a]);
    a=[r for r in rows if r['kind']=='yaw_ref'];x=np.array([r['t'] for r in a]);v=np.unwrap([r['data'] for r in a]);offset=2*np.pi*round((np.interp(sim,x,v)-np.interp(sim,t,y))/(2*np.pi));y+=offset
    axs[0].plot(t-sim,np.degrees(y),label=name+' truth')
    tr=[]
    for line in (p/'nav.log').read_text().splitlines():
        m=re.search(r'\[(\d+\.\d+)\].*ZTRACE slope_3\+tunnel_3 ph (\w+) .* ref ([-\d.]+).*e_yaw ([+\-\d.]+)',line)
        if m:tr.append((float(m[1]),m[2],float(m[3]),float(m[4])))
    tr=np.array([(a,c,d) for a,b,c,d in tr])
    if len(tr):
        trace_t=tr[:,0]-start; trace_ref=tr[:,1].copy(); trace_err=tr[:,2].copy(); trace_ref[np.r_[False,np.diff(trace_t)>.2]]=np.nan; trace_err[np.r_[False,np.diff(trace_t)>.2]]=np.nan
        axs[0].plot(trace_t,trace_ref,'--',alpha=.7,label=name+' reference')
    if len(tr):
        mask=(tr[:,0]>=start+5)&(tr[:,0]<=start+20); e=tr[mask,2]
        metrics['active_zone_error_wall_5_20s']={'n':int(len(e)),'max_abs_deg':float(np.max(np.abs(e))),'p95_abs_deg':float(np.percentile(np.abs(e),95))} if len(e) else None
        axs[1].plot(trace_t,trace_err,label=name+' heading error')
    metrics['goal_result_tail']=(p/'runner.log').read_text().splitlines()[-5:];result[name]=metrics
for ax in axs:ax.grid(alpha=.25);ax.legend(ncol=2)
axs[0].set_ylabel('Yaw (deg)');axs[0].set_ylim(-180,40);axs[0].set_title('Diagnostic A/B: current-time TF correction vs latest TF (one run each)')
axs[1].set_ylabel('Controller heading error (deg)');axs[1].set_xlabel('Seconds since entering tunnel_3 tracking window');axs[1].set_xlim(0,10)
fig.tight_layout();fig.savefig(out/'yaw_ab.png',dpi=150)
(out/'ab_metrics.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))