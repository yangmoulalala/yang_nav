import json, math,re,hashlib,shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('/home/yang/yang_nav');out=root/'reports/navigation/2026-11-13/yaw_diagnosis'
rows=[json.loads(s) for s in (out/'repro/chain_trace.jsonl').read_text().splitlines()]
truth=[r for r in rows if r['kind']=='truth'];t=np.array([r['data']['stamp'] for r in truth]);yaw=np.unwrap([r['data']['pose'][3] for r in truth]);begin=next(r['t'] for r in rows if r['kind']=='yaw_ref')
def val(k,col=-1):
    a=[r for r in rows if r['kind']==k];return np.array([r['t'] for r in a]),np.array([r['data'] if col is None else r['data'][col] for r in a])
grid=np.arange(begin,begin+25,.02);rate=(np.interp(grid+.08,t,yaw)-np.interp(grid-.08,t,yaw))/.16
kin=['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel_collision_checked','/cmd_vel'];stats={}
for k in kin:
    x,v=val(k);scores=[]
    for lag in np.arange(0,.81,.01):
        c=np.interp(grid-lag,x,v);scores.append((float(np.corrcoef(c,rate)[0,1]),float(lag),float(np.sqrt(np.mean((c-rate)**2)))))
    best=max(scores);stats[k]={'correlation':best[0],'best_lag_s':best[1],'wz_rmse_rad_s':best[2]}
a=63.4;b=67.3
m=(t>=a)&(t<=b);s=np.interp(np.arange(a,b,.02),*val('yaw_ref',None));stats['fixed_reference_window']={'sim_t_range':[a,b],'reference_range_deg':list(np.degrees([s.min(),s.max()])),'truth_range_deg':list(np.degrees([yaw[m].min(),yaw[m].max()]))}
(out/'repro_metrics.json').write_text(json.dumps(stats,indent=2))
fig,axs=plt.subplots(3,1,figsize=(13,10),sharex=True)
for k,ls in [('yaw_ref','--'),('tf_pose',':')]:
    x,v=val(k,None if k=='yaw_ref' else -1);axs[0].plot(x-begin,np.degrees(np.unwrap(v)),ls,label=k)
axs[0].plot(t-begin,np.degrees(yaw),label='Gazebo truth',lw=1.5);axs[0].set_ylim(-160,40);axs[0].set_ylabel('Yaw (deg)');axs[0].legend(ncol=3);axs[0].set_title('Original configuration: yaw oscillation before tunnel_3')
for k in ['/cmd_vel_nav','/cmd_vel']:
    x,v=val(k);axs[1].plot(x-begin,v,label=k)
axs[1].plot(grid-begin,rate,label='truth yaw rate',alpha=.8);axs[1].set_ylabel('Yaw rate (rad/s)');axs[1].legend(ncol=3)
for k in ['/cmd_vel_smoothed','/cmd_vel_collision_checked','/cmd_vel']:
    x,v=val(k);axs[2].step(x-begin,v,where='post',label=k,lw=1.2)
axs[2].set_ylabel('Yaw rate (rad/s)');axs[2].legend(ncol=3);axs[2].set_xlabel('Simulation seconds since first zone reference');axs[2].set_xlim(0,12)
for ax in axs:ax.grid(alpha=.25)
fig.tight_layout();fig.savefig(out/'yaw_chain.png',dpi=150)
print(json.dumps(stats,indent=2))
# Preserve exact scripts and diagnostic parameter overrides; production files were not edited.
scripts=out/'scripts';scripts.mkdir(exist_ok=True)
for name in ['yaw_open_loop.py','yaw_open_loop.sh','record_yaw_chain.py','yaw_repro.sh','yaw_ab.sh','run_nav_trace.sh','run_nav_ab.sh','yaw_nav2.yaml','yaw_nav2_ab.yaml','analyze_yaw.py','analyze_chain.py','chain_delays.py','make_evidence.py']:
    src=Path('/tmp/navdev')/name
    if src.exists():shutil.copy2(src,scripts/name)
files=['src/nav_behaviors/src/zone_tracker.cpp','src/nav_behaviors/src/zone_aware_controller.cpp','src/nav_bringup/params/sim/nav2.yaml','src/nav_bringup/behavior_trees/navigate_to_pose.xml','src/nav_safety/src/cmd_gate.cpp','src/sentry_gazebo/sentry_gazebo/sdf_builder.py']
(out/'source_hashes.json').write_text(json.dumps({p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in files},indent=2))