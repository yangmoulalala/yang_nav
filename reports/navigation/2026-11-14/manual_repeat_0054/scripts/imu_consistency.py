from pathlib import Path
exec((Path('/home/yang/yang_nav/reports/navigation/2026-11-14/manual_repeat_0054/scripts/inspect_incident.py')).read_text().split('lines=[]')[0])
from scipy.integrate import cumulative_trapezoid
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ib=extra[0];bi=inv(ib)
# IMU origin truth from model/base pose and constant recorded base-to-IMU transform.
gt_sensor=np.array([mul(a,bi) for a in truth_p])
imus=data('imu');it=np.array([x['stamp'] for x in imus]);ac=np.array([x['accel'] for x in imus]);gy=np.array([x['gyro'] for x in imus]);igs=interp(truth_t,gt_sensor,it);world_acc=R.from_quat(igs[:,3:]).apply(ac)+np.array([0,0,-9.81])
# Use centered truth velocity over 96ms to reduce pose subsampling effects.
def true_v(ts,h=.048):return (interp(truth_t,gt_sensor,np.array(ts)+h)[:,:3]-interp(truth_t,gt_sensor,np.array(ts)-h)[:,:3])/(2*h)
results=[]
for a,b in [(45.6,55),(60,85),(90,93),(93,94),(94.3,96),(96,98.4),(98.6,110),(113.5,117.5)]:
 m=(it>=a)&(it<=b);tt=it[m];acc=world_acc[m];vi=true_v([tt[0]])[0];v=vi+cumulative_trapezoid(acc,tt,axis=0,initial=0);pi=interp(truth_t,gt_sensor,tt[0])[0,:3];pos=pi+cumulative_trapezoid(v,tt,axis=0,initial=0);gt=interp(truth_t,gt_sensor,tt);vel_gt=true_v(tt)
 results.append({'window':[a,b],'integrated_velocity_error_at_end_m_s':(v[-1]-vel_gt[-1]).tolist(),'integrated_position_error_at_end_m':(pos[-1]-gt[-1,:3]).tolist(),'accel_norm_max':float(np.linalg.norm(ac[m],axis=1).max())})
print(json.dumps(results,indent=2));(p/'imu_consistency_exploratory.json').write_text(json.dumps({'method':'Exploratory integration of world acceleration using ground-truth attitude and a fixed base-to-IMU extrinsic; 24ms truth interpolated, initial velocity from centered 96ms truth finite difference; noise and interpolation approximations remain. Not a proof of sensor or algorithm defect.','windows':results},indent=2)+'\n')
fig,ax=plt.subplots(3,1,figsize=(12,8),sharex=True)
m=(it>=90)&(it<=100);ax[0].plot(it[m],np.linalg.norm(ac[m],axis=1),lw=.7);ax[0].set_ylabel('IMU acceleration\nnorm (m/s2)')
vel=true_v(truth_t);m2=(truth_t>=90)&(truth_t<=100);ax[1].plot(truth_t[m2],np.linalg.norm(vel[m2],axis=1));ax[1].set_ylabel('True sensor\nspeed (m/s)')
for k,c,label in [('lio','#d45030','Raw LIO base TF'),('global','#2b67a0','Global base odometry')]:
 t=err[k+'_t'];mask=(t>=90)&(t<=100);ax[2].plot(t[mask],err[k+'_norm'][mask],label=label,color=c)
ax[2].set_ylabel('Position error (m)');ax[2].legend();ax[2].set_xlabel('Simulation time (s)')
for a in ax:a.grid(alpha=.2)
fig.tight_layout();fig.savefig(p/'incident_imu.png',dpi=150)