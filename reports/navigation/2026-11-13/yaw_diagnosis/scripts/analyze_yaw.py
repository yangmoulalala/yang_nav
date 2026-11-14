import json, math
from pathlib import Path
import numpy as np
p=Path('/home/yang/yang_nav/reports/navigation/2026-11-13/yaw_diagnosis')
rows=[json.loads(s) for s in (p/'open_loop_trace.jsonl').read_text().splitlines()]
out=[]
for name in dict.fromkeys(r['stage'] for r in rows):
    a=[r for r in rows if r['stage']==name]
    if not a:continue
    t=np.array([r['truth'][0] for r in a]); y=np.unwrap([r['truth'][4] for r in a]); m=(t>=t[0]+1)
    if m.sum()<3:continue
    slope=np.polyfit(t[m]-t[0],y[m],1)[0]; d=np.diff(y[m])/np.diff(t[m]); cmd=a[0]['cmd']
    residual=y[m]-(np.polyval(np.polyfit(t[m]-t[0],y[m],1),t[m]-t[0]))
    v={'stage':name,'command':cmd,'steady_wz':float(slope),'rate_std':float(np.std(d)),'yaw_residual_peak_to_peak_deg':float(np.ptp(residual)*180/np.pi),'yaw_delta_deg':float((y[-1]-y[0])*180/np.pi)}
    out.append(v)
(p/'open_loop_summary.json').write_text(json.dumps(out,indent=2))
for r in out:
    if r['stage'].startswith(('w','vx','vy')):print(r)
reg=json.loads(Path('/home/yang/yang_nav/maps/sim/terrain/terrain.json').read_text())['registration']; print('registration',reg)