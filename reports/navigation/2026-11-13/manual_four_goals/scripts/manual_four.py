from pathlib import Path
import re,json,math
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
from PIL import Image
import yaml
root=Path('/home/yang/yang_nav');out=root/'reports/navigation/2026-11-13/manual_four_goals';out.mkdir(exist_ok=True)
nav=(out/'nav.log').read_text();loc=(out/'loc.log').read_text();start=1794580380;interesting=[]
for n,line in enumerate(nav.splitlines(),1):
    m=re.search(r'\[(\d+\.\d+)\]',line)
    if not m or float(m[1])<start:continue
    if any(k in line for k in ['Begin navigating','Goal succeeded','Goal failed','Start occupied','holding:','released','done,','window,','Failed to make progress','ZoneRecovery: attempt','giving up','Controller patience exceeded']):
        stamp=float(m[1]);time=datetime.fromtimestamp(stamp,ZoneInfo('Asia/Shanghai')).strftime('%H:%M:%S.%f')[:12]
        interesting.append({'line':n,'stamp':stamp,'local_time':time,'text':line.split(']: ',1)[-1]})
(out/'timeline.json').write_text(json.dumps(interesting,ensure_ascii=False,indent=2))
for row in interesting:print(row['line'],row['local_time'],row['text'])
truth=np.loadtxt(out/'truth.log');truth=truth[truth[:,0]>=start];reg=json.loads((root/'maps/sim/terrain/terrain.json').read_text())['registration'];alpha=math.radians(reg['rpy_deg'][2]);tx,ty,_=reg['t_world_map']
t=truth[:,0];dx=truth[:,1]-tx;dy=truth[:,2]-ty;mx=math.cos(alpha)*dx+math.sin(alpha)*dy;my=-math.sin(alpha)*dx+math.cos(alpha)*dy;myaw=truth[:,4]-reg['rpy_deg'][2]
print('Registered truth: timestamp map_x map_y map_yaw')
for stamp in [1794580430,1794580432,1794580443,1794580452,1794580461.56,1794580462.64,1794580472.32,1794580480.57,1794580485.71,1794580498.95]:print(stamp,*(round(float(np.interp(stamp,t,v)),3) for v in [mx,my,myaw]))
cfg=yaml.safe_load((root/'maps/sim/terrain/nav_map.yaml').read_text());grid=np.flipud(np.asarray(Image.open(root/'maps/sim/terrain'/cfg['image'])));res=cfg['resolution'];ox,oy,_=cfg['origin']
def cell(x,y):
    j=int(math.floor((x-ox)/res));i=int(math.floor((y-oy)/res));return int(grid[i,j]) if 0<=i<grid.shape[0] and 0<=j<grid.shape[1] else -1
print('Static source pixels (0 obstacle,255 free):')
for x,y in [(4.52,-5.24),(4.54,-5.20),(4.59,-5.23),(4.59,-5.27)]:print(x,y,cell(x,y))
np.savez_compressed(out/'registered_truth.npz',t=t,x=mx,y=my,yaw=myaw)
print('Localization transitions:')
for n,line in enumerate(loc.splitlines(),1):
    m=re.search(r'\[(\d+\.\d+)\]',line)
    if m and float(m[1])>1794580413 and 'localization state ->' in line:print(n,line)