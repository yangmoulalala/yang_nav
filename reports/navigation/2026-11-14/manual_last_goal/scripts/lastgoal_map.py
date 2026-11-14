import json, math
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ws=Path('/home/yang/yang_nav')
run=Path('/tmp/nav_manual_lastgoal_replay')
meta=yaml.safe_load((ws/'maps/sim/terrain/nav_map.yaml').read_text())
img=np.array(Image.open(ws/'maps/sim/terrain'/meta['image']))[::-1]
res=meta['resolution']; ox,oy=meta['origin'][:2]
fig,ax=plt.subplots(figsize=(12,9))
ax.imshow(img,origin='lower',extent=(ox,ox+res*img.shape[1],oy,oy+res*img.shape[0]),cmap='gray',vmin=0,vmax=255)
plans=[json.loads(l) for l in (run/'plans.txt.jsonl').open()]
plans=[p for p in plans if p['topic']=='/plan_smoothed' and p['poses'][-1][0]>16]
for p,col,lab in [(plans[0],'tab:blue','North approach'),(next(p for p in plans if len(p['poses'])>1100),'tab:orange','South detour')]:
    a=np.array(p['poses']);ax.plot(a[:,0],a[:,1],color=col,lw=1.8,label=lab)
zones=yaml.safe_load((ws/'maps/sim/terrain/zones.yaml').read_text())['zones']
for z in zones:
    x,y=z['centre']
    if x<9:continue
    a=math.radians(z['yaw_deg']);d,w=z['size'];c,s=math.cos(a),math.sin(a)
    corners=np.array([[-d/2,-w/2],[d/2,-w/2],[d/2,w/2],[-d/2,w/2],[-d/2,-w/2]])@np.array([[c,s],[-s,c]])+[x,y]
    ax.plot(corners[:,0],corners[:,1],color='green',alpha=.45,lw=.8)
    ax.text(x,y,z['id'],fontsize=7,color='darkgreen')
ax.scatter([7.6,16.9],[ -2.16,.34],c=['purple','red'],s=45,zorder=4)
ax.set(xlim=(7,19),ylim=(-6.6,7.3),xlabel='map x (m)',ylabel='map y (m)',title='Last goal: accepted routes alternate across the central wall')
ax.legend();ax.grid(alpha=.15);fig.tight_layout();fig.savefig('/tmp/lastgoal_routes.png',dpi=140)

# Static-vs-live cells intersecting the first reported colliding padded body.
def contacts(path,pose):
    m=np.load(path);a=m['data'].reshape(int(m['h']),int(m['w']))
    rr,cc=np.where(a>=100);xx=float(m['ox'])+(cc+.5)*float(m['res']);yy=float(m['oy'])+(rr+.5)*float(m['res'])
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);u=(xx-x)*c+(yy-y)*s;v=-(xx-x)*s+(yy-y)*c
    # Conservative bounding rectangle around cells, for diagnosis only.
    hit=(u>=-.3225-.04)&(u<=.4025+.04)&(abs(v)<=.3225+.04)
    cells=[]
    for px,py in zip(xx[hit],yy[hit]):
        row=int((py-oy)/res);col=int((px-ox)/res)
        cells.append([round(px,4),round(py,4),int(img[row,col])])
    return {'snapshot':str(path),'wall':float(m['t']),'stamp':float(m['stamp']),'cells_xy_static_gray':cells}
files=list((run/'costmaps').glob('global*.npz'))
best=min(files,key=lambda p:abs(float(np.load(p)['t'])-1794655000.31))
print(json.dumps(contacts(best,(14.049,5.752,math.radians(90.4))),indent=2))
