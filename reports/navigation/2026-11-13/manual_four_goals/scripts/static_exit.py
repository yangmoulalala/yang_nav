import json,math
from pathlib import Path
import numpy as np,yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
root=Path('/home/yang/yang_nav');out=root/'reports/navigation/2026-11-13/manual_four_goals';cfg=yaml.safe_load((root/'maps/sim/terrain/nav_map.yaml').read_text());g=np.flipud(np.asarray(Image.open(root/'maps/sim/terrain'/cfg['image'])));res=cfg['resolution'];ox,oy,_=cfg['origin']; lethal=(1-g/255.)>cfg['occupied_thresh'];dist=distance_transform_edt(~lethal)*res
lethal_ij=np.argwhere(lethal);lethal_xy=lethal_ij[:,::-1]*res+np.array([ox,oy])+res/2
foot=np.array([[.39,.31],[.39,-.31],[-.31,-.31],[-.31,.31]])
def collision(x,y,yaw):
    a=math.radians(yaw);rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]]);poly=foot@rot.T+[x,y];lo=poly.min(0);hi=poly.max(0);pts=[]
    i0,j0=np.floor((lo-[ox,oy])/res).astype(int);i1,j1=np.floor((hi-[ox,oy])/res).astype(int)
    for j in range(j0,j1+1):
        for i in range(i0,i1+1):
            if not lethal[j,i]:continue
            c=np.array([ox+(i+.5)*res,oy+(j+.5)*res]);sep=False
            for p,q in zip(poly,np.roll(poly,-1,axis=0)):
                n=np.array([p[1]-q[1],q[0]-p[0]]);v=poly@n;reach=.5*res*abs(n).sum();cp=c@n
                if v.max()<cp-reach or v.min()>cp+reach:sep=True;break
            if not sep:pts.append(c.tolist())
    return poly,pts
summary=[]
for label,x,y in [('goal3 estimate',4.52,-5.24),('goal4 estimate',4.59,-5.27),('registered Gazebo truth',4.836,-5.40)]:
    i,j=np.floor((np.array([x,y])-[ox,oy])/res).astype(int);poly,hits=collision(x,y,-90)
    row={'label':label,'x':x,'y':y,'assumed_yaw_deg':-90,'source_pixel':int(g[j,i]),'source_cell_edt_distance_m':float(dist[j,i]),'nearest_lethal_cell_center_distance_m':float(np.linalg.norm(lethal_xy-[x,y],axis=1).min()),'padded_footprint_overlap_lethal_cells':len(hits),'collision_cells':hits}
    summary.append(row);print(row)
(out/'static_map_checks.json').write_text(json.dumps(summary,indent=2))
z=np.load(out/'registered_truth.npz');fig,ax=plt.subplots(figsize=(10,9));ax.imshow(g,origin='lower',extent=[ox,ox+g.shape[1]*res,oy,oy+g.shape[0]*res],cmap='gray',vmin=0,vmax=255)
# Static inscribed inflation preview: exact production inflation is at 0.025m; this is source-grid evidence only.
ax.contour(ox+(np.arange(g.shape[1])+.5)*res,oy+(np.arange(g.shape[0])+.5)*res,dist,levels=[.31],colors='orange',linewidths=.7)
m=z['t']>1794580428;ax.plot(z['x'][m],z['y'][m],color='dodgerblue',lw=2,label='Gazebo truth (registered to map)')
for row,col in zip(summary,['red','magenta','limegreen']):
    x,y=row['x'],row['y'];poly,hits=collision(x,y,-90);ax.add_patch(Polygon(poly,fill=False,ec=col,lw=2));ax.plot(x,y,'o',color=col,label=row['label'])
ax.plot([4.05,6.16,6.45,6.47],[-2.10,-6.41,-6.13,-5.88],'kx',markersize=9)
for i,(x,y) in enumerate(zip([4.05,6.16,6.45,6.47],[-2.1,-6.41,-6.13,-5.88]),1):ax.text(x+.08,y+.08,'Goal '+str(i),fontsize=10)
ax.set_xlim(3.7,7);ax.set_ylim(-6.9,-2);ax.set_aspect('equal');ax.legend(loc='upper right',fontsize=8);ax.set_title('Latest manual run: exit stop and reported poses\nFootprints assume yaw -90 deg; orange: 0.31 m from static lethal cells');ax.set_xlabel('Map x (m)');ax.set_ylabel('Map y (m)');fig.tight_layout();fig.savefig(out/'exit_stop_map.png',dpi=150)