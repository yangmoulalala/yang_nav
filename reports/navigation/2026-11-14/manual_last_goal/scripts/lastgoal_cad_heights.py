import sys,json,math
import numpy as np
sys.path.insert(0,'/home/yang/yang_nav/src/nav_terrain/scripts')
from terrain_map import TerrainMap,_quat_mat
from pathlib import Path
ws=Path('/home/yang/yang_nav')
t=TerrainMap.load(ws/'maps/sim/terrain/terrain.msgpack')
reg=json.loads((ws/'maps/sim/terrain/terrain.json').read_text())['registration']
# Registration Rz Ry Rx, mapping prior-map coordinates to Gazebo world.
r,p,y=np.radians(reg['rpy_deg']);cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
R=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
trans=np.array(reg['t_world_map'])
raw=(ws/'src/sentry_gazebo/models/rmuc_2027/meshes/field_collision.stl').read_bytes()
n=int(np.frombuffer(raw,dtype='<u4',count=1,offset=80)[0]);rec=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]);world=np.frombuffer(raw,dtype=rec,count=n,offset=84)['v'].astype(float)
tri=(world-trans)@R
lo=tri[:,:,:2].min(axis=1);hi=tri[:,:,:2].max(axis=1)
out=[]
for x,y in [(14.05,5.55),(14.05,5.65),(14.05,6.05),(13.9,5.1),(12.1,5.0),(8.,-5.55),(10.,-5.65),(16.85,.35),(7.6,-2.16),(11.,2.5)]:
    a=tri[(lo[:,0]<=x)&(hi[:,0]>=x)&(lo[:,1]<=y)&(hi[:,1]>=y)]
    v0=a[:,1,:2]-a[:,0,:2];v1=a[:,2,:2]-a[:,0,:2];v2=np.array([x,y])-a[:,0,:2]
    den=v0[:,0]*v1[:,1]-v1[:,0]*v0[:,1];good=abs(den)>1e-10
    a,v0,v1,v2,den=a[good],v0[good],v1[good],v2[good],den[good]
    u=(v2[:,0]*v1[:,1]-v1[:,0]*v2[:,1])/den;v=(v0[:,0]*v2[:,1]-v2[:,0]*v0[:,1])/den
    z=a[:,0,2]+u*(a[:,1,2]-a[:,0,2])+v*(a[:,2,2]-a[:,0,2]);z=np.unique(np.round(z[(u>=-1e-7)&(v>=-1e-7)&(u+v<=1+1e-7)],5))
    c,rr=np.floor((np.array([x,y])-t.origin)/t.resolution).astype(int);ground=float(t.elevation[rr,c])
    out.append(dict(x=x,y=y,label=int(t.terrain[rr,c]),grid=ground,cad_surfaces=z.tolist(),deltas=(z-ground).tolist()))
print(json.dumps(out,indent=2))
