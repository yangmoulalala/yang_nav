import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,'/home/yang/yang_nav/src/nav_terrain/scripts')
from terrain_map import TerrainMap
from obstacle_filter import GroundFilter
t=TerrainMap.load('/home/yang/yang_nav/maps/sim/terrain/terrain.msgpack');f=GroundFilter(t)
rows=[]
for l in Path('/tmp/nav_lastgoal_clean/observations.npz').open():
    if l.startswith('#'):continue
    a=np.fromstring(l,sep=' ');p=a[3:].reshape(-1,3)
    if not len(p):continue
    sel=(abs(p[:,0]-14.05)<.40)&(abs(p[:,1]-5.75)<.70)
    p=p[sel]
    if len(p):
        g,_=f.sample(p[:,:2],0.)
        for q,h in zip(p,g):rows.append([a[0],a[1],*q,h,q[2]-h])
a=np.array(rows)
print('tunnel hits',len(a))
if len(a):
    for i,name in [(2,'x'),(3,'y'),(4,'z'),(5,'ground'),(6,'relative height')]:print(name,np.quantile(a[:,i],[0,.1,.5,.9,1]).tolist())
    for row in a[::max(1,len(a)//20)]:print(row.tolist())
np.savetxt('/tmp/lastgoal_tunnel_hits.csv',a,delimiter=',',header='wall,sim,x,y,z,ground,height')
