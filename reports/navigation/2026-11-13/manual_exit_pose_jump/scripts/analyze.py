from pathlib import Path
import re,math,json,datetime,statistics
out=Path(__file__).resolve().parents[1]
def tm(t): return datetime.datetime.fromtimestamp(t,datetime.timezone(datetime.timedelta(hours=8))).strftime('%H:%M:%S.%f')[:-3]
truth=[]
for n,line in enumerate((out/'logs/truth.log').read_text().splitlines(),1):
 try:
  v=list(map(float,line.split()))
  if len(v)==5 and v[0]>1794584240: truth.append((n,*v))
 except ValueError: pass
print('truth',len(truth),'first/last',truth[0],truth[-1])
steps=[]; rollbacks=[]
for a,b in zip(truth,truth[1:]):
 dt=b[1]-a[1]; d=math.dist(a[2:5],b[2:5]); yaw=abs((b[5]-a[5]+180)%360-180)
 steps.append((d,yaw,dt,a[1],b[1]))
 if dt<=0: rollbacks.append({'before':a[1],'after':b[1],'dt':dt})
print('truth largest xyz step',max(steps),'max yaw step',max(steps,key=lambda x:x[1]),'rollbacks',rollbacks)
print('TRUTH MAP COARSE')
tx,ty=-9.415037765663683,-.22529290326302126
alpha=math.radians(-.46518737473210825)
for t in range(1794584298,1794584342,2):
 r=min(truth,key=lambda r:abs(r[1]-t))
 dx,dy=r[2]-tx,r[3]-ty
 print(tm(r[1]),'mapxy',round(math.cos(alpha)*dx+math.sin(alpha)*dy,3),round(-math.sin(alpha)*dx+math.cos(alpha)*dy,3),'wz/yaw',r[4],r[5])
loc=[];states=[];recoveries=[];committed=[];warn=[]
for n,l in enumerate((out/'logs/loc.log').read_text().splitlines(),1):
 m=re.search(r'\[(179\d{7}\.\d+)\]',l)
 if not m: continue
 t=float(m[1])
 if t<1794584240: continue
 loc.append((t,n,l))
 s=re.search(r'localization state -> (\w+)',l)
 if s: states.append({'epoch':t,'time':tm(t),'state':s[1],'line':n,'message':l})
 c=re.search(r'delta_t=([\d.e+-]+)m, delta_r_deg=([\d.e+-]+)',l)
 if c and ('recovered via' in l or 'global correction committed' in l):
  e={'epoch':t,'time':tm(t),'translation_3d_m':float(c[1]),'rotation_3d_deg':float(c[2]),'line':n}
  (recoveries if 'recovered via' in l else committed).append(e)
 if 'WARN' in l and not re.search(r'factor conflicted|localization state|global observation rejected|ambiguous',l):warn.append((t,n,l))
print('STATES')
for e in states:
 if e['epoch']>1794584298: print(e['time'],e['state'],'line',e['line'])
print('RECOVERIES')
for e in recoveries:
 print(e)
 if e['epoch']>1794584298:
  br=[r for r in truth if e['epoch']-.12<=r[1]<=e['epoch']+.12]
  if len(br)>1: print(' adjacent truth span',round(br[-1][1]-br[0][1],3),'xyz motion',round(math.dist(br[0][2:5],br[-1][2:5]),4),'yaw',round((br[-1][5]-br[0][5]+180)%360-180,3))
print('NONMATCH WARNINGS')
for t,n,l in warn:
 if t>1794584298: print(tm(t),n,l.split(']: ',1)[-1])
summary={'session_wall_start_lower_bound':1794584240,'goal_epoch':1794584298.859717147,'goal_map_xy':[6.93,-1.53], 'goal_succeeded_epoch':1794584339.999598424,'truth_samples':len(truth),'truth_max_xyz_step_m':max(s[0] for s in steps),'truth_max_yaw_step_deg':max(s[1] for s in steps),'truth_wall_clock_rollbacks':rollbacks,'states':states,'recoveries':recoveries,'committed_corrections':committed}
(out/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
(out/'timeline.txt').write_text('\n'.join(f"{tm(t)} loc.log:{n} {l}" for t,n,l in loc if t>=1794584298 and re.search(r'localization state|recovery candidate|ambiguous result|IMU|invalid ZLIO',l))+'\n')