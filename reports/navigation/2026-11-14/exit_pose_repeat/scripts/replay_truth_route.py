from pathlib import Path
import json,math,sys,threading,time,re
import numpy as np
import rclpy
from rclpy.node import Node as RosNode
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
out=Path(sys.argv[1]);base=out.parent
nav=(base/'manual_logs/nav.log').read_text()
start=float(re.findall(r'\[(179\d{7}\.\d+)\].*Begin navigating',nav)[-1]);end=float(re.findall(r'\[(179\d{7}\.\d+)\].*Goal succeeded',nav)[-1])
path=[]
for l in (base/'manual_logs/truth.log').read_text().splitlines():
 try:
  v=list(map(float,l.split()))
  if len(v)==5 and start<=v[0]<=end+1:path.append([v[0]-start,v[1],v[2],math.radians(v[4])])
 except ValueError:pass
path=np.array(path);path[:,3]=np.unwrap(path[:,3]);np.save(out/'replay_reference.npy',path)
rclpy.init();node=RosNode('truth_route_replay_fixture',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
pub=node.create_publisher(Twist,'/cmd_vel',10);lock=threading.Lock();truth=None
log=(out/'driver_trace.jsonl').open('w',buffering=1)
def truth_cb(m):
 global truth
 q=m.orientation;y=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 with lock:truth=(m.header.stamp.sec+m.header.stamp.nsec*1e-9,m.position.x,m.position.y,y)
gz=GzNode();assert gz.subscribe(Pose,'/model/sentry/pose',truth_cb)
target_time=0.;beg=None;prev=None;done=False
wallbeg=time.monotonic()
try:
 while rclpy.ok() and time.monotonic()-wallbeg<120:
  rclpy.spin_once(node,timeout_sec=.015)
  with lock:g=truth
  if g is None:continue
  st,x,y,yaw=g
  if prev is not None and st<=prev:continue
  if beg is None:beg=st;prev=st;continue
  dt=min(.1,st-prev);prev=st
  ref=np.array([np.interp(target_time,path[:,0],path[:,k]) for k in (1,2,3)])
  vel=np.array([np.interp(target_time+.08,path[:,0],path[:,k])-ref[k-1] for k in (1,2,3)])/.08
  err=ref[:2]-[x,y];eyaw=(ref[2]-yaw+math.pi)%(2*math.pi)-math.pi
  rate=min(1.,.18/max(float(np.linalg.norm(err)),.18),.22/max(abs(eyaw),.22))
  v=vel[:2]*rate+2.0*err
  if np.linalg.norm(v)>1.35:v*=1.35/np.linalg.norm(v)
  w=float(np.clip(vel[2]*rate+2.0*eyaw,-1.2,1.2))
  cmd=Twist();cmd.linear.x=float(math.cos(yaw)*v[0]+math.sin(yaw)*v[1]);cmd.linear.y=float(-math.sin(yaw)*v[0]+math.cos(yaw)*v[1]);cmd.angular.z=w;pub.publish(cmd)
  log.write(json.dumps({'sim':st,'reference_time':target_time,'truth':[x,y,yaw],'target':ref.tolist(),'position_error':float(np.linalg.norm(err)),'yaw_error':eyaw,'cmd':[cmd.linear.x,cmd.linear.y,w]})+'\n')
  target_time+=dt*rate
  if target_time>=path[-1,0] and np.linalg.norm(err)<.12:done=True;break
  if st-beg>95:break
finally:
 for _ in range(10):pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
 log.close();node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 print(json.dumps({'replay_completed':done,'reference_time':target_time,'reference_duration':float(path[-1,0]),'simulation_duration':st-beg,'driver_uses_ground_truth_only_as_test_fixture':True}),flush=True)