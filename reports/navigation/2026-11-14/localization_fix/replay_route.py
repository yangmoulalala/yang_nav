from pathlib import Path
import json,math,sys,threading,time,re
import numpy as np
import rclpy
from rclpy.node import Node as RosNode
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
out=Path(sys.argv[1]);base=Path('/home/yang/yang_nav/reports/navigation/2026-11-14/manual_repeat_0054/logs')
rows=[json.loads(l) for l in (base/'localization_trace.jsonl').read_text().splitlines()]
start=45.452;end=118.4
path=np.array([[a['data']['stamp']-start,*a['data']['pose'][:2],math.atan2(2*(a['data']['pose'][6]*a['data']['pose'][5]+a['data']['pose'][3]*a['data']['pose'][4]),1-2*(a['data']['pose'][4]**2+a['data']['pose'][5]**2))] for a in rows if a['kind']=='truth' and start<=a['data']['stamp']<=end])
path[:,3]=np.unwrap(path[:,3]);np.save(out/'replay_reference.npy',path)
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
 while rclpy.ok() and time.monotonic()-wallbeg<150:
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
  if np.linalg.norm(v)>1.6:v*=1.6/np.linalg.norm(v)
  w=float(np.clip(vel[2]*rate+2.0*eyaw,-1.2,1.2))
  cmd=Twist();cmd.linear.x=float(math.cos(yaw)*v[0]+math.sin(yaw)*v[1]);cmd.linear.y=float(-math.sin(yaw)*v[0]+math.cos(yaw)*v[1]);cmd.angular.z=w;pub.publish(cmd)
  log.write(json.dumps({'sim':st,'reference_time':target_time,'truth':[x,y,yaw],'target':ref.tolist(),'position_error':float(np.linalg.norm(err)),'yaw_error':eyaw,'cmd':[cmd.linear.x,cmd.linear.y,w]})+'\n')
  target_time+=dt*rate
  if target_time>=path[-1,0] and np.linalg.norm(err)<.12:done=True;break
  if st-beg>125:break
finally:
 for _ in range(10):pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
 log.close();node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 print(json.dumps({'replay_completed':done,'reference_time':target_time,'reference_duration':float(path[-1,0]),'simulation_duration':st-beg,'driver_uses_ground_truth_only_as_test_fixture':True}),flush=True)