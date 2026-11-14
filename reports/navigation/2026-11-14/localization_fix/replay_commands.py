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
cmds=[(x['sim']-start,x['data']) for x in rows if x['kind']=='/cmd_vel' and start<=x['sim']<=end]
beg=None;idx=0;last=-1;finished=False
try:
 while rclpy.ok():
  rclpy.spin_once(node,timeout_sec=.002)
  with lock:g=truth
  if g is None:continue
  st,x,y,yaw=g
  if st<=last:continue
  last=st
  if beg is None:beg=st
  t=st-beg
  while idx+1<len(cmds) and cmds[idx+1][0]<=t:idx+=1
  vx,vy,w=cmds[idx][1]
  cmd=Twist();cmd.linear.x=vx;cmd.linear.y=vy;cmd.angular.z=w;pub.publish(cmd)
  log.write(json.dumps({'sim':st,'reference_time':t,'truth':[x,y,yaw],'cmd':[vx,vy,w]})+'\n')
  if t>end-start:finished=True;break
finally:
 for _ in range(20):pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
 log.close();node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 print(json.dumps({'input_command_replay_completed':finished,'duration':st-beg}),flush=True)
