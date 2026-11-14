from pathlib import Path
import os,subprocess,signal,time,json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from localization_interfaces.msg import FrontendFrame
from rosgraph_msgs.msg import Clock
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose
out=Path('/home/yang/yang_nav/reports/navigation/2026-11-14/exit_pose_repeat/recorder_smoke');out.mkdir(exist_ok=True)
original=out/'localization_trace.jsonl';original.write_text('preserve me\n')
rclpy.init();node=Node('diagnostic_recorder_smoke_source');pub=node.create_publisher(FrontendFrame,'/frontend/frame',10);imu=node.create_publisher(Imu,'/lidar_mid360/imu',10);clock=node.create_publisher(Clock,'/clock',10)
gz=GzNode();gp=gz.advertise('/model/sentry/pose',Pose)
child=subprocess.Popen(['python3','/home/yang/yang_nav/tools/nav_regress/record_localization.py',str(out)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
t0=time.monotonic();i=0
try:
 while time.monotonic()-t0<4:
  i+=1;t=i*.02
  c=Clock();c.clock.sec=int(t);c.clock.nanosec=int((t-int(t))*1e9);clock.publish(c)
  f=FrontendFrame();f.odometry.header.stamp=c.clock;f.odometry.header.frame_id='lio_odom';f.odometry.child_frame_id='mid360_link';f.odometry.pose.pose.orientation.w=1.;f.sequence=i;pub.publish(f)
  m=Imu();m.header.stamp=c.clock;m.linear_acceleration.z=9.81;imu.publish(m)
  g=Pose();g.header.stamp.sec=c.clock.sec;g.header.stamp.nsec=c.clock.nanosec;g.orientation.w=1.;gp.publish(g)
  rclpy.spin_once(node,timeout_sec=.02)
finally:
 child.send_signal(signal.SIGINT);stdout,_=child.communicate(timeout=8);node.destroy_node();rclpy.shutdown()
print(stdout)
assert child.returncode==0,child.returncode
assert original.read_text()=='preserve me\n'
traces=list(out.glob('localization_trace_*.jsonl'));assert len(traces)==1
rows=[json.loads(l) for l in traces[0].read_text().splitlines()];kinds=sorted(set(r['kind'] for r in rows))
assert {'frontend_frame','truth','imu'}.issubset(set(kinds)),kinds
result={'exit_code':child.returncode,'preserved_existing_trace':True,'samples':len(rows),'kinds':kinds,'fixture':'Independent ROS and Gazebo transport publishers; no simulation or navigation stack.'}
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(result)