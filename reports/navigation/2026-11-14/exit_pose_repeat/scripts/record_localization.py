from pathlib import Path
import json,math,sys,threading,time
import rclpy
from rclpy.node import Node as RosNode
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from localization_interfaces.msg import FrontendFrame, FrontendOdometry
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose

out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
f=(out/'localization_trace.jsonl').open('w',buffering=1);lock=threading.Lock()
rclpy.init();node=RosNode('localization_diagnostic_recorder',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
last={}
def stamp(s): return s.sec+s.nanosec*1e-9
def pose(p):return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
def emit(kind,data):
 with lock:f.write(json.dumps({'kind':kind,'sim':node.get_clock().now().nanoseconds*1e-9,'monotonic':time.monotonic(),'wall':time.time(),'data':data},allow_nan=False)+'\n')
def odom_data(m):
 return {'stamp':stamp(m.header.stamp),'parent':m.header.frame_id,'child':m.child_frame_id,'pose':pose(m.pose.pose),'cov':list(m.pose.covariance),'velocity':[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z,m.twist.twist.angular.x,m.twist.twist.angular.y,m.twist.twist.angular.z]}
def frame(m):
 d=odom_data(m.odometry);d.update({'epoch':m.epoch,'sequence':m.sequence,'instance_id':m.instance_id,'begin':stamp(m.begin),'matches':m.matches,'lidar_updated':m.lidar_updated,'cloud_stamp':stamp(m.cloud.header.stamp),'cloud_frame':m.cloud.header.frame_id,'cloud_shape':[m.cloud.height,m.cloud.width],'cloud_point_step':m.cloud.point_step})
 emit('frontend_frame',d)
def prediction(m):
 t=stamp(m.odometry.header.stamp)
 if t-last.get('prediction',-1)<.025 and t>=last.get('prediction',-1):return
 last['prediction']=t;d=odom_data(m.odometry);d['epoch']=m.epoch;emit('frontend_prediction',d)
def tfs(m,static=False):
 for a in m.transforms:
  if static or (a.header.frame_id,a.child_frame_id) in [('map','lio_odom'),('lio_odom','base_link')]:
   p=a.transform.translation;q=a.transform.rotation
   emit('tf_static' if static else 'tf',{'stamp':stamp(a.header.stamp),'parent':a.header.frame_id,'child':a.child_frame_id,'pose':[p.x,p.y,p.z,q.x,q.y,q.z,q.w]})
subs=[node.create_subscription(FrontendFrame,'/frontend/frame',frame,qos_profile_sensor_data),node.create_subscription(FrontendOdometry,'/frontend/odometry',prediction,qos_profile_sensor_data),node.create_subscription(TFMessage,'/tf',tfs,200),node.create_subscription(TFMessage,'/tf_static',lambda m:tfs(m,True),QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)),node.create_subscription(Odometry,'/localization/odometry',lambda m:emit('localization_odom',odom_data(m)),100)]
for name in ['/backend_node/localization_state','/nav_zone_state']:
 subs.append(node.create_subscription(String,name,lambda m,k=name:emit(k,m.data),100))
for name in ['/cmd_vel_nav','/cmd_vel']:
 subs.append(node.create_subscription(Twist,name,lambda m,k=name:emit(k,[m.linear.x,m.linear.y,m.angular.z]),100))
lasttruth=-1.
def truth(m):
 global lasttruth
 t=m.header.stamp.sec+m.header.stamp.nsec*1e-9
 if t-lasttruth<.02 and t>=lasttruth:return
 lasttruth=t;emit('truth',{'stamp':t,'pose':pose(m)})
gz=GzNode();assert gz.subscribe(Pose,'/model/sentry/pose',truth)
emit('recorder_started',{'topics':['/frontend/frame','/frontend/odometry','/tf','/tf_static','/localization/odometry','/backend_node/localization_state','Gazebo /model/sentry/pose']})
try:rclpy.spin(node)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 f.close()