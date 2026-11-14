import time, json, math, threading, sys
from pathlib import Path
import rclpy
from rclpy.node import Node as RN
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer,TransformListener
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
out=Path(sys.argv[1]); out.mkdir(parents=True,exist_ok=True)
f=open(out/'chain_trace.jsonl','w',buffering=1); lock=threading.Lock()
rclpy.init(); ros=RN('yaw_chain_recorder',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
tf=Buffer(); listener=TransformListener(tf,ros)
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def emit(kind,data):
    with lock: f.write(json.dumps({'wall':time.time(),'t':ros.get_clock().now().nanoseconds*1e-9,'kind':kind,'data':data})+'\n')
def tw(m):return [m.linear.x,m.linear.y,m.angular.z]
def odom(m):emit('odom',{'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'pose':[m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)],'twist':tw(m.twist.twist)})
def markers(m):
    for a in m.markers:
        if a.action!=0:continue
        if a.ns=='tracker_state':emit('tracker_text',a.text)
        if a.ns=='tracker_yaw_ref':emit('yaw_ref',yaw(a.pose.orientation))
def path(m,topic):emit(topic,[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in m.poses])
subs=[]
for name in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel_collision_checked','/cmd_vel']:
    subs.append(ros.create_subscription(Twist,name,lambda m,k=name:emit(k,tw(m)),100))
for name in ['/nav_zone_state','/nav_mode','/backend_node/localization_state']:
    subs.append(ros.create_subscription(String,name,lambda m,k=name:emit(k,m.data),20))
subs.append(ros.create_subscription(Odometry,'/localization/odometry',odom,100))
subs.append(ros.create_subscription(MarkerArray,'/nav_zones/tracker',markers,20))
for topic in ['/plan','/local_plan']:
    subs.append(ros.create_subscription(NavPath,topic,lambda m,k=topic:path(m,k),10))
lasttruth=-1.
def truth(m):
    global lasttruth
    t=m.header.stamp.sec+m.header.stamp.nsec*1e-9
    if t-lasttruth<.02 and t>=lasttruth:return
    lasttruth=t;emit('truth',{'stamp':t,'pose':[m.position.x,m.position.y,m.position.z,yaw(m.orientation)]})
gz=Node(); gz.subscribe(Pose,'/model/sentry/pose',truth)
def tick():
    try:
        m=tf.lookup_transform('map','base_link',Time());p=m.transform.translation
        emit('tf_pose',[p.x,p.y,p.z,yaw(m.transform.rotation)])
    except Exception:pass
ros.create_timer(.05,tick)
try:rclpy.spin(ros)
except KeyboardInterrupt:pass
finally:f.close();ros.destroy_node();rclpy.shutdown()