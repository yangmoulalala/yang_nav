#!/usr/bin/env python3
"""Preserve normalized, timestamped LiDAR and IMU inputs for deterministic core replay."""
import sys,struct,json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Imu
from sensor_msgs_py import point_cloud2
from localization_interfaces.msg import FrontendSession
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
f=(out/'inputs.bin').open('xb');f.write(b'ZLIOIN01');counts={'imu':0,'cloud':0,'reset':0};epoch=None
rclpy.init();node=Node('localization_input_fixture',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
def imu(m):
 f.write(b'I'+struct.pack('<q6d',m.header.stamp.sec*10**9+m.header.stamp.nanosec,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z));counts['imu']+=1
def cloud(m):
 a=point_cloud2.read_points(m,field_names=['x','y','z','offset_time']);a=a.reshape(-1);b=np.empty(len(a),dtype=[('time','<i8'),('x','<f8'),('y','<f8'),('z','<f8')]);b['time']=m.header.stamp.sec*10**9+m.header.stamp.nanosec+a['offset_time'].astype(np.int64)
 for k in ['x','y','z']:b[k]=a[k]
 f.write(b'C'+struct.pack('<I',len(b))+b.tobytes());counts['cloud']+=1
 f.flush()
def session(m):
 global epoch
 if epoch!=m.epoch:
  epoch=m.epoch;f.write(b'R'+struct.pack('<Q',epoch));counts['reset']+=1
subs=[node.create_subscription(Imu,'/lidar_mid360/imu',imu,QoSProfile(depth=4000,reliability=ReliabilityPolicy.BEST_EFFORT)),node.create_subscription(PointCloud2,'/lidar/points',cloud,QoSProfile(depth=200,reliability=ReliabilityPolicy.RELIABLE)),node.create_subscription(FrontendSession,'/frontend/session',session,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
try:rclpy.spin(node)
except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
finally:
 f.close();(out/'input_counts.json').write_text(json.dumps(counts,indent=2));node.destroy_node()
 if rclpy.ok():rclpy.shutdown()