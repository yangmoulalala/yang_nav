import os, sys, time, math, json, threading, signal, subprocess
from pathlib import Path
import rclpy
from rclpy.node import Node as RosNode
from geometry_msgs.msg import Twist as RosTwist
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
OUT=Path('/home/yang/yang_nav/reports/navigation/2026-11-13/yaw_diagnosis')
OUT.mkdir(parents=True, exist_ok=True)
latest=None
rows=[]
label='startup'
command=(0.,0.,0.)
def truth(msg):
    global latest
    q=msg.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
    latest=(stamp,msg.position.x,msg.position.y,msg.position.z,yaw)
    rows.append({'stage':label,'cmd':command,'truth':latest,'wall':time.time()})
gz=Node(); gz.subscribe(Pose,'/model/sentry/pose',truth)
rclpy.init(); ros=RosNode('yaw_open_loop_probe'); pub=ros.create_publisher(RosTwist,'/cmd_vel',10)
proc=subprocess.Popen(['ros2','launch','sentry_gazebo','sim.launch.py','arena:=rmuc','uwb:=false','gui:=false','generated_dir:=/tmp/navdev/yaw_sim'],stdout=open(OUT/'physics_sim.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
def send(cmd):
    msg=RosTwist(); msg.linear.x,msg.linear.y,msg.angular.z=cmd; pub.publish(msg)
def stage(name,cmd,duration):
    global label,command
    label=name; command=cmd; start=latest[0]; wall=time.monotonic()
    while latest[0]-start<duration:
        if time.monotonic()-wall>duration*5+10: raise RuntimeError('simulation stalled')
        send(cmd); rclpy.spin_once(ros,timeout_sec=0); time.sleep(.02)
    print(name,latest,flush=True)
try:
    deadline=time.monotonic()+50
    while latest is None and time.monotonic()<deadline: time.sleep(.1)
    if latest is None: raise RuntimeError('no Gazebo truth')
    stage('settle',(0.,0.,0.),3)
    for rate in [.3,-.3,.6,-.6,1.,-1.]:
        stage('w%+.1f'%rate,(0.,0.,rate),5)
        stage('stop_w%+.1f'%rate,(0.,0.,0.),2)
    for name,cmd in [('vx+.3',(.3,0.,0.)),('vx-.3',(-.3,0.,0.)),('vy+.3',(0.,.3,0.)),('vy-.3',(0.,-.3,0.))]:
        stage(name,cmd,3); stage('stop_'+name,(0.,0.,0.),2)
finally:
    for i in range(10): send((0.,0.,0.)); time.sleep(.03)
    (OUT/'open_loop_trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    os.killpg(proc.pid,signal.SIGINT)
    try: proc.wait(timeout=12)
    except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGTERM); proc.wait(timeout=8)
    ros.destroy_node(); rclpy.shutdown()
