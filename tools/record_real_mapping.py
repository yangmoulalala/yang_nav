#!/usr/bin/env python3
"""Record raw sensors and localization during hand-controlled real-robot mapping.

Run before moving the robot; Ctrl-C finalizes the bag. Start drivers/mapping in a
separate terminal. This tool never launches navigation or publishes a command.
Replay raw input on an isolated ROS domain; recorded /cmd_vel is included for debug.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time

WS=Path(__file__).resolve().parents[1]
TOPICS='''/livox/lidar /livox/imu /unilidar/cloud /lidar/points
/tf /tf_static /robot_description /joint_states /chassis/joint_states
/odometry /lidar_odometry /frontend/frame /frontend/odometry /frontend/session
/localization/odometry /localization/lio_world_cloud
/backend_node/localization_state /backend_node/localization_pose
/backend_node/odom_path /backend_node/optimized_path
/backend_node/debug/aligned_source /backend_node/debug/target
/cmd_vel /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_collision_checked
/uwb_pose /diagnostics /parameter_events /rosout /localization_start'''.split()


def capture(command, timeout=8):
    try:
        p=subprocess.run(command,cwd=WS,text=True,stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT,timeout=timeout)
        return p.stdout, p.returncode
    except (OSError,subprocess.TimeoutExpired) as e:return str(e), -1


def snapshot(out, phase):
    directory=out/phase;directory.mkdir(exist_ok=True)
    for name,command in [('topics',['ros2','topic','list','-t']),
                         ('nodes',['ros2','node','list']),
                         ('services',['ros2','service','list','-t'])]:
        text,code=capture(command);(directory/(name+'.txt')).write_text(text)
    nodes=(directory/'nodes.txt').read_text().splitlines()
    for index,node in enumerate(sorted(set(n.strip() for n in nodes if n.startswith('/')))):
        text,code=capture(['ros2','param','dump',node],timeout=3)
        (directory/(f'params_{index}_'+node.strip('/').replace('/','_')+'.yaml')).write_text(text)


def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--output',type=Path,help='new session directory; default bags/real_mapping/<timestamp>')
    ap.add_argument('--topic',action='append',default=[],help='extra raw sensor/chassis topic; repeat for remapped inputs')
    ap.add_argument('--all',action='store_true',help='record every discovered topic, including hidden topics')
    ap.add_argument('--duration',type=float,default=0,help='optional recording duration in seconds; 0 until Ctrl-C')
    ap.add_argument('--storage',choices=['mcap','sqlite3'],default='mcap')
    ap.add_argument('--note',default='',help='hardware, map route, known symptom or experiment note')
    ap.add_argument('--dry-run',action='store_true',help='print topic list without starting ROS or creating files')
    args=ap.parse_args()
    if args.duration<0:ap.error('--duration must be nonnegative')
    topics=sorted(set(TOPICS+args.topic))
    if args.dry_run:
        print(json.dumps(dict(topics=topics,all_topics=args.all,storage=args.storage,use_sim_time=False),indent=2));return 0
    import rosbag2_py
    if args.storage not in rosbag2_py.get_registered_writers():
        ap.error(f'{args.storage} writer unavailable; installed writers: {rosbag2_py.get_registered_writers()}')
    out=(args.output or WS/'bags/real_mapping'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')).expanduser().resolve()
    out.mkdir(parents=True,exist_ok=False)
    config=out/'config';config.mkdir()
    sources=['src/z-localization/bringup/config/real','src/z-localization/bringup/launch',
             'src/sentry_description/urdf','src/nav_bringup','src/nav_behaviors','src/nav_safety','src/nav_terrain']
    for relative in sources:
        source=WS/relative
        if source.exists():shutil.copytree(source,config/relative,ignore=shutil.ignore_patterns('__pycache__','.git'),dirs_exist_ok=True)
    for name,command in [('git_status',['git','status','--short']),('git_revision',['git','rev-parse','HEAD']),
                         ('git_diff',['git','diff','HEAD','--','src','tools']),('system',['uname','-a'])]:
        text,code=capture(command);(out/(name+'.txt')).write_text(text)
    raw=set(['/livox/lidar','/livox/imu','/unilidar/cloud','/lidar/points']+args.topic)
    qos={topic:dict(reliability='best_effort',durability='volatile',history='keep_last',depth=100) for topic in raw}
    for topic in ['/tf_static','/robot_description','/frontend/session','/backend_node/localization_state']:
        qos[topic]=dict(reliability='reliable',durability='transient_local',history='keep_last',depth=100)
    import yaml
    qos_path=out/'qos.yaml';qos_path.write_text(yaml.safe_dump(qos,sort_keys=True))
    command=['ros2','bag','record','--output',str(out/'bag'),'--storage',args.storage,
             '--max-bag-duration','120','--max-cache-size','268435456','--disable-keyboard-controls',
             '--qos-profile-overrides-path',str(qos_path)]
    if args.all:command+=['--all','--include-hidden-topics']
    else:command+=['--topics',*topics]
    manifest=dict(created=datetime.now().astimezone().isoformat(),workspace=str(WS),note=args.note,
                  ros_domain_id=os.environ.get('ROS_DOMAIN_ID','0'),command=command,
                  use_sim_time=False,topics=topics,all_topics=args.all,completed=False)
    (out/'session.json').write_text(json.dumps(manifest,indent=2)+'\n')
    stop=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.set())
    print(f'Session: {out}\nRecording existing topics. Ctrl-C stops only this recorder.',flush=True)
    log=(out/'recorder.log').open('w')
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    thread=threading.Thread(target=snapshot,args=(out,'start_graph'),daemon=True);thread.start()
    begin=time.monotonic()
    try:
        while process.poll() is None and not stop.wait(.2):
            if args.duration and time.monotonic()-begin>=args.duration:break
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            print('Recorder did not finalize in 30 s; sending TERM. Inspect recorder.log.',file=sys.stderr)
            os.killpg(process.pid,signal.SIGTERM)
            process.wait(timeout=10)
        log.close();thread.join(timeout=5)
    info,info_code=capture(['ros2','bag','info',str(out/'bag')],timeout=15)
    (out/'bag_info.txt').write_text(info)
    # When recording starts before bringup, start_graph can legitimately be
    # empty. Capture the running mapping graph too, before the user stops it.
    print('Saving final ROS graph and runtime parameters; keep mapping running.',flush=True)
    snapshot(out,'stop_graph')
    manifest.update(completed=info_code==0,recorder_exit_code=process.returncode,
                    elapsed_seconds=time.monotonic()-begin,finished=datetime.now().astimezone().isoformat())
    (out/'session.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(info,flush=True)
    print(f'Saved session: {out}',flush=True)
    return 0 if manifest['completed'] and process.returncode in (0,130,-signal.SIGINT) else 1

if __name__=='__main__':raise SystemExit(main())
