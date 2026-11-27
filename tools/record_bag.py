#!/usr/bin/env python3
"""Record an existing mapping, localization, navigation or sensor-debug session.

Start the required ROS nodes separately, in the same ROS domain. Ctrl-C finalizes
the bag and saves the final graph and parameters. This tool only records data.
Use --use-sim-time for simulation; the default uses the real robot clock.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time

WS=Path(__file__).resolve().parents[1]
SENSOR_TOPICS='''/livox/lidar /livox/imu /unilidar/cloud /lidar/points
/lidar_mid360/points /lidar_l2/points /lidar_mid360/imu'''.split()
TOPICS=SENSOR_TOPICS+'''
/tf /tf_static /robot_description /joint_states /chassis/joint_states
/odometry /lidar_odometry /frontend/frame /frontend/odometry /frontend/session
/localization/odometry /localization/lio_world_cloud
/backend_node/localization_state /backend_node/localization_diagnostics /backend_node/localization_pose
/backend_node/odom_path /backend_node/optimized_path
/backend_node/debug/aligned_source /backend_node/debug/target
/cmd_vel /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_collision_checked
/chassis/yaw_target /chassis/pass_hole /chassis/nav_target
/chassis/big_yaw /chassis/big_yaw_online
/uwb_pose /diagnostics /parameter_events /rosout /localization_start
/initialpose /goal_pose /plan /plan_smoothed /local_plan /transformed_global_plan
/map /nav_map /speed_filter_mask /speed_filter_info /costmap_filter_info /speed_limit
/global_costmap/costmap /global_costmap/costmap_updates /global_costmap/published_footprint
/local_costmap/costmap /local_costmap/costmap_updates /local_costmap/published_footprint
/nav/obstacles /nav/clearing /nav/observation /nav_mode /nav_zone_state
/nav_zones/markers /nav_zones/tracker /collision_monitor_state /behavior_tree_log'''.split()
# Action feedback and status are hidden ROS topics. Goal/result services are not
# topics and require server-side service introspection to be recorded.
for action in ('navigate_to_pose', 'navigate_through_poses', 'follow_path',
               'compute_path_to_pose', 'compute_path_through_poses', 'smooth_path',
               'spin', 'backup', 'wait'):
    TOPICS += [f'/{action}/_action/feedback', f'/{action}/_action/status']
LATCHED_TOPICS='''/tf_static /robot_description /frontend/session /chassis/big_yaw_online
/backend_node/localization_state /backend_node/localization_diagnostics
/map /nav_map /speed_filter_mask /speed_filter_info /costmap_filter_info /nav_mode /nav_zone_state /nav_zones/markers'''.split()
LATCHED_TOPICS += [topic for topic in TOPICS if topic.endswith('/_action/status')]


def capture(command, timeout=8):
    try:
        p=subprocess.run(command,cwd=WS,text=True,stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT,timeout=timeout)
        return p.stdout, p.returncode
    except (OSError,subprocess.TimeoutExpired) as e:return str(e), -1


def snapshot(out, phase):
    text,code=capture([sys.executable,str(WS/'tools/record_bag_graph.py'),str(out/phase)],timeout=8)
    if text:print(text.rstrip(),flush=True)
    if code:print(f'{phase}: ROS snapshot incomplete; bag data is unaffected.',flush=True)


def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--output',type=Path,help='new session directory; default bags/recordings/<timestamp>')
    ap.add_argument('--topic',action='append',default=[],help='extra topic; repeat for remapped inputs or namespaced nodes')
    ap.add_argument('--all',action='store_true',help='record every discovered topic, including hidden topics')
    ap.add_argument('--duration',type=float,default=0,help='optional recording duration in seconds; 0 until Ctrl-C')
    ap.add_argument('--use-sim-time',action='store_true',help='use simulation clock and record /clock; waits for the first clock message')
    ap.add_argument('--storage',choices=['mcap','sqlite3'],default='mcap')
    ap.add_argument('--note',default='',help='session purpose, hardware, map, route or observed symptom')
    ap.add_argument('--dry-run',action='store_true',help='print topic list without starting ROS or creating files')
    args=ap.parse_args()
    if not math.isfinite(args.duration) or args.duration<0:ap.error('--duration must be finite and nonnegative')
    topics=sorted(set(TOPICS+args.topic+(['/clock'] if args.use_sim_time else [])))
    if args.dry_run:
        print(json.dumps(dict(topics=topics,all_topics=args.all,storage=args.storage,use_sim_time=args.use_sim_time),indent=2));return 0
    stop=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.set())
    import rosbag2_py
    if args.storage not in rosbag2_py.get_registered_writers():
        ap.error(f'{args.storage} writer unavailable; installed writers: {rosbag2_py.get_registered_writers()}')
    out=(args.output or WS/'bags/recordings'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')).expanduser().resolve()
    out.mkdir(parents=True,exist_ok=False)
    # colcon would otherwise discover the snapshotted packages inside config/
    (out/'COLCON_IGNORE').write_text('recording snapshot of src/, not a colcon source tree\n')
    config=out/'config';config.mkdir()
    sources=['src/localization/bringup/config','src/localization/bringup/launch',
             'src/platform/sentry_description/urdf','src/navigation/nav_bringup','src/navigation/nav_behaviors','src/navigation/nav_safety','src/navigation/nav_terrain']
    for relative in sources:
        source=WS/relative
        if source.exists():shutil.copytree(source,config/relative,ignore=shutil.ignore_patterns('__pycache__','.git'),dirs_exist_ok=True)
    for name,command in [('git_status',['git','status','--short']),('git_revision',['git','rev-parse','HEAD']),
                         ('git_diff',['git','diff','HEAD','--','src','tools']),('system',['uname','-a'])]:
        text,code=capture(command);(out/(name+'.txt')).write_text(text)
    raw=set(SENSOR_TOPICS)-{'/livox/lidar'}
    qos={topic:dict(reliability='best_effort',durability='volatile',history='keep_last',depth=100) for topic in raw}
    for topic in LATCHED_TOPICS:
        qos[topic]=dict(reliability='reliable',durability='transient_local',history='keep_last',depth=100)
    import yaml
    qos_path=out/'qos.yaml';qos_path.write_text(yaml.safe_dump(qos,sort_keys=True))
    command=['ros2','bag','record','--output',str(out/'bag'),'--storage',args.storage,
             '--max-bag-duration','120','--max-cache-size','268435456','--disable-keyboard-controls',
             '--qos-profile-overrides-path',str(qos_path)]
    command+=['--include-hidden-topics']
    if args.use_sim_time:command+=['--use-sim-time']
    if args.all:command+=['--all']
    else:command+=['--topics',*topics]
    manifest=dict(created=datetime.now().astimezone().isoformat(),workspace=str(WS),note=args.note,
                  ros_domain_id=os.environ.get('ROS_DOMAIN_ID','0'),command=command,
                  use_sim_time=args.use_sim_time,topics=topics,all_topics=args.all,completed=False)
    (out/'session.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'Session: {out}\nRecording existing topics. Ctrl-C stops only this recorder.',flush=True)
    with ThreadPoolExecutor(max_workers=2) as snapshots:
        with (out/'recorder.log').open('w') as log:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            snapshots.submit(snapshot,out,'start_graph')
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
        recorder_ok=process.returncode in (0,130,-signal.SIGINT)
        if recorder_ok and (out/'bag'/'metadata.yaml').is_file():
            print(f'Bag data saved: {out}',flush=True)
        print('Collecting final ROS diagnostics and checking bag metadata.',flush=True)
        snapshots.submit(snapshot,out,'stop_graph')
        info,info_code=capture(['ros2','bag','info',str(out/'bag')],timeout=15)
        (out/'bag_info.txt').write_text(info)
    manifest.update(completed=recorder_ok and info_code==0,recorder_exit_code=process.returncode,
                    elapsed_seconds=time.monotonic()-begin,finished=datetime.now().astimezone().isoformat())
    (out/'session.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(info,flush=True)
    print(f'{"Saved session" if manifest["completed"] else "Incomplete session"}: {out}',flush=True)
    return 0 if manifest['completed'] else 1

if __name__=='__main__':raise SystemExit(main())
