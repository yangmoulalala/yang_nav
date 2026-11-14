#!/usr/bin/env python3
"""Real Nav2 dynamic-obstacle acceptance against the final-command omni fixture.

This is a software closed-loop test. It does not model wheel slip or real sensors.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from framework_accept import Plant, fixture


def polygon_clearance(a, b):
    """Exact separation, or negative SAT penetration, for convex polygons."""
    def dot(p, q):
        return p[0]*q[0] + p[1]*q[1]
    gaps = []
    for polygon in (a, b):
        for p, q in zip(polygon, polygon[1:]+polygon[:1]):
            axis = (p[1]-q[1], q[0]-p[0])
            norm = math.hypot(*axis)
            axis = (axis[0]/norm, axis[1]/norm)
            aa, bb = [dot(v, axis) for v in a], [dot(v, axis) for v in b]
            gaps.append(max(min(aa)-max(bb), min(bb)-max(aa)))
    if max(gaps) <= 0:
        return max(gaps)
    def point_segment(p, u, v):
        d = (v[0]-u[0], v[1]-u[1])
        t = max(0., min(1., dot((p[0]-u[0], p[1]-u[1]), d)/dot(d, d)))
        return math.hypot(p[0]-u[0]-t*d[0], p[1]-u[1]-t*d[1])
    return min(point_segment(p, u, v) for points, edges in ((a, b), (b, a))
               for p in points for u, v in zip(edges, edges[1:]+edges[:1]))


def clearance(samples, obstacle):
    minimum = float('inf')
    bx, by = obstacle
    square = [(bx+u, by+v) for u, v in [(-.05,-.05),(.05,-.05),(.05,.05),(-.05,.05)]]
    for s in samples:
        x, y, yaw = s['pose']; c, t = math.cos(yaw), math.sin(yaw)
        footprint = [(x+c*u-t*v, y+t*u+c*v) for u, v in
                     [(-.31,-.31),(.39,-.31),(.39,.31),(-.31,.31)]]
        minimum = min(minimum, polygon_clearance(footprint, square))
    assert minimum >= -1e-6, f'padded footprint collision: clearance={minimum}'
    return minimum


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '87')
    out = Path(args.out).resolve();out.mkdir(parents=True, exist_ok=True)
    terrain = fixture(out)
    rclpy.init();plant = Plant(out)
    checks=[];metrics={};process=None
    def passed(name):
        checks.append(name);print('PASS '+name,flush=True)
    try:
        plant.pose=np.array([-5.,-3.,0.])
        with (out/'navigation.log').open('w') as log:
            process=subprocess.Popen(['ros2','launch','nav_bringup','navigation.launch.py',
                'terrain_dir:='+str(terrain),'use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            plant.wait(lambda:'Managed nodes are active' in (out/'navigation.log').read_text(),50.)
            target=(-2.,-3.,0.)
            handle,future=plant.start(target)
            moving=[]
            begin=time.monotonic()
            for k in range(55):
                plant.blocker=(-3.5,-2.2-1.6*k/54)
                j=len(plant.samples);plant.run_for(.06)
                moving.append(clearance(plant.samples[j:],plant.blocker))
            plant.blocker=None
            plant.finish(future,target)
            metrics['crossing_min_clearance_m']=min(moving)
            metrics['crossing_total_seconds']=time.monotonic()-begin
            passed('moving cross traffic is avoided without footprint contact')
            plant.goal((-5.,-2.,0.))
            plant.blocker=(-3.5,-2.);j=len(plant.samples)
            handle,future=plant.start((-2.,-2.,0.));plant.run_for(5.)
            metrics['fixed_min_clearance_m']=clearance(plant.samples[j:],plant.blocker)
            plant.blocker=None;plant.finish(future,(-2.,-2.,0.))
            passed('path blocker permits safe reroute and continues after it moves away')
            plant.goal((-5.,-3.,0.))
            target=(-2.,-3.,0.);plant.blocker=target[:2];plant.run_for(1.)
            j=len(plant.samples);before=plant.pose.copy()
            handle,future=plant.start(target);plant.run_for(10.)
            assert not future.done(),'occupied original goal incorrectly completed or aborted'
            assert np.linalg.norm(plant.pose[:2]-before[:2])>.5,'occupied target was rejected without approaching'
            metrics['occupied_min_clearance_m']=clearance(plant.samples[j:],plant.blocker)
            metrics['occupied_wait_distance_m']=float(np.linalg.norm(plant.pose[:2]-np.array(target[:2])))
            assert metrics['occupied_wait_distance_m']>.35
            plant.observations=False;plant.run_for(1.2)
            assert plant.stopped(),'observation outage failed to stop approach'
            plant.observations=True;plant.run_for(.8)
            plant.blocker=None;released=time.monotonic();plant.finish(future,target)
            metrics['goal_release_to_success_seconds']=time.monotonic()-released
            passed('occupied goal is approached and remains pending, sensor outage stops it, removal resumes original goal')
            plant.goal((-5.,-2.,0.));plant.blocker=(-2.,-2.);plant.run_for(1.)
            handle,future=plant.start((-2.,-2.,0.));plant.run_for(12.)
            assert not future.done(),'persistent occupied target must remain pending'
            plant.cancel(handle,future)
            passed('persistent occupied goal remains pending and cancellation stops output')
            old,old_future=plant.start((-2.,-2.,0.));plant.run_for(2.)
            new,new_future=plant.start((-4.5,-3.5,0.));plant.finish(new_future,(-4.5,-3.5,0.))
            plant.wait(old_future.done,5.)
            assert old_future.result().status in (GoalStatus.STATUS_ABORTED,GoalStatus.STATUS_CANCELED)
            plant.blocker=None
            passed('fresh target preempts an occupied pending target')
            handle,future=plant.start((0.,-2.,0.));plant.wait(future.done,40.)
            assert future.result().status==GoalStatus.STATUS_ABORTED
            passed('static wall target remains an error rather than an unsafe approach target')
        result=dict(passed=True,checks=checks,metrics=metrics,model='real Nav2; deterministic final-command omni fixture')
        code=0
    except Exception as exc:
        result=dict(passed=False,checks=checks,metrics=metrics,error=str(exc));code=1
        print('FAIL '+str(exc),flush=True)
    finally:
        if process:
            for sig,timeout in ((signal.SIGINT,8),(signal.SIGTERM,5),(signal.SIGKILL,3)):
                try:os.killpg(process.pid,sig);process.wait(timeout=timeout);break
                except ProcessLookupError:break
                except subprocess.TimeoutExpired:continue
        plant.trace.close();plant.destroy_node();rclpy.shutdown()
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    return code

if __name__=='__main__':
    raise SystemExit(main())
