#!/usr/bin/env python3
"""Exercise a stationary coordinated reset in an already running simulation.

Call only in an isolated regression ROS domain, with the navigation action idle.
No motion commands are published. Records real TF messages across the reset.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', required=True, type=Path)
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('reset_tf_accept')
    state, commands, odometry, samples, errors = {}, [], [], [], []
    def status(m):
        state.update(dict(s.split('=', 1) for s in m.data.split(';') if '=' in s))
    def tf(m):
        edges = {(t.header.frame_id, t.child_frame_id): t for t in m.transforms}
        a, b = edges.get(('map', 'lio_odom')), edges.get(('lio_odom', 'base_link'))
        if a is None and b is None:
            return
        if a is None or b is None:
            errors.append('unpaired dynamic TF message')
            return
        if a.header.stamp != b.header.stamp:
            errors.append('different TF stamps')
        def matrix(t):
            q, p = t.transform.rotation, t.transform.translation
            v = np.eye(4)
            v[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            v[:3, 3] = [p.x, p.y, p.z]
            return v
        v = matrix(a) @ matrix(b)
        samples.append(dict(wall=time.time(), stamp=a.header.stamp.sec+a.header.stamp.nanosec*1e-9,
                            xy=v[:2, 3].tolist(), yaw=math.atan2(v[1, 0], v[0, 0])))
    subs = [node.create_subscription(String, '/backend_node/localization_state', status,
             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            node.create_subscription(TFMessage, '/tf', tf, 1000),
            node.create_subscription(Twist, '/cmd_vel', lambda m: commands.append(
                (time.monotonic(), math.hypot(m.linear.x, m.linear.y)+abs(m.angular.z))), 100),
            node.create_subscription(Odometry, '/localization/odometry', odometry.append, 100)]
    client = node.create_client(Trigger, '/localization_reset')
    def wait(predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return True
        return False
    result = {}
    try:
        assert client.wait_for_service(timeout_sec=10), 'reset service unavailable'
        observe_start = time.monotonic()
        assert wait(lambda: time.monotonic()-observe_start > 2 and len(samples) > 30 and
                    state.get('pose_trusted') == 'true' and len(odometry) > 30 and
                    max([v for t, v in commands if t > time.monotonic()-1] or [0]) < .02 and
                    math.dist(samples[-1]['xy'], samples[-30]['xy']) < .02 and
                    math.hypot(odometry[-1].twist.twist.linear.x,
                               odometry[-1].twist.twist.linear.y) < .02, 15), 'not stationary/trusted'
        epoch = int(state['epoch'])
        before = samples[-1]
        start = time.time()
        future = client.call_async(Trigger.Request())
        assert wait(future.done, 10), 'reset request timeout'
        assert future.result().success, future.result().message
        assert wait(lambda: int(state.get('epoch', -1)) > epoch and state.get('pose_trusted') == 'true'
                    and samples[-1]['wall'] > start+2, 25), 'reset did not recover'
        wait(lambda: False, 2)
        after = samples[-1]
        jumps = [math.dist(a['xy'], b['xy']) for a, b in zip(samples, samples[1:])]
        maximum = max(jumps, default=0)
        displacement = math.dist(before['xy'], after['xy'])
        assert maximum < .3, f'TF step {maximum:.3f} m'
        assert displacement < .3, f'stationary reset displaced pose {displacement:.3f} m'
        assert not errors, errors[:5]
        result.update(passed=True, source_epoch=epoch, target_epoch=int(state['epoch']),
                      tf_messages=len(samples), max_tf_step_m=maximum,
                      stationary_reset_displacement_m=displacement)
    except (AssertionError, RuntimeError) as e:
        result.update(passed=False, error=str(e), state=state)
    finally:
        result['samples'] = samples
        result['errors'] = errors
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k != 'samples'}, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
