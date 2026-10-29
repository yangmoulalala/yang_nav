"""Drive the simulated sentry with ground-truth feedback, record GT vs map->base_link, optional kidnap.

    zt_driver.py mapping  OUT.json [laps]
    zt_driver.py localize OUT.json [laps]   (drives laps, then kidnaps and watches recovery)
"""
import json
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

WORLD = 'rmul_2027'
LAP = [(1.25, 1.5), (6.5, 1.5), (6.5, 2.5), (1.25, 2.5), (1.25, 6.75)]
KIDNAP = (6.0, 1.5, math.pi / 2)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Driver(Node):
    def __init__(self):
        super().__init__('zt_driver', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf = Buffer(cache_time=Duration(seconds=900))
        self.listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/backend_node/localization_state', self.on_state, qos)
        self.state = None
        self.states = []
        self.gt = None
        self.lock = threading.Lock()
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f'/world/{WORLD}/pose/info', self.on_pose)
        self.samples = []
        self.t0 = time.monotonic()

    def on_state(self, msg):
        fields = dict(kv.split('=', 1) for kv in msg.data.split(';') if '=' in kv)
        status = fields.get('status')
        if status != self.state:
            self.state = status
            entry = {'wall': round(time.monotonic() - self.t0, 2), 'status': status,
                     'reason': fields.get('reason', '')}
            self.states.append(entry)
            print('STATE', entry, flush=True)

    def on_pose(self, msg):
        for p in msg.pose:
            if p.name == 'sentry':
                stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                with self.lock:
                    self.gt = (stamp, p.position.x, p.position.y, yaw_of(p.orientation))
                return

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_state(self, wanted, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.state in wanted:
                return True
        return False

    def record(self, tag):
        with self.lock:
            gt = self.gt
        if gt and (not self.samples or gt[0] - self.samples[-1]['t'] >= 0.2):
            self.samples.append({'t': gt[0], 'gt': gt[1:], 'tag': tag, 'state': self.state})

    def stop(self):
        self.cmd.publish(Twist())

    def drive(self, waypoints, tag, vmax=0.8, timeout=120):
        end = time.monotonic() + timeout
        for wx, wy in waypoints:
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
                with self.lock:
                    gt = self.gt
                if gt is None:
                    continue
                _, x, y, yaw = gt
                ex, ey = wx - x, wy - y
                dist = math.hypot(ex, ey)
                self.record(tag)
                if dist < 0.08:
                    break
                speed = min(vmax, 1.5 * dist)
                vx_w, vy_w = ex / dist * speed, ey / dist * speed
                c, s = math.cos(yaw), math.sin(yaw)
                t = Twist()
                t.linear.x = c * vx_w + s * vy_w
                t.linear.y = -s * vx_w + c * vy_w
                t.angular.z = max(-1.0, min(1.0, -2.0 * wrap(yaw)))
                self.cmd.publish(t)
        self.stop()

    def hold(self, seconds, tag):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            self.cmd.publish(Twist())
            self.record(tag)

    def teleport(self, x, y, yaw):
        req = Pose()
        req.name = 'sentry'
        req.position.x, req.position.y, req.position.z = x, y, 0.062
        req.orientation.z, req.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        ok, rep = self.gz.request(f'/world/{WORLD}/set_pose', req, Pose, Boolean, 3000)
        print('TELEPORT', ok, rep.data if ok else None, flush=True)

    def call(self, name):
        client = self.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=5.0):
            return {'service': name, 'success': False, 'message': 'unavailable'}
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 60
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        res = future.result()
        out = {'service': name, 'success': bool(res and res.success),
               'message': res.message if res else 'timeout'}
        print('SERVICE', out, flush=True)
        return out

    def attach_estimates(self):
        for s in self.samples:
            try:
                tr = self.tf.lookup_transform('map', 'base_link', Time(seconds=s['t']))
                s['est'] = (tr.transform.translation.x, tr.transform.translation.y,
                            yaw_of(tr.transform.rotation))
            except Exception:
                s['est'] = None


def main():
    mode, out = sys.argv[1], sys.argv[2]
    laps = int(sys.argv[3]) if len(sys.argv) > 3 else 2
    rclpy.init()
    d = Driver()
    result = {'mode': mode, 'laps': laps, 'services': []}
    ready = d.wait_state({'NORMAL', 'OBS_DEGRADED'}, 120)
    result['ready'] = ready
    print('READY', ready, d.state, flush=True)
    if ready:
        d.hold(3, 'start')
        for lap in range(laps):
            d.drive(LAP, f'lap{lap}')
            d.hold(2, f'lap{lap}_end')
        if mode == 'mapping':
            result['services'].append(d.call('/backend_node/save_map'))
        else:
            d.teleport(*KIDNAP)
            d.hold(1, 'kidnap')
            recovered = False
            end = time.monotonic() + 90
            asked = False
            while time.monotonic() < end:
                d.hold(0.5, 'kidnap')
                if d.state == 'LOST' and not asked and time.monotonic() > end - 60:
                    result['services'].append(d.call('/backend_node/relocalize'))
                    asked = True
                if any(e['status'] in ('SUSPECT', 'LOST') for e in d.states) and \
                        d.state in ('NORMAL', 'OBS_DEGRADED'):
                    recovered = True
                    break
            result['kidnap_recovered'] = recovered
            d.hold(3, 'after_recovery')
            if recovered:
                d.drive([(6.5, 1.5), (6.5, 2.5), (1.25, 2.5)], 'after_drive')
                d.hold(2, 'after_drive_end')
    d.stop()
    d.spin_for(1.0)
    d.attach_estimates()
    result['states'] = d.states
    result['samples'] = d.samples
    with open(out, 'w') as f:
        json.dump(result, f)
    print('WROTE', out, len(d.samples), flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
