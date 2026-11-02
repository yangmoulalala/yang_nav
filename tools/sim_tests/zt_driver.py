"""Drive the simulated sentry with ground-truth feedback, record GT vs map->base_link, optional kidnap.

    zt_driver.py mapping  OUT.json [laps]
    zt_driver.py localize OUT.json [laps]   (drives laps, then kidnaps and watches recovery)
    zt_driver.py spin     OUT.json 0        (spins in place at the spawn point, slow then fast)
    zt_driver.py calib    OUT.json 0        (parks at structured spots with varied headings for dual-LiDAR calibration)
    optional 4th argument (zt_run.sh 6th): comma-separated flags
      spin       turn one full circle at every waypoint
      initspin   after ZLIO initializes, spin slowly until the backend is localized
      earlyspin  hold 2 s, then spin slowly during ZLIO initialization until the backend is localized
      spawnspin  one slow full turn at the spawn point before the laps (mapping with a side-mounted LiDAR)
      kidnapspin after the kidnap, turn slowly while the backend is SUSPECT/LOST (gives recovery new views)
"""
import json
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
# 标定采集点：路线上的拐角与中央，每处四个航向，静止采集；仿真中只有部分航向的几何能通过 support 门。
CALIB_SPOTS = [(x, y, h) for x, y in ((1.25, 1.5), (6.5, 1.5), (6.5, 2.5), (1.25, 2.5), (1.25, 6.75))
               for h in (0.0, 1.6, 3.1, -1.6)]


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
        self.odom_seen = False
        self.init_timing = {}
        self.create_subscription(Odometry, '/odometry', self.on_odom, 10)
        self.gt = None
        self.lock = threading.Lock()
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f'/world/{WORLD}/pose/info', self.on_pose)
        self.samples = []
        self.t0 = time.monotonic()
        self.heading = 0.0

    def on_state(self, msg):
        fields = dict(kv.split('=', 1) for kv in msg.data.split(';') if '=' in kv)
        status = fields.get('status')
        if status != self.state:
            self.state = status
            entry = {'wall': round(time.monotonic() - self.t0, 2), 'status': status,
                     'reason': fields.get('reason', '')}
            self.states.append(entry)
            print('STATE', entry, flush=True)

    def on_odom(self, _msg):
        self.odom_seen = True

    def spin_until_ready(self, wanted, rate=0.3, timeout=150, early=False, still_seconds=2.0):
        """等 ZLIO 初始化（收到 /odometry），再慢转直到后端进入 wanted，最后转回航向 0。

        early=True 时前端初始化期间就慢转：侧装雷达静止时只看一侧，初始化窗口缺少某个方向的平面支撑。
        """
        end = time.monotonic() + timeout
        if early:
            # 先静止 still_seconds 再转：首次尝试的重力初猜取自静止 IMU；静止期间就收到 /odometry 说明前端已静止初始化。
            still_end = time.monotonic() + still_seconds
            while not self.odom_seen and time.monotonic() < still_end:
                rclpy.spin_once(self, timeout_sec=0.02)
                self.cmd.publish(Twist())
            self.init_timing['initialized_while_still'] = self.odom_seen
        self.init_timing['spin_start'] = round(time.monotonic() - self.t0, 2)
        self.init_timing['spin_start_wall'] = time.time()
        while not self.odom_seen and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02 if early else 0.05)
            if early:
                t = Twist()
                t.angular.z = rate
                self.cmd.publish(t)
        print('ODOM', self.odom_seen, round(time.monotonic() - self.t0, 2), flush=True)
        self.init_timing['odom_seen'] = self.odom_seen
        self.init_timing['odom'] = round(time.monotonic() - self.t0, 2)
        ready = self.state in wanted
        while not ready and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            t = Twist()
            t.angular.z = rate
            self.cmd.publish(t)
            self.record('init_spin')
            ready = self.state in wanted
        self.stop()
        with self.lock:
            yaw = self.gt[3] if self.gt else None
        print('INIT_SPIN ready', ready, 'yaw', yaw, round(time.monotonic() - self.t0, 2), flush=True)
        if ready:
            self.turn_to(0.0, 'init_spin')
        return ready

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
                t.angular.z = max(-1.0, min(1.0, -2.0 * wrap(yaw - self.heading)))
                self.cmd.publish(t)
        self.stop()

    def spin(self, turns, tag, rate=0.8, timeout=120):
        with self.lock:
            last = self.gt[3] if self.gt else None
        total = 0.0
        end = time.monotonic() + timeout
        while abs(total) < turns * 2 * math.pi and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            with self.lock:
                gt = self.gt
            if gt is None:
                continue
            if last is not None:
                total += wrap(gt[3] - last)
            last = gt[3]
            t = Twist()
            t.angular.z = rate
            self.cmd.publish(t)
            self.record(tag)
        self.stop()

    def turn_to(self, heading, tag, timeout=30):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            with self.lock:
                gt = self.gt
            if gt is None:
                continue
            err = wrap(heading - gt[3])
            if abs(err) < 0.03:
                break
            t = Twist()
            t.angular.z = max(-0.8, min(0.8, 2.0 * err))
            self.cmd.publish(t)
            self.record(tag)
        self.heading = heading
        self.stop()

    def hold(self, seconds, tag):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            self.cmd.publish(Twist())
            self.record(tag)

    def rotate(self, seconds, tag, rate=0.3):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            t = Twist()
            t.angular.z = rate
            self.cmd.publish(t)
            self.record(tag)
        self.stop()

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
    extras = set(sys.argv[4].split(',')) if len(sys.argv) > 4 else set()
    if mode in ('spin', 'calib'):
        laps = 0
    rclpy.init()
    d = Driver()
    result = {'mode': mode, 'laps': laps, 'services': []}
    if extras & {'initspin', 'earlyspin'}:  # 启动时慢转：雷达侧装只看一侧，转动让源点云覆盖更多方向
        ready = d.spin_until_ready({'NORMAL', 'OBS_DEGRADED'}, early='earlyspin' in extras)
    else:
        ready = d.wait_state({'NORMAL', 'OBS_DEGRADED'}, 120)
    result['ready'] = ready
    print('READY', ready, d.state, flush=True)
    if ready:
        d.hold(3, 'start')
        if 'spawnspin' in extras:  # 只在出生点慢转一圈：侧装雷达各航向视野不同，补齐出生点各航向的关键帧
            d.spin(1, 'start_spin', rate=0.3, timeout=60)
            d.turn_to(0.0, 'start_spin')
            d.hold(2, 'start_spin_end')
        for lap in range(laps):
            if 'spin' in extras:
                for wp in LAP:
                    d.drive([wp], f'lap{lap}')
                    d.spin(1, f'lap{lap}')
            else:
                d.drive(LAP, f'lap{lap}')
            d.hold(2, f'lap{lap}_end')
        if mode == 'mapping':
            result['services'].append(d.call('/backend_node/save_map'))
        elif mode == 'calib':
            for x, y, heading in CALIB_SPOTS:
                d.drive([(x, y)], 'calib_move')
                d.turn_to(heading, 'calib_move')
                d.hold(6, 'calib_hold')
        elif mode == 'spin':
            d.spin(2, 'spin', 0.6)
            d.hold(3, 'spin_end')
            d.spin(2, 'spinfast', -1.5)
            d.hold(3, 'spinfast_end')
        else:
            d.teleport(*KIDNAP)
            d.hold(1, 'kidnap')
            recovered = False
            end = time.monotonic() + 90
            asked = False
            while time.monotonic() < end:
                if 'kidnapspin' in extras and d.state in ('SUSPECT', 'LOST'):
                    d.rotate(0.5, 'kidnap')
                else:
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
    result['init_timing'] = d.init_timing
    result['samples'] = d.samples
    with open(out, 'w') as f:
        json.dump(result, f)
    print('WROTE', out, len(d.samples), flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
