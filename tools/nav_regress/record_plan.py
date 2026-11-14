#!/usr/bin/env python3
"""Record /plan and /local_plan (nav_msgs/Path) for offline analysis.

    record_plan.py OUT.txt [SECONDS]

One line per plan message:

    <wall_time> <frame_id> <n_poses> x0 y0 x1 y1 ...

Poses are subsampled to every 5th point to keep the file small; the full path is
rarely needed and the plan is re-published at the planner rate, so a run produces
hundreds of copies of almost the same path.

A sibling OUT.txt.jsonl preserves every pose quaternion and its topic, including
/plan_smoothed, so physical heading changes can be compared with the controller.

Goals are logged by goal.py, but the path is not: a run where the robot drives to
the wrong part of the field cannot be told apart from a controller bug without
seeing what the planner actually sent.
"""
import sys
import time
import json

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node

TOPICS = ('/plan', '/plan_smoothed', '/local_plan', '/received_global_plan')


class PlanRecorder(Node):
    def __init__(self, path):
        super().__init__('record_plan')
        self.f = open(path, 'w')
        self.detail = open(path + '.jsonl', 'w')
        self.f.write('# wall_time frame n_poses x0 y0 x1 y1 ...  (every 5th pose)\n')
        self.count = 0
        for t in TOPICS:
            self.create_subscription(Path, t, lambda msg, topic=t: self.cb(msg, topic), 10)

    def cb(self, msg, topic):
        self.count += 1
        pts = ' '.join('%.3f %.3f' % (p.pose.position.x, p.pose.position.y)
                       for p in msg.poses[::5])
        self.f.write('%.3f %s %d %s\n' % (
            self.get_clock().now().nanoseconds * 1e-9,
            msg.header.frame_id or '-', len(msg.poses), pts))
        self.f.flush()
        self.detail.write(json.dumps(dict(
            time=self.get_clock().now().nanoseconds * 1e-9,
            topic=topic, frame=msg.header.frame_id,
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            poses=[[p.pose.position.x, p.pose.position.y, p.pose.position.z,
                    p.pose.orientation.x, p.pose.orientation.y,
                    p.pose.orientation.z, p.pose.orientation.w] for p in msg.poses])) + '\n')
        self.detail.flush()


def main():
    out = sys.argv[1]
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3600.0
    rclpy.init()
    node = PlanRecorder(out)
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.2)
    print('record_plan: %d plans -> %s' % (node.count, out))


if __name__ == '__main__':
    main()
