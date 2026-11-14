#!/usr/bin/env python3
"""Record /nav/obstacles (+ /nav/clearing) in the map frame for offline analysis.

    record_obs.py OUT.csv [SECONDS] [map_frame]

One row per /nav/obstacles message: `t_now stamp npoints x y z ...`, positions in
`map_frame` (default `map`).

terrain_obstacles.py publishes the cloud in the *sensor* frame (header.frame_id
is the lidar), so the raw points must be transformed before they mean anything in
map coordinates - recording them untransformed produced points at
x -6..6 / z -4..4 that could not be matched to a map location at all.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def quat_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Rec(Node):
    def __init__(self, path, map_frame):
        super().__init__('record_obs')
        self.map_frame = map_frame
        self.f = open(path, 'w')
        self.f.write('# t_now stamp npoints x y z ...  (map frame)\n')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(PointCloud2, '/nav/obstacles', self.cb, qos_profile_sensor_data)
        self.dropped = 0

    def cb(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.asarray(pts).reshape(-1, 3)
        if len(pts):
            try:
                tf = self.tf.lookup_transform(self.map_frame, msg.header.frame_id, stamp).transform
            except TransformException:
                self.dropped += 1
                return
            r = tf.rotation
            pts = pts @ quat_mat(r.x, r.y, r.z, r.w).T + np.array(
                [tf.translation.x, tf.translation.y, tf.translation.z])
        row = ['%.3f %.3f %d' % (self.now(), stamp.nanoseconds * 1e-9, len(pts))]
        if len(pts):
            row.append(' '.join('%.3f %.3f %.3f' % (p[0], p[1], p[2]) for p in pts[:800]))
        self.f.write(' '.join(row) + '\n')
        self.f.flush()

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9


def main():
    path = sys.argv[1]
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    map_frame = sys.argv[3] if len(sys.argv) > 3 else 'map'
    rclpy.init()
    node = Rec(path, map_frame)
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.dropped:
        print(f'record_obs: {node.dropped} messages dropped (no TF at stamp)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
