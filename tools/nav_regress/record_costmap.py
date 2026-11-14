#!/usr/bin/env python3
"""Record global/local costmap snapshots for offline analysis.

    record_costmap.py OUTDIR [SECONDS] [TOPIC ...]

Writes one `<topic>_<seq>.npz` per snapshot (default every 0.5 s) plus a
`meta.txt` line per file.  Snapshotting as we go means a run that is killed
(acceptance runs are torn down with SIGINT) still leaves usable data - the
single-npz-at-exit version lost everything.

Each npz holds `data` (int8 OccupancyGrid data, row-major), `res`, `ox`,
`oy`, `w`, `h` and `t` (wall time).  Use cm_dump.py to print one.
"""
import os
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid


def main():
    out = sys.argv[1]
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0
    topics = sys.argv[3:] or ['/global_costmap/costmap']
    os.makedirs(out, exist_ok=True)
    meta = open(os.path.join(out, 'meta.txt'), 'a')

    rclpy.init()
    node = rclpy.create_node('record_costmap')
    last = {}
    seq = {}

    def make_cb(topic):
        name = topic.strip('/').replace('/', '_')

        def cb(msg):
            now = time.time()
            if now - last.get(name, 0.0) < 0.5:
                return
            last[name] = now
            i = seq.get(name, 0)
            seq[name] = i + 1
            path = os.path.join(out, f'{name}_{i:04d}.npz')
            np.savez_compressed(
                path,
                data=np.asarray(msg.data, dtype=np.int8),
                res=np.float64(msg.info.resolution),
                ox=np.float64(msg.info.origin.position.x),
                oy=np.float64(msg.info.origin.position.y),
                w=np.int32(msg.info.width),
                h=np.int32(msg.info.height),
                frame=np.str_(msg.header.frame_id),
                stamp=np.float64(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9),
                t=np.float64(now))
            meta.write(f'{os.path.basename(path)} {now:.3f} {topic}\n')
            meta.flush()

        return cb

    for t in topics:
        node.create_subscription(OccupancyGrid, t, make_cb(t), 10)

    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.2)
    meta.close()
    total = sum(seq.values())
    print(f'record_costmap: {total} snapshots -> {out}')


if __name__ == '__main__':
    main()
