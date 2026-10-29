"""Count PointCloud2 messages without deserializing them; report wall-time and sim-time rates."""
import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

duration = float(sys.argv[1])
topics = sys.argv[2:]
rclpy.init()
node = Node('zt_cloud_rate')
counts = {t: 0 for t in topics}


def counter(topic):
    def callback(_):
        counts[topic] += 1
    return callback


for t in topics:
    node.create_subscription(PointCloud2, t, counter(t), 10, raw=True)
clock = [None, None]


def on_clock(msg):
    s = msg.clock.sec + msg.clock.nanosec * 1e-9
    if clock[0] is None:
        clock[0] = s
    clock[1] = s


node.create_subscription(Clock, '/clock', on_clock, 10)
start = time.monotonic()
while time.monotonic() - start < duration:
    rclpy.spin_once(node, timeout_sec=0.05)
wall = time.monotonic() - start
sim = (clock[1] - clock[0]) if clock[0] is not None else float('nan')
print(f'wall={wall:.1f}s sim={sim:.1f}s rtf={sim / wall:.2f}')
for t, c in counts.items():
    print(f'{t}: {c} msgs, {c / wall:.2f} Hz wall, {c / sim:.2f} Hz sim')
rclpy.shutdown()
