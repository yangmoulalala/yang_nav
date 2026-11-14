#!/usr/bin/env python3
"""Read `gz topic -e -t /model/sentry/pose` (text form) on stdin, print `t x y z yaw_deg`.

     gz topic -e -t /model/sentry/pose | gz_truth.py > truth.log

Ground-truth trace for the acceptance checks (tools/nav_regress/tunnel_stats.py).  A tunnel body is
0.35 m long, i.e. 0.6 s at 0.6 m/s, so the trace is printed at 20 Hz: a 2 Hz trace left one sample per
tunnel and could not measure the error inside the body.  yaw_deg is nan when the message has none.
"""
import math
import re
import sys
import time

NUM = re.compile(r'(\w+):\s*([-0-9.eE+]+)')
POS = re.compile(r'position\s*\{(.*?)\}', re.S)
ORI = re.compile(r'orientation\s*\{(.*?)\}', re.S)
PERIOD = 0.05


def fields(text):
    return {k: float(v) for k, v in NUM.findall(text)}


def emit(msg):
    pos, ori = POS.search(msg), ORI.search(msg)
    if not pos:
        return
    p = fields(pos.group(1))
    yaw = float('nan')
    if ori:
        o = fields(ori.group(1))
        x, y, z, w = (o.get(k, 0.0) for k in 'xyzw')
        yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    print('%.2f %.4f %.4f %.4f %.2f' % (time.time(), p.get('x', 0.0), p.get('y', 0.0), p.get('z', 0.0), yaw),
          flush=True)


def main():
    buf, depth, last = '', 0, 0.0
    for line in sys.stdin:
        buf += line
        depth += line.count('{') - line.count('}')
        if depth > 0:
            continue
        # One pose message is a sequence of top-level blocks (header, name, position, orientation),
        # each closing back at depth 0: emit once the last of them, orientation, has been read.
        if 'orientation' not in buf:
            continue
        msg, buf = buf, ''
        now = time.time()
        if now - last >= PERIOD:
            last = now
            emit(msg)


if __name__ == '__main__':
    main()
