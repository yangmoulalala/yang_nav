#!/usr/bin/env python3
"""Summarize received TF edge discontinuities and local costmap origin motion.

The composed pose uses the latest received edges (a diagnostic upper bound, not
tf2's timestamp-interpolated lookup). Report edges separately to identify reset
and recovery events; this tool never alters runtime localization.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def matrix(p):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(p[3:]).as_matrix()
    t[:3, 3] = p[:3]
    return t


def analyze(directory):
    edges = {}
    previous = None
    edge_jumps = []
    composed_jumps = []
    sessions = []
    states = []
    last_state = None
    pending_batch = None
    batch_pose = None
    paired = 0
    max_composed_step = 0.0
    def compose():
        if ('map', 'lio_odom') in edges and ('lio_odom', 'base_link') in edges:
            return edges['map', 'lio_odom'] @ edges['lio_odom', 'base_link']
        return None
    def finish_batch(sim):
        nonlocal previous, max_composed_step
        body = compose()
        if body is not None and previous is not None:
            distance = float(np.linalg.norm(body[:2, 3] - previous[:2, 3]))
            max_composed_step = max(max_composed_step, distance)
            if distance > .3:
                composed_jumps.append(dict(sim=sim, distance=distance))
        previous = body
    for line in (directory / 'localization_trace.jsonl').open():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue  # a running recorder may have a partial final line
        kind, d = row['kind'], row['data']
        if kind == 'frontend_session':
            sessions.append(dict(sim=row['sim'], **d))
        if kind == '/backend_node/localization_state':
            state = d.split(';')[0]
            if state != last_state:
                states.append(dict(sim=row['sim'], state=d))
                last_state = state
        if kind != 'tf':
            continue
        edge = (d['parent'], d['child'])
        batch = d.get('batch')
        # A TFMessage carries both edges. Evaluate its completed state, rather
        # than treating the recorder's two JSON rows as separate messages.
        if batch is None or batch != pending_batch:
            if batch_pose is not None:
                finish_batch(last_tf_sim)
            pending_batch = batch
            batch_pose = None
        elif batch_pose is not None:
            paired += 1
        last_tf_sim = row['sim']
        batch_pose = edge
        transform = matrix(d['pose'])
        old = edges.get(edge)
        if old is not None:
            distance = np.linalg.norm(transform[:2, 3] - old[:2, 3])
            if distance > .3:
                edge_jumps.append(dict(sim=row['sim'], edge='/'.join(edge), distance=float(distance)))
        edges[edge] = transform
    if batch_pose is not None:
        finish_batch(last_tf_sim)
    origins = []
    for path in (directory / 'costmaps').glob('local*.npz'):
        with np.load(path) as a:
            origins.append([float(a['stamp'] if 'stamp' in a else a['t']), float(a['ox']), float(a['oy'])])
    origins.sort()
    origin_jumps = []
    for a, b in zip(origins, origins[1:]):
        distance = math.hypot(b[1] - a[1], b[2] - a[2])
        dt = b[0] - a[0]
        if distance > max(.3, 2 * dt + .1):
            origin_jumps.append(dict(time=b[0], dt=dt, distance=distance, before=a[1:], after=b[1:]))
    return dict(sessions=sessions, state_transitions=states, paired_tf_messages=paired, edge_jumps_over_30cm=edge_jumps,
                max_latest_received_composition_step_m=max_composed_step,
                latest_received_composition_jumps_over_30cm=composed_jumps,
                costmap_snapshots=len(origins), costmap_origin_jumps=origin_jumps)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = json.dumps(analyze(args.directory), indent=2)
    if args.output:
        args.output.write_text(result + '\n')
    print(result)
