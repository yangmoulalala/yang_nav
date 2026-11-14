#!/usr/bin/env python3
"""Summarize the latest manual last-goal trace and the two controlled reruns."""
import collections
import json
import math
import re
from pathlib import Path

import numpy as np

WS = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent


def summarize(run, trace):
    log = (run / 'nav.log').read_text()
    starts = re.findall(r'\[(\d+\.\d+)\].*Begin navigating from.*to \(16\.90, 0\.34\)', log)
    start = float(starts[-1])
    states = collections.Counter()
    loc, truth, last_cmd = [], [], None
    for line in trace.open():
        row = json.loads(line)
        if row['wall'] < start:
            continue
        kind, data = row['kind'], row['data']
        if kind == '/backend_node/localization_state':
            values = dict(x.split('=', 1) for x in data.split(';') if '=' in x)
            states[(values.get('status'), values.get('pose_trusted'), values.get('epoch'))] += 1
        elif kind == 'localization_odom':
            loc.append([row['sim'], row['wall'], *data['pose']])
        elif kind == 'truth':
            truth.append([row['sim'], row['wall'], *data['pose']])
        elif kind == '/cmd_vel_nav':
            last_cmd = row
    result = dict(run=str(run), trace=str(trace), last_goal_wall=start,
                  states=[dict(status=k[0], trusted=k[1], epoch=k[2], count=v)
                          for k, v in states.items()],
                  last_localization_pose=loc[-1] if loc else None,
                  last_truth_pose=truth[-1] if truth else None,
                  last_command=last_cmd)
    if loc:
        result['last_goal_distance_m'] = math.hypot(loc[-1][2] - 16.90, loc[-1][3] - .34)
    if loc and truth:
        # Match simulation acquisition time; keep the planar convention used by goal.py.
        reg = json.loads((WS / 'maps/sim/terrain/terrain.json').read_text())['registration']
        tx, ty, _ = reg['t_world_map']
        angle = math.radians(reg['rpy_deg'][2])
        c, s = math.cos(angle), math.sin(angle)
        a = np.array(truth)
        a = a[np.argsort(a[:, 0], kind='stable')]
        errors, yaw_errors, gaps = [], [], []
        yaw = lambda q: math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
                                   1 - 2 * (q[1] ** 2 + q[2] ** 2))
        for q in loc:
            j = int(np.searchsorted(a[:, 0], q[0]))
            j = min((k for k in (j - 1, j) if 0 <= k < len(a)), key=lambda k: abs(a[k, 0] - q[0]))
            t = a[j]
            gap = abs(t[0] - q[0])
            if gap > .06:
                continue
            x, y = c * (t[2] - tx) + s * (t[3] - ty), -s * (t[2] - tx) + c * (t[3] - ty)
            dyaw = yaw(q[5:9]) - (yaw(t[5:9]) - angle)
            errors.append(math.hypot(q[2] - x, q[3] - y))
            yaw_errors.append(abs(math.degrees(math.atan2(math.sin(dyaw), math.cos(dyaw)))))
            gaps.append(gap)
        if errors:
            result['planar_truth_comparison'] = dict(samples=len(errors), position_p95_m=float(np.percentile(errors, 95)),
                position_max_m=max(errors), yaw_p95_deg=float(np.percentile(yaw_errors, 95)),
                yaw_max_deg=max(yaw_errors), acquisition_gap_max_s=max(gaps),
                convention='inverse registration yaw for XY; inverse yaw for heading; not a 3D calibration')
    return result


if __name__ == '__main__':
    runs = {
        'manual': (Path('/tmp/nav_manual_gui'), 'localization_trace_1794654051212432777.jsonl'),
        'replay_with_residual': (Path('/tmp/nav_manual_lastgoal_replay'), 'localization_trace.jsonl'),
        'clean_default': (Path('/tmp/nav_lastgoal_clean'), 'localization_trace.jsonl'),
        'clean_cost2_rejected': (Path('/tmp/nav_lastgoal_cost2'), 'localization_trace.jsonl'),
    }
    results = {name: summarize(run, run / trace) for name, (run, trace) in runs.items()}
    (OUT / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
    for name, r in results.items():
        print(name, 'last distance', round(r.get('last_goal_distance_m', -1), 3), r.get('planar_truth_comparison'), r['states'])
