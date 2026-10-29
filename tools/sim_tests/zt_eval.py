"""Evaluate result.json: SE2-align GT to map frame on reference samples, report errors per tag group."""
import json
import math
import sys

import numpy as np

res = json.load(open(sys.argv[1]))
samples = [s for s in res['samples'] if s.get('est')]
print('mode', res['mode'], 'ready', res.get('ready'), 'samples', len(res['samples']), 'with_est', len(samples))
for e in res['states']:
    print('  state', e)
for s in res['services']:
    print('  service', s)
if 'kidnap_recovered' in res:
    print('kidnap_recovered', res['kidnap_recovered'])
ref = [s for s in samples if s['tag'].startswith(('start', 'lap'))]
if len(ref) < 3:
    sys.exit('not enough reference samples')
G = np.array([s['gt'][:2] for s in ref]); E = np.array([s['est'][:2] for s in ref])
mg, me = G.mean(0), E.mean(0)
H = (G - mg).T @ (E - me)
U, _, Vt = np.linalg.svd(H)
R = Vt.T @ U.T
if np.linalg.det(R) < 0:
    Vt[-1] *= -1; R = Vt.T @ U.T
t = me - R @ mg
dyaw = math.atan2(R[1, 0], R[0, 0])
print(f'alignment: yaw={math.degrees(dyaw):.2f}deg t=({t[0]:.3f},{t[1]:.3f})')
groups = {}
for s in samples:
    g = np.array(s['gt'][:2]); e = np.array(s['est'][:2])
    err = np.linalg.norm(R @ g + t - e)
    yerr = abs(math.degrees(math.atan2(math.sin(s['gt'][2] + dyaw - s['est'][2]), math.cos(s['gt'][2] + dyaw - s['est'][2]))))
    key = s['tag'].rstrip('_end')
    groups.setdefault(key, []).append((err, yerr))
for k, v in groups.items():
    a = np.array(v)
    print(f'{k:16s} n={len(a):4d} rmse={math.sqrt((a[:,0]**2).mean()):.3f}m max={a[:,0].max():.3f}m last={a[-1,0]:.3f}m yaw_max={a[:,1].max():.2f}deg')
