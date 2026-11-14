from pathlib import Path
import difflib
import hashlib
import json
import math
import shutil
import subprocess
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT = Path('/home/yang/yang_nav')
OUT = ROOT / 'reports/navigation/2026-11-14/localization_fix'
PRODUCT = ROOT / 'src/z-localization'
paths = [
    'frontend/src/input.cpp', 'frontend/CMakeLists.txt', 'frontend/package.xml',
    'frontend/test/input_timing_test.cpp', 'bringup/config/sim/frontend.yaml',
    'backend/src/localization_recovery.cpp',
    'backend/test/integration/localization_test.cpp',
]
patch = []
for name in paths:
    source = PRODUCT / name
    saved = OUT / 'after' / name
    saved.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, saved)
    before = OUT / 'before' / name
    patch.extend(difflib.unified_diff(
        before.read_text().splitlines(True) if before.exists() else [],
        source.read_text().splitlines(True),
        fromfile='before/' + name, tofile='after/' + name))
(OUT / 'localization_changes.patch').write_text(''.join(patch))

tests = {}
for package in ('frontend', 'backend'):
    rows = []
    for path in sorted((ROOT / 'build' / package / 'test_results').rglob('*.xml')):
        tree = ET.parse(path).getroot()
        cases = list(tree.iter('testcase'))
        rows.append({'file': str(path.relative_to(ROOT)), 'cases': len(cases),
                     'failures': sum(c.find('failure') is not None for c in cases),
                     'errors': sum(c.find('error') is not None for c in cases),
                     'skipped': sum(c.find('skipped') is not None for c in cases)})
    tests[package] = rows

terrain = json.loads((ROOT / 'maps/sim/terrain/terrain.json').read_text())
registration = terrain['registration']
angle = math.radians(registration['rpy_deg'][2])
rotation = np.array([[math.cos(angle), math.sin(angle)],
                     [-math.sin(angle), math.cos(angle)]])
zones = yaml.safe_load((ROOT / 'maps/sim/terrain/zones.yaml').read_text())['zones']
tunnel = next(z for z in zones if z['id'] == 'tunnel_3')
ta = math.radians(tunnel['yaw_deg'])
forward = np.array([math.cos(ta), math.sin(ta)])
lateral = np.array([-math.sin(ta), math.cos(ta)])

runs = {}
fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharex=True, sharey=True)
for ax, name in zip(axes, ['fixed_route1', 'fixed_route2']):
    run = OUT / name
    rows = [json.loads(s) for s in (run / 'localization_trace.jsonl').read_text().splitlines()]
    driver = [json.loads(s) for s in (run / 'driver_trace.jsonl').read_text().splitlines()]
    truth = np.array([[r['data']['stamp'], *r['data']['pose'][:3]] for r in rows if r['kind'] == 'truth'])
    truth = truth[np.argsort(truth[:, 0])]
    mapped = (truth[:, 1:3] - np.array(registration['t_world_map'][:2])) @ rotation.T
    along = (mapped - tunnel['centre']) @ forward
    across = (mapped - tunnel['centre']) @ lateral
    near = (np.abs(along) < tunnel['size'][0] / 2 + .8) & (np.abs(across) < tunnel['size'][1] / 2)
    body = (np.abs(along) < tunnel['size'][0] / 2) & (np.abs(across) < tunnel['size'][1] / 2)
    dx = np.diff(truth[truth[:, 0] > driver[-1]['sim'], 1:3], axis=0)
    diag = json.loads((run / 'diagnosis.json').read_text())
    during = [s for s in diag['states'] if s['sim'] >= diag['first_motion_sim']]
    runs[name] = {
        'diagnosis': diag,
        'route_end_sim': driver[-1]['sim'],
        'route_target_time': driver[-1]['reference_time'],
        'max_driver_path_error_m': max(d['position_error'] for d in driver),
        'tunnel3_body_truth_samples': int(body.sum()),
        'tunnel3_near_sim_range': truth[near, 0][[0, -1]].tolist(),
        'tunnel3_body_sim_range': truth[body, 0][[0, -1]].tolist(),
        'post_fixture_truth_path_m': float(np.linalg.norm(dx, axis=1).sum()),
        'states_during_motion': during,
    }
    ax.plot(mapped[:, 0], mapped[:, 1], color='#34669a', lw=1.5, label='Physical robot truth')
    corners = np.array([[a, b] for a, b in [(-1,-1), (1,-1), (1,1), (-1,1), (-1,-1)]])
    box = np.array(tunnel['centre']) + corners[:, :1] * tunnel['size'][0]/2 * forward + corners[:, 1:] * tunnel['size'][1]/2 * lateral
    ax.plot(box[:, 0], box[:, 1], color='#dc642d', lw=2, label='Tunnel 3 body')
    ax.scatter(mapped[body, 0], mapped[body, 1], s=8, c='#dc642d')
    ax.set_title(name)
    ax.set_xlabel('Map x (m)')
    ax.set_xlim(3.8, 8.7)
    ax.set_ylim(-6.2, -2.3)
    ax.grid(alpha=.2)
    ax.set_aspect('equal')
    ax.legend(fontsize=8)
axes[0].set_ylabel('Map y (m)')
fig.suptitle('Physical route coverage; truth used only by test fixture and evaluation')
fig.tight_layout()
fig.savefig(OUT / 'physical_route_coverage.png', dpi=160)
plt.close(fig)

verification = {
    'tests': tests,
    'test_cases': sum(r['cases'] for rs in tests.values() for r in rs),
    'test_failures': sum(r['failures'] + r['errors'] for rs in tests.values() for r in rs),
    'runs': runs,
    'before_fixed_replay': json.loads((OUT / 'baseline_route/replay_comparison.json').read_text()),
    'command_fixture_replay': json.loads((OUT / 'command_fixture1/replay_comparison.json').read_text()),
}
(OUT / 'verification.json').write_text(json.dumps(verification, indent=2) + '\n')

hashpaths = [OUT / name for name in (
    'README.md', 'localization_changes.patch', 'verification.json',
    'build_final.log', 'tests_full.log', 'backend_regression_before_test.log',
    'backend_test_final.log', 'test_results_backend.log', 'test_results_frontend.log')]
hashpaths.extend((OUT / 'after').rglob('*'))
for name in ('baseline_route', 'command_fixture1', 'fixed_nav1', 'fixed_route1', 'fixed_route2'):
    hashpaths.extend(p for p in (OUT / name).iterdir() if p.name in {
        'inputs.bin', 'input_counts.json', 'localization_trace.jsonl', 'loc.log', 'sim.log',
        'nav.log', 'nav_after.log', 'diagnosis.json', 'goal_after_exit.log', 'goal_followup.log',
        'driver_trace.jsonl', 'replay_comparison.json'})
with (OUT / 'SHA256SUMS').open('w') as stream:
    for path in sorted(p for p in hashpaths if p.is_file()):
        digest = hashlib.file_digest(path.open('rb'), 'sha256').hexdigest()
        stream.write(f'{digest}  {path.relative_to(OUT)}\n')
print(json.dumps({'test_cases': verification['test_cases'], 'test_failures': verification['test_failures'],
                  'runs': {n: {k: v for k, v in r.items() if k != 'diagnosis'} for n, r in runs.items()}}, indent=2))
