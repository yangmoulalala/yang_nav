"""Evaluate result.json: SE2-align GT to map frame, report errors per tag group.

    zt_eval.py result.json [--align YAW_DEG TX TY]   (fixed GT->map alignment, e.g. from the mapping run)
"""
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
if not ref and '--align' not in sys.argv:
    sys.exit('not enough reference samples')
if '--align' in sys.argv:
    i = sys.argv.index('--align')
    dyaw = math.radians(float(sys.argv[i + 1]))
    t = np.array([float(sys.argv[i + 2]), float(sys.argv[i + 3])])
    R = np.array([[math.cos(dyaw), -math.sin(dyaw)], [math.sin(dyaw), math.cos(dyaw)]])
    ref = []
G = np.array([s['gt'][:2] for s in ref]); E = np.array([s['est'][:2] for s in ref])
# 参考样本几乎重合（原地旋转、标定、建图 0 圈）时 SE(2) 拟合无意义：改用首个样本的位姿锚定。
if ref and np.ptp(G, axis=0).max() < 0.5:
    s0 = ref[0]
    dyaw = s0['est'][2] - s0['gt'][2]
    R = np.array([[math.cos(dyaw), -math.sin(dyaw)], [math.sin(dyaw), math.cos(dyaw)]])
    t = np.array(s0['est'][:2]) - R @ np.array(s0['gt'][:2])
    print('reference samples do not move; anchoring on the first sample')
    ref = []
if ref:
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
false_trust = []  # 定位报可信（NORMAL/OBS_DEGRADED）但误差 > 0.5 m 的样本：错误恢复
for s in samples:
    g = np.array(s['gt'][:2]); e = np.array(s['est'][:2])
    err = np.linalg.norm(R @ g + t - e)
    if err > 0.5 and s.get('state') in ('NORMAL', 'OBS_DEGRADED'):
        false_trust.append((s['tag'], s['t'], err))
    yerr = abs(math.degrees(math.atan2(math.sin(s['gt'][2] + dyaw - s['est'][2]), math.cos(s['gt'][2] + dyaw - s['est'][2]))))
    key = s['tag'].removesuffix('_end')
    trusted = s.get('state') in (None, 'NORMAL', 'OBS_DEGRADED')
    groups.setdefault(key, []).append((err, yerr, trusted))
for k, v in groups.items():
    a = np.array(v, dtype=float)
    line = (f'{k:16s} n={len(a):4d} rmse={math.sqrt((a[:,0]**2).mean()):.3f}m max={a[:,0].max():.3f}m '
            f'last={a[-1,0]:.3f}m yaw_max={a[:,1].max():.2f}deg')
    # SUSPECT/LOST 时后端冻结输出为最后可信位姿，误差随行驶距离增长；分开统计才能看出定位精度。
    tr, un = a[a[:, 2] > 0], a[a[:, 2] == 0]
    if len(un):
        line += f' | trusted n={len(tr)}'
        if len(tr):
            line += f' rmse={math.sqrt((tr[:,0]**2).mean()):.3f}m max={tr[:,0].max():.3f}m'
        line += f'; untrusted n={len(un)} max={un[:,0].max():.3f}m'
    print(line)
# 瞬移后的前几个样本本就来不及恢复，只按定位自报可信的样本统计。
if false_trust:
    tags = sorted({tag for tag, _, _ in false_trust})
    worst = max(e for _, _, e in false_trust)
    span = false_trust[-1][1] - false_trust[0][1]
    print(f'FALSE TRUST: {len(false_trust)} samples reported NORMAL/OBS_DEGRADED with error > 0.5 m '
          f'(max {worst:.2f} m, span {span:.1f} s, tags {tags})')
else:
    print('false trust: none (no trusted sample with error > 0.5 m)')
# 裁判系统 UWB（RMUC uwb_sim）：原始观测与真值比较，确认输入质量；不参与上面的定位误差。
# 真值取接收时刻前 UWB_LATENCY 秒（uwb_sim 默认延迟）并加天线杆臂，变换到裁判场地坐标系。
REFEREE = {'rmuc': (14.0, 8.0, math.pi)}   # 原点 (world)、+X 航向，与 sentry_gazebo arena_spec.REFEREE_FRAMES 一致
UWB_ANTENNA_X, UWB_LATENCY = -0.245, 0.20
uwb = res.get('uwb') or []
if uwb:
    ts = np.array([s['t'] for s in res['samples']])
    gts = np.array([s['gt'] for s in res['samples']])
    errs = []
    if res.get('arena') in REFEREE and len(ts) > 1:
        ox, oy, ryaw = REFEREE[res['arena']]
        c, s_ = math.cos(ryaw), math.sin(ryaw)
        # 瞬移前后的真值不能插值：跳过绝架时刻附近的 UWB
        kidnap_t = [s['t'] for s in res['samples'] if s['tag'] in ('kidnap', 'symkidnap')][:1]
        for u in uwb:
            tq = u['t'] - UWB_LATENCY
            if tq < ts[0] or tq > ts[-1]:
                continue
            if kidnap_t and abs(tq - kidnap_t[0]) < 1.5:
                continue
            i = min(max(int(np.searchsorted(ts, tq)), 1), len(ts) - 1)
            if ts[i] - ts[i - 1] > 0.5:   # 样本间隔过大（绑架、停顿）时不插值
                continue
            a = (tq - ts[i - 1]) / (ts[i] - ts[i - 1])
            x, y = gts[i - 1][:2] + a * (gts[i][:2] - gts[i - 1][:2])
            yaw = gts[i][2]
            ax, ay = x + UWB_ANTENNA_X * math.cos(yaw), y + UWB_ANTENNA_X * math.sin(yaw)
            dx, dy = ax - ox, ay - oy
            ref = np.array([c * dx + s_ * dy, -s_ * dx + c * dy])
            errs.append(np.linalg.norm(np.array(u['xy']) - ref))
    span = uwb[-1]['t'] - uwb[0]['t']
    if res.get('uwb_team') is not None:
        ok = res.get('uwb_team') == res.get('team')
        print(f"uwb team: {res.get('uwb_team')} (expected {res.get('team')}){'' if ok else '  MISMATCH'}")
    line = f'uwb input: n={len(uwb)} rate={(len(uwb) - 1) / span if span > 0 else 0:.2f}Hz'
    if errs:
        e = np.array(errs)
        line += f' | vs GT: rmse={math.sqrt((e ** 2).mean()):.3f}m p95={np.percentile(e, 95):.3f}m max={e.max():.3f}m'
    print(line)
# UWB 中断事件（zt_driver.py uwbdrop / uwbdropkidnap）
for ev in res.get('uwb_events') or []:
    print(f"uwb outage: {ev['event']} ok={ev['ok']} t={ev['t']} state={ev['state']}")
