import json, math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
root = Path('/home/yang/yang_nav/reports/navigation/2026-11-14/manual_third_goal_stall')
targets = [1794635627.21, 1794635646.14, 1794635650.30, 1794635652.34, 1794635662.06, 1794635730.0]
nearest = {t: (float('inf'), None) for t in targets}
edges = {}
for line in (root/'original/localization_trace.jsonl').open():
    r = json.loads(line)
    if r['kind'] != 'tf': continue
    d = r['data']; p = d['pose']
    m = np.eye(4); m[:3,:3] = Rotation.from_quat(p[3:]).as_matrix(); m[:3,3] = p[:3]
    edges[d['parent'], d['child']] = m
    if d['child'] != 'base_link' or ('map','lio_odom') not in edges: continue
    body = edges['map','lio_odom'] @ m
    for t in targets:
        dt = abs(r['wall'] - t)
        if dt < nearest[t][0]:
            nearest[t] = (dt, dict(wall=r['wall'], sim=r['sim'], xyz=body[:3,3].tolist(), yaw_deg=math.degrees(math.atan2(body[1,0],body[0,0]))))
result={str(t):dict(wall_error=dt, **sample) for t,(dt,sample) in nearest.items()}
(root/'original_event_poses.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
