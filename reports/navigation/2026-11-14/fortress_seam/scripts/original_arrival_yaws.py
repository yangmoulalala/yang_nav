import json
from pathlib import Path
r=Path('/home/yang/yang_nav/reports/navigation/2026-11-14/fortress_seam/original/last_goal_map.csv')
# Goal arrival yaw is a proxy only; the action's requested quaternion was not recorded.
import math
latest={}
for s in Path('/tmp/nav_manual_gui/localization_trace_1794651945159297151.jsonl').open():
 d=json.loads(s)
 if d['kind']!='localization_odom': continue
 for wall in (1794652072.738,1794652092.759,1794652113.555):
  if d['wall']<=wall:
   p=d['data']['pose']; x,y,z,w=p[3:]
   latest[wall]=[d['sim'],p[0],p[1],math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))]
print(latest)
