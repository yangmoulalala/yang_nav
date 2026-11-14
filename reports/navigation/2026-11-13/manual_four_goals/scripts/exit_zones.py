from pathlib import Path
import yaml,math,json
root=Path('/home/yang/yang_nav');zones=yaml.safe_load((root/'maps/sim/terrain/zones.yaml').read_text())['zones']
checks=[]
for label,x,y in [('goal3 estimated',4.52,-5.24),('goal4 estimated',4.59,-5.27),('registered truth',4.836,-5.4)]:
    hits=[]
    for z in zones:
        if z.get('cells',0)<50:continue
        a=math.radians(z['yaw_deg']);dx=x-z['centre'][0];dy=y-z['centre'][1];u=math.cos(a)*dx+math.sin(a)*dy;v=-math.sin(a)*dx+math.cos(a)*dy
        if abs(u)<=z['size'][0]/2+.55 and abs(v)<=z['size'][1]/2+.55:hits.append((z['id'],round(u,3),round(v,3)))
    checks.append({'label':label,'x':x,'y':y,'expanded_zone_hits':hits})
    print(label,hits)
# Exact distances from reported centre to static lethal-cell centres.
import numpy as np
from PIL import Image
cfg=yaml.safe_load((root/'maps/sim/terrain/nav_map.yaml').read_text());g=np.flipud(np.asarray(Image.open(root/'maps/sim/terrain'/cfg['image'])));ij=np.argwhere((1-g/255.)>cfg['occupied_thresh']);r=cfg['resolution'];xy=ij[:,::-1]*r+np.array(cfg['origin'][:2])+r/2
for x,y in [(4.52,-5.24),(4.59,-5.27),(4.836,-5.4)]:print('exact nearest',x,y,float(np.linalg.norm(xy-[x,y],axis=1).min()))
(root/'reports/navigation/2026-11-13/manual_four_goals/exit_zone_checks.json').write_text(json.dumps({'zone_margin_m':.55,'poses':checks},indent=2))
