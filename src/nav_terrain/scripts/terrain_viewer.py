#!/usr/bin/env python3
"""terrain_viewer: browser view of a terrain package + hand annotation of tunnels (zones_manual.yaml).

    # static page (open the file in a browser, export with Download / Copy)
    terrain_viewer.py maps/sim/terrain --ref maps/sim/nav --out maps/sim/terrain/viewer.html
    # local server: page can save zones_manual.yaml and rebuild the terrain package
    terrain_viewer.py maps/sim/terrain --ref maps/sim/nav --serve --port 8765   -> http://localhost:8765

Layers: cost (nav_map), ground z, clearance, slope, prior-map coverage, and with --ref the reference
(mesh-built) cost / ground / clearance resampled onto the same grid. Overlays: auto-detected tunnels
and ramps (zones.yaml), reference low-clearance zones, and the editable manual tunnels.
Manual tunnels come from --manual (default <map>/zones_manual.yaml) or, if that does not exist, --draft.
"""
import argparse
import base64
import json
import shutil
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
import yaml


def read_pgm(path, nx, ny):
    raw = Path(path).read_bytes()
    hdr = raw.split(b'\n', 3)
    w, h = map(int, hdr[1].split())
    img = np.frombuffer(raw[-w * h:], np.uint8).reshape(h, w)
    return np.ascontiguousarray(img[::-1, :].T[:nx, :ny])


def load_pkg(d):
    d = Path(d)
    m = json.loads((d / 'terrain.json').read_text())
    nx, ny = m['size']

    def f32(n):
        p = d / n
        return np.fromfile(p, '<f4').reshape(nx, ny).astype(float) if p.is_file() else None
    P = dict(meta=m, nx=nx, ny=ny, res=m['resolution'], ox=m['origin'][0], oy=m['origin'][1],
             cost=read_pgm(d / 'nav_map.pgm', nx, ny), ground=f32('ground_z.f32'), clear=f32('clearance.f32'),
             slope=f32('slope.f32'))
    if P['slope'] is None and P['ground'] is not None:
        gx, gy = np.gradient(P['ground'], P['res'])
        P['slope'] = np.degrees(np.arctan(np.hypot(gx, gy)))
    ob = d / 'observed.u8'
    P['observed'] = np.fromfile(ob, np.uint8).reshape(nx, ny) if ob.is_file() else None
    z = d / 'zones.yaml'
    P['zones'] = (yaml.safe_load(z.read_text()) if z.is_file() else None) or {}
    return P


def q8(a, lo, hi):
    """NaN -> 0, else 1..255 over [lo, hi]."""
    out = np.zeros(a.shape, np.uint8)
    ok = np.isfinite(a)
    span = max(hi - lo, 1e-6)
    out[ok] = (1 + np.clip((a[ok] - lo) / span, 0, 1) * 254).round().astype(np.uint8)
    return out


def b64(a):
    return base64.b64encode(np.ascontiguousarray(a, np.uint8).tobytes()).decode()


def resample(R, arr, P, fill):
    I, J = np.indices((P['nx'], P['ny']))
    X, Y = P['ox'] + (I + 0.5) * P['res'], P['oy'] + (J + 0.5) * P['res']
    ix = np.floor((X - R['ox']) / R['res']).astype(int)
    iy = np.floor((Y - R['oy']) / R['res']).astype(int)
    ok = (ix >= 0) & (iy >= 0) & (ix < R['nx']) & (iy < R['ny'])
    out = np.full(X.shape, fill, dtype=arr.dtype)
    out[ok] = arr[ix[ok], iy[ok]]
    return out


def norm_tunnel(t, i):
    def opt(k):
        return None if t.get(k) is None else float(t[k])
    return dict(id=str(t.get('id', 'tunnel_%d' % i)), centre=[float(v) for v in t['centre']],
                yaw_deg=float(t.get('yaw_deg', 0.0)), size=[float(v) for v in t.get('size', [0.9, 0.8])],
                floor_z=opt('floor_z'), roof_z=opt('roof_z'))


def load_manual(path):
    if not path or not Path(path).is_file():
        return None
    data = yaml.safe_load(Path(path).read_text()) or {}
    return [norm_tunnel(t, i) for i, t in enumerate(data.get('tunnels') or [])]


def build_data(a):
    P = load_pkg(a.terrain)
    layers = {}

    def add(key, label, arr, kind, lo=0.0, hi=1.0):
        layers[key] = dict(label=label, kind=kind, lo=float(lo), hi=float(hi), b64=b64(arr))
    add('cost', '代价 nav_map（黑=致命）', P['cost'], 'cost')
    zlo = zhi = 0.0
    if P['ground'] is not None and np.isfinite(P['ground']).any():
        zlo, zhi = np.nanpercentile(P['ground'], [1, 99])
        add('ground', '地面高度 z', q8(P['ground'], zlo, zhi), 'scalar', zlo, zhi)
    if P['clear'] is not None:
        add('clearance', '净空 0.2-0.6 m（红=低）', q8(np.minimum(P['clear'], 0.6), 0.2, 0.6), 'clear', 0.2, 0.6)
    if P['slope'] is not None:
        add('slope', '坡度 0-30°', q8(P['slope'], 0, 30), 'scalar', 0, 30)
    if P['observed'] is not None:
        add('observed', '先验点云覆盖', np.where(P['observed'] > 0, 255, 0), 'mask')
    ref_zones, reg = [], P['meta'].get('registration')
    if a.ref:
        R = load_pkg(a.ref)
        reg = reg or R['meta'].get('registration')
        add('ref_cost', '参考图 代价', resample(R, R['cost'], P, np.uint8(0)), 'cost')
        if R['ground'] is not None:
            add('ref_ground', '参考图 地面高度', q8(resample(R, R['ground'], P, np.nan), zlo, zhi), 'scalar', zlo, zhi)
        if R['clear'] is not None:
            add('ref_clearance', '参考图 净空', q8(np.minimum(resample(R, R['clear'], P, np.nan), 0.6), 0.2, 0.6),
                'clear', 0.2, 0.6)
        for z in R['meta'].get('tight_zones', []):
            if isinstance(z.get('x'), list) and z.get('min_clearance', 1) < 0.30 and z.get('cells', 0) >= 300:
                ref_zones.append(dict(x=sum(z['x']) / 2, y=sum(z['y']) / 2, clear=z['min_clearance']))
    manual, src = load_manual(a.manual), a.manual
    if manual is None:
        manual, src = (load_manual(a.draft) or []), (a.draft or '(无)') + '（草稿）'
    zones = P['zones']
    tunnels = [{k: v for k, v in t.items() if k != 'cells'} for t in zones.get('tunnels', [])]
    return dict(meta=dict(nx=P['nx'], ny=P['ny'], res=P['res'], ox=P['ox'], oy=P['oy']), layers=layers,
                zones=dict(tunnels=tunnels, ramps=zones.get('ramps', [])), ref_zones=ref_zones,
                registration=reg, manual=manual, manual_source=src, manual_path=str(a.manual),
                key=str(Path(a.terrain).resolve()))


def render(a):
    return TEMPLATE.replace('__DATA__', json.dumps(build_data(a)))


def check_yaml(text):
    data = yaml.safe_load(text) or {}
    tl = data.get('tunnels') or []
    for t in tl:
        for k in ('centre', 'size', 'floor_z', 'roof_z'):
            if t.get(k) is None:
                raise ValueError('%s: %s 为空' % (t.get('id', '?'), k))
    return len(tl)


def serve(a):
    script_dir = Path(__file__).resolve().parent

    class H(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype='text/plain; charset=utf-8'):
            b = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path.split('?')[0] in ('/', '/index.html'):
                try:
                    self._send(200, render(a), 'text/html; charset=utf-8')
                except Exception as e:  # noqa: BLE001
                    self._send(500, 'render failed: %s' % e)
            else:
                self._send(404, 'not found')

        def do_POST(self):
            body = self.rfile.read(int(self.headers.get('Content-Length', 0))).decode()
            if self.path == '/save':
                try:
                    n = check_yaml(body)
                    mp = Path(a.manual)
                    if mp.is_file():
                        shutil.copy(mp, mp.with_name(mp.name + '.bak'))
                    mp.write_text(body)
                    self._send(200, '已保存 %d 个洞到 %s' % (n, mp))
                except Exception as e:  # noqa: BLE001
                    self._send(400, '保存失败: %s' % e)
            elif self.path == '/rebuild':
                cmd = [sys.executable, '-W', 'ignore', str(script_dir / 'terrain_builder.py'), '--map', str(a.map),
                       '--out', str(a.terrain)]
                if a.ref:
                    cmd += ['--registration-from', str(a.ref)]
                if Path(a.manual).is_file():
                    cmd += ['--manual', str(a.manual)]
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
                tail = '\n'.join((r.stdout + r.stderr).strip().splitlines()[-25:])
                self._send(200 if r.returncode == 0 else 500, tail)
            else:
                self._send(404, 'not found')

        def log_message(self, *args):
            pass

    httpd = ThreadingHTTPServer((a.host, a.port), H)
    print('terrain viewer on http://%s:%d  (manual file %s)' % (a.host, a.port, a.manual), flush=True)
    httpd.serve_forever()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('terrain', help='terrain package dir (terrain_builder output)')
    ap.add_argument('--ref', help='reference package (e.g. mesh-built maps/sim/nav), shown as extra layers')
    ap.add_argument('--map', help='prior map bundle (default: parent of terrain dir), used for rebuild')
    ap.add_argument('--manual', help='zones_manual.yaml to load/save (default: <map>/zones_manual.yaml)')
    ap.add_argument('--draft', help='initial annotations if the manual file does not exist yet')
    ap.add_argument('--out', help='static html output (default: <terrain>/viewer.html)')
    ap.add_argument('--serve', action='store_true')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8765)
    a = ap.parse_args(argv)
    a.map = a.map or str(Path(a.terrain).resolve().parent)
    a.manual = a.manual or str(Path(a.map) / 'zones_manual.yaml')
    if a.serve:
        serve(a)
        return
    out = Path(a.out or Path(a.terrain) / 'viewer.html')
    out.write_text(render(a))
    print('wrote', out)


TEMPLATE = r'''<!doctype html><html><head><meta charset="utf-8"><title>Terrain Annotator</title>
<style>
body{margin:0;font:13px system-ui,sans-serif;display:flex;height:100vh;background:#1b1d21;color:#ddd}
#side{width:340px;overflow:auto;padding:10px;box-sizing:border-box;border-right:1px solid #333}
#main{flex:1;position:relative;min-width:0}
canvas{display:block;width:100%;height:100%;cursor:crosshair}
#info{position:absolute;left:8px;bottom:8px;background:#000b;padding:4px 8px;border-radius:4px;font-family:monospace;white-space:pre}
h3{margin:12px 0 6px;font-size:13px;color:#9cf}
label{display:flex;justify-content:space-between;align-items:center;margin:3px 0}
input[type=number],input[type=text],select{width:150px;background:#2a2d33;color:#eee;border:1px solid #444}
select{width:100%}
button{margin:2px;background:#2f5d8a;color:#fff;border:0;padding:4px 8px;border-radius:3px;cursor:pointer}
button.warn{background:#8a2f2f}button.on{background:#3a8a2f}
#list div{padding:3px 6px;cursor:pointer;border-radius:3px}#list div.sel{background:#2f5d8a}
textarea{width:100%;height:150px;background:#111;color:#cfc;font-family:monospace;font-size:11px;box-sizing:border-box}
.legend{font-size:11px;color:#aaa;margin:4px 0}
#status{color:#fd8;min-height:1.2em;white-space:pre-wrap}
</style></head><body>
<div id="side">
<h3>图层</h3><select id="layer"></select>
<label><span>叠加致命区（暗红）</span><input type="checkbox" id="lethal" checked></label>
<label><span>自动识别的洞（橙）/坡（蓝）</span><input type="checkbox" id="auto" checked></label>
<label><span>参考图低净空区（紫虚线）</span><input type="checkbox" id="refz" checked></label>
<div class="legend" id="legend"></div>
<h3>人工标注的洞</h3><div class="legend" id="src"></div>
<div id="list"></div>
<button id="add">+ 新增（再点地图）</button><button id="del" class="warn">删除选中</button><button id="restore" style="display:none">恢复浏览器草稿</button>
<label><span>id</span><input type="text" id="f_id"></label>
<label><span>中心 x（map）</span><input type="number" step="0.01" id="f_cx"></label>
<label><span>中心 y（map）</span><input type="number" step="0.01" id="f_cy"></label>
<label><span>通行方向 yaw °</span><input type="number" step="1" id="f_yaw"></label>
<label><span>长（沿通行方向）m</span><input type="number" step="0.05" id="f_len"></label>
<label><span>宽（两侧墙之间）m</span><input type="number" step="0.05" id="f_wid"></label>
<label><span>洞底 floor_z m</span><input type="number" step="0.005" id="f_floor"></label>
<label><span>洞顶下沿 roof_z m</span><input type="number" step="0.005" id="f_roof"></label>
<button id="fill">从地图取洞底/洞顶</button><button id="goto">定位到选中</button>
<div class="legend">框只框洞底，两侧墙放在框外。选中后拖动=移动；Q/E 旋转 5°（Shift 1°）；方向键微移 1 cm（Shift 5 cm）；Del 删除；滚轮缩放，拖空白处平移。</div>
<h3>zones_manual.yaml</h3><textarea id="yaml" readonly></textarea>
<button id="save">保存</button><button id="rebuild">保存并重建地形包</button><button id="dl">下载</button><button id="copy">复制</button>
<div id="status"></div>
</div>
<div id="main"><canvas id="cv"></canvas><div id="info"></div></div>
<script>
const D=__DATA__, M=D.meta, dpr=window.devicePixelRatio||1, $=id=>document.getElementById(id);
function dec(s){const b=atob(s),a=new Uint8Array(b.length);for(let i=0;i<b.length;i++)a[i]=b.charCodeAt(i);return a;}
const L={};for(const k in D.layers){L[k]=Object.assign({},D.layers[k],{a:dec(D.layers[k].b64)});}
function cell(x,y){return [Math.floor((x-M.ox)/M.res),Math.floor((y-M.oy)/M.res)];}
function val(n,ix,iy){const l=L[n];if(!l||ix<0||iy<0||ix>=M.nx||iy>=M.ny)return NaN;const v=l.a[ix*M.ny+iy];
 if(l.kind==='cost'||l.kind==='mask')return v;if(v===0)return NaN;return l.lo+(v-1)/254*(l.hi-l.lo);}
function jet(t){t=Math.max(0,Math.min(1,t));const f=c=>Math.round(255*Math.min(1,Math.max(0,1.5-Math.abs(4*t-c))));return [f(3),f(2),f(1)];}
const f3=v=>(+v).toFixed(3), status=m=>{$('status').textContent=m;};
const sel=$('layer');for(const k in L){const o=document.createElement('option');o.value=k;o.textContent=L[k].label;sel.appendChild(o);}
sel.value=L.clearance?'clearance':'cost';
const LEG={cost:'灰度=代价（白=空闲，暗红=致命）',scalar:'彩色：蓝=低 红=高；黑=无数据',clear:'红=净空低（≈0.2 m），蓝绿=中等，灰=开阔；黑=无数据',mask:'绿=先验点云扫到过'};
let man=JSON.parse(JSON.stringify(D.manual||[])), selIdx=man.length?0:-1, adding=false, drag=null;
const KEY='terrain_viewer:'+D.key;
// ---------------- view
const cv=$('cv'), ctx=cv.getContext('2d'), off=document.createElement('canvas'); off.width=M.nx; off.height=M.ny;
const octx=off.getContext('2d'); let S=1,TX=0,TY=0;
function size(){const r=cv.getBoundingClientRect();cv.width=Math.max(1,r.width*dpr);cv.height=Math.max(1,r.height*dpr);}
function fit(){size();S=Math.min(cv.width/M.nx,cv.height/M.ny);TX=(cv.width-M.nx*S)/2;TY=(cv.height-M.ny*S)/2;}
const w2s=(x,y)=>[(x-M.ox)/M.res*S+TX,(M.ny-(y-M.oy)/M.res)*S+TY];
const s2w=(sx,sy)=>[(sx-TX)/S*M.res+M.ox,M.oy+(M.ny-(sy-TY)/S)*M.res];
function renderBase(){const l=L[sel.value],img=octx.createImageData(M.nx,M.ny),d=img.data,cost=L.cost.a,le=$('lethal').checked;
 for(let ix=0;ix<M.nx;ix++)for(let iy=0;iy<M.ny;iy++){const v=l.a[ix*M.ny+iy],p=((M.ny-1-iy)*M.nx+ix)*4;let c;
  if(l.kind==='cost')c=v<3?[110,20,20]:[v,v,v];
  else if(v===0)c=[0,0,0];
  else if(l.kind==='clear'){const t=(v-1)/254;c=t>0.995?[70,70,76]:jet(1-t);}
  else if(l.kind==='mask')c=[70,150,80];
  else c=jet((v-1)/254);
  if(le&&l.kind!=='cost'&&cost[ix*M.ny+iy]<3)c=[c[0]*0.3+80,c[1]*0.3,c[2]*0.3];
  d[p]=c[0];d[p+1]=c[1];d[p+2]=c[2];d[p+3]=255;}
 octx.putImageData(img,0,0);$('legend').textContent=LEG[l.kind]||'';draw();}
function corners(t){const a=t.yaw_deg*Math.PI/180,c=Math.cos(a),s=Math.sin(a),hl=t.size[0]/2,hw=t.size[1]/2;
 return [[hl,hw],[hl,-hw],[-hl,-hw],[-hl,hw]].map(([u,v])=>[t.centre[0]+c*u-s*v,t.centre[1]+s*u+c*v]);}
function inside(t,x,y){const a=t.yaw_deg*Math.PI/180,c=Math.cos(a),s=Math.sin(a),dx=x-t.centre[0],dy=y-t.centre[1];
 return Math.abs(c*dx+s*dy)<=t.size[0]/2&&Math.abs(-s*dx+c*dy)<=t.size[1]/2;}
function arrow(x0,y0,x1,y1,col){const [a,b]=w2s(x0,y0),[c,d]=w2s(x1,y1),ang=Math.atan2(d-b,c-a),h=8*dpr;
 ctx.strokeStyle=col;ctx.fillStyle=col;ctx.beginPath();ctx.moveTo(a,b);ctx.lineTo(c,d);ctx.stroke();
 ctx.beginPath();ctx.moveTo(c,d);ctx.lineTo(c-h*Math.cos(ang-0.4),d-h*Math.sin(ang-0.4));ctx.lineTo(c-h*Math.cos(ang+0.4),d-h*Math.sin(ang+0.4));ctx.fill();}
function label(x,y,txt,col){const [a,b]=w2s(x,y);ctx.fillStyle='#000a';const w=ctx.measureText(txt).width;ctx.fillRect(a+6*dpr,b-14*dpr,w+6*dpr,16*dpr);ctx.fillStyle=col;ctx.fillText(txt,a+9*dpr,b-2*dpr);}
function draw(){ctx.setTransform(1,0,0,1,0,0);ctx.fillStyle='#111';ctx.fillRect(0,0,cv.width,cv.height);
 ctx.imageSmoothingEnabled=false;ctx.drawImage(off,TX,TY,M.nx*S,M.ny*S);
 const ppm=S/M.res;ctx.lineWidth=1.5*dpr;ctx.font=(12*dpr)+'px sans-serif';
 if($('refz').checked)for(const z of D.ref_zones){const [x,y]=w2s(z.x,z.y);ctx.strokeStyle='#d7f';ctx.setLineDash([5*dpr,4*dpr]);ctx.beginPath();ctx.arc(x,y,0.45*ppm,0,7);ctx.stroke();ctx.setLineDash([]);}
 if($('auto').checked){
  for(const r of D.zones.ramps){arrow(r.foot.x,r.foot.y,r.top.x,r.top.y,'#4af');label(r.centre[0],r.centre[1],r.id+' '+r.slope_median_deg+'°','#8cf');}
  for(const t of D.zones.tunnels){const [x,y]=w2s(t.centre[0],t.centre[1]);ctx.strokeStyle='#f93';ctx.beginPath();ctx.arc(x,y,0.3*ppm,0,7);ctx.stroke();label(t.centre[0],t.centre[1],t.id+' '+(t.source||''),'#fb6');}}
 man.forEach((t,k)=>{const c=corners(t).map(p=>w2s(p[0],p[1]));ctx.beginPath();c.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();
  ctx.fillStyle=k===selIdx?'rgba(80,255,120,.35)':'rgba(80,255,120,.15)';ctx.fill();ctx.lineWidth=(k===selIdx?2.5:1.5)*dpr;ctx.strokeStyle=k===selIdx?'#6f9':'#3a6';ctx.stroke();
  const a=t.yaw_deg*Math.PI/180,d=t.size[0]/2+0.35;ctx.lineWidth=1.5*dpr;
  arrow(t.centre[0]-d*Math.cos(a),t.centre[1]-d*Math.sin(a),t.centre[0]+d*Math.cos(a),t.centre[1]+d*Math.sin(a),'#6f9');
  label(t.centre[0]+t.size[1]/2,t.centre[1]+t.size[1]/2,t.id+((t.floor_z==null||t.roof_z==null)?' (缺高度)':' 净空 '+(t.roof_z-t.floor_z).toFixed(3)),'#8fb');});}
// ---------------- list / form / yaml
const FIELDS=['f_id','f_cx','f_cy','f_yaw','f_len','f_wid','f_floor','f_roof'];
function refreshList(){const el=$('list');el.innerHTML='';man.forEach((t,k)=>{const d=document.createElement('div');d.textContent=t.id+'  ('+t.centre[0].toFixed(2)+', '+t.centre[1].toFixed(2)+')  '+t.yaw_deg.toFixed(0)+'°';
  if(k===selIdx)d.className='sel';d.onclick=()=>{selIdx=k;refreshList();toForm();draw();};el.appendChild(d);});
 $('src').textContent='来源: '+D.manual_source+'；保存到: '+D.manual_path;}
function toForm(){const t=man[selIdx];FIELDS.forEach(f=>$(f).disabled=!t);if(!t){FIELDS.forEach(f=>$(f).value='');return;}
 $('f_id').value=t.id;$('f_cx').value=t.centre[0].toFixed(3);$('f_cy').value=t.centre[1].toFixed(3);$('f_yaw').value=t.yaw_deg.toFixed(1);
 $('f_len').value=t.size[0].toFixed(3);$('f_wid').value=t.size[1].toFixed(3);$('f_floor').value=t.floor_z==null?'':t.floor_z.toFixed(3);$('f_roof').value=t.roof_z==null?'':t.roof_z.toFixed(3);}
function fromForm(){const t=man[selIdx];if(!t)return;const n=id=>{const v=parseFloat($(id).value);return isFinite(v)?v:null;};
 t.id=$('f_id').value.trim()||t.id;t.centre=[n('f_cx')??t.centre[0],n('f_cy')??t.centre[1]];t.yaw_deg=n('f_yaw')??t.yaw_deg;
 t.size=[Math.max(0.05,n('f_len')??t.size[0]),Math.max(0.05,n('f_wid')??t.size[1])];t.floor_z=n('f_floor');t.roof_z=n('f_roof');refreshList();changed();}
FIELDS.forEach(f=>$(f).addEventListener('change',fromForm));
function yamlOut(){if(!man.length)return '# zones_manual.yaml (map frame)\ntunnels: []\n';
 let s='# zones_manual.yaml - hand-annotated tunnels, map frame (terrain_viewer)\ntunnels:\n';
 for(const t of man)s+='  - id: '+t.id+'\n    centre: ['+f3(t.centre[0])+', '+f3(t.centre[1])+']\n    yaw_deg: '+t.yaw_deg.toFixed(1)+'\n    size: ['+f3(t.size[0])+', '+f3(t.size[1])+']\n    floor_z: '+(t.floor_z==null?'null':f3(t.floor_z))+'\n    roof_z: '+(t.roof_z==null?'null':f3(t.roof_z))+'\n';
 return s;}
function changed(){draw();$('yaml').value=yamlOut();try{localStorage.setItem(KEY,JSON.stringify(man));}catch(e){}}
// ---------------- sampling
function sampleRect(t,layer){const out=[];if(!L[layer])return out;const c=corners(t),xs=c.map(p=>p[0]),ys=c.map(p=>p[1]);
 for(let x=Math.min(...xs);x<=Math.max(...xs);x+=M.res/2)for(let y=Math.min(...ys);y<=Math.max(...ys);y+=M.res/2){if(!inside(t,x,y))continue;const [i,j]=cell(x,y),v=val(layer,i,j);if(isFinite(v))out.push(v);}return out;}
function q(a,p){if(!a.length)return NaN;a=a.slice().sort((x,y)=>x-y);return a[Math.min(a.length-1,Math.floor(p*a.length))];}
function fillFromMap(t,quiet){let g=sampleRect(t,'ground'),src='地形包';if(g.length<4&&L.ref_ground){g=sampleRect(t,'ref_ground');src='参考图';}
 if(!g.length){if(!quiet)status('框内没有地面数据，请手动填写 floor_z / roof_z');return false;}
 let cl=sampleRect(t,'clearance').filter(v=>v<0.45),csrc='地形包';if(cl.length<4&&L.ref_clearance){cl=sampleRect(t,'ref_clearance').filter(v=>v<0.45);csrc='参考图';}
 const fl=q(g,0.3);t.floor_z=+fl.toFixed(3);t.roof_z=+(fl+(cl.length?q(cl,0.5):0.25)).toFixed(3);
 if(!quiet)status('洞底取自'+src+'（'+g.length+' 个采样），洞顶'+(cl.length?'取自'+csrc+'净空（'+cl.length+' 个采样）':'按 0.25 m 估计，请核对'));return true;}
// ---------------- interaction
function addAt(x,y){let n=man.length;while(man.some(t=>t.id==='tunnel_'+n))n++;const t={id:'tunnel_'+n,centre:[+x.toFixed(3),+y.toFixed(3)],yaw_deg:0,size:[0.9,0.8],floor_z:null,roof_z:null};
 fillFromMap(t);man.push(t);selIdx=man.length-1;refreshList();toForm();changed();}
const ev=e=>[e.offsetX*dpr,e.offsetY*dpr];
cv.addEventListener('mousedown',e=>{const [sx,sy]=ev(e),[x,y]=s2w(sx,sy);
 if(adding){adding=false;$('add').classList.remove('on');addAt(x,y);return;}
 let hit=-1;for(let k=man.length-1;k>=0;k--)if(inside(man[k],x,y)){hit=k;break;}
 if(hit>=0){selIdx=hit;drag={mode:'move',x,y,c0:man[hit].centre.slice()};refreshList();toForm();draw();}
 else drag={mode:'pan',sx,sy,tx:TX,ty:TY};});
cv.addEventListener('mousemove',e=>{const [sx,sy]=ev(e),[x,y]=s2w(sx,sy);
 if(drag&&drag.mode==='pan'){TX=drag.tx+sx-drag.sx;TY=drag.ty+sy-drag.sy;draw();}
 else if(drag&&drag.mode==='move'){const t=man[selIdx];t.centre=[+(drag.c0[0]+x-drag.x).toFixed(3),+(drag.c0[1]+y-drag.y).toFixed(3)];toForm();draw();}
 const [i,j]=cell(x,y),fmt=(n,d)=>{const v=val(n,i,j);return L[n]?(isFinite(v)?v.toFixed(d):'—'):null;};
 let s='map ('+x.toFixed(2)+', '+y.toFixed(2)+')';
 if(D.registration){const r=D.registration,a=r.rpy_deg[2]*Math.PI/180,t=r.t_world_map;s+='  world ('+(Math.cos(a)*x-Math.sin(a)*y+t[0]).toFixed(2)+', '+(Math.sin(a)*x+Math.cos(a)*y+t[1]).toFixed(2)+')';}
 const c=fmt('cost',0),z=fmt('ground',3),cl=fmt('clearance',3),sl=fmt('slope',1);
 s+='\n代价 '+c+(c!==null&&+c<3?'(致命)':'')+'  z '+z+'  净空 '+(cl==='0.600'?'开阔':cl)+'  坡度 '+sl+'°';
 if(L.ref_cost){const rc=fmt('ref_cost',0),rcl=fmt('ref_clearance',3);s+='\n参考图: 代价 '+rc+'  z '+fmt('ref_ground',3)+'  净空 '+(rcl==='0.600'?'开阔':rcl);}
 $('info').textContent=s;});
window.addEventListener('mouseup',()=>{if(drag&&drag.mode==='move')changed();drag=null;});
cv.addEventListener('wheel',e=>{e.preventDefault();const [sx,sy]=ev(e),k=e.deltaY<0?1.25:0.8;TX=sx-(sx-TX)*k;TY=sy-(sy-TY)*k;S*=k;draw();},{passive:false});
document.addEventListener('keydown',e=>{const tg=e.target.tagName;if(tg==='INPUT'||tg==='TEXTAREA'||tg==='SELECT')return;const t=man[selIdx];if(!t)return;
 const st=e.shiftKey,m=st?0.05:0.01;let h=true;switch(e.key){case 'q':case 'Q':t.yaw_deg+=st?1:5;break;case 'e':case 'E':t.yaw_deg-=st?1:5;break;
  case 'ArrowLeft':t.centre[0]-=m;break;case 'ArrowRight':t.centre[0]+=m;break;case 'ArrowUp':t.centre[1]+=m;break;case 'ArrowDown':t.centre[1]-=m;break;
  case 'Delete':case 'Backspace':delSel();return;default:h=false;}
 if(h){e.preventDefault();t.yaw_deg=((t.yaw_deg+180)%360+360)%360-180;t.centre=t.centre.map(v=>+v.toFixed(3));refreshList();toForm();changed();}});
function delSel(){if(selIdx<0)return;man.splice(selIdx,1);selIdx=Math.min(selIdx,man.length-1);refreshList();toForm();changed();}
$('add').onclick=()=>{adding=!adding;$('add').classList.toggle('on',adding);status(adding?'在地图上点击洞的中心':'');};
$('del').onclick=delSel;
$('fill').onclick=()=>{const t=man[selIdx];if(t&&fillFromMap(t)){toForm();changed();}};
$('goto').onclick=()=>{const t=man[selIdx];if(!t)return;size();S=Math.max(S,(cv.height/3)*M.res);const [sx,sy]=w2s(t.centre[0],t.centre[1]);TX+=cv.width/2-sx;TY+=cv.height/2-sy;draw();};
sel.onchange=renderBase;$('lethal').onchange=renderBase;$('auto').onchange=draw;$('refz').onchange=draw;
$('dl').onclick=()=>{const b=new Blob([yamlOut()],{type:'text/yaml'}),a=document.createElement('a');a.href=URL.createObjectURL(b);a.download='zones_manual.yaml';a.click();};
$('copy').onclick=()=>{const y=yamlOut();(navigator.clipboard?navigator.clipboard.writeText(y):Promise.reject()).then(()=>status('已复制'),()=>{$('yaml').select();document.execCommand('copy');status('已复制');});};
const served=location.protocol.startsWith('http');
if(!served){$('save').style.display='none';$('rebuild').style.display='none';}
function save(){if(man.some(t=>t.floor_z==null||t.roof_z==null)){status('有洞缺 floor_z / roof_z，先补全');return Promise.reject();}
 return fetch('save',{method:'POST',body:yamlOut()}).then(r=>r.text().then(m=>{status(m);if(!r.ok)throw m;}));}
$('save').onclick=()=>{save().catch(()=>{});};
$('rebuild').onclick=()=>{save().then(()=>{status('重建中…');return fetch('rebuild',{method:'POST'});}).then(r=>r.text().then(m=>{status(m);if(r.ok)setTimeout(()=>location.reload(),1500);})).catch(()=>{});};
try{const s=localStorage.getItem(KEY);if(s&&s!==JSON.stringify(man)){$('restore').style.display='';$('restore').onclick=()=>{man=JSON.parse(s);selIdx=man.length?0:-1;refreshList();toForm();changed();$('restore').style.display='none';};}}catch(e){}
let missing=0;for(const t of man)if(t.floor_z==null||t.roof_z==null){if(!fillFromMap(t,true))missing++;}
window.addEventListener('resize',()=>{const w=cv.width,h=cv.height;size();TX+=(cv.width-w)/2;TY+=(cv.height-h)/2;draw();});
fit();renderBase();refreshList();toForm();$('yaml').value=yamlOut();
status(missing?missing+' 个洞缺高度，请手动填写':(served?'':'静态页面：改完用“下载”或“复制”，放到 '+D.manual_path));
</script></body></html>
'''


if __name__ == '__main__':
    main()
