#!/usr/bin/env python3
"""surface_project: surface patches + operator annotations (type, passability, direction, extent).

A project holds the height grid of one source (field model or prior cloud), its automatic
segmentation and the operator's work on top of it:
  owner   int32 [H, W]  patch id of every cell; starts as the segmentation and changes with
                        merge / grow / shrink / brush (the size correction)
  ann     {patch id: {'type': label id or -1 (not annotated), 'passable': bool,
                      'dir': map/field-frame angle in degrees or None (= geometric default)}}
Nothing is guessed: a new patch is not annotated.  Re-running the segmentation hands each new
patch the annotation of the old patch it overlaps most (> half of its cells); the extent edits are
replaced by the new segmentation.

A prior-cloud project (frame 'map') can be registered onto a field-model project (frame 'field'),
p_field = R p_map + t, and then adopt the model's patches and annotations: a sparse cloud splits one
surface into many fragments, the model's patches are exact.  The cloud keeps its own heights.

Export: terrain = annotated type; not passable, no data and not annotated -> OBSTACLE (everything is
an obstacle until the operator opens it up; not annotated cells are counted in the report), then terrain_export (Nav2 maps, direction field, zones).
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
import surface_seg as ss  # noqa: E402
from terrain_map import (OBSTACLE, SLOPE, STEP_HIGH, STEP_L1, STEP_L2, TUNNEL, UNDULATION,  # noqa: E402
                         DIRECTIONAL, TerrainMap, encode_angle)

UNSET = -1
FORMAT = 1


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(r, p, y):
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rot_z(y) @ ry @ rx


def matrix_rpy(R):
    return (math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


def reg_matrix(reg):
    return rpy_matrix(*[math.radians(v) for v in reg['rpy_deg']]), np.asarray(reg['t_world_map'], float)


def icp(target, source, guess_xyz, guess_yaw, iters=60):
    """Point-to-point ICP, source (map) onto target (field) -> registration dict."""
    tree = cKDTree(target)
    R, t = rot_z(guess_yaw), np.asarray(guess_xyz, float)
    for it in range(iters):
        q = source @ R.T + t
        d, j = tree.query(q)
        m = d < (0.5 if it < 15 else (0.2 if it < 35 else 0.08))
        if m.sum() < 100:
            raise ValueError('ICP lost the overlap (%d matches); give a better start pose' % int(m.sum()))
        a, b = q[m], target[j[m]]
        ca, cb = a.mean(0), b.mean(0)
        u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
        dr = vt.T @ np.diag([1, 1, np.sign(np.linalg.det(vt.T @ u.T))]) @ u.T
        R, t = dr @ R, dr @ t + cb - dr @ ca
    d, _ = tree.query(source @ R.T + t)
    return {'t_world_map': [float(v) for v in t], 'rpy_deg': [math.degrees(v) for v in matrix_rpy(R)],
            'residual_p50': float(np.percentile(d, 50)), 'residual_p90': float(np.percentile(d, 90)),
            'inlier_ratio_8cm': float((d < 0.08).mean())}


class Project:
    def __init__(self, surf, params, frame, source=''):
        self.surf, self.params, self.frame, self.source = surf, params, frame, str(source)
        self.seg = self.owner = None
        self.grad = None
        self.stats = {}
        self.ann = {}
        self.reg = None
        self.undo_stack, self.redo_stack = [], []
        self._before = None

    # ------------------------------------------------------------------ construction
    @classmethod
    def from_mesh(cls, tri, params, source='', log=print):
        p = cls(ss.mesh_surface(tri, params, log), params, 'field', source)
        p.segment(log=log)
        return p

    @classmethod
    def from_cloud(cls, points, params, source='', log=print):
        p = cls(ss.cloud_surface(points, params, log), params, 'map', source)
        p.segment(log=log)
        return p

    @property
    def shape(self):
        return self.surf.z.shape

    # ------------------------------------------------------------------ segmentation
    def segment(self, params=None, log=print):
        """(Re)run the segmentation; annotations move to the most-overlapping new patch."""
        if params is not None:
            self.params = params
        new = ss.segment(self.surf, self.params)
        old, old_ann = self.owner, self.ann
        ann, moved = {}, 0
        if old is not None and old_ann:
            m = (new > 0) & (old > 0)
            L, Lo = int(new.max()) + 1, int(old.max()) + 1
            pair, cnt = np.unique(new[m].astype(np.int64) * Lo + old[m], return_counts=True)
            pn, po = pair // Lo, pair % Lo
            size = np.bincount(new.ravel(), minlength=L)
            order = np.lexsort((-cnt, pn))
            pn, po, cnt = pn[order], po[order], cnt[order]
            first = np.r_[True, pn[1:] != pn[:-1]]
            for a, b, c in zip(pn[first], po[first], cnt[first]):
                if int(b) in old_ann and c * 2 > size[a]:
                    ann[int(a)] = dict(old_ann[int(b)])
                    moved += 1
        self.grad = ss.cell_gradient(self.surf, self.params)
        self.seg, self.owner, self.ann = new, new.copy(), ann
        self.undo_stack, self.redo_stack = [], []
        self.refresh()
        log('分割出 %d 个面片%s' % (len(self.ids()), '，保留 %d 个标注（原有 %d 个）' % (moved, len(old_ann))
                                    if old_ann else ''))
        return moved

    def refresh(self):
        self.stats = ss.patch_stats(self.surf, self.owner, self.params)

    def ids(self):
        n = self.stats.get('n')
        return [] if n is None else [int(i) for i in np.nonzero(n > 0)[0] if i > 0]

    def info(self, pid):
        """Geometry of one patch (dict) or None."""
        s = self.stats
        if pid <= 0 or pid >= len(s['n']) or s['n'][pid] == 0:
            return None
        d = {k: float(v[pid]) for k, v in s.items()}
        d['groups'] = self.dir_groups(pid)[0]
        d['dirs'] = [g['az'] for g in d['groups']]
        d['multi'] = len(d['groups']) > 1
        if d['multi']:          # the whole-patch plane of a ring / ridge is ~flat: report the faces instead
            w = sum(g['area'] for g in d['groups'])
            d['incline'] = sum(g['incline'] * g['area'] for g in d['groups']) / w
        d['kind'] = 'slope' if d['incline'] >= self.params.flat_deg else 'plane'
        along = d['azimuth'] if d['kind'] == 'slope' and not d['multi'] else d['axis']
        d['length'], d['width'] = self.extent(pid, along)
        return d

    def dir_groups(self, pid):
        # uphill directions inside the patch seen from above -> (groups, (rows, cols, group per cell))
        gx, gy = self.grad
        return ss.slope_groups(gx, gy, self.owner == pid, self.surf.resolution, self.surf.origin,
                               self.params.flat_deg)

    def is_local(self, pid):
        # multi-direction slope (fortress hexagon ring, ridge) on the geometric direction: one per face
        a = self.a(pid)
        return a['dir'] is None and a['type'] in (SLOPE, UNSET) and len(self.dir_groups(pid)[0]) > 1

    def arrows(self, pid):
        # [(x, y, deg)]: one arrow per face of a multi-direction slope, else one
        i = self.info(pid)
        if i is None:
            return []
        if self.is_local(pid):
            off = 180.0 if self.a(pid).get('rev') else 0.0
            return [(g['cx'], g['cy'], (g['az'] + off) % 360.0) for g in i['groups']]
        return [(i['cx'], i['cy'], self.direction(pid))]

    def extent(self, pid, along_deg):
        r, c = np.nonzero(self.owner == pid)
        res = self.surf.resolution
        a = math.radians(along_deg)
        u = c * math.cos(a) + r * math.sin(a)
        v = -c * math.sin(a) + r * math.cos(a)
        return float((u.max() - u.min() + 1) * res), float((v.max() - v.min() + 1) * res)

    def neighbours(self, pid):
        return ss.neighbours(self.surf, self.owner, pid)

    def patch_at(self, col, row):
        h, w = self.shape
        return int(self.owner[row, col]) if 0 <= row < h and 0 <= col < w else 0

    # ------------------------------------------------------------------ annotation
    def a(self, pid):
        return self.ann.get(int(pid), {'type': UNSET, 'passable': True, 'dir': None, 'rev': False})

    def default_dir(self, pid, typ):
        """Geometric direction for a type: slope uphill, step lower -> higher, passage long axis."""
        i = self.info(pid)
        if i is None:
            return 0.0
        if typ in (STEP_L1, STEP_L2, STEP_HIGH):
            if i['step_w'] > 1e-6:
                return i['step_dir']
            nb = [(n, dz) for n, _, dz in self.neighbours(pid) if abs(dz) > self.params.jump / 2]
            if nb:                               # brushed step band: from the lower to the higher neighbour
                vx = sum(dz * (self.stats['cx'][n] - i['cx']) for n, dz in nb)
                vy = sum(dz * (self.stats['cy'][n] - i['cy']) for n, dz in nb)
                if abs(vx) + abs(vy) > 1e-9:
                    return math.degrees(math.atan2(vy, vx)) % 360.0
        if typ in (TUNNEL, UNDULATION) or i['kind'] == 'plane':
            return i['axis']
        return i['azimuth']

    def direction(self, pid):
        a = self.a(pid)
        if a['dir'] is not None:
            return a['dir']
        return (self.default_dir(pid, a['type']) + (180.0 if a.get('rev') else 0.0)) % 360.0

    def set_ann(self, pids, **kw):
        """kw: type=, passable=, dir= (None = geometric default)."""
        pids = [int(p) for p in pids if p > 0]
        if not pids:
            return 0
        self.begin()
        for p in pids:
            a = dict(self.a(p))
            if 'type' in kw and kw['type'] != a['type']:
                a['type'] = int(kw['type'])
                a['dir'], a['rev'] = None, False
            for k in ('passable', 'dir', 'rev'):
                if k in kw:
                    a[k] = kw[k]
            if a['type'] == UNSET and a['passable'] and a['dir'] is None and not a.get('rev'):
                self.ann.pop(p, None)
            else:
                self.ann[p] = a
        return self.commit()

    def flip_dir(self, pids):
        self.begin()
        for p in pids:
            a = dict(self.a(p))
            if a['dir'] is None:                     # geometric (possibly per-face) direction: reverse all
                a['rev'] = not a.get('rev', False)
            else:
                a['dir'] = (a['dir'] + 180.0) % 360.0
            self.ann[int(p)] = a
        return self.commit()

    # ------------------------------------------------------------------ extent (size correction)
    def merge(self, pids):
        """Join patches into the first annotated one (else the largest); -> resulting id."""
        pids = [int(p) for p in dict.fromkeys(pids) if p > 0]
        if len(pids) < 2:
            return pids[0] if pids else 0
        keep = next((p for p in pids if self.a(p)['type'] != UNSET), max(pids, key=lambda p: self.stats['n'][p]))
        self.begin()
        self.owner[np.isin(self.owner, pids)] = keep
        for p in pids:
            if p != keep:
                self.ann.pop(p, None)
        self.commit()
        return keep

    def _valid(self):
        return np.isfinite(self.surf.z)

    def grow(self, pid, cells=1):
        """Extend the patch by `cells` rings into the surrounding patches."""
        self.begin()
        m = self.owner == pid
        g = ndimage.binary_dilation(m, iterations=int(cells)) & self._valid()
        self.owner[g] = pid
        return self.commit()

    def shrink(self, pid, cells=1):
        """Give the outer `cells` rings of the patch to the patch next to each cell."""
        self.begin()
        for _ in range(int(cells)):
            m = self.owner == pid
            ring = m & ~ndimage.binary_erosion(m, border_value=1)
            other = np.where(m, 0, self.owner)
            near = ndimage.grey_dilation(other, size=(3, 3))
            take = ring & (near > 0)
            if not take.any() or take.sum() >= m.sum():
                break
            self.owner[take] = near[take]
        return self.commit()

    def paint(self, mask, pid):
        """Brush: hand cells to patch `pid` (0 -> a new patch).  -> patch id."""
        mask = np.asarray(mask, bool) & self._valid()
        if not mask.any():
            return pid
        src = 0
        if pid <= 0:                             # new patch: takes the annotation of the patch it was cut from
            pid = self.new_id()
            o = self.owner[mask]
            o = o[o > 0]
            src = int(np.bincount(o).argmax()) if o.size else 0
        self.begin()
        if src in self.ann:
            self.ann[pid] = dict(self.ann[src])
        self.owner[mask] = pid
        self.commit()
        return pid

    def new_id(self):
        return int(max(self.owner.max(), self.seg.max(), max(self.ann, default=0))) + 1

    def restore_cells(self, mask):
        # cells back to the automatic segmentation
        mask = np.asarray(mask, bool)
        self.begin()
        self.owner[mask] = self.seg[mask]
        return self.commit()

    def split_line(self, a, b, pids=None):
        """Cut along the cell line a -> b (col, row).  Every patch the line crosses completely (restricted to
        `pids` if given) falls apart into its pieces; the largest keeps the id, the others get new ids and
        a copy of the annotation.  -> ids of all pieces (empty: nothing was cut through)."""
        h, w = self.shape
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1)
        k = np.arange(n + 1)
        cc = np.rint(a[0] + (b[0] - a[0]) * k / n).astype(np.int64)
        rr = np.rint(a[1] + (b[1] - a[1]) * k / n).astype(np.int64)
        ok = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
        line = np.zeros((h, w), bool)
        line[rr[ok], cc[ok]] = True
        line |= ndimage.binary_dilation(line, structure=np.array([[0, 0, 0], [0, 1, 1], [0, 0, 0]], bool))
        hit = [int(v) for v in np.unique(self.owner[line]) if v > 0]
        if pids:
            hit = [v for v in hit if v in set(int(x) for x in pids)]
        self.begin()
        out, nxt = [], self.new_id()
        for pid in hit:
            m = self.owner == pid
            comp, nc = ndimage.label(m & ~line)
            if nc < 2:
                continue
            size = np.bincount(comp.ravel())
            size[0] = 0
            big = int(np.argmax(size))
            on = m & line                       # line cells join the piece next to them
            _, (ir, ic) = ndimage.distance_transform_edt(comp == 0, return_indices=True)
            comp[on] = comp[ir[on], ic[on]]
            ids = [pid]
            for j in range(1, nc + 1):
                if j == big:
                    continue
                self.owner[m & (comp == j)] = nxt
                if pid in self.ann:
                    self.ann[nxt] = dict(self.ann[pid])
                ids.append(nxt)
                nxt += 1
            out += ids
        self.commit()
        return out

    def flood_level(self, seeds, tol=0.02, min_border=3):
        """Seeds plus every plane patch reachable through borders without a step (|dz| <= tol on at least
        min_border cells); stops at slopes, multi-direction patches and patches annotated differently."""
        s = self.stats
        flat = (s['incline'] < self.params.flat_deg) & (s['n'] > 0)
        a, b, cnt, cont = ss.adjacency(self.surf, self.owner, tol)
        keep = cont >= min_border
        nb = {}
        for x, y in zip(a[keep].tolist(), b[keep].tolist()):
            nb.setdefault(x, []).append(y)
            nb.setdefault(y, []).append(x)
        seeds = [int(x) for x in seeds if x > 0]
        types = {self.a(x)['type'] for x in seeds}
        seen, todo = set(seeds), list(seeds)
        while todo:
            x = todo.pop()
            for y in nb.get(x, ()):
                if y in seen or not flat[y] or self.a(y)['type'] not in types | {UNSET}:
                    continue
                seen.add(y)
                todo.append(y)
        return seeds + sorted(seen - set(seeds))

    def reset_extent(self, pids):
        """Cells of these patches and cells they took over go back to the segmentation."""
        pids = [int(p) for p in pids]
        self.begin()
        m = np.isin(self.owner, pids) | np.isin(self.seg, pids)
        self.owner[m] = self.seg[m]
        return self.commit()

    # ------------------------------------------------------------------ undo
    def begin(self):
        if self._before is None:
            self._before = (self.owner.copy(), {k: dict(v) for k, v in self.ann.items()})

    def commit(self):
        o0, a0 = self._before
        self._before = None
        diff = o0 != self.owner
        if not diff.any() and a0 == self.ann:
            return 0
        if diff.any():
            r, c = np.nonzero(diff)
            sl = np.s_[r.min():r.max() + 1, c.min():c.max() + 1]
            rec = (sl, o0[sl].copy(), self.owner[sl].copy())
        else:
            rec = None
        self.undo_stack.append((rec, a0, {k: dict(v) for k, v in self.ann.items()}))
        del self.undo_stack[:-200]
        self.redo_stack.clear()
        if diff.any():
            self.refresh()
        return int(diff.sum()) or 1

    def _apply(self, src, dst, old):
        if not src:
            return False
        rec, a_old, a_new = src.pop()
        if rec is not None:
            sl, before, after = rec
            self.owner[sl] = before if old else after
        self.ann = {k: dict(v) for k, v in (a_old if old else a_new).items()}
        dst.append((rec, a_old, a_new))
        if rec is not None:
            self.refresh()
        return True

    def undo(self):
        return self._apply(self.undo_stack, self.redo_stack, True)

    def redo(self):
        return self._apply(self.redo_stack, self.undo_stack, False)

    # ------------------------------------------------------------------ registration / transfer
    def register_to(self, field_proj, guess_xyz, guess_yaw_deg, log=print):
        if self.frame != 'map' or field_proj.frame != 'field':
            raise ValueError('register a prior-cloud project onto a field-model project')
        tgt = ss.surface_points(field_proj.surf, field_proj.params)
        src = ss.surface_points(self.surf, self.params, max_points=200000)
        self.reg = icp(tgt, src, guess_xyz, math.radians(guess_yaw_deg))
        log('配准：t %s，rpy %s°，残差 p50 %.3f / p90 %.3f m，内点 %.0f %%'
            % (np.round(self.reg['t_world_map'], 3).tolist(), np.round(self.reg['rpy_deg'], 2).tolist(),
               self.reg['residual_p50'], self.reg['residual_p90'], 100 * self.reg['inlier_ratio_8cm']))
        return self.reg

    def field_lookup(self, field_proj, grid=None):
        """Field patch id (or a value of another field grid) under the centre of every cell of this
        (map-frame) project; 0 / NaN outside the field grid."""
        R, t = reg_matrix(self.reg)
        h, w = self.shape
        rr, cc = np.mgrid[0:h, 0:w]
        res = self.surf.resolution
        x = self.surf.origin[0] + (cc + 0.5) * res
        y = self.surf.origin[1] + (rr + 0.5) * res
        fx = R[0, 0] * x + R[0, 1] * y + t[0]
        fy = R[1, 0] * x + R[1, 1] * y + t[1]
        fs = field_proj.surf
        fc = np.floor((fx - fs.origin[0]) / fs.resolution).astype(np.int64)
        fr = np.floor((fy - fs.origin[1]) / fs.resolution).astype(np.int64)
        fh, fw = field_proj.shape
        ok = (fc >= 0) & (fc < fw) & (fr >= 0) & (fr < fh)
        src = field_proj.owner if grid is None else grid
        out = np.zeros((h, w), np.int64) if grid is None else np.full((h, w), np.nan)
        out[ok] = src[fr[ok], fc[ok]]
        return out

    def adopt_from(self, field_proj, log=print):
        """Replace this project's patches by the field model's (same ids) and copy its annotations.

        Every cloud cell takes the model patch under its centre.  Cells of a model patch without cloud
        data (tunnel floors under the roof, scan shadows) take the model height, so the patch stays
        whole; cells outside the model get no patch (obstacle on export).  Directions are rotated into the map frame; a
        multi-direction slope on its geometric direction stays per face.  Like a re-segmentation this
        resets the undo history and the 'restore' extent (now the model patches)."""
        if self.reg is None:
            raise ValueError('register first')
        before = len(self.ids())
        look = self.field_lookup(field_proj)
        hole = (look > 0) & ~np.isfinite(self.surf.z)
        if hole.any():
            fz = self.field_lookup(field_proj, field_proj.surf.z)
            R, t = reg_matrix(self.reg)
            z = self.surf.z.copy()
            z[hole] = (fz[hole] - t[2]) / max(R[2, 2], 1e-6)     # field height back into the map frame
            self.surf.z = z.astype(self.surf.z.dtype)
            self.grad = ss.cell_gradient(self.surf, self.params)
            log('%d 个格子点云无数据，用模型高程补齐' % int(hole.sum()))
        look[~np.isfinite(self.surf.z)] = 0
        self.owner = look.astype(np.int32)
        self.seg = self.owner.copy()
        self.refresh()
        yaw = math.degrees(matrix_rpy(reg_matrix(self.reg)[0])[2])
        present = set(int(v) for v in np.unique(self.owner))
        ann = {}
        for fp, a in field_proj.ann.items():
            if fp not in present:
                continue
            b = dict(a)
            if a['type'] in DIRECTIONAL and not field_proj.is_local(fp):
                b['dir'], b['rev'] = (field_proj.direction(fp) - yaw) % 360.0, False
            ann[fp] = b
        self.ann = ann
        self.undo_stack, self.redo_stack = [], []
        log('采用模型面片：%d 个点云面片 -> %d 个模型面片，%d 个标注（模型共 %d 个）'
            % (before, len(self.ids()), len(ann), len(field_proj.ann)))
        return len(ann)

    # ------------------------------------------------------------------ output
    def label_grids(self):
        """-> terrain (annotated, -1 = not annotated), direction code, passable (bool) [H, W]."""
        L = int(self.owner.max()) + 1
        typ = np.full(L, UNSET, np.int16)
        pas = np.ones(L, bool)
        dcode = np.zeros(L, np.uint8)
        for p, a in self.ann.items():
            if p < L:
                typ[p], pas[p] = a['type'], bool(a['passable'])
                if a['type'] in DIRECTIONAL:
                    dcode[p] = encode_angle(math.radians(self.direction(p)))
        typ[0] = UNSET
        dgrid = dcode[self.owner]
        for p, a in self.ann.items():
            if p < L and a['type'] == SLOPE and self.is_local(p):
                groups, (r, c, asg) = self.dir_groups(p)
                off = 180.0 if a.get('rev') else 0.0
                codes = np.array([encode_angle(math.radians(g['az'] + off)) for g in groups], np.uint8)
                k = asg >= 0
                dgrid[r[k], c[k]] = codes[asg[k]]
        return typ[self.owner], dgrid, pas[self.owner]

    def terrain_map(self):
        typ, dcode, pas = self.label_grids()
        valid = self._valid()
        terrain = np.where(typ >= 0, typ, OBSTACLE).astype(np.uint8)      # not annotated = not passable
        terrain[~pas | ~valid] = OBSTACLE
        direction = np.where(np.isin(terrain, list(DIRECTIONAL)), dcode, 0).astype(np.uint8)
        m = TerrainMap(terrain, direction, self.surf.resolution, tuple(self.surf.origin),
                       elevation=self.surf.z.astype(np.float32))
        m.extra['frame'] = self.frame
        m.extra['source'] = 'surface_studio %s' % self.source
        if self.reg is not None:
            m.extra['registration'] = dict(self.reg)
        return m, {'unannotated_cells': int((valid & (typ < 0)).sum()),
                   'blocked_cells': int((valid & ~pas).sum()), 'no_data_cells': int((~valid).sum())}

    def export(self, out, cfg=None):
        import terrain_export as tx
        out = Path(out)
        m, rep = self.terrain_map()
        if cfg is None:
            cfg = tx.build_parser().parse_args(['x', '--out', str(out)])
            cfg.speed_limits = {'tunnel': 35, 'slope': 60, 'undulation': 50}
        cfg.src = Path(self.source or 'surface_studio')
        meta, zones = tx.export(m, out, cfg)
        m.save(out / 'terrain.msgpack')
        meta.update(rep)
        meta['frame'] = self.frame
        (out / 'terrain.json').write_text(json.dumps(meta, indent=2))
        return meta, zones

    # ------------------------------------------------------------------ files
    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        meta = {'format': FORMAT, 'frame': self.frame, 'source': self.source, 'params': self.params.to_dict(),
                'resolution': self.surf.resolution, 'origin': list(self.surf.origin), 'reg': self.reg,
                'ann': {str(k): v for k, v in self.ann.items()}}
        tmp = path.with_name(path.name + '.tmp.npz')
        grads = {'gx': self.surf.gx, 'gy': self.surf.gy} if self.surf.gx is not None else {}
        np.savez_compressed(tmp, z=self.surf.z, clearance=self.surf.clearance, seg=self.seg,
                            owner=self.owner, meta=np.frombuffer(json.dumps(meta).encode(), np.uint8), **grads)
        tmp.replace(path)

    @classmethod
    def load(cls, path):
        d = np.load(path)
        meta = json.loads(bytes(d['meta']).decode())
        surf = ss.Surface(d['z'], float(meta['resolution']), tuple(meta['origin']), d['clearance'],
                          d['gx'] if 'gx' in d.files else None, d['gy'] if 'gy' in d.files else None)
        p = cls(surf, ss.SegParams.from_dict(meta['params']), meta['frame'], meta.get('source', ''))
        p.seg, p.owner = d['seg'].astype(np.int32), d['owner'].astype(np.int32)
        p.ann = {int(k): v for k, v in meta.get('ann', {}).items()}
        p.reg = meta.get('reg')
        p.grad = ss.cell_gradient(p.surf, p.params)
        p.refresh()
        return p
