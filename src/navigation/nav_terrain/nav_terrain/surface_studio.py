"""surface_studio: 地形面片标注工具。

打开官方场地模型（STL / STEP / npz）或先验点云（PCD / 建图目录），软件自动分割出平面和斜面，
给出每块的坡度、朝向、面积、高度；类型、可通过性、方向和大小全部由人工标注 / 修正，最后导出 Nav2 地图。

    surface_studio.py [文件]

操作
  左键            选中面片          Ctrl+左键  加选 / 取消      Shift+左键拖动  框选
  中键拖动 / 滚轮  平移 / 缩放      B          画笔（把格子划给选中的面片，未选中则新建面片）
  1-8            设置类型          0          清除类型        X  切换可通过     R  方向反向
  M              合并选中          + / -      扩大 / 缩小选中面片一圈
  Ctrl+Z / Ctrl+Y 撤销 / 重做      Ctrl+S     保存工程
"""
import os
import subprocess
import sys
import tempfile
from importlib.util import find_spec
from pathlib import Path

import numpy as np

from nav_terrain import surface_project as sp
from nav_terrain import surface_seg as ss
from nav_terrain.point_cloud import load_keyframe_cloud, read_pcd
from nav_terrain.studio_render import compose_map, patch_edges
from nav_terrain.terrain_map import DIRECTIONAL

try:
    from PyQt5 import QtWidgets as W
    from PyQt5.QtCore import QPointF, Qt, QTimer
except ImportError:  # pragma: no cover - the core modules work without Qt
    W = None

_APP = None
_OPEN3D_REQUIRED = '三维预览需要 Open3D；请按 nav_terrain/README.md 安装依赖，并用 ros2 run nav_terrain surface_studio.py 启动。'


def make_app(argv):
    global _APP
    _APP = W.QApplication.instance() or W.QApplication(argv)
    return _APP


def disc(r):
    y, x = np.ogrid[-r:r + 1, -r:r + 1]
    return x * x + y * y <= r * r + 0.5


if W is not None:
    from nav_terrain.studio_layout import EditorLayout, TYPE_COLOR, TYPE_ZH
    from nav_terrain.studio_inspector import patch_details
    from nav_terrain.studio_widgets import Task

    class Studio(W.QMainWindow, EditorLayout):
        def __init__(self, sync=False):
            super().__init__()
            self.sync = sync
            self.p = None            # working project
            self.ref = None          # field-model project (registration reference)
            self.path = None
            self.sel = []
            self.dirty = False
            self.overlay = None
            self.params = ss.SegParams()
            self._tasks = []
            self._stroke = None
            self._paint_box = False
            self._updating = False
            self.setWindowTitle('地形面片标注')
            self._build()
            self.render_timer = QTimer(self, singleShot=True, interval=30, timeout=self.render)

        # -------------------------------------------------------------- ui
        def log(self, msg):
            self.logbox.appendPlainText(str(msg))
            self.status.showMessage(str(msg), 5000)

        def warn(self, msg):
            self.log('⚠ ' + msg)
            if not self.sync:
                W.QMessageBox.warning(self, '地形面片标注', msg)

        def run(self, fn, done, what):
            if self.sync:
                done(fn(self.log))
                return
            self.log(what + '…')
            self.setEnabled(False)
            t = Task(fn)
            t.log.connect(self.log)
            t.done.connect(lambda r: (self.setEnabled(True), done(r)))
            t.failed.connect(lambda e: (self.setEnabled(True), self.warn('%s失败：%s' % (what, e))))
            t.finished.connect(lambda: self._tasks.remove(t))
            self._tasks.append(t)
            t.start()

        # -------------------------------------------------------------- params
        def set_params(self, p):
            self.params = p
            for k, s in self.pw.items():
                s.setValue(getattr(p, k))

        def read_params(self):
            p = ss.SegParams(**{k: s.value() for k, s in self.pw.items()})
            self.params = p
            return p

        # -------------------------------------------------------------- files
        def _ok_discard(self):
            if not self.dirty or self.sync:
                return True
            r = W.QMessageBox.question(self, '地形面片标注', '当前工程有未保存的修改，先保存吗？',
                                       W.QMessageBox.Save | W.QMessageBox.Discard | W.QMessageBox.Cancel)
            if r == W.QMessageBox.Save:
                return self.save_project()
            return r == W.QMessageBox.Discard

        def ask_open_model(self):
            f, _ = W.QFileDialog.getOpenFileName(self, '打开场地模型', '', '场地模型 (*.stl *.step *.stp *.npz)')
            if f:
                self.open_model(f)

        def ask_open_cloud(self):
            f, _ = W.QFileDialog.getOpenFileName(self, '打开先验点云（取消则选择建图目录）', '', '点云 (*.pcd)')
            if not f:
                f = W.QFileDialog.getExistingDirectory(self, '选择建图目录（含 keyframes.txt）')
            if f:
                self.open_cloud(f)

        def ask_open_project(self):
            f, _ = W.QFileDialog.getOpenFileName(self, '打开工程', '', '标注工程 (*.surf.npz *.npz)')
            if f:
                self.open_project(f)

        def ask_open_ref(self):
            f, _ = W.QFileDialog.getOpenFileName(self, '参考场地', '', '模型或工程 (*.stl *.step *.stp *.npz)')
            if f:
                self.open_ref(f)

        def _load_model_project(self, path, log):
            from nav_terrain import mesh_io
            if path.endswith('.surf.npz'):
                return sp.Project.load(path)
            tri = mesh_io.load_mesh(path, [s.value() for s in self.step_off], log=log)
            return sp.Project.from_mesh(tri, self.read_params(), path, log)

        def open_model(self, path):
            if not self._ok_discard():
                return
            self.run(lambda log: self._load_model_project(str(path), log),
                     lambda p: self._set_project(p, None), '读取模型并分割')

        def open_cloud(self, path):
            if not self._ok_discard():
                return
            ref = self.p if self.p is not None and self.p.frame == 'field' else self.ref

            def work(log):
                pts = load_keyframe_cloud(path) if Path(path).is_dir() else read_pcd(path)
                return sp.Project.from_cloud(pts, self.read_params(), path, log)
            self.run(work, lambda p: self._set_project(p, None, ref), '读取点云并分割')

        def open_project(self, path):
            if not self._ok_discard():
                return
            self._set_project(sp.Project.load(path), path)

        def open_ref(self, path):
            def done(p):
                if p.frame != 'field':
                    self.warn('参考必须是场地模型工程')
                    return
                self.ref = p
                self._ref_changed()
            self.run(lambda log: self._load_model_project(str(path), log), done, '读取参考场地')

        def _set_project(self, p, path, ref=None):
            self.p, self.path, self.sel, self.dirty = p, path, [], False
            if ref is not None:
                self.ref = ref
            self.set_params(p.params)
            self._make_overlay()
            self._ref_changed()
            self.update_title()
            self.refresh_list()
            self.update_panel()
            self.render(refit=True)
            self.log('%s：%d 个面片，%s坐标系' % (Path(p.source).name or '工程', len(p.ids()),
                                          '场地' if p.frame == 'field' else '地图'))

        def _ref_changed(self):
            r = self.ref
            self.ref_label.setText('参考场地：%s（%d 个面片，%d 个已标注）' % (Path(r.source).name, len(r.ids()), len(r.ann))
                                   if r is not None else '参考场地：无')
            if self.p is not None and self.p.reg is not None:
                self._show_reg()

        def save_project(self, path=None):
            if self.p is None:
                return False
            path = self.path if path is None else path
            if path is None:
                if self.sync:
                    return False
                base = Path(self.p.source).with_suffix('').name or 'terrain'
                path, _ = W.QFileDialog.getSaveFileName(self, '保存工程', base + '.surf.npz', '标注工程 (*.surf.npz)')
                if not path:
                    return False
                if not path.endswith('.npz'):
                    path += '.surf.npz'
            self.p.save(path)
            self.path, self.dirty = path, False
            self.update_title()
            self.log('已保存 %s' % path)
            return True

        def ask_export(self):
            if self.p is None:
                return
            d = W.QFileDialog.getExistingDirectory(self, '导出到目录（terrain_dir）')
            if d:
                self.export_to(d)

        def export_to(self, out):
            if self.p is None:
                return None
            meta, zones = self.p.export(out)
            msg = '已导出 %s：%d 个方向区域' % (out, len(zones))
            if meta['unannotated_cells']:
                msg += '；%d 格未标注，按障碍（不可通行）导出' % meta['unannotated_cells']
            if meta['blocked_cells']:
                msg += '；%d 格禁止通行' % meta['blocked_cells']
            self.log(msg)
            return meta

        def update_title(self):
            name = Path(self.path).name if self.path else (Path(self.p.source).name if self.p else '')
            self.setWindowTitle('地形面片标注 - %s%s' % (name, ' *' if self.dirty else ''))

        def closeEvent(self, e):
            if self._ok_discard():
                e.accept()
            else:
                e.ignore()

        # -------------------------------------------------------------- selection
        def select(self, pids):
            self.sel = [int(p) for p in dict.fromkeys(pids) if p > 0 and self.p.info(p) is not None]
            self.update_panel()
            self.tree.select_ids(self.sel)
            self.render_timer.start()
            if self.sync:
                self.render()

        def click_cell(self, cell, mods):
            if self.p is None:
                return
            pid = self.p.patch_at(*cell)
            if mods & Qt.ControlModifier:
                s = list(self.sel)
                if pid in s:
                    s.remove(pid)
                elif pid:
                    s.append(pid)
                self.select(s)
            else:
                self.select([pid] if pid else [])

        def press_cell(self, cell, mods):
            if self.p is None:
                return
            if self.tool_brush.isChecked():
                self._stroke = {'mask': np.zeros(self.p.shape, bool), 'last': cell}
                self._stamp(cell, cell)
            elif self.tool_cut.isChecked():
                self.canvas.cut = (cell, cell)
            elif self.tool_lasso.isChecked():
                self.canvas.lasso = [cell]
            elif self.tool_rect.isChecked() or mods & Qt.ShiftModifier:
                self._paint_box = self.tool_rect.isChecked()
                self.canvas.box = (cell, cell)
            else:
                self.click_cell(cell, mods)

        def move_cell(self, cell):
            if self._stroke is not None:
                self._stamp(self._stroke['last'], cell)
                self._stroke['last'] = cell
                self.render_timer.start()
                return
            if self.canvas.box is not None:
                self.canvas.box = (self.canvas.box[0], cell)
            elif self.canvas.cut is not None:
                self.canvas.cut = (self.canvas.cut[0], cell)
            elif self.canvas.lasso and cell != self.canvas.lasso[-1]:
                self.canvas.lasso.append(cell)
            else:
                return
            self.canvas.update()

        def release_cell(self, cell):
            if self._stroke is not None:
                self.move_cell(cell)
                mask, self._stroke = self._stroke['mask'], None
                self.apply_region(mask, '画笔')
            elif self.canvas.cut is not None:
                start, self.canvas.cut = self.canvas.cut[0], None
                self.cut_line(start, cell)
            elif self.canvas.lasso is not None:
                pts, self.canvas.lasso = self.canvas.lasso + [cell], None
                self.apply_region(self._poly_mask(pts), '套索')
            elif self.canvas.box is not None:
                region = self._rect_slice(self.canvas.box[0], cell)
                self.canvas.box = None
                if self._paint_box:
                    mask = np.zeros(self.p.shape, bool)
                    mask[region] = True
                    self.apply_region(mask, '矩形')
                else:
                    self.select([*self.sel, *np.unique(self.p.owner[region])])

        def brush_cells(self):
            return int(round(self.brush_r.value() / 100.0 / self.p.surf.resolution)) if self.p else 0

        def _stamp(self, a, b):
            m = self._stroke['mask']
            r = self.brush_cells()
            d = disc(r)
            n = max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1)
            h, w = m.shape
            for k in range(n + 1):
                c = int(round(a[0] + (b[0] - a[0]) * k / n))
                rr = int(round(a[1] + (b[1] - a[1]) * k / n))
                r0, r1, c0, c1 = rr - r, rr + r + 1, c - r, c + r + 1
                sr0, sc0 = max(0, -r0), max(0, -c0)
                R0, R1, C0, C1 = max(0, r0), min(h, r1), max(0, c0), min(w, c1)
                if R0 < R1 and C0 < C1:
                    m[R0:R1, C0:C1] |= d[sr0:sr0 + R1 - R0, sc0:sc0 + C1 - C0]

        def hover_cell(self, cell):
            if self.p is None:
                return
            c, r = cell
            h, w = self.p.shape
            if not (0 <= r < h and 0 <= c < w):
                return
            x, y = self.p.surf.centre(c, r)
            z = self.p.surf.z[r, c]
            pid = self.p.owner[r, c]
            self.status.showMessage('x %.2f  y %.2f  高度 %s  面片 %s' % (
                x, y, '%.3f m' % z if np.isfinite(z) else '无数据', pid or '-'))

        def zoom_to_sel(self):
            if not self.sel or self.p is None:
                return
            r, c = np.nonzero(np.isin(self.p.owner, self.sel))
            h = self.p.shape[0]
            cv = self.canvas
            span = max(c.max() - c.min() + 1, r.max() - r.min() + 1) + 20
            cv.scale = min(cv.width(), cv.height()) / span
            cx, cy = (c.min() + c.max() + 1) / 2, h - (r.min() + r.max() + 1) / 2
            cv.off = QPointF(cv.width() / 2 - cx * cv.scale, cv.height() / 2 - cy * cv.scale)
            cv.update()

        # -------------------------------------------------------------- annotation
        def _need_sel(self):
            if self.p is None or not self.sel:
                self.status.showMessage('先选中面片', 3000)
                return False
            return True

        def _changed(self, msg=None):
            self.dirty = True
            self.sel = [s for s in self.sel if self.p.info(s) is not None]
            self.update_title()
            self.update_panel()
            self.refresh_list()
            self.render()
            if msg:
                self.log(msg)

        def set_type(self, t):
            if self._need_sel() and self.p.set_ann(self.sel, type=int(t)):
                self._changed('%d 个面片 → %s' % (len(self.sel), TYPE_ZH.get(t, '未标注')))

        def _sel_passable(self):
            return all(self.p.a(s)['passable'] for s in self.sel) if self.p and self.sel else True

        def set_passable(self, on):
            if self._need_sel() and self.p.set_ann(self.sel, passable=bool(on)):
                self._changed('%d 个面片 → %s' % (len(self.sel), '允许通过' if on else '禁止通过'))
            else:
                self.update_panel()

        def set_dir(self, deg):
            kw = {'dir': None, 'rev': False} if deg is None else {'dir': float(deg) % 360.0}
            if self._need_sel() and self.p.set_ann(self.sel, **kw):
                self._changed('方向 → %s' % ('几何方向' if deg is None else '%.1f°' % deg))

        def _dir_edited(self, deg):
            if self._updating or self.p is None or not self.sel:
                return
            if all(abs((self.p.direction(s) - deg + 180.0) % 360.0 - 180.0) < 0.05 for s in self.sel):
                return
            self.set_dir(deg)

        def flip_dir(self):
            if self._need_sel() and self.p.flip_dir(self.sel):
                self._changed('方向反向')

        def merge_selected(self):
            if not self._need_sel() or len(self.sel) < 2:
                self.status.showMessage('合并需要至少选中两个面片（Ctrl+左键加选）', 3000)
                return
            n = len(self.sel)
            keep = self.p.merge(self.sel)
            self.sel = [keep]
            self._changed('%d 个面片合并为面片 %d' % (n, keep))

        def grow(self, cells):
            if not self._need_sel():
                return
            for s in self.sel:
                (self.p.grow if cells > 0 else self.p.shrink)(s, abs(cells))
            self._changed('面片%s %d 格' % ('扩大' if cells > 0 else '缩小', abs(cells)))

        def _rect_slice(self, start, end):
            height, width = self.p.shape
            left, right = sorted(np.clip((start[0], end[0]), 0, width - 1))
            bottom, top = sorted(np.clip((start[1], end[1]), 0, height - 1))
            return np.s_[bottom:top + 1, left:right + 1]

        def _poly_mask(self, pts):
            import cv2
            m = np.zeros(self.p.shape, np.uint8)
            if len(pts) >= 3:
                cv2.fillPoly(m, [np.array(pts, np.int32).reshape(-1, 1, 2)], 1)
            return m.astype(bool)

        def apply_region(self, mask, what):
            p = self.p
            mode = self.region_mode.currentIndex()
            if mode != 1 and self.clip_sel.isChecked() and self.sel:
                mask = mask & np.isin(p.owner, self.sel)
            mask = mask & np.isfinite(p.surf.z)
            n = int(mask.sum())
            if not n:
                self.status.showMessage('区域里没有可改的格子（勾选了只改选中面片时，要画在选中面片上）', 4000)
                self.render()
                return
            if mode == 2:
                p.restore_cells(mask)
                self._changed('%s：%d 格恢复自动分割' % (what, n))
                return
            if mode == 1:
                if not self.sel:
                    self.status.showMessage('并入模式要先选中一个面片', 4000)
                    self.render()
                    return
                pid = p.paint(mask, self.sel[0])
            else:
                pid = p.paint(mask, 0)
            self._changed('%s：%d 格%s面片 %d' % (what, n, '并入' if mode == 1 else '新建为', pid))
            self.select([pid])

        def cut_line(self, a, b):
            ids = self.p.split_line(a, b, self.sel or None)
            if not ids:
                self.status.showMessage('切割线要完整穿过面片（从一侧边界外拖到另一侧边界外）', 4000)
                self.canvas.update()
                return
            self.sel = ids
            self._changed('切割：得到 %d 块（%s），已全部选中，点击单块单独标注' % (len(ids), ', '.join(map(str, ids))))

        def select_level(self):
            if not self._need_sel():
                return
            ids = self.p.flood_level(self.sel)
            n = len(ids) - len(self.sel)
            self.select(ids)
            self.log('同高相连平面：新增 %d 块，共选中 %d 块，%.1f m²' % (
                n, len(ids), sum(self.p.info(i)['area'] for i in ids)))

        def reset_extent(self):
            if self._need_sel() and self.p.reset_extent(self.sel):
                self._changed('已恢复自动分割范围')

        def undo(self):
            if self.p is not None and self.p.undo():
                self._changed('撤销')

        def redo(self):
            if self.p is not None and self.p.redo():
                self._changed('重做')

        def resegment(self):
            if self.p is None:
                return
            p = self.read_params()
            if abs(p.resolution - self.p.surf.resolution) > 1e-9:
                self.log('分辨率改动需重新打开文件生效，本次按 %.3f m 分割' % self.p.surf.resolution)
                p.resolution = self.p.surf.resolution
            self.p.segment(p, log=self.log)
            self.sel = []
            self.overlay = None
            self._changed()

        # -------------------------------------------------------------- registration
        def ask_load_reg(self):
            f, _ = W.QFileDialog.getOpenFileName(self, 'terrain.json', '', 'json (*.json)')
            if f:
                import json
                reg = json.loads(Path(f).read_text())
                reg = reg.get('registration', reg)
                t, rpy = reg['t_world_map'], reg['rpy_deg']
                for s, v in zip(self.guess, list(t) + [rpy[2]]):
                    s.setValue(v)
                self.log('初值已读取：%s' % f)

        def run_icp(self):
            if self.p is None or self.p.frame != 'map':
                self.warn('当前工程不是点云工程')
                return
            if self.ref is None:
                self.warn('先载入参考场地')
                return
            g = [s.value() for s in self.guess]

            def done(reg):
                self._make_overlay()
                self._show_reg()
                self.dirty = True
                self.render()
            self.run(lambda log: self.p.register_to(self.ref, g[:3], g[3], log), done, 'ICP 配准')

        def _show_reg(self):
            r = self.p.reg
            ok = r['residual_p50'] < 0.06 and r.get('inlier_ratio_8cm', 1) > 0.6
            self.reg_label.setText('%s  t = (%.3f, %.3f, %.3f) m，偏航 %.2f°，残差 p50 %.1f cm / p90 %.1f cm，内点 %.0f %%'
                                   % ('配准成功' if ok else '⚠ 配准可疑', *r['t_world_map'], r['rpy_deg'][2],
                                      100 * r['residual_p50'], 100 * r['residual_p90'],
                                      100 * r.get('inlier_ratio_8cm', 0)))

        def _make_overlay(self):
            if self.ref is None or self.p is None or self.p.reg is None:
                self.overlay = None
                return
            self.overlay = patch_edges(self.p.field_lookup(self.ref))

        def transfer(self):
            if self.p is None or self.p.reg is None or self.ref is None:
                self.warn('先载入参考场地并完成配准')
                return
            if self.p.ann and W.QMessageBox.question(self, '采用模型面片', '点云上的面片和标注会被模型的替换（不能撤销），继续？') != W.QMessageBox.Yes:
                return
            n = self.p.adopt_from(self.ref, log=self.log)
            self.sel = []
            self._changed('已采用模型面片：%d 个标注' % n)

        # -------------------------------------------------------------- panel
        def update_panel(self):
            self._updating = True
            p, sel = self.p, self.sel
            for t, b in self.type_btns.items():
                b.setChecked(bool(sel) and all(p.a(s)['type'] == t for s in sel))
            self.passable.setChecked(self._sel_passable())
            if not sel:
                self.info.setText('未选中面片。<br>左键点击画面选择，Ctrl 加选，Shift 拖动框选。')
            elif len(sel) == 1:
                self.info.setText(patch_details(p, sel[0], TYPE_ZH))
            else:
                patches = [p.info(pid) for pid in sel]
                area = sum(patch['area'] for patch in patches)
                kinds = [patch['kind'] for patch in patches]
                self.info.setText('选中 %d 个面片，共 %.2f m²<br>平面 %d 个，斜面 %d 个<br>编号 %s' % (
                    len(sel), area, kinds.count('plane'), kinds.count('slope'),
                    ', '.join(map(str, sel[:20])) + (' …' if len(sel) > 20 else '')))
            if sel:
                self.dir_spin.setValue(p.direction(sel[0]))
            self._updating = False

        def refresh_list(self):
            if self.p is None:
                return
            project = self.p
            ids = project.ids()
            areas = project.stats['area']
            annotated = {pid for pid in ids if project.a(pid)['type'] != sp.UNSET}
            minimum = 0 if self.sync else self.list_min.value()
            only_unset = self.list_unset.isChecked()
            visible = (pid for pid in ids if areas[pid] >= minimum and (not only_unset or pid not in annotated))
            self.tree.populate(project, visible, TYPE_ZH, TYPE_COLOR)
            self.tree.select_ids(self.sel)
            area, done_area = areas[1:].sum(), sum(areas[pid] for pid in annotated)
            self.progress.setText('已标注 %d / %d 个面片，面积 %.1f / %.1f m²（%.0f %%）；未标注的按障碍导出' % (
                len(annotated), len(ids), done_area, area, 100 * done_area / max(area, 1e-9)))

        # -------------------------------------------------------------- drawing
        def compose(self):
            return compose_map(self.p, TYPE_COLOR, show_unset=self.show_unset.isChecked(),
                               opacity=self.alpha.value() / 100.0, edges=self.show_edges.isChecked(),
                               overlay=self.overlay if self.show_overlay.isChecked() else None,
                               selected=self.sel, stroke=self._stroke['mask'] if self._stroke else None)

        def render(self, refit=False):
            if self.p is None:
                return
            self.canvas.set_image(self.compose(), refit)

        def arrows(self):
            p = self.p
            if p is None or not self.show_arrows.isChecked():
                return []
            out = []
            show = set(pid for pid, a in p.ann.items() if a['type'] in DIRECTIONAL) | set(self.sel)
            S = p.stats
            for pid in show:
                if pid >= len(S['n']) or S['n'][pid] == 0 or (pid not in self.sel and S['area'][pid] < 0.15):
                    continue
                i = p.info(pid)
                if i is None or not (p.a(pid)['type'] in DIRECTIONAL or i['kind'] == 'slope' or i['dirs']):
                    continue
                for x, y, d in p.arrows(pid):
                    out.append((x, y, d, pid in self.sel))
            return out

        def view3d(self):
            if self.p is None:
                return
            if find_spec('open3d') is None:
                self.warn(_OPEN3D_REQUIRED)
                return
            p = self.p
            rr, cc = np.nonzero(np.isfinite(p.surf.z))
            res = p.surf.resolution
            xyz = np.stack([p.surf.origin[0] + (cc + 0.5) * res, p.surf.origin[1] + (rr + 0.5) * res,
                            p.surf.z[rr, cc]], 1).astype(np.float32)
            rgb = self.compose()[::-1][rr, cc].astype(np.float32) / 255.0
            f = Path(tempfile.gettempdir()) / ('surface_studio_%d.npz' % os.getpid())
            np.savez(f, xyz=xyz, rgb=rgb)
            subprocess.Popen([sys.executable, '-m', 'nav_terrain.surface_studio', '--view3d', str(f)])
            self.log('三维预览已在新窗口打开（与当前标注着色一致）')

def view3d_main(path):
    if find_spec('open3d') is None:
        raise SystemExit(_OPEN3D_REQUIRED)
    # GLFW can stall on Wayland; use XWayland when an X display is available.
    if sys.platform.startswith('linux') and os.environ.get('DISPLAY'):
        os.environ['WAYLAND_DISPLAY'] = ''
        os.environ['XDG_SESSION_TYPE'] = 'x11'
    import open3d as o3d
    d = np.load(path)
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(d['xyz'].astype(np.float64)))
    pc.colors = o3d.utility.Vector3dVector(d['rgb'].astype(np.float64))
    o3d.visualization.draw_geometries([pc], window_name='地形面片 三维预览', width=1280, height=800)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ['--view3d']:
        return view3d_main(argv[1])
    if W is None:
        sys.exit('surface_studio needs PyQt5')
    app = make_app(sys.argv[:1])
    s = Studio()
    s.resize(1500, 950)
    s.show()
    if argv:
        f = argv[0]
        if f.endswith('.surf.npz'):
            s.open_project(f)
        elif f.lower().endswith(('.pcd',)) or Path(f).is_dir():
            s.open_cloud(f)
        else:
            s.open_model(f)
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
