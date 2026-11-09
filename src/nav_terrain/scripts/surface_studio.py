#!/usr/bin/env python3
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
import math
import os
import subprocess
import sys
import tempfile
import traceback
from pathlib import Path

import numpy as np
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent))
import surface_project as sp  # noqa: E402
import surface_seg as ss  # noqa: E402
from terrain_map import (DIRECTIONAL, FLAT, OBSTACLE, SLOPE, STEP_HIGH, STEP_L1, STEP_L2, TUNNEL,  # noqa: E402
                         UNDULATION, load_keyframe_cloud, read_pcd)

TYPES = [  # (id, 中文名, 颜色, 快捷键)
    (FLAT, '平地', (76, 175, 80)),
    (SLOPE, '坡道', (255, 152, 0)),
    (STEP_L1, '一级台阶 200', (255, 235, 59)),
    (STEP_L2, '二级台阶 2×200', (156, 39, 176)),
    (STEP_HIGH, '高台阶 400', (0, 200, 230)),
    (UNDULATION, '起伏路段', (233, 30, 99)),
    (TUNNEL, '隧道 / 限高', (63, 81, 181)),
    (OBSTACLE, '障碍', (244, 67, 54)),
]
TYPE_ZH = {t: n for t, n, _ in TYPES}
TYPE_COLOR = {t: c for t, _, c in TYPES}

PARAM_UI = [  # (字段, 中文, 单位, 最小, 最大, 步长, 小数)
    ('resolution', '栅格分辨率', 'm', 0.01, 0.2, 0.005, 3),
    ('jump', '高差分界', 'm', 0.005, 0.3, 0.005, 3),
    ('flat_deg', '平面 / 斜面分界', '°', 0.5, 20, 0.5, 1),
    ('normal_deg', '斜面法向差', '°', 1, 30, 0.5, 1),
    ('smooth', '坡度平滑半径', 'm', 0.0, 0.5, 0.025, 3),
    ('min_area', '最小面片', 'm²', 0.0, 2.0, 0.01, 2),
    ('clutter_dz', '碎块并入高差', 'm', 0.0, 0.5, 0.01, 2),
    ('min_clear', '模型：地面最小净空', 'm', 0.05, 1.0, 0.01, 2),
    ('roof_height', '模型：顶棚分界高度', 'm', 0.1, 3.0, 0.05, 2),
    ('z_max', '模型：忽略高于', 'm', 0.2, 5.0, 0.1, 1),
    ('cloud_z_min', '点云：高度下限', 'm', -5.0, 5.0, 0.1, 2),
    ('cloud_z_max', '点云：高度上限', 'm', -5.0, 5.0, 0.1, 2),
    ('cloud_res', '点云：栅格分辨率', 'm', 0.02, 0.2, 0.005, 3),
    ('cloud_smooth', '点云：坡度平滑半径', 'm', 0.0, 1.0, 0.025, 3),
]

try:
    from PyQt5 import QtWidgets as W
    from PyQt5.QtCore import QPointF, QRect, Qt, QThread, QTimer, pyqtSignal
    from PyQt5.QtGui import QColor, QImage, QKeySequence, QPainter, QPen, QPolygonF
except ImportError:  # pragma: no cover - the core modules work without Qt
    W = None

_APP = None


def make_app(argv):
    global _APP
    _APP = W.QApplication.instance() or W.QApplication(argv)
    return _APP


def patch_palette(n):
    """Pale distinct colours for not annotated patches."""
    rng = np.random.default_rng(7)
    c = rng.integers(110, 230, (n + 1, 3)).astype(np.float32)
    c[0] = 0
    return c


def hillshade(z):
    zz = np.where(np.isfinite(z), z, np.nanmin(z) if np.isfinite(z).any() else 0.0)
    gy, gx = np.gradient(zz)
    shade = np.clip(0.75 + 2.5 * (gx - gy), 0.35, 1.15)
    lo, hi = np.nanpercentile(zz, [1, 99]) if zz.size else (0, 1)
    h = np.clip((zz - lo) / max(hi - lo, 1e-6), 0, 1)
    return np.clip((0.35 + 0.65 * h) * shade, 0, 1).astype(np.float32)


def disc(r):
    y, x = np.ogrid[-r:r + 1, -r:r + 1]
    return x * x + y * y <= r * r + 0.5


if W is not None:
    class Task(QThread):
        done = pyqtSignal(object)
        failed = pyqtSignal(str)
        log = pyqtSignal(str)

        def __init__(self, fn):
            super().__init__()
            self.fn = fn

        def run(self):
            try:
                self.done.emit(self.fn(self.log.emit))
            except Exception as e:  # noqa: BLE001
                self.failed.emit('%s\n%s' % (e, traceback.format_exc(limit=3)))

    class Canvas(W.QWidget):
        def __init__(self, studio):
            super().__init__()
            self.s = studio
            self.qimg = None
            self.scale, self.off = 1.0, QPointF(0, 0)
            self.pan_from = None
            self.box = None
            self.cut = None
            self.lasso = None
            self.hover = None
            self.setMouseTracking(True)
            self.setFocusPolicy(Qt.StrongFocus)
            self.setMinimumSize(500, 400)

        def set_image(self, rgb, refit=False):
            h, w, _ = rgb.shape
            self._buf = np.ascontiguousarray(rgb)
            self.qimg = QImage(self._buf.data, w, h, 3 * w, QImage.Format_RGB888)
            if refit:
                self.fit()
            self.update()

        def fit(self):
            if self.qimg is None:
                return
            w, h = self.qimg.width(), self.qimg.height()
            self.scale = max(1e-3, min(self.width() / w, self.height() / h) * 0.97)
            self.off = QPointF((self.width() - w * self.scale) / 2, (self.height() - h * self.scale) / 2)
            self.update()

        def to_cell(self, pos):
            H = self.qimg.height()
            c = int(math.floor((pos.x() - self.off.x()) / self.scale))
            ri = int(math.floor((pos.y() - self.off.y()) / self.scale))
            return c, H - 1 - ri

        def to_screen(self, col, row):
            H = self.qimg.height()
            return QPointF(self.off.x() + col * self.scale, self.off.y() + (H - row) * self.scale)

        def paintEvent(self, _):
            g = QPainter(self)
            g.fillRect(self.rect(), QColor(30, 32, 36))
            if self.qimg is None:
                g.setPen(QColor(170, 170, 170))
                g.drawText(self.rect(), Qt.AlignCenter, '打开场地模型（STL / STEP）或先验点云开始标注')
                return
            g.save()
            g.translate(self.off)
            g.scale(self.scale, self.scale)
            g.drawImage(0, 0, self.qimg)
            g.restore()
            g.setRenderHint(QPainter.Antialiasing)
            for x, y, ang, sel in self.s.arrows():
                self._arrow(g, x, y, ang, sel)
            g.setBrush(Qt.NoBrush)
            if self.box is not None:
                a, b = self.to_screen(*self.box[0]), self.to_screen(*self.box[1])
                g.setPen(QPen(QColor(255, 255, 0), 1, Qt.DashLine))
                g.drawRect(QRect(a.toPoint(), b.toPoint()).normalized())
            if self.lasso is not None and len(self.lasso) > 1:
                g.setPen(QPen(QColor(255, 255, 0), 1, Qt.DashLine))
                g.drawPolygon(QPolygonF([self.to_screen(c + 0.5, r + 0.5) for c, r in self.lasso]))
            if self.cut is not None:
                a = self.to_screen(self.cut[0][0] + 0.5, self.cut[0][1] + 0.5)
                b = self.to_screen(self.cut[1][0] + 0.5, self.cut[1][1] + 0.5)
                g.setPen(QPen(QColor(255, 60, 60), 2))
                g.drawLine(a, b)
            if self.s.tool_brush.isChecked() and self.hover is not None:
                r = self.s.brush_cells() + 0.5
                p = self.to_screen(self.hover[0] + 0.5, self.hover[1] + 0.5)
                g.setPen(QPen(QColor(255, 255, 255), 1))
                g.drawEllipse(p, r * self.scale, r * self.scale)

        def _arrow(self, g, x, y, ang, sel):
            s = self.s.p.surf
            p = self.to_screen((x - s.origin[0]) / s.resolution, (y - s.origin[1]) / s.resolution)
            L = max(14.0, min(40.0, 0.35 / s.resolution * self.scale))
            dx, dy = math.cos(math.radians(ang)), -math.sin(math.radians(ang))
            tip = QPointF(p.x() + dx * L, p.y() + dy * L)
            tail = QPointF(p.x() - dx * L * 0.6, p.y() - dy * L * 0.6)
            col = QColor(255, 255, 0) if sel else QColor(255, 255, 255)
            g.setPen(QPen(QColor(0, 0, 0), 4))
            g.drawLine(tail, tip)
            g.setPen(QPen(col, 2))
            g.drawLine(tail, tip)
            nx, ny = -dy, dx
            head = QPolygonF([tip, QPointF(tip.x() - dx * 9 + nx * 5, tip.y() - dy * 9 + ny * 5),
                              QPointF(tip.x() - dx * 9 - nx * 5, tip.y() - dy * 9 - ny * 5)])
            g.setBrush(col)
            g.drawPolygon(head)

        def wheelEvent(self, e):
            if self.qimg is None:
                return
            f = 1.25 if e.angleDelta().y() > 0 else 0.8
            p = QPointF(e.pos())
            self.off = p - (p - self.off) * f
            self.scale *= f
            self.update()

        def mousePressEvent(self, e):
            if self.qimg is None:
                return
            if e.button() in (Qt.MiddleButton, Qt.RightButton):
                self.pan_from = (QPointF(e.pos()), QPointF(self.off))
            elif e.button() == Qt.LeftButton:
                self.s.press_cell(self.to_cell(e.pos()), e.modifiers())

        def mouseMoveEvent(self, e):
            if self.qimg is None:
                return
            if self.pan_from is not None:
                self.off = self.pan_from[1] + (QPointF(e.pos()) - self.pan_from[0])
                self.update()
                return
            cell = self.to_cell(e.pos())
            self.hover = cell
            if e.buttons() & Qt.LeftButton:
                self.s.move_cell(cell)
            self.s.hover_cell(cell)
            if self.s.tool_brush.isChecked():
                self.update()

        def mouseReleaseEvent(self, e):
            if self.pan_from is not None and e.button() in (Qt.MiddleButton, Qt.RightButton):
                self.pan_from = None
            elif e.button() == Qt.LeftButton and self.qimg is not None:
                self.s.release_cell(self.to_cell(e.pos()))

        def mouseDoubleClickEvent(self, e):
            if e.button() == Qt.MiddleButton:
                self.fit()

        def resizeEvent(self, _):
            if self.qimg is not None and getattr(self, '_fitted', False) is False:
                self._fitted = True
                self.fit()

    class Studio(W.QMainWindow):
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
            self._box_from = None
            self._cut_from = None
            self._rect_from = None
            self._lasso = None
            self._updating = False
            self.setWindowTitle('地形面片标注')
            self._build()
            self.render_timer = QTimer(self, singleShot=True, interval=30, timeout=self.render)

        # -------------------------------------------------------------- ui
        def _build(self):
            tb = self.addToolBar('文件')
            tb.setMovable(False)
            for text, fn, key in (('打开模型', self.ask_open_model, 'Ctrl+O'), ('打开点云', self.ask_open_cloud, None),
                                  ('打开工程', self.ask_open_project, None), ('保存工程', self.save_project, 'Ctrl+S'),
                                  ('导出 Nav2', self.ask_export, 'Ctrl+E'), (None, None, None),
                                  ('撤销', self.undo, 'Ctrl+Z'), ('重做', self.redo, 'Ctrl+Y'), (None, None, None),
                                  ('适应窗口', lambda: self.canvas.fit(), 'F'), ('三维预览', self.view3d, None)):
                if text is None:
                    tb.addSeparator()
                    continue
                a = tb.addAction(text)
                a.triggered.connect(lambda _=False, fn=fn: fn())
                if key:
                    a.setShortcut(QKeySequence(key))
            self.canvas = Canvas(self)
            side = W.QTabWidget()
            side.setMinimumWidth(400)
            scroll = W.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(W.QFrame.NoFrame)
            scroll.setWidget(self._tab_annotate())
            side.addTab(scroll, '标注')
            side.addTab(self._tab_list(), '面片列表')
            side.addTab(self._tab_seg(), '分割参数')
            side.addTab(self._tab_reg(), '点云配准')
            split = W.QSplitter()
            split.addWidget(self.canvas)
            split.addWidget(side)
            split.setStretchFactor(0, 1)
            self.logbox = W.QPlainTextEdit(readOnly=True)
            self.logbox.setMaximumBlockCount(2000)
            v = W.QSplitter(Qt.Vertical)
            v.addWidget(split)
            v.addWidget(self.logbox)
            v.setSizes([700, 120])
            self.setCentralWidget(v)
            self.status = self.statusBar()
            self._keys()

        def _keys(self):
            def sc(key, fn):
                a = W.QShortcut(QKeySequence(key), self)
                a.activated.connect(fn)
            for i, (t, _, _) in enumerate(TYPES):
                sc(str(i + 1), lambda t=t: self.set_type(t))
            sc('0', lambda: self.set_type(sp.UNSET))
            sc('X', lambda: self.set_passable(not self._sel_passable()))
            sc('R', self.flip_dir)
            sc('M', self.merge_selected)
            sc('V', lambda: self.tool_select.setChecked(True))
            sc('B', lambda: self.tool_brush.setChecked(True))
            sc('U', lambda: self.tool_rect.setChecked(True))
            sc('L', lambda: self.tool_lasso.setChecked(True))
            sc('K', lambda: self.tool_cut.setChecked(True))
            sc('G', self.select_level)
            sc('+', lambda: self.grow(1))
            sc('=', lambda: self.grow(1))
            sc('-', lambda: self.grow(-1))
            sc('Esc', lambda: (self.tool_select.setChecked(True), self.p is not None and self.select([])))
            sc('Ctrl+Shift+Z', self.redo)

        def _tab_annotate(self):
            w = W.QWidget()
            v = W.QVBoxLayout(w)
            self.info = W.QLabel('未选中面片')
            self.info.setWordWrap(True)
            self.info.setTextFormat(Qt.RichText)
            self.info.setAlignment(Qt.AlignTop)
            self.info.setMinimumHeight(220)
            box = W.QGroupBox('几何信息（自动识别）')
            QV = W.QVBoxLayout(box)
            QV.addWidget(self.info)
            v.addWidget(box)

            box = W.QGroupBox('人工标注')
            g = W.QGridLayout(box)
            self.type_btns = {}
            grid = W.QGridLayout()
            b = W.QPushButton('0  未标注')
            b.clicked.connect(lambda: self.set_type(sp.UNSET))
            grid.addWidget(b, 0, 0)
            self.type_btns[sp.UNSET] = b
            for i, (t, name, col) in enumerate(TYPES):
                b = W.QPushButton('%d  %s' % (i + 1, name))
                b.setStyleSheet('QPushButton{border-left:8px solid rgb(%d,%d,%d);text-align:left;padding:4px}' % col)
                b.clicked.connect(lambda _=False, t=t: self.set_type(t))
                grid.addWidget(b, (i + 1) // 2, (i + 1) % 2)
                self.type_btns[t] = b
            for b in self.type_btns.values():
                b.setCheckable(True)
            g.addLayout(grid, 0, 0, 1, 3)
            self.passable = W.QCheckBox('允许通过   X')
            self.passable.setChecked(True)
            self.passable.clicked.connect(lambda on: self.set_passable(on))
            g.addWidget(self.passable, 1, 0, 1, 3)
            g.addWidget(W.QLabel('方向 (°)'), 2, 0)
            self.dir_spin = W.QDoubleSpinBox(decimals=1, minimum=0, maximum=359.9, singleStep=5, wrapping=True)
            self.dir_spin.setToolTip('坡道：上坡方向；台阶：由低到高；隧道 / 起伏：通行方向。0° = +x，逆时针。\n'
                                     '多向坡（如堡垒六边形坡）保持“用几何方向”即按各坡面分别给方向，R 整体反向；'
                                     '输入角度则整块统一成一个方向')
            self.dir_spin.editingFinished.connect(lambda: self.set_dir(self.dir_spin.value()))
            g.addWidget(self.dir_spin, 2, 1)
            row = W.QHBoxLayout()
            b = W.QPushButton('反向  R')
            b.clicked.connect(self.flip_dir)
            row.addWidget(b)
            b = W.QPushButton('用几何方向')
            b.setToolTip('恢复为由面片几何算出的方向')
            b.clicked.connect(lambda: self.set_dir(None))
            row.addWidget(b)
            g.addLayout(row, 2, 2)
            v.addWidget(box)

            box = W.QGroupBox('大小修正')
            g = W.QGridLayout(box)
            b = W.QPushButton('合并选中  M')
            b.clicked.connect(self.merge_selected)
            g.addWidget(b, 0, 0)
            b = W.QPushButton('恢复自动分割范围')
            b.clicked.connect(self.reset_extent)
            g.addWidget(b, 0, 1)
            self.step_cells = W.QSpinBox(minimum=1, maximum=40, value=1, suffix=' 格')
            b1, b2 = W.QPushButton('扩大  +'), W.QPushButton('缩小  -')
            b1.clicked.connect(lambda: self.grow(self.step_cells.value()))
            b2.clicked.connect(lambda: self.grow(-self.step_cells.value()))
            g.addWidget(b1, 1, 0)
            g.addWidget(b2, 1, 1)
            g.addWidget(self.step_cells, 1, 2)
            b = W.QPushButton('选同高相连平面  G')
            b.setToolTip('从选中面片出发，把没有台阶隔开、尚未标注或标注相同的平面全部加入选择；遇到台阶和斜面停止')
            b.clicked.connect(self.select_level)
            g.addWidget(b, 2, 0, 1, 2)
            v.addWidget(box)

            box = W.QGroupBox('编辑工具（改面片范围）')
            g = W.QGridLayout(box)
            self.tool_select = W.QRadioButton('选择  V')
            self.tool_brush = W.QRadioButton('画笔  B')
            self.tool_rect = W.QRadioButton('矩形  U')
            self.tool_lasso = W.QRadioButton('套索  L')
            self.tool_cut = W.QRadioButton('切割线  K')
            self.tool_select.setChecked(True)
            for i, t in enumerate((self.tool_select, self.tool_brush, self.tool_rect, self.tool_lasso, self.tool_cut)):
                t.toggled.connect(lambda _: self.canvas.update())
                g.addWidget(t, i // 3, i % 3)
            g.addWidget(W.QLabel('画出的区域'), 2, 0)
            self.region_mode = W.QComboBox()
            self.region_mode.addItems(['新建面片（继承原标注）', '并入选中面片', '恢复自动分割'])
            g.addWidget(self.region_mode, 2, 1, 1, 2)
            self.clip_sel = W.QCheckBox('只改选中面片里的格子（新建 / 恢复时）')
            self.clip_sel.setChecked(True)
            g.addWidget(self.clip_sel, 3, 0, 1, 3)
            g.addWidget(W.QLabel('画笔半径'), 4, 0)
            self.brush_r = W.QSpinBox(minimum=0, maximum=200, value=10, suffix=' cm')
            g.addWidget(self.brush_r, 4, 1)
            hint = W.QLabel('一块面里只有一部分不能通过：先选中它，用矩形 / 套索 / 画笔圈出那部分 → 新建面片，'
                            '再按 X 设为禁止通过。切割线：拖一条线完整穿过面片，切成两块。')
            hint.setWordWrap(True)
            hint.setStyleSheet('color:#888')
            g.addWidget(hint, 5, 0, 1, 3)
            v.addWidget(box)

            box = W.QGroupBox('显示')
            g = W.QGridLayout(box)
            self.show_unset = W.QCheckBox('未标注面片着色')
            self.show_unset.setChecked(True)
            self.show_edges = W.QCheckBox('面片边界')
            self.show_edges.setChecked(True)
            self.show_arrows = W.QCheckBox('方向箭头')
            self.show_arrows.setChecked(True)
            self.show_overlay = W.QCheckBox('场地模型边界（配准后）')
            self.show_overlay.setChecked(True)
            for i, c in enumerate((self.show_unset, self.show_edges, self.show_arrows, self.show_overlay)):
                c.toggled.connect(lambda _: self.render())
                g.addWidget(c, i // 2, i % 2)
            self.alpha = W.QSlider(Qt.Horizontal, minimum=10, maximum=100, value=60)
            self.alpha.valueChanged.connect(lambda _: self.render_timer.start())
            g.addWidget(W.QLabel('标注不透明度'), 2, 0)
            g.addWidget(self.alpha, 2, 1)
            v.addWidget(box)
            hint = W.QLabel('左键选中，Ctrl+左键加选，Shift+拖动框选；中键或右键拖动平移，滚轮缩放。')
            hint.setWordWrap(True)
            hint.setStyleSheet('color:#888')
            v.addWidget(hint)
            v.addStretch(1)
            return w

        def _tab_list(self):
            w = W.QWidget()
            v = W.QVBoxLayout(w)
            row = W.QHBoxLayout()
            row.addWidget(W.QLabel('只列出面积 ≥'))
            self.list_min = W.QDoubleSpinBox(decimals=2, minimum=0, maximum=50, value=0.10, singleStep=0.05,
                                             suffix=' m²')
            self.list_min.valueChanged.connect(lambda _: self.refresh_list())
            row.addWidget(self.list_min)
            self.list_unset = W.QCheckBox('只看未标注')
            self.list_unset.toggled.connect(lambda _: self.refresh_list())
            row.addWidget(self.list_unset)
            row.addStretch(1)
            v.addLayout(row)
            self.tree = W.QTreeWidget()
            self.tree.setHeaderLabels(['编号', '几何', '坡度°', '朝向°', '面积m²', '高度m', '类型', '通行'])
            self.tree.setRootIsDecorated(False)
            self.tree.setSortingEnabled(True)
            self.tree.setSelectionMode(W.QAbstractItemView.ExtendedSelection)
            self.tree.itemSelectionChanged.connect(self._tree_selected)
            self.tree.itemDoubleClicked.connect(lambda *_: self.zoom_to_sel())
            v.addWidget(self.tree)
            self.progress = W.QLabel('')
            v.addWidget(self.progress)
            return w

        def _tab_seg(self):
            w = W.QWidget()
            v = W.QVBoxLayout(w)
            form = W.QFormLayout()
            self.pw = {}
            for key, zh, unit, lo, hi, st, dec in PARAM_UI:
                s = W.QDoubleSpinBox(decimals=dec, minimum=lo, maximum=hi, singleStep=st, suffix=' ' + unit)
                form.addRow(zh, s)
                self.pw[key] = s
            v.addLayout(form)
            self.step_off = [W.QDoubleSpinBox(decimals=1, minimum=-1e5, maximum=1e5) for _ in range(3)]
            for s, val in zip(self.step_off, (0.0, 1624.3, -1941.0)):
                s.setValue(val)
            row = W.QHBoxLayout()
            for s in self.step_off:
                row.addWidget(s)
            form.addRow('STEP 原点偏移 (mm)', row)
            b = W.QPushButton('重新分割（已有标注按重叠保留）')
            b.clicked.connect(self.resegment)
            v.addWidget(b)
            note = W.QLabel('分辨率和高度范围在重新打开文件后生效。高差超过“高差分界”的相邻格子分到不同面片；'
                            '坡度低于“平面 / 斜面分界”算平面；相邻斜面法向差小于“斜面法向差”合为一块；'
                            '小于“最小面片”的碎块并入相邻面片。')
            note.setWordWrap(True)
            note.setStyleSheet('color:#888')
            v.addWidget(note)
            v.addStretch(1)
            self.set_params(self.params)
            return w

        def _tab_reg(self):
            w = W.QWidget()
            v = W.QVBoxLayout(w)
            note = W.QLabel('点云工程可以配准到官方场地模型：先打开模型并标注，再打开点云（模型自动作为参考），'
                            '配准后点云改用模型的面片和标注（点云自己的分割被替换），再人工修正。')
            note.setWordWrap(True)
            v.addWidget(note)
            self.ref_label = W.QLabel('参考场地：无')
            v.addWidget(self.ref_label)
            b = W.QPushButton('载入参考场地（模型或工程）')
            b.clicked.connect(self.ask_open_ref)
            v.addWidget(b)
            form = W.QFormLayout()
            self.guess = [W.QDoubleSpinBox(decimals=3, minimum=-100, maximum=100, singleStep=0.05) for _ in range(3)]
            self.guess.append(W.QDoubleSpinBox(decimals=2, minimum=-360, maximum=360, singleStep=1))
            for s, n, val in zip(self.guess, ('x (m)', 'y (m)', 'z (m)', '偏航 (°)'), (-9.45, -0.16, 0.5, 0.0)):
                s.setValue(val)
                form.addRow('初值 ' + n, s)
            v.addLayout(form)
            b = W.QPushButton('从 terrain.json 读取配准')
            b.clicked.connect(self.ask_load_reg)
            v.addWidget(b)
            b = W.QPushButton('运行 ICP 配准')
            b.clicked.connect(self.run_icp)
            v.addWidget(b)
            self.reg_label = W.QLabel('未配准')
            self.reg_label.setWordWrap(True)
            v.addWidget(self.reg_label)
            b = W.QPushButton('采用模型面片和标注')
            b.clicked.connect(self.transfer)
            v.addWidget(b)
            v.addStretch(1)
            return w

        # -------------------------------------------------------------- tasks
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
            import mesh_io
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
                pts = pts[np.isfinite(pts).all(1)]
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
            self.overlay = None
            if p.reg is not None and self.ref is not None:
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
            if path is None:
                path = self.path
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
            self._sync_tree()
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
                self._cut_from = cell
                self.canvas.cut = (cell, cell)
            elif self.tool_rect.isChecked():
                self._rect_from = cell
                self.canvas.box = (cell, cell)
            elif self.tool_lasso.isChecked():
                self._lasso = [cell]
                self.canvas.lasso = self._lasso
            elif mods & Qt.ShiftModifier:
                self._box_from = cell
                self.canvas.box = (cell, cell)
            else:
                self.click_cell(cell, mods)

        def move_cell(self, cell):
            if self._stroke is not None:
                self._stamp(self._stroke['last'], cell)
                self._stroke['last'] = cell
                self.render_timer.start()
            elif self._rect_from is not None:
                self.canvas.box = (self._rect_from, cell)
                self.canvas.update()
            elif self._lasso is not None:
                if cell != self._lasso[-1]:
                    self._lasso.append(cell)
                self.canvas.update()
            elif self._cut_from is not None:
                self.canvas.cut = (self._cut_from, cell)
                self.canvas.update()
            elif self._box_from is not None:
                self.canvas.box = (self._box_from, cell)
                self.canvas.update()

        def release_cell(self, cell):
            if self._stroke is not None:
                self.move_cell(cell)
                mask, self._stroke = self._stroke['mask'], None
                self.apply_region(mask, '画笔')
            elif self._cut_from is not None:
                a, self._cut_from, self.canvas.cut = self._cut_from, None, None
                self.cut_line(a, cell)
            elif self._rect_from is not None:
                a, self._rect_from, self.canvas.box = self._rect_from, None, None
                self.apply_region(self._rect_mask(a, cell), '矩形')
            elif self._lasso is not None:
                pts, self._lasso, self.canvas.lasso = self._lasso + [cell], None, None
                self.apply_region(self._poly_mask(pts), '套索')
            elif self._box_from is not None:
                (c0, r0), (c1, r1) = self._box_from, cell
                self._box_from, self.canvas.box = None, None
                h, w = self.p.shape
                r0, r1 = sorted((max(0, min(h - 1, r0)), max(0, min(h - 1, r1))))
                c0, c1 = sorted((max(0, min(w - 1, c0)), max(0, min(w - 1, c1))))
                ids = np.unique(self.p.owner[r0:r1 + 1, c0:c1 + 1])
                self.select(list(self.sel) + [int(i) for i in ids if i > 0])

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

        def _rect_mask(self, a, b):
            h, w = self.p.shape
            m = np.zeros((h, w), bool)
            c0, c1 = sorted((max(0, min(w - 1, a[0])), max(0, min(w - 1, b[0]))))
            r0, r1 = sorted((max(0, min(h - 1, a[1])), max(0, min(h - 1, b[1]))))
            m[r0:r1 + 1, c0:c1 + 1] = True
            return m

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
            look = self.p.field_lookup(self.ref)
            e = np.zeros(look.shape, bool)
            e[:, 1:] |= look[:, 1:] != look[:, :-1]
            e[1:] |= look[1:] != look[:-1]
            self.overlay = e

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
                self.info.setText(self._info_html(sel[0]))
                self.dir_spin.setValue(p.direction(sel[0]))
            else:
                area = sum(p.info(s)['area'] for s in sel)
                kinds = [p.info(s)['kind'] for s in sel]
                self.info.setText('选中 %d 个面片，共 %.2f m²<br>平面 %d 个，斜面 %d 个<br>编号 %s' % (
                    len(sel), area, kinds.count('plane'), kinds.count('slope'),
                    ', '.join(map(str, sel[:20])) + (' …' if len(sel) > 20 else '')))
                self.dir_spin.setValue(p.direction(sel[0]))
            self._updating = False

        def _info_html(self, pid):
            p = self.p
            i = p.info(pid)
            a = p.a(pid)
            geo = ('多向斜面（%d 个坡面）' % len(i['groups'])) if i['multi'] else ('斜面' if i['kind'] == 'slope' else '平面')
            rows = [('编号', str(pid)), ('几何', geo),
                    ('坡度', ('各坡面平均 %.1f°' if i['multi'] else '%.1f°') % i['incline'])]
            if len(i['dirs']) > 1:
                rows.append(('上坡朝向', '<b>多向</b>：%s' % '、'.join('%.0f°（坡 %.1f°）' % (g['az'], g['incline']) for g in i['groups'])))
            elif i['kind'] == 'slope':
                rows.append(('上坡朝向', '%.0f°' % i['azimuth']))
            rows += [('面积', '%.2f m²' % i['area']),
                     ('尺寸', '%.2f × %.2f m（%s）' % (i['length'], i['width'], '沿坡 × 横向' if i['kind'] == 'slope' else '长 × 宽')),
                     ('长轴', '%.0f°' % i['axis']),
                     ('高度', '平均 %.3f m（%.3f ~ %.3f）' % (i['z'], i['zmin'], i['zmax'])),
                     ('平整度', '拟合残差 %.1f mm' % (1000 * i['rms'])),
                     ('中心', '(%.2f, %.2f)' % (i['cx'], i['cy']))]
            if math.isfinite(i['clearance']) and p.frame == 'field':
                rows.append(('头顶净空', '%.3f m' % i['clearance']))
            if i['step_w'] > 1e-6:
                rows.append(('边缘高差方向', '%.0f°（低 → 高）' % i['step_dir']))
            nb = p.neighbours(pid)[:6]
            if nb:
                rows.append(('相邻面片', '<br>'.join('%d：%+.0f mm，边长 %.2f m' % (n, 1000 * dz, c * p.surf.resolution)
                                                  for n, c, dz in nb)))
            dtext = ('按各坡面' if p.is_local(pid) else '%.0f°' % p.direction(pid)) \
                + ('（反向）' if a.get('rev') else '') + ('' if a['dir'] is None else '（手动）')
            rows.append(('标注', '%s，%s，方向 %s' % (
                TYPE_ZH.get(a['type'], '<b>未标注</b>'), '允许通过' if a['passable'] else '<b>禁止通过</b>', dtext)))
            return '<table>' + ''.join('<tr><td style="color:#888;padding-right:8px">%s</td><td>%s</td></tr>' % r
                                       for r in rows) + '</table>'

        def refresh_list(self):
            if self.p is None:
                return
            p = self.p
            st = p.stats
            amin = self.list_min.value()
            only_unset = self.list_unset.isChecked()
            self._updating = True
            self.tree.setSortingEnabled(False)
            self.tree.clear()
            items = []
            for pid in p.ids():
                if st['area'][pid] < amin and not self.sync:
                    continue
                a = p.a(pid)
                if only_unset and a['type'] != sp.UNSET:
                    continue
                kind = '斜面' if st['incline'][pid] >= p.params.flat_deg else '平面'
                it = NumItem([str(pid), kind, '%.1f' % st['incline'][pid],
                              '%.0f' % st['azimuth'][pid] if kind == '斜面' else '',
                              '%.2f' % st['area'][pid], '%.3f' % st['z'][pid], TYPE_ZH.get(a['type'], '—'),
                              '是' if a['passable'] else '否'])
                it.setData(0, Qt.UserRole, pid)
                if a['type'] in TYPE_COLOR:
                    it.setForeground(6, QColor(*TYPE_COLOR[a['type']]).darker(130))
                items.append(it)
            self.tree.addTopLevelItems(items)
            self.tree.setSortingEnabled(True)
            done = sum(1 for pid in p.ids() if p.a(pid)['type'] != sp.UNSET)
            area = st['area'][1:].sum()
            done_area = sum(st['area'][pid] for pid in p.ann if pid < len(st['area']) and p.a(pid)['type'] != sp.UNSET)
            self.progress.setText('已标注 %d / %d 个面片，面积 %.1f / %.1f m²（%.0f %%）；未标注的按障碍导出' % (
                done, len(p.ids()), done_area, area, 100 * done_area / max(area, 1e-9)))
            self._updating = False
            self._sync_tree()

        def _sync_tree(self):
            self._updating = True
            sel = set(self.sel)
            self.tree.clearSelection()
            for i in range(self.tree.topLevelItemCount()):
                it = self.tree.topLevelItem(i)
                if it.data(0, Qt.UserRole) in sel:
                    it.setSelected(True)
            self._updating = False

        def _tree_selected(self):
            if self._updating or self.p is None:
                return
            ids = [it.data(0, Qt.UserRole) for it in self.tree.selectedItems()]
            self.sel = [int(i) for i in ids]
            self.update_panel()
            self.render()

        # -------------------------------------------------------------- drawing
        def compose(self):
            p = self.p
            own = p.owner
            valid = np.isfinite(p.surf.z)
            base = hillshade(p.surf.z)[..., None] * 255.0
            img = base * np.array([0.85, 0.87, 0.9], np.float32)
            L = int(own.max()) + 1
            col = np.zeros((L, 3), np.float32)
            alpha = np.zeros(L, np.float32)
            if self.show_unset.isChecked():
                col[:] = patch_palette(L - 1)[:L]
                alpha[:] = 0.28
            hatch_ids = np.zeros(L, bool)
            for pid, a in p.ann.items():
                if pid < L:
                    if a['type'] in TYPE_COLOR:
                        col[pid] = TYPE_COLOR[a['type']]
                        alpha[pid] = self.alpha.value() / 100.0
                    hatch_ids[pid] = not a['passable']
            alpha[0] = 0
            A = alpha[own][..., None]
            img = img * (1 - A) + col[own] * A
            h, w = own.shape
            if hatch_ids.any():
                rr, cc = np.ogrid[0:h, 0:w]
                hm = hatch_ids[own] & (((rr + cc) % 8) < 3)
                img[hm] = img[hm] * 0.25
            if self.show_edges.isChecked():
                e = np.zeros(own.shape, bool)
                e[:, 1:] |= own[:, 1:] != own[:, :-1]
                e[1:] |= own[1:] != own[:-1]
                img[e] = img[e] * 0.35
            if self.overlay is not None and self.show_overlay.isChecked() and self.overlay.shape == own.shape:
                img[self.overlay] = (0, 230, 255)
            if self.sel:
                sm = np.isin(own, self.sel)
                img[sm] = img[sm] * 0.6 + np.array([255, 255, 0], np.float32) * 0.4
                edge = sm & ~ndimage.binary_erosion(sm)
                img[edge] = (255, 255, 0)
            if self._stroke is not None:
                m = self._stroke['mask']
                img[m] = img[m] * 0.4 + np.array([255, 255, 255], np.float32) * 0.6
            img[~valid] = (25, 25, 28)
            return np.clip(img, 0, 255).astype(np.uint8)[::-1]

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
            p = self.p
            rr, cc = np.nonzero(np.isfinite(p.surf.z))
            res = p.surf.resolution
            xyz = np.stack([p.surf.origin[0] + (cc + 0.5) * res, p.surf.origin[1] + (rr + 0.5) * res,
                            p.surf.z[rr, cc]], 1).astype(np.float32)
            rgb = self.compose()[::-1][rr, cc].astype(np.float32) / 255.0
            f = Path(tempfile.gettempdir()) / ('surface_studio_%d.npz' % os.getpid())
            np.savez(f, xyz=xyz, rgb=rgb)
            subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--view3d', str(f)])
            self.log('三维预览已在新窗口打开（与当前标注着色一致）')

    class NumItem(W.QTreeWidgetItem):
        def __lt__(self, other):
            c = self.treeWidget().sortColumn() if self.treeWidget() else 0
            try:
                return float(self.text(c)) < float(other.text(c))
            except ValueError:
                return self.text(c) < other.text(c)


def view3d_main(path):
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
