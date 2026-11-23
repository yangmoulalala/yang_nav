"""Build the terrain editor's panels, actions and shortcuts."""
from PyQt5 import QtWidgets as W
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence

from nav_terrain import surface_project as sp
from nav_terrain.studio_inspector import PatchTable
from nav_terrain.studio_widgets import Canvas
from nav_terrain.terrain_map import FLAT, OBSTACLE, SLOPE, STEP_HIGH, STEP_L1, STEP_L2, TUNNEL, UNDULATION

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

class EditorLayout:
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
        scroll = W.QScrollArea(widgetResizable=True, frameShape=W.QFrame.NoFrame)
        scroll.setWidget(self._tab_annotate())
        side.addTab(scroll, '标注')
        side.addTab(self._tab_list(), '面片列表')
        side.addTab(self._tab_seg(), '分割参数')
        side.addTab(self._tab_reg(), '点云配准')
        split = W.QSplitter()
        split.addWidget(self.canvas)
        split.addWidget(side)
        split.setStretchFactor(0, 1)
        self.logbox = W.QPlainTextEdit(readOnly=True, maximumBlockCount=2000)
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
        self.info = W.QLabel('未选中面片', wordWrap=True, textFormat=Qt.RichText, alignment=Qt.AlignTop,
                             minimumHeight=220)
        box = W.QGroupBox('几何信息（自动识别）')
        QV = W.QVBoxLayout(box)
        QV.addWidget(self.info)
        v.addWidget(box)

        box = W.QGroupBox('人工标注')
        g = W.QGridLayout(box)
        self.type_btns = {}
        grid = W.QGridLayout()
        for index, (label, name, color) in enumerate([(sp.UNSET, '未标注', None), *TYPES]):
            button = W.QPushButton('%d  %s' % (index, name), checkable=True,
                                   clicked=lambda checked, label=label: self.set_type(label))
            if color is not None:
                button.setStyleSheet('QPushButton{border-left:8px solid rgb(%d,%d,%d);text-align:left;padding:4px}' % color)
            grid.addWidget(button, index // 2, index % 2)
            self.type_btns[label] = button
        g.addLayout(grid, 0, 0, 1, 3)
        self.passable = W.QCheckBox('允许通过   X', checked=True, clicked=self.set_passable)
        g.addWidget(self.passable, 1, 0, 1, 3)
        g.addWidget(W.QLabel('方向 (°)'), 2, 0)
        self.dir_spin = W.QDoubleSpinBox(decimals=1, minimum=0, maximum=359.9, singleStep=5, wrapping=True)
        self.dir_spin.setToolTip('坡道：上坡方向；台阶：由低到高；隧道 / 起伏：通行方向。0° = +x，逆时针。\n'
                                 '多向坡（如堡垒六边形坡）保持“用几何方向”即按各坡面分别给方向，R 整体反向；'
                                 '输入角度则整块统一成一个方向')
        # arrows / wheel / Enter all apply at once (editingFinished alone needs Enter or a focus change,
        # and the canvas never takes focus); typing applies on Enter
        self.dir_spin.setKeyboardTracking(False)
        self.dir_spin.valueChanged.connect(self._dir_edited)
        g.addWidget(self.dir_spin, 2, 1)
        row = W.QHBoxLayout()
        row.addWidget(W.QPushButton('反向  R', clicked=self.flip_dir))
        row.addWidget(W.QPushButton('用几何方向', clicked=lambda: self.set_dir(None), toolTip='恢复为由面片几何算出的方向'))
        g.addLayout(row, 2, 2)
        v.addWidget(box)

        box = W.QGroupBox('大小修正')
        g = W.QGridLayout(box)
        g.addWidget(W.QPushButton('合并选中  M', clicked=self.merge_selected), 0, 0)
        g.addWidget(W.QPushButton('恢复自动分割范围', clicked=self.reset_extent), 0, 1)
        self.step_cells = W.QSpinBox(minimum=1, maximum=40, value=1, suffix=' 格')
        g.addWidget(W.QPushButton('扩大  +', clicked=lambda: self.grow(self.step_cells.value())), 1, 0)
        g.addWidget(W.QPushButton('缩小  -', clicked=lambda: self.grow(-self.step_cells.value())), 1, 1)
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
        self.clip_sel = W.QCheckBox('只改选中面片里的格子（新建 / 恢复时）', checked=True)
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
        self.show_unset = W.QCheckBox('未标注面片着色', checked=True)
        self.show_edges = W.QCheckBox('面片边界', checked=True)
        self.show_arrows = W.QCheckBox('方向箭头', checked=True)
        self.show_overlay = W.QCheckBox('场地模型边界（配准后）', checked=True)
        for i, c in enumerate((self.show_unset, self.show_edges, self.show_arrows, self.show_overlay)):
            c.toggled.connect(lambda _: self.render())
            g.addWidget(c, i // 2, i % 2)
        self.alpha = W.QSlider(Qt.Horizontal, minimum=10, maximum=100, value=60)
        self.alpha.valueChanged.connect(lambda _: self.render_timer.start())
        g.addWidget(W.QLabel('标注不透明度'), 2, 0)
        g.addWidget(self.alpha, 2, 1)
        v.addWidget(box)
        hint = W.QLabel('左键选中，Ctrl+左键加选，Shift+拖动框选；中键或右键拖动平移，滚轮缩放。', wordWrap=True, styleSheet='color:#888')
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
        self.tree = PatchTable()
        self.tree.itemSelectionChanged.connect(lambda: self.select(self.tree.selected_ids()))
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
        v.addWidget(W.QPushButton('重新分割（已有标注按重叠保留）', clicked=self.resegment))
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
        v.addWidget(W.QPushButton('载入参考场地（模型或工程）', clicked=self.ask_open_ref))
        form = W.QFormLayout()
        self.guess = [W.QDoubleSpinBox(decimals=3, minimum=-100, maximum=100, singleStep=0.05) for _ in range(3)]
        self.guess.append(W.QDoubleSpinBox(decimals=2, minimum=-360, maximum=360, singleStep=1))
        for s, n, val in zip(self.guess, ('x (m)', 'y (m)', 'z (m)', '偏航 (°)'), (-9.45, -0.16, 0.5, 0.0)):
            s.setValue(val)
            form.addRow('初值 ' + n, s)
        v.addLayout(form)
        v.addWidget(W.QPushButton('从 terrain.json 读取配准', clicked=self.ask_load_reg))
        v.addWidget(W.QPushButton('运行 ICP 配准', clicked=self.run_icp))
        self.reg_label = W.QLabel('未配准', wordWrap=True)
        v.addWidget(self.reg_label)
        v.addWidget(W.QPushButton('采用模型面片和标注', clicked=self.transfer))
        v.addStretch(1)
        return w
