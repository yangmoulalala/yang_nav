"""Map canvas, background task and sortable table items for the terrain editor."""
import math
import traceback

import numpy as np
from PyQt5 import QtWidgets as W
from PyQt5.QtCore import QPointF, QRect, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPolygonF


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


class NumItem(W.QTreeWidgetItem):
    def __lt__(self, other):
        c = self.treeWidget().sortColumn() if self.treeWidget() else 0
        try:
            return float(self.text(c)) < float(other.text(c))
        except ValueError:
            return self.text(c) < other.text(c)
