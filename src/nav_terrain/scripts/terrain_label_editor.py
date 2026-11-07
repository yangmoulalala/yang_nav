#!/usr/bin/env python3
"""terrain_label_editor: paint terrain labels + directions on the top-down height map (HWSentryNav26 flow).

    terrain_label_editor.py maps/sim/terrain.msgpack

Adapted from HWSentryNav26 utils/py/map/terrain_label_editor.py: same two-channel model and tools,
plus map-frame origin, the embedded height map backdrop, and the TUNNEL label.

Tools   pixel brush | rectangle | line. A line sets the direction of directional labels to the
        LEFT perpendicular of the stroke (draw a ramp's bottom edge left-to-right looking uphill;
        draw across a tunnel mouth so the left normal points through the tunnel).
Keys    0-7 label   p/r/l tool   [ ] brush size   +/- zoom   Ctrl+Z undo   Ctrl+S save   e/a toggle
        elevation / arrows. Right-drag pans.
Labels  0 flat  1 obstacle  2 slope  3 step_l1  4 step_l2  5 fly_slope  6 step_high  7 tunnel
"""
import argparse
import math
import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
from PIL import Image, ImageTk

sys.path.insert(0, str(Path(__file__).resolve().parent))
from terrain_map import DIRECTIONAL, LABELS, TerrainMap, encode_angle, load_keyframe_cloud, read_pcd  # noqa
from terrain_export import COLORS  # noqa: E402

ELEV_OPACITY = 0.45
BG = (30, 30, 30)
HISTORY = 30


def line_direction(c0, r0, c1, r1):
    """Left normal of a stroke in grid coordinates (rows go up in y) -> direction code."""
    dx, dy = c1 - c0, r1 - r0
    if dx == 0 and dy == 0:
        return 0
    return encode_angle(math.atan2(dx, -dy))


def bresenham(x0, y0, x1, y1):
    pts = []
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx + dy
    while True:
        pts.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return pts
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def brush_mask(shape, pts, width):
    m = np.zeros(shape, bool)
    off = -(width // 2)
    h, w = shape
    for c, r in pts:
        m[max(0, r + off):max(0, min(h, r + off + width)), max(0, c + off):max(0, min(w, c + off + width))] = True
    return m


def elevation_gray(elev):
    valid = np.isfinite(elev)
    out = np.zeros(elev.shape + (3,), np.uint8)
    if valid.any():
        lo, hi = np.percentile(elev[valid], [2, 98])
        g = (np.clip((np.where(valid, elev, lo) - lo) / max(hi - lo, 1e-6), 0, 1) * 255).astype(np.uint8)
        out[:] = g[..., None]
        out[~valid] = 0
    return out, valid


class Editor:
    def __init__(self, root, path=None):
        self.root = root
        self.m = None
        self.path = None
        self.history = []
        self.scale = 2.0
        self.label = 0
        self.dir_code = tk.IntVar(value=64)
        self.tool = tk.StringVar(value='pixel')
        self.brush = tk.IntVar(value=3)
        self.show_elev = tk.BooleanVar(value=True)
        self.show_arrows = tk.BooleanVar(value=True)
        self.drag_start = None
        self.preview = []
        self._img = None
        self._build()
        if path:
            self.open(path)

    # ---------------------------------------------------------------- ui
    def _build(self):
        self.root.title('terrain_label_editor')
        self.root.geometry('1600x1000')
        bar = tk.Frame(self.root, bd=1, relief=tk.RAISED)
        bar.pack(side=tk.TOP, fill=tk.X)
        for text, cmd in (('Open', self.open_dialog), ('Save', self.save), ('Save as', self.save_as),
                          ('Undo', self.undo), ('Z+', lambda: self.zoom(1.5)), ('Z-', lambda: self.zoom(1 / 1.5)),
                          ('Load cloud', self.load_cloud)):
            tk.Button(bar, text=text, command=cmd).pack(side=tk.LEFT, padx=2, pady=2)
        for text, val in (('Brush', 'pixel'), ('Rect', 'rect'), ('Line', 'line')):
            tk.Radiobutton(bar, text=text, variable=self.tool, value=val).pack(side=tk.LEFT)
        tk.Label(bar, text='size').pack(side=tk.LEFT, padx=(8, 0))
        tk.Spinbox(bar, from_=1, to=40, width=3, textvariable=self.brush).pack(side=tk.LEFT)
        tk.Label(bar, text='label').pack(side=tk.LEFT, padx=(8, 0))
        self.label_box = ttk.Combobox(bar, state='readonly', width=10,
                                      values=['%d %s' % (k, v) for k, v in LABELS.items()])
        self.label_box.current(0)
        self.label_box.bind('<<ComboboxSelected>>', lambda _e: self.set_label(self.label_box.current()))
        self.label_box.pack(side=tk.LEFT)
        self.swatch = tk.Canvas(bar, width=20, height=20, highlightthickness=1)
        self.swatch.pack(side=tk.LEFT, padx=4)
        tk.Label(bar, text='direction').pack(side=tk.LEFT, padx=(8, 0))
        self.dir_scale = tk.Scale(bar, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.dir_code,
                                  showvalue=False, length=140)
        self.dir_scale.pack(side=tk.LEFT)
        self.dir_text = tk.Label(bar, width=10, anchor='w')
        self.dir_text.pack(side=tk.LEFT)
        self.dir_code.trace_add('write', lambda *_: self._dir_label())
        tk.Checkbutton(bar, text='elevation', variable=self.show_elev, command=self.redraw).pack(side=tk.LEFT)
        tk.Checkbutton(bar, text='arrows', variable=self.show_arrows, command=self.redraw).pack(side=tk.LEFT)

        frame = tk.Frame(self.root)
        frame.pack(fill=tk.BOTH, expand=True)
        vs, hs = tk.Scrollbar(frame, orient=tk.VERTICAL), tk.Scrollbar(frame, orient=tk.HORIZONTAL)
        self.cv = tk.Canvas(frame, bg='#1a1a1a', xscrollcommand=hs.set, yscrollcommand=vs.set)
        vs.config(command=self.cv.yview)
        hs.config(command=self.cv.xview)
        vs.pack(side=tk.RIGHT, fill=tk.Y)
        hs.pack(side=tk.BOTTOM, fill=tk.X)
        self.cv.pack(fill=tk.BOTH, expand=True)
        self.status = tk.Label(self.root, anchor='w', bd=1, relief=tk.SUNKEN)
        self.status.pack(side=tk.BOTTOM, fill=tk.X)

        self.cv.bind('<ButtonPress-1>', self.on_down)
        self.cv.bind('<B1-Motion>', self.on_drag)
        self.cv.bind('<ButtonRelease-1>', self.on_up)
        self.cv.bind('<Motion>', self.on_move)
        self.cv.bind('<ButtonPress-3>', lambda e: self.cv.scan_mark(e.x, e.y))
        self.cv.bind('<B3-Motion>', lambda e: self.cv.scan_dragto(e.x, e.y, gain=1))
        self.root.bind('<Control-z>', lambda _e: self.undo())
        self.root.bind('<Control-s>', lambda _e: self.save())
        self.root.bind('<Key>', self.on_key)
        self.set_label(0)

    def _dir_label(self):
        v = self.dir_code.get()
        self.dir_text.config(text='%3.0f deg (%d)' % (v / 255 * 360, v))

    def set_label(self, k):
        self.label = k
        self.label_box.current(k)
        self.swatch.config(bg='#%02x%02x%02x' % COLORS[k])
        self.dir_scale.config(state=tk.NORMAL if k in DIRECTIONAL else tk.DISABLED)
        self._dir_label()

    def on_key(self, e):
        if e.state & 0x4:
            return
        ch = e.char
        if ch.isdigit() and int(ch) in LABELS:
            self.set_label(int(ch))
        elif ch in 'prl' and ch:
            self.tool.set({'p': 'pixel', 'r': 'rect', 'l': 'line'}[ch])
        elif ch == '[':
            self.brush.set(max(1, self.brush.get() - 1))
        elif ch == ']':
            self.brush.set(min(40, self.brush.get() + 1))
        elif ch in '+=':
            self.zoom(1.5)
        elif ch == '-':
            self.zoom(1 / 1.5)
        elif ch == 'e':
            self.show_elev.set(not self.show_elev.get())
            self.redraw()
        elif ch == 'a':
            self.show_arrows.set(not self.show_arrows.get())
            self.redraw()

    # ---------------------------------------------------------------- files
    def open_dialog(self):
        p = filedialog.askopenfilename(filetypes=[('terrain msgpack', '*.msgpack'), ('all', '*')])
        if p:
            self.open(p)

    def open(self, path):
        try:
            self.m = TerrainMap.load(path)
        except Exception as exc:   # noqa: BLE001 - show any load error to the user
            messagebox.showerror('open failed', str(exc))
            return
        self.path = Path(path)
        self.history.clear()
        self.root.title('terrain_label_editor - %s' % self.path)
        self.redraw()

    def save(self):
        if self.m is None:
            return
        if self.path is None:
            return self.save_as()
        try:
            self.m.save(self.path)
        except Exception as exc:   # noqa: BLE001
            messagebox.showerror('save failed', str(exc))
            return
        self.status.config(text='saved %s' % self.path)

    def save_as(self):
        if self.m is None:
            return
        p = filedialog.asksaveasfilename(defaultextension='.msgpack', filetypes=[('terrain msgpack', '*.msgpack')])
        if p:
            self.path = Path(p)
            self.save()

    def load_cloud(self):
        """Replace the elevation backdrop with the mean z per cell of a PCD (or a map bundle's keyframes.txt)."""
        if self.m is None:
            return
        p = filedialog.askopenfilename(filetypes=[('cloud / keyframes', '*.pcd keyframes.txt'), ('all', '*')])
        if not p:
            return
        try:
            pts = load_keyframe_cloud(Path(p).parent) if p.endswith('keyframes.txt') else read_pcd(p)
        except Exception as exc:   # noqa: BLE001
            messagebox.showerror('load failed', str(exc))
            return
        m = self.m
        c = np.floor((pts[:, 0] - m.origin[0]) / m.resolution).astype(np.int64)
        r = np.floor((pts[:, 1] - m.origin[1]) / m.resolution).astype(np.int64)
        ok = (c >= 0) & (c < m.width) & (r >= 0) & (r < m.height)
        idx = r[ok] * m.width + c[ok]
        cnt = np.bincount(idx, minlength=m.width * m.height)
        s = np.bincount(idx, weights=pts[ok, 2], minlength=m.width * m.height)
        with np.errstate(invalid='ignore', divide='ignore'):
            m.elevation = np.where(cnt > 0, s / cnt, np.nan).reshape(m.height, m.width).astype(np.float32)
        self.status.config(text='elevation from %s (%d points in map)' % (p, int(ok.sum())))
        self.redraw()

    # ---------------------------------------------------------------- render
    def base_rgb(self):
        m = self.m
        rgb = np.zeros((m.height, m.width, 3), np.uint8)
        rgb[:] = BG
        for k, col in COLORS.items():
            if k:
                rgb[m.terrain == k] = col
        if self.show_elev.get() and m.elevation is not None:
            g, valid = elevation_gray(m.elevation)
            flat = m.terrain == 0
            rgb[flat & valid] = g[flat & valid]
            lab = ~flat & valid
            rgb[lab] = ((1 - ELEV_OPACITY) * rgb[lab] + ELEV_OPACITY * g[lab]).astype(np.uint8)
        return rgb

    def redraw(self):
        self.cv.delete('all')
        self.preview = []
        if self.m is None:
            return
        img = Image.fromarray(self.base_rgb()[::-1])
        sw, sh = max(1, int(self.m.width * self.scale)), max(1, int(self.m.height * self.scale))
        self._img = ImageTk.PhotoImage(img.resize((sw, sh), Image.NEAREST))
        self.cv.create_image(0, 0, image=self._img, anchor=tk.NW)
        self.cv.config(scrollregion=(0, 0, sw, sh))
        if self.show_arrows.get():
            self.draw_arrows()

    def draw_arrows(self):
        m = self.m
        step = max(3, int(round(16 / self.scale)))
        rows, cols = np.mgrid[step // 2:m.height:step, step // 2:m.width:step]
        lab = m.terrain[rows, cols]
        sel = np.isin(lab, list(DIRECTIONAL))
        for r, c in zip(rows[sel][:20000], cols[sel][:20000]):
            a = m.direction[r, c] / 255.0 * 2 * math.pi
            x, y = self.to_canvas(c, r)
            ex, ey = x + 11 * math.cos(a), y - 11 * math.sin(a)
            self.cv.create_line(x, y, ex, ey, fill='white', arrow=tk.LAST, arrowshape=(5, 6, 2))

    def to_canvas(self, c, r):
        return (c + 0.5) * self.scale, (self.m.height - 1 - r + 0.5) * self.scale

    def to_cell(self, e, clamp=False):
        if self.m is None:
            return None
        c = math.floor(self.cv.canvasx(e.x) / self.scale)
        r = self.m.height - 1 - math.floor(self.cv.canvasy(e.y) / self.scale)
        if clamp:
            return min(self.m.width - 1, max(0, c)), min(self.m.height - 1, max(0, r))
        if 0 <= c < self.m.width and 0 <= r < self.m.height:
            return c, r
        return None

    def zoom(self, f):
        self.scale = min(30.0, max(0.25, self.scale * f))
        self.redraw()

    # ---------------------------------------------------------------- editing
    def push(self):
        self.history.append((self.m.terrain.copy(), self.m.direction.copy()))
        del self.history[:-HISTORY]

    def undo(self):
        if self.history:
            self.m.terrain, self.m.direction = self.history.pop()
            self.redraw()

    def apply(self, mask, code=None):
        self.m.terrain[mask] = self.label
        if self.label in DIRECTIONAL:
            self.m.direction[mask] = self.dir_code.get() if code is None else code
        else:
            self.m.direction[mask] = 0

    def on_down(self, e):
        cell = self.to_cell(e)
        if cell is None:
            return
        self.push()
        self.drag_start = cell
        self.last = cell
        if self.tool.get() == 'pixel':
            self.paint([cell])

    def paint(self, pts):
        mask = brush_mask(self.m.terrain.shape, pts, self.brush.get())
        self.apply(mask)
        col = '#%02x%02x%02x' % COLORS[self.label]
        rr, cc = np.nonzero(mask)
        for r, c in zip(rr, cc):
            x, y = c * self.scale, (self.m.height - 1 - r) * self.scale
            self.cv.create_rectangle(x, y, x + self.scale, y + self.scale, fill=col, outline='')

    def on_drag(self, e):
        if self.drag_start is None:
            return
        cell = self.to_cell(e, clamp=True)
        tool = self.tool.get()
        if tool == 'pixel':
            self.paint(bresenham(*self.last, *cell))
            self.last = cell
            return
        for i in self.preview:
            self.cv.delete(i)
        x0, y0 = self.to_canvas(*self.drag_start)
        x1, y1 = self.to_canvas(*cell)
        col = '#%02x%02x%02x' % COLORS[self.label]
        if tool == 'rect':
            self.preview = [self.cv.create_rectangle(x0, y0, x1, y1, outline=col, width=2, dash=(4, 2))]
        else:
            self.preview = [self.cv.create_line(x0, y0, x1, y1, fill=col,
                                                width=max(1, self.brush.get() * self.scale))]
            if self.label in DIRECTIONAL:
                a = line_direction(*self.drag_start, *cell) / 255 * 2 * math.pi
                mx, my = (x0 + x1) / 2, (y0 + y1) / 2
                self.preview.append(self.cv.create_line(mx, my, mx + 30 * math.cos(a), my - 30 * math.sin(a),
                                                        fill='white', width=2, arrow=tk.LAST))

    def on_up(self, e):
        if self.drag_start is None:
            return
        cell = self.to_cell(e, clamp=True)
        tool = self.tool.get()
        (c0, r0), (c1, r1) = self.drag_start, cell
        if tool == 'rect':
            mask = np.zeros(self.m.terrain.shape, bool)
            mask[min(r0, r1):max(r0, r1) + 1, min(c0, c1):max(c0, c1) + 1] = True
            self.apply(mask)
        elif tool == 'line':
            code = line_direction(c0, r0, c1, r1)
            self.apply(brush_mask(self.m.terrain.shape, bresenham(c0, r0, c1, r1), self.brush.get()), code)
            if self.label in DIRECTIONAL:
                self.dir_code.set(code)
        self.drag_start = None
        self.redraw()

    def on_move(self, e):
        cell = self.to_cell(e)
        if cell is None:
            return
        c, r = cell
        m = self.m
        x, y = m.cell_center(c, r)
        lab = int(m.terrain[r, c])
        z = m.elevation[r, c] if m.elevation is not None else float('nan')
        d = ' dir %.0f deg' % (m.direction[r, c] / 255 * 360) if lab in DIRECTIONAL else ''
        self.status.config(text='x %.2f  y %.2f  (col %d row %d)  z %s  %s%s'
                           % (x, y, c, r, '%.3f' % z if np.isfinite(z) else '-', LABELS[lab], d))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map', nargs='?', help='terrain msgpack (create one with cloud_to_height_map.py)')
    a = ap.parse_args(argv)
    root = tk.Tk()
    Editor(root, a.map)
    root.mainloop()


if __name__ == '__main__':
    main()
