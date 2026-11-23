"""Format patch geometry and display selection in the inspector table."""
import math

from PyQt5.QtCore import QSignalBlocker, Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QAbstractItemView, QTreeWidget

from nav_terrain.studio_widgets import NumItem


def patch_details(p, pid, type_names):
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
        type_names.get(a['type'], '<b>未标注</b>'), '允许通过' if a['passable'] else '<b>禁止通过</b>', dtext)))
    return '<table>' + ''.join('<tr><td style="color:#888;padding-right:8px">%s</td><td>%s</td></tr>' % r
                               for r in rows) + '</table>'

def _patch_row(p, pid, type_names, colors):
    st, a = p.stats, p.a(pid)
    kind = '斜面' if st['incline'][pid] >= p.params.flat_deg else '平面'
    it = NumItem([str(pid), kind, '%.1f' % st['incline'][pid],
                  '%.0f' % st['azimuth'][pid] if kind == '斜面' else '',
                  '%.2f' % st['area'][pid], '%.3f' % st['z'][pid], type_names.get(a['type'], '—'),
                  '是' if a['passable'] else '否'])
    it.setData(0, Qt.UserRole, pid)
    if a['type'] in colors:
        it.setForeground(6, QColor(*colors[a['type']]).darker(130))
    return it


class PatchTable(QTreeWidget):
    def __init__(self):
        super().__init__(rootIsDecorated=False, sortingEnabled=True,
                         selectionMode=QAbstractItemView.ExtendedSelection)
        self.setHeaderLabels(['编号', '几何', '坡度°', '朝向°', '面积m²', '高度m', '类型', '通行'])

    def populate(self, project, visible_ids, type_names, colors):
        with QSignalBlocker(self):
            self.setSortingEnabled(False)
            self.clear()
            self.addTopLevelItems([_patch_row(project, pid, type_names, colors) for pid in visible_ids])
            self.setSortingEnabled(True)

    def select_ids(self, selected):
        selected = set(selected)
        with QSignalBlocker(self):
            for index in range(self.topLevelItemCount()):
                item = self.topLevelItem(index)
                item.setSelected(item.data(0, Qt.UserRole) in selected)

    def selected_ids(self):
        return [item.data(0, Qt.UserRole) for item in self.selectedItems()]
