from ctypes import CDLL, get_errno
from dataclasses import dataclass, replace
import json
import os
from pathlib import Path
import secrets
from tempfile import TemporaryDirectory
from threading import Lock
from zipfile import BadZipFile

import cv2
import numpy as np

from nav_terrain.mesh_io import STEP_SUFFIXES, SUFFIXES, load_mesh
from nav_terrain.point_cloud import load_keyframe_cloud, read_pcd
from nav_terrain.studio_render import compose_map, patch_edges
from nav_terrain.surface_project import Project
from nav_terrain.surface_seg import SegParams
from nav_terrain.terrain_export import COLORS, ExportConfig

from .settings import WORKSPACE, flag, map_output, number, path as resolve_path, text, workspace_path


TYPES = {-1: '未标注', 0: '平地', 2: '坡道', 3: '一级台阶', 4: '二级台阶',
         6: '高台阶', 8: '起伏路段', 7: '隧道 / 限高', 1: '障碍'}
SEGMENT_FIELDS = {'jump': (0.005, 0.3), 'flat_deg': (0.5, 20.0),
                  'normal_deg': (1.0, 30.0), 'min_area': (0.0, 2.0)}
LOAD_FIELDS = {'resolution': (0.01, 0.2), 'cloud_res': (0.02, 0.2),
               'cloud_z_min': (-20.0, 20.0), 'cloud_z_max': (-20.0, 20.0),
               'z_max': (0.2, 5.0), 'min_clear': (0.05, 1.0), 'roof_height': (0.1, 3.0)}
RMUL_STEP_STEM = 'rmul2027_3v3_v1.0.0'
AT_FDCWD = -100
RENAME_EXCHANGE = 2


def publish_export(staging, destination):
    if not destination.exists():
        staging.rename(destination)
        return
    rename = CDLL(None, use_errno=True).renameat2
    if rename(AT_FDCWD, os.fsencode(staging), AT_FDCWD, os.fsencode(destination), RENAME_EXCHANGE):
        error = get_errno()
        raise OSError(error, os.strerror(error), destination)


def terrain_sources():
    paths = list((WORKSPACE / 'src/platform/sentry_gazebo/models').glob('*/meshes/field_collision.stl'))
    for directory, subdirs, files in os.walk(WORKSPACE / 'maps'):
        subdirs[:] = [name for name in subdirs if name != 'clouds' and not name.startswith('.')]
        paths.extend(Path(directory) / name for name in files
                     if Path(name).suffix.lower() in (*SUFFIXES, '.pcd') and name != 'ground_z.npz')
    return sorted(str(item.relative_to(WORKSPACE)) for item in paths)


def matches_prior(frame, source, prior):
    return bool(frame == 'map' and prior is not None and isinstance(source, str) and source
                and resolve_path(source) in (prior, prior / 'map.pcd'))


def field_references(sources, prior=None):
    projects, models = [], []
    for source in sources:
        if source.lower().endswith('.surf.npz'):
            try:
                path = workspace_path(source, '参考工程')
                with np.load(path, allow_pickle=False) as archive:
                    meta = json.loads(archive['meta'].tobytes())
                frame = meta.get('frame')
                if frame in ('field', 'map'):
                    projects.append(dict(path=source, frame=frame, annotated=len(meta.get('ann', {})),
                                         matches_prior=matches_prior(frame, meta.get('source'), prior)))
            except (OSError, ValueError, KeyError, AttributeError, TypeError, BadZipFile):
                continue
        elif Path(source).suffix.lower() in SUFFIXES:
            models.append(source)
    return dict(projects=projects, models=models)


def resolve_surface(value):
    label = '地形来源'
    raw = Path(text(value, label))
    source = workspace_path(value, label)
    if source.exists():
        return source
    if raw.suffix.lower() in STEP_SUFFIXES:
        candidates = [Path(name) for name in terrain_sources() if Path(name).suffix.lower() in STEP_SUFFIXES
                      and Path(name).stem.casefold() == raw.stem.casefold()]
        relative = source.relative_to(WORKSPACE)
        matches = candidates if raw.parent == Path('.') else [item for item in candidates if item.parent == relative.parent]
        exact = [item for item in matches if item.name.casefold() == raw.name.casefold()]
        matches = exact or matches
        if len(matches) == 1:
            match = workspace_path(str(matches[0]), label)
            if match.is_file():
                return match
        if candidates:
            raise ValueError(f'找不到唯一的 STEP 路径，请选择完整路径：{", ".join(str(item) for item in candidates)}')
    raise ValueError(f'找不到地形文件或目录：{source.relative_to(WORKSPACE)}。请选择服务器工作区中的路径。')


def checked_options(raw, limits):
    if not isinstance(raw, dict) or raw.keys() - limits.keys():
        raise ValueError('分割参数无效')
    return {key: number(value, *limits[key], key) for key, value in raw.items()}


def coordinates(raw, limits, label):
    if not isinstance(raw, list) or len(raw) != len(limits):
        raise ValueError(f'{label}坐标无效')
    return [number(value, *bounds, label) for value, bounds in zip(raw, limits)]


def load_surface(source, body):
    if source.name.lower().endswith('.surf.npz') and source.is_file():
        return Project.load(source)
    params = SegParams(**checked_options(body.get('params', {}), SEGMENT_FIELDS | LOAD_FIELDS))
    if params.cloud_z_min >= params.cloud_z_max:
        raise ValueError('点云高度下限须小于上限')
    if source.is_dir():
        if (source / 'keyframes.txt').is_file() and (source / 'clouds').is_dir():
            return Project.from_cloud(load_keyframe_cloud(source), params, str(source))
        raise ValueError('建图目录需要包含 keyframes.txt 和 clouds/，请选择具体的场地模型文件或建图目录。')
    suffix = source.suffix.lower()
    if source.is_file() and suffix == '.pcd':
        return Project.from_cloud(read_pcd(source), params, str(source))
    if source.is_file() and suffix in SUFFIXES:
        rmul = suffix in STEP_SUFFIXES and source.stem.casefold() == RMUL_STEP_STEM
        default_offset = [6000, 0, 4000] if rmul else [0, 1624.3, -1941]
        offset = coordinates(body.get('offset', default_offset), [(-100000, 100000)] * 3, 'STEP 原点')
        try:
            mesh = load_mesh(source, offset)
        except ImportError as exc:
            raise ValueError('STEP 解析器不可用。请配置 ~/.venvs/nav-terrain 或 YANG_NAV_WEB_PYTHON 后重启 WebUI。') from exc
        if rmul:
            mesh = -mesh[..., [0, 2, 1]]
        return Project.from_mesh(mesh, params, str(source))
    raise ValueError(f'不支持的地形文件：{source.name}。支持 STEP、STP、STL、NPZ、PCD 和 .surf.npz。')


def render_preview(project, overlay=None):
    surface = compose_map(project, COLORS, show_unset=True, opacity=0.72, edges=True,
                          overlay=overlay, selected=[], stroke=None)
    success, image = cv2.imencode('.png', np.ascontiguousarray(surface[:, :, ::-1]))
    if not success:
        raise ValueError('地形预览生成失败')
    return image.tobytes()


def render_match_reference(folder, grid, position):
    resolution, origin, size = grid
    if (type(resolution) not in (int, float) or not np.isfinite(resolution) or resolution <= 0 or
            not isinstance(origin, list) or len(origin) != 2 or
            not all(type(value) in (int, float) and np.isfinite(value) for value in origin)):
        raise ValueError('地形缺少有效的地图栅格坐标')
    image = cv2.imread(str(folder / 'terrain.png'))
    if image is None or size != [image.shape[1], image.shape[0]]:
        raise ValueError('缺少或不匹配的地形预览图，请重新导出')
    col, row = np.floor((np.asarray(position) - origin) / resolution)
    if not (0 <= col < size[0] and 0 <= row < size[1]):
        raise ValueError('匹配参考点在地图范围外')
    center = (int(col), size[1] - 1 - int(row))
    radius = max(4, min(size) // 45)
    for color, thickness in [((25, 25, 25), 4), ((255, 255, 255), 2)]:
        cv2.circle(image, center, radius, color, thickness)
        cv2.drawMarker(image, center, color, cv2.MARKER_CROSS, radius * 3, thickness)
    ok, png = cv2.imencode('.png', image)
    if not ok:
        raise ValueError('无法生成匹配参考图')
    return png.tobytes()


def preview_archive(value, prior=None):
    source = workspace_path(value, '标注工程路径')
    if not source.name.lower().endswith('.surf.npz') or not source.is_file():
        raise ValueError('请选择工作区中已保存的 .surf.npz 标注工程')
    try:
        project = Project.load(source)
        ids = project.ids()
        annotated_types = [kind for pid in ids if (kind := project.a(pid)['type']) >= 0]
        legend = [(TYPES.get(kind, f'类型 {kind}'), COLORS[kind])
                  for kind in sorted(set(annotated_types)) if kind in COLORS]
        metadata = dict(frame=project.frame, resolution=project.surf.resolution, origin=project.surf.origin,
                        matches_prior=matches_prior(project.frame, project.source, prior),
                        patches=len(ids), annotated=len(annotated_types), legend=legend)
        return render_preview(project), metadata
    except (OSError, ValueError, KeyError, TypeError, IndexError, BadZipFile, cv2.error) as exc:
        raise ValueError('无法预览该工程：文件无效、已损坏或无法读取') from exc


@dataclass
class TerrainDocument:
    project: Project
    source: Path
    archive: Path | None = None
    dirty: bool = False
    registered_to: Project | None = None

    def summary(self):
        return dict(source=str(self.source.relative_to(WORKSPACE)), dirty=self.dirty,
                    patches=len(self.project.ids()), annotated=len(self.project.ann))


class TerrainEditor:
    def __init__(self):
        self.lock = Lock()
        self.action_lock = Lock()
        self.documents = {}
        self.map_reference: TerrainDocument | None = None
        self.active = 'map'
        self.revision = secrets.token_hex(8)

    @property
    def document(self):
        return self.documents.get(self.active)

    @property
    def reference(self):
        return self.map_reference or self.documents.get('field')

    @property
    def reference_path(self):
        reference = self.reference
        return str((reference.archive or reference.source).relative_to(WORKSPACE)) if reference else ''

    def aligned(self, document):
        return (document.project.frame == 'map' and document.project.reg is not None and
                self.reference is not None and document.registered_to is self.reference.project)

    def require_revision(self, revision):
        if revision != self.revision:
            raise ValueError('地形工程已在其他页面更新，请重新读取当前工程')

    def snapshot(self):
        with self.lock:
            document = self.document
            state = dict(revision=self.revision, project=None, types=TYPES, active=self.active,
                         documents={name: item.summary() for name, item in self.documents.items()})
            if document is None:
                return state
            project = document.project
            stats = project.stats
            patches = [dict(id=pid, area=float(stats['area'][pid]),
                            x=float(stats['cx'][pid]), y=float(stats['cy'][pid]), **project.a(pid))
                       for pid in project.ids()]
            archive = str(document.archive.relative_to(WORKSPACE)) if document.archive else ''
            source = str(document.source.relative_to(WORKSPACE))
            state['project'] = dict(source=source, directory=document.source.is_dir(), archive=archive, dirty=document.dirty,
                                    frame=project.frame, width=project.shape[1], height=project.shape[0],
                                    resolution=project.surf.resolution, origin=project.surf.origin,
                                    patches=patches, params=project.params.to_dict(), registration=project.reg,
                                    landmarks=project.landmarks.copy(),
                                    reference=self.reference_path, can_adopt=self.aligned(document),
                                    reference_annotated=len(self.reference.project.ann) if self.reference else 0,
                                    registration_seed=self.default_seed(document))
            return state

    def raster(self, route, revision):
        with self.lock:
            self.require_revision(revision)
            document = self.document
            if document is None:
                raise ValueError('请先打开地形工程')
            project = document.project
            if route == 'owners':
                return project.owner[::-1].astype('<u4', copy=False).tobytes(), 'application/octet-stream'
            overlay = patch_edges(project.field_lookup(self.reference.project)) if self.aligned(document) else None
            return render_preview(project, overlay), 'image/png'

    def details(self, revision, pid):
        with self.lock:
            self.require_revision(revision)
            document = self.document
            info = document.project.info(pid) if document else None
            if info is None:
                raise ValueError('面片不存在')
            return dict(info=info, arrows=document.project.arrows(pid))

    def action(self, operation, body):
        if not self.action_lock.acquire(blocking=False):
            raise ValueError('地形工程正在处理，请稍后再试')
        try:
            with self.lock:
                self.require_revision(body.get('revision'))
                response = self.perform(operation, body)
                if operation not in ('connected', 'seed'):
                    self.revision = secrets.token_hex(8)
                return response
        finally:
            self.action_lock.release()

    def perform(self, operation, body):
        if operation in ('open', 'reference'):
            source = resolve_surface(body.get('source'))
            target = body.get('target') if operation == 'open' else None
            if target not in (None, 'map', 'field'):
                raise ValueError('地形工作区无效')
            if target:
                self.check_discard(target, body)
            archive = source if source.name.lower().endswith('.surf.npz') and source.is_file() else None
            if operation == 'reference' and not archive and (not source.is_file() or source.suffix.lower() not in SUFFIXES):
                raise ValueError('配准参考需要已保存的 .surf.npz 标注工程或场地模型')
            loaded = load_surface(source, body)
            if loaded.frame not in ('map', 'field'):
                raise ValueError('地形工程坐标系无效')
            if target and loaded.frame != target:
                raise ValueError('场地工作区需要场地坐标系，建图工作区需要地图坐标系')
            if operation == 'reference' and loaded.frame == 'map':
                self.map_reference = TerrainDocument(loaded, source, archive)
            else:
                self.check_discard(loaded.frame, body)
                self.documents[loaded.frame] = TerrainDocument(loaded, source, archive)
                if loaded.frame == 'field':
                    self.map_reference = None
                if operation == 'open' or self.document is None:
                    self.active = loaded.frame
            if operation == 'reference':
                return dict(message=f'已载入参考场地，共 {len(loaded.ids())} 个面片，{len(loaded.ann)} 个有标注')
            return dict(message=f'已打开工程，共 {len(loaded.ids())} 个面片')
        if operation == 'switch':
            frame = body.get('frame')
            if frame not in ('map', 'field'):
                raise ValueError('地形工作区无效')
            self.active = frame
            return dict(selected=[])
        if operation == 'seed':
            filename = workspace_path(body.get('source'), '配准初值文件')
            if filename.name != 'terrain.json' or not filename.is_file():
                raise ValueError('请选择已导出的 terrain.json')
            frame = self.reference.project.frame if self.reference else 'field'
            return dict(message='已读入配准初值', guess=self.read_seed(filename, frame))
        document = self.document
        if document is None:
            raise ValueError('请先打开地形工程')
        project = document.project
        if operation == 'save':
            destination = workspace_path(body.get('output'), '标注工程路径')
            if not destination.name.lower().endswith('.surf.npz'):
                raise ValueError('工程文件名需要以 .surf.npz 结尾')
            project.save(destination)
            document.archive, document.dirty = destination, False
            return dict(message=f'工程已保存到 {destination.relative_to(WORKSPACE)}')
        if operation == 'export':
            destination = map_output(body.get('output'))
            sources = [*self.documents.values()]
            if self.map_reference:
                sources.append(self.map_reference)
            for item in sources:
                if item.source.is_relative_to(destination) or (item.archive and item.archive.is_relative_to(destination)):
                    raise ValueError('导出目录不能覆盖已打开的地形来源或工程文件')
            cfg = ExportConfig(direction_split_deg=number(body.get('direction_split_deg', 20), 1, 180, '方向拆分角'),
                               speed_limits={key: int(number(body.get(key, default), 1, 100, '限速百分比'))
                                             for key, default in (('tunnel', 35), ('slope', 60), ('undulation', 50))})
            destination.parent.mkdir(parents=True, exist_ok=True)
            with TemporaryDirectory(prefix='.terrain_', dir=destination.parent) as temporary:
                staging = Path(temporary) / 'terrain'
                meta, zones = project.export(staging, cfg)
                if map_output(body.get('output')) != destination:
                    raise ValueError('导出路径已变更，请重新提交')
                publish_export(staging, destination)
            return dict(message=f'地形已导出到 {destination.relative_to(WORKSPACE)}，共 {len(zones)} 个方向区域，'
                                f'{meta["unannotated_cells"]} 格未标注，{meta["blocked_cells"]} 格禁止通行',
                        output=str(destination.relative_to(WORKSPACE)))
        response = self.edit(document, operation, body)
        if operation != 'connected':
            document.dirty = True
        return response

    def check_discard(self, frame, body):
        document = self.documents.get(frame)
        if document and document.dirty and body.get('discard') is not True:
            raise ValueError('该工程有未保存修改，请先保存或确认放弃')

    @staticmethod
    def read_seed(filename, reference_frame='field'):
        content = json.loads(filename.read_text(encoding='utf-8'))
        reg = content.get('registration', content) if isinstance(content, dict) else None
        if isinstance(reg, dict) and reg.get('reference_frame', 'field') != reference_frame:
            raise ValueError('配准文件的参考坐标系与当前参考不一致')
        return TerrainEditor.guess(reg)

    @staticmethod
    def guess(reg):
        if not isinstance(reg, dict):
            raise ValueError('文件中没有配准初值')
        translation = coordinates(reg.get('t_world_map'), [(-1000, 1000)] * 3, '配准平移')
        rotation = coordinates(reg.get('rpy_deg'), [(-360, 360)] * 3, '配准姿态')
        return [*translation, rotation[2]]

    def default_seed(self, document):
        if document.project.frame != 'map':
            return None
        reference_frame = self.reference.project.frame if self.reference else 'field'
        reg = document.project.reg
        if reg and reg.get('residual_p50', float('inf')) < 0.06 and reg.get('inlier_ratio_8cm', 0) > 0.6:
            saved_for_reference = (document.registered_to is None and
                                   reg.get('reference_frame', 'field') == reference_frame and
                                   reg.get('reference', self.reference_path) == self.reference_path)
            if self.aligned(document) or saved_for_reference:
                return self.guess(reg)
        if reference_frame == 'map':
            return None
        base = document.source if document.source.is_dir() else document.source.parent
        for file in (base / 'terrain' / 'terrain.json', base / 'terrain.json'):
            if file.is_file():
                try:
                    return self.read_seed(file)
                except (OSError, ValueError):
                    continue
        return None

    def edit(self, document, operation, body):
        project = document.project
        selected = body.get('selected', [])
        if not isinstance(selected, list) or any(type(pid) is not int for pid in selected):
            raise ValueError('面片选择无效')
        present = set(project.ids())
        ids = list(dict.fromkeys(pid for pid in selected if pid in present))
        if operation in ('annotate', 'flip', 'auto_direction', 'merge', 'grow', 'shrink', 'reset', 'connected') and not ids:
            raise ValueError('请先选中面片')
        if operation == 'landmark':
            if 'cell' not in body:
                raise ValueError('需要标点格子或明确清除标点')
            project.set_landmark(body.get('name'), body['cell'])
        elif operation == 'annotate':
            changes = {}
            if 'type' in body:
                label = body['type']
                if type(label) is not int or label not in TYPES:
                    raise ValueError('地形类型无效')
                changes['type'] = label
            if 'passable' in body:
                changes['passable'] = flag(body['passable'], '允许通行')
            if 'dir' in body:
                changes.update(dir=number(body['dir'], -360, 360, '方向') % 360, rev=False)
            project.set_ann(ids, **changes)
        elif operation == 'flip':
            project.flip_dir(ids)
        elif operation == 'auto_direction':
            project.set_ann(ids, dir=None, rev=False)
        elif operation == 'merge':
            ids = [project.merge(ids)]
        elif operation == 'grow':
            project.grow(ids[0])
        elif operation == 'shrink':
            project.shrink(ids[0])
        elif operation == 'reset':
            project.reset_extent(ids)
        elif operation == 'connected':
            ids = project.flood_level(ids)
        elif operation == 'undo':
            if not project.undo():
                return dict(message='没有可撤销的操作', selected=ids)
        elif operation == 'redo':
            if not project.redo():
                return dict(message='没有可重做的操作', selected=ids)
        elif operation == 'region':
            ids = self.region(project, ids, body)
        elif operation == 'segment':
            project.segment(replace(project.params, **checked_options(body.get('params'), SEGMENT_FIELDS)))
            ids = []
        elif operation == 'register':
            reference = self.reference
            if reference is None or project.frame != 'map':
                raise ValueError('配准需要已打开的点云工程和参考工程或场地模型')
            guess = coordinates(body.get('guess'), [(-1000, 1000)] * 3 + [(-360, 360)], '配准初值')
            registration = project.register_to(reference.project, guess[:3], guess[3])
            registration['reference'] = self.reference_path
            document.registered_to = reference.project
        elif operation == 'adopt':
            if not self.aligned(document):
                raise ValueError('请先配准点云和参考工程或场地模型')
            project.adopt_from(self.reference.project)
            document.registered_to = None
            ids = []
        else:
            raise ValueError('未知地形操作')
        return dict(message='已更新地形工程' if operation != 'connected' else f'已选中 {len(ids)} 个同高相连面片',
                    selected=ids)

    def region(self, project, ids, body):
        height, width = project.shape
        raw = body.get('points')
        if not isinstance(raw, list) or not 1 <= len(raw) <= 500:
            raise ValueError('绘制区域无效')
        points = np.asarray([coordinates(point, [(0, width - 1), (0, height - 1)], '格子')
                             for point in raw], dtype=np.int32)
        tool = body.get('tool')
        start, end = tuple(points[0]), tuple(points[-1])
        if tool == 'line':
            result = project.split_line(start, end, ids or None)
            if not result:
                raise ValueError('切割线须完整穿过面片')
            return result
        mask = np.zeros(project.shape, dtype=np.uint8)
        if tool == 'rectangle':
            cv2.rectangle(mask, start, end, 1, -1)
        elif tool == 'polygon' and len(points) >= 3:
            cv2.fillPoly(mask, [points], 1)
        elif tool == 'brush':
            radius = int(number(body.get('radius', 3), 1, 50, '画笔半径'))
            cv2.polylines(mask, [points], False, 1, radius * 2)
            cv2.circle(mask, start, radius, 1, -1)
        else:
            raise ValueError('绘制工具或点数无效')
        region = mask.astype(bool) & np.isfinite(project.surf.z)
        mode = body.get('mode')
        if mode != 'join' and ids and body.get('clip') is True:
            region &= np.isin(project.owner, ids)
        if not region.any():
            raise ValueError('绘制范围内没有可编辑的格子')
        if mode == 'restore':
            project.restore_cells(region)
            return ids
        if mode == 'join' and ids:
            return [project.paint(region, ids[0])]
        if mode == 'new':
            return [project.paint(region, 0)]
        raise ValueError('并入前请先选择目标面片')
