"""Read and atomically write version-1 surface annotation archives."""
import json
from pathlib import Path

import numpy as np

from nav_terrain.surface_seg import SegParams, Surface

FORMAT = 1


def save_project(project, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    surface = project.surf
    meta = {'format': FORMAT, 'frame': project.frame, 'source': project.source, 'params': project.params.to_dict(),
            'resolution': surface.resolution, 'origin': list(surface.origin), 'reg': project.reg,
            'ann': {str(key): value for key, value in project.ann.items()}, 'landmarks': project.landmarks}
    tmp = path.with_name(path.name + '.tmp.npz')
    if project.match_yaw_deg is not None:
        meta['match_yaw_deg'] = project.match_yaw_deg
    gradients = {'gx': surface.gx, 'gy': surface.gy} if surface.gx is not None else {}
    scan = {} if project.scan_surface is None else {'scan_z': project.scan_surface.z,
                                                   'scan_clearance': project.scan_surface.clearance}
    np.savez_compressed(tmp, z=surface.z, clearance=surface.clearance, seg=project.seg, owner=project.owner,
                        reference_fill=project.reference_fill, meta=np.frombuffer(json.dumps(meta).encode(), np.uint8), **gradients, **scan)
    tmp.replace(path)


def load_project(project_type, path):
    with np.load(path, allow_pickle=False) as data:
        meta = json.loads(data['meta'].tobytes())
        surface = Surface(data['z'], float(meta['resolution']), tuple(meta['origin']), data['clearance'],
                          data.get('gx'), data.get('gy'))
        project = project_type(surface, SegParams.from_dict(meta['params']), meta['frame'], meta.get('source', ''))
        project.seg = data['seg'].astype(np.int32)
        project.owner = data['owner'].astype(np.int32)
        if 'reference_fill' in data:
            mask = data['reference_fill']
            if mask.dtype != bool or mask.shape != surface.z.shape:
                raise ValueError('参考补齐来源网格无效')
            project.reference_fill = mask
        if 'scan_z' in data or 'scan_clearance' in data:
            if not {'scan_z', 'scan_clearance'}.issubset(data.files):
                raise ValueError('原始观测网格不完整')
            scan_z, scan_clearance = data['scan_z'], data['scan_clearance']
            if scan_z.shape != surface.z.shape or scan_clearance.shape != surface.z.shape:
                raise ValueError('原始观测网格尺寸不一致')
            project.scan_surface = Surface(scan_z, surface.resolution, surface.origin, scan_clearance,
                                           surface.gx, surface.gy)
    project.ann = {int(key): value for key, value in meta.get('ann', {}).items()}
    project.reg = meta.get('reg')
    for name, point in meta.get('landmarks', {}).items():
        if (project.frame != 'map' or name not in ('home', 'target', 'match') or not isinstance(point, list)
                or len(point) != 2 or not all(type(value) in (int, float) and np.isfinite(value) for value in point)):
            raise ValueError('工程标点无效')
        project.landmarks[name] = tuple(point)
    if meta.get('match_yaw_deg') is not None:
        project.set_match_yaw(meta['match_yaw_deg'])
        project.history.clear()
    return project
