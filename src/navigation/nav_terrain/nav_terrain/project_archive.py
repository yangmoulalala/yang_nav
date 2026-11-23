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
            'ann': {str(key): value for key, value in project.ann.items()}}
    tmp = path.with_name(path.name + '.tmp.npz')
    gradients = {'gx': surface.gx, 'gy': surface.gy} if surface.gx is not None else {}
    np.savez_compressed(tmp, z=surface.z, clearance=surface.clearance, seg=project.seg, owner=project.owner,
                        meta=np.frombuffer(json.dumps(meta).encode(), np.uint8), **gradients)
    tmp.replace(path)


def load_project(project_type, path):
    with np.load(path, allow_pickle=False) as data:
        meta = json.loads(data['meta'].tobytes())
        surface = Surface(data['z'], float(meta['resolution']), tuple(meta['origin']), data['clearance'],
                          data.get('gx'), data.get('gy'))
        project = project_type(surface, SegParams.from_dict(meta['params']), meta['frame'], meta.get('source', ''))
        project.seg = data['seg'].astype(np.int32)
        project.owner = data['owner'].astype(np.int32)
    project.ann = {int(key): value for key, value in meta.get('ann', {}).items()}
    project.reg = meta.get('reg')
    return project
