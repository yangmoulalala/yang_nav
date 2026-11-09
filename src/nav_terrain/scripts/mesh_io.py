#!/usr/bin/env python3
"""mesh_io: field model loading for nav_terrain -> triangles (M, 3, 3) float64, metres, field frame.

Formats  .stl          binary or ascii
         .npz          V (N, 3), F (M, 3)   (tools/cad_map, drone parse_step output)
         .step / .stp  tessellated with OpenCASCADE (pip install cadquery-ocp); the raw tessellation is
                       cached under $XDG_CACHE_HOME/nav_terrain keyed by path, size, mtime, deflection
STEP coordinates are millimetres: p = (p_step_mm - offset_mm) / 1000.  RMUC2027_V1.0.0.step has the
field centre at offset_mm = (0, 1624.3, -1941): floor at z = 0 like field_collision.stl.
Triangle winding follows the outward face normal (reversed faces are flipped); mesh_to_terrain relies
on it to tell floors (up-facing) from ceilings.
"""
import hashlib
import os
import time
from pathlib import Path

import numpy as np

STEP_SUFFIXES = ('.step', '.stp')
SUFFIXES = ('.stl', '.npz') + STEP_SUFFIXES


def _check_lfs(data, path):
    if data[:40].startswith(b'version https://git-lfs'):
        raise ValueError('%s is a Git LFS pointer; run: git lfs pull --include=%s' % (path, path))


def load_stl(path):
    data = Path(path).read_bytes()
    _check_lfs(data, path)
    if len(data) >= 84:
        n = int(np.frombuffer(data, np.uint32, 1, 80)[0])
        if 84 + 50 * n == len(data):
            rec = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
            return np.frombuffer(data, rec, n, 84)['v'].astype(np.float64)
    text = data.decode('ascii', errors='replace')
    if not text.lstrip().startswith('solid'):
        raise ValueError('not an STL file: %s' % path)
    v = [ln.split()[1:4] for ln in text.splitlines() if ln.strip().startswith('vertex')]
    if not v or len(v) % 3:
        raise ValueError('malformed ascii STL: %s' % path)
    return np.asarray(v, float).reshape(-1, 3, 3)


def load_npz(path):
    d = np.load(path)
    return d['V'].astype(np.float64)[d['F']]


def step_cache_path(path, deflection_mm):
    p = Path(path).resolve()
    st = p.stat()
    key = '%s|%d|%d|%.6f' % (p, st.st_size, int(st.st_mtime), deflection_mm)
    root = Path(os.environ.get('XDG_CACHE_HOME') or Path.home() / '.cache') / 'nav_terrain'
    return root / ('%s_%s.npz' % (p.stem, hashlib.sha1(key.encode()).hexdigest()[:16]))


def tessellate_step(path, deflection_mm=10.0, angular_rad=0.5, log=print):
    """STEP -> (V mm float32 [N, 3], F int32 [M, 3]) with outward winding."""
    try:
        from OCP.BRep import BRep_Tool
        from OCP.BRepMesh import BRepMesh_IncrementalMesh
        from OCP.IFSelect import IFSelect_RetDone
        from OCP.STEPControl import STEPControl_Reader
        from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopLoc import TopLoc_Location
        from OCP.TopoDS import TopoDS
    except ImportError as e:
        raise ImportError('reading STEP needs OpenCASCADE: pip install cadquery-ocp') from e
    t0 = time.time()
    r = STEPControl_Reader()
    if r.ReadFile(str(path)) != IFSelect_RetDone:
        raise ValueError('cannot read STEP %s' % path)
    r.TransferRoots()
    shape = r.OneShape()
    log('STEP %s read in %.0f s, tessellating (deflection %.1f mm)' % (path, time.time() - t0, deflection_mm))
    BRepMesh_IncrementalMesh(shape, deflection_mm, False, angular_rad, True)
    Vs, Fs, nv = [], [], 0
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        f = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None and tri.NbTriangles():
            tr = loc.Transformation()
            n, m = tri.NbNodes(), tri.NbTriangles()
            P = np.empty((n, 3))
            for k in range(1, n + 1):
                p = tri.Node(k).Transformed(tr)
                P[k - 1] = (p.X(), p.Y(), p.Z())
            T = np.empty((m, 3), np.int64)
            for k in range(1, m + 1):
                T[k - 1] = tri.Triangle(k).Get()
            if f.Orientation() == TopAbs_REVERSED:
                T = T[:, [0, 2, 1]]
            Vs.append(P.astype(np.float32))
            Fs.append((T - 1 + nv).astype(np.int32))
            nv += n
        ex.Next()
    if not Fs:
        raise ValueError('STEP %s produced no triangles' % path)
    V, F = np.concatenate(Vs), np.concatenate(Fs)
    log('STEP tessellated: %d triangles in %.0f s' % (len(F), time.time() - t0))
    return V, F


def load_step(path, offset_mm=(0.0, 0.0, 0.0), deflection_mm=10.0, log=print):
    cache = step_cache_path(path, deflection_mm)
    if cache.is_file():
        d = np.load(cache)
        V, F = d['V'], d['F']
        log('STEP tessellation from cache %s' % cache)
    else:
        V, F = tessellate_step(path, deflection_mm, log=log)
        cache.parent.mkdir(parents=True, exist_ok=True)
        tmp = cache.with_name(cache.stem + '.tmp.npz')
        np.savez_compressed(tmp, V=V, F=F)
        tmp.replace(cache)
    V = (V.astype(np.float64) - np.asarray(offset_mm, float)) / 1000.0
    return V[F]


def load_mesh(path, step_offset_mm=(0.0, 0.0, 0.0), step_deflection_mm=10.0, log=print):
    """Any supported field model -> triangles (M, 3, 3) float64 in metres."""
    path = Path(path)
    suf = path.suffix.lower()
    if suf == '.stl':
        return load_stl(path)
    if suf == '.npz':
        return load_npz(path)
    if suf in STEP_SUFFIXES:
        return load_step(path, step_offset_mm, step_deflection_mm, log)
    raise ValueError('unsupported field model %s (expected %s)' % (path, ', '.join(SUFFIXES)))
