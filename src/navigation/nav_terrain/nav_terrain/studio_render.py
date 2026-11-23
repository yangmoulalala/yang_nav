"""Terrain editor raster layers. Rendering is independent of Qt widgets."""
import numpy as np
from scipy import ndimage


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


def patch_edges(labels):
    edges = np.zeros(labels.shape, bool)
    edges[:, 1:] = labels[:, 1:] != labels[:, :-1]
    edges[1:] |= labels[1:] != labels[:-1]
    return edges


def compose_map(project, colors, *, show_unset, opacity, edges, overlay, selected, stroke):
    """Return a top-down RGB preview, with rows flipped from map to image coordinates."""
    own = project.owner
    valid = np.isfinite(project.surf.z)
    base = hillshade(project.surf.z)[..., None] * 255.0
    img = base * np.array([0.85, 0.87, 0.9], np.float32)
    L = int(own.max()) + 1
    col = np.zeros((L, 3), np.float32)
    alpha = np.zeros(L, np.float32)
    if show_unset:
        col[:] = patch_palette(L - 1)
        alpha[:] = 0.28
    hatch_ids = np.zeros(L, bool)
    for pid, a in project.ann.items():
        if pid < L:
            if a['type'] in colors:
                col[pid] = colors[a['type']]
                alpha[pid] = opacity
            hatch_ids[pid] = not a['passable']
    alpha[0] = 0
    A = alpha[own][..., None]
    img = img * (1 - A) + col[own] * A
    h, w = own.shape
    if hatch_ids.any():
        rr, cc = np.ogrid[0:h, 0:w]
        hm = hatch_ids[own] & (((rr + cc) % 8) < 3)
        img[hm] = img[hm] * 0.25
    if edges:
        img[patch_edges(own)] *= 0.35
    if overlay is not None and overlay.shape == own.shape:
        img[overlay] = (0, 230, 255)
    if selected:
        sm = np.isin(own, selected)
        img[sm] = img[sm] * 0.6 + np.array([255, 255, 0], np.float32) * 0.4
        edge = sm & ~ndimage.binary_erosion(sm)
        img[edge] = (255, 255, 0)
    if stroke is not None:
        img[stroke] = img[stroke] * 0.4 + np.array([255, 255, 255], np.float32) * 0.6
    img[~valid] = (25, 25, 28)
    return np.clip(img, 0, 255).astype(np.uint8)[::-1]
