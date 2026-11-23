"""Geometry-only segmentation of a field mesh or prior cloud into planar and sloped patches."""
from ._geometry import adjacency, neighbours, patch_stats, slope_groups, surface_points
from ._grid import SegParams, Surface, cell_gradient
from ._height import bilateral, cloud_surface, fill_small_holes, grid_bounds, mesh_surface, raycast_ids, surfaces
from ._regions import segment, segment_with_gradient

__all__ = [
    'SegParams', 'Surface', 'mesh_surface', 'cloud_surface', 'segment', 'segment_with_gradient', 'cell_gradient',
    'patch_stats', 'neighbours', 'adjacency', 'slope_groups', 'surface_points',
    'bilateral', 'fill_small_holes', 'grid_bounds', 'raycast_ids', 'surfaces',
]
