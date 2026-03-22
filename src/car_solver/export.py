"""Geometry export helpers separate from visualisation."""

from __future__ import annotations

from car_solver.output import output_path


def export_thresholded_density_vtk(
    densities,
    nelx: int,
    nely: int,
    nelz: int,
    save_path: str | None = None,
    threshold: float = 0.3,
) -> str:
    """Export thresholded density cells as a VTK dataset."""
    import pyvista as pv

    grid = pv.ImageData(dimensions=(nelx + 1, nely + 1, nelz + 1))
    grid.cell_data["density"] = densities
    threshed = grid.threshold(threshold, scalars="density")

    path = save_path or output_path("density3d.vtk")
    threshed.save(path)
    return path


def export_thresholded_density_stl(
    densities,
    nelx: int,
    nely: int,
    nelz: int,
    save_path: str | None = None,
    threshold: float = 0.3,
) -> str:
    """Export a thresholded density field as an STL surface mesh."""
    import pyvista as pv

    grid = pv.ImageData(dimensions=(nelx + 1, nely + 1, nelz + 1))
    grid.cell_data["density"] = densities
    threshed = grid.threshold(threshold, scalars="density")
    surface = threshed.extract_surface()

    path = save_path or output_path("density3d.stl")
    surface.save(path)
    return path
