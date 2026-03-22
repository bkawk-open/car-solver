"""3D visualisation for topology optimisation results using PyVista."""

import numpy as np


def save_density_vtk(
    densities: np.ndarray,
    nelx: int,
    nely: int,
    nelz: int,
    save_path: str = "density3d.vtk",
    threshold: float = 0.3,
):
    """Save 3D density field as VTK file for viewing in ParaView.

    Elements below threshold are excluded. The remaining elements are
    coloured by density value.
    """
    import pyvista as pv

    grid = pv.ImageData(dimensions=(nelx + 1, nely + 1, nelz + 1))
    grid.cell_data["density"] = densities

    # Threshold to show only material above cutoff
    threshed = grid.threshold(threshold, scalars="density")

    threshed.save(save_path)
    print(f"Saved 3D density to {save_path} ({threshed.n_cells} cells above {threshold})")


def plot_density_3d(
    densities: np.ndarray,
    nelx: int,
    nely: int,
    nelz: int,
    save_path: str = "density3d.png",
    threshold: float = 0.3,
):
    """Render 3D density field to PNG using PyVista off-screen."""
    import pyvista as pv

    pv.OFF_SCREEN = True

    grid = pv.ImageData(dimensions=(nelx + 1, nely + 1, nelz + 1))
    grid.cell_data["density"] = densities

    threshed = grid.threshold(threshold, scalars="density")

    plotter = pv.Plotter(off_screen=True, window_size=[1600, 900])
    plotter.add_mesh(
        threshed,
        scalars="density",
        cmap="bone_r",
        show_edges=False,
        opacity=0.9,
    )
    plotter.add_axes()
    plotter.camera_position = "iso"
    plotter.screenshot(save_path)
    plotter.close()
    print(f"Saved 3D render to {save_path}")


if __name__ == "__main__":
    import sys
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from car_solver.solver3d import cantilever_3d

    def on_iter(it, densities, compliance, change):
        if it % 5 == 0:
            print(f"  Iteration {it:3d}: compliance={compliance:.4f}, change={change:.4f}")

    nelx, nely, nelz = 30, 10, 6
    if len(sys.argv) > 1:
        parts = sys.argv[1].split("x")
        if len(parts) == 3:
            nelx, nely, nelz = int(parts[0]), int(parts[1]), int(parts[2])

    print(f"Running 3D cantilever ({nelx}x{nely}x{nelz}, {nelx*nely*nelz} elements)...")
    densities, history, *_ = cantilever_3d(
        nelx=nelx, nely=nely, nelz=nelz, on_iteration=on_iter,
    )
    print(f"Converged in {len(history)} iterations")
    print(f"Final compliance: {history[-1]:.4f}")

    # Save convergence plot
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(history, "b-", linewidth=1.5)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Compliance")
    ax.set_title(f"3D Cantilever Convergence ({nelx}x{nely}x{nelz})")
    ax.grid(True, alpha=0.3)
    fig.savefig("convergence3d.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("Saved convergence3d.png")

    # Save VTK for ParaView
    save_density_vtk(densities, nelx, nely, nelz)

    # Try PNG render (may fail without GPU)
    try:
        plot_density_3d(densities, nelx, nely, nelz)
    except Exception as e:
        print(f"PNG render skipped (headless): {e}")
