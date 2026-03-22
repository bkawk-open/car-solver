"""3D monocoque tub geometry and load case definitions.

Generates a voxel representation of the monocoque tub from vehicle
geometry config, with obstacle/preserved regions and pickup points.
Applies load cases per the spec development sequence: torsion first,
then bending, then corner loads.

Coordinate system:
    x = longitudinal (0 at front, positive rearward)
    y = lateral (0 at centreline, positive to driver's right)
    z = vertical (0 at floor, positive upward)

The model uses half-width symmetry (y >= 0) to halve element count.
"""

import numpy as np

from car_solver.config import Config, SIMPConfig
from car_solver.solver3d import Solver3D


def _node_index(ix: int, iy: int, iz: int, nely: int, nelz: int) -> int:
    """Node index from grid coordinates."""
    return ix * (nely + 1) * (nelz + 1) + iy * (nelz + 1) + iz


def _elem_index(ix: int, iy: int, iz: int, nely: int, nelz: int) -> int:
    """Element index from grid coordinates."""
    return ix * nely * nelz + iy * nelz + iz


class MonocoqueTub:
    """3D monocoque tub geometry derived from vehicle config.

    All dimensions are in element units. The element size controls
    resolution vs computation time.
    """

    def __init__(self, cfg: Config, element_size_mm: float = 50.0):
        self.cfg = cfg
        self.es = element_size_mm

        # Tub dimensions in elements
        # Length: wheelbase (front axle to rear axle)
        self.nelx = int(cfg.geometry.wheelbase_mm / element_size_mm)
        # Width: half the front track (symmetry model)
        self.nely = int(cfg.geometry.front_track_mm / 2 / element_size_mm)
        # Height: approximate tub depth as 40% of overall height
        tub_height_mm = cfg.geometry.overall_height_mm * 0.40
        self.nelz = int(tub_height_mm / element_size_mm)

        # Ensure minimum dimensions
        self.nelx = max(self.nelx, 10)
        self.nely = max(self.nely, 6)
        self.nelz = max(self.nelz, 4)

        self.nel = self.nelx * self.nely * self.nelz

        # Build element masks
        self.designable = np.ones(self.nel, dtype=bool)
        self.obstacle = np.zeros(self.nel, dtype=bool)
        self.x_init = np.full(self.nel, cfg.simp.volume_fraction)

        self._define_regions()

    def _define_regions(self):
        """Define preserved, obstacle, and designable regions.

        Preserved regions use 1-element-thick walls to minimise the
        non-designable fraction. The optimiser decides where to add
        material within the designable sill box and internal web space.
        """
        nelx, nely, nelz = self.nelx, self.nely, self.nelz

        def _preserve(ix, iy, iz):
            el = _elem_index(ix, iy, iz, nely, nelz)
            self.designable[el] = False
            self.x_init[el] = 1.0

        def _obstacle(ix, iy, iz):
            el = _elem_index(ix, iy, iz, nely, nelz)
            self.obstacle[el] = True
            self.designable[el] = False
            self.x_init[el] = 0.001

        # --- Preserved: 1-element-thick shells ---

        # Floor: z=0
        for ix in range(nelx):
            for iy in range(nely):
                _preserve(ix, iy, 0)

        # Scuttle top: z=nelz-1
        for ix in range(nelx):
            for iy in range(nely):
                _preserve(ix, iy, nelz - 1)

        # Front bulkhead: x=0
        for iy in range(nely):
            for iz in range(nelz):
                _preserve(0, iy, iz)

        # Rear bulkhead: x=nelx-1
        for iy in range(nely):
            for iz in range(nelz):
                _preserve(nelx - 1, iy, iz)

        # Sill outer wall: y=nely-1
        for ix in range(nelx):
            for iz in range(nelz):
                _preserve(ix, nely - 1, iz)

        # --- Obstacle: cockpit void ---
        # Large interior cavity leaving the sill box region designable
        cockpit_x_start = 2
        cockpit_x_end = nelx - 2
        cockpit_y_end = int(nely * 0.50)
        cockpit_z_start = 2
        cockpit_z_end = nelz - 2

        for ix in range(cockpit_x_start, cockpit_x_end):
            for iy in range(cockpit_y_end):
                for iz in range(cockpit_z_start, cockpit_z_end):
                    _obstacle(ix, iy, iz)

    def node(self, ix: int, iy: int, iz: int) -> int:
        """Get node index."""
        return _node_index(ix, iy, iz, self.nely, self.nelz)

    def node_dof(self, ix: int, iy: int, iz: int, axis: int) -> int:
        """Get DOF index for a node. axis: 0=x, 1=y, 2=z."""
        return 3 * self.node(ix, iy, iz) + axis

    # Tuned SIMP parameters for 3D monocoque: p=4 and r=1.2 give
    # 17% grey elements vs 57% with the 2D defaults (p=3 r=1.5)
    MONOCOQUE_SIMP = dict(penalty=4.0, convergence_tolerance=0.005, filter_radius=1.2)

    def pickup_nodes(self) -> dict[str, int]:
        """Return node indices for all four suspension pickup points.

        Right-side pickups are at the outer sill (y=nely).
        Left-side pickups are on the centreline (y=0) - the symmetry
        plane mirrors these to the physical left-side position.
        All pickups at floor level (z=0).
        """
        return {
            "front_right": self.node(0, self.nely, 0),
            "rear_right": self.node(self.nelx, self.nely, 0),
            "front_left": self.node(0, 0, 0),
            "rear_left": self.node(self.nelx, 0, 0),
        }

    def build_solver(self) -> Solver3D:
        """Create a Solver3D with tuned monocoque SIMP parameters."""
        simp = SIMPConfig(
            volume_fraction=self.cfg.simp.volume_fraction,
            max_iterations=self.cfg.simp.max_iterations,
            **self.MONOCOQUE_SIMP,
        )
        return Solver3D(self.nelx, self.nely, self.nelz, simp)

    def symmetry_dofs(self) -> np.ndarray:
        """DOFs fixed for half-width symmetry: y-displacement = 0 on y=0 face."""
        dofs = []
        for ix in range(self.nelx + 1):
            for iz in range(self.nelz + 1):
                dofs.append(self.node_dof(ix, 0, iz, 1))  # fix y
        return np.array(dofs, dtype=int)


def _pickup_dofs(tub: MonocoqueTub, names: list[str]) -> np.ndarray:
    """Fix all 3 DOFs (x, y, z) at the named pickup nodes."""
    pickups = tub.pickup_nodes()
    dofs = []
    for name in names:
        n = pickups[name]
        dofs.extend([3*n, 3*n+1, 3*n+2])
    return np.array(dofs, dtype=int)


def _rear_face_dofs(tub: MonocoqueTub) -> np.ndarray:
    """Fix all DOFs on the rear face (x=nelx)."""
    dofs = []
    for iy in range(tub.nely + 1):
        for iz in range(tub.nelz + 1):
            n = tub.node(tub.nelx, iy, iz)
            dofs.extend([3*n, 3*n+1, 3*n+2])
    return np.array(dofs, dtype=int)


def _solve_load_case(
    tub: MonocoqueTub,
    fixed_dofs: np.ndarray,
    forces: np.ndarray | list[np.ndarray],
    weights: list[float] | None = None,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float]]:
    """Common solve wrapper for all monocoque load cases."""
    solver = tub.build_solver()
    return solver.solve(
        fixed_dofs,
        forces,
        weights=weights,
        designable=tub.designable,
        obstacle=tub.obstacle,
        x_init=tub.x_init,
        on_iteration=on_iteration,
        continuation=True,
    )


def torsion_load_case(
    cfg: Config,
    element_size_mm: float = 50.0,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float], MonocoqueTub]:
    """Torsion load case: equal and opposite vertical loads at front corners.

    Per spec: 1000N vertical at front left and front right pickup points,
    rear fixed. Since we model half-width (right side only), we apply
    1000N downward at the front-right pickup. The symmetry BC on the
    centreline provides the equal-and-opposite constraint.

    Rear is fully fixed (all DOFs on rear bulkhead face).

    Returns (densities, compliance_history, tub).
    """
    tub = MonocoqueTub(cfg, element_size_mm)
    pickups = tub.pickup_nodes()

    # BCs: symmetry + rear face fully fixed
    fixed_dofs = np.union1d(tub.symmetry_dofs(), _rear_face_dofs(tub))

    # 1000N downward at front-right pickup (symmetry gives equal-opposite)
    solver = tub.build_solver()
    force = np.zeros(solver.ndof)
    force[3 * pickups["front_right"] + 2] = -1.0

    densities, history = _solve_load_case(tub, fixed_dofs, force, on_iteration=on_iteration)
    return densities, history, tub


def bending_load_case(
    cfg: Config,
    element_size_mm: float = 50.0,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float], MonocoqueTub]:
    """Bending load case: distributed vertical load, fixed at all four pickups.

    Per spec: distributed load along sill representing vehicle weight at 1g,
    fixed at all four pickup points. The load is applied as uniform downward
    force on nodes along the top of the outer sill (y=nely, z=nelz).

    In the half-width symmetry model, we fix all four pickup corners
    (right-side at the sill, left-side on the centreline) and distribute
    the total vehicle weight as downward force along the sill top edge.

    Returns (densities, compliance_history, tub).
    """
    tub = MonocoqueTub(cfg, element_size_mm)
    solver = tub.build_solver()

    # BCs: symmetry + all four pickup corners fixed
    pickup_dofs = _pickup_dofs(tub, ["front_right", "rear_right", "front_left", "rear_left"])
    fixed_dofs = np.union1d(tub.symmetry_dofs(), pickup_dofs)

    # Distributed downward load along sill top edge (y=nely, z=nelz)
    # These nodes run the full length of the tub at the outer sill top
    force = np.zeros(solver.ndof)
    sill_top_nodes = []
    for ix in range(tub.nelx + 1):
        sill_top_nodes.append(tub.node(ix, tub.nely, tub.nelz))

    load_per_node = -1.0 / len(sill_top_nodes)
    for n in sill_top_nodes:
        force[3 * n + 2] += load_per_node  # z-direction, downward

    densities, history = _solve_load_case(tub, fixed_dofs, force, on_iteration=on_iteration)
    return densities, history, tub


def _run_and_save(name, densities, history, tub):
    """Save convergence plot, VTK, and PNG for a load case."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from car_solver.output import output_path
    from car_solver.visualise3d import save_density_vtk, plot_density_3d

    print(f"Grid: {tub.nelx}x{tub.nely}x{tub.nelz} ({tub.nel:,} elements)")
    print(f"Converged in {len(history)} iterations")
    print(f"Final compliance: {history[-1]:.4f}")

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(history, "b-", linewidth=1.5)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Compliance")
    ax.set_title(f"Monocoque {name.title()} Convergence")
    ax.grid(True, alpha=0.3)
    fig.savefig(output_path(f"{name}_convergence.png"), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {output_path(f'{name}_convergence.png')}")

    save_density_vtk(densities, tub.nelx, tub.nely, tub.nelz, output_path(f"{name}_density.vtk"))

    try:
        plot_density_3d(
            densities, tub.nelx, tub.nely, tub.nelz,
            output_path(f"{name}_density.png"), threshold=0.25,
        )
    except Exception as e:
        print(f"PNG render skipped: {e}")


if __name__ == "__main__":
    import sys
    from car_solver.config import load_config

    cfg = load_config()
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"

    def on_iter(it, densities, compliance, change):
        if it % 10 == 0:
            print(f"  Iteration {it:3d}: compliance={compliance:.4f}, change={change:.4f}")

    if mode in ("all", "torsion"):
        print("Running monocoque torsion...")
        d, h, tub = torsion_load_case(cfg, on_iteration=on_iter)
        _run_and_save("torsion", d, h, tub)

    if mode in ("all", "bending"):
        print("\nRunning monocoque bending...")
        d, h, tub = bending_load_case(cfg, on_iteration=on_iter)
        _run_and_save("bending", d, h, tub)
