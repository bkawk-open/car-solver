"""3D monocoque tub geometry and load case definitions.

Generates a voxel representation of the monocoque tub from vehicle
geometry config, with obstacle/preserved regions and pickup points.
Applies load cases per the spec development sequence: torsion first,
then bending, then corner loads.

Coordinate system:
    x = longitudinal (0 at front, positive rearward)
    y = lateral (0 at left edge, nely at right edge, centreline at nely//2)
    z = vertical (0 at floor, positive upward)

Full-width model (no symmetry reduction) so torsion antisymmetry
is handled correctly.
"""

import numpy as np

from car_solver.config import Config, SIMPConfig
from car_solver.solver3d import Solver3D


def _node_index(ix: int, iy: int, iz: int, nely: int, nelz: int) -> int:
    return ix * (nely + 1) * (nelz + 1) + iy * (nelz + 1) + iz


def _elem_index(ix: int, iy: int, iz: int, nely: int, nelz: int) -> int:
    return ix * nely * nelz + iy * nelz + iz


class MonocoqueTub:
    """3D monocoque tub geometry derived from vehicle config.

    Full-width model. Element size controls resolution vs speed.
    """

    MONOCOQUE_SIMP = dict(penalty=4.0, convergence_tolerance=0.005, filter_radius=1.2)

    def __init__(self, cfg: Config, element_size_mm: float = 50.0):
        self.cfg = cfg
        self.es = element_size_mm

        self.nelx = max(int(cfg.geometry.wheelbase_mm / element_size_mm), 10)
        # Full width: front track
        self.nely = max(int(cfg.geometry.front_track_mm / element_size_mm), 12)
        tub_height_mm = cfg.geometry.overall_height_mm * 0.40
        self.nelz = max(int(tub_height_mm / element_size_mm), 4)

        self.nel = self.nelx * self.nely * self.nelz

        self.designable = np.ones(self.nel, dtype=bool)
        self.obstacle = np.zeros(self.nel, dtype=bool)
        self.x_init = np.full(self.nel, cfg.simp.volume_fraction)

        self._define_regions()

    def _define_regions(self):
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

        # Sill walls: y=0 (left) and y=nely-1 (right)
        for ix in range(nelx):
            for iz in range(nelz):
                _preserve(ix, 0, iz)
                _preserve(ix, nely - 1, iz)

        # Cockpit void: central interior cavity
        cockpit_x_start = 2
        cockpit_x_end = nelx - 2
        cockpit_y_start = int(nely * 0.20)
        cockpit_y_end = int(nely * 0.80)
        cockpit_z_start = 2
        cockpit_z_end = nelz - 2

        for ix in range(cockpit_x_start, cockpit_x_end):
            for iy in range(cockpit_y_start, cockpit_y_end):
                for iz in range(cockpit_z_start, cockpit_z_end):
                    _obstacle(ix, iy, iz)

    def node(self, ix: int, iy: int, iz: int) -> int:
        return _node_index(ix, iy, iz, self.nely, self.nelz)

    def node_dof(self, ix: int, iy: int, iz: int, axis: int) -> int:
        return 3 * self.node(ix, iy, iz) + axis

    def pickup_nodes(self) -> dict[str, int]:
        """All four suspension pickup points at floor level."""
        return {
            "front_left": self.node(0, 0, 0),
            "front_right": self.node(0, self.nely, 0),
            "rear_left": self.node(self.nelx, 0, 0),
            "rear_right": self.node(self.nelx, self.nely, 0),
        }

    def build_solver(self) -> Solver3D:
        simp = SIMPConfig(
            volume_fraction=self.cfg.simp.volume_fraction,
            max_iterations=self.cfg.simp.max_iterations,
            **self.MONOCOQUE_SIMP,
        )
        return Solver3D(self.nelx, self.nely, self.nelz, simp)


def _pickup_dofs(tub: MonocoqueTub, names: list[str]) -> np.ndarray:
    pickups = tub.pickup_nodes()
    dofs = []
    for name in names:
        n = pickups[name]
        dofs.extend([3*n, 3*n+1, 3*n+2])
    return np.array(dofs, dtype=int)


def _rear_face_dofs(tub: MonocoqueTub) -> np.ndarray:
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
    """Torsion: equal and opposite vertical loads at front corners, rear fixed.

    Per spec: +1000N at front-left, -1000N at front-right, rear face fixed.
    Full-width model required because torsion is antisymmetric.
    """
    tub = MonocoqueTub(cfg, element_size_mm)
    solver = tub.build_solver()
    pickups = tub.pickup_nodes()

    fixed_dofs = _rear_face_dofs(tub)

    force = np.zeros(solver.ndof)
    force[3 * pickups["front_left"] + 2] = 1.0    # upward
    force[3 * pickups["front_right"] + 2] = -1.0   # downward

    densities, history = _solve_load_case(tub, fixed_dofs, force, on_iteration=on_iteration)
    return densities, history, tub


def bending_load_case(
    cfg: Config,
    element_size_mm: float = 50.0,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float], MonocoqueTub]:
    """Bending: distributed vertical load along sills, fixed at all four pickups.

    Per spec: vehicle weight at 1g distributed along sill, all four
    pickup points fixed.
    """
    tub = MonocoqueTub(cfg, element_size_mm)
    solver = tub.build_solver()

    pickup_dofs = _pickup_dofs(tub, ["front_left", "front_right", "rear_left", "rear_right"])
    fixed_dofs = pickup_dofs

    # Distributed load along both sill top edges
    force = np.zeros(solver.ndof)
    sill_nodes = []
    for ix in range(tub.nelx + 1):
        sill_nodes.append(tub.node(ix, 0, tub.nelz))        # left sill top
        sill_nodes.append(tub.node(ix, tub.nely, tub.nelz))  # right sill top

    load_per_node = -1.0 / len(sill_nodes)
    for n in sill_nodes:
        force[3 * n + 2] += load_per_node

    densities, history = _solve_load_case(tub, fixed_dofs, force, on_iteration=on_iteration)
    return densities, history, tub


def corner_load_case(
    cfg: Config,
    element_size_mm: float = 50.0,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float], MonocoqueTub]:
    """Corner loads: front and rear, vertical + lateral + braking.

    Per spec: front corner vertical (3g), lateral (1.5g), braking (1.2g),
    and rear corner equivalent. Loads applied one element above floor
    at sill edges. All four pickups pinned.
    """
    from car_solver.loads import calculate_load_cases

    tub = MonocoqueTub(cfg, element_size_mm)
    solver = tub.build_solver()
    cases = calculate_load_cases(cfg)

    pickup_dofs = _pickup_dofs(tub, ["front_left", "front_right", "rear_left", "rear_right"])
    fixed_dofs = pickup_dofs

    # Load nodes: one element up from floor at sill edges
    z_load = min(1, tub.nelz)
    fl_node = tub.node(0, 0, z_load)
    fr_node = tub.node(0, tub.nely, z_load)
    rl_node = tub.node(tub.nelx, 0, z_load)
    rr_node = tub.node(tub.nelx, tub.nely, z_load)

    def _force(node, axis, sign=1.0):
        f = np.zeros(solver.ndof)
        f[3 * node + axis] = sign
        return f

    fl = cases.front_left_dynamic
    rl = cases.rear_left_dynamic
    front_total = fl.vertical_n + fl.lateral_n + fl.longitudinal_n
    rear_total = rl.vertical_n + rl.lateral_n + rl.longitudinal_n

    front_frac = 2.0 / 3.0  # spec: 0.20 vs 0.10
    rear_frac = 1.0 / 3.0

    forces = [
        # Front-right: vertical down, lateral outward (+y), braking rearward (+x)
        _force(fr_node, 2, -1.0),
        _force(fr_node, 1, 1.0),
        _force(fr_node, 0, 1.0),
        # Front-left: vertical down, lateral outward (-y), braking rearward (+x)
        _force(fl_node, 2, -1.0),
        _force(fl_node, 1, -1.0),
        _force(fl_node, 0, 1.0),
        # Rear-right: vertical down, lateral outward (+y), braking forward (-x)
        _force(rr_node, 2, -1.0),
        _force(rr_node, 1, 1.0),
        _force(rr_node, 0, -1.0),
        # Rear-left: vertical down, lateral outward (-y), braking forward (-x)
        _force(rl_node, 2, -1.0),
        _force(rl_node, 1, -1.0),
        _force(rl_node, 0, -1.0),
    ]

    weights = [
        # Front-right
        front_frac * fl.vertical_n / front_total / 2,
        front_frac * fl.lateral_n / front_total / 2,
        front_frac * fl.longitudinal_n / front_total / 2,
        # Front-left (same magnitudes)
        front_frac * fl.vertical_n / front_total / 2,
        front_frac * fl.lateral_n / front_total / 2,
        front_frac * fl.longitudinal_n / front_total / 2,
        # Rear-right
        rear_frac * rl.vertical_n / rear_total / 2,
        rear_frac * rl.lateral_n / rear_total / 2,
        rear_frac * rl.longitudinal_n / rear_total / 2,
        # Rear-left
        rear_frac * rl.vertical_n / rear_total / 2,
        rear_frac * rl.lateral_n / rear_total / 2,
        rear_frac * rl.longitudinal_n / rear_total / 2,
    ]

    densities, history = _solve_load_case(
        tub, fixed_dofs, forces, weights=weights, on_iteration=on_iteration,
    )
    return densities, history, tub


def combined_load_case(
    cfg: Config,
    element_size_mm: float = 50.0,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float], MonocoqueTub]:
    """All load cases combined with spec objective weights.

    Torsion: 0.50, Bending: 0.20, Front corners: 0.20, Rear corners: 0.10.

    All cases share a single BC set: rear face fixed. This is correct for
    torsion and conservative for bending/corners (rear face provides more
    constraint than four point pickups).
    """
    from car_solver.loads import calculate_load_cases

    tub = MonocoqueTub(cfg, element_size_mm)
    solver = tub.build_solver()
    pickups = tub.pickup_nodes()
    cases = calculate_load_cases(cfg)

    fixed_dofs = _rear_face_dofs(tub)

    # --- Torsion ---
    f_torsion = np.zeros(solver.ndof)
    f_torsion[3 * pickups["front_left"] + 2] = 1.0
    f_torsion[3 * pickups["front_right"] + 2] = -1.0

    # --- Bending ---
    f_bending = np.zeros(solver.ndof)
    sill_nodes = []
    for ix in range(tub.nelx + 1):
        sill_nodes.append(tub.node(ix, 0, tub.nelz))
        sill_nodes.append(tub.node(ix, tub.nely, tub.nelz))
    for n in sill_nodes:
        f_bending[3 * n + 2] += -1.0 / len(sill_nodes)

    # --- Corner loads ---
    z_load = min(1, tub.nelz)
    fl_n = tub.node(0, 0, z_load)
    fr_n = tub.node(0, tub.nely, z_load)
    rl_n = tub.node(tub.nelx, 0, z_load)
    rr_n = tub.node(tub.nelx, tub.nely, z_load)

    fl = cases.front_left_dynamic
    rl = cases.rear_left_dynamic
    front_total = fl.vertical_n + fl.lateral_n + fl.longitudinal_n
    rear_total = rl.vertical_n + rl.lateral_n + rl.longitudinal_n

    def _f(node, axis, sign=1.0):
        f = np.zeros(solver.ndof)
        f[3 * node + axis] = sign
        return f

    w_t = cfg.weights.torsion
    w_b = cfg.weights.bending
    w_fc = cfg.weights.front_corner
    w_rc = cfg.weights.rear_corner

    forces = [f_torsion, f_bending]
    weights = [w_t, w_b]

    # Front corners (both sides)
    for node in [fr_n, fl_n]:
        y_sign = 1.0 if node == fr_n else -1.0
        forces.extend([_f(node, 2, -1.0), _f(node, 1, y_sign), _f(node, 0, 1.0)])
        weights.extend([
            w_fc * fl.vertical_n / front_total / 2,
            w_fc * fl.lateral_n / front_total / 2,
            w_fc * fl.longitudinal_n / front_total / 2,
        ])

    # Rear corners (both sides)
    for node in [rr_n, rl_n]:
        y_sign = 1.0 if node == rr_n else -1.0
        forces.extend([_f(node, 2, -1.0), _f(node, 1, y_sign), _f(node, 0, -1.0)])
        weights.extend([
            w_rc * rl.vertical_n / rear_total / 2,
            w_rc * rl.lateral_n / rear_total / 2,
            w_rc * rl.longitudinal_n / rear_total / 2,
        ])

    densities, history = _solve_load_case(
        tub, fixed_dofs, forces, weights=weights, on_iteration=on_iteration,
    )
    return densities, history, tub


def _run_and_save(name, densities, history, tub):
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

    if mode in ("all", "corner"):
        print("\nRunning monocoque corner loads...")
        d, h, tub = corner_load_case(cfg, on_iteration=on_iter)
        _run_and_save("corner", d, h, tub)

    if mode in ("all", "combined"):
        print("\nRunning monocoque combined (all load cases)...")
        d, h, tub = combined_load_case(cfg, on_iteration=on_iter)
        _run_and_save("combined", d, h, tub)
