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

from dataclasses import dataclass

import numpy as np

from car_solver.config import Config, SIMPConfig
from car_solver.loads import (
    DistributedLoadSpec,
    PointLoadSpec,
    SolverLoadCaseDefinition,
    build_monocoque_load_definitions,
)
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
        self.preserve = np.zeros(self.nel, dtype=bool)
        self.preserve_regions: dict[str, np.ndarray] = {}
        self.obstacle_regions: dict[str, np.ndarray] = {}
        self.load_node_groups: dict[str, np.ndarray] = {}
        self.support_dof_groups: dict[str, np.ndarray] = {}

        self._define_regions()
        self._define_analysis_groups()

    def _define_regions(self):
        nelx, nely, nelz = self.nelx, self.nely, self.nelz

        def empty_region() -> np.ndarray:
            return np.zeros(self.nel, dtype=bool)

        def fill_box(
            region: np.ndarray,
            x_range: range,
            y_range: range,
            z_range: range,
        ) -> None:
            for ix in x_range:
                for iy in y_range:
                    for iz in z_range:
                        region[_elem_index(ix, iy, iz, nely, nelz)] = True

        def add_preserve_region(name: str, region: np.ndarray) -> None:
            self.preserve_regions[name] = region
            self.preserve |= region

        def add_obstacle_region(name: str, region: np.ndarray) -> None:
            self.obstacle_regions[name] = region
            self.obstacle |= region

        # --- Preserved structural regions ---
        floor = empty_region()
        fill_box(floor, range(nelx), range(nely), range(1))
        add_preserve_region("floor", floor)

        scuttle = empty_region()
        fill_box(scuttle, range(nelx), range(nely), range(nelz - 1, nelz))
        add_preserve_region("scuttle", scuttle)

        front_bulkhead = empty_region()
        fill_box(front_bulkhead, range(1), range(nely), range(nelz))
        add_preserve_region("front_bulkhead", front_bulkhead)

        rear_bulkhead = empty_region()
        fill_box(rear_bulkhead, range(nelx - 1, nelx), range(nely), range(nelz))
        add_preserve_region("rear_bulkhead", rear_bulkhead)

        left_sill = empty_region()
        fill_box(left_sill, range(nelx), range(1), range(nelz))
        add_preserve_region("left_sill", left_sill)

        right_sill = empty_region()
        fill_box(right_sill, range(nelx), range(nely - 1, nely), range(nelz))
        add_preserve_region("right_sill", right_sill)

        # Keep pickup hard points explicit even though they overlap other preserve regions.
        pickup_hard_points = empty_region()
        pickup_boxes = [
            (range(0, min(2, nelx)), range(0, min(2, nely)), range(0, min(2, nelz))),
            (range(0, min(2, nelx)), range(max(nely - 2, 0), nely), range(0, min(2, nelz))),
            (range(max(nelx - 2, 0), nelx), range(0, min(2, nely)), range(0, min(2, nelz))),
            (range(max(nelx - 2, 0), nelx), range(max(nely - 2, 0), nely), range(0, min(2, nelz))),
        ]
        for x_range, y_range, z_range in pickup_boxes:
            fill_box(pickup_hard_points, x_range, y_range, z_range)
        add_preserve_region("pickup_hard_points", pickup_hard_points)

        # --- Obstacle regions ---
        cockpit_void = empty_region()
        cockpit_x_start = 2
        cockpit_x_end = nelx - 2
        cockpit_y_start = int(nely * 0.20)
        cockpit_y_end = int(nely * 0.80)
        cockpit_z_start = 2
        cockpit_z_end = nelz - 2
        fill_box(
            cockpit_void,
            range(cockpit_x_start, cockpit_x_end),
            range(cockpit_y_start, cockpit_y_end),
            range(cockpit_z_start, cockpit_z_end),
        )
        add_obstacle_region("cockpit_void", cockpit_void)

        # Simplified drivetrain tunnel running through the centerline of the cockpit volume.
        drivetrain_tunnel = empty_region()
        tunnel_half_width = max(1, nely // 12)
        centre_y = nely // 2
        fill_box(
            drivetrain_tunnel,
            range(max(1, nelx // 4), min(nelx - 1, int(nelx * 0.75))),
            range(max(0, centre_y - tunnel_half_width), min(nely, centre_y + tunnel_half_width + 1)),
            range(1, min(nelz, max(2, nelz // 2))),
        )
        add_obstacle_region("drivetrain_tunnel", drivetrain_tunnel)

        def add_wheel_arch(name: str, x_center: int, y_center: int) -> None:
            region = empty_region()
            rx = max(1, nelx // 14)
            ry = max(1, nely // 10)
            z_start = max(1, nelz // 3)
            for ix in range(max(0, x_center - rx), min(nelx, x_center + rx + 1)):
                for iy in range(max(0, y_center - ry), min(nely, y_center + ry + 1)):
                    for iz in range(z_start, nelz):
                        dx = (ix - x_center) / max(rx, 1)
                        dy = (iy - y_center) / max(ry, 1)
                        if dx * dx + dy * dy <= 1.0:
                            region[_elem_index(ix, iy, iz, nely, nelz)] = True
            add_obstacle_region(name, region)

        front_x = max(2, nelx // 10)
        rear_x = min(nelx - 3, int(nelx * 0.85))
        side_offset = max(1, nely // 8)
        add_wheel_arch("front_left_wheel_arch", front_x, side_offset)
        add_wheel_arch("front_right_wheel_arch", front_x, nely - 1 - side_offset)
        add_wheel_arch("rear_left_wheel_arch", rear_x, side_offset)
        add_wheel_arch("rear_right_wheel_arch", rear_x, nely - 1 - side_offset)

        # Final combined masks
        for name, region in self.obstacle_regions.items():
            self.obstacle_regions[name] = region & ~self.preserve
        self.obstacle = np.zeros(self.nel, dtype=bool)
        for region in self.obstacle_regions.values():
            self.obstacle |= region

        self.designable[self.preserve] = False
        self.designable[self.obstacle] = False
        self.x_init[self.preserve] = 1.0
        self.x_init[self.obstacle] = 0.001

    def _define_analysis_groups(self):
        """Define named load and support groups for case resolution."""
        pickups = self.pickup_nodes()
        z_load = min(1, self.nelz)

        self.load_node_groups = {
            "front_left_pickup": np.array([pickups["front_left"]], dtype=int),
            "front_right_pickup": np.array([pickups["front_right"]], dtype=int),
            "rear_left_pickup": np.array([pickups["rear_left"]], dtype=int),
            "rear_right_pickup": np.array([pickups["rear_right"]], dtype=int),
            "front_left_load": np.array([self.node(0, 0, z_load)], dtype=int),
            "front_right_load": np.array([self.node(0, self.nely, z_load)], dtype=int),
            "rear_left_load": np.array([self.node(self.nelx, 0, z_load)], dtype=int),
            "rear_right_load": np.array([self.node(self.nelx, self.nely, z_load)], dtype=int),
            "left_sill_top": np.array(
                [self.node(ix, 0, self.nelz) for ix in range(self.nelx + 1)],
                dtype=int,
            ),
            "right_sill_top": np.array(
                [self.node(ix, self.nely, self.nelz) for ix in range(self.nelx + 1)],
                dtype=int,
            ),
        }

        pickup_dofs = [
            3 * node + axis
            for node in pickups.values()
            for axis in range(3)
        ]
        rear_face_dofs = []
        for iy in range(self.nely + 1):
            for iz in range(self.nelz + 1):
                n = self.node(self.nelx, iy, iz)
                rear_face_dofs.extend([3 * n, 3 * n + 1, 3 * n + 2])

        self.support_dof_groups = {
            "pickup_points": np.array(sorted(set(pickup_dofs)), dtype=int),
            "rear_face": np.array(rear_face_dofs, dtype=int),
        }

    @property
    def non_designable(self) -> np.ndarray:
        """Elements locked out of optimisation for any reason."""
        return self.preserve | self.obstacle

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
        min_wall_elements = max(
            1,
            int(np.ceil(self.cfg.manufacturing.min_wall_thickness_mm / self.es)),
        )
        min_void_elements = max(
            1,
            int(np.ceil(self.cfg.manufacturing.min_lattice_cell_mm / self.es)),
        )
        simp = SIMPConfig(
            volume_fraction=self.cfg.simp.volume_fraction,
            max_iterations=self.cfg.simp.max_iterations,
            **self.MONOCOQUE_SIMP,
        )
        return Solver3D(
            self.nelx,
            self.nely,
            self.nelz,
            simp,
            min_wall_elements=min_wall_elements,
            min_void_elements=min_void_elements,
        )


@dataclass(frozen=True)
class ResolvedMonocoqueCase:
    """Geometry-resolved solver inputs for a named monocoque case."""

    name: str
    tub: MonocoqueTub
    fixed_dofs: np.ndarray
    forces: list[np.ndarray]
    weights: list[float]
    definition: SolverLoadCaseDefinition


AUTHORITATIVE_MONOCOQUE_CASES = ("torsion", "bending", "corner")
EXPLORATORY_MONOCOQUE_CASES = ("combined",)


def _pickup_dofs(tub: MonocoqueTub, names: list[str]) -> np.ndarray:
    if set(names) == {"front_left", "front_right", "rear_left", "rear_right"}:
        return tub.support_dof_groups["pickup_points"]
    pickups = tub.pickup_nodes()
    dofs = []
    for name in names:
        n = pickups[name]
        dofs.extend([3 * n, 3 * n + 1, 3 * n + 2])
    return np.array(sorted(set(dofs)), dtype=int)


def _rear_face_dofs(tub: MonocoqueTub) -> np.ndarray:
    return tub.support_dof_groups["rear_face"]


def _axis_index(axis: str) -> int:
    return {"x": 0, "y": 1, "z": 2}[axis]


def _node_group(tub: MonocoqueTub, name: str) -> np.ndarray:
    """Resolve a semantic load target into one or more node ids."""
    try:
        return tub.load_node_groups[name]
    except KeyError as exc:
        raise ValueError(f"Unknown load target group: {name}") from exc


def _support_dofs(tub: MonocoqueTub, support_set: str) -> np.ndarray:
    """Resolve a semantic support set into fixed DOFs."""
    try:
        return tub.support_dof_groups[support_set]
    except KeyError as exc:
        raise ValueError(f"Unknown support set: {support_set}") from exc


def _force_vector(tub: MonocoqueTub, ndof: int, load: PointLoadSpec | DistributedLoadSpec) -> np.ndarray:
    """Build a force vector for a structured load spec."""
    force = np.zeros(ndof)
    axis = _axis_index(load.axis)
    nodes = _node_group(tub, load.target)

    if isinstance(load, PointLoadSpec):
        if len(nodes) != 1:
            raise ValueError(f"Point load target {load.target} resolved to {len(nodes)} nodes")
        force[3 * nodes[0] + axis] = load.magnitude_n
        return force

    load_per_node = load.total_magnitude_n / len(nodes)
    for node in nodes:
        force[3 * node + axis] += load_per_node
    return force


def _solver_inputs(
    tub: MonocoqueTub,
    case: SolverLoadCaseDefinition,
) -> tuple[np.ndarray, list[np.ndarray], list[float]]:
    """Resolve a structured load-case definition into solver inputs."""
    solver = tub.build_solver()
    fixed_dofs = _support_dofs(tub, case.support_set)
    forces = []
    weights = []

    for subcase in case.subcases:
        force = np.zeros(solver.ndof)
        for load in subcase.loads:
            force += _force_vector(tub, solver.ndof, load)
        forces.append(force)
        weights.append(subcase.weight)

    return fixed_dofs, forces, weights


def resolve_monocoque_case(
    cfg: Config,
    case_name: str,
    element_size_mm: float = 50.0,
) -> ResolvedMonocoqueCase:
    """Resolve a named monocoque case into explicit solver inputs.

    This is primarily useful for regression tests and inspection of the
    authoritative case setup.
    """
    valid_cases = AUTHORITATIVE_MONOCOQUE_CASES + EXPLORATORY_MONOCOQUE_CASES
    if case_name not in valid_cases:
        raise ValueError(f"Unknown monocoque case '{case_name}'. Expected one of {valid_cases}.")

    tub = MonocoqueTub(cfg, element_size_mm)
    definitions = build_monocoque_load_definitions(cfg)
    case = getattr(definitions, case_name)
    fixed_dofs, forces, weights = _solver_inputs(tub, case)
    return ResolvedMonocoqueCase(
        name=case.name,
        tub=tub,
        fixed_dofs=fixed_dofs,
        forces=forces,
        weights=weights,
        definition=case,
    )


def monocoque_case_kind(case_name: str) -> str:
    """Classify a monocoque case as authoritative or exploratory."""
    if case_name in AUTHORITATIVE_MONOCOQUE_CASES:
        return "authoritative"
    if case_name in EXPLORATORY_MONOCOQUE_CASES:
        return "exploratory"
    raise ValueError(f"Unknown monocoque case '{case_name}'")


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
    resolved = resolve_monocoque_case(cfg, "torsion", element_size_mm)
    densities, history = _solve_load_case(
        resolved.tub,
        resolved.fixed_dofs,
        resolved.forces[0],
        on_iteration=on_iteration,
    )
    return densities, history, resolved.tub


def bending_load_case(
    cfg: Config,
    element_size_mm: float = 50.0,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float], MonocoqueTub]:
    """Bending: distributed vertical load along sills, fixed at all four pickups.

    Per spec: vehicle weight at 1g distributed along sill, all four
    pickup points fixed.
    """
    resolved = resolve_monocoque_case(cfg, "bending", element_size_mm)
    densities, history = _solve_load_case(
        resolved.tub,
        resolved.fixed_dofs,
        resolved.forces[0],
        on_iteration=on_iteration,
    )
    return densities, history, resolved.tub


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
    resolved = resolve_monocoque_case(cfg, "corner", element_size_mm)
    densities, history = _solve_load_case(
        resolved.tub,
        resolved.fixed_dofs,
        resolved.forces,
        weights=resolved.weights,
        on_iteration=on_iteration,
    )
    return densities, history, resolved.tub


def combined_load_case(
    cfg: Config,
    element_size_mm: float = 50.0,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float], MonocoqueTub]:
    """All load cases combined with spec objective weights.

    Torsion: 0.50, Bending: 0.20, Front corners: 0.20, Rear corners: 0.10.

    Exploratory case only.

    All cases share a single BC set: rear face fixed. This is correct for
    torsion and conservative for bending/corners (rear face provides more
    constraint than four point pickups). Use the per-case authoritative
    entry points for reference structural analysis.
    """
    resolved = resolve_monocoque_case(cfg, "combined", element_size_mm)
    densities, history = _solve_load_case(
        resolved.tub,
        resolved.fixed_dofs,
        resolved.forces,
        weights=resolved.weights,
        on_iteration=on_iteration,
    )
    return densities, history, resolved.tub


def _run_and_save(name, densities, history, tub, snapshot_paths: list[str] | None = None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from car_solver.export import (
        export_thresholded_density_stl,
        export_thresholded_density_vtk,
    )
    from car_solver.output import (
        build_monocoque_run_summary,
        build_run_manifest,
        build_monocoque_constraint_report,
        output_path,
        write_constraint_report,
        write_run_manifest,
        write_run_summary,
    )
    from car_solver.visualise3d import plot_density_3d

    print(f"Grid: {tub.nelx}x{tub.nely}x{tub.nelz} ({tub.nel:,} elements)")
    print(f"Converged in {len(history)} iterations")
    print(f"Final compliance: {history[-1]:.4f}")

    constraint_report = build_monocoque_constraint_report(name, tub, densities)
    for entry in constraint_report.constraints:
        status = "active" if entry.active else "inactive"
        print(
            f"Constraint {entry.name}: {status}, "
            f"{entry.parameter_mm:.1f} mm ({entry.parameter_elements} el), "
            f"remaining violating elements={entry.violating_elements}"
        )
    report_path = write_constraint_report(constraint_report)
    print(f"Saved {report_path}")

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(history, "b-", linewidth=1.5)
    ax.set_xlabel("Iteration")
    ax.set_ylabel("Compliance")
    ax.set_title(f"Monocoque {name.title()} Convergence")
    ax.grid(True, alpha=0.3)
    convergence_path = output_path(f"{name}_convergence.png")
    fig.savefig(convergence_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {convergence_path}")

    vtk_path = export_thresholded_density_vtk(
        densities,
        tub.nelx,
        tub.nely,
        tub.nelz,
        output_path(f"{name}_density.vtk"),
        threshold=0.25,
    )
    print(f"Saved {vtk_path}")

    stl_path = export_thresholded_density_stl(
        densities,
        tub.nelx,
        tub.nely,
        tub.nelz,
        output_path(f"{name}_density.stl"),
        threshold=0.25,
    )
    print(f"Saved {stl_path}")

    png_path = output_path(f"{name}_density.png")

    try:
        plot_density_3d(
            densities, tub.nelx, tub.nely, tub.nelz,
            png_path, threshold=0.25,
        )
    except Exception as e:
        print(f"PNG render skipped: {e}")
        png_path = ""

    summary = build_monocoque_run_summary(
        name,
        monocoque_case_kind(name),
        tub,
        densities,
        history,
        artifacts={
            "constraint_report": report_path,
            "convergence_plot": convergence_path,
            "density_vtk": vtk_path,
            "density_stl": stl_path,
            **({"density_png": png_path} if png_path else {}),
        },
    )
    summary_path = write_run_summary(summary)
    print(f"Saved {summary_path}")

    manifest_artifacts: dict[str, object] = {
        **summary.artifacts,
        "summary": summary_path,
    }
    if snapshot_paths:
        manifest_artifacts["iteration_snapshots"] = snapshot_paths
    manifest = build_run_manifest(
        case_name=name,
        case_kind=monocoque_case_kind(name),
        entry_point="car_solver.monocoque.__main__",
        element_size_mm=tub.es,
        config_snapshot=summary.config_snapshot,
        artifacts=manifest_artifacts,
    )
    manifest_path = write_run_manifest(manifest)
    print(f"Saved {manifest_path}")


if __name__ == "__main__":
    import os
    import sys
    from car_solver.config import load_config
    from car_solver.visualise3d import make_iteration_snapshot_callback

    cfg = load_config()
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    snapshot_every = int(os.getenv("SNAPSHOT_EVERY", "0") or "0")

    def base_on_iter(it, densities, compliance, change):
        if it % 10 == 0:
            print(f"  Iteration {it:3d}: compliance={compliance:.4f}, change={change:.4f}")

    def run_case(name, label, solve_fn):
        snapshot_paths: list[str] = []
        on_iter = base_on_iter
        if snapshot_every > 0:
            preview_tub = MonocoqueTub(cfg)
            on_iter, snapshot_paths = make_iteration_snapshot_callback(
                nelx=preview_tub.nelx,
                nely=preview_tub.nely,
                nelz=preview_tub.nelz,
                filename_prefix=f"{name}_frame",
                every=snapshot_every,
                threshold=0.25,
                base_callback=base_on_iter,
            )
        print(label)
        d, h, tub = solve_fn(cfg, on_iteration=on_iter)
        _run_and_save(name, d, h, tub, snapshot_paths=snapshot_paths)

    if mode in ("all", "torsion"):
        run_case("torsion", "Running monocoque torsion [authoritative]...", torsion_load_case)

    if mode in ("all", "bending"):
        run_case("bending", "\nRunning monocoque bending [authoritative]...", bending_load_case)

    if mode in ("all", "corner"):
        run_case("corner", "\nRunning monocoque corner loads [authoritative]...", corner_load_case)

    if mode in ("all", "combined"):
        run_case(
            "combined",
            "\nRunning monocoque combined (all load cases) [exploratory]...",
            combined_load_case,
        )
