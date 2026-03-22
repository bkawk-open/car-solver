"""2D SIMP topology optimisation solver.

Solves the minimum compliance problem on a 2D rectangular domain using
Q4 (4-node quadrilateral) finite elements and the SIMP material model.

Supports multiple load cases with weighted compliance objective,
obstacle regions (forced void), and preserved regions (forced solid).
"""

from dataclasses import dataclass
from time import perf_counter

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix

from car_solver.config import Config, SIMPConfig
from car_solver.linalg_backend import LinearAlgebraBackend, SciPySparseBackend


@dataclass(frozen=True)
class SolverTiming:
    iterations: int
    assembly_seconds: float
    factorization_seconds: float
    solve_seconds: float
    sensitivity_filter_seconds: float
    update_seconds: float
    total_seconds: float


def element_stiffness(nu: float) -> np.ndarray:
    """8x8 stiffness matrix for a unit-size Q4 plane stress element
    with unit Young's modulus.

    Analytically integrated (exact for unit square, uniform material).
    """
    k = np.array([
        1/2 - nu/6, 1/8 + nu/8, -1/4 - nu/12, 3/8 - nu/8,
        -1/4 + nu/12, -1/8 - nu/8, nu/6, -3/8 + nu/8,
    ])
    KE = 1.0 / (1 - nu**2) * np.array([
        [k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7]],
        [k[1], k[0], k[7], k[6], k[5], k[4], k[3], k[2]],
        [k[2], k[7], k[0], k[5], k[6], k[3], k[4], k[1]],
        [k[3], k[6], k[5], k[0], k[7], k[2], k[1], k[4]],
        [k[4], k[5], k[6], k[7], k[0], k[1], k[2], k[3]],
        [k[5], k[4], k[3], k[2], k[1], k[0], k[7], k[6]],
        [k[6], k[3], k[4], k[1], k[2], k[7], k[0], k[5]],
        [k[7], k[2], k[1], k[4], k[3], k[6], k[5], k[0]],
    ])
    return KE


def density_filter(nelx: int, nely: int, rmin: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build density filter weights for sensitivity smoothing.

    Returns sparse matrix components (rows, cols, weights) that define
    the convolution kernel centred on each element within radius rmin.
    """
    nfilter = nelx * nely * ((2 * int(np.ceil(rmin)) + 1) ** 2)
    iH = np.zeros(nfilter, dtype=int)
    jH = np.zeros(nfilter, dtype=int)
    sH = np.zeros(nfilter)
    cc = 0
    for i in range(nelx):
        for j in range(nely):
            row = i * nely + j
            imin = max(i - int(np.ceil(rmin)) + 1, 0)
            imax = min(i + int(np.ceil(rmin)), nelx)
            jmin = max(j - int(np.ceil(rmin)) + 1, 0)
            jmax = min(j + int(np.ceil(rmin)), nely)
            for ii in range(imin, imax):
                for jj in range(jmin, jmax):
                    col = ii * nely + jj
                    dist = max(0, rmin - np.sqrt((i - ii)**2 + (j - jj)**2))
                    if dist > 0:
                        iH[cc] = row
                        jH[cc] = col
                        sH[cc] = dist
                        cc += 1
    iH = iH[:cc]
    jH = jH[:cc]
    sH = sH[:cc]
    return iH, jH, sH


def enforce_min_wall_thickness_2d(
    densities: np.ndarray,
    nelx: int,
    nely: int,
    min_wall_elements: int,
    active: np.ndarray | None = None,
    threshold: float = 0.5,
) -> np.ndarray:
    """Suppress solid features thinner than the requested element width.

    This is a first-pass manufacturability constraint. It removes thin
    solid regions by applying a binary opening to thresholded densities.
    """
    if min_wall_elements <= 1:
        return np.zeros_like(densities, dtype=bool)

    if active is None:
        active = np.ones_like(densities, dtype=bool)

    grid = densities.reshape(nelx, nely)
    active_grid = active.reshape(nelx, nely)
    solid = (grid >= threshold) & active_grid
    structure = np.ones((min_wall_elements, min_wall_elements), dtype=bool)
    opened = ndimage.binary_opening(solid, structure=structure)
    removed = solid & ~opened
    return removed.ravel()


def enforce_min_lattice_cell_size_2d(
    densities: np.ndarray,
    nelx: int,
    nely: int,
    min_void_elements: int,
    active: np.ndarray | None = None,
    threshold: float = 0.5,
) -> np.ndarray:
    """Fill void features smaller than the requested element width."""
    if min_void_elements <= 1:
        return np.zeros_like(densities, dtype=bool)

    if active is None:
        active = np.ones_like(densities, dtype=bool)

    grid = densities.reshape(nelx, nely)
    active_grid = active.reshape(nelx, nely)
    void = (grid < threshold) & active_grid
    structure = np.ones((min_void_elements, min_void_elements), dtype=bool)
    opened = ndimage.binary_opening(void, structure=structure)
    filled = void & ~opened
    return filled.ravel()


class Solver2D:
    """2D SIMP topology optimisation on a rectangular domain."""

    def __init__(
        self,
        nelx: int,
        nely: int,
        simp: SIMPConfig,
        Emin: float = 1e-9,
        nu: float = 0.3,
        min_wall_elements: int = 1,
        min_void_elements: int = 1,
        backend: LinearAlgebraBackend | None = None,
    ):
        self.nelx = nelx
        self.nely = nely
        self.penalty = simp.penalty
        self.volfrac = simp.volume_fraction
        self.tol = simp.convergence_tolerance
        self.max_iter = simp.max_iterations
        self.rmin = simp.filter_radius
        self.Emin = Emin
        self.E0 = 1.0
        self.nu = nu
        self.min_wall_elements = max(1, int(min_wall_elements))
        self.min_void_elements = max(1, int(min_void_elements))
        self.backend = backend or SciPySparseBackend()

        self.nel = nelx * nely
        self.ndof = 2 * (nelx + 1) * (nely + 1)
        self.KE = element_stiffness(nu)

        # Build filter
        iH, jH, sH = density_filter(nelx, nely, self.rmin)
        self.H = coo_matrix((sH, (iH, jH)), shape=(self.nel, self.nel)).tocsc()
        self.Hs = np.array(self.H.sum(axis=1)).flatten()

        # DOF connectivity per element (vectorised)
        self.edofMat = self._build_edof_mat()

        # Precompute sparse assembly indices
        iK = np.kron(self.edofMat, np.ones((8, 1), dtype=int)).flatten()
        jK = np.kron(self.edofMat, np.ones((1, 8), dtype=int)).flatten()
        self.iK = iK
        self.jK = jK

        # Precompute flattened KE for vectorised assembly
        self.KE_flat = self.KE.flatten()
        self.last_timing: SolverTiming | None = None

    def _build_edof_mat(self) -> np.ndarray:
        ix, iy = np.meshgrid(np.arange(self.nelx), np.arange(self.nely), indexing="ij")
        ix = ix.flatten()
        iy = iy.flatten()
        n1 = ix * (self.nely + 1) + iy
        n2 = (ix + 1) * (self.nely + 1) + iy
        edof = np.column_stack([
            2*n1, 2*n1+1, 2*n2, 2*n2+1,
            2*n2+2, 2*n2+3, 2*n1+2, 2*n1+3,
        ])
        return edof

    def _element_compliance_vectorised(self, u: np.ndarray) -> np.ndarray:
        """Compute element compliance ce = ue^T @ KE @ ue for all elements."""
        # Gather element displacements: (nel, 8)
        ue = u[self.edofMat]
        # KE @ ue for all elements: (nel, 8)
        Kue = ue @ self.KE
        # ce = sum(ue * Kue, axis=1): (nel,)
        return np.sum(ue * Kue, axis=1)

    def solve(
        self,
        fixed_dofs: np.ndarray,
        forces: np.ndarray | list[np.ndarray],
        weights: list[float] | None = None,
        designable: np.ndarray | None = None,
        obstacle: np.ndarray | None = None,
        x_init: np.ndarray | None = None,
        on_iteration: callable = None,
        collect_timing: bool = False,
    ) -> tuple[np.ndarray, list[float]]:
        """Run SIMP optimisation.

        Args:
            fixed_dofs: array of DOF indices with zero displacement.
            forces: single force vector (ndof,) or list of force vectors
                for multi-load-case optimisation.
            weights: weight per load case for compliance sum. Must match
                length of forces list. Defaults to equal weights.
            designable: boolean mask per element. Non-designable elements
                stay at their initial density. Defaults to all designable.
            obstacle: boolean mask per element. Obstacle elements are forced
                to minimum density (void) every iteration. Obstacles must
                also be marked non-designable.
            x_init: initial density per element. Defaults to uniform at
                volume fraction. Obstacle elements are forced to Emin
                regardless of x_init.
            on_iteration: callback(iteration, densities, compliance, change)
                called after each iteration for live visualisation.

        Returns:
            (densities, compliance_history) where densities is (nelx*nely,)
            array of final element densities.
        """
        nel = self.nel

        # Normalise forces to list + weights
        if isinstance(forces, np.ndarray) and forces.ndim == 1:
            force_list = [forces]
        else:
            force_list = list(forces)
        n_cases = len(force_list)

        if weights is None:
            weights = [1.0 / n_cases] * n_cases
        if len(weights) != n_cases:
            raise ValueError(f"Got {len(weights)} weights for {n_cases} load cases")

        # Element masks
        if designable is None:
            designable = np.ones(nel, dtype=bool)
        if obstacle is None:
            obstacle = np.zeros(nel, dtype=bool)

        # Initial densities
        if x_init is not None:
            x = x_init.copy()
        else:
            x = np.full(nel, self.volfrac)
        x[obstacle] = 0.001
        xphys = np.array(self.H @ x / self.Hs).flatten()
        xphys[obstacle] = 0.001

        free_dofs = np.setdiff1d(np.arange(self.ndof), fixed_dofs)

        # Mask for elements that participate in sensitivity calculation
        active = designable & ~obstacle

        removed = enforce_min_wall_thickness_2d(
            xphys, self.nelx, self.nely, self.min_wall_elements, active=active,
        )
        x[removed] = 0.001
        xphys[removed] = 0.001
        filled = enforce_min_lattice_cell_size_2d(
            xphys, self.nelx, self.nely, self.min_void_elements, active=active,
        )
        x[filled] = 1.0
        xphys[filled] = 1.0

        compliance_history = []
        change = 1.0
        timing = {
            "assembly_seconds": 0.0,
            "factorization_seconds": 0.0,
            "solve_seconds": 0.0,
            "sensitivity_filter_seconds": 0.0,
            "update_seconds": 0.0,
        }
        total_start = perf_counter()

        for iteration in range(self.max_iter):
            if change < self.tol and iteration > 1:
                break

            # Effective modulus per element
            E_eff = self.Emin + xphys**self.penalty * (self.E0 - self.Emin)

            # Assemble global stiffness (vectorised)
            phase_start = perf_counter()
            sK = (self.KE_flat[np.newaxis].T * E_eff).flatten(order="F")
            K = self.backend.assemble(sK, self.iK, self.jK, (self.ndof, self.ndof))
            if collect_timing:
                timing["assembly_seconds"] += perf_counter() - phase_start

            # Factor once, solve for all load cases
            phase_start = perf_counter()
            lu = self.backend.factorize_free_matrix(K, free_dofs)
            if collect_timing:
                timing["factorization_seconds"] += perf_counter() - phase_start

            # Accumulate weighted compliance and sensitivity across load cases
            total_compliance = 0.0
            dc = np.zeros(nel)

            phase_start = perf_counter()
            for f_vec, w in zip(force_list, weights):
                u = np.zeros(self.ndof)
                u[free_dofs] = lu.solve(f_vec[free_dofs])

                # Element compliance (vectorised)
                ce = self._element_compliance_vectorised(u)

                total_compliance += w * np.sum(E_eff * ce)

                # Sensitivity (only for active elements)
                dc[active] += w * (
                    -self.penalty
                    * xphys[active] ** (self.penalty - 1)
                    * (self.E0 - self.Emin)
                    * ce[active]
                )
            if collect_timing:
                timing["solve_seconds"] += perf_counter() - phase_start

            compliance_history.append(total_compliance)

            dv = np.zeros(nel)
            dv[active] = 1.0

            # Apply density filter to sensitivities
            phase_start = perf_counter()
            dc = np.array(self.H @ (dc * xphys / self.Hs)).flatten()
            dv = np.array(self.H @ (dv * xphys / self.Hs)).flatten()
            if collect_timing:
                timing["sensitivity_filter_seconds"] += perf_counter() - phase_start

            # Optimality criteria update
            phase_start = perf_counter()
            xold = x.copy()
            x = self._oc_update(x, dc, dv, designable)
            x[obstacle] = 0.001

            # Filter densities
            xphys = np.array(self.H @ x / self.Hs).flatten()
            xphys[obstacle] = 0.001

            removed = enforce_min_wall_thickness_2d(
                xphys, self.nelx, self.nely, self.min_wall_elements, active=active,
            )
            x[removed] = 0.001
            xphys[removed] = 0.001
            filled = enforce_min_lattice_cell_size_2d(
                xphys, self.nelx, self.nely, self.min_void_elements, active=active,
            )
            x[filled] = 1.0
            xphys[filled] = 1.0
            change = np.max(np.abs(x - xold))
            if collect_timing:
                timing["update_seconds"] += perf_counter() - phase_start

            if on_iteration:
                on_iteration(iteration, xphys.copy(), total_compliance, change)

        self.last_timing = (
            SolverTiming(
                iterations=len(compliance_history),
                assembly_seconds=timing["assembly_seconds"],
                factorization_seconds=timing["factorization_seconds"],
                solve_seconds=timing["solve_seconds"],
                sensitivity_filter_seconds=timing["sensitivity_filter_seconds"],
                update_seconds=timing["update_seconds"],
                total_seconds=perf_counter() - total_start,
            )
            if collect_timing
            else None
        )
        return xphys, compliance_history

    def _oc_update(
        self,
        x: np.ndarray,
        dc: np.ndarray,
        dv: np.ndarray,
        designable: np.ndarray,
        move: float = 0.2,
    ) -> np.ndarray:
        """Optimality criteria update with bisection on volume constraint."""
        l1, l2 = 1e-12, 1e9

        # For non-active elements dc=0 and dv=0, so ratio is irrelevant.
        # Clamp dc to small negative to keep sqrt well-defined.
        dc_safe = np.where(dc < 0, dc, -1e-20)

        while (l2 - l1) / (l1 + l2) > 1e-3:
            lmid = 0.5 * (l2 + l1)
            ratio = np.sqrt(-dc_safe / (dv * lmid + 1e-12))
            xnew = np.maximum(
                0.001,
                np.maximum(
                    x - move,
                    np.minimum(1.0, np.minimum(x + move, x * ratio)),
                ),
            )
            xnew[~designable] = x[~designable]

            if np.sum(xnew[designable]) > self.volfrac * np.sum(designable):
                l1 = lmid
            else:
                l2 = lmid

        return xnew


def cantilever_beam(
    nelx: int = 120,
    nely: int = 40,
    simp: SIMPConfig | None = None,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float]]:
    """Classic cantilever beam test case.

    Left edge fully fixed, point load downward at mid-right edge.
    Known correct result: truss-like structure with diagonal members.
    """
    if simp is None:
        simp = SIMPConfig(
            penalty=3.0,
            volume_fraction=0.30,
            convergence_tolerance=0.01,
            max_iterations=200,
            filter_radius=1.5,
        )

    solver = Solver2D(nelx, nely, simp)

    fixed_nodes = np.arange(nely + 1)
    fixed_dofs = np.union1d(2 * fixed_nodes, 2 * fixed_nodes + 1)

    force = np.zeros(solver.ndof)
    load_node = (nelx + 1) * (nely + 1) - 1 - nely // 2
    force[2 * load_node + 1] = -1.0

    return solver.solve(fixed_dofs, force, on_iteration=on_iteration)


def mbb_beam(
    nelx: int = 180,
    nely: int = 60,
    simp: SIMPConfig | None = None,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float]]:
    """MBB beam - half-beam with symmetry.

    Left edge x-fixed (symmetry), roller at bottom-right corner.
    Top-left point load. Known correct result: arch with vertical members.
    """
    if simp is None:
        simp = SIMPConfig(
            penalty=3.0,
            volume_fraction=0.30,
            convergence_tolerance=0.01,
            max_iterations=200,
            filter_radius=1.5,
        )

    solver = Solver2D(nelx, nely, simp)

    # Symmetry: fix x-displacement on left edge
    left_nodes = np.arange(nely + 1)
    sym_dofs = 2 * left_nodes  # x-DOFs only

    # Pin bottom-right corner (x+y). In the full beam this node is at
    # mid-span where x-displacement is zero by symmetry, so pinning x
    # is physically correct and eliminates the zero-energy mode.
    bottom_right_node = (nelx + 1) * (nely + 1) - 1
    pin_dofs = np.array([2 * bottom_right_node, 2 * bottom_right_node + 1])

    fixed_dofs = np.union1d(sym_dofs, pin_dofs)

    # Point load downward at top-left corner
    force = np.zeros(solver.ndof)
    force[1] = -1.0  # node 0, y-DOF

    return solver.solve(fixed_dofs, force, on_iteration=on_iteration)


def plate_with_hole(
    nelx: int = 80,
    nely: int = 80,
    hole_radius: float = 0.25,
    simp: SIMPConfig | None = None,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float]]:
    """Plate with central circular hole under biaxial tension.

    Quarter-plate with symmetry BCs. The obstacle region (hole) is forced
    void. Known result: material concentrates around the hole edges at
    the stress concentration points.
    Validates obstacle masking with multi-load-case.

    Args:
        hole_radius: radius as fraction of plate half-width (0-1).
    """
    if simp is None:
        simp = SIMPConfig(
            penalty=3.0,
            volume_fraction=0.40,
            convergence_tolerance=0.01,
            max_iterations=200,
            filter_radius=1.5,
        )

    # Higher Emin prevents near-singular stiffness in the hole region
    # where many adjacent elements are at minimum density
    solver = Solver2D(nelx, nely, simp, Emin=1e-3)

    # Mark circular hole as obstacle
    nel = solver.nel
    obstacle = np.zeros(nel, dtype=bool)
    designable = np.ones(nel, dtype=bool)
    cx, cy = 0.0, 0.0  # hole centre at origin (symmetry corner)
    r = hole_radius * nelx

    for i in range(nelx):
        for j in range(nely):
            ex, ey = i + 0.5, j + 0.5
            if np.sqrt((ex - cx)**2 + (ey - cy)**2) < r:
                el = i * nely + j
                obstacle[el] = True
                designable[el] = False

    # Symmetry BCs: fix x on left edge, fix y on bottom edge
    left_nodes = np.arange(nely + 1)
    bottom_nodes = np.array([i * (nely + 1) + nely for i in range(nelx + 1)])

    sym_x = 2 * left_nodes
    sym_y = 2 * bottom_nodes + 1

    # Pin origin node (top-left corner) in y to eliminate zero-energy
    # rotation mode. This node already has x fixed from the left edge.
    origin_y = np.array([1])  # node 0, y-DOF

    fixed_dofs = np.union1d(np.union1d(sym_x, sym_y), origin_y)

    # Two load cases for biaxial tension
    right_nodes = np.array([nelx * (nely + 1) + j for j in range(nely + 1)])
    top_nodes = np.array([i * (nely + 1) for i in range(nelx + 1)])

    f1 = np.zeros(solver.ndof)
    load_per_node = 1.0 / len(right_nodes)
    for n in right_nodes:
        f1[2 * n] = load_per_node

    f2 = np.zeros(solver.ndof)
    load_per_node = 1.0 / len(top_nodes)
    for n in top_nodes:
        f2[2 * n + 1] = -load_per_node

    return solver.solve(
        fixed_dofs,
        forces=[f1, f2],
        weights=[0.5, 0.5],
        designable=designable,
        obstacle=obstacle,
        on_iteration=on_iteration,
    )


def monocoque_cross_section(
    cfg: Config,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float], int, int]:
    """Monocoque tub cross-section optimisation using real vehicle loads.

    Models a transverse slice through the tub viewed from the front.
    The domain represents one half (symmetric about centreline).

    Uses multi-load-case objective with vertical and lateral loads
    as separate cases, weighted by the config objective weights.

    Returns (densities, history, nelx, nely) for plotting.
    """
    half_width_mm = cfg.geometry.front_track_mm / 2
    tub_height_mm = cfg.geometry.overall_height_mm * 0.40

    element_size_mm = 10.0
    nelx = int(half_width_mm / element_size_mm)
    nely = int(tub_height_mm / element_size_mm)

    solver = Solver2D(nelx, nely, cfg.simp)
    nel = solver.nel

    def elem(ix, iy):
        return ix * nely + iy

    # --- Element masks ---
    designable = np.ones(nel, dtype=bool)
    obstacle = np.zeros(nel, dtype=bool)

    floor_thickness = 3
    for i in range(nelx):
        for j in range(nely - floor_thickness, nely):
            designable[elem(i, j)] = False

    scuttle_thickness = 3
    for i in range(nelx):
        for j in range(scuttle_thickness):
            designable[elem(i, j)] = False

    cockpit_x_end = int(nelx * 0.60)
    cockpit_y_start = scuttle_thickness + 2
    cockpit_y_end = nely - floor_thickness - 2
    for i in range(0, cockpit_x_end):
        for j in range(cockpit_y_start, cockpit_y_end):
            obstacle[elem(i, j)] = True
            designable[elem(i, j)] = False

    x_init = np.full(nel, cfg.simp.volume_fraction)
    x_init[obstacle] = 0.001
    x_init[~designable & ~obstacle] = 1.0

    # --- Boundary conditions ---
    left_nodes = np.arange(nely + 1)
    sym_dofs = 2 * left_nodes

    top_nodes = np.array([i * (nely + 1) for i in range(nelx + 1)])
    top_y_dofs = 2 * top_nodes + 1

    pin_dofs = np.array([2 * nely, 2 * nely + 1])

    fixed_dofs = np.union1d(np.union1d(sym_dofs, top_y_dofs), pin_dofs)

    # --- Load cases ---
    from car_solver.loads import calculate_load_cases

    cases = calculate_load_cases(cfg)
    pickup_node = (nelx + 1) * (nely + 1) - 1

    f_vertical = np.zeros(solver.ndof)
    f_vertical[2 * pickup_node + 1] = -1.0

    f_lateral = np.zeros(solver.ndof)
    f_lateral[2 * pickup_node] = 1.0

    lat_ratio = cases.front_left_dynamic.lateral_n / cases.front_left_dynamic.vertical_n

    densities, history = solver.solve(
        fixed_dofs,
        forces=[f_vertical, f_lateral],
        weights=[1.0 / (1.0 + lat_ratio), lat_ratio / (1.0 + lat_ratio)],
        designable=designable,
        obstacle=obstacle,
        x_init=x_init,
        on_iteration=on_iteration,
    )
    return densities, history, nelx, nely
