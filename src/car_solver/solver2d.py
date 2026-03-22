"""2D SIMP topology optimisation solver.

Solves the minimum compliance problem on a 2D rectangular domain using
Q4 (4-node quadrilateral) finite elements and the SIMP material model.

Supports multiple load cases with weighted compliance objective,
obstacle regions (forced void), and preserved regions (forced solid).
"""

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import splu

from car_solver.config import Config, SIMPConfig


def element_stiffness(E: float, nu: float) -> np.ndarray:
    """8x8 stiffness matrix for a unit-size Q4 plane stress element.

    Analytically integrated (exact for unit square, uniform material).
    """
    k = np.array([
        1/2 - nu/6, 1/8 + nu/8, -1/4 - nu/12, 3/8 - nu/8,
        -1/4 + nu/12, -1/8 - nu/8, nu/6, -3/8 + nu/8,
    ])
    KE = E / (1 - nu**2) * np.array([
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


class Solver2D:
    """2D SIMP topology optimisation on a rectangular domain."""

    def __init__(
        self,
        nelx: int,
        nely: int,
        simp: SIMPConfig,
        E0: float = 1.0,
        Emin: float = 1e-9,
        nu: float = 0.3,
    ):
        self.nelx = nelx
        self.nely = nely
        self.penalty = simp.penalty
        self.volfrac = simp.volume_fraction
        self.tol = simp.convergence_tolerance
        self.max_iter = simp.max_iterations
        self.rmin = simp.filter_radius
        self.E0 = E0
        self.Emin = Emin
        self.nu = nu

        self.ndof = 2 * (nelx + 1) * (nely + 1)
        self.KE = element_stiffness(1.0, nu)

        # Build filter
        iH, jH, sH = density_filter(nelx, nely, self.rmin)
        self.H = coo_matrix((sH, (iH, jH)), shape=(nelx * nely, nelx * nely)).tocsc()
        self.Hs = np.array(self.H.sum(axis=1)).flatten()

        # DOF connectivity per element
        self.edofMat = np.zeros((nelx * nely, 8), dtype=int)
        for i in range(nelx):
            for j in range(nely):
                el = i * nely + j
                n1 = i * (nely + 1) + j
                n2 = (i + 1) * (nely + 1) + j
                self.edofMat[el] = [
                    2*n1, 2*n1+1, 2*n2, 2*n2+1,
                    2*n2+2, 2*n2+3, 2*n1+2, 2*n1+3,
                ]

        # Precompute sparse assembly indices
        iK = np.kron(self.edofMat, np.ones((8, 1), dtype=int)).flatten()
        jK = np.kron(self.edofMat, np.ones((1, 8), dtype=int)).flatten()
        self.iK = iK
        self.jK = jK

    def solve(
        self,
        fixed_dofs: np.ndarray,
        forces: np.ndarray | list[np.ndarray],
        weights: list[float] | None = None,
        designable: np.ndarray | None = None,
        obstacle: np.ndarray | None = None,
        x_init: np.ndarray | None = None,
        on_iteration: callable = None,
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
        nel = self.nelx * self.nely

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

        compliance_history = []
        change = 1.0

        for iteration in range(self.max_iter):
            if change < self.tol and iteration > 1:
                break

            # Assemble global stiffness
            sK = (
                (self.KE.flatten()[np.newaxis]).T
                * (self.Emin + xphys**self.penalty * (self.E0 - self.Emin))
            ).flatten(order="F")
            K = coo_matrix((sK, (self.iK, self.jK)), shape=(self.ndof, self.ndof)).tocsc()

            # Factor once, solve for all load cases
            K_free = K[free_dofs, :][:, free_dofs]
            lu = splu(K_free.tocsc())

            # Accumulate weighted compliance and sensitivity across load cases
            total_compliance = 0.0
            dc = np.zeros(nel)
            ce_combined = np.zeros(nel)

            for f_vec, w in zip(force_list, weights):
                u = np.zeros(self.ndof)
                u[free_dofs] = lu.solve(f_vec[free_dofs])

                # Element compliance for this load case
                ce = np.zeros(nel)
                for el in range(nel):
                    ue = u[self.edofMat[el]]
                    ce[el] = ue @ self.KE @ ue

                E_eff = self.Emin + xphys**self.penalty * (self.E0 - self.Emin)
                case_compliance = np.sum(E_eff * ce)
                total_compliance += w * case_compliance

                # Sensitivity for this load case (only for active elements)
                dc_case = np.zeros(nel)
                dc_case[active] = (
                    -self.penalty
                    * xphys[active] ** (self.penalty - 1)
                    * (self.E0 - self.Emin)
                    * ce[active]
                )
                dc += w * dc_case
                ce_combined += w * ce

            compliance_history.append(total_compliance)

            dv = np.zeros(nel)
            dv[active] = 1.0

            # Apply density filter to sensitivities
            dc = np.array(self.H @ (dc * xphys / self.Hs)).flatten()
            dv = np.array(self.H @ (dv * xphys / self.Hs)).flatten()

            # Optimality criteria update
            xold = x.copy()
            x = self._oc_update(x, dc, dv, designable)
            x[obstacle] = 0.001

            # Filter densities
            xphys = np.array(self.H @ x / self.Hs).flatten()
            xphys[obstacle] = 0.001
            change = np.max(np.abs(x - xold))

            if on_iteration:
                on_iteration(iteration, xphys.copy(), total_compliance, change)

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
        nel = len(x)
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

            if np.sum(xnew) > self.volfrac * nel:
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
    """MBB beam - full beam, simply supported.

    Pin support at bottom-left, roller at bottom-right (support pads for
    numerical stability). Central top load. Known correct result: arch
    with diagonal members.
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

    pad = max(3, nelx // 30)

    left_pad_nodes = np.array([i * (nely + 1) + nely for i in range(pad)])
    left_dofs = np.union1d(2 * left_pad_nodes, 2 * left_pad_nodes + 1)

    right_pad_nodes = np.array([(nelx - i) * (nely + 1) + nely for i in range(pad)])
    right_dofs = 2 * right_pad_nodes + 1

    fixed_dofs = np.union1d(left_dofs, right_dofs)

    mid_top_node = (nelx // 2) * (nely + 1)
    force = np.zeros(solver.ndof)
    force[2 * mid_top_node + 1] = -1.0

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
    the stress concentration points (top/bottom of hole in each axis).
    Validates obstacle masking and stress-driven material placement.

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

    solver = Solver2D(nelx, nely, simp)

    # Mark circular hole as obstacle
    nel = nelx * nely
    obstacle = np.zeros(nel, dtype=bool)
    designable = np.ones(nel, dtype=bool)
    cx, cy = 0.0, 0.0  # hole centre at bottom-left (symmetry corner)
    r = hole_radius * nelx

    for i in range(nelx):
        for j in range(nely):
            ex, ey = i + 0.5, j + 0.5  # element centre
            if np.sqrt((ex - cx)**2 + (ey - cy)**2) < r:
                el = i * nely + j
                obstacle[el] = True
                designable[el] = False

    # Symmetry BCs: fix x on left edge, fix y on bottom edge
    left_nodes = np.arange(nely + 1)
    bottom_nodes = np.array([i * (nely + 1) + nely for i in range(nelx + 1)])

    sym_x = 2 * left_nodes      # fix x-displacement on left edge
    sym_y = 2 * bottom_nodes + 1  # fix y-displacement on bottom edge

    fixed_dofs = np.union1d(sym_x, sym_y)

    # Two load cases for biaxial tension:
    # 1) Horizontal tension on right edge
    # 2) Vertical tension on top edge
    right_nodes = np.array([nelx * (nely + 1) + j for j in range(nely + 1)])
    top_nodes = np.array([i * (nely + 1) for i in range(nelx + 1)])

    f1 = np.zeros(solver.ndof)
    load_per_node = 1.0 / len(right_nodes)
    for n in right_nodes:
        f1[2 * n] = load_per_node  # x-force on right edge

    f2 = np.zeros(solver.ndof)
    load_per_node = 1.0 / len(top_nodes)
    for n in top_nodes:
        f2[2 * n + 1] = -load_per_node  # y-force on top edge (upward in image = negative y)

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

    Layout (nelx horizontal = half-width, nely vertical = height):

        Scuttle top (preserved solid strip)
        |                                  |
        |     Cockpit void (obstacle)      |  <- outer sill wall
        |                                  |
        Floor (preserved solid strip)------+
        ^                                  ^
        Centreline                    Sill outer edge
        (symmetry)                    (suspension pickup)

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
    nel = nelx * nely

    def elem(ix, iy):
        return ix * nely + iy

    # --- Element masks ---
    designable = np.ones(nel, dtype=bool)
    obstacle = np.zeros(nel, dtype=bool)

    # Floor: bottom 3 elements, full width (preserved solid)
    floor_thickness = 3
    for i in range(nelx):
        for j in range(nely - floor_thickness, nely):
            designable[elem(i, j)] = False

    # Scuttle top: top 3 elements, full width (preserved solid)
    scuttle_thickness = 3
    for i in range(nelx):
        for j in range(scuttle_thickness):
            designable[elem(i, j)] = False

    # Cockpit void
    cockpit_x_end = int(nelx * 0.60)
    cockpit_y_start = scuttle_thickness + 2
    cockpit_y_end = nely - floor_thickness - 2
    for i in range(0, cockpit_x_end):
        for j in range(cockpit_y_start, cockpit_y_end):
            obstacle[elem(i, j)] = True
            designable[elem(i, j)] = False

    # Initial densities
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

    # Load case 1: vertical at suspension pickup
    f_vertical = np.zeros(solver.ndof)
    f_vertical[2 * pickup_node + 1] = -1.0

    # Load case 2: lateral at suspension pickup
    f_lateral = np.zeros(solver.ndof)
    f_lateral[2 * pickup_node] = 1.0

    # Weight lateral relative to vertical using actual force ratio
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
