"""2D SIMP topology optimisation solver.

Solves the minimum compliance problem on a 2D rectangular domain using
Q4 (4-node quadrilateral) finite elements and the SIMP material model.

Reference test case: cantilever beam with tip load. The optimised result
should produce a truss-like structure matching known analytical solutions.
"""

import numpy as np
from scipy.sparse import coo_matrix, diags, lil_matrix
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
        force: np.ndarray,
        designable: np.ndarray | None = None,
        on_iteration: callable = None,
    ) -> tuple[np.ndarray, list[float]]:
        """Run SIMP optimisation.

        Args:
            fixed_dofs: array of DOF indices with zero displacement.
            force: global force vector of length ndof.
            designable: boolean mask per element. Non-designable elements
                stay at their initial density. Defaults to all designable.
            on_iteration: callback(iteration, densities, compliance, change)
                called after each iteration for live visualisation.

        Returns:
            (densities, compliance_history) where densities is (nelx*nely,)
            array of final element densities.
        """
        nel = self.nelx * self.nely
        x = np.full(nel, self.volfrac)
        xphys = x.copy()

        if designable is None:
            designable = np.ones(nel, dtype=bool)

        free_dofs = np.setdiff1d(np.arange(self.ndof), fixed_dofs)

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

            # Solve Ku = f using sparse LU for numerical stability
            K_free = K[free_dofs, :][:, free_dofs]
            f_free = force[free_dofs]
            u = np.zeros(self.ndof)
            lu = splu(K_free.tocsc())
            u[free_dofs] = lu.solve(f_free)

            # Element compliance and sensitivity
            ce = np.zeros(nel)
            for el in range(nel):
                ue = u[self.edofMat[el]]
                ce[el] = ue @ self.KE @ ue

            compliance = np.sum(
                (self.Emin + xphys**self.penalty * (self.E0 - self.Emin)) * ce
            )
            compliance_history.append(compliance)

            dc = -self.penalty * xphys**(self.penalty - 1) * (self.E0 - self.Emin) * ce
            dv = np.ones(nel)

            # Apply density filter to sensitivities
            dc = np.array(self.H @ (dc * xphys / self.Hs)).flatten()
            dv = np.array(self.H @ (dv * xphys / self.Hs)).flatten()

            # Optimality criteria update
            xold = x.copy()
            x = self._oc_update(x, dc, dv, designable)

            # Filter densities
            xphys = np.array(self.H @ x / self.Hs).flatten()
            change = np.max(np.abs(x - xold))

            if on_iteration:
                on_iteration(iteration, xphys.copy(), compliance, change)

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

        # Clamp positive sensitivities to avoid sqrt of negative
        dc = np.minimum(dc, -1e-12)

        while (l2 - l1) / (l1 + l2) > 1e-3:
            lmid = 0.5 * (l2 + l1)
            ratio = np.sqrt(-dc / (dv * lmid + 1e-12))
            xnew = np.maximum(
                0.001,
                np.maximum(
                    x - move,
                    np.minimum(1.0, np.minimum(x + move, x * ratio)),
                ),
            )
            # Lock non-designable elements
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

    # Fix left edge (all DOFs on x=0 nodes)
    fixed_nodes = np.arange(nely + 1)
    fixed_dofs = np.union1d(2 * fixed_nodes, 2 * fixed_nodes + 1)

    # Point load downward at middle of right edge
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

    Pin support (x+y fixed) at bottom-left, roller (y fixed) at bottom-right.
    Central top load. Known correct result: arch with diagonal members.
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

    # Support pads: fix y-displacement over a few nodes at each bottom corner
    # to avoid numerical ill-conditioning from single-point supports.
    # Pad width scales with grid size (minimum 3 nodes per pad).
    pad = max(3, nelx // 30)

    # Left support pad: bottom-left corner, fix x+y
    left_pad_nodes = np.array([i * (nely + 1) + nely for i in range(pad)])
    left_dofs = np.union1d(2 * left_pad_nodes, 2 * left_pad_nodes + 1)

    # Right support pad: bottom-right corner, fix y only (roller)
    right_pad_nodes = np.array([(nelx - i) * (nely + 1) + nely for i in range(pad)])
    right_dofs = 2 * right_pad_nodes + 1

    fixed_dofs = np.union1d(left_dofs, right_dofs)

    # Point load downward at top-centre
    mid_top_node = (nelx // 2) * (nely + 1)
    force = np.zeros(solver.ndof)
    force[2 * mid_top_node + 1] = -1.0

    return solver.solve(fixed_dofs, force, on_iteration=on_iteration)


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

    Loads applied:
    - Vertical at lower-right corner (suspension pickup)
    - Lateral at lower-right corner
    - Bending distributed along floor

    Returns (densities, history, nelx, nely) for plotting.
    """
    # Scale geometry to element grid
    # Half-width from centreline to outer sill edge
    half_width_mm = cfg.geometry.front_track_mm / 2
    # Tub height from floor to scuttle top (approximate as 40% of overall height)
    tub_height_mm = cfg.geometry.overall_height_mm * 0.40

    # Element size: 10mm per element gives good resolution
    element_size_mm = 10.0
    nelx = int(half_width_mm / element_size_mm)
    nely = int(tub_height_mm / element_size_mm)

    solver = Solver2D(nelx, nely, cfg.simp)

    # --- Define regions ---
    designable = np.ones(nelx * nely, dtype=bool)

    def elem(ix, iy):
        return ix * nely + iy

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

    # Cockpit void: inner region is obstacle (forced void)
    # Cockpit occupies roughly 60% of width from centreline, 70% of height
    cockpit_x_end = int(nelx * 0.60)
    cockpit_y_start = scuttle_thickness + 2
    cockpit_y_end = nely - floor_thickness - 2
    obstacle = np.zeros(nelx * nely, dtype=bool)
    for i in range(0, cockpit_x_end):
        for j in range(cockpit_y_start, cockpit_y_end):
            obstacle[elem(i, j)] = True
            designable[elem(i, j)] = False

    # Initialise densities: obstacles start at minimum, preserved at 1.0, rest at volfrac
    nel = nelx * nely
    x_init = np.full(nel, cfg.simp.volume_fraction)
    x_init[obstacle] = 0.001
    x_init[~designable & ~obstacle] = 1.0

    # --- Boundary conditions ---
    # Symmetry on left edge: fix x-displacement for all left-edge nodes
    left_nodes = np.arange(nely + 1)
    sym_dofs = 2 * left_nodes  # x-DOFs only

    # Fix y-displacement along the entire top edge (scuttle connects to
    # the rest of the monocoque longitudinally - reacted by adjacent sections)
    top_nodes = np.array([i * (nely + 1) for i in range(nelx + 1)])
    top_y_dofs = 2 * top_nodes + 1

    # Pin bottom-left corner fully to prevent rigid body motion
    pin_dofs = np.array([2 * nely, 2 * nely + 1])

    fixed_dofs = np.union1d(np.union1d(sym_dofs, top_y_dofs), pin_dofs)

    # --- Apply loads ---
    from car_solver.loads import calculate_load_cases

    cases = calculate_load_cases(cfg)

    force = np.zeros(solver.ndof)

    # Suspension pickup: bottom-right corner node
    pickup_node = (nelx + 1) * (nely + 1) - 1  # bottom-right
    force[2 * pickup_node + 1] = -1.0  # vertical downward (normalised)
    force[2 * pickup_node] = (
        cases.front_left_dynamic.lateral_n / cases.front_left_dynamic.vertical_n
    )  # lateral as ratio of vertical

    # --- Solve with custom initial densities ---
    # Override the solver's internal initialisation by patching solve
    original_solve = solver.solve

    def solve_with_init(fixed_dofs, force, designable=None, on_iteration=None):
        nel = solver.nelx * solver.nely
        x = x_init.copy()
        xphys = x.copy()

        if designable is None:
            designable_mask = np.ones(nel, dtype=bool)
        else:
            designable_mask = designable

        free_dofs = np.setdiff1d(np.arange(solver.ndof), fixed_dofs)
        compliance_history = []
        change = 1.0

        for iteration in range(solver.max_iter):
            if change < solver.tol and iteration > 1:
                break

            sK = (
                (solver.KE.flatten()[np.newaxis]).T
                * (solver.Emin + xphys**solver.penalty * (solver.E0 - solver.Emin))
            ).flatten(order="F")
            K = coo_matrix(
                (sK, (solver.iK, solver.jK)), shape=(solver.ndof, solver.ndof)
            ).tocsc()

            K_free = K[free_dofs, :][:, free_dofs]
            f_free = force[free_dofs]
            u = np.zeros(solver.ndof)
            lu = splu(K_free.tocsc())
            u[free_dofs] = lu.solve(f_free)

            ce = np.zeros(nel)
            for el in range(nel):
                ue = u[solver.edofMat[el]]
                ce[el] = ue @ solver.KE @ ue

            compliance = np.sum(
                (solver.Emin + xphys**solver.penalty * (solver.E0 - solver.Emin)) * ce
            )
            compliance_history.append(compliance)

            dc = -solver.penalty * xphys**(solver.penalty - 1) * (solver.E0 - solver.Emin) * ce
            dv = np.ones(nel)

            dc = np.array(solver.H @ (dc * xphys / solver.Hs)).flatten()
            dv = np.array(solver.H @ (dv * xphys / solver.Hs)).flatten()

            xold = x.copy()
            x = solver._oc_update(x, dc, dv, designable_mask)

            # Force obstacle elements to void
            x[obstacle] = 0.001

            xphys = np.array(solver.H @ x / solver.Hs).flatten()
            xphys[obstacle] = 0.001
            change = np.max(np.abs(x - xold))

            if on_iteration:
                on_iteration(iteration, xphys.copy(), compliance, change)

        return xphys, compliance_history

    densities, history = solve_with_init(fixed_dofs, force, designable, on_iteration)
    return densities, history, nelx, nely
