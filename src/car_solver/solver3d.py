"""3D SIMP topology optimisation solver.

Solves the minimum compliance problem on a 3D rectangular domain using
H8 (8-node hexahedral) finite elements and the SIMP material model.

Supports multiple load cases with weighted compliance objective,
obstacle regions (forced void), and preserved regions (forced solid).
"""

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import splu

from car_solver.config import SIMPConfig


def h8_element_stiffness(nu: float) -> np.ndarray:
    """24x24 stiffness matrix for a unit-cube H8 element with unit modulus.

    Uses 2x2x2 Gauss quadrature (full integration). Each node has 3 DOFs
    (ux, uy, uz). Node ordering follows a consistent corner convention.

    Derived from the standard isoparametric formulation with shape functions
    N_i = (1/8)(1 + xi_i*xi)(1 + eta_i*eta)(1 + zeta_i*zeta).
    """
    # Gauss points and weights for 2x2x2 integration
    gp = 1.0 / np.sqrt(3.0)
    gauss_pts = np.array([-gp, gp])
    weights = np.array([1.0, 1.0])

    # Node coordinates in natural coordinates (xi, eta, zeta)
    # Ordering: bottom face (z=-1) then top face (z=+1), each CCW
    node_coords = np.array([
        [-1, -1, -1],  # 0
        [+1, -1, -1],  # 1
        [+1, +1, -1],  # 2
        [-1, +1, -1],  # 3
        [-1, -1, +1],  # 4
        [+1, -1, +1],  # 5
        [+1, +1, +1],  # 6
        [-1, +1, +1],  # 7
    ], dtype=float)

    # Constitutive matrix (3D isotropic, unit modulus)
    C = _constitutive_matrix(nu)

    KE = np.zeros((24, 24))

    for i, xi in enumerate(gauss_pts):
        for j, eta in enumerate(gauss_pts):
            for k, zeta in enumerate(gauss_pts):
                w = weights[i] * weights[j] * weights[k]

                # Shape function derivatives in natural coordinates
                dNdnat = np.zeros((3, 8))
                for n in range(8):
                    xi_n, eta_n, zeta_n = node_coords[n]
                    dNdnat[0, n] = xi_n * (1 + eta_n * eta) * (1 + zeta_n * zeta) / 8
                    dNdnat[1, n] = (1 + xi_n * xi) * eta_n * (1 + zeta_n * zeta) / 8
                    dNdnat[2, n] = (1 + xi_n * xi) * (1 + eta_n * eta) * zeta_n / 8

                # Jacobian for unit cube is 0.5 * I (maps [-1,1] to [0,1])
                # det(J) = 0.125
                J = dNdnat @ (node_coords + 1) / 2  # physical coords [0,1]^3
                detJ = np.linalg.det(J)
                Jinv = np.linalg.inv(J)

                # Shape function derivatives in physical coordinates
                dNdx = Jinv @ dNdnat

                # Strain-displacement matrix B (6x24)
                B = np.zeros((6, 24))
                for n in range(8):
                    B[0, 3*n] = dNdx[0, n]       # eps_xx
                    B[1, 3*n+1] = dNdx[1, n]     # eps_yy
                    B[2, 3*n+2] = dNdx[2, n]     # eps_zz
                    B[3, 3*n+1] = dNdx[2, n]     # gamma_yz
                    B[3, 3*n+2] = dNdx[1, n]
                    B[4, 3*n] = dNdx[2, n]       # gamma_xz
                    B[4, 3*n+2] = dNdx[0, n]
                    B[5, 3*n] = dNdx[1, n]       # gamma_xy
                    B[5, 3*n+1] = dNdx[0, n]

                KE += w * detJ * (B.T @ C @ B)

    return KE


def _constitutive_matrix(nu: float) -> np.ndarray:
    """6x6 isotropic elasticity matrix for unit Young's modulus."""
    c1 = 1.0 / ((1 + nu) * (1 - 2 * nu))
    C = c1 * np.array([
        [1 - nu, nu, nu, 0, 0, 0],
        [nu, 1 - nu, nu, 0, 0, 0],
        [nu, nu, 1 - nu, 0, 0, 0],
        [0, 0, 0, (1 - 2*nu)/2, 0, 0],
        [0, 0, 0, 0, (1 - 2*nu)/2, 0],
        [0, 0, 0, 0, 0, (1 - 2*nu)/2],
    ])
    return C


def density_filter_3d(
    nelx: int, nely: int, nelz: int, rmin: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build 3D density filter weights.

    Element index convention: el = x * (nely * nelz) + y * nelz + z
    """
    ceil_r = int(np.ceil(rmin))
    max_per_el = (2 * ceil_r + 1) ** 3
    nfilter = nelx * nely * nelz * max_per_el
    iH = np.zeros(nfilter, dtype=int)
    jH = np.zeros(nfilter, dtype=int)
    sH = np.zeros(nfilter)
    cc = 0

    for i in range(nelx):
        for j in range(nely):
            for k in range(nelz):
                row = i * (nely * nelz) + j * nelz + k
                for ii in range(max(i - ceil_r + 1, 0), min(i + ceil_r, nelx)):
                    for jj in range(max(j - ceil_r + 1, 0), min(j + ceil_r, nely)):
                        for kk in range(max(k - ceil_r + 1, 0), min(k + ceil_r, nelz)):
                            col = ii * (nely * nelz) + jj * nelz + kk
                            dist = np.sqrt((i-ii)**2 + (j-jj)**2 + (k-kk)**2)
                            w = max(0, rmin - dist)
                            if w > 0:
                                iH[cc] = row
                                jH[cc] = col
                                sH[cc] = w
                                cc += 1

    return iH[:cc], jH[:cc], sH[:cc]


class Solver3D:
    """3D SIMP topology optimisation on a rectangular domain."""

    def __init__(
        self,
        nelx: int,
        nely: int,
        nelz: int,
        simp: SIMPConfig,
        Emin: float = 1e-9,
        nu: float = 0.3,
    ):
        self.nelx = nelx
        self.nely = nely
        self.nelz = nelz
        self.penalty = simp.penalty
        self.volfrac = simp.volume_fraction
        self.tol = simp.convergence_tolerance
        self.max_iter = simp.max_iterations
        self.rmin = simp.filter_radius
        self.Emin = Emin
        self.E0 = 1.0
        self.nu = nu

        self.nel = nelx * nely * nelz
        self.ndof = 3 * (nelx + 1) * (nely + 1) * (nelz + 1)
        self.KE = h8_element_stiffness(nu)

        # Build filter
        iH, jH, sH = density_filter_3d(nelx, nely, nelz, self.rmin)
        self.H = coo_matrix((sH, (iH, jH)), shape=(self.nel, self.nel)).tocsc()
        self.Hs = np.array(self.H.sum(axis=1)).flatten()

        # DOF connectivity (vectorised)
        self.edofMat = self._build_edof_mat()

        # Precompute sparse assembly indices
        iK = np.kron(self.edofMat, np.ones((24, 1), dtype=int)).flatten()
        jK = np.kron(self.edofMat, np.ones((1, 24), dtype=int)).flatten()
        self.iK = iK
        self.jK = jK

        # Precompute flattened KE
        self.KE_flat = self.KE.flatten()

    def _build_edof_mat(self) -> np.ndarray:
        """Build element-to-DOF connectivity matrix.

        Node index: node(ix, iy, iz) = ix*(nely+1)*(nelz+1) + iy*(nelz+1) + iz
        Element (ix, iy, iz) has 8 corner nodes, each with 3 DOFs.
        """
        nyz = (self.nely + 1) * (self.nelz + 1)

        ix, iy, iz = np.meshgrid(
            np.arange(self.nelx),
            np.arange(self.nely),
            np.arange(self.nelz),
            indexing="ij",
        )
        ix = ix.flatten()
        iy = iy.flatten()
        iz = iz.flatten()

        # 8 corner nodes of each element
        n = np.column_stack([
            ix * nyz + iy * (self.nelz + 1) + iz,           # 0: (0,0,0)
            (ix+1) * nyz + iy * (self.nelz + 1) + iz,       # 1: (1,0,0)
            (ix+1) * nyz + (iy+1) * (self.nelz + 1) + iz,   # 2: (1,1,0)
            ix * nyz + (iy+1) * (self.nelz + 1) + iz,       # 3: (0,1,0)
            ix * nyz + iy * (self.nelz + 1) + (iz+1),       # 4: (0,0,1)
            (ix+1) * nyz + iy * (self.nelz + 1) + (iz+1),   # 5: (1,0,1)
            (ix+1) * nyz + (iy+1) * (self.nelz + 1) + (iz+1), # 6: (1,1,1)
            ix * nyz + (iy+1) * (self.nelz + 1) + (iz+1),   # 7: (0,1,1)
        ])

        # Each node has 3 DOFs (ux, uy, uz)
        edof = np.zeros((self.nel, 24), dtype=int)
        for i in range(8):
            edof[:, 3*i] = 3 * n[:, i]
            edof[:, 3*i+1] = 3 * n[:, i] + 1
            edof[:, 3*i+2] = 3 * n[:, i] + 2

        return edof

    def _element_compliance_vectorised(self, u: np.ndarray) -> np.ndarray:
        """Compute ce = ue^T @ KE @ ue for all elements."""
        ue = u[self.edofMat]       # (nel, 24)
        Kue = ue @ self.KE         # (nel, 24)
        return np.sum(ue * Kue, axis=1)  # (nel,)

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
        """Run 3D SIMP optimisation. Same interface as Solver2D.solve()."""
        nel = self.nel

        if isinstance(forces, np.ndarray) and forces.ndim == 1:
            force_list = [forces]
        else:
            force_list = list(forces)
        n_cases = len(force_list)

        if weights is None:
            weights = [1.0 / n_cases] * n_cases
        if len(weights) != n_cases:
            raise ValueError(f"Got {len(weights)} weights for {n_cases} load cases")

        if designable is None:
            designable = np.ones(nel, dtype=bool)
        if obstacle is None:
            obstacle = np.zeros(nel, dtype=bool)

        if x_init is not None:
            x = x_init.copy()
        else:
            x = np.full(nel, self.volfrac)
        x[obstacle] = 0.001
        xphys = np.array(self.H @ x / self.Hs).flatten()
        xphys[obstacle] = 0.001

        free_dofs = np.setdiff1d(np.arange(self.ndof), fixed_dofs)
        active = designable & ~obstacle

        compliance_history = []
        change = 1.0

        for iteration in range(self.max_iter):
            if change < self.tol and iteration > 1:
                break

            E_eff = self.Emin + xphys**self.penalty * (self.E0 - self.Emin)
            sK = (self.KE_flat[np.newaxis].T * E_eff).flatten(order="F")
            K = coo_matrix((sK, (self.iK, self.jK)), shape=(self.ndof, self.ndof)).tocsc()

            K_free = K[free_dofs, :][:, free_dofs]
            lu = splu(K_free.tocsc())

            total_compliance = 0.0
            dc = np.zeros(nel)

            for f_vec, w in zip(force_list, weights):
                u = np.zeros(self.ndof)
                u[free_dofs] = lu.solve(f_vec[free_dofs])

                ce = self._element_compliance_vectorised(u)
                total_compliance += w * np.sum(E_eff * ce)

                dc[active] += w * (
                    -self.penalty
                    * xphys[active] ** (self.penalty - 1)
                    * (self.E0 - self.Emin)
                    * ce[active]
                )

            compliance_history.append(total_compliance)

            dv = np.zeros(nel)
            dv[active] = 1.0

            dc = np.array(self.H @ (dc * xphys / self.Hs)).flatten()
            dv = np.array(self.H @ (dv * xphys / self.Hs)).flatten()

            xold = x.copy()
            x = self._oc_update(x, dc, dv, designable)
            x[obstacle] = 0.001

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
        nel = len(x)
        l1, l2 = 1e-12, 1e9
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


def cantilever_3d(
    nelx: int = 30,
    nely: int = 10,
    nelz: int = 6,
    simp: SIMPConfig | None = None,
    on_iteration: callable = None,
) -> tuple[np.ndarray, list[float], int, int, int]:
    """3D cantilever beam test case.

    Left face (x=0) fully fixed. Point load downward at centre of
    right face (x=nelx). Returns (densities, history, nelx, nely, nelz).
    """
    if simp is None:
        simp = SIMPConfig(
            penalty=3.0,
            volume_fraction=0.30,
            convergence_tolerance=0.01,
            max_iterations=100,
            filter_radius=1.5,
        )

    solver = Solver3D(nelx, nely, nelz, simp)

    # Fix all DOFs on left face (x=0)
    left_nodes = []
    nyz = (nely + 1) * (nelz + 1)
    for j in range(nely + 1):
        for k in range(nelz + 1):
            left_nodes.append(j * (nelz + 1) + k)
    left_nodes = np.array(left_nodes)
    fixed_dofs = np.union1d(
        np.union1d(3 * left_nodes, 3 * left_nodes + 1),
        3 * left_nodes + 2,
    )

    # Point load downward (negative z) at centre of right face
    mid_y = nely // 2
    mid_z = nelz // 2
    load_node = nelx * nyz + mid_y * (nelz + 1) + mid_z
    force = np.zeros(solver.ndof)
    force[3 * load_node + 2] = -1.0  # z-direction

    densities, history = solver.solve(fixed_dofs, force, on_iteration=on_iteration)
    return densities, history, nelx, nely, nelz
