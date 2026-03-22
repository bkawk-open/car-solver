"""Tests for the 3D SIMP solver."""

import numpy as np

from car_solver.config import SIMPConfig
from car_solver.solver3d import Solver3D, cantilever_3d, h8_element_stiffness


def _quick_simp(**overrides) -> SIMPConfig:
    defaults = dict(
        penalty=3.0,
        volume_fraction=0.4,
        convergence_tolerance=0.01,
        max_iterations=30,
        filter_radius=1.5,
    )
    defaults.update(overrides)
    return SIMPConfig(**defaults)


def test_h8_stiffness_symmetry():
    KE = h8_element_stiffness(0.3)
    assert KE.shape == (24, 24)
    np.testing.assert_allclose(KE, KE.T, atol=1e-12)


def test_h8_stiffness_positive_semidefinite():
    KE = h8_element_stiffness(0.3)
    eigenvalues = np.linalg.eigvalsh(KE)
    # 24 DOFs - 6 rigid body modes = 18 positive eigenvalues
    assert np.sum(eigenvalues > 1e-10) >= 18
    # No negative eigenvalues beyond numerical noise
    assert np.all(eigenvalues > -1e-10)


def test_cantilever_3d_converges():
    """3D cantilever should converge with decreasing compliance."""
    densities, history, *_ = cantilever_3d(
        nelx=10, nely=4, nelz=4, simp=_quick_simp(),
    )
    assert len(history) > 1
    assert history[-1] < history[0]
    actual_vf = np.mean(densities)
    assert abs(actual_vf - 0.4) < 0.1


def test_cantilever_3d_structure_sensible():
    """More material near fixed end than free end."""
    densities, _, nelx, nely, nelz = cantilever_3d(
        nelx=10, nely=4, nelz=4, simp=_quick_simp(),
    )
    grid = densities.reshape(nelx, nely, nelz)
    left_avg = grid[:3, :, :].mean()
    right_avg = grid[7:, :, :].mean()
    assert left_avg > right_avg


def test_solver3d_obstacle():
    """Obstacle elements should stay void."""
    simp = _quick_simp(max_iterations=10)
    solver = Solver3D(6, 4, 4, simp)

    left_nodes = []
    for j in range(5):
        for k in range(5):
            left_nodes.append(j * 5 + k)
    left_nodes = np.array(left_nodes)
    fixed_dofs = np.union1d(
        np.union1d(3 * left_nodes, 3 * left_nodes + 1),
        3 * left_nodes + 2,
    )

    force = np.zeros(solver.ndof)
    load_node = 6 * 5 * 5 + 2 * 5 + 2
    force[3 * load_node + 2] = -1.0

    obstacle = np.zeros(solver.nel, dtype=bool)
    obstacle[40:60] = True
    designable = np.ones(solver.nel, dtype=bool)
    designable[40:60] = False

    densities, _ = solver.solve(
        fixed_dofs, force, designable=designable, obstacle=obstacle,
    )
    assert np.all(densities[40:60] < 0.01)
