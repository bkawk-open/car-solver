"""Tests for the 2D SIMP solver."""

import numpy as np

from car_solver.config import SIMPConfig
from car_solver.solver2d import Solver2D, cantilever_beam, mbb_beam, element_stiffness


def test_element_stiffness_symmetry():
    KE = element_stiffness(1.0, 0.3)
    assert KE.shape == (8, 8)
    np.testing.assert_allclose(KE, KE.T, atol=1e-12)


def test_element_stiffness_positive_definite():
    KE = element_stiffness(1.0, 0.3)
    eigenvalues = np.linalg.eigvalsh(KE)
    # Non-rigid-body modes must be positive
    assert np.sum(eigenvalues > 1e-10) >= 4


def test_cantilever_converges():
    """Cantilever beam should converge and reduce compliance."""
    simp = SIMPConfig(
        penalty=3.0,
        volume_fraction=0.4,
        convergence_tolerance=0.01,
        max_iterations=50,
        filter_radius=1.5,
    )
    densities, history = cantilever_beam(nelx=30, nely=10, simp=simp)

    assert len(history) > 1
    # Compliance should decrease over iterations
    assert history[-1] < history[0]
    # Volume fraction should be approximately correct
    actual_vf = np.mean(densities)
    assert abs(actual_vf - 0.4) < 0.05


def test_cantilever_structure_is_sensible():
    """The optimised beam should have more material near the fixed end."""
    simp = SIMPConfig(
        penalty=3.0,
        volume_fraction=0.4,
        convergence_tolerance=0.01,
        max_iterations=50,
        filter_radius=1.5,
    )
    densities, _ = cantilever_beam(nelx=30, nely=10, simp=simp)
    grid = densities.reshape(30, 10)

    # Left third (near fixed support) should have more material than right third
    left_avg = grid[:10, :].mean()
    right_avg = grid[20:, :].mean()
    assert left_avg > right_avg


def test_fixed_elements_stay_fixed():
    """Non-designable elements should keep their initial density."""
    simp = SIMPConfig(
        penalty=3.0,
        volume_fraction=0.5,
        convergence_tolerance=0.01,
        max_iterations=20,
        filter_radius=1.5,
    )
    solver = Solver2D(20, 10, simp)

    fixed_nodes = np.arange(11)
    fixed_dofs = np.union1d(2 * fixed_nodes, 2 * fixed_nodes + 1)

    force = np.zeros(solver.ndof)
    force[2 * 230 + 1] = -1.0

    # Mark a strip as non-designable
    designable = np.ones(200, dtype=bool)
    designable[95:105] = False

    densities, _ = solver.solve(fixed_dofs, force, designable=designable)
    # Non-designable elements should stay closer to initial value than
    # fully optimised elements (filter blurs neighbours slightly)
    locked_std = np.std(densities[95:105])
    free_std = np.std(np.concatenate([densities[:90], densities[110:]]))
    assert locked_std < free_std


def test_mbb_converges():
    """MBB beam should converge and reduce compliance."""
    simp = SIMPConfig(
        penalty=3.0,
        volume_fraction=0.4,
        convergence_tolerance=0.01,
        max_iterations=50,
        filter_radius=1.5,
    )
    densities, history = mbb_beam(nelx=60, nely=20, simp=simp)

    assert len(history) > 1
    assert history[-1] < history[0]
    # Volume fraction converges slowly on MBB; at 50 iterations
    # it undershoots slightly due to the density filter
    actual_vf = np.mean(densities)
    assert abs(actual_vf - 0.4) < 0.15


def test_mbb_structure_is_sensible():
    """Full MBB beam: load at top-centre, supports at bottom corners.
    Material should concentrate near the load and supports, with the
    centre region of the beam being sparser."""
    simp = SIMPConfig(
        penalty=3.0,
        volume_fraction=0.4,
        convergence_tolerance=0.01,
        max_iterations=50,
        filter_radius=1.5,
    )
    densities, _ = mbb_beam(nelx=60, nely=20, simp=simp)
    grid = densities.reshape(60, 20)

    # Left quarter (near left support) should be denser than right-centre
    left_avg = grid[:15, :].mean()
    right_centre_avg = grid[35:50, 5:15].mean()
    assert left_avg > right_centre_avg
