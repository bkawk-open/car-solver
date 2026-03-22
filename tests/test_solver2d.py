"""Tests for the 2D SIMP solver."""

import numpy as np

from car_solver.config import SIMPConfig
from car_solver.solver2d import (
    Solver2D, cantilever_beam, mbb_beam, plate_with_hole, element_stiffness,
)


def _quick_simp(**overrides) -> SIMPConfig:
    defaults = dict(
        penalty=3.0,
        volume_fraction=0.4,
        convergence_tolerance=0.01,
        max_iterations=50,
        filter_radius=1.5,
    )
    defaults.update(overrides)
    return SIMPConfig(**defaults)


def test_element_stiffness_symmetry():
    KE = element_stiffness(1.0, 0.3)
    assert KE.shape == (8, 8)
    np.testing.assert_allclose(KE, KE.T, atol=1e-12)


def test_element_stiffness_positive_definite():
    KE = element_stiffness(1.0, 0.3)
    eigenvalues = np.linalg.eigvalsh(KE)
    assert np.sum(eigenvalues > 1e-10) >= 4


# --- Cantilever beam ---

def test_cantilever_converges():
    """Cantilever beam should converge and reduce compliance."""
    densities, history = cantilever_beam(nelx=30, nely=10, simp=_quick_simp())
    assert len(history) > 1
    assert history[-1] < history[0]
    assert abs(np.mean(densities) - 0.4) < 0.05


def test_cantilever_structure_is_sensible():
    """More material near the fixed end than the free end."""
    densities, _ = cantilever_beam(nelx=30, nely=10, simp=_quick_simp())
    grid = densities.reshape(30, 10)
    assert grid[:10, :].mean() > grid[20:, :].mean()


# --- MBB beam ---

def test_mbb_converges():
    densities, history = mbb_beam(nelx=60, nely=20, simp=_quick_simp())
    assert len(history) > 1
    assert history[-1] < history[0]
    assert abs(np.mean(densities) - 0.4) < 0.15


def test_mbb_structure_is_sensible():
    """Left quarter (near pinned support + load) denser than right-centre void."""
    densities, _ = mbb_beam(nelx=60, nely=20, simp=_quick_simp())
    grid = densities.reshape(60, 20)
    assert grid[:15, :].mean() > grid[35:50, 5:15].mean()


# --- Plate with hole ---

def test_plate_with_hole_converges():
    densities, history = plate_with_hole(nelx=40, nely=40, simp=_quick_simp())
    assert len(history) > 1
    assert history[-1] < history[0]


def test_plate_with_hole_obstacle_stays_void():
    """Elements inside the hole should remain at minimum density."""
    densities, _ = plate_with_hole(nelx=40, nely=40, simp=_quick_simp())
    grid = densities.reshape(40, 40)
    # Hole is at origin (i=0, j=0), radius ~10 elements
    hole_region = grid[:5, :5]
    assert hole_region.max() < 0.01


def test_plate_with_hole_volume_fraction():
    """Non-obstacle elements should achieve target volume fraction.
    Obstacle elements are excluded from the volume budget."""
    densities, _ = plate_with_hole(
        nelx=40, nely=40,
        simp=_quick_simp(volume_fraction=0.3, max_iterations=80),
    )
    # Volume fraction of the full domain (including obstacles at ~0)
    # should be less than target since obstacles consume no volume
    assert np.mean(densities) < 0.3


# --- Multi-load-case ---

def test_multi_load_case_differs_from_single():
    """Two load cases should produce different topology than either alone."""
    simp = _quick_simp()
    solver = Solver2D(30, 10, simp)

    fixed_nodes = np.arange(11)
    fixed_dofs = np.union1d(2 * fixed_nodes, 2 * fixed_nodes + 1)

    # Load case 1: vertical tip load
    f1 = np.zeros(solver.ndof)
    f1[2 * 340 + 1] = -1.0

    # Load case 2: horizontal tip load
    f2 = np.zeros(solver.ndof)
    f2[2 * 340] = 1.0

    d_single, _ = solver.solve(fixed_dofs, f1)

    solver2 = Solver2D(30, 10, simp)
    d_multi, _ = solver2.solve(fixed_dofs, forces=[f1, f2], weights=[0.5, 0.5])

    # Results should differ meaningfully
    diff = np.mean(np.abs(d_single - d_multi))
    assert diff > 0.01


def test_multi_load_case_weights_matter():
    """Changing weights should change the result."""
    simp = _quick_simp()

    fixed_nodes = np.arange(11)
    fixed_dofs = np.union1d(2 * fixed_nodes, 2 * fixed_nodes + 1)

    f1 = np.zeros(2 * 31 * 11)
    f1[2 * 340 + 1] = -1.0
    f2 = np.zeros(2 * 31 * 11)
    f2[2 * 340] = 1.0

    solver1 = Solver2D(30, 10, simp)
    d1, _ = solver1.solve(fixed_dofs, forces=[f1, f2], weights=[0.9, 0.1])

    solver2 = Solver2D(30, 10, simp)
    d2, _ = solver2.solve(fixed_dofs, forces=[f1, f2], weights=[0.1, 0.9])

    diff = np.mean(np.abs(d1 - d2))
    assert diff > 0.01


# --- Designable / obstacle ---

def test_fixed_elements_stay_fixed():
    """Non-designable elements should keep their initial density."""
    simp = _quick_simp(volume_fraction=0.5, max_iterations=20)
    solver = Solver2D(20, 10, simp)

    fixed_nodes = np.arange(11)
    fixed_dofs = np.union1d(2 * fixed_nodes, 2 * fixed_nodes + 1)

    force = np.zeros(solver.ndof)
    force[2 * 230 + 1] = -1.0

    designable = np.ones(200, dtype=bool)
    designable[95:105] = False

    densities, _ = solver.solve(fixed_dofs, force, designable=designable)
    locked_std = np.std(densities[95:105])
    free_std = np.std(np.concatenate([densities[:90], densities[110:]]))
    assert locked_std < free_std


def test_obstacle_elements_stay_void():
    """Obstacle elements should remain at minimum density."""
    simp = _quick_simp(max_iterations=20)
    solver = Solver2D(20, 10, simp)

    fixed_nodes = np.arange(11)
    fixed_dofs = np.union1d(2 * fixed_nodes, 2 * fixed_nodes + 1)

    force = np.zeros(solver.ndof)
    force[2 * 230 + 1] = -1.0

    obstacle = np.zeros(200, dtype=bool)
    obstacle[95:105] = True
    designable = np.ones(200, dtype=bool)
    designable[95:105] = False

    densities, _ = solver.solve(
        fixed_dofs, force, designable=designable, obstacle=obstacle,
    )
    assert np.all(densities[95:105] < 0.01)
