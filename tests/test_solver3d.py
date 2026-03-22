"""Tests for the 3D SIMP solver."""

import numpy as np
import pytest

from car_solver.config import SIMPConfig
from car_solver.solver3d import (
    Solver3D,
    cantilever_3d,
    enforce_min_lattice_cell_size_3d,
    enforce_min_wall_thickness_3d,
    h8_element_stiffness,
)


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


@pytest.fixture(scope="module")
def cantilever_3d_result():
    return cantilever_3d(nelx=10, nely=4, nelz=4, simp=_quick_simp())


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


def test_cantilever_3d_converges(cantilever_3d_result):
    """3D cantilever should converge with decreasing compliance."""
    densities, history, *_ = cantilever_3d_result
    assert len(history) > 1
    assert history[-1] < history[0]
    actual_vf = np.mean(densities)
    assert abs(actual_vf - 0.4) < 0.1


def test_cantilever_3d_structure_sensible(cantilever_3d_result):
    """More material near fixed end than free end."""
    densities, _, nelx, nely, nelz = cantilever_3d_result
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


def test_min_wall_thickness_3d_removes_single_voxel_column():
    densities = np.zeros(27)
    densities[[4, 13, 22]] = 1.0  # one-element column through z in 3x3x3
    removed = enforce_min_wall_thickness_3d(
        densities, nelx=3, nely=3, nelz=3, min_wall_elements=2,
    )
    assert np.all(removed[[4, 13, 22]])


def test_min_wall_thickness_3d_preserves_two_by_two_by_two_block():
    densities = np.zeros(27)
    keep = [
        0, 1, 3, 4,
        9, 10, 12, 13,
    ]
    densities[keep] = 1.0
    removed = enforce_min_wall_thickness_3d(
        densities, nelx=3, nely=3, nelz=3, min_wall_elements=2,
    )
    assert not np.any(removed[keep])


def test_min_lattice_cell_size_3d_fills_single_voxel_void():
    densities = np.ones(27)
    densities[13] = 0.0
    filled = enforce_min_lattice_cell_size_3d(
        densities, nelx=3, nely=3, nelz=3, min_void_elements=2,
    )
    assert filled[13]


def test_min_lattice_cell_size_3d_preserves_two_by_two_by_two_void():
    densities = np.ones(27)
    keep_void = [
        0, 1, 3, 4,
        9, 10, 12, 13,
    ]
    densities[keep_void] = 0.0
    filled = enforce_min_lattice_cell_size_3d(
        densities, nelx=3, nely=3, nelz=3, min_void_elements=2,
    )
    assert not np.any(filled[keep_void])


def test_min_wall_thickness_3d_before_after_regression():
    densities = np.zeros(64)
    thin_column = [0, 16, 32, 48]
    densities[thin_column] = 1.0
    before_solid = densities.reshape(4, 4, 4) >= 0.5
    assert before_solid[:, 0, 0].all()

    removed = enforce_min_wall_thickness_3d(
        densities, nelx=4, nely=4, nelz=4, min_wall_elements=2,
    )
    constrained = densities.copy()
    constrained[removed] = 0.0
    after_solid = constrained.reshape(4, 4, 4) >= 0.5

    assert not after_solid[:, 0, 0].any()


def test_min_lattice_cell_size_3d_before_after_regression():
    densities = np.zeros(64)
    for x in range(1, 4):
        for y in range(1, 4):
            for z in range(1, 4):
                densities[x * 16 + y * 4 + z] = 1.0
    tiny_void = [42]
    densities[tiny_void] = 0.0
    before_solid = densities.reshape(4, 4, 4) >= 0.5
    assert not before_solid[2, 2, 2]

    filled = enforce_min_lattice_cell_size_3d(
        densities, nelx=4, nely=4, nelz=4, min_void_elements=2,
    )
    constrained = densities.copy()
    constrained[filled] = 1.0
    after_solid = constrained.reshape(4, 4, 4) >= 0.5
    assert after_solid[2, 2, 2]


def test_manufacturing_constraints_3d_respect_active_mask():
    densities = np.ones(27)
    densities[13] = 0.0
    active = np.ones(27, dtype=bool)
    active[13] = False

    filled = enforce_min_lattice_cell_size_3d(
        densities,
        nelx=3,
        nely=3,
        nelz=3,
        min_void_elements=2,
        active=active,
    )
    assert not filled[13]
