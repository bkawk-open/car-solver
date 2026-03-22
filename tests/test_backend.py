"""Tests for the solver backend boundary."""

import numpy as np

from car_solver.config import SIMPConfig
from car_solver.linalg_backend import SciPySparseBackend
from car_solver.solver2d import Solver2D
from car_solver.solver3d import Solver3D


class _FakeFactorization:
    def __init__(self, size: int):
        self.size = size
        self.calls = 0

    def solve(self, rhs: np.ndarray) -> np.ndarray:
        self.calls += 1
        return np.zeros_like(rhs)


class _FakeBackend:
    def __init__(self):
        self.assemble_calls = 0
        self.factorize_calls = 0
        self.last_shape = None
        self.last_free_count = None
        self.last_factorization = None

    def assemble(self, sK, iK, jK, shape):
        self.assemble_calls += 1
        self.last_shape = shape
        return {"shape": shape}

    def factorize_free_matrix(self, K, free_dofs):
        self.factorize_calls += 1
        self.last_free_count = len(free_dofs)
        self.last_factorization = _FakeFactorization(len(free_dofs))
        return self.last_factorization


def _quick_simp() -> SIMPConfig:
    return SIMPConfig(
        penalty=3.0,
        volume_fraction=0.4,
        convergence_tolerance=0.05,
        max_iterations=2,
        filter_radius=1.5,
    )


def test_solver2d_uses_configured_backend():
    backend = _FakeBackend()
    solver = Solver2D(6, 3, _quick_simp(), backend=backend)
    fixed_nodes = np.arange(4)
    fixed_dofs = np.union1d(2 * fixed_nodes, 2 * fixed_nodes + 1)
    force = np.zeros(solver.ndof)
    force[-1] = -1.0

    densities, history = solver.solve(fixed_dofs, force)

    assert len(history) >= 1
    assert densities.shape == (solver.nel,)
    assert backend.assemble_calls >= 1
    assert backend.factorize_calls >= 1
    assert backend.last_shape == (solver.ndof, solver.ndof)
    assert backend.last_factorization.calls >= 1


def test_solver3d_uses_configured_backend():
    backend = _FakeBackend()
    solver = Solver3D(3, 2, 2, _quick_simp(), backend=backend)
    left_nodes = []
    for j in range(3):
        for k in range(3):
            left_nodes.append(j * 3 + k)
    left_nodes = np.array(left_nodes)
    fixed_dofs = np.union1d(
        np.union1d(3 * left_nodes, 3 * left_nodes + 1),
        3 * left_nodes + 2,
    )
    force = np.zeros(solver.ndof)
    force[-1] = -1.0

    densities, history = solver.solve(fixed_dofs, force)

    assert len(history) >= 1
    assert densities.shape == (solver.nel,)
    assert backend.assemble_calls >= 1
    assert backend.factorize_calls >= 1
    assert backend.last_shape == (solver.ndof, solver.ndof)
    assert backend.last_factorization.calls >= 1


def test_reference_backend_is_scipy_sparse():
    backend = SciPySparseBackend()
    assert hasattr(backend, "assemble")
    assert hasattr(backend, "factorize_free_matrix")
