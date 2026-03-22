"""Tests for optional solver timing instrumentation."""

import numpy as np

from car_solver.config import SIMPConfig
from car_solver.solver2d import Solver2D
from car_solver.solver3d import Solver3D


def _quick_simp() -> SIMPConfig:
    return SIMPConfig(
        penalty=3.0,
        volume_fraction=0.4,
        convergence_tolerance=0.05,
        max_iterations=5,
        filter_radius=1.5,
    )


def test_solver2d_collect_timing_records_phase_totals():
    solver = Solver2D(10, 4, _quick_simp())
    fixed_nodes = np.arange(5)
    fixed_dofs = np.union1d(2 * fixed_nodes, 2 * fixed_nodes + 1)
    force = np.zeros(solver.ndof)
    force[-1] = -1.0

    _, history = solver.solve(fixed_dofs, force, collect_timing=True)

    assert len(history) >= 1
    assert solver.last_timing is not None
    assert solver.last_timing.iterations == len(history)
    assert solver.last_timing.total_seconds >= 0.0
    assert solver.last_timing.assembly_seconds >= 0.0
    assert solver.last_timing.factorization_seconds >= 0.0
    assert solver.last_timing.solve_seconds >= 0.0
    assert solver.last_timing.sensitivity_filter_seconds >= 0.0
    assert solver.last_timing.update_seconds >= 0.0


def test_solver3d_collect_timing_records_phase_totals():
    solver = Solver3D(4, 3, 2, _quick_simp())
    left_nodes = []
    for j in range(4):
        for k in range(3):
            left_nodes.append(j * 3 + k)
    left_nodes = np.array(left_nodes)
    fixed_dofs = np.union1d(
        np.union1d(3 * left_nodes, 3 * left_nodes + 1),
        3 * left_nodes + 2,
    )
    force = np.zeros(solver.ndof)
    force[-1] = -1.0

    _, history = solver.solve(fixed_dofs, force, collect_timing=True)

    assert len(history) >= 1
    assert solver.last_timing is not None
    assert solver.last_timing.iterations == len(history)
    assert solver.last_timing.total_seconds >= 0.0
    assert solver.last_timing.assembly_seconds >= 0.0
    assert solver.last_timing.factorization_seconds >= 0.0
    assert solver.last_timing.solve_seconds >= 0.0
    assert solver.last_timing.sensitivity_filter_seconds >= 0.0
    assert solver.last_timing.update_seconds >= 0.0
