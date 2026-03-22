"""Tests for monocoque tub geometry and load cases."""

import numpy as np

from car_solver.config import (
    Config, MassConfig, GeometryConfig, WheelConfig, MaterialConfig,
    SafetyConfig, ObjectiveWeights, DynamicLoads, SIMPConfig, ManufacturingConfig,
)
from car_solver.monocoque import MonocoqueTub, torsion_load_case


def _test_config() -> Config:
    return Config(
        mass=MassConfig(60, 20, 80, 60, 20, 80),
        geometry=GeometryConfig(2457, 1598, 1530, 4573, 1852, 1279, 150, 420, 0.38),
        wheels=WheelConfig(20, 9.5, 255, 35, 21, 12, 315, 30),
        material=MaterialConfig(0.45, 230, 3.5, 0.375, 1.49, 5.0, 10, 2),
        safety=SafetyConfig(2.0, 3.5, 3.5),
        weights=ObjectiveWeights(0.50, 0.20, 0.20, 0.10),
        loads=DynamicLoads(3.0, 1.5, 1.2, 0.70, 0.60, 10000),
        simp=SIMPConfig(3.0, 0.30, 0.01, 200, 1.5),
        manufacturing=ManufacturingConfig(3.0, 3.5, 45, 1200, 1200, 600, 8.0, 0.5),
    )


# --- Geometry ---

def test_tub_dimensions_from_config():
    """Tub dimensions should scale from vehicle geometry."""
    cfg = _test_config()
    tub = MonocoqueTub(cfg, element_size_mm=50.0)
    # Wheelbase 2457mm / 50 = 49 elements
    assert tub.nelx == 49
    # Half-track 799mm / 50 = 15 elements
    assert tub.nely == 15
    # Tub height 0.4 * 1279 = 511mm / 50 = 10 elements
    assert tub.nelz == 10


def test_element_masks_consistent():
    """Every element should be exactly one of: designable, obstacle, or preserved."""
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    preserved = ~tub.designable & ~tub.obstacle
    # No element should be both obstacle and designable
    assert not np.any(tub.obstacle & tub.designable)
    # Every element is in exactly one category
    assert np.sum(tub.designable) + np.sum(tub.obstacle) + np.sum(preserved) == tub.nel


def test_designable_fraction_reasonable():
    """Designable region should be 30-60% of total elements."""
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    frac = np.sum(tub.designable) / tub.nel
    assert 0.30 < frac < 0.60


def test_initial_densities_match_masks():
    """Preserved elements at 1.0, obstacles at 0.001, designable at volfrac."""
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    preserved = ~tub.designable & ~tub.obstacle
    np.testing.assert_allclose(tub.x_init[preserved], 1.0)
    np.testing.assert_allclose(tub.x_init[tub.obstacle], 0.001)
    np.testing.assert_allclose(tub.x_init[tub.designable], cfg.simp.volume_fraction)


# --- Pickup nodes ---

def test_pickup_nodes_exist():
    """All four pickup nodes should be valid node indices."""
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    solver = tub.build_solver()
    max_node = solver.ndof // 3

    pickups = tub.pickup_nodes()
    assert len(pickups) == 4
    for name, n in pickups.items():
        assert 0 <= n < max_node, f"{name} node {n} out of range"


def test_pickup_nodes_at_floor():
    """All pickups should be at floor level (z=0)."""
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    pickups = tub.pickup_nodes()

    # front_right and rear_right at y=nely (sill outer edge)
    assert pickups["front_right"] == tub.node(0, tub.nely, 0)
    assert pickups["rear_right"] == tub.node(tub.nelx, tub.nely, 0)
    # front_left and rear_left at y=0 (centreline)
    assert pickups["front_left"] == tub.node(0, 0, 0)
    assert pickups["rear_left"] == tub.node(tub.nelx, 0, 0)


def test_pickup_nodes_not_in_obstacle():
    """No pickup node should be surrounded by obstacle elements."""
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    pickups = tub.pickup_nodes()
    # Pickup nodes are at corners of the grid (x=0 or nelx, y=0 or nely, z=0)
    # These should be in preserved regions (bulkheads/floor), not obstacles
    # Check that adjacent elements are not obstacles
    for name, node_idx in pickups.items():
        # The element containing this node (at floor level) should not be obstacle
        # For corner nodes, check the nearest element
        if "front" in name:
            ix = 0
        else:
            ix = tub.nelx - 1
        iy = tub.nely - 1 if "right" in name else 0
        iz = 0  # floor
        el = ix * tub.nely * tub.nelz + iy * tub.nelz + iz
        assert not tub.obstacle[el], f"{name} pickup adjacent element is obstacle"


# --- Symmetry ---

def test_symmetry_dofs_cover_centreline():
    """Symmetry should fix y-DOF for every node on the y=0 face."""
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    sym = tub.symmetry_dofs()
    expected_count = (tub.nelx + 1) * (tub.nelz + 1)
    assert len(sym) == expected_count


# --- Solver ---

def test_build_solver_uses_tuned_params():
    """build_solver should use the tuned monocoque SIMP parameters."""
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    solver = tub.build_solver()
    assert solver.penalty == 4.0
    assert solver.rmin == 1.2


# --- Torsion ---

def test_torsion_converges():
    """Torsion load case should converge with positive compliance."""
    cfg = _test_config()
    # Use small grid and few iterations for speed
    simp = SIMPConfig(3.0, 0.30, 0.05, 20, 1.5)
    cfg_fast = Config(
        mass=cfg.mass, geometry=cfg.geometry, wheels=cfg.wheels,
        material=cfg.material, safety=cfg.safety, weights=cfg.weights,
        loads=cfg.loads, simp=simp, manufacturing=cfg.manufacturing,
    )
    densities, history, tub = torsion_load_case(cfg_fast, element_size_mm=100.0)
    assert len(history) > 1
    assert all(c > 0 for c in history)
    # Obstacles should stay void
    assert np.all(densities[tub.obstacle] < 0.01)
