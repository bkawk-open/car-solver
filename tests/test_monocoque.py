"""Tests for monocoque tub geometry and load cases."""

import numpy as np

from car_solver.config import (
    Config, MassConfig, GeometryConfig, WheelConfig, MaterialConfig,
    SafetyConfig, ObjectiveWeights, DynamicLoads, SIMPConfig, ManufacturingConfig,
)
from car_solver.monocoque import (
    MonocoqueTub, torsion_load_case, bending_load_case,
    corner_load_case, combined_load_case,
)


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


def _fast_config() -> Config:
    cfg = _test_config()
    simp = SIMPConfig(3.0, 0.30, 0.05, 20, 1.5)
    return Config(
        mass=cfg.mass, geometry=cfg.geometry, wheels=cfg.wheels,
        material=cfg.material, safety=cfg.safety, weights=cfg.weights,
        loads=cfg.loads, simp=simp, manufacturing=cfg.manufacturing,
    )


# --- Geometry ---

def test_tub_dimensions_full_width():
    """Full-width tub: nely should be full track, not half."""
    cfg = _test_config()
    tub = MonocoqueTub(cfg, element_size_mm=50.0)
    assert tub.nelx == 49
    # Full track 1598mm / 50 = 31 elements
    assert tub.nely == 31
    assert tub.nelz == 10


def test_element_masks_consistent():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    preserved = ~tub.designable & ~tub.obstacle
    assert not np.any(tub.obstacle & tub.designable)
    assert np.sum(tub.designable) + np.sum(tub.obstacle) + np.sum(preserved) == tub.nel


def test_designable_fraction_reasonable():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    frac = np.sum(tub.designable) / tub.nel
    assert 0.20 < frac < 0.60


def test_initial_densities_match_masks():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    preserved = ~tub.designable & ~tub.obstacle
    np.testing.assert_allclose(tub.x_init[preserved], 1.0)
    np.testing.assert_allclose(tub.x_init[tub.obstacle], 0.001)
    np.testing.assert_allclose(tub.x_init[tub.designable], cfg.simp.volume_fraction)


# --- Pickup nodes ---

def test_pickup_nodes_four_corners():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    pickups = tub.pickup_nodes()
    assert len(pickups) == 4
    assert set(pickups.keys()) == {"front_left", "front_right", "rear_left", "rear_right"}


def test_pickup_nodes_at_correct_positions():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    pickups = tub.pickup_nodes()
    assert pickups["front_left"] == tub.node(0, 0, 0)
    assert pickups["front_right"] == tub.node(0, tub.nely, 0)
    assert pickups["rear_left"] == tub.node(tub.nelx, 0, 0)
    assert pickups["rear_right"] == tub.node(tub.nelx, tub.nely, 0)


def test_pickup_nodes_not_in_obstacle():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    for name in tub.pickup_nodes():
        if "front" in name:
            ix = 0
        else:
            ix = tub.nelx - 1
        iy = tub.nely - 1 if "right" in name else 0
        el = _elem_idx(ix, iy, 0, tub)
        assert not tub.obstacle[el], f"{name} adjacent element is obstacle"


def _elem_idx(ix, iy, iz, tub):
    return ix * tub.nely * tub.nelz + iy * tub.nelz + iz


# --- Solver ---

def test_build_solver_uses_tuned_params():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    solver = tub.build_solver()
    assert solver.penalty == 4.0
    assert solver.rmin == 1.2


# --- Load cases ---

def test_torsion_converges():
    densities, history, tub = torsion_load_case(_fast_config(), element_size_mm=100.0)
    assert len(history) > 1
    assert all(c > 0 for c in history)
    assert np.all(densities[tub.obstacle] < 0.01)


def test_bending_converges():
    densities, history, tub = bending_load_case(_fast_config(), element_size_mm=100.0)
    assert len(history) > 1
    assert all(c > 0 for c in history)
    assert np.all(densities[tub.obstacle] < 0.01)


def test_corner_converges():
    densities, history, tub = corner_load_case(_fast_config(), element_size_mm=100.0)
    assert len(history) > 1
    assert all(c > 0 for c in history)
    assert np.all(densities[tub.obstacle] < 0.01)


def test_combined_converges():
    densities, history, tub = combined_load_case(_fast_config(), element_size_mm=100.0)
    assert len(history) > 1
    assert all(c > 0 for c in history)
    assert np.all(densities[tub.obstacle] < 0.01)
