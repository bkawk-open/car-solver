"""Tests for load case calculator."""

import numpy as np

from car_solver.config import (
    Config, MassConfig, GeometryConfig, WheelConfig, MaterialConfig,
    SafetyConfig, ObjectiveWeights, DynamicLoads, SIMPConfig, ManufacturingConfig,
)
from car_solver.loads import calculate_static_corners, calculate_load_cases, G


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


def test_static_corners_sum_to_total_weight():
    cfg = _test_config()
    sc = calculate_static_corners(cfg)
    expected = cfg.mass.total_kg * G
    assert abs(sc.total_n - expected) < 0.01


def test_static_corners_front_rear_split():
    cfg = _test_config()
    sc = calculate_static_corners(cfg)
    front_ratio = sc.front_total_n / sc.total_n
    assert abs(front_ratio - 0.38) < 0.001


def test_static_corners_symmetric():
    cfg = _test_config()
    sc = calculate_static_corners(cfg)
    assert sc.front_left_n == sc.front_right_n
    assert sc.rear_left_n == sc.rear_right_n


def test_dynamic_vertical_scales_by_g_factor():
    cfg = _test_config()
    cases = calculate_load_cases(cfg)
    sc = cases.static_corners
    fl = cases.front_left_dynamic
    assert abs(fl.vertical_n - sc.front_left_n * 3.0) < 0.01


def test_dynamic_lateral_distribution():
    cfg = _test_config()
    cases = calculate_load_cases(cfg)
    total_w = cases.static_corners.total_n
    # Front lateral per corner = total * 1.5g * 0.60 / 2
    expected = total_w * 1.5 * 0.60 / 2
    assert abs(cases.front_left_dynamic.lateral_n - expected) < 0.01


def test_dynamic_braking_distribution():
    cfg = _test_config()
    cases = calculate_load_cases(cfg)
    total_w = cases.static_corners.total_n
    # Front braking per corner = total * 1.2g * 0.70 / 2
    expected = total_w * 1.2 * 0.70 / 2
    assert abs(cases.front_left_dynamic.longitudinal_n - expected) < 0.01


def test_torsion_equal_and_opposite():
    cfg = _test_config()
    cases = calculate_load_cases(cfg)
    assert cases.torsion.front_left_n == -cases.torsion.front_right_n
