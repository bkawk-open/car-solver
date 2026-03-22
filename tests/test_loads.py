"""Tests for load case calculator."""

from car_solver.config import (
    Config, MassConfig, GeometryConfig, WheelConfig, MaterialConfig,
    SafetyConfig, ObjectiveWeights, DynamicLoads, SIMPConfig, ManufacturingConfig,
)
from car_solver.loads import (
    DistributedLoadSpec,
    PointLoadSpec,
    build_monocoque_load_definitions,
    calculate_static_corners,
    calculate_load_cases,
    G,
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


def test_sandwich_bending_stiffness():
    """Sandwich panel should be much stiffer than a solid panel of the
    same skin material weight (2x 2mm skins = 4mm solid equivalent)."""
    cfg = _test_config()
    mat = cfg.material
    d_sandwich = mat.sandwich_bending_stiffness_n_mm

    # Equivalent solid panel using only the skin material (4mm)
    skin_total = 2 * mat.carbon_skin_thickness_mm
    e_solid = mat.composite_modulus_gpa * 1000  # MPa
    d_solid = e_solid * skin_total**3 / 12

    # Sandwich should be dramatically stiffer at equal skin weight
    assert d_sandwich > 5 * d_solid

    # Sanity check: stiffness should be positive and finite
    assert d_sandwich > 0
    assert d_sandwich < 1e12


def test_composite_modulus():
    """Rule of mixtures composite modulus should match spec (~39 GPa)."""
    cfg = _test_config()
    E = cfg.material.composite_modulus_gpa
    assert 35 < E < 45


def test_monocoque_load_definitions_have_expected_support_sets():
    cfg = _test_config()
    defs = build_monocoque_load_definitions(cfg)
    assert defs.torsion.support_set == "rear_face"
    assert defs.bending.support_set == "pickup_points"
    assert defs.corner.support_set == "pickup_points"
    assert defs.combined.support_set == "rear_face"


def test_monocoque_torsion_definition_uses_spec_magnitudes():
    cfg = _test_config()
    defs = build_monocoque_load_definitions(cfg)
    subcase = defs.torsion.subcases[0]
    assert len(subcase.loads) == 2
    left, right = subcase.loads
    assert isinstance(left, PointLoadSpec)
    assert isinstance(right, PointLoadSpec)
    assert left.target == "front_left_pickup"
    assert left.axis == "z"
    assert left.magnitude_n == 1000.0
    assert right.target == "front_right_pickup"
    assert right.axis == "z"
    assert right.magnitude_n == -1000.0


def test_monocoque_bending_definition_distributes_total_weight():
    cfg = _test_config()
    defs = build_monocoque_load_definitions(cfg)
    cases = calculate_load_cases(cfg)
    subcase = defs.bending.subcases[0]
    assert len(subcase.loads) == 2
    left, right = subcase.loads
    assert isinstance(left, DistributedLoadSpec)
    assert isinstance(right, DistributedLoadSpec)
    assert left.target == "left_sill_top"
    assert right.target == "right_sill_top"
    assert abs(left.total_magnitude_n + right.total_magnitude_n + cases.bending.total_vertical_n) < 1e-9


def test_monocoque_corner_weights_sum_to_one():
    cfg = _test_config()
    defs = build_monocoque_load_definitions(cfg)
    total = sum(subcase.weight for subcase in defs.corner.subcases)
    assert abs(total - 1.0) < 1e-9


def test_monocoque_combined_weights_match_objective_weights():
    cfg = _test_config()
    defs = build_monocoque_load_definitions(cfg)
    total = sum(subcase.weight for subcase in defs.combined.subcases)
    assert abs(total - 1.0) < 1e-9
    assert defs.combined.subcases[0].weight == cfg.weights.torsion
    assert defs.combined.subcases[1].weight == cfg.weights.bending
