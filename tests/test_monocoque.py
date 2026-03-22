"""Tests for monocoque tub geometry and load cases."""

import numpy as np

from car_solver.config import (
    Config, MassConfig, GeometryConfig, WheelConfig, MaterialConfig,
    SafetyConfig, ObjectiveWeights, DynamicLoads, SIMPConfig, ManufacturingConfig,
)
from car_solver.loads import calculate_load_cases
from car_solver.monocoque import (
    MonocoqueTub, torsion_load_case, bending_load_case,
    corner_load_case, combined_load_case, resolve_monocoque_case,
    monocoque_case_kind,
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
    preserved = tub.preserve
    assert not np.any(tub.obstacle & tub.designable)
    assert np.sum(tub.designable) + np.sum(tub.obstacle) + np.sum(preserved) == tub.nel
    assert not np.any(tub.preserve & tub.obstacle)
    np.testing.assert_array_equal(tub.non_designable, tub.preserve | tub.obstacle)


def test_designable_fraction_reasonable():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    frac = np.sum(tub.designable) / tub.nel
    assert 0.20 < frac < 0.60


def test_initial_densities_match_masks():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    preserved = tub.preserve
    np.testing.assert_allclose(tub.x_init[preserved], 1.0)
    np.testing.assert_allclose(tub.x_init[tub.obstacle], 0.001)
    np.testing.assert_allclose(tub.x_init[tub.designable], cfg.simp.volume_fraction)


def test_named_preserve_regions_exist():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    expected = {
        "floor",
        "scuttle",
        "front_bulkhead",
        "rear_bulkhead",
        "left_sill",
        "right_sill",
        "pickup_hard_points",
    }
    assert expected.issubset(tub.preserve_regions.keys())
    for name in expected:
        assert np.any(tub.preserve_regions[name]), f"{name} should not be empty"


def test_pickup_hard_points_cover_all_pickup_corners():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    region = tub.preserve_regions["pickup_hard_points"].reshape(tub.nelx, tub.nely, tub.nelz)
    assert region[0, 0, 0]
    assert region[0, tub.nely - 1, 0]
    assert region[tub.nelx - 1, 0, 0]
    assert region[tub.nelx - 1, tub.nely - 1, 0]


def test_named_obstacle_regions_exist():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    expected = {
        "cockpit_void",
        "drivetrain_tunnel",
        "front_left_wheel_arch",
        "front_right_wheel_arch",
        "rear_left_wheel_arch",
        "rear_right_wheel_arch",
    }
    assert expected.issubset(tub.obstacle_regions.keys())
    for name in expected:
        assert np.any(tub.obstacle_regions[name]), f"{name} should not be empty"


def test_drivetrain_tunnel_runs_near_vehicle_centerline():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    tunnel = tub.obstacle_regions["drivetrain_tunnel"].reshape(tub.nelx, tub.nely, tub.nelz)
    ys = np.argwhere(tunnel)[:, 1]
    centre_y = tub.nely // 2
    assert np.min(ys) <= centre_y <= np.max(ys)


def test_wheel_arches_stay_off_centreline():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    centre_y = tub.nely // 2
    for name in [
        "front_left_wheel_arch",
        "front_right_wheel_arch",
        "rear_left_wheel_arch",
        "rear_right_wheel_arch",
    ]:
        arch = tub.obstacle_regions[name].reshape(tub.nelx, tub.nely, tub.nelz)
        ys = np.argwhere(arch)[:, 1]
        assert not np.any(ys == centre_y), f"{name} should remain outboard of the centerline"


def test_region_unions_match_combined_masks():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    preserve_union = np.zeros(tub.nel, dtype=bool)
    for region in tub.preserve_regions.values():
        preserve_union |= region
    obstacle_union = np.zeros(tub.nel, dtype=bool)
    for region in tub.obstacle_regions.values():
        obstacle_union |= region
    np.testing.assert_array_equal(preserve_union, tub.preserve)
    np.testing.assert_array_equal(obstacle_union, tub.obstacle)


def test_preserve_is_not_inferred_indirectly():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    # Preserve is now first-class; keep the direct mask stable.
    np.testing.assert_array_equal(tub.preserve, ~tub.designable & ~tub.obstacle)


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


def test_named_load_node_groups_exist():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    expected = {
        "front_left_pickup",
        "front_right_pickup",
        "rear_left_pickup",
        "rear_right_pickup",
        "front_left_load",
        "front_right_load",
        "rear_left_load",
        "rear_right_load",
        "left_sill_top",
        "right_sill_top",
    }
    assert expected.issubset(tub.load_node_groups.keys())
    for name in expected:
        assert len(tub.load_node_groups[name]) >= 1


def test_named_support_dof_groups_exist():
    cfg = _test_config()
    tub = MonocoqueTub(cfg)
    assert set(tub.support_dof_groups.keys()) == {"pickup_points", "rear_face"}
    assert len(tub.support_dof_groups["pickup_points"]) == 12
    expected_rear = (tub.nely + 1) * (tub.nelz + 1) * 3
    assert len(tub.support_dof_groups["rear_face"]) == expected_rear


def test_load_and_support_groups_match_case_resolution():
    cfg = _test_config()
    tub = MonocoqueTub(cfg, element_size_mm=100.0)
    resolved_torsion = resolve_monocoque_case(cfg, "torsion", element_size_mm=100.0)
    resolved_bending = resolve_monocoque_case(cfg, "bending", element_size_mm=100.0)

    assert set(resolved_torsion.fixed_dofs.tolist()) == set(tub.support_dof_groups["rear_face"].tolist())
    assert set(resolved_bending.fixed_dofs.tolist()) == set(tub.support_dof_groups["pickup_points"].tolist())


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


def test_torsion_uses_rear_face_supports_only():
    cfg = _test_config()
    resolved = resolve_monocoque_case(cfg, "torsion", element_size_mm=100.0)
    tub = resolved.tub
    expected_count = (tub.nely + 1) * (tub.nelz + 1) * 3
    assert len(resolved.fixed_dofs) == expected_count

    front_left = tub.pickup_nodes()["front_left"]
    front_right = tub.pickup_nodes()["front_right"]
    front_pickup_dofs = {
        3 * front_left,
        3 * front_left + 1,
        3 * front_left + 2,
        3 * front_right,
        3 * front_right + 1,
        3 * front_right + 2,
    }
    assert front_pickup_dofs.isdisjoint(set(resolved.fixed_dofs.tolist()))


def test_torsion_force_vector_matches_spec():
    cfg = _test_config()
    resolved = resolve_monocoque_case(cfg, "torsion", element_size_mm=100.0)
    tub = resolved.tub
    force = resolved.forces[0]

    pickups = tub.pickup_nodes()
    fl_z = 3 * pickups["front_left"] + 2
    fr_z = 3 * pickups["front_right"] + 2

    assert len(resolved.forces) == 1
    assert force[fl_z] == 1000.0
    assert force[fr_z] == -1000.0
    assert abs(np.sum(force)) < 1e-9


def test_monocoque_case_kind_classifies_reference_cases():
    assert monocoque_case_kind("torsion") == "authoritative"
    assert monocoque_case_kind("bending") == "authoritative"
    assert monocoque_case_kind("corner") == "authoritative"
    assert monocoque_case_kind("combined") == "exploratory"


def test_resolve_monocoque_case_rejects_unknown_case():
    cfg = _test_config()
    try:
        resolve_monocoque_case(cfg, "not_a_case")
    except ValueError as exc:
        assert "Unknown monocoque case" in str(exc)
    else:
        raise AssertionError("Expected resolve_monocoque_case to reject unknown case")


def test_bending_converges():
    densities, history, tub = bending_load_case(_fast_config(), element_size_mm=100.0)
    assert len(history) > 1
    assert all(c > 0 for c in history)
    assert np.all(densities[tub.obstacle] < 0.01)


def test_bending_force_vector_matches_total_vehicle_weight():
    cfg = _test_config()
    cases = calculate_load_cases(cfg)
    resolved = resolve_monocoque_case(cfg, "bending", element_size_mm=100.0)
    force = resolved.forces[0]

    z_sum = np.sum(force[2::3])
    assert abs(z_sum + cases.bending.total_vertical_n) < 1e-9


def test_bending_uses_pickup_point_supports():
    cfg = _test_config()
    resolved = resolve_monocoque_case(cfg, "bending", element_size_mm=100.0)
    assert len(resolved.forces) == 1
    assert len(resolved.weights) == 1
    assert resolved.weights[0] == 1.0

    pickups = resolved.tub.pickup_nodes()
    expected = {
        3 * node + axis
        for node in pickups.values()
        for axis in range(3)
    }
    assert set(resolved.fixed_dofs.tolist()) == expected


def test_corner_converges():
    densities, history, tub = corner_load_case(_fast_config(), element_size_mm=100.0)
    assert len(history) > 1
    assert all(c > 0 for c in history)
    assert np.all(densities[tub.obstacle] < 0.01)


def test_corner_weight_split_matches_spec():
    cfg = _test_config()
    resolved = resolve_monocoque_case(cfg, "corner", element_size_mm=100.0)
    front = sum(weight for subcase, weight in zip(resolved.definition.subcases, resolved.weights)
                if subcase.name.startswith("front_"))
    rear = sum(weight for subcase, weight in zip(resolved.definition.subcases, resolved.weights)
               if subcase.name.startswith("rear_"))
    assert abs(front - (2.0 / 3.0)) < 1e-9
    assert abs(rear - (1.0 / 3.0)) < 1e-9


def test_corner_force_magnitudes_match_derived_loads():
    cfg = _test_config()
    cases = calculate_load_cases(cfg)
    resolved = resolve_monocoque_case(cfg, "corner", element_size_mm=100.0)
    tub = resolved.tub

    z_load = min(1, tub.nelz)
    fr = tub.node(0, tub.nely, z_load)
    fl = tub.node(0, 0, z_load)
    rr = tub.node(tub.nelx, tub.nely, z_load)
    rl = tub.node(tub.nelx, 0, z_load)

    expected = {
        "front_right_vertical": (3 * fr + 2, -cases.front_right_dynamic.vertical_n),
        "front_right_lateral": (3 * fr + 1, cases.front_right_dynamic.lateral_n),
        "front_right_braking": (3 * fr, cases.front_right_dynamic.longitudinal_n),
        "front_left_vertical": (3 * fl + 2, -cases.front_left_dynamic.vertical_n),
        "front_left_lateral": (3 * fl + 1, -cases.front_left_dynamic.lateral_n),
        "front_left_braking": (3 * fl, cases.front_left_dynamic.longitudinal_n),
        "rear_right_vertical": (3 * rr + 2, -cases.rear_right_dynamic.vertical_n),
        "rear_right_lateral": (3 * rr + 1, cases.rear_right_dynamic.lateral_n),
        "rear_right_braking": (3 * rr, -cases.rear_right_dynamic.longitudinal_n),
        "rear_left_vertical": (3 * rl + 2, -cases.rear_left_dynamic.vertical_n),
        "rear_left_lateral": (3 * rl + 1, -cases.rear_left_dynamic.lateral_n),
        "rear_left_braking": (3 * rl, -cases.rear_left_dynamic.longitudinal_n),
    }

    for subcase, force in zip(resolved.definition.subcases, resolved.forces):
        dof, magnitude = expected[subcase.name]
        assert force[dof] == magnitude
        assert abs(np.sum(np.abs(force)) - abs(magnitude)) < 1e-9


def test_combined_converges():
    densities, history, tub = combined_load_case(_fast_config(), element_size_mm=100.0)
    assert len(history) > 1
    assert all(c > 0 for c in history)
    assert np.all(densities[tub.obstacle] < 0.01)


def test_combined_weight_groups_match_objective_config():
    cfg = _test_config()
    resolved = resolve_monocoque_case(cfg, "combined", element_size_mm=100.0)
    grouped = {
        "torsion": 0.0,
        "bending": 0.0,
        "front": 0.0,
        "rear": 0.0,
    }
    for subcase, weight in zip(resolved.definition.subcases, resolved.weights):
        if subcase.name == "torsion":
            grouped["torsion"] += weight
        elif subcase.name == "bending":
            grouped["bending"] += weight
        elif subcase.name.startswith("front_"):
            grouped["front"] += weight
        elif subcase.name.startswith("rear_"):
            grouped["rear"] += weight

    assert abs(grouped["torsion"] - cfg.weights.torsion) < 1e-9
    assert abs(grouped["bending"] - cfg.weights.bending) < 1e-9
    assert abs(grouped["front"] - cfg.weights.front_corner) < 1e-9
    assert abs(grouped["rear"] - cfg.weights.rear_corner) < 1e-9
