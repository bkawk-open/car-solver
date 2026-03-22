"""Tests for lightweight output reporting."""

import json

import numpy as np

from car_solver.config import Config, ManufacturingConfig
from car_solver.monocoque import MonocoqueTub
from car_solver.output import (
    build_monocoque_constraint_report,
    build_monocoque_run_summary,
    build_run_manifest,
    write_constraint_report,
    write_run_manifest,
    write_run_summary,
)
from tests.test_monocoque import _test_config


def _reporting_config() -> Config:
    cfg = _test_config()
    manufacturing = ManufacturingConfig(
        min_wall_thickness_mm=120.0,
        carbon_wall_thickness_mm=cfg.manufacturing.carbon_wall_thickness_mm,
        max_overhang_angle_deg=cfg.manufacturing.max_overhang_angle_deg,
        print_volume_x_mm=cfg.manufacturing.print_volume_x_mm,
        print_volume_y_mm=cfg.manufacturing.print_volume_y_mm,
        print_volume_z_mm=cfg.manufacturing.print_volume_z_mm,
        min_lattice_cell_mm=150.0,
        vent_hole_diameter_mm=cfg.manufacturing.vent_hole_diameter_mm,
    )
    return Config(
        mass=cfg.mass,
        geometry=cfg.geometry,
        wheels=cfg.wheels,
        material=cfg.material,
        safety=cfg.safety,
        weights=cfg.weights,
        loads=cfg.loads,
        simp=cfg.simp,
        manufacturing=manufacturing,
    )


def _find_designable_block(tub: MonocoqueTub, size: int) -> tuple[int, int, int]:
    grid = tub.designable.reshape(tub.nelx, tub.nely, tub.nelz)
    for ix in range(tub.nelx - size + 1):
        for iy in range(tub.nely - size + 1):
            for iz in range(tub.nelz - size + 1):
                block = grid[ix:ix + size, iy:iy + size, iz:iz + size]
                if np.all(block):
                    return ix, iy, iz
    raise AssertionError(f"Could not find a {size}x{size}x{size} designable block")


def test_monocoque_constraint_report_flags_active_constraints():
    tub = MonocoqueTub(_reporting_config(), element_size_mm=50.0)
    densities = tub.x_init.copy()

    report = build_monocoque_constraint_report("torsion", tub, densities)

    assert report.case_name == "torsion"
    assert report.element_size_mm == 50.0
    assert report.designable_elements == int(np.sum(tub.designable))
    assert report.preserve_elements == int(np.sum(tub.preserve))
    assert report.obstacle_elements == int(np.sum(tub.obstacle))
    names = {entry.name for entry in report.constraints}
    assert names == {"min_wall_thickness", "min_lattice_cell_size"}
    assert all(entry.active for entry in report.constraints)


def test_monocoque_constraint_report_detects_violating_features():
    tub = MonocoqueTub(_reporting_config(), element_size_mm=50.0)
    densities = np.zeros(tub.nel)
    ix, iy, iz = _find_designable_block(tub, 3)
    idx = lambda x, y, z: x * tub.nely * tub.nelz + y * tub.nelz + z

    densities[idx(ix, iy, iz)] = 1.0
    for x in range(ix, ix + 3):
        for y in range(iy, iy + 3):
            for z in range(iz, iz + 3):
                densities[idx(x, y, z)] = 1.0
    densities[idx(ix + 1, iy + 1, iz + 1)] = 0.0

    report = build_monocoque_constraint_report("corner", tub, densities)
    by_name = {entry.name: entry for entry in report.constraints}

    assert by_name["min_wall_thickness"].violating_elements > 0
    assert by_name["min_lattice_cell_size"].violating_elements > 0


def test_write_constraint_report_emits_json(tmp_path):
    tub = MonocoqueTub(_reporting_config(), element_size_mm=50.0)
    report = build_monocoque_constraint_report("bending", tub, tub.x_init.copy())

    path = write_constraint_report(report, save_path=str(tmp_path / "constraints.json"))

    payload = json.loads((tmp_path / "constraints.json").read_text())
    assert path == str(tmp_path / "constraints.json")
    assert payload["case_name"] == "bending"
    assert len(payload["constraints"]) == 2


def test_monocoque_run_summary_includes_history_stats_and_config():
    tub = MonocoqueTub(_reporting_config(), element_size_mm=50.0)
    densities = tub.x_init.copy()
    history = [120.0, 95.0, 80.0]

    summary = build_monocoque_run_summary(
        "torsion",
        "authoritative",
        tub,
        densities,
        history,
    )

    assert summary.case_name == "torsion"
    assert summary.case_kind == "authoritative"
    assert summary.iterations == 3
    assert summary.initial_compliance == 120.0
    assert summary.final_compliance == 80.0
    assert summary.compliance_history == history
    assert summary.geometry["elements"] == tub.nel
    assert summary.geometry["designable_elements"] == int(np.sum(tub.designable))
    assert summary.density_statistics.mean_density == float(np.mean(densities))
    assert summary.config_snapshot["manufacturing"]["min_wall_thickness_mm"] == 120.0
    assert summary.constraint_report.case_name == "torsion"
    assert summary.artifacts == {}


def test_write_run_summary_emits_json(tmp_path):
    tub = MonocoqueTub(_reporting_config(), element_size_mm=50.0)
    summary = build_monocoque_run_summary(
        "combined",
        "exploratory",
        tub,
        tub.x_init.copy(),
        [10.0, 8.0],
    )

    path = write_run_summary(summary, save_path=str(tmp_path / "summary.json"))

    payload = json.loads((tmp_path / "summary.json").read_text())
    assert path == str(tmp_path / "summary.json")
    assert payload["case_name"] == "combined"
    assert payload["case_kind"] == "exploratory"
    assert payload["iterations"] == 2
    assert payload["geometry"]["elements"] == tub.nel


def test_monocoque_run_summary_preserves_artifact_paths():
    tub = MonocoqueTub(_reporting_config(), element_size_mm=50.0)
    summary = build_monocoque_run_summary(
        "corner",
        "authoritative",
        tub,
        tub.x_init.copy(),
        [5.0, 4.0],
        artifacts={"density_stl": "/tmp/corner_density.stl"},
    )

    assert summary.artifacts["density_stl"] == "/tmp/corner_density.stl"


def test_run_manifest_captures_entry_point_and_artifacts():
    tub = MonocoqueTub(_reporting_config(), element_size_mm=50.0)
    manifest = build_run_manifest(
        case_name="torsion",
        case_kind="authoritative",
        entry_point="car_solver.monocoque.__main__",
        element_size_mm=tub.es,
        config_snapshot={"simp": {"max_iterations": 200}},
        artifacts={"summary": "/tmp/torsion_summary.json", "iteration_snapshots": ["/tmp/f0.png"]},
    )

    assert manifest.case_name == "torsion"
    assert manifest.entry_point == "car_solver.monocoque.__main__"
    assert manifest.artifacts["summary"] == "/tmp/torsion_summary.json"
    assert manifest.artifacts["iteration_snapshots"] == ["/tmp/f0.png"]


def test_write_run_manifest_emits_json(tmp_path):
    manifest = build_run_manifest(
        case_name="combined",
        case_kind="exploratory",
        entry_point="car_solver.monocoque.__main__",
        element_size_mm=50.0,
        config_snapshot={"weights": {"torsion": 0.5}},
        artifacts={"density_stl": "/tmp/combined.stl"},
    )

    path = write_run_manifest(manifest, save_path=str(tmp_path / "manifest.json"))

    payload = json.loads((tmp_path / "manifest.json").read_text())
    assert path == str(tmp_path / "manifest.json")
    assert payload["case_name"] == "combined"
    assert payload["artifacts"]["density_stl"] == "/tmp/combined.stl"
