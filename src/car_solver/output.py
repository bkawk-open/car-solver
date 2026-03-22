"""Output directory management and lightweight run reporting."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from car_solver.solver3d import (
    enforce_min_lattice_cell_size_3d,
    enforce_min_wall_thickness_3d,
)

OUTPUT_DIR = Path(__file__).resolve().parents[2] / "output"


def output_path(filename: str) -> str:
    """Return full path to a file in the output directory, creating it if needed."""
    OUTPUT_DIR.mkdir(exist_ok=True)
    return str(OUTPUT_DIR / filename)


@dataclass(frozen=True)
class ConstraintReportEntry:
    name: str
    active: bool
    parameter_mm: float
    parameter_elements: int
    violating_elements: int


@dataclass(frozen=True)
class ConstraintReport:
    case_name: str
    element_size_mm: float
    designable_elements: int
    preserve_elements: int
    obstacle_elements: int
    solid_elements_above_threshold: int
    constraints: list[ConstraintReportEntry]


@dataclass(frozen=True)
class DensityStatistics:
    min_density: float
    max_density: float
    mean_density: float
    mean_designable_density: float
    solid_elements_above_threshold: int
    solid_fraction_above_threshold: float


@dataclass(frozen=True)
class RunSummary:
    case_name: str
    case_kind: str
    element_size_mm: float
    iterations: int
    initial_compliance: float | None
    final_compliance: float | None
    compliance_history: list[float]
    geometry: dict[str, int | float]
    density_statistics: DensityStatistics
    constraint_report: ConstraintReport
    config_snapshot: dict
    artifacts: dict[str, str]


@dataclass(frozen=True)
class RunManifest:
    generated_at_utc: str
    case_name: str
    case_kind: str
    entry_point: str
    element_size_mm: float
    config_snapshot: dict
    artifacts: dict[str, object]


def build_monocoque_constraint_report(
    case_name: str,
    tub,
    densities: np.ndarray,
    threshold: float = 0.5,
) -> ConstraintReport:
    """Build a lightweight manufacturability report for a monocoque result."""
    min_wall_elements = max(
        1,
        int(np.ceil(tub.cfg.manufacturing.min_wall_thickness_mm / tub.es)),
    )
    min_void_elements = max(
        1,
        int(np.ceil(tub.cfg.manufacturing.min_lattice_cell_mm / tub.es)),
    )
    active = tub.designable & ~tub.obstacle
    violating_wall = int(np.sum(
        enforce_min_wall_thickness_3d(
            densities,
            tub.nelx,
            tub.nely,
            tub.nelz,
            min_wall_elements,
            active=active,
            threshold=threshold,
        )
    ))
    violating_void = int(np.sum(
        enforce_min_lattice_cell_size_3d(
            densities,
            tub.nelx,
            tub.nely,
            tub.nelz,
            min_void_elements,
            active=active,
            threshold=threshold,
        )
    ))

    return ConstraintReport(
        case_name=case_name,
        element_size_mm=float(tub.es),
        designable_elements=int(np.sum(tub.designable)),
        preserve_elements=int(np.sum(tub.preserve)),
        obstacle_elements=int(np.sum(tub.obstacle)),
        solid_elements_above_threshold=int(np.sum(densities >= threshold)),
        constraints=[
            ConstraintReportEntry(
                name="min_wall_thickness",
                active=min_wall_elements > 1,
                parameter_mm=float(tub.cfg.manufacturing.min_wall_thickness_mm),
                parameter_elements=min_wall_elements,
                violating_elements=violating_wall,
            ),
            ConstraintReportEntry(
                name="min_lattice_cell_size",
                active=min_void_elements > 1,
                parameter_mm=float(tub.cfg.manufacturing.min_lattice_cell_mm),
                parameter_elements=min_void_elements,
                violating_elements=violating_void,
            ),
        ],
    )


def write_constraint_report(report: ConstraintReport, save_path: str | None = None) -> str:
    """Write a constraint report to JSON."""
    path = save_path or output_path(f"{report.case_name}_constraints.json")
    Path(path).write_text(json.dumps(asdict(report), indent=2) + "\n")
    return path


def build_density_statistics(
    densities: np.ndarray,
    designable: np.ndarray,
    threshold: float = 0.5,
) -> DensityStatistics:
    """Summarise the final density field in a comparison-friendly form."""
    solid_elements = int(np.sum(densities >= threshold))
    return DensityStatistics(
        min_density=float(np.min(densities)),
        max_density=float(np.max(densities)),
        mean_density=float(np.mean(densities)),
        mean_designable_density=float(np.mean(densities[designable])),
        solid_elements_above_threshold=solid_elements,
        solid_fraction_above_threshold=float(solid_elements / len(densities)),
    )


def build_monocoque_run_summary(
    case_name: str,
    case_kind: str,
    tub,
    densities: np.ndarray,
    history: list[float],
    threshold: float = 0.5,
    artifacts: dict[str, str] | None = None,
) -> RunSummary:
    """Build a single JSON-friendly summary artifact for a monocoque run."""
    constraint_report = build_monocoque_constraint_report(
        case_name,
        tub,
        densities,
        threshold=threshold,
    )
    density_statistics = build_density_statistics(
        densities,
        tub.designable,
        threshold=threshold,
    )
    return RunSummary(
        case_name=case_name,
        case_kind=case_kind,
        element_size_mm=float(tub.es),
        iterations=len(history),
        initial_compliance=float(history[0]) if history else None,
        final_compliance=float(history[-1]) if history else None,
        compliance_history=[float(value) for value in history],
        geometry={
            "nelx": tub.nelx,
            "nely": tub.nely,
            "nelz": tub.nelz,
            "elements": tub.nel,
            "designable_elements": int(np.sum(tub.designable)),
            "preserve_elements": int(np.sum(tub.preserve)),
            "obstacle_elements": int(np.sum(tub.obstacle)),
        },
        density_statistics=density_statistics,
        constraint_report=constraint_report,
        config_snapshot=asdict(tub.cfg),
        artifacts=artifacts or {},
    )


def write_run_summary(summary: RunSummary, save_path: str | None = None) -> str:
    """Write a run summary to JSON."""
    path = save_path or output_path(f"{summary.case_name}_summary.json")
    Path(path).write_text(json.dumps(asdict(summary), indent=2) + "\n")
    return path


def build_run_manifest(
    case_name: str,
    case_kind: str,
    entry_point: str,
    element_size_mm: float,
    config_snapshot: dict,
    artifacts: dict[str, object],
) -> RunManifest:
    """Build a manifest that ties artifacts back to a reproducible code path."""
    return RunManifest(
        generated_at_utc=datetime.now(timezone.utc).isoformat(),
        case_name=case_name,
        case_kind=case_kind,
        entry_point=entry_point,
        element_size_mm=float(element_size_mm),
        config_snapshot=config_snapshot,
        artifacts=artifacts,
    )


def write_run_manifest(manifest: RunManifest, save_path: str | None = None) -> str:
    """Write a run manifest to JSON."""
    path = save_path or output_path(f"{manifest.case_name}_manifest.json")
    Path(path).write_text(json.dumps(asdict(manifest), indent=2) + "\n")
    return path
