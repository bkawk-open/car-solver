"""Run a repeatable Stage 1 baseline and save comparable summaries."""

from __future__ import annotations

import json
from dataclasses import asdict, replace

import numpy as np

from car_solver.config import SIMPConfig, load_config
from car_solver.monocoque import (
    bending_load_case,
    combined_load_case,
    corner_load_case,
    torsion_load_case,
)
from car_solver.output import output_path
from car_solver.solver2d import cantilever_beam, mbb_beam, plate_with_hole
from car_solver.solver3d import cantilever_3d


def _case_summary(name: str, kind: str, densities: np.ndarray, history: list[float], shape: tuple[int, ...]):
    return {
        "name": name,
        "kind": kind,
        "shape": shape,
        "iterations": len(history),
        "initial_compliance": float(history[0]) if history else None,
        "final_compliance": float(history[-1]) if history else None,
        "mean_density": float(np.mean(densities)),
        "solid_fraction_above_0_5": float(np.mean(densities >= 0.5)),
        "min_density": float(np.min(densities)),
        "max_density": float(np.max(densities)),
    }


def _comparison_summary(name: str, reference: np.ndarray, combined: np.ndarray):
    return {
        "name": name,
        "mean_abs_density_difference": float(np.mean(np.abs(reference - combined))),
        "max_abs_density_difference": float(np.max(np.abs(reference - combined))),
        "solid_fraction_delta_above_0_5": float(
            np.mean(reference >= 0.5) - np.mean(combined >= 0.5)
        ),
    }


def main():
    cfg = load_config()
    baseline_simp = replace(
        cfg.simp,
        max_iterations=40,
        convergence_tolerance=0.02,
    )
    baseline_cfg = replace(cfg, simp=baseline_simp)

    results: dict[str, object] = {
        "profile": {
            "name": "stage1_coarse_baseline",
            "notes": (
                "Integration baseline uses reduced optimisation effort and "
                "100 mm monocoque voxels to keep runs repeatable."
            ),
            "simp": asdict(baseline_simp),
            "monocoque_element_size_mm": 100.0,
        },
        "validation_2d": [],
        "validation_3d": [],
        "monocoque_cases": [],
        "comparisons": [],
    }

    densities, history = cantilever_beam(nelx=60, nely=20, simp=baseline_simp)
    results["validation_2d"].append(
        _case_summary("cantilever_2d", "validation", densities, history, (60, 20))
    )

    densities, history = mbb_beam(nelx=90, nely=30, simp=baseline_simp)
    results["validation_2d"].append(
        _case_summary("mbb_2d", "validation", densities, history, (90, 30))
    )

    densities, history = plate_with_hole(nelx=60, nely=60, simp=baseline_simp)
    results["validation_2d"].append(
        _case_summary("plate_with_hole_2d", "validation", densities, history, (60, 60))
    )

    densities, history, nelx, nely, nelz = cantilever_3d(
        nelx=20,
        nely=8,
        nelz=6,
        simp=baseline_simp,
    )
    results["validation_3d"].append(
        _case_summary("cantilever_3d", "validation", densities, history, (nelx, nely, nelz))
    )

    torsion_d, torsion_h, torsion_tub = torsion_load_case(
        baseline_cfg,
        element_size_mm=100.0,
    )
    results["monocoque_cases"].append(
        _case_summary(
            "torsion",
            "authoritative",
            torsion_d,
            torsion_h,
            (torsion_tub.nelx, torsion_tub.nely, torsion_tub.nelz),
        )
    )

    bending_d, bending_h, bending_tub = bending_load_case(
        baseline_cfg,
        element_size_mm=100.0,
    )
    results["monocoque_cases"].append(
        _case_summary(
            "bending",
            "authoritative",
            bending_d,
            bending_h,
            (bending_tub.nelx, bending_tub.nely, bending_tub.nelz),
        )
    )

    corner_d, corner_h, corner_tub = corner_load_case(
        baseline_cfg,
        element_size_mm=100.0,
    )
    results["monocoque_cases"].append(
        _case_summary(
            "corner",
            "authoritative",
            corner_d,
            corner_h,
            (corner_tub.nelx, corner_tub.nely, corner_tub.nelz),
        )
    )

    combined_d, combined_h, combined_tub = combined_load_case(
        baseline_cfg,
        element_size_mm=100.0,
    )
    results["monocoque_cases"].append(
        _case_summary(
            "combined",
            "exploratory",
            combined_d,
            combined_h,
            (combined_tub.nelx, combined_tub.nely, combined_tub.nelz),
        )
    )

    results["comparisons"].append(_comparison_summary("torsion_vs_combined", torsion_d, combined_d))
    results["comparisons"].append(_comparison_summary("bending_vs_combined", bending_d, combined_d))
    results["comparisons"].append(_comparison_summary("corner_vs_combined", corner_d, combined_d))

    path = output_path("stage1_baseline.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
        f.write("\n")

    print(path)


if __name__ == "__main__":
    main()
