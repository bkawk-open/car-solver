"""Run a measured solver performance baseline."""

from __future__ import annotations

import json
from dataclasses import asdict, replace

import numpy as np

from car_solver.config import load_config
from car_solver.monocoque import resolve_monocoque_case
from car_solver.output import output_path
from car_solver.solver2d import Solver2D
from car_solver.solver3d import Solver3D


def _timing_dict(timing):
    payload = asdict(timing)
    total = payload["total_seconds"] or 1.0
    payload["percentages"] = {
        "assembly": payload["assembly_seconds"] / total,
        "factorization": payload["factorization_seconds"] / total,
        "solve": payload["solve_seconds"] / total,
        "sensitivity_filter": payload["sensitivity_filter_seconds"] / total,
        "update": payload["update_seconds"] / total,
    }
    return payload


def run_2d_case(simp):
    solver = Solver2D(60, 20, simp)
    fixed_nodes = np.arange(21)
    fixed_dofs = np.union1d(2 * fixed_nodes, 2 * fixed_nodes + 1)
    force = np.zeros(solver.ndof)
    load_node = (60 + 1) * (20 + 1) - 1 - 20 // 2
    force[2 * load_node + 1] = -1.0
    densities, history = solver.solve(fixed_dofs, force, collect_timing=True)
    return {
        "name": "cantilever_2d",
        "shape": [60, 20],
        "iterations": len(history),
        "final_compliance": float(history[-1]),
        "mean_density": float(np.mean(densities)),
        "timing": _timing_dict(solver.last_timing),
    }


def run_3d_case(simp):
    solver = Solver3D(20, 8, 6, simp)
    left_nodes = []
    for j in range(9):
        for k in range(7):
            left_nodes.append(j * 7 + k)
    left_nodes = np.array(left_nodes)
    fixed_dofs = np.union1d(
        np.union1d(3 * left_nodes, 3 * left_nodes + 1),
        3 * left_nodes + 2,
    )
    nyz = (8 + 1) * (6 + 1)
    load_node = 20 * nyz + (8 // 2) * (6 + 1) + (6 // 2)
    force = np.zeros(solver.ndof)
    force[3 * load_node + 2] = -1.0
    densities, history = solver.solve(fixed_dofs, force, collect_timing=True)
    return {
        "name": "cantilever_3d",
        "shape": [20, 8, 6],
        "iterations": len(history),
        "final_compliance": float(history[-1]),
        "mean_density": float(np.mean(densities)),
        "timing": _timing_dict(solver.last_timing),
    }


def run_monocoque_case(cfg):
    resolved = resolve_monocoque_case(cfg, "torsion", element_size_mm=100.0)
    solver = resolved.tub.build_solver()
    densities, history = solver.solve(
        resolved.fixed_dofs,
        resolved.forces[0],
        designable=resolved.tub.designable,
        obstacle=resolved.tub.obstacle,
        x_init=resolved.tub.x_init,
        continuation=True,
        collect_timing=True,
    )
    return {
        "name": "monocoque_torsion",
        "shape": [resolved.tub.nelx, resolved.tub.nely, resolved.tub.nelz],
        "iterations": len(history),
        "final_compliance": float(history[-1]),
        "mean_density": float(np.mean(densities)),
        "timing": _timing_dict(solver.last_timing),
    }


def main():
    cfg = load_config()
    baseline_simp = replace(
        cfg.simp,
        max_iterations=40,
        convergence_tolerance=0.02,
    )
    baseline_cfg = replace(cfg, simp=baseline_simp)

    payload = {
        "profile": {
            "name": "stage1_performance_baseline",
            "notes": (
                "Uses the same coarse profile as the Stage 1 integration baseline "
                "so timing and behaviour can be compared directly."
            ),
            "simp": asdict(baseline_simp),
            "monocoque_element_size_mm": 100.0,
        },
        "cases": [
            run_2d_case(baseline_simp),
            run_3d_case(baseline_simp),
            run_monocoque_case(baseline_cfg),
        ],
    }

    path = output_path("performance_baseline.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")

    print(path)


if __name__ == "__main__":
    main()
