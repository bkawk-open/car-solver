# Stage 1 Implementation Status

Status date: 2026-03-22

This report summarizes what is implemented in the repo after the first Stage 1 integration baseline.

## Complete enough for the current milestone

### Solver foundation

- 2D SIMP validation cases are implemented and reproducibly runnable
- 3D voxel SIMP is implemented and reproducibly runnable
- test execution is part of the normal repo workflow via `make test`
- the current test suite passes end to end

### Load-case layer

- monocoque load cases are defined through structured load-case objects
- torsion is implemented as the documented equal-and-opposite front vertical load case with rear support
- authoritative and exploratory monocoque paths are explicitly separated in code and docs
- numeric regressions now guard the spec-derived load magnitudes and weight splits

### Geometry layer

- the monocoque geometry now uses named preserve and obstacle regions
- preserve is a first-class concept, distinct from generic non-designable regions
- load application groups and support groups are tagged explicitly on the geometry model
- geometry regressions cover region presence, placement, and basic consistency

### Manufacturing constraints

- minimum wall thickness is implemented in 2D and 3D
- minimum lattice cell size is implemented in 2D and 3D
- both constraints have direct synthetic regression coverage
- constraint reporting is now emitted for monocoque runs

### Reporting and export

- monocoque runs can emit:
  - constraint reports
  - run summaries
  - run manifests
  - thresholded VTK export
  - thresholded STL export
  - optional iteration snapshots through `SNAPSHOT_EVERY`
- the baseline integration run now produces a comparable JSON artifact in `output/stage1_baseline.json`

## Implemented, but still provisional

### Monocoque baseline quality

The current monocoque path is structurally useful, but still provisional in engineering terms.

- coarse baseline runs execute end to end
- authoritative and exploratory results can be compared directly
- the current coarse baseline is not yet a calibrated structural-performance benchmark

Reasons:

- the coarse monocoque runs hit the iteration cap
- compliance values are sensitive to coarse resolution and current continuation settings
- manufacturing constraints are still first-pass approximations rather than process-complete rules

### Combined-case interpretation

The combined case is now properly documented and measured, but it remains exploratory.

- total material usage is similar to the single-case runs
- local density fields differ meaningfully from the authoritative cases
- corner loading is the least well approximated by the combined shortcut

This means the current combined path is acceptable for exploratory topology generation, but not for authoritative structural interpretation.

## Not complete for Stage 1 yet

### Manufacturing constraints still missing

The following spec-driven constraints are not implemented yet:

- vent connectivity
- flow-path continuity
- print orientation / overhang logic
- print-volume partitioning

### Reporting gaps

The reporting pipeline is improving, but still incomplete relative to the Stage 1 target.

- no iteration-frame manifest beyond the monocoque CLI path
- no case-by-case compliance tables generated automatically from normal runs
- no Stage 1 baseline dashboard or aggregated report beyond the JSON artifact and docs

### Performance groundwork still pending

The repo has not yet completed the planned optimisation groundwork.

- no optional timing instrumentation yet
- no formal performance baseline document yet
- no backend abstraction for future GPU experiments yet

## Intentionally deferred

The following are intentionally outside the current milestone:

- GPU acceleration and CUDA-specific solver backends
- vent/connectivity-aware manufacturing logic
- print partition planning
- live controls and interactive dashboards
- stress overlays and constraint-violation visualisation beyond simple counts

## Evidence-based next milestone starting point

The current evidence suggests the next work should stay in this order:

1. complete the remaining Stage 1 reporting and baseline-hardening work
2. add timing instrumentation and a performance baseline
3. only then start backend abstraction and GPU experiments

For manufacturing logic, the most useful next constraint is still whichever best improves downstream export usefulness without destabilizing the current solver contract. That choice should be made explicitly in the next milestone step rather than assumed.

That choice has now been made in [next-manufacturing-constraint.md](/Volumes/bkawk/projects/car-solver/docs/next-manufacturing-constraint.md):

- next constraint: flow-path continuity
- supporting data-model addition: explicit vent-connectivity semantics
