# Milestone Plan

## Goal

This plan turns the current roadmap into issue-sized tasks with a practical implementation order for the next several weeks.

Primary near-term milestone:

**Milestone A: Credible Stage 1 Solver**

By the end of this milestone, the codebase should:

- run the documented 2D and 3D validation cases reproducibly
- define monocoque load cases from explicit physical loads and boundary conditions
- model monocoque geometry with explicit preserve/obstacle regions
- enforce at least two manufacturing constraints
- export and report results in a repeatable way
- capture enough performance data to support a later optimisation pass

## Priority System

- `P0`: blocking foundation work
- `P1`: high-value core implementation
- `P2`: supporting work that becomes valuable after the core is stable
- `P3`: nice to have, defer if the core is still moving

## Working Order

Implement in this order:

1. development workflow and testability
2. load-case architecture
3. monocoque geometry model
4. explicit preserve/load/boundary-condition semantics
5. first manufacturing constraints
6. export and reporting
7. integration baseline and gap review
8. profiling and optimisation planning

## Milestone Checklist

### Week 1: Foundations

#### Issue 1: Define Stage 1 software scope
- Priority: `P0`
- Depends on: none
- Files: `docs/`
- Task:
  - add a short software-only Stage 1 scope document derived from `docs/spec.md`
  - define what is in scope for the solver repo and what remains future-stage context
- Acceptance criteria:
  - there is a clear definition of done for Stage 1 software
  - authoritative deliverables are listed explicitly
  - exploratory features are identified as such

#### Issue 2: Document local development workflow
- Priority: `P0`
- Depends on: none
- Files: `README` or `docs/dev.md`
- Task:
  - document environment setup
  - document install commands
  - document how to run tests and main solver entry points
- Acceptance criteria:
  - a new developer can set up the repo from documented steps alone
  - the main run commands are listed and named consistently

#### Issue 3: Make test execution first-class
- Priority: `P0`
- Depends on: Issue 2
- Files: `pyproject.toml`, optional helper script or Make target
- Task:
  - ensure test dependencies are installable through a documented path
  - add one standard command for running the suite
- Acceptance criteria:
  - one documented command runs the test suite
  - test tooling is part of the normal dev setup

#### Issue 4: Audit current solver entry points
- Priority: `P1`
- Depends on: Issue 1
- Files: `docs/`, `src/car_solver/`
- Task:
  - classify current entry points as authoritative, exploratory, or deprecated
  - cover `monocoque_cross_section`, `torsion_load_case`, `bending_load_case`, `corner_load_case`, and `combined_load_case`
- Acceptance criteria:
  - each entry point has a documented role
  - exploratory shortcuts are clearly labeled

### Week 2: Load-Case Architecture

#### Issue 5: Introduce structured load-case dataclasses
- Priority: `P0`
- Depends on: Issue 4
- Files: `src/car_solver/loads.py` or new module
- Task:
  - define structured representations for force application, constrained DOFs, case metadata, and weighting
  - replace ad hoc force-vector construction where appropriate
- Acceptance criteria:
  - monocoque cases can be defined from structured objects
  - the interface cleanly separates load definition from solver execution

#### Issue 6: Refactor load derivation into solver-ready cases
- Priority: `P0`
- Depends on: Issue 5
- Files: `src/car_solver/loads.py`
- Task:
  - keep human-readable scalar summaries
  - add explicit solver-ready definitions for torsion, bending, front-corner, rear-corner, and combined weighting inputs
- Acceptance criteria:
  - code can generate physical load-case definitions directly from config
  - the mapping from spec values to solver inputs is explicit

#### Issue 7: Make torsion match the spec exactly
- Priority: `P1`
- Depends on: Issue 6
- Files: `src/car_solver/monocoque.py`, `tests/test_monocoque.py`, `tests/test_loads.py`
- Task:
  - ensure torsion uses equal-and-opposite vertical loads at the front corners with the intended rear constraints
  - remove or isolate any remaining symmetry shortcut assumptions for torsion
- Acceptance criteria:
  - torsion case matches the documented case in `docs/spec.md`
  - regression tests verify the setup

#### Issue 8: Separate authoritative cases from convenience cases
- Priority: `P1`
- Depends on: Issue 6
- Files: `src/car_solver/monocoque.py`, `docs/`
- Task:
  - keep `combined_load_case` as an exploratory workflow if desired
  - mark single-case runs as the authoritative source of truth
- Acceptance criteria:
  - docs and code both distinguish authoritative and exploratory cases
  - no convenience path is mistaken for the reference analysis

#### Issue 9: Add regression tests for spec load values
- Priority: `P0`
- Depends on: Issue 6
- Files: `tests/test_loads.py`, `tests/test_monocoque.py`
- Task:
  - add tests for torsion force magnitudes
  - add tests for bending total load
  - add tests for front and rear corner magnitudes and weighting splits
- Acceptance criteria:
  - changes to formulas cannot silently drift from the spec
  - the key numerical targets are enforced by tests

### Week 3: Monocoque Geometry Model

#### Issue 10: Replace the generic tub abstraction with named structural regions
- Priority: `P0`
- Depends on: Issue 6
- Files: `src/car_solver/monocoque.py`
- Task:
  - represent floor, sills, bulkheads, scuttle, cockpit void, drivetrain tunnel, and wheel-arch exclusions explicitly
  - keep the implementation simple but structurally legible
- Acceptance criteria:
  - geometry generation maps clearly to the spec
  - obstacle and structural regions are named, not implied

#### Issue 11: Add explicit preserve semantics
- Priority: `P1`
- Depends on: Issue 10
- Files: `src/car_solver/monocoque.py`, optional shared data structure module
- Task:
  - promote preserve from an implicit convention to a first-class mask or state
  - keep designable, obstacle, and preserve distinct
- Acceptance criteria:
  - the code can distinguish must-stay-solid from currently-solid
  - preserve regions are testable and visible in geometry setup

#### Issue 12: Tag load and boundary-condition regions explicitly
- Priority: `P1`
- Depends on: Issue 10
- Files: `src/car_solver/monocoque.py`
- Task:
  - tag nodes or elements used for load application
  - tag nodes or elements used for supports and constraints
- Acceptance criteria:
  - load locations and supports can be inspected programmatically
  - geometry is no longer the only source of truth for case setup

#### Issue 13: Add geometry regression tests
- Priority: `P0`
- Depends on: Issues 10-12
- Files: `tests/test_monocoque.py`
- Task:
  - test obstacle placement
  - test preserve placement
  - test pickup placement
  - test structural region coverage and consistency
- Acceptance criteria:
  - geometry changes are guarded by tests
  - region placement errors fail fast

### Week 4: First Manufacturing Constraints

#### Issue 14: Implement minimum wall thickness constraint
- Priority: `P1`
- Depends on: Issue 11
- Files: `src/car_solver/solver2d.py`, `src/car_solver/solver3d.py`, or new constraint module
- Task:
  - implement a simple enforceable rule for minimum wall thickness
  - prefer a representation that can later be extended rather than hard-coded into one solver path
- Acceptance criteria:
  - thin unsupported members are suppressed or thickened predictably
  - the behavior is testable on synthetic cases

#### Issue 15: Implement minimum lattice cell size constraint
- Priority: `P1`
- Depends on: Issue 11
- Files: solver or new constraint module
- Task:
  - prevent sub-scale micro-features that would be non-manufacturable
  - align the parameter source with `ManufacturingConfig`
- Acceptance criteria:
  - tiny isolated features do not survive optimisation
  - the constraint is measurable and repeatable

#### Issue 16: Add dedicated tests for manufacturing constraints
- Priority: `P0`
- Depends on: Issues 14-15
- Files: `tests/`
- Task:
  - add focused synthetic cases for each constraint
  - verify both activation and non-activation paths
- Acceptance criteria:
  - each implemented constraint has direct test coverage
  - tests check behavior, not just that code runs

#### Issue 17: Add constraint reporting
- Priority: `P2`
- Depends on: Issues 14-15
- Files: reporting/output modules
- Task:
  - report which constraints were active in a run
  - report simple violation or correction metrics
- Acceptance criteria:
  - solver outputs include constraint metadata
  - users can tell what manufacturing logic affected a result

### Week 5: Outputs and Reporting

#### Issue 18: Add per-run summary artifacts
- Priority: `P1`
- Depends on: Issues 6, 10, 17
- Files: `src/car_solver/output.py` or new reporting module
- Task:
  - save config snapshot
  - save compliance history
  - save final density statistics
  - save geometry and case metadata
- Acceptance criteria:
  - each run leaves behind a usable summary artifact
  - run outputs are comparable across iterations of the codebase

#### Issue 19: Add thresholded mesh export
- Priority: `P1`
- Depends on: Issue 18
- Files: new export module
- Task:
  - export thresholded density results to a downstream-friendly mesh format
  - keep the export interface separate from visualisation
- Acceptance criteria:
  - monocoque runs can export geometry for downstream inspection
  - exported files are tied to a run manifest or summary

#### Issue 20: Capture iteration snapshots for timelapse generation
- Priority: `P2`
- Depends on: Issue 18
- Files: visualisation/reporting modules
- Task:
  - save density frames every N iterations
  - keep capture configurable to avoid slowing every run
- Acceptance criteria:
  - at least one case can produce a full iteration-history frame set
  - the capture path is reproducible

#### Issue 21: Add run manifest metadata
- Priority: `P1`
- Depends on: Issue 18
- Files: reporting/output modules
- Task:
  - record code path, case name, config values, and generated artifacts per run
- Acceptance criteria:
  - old run outputs can be traced back to how they were produced
  - artifact directories are self-describing

### Week 6: Integration and Hardening

#### Issue 22: Run the full Stage 1 baseline
- Priority: `P0`
- Depends on: Issues 9, 13, 16, 18
- Files: docs and reporting outputs
- Task:
  - run the 2D validation cases
  - run the 3D validation case
  - run the monocoque authoritative cases
- Acceptance criteria:
  - baseline cases execute end to end
  - results and summaries are stored in a comparable format

#### Issue 23: Compare authoritative cases to combined exploratory cases
- Priority: `P1`
- Depends on: Issue 22
- Files: docs/reporting outputs
- Task:
  - compare separate case results against the combined case result
  - document where the exploratory shortcut is acceptable and where it is misleading
- Acceptance criteria:
  - differences are documented rather than assumed away
  - combined mode has a clear engineering interpretation

#### Issue 24: Write a Stage 1 implementation status report
- Priority: `P1`
- Depends on: Issue 22
- Files: `docs/`
- Task:
  - summarize what is complete
  - summarize what remains
  - summarize what is intentionally deferred
- Acceptance criteria:
  - the repo has a current implementation status document
  - the next milestone can be scoped from evidence, not memory

#### Issue 25: Select the next major manufacturing constraint
- Priority: `P2`
- Depends on: Issue 24
- Files: `docs/`
- Task:
  - choose between vent connectivity, flow continuity, overhang logic, and print partitioning for the next milestone
  - justify the choice based on downstream usefulness and implementation risk
- Acceptance criteria:
  - the next milestone starts with a deliberate technical choice
  - the constraint order is documented

### Week 7: Profiling and Optimisation Planning

#### Issue 26: Add lightweight solver timing instrumentation
- Priority: `P1`
- Depends on: Issue 22
- Files: `src/car_solver/solver2d.py`, `src/car_solver/solver3d.py`, reporting modules
- Task:
  - capture wall-clock timing for matrix assembly
  - capture wall-clock timing for factorization
  - capture wall-clock timing for repeated solves within an iteration
  - make timing capture optional so it does not affect every run
- Acceptance criteria:
  - at least one baseline monocoque run can emit phase-by-phase timings
  - timing output is structured enough to compare runs

#### Issue 27: Reduce duplicated expensive test solves
- Priority: `P1`
- Depends on: Issue 3
- Files: `tests/`
- Task:
  - use pytest fixtures or equivalent structure to avoid rerunning identical expensive solves across tests
  - preserve test clarity while reducing repeated numerical work
- Acceptance criteria:
  - obvious duplicate 2D and 3D solves are shared
  - test intent remains easy to read

#### Issue 28: Add optional parallel test execution guidance
- Priority: `P2`
- Depends on: Issue 3
- Files: `pyproject.toml`, `docs/dev.md`
- Task:
  - document optional parallel test execution as a developer convenience
  - keep the default test command simple and stable
- Acceptance criteria:
  - developers have a documented path for parallel test runs
  - the baseline workflow does not require parallel tooling

#### Issue 29: Write a solver performance baseline report
- Priority: `P1`
- Depends on: Issue 26
- Files: `docs/`, reporting outputs
- Task:
  - capture representative timings for 2D baseline, 3D baseline, and at least one monocoque case
  - identify whether runtime is dominated by assembly, factorization, repeated solves, or Python overhead
- Acceptance criteria:
  - the repo contains a concrete performance baseline
  - future optimisation work can be judged against measured data

#### Issue 30: Design a solver-backend abstraction
- Priority: `P1`
- Depends on: Issue 29
- Files: `src/car_solver/`
- Task:
  - define the interface boundary between solver logic and linear algebra backend
  - keep CPU SciPy as the reference backend
  - make room for future experimental backends without rewriting load/geometry code
- Acceptance criteria:
  - backend responsibilities are explicit
  - future optimisation work can be isolated behind a stable interface

#### Issue 31: Evaluate GPU acceleration as a follow-on experiment
- Priority: `P3`
- Depends on: Issue 30
- Files: `docs/`
- Task:
  - evaluate CuPy, cuSOLVER-backed paths, and AMGX against the measured bottlenecks
  - treat GPU support as optional and experimental unless profiling shows clear value
  - note environment constraints, including when a CUDA GPU is not available to the repo runtime
- Acceptance criteria:
  - GPU work is justified by measurements, not intuition
  - there is a written go/no-go decision for a first experimental backend

## Suggested Issue Batch Order

### Batch 1
- Issue 1: Define Stage 1 software scope
- Issue 2: Document local development workflow
- Issue 3: Make test execution first-class
- Issue 4: Audit current solver entry points
- Issue 5: Introduce structured load-case dataclasses
- Issue 6: Refactor load derivation into solver-ready cases

### Batch 2
- Issue 7: Make torsion match the spec exactly
- Issue 8: Separate authoritative cases from convenience cases
- Issue 9: Add regression tests for spec load values
- Issue 10: Replace the generic tub abstraction with named structural regions
- Issue 11: Add explicit preserve semantics

### Batch 3
- Issue 12: Tag load and boundary-condition regions explicitly
- Issue 13: Add geometry regression tests
- Issue 14: Implement minimum wall thickness constraint
- Issue 15: Implement minimum lattice cell size constraint
- Issue 16: Add dedicated tests for manufacturing constraints

### Batch 4
- Issue 17: Add constraint reporting
- Issue 18: Add per-run summary artifacts
- Issue 19: Add thresholded mesh export
- Issue 20: Capture iteration snapshots for timelapse generation
- Issue 21: Add run manifest metadata

### Batch 5
- Issue 22: Run the full Stage 1 baseline
- Issue 23: Compare authoritative cases to combined exploratory cases
- Issue 24: Write a Stage 1 implementation status report
- Issue 25: Select the next major manufacturing constraint

### Batch 6
- Issue 26: Add lightweight solver timing instrumentation
- Issue 27: Reduce duplicated expensive test solves
- Issue 28: Add optional parallel test execution guidance
- Issue 29: Write a solver performance baseline report
- Issue 30: Design a solver-backend abstraction
- Issue 31: Evaluate GPU acceleration as a follow-on experiment

## Definition of Done for Milestone A

Milestone A is complete when all of the following are true:

- the repo has a documented and reproducible dev/test workflow
- primary monocoque load cases are defined explicitly from config
- authoritative and exploratory solver paths are clearly separated
- monocoque geometry uses explicit structural, obstacle, and preserve regions
- at least two manufacturing constraints are implemented and tested
- runs produce summary artifacts and traceable output metadata
- a baseline run and implementation status report exist in `docs/`
- a measured solver performance baseline exists for future optimisation work
