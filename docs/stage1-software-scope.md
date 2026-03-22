# Stage 1 Software Scope

## Purpose

This document narrows `docs/spec.md` into the software deliverables that belong in this repository during Stage 1.

The goal is to make "Stage 1 complete" concrete for the solver codebase without mixing it with fabrication, equipment, or show-prep work from later stages.

## Stage 1 Definition

Stage 1 for this repository is complete when we have a credible, reproducible software pipeline for:

- defining vehicle and material assumptions from config
- deriving the primary structural load cases from those assumptions
- validating the optimisation approach in 2D
- extending the same approach to 3D voxel optimisation
- generating a monocoque structural problem that reflects the intended vehicle packaging
- applying the first manufacturing-aware constraints
- exporting and reporting results in a way that supports downstream design work

## In Scope

### 1. Configuration and derived inputs

The repo should own:

- typed configuration loading
- vehicle mass and geometry assumptions
- material property calculations
- safety-factor values
- optimisation parameters
- manufacturing constraint parameters

Current status:

- largely present in `src/car_solver/config.py`
- needs stronger integration with solver-ready load-case and geometry generation

### 2. Load-case derivation

The repo should own:

- primary load-case definitions from the spec
- explicit mapping from config values to solver-ready force and support definitions
- clear separation between authoritative physical cases and exploratory convenience cases

Authoritative Stage 1 load cases:

- torsion
- bending
- front corner loads
- rear corner loads
- combined weighted study across the above cases

Deferred but acknowledged Stage 1-adjacent cases:

- engine mount torque reaction
- scuttle local load case

These are still part of the broader spec, but they are not required to call the first solver milestone complete.

### 3. 2D solver validation

The repo should own a validated 2D SIMP implementation with known test problems.

Minimum expected validation cases:

- cantilever beam
- MBB beam
- plate with hole

Done means:

- tests exist
- results converge consistently
- the 2D code is treated as the mathematical baseline, not just a demo

### 4. 3D voxel solver

The repo should own:

- a working 3D SIMP implementation
- multi-load-case support
- obstacle and preserve handling
- reproducible convergence behavior on at least one baseline 3D case

Done does not mean production-scale performance yet. It means the formulation is trustworthy enough to support the monocoque work.

Performance optimisation is explicitly a later step than solver correctness. Stage 1 should leave the repo with enough instrumentation and structure to support optimisation work, but it should not trade away clarity or correctness to chase speed too early.

### 5. Monocoque structural model

The repo should own a monocoque problem definition that is structurally legible and clearly tied to the spec.

Minimum geometry concepts for the first credible version:

- floor
- sills
- front bulkhead
- rear bulkhead
- scuttle
- cockpit void
- drivetrain tunnel
- wheel-arch exclusions
- suspension pickup preserve regions

Minimum semantics the model should track:

- designable
- obstacle
- preserve
- load application regions
- boundary-condition regions

### 6. First manufacturing-aware constraints

The repo should implement the first two manufacturing constraints from the spec before Stage 1 software is considered credible.

Priority order:

1. minimum wall thickness
2. minimum lattice cell size

Follow-on constraints, not required for the first milestone:

- vent connectivity
- flow-path continuity
- print orientation / overhang control
- print-volume partitioning

### 7. Output and reporting

The repo should produce enough output to compare runs and support downstream decisions.

Minimum expected outputs:

- convergence history
- saved density field
- run metadata
- case metadata
- config snapshot
- thresholded geometry export

Nice to have but not required for the first milestone:

- live sliders
- stress overlays
- violation highlighting
- timelapse video generation

### 8. Performance groundwork

The repo should leave behind enough performance groundwork to make later optimisation work deliberate instead of speculative.

Minimum expected groundwork:

- timing instrumentation for key solver phases
- a performance baseline for representative 2D, 3D, and monocoque runs
- a clear boundary between solver logic and linear algebra backend assumptions

Not required for the first milestone:

- GPU acceleration
- CUDA-specific codepaths
- backend-specific solver tuning
- cluster or distributed execution

## Explicitly Out of Scope for This Milestone

The following are part of the broader project, but not required to complete the first Stage 1 software milestone in this repo:

- printer design and controls
- injection machine design
- autoclave design
- process validation panel planning beyond documenting constraints
- chassis, drivetrain, suspension, and body fabrication work
- GNRS / AMBR display planning
- materials lab correlation work beyond storing the hooks for future data

These should remain in `docs/spec.md` as project context, but they should not block current solver progress.

## Authoritative vs Exploratory Outputs

The repo should distinguish between:

- authoritative analyses: runs that match the documented physical case definitions
- exploratory analyses: runs that intentionally simplify or combine cases for fast iteration

Examples of exploratory outputs:

- cross-section studies used to probe geometry tendencies
- combined runs that reuse a convenience boundary-condition set
- reduced-resolution studies used only to compare solver behavior

Exploratory outputs are useful. They just cannot be treated as the final structural answer.

## Definition of Done

Stage 1 software is complete for this repository when all of the following are true:

- config, materials, and load assumptions are explicit and reproducible
- 2D baseline cases are validated and tested
- 3D solver behavior is stable on baseline cases
- monocoque geometry uses explicit structural, obstacle, and preserve regions
- primary load cases are solver-ready and clearly documented
- at least two manufacturing constraints are implemented and tested
- authoritative and exploratory run modes are clearly separated
- outputs and reporting are sufficient to compare and reuse results
- representative solver timings are captured for future optimisation work

## Next-Milestone Boundary

Once the above is complete, the next milestone should move toward:

- stronger manufacturing constraints
- better downstream geometry export
- physical test correlation hooks
- more realistic packaging and local load cases
- measured performance optimisation, potentially including alternative linear algebra backends
