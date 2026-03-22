# Solver Entry Points

## Purpose

This document classifies the current public entry points in the repo as:

- authoritative: intended to represent the reference solver behavior for a documented problem
- exploratory: useful for fast iteration or visualisation, but not the final reference analysis
- support: helper utilities that are part of the workflow, but not structural analysis entry points

This classification is meant to guide refactoring. It gives us a stable target for the next load-case and geometry work.

## Classification Summary

### Authoritative

These are the entry points we should preserve and improve as the main Stage 1 analysis paths.

#### `car_solver.solver2d.cantilever_beam`
- File: `src/car_solver/solver2d.py`
- Role:
  - reference 2D validation case
- Why authoritative:
  - simple, standard SIMP benchmark
  - useful for solver correctness and regression checks

#### `car_solver.solver2d.mbb_beam`
- File: `src/car_solver/solver2d.py`
- Role:
  - reference 2D validation case
- Why authoritative:
  - standard symmetry benchmark for topology optimisation
  - useful for checking expected structural patterns

#### `car_solver.solver2d.plate_with_hole`
- File: `src/car_solver/solver2d.py`
- Role:
  - reference 2D validation case with obstacle handling and multi-load behavior
- Why authoritative:
  - validates more than a plain beam case
  - exercises obstacle masking and weighted multi-case solving

#### `car_solver.solver3d.cantilever_3d`
- File: `src/car_solver/solver3d.py`
- Role:
  - reference 3D validation case
- Why authoritative:
  - baseline 3D solver check before trusting monocoque-specific results

#### `car_solver.monocoque.torsion_load_case`
- File: `src/car_solver/monocoque.py`
- Role:
  - intended reference monocoque torsion analysis
- Why authoritative:
  - maps directly to a primary load case in the spec
- Current caution:
  - still uses normalized unit load magnitudes rather than explicit physical magnitudes

#### `car_solver.monocoque.bending_load_case`
- File: `src/car_solver/monocoque.py`
- Role:
  - intended reference monocoque bending analysis
- Why authoritative:
  - maps directly to a primary load case in the spec
- Current caution:
  - load definition still needs to become solver-ready from explicit physical case objects

#### `car_solver.monocoque.corner_load_case`
- File: `src/car_solver/monocoque.py`
- Role:
  - intended reference monocoque corner-load analysis
- Why authoritative:
  - maps to the spec’s front and rear corner loading intent
- Current caution:
  - case construction is still embedded directly in `monocoque.py`
  - should eventually be built from structured load-case definitions

### Exploratory

These are useful, but they should not be treated as the final structural answer.

#### `car_solver.solver2d.monocoque_cross_section`
- File: `src/car_solver/solver2d.py`
- Role:
  - quick 2D exploratory monocoque slice study
- Why exploratory:
  - reduced-dimensional model
  - simplified boundary conditions and geometry
  - best used to probe tendencies, not to make final decisions

#### `car_solver.monocoque.combined_load_case`
- File: `src/car_solver/monocoque.py`
- Role:
  - combined weighted monocoque study for fast iteration
- Why exploratory:
  - combines multiple cases into one workflow for convenience
  - shares a single boundary-condition set across cases
  - useful for trend exploration, not as a replacement for per-case reference runs

#### `python -m car_solver.visualise`
- File: `src/car_solver/visualise.py`
- Role:
  - visualisation and demo runner for 2D studies
- Why exploratory:
  - wraps multiple studies for convenience
  - oriented toward output generation and inspection rather than defining solver truth

#### `python -m car_solver.visualise3d`
- File: `src/car_solver/visualise3d.py`
- Role:
  - visualisation and demo runner for 3D baseline studies
- Why exploratory:
  - rendering-oriented
  - useful for inspection, not the core analysis API

#### `python -m car_solver.monocoque`
- File: `src/car_solver/monocoque.py`
- Role:
  - convenience runner for all current monocoque studies
- Why exploratory:
  - wraps both authoritative and exploratory case functions under one CLI-style path
  - useful for iteration, but not itself a source of analysis semantics

### Support

These are part of the workflow, but they are not structural solver entry points.

#### `car_solver.loads.calculate_load_cases`
- File: `src/car_solver/loads.py`
- Role:
  - scalar load derivation support
- Why support:
  - foundational for monocoque work
  - not yet the authoritative solver-ready case interface

#### `python -m car_solver.loads`
- File: `src/car_solver/loads.py`
- Role:
  - human-readable load summary output
- Why support:
  - useful for checking numbers and config assumptions
  - not an optimisation entry point

#### `car_solver.config.load_config`
- File: `src/car_solver/config.py`
- Role:
  - typed configuration loading
- Why support:
  - shared infrastructure rather than a solver workflow entry point

## Refactoring Guidance

The next load-case refactor should aim for this structure:

- keep the 2D and 3D benchmark cases authoritative for solver validation
- keep `torsion_load_case`, `bending_load_case`, and `corner_load_case` authoritative for monocoque analysis
- keep `monocoque_cross_section` and `combined_load_case` as exploratory studies unless they are later promoted by matching the same case-definition rules
- move case-definition logic out of ad hoc array construction and into shared load-case objects
- keep visualisation modules as thin wrappers over solver entry points rather than places where analysis semantics live
