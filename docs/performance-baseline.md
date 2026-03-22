# Solver Performance Baseline

Date: 2026-03-22

This document records the first measured solver performance baseline using the new opt-in timing instrumentation.

## Measurement source

- Command: `python3 scripts/run_performance_baseline.py`
- Output artifact: [performance_baseline.json](/Volumes/bkawk/projects/car-solver/output/performance_baseline.json)

The timing run uses the same coarse profile as the Stage 1 integration baseline:

- SIMP max iterations: `40`
- SIMP convergence tolerance: `0.02`
- monocoque voxel size: `100 mm`

This keeps behaviour and timing aligned with the current reproducible baseline rather than measuring a separate ad hoc setup.

## Representative cases

### 2D cantilever

- grid: `60 x 20`
- iterations: `40`
- total runtime: `0.206 s`

Phase breakdown:

- factorization: `0.151 s` (`73.4%`)
- assembly: `0.031 s` (`15.2%`)
- update: `0.013 s` (`6.3%`)
- repeated solves: `0.009 s` (`4.5%`)
- sensitivity filter: `0.001 s` (`0.4%`)

### 3D cantilever

- grid: `20 x 8 x 6`
- iterations: `40`
- total runtime: `2.772 s`

Phase breakdown:

- factorization: `2.364 s` (`85.3%`)
- assembly: `0.347 s` (`12.5%`)
- repeated solves: `0.040 s` (`1.5%`)
- update: `0.018 s` (`0.6%`)
- sensitivity filter: `0.001 s` (`0.1%`)

### Monocoque torsion

- grid: `24 x 15 x 5`
- iterations: `40`
- total runtime: `7.813 s`

Phase breakdown:

- factorization: `7.055 s` (`90.3%`)
- assembly: `0.624 s` (`8.0%`)
- repeated solves: `0.111 s` (`1.4%`)
- update: `0.021 s` (`0.3%`)
- sensitivity filter: `0.002 s` (effectively `0%`)

## What the measurements say

### 1. Sparse factorization is the dominant bottleneck

This is now measured rather than assumed:

- 2D: factorization is already the largest single cost
- 3D: factorization dominates the run
- monocoque: factorization is overwhelmingly dominant

That means future optimisation work should focus first on:

- reducing the number or cost of factorization steps
- changing the linear algebra backend only if it clearly improves factorization

### 2. Repeated solves are not the main problem right now

The repeated `lu.solve(...)` calls are a small share of total runtime in every representative case.

That means replacing only the solve step without addressing factorization is unlikely to move the needle much.

### 3. Python-side post-processing is not the current limiter

Sensitivity filtering and density update logic are a tiny fraction of runtime in these measured cases.

So the current optimisation priority is not generic Python cleanup. It is the sparse linear algebra path.

## Practical interpretation for next steps

These measurements support the current roadmap ordering:

1. keep performance work focused on measured bottlenecks
2. continue toward backend-boundary design rather than premature GPU coding
3. treat factorization cost as the main reference metric for future experiments

They also explain why:

- fixture-based test deduplication helped a little
- parallel test execution is optional but not a fundamental solver fix
- GPU work should be judged mainly on whether it reduces sparse factorization cost at useful problem sizes

## Immediate conclusion

The repo now has a concrete measured performance baseline.

The main takeaway is simple:

**factorization dominates runtime, increasingly so as problem size grows.**

That is the result future optimisation work should target first.
