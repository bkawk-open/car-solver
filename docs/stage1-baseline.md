# Stage 1 Baseline

This document records the first reproducible Stage 1 integration baseline for the current solver implementation.

## Baseline profile

- Date: 2026-03-22
- Command: `python3 scripts/run_stage1_baseline.py`
- Output artifact: `output/stage1_baseline.json`
- Profile name: `stage1_coarse_baseline`

The baseline uses a deliberately coarse profile so it can be rerun during development:

- SIMP iterations capped at `40`
- SIMP convergence tolerance relaxed to `0.02`
- monocoque voxel size set to `100 mm`

This is an integration baseline, not a final design-resolution run.

## Validation cases

### 2D

| Case | Grid | Iterations | Final compliance | Mean density | Solid fraction (`>= 0.5`) |
| --- | --- | ---: | ---: | ---: | ---: |
| cantilever | `60 x 20` | 40 | 71,589.34 | 0.2991 | 0.2483 |
| MBB | `90 x 30` | 40 | 225,171.05 | 0.3000 | 0.2993 |
| plate with hole | `60 x 60` | 40 | 69,620.48 | 0.2835 | 0.2839 |

All three 2D validation cases executed end to end and landed close to the intended volume fraction. The plate-with-hole mean density stays below `0.30` because the void region is excluded from the usable material budget.

### 3D

| Case | Grid | Iterations | Final compliance | Mean density | Solid fraction (`>= 0.5`) |
| --- | --- | ---: | ---: | ---: | ---: |
| cantilever | `20 x 8 x 6` | 40 | 73.09 | 0.2991 | 0.3000 |

The 3D validation case also executed end to end and stayed aligned with the configured volume fraction.

## Monocoque baseline

All monocoque cases used the same coarse geometry:

- grid: `24 x 15 x 5`
- element size: `100 mm`
- iterations: `40`

| Case | Kind | Final compliance | Mean density | Solid fraction (`>= 0.5`) |
| --- | --- | ---: | ---: | ---: |
| torsion | authoritative | 57,431,236.05 | 0.6130 | 0.6244 |
| bending | authoritative | 25,033,772.05 | 0.6114 | 0.6244 |
| corner | authoritative | 22,226,602.13 | 0.6121 | 0.6222 |
| combined | exploratory | 61,607,409.11 | 0.6118 | 0.6233 |

These cases all executed end to end, but this coarse baseline should be interpreted carefully:

- every monocoque case hit the `40` iteration cap
- the coarse profile is useful for structural-pattern comparison
- the coarse profile is not yet a calibrated structural-performance baseline

In particular, the torsion and bending compliance histories increased across the capped run, which is a sign that this coarse baseline is better suited to integration and topology comparison than to reporting absolute performance.

## Authoritative vs combined comparison

The combined case preserves roughly the same gross material usage as the authoritative single-case runs, but the density fields are still materially different.

| Comparison | Mean absolute density difference | Max absolute density difference | Solid fraction delta (`>= 0.5`) |
| --- | ---: | ---: | ---: |
| torsion vs combined | 0.0669 | 0.8325 | +0.0011 |
| bending vs combined | 0.0913 | 0.9158 | +0.0011 |
| corner vs combined | 0.1296 | 0.9158 | -0.0011 |

Interpretation:

- the combined case is close in total material usage
- the combined case is not close enough in local density distribution to stand in for any authoritative case
- the gap is largest for corner loading, which is the least well represented by one shared support condition and one shared optimisation trajectory

So the current engineering reading remains:

- `torsion`, `bending`, and `corner` are the reference analyses
- `combined` is a useful exploratory topology generator
- `combined` should not be used as a surrogate for case-specific structural conclusions

## Current artifact format

The baseline run now leaves behind a comparable machine-readable artifact:

- `output/stage1_baseline.json`

That JSON captures:

- the baseline profile
- 2D validation summaries
- 3D validation summaries
- authoritative monocoque case summaries
- exploratory combined-case summary
- density-difference comparisons between authoritative and exploratory results
