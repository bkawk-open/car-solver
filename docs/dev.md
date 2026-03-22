# Development Workflow

## Purpose

This document is the practical setup and run guide for the solver repo.

It covers:

- environment setup
- dependency installation
- test execution
- main solver entry points

## Requirements

- Python 3.12 or newer
- a shell environment that can run `python3`

The project currently declares:

- runtime dependencies in `pyproject.toml`
- dev dependencies in the optional `dev` extra

## Setup

### 1. Create and activate a virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 2. Install the project with dev dependencies

```bash
python3 -m pip install -e '.[dev]'
```

This installs:

- runtime dependencies such as `numpy`, `scipy`, `matplotlib`, `pyvista`, and `python-dotenv`
- dev tools such as `pytest` and `ruff`

## Configuration

The code loads configuration from:

1. `.env`
2. `.env.example` if `.env` does not exist

If you want local values, create a `.env` file at the repo root using the same keys expected by `src/car_solver/config.py`.

## Running Tests

Standard repo command:

```bash
make test
```

Underlying command:

```bash
python3 -m pytest -q
```

If that fails because dependencies are missing, confirm the virtual environment is active and that the dev dependencies were installed.

If `make` is unavailable in your environment, run the underlying command directly.

### Optional parallel test runs

The default workflow should stay on the standard serial command:

```bash
make test
```

If you want a faster local test pass and are willing to install optional tooling, install:

```bash
python3 -m pip install -e '.[parallel]'
```

Then you can use:

```bash
make test-parallel
```

Underlying command:

```bash
python3 -m pytest -q -n auto
```

Notes:

- this is a developer convenience, not the baseline workflow
- parallel execution may help on machines with multiple cores, but speedup will vary
- keep `make test` as the default documented path for reproducibility and simpler debugging

## Main Entry Points

### 2D visualisation and baseline runs

Run all current 2D demo cases:

```bash
python3 -m car_solver.visualise all
```

Run a specific 2D case:

```bash
python3 -m car_solver.visualise cantilever
python3 -m car_solver.visualise mbb
python3 -m car_solver.visualise plate
python3 -m car_solver.visualise monocoque
```

### 3D visualisation and baseline runs

Run the default 3D cantilever case:

```bash
python3 -m car_solver.visualise3d
```

Run a custom 3D cantilever grid:

```bash
python3 -m car_solver.visualise3d 30x10x6
```

### Monocoque runs

Run all current monocoque cases:

```bash
python3 -m car_solver.monocoque all
```

Run individual monocoque cases:

```bash
python3 -m car_solver.monocoque torsion
python3 -m car_solver.monocoque bending
python3 -m car_solver.monocoque corner
python3 -m car_solver.monocoque combined
```

### Load summary

Print the current derived vehicle loads:

```bash
python3 -m car_solver.loads
```

## Output Locations

Generated artifacts are written under the repo `output/` directory.

Typical outputs include:

- convergence plots
- density images
- VTK files

## Current Run Classification

Use the current entry points with the following intent:

- `car_solver.visualise` cases: baseline validation and exploratory studies
- `car_solver.visualise3d`: baseline 3D validation and rendering
- `car_solver.monocoque torsion|bending|corner`: intended structural monocoque studies
- `car_solver.monocoque combined`: exploratory combined weighting study

As the solver matures, this document should be updated to reflect which paths are authoritative versus convenience workflows.

For the current detailed classification, see `docs/entry-points.md`.

## Troubleshooting

### `No module named pytest`

Install dev dependencies:

```bash
python3 -m pip install -e '.[dev]'
```

### `make: command not found`

Use the direct command instead:

```bash
python3 -m pytest -q
```

### `ModuleNotFoundError` for project imports

Make sure you installed the project in editable mode:

```bash
python3 -m pip install -e '.[dev]'
```

### Headless render issues

Some `pyvista` image rendering paths may behave differently in headless environments. If a PNG render fails, check whether the underlying data export still succeeded in `output/`.

## Working Agreement

Before changing solver behavior, prefer this order:

1. add or update tests
2. make the solver or geometry change
3. regenerate outputs only after behavior is validated

That keeps exploratory rendering work from getting ahead of solver correctness.

For performance work, use this order:

1. get the authoritative solver path correct
2. measure representative runs
3. remove obvious duplicated work
4. add instrumentation and compare results
5. only then experiment with alternative backends such as GPU paths

That keeps optimisation work grounded in actual bottlenecks rather than assumptions.
