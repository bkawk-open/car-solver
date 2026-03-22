# GPU Follow-On Evaluation

Date: 2026-03-22

This document records the go/no-go decision for a first experimental GPU backend after the initial performance baseline and backend-boundary work.

## Decision

**No immediate GPU implementation in the main solver path.**

Recommended next step:

- keep SciPy as the reference backend
- use the new backend boundary in [linalg_backend.py](/Volumes/bkawk/projects/car-solver/src/car_solver/linalg_backend.py)
- only build a GPU prototype as an isolated experiment behind that boundary

## Why this is the current decision

## 1. The measured bottleneck is clear

From [performance-baseline.md](/Volumes/bkawk/projects/car-solver/docs/performance-baseline.md):

- 2D cantilever: factorization `73.4%` of runtime
- 3D cantilever: factorization `85.3%`
- monocoque torsion: factorization `90.3%`

So any GPU experiment should be judged mainly on whether it improves sparse factorization, not just sparse solves in isolation.

## 2. The current runtime environment is not a reliable CUDA target

The repo work in this session was executed in an environment where:

- the available local Python stack is CPU/SciPy-first
- CUDA availability is not something the repo can assume
- the standard development workflow should remain reproducible without GPU-only dependencies

That means GPU support should remain optional and experimental unless the project establishes a dedicated CUDA-capable runtime path.

## 3. The current problem sizes do not justify blind GPU work

The current measured baseline cases are:

- 2D: small and fast already
- 3D cantilever: moderate
- coarse monocoque: still modest compared with the sizes where GPU sparse methods usually become clearly compelling

That does not mean GPU work is pointless. It means the project should not assume a win without measuring it on the actual large monocoque cases that matter.

## Candidate paths

### Option A: CuPy sparse backend

Pros:

- closest conceptual fit to the new backend boundary
- potentially simplest first experiment in Python

Cons:

- benefit depends heavily on whether sparse factorization is actually accelerated well for the target matrices
- sparse support and performance characteristics can be uneven compared with dense GPU paths

Assessment:

- plausible first experiment
- not yet justified for mainline integration

### Option B: cuSOLVER-backed sparse factorization path

Pros:

- closer to the measured bottleneck
- more defensible than replacing only the solve phase

Cons:

- more implementation and environment complexity
- should not be attempted before a controlled experimental backend exists

Assessment:

- strongest technical direction if a GPU prototype is attempted
- should be treated as an experiment, not a commitment

### Option C: AMGX / heavier GPU solver stack

Pros:

- purpose-built for sparse systems

Cons:

- highest integration complexity
- least appropriate while the repo is still stabilizing its Stage 1 solver contract

Assessment:

- defer

## Go / no-go result

### Go

Go on:

- keeping the backend boundary in place
- profiling larger monocoque cases when available
- building one isolated experimental GPU backend only after a CUDA-capable environment is explicitly available

### No-go

No-go on:

- adding mandatory GPU dependencies to the repo
- claiming expected speedups without measurements
- prioritizing GPU work ahead of remaining solver/backend cleanup

## Recommended first experiment

If a CUDA-capable environment is available later, the first experiment should be:

1. keep the current SciPy backend as the control
2. add one optional experimental backend behind the same interface
3. target sparse factorization first, not just repeated solves
4. benchmark it on the largest monocoque case the team actually wants to use
5. compare total runtime and factorization time directly against the current baseline

## Final recommendation

The repo is now in the right shape to support a future GPU experiment.

But the correct decision today is:

**do not start GPU implementation as core product work yet.**

The evidence supports backend isolation first, then controlled experiments only when the runtime environment and target case sizes justify them.
