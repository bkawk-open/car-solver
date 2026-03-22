# Next Manufacturing Constraint Choice

Decision date: 2026-03-22

## Chosen next constraint

The next major manufacturing constraint should be:

**Flow-path continuity, implemented together with explicit vent-connectivity semantics.**

## Why this is the right next step

This is the best next constraint based on downstream usefulness and implementation risk.

### 1. It is central to the actual process

The Stage 1 solver is not just a generic topology optimiser. The process in [spec.md](/Volumes/bkawk/projects/car-solver/docs/spec.md) depends on a printed core that can be filled or injected reliably, with void regions connecting to the outer vent network.

That makes flow continuity more core to the process than:

- overhang logic, which matters later in print preparation
- print partitioning, which matters later once geometry size is stable

If the internal void network is not connected, the current exported topology can look plausible while being fundamentally incompatible with the intended manufacturing process.

### 2. It fits the current data model trajectory

The repo now already has:

- explicit voxel masks
- preserve / obstacle / designable semantics
- named geometry regions
- first-pass manufacturing constraints operating on voxel fields

Flow continuity is the next natural step because it extends this voxel logic rather than requiring a separate planning layer.

It also lines up with the missing spec flag:

- vent-connected flag

So the implementation work can improve both the element state model and the manufacturing logic at the same time.

### 3. It improves export usefulness immediately

We now have:

- thresholded VTK export
- thresholded STL export
- run summaries
- manifests

The next useful question for exported geometry is not just "is it thick enough?" but "does the internal void network remain manufacturable?"

Flow-path continuity gives us a stronger go/no-go signal on exported topologies than overhang logic or partitioning would at this stage.

### 4. It is lower risk than partitioning and more process-relevant than overhang logic

Print partitioning depends on:

- stable split-line conventions
- build-volume strategy
- later-stage geometry decisions

Overhang logic depends on:

- chosen print orientation
- support strategy assumptions
- whether geometry will be split before printing

Those are real constraints, but they are less stable right now.

Flow continuity is more self-contained:

- it can be checked directly on the voxel field
- it does not depend on a final orientation decision
- it can be validated with synthetic graph-connectivity tests

## What this choice means technically

The next constraint milestone should likely include:

1. Add explicit vent-connectivity semantics to the voxel/geometry model.
2. Define outer vent boundary regions on the monocoque geometry.
3. Implement connected-component or graph traversal checks on void regions.
4. Penalise or remove trapped void regions that do not connect to the vent network.
5. Report continuity violations in the existing run summary / manifest pipeline.

## Why the others are not next

### Overhang logic

Worth doing, but it should follow once:

- print orientation is represented explicitly
- export and geometry conventions are a bit more stable

### Print partitioning

Important later, but it is the highest-coupling option:

- build volume
- split interfaces
- join strategy
- downstream tooling assumptions

That makes it a poor next constraint while the geometry model is still settling.

### Vent connectivity as a separate first step

Vent connectivity matters, but on its own it is too narrow.

The better framing is:

- treat vent-connectivity semantics as the data-model foundation
- treat flow-path continuity as the actual next constraint

That gives us a complete engineering rule instead of just another flag with no enforcement.

## Result

The next manufacturing-constraint milestone should start with:

**flow-path continuity with vent-connectivity support**

That is the most useful, most defensible next step for making exported topologies closer to the real process described in the spec.
