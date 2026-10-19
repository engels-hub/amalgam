# Amalgam — Vision

Amalgam is a local-first, Rust-native CAD/CAE desktop application for personal use,
modelled on Autodesk Fusion's workflow but focused on **solid + mesh modeling for
additive manufacturing (AM)**. Everything — including the features Fusion runs in the
cloud or behind paid extensions (generative design, simulation, volumetric lattices,
additive build prep) — runs on the local machine, primarily on the GPU.

## Goals

1. **Feature parity with Fusion in the areas the owner uses**: sketching, parametric solid
   modeling, mesh workspace, inspection, assemblies (basic), import/export, simulation,
   generative design, additive manufacturing prep. See [03-fusion-parity.md](03-fusion-parity.md).
2. **Fast.** No server round-trips, no waiting on "cloud processing". Interactive operations
   must feel instant; heavy computations run in the background with progress and cancel.
3. **Mesh is a first-class citizen**, not an afterthought. Meshes, B-rep solids, implicit
   (SDF) volumes and subdivision surfaces coexist in one design and convert between each other.
4. **AM-aware throughout**: overhang analysis, build-direction-aware generative design,
   lattices, watertight output guarantees, 3MF as the primary export.
5. **Scriptable and headless**: every modeling operation is reachable from a Python API and
   a CLI, so designs and studies can be automated.

## Non-goals (for now)

- Cloud storage, collaboration, versioning server, team hubs.
- CAM / CNC toolpaths, electronics (ECAD/PCB), injection-molding simulation.
- Building our own slicer — we hand off to OrcaSlicer / PrusaSlicer via 3MF.
- Writing a B-rep kernel from scratch (we wrap OpenCASCADE behind a trait; see ADR-002).
- Reading native Fusion `.f3d` files (proprietary). STEP is the bridge.
- Cross-platform polish beyond what comes "for free". **Windows is the primary target**;
  Linux should keep building but is best-effort.

## Performance budgets (reference machine: RTX 5060 Ti, modern 8+ core CPU)

| Scenario | Budget |
|---|---|
| Cold start to interactive viewport | < 2 s |
| Viewport, 5 M-triangle mesh, orbit | ≥ 60 fps |
| Recompute of a typical 50-feature part after editing an early sketch | < 500 ms |
| Mesh boolean, two 1 M-triangle meshes | < 1 s |
| Linear static FEA, ~500 k DOF | < 30 s |
| Topology optimization, 1 M voxels, 100 iterations | < 5 min |
| UI never blocks for more than | 16 ms (everything heavier goes to the job system) |

## Principles

- **Determinism**: the same document recomputes to the same geometry (bit-stable within a build).
- **Never lose work**: autosave + crash-safe journal; every operation undoable.
- **Robust over clever**: booleans and exports must produce valid, watertight results or fail loudly.
- **Units**: internal length unit is the millimetre, angles in radians, SI for everything else.
