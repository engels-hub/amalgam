# Architecture Decision Records

Each ADR: context → decision → consequences. Status is `Accepted`, `Proposed` (needs owner
sign-off) or `Superseded by ADR-xxx`. Agents add new ADRs as `Proposed`; the owner accepts them.

---

## ADR-001 — Language: Rust (Accepted)
Rust for all first-party code. C++ only inside `-sys` FFI crates.
**Consequences:** need FFI for OCCT, Manifold, fTetWild. Python API via pyo3.

## ADR-002 — B-rep kernel: OpenCASCADE behind a trait (Proposed)
**Context:** writing a NURBS B-rep kernel is a multi-year effort. Rust-native kernels
(truck, Fornjot) are not mature enough for fillets, shells, drafts, STEP.
**Decision:** use OCCT 7.8+ through a hand-written `cxx` bridge (`amg-occt-sys`), wrapped by a
Rust `BrepKernel` trait in `amg-brep`. No OCCT type leaks past `amg-brep`.
`opencascade-rs` is a reference for the build setup but we own our bindings (it covers too little).
**Consequences:** C++ build dependency (OCCT via vcpkg on Windows); the trait keeps the door open to
swap or mix in `truck` later. OCCT's weak spots (fillet robustness, boolean speed) are mitigated by
the mesh path for AM workflows.
**Note (2026-10-19, S00 refinement):** the vcpkg `opencascade` port is now 8.0.0. BREP-01 decides
whether to take 8.0.0 or pin 7.9.x.

## ADR-003 — Mesh booleans: Manifold (Accepted 2026-10-20)
Use the Manifold library (C++, Apache-2.0) for mesh booleans, which guarantees manifold output.
Everything else in `amg-mesh` (repair, remesh, decimate) is written in Rust.
**Amendment (S07 refinement):**
- We own the bindings: a new `amg-manifold-sys` crate over Manifold's C API (`manifoldc`).
  `manifold-csg-sys` is the fallback only if that route fails.
- Manifold and oneTBB (Apache-2.0) are built via a vcpkg overlay port, with
  `MANIFOLD_CROSS_SECTION=OFF`.
- `MeshGL64` is used for f64 exchange.
- Everything sits behind a `manifold` cargo feature. Without it, `boolean()` returns
  `MeshError::BackendUnavailable`.

Details: `docs/notes/mesh-processing.md` §4.
**Consequences:** new lane-C crate `amg-manifold-sys`, which architecture §Crates and the S00
layering allow-list must include.

## ADR-004 — GPU: wgpu for rendering and compute (Proposed)
One `wgpu` device for both. WGSL compute for topology optimization, SDF evaluation, CG solvers.
Optional `cudarc` backend behind a feature flag for solvers only if benchmarks show wgpu is a
bottleneck. Owner's GPU is an RTX 5060 Ti (NVIDIA), so CUDA is available if needed.

## ADR-005 — UI toolkit: egui + egui_dock (Proposed)
Immediate-mode UI is fast to iterate on and fits tool-heavy apps; `egui_dock` for dockable panels,
`egui-wgpu` to share the device with the viewport. Alternative considered: Slint (prettier, slower
to iterate), iced (less mature docking). Revisit if egui's look becomes a blocker.

## ADR-006 — Math: glam (f64) + nalgebra/faer where needed (Proposed)
`glam::DVec3/DMat4/DQuat` for geometry (f64 everywhere in the model), `glam::Vec3` (f32) only on
the GPU side. `nalgebra` for small dense linear algebra (sketch solver), `faer` for large sparse
systems (FEA).
**Clarifications (2026-10-19 refinements of S00 and S07):**
- glam f64 types are the only math types in cross-crate public APIs. `nalgebra` and `faer`
  stay crate-internal and convert at the boundary.
- On the CPU, f32 is allowed only for outward-rounded BVH node bounds and for rebased render
  upload buffers. Neither stores model geometry.

## ADR-007 — Sketch constraint solver: own Rust solver primary; libslvs as test oracle (Proposed)
**Decision:**
- A native Rust solver in `amg-sketch` is the only shipped solver. It uses substitution,
  Dulmage–Mendelsohn decomposition, damped weighted min-norm Gauss–Newton/LM, and QR-based DOF
  analysis and diagnosis, modelled on SolveSpace and FreeCAD's PlaneGCS.
- SolveSpace `libslvs` (GPL-3.0) is available only behind cargo feature `slvs`, as a
  differential-test oracle. ADR-012 forbids it being the sole solver. It stays test-only unless
  SK-13 shows a > 2 % corpus gap.
- ezpz (MIT) is a test-case source and fallback candidate.

**Status:** the decision rule in `docs/notes/sketch-solver.md` §5.4 was accepted on 2026-10-20.
This ADR becomes Accepted when the SK-13 benchmark report meets that rule. SK-03 waits for it.

## ADR-008 — Tet meshing: fTetWild via FFI (Proposed)
fTetWild (MPL-2.0) is robust to dirty input, which matters for meshes from scans/generative.
TetGen (AGPL) as a fallback for quality-constrained meshing. Personal use, so licences are OK,
but keep each behind a feature flag.

## ADR-009 — Subdivision instead of T-splines for the Form workspace (Proposed)
Fusion's Form workspace uses T-splines. We implement Catmull–Clark SubD with creases (OpenSubdiv
semantics) and SubD→NURBS conversion for B-rep interop. Equivalent workflow, well-documented math.

## ADR-010 — Document file format: zip + JSON + binary caches (Proposed)
See architecture §File format. JSON (serde) for the authored part keeps diffs and debugging easy;
caches are optional and rebuildable.

## ADR-011 — Scripting: Python via pyo3 (Proposed)
Python because it matches Fusion's API language and the scientific ecosystem (numpy for results).
Embedded interpreter for in-app scripts + a `amalgam` wheel for headless use.

## ADR-012 — Licensing (Proposed)
Personal-use project; GPL/AGPL dependencies are acceptable but must be listed in
`THIRD_PARTY.md` and isolated behind features so a permissive build remains possible.

## ADR-013 — Own Rust NURBS module; Rust kernel rewrite is the last phase (Proposed)
**Context:** OCCT lofts (`ThruSections`) and fillets (`ChFi3d`) are the operations that fail
most; the owner specifically hit unstable lofts in Fusion and wants this to be better.
**Decision:** add `amg-nurbs` (pure Rust) for surface *construction* where OCCT is weak, starting
with lofts (skinning, end conditions, Gordon surfaces for rails); OCCT keeps doing topology,
sewing, booleans and validation. Fillets get strategy wrappers and an analytic fast path first.
A Rust rewrite of the fillet engine, and any further replacement of OCCT operations, is
scheduled **after all other roadmap work (Phase 7)** and only ships if it beats the existing
path on the failure corpora (S20).
**Consequences:** extra crate in the domain layer; NURBS math shared later by Form (S09 SubD→NURBS).

## ADR-014 — Document state split, ids and undo model (Accepted 2026-10-20)
**Context:** architecture §Core data model puts body geometry and feature health inside the
document tree. That makes undo snapshots carry geometry, and recompute results would have to be
written back into authored state from worker threads. Random UUIDs in cache-relevant data break
determinism.
**Decision:**
1. `amg-doc` separates authored state (`Design`: timeline, parameters, component tree, body
   properties, extension records) from evaluated state (`Evaluated`: body geometry, sketches,
   construction geometry, naming maps, health, cache keys).
2. Authored state lives in persistent collections (`imbl`) and is snapshotted as `Arc<Design>`.
3. Undo/redo swaps snapshots, with no inverse commands. Evaluated state comes from a
   content-addressed cache (blake3 keys over feature data, parameter values and upstream keys),
   so undo is instant.
4. Authored ids are monotonic typed `u64`s persisted in the file. Body ids are derived as
   `(creating FeatureId, output slot)`. UUIDs identify documents only.

**Consequences:**
- Architecture §Core data model should show body geometry under evaluated state.
- Features never mutate authored state during recompute.
- The `amg-brep` `Shape` must be `Sync`. S02 currently says OCCT objects are "Send but not
  Sync", and S02 refinement must resolve this.
- Studies and print setups attach as extension records.
