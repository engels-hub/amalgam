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

## ADR-003 — Mesh booleans: Manifold (Proposed)
Use the Manifold library (C++, Apache-2.0) via its C API / existing Rust bindings for mesh booleans,
which guarantees manifold output. Everything else in `amg-mesh` (repair, remesh, decimate) is
written in Rust.

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

## ADR-007 — Sketch constraint solver: own Rust solver (Proposed)
Own Newton/Levenberg–Marquardt solver with graph decomposition and DOF analysis, modelled on
SolveSpace and FreeCAD's PlaneGCS. Fallback option: FFI to SolveSpace's `libslvs` (GPL-3 — fine
for personal use). Decision to be finalized during S03 refinement.

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
