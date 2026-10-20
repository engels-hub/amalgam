# S13 — Simulation (FEA)

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P1 (static, modal) / P2 (thermal, buckling, nonlinear) / P3 (event) |
| **Crates** | `amg-fem`, `amg-features::sim`, `amg-ui::sim` (Simulation workspace) |
| **Lane** | F |
| **Depends on** | S07 (mesh), S15 (solvers), S17 (materials), S10 (result plots) |
| **Parity** | Simulation section — all of it cloud-solved in Fusion, local here |

## Summary
Local finite-element analysis with a Fusion-like study workflow: create study → simplify
geometry → assign materials → constraints → loads → contacts → mesh → solve (background job)
→ results (stress, displacement, safety factor, strain, reaction forces, modes) with
fringe plots, probes, deformation animation, and reports.

## Design
- **Study model** (`Study` stored in document, not in timeline): type, geometry snapshot ref
  (bodies by `EntityRef`, auto-updated when design changes → "out of date" badge), material
  overrides, BCs, loads, contacts, mesh settings, solver settings, results.
- **Geometry → analysis mesh:** B-rep tessellation or mesh body → fTetWild (ADR-008) →
  quadratic tets (TET10) by mid-node insertion snapped to geometry; local refinement; mesh
  quality report. Feature-based sizing (curvature, small features).
- **Boundary conditions:** fixed, pin (radial/axial/tangential), frictionless, prescribed
  displacement; **loads:** force, pressure, moment, bearing load, remote force, gravity,
  linear/angular acceleration, temperature. Applied to faces/edges/vertices via `EntityRef`
  mapped onto mesh facets.
- **Contacts:** bonded (default, via tied MPCs), separation/sliding (nonlinear — P2).
- **Element & assembly:** TET10 isoparametric, parallel assembly into CSR (faer), BC
  elimination or penalty; thermal: scalar Laplacian with conduction/convection/radiation (lin.).
- **Solvers (S15):** sparse Cholesky (faer/CHOLMOD) for ≤ 1–2 M DOF; GPU preconditioned CG
  (AMG or Jacobi/IC preconditioner) for larger; modal: shift-invert Lanczos / LOBPCG;
  buckling: generalized eigen with geometric stiffness.
- **Nonlinear (P2):** Newton–Raphson with load stepping; material nonlinearity (bilinear
  plasticity), large displacement (total Lagrangian), frictional contact (augmented Lagrangian).
- **Event simulation (P3):** explicit central-difference dynamics, GPU, mass scaling.
- **Results:** von Mises/principal stress, displacement magnitude & components, safety factor
  (yield-based), strain, reaction forces, contact pressure, mode shapes + frequencies;
  nodal averaging; result fields exported as `ScalarField` for S08 field-driven design and S14.
- **Verification:** NAFEMS benchmarks (LE1, LE10, FV32…), analytic cantilever, plate with hole (Kt).

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| FEM-01 | Study data model, persistence, out-of-date tracking | DOC-07 | P1 | Todo | |
| FEM-02 | Tet meshing via fTetWild (`-sys`), TET10 promotion, sizing controls, quality report | MESH-04 | P1 | Todo | meshes 95% of corpus without user intervention |
| FEM-03 | BC/load mapping from `EntityRef` to mesh entities | FEM-02, DOC-06 | P1 | Todo | |
| FEM-04 | Linear static: element, assembly, direct solve, results | FEM-03, CMP-03 | P1 | Todo | NAFEMS LE1/LE10 within 2% |
| FEM-05 | Simulation workspace UI: study tree, BC/load dialogs with glyphs, solve button | UI-05, FEM-01 | P1 | Todo | |
| FEM-06 | Results: fields, fringe plots, probes, deformation animation, safety factor, report export | FEM-04, RND-08 | P1 | Todo | |
| FEM-07 | Modal frequencies (Lanczos/LOBPCG) + mode shape display | FEM-04, CMP-04 | P1 | Todo | free-free bar modes analytic match |
| FEM-08 | GPU PCG solver path for large models | FEM-04, CMP-05 | P1 | Todo | 2 M DOF static < 60 s |
| FEM-09 | Thermal steady-state + thermal stress | FEM-04 | P2 | Todo | analytic slab/fin tests |
| FEM-10 | Linear buckling | FEM-07 | P2 | Todo | Euler column match |
| FEM-11 | Nonlinear static (plasticity, large deflection, contacts) | FEM-04 | P2 | Todo | NAFEMS nonlinear benchmarks |
| FEM-12 | Event simulation (explicit dynamics, drop test) | FEM-11 | P3 | Todo | |
| FEM-13 | Shared load-case model reused by generative design (S14) | FEM-03 | P1 | Todo | |

## Open questions
1. Owner: which load types are used most? Prioritize UI for them.
   **Answered 2026-10-20:** first-class UI for fixed/pinned, force, pressure, bearing/pin load,
   gravity and symmetry; remote force, moment and thermal loads follow.
2. Orthotropic / printed-material models (layer-dependent stiffness) — valuable for AM, P2?
   **Answered 2026-10-20:** yes, P2. Isotropic first (P1); then a transversely isotropic material
   with build direction taken from the S16 print setup, plus a directional failure criterion
   (Tsai-Wu or reduced Z allowables). Matters mainly for FFF; resin treated as isotropic.
