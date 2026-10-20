# Roadmap

Phases are milestones the owner can actually *use*. Inside a phase, work runs in parallel
**lanes** (one agent per lane at a time, each lane owns distinct crates and modules — see
AGENTS.md). Work-package (WP) IDs refer to the tables in `docs/specs/`. A WP is never
scheduled in an earlier phase than any of its dependencies.

## Lanes

| Lane | Specs | Crates / modules owned |
|---|---|---|
| A — Kernel & solid modeling | S02, S04, S05, S20 | `amg-occt-sys`, `amg-brep`, `amg-nurbs`, `amg-features::{solid,construct,inspect}`, `amg-ui::{inspect,construct}` (solid features need no `amg-ui::solid`: they register through the generic dialogs, S11) |
| B — Sketch | S03 | `amg-sketch`, `amg-features::sketch` |
| C — Mesh & implicit | S07, S08, S09 | `amg-mesh`, `amg-manifold-sys`, `amg-implicit`, `amg-subd`, `amg-features::{mesh,implicit,form}`, `amg-ui::{mesh,form}` |
| D — Viewport & UI framework | S10, S11 | `amg-render`, `amg-app`, `amg-ui` framework (shell, widgets, generic dialogs, viewport input, browser, timeline, sketch-mode UX) |
| E — Document & API | S00, S01, S06, S18 | `amg-core`, `amg-jobs`, `amg-testkit`, `amg-doc`, `amg-script`, `amg-cli`, `amg-features::assembly` |
| F — Numerics & simulation | S15, S13, S14 | `amg-compute`, `amg-fem`, `amg-topopt`, `amg-features::sim`, `amg-ui::{sim,generative}` |
| G — AM, I/O, materials | S12, S16, S17 | `amg-io`, `amg-am`, `amg-materials`, `amg-features::{io,am}`, `amg-ui::{am,io,materials}` |

**UI ownership rule:** lane D owns the `amg-ui` framework. Each domain lane owns its own
workspace/tool module inside `amg-ui` (listed above) and builds it only on the framework's
public API (command registry, tool state machine, generic dialogs, overlays). If a domain
module needs a framework change, that change is lane D's (see AGENTS.md §Interface changes).

## Decision gates (owner sign-off, see `docs/QUESTIONS.md`)

| Before starting | Needs |
|---|---|
| Every lane-A WP that calls OCCT: BREP-\*, SOL-\*, CON-03, INS-04, INS-06, INS-07, INS-10, INS-11, INS-12, KH-01..06, KH-08..11, KH-16..19 | ADR-002 (OpenCASCADE) accepted — **cleared 2026-10-20** |
| MESH-05, MESH-27 | ADR-003 (Manifold) accepted — **cleared 2026-10-20** |
| RND-01, UI-01 | ADR-004 (wgpu) and ADR-005 (egui) accepted — **cleared 2026-10-20** |
| FND-09 | ADR-004 (wgpu) accepted — **cleared 2026-10-20** |
| SK-03 | ADR-007 (own sketch solver) decided — SK-13 prototypes and benchmarks both options (ungated) |

FND-01..08 and FND-10 have no gate, so Phase 0 can start before these are settled. INS-01, INS-02,
INS-03 (mesh mass properties), INS-05, INS-08, INS-09, CON-01, CON-02 (pure math), CON-04 (UI) and
KH-07, KH-12..15 (`amg-nurbs`) don't touch OCCT and are not gated by ADR-002. The GitHub remote exists
(QUESTIONS #5), so FND-05 (CI) is unblocked.

## Phase 0 — Foundation (serial, small)
Goal: everyone can build; shared types exist.
`FND-01 → FND-02 → (FND-03, FND-04 in parallel)`, then `FND-03 → (FND-06, FND-10)` and
`FND-04 → FND-09` (FND-09 gated by ADR-004). FND-05 (CI) also runs in Phase 0 after FND-02,
because BREP-01's done-when includes the CI vcpkg cache (S02 refinement 2026-10-20).
Then spikes in parallel to de-risk the big unknowns: **BREP-01** (OCCT builds and links on
Windows), **RND-01** (mesh spinning in an egui window), **MESH-01**, **SK-01**, **DOC-01**.

## Phase 1 — "Mesh MVP" (first daily-usable version)
Open STL/3MF/OBJ → view → analyse → repair / remesh / reduce / smooth → booleans, plane cut,
shell → measure, section → overhang analysis on a build plate → export 3MF → open in slicer.
- A: INS-01 (measure core on meshes), INS-08 (Measure tool), INS-02 (section caps), INS-09
  (Section Analysis tool), INS-03 (mesh physical properties); in the background for Phase 2:
  BREP-02, BREP-12, BREP-03, BREP-14, BREP-04, BREP-08, KH-01, KH-07 (`amg-nurbs` core)
- B: SK-02, SK-14, SK-13 (ADR-007 benchmark), SK-03..06, SK-15, SK-20, SK-21 (in the background,
  preparing Phase 2; SK-01 is a Phase 0 spike)
- C: MESH-28 (corpus), MESH-02..10, MESH-16..23, MESH-26, MESH-27, IMP-03
- D: RND-02..04, RND-06..08, RND-11 (navigation, view cube), RND-12 (lines, edges), UI-01..04,
  UI-07, UI-08, UI-10, UI-15 (dialog widgets), UI-19 (file ops, recovery), UI-20 (scene adapter),
  UI-21 (marking menu). Lane D is heavy here; RND-08's analysis variants or RND-11's view cube may
  slide to late Phase 1.
- E: DOC-02..05, DOC-12, DOC-07, DOC-09, DOC-14, DOC-11 (persistent naming design note),
  FND-07, FND-08
- G: IO-01, IO-02, IO-03, IO-11 (3MF Generic writer), IO-04 (3MF Orca/Bambu), IO-05, IO-13 (PLY),
  IO-14 (import feature), IO-15 (I/O UI), IO-16 (fuzzing), AM-01, AM-03, AM-02, AM-06, AM-11
  (Manufacture workspace), AM-12 (overhang overlay), MAT-01, MAT-05 (built-in library), MAT-07
  (`LibrarySet`), MAT-06 (document integration). Lane G is heavy here: a second lane-G agent may
  take IO-05/IO-13/IO-16, and AM-12 may slip to early Phase 2 without blocking anything.

## Phase 2 — Parametric core
Sketch → extrude/revolve → fillet/chamfer/shell → patterns → timeline edits that don't break
(persistent naming) → parameters → STEP in/out → components.
- A: BREP-05..07, BREP-09, BREP-10, BREP-13 (STEP/IGES), BREP-15 (sweeps, raw loft), BREP-16
  (draft, split, defeature), BREP-17 (section, projection), SOL-01..09, CON-01..04, INS-07, INS-10,
  INS-11, KH-02, KH-03, KH-05, KH-12, KH-13, KH-16, KH-17
- B: SK-07, SK-16..19, SK-22, SK-09
- D: RND-13 (device loss, diagnostics), UI-05, UI-06, UI-09, UI-12, UI-13, UI-14, UI-16, UI-17,
  UI-22 (solid body display)
- E: DOC-06, DOC-13, ASM-01, ASM-02, FND-12
- G: IO-06, IO-09, AM-10 (slicer profile import)
- F (starts early, independent): CMP-01..03

## Phase 3 — Advanced modeling & lattices
Sweep/loft/holes/threads/text, mesh→B-rep, wall thickness, implicit modeling with lattices,
joints, direct mode, Python API.
- A: BREP-11, SOL-10..13, INS-04..06, INS-12, KH-04, KH-19, KH-06, KH-14, KH-15, KH-08, KH-18
  (loft builder lands with SOL-11)
- B: SK-08, SK-10, SK-11
- C: MESH-11..15, MESH-24, MESH-25, IMP-01, IMP-02, IMP-04..06, IMP-08
- D: RND-05, RND-09, UI-11, UI-18
- E: DOC-08, ASM-03..05, SCR-01..03, SCR-05
- G: IO-07, IO-08, IO-10, IO-12 (3MF Prusa), MAT-02, MAT-03, MAT-08, AM-07, AM-15 (resin
  printability)
- F: CMP-04, CMP-05, FEM-01..03

## Phase 4 — Simulation & generative design (the "paid Fusion" milestone)
Linear static + modal FEA, GPU topology optimization with AM overhang constraint, outcome →
printable body, orientation optimization and plate arrangement.
- F: FEM-04..08, FEM-13, GEN-01..07
- G: AM-04, AM-05, AM-13 (GPU orientation scoring), AM-08 (support preview, now P1)
- C: IMP-07 (stress-driven lattice density)

## Phase 5 — Advanced parity
Thermal, buckling, nonlinear, outcome explorer, shape optimization, conformal lattices,
automated modeling, Form workspace, configurations, add-ins, motion, 3D sketch.
- F: FEM-09..11, GEN-08..10
- C: IMP-09, IMP-10, FRM-01..06
- A: SOL-14..16
- E: DOC-10, SCR-04, SCR-06, ASM-06, ASM-07
- B: SK-12
- G: MAT-04, IO-18 (3MF beam lattice), IO-19 (3MF full materials)
- D: RND-10, RND-14 (large-mesh clusters/LOD)

## Phase 6 — Later (P3)
FEM-12, GEN-11, GEN-12, FRM-07, FRM-08, SOL-17, ASM-08, AM-09, AM-14, CMP-06, LAT-01..03 (S19 has no lane yet; assign one when the owner raises its priority and it is refined).
Ongoing from Phase 2 onward: KH-09 (upstream OCCT bug reports/patches).

## Phase 7 — Rust kernel (only after everything above is done)
Rewrite the fillet engine in Rust (KH-10), then replace further OCCT operations with Rust
implementations where the failure corpora show a clear win (KH-11). Each replacement ships
only if it beats the existing path on its corpus (ADR-013).

## Biggest risks (de-risk early)
1. **OCCT on Windows via Rust FFI** — BREP-01 spike in Phase 0.
2. **Persistent naming** — design note (DOC-11) before DOC-06; golden corpus from day one.
3. **Sketch solver robustness** — SK-03 benchmark against SolveSpace/PlaneGCS cases.
4. **GPU solver performance** (FEA/topopt) — CMP-05 benchmarks before GEN/FEM UI work.
5. **Scope** — 200+ WPs. Re-prioritize after Phase 1 based on actual use.
