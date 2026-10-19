# Roadmap

Phases are milestones the owner can actually *use*. Inside a phase, work runs in parallel
**lanes** (one agent per lane at a time, each lane owns distinct crates and modules — see
AGENTS.md). Work-package (WP) IDs refer to the tables in `docs/specs/`. A WP is never
scheduled in an earlier phase than any of its dependencies.

## Lanes

| Lane | Specs | Crates / modules owned |
|---|---|---|
| A — Kernel & solid modeling | S02, S04, S05, S20 | `amg-occt-sys`, `amg-brep`, `amg-nurbs`, `amg-features::{solid,construct}`, `amg-ui::inspect` |
| B — Sketch | S03 | `amg-sketch`, `amg-features::sketch` |
| C — Mesh & implicit | S07, S08, S09 | `amg-mesh`, `amg-implicit`, `amg-subd`, `amg-features::{mesh,implicit,form}`, `amg-ui::{mesh,form}` |
| D — Viewport & UI framework | S10, S11 | `amg-render`, `amg-app`, `amg-ui` framework (shell, widgets, generic dialogs, viewport input, browser, timeline, sketch-mode UX) |
| E — Document & API | S00, S01, S06, S18 | `amg-core`, `amg-jobs`, `amg-doc`, `amg-script`, `amg-cli`, `amg-features::assembly` |
| F — Numerics & simulation | S15, S13, S14 | `amg-compute`, `amg-fem`, `amg-topopt`, `amg-features::sim`, `amg-ui::{sim,generative}` |
| G — AM, I/O, materials | S12, S16, S17 | `amg-io`, `amg-am`, `amg-materials`, `amg-ui::{am,io,materials}` |

**UI ownership rule:** lane D owns the `amg-ui` framework. Each domain lane owns its own
workspace/tool module inside `amg-ui` (listed above) and builds it only on the framework's
public API (command registry, tool state machine, generic dialogs, overlays). If a domain
module needs a framework change, that change is lane D's (see AGENTS.md §Interface changes).

## Decision gates (owner sign-off, see `docs/QUESTIONS.md`)

| Before starting | Needs |
|---|---|
| Every lane-A WP that calls OCCT: BREP-\*, SOL-\*, CON-\*, INS-03..07, KH-01..06, KH-08..11 | ADR-002 (OpenCASCADE) accepted |
| MESH-05 | ADR-003 (Manifold) accepted |
| RND-01, UI-01 | ADR-004 (wgpu) and ADR-005 (egui) accepted |
| SK-03 | ADR-007 (own sketch solver) decided — SK-03 refinement prototypes both options |

FND-01..06 have no gate, so Phase 0 can start before these are settled. INS-01, INS-02 and
KH-07 (`amg-nurbs`) don't touch OCCT and are not gated by ADR-002. The GitHub remote exists
(QUESTIONS #5), so FND-05 (CI) is unblocked.

## Phase 0 — Foundation (serial, small)
Goal: everyone can build; shared types exist.
`FND-01 → FND-02 → (FND-03, FND-04 in parallel)`, then `FND-03 → FND-06`.
Then spikes in parallel to de-risk the big unknowns: **BREP-01** (OCCT builds and links on
Windows), **RND-01** (mesh spinning in an egui window), **MESH-01**, **SK-01**, **DOC-01**.

## Phase 1 — "Mesh MVP" (first daily-usable version)
Open STL/3MF/OBJ → view → analyse → repair / remesh / reduce / smooth → booleans, plane cut,
shell → measure, section → overhang analysis on a build plate → export 3MF → open in slicer.
- A: INS-01 (measure on meshes), INS-02 (section analysis); in the background for Phase 2:
  BREP-02..04, BREP-08, KH-01
- B: SK-02..06 (in the background, preparing Phase 2; SK-01 is a Phase 0 spike)
- C: MESH-02..10, MESH-16, IMP-03
- D: RND-02..04, RND-06..08, UI-01..04, UI-07, UI-08, UI-10
- E: DOC-02..05, DOC-07, DOC-09, DOC-11 (persistent naming design note), FND-05, FND-07, FND-08
- G: IO-01, IO-02, IO-03, IO-05, AM-01, AM-02, AM-03, AM-06, MAT-01

## Phase 2 — Parametric core
Sketch → extrude/revolve → fillet/chamfer/shell → patterns → timeline edits that don't break
(persistent naming) → parameters → STEP in/out → components.
- A: BREP-05..07, BREP-09, BREP-10, SOL-01..09, CON-01, CON-02, INS-03, INS-07, KH-02, KH-03, KH-05, KH-07
- B: SK-07, SK-09
- D: UI-05, UI-06, UI-09, UI-12
- E: DOC-06, ASM-01, ASM-02
- G: IO-06, IO-09
- F (starts early, independent): CMP-01..03

## Phase 3 — Advanced modeling & lattices
Sweep/loft/holes/threads/text, mesh→B-rep, wall thickness, implicit modeling with lattices,
joints, direct mode, Python API.
- A: BREP-11, SOL-10..13, INS-04..06, KH-04, KH-06, KH-08 (loft builder lands with SOL-11)
- B: SK-08, SK-10, SK-11
- C: MESH-11..15, IMP-01, IMP-02, IMP-04..06, IMP-08
- D: RND-05, RND-09, UI-11
- E: DOC-08, ASM-03..05, SCR-01..03, SCR-05
- G: IO-04, IO-07, IO-08, IO-10, MAT-02, MAT-03, AM-07
- F: CMP-04, CMP-05, FEM-01..03

## Phase 4 — Simulation & generative design (the "paid Fusion" milestone)
Linear static + modal FEA, GPU topology optimization with AM overhang constraint, outcome →
printable body, orientation optimization and plate arrangement.
- F: FEM-04..08, FEM-13, GEN-01..07
- G: AM-04, AM-05
- C: IMP-07 (stress-driven lattice density)

## Phase 5 — Advanced parity
Thermal, buckling, nonlinear, outcome explorer, shape optimization, conformal lattices,
automated modeling, Form workspace, configurations, add-ins, motion, 3D sketch.
- F: FEM-09..11, GEN-08..10
- C: IMP-09, IMP-10, FRM-01..06
- A: SOL-14..16
- E: DOC-10, SCR-04, SCR-06, ASM-06, ASM-07
- B: SK-12
- G: AM-08, MAT-04
- D: RND-10

## Phase 6 — Later (P3)
FEM-12, GEN-11, GEN-12, FRM-07, FRM-08, SOL-17, ASM-08, AM-09, CMP-06, LAT-01..03 (S19 has no lane yet; assign one when the owner raises its priority and it is refined).
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
