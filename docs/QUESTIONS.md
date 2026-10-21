# Open questions for the owner

Agents append here; the owner answers inline and the refining agent moves the answer into the spec.

| # | Spec | Question | Answer |
|---|---|---|---|
| 1 | Parity | Review priorities in `03-fusion-parity.md`, especially rows marked `?` (surface tools, sheet metal, drawings). | 2026-10-20: split surface tools: patch, stitch, unstitch, reverse normal → P1 (import repair); ruled, extend, trim, untrim stay P2. Sheet metal P3. Drawings P3. Rest of the matrix as is. To be applied to `03-fusion-parity.md` at the next refinement. |
| 2 | S16 | Which printers / technologies (FFF, resin, SLS, metal) and which slicer (Orca, Prusa, Bambu Studio)? | FFF: Ender 3 V3 KE with MMU mod; MSLA resin printer. Slicers: OrcaSlicer (FFF), Chitubox (resin) (2026-10-20). Moved into S16. |
| 3 | S13 | Which loads/constraints do you use most in simulation? Do you need orthotropic printed-material models? | Loads first: fixed/pinned, force, pressure, bearing/pin, gravity, symmetry; then remote force, moment, thermal. Orthotropic (transversely isotropic, build direction from print setup) at P2 (2026-10-20). Moved into S13. |
| 4 | S04 | Can you supply 5–10 representative Fusion designs (STEP export + timeline screenshots) as parity references? | Yes, private, later (before Phase 2 S04 work); under `$AMG_TESTDATA_DIR/fusion/`, not committed (2026-10-20). Moved into S04. |
| 5 | S00 | Should the repo get a GitHub remote (enables CI)? | Yes — `github.com/engels-hub/amalgam` (2026-10-19). Moved into S00. |
| 6 | ADRs | Accept or change the `Proposed` ADRs (OCCT, Manifold, wgpu, egui, own sketch solver, SubD instead of T-splines, Python, own NURBS module). ADR-002 (OCCT), ADR-003, ADR-004/005 and ADR-007 gate specific work packages — see roadmap §Decision gates. Phase 0 (FND-01..04, FND-06) can start without them. | 2026-10-20: ADR-002 Accepted with amendment (OCCT 7.9.x or 8.0.x via vcpkg; BREP-01 pins). ADR-004 Accepted with amendment (CUDA backend is a priority for later performance work; wgpu stays the baseline). ADR-005 Accepted as written. ADR-006 Accepted with both clarifications. ADR-008 Accepted as written (fTetWild primary, TetGen fallback, each behind a feature). ADR-009 Accepted, with a note to add a T-spline→SubD translation layer later. ADR-010 Accepted as written. ADR-011 Accepted as written. ADR-012 Accepted as written (Amalgam's own licence left open). ADR-013 Accepted as written. All #6 ADRs settled except ADR-007 (waits for SK-13). |
| 7 | S20 | Do you have old Fusion designs with failing lofts? Exporting their section sketches (DXF/STEP) gives the loft test set real cases. | Yes, private, later (before KH-05); under `$AMG_TESTDATA_DIR/kernel/loft/owner/`, not committed (2026-10-20). Moved into S20. |
| 8 | S00 | Local toolchain: stay on VS Build Tools 2022 (17.14) while CI `windows-2025` ships VS 2026, or move local to VS 2026 so compilers match? *Default:* keep VS 2022 locally (S00 OQ 1). | Keep VS 2022 locally, `windows-2025` in CI (recommended default, 2026-10-20). Moved into S00. |
| 9 | S00 | Expected size of the `testdata/` corpus (GitHub LFS free tier: 10 GiB storage, 10 GiB/month bandwidth)? If it may exceed ~5 GiB, large corpora live outside the repo via `AMG_TESTDATA_DIR` (S00 OQ 2). | Hybrid: curated LFS set with a 2 GiB soft cap; large/private/non-redistributable data under `AMG_TESTDATA_DIR` with a sha256 manifest, skipped when absent (2026-10-20). Moved into S00. |
| 10 | S00 | Canonical units: adopt the coherent mm–t–s system (N, MPa, mass in tonnes, density in t/mm³) instead of the vision's "SI for everything else"? *Default:* adopt; see `docs/notes/units-and-quantities.md` §2 (S00 OQ 4). | Adopt mm–t–s (recommended default, 2026-10-20). Moved into S00. |
| 11 | S00 | Accept `,` as a decimal separator in input fields, or `.` only? *Default:* `.` only with a helpful error (S00 OQ 5). | `.` only (recommended default, 2026-10-20). Moved into S00. |
| 12 | S01 | Embed geometry caches in `.amg` by default (2–10× larger, instant open) or save compact and recompute on open? *Default:* embed, plus "Save compact" (S01 OQ 2). | Embed caches + "Save compact" (recommended default, 2026-10-20). Moved into S01. |
| 13 | S01 | Expressions: a bare number takes the default unit like Fusion (`d1 + 5` = `d1 + 5 mm`), or strict unit error? *Default:* lenient, with a strict-mode preference (S01 OQ 3). | Lenient, with strict-mode preference (recommended default, 2026-10-20). Moved into S01. |
| 14 | S01 | Persist undo history in the `.amg` file? *Default:* no (Fusion doesn't); only the crash journal persists commands (S01 OQ 4). | No (recommended default, 2026-10-20). Moved into S01. |
| 15 | S03 | ADR-007: accept the SK-13 decision rule in `docs/notes/sketch-solver.md` §5.4 (own Rust solver primary, `libslvs` only as a GPL test oracle)? Final decision after the SK-13 report; SK-03 waits for it (S03 OQ 1). | Decision rule accepted (recommended, 2026-10-20); ADR-007 final after SK-13. Moved into S03 and ADR-007. |
| 16 | S03 | Over-constraint UX: refuse an over-constraining dimension and offer to make it driven (Fusion, recommended), or allow it and show red (SolveSpace)? (S03 OQ 2) | Refuse + offer driven (Fusion, recommended, 2026-10-20). Moved into S03. |
| 17 | S03 | Near-closed gaps (≤ 0.01 mm, common in DXF): warn only in "sketch doctor" (default), or offer opt-in "heal gaps" that adds coincident constraints? Affects SK-06/SK-11 (S03 OQ 3). | Warn only (recommended default, 2026-10-20). Moved into S03. |
| 18 | S03 | Should `libslvs` ever ship as a runtime fallback in a GPL build, or stay test-only? *Recommended:* test-only unless SK-13 shows a > 2 % corpus gap (S03 OQ 4). | Test-only unless SK-13 shows > 2 % gap (recommended, 2026-10-20). Moved into S03. |
| 19 | S07 | Mesh corpus licensing: is the repo public or private? *Default:* commit only CC0/PD/CC-BY/CC-BY-SA models to LFS; NC/ND Thingi10K entries and Stanford scans stay fetch-only (S07 OQ 1). | Default plan: only permissive-licence models committed; NC/ND fetch-only (2026-10-20). Moved into S07. |
| 20 | S07 | Can you supply your own failing STLs/3MFs (scans, downloads, slicer failures) for `testdata/mesh/real/owner/`? (S07 OQ 2) | No (2026-10-20): MESH-28 uses the public corpus + synthetic generators only. Moved into S07. |
| 21 | S07 | ADR-003 acceptance with the proposed amendments (own `amg-manifold-sys` over the C API, Manifold + oneTBB via vcpkg, `MANIFOLD_CROSS_SECTION=OFF`)? MESH-05/MESH-27 wait on it (S07 OQ 3; text below). | Accepted with amendments (recommended, 2026-10-20). ADR-003 updated; MESH-05/MESH-27 ungated. |
| 22 | S07 | Phase 1 mesh dialogs: UI-05 (generic feature dialogs) depends on SOL-01 (Phase 2). Accept hand-built Phase 1 dialogs in MESH-23, migrated later, or should lane D decouple UI-05 from SOL-01? (S07 OQ 4) | Hand-built Phase 1 dialogs in MESH-23 (2026-10-20). Moved into S07. |
| 23 | S10, S11 | egui_dock 0.21.1 (latest) needs egui ^0.36; egui_tiles 0.18 already targets 0.37. If egui_dock has no 0.37 release when UI-01 starts: (a) pin egui/eframe/egui-wgpu to 0.36.2 until egui_dock 0.22 ships (egui-wgpu 0.36.2 also uses wgpu 30, so FND-09 and S10 are unaffected); (b) use `egui_tiles` 0.18 behind the same `amg_ui::dock::DockHost` facade; (c) use egui_dock from git. `amg-render` works with either egui version (S11 OQ 1, S10 OQ 2). *Recommended:* (a). See the ADR-005 amendment below. | (a): pin egui/eframe/egui-wgpu to 0.36.2 until egui_dock supports 0.37 (recommended, owner, 2026-10-20). Moved into S10, S11; ADR-005 amended. |
| 24 | S11 | Several documents as tabs in one window (Fusion), or one document per window? *Recommended:* tabs (S11 OQ 2). | Tabs in one window (recommended, owner, 2026-10-20). Moved into S11. |
| 25 | S11, S07 | Workspace grouping: follow current Fusion (Mesh and Form are tabs inside Design; Simulation, Generative and Manufacture-AM are workspaces), or keep Mesh and Form as separate workspaces as the Draft summary listed? Only lane C's registration data changes either way. *Recommended:* current Fusion (S11 OQ 3). | Follow current Fusion: Mesh and Form are tabs inside Design (recommended, owner, 2026-10-20). Moved into S11, S07. |
| 26 | S10 | Main monitor resolution and refresh rate? The 5 M-triangle ≥ 60 fps budget is specified at 2560×1440 with 4× MSAA. *Recommended:* measure at your native resolution with 4× MSAA; at 4K accept 2× MSAA if needed (S10 OQ 1). | Unknown; measure at native resolution with 4× MSAA (2× allowed at 4K); owner to confirm hardware (2026-10-20). Moved into S10. |
| 27 | S02, S20, S00 | OCCT 8.0.1 needs a vcpkg baseline bump. At the pinned vcpkg tag `2026.07.29` the `opencascade` port is 8.0.0 port-version 1 (verified locally); 8.0.1 needs a newer vcpkg tag. 8.0.1 fixes a `Standard_Strtod` thread race (parallel text `.brep`/STEP parsing can mis-parse numbers) and some ThruSections/boolean crashes, makes fillet state thread-local, and ships the reworked `GeomFill_Gordon` that S20 wants as engine and oracle (S20 R15). Options: (a) BREP-01 on 8.0.0; S02 keeps STEP and text `.brep` reads behind the global lock and S20 skips the OCCT Gordon path until a lane-E pin update moves the S00 vcpkg baseline to a tag with 8.0.1, before Phase 2 (BREP-13, KH-16); (b) bump the baseline now, before BREP-01. *Recommended:* (a). | (a): do not bump now (recommended, owner, 2026-10-20). Scheduled as new WP FND-13 (bump the vcpkg baseline to a tag shipping opencascade 8.0.1) in Phase 0 before BREP-01, which now depends on it. Moved into S00, S02, S20. |
| 28 | S05, S17 | Default material for bodies without an assigned material (mass properties only), and the mass display unit. Density: PLA 1.24 g/cm³ (AM-focused) or steel (Fusion's default)? Mass shown in g or kg by default? Simulation (S13) treats an unassigned body as an error either way. *Recommended:* PLA, changeable in preferences; mass in g (S05 OQ 1, S17 OQ 1). | PLA 1.24 g/cm³ default, changeable in preferences; mass in g (recommended, owner, 2026-10-20). Moved into S05, S17. |
| 29 | S05, S10 | Several visible section analyses: combine them (kept region = intersection of their kept sides, giving box sections), or only one visible at a time? Combining a full box needs S10 RND-06 to support 6 planes instead of 4 (requested under S10 Interface changes). *Recommended:* combine, up to 6 planes (S05 OQ 2). | Combine, up to 6 planes (recommended, owner, 2026-10-20). Moved into S05, S10 (RND-06 now 6 planes). |
| 30 | S12 | Default 3MF flavour for File > Export: Generic (strictly valid, opens everywhere, but Orca doesn't map its colours to extruders) or Orca/Bambu (multi-material extruders in Orca, not strictly valid because of `paint_color`)? *Recommended:* Generic; AM-06 "Send to OrcaSlicer" picks Orca automatically, and the dialog remembers the last choice. | Generic by default; AM-06 picks Orca automatically; the dialog remembers the last choice (recommended, owner, 2026-10-20). Moved into S12. |
| 31 | S12 | Non-watertight 3MF export: block by default with "Repair first" / "Export anyway", or warn only? *Recommended:* block (the vision's "fail loudly"). | Block by default with "Repair first" / "Export anyway" (recommended, owner, 2026-10-20). Moved into S12. |
| 32 | S12, S16 | Chitubox: S16 found Chitubox v1.2.0 and Chitubox Basic 2.x installed. Which edition do you print with, and does it import 3MF with object positions? This decides whether the resin hand-off is STL (default) or 3MF; the AM-06 manual check on your machine records the result. *Recommended:* STL hand-off until that check passes. | STL hand-off until the AM-06 manual 3MF check passes (owner, 2026-10-20). Moved into S12, S16. |
| 33 | S12, S00 | Add a vision performance budget "Import a 1 GB binary STL (20 M triangles) < 5 s"? FND-08's `BUDGETS` table and its row-count test then get one more entry (lane E). *Recommended:* yes. | Yes (recommended, owner, 2026-10-20). Row added to `docs/00-vision.md`; FND-08's `BUDGETS` table gets its entry when the FND-08 branch is merged. Moved into S12, S00. |
| 34 | S16 | Confirm the hardware found on your machine: resin printer **ELEGOO Mars 3 Pro** (143.43 × 89.60 × 175 mm, 4098 × 2560 px, from the Chitubox config) and MMU **CoPrint KCM** (4 filament slots, one nozzle; Orca preset "… - Connected KCM") on the Ender-3 V3 KE. These become built-in AM-01 machine profiles. *Recommended:* yes to both. | Yes — ELEGOO Mars 3 Pro and CoPrint KCM (assumed; owner may correct) (2026-10-20). Moved into S16. |
| 35 | S16, S17 | Default critical overhang angle: 45° (DfAM rule, Fusion default; flags more than Orca supports) or 30° (OrcaSlicer `support_threshold_angle` default, so red ≈ "Orca adds supports")? Editable per setup either way; S17 may later default it per material (optional `AmProps`, P1). *Recommended:* 45°, offering the imported Orca process value once AM-10 lands. | 45°, offering the imported Orca process value once AM-10 lands (recommended, owner, 2026-10-20). Moved into S16, S17. |
| 36 | S17 | Which resin brand/type do you print on the MSLA printer, and which filament brands do you use most? MAT-05 cites those datasheets first. *Recommended:* the generic set if no preference. | Generic set; no brand preference given (2026-10-20). Moved into S17. |
| 37 | S17, S16 | Default currency for material cost (S16 estimates)? *Recommended:* EUR. | EUR (recommended, owner, 2026-10-20). Moved into S17, S16. |
| 38 | S17 | Licence of the built-in material library data file? *Recommended:* same as the code, with per-value citations kept. | Same licence as the code, with per-value citations kept (recommended, owner, 2026-10-20). Moved into S17. |
| 39 | S17, S01 | Materials in the document: typed `materials`/`appearances` tables in `Design` with their own commands (S17 MAT-06), or the extension-record fallback? An additive S01 (lane E) change either way, see S01 Interface changes. *Recommended:* typed tables. | Typed tables (recommended, owner, 2026-10-20). Moved into S17, S01. |
| 40 | S20 | Fuzzy auto-escalation cap: may timeline recompute escalate the boolean fuzzy tolerance up to 1e-3 mm on its own, with a health warning (far below FFF/MSLA resolution), or stop at 1e-5 mm and require an explicit "try harder" above that? *Recommended:* auto up to 1e-3 mm with a warning (S20 OQ 3). | Auto up to 1e-3 mm with a health warning (recommended, owner, 2026-10-20). Moved into S20. |
| 41 | S20, S09, S00 | Layering: allow `amg-subd → amg-nurbs` so FRM-06 (SubD → NURBS) builds on `amg-nurbs` (ADR-013 "NURBS math shared later by Form")? Optionally also `amg-sketch → amg-nurbs` (not recommended now: S03 is Ready with its own `BSpline2`). Needs the architecture and allow-list change below. *Recommended:* allow `amg-subd → amg-nurbs` only (S20 OQ 4). | Allow `amg-subd → amg-nurbs` only (recommended, owner, 2026-10-20). Applied in the architecture crate table and the layering allow-list. Moved into S20, S09. |
| 42 | ADRs | Accept the wave-2 proposed ADR and architecture changes listed below (ADR-002 note, ADR-005 amendment, ADR-013 note, ADR-014 note, architecture threading paragraph, crate table and rendering-pipeline wording, and the `amg-subd → amg-nurbs` edge if #41 is accepted)? *Recommended:* accept all. | Accepted: all wave-2 changes (recommended, owner, 2026-10-20). Applied; see below. |
| 43 | S01 (DOC-11) | Pattern count change: does a ref into pattern instance *k* follow (a) the instance number quietly (Fusion/Onshape), (b) the instance number with a warning when it moved, or (c) the nearest instance by position? *Recommended:* (a). See `docs/notes/persistent-naming.md` §15 Q1. | |
| 44 | S01 (DOC-11) | Multi-entity selections (fillet edges, shell faces) when some members are lost: (a) drop lost/ambiguous members with a warning, (b) fail the feature, or (c) as (a), but a member split by an upstream feature expands to all its pieces with a warning? *Recommended:* (c). Note §15 Q2. | |
| 45 | S01 (DOC-11) | Should `Widened` (a split the ref saw no longer happens) and `NewSplit` (the ref predates a split) always warn, or resolve quietly when the match is clear? *Recommended:* both warn for now. Note §15 Q3. | |
| 46 | S01 (DOC-11) | A sketch curve deleted and redrawn in the same place: (a) rescue by fingerprint with a warning, (b) never rescue, or (c) (a) now plus a later "Replace curve" sketch tool that keeps the id? *Recommended:* (c). Note §15 Q4. | |
| 47 | S01 (DOC-11) | Accept the provisional resolver thresholds now (re-tuned by DOC-06/DOC-13, each change bumps `NAMING_VERSION`), wait for DOC-06 measurements, or make them user settings? *Recommended:* accept now; never per-user (ADR-014). Note §15 Q5. | |
| 48 | S01, S04 (DOC-11) | Simplify every boolean result (join/cut/combine) with unify-same-domain, as Fusion does, only joins, or never? *Recommended:* always. Note §15 Q6. | |
| 49 | S01, S11 (DOC-11) | Re-pick of a selection member keeps its selection key (downstream refs into the feature's faces follow the repair) or allocates a new one? *Recommended:* keep. Note §15 Q7. | |

## Proposed ADR changes (wave-1 refinement, 2026-10-19)

**Applied 2026-10-20** (owner: take the recommended side). In `docs/02-decisions.md`:
- ADR-014 added as Accepted; ADR-003 Accepted with the amendment.
- ADR-007 reworded and stays Proposed until SK-13.
- ADR-006 clarifications and the ADR-002 note added; both stay Proposed (QUESTIONS #6).

Original proposals:

- **ADR-014 (new), Document state split, ids and undo model** (from S01).
  - **Decision.**
    1. `amg-doc` separates authored state (`Design`: timeline, parameters, component tree,
       body properties, extension records) from evaluated state (`Evaluated`: body geometry,
       sketches, construction geometry, naming maps, health, cache keys).
    2. Authored state lives in persistent collections (`imbl`) and is snapshotted as
       `Arc<Design>`.
    3. Undo/redo swaps snapshots, with no inverse commands. Evaluated state comes from a
       content-addressed cache (blake3 keys over feature data, parameter values and upstream
       keys), so undo is instant.
    4. Authored ids are monotonic typed `u64`s persisted in the file. Body ids are derived as
       `(creating FeatureId, output slot)`. UUIDs identify documents only.
  - **Consequences.**
    - Architecture §Core data model should show body geometry under evaluated state.
    - Features never mutate authored state during recompute.
    - The `amg-brep` `Shape` must be `Sync`. This conflicts with S02's current "OCCT objects
      are Send but not Sync" and needs resolving in S02 refinement.
    - Studies and print setups attach as extension records.
- **ADR-007 wording** (from S03; see `docs/notes/sketch-solver.md`).
  - The native Rust solver in `amg-sketch` is the only shipped solver: substitution,
    Dulmage–Mendelsohn decomposition, damped weighted min-norm Gauss–Newton/LM, and QR-based
    DOF analysis and diagnosis.
  - SolveSpace `libslvs` (GPL-3.0) is available only behind cargo feature `slvs`, as a
    differential-test oracle, since ADR-012 forbids it being the sole solver.
  - ezpz (MIT) is a test-case source and fallback candidate.
  - Final acceptance comes after the SK-13 benchmark meets the decision rule in the note §5.4.
- **ADR-003 amendment** (from S07; see `docs/notes/mesh-processing.md` §4).
  - Own `amg-manifold-sys` crate over Manifold's C API (`manifoldc`).
  - Manifold + oneTBB built via a vcpkg overlay port.
  - `MeshGL64` for f64 exchange; `MANIFOLD_CROSS_SECTION=OFF`.
  - Everything behind a `manifold` cargo feature.
  - The new crate is added to lane C in the roadmap. Architecture §Crates and the S00
    layering allow-list need it too.
- **ADR-006 clarifications.**
  - From S00: "glam f64 types are the only math types in cross-crate public APIs; `nalgebra`
    and `faer` stay crate-internal."
  - From S07: "f32 is allowed on the CPU only for outward-rounded BVH node bounds and for
    rebased render upload buffers."
- **ADR-002 note** (from S00). The vcpkg `opencascade` port is now 8.0.0, while ADR-002 says
  "7.8+". BREP-01 decides whether to take 8.0.0 or pin 7.9.x.

## Proposed ADR changes (wave-2 refinement, 2026-10-20)

**Applied 2026-10-20** (owner accepted all, QUESTIONS #42): every item below is applied in
`docs/02-decisions.md` and `docs/01-architecture.md`, and the `amg-subd → amg-nurbs` edge also in
`crates/amg-testkit/tests/layering.rs` (QUESTIONS #41). The ADR-005 amendment records the owner's
choice for #23 (pin egui 0.36.2). Original proposals:

- **ADR-002 note** (from S02, S20; see `docs/notes/occt-integration.md` §1).
  - "BREP-01 pins OCCT 8.0.0 (vcpkg `opencascade` 8.0.0#1 at baseline `2026.07.29`); fallback
    7.9.3#1. The port is built with `default-features: false` and without TBB, so it can't clash
    with Manifold's oneTBB."
  - The move to 8.0.1 follows QUESTIONS #27. S20 supports 8.0.1 (reworked `GeomFill_Gordon`,
    thread-local fillet state).
- **ADR-014 note** (from S02; see `docs/notes/occt-integration.md` §4), under Consequences:
  "Resolved by S02 refinement 2026-10-20: `amg_brep::Shape` is an immutable `Arc` handle and is
  `Send + Sync`." No change to the decision.
- **ADR-005 amendment** (from S11; see `docs/notes/ui-framework.md`).
  - The docking crate sits behind `amg_ui::dock::DockHost`; egui_dock is preferred.
  - If egui_dock lags the pinned egui, either pin egui to the previous minor or use `egui_tiles`
    (MIT OR Apache-2.0) as the fallback; the owner chooses (QUESTIONS #23).
  - New egui widget crates are allowed only if they release in lockstep with egui or sit behind a
    facade.
- **ADR-013 note** (from S20; see `docs/notes/nurbs-library.md` §2): "Build decision confirmed
  2026-10-20: no crates.io NURBS crate covers approximate compatibility, end-conditioned skinning
  or correct Gordon surfaces. `curvo` (MIT) is used only as a dev-dependency test oracle."
- **Architecture §Threading & compute model** (from S02). Replace "OCCT is not thread-safe for all
  ops: the brep backend serializes calls per shape graph and only parallelizes independent
  operations" with: "`amg_brep::Shape` is immutable and `Send + Sync`. Kernel ops never mutate
  their inputs: non-destructive booleans, and private topology copies for meshing and healing.
  Only IGES I/O (and, on OCCT 8.0.0, text `.brep` and STEP reads) take a process-wide lock.
  Independent operations, including different ops on the same shape, may run in parallel."
- **Architecture §Crates** (from S02): the `amg-occt-sys` row "cxx → OpenCASCADE 7.8+" becomes
  "8.0.x (7.9.x fallback)".
- **Architecture §Crates + S00 layering allow-list** (from S20, only if QUESTIONS #41 is
  accepted): add `amg-nurbs` to `amg-subd`'s allowed deps.
- **Architecture §Rendering pipeline** (from S10, descriptive; see
  `docs/notes/viewport-rendering.md`): "Forward+ shading" → "forward shading (2–3 fixed studio
  lights)"; sections → "fragment discard (DX12 has no clip distances), one-pass back-face caps";
  add "on-canvas text painted by egui".
- No ADR change is needed for S05, S12, S16 or S17 (their reports checked ADR-006, ADR-010,
  ADR-012 and ADR-014).
