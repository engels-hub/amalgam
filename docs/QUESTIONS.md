# Open questions for the owner

Agents append here; the owner answers inline and the refining agent moves the answer into the spec.

| # | Spec | Question | Answer |
|---|---|---|---|
| 1 | Parity | Review priorities in `03-fusion-parity.md`, especially rows marked `?` (surface tools, sheet metal, drawings). | |
| 2 | S16 | Which printers / technologies (FFF, resin, SLS, metal) and which slicer (Orca, Prusa, Bambu Studio)? | |
| 3 | S13 | Which loads/constraints do you use most in simulation? Do you need orthotropic printed-material models? | |
| 4 | S04 | Can you supply 5–10 representative Fusion designs (STEP export + timeline screenshots) as parity references? | |
| 5 | S00 | Should the repo get a GitHub remote (enables CI)? | Yes — `github.com/engels-hub/amalgam` (2026-10-19). Moved into S00. |
| 6 | ADRs | Accept or change the `Proposed` ADRs (OCCT, Manifold, wgpu, egui, own sketch solver, SubD instead of T-splines, Python, own NURBS module). ADR-002 (OCCT), ADR-003, ADR-004/005 and ADR-007 gate specific work packages — see roadmap §Decision gates. Phase 0 (FND-01..04, FND-06) can start without them. | |
| 7 | S20 | Do you have old Fusion designs with failing lofts? Exporting their section sketches (DXF/STEP) gives the loft test set real cases. | |
| 8 | S00 | Local toolchain: stay on VS Build Tools 2022 (17.14) while CI `windows-2025` ships VS 2026, or move local to VS 2026 so compilers match? *Default:* keep VS 2022 locally (S00 OQ 1). | Keep VS 2022 locally, `windows-2025` in CI (recommended default, 2026-10-20). Moved into S00. |
| 9 | S00 | Expected size of the `testdata/` corpus (GitHub LFS free tier: 10 GiB storage, 10 GiB/month bandwidth)? If it may exceed ~5 GiB, large corpora live outside the repo via `AMG_TESTDATA_DIR` (S00 OQ 2). | |
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
| 20 | S07 | Can you supply your own failing STLs/3MFs (scans, downloads, slicer failures) for `testdata/mesh/real/owner/`? (S07 OQ 2) | |
| 21 | S07 | ADR-003 acceptance with the proposed amendments (own `amg-manifold-sys` over the C API, Manifold + oneTBB via vcpkg, `MANIFOLD_CROSS_SECTION=OFF`)? MESH-05/MESH-27 wait on it (S07 OQ 3; text below). | Accepted with amendments (recommended, 2026-10-20). ADR-003 updated; MESH-05/MESH-27 ungated. |
| 22 | S07 | Phase 1 mesh dialogs: UI-05 (generic feature dialogs) depends on SOL-01 (Phase 2). Accept hand-built Phase 1 dialogs in MESH-23, migrated later, or should lane D decouple UI-05 from SOL-01? (S07 OQ 4) | Hand-built Phase 1 dialogs in MESH-23 (2026-10-20). Moved into S07. |

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
