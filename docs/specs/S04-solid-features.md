# S04 — Solid modeling features

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 / P1 |
| **Crates** | `amg-features::solid` |
| **Lane** | A |
| **Depends on** | S01 (Feature trait, EntityRef), S02 (kernel), S03 (profiles) |
| **Parity** | Solid modeling section |

## Summary
Every Fusion "Solid" tab feature as a timeline `Feature`: inputs (profiles, faces, edges,
bodies, `Expr` values), operation (new body / join / cut / intersect / new component),
compute via `BrepKernel`, output bodies + naming history. Each feature also defines its
dialog schema (consumed by S11 to build the UI generically).

## Design
- Common `FeatureSpec` pattern: `struct Extrude { profiles: Vec<EntityRef>, start: StartSpec, extent: ExtentSpec, taper: Expr, thin: Option<ThinSpec>, op: BodyOp, participants: Vec<BodyRef> }`.
- `BodyOp` logic (join/cut/intersect into participants, new body, new component) is shared.
- Extents: distance, two sides, symmetric, to object (face/body/vertex), all, from offset.
- **Dialog schema**: each feature exposes a declarative description (fields, selection filters,
  defaults, validation) so S11 renders dialogs and on-canvas manipulators without per-feature UI code
  (custom manipulators allowed where needed).
- **Preview**: features compute in "preview mode" (lower tessellation, no history) for live
  feedback while the dialog is open; must hit < 100 ms for typical inputs.
- Hole feature includes a standards table (ISO metric, UNC/UNF, clearance/tap drill sizes);
  thread feature uses the same table (modeled thread via BREP-11, cosmetic as attribute).

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| SOL-01 | Feature framework: `BodyOp`, extents, dialog schema, preview mode | DOC-02, BREP-04 | P0 | Todo | Extrude implemented on it end-to-end |
| SOL-02 | Extrude (all extents, taper, thin) | SOL-01, SK-06, SK-19 | P0 | Todo | Fusion extrude options reproduced |
| SOL-03 | Revolve | SOL-01 | P0 | Todo | |
| SOL-04 | Primitives: box, cylinder, sphere, torus, coil, pipe | SOL-01 | P0 | Todo | |
| SOL-05 | Fillet (constant, variable, chord, setback corners) + chamfer | SOL-01, BREP-06 | P0 | Todo | edge chains via tangency; failure highlights edge. Robustness work continues in S20 (KH-03, KH-04) |
| SOL-06 | Shell, draft, scale (uniform/non-uniform), thicken | SOL-01, BREP-07 | P0 | Todo | |
| SOL-07 | Combine, split body, split face, silhouette split | SOL-01, BREP-04 | P0 | Todo | |
| SOL-08 | Pattern (rect, circular, on path) + mirror — features, faces, bodies, components | SOL-01 | P0 | Todo | identical-instance optimisation for bodies |
| SOL-09 | Move/copy, align, press-pull, offset face, replace face, delete face | SOL-01, BREP-07 | P0 | Todo | |
| SOL-10 | Sweep (path, guide rail, guide surface, twist, scale) | SOL-01, BREP-03 | P1 | Todo | |
| SOL-11 | Loft feature + dialog (profiles, rails, centerline, end conditions, point mapping) on the S20 loft builder, not raw `ThruSections` | SOL-01, KH-08 | P1 | Todo | ≥ 98% success on loft corpus (KH-05) |
| SOL-12 | Hole (all types, standards table, from sketch points) + thread (modeled/cosmetic) | SOL-02, BREP-11 | P1 | Todo | ISO metric table complete |
| SOL-13 | Rib, web, emboss (emboss/deboss text on curved faces) | SOL-02, SK-10 | P1 | Todo | |
| SOL-14 | Boundary fill, patch/ruled/extend/stitch/trim surfaces | SOL-01 | P2 | Todo | |
| SOL-15 | Rule fillet (rule-based edge selection) | SOL-05 | P2 | Todo | |
| SOL-16 | Derive / insert design as component | DOC-07 | P2 | Todo | |
| SOL-17 | Sheet metal (flange, bend, unfold, flat pattern) | SOL-01 | P3 | Todo | |

## Testing
Per-feature golden tests; scripted "Fusion tutorial" models rebuilt via Python API (S18)
compared by volume/area/bbox against STEP exports from Fusion supplied by the owner.

## Open questions
1. Owner to supply 5–10 representative Fusion designs (STEP + screenshots of timeline) as parity references.
   **Answered 2026-10-20:** yes, private, supplied later (before Phase 2 S04 implementation). They go
   under `$AMG_TESTDATA_DIR/fusion/` (S00 Open question 2) with a short note per design (purpose,
   features used); not committed.
2. (from S02, 2026-10-20) WP dependency repoints for the S02 WP split: SOL-01 also on BREP-05 (history) and BREP-03 (planar faces); SOL-02/03 → BREP-14; SOL-04 → BREP-14, and coil uses `helix` (BREP-11), so SOL-04 needs BREP-11 or coil moves to a later WP; SOL-06 → BREP-07 + BREP-16; SOL-07 → BREP-16 (split); SOL-09 → BREP-07 + BREP-16; SOL-10 → BREP-15 (was BREP-03); SOL-12 → BREP-11 (unchanged). SOL-02/03 pass profiles through `make_planar_faces` with a `geom::Tag` per edge (the sketch entity key).
3. (from S11, 2026-10-20) Adopt `amg_features::dialog` for SOL-01 (S11 Interfaces §B): `EntityFilter`, `DialogSchema`, `FieldSpec`/`FieldKind`/`Condition`/`FieldValue(s)`, `ManipAnchor`, `CommandMeta`, `DialogRegistration` via `inventory`, extendable additively. Expression fields carry `param: Option<ParamId>` in edit mode. Features registered this way get command, toolbar and timeline entries automatically, so lane A needs no `amg-ui::solid`.
4. (from S20, 2026-10-20) SOL-05 uses `robust::fillet::fillet` and `FilletDiagnosis` (failing-edge highlight, max radius) once KH-03 lands, raw BREP-06 before. SOL-11 deps should become KH-08 + KH-18 (rails/centerline) + SK-06; it renders `LoftDiagnostics`, uses `loft::prepare` for live correspondence lines and `RetryPolicy::interactive()` for preview. Its Done-when (≥ 98 % on the loft corpus) matches KH-18 Done-when 1.
5. (integration, 2026-10-20) Agreed earlier-wave repoint applied: SOL-02 now also depends on SK-19 (`amg-features::sketch` profiles).
