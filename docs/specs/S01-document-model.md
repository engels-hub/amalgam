# S01 — Document model, timeline & persistent naming

| | |
|---|---|
| **Status** | Draft |
| **Priority** | P0 |
| **Crates** | `amg-doc` |
| **Depends on** | S00 (FND-03/04); data types from S02/S03/S07 (can start with stubs) |
| **Lane** | E |
| **Parity** | Timeline, parameters, components/bodies browser, undo, save/open |

## Summary
The heart of the app: the design tree (components, bodies, sketches, construction geometry),
the parametric timeline with incremental recompute, parameters with unit-aware expressions,
persistent references to topology (the "topological naming problem"), commands + undo/redo,
and the `.amg` file format.

## Scope
**In:** data model, `Feature` trait, recompute engine, `EntityRef` resolution framework,
parameters/expressions, commands/undo, direct (no-history) mode, file format + autosave,
change notifications for UI/render.
**Out:** concrete features (S04 and domain specs), UI (S11).

## Design

### Data model
```
Document { design: Design, history: UndoStack, studies: Vec<StudyId>, meta }
Design   { params: ParamTable, timeline: Timeline, components: Arena<Component>, root: ComponentId, mode: History|Direct }
Component{ name, occurrences: Vec<Occurrence>, origin, construction: Vec<ConstructionId>,
           sketches: Vec<SketchId>, bodies: Vec<BodyId>, joints: Vec<JointId> }
Body     { name, kind: BodyKind /* Solid | Mesh | Implicit | Form */, material, appearance, visible, … }
```
- All entities are stored in arenas keyed by generational ids, plus a stable `Uuid` for files.
- Structural sharing (`Arc`, `im` crate or `rpds`) so recompute works on cheap snapshots.

### Timeline & recompute
- `Timeline { nodes: Vec<FeatureNode>, marker: usize }` — rollback marker like Fusion.
- `FeatureNode { id, feature: Box<dyn Feature>, suppressed, group, health: Ok|Warning|Error(msg), cache }`.
- `trait Feature { fn inputs(&self) -> Vec<Dependency>; fn compute(&self, ctx: &mut RecomputeCtx) -> Result<FeatureOutput>; fn kind(&self) -> &'static str; fn serialize(..) }`
  registered via `inventory` so domain crates add feature types without touching `amg-doc`.
- Dependencies: parameters, other features' outputs (bodies, sketches, construction), `EntityRef`s.
- Dirty propagation through the dependency DAG; clean features reuse cached outputs keyed by
  `hash(feature params + input hashes)`. Recompute runs as a job on a snapshot (S00 jobs).
- Errors don't abort: failed feature is flagged red, downstream features try to resolve what
  they can (Fusion behaviour).

### Persistent naming (hardest problem — budget real research time)
`EntityRef` identifies a face/edge/vertex of a body across recomputes:
```
EntityRef { body: BodyRef, kind: Face|Edge|Vertex, name: TopoName, fingerprint: GeomFingerprint }
TopoName  = generator feature id + role path, e.g.
            Extrude#7 / LateralFace / from SketchCurve(S3, Line#12)
            Extrude#7 / EndCap / 0
            Fillet#9  / FilletFace / from Edge(<TopoName of filleted edge>)
            Boolean#4 / Modified(<TopoName>) / split#1
GeomFingerprint = surface type, area, centroid, normal/axis, bbox — for fallback matching.
```
- Every feature that creates topology must report names for what it generated/modified/deleted
  (OCCT `BRepTools_History` gives generated/modified/deleted maps; S02 exposes them).
- Resolution: exact name match → name match with split disambiguation (by fingerprint proximity)
  → fingerprint-only match (warning) → fail (feature error, user re-picks).
- Mesh bodies use face-group ids + fingerprints; implicit bodies have no sub-entity refs.
- Prior art to study during refinement: FreeCAD 1.0 TNP fix (realthunder's element mapping),
  Onshape/Parasolid attribute approach, CadQuery selectors, the paper
  *"Persistent naming for parametric models"* (Kripac 1997), Marcheix & Pierra survey.

### Parameters
- `ParamTable` with user params and model params (auto-created per dimension, e.g. `d12`).
- Expressions: small parser (`+ - * / ^`, functions sin/cos/sqrt/min/max/floor/…, units in
  literals, references to other params). Unit-checked. Cycles rejected.
- Feature fields that accept values are `Expr` not `f64`.

### Commands & undo
- `enum Command { AddFeature, EditFeature, DeleteFeature, MoveMarker, SetParam, Rename, SetVisibility, TransformOccurrence, … }`
  — the **only** mutation path; each has an inverse or stores a snapshot. Python API mirrors it.
- Undo granularity: one user action = one entry; tool previews don't create entries.

### Direct mode
Features computed once and baked into bodies (`BakedBody`); timeline hidden. Switching
History→Direct bakes everything; Direct→History starts a fresh timeline from the baked state.

### File format
Per architecture §File format. Schema versioned (`format_version`), with migrations.
Autosave every N minutes + on recompute success into `%LOCALAPPDATA%/Amalgam/recovery`.

### Change notifications
`DocEvent` stream (body changed, visibility, selection-relevant changes) consumed by render/UI.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| DOC-01 | Core data model types + arenas + snapshots | FND-03 | P0 | Todo | create design, components, bodies; snapshot is O(1) |
| DOC-02 | `Feature` trait, registry, timeline, rollback marker, suppression | DOC-01 | P0 | Todo | dummy features recompute in order |
| DOC-03 | Incremental recompute engine (DAG, dirty prop, caching, job-based, cancel) | DOC-02, FND-04 | P0 | Todo | editing feature k recomputes only dependents; test with counters |
| DOC-04 | Parameters & expression engine with units | FND-03 | P0 | Todo | parse/eval/unit-check suite; cycle detection |
| DOC-05 | Commands + undo/redo | DOC-02 | P0 | Todo | random command sequence → undo all → equals initial (property test) |
| DOC-11 | Persistent naming design note `docs/notes/persistent-naming.md` (prior-art study, chosen scheme, per-feature naming rules, golden corpus list) | DOC-01 | P0 | Todo | owner has reviewed the note |
| DOC-06 | Persistent naming framework: `TopoName`, history ingestion API, resolver | DOC-02, DOC-11, BREP-05 | P0 | Todo | golden tests: edit sketch dims of 20 canonical models, fillets stay on intended edges |
| DOC-07 | `.amg` file format, versioning, caches, autosave/recovery | DOC-05 | P0 | Todo | save→load round-trip equality; corrupted file → clear error |
| DOC-08 | Direct mode + bake | DOC-03 | P1 | Todo | toggle both ways keeps geometry |
| DOC-09 | Change events for UI/render | DOC-03 | P0 | Todo | render receives minimal diffs |
| DOC-10 | Configurations (variant tables over params/suppression) | DOC-04 | P2 | Todo | switch configuration recomputes |

## Testing
Property tests for commands/undo; golden TNP corpus (refinement deliverable: list of ~20
models + edit scripts that break naive naming); recompute benchmarks.

## Performance targets
Snapshot < 1 ms; recompute overhead (excluding geometry) < 5 ms for 200 features.

## Open questions
1. `im`/`rpds` persistent collections vs `Arc` copy-on-write per entity — benchmark in DOC-01.
2. How much of TNP can be solved generically vs per-feature? Answered by DOC-11 before DOC-06 starts.
