# Solid features: body operations, extents, tools, preview, failures — design note

| | |
|---|---|
| **Spec** | [S04 — Solid modeling features](../specs/S04-solid-features.md) |
| **Written by** | S04 refinement, 2026-11-08 |
| **Needed before** | SOL-01 (framework), SOL-02 (extents), SOL-08 (feature pattern) |
| **Status** | Written |

This note holds the reasoning behind S04's Design and Interfaces. The spec states the
rules; this note says why, and how each algorithm works on top of the S02 `BrepKernel`.
Naming rules are not repeated here: they are in
[persistent-naming.md](persistent-naming.md) §5 (accepted 2026-10-21) and S04 follows them.

## 1. Prior art

### 1.1 Fusion (the behaviour we copy)
Sources: the Autodesk Fusion help pages *Extrude reference*, *Extrude a solid body*, *Hole
reference*, *Holes and threads*, and the Fusion API (`ExtrudeFeatureInput`,
`setTwoSidesExtent`, `HoleFeatureInput`), checked 2026-11-08.

- **Extrude.**
  - *Start*: profile plane, offset (a distance from the profile plane), or from an object
    (face or plane), with an offset.
  - *Direction*: one side, two sides (an independent extent per side), symmetric (the
    distance is either half or the whole length).
  - *Extent*: distance, to object (body, face, plane or vertex, with an offset; "extend
    faces" for targets smaller than the profile), all (through all visible bodies, with a
    flip).
  - Taper angle per side. Thin extrude: thickness and side (side 1, side 2, center). Thin
    works on open curves too.
  - *Operation*: join, cut, intersect, new body, new component. "Objects to cut" lists the
    participant bodies; Fusion fills it with the bodies the tool touches and the user can
    remove some.
- **Revolve**: profile, axis (sketch line, edge, construction or origin axis), extent angle,
  full, or to object; one side, two sides, symmetric.
- **Sweep**: single path; path + guide rail (profile scaling: none, scale, stretch); path +
  guide surface. Distance along the path (fraction), taper angle, twist angle, orientation
  perpendicular or parallel.
- **Loft**: profiles (with end conditions free, direction, tangent, smooth), rails or a
  centerline, closed, tangent-edge merge, point mapping. S20 owns the builder.
- **Fillet**: edge sets with constant, chord-length or variable radius; tangent chain;
  corner type rolling ball or setback. Rule fillet and full-round fillet are separate
  commands. G2 (curvature-continuous) fillets are an option in Fusion (QUESTIONS #55).
- **Chamfer**: equal distance, two distances, distance and angle; corner type.
- **Shell**: faces to remove (none = hollow body), inside and outside thickness, direction
  inside/outside/both, tangent chain.
- **Draft**: pull direction from a plane, faces, angle, flip, tangent chain; one side, two
  sides or symmetric (two-sided drafts split the faces at the plane).
- **Hole**: placement (single on a face with two edge references, from sketch points,
  multiple); hole type simple/counterbore/countersink; tap type simple, clearance, tapped,
  taper tapped; drill point flat or angle (default 118°); extent distance, to object, all.
  A tapped hole carries a thread designation (size, pitch, class), cosmetic by default,
  "modeled" on request.
- **Pattern**: rectangular (two directions, extent or spacing, symmetric), circular (full,
  angle extent or spacing, symmetric), on path; objects are bodies, faces, features or
  components; suppressed instances; compute type optimized/identical/adjust.
- **Mirror**: same objects about a plane; the body variant can create or join.
- **Combine**: target, tool bodies, join/cut/intersect, keep tools, new component.
- **Press pull** is a context tool: it starts Offset Face on a face, Fillet on an edge and
  Extrude on a profile. It has no timeline feature of its own.

### 1.2 OCCT behaviour and failure modes we design around
Sources: OCCT 8.0.0 reference manual (`BRepPrimAPI_MakePrism`, `BRepPrimAPI_MakeRevol`,
`BRepOffsetAPI_MakeThickSolid`, `BRepOffsetAPI_MakeOffset`, `BRepOffsetAPI_MakePipeShell`,
`BRepOffsetAPI_DraftAngle`, `BRepFilletAPI_MakeFillet`/`MakeChamfer`, `BRepAlgoAPI_*`,
`BRepAlgoAPI_Splitter`, `Contap_Contour`), the OCCT tracker (e.g. #25916 "Incorrect set of
faces to remove, the remaining shell is not connected") and forum threads on thick solids,
plus [occt-integration.md](occt-integration.md) and [kernel-robustness.md](kernel-robustness.md).

| Op | Known failure | S04 response |
|---|---|---|
| Prism | Infinite prisms in booleans are slow or fail | Never pass infinite solids to booleans. "All" and "to object" use a finite length from the bounding boxes (§3) |
| Prism + taper | Draft on a non-planar or self-touching lateral set fails | Taper is applied per side by S02 (`PrismOpts::taper`); a failure names the profile, hint "reduce the taper" |
| Revol | An edge on the axis gives a degenerate face; a profile crossing the axis gives an invalid solid | Validate before calling: every profile lies on one side of the axis (`AxisCrossesProfile`); edges on the axis are allowed |
| MakeThickSolid | Removed faces leaving a disconnected shell; thickness above the smallest concave radius; open shells | Pre-check that the kept faces are edge-connected; S02 returns `LocalOpFailed { offending }` for the rest; the dialog highlights them |
| MakeFillet | Radius too large for adjacent faces; vertex blends of 3+ edges; partial results | Raw BREP-06 first (SOL-05), then S20's chain and `FilletDiagnosis` (SOL-20). Partial results are never used |
| DraftAngle | Faces whose neighbours can't be re-intersected; drafting a face that touches the neutral plane on a curved edge | Fail with offending faces; no automatic retry in S04 |
| Booleans | Coplanar and tangent cases, near-coincident tolerances | Through `ctx.kernel()`, which is S20's `HardenedKernel` (fuzzy ladder, glue). S04 always runs `unify_same_domain` afterwards (§2.4) |
| MakePipeShell | Profile not at the path start; self-intersecting sweeps; twist with a guide | Move the profile frame to the path start (S04 does the transform); a self-intersecting result fails validation and is reported |
| Splitter | Tool faces that don't cross the object leave it whole | Detect "no split" (piece count 1) and fail with a hint ("extend the tool") |

### 1.3 Other systems
- **Onshape** (FeatureScript `opExtrude`, `opBoolean`, `opPattern`): booleans are separate
  ops after the tool is built; pattern of features re-runs the feature's tool with
  transformed inputs. We take the tool/boolean split (§4).
- **FreeCAD PartDesign** (Pad, Pocket, Revolution, Pattern): one solid per body ("single
  solid rule"). We don't take that: Fusion bodies are free, and a cut may split a body.
- **CadQuery / build123d**: `extrude(until=...)` up to a face is implemented as a long
  prism split by the target face. Our "to object" extent does the same (§3.2).

## 2. Body operations (`BodyOp`)

### 2.1 Tool, then boolean
Every additive or subtractive feature builds a **tool** (one or more solids, with names from
the feature's own rules), then applies its `BodyOp`. The tool step and the boolean step are
separate functions, because pattern and mirror re-use the tool step (§4).

### 2.2 Participants
- `participants: Vec<BodyId>` is stored in the feature. The dialog fills it from the preview
  (every solid body of the context component whose bounding box overlaps the tool and that
  the tool actually intersects or touches), like Fusion's "Objects to cut". The user can
  remove bodies.
- `inputs()` lists `Dependency::Body` for each participant, so the ordering rule and the
  cache key see them.
- A participant that no longer exists at recompute is looked up through
  `Evaluated::successors` (a body consumed by a combine is followed to the body it went
  into). If it has no successor, it is dropped with a warning.
- P0 rule: participants are bodies of the feature's context component. Cross-component cuts
  wait for assemblies (SOL-24).

### 2.3 Semantics per op
- **NewBody**: each connected tool solid becomes a body `BodyId { self, k }`, `k` in tool
  order (profile order, then piece order).
- **Join**: the tool is fused with every participant it touches.
  - The result keeps the `BodyId` of the first touched participant in list order. The other
    touched participants are `Consumed { into }`.
  - If it touches none, the tool becomes a new body with a warning
    ("nothing to join; created a new body"). This is assumed Fusion behaviour; the owner's
    reference designs (S04 Open question 1) will confirm it.
- **Cut**: every participant is cut by the tool separately.
  - A participant the tool doesn't touch is unchanged (no warning).
  - A cut that splits a body into several solids keeps piece 0 (ordered by volume desc, then
    centroid, the S02 split order) under the original `BodyId`. The others are new bodies
    `BodyId { self, k }` with `Created { derived_from }`. This uses S02's
    `OpResult::per_solid` (naming note §12 Q3).
  - A cut that removes a whole body gives `Deleted` and a warning.
  - If no participant is touched, the feature fails with `NothingToCut`.
- **Intersect**: every participant is replaced by its common part with the tool.
  - An empty common part deletes that participant, with a warning.
  - If every participant becomes empty, the feature fails with `NothingToIntersect`.
- **NewComponent**: this is not a feature-data value. The dialog creates the component and
  the feature in one transaction (`AddComponent`, then `AddFeature` with that component as
  the context and `op = NewBody`). This needs S06 ASM-01 and one additive S11 hook (SOL-24).
  Reason: components are authored state, and a feature may only produce evaluated state
  (S01 §1).

### 2.4 Simplification after every boolean
Every join, cut, intersect and combine result goes through
`unify_same_domain { faces: true, edges: true, concat_bsplines: false }` with the default
tolerances, and the unify history is composed into the boolean history
(`ShapeHistory::compose`) before naming. This is the naming-note contract (§5.1, QUESTIONS
#48 answered "always"). New bodies made from a single prism are already unified by
`PrismOpts::merge_coplanar`.

### 2.5 Mixed body types
Participants must be solids (`BodyGeom::Solid`). A mesh participant fails with
`WrongBodyKind` and a hint: convert it first (S07) or use a mesh boolean. The architecture's
"lowest common representation" rule (mesh) is a mesh-workspace feature, not an implicit S04
fallback.

## 3. Extents

### 3.1 Distance, symmetric, two sides
- One side: `PrismExtent::Vector(d · n)`, with `n` the profile normal (flipped if `flip`).
- Symmetric: the profile face is moved by `−d/2 · n` (or `−d · n` for the whole-length
  option) and extruded by `d` (or `2d`). Taper is applied symmetrically: two prisms (one per
  side, each tapered) fused, because a tapered prism tapers from its start face only.
- Two sides: two prisms (side 1 along `n`, side 2 along `−n`), each with its own extent and
  taper, fused before the body op. The naming rule's `side` index (0/1) comes from here.
- Start offset: the profile face is translated by `offset · n` first. Start "from object"
  projects the profile along `n` onto the start face: planar start faces only in SOL-02
  (translate to the plane); a curved start face is an error until a later need appears.

### 3.2 To object
1. Compute a finite length `L` that surely passes the target: the distance along `n` from
   the profile plane to the far side of the target's bounding box, plus 1 mm, plus `|offset|`.
2. Build the prism of length `L`.
3. Target cases:
   - **Plane, or a planar face parallel to the profile**: the exact distance is computed
     and a plain prism is built (no split). This is the common case and the cheapest.
   - **Face** (non-parallel or curved): split the long prism with the face (`split`).
     With `extend`, the face is first replaced by its untrimmed surface
     (`make_face(surface, UvRect)` over a box larger than the prism). B-spline faces can't be
     extended in SOL-02 (`ExtendUnsupported`).
   - **Body**: split with the body's faces.
   - **Vertex**: the distance to the plane through the vertex parallel to the profile.
4. Keep the piece that contains the profile face (classify a point just above the profile
   centroid). If the split leaves one piece, the target doesn't cover the profile: fail with
   `TargetMissed` and the hint "turn on Extend faces".
5. Offset: a non-zero offset on a non-planar target moves the target face by `offset` along
   its normal (`offset` op on the target face as a sheet) before step 3.

### 3.3 All
`L` = the extent of the bounding boxes of every solid body of the context component, along
`n`, plus 1 mm. For a cut, the tool goes through everything in the chosen direction (or both,
for two-sided). The tool's end cap never survives a cut-through, so its name doesn't matter.

### 3.4 Profiles
- An extrude takes `EntityRef::Profile` regions and planar `EntityRef::Topo` faces.
- Profiles must have parallel planes (`NonParallelProfiles` otherwise). Each is extruded from
  its own plane.
- Selected regions of one sketch that share edges (an outer ring and the island inside it, or
  two halves of a rectangle split by a line) are fused into one tool solid before the body
  op, so the shared lateral faces disappear. The fuse is unified.
- Profile → face: `amg_sketch::Curve2` converts to `amg_brep::geom::Curve2d` with a total
  `match` (S02 §4). Each edge's tag is `Tag(u64::from(edge.source.0))`, the sketch entity id,
  and `NamingBuilder::tag` maps it to `NameSource::SketchCurve`.

## 4. Tools for pattern and mirror (`SolidTool`)

A *feature pattern* repeats what a feature did, not its result.
- Every additive/subtractive feature implements `SolidTool::tool(&self, ctx) -> ToolBody`.
  `ToolBody` is the tool shape, its `BodyNaming` (built with the seed's own rules, feature =
  the seed's id), the `BodyOp` and participants.
- A `ToolRegistration { kind, tool }` is submitted with `inventory` next to
  `register_feature!`, so `solid.pattern` can find the tool function of any seed kind from
  the seed's `FeatureBox` (looked up in `ctx.design()`'s timeline).
- The pattern feature calls the seed's tool in **its own** context (the state just before the
  pattern). Profiles and sketches are unchanged there. Body references in the seed resolve
  through lineage. For the "identical" compute type, the tool is built once and moved per
  instance with `Shape::transformed` (shared TShapes, cheap); for "adjust" (P1, SOL-23) the
  seed's inputs are transformed and the tool is rebuilt per instance.
- Names: instance `k`'s tool naming is `BodyNaming::instanced(self, k, &shape)` of the seed's
  tool naming. All instance tools go into **one** boolean (N-ary fuse/cut) with the pattern's
  `BodyOp`, so a 100-hole pattern is one boolean, not 100.
- Mirror of features is the same with one instance (`k = 1`) and a mirror transform. Mirror
  needs S02's `mirrored` (request R-S04-6): `Shape::transformed` takes only proper rigid
  transforms, and a reflection must also flip face orientations.
- Body pattern and body mirror copy whole bodies: new `BodyId { self, k }`, naming
  `instanced(self, k)`, no boolean (mirror may join, as Fusion).
- Face pattern (SOL-23): the selected faces must bound a boss or pocket whose boundary
  edges lie on one planar support face. The tool is the faces sewn with cap faces built from
  their boundary loops on the support plane; a boss is fused, a pocket cut.
- Instance numbering follows the naming note: rectangular `k = i + (j << 16)`, circular and
  path `k` = position along the pattern; suppressed instances keep their number (QUESTIONS #43).

## 5. Preview mode
- `ComputeMode::Preview` (DOC-12) runs the same `compute` code. Differences:
  - S20's kernel policy is `RetryPolicy::interactive()` (the engine configures
    `ctx.kernel()`; features that call `robust::*` directly pass `interactive()` themselves
    when `ctx.mode()` is `Preview`);
  - `KernelConfig::validate` is `Off` (no BRepCheck on preview results);
  - naming is still built: it costs far less than the boolean (DOC-06 budget), and a preview
    with wrong names would hide naming bugs until OK is pressed.
- The < 100 ms preview budget is measured for typical inputs: an extrude of a 20-edge profile
  cutting a 200-face body, a 10-edge constant fillet on a 200-face body, a shell of a
  100-face body (SOL-01/05/06 Done-when). Tessellation for display is S11's, at
  `TessQuality::preview()`, and is not in the budget.
- A preview that fails shows the feature error in the dialog banner (S11 UI-05), with the
  offending entities highlighted (§6). It never enters the cache or undo stack (DOC-12).

## 6. Errors and failure UX
- One `SolidError` enum (`thiserror`) covers every S04 failure. It converts into
  `FeatureError::InvalidValue { field, msg }` for input problems and
  `FeatureError::Kernel(msg)` for kernel failures.
- Offending sub-shapes from `KernelError::{LocalOpFailed, OpFailed, InvalidInput}` are turned
  into `EntityRef`s with `topo_ref_at` (DOC-06) and put into `Diagnostic::entities`, so the
  viewport highlights the failing edges or faces (SOL-05 Done-when "failure highlights
  edge").
- A dropped set member (fillet edge, shell face) is a warning, not an error
  (`SetPolicy::DropMissing`, QUESTIONS #44). A single-valued reference (axis, extent target,
  mirror plane) that doesn't resolve is an error (`AllOrError`).
- Hints come from `amg_core::Diagnostic::hint` (S02's per-variant hints plus S04's own:
  "reduce the radius", "turn on Extend faces", "the profile crosses the axis").

## 7. Standards tables (holes and threads)
- Data lives in `crates/amg-features/data/threads/*.toml`, embedded with `include_str!`,
  parsed once (`OnceLock`), and is checked by a unit test against the ISO 68-1 basic-profile
  formulas:
  - fundamental triangle height `H = (√3 / 2) · P`;
  - internal minor diameter `D1 = D − 1.082532 · P` (= `D − 5/4 · H`);
  - external minor diameter `d3 = d − 1.226869 · P` (= `d − 17/12 · H`);
  - pitch diameter `D2 = D − 0.649519 · P` (= `D − 3/4 · H`).
- **ISO metric coarse and fine** (ISO 261 size/pitch selection, M1.6–M64): nominal
  diameter, pitch, D1, D2, d3, tap drill (D − P, rounded to the 0.05 mm drill series; e.g.
  M3 2.5, M4 3.3, M5 4.2, M6 5.0, M8 6.8, M10 8.5), and clearance holes after ISO 273
  fine / medium / coarse (e.g. M3 3.2/3.4/3.6, M4 4.3/4.5/4.8, M5 5.3/5.5/5.8,
  M6 6.4/6.6/7.0, M8 8.4/9.0/10.0, M10 10.5/11.0/12.0). Counterbore and countersink defaults
  for socket-head and countersunk screws (ISO 4762, ISO 10642 head sizes) give the dialog's
  defaults only; the user can always type values.
- The values are facts from published standards and drill charts, not copied text. Sources
  are listed in the file header. The SOL-12 test recomputes every derived column from the
  formulas, so a typo fails the test.
- **UNC/UNF** come with the Thread feature (SOL-25); scope depends on QUESTIONS #54.
- Print allowances for FFF parts are QUESTIONS #53.

## 8. Recompute budget
The vision budget is < 500 ms for a typical 50-feature part after an early sketch edit. That
is ≤ 10 ms of kernel work per feature on average (S02 performance targets). SOL-29 builds a
scripted 50-feature part (sketch → extrudes, cuts, fillets, shells, a pattern, holes) and
measures the full recompute after a dimension edit on the first sketch (criterion bench
`budget/sol-recompute-50`). If it misses, the bench output names the slowest features, and
the fix goes to the owning WP.

## 9. What S04 asks of other specs
Recorded in the owning specs' **Interface changes** (2026-11-08 entries):
- **S02**: R-S04-1 … R-S04-9 (S04 Interfaces, "Kernel calls"), all additive.
- **S01**: confirms the DOC-06 additions S04 relies on (`RecomputeCtx::kernel`,
  `naming_builder`, `resolve_all`, `resolve_profile`, `region_source`,
  `BodyChangeKind::Consumed`, `Created { derived_from }`, `Evaluated::successors`,
  `topo_ref_at`). No new request.
- **S11**: adopts §B as is, plus one additive field, `DialogRegistration::new_component`
  (used from SOL-24; `None` until then).
