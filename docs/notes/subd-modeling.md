# SubD modeling: cage, evaluation, Form timeline model and SubD → NURBS

Design note for S09 (`docs/specs/S09-form-subd.md`). Written by the S09 refinement
(2026-11-08). Needed before FRM-01 (§1–§3), FRM-15 (§4), FRM-16 (§5), FRM-03 (§6) and
FRM-06 (§7).

## 0. Prior art

| Source | What it does | What we take | What we reject |
|---|---|---|---|
| Autodesk Fusion, Form contextual environment (T-splines; Fusion help "Modify T-Spline bodies with Edit Form", "T-Spline primitives", "T-Spline utilities", "Pull vertices to a target body") | "Create Form" opens a FORM contextual tab. Create: box, plane, cylinder, sphere, torus, quadball, pipe, face, extrude, revolve, sweep, loft. Modify: Edit Form (move/rotate/scale faces, edges, vertices with a combined manipulator, soft modification, Alt-drag adds geometry), insert edge, insert point, subdivide, bridge, fill hole, erase and fill, weld vertices, unweld edges, crease, uncrease, bevel edge, slide edge, pull, flatten, match, interpolate, thicken, freeze. Symmetry: mirror/circular internal and duplicate, clear, isolate. Utilities: display mode (box, control frame, smooth), repair body, make uniform, convert. Finish Form leaves one "Form" node in the timeline; the edits inside are not separate features. The finished body behaves as a B-rep body for solid features. | The tool set, the contextual environment, one timeline node per form, the three display modes, per-edit undo inside the form, and Finish Form handing a B-rep body to solid features (§5.3). | T-splines themselves (ADR-009: Catmull–Clark with creases instead). T-junctions (T-points) do not exist in a SubD cage; foreign T-spline data is converted on import (FRM-10). "Make uniform" has no meaning for SubD (Catmull–Clark is uniform by construction). |
| OpenSubdiv 3.6 / 3.7 (Pixar, "Modified Apache 2.0" licence; `Sdc`, `Vtr`, `Far` layers) | Reference semantics for Catmull–Clark: smooth, crease, corner and dart vertex rules; semi-sharp creases where sharpness drops by 1 per level; Chaikin crease method; boundary interpolation `EDGE_ONLY` / `EDGE_AND_CORNER`; triangle subdivision `CATMARK` / `SMOOTH`; limit masks; feature-adaptive refinement with regular B-spline patches and Gregory or B-spline "end caps" at an isolation level; stencil tables. | The rules and options exactly (FRM-01 tests against OpenSubdiv output), the stencil-table idea for interactive drag (§4.3), feature-adaptive patch tables (§4.2). | A runtime dependency (§2). |
| `opensubdiv-petite` (Rust, Apache-2.0 wrapper; bundles OpenSubdiv 3.7 in `-sys`) | Thin Rust bindings over `Far` topology refiner, primvar refiner and stencil tables. The 0.3.1 docs.rs build fails. | Possibly the oracle shim (§2). | Depending on it at runtime. |
| Rhino 7/8 SubD and `ToNURBS` (McNeel; Rhino C++ `ON_SubDToBrepParameters`, RhinoCommon `SubDToBrepOptions`) | One bicubic NURBS patch per SubD quad, n patches per n-gon. Extraordinary-vertex options: none (smallest deviation, not G1), local G1, local G1x (G1 near the vertex, one double knot near the vertex), G2 (larger deviation, bulges). "Packed" merges grids of regular quads into one surface; McNeel says packing and G1 smoothing are post-processes after the face-by-face conversion. | The output shape: unpacked by default with G1 at extraordinary vertices via a double knot (§7.3); packed as an option (FRM-11); deviation reported. | G2 at extraordinary vertices (Rhino's own help says it bulges). |
| Stam, "Exact Evaluation of Catmull–Clark Subdivision Surfaces at Arbitrary Parameter Values" (SIGGRAPH 1998) | Closed-form eval near an extraordinary vertex by eigen-decomposition. | Only as a cross-check in tests. | As the main evaluator: it does not cover creases and boundaries without extra cases. |
| Nießner, Loop, Meyer, DeRose, "Feature Adaptive GPU Rendering of Catmull–Clark Subdivision Surfaces" (ToG 2012); Loop, Schaefer, Ni, Castaño, "Approximating Subdivision Surfaces with Gregory Patches" (2009) | Refine only around extraordinary vertices and creases; regular regions are bicubic B-spline patches; the remaining ring at the isolation level is a Gregory patch. | The evaluation structure (§4.2). | — |
| Peters, "Patching Catmull–Clark Meshes" (SIGGRAPH 2000) | Converts a Catmull–Clark cage (after one refinement) to bicubic NURBS: regular faces exactly, faces at an extraordinary vertex as 2×2 bicubic pieces with G1 constraints across the vertex and its edges. | The G1 construction at extraordinary vertices (§7.3). One surface per face with a double knot at ½ in u and v holds the 2×2 pieces. | — |
| Halstead, Kass, DeRose, "Efficient, Fair Interpolation using Catmull–Clark Surfaces" (SIGGRAPH 1993) | Solve for cage vertices whose limit points interpolate given points. | Interpolate and pull (FRM-07) as a sparse least-squares solve on limit masks (§8). | The fairing term in v1. |
| Instant Meshes (Jakob et al. 2015, BSD-3-Clause), QuadriFlow (Huang et al. 2018, MIT; its Eigen sparse Cholesky is LGPL unless built with `BUILD_FREE_LICENSE`), `quadrs` (Rust, MIT, experimental, Instant-Meshes based) | Field-aligned quad remeshing of triangle meshes. | Candidates for FRM-08 (§9). | Deciding now; FRM-08 is P3. |

## 1. Cage data structure (FRM-01)

- `SubdCage` is a polygon mesh: faces have 3 or more vertices, quads are the common case.
  It is a 2-manifold with boundary: every edge has one or two faces, and every vertex has one
  fan of faces. Non-manifold input is rejected (`SubdError::NonManifold`), as in OpenSubdiv's
  manifold mode.
- **Half-edge storage** of its own. `amg-mesh::HalfEdgeMesh` is triangle-only (directed edges,
  `3f + k`), so it can't hold n-gons. The cage uses explicit half-edges: `next`, `twin`,
  `vertex`, `face`, and per-vertex / per-face one half-edge. Boundary half-edges are explicit
  (face = `None`), which makes boundary loops and "fill hole" cheap.
- **Stable ids.** `VertId`, `EdgeId`, `FaceId` are `u32` slots with tombstones. New elements
  always take a fresh id from a monotonic counter stored in the cage; ids are never reused
  within a cage. Ids survive save/load (they are part of the encoding). Selections, symmetry
  pairs, freeze flags and the B-rep face tags (§7.5) all key on them.
- **Tags** (OpenSubdiv semantics):
  - edge sharpness `f32` in `[0, 10]`; `10` = infinitely sharp (`SHARPNESS_INFINITE`), `0` =
    smooth. f32 matches OpenSubdiv so goldens compare exactly; it is a tag, not geometry.
  - vertex (corner) sharpness `f32` with the same range.
  - per-vertex `frozen: bool` (Fusion "Freeze"; edits skip frozen vertices).
- **Options** (`SubdOptions`), fixed per cage:
  - `boundary: BoundaryInterp` — `EdgeOnly` or `EdgeAndCorner` (default; a valence-2 boundary
    vertex stays a corner, which is what a plane form's corners look like in Fusion).
  - `crease: CreaseMethod` — `Uniform` or `Chaikin` (default `Uniform`, OpenSubdiv's default).
  - `triangles: TriangleRule` — `Catmark` (default) or `Smooth`.
- **Positions** are `Point3` (f64, mm). The cage is plain data and `Send + Sync`.
- **Encoding.** Binary `AMGS` v1: magic, version, options, counters, positions, faces (vertex
  lists), tags, symmetry, then a blake3 checksum. Little-endian, f64 bit-exact. The same
  bytes feed the cache hash (`content_hash`).
- **Validation** runs on every topology op result and on decode: manifold, consistent
  orientation, no zero-length edges, no face with repeated vertices. Ops never leave an
  invalid cage; they return `SubdError` and leave the input untouched (the cage is
  immutable; ops return a new cage).

## 2. Own Rust evaluator, OpenSubdiv as oracle

Decided (owner-confirmed 2026-11-09, QUESTIONS #100): `amg-subd` implements refinement and
evaluation in Rust. OpenSubdiv is a test oracle only, behind a `osd-oracle` cargo feature
that is off by default and not built in CI.

Why:
- The cage, its topology ops and its ids must be ours anyway (OpenSubdiv refines and
  evaluates; it does not edit). What remains is about 2–3 k lines: refinement rules
  (published masks), limit masks, patch extraction, stencils.
- No new C++ build. OpenSubdiv has a vcpkg port, but every C++ dependency so far needed an
  overlay or a TBB/feature fight (ADR-003, ADR-002 notes).
- ADR-009 asks for OpenSubdiv *semantics*, not the library. ADR-007 (libslvs oracle) and S08
  (fidget oracle) set the pattern.

How the oracle works: a small tool `crates/amg-subd/tools/osd-fixtures` (only with
`osd-oracle`) refines and evaluates the fixture cages with OpenSubdiv and writes JSON to
`testdata/subd/osd/`. The fixtures are committed (small, a few MB), so normal test runs
compare against them without OpenSubdiv. The fixture set: every primitive, the "crease zoo"
(one cage per vertex rule: smooth, dart, crease, corner, each semi-sharp at 0.5/1.5/2.5,
Chaikin on and off), boundaries in both modes, triangles in both modes, pentagons, valence
3/5/6/8 vertices.

## 3. Refinement and limit masks (FRM-01)

- **Uniform refinement** of level `k`: face points, edge points, vertex points with the
  smooth / crease / corner / dart rules; semi-sharp edges blend smooth and sharp rules by the
  fractional sharpness and pass `s − 1` (Uniform) or the Chaikin average (Chaikin) to the
  child edges. After level 1 every face is a quad.
- Each refined face remembers its **root cage face** (`FaceId`) and its (u, v) square in that
  face's parameter domain. This is what picking, display highlights and B-rep face tags use.
- **Limit masks** give each refined vertex its limit position and two limit tangents
  (OpenSubdiv `LimitStencil` semantics), so `to_mesh(level)` returns points on the limit
  surface with exact limit normals.
- `to_mesh(level)` → `amg_mesh::IndexedMesh` (two triangles per quad, consistent winding,
  `FaceGroups` = one group per root cage face). The mesh is closed and manifold when the cage
  is closed. This is the "subdivide" conversion in architecture §Geometry representations.
- `to_mesh_tol(chord_tol)` picks the smallest level whose max distance between the level-`k`
  limit mesh and the limit surface is ≤ `chord_tol` (estimated per root face from the
  second differences of the refined points; capped at level 6).

## 4. Exact evaluation and stencils (FRM-15)

### 4.1 What needs exact evaluation
Pull and interpolate (closest point, limit positions), conversion accuracy checks, Thicken
(limit normals), and "Match" boundary samples. Display does not: it uses refined limit points.

### 4.2 Patch table
- Feature-adaptive refinement up to an isolation level `L` (default 6, max 10).
- A quad whose 1-ring is regular (all four vertices valence 4 interior, or a regular
  boundary/crease configuration with infinitely sharp edges) becomes a **bicubic uniform
  B-spline patch** (16 control points; boundary and infinitely sharp crease cases use the
  OpenSubdiv phantom-point rule). Semi-sharp creases are refined `ceil(s)` levels first.
- Faces next to an extraordinary vertex are refined until level `L`; the last ring uses a
  **Gregory basis end cap** (OpenSubdiv `ENDCAP_GREGORY_BASIS`). Error at `L = 6` is far below
  display and conversion tolerances (OpenSubdiv's own default).
- `eval(face, u, v)` returns position and first and second derivatives; `closest_point(p)`
  uses a BVH over patch bounding boxes (control-hull boxes are conservative) plus Newton on
  the patch.

### 4.3 Stencil tables (interactive drag)
- For a fixed topology and tag set, every display point (refined limit point at level `k`)
  is a fixed linear combination of cage vertices. `StencilTable` stores these weights
  (sparse rows, f64), built once per topology.
- A drag changes positions only, so the new display mesh is one sparse mat-vec. Weights are
  rebuilt only after a topology op or a crease/option change.
- Budget: rows × average row size. For a 2 000-face cage at level 3 (≈ 128 k display points,
  ≈ 25 weights per row) that is ≈ 3.2 M multiply-adds, under 2 ms on 8 cores. Partial update:
  only rows touching moved vertices are recomputed (an index of rows per cage vertex), which
  makes small drags much cheaper.
- Normals come from the tangent stencils the same way.

## 5. Form bodies in the timeline (FRM-16, FRM-17)

### 5.1 Evaluated payload
- `BodyGeom::Form(Arc<SubdMesh>)` (the `amg-doc` stub name). `SubdMesh` wraps an
  `Arc<SubdCage>` plus lazily built caches (`OnceLock`): the stencil table per display
  level and the patch table. It is immutable and `Send + Sync`; caches are pure functions of
  the cage, so sharing them across threads is safe and deterministic.
- The body codec stores a Form body as the cage's `AMGS` bytes; caches are rebuilt on load.

### 5.2 Authored feature: one node per form
- `FormFeature` (`form.create`) holds the whole authored cage (`Arc<SubdCage>`) plus a name
  and the target component. This is Fusion's model: one timeline node; edits inside it are
  not parametric.
- The cage is stored in `design.json` as base64 of its `AMGS` encoding (`#[serde(with =
  "amg_subd::serde_b64")]`): bit-exact f64 round trip, about 4× smaller than JSON numbers, and
  the cache key is the blake3 of those bytes. A 5 000-vertex cage is ≈ 200 KB of base64.
- An edit inside the form (one drag, one insert edge) is one `EditFeature` that swaps the
  cage `Arc`. Undo/redo is per edit for free (ADR-014 snapshots), as in Fusion.
- Recompute is trivial: `compute()` wraps the cage in a `SubdMesh` and emits the body. Nothing
  upstream can change it (the form reads no inputs), so it is always a cache hit after the
  first time.
- Direct mode (S01): the form is baked like any other feature (`bake_feature`).

### 5.3 Handing the form to solid features
- Solid features read `BodyGeom::Solid` only. A Form body needs conversion first.
- `FormConvertFeature` (`form.convert`): input a Form body; target `Brep` (FRM-06) or `Mesh`
  (FRM-16, `to_mesh_tol`); output replaces the body (default) or a new body.
- **Finish Form** (FRM-17) inserts or updates a `form.convert` (target B-rep) right after the
  form node when the preference "Form output" is `B-rep` (default; owner decision 2026-11-09, QUESTIONS #101). Then
  fillets, shells and booleans work on a form exactly as in Fusion. Re-entering the form
  edits the `form.create` node (timeline edit with rollback, S01), and the convert node and
  everything after it recompute on Finish.
- A Form body that stays SubD can still be exported (3MF/STL via `to_mesh_tol`), measured
  and displayed. Booleans with it go through the mesh path with the usual warning
  (architecture §Geometry representations).

### 5.4 References
- Inside the form, tools address cage elements by `VertId`/`EdgeId`/`FaceId` directly. Those
  are authored data of the form node, so they need no persistent naming.
- From outside, a Form body has body-level references only (S01 §Persistent naming,
  unchanged).
- The converted B-rep names each face by its root cage face and each edge by its cage edge
  (§7.5), so downstream fillets survive form edits that move vertices or add geometry
  elsewhere.

## 6. Edit Form (FRM-03)

- **Picking** is tool-internal (no framework change): the tool ray-casts the display mesh
  (`amg_mesh::Bvh`, MESH-03) to get the root face, then picks the nearest cage vertex or edge
  in screen space within 8 px of the projected control frame. The filter (vertex / edge /
  face / all) is the dialog's, like Fusion.
- **Selection modes** are topology queries on the cage: edge loop (through valence-4
  vertices, stops at extraordinary vertices and boundaries), edge ring (across quads, stops at
  n-gons), face loop, grow / shrink (one ring), select range between two picks (shortest
  loop path). Fusion shortcuts: double-click = loop, Alt+double-click = ring, Shift+Ctrl+arrow
  = grow/shrink; scope `Viewport`, enabled only in the `form.edit` context.
- **Manipulator**: the UI-06 triad (translate arrows, planar handles, rotation rings, uniform
  and axis scale). The frame is world, view or local (average normal of the selection).
- **Soft modification**: `SoftFalloff { None, Distance { radius, curve }, Rings { n } }`
  applied to vertex displacements; `curve` is `Linear` or `Smooth` (cubic). Distance is
  Euclidean from the nearest selected vertex.
- **Alt-drag** of faces or edges extrudes them (FRM-04 `extrude`) and then moves the new
  elements; Alt+Ctrl keeps creases on the new edges (Fusion behaviour).
- **Live preview** goes through the stencil table (§4.3) and the S11 preview layer; the UI
  thread only runs the mat-vec for small cages (≤ 2 000 faces), larger cages update on a job
  with latest-wins. Releasing the mouse commits one `EditFeature`.
- **Symmetry** (FRM-05) is applied inside the tool: the moved set is closed under the
  symmetry map before the stencils run.

## 7. SubD → NURBS B-rep (FRM-06, FRM-11)

### 7.1 Inputs
- One patch per cage quad, as Rhino does. An n-gon (n ≠ 4) is split into n quads around a new
  valence-n vertex (the first Catmull–Clark step, applied to that face only).
- A quad with two or more extraordinary vertices is split into 4 (one local refinement step),
  so every patch has at most one extraordinary corner (Peters 2000 needs that).
- Semi-sharp creases are refined locally until every sharpness is 0 or ∞.
- Each resulting patch maps back to its root cage face (`FaceId`, sub-index).

### 7.2 Regular quads
A quad with a regular 1-ring is exactly a uniform bicubic B-spline patch (§4.2). Its 16
control points convert to a `amg_nurbs::NurbsSurface3` of degree 3×3 with knots
`[0,0,0,0,1,1,1,1]` after Bézier conversion. Shared edges between regular patches are C2,
so G1 holds exactly.

### 7.3 Faces at an extraordinary vertex (G1)
- Each such quad becomes one bicubic B-spline surface with a double knot at ½ in u and v
  (2×2 Bézier pieces, C1 inside).
- Control points follow Peters 2000: corner points and tangents at the extraordinary vertex
  from the limit masks; tangent planes made consistent around the vertex (the "G1 at the
  extraordinary vertex" constraint); the cross-boundary derivatives along the edges out of the
  vertex built with Peters' scalar weights so neighbouring pieces are G1 across them.
- This is Rhino's "local G1x" shape (one double knot near the vertex). Deviation from the
  exact limit surface is measured on a 16×16 sample grid per face and reported.
- Option `ExtraordinaryMode::Approximate` (no G1 fix, smallest deviation) exists for
  debugging and comparison; it is not the default.

### 7.4 Assembly
- `make_face(Surface::BSpline(..), FaceBounds::Natural)` per patch (BREP-03), with `Tag`s on
  the four edges; `sew` (BREP-10, with history) into a shell; `make_solid` when the cage is
  closed, otherwise the result is an open shell (surface body, like Fusion's open forms).
- Infinitely sharp crease edges become ordinary B-rep edges where the two sides meet G0.
- Validation: `check` (BRepCheck) must pass; G1 is measured across every shared edge on 9
  sample points (normal angle), regular edges ≤ 1e-6 rad, edges at extraordinary vertices
  ≤ 0.5° (0.0087 rad), crease edges are excluded.
- Runs as a job with progress per patch and cancellation between patches.

### 7.5 Naming
- Face `Tag` = `(root FaceId << 8) | sub-index` (sub-index = which of the n quads of an n-gon,
  0 for quads). Edge `Tag` = `(1 << 63) | (root EdgeId << 8) | half` for edges on cage edges;
  interior edges of an n-gon get their own tag range. Tags go into the body naming through the
  S01/DOC-06 naming builder that solid features use.
- So a fillet on a converted form edge keeps its reference while the form is edited, as long
  as that cage edge exists.

### 7.6 Packed output (FRM-11)
Merges rectangular grids of regular quads (no creases, no extraordinary vertices inside) into
one B-spline surface with simple interior knots (C2), using `amg-nurbs` knot ops (KH-12). This
cuts face count (a 6×6×6 box form goes from 216 faces to 42: per side three regular grids
plus the four corner faces at the valence-3 vertices) and helps
OCCT fillets and shells. Grid detection is greedy, largest rectangle first, deterministic by
`FaceId` order. Face tags then name the grid by its lowest root `FaceId`, with a warning that
references into a packed face resolve by fingerprint after a topology edit.

## 8. Linear-solve tools (FRM-07, FRM-12)

- **Interpolate / pull**: the limit position of each cage vertex is `L · P` (limit masks, a
  sparse matrix). To make the limit surface pass through targets `T` at chosen vertices,
  solve `min ‖L_sel · P − T‖² + λ ‖P − P₀‖²` (λ small, keeps unselected vertices near their
  start). Sparse normal equations with `faer` (crate-internal, ADR-006).
- **Pull to target**: targets are closest points on a target body (mesh via `MeshSpatial`,
  solid via the tessellation at fine quality), iterated 3 times (closest points move as the
  surface moves).
- **Flatten**: targets are projections onto a plane (picked or least-squares fit).
- **Thicken** (FRM-12): offset the limit surface by `t` along limit normals at the cage
  vertices' limit points, solve for an offset cage that interpolates them (same solve), then
  bridge open boundaries with a quad strip (creased or smooth per option). A closed cage gets
  an inner shell with reversed orientation instead. Output is a new cage, so the result stays
  editable.

## 9. Quad remesh (FRM-08, P3)
Approach decided when FRM-08 is refined: a Rust Instant Meshes style field-aligned remesher
(prior art `quadrs`, MIT) or QuadriFlow via a `-sys` crate (MIT, built with
`BUILD_FREE_LICENSE`). Either way the output is a quad-dominant cage fitted with the §8 solve
so its limit surface lies on the input mesh.

## 10. Open points
- QUESTIONS #100: own evaluator with OpenSubdiv as oracle (decided by the owner, 2026-11-09).
- QUESTIONS #101: Finish Form adds/updates a B-rep `form.convert` (decided, 2026-11-09).
- QUESTIONS #102: FRM-10 reads OBJ control cages only (decided 2026-11-09 as OBJ + `.tsm`, changed
  2026-11-10 because no `.tsm` samples exist); see §11.

## 11. Foreign cages: OBJ and `.tsm` (FRM-10)
**2026-11-10:** FRM-10 implements OBJ only (owner, QUESTIONS #102). The `.tsm` research below is
kept for a later WP if sample files turn up.

- **OBJ** control cages: polygon-preserving read (quads and n-gons kept), `o`/`g` per body, crease
  tags where present.
- **`.tsm`** is the T-Splines plug-in (Rhino/Maya) model format, which Fusion's "Insert T-Spline"
  also reads. Research on 2026-11-09:
  - No official specification is published. Autodesk's help describes importing, not the layout.
    A McNeel forum answer says the storage details must be asked from T-Splines (Autodesk).
  - It is a text-based format ("T-spline Mesh"; a `.tss` holds a whole scene with other objects).
    A Computer-Aided Design 2016 paper (by the Beihang T-SPLINE library authors), "Reconsideration of T-spline data models and their
    exchanges using STEP", describes the T-spline data model that the file carries (control
    points, face/edge structure with T-junctions, knot intervals).
  - The only open implementation found is the Beihang "T-SPLINE" C++ library (GitHub
    GrapeTec/T-SPLINE, SourceForge tspline), with `tsm2obj`/`tsm2stp` tools. It is **LGPL-3.0**; its
    parser is not documented as a spec. We must not copy or translate its code (licence policy,
    ADR-012); reading it only to learn the layout is a grey area, so we do not rely on it.
  - Plan: a clean-room reader written from owner-supplied `.tsm` files (exported from Fusion or
    Rhino) and the facts in the paper. It reads control points, face lists and T-junction edges,
    and refines T-junctions into quads. Without samples the `.tsm` part of FRM-10 stays
    unstarted; it is a stretch goal and does not block the OBJ path.
  - Open for the owner: provide 3-5 `.tsm` samples (a box, a sphere with creases, a form with
    T-junctions) under `$AMG_TESTDATA_DIR/subd/tsm/`. If the owner has Fusion, exporting the same
    form to OBJ as well gives a reference limit mesh.
