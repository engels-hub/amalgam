# S02 — B-rep kernel (OpenCASCADE integration)

| | |
|---|---|
| **Status** | Ready |
| **Priority** | P0 |
| **Crates** | `amg-occt-sys`, `amg-brep` |
| **Lane** | A |
| **Depends on** | S00 (FND-01 toolchain + vcpkg pin, FND-02 workspace, FND-03 `amg-core`, FND-04 `amg-jobs`, FND-05 CI vcpkg cache, FND-06 testkit); ADR-002 (cleared 2026-10-20) |
| **Parity** | Underpins all solid modeling, STEP/IGES |

## Summary
A safe Rust wrapper around OpenCASCADE (OCCT) **8.0.x** that exposes exactly what Amalgam needs
through an object-safe `BrepKernel` trait. It covers shape construction (including face
builders from sketch profiles, polygons, analytic and B-spline surfaces), sweeps and lofts,
booleans, fillets/chamfers, offsets/shells/drafts, healing/sewing, topology and geometry
queries, mass properties, tessellation into `amg-mesh`-compatible buffers, history for
persistent naming, `.brep` serialization and STEP/IGES I/O.
`Shape` is an **immutable `Send + Sync`** handle (ADR-014 holds), and no OCCT type leaves
`amg-brep`. Evidence and C++-level detail are in
[docs/notes/occt-integration.md](../notes/occt-integration.md).

## Scope
**In:**
- The OCCT build through vcpkg manifest mode (port, features, triplet, toolkit list).
- The `amg-occt-sys` cxx bridge and C++ shim, exception mapping, logging bridge, and
  progress/cancel bridge.
- `Shape` handles, sub-shape indexing, topology/adjacency, and geometry extraction to plain
  Rust types.
- The `BrepKernel` trait and its OCCT backend `OcctKernel`:
  - primitives, face/wire builders, prism, revol, pipe/pipe-shell, raw ThruSections loft,
    helix;
  - booleans, fillet/chamfer, shell/thicken/offset, draft, replace/split/defeature;
  - healing, sewing and unify-same-domain, validity checks;
  - mass properties, distance/ray/classification, interference, plane sections and
    projection.
- Tessellation, `ShapeHistory` with roles and tags, and fingerprint queries for S01.
- Binary/text `.brep` I/O. STEP/IGES read/write with names, colours and assembly structure as
  plain data (XCAF stays inside).
- The robustness *mechanisms* S20 builds on: serde-able op parameters and a forwardable trait
  (for S20's capture decorator), validation policy,
  fuzzy/non-destructive booleans, structured failure reports with offending sub-shapes.
- A cargo feature `occt` (default on). Without it, `Shape` and the trait exist, but
  `OcctKernel` returns `KernelError::BackendUnavailable`. Crates above can then build
  without the OCCT C++ build, the same pattern as `amg-mesh/manifold`.

**Out:**
- Feature semantics: extents, `BodyOp`, dialog schemas (S04). Naming *policy* (S01 DOC-11):
  this crate only *reports* history.
- Robustness *strategies*: fillet strategy wrapper, analytic fillet, loft builder, failure dump
  files and replay CLI (S20). They are decorators over `BrepKernel` (see §Robustness).
- NURBS construction math (`amg-nurbs`, S20 KH-07). S02 only converts plain B-spline data to
  OCCT.
- Mapping STEP assemblies to components (S12 IO-06, S06). Conversion of `TessMesh` into
  `amg_mesh::IndexedMesh` (lives where both crates are visible; see §Tessellation).
- Hidden-line removal and drawings (S19 LAT-01, later).

## Design

### 1. Version, build, bindings (BREP-01)
- **OCCT 8.0.0** (vcpkg port `opencascade` 8.0.0, port-version 1). This is what the S00
  baseline `2026.07.29` ships, so no `overrides` entry is needed. 7.9.3#1 is the documented
  fallback if BREP-01 hits a blocking 8.0 regression (note §1.3).
  - Why 8.0: `Standard_Failure` derives from `std::exception`; STEP read/write is
    thread-safe per instance; mutexes are `std::mutex`; it is the maintained line;
    `TopoDS_TShape` and adaptor speedups.
  - 8.0.1 (later vcpkg baselines) fixes a `Standard_Strtod` thread race that affects
    parallel *text* `.brep` reads. We avoid it by design (binary caches; text reads under the
    global I/O lock). The baseline bump is an S00 decision (report).
- **vcpkg.json** (workspace root, first populated here):
  `{"name": "opencascade", "default-features": false}`.
  - No `freetype` (Visualization only).
  - No `tbb`: OCCT uses its own `OSD_ThreadPool`, and a second TBB next to Manifold's is a known
    crash source.
- **Triplet**: S00's `x64-windows-amg` stays (static libs, dynamic release CRT `/MD`,
  release-only). The port maps static linkage to `BUILD_LIBRARY_TYPE=Static` and defines
  `OCCT_STATIC_BUILD`. It leaves the CRT alone. Settles draft open question 1.
- **`amg-occt-sys/build.rs`**:
  - Reads `AMG_OCCT_DIR`, which `.cargo/config.toml [env]` sets to
    `vcpkg_installed/x64-windows-amg` relative to the workspace.
  - Never runs vcpkg or CMake itself.
  - Compiles the shim with `cxx_build` (`/std:c++17 /EHsc /MD /utf-8 /bigobj /permissive-`).
  - Links the static toolkit list from the note §2.
  - If `AMG_OCCT_DIR` is missing, fails with an actionable message.
- **Bindings**: a hand-written **cxx** bridge over a coarse C++ shim, one shim function per
  kernel operation (ADR-002).
  - autocxx is rejected: libclang dependency; `Handle`/NCollection templates; fine-grained
    calls. A plain `extern "C"` shim is rejected because cxx already gives ownership,
    `Result` mapping and Rust callbacks. Comparison in the note §3.1.
  - Plain data (points, profiles, curve/surface parameters, sub-shape refs, bulk buffers)
    crosses as cxx shared structs and `rust::Vec`. Opaque C++ types (`OcShape`, `OcIndex`,
    `OcOpOutput`) never leave `amg-occt-sys`'s private `ffi` module, which only `amg-brep`
    uses.
- **Exceptions**: a custom `rust::behavior::trycatch` catches `Standard_Failure`,
  `std::exception` and `...`. Every bridge function returns `Result`, so `std::terminate`
  can't be reached.
  - Algorithm failures that aren't exceptions are returned as structured statuses carrying
    the offending sub-shapes: `IsDone() == false`, BOPAlgo alerts, faulty fillet contours.
  - Access violations are not caught: no `/EHa`, no `OSD::SetSignal`. They go to FND-12
    minidumps.
- **Logging**: OCCT `Message` printers are replaced by one that forwards to `tracing`
  (target `occt`).
- **Progress/cancel**: a `Message_ProgressIndicator` subclass forwards `Show`/`UserBreak`
  to the `JobCtx`. Mid-operation cancel works for booleans, tessellation, ThruSections,
  pipe-shell, sewing and STEP/IGES. Fillet, chamfer, offset, draft and defeature check
  before and after only. These are documented per method.

### 2. Thread safety: `Shape` is immutable and `Sync` (resolves the ADR-014 conflict)
The draft said OCCT objects are "Send but not Sync". The evidence (note §4.1) is more precise:
- Reference counting (`Standard_Transient`) is atomic.
- Built topology and geometry carry no internal caches. Since OCCT 7.0, evaluation caches
  live in per-algorithm adaptors.
- Concurrent **reads** of one shape are therefore race-free.
- The hazards are algorithms that **write into their input TShapes**:
  - `BRepMesh_IncrementalMesh` stores triangulations on faces and edges (OCCT forum,
    tracker #22850);
  - booleans may enlarge input tolerances unless `SetNonDestructive(true)` is set;
  - `ShapeFix_*`, `BRepLib::SameParameter`/`UpdateTolerances` and in-place
    `UnifySameDomain` modify their argument.
- Located copies and unchanged sub-shapes in op results *share* TShapes, so a lock per
  `Shape` would not even protect against this.

**Decision:** `amg_brep::Shape` wraps an `Arc` around a C++ `TopoDS_Shape` that is **never
mutated after construction**. Every shim function belongs to one category:

| Cat. | Rule | Members |
|---|---|---|
| R | read-only, `const OcShape&`, concurrent on the same shape | exploration, index, queries, mass props, distance, classify, BRepCheck, BinTools/STEP write |
| N | non-destructive flag set | all `BRepAlgoAPI`/`BOPAlgo` ops (`SetNonDestructive(true)`), `UnifySameDomain` into a new result |
| C | algorithm runs on a private `BRepBuilderAPI_Copy` made inside the shim; the copy's history is composed into the result's | **tessellation**, `ShapeFix`, sewing inputs, anything the stress test catches mutating |
| G | process-wide mutex in `amg-brep` | IGES read/write; on 8.0.0 also text `.brep` and STEP *reads* (`Standard_Strtod` race, fixed in 8.0.0p1/8.0.1; the lock is dropped after a baseline bump) |

- So `unsafe impl Send + Sync for ffi::OcShape` is sound. The `SAFETY:` comment cites this
  table.
- `Shape` gets `static_assertions::assert_impl_all!(Shape: Send, Sync)`.
- **BREP-12 proves it** with an 8-thread stress test. Over one shared shape it checks: no
  crash; bit-identical query results; the shared input's BinTools hash is unchanged.
  Every later op WP re-runs the test with its op added.
- **Consequences:**
  - S01's interim `Mutex` around `BodyGeom::Solid` is dropped when BREP-02 lands.
  - Architecture §Threading's "the brep backend serializes calls per shape graph" is
    replaced by "shapes are immutable; ops never mutate inputs; only IGES and text-`.brep`
    reads take a global lock". That is an architecture text change, in the report.
  - S01 open question 6 (parallel branch recompute) no longer waits on any S02 locking.

### 3. Shapes, sub-shape indices, queries (BREP-02, BREP-09, BREP-17)
- `Shape`: `Clone` = `Arc` clone.
  - Holds a lazily built `SubShapeIndex` (`OnceLock`): faces, edges and vertices from
    `TopExp::MapShapes` (0-based, unlike OCCT's 1-based maps).
  - The index also has face↔edge and edge↔vertex adjacency, built from
    `MapShapesAndAncestors`.
- **Index stability contract** (S01 element map):
  - Indices are a deterministic function of the topology structure (DFS order, never
    addresses).
  - They are stable for an unmodified `Shape`, across processes, and across binary `.brep`
    round-trips. BREP-02 tests all three.
- `FaceId`/`EdgeId`/`VertexId` are `u32` newtypes valid only for the `Shape` they came from.
  Debug builds check this through a per-shape generation stamp.
- Geometry comes out as plain Rust data (`geom` module: `Surface`, `Curve3`), so queries
  never expose OCCT. Examples: plane frame, cylinder axis/radius, B-spline poles/knots, edge
  curve with parameter range. Used by S03 SK-09 (projection), S05 (measure), S07 (round-trip
  tests) and S12 (DXF of sections).
- **Fingerprint data** for S01 `GeomFingerprint` (BREP-05): per sub-shape,
  `SubShapeGeom { class, measure (area | length | 0), centroid, axis, bbox }`.
  - `class` uses this crate's own `GeomClass`, which mirrors S01's variants one-to-one.
  - `amg-doc` maps it. `amg-brep` can't depend on `amg-doc`.

### 4. Face and wire builders (BREP-03; requested by S03, S04, S07, S20)
`amg-brep` may depend only on `amg-occt-sys`, `amg-nurbs`, `amg-core` and `amg-jobs`, so it
can't take S03 or S07 types directly.

**Profiles (S03/S04):**
- `geom::Curve2d` mirrors S03's `Curve2` variant-for-variant and field-for-field: `Line`,
  `Circle`, `Arc`, `Ellipse`, `BSpline`. S04's `amg-features::solid` converts with a total
  `match`.
- `PlanarProfile { plane: Plane, outer: Loop2, holes: Vec<Loop2> }` → one planar face.
  - Each `ProfileEdge2` carries a caller `Tag(u64)`. S04 puts the sketch entity key there.
  - The builder returns `(EdgeId, Tag)` for each created edge, so later `Generated(edge)`
    history links lateral faces to sketch curves (naming hook, §6).
  - Loops are validated: closed within `Tolerance::linear`, outer CCW, holes CW, holes
    inside the outer loop.
  - Validation errors name the offending loop and edge (`KernelError::InvalidInput`).
- `make_planar_faces(&[PlanarProfile])` builds many faces in one FFI call (sketches with many
  regions).

**Polygons (S07 MESH-13):** `make_planar_polygon(outer: &[Point3], holes: &[Vec<Point3>])`.
Vertices are merged within tolerance, and the plane is fitted (max deviation must be
≤ `Tolerance::linear`, else `NotPlanar`).

**Faces on surfaces (S07 MESH-25, S09 FRM-06, S20 KH-08):**
`make_face(surface: &Surface, bounds: FaceBounds)`.
- `Surface` covers Plane, Cylinder, Cone, Sphere, Torus and `BSpline(BSplineSurfaceData)`.
  S07's `Primitive` converts to it in `amg-features::mesh`. S20's `amg-nurbs` surfaces
  convert via `impl From<&amg_nurbs::…>` added in KH-07/KH-08.
- `FaceBounds::Natural`: the full parameter domain, or a UV rectangle.
- `FaceBounds::Loops(Vec<Vec<Curve3>>)`: 3D boundary curves on (within `tol` of) the
  surface. Pcurves come from `BRepLib::BuildPCurveForEdgeOnPlane` / projection.
- `FaceBounds::Polylines(Vec<Vec<Point3>>)`: each polyline is projected to UV and
  approximated by a B-spline pcurve within `tol`.

**Wires and 3D curves:** `make_wire(&[Curve3])` builds sweep paths and rails. It returns an
`OpResult` with tags per edge.

**Assembly of faces:** `sew(&[Shape], SewOpts)` → shell/solid, and `make_solid(shell)` are in
BREP-10.

### 5. Operations (BREP-14, BREP-15, BREP-04, BREP-06, BREP-07, BREP-16, BREP-11)
Every op returns `OpResult { shape, history, warnings }` and takes `&JobCtx`. Kernel methods
take `&self` and inputs by `&Shape`.
- **Primitives (BREP-14):** box, cylinder, cone, sphere, torus, wedge. Each takes a `Plane`
  frame.
- **Prism / revol (BREP-14):**
  - Prism: from a face, wire or shell, with a direction vector or `to_shape` extents.
    Extent semantics stay in S04, which computes the distances.
  - Taper prism: `LocOpe_DPrism` / `BRepOffsetAPI_DraftAngle` on the result.
  - Revol: about a `Line3`, with an angle.
  - Roles: `StartCap`, `EndCap`, `Lateral`.
- **Pipe / pipe-shell (BREP-15):** `BRepOffsetAPI_MakePipeShell`.
  - Options: profile(s) along a spine wire; Frenet, corrected-Frenet, fixed or binormal
    mode; an auxiliary guide wire; twist and scale laws (`Law_Function` from samples);
    solid or shell output.
- **Loft (BREP-15):** a raw `BRepOffsetAPI_ThruSections` (ruled/smooth, closed, point
  sections, `CheckCompatibility`).
  - This is the *baseline* that KH-05 measures and the fallback in KH-08's chain. SOL-11
    uses S20's own builder, not this directly.
- **Booleans (BREP-04):** fuse, cut, common, split (`BRepAlgoAPI_Splitter`), and general
  fuse for N-ary.
  - Options:
    - `fuzzy` (default 0; S20 strategies raise it);
    - `glue` (`Off`/`Shift`/`Full`);
    - `non_destructive = true` (always on, category N);
    - `check_inverted`;
    - `use_obb`;
    - `parallel` (default per §7).
  - Failures carry the BOPAlgo alert names and the shapes involved.
- **Fillet / chamfer (BREP-06):**
  - Fillet: constant, variable (radius at parameters along the edge) and chord-length
    (converted to radius per edge from the dihedral angle).
  - Chamfer: equal distance, two distances, distance-angle.
  - Failure: `KernelError::LocalOpFailed { offending: faulty contours'/vertices' edges }`,
    from `NbFaultyContours`/`FaultyContour`/`NbFaultyVertices`.
  - Partial results (`HasResult`) are exposed only as a diagnostic shape for the UI, never
    as the op result.
- **Shell / thicken / offset (BREP-07):**
  - `MakeThickSolid` by join (arc / intersection), with removed faces and per-face
    thickness.
  - `BRepOffsetAPI_MakeOffsetShape` (offset face/shell, thicken a face or shell).
  - Roles: `ShellInner`, `OffsetFace`.
- **Draft / replace / split / defeature (BREP-16):**
  - `BRepOffsetAPI_DraftAngle` (neutral plane, pull direction, faces).
  - Replace face: `offset_faces` pushes the face past the new surface (when it lies beyond),
    then a `BRepAlgoAPI_Splitter` by a face on the surface drops the pieces past it (BREP-16 log;
    the planned defeature + half-space route cannot add material).
  - Split face by curves or by a tool (`BRepFeat_SplitShape`, `BRepAlgoAPI_Splitter`).
  - Split body by a plane or a face set.
  - Delete face / defeature: `BRepAlgoAPI_Defeaturing`.
- **Transforms (BREP-02):** rigid transforms set a `TopLoc_Location` (shares TShapes; safe under
  §2). Non-rigid ones use `BRepBuilderAPI_GTransform` (S04 scale).
- **Helix / threads (BREP-11):**
  - Helical spine: a `Geom2d_Line` on a cylindrical/conical surface, giving a 3D B-spline
    via `BRepLib::BuildCurves3d` on a private copy.
  - Thread profile swept with pipe-shell in binormal mode, then cut or fused.

### 6. History and naming hooks (BREP-05)
- `ShapeHistory` is index-based and serde-serialisable (S01 caches it inside
  `FeatureOutput`). For each operand `i` and each of its sub-shapes, the relation is one of
  `Modified → outputs`, `Generated → outputs` (which may cross kinds: edge→face, vertex→edge),
  `Deleted`, or `Kept` (the same TShape passes through).
- Op-semantic **roles** go on output sub-shapes (`StartCap`, `EndCap`, `Lateral`,
  `FilletBlend { contour }`, `ChamferFace`, `ShellInner`, `OffsetFace`, `SectionEdge`,
  `LoftSection { index }`, `SplitPiece { index }`, `Unexplained`), plus builder **tags**
  (§4).
- Built from OCCT `Generated`/`Modified`/`IsDeleted`, op-specific accessors
  (`FirstShape`/`LastShape`, `GeneratedFace`, `NewFaces`) and `BRepTools_History`. Internal
  steps (the category-C copy, `UnifySameDomain` after prism) are composed with
  `BRepTools_History::Merge`, so callers see one input→output step.
- **Deterministic split order**: outputs of one Modified input are sorted by (area desc,
  centroid lexicographic). DOC-11 may change the rule; it lives in one function.
- **Completeness guarantee**: every output face is Modified/Generated/Kept from some input or
  carries a role. BREP-05 tests this over the op corpus. Every op WP after BREP-05 adds its
  op to that test.
- S01's `NamingBuilder::from_history(input: &BodyNaming, history)` takes *one* input
  naming. Booleans and patterns have several operands, so a slice of `(operand index,
  &BodyNaming)` is needed. That is an S01 interface request, in the report.

### 7. Parallelism and determinism
- **Inside OCCT** (draft open question 2):
  - Booleans run `SetRunParallel(true)` by default, but only if BREP-04's determinism test
    passes: 20 cases × 10 runs give byte-identical BinTools output. Otherwise the default
    is off, and the result is recorded here.
  - `tessellate()` meshes one body with `InParallel = true`. `tessellate_many()` runs
    rayon over bodies with `InParallel = false`.
  - `KernelConfig::occt_threads` defaults to max(1, cores/2). BREP-04 benchmarks it.
- **Determinism**:
  - Indices come from structural DFS (§3).
  - BREP-04 and BREP-06 run each corpus op 10× and compare bytes.
  - A non-deterministic case is canonicalised (rebuild compounds in fingerprint order) and
    added to the S20 corpus.

### 8. Tessellation (BREP-08)
- `tessellate(shape, &TessQuality)` copies the topology (category C) and runs
  `BRepMesh_IncrementalMesh` with linear deflection, angular deflection, `relative` and
  `min_size`. It copies out a `TessMesh`:
  - `positions: Vec<Point3>`, `normals: Vec<Vec3>`, `tris: Vec<[u32; 3]>`, in f64 and mm.
    Vertices are **per face**, so normals are split at face boundaries for rendering.
  - `face_ranges: Vec<Range<u32>>`: the triangle range per `FaceId`, in index order. This
    maps 1:1 to S07 `FaceGroups` and to S10 picking (triangle → face).
  - `weld: Vec<u32>`: per vertex, a canonical shared-vertex id. Nodes on a shared edge come
    from that edge's single `Poly_PolygonOnTriangulation`, so welding is exact (no distance
    search).
  - `edges: Vec<EdgePolyline { edge: EdgeId, points: Range<u32> }>` + `edge_points:
    Vec<Point3>`, for edge rendering and edge picking.
  - Optional `uvs` (texture-mapped appearances, P2).
- **amg-mesh compatibility**: `TessMesh::to_indexed_parts()` returns
  `(Vec<Point3>, Vec<[u32; 3]>, Vec<u32> /* face of each tri */)`, already welded and
  watertight. These are exactly the core fields of `amg_mesh::IndexedMesh::new(positions,
  tris)` (S07), plus a per-triangle face id for `FaceGroups`.
  - The conversion (`solid_to_mesh`) is ~10 lines in `amg-features::solid`. `amg-io` has its
    own for STL/3MF export, because it can see both crates but not `amg-features`.
  - It adds the S07 `GroupOrigin::BrepFace { face }` variant, an S07 interface request.
  - `TessMesh::as_soup()` gives the `amg_testkit::TriSoup` view for golden tests.
- **Quality presets** (`TessQuality::{preview, display, export}`; deflection relative to the
  bbox diagonal, clamped):

  | Preset | Linear deflection | Angular |
  |---|---|---|
  | preview | 0.5 % | 0.5 rad |
  | display | 0.1 % | 0.35 rad |
  | export | 0.01 %, ≤ 0.01 mm | 0.2 rad |

  S12 IO-09 builds its per-body controls on `TessQuality`'s fields.

### 9. Validation, healing, I/O (BREP-10, BREP-13, BREP-02)
- **Validation**: `check(shape) -> CheckReport` runs `BRepCheck_Analyzer` and
  `BOPAlgo_ArgumentAnalyzer` (self-intersection, small edges). Problems are listed per
  sub-shape.
  - `KernelConfig::validate`: `Off` / `DebugOnly` (default) / `Always`. With validation on,
    every op result is checked, and an invalid result becomes
    `KernelError::InvalidResult { report }`. The draft's "always check in debug builds" is
    kept as the default.
  - S20's validation gate (`robust::validate`) builds on `check` + `max_tolerance`.
- **Healing (BREP-10)**:
  - `heal(shape, HealOpts)`: `ShapeFix_Shape` (category C, `copyGeom` true) and
    `ShapeFix_Wireframe`.
  - `unify_same_domain(shape, UnifyOpts)`.
  - `sew(&[Shape], SewOpts)`: `BRepBuilderAPI_Sewing`, then `make_solid`
    (`BRepBuilderAPI_MakeSolid` + `ShapeFix_Solid` orientation).
  - All return history.
- **`.brep` I/O (BREP-02)**:
  - Binary (`BinTools`, versioned header `AMGB` + format byte) for S01 caches and
    `bodies/`. It is bit-exact, and indices are stable across it.
  - Text (`BRepTools`, global lock on read) for S20 failure dumps and debugging.
- **STEP / IGES (BREP-13)**: `STEPCAFControl_Reader`/`Writer` (one per call, thread-safe in
  8.0; on 8.0.0 reads still take the global lock because of the `strtod` race) and IGES
  (global lock).
  - Output is a `ForeignModel`: a tree of plain-data nodes (name, transform, colour,
    optional shape, children), so S12 maps it to components without seeing XCAF.
  - Import runs `heal` by default (`ImportOpts::heal`).
  - Units: the file's unit is converted to mm (`xstep.cascade.unit` = MM through the 8.0
    `DE_*` parameters).

### 10. Robustness mechanisms (coordinated with S20)
S02 provides mechanisms; S20 builds strategies on them **without changing S02 code**:
- **Decorator pattern**: S20 strategies are `struct HardenedKernel<K: BrepKernel>(K)`
  implementing `BrepKernel` (KH-03 fillet strategy chain, KH-08 loft chain). Features take
  `&dyn BrepKernel`, so swapping in the hardened kernel is a one-line change in the recompute
  context.
- **Failure capture is S20's** (`amg_brep::capture`, KH-01, aligned with
  `docs/notes/kernel-robustness.md` §1). S20's `RecordingKernel<K>` decorator serialises each
  call as its own `KernelOp` and dumps it on failure or write-ahead. S02 guarantees what that
  needs (S20 request R3):
  - every op parameter type in the trait is `Clone + Debug + PartialEq + Serialize +
    Deserialize`, and inputs are only `&Shape`/`&[Shape]` (so a dump can reference them by
    position);
  - the trait is object-safe and every method can be forwarded unchanged;
  - text `.brep` write/read (`Shape::write_text`/`read_text`) never contains a triangulation,
    since shapes are never meshed in place (§2).

  S02 itself has no capture hook. The WP that adds a trait method also adds its `KernelOp`
  variant (rule from the S20 note §1.2).
- **Structured failures**: `KernelError` variants carry offending `SubShape`s (fillet edges,
  boolean alert shapes, invalid loop edges) and the OCCT exception type/message. They
  implement `amg_core::Diagnostic` with hints ("reduce radius", "try fuzzy value").
- **Knobs strategies need**:
  - boolean `fuzzy`/`glue`;
  - `heal` and `unify_same_domain` as standalone ops;
  - `check`;
  - a fillet max-radius probe `fillet_max_radius(shape, edges, hi, iterations)` by bisection
    (BREP-06), which KH-03 reports to the user;
  - `FilletOpts` exposing the blend shape (`Rational`/`QuasiAngular`/`Polynomial`) and the
    approximation parameters `tol_app_3d`/`fleche` (`SetParams`), for S20's shape-variant step
    (request R7);
  - `Shape::max_tolerance()` (`ShapeAnalysis_ShapeTolerance`, max over all sub-shapes) for
    S20's validation gate;
  - `check` includes the optional `BOPAlgo_ArgumentAnalyzer` self-interference pass
    (`CheckOpts::self_interference`, request R5);
  - `ShapeHistory::compose` (request R8), so healing and unify steps chain into the op
    history;
  - `ThruSections` and pipe-shell exposed raw (BREP-15) for KH-08's fallback chain.

## Interfaces
Public API of `amg-brep`. `amg-occt-sys` is private to lane A: only `amg-brep` depends on it.
Bodies are omitted.

```rust
// ===== amg_brep (lib.rs) ===================================================================
pub use amg_core::{Point3, Vec3, Point2, Vec2, Transform, Plane, Line3, Dir3, Aabb3, Tolerance};
pub use amg_jobs::JobCtx;

/// "occt-8.0.0;amg-brep-<FORMAT_VERSION>;policy-<robust::POLICY_VERSION>"; part of S01's
/// ENGINE_SALT. The policy part comes from S20's `amg_brep::robust` module (same crate).
pub fn kernel_version() -> &'static str;
/// Idempotent process init (message printer, OSD thread pool size). Called by OcctKernel::new.
pub fn init(cfg: &KernelConfig);

pub type KResult<T> = Result<T, KernelError>;

// ===== shape ===============================================================================
/// Immutable B-rep shape. Clone = Arc clone. Send + Sync (see §2).
#[derive(Clone)]
pub struct Shape { /* Arc<ShapeInner> */ }
static_assertions::assert_impl_all!(Shape: Send, Sync);

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum ShapeKind { Compound, CompSolid, Solid, Shell, Face, Wire, Edge, Vertex }
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub enum SubKind { Face, Edge, Vertex }
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
#[serde(transparent)] pub struct FaceId(pub u32);
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
#[serde(transparent)] pub struct EdgeId(pub u32);
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
#[serde(transparent)] pub struct VertexId(pub u32);
/// Kind + 0-based in-shape index (TopExp::MapShapes order). Valid only for its Shape.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct SubShape { pub kind: SubKind, pub index: u32 }

impl Shape {
    pub fn kind(&self) -> ShapeKind;
    pub fn is_null(&self) -> bool;
    pub fn ptr_eq(&self, o: &Shape) -> bool;                // same Arc
    pub fn is_same(&self, o: &Shape) -> bool;               // OCCT IsSame (same TShape + location)
    pub fn count(&self, k: SubKind) -> u32;
    pub fn faces(&self) -> impl ExactSizeIterator<Item = FaceId> + '_;
    pub fn edges(&self) -> impl ExactSizeIterator<Item = EdgeId> + '_;
    pub fn vertices(&self) -> impl ExactSizeIterator<Item = VertexId> + '_;
    pub fn solids(&self) -> Vec<Shape>;                     // top-level solids in order
    pub fn sub_shape(&self, s: SubShape) -> KResult<Shape>; // extract as standalone Shape (shares TShape)
    // adjacency
    pub fn face_edges(&self, f: FaceId) -> &[EdgeId];       // outer wire first, then holes, in wire order
    pub fn face_wires(&self, f: FaceId) -> Vec<Vec<EdgeId>>;
    pub fn edge_faces(&self, e: EdgeId) -> &[FaceId];       // 0 (free), 1 (boundary/seam), 2 (manifold), >2
    pub fn edge_vertices(&self, e: EdgeId) -> [VertexId; 2];
    pub fn vertex_edges(&self, v: VertexId) -> &[EdgeId];
    pub fn is_seam(&self, e: EdgeId, f: FaceId) -> bool;
    // basic geometry (plain data)
    pub fn bbox(&self) -> Aabb3;                            // optimal (BRepBndLib::AddOptimal), cached
    pub fn surface(&self, f: FaceId) -> geom::Surface;      // underlying surface, untrimmed
    pub fn face_reversed(&self, f: FaceId) -> bool;
    pub fn face_uv_bounds(&self, f: FaceId) -> [f64; 4];    // umin, umax, vmin, vmax
    pub fn curve(&self, e: EdgeId) -> Option<(geom::Curve3, f64, f64)>; // None for degenerated edges
    pub fn vertex_point(&self, v: VertexId) -> Point3;
    pub fn tolerance(&self, s: SubShape) -> f64;            // OCCT sub-shape tolerance, mm
    pub fn max_tolerance(&self) -> f64;                     // max over all sub-shapes (S20 validation gate)
    // transforms
    pub fn transformed(&self, t: &Transform) -> KResult<Shape>; // rigid → TopLoc; else Err(NonRigid)
    pub fn gtransformed(&self, t: &Transform) -> KResult<OpResult>; // any affine (scale), new geometry
    // serialization
    pub fn write_bin<W: std::io::Write>(&self, w: W) -> KResult<()>;   // "AMGB" header + BinTools
    pub fn read_bin<R: std::io::Read>(r: R) -> KResult<Shape>;
    pub fn write_text<W: std::io::Write>(&self, w: W) -> KResult<()>;  // BRepTools ASCII (.brep)
    pub fn read_text<R: std::io::Read>(r: R) -> KResult<Shape>;        // global lock (§2, cat. G)
    /// Approximate heap footprint for S01's LRU cache accounting.
    pub fn estimated_bytes(&self) -> u64;
}
impl std::fmt::Debug for Shape { /* kind + counts + bbox */ }

// ===== geom (plain data in/out; no OCCT) ===================================================
pub mod geom {
    use super::*;
    /// Caller-chosen tag carried from builder inputs to created edges/faces (naming hook).
    #[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
    pub struct Tag(pub u64);

    // ---- 2D (profile) curves: mirror amg_sketch::geom::Curve2 one-to-one (S03) ----
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum Curve2d {
        Line   { a: Point2, b: Point2 },
        Circle { center: Point2, radius: f64 },                                  // full, CCW from +u
        Arc    { center: Point2, radius: f64, start_angle: f64, sweep: f64 },    // sweep>0 CCW
        Ellipse{ center: Point2, major: Vec2, minor_radius: f64, t0: f64, t1: f64 },
        BSpline(BSpline2dData),
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct BSpline2dData { pub degree: u8, pub knots: Vec<f64>, pub poles: Vec<Point2>, pub weights: Option<Vec<f64>> }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct ProfileEdge2 { pub curve: Curve2d, pub tag: Tag }                 // oriented along the loop
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct Loop2 { pub edges: Vec<ProfileEdge2> }
    /// Outer loop CCW, holes CW (S03 convention), in `plane`'s local (u, v).
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct PlanarProfile { pub plane: Plane, pub outer: Loop2, pub holes: Vec<Loop2> }

    // ---- 3D curves ----
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum Curve3 {
        Line   { origin: Point3, dir: Dir3 },                         // param = distance
        Circle { frame: Plane, radius: f64 },                         // param = angle from frame.x
        Ellipse{ frame: Plane, major_radius: f64, minor_radius: f64 },
        BSpline(BSplineCurveData),
        Other,                                                        // offset/other; use tessellate_edge
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct BSplineCurveData { pub degree: u8, pub knots: Vec<f64>, pub mults: Vec<u32>,
                                  pub poles: Vec<Point3>, pub weights: Option<Vec<f64>>, pub periodic: bool }
    /// A trimmed 3D curve: input to wires (sweep paths, rails, face loops).
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct CurveSeg { pub curve: Curve3, pub t0: f64, pub t1: f64, pub tag: Tag }

    // ---- surfaces ----
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum Surface {
        Plane    (Plane),
        Cylinder { frame: Plane, radius: f64 },                       // axis = frame.normal
        Cone     { frame: Plane, ref_radius: f64, half_angle: f64 },  // radius at frame.origin
        Sphere   { frame: Plane, radius: f64 },
        Torus    { frame: Plane, major: f64, minor: f64 },
        BSpline  (BSplineSurfaceData),
        Other    { class: GeomClass },                                // revolution/extrusion/offset: read-only
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct BSplineSurfaceData {
        pub u_degree: u8, pub v_degree: u8,
        pub u_knots: Vec<f64>, pub u_mults: Vec<u32>, pub v_knots: Vec<f64>, pub v_mults: Vec<u32>,
        pub poles: Vec<Point3>,          // row-major: u index outer, nu × nv
        pub nu: u32, pub nv: u32,
        pub weights: Option<Vec<f64>>,
        pub u_periodic: bool, pub v_periodic: bool,
    }
    /// Mirrors amg_doc::GeomClass (S01) variant-for-variant.
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
    pub enum GeomClass { Plane, Cylinder, Cone, Sphere, Torus, BSpline, Other, Line, Circle, Ellipse, Curve, Point }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum FaceBounds {
        Natural,                                 // full domain (finite surfaces) or error
        UvRect { u0: f64, u1: f64, v0: f64, v1: f64 },
        Loops(Vec<Vec<CurveSeg>>),               // first = outer; 3D curves on the surface
        Polylines(Vec<Vec<Point3>>),             // first = outer; projected + approximated within tol
    }
}

// ===== queries (BREP-05 fingerprints, BREP-09, BREP-17) ====================================
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SubShapeGeom {
    pub class: geom::GeomClass,
    pub measure: f64,              // face area mm² | edge length mm | 0 for vertices
    pub centroid: Point3,          // area/length-weighted
    pub axis: Option<Vec3>,        // plane normal (oriented) / cyl-cone-torus axis / line dir
    pub bbox: Aabb3,
}
#[derive(Clone, Debug, PartialEq)]
pub struct MassProps {
    pub volume: f64, pub area: f64, pub centroid: Point3,
    pub inertia: amg_core::Mat3,   // about centroid, unit density (mm⁵); scale by density in S05
    pub principal_moments: [f64; 3], pub principal_axes: [Dir3; 3],
}
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Classification { Inside, Outside, OnBoundary }
#[derive(Clone, Debug, PartialEq)]
pub struct DistanceResult { pub distance: f64, pub on_a: Point3, pub on_b: Point3,
                            pub sub_a: Option<SubShape>, pub sub_b: Option<SubShape> }
#[derive(Clone, Debug, PartialEq)]
pub struct RayHit { pub t: f64, pub point: Point3, pub face: FaceId, pub uv: Point2, pub normal: Dir3 }
#[derive(Clone, Debug, PartialEq)]
pub struct SurfacePoint { pub point: Point3, pub normal: Dir3, pub d1u: Vec3, pub d1v: Vec3,
                          pub k_min: f64, pub k_max: f64, pub dir_min: Vec3, pub dir_max: Vec3 }
// BREP-09: normal out of the material; curvatures convex-positive against it (sphere +1/r).
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeEnds { pub p0: Point3, pub t0: Dir3, pub p1: Point3, pub t1: Dir3 }   // R-S04-1
pub const MASS_PROPS_EPS: f64 = 1e-12;   // adaptive BRepGProp bound behind mass_props

// ===== tessellation (BREP-08) ==============================================================
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TessQuality {
    pub linear: f64,            // mm, or fraction of bbox diagonal if `relative`
    pub angular: f64,           // rad
    pub relative: bool,
    pub min_size: f64,          // mm; 0 = OCCT default
    pub max_linear_abs: Option<f64>, // clamp for relative mode (export: 0.01 mm)
}
impl TessQuality { pub fn preview() -> Self; pub fn display() -> Self; pub fn export() -> Self; }
#[derive(Clone, Debug, Default, PartialEq)]
pub struct EdgePolyline { pub edge: EdgeId, pub points: std::ops::Range<u32> }
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TessMesh {
    pub positions: Vec<Point3>,                 // per-face vertices (split normals)
    pub normals: Vec<Vec3>,                     // unit, len == positions.len()
    pub tris: Vec<[u32; 3]>,                    // CCW seen from outside
    pub face_ranges: Vec<std::ops::Range<u32>>, // triangle range per FaceId (len == face count)
    pub weld: Vec<u32>,                         // canonical shared-vertex id per vertex (exact)
    pub edges: Vec<EdgePolyline>,
    pub edge_points: Vec<Point3>,
    pub uvs: Option<Vec<Point2>>,
}
impl TessMesh {
    pub fn face_of_triangle(&self, tri: u32) -> FaceId;           // binary search on face_ranges
    /// Welded, watertight (for closed solids): (positions, tris, face id per tri).
    pub fn to_indexed_parts(&self) -> (Vec<Point3>, Vec<[u32; 3]>, Vec<u32>);
    pub fn triangle_count(&self) -> usize;
    pub fn estimated_bytes(&self) -> u64;
}

// ===== history (BREP-05) ===================================================================
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct InputSub { pub operand: u16, pub sub: SubShape }   // operand = position in the op's input list
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Relation {
    Modified(smallvec::SmallVec<[SubShape; 2]>),   // same kind; split order: area desc, centroid lex
    Generated(smallvec::SmallVec<[SubShape; 2]>),  // may cross kinds (edge → face, vertex → edge)
    Kept(SubShape),                                // same TShape passed through
    Deleted,
}
#[derive(Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
pub enum OpRole {
    StartCap, EndCap, Lateral,
    FilletBlend { contour: u32 }, ChamferFace, CornerPatch,
    ShellInner, OffsetFace, SectionEdge,
    LoftSection { index: u32 }, SplitPiece { index: u32 },
    Unexplained,                                    // created with no source and no other role
}
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ShapeHistory {
    /// Sorted by InputSub; one entry per input sub-shape that has any relation (Generated and
    /// Modified may both appear for one input → two entries).
    pub entries: Vec<(InputSub, Relation)>,
    pub roles: Vec<(SubShape, OpRole)>,             // on output sub-shapes
    pub tags: Vec<(SubShape, geom::Tag)>,           // builder tags on output edges/faces
    pub operand_counts: Vec<[u32; 3]>,              // faces/edges/vertices per operand (validation)
}
impl ShapeHistory {
    pub fn of_input(&self, i: InputSub) -> impl Iterator<Item = &Relation> + '_;
    /// Inverse view: every input relation that produced output sub-shape `o`.
    pub fn sources_of(&self, o: SubShape) -> Vec<(InputSub, &Relation)>;
    pub fn role_of(&self, o: SubShape) -> Option<&OpRole>;
    pub fn tag_of(&self, o: SubShape) -> Option<geom::Tag>;
    /// Compose `self` (A→B) with `next` (B→C) where B is `next`'s operand 0 (S20 request R8).
    pub fn compose(&self, next: &ShapeHistory) -> ShapeHistory;
    /// BREP-05: sort multi-output lists into the split order of `result` (the one place it is applied).
    pub fn order_splits(&mut self, result: &Shape) -> KResult<()>;
    pub fn unexplained_faces(&self, face_count: u32) -> Vec<FaceId>;   // completeness check
    pub fn range_error(&self, output_counts: [u32; 3]) -> Option<String>;
}
/// BREP-05 split-order key (naming note §4.3, Q5): (−q(measure), q(cx), q(cy), q(cz), index), q = round(x / 1e-6).
pub const SPLIT_QUANTUM: f64 = 1e-6;
pub type SplitKey = (i64, i64, i64, i64, u32);
pub fn split_order_key(g: &SubShapeGeom, index: u32) -> SplitKey;
#[derive(Clone, Debug)]
pub struct OpResult { pub shape: Shape, pub history: ShapeHistory, pub warnings: Vec<KernelWarning> }
impl OpResult { pub fn per_solid(&self) -> KResult<Vec<OpResult>>; }   // R-S04-2 (BREP-05)
#[derive(Clone, Debug, PartialEq)]
pub enum KernelWarning { Healed { fixes: u32 }, ToleranceGrown { max: f64 }, FuzzyUsed(f64), PartialHistory }

// ===== op options ==========================================================================
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum BoolOp { Fuse, Cut, Common }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum Glue { #[default] Off, Shift, Full }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BoolOpts { pub fuzzy: f64, pub glue: Glue, pub check_inverted: bool, pub use_obb: bool,
                      pub parallel: Option<bool> /* None = KernelConfig default */ }
impl Default for BoolOpts { fn default() -> Self; }   // fuzzy 0, Off, true, false, None
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PrismExtent { Vector(Vec3), Infinite { dir: Dir3, both: bool } }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PrismOpts { pub taper: f64 /* rad, 0 = none */, pub merge_coplanar: bool /* UnifySameDomain */ }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum PipeMode { CorrectedFrenet, Frenet, Fixed { binormal: Dir3 }, Binormal(Dir3), Guide { wire: usize /* operand */, contact: bool },
                   Support { shell: usize /* operand; R-S04-7, BREP-15 */ } }   // aux operand = the one after the spine
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PipeOpts { pub mode: PipeMode, pub twist: Option<Vec<(f64, f64)>> /* (param 0..1, rad) */,
                      pub scale: Option<Vec<(f64, f64)>>, pub solid: bool, pub transition: Transition }
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Transition { Transformed, RightCorner, RoundCorner }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThruSectionsOpts { pub solid: bool, pub ruled: bool, pub check_compatibility: bool,
                              pub smoothing: bool, pub max_degree: u8 }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum LoftSection { Wire(usize /* operand */), Point(Point3) }
impl Default for PipeOpts { fn default() -> Self; }          // BREP-15: CorrectedFrenet, no laws, solid, Transformed
impl Default for ThruSectionsOpts { fn default() -> Self; }  // BREP-15: solid, smooth, check_compatibility, no smoothing, max_degree 0 (OCCT's 8)
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum FilletSpec {
    Constant(f64),
    Variable(Vec<(f64 /* normalized param 0..1 */, f64 /* radius */)>),
    Chord(f64),                                     // chord length, converted per edge
}
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum FilletShape { #[default] Rational, QuasiAngular, Polynomial }   // ChFi3d_FilletShape
#[derive(Clone, Debug, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct FilletOpts { pub shape: FilletShape, pub tol_app_3d: Option<f64>, pub fleche: Option<f64> } // None = OCCT default
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ChamferSpec { Equal(f64), TwoDistances { d1: f64, d2: f64, face: FaceId }, DistanceAngle { d: f64, angle: f64, face: FaceId } }
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum OffsetJoin { Arc, Intersection }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ShellOpts { pub thickness: f64 /* <0 inward */, pub join: OffsetJoin,
                       pub per_face: Vec<(FaceId, f64)>, pub tol: f64 }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DraftOpts { pub pull: Dir3, pub angle: f64, pub neutral: Plane }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct HealOpts { pub tol: f64, pub fix_small_edges: bool, pub fix_orientation: bool, pub max_tol: f64 }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct UnifyOpts { pub faces: bool, pub edges: bool, pub concat_bsplines: bool, pub lin_tol: f64, pub ang_tol: f64 }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SewOpts { pub tol: f64, pub non_manifold: bool, pub make_solid: bool }
// BREP-10 defaults: HealOpts { 1e-6, true, true, 1e-3 }, UnifyOpts = merge_coplanar's
// (faces, edges, no concat, 1e-7 mm, 1e-12 rad), SewOpts { 1e-6, false, true }.
impl Default for HealOpts { fn default() -> Self; }
impl Default for UnifyOpts { fn default() -> Self; }
impl Default for SewOpts { fn default() -> Self; }
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct HelixSpec { pub axis: Plane, pub radius: f64, pub pitch: f64, pub height: f64,
                       pub taper: f64 /* rad */, pub left_handed: bool }
#[derive(Clone, Debug)]
pub struct FaceBuild { pub face: Shape, pub edge_tags: Vec<(EdgeId, geom::Tag)> }
#[derive(Clone, Debug, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct CheckOpts { pub self_interference: bool /* BOPAlgo_ArgumentAnalyzer; slower */ }
#[derive(Clone, Debug, PartialEq)]
pub struct CheckReport { pub valid: bool, pub problems: Vec<(Option<SubShape>, String /* BRepCheck_Status / BOPAlgo check name */)> }

// ===== the trait ===========================================================================
/// Object-safe. All mutating ops: inputs are never modified (§2); `ctx` gives progress and
/// cancellation (mid-op where OCCT supports it, else before/after; noted per method).
pub trait BrepKernel: Send + Sync {
    // -- builders (BREP-03) --
    fn make_planar_faces(&self, profiles: &[geom::PlanarProfile], tol: &Tolerance) -> KResult<Vec<FaceBuild>>;
    fn make_planar_polygon(&self, outer: &[Point3], holes: &[Vec<Point3>], tol: &Tolerance) -> KResult<Shape>;
    fn make_face(&self, surface: &geom::Surface, bounds: &geom::FaceBounds, tol: f64) -> KResult<FaceBuild>;
    fn make_wire(&self, segs: &[geom::CurveSeg], tol: &Tolerance) -> KResult<OpResult>;
    // -- primitives, sweeps (BREP-14, BREP-15, BREP-11) --
    fn make_box(&self, frame: &Plane, size: Vec3) -> KResult<OpResult>;
    fn make_cylinder(&self, frame: &Plane, radius: f64, height: f64, angle: f64) -> KResult<OpResult>;
    fn make_cone(&self, frame: &Plane, r1: f64, r2: f64, height: f64, angle: f64) -> KResult<OpResult>;
    fn make_sphere(&self, frame: &Plane, radius: f64) -> KResult<OpResult>;
    fn make_torus(&self, frame: &Plane, major: f64, minor: f64, angle: f64) -> KResult<OpResult>;
    fn prism(&self, profile: &Shape, extent: &PrismExtent, opts: &PrismOpts, ctx: &JobCtx) -> KResult<OpResult>;
    fn revol(&self, profile: &Shape, axis: &Line3, angle: f64, ctx: &JobCtx) -> KResult<OpResult>;
    /// operands: [profiles..., spine, guide?]; `profiles` = count of leading profile operands.
    fn pipe(&self, operands: &[Shape], profiles: usize, opts: &PipeOpts, ctx: &JobCtx) -> KResult<OpResult>;
    fn thru_sections(&self, wires: &[Shape], sections: &[LoftSection], opts: &ThruSectionsOpts, ctx: &JobCtx) -> KResult<OpResult>;
    fn helix(&self, spec: &HelixSpec) -> KResult<OpResult>;            // wire
    // -- booleans (BREP-04) --
    fn boolean(&self, op: BoolOp, objects: &[Shape], tools: &[Shape], opts: &BoolOpts, ctx: &JobCtx) -> KResult<OpResult>;
    fn split(&self, objects: &[Shape], tools: &[Shape], opts: &BoolOpts, ctx: &JobCtx) -> KResult<OpResult>;
    fn section(&self, a: &Shape, b: &Shape, ctx: &JobCtx) -> KResult<OpResult>;   // edges (compound)
    // -- local ops (BREP-06, BREP-07, BREP-16) --
    fn fillet(&self, s: &Shape, edges: &[(EdgeId, FilletSpec)], opts: &FilletOpts, ctx: &JobCtx) -> KResult<OpResult>;
    fn chamfer(&self, s: &Shape, edges: &[(EdgeId, ChamferSpec)], ctx: &JobCtx) -> KResult<OpResult>;
    /// Largest constant radius ≤ `hi` that succeeds, by bisection (`iterations` steps).
    fn fillet_max_radius(&self, s: &Shape, edges: &[EdgeId], hi: f64, iterations: u32, ctx: &JobCtx) -> KResult<f64>;
    fn shell(&self, s: &Shape, remove: &[FaceId], opts: &ShellOpts, ctx: &JobCtx) -> KResult<OpResult>;
    fn offset(&self, s: &Shape, distance: f64, join: OffsetJoin, tol: f64, ctx: &JobCtx) -> KResult<OpResult>;
    fn thicken(&self, s: &Shape, thickness: f64, join: OffsetJoin, ctx: &JobCtx) -> KResult<OpResult>;
    fn draft(&self, s: &Shape, faces: &[FaceId], opts: &DraftOpts, ctx: &JobCtx) -> KResult<OpResult>;
    fn defeature(&self, s: &Shape, faces: &[FaceId], ctx: &JobCtx) -> KResult<OpResult>;     // delete face
    fn replace_face(&self, s: &Shape, face: FaceId, with: &geom::Surface, ctx: &JobCtx) -> KResult<OpResult>;
    fn split_faces(&self, s: &Shape, faces: &[FaceId], tool: &Shape, ctx: &JobCtx) -> KResult<OpResult>;
    /// R-S04-5 (BREP-16): the listed faces moved along their normals, the others extended/trimmed.
    fn offset_faces(&self, s: &Shape, faces: &[(FaceId, f64)], ctx: &JobCtx) -> KResult<OpResult>;
    // -- healing, assembly (BREP-10) --
    fn heal(&self, s: &Shape, opts: &HealOpts, ctx: &JobCtx) -> KResult<OpResult>;
    fn unify_same_domain(&self, s: &Shape, opts: &UnifyOpts) -> KResult<OpResult>;
    fn sew(&self, faces: &[Shape], opts: &SewOpts, ctx: &JobCtx) -> KResult<OpResult>;
    fn make_solid(&self, shell: &Shape) -> KResult<OpResult>;      // BREP-10: closed shells → outward solids
    fn make_compound(&self, parts: &[Shape]) -> KResult<Shape>;
    fn check(&self, s: &Shape, opts: &CheckOpts) -> CheckReport;
    // -- queries (BREP-05, BREP-09, BREP-17) --
    fn sub_geom(&self, s: &Shape, sub: SubShape) -> KResult<SubShapeGeom>;
    fn sub_geom_all(&self, s: &Shape) -> KResult<[Vec<SubShapeGeom>; 3]>;  // faces, edges, vertices in index order
    fn mass_props(&self, s: &Shape) -> KResult<MassProps>;
    fn classify(&self, s: &Shape, p: Point3, tol: f64) -> KResult<Classification>;
    fn distance(&self, a: &Shape, b: &Shape) -> KResult<DistanceResult>;
    fn closest_point(&self, s: &Shape, p: Point3) -> KResult<DistanceResult>;
    fn ray_cast(&self, s: &Shape, ray: &amg_core::Ray3, max_t: f64) -> KResult<Option<RayHit>>;
    fn interference(&self, a: &Shape, b: &Shape, ctx: &JobCtx) -> KResult<Option<OpResult>>;   // common volume
    fn eval_surface(&self, s: &Shape, f: FaceId, uv: Point2) -> KResult<SurfacePoint>;
    fn project_point_on_face(&self, s: &Shape, f: FaceId, p: Point3) -> KResult<(Point2, Point3)>;
    fn edge_ends(&self, s: &Shape, e: EdgeId) -> KResult<EdgeEnds>;   // R-S04-1, landed in BREP-09
    /// Plane ∩ shape → 3D section curves (S03 SK-08/SK-09, S07 tests, S12 DXF).
    fn plane_section(&self, s: &Shape, plane: &Plane) -> KResult<Vec<geom::CurveSeg>>;
    /// Edges orthogonally projected onto `plane` (S03 SK-09 "project").
    fn project_edges(&self, s: &Shape, edges: &[EdgeId], plane: &Plane) -> KResult<Vec<geom::CurveSeg>>;
    // -- tessellation (BREP-08) --
    fn tessellate(&self, s: &Shape, q: &TessQuality, ctx: &JobCtx) -> KResult<TessMesh>;
    fn tessellate_many(&self, shapes: &[Shape], q: &TessQuality, ctx: &JobCtx) -> Vec<KResult<TessMesh>>;
    fn tessellate_edge(&self, s: &Shape, e: EdgeId, q: &TessQuality) -> KResult<Vec<Point3>>;
}

// ===== OCCT backend ========================================================================
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum Validate { Off, #[default] DebugOnly, Always }
#[derive(Clone)]
pub struct KernelConfig {
    pub validate: Validate,
    pub occt_threads: Option<std::num::NonZeroUsize>,  // default max(1, cores/2)
    pub bool_parallel: bool,                            // default set by BREP-04 result
    pub tolerance: Tolerance,
}
impl Default for KernelConfig { fn default() -> Self; }
impl KernelConfig { pub fn from_env() -> Self; }       // AMG_KERNEL_VALIDATE, AMG_OCCT_THREADS
/// Cheap to clone; Send + Sync. Without cargo feature `occt` every op returns BackendUnavailable.
#[derive(Clone)]
pub struct OcctKernel { /* Arc<KernelConfig> */ }
impl OcctKernel {
    pub fn new(cfg: KernelConfig) -> OcctKernel;
    pub fn config(&self) -> &KernelConfig;
}
impl BrepKernel for OcctKernel { /* … */ }

// ===== STEP / IGES (BREP-13) ===============================================================
#[derive(Clone, Debug)]
pub struct ForeignModel { pub roots: Vec<ForeignNode>, pub source_unit_mm: f64, pub warnings: Vec<String> }
#[derive(Clone, Debug)]
pub struct ForeignNode {
    pub name: Option<String>,
    pub transform: Transform,          // relative to parent
    pub color: Option<[f32; 4]>,       // linear RGBA from XCAF
    pub shape: Option<Shape>,          // leaf part (solid / shell / compound)
    pub face_colors: Vec<(FaceId, [f32; 4])>,
    pub children: Vec<ForeignNode>,
    pub instance_of: Option<u32>,      // index into the shared-prototype table (assemblies reuse parts)
}
#[derive(Clone, Debug, PartialEq)]
pub struct ImportOpts { pub heal: bool, pub read_colors: bool, pub read_names: bool }
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum StepSchema { Ap203, Ap214, Ap242 }
pub mod io {
    use super::*;
    pub fn read_step(path: &std::path::Path, opts: &ImportOpts, ctx: &JobCtx) -> KResult<ForeignModel>;
    pub fn write_step(path: &std::path::Path, model: &ForeignModel, schema: StepSchema, ctx: &JobCtx) -> KResult<()>;
    pub fn read_iges(path: &std::path::Path, opts: &ImportOpts, ctx: &JobCtx) -> KResult<ForeignModel>;  // global lock
    pub fn write_iges(path: &std::path::Path, model: &ForeignModel, ctx: &JobCtx) -> KResult<()>;       // global lock
}

// ===== errors ==============================================================================
#[derive(Clone, Debug, thiserror::Error)]
#[non_exhaustive]
pub enum KernelError {
    #[error("geometry kernel is not available in this build")] BackendUnavailable,
    #[error("invalid input `{what}`: {reason}")]
    InvalidInput { what: &'static str, reason: String, offending: Vec<SubShape> },
    #[error("profile is not closed (loop {loop_index}, edge {edge_index}, gap {gap:.3e} mm)")]
    OpenLoop { loop_index: u32, edge_index: u32, gap: f64 },
    #[error("points are not coplanar (max deviation {max_dev:.3e} mm)")] NotPlanar { max_dev: f64 },
    #[error("transform is not rigid")] NonRigid,
    #[error("{op} failed: {message}")]
    OpFailed { op: &'static str, message: String, offending: Vec<SubShape> },
    #[error("{op} failed on {} edge(s)", offending.len())]
    LocalOpFailed { op: &'static str, message: String, offending: Vec<SubShape>, partial: Option<Shape> },
    #[error("{op} produced an invalid shape")] InvalidResult { op: &'static str, report: CheckReport },
    #[error("OCCT exception {kind}: {message}")] Occt { kind: String, message: String },
    #[error("I/O: {0}")] Io(String),
    #[error("unsupported file content: {0}")] Format(String),
    #[error("operation cancelled")] Cancelled,
}
impl From<amg_core::Cancelled> for KernelError { fn from(_: amg_core::Cancelled) -> Self { KernelError::Cancelled } }
impl amg_core::Diagnostic for KernelError { /* hints per variant; is_cancelled */ }
```

**For other specs:**
- **S01**: `Shape` (`Send + Sync`, cheap `Clone`, `estimated_bytes`, `write_bin`/`read_bin`).
  - `ShapeHistory` (serde), `SubShape`/`FaceId`/…, `sub_geom_all` (fingerprints),
    `geom::GeomClass`, `kernel_version()`.
  - `BodyNaming` index i = `FaceId(i)` etc.
- **S03**: `geom::Curve2d`/`PlanarProfile` mirror `Curve2`/`Profile`.
  - `plane_section`/`project_edges` for SK-08/SK-09.
- **S04**: the whole trait. Tags in `ProfileEdge2` → `FaceBuild::edge_tags` → lateral-face
  history.
- **S07**: `make_planar_polygon` (their `make_planar_face`), `make_face(Surface,
  FaceBounds::Polylines | Loops)` (their `make_face_on_surface`), `sew`,
  `TessMesh::to_indexed_parts`.
- **S12**: `io::*`, `ForeignModel`, `TessQuality`, `tessellate`.
- **S20**:
  - serde-able op parameter types (for its `KernelOp`);
  - a forwardable trait (for the `RecordingKernel`/`robust` decorators);
  - `fillet_max_radius`, `FilletOpts`, `Shape::max_tolerance`,
    `CheckOpts::self_interference`, `ShapeHistory::compose`;
  - raw `thru_sections`/`pipe`, `heal`, `unify_same_domain`, `check`;
  - `geom::BSplineSurfaceData` + `make_face` + `sew` for the loft builder.
- **S05**: `mass_props`, `distance`, `closest_point`, `eval_surface` (curvature),
  `interference`.

## Work packages
All lane A, gated by ADR-002 (cleared 2026-10-20). Each op WP's Done-when includes:
- its history test (after BREP-05);
- its op in the BREP-12 concurrency stress test;
- BRepCheck validity of every result.

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| BREP-01 | **Phase-0 spike.** OCCT 8.0.1 (8.0.0#1 fallback; QUESTIONS #27) via vcpkg manifest mode (`vcpkg.json` with `opencascade` `default-features: false`, overlay triplet `x64-windows-amg`), `amg-occt-sys` build.rs (`AMG_OCCT_DIR`, toolkit list, cxx_build flags), cxx bridge skeleton with custom `trycatch`, `OcShape` opaque type, `make_box` + `volume` + a throwing call; cargo feature `occt`; CI job uses the vcpkg `files` cache | FND-02, FND-05, FND-13 | P0 | Done | 1. On the owner's machine and in CI (`windows-2025`), `cargo nextest run -p amg-occt-sys` passes. A Rust test creates a 10×20×30 box and reads volume 6000 ± 1e-9. 2. A shim call that triggers `Standard_Failure` (`gp_Dir` from a zero vector) returns `Err` whose message contains the OCCT exception type, and no process abort. 3. `kernel_version()` returns `occt-8.0.0;…`, checked against `OCC_VERSION_COMPLETE` at compile time. 4. The Refinement log records: cold OCCT vcpkg build time, cached CI restore time, link time and test-binary size. The second CI run restores OCCT from cache. 5. `cargo build -p amg-brep --no-default-features` works on a machine without vcpkg. 6. If 8.0.0 is rejected, the reason and the 7.9.3 override are recorded (note §1.3). |
| BREP-02 | `Shape` handle (immutable, `Send + Sync`, `Arc`), `SubShapeIndex` (0-based `TopExp::MapShapes`, adjacency), topology iteration, `surface()`/`curve()`/`vertex_point()`/`bbox()`/`tolerance()` to plain `geom` types, rigid `transformed`, binary + text `.brep` I/O | BREP-01, FND-03 | P0 | Done | 1. For 10 fixture parts (`testdata/brep/*.brep`, ≥ 1 with > 500 faces): face/edge/vertex counts and per-face `GeomClass` match a checked-in expectation file. 2. Index stability: indices + per-face area computed in two separate processes and after a `write_bin`/`read_bin` round-trip are identical. 3. `write_bin`→`read_bin` is byte-identical on re-write. 4. `assert_impl_all!(Shape: Send, Sync)` compiles. 5. Adjacency invariants hold on all fixtures (every manifold edge has 2 faces, `face_edges`/`edge_faces` are mutually consistent). 6. A cylinder face reports `Surface::Cylinder` with the correct radius/axis within 1e-12. |
| BREP-12 | Kernel infrastructure: `KernelConfig`, `init()` (OCCT message printer → `tracing`, `OSD_ThreadPool` size), progress/cancel bridge (`Message_ProgressIndicator` ↔ `JobCtx`), validation policy, `KernelError` + `Diagnostic`, `Shape::max_tolerance`, shim CATEGORY (R/N/C/G) lint test, concurrency stress-test harness, global I/O lock, serde-derive check for every op parameter type | BREP-02, FND-04 | P0 | Done | 1. An OCCT warning appears as a `tracing` event with target `occt`, and nothing goes to stdout. 2. A test op with a progress range reports monotone progress to a `JobCtx`. Cancelling the token makes it return `KernelError::Cancelled` within 50 ms. 3. A compile-time test asserts that every op parameter type is `Clone + Debug + PartialEq + Serialize + DeserializeOwned` (S20 KH-01 prerequisite), and a forwarding decorator implementing `BrepKernel` compiles (object safety). 4. The CATEGORY lint fails when a shim function lacks a category comment. 5. Stress harness: 8 threads × 2 000 iterations of R queries on one shared shape. No crash, results are bit-identical to single-threaded ones, and the input's BinTools hash is unchanged. |
| BREP-03 | Face and wire builders: `make_planar_faces` (from `PlanarProfile` with tags + loop validation), `make_planar_polygon`, `make_face` (analytic + B-spline surfaces; `Natural`/`UvRect`/`Loops`/`Polylines` bounds), `make_wire` | BREP-12 | P0 | Done | 1. A profile with all 5 `Curve2d` kinds (line, circle hole, arc, ellipse arc, rational B-spline) gives a valid face whose area matches the analytic area within 1e-9 relative. Every edge has its input tag. 2. An open loop gives `OpenLoop` with the right loop/edge index. A hole outside the outer loop gives `InvalidInput`. 3. A 1000-vertex polygon with 10 holes builds in < 20 ms. A non-planar input gives `NotPlanar`. 4. `make_face` on cylinder/cone/sphere/torus with `Polylines` bounds taken from a tessellated reference face reproduces the face area within 0.1 %. A 5×5 B-spline patch with `Natural` bounds is valid. 5. 1000 random valid profiles (proptest) never crash and always give valid faces or a clean `Err`. |
| BREP-14 | Primitives (box, cylinder, cone, sphere, torus) on a `Plane` frame; `prism` (vector, infinite, taper, `merge_coplanar`); `revol`; `gtransformed` | BREP-03 | P0 | Done | 1. Golden volume/area/topology counts for each primitive, for prism of 5 profile types and for revol (90°, 360°) within 1e-9 relative (`tests/golden/brep/`). 2. A tapered prism of a square at 5° matches the analytic frustum volume within 1e-9. 3. Results are BRepCheck-valid. 4. Prism and revol run in the stress test. |
| BREP-04 | Booleans: `boolean` (fuse/cut/common, N objects × M tools), `split`, `section`; non-destructive always on; fuzzy/glue/OBB/check-inverted; progress + cancel; parallel-mode determinism test and `occt_threads` benchmark | BREP-14 | P0 | Done | 1. A 100-case boolean corpus (`testdata/brep/bool/`: primitives, coplanar, tangent, fuzzy-needed cases from OCCT/FreeCAD trackers) gives valid results, with volumes within 1e-6 relative of reference values (or the expected `Err`). 2. Proptest: 500 random primitive pairs give `Ok(valid)` or a clean `Err`, never a crash. 3. Inputs' BinTools hashes are unchanged after every op. 4. Determinism: 20 cases × 10 runs with parallel on are byte-identical, or parallel defaults off and the Refinement log says so. 5. Criterion: a cut of two 1000-face parts, timed with 1 / half / all OCCT threads; the default is set from the result. 6. Cancelling a long boolean returns `Cancelled` within 200 ms. |
| BREP-05 | History framework: `ShapeHistory` (entries, `Kept`, roles, tags, `sources_of`, `compose`), extraction from `BRepBuilderAPI_MakeShape`/`BRepTools_History`/op accessors, composition of internal steps, deterministic split order, `sub_geom`/`sub_geom_all` fingerprints; applied to BREP-03/14/04 ops | BREP-04 | P0 | Done | 1. Completeness test: for every op in the BREP-03/14/04 corpora, every output face is Modified/Generated/Kept from some input or has a role, and no output index is out of range. 2. Prism of a tagged profile: each lateral face's `sources_of` is the generating edge, which carries the profile tag; caps have `StartCap`/`EndCap`. 3. Box cut by a cylinder through one face: the split face has 2 `Modified` outputs, in area-desc order, stable across 10 runs. 4. `compose` of prism + unify equals the history observed end-to-end on 5 cases. 5. `ShapeHistory` serde round-trips. 6. `sub_geom_all` on a 1000-face part takes < 20 ms. Usable by DOC-06 tests (box, prism, boolean cut). |
| BREP-06 | Fillet (constant, variable, chord) and chamfer (equal, two-dist, dist-angle) with history (`FilletBlend`, `ChamferFace`) and `LocalOpFailed { offending }`; `fillet_max_radius` | BREP-05 | P0 | Done | 1. Golden volumes for 20 fillet/chamfer cases (cube edge, edge chains, vertex blends of 3 edges) within 1e-6 relative. 2. A radius larger than the adjacent face fails with `offending` naming the failing edge(s), checked against the expected edge ids. 3. `fillet_max_radius` on 5 cases is within 1 % of the reference largest workable radius. 4. History: each blend face maps to its source edge. 5. 10 runs are byte-identical. 6. The op is in the stress test. |
| BREP-07 | Shell / thick solid (removed faces, per-face thickness, join arc/intersection), `offset`, `thicken` with roles `ShellInner`/`OffsetFace` | BREP-05 | P0 | Done | 1. Golden volumes: shelled box (1 face removed, 1 mm) equals the analytic volume within 1e-9 relative. Shelled cylinder, filleted box (arc join) and thickened B-spline face (0.5 %) pass. 2. Inward/outward offset of a sphere: radius within 1e-9. 3. A thickness exceeding the minimum radius gives `LocalOpFailed` with offending faces. 4. History roles are correct on 5 cases. |
| BREP-16 | Draft, defeature (delete face), replace face, split faces by tool, split body (via `split`) with history | BREP-05 | P0 | Review | 1. Draft of 4 box side faces at 3° about the bottom plane: the measured face angles are 3° ± 1e-9 rad and the volume is analytic. 2. Deleting a fillet face and a boss face gives the pre-feature volume within 1e-6. 3. Replacing a top face with an offset plane changes the height exactly. 4. Splitting a face by a sketch wire gives 2 faces with the right areas. 5. Every result is BRepCheck-valid with complete history. |
| BREP-08 | Tessellation: `tessellate` on a private topology copy (category C), `TessMesh` (per-face vertices, normals, `face_ranges`, exact `weld`, edge polylines), `to_indexed_parts`, `tessellate_many` (rayon, `InParallel` off), `tessellate_edge`, quality presets | BREP-12 | P0 | Done (owner-accepted 2026-11-19; Done-when 1 moved to BREP-18) | 1. A 1000-face fixture part tessellates (display preset) in < 100 ms median, including the topology copy (criterion `budget/brep-tess-1000`); the copy share is recorded. 2. `to_indexed_parts` of every closed fixture is watertight (`amg_testkit::metrics`: 0 boundary and 0 non-manifold edges), and its volume matches `mass_props` within the deflection bound. 3. Sphere r=10 tessellations converge to the exact area as deflection shrinks. 4. Tessellating one shared `Shape` from 8 threads while another thread runs booleans on it: no crash, identical meshes, input hash unchanged. 5. `face_of_triangle` is consistent with `face_ranges` for all triangles. |
| BREP-09 | Mass properties (volume, area, centroid, inertia, principal axes), `classify`, `distance`, `closest_point`, `ray_cast`, `eval_surface` (curvatures), `project_point_on_face`, `interference` | BREP-04 | P0 | Done | 1. Box, cylinder, sphere and torus mass props match analytic values within 1e-9 relative. Inertia tensors match within 1e-9. 2. Distance between two spheres and between a point and a face matches analytic values within 1e-9. 3. `ray_cast` hits the expected face with the correct normal on 1000 random rays at a cylinder (vs analytic). 4. Curvatures of a torus at 10 points match analytic values within 1e-9. 5. `interference` volume of two overlapping boxes is analytic. |
| BREP-17 | Geometry extraction for sketches and drawings: `plane_section` and `project_edges` to `CurveSeg` (lines/circles/ellipses recognised, others B-spline), `tessellate_edge` reuse | BREP-12 | P0 | Done | 1. A plane section of a cylinder (perpendicular) gives one `Curve3::Circle` with exact radius. An oblique section gives an `Ellipse` with analytic axes within 1e-9. 2. Projecting the edges of a filleted box top onto the XY plane gives lines and circle arcs with the correct radii and endpoints. 3. Section of a B-spline surface returns B-spline segments within 1e-6 mm of the true intersection (sampled). |
| BREP-10 | Healing and assembly: `heal` (`ShapeFix_Shape`, `ShapeFix_Wireframe` on a copy), `unify_same_domain`, `sew` (+ `make_solid`, orientation fix), `make_compound`, `check` (BRepCheck + ArgumentAnalyzer) | BREP-05 | P0 | Done | 1. Sewing the 6 planar polygon faces of a cube (with 1e-4 mm gaps) gives a valid closed solid with volume within 1e-6. 2. Sewing the faces of 20 fixture parts exploded via `sub_shape` gives back solids with the same volume and face count. 3. `heal` fixes 10 corpus shapes with known defects (BRepCheck invalid → valid). 4. `unify_same_domain` after a fuse of two coplanar boxes yields 6 faces, and history maps the merged faces. 5. `check` lists the expected problem kinds on 5 broken fixtures. |
| BREP-13 | STEP (AP203/214/242) and IGES read/write via XCAF into `ForeignModel` (names, colours incl. per-face, assembly tree with shared prototypes, units → mm), heal on import, progress + cancel; IGES under the global lock | BREP-10 | P0 | Todo | 1. Round-trip of 10 STEP fixtures (≥ 1 assembly with reused parts, ≥ 1 coloured, ≥ 1 in inches): write then read again gives the same tree, names, colours, solid count, and per-solid volume within 1e-9 relative. Inch input is scaled to mm. 2. Reading the same STEP file on 4 threads in parallel gives identical results (on 8.0.0 the reads serialise through the global lock; after a bump to 8.0.1 the lock is removed and the test still passes). 3. IGES round-trip of 3 fixtures. 4. A 50 MB STEP read reports progress and cancels within 500 ms. 5. The owner's Fusion STEP exports (if present under `$AMG_TESTDATA_DIR/fusion/`) import as valid solids. |
| BREP-15 | Sweeps and raw loft: `pipe` (pipe-shell: Frenet/corrected/fixed/binormal/guide, twist/scale laws, transition, solid/shell), `thru_sections` (ruled/smooth, point sections, compatibility check) with roles `LoftSection`/`Lateral` | BREP-05 | P0 | Review | 1. Golden volumes: a circle swept along a straight line and along an arc (torus segment) are analytic within 1e-6. A square along a helix is within 0.1 %. 2. A twisted sweep (90°) and a scaled sweep (1→2) match reference volumes. 3. ThruSections of 3 circles of different radii (smooth) is within 1e-4 relative of the analytic volume. Point-section cones work. 4. History: lateral faces map to the section edges. 5. Failures (incompatible sections) return `OpFailed` and never crash. KH-05 can call it as the baseline. |
| BREP-11 | Thread geometry: `helix`, thread profile swept in binormal mode, cut/fuse onto a cylinder (`amg-brep` helper used by SOL-12) | BREP-15, BREP-04 | P1 | Todo | 1. ISO metric M3, M6, M10 and M20 external and internal threads (coarse pitch) build valid solids. 2. Pitch, measured as the axial distance between crests, is within 1e-6 mm. 3. The major/minor diameters measured by sections are within 1e-4 mm of ISO 68-1. 4. An M6×20 thread builds in < 200 ms. |
| BREP-18 | Planar-face mesher for `tessellate`: planes with many hole loops bypass OCCT's `BRepMesh_Delaun` (a custom `IMeshTools_MeshAlgo` for planar faces over a real constrained Delaunay triangulation, or a Rust CDT fed from OCCT's edge discretisation), keeping the exact weld through `Poly_PolygonOnTriangulation` | BREP-08 | P0 | Todo | 1. The 1030-face `plate_with_holes` meets the BREP-08 display budget: `budget/brep-tess-1000` < 100 ms median, topology copy included. 2. All BREP-08 tests stay green (watertight `to_indexed_parts`, volume bound, layout, determinism, the Done-when 4 stress test) and the `tests/golden/brep/` goldens pass or are re-blessed with the reason recorded. 3. The chosen CDT passes the ADR-012 licence check and has a `THIRD_PARTY.md` row. |

**Dependency changes in this refinement:** BREP-01 +FND-05 (+FND-13 on 2026-10-20, QUESTIONS #27). BREP-02 +FND-03.
BREP-03 BREP-02 → BREP-12. BREP-04 BREP-02 → BREP-14. BREP-05 BREP-03 → BREP-04.
BREP-06/07 BREP-02 → BREP-05. BREP-08 BREP-02 → BREP-12. BREP-09 BREP-02 → BREP-04.
BREP-10 BREP-02 → BREP-05. BREP-11 BREP-03 → BREP-15 + BREP-04.

**New WPs:**
- **BREP-12** (infrastructure), split out of the draft's BREP-01/BREP-02 scope.
- **BREP-13** (STEP/IGES), split from BREP-10.
- **BREP-14** (primitives, prism, revol) and **BREP-15** (pipe, loft), split from BREP-03.
  BREP-03 keeps the face builders.
- **BREP-16** (draft, replace, split, defeature), split from BREP-07.
- **BREP-17** (section/projection), new for S03 SK-09.

## Testing
- **Golden geometry** (`tests/golden/brep/`, `amg_testkit::assert_golden!` on
  `TessMesh::as_soup()`): volume, area, bbox and topology counts per op case. Exact
  `mass_props` values are compared directly at 1e-9 relative.
- **Validity**: BRepCheck on every op result in tests (`Validate::Always` in the test
  `KernelConfig`).
- **Fuzz/proptest**: random primitives × booleans (BREP-04), random profiles (BREP-03),
  random fillet radii (BREP-06). Each must give `Ok(valid)` or a clean `Err`, never an abort.
- **Concurrency stress test** (BREP-12 harness, extended by every op WP): shared inputs never
  change (BinTools hash) and results are bit-identical. On Linux CI (best effort) it also runs
  under ThreadSanitizer.
- **Determinism**: 10× byte-identical BinTools output for every corpus op (BREP-04, BREP-06).
- **Index stability**: across processes and `.brep` round-trips (BREP-02).
- **Fixtures**: `testdata/brep/` (LFS; generated from scripts in `crates/amg-brep/tests/gen/`
  where possible). Boolean corpus cases are harvested from OCCT/FreeCAD trackers, with licence
  notes in each subfolder. Large/private STEP files live under `$AMG_TESTDATA_DIR/brep/`.

## Performance targets
| Target | Budget | WP |
|---|---|---|
| Tessellate 1000-face part, display preset | < 100 ms | BREP-08 |
| FFI overhead per op (box, no work) | < 20 µs | BREP-12 |
| `sub_geom_all`, 1000 faces | < 20 ms | BREP-05 |
| Prism of a 100-edge profile | < 10 ms | BREP-14 |
| Typical boolean (two 200-face parts) | < 50 ms | BREP-04 |
| Planar faces for a 50-region sketch | < 10 ms | BREP-03 |
| `write_bin` + `read_bin` of a 1000-face part | < 30 ms | BREP-02 |

These budgets back the vision's "< 500 ms recompute of a 50-feature part" (an average of
≤ 10 ms of kernel time per feature) and the "< 16 ms UI" rule: all kernel calls run on
`amg-jobs`.

## Open questions
1. ~~OCCT version pin and vcpkg triplet.~~ **Resolved 2026-10-20:** OCCT 8.0.0 (vcpkg
   8.0.0#1 at baseline `2026.07.29`) with 7.9.3#1 as the documented fallback. The triplet stays
   S00's `x64-windows-amg` (static, release, `/MD`). See §1 and the note §1–2.
2. ~~Expose BOPAlgo parallel mode by default?~~ **Resolved 2026-10-20 (rule):** on by default
   if BREP-04's determinism test passes, otherwise off. The thread count is set from BREP-04's
   benchmark (§7).
3. Bump the vcpkg baseline to one that ships OCCT 8.0.1 (fixes a `strtod` thread race and
   some ThruSections/boolean crashes)? This is an S00 pin change, so it goes to the owner
   through the report. S02 works on 8.0.0 either way (§1).
   **Resolved 2026-10-20 (owner, QUESTIONS #27):** the bump is not done now. S00 gets the new
   WP FND-13, scheduled before BREP-01, and BREP-01 depends on it, so BREP-01 pins 8.0.1
   (8.0.0#1 and 7.9.3#1 stay the documented fallbacks). The 8.0.0-only workarounds (global
   lock for STEP and text `.brep` reads, no OCCT Gordon path) become needed only if FND-13
   slips past BREP-01.
4. Is 8.0.0 stable enough? Decided by BREP-01 against the note §1.3 fallback rule; no owner
   input needed.

## Refinement log
**2026-10-20 — refinement (lane A agent).**
- **Researched:**
  - vcpkg `opencascade` port: the manifest and portfile on master; the versions DB (8.0.1#0,
    8.0.0#1, 8.0.0#0, 7.9.3#1, …). The tag `2026.07.29` has 8.0.0#1. The portfile maps
    static linkage → `BUILD_LIBRARY_TYPE=Static` and patches `OCCT_STATIC_BUILD`. It sets no
    CRT, Draw is off, and freetype is the only default feature.
  - The OCCT 8.0.0 release (2026-05-07): `Standard_Failure : std::exception`, `std::mutex`,
    thread-local error handler, thread-safe STEP per instance, IGES not thread-safe,
    `BRepGraph`, C++17, removed packages.
  - The 8.0.0p1 notes: `Standard_Strtod` thread race fix, ThruSections/BOP fixes.
  - The OCCT forum and tracker on the thread safety of meshing and booleans (BRepMesh writes
    the triangulation into TShapes; #22850; `SetNonDestructive`).
  - The cxx `Result` / `rust::behavior::trycatch` customisation. autocxx's own guidance. The
    `opencascade-rs` approach (cxx + `wrapper.hxx`).
  - Cross-spec needs: S00 (triplet, layering, FND-11), S01 (`Shape: Sync`, history,
    fingerprints, `kernel_version`), S03 (planar face builder from `Curve2` loops), S07
    (polygon/surface faces, sew, tessellation), S20 (failure capture, raw loft/pipe,
    strategies), S12, S05, S09.
- **Decided:**
  - OCCT 8.0.0, with 7.9.3 as fallback.
  - The triplet is kept. Port `default-features: false`, no TBB.
  - A cxx bridge over a coarse shim. Custom `trycatch`. No SEH translation.
  - **`Shape` is immutable and `Send + Sync`**: R/N/C/G categories, tessellation on a private
    topology copy, booleans non-destructive. This proves ADR-014's requirement holds, so no
    ADR amendment is needed, only an architecture text change (report).
  - Plain-data `geom` types mirroring S03 `Curve2` and S01 `GeomClass`, because of the
    layering rule (`amg-brep` can't see `amg-sketch`/`amg-doc`). So S02 does **not** need
    shared analytic curves in `amg-core` (answers S00 open question 3 for S02).
  - `ShapeHistory` with `Kept`, roles and tags, and a deterministic split order.
  - `TessMesh` with per-face vertices, an exact weld map and `to_indexed_parts` matching
    `IndexedMesh`'s core fields.
  - The decorator pattern for S20 strategies. Failure capture (`KernelOp`, dumps, replay) is
    S20's, aligned with the S20 refinement's `docs/notes/kernel-robustness.md`. S02 covers
    its requests R3 (serde-able params), R5 (self-interference check), R7 (`FilletOpts`) and
    R8 (`ShapeHistory::compose`), plus `Shape::max_tolerance`.
  - The `occt` cargo feature.
- **Changed:**
  - Status Draft → Refined.
  - Scope, Design, Interfaces, Testing and Performance targets were rewritten.
  - WPs re-split (see the notes under the WP table). New: BREP-12..17.
  - Every WP has deps and a measurable Done-when.
- **Design note:** [docs/notes/occt-integration.md](../notes/occt-integration.md) (new).

### 2026-11-09 - BREP-01 OCCT build/link spike
**Landed.** `vcpkg.json` gains `opencascade` (`default-features: false`). `amg-occt-sys` has the
cxx bridge (`src/bridge.rs`), the C++ shim (`cpp/occt_shim.{hxx,cpp}`) with the custom
`rust::behavior::trycatch`, the opaque `OcShape`, `make_box`, `volume`, `kernel_version` and the
throwing `gp_dir_from_zero_vector`. `build.rs` reads `AMG_OCCT_DIR` and links the 27 toolkits from
note §2, all present in the 8.0.1 install under those names. CI has a new `occt` job.

**Measured** (Windows 11, 16 logical CPUs, 32 GB, `x64-windows-amg`):

| What | Result |
|---|---|
| Cold vcpkg build of OCCT 8.0.1, `VCPKG_MAX_CONCURRENCY=8` | 16 701 s (4 h 38 min) wall |
| Of which the serial `install` step (ninja log) | 8 557 s (2 h 23 min) |
| Compile/link work in the ninja log | 15.3 CPU-hours including `install`, average parallelism 3.7 |
| Local restore of all 9 ports from the `files` cache into a fresh install root | 40 min (vcpkg reports 38 min) |
| Installed OCCT and dependencies | `lib/` 2.1 GB (includes Manifold, TBB), `include/` 36 MB |
| First `--features occt` build of the tests, 3 cargo jobs | 18 min 59 s |
| Test-binary rebuild after touching the test file (compile + full link, 2 runs) | 13 s, 14 s |
| `occt_smoke` test binary | 4.17 MB exe, 31.1 MB PDB |

`static:-bundle` keeps the OCCT libraries out of the `amg-occt-sys` rlib. The cold build shared the
machine with a 5 min 43 s cargo build.

**Done-when status.** 1: local part passes (box volume 6000 ± 1e-9); CI not run yet. 2: `gp_Dir`
from a zero vector returns `Err` containing `Standard_ConstructionError`, no abort. 3:
`amg_occt_sys::kernel_version()` returns `occt-8.0.1;shim-1`; a `static_assert` compares
`OCC_VERSION_COMPLETE` with the pin and was confirmed to fail the build on a wrong pin. 4: local
numbers above; cached CI restore time and the second-run cache hit are still to be taken from CI.
5: `cargo build -p amg-brep --no-default-features` passes without vcpkg. 6: not applicable, 8.0.1
is kept and the spike hit no regression.

**Deviations.**
- `occt` is **off by default** in `amg-brep` and `amg-occt-sys`, against §Scope's "default on", so
  the workspace default build keeps working without vcpkg (`amg-mesh/manifold` pattern; agreed with
  the coordinator). Turning it on later is a one-line change.
- The version is 8.0.1 per FND-13, not the `occt-8.0.0` that Done-when 3 still names.
- `kernel_version()` lives in `amg-occt-sys` for now. `amg_brep::kernel_version` needs
  `FORMAT_VERSION` and `POLICY_VERSION`, which don't exist yet.

**Leftovers.**
- **CI cold-build risk.** A cache miss builds OCCT in each of the `clippy`, `test`, `occt` and
  `bench` jobs. Timeouts are raised to 350 min, near GitHub's 360 min limit, but at 15.3 CPU-hours
  a 4-vCPU runner may not finish a cold build within it. If the first run times out, the binary
  cache has to be seeded another way before CI can go green.
- The system library list (`advapi32 user32 ws2_32`) is untrimmed; these tests reference only a
  small part of OCCT, so later WPs may need additions.
- `unsafe impl Send + Sync for OcShape` is left to BREP-02, and OCCT logging and progress to
  BREP-12. `docs/dev-setup.md` does not mention `AMG_OCCT_DIR` yet.

### 2026-11-09 - BREP-01 merge (coordinator)
- `opencascade` moved from the top-level `vcpkg.json` dependencies into a manifest feature `occt`
  (`vcpkg install --x-feature=occt`). Otherwise every CI job and every local `vcpkg install`
  for Manifold would also build OCCT (about 15 CPU-hours cold). The vcpkg action takes a
  `features` input that is part of the binary-cache key; only the `occt` job passes `occt`.
- The clippy, test and bench job timeouts are back to 45/45/60 min. Only the `occt` job keeps
  350 min, for a cold build on a cache miss.
- The workspace all-features clippy (CI and local gate) excludes `amg-occt-sys` and `amg-brep`;
  the `occt` job lints them with `--all-features`.

### 2026-11-10 - BREP-02 Shape handle, sub-shape index, queries, `.brep` I/O
**Landed.** `amg-brep` has `Shape` (`Arc` around an `occt`/no-backend `Occ`, lazy `OnceLock`
index and bbox), `SubShapeIndex` (plain-Rust compressed tables built from one `OcTopology` FFI
call), the `geom` types, a `KernelError` subset, rigid `transformed`, and `write_bin`/`read_bin`
(`AMGB` + `FORMAT_VERSION` 1 + BinTools v4, no triangles) plus `write_text`/`read_text` (BRepTools
v3, no triangles, classic locale). `amg-occt-sys` gains the query/IO shim functions (each tagged
`// CATEGORY:`), `OcIndex`, `unsafe impl Send + Sync` for `OcShape`/`OcIndex` with SAFETY notes,
and a `fixture-gen` feature with 11 C++ fixture builders. `testdata/brep/` holds 11 text fixtures
(largest `plate_with_holes`, 1030 faces, 823 KB) and `expected.json`, regenerated by
`cargo run -p amg-brep --features fixture-gen --example gen_brep_fixtures`. The generator checks
counts and class histograms against values derived by hand from each construction.

**Done-when status** (`cargo nextest run -p amg-occt-sys -p amg-brep --features occt`, 46 tests).
1: counts and per-face classes match `expected.json` for all 11 parts. 2: index + `face_area`
fingerprints (exact bit patterns) match in two child processes (one with allocator churn) and
after a `write_bin`/`read_bin` round trip. 3: re-write is byte-identical for all 11 fixtures. 4:
`static_assertions::assert_impl_all!(Shape: Send, Sync)` in `shape.rs`. 5: adjacency invariants
hold on all fixtures; one-face edges are only seams, degenerated edges or open-shell boundaries.
6: the cylinder fixture reports radius 5 and axis +Z through (3, 1) within 1e-12.

**Budget** (`cargo bench -p amg-brep --features occt --bench brep_io`, plate, 1030 faces):
`budget/brep-bin-roundtrip-1000` 27.7 ms (write 12.4 ms, read 13.6 ms) for a shape that came from
`read_bin`. The first `write_bin` of any other shape takes 37.2 ms (see below).

**Findings and deviations.**
- **BinTools is not a fixed point on reflected frames.** Its reader rebuilds each `gp_Ax3` Y
  direction as `±(N × X)`, so a left-handed surface frame (any reflection) loses `-0.0` signs on
  its first read and re-writes different bytes. `write_bin` therefore writes, re-reads and
  writes again unless the shape came from `read_bin` (or shares its TShapes via `transformed`,
  `solids`, `sub_shape`). Cost: one extra round trip on a shape's first save.
- Frames are reported right-handed (`normal = x × y`); for a reflected surface the normal is
  opposite to OCCT's axis and `v` runs against it.
- An identity `transformed` returns a shape `IsSame` to the input (no identity `TopLoc_Location`).
- Filleted-box corners are spherical patches with a degenerated pole edge: 26/56/24, not the
  draft's 26/48/24.
- 11 fixtures rather than 10 (`cylinder_mirrored` covers the reflection case). `.brep` is not in
  the LFS patterns, so the fixtures are plain files (all < 1 MB).
- No debug generation stamp on ids: `FaceId(pub u32)` is a transparent type; ids are range-checked
  and panic when out of range.
- Adjacency is built by one traversal of faces → wires (`BRepTools::OuterWire` first,
  `BRepTools_WireExplorer` order with stored order as fallback) rather than
  `MapShapesAndAncestors`; `edge_faces` is its inverse in Rust.
- `read_text` takes no global lock on 8.0.1 (the `Strtod` race is fixed there, §9).

**Leftovers.** `KernelError` is the subset BREP-02 needs (BREP-12 completes it and adds
`Diagnostic`); `max_tolerance` (BREP-12), `gtransformed` (BREP-14) and
`amg_brep::kernel_version` are not done. Skipping the extra BinTools round trip for shapes with
no reflected frames would bring a first save to the steady-state cost.

### 2026-11-10 - BREP-12 kernel infrastructure
**Landed.**
- `amg-occt-sys` (`SHIM_VERSION` 3): an `extern "Rust"` block (`ProgressSink` with
  `progress_show`/`progress_user_break`, and `occt_log`), and shim functions
  `install_message_printer`, `emit_message`, `set_default_thread_pool`,
  `default_thread_pool_size`, `progress_test_op`, `shape_max_tolerance`, `check_shape`,
  `make_open_wire_face`. The C++ side has a `Message_Printer` subclass that forwards every message
  (trace level lowest) to `occt_log`, which emits `tracing` events with target `occt`
  (`LOG_TARGET`), and a `Message_ProgressIndicator` subclass over a `ProgressSink`.
  `ProgressTarget` is the Rust trait behind the sink. Callback panics are caught, so they cannot
  unwind into C++.
- `amg-brep`: `KernelConfig` (`Default`, `from_env` with `AMG_KERNEL_VALIDATE` /
  `AMG_OCCT_THREADS`, `effective_occt_threads`), `Validate` (`is_enabled`, `parse`),
  idempotent `init` (first call wins, under the global lock), `kernel_version()`, `POLICY_VERSION`,
  the `BrepKernel` trait (object-safe; `check` is its first method), `OcctKernel` (`new`,
  `config`, `validate_result`, hidden `progress_probe`), `CheckOpts`/`CheckReport`, the full
  `KernelError` (all spec variants plus `offending()`), `From<Cancelled>`, `Diagnostic` (hint per
  variant, `Cancelled` is `Info` and `is_cancelled`), `Shape::max_tolerance`, the global I/O lock
  (`lock::global_io_lock`, poison-free), and every op parameter type from the Interfaces in a new
  `opts` module (`BoolOpts` … `HelixSpec`, `TessQuality` with the §8 presets).
- Tests: `amg-occt-sys/tests/category_lint.rs`, `amg-brep/tests/{api_contract, infra, init_pool,
  stress}.rs`, the reusable harness `tests/common/stress.rs`, and unit tests for config, errors,
  lock, validation and `max_tolerance`. Bench `benches/kernel.rs`.

**Done-when status** (`cargo nextest run -p amg-occt-sys -p amg-brep --features occt`, 78 tests,
all pass; without the feature the OCCT-free tests pass too).
1. `occt_messages_become_tracing_events`: messages of all five gravities arrive as `tracing`
   events with target `occt` (trace, info, warn, error, error). `occt_messages_never_reach_stdout`
   runs two child processes: without `init` the marker shows up on stdout (control), with `init`
   it does not.
2. `job_progress_is_monotone`: a 300-step probe on a `JobSystem` job gives 284 samples of
   `JobHandle::progress`, 281 of them strictly between 0 and 1, never decreasing.
   `bridge_reports_monotone_progress` checks the raw `Show` fractions: at least 500, monotone,
   ending at 1. `cancel_returns_cancelled_within_50_ms`: the probe returns
   `KernelError::Cancelled` **0.83 ms** after `cancel()` and the job ends `JobError::Cancelled`.
   The < 50 ms assertion only runs with `AMG_STRICT_TIMING=1`.
3. `api_contract.rs`: `assert_op_param::<T>()` (`Clone + Debug + PartialEq + Serialize +
   DeserializeOwned`) for every op parameter type that exists (ids, `SubShape`, `Plane`,
   `Tolerance`, the BREP-02 `geom` types, all `opts` types, `CheckOpts`), with serde round trips.
   A `Forward<K: BrepKernel>` decorator with no default methods implements the trait, stacks,
   erases to `Arc<dyn BrepKernel>`, moves across threads and forwards `check`.
   `assert_obj_safe!(BrepKernel)` is in `kernel.rs`.
4. `category_lint.rs` parses `occt_shim.hxx` and `occt_fixtures.hxx` (free functions in
   `namespace amg::occt`) and fails for a function without `// CATEGORY: R|N|C|G <reason>`
   directly above it, for an unknown category, and for a bridged `fn` that the header does not
   declare. Two synthetic headers check that it fails. The real headers pass (37 functions).
5. `stress.rs`: 8 threads × 2 000 iterations on one shared `plate_with_holes` (1 030 faces,
   27 633 distinct R queries: surfaces, curves, points, areas, uv bounds, adjacency, tolerances,
   `sub_shape`, `max_tolerance`, bbox, solids, `estimated_bytes`, `check`). The same runs on
   `filleted_box` (526 queries). Each thread starts with `check`; the shared copy builds its lazy
   index concurrently. There are 0 mismatches against the single-threaded reference, which
   compares `Debug` strings, so every f64 bit counts. The BinTools hash is unchanged
   (`42a46e2564f1eef4` before and after for the plate).

**Budget** (`cargo bench -p amg-brep --features occt --bench kernel`, release, on a loaded shared
machine): `budget/brep-ffi-overhead` (build and drop a 10×20×30 box through the bridge) takes
**34.1 µs**, over the 20 µs target. The bridge crossing itself (`ffi/shape-is-null`) takes
**7.6 ns**, so nearly all of the 34 µs is OCCT's `BRepPrimAPI_MakeBox` topology build, not the
FFI. `ffi/max-tolerance-box` takes 3.1 µs, and `ffi/progress-100-steps` 7.2 µs (72 ns per progress
step, including the Rust callbacks). Either the target means the call overhead (met by a wide
margin), or it needs re-basing for a real box build; BREP-14 should re-measure on an idle machine.

**Deviations.**
- The trait has only `check` for now. Every op WP adds its methods together with its `KernelOp`
  variant and the forwarding arm in `tests/api_contract.rs`. The decorator has no default methods,
  so a missing arm is a compile error. The op parameter types are already in, so the serde check
  covers them.
- The `geom` builder inputs (`Tag`, `Curve2d`, `PlanarProfile`, `CurveSeg`, `FaceBounds`) are
  left to BREP-03, as BREP-02 planned. BREP-03 adds them to `op_parameter_types_are_capturable`.
- `check` runs `BRepCheck_Analyzer` only. `CheckOpts::self_interference`
  (`BOPAlgo_ArgumentAnalyzer`) is ignored until BREP-10. A wire's problems are reported on its
  face (wires are not indexed); shell, solid and root problems have `None`.
- `KernelConfig::occt_threads` is `None` = auto, and `effective_occt_threads()` resolves it to
  `max(1, cores / 2)`. `bool_parallel` defaults to `false` until BREP-04 decides. `KernelConfig`
  also derives `Debug` and `PartialEq`.
- `kernel_version()` drops the shim part of `amg_occt_sys::kernel_version` (shim bumps don't
  change results): `occt-8.0.1;amg-brep-1;policy-0`, or `occt-none;…` without OCCT.
  `POLICY_VERSION` (0) lives in `kernel.rs` until S20's `robust` module takes it over.
- The global lock is used by `init` only. Text `.brep` reads stay unlocked on 8.0.1 (BREP-02).
  IGES (BREP-13) is its first real user.
- `OSD_ThreadPool::DefaultPool(n)` is resized with `Init(n)` if it already existed with another
  size. `init_pool.rs` checks that a pool of 3 is kept when a later `init` asks for 5.
- The stress test uses the 1 030-face plate and the 26-face filleted box rather than the note's
  "200-face filleted part" (there is no such fixture). It runs R queries only; tessellation,
  booleans and fillets join it in their WPs.

**Leftovers.**
- No real OCCT algorithm uses the progress bridge yet (`progress_sink` is ready in
  `backend/occt.rs`). BREP-04/08/13/15 pass it to their `Message_ProgressRange`.
- Architecture §Threading text change (S02 §2 consequence) is still not applied (not lane A's
  file).
- ThreadSanitizer CI run of the stress test (Linux, best effort) is not set up.


### 2026-11-12 - BREP-08 tessellation
**Landed.**
- `amg-occt-sys` (bridge additions; `SHIM_VERSION` unchanged at 3, since no existing function
  changed): shared structs `OcTessParams` and `OcTessMesh` (flat `f64`/`u32` arrays, timings,
  `weld_mismatches`, `cancelled`), and shim functions `tessellate_shape` (category **C**),
  `tessellate_edge` (R), `make_sphere` (R, test geometry) and `boolean_cut_probe` (N). They are
  declared and tagged in `occt_shim.hxx`, so the BREP-12 category lint covers them, and
  implemented in a new `cpp/occt_tess.cpp` (built by `build.rs`).
  - `tessellate_shape` makes a `BRepBuilderAPI_Copy` (`copyGeom = false`, `copyMesh = false`) and
    maps the copy's faces, edges and vertices. The copy keeps the input's structure, so its
    `MapShapes` order is the input's index order. It then runs `BRepMesh_IncrementalMesh` with an
    absolute deflection (`Relative = false`; amg-brep resolves relative qualities), the angular
    deflection, the optional `MinSize` and `InParallel`, under the BREP-12 progress/cancel
    bridge, and copies the result out.
  - Normals come from `BRepAdaptor_Surface::D1` at each UV node, flipped on reversed faces, with
    the summed triangle normals as a fallback where the surface normal vanishes (poles).
  - Triangles are swapped on reversed faces, so they are CCW seen from outside.
  - Weld ids: B-rep vertex index for the end nodes of each edge polygon and for every node of a
    degenerated edge; edge interior nodes get one id range per edge from that edge's
    `Poly_PolygonOnTriangulation` (in parameter order, the seam's two polygons included); face
    interior nodes get fresh ids. Edge polylines are the first face polygon's points, or the
    `Poly_Polygon3D` of a free edge, and empty for degenerated edges.
- `amg-brep`: module `tess` with `TessMesh`, `EdgePolyline`, `face_of_triangle`,
  `to_indexed_parts`, `triangle_count` and `estimated_bytes`, plus a hidden `TessProfile`. The
  `BrepKernel` trait gains `tessellate`, `tessellate_many` and `tessellate_edge`; `OcctKernel`
  implements them. `tessellate_many` runs rayon over the bodies with `InParallel` off; its
  progress is the number of finished bodies.
  - Hidden `OcctKernel::tessellate_profiled` (copy / mesh / copy-out times) and
    `OcctKernel::boolean_cut_probe`.
  - `EdgeId` derives `Default` (for `EdgePolyline: Default`).
  - New dependency on the workspace `rayon` (already listed; `THIRD_PARTY.md` row names amg-brep).
- Tests: `tests/tess.rs` (19 tests), the sphere convergence unit test in `src/tess.rs`, unit tests
  for `face_of_triangle`, `to_indexed_parts` and deflection resolution, and two new stress tests in
  `tests/stress.rs`. `common/stress.rs` gains `StressOp::{Tessellate, TessellateEdge}`,
  `tess_queries`, `mesh_hash` and `run_with` (a background closure on an extra thread). Goldens:
  `tests/golden/brep/tess_display_{box, cylinder, sphere, torus, filleted_box,
  cylinder_mirrored}.toml`. Bench `benches/tess.rs`.

**Done-when status** (`cargo nextest run -p amg-occt-sys -p amg-brep --features occt`: 104 tests
pass; without the feature 34 pass. `cargo clippy -p amg-occt-sys -p amg-brep --all-targets
--all-features -- -D warnings` is clean. All numbers are from a machine shared with three other
workers' builds.)
1. **Not met.** The 1030-face `plate_with_holes` at the display preset (deflection 0.467 mm,
   0.35 rad; 151 564 triangles) takes a **4.95 s** median (`plate_display_mesh_time`, 11 runs).
   The topology copy takes 23-33 ms (**0.7 %**) and the copy-out 33 ms, so nearly all of it is
   meshing.
   - Per-face profile: the two planar caps (1028 edges each, 1024 hole loops) take **3.9 s each**
     inside OCCT's `BRepMesh_Delaun` (Watson) for 38 914 triangles each. Each of the 1024
     cylinders takes about 1 ms. `InParallel` brings the whole shape from 8.8 s to 4.9 s (the two
     caps run concurrently).
   - `IMeshTools_MeshAlgoType_Delabella` is worse: over 2 minutes for the plate. Its
     constraint/hole recovery runs through the same `BRepMesh_Delaun` frontier code.
   - Ordinary faces fit the budget's order of magnitude: 38 filleted boxes (988 faces) through
     `tessellate_many` take 122 ms median, and the plate's 1028 non-cap faces 188 ms
     (`thousand_ordinary_faces_mesh_time`, loaded machine).
   - Follow-up **BREP-18**: a planar-face mesher with a real CDT. `budget/brep-tess-1000` is in
     place in `benches/tess.rs`, and the strict test assertion runs with `AMG_STRICT_TIMING=1`.
2. Met. For every closed fixture (box, cylinder, cone, sphere, torus, filleted_box,
   plate_with_holes, located_compound, cylinder_mirrored), at the preview and display presets,
   `to_indexed_parts` has 0 boundary and 0 non-manifold edges and no unused vertex. The volume
   is within `deflection × exact area` of `Shape::volume`; `mass_props` is BREP-09, and the
   exact `BRepGProp` volume stands in. Display examples: sphere 1429.49 vs 1436.76 (bound
   14.9), torus 9801.0 vs 9869.6 (bound 282), plate 400 469 vs 399 735 (bound 1.2e5). Euler is 2
   for genus 0, 0 for the torus, -2046 for the 1024-hole plate and 4 for the two-box compound.
3. Met (`sphere_area_converges_as_deflection_shrinks`, r = 10, absolute deflection, angular
   1.5 rad). Relative area error by deflection: 1 → 7.77e-2, 0.3 → 2.07e-2, 0.1 → 7.82e-3,
   0.03 → 2.39e-3, 0.01 → 7.74e-4, 0.003 → 2.37e-4, 0.001 → 7.94e-5 (100 520 triangles). It
   falls strictly, stays at or below 2.5 d/r, and ends below 1e-4.
4. Met (`shared_*_tessellates_during_booleans`). 8 threads run whole-shape tessellations
   (preview and display), `tessellate_edge` on 8 edges and reads, while another thread loops
   non-destructive `BRepAlgoAPI_Cut`s of the same shared shape with a box at its corner.
   - `filleted_box`: 8 × 40 iterations, 12 booleans. `plate_with_holes`: 8 × 16 iterations,
     62 booleans, 34.6 s.
   - No crash, and 0 mismatches against the single-threaded reference (bit hash of every
     position, normal, index and range).
   - Every cut has the same face count and the same volume bits.
   - The BinTools hash is unchanged (`cad3beeac5e9c130` box, `42a46e2564f1eef4` plate). A final
     single-threaded pass still matches, so no triangulation leaked onto the input.
5. Met (`face_of_triangle_matches_face_ranges`): checked for every triangle of every fixture.
   The per-triangle face ids of `to_indexed_parts` are non-decreasing.

**Deviations.**
- `TessQuality::relative` means "fraction of the shape's bbox diagonal" (the `opts` docs), not
  OCCT's per-edge `Relative` flag. amg-brep turns it into an absolute deflection, clamps it to
  `max_linear_abs`, and passes OCCT `Relative = false`. `tessellate_edge` resolves it against
  the shape's bbox too.
- No `TessMesh::as_soup()`: `amg-testkit` is a dev-dependency only, so the library cannot return
  a `TriSoup`. Tests build the soup from `to_indexed_parts` (two fields).
- `uvs` is always `None` (P2 as specified). Optional per-vertex curvatures (S10 request) are not
  done.
- The Done-when 4 booleans use the hidden `boolean_cut_probe` (one `BRepAlgoAPI_Cut`,
  `SetNonDestructive(true)`, no history), because BREP-04 has not landed. BREP-04 replaces the
  probe with `boolean` and keeps the test. The tool is the `box` fixture moved by `transformed`,
  since there is no box builder before BREP-14.
- Mid-op cancellation is coarse: OCCT polls the indicator between its steps, not inside a face's
  Delaunay. On the plate at the export preset (13 s), a cancel after 3.3 s returned `Cancelled`
  only after 13.0 s. The `cancel_during_meshing_returns_cancelled` time bound is asserted only
  under `AMG_STRICT_TIMING=1`, and would fail on this fixture today (BREP-18 shrinks the step).
- `iso_curves` (S05 request, 2026-10-20) is not part of BREP-08 as written and is left for the
  S05 WP that needs it.

**Leftovers.**
- BREP-18 (planar-face mesher) for Done-when 1.
- The criterion bench `budget/brep-tess-1000` was not run to completion on the shared machine:
  one display mesh is 5 s, so 30 samples would take minutes per group. Run it on an idle machine
  after BREP-18.

### 2026-11-12 - BREP-03 face and wire builders
**Landed.**
- `amg-brep`:
  - `geom`: the builder inputs of the Interfaces (`Tag`, `Curve2d`, `BSpline2dData`,
    `ProfileEdge2`, `Loop2`, `PlanarProfile`, `CurveSeg`, `FaceBounds`), field for field with S03's
    `Curve2`/`BSpline2`.
  - New `history` module: the data types `InputSub`, `Relation`, `OpRole`, `ShapeHistory`
    (`of_input`, `sources_of`, `role_of`, `tag_of`), `KernelWarning`, `OpResult`, `FaceBuild`.
    `make_wire` needs `OpResult`; BREP-05 adds extraction from OCCT and `compose`.
  - `BrepKernel::{make_planar_faces, make_planar_polygon, make_face, make_wire}` on the trait and
    `OcctKernel`; every result goes through `validate_result`.
  - New `builders` module: validation and lowering in plain Rust (so the unit tests run without
    OCCT), then one backend call. Profile curves become exact 3D curves on the plane (line,
    circle, ellipse, B-spline with the plane map applied to the poles). Closure is checked from
    curve end points (own rational de Boor); orientation from the signed area of a sampled
    polygon; containment by winding number on the sampled outer loop, with the sampling's chord
    error plus `tol.linear` as slack.
  - `backend/occt/build.rs` lowers to the bridge; the no-OCCT backend returns
    `BackendUnavailable` (checked before validation, as for every op).
- `amg-occt-sys`: shared structs `OcEdgeIn`, `OcLoopsIn`, `OcPlanarIn`, the opaque `OcBuild`, and
  shim functions `make_planar_faces`, `make_face_on_surface`, `make_wire`, `build_count`,
  `build_shape`, `build_edge_map` (all category R: plain data in, new topology out), in a new
  `cpp/occt_build.cpp`. Loops share one vertex per junction, with the junction's gap as its
  tolerance; planar faces get stored pcurves (`BRepLib::BuildPCurveForEdgesOnPlane`). The edge map
  gives every input edge's index in the result's `TopExp::MapShapes` order, which is the
  `EdgeId` space, so tags need no search on the Rust side.
- Tests: `tests/builders.rs` (17 tests incl. the proptest), unit tests in `src/builders/` (12) and
  `src/history.rs`, `api_contract.rs` (builder types in the serde/capture check, forwarding arms, a
  no-backend test). Golden `tests/golden/brep/face_profile_five_kinds.toml`. Bench
  `benches/builders.rs` (`budget/brep-planar-faces-50`, `budget/brep-planar-polygon-1000`).

**Done-when status** (`cargo nextest run -p amg-occt-sys -p amg-brep --features occt`:
134 tests pass; without the feature 48 pass. `cargo clippy -p amg-occt-sys -p amg-brep
--all-targets --all-features -- -D warnings` is clean.)
1. Met (`five_curve_kinds_give_a_valid_face_with_the_analytic_area`). Outer loop: line, arc
   (180°), ellipse arc, rational quadratic B-spline (an exact quarter circle), line. Hole: a
   circle. On `Plane::XY` and on a tilted plane the face is BRepCheck-valid and its area is
   953.163691086915 against the closed-form 953.163691071415 (**1.6e-11** relative). The face
   has 6 edges; `edge_tags` gives the 6 input tags on 6 distinct edges, and the hole's edge is
   alone in the second wire. `tags_are_on_the_right_edges` checks each tagged edge's curve kind.
2. Met. A hole whose last edge stops 1e-3 short gives `OpenLoop { loop_index: 2, edge_index: 3,
   gap: 1e-3 }`; an outer loop open after edge 0 gives `loop_index 0, edge_index 0`. A circle
   hole outside the square gives `InvalidInput { what: "profiles", .. }` naming
   `profile 0, loop 1, edge 0`. Unit tests also cover a hole crossing the boundary, wrong
   orientations and malformed curves.
3. Met. 1000-vertex outer polygon plus 10 holes of 20 vertices on a tilted plane: 1200 edges,
   11 wires, valid. **3.87 ms** median of 11 builds (debug test profile, `opt-level = 1`,
   validation off; loaded shared machine). Asserted < 20 ms only with `AMG_STRICT_TIMING=1`.
   Moving one vertex 1e-3 mm off the plane gives `NotPlanar { max_dev }` with max_dev in
   (5e-4, 1e-3); 5e-7 mm is accepted and projected.
4. Met (`polylines_from_tessellated_faces_reproduce_the_area`). Reference faces are `UvRect`
   patches on a cylinder, cone, sphere and torus with a tilted frame, the cone patch
   crossing the `u` seam. Each is tessellated (0.002 mm, 0.05 rad), its boundary edge polylines
   are chained into one loop and fed back as `Polylines`. Area error: cylinder 1.7e-16, cone
   2.7e-16, sphere 2.8e-15, torus 2.2e-15 (171–385 points). The reversed loop gives the same
   face. The 5×5 bicubic rational B-spline patch with `Natural` bounds is valid (4 edges).
   Also: `Natural` sphere and torus areas within 1e-9, a polyline hole on a cylinder (64 points)
   within 2e-3 of `400 − 4π`, and a polyline face on the B-spline patch.
5. Met (`random_profiles_give_valid_faces_or_clean_errors`, 1000 cases). Star-shaped loops of
   4–11 edges (line, arc, rational quadratic B-spline, bulging both ways), with no hole, a circle
   hole or a square hole, on a tilted plane. 30 % carry a defect (a gap, a scrambled edge order,
   a hole outside). Every defect-free profile builds a BRepCheck-valid face with one tag per edge;
   every defective one is `OpenLoop`/`InvalidInput`; nothing else happened.

**Findings and deviations.**
- **`Shape::face_area` (BREP-02) under-integrates pcurves with many spans.** It calls
  `BRepGProp::SurfaceProperties` without `Eps`, i.e. fixed-order Gauss per edge. A closed
  polyline hole made of one interpolating pcurve (64 points) read 12.30 against the true
  12.566 (its own tessellation gives 12.5657). The `Polylines` builder therefore splits each
  run into edges of at most 12 polyline segments (`MAX_EDGE_SEGMENTS`), which keeps every
  fixed-order integrator accurate. The query itself is unchanged; an adaptive `Eps` variant
  would cost speed in BREP-05's fingerprint budget, so it is a note for BREP-05/BREP-09.
- `Polylines`: points are projected with `ShapeAnalysis_Surface::ValueOfUV` and unwrapped by
  whole periods (not `NextValueOfUV`, which fell back to the principal range on the cone and
  produced a 62 % area error). Runs are split at 3D turns above 0.7 rad, and each run's pcurve
  interpolates its points (`Geom2dAPI_Interpolate`; two points give a segment) instead of the
  planned `Geom2dAPI_PointsToBSpline` approximation, which was no more accurate here. A
  polyline that winds once around a periodic direction (a full band) is rejected. Loops may run
  either way: outer loops are made CCW and holes CW in (u, v).
- `Loops`: wires are built from the 3D curves, `ShapeFix_Face` adds the pcurves, then the wires
  are re-oriented from their (u, v) area (first loop CCW, others CW); `BRepBuilderAPI_MakeFace`'s
  `Inside` flag alone left `BadOrientationOfSubshape` on a cylinder patch and a plane with a hole.
  Tags follow `ShapeFix`'s replacements through its `ShapeBuild_ReShape` context.
- Curve conventions the Interfaces left open (documented on the types): a `Curve2d::Circle` must
  be a loop by itself and runs the way its loop must (CCW outer, CW hole); S03 emits oriented
  full circles as `Arc` with `sweep = ±2π`, which work anywhere. `Ellipse` runs from `t0` to `t1`
  (`t1 < t0` is clockwise), and `minor_radius > |major|` is supported (axes swapped for OCCT).
  `CurveSeg` runs from `t0` to `t1`; `t1 < t0` is a reversed edge.
- `make_planar_faces` reports the first failing profile's loop and edge in `OpenLoop` (the
  variant has no profile index); `InvalidInput` reasons name the profile.
- `make_planar_polygon` auto-orients holes (meshes come either way); the outer loop defines the
  normal (Newell). Vertices are projected onto the fitted plane, so they move by at most
  `tol.linear`.
- Not validated: holes overlapping each other, self-intersecting loops. Periodic B-spline
  `CurveSeg`s are rejected for now.
- `make_wire` fills `ShapeHistory::tags` only (sorted by edge); no `entries`, since a wire has
  no input shapes.
- New dependency on `smallvec` 1.16.3 (already in use, listed in `THIRD_PARTY.md` for the
  amg-brep history); `proptest` as a dev-dependency (workspace).

**Leftovers.**
- R-S04-3 `make_thin_face` (2026-11-08 request) is not in this WP's row; S04's SOL-18/SOL-13
  adds it (same lane) or a follow-up BREP WP takes it.
- The criterion bench `benches/builders.rs` was not run on the shared machine (a release build
  with OCCT); the test-profile numbers above stand in.
- Self-intersection and hole-overlap checks for profiles (S03 SK-06 already guarantees both for
  sketch regions).

### 2026-11-12 - BREP-17 geometry extraction (plane sections, edge projection)
**Landed.**
- `amg-brep`: `BrepKernel::{plane_section, project_edges}` on the trait and `OcctKernel` (both use
  `KernelConfig::tolerance`), new module `extract`, `Curve3::point_at`, `CurveSeg::{start, end}`.
  - `project_edges` is exact affine geometry in plain Rust (unit-tested without OCCT): a line
    stays a line (parameters scaled by the direction's in-plane length); a circle or ellipse
    becomes an ellipse from its projected conjugate semi-diameters (principal axes at
    `t* = ½·atan2(2A'·B', |A'|² − |B'|²)`), a circle when the axes agree within 1e-9 relative, or
    line segments when the minor axis is below `tol.linear` (edge-on), split where the image
    turns back; a B-spline keeps knots, multiplicities and weights with projected poles, and
    becomes a line when its poles are collinear and monotone. Conic frames get
    `normal = plane.normal`, so `t1 > t0` runs counter-clockwise seen from the normal. B-spline
    edges and every other curve go first through the new shim `edge_curve_recognised`
    (`GeomConvert_CurveToAnaCurve` for lines/circles/ellipses within the tolerance, exact
    `GeomConvert` B-spline for parabolas/hyperbolas/periodic B-splines, `GeomConvert_ApproxCurve`
    C2 within the tolerance for offset curves and pcurve-only edges). Lines along the normal and
    degenerated edges give nothing. Tags are edge indices; output keeps the input edge order and
    flips segments so consecutive ones sharing an end point run head to tail.
  - `plane_section`: shim `plane_section` runs `BRepAlgoAPI_Section` (non-destructive,
    `Approximation(true)`, no pcurves, serial) against a finite face on the plane covering the
    shape's bbox, and returns each section edge's curve through the same recognition plus the
    face it lies on (`HasAncestorFaceOn1`). Rust chains the segments into head-to-tail loops
    (link distance `10 × max(tol.linear, shape max tolerance)`) and merges consecutive pieces of
    one carrier line/circle/ellipse on one face (also across the closing joint of a loop). Tags
    are face indices (`Tag(u64::MAX)` if OCCT names none).
- `amg-occt-sys`: shared struct `OcSection`, shim functions `plane_section` (category **N**) and
  `edge_curve_recognised` (R), in a new `cpp/occt_extract.cpp` (built by `build.rs`). Both are
  tagged in `occt_shim.hxx`, so the category lint covers them. `SHIM_VERSION` unchanged (no
  existing function changed).
- Tests: `tests/extract.rs` (12 tests), 14 unit tests in `src/extract/tests.rs`, the forwarding
  arms in `api_contract.rs`. Golden `tests/golden/brep/section_filleted_box_mid.toml`.

**Done-when status** (`cargo nextest run -p amg-occt-sys -p amg-brep --features occt`: 160 tests
pass; without the feature 62 pass. `cargo clippy -p amg-occt-sys -p amg-brep --all-targets
--all-features -- -D warnings` is clean, and so is the same without features.)
1. Met. `cylinder` fixture cut at z = 0.5, 7, 19.5: one `Curve3::Circle` each, radius exactly
   `5.0` (bit equality), centre (3, 1, z) within 1e-12, sweep 2π, tagged with the lateral face.
   Oblique cuts through (3, 1, 10) tilted θ = 0.2, 0.5, 0.9 rad about x: one `Ellipse` each;
   major radius vs `5 / cos θ` and minor vs 5 differ by **0** (exact `IntAna` conics), centre,
   major-axis direction and normal within 1e-9.
2. Met. The 20 non-degenerated edges of the filleted box's top band (z ≥ 29: top face, top
   fillet cylinders, corner spheres) projected onto XY give 16 lines (4 top-face sides, 4
   fillet/side boundaries, 8 corner arcs in vertical planes seen edge-on) and 4 circle arcs of
   radius 1 (error < 1e-9) centred on (1, 1), (9, 1), (1, 19), (9, 19), sweep π/2. Every
   segment's end points are its edge's projected vertices within 1e-7; the sphere pole edges are
   skipped.
3. Met. Cuts of the `bspline_patch` (z = 45u(1−u)v(1−v), x = 30u, y = 30v) by x = 12, y = 21 and
   two oblique planes: one B-spline segment each, boundary to boundary. Sampled at 201 points,
   the largest distance off the plane or off the surface is **1.5e-8 to 3.6e-8 mm** (bound 1e-6).

**Also checked.** A plane containing the cylinder axis gives a closed loop of 4 lines (length
60); sphere and torus cuts give circles of the analytic radii (torus: two closed loops); a box
cut gives a 4-line loop with one face tag per line, deterministic; a plane missing the shape gives
nothing; a bad plane or edge index is `InvalidInput`. The patch's B-spline boundary edges come
back as lines. Every wire of every filleted-box face projected onto a tilted plane is a closed
head-to-tail loop, and the `plate_with_holes` cap's 1025 wires give the outer 4-line loop first,
then 1024 single closed circles. Golden: the filleted box cut at z = 15 is an 8-segment loop (4
arcs) that `make_face(Plane, Loops)` turns into a face of area 199.14159265358990 vs the exact
`200 − (4 − π)` (1.1e-13).

**Answers to requests.** S12 (2026-10-20, item 3): confirmed. `project_edges` on
`Shape::face_wires(f)` wire by wire (outer wire first) gives ordered closed loops, so
`planar_face_loops` is not needed (a caller-side loop over wires). The projected loops run in
wire order; their orientation follows the first edge (callers that need CCW/CW check the signed
area).

**Deviations.**
- "`tessellate_edge` reuse": no new polyline API. Drawings that need polylines of model edges
  call `tessellate_edge`; extracted `CurveSeg`s are sampled with `Curve3::point_at`. Curves
  without an exact form are approximated as B-splines in the shim rather than tessellated.
- Neither query takes a `JobCtx` (as specified); a section of a large part cannot be cancelled.
- Projected B-splines are only recognised as lines (in Rust); a B-spline image that is a circle or
  ellipse stays a B-spline. Edges whose 3D curve is a B-spline circle are recognised before
  projection.
- No bench: S02 has no budget for these queries.

**Leftovers.**
- R-S04-4 `silhouette_curves` (2026-11-08 request) is not in this row; SOL-22 or a follow-up
  BREP WP adds it next to `extract`.
- Chaining is O(n²) in the segment count (fine for the 1030-face plate's sections; a spatial hash
  if sections of much larger parts get slow).

### 2026-11-13 - BREP-14 primitives, prism, revol, gtransformed
**Landed.**
- `amg-brep`:
  - `BrepKernel::{make_box, make_cylinder, make_cone, make_sphere, make_torus, prism, revol}`
    as in the Interfaces, plus `mirrored(s, &Plane)` (R-S04-6, assigned to BREP-14 on
    2026-11-08) on the trait and `OcctKernel`; `Shape::gtransformed(&Transform) ->
    KResult<OpResult>`. Every result goes through `validate_result` (except prisms infinite
    both ways, below).
  - New module `ops`: parameter validation in plain Rust (frames orthonormal and right-handed
    within 1e-9; sizes and radii above `tol.linear`; sweep angles in `(0, 2π]`, values up to
    `2π + 1e-12` clamped; cone radii different; torus `major > minor`; profiles are faces,
    shells, wires, edges or compounds, not solids; prism vector not in the plane of a planar
    face; taper within ±(π/2 − 1e-6), finite prisms of one planar face only), the reflection
    matrix for `mirrored`, and the conversion of the shim's flat history into `ShapeHistory`.
  - Primitives: `BRepPrimAPI_Make{Box,Cylinder,Cone,Sphere,Torus}` on
    `gp_Ax2(origin, normal, x)`; the box has a corner at the frame origin, the others are
    centred on it. Empty history (no input).
  - Prism and revol return history from operand 0 (the profile): each face `Modified` into the
    start cap and `Generated` into the end cap; each edge `Modified` into its start-cap edge
    and `Generated` into its side face(s) and end-cap edge; each vertex likewise into its
    start vertex, side edge and end vertex; roles `StartCap`, `EndCap`, `Lateral`;
    `operand_counts`. Every output face of the tested prisms has a source and a role (the
    BREP-05 completeness rule, ahead of BREP-05). Cancellation is checked before and after the
    OCCT call (neither `MakePrism` nor `MakeRevol` reports progress).
  - Taper: the straight prism, then `BRepOffsetAPI_DraftAngle` on every side face (pull = the
    prism vector, neutral plane = the profile plane, angle = −taper). **Positive taper grows the
    profile along the sweep** (outer walls lean out, holes shrink), the same rule for every wall,
    as an offset of the region. The spec did not fix the sign; S04 SOL-02 maps its dialog onto it.
  - `merge_coplanar`: `ShapeUpgrade_UnifySameDomain` (faces and edges, no B-spline
    concatenation) on the new result, its `BRepTools_History` composed into the prism history.
  - `gtransformed`: a similarity (`MᵀM = s²I` within 1e-12) goes through `gp_Trsf::SetValues` and
    `BRepBuilderAPI_Transform` with copied geometry (analytic surfaces stay analytic, reflections
    keep faces outward); any other invertible map through `BRepBuilderAPI_GTransform` (B-spline
    geometry). History: every face, edge and vertex `Modified` 1:1. Singular or non-finite maps
    are `InvalidInput { what: "transform" }`. `transformed` still rejects every non-rigid map.
- `amg-occt-sys`: shared struct `OcOpHistory`, opaque `OcOp`, shim functions `make_primitive`
  (R), `prism` (**C**), `revol` (**C**), `shape_gtransformed` (R), `op_shape` (R) and
  `op_history` (R), in a new `cpp/occt_ops.cpp` (built by `build.rs`), declared and tagged in
  `occt_shim.hxx` (the category lint covers them). The test-only `make_sphere` shim (BREP-08) is
  gone; its one user, the sphere-convergence unit test, calls `make_sphere` on the kernel.
  `SHIM_VERSION` unchanged (no existing function changed).
- Tests: `tests/ops.rs` (18 tests), 5 unit tests in `src/ops.rs`, forwarding arms and a
  no-backend test in `api_contract.rs`, three sweep stress tests in `tests/stress.rs`
  (`StressOp::{Prism, Revol}` in `common/stress.rs`). The BREP-08 tessellation stress test now
  cuts with a `make_box` primitive instead of the `box` fixture moved by `transformed`. Goldens:
  20 files `tests/golden/brep/ops_*.toml` with exact B-rep metrics (volume, area, face/edge/
  vertex counts, face classes; relative bound 1e-9), checked by a small helper in
  `tests/ops.rs` (`AMG_BLESS=1` rewrites, refused under `CI`). Bench `benches/ops.rs`.

**Done-when status** (`cargo nextest run -p amg-occt-sys -p amg-brep --features occt`: 186 tests
pass; without the feature the OCCT-free tests pass, see the handoff counts. `cargo clippy -p
amg-occt-sys -p amg-brep --all-targets --all-features -- -D warnings` is clean. Shared machine.)
1. Met. Analytic values vs `Shape::volume` / summed `Shape::face_area`, all on a tilted frame for
   the primitives; relative errors are at most **7.5e-16** everywhere:
   - box 10×20×30 (6000 / 2200, 6/12/8), full and 90° cylinder, cone frustum (r 5→2, h 10) and
     full cone, sphere r 7, full and 180° torus (R 10, r 3);
   - prisms of five profile kinds: square face, circle face, square with a circular hole,
     ellipse face on a tilted plane (as four quarter arcs, below), open two-line wire (a shell,
     area 105, 2/7/6); also a slot (tangent lines and arcs), an oblique vector (Cavalieri) and a
     `merge_coplanar` case (7 → 6 faces, the merged side face has both bottom edges as sources);
   - revol of a rectangle at 360° (Pappus 156π / 182π) and 90° (39π / 45.5π + 24), a triangle
     with an edge on the axis (cone, 32π), and a wire (shell);
   - goldens for all of them in `tests/golden/brep/ops_*.toml`.
2. Met. A 10×10 square, h = 10, at +5° and −5°: volumes 1185.1830153792514 / 835.2283612755552
   against `h/3 (A₁ + A₂ + √(A₁A₂))`, relative error **1.9e-16 / 1.4e-16**; areas within 2.1e-16;
   every side face is a plane tilted by exactly the taper (1e-12). Also tapered circles (cone
   frusta), slots (Steiner: `A h + P tan t h²/2 + π tan² t h³/3`) and a square with a hole that
   shrinks, all within 2.2e-16.
3. Met. Every test kernel runs with `Validate::Always`, and `assert_measures` also runs `check`;
   all results are BRepCheck-valid, except prisms infinite **both** ways (deviation below). A
   profile crossing the revol axis and an inward taper that collapses the square fail cleanly.
4. Met (`shared_{box, filleted_box, plate}_survives_concurrent_sweeps`). 8 threads run straight
   prisms, tapered prisms (faces with ≤ 16 edges) and 90° revols of the planar faces of one shared
   shape, taken out with `sub_shape` (so the profiles share the input's TShapes), alongside
   `Bbox`, `Counts` and `Check`. Each result is compared as volume bits, counts and the whole
   `Debug` history: 0 mismatches. BinTools hash unchanged: box `e40ca138fde6fcd8`, filleted box
   `cad3beeac5e9c130`, plate `42a46e2564f1eef4` (8 × 200 / 60 / 6 iterations; the plate run with
   its two 1028-edge caps takes 19.9 s).

**Budget.** Prism of a 100-edge star profile: **1.59 ms** median of 11 (test profile, validation
off, `prism_of_a_100_edge_profile_is_fast`; asserted < 10 ms with `AMG_STRICT_TIMING=1`).
`budget/brep-prism-100` is in `benches/ops.rs` with taper, revol, `make_box` and `make_sphere`
benches. The criterion run was not made on the shared machine (release build with OCCT), so the
BREP-12 `brep-ffi-overhead` re-measurement on an idle machine is still open.

**Findings and deviations.**
- **Sweeps copy their profile (category C, not N).** `BRepSweep` stores the side faces' pcurves on
  the base edges it sweeps. With `Copy = false` that writes into the caller's shape, which may
  share TShapes with a stored body (a profile from `sub_shape`). The shim sweeps a private
  `BRepBuilderAPI_Copy` (`copyGeom = false`) and composes the copy's map into the history. The
  stress test's unchanged hashes cover it.
- **`MakeRevol` loses the history of canonised faces.** After the sweep it replaces some faces
  (a revolved radial line becomes a plane) and then reports their generating edges as deleted,
  with nothing generated. The shim falls back to the raw sweep (`BRepSweep_Revol::Shape(edge)`)
  and maps an image missing from the result to the one result face or edge with the same vertex
  TShapes and edge count (vertices survive). Before the fix a 360° revol of a rectangle had 2 of
  its 4 faces unexplained; now all 4 have a source and the `Lateral` role.
- **Taper is limited to planar, cylindrical and conical side faces** (lines and circle arcs in
  the profile), which is what `BRepOffsetAPI_DraftAngle` accepts; tangent chains (slots) are
  drafted together. An ellipse or B-spline side face gives `Occt { .. }`. S04 SOL-02's text
  profiles (B-splines) need another route (`LocOpe_DPrism`, or offset + ruled loft): a leftover.
- **Prisms infinite both ways are never BRepCheck-valid** (`UnorientableShape`, `NotConnected`:
  no cap closes the shell), so `prism` skips the validation policy for them only. Semi-infinite
  prisms are valid and validated.
- **`Shape::face_area` and `Shape::volume` use fixed-order quadrature** (the BREP-03 finding,
  again): the side face of a single full-turn elliptic edge reads **2.3e-6** low, while the same
  ellipse as four quarter arcs is exact (2e-16), so the Done-when 1 ellipse profile uses quarters
  and the one-edge case is checked at 1e-5. The `gtransformed` ellipsoid (sphere r 10 scaled
  (1, 2, 3), rational B-spline faces) reads **4.4e-4** low in volume while its extent is exact to
  1e-6. S04 SOL-06 Done-when 5 (ellipsoid within 1e-9) needs BREP-09's adaptive `mass_props`
  (`BRepGProp` with `Eps`), not a change here.
- No wedge (S02 §5 lists it; the trait and the WP row do not). `make_sphere` takes no angle
  (as the trait).
- `mirrored` (R-S04-6) is on the trait here rather than in a later WP: it is `gtransformed` with
  the reflection about the plane, plus validation.
- Goldens are B-rep metrics in their own TOML files, not `amg_testkit::assert_golden!`, which
  compares tessellations at 1e-6 and cannot hold the 1e-9 bound.

**Leftovers.**
- A taper for side faces `DraftAngle` cannot tilt (ellipses, B-splines; S04 SOL-02 text
  profiles).
- Run `benches/ops.rs` and BREP-12's `brep-ffi-overhead` on an idle machine.
- Prism/revol take no `Message_ProgressRange` (OCCT reports none for them); cancellation is
  before/after only.

### 2026-11-14 - BREP-04 booleans, split, section
**Landed.**
- `amg-brep`:
  - `BrepKernel::{boolean, split, section}` on the trait and `OcctKernel`, replacing BREP-08's
    hidden `boolean_cut_probe` (removed from the kernel, backend and shim; the BREP-08
    tessellate-during-booleans stress test now loops `boolean(Cut, ..)`).
  - New module `boolean`: operand-list and option checks in plain Rust (objects non-empty; a fuse
    needs two shapes in all and may leave `tools` empty, which fuses the first object with the
    others; cut/common need a tool; no null operand; `fuzzy` finite and ≥ 0), `BoolOpts`
    lowering (`parallel: None` = `KernelConfig::bool_parallel`), the status → error mapping and
    the warnings. Validation errors come after `BackendUnavailable`, as for every op.
  - Results: `OpResult` with history over every operand (objects first, then tools;
    `operand_counts` per operand): each face, edge and vertex `Kept` (its TShape is in the
    result), `Modified` / `Generated` from `BRepAlgoAPI_BuilderAlgo`'s history, else `Deleted`.
    `section` reports each face of `a` (operand 0) and `b` (operand 1) `Generated` into the
    section edges on it (`HasAncestorFaceOn1/2`) and gives every edge the `SectionEdge` role.
    Warnings: `FuzzyUsed` when `fuzzy > 0`; `ToleranceGrown { max }` when the result's largest
    tolerance exceeds every input's and the kernel's linear tolerance. Failures:
    `OpFailed { op: "boolean" | "split" | "section" }` whose message lists the BOPAlgo error
    alert names (and the operand sub-shapes they name), `offending` holds those sub-shapes;
    a cancelled run (BOPAlgo `AlertUserBreak` or the sink's break) is `Cancelled`. BOPAlgo
    warnings go to `tracing` (debug).
  - `ops::history_n` (multi-operand history; relation code 3 = kept); `ops::history` wraps it.
  - Hidden `Shape::volume_adaptive(eps)` (`BRepGProp::VolumeProperties` with `Eps`), see the
    findings.
  - `KernelConfig::bool_parallel` now defaults to **`true`** (see Done-when 4 and 5).
- `amg-occt-sys`: shared structs `OcBoolOpts`, `OcBoolStatus`, `OcOpHistory::in_operands` (empty
  for the one-operand BREP-14 ops), opaque `OcShapeList`, shim functions `new_shape_list` (R),
  `shape_list_push` (R), `boolean_op` (**N**), `section_op` (**N**), `op_bool_status` (R) and
  `volume_adaptive` (R) in a new `cpp/occt_bool.cpp` (built by `build.rs`), declared and tagged
  in `occt_shim.hxx`; `OcOp` gains `in_operands`, `cancelled`, `errors`, `warnings` and the
  offending lists. `boolean_cut_probe` is gone. `SHIM_VERSION` unchanged (no existing function
  changed behaviour). Every boolean runs `SetNonDestructive(true)`, `SetToFillHistory(true)`,
  the fuzzy value, glue, check-inverted, OBB and parallel flags, under the BREP-12
  progress/cancel bridge (`Build(Message_ProgressRange)`).
- Corpus `testdata/brep/bool/cases.toml` (136 cases, one text file, each with its `source`),
  tests `tests/boolean.rs` (13 tests + 1 ignored timing run), `tests/common/parts.rs`
  (perforated plates, shared with the bench), forwarding arms and a no-backend test in
  `api_contract.rs`, unit tests in `src/boolean.rs` and `src/ops.rs`. Goldens
  `tests/golden/brep/bool_{blind_hole, steinmetz, cylinder_cross_fuse, spheres_split}.toml`
  (exact metrics, 1e-9; their values match the closed forms: 1000 − 16π, Steinmetz 144 mm³ and
  144 mm², 2·180π − 144, sphere union). Bench `benches/boolean.rs`.

**Done-when status** (`cargo nextest run -p amg-occt-sys -p amg-brep --features occt`: 201 tests
pass, 1 skipped (the ignored timing run); without the feature 72 pass. `cargo clippy -p
amg-occt-sys -p amg-brep --all-targets --all-features -- -D warnings` is clean;
`cargo build -p amg-brep --no-default-features` builds. Shared, loaded machine.)
1. Met (`corpus_gives_reference_volumes_or_expected_errors`). 136 cases: 131 valid results
   (`Validate::Always` plus an explicit `check`), largest relative volume error **1.67e-7**
   (bound 1e-6), and 5 expected errors (`OpFailed` `BOPAlgo_AlertBOPNotAllowed` for a fuse of a
   solid and a face; four `InvalidInput`). Cases: box/box (axis-aligned and rotated), through
   holes, coaxial cylinders, sphere caps, sphere/sphere lenses, Steinmetz solids, napkin rings,
   cone frusta, torus halves, tubes, N × M operand lists, split (by faces, by a box, objects by
   each other), coplanar (stacked, flush slots, identical boxes, glue Shift/Full), tangent
   (point and line contact), fuzzy-needed (1e-5 gap, 2e-6 / 3e-6 near-coincident faces, two
   drills 5e-6 apart) and disjoint/contained. Expected solid counts are checked where they are
   determined; history indices are in range for every result. The reference volumes are
   analytic; the fuzzy/coplanar/tangent cases are modelled on the classes of reports on the
   OCCT and FreeCAD trackers rather than harvested shapes (deviation below).
2. Met (`random_primitive_pairs_give_valid_results_or_clean_errors`, 500 cases): random box,
   cylinder, cone, sphere and torus pairs (random sizes, positions and axes), random fuse, cut,
   common or split. **500 Ok(valid), 0 Err**, no crash; both inputs' hashes unchanged each time.
3. Met: every corpus op and every proptest pair compares both operands' `BinTools` hashes before
   and after. **Finding:** this first failed on 6 corpus cases (disjoint and contained operands,
   two boxes 1e-5 apart): BOPAlgo puts an argument it leaves untouched into the result compound
   as is, and `TopoDS_Builder::Add` clears that TShape's `Free` flag, which BinTools writes
   (`1100000` → `0100000` in the text format). Non-destructive mode does not cover it. The shim
   now gives every operand a private top-level TShape with the same children, location,
   orientation and flags (`detached` in `occt_bool.cpp`), so the write lands on the copy; the
   index order below the top is unchanged. Below the top, `Add` only rewrites the already-clear
   flag of the operands' own children (a same-value write, as every OCCT builder that shares
   sub-shapes does).
4. Met (`parallel_mode_is_deterministic`): the 20 `determinism = true` cases, each 10 runs with
   `parallel: Some(true)` on an 8-thread OCCT pool: **0 differ** byte for byte, and each equals
   the serial run's bytes as well. `fuzzy_joins_a_gap_and_says_so` also checks that OBB,
   check-inverted off, serial and parallel give identical bytes and history.
5. Met (`benches/boolean.rs`, `occt-threads/cut-1030/*`, release, loaded shared machine with
   16 logical cores): a cut of two 1030-face perforated plates (the tool shifted by half a
   pitch and half the thickness, so every face of both meets the other; 3143 result faces):
   serial **9.91 s**, parallel with 1 thread **12.25 s**, with 8 (half) **8.15 s**, with 16 (all) **7.56 s**.
   (criterion medians, 10 samples each). **Defaults set from it:** `bool_parallel` is now on
   (half the cores: 18 % faster than serial; all cores: 24 %), but the kernel runs serially when
   its OCCT pool has one thread, since parallel mode on one thread is 24 % slower than serial.
   `occt_threads` stays at `max(1, cores / 2)`: all 16 threads gain only 7 % over 8 and would
   compete with the Interactive rayon pool (occt-integration.md §5).
6. Met (`cancelling_a_long_boolean_returns_cancelled_quickly`): the same 1030-face cut (12.7 s to
   completion in the test profile) cancelled 300 ms in returns `Cancelled` **15-16 ms** after
   `cancel()` (two runs). The < 200 ms assertion runs with `AMG_STRICT_TIMING=1`.

**Budget.** `budget/brep-bool-200` (cut of two 202-face perforated plates, the same layout as
above): **442 ms** median with the new default (parallel, 8-thread pool; 657 ms serial),
criterion, release, loaded machine, against the < 50 ms target: **not met** by this layout. It is a worst case rather than a typical one: every one of the 2 × 196
holes and both caps of each part intersect the other part, and the plate caps carry ~200 hole
loops each, which BOPAlgo's face builder splits. Simple cases are far below it (the whole
136-case corpus, debug-built test profile, runs in about 1 s). A typical-part case (a 200-face
filleted body cut by a pocket tool) needs a fixture BREP-06 can build; leftover below.

**Findings and deviations.**
- **`Shape::volume`'s fixed-order quadrature misreads boolean results** (third time after BREP-03
  and BREP-14): the lens of two spheres whose centre line is oblique to their poles reads
  **9.5e-5** relative low with `Shape::volume`, while `BRepGProp` with an adaptive `Eps`
  (`Shape::volume_adaptive(1e-10)`) gets the analytic value within 1.7e-7. The corpus compares
  adaptive volumes and prints both. BREP-09 `mass_props` should use the adaptive path.
- A common of one object with several tools is BOPAlgo's group semantics (object ∩ union of
  tools), returned as the pieces the tool boundaries split it into (3 solids for 2 overlapping
  tools), not one solid; the corpus records it.
- The corpus is synthesised from primitives with analytic reference volumes rather than
  harvested tracker shapes: the trackers' attachments are mostly STEP/BREP files without a stated
  licence and without reference volumes. The coplanar, tangent and fuzzy-needed cases reproduce
  the configurations those reports are about. Harvesting real files is a leftover.
- `split` gives no `SplitPiece` roles and the boolean history has no split order (area desc) yet:
  both are BREP-05's (which applies the history framework to BREP-04).
- `section` takes no options (as the trait): it runs with `KernelConfig::bool_parallel`, no fuzzy.
- `KernelError::OpFailed::offending` holds sub-shapes of several operands; the message names the
  operand of each.

**Leftovers.**
- The < 50 ms "typical boolean" budget needs a typical 200-face pair (after BREP-06) and an idle
  machine; the perforated-plate case (442 ms) is a stress layout.
- Harvest OCCT/FreeCAD tracker shapes into the corpus where licences allow.
- R-S04-2 `OpResult::per_solid` (BREP-05) for cut/split results.

### 2026-11-15 - BREP-09 mass properties and measurement queries
**Landed.**
- `amg-brep`: new module `query` with `MassProps`, `Classification`, `DistanceResult`, `RayHit`,
  `SurfacePoint` and `EdgeEnds` (R-S04-1), all also `Serialize`/`Deserialize`, and
  `MASS_PROPS_EPS` (1e-12). `BrepKernel::{mass_props, classify, distance, closest_point,
  ray_cast, interference, eval_surface, project_point_on_face, edge_ends}` on the trait and
  `OcctKernel`, with forwarding arms in `api_contract.rs`. The crate re-exports
  `amg_core::{Mat3, Ray3}`. The shim returns OCCT's raw values and the conventions are applied in
  Rust, where they are unit-tested without OCCT:
  - `mass_props`: adaptive `BRepGProp::VolumeProperties(eps, OnlyClosed)` over the closed shells,
    and `SurfaceProperties(eps)` for the area. Inertia is about the centroid at unit density
    (GProp's convention, `I_xy = −∫xy`). Principal moments (ascending) and axes come from a cyclic
    Jacobi decomposition in Rust. The axes are right-handed, and axes 0 and 1 have their largest
    component positive. Off-diagonal round-off below 1e-13 × the largest moment is cleared first,
    so a sphere keeps its frame's axes, deterministically. A shape without solids is measured by
    its faces (unit areal density) or its edges, and `volume` is then 0. A shape with only
    vertices gives `InvalidInput`.
  - `classify`: `BRepClass3d_SolidClassifier` per solid of the index (inside any solid wins over
    on a boundary). A shape without solids gives `InvalidInput`.
  - `distance`: `BRepExtrema_DistShapeShape` (MIN, single-threaded), first solution, with the
    supports mapped to `SubShape`s through the index. Overlap or containment gives 0 with no
    supports.
  - `closest_point` measures to the boundary. Inside a solid, BRepExtrema returns an inner
    solution at distance 0; it is replaced by the nearest of the faces, taken one by one. Putting
    the faces into one compound would clear their TShapes' `Free` flag (the BREP-04 finding).
    `on_a`/`sub_a` are on the shape, `on_b` is the point.
  - `ray_cast`: `IntCurvesFace_Intersector` per face (states IN or ON, kernel linear tolerance),
    nearest `t`, ties to the lower face index; `max_t` is clamped to the bbox reach. The normal
    comes from `BRepLProp_SLProps` at the hit, flipped on reversed faces.
  - `eval_surface`: `BRepLProp_SLProps` (order 2). The normal points out of the material and the
    curvatures are **convex-positive** against it: a sphere is +1/r and a hole wall −1/r, which is
    `−sign × OCCT`. Min/max and the directions swap with the sign. At an umbilic point `dir_min` is
    the normalised `d1u` and `dir_max = normal × dir_min`. A point with no normal or curvature (a
    sphere pole) gives `OpFailed { op: "eval_surface" }`.
  - `project_point_on_face`: `BRepExtrema` against the face, so the result is within its boundary.
    When the foot is on an edge or vertex, `uv` comes from `ShapeAnalysis_Surface::ValueOfUV`,
    wrapped into the face's UV range on periodic surfaces.
  - `interference`: `boolean(Common)` with default options. It returns `None` when the result has
    no solid or a volume below `tol.linear³` (touching boxes). Both shapes must have solids.
  - `edge_ends`: `BRepAdaptor_Curve::D1` at both ends, in the curve's parameter direction (the
    order of `edge_vertices` and `curve`). Degenerated edges and vanishing tangents give
    `InvalidInput { what: "edge" }`.
- `amg-occt-sys`: shared structs `OcMassProps`, `OcDistance`, `OcRayHit`, `OcSurfacePoint` and
  `OcEdgeEnds`, and shim functions `mass_props`, `classify_point`, `distance_shapes`,
  `closest_point_on`, `ray_cast_faces`, `eval_face`, `project_on_face` and `edge_ends` (all
  category **R**). They are in a new `cpp/occt_query.cpp` (built by `build.rs`) and are declared
  and tagged in `occt_shim.hxx`. `SHIM_VERSION` is unchanged (no existing function changed).
- BREP-08's `indexed_parts_of_closed_fixtures_are_watertight` now takes the exact volume and area
  from `mass_props` instead of the `Shape::volume` stand-in.
- Tests: `tests/query.rs` (17 tests), 6 unit tests in `src/query.rs`, and three measurement stress
  tests in `tests/stress.rs` (`StressOp::{MassProps, Classify, ClosestPoint, RayCast,
  EvalSurface, ProjectOnFace, EdgeEnds, DistanceToSub}`, `measure_queries` in
  `common/stress.rs`). Bench `benches/query.rs`. There are no goldens, because no new geometry is
  produced (`interference` is a boolean, covered by the BREP-04 goldens). The mass properties are
  compared with closed forms directly, as §Testing asks.

**Done-when status.** `cargo nextest run -p amg-occt-sys -p amg-brep --features occt`: 227 tests
pass, 1 skipped (BREP-04's ignored timing run). Without the feature, 78 pass. `cargo clippy -p
amg-occt-sys -p amg-brep --all-targets --all-features -- -D warnings` is clean, also without
`--all-features`. `cargo build -p amg-brep --no-default-features` and `cargo check --workspace`
build. All numbers are from a shared machine, and all bodies sit on a frame tilted off every axis.
1. Met. Volume, area, centroid, the full inertia tensor (`R·diag·Rᵀ`) and the principal moments
   were checked against closed forms. The tensor error is the largest entry error over the
   largest entry. Largest relative errors (volume / tensor):
   - box 10×20×30: 6e-16 / 8.6e-16; the principal axes are the frame axes;
   - cylinder r 5, h 12: 6e-16 / 8.6e-12;
   - sphere r 7: 1.6e-16 / 3.9e-16;
   - torus R 10, r 3: 7.7e-15 / 1.5e-11;
   - also a cone frustum (tensor 1.9e-11), two disjoint boxes as one compound (parallel-axis
     rule, 8e-16) and a planar face (area basis, polar moment 1.1e-15).
2. Met.
   - Two spheres (r 3 and r 5) with centres 20 apart along an oblique direction: distance 12
     within 4.4e-16, nearest points within 1e-9 × 20, both supports faces, symmetric.
     Overlapping spheres give 0.
   - Point to face: cylinder side 7 mm off (exact), cap plane 4.5 (5.9e-16), cap rim 5 (edge
     support, 5.3e-16), sphere 5 (1.8e-16).
   - A point 1.5 mm inside a cylinder to its boundary: 4.4e-16.
3. Met (`rays_hit_a_cylinder_where_the_analytic_solution_does`). 1000 seeded rays start on a
   sphere of radius 40 and are aimed at random points inside a cylinder r 5, h 12. 778 hit the
   side, 120 the bottom and 102 the top, each on the analytic face. The worst `t` error is
   **1.5e-14** relative and the worst normal error **1.1e-13**; every normal faces the ray. A miss
   and a ray stopped short give `None`.
4. Met (`torus_curvatures_are_analytic`). At 10 points on a torus R 10, r 3: point, outward
   normal, `k = 1/r` and `cos v / (R + r cos v)`, and both principal directions. Worst curvature
   error **5e-16** relative to 1/r. Sign checks: a sphere is +1/r (umbilic), a cylinder 1/r and
   0, and a drilled hole's wall −1/r with the normal pointing into the hole.
5. Met. A 10×20×30 box and a 10 mm cube at (5, 5, 5): common volume 500 within 8e-16, centroid
   within 1e-9, symmetric. Touching faces and a 1 mm gap give `None`.

**Concurrency** (`shared_{box, filleted_box, torus}_survives_concurrent_measurements`). 8 threads
× 300 / 100 / 200 iterations over 55 / 115 / 41 ops, `Check` included, give 0 mismatches against
the single-threaded reference. The BinTools hashes are unchanged (`e40ca138fde6fcd8`,
`cad3beeac5e9c130`, `b189a803c8b4f159`). The filleted box's 3 expected errors (sphere-patch poles
and degenerated edges) are deterministic too.

**Cost** (`benches/query.rs`, release, criterion, shared machine; S02 has no budget for these).
On the cylinder: `mass_props` 1.8 ms, closest point 82 µs, ray cast 16 µs, cylinder–sphere
`distance` 0.61 ms, `eval_surface` 0.36 µs. On the 1030-face plate: ray cast 34 ms, closest point
5.4 ms, and `mass_props` **1.8 s** (see the findings).

**Findings and deviations.**
- **Adaptive integration fixes the fixed-order misreads.**
  - The BREP-04 oblique sphere lens is within 2.6e-11 (fixed order: 1.4e-7).
  - The BREP-14 ellipsoid (sphere r 10 scaled (1, 2, 3)) is within **1.1e-9** for volume and
    inertia (fixed order: 4.4e-4). The ellipsoid saturates there: eps 1e-13 gives the same digits.
    OCCT's `VolumePropertiesGK` with spans returns a wrong centroid and inertia, so it is not
    used. S04 SOL-06 Done-when 5 (1e-9) is therefore missed by 0.1e-9.
- **Adaptive `mass_props` is slow on faces with many holes.** The plate (two caps with 1024 hole
  loops each) takes 1.8 s; ordinary parts take milliseconds. eps 1e-10 is no faster and loses
  accuracy (cylinder inertia 8e-10), so the eps stays 1e-12. `Shape::volume` (fixed order) stays
  as the fast path, and S05 runs `mass_props` as a job.
- `amg_core::Ray3` gains `serde::Serialize`/`Deserialize` (additive, in lane D's crate; see
  Interface changes), so S20's `KernelOp` can capture `ray_cast`'s parameters like the others.
- The result types also derive serde (not in the Interfaces), for S20 recording.
- `MassProps` keeps the Interfaces' fields and has no explicit basis field. A shape without solids
  reports `volume = 0` and its faces' (or edges') centroid and inertia, as documented on the type.
  S05's `Volume | Surface | Linear` maps onto it.
- `closest_point` measures to the boundary even for points inside a solid (S05 measure
  semantics). `distance` keeps BRepExtrema's 0 for overlapping or nested solids.
- `iso_curves` and `faces_of_edge` (S05 requests) are not part of BREP-09 as written;
  `Shape::edge_faces` already gives the adjacency.

**Leftovers.**
- A fast `mass_props` path for faces bounded by iso-lines (fixed order there, adaptive elsewhere),
  for parts like the perforated plate.
- The ellipsoid's remaining 0.1e-9 for S04 SOL-06 Done-when 5: B-spline span splitting that keeps
  the inertia, or an exact path for `gtransformed` similarity or affine images of primitives.
- `ray_cast` intersects every face. A bbox or BVH prefilter would help large parts (34 ms on the
  plate).

### 2026-11-25 - BREP-05 history framework
**Landed.**
- `amg-brep`, module `history`:
  - `ShapeHistory::compose(&self, next)` (S20 R8). Relations chain per output: `Kept` then `Kept`
    is `Kept`; `Modified` or `Kept` followed by `Modified` or `Kept` is `Modified`; anything
    involving `Generated` is `Generated`. An input with no image left becomes one `Deleted`
    entry, and a B sub-shape with no entry in `next` counts as deleted. `next`'s operands 1..
    are appended after `self`'s, and so are their `operand_counts`. Roles and tags of `self`
    carry through `Modified`/`Kept` (not `Generated`), and `next`'s win. Output lists come out in
    index order.
  - `ShapeHistory::order_splits(&mut self, &Shape)`: the one place the split order is applied.
    It sorts every `Modified`/`Generated` list that has several outputs of one kind, by kind
    and then by `split_order_key`. The key is `(−q(measure), q(cx), q(cy), q(cz), index)` with
    `q(x) = round(x / 1e-6)` (naming note §4.3 and request Q5; `SPLIT_QUANTUM`, `SplitKey`
    exported for DOC-06's `Bound` order). Fingerprints are computed only for the outputs it
    sorts.
  - `ShapeHistory::unexplained_faces(face_count)` and `range_error(output_counts)` for the
    contract checks.
  - `OpResult::per_solid()` (R-S04-2, naming note Q3): one result per solid in `Shape::solids`
    order. Each history is restricted and reindexed by composing with a `Kept` map of the solid
    (`sub_index_map`), and the split order is applied again.
  - Crate-internal `finish`, run on every op result: the split order, then the completeness
    rule. A face with no source and no role gets `OpRole::Unexplained`, and the result gets
    `KernelWarning::PartialHistory` (naming note Q4).
- Applied to the BREP-03/14/04 ops:
  - Primitives now carry roles, read as sweeps along `frame.normal` (`ops::primitive_roles`):
    the planar face facing −normal is `StartCap`, the one facing +normal is `EndCap`, every
    other face is `Lateral`. Before, they had an empty history.
  - prism, revol, `gtransformed`/`mirrored`, boolean, split and section go through `finish`.
    `split` labels every face `SplitPiece { index }`, where `index` is the face's solid in
    `Shape::solids` order; a face shared by two pieces takes the lower index.
  - `make_wire` keeps tags only (a wire has no faces and no input).
  - Hidden `OcctKernel::unify_coplanar(s)`: the `merge_coplanar` step as an op of its own, with
    history from `s`, for Done-when 4. BREP-10 owns the public `unify_same_domain` with options.
- `amg-brep`, module `query`: `SubShapeGeom { class, measure, centroid, axis, bbox }` and
  `BrepKernel::{sub_geom, sub_geom_all}`. Forwarding arms were added to `RecordingKernel`,
  `api_contract.rs` and `tests/common/fault.rs`. `axis` is the outward normal of a plane face
  (flipped on a reversed face), the axis of a cylinder, cone or torus, the direction of a line,
  or the normal of a circle or ellipse; it is `None` otherwise. `sub_geom_all` spreads the work
  over the current rayon pool: faces in groups of 4, those with the most edges first; edges in
  chunks of 128; vertices in chunks of 512.
- `amg-occt-sys`:
  - New `cpp/occt_history.hxx`, the shared extraction framework:
    - `Recorder` (operand-aware, `Kept`, roles, optional vertex match for revol's canonised
      faces), `Maps`, `Step`, `through`;
    - `history_step` (a `BRepTools_History` step) and `modified_step` (a
      `BRepBuilderAPI_MakeShape` step);
    - `Source` / `source_of(MakeShape | BRepTools_History)`;
    - `record_operands`: `Kept`, else `Modified`/`Generated` through the internal steps, else
      `Deleted`.

    `occt_ops.cpp` and `occt_bool.cpp` use it; their two copies of the recorder are gone.
  - `unify_op` (**C**: `ShapeUpgrade_UnifySameDomain` on a private `BRepBuilderAPI_Copy`,
    whose map is the first step).
  - New `cpp/occt_naming.cpp` with `sub_geoms` and `sub_index_map` (R), and the bridge structs
    `OcSubGeom` and `OcSubMap`.
  - `SHIM_VERSION` 3 → 4, because the op histories changed (it is part of `kernel_version`, so
    cached results are invalidated).
- Tests:
  - `tests/history.rs`: 12 tests.
  - `tests/common/history.rs`: `completeness_error`. Later op WPs add their op to
    `every_output_face_has_a_source_or_a_role` with it.
  - `tests/common/bool_corpus.rs`: the BREP-04 corpus loader, moved out of `tests/boolean.rs`,
    which now uses it.
  - 6 new unit tests in `src/history.rs` (`compose` rules, identity, split key, ranges, serde).
- Bench: `benches/history.rs` (`budget/brep-sub-geom-all-1000`, plus `sub-geom-one-face`,
  `compose-prism-unify`, `order-splits-cut` and `per-solid-cut`).

**Done-when status.** With `--features occt`, `cargo nextest run -p amg-occt-sys -p amg-brep`
passes 267 tests (1 skipped: BREP-04's ignored timing run). Without the feature, 94 pass.
`cargo clippy -p amg-occt-sys -p amg-brep --all-targets --all-features -- -D warnings` is clean,
also without `--all-features`. `cargo build -p amg-brep --no-default-features` builds, and so do
the dependents of `amg-brep` (`amg-doc`, `amg-features`, `amg-subd`, `amg-io/brep`).
1. Met (`every_output_face_has_a_source_or_a_role`). The corpus has 31 BREP-03/14 results and
   131 BREP-04 results, with 1054 output faces in all:
   - BREP-03/14: wires; 9 primitives; prisms of a square, a disc, a holed square, an ellipse
     (tilted), a wire, a slot, an oblique vector, `merge_coplanar`, three tapers, semi-infinite
     and infinite both ways; 4 revols; 2 `gtransformed`; `mirrored`.
   - BREP-04: every non-error case of the 136-case corpus.

   Every face is `Modified`/`Generated`/`Kept` from an input or has a role. Every index is in
   range, operand counts match, and no result has `Unexplained` or `PartialHistory`. Face-build
   tags are in range.
   **Finding, fixed in the shim:** BREP-14 left side faces without a source in two cases:
   - Tapered prisms (4 side faces of a square, the cone face of a disc):
     `BRepOffsetAPI_DraftAngle::Modified` does not list the tilted faces. The step now falls back
     to `ModifiedShape`.
   - Prisms infinite both ways (4 side faces): `BRepPrimAPI_MakePrism::Generated` is empty when
     the generating shape is not used. The record now falls back to `BRepSweep_Prism::Shape`.
2. Met (`prism_laterals_come_from_tagged_profile_edges`). The profile is an L-shape with a
   two-arc hole on a tilted plane: 8 tagged edges. Each of the 8 lateral faces has exactly one
   source, `Generated` from one profile edge of operand 0. Their tags are the 8 profile tags,
   each once. There is one `StartCap` (`Modified` from the face) and one `EndCap` (`Generated`
   from it).
3. Met (`a_split_face_lists_its_pieces_by_area_descending_every_run`). A 10 mm cube is cut by a
   cylinder (r 1) along X at y = 3 on its top face. The top face is `Modified` into
   [60 mm², 20 mm²] (within 1e-9), and it also `Generates` the section edges on it. Over 10 runs
   in one process, the BinTools bytes, the whole history (`Debug`) and the piece order are
   identical.
4. Met (`compose_of_prism_and_unify_equals_the_end_to_end_history`). For each of 5 profiles:
   prism, then `unify_coplanar`, composed and `order_splits`, equals the history of a prism with
   `merge_coplanar` exactly (`PartialEq`). The profiles and their merges:
   - a square with a split side: 7 → 6 faces;
   - an L-shape with two splits: 10 → 8;
   - a rectangle with every side split: 10 → 6;
   - a split side plus a two-arc hole: 9 → 7;
   - a slot with a split arc: 7 → 6.

   The two result shapes have equal `sub_geom_all` in index order. Their BinTools bytes differ,
   since the stand-alone unify runs on a copy.
5. Met (`histories_round_trip_through_serde`, plus unit tests). 33 histories (every corpus op,
   the cut, a tagged wire) round-trip through compact and pretty JSON, including multi-operand
   histories and histories with tags.
6. Met. `budget/brep-sub-geom-all-1000` (release, criterion) on the perforated plate
   (1030 faces, 3084 edges, 2056 vertices) takes **4.16 ms** median. An earlier run at 83 %
   machine load took 5.86 ms, and with `RAYON_NUM_THREADS=1` it takes **14.4 ms**. The test
   profile takes 22 ms; the assertion runs only with `AMG_STRICT_TIMING=1`. One hole-wall face
   takes 9.3 µs.

**Findings and deviations.**
- **Fingerprints do not use `BRepGProp`'s fixed-order integration on analytic faces.**
  - It cost 85 µs per hole-wall face (110 ms of CPU for the plate). `AddOptimal` boxes cost
    another 39 ms for a cap with 1024 holes. The first version took 24.6 ms in release.
  - `sub_geom` now integrates area and centroid itself on plane, cylinder, cone, sphere and
    torus faces. It uses Green's theorem in (u, v) with 8-point Gauss–Legendre along each
    pcurve (one rule per line, per quarter circle or ellipse, per eighth of a span otherwise)
    and along v off planes; iso-u lines are skipped. B-spline and other surfaces fall back to
    fixed-order `BRepGProp`.
  - Boxes come from `BRepBndLib::Add` without the tolerance gap. They are exact on lines,
    circles, planar and analytic faces; B-splines get the pole hull.
  - Checked against the adaptive `mass_props` of each face over 1445 faces of 76 shapes
    (`face_fingerprints_match_adaptive_integration`):
    - Analytic faces: area within **1.0e-6** relative and centroid within 3.5e-7 of the face
      size. The worst case is a fuzzy-glued fuse face whose pcurves sit within its 1e-5 mm gap.
    - B-spline fallback: 3.5e-3 / 9.2e-5 on the `gtransformed` ellipsoid. This is the same
      fixed-order misread as `Shape::face_area` (BREP-14), but deterministic, which is what
      fingerprints need.
- The split order applies to `Generated` lists as well as `Modified` ones (the spec named
  `Modified`), whenever one list has several outputs of one kind: the section edges on a face,
  or one edge sweeping several faces. Mixed-kind lists are sorted by kind first.
- **Primitive histories are no longer empty** (they carry roles). BREP-14's
  `golden_primitives_match_the_analytic_values` asserted `history == default`; it now asserts
  no entries, 6 roles and no warnings. This is the one changed assertion in an existing test.
- `per_solid`: an input whose outputs are all in other solids is `Deleted` in that piece, so
  DOC-06 should not record a tombstone from a piece view alone. `SplitPiece` indices follow
  `Shape::solids`, not the naming note's `PieceSlot`.
- `compose` gives output lists in index order. The kernel's own histories are always in split
  order, because `finish` runs on each of them.
- `capture::SubShapeGeom` (KH-01's own fingerprint: bbox centre, no axis) is unchanged.
  Replacing it with this type would change the failure-dump format, so S20 decides.
- `cargo check --workspace` fails in this worktree in `amg-manifold-sys`: Manifold is not
  installed here, and `amg-app` enables it by default since 44a7852. This is unrelated to
  BREP-05, so the dependents of `amg-brep` were checked instead.

**For DOC-06.** Fixtures to build on, all in `tests/history.rs`:
- `make_box`: roles only;
- the tagged L-shape prism (lateral → edge → tag);
- `grooved_cube` (a cut with a split face in area order);
- the bar cut into two lumps (`per_solid`);
- the cube split by a face (`SplitPiece`).

The key helpers are `split_order_key`, `ShapeHistory::compose` and `order_splits`, and
`sub_geom`/`sub_geom_all`.

**Leftovers.**
- A Gauss–Green path for B-spline faces (split at knot spans) to remove the 3.5e-3 fallback
  error.
- S20: move `capture::SubShapeGeom` to the S02 type.
- Every later op WP adds its op to the completeness test (`completeness_error`).

### 2026-11-25 - BREP-06 fillets, chamfers and the max-radius probe
**Landed.**
- `amg-brep`, trait `BrepKernel` (with the forwarding arms in `RecordingKernel`,
  `tests/api_contract.rs` and `tests/common/fault.rs`):
  - `fillet(s, &[(EdgeId, FilletSpec)], &FilletOpts, ctx)`, `chamfer(s, &[(EdgeId, ChamferSpec)],
    ctx)` and `fillet_max_radius(s, edges, hi, iterations, ctx)`, with the Interfaces
    signatures.
  - S20's `KernelOp::{Fillet, Chamfer, FilletMaxRadius}`. Edges are `EdgeRecord`s (index plus
    fingerprint), so `reindex` covers them as it covers `ProjectEdges`. The reference faces of
    asymmetric chamfers stay plain `FaceId`s.
- New module `blend` (plain Rust, unit-tested without OCCT):
  - Validation: a non-empty, duplicate-free edge list; each edge between two distinct faces;
    finite positive radii and distances; a variable law of ≥ 2 samples, strictly increasing,
    from t = 0 to t = 1 (ends snapped within 1e-9); a reference face that is one of the edge's
    faces; an angle in (0, π/2); approximation overrides finite and positive.
  - The chord conversion `r = c / (2 sin(φ/2))`. Here `φ` is the angle between the two faces'
    outward normals at the middle of the edge (new R shim `edge_face_angle`). This is exact for
    edges of constant dihedral angle.
  - The mapping of the shim status to `InvalidInput` or `LocalOpFailed`.
- `fillet_max_radius` bisects. It returns `hi` when `hi` works. Otherwise it runs `iterations`
  halvings of (0, hi]; a radius "works" when the build succeeds and the result is
  BRepCheck-valid, whatever the validation policy. The probes skip the history. When no tried
  radius works, it returns `LocalOpFailed { op: "fillet_max_radius" }` with the last failure's
  edges. Cancellation is checked between steps.
- `amg-occt-sys`, new `cpp/occt_blend.cpp`:
  - `blend_op` (**C**): `BRepFilletAPI_MakeFillet` / `_MakeChamfer` on a private
    `BRepBuilderAPI_Copy`. Edges are added in the caller's order, and OCCT numbers the contours
    in that order. An edge that tangent propagation already put in a contour is accepted when
    its spec equals the contour's first spec, and refused (`InvalidInput`) otherwise.
  - `FilletOpts` overrides go through `SetParams`. The other `SetParams` values are ChFi3d's
    own defaults, read from a `ChFi3d_FilBuilder` subclass on the same shape (they are protected
    members with no getters).
  - Bridge structs `OcBlendEdge` (variable laws in one shared `f64` slice) and
    `OcFilletParams`. `OcOp`/`OcOpHistory` get `role_args` (the contour of a blend face); every
    `Recorder` role pushes 0 there.
  - Role codes 3 `FilletBlend`, 4 `ChamferFace`, 5 `CornerPatch`.
- History (operand 0 = the input):
  - Every input sub-shape goes through the copy, then `Modified` (or itself when it passes
    through).
  - Blended edges are `Generated` into their blend faces, and vertices into the corner faces
    OCCT attributes to them.
  - A new face generated by an edge of contour `ic` (the propagated edges included) gets
    `FilletBlend { contour: ic }` or `ChamferFace`. Every other face that no input face became
    gets `CornerPatch`.
- Failures:
  - `offending` lists the input edges of `NbFaultyContours` contours and of the requested edges
    at `NbFaultyVertices` vertices.
  - When OCCT names none (always for chamfers, whose API hides its builder), each contour is
    built on its own, and the ones that fail are offending. When none fails alone, every edge
    is offending.
  - `message` joins the stripe statuses (`contour 0: ChFiDS_StartsolFailure`), the corner
    failures and the OCCT exception.
  - `partial` is `BadShape()` when `HasResult()`, else the done-but-broken shape (see below),
    else `None`.
- Tests:
  - `tests/blend.rs`: 10 tests.
  - `tests/common/blend_corpus.rs`: 21 cases. Analytic volumes use Pappus and section
    integrals: spandrel `(1 − π/4) r²` with its centroid at `r (10 − 3π)/(12 − 3π)` from either
    face; the rounded cube by Steiner's formula; the top-loop band by integrating its inset
    square section.
  - 21 goldens `tests/golden/brep/blend_*.toml` (adaptive `mass_props` volume and area,
    topology counts and face classes, rel 1e-6).
  - Stress ops `Fillet`/`Chamfer` in `tests/common/stress.rs`, with 4 stress tests in
    `tests/stress.rs`.
  - The corpus joins `every_output_face_has_a_source_or_a_role` (`tests/history.rs`).
  - 5 unit tests: 4 in `src/blend.rs` and 1 in `src/ops.rs` for the roles.
- Bench: `benches/blend.rs` (`brep-blend/*`).

**Done-when status.** With `--features occt`, `cargo nextest run -p amg-occt-sys -p amg-brep`
passes 286 tests (1 skipped: BREP-04's ignored timing run). Without the feature, 99 pass.
`cargo clippy -p amg-occt-sys -p amg-brep --all-targets --all-features -- -D warnings` is clean,
also without `--all-features`. `cargo build -p amg-brep --no-default-features` builds.
1. Met (`golden_fillets_and_chamfers_match_the_reference_volumes`). The 21 cases:
   - cube edge, tilted box edge, long box edge;
   - the 4-edge top loop (mitred corners), the 3-edge vertex blend (sphere corner) and all 12
     edges;
   - the slot's 4-edge tangent chain from one edge;
   - variable linear and variable 3-sample laws, chord, cylinder rim, concave edge of an
     L-block, mixed radii at one vertex, `QuasiAngular`;
   - chamfers: equal, two distances, distance-angle, top loop, 3-edge vertex, cylinder rim and
     slot chain.

   All match their goldens within 1e-6. The 17 with closed forms match them within 6.1e-16,
   except the fillet top loop at 7.2e-11 (its mitred corners are intersection curves). The 4
   golden-only cases are both variable laws, the mixed radii and the 3-edge chamfer vertex. The
   chamfer vertex removes 54.667 mm³, against 54 mm³ for three mutually trimmed wedges: OCCT
   adds a triangular corner face.
2. Met (`a_radius_larger_than_the_adjacent_face_fails_on_that_edge`). The plate is 20 × 20 × 4.
   - The front top edge with r = 6 gives `offending == [that edge]`, with message
     `contour 0: ChFiDS_StartsolFailure`.
   - Together with two workable edges that do not touch it (r = 1), only that edge is offending
     (`contour 1: ...`).
   - The same holds for a chamfer (d = 6 next to d = 1, found by the per-contour probe) and for
     a variable law of 5 → 7.
3. Met (`fillet_max_radius_is_within_one_percent_of_the_largest_workable_radius`). 20 steps are
   compared with a downward scan in steps of `hi`/1000:
   - plate front top edge 3.999996 vs 3.990 (geometric limit 4);
   - two plate top edges 3.999996 vs 3.990;
   - 3-edge cube vertex 9.999996 vs 9.996 (limit 10);
   - slot chain 2.999992 vs 2.992 (limit 3, the slot radius);
   - rim of a 3 mm high cylinder 2.999992 vs 2.992 (limit 3).

   The worst difference is 0.27 %, the scan's grid step. Each answer fillets successfully. A
   workable `hi` comes back bit-exact; NaN `hi`, 0 iterations and a cancelled job are refused.
4. Met (`every_blend_face_maps_to_its_source_edge`). Over all 21 cases (49 blend and chamfer
   faces), every blend face has its role and is `Generated` from an input edge, and every
   requested edge generates a blend face. In the single-edge case the edge generates exactly
   the one `FilletBlend { contour: 0 }` face, and the result has 7 faces.
5. Met (`ten_runs_are_byte_identical`). Each corpus case run 10 times gives identical BinTools
   bytes and history (`Debug`), and the input's bytes are unchanged.
6. Met (`shared_{box, filleted_box, cylinder, plate}_survives_concurrent_blends`). 8 threads
   fillet (r 0.5) and chamfer (d 0.4) up to 12 edges of the shared shape itself:
   - box: 120 iterations;
   - filleted box: 40 iterations. Every op is an error, because all its edges are smooth (see
     below);
   - cylinder: 80 iterations;
   - plate: 2 edges, 2 iterations.

   There are 0 mismatches, and the BinTools hashes are unchanged (box `e40ca138fde6fcd8`,
   filleted box `cad3beeac5e9c130`, cylinder `4a534faf4196f65e`, plate `42a46e2564f1eef4`).

The history completeness test now covers 21 BREP-06 results as well (1235 output faces in all),
with no `Unexplained` face. `random_radii_give_valid_results_or_clean_errors` (S02 Testing) runs
200 random edge sets with radii and distances of 0.2–7.2 mm on a 12 × 8 × 5 box: 106 valid
results and 94 `LocalOpFailed`, each naming only requested edges.

**Bench** (release, criterion, validation off; S02 has no fillet budget):

| Bench | Median |
|---|---|
| `fillet-cube-edge` | 1.05 ms |
| `fillet-cube-12-edges` | 6.53 ms |
| `chamfer-cube-edge` | 1.05 ms |
| the 21-case corpus | 55.1 ms |
| `max-radius-cube-edge-20` | 29.8 ms |
| `fillet-plate-1030-one-edge` | 110.7 ms |

The plate's cost grows with the face count of the whole shape, not with the blend (one edge
costs 106 times the cube's). The share of the private copy was not measured.

**Findings and deviations.**
- **The blend runs on a private copy (category C)** as a precaution, as the sweeps do; a
  copy-free run (which could report untouched sub-shapes as `Kept`) was not tried. Untouched
  faces and edges therefore come back `Modified` 1:1 (new TShapes), not `Kept`.
- **`IsDeleted` is not used.** `BRepFilletAPI_MakeFillet::IsDeleted` reports every sub-shape
  outside its own map as deleted, including vertices that pass through unchanged. A copy that
  is not in the result simply has no relation.
- **ChFi3d can report done on a bad blend.** A variable law that outgrows the face only near
  one end (1 → 7 mm on the 4 mm face) is reported done. The result has the topology of an
  ordinary fillet, and BRepCheck accepts it. But its volume is 1517.6 mm³, below the 1531.3 mm³
  that a blend lying within the face can reach (an r = 4 fillet along the whole edge), so the
  blend self-intersects. The test logs this case and does not assert it. S20's
  self-interference check (R5) is the guard for it. The shim also treats a done result with a
  different solid count from the input as a failure (in the op and in the per-contour probe);
  no corpus or random case has hit that rule.
- **Smooth edges are refused.** OCCT ignores `Add` on an edge between tangent faces; such an
  edge is reported as `InvalidInput` naming the edge. This is why every edge of the filleted-box
  fixture is refused.
- **A plane–plane constant fillet is an exact cylinder for every `FilletShape`** (ChFiKPart).
  `opts_reach_the_blend` therefore checks the blend shape and the approximation overrides on a
  variable law: both change the blend surface, and the volume stays within 1e-3.
- `Variable` law parameters are relative along the whole contour (OCCT's spine), not along the
  one edge. They are the same for an isolated edge; the trait docs say so.
- `TwoDistances`/`DistanceAngle` on a tangent chain use the first edge's spec and face for the
  whole contour. Specs on later edges of the chain must have equal values; their faces are not
  compared.
- `SHIM_VERSION` stays 4: no existing op's history changed (`role_args` is new data only).

**For DOC-06.** Fillet histories are one-operand, with roles carrying the contour index, in the
caller's edge order. The fixtures are in `tests/common/blend_corpus.rs` (cube edge, vertex blend,
slot chain).

**Leftovers.**
- Mid-op cancellation: ChFi3d takes no progress range, so cancellation is checked before and
  after the call only.
- A face-fingerprint `reindex` for the reference faces of asymmetric chamfers in `KernelOp`.
- KH-03 (S20) can build its fillet strategy chain on `fillet_max_radius` and `FilletOpts`.

### 2026-11-25 - BREP-07 shell, offset and thicken
**Landed.**
- `amg-brep`, trait `BrepKernel` (with the forwarding arms in `RecordingKernel`,
  `tests/api_contract.rs` and `tests/common/fault.rs`):
  - `shell(s, remove, &ShellOpts, ctx)`, `offset(s, distance, join, tol, ctx)` and
    `thicken(s, thickness, join, ctx)`, with the Interfaces signatures.
  - S20's `KernelOp::{Shell, Offset, Thicken}`. Removed and per-face faces stay plain `FaceId`s
    (as BREP-06's chamfer reference faces).
- New module `offset` (plain Rust, unit-tested without OCCT): validation and lowering.
  - `shell`: exactly one solid; a finite thickness longer than the linear tolerance (either
    sign); removed faces distinct, in range and not all of them; per-face thicknesses distinct,
    not on a removed face, of the shell's sign.
  - `offset`: any shape with faces. `thicken`: a face, a shell or a compound of them, without
    solids. `tol` finite and ≥ 0, where 0 means the kernel's linear tolerance.
  - The mapping of the shim status to `InvalidInput`, `LocalOpFailed` or `Cancelled`.
- `amg-occt-sys`, new `cpp/occt_offset.cpp` with `offset_op` (**C**): `BRepOffset_MakeOffset`
  used directly, on a private `BRepBuilderAPI_Copy`. It is the algorithm behind
  `BRepOffsetAPI_MakeThickSolid` / `_MakeOffsetShape`; used directly, it gives
  `SetOffsetOnFace`, thickening and a progress range.
  - `shell` with removed faces runs `MakeThickSolid` (`AddFace` per removed face).
  - `shell` without removed faces runs `MakeOffsetShape`, then builds one solid from the
    input's shell and the offset shell, with the material between them (R-S04-8). This is
    needed because `MakeThickSolid` with no closing face returns the offset skin alone: the
    inner box, or an outward skin of negative volume. The input must then have one shell.
  - `offset` runs `MakeOffsetShape`. `thicken` runs `MakeOffsetShape` with `Thickening = true`.
  - Mode `Skin`, no self-intersection, `RemoveIntEdges` off. Join `GeomAbs_Arc` or
    `GeomAbs_Intersection`.
  - Bridge struct `OcOffsetParams`; role codes 6 `ShellInner`, 7 `OffsetFace`.
- **Collapse check** (before the build): every face that is offset is sampled on a 9 × 9 grid
  of cell centres. `BRepLProp_SLProps` gives both principal curvatures `k` (signed towards the
  surface normal; the offset `d` is flipped on reversed faces). A scale factor `1 − d·k < 1e-7`
  anywhere means the offset surface folds, and that face is offending. OCCT does not refuse
  these offsets: it builds folded or empty results.
- **Canonical output order.** `BRepOffset_MakeOffset` collects faces in maps hashed by TShape
  address, so the face order changed on every run of `MakeOffsetShape`. With it changed the edge
  order, the sub-shape indices and the BinTools bytes (5 of 5 reruns differed).
  - The shim rebuilds every shell (and compound) of the result with its faces in key order:
    area largest first, then the centroid lexicographically. The faces themselves are kept.
  - It rewrites each edge's `BRep_CurveOn2Surfaces` regularity records in face-index order.
    OCCT stores each face pair in hash order (`G1 2 0 1 0` vs `G1 1 0 2 0`), and the bytes keep
    that order. A seam's `BRep_CurveOnClosedSurface` record is left alone.
  - The edges are private to the op (copies or new), so they are changed in place.
- History (operand 0 = the input), via the copy:
  - **shell / thicken**: the input's faces, edges and vertices are `Modified` 1:1 into their
    copies, which stay in the result. Every OCCT image (`Modified` ∪ `Generated` of the copy) is
    `Generated`: offset faces from their face, arc pipes from edges, arc spheres from vertices,
    and the rim ring from the removed face.
  - **offset**: images of the input's own kind (offset faces, offset edges) are `Modified`,
    cross-kind images (arc faces) `Generated`.
  - New faces that are no image (the side walls of a thickened face) are `Generated` from the
    input edges on them.
  - Roles: the offset skin gets `ShellInner` (shell) or `OffsetFace` (offset, thicken). The skin
    is the images of offset faces, plus those of edges and vertices not on a removed face. Every
    other new face (the rim at a removed face, thicken's side walls) is `Lateral`.
  - Removed faces have no `Modified` relation.
- Failures:
  - `LocalOpFailed { op }` names the faces the collapse check flags. Its message gives each
    face's radius (`face 2: offset -4 mm reaches its centre of curvature (radius 3 mm)`).
  - Otherwise it carries OCCT's `BRepOffset_Error` name (`BRepOffset_NotConnectedShell`, ...)
    and the faces of `GetBadShape()`, which may be none.
  - A build that gives no solid when one is needed is also a failure. It keeps the result as
    `partial`.
- Cancellation goes through the BREP-12 `ProgressSink` into `MakeThickSolid`/`MakeOffsetShape`.
  It is also checked before and after the call.
- Tests:
  - `tests/offset.rs`: 11 tests.
  - `tests/common/offset_corpus.rs`: 20 cases. The closed forms are box and cylinder
    differences, rounded-rectangle prisms, and Steiner's formula for the arc-joined box and
    cylinder (`V + S t + M t² + 4π t³/3`, with `M = π(a+b+c)` for the box and `πh + π²r` for the
    cylinder). The thickened B-spline face is checked against a 24 × 24 Gauss quadrature of
    `det(S_u + sN_u, S_v + sN_v, N)` over `s ∈ [0, t]`.
  - 20 goldens `tests/golden/brep/{shell,offset,thicken}_*.toml` (named after the case) (adaptive `mass_props` volume and area,
    topology counts and face classes, rel 1e-6).
  - The corpus joins `every_output_face_has_a_source_or_a_role` (`tests/history.rs`).
  - Unit tests: 2 in `src/offset.rs`, 1 in `src/ops.rs` (role codes). The `KernelOp`
    round-trip test has the 3 new variants.
- Bench: `benches/offset.rs` (`brep-offset/*`).

**Done-when status.** With `--features occt`, `cargo nextest run -p amg-occt-sys -p amg-brep`
passes 300 tests (1 skipped: BREP-04's ignored timing run). Without the feature, 102 pass.
`cargo clippy -p amg-occt-sys -p amg-brep --all-targets --all-features -- -D warnings` is clean,
also without `--all-features`. `cargo build -p amg-brep --no-default-features` builds.
1. Met (`golden_shells_offsets_and_thickens_match_the_reference_volumes`). All 20 cases are
   valid single solids that match their goldens within 1e-6. 19 have a reference volume:
   - The shelled box (top removed, 1 mm inward) matches `abc − (a−2)(b−2)(c−1)` exactly
     (0.0). So do the intersection join, the hollow inward box, the hollow outward box with
     intersection join, the tube (two faces removed) and the per-face case (1, 2 and 3 mm walls).
   - The shelled cylinder matches within 2.7e-16 and the shelled filleted box (4 vertical
     fillets r = 3, arc join, inner fillets r = 2) within 4.5e-16.
   - The arc-joined outward hollow box matches within 2.7e-11 and the arc-joined offset box
     within 8.2e-12. These are the adaptive integration error of the corner spheres.
   - The thickened planes, the cylinder patch and the offset cylinder match within 1.4e-15.
   - The thickened B-spline face (bicubic Bézier, 3 mm bump, t = 1) matches the quadrature
     within 1.4e-8 (bound 0.5 %).
   - The outward shell with a removed face is golden-only.
2. Met (`offsets_of_a_sphere_keep_its_centre_and_radius`). Offsets of d = +2, +0.5, −0.5, −2
   and −4.5 of the r = 5 sphere, with both joins, give one spherical face. Its radius is within
   1e-9 relative of `5 + d`, its centre within 1e-9 of the origin, and the volume within 1e-9.
3. Met (`a_thickness_beyond_the_radius_fails_on_those_faces`).
   - Cylinder r = 6, top removed, 7 mm wall: `offending == [the lateral face]`, with
     "radius 6" in the message. 5.5 mm works. The per-face path also fails on that face only.
   - Filleted box, 4 mm inward wall: exactly the 4 fillet faces ("radius 3"). Outward works.
   - Sphere r = 5, offsets −5 and −6, both joins: `[face 0]`.
   - Cylinder patch r = 10 thickened by −12: `[face 0]`; −9 works.
4. Met (`history_roles_follow_the_offset_skin`), on 6 cases:
   - Box with top removed: 5 `ShellInner` faces, each `Generated` from one input face 1 mm away;
     1 `Lateral` rim from the removed face. The 5 kept faces are `Modified` 1:1 on the same
     planes. The removed face is not `Modified`.
   - Per-face shell: each inner face lies at its own face's thickness.
   - Hollow outward box with arcs: 26 `ShellInner` faces, generated from 6 faces (planes),
     12 edges (cylinders) and 8 vertices (spheres).
   - Offset box with arcs: 26 `OffsetFace` faces, 6 planes `Modified` from faces 1 mm away, 12
     generated from edges, 8 from vertices.
   - Sphere offset: the face `Modified` into the `OffsetFace`.
   - Thickened plane: 1 `OffsetFace` from the face, 4 `Lateral` walls each `Generated` from one
     boundary edge (edges 0–3).

   Across the whole corpus every skin face has a source, every other new face is `Lateral`, and
   every face of an offset result is `OffsetFace`.

Also tested:
- `ten_runs_are_byte_identical`: each corpus case, 10 runs, identical BinTools bytes and history
  (`Debug`); the input's bytes are unchanged.
- `shared_shapes_survive_concurrent_offsets`: 8 threads run 40 corpus ops each on the shared
  inputs. Every result has the single-thread bytes, and the inputs are unchanged.
- `a_shell_without_removed_faces_is_hollow`: R-S04-8, classified void and wall points.
- `thicken_follows_the_normal`: +t goes along the face normal.
- `bad_inputs_are_invalid_input`: 14 refusals. A compound of two loose solids fails as a whole
  with `BRepOffset_NotConnectedShell`.
- `cancelled_jobs_do_not_run`.
- `cancel_stops_a_running_offset`: a cancel 200 ms into the offset of the 1030-face plate returns
  `Cancelled` after 0.47 s, against 12.2 s for the full run in release. The bound is asserted
  only under `AMG_STRICT_TIMING=1`.

The history completeness test now covers 20 BREP-07 results as well (1450 output faces in all),
with no `Unexplained` face.

**Bench** (release, criterion, validation off; S02 has no shell/offset budget):

| Bench | Median |
|---|---|
| `shell-box-top` | 4.60 ms |
| `shell-box-hollow-arc` | 10.4 ms |
| `offset-box-arc` | 7.18 ms |
| `offset-sphere` | 0.32 ms |
| `thicken-bspline-face` | 6.80 ms |
| the 20-case corpus | 71.4 ms |
| `offset-plate-1030` (0.1 mm, intersection) | 12.2 s |

The share of the collapse check and of the canonical rebuild was not measured separately. The
plate's 12 s is OCCT's offset of 1030 faces: an S04 press-pull on large parts goes through
`offset_faces` (R-S04-5, BREP-16), not this op.

**Findings and deviations.**
- **`MakeThickSolid` without closing faces is not a hollow solid** (see above). The shim builds
  the hollow solid itself from the two shells. This needs an input with one shell
  (`InvalidInput` otherwise).
- **`BRepBuilderAPI_Copy::Modified` drops the occurrence's orientation**: it returns the clone
  in an orientation of its own. A reversed face's copy came back forward, so 2 of the 4 fillets
  had the wrong curvature sign. The offset shim's copy map puts the key's orientation back on
  the clone. BREP-06's `occt_blend.cpp` uses its copies only as identities (map keys, a chamfer's reference face), so it is not affected.
- **BRepOffset is not deterministic in order** (see the canonical order above). BREP-06's
  ChFi3d was, so this is the first op that canonicalises (S02 §7).
- **The rim of a removed face is the removed face's own OCCT image**: a thin ring for an inward
  wall. It is `Generated` from that face with role `Lateral`, not `ShellInner`.
- **Offset histories call a moved face `Modified`**, while shell and thicken call the offset face
  `Generated`, since the original stays. This matters for DOC-06: a feature that renames on
  `Modified` keeps the face's name through an offset, not through a shell.
- **Disconnected compounds** cannot be offset (`BRepOffset_NotConnectedShell`); `offset` and
  `thicken` take connected inputs only.
- The collapse check samples 81 points per face. A B-spline whose highest curvature falls
  between samples can slip through, and OCCT then builds what it builds. S20's validation gate
  (the R5 self-interference check) is the backstop.
- `SHIM_VERSION` stays 4: no existing op's history changed. The new role codes 6 and 7 are new
  data only.

**For DOC-06.** Shell, offset and thicken histories are one-operand. Removed faces have no
`Modified` relation. The skin roles separate the inner wall (`ShellInner`) from the kept outer
faces, which are `Modified` 1:1. The fixtures are in `tests/common/offset_corpus.rs`.

**Leftovers.**
- A face-fingerprint `reindex` for `KernelOp::Shell`'s removed and per-face faces (as for
  BREP-06's chamfer faces).
- Offsetting disconnected compounds part by part, if a feature needs it.
- `offset_faces` (R-S04-5) stays with BREP-16.

### 2026-11-25 - BREP-10 healing, unify, sewing, make solid, compounds and the full check
**Landed.**
- `amg-brep`, trait `BrepKernel` (with the forwarding arms in `RecordingKernel`,
  `tests/api_contract.rs` and `tests/common/fault.rs`):
  - `heal(s, &HealOpts, ctx)`, `unify_same_domain(s, &UnifyOpts)`, `sew(faces, &SewOpts, ctx)`
    and `make_compound(parts)` with the Interfaces signatures.
  - **New:** `make_solid(shell) -> KResult<OpResult>` (§4 names it; it was not in the trait).
    `sew` with `make_solid` uses the same step.
  - `check(s, &CheckOpts)` now honours `self_interference` (below).
  - S20's `KernelOp::{Heal, UnifySameDomain, Sew, MakeSolid, MakeCompound}` with replay arms and
    the round-trip test.
  - `impl Default` for `HealOpts` (tol 1e-6, small edges and orientation fixed, `max_tol`
    1e-3), `UnifyOpts` (exactly `merge_coplanar`'s options: faces, edges, no B-spline
    concatenation, 1e-7 mm, 1e-12 rad) and `SewOpts` (1e-6, manifold, with solids).
- New module `heal` (plain Rust, unit-tested without OCCT): validation, lowering and the
  failure mapping (`invalid: ` → `InvalidInput`, otherwise `LocalOpFailed` with `partial`).
- `amg-occt-sys`, new `cpp/occt_heal.cpp`:
  - `heal_op` (**C**): `BRepBuilderAPI_Copy` with geometry (S02 §9 `copyGeom`), then
    `ShapeFix_Shape` (precision `tol`, max tolerance `max_tol`, min tolerance
    `min(tol, 1e-7)`, orientation modes off with `fix_orientation: false`) with a progress range,
    then `ShapeFix_Wireframe` (`FixWireGaps`, `FixSmallEdges` with `ModeDropSmallEdges`). Each
    has its own `ShapeBuild_ReShape`, and their `BRepTools_History` are the two history steps
    after the copy.
  - `unify_same_domain_op` (**C**): as BREP-05's `unify_op`, with the options.
  - `sew_op` (**C**): `BRepBuilderAPI_Sewing` (sewing, degenerated-face analysis, cutting,
    `non_manifold`) on one copy per input (geometry copied too), progress and cancel inside
    `Perform`. The history step is the sewing's `ModifiedSubShape` / `Modified`, an image
    exploded to the input's kind (an edge cut into sections maps to each).
  - `make_solid_op` (**C**): on a copy, every closed shell (and every face that closes on
    itself, a sphere or a torus) becomes a solid through `ShapeFix_Solid::SolidFromShell`
    (`MakeSolid`, reversed when the infinite point classifies inside).
  - `make_compound_of` (**N**), `analyze_shape` (**C**, `BOPAlgo_ArgumentAnalyzer` on a copy:
    self-interference, small edges, curve on surface; findings mapped back to the caller's
    indices), `op_fix_count` (**R**); bridge structs `OcHealParams`, `OcUnifyParams`,
    `OcSewParams`; `OcOp::fixes`. `SHIM_VERSION` stays 4 (no existing op changed).
- `check` with `self_interference`: the BRepCheck report plus the analyzer's findings under
  their full names (`BOPAlgo_SelfIntersect` on each interfering sub-shape,
  `BOPAlgo_TooSmallEdge`, `BOPAlgo_InvalidCurveOnSurface`, ...); any finding makes it invalid.
  Without the flag `check` is BRepCheck only, as before, and so is the validation policy.
- Warnings: `heal` gives `Healed { fixes }` (copied sub-shapes that the fixes replaced, removed
  or split, or whose tolerance grew, whose `SameParameter`/`SameRange` flag changed or whose
  face orientation flipped) and `ToleranceGrown`; `sew` gives `ToleranceGrown`.
- Defect corpus `testdata/brep/defects/*.brep` (17 text files, plain git, not LFS), made by the
  new `gen_brep_defects` example (`fixture-gen`; builders `defect_*` in
  `cpp/occt_fixtures.cpp`). Each is a box, cylinder or square face with one defect made by
  construction: a vertex moved off its edges, an edge's 3D line moved off its faces, a missing
  pcurve, a pcurve moved off its edge, a reversed lateral face, a reversed box face, the
  SameParameter / SameRange flags cleared, wire gaps, a wire out of order, a reversed wire, an
  inside-out solid, an edge tolerance above its vertices', a 3e-7 mm edge, a bow-tie, an open
  solid, two overlapping boxes. The generator checks that every defect survives the text
  round trip.
- Tests: `tests/heal.rs` (14 tests), `tests/common/heal_corpus.rs` (the corpus, 20 parts, the
  gapped cube, the coplanar fuse), `tests/common/defects.rs`; 12 goldens
  `tests/golden/brep/{heal,sew,unify}_*.toml`; the corpus joins
  `every_output_face_has_a_source_or_a_role` (79 BREP-10 results); 3 stress tests
  (`shared_{box,filleted_box,cylinder}_survives_concurrent_healing`: heal, unify, sew of the
  shared shape's own faces, make_compound, the full check, 8 threads); 3 unit tests in
  `src/heal.rs`.
- Bench `benches/heal.rs` (`brep-heal/*`).

**Done-when status.** With `--features occt`, `cargo nextest run -p amg-occt-sys -p amg-brep`
passes 320 tests (1 skipped: BREP-04's ignored timing run). Without the feature, 105 pass.
`cargo clippy -p amg-occt-sys -p amg-brep --all-targets --all-features -- -D warnings` is clean,
also without `--all-features`. `cargo build -p amg-brep --no-default-features` builds, and
`cargo check` of the dependents (`amg-doc`, `amg-features`, `amg-io`, `amg-nurbs`, `amg-subd`,
`amg-ui`) passes.
1. Met (`sewing_a_gapped_cube_gives_a_closed_solid`). Six planar polygons of the 10 mm cube,
   each moved 1e-4/√2 mm along its own axis, so neighbouring edges are 1e-4 mm apart and no
   vertex is shared (checked: nearest corners 1e-4 apart). Sewing at 1e-3 mm gives one valid
   solid with 6 faces, 12 edges and 8 vertices, volume 1000 within **2.3e-16** relative, max
   tolerance 1.05e-4 and a `ToleranceGrown` warning. Each face is `Modified` 1:1, each output
   edge has two input edges. At 1e-5 mm the sewing stays open and `LocalOpFailed` names the free
   edges, with the shell as `partial`.
2. Met (`sewing_exploded_parts_gives_them_back`). 20 parts: 8 closed fixtures (box, cylinder,
   cone, sphere, torus, filleted box, located compound, mirrored cylinder) and 12 built ones
   (tilted box, half cylinder, full cone, quarter torus, sphere, L prism, holed prism on a tilted
   plane, quarter and full revol, a grooved cube (cut), a cube with a bar (fuse), a box with one
   fillet). Each is exploded twice: `sub_shape` faces (still sharing edges) and each face copied
   apart into new geometry. All 40 sews give valid shapes with the part's solid count and face
   count, volume within **2.2e-16** relative, and every face operand `Modified` into exactly one
   distinct face.
3. Met (`heal_fixes_the_defect_corpus`). 10 defect fixtures, each BRepCheck-invalid before and
   valid after `heal` (tol 1e-4, max_tol 1e-3), same kind, solids within 1e-6 of the part they
   were made from, complete history, a `Healed` warning:

   | Fixture | BRepCheck before | Fixes | Max tolerance after |
   |---|---|---|---|
   | `vertex_off_edges` | InvalidPointOnCurve | 11 | 5.2e-5 |
   | `edge_off_faces` | InvalidCurveOnSurface, InvalidPointOnCurve, InvalidSameParameterFlag, UnorientableShape | 3 | 3.5e-5 |
   | `missing_pcurve` | NoCurveOnSurface, SelfIntersectingWire, UnorientableShape | 1 | 1e-7 |
   | `pcurve_off_edge` | InvalidCurveOnSurface, InvalidSameParameterFlag, UnorientableShape | 2 | 4.0e-5 |
   | `reversed_lateral` | BadOrientationOfSubshape | 1 | 1e-7 |
   | `reversed_face` | BadOrientationOfSubshape | 1 | 1e-7 |
   | `same_parameter_off` | InvalidSameParameterFlag | 2 | 1e-7 |
   | `same_range_off` | InvalidSameParameterFlag, InvalidSameRangeFlag | 1 | 1e-7 |
   | `wire_gaps` | NotConnected, SelfIntersectingWire, UnorientableShape | 13 | 1.5e-5 |
   | `reversed_wire` | BadOrientationOfSubshape | 1 | 1e-7 |

   Also `heal` makes `bow_tie` valid (it splits the face) and `open_solid` valid (it gives up
   the solid and returns the open shell); neither is counted.
4. Met (`unify_after_fusing_coplanar_boxes_gives_six_faces`). Boxes `[0,10]` and `[10,20]` along
   x fuse into 10 faces; `unify_same_domain` gives 6 faces, 12 edges, 8 vertices and volume 2000.
   Its history maps the 8 side pieces 2:1 into the 4 merged faces and the 2 ends 1:1. Composed
   with the fuse (`ShapeHistory::compose`), each box reaches 5 block faces; each merged face has
   one source in each box. With the default options the result and history equal
   `unify_coplanar`'s exactly. With `edges: false` the faces merge and the collinear edges stay
   split.
5. Met (`check_lists_the_expected_problem_kinds`), on 8 broken fixtures, the exact kind list
   and the defect's own sub-shape:
   - BRepCheck: `open_solid` [NotClosed]; `reversed_face` [BadOrientationOfSubshape];
     `same_range_off` [InvalidSameParameterFlag, InvalidSameRangeFlag] on edge 0;
     `vertex_off_edges` [InvalidPointOnCurve] on vertex 0; `missing_pcurve` [NoCurveOnSurface
     on edge 0, SelfIntersectingWire, UnorientableShape]; `bow_tie` [SelfIntersectingWire on
     face 0, UnorientableShape].
   - With `self_interference`: `bow_tie` adds `BOPAlgo_SelfIntersect` on edges 0 and 2;
     `overlapping_boxes` (BRepCheck-valid) gives `BOPAlgo_SelfIntersect` on faces of both boxes;
     `small_edge` (BRepCheck-valid) gives `BOPAlgo_TooSmallEdge` on edge 0 and
     `BOPAlgo_SelfIntersect` on its two vertices.
   - Six valid fixtures stay valid with the analyzer on.

Also tested: `ten_runs_are_byte_identical` (every corpus op, 10 runs, identical BinTools bytes
and history, inputs unchanged); `sewn_and_made_solids_face_outwards` (the faces of an inside-out
box, volume −6000, sew and make solid into +6000; two shells give a compound of two solids);
`healing_a_valid_shape_changes_nothing`; `heal_options_switch_their_fixes`
(`fix_orientation: false` leaves the reversed face invalid; the 3e-7 mm edge merges only with
`fix_small_edges`); `bad_inputs_are_invalid_input`; `make_compound_shares_its_parts`;
`cancelled_jobs_do_not_run`; `cancel_stops_a_running_sew` (the 1030 plate faces apart: 2.3 s in
the test profile, a cancel 100 ms in returns `Cancelled` 4.6 ms after the cancel; the 500 ms
bound only under `AMG_STRICT_TIMING=1`); `sew_takes_faces_of_one_shape_and_its_copy`.

**Bench** (release, criterion, validation off; S02 has no healing budget):

| Bench | Median |
|---|---|
| `heal-wire-gaps` | 0.43 ms |
| `heal-filleted-box` | 7.99 ms |
| `check-full-filleted-box` (analyzer on) | 84.2 ms |
| `unify-coplanar-fuse` | 1.24 ms |
| `sew-gapped-cube` | 2.12 ms |
| `sew-plate-1030-apart` | 259 ms |
| `make-solid-plate-1030` | 47.5 ms |
| `make-compound-two-plates` | 0.78 µs |

**Findings and deviations.**
- **`TopoDS_Builder::Add` freezes the component** (clears its TShape's `Free` flag, which BinTools
  writes). A compound holding the caller's shapes therefore changes their bytes. `sew` copies
  each input on its own instead of copying one compound of them (shared sub-shapes become
  coincident copies, which the sewing merges again). `make_compound` adds each part under a new
  top-level TShape of its type that holds the same children (the same geometry for a face, edge
  or vertex part), so everything below the top level is shared and the parts stay as they were.
  `two.sub_shape(f).is_same(part.sub_shape(f))` holds for every face of a solid part; a part
  given twice is indexed once below its top level.
- **Sewing copies geometry too**: with shared curves (`copyGeom` false) the sewing updated the
  input's curves and the input's bytes changed.
- **Sewing keeps each face's own boundary**: it merges edges by tolerance but does not move a
  face's pcurves. A gap made by shrinking faces leaves the shrunk area out (2.8e-5 relative
  volume loss on the cube), so Done-when 1's gaps keep every face's full square and move
  opposite faces together, which keeps the enclosed volume exact.
- **ShapeFix builds faces it does not record** (`ShapeFix_Face` splitting the bow-tie): such a
  result face is `Modified` from the input face whose copy has its surface. Without it the
  bow-tie heal left an unexplained face (`PartialHistory`).
- **ShapeFix changes valid shapes a little**: it rebuilds faces around seams and degenerated
  edges (1 sub-shape on the torus, 18 on the filleted box) without moving geometry, so `heal` of
  a valid shape may still report fixes. A box reports none.
- **A box edge without a 3D curve crashes the existing `check`** (access violation in the index
  build or `BRepCheck_Analyzer`, before any BREP-10 code). The defect was dropped from the
  corpus; see Leftovers.
- `inside_out`, `wire_order`, `vertex_tolerance` and `small_edge` are BRepCheck-valid, so they
  are in the corpus for `check` and the orientation test, not for Done-when 3.
- `check`'s analyzer pass is opt-in (`self_interference`), and it also runs the small-edge and
  curve-on-surface tests: running it by default would change `check` for every existing caller
  and cost 84 ms on the filleted box.

**For DOC-06 and S20.** `heal`, `unify_same_domain`, `sew` and `make_solid` histories are 1:1
`Modified` through their copy, so `compose` chains them (S20 R8; Done-when 4 checks fuse +
unify). `sew` is multi-operand (one operand per input shape). `make_compound` has no history:
its indices are `TopExp::MapShapes` over the parts in order.

**Leftovers.**
- Shapes with an edge without a 3D curve (possible from IGES) crash the index build or
  `BRepCheck_Analyzer`: guard them before BREP-13's IGES import (heal could add the curve with
  `ShapeFix_Edge::FixAddCurve3d`, but only once the shape can be indexed).
- `heal` cannot fix a gap wider than `tol` in a wire, or close an open solid; S20's heal levels
  choose `tol`/`max_tol`.

### 2026-11-25 - BREP-15 pipe-shell sweeps and the raw ThruSections loft
**Landed.**
- `amg-brep`, trait `BrepKernel` (with the forwarding arms in `RecordingKernel`,
  `tests/api_contract.rs` and `tests/common/fault.rs`):
  - `pipe(operands, profiles, &PipeOpts, ctx)` and `thru_sections(wires, sections,
    &ThruSectionsOpts, ctx)`, with the Interfaces signatures.
  - S20's `KernelOp::{Pipe, ThruSections}` (the S20 field names).
  - `PipeMode::Support { shell }` (R-S04-7); `impl Default` for `PipeOpts` and
    `ThruSectionsOpts`.
- New module `sweep` (plain Rust, unit-tested without OCCT): validation, lowering and the
  failure mapping.
  - `pipe` operands are `[profiles..., spine, aux?]`. Profiles are wires or edges; the first and
    last may be vertices. The spine is a wire or an edge. The guide or support is the operand
    right after the spine (`Guide { wire }` / `Support { shell }` must name it).
  - Laws: at least 2 samples, `t` strictly increasing from 0 to 1, values finite; scale factors
    ≥ 1e-6.
  - A twist needs the corrected Frenet, Frenet or binormal mode. A scale needs exactly one wire
    profile, and no guide, support or twist (OCCT: an auxiliary spine and a homothetic law do
    not combine).
  - `thru_sections`: at least 2 sections; points only first or last; at least one wire; each
    wire used once, except the first repeated last (a closed loft); `max_degree` 0..=25.
  - Shim errors map to `Cancelled`, `InvalidInput` (prefix `invalid: `) or `OpFailed`.
- `amg-occt-sys`, new `cpp/occt_sweep.cpp` with `pipe_op` and `thru_sections_op` (both **C**).
  - Every operand runs on a private `BRepBuilderAPI_Copy`. An edge operand becomes a one-edge
    wire of its copy, never of the caller's edge (`TopoDS_Builder::Add` flags, BREP-10).
  - Bridge structs `OcPipeParams`, `OcLoftParams`, `OcLoftSection`. Role code 8 is
    `LoftSection`, its `role_args` entry the section index. `Recorder::role` takes that argument
    (default 0; `occt_history.hxx`).
  - Progress and cancellation go through the BREP-12 `ProgressSink` into `Build`.
- **Pipe modes.**
  - `SetMode(false/true)` for corrected Frenet / Frenet, and `SetMode(gp_Dir)` for binormal.
  - Fixed: `SetMode(gp_Ax2)` on the binormal. A constant trihedron only translates the sections,
    so its axes do not matter.
  - Guide: `SetMode(wire, false, Contact | NoContact)`. This is the plane mode: the guide point
    is where the spine's normal plane cuts the guide.
  - Support: `SetMode(shape)`. OCCT only uses the spine edges' pcurves on the support faces,
    which only the support's own edges carry. The shim builds the spine from the support's copy
    when every spine edge is a support edge, and refuses anything else (`InvalidInput`).
    Otherwise OCCT silently ignored the support, or used planes' projected pcurves.
- **Twist law.** OCCT's pipe shell has no twist law. The shim samples the base mode's trihedron
  law (`GeomFill_CorrectedFrenet` / `_Frenet` / `_ConstantBiNormal` on a
  `BRepAdaptor_CompCurve`), with ≥ 64 samples (32 per spine edge). At each sample it puts a
  point at `cos θ N + sin θ B` (θ linear between the law samples), at half the profiles' box
  diagonal from the spine. A cubic interpolation of the points is the auxiliary spine of a
  plane-mode guide sweep. So the section turns by `twist(t) − twist(0)` about the tangent,
  counter-clockwise for positive angles.
- **Scale law.** `SetLaw(profile, Law_Linear | Law_Interpol)`. The law's `t` is mapped onto the
  spine's `BRepAdaptor_CompCurve` (edge-parameter) range; measured right on 1- and 2-edge spines.
- **Solid.** `MakeSolid` after the build; it fails (`OpFailed`) on open profiles. Otherwise the
  result is the swept shell.
- **History** (one `operand_counts` entry per operand):
  - A copied sub-shape that is in the result is `Modified` into it (the first section's edges).
    OCCT's `Generated` images are `Generated`.
  - Pipe: profile edges give their lateral faces along the whole spine, profile vertices the
    lateral edges. **`Generated` also covers the spine**: each spine edge gives the faces along
    it, and each spine vertex the section edges there.
  - Loft: `Generated` ∪ `GeneratedFace`. OCCT reports a section edge's whole column of faces.
    The shim narrows that to the faces the edge bounds, when the edge is itself in the result
    (ruled lofts, and the end sections of smooth ones). A middle section of a smooth loft
    keeps its column, which is one face per edge column.
  - Roles: a cap is a face whose edges all belong to `FirstShape`/`LastShape`. Pipe caps are
    `StartCap`/`EndCap`, loft caps `LoftSection { 0 | n−1 }`. Every other face is `Lateral`.
- Failures:
  - `pipe`: the `BRepBuilderAPI_PipeError` name (`PlaneNotIntersectGuide`, ...), a caught
    exception, or a failed `MakeSolid`.
  - `thru_sections`: `NotDone` (closed and open sections mixed, coincident sections) or a caught
    exception.
  - Without the compatibility check, OCCT built an open, invalid shape from sections of
    different edge counts. The shim now refuses that before the build ("incompatible
    sections").
- Tests:
  - `tests/sweep.rs`: 10 tests.
  - `tests/common/sweep_corpus.rs`: 25 cases, 17 pipes and 8 lofts, 20 with a closed form.
  - 25 goldens `tests/golden/brep/{pipe,loft}_*.toml` (adaptive volume and area, topology,
    face classes, rel 1e-6).
  - The corpus joins `every_output_face_has_a_source_or_a_role`.
  - Unit tests: 2 in `src/sweep.rs`, 1 in `src/ops.rs` (role 8). The `KernelOp` round trip has
    the 2 new variants.
- Bench: `benches/sweep.rs` (`brep-sweep/*`).

**Done-when status.** With `--features occt`, `cargo nextest run -p amg-occt-sys -p amg-brep`
passes 333 tests (1 skipped: BREP-04's ignored timing run). Without the feature, 108 pass.
`cargo clippy -p amg-occt-sys -p amg-brep --all-targets --all-features -- -D warnings` is clean,
also without features. `cargo build -p amg-brep --no-default-features` builds.
1. Met (`golden_sweeps_and_lofts_match_the_reference_volumes`). Every case is valid: one solid,
   or a shell for the two shell cases.
   - Circle r 2 along a 30 mm line: π r² L exactly (0.0).
   - Along a quarter arc of R 20 (torus segment): 2.9e-16.
   - Square 2 × 2 along a 2-turn helix (R 10, pitch 8; a C1 cubic B-spline, one Bézier per
     quarter turn), against area × the spline's Gauss length: 1.4e-6 (corrected Frenet) and
     9.5e-6 (Frenet). The bound is 1e-3.
2. Met.
   - The 90° twist of a 4 × 4 square over 20 mm is within 3.9e-7 of A·L. A 3-sample law to π of
     a 6 × 2 rectangle is within 1.3e-6.
   - The linear scale 1 → 2 of a circle (π r² L · 7/3) is within 1.9e-16, on 1 and 2 spine
     edges alike.
   - `twist_and_scale_laws_shape_the_end_section`: a law ending at π/2 puts the rectangle's long
     side along y, and π puts it back along x. +π/4 puts the corner (3, 1) at 45° further
     counter-clockwise, within 1e-3. The scaled end cap has area 16π within 1e-4.
3. Met.
   - The smooth loft through circles r = 2, 4, 3 at z = 0, 2, 3 is within 1.9e-9 of π∫r(z)² dz.
     r(z) is the quadratic through the three sections. Both spans rise as much as the radius
     changes, so OCCT's chord-length parameters (0, 2/3, 1) are the same for every pole, and the
     smooth surface is exactly that revolution.
   - The ruled version matches two frusta exactly. Point-section cones: circle → apex 0.0, and
     apex → circle → apex 1.3e-16.
   - Also matched: ruled squares (two square frusta, 1.1e-16); right-corner mitre (A × centre
     line, exact); fixed mode (prism, exact); binormal mode along the arc (5.7e-16); the guide
     (0.0); the support (torus on a cylinder rim, 8.0e-7); two sections (frustum, 2.9e-16); and a
     point section to a circle (2.4e-9).
4. Met (`lateral_faces_come_from_the_section_edges`), on all 25 cases.
   - Every lateral face is `Generated` from an edge of a section operand, and every edge of the
     first section sweeps a face.
   - Caps carry `StartCap`/`EndCap` or `LoftSection { 0 | n−1 }`. Point sections and the support
     case's closed spine have none.
   - Ruled squares: each lateral face comes from exactly the two sections it joins.
   - The recorded coverage (naming note Q2) is lateral faces per section operand, all sections
     covered. For example: `pipe_two_sections` [1, 1], `pipe_square_corner_right` [8] (4 edges ×
     2 spine edges), `pipe_point_to_circle` [0, 1] (the point has no edge), `loft_squares_ruled`
     [8, 8, 8], `loft_three_circles_smooth` [1, 1, 1].
5. Met (`incompatible_sections_fail_cleanly`).
   - These all return `OpFailed`, with no crash: closed and open sections (smooth and ruled),
     different edge counts without the check, coincident sections, a solid pipe of an open
     profile, and a guide out of reach (`PlaneNotIntersectGuide`).
   - A support that does not carry the spine is `InvalidInput`.
   - `bad_inputs_are_invalid_input`: 30 refusals.
   - KH-05: the loft runs through `&dyn BrepKernel` and behind `RecordingKernel`
     (`the_raw_loft_runs_behind_the_recording_kernel`).

Also tested:
- `ten_runs_are_byte_identical`: every case, 10 runs, identical BinTools bytes and history, and
  the inputs' bytes unchanged. Pipe shell and ThruSections are deterministic, so no canonical
  rebuild was needed.
- `shared_shapes_survive_concurrent_sweeps`: 8 threads run the corpus on shared inputs.
- `cancelled_jobs_do_not_run`.
- `cancel_stops_a_running_loft`: a cancel 100 ms into the ruled 40-section loft of 48-gons
  returns `Cancelled` after 0.51 s, against 2.0 s for the full run (debug-opt). The bound is
  asserted only under `AMG_STRICT_TIMING=1`.

The history completeness test now covers 25 BREP-15 results as well, with no `Unexplained`
face.

**Bench** (release, criterion, validation off; S02 has no sweep or loft budget):

| Bench | Median |
|---|---|
| `pipe_circle_line` | 0.40 ms |
| `pipe_circle_scale` | 0.80 ms |
| `pipe_square_corner_right` | 2.52 ms |
| `pipe_square_twist` (with the auxiliary spine) | 3.60 ms |
| `pipe_square_helix` | 24.0 ms |
| `loft_three_circles_smooth` | 1.16 ms |
| `loft_squares_ruled` | 2.68 ms |
| the 25-case corpus | 103 ms |
| `loft-40x48-smooth` (50 faces) | 199 ms |
| `loft-40x48-ruled` (1874 faces) | 1.48 s |

**Findings and deviations.**
- **No twist law in OCCT** (see above). The twist is realised as a guide sweep, so it cannot be
  combined with a scale law, a guide or a support.
- **Support mode needs the support's own edges, and curved ones.** On a straight spine edge
  OCCT raises `Standard_NotImplemented` (`Adaptor3d_CurveOnSurface::EvalDN`); that is
  `OpFailed`. The corpus case sweeps along a cylinder's rim.
- **`BRep_Tool::CurveOnSurface` projects onto planes.** A "carries the spine" check must ask for
  stored pcurves (`theIsStored`).
- **ThruSections without the compatibility check** builds invalid shapes from different edge
  counts. It is refused before the build.
- **Spine history.** OCCT's `Generated` on the spine gives the laterals per spine edge. DOC-06
  can tag pipe laterals with their spine edge's tag through that relation. The spine is an
  operand, so `pipe` itself takes no tags (naming note Q2).
- **Ruled-loft history** is narrowed to the faces an edge bounds (above), with an edge→faces
  ancestor map. Matching face edges per image was 3× slower on the 1874-face loft.
- `SHIM_VERSION` stays 4: no existing op's history changed. Role code 8 is new data.

**For DOC-06.**
- Pipe and loft histories are multi-operand: one entry per profile, spine and aux, or per wire.
- Laterals are `Generated` from the section edges (and, for pipes, from the spine edges). The
  first section's edges are `Modified` into the start cap's boundary.
- Caps are role-only, with no face source. The fixtures are in `tests/common/sweep_corpus.rs`.

**Leftovers.**
- `PipeMode::Support` on straight spine edges (an OCCT limit; S04 SOL-10 might use a guide
  instead).
- Twist combined with scale, for example by sweeping scaled copies as several sections.
- A real helix spine comes with BREP-11 (`helix`); the corpus approximates it with a C1 cubic.

### 2026-11-25 - BREP-16 draft, defeature, replace face, split faces, split body and offset faces
**Landed.**
- `amg-brep`, trait `BrepKernel` (with the forwarding arms in `RecordingKernel`,
  `tests/api_contract.rs` and `tests/common/fault.rs`):
  - `draft`, `defeature`, `replace_face` and `split_faces`, with the Interfaces signatures.
  - **New:** `offset_faces(s, &[(FaceId, f64)], ctx) -> KResult<OpResult>` (R-S04-5, left to this
    WP by BREP-07). It is also the first step of `replace_face`.
  - S20's `KernelOp::{Draft, Defeature, ReplaceFace, SplitFaces, OffsetFaces}`, with replay arms
    and the round-trip test. Faces stay plain `FaceId`s, as for BREP-07's shell faces.
- New module `local` (plain Rust, unit-tested without OCCT): validation and lowering.
  - Face lists must be non-empty, distinct and in range.
  - `draft`: the angle must be finite and in (−π/2, π/2), the neutral frame orthonormal, and
    `pull` not in the neutral plane.
  - `defeature` and `replace_face` need a solid. `defeature` must not remove every face.
  - `replace_face` refuses `Surface::Other`.
  - `offset_faces`: each distance finite and longer than the linear tolerance.
  - `tool_bounds`: the parameter box of replace face's tool face.
  - `overshoot`: how far past the new surface the face is pushed.
  - Shim failures map through BREP-07's `offset::failure`.
- `amg-occt-sys`, new `cpp/occt_local.cpp` (all **C**, each on a private `BRepBuilderAPI_Copy`,
  history through the copy):
  - `draft_op`: `BRepOffsetAPI_DraftAngle`. Each face's `Add` result is checked (`AddDone`,
    `ProblematicShape`). Faces that are not planar, cylindrical or conical are `invalid: `. The
    images come from `Modified`, else `ModifiedShape`, as BREP-05 found for the taper prism.
  - `defeature_op`: `BRepAlgoAPI_Defeaturing` with history and a progress range. OCCT *skips*
    a feature it cannot remove and only warns (`BOPAlgo_AlertUnableToRemoveTheFeature`). The
    shim treats that warning as a failure naming the feature's faces.
  - `trim_face_op`: `BRepAlgoAPI_Splitter` (non-destructive) of the body by the tool face. A
    piece is kept when its face on the new surface faces away from the material: its outward
    normal agrees with the surface normal, turned to point along the old face's outward
    normal. A piece the surface does not bound is placed by a vertex off the surface.
  - `face_reach` (**R**): the largest signed distance from the face's samples (vertices, and a
    5 × 5 grid of cell centres inside the face) to the new surface along their outward normals
    (`GeomAPI_IntCS`). It is NaN when no normal line meets the surface.
  - `split_faces_op`: first find the tool's curves on the chosen faces. A tool with faces gives
    its `BRepAlgoAPI_Section` with a compound of the chosen faces. A wire or edge tool is cut
    at the faces' boundaries by a Splitter, and the pieces whose midpoint lies within 1e-5 mm of
    a chosen face are kept. Then `BRepAlgoAPI_Splitter` of the body by those edges. Edges do not
    split solids, so the solid stays one.
  - Bridge struct `OcDraftParams`. Bridge functions `draft_op`, `defeature_op`, `face_reach`,
    `trim_face_op` and `split_faces_op`.
- `cpp/occt_offset.cpp`: `offset_op` mode 3 (`offset_faces`).
  - Base offset 0, one `SetOffsetOnFace` per listed face, intersection join, `MakeOffsetShape`.
  - The collapse check, the canonical face order and the regularity rewrite are BREP-07's.
  - Only the moved faces' images get `OffsetFace`. Neighbours that were extended or trimmed are
    `Modified` with no role.
  - Mode 1's `needs_solid` rule is unchanged.
- **Replace face** (`replace_face` in `kernel.rs`):
  1. `make_face(with, tool_bounds)` gives a face 4 box diagonals wide. Closed surfaces and
     B-splines use their natural domain. A cone stops short of its apex.
  2. `face_reach` measures how far the surface lies beyond the face.
  3. When the reach is positive, `offset_faces` pushes the face `reach + 5 % + 1 %` of the body
     size past the surface, so the neighbours extend.
  4. `trim_face_op` cuts the body back to the surface.

  The two histories are composed (`ShapeHistory::compose`). The offset's roles are dropped
  first: the moved face becomes the new face, and `OffsetFace` would mislabel it. The replaced
  face is `Modified` into the faces on the new surface. Its edges that are gone are `Modified`
  into the result edges between those faces and the images of the edge's other face.
- Split body is BREP-04's `split` with a face or a face set as the tool. No new op was needed.
  The corpus covers a box split by a plane face (2 solids) and by a compound of two faces
  (3 solids), with `SplitPiece` roles 0..2.
- Tests:
  - `tests/local.rs`: 12 tests.
  - `tests/common/local_corpus.rs`: 27 cases, every one with a closed-form volume. 5 drafts,
    5 defeatures, 6 replace faces, 5 split faces, 2 body splits, 4 offset faces.
  - 27 goldens `tests/golden/brep/{draft,defeature,replace,split,offset_faces}_*.toml`. Each
    holds the adaptive volume and area, the topology counts and the face classes (rel 1e-6).
  - The corpus joins `every_output_face_has_a_source_or_a_role`.
  - 2 unit tests in `src/local.rs`. The `KernelOp` round trip has the 5 new variants.
- Bench: `benches/local.rs` (`brep-local/*`).

**Done-when status.** With `--features occt`, `cargo nextest run -p amg-occt-sys -p amg-brep`
passes 347 tests (1 skipped: BREP-04's ignored timing run). Without the feature, 109 pass.
`cargo clippy -p amg-occt-sys -p amg-brep --all-targets --all-features -- -D warnings` is clean,
also without features. `cargo build -p amg-brep --no-default-features` builds, and `cargo check
-p amg-doc -p amg-features` passes.
1. Met (`a_three_degree_draft_tilts_the_sides_by_three_degrees`). The 4 sides of the
   20 × 16 × 12 box are drafted 3° with pull +z about z = 0.
   - Each tilted face's outward normal · pull is sin 3°. The worst angle error is
     **1.5e-16 rad** (bound 1e-9).
   - The volume is within 1e-12 of the prismatoid `C/6 (A₀ + 4 A_m + A₁)` (3574.646 mm³).
   - The top shrinks to `(A − 2C tan 3°)(B − 2C tan 3°)` within 1e-9.
   - In the corpus goldens: a negative angle (the part grows), a neutral plane at mid-height,
     one side at 5°, and a cylinder wall at 4° into a frustum. All match their closed forms
     within 4.4e-16. A sphere face is refused (`InvalidInput` naming face 0).
2. Met (`deleting_a_fillet_and_a_boss_restores_the_volume`). Each case gives a 6-face box of
   volume A·B·C with error **0.0**, and every removed face is `Deleted`:
   - fillet r 3: 3801.37 → 3840 mm³;
   - boss r 3 × 5: 3981.37 → 3840;
   - chamfer 2: 3800 → 3840;
   - through hole r 2: 3689.20 → 3840;
   - blind hole 4 deep: 3789.73 → 3840.

   Removing a box's whole top is refused with `LocalOpFailed { op: "defeature" }` naming that
   face (the alert above).
3. Met (`replacing_the_top_by_an_offset_plane_changes_the_height_exactly`). Planes at
   z = C + 3, C − 4, C + 0.5 and C + 25:
   - every vertex of the new top is at that z within 1e-9 (measured exact);
   - the volume `A B (C + dz)` is within 1e-12;
   - the top is `Modified` into the one new face, centred within 1e-9;
   - each of its 4 edges is `Modified` into an edge of the new top.

   Corpus goldens, all matching their closed forms within 4.2e-16:
   - a plane tilted 10° about the top's centre line (half extends, half trims; the volume
     stays A·B·C);
   - a side pushed out 5 mm;
   - a cylindrical crown (R 40, axis along y), against `B (z_c A + a/2 √(R² − a²/4) +
     R² asin(a/2R))`;
   - a cylinder top raised 2 mm.
4. Met (`splitting_a_face_by_a_sketch_wire_gives_two_faces_of_the_right_areas`). The top is
   split by:
   - a sketch line at x = 5 reaching 2 mm past both sides: 240 and 80 mm², within 2.4e-16;
   - a sketch circle r 3: 291.73 and 28.27 mm², within 1.4e-15;
   - a plane face at x = 12: 192 and 128 mm².

   Each result is one solid of 7 faces of unchanged volume, and every other face maps 1:1. A
   line 5 mm above the top is refused (`InvalidInput`). The corpus also splits the top and
   bottom by one plane (8 faces) and the cylinder wall at z = 4.
5. Met.
   - `golden_local_ops_match_the_reference_volumes`: all 27 results are BRepCheck-valid with the
     expected solid count.
   - `every_result_has_a_complete_history`: every result passes `completeness_error` with no
     warning.
   - The history completeness test now covers 27 BREP-16 results (2194 output faces in all),
     with no `Unexplained` face.
   - `roles_follow_the_ops`: `offset_faces` labels only the 2 moved faces `OffsetFace`. The face
     set split gives `SplitPiece` 0, 1 and 2. Draft, defeature, replace and split faces carry no
     role: every face has a source.

Also tested:
- `ten_runs_are_byte_identical`: every case, 10 runs, identical BinTools bytes and history; the
  inputs' bytes are unchanged.
- `shared_shapes_survive_concurrent_local_ops`: 8 threads run the whole corpus on the shared
  inputs. Every result has the single-thread bytes, and the inputs are unchanged.
- `bad_inputs_are_invalid_input`: 17 refusals.
- `cancelled_jobs_do_not_run`.
- `local_ops_run_behind_the_recording_kernel`: the corpus through `&dyn BrepKernel` behind S20's
  `RecordingKernel` (capture on failure).

**Bench** (release, criterion, validation off; S02 has no budget for these ops):

| Bench | Median |
|---|---|
| `draft-box-sides` | 0.42 ms |
| `defeature-fillet` | 9.53 ms |
| `defeature-boss` | 2.43 ms |
| `replace-top-up` (offset + trim) | 4.65 ms |
| `replace-top-down` (trim only) | 2.65 ms |
| `split-faces-top-line` | 1.13 ms |
| `offset-faces-top` | 1.77 ms |
| the 27-case corpus | 71.3 ms |
| `defeature-plate-1030-one-hole` | 8.40 s |
| `offset-faces-plate-1030-top` | 11.6 s |
| `split-faces-plate-1030-top` | 250 ms |

**Findings and deviations.**
- **`BRepAlgoAPI_Defeaturing` skips what it cannot remove.** It reports success with a warning
  and returns the shape with the feature still in it. The shim makes this a `LocalOpFailed` that
  names the feature's faces. Other warnings are dropped.
- **Replace face is offset, then trim**, not "defeature, then a half-space boolean" (S02 §5).
  Defeaturing the face to be replaced fails whenever the neighbours cannot close the gap without
  it (a box's top), and a half-space boolean cannot add material. Pushing the face past the new
  surface with `offset_faces` and trimming back covers both directions. A surface that crosses
  the face (the tilted plane) also works. The tool face is a bounded face, so the new surface
  must cross the whole body inside 4 box diagonals; a half-space solid is not needed.
- **`offset_faces` and defeature cost grows with the whole shape.** On the 1030-face plate they
  take 11.6 s (BRepOffset processes every face, as for BREP-07's 12.2 s offset) and 8.4 s
  (BOPAlgo rebuilds every face), against a few ms on the box. S04's press-pull and delete-face
  on large parts (SOL-09) need a local path; see Leftovers.
- **Split faces imprints curves, not a tool's faces.** A tool with faces gives its section with
  the chosen faces, so only they are split; the solid is not split (that is `split`). A curve
  that runs along an edge shared with a face that is not chosen splits that face too; the docs
  say so.
- `draft` keeps every face and only re-intersects the neighbours, so the history is 1:1
  `Modified` with no role. `ModifiedShape` is needed for the tilted faces (BREP-05).
- `SHIM_VERSION` stays 4: no existing op's history changed. Mode 3 of `offset_op` is new.

**For DOC-06.** All five ops have one operand. Draft is 1:1 `Modified`. Defeature has
`Deleted` removed faces and `Modified` extended neighbours. Replace face sends the replaced
face (and its edges) to the new face. Split faces sends a face to its pieces in split order.
Offset faces sends every face 1:1 and gives `OffsetFace` to the moved ones. The fixtures are in
`tests/common/local_corpus.rs`.

**Leftovers.**
- A local press-pull path for large parts (a prism of the face, fused or cut, when the
  neighbours are perpendicular planes), and the same for removing a hole. Both are 8–12 s on the
  1030-face plate today.
- Mid-op cancellation of `draft` was not measured: `Build` gets the progress range, and
  cancellation is checked before and after it.
- A face-fingerprint `reindex` for the faces of `KernelOp::{Draft, Defeature, ReplaceFace,
  SplitFaces, OffsetFaces}`, as for BREP-06's and BREP-07's face arguments.
- `replace_face` with a surface that crosses the face's normal lines only partly (a small
  cylinder) relies on `face_reach`'s 5 × 5 samples. A surface that misses all of them is
  refused.

## Interface changes

### 2026-10-20: requests from the wave-2 refinements (S02, S05, S10, S11, S12, S16, S17, S20)

Additive requests recorded by the integration pass; the owning lane confirms or adjusts them when it implements the affected WP.

- **From S05 (inspection).** Mostly covered by BREP-09 (`mass_props`, `distance`,
  `closest_point`, `eval_surface`, `interference`). Still to confirm in the Interfaces:
  - a public face↔edge adjacency query (`faces_of_edge`) on top of the `SubShapeIndex`;
  - `eval_surface` returns principal curvatures **and** directions `(k1, k2, d1, d2)` (INS-06);
  - `iso_curves(face, nu, nv, deflection)` for INS-12 (BREP-08).
  - INS-04 now also depends on BREP-09 (`interference`); applied in S05's WP table.
- **From S10 (rendering), consumed by S11 UI-22.** `TessMesh` guarantees: no vertex shared between
  faces; `face_ranges` in in-shape face order; edge polylines in in-shape edge order; optional
  per-vertex principal curvatures.
- **From S12 (import/export).**
  1. Keep `ForeignModel` / `ForeignNode` as drafted (`instance_of`, per-face colours by `FaceId`,
     `source_unit_mm`, `JobCtx` progress/cancel).
  2. Keep `TessMesh::to_indexed_parts` watertight for closed solids. P1: optional
     `max_edge_length` / `max_aspect_ratio` on `TessQuality`.
  3. Confirm that BREP-17 `project_edges` returns ordered, closed loops for a planar face's edges
     (outer loop first). If not, add
     `planar_face_loops(&Shape, FaceId) -> KResult<(Plane, Vec<Vec<CurveSeg>>)>` (IO-08 needs it).
- **From S20 (kernel hardening).**
  - R14: `amg-occt-sys` accepts additive S20 shim files `plate.cpp` (GeomPlate, KH-19) and
    `gordon.cpp` (`GeomFill_Gordon`, KH-16/KH-18).
  - R15: OCCT 8.0.1 bump, see QUESTIONS #27 (with 8.0.0, S20 skips the OCCT Gordon path).
  - R16: ordered wire traversal `Shape::wire_edges() -> Vec<(EdgeId, bool)>`
    (`BRepTools_WireExplorer`), if not already covered by the topology iteration in BREP-02.
  - R17: additive `KernelWarning::Strategy { name, detail }` so `HardenedKernel` can report
    non-direct acceptances.
  - S20 relies on: `fillet_max_radius` (BREP-06), `CheckOpts { self_interference }`,
    `ShapeHistory::compose`, curvatures from `eval_surface` (BREP-09), and sub-shape indices
    surviving a text-`.brep` round trip (KH-01 adds that test).
- **From S16 (AM).** Solid bodies on build plates need BREP-08 tessellation at export quality
  (IO-09 presets); until then Phase 1 plate items are mesh bodies.

### 2026-11-08: requests from the S04 refinement (lane A)

Additive; S04 Interfaces ("Kernel calls") has the signatures. The S04 WP that needs a request
adds it to `amg-brep` itself if the S02 WP named here is already Done (same lane), with its
`KernelOp` variant (S20 rule).
- **R-S04-1 (BREP-09):** `edge_ends(s, e) -> KResult<EdgeEnds { p0, t0, p1, t1 }>`, for
  tangent-chain propagation in fillet/chamfer (SOL-05).
- **R-S04-2 (BREP-05):** `OpResult::per_solid` (= the naming note's Q3); cut and split results
  become one body per solid (SOL-01, SOL-07).
- **R-S04-3 (BREP-03):** `make_thin_face(&ThinProfile, &Tolerance) -> KResult<FaceBuild>`
  (`BRepOffsetAPI_MakeOffset`, open-wire mode), for thin extrude/revolve and ribs (SOL-18,
  SOL-13).
- **R-S04-4 (BREP-17):** `silhouette_curves(s, faces, dir) -> KResult<Vec<CurveSeg>>`
  (`Contap_Contour`), for silhouette split (SOL-22).
- **R-S04-5 (BREP-16):** `offset_faces(s, &[(FaceId, f64)], ctx) -> KResult<OpResult>`, for
  offset face and press-pull (SOL-09).
- **R-S04-6 (BREP-14):** `mirrored(s, &Plane) -> KResult<OpResult>` (reflection with fixed
  face orientation); `Shape::transformed` keeps rejecting improper transforms (SOL-08).
- **R-S04-7 (BREP-15):** `PipeMode::Support { shell }` (`MakePipeShell::SetMode(SpineSupport)`),
  for guide-surface sweeps (SOL-10).
- **R-S04-8 (BREP-07, contract):** `shell` with an empty `remove` list returns a hollow solid
  (SOL-06).
- **R-S04-9 (BREP-10, Phase 3):** `fill_patch(boundary, support, Continuity, ctx)`
  (`BRepOffsetAPI_MakeFilling`) and `reverse_faces(s, faces)`, for the surface repair tools
  (SOL-27).
- S04 also relies on the naming note's Q1 (rigid `transformed` keeps index order), used by
  pattern instances.

### 2026-11-09: shared files changed by BREP-01 (lane A)
All additive except the CI timeouts:
- `.cargo/config.toml`: `[env] AMG_OCCT_DIR` (relative to the workspace; a real environment
  variable wins), as §1 specifies.
- `Cargo.lock`: `cxx`, `cxx-build` and their dependencies. Both are declared in
  `amg-occt-sys/Cargo.toml` rather than the workspace table, since only that crate uses them.
- `.github/workflows/ci.yml`: new `occt` job. `clippy`, `test` and `bench` timeouts raised to
  350 min, because `vcpkg install` in those jobs now includes OCCT on a cache miss.
- The `--all-features` clippy step in CI now compiles the shim and needs the vcpkg install, which
  that job already runs.

### 2026-11-10: additions and shared files changed by BREP-02 (lane A)
All additive:
- `amg-brep`: `Shape::face_area(FaceId) -> f64` (exact area, used for index-stability checks),
  `geom::Surface::class() -> GeomClass`, `pub const FORMAT_VERSION: u8`, `#[doc(hidden)]
  Shape::fixture` behind the new `fixture-gen` feature.
- `amg-occt-sys`: `pub use cxx` (so `amg-brep` names `UniquePtr`/`Exception` without its own
  `cxx` dependency), the `fixture-gen` feature and `fixture_ffi` bridge; `SHIM_VERSION` is 2.
- New dependency `static_assertions` 1.1.0 (MIT/Apache-2.0, `THIRD_PARTY.md`); `Cargo.lock`
  changes accordingly.

### 2026-11-10: additions and shared files changed by BREP-12 (lane A)
All additive:
- `amg-brep`: `opts` module (all op parameter types of the Interfaces), `CheckOpts`,
  `CheckReport` (+ `valid()`, `problem_kinds()`), `KernelConfig` (+ `Debug`, `PartialEq`,
  `effective_occt_threads`), `Validate::{is_enabled, parse}`, `ENV_VALIDATE`,
  `ENV_OCCT_THREADS`, `init`, `kernel_version`, `POLICY_VERSION`, `BrepKernel { check }`,
  `OcctKernel::{new, config, validate_result}` (+ hidden `progress_probe`),
  `KernelError::offending()` and the remaining spec variants, `Shape::max_tolerance`.
- `amg-occt-sys`: `ProgressTarget`, `ProgressSink`, `LOG_TARGET`, the BREP-12 shim functions;
  optional `tracing` dependency under `occt`; `SHIM_VERSION` 3.
- `amg-brep` gains `tracing` (and `tracing-subscriber` as a dev-dependency), both already
  workspace dependencies; `THIRD_PARTY.md`'s tracing row names the new users. `Cargo.lock` changes
  accordingly.

### 2026-11-11: addition requested by S06 ASM-09 (lane E)
Additive:
- `amg-brep`: `Shape::volume() -> KResult<f64>` (`BRepGProp::VolumeProperties` through the existing
  BREP-01 shim function `amg_occt_sys::ffi::volume`; no shim change, `SHIM_VERSION` unchanged). ASM-09
  uses it to check that moving a body between components keeps its volume. S05's mass properties
  (INS-10) may replace or extend it.

### 2026-11-12: additions and shared files changed by BREP-08 (lane A)
All additive:
- `amg-brep`: `TessMesh`, `EdgePolyline`, hidden `TessProfile`; `BrepKernel::{tessellate,
  tessellate_many, tessellate_edge}` (every `BrepKernel` implementor must add them; the
  `api_contract.rs` decorator forwards them); hidden `OcctKernel::{tessellate_profiled,
  boolean_cut_probe}`; `EdgeId: Default`; `rayon` dependency (workspace).
- `amg-occt-sys`: `OcTessParams`, `OcTessMesh`, shim functions `tessellate_shape`,
  `tessellate_edge`, `make_sphere`, `boolean_cut_probe`, new `cpp/occt_tess.cpp` in `build.rs`.
- `tests/golden/brep/` (new suite), `THIRD_PARTY.md` rayon row (amg-brep added as a user),
  `Cargo.lock`.

### 2026-11-12: additions and shared files changed by BREP-03 (lane A)
All additive:
- `amg-brep`: the `geom` builder inputs (`Tag`, `Curve2d`, `BSpline2dData`, `ProfileEdge2`,
  `Loop2`, `PlanarProfile`, `CurveSeg`, `FaceBounds`); the `history` data types of the Interfaces
  (`InputSub`, `Relation`, `OpRole`, `ShapeHistory` with `of_input`/`sources_of`/`role_of`/
  `tag_of`, `KernelWarning`, `OpResult`, `FaceBuild`); `BrepKernel::{make_planar_faces,
  make_planar_polygon, make_face, make_wire}` (every `BrepKernel` implementor must add them; the
  `api_contract.rs` decorator forwards them). `ShapeHistory::compose` and history extraction stay
  with BREP-05.
- `amg-occt-sys`: `OcEdgeIn`, `OcLoopsIn`, `OcPlanarIn`, opaque `OcBuild`, shim functions
  `make_planar_faces`, `make_face_on_surface`, `make_wire`, `build_count`, `build_shape`,
  `build_edge_map`; new `cpp/occt_build.cpp` in `build.rs`; `SHIM_VERSION` unchanged (no existing
  function changed).
- `smallvec` dependency for amg-brep (`THIRD_PARTY.md` status names it), `proptest`
  dev-dependency (workspace), `tests/golden/brep/face_profile_five_kinds.toml`, `Cargo.lock`.

### 2026-11-12: additions and shared files changed by BREP-17 (lane A)
All additive:
- `amg-brep`: `BrepKernel::{plane_section, project_edges}` (every `BrepKernel` implementor must
  add them; the `api_contract.rs` decorator forwards them), `Curve3::point_at`,
  `CurveSeg::{start, end}`.
- `amg-occt-sys`: `OcSection`, shim functions `plane_section` and `edge_curve_recognised`, new
  `cpp/occt_extract.cpp` in `build.rs`; `SHIM_VERSION` unchanged.
- `tests/golden/brep/section_filleted_box_mid.toml`. No new dependency.

### 2026-11-13: additions and shared files changed by BREP-14 (lane A)
All additive:
- `amg-brep`: `BrepKernel::{make_box, make_cylinder, make_cone, make_sphere, make_torus, prism,
  revol, mirrored}` (every `BrepKernel` implementor must add them; the `api_contract.rs`
  decorator forwards them), `Shape::gtransformed`. `mirrored` is R-S04-6.
- `amg-occt-sys`: `OcOpHistory`, opaque `OcOp`, shim functions `make_primitive`, `prism`,
  `revol`, `shape_gtransformed`, `op_shape`, `op_history`; new `cpp/occt_ops.cpp` in `build.rs`.
  Removed: the test-only `make_sphere` shim (BREP-08), replaced by the primitive.
  `SHIM_VERSION` unchanged.
- `tests/golden/brep/ops_*.toml` (20 files); `toml` dev-dependency for amg-brep (workspace,
  `THIRD_PARTY.md` row names it); `Cargo.lock`.

### 2026-11-14: additions and shared files changed by BREP-04 (lane A)
All additive except the removal of the hidden BREP-08 probe:
- `amg-brep`: `BrepKernel::{boolean, split, section}` (every `BrepKernel` implementor must add
  them; the `api_contract.rs` decorator forwards them), hidden `Shape::volume_adaptive`.
  `KernelConfig::bool_parallel` defaults to `true` (was `false` pending BREP-04). Removed: the
  hidden `OcctKernel::boolean_cut_probe` (BREP-08 test probe; its one user now calls `boolean`).
- `amg-occt-sys`: `OcBoolOpts`, `OcBoolStatus`, `OcOpHistory::in_operands`, opaque
  `OcShapeList`, shim functions `new_shape_list`, `shape_list_push`, `boolean_op`, `section_op`,
  `op_bool_status`, `volume_adaptive`; new `cpp/occt_bool.cpp` in `build.rs`. Removed: the
  `boolean_cut_probe` shim. `SHIM_VERSION` unchanged.
- `testdata/brep/bool/cases.toml` (new corpus), `tests/golden/brep/bool_*.toml` (4 files). No
  new dependency.

### 2026-11-15: additions and shared files changed by BREP-09 (lane A)
All additive:
- `amg-brep`: module `query` (`MassProps`, `Classification`, `DistanceResult`, `RayHit`,
  `SurfacePoint`, `EdgeEnds`, `MASS_PROPS_EPS`); `BrepKernel::{mass_props, classify, distance,
  closest_point, ray_cast, interference, eval_surface, project_point_on_face, edge_ends}` (every
  `BrepKernel` implementor must add them; the `api_contract.rs` decorator forwards them);
  re-exports `Mat3` and `Ray3`. `edge_ends` is R-S04-1.
- `amg-occt-sys`: `OcMassProps`, `OcDistance`, `OcRayHit`, `OcSurfacePoint`, `OcEdgeEnds`, the
  eight query shim functions, new `cpp/occt_query.cpp` in `build.rs`; `SHIM_VERSION` unchanged.
- **`amg-core` (lane D, shared):** `Ray3` derives `serde::Serialize, serde::Deserialize` (one
  derive line; `amg-core` already depends on serde), so `ray_cast`'s parameter is an op parameter
  type like `Line3`.
- New criterion bench `benches/query.rs` (`[[bench]]` in `amg-brep/Cargo.toml`). No new
  dependency, no goldens.

### 2026-11-25: additions and changed contracts from BREP-05 (lane A)
Every change is inside lane A's crates. No other lane's files changed.
- `amg-brep`, additive:
  - `SubShapeGeom`, `SplitKey`, `SPLIT_QUANTUM` and `split_order_key`;
  - `ShapeHistory::{compose, order_splits, unexplained_faces, range_error}` and
    `OpResult::per_solid` (R-S04-2);
  - `BrepKernel::{sub_geom, sub_geom_all}`, which every `BrepKernel` implementor must add (the
    in-crate decorators forward them);
  - the hidden `OcctKernel::unify_coplanar`.
- `amg-brep`, changed contracts:
  - Primitive results carry face roles (`StartCap`/`EndCap`/`Lateral`).
  - Every op history is in split order, and a face without a source or role gets `Unexplained`
    plus `KernelWarning::PartialHistory`.
  - `split` labels faces `SplitPiece { index }`.
  - `kernel_version()` changes (shim 3 → 4).
- `amg-occt-sys`: new `cpp/occt_history.hxx` and `cpp/occt_naming.cpp` (in `build.rs`);
  `unify_op`, `sub_geoms` and `sub_index_map`; `OcSubGeom` and `OcSubMap`. The prism history
  is complete for tapered and both-way infinite prisms.
- **For S01 (DOC-06):** `NamingBuilder` can rely on the completeness rule and on the split order
  (Q4, Q5). `per_solid` is the per-body view for multi-lump results (Q3). The fixtures are listed
  in the BREP-05 Refinement-log entry. The several-operand `from_history` request (§6) stays
  S01's.
- **For S20:** `compose` is available (R8). `capture::SubShapeGeom` may move to the S02
  `SubShapeGeom`; that would change the dump format.
- No new dependency. New bench `benches/history.rs` (`[[bench]]` in `amg-brep/Cargo.toml`).

### 2026-11-25: tessellation speed, measured by IO-09 (note for lane A, no code change)
- 3MF export of a 1 000-face solid is bound by `BrepKernel::tessellate` on large planar faces with
  many boundary points (release, absolute 0.025 mm / 15°): `plate_with_holes` 9.6 s (BREP-18), and
  a wheel whose side faces alternate 499 planes and 499 cylinders, with two caps, about 0.40 s per
  call, 0.38 s of it in `BRepMesh_IncrementalMesh` (a leftover for BREP-18: the caps have no holes,
  only long curved outer boundaries). The part is `scalloped_wheel_1000` in
  `crates/amg-features/tests/io_tess_common/mod.rs`; see the S12 IO-09 log.
- OCCT's mesh can exceed the requested linear deflection: a sphere (r = 7 mm fixture) at 0.005 mm /
  10° measured 0.010 mm. IO-09 re-tessellates tighter in its adapter.

### 2026-11-25: additions and changed contracts from BREP-06 (lane A)
Every change is inside lane A's crates. No other lane's files changed.
- `amg-brep`, additive:
  - `BrepKernel::{fillet, chamfer, fillet_max_radius}` with the Interfaces signatures. Every
    `BrepKernel` implementor must add them; the in-crate decorators forward them.
  - `KernelOp::{Fillet, Chamfer, FilletMaxRadius}` (S20 dump format); `reindex` covers their
    edges.
  - Roles `FilletBlend { contour }`, `ChamferFace` and `CornerPatch` now appear in op histories.
  - New bench `benches/blend.rs` (`[[bench]]` in `amg-brep/Cargo.toml`).
- `amg-occt-sys`: new `cpp/occt_blend.cpp` (in `build.rs`), with `blend_op` (C) and
  `edge_face_angle` (R); structs `OcBlendEdge` and `OcFilletParams`; field `role_args` on `OcOp`
  and `OcOpHistory` (0 for every existing role, so no existing history changed; `SHIM_VERSION`
  stays 4).
- **For S04 / SOL-05:** contours follow the caller's edge order. An edge already on another
  edge's tangent chain must carry the same spec. Smooth edges are `InvalidInput`.
- **For S20 (KH-03):** `fillet_max_radius` treats "works" as a successful and BRepCheck-valid
  build. A blend can self-intersect undetected (see the BREP-06 log), so the R5 check belongs in
  the strategy's validation gate.
- No new dependency; `THIRD_PARTY.md` unchanged.

### 2026-11-25: additions and changed contracts from BREP-07 (lane A)
Every change is inside lane A's crates. No other lane's files changed.
- `amg-brep`, additive:
  - `BrepKernel::{shell, offset, thicken}` with the Interfaces signatures. Every `BrepKernel`
    implementor must add them; the in-crate decorators forward them.
  - `KernelOp::{Shell, Offset, Thicken}` (S20 dump format).
  - Roles `ShellInner` and `OffsetFace` now appear in op histories, and `Lateral` on shell rims
    and thicken side walls.
  - New module `offset`, new bench `benches/offset.rs` (`[[bench]]` in `amg-brep/Cargo.toml`).
- `amg-occt-sys`: new `cpp/occt_offset.cpp` (in `build.rs`) with `offset_op` (C), and the struct
  `OcOffsetParams`. `SHIM_VERSION` stays 4.
- **For S04 / SOL-06 (R-S04-8):** `shell` with an empty `remove` list returns one solid with a
  closed inner void. The input must have one shell. `ShellOpts::thickness < 0` keeps the input's
  faces as the outside.
- **For S04 / DOC-06:** an offset face is `Modified` from its face in `offset`, but `Generated`
  in `shell` and `thicken` (where the input face stays, `Modified` 1:1). A removed face has no
  `Modified` relation; its rim ring is `Generated` from it with role `Lateral`.
- **For S20:** a thickness that reaches a face's centre of curvature is refused up front, with
  `LocalOpFailed` naming the faces. It is found by sampling, so a strategy's validation gate
  still needs the R5 self-interference check for B-spline faces.
- No new dependency; `THIRD_PARTY.md` unchanged.

### 2026-11-25: additions and changed contracts from BREP-10 (lane A)
Every change is inside lane A's crates. No other lane's files changed.
- `amg-brep`, additive:
  - `BrepKernel::{heal, unify_same_domain, sew, make_compound}` with the Interfaces signatures,
    and the new `BrepKernel::make_solid(shell) -> KResult<OpResult>`. Every `BrepKernel`
    implementor must add them; the in-crate decorators forward them.
  - `KernelOp::{Heal, UnifySameDomain, Sew, MakeSolid, MakeCompound}` (S20 dump format).
  - `impl Default` for `HealOpts`, `UnifyOpts` and `SewOpts`.
  - New module `heal`, new bench `benches/heal.rs` and example `gen_brep_defects`
    (`[[bench]]`/`[[example]]` in `amg-brep/Cargo.toml`).
- `amg-brep`, changed contract: `check` with `CheckOpts::self_interference` runs
  `BOPAlgo_ArgumentAnalyzer` (it was ignored); without the flag nothing changes.
- `amg-occt-sys`: new `cpp/occt_heal.cpp` (in `build.rs`) with `heal_op`, `unify_same_domain_op`,
  `sew_op`, `make_solid_op` (C), `make_compound_of` (N), `analyze_shape` (C), `op_fix_count` (R);
  structs `OcHealParams`, `OcUnifyParams`, `OcSewParams`; field `OcOp::fixes`. The `fixture-gen`
  builders gain the `defect_*` shapes. `SHIM_VERSION` stays 4.
- New test data: `testdata/brep/defects/*.brep` (17 small text files, not LFS).
- **For S20 (KH-17, R5, R8):** `check(.., &CheckOpts { self_interference: true })` is the R5
  check; its findings are named `BOPAlgo_*`. `heal`, `unify_same_domain`, `sew` and `make_solid`
  return composable histories.
- **For S07 / S12:** `sew` with `make_solid` fails (`LocalOpFailed`, `partial` = the sewn shape)
  when anything stays open; ask for `make_solid: false` to get open shells. Sewing does not move
  face boundaries, so a gap costs the area it leaves out.
- No new dependency; `THIRD_PARTY.md` unchanged.

### 2026-11-25: request from DOC-06 (lane E; not changed here)
- `BrepKernel::unify_same_domain(&self, s: &Shape, opts: &UnifyOpts) -> KResult<OpResult>` on
  the trait (BREP-10, additive). The persistent-naming note §5.1 has every BodyOp result
  unified, with the history composed in; DOC-06's lane-E test features (`test.brep_prism`
  Cut/Join) use the hidden `OcctKernel::unify_coplanar` meanwhile, so they need an `OcctKernel`
  rather than any `&dyn BrepKernel`. Nothing else was missing: DOC-06 uses `make_box`,
  `make_planar_faces`, `prism`, `boolean`, `fillet`, `sub_geom`, `sub_geom_all`,
  `ShapeHistory::{compose, order_splits}`, `split_order_key`'s quantum, `Shape` adjacency
  (`edge_faces`, `vertex_edges`, `is_seam`), `surface`/`curve` for radii and
  `write_bin`/`read_bin` (index-stable round trip, relied on by the `.amg` cache).

### 2026-11-25: additions and changed contracts from BREP-15 (lane A)
Every change is inside lane A's crates. No other lane's files changed.
- `amg-brep`, additive:
  - `BrepKernel::{pipe, thru_sections}` with the Interfaces signatures. Every `BrepKernel`
    implementor must add them; the in-crate decorators forward them.
  - `KernelOp::{Pipe, ThruSections}` (S20 dump format, S20's field names).
  - `PipeMode::Support { shell }` (R-S04-7).
  - `impl Default` for `PipeOpts` and `ThruSectionsOpts`.
  - New module `sweep` and new bench `benches/sweep.rs` (`[[bench]]` in `amg-brep/Cargo.toml`).
  - `OpRole::LoftSection { index }` is now produced (role code 8).
- `amg-occt-sys`: new `cpp/occt_sweep.cpp` (in `build.rs`) with `pipe_op` and `thru_sections_op`
  (both C), and structs `OcPipeParams`, `OcLoftParams`, `OcLoftSection`. In `occt_history.hxx`,
  `Recorder::role` gains a default `arg` parameter and there is a new `LOFT_SECTION` code.
  `SHIM_VERSION` stays 4.
- **For S04 (SOL-03 sweep, SOL-10, SOL-11):**
  - Pipe operands are `[profiles..., spine, guide | support]`.
  - Twist and scale laws take `t` from 0 to 1 along the spine.
  - A support's spine must be curved edges of the support itself.
  - A twist cannot be combined with a scale (OCCT).
- **For S20 (KH-05, KH-08):** `thru_sections` is the raw baseline. Incompatible sections fail
  with `OpFailed`.
- No new dependency; `THIRD_PARTY.md` unchanged.

### 2026-11-25: additions and changed contracts from BREP-16 (lane A)
Every change is inside lane A's crates. No other lane's files changed.
- `amg-brep`, additive:
  - `BrepKernel::{draft, defeature, replace_face, split_faces}` with the Interfaces signatures,
    and the new `BrepKernel::offset_faces(s, &[(FaceId, f64)], ctx) -> KResult<OpResult>`
    (R-S04-5). Every `BrepKernel` implementor must add them; the in-crate decorators forward
    them.
  - `KernelOp::{Draft, Defeature, ReplaceFace, SplitFaces, OffsetFaces}` (S20 dump format).
  - New module `local`, new bench `benches/local.rs` (`[[bench]]` in `amg-brep/Cargo.toml`).
- `amg-occt-sys`: new `cpp/occt_local.cpp` (in `build.rs`) with `draft_op`, `defeature_op`,
  `trim_face_op`, `split_faces_op` (C) and `face_reach` (R), and the struct `OcDraftParams`.
  `offset_op` gains mode 3 (offset faces). `SHIM_VERSION` stays 4.
- **For S04:**
  - SOL-09 (offset face, press-pull): use `offset_faces`.
  - Delete face: use `defeature`.
  - Replace face: use `replace_face`.
  - Split face: use `split_faces`.
  - Split body: use BREP-04's `split` with a face or face set as the tool.
  - A positive draft angle removes material on the `pull` side of the neutral plane.
  - `offset_faces` and `defeature` take seconds on parts of about 1000 faces (BREP-16 bench).
- **For DOC-06:** the five histories are one-operand. Replace face is two steps composed into
  one history, and the replaced face is `Modified` into the new face.
- No new dependency; `THIRD_PARTY.md` unchanged.
