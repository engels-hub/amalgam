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
  - Replace face: `BRepAlgoAPI_Defeaturing`, then a boolean against the target surface's
    half-space, or `BRepTools_ReShape` with `ShapeFix`.
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
}
#[derive(Clone, Debug)]
pub struct OpResult { pub shape: Shape, pub history: ShapeHistory, pub warnings: Vec<KernelWarning> }
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
pub enum PipeMode { CorrectedFrenet, Frenet, Fixed { binormal: Dir3 }, Binormal(Dir3), Guide { wire: usize /* operand */, contact: bool } }
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
    // -- healing, assembly (BREP-10) --
    fn heal(&self, s: &Shape, opts: &HealOpts, ctx: &JobCtx) -> KResult<OpResult>;
    fn unify_same_domain(&self, s: &Shape, opts: &UnifyOpts) -> KResult<OpResult>;
    fn sew(&self, faces: &[Shape], opts: &SewOpts, ctx: &JobCtx) -> KResult<OpResult>;
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
| BREP-14 | Primitives (box, cylinder, cone, sphere, torus) on a `Plane` frame; `prism` (vector, infinite, taper, `merge_coplanar`); `revol`; `gtransformed` | BREP-03 | P0 | Todo | 1. Golden volume/area/topology counts for each primitive, for prism of 5 profile types and for revol (90°, 360°) within 1e-9 relative (`tests/golden/brep/`). 2. A tapered prism of a square at 5° matches the analytic frustum volume within 1e-9. 3. Results are BRepCheck-valid. 4. Prism and revol run in the stress test. |
| BREP-04 | Booleans: `boolean` (fuse/cut/common, N objects × M tools), `split`, `section`; non-destructive always on; fuzzy/glue/OBB/check-inverted; progress + cancel; parallel-mode determinism test and `occt_threads` benchmark | BREP-14 | P0 | Todo | 1. A 100-case boolean corpus (`testdata/brep/bool/`: primitives, coplanar, tangent, fuzzy-needed cases from OCCT/FreeCAD trackers) gives valid results, with volumes within 1e-6 relative of reference values (or the expected `Err`). 2. Proptest: 500 random primitive pairs give `Ok(valid)` or a clean `Err`, never a crash. 3. Inputs' BinTools hashes are unchanged after every op. 4. Determinism: 20 cases × 10 runs with parallel on are byte-identical, or parallel defaults off and the Refinement log says so. 5. Criterion: a cut of two 1000-face parts, timed with 1 / half / all OCCT threads; the default is set from the result. 6. Cancelling a long boolean returns `Cancelled` within 200 ms. |
| BREP-05 | History framework: `ShapeHistory` (entries, `Kept`, roles, tags, `sources_of`, `compose`), extraction from `BRepBuilderAPI_MakeShape`/`BRepTools_History`/op accessors, composition of internal steps, deterministic split order, `sub_geom`/`sub_geom_all` fingerprints; applied to BREP-03/14/04 ops | BREP-04 | P0 | Todo | 1. Completeness test: for every op in the BREP-03/14/04 corpora, every output face is Modified/Generated/Kept from some input or has a role, and no output index is out of range. 2. Prism of a tagged profile: each lateral face's `sources_of` is the generating edge, which carries the profile tag; caps have `StartCap`/`EndCap`. 3. Box cut by a cylinder through one face: the split face has 2 `Modified` outputs, in area-desc order, stable across 10 runs. 4. `compose` of prism + unify equals the history observed end-to-end on 5 cases. 5. `ShapeHistory` serde round-trips. 6. `sub_geom_all` on a 1000-face part takes < 20 ms. Usable by DOC-06 tests (box, prism, boolean cut). |
| BREP-06 | Fillet (constant, variable, chord) and chamfer (equal, two-dist, dist-angle) with history (`FilletBlend`, `ChamferFace`) and `LocalOpFailed { offending }`; `fillet_max_radius` | BREP-05 | P0 | Todo | 1. Golden volumes for 20 fillet/chamfer cases (cube edge, edge chains, vertex blends of 3 edges) within 1e-6 relative. 2. A radius larger than the adjacent face fails with `offending` naming the failing edge(s), checked against the expected edge ids. 3. `fillet_max_radius` on 5 cases is within 1 % of the reference largest workable radius. 4. History: each blend face maps to its source edge. 5. 10 runs are byte-identical. 6. The op is in the stress test. |
| BREP-07 | Shell / thick solid (removed faces, per-face thickness, join arc/intersection), `offset`, `thicken` with roles `ShellInner`/`OffsetFace` | BREP-05 | P0 | Todo | 1. Golden volumes: shelled box (1 face removed, 1 mm) equals the analytic volume within 1e-9 relative. Shelled cylinder, filleted box (arc join) and thickened B-spline face (0.5 %) pass. 2. Inward/outward offset of a sphere: radius within 1e-9. 3. A thickness exceeding the minimum radius gives `LocalOpFailed` with offending faces. 4. History roles are correct on 5 cases. |
| BREP-16 | Draft, defeature (delete face), replace face, split faces by tool, split body (via `split`) with history | BREP-05 | P0 | Todo | 1. Draft of 4 box side faces at 3° about the bottom plane: the measured face angles are 3° ± 1e-9 rad and the volume is analytic. 2. Deleting a fillet face and a boss face gives the pre-feature volume within 1e-6. 3. Replacing a top face with an offset plane changes the height exactly. 4. Splitting a face by a sketch wire gives 2 faces with the right areas. 5. Every result is BRepCheck-valid with complete history. |
| BREP-08 | Tessellation: `tessellate` on a private topology copy (category C), `TessMesh` (per-face vertices, normals, `face_ranges`, exact `weld`, edge polylines), `to_indexed_parts`, `tessellate_many` (rayon, `InParallel` off), `tessellate_edge`, quality presets | BREP-12 | P0 | Review (wp/BREP-08-tess; Done-when 1 not met, see BREP-18) | 1. A 1000-face fixture part tessellates (display preset) in < 100 ms median, including the topology copy (criterion `budget/brep-tess-1000`); the copy share is recorded. 2. `to_indexed_parts` of every closed fixture is watertight (`amg_testkit::metrics`: 0 boundary and 0 non-manifold edges), and its volume matches `mass_props` within the deflection bound. 3. Sphere r=10 tessellations converge to the exact area as deflection shrinks. 4. Tessellating one shared `Shape` from 8 threads while another thread runs booleans on it: no crash, identical meshes, input hash unchanged. 5. `face_of_triangle` is consistent with `face_ranges` for all triangles. |
| BREP-09 | Mass properties (volume, area, centroid, inertia, principal axes), `classify`, `distance`, `closest_point`, `ray_cast`, `eval_surface` (curvatures), `project_point_on_face`, `interference` | BREP-04 | P0 | Todo | 1. Box, cylinder, sphere and torus mass props match analytic values within 1e-9 relative. Inertia tensors match within 1e-9. 2. Distance between two spheres and between a point and a face matches analytic values within 1e-9. 3. `ray_cast` hits the expected face with the correct normal on 1000 random rays at a cylinder (vs analytic). 4. Curvatures of a torus at 10 points match analytic values within 1e-9. 5. `interference` volume of two overlapping boxes is analytic. |
| BREP-17 | Geometry extraction for sketches and drawings: `plane_section` and `project_edges` to `CurveSeg` (lines/circles/ellipses recognised, others B-spline), `tessellate_edge` reuse | BREP-12 | P0 | Todo | 1. A plane section of a cylinder (perpendicular) gives one `Curve3::Circle` with exact radius. An oblique section gives an `Ellipse` with analytic axes within 1e-9. 2. Projecting the edges of a filleted box top onto the XY plane gives lines and circle arcs with the correct radii and endpoints. 3. Section of a B-spline surface returns B-spline segments within 1e-6 mm of the true intersection (sampled). |
| BREP-10 | Healing and assembly: `heal` (`ShapeFix_Shape`, `ShapeFix_Wireframe` on a copy), `unify_same_domain`, `sew` (+ `make_solid`, orientation fix), `make_compound`, `check` (BRepCheck + ArgumentAnalyzer) | BREP-05 | P0 | Todo | 1. Sewing the 6 planar polygon faces of a cube (with 1e-4 mm gaps) gives a valid closed solid with volume within 1e-6. 2. Sewing the faces of 20 fixture parts exploded via `sub_shape` gives back solids with the same volume and face count. 3. `heal` fixes 10 corpus shapes with known defects (BRepCheck invalid → valid). 4. `unify_same_domain` after a fuse of two coplanar boxes yields 6 faces, and history maps the merged faces. 5. `check` lists the expected problem kinds on 5 broken fixtures. |
| BREP-13 | STEP (AP203/214/242) and IGES read/write via XCAF into `ForeignModel` (names, colours incl. per-face, assembly tree with shared prototypes, units → mm), heal on import, progress + cancel; IGES under the global lock | BREP-10 | P0 | Todo | 1. Round-trip of 10 STEP fixtures (≥ 1 assembly with reused parts, ≥ 1 coloured, ≥ 1 in inches): write then read again gives the same tree, names, colours, solid count, and per-solid volume within 1e-9 relative. Inch input is scaled to mm. 2. Reading the same STEP file on 4 threads in parallel gives identical results (on 8.0.0 the reads serialise through the global lock; after a bump to 8.0.1 the lock is removed and the test still passes). 3. IGES round-trip of 3 fixtures. 4. A 50 MB STEP read reports progress and cancels within 500 ms. 5. The owner's Fusion STEP exports (if present under `$AMG_TESTDATA_DIR/fusion/`) import as valid solids. |
| BREP-15 | Sweeps and raw loft: `pipe` (pipe-shell: Frenet/corrected/fixed/binormal/guide, twist/scale laws, transition, solid/shell), `thru_sections` (ruled/smooth, point sections, compatibility check) with roles `LoftSection`/`Lateral` | BREP-05 | P0 | Todo | 1. Golden volumes: a circle swept along a straight line and along an arc (torus segment) are analytic within 1e-6. A square along a helix is within 0.1 %. 2. A twisted sweep (90°) and a scaled sweep (1→2) match reference volumes. 3. ThruSections of 3 circles of different radii (smooth) is within 1e-4 relative of the analytic volume. Point-section cones work. 4. History: lateral faces map to the section edges. 5. Failures (incompatible sections) return `OpFailed` and never crash. KH-05 can call it as the baseline. |
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
