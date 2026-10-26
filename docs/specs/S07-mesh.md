# S07 — Mesh workspace

| | |
|---|---|
| **Status** | Ready |
| **Priority** | P0 (core of the owner's AM workflow) |
| **Crates** | `amg-mesh`, `amg-manifold-sys` (new FFI crate, MESH-05), `amg-features::mesh`, `amg-ui::mesh` |
| **Lane** | C |
| **Depends on** | S00 (FND-03 core types, FND-04 jobs/GPU, FND-06 testdata); S01 (DOC-02/05) for features; S02 (BREP-02/03/10) for mesh→B-rep; S03 (SK-08) for section sketch; S08 (IMP-03) for robust shell and wrap; S10 (RND-08) for overlays; S11 (UI-02..04, UI-06, UI-10) for UI |
| **Parity** | Mesh workspace section |

## Summary
High-performance triangle-mesh processing in Rust:
- data structures and spatial queries;
- analysis and repair;
- remeshing, decimation and smoothing;
- booleans (Manifold);
- cutting and shelling;
- face groups and primitive fitting;
- conversion to B-rep.

Mesh bodies are first-class bodies in the design, and every operation is a timeline feature
(or a direct edit). Algorithm-level detail, evidence and the corpus plan are in the design note
[`docs/notes/mesh-processing.md`](../notes/mesh-processing.md).

## Scope
**In:**
- `amg-mesh`:
  - `IndexedMesh`, `HalfEdgeMesh`, attributes, face groups, `MeshBody`;
  - BVH, ray / closest-point / winding-number queries;
  - analysis;
  - repair: weld, degenerates, non-manifold, orientation, hole fill, self-intersections;
  - remesh (isotropic + adaptive), QEM decimation, smoothing;
  - Manifold booleans;
  - plane cut, section, separate, erase & fill, reverse normals;
  - normal-offset shell;
  - isosurface extraction;
  - wall thickness;
  - face-group segmentation and primitive fitting;
  - soft transform.
- `amg-manifold-sys`: FFI to Manifold's C API.
- `amg-features::mesh`:
  - timeline features for all of the above;
  - robust SDF shell and wrap, which compose `amg-mesh` and `amg-implicit`;
  - mesh→B-rep (faceted, prismatic), which composes `amg-mesh` and `amg-brep`;
  - mesh section → sketch.
- `amg-ui::mesh`: Mesh workspace toolbar, tool dialogs and analysis panels.

**Out:**
- File formats (STL / 3MF / OBJ / PLY readers and writers) → S12 (`amg-io`). This spec defines
  the in-memory type they fill.
- Mesh SDF and dual-contouring mesher → S08 (`amg-implicit`).
- Overhang / support analysis → S16.
- Measure → S05 INS-01 (uses MESH-02/03).
- Section-plane capping in the viewport → S10 RND-06.
- Rendering of meshes and overlays → S10 (fed `RenderBuffers` / scalar fields by `amg-ui`).
- Quad remesh / SubD → S09.
- Tet meshing → S13.

## Design

### Decisions (details and evidence in the design note)
1. **f64 positions everywhere on the CPU** (`DVec3`, mm). This is not an exception to ADR-006.
   - 10 M triangles cost ≈ 280 MB as `IndexedMesh`; f32 would save only 60 MB of a ≈ 625 MB
     working set.
   - f32 cannot represent the 1e-6 mm default tolerance beyond ≈ 16 mm from the origin (ulp at
     1000 mm ≈ 6e-5 mm).
   - STL input is f32, so the conversion to f64 is lossless.
   - **f32 appears only** in (a) BVH node bounds, rounded outward (conservative, exact tests in
     f64), and (b) render upload buffers, rebased to a body-local origin.
2. **Indices are `u32`** (≤ 4.29 G vertices/faces, far above the 10 M-triangle target). Every
   constructor checks for overflow (`MeshError::TooLarge`).
3. **`IndexedMesh` is the interchange type.** It tolerates anything a file contains
   (non-manifold, duplicates, degenerates). `HalfEdgeMesh` is a compact *directed-edges*
   structure (only `twin` is stored per half-edge), built on demand for topology-changing ops.
   Non-manifold input converts with `NonManifoldPolicy::SplitVertices`.
4. **Own implementations** of BVH, queries, repair, remesh, decimation and smoothing on `glam`
   + `rayon` + `robust` (exact predicates) + `spade` (CDT) + `faer` (sparse solves). Evaluated
   crates and why each was rejected or kept as reference:
   - `obvhs`: f32-only, ray-tracing-oriented → benchmark reference;
   - `parry3d-f64`: heavy, own math types, no fast winding number → test oracle;
   - `baby_shark`: nalgebra-based, no attribute/group propagation → reference;
   - `meshopt`: f32, render-oriented → suggested to S10 for LODs;
   - `bvh`, `remesh`, `lin-alg`.
5. **Manifold via an own thin `amg-manifold-sys`** (hand-written `extern "C"` for `manifoldc`,
   `MeshGL64` f64 path). Manifold and oneTBB are built by vcpkg in manifest mode (overlay port
   if the upstream port lacks `MANIFOLD_CBIND`), pinned next to OCCT, with no git/CMake inside
   `build.rs`. `manifold3d` / `manifold-csg-sys` (MIT / Apache-2.0) is the fallback if the
   vcpkg route fails on MSVC. The cargo feature `amg-mesh/manifold` is **off by default**
   (MESH-05: a plain `cargo build` must work without the native library; the app enables it);
   without it, `boolean()` returns `MeshError::BackendUnavailable`. **MESH-05 was gated by ADR-003 (accepted 2026-10-20).**
6. **Determinism.** Every op is bit-identical for the same input, regardless of rayon thread
   count:
   - ordered reductions;
   - priority queues tie-broken by ids;
   - RANSAC seeded with `ChaCha8Rng`;
   - parallel outputs sorted or keyed by grid/edge ids.

   Manifold+TBB determinism is verified in MESH-05; if it fails, the output is canonically
   sorted.
7. **Everything runs as a job.** Ops take `&JobCtx` (S00), report progress, check cancellation
   at least every ≈ 10 ms of work, and return `MeshError::Cancelled`. Nothing in `amg-mesh`
   touches the UI thread. `amg-ui::mesh` spawns ops via `JobSystem` and polls `JobHandle`s.

### Data structures
- **`IndexedMesh`** has four parts:
  - `positions: Vec<DVec3>`;
  - `tris: Vec<[u32;3]>`;
  - `attrs: MeshAttrs`, holding vertex normals and colours, corner UVs, and per-face 3MF
    property references;
  - `groups: FaceGroups`, holding a per-face `FaceGroupId` and a group table with origin and an
    optional fitted primitive.

  Attribute policy: vertex attributes are interpolated, corner attributes are interpolated
  inside the parent face, face attributes and groups are inherited from the parent face, and
  new cap/fill faces get a fresh group.
- **`HalfEdgeMesh`**:
  - half-edge `3f+k` of face `f`, so `next`/`prev` are arithmetic;
  - `twin: Vec<u32>`;
  - one outgoing half-edge per vertex (the boundary one if the vertex is on the boundary);
  - tombstones plus `garbage_collect()`.

  Operations: `split_edge`, `collapse_edge` (link condition), `flip_edge`, `add_face`
  (hole fill).
- **`MeshBody`** (stored by `amg-doc` in `BodyKind::Mesh`) is a cheap-clone `Arc<IndexedMesh>`
  plus lazily built derived data (`MeshSpatial`, `AnalysisReport`) in `OnceLock`s, and a
  versioned little-endian binary encoding for `.amg` `bodies/` and `cache/`.
- **Persistent naming (S01):** a mesh `EntityRef` = `FaceGroupId` + `GroupFingerprint` (area,
  centroid, mean normal, AABB, primitive kind). Group ids come from a monotonic counter and are
  never reused within a body lineage. Every op propagates them from parent faces. Booleans map
  them through Manifold's `runOriginalID` / `faceID`.

### Spatial queries
- `Bvh`: binned SAH (16 bins), parallel top-level build, ≤ 4 tris/leaf, 32-byte flattened
  nodes with f32 bounds.
- `MeshSpatial` (`Arc<IndexedMesh>` + `Bvh` + lazily built fast-winding-number tree) answers:
  - watertight f64 ray cast (Woop 2013, robust traversal per Ize 2013);
  - closest point with sub-feature (vertex/edge/face);
  - fast generalized winding number (Barill et al. 2018, order-2 dipoles, β = 2);
  - exact winding number (reference);
  - box query;
  - signed distance (closest point + winding-number sign).

### Analysis
One parallel pass over a vertex-to-corner table (no edge sort; see the MESH-02 log) gives:
- boundary loops;
- non-manifold edges and vertices;
- orientation conflicts;
- shells (union-find);
- degenerate and duplicate faces;
- area and signed volume per shell;
- AABB;
- `is_closed`, `is_manifold`, `is_oriented`, `is_inverted`.

Self-intersection count is opt-in (MESH-19).

### Repair (MESH-04 pipeline; MESH-19/20/21 building blocks; MESH-18 wrap)
Steps, in order: weld (quantised radix sort + exact check) → degenerate/duplicate removal →
non-manifold resolution → small-shell removal → orientation (BFS per shell, outward by signed
volume or winding number) → hole fill (Liepa 2003: min-weight DP triangulation, refinement,
harmonic / bi-harmonic fairing with `faer`) → self-intersection removal (local delete + refill,
Attene 2010 style, ≤ `max_rounds`) → re-weld and re-analyse. The pipeline repeats while defects
decrease.

When the topology is hopeless, **Wrap** (MESH-18) is a morphological closing through the SDF:
dilate by unsigned distance, keep the outer component, erode by signed distance. It always
yields a watertight envelope.

### Remesh, reduce, smooth
- **Remesh (MESH-06):** isotropic Botsch–Kobbelt 2004 — split > 4/3 L, collapse < 4/5 L, flip
  toward valence 6, tangential relax, project to input. Feature edges (dihedral > 45° by
  default, plus group borders and boundaries when requested) are constrained.
- **Adaptive remesh (MESH-22):** curvature sizing L = √(6ε/κ − 3ε²), per Dunyach et al. 2013.
- **Reduce (MESH-07):** Garland–Heckbert 1997 QEM with optimal placement and boundary /
  group-border / feature penalty quadrics. Collapses that flip a normal, break quality or fail
  the link condition are rejected. Deterministic queue. Attribute-aware quadrics (Hoppe 1999)
  are optional.
- **Smooth (MESH-08):** Laplacian (uniform / cotangent), Taubin λ|μ (0.5 / −0.53) and bilateral
  normal filtering (Zheng et al. 2011).

### Booleans (MESH-05, ADR-003)
`IndexedMesh` → `MeshGL64` → Manifold boolean / batch boolean → `MeshGL64` → `IndexedMesh`,
with groups mapped back. Non-closed or non-manifold operands are auto-repaired when
`BoolOpts::repair_inputs` is set; otherwise a clear error is returned. Manifold calls are not
interruptible, so cancellation is checked between stages.

### Cut, shell, misc (MESH-09, MESH-10, MESH-17)
- **Plane cut:** f64 plane classification with vertex snapping inside `snap_tolerance`, edge
  splitting, loop extraction, `spade` CDT caps, and a new `Cap` group. `section()` returns the
  polylines (also used by MESH-14 and INS-02).
- **Separate shells, erase & fill, reverse normals.**
- **Shell, quick** (`amg-mesh`): angle-weighted vertex-normal offset plus stitched rim. It may
  self-intersect, and its report says how often.
- **Shell, robust** (`amg-features::mesh::shell`): the field
  f(p) = max(s(p), −(s(p) + t)) uses s = `amg_implicit::MeshSdf` (IMP-03) and is extracted
  with `amg_mesh::iso` (MESH-17), then decimated. The result is always closed, manifold and
  free of self-intersections.
- **`amg_mesh::iso` (MESH-17):** sparse marching tetrahedra (Kuhn 6-tet split, symbolic
  perturbation at iso = value, outside-of-bounds forced positive). It guarantees a closed,
  oriented 2-manifold for any `ScalarField`. It is generic: it knows nothing about SDF graphs.

### No dependency cycle between `amg-mesh` and `amg-implicit`
The architecture says `amg-implicit` depends on `amg-mesh`, never the reverse. MESH-10 (robust
shell) and MESH-18 (wrap) need both, so:
- `amg-mesh` defines the trait **`amg_mesh::iso::ScalarField`** and the extractor (MESH-17).
  It knows nothing about `amg-implicit`.
- `amg-implicit` (IMP-03) builds `MeshSdf` on `amg_mesh::MeshSpatial`, and **implements
  `amg_mesh::iso::ScalarField` for `MeshSdf`**. That is legal, because implicit depends on
  mesh.
- The composition (field formula, extraction, decimation) lives in **`amg-features::mesh`**
  (`shell.rs`, `wrap.rs`). `amg-features` may depend on `doc` + all domain crates. These
  modules are plain functions over `amg-mesh` / `amg-implicit` types (no `amg-doc` types in
  their signatures), so they are unit-testable without a document.
- Later, `amg-implicit`'s own mesher (IMP-04 dual contouring) can replace MESH-17 inside these
  functions without any `amg-mesh` change.

### Analysis overlays, face groups, mesh→B-rep, section sketch, direct edit
- **Wall thickness (MESH-11):** per vertex, a cone of inward rays (1 + 8, 15°), minimum hit
  distance excluding the 1-ring. GPU path: WGSL BVH traversal on the shared `Gpu` (FND-04).
  CPU fallback via `MeshSpatial`. The result is a per-vertex `Vec<f64>`, which `amg-ui::mesh`
  feeds to the RND-08 scalar-field overlay (`amg-render` never sees `amg-mesh` types).
- **Face groups (MESH-12):** curvature-ordered region growing by normal deviation and curvature,
  plus merging of small regions.
- **Primitives (MESH-24):** Efficient RANSAC (Schnabel 2007) for plane, sphere, cylinder, cone
  and torus, with Gauss–Newton refinement (Lukács 1998). Seeded and deterministic.
- **Mesh→B-rep** (`amg-features::mesh::to_brep`, which composes `amg-mesh` and `amg-brep`;
  `amg-mesh` never sees B-rep and no OCCT types leave `amg-brep`):
  - **faceted (MESH-13):** one planar polygon face per coplanar group, sewn (BREP-10), solid
    if closed;
  - **prismatic (MESH-25):** a fitted analytic surface per group, with boundaries from
    neighbouring-surface intersection, trimmed and sewn.

  Both need two `BrepKernel` calls from S02 (see Interfaces; requested additively).
- **Section sketch (MESH-14):** `section()` polylines → Douglas–Peucker → line / arc / spline
  fit within tolerance → sketch entities (S03 API).
- **Direct edit (MESH-15):** soft transform of selected faces / groups with radial or geodesic
  falloff, driven by UI-06 manipulators.

### UI (`amg-ui::mesh`)
The Mesh workspace has these tool groups:
- **Modify:** Repair, Remesh, Reduce, Smooth, Plane Cut, Shell, Combine, Separate, Reverse
  Normal, Erase & Fill, Wrap.
- **Prepare:** Generate Face Groups, Convert Mesh.
- **Inspect:** Mesh Analysis panel (report + highlight of defects), Wall Thickness.

Phase 1 dialogs are hand-built on the framework's public API (UI-02 command registry, UI-03
tool state machine, UI-04 selection, UI-10 progress). The generic feature dialog renderer
(UI-05) only arrives in Phase 2 (it depends on SOL-01), so the Phase 1 dialogs migrate to it
later. See the Refinement log.

Previews run as cancellable jobs on a decimated proxy when the input has more than 2 M
triangles.

## Interfaces
Signatures are what other crates (`amg-io`, `amg-doc`, `amg-implicit`, `amg-fem`, `amg-am`,
`amg-subd`, `amg-features`, `amg-ui`) rely on. Types from `amg-core` (S00): `Point3 = DVec3`,
`Vec3`, `Transform` (`DAffine3`), `Aabb3`, `Plane`, `Ray`, `Tolerance`. From `amg-jobs`:
`JobCtx`, `Gpu`.

```rust
// ───────────────────────── amg_mesh (lib.rs re-exports) ─────────────────────────
use std::sync::{Arc, OnceLock};
use glam::{DVec2, DVec3};
use amg_core::{Aabb3, Plane, Ray, Transform};
use amg_jobs::{Gpu, JobCtx};

pub const INVALID: u32 = u32::MAX;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(transparent)] pub struct VertexId(pub u32);
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(transparent)] pub struct FaceId(pub u32);
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(transparent)] pub struct HalfEdgeId(pub u32);

// ── errors ──
#[derive(Debug, thiserror::Error)]
pub enum MeshError {
    #[error("mesh is empty")]
    Empty,
    #[error("vertex index {index} out of range (mesh has {len} vertices)")]
    IndexOutOfRange { index: u32, len: usize },
    #[error("mesh too large: {count} elements exceed the u32 index range")]
    TooLarge { count: usize },
    #[error("attribute `{name}` has {got} entries, expected {expected}")]
    AttributeLength { name: &'static str, got: usize, expected: usize },
    #[error("mesh is not manifold: {edges} non-manifold edges, {vertices} non-manifold vertices")]
    NonManifold { edges: usize, vertices: usize },
    #[error("mesh is not closed: {boundary_loops} boundary loops")]
    NotClosed { boundary_loops: usize },
    #[error("invalid option `{name}`: {reason}")]
    InvalidOption { name: &'static str, reason: String },
    #[error("topology operation rejected: {0}")]
    Topology(#[from] TopoError),
    #[error("boolean failed: {status}")]
    Boolean { status: String },
    #[error("boolean backend unavailable (amg-mesh built without the `manifold` feature)")]
    BackendUnavailable,
    #[error("GPU error: {0}")]
    Gpu(String),
    #[error("corrupt mesh data: {0}")]
    Decode(String),
    #[error("operation cancelled")]
    Cancelled,
}
pub type MeshResult<T> = Result<T, MeshError>;
impl From<MeshError> for amg_core::Error { fn from(e: MeshError) -> Self { /* user msg + debug ctx */ } }

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum TopoError {
    #[error("half-edge is a boundary edge")]          BoundaryEdge,
    #[error("collapse violates the link condition")]  LinkCondition,
    #[error("flip would create a duplicate edge")]     DuplicateEdge,
    #[error("element was deleted")]                    Dead,
}

// ── attributes & face groups ──
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct Rgba8(pub [u8; 4]);                          // sRGB + linear alpha

/// 3MF triangle property: property group id + per-corner index (all equal when uniform).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct TriProperty { pub group: u32, pub index: [u32; 3] }

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MeshAttrs {
    pub vertex_normals: Option<Vec<DVec3>>,   // len == positions.len()
    pub vertex_colors:  Option<Vec<Rgba8>>,   // len == positions.len()
    pub corner_uvs:     Option<Vec<[DVec2; 3]>>, // len == tris.len()
    pub tri_properties: Option<Vec<TriProperty>>, // len == tris.len()
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
#[repr(transparent)] pub struct FaceGroupId(pub u32);

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum GroupOrigin {
    Import { object: u32 },
    Segmentation,
    Boolean { operand: u8, source: FaceGroupId },
    Cap,
    Fill,
    Shell { side: ShellSide },
    User,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum PrimitiveKind { Plane, Sphere, Cylinder, Cone, Torus }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Primitive {
    Plane    { origin: DVec3, normal: DVec3 },
    Sphere   { center: DVec3, radius: f64 },
    Cylinder { origin: DVec3, axis: DVec3, radius: f64 },
    Cone     { apex: DVec3, axis: DVec3, half_angle: f64 },
    Torus    { center: DVec3, axis: DVec3, major: f64, minor: f64 },
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FaceGroupInfo { pub name: Option<String>, pub origin: GroupOrigin, pub primitive: Option<Primitive> }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GroupFingerprint {
    pub area: f64, pub centroid: DVec3, pub mean_normal: DVec3, pub aabb: Aabb3,
    pub primitive: Option<PrimitiveKind>,
}

#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FaceGroups {
    per_face: Vec<FaceGroupId>,                         // len == tris.len(); default group 0
    info: std::collections::BTreeMap<FaceGroupId, FaceGroupInfo>,
    next_id: u32,                                       // monotonic, never reused
}
impl FaceGroups {
    pub fn group_of(&self, f: FaceId) -> FaceGroupId;
    pub fn set(&mut self, f: FaceId, g: FaceGroupId);
    pub fn alloc(&mut self, info: FaceGroupInfo) -> FaceGroupId;
    pub fn info(&self, g: FaceGroupId) -> Option<&FaceGroupInfo>;
    pub fn ids(&self) -> impl Iterator<Item = FaceGroupId> + '_;
    pub fn faces_of(&self, g: FaceGroupId) -> impl Iterator<Item = FaceId> + '_;
    pub fn fingerprint(&self, mesh: &IndexedMesh, g: FaceGroupId) -> Option<GroupFingerprint>;
}

// ── IndexedMesh ──
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct IndexedMesh {
    pub positions: Vec<DVec3>,
    pub tris: Vec<[u32; 3]>,
    pub attrs: MeshAttrs,
    pub groups: FaceGroups,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WeldOpts { pub tolerance: f64 /* mm; 0.0 = exact match; repair default = Tolerance::mesh().linear (1e-5 mm, S00) */ }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NormalWeighting { Uniform, Area, Angle }

/// Vertex remap produced by compaction/welding: old index → new index (INVALID = dropped).
#[derive(Clone, Debug, Default)]
pub struct Remap { pub vertices: Vec<u32>, pub faces: Vec<u32> }

/// f32 buffers for `amg-render` (rebased to `origin` before the cast). Face order is preserved,
/// so GPU `primitive_index` == face index for picking.
pub struct RenderBuffers { pub origin: DVec3, pub positions: Vec<[f32; 3]>, pub normals: Vec<[f32; 3]>, pub indices: Vec<u32> }

impl IndexedMesh {
    pub fn new(positions: Vec<DVec3>, tris: Vec<[u32; 3]>) -> MeshResult<Self>; // validates
    pub fn from_soup(tris: &[[DVec3; 3]], weld: WeldOpts) -> MeshResult<Self>;
    pub fn validate(&self) -> MeshResult<()>;               // index ranges + attribute lengths
    pub fn vertex_count(&self) -> usize;
    pub fn face_count(&self) -> usize;
    pub fn triangle(&self, f: FaceId) -> [DVec3; 3];
    pub fn face_normal(&self, f: FaceId) -> DVec3;          // unit; ZERO if degenerate
    pub fn face_area(&self, f: FaceId) -> f64;
    pub fn aabb(&self) -> Aabb3;
    pub fn area(&self) -> f64;
    pub fn signed_volume(&self) -> f64;                     // meaningful only if closed+oriented
    pub fn vertex_normals(&self, w: NormalWeighting) -> Vec<DVec3>;
    pub fn transform(&mut self, t: &Transform);
    pub fn append(&mut self, other: &IndexedMesh);          // re-allocates other's group ids
    pub fn weld(&mut self, opts: WeldOpts) -> Remap;
    pub fn compact(&mut self) -> Remap;                     // drop unreferenced vertices
    pub fn content_hash(&self) -> u64;                      // stable within a build (feature cache key)
    pub fn render_buffers(&self, origin: DVec3, smooth_normals: bool) -> RenderBuffers;
}

// ── HalfEdgeMesh ──
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NonManifoldPolicy { Fail, SplitVertices }

#[derive(Clone, Debug, Default)]
pub struct ConversionReport { pub split_vertices: usize, pub cut_edges: usize, pub dropped_degenerate: usize }

pub struct HalfEdgeMesh { /* positions, tris, twin, vert_he, alive flags, attrs, groups */ }
impl HalfEdgeMesh {
    pub fn from_indexed(m: &IndexedMesh, policy: NonManifoldPolicy) -> MeshResult<(Self, ConversionReport)>;
    pub fn to_indexed(&self) -> IndexedMesh;                // compacts
    pub fn position(&self, v: VertexId) -> DVec3;
    pub fn set_position(&mut self, v: VertexId, p: DVec3);
    pub fn next(&self, h: HalfEdgeId) -> HalfEdgeId;
    pub fn prev(&self, h: HalfEdgeId) -> HalfEdgeId;
    pub fn twin(&self, h: HalfEdgeId) -> Option<HalfEdgeId>;
    pub fn origin(&self, h: HalfEdgeId) -> VertexId;
    pub fn target(&self, h: HalfEdgeId) -> VertexId;
    pub fn face(&self, h: HalfEdgeId) -> FaceId;
    pub fn is_boundary_vertex(&self, v: VertexId) -> bool;
    pub fn outgoing(&self, v: VertexId) -> impl Iterator<Item = HalfEdgeId> + '_; // one-ring, CCW
    pub fn valence(&self, v: VertexId) -> usize;
    pub fn boundary_loops(&self) -> Vec<Vec<HalfEdgeId>>;
    pub fn split_edge(&mut self, h: HalfEdgeId, p: DVec3) -> Result<VertexId, TopoError>;
    pub fn can_collapse(&self, h: HalfEdgeId) -> bool;
    pub fn collapse_edge(&mut self, h: HalfEdgeId, p: DVec3) -> Result<VertexId, TopoError>;
    pub fn flip_edge(&mut self, h: HalfEdgeId) -> Result<(), TopoError>;
    pub fn add_vertex(&mut self, p: DVec3) -> VertexId;
    pub fn add_face(&mut self, v: [VertexId; 3], group: FaceGroupId) -> Result<FaceId, TopoError>;
    pub fn delete_face(&mut self, f: FaceId);
    pub fn garbage_collect(&mut self) -> Remap;
}

// ── MeshBody (stored by amg-doc in BodyKind::Mesh) ──
#[derive(Clone)]
pub struct MeshBody { mesh: Arc<IndexedMesh>, spatial: Arc<OnceLock<MeshSpatial>>, report: Arc<OnceLock<AnalysisReport>> }
impl MeshBody {
    pub fn new(mesh: IndexedMesh) -> Self;
    pub fn mesh(&self) -> &Arc<IndexedMesh>;
    pub fn spatial(&self, ctx: &JobCtx) -> MeshResult<&MeshSpatial>;   // built once
    pub fn analysis(&self, ctx: &JobCtx) -> MeshResult<&AnalysisReport>;
    pub fn write_binary<W: std::io::Write>(&self, w: W) -> std::io::Result<()>;   // "AMGM" v1, LE
    pub fn read_binary<R: std::io::Read>(r: R) -> MeshResult<Self>;
}

// ── spatial (MESH-03) ──
#[derive(Clone, Copy, Debug)]
pub struct BvhOpts { pub max_leaf_size: u32 /* 4 */, pub sah_bins: u32 /* 16 */ }

pub struct Bvh { /* nodes: Vec<BvhNode /*32 B, f32 bounds rounded outward*/>, prim_indices: Vec<u32> */ }
impl Bvh {
    pub fn build(prim_bounds: &[Aabb3], opts: &BvhOpts, ctx: &JobCtx) -> MeshResult<Self>;
    pub fn node_count(&self) -> usize;
    pub fn query_aabb(&self, b: &Aabb3, visit: &mut dyn FnMut(u32));
    /// All prim pairs whose bounds overlap (self-traversal), visited in deterministic order.
    pub fn overlapping_pairs(&self, visit: &(dyn Fn(u32, u32) + Sync), ctx: &JobCtx) -> MeshResult<()>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit { pub t: f64, pub face: FaceId, pub bary: DVec3 }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriFeature { Vertex(u8), Edge(u8), Face }

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosestPoint { pub point: DVec3, pub face: FaceId, pub bary: DVec3, pub dist_sq: f64, pub feature: TriFeature }

pub struct MeshSpatial { /* mesh: Arc<IndexedMesh>, bvh: Bvh, winding: OnceLock<WindingTree> */ }
impl MeshSpatial {
    pub fn build(mesh: Arc<IndexedMesh>, opts: &BvhOpts, ctx: &JobCtx) -> MeshResult<Self>;
    pub fn mesh(&self) -> &Arc<IndexedMesh>;
    pub fn bvh(&self) -> &Bvh;
    pub fn ray_cast(&self, ray: &Ray, t_max: f64) -> Option<RayHit>;
    pub fn ray_cast_all(&self, ray: &Ray, t_max: f64, out: &mut Vec<RayHit>);   // sorted by t
    pub fn closest_point(&self, p: DVec3, max_dist: f64) -> Option<ClosestPoint>;
    pub fn faces_in_aabb(&self, b: &Aabb3, out: &mut Vec<FaceId>);
    /// Fast generalized winding number (Barill et al. 2018), accuracy parameter β (default 2).
    pub fn winding_number(&self, p: DVec3) -> f64;
    pub fn winding_number_with_beta(&self, p: DVec3, beta: f64) -> f64;
    pub fn winding_number_exact(&self, p: DVec3) -> f64;                        // O(n) reference
    pub fn is_inside(&self, p: DVec3) -> bool { self.winding_number(p) > 0.5 }
    /// |d| from closest point, sign from winding number; None if farther than `max_dist`.
    pub fn signed_distance(&self, p: DVec3, max_dist: f64) -> Option<f64>;
}

// ── analysis (MESH-02) ──
#[derive(Clone, Debug)]
pub struct AnalyzeOpts { pub degenerate_area: f64 /* mm² */, pub self_intersections: bool }
impl Default for AnalyzeOpts { fn default() -> Self { Self { degenerate_area: 1e-12, self_intersections: false } } }

#[derive(Clone, Debug, PartialEq)]
pub struct ShellInfo { pub faces: usize, pub area: f64, pub signed_volume: f64, pub closed: bool, pub aabb: Aabb3 }

#[derive(Clone, Debug, PartialEq)]
pub struct AnalysisReport {
    pub vertices: usize, pub faces: usize, pub unreferenced_vertices: usize,
    pub shells: Vec<ShellInfo>, pub shell_of_face: Vec<u32>,
    pub boundary_loops: Vec<Vec<u32>>,               // vertex loops
    pub non_manifold_edges: Vec<[u32; 2]>, pub non_manifold_vertices: Vec<u32>,
    pub orientation_conflicts: usize,
    pub degenerate_faces: Vec<FaceId>, pub duplicate_faces: Vec<FaceId>,
    pub self_intersecting_pairs: Option<Vec<[FaceId; 2]>>,
    pub area: f64, pub volume: Option<f64>, pub aabb: Aabb3,
    pub is_closed: bool, pub is_manifold: bool, pub is_oriented: bool, pub is_inverted: bool,
}
impl AnalysisReport { pub fn is_watertight(&self) -> bool { self.is_closed && self.is_manifold && self.is_oriented } }
pub fn analyze(m: &IndexedMesh, opts: &AnalyzeOpts, ctx: &JobCtx) -> MeshResult<AnalysisReport>;

// ── self-intersection (MESH-19) ──
#[derive(Clone, Debug, Default)]
pub struct SelfIntersectOpts { pub max_pairs: Option<usize> }
pub fn self_intersections(s: &MeshSpatial, opts: &SelfIntersectOpts, ctx: &JobCtx) -> MeshResult<Vec<[FaceId; 2]>>; // sorted

// ── repair (MESH-04, MESH-20, MESH-21) ──
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fairing { None, Harmonic, Biharmonic }

#[derive(Clone, Debug)]
pub struct HoleFillOpts { pub max_boundary_edges: usize, pub max_dp_edges: usize /* 250 */, pub refine: bool, pub density_factor: f64 /* √2 */, pub fairing: Fairing }

#[derive(Clone, Debug, Default)]
pub struct HoleFillReport { pub filled: usize, pub skipped_too_large: usize, pub failed: usize /* rolled back, not expected */, pub new_faces: usize, pub new_group: Option<FaceGroupId> }
/// Same on an `IndexedMesh` (splits non-manifold vertices first). Added by MESH-20.
pub fn fill_holes_indexed(m: &IndexedMesh, opts: &HoleFillOpts, ctx: &JobCtx) -> MeshResult<(IndexedMesh, HoleFillReport)>;

/// A shell is removed if it has <= max_faces faces and its extent (abs. signed volume if closed, else AABB volume)
/// is <= max_volume_fraction x the largest shell's; the largest shell always stays.
#[derive(Clone, Debug)]
pub struct SmallShellFilter { pub max_faces: usize, pub max_volume_fraction: f64 }

#[derive(Clone, Debug)]
pub struct RepairOpts {
    pub weld: Option<WeldOpts>,
    pub remove_degenerate: bool,
    pub remove_duplicates: bool,
    pub fix_non_manifold: bool,
    pub remove_small_shells: Option<SmallShellFilter>,
    pub orient: bool,
    pub fill_holes: Option<HoleFillOpts>,
    pub remove_self_intersections: bool,
    pub max_rounds: u32,                  // default 3
}
/// Default (MESH-04): weld at Tolerance::mesh(), degenerate / duplicates / non-manifold on, small shells
/// { 12 faces, 1e-3 }, orient, hole fill with HoleFillOpts::default(), SI removal on, max_rounds 3.
impl Default for RepairOpts { fn default() -> Self; }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RepairStepKind { Weld, Degenerate, Duplicate, NonManifold, SmallShells, Orient, FillHoles, SelfIntersections }
#[derive(Clone, Debug)]
pub struct RepairStep { pub kind: RepairStepKind, pub round: u32, pub changed: usize, pub millis: u32 }
#[derive(Clone, Debug)]
pub enum UnresolvedIssue { HoleTooLarge { boundary_edges: usize }, SelfIntersections { pairs: usize }, NonManifold { edges: usize } }
#[derive(Clone, Debug)]
pub struct RepairReport { pub before: AnalysisReport, pub after: AnalysisReport, pub steps: Vec<RepairStep>, pub unresolved: Vec<UnresolvedIssue> }

pub fn repair(m: &IndexedMesh, opts: &RepairOpts, ctx: &JobCtx) -> MeshResult<(IndexedMesh, RepairReport)>;
pub fn orient_consistently(m: &mut IndexedMesh, ctx: &JobCtx) -> MeshResult<usize /* faces flipped */>;
pub fn fill_holes(m: &mut HalfEdgeMesh, opts: &HoleFillOpts, ctx: &JobCtx) -> MeshResult<HoleFillReport>;
pub fn remove_self_intersections(m: &IndexedMesh, max_rounds: u32, ctx: &JobCtx) -> MeshResult<(IndexedMesh, usize /* remaining pairs */)>;

// Global self-intersection removal for overlapping solids (MESH-29): cut the surface along the
// intersection curves at exact crossings, drop the patches the generalized winding number puts
// inside the solid. Output faces are pieces of input faces (vertices move by <= `perturbation`).
#[derive(Clone, Debug)]
pub struct GlobalSelfIntersectionOpts {
    pub probe_offset: f64,                // 0.01 of sqrt(face area): where the winding number is sampled
    pub winding_threshold: f64,           // 0.5
    pub winding_beta: f64,                // DEFAULT_WINDING_BETA
    pub perturbation: f64,                // 1e-9 of the bbox diagonal; 0 = off (exact contacts stay)
    pub allow_seams: bool,                // true: sheets that touch along the cut (open surfaces) are accepted
    pub min_drop_area: f64,               // 1e-6 of the area: patches below this are kept by candidate 2
    pub cleanup_max_pairs: usize,         // 100
    pub cleanup: Option<RemoveSelfIntersectionsOpts>, // MESH-21 on the pairs the cut leaves (4 rounds, loops <= 300 edges)
}
pub struct GlobalSelfIntersectionReport { /* pairs_before/pairs_cut/faces_cut/faces_fallback/patches/patches_dropped/
    faces_dropped/area_dropped/pairs_after_drop/health_before/health_after_drop/candidate/accepted/cleanup/pairs_after */ }
pub enum GlobalCandidate { None, Perturbed, Drop, DropLarge, CutOnly }
/// Faces share vertices by index: weld and orient (outward) first. Never opens a watertight input, never adds
/// a boundary loop; when no candidate qualifies the input is returned unchanged (`accepted == false`).
/// Attributes and groups are not carried over.
pub fn remove_self_intersections_global(m: &IndexedMesh, opts: &GlobalSelfIntersectionOpts, ctx: &JobCtx)
    -> MeshResult<(IndexedMesh, GlobalSelfIntersectionReport)>;

// The entry point for `repair` (MESH-04): MESH-21 first, MESH-29 when it leaves pairs or strays.
#[derive(Clone, Debug)]
pub struct AutoSelfIntersectionOpts { pub local: RemoveSelfIntersectionsOpts, pub global: GlobalSelfIntersectionOpts,
    pub max_deviation: Option<f64> /* Some(0.01) of the bbox diagonal, compared with the local deviation bound */ }
pub enum SelfIntersectionMethod { Unchanged, Local, Global }
pub struct AutoSelfIntersectionReport { pub method: SelfIntersectionMethod, pub pairs_before: usize, pub pairs_after: usize,
    pub local: RemoveSelfIntersectionsReport, pub global: Option<GlobalSelfIntersectionReport> }
pub fn remove_self_intersections_auto(m: &IndexedMesh, opts: &AutoSelfIntersectionOpts, ctx: &JobCtx)
    -> MeshResult<(IndexedMesh, AutoSelfIntersectionReport)>;

// ── remesh (MESH-06, MESH-22) ──
#[derive(Clone, Copy, Debug)]
pub enum EdgeLength { Absolute(f64), RelativeToMean(f64) }
#[derive(Clone, Copy, Debug)]
pub enum Sizing { Uniform(EdgeLength), Adaptive { tolerance: f64, min_length: f64, max_length: f64 } }
#[derive(Clone, Debug)]
pub struct RemeshOpts {
    pub sizing: Sizing,
    pub iterations: u32,                  // 5
    pub feature_angle: Option<f64>,       // radians; Some(45°) default; None = no feature edges
    pub preserve_boundary: bool,
    pub preserve_group_borders: bool,
    pub project_to_input: bool,
    pub selection: Option<Vec<FaceId>>,   // None = whole mesh
}
pub fn remesh(m: &IndexedMesh, opts: &RemeshOpts, ctx: &JobCtx) -> MeshResult<IndexedMesh>;

// ── decimate (MESH-07) ──
#[derive(Clone, Copy, Debug)]
pub enum DecimateTarget { FaceCount(usize), Ratio(f64), MaxError(f64 /* mm */) }
#[derive(Clone, Debug)]
pub struct DecimateOpts {
    pub target: DecimateTarget,
    pub max_error: Option<f64>,           // hard cap in mm, combined with target
    pub preserve_boundary: bool,
    pub preserve_group_borders: bool,
    pub feature_angle: Option<f64>,
    pub boundary_weight: f64,             // penalty-quadric weight, default 1e3
    pub min_triangle_quality: f64,        // 0..1, default 0.1
    pub preserve_attributes: bool,
}
#[derive(Clone, Debug)]
pub struct DecimateReport { pub faces_before: usize, pub faces_after: usize, pub max_quadric_error: f64, pub dropped_attributes: Vec<&'static str> }
pub fn decimate(m: &IndexedMesh, opts: &DecimateOpts, ctx: &JobCtx) -> MeshResult<(IndexedMesh, DecimateReport)>;

// ── smooth (MESH-08) ──
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LaplacianWeights { Uniform, Cotangent }
#[derive(Clone, Copy, Debug)]
pub enum SmoothMethod {
    Laplacian { lambda: f64, weights: LaplacianWeights },
    Taubin { lambda: f64, mu: f64 },                         // 0.5, −0.53
    BilateralNormal { sigma_normal: f64, normal_iterations: u32, vertex_iterations: u32 },
}
#[derive(Clone, Debug)]
pub struct SmoothOpts { pub method: SmoothMethod, pub iterations: u32, pub fix_boundary: bool, pub preserve_group_borders: bool, pub selection: Option<Vec<FaceId>> }
pub fn smooth(m: &IndexedMesh, opts: &SmoothOpts, ctx: &JobCtx) -> MeshResult<IndexedMesh>;

// ── booleans (MESH-05; feature "manifold") ──
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum BoolOp { Union, Difference, Intersection }
#[derive(Clone, Debug, Default)]
pub struct BoolOpts { pub repair_inputs: bool }
#[derive(Clone, Debug, Default)]
pub struct BoolReport { pub repaired_inputs: Vec<usize>, pub millis: u32 }
pub fn boolean(op: BoolOp, a: &IndexedMesh, b: &IndexedMesh, opts: &BoolOpts, ctx: &JobCtx) -> MeshResult<(IndexedMesh, BoolReport)>;
/// `inputs[0]` op all others (Manifold batch boolean); group ids of inputs[0] are preserved.
pub fn boolean_n(op: BoolOp, inputs: &[&IndexedMesh], opts: &BoolOpts, ctx: &JobCtx) -> MeshResult<(IndexedMesh, BoolReport)>;

// ── cut / section / misc (MESH-09) ──
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeepSide { Positive, Negative, Both }
#[derive(Clone, Debug)]
pub struct PlaneCutOpts { pub plane: Plane, pub keep: KeepSide, pub cap: bool, pub snap_tolerance: f64 }
#[derive(Clone, Debug)]
pub struct PlaneCutResult { pub positive: Option<IndexedMesh>, pub negative: Option<IndexedMesh>, pub cap_group: Option<FaceGroupId>, pub open_loops: usize }
#[derive(Clone, Debug, PartialEq)]
pub struct Polyline3 { pub points: Vec<DVec3>, pub closed: bool }

pub fn plane_cut(m: &IndexedMesh, opts: &PlaneCutOpts, ctx: &JobCtx) -> MeshResult<PlaneCutResult>;
pub fn section(m: &IndexedMesh, plane: &Plane, snap_tolerance: f64) -> Vec<Polyline3>;
pub fn separate_shells(m: &IndexedMesh) -> Vec<IndexedMesh>;          // ordered by descending area
pub fn erase_and_fill(m: &IndexedMesh, faces: &[FaceId], opts: &HoleFillOpts, ctx: &JobCtx) -> MeshResult<IndexedMesh>;
pub fn reverse_normals(m: &mut IndexedMesh, faces: Option<&[FaceId]>);

// ── quick shell (MESH-10, part 1) ──
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ShellSide { Inside, Outside, Both }
#[derive(Clone, Debug)]
pub struct NormalOffsetOpts { pub thickness: f64, pub side: ShellSide, pub open_faces: Vec<FaceId> /* removed to make an open shell */ }
#[derive(Clone, Debug)]
pub struct ShellReport { pub self_intersecting_pairs: usize }
pub fn offset_shell_normal(m: &IndexedMesh, opts: &NormalOffsetOpts, ctx: &JobCtx) -> MeshResult<(IndexedMesh, ShellReport)>;

// ── isosurface extraction (MESH-17) — generic, no amg-implicit dependency ──
pub trait ScalarField: Sync {
    fn value(&self, p: DVec3) -> f64;
    /// Batched evaluation; default loops over `value`.
    fn values(&self, ps: &[DVec3], out: &mut [f64]) { for (p, o) in ps.iter().zip(out) { *o = self.value(*p) } }
    /// Conservative: return `false` only if the field cannot reach `iso` inside `b`.
    fn may_cross(&self, b: &Aabb3, iso: f64) -> bool { let _ = (b, iso); true }
}
#[derive(Clone, Debug)]
pub struct IsoOpts { pub bounds: Aabb3, pub cell_size: f64, pub iso: f64, pub block_cells: u32 /* 16 */, pub project_steps: u32 /* 0..3 secant steps */ }
/// Sparse marching tetrahedra; output is always a closed, oriented 2-manifold (possibly empty or
/// multi-shell). Values == iso count as outside; outside `bounds` is forced outside.
pub fn extract_isosurface(f: &dyn ScalarField, opts: &IsoOpts, ctx: &JobCtx) -> MeshResult<IndexedMesh>;

// ── wall thickness (MESH-11) ──
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ComputeBackend { Auto, Cpu, Gpu }
#[derive(Clone, Debug)]
pub struct ThicknessOpts { pub max_distance: f64, pub cone_half_angle: f64 /* 15° */, pub rays_per_vertex: u32 /* 9 */, pub backend: ComputeBackend }
/// Per-vertex thickness in mm; `f64::INFINITY` where no opposite surface within max_distance.
pub fn wall_thickness(s: &MeshSpatial, opts: &ThicknessOpts, gpu: Option<&Gpu>, ctx: &JobCtx) -> MeshResult<Vec<f64>>;

// ── face groups & primitives (MESH-12, MESH-24) ──
#[derive(Clone, Debug)]
pub struct SegmentOpts { pub angle_threshold: f64, pub curvature_threshold: Option<f64>, pub min_group_faces: usize }
pub fn segment_face_groups(m: &mut IndexedMesh, opts: &SegmentOpts, ctx: &JobCtx) -> MeshResult<usize /* groups */>;
#[derive(Clone, Debug)]
pub struct FitOpts { pub kinds: Vec<PrimitiveKind>, pub epsilon: f64, pub normal_threshold: f64, pub min_support: usize, pub seed: u64, pub max_iterations: u32 }
#[derive(Clone, Debug)]
pub struct FitResult { pub primitive: Primitive, pub rms: f64, pub max_dev: f64, pub support: usize }
pub fn fit_primitive(points: &[DVec3], normals: &[DVec3], kind: PrimitiveKind, opts: &FitOpts) -> Option<FitResult>;
pub fn fit_group_primitives(m: &mut IndexedMesh, opts: &FitOpts, ctx: &JobCtx) -> MeshResult<Vec<(FaceGroupId, FitResult)>>;

// ── direct edit (MESH-15) ──
#[derive(Clone, Copy, Debug)]
pub enum Falloff { None, Linear { radius: f64 }, Smooth { radius: f64 }, Geodesic { radius: f64 } }
pub fn soft_transform(m: &IndexedMesh, faces: &[FaceId], t: &Transform, falloff: Falloff, ctx: &JobCtx) -> MeshResult<IndexedMesh>;
```

```rust
// ───────────── amg_manifold_sys (MESH-05) — raw FFI + thin safe layer, unsafe confined here ─────────────
// `ffi`: hand-written declarations for the manifoldc subset we use (Manifold 3.5.4, overlay port
// `ports/manifold`); build.rs checks every `manifold_*` name against the installed manifoldc.h.
// Feature `native` (enabled by `amg-mesh/manifold`) links manifoldc, manifold, Clipper2, tbb12.
pub mod ffi { /* ManifoldManifold, ManifoldMeshGL64, ManifoldManifoldVec, ManifoldMeshGL64Options,
    manifold_alloc_* / manifold_delete_*, manifold_meshgl64, manifold_meshgl64_w_options,
    manifold_of_meshgl64, manifold_as_original, manifold_empty, manifold_boolean,
    manifold_manifold_empty_vec, manifold_manifold_vec_push_back, manifold_batch_boolean,
    manifold_status, manifold_original_id, manifold_num_tri, manifold_get_meshgl64,
    manifold_meshgl64_{num_prop,vert_properties,tri_verts,merge_from_vert,merge_to_vert,
    run_index,run_original_id,face_id}(+ _length) */ }
pub enum OpType { Add, Subtract, Intersect }
pub enum Status { NoError, NotManifold, VertexIndexOutOfBounds, NonFiniteVertex, Other(i32) }
pub struct MeshGL64;   // owns a ManifoldMeshGL64, Send
impl MeshGL64 {
    pub fn new(vert_props: &[f64], n_props: usize, tri_verts: &[u64]) -> MeshGL64;
    pub fn with_merge(vert_props: &[f64], n_props: usize, tri_verts: &[u64], merge_from: &[u64], merge_to: &[u64]) -> MeshGL64;
    pub fn to_data(&self) -> MeshData;
}
pub struct MeshData { pub n_props: usize, pub vert_props: Vec<f64>, pub tri_verts: Vec<u64>, pub merge_from: Vec<u64>,
                      pub merge_to: Vec<u64>, pub run_index: Vec<u64>, pub run_original_id: Vec<u32>, pub face_id: Vec<u64> }
pub struct Manifold;   // owns a ManifoldManifold, Send + Sync
impl Manifold {
    pub fn from_mesh(mesh: &MeshGL64) -> Result<Manifold, Status>;   // + AsOriginal: fresh original id
    pub fn empty() -> Manifold;
    pub fn status(&self) -> Status;
    pub fn original_id(&self) -> i32;
    pub fn num_tri(&self) -> usize;
    pub fn boolean(&self, other: &Manifold, op: OpType) -> Result<Manifold, Status>;
    pub fn batch_boolean(ms: &[&Manifold], op: OpType) -> Result<Manifold, Status>;
    pub fn to_mesh(&self) -> MeshGL64;
}
```

```rust
// ───────────── amg_features::mesh (MESH-10, MESH-13, MESH-14, MESH-16, MESH-18, MESH-25..27) ─────────────
// Pure compositions (no amg-doc types in these signatures; unit-testable without a document):
pub mod shell {
    #[derive(Clone, Debug)]
    pub struct SdfShellOpts { pub thickness: f64, pub side: amg_mesh::ShellSide, pub voxel_size: f64, pub simplify_tolerance: Option<f64> }
    pub fn offset_shell_sdf(m: &std::sync::Arc<amg_mesh::IndexedMesh>, opts: &SdfShellOpts, ctx: &amg_jobs::JobCtx) -> Result<amg_mesh::IndexedMesh, MeshFeatureError>;
}
pub mod wrap {
    #[derive(Clone, Debug)]
    pub struct WrapOpts { pub voxel_size: f64, pub offset: Option<f64> /* default 1.5 × voxel */, pub simplify_tolerance: Option<f64> }
    pub fn wrap(m: &std::sync::Arc<amg_mesh::IndexedMesh>, opts: &WrapOpts, ctx: &amg_jobs::JobCtx) -> Result<amg_mesh::IndexedMesh, MeshFeatureError>;
}
pub mod to_brep {
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum ConvertMethod { Faceted, Prismatic }
    #[derive(Clone, Debug)]
    pub struct ConvertOpts { pub method: ConvertMethod, pub coplanar_angle: f64, pub fit_tolerance: f64 }
    pub fn mesh_to_brep(k: &dyn amg_brep::BrepKernel, m: &amg_mesh::IndexedMesh, opts: &ConvertOpts, ctx: &amg_jobs::JobCtx) -> Result<amg_brep::Shape, MeshFeatureError>;
}
pub mod section_sketch {
    pub fn section_to_sketch_curves(polys: &[amg_mesh::Polyline3], plane: &amg_core::Plane, tolerance: f64) -> Vec<amg_sketch::Curve2>;
}

#[derive(Debug, thiserror::Error)]
pub enum MeshFeatureError {
    #[error(transparent)] Mesh(#[from] amg_mesh::MeshError),
    #[error("signed distance field: {0}")] Sdf(String),      // from amg_implicit error (S08 owns its type)
    #[error("B-rep conversion: {0}")] Brep(String),          // from amg_brep::KernelError (S02)
    #[error("input body is not a mesh")] NotAMesh,
    #[error("referenced face group {0:?} no longer resolves")] UnresolvedGroup(amg_mesh::FaceGroupId),
}

// Timeline features (implement S01 `Feature`; registered via `inventory`). Numeric fields are
// `Expr` (S01) so they can reference parameters. Each takes a `BodyRef` input and either replaces
// the body (default) or creates a new one (`output: OutputMode`).
pub enum OutputMode { Replace, NewBody }
pub struct MeshRepairFeature    { pub body: BodyRef, pub opts: RepairParams,  pub output: OutputMode }
pub struct MeshReverseNormalFeature { pub body: BodyRef, pub groups: Option<Vec<MeshEntityRef>> }
pub struct MeshRemeshFeature    { pub body: BodyRef, pub params: RemeshParams, pub output: OutputMode }
pub struct MeshReduceFeature    { pub body: BodyRef, pub params: ReduceParams, pub output: OutputMode }
pub struct MeshSmoothFeature    { pub body: BodyRef, pub params: SmoothParams, pub output: OutputMode }
pub struct MeshPlaneCutFeature  { pub body: BodyRef, pub plane: PlaneRef, pub keep: KeepSideParam, pub cap: bool }
pub struct MeshShellFeature     { pub body: BodyRef, pub thickness: Expr, pub side: ShellSide, pub robust: bool, pub voxel: Option<Expr> }
pub struct MeshSeparateFeature  { pub body: BodyRef }
pub struct MeshEraseFillFeature { pub body: BodyRef, pub groups: Vec<MeshEntityRef> }
pub struct MeshWrapFeature      { pub body: BodyRef, pub voxel: Expr }
pub struct MeshCombineFeature   { pub target: BodyRef, pub tools: Vec<BodyRef>, pub op: amg_mesh::BoolOp, pub keep_tools: bool }
pub struct MeshFaceGroupsFeature{ pub body: BodyRef, pub params: SegmentParams, pub fit_primitives: bool }
pub struct MeshConvertFeature   { pub body: BodyRef, pub method: to_brep::ConvertMethod }   // → BodyKind::Solid
pub struct MeshSoftMoveFeature  { pub body: BodyRef, pub groups: Vec<MeshEntityRef>, pub transform: TransformExpr, pub falloff: FalloffParam }
/// Mesh sub-entity reference (S01 EntityRef payload for mesh bodies).
pub struct MeshEntityRef { pub group: amg_mesh::FaceGroupId, pub fingerprint: amg_mesh::GroupFingerprint }
```

**Expected from other specs** (owned there; listed here so the dependency is explicit):
- **S08 / IMP-03:**
  ```rust
  pub struct MeshSdf { /* Arc<MeshSpatial>, band */ }
  impl MeshSdf {
      pub fn new(spatial: Arc<amg_mesh::MeshSpatial>, band: f64) -> Self;
      pub fn signed(&self, p: DVec3) -> f64;
      pub fn unsigned(&self, p: DVec3) -> f64;
  }
  impl amg_mesh::iso::ScalarField for MeshSdf { … }  // may_cross via 1-Lipschitz bound
  ```
- **S02 (additive request for MESH-13/25):**
  ```rust
  fn make_planar_face(&self, outer: &[DVec3], holes: &[Vec<DVec3>]) -> KResult<Shape>;
  fn make_face_on_surface(&self, surf: &AnalyticSurface, loops: &[Vec<DVec3>], tol: f64) -> KResult<Shape>;
  ```
  Use `sew(&[Shape], tol)` from BREP-10.
- **S01:** `BodyKind::Mesh(amg_mesh::MeshBody)`, with mesh sub-entity refs carried as
  `MeshEntityRef`.
- **S12:** fills `IndexedMesh` including `MeshAttrs::tri_properties` (3MF), `corner_uvs`
  (OBJ) and `GroupOrigin::Import { object }`.

## Work packages
WPs are ≈ 1–3 days each. MESH-01..16 keep their IDs (some re-scoped); MESH-17..28 are new.
"Corpus" = the MESH-28 manifest set. Times are on the reference machine (vision doc), release
build, 8+ cores.

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| MESH-01 | `IndexedMesh`, `MeshAttrs`, `FaceGroups`, `HalfEdgeMesh` (directed edges, split/collapse/flip, GC), conversions, weld, `MeshBody` binary encoding | FND-03 | P0 | Done | Round-trip Indexed→HalfEdge→Indexed preserves positions, faces (up to compaction), attributes and group ids (property test, 1 000 random meshes). Fins / bow-ties convert with `SplitVertices`, and the counts are reported. Topology-edit property test keeps all half-edge invariants. 1 M-tri half-edge build < 100 ms. Weld of 1 M-tri soup < 80 ms. `MeshBody` binary round-trip is bit-exact. |
| MESH-28 | Broken-mesh corpus: `testdata/mesh/corpus.toml` manifest, LFS-committed redistributable set, `fetch.ps1/.sh` for fetch-only sets, seeded synthetic defect + benchmark generators, corpus test harness | FND-06, MESH-01 | P0 | Done | Manifest has ≥ 150 real models covering all 11 defect categories, each with SPDX licence, source URL and sha256. LFS total ≤ 300 MB. Generator reproduces each defect category from `(shape, defects, seed)`, plus 100 k / 1 M / 10 M benchmark meshes, bit-identically. The harness iterates the manifest under `cargo nextest` and skips fetch-only entries when they are absent. |
| MESH-29 | Global self-intersection removal for overlapping solids: generalized-winding-number (or mesh-arrangement) classification that drops inner sheets, as a fallback when MESH-21's local refill is rejected or exceeds its deviation budget | MESH-21 | P1 | Done | On the corpus models MESH-21 leaves unrepaired or repairs with Hausdorff > 1 % of bbox diagonal, ≥ 80 % become SI-free with Hausdorff to input ≤ 1 % of bbox diagonal. A watertight input never becomes non-watertight. Corpus test skips LFS pointers unless `AMG_REQUIRE_TESTDATA=1`. |
| MESH-02 | Analysis: shells, boundary loops, non-manifold edges/vertices, orientation, degenerate/duplicate, area/volume, `AnalysisReport` | MESH-01, MESH-28 | P0 | Done | Report matches the manifest's defect flags on 100 % of corpus entries and the synthetic generator's ground truth. 1 M-tri analysis < 100 ms; 10 M < 1.2 s. |
| MESH-03 | `Bvh` (binned SAH, parallel), `MeshSpatial`: ray cast, closest point, box query, fast winding number (Barill 2018), exact winding number, signed distance | MESH-01 | P0 | Done | 1 M-tri BVH build < 150 ms (and within 2× of `obvhs` PLOC in a dev-bench). Ray cast and closest point equal brute force on 10 k random queries (and agree with `parry3d-f64` in a dev-test). Fast winding number within 0.01 of exact at β = 2 for points ≥ 1 mean edge length off the surface. 1 M FWN queries on a 1 M mesh < 1.5 s. Results are identical across 1/4/16 rayon threads. |
| MESH-19 | Self-intersection detection (BVH self-traversal + exact `robust` predicates) | MESH-03 | P0 | Done | On 10 k-tri random soups, the pair set equals the O(n²) exact brute force. No false positives for edge/vertex-adjacent triangles on the corpus's clean models. Intersecting-spheres synthetic case is found. 1 M-tri check < 500 ms. |
| MESH-20 | Hole filling (Liepa: DP triangulation, refinement, harmonic/bi-harmonic fairing; CDT for planar loops) | MESH-02 | P0 | Done | All synthetic holes with boundary 3–2000 edges are filled; result is watertight with no new non-manifold elements. Patch mean edge length within 20 % of the surrounding edges. Fair-filled hole in a sphere (r = 20 mm) deviates < 2 % r. A 500-edge hole fills in < 200 ms. |
| MESH-21 | Self-intersection removal (local delete + k-ring + refill, iterative) | MESH-19, MESH-20 | P0 | Done | ≥ 85 % of corpus models flagged `self_intersections` and `expect_repairable` become SI-free. Median Hausdorff distance (output → input) over the repaired models ≤ 1 mean input edge length, per-model values reported (owner, 2026-10-23, QUESTIONS #51; was ≤ 1 % of bbox diagonal). A watertight input never becomes non-watertight. Remaining pairs are reported. |
| MESH-04 | Repair pipeline (weld, degenerate, duplicate, non-manifold, small shells, orient, holes, SI) + `RepairReport` | MESH-02, MESH-03, MESH-20, MESH-21 | P0 | Done | ≥ 95 % of `expect_repairable` corpus models become watertight (closed + manifold + oriented, outward), and ≥ 85 % are also SI-free. Repair is idempotent (second run changes nothing). No step increases any defect count. 1 M-tri typical repair < 3 s. |
| MESH-05 | Booleans via Manifold: `amg-manifold-sys` (vcpkg + overlay port, TBB), `boolean`/`boolean_n`, group mapping, input auto-repair, `manifold` feature flag. **Gate: ADR-003 (cleared 2026-10-20)** | MESH-01, MESH-04 | P0 | Review | Builds on Windows MSVC via vcpkg (Linux best-effort). Two 1 M-tri offset spheres union < 1 s end-to-end including conversion. Volume identity vol(A∪B)+vol(A∩B) = vol(A)+vol(B) within 1e-6 relative (property test, 200 random pairs). Output always watertight. Group ids survive (A kept, B re-allocated). 10 runs with TBB are bit-identical (or canonical sort documented). Without the feature: compiles, and `boolean` returns `BackendUnavailable`. |
| MESH-06 | Isotropic remesh (Botsch–Kobbelt) with feature-edge, boundary and group-border preservation, projection to input | MESH-01, MESH-03 | P0 | Done | On corpus watertight models and synthetic shapes: ≥ 95 % of triangles have min angle ≥ 30°; ≥ 90 % of edges within [0.8 L, 1.33 L]; Hausdorff to input ≤ 0.5 L. Cube feature edges stay within 1e-3 mm. Topology (genus, shells) unchanged. 1 M → ~1 M tris, 5 iterations, < 10 s. |
| MESH-22 | Adaptive remesh (curvature sizing field, tolerance ε) | MESH-06 | P1 | Done | Max deviation from input ≤ ε on synthetic surfaces (sphere, torus, displaced sphere). Triangle count ≤ 60 % of uniform remesh at the same max deviation. |
| MESH-07 | QEM decimation (boundary/group/feature penalties, flip and quality checks, deterministic) | MESH-01 | P0 | Done | 1 M → 100 k in < 2 s. Symmetric Hausdorff ≤ 0.2 % of bbox diagonal on the displaced-sphere benchmark. A closed manifold input stays closed and manifold. Group borders are kept when `preserve_group_borders`. Output is bit-identical across thread counts. |
| MESH-08 | Smoothing: Laplacian (uniform/cotan), Taubin, bilateral normal filtering | MESH-01 | P0 | Done | Taubin (50 iterations) on a noisy sphere (σ = 1 % r) shrinks volume by < 1 % and cuts RMS noise ≥ 3×. Bilateral on a noisy cube keeps feature dihedral angles within 5° of 90° and cuts RMS noise ≥ 3×. 1 M tris × 10 Taubin iterations < 300 ms. |
| MESH-09 | Plane cut with CDT capping, `section`, separate shells, erase & fill, reverse normals | MESH-04 | P0 | Done | Cutting a closed 1 M-tri mesh gives closed manifold halves whose volumes sum to the original within 1e-9 relative, including cuts exactly through vertices and edges, in < 300 ms. Cap faces are in one new `Cap` group. `separate_shells` count equals the analysis shell count. Erase & fill output is watertight. Reverse normals negates signed volume. |
| MESH-17 | Isosurface extraction: sparse marching tetrahedra over `ScalarField` (`amg_mesh::iso`) | MESH-01 | P0 | Done | Fuzz test over 1 000 random CSG fields (spheres, boxes, exact zeros on grid points, surfaces touching `bounds`): output is always a closed, oriented 2-manifold. Hausdorff to analytic sphere/torus ≤ cell size. 400³-effective narrow-band extraction of a 100 mm part < 2 s. Output is identical across thread counts. |
| MESH-10 | Shell/offset: quick normal offset (`amg-mesh`) + robust SDF shell (`amg-features::mesh::shell`, composes IMP-03 `MeshSdf` + MESH-17) | MESH-04, MESH-17, IMP-03 | P0 | Todo | Robust shell output is closed, manifold and SI-free (MESH-19 check) on 100 % of watertight corpus models. Measured wall thickness within ±1 voxel of target. Quick shell of 1 M tris < 300 ms, with its SI count reported. Robust shell of a 1 M-tri, 100 mm part at 0.25 mm voxel < 20 s. |
| MESH-18 | Wrap repair (SDF dilate→erode closing, `amg-features::mesh::wrap`) | MESH-17, MESH-07, IMP-03 | P1 | Todo | 100 % of corpus models (including `expect_repairable = false`) give a watertight, SI-free result. Hausdorff to input ≤ 2× (offset + voxel). 1 M-tri scan at 0.5 mm voxel < 30 s. |
| MESH-16 | Mesh bodies in the timeline: `BodyKind::Mesh(MeshBody)` integration, `MeshEntityRef` resolution (group id → fingerprint fallback), feature base (`OutputMode`), `MeshRepairFeature`, `MeshReverseNormalFeature` | DOC-02, DOC-05, MESH-04 | P0 | Done | Repair and Reverse Normal run as features: undo/redo restores the exact mesh hash; re-editing params recomputes only dependents; cached results are hit on unchanged inputs. A downstream reference to a face group resolves after an upstream repair. Baked mesh body survives a `.amg` save/load bit-exact (with DOC-07 when available, else via `MeshBody` binary). |
| MESH-26 | Phase 1 op features: Remesh, Reduce, Smooth, Plane Cut, Shell (quick/robust), Separate, Erase & Fill | MESH-16, MESH-06, MESH-07, MESH-08, MESH-09, MESH-10 | P0 | Todo | Each op is a registered feature with `Expr` params: undoable, parametric (editing a param recomputes it and its dependents only), serialises round-trip, and reports health (warning on quick-shell SIs). Golden tests in `tests/golden/mesh/` (volume, area, bbox, Hausdorff) for each. |
| MESH-23 | `amg-ui::mesh` Phase 1 workspace: toolbar, hand-built dialogs on the framework API, job-backed cancellable previews (proxy > 2 M tris), Mesh Analysis panel with defect highlighting | MESH-26, UI-02, UI-03, UI-04, UI-10 | P0 | Todo | Every MESH-26 op plus Repair / Reverse Normal / Analysis is reachable from the toolbar and S-key search. Dialogs never block the UI thread (no frame > 16 ms in the tracing profile while a 1 M-tri op runs). Cancel works within 200 ms. A manual Phase 1 script (open STL → analyse → repair → remesh → reduce → plane cut → shell → export 3MF) passes on 3 corpus models. |
| MESH-27 | Combine feature (join/cut/intersect, keep tools) + its `amg-ui::mesh` dialog | MESH-05, MESH-16, MESH-23 | P0 | Todo | Combine of two corpus bodies is undoable and parametric. Non-manifold tool input triggers auto-repair with a warning. Cross-type inputs (solid/implicit) produce the architecture's "converted to mesh" warning. Golden volume test passes. |
| MESH-11 | Wall thickness analysis (GPU WGSL BVH traversal + CPU fallback), per-vertex field for RND-08 overlay | MESH-03, FND-09, RND-08 | P1 | Todo | A 2 mm synthetic plate reads 2 ± 0.02 mm in its interior. GPU and CPU agree within 1e-3 × bbox diagonal. 1 M-tri model < 1 s on GPU. Overlay with legend shown via `amg-ui::mesh` → RND-08. |
| MESH-12 | Face-group segmentation (region growing by normal/curvature, small-region merge) + `MeshFaceGroupsFeature` | MESH-03, MESH-16 | P1 | Todo | On 20 synthetic tessellated CAD parts (known face ids, with and without 0.02 mm noise), ≥ 95 % of faces get the ground-truth label after optimal matching. Deterministic. 1 M tris < 2 s. |
| MESH-24 | Primitive fitting: Efficient RANSAC + Gauss–Newton refine (plane, sphere, cylinder, cone, torus), stored on groups | MESH-12 | P1 | Todo | Noise-free samples recover parameters within 1e-6 relative. At σ = 0.05 mm noise, radius/axis errors are < 0.1 mm / 0.5°. Same seed gives same result. |
| MESH-13 | Mesh→B-rep faceted (coplanar groups → planar faces → sew → solid) + `MeshConvertFeature` | MESH-12, BREP-03, BREP-10 | P1 | Todo | A closed 10 k-tri mechanical mesh gives a valid closed solid (BRepCheck), with face count = number of planar regions, volume equal to the mesh volume within 1e-9 relative, in < 5 s. |
| MESH-25 | Mesh→B-rep prismatic (fitted analytic surfaces, trimmed, sewn) | MESH-13, MESH-24, BREP-03 | P1 | Todo | On 20 synthetic tessellated CAD parts (boxes, holes, bosses, fillets as cylinders/tori), output is a valid solid with analytic faces, volume within 0.5 %. A fillet and an extrude-cut on the result succeed. |
| MESH-14 | Mesh section sketch (section polylines → line/arc/spline fit → sketch) | MESH-09, SK-08 | P1 | Todo | Section of a tessellated cylinder gives a circle with radius error < 0.5 %. All fitted curves are within the 0.01 mm tolerance of the polyline. The sketch is usable by extrude. |
| MESH-15 | Direct edit: soft transform of face groups (radial/geodesic falloff) + `MeshSoftMoveFeature` + manipulator tool | MESH-12, MESH-23, UI-06 | P1 | Todo | Moving a group with smooth falloff keeps the mesh manifold and SI-free for displacements ≤ falloff radius / 2. The edit is undoable and parametric. Manipulator drag previews at ≥ 30 fps on a 1 M-tri mesh (proxy). |

Phase placement for the new WPs is proposed in the Refinement log; the roadmap is not edited
here.

## Testing
- **Unit tests** per module, with the analytic cases spelled out in each WP's Done-when.
- **Property tests (`proptest`):**
  - half-edge invariants under random split/collapse/flip;
  - `IndexedMesh` ↔ `HalfEdgeMesh` round-trip;
  - repair idempotence;
  - boolean volume identities;
  - plane-cut volume conservation;
  - `extract_isosurface` manifoldness on random fields.
- **Corpus tests (MESH-28):** for every manifest entry, record analyse → repair → metrics
  (watertight, SI-free, Hausdorff to input, triangle quality). Thresholds as in the WPs. The
  harness prints a summary table. Regressions fail CI when the pass rate drops.
- **Golden geometry** (`tests/golden/mesh/`, via `amg-testkit` from FND-06): volume, area,
  bbox, topology counts and Hausdorff for each op on fixed inputs.
- **Determinism:** each op is run with 1 and 16 rayon threads and twice in a row, and the
  output `content_hash` must be equal.
- **Cross-checks (dev-dependencies only):** `parry3d-f64` for ray / closest point, `obvhs`
  for BVH build-speed comparison. Optionally, offline comparisons of remesh and decimation
  quality against PMP / MeshLab outputs, stored as numbers, not binaries.
- **Benchmarks (`criterion`, FND-08):** every op in §Performance targets at 100 k / 1 M / 10 M
  triangles on generated meshes.

## Performance targets
Reference machine (vision doc), release build. Inputs come from the MESH-28 benchmark generator
(displaced icosphere; for booleans, two offset spheres). Anything that can exceed 16 ms runs as
a job (vision: the UI never blocks > 16 ms). Values are upper bounds. "—" means no target
(best effort, job with progress).

| Operation | 100 k tris | 1 M tris | 10 M tris |
|---|---|---|---|
| Weld soup → `IndexedMesh` | 10 ms | 80 ms | 1 s |
| `HalfEdgeMesh` build | 10 ms | 100 ms | 1.2 s |
| Analysis (no SI) | 10 ms | 100 ms | 1.2 s |
| BVH build | 15 ms | 150 ms | 2 s |
| Ray cast throughput (8 cores) | ≥ 10 M rays/s | ≥ 5 M rays/s | ≥ 2 M rays/s |
| Fast winding number, 1 M queries | 0.5 s | 1.5 s | 4 s |
| Self-intersection detection | 50 ms | 500 ms | 6 s |
| Repair, typical defects | 0.3 s | 3 s | 40 s |
| Isotropic remesh, 5 iterations (≈ same count) | 1 s | 10 s | 120 s |
| QEM decimate to 10 % | 0.2 s | 2 s | 25 s |
| Taubin smooth, 10 iterations | 30 ms | 300 ms | 3.5 s |
| Boolean (A ∪ B, each of the given size) | 150 ms | **1 s (vision budget)** | 15 s |
| Plane cut + cap | 30 ms | 300 ms | 4 s |
| Quick shell | 30 ms | 300 ms | 4 s |
| Robust SDF shell (100 mm part, 0.25 mm voxel) | 8 s | 20 s | — |
| Wall thickness (GPU) | 0.1 s | 1 s | 10 s |
| `render_buffers` (f32 upload prep) | 2 ms | 20 ms | 250 ms |

**Memory** for 10 M triangles:
- `IndexedMesh` ≤ 300 MB;
- `+ HalfEdgeMesh` ≤ 160 MB;
- `+ MeshSpatial` (BVH) ≤ 220 MB, plus ≤ 260 MB when the winding tree is built;
- peak during repair ≤ 2 GB.

## Open questions
1. **(owner, answered)** Corpus licensing: is the GitHub repo public or private? Default plan: commit only
   CC0 / PD / CC-BY / CC-BY-SA models to LFS, and keep NC/ND Thingi10K entries and the Stanford
   scans fetch-only (downloaded by `fetch.ps1`, never committed). OK, or may research-only and
   NC models be committed?
   **Answered 2026-10-20:** default plan: commit only CC0 / PD / CC-BY / CC-BY-SA models; NC/ND and Stanford scans stay fetch-only.
2. **(owner, answered)** Can you supply your own failing STLs/3MFs (scans, Thingiverse downloads, slicer
   failures) for `testdata/mesh/real/owner/`? Real failures beat synthetic ones.
   **Answered 2026-10-20:** no. MESH-28 uses only the public corpus and the synthetic defect
   generators. The `owner/` set is not required. If owner files turn up later, they go under
   `$AMG_TESTDATA_DIR/mesh/real/owner/` (S00 Open question 2).
3. **(owner, answered)** ADR-003 acceptance, including the amendments proposed in this refinement:
   - own `amg-manifold-sys` over the C API;
   - Manifold + oneTBB (Apache-2.0) via vcpkg;
   - `MANIFOLD_CROSS_SECTION=OFF`.

   MESH-05 / MESH-27 wait on it. Everything else in S07, including the robust shell, does not.
   **Answered 2026-10-20:** ADR-003 accepted with the proposed amendments (recommended). MESH-05 / MESH-27 are no longer gated.
4. **(owner, answered)** Phase 1 mesh dialogs: UI-05 (generic feature dialogs) depends on SOL-01 and
   lands in Phase 2. Accept hand-built Phase 1 mesh dialogs in MESH-23 (migrated later), or
   should lane D decouple UI-05 from SOL-01 so it can land in Phase 1?
   **Answered 2026-10-20:** hand-built Phase 1 mesh dialogs in MESH-23, migrated to UI-05 later (the plan the spec is built on).
5. Resolved: f64 vs f32 positions → f64 (see Design §Decisions 1 and the design note).
6. Resolved: winding number stays in MESH-03, so IMP-03's dependency (MESH-03) is unchanged.
   Self-intersection detection moved to the new MESH-19.
7. Resolved in MESH-05 (implementation-time check, not an owner decision): whether the upstream
   vcpkg `manifold` port enables `MANIFOLD_CBIND`, which decides overlay port vs upstream port;
   and whether Manifold + TBB output is bit-deterministic.

## Refinement log
**2026-10-19 — S07 refinement (lane C agent).**

*Researched:*
- Manifold upstream: C binding `MANIFOLD_CBIND`, TBB `MANIFOLD_PAR` with the warning against
  bundled TBB, vcpkg port, `LevelSet`.
- Rust bindings:
  - `manifold3d` 0.4.1 / `manifold-csg(-sys)`: MIT/Apache-2.0, C-API FFI, f64 `MeshGL64`
    default, TBB `parallel` default, git-clone + CMake in `build.rs` unless
    `MANIFOLD_CSG_LIB_DIR` is set;
  - `manifold-rs` 0.6 (WIP).
- BVH / geometry crates:
  - `obvhs` 0.4: MIT/Apache, PLOC + CWBVH, f32, rayon `parallel`;
  - `parry3d-f64` 0.31: Apache-2.0, now on `glamx`;
  - `baby_shark` 0.3.12: MIT, nalgebra 0.34, last release 2025-09;
  - `bvh`, `remesh`, `meshopt`, `lin-alg`, `robust`, `spade`.
- Papers, each cited in the design note: Barill 2018 FWN, Jacobson 2013 GWN, Liepa 2003,
  Botsch–Kobbelt 2004, Dunyach 2013, Garland–Heckbert 1997, Hoppe 1999, Taubin 1995,
  Zheng 2011, Attene 2010, Portaneri 2022, Schnabel 2007, Campagna 1998, Wald 2007, Ize 2013,
  Woop 2013, Guigue–Devillers 2003, Shewchuk 1997, Zhou & Jacobson 2016 (Thingi10K).

*Decided:*
- f64 positions with no exception. f32 only for conservative BVH bounds and render upload; both
  are non-model data and fit ADR-006 (clarification proposed).
- `u32` indices; directed-edges half-edge.
- Own BVH / processing code. Adopt `robust`, `spade`, `smallvec`, `rand` + `rand_chacha`.
- Own `amg-manifold-sys` built from vcpkg, with `manifold-csg-sys` as fallback.
- Sparse marching-tetrahedra isosurfacing in `amg-mesh` (MESH-17) so the robust shell needs no
  Phase 3 IMP-04.
- Robust shell and wrap composed in `amg-features::mesh`, with `MeshSdf` implementing
  `amg_mesh::iso::ScalarField`. No dependency cycle.
- Determinism and `JobCtx` cancellation rules. Corpus layout, licensing policy and generators.
- Per-op performance targets at 100 k / 1 M / 10 M, consistent with the 1 s boolean, 16 ms UI
  and 5 M-tri viewport budgets.

*Changed WPs:*
- Re-scoped:
  - MESH-01: adds weld and `MeshBody` encoding;
  - MESH-03: self-intersection moved out, FWN kept;
  - MESH-04: now orchestration only; holes and SI split out;
  - MESH-06: adaptive split out; adds dep MESH-03 for projection;
  - MESH-09: dep stays MESH-04;
  - MESH-10: adds deps MESH-17, IMP-03 already present; wrap moved out;
  - MESH-11: adds dep RND-08;
  - MESH-12: adds dep MESH-16 for its feature; RANSAC split out;
  - MESH-13: faceted only; adds dep BREP-02;
  - MESH-14: dep MESH-03 → MESH-09 (`section`);
  - MESH-15: adds deps MESH-23, UI-06;
  - MESH-16: now plumbing + Repair / Reverse Normal; adds deps DOC-05, MESH-04.
- MESH-02 adds dep MESH-28; MESH-05 adds dep MESH-04 (input auto-repair).
- New: MESH-17 isosurface, MESH-18 wrap, MESH-19 SI detection, MESH-20 hole fill, MESH-21 SI
  removal, MESH-22 adaptive remesh, MESH-23 Phase 1 UI, MESH-24 RANSAC, MESH-25 prismatic,
  MESH-26 Phase 1 op features, MESH-27 Combine feature, MESH-28 corpus.
- Every WP now has a measurable Done-when.
- Added Scope, Interfaces (concrete), Testing, Performance targets and Open questions.

*Roadmap phase-order notes (roadmap not edited):*
- Proposed placement:
  - Phase 1, lane C: MESH-17, MESH-18, MESH-19, MESH-20, MESH-21, MESH-22, MESH-23, MESH-26,
    MESH-27, MESH-28. All their deps are Phase 0/1: FND-06, DOC-02/05, UI-02/03/04/10, IMP-03
    and MESH-05's gate.
  - Phase 3, lane C: MESH-24, MESH-25 (deps MESH-12 / MESH-13 / BREP-03 are in Phase 1–3).
- MESH-28 could also run late in Phase 0 next to the MESH-01 spike.
- No existing WP moves earlier than its deps:
  - MESH-11 (Phase 3) now needs RND-08 (Phase 1);
  - MESH-13 (Phase 3) needs BREP-02 (Phase 1 background);
  - MESH-14 (Phase 3) needs MESH-09 (Phase 1);
  - MESH-15 (Phase 3) needs UI-06 (Phase 2).
- **Issue:** Phase 1 has no generic feature-dialog WP, because UI-05 depends on SOL-01
  (Phase 2). MESH-23 therefore hand-builds its dialogs (owner question 4).
- MESH-27 and MESH-05 sit in Phase 1 but cannot start until ADR-003 is accepted.

*Recommended dependencies* (for `THIRD_PARTY.md`; the coordinator updates it):

| Dependency | Licence | Purpose |
|---|---|---|
| `robust` | MIT / Apache-2.0 | exact orientation predicates |
| `spade` | MIT / Apache-2.0 | 2D CDT for caps and planar holes |
| `smallvec` | MIT / Apache-2.0 | one-ring buffers |
| `rand` + `rand_chacha` | MIT / Apache-2.0 | seeded RANSAC and corpus generator |
| oneTBB (vcpkg `tbb`) | Apache-2.0 | Manifold parallel backend |
| `toml` (dev) | MIT / Apache-2.0 | corpus manifest |
| `parry3d-f64` (dev, optional) | Apache-2.0 | test oracle |
| `obvhs` (dev, optional) | MIT / Apache-2.0 | BVH benchmark reference |
| `manifold-csg-sys` (fallback only) | MIT / Apache-2.0 | used if the own `amg-manifold-sys` route fails |

Already listed: Manifold (Apache-2.0), `faer`, `nalgebra`, `rayon`, `glam`, `thiserror`,
`serde`.

*Design note created:* `docs/notes/mesh-processing.md`.

**2026-10-21 — MESH-28 implemented (lane C agent, branch `wp/MESH-28-corpus`).**
- **Corpus:** `testdata/mesh/corpus.toml` has 176 entries. 165 are committed (Thingi10K, 70 MB of
  the 300 MB budget, largest file 1.35 MB, LFS through the existing `*.stl` patterns); 11 are
  fetch-only (8 Thingi10K NC models, Stanford bunny / dragon / armadillo) through `fetch.sh` /
  `fetch.ps1`, driven by `fetch.tsv` (URL + sha256 of the download and of the extracted file).
- **Coverage of the committed set:** self_intersections 116, holes 78, multiple_shells 63,
  degenerate 52, non_manifold_edges 45, unwelded 38, inconsistent_orientation 36, duplicate 25,
  non_manifold_vertices 25, hopeless 12 (`expect_repairable = false`), inverted 5, plus 20 clean
  controls. The Thingi10K dataset has few small closed models with negative volume, hence 5.
- **Licence evidence:** the dataset's `input_summary.csv` licence column per model; only CC0,
  public domain, CC-BY and CC-BY-SA were taken. SPDX versions (3.0) are assumed because the
  dataset records only the family (see `real/thingi10k/README.md`; the owner may want to confirm).
- **Defect ground truth** was derived with a numpy script (exact-weld edge statistics, signed
  volume, a 1e-4 x diagonal weld test for `unwelded`, Thingi10K self-intersection counts). The
  Rust test `every_corpus_model_matches_its_manifest_entry` re-derives holes, non-manifold edges
  and vertices, orientation, degenerate, duplicate, shells and inverted independently and
  matches all of them; `unwelded` and `self_intersections` stay provisional until MESH-02 /
  MESH-19.
- **Code:** `amg_mesh::corpus` behind the cargo feature `corpus` (off by default; this crate's
  tests enable it through a self dev-dependency): manifest types and validation, STL / PLY
  reader, `quick_stats`, `synth` (5 shapes, 12 defect classes, `ChaCha8Rng`, ground truth,
  reproducible from `(shape, defects, seed)`), `bench` (displaced geodesic spheres, 100 820 /
  1 003 520 / 9 996 980 triangles, bit-identical for 1 and N threads; `boolean_pair`).
- **Left for others:** MESH-02 must reproduce the manifest flags (including `unwelded`); the
  corpus tests need an LFS pull (CI `golden` job sets `AMG_REQUIRE_TESTDATA=1`).

**2026-10-23 - MESH-02 implemented (lane C agent, branch `wp/MESH-02-analysis`).**
- **Code:** `amg_mesh::analyze` / `AnalysisReport` in `src/analysis/` (own module). No sort of
  edge keys: a CSR vertex-to-corner table (atomic counting and fill) is visited per vertex in
  parallel; each edge `(v, w)` with `v < w` is classified at its smaller vertex (1 face boundary,
  2 faces manifold plus a same-direction check, 3+ non-manifold), a vertex is non-manifold when
  the link graph of its faces has more than one component, and duplicates are found at the
  smallest vertex of the triple. Shells come from a lock-free union-find over vertices (root =
  smallest member, so numbering is deterministic). Shell statistics use fixed 16 k-face chunks;
  results are bit-identical for 1, 3 and 8 threads (test).
- **Definitions (these match the manifest and `quick_stats`):** shells are vertex-connected
  components; faces with a repeated vertex index are degenerate and outside the topology (their
  `shell_of_face` is `INVALID`), but two of them with the same sorted triple count as duplicates,
  as in the manifest; `is_inverted` = watertight and total volume < 0; `volume` is `Some` only
  when watertight.
- **Bench (release, 16 logical cores, shared machine):** displaced-sphere benchmark meshes,
  `mesh_analysis/100k` 3.07 ms, `1m` 16.8 ms (budget 100 ms), `10m` 166 ms (budget 1.2 s).
- **Manifest corrections (S07 asked for them; 165 committed entries were measured, all agree
  now).** `non_manifold_vertices` added to 12 entries: 41928, 47979, 49333, 55558, 70368, 89802,
  111959, 112855, 180690, 503947, 1017013, 1681239. The corpus builder only measured vertex
  fans on edge-manifold meshes (the Rust harness skipped the others for the same reason); all
  12 have non-manifold edges, and both `quick_stats` (MESH-28) and `analyze` find vertices
  whose faces form several fans (1 to 64 per model), which is the generator's definition too.
  `unwelded` added to 42881, 55558, 55572, 124709, 130443, 503947: each has 2 to 197 boundary
  edges whose endpoints coincide with another boundary edge's within 1e-4 x diagonal (130443
  already within 1e-6 x diagonal) while the vertices differ. The builder's grid snap test splits
  such pairs across cell borders. `unwelded` removed from 656709: its 16 boundary edges are
  3.6 to 5.4 micrometres long, all shorter than the 1e-4 x diagonal (9.7 micrometres) merge
  distance, so welding only closes a pinhole and there is no seam (lengths and neighbours are
  printed by the diagnostic test `dump_manifest_disagreements`). 74452 keeps its flag: its three
  boundary edges (2 to 20 mm long) each face a two-face edge whose endpoints coincide to 2e-14 mm
  with distinct vertex ids. `unwelded` is defined as: a boundary edge longer than the merge
  distance has another boundary edge within the merge distance at both endpoints, or any other
  edge at numerically coincident (merge distance / 1000) endpoints; the looser distance is not
  applied to interior partners because 41928 and 180689 have boundary edges next to
  two-face edges at 97 to 98 % of the merge distance (slivers), which are not seams. Every other flag (holes, non-manifold edges, orientation,
  degenerate, duplicate, shells, inverted) matched without change, and all counts agree with
  `quick_stats` (boundary edges, loop lengths, non-manifold edges and vertices, conflicts,
  degenerate, duplicate, shells, volume, area). `self_intersections` and `hopeless` are not
  measured here (MESH-19 / repair verdict).
- **Left for others:** `AnalyzeOpts::self_intersections = true` returns
  `InvalidOption` until MESH-19 can be called from `analyze` (it needs a `MeshSpatial`);
  `MeshBody::analysis()` is not added (body.rs is shared with parallel WPs); a 10 M-triangle
  soup with tens of millions of boundary edges is not optimised in the unwelded check.

### 2026-10-23 - MESH-04 implemented (repair pipeline; lane C agent, branch `wp-MESH-04-repair`)
Code: `crates/amg-mesh/src/repair/` (`mod.rs` driver, opts and report; `clean.rs` weld / degenerate / duplicates / small shells; `nonmanifold.rs`; `orient.rs`; `edges.rs`), tests `tests/repair.rs`, bench `benches/mesh_repair.rs`, goldens `tests/golden/mesh/repair_{sphere,torus}_defects.toml`. `lib.rs` gained one `mod` and one `pub use`, `Cargo.toml` one `[[bench]]`; no new dependency. API as in Interfaces: `repair`, `RepairOpts` (+ `Default`), `SmallShellFilter`, `RepairReport`, `RepairStep(Kind)`, `UnresolvedIssue`, `orient_consistently`. MESH-21's corpus test no longer carries its own weld / orient stand-in: `pre_clean` calls `repair` with hole filling and SI removal switched off.
- **Steps (in order, each does nothing when the analysis shows nothing for it, so a clean mesh comes back bit-identical):**
  - *Weld:* merges vertices that lie on a boundary edge, round 0 only (see Deviations).
  - *Degenerate:* a face with a repeated index goes. A zero-area face with three distinct vertices is removed by collapsing its shortest edge that passes the link condition and flips no neighbour, so a closed mesh stays closed. If no edge qualifies it is dropped only when that opens no 3-edge hole; an interior face that cannot be collapsed stays and shows in `after.degenerate_faces` (dropping it, then hole filling, recreates the same face every run).
  - *Duplicates:* per vertex set, opposite-winding pairs cancel and at most one face of the remaining winding stays.
  - *Non-manifold:* "unzip": faces are paired across every edge, vertex corners are grouped into fans along the pairs, each fan gets its own vertex. Edges with more than two faces are sorted by angle around the edge, faces oriented outward, and a face is paired with its neighbour on the material side, so two solids touching along an edge become two sheets again and a zero-thickness fin becomes a 2-face shell (then removed as debris). Unpairable faces become boundary. Copies are moved 1e-3 of the distance to their fan's neighbours into the fan: copies on top of each other would count as intersecting in MESH-19 ("touching counts").
  - *Small shells:* `SmallShellFilter { max_faces, max_volume_fraction }` against the largest shell (closed: |signed volume|, open: AABB volume); the largest always stays.
  - *Orient:* BFS over manifold edges per component, then per component outward by signed volume. A closed component inside another closed component (winding number of 8 sample vertices, at most 32 closed components) is a cavity and is pointed inward, so a hollow part keeps its volume. Runs again after hole filling when it filled something.
  - *Holes:* `fill_holes` (MESH-20) directly on a half-edge copy.
  - *Self-intersections:* `remove_self_intersections_with` defaults (MESH-21); `RepairOpts::max_rounds` is the number of pipeline rounds, not MESH-21's rounds.
  - Rounds repeat until one changes nothing (a fixed point) or `max_rounds`; a later round that makes the total defect count worse is discarded.
- **Deviations / choices:**
  - The weld only merges boundary vertices and only in round 0. A seam is two boundary chains; interior coincident copies are made by the non-manifold step and merging them back would undo it (and make the second run non-idempotent). Consequence: coincident interior duplicates are not merged. Default tolerance stays 1e-5 mm (`Tolerance::mesh()`); the corpus' 1e-4 x diagonal "unwelded" seams are generated with jitter below it.
  - Non-manifold edges are never resolved by deleting faces (no geometry lost); the price is the 1e-3 edge nudge of vertex copies.
  - `UnresolvedIssue::HoleTooLarge` is reported for every boundary loop left in `after` (too large, fill refused or filling off).
  - `RepairReport::before` / `after` include the self-intersection pairs only if `remove_self_intersections` is set; `before` is measured on the raw, unwelded input, so seams count there.
- **Measured (Done-when; `tests/repair.rs::corpus_repairable_models`, all 153 fetched / committed `expect_repairable` Thingi10K models, default `RepairOpts`, debug-ish test profile, loaded machine):**
  - Watertight (closed + manifold + oriented, outward, `volume > 0`): **153 / 153 = 100 %** (bar 95 %).
  - Watertight and SI-free: **144 / 153 = 94.1 %** (bar 85 %). Of the 104 models flagged `self_intersections`, 95 end watertight and SI-free. The 9 that keep intersections (not examined one by one) are expected to be the overlapping-solid cases MESH-29 is for.
  - Idempotent: second `repair` returns the same content hash and every step has `changed == 0` on **153 / 153**.
  - No defect count grows: boundary loops, non-manifold edges / vertices, orientation conflicts, degenerate and duplicate faces never grow on any of the 153. The SI pair count ends above the raw input's on 3 models (thingi10k-49162, -68434, -82177), all with hundreds of holes or pinch points in the input: the closing patches cross the neighbouring solids and MESH-21 reduces but does not clear them; the test holds the SI count to models without holes or non-manifold elements. On synthetic meshes `no_single_step_makes_any_count_worse` applies the steps one after another and checks that the total never grows and that the count each step is for never grows, plus watertight at the end.
  - Time on the corpus: 9.9 s for 153 models (about 65 ms each, slowest thingi10k-111399 with 958 ms).
  - Bench (`benches/mesh_repair.rs`, release, displaced sphere with holes, an unwelded seam, flipped / duplicate / zero-area / needle faces, fins and debris, default options including SI removal): **100 k tris (100 588) 144 ms** [138, 158] (budget 0.3 s), **1 M tris (1 003 288) 1.08 s** [1.06, 1.09] (budget 3 s). Step times at 1 M: weld 3 ms, degenerate 59, duplicates 61, non-manifold 53, small shells 3, orient 83, holes 102, orient 83, SI 230 (the rest is the analysis passes).
- **MESH-21 corpus test with the real pipeline** (`pre_clean` = `repair` without holes and SI): 96 / 104 flagged models SI-free (59 of 67 repaired by MESH-21, 37 already clean after cleaning), against 90 / 104 with the old local stand-in; the 85 % bar still holds.
- **Synthetic checks** (`tests/repair.rs`): each MESH-28 defect class alone and all together on a sphere and a torus end watertight, outward and SI-free with the volume within 3 % of the clean body; clean input is bit-identical; cavity orientation (hollow body keeps outer minus inner volume); fins and bow ties; small-shell filter edge cases; unresolved holes are reported; cancel and option errors; identical hash for 1 / 4 / 13 threads; goldens.
- **Leftovers:** vertex normals are not re-oriented or recomputed after a flip or a split (recompute them); corner UVs and colours of split vertices are copies, patches from hole filling have none (MESH-20); the non-manifold step leaves faces it cannot pair as boundary (an odd number of sheets) and the hole filler decides what to do with them; nesting is only tested for up to 32 closed components; an uncollapsible interior degenerate face stays (reported in `after`); MESH-18 Wrap is not part of this WP (hopeless soups are excluded from the corpus targets); the SI step cannot repair overlapping solids (MESH-29).

### 2026-10-23 - MESH-05 implemented (booleans via Manifold; lane C agent, branch `wp-MESH-05-manifold`)
Code: new crate `crates/amg-manifold-sys` (`build.rs`, `src/lib.rs`), `crates/amg-mesh/src/boolean/` (`mod.rs` API + feature dispatch, `backend.rs` conversion and group mapping), tests `tests/boolean.rs`, bench `benches/mesh_boolean.rs`, goldens `tests/golden/mesh/boolean_spheres_{union,difference,intersection}.toml` and `boolean_box_minus_spheres.toml`; vcpkg overlay port `ports/manifold/`, `vcpkg.json` (`"dependencies": ["manifold"]`), `vcpkg-configuration.json` (`overlay-ports`); workspace dependency `amg-manifold-sys`; `amg-mesh/Cargo.toml` feature `manifold`, one `[[bench]]`, one `[[test]]` (both `required-features = ["manifold"]`); `lib.rs` one `mod` + one `pub use`. API as in Interfaces: `BoolOp`, `BoolOpts`, `BoolReport`, `boolean`, `boolean_n`.
- **Native build.** Manifold 3.5.4 + oneTBB 2023.1.0 + Clipper2 2.0.1 from vcpkg (baseline `2026.10.16`), static, release-only, triplet `x64-windows-amg`; built on this machine with MSVC 14.44 in 5.3 min (cold, binary cache warm afterwards). The upstream port already sets `MANIFOLD_CBIND=ON`; the overlay exists for `MANIFOLD_PAR=ON` (TBB) and `MANIFOLD_DOWNLOADS=OFF`. `amg-manifold-sys/build.rs` does no git/CMake: with feature `native` it links `manifoldc`, `manifold`, `Clipper2`, `tbb12` from `AMG_VCPKG_INSTALLED` (else `<repo>/vcpkg_installed`) `/<triplet>/lib` and fails with a how-to message if `manifoldc.h` is missing; it also checks that every `manifold_*` function declared in `lib.rs` appears in the installed header (version drift guard). Without `native` nothing is linked, so `cargo build --workspace` needs no C++ library.
- **Safe layer in the -sys crate.** `amg-mesh` keeps `unsafe_code = deny`; all `unsafe` is in `amg-manifold-sys` (`MeshGL64` / `Manifold` RAII owners, `MeshData` copy-out, `reserve_ids`), each block with a SAFETY comment. Signatures are in Interfaces.
- **Group transport.** Manifold's `faceID` cannot be set through the C API (and is overwritten by coplanar ids), so groups travel as a vertex property: an operand with one group is passed as plain xyz and recognised by its run; an operand with several groups gets a 4th property (dense group index), each vertex shared by faces of different groups is duplicated per extra group, and the duplicates are declared with `mergeFromVert` / `mergeToVert` (`manifold_meshgl64_w_options`), so topology is exact and no position-based merge runs. Each operand gets its own `runOriginalID` from `manifold_reserve_ids`; output runs map back to operands through it. Output corners are interpolated inside one source triangle, so the property is that triangle's group index (rounded). Result table = operand 0's table unchanged (incl. groups whose faces vanished); each later operand's group that occurs in the result is re-allocated in (operand, source id) order with `GroupOrigin::Boolean { operand, source }`, name and primitive kept. Property vertices are collapsed back through the output merge vectors. **Canonical order:** because the parallel import can hand Manifold's internal mesh ids to the operands in either order, and Manifold orders output runs by those ids, the result puts runs in operand order (triangle order inside a run kept) and numbers vertices by first use; without this the 10-run determinism test failed once parallel import was added.
- **Input checks.** An operand Manifold rejects as `NotManifold` is repaired with `repair(RepairOpts::default())` when `repair_inputs` (index in `BoolReport::repaired_inputs`), otherwise `analyze` names the problem: `NonManifold { edges, vertices }`, `NotClosed { boundary_loops }`, or `Boolean { status }`. Other Manifold statuses become `MeshError::Boolean`. `boolean_n` takes 1 to 256 operands (`InvalidOption` otherwise); `[a]` alone returns Manifold's normalised copy; 3+ use `manifold_batch_boolean` (`Difference` = first minus the union of the rest).
- **Performance work.** First version: 1.45 s release for the 1 M x 1 M union, of which 1.06 s was operand import. `AsOriginal` (used to get a traceable id) recomputes normals and coplanar faces a second time; replacing it with a reserved `runOriginalID` saved about 0.4 s, importing the operands in parallel (rayon; Manifold's import is largely serial per mesh) about 0.2 s more, and parallel import/export loops on the Rust side the rest (release: 1.45 s, then 0.99 s, then 0.97 s). Manifold's own coplanar-face pass in the import cannot be skipped through the API.
- **Measured (Done-when).**
  - Builds on Windows MSVC via vcpkg with the feature on (above). Linux: not tried (best effort; `build.rs` links `libtbb`, `stdc++`, `pthread` there).
  - Two 1 M-triangle offset spheres (`corpus::bench::boolean_pair(M1)`, 1 003 520 tris each, displaced, centres 7 mm apart), union end-to-end including both conversions: **974 ms** [964, 985] (criterion, release, `benches/mesh_boolean.rs -- _1M`, 16 logical cores, machine otherwise idle); 999 ms [983, 1018] while three other workers were building. Difference 884 ms [876, 893], intersection 771 ms [762, 781]; 100 k x 100 k: union 94 ms, difference 84 ms, intersection 75 ms. The margin under the budget is small (about 3 %): most of the time is inside Manifold (import with its coplanar-face pass about 450 ms for the two operands in parallel, boolean about 300 ms, mesh export about 90 ms). `tests/boolean.rs::timing_union_1m_spheres` (test profile) asserts < 1 s only with `AMG_STRICT_TIMING=1`. Output 1 366 154 tris, watertight.
  - Volume identity: `volume_identity` proptest, 200 random pairs (icospheres of 1 to 3 subdivisions and boxes, radius 2 to 12, offsets up to 15 mm, so disjoint, nested, touching and overlapping cases): vol(A∪B)+vol(A∩B) = vol(A)+vol(B) and vol(A−B)+vol(A∩B) = vol(A) within 1e-6 relative; every non-empty output passes `analyze().is_watertight()`.
  - Watertight: every output in every test is checked (closed, manifold, oriented, non-negative volume).
  - Group ids: `group_ids_survive` (two cubes with named side groups whose ids collide): A's table and ids unchanged, B's groups re-allocated above A's counter with `Boolean { operand: 1, source }` and names kept; faces land in the geometrically right group, and the uncovered L-shaped rest of A's covered face keeps A's id (area exact). `boolean_n_batch` sees operands 1 and 2.
  - Determinism: `determinism_10_runs`, 100 k x 100 k displaced spheres with extra groups on both operands, union / difference / intersection, 10 runs each: positions (bits), triangles and per-face groups identical; the test passed 6 times in a row (180 booleans). Manifold + TBB itself is deterministic for fixed operand ids; the run reordering above makes the result independent of the import order (documented in the module docs).
  - Without the feature: `cargo nextest run -p amg-mesh` (default features) compiles and `boolean` returns `BackendUnavailable` (unit test `without_feature_backend_is_unavailable`).
- **Deviations.** (1) `amg-mesh/manifold` is **off by default** (design decision 5 said on): a plain build and CI's default jobs must work without the native library; the app crate should enable it. (2) `MANIFOLD_CROSS_SECTION` stays **ON** (ADR-003's amendment said OFF): in Manifold 3.5.4 `MANIFOLD_CBIND` is a `cmake_dependent_option` on `MANIFOLD_CROSS_SECTION`, so the C API does not exist without it. That adds Clipper2 (BSL-1.0, permissive, static); Amalgam never calls it. Owner may want to note this on ADR-003. (3) Mesh attributes (vertex normals / colours, corner UVs, 3MF properties) are not carried through a boolean; the result has empty `MeshAttrs`.
- **Leftovers.** `docs/01-architecture.md` §Crates does not list `amg-manifold-sys` (the layering test now does; see S00 Interface changes). Attribute propagation (colours/UVs as extra Manifold properties) if MESH-27 needs it. Linux build untested. CI's first run with a non-empty `vcpkg.json` will build Manifold + TBB + Clipper2 cold (about 5 min) before the binary cache takes over.

## Interface changes
### 2026-10-20 — MESH-01 (`IndexedMesh`, `HalfEdgeMesh`, weld, `MeshBody`; lane C)
Deviations from §Interfaces:
- **`MeshBody` has no `spatial()` / `analysis()` yet.** `MeshSpatial` (MESH-03) and
  `AnalysisReport` (MESH-02) do not exist, so the `OnceLock` fields and both methods are added
  additively by MESH-03 / MESH-02 (they take `JobCtx` from `amg-jobs`, FND-04, which is a
  dependency of `amg-mesh` already). `MeshBody` derives `Clone, Debug, PartialEq`.
- **No `From<MeshError> for amg_core::Error`.** `amg_core::Error` has no variant that can carry
  a mesh error faithfully. Following the S00 convention instead, `MeshError` implements
  `amg_core::Diagnostic` (hints for `NonManifold` / `NotClosed` / `BackendUnavailable` /
  `Decode`; `Cancelled` is `Severity::Info`) and `From<amg_core::Cancelled>`. If a caller
  needs the conversion, S00 should add a boxed variant (e.g. `Error::Domain(Box<dyn
  Diagnostic>)`); not requested yet.
- `MeshError` and `TopoError` are `#[non_exhaustive]`. **New `TopoError::NonManifold`**:
  `add_face` that repeats a vertex or would give a vertex a second fan, and `split_edge` of
  the shared edge of a two-face shell (both new edges to the opposite vertex would coincide).
- `ShellSide` is defined next to `GroupOrigin` (which needs it) and re-exported at the crate
  root; MESH-10 uses the same type.

Additive API:
- `DEFAULT_GROUP` (`FaceGroupId(0)`); `FaceGroups::{new(face_count), info_mut, per_face,
  next_id, validate}`; `FaceGroupInfo::new(origin)`; `Primitive::kind`.
- `MeshAttrs::{validate, is_empty}`; `Rgba8::{WHITE, lerp}`; `TriProperty::NONE` ("no
  property": group and indices `INVALID`).
- `WeldOpts::EXACT`; `Default for WeldOpts` = `Tolerance::mesh().linear` (1e-5 mm).
  `Remap: PartialEq, Eq`; `RenderBuffers: Clone, Debug, Default, PartialEq`;
  `ConversionReport: PartialEq, Eq`.
- `HalfEdgeMesh` (derives `Clone, Debug, Default, PartialEq`): `vertex_count`, `face_count`
  (live), `vertex_slots`, `face_slots`, `halfedge_slots` (with tombstones),
  `is_vertex_alive`, `is_face_alive`, `vertices()`, `faces()`, `face_vertices`,
  `halfedge(f, k)`, `vertex_halfedge`, `attrs`, `groups`, `groups_mut`, `neighbors`,
  `find_halfedge`, `delete_vertex`, `check_invariants() -> Result<(), String>`.

Behaviour the interface left open:
- `IndexedMesh::new` puts every face in `DEFAULT_GROUP` (origin `GroupOrigin::User`).
- `HalfEdgeMesh::from_indexed`: vertex ids are kept and split copies are appended; faces with a
  repeated index are dropped (`dropped_degenerate`). Edges with more than two faces **or two
  faces of the same direction** (orientation conflict) are cut and counted in `cut_edges`; under
  `Fail` they are the `edges` of `MeshError::NonManifold`, and multi-fan vertices its
  `vertices`. A fin (3 faces on one edge) reports `cut_edges = 1`, `split_vertices = 4`; a
  bow-tie `split_vertices = 1`. A fix-up pass (only after cuts) re-pairs the two ends of a slit
  fan and unlinks duplicate directed edges inside one fan, so the result always passes
  `check_invariants` and re-converts under `Fail` without changes.
- `to_indexed` drops tombstones only (isolated live vertices are kept).
- `collapse_edge(h, p)` removes `origin(h)` and keeps `target(h)` (PMP convention); the
  link condition plus PMP's boundary rules are in `can_collapse`.
- `outgoing(v)` yields face half-edges counter-clockwise from the stored (boundary) one; on a
  boundary vertex the last neighbour is `origin(prev(last))`. `valence` counts neighbours.
  `is_boundary_vertex` is true for isolated vertices (PMP / OpenMesh).
- `delete_face` splits a boundary vertex that would otherwise become a bow-tie (the faces
  counter-clockwise of the deleted face move to an appended copy).
- New vertices without a parent get a zero normal and white; faces from `add_face` get zero
  UVs and `TriProperty::NONE`. Split vertices interpolate normals (renormalised) and colours;
  new corners interpolate UVs; per-corner 3MF property indices take the nearer endpoint's.
- `IndexedMesh::transform` with a negative determinant also reverses the winding (and corner
  attributes), so signed volume keeps its sign. `append` fills an attribute present on only
  one side with defaults on the other.
- `weld` / `from_soup`: transitive clustering; the smallest index of each cluster survives
  bit-for-bit; faces are kept (degenerates included); `Remap::faces` is the identity.
- `content_hash` is an own fixed-seed 64-bit hash, stable across runs and builds.
- `MeshBody` binary: `AMGM` v1 layout is documented in `amg-mesh/src/body.rs` (raw f64 bits,
  trailing 64-bit checksum; truncation, bad magic/version/flags and checksum mismatch are
  `MeshError::Decode`).
- `render_buffers(origin, smooth_normals)`: with `smooth_normals == false` the vertices stay
  shared (`positions` has one entry per vertex, `indices` is the face list in face order) and
  `normals` is **empty**; the renderer derives flat normals per primitive (S10 request, see
  below). With `true`, `normals` holds one unit vertex normal per position.

Review-round changes (2026-10-20), all inside `amg-mesh`:
- **Fallible size limits instead of panics.** `IndexedMesh::append` returns `MeshResult<()>` and
  `IndexedMesh::weld` returns `MeshResult<Remap>`; both fail with the new
  `MeshError::TooLarge` when counts reach the `u32` index range (weld: 2³¹ points, the limit of
  its packed sort keys) or face-group ids run out, leaving the mesh unchanged.
  `FaceGroups::try_alloc` / `remaining_ids` are new; `alloc` panics when the id counter is
  exhausted, and `validate` rejects `next_id == u32::MAX`. New `TopoError::TooLarge`:
  `split_edge` and `add_face` return it at the vertex/face limit. `add_vertex` and the vertex
  split inside `delete_face` / `delete_vertex` still panic there (documented).
- **Deserialisation validates.** `IndexedMesh` and `FaceGroups` deserialise through
  `try_from` and run `validate`, so malformed serde input is an error, not a mesh that panics
  later. `MeshAttrs` has no counts of its own to check against; it is validated as part of
  `IndexedMesh`.
- **Edge pairing is sort based** (`(min, max)` key, O(n log n)); a vertex of valence 100 000
  no longer costs O(k²). Results are unchanged and independent of the thread count.
- **Weld grid**: cell edge 8 × tolerance (was 32×) so clustered input compares a bounded number
  of pairs per cell; ghost margin is `1/8 + 1/64` cells (tolerance reach plus rounding slack).
  Exact-equivalence with brute force is covered by tests.
- `HalfEdgeMesh::from_indexed` face map when degenerate faces are dropped: face `k` of the
  result is the `k`-th face of the input with three distinct indices (attributes and groups
  follow).
- **For DOC / MESH-02.** `amg-doc`'s stub mesh type derives `Eq`, which `IndexedMesh` cannot
  (it holds `f64`); the real mesh reference in the document must compare by `content_hash` or
  derive `PartialEq` only.
- **Proposal for `amg-testkit` (not done here).** The torus and quad-grid generators in
  `amg-mesh/tests/common` belong in `amg_testkit::shapes` so other crates can reuse them.

Measured (release, 16-thread desktop, 1 M-tri closed torus, criterion means): half-edge build
50 ms (budget 100 ms; 35 ms before the sort-based edge pairing, which trades a little speed for
O(n log n) on high-valence vertices); weld of the 1 M-tri soup 36 ms exact, 57 ms at 1e-5 mm
(budget 80 ms, cell 8 × tol); `to_indexed` 6 ms; `MeshBody` write 12 ms / read 16 ms. The machine was shared with other
builds, so individual runs varied up to 3×.

### 2026-10-20: requests from the wave-2 refinements (S02, S05, S10, S11, S12, S16, S17, S20)

Additive requests recorded by the integration pass; the owning lane confirms or adjusts them when it implements the affected WP.

- **From S02 (B-rep kernel).**
  - MESH-13 now depends on BREP-03 (polygon faces) + BREP-10 (sew) instead of BREP-02; applied in
    the WP table. MESH-25 stays on BREP-03.
  - The requested `make_planar_face(outer, holes)` is `make_planar_polygon` (BREP-03);
    `make_face_on_surface(&AnalyticSurface, loops, tol)` is
    `make_face(&geom::Surface, &FaceBounds::Polylines(..), tol)`; `amg-features::mesh` converts
    `amg_mesh::Primitive` → `geom::Surface`; `sew` is in BREP-10.
  - Additive: a `GroupOrigin::BrepFace { face: u32 }` variant for tessellated solids converted to
    meshes.
- **From S05 (inspection).**
  - MESH-03: read-only `Bvh` node access `root()`, `node(i) -> BvhNode { bounds: Aabb3, kind:
    Inner { left, right } | Leaf { first, count } }`, `prim(slot)` (INS-01 pair distance, INS-04).
    Fallback: a private BVH in `inspect`.
  - MESH-01/03: non-blocking `MeshBody::try_spatial()` / `try_analysis()` returning `Option`, for
    use on the UI thread.
  - MESH-09: document that closed `section()` polylines are oriented with material on the left
    seen from the plane's +normal side, for oriented closed input.
  - Naming: S07 uses `amg_core::Ray`; S00 defines `Ray3` (lane C/E to reconcile).
- **From S10 (rendering), consumed by S11 UI-20.**
  - `RenderBuffers.normals` is empty and vertices stay shared when `smooth_normals == false`.
  - New `analysis::feature_edges(&IndexedMesh, crease_angle) -> Vec<[u32; 2]>` (≤ 50 ms at
    1 M triangles).
  - New `vertex_curvature(&IndexedMesh, CurvatureKind) -> Vec<f64>` (INS-05).
- **From S11 (UI shell).** Mesh features register `EditAction::StartTool`. MESH-23 may use the
  Phase 1 UI-15 dialog widgets; no dependency added, since MESH-23 builds its own Phase 1 dialogs
  (QUESTIONS #22).
- **From S12 (import/export).** `MeshAttrs::property_groups: Option<Arc<PropertyGroups>>` with
  `PropertyGroup::{Materials(Vec<(String, Rgba8)>), Colors(Vec<Rgba8>)}`, giving the existing
  `tri_properties` its table; `IndexedMesh::append` remaps group ids. Until it lands, per-face
  slots survive only on unmodified imported bodies.

### 2026-10-21 - MESH-28 (corpus; lane C)
Additive only: cargo feature `amg-mesh/corpus` (optional deps `toml`, `sha2` 0.10, `rand_chacha`
0.9, declared in `crates/amg-mesh/Cargo.toml` rather than the workspace table) exposing
`amg_mesh::corpus::{Manifest, Entry, Defect, FileStatus, read_mesh, quick_stats, synth, bench}`.
Other crates (INS-02, IMP-03) use it with `amg-mesh = { workspace = true, features = ["corpus"] }`
under `[dev-dependencies]`. `lib.rs` gained one `#[cfg(feature = "corpus")] pub mod corpus;`.

**2026-10-21 — MESH-08 implementation (lane C agent, wp/MESH-08-smooth).**
- `amg-mesh::smooth` (own module): `smooth`, `SmoothOpts`, `SmoothMethod`, `LaplacianWeights`;
  `SmoothOpts::new(method, iterations)` and `SmoothMethod::TAUBIN_DEFAULT` are additive helpers.
  The `Interfaces` signatures are unchanged.
- No sparse solve and no new dependency: all methods are explicit Jacobi iterations over a CSR
  vertex→face / vertex→vertex adjacency built inside the module (MESH-01 internals untouched).
  Results are independent of thread count.
- Taubin uses uniform weights. Cotangent weights are clamped at 0 and recomputed each step; a
  degenerate vertex falls back to uniform.
- Bilateral: face-normal filter with vertex-ring neighbours (spatial sigma = mean centroid
  distance, range sigma = `sigma_normal` on |ni - nj|), then area-weighted vertex update
  (Sun 2007 / Zheng 2011). `iterations` = outer (normal filter + vertex update) rounds.
- Locked vertices: boundary / non-manifold-edge vertices (`fix_boundary`), vertices touching more
  than one face group (`preserve_group_borders`), vertices outside the `selection`, unreferenced
  vertices.
- Finding: Taubin 0.5/-0.53 slightly inflates the low frequencies (factor 1 + 0.03k per pair), so
  volume change is resolution dependent; on an icosphere with 20 480 tris, 1 % noise, 50
  iterations: +0.32 % volume, radial RMS 0.0993 → 0.0276 mm (3.6×). On 5 120 tris it was +1.2 %.
- Tests in `smooth/tests.rs` (Taubin sphere, bilateral cube, locking, options, cancel,
  determinism, goldens `tests/golden/mesh/smooth_*`); bench `benches/smooth.rs`.

### 2026-10-21 — MESH-17 (`amg_mesh::iso`, sparse marching tetrahedra; lane C)
Implemented per §Interfaces. Notes and small deviations:
- **Sign convention:** inside is `value < iso` (negative-inside SDF). `value >= iso`, NaN and
  everything outside `bounds` are outside, so triangle normals point towards increasing values
  and the signed volume of the inside is positive. `iso`/`cell_size`/`bounds`/`block_cells` are
  validated (`MeshError::InvalidOption`).
- **Lattice:** anchored at `bounds.min` with spacing `cell_size`; one extra forced-outside
  layer on every side, which is what closes the surface at `bounds`. Cap vertices between an
  in-bounds inside sample and a forced-outside sample sit at the clamped edge midpoint (so on
  the bounds face when `bounds` is a multiple of `cell_size`).
- **Vertex identity:** a tet-edge key = lower lattice point + direction bitmask (1..=7: three
  axis edges, three face diagonals, the body diagonal). Same key in every block, so blocks are
  merged by key in block order. Output is bit-identical across thread counts (tested with 1, 3
  and 8 threads).
- **Sparsity:** flat grid of `block_cells³` blocks (not a literal octree; the per-block
  `may_cross` test gives the same pruning at the sizes in the budget). A block is skipped when
  `may_cross` is false on its in-bounds part. At `bounds` the block is probed once: outside
  means empty, inside means the probe value stands in for every sample (only the cap against
  the forced-outside layer is generated). So `may_cross == false` must mean "all values on one
  side of `iso` in this box".
- **`project_steps`** (clamped to 3) does secant steps along the same tet edge, so topology and
  the manifold guarantee are unaffected. Only edges with two real finite endpoints are refined.
- Not done: no octree refinement above the block level; no GPU path. Slivers are expected, so
  callers decimate (MESH-04).
- Tests: `crates/amg-mesh/tests/iso.rs` (1 000-field CSG fuzz incl. exact zeros on lattice
  points and surfaces crossing `bounds`; sphere/torus volume convergence; Hausdorff <= cell;
  determinism; sparsity; cancellation; goldens in `tests/golden/mesh/`), bench
  `crates/amg-mesh/benches/iso.rs`.

### 2026-10-21 — MESH-03 (Bvh, MeshSpatial; lane C)
Landed in `crates/amg-mesh/src/spatial/` (`bvh.rs`, `tri.rs`, `winding.rs`, `mod.rs`), tests in `tests/spatial.rs`, bench `benches/mesh_spatial.rs`.
- **Interface deviations / additions (all additive):**
  - `Ray3` from `amg_core` is used (not `Ray`), per the S05 naming note.
  - `Bvh::build` returns `InvalidOption` for non-finite boxes or bad options; an empty set is allowed (0 nodes).
  - `Bvh` gained `prim_count()`, `root()`, `node(i) -> BvhNode { bounds, kind: Inner | Leaf }`, `prim(slot)` and `depth()` (S05 request). `query_aabb` visits all prims of leaves whose bounds overlap (conservative); `MeshSpatial::faces_in_aabb` filters on the exact triangle box.
  - `Bvh::overlapping_pairs` is **not** in MESH-03; it belongs to MESH-19 (self-traversal).
  - `ray_cast_all` and `faces_in_aabb` append to `out`; `ray_cast_all` is sorted by `(t, face)`. `closest_point` and `ray_cast` break exact ties by lowest face id.
  - `TriFeature::Edge(i)` is the edge from corner `i` to `(i+1)%3`.
  - `signed_distance` is negative inside (winding number > 0.5).
  - New `MeshSpatial::prepare_winding(&JobCtx)` (cancellable tree build; otherwise built lazily on first winding query) and `winding_numbers(&[Point3], beta)` (parallel batch).
  - `MeshBody::spatial()` / `try_spatial()` are not added (lib.rs/body.rs are shared with parallel WPs); a follow-up WP-sized tweak.
- **Winding tree:** order-2 dipole expansions with exact triangle moments and exact translation to parents, stored only for nodes covering more than 8 triangles; radius is a conservative bound (`|c_child - c| + r_child`).
- **Dev-only references (added 2026-10-22):** `obvhs =0.4.0` (MIT/Apache-2.0, feature `parallel`) and `parry3d-f64 =0.31.1` (Apache-2.0) are `amg-mesh` dev-dependencies. Both licences are on the `deny.toml` allow-list (ADR-012), `cargo deny check` passes (advisories, bans, licences, sources), and the `THIRD_PARTY.md` rows are updated. Cargo.lock gains about 30 packages (glam 0.33, glamx, simba, rstar, spade, criterion 0.5 and others), all dev/bench-only.
  - **2x bench vs obvhs PLOC** (1 M triangles, same f32 boxes, excluding box construction, loaded machine): ours 117-140 ms; obvhs PLOC `Minimum` 89 ms, `Low` 102 ms, `Medium` 105 ms. Ratio 1.3x-1.6x, inside the 2x bar.
  - **parry3d-f64 cross-check** (`tests/spatial_parry.rs`): 10 k random closest-point and ray queries each on an 800-triangle soup and a torus. Closest distances agree to 1e-9 relative, ray `t` to 1e-8 relative, hit/miss disagreements at most 0.1 % (edge-grazing).
- **Numbers (loaded machine, ~10 parallel agents, so pessimistic):** 1 M-tri BVH build 117 ms (budget < 150 ms); `MeshSpatial::build` 104 ms; winding tree build 132 ms; 1 M FWN queries (rayon) 0.90 s (budget < 1.5 s); 1 M closest-point queries 2.1 s; 1 M axis-aligned ray casts 40 ms.

**2026-10-22 — MESH-07 landed (lane C agent, branch `wp/MESH-07-decimate`).**
- `amg-mesh::decimate` (`DecimateTarget`, `DecimateOpts` + `Default`, `DecimateReport`, `decimate`) as in Interfaces; no half-edge dependency: it works on a private triangle list with vertex-face adjacency for speed.
- Choices: area-weighted plane quadrics; penalty quadrics (planes through protected edges, perpendicular to the adjacent face, weight `boundary_weight * len^2`) for boundary, group borders, feature edges and UV seams; unprotected boundary edges get a soft weight 1. Rules: unprotected edge between two protected vertices is never collapsed, a protected vertex absorbs a free one without moving, link condition, no face-normal flip (cos < 0.05), quality < `min_triangle_quality` and worse than before is rejected, non-manifold and degenerate-face vertices are locked, at least 4 faces remain. Queue key (f32 cost, min id, max id), lazy invalidation by vertex-version sum; only the sort and the initial costs are parallel, so output is identical across thread counts (tested with 1/4/13).
- `max_error` / `MaxError` compare the area-weighted RMS plane distance (mm); `DecimateReport::max_quadric_error` is the largest accepted value in mm. Vertex normals are always dropped (reported); colours and corner UVs are kept when `preserve_attributes`, else dropped and reported. Groups and 3MF properties are kept per face. `FaceCount` may undershoot by one.
- Tests: `crates/amg-mesh/tests/decimate.rs` (closed/manifold/genus, displaced-sphere Hausdorff <= 0.2 % diag at 10 %, cube features exact, group border and boundary kept, determinism, cancel, attributes, goldens in `tests/golden/mesh/`). Bench `benches/mesh_decimate.rs`.
- **Speed rework (same day).** Lazy-deletion edge heap replaced by an indexed binary heap over vertices (`decimate/queue.rs`; key = the vertex's cheapest edge, strict order `(f32 cost, lo, hi, vertex)`) plus a per-vertex costed neighbour list, so a collapse re-plans only the edges of the surviving vertex (about 6 plans instead of about 18 per collapse) and ring vertices re-pick their best from the cached list. Initial costs/best candidates and per-vertex quadric sums are parallel but pure (identical across thread counts). Results are unchanged: goldens did not need re-blessing.
- **Numbers (1.31 M -> 100 k displaced sphere, machine loaded by ~10 agents, so pessimistic):** 3.4 s (lazy heap) -> **2.19 s**. Profile of the lazy-heap version: setup about 0.2 s (sort 35 ms, quadrics 90 ms, edge scan 35 ms, initial costs 30 ms); loop 3.0 s = pop+plan+legal 0.8 s, apply 0.6 s, re-planning/rescans 2.1 s (about 18 M `plan` calls), heap work was not the bottleneck. A Morton vertex reorder was tried and was 2x slower, so it was dropped. Remaining ideas: reuse the plan position at pop, cheaper `apply` (avoid per-vertex `Vec` retain), CSR adjacency. The < 2 s budget is within about 10 % on a loaded machine; re-measure idle.

### 2026-10-23 — MESH-19 (self-intersection detection; lane C, branch `wp/MESH-19-selfx`)
Landed in `crates/amg-mesh/src/selfint/` (`mod.rs`, `exact.rs`) and `src/spatial/pairs.rs`; tests `tests/selfint.rs`, bench `benches/mesh_selfint.rs`. New dependency `robust =1.2.0` (MIT / Apache-2.0, `cargo deny check` clean, `THIRD_PARTY.md` updated).
- **API (additive):** `self_intersections`, `SelfIntersectOpts { max_pairs }` as in Interfaces; extra `faces_intersect(&IndexedMesh, FaceId, FaceId) -> bool` (single exact pair test, for MESH-21). `Bvh::overlapping_pairs(prim_bounds, visit)` (sequential) and `Bvh::par_overlapping_pairs(prim_bounds, max_items, ctx, f)` (parallel, results in leaf order, thread-count independent) complete the MESH-03 leftover. Each leaf walks the tree once and meets only later leaves in depth-first order, so each pair appears once.
- **Semantics:** faces share vertices by index (weld first; an unwelded seam is reported). Closed triangles. No shared vertex: any common point counts (touching and T-junctions included). One shared vertex: they meet anywhere else (checked via the opposite edge of each triangle against the other). Shared edge: only if coplanar and overlapping with area (fold-over). Duplicate faces (three shared vertices) count. Exactly collinear (zero-area) faces are skipped (MESH-02 reports them).
- **Exactness:** every decision is the sign of `robust::orient3d` / `orient2d`; no tolerances. Non-coplanar and coplanar cases reduce to six exact closed segment-triangle tests (plane sides, then the three edge side-products; coplanar segments via exact 2D tests on the projection that drops the dominant axis).
- **`max_pairs`:** returns a deterministic prefix (first leaf batches in BVH order, truncated, then sorted), not the lowest-id pairs.
- **Verification:** 10 k-triangle random soup, a dense 1.5 k soup and lattice-pool soups (heavy sharing / coplanarity / touching) equal the O(n^2) exact brute force; lattice and integer soups also equal an independent i128 oracle (SAT for unshared pairs, rational segment-triangle for shared); identical for 1/4/16 threads; cancellation; intersecting spheres found, clean sphere / torus / separated spheres empty.
- **Corpus flags:** all committed models run; the result agrees with the manifest's `self_intersections` flag on every entry (no flag corrected), and all clean controls report zero pairs.
- **Numbers (16-core machine shared with other agents, release bench):** 1 M-tri displaced sphere, no hits: check 83 ms (79-87), BVH build plus check 202 ms (budget < 500 ms). 200 k-tri two-sphere case with 1 644 pairs: check 18 ms.

### 2026-10-22 — MESH-07 review (owner side)
- Idle-machine criterion run of `mesh_decimate/1_3m_to_100k` (1.31 M → 100 k triangles):
  2.12 s median [2.09, 2.16]. The input is 31 % larger than the Done-when's 1 M, and both setup
  and the number of collapses scale about linearly with it, so 1 M → 100 k is about 1.6 s, inside
  the 2 s budget, so MESH-07 is Done. A bench case at exactly 1 M input would remove the estimate.

### 2026-10-23 - MESH-02 (analysis; lane C)
Additive: `amg_mesh::{analyze, AnalyzeOpts, AnalysisReport, ShellInfo}` (module `analysis`) and
one `[[bench]] mesh_analysis`. Deviations from §Interfaces:
- `AnalyzeOpts` gains `degenerate_relative: f64` (default 1e-12; the threshold is
  `max(degenerate_area, degenerate_relative * bbox_diagonal^2)`, which is what the corpus
  manifest uses) and `unwelded_tolerance: f64` (default 1e-4 x diagonal, 0 disables).
- `AnalysisReport` gains `boundary_edges: usize`, `unclosed_boundary_loops: usize` (loops that
  cannot close, only on non-manifold or inconsistent input; such a chain lists edges + 1
  vertices) and `unwelded_boundary_edges: usize`.
- `analyze` rejects `self_intersections: true` with `MeshError::InvalidOption` until MESH-19
  is wired in. `MeshBody::analysis()` / `try_analysis()` are not added yet (shared body.rs).
- `testdata/mesh/corpus.toml` defect flags corrected as logged above (lane C owns the corpus).

### 2026-10-23 — MESH-02 / MESH-19 wiring (merge review)
- `analyze` with `AnalyzeOpts::self_intersections = true` now fills
  `AnalysisReport::self_intersecting_pairs` by building a `MeshSpatial` over a copy of the mesh
  and calling `self_intersections` (MESH-19). It no longer returns `InvalidOption`. Callers that
  already hold a `MeshSpatial` should call `self_intersections` directly to avoid the copy and
  rebuild.

**2026-10-23 — MESH-06 implemented (lane C agent, branch `wp/MESH-06-remesh`).**
- `amg-mesh::remesh` (own module `src/remesh/`: `mod.rs` driver and options, `features.rs`, `ops.rs`, `relax.rs`): `remesh`, `RemeshOpts` (+ `Default`), `Sizing`, `EdgeLength` as in Interfaces. `Sizing::Adaptive` returns `InvalidOption` until MESH-22. `lib.rs` gained one `mod` and one `pub use`; `Cargo.toml` one `[[bench]]` entry. No new dependency.
- Built on the MESH-01 `HalfEdgeMesh` (split / collapse / flip, ids stable until the final GC) and the MESH-03 `MeshSpatial::closest_point` for projection. Non-manifold input returns `NonManifold` (the conversion is run with `Fail`); topology (genus, shells, boundary loops) is preserved because collapse uses the link condition.
- Pass order per iteration: split (> 4/3 L, longest first, cascading through new edges), collapse (< 4/5 L, shortest first, rejects edges that would exceed 4/3 L or fold a triangle), flip (valence 6 / 4 on boundaries; rejects folds and flips that worsen the smaller min angle by more than 20 %), then one parallel pure relaxation + projection pass (area-weighted centroid, tangential, lambda 0.7, projection with `max_dist = 2 L`, moves that rotate an incident triangle past about 84 degrees are dropped).
- Constraints: feature edges (dihedral > `feature_angle`), group borders and boundaries are constrained edges. They are never flipped; they can be split and collapsed, and their vertices slide along the curve (midpoint of the two curve neighbours) and are snapped back to the nearest input feature segment (`Bvh` over segments). A vertex with a constrained-edge count other than 2, or with a turn above 45 degrees, is a corner and is fixed. `preserve_boundary = false` makes the boundary a "soft" curve: it relaxes without snapping (and shrinks).
- Selection: faces outside the selection are untouched; every vertex of such a face is fixed and no edge next to one is split, collapsed or flipped. A split is also refused when a neighbouring triangle has another edge that is too long and frozen (otherwise the cascade bisects it into ever thinner slivers; found by a hang in the selection test). A per-pass split cap is a safety net.
- Determinism: edits are sequential with total-order tie-breaks; the parallel stages are pure per-vertex (tested with 1 / 4 / 13 threads, bit-identical).
- Attributes: vertex normals are dropped; colours, UVs and groups follow the half-edge edit policy (split children inherit the group). UVs and colours are not re-evaluated when vertices slide or are projected.
- **Measured (Done-when):** icosphere (fine to coarse and coarse to fine), displaced sphere, torus, anisotropic torus, two-shell: min angle >= 30 degrees for 100 % (>= 99.9 % rounded) of triangles, 99.1-100 % of edges within [0.8 L, 1.33 L], Hausdorff 0.014-0.12 L, topology and shell count unchanged. Subdivided cube with L = 1.7: all 12 edges covered by mesh edges within 1e-3 mm, 8 corners kept, volume 1000 within 1e-6 relative; 100 % / 99.4 %. Two-group grid: group border and outer boundary exact to 1e-9. Corpus (20 committed watertight Thingi10K models, 500..60 000 faces): topology unchanged for all; at L = diag/120 18 of 20 meet all three bars; at diag/60 15 of 20. The failures are inherent: thingi10k-94016 (features thinner than L, Hausdorff 0.51 L at diag/120), thingi10k-1514498 (sharp wedges of constrained feature edges force corner angles < 30 degrees), and at the coarser diag/60 three models whose thin parts are below L. `tests/remesh.rs::corpus_watertight_models` asserts topology for all and >= 85 % quality pass.
- **Bench (`benches/mesh_remesh.rs`, release, loaded shared machine):** 5 iterations, `RelativeToMean(1.0)`: displaced geodesic sphere 1 003 520 -> 990 664 tris in 1.03 s; 1 M-tri torus (1000 x 500, real resampling) -> 714 490 tris in 2.92 s. Budget < 10 s: met with margin.
- Goldens: `tests/golden/mesh/remesh_{sphere_r10_l1.5,cube_l1.7,torus_l2.5}.toml`.
- Leftovers: UV / colour re-evaluation; adaptive sizing (MESH-22); tangential relaxation of constrained-curve vertices is uniform in the neighbour midpoint (no arc-length reparametrisation); a degenerate (zero-area) input face is dropped by the conversion and `selection` ids are mapped past it.

### 2026-10-23 — MESH-06 review
- Accepted as Done. The quality bars hold on every synthetic shape. On the corpus, 18 of 20
  models meet all three bars at L = diag/120. The two misses are properties of the input, not of
  the remesher: `thingi10k-94016` has features thinner than L (Hausdorff 0.51 L), and
  `thingi10k-1514498` has feature wedges sharper than 30°, which a feature-preserving remesh
  must keep. Topology is unchanged on all 20. The corpus test asserts topology on every model
  and the quality bars on at least 85 %.
- The bench's true 1 M → ~1 M case (displaced sphere, 1.00 M → 0.99 M) runs in 1.03 s against the
  10 s budget, on the loaded machine.

**2026-10-23 - MESH-22 implemented (lane C agent, branch `wp/MESH-22-adaptive`).**
- `Sizing::Adaptive { tolerance, min_length, max_length }` is now accepted by `remesh` (new `src/remesh/adaptive.rs`; `mod.rs`, `ops.rs`, `relax.rs` changed; no change to `lib.rs` or `Cargo.toml`, no new dependency). Invalid values (non-finite or <= 0 tolerance / min, `max < min`, non-finite max) return `InvalidOption`. `tests/remesh.rs::errors_and_cancel` changed from "valid adaptive options are rejected" to "tolerance 0 is rejected" because the option is no longer unimplemented; no other MESH-06 test or golden changed.
- Field: per input edge `kappa = theta / d` (`theta` = angle between the two face normals, `d` = centroid offset across the edge with its along-edge component removed, so slivers are measured correctly); edges across a feature crease (`theta` > `feature_angle`) are skipped. Vertex `kappa` = max over its edges, then max over its one-ring. `L = 0.75 * sqrt(6 eps / kappa - 3 eps^2)` (Dunyach 2013; the factor 0.75 = 3/4 because the remesher lets edges grow to 4/3 of the target, so the Dunyach length is the longest edge); `eps >= 2/kappa` gives `max_length`. Clamped to `[min_length, max_length]`, then graded with a deterministic Dijkstra sweep (`L` grows at most 0.5 per mm of surface distance).
- Remesher: the scalar target became a per-vertex size array `State::vs`. An edge's thresholds (split > 4/3, collapse < 4/5) use the smaller size of its two ends. A split vertex gets the mean of its ends, a collapse keeps the survivor's size, and relaxation re-reads the size from the input field at the moved (projected) position. Uniform sizing is bit-identical to before (same arithmetic); all MESH-06 tests and goldens pass unchanged.
- Adaptive builds the closest-point index even with `project_to_input = false` (needed to look the field up).
- Measured (eps = 0.05, `tests/remesh_adaptive.rs`): sphere deviation 0.0425, torus (R 4, r 3.5, saddle pinch) 0.0247 with 3 498 tris vs 19 540 uniform (18 %), localised-bump sphere 0.0421 with 2 172 tris vs 6 712 uniform (32 %). "Uniform at the same deviation" = bisection on the uniform length for the largest one whose deviation is <= the adaptive result. A constant-curvature shape (sphere, or a round torus's tube direction) cannot beat uniform by construction, so the 60 % bar is asserted on the torus and the displaced sphere only; the plain sphere and a round torus check the deviation.
- Leftovers: the isotropic field uses the larger principal curvature, so it is conservative on cylinders and tubes; anisotropic sizing would be a separate WP. The field is computed from the input mesh's own discretisation, so an input coarser than the features it should resolve underestimates `kappa`.

**2026-10-23 - MESH-20 implemented (lane C agent, branch `wp/MESH-20-holefill`).**
- `amg-mesh::holefill` (`src/holefill/`: `mod.rs` driver and commit, `triangulate.rs`, `refine.rs`, `fair.rs`): `fill_holes(&mut HalfEdgeMesh, ..)`, `fill_holes_indexed` (additive; uses `analyze` for the no-hole early exit), `HoleFillOpts` (+ `Default`: 2000 / 250 / refine / sqrt 2 / bi-harmonic), `Fairing`, `HoleFillReport`. Interface change (additive): `HoleFillReport::failed` for a loop the half-edge structure refused (rolled back; never seen in tests).
- Per loop: planar loop (within 1e-5 of its extent) -> CDT with `spade` (no Steiner points, falls back if a chord already exists in the mesh); else <= `max_dp_edges` -> Liepa DP over (worst dihedral cosine incl. the surrounding faces, then area), chords that already exist as mesh edges forbidden; else ear clipping (sharpest ear first) in the best-fit plane; last resort a centroid fan, so every loop fills. Refinement: Liepa's centroid split with sigma per vertex (mean incident edge length of the loop vertices) and alpha = `density_factor`, Delaunay-sphere flips of interior edges only (never creating an existing edge), 40 sweeps max.
- Fairing: Kobbelt umbrella energy with 1/edge-length weights, solved through the normal equations with `faer` sparse Cholesky (assembled directly in CSC; CG fallback). Harmonic: rows of the new vertices. Bi-harmonic: also the loop rows, using the full ring of the surrounding mesh, which gives tangent continuity. Patches over 60 000 new vertices are downgraded to harmonic, over 600 000 are not faired. **Planar (CDT) loops are not faired**: bi-harmonic would dome a flat plane-cut cap to match the wall tangents.
- Commit order: triangles are added by peeling the patch disc from the loop inwards (each step leaves a disc). A breadth-first order deadlocked on `add_face`'s "no second fan at a vertex" rule at about 180 edges. Failure rolls back.
- Determinism: sequential, no hash-order decisions (rayon only builds independent fairing rows); same hash with 1 and 8 threads.
- **Measured (Done-when):** tube holes of 3, 4, ..., 250, 251, 400, 500, 1000, 2000 edges, planar and non-planar, are all filled to a closed, manifold, oriented surface (Euler 2). Patch/surround mean edge length ratio 0.95-1.12 for 24-400 edge holes. Sphere r = 20: bi-harmonic fair fill deviates 0.06 % r (hole radius 3) and 0.27 % r (radius 5) against the 2 % bound; harmonic 0.94 % r. 14 corpus models with holes (all manifold ones, <= 150 k faces) end closed. Bench numbers in the handoff.
- Dependencies: `spade =2.15.1` (MIT / Apache-2.0, already in the lock file), `faer =0.23.2` (MIT, `std` + `sparse-linalg` only). THIRD_PARTY.md updated.
- Leftovers: a hole whose loop touches itself (bow-tie) is split by the half-edge conversion, not filled as one; no UV / colour interpolation on the patch (default attributes); fairing is single-threaded in the factorisation.

### 2026-10-23 - MESH-21 (self-intersection removal; lane C)
Additive only, all in `amg-mesh`:
- `remove_self_intersections(m, max_rounds, ctx) -> (IndexedMesh, remaining_pairs)` as in
  Interfaces, plus `remove_self_intersections_with(m, &RemoveSelfIntersectionsOpts, ctx) ->
  (IndexedMesh, RemoveSelfIntersectionsReport)`; both re-exported from the crate root.
- `holefill::fill_holes_where` (`pub(crate)`): `fill_holes` restricted to a subset of the loops
  (MESH-21 fills only the holes it cut). `fill_holes` behaves as before.
- Note for MESH-04: `RepairOpts::remove_self_intersections` can call `remove_self_intersections`
  directly; its `max_rounds` argument is the number of delete/refill rounds, not `RepairOpts::max_rounds`.

### 2026-10-23 - MESH-21 implemented (self-intersection removal; lane C agent, branch `wp/MESH-21-selfint-repair`)
- Code: `crates/amg-mesh/src/selfint/repair.rs`; tests `crates/amg-mesh/tests/selfint_repair.rs`; goldens `tests/golden/mesh/selfint_repair_{sphere_r10,torus}_folds.toml`. No new dependency.
- **Algorithm.** Per round: detect pairs (MESH-19), take their faces, grow by `k` *vertex* rings, delete; then delete more faces until no vertex of the cut is a pinch point (remaining faces in more than one fan) and no lone face is left whose three edges are all cut (it would be refilled as its own mirror image). Convert to half-edge, fill **only the loops that touch the cut** (`fill_holes_where`; pre-existing holes stay open), convert back, exact-weld if the conversion split pre-existing non-manifold places. The candidate is re-analysed and accepted only if it has fewer pairs, no more open loops / non-manifold edges / non-manifold vertices / orientation conflicts, and is watertight whenever the current mesh is. Otherwise the round is retried with `k + 1` (cap 6); after an accepted round `k` resets to 1 (leftovers are local). A rejected plain-vs-refined fill is retried once with plain triangulation (no refinement, no fairing). The best accepted mesh is returned with `pairs_after`, so a watertight input is never returned open, and an input without pairs is returned unchanged (bit-identical). Candidate detection stops counting at the current pair count (a candidate only has to beat it), which removed a 73 s case (tens of thousands of pairs created by a bad patch).
- **Defaults:** 12 rounds, rings 1..6, `Fairing::None` with refinement (fairing made no difference to the success rate on the corpus, 60-66 vs 64 of 81, and bulges patches), fill loops up to 2000 edges, a round may delete at most 90 % of the area, no distance budget. Optional `max_deviation` (fraction of bbox diagonal) / `max_deviation_edges` (mean edges, the larger applies) bound the sum over accepted rounds of the largest distance of a new face to the previous surface.
- **Measured (committed + fetched corpus, models flagged `self_intersections` and `expect_repairable`, <= 60 k faces, after the stand-in cleaning weld + degenerate/duplicate removal + orientation):** 111 flagged, 104 with <= 60 k faces/available; 23 are already SI-free after cleaning (the flag came from unwelded seams / duplicates), of the other 81 **67 become SI-free (83 %)**, so **90 / 104 = 86.5 % of the flagged models are SI-free** (Done-when >= 85 %). 2 more are reduced but not cleared, 12 are unchanged (the 14 not cleared are heavy overlaps of solids, 5 k pairs on a 3.9 k-face model, or very long sliver triangles). Whole corpus test 6 s; slowest model 336 ms. No watertight input became non-watertight (asserted per model).
- **Deviation from the Done-when: Hausdorff <= 1 % of the bbox diagonal is NOT met on the corpus.** Output -> input distance of the repaired models: median 2.7 % of the diagonal (0.54 mean edge lengths), p90 6.9 %, max 22 %; only 15 of the 67 are within 1 %. Reason: the corpus (Thingi10K) is coarse (mean edge 1-10 % of the diagonal) and its SI are mostly overlapping solids, so removing them removes whole surface sheets and the refill can only be accurate to a fraction of an edge. With a hard 1 % budget (`max_deviation: Some(0.01)`, `max_deviation_edges: 0`) only 8-11 of 81 models are repaired; with the budget "max(1 % diagonal, 0.5 mean edge)" 20 of 81. So the default has no budget and the budget is opt-in. On fine, locally defective meshes the patches are tight: synthetic folded sphere / torus (fold vertices moved by 3-4 edges) refill to 0.02 / 0.03 mm (0.1 % of the diagonal) away from the unfolded surface. MESH-04's "85 % SI-free" and this WP's "1 % Hausdorff" cannot both be met on this corpus with a local method; the owner should decide whether to relax the Hausdorff bar (e.g. relative to local edge length, as `max_deviation_edges` does) or to add a global approach (winding-number based removal of inner sheets / union by boolean) as a new WP.
- **Remaining pairs** are reported (`pairs_after`, and the second return value of `remove_self_intersections`).
- Leftovers: no UV / colour interpolation on the patches (inherited from MESH-20); groups of new faces are `Fill` as in MESH-20; unreferenced vertices of the input are kept (only vertices freed by deletion stay as isolated vertices, `compact` them); the input should be welded and oriented first (a non-manifold input is handled but not improved); no bench because the vision doc has no budget for this step (the corpus test is the timing check).

### 2026-10-23 - MESH-21 merge review
- Merged to `main` but left in `Review`: Done-when 2 (Hausdorff ≤ 1 % of the bbox diagonal) is not met on
  the corpus (median 2.7 %, p90 6.9 %, max 22 % over 67 repaired models). Owner decision in
  QUESTIONS #51 (relax to an edge-relative bar and/or add a global inner-sheet removal WP).

### 2026-10-23 - MESH-21 bar decision (owner, QUESTIONS #51)
- MESH-21's Hausdorff bar becomes edge-relative: the median output→input distance over repaired corpus
  models must be ≤ 1 mean input edge length (measured 0.54). MESH-21 is Done.
- New WP **MESH-29** (P1) adds a global inner-sheet removal for overlapping solids and keeps the
  original 1 % of bbox diagonal bar for them.

### 2026-10-23 - MESH-29 implemented (global self-intersection removal; lane C agent, branch `wp-MESH-29-global-si`)
- Code: `crates/amg-mesh/src/selfint/global.rs`; tests `crates/amg-mesh/tests/selfint_global.rs`; goldens `tests/golden/mesh/selfint_global_{two_spheres,two_cubes}.toml`. No new dependency (`spade`, `robust` and the MESH-03 winding tree were already there). Outside the new module: `lib.rs` (+4 exported names), `selfint/mod.rs` (re-exports) and `selfint/exact.rs` (`o2`/`o3` made `pub(crate)`). Signatures are in Interfaces; MESH-04 should call `remove_self_intersections_auto` (local first, global when the local result leaves pairs or exceeds `max_deviation`, the better one wins).
- **Prior art.** Jacobson, Kavan, Sorkine-Hornung 2013 (generalized winding number is a robust inside/outside label for any mesh, about 0 or 1 away from a closed surface, fractional near gaps); Barill et al. 2018 (the fast hierarchical evaluation MESH-03 already has); Zhou, Grinspun, Zorin, Jacobson 2016, *Mesh Arrangements for Solid Geometry* (cut at exact intersections, label the cells by winding number; libigl/CGAL with rational arithmetic); Cork (Bernstein and Fussell 2009). A full arrangement needs exact constructions (rational or expansion arithmetic, a new dependency under ADR-012); this WP keeps its two ideas, cut at crossings decided by exact predicates and label by winding number, with f64 constructions and a bounded perturbation instead.
- **Algorithm** (module docs have the detail). (0) Move every vertex by a deterministic pseudo-random offset of at most 1e-9 of the bbox diagonal (`perturbation`): exact contacts (a vertex on a face, coplanar faces, T-junctions), which are what most corpus "intersections" are, become clear gaps or clear crossings, so every pair is transversal. (1) For each pair from MESH-19, the crossings of the edges of one face with the other (exact `orient3d`, point in f64) give the intersection segment; a crossing is cached per `(edge, face)` and its vertex is shared by all faces around the edge (no cracks); pairs sharing one vertex cut in a segment from it; where three cut faces meet the three segments cross in one shared vertex (three-plane intersection). (2) Each cut face is re-triangulated with a `spade` CDT with the segments as constraints (a face that cannot take them keeps only its edge subdivision). (3) Patches = faces connected through edges that are not cut curves; the winding number of the input is sampled 0.01 sqrt(area) off both sides of each face, and a patch is an inner sheet when by area it is inside on both sides or the winding number does not jump by one across it (coincident or inverted layers). Only patches that touch a cut curve can be dropped, and a drop that would leave a single face on an edge is undone. (4) Candidates are tried in order: drop all inner patches, drop only those above 1e-6 of the area, drop none (sheets touch along the curve; open surfaces); the first with fewer pairs, no new boundary loop, and watertight if the input was, is returned; otherwise the input is returned unchanged. Up to 100 leftover pairs go through MESH-21 with a small budget (4 rounds, at most 5 % of the area, loops up to 300 edges).
- **Measured (Done-when).** Models flagged `self_intersections` and `expect_repairable`, at most 60 k faces, still intersecting after the cleaning (weld, degenerate and duplicate removal, per-shell orientation outward): 81. MESH-21 with its defaults repairs 15 of them to SI-free within 1 % of the diagonal; the other **66** are the WP's set (unrepaired, or Hausdorff > 1 %). On those 66, **57 (86.4 %) become SI-free with the one-sided Hausdorff distance output to input at most 1e-4 % of the diagonal** (the 1e-9 perturbation; the bar is 1 %; 0.0000 % at the printed precision for all 57), against the 80 % bar. Which candidate won: perturbation alone 4, drop all 36, drop large 16, cut only 2 (one of them leaves 89 pairs), none 8. All 8 watertight inputs in the set stay watertight (asserted per model). Slowest model 0.26 s for the global stage (the failing 49162 spends 37 s in the MESH-21 clean-up of its 89 leftover pairs). On the whole flagged set the combined entry point `remove_self_intersections_auto` gives 79 of 81 SI-free (72 within 1 % of the diagonal) against 67 of 81 for MESH-21 alone, and is never worse than MESH-21 per model (asserted). Corpus tests skip git-LFS pointers unless `AMG_REQUIRE_TESTDATA=1`.
- **Not repaired (9 of the 66).** 49162, 52310 (mass overlaps, 1.5 k and 5.7 k pairs on 2.4 k and 3.9 k faces; triangulation fallbacks remain), 49333, 50489, 425321 (open surfaces: every candidate would add boundary loops), and 259180, 458128, 1181082, 425320 (watertight or nearly; the cut curve ends at mesh vertices, a fold, so the two sides of the sheet are not separated, no candidate keeps the mesh closed). They stay at the MESH-21 result (1.3 to 6.1 % of the diagonal) under `remove_self_intersections_auto`.
- **Deviations from the WP text.** (1) Output vertices are not exactly on the input surface: up to 1e-9 of the diagonal per axis (the general-position move); `perturbation: 0.0` turns it off and leaves exact contacts to MESH-21. (2) For open surfaces the result can be intersection-free but non-manifold (sheets touch along the cut curve, `allow_seams`); a watertight input never becomes open and no loop is added. (3) Attributes and face groups are not carried over (the face set changes completely).
- **Leftovers.** Fold-ended cut curves (a sheet that crosses itself at a mesh vertex) are not separated; segments that cross in a face without a shared third face fall back to the edge subdivision; no UV/colour interpolation on new vertices; the MESH-21 clean-up is slow on large leftovers (37 s on 89 pairs), which is why it is capped at 100 pairs; no bench because the vision doc has no budget for this step. A true arrangement (exact arithmetic) would remove the perturbation and the fallbacks if the owner wants them.

### 2026-10-26 - MESH-16 implemented (mesh bodies in the timeline; lane C agent, branch `wp-MESH-16-mesh-bodies`)
Code: `crates/amg-features/src/mesh/` (`mod.rs`, `base.rs` OutputMode + input/emit, `refs.rs`, `repair.rs`, `reverse.rs`, `error.rs`, `tests.rs`); `amg-features` gained `serde`/`thiserror` deps and `pub mod mesh;`. Features: `mesh.repair` (`MeshRepairFeature { body, opts: RepairParams, output: OutputMode }`) and `mesh.reverse_normal` (`MeshReverseNormalFeature { body, groups: Option<Vec<MeshEntityRef>> }`), both v1, registered with `register_feature!` (call `mesh::link()`).
- **Mesh body = `BodyGeom::Mesh(Arc<IndexedMesh>)`.** The document's name for the S01 "BodyKind::Mesh(MeshBody)" already exists; the stub `IndexedMesh` in `amg-doc::stub` is now `pub use amg_mesh::IndexedMesh` (see S01 Interface changes). `MeshBody::from_arc` (amg-mesh, additive) wraps the shared mesh for encoding without a copy.
- **Repair.** `RepairParams` mirrors `RepairOpts` with serde types; `weld_tolerance` is an `Expr` (length, default `0.00001 mm`) so it joins the cache key via `exprs()`. A mesh that needs no repair (same `content_hash`) comes back as the *same* `Arc`, so no geometry-change event fires. `UnresolvedIssue`s become `Health::Warning` diagnostics. `OutputMode::Replace` emits `Modified` on the input body, `NewBody` emits `Created` in slot 0 and leaves the input.
- **References.** `MeshEntityRef { group, fingerprint }` and `resolve_group(&IndexedMesh, &MeshEntityRef) -> ResolvedGroup { group, quality }`: the id wins if it still has faces (`Exact`); otherwise the unique best group by fingerprint (area within 25 %, centroid within 15 % of the reference bbox diagonal, mean normal dot >= 0.9, same primitive kind; best score <= 0.5 x second best) gives `FingerprintOnly` and the feature warns; no match is `MeshFeatureError::UnresolvedGroup` (feature `Error`). Repair keeps group ids (hole patches get a new `Fill` group), so a downstream reference resolves `Exact` after an upstream repair (tested).
- **Reverse normal.** `reverse_normals(&IndexedMesh, Option<&[FaceId]>)` swaps corners 1/2 of the faces, and the matching corner UVs and `tri_properties` indices; vertex normals are negated for the whole body and dropped for a partial flip. Group table, face order and vertices are untouched; flipping twice restores the `content_hash`.
- **Persistence.** `StubBodyCodec` now writes meshes with `MeshBody::write_binary` (and reads `AMGM` via `read_binary`; legacy `stub:mesh` tags still decode to an empty mesh), so baked mesh bodies and the geometry cache (`StubOutputCodec` reuses it) round-trip bit-exact. `GeomCache` size estimates now count mesh arrays.
- **Evidence (`cargo nextest run -p amg-features`, 13 tests, all pass):** undo/redo restores the exact `content_hash` (defective sphere: source hash h0, repaired h1, undo -> h0, redo -> h1); re-edit of the repair recomputes `[repair, reverse]` only and takes both sources and the other chain from the cache, editing it back computes nothing (5 of 5 cached); a downstream group reference resolves `Exact` after repair; fingerprint fallback and unresolvable cases; baked repaired mesh saved in Direct mode and loaded back is `==`, same hash, same `AMGM` bytes, and the re-saved archive is byte-identical; a History document saved with `cache_export` reopens with every feature served from the persisted cache.
- **Deviations / leftovers.** `MeshEntityRef` is not wrapped in `amg_doc::EntityRef` (its `TopoRef` needs a DOC-06 `TopoName`), so features list the whole body as their `Dependency` and resolve groups themselves. Dependency on the body (not the group) means any change to the body recomputes the feature, as intended. Other mesh features (MESH-26/27/12) can reuse `base::{input_mesh, emit}` and `resolve_group`; `base` helpers are crate-private for now. The spec's `BodyRef` is `BodyId`.

### 2026-10-23 - MESH-09 implemented (plane cut, section, shells, erase & fill, reverse normals; lane C agent, branch `wp-MESH-09-plane-cut`)
- Code: `crates/amg-mesh/src/cut/` (`mod.rs` classification, pieces, halves, `plane_cut`, `section`; `side.rs` exact plane side; `cap.rs` section loops and CDT cap; `misc.rs` `separate_shells`, `erase_and_fill`, `reverse_normals`); tests `tests/cut.rs` (+ `tests/cut_common/mod.rs`, the lattice box); bench `benches/mesh_cut.rs`; goldens `tests/golden/mesh/cut_{sphere_r10,box6_diagonal}_{pos,neg}.toml`, `cut_torus_z0_pos.toml`. `lib.rs` gained one `mod` and one `pub use`, `Cargo.toml` one `[[bench]]`; no new dependency (`spade` and `rayon` were there; the expansion arithmetic of the exact side test is hand-written in `side.rs`). Signatures are those of Interfaces (`KeepSide`, `PlaneCutOpts`, `PlaneCutResult`, `Polyline3`, `plane_cut`, `section`, `separate_shells`, `erase_and_fill`, `reverse_normals`). No change in code another lane owns.
- **Plane cut.** (1) Every vertex gets an exact side from the sign of `n . (p - o)` for the plane's own f64 coefficients: a float filter (12 u of the term magnitudes), then a grow-expansion sum of the exact products (two-sum / FMA two-product) when the filter cannot decide. `snap_tolerance > 0` instead counts vertices within that distance as on the plane. Topology comes from these signs only, so rounding of new vertices cannot change it. (2) A triangle with vertices strictly on both sides is clipped (Sutherland-Hodgman on the triangle; a quad is split along its shorter diagonal); each crossed edge gets one new vertex, shared by both neighbours and both halves (sorted edge keys, binary search, so numbering is deterministic). A vertex on the plane is its own cut vertex: no duplicate, no sliver. (3) Triangles that only touch the plane go whole to their side; a triangle lying in the plane goes to the negative half when its normal is `+n` (the solid is below it) and to the positive half otherwise, so a plane through a face of a solid gives one half and `None` for the other. (4) The in-plane boundary edges of each half (edges used once, both ends on the plane) are capped: `cap::triangulate` builds a constrained Delaunay triangulation of the loops (`spade`, no Steiner points) and flood-fills from the left face of each reversed boundary edge without crossing a constraint, so holes (annulus, hollow bodies), islands and loops that touch at a vertex need no special case. Cap faces are one new `Cap` group, the same id in both halves (allocated from the id counter of the input). A half whose boundary does not close (open input), is non-manifold in the plane, or whose CDT is inconsistent stays uncapped and `open_loops` counts the loops.
- **Numerical coincidence.** A cut point within 1e-12 of the bounding-box diagonal of an end of its edge (a vertex 1e-16 off the plane, e.g. a torus ring at `sin(pi)`) is that end: the end vertex stands in for it and counts as on the plane. Without this two loop vertices land on the same 2D point, which no CDT can tell apart (it happened on the torus and on 3 corpus models). Topology stays closed (the decision is per edge, shared by both halves).
- **Attributes.** Vertex normals (renormalised) and colours are interpolated at cut points; corner UVs follow the barycentric position inside the parent face; 3MF property indices of new corners are those of the nearest parent corner; cap faces have zero UVs and `TriProperty::NONE`. Face order of a half: untouched faces in input order, then the pieces of cut faces, then the cap. Unreferenced vertices are dropped. Face groups: the input table is kept whole in both halves (groups without faces stay in the table).
- **`section`.** The positive-side in-plane boundary edges, chained into polylines. For a closed oriented input the loops are closed and run with material on the left seen from the plane's `+normal` side (the S05 request for MESH-14 / INS-02); loops touching at a vertex stay apart (tightest turn); chains of an open mesh come back with `closed == false`, listed first. No `JobCtx` (as in Interfaces).
- **`separate_shells`.** Union-find over vertex indices of faces without a repeated index, the definition of MESH-02 (shells numbered by smallest vertex); faces with a repeated vertex index belong to no shell and are dropped, as in the analysis. Ordered by descending area, ties by smallest vertex. Attributes and the full group table are kept per shell.
- **`erase_and_fill`.** Deletes the faces, drops vertices left without a face, converts to a half-edge mesh (`SplitVertices`, as `fill_holes_indexed`) and calls MESH-20 `fill_holes_where` for the loops that contain an edge that was not a boundary edge before: holes the input had stay open. The patch is one `Fill` group. A new loop that cannot be filled (`max_boundary_edges`, refused patch) is `MeshError::NotClosed { boundary_loops }`, not a silent open result. Face ids out of range are `InvalidOption`.
- **`reverse_normals`.** Swaps corners 1 and 2 of the faces (all for `None`), with corner UVs and property indices; vertex normals are negated on vertices all of whose faces flipped (recompute after a partial flip). Ids out of range are ignored.
- **Measured (Done-when).**
  - 1 M-triangle closed meshes, `benches/mesh_cut.rs` (release, both halves with caps, `keep: Both`, loaded machine): icosphere 1 310 720 tris, oblique plane **17.5 ms** [17.3, 17.7]; lattice box 1 002 252 tris, generic plane (x = 100.5) **31.5 ms** [31.1, 32.0]; plane through lattice vertices and edges (x = 100) **18.4 ms** [18.1, 18.8]; diagonal plane through vertices, edges and cell diagonals (x + y = 200) **17.0 ms** [16.8, 17.3]. Budget 300 ms.
  - `tests/cut.rs::million_triangles` (same box, three planes, test profile): halves watertight (closed, manifold, oriented) and volumes add up within 1e-9 relative; 18 to 31 ms per cut with `AMG_STRICT_TIMING=1` (the time is asserted only then).
  - Planes through vertices and edges on the lattice box (n = 6): `x=2`, `y=3`, `z=1`, `-x=-4`, `x+y=5`, `x+y=6`, `x-y=1`, `x+y+z=7`, corner cuts `x+y+z=1, 2, 3n-1`, generic `x=2.5`: halves closed and manifold, volumes within 1e-9, caps of both halves equal; the axis-aligned section area is exact (36 = n^2 to 1e-9) and the diagonal one is 36 sqrt 2. Coplanar faces and touching planes (edge, corner) give one half and `None`.
  - Nested loops: torus `z = 0` (annulus cap, area within 5 % of the exact annulus), hollow sphere (annulus), multi-shell and corner-touching cases; identical content hash for 1 / 4 / 13 threads.
  - Corpus (`corpus_models_cut_cleanly`: repaired, watertight, SI-free, single-shell, at most 20 000 faces): 83 models, two cuts each (oblique plane through the bbox centre and through a vertex): **166 / 166 fully capped**, all halves watertight, volumes within 1e-9. Skips un-fetched LFS pointers unless `AMG_REQUIRE_TESTDATA=1` (then at least 20 models must be usable).
  - `separate_shells` count equals `analyze().shells.len()` on a three-sphere mesh, on cut torus halves and on cut multi-shell meshes; erase & fill is watertight on a sphere patch, on one face and on a vertex fan; reverse normals negates `signed_volume` to 1e-9.
- **Deviations.** (1) `snap_tolerance` defaults to exact (0); the 1e-12 coincidence rule above is on top of it. (2) A face in the plane is assigned by its normal (3 above) instead of being cut, so `section` of a plane that coincides with a face shows an outline only where the solid continues. (3) `erase_and_fill` fails with `NotClosed` instead of returning an open mesh when a new loop is too big.
- **Leftovers.** A cap cannot be made for input that crosses itself in the plane (crossing constraints: the half is left open and counted). Coincident input vertices that are not cut points still make the CDT give up for that half (counted in `open_loops`). Property indices on cut faces are nearest-corner, not interpolated. `erase_and_fill` always splits non-manifold vertices of its input. MESH-26 (Plane Cut feature) and MESH-14 (section sketch) can use `plane_cut` / `section` as they are.
