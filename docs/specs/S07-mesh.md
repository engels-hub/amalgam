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
   vcpkg route fails on MSVC. The cargo feature `amg-mesh/manifold` is on by default; without
   it, `boolean()` returns `MeshError::BackendUnavailable`. **MESH-05 was gated by ADR-003 (accepted 2026-10-20).**
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
One parallel pass over sorted edge keys gives:
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
pub struct HoleFillReport { pub filled: usize, pub skipped_too_large: usize, pub new_faces: usize, pub new_group: Option<FaceGroupId> }

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
// ───────────── amg_manifold_sys (MESH-05) — raw FFI, unsafe confined here ─────────────
// Hand-written declarations for the subset of manifoldc (Manifold C API) we use; version pinned
// in vcpkg.json. Exposed only to amg-mesh::boolean.
#[repr(C)] pub struct ManifoldManifold { _p: [u8; 0] }
#[repr(C)] pub struct ManifoldMeshGL64 { _p: [u8; 0] }
extern "C" {
    pub fn manifold_alloc_manifold() -> *mut ManifoldManifold;
    pub fn manifold_alloc_meshgl64() -> *mut ManifoldMeshGL64;
    pub fn manifold_meshgl64(mem: *mut ManifoldMeshGL64, vert_props: *const f64, n_verts: usize,
                             n_props: usize, tri_verts: *const u64, n_tris: usize) -> *mut ManifoldMeshGL64;
    pub fn manifold_of_meshgl64(mem: *mut ManifoldManifold, mesh: *mut ManifoldMeshGL64) -> *mut ManifoldManifold;
    pub fn manifold_boolean(mem: *mut ManifoldManifold, a: *mut ManifoldManifold, b: *mut ManifoldManifold, op: i32) -> *mut ManifoldManifold;
    pub fn manifold_status(m: *mut ManifoldManifold) -> i32;
    pub fn manifold_get_meshgl64(mem: *mut ManifoldMeshGL64, m: *mut ManifoldManifold) -> *mut ManifoldMeshGL64;
    // … accessors for vert_properties / tri_verts / run_original_id / face_id, batch boolean,
    //   destructors. Exact list fixed by the MESH-05 spike against the pinned header.
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
| MESH-28 | Broken-mesh corpus: `testdata/mesh/corpus.toml` manifest, LFS-committed redistributable set, `fetch.ps1/.sh` for fetch-only sets, seeded synthetic defect + benchmark generators, corpus test harness | FND-06, MESH-01 | P0 | Todo | Manifest has ≥ 150 real models covering all 11 defect categories, each with SPDX licence, source URL and sha256. LFS total ≤ 300 MB. Generator reproduces each defect category from `(shape, defects, seed)`, plus 100 k / 1 M / 10 M benchmark meshes, bit-identically. The harness iterates the manifest under `cargo nextest` and skips fetch-only entries when they are absent. |
| MESH-02 | Analysis: shells, boundary loops, non-manifold edges/vertices, orientation, degenerate/duplicate, area/volume, `AnalysisReport` | MESH-01, MESH-28 | P0 | Todo | Report matches the manifest's defect flags on 100 % of corpus entries and the synthetic generator's ground truth. 1 M-tri analysis < 100 ms; 10 M < 1.2 s. |
| MESH-03 | `Bvh` (binned SAH, parallel), `MeshSpatial`: ray cast, closest point, box query, fast winding number (Barill 2018), exact winding number, signed distance | MESH-01 | P0 | Todo | 1 M-tri BVH build < 150 ms (and within 2× of `obvhs` PLOC in a dev-bench). Ray cast and closest point equal brute force on 10 k random queries (and agree with `parry3d-f64` in a dev-test). Fast winding number within 0.01 of exact at β = 2 for points ≥ 1 mean edge length off the surface. 1 M FWN queries on a 1 M mesh < 1.5 s. Results are identical across 1/4/16 rayon threads. |
| MESH-19 | Self-intersection detection (BVH self-traversal + exact `robust` predicates) | MESH-03 | P0 | Todo | On 10 k-tri random soups, the pair set equals the O(n²) exact brute force. No false positives for edge/vertex-adjacent triangles on the corpus's clean models. Intersecting-spheres synthetic case is found. 1 M-tri check < 500 ms. |
| MESH-20 | Hole filling (Liepa: DP triangulation, refinement, harmonic/bi-harmonic fairing; CDT for planar loops) | MESH-02 | P0 | Todo | All synthetic holes with boundary 3–2000 edges are filled; result is watertight with no new non-manifold elements. Patch mean edge length within 20 % of the surrounding edges. Fair-filled hole in a sphere (r = 20 mm) deviates < 2 % r. A 500-edge hole fills in < 200 ms. |
| MESH-21 | Self-intersection removal (local delete + k-ring + refill, iterative) | MESH-19, MESH-20 | P0 | Todo | ≥ 85 % of corpus models flagged `self_intersections` and `expect_repairable` become SI-free. Hausdorff distance to input ≤ 1 % of bbox diagonal. A watertight input never becomes non-watertight. Remaining pairs are reported. |
| MESH-04 | Repair pipeline (weld, degenerate, duplicate, non-manifold, small shells, orient, holes, SI) + `RepairReport` | MESH-02, MESH-03, MESH-20, MESH-21 | P0 | Todo | ≥ 95 % of `expect_repairable` corpus models become watertight (closed + manifold + oriented, outward), and ≥ 85 % are also SI-free. Repair is idempotent (second run changes nothing). No step increases any defect count. 1 M-tri typical repair < 3 s. |
| MESH-05 | Booleans via Manifold: `amg-manifold-sys` (vcpkg + overlay port, TBB), `boolean`/`boolean_n`, group mapping, input auto-repair, `manifold` feature flag. **Gate: ADR-003 (cleared 2026-10-20)** | MESH-01, MESH-04 | P0 | Todo | Builds on Windows MSVC via vcpkg (Linux best-effort). Two 1 M-tri offset spheres union < 1 s end-to-end including conversion. Volume identity vol(A∪B)+vol(A∩B) = vol(A)+vol(B) within 1e-6 relative (property test, 200 random pairs). Output always watertight. Group ids survive (A kept, B re-allocated). 10 runs with TBB are bit-identical (or canonical sort documented). Without the feature: compiles, and `boolean` returns `BackendUnavailable`. |
| MESH-06 | Isotropic remesh (Botsch–Kobbelt) with feature-edge, boundary and group-border preservation, projection to input | MESH-01, MESH-03 | P0 | Todo | On corpus watertight models and synthetic shapes: ≥ 95 % of triangles have min angle ≥ 30°; ≥ 90 % of edges within [0.8 L, 1.33 L]; Hausdorff to input ≤ 0.5 L. Cube feature edges stay within 1e-3 mm. Topology (genus, shells) unchanged. 1 M → ~1 M tris, 5 iterations, < 10 s. |
| MESH-22 | Adaptive remesh (curvature sizing field, tolerance ε) | MESH-06 | P1 | Todo | Max deviation from input ≤ ε on synthetic surfaces (sphere, torus, displaced sphere). Triangle count ≤ 60 % of uniform remesh at the same max deviation. |
| MESH-07 | QEM decimation (boundary/group/feature penalties, flip and quality checks, deterministic) | MESH-01 | P0 | Todo | 1 M → 100 k in < 2 s. Symmetric Hausdorff ≤ 0.2 % of bbox diagonal on the displaced-sphere benchmark. A closed manifold input stays closed and manifold. Group borders are kept when `preserve_group_borders`. Output is bit-identical across thread counts. |
| MESH-08 | Smoothing: Laplacian (uniform/cotan), Taubin, bilateral normal filtering | MESH-01 | P0 | Todo | Taubin (50 iterations) on a noisy sphere (σ = 1 % r) shrinks volume by < 1 % and cuts RMS noise ≥ 3×. Bilateral on a noisy cube keeps feature dihedral angles within 5° of 90° and cuts RMS noise ≥ 3×. 1 M tris × 10 Taubin iterations < 300 ms. |
| MESH-09 | Plane cut with CDT capping, `section`, separate shells, erase & fill, reverse normals | MESH-04 | P0 | Todo | Cutting a closed 1 M-tri mesh gives closed manifold halves whose volumes sum to the original within 1e-9 relative, including cuts exactly through vertices and edges, in < 300 ms. Cap faces are in one new `Cap` group. `separate_shells` count equals the analysis shell count. Erase & fill output is watertight. Reverse normals negates signed volume. |
| MESH-17 | Isosurface extraction: sparse marching tetrahedra over `ScalarField` (`amg_mesh::iso`) | MESH-01 | P0 | Review (wp/MESH-17-iso) | Fuzz test over 1 000 random CSG fields (spheres, boxes, exact zeros on grid points, surfaces touching `bounds`): output is always a closed, oriented 2-manifold. Hausdorff to analytic sphere/torus ≤ cell size. 400³-effective narrow-band extraction of a 100 mm part < 2 s. Output is identical across thread counts. |
| MESH-10 | Shell/offset: quick normal offset (`amg-mesh`) + robust SDF shell (`amg-features::mesh::shell`, composes IMP-03 `MeshSdf` + MESH-17) | MESH-04, MESH-17, IMP-03 | P0 | Todo | Robust shell output is closed, manifold and SI-free (MESH-19 check) on 100 % of watertight corpus models. Measured wall thickness within ±1 voxel of target. Quick shell of 1 M tris < 300 ms, with its SI count reported. Robust shell of a 1 M-tri, 100 mm part at 0.25 mm voxel < 20 s. |
| MESH-18 | Wrap repair (SDF dilate→erode closing, `amg-features::mesh::wrap`) | MESH-17, MESH-07, IMP-03 | P1 | Todo | 100 % of corpus models (including `expect_repairable = false`) give a watertight, SI-free result. Hausdorff to input ≤ 2× (offset + voxel). 1 M-tri scan at 0.5 mm voxel < 30 s. |
| MESH-16 | Mesh bodies in the timeline: `BodyKind::Mesh(MeshBody)` integration, `MeshEntityRef` resolution (group id → fingerprint fallback), feature base (`OutputMode`), `MeshRepairFeature`, `MeshReverseNormalFeature` | DOC-02, DOC-05, MESH-04 | P0 | Todo | Repair and Reverse Normal run as features: undo/redo restores the exact mesh hash; re-editing params recomputes only dependents; cached results are hit on unchanged inputs. A downstream reference to a face group resolves after an upstream repair. Baked mesh body survives a `.amg` save/load bit-exact (with DOC-07 when available, else via `MeshBody` binary). |
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
