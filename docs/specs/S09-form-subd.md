# S09 — Form workspace (subdivision surfaces)

| | |
|---|---|
| **Status** | Ready |
| **Priority** | P2 |
| **Crates** | `amg-subd`, `amg-features::form`, `amg-ui::form` |
| **Lane** | C |
| **Depends on** | S07 (MESH-01 `IndexedMesh`, MESH-03 `Bvh`/`MeshSpatial`, MESH-16 feature pattern), S02 (BREP-03 `make_face`, BREP-10 `sew`/`check`), S20 (KH-07/KH-12 `amg-nurbs`), S01 (DOC-05 features, DOC-06 naming), S11 (UI-05, UI-06, UI-20), S10 (RND-07 overlays) |
| **Parity** | Form / T-splines section (ADR-009) |

## Summary
Freeform modeling equivalent to Fusion's Form contextual environment, using Catmull–Clark
subdivision with creases (OpenSubdiv semantics) instead of T-splines (ADR-009): primitives,
Edit Form (move/rotate/scale faces, edges and vertices with soft modification), insert edge
and point, subdivide, bridge, fill hole, weld, crease/uncrease, symmetry, pull, flatten,
thicken, then conversion to a NURBS B-rep for downstream solid features, or to a mesh for
printing. Design detail and prior art: `docs/notes/subd-modeling.md`.

## Scope
**In:**
- `amg-subd`: polygon control cage with stable element ids and crease tags; uniform
  refinement and limit masks; exact limit evaluation (patch table); stencil tables for
  interactive edits; topology ops; primitives; symmetry; SubD → mesh; SubD → NURBS B-rep;
  linear-solve tools (interpolate, pull, flatten, thicken).
- `amg-features::form`: `form.create` (one timeline node per form, like Fusion) and
  `form.convert` (to B-rep or mesh).
- `amg-ui::form`: the FORM contextual tab inside Design (QUESTIONS #25), Create Form / Finish
  Form, display modes (box, control frame, smooth), the Edit Form tool, topology and symmetry
  tools, the Form `BodyTessellator`.
- Later (P3): pull/flatten/interpolate, match, sketch-driven form creation, quad remesh
  (mesh → cage), foreign T-spline/SubD import (ADR-009 note).

**Out:**
- T-splines and T-junctions (ADR-009). Foreign T-spline cages are converted on import (FRM-10).
- Parametric history inside a form (Fusion has none either).
- Booleans on SubD cages. A Form body combined with others goes through B-rep (after
  `form.convert`) or the mesh path with a warning (architecture §Geometry representations).
- GPU evaluation. CPU stencils meet the budgets (note §4.3); revisit only if FRM-03 misses them.
- 3MF/STL writers (S12) and print setups (S16): they call `mesh_for_export` (Interfaces).

## Design
Full detail in `docs/notes/subd-modeling.md`. Short version:

- **Cage** (note §1). `SubdCage`: n-gon half-edge mesh, 2-manifold with boundary, own storage
  (`amg-mesh::HalfEdgeMesh` is triangle-only). Stable `VertId`/`EdgeId`/`FaceId` (monotonic,
  never reused). Edge and vertex sharpness as `f32` in `[0, 10]` (10 = infinite), freeze flags,
  symmetry map. Immutable: every op returns a new cage or an error. Binary `AMGS` encoding,
  blake3 content hash.
- **Options** (OpenSubdiv `Sdc::Options`): boundary `EdgeOnly` / `EdgeAndCorner` (default),
  crease method `Uniform` (default) / `Chaikin`, triangle rule `Catmark` (default) / `Smooth`.
- **Evaluator** (note §2–§4). Own Rust code; OpenSubdiv is a test oracle behind feature
  `osd-oracle` (fixtures committed under `testdata/subd/osd/`, so CI never builds OpenSubdiv).
  Uniform refinement + limit masks for display and mesh output; a feature-adaptive patch table
  (bicubic B-spline patches, Gregory end caps at isolation level 6) for exact evaluation and
  closest point; stencil tables so a drag is one sparse mat-vec.
- **Timeline** (note §5). `form.create` stores the authored cage (base64 `AMGS` in
  `design.json`). Each edit inside the form is one `EditFeature` swapping the cage `Arc`, so
  undo is per edit. The body is `BodyGeom::Form(Arc<SubdMesh>)`. `form.convert` turns it into a
  Solid (FRM-06) or a Mesh (FRM-16). Finish Form adds the B-rep convert node by default
  (decided, QUESTIONS #101; a preference can switch it off), so solid features work on forms as in Fusion.
- **References.** Inside the form, tools use cage ids directly (authored data). From outside,
  Form bodies stay body-level only (S01 §Persistent naming, unchanged). The converted B-rep tags
  faces by root cage face and edges by cage edge (note §7.5), so downstream fillets survive form
  edits.
- **SubD → NURBS** (note §7). One patch per cage quad (n-gons give n patches; a quad with two or
  more extraordinary vertices is split in 4 first): regular quads → exact bicubic
  B-spline patches; quads at an extraordinary vertex → one bicubic surface each with a double
  knot at ½ (Peters 2000; Rhino's "local G1x" shape), G1 across the vertex and its edges.
  `make_face` per patch, `sew` with history, `make_solid` when closed. "Packed" output (FRM-11)
  merges regular grids.
- **Linear-solve tools** (note §8). Limit positions are `L · P`; interpolate, pull, flatten and
  thicken solve a damped sparse least-squares problem with `faer` (crate-internal, ADR-006).

### Fusion behaviour we match (Form contextual environment)
| Fusion | Amalgam | WP |
|---|---|---|
| Create Form → FORM contextual tab; Finish Form | `ContextId("form.edit")`, contextual tab FORM in Design; Finish Form commits and exits | FRM-17 |
| Box, plane, cylinder, sphere, torus, quadball, pipe, face | Same primitives with Fusion's parameters (faces per direction, symmetry) | FRM-02 |
| Extrude, revolve, sweep, loft (from sketches) | Same, from sketch curves | FRM-09 |
| Edit Form, soft modification, Alt-drag adds geometry, selection loop/ring/grow/shrink/range, Freeze | Same | FRM-03 |
| Insert edge, insert point, subdivide, delete | Same | FRM-04 |
| Bridge, fill hole, erase and fill, weld vertices, unweld edges, bevel edge, slide edge, merge edge | Same | FRM-18 |
| Crease, uncrease; symmetry mirror/circular internal and duplicate, clear, isolate | Same, plus semi-sharp sharpness value (beyond Fusion) | FRM-05 |
| Display mode box / control frame / smooth | Same (Ctrl+1/2/3 in the form context) | FRM-17 |
| Convert (T-spline → B-rep), automatic B-rep for solid tools after Finish | `form.convert` (B-rep, unpacked G1 / packed); added by Finish Form | FRM-06, FRM-11, FRM-17 |
| Thicken | Same, result stays a cage | FRM-12 |
| Pull, flatten, interpolate | Same | FRM-07 |
| Match | Same (position/tangent) | FRM-13 |
| Repair body | Cage validation report + fix of zero-length edges and flipped faces | FRM-13 |
| Make uniform | Not needed (Catmull–Clark is uniform) | — |
| (no Fusion equivalent) Convert Form → mesh for printing | `form.convert` target Mesh | FRM-16 |

### UI (`amg-ui::form`)
- Registers through the S11 framework only (`UiModule`, `Registrar`): toolbar tab FORM with
  `contextual: Some(ContextId("form.edit"))` in Design, groups CREATE / MODIFY / SYMMETRY /
  UTILITIES / FINISH, the Form `BodyTessellator`, `FeatureUiInfo` for `form.create` with
  `EditAction::StartTool` (re-enters the form) and for `form.convert` with `GenericDialog`.
- The control frame (cage edges, vertices) is drawn as RND-07 overlays by the active tool.
- Picking of cage elements is tool-internal (note §6); no new `EntityFilter` bits.
- No framework change is needed.

## Interfaces
Types from `amg-core` (S00): `Point3`, `Vec3`, `Dir3`, `Plane`, `Transform`, `Aabb3`, `Ray3`,
`Tolerance`, `Cancelled`. From `amg-jobs`: `JobCtx`. From `amg-mesh` (S07): `IndexedMesh`,
`FaceGroupId`. From `amg-nurbs` (S20): `NurbsSurface3`. From `amg-brep` (S02): `Shape`,
`geom::Tag`, `OpResult`. Provisional until FRM-01/FRM-15/FRM-16/FRM-06 land; changes after
`Ready` go under Interface changes.

```rust
// ===== amg_subd (crate root) =====
pub use cage::{SubdCage, CageBuilder, VertId, EdgeId, FaceId, HalfEdgeId};
pub use options::{SubdOptions, BoundaryInterp, CreaseMethod, TriangleRule};
pub use mesh::SubdMesh;
pub use error::{SubdError, SubdResult};

pub const SHARPNESS_INFINITE: f32 = 10.0;

// ----- ids: stable within a cage, never reused, persisted in the encoding -----
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct VertId(pub u32);
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct EdgeId(pub u32);
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct FaceId(pub u32);
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct HalfEdgeId(pub u32);

// ----- options (OpenSubdiv Sdc::Options semantics) -----
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum BoundaryInterp { EdgeOnly, #[default] EdgeAndCorner }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum CreaseMethod { #[default] Uniform, Chaikin }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum TriangleRule { #[default] Catmark, Smooth }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub struct SubdOptions { pub boundary: BoundaryInterp, pub crease: CreaseMethod, pub triangles: TriangleRule }

// ----- the cage (FRM-01) -----
/// Polygon control cage: 2-manifold with boundary, consistently oriented. Immutable;
/// `Clone` is a deep copy, share it as `Arc<SubdCage>`. `Send + Sync`.
#[derive(Clone, Debug, PartialEq)]
pub struct SubdCage { /* half-edge arenas with tombstones, positions, tags, symmetry, id counters, options */ }

impl SubdCage {
    /// Faces as vertex-index lists (CCW seen from outside). Validates (manifold, orientation,
    /// no repeated vertex in a face, no zero-length edge within `tol.linear`).
    pub fn from_polygons(positions: &[Point3], faces: &[Vec<u32>], opts: SubdOptions,
                         tol: &Tolerance) -> SubdResult<SubdCage>;
    pub fn options(&self) -> SubdOptions;
    pub fn vertex_count(&self) -> usize;
    pub fn edge_count(&self) -> usize;
    pub fn face_count(&self) -> usize;
    pub fn vertices(&self) -> impl Iterator<Item = VertId> + '_;   // ascending id order
    pub fn edges(&self) -> impl Iterator<Item = EdgeId> + '_;
    pub fn faces(&self) -> impl Iterator<Item = FaceId> + '_;
    pub fn position(&self, v: VertId) -> Point3;
    pub fn face_vertices(&self, f: FaceId) -> smallvec::SmallVec<[VertId; 4]>;
    pub fn edge_vertices(&self, e: EdgeId) -> [VertId; 2];
    pub fn edge_faces(&self, e: EdgeId) -> [Option<FaceId>; 2];   // [left, right]; one None = boundary
    pub fn vertex_edges(&self, v: VertId) -> smallvec::SmallVec<[EdgeId; 6]>; // CCW fan order
    pub fn valence(&self, v: VertId) -> usize;
    pub fn is_boundary_vertex(&self, v: VertId) -> bool;
    pub fn boundary_loops(&self) -> Vec<Vec<EdgeId>>;
    pub fn is_closed(&self) -> bool;
    pub fn edge_sharpness(&self, e: EdgeId) -> f32;
    pub fn vertex_sharpness(&self, v: VertId) -> f32;
    pub fn is_frozen(&self, v: VertId) -> bool;
    pub fn symmetry(&self) -> Option<&symmetry::Symmetry>;
    pub fn bbox(&self) -> Aabb3;
    /// blake3 over the `AMGS` encoding.
    pub fn content_hash(&self) -> [u8; 32];
    pub fn write_binary<W: std::io::Write>(&self, w: W) -> std::io::Result<()>; // `AMGS` v1, checksummed
    pub fn read_binary<R: std::io::Read>(r: R) -> SubdResult<SubdCage>;          // validates

    // --- position and tag edits (FRM-01; topology unchanged, so stencils stay valid) ---
    pub fn with_positions(&self, moved: &[(VertId, Point3)]) -> SubdResult<SubdCage>;
    pub fn transformed(&self, verts: &[VertId], t: &Transform, falloff: &edit::SoftFalloff) -> SubdCage; // FRM-03
    pub fn with_edge_sharpness(&self, edges: &[EdgeId], s: f32) -> SubdResult<SubdCage>;  // FRM-05
    pub fn with_vertex_sharpness(&self, verts: &[VertId], s: f32) -> SubdResult<SubdCage>; // FRM-05
    pub fn with_frozen(&self, verts: &[VertId], frozen: bool) -> SubdCage;                // FRM-03
    /// True if `other` has the same topology and tags (stencil tables can be reused).
    pub fn same_topology(&self, other: &SubdCage) -> bool;
}

/// For tests and primitives: incremental construction with explicit ids.
pub struct CageBuilder { /* … */ }
impl CageBuilder {
    pub fn new(opts: SubdOptions) -> Self;
    pub fn vertex(&mut self, p: Point3) -> VertId;
    pub fn face(&mut self, verts: &[VertId]) -> SubdResult<FaceId>;
    pub fn crease(&mut self, a: VertId, b: VertId, s: f32) -> SubdResult<EdgeId>;
    pub fn finish(self, tol: &Tolerance) -> SubdResult<SubdCage>;
}

// ----- refinement and evaluation (FRM-01, FRM-15) -----
pub mod eval {
    use super::*;
    /// A point of the refined limit mesh, with the root cage face it lies in.
    #[derive(Copy, Clone, Debug, PartialEq)]
    pub struct LimitSample { pub point: Point3, pub du: Vec3, pub dv: Vec3, pub face: FaceId, pub uv: [f64; 2] }

    /// Uniform refinement to `level` (0..=6), vertices moved to the limit surface.
    #[derive(Clone, Debug)]
    pub struct LimitMesh {
        pub positions: Vec<Point3>,
        pub normals: Vec<Dir3>,
        pub quads: Vec<[u32; 4]>,
        /// Root cage face of each quad (picking, highlights, B-rep tags).
        pub quad_face: Vec<FaceId>,
        pub level: u8,
    }
    pub fn refine(cage: &SubdCage, level: u8) -> LimitMesh;                     // FRM-01
    /// Refined but *not* limit-projected cage points at `level` (oracle comparison, FRM-01).
    pub fn refine_points(cage: &SubdCage, level: u8) -> (Vec<Point3>, Vec<Vec<u32>>);
    /// Limit position and tangents of every cage vertex (limit masks).
    pub fn limit_of_vertices(cage: &SubdCage) -> Vec<LimitSample>;              // FRM-01

    /// Exact evaluation (FRM-15): feature-adaptive patches, Gregory end caps.
    pub struct PatchTable { /* … */ }
    #[derive(Copy, Clone, Debug, PartialEq)]
    pub struct SurfaceEval { pub point: Point3, pub du: Vec3, pub dv: Vec3,
                             pub duu: Vec3, pub duv: Vec3, pub dvv: Vec3 }
    #[derive(Copy, Clone, Debug, PartialEq)]
    pub struct ClosestPoint { pub face: FaceId, pub uv: [f64; 2], pub point: Point3, pub distance: f64 }
    impl PatchTable {
        pub fn build(cage: &SubdCage, isolation_level: u8 /* default 6, max 10 */) -> PatchTable;
        /// `uv` in the root face's domain [0,1]²; n-gons use n sub-domains (`sub`).
        pub fn eval(&self, face: FaceId, sub: u8, uv: [f64; 2]) -> SurfaceEval;
        pub fn closest_point(&self, p: Point3, max_dist: f64) -> Option<ClosestPoint>;
        pub fn ray_cast(&self, ray: &Ray3, max_t: f64) -> Option<(f64, ClosestPoint)>;
    }

    /// Display points as fixed linear combinations of cage vertices (FRM-15).
    pub struct StencilTable { /* sparse rows (f64), row index per cage vertex */ }
    impl StencilTable {
        pub fn build(cage: &SubdCage, level: u8) -> StencilTable;
        /// Recompute every display point and normal from cage positions.
        pub fn apply(&self, cage: &SubdCage, out: &mut LimitMesh);
        /// Recompute only rows that depend on `moved`; returns the touched display vertices.
        pub fn apply_partial(&self, cage: &SubdCage, moved: &[VertId], out: &mut LimitMesh) -> Vec<u32>;
    }
}

// ----- evaluated body payload (FRM-16) -----
/// `BodyGeom::Form(Arc<SubdMesh>)`. Immutable, `Send + Sync`; caches are pure functions of
/// the cage, built on first use.
#[derive(Clone, Debug)]
pub struct SubdMesh { /* cage: Arc<SubdCage>, stencils: [OnceLock<Arc<StencilTable>>; 7], patches: OnceLock<Arc<PatchTable>> */ }
impl SubdMesh {
    pub fn new(cage: Arc<SubdCage>) -> SubdMesh;
    pub fn cage(&self) -> &Arc<SubdCage>;
    pub fn stencils(&self, level: u8) -> &Arc<eval::StencilTable>;
    pub fn patches(&self) -> &Arc<eval::PatchTable>;
    /// Triangle mesh of the limit surface at `level` (2 triangles per quad, one face group per
    /// root cage face, groups numbered by `FaceId`). Closed + manifold when the cage is closed.
    pub fn to_mesh(&self, level: u8) -> IndexedMesh;
    /// Smallest level whose chordal error is ≤ `chord_tol` (mm), capped at 6.
    pub fn to_mesh_tol(&self, chord_tol: f64, ctx: &JobCtx) -> Result<IndexedMesh, Cancelled>;
    pub fn content_hash(&self) -> [u8; 32];
}

// ----- primitives (FRM-02) -----
pub mod primitive {
    use super::*;
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum FormPrimitive {
        Box      { frame: Plane, size: Vec3, faces: [u32; 3] },
        Plane    { frame: Plane, size: [f64; 2], faces: [u32; 2] },
        Cylinder { frame: Plane, radius: f64, height: f64, faces_around: u32, faces_height: u32 },
        Sphere   { frame: Plane, radius: f64, faces_longitude: u32, faces_latitude: u32 },
        Torus    { frame: Plane, major: f64, minor: f64, faces_around: u32, faces_tube: u32 },
        Quadball { frame: Plane, radius: f64, span_faces: u32 },
        Face     { corners: Vec<Point3> /* 3 or 4, the "Face" tool */ },
        Pipe     { path: Vec<Point3>, radius: f64, faces_around: u32, closed: bool },
    }
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
    pub enum PrimitiveSymmetry { #[default] None, Mirror { axes: [bool; 3] }, Circular { segments: u32 } }
    pub fn build(p: &FormPrimitive, sym: PrimitiveSymmetry, opts: SubdOptions) -> SubdResult<SubdCage>;
}

// ----- selection queries and soft edits (FRM-03) -----
pub mod edit {
    use super::*;
    #[derive(Clone, Debug, Default, PartialEq, Eq)]
    pub struct CageSelection { pub verts: Vec<VertId>, pub edges: Vec<EdgeId>, pub faces: Vec<FaceId> }
    impl CageSelection { pub fn vertices(&self, cage: &SubdCage) -> Vec<VertId>; } // closure of all three
    pub fn edge_loop(cage: &SubdCage, e: EdgeId) -> Vec<EdgeId>;
    pub fn edge_ring(cage: &SubdCage, e: EdgeId) -> Vec<EdgeId>;
    pub fn face_loop(cage: &SubdCage, e: EdgeId) -> Vec<FaceId>;
    pub fn grow(cage: &SubdCage, s: &CageSelection) -> CageSelection;
    pub fn shrink(cage: &SubdCage, s: &CageSelection) -> CageSelection;
    pub fn range(cage: &SubdCage, a: VertId, b: VertId) -> Option<Vec<EdgeId>>;   // shortest loop path
    #[derive(Copy, Clone, Debug, PartialEq, Default, serde::Serialize, serde::Deserialize)]
    pub enum FalloffCurve { Linear, #[default] Smooth }
    #[derive(Copy, Clone, Debug, PartialEq, Default, serde::Serialize, serde::Deserialize)]
    pub enum SoftFalloff { #[default] None, Distance { radius: f64, curve: FalloffCurve }, Rings { n: u32 } }
}

// ----- topology ops (FRM-04, FRM-18); each returns a new valid cage -----
pub mod topo {
    use super::*;
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
    pub enum InsertMode { #[default] Simple, Exact /* keep the limit surface: re-fit neighbours */ }
    pub struct TopoResult { pub cage: SubdCage, pub created: edit::CageSelection }
    // FRM-04
    pub fn insert_edge(c: &SubdCage, ring: &[EdgeId], t: f64, mode: InsertMode) -> SubdResult<TopoResult>;
    pub fn insert_point(c: &SubdCage, a: (EdgeId, f64), b: (EdgeId, f64), mode: InsertMode) -> SubdResult<TopoResult>;
    pub fn subdivide(c: &SubdCage, faces: &[FaceId], mode: InsertMode) -> SubdResult<TopoResult>;
    pub fn delete(c: &SubdCage, s: &edit::CageSelection) -> SubdResult<TopoResult>;
    pub fn extrude(c: &SubdCage, s: &edit::CageSelection, keep_creases: bool) -> SubdResult<TopoResult>;
    // FRM-18
    pub fn bridge(c: &SubdCage, a: &[EdgeId], b: &[EdgeId], segments: u32, twist: i32) -> SubdResult<TopoResult>;
    #[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum FillMode { ReducedStar, FillStar, Collapse }
    pub fn fill_hole(c: &SubdCage, boundary_edge: EdgeId, mode: FillMode) -> SubdResult<TopoResult>;
    pub fn erase_and_fill(c: &SubdCage, faces: &[FaceId]) -> SubdResult<TopoResult>;
    pub fn weld_vertices(c: &SubdCage, pairs: &[(VertId, VertId)], tol: f64) -> SubdResult<TopoResult>;
    pub fn unweld_edges(c: &SubdCage, edges: &[EdgeId]) -> SubdResult<TopoResult>;
    pub fn merge_edges(c: &SubdCage, a: &[EdgeId], b: &[EdgeId]) -> SubdResult<TopoResult>;
    pub fn bevel_edges(c: &SubdCage, edges: &[EdgeId], width: f64, segments: u32) -> SubdResult<TopoResult>;
    pub fn slide_edges(c: &SubdCage, edges: &[EdgeId], t: f64) -> SubdResult<SubdCage>;
}

// ----- symmetry (FRM-05) -----
pub mod symmetry {
    use super::*;
    #[derive(Clone, Debug, PartialEq)]
    pub enum SymmetryKind { Mirror(Plane), Circular { axis: Ray3, segments: u32 } }
    /// Vertex map closed under the symmetry group; vertices on a mirror plane map to themselves.
    #[derive(Clone, Debug, PartialEq)]
    pub struct Symmetry { pub kind: SymmetryKind, /* pairs/orbits by VertId */ }
    pub fn set_internal(c: &SubdCage, kind: SymmetryKind, tol: f64) -> SubdResult<SubdCage>; // match by position
    pub fn duplicate(c: &SubdCage, kind: SymmetryKind, weld_tol: f64) -> SubdResult<SubdCage>;
    pub fn clear(c: &SubdCage) -> SubdCage;
    pub fn isolate(c: &SubdCage, verts: &[VertId]) -> SubdCage;   // drop these from the map
    /// Closure of `moved` under the symmetry with mirrored positions.
    pub fn propagate(c: &SubdCage, moved: &[(VertId, Point3)]) -> Vec<(VertId, Point3)>;
}

// ----- conversion (FRM-06, FRM-11) -----
pub mod convert {
    use super::*;
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
    pub enum ExtraordinaryMode { #[default] G1, Approximate }
    #[derive(Copy, Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct BrepOpts { pub extraordinary: ExtraordinaryMode, pub packed: bool /* FRM-11 */, pub sew_tol: f64 }
    impl Default for BrepOpts { fn default() -> Self; }   // G1, unpacked, sew_tol 1e-6 mm
    #[derive(Clone, Debug)]
    pub struct PatchSurface { pub surface: NurbsSurface3, pub face: FaceId, pub sub: u8,
                              pub edge_tags: [amg_brep::geom::Tag; 4] }
    #[derive(Clone, Debug, PartialEq)]
    pub struct ConversionReport { pub patches: usize, pub extraordinary_vertices: usize,
                                  pub max_deviation: f64 /* mm, to the limit surface */,
                                  pub max_g1_angle_ev: f64 /* rad */, pub max_g1_angle_regular: f64 }
    /// Pure-Rust step: one NURBS patch per (refined) quad, with tags (no OCCT).
    pub fn to_patches(m: &SubdMesh, o: &BrepOpts, ctx: &JobCtx) -> Result<(Vec<PatchSurface>, ConversionReport), SubdError>;
    /// Faces + sew + solid via the kernel. `OpResult.history` carries the face/edge tags.
    pub fn to_brep(m: &SubdMesh, k: &dyn amg_brep::BrepKernel, o: &BrepOpts, ctx: &JobCtx)
        -> Result<(amg_brep::OpResult, ConversionReport), SubdError>;
    /// Tag layout (note §7.5).
    pub fn face_tag(face: FaceId, sub: u8) -> amg_brep::geom::Tag;
    pub fn edge_tag(edge: EdgeId, half: u8) -> amg_brep::geom::Tag;
}

// ----- linear-solve tools (FRM-07, FRM-12, FRM-13) -----
pub mod fit {
    use super::*;
    pub trait Target: Sync { fn closest(&self, p: Point3) -> Option<Point3>; }
    pub fn interpolate(c: &SubdCage, targets: &[(VertId, Point3)], damping: f64) -> SubdResult<SubdCage>; // FRM-07
    pub fn pull(c: &SubdCage, verts: &[VertId], target: &dyn Target, iterations: u32, ctx: &JobCtx) -> SubdResult<SubdCage>;
    pub fn flatten(c: &SubdCage, verts: &[VertId], plane: Option<Plane>) -> SubdResult<SubdCage>;
    #[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum ThickenEdges { Sharp, Soft }
    pub fn thicken(c: &SubdCage, thickness: f64 /* <0 inward */, edges: ThickenEdges) -> SubdResult<SubdCage>; // FRM-12
    #[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum MatchContinuity { Position, Tangent }
    pub fn match_boundary(c: &SubdCage, edges: &[EdgeId], curve: &[Point3], tangents: Option<&[Vec3]>,
                          cont: MatchContinuity) -> SubdResult<SubdCage>;                          // FRM-13
    pub fn repair(c: &SubdCage, tol: &Tolerance) -> (SubdCage, Vec<String>);                       // FRM-13
}

// ----- errors -----
#[derive(Debug, thiserror::Error)]
pub enum SubdError {
    #[error("cage is not manifold at vertex {0:?}")] NonManifold(VertId),
    #[error("faces around edge {0:?} have inconsistent orientation")] Orientation(EdgeId),
    #[error("degenerate element: {0}")] Degenerate(String),
    #[error("invalid selection for this operation: {0}")] InvalidSelection(String),
    #[error("element {0} does not exist in this cage")] Missing(String),
    #[error("encoding: {0}")] Encoding(String),
    #[error("solve did not converge (residual {residual:e} mm)")] NoConvergence { residual: f64 },
    #[error(transparent)] Kernel(#[from] amg_brep::KernelError),
    #[error(transparent)] Cancelled(#[from] Cancelled),
}
impl amg_core::Diagnostic for SubdError {}
pub type SubdResult<T> = Result<T, SubdError>;

/// serde helper: a cage as base64 of its `AMGS` bytes (used by `form.create`).
pub mod serde_b64 { /* serialize / deserialize for Arc<SubdCage> */ }
```

```rust
// ===== amg_features::form (FRM-16, FRM-06) =====
pub use base::{FormFeatureError, input_form};

/// One Form node in the timeline (Fusion "Form1"). Holds the authored cage; edits inside the
/// form replace the cage via `EditFeature`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FormFeature {
    #[serde(with = "amg_subd::serde_b64")]
    pub cage: Arc<amg_subd::SubdCage>,
    /// Body slot 0 is created; `Some(body)` edits an existing Form body in place (Fusion
    /// "Edit Form" on a body made by an earlier form).
    pub replaces: Option<amg_doc::BodyId>,
}
amg_doc::register_feature!(FormFeature, "form.create", version = 1);

#[derive(Copy, Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum ConvertTarget {
    Brep(amg_subd::convert::BrepOpts),        // FRM-06 (packed: FRM-11)
    Mesh { chord_tol: f64 /* mm, default 0.01 */ }, // FRM-16
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FormConvertFeature {
    pub body: amg_doc::BodyId,
    pub target: ConvertTarget,
    pub output: amg_features::mesh::OutputMode,   // Replace (default) | NewBody; same enum as mesh
}
amg_doc::register_feature!(FormConvertFeature, "form.convert", version = 1);

/// For S12 export and S16 build items: a Form body as a watertight mesh at export quality.
/// Cached by (cage hash, chord_tol); a second call returns the same `Arc`.
pub fn mesh_for_export(m: &amg_subd::SubdMesh, chord_tol: f64, ctx: &amg_jobs::JobCtx)
    -> Result<Arc<amg_mesh::IndexedMesh>, amg_core::Cancelled>;

#[derive(Debug, thiserror::Error)]
pub enum FormFeatureError {
    #[error("body is not a Form body")] NotAForm,
    #[error(transparent)] Subd(#[from] amg_subd::SubdError),
}
pub fn link() {}
```

```rust
// ===== amg_ui::form (FRM-17, FRM-02, FRM-03, FRM-04, FRM-05, FRM-18) =====
pub const FORM_CONTEXT: amg_ui::ContextId = amg_ui::ContextId("form.edit");
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum FormDisplay { Box, ControlFrame, #[default] Smooth }      // Ctrl+1/2/3
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum FinishOutput { #[default] Brep, Subd }                    // preference; Brep default decided, QUESTIONS #101
/// `BodyGeom::Form` → display mesh: Preview level 1, Normal level 2, Fine level 3; per-quad
/// root `FaceId` as the render sub-id.
pub struct FormTessellator;
impl amg_ui::scene::BodyTessellator for FormTessellator { /* … */ }
pub fn register(r: &mut amg_ui::module::Registrar<'_>);
// inventory::submit! { amg_ui::module::UiModule { id: "form", register: crate::form::register } }
```

**Cross-spec contracts** (each recorded in that spec's Interface changes, 2026-11-08):
- **S01:** `amg-doc::stub::SubdMesh` becomes `pub use amg_subd::SubdMesh;` (immutable,
  `Send + Sync`, `Clone + Debug`, no `Default`/`Eq`). The body codec stores Form bodies as the
  cage's `AMGS` bytes; the `stub:form` tag stays readable as an empty placeholder until DOC-07
  freezes the format. Form bodies stay body-level only for references. Done in FRM-16.
- **S16 (and S12 export):** Form build items and exported Form bodies resolve through
  `amg_features::form::mesh_for_export`, as implicit bodies do through S08's.
- **S02, S20, S07, S11:** no change. FRM-06 uses `make_face` (B-spline surface, `Natural`),
  `sew` with history (R8) and `check`; `From<&NurbsSurface3> for BSplineSurfaceData`
  (KH-07); `Bvh`/`MeshSpatial` (MESH-03); `BodyTessellator`, `EditAction::StartTool`,
  contextual tabs and UI-06 triads.
- **`amg-subd` Cargo.toml** gains `amg-nurbs` (allowed edge, QUESTIONS #41), `faer`
  (FRM-07/FRM-12, already a workspace dependency of S13), `smallvec`, `blake3`, `base64`
  (MIT/Apache-2.0; listed in `THIRD_PARTY.md` by the WP that adds them). The crate's doc
  comment says "Lane: A"; FRM-01 corrects it to lane C.

## Work packages
WPs are ≈ 1–3 days each. FRM-01..08 keep their IDs (some re-scoped, see Refinement log);
FRM-09..18 are new. Reference cages C1–C3 are defined under Performance targets. Times are on
the reference machine (vision doc), release build. Golden tests live in `tests/golden/subd/`.

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| FRM-01 | Control cage + uniform refinement: `SubdCage` (n-gon half-edge, stable ids, sharpness/freeze tags, validation, `AMGS` encoding, blake3 hash, `CageBuilder`), `SubdOptions`, Catmull–Clark refinement with OpenSubdiv rules (smooth/crease/corner/dart, semi-sharp Uniform and Chaikin, both boundary modes, both triangle rules), limit masks, `refine`, `limit_of_vertices`; `osd-oracle` fixture tool + committed fixtures | MESH-01 | P2 | Todo | 1. On the fixture set (note §2: every rule, sharpness 0.5/1.5/2.5, Chaikin on/off, both boundary modes, both triangle rules, valence 3/5/6/8, pentagons), refined points at levels 1–3 and limit points match the committed OpenSubdiv fixtures within 1e-12 × bbox diagonal. 2. `AMGS` round trip is bit-exact and the hash is equal across runs and 1/4/16 threads; a corrupted byte fails decode with `Encoding`. 3. `from_polygons` rejects non-manifold and inconsistently oriented input with the offending element (proptest, 512 cases). 4. Ids are never reused across 1 000 random edits (proptest). 5. `refine` of C2 at level 3 < 20 ms. |
| FRM-15 | Exact evaluation and stencils: `PatchTable` (feature-adaptive, bicubic B-spline regular patches incl. boundary/crease phantom rules, Gregory end caps, `eval` with 2nd derivatives, `closest_point`, `ray_cast`), `StencilTable` (`apply`, `apply_partial`) | FRM-01 | P2 | Todo | 1. `PatchTable::eval` at the refined vertex parameters equals `refine` limit points within 1e-9 × diagonal (levels 1–4, fixture set); on regular faces it equals the analytic bicubic B-spline within 1e-12. 2. Near extraordinary vertices (valence 3, 5, 8), eval differs from Stam's exact evaluation (test-only implementation) by ≤ 1e-6 × diagonal at isolation level 6. 3. `closest_point` of 10 k random points within 5 mm of C2 agrees with dense-sampling brute force within 1e-6 mm. 4. `StencilTable::apply` equals `refine` within 1e-12 × diagonal; `apply_partial` after moving 10 vertices equals a full `apply`. 5. `apply` on C2 at level 3 < 2 ms (8 cores); stencil build < 100 ms. |
| FRM-16 | Form bodies in the timeline: swap `amg-doc` `SubdMesh` stub, body codec arm (`AMGS`), `SubdMesh` caches, `to_mesh`/`to_mesh_tol`, `amg-features::form` base (`input_form`, errors), `FormFeature` (`form.create`), `FormConvertFeature` with `ConvertTarget::Mesh`, `mesh_for_export` cache | FRM-15, DOC-05, MESH-16 | P2 | Todo | 1. A `form.create` with C1 is undoable (undo/redo restores the exact cage hash), round-trips through `design.json` bit-exactly and survives `.amg` save/load with an equal cage hash. 2. Ten successive cage edits give ten undo steps, each restoring the previous hash. 3. `form.convert` (Mesh, chord 0.01 mm) on closed C1 gives a closed, oriented, manifold mesh (MESH-02 report clean) whose max distance to the limit surface (FRM-15 `closest_point` on 10 k samples) is ≤ 0.01 mm. 4. A second `mesh_for_export` call with the same inputs returns the same `Arc` in < 1 ms. 5. `cargo nextest run -p amg-doc` still passes. |
| FRM-17 | Form environment shell in `amg-ui::form`: Create Form (creates the node, enters `form.edit`), FORM contextual tab and groups, Finish Form (commit, exit, add/update `form.convert` per `FinishOutput`), re-enter via timeline edit (`EditAction::StartTool`), display modes box/control frame/smooth, control-frame overlays, `FormTessellator` | FRM-16, UI-02, UI-05, UI-20, RND-07 | P2 | Todo | 1. Kittest: Create Form shows the FORM tab and hides it after Finish; Esc cancels and leaves no node. 2. Finish with `FinishOutput::Brep` adds exactly one `form.convert` after the form node; re-entering and finishing again does not add a second. 3. Ctrl+1/2/3 switch display modes (snapshot test on WARP for each). 4. A C2 body displays at Normal quality without a UI frame > 16 ms (tessellation on a job, latest-wins). 5. Display sub-ids map back to the right root `FaceId` (PickMap round trip for every face of C1). |
| FRM-02 | Form primitives: box, plane, cylinder, sphere, torus, quadball, face, pipe with Fusion's parameters and primitive symmetry; their dialogs (generic UI-05 dialogs in the FORM tab, live preview) | FRM-17 | P2 | Todo | 1. Each primitive builds a valid cage for face counts 1..=16 per direction (closed for box, cylinder with caps, sphere, torus, quadball). 2. Golden tests per primitive (volume and area of the level-4 limit mesh, bbox, vertex/face counts), e.g. sphere r = 10 mm: limit-surface radius within 1.5 % at 8×4 faces. 3. Mirror symmetry of a box primitive yields a symmetry map covering every vertex. 4. Kittest: each dialog creates its primitive with preview, OK and Cancel. |
| FRM-03 | Edit Form tool: tool-internal picking of vertices/edges/faces (filter), selection modes (loop, ring, face loop, grow/shrink, range; Fusion shortcuts), UI-06 triad (translate, planar, rotate, scale; world/view/local frame), soft modification (`SoftFalloff`), Freeze/Unfreeze, Alt-drag hook (FRM-04 `extrude`), live preview through stencils, one `EditFeature` per drag | FRM-15, FRM-17, UI-06 | P2 | Todo | 1. Kittest: pick each element type on C1 by a scripted click; loop/ring/grow/shrink select the expected id sets on a 6×6 box (table test, 12 cases). 2. Dragging a face by a scripted world delta moves its vertices by exactly that delta (1e-9 mm); frozen vertices never move. 3. Distance falloff: displacement is 1 at the selection and 0 at ≥ radius (property test). 4. Drag preview on C2 (2 000 faces, Normal quality): p95 UI-thread frame time ≤ 16 ms over a 200-frame scripted drag; on C3 updates run on a job and the UI never blocks > 16 ms. 5. One drag = one undo step. |
| FRM-04 | Topology tools I: `insert_edge` (simple/exact), `insert_point`, `subdivide` (simple/exact), `delete` (vertices/edges/faces), `extrude` (Alt-drag, keep creases); tools in `amg-ui::form` | FRM-03 | P2 | Todo | 1. Every op keeps the cage valid (proptest: 1 000 random op sequences on primitives, validation after each). 2. `Exact` insert edge and subdivide keep the limit surface: max distance before/after ≤ 1e-9 × diagonal on 10 k samples (for edges away from extraordinary vertices) and ≤ 1e-3 × diagonal next to them, reported. 3. Deleting a face of a closed box leaves one boundary loop with 4 edges. 4. Kittest per tool, each one undo step. |
| FRM-18 | Topology tools II: `bridge` (segments, twist), `fill_hole` (reduced star, fill star, collapse), `erase_and_fill`, `weld_vertices`, `unweld_edges`, `merge_edges`, `bevel_edges`, `slide_edges`; tools in `amg-ui::form` | FRM-04 | P2 | Todo | 1. Validity proptest as FRM-04. 2. Bridging the open ends of two cylinders (8 faces around) gives a closed cage with 8 × segments new faces; twist ±1 rotates the matching by one edge. 3. `fill_hole` on an n-gon boundary (n = 3..12) closes it; reduced star creates no vertex of valence > 5 for even n. 4. Weld then unweld restores the original topology (hash equal up to new ids). 5. Kittest per tool, each one undo step. |
| FRM-05 | Crease / uncrease (infinite and semi-sharp value), vertex corners; symmetry: mirror and circular internal (`set_internal`, `propagate`), mirror and circular duplicate, clear, isolate; symmetry display and tool integration | FRM-03 | P2 | Todo | 1. A creased edge loop on a box gives a limit surface with a G0 (not G1) crease: normal angle across it > 10° on the fixture, and the crease fixtures still match OpenSubdiv (FRM-01 tolerance). 2. With mirror internal symmetry, any random drag (proptest, 256 cases) leaves the cage mirror-symmetric within 1e-12 mm; on-plane vertices stay on the plane. 3. Circular symmetry with 6 segments: same property. 4. Mirror duplicate of a half box welds along the plane into a closed cage with no duplicate vertices. 5. Kittest: crease, uncrease, each symmetry command. |
| FRM-06 | SubD → NURBS B-rep (unpacked): `to_patches` (one patch per cage quad, n patches per n-gon, a quad with two or more extraordinary vertices split in 4; regular quads exact bicubic, extraordinary-vertex quads G1 with double knot, `Approximate` mode), `to_brep` (`make_face` + tags, `sew` with history, `make_solid`), `ConversionReport`, `FormConvertFeature` with `ConvertTarget::Brep`, B-rep naming from tags (DOC-06 `NamingBuilder::from_history`) | FRM-16, BREP-03, BREP-10, KH-07, DOC-06 | P2 | Todo | 1. On C1, C2 and every closed primitive the result passes `check` (BRepCheck) and is a closed solid; open primitives give a valid open shell. 2. G1: across every shared non-crease edge, normal angle ≤ 1e-6 rad on regular edges and ≤ 0.5° on edges at extraordinary vertices (9 samples per edge). 3. Deviation from the limit surface: ≤ 1e-9 mm on regular patches, ≤ 0.1 % of the bbox diagonal on extraordinary-vertex patches (16×16 samples per face), reported in `ConversionReport`. 4. Solid volume equals the level-5 limit-mesh volume within 0.1 %. 5. A fillet on a converted edge keeps its reference after a form edit that moves vertices elsewhere (golden naming test). 6. C2 (2 000 faces) converts in < 5 s as a cancellable job with progress. Golden tests `subd_to_brep_*`. |
| FRM-11 | Packed conversion: greedy rectangular grids of regular quads → one B-spline surface each (C2 interior knots, KH-12 knot ops), tags by lowest root `FaceId`, fingerprint warning | FRM-06, KH-12 | P2 | Todo | 1. A 6×6×6 box form (no creases) converts to ≤ 48 faces packed (216 unpacked) and passes `check`. 2. Packed and unpacked solids of C1/C2 have equal volume within 1e-6 relative and max distance between them ≤ 1e-9 mm on regular regions. 3. Grid choice is deterministic (same face set across runs). 4. A 3 mm fillet (S20 chain) along a box-form edge succeeds on the packed result; the unpacked result is tried too and both outcomes are reported (input for open question 6). |
| FRM-12 | Thicken: offset along limit normals, interpolating offset cage (`fit` solve, `faer`), boundary bridge (sharp/soft), closed-cage inner shell; Thicken tool | FRM-04, FRM-15 | P2 | Todo | 1. Thickening an open 8×8 plane form by 2 mm gives a closed valid cage; measured thickness of the level-4 limit mesh (MESH-11 or closest-point sampling) is 2 ± 0.05 mm away from the boundary strip. 2. A closed sphere form thickened by −1 mm gives two oppositely oriented shells, inner radius within 1 % of r − 1. 3. Converts through FRM-06 to a valid solid. 4. Kittest: Thicken tool, one undo step. |
| FRM-07 | Pull to target body (mesh via `MeshSpatial`, solid via fine tessellation), interpolate, flatten (picked or fitted plane); tools | FRM-04, FRM-15 | P3 | Todo | 1. Pulling all vertices of an 8×8×8 box form onto a r = 20 mm sphere mesh (3 iterations): limit points of the pulled vertices lie ≤ 0.01 mm from the sphere. 2. `interpolate` makes the limit surface pass through the targets within 1e-6 mm. 3. `flatten` on a face region: limit points of the region within 1e-6 mm of the plane. 4. Kittest per tool. |
| FRM-13 | Match (boundary to a sketch curve or B-rep edge, position/tangent) and Repair body (validation report, fix zero-length edges and flipped faces); tools | FRM-07 | P3 | Todo | 1. Matching a plane form's boundary to a circle edge: boundary limit curve within 0.01 mm of the circle at 64 samples; tangent mode: normal angle to the target face ≤ 0.5°. 2. Repair fixes every injected defect of a 20-case table (or reports it). 3. Kittest per tool. |
| FRM-09 | Sketch-driven form creation: extrude, revolve, sweep, loft (sketch curves → cage strips with face counts), pipe from a sketch path | FRM-02, SK-19 | P3 | Todo | 1. Revolving a sketch spline 360° with 8 faces around gives a closed cage whose limit profile is within 1 % of the sketch curve's height at 32 samples. 2. Loft of 3 circles gives an open tube; extrude of a closed sketch profile gives a capped body. 3. Golden tests per tool. 4. Kittest per tool. |
| FRM-08 | Quad remesh (mesh → SubD cage) for organic mesh → B-rep: approach per note §9 (decided when this WP is refined), fit with the §8 solve, `form.from_mesh` feature | FRM-06, FRM-07, MESH-06 | P3 | Todo | On 10 corpus organic meshes (scans, sculpts): ≥ 95 % quads, valid closed cage for closed input, Hausdorff from the limit surface to the input ≤ 0.5 % of the bbox diagonal, target face count within ±20 %, < 30 s for a 200 k-triangle input; the result converts through FRM-06 to a valid solid. |
| FRM-10 | Foreign SubD / T-spline import (ADR-009 note): OBJ quad cage as SubD (`o`/`g` per body, crease tags where present; needs a polygon-preserving IO-05 read option, requested from S12 when this WP is refined) (owner decision QUESTIONS #102, changed 2026-11-10: OBJ only; `.tsm` is out of scope until sample files exist, note §11), with T-junctions resolved by local refinement into quads, with a warning | FRM-16, IO-05 | P3 | Todo | 1. An OBJ control net exported from a SubD modeler imports as a Form body whose level-3 limit mesh matches a reference export within 0.1 % of the diagonal. 2. A cage with T-junctions imports as a valid manifold cage with the warning. 3. Import is one undoable command. |
| FRM-14 | Form performance and fuzz hardening: criterion benches C1–C3 for refine, stencils, patch table, conversion; cargo-fuzz target for `AMGS` decode; random op-sequence fuzz over FRM-04/FRM-18 | FRM-06, FRM-18 | P2 | Todo | 1. Benches for every budget under Performance targets run in CI (report only) and meet the targets locally. 2. `AMGS` decode fuzz runs 10 min with no panic. 3. 10 000 random op sequences never yield an invalid cage or a panic. |

## Testing
- **Oracle:** OpenSubdiv fixtures (FRM-01, FRM-05), generated by the `osd-oracle` tool and
  committed; Stam exact evaluation (test-only) near extraordinary vertices (FRM-15).
- **Property tests (`proptest`):** cage validity after random op sequences, id monotonicity,
  symmetry preservation under random drags, `AMGS` round trip, stencil `apply_partial` equals
  `apply`.
- **Golden geometry (`tests/golden/subd/`):** limit-mesh volume/area/bbox per primitive;
  converted solids (`check`, volume, face counts, G1 angles, deviation).
- **Differential:** `PatchTable::eval` vs `refine`; packed vs unpacked conversion; `to_mesh_tol`
  vs `closest_point`.
- **UI:** kittests per tool (`AppHarness`), WARP snapshots of the three display modes.
- **Fuzz:** `AMGS` decode, random topology sequences (FRM-14).
- **Benchmarks (`criterion`):** C1–C3 (below).

## Performance targets
Reference cages (built in `amg-subd` tests): **C1** 6×6×6 box form with 8 creased edges
(216 faces); **C2** organic character-like cage, 2 000 faces, 24 extraordinary vertices,
2 semi-sharp loops (generated from a quadball + random subdivides with a fixed seed);
**C3** 20 000 faces (C2 subdivided once, plus noise).

| Operation | Target | WP |
|---|---|---|
| `refine` C2 level 3 | < 20 ms | FRM-01 |
| `StencilTable::build` C2 level 3 | < 100 ms | FRM-15 |
| `StencilTable::apply` C2 level 3 (≈ 128 k points) | < 2 ms (8 cores) | FRM-15 |
| Edit Form drag preview C2 | p95 frame ≤ 16 ms on the UI thread | FRM-03 |
| Edit Form drag preview C3 | UI never blocks > 16 ms; preview latency < 100 ms (job) | FRM-03 |
| `to_mesh_tol(0.01 mm)` C2 | < 300 ms | FRM-16 |
| SubD → B-rep C2 (unpacked) | < 5 s (job, cancellable) | FRM-06 |
| Recompute of a document after a form edit | only the form node and its dependents recompute (vision: < 500 ms budget for typical parts) | FRM-16 |

## Open questions
1. (from S02, 2026-10-20) FRM-06 should depend on BREP-03 (B-spline `make_face`) + BREP-10 (sew).
   **Resolved 2026-11-08 (refinement):** FRM-06 deps are BREP-03, BREP-10, KH-07, DOC-06,
   FRM-16.
2. (from S20, 2026-10-20) FRM-06 should build on `amg-nurbs`; this needs the layering edge `amg-subd → amg-nurbs` (QUESTIONS #41).
   **Resolved 2026-10-20 (owner, QUESTIONS #41):** the edge is allowed (architecture crate table
   and the layering test updated). FRM-06 may depend on `amg-nurbs`.
3. Own Rust evaluator vs FFI to OpenSubdiv. **Decided 2026-11-08 (refinement), confirmed
   2026-11-09 (owner, QUESTIONS #100):** own Rust, OpenSubdiv as a test oracle behind
   `osd-oracle` (note §2).
4. Finish Form output. **Resolved 2026-11-09 (owner, QUESTIONS #101):** Finish Form adds or updates a
   `form.convert` node producing a B-rep body (Fusion behaviour; preference `FinishOutput`).
5. Foreign formats for FRM-10. **Resolved 2026-11-09 (owner, QUESTIONS #102):** OBJ control cages plus
   `.tsm`; `.tsm` is a stretch goal gated on owner-supplied samples (note §11).
   **Changed 2026-11-10 (owner):** no `.tsm` samples exist, so FRM-10 reads OBJ control cages
   only (option (a)). A `.tsm` reader becomes a new WP only if sample files turn up.
6. Packed conversion as the default once FRM-11 lands? Decide from FRM-11's fillet results;
   unpacked stays the default until then (Rhino's default is packed, Fusion's conversion gives
   one face per T-spline face region).

## Refinement log
### 2026-11-08 - S09-refine (lane C refinement)
- **Read:** AGENTS.md, vision, architecture, ADRs (ADR-009, ADR-013, ADR-014), roadmap,
  QUESTIONS, S01 (geometry stubs, `BodyGeom::Form`, persistent naming rules, direct mode, body
  codec), S02 (§4 builders, `geom` types, kernel trait), S07 (MESH-01 `HalfEdgeMesh`, MESH-03,
  MESH-15 falloff, MESH-16 feature pattern), S08 and its 2026-11-08 log (pattern for stub swap
  and oracle), S11 (module registry, contextual tabs, `Tool`, `BodyTessellator`,
  `EditAction`, UI-06), S12/S16 (body kinds at export), S20 (`amg-nurbs` surface API, KH-07/
  KH-12), `docs/notes/persistent-naming.md` (Form bodies body-level only). Code:
  `crates/amg-subd` (empty crate, deps core/jobs/brep/mesh; doc says lane A),
  `crates/amg-features/src/mesh` (serde structs, `register_feature!`, `OutputMode`,
  `input_mesh`/`emit`), `crates/amg-doc` (`stub::SubdMesh`, `BodyGeom::Form`,
  `stub:form` codec tag), `crates/amg-mesh/src/halfedge` (triangle-only directed edges).
- **Researched:** Fusion's Form environment tool set and timeline behaviour (Fusion help pages
  for Edit Form, primitives, utilities, Pull); OpenSubdiv `Sdc` options and crease rules
  (Chaikin, boundary interpolation), `opensubdiv-petite` (Apache-2.0 wrapper; 0.3.1 docs build
  fails); Rhino `ToNURBS` (packed/unpacked, extraordinary vertex options none/G1/G1x/G2, packing
  as post-process); Stam 1998; Nießner et al. 2012; Loop et al. 2009; Peters 2000; Halstead et
  al. 1993; Instant Meshes (BSD-3), QuadriFlow (MIT, LGPL Eigen part unless
  `BUILD_FREE_LICENSE`), `quadrs` (MIT). Details and sources in `docs/notes/subd-modeling.md` §0.
- **Decided:**
  - Own n-gon half-edge cage (amg-mesh's is triangle-only) with stable, never-reused ids.
  - Own evaluator; OpenSubdiv is a fixture oracle only (owner-confirmed, QUESTIONS #100).
  - One `form.create` node per form holding the authored cage (Fusion model), base64 `AMGS` in
    `design.json`; undo per edit through `EditFeature`.
  - `form.convert` (B-rep or mesh) instead of teaching solid features about Form bodies; Finish
    Form inserts it by default (owner-confirmed, QUESTIONS #101).
  - Form bodies stay body-level for references; the converted B-rep carries cage-id tags.
  - G1 at extraordinary vertices via Peters' construction with a double knot (Rhino "G1x"
    shape); G2 rejected (bulges).
  - Picking of cage elements is tool-internal; no S11 framework change.
  - Sharpness tags are f32 to match OpenSubdiv exactly (tags, not geometry).
- **WP changes** (IDs other specs reference: only the roadmap, by range FRM-01..06 and FRM-07,
  FRM-08; meanings kept):
  - FRM-01: Done-when made measurable; exact evaluation split out to FRM-15.
  - FRM-02: deps FRM-17 (dialogs live in the FORM tab); adds pipe and face.
  - FRM-03: deps FRM-15, FRM-17, UI-06 (was FRM-01, UI-06).
  - FRM-04: insert edge/point, subdivide, delete, extrude; bridge, fill hole, merge, weld moved
    to new FRM-18. Deps FRM-03.
  - FRM-05: deps FRM-03; "sharpen" is the semi-sharp crease value.
  - FRM-06: deps FRM-16, BREP-03, BREP-10, KH-07, DOC-06 (was FRM-01, BREP-10); packed output
    split to FRM-11.
  - FRM-07: now pull, interpolate, flatten (P3); thicken moved to FRM-12 (P2, it matters for
    printing open forms), match to FRM-13. Deps FRM-04, FRM-15.
  - FRM-08: deps FRM-06, FRM-07, MESH-06 (fit needs the §8 solve).
  - New: FRM-09 (sketch-driven creation, P3), FRM-10 (foreign T-spline/SubD import, ADR-009
    note, P3), FRM-11 (packed conversion), FRM-12 (thicken), FRM-13 (match, repair, P3),
    FRM-14 (benches and fuzz), FRM-15 (exact evaluation, stencils), FRM-16 (timeline
    integration, mesh conversion), FRM-17 (Form environment shell, display), FRM-18
    (topology tools II).
- **Roadmap:** Phase 5 C lists FRM-01..06, FRM-11, FRM-12, FRM-14..18; Phase 6 lists FRM-07,
  FRM-08, FRM-09, FRM-10, FRM-13. All deps are in Phase 5 or earlier (UI-05/UI-06/DOC-06/
  KH-12/BREP-10 Phase 2, KH-07/UI-20/RND-07/MESH-16 Phase 1, SK-19 and IO-05 earlier).
- **Questions for the owner:** QUESTIONS #100 (evaluator), #101 (Finish Form output), #102
  (FRM-10 formats).
- **Requests to other specs:** S01 (stub swap + codec, implemented by FRM-16), S16
  (`mesh_for_export` for Form bodies). Each is a short dated note in that spec.
- Status set to `Refined`.

### 2026-11-09 - owner answers moved in
- QUESTIONS #100 (own evaluator, OpenSubdiv oracle only), #101 (Finish Form adds/updates `form.convert` B-rep; FRM-17 Done-when 2 already matches), #102 (FRM-10 reads OBJ + `.tsm`). `.tsm` research: no public specification found; text-based; the only open implementation is LGPL-3.0, so FRM-10's `.tsm` part is a stretch goal that needs owner sample files and a clean-room reader (FRM-10 Done-when 4, note §11). Open questions 3-5 updated.

### 2026-11-10 - owner changes QUESTIONS #102 to (a)
- The owner has no `.tsm` files, so FRM-10 drops the `.tsm` reader and its stretch Done-when 4.
  FRM-10 reads OBJ control cages only. Note §11 keeps the `.tsm` research for a later WP.

## Interface changes
None yet (the spec has not reached `Ready`).
