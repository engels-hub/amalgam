# S08 — Implicit (SDF) modeling & volumetric lattices

| | |
|---|---|
| **Status** | Ready |
| **Priority** | P1 (lattice is a key AM differentiator); IMP-03 is P0 (S07 robust shell and wrap) |
| **Crates** | `amg-implicit`, `amg-features::implicit`, `amg-ui::implicit` (owned by lane C, QUESTIONS #61) |
| **Lane** | C |
| **Depends on** | S00 (FND-03 core types, FND-04 jobs, FND-09 `Gpu`), S07 (MESH-02/03/07/16/17/19: mesh types, `MeshSpatial`, `ScalarField`, checks), S15 (CMP-02 brick grids), S01 (DOC-05 features), S10 (RND-03 `ExternalLayer`), S11 (UI-05, UI-20), S02 (BREP-08 tessellation, for solid inputs), S13 (FEM-06, for IMP-07 only) |
| **Parity** | `03-fusion-parity.md` §Volumetric / implicit: volumetric lattice, variable shells / smooth booleans / offsets, automated modeling |

## Summary
nTop-style implicit modeling: bodies defined as signed distance fields built from an
expression graph (primitives, mesh/B-rep SDFs, booleans, smooth blends, offsets, shells,
lattices, field-driven variation), evaluated on the GPU, rendered directly (ray marching
preview) and meshed robustly for export. Also the backbone for robust shelling, "wrap" repair
and generative-design outputs.

The design note [`docs/notes/implicit-modeling.md`](../notes/implicit-modeling.md) has the
prior art, the formulas and the reasoning behind each decision below.

## Scope
**In:**
- `SdfGraph` (evaluated state), its CPU f64 evaluator, interval evaluator, binary encoding and
  hash.
- GPU sampling into a sparse brick grid with interval pruning.
- Mesh SDF, sampled-grid nodes, fields and ramps.
- Dual contouring mesher with a guaranteed-manifold fallback.
- TPMS, periodic strut, stochastic and (later) conformal lattices, with skin, open faces and
  solid regions; explicit beam graphs for 3MF beam lattices.
- Viewport ray-march preview through the S10 `ExternalLayer`.
- Timeline features in `amg-features::implicit` and the Lattice dialog / implicit display in
  `amg-ui::implicit`.

**Out:**
- MESH-17 marching tetrahedra and the S07 robust shell / wrap features (S07; they use IMP-03).
- 3MF beam-lattice reading and writing (S12 IO-18; it converts `BeamGraph`).
- Generative-design density → SDF (S14 GEN-07 composes IMP-04 and IMP-12).
- FEA result computation (S13). S08 consumes a resampled grid.
- A node-graph editor UI. Features build graphs; scripts (S18) can build any graph.

## Design
Summary of the design note; section numbers refer to it.

- **Authored vs evaluated (§1).** Timeline features are serde structs with `Expr` fields. In
  `compute()` they build an `SdfGraph`, which lives in `BodyGeom::Implicit(Arc<SdfGraph>)`
  (ADR-014). A graph holds evaluated f64 values only. Heavy payloads (`MeshSdf`, grids, beam
  graphs) are `Arc`-shared. The hash covers the canonical encoding and payload content hashes.
- **Own evaluator (§2).** fidget (MPL-2.0) is f32, closed-form only and 0.x. It cannot host mesh
  SDFs or sampled fields, and has no GPU path. It becomes a dev-dependency test oracle and
  benchmark behind `fidget-bench`.
- **Graph (§3).** An arena DAG of nodes: primitives (exact SDFs), rigid + uniform-scale transforms,
  booleans and smooth booleans, offset, shell, lattices, mesh SDF, sampled grid, fields. Each
  finished graph has finite bounds and a Lipschitz bound `L`. Lowered once to a register tape
  with "fat" ops; parameter values are in a separate buffer.
- **Evaluation (§4).** 64³ tiles → interval pruning → 8³ bricks → interval pruning → corner
  samples, stored as f16 `d / cell_size` in a CMP-02 brick grid. GPU: one WGSL interpreter
  shader, so no shader compile per graph. CPU fallback: same steps, f64, rayon.
- **Mesh SDF (§5).** Closest point within a band plus fast winding number sign (MESH-03).
  Clamped to `±band` outside, so it stays 1-Lipschitz. Baked to a sampled grid for the GPU.
  Solid bodies come in through the BREP-08 tessellation.
- **Meshing (§6).** Dual contouring on the sparse grid with one vertex per marching-cubes patch
  (DMC/MDC), QEF placement clamped to the cell, sharp features. If the manifold check ever
  fails, the extraction reruns with MESH-17 and the report says so. Self-intersections are
  relaxed once and the rest counted. Optional MESH-07 decimation.
- **Lattices (§7).** TPMS (gyroid, Schwarz P, diamond = Schwarz D, Neovius, Lidinoid) in sheet or
  network mode, with gradient-normalised distance. Periodic struts (cubic, BCC, FCC, octet,
  Kelvin, diamond, custom unit cells) as exact capsules / round cones. Explicit `BeamGraph`s for
  clipped, stochastic and conformal lattices and for 3MF. Skin, open faces and solid regions.
- **Preview (§8).** Ray marching of the brick grid into the S10 `ExternalLayer`, coarse grid
  after each edit and fine grid in the background.
- **Fields (§9).** `Scalar = Const | Field { field, ramp }` drives thickness and offsets. Fields:
  distance to a body or faces, `x, y, z` expressions, projected images, sampled grids (FEA,
  topology optimisation).
- **Cross-type booleans (§10).** Decided (owner, QUESTIONS #63): solid/mesh combined with implicit
  stays implicit (the other operand enters as a `MeshSdf`); meshed only on demand or at export.

### Fusion behaviour we match (Volumetric Lattice, Design Extension)
- One command on a body: cell type from a library, cell size (per axis), rotation, thickness
  (strut diameter or wall thickness), skin thickness on the outer faces, faces without skin, solid
  regions near chosen faces, variable thickness from fields.
- The body stays a lattice (here: an implicit body) in the browser and timeline. It is meshed for
  display and export.
- Beyond Fusion: TPMS sheet/network modes, stochastic and conformal lattices, field-driven
  thickness from expressions, images and FEA, smooth booleans, variable shells, offsets.

### UI (`amg-ui::implicit`)
- **Design workspace, Create panel:** Volumetric Lattice (IMP-15) and the implicit primitives
  (IMP-11). **Modify panel:** Implicit Combine, Shell, Offset, Convert to Mesh (IMP-08). All
  use the S11 generic feature dialog (UI-05) with live preview.
- Display: a `BodyTessellator` for `BodyGeom::Implicit` (IMP-13) and the ray-march layer (IMP-05).
  The ray march is the default while editing; the mesh is used when the layer is unavailable.

## Interfaces
Signatures other crates rely on (`amg-doc`, `amg-features`, `amg-ui`, `amg-io`, `amg-am`,
`amg-topopt`, `amg-script`). Types from `amg-core` (S00): `Point3 = DVec3`, `Vec3`,
`Transform` (`DAffine3`), `Aabb3`. From `amg-jobs`: `JobCtx`, `Gpu` (and `Cancelled`, re-exported from `amg-core`). From `amg-doc`: `BodyId`, `EntityRef`, `Expr`, `ExtKey`, `BlobRef`. From
`amg-mesh`: `IndexedMesh`, `MeshSpatial`, `FaceGroupId`, `ShellSide`, `BoolOp`,
`iso::ScalarField`.

```rust
// ───────────── amg_implicit (crate root) ─────────────
#[derive(Debug, thiserror::Error)]
pub enum ImplicitError {
    #[error("graph is unbounded; intersect lattices and half-spaces with a bounded shape")] Unbounded,
    #[error("non-uniform scale breaks the distance property")] NonUniformScale,
    #[error("invalid value for {field}: {msg}")] InvalidValue { field: &'static str, msg: String },
    #[error("graph needs more than {0} registers")] TapeTooLarge(u32),
    #[error("GPU: {0}")] Gpu(String),
    #[error("encoding: {0}")] Encoding(String),
    #[error(transparent)] Mesh(#[from] amg_mesh::MeshError),
    #[error(transparent)] Cancelled(#[from] amg_core::Cancelled),
}
pub type ImplicitResult<T> = Result<T, ImplicitError>;

/// Where heavy evaluation runs. `Auto` = GPU when a `Gpu` is given, else CPU.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Backend { #[default] Auto, Cpu, Gpu }

/// Closed interval, used by interval evaluation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Interval { pub lo: f64, pub hi: f64 }

// ───────────── amg_implicit::graph (IMP-01) ─────────────
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NodeId(pub u32);
/// blake3 of the canonical encoding (payloads by content hash).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub struct GraphHash(pub [u8; 32]);

/// Value that is constant or driven by a field node (IMP-16).
#[derive(Clone, Debug, PartialEq)]
pub enum Scalar { Const(f64), Field { field: NodeId, ramp: Ramp } }
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ramp { pub input: [f64; 2], pub output: [f64; 2], pub curve: RampCurve }   // output clamped
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RampCurve { Linear, Smoothstep }

#[derive(Clone, Debug, PartialEq)]
pub enum Primitive {
    Sphere { r: f64 },
    Box { half: DVec3, corner_r: f64 },
    Cylinder { r: f64, half_h: f64 },          // axis +Z, centred
    Capsule { a: Point3, b: Point3, r: f64 },
    Cone { r0: f64, r1: f64, h: f64 },         // base at z = 0, top at z = h
    Torus { major: f64, minor: f64 },          // in the XY plane
    HalfSpace { normal: Vec3, offset: f64 },   // d = n·p − offset; unbounded
}
/// Rigid motion plus uniform scale (anything else is refused).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Similarity { pub rigid: Transform, pub scale: f64 }

#[derive(Clone, Debug)]
pub enum Node {
    Prim(Primitive),
    Xform { child: NodeId, xf: Similarity },
    Bool { op: BoolOp, a: NodeId, b: NodeId, blend: f64 /* 0 = sharp; smooth-min radius, mm */ },
    Offset { child: NodeId, distance: Scalar },                      // d − r
    Shell { child: NodeId, thickness: Scalar, side: ShellSide },
    Lattice(LatticeNode),                                            // IMP-06, unbounded
    InBody { lattice: NodeId, body: NodeId, skin: Option<Skin>, solid_regions: Vec<SolidRegion> }, // IMP-06/15
    MeshSdf(Arc<crate::MeshSdf>),                                    // IMP-03 (CPU only)
    Grid(Arc<crate::SampledGrid>),                                   // IMP-12
    Field(FieldNode),                                                // IMP-16; not a distance
}

pub struct GraphBuilder { /* arena */ }
impl GraphBuilder {
    pub fn new() -> Self;
    pub fn add(&mut self, node: Node) -> ImplicitResult<NodeId>;     // children must already exist
    /// Imports another graph's nodes (payloads stay shared); returns its root here.
    pub fn import(&mut self, g: &SdfGraph) -> NodeId;
    pub fn finish(self, root: NodeId) -> ImplicitResult<SdfGraph>;   // bounds, Lipschitz, hash, tape
}

/// Evaluated implicit body: `BodyGeom::Implicit(Arc<SdfGraph>)`. Immutable, `Send + Sync`.
#[derive(Clone, Debug)]
pub struct SdfGraph { /* nodes, root, bounds, lipschitz, hash, OnceLock<Tape> */ }
impl SdfGraph {
    pub fn root(&self) -> NodeId;
    pub fn nodes(&self) -> &[Node];
    pub fn bounds(&self) -> Aabb3;                 // conservative, contains d ≤ 0
    pub fn lipschitz(&self) -> f64;                // upper bound, ≥ 1
    pub fn hash(&self) -> GraphHash;
    pub fn eval(&self, p: Point3) -> f64;          // CPU f64 reference
    pub fn eval_grad(&self, p: Point3) -> (f64, Vec3);
    pub fn eval_batch(&self, ps: &[Point3], out: &mut [f64]);   // rayon inside
    pub fn eval_interval(&self, b: &Aabb3) -> Interval;          // conservative
    pub fn write_binary(&self, w: &mut dyn std::io::Write) -> ImplicitResult<()>;  // versioned "AMGI", checksummed
    pub fn read_binary(bytes: &[u8]) -> ImplicitResult<SdfGraph>;
}
impl amg_mesh::iso::ScalarField for SdfGraph { /* value = eval, may_cross via eval_interval */ }

// ───────────── amg_implicit::sdf (IMP-03, IMP-12) ─────────────
/// Exact narrow-band signed distance to a mesh (negative inside, winding number > 0.5).
pub struct MeshSdf { /* Arc<MeshSpatial>, band */ }
impl MeshSdf {
    pub fn new(spatial: Arc<amg_mesh::MeshSpatial>, band: f64) -> Self;   // as expected by S07
    pub fn band(&self) -> f64;
    pub fn signed(&self, p: Point3) -> f64;        // clamped to ±band outside the band
    pub fn unsigned(&self, p: Point3) -> f64;      // clamped to band
    pub fn closest_group(&self, p: Point3) -> Option<amg_mesh::FaceGroupId>;   // None beyond band
    pub fn content_hash(&self) -> u64;             // mesh content hash + band
    // IMP-03 additions:
    pub fn spatial(&self) -> &Arc<amg_mesh::MeshSpatial>;
    pub fn is_closed(&self) -> bool;               // boundary chain empty; only then may_cross prunes
    pub fn is_inside(&self, p: Point3) -> bool;    // winding > 0.5 (β = 1 estimate, β = 2 if ambiguous)
}
impl amg_mesh::iso::ScalarField for MeshSdf { /* values: parallel, Morton order; may_cross: distance + 1-Lipschitz bound on closed meshes */ }

/// Narrow-band sparse grid of f16 samples (`d / cell_size`), trilinear. CMP-02 storage.
pub struct SampledGrid { /* amg_compute brick grid (CMP-02), origin, cell_size, band, kind */ }
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GridKind { Distance /* clamped to ±band */, Scalar /* fields: FEA, density, images */ }
impl SampledGrid {
    pub fn bake(f: &dyn amg_mesh::iso::ScalarField, bounds: Aabb3, cell_size: f64, band: f64,
                kind: GridKind, ctx: &JobCtx) -> ImplicitResult<SampledGrid>;   // CPU, rayon
    /// Producer path for S13/S14: values at the cell corners of a dense box, x fastest.
    pub fn from_dense(origin: Point3, cell_size: f64, dims: [u32; 3], values: &[f32]) -> ImplicitResult<SampledGrid>;
    pub fn value(&self, p: Point3) -> f64;
    pub fn interval(&self, b: &Aabb3) -> Interval;
    pub fn cell_size(&self) -> f64;
    pub fn content_hash(&self) -> u64;
}

// ───────────── amg_implicit::eval (IMP-02) ─────────────
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SampleRequest { pub bounds: Aabb3, pub cell_size: f64, pub band: f64 /* keep bricks that may reach |d| < band */ }
/// Result of sampling: 8³-cell bricks of corner values plus tile/brick inside-outside flags.
pub struct SdfBricks { /* CMP-02 brick grid, origin, cell_size, per-tile classification */ }
impl SdfBricks {
    pub fn cell_size(&self) -> f64;
    pub fn bounds(&self) -> Aabb3;
    pub fn brick_count(&self) -> usize;
    pub fn sample(&self, ijk: [i32; 3]) -> Option<f32>;   // None = pruned (sign known from classification)
    pub fn is_inside(&self, ijk: [i32; 3]) -> bool;
}
/// Shared GPU state: pipelines are created once; graphs only change buffers.
pub struct GpuEvaluator { /* pipelines, buffer pools */ }
impl GpuEvaluator {
    pub fn new(gpu: &Gpu) -> ImplicitResult<Self>;
    pub fn pipelines_created(&self) -> u32;               // test hook: parameter edits create none
}
pub fn sample(g: &SdfGraph, req: &SampleRequest, backend: Backend, gpu: Option<&GpuEvaluator>,
              ctx: &JobCtx) -> ImplicitResult<SdfBricks>;

// ───────────── amg_implicit::meshing (IMP-04) ─────────────
#[derive(Clone, Debug, PartialEq)]
pub struct MeshOpts {
    pub cell_size: f64,
    pub bounds: Option<Aabb3>,                 // default: graph bounds + 2 cells
    pub sharp_features: bool,                  // QEF placement (default true)
    pub decimate_tolerance: Option<f64>,       // MESH-07 afterwards; default Some(0.5 × cell_size)
    pub backend: Backend,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MeshFallback { NonManifold { edges: usize }, NotClosed { boundary_edges: usize } }
#[derive(Clone, Debug, PartialEq)]
pub struct MeshReport { pub bricks: usize, pub triangles: usize, pub fallback: Option<MeshFallback>,
                        pub self_intersections: usize, pub seconds: f64 }
/// Always closed, oriented (outward) and manifold; possibly empty or multi-shell.
pub fn to_mesh(g: &SdfGraph, opts: &MeshOpts, gpu: Option<&GpuEvaluator>, ctx: &JobCtx)
    -> ImplicitResult<(IndexedMesh, MeshReport)>;

// ───────────── amg_implicit::lattice (IMP-06, IMP-09, IMP-14) ─────────────
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum TpmsKind { Gyroid, SchwarzP, Diamond, Neovius, Lidinoid }
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum TpmsMode { Sheet, Network, NetworkInverse }
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum StrutCell { Cubic, Bcc, Fcc, Octet, Kelvin, Diamond }
/// Custom unit cell: nodes in [0, 1]³, beams as node index pairs.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct UnitCell { pub nodes: Vec<Point3>, pub beams: Vec<[u32; 2]> }
impl UnitCell { pub fn builtin(c: StrutCell) -> UnitCell; }

#[derive(Clone, Debug)]
pub enum LatticeKind { Tpms { kind: TpmsKind, mode: TpmsMode }, Strut(Arc<UnitCell>), Beams(Arc<BeamGraph>) }
#[derive(Clone, Debug)]
pub struct LatticeNode {
    pub kind: LatticeKind,
    pub cell: Vec3,                 // cell size per axis, mm (ignored for Beams)
    pub frame: Transform,           // rigid; lattice frame → world
    pub thickness: Scalar,          // wall thickness (TPMS) or strut diameter (struts)
    pub map: Option<Arc<dyn UvwMap>>, // IMP-09 conformal; None = periodic in `frame`
}
#[derive(Clone, Debug, PartialEq)]
pub struct Skin { pub thickness: Scalar, pub open_groups: Vec<amg_mesh::FaceGroupId> }
#[derive(Clone, Debug, PartialEq)]
pub struct SolidRegion { pub groups: Vec<amg_mesh::FaceGroupId>, pub distance: f64 }

/// Explicit strut lattice; maps 1:1 to a 3MF beam lattice (IO-18).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BeamGraph { pub nodes: Vec<Point3>, pub beams: Vec<Beam>, pub ball_radius: Option<f64> }
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Beam { pub a: u32, pub b: u32, pub r_a: f64, pub r_b: f64 }   // sphere caps
impl BeamGraph {
    /// Instantiates a periodic strut lattice inside `region` (d ≤ 0), clipping crossing beams.
    pub fn from_periodic(lattice: &LatticeNode, region: &dyn amg_mesh::iso::ScalarField,
                         bounds: Aabb3, ctx: &JobCtx) -> ImplicitResult<BeamGraph>;
    /// IMP-14: jittered-grid Voronoi foam (deterministic for a seed).
    pub fn stochastic(region: &dyn amg_mesh::iso::ScalarField, bounds: Aabb3, mean_cell: f64,
                      radius: f64, seed: u64, ctx: &JobCtx) -> ImplicitResult<BeamGraph>;
    pub fn content_hash(&self) -> u64;
}
/// IMP-09: world point → lattice coordinates (cell units). Must be Lipschitz with bound `lipschitz()`.
pub trait UvwMap: Send + Sync + std::fmt::Debug {
    fn map(&self, p: Point3) -> Vec3;
    fn lipschitz(&self) -> f64;
    fn content_hash(&self) -> u64;
}

// ───────────── amg_implicit::field (IMP-16) ─────────────
#[derive(Clone, Debug)]
pub enum FieldNode {
    Distance { sdf: NodeId },                         // |d| of an SDF node
    Expr(Arc<FieldExpr>),                             // x, y, z in mm, world frame
    Image { image: Arc<GrayImage>, plane_to_world: Transform, size: [f64; 2] },  // projected along local Z
    Grid(Arc<SampledGrid>),                           // GridKind::Scalar (FEA, density)
}
/// Parsed point expression (`+ - * / ^`, sin cos tan sqrt abs min max clamp exp, x y z, numbers).
#[derive(Clone, Debug, PartialEq)]
pub struct FieldExpr { /* postfix program */ }
impl FieldExpr { pub fn parse(src: &str) -> ImplicitResult<FieldExpr>; pub fn eval(&self, p: Point3) -> f64; }
#[derive(Clone, Debug, PartialEq)]
pub struct GrayImage { pub width: u32, pub height: u32, pub pixels: Vec<f32> /* 0..1, row-major */ }

// ───────────── amg_implicit::preview (IMP-05) ─────────────
/// Must match `amg-render/shaders/camera.wgsl` (viewport-rendering note §14). Copied, not shared.
#[repr(C)] #[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniform { pub view_rte: [[f32; 4]; 4], pub proj: [[f32; 4]; 4], pub inv_view_proj_rte: [[f32; 4]; 4],
                           pub eye_hi: [f32; 3], pub _p0: f32, pub eye_lo: [f32; 3], pub _p1: f32,
                           pub size_px: [f32; 2], pub pixel_size_at_1mm: f32, pub ortho: u32 }
pub struct RayMarcher { /* pipelines */ }
pub struct PreviewVolume { /* GPU copy of SdfBricks + occupancy + colour */ }
impl RayMarcher {
    pub fn new(gpu: &Gpu) -> ImplicitResult<Self>;
    pub fn upload(&self, gpu: &Gpu, bricks: &SdfBricks, world_from_volume: Transform, color: [f32; 4]) -> PreviewVolume;
    /// Writes Rgba16Float premultiplied colour and R32Float reversed-Z depth (0 = empty).
    pub fn encode(&self, enc: &mut wgpu::CommandEncoder, cam: &CameraUniform, volumes: &[&PreviewVolume],
                  color: &wgpu::TextureView, depth: &wgpu::TextureView, size_px: [u32; 2]);
}

// ───────────── amg_features::implicit (IMP-08, IMP-11, IMP-13, IMP-15) ─────────────
// Registered with `register_feature!`; numeric fields are `Expr`; `OutputMode` is the one from
// `amg_features::mesh`. Kind strings: "implicit.primitive", "implicit.from_body", "implicit.lattice",
// "implicit.combine", "implicit.shell", "implicit.offset", "implicit.to_mesh".
// `*Params` types are plain serde structs/enums of `Expr`s mirroring the amg_implicit types
// (e.g. `RampParams { input: [Expr; 2], output: [Expr; 2], curve: RampCurve }`); IMP-11 fixes them.
pub struct ImplicitPrimitiveFeature { pub shape: PrimitiveParams, pub placement: PlacementParams }
pub struct ImplicitFromBodyFeature  { pub body: BodyId, pub band: Option<Expr>, pub output: OutputMode }
pub struct LatticeFeature {
    pub body: BodyId,                                   // solid, mesh or implicit body
    pub cell: LatticeCellParam,                         // Tpms { kind, mode } | Strut(StrutCell) | Custom(UnitCell) | Stochastic { seed }
    pub cell_size: [Expr; 3],
    pub rotation: [Expr; 3],                            // XYZ angles of the lattice frame
    pub thickness: ThicknessParam,                      // Uniform(Expr) | Graded { field: FieldParam, ramp: RampParams }
    pub skin: Option<SkinParams>,                       // { thickness: Expr, open_faces: Vec<EntityRef> }
    pub solid_regions: Vec<SolidRegionParams>,          // { faces: Vec<EntityRef>, distance: Expr }
    pub min_feature: Expr,                              // warning threshold, default 0.4 mm
    pub output: OutputMode,
}
pub enum FieldParam { DistanceToBody(BodyId), DistanceToFaces(Vec<EntityRef>), Expression(String),
                      Image { blob: BlobRef, plane: PlaneParams, size: [Expr; 2] },
                      StudyResult { study: ExtKey, quantity: String } /* IMP-07 */ }
pub struct ImplicitCombineFeature { pub target: BodyId, pub tools: Vec<BodyId>, pub op: BoolOp, pub blend: Expr, pub keep_tools: bool }
pub struct ImplicitShellFeature   { pub body: BodyId, pub thickness: ThicknessParam, pub side: ShellSide, pub output: OutputMode }
pub struct ImplicitOffsetFeature  { pub body: BodyId, pub distance: ThicknessParam, pub output: OutputMode }
pub struct ImplicitToMeshFeature  { pub body: BodyId, pub cell_size: Expr, pub sharp_features: bool, pub output: OutputMode }

/// IMP-13: the mesh other crates use for an implicit body (export, AM, mesh-only features).
/// Cached by (graph hash, cell size); `ExportQuality` maps to a cell size (Draft 0.4, Normal 0.2, Fine 0.1 mm).
pub fn mesh_for_export(g: &Arc<SdfGraph>, q: ExportQuality, ctx: &JobCtx) -> Result<Arc<IndexedMesh>, ImplicitFeatureError>;
#[derive(Clone, Copy, Debug, PartialEq, Eq)] pub enum ExportQuality { Draft, Normal, Fine }

#[derive(Debug, thiserror::Error)]
pub enum ImplicitFeatureError {
    #[error(transparent)] Implicit(#[from] amg_implicit::ImplicitError),
    #[error("input body has no geometry this feature can read")] UnsupportedInput,
    #[error("solid input needs tessellation (BREP-08): {0}")] Tessellation(String),
    #[error("referenced face no longer resolves")] UnresolvedFace,
}
```

**Expected from other specs** (owned there; listed so the dependency is explicit):
- **S07:** `amg_mesh::iso::ScalarField`, `extract_isosurface` (MESH-17, Done), `MeshSpatial`
  (MESH-03, Done), `analyze` (MESH-02), `self_intersections` (MESH-19), `decimate` (MESH-07),
  `wall_thickness` (MESH-11, tests only). No change requested: `MeshSdf::new(spatial, band)`,
  `signed`, `unsigned` match S07's expected signatures.
- **S15 (CMP-02):** a sparse brick grid with a CPU and a GPU mirror: `BrickGrid<T>` with brick edge
  8, f16 (`half::f16`) storage, insert/lookup by brick coordinate, and a GPU buffer layout
  (brick table + hash or dense index) that WGSL can read. Requested in S15 open questions.
- **S01 / lane E:** `amg-doc::stub::SdfGraph` becomes `pub use amg_implicit::SdfGraph`; the body
  codec stores implicit bodies with `SdfGraph::write_binary` (IMP-11, as MESH-16 did for meshes).
  Recorded in S01 Interface changes.
- **S10:** `ExternalLayer` and the `Camera` WGSL layout (RND-03, viewport-rendering note §14). No
  change.
- **S11:** `BodyTessellator` and the generic feature dialog (UI-20, UI-05). No change.
- **S12 (IO-18):** converts `BeamGraph` ↔ 3MF beam lattice (sphere caps; balls when
  `ball_radius` is set). Recorded in S12 Interface changes.
- **S13 (FEM-06), S14 (GEN-07):** hand result fields to S08 as `SampledGrid::from_dense`
  (resampled on a regular grid). Requested in S13 and S14 open questions.
- **S16:** implicit build items use `mesh_for_export` (IMP-13). Recorded in S16 Interface changes.

## Work packages
WPs are ≈ 1–3 days each. IMP-01..10 keep their IDs (some re-scoped, see Refinement log);
IMP-11..16 are new. Reference cases R1–R3 are defined under Performance targets. Times are on
the reference machine (vision doc), release build. Golden tests live in `tests/golden/implicit/`.

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| IMP-03 | Mesh SDF: `MeshSdf` (narrow-band closest point + fast-winding-number sign, clamped to ±band), `closest_group`, `ScalarField` impl | MESH-03 | P0 | Done | On 10 k random points within the band of corpus watertight models, the absolute signed distance equals brute-force closest distance within 1e-9 mm, and the sign equals the exact winding number's for points ≥ 1 mean edge length off the surface. A property test shows the field is 1-Lipschitz (1e-9 slack) and `may_cross` never returns false for a box where 64³ dense sampling finds a crossing. 2 M signed queries on a 1 M-tri mesh (band 2 mm) < 2 s. Results identical across 1/4/16 threads. |
| IMP-01 | `SdfGraph`: nodes (primitives, similarity transforms, booleans + smooth booleans, offset, shell), `GraphBuilder`, bounds + Lipschitz bound, tape lowering, CPU f64 point/gradient/interval evaluators, `AMGI` binary encoding, blake3 hash, `ScalarField` impl; fidget oracle behind `fidget-bench` | FND-03, MESH-17 | P1 | Todo | Primitives match their closed forms within 1e-12 on 10 k points; smooth booleans with blend 0 equal min/max exactly. Interval property test (1 000 random graphs × 100 boxes): every sampled value lies in the box's interval. Finite-difference test: no graph exceeds its Lipschitz bound. Binary round trip is bit-exact and the hash is equal across runs and thread counts. `finish` rejects unbounded graphs and non-uniform scale. With `fidget-bench`, values match fidget within 1e-5 relative on random closed-form graphs. 1 M point evaluations of R1 < 200 ms on 8+ cores. |
| IMP-11 | Implicit bodies in the timeline: swap `amg-doc` `SdfGraph` stub, body codec arm, `amg-features::implicit` base (input lookup, emit, `ImplicitFeatureError`), `ImplicitPrimitiveFeature` (`implicit.primitive`) | IMP-01, DOC-05, MESH-16 | P1 | Todo | A primitive feature is undoable (undo/redo restores the exact graph hash), parametric (editing a value recomputes it and its dependents only), hits the cache on unchanged inputs and round-trips through serialization. An implicit body survives `.amg` save/load with an equal graph hash. `cargo nextest run -p amg-doc` still passes. |
| IMP-02 | GPU evaluator: WGSL tape interpreter, tile/brick interval pruning, f16 brick output (`SdfBricks`), `sample()` with CPU fallback | IMP-01, CMP-02, FND-09 | P1 | Todo | GPU vs CPU: on 200 random graphs × 10 k points, the difference is ≤ 1e-4 mm plus f16 quantisation. Pruning is conservative: against a dense 128³ CPU reference, no cell with a sign change within the band is pruned. R1 at 512³ effective resolution (band 2 cells) samples in < 1 s on the GPU and < 10 s on the CPU fallback. A parameter-only edit creates no new pipeline (`pipelines_created` unchanged). Tests run on WARP in CI. |
| IMP-12 | Sampled grids: `SampledGrid` (bake, `from_dense`, trilinear value + interval), `Grid` node on CPU and GPU, baked `MeshSdf` for GPU use | IMP-02, IMP-03 | P1 | Todo | A baked corpus mesh SDF at cell h differs from the exact `MeshSdf` by ≤ 0.5 h inside the band. Grid interval evaluation is conservative (property test). Baking a 1 M-tri, 100 mm part at 0.25 mm, band 1 mm < 5 s. A graph with a `Grid` node gives the same `sample()` result on GPU and CPU within the IMP-02 tolerance. |
| IMP-04 | Dual contouring mesher `to_mesh`: one vertex per MC patch, QEF with mass-point bias clamped to cell, edge refinement on the f64 tape, MESH-02 check with MESH-17 fallback, SI relaxation + count (MESH-19), optional MESH-07 decimation | IMP-02, MESH-02, MESH-07, MESH-17, MESH-19 | P1 | Todo | Fuzz over 1 000 random CSG graphs (including surfaces through grid points and touching the bounds): output always closed, oriented and manifold; MESH-17 fallback used in ≤ 1 % of cases. Primitive corpus has 0 self-intersections. Sphere and torus: Hausdorff to analytic ≤ 0.1 cell. 20 mm box at 0.5 mm cells: edges and corners within 0.01 mm. Box via `MeshSdf` (from a tessellated cube): corners within 0.02 mm. Golden tests (volume, area, bbox, Hausdorff). Output identical across thread counts. |
| IMP-05 | Viewport ray-march preview: `RayMarcher` over `SdfBricks` (brick DDA, sphere tracing, bisection, grid normals, S10 lights), `ExternalLayer` output, coarse/fine rebuild; wiring in `amg-ui::implicit` | IMP-02, RND-03, UI-20 | P1 | Todo | R2 at 1920×1080: ray-march pass ≤ 8 ms GPU time on the reference machine. Coarse rebuild after a parameter edit < 300 ms. A WARP golden image (S10 §15 thresholds) of R2 passes. Silhouettes of the ray-march and of the IMP-04 mesh drawn by S10 differ in ≤ 0.5 % of pixels. Never blocks the UI thread (rebuilds are jobs). |
| IMP-06 | Lattices: TPMS (5 kinds × sheet/network), periodic struts (6 built-in cells + custom `UnitCell`), round-cone beams, `BeamGraph` + `from_periodic` clipping, `InBody` node (lattice ∩ body + skin + solid regions, open faces) | IMP-02, IMP-04 | P1 | Todo | Network mode at `c = 0` for gyroid, Schwarz P and diamond gives volume fraction 0.5 ± 0.005 (1e6 Monte Carlo samples). Sheet wall thickness measured on the mesh (MESH-11) within ±5 % of target or ±1 cell. Strut lattice volume matches Monte Carlo of the explicit capsule union within 1 %. `from_periodic` on a cell-aligned box gives the analytic beam count, and every kept beam end lies in the region within 1e-4 mm. R2 full pipeline (sample + mesh at 0.2 mm) < 30 s; output watertight and manifold. Golden tests for each cell type. |
| IMP-16 | Fields and grading: `FieldNode` (distance, `FieldExpr`, projected image, grid), `Ramp`, field-driven `Scalar` in offset/shell/lattice thickness, Lipschitz bounds for fields | IMP-06, IMP-12 | P1 | Todo | A gyroid with thickness ramped linearly in z (0.5 → 1.5 mm over 40 mm): measured wall thickness per 5 mm slab within ±5 % of the ramp. Graded strut radii equal the ramp at the beam end nodes exactly. `FieldExpr` table tests (≥ 60 cases, including errors with spans). Image field equals pixel values at pixel centres. Finite-difference test: no field-driven graph exceeds its Lipschitz bound. |
| IMP-13 | Implicit display and export plumbing: `BodyTessellator` for `BodyGeom::Implicit` in `amg-ui::implicit`, `mesh_for_export` with a (graph hash, cell size) cache | IMP-04, IMP-11, UI-20 | P1 | Todo | Implicit bodies show in the viewport at Preview/Normal/Fine quality without blocking the UI (latest-wins). A second `mesh_for_export` call for the same graph and quality returns the cached `Arc` in < 1 ms. An implicit body exported through IO-11 gives a watertight 3MF that the IO-02 reader loads back with the same triangle count. |
| IMP-08 | Implicit modeling features: `implicit.from_body`, `implicit.combine` (smooth booleans), `implicit.shell` (variable thickness), `implicit.offset`, `implicit.to_mesh`; their generic dialogs | IMP-11, IMP-12, IMP-13, IMP-16, UI-05 | P1 | Todo | Each is a registered feature: undoable, parametric, serialises round-trip, reports health. Smooth union of two r = 10 mm spheres (blend 2 mm) matches a Monte Carlo volume of the analytic field within 0.5 %. A field-driven shell has measured thickness within ±1 cell of the field. `implicit.to_mesh` gives a watertight `BodyGeom::Mesh`. Golden tests for each. Cross-type inputs (QUESTIONS #63, decided): a solid or mesh combined with an implicit body gives an implicit result (the other operand enters as a `MeshSdf`); test: lattice minus a solid cylinder stays `BodyGeom::Implicit`, no mesh is built until `implicit.to_mesh` or export, and the result volume matches a Monte Carlo reference within 1 %. |
| IMP-15 | Volumetric Lattice feature (`implicit.lattice`) on solid/mesh/implicit bodies: cell library, cell size, rotation, uniform/graded thickness, skin with open faces, solid regions, min-feature warning; Lattice dialog in `amg-ui::implicit` with ray-march preview | IMP-06, IMP-16, IMP-08, IMP-05, BREP-08, UI-05 | P1 | Todo | Lattice on a corpus STEP bracket (via BREP-08) and on a corpus mesh: undoable, parametric (cell-size edit recomputes only the lattice and dependents), serialises round-trip. No skin within 1 cell of an open face; solid within `distance` of a solid-region face. A thickness below `min_feature` gives a warning. R3 completes (bake + lattice + mesh) < 45 s, watertight. Golden tests `lattice_*`. |
| IMP-07 | FEA-driven variation: `FieldParam::StudyResult` (von Mises, displacement, safety factor) via `SampledGrid::from_dense`, out-of-date when the study changes | IMP-16, IMP-15, FEM-06 | P2 | Todo | On the S13 plate-with-hole study, strut radius follows the von Mises ramp: in the top-decile stress region the mean measured radius is ≥ 90 % of the ramp maximum, in the bottom decile ≤ 110 % of the ramp minimum. Re-solving the study marks the lattice out of date, and recompute picks up the new field. |
| IMP-14 | Stochastic (Voronoi foam) lattice: jittered-grid seeds, own Bowyer–Watson 3D Delaunay (`robust` predicates), Voronoi edges → `BeamGraph::stochastic`, `LatticeCellParam::Stochastic` | IMP-06, IMP-15 | P2 | Todo | Delaunay of 10 k jittered seeds passes the empty-circumsphere check with exact predicates. Same seed gives a bit-identical beam graph. Mean beam length within ±15 % of the target cell size. A 100 mm part at 5 mm mean cell builds its beam graph in < 5 s. Meshed result watertight and manifold. |
| IMP-09 | Conformal lattices: `UvwMap` (cylindrical, spherical, sweep along a curve), strut instancing in UVW → `BeamGraph`, TPMS through the map | IMP-06, IMP-15 | P2 | Todo | A cylindrical strut lattice in a tube (r 20–30 mm, 100 mm long, 24 cells around) closes across the seam (shared nodes within 1e-6 mm, no duplicate beams). Measured cell size within ±10 % of target across the wall. A conformal TPMS in the tube is watertight and within the IMP-06 thickness tolerance. Beam graph exports through IO-18 when present. |
| IMP-10 | Automated modeling: connect selected faces with generated shapes that avoid obstacle bodies (approach per QUESTIONS #62: implicit tube heuristic first, small topology-optimisation mode later via S14) | IMP-08 | P2 | Todo | On 3 reference cases (2–4 connect faces, 1–2 obstacles): the result touches every connect face (gap ≤ 0.01 mm), keeps the clearance to obstacles (≥ set value − 1 cell), is watertight, and finishes in < 10 s. Approach (owner, QUESTIONS #62): the implicit heuristic (smooth-union tubes between connect faces, obstacles subtracted with a clearance offset, smoothed) ships first; a small topology-optimisation mode (S14 GEN, connect faces as preserves, obstacles as keep-outs, minutes per run) is added later as an option, as a new WP once S13/S14 exist. |

## Testing
- **Analytic:** primitive SDFs, booleans, smooth booleans, TPMS formulas and their volume fractions
  by symmetry, strut volume vs Monte Carlo.
- **Property tests (`proptest`):** interval conservativeness, Lipschitz bounds, `may_cross`,
  encode/decode round trip, hash stability.
- **Fuzz:** random CSG graphs through `to_mesh` (always closed, oriented, manifold; fallback rate
  reported).
- **Differential:** GPU vs CPU sampling, fidget vs CPU evaluator (`fidget-bench`), `MeshSdf` vs
  brute force, IMP-04 vs MESH-17 (Hausdorff).
- **Golden geometry (`tests/golden/implicit/`):** volume, area, bbox, Hausdorff to analytic or
  reference meshes for each primitive, each lattice cell type, R2 and R3.
- **Render goldens:** ray-march images on WARP (S10 §15 thresholds).
- **Benchmarks (`criterion`):** R1–R3 at 3 resolutions (0.4, 0.2, 0.1 mm) on CPU and GPU.

## Performance targets
Reference cases (generators and data in `amg-implicit` tests and `testdata/`):
- **R1 "csg-100":** seeded random CSG of 100 primitive nodes (sharp and smooth booleans) in a
  100 mm box.
- **R2 "lattice-bracket":** rounded box 100 × 60 × 40 mm (corner radius 5 mm), gyroid sheet, 8 mm
  cell, 0.8 mm wall, 1.5 mm skin, meshed at 0.2 mm.
- **R3 "mesh-lattice":** a ~1 M-tri watertight corpus model scaled to 100 mm, octet struts,
  6 mm cell, 1.0 mm diameter, 1.0 mm skin, baked SDF at 0.25 mm, meshed at 0.2 mm.

| Scenario | Budget | Ties to |
|---|---|---|
| R1 sample, 512³ effective, GPU / CPU | < 1 s / < 10 s | original IMP-02 target |
| R2 full pipeline to watertight mesh | < 30 s | original IMP-06 target ("lattice of 100 mm part") |
| R3 full pipeline from mesh body | < 45 s | — |
| Ray-march pass, R2, 1080p | ≤ 8 ms GPU (60 fps with the mesh scene) | vision: interactive viewport |
| Coarse preview rebuild after an edit | < 300 ms | vision: recompute < 500 ms |
| `MeshSdf`, 2 M queries, 1 M tris | < 2 s | S07 MESH-10 robust shell < 20 s |
| UI thread | never > 16 ms | vision; all of the above are jobs with progress + cancel |

## Open questions
1. ~~Adopt `fidget` vs own evaluator.~~ **Resolved 2026-11-08:** own evaluator; fidget is a
   dev-dependency oracle and benchmark (`fidget-bench`). Reasons in the design note §2.
2. ~~(from S10, 2026-10-20) IMP-05 writes an `ExternalLayer` and copies the `Camera` WGSL
   layout.~~ **Resolved 2026-11-08:** accepted as is; `CameraUniform` in Interfaces copies it,
   and IMP-05's WARP golden catches drift.
3. ~~(from S12, 2026-10-20) Beam/ball representation for IO-18.~~ **Resolved 2026-11-08:**
   `BeamGraph` (nodes, tapered beams `r_a`/`r_b` with sphere caps, optional uniform ball radius)
   maps 1:1 to 3MF beams and balls. Recorded in S12 Interface changes.
4. Does lane C get an `amg-ui::implicit` module? (QUESTIONS #61.) **Resolved 2026-11-09 (owner):** yes, lane C owns `amg-ui::implicit`.
5. IMP-10 approach: implicit heuristic or topology optimisation? (QUESTIONS #62.) **Resolved 2026-11-09 (owner):** implicit heuristic first, small topology-optimisation mode later.
6. Cross-type booleans with implicit bodies stay implicit instead of meshing? (QUESTIONS #63.) **Resolved 2026-11-09 (owner):** yes, stay implicit.
7. CMP-02 must provide the brick grid shape described under Interfaces (S15 is Draft). If S15
   refinement chooses a different layout, IMP-02 adapts; the `SdfBricks` API does not change.
8. (from S15, 2026-11-08) CMP-02 shape is fixed in S15 Interfaces: `amg_compute::grid::BrickGrid<f16>` with `BrickLayout::CORNERS` (9³ samples per 8³-cell brick), `insert`/`lookup` by `BrickCoord`, fills `Fill0` = outside and `Fill1` = inside (per brick or per 64³-cell tile via `set_tile_fill`, matching IMP-02's tiles), and `GpuBrickGrid` with a two-level dense index (no hash), deterministic `allocate` from a brick mask, and WGSL from `brick_grid_wgsl`. Values are in index space; `SdfBricks`/`SampledGrid` keep origin and cell size. `amg-implicit` needs `amg-compute` in its Cargo.toml.

## Refinement log
### 2026-11-08 - S08-refine (lane C refinement)
- **Read:** AGENTS.md, vision, architecture, ADRs, roadmap, QUESTIONS, S01, S07, S10, S11,
  S12, S13, S14, S15, S16, the viewport-rendering note §14, the mesh-processing note, and the code
  in `crates/amg-implicit` (empty crate: Cargo.toml depends on core, jobs, mesh),
  `crates/amg-features/src/mesh` (feature pattern: serde structs, `register_feature!`,
  `OutputMode`, `input_mesh`/`emit`), `crates/amg-doc` (`BodyGeom::Implicit(Arc<SdfGraph>)`
  stub, `StubBodyCodec` stores implicit as a tag), `crates/amg-mesh/src/iso` (`ScalarField`,
  `IsoOpts`, `extract_isosurface`) and `crates/amg-mesh/src/spatial` (`closest_point`,
  `winding_number`, `signed_distance`), and the layering test (implicit may use mesh and the
  foundation layer, including `amg-compute`).
- **Researched:** Fusion's Volumetric Lattice (Autodesk forum preview thread; AU 2022 class
  CP502032): cell library, cell size, thickness/solidity, skin, offset regions, gradients, mesh for
  export. fidget (MPL-2.0, 0.4.x): JIT, interval tape simplification, own MDC mesher, f32, Windows
  executable-memory path in `fidget-jit`. libfive, Keeter MPR 2020, DC / MDC / DMC, sphere
  tracing, fast winding numbers, TPMS level-set formulas, 3MF beam lattice. Details and sources in
  `docs/notes/implicit-modeling.md`.
- **Decided:**
  - Own evaluator; fidget as oracle (open question 1).
  - `SdfGraph` is evaluated state with f64 values and `Arc` payloads; features stay serde + `Expr`.
  - GPU path is one WGSL interpreter (no per-graph shader compile; parameter edits only touch
    buffers), following MPR.
  - Mesher: DMC/MDC-style dual contouring with a MESH-17 fallback, so "always manifold" is a
    guarantee. S07's robust shell and wrap keep MESH-17.
  - Strut lattices have an explicit `BeamGraph` form that is the 3MF beam lattice (open
    question 3). "Schwarz D" and "diamond" are the same TPMS, so the list has 5 kinds.
  - Preview ray-marches the sampled brick grid rather than the tape (fast and independent of graph
    size); `ExternalLayer` contract accepted as is (open question 2).
  - Field expressions use their own small `FieldExpr` grammar; parameter values are substituted
    by the feature, so `amg-doc` expressions are not evaluated per point.
  - `amg-implicit` has its own `Backend` enum: `amg_mesh::ComputeBackend` (MESH-11) does not
    exist yet, and implicit must not wait on it.
- **WP changes** (IDs that other specs reference keep their meaning: IMP-02, IMP-03, IMP-04,
  IMP-05, IMP-06, IMP-07 are referenced by S07, S10, S12, S14, S15, S16 and the roadmap):
  - IMP-01: re-scoped to the graph core (smooth booleans, offsets, shells at the graph level
    move here; their *features* stay in IMP-08). Adds dep MESH-17 (`ScalarField` trait).
  - IMP-02: unchanged meaning; adds dep FND-09; budget made concrete (R1, GPU and CPU).
  - IMP-03: Done-when made measurable; adds `closest_group`.
  - IMP-04: adds deps MESH-02, MESH-07, MESH-17, MESH-19 (check, decimation, fallback, SI count);
    drops MESH-01 (implied).
  - IMP-05: adds dep UI-20 (it wires the layer in `amg-ui::implicit`).
  - IMP-06: stochastic lattices split out to IMP-14. Strut lattices, `BeamGraph` and skin stay, so
    IO-18 (deps IMP-06) is unchanged. Adds dep IMP-04 (thickness is measured on the mesh).
  - IMP-07: now only FEA-driven grading (the roadmap already calls it "stress-driven lattice
    density"); generic fields moved to IMP-16. Deps IMP-16, IMP-15, FEM-06.
  - IMP-08: now the implicit modeling features; adds deps IMP-11, IMP-12, IMP-13, IMP-16, UI-05.
  - IMP-09: adds dep IMP-15 (feature integration). IMP-10: approach per QUESTIONS #62 (decided 2026-11-09).
  - New: IMP-11 (timeline integration), IMP-12 (sampled grids), IMP-13 (display/export
    plumbing), IMP-14 (stochastic), IMP-15 (Volumetric Lattice feature + dialog), IMP-16 (fields).
- **Roadmap:** Phase 3 C gains IMP-11, IMP-12, IMP-13, IMP-15, IMP-16; Phase 5 C gains IMP-14.
  Every WP is scheduled no earlier than its deps: IMP-11 needs DOC-05/MESH-16 (Phase 1), IMP-02
  needs CMP-02 (Phase 2), IMP-05 needs RND-03/UI-20 (Phase 1), IMP-08/IMP-15 need UI-05 (Phase 2)
  and BREP-08 (Phase 1), IMP-07 needs FEM-06 (Phase 4).
- **Questions for the owner:** QUESTIONS #61 (`amg-ui::implicit` for lane C), #62 (IMP-10
  approach), #63 (cross-type booleans stay implicit).
- **Requests to other specs:** S01 (stub swap + codec), S12 (`BeamGraph` for IO-18), S13 and S14
  (`SampledGrid::from_dense` for result fields), S15 (CMP-02 brick-grid shape), S16
  (`mesh_for_export`). Each is a short dated note in that spec.
- Status set to `Refined`.

### 2026-11-09 - owner answers moved in
- QUESTIONS #61 (`amg-ui::implicit` for lane C; roadmap Lanes row needs the module), #62 (IMP-10: implicit heuristic first, topology-optimisation mode later), #63 (booleans with an implicit body stay implicit; IMP-08 Done-when, design note §10). Open questions 4-6 resolved.

### 2026-11-10 - IMP-03 Mesh SDF (lane C)
- **Landed** in `crates/amg-implicit`: `src/sdf.rs` (`amg_implicit::sdf::MeshSdf`, re-exported
  as `amg_implicit::MeshSdf`), `tests/mesh_sdf.rs`, `tests/common/mod.rs`, `benches/mesh_sdf.rs`.
  Deps: `rayon` (already workspace / THIRD_PARTY); dev: `amg-mesh` with `corpus`, `amg-testkit`,
  `criterion`, `proptest`. No new third-party crate, no `amg-mesh` change.
- `ScalarField` already exists in `amg_mesh::iso` (MESH-17), so IMP-03 did not define a trait;
  IMP-01 implements the same trait for `SdfGraph`. IMP-03 adds none of the crate-root types
  (`ImplicitError`, `Backend`, `Interval`); IMP-01 adds them.
- **Design as built:**
  - `|d|` = MESH-03 `closest_point` within the band, clamped to `band`; `±band` beyond it.
    For an interior (`Face`) closest point the distance is recomputed as the plane distance with
    a normal from the two shortest edges: MESH-03's Gram-determinant kernel loses up to ~2e-9 mm
    on long thin faces (corpus thingi10k-99899: 12 mm × 3 µm faces), which broke the 1e-9 mm
    Done-when. Edge and vertex distances are kept.
  - Sign: winding number > 0.5. On a closed mesh (boundary chain empty, checked by vertex index
    in `new`) the winding number is an integer off the surface, so a β = 1 estimate decides when
    within 0.2 of an integer, otherwise β = 2 (MESH-03 default). Open meshes always use β = 2.
    About 4× cheaper than β = 2 alone near the surface.
  - `values()` evaluates batches of ≥ 4096 points in parallel and in Morton order (scattered
    back; no value changes). On 2 M band points spread at random this goes from 1.73 s to 1.11 s.
  - `may_cross` (closed meshes only): false when no surface lies within `half_diagonal + |iso|`
    of the box centre, or when `|f(centre) − iso| > half_diagonal`; always false for
    `|iso| > band`. On open meshes it returns true for `|iso| ≤ band` (the sign can jump where the
    winding number passes 0.5, away from the surface).
  - `new` builds the winding tree eagerly (detached context; call `MeshSpatial::prepare_winding`
    first to make that cancellable). This also avoids the lazy `OnceLock` initialisation running
    rayon work from inside a parallel query loop.
  - Negative or NaN band → 0; `-0.0` normalised so `content_hash` is equal; NaN points → NaN.
- **Evidence** (16-thread dev machine; release for timings):
  - Brute force (10 k points in the band per model; the oracle uses the plane projection with a
    shortest-edge normal and is independent of the MESH-03 kernel): 5 synthetic closed meshes,
    worst `|d|` error 2.2e-15 mm; 56 watertight committed corpus models (all ≤ 120 k faces, none
    skipped), worst 3.9e-11 mm; 58 002 sign checks ≥ 1 mean edge
    off the surface, all equal to the exact winding number's sign.
  - Proptest: 1-Lipschitz (1e-9 slack) on 48 × 300 pairs straddling the surface and the band
    edge; `may_cross` never false when 64³ dense sampling crosses `iso` (48 boxes).
  - Budget: 2 M signed queries in the band of a 1 003 520-tri mesh (S07 benchmark sphere scaled
    to r = 50 mm), band 2 mm: 1.11 s (criterion `mesh_sdf_1m/signed_2m_band`; 2 M uniform box
    points 0.41 s). Asserted < 2 s by `budget_2m_queries_on_1m_tris` under `AMG_STRICT_TIMING=1`
    (release: 1.036 s, best of 3).
    There is no row for it in the vision budget table, so no `budget/<id>` bench.
  - Bit-identical values, groups, `may_cross` and hash across 1/4/16 threads (120 k-tri torus).
  - MESH-17 extraction of a torus `MeshSdf` (cell 0.25 mm) is closed, manifold, one shell,
    volume within 2 %.
- **Deviations:** no golden geometry test: IMP-03 produces a field, not geometry (IMP-04/IMP-12
  carry goldens). Additive API above the spec: `spatial()`, `is_closed()`, `is_inside()`,
  `FAST_BETA`, `FAST_MARGIN` (Interfaces updated).
- **Leftovers:** (1) MESH-03 `closest_on_tri` precision on high-aspect faces should be fixed in
  `amg-mesh` (a S07 follow-up; `MeshSdf` then no longer needs its plane-distance refinement).
  (2) `may_cross` does no pruning on open meshes (wrap inputs); a winding-number bound per box
  could add it later. (3) The closedness check is by vertex index: unwelded but geometrically
  closed meshes count as open (conservative).

## Interface changes
None yet (the spec has not reached `Ready`).
### 2026-11-08: from the S14 refinement (lane F, additive; needed by GEN-07)
- GEN-07 calls `SampledGrid::from_dense` with signed distances in mm (negative inside) on a cubic
  grid (S14 resamples and redistances the density first, `docs/notes/generative-design.md` §8.1).
  The result must be a `GridKind::Distance` grid, or `from_dense` takes a `kind: GridKind`
  argument (IMP-12 picks one).
- GEN-07 builds a graph `Grid ∪_blend MeshSdf(preserves) − (MeshSdf(obstacles) ⊕ offset)` and
  meshes it with `to_mesh` (IMP-04). No other change.
