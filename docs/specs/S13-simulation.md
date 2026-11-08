# S13 — Simulation (FEA)

| | |
|---|---|
| **Status** | Ready |
| **Priority** | P1 (static, modal) / P2 (thermal, buckling, nonlinear) / P3 (event) |
| **Crates** | `amg-fem`, `amg-tetwild-sys` (new, FFI), `amg-features::sim`, `amg-ui::sim` (Simulation workspace) |
| **Lane** | F |
| **Depends on** | S01 (DOC-05 `SetExt`, DOC-06 resolver, DOC-07 `ext/`, DOC-09 `ExtChanged`); S02 (BREP-08 tessellation, BREP-09 projection); S07 (MESH-03 `MeshSpatial`, MESH-04 repair); S10 (RND-07 overlays, RND-08 field shading); S11 (UI-02, UI-04, UI-05); S15 (CMP-03 direct, CMP-04 eigen, CMP-08 PCG, CMP-05 multigrid); S16 (AM-01 print setup, for FEM-22); S17 (MAT-01, MAT-06; MAT-04 for FEM-22) |
| **Parity** | `03-fusion-parity.md` §Simulation — all of it cloud-solved in Fusion, local here |

Design note: [`docs/notes/simulation-fem.md`](../notes/simulation-fem.md) (pipeline, meshing,
constraints, solvers, results, verification targets).

## Summary
Local finite-element analysis with a Fusion-like study workflow: create study → pick bodies →
assign materials → constraints → loads → contacts → mesh → solve (background job) → results
(stress, displacement, safety factor, strain, reaction forces, modes) with fringe plots, probes,
deformation animation and a report. P1 is linear static and modal on quadratic tets (TET10)
with a direct solver; a GPU iterative path takes larger models. Thermal, buckling, nonlinear
and printed (transversely isotropic) materials are P2; explicit event simulation is P3.

## Scope
**In:**
- Study model as an extension record, persistence, out-of-date tracking, pre-check.
- Tet meshing (fTetWild default, TetGen optional), TET10 promotion with snapping, sizing
  controls, quality report, mesh cache.
- Constraints: fixed (per axis), pin, frictionless, symmetry, prescribed displacement.
- Loads: force, pressure, bearing, gravity and linear acceleration, remote force, moment,
  angular velocity/acceleration (model P1, UI P2), temperature (thermal stress, P2).
- Bonded contacts (P1); separation, sliding and friction (P2).
- Linear static (multiple load cases), modal, linear buckling, steady-state thermal and thermal
  stress, nonlinear static (plasticity, large deflection, contact), explicit dynamics (P3).
- Results: fields, nodal averaging, safety factor, reactions, field surface for RND-08, field
  sampler for S08/S14, probes, animation, HTML report.
- Shared load-case model used by generative design (S14).
- Verification suite (NAFEMS, analytic) and a CalculiX export for differential tests.

**Out:**
- Topology and shape optimisation solvers → S14 (shape optimisation is listed under Fusion's
  simulation menu; it is S14 GEN-10).
- Generic sparse/eigen/GPU solvers → S15 (`amg-compute`); S13 states what it needs from them.
- Metal PBF inherent-strain simulation → S16 AM-09 (uses FEM-04 and CMP-05).
- Material data and anisotropic constitutive matrices → S17.
- Injection moulding, electronics cooling, CFD (non-goals).
- Hydrostatic pressure loads, rigid (RBE2) remote connections, transient thermal: later, no WP.

## Design
Full detail and the reasoning are in the design note. Summary:

- **Study model.** A study is an `ExtRecord` of kind `fem.study` (S01 §11), edited only through
  whole-record `Command::SetExt` (like the S16 print setup). It holds: type, bodies
  (`BodyId`s, with exclusions), per-body material overrides (default "same as model";
  unassigned is an error), load cases (each a list of constraints and loads on `EntityRef`
  targets), contacts (auto or manual pairs), mesh settings, solver settings, and a results
  header with `inputs_hash`. Results are a binary blob (`ext/`, format `AMGFEMR1`, note §9.3).
  `Study::status` compares `inputs_hash` with the current inputs (body geometry keys,
  material hashes, study JSON) → `OutOfDate` badge.
- **Layering.** `amg-fem` is document-agnostic: it works on surface meshes with `PatchId`s,
  tet meshes, `FeModel`s and solutions. `amg-features::sim` turns document bodies and
  `EntityRef`s into those (it sees `amg-doc`, `amg-brep`, `amg-mesh`, `amg-fem`) and runs the
  study job. `amg-ui::sim` is the workspace.
- **Geometry → analysis mesh.** Solid bodies are tessellated (BREP-08, face id per triangle)
  and mesh bodies use their face groups, so each surface triangle has a `PatchId`. The mesher
  (fTetWild via `amg-tetwild-sys`; TetGen behind feature `tetgen`, used by `MesherChoice::Auto` for clean closed input; QUESTIONS #71, decided) makes TET4;
  patch tags transfer to the tet boundary by nearest input triangle. TET10 promotion adds
  mid-nodes, snaps boundary ones to the exact surface (`SurfaceProjector`: BREP-09
  `project_point_on_face` for solids, `MeshSpatial::closest_point` for meshes) and
  straightens elements whose Jacobian turns bad. Sizing follows Fusion's defaults (10 % of the
  largest bbox side, min 20 %, growth 1.5) plus per-face and curvature refinement. The mesh is
  cached by (geometry keys, settings, mesher version).
- **Constraints and loads.** Every linear constraint (fixed, pin, frictionless, symmetry,
  prescribed displacement, bonded tie) becomes one map `u = T û + g` with per-node local frames;
  the reduced system `TᵀKT û = Tᵀ(f − Kg)` stays SPD. Loads become consistent nodal forces
  (TRI6 / TET10 quadrature). Bearing load = cosine pressure on the loaded half-cylinder. Remote
  force and moment use a distributing (RBE3-style) coupling, adding no stiffness.
- **Contacts.** Bodies mesh separately. `find_contacts` pairs facing patches of different
  bodies within tolerance; bonded pairs tie slave nodes to master TRI6 facets (node-to-surface
  tie, like CalculiX `*TIE`).
- **Element & assembly.** TET10 isoparametric (4-point stiffness rule, 11-point mass rule);
  parallel element computation and lock-free CSR scatter of the upper triangle; RCM node order.
  Thermal: the same shape functions, one DOF per node; conduction, convection, heat flux,
  internal heat, fixed temperature; radiation linearised by Picard iteration.
- **Solvers (S15).** Direct sparse Cholesky (faer, CMP-03) up to about 1.5 M DOF; one
  factorisation for all load cases with the same constraints. Modal: shift-invert Lanczos
  (CMP-04) with σ < 0 so free-free models work; default 8 modes. Larger models: GPU PCG, generic
  over `R: GpuReal` (CMP-05), with a p-multigrid preconditioner (TET10 smoother, TET4 coarse level by
  CPU Cholesky): f64 kernels where the device reports `SHADER_F64` (in practice Vulkan), otherwise
  f32 kernels inside f64 CPU iterative refinement (note §8.3; ADR-004 note, S15 `solve_gpu`). Buckling: generalised eigen
  with geometric stiffness. Unconstrained bodies are caught by a graph pre-check and by the
  pivot check, and reported by body name.
- **Nonlinear (P2).** Newton–Raphson with load stepping and cut-back; J2 plasticity with
  bilinear hardening (radial return, consistent tangent); total Lagrangian large displacement;
  contact by node-to-surface augmented Lagrangian with regularised Coulomb friction.
- **Event simulation (P3).** Explicit central differences, HRZ lumped mass, Courant step,
  optional mass scaling, GPU internal-force kernels.
- **Results.** Nodal displacement; Gauss-point stress extrapolated and averaged per body;
  derived von Mises, principal, max shear, strain, strain energy density, safety factor
  (`VonMisesYield` default, `MaxPrincipalUltimate` for brittle, Tsai-Wu in FEM-22); reactions
  per constraint; contact pressure (P2); modes with frequencies, participation and effective
  mass. `FieldSurface` (boundary TRI6 split into 4 triangles, f32 per-vertex values) feeds
  RND-08 `Shading::Field`; `TetFieldSampler` / `resample_grid` export fields for S08 IMP-07 and
  S14.
- **Verification.** Golden cases with targets in note §10: patch test, cantilever, NAFEMS LE1
  (92.7 MPa), LE10 (−5.38 MPa), plate with hole (Kt 3.02), FV32 (44.623 Hz …), free-free bar,
  LE11, Euler column, elastica, thick cylinder, Hertz, bar wave.

## Interfaces
Provisional Rust signatures. Names may change during implementation, but these shapes are what
S14, S08, S18 and the UI code against. Units are the `amg-core` canonical system (mm, t, s → N,
MPa, mJ); plain `f64` fields are canonical values.

```rust
// ───────────────────────────── amg_fem (crate root) ─────────────────────────────
// Deps: amg-core, amg-jobs, amg-compute, amg-mesh, amg-materials, amg-tetwild-sys (feature
// `tetwild`, default on), tetgen bindings (feature `tetgen`, off by default; AGPL, ADR-012).
use amg_core::{Point3, Vec3, Aabb3};
use amg_jobs::JobCtx;
use std::sync::Arc;

/// Patch of the analysis surface: one (body, B-rep face) or (body, mesh face group).
/// Assigned by the caller; `amg-fem` never interprets it beyond equality.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct PatchId(pub u32);
/// Index of a study body inside one model (0..n).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize)]
pub struct BodyIx(pub u16);

pub mod mesh {                                   // FEM-02, FEM-14
    use super::*;
    /// One closed surface per body, a patch per triangle (FEM-02).
    #[derive(Clone, Debug)]
    pub struct SurfaceInput {
        pub bodies: Vec<SurfaceBody>,
    }
    #[derive(Clone, Debug)]
    pub struct SurfaceBody {
        pub body: BodyIx,
        pub positions: Vec<Point3>,
        pub tris: Vec<[u32; 3]>,
        pub patch: Vec<PatchId>,                 // len == tris.len()
        /// Patch kind for frames and sizing: plane / cylinder (axis, radius) / other.
        pub patch_info: std::collections::BTreeMap<PatchId, PatchInfo>,
    }
    #[derive(Clone, Debug, PartialEq)]
    pub enum PatchInfo {
        Plane { origin: Point3, normal: Vec3 },
        Cylinder { origin: Point3, axis: Vec3, radius: f64 },
        Curved { min_radius: f64 },              // from B-rep curvature or mesh fit
        Other,
    }
    /// Exact surface for mid-node snapping (implemented in amg-features::sim).
    pub trait SurfaceProjector: Send + Sync {
        /// Closest point on the exact surface of `patch` near `p`; `None` if projection fails
        /// (the mid-node stays straight).
        fn project(&self, body: BodyIx, patch: PatchId, p: Point3) -> Option<Point3>;
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct MeshSettings {
        pub size: SizeMode,
        pub min_size_ratio: f64,                 // 0.2
        pub growth: f64,                         // 1.5
        pub curvature: Option<f64>,              // max angle per element on curved patches, rad (30°)
        pub local: Vec<LocalSize>,               // FEM-14
        pub order: ElementOrder,                 // Quadratic (default)
        pub mesher: MesherChoice,
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum SizeMode { ModelBased { percent: f64 /* 10.0 of largest bbox side */ }, Absolute { mm: f64 } }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct LocalSize { pub patches: Vec<(BodyIx, PatchId)>, pub size: f64 }
    #[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum ElementOrder { Linear, Quadratic }
    #[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum MesherChoice { Auto, FTetWild, TetGen }
    impl Default for MeshSettings { fn default() -> Self; }
    impl MeshSettings { pub fn cache_key(&self) -> [u8; 32]; }

    /// Evaluated size field (global + local + curvature), mm.
    pub struct SizeField { /* background octree */ }
    impl SizeField {
        pub fn build(input: &SurfaceInput, s: &MeshSettings) -> Result<SizeField, FemError>;
        pub fn at(&self, p: Point3) -> f64;
        pub fn global(&self) -> f64;
    }
    pub trait TetMesher: Send + Sync {
        fn name(&self) -> &'static str;
        fn version(&self) -> &'static str;       // goes into the mesh cache key
        fn mesh(&self, input: &SurfaceInput, size: &SizeField, ctx: &JobCtx) -> Result<TetMesh, FemError>;
    }
    pub fn mesher(choice: MesherChoice, input: &SurfaceInput) -> Result<Box<dyn TetMesher>, FemError>;

    #[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
    pub struct NodeId(pub u32);
    #[derive(Clone, Debug)]
    pub struct TetMesh {
        pub nodes: Vec<Point3>,
        /// TET4: 4 corner nodes; TET10: + 6 mid-nodes in edge order (0-1, 1-2, 2-0, 0-3, 1-3, 2-3).
        pub elems: TetElems,
        pub elem_body: Vec<BodyIx>,
        /// Boundary facets: element, local face (0..4), patch.
        pub boundary: Vec<BoundaryFacet>,
        /// Nodes on a patch-boundary edge (for edge/vertex BCs, note §6.1).
        pub edge_nodes: Vec<NodeId>,
    }
    #[derive(Clone, Debug)]
    pub enum TetElems { Tet4(Vec<[u32; 4]>), Tet10(Vec<[u32; 10]>) }
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub struct BoundaryFacet { pub elem: u32, pub face: u8, pub body: BodyIx, pub patch: PatchId }
    impl TetMesh {
        pub fn node_count(&self) -> usize;
        pub fn elem_count(&self) -> usize;
        pub fn dof_count(&self, dofs_per_node: usize) -> usize;
        /// Corner-node volume per body (for the volume check against the input).
        pub fn volume(&self, body: BodyIx) -> f64;
        pub fn facet_nodes(&self, f: &BoundaryFacet) -> smallvec::SmallVec<[u32; 6]>;
        pub fn quality(&self) -> QualityReport;
        pub fn content_hash(&self) -> [u8; 32];
    }
    /// FEM-14: add mid-nodes, snap boundary mid-nodes, straighten bad elements.
    pub fn promote_tet10(m: &TetMesh, proj: &dyn SurfaceProjector, ctx: &JobCtx) -> Result<TetMesh, FemError>;
    /// FEM-02: patch transfer from input triangles to tet boundary facets (nearest centroid).
    pub fn transfer_patches(m: &mut TetMesh, input: &SurfaceInput, ctx: &JobCtx) -> Result<TagReport, FemError>;
    #[derive(Clone, Debug, Default, PartialEq)]
    pub struct TagReport { pub facets: usize, pub ambiguous: usize }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct QualityReport {
        pub elements: usize, pub nodes: usize, pub dofs: usize,
        pub radius_ratio_min: f64, pub radius_ratio_median: f64,
        pub dihedral_min: f64,                   // rad
        pub aspect_max: f64,
        pub jacobian_ratio_hist: [u32; 10],      // bins of 0.1
        pub poor: usize,                         // radius ratio < 0.1 or dihedral < 5°
        pub inverted: usize,                     // det J ≤ 0 anywhere (solver refuses if > 0)
        pub curved: usize, pub straightened: usize,
        pub ambiguous_tags: usize,
    }
}

pub mod model {                                  // FEM-13 (types), FEM-15 (discretisation)
    use super::*;
    /// Geometric target of a BC or load, in patch terms (shared with S14).
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum Target {
        Faces(Vec<(BodyIx, PatchId)>),
        /// Edge between two patches (both on the same body).
        Edge { body: BodyIx, a: PatchId, b: PatchId },
        /// Point (snapped to the nearest edge node, or any boundary node).
        Vertex { body: BodyIx, at: Point3 },
        Bodies(Vec<BodyIx>),                     // body loads
    }
    #[derive(Copy, Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct Axes(pub u8);                     // bit mask: 1 = x/radial, 2 = y/tangential, 4 = z/axial
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum Frame { Global, Custom(glam::DMat3) /* columns = axes */ }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum Constraint {
        Fixed { target: Target, axes: Axes, frame: Frame },
        /// Cylindrical face; frame from `PatchInfo::Cylinder`.
        Pin { target: Target, radial: bool, tangential: bool, axial: bool },
        Frictionless { target: Target },
        /// Planar faces only; validated against `PatchInfo::Plane`.
        Symmetry { target: Target },
        Prescribed { target: Target, frame: Frame, value: [Option<f64>; 3] /* mm */ },
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum Direction { Vector(Vec3), Normal /* outward; force only */ }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum Load {
        Force { target: Target, total: f64 /* N */, dir: Direction },
        Pressure { target: Target, p: f64 /* MPa, positive pushes in */ },
        Bearing { target: Target, total: f64, dir: Vec3 },
        RemoteForce { target: Target, point: Point3, force: Vec3, moment: Vec3 },
        Moment { target: Target, moment: Vec3 /* N·mm about the region centroid */ },
        Gravity { accel: Vec3 /* mm/s², default (0,0,-9806.65) */ },
        LinearAccel { accel: Vec3 },
        Rotation { origin: Point3, axis: Vec3, omega: f64 /* rad/s */, alpha: f64 /* rad/s² */ },
        Temperature { bodies: Vec<BodyIx>, delta: f64 /* K, uniform; FEM-20 */ },
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct LoadCase { pub name: String, pub constraints: Vec<Constraint>, pub loads: Vec<Load> }
    impl LoadCase {
        /// Hash of the constraint set only (load cases with equal keys share a factorisation).
        pub fn constraint_key(&self) -> [u8; 32];
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum ContactKind { Bonded, Separation, Sliding, Frictional { mu: f64 } }   // P2: all but Bonded
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct ContactPair { pub a: Target, pub b: Target, pub kind: ContactKind }
    /// FEM-16: facing patches of different bodies within `tol` (opposed normals, cos ≤ −0.9).
    pub fn find_contacts(input: &mesh::SurfaceInput, tol: f64, ctx: &JobCtx) -> Vec<ContactPair>;

    /// Per-body material as the solver sees it (P1: isotropic only).
    #[derive(Clone, Debug, PartialEq)]
    pub struct SolidMaterial {
        pub elastic: ElasticModel,
        pub density: f64,                        // t/mm³
        pub yield_strength: Option<f64>,         // MPa (Directional::min in P1)
        pub ultimate_tensile: Option<f64>,
        pub ultimate_compressive: Option<f64>,
        pub expansion: Option<f64>,              // 1/K (FEM-20)
        pub conductivity: Option<f64>,           // W/(m·K) canonical (FEM-09)
        pub plasticity: Option<amg_materials::Plasticity>,   // FEM-11
        pub warnings: Vec<String>,               // e.g. isotropic equivalent used
    }
    #[derive(Clone, Debug, PartialEq)]
    pub enum ElasticModel {
        Isotropic { e: f64, nu: f64 },
        /// FEM-22: Voigt (VOIGT_ORDER) stiffness in the global frame (already rotated).
        Anisotropic { c: [[f64; 6]; 6] },
    }
    impl SolidMaterial {
        /// P1 rule (S17): `Elastic::as_isotropic()`, else `isotropic_equivalent(policy)` + warning.
        pub fn from_material(m: &amg_materials::Material, policy: amg_materials::IsoPolicy) -> Result<SolidMaterial, FemError>;
    }

    /// Everything the solver needs; built by amg-features::sim from a study.
    pub struct FeModel {
        pub mesh: Arc<mesh::TetMesh>,
        pub surface: Arc<mesh::SurfaceInput>,    // patch info, for frames and pins
        pub materials: Vec<SolidMaterial>,       // indexed by BodyIx
        pub cases: Vec<LoadCase>,
        pub contacts: Vec<ContactPair>,
    }
    #[derive(Clone, Debug, PartialEq)]
    pub struct Estimate { pub dofs: usize, pub nnz: usize, pub direct_bytes: u64, pub iterative_bytes: u64 }
    impl FeModel {
        pub fn estimate(&self) -> Estimate;
        /// Static checks: every body has a material, constraints hit the mesh, pins hit
        /// cylinders, symmetry hits planes, each body is constrained or tied to one (note §6.5).
        pub fn validate(&self) -> Vec<FemDiagnostic>;
    }

    /// FEM-15: mesh entities of a target.
    #[derive(Clone, Debug, Default)]
    pub struct MeshRegion { pub facets: Vec<u32> /* index into TetMesh::boundary */, pub nodes: Vec<NodeId>, pub area: f64 }
    pub fn map_target(m: &mesh::TetMesh, t: &Target) -> Result<MeshRegion, FemError>;
    /// FEM-15: u = T û + g for one load case (incl. bonded ties).
    pub struct ConstraintMap { /* sparse T (CSR), g, local frames, reduced dof count */ }
    impl ConstraintMap {
        pub fn build(model: &FeModel, case: usize, dofs_per_node: usize) -> Result<(ConstraintMap, Vec<FemDiagnostic>), FemError>;
        pub fn reduced_dofs(&self) -> usize;
        pub fn expand(&self, reduced: &[f64]) -> Vec<f64>;
    }
    /// FEM-15: consistent nodal load vector (full DOFs) of one load case.
    pub fn load_vector(model: &FeModel, case: usize) -> Result<Vec<f64>, FemError>;
}

pub mod solve {                                  // FEM-04, FEM-07, FEM-08, FEM-10
    use super::*;
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum SolverChoice { Auto, Direct, GpuIterative { rel_tol: f64 /* 1e-8 */, max_iter: u32 } }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct SolverSettings { pub solver: SolverChoice, pub modes: u32 /* 8 */, pub shift_hz: f64 /* -1.0 */,
                                pub min_freq_hz: Option<f64>, pub max_freq_hz: Option<f64> }
    impl Default for SolverSettings { fn default() -> Self; }
    pub fn solve_static(model: &model::FeModel, s: &SolverSettings, ctx: &JobCtx) -> Result<results::StaticSolution, FemError>;
    pub fn solve_modal(model: &model::FeModel, s: &SolverSettings, ctx: &JobCtx) -> Result<results::ModalSolution, FemError>;
    pub fn solve_buckling(model: &model::FeModel, s: &SolverSettings, ctx: &JobCtx) -> Result<results::BucklingSolution, FemError>;  // FEM-10
    pub fn solve_thermal(model: &model::FeModel, t: &thermal::ThermalCase, s: &SolverSettings, ctx: &JobCtx) -> Result<results::ThermalSolution, FemError>; // FEM-09
    /// Timing and solver facts recorded in the result header and printed by the CLI.
    #[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct SolveStats { pub dofs: usize, pub nnz: usize, pub solver: String, pub iterations: u32,
                            pub t_assemble_s: f64, pub t_factor_s: f64, pub t_solve_s: f64, pub t_recover_s: f64, pub peak_bytes: u64 }
}

pub mod thermal {                                // FEM-09
    use super::*;
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub enum ThermalBc {
        Temperature { target: model::Target, t: f64 /* K */ },
        HeatFlux { target: model::Target, q: f64 /* mW/mm² canonical */ },
        HeatSource { bodies: Vec<BodyIx>, power: f64 /* mW, total */ },
        Convection { target: model::Target, h: f64 /* mW/(mm²·K) */, t_ambient: f64 },
        Radiation { target: model::Target, emissivity: f64, t_ambient: f64 },
    }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct ThermalCase { pub name: String, pub bcs: Vec<ThermalBc> }
}

pub mod results {                                // FEM-04, FEM-06, FEM-07
    use super::*;
    #[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum Quantity {
        DisplacementMag, DisplacementX, DisplacementY, DisplacementZ,
        VonMises, Principal1, Principal2, Principal3, MaxShear,
        StressXX, StressYY, StressZZ, StressYZ, StressXZ, StressXY,
        EquivalentStrain, StrainEnergyDensity, SafetyFactor,
        Temperature, HeatFluxMag,                // thermal
        ContactPressure,                         // P2
    }
    #[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
    pub enum SafetyCriterion { VonMisesYield, MaxPrincipalUltimate, TsaiWu /* FEM-22 */ }
    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    pub struct Reaction { pub constraint: usize, pub force: Vec3, pub moment: Vec3 /* about region centroid */ }
    /// One solved state (a load case, a mode or a buckling shape).
    #[derive(Clone, Debug)]
    pub struct Field {
        pub displacement: Vec<[f64; 3]>,         // per node
        pub stress: Vec<[f64; 6]>,               // per node, averaged per body; VOIGT_ORDER
        pub gauss_stress: Vec<[f32; 6]>,         // per element × 4
        pub temperature: Option<Vec<f64>>,
    }
    impl Field {
        pub fn nodal(&self, q: Quantity, mats: &[model::SolidMaterial], mesh: &mesh::TetMesh, c: SafetyCriterion) -> Vec<f64>;
        pub fn max(&self, q: Quantity, mats: &[model::SolidMaterial], mesh: &mesh::TetMesh, c: SafetyCriterion) -> (f64, NodeIdx);
        pub fn min(&self, q: Quantity, mats: &[model::SolidMaterial], mesh: &mesh::TetMesh, c: SafetyCriterion) -> (f64, NodeIdx);
    }
    pub type NodeIdx = u32;
    pub struct StaticSolution { pub mesh: Arc<mesh::TetMesh>, pub cases: Vec<(Field, Vec<Reaction>)>, pub stats: solve::SolveStats }
    pub struct Mode { pub freq_hz: f64, pub field: Field, pub participation: [f64; 6], pub effective_mass: [f64; 6] }
    pub struct ModalSolution { pub mesh: Arc<mesh::TetMesh>, pub modes: Vec<Mode>, pub total_mass: f64, pub stats: solve::SolveStats }
    pub struct BucklingSolution { pub mesh: Arc<mesh::TetMesh>, pub factors: Vec<(f64, Field)>, pub stats: solve::SolveStats }
    pub struct ThermalSolution { pub mesh: Arc<mesh::TetMesh>, pub temperature: Vec<f64>, pub stats: solve::SolveStats }

    /// FEM-06: what RND-08 draws (S10 `upload_scalar_field` / `upload_vector_field`).
    pub struct FieldSurface {
        pub positions: Vec<Point3>,              // boundary TRI6 split into 4 triangles
        pub tris: Vec<[u32; 3]>,
        pub tri_body: Vec<BodyIx>,
        pub node_of_vertex: Vec<NodeIdx>,
    }
    impl FieldSurface {
        pub fn build(mesh: &mesh::TetMesh) -> FieldSurface;
        pub fn scalars(&self, nodal: &[f64]) -> Vec<f32>;
        pub fn vectors(&self, displacement: &[[f64; 3]]) -> Vec<[f32; 3]>;
        /// Fusion-like "adjusted" scale: 5 % of bbox diagonal / max displacement.
        pub fn auto_scale(&self, displacement: &[[f64; 3]]) -> f32;
    }
    /// FEM-06: point sampling for S08 IMP-07 and S14 (world frame of the study bodies).
    pub struct TetFieldSampler { /* Arc<TetMesh>, tet BVH, nodal values */ }
    impl TetFieldSampler {
        pub fn new(mesh: Arc<mesh::TetMesh>, nodal: Arc<[f64]>, ctx: &JobCtx) -> Result<TetFieldSampler, FemError>;
        pub fn sample(&self, p: Point3) -> Option<f64>;
        /// Regular grid, x fastest; NaN outside the mesh.
        pub fn resample_grid(&self, origin: Point3, spacing: f64, dims: [u32; 3], ctx: &JobCtx) -> Result<Vec<f32>, FemError>;
    }
    /// FEM-06: probe value at a point on or in the mesh (interpolated with TET10 shape functions).
    pub fn probe(mesh: &mesh::TetMesh, nodal: &[f64], p: Point3) -> Option<f64>;
}

pub mod store {                                  // FEM-17: result blob `AMGFEMR1` (note §9.3)
    use super::*;
    pub const RESULT_FORMAT: u32 = 1;
    pub enum Solution { Static(results::StaticSolution), Modal(results::ModalSolution),
                        Buckling(results::BucklingSolution), Thermal(results::ThermalSolution) }
    pub fn write<W: std::io::Write>(s: &Solution, w: W, ctx: &JobCtx) -> Result<(), FemError>;
    pub fn read<R: std::io::Read>(r: R, ctx: &JobCtx) -> Result<Solution, FemError>;
}

pub mod export {                                 // FEM-25
    /// CalculiX `.inp` (C3D10, *TIE, *BOUNDARY, *CLOAD/*DLOAD, *STATIC/*FREQUENCY).
    pub fn calculix_inp<W: std::io::Write>(model: &super::model::FeModel, case: usize, w: W) -> std::io::Result<()>;
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FemDiagnostic { pub severity: Severity, pub message: String, pub target: Option<String> }
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Severity { Info, Warning, Error }

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum FemError {
    #[error("mesher `{mesher}` failed: {reason}")] Mesher { mesher: &'static str, reason: String },
    #[error("mesher `{0}` is not built in (cargo feature)")] MesherUnavailable(&'static str),
    #[error("{0} inverted elements after promotion")] Inverted(usize),
    #[error("body {body:?} can move freely (zero pivot at dof {dof})")] Unconstrained { body: BodyIx, dof: usize },
    #[error("constraint {index} has no mesh entities")] EmptyTarget { index: usize },
    #[error("pin constraint {index} is not on a cylindrical face")] NotCylinder { index: usize },
    #[error("symmetry constraint {index} is not on a planar face")] NotPlanar { index: usize },
    #[error("body {0:?} has no usable material: {1}")] Material(BodyIx, String),
    #[error("model needs about {need} bytes, {have} available")] OutOfMemory { need: u64, have: u64 },
    #[error("solver did not converge: {0}")] NoConvergence(String),
    #[error("result file: {0}")] Store(String),
    #[error("solver: {0}")] Solver(String),       // wraps amg_compute errors
    #[error("operation cancelled")] Cancelled,
}
impl From<amg_core::Cancelled> for FemError { fn from(_: amg_core::Cancelled) -> Self { FemError::Cancelled } }
impl amg_core::Diagnostic for FemError { /* hints: "add a fixed constraint to Body 2", "use a coarser mesh or the GPU solver" */ }
```

```rust
// ───────────────────────── amg_features::sim (FEM-01, FEM-03, FEM-13, FEM-17) ─────────────────────────
use amg_doc::{BodyId, Command, Design, Evaluated, EntityRef, ExtKey, ExtRecord, ExtRegistration, MigrationError};
use amg_features::mesh::MeshEntityRef;
use amg_materials::MaterialId;

pub const STUDY_KIND: &str = "fem.study";
pub const STUDY_VERSION: u32 = 1;

#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum StudyType { Static, Modal, Thermal, ThermalStress, Buckling, NonlinearStatic, Event }

/// A face, edge or vertex of a study body, as the user picked it.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum SimRef {
    Solid(EntityRef),                            // EntityRef::Topo on a solid body (DOC-06)
    MeshGroup { body: BodyId, group: MeshEntityRef },
    Body(BodyId),
    Point(amg_core::Point3),                     // remote-force point, vertex fallback
}
/// FEM-13: the load-case model in document terms (shared with S14 GEN-05).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LoadCaseDef {
    pub name: String,
    pub constraints: Vec<BcDef<amg_fem::model::Constraint>>,
    pub loads: Vec<BcDef<amg_fem::model::Load>>,
}
/// The `amg-fem` item with its document targets; the item's own `Target` is filled by
/// `resolve_load_case` (stored as `Target::Faces(vec![])` in the record).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BcDef<T> { pub name: String, pub targets: Vec<SimRef>, pub item: T, pub suppressed: bool }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ContactSettings { pub auto: bool, pub tolerance: Option<f64>, pub default_kind: amg_fem::model::ContactKind,
                             pub manual: Vec<(SimRef, SimRef, amg_fem::model::ContactKind)>, pub excluded: Vec<(SimRef, SimRef)> }

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Study {
    pub name: String,
    pub kind: StudyType,
    pub bodies: Vec<BodyId>,                     // empty = all visible solid and mesh bodies
    pub excluded: Vec<BodyId>,
    pub materials: std::collections::BTreeMap<BodyId, MaterialId>,   // overrides; else Design::body_material
    pub iso_policy: IsoPolicyDef,                // Conservative (default) | InPlane
    pub cases: Vec<LoadCaseDef>,
    pub thermal: Vec<amg_fem::thermal::ThermalCase>,   // targets as in BcDef (FEM-09)
    pub contacts: ContactSettings,
    pub mesh: amg_fem::mesh::MeshSettings,
    pub local_sizes: Vec<(Vec<SimRef>, f64)>,
    pub solver: amg_fem::solve::SolverSettings,
    pub safety: amg_fem::results::SafetyCriterion,
    pub results: Option<ResultsHeader>,
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ResultsHeader {
    pub inputs_hash: String,                     // hex blake3, note §9.6
    pub blob: amg_doc::BlobRef,
    pub summary: ResultSummary,
    pub stats: amg_fem::solve::SolveStats,
    pub quality: amg_fem::mesh::QualityReport,
    pub diagnostics: Vec<amg_fem::FemDiagnostic>,
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ResultSummary {
    pub cases: Vec<CaseSummary>,                 // static
    pub modes_hz: Vec<f64>,                      // modal
    pub buckling_factors: Vec<f64>,
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CaseSummary { pub name: String, pub max_displacement: f64, pub max_von_mises: f64,
                         pub min_safety_factor: f64, pub reactions: Vec<amg_fem::results::Reaction> }

impl Study {
    pub fn new(name: &str, kind: StudyType) -> Study;            // Fusion defaults, one empty load case
    pub fn to_record(&self) -> ExtRecord;                         // deterministic JSON
    pub fn from_record(r: &ExtRecord) -> Result<Study, SimError>;
    /// The one mutation path: whole-record replace (undoable).
    pub fn command(&self, key: ExtKey) -> Command;                // Command::SetExt
    /// Bodies actually in the study (resolves "empty = all", drops excluded and missing).
    pub fn study_bodies(&self, d: &Design, e: &Evaluated) -> Vec<BodyId>;
    pub fn status(&self, d: &Design, e: &Evaluated) -> StudyStatus;
    /// Fusion's pre-check: what is missing before Solve (shared by UI and CLI).
    pub fn precheck(&self, d: &Design, e: &Evaluated) -> Vec<amg_fem::FemDiagnostic>;
    pub fn inputs_hash(&self, d: &Design, e: &Evaluated) -> String;
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StudyStatus { NotSolved, Solved, OutOfDate, Invalid(String) }
pub fn new_study_command(d: &Design, s: &Study) -> (ExtKey, Command);   // AddExt
pub fn delete_study_command(key: ExtKey) -> Command;                   // SetExt { record: None }
pub fn studies(d: &Design) -> Vec<(ExtKey, Result<Study, SimError>)>;  // needs S01 Design::exts_of_kind

/// FEM-13: document targets → geometric patches (also used by S14 for voxel BCs).
pub struct StudyGeometry {
    pub input: Arc<amg_fem::mesh::SurfaceInput>,
    pub body_ix: Vec<(BodyId, amg_fem::BodyIx)>,
    pub patch_of: std::collections::BTreeMap<(BodyId, PatchKey), amg_fem::PatchId>,
    pub projector: Arc<dyn amg_fem::mesh::SurfaceProjector>,
    pub geometry_keys: Vec<(BodyId, [u8; 32])>,
}
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum PatchKey { BrepFace(u32), MeshGroup(u32) }
pub fn study_geometry(s: &Study, d: &Design, e: &Evaluated, ctx: &amg_jobs::JobCtx) -> Result<StudyGeometry, SimError>;
/// Fills each item's `Target` from its `SimRef`s; unresolved refs → diagnostics, not panics.
pub fn resolve_load_case(c: &LoadCaseDef, g: &StudyGeometry, d: &Design, e: &Evaluated)
    -> (amg_fem::model::LoadCase, Vec<amg_fem::FemDiagnostic>);
/// World-space triangles of a resolved target (for S14 voxel mapping and UI glyphs).
pub fn target_triangles(g: &StudyGeometry, t: &amg_fem::model::Target) -> Vec<[amg_core::Point3; 3]>;

/// FEM-17: the solve job (mesh cache → model → solve → result blob → SetExt).
pub struct RunStudy { pub key: ExtKey, pub design: amg_doc::DesignSnapshot, pub eval: Arc<Evaluated>, pub cache: Arc<MeshCache> }
impl amg_jobs::Job for RunStudy {
    type Output = RunOutput; type Error = SimError;
    fn label(&self) -> std::borrow::Cow<'static, str>;
    fn run(self, ctx: &amg_jobs::JobCtx) -> Result<RunOutput, SimError>;
}
pub struct RunOutput { pub command: Command /* SetExt with ResultsHeader */, pub blob: (amg_doc::BlobRef, Vec<u8>), pub solution: Arc<amg_fem::store::Solution> }
/// Session cache of tet meshes keyed by (geometry keys, MeshSettings::cache_key, mesher version).
pub struct MeshCache { /* LRU, byte budget */ }
impl MeshCache { pub fn new(budget_bytes: u64) -> MeshCache; pub fn len(&self) -> usize; }
/// Loads a study's result blob (lazily, as a job) for display or scripting.
pub fn load_results(d: &Design, key: &ExtKey, blobs: &dyn amg_doc::BlobSource, ctx: &amg_jobs::JobCtx) -> Result<amg_fem::store::Solution, SimError>;

#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum SimError {
    #[error("study record is invalid: {0}")] Decode(String),
    #[error("study v{0} is newer than supported")] TooNew(u32),
    #[error("study has no bodies")] NoBodies,
    #[error("body {0:?} has no material assigned")] NoMaterial(BodyId),
    #[error("body {0:?} cannot be meshed: {1}")] Geometry(BodyId, String),
    #[error(transparent)] Fem(#[from] amg_fem::FemError),
    #[error("operation cancelled")] Cancelled,
}
inventory::submit! { ExtRegistration { kind: STUDY_KIND, version: STUDY_VERSION, migrate: migrate_study } }
fn migrate_study(from: u32, v: serde_json::Value) -> Result<serde_json::Value, MigrationError>;
```

`amg-ui::sim` exposes no API to other crates. It registers (UI-02) the Simulation workspace and
commands `sim.new_study`, `sim.study_materials`, `sim.fixed`, `sim.pin`, `sim.frictionless`,
`sim.symmetry`, `sim.prescribed`, `sim.force`, `sim.pressure`, `sim.bearing`, `sim.gravity`,
`sim.remote_force`, `sim.moment`, `sim.contacts_auto`, `sim.contacts_manage`, `sim.mesh`,
`sim.mesh_settings`, `sim.precheck`, `sim.solve`, `sim.results`, `sim.probe`, `sim.animate`,
`sim.report`, plus a `BrowserProvider` (study tree: bodies, materials, load cases, contacts,
mesh, results) and an `OverlayProvider` (BC/load glyphs, RND-07).

**Needed from other specs** (requests, not edits; listed in the refinement log):
- **S01:**
  - `ExtRegistration` collection and migration on load (S01 §11; noted as not done in the
    DOC-07 log). Until it lands, `Study::from_record` checks `version` itself (`TooNew`).
  - `Design::exts_of_kind(&self, kind) -> impl Iterator<Item = (&ExtKey, &ExtRecord)>` (same
    request as S16).
  - `Evaluated::body_key(&self, BodyId) -> Option<CacheKey>`: the cache key of the node that
    last produced the body's geometry (out-of-date tracking and the mesh cache, note §9.6).
  - Blob read/write for extension records outside a full save: a `BlobSource` trait to read
    a `BlobRef` from the open document's container, and a way for a job result to attach a new
    blob (`Command::SetExt` plus pending blob bytes kept until the next save). Large blobs
    (tens of MB) must not be copied into undo snapshots.
  - A resolver entry point usable outside `RecomputeCtx`:
    `Evaluated::resolve(&self, r: &TopoRef) -> Result<Resolved, ResolveError>` (DOC-06).
- **S02:** `project_point_on_face` (BREP-09) and per-face `GeomClass` with cylinder axis and
  radius and plane origin/normal (for `PatchInfo`). Already in the S02 Interfaces; S13 only
  confirms the use. Min curvature radius per face is optional (sizing; else estimated from the
  tessellation).
- **S07:** none new. Uses `MeshSpatial::{build, closest_point, faces_in_aabb}`,
  `FaceGroups` with `Primitive::{Plane, Cylinder}`, `AnalysisReport` (closed, manifold,
  self-intersections) for `MesherChoice::Auto`.
- **S10 (RND-08, RND-07):** none new: `upload_scalar_field`, `upload_vector_field`,
  `Shading::Field` with `Deformation { animation: Sine }`, `legend`; overlay arrows, rings and
  triads for glyphs. Min/max markers use RND-07 labels.
- **S11:** a Simulation workspace slot (UI-01 stub module `sim` already planned); generic
  selection with an `EntityFilter` for faces/edges/vertices of solid and mesh bodies (UI-04).
  Study dialogs are custom `Tool`s, not feature dialogs.
- **S15** (S15 is Draft; these are inputs to its refinement):
  - CMP-03: CSR type with symmetric upper storage; Cholesky with separate symbolic and numeric
    phases, multiple right-hand sides, a pivot report (index of the first pivot below a
    threshold) and a memory estimate before factoring; LDLᵀ for indefinite shifted systems
    (shift-invert with σ < 0 is SPD for SPD K, but buckling needs indefinite).
  - CMP-04: shift-invert Lanczos for `K φ = λ M φ` taking a factorisation callback, n smallest
    or in a frequency band, with M-orthonormal vectors.
  - CMP-08 (was CMP-05 before the S15 re-split): PCG on the GPU with CSR values of type `R: GpuReal` (f32 or f64), a `Preconditioner` trait so S13 can plug in the
    p-multigrid, and Chebyshev–Jacobi smoothing.
- **S16 (AM-01):** read access to a print setup's item rotation for a body (build direction in
  body coordinates) for FEM-22. Uses `amg_features::am::setups`.
- **S17:** none new: `Material::{elastic, content_hash}`, `Elastic::{as_isotropic,
  isotropic_equivalent, stiffness_voigt}`, `rotate_stiffness`, `Strength`, `Directional::min`,
  `Thermal`, `Plasticity`, `Strength::tsai_wu` (FEM-22).
- **S18 (SCR-12):** should depend on FEM-17 (run job), not only FEM-01; the Python study API
  wraps `Study`, `RunStudy` and `load_results`.
- **S14:** consumes `LoadCaseDef`, `study_geometry`, `resolve_load_case`, `target_triangles`
  (FEM-13) and `TetFieldSampler` (FEM-06).
- **S08 (IMP-07):** consumes `TetFieldSampler::resample_grid` through `amg-features`.
- **Roadmap / S00:** add `amg-tetwild-sys` to lane F's crates and to the S00 layering
  allow-list (`amg-fem → amg-tetwild-sys`). THIRD_PARTY rows for fTetWild (MPL-2.0), geogram
  (BSD-3), libigl (MPL-2.0), MPIR/GMP (LGPL-3.0, dynamic), TetGen (AGPL-3.0, feature `tetgen`).

## Work packages
WPs are ≈ 1–3 days each. FEM-01..13 keep their IDs (several re-scoped: FEM-02 split into
FEM-02 + FEM-14, FEM-03 into FEM-03 + FEM-15, FEM-06 into FEM-06 + FEM-19, FEM-09 into FEM-09 +
FEM-20, FEM-11 into FEM-11 + FEM-21). FEM-14..25 are new. Times are on the reference machine
(RTX 5060 Ti, 8+ cores), release build. "Golden" = `tests/golden/fem/` with the targets in
note §10. "Corpus" = `testdata/fem/corpus/` (note §11).

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| FEM-01 | Study data model: `Study` record (`fem.study`, v1), `to_record`/`from_record`, `command`, `new_study_command`, `studies`, `study_bodies`, `inputs_hash`, `status` (out-of-date), `precheck`; `ExtRegistration` + migration stub | DOC-05, DOC-07, DOC-09, MAT-06 | P1 | Todo | 1. A study with 2 load cases, 5 BCs and a material override round-trips `to_record`/`from_record` and a `.amg` save→load→save with byte-identical `design.json`. 2. `SetExt` edits undo/redo to equal records. 3. Changing a body's upstream feature parameter, its material, or any BC flips `status` to `OutOfDate`; changing only a UI-only field (study name) does not. 4. `precheck` reports: no bodies, unassigned material, no constraint on a body, empty target — each as a named diagnostic (table test). 5. A v2 fixture record gives `TooNew`, a garbled one `Decode` (no panic). |
| FEM-02 | `amg-tetwild-sys` (vcpkg overlay port, MPIR, shared vcpkg oneTBB) + `TetMesher` trait, fTetWild backend (in-process API or `.msh` subprocess fallback, note §12), `SurfaceInput`, global sizing, `transfer_patches`; optional TetGen backend behind `tetgen` | MESH-04, MESH-03, FND-05 | P1 | Todo | 1. CI builds `amg-tetwild-sys` on Windows; `cargo deny` passes; THIRD_PARTY rows added. 2. Corpus (≥ 40 bodies, note §11) at default settings: ≥ 95 % mesh with no error, 0 inverted TET4, volume within 1 % of input, `ambiguous_tags` < 1 % of boundary facets. 3. Per-body mesh times recorded in the note (median, p95, and the 120 k-tet case must be < 30 s, the meshing budget of QUESTIONS #70); with `tetgen`, the same table for TetGen on clean inputs, and a recommendation on Gmsh/HXT (QUESTIONS #72) if either mesher misses the budget or the 95 % success rate. `Auto` picks TetGen for clean closed input (QUESTIONS #71). 4. A cancelled meshing job returns `Cancelled` within 1 s. 5. Only one TBB library is loaded in a process using both Manifold and fTetWild (test lists loaded modules). |
| FEM-03 | BC/load target mapping: `SimRef` → `PatchId` sets (DOC-06 resolver for solids, `resolve_group` for meshes), `study_geometry` (`SurfaceInput`, `PatchInfo`, `SurfaceProjector` impls), `map_target` (facets, edge regions, vertex snapping), `target_triangles` | FEM-02, FEM-13, DOC-06, BREP-08, BREP-09 | P1 | Todo | 1. Box, cylinder-with-hole and L-bracket fixtures: every face/edge/vertex `SimRef` maps to the expected patch and the mapped facet area equals the B-rep face area within 0.5 % (mesh deflection). 2. After an upstream edit that keeps the face (DOC-11 corpus case TNP-02-style), the BC still maps to the same face; a deleted face gives a diagnostic naming the BC, not a panic. 3. A mesh body with 3 face groups maps by group id and by fingerprint after a group renumbering. 4. `PatchInfo::Cylinder` axis within 1e-6 rad of the B-rep axis. |
| FEM-04 | Linear static core: TET10 element (4-pt stiffness, B-matrix), quadrature tables, parallel assembly into reduced CSR (`TᵀKT`), direct solve via CMP-03 (multi-RHS per constraint key), Gauss-point stress, extrapolation and per-body nodal averaging, reactions, unconstrained-body detection, `solve_static` | FEM-15, CMP-03 | P1 | Todo | 1. Quadrature rules integrate all monomials up to their degree within 1e-14. 2. Golden: patch test (1e-10), cantilever (1 %), NAFEMS LE1 92.7 MPa and LE10 −5.38 MPa within 2 %, plate with hole Kt 3.02 within 3 %. 3. A free body gives `Unconstrained` naming it. 4. Two load cases with equal constraints factor once (counter). 5. Criterion bench `budget/fem-static-500k` (≈ 500 k DOF TET10 bracket, mesh from cache): assemble + factor + solve + recover < 30 s (vision), with the split recorded in note §8.1. 6. Result bit-identical across two runs (determinism). |
| FEM-05 | Simulation workspace UI: workspace + toolbar, study browser tree (`BrowserProvider`), new-study dialog, study materials dialog, constraint and load dialogs for fixed, pin, frictionless, symmetry, force, pressure, bearing, gravity (QUESTIONS #3 first set), glyph overlays, edit/suppress/delete, load-case switcher | UI-02, UI-04, UI-05, RND-07, FEM-01, FEM-13 | P1 | Todo | 1. Kittest: create a static study, fix one face, add a 100 N force on another → the study record contains exactly those items (one undo entry each). 2. Glyphs (arrows for force, hatching for fixed, rings for pin) follow the target faces after an upstream edit (snapshot). 3. Selection filter allows only faces for pressure and only cylindrical faces for pin and bearing (kittest). 4. The browser shows an "out of date" badge after a geometry edit (kittest on FEM-01 status). 5. All dialogs accept expressions with units (`10 N`, `2 MPa`). |
| FEM-06 | Results core: `Field::nodal` for all `Quantity`s, safety factor (`VonMisesYield`, `MaxPrincipalUltimate`, P1 directional rule), `FieldSurface`, `probe`, `TetFieldSampler`/`resample_grid`, fringe display via RND-08 (colormap, range, bands, deformation, min/max markers) | FEM-04, RND-08 | P1 | Todo | 1. Von Mises, principal and max shear of random tensors match a reference implementation within 1e-12. 2. Safety factor on the cantilever equals σ_y / max σ_vM; a material without yield falls back to UTS with a warning. 3. `probe` and `TetFieldSampler` reproduce a quadratic field exactly (1e-10) inside TET10s and return `None` outside. 4. `resample_grid` of 128³ on a 500 k DOF result < 2 s. 5. RND-08 readback of the fringe on the cantilever matches `colormap_rgb` of the CPU value within 2/255 at 100 sampled pixels. |
| FEM-07 | Modal: consistent mass (11-pt rule), shift-invert Lanczos via CMP-04, rigid-mode handling (σ < 0), participation factors and effective mass, `solve_modal`, mode-shape display (animated deformation) | FEM-04, FEM-06, CMP-04 | P1 | Todo | 1. Free-free bar: 6 modes < 0.01 Hz, first bending (β₁L = 4.730) and first axial within 1 %. 2. NAFEMS FV32 modes 1–6 within 3 %. 3. Total effective mass in each direction ≥ 90 % of the model mass when 30 modes are requested on the cantilever. 4. 500 k DOF, 8 modes < 60 s (bench `budget/fem-modal-500k`). |
| FEM-08 | GPU iterative static path: p-multigrid preconditioner (TET10 Chebyshev–Jacobi smoother, TET4 coarse Cholesky), operators and preconditioner generic over `R: GpuReal`, run through S15 `solve_gpu` (f64 kernels where `SHADER_F64`, else f32 plus f64 iterative refinement), `SolverChoice::Auto` switch from `Estimate` | FEM-04, CMP-08 | P1 | Todo | 1. On the FEM-04 golden models the GPU result matches the direct one within 1e-6 (relative energy norm). 2. 2 M DOF static (mesh cached) < 60 s on the reference GPU (bench `budget/fem-static-2m`), iteration counts recorded in note §8.3. 3. `Auto` picks GPU when `direct_bytes` exceeds 60 % of free RAM (unit test with a fake budget). 4. Device loss mid-solve returns an error, not a hang. 5. Both precision paths pass item 1: f32 + refinement on any device (CI WARP included) and f64 via S15's local f64 gate (`AMG_COMPUTE_PRECISION=f64`; skipped with a log line on devices without `SHADER_F64`). |
| FEM-09 | Steady-state thermal: scalar TET10 conduction, `ThermalCase` BCs (temperature, heat flux, heat source, convection, radiation by Picard iteration), `solve_thermal`, temperature and heat-flux results and display | FEM-04, FEM-06 | P2 | Todo | 1. Slab with fixed end temperatures: linear profile within 1e-8 K. 2. Pin fin with convection, adiabatic tip: tip temperature within 1 % of cosh(m(L−x))/cosh(mL). 3. Radiation-only plate converges in ≤ 20 Picard iterations to the analytic balance within 1 %. 4. Study UI gains the thermal BC dialogs. |
| FEM-10 | Linear buckling: geometric stiffness from a static pre-stress, generalised eigen (CMP-04 with LDLᵀ), buckling factors and shapes, study type and UI | FEM-07 | P2 | Todo | 1. Fixed-free column L/r = 100: first factor within 2 % of π²EI/(4L²). 2. A plate in shear gives a positive first factor and the expected wrinkle shape (snapshot). 3. Tensile pre-stress reports "no buckling in range" instead of negative factors. |
| FEM-11 | Nonlinear static: Newton–Raphson with load stepping and cut-back, total Lagrangian large displacement, J2 plasticity with bilinear hardening (radial return, consistent tangent), plastic strain results | FEM-04, FEM-06 | P2 | Todo | 1. Elastica cantilever PL²/EI = 2: tip deflection/L within 2 % of 0.4934. 2. Thick cylinder, elastic–perfectly plastic: collapse pressure within 2 % of (2σ_y/√3) ln(b/a). 3. Uniaxial bar with bilinear hardening matches the analytic stress–strain curve within 0.5 %. 4. Quadratic Newton convergence observed (residual ratio log table recorded). |
| FEM-12 | Event simulation (explicit dynamics, drop test): central differences, HRZ lumped mass, Courant step, mass scaling, GPU internal forces, time-history results and animation | FEM-11, CMP-05 | P3 | Todo | 1. Elastic wave in a bar: speed within 2 % of √(E/ρ); total energy drift < 1 % over 10 transits. 2. Drop of a block on a rigid plane: rebound velocity matches the elastic impact estimate within 5 %. 3. 1 M elements, 10 k steps < 10 min on the GPU. |
| FEM-13 | Shared load-case model: `amg_fem::model::{Target, Constraint, Load, LoadCase, ContactPair}` types, `amg_features::sim::{LoadCaseDef, BcDef, SimRef}`, `resolve_load_case` (targets → `Target`), `target_triangles`; reused by S14 | FEM-01, DOC-06 | P1 | Todo | 1. All P1 constraint and load variants round-trip serde (proptest). 2. `resolve_load_case` on the FEM-03 fixtures returns the expected `Target`s and a diagnostic per unresolved ref. 3. `target_triangles` of a face target has the face's area within the tessellation bound. 4. A GEN-05-style test maps a load case to voxel boundary cells using only this API (no `amg-fem` mesh types). |
| FEM-14 | TET10 promotion and snapping (`promote_tet10`, `SurfaceProjector`), curved-element validity and straightening, local sizing (per-face, curvature, small features) via background size field, `QualityReport`, mesh cache (`MeshCache`) | FEM-02, BREP-09 | P1 | Todo | 1. Sphere r = 10 meshed at h = 2: TET10 volume error < 0.1 % (vs > 1 % for TET4). 2. 0 inverted elements after promotion over the corpus; straightened count recorded. 3. A per-face local size of h/4 gives a mean facet edge on that face within 20 % of the target. 4. Default model-based size (10 %) on a 100 mm cube gives a mean edge of 10 mm ± 20 %. 5. Re-solving with unchanged geometry and mesh settings performs 0 meshing calls (cache counter); changing a load does not invalidate the mesh. |
| FEM-15 | Load and constraint discretisation: `map_target` regions, `ConstraintMap` (local frames, fixed/per-axis, pin, frictionless, symmetry, prescribed, conflict rules), `load_vector` (force, pressure, bearing, remote force and moment as distributing coupling, gravity, linear acceleration, rotation) | FEM-03, FEM-14 | P1 | Todo | 1. Each load's nodal resultant equals the requested force and moment within 1e-10 (relative), for flat and curved targets. 2. Bearing load resultant equals F and the pressure is zero on the unloaded half. 3. Pin with only radial fixed lets a shaft rotate (zero reaction torque) and slide axially. 4. Symmetry on a half model gives the same deflection as the full model within 0.5 % (golden, needs FEM-04 to run; the operator test runs here). 5. Conflicting constraints on one node produce one warning naming both and follow the precedence in note §6.2. |
| FEM-16 | Bonded contacts: `find_contacts` (tolerance, opposed normals, AABB tree), node-to-surface ties in `ConstraintMap`, contact UI (auto, manage, exclude) | FEM-15, FEM-05 | P1 | Todo | 1. Two bonded blocks give the single-block tip deflection within 0.5 % (non-matching meshes). 2. Auto detection on a 3-body assembly fixture finds exactly the 2 touching pairs, none for a 0.5 mm gap at default tolerance. 3. Patch test across a planar tie with non-matching meshes: constant stress within 1e-6. 4. Unprojected slave nodes are counted and reported. |
| FEM-17 | Study run job: `RunStudy` (mesh cache → `FeModel` → solve → `ResultsHeader` + blob → `SetExt`), progress per phase, cancel, `load_results`, result blob `store::{write, read}`, "save without results" | FEM-01, FEM-04, FEM-06, FEM-14 | P1 | Todo | 1. A cantilever study solves from a `Document` as a job; progress reaches 1.0 through ≥ 4 named phases; cancel in each phase returns `Cancelled` within 1 s and leaves the record unchanged. 2. Save→load keeps results; `load_results` returns fields equal to the in-memory ones (bit-identical f64). 3. The result blob of a 500 k DOF static model is ≤ 60 MB in the container. 4. An edit during the solve marks the new results `OutOfDate` on arrival (hash check). 5. Undo snapshots do not hold blob bytes (memory counter). |
| FEM-18 | Mesh and solve UI: mesh settings dialog (model-based %, absolute, local controls), mesh preview with quality colouring, quality report panel, pre-check indicator, solve button with job panel integration, error toasts mapped to bodies/BCs | FEM-05, FEM-17 | P1 | Todo | 1. Kittest: pre-check shows the missing-material and missing-constraint diagnostics from FEM-01 and turns green when fixed. 2. Mesh preview shows element edges and colours poor elements (snapshot). 3. A solve runs without blocking the UI (no frame > 16 ms in the trace during a 500 k DOF solve). 4. `Unconstrained` from the solver highlights the named body. |
| FEM-19 | Results tools UI: result type picker, legend and range controls, min/max markers, probes (click, list, export CSV), deformation scale and animation, load-case/mode switcher, reaction-force table, HTML report export (setup, mesh, materials, results images, tables) | FEM-06, FEM-17, FEM-18 | P1 | Todo | 1. Kittest: switching quantity re-binds the field without re-uploading geometry (RND-08 counter). 2. A probe at the cantilever root reports the same von Mises as `probe` within 1e-6. 3. Report export of the cantilever study writes one self-contained HTML file containing the setup tables, the min SF, max displacement and 3 images. 4. Mode switcher shows each mode's frequency. |
| FEM-20 | Thermal stress: temperature field (uniform ΔT or from a FEM-09 solve) → thermal strain body load, `ThermalStress` study type | FEM-09, FEM-15 | P2 | Todo | 1. NAFEMS LE11: σzz at A within 3 % of −105 MPa. 2. A free bar heated by ΔT expands by αLΔT within 1e-6 with zero stress (1e-6 MPa). |
| FEM-21 | Nonlinear contact: separation, sliding (frictionless) and frictional contact by node-to-surface augmented Lagrangian, contact pressure result, contact kinds in the UI | FEM-11, FEM-16 | P2 | Todo | 1. Hertz sphere on flat: contact radius and peak pressure within 5 %. 2. A block on an inclined plane sticks for μ > tan θ and slides for μ < tan θ. 3. Two blocks with `Separation` under tension separate (zero contact force), under compression transmit the load. |
| FEM-22 | Printed-material models: `ElasticModel::Anisotropic` from MAT-04 (transversely isotropic, axis 3 = build direction from the S16 print setup or a study override), anisotropic TET10 stiffness, Tsai-Wu safety factor | FEM-04, FEM-06, MAT-04, AM-01 | P2 | Todo | 1. A transversely isotropic bar loaded along axis 3 deflects by FL/(E_t A) within 1 %; rotating the build direction 90° gives FL/(E_p A). 2. Tsai-Wu SF = 1 at the uniaxial Z strength (1e-9). 3. The study shows the build direction used and warns when a body is in no print setup (falls back to +Z). |
| FEM-23 | Adaptive mesh refinement: Zienkiewicz–Zhu error estimate, refine size field where error is high, repeat until the max von Mises changes < 5 % or N passes (default 3), convergence plot | FEM-06, FEM-14, FEM-17 | P2 | Todo | 1. Plate with hole: from a coarse start, AMR reaches Kt within 3 % of 3.02 in ≤ 3 passes. 2. The convergence plot shows the per-pass max value and DOFs. 3. Cancel between passes keeps the last completed result. |
| FEM-24 | Study geometry simplification: per-study exclusion of small features (fillets, holes below a size) and bodies via BREP-16 defeature on a study-private copy; the design is not touched | FEM-17, BREP-16 | P2 | Todo | 1. Removing fillets < 2 mm on a bracket fixture leaves the design's bodies unchanged (hash) and meshes the simplified copy. 2. BCs on removed faces give a diagnostic. 3. Out-of-date tracking includes the simplification settings. |
| FEM-25 | CalculiX `.inp` export (C3D10, ties, BCs, loads, static and frequency steps) + differential tests against CalculiX (`ccx`, GPL, test-only, optional in CI) | FEM-04, FEM-07, FEM-16 | P2 | Todo | 1. The exported LE10 deck runs in ccx 2.21+ and its σyy at D agrees with ours within 1 %. 2. On 3 corpus parts, max displacement and max von Mises agree with ccx within 2 % (same mesh). 3. The export uses `VOIGT_ORDER` mapping explicitly (unit test on an anisotropic material line, MAT-04). |

## Testing
- **Unit:** quadrature exactness; TET10 shape functions (partition of unity, Kronecker delta);
  element stiffness symmetry and rigid-body null space (6 zero eigenvalues per free element);
  constraint maps (resultants, frames); serde of every record type (proptest); result blob
  round-trip.
- **Golden** (`tests/golden/fem/`, note §10): patch test, cantilever, NAFEMS LE1, LE10, LE11,
  FV32, plate with hole, free-free bar, Euler column, elastica, thick cylinder, Hertz, bar wave,
  bonded blocks, symmetry half model. Each case stores model, expected value and tolerance; CI
  runs the small ones on every PR and the large ones nightly.
- **Corpus:** `testdata/fem/corpus/` meshing success rate, volume error, tag ambiguity, times
  (FEM-02, FEM-14).
- **Differential:** CalculiX on the same mesh (FEM-25), optional.
- **Fusion comparison:** owner-exported Fusion results for 3 corpus parts, informative (10 %).
- **UI:** kittest flows for study setup, pre-check, solve and results (FEM-05, FEM-18, FEM-19).
- **Benchmarks** (criterion, `budget/` prefix): `fem-static-500k`, `fem-modal-500k`,
  `fem-static-2m`, `fem-mesh-120k-tets`, `fem-resample-128`.

## Performance targets
| Scenario | Target | WP |
|---|---|---|
| Linear static, ~500 k DOF, assembly + solve + recovery with the mesh cached (vision budget; QUESTIONS #70) | < 30 s (assembly ≤ 3 s, factor + solve ≤ 20 s, recovery ≤ 3 s) | FEM-04 |
| Tet meshing of the same model (≈ 120 k TET10), own budget, cached (QUESTIONS #70, decided) | < 30 s | FEM-02, FEM-14 |
| Modal, 500 k DOF, 8 modes | < 60 s | FEM-07 |
| Static, 2 M DOF, GPU iterative | < 60 s | FEM-08 |
| Field resample 128³ | < 2 s | FEM-06 |
| UI during any solve | no frame > 16 ms (vision) | FEM-18 |
| Explicit, 1 M elements, 10 k steps | < 10 min | FEM-12 |

## Open questions
1. Owner: which load types are used most? Prioritize UI for them.
   **Answered 2026-10-20:** first-class UI for fixed/pinned, force, pressure, bearing/pin load,
   gravity and symmetry; remote force, moment and thermal loads follow.
2. Orthotropic / printed-material models (layer-dependent stiffness) — valuable for AM, P2?
   **Answered 2026-10-20:** yes, P2. Isotropic first (P1); then a transversely isotropic material
   with build direction taken from the S16 print setup, plus a directional failure criterion
   (Tsai-Wu or reduced Z allowables). Matters mainly for FFF; resin treated as isotropic.
   **Applied 2026-11-08:** FEM-22.
3. (from S10, 2026-10-20) FEM-06 supplies a surface mesh plus per-vertex scalar/displacement fields to RND-08 `Shading::Field`.
   **Applied 2026-11-08:** `FieldSurface` (note §9.4).
4. (from S17, 2026-10-20) The orthotropic/printed FEM WP should depend on MAT-04. P1 solvers use `Elastic::as_isotropic()` or an explicit `isotropic_equivalent(IsoPolicy)` with a UI warning. Unassigned bodies are an error (no default material). Voigt order is `(11,22,33,23,13,12)` (`VOIGT_ORDER`), not the CalculiX order. Material axis 3 = build direction from the S16 print setup (plate +Z, `d_local = R⁻¹·Z`, `docs/notes/am-build-prep.md`).
   **Applied 2026-11-08:** FEM-22 depends on MAT-04; `SolidMaterial::from_material`; FEM-01
   pre-check; note §1.
5. Does the 30 s static budget include meshing? **Resolved 2026-11-09 (owner, QUESTIONS #70):** no; the
   30 s covers assembly + solve (+ recovery) with a cached mesh, meshing has its own < 30 s budget.
   Vision budgets updated.
6. Default mesher for clean input? **Resolved 2026-11-09 (owner, QUESTIONS #71):** `MesherChoice::Auto`:
   TetGen for clean closed input (feature `tetgen`, on in the owner's build), fTetWild otherwise.
7. Gmsh/HXT as a third mesher? **Resolved 2026-11-09 (owner, QUESTIONS #72):** not now; decide on
   FEM-02's measurements (an ADR would be needed).
8. (resolved 2026-11-08) Where results live: in the `.amg` `ext/` blobs, with "save without
   results". Fusion keeps them in the cloud; a sidecar file would break "one file" sharing.
9. (resolved 2026-11-08) Remote force default: distributing coupling (RBE3-style, no added
   stiffness), because a rigid coupling stiffens the face and overstates stiffness on thin
   parts; the rigid variant is later.

10. (from S08, 2026-11-08) IMP-07 (stress-driven lattice density) reads FEA results as a regular grid: FEM-06 should resample a chosen result (von Mises, displacement magnitude, safety factor) onto corner values of a box and call `amg_implicit::SampledGrid::from_dense(origin, cell_size, dims, values)`.
11. (from S15, 2026-11-08) S15 is Refined and covers every CMP request in Interfaces (`SymCsr`, `SymbolicFactor`, `Cholesky`/`Ldlt`, `solve_many`, `PivotReport`, `FactorEstimate`, `Inertia`; `lanczos` with a `ShiftInvert` callback; `GpuCsr`, `pcg_gpu`, `Chebyshev`, `refine`, trait `GpuPreconditioner`). The f32 PCG and preconditioner trait are now CMP-08 (CMP-05 adds geometric multigrid on top), so FEM-08 and FEM-12 could depend on CMP-08 instead of CMP-05. LU is not provided (no consumer). Sizes for benches can come from `amg_compute::testprob`.

## Refinement log
### 2026-11-08 - S13-refine (lane F refinement)
- **Read:** AGENTS.md, vision, architecture, ADRs (ADR-004 wgpu/CUDA, ADR-006 faer, ADR-008
  fTetWild/TetGen, ADR-012, ADR-014), roadmap, QUESTIONS, S01 (§11 extension records, naming,
  DOC-07 log: `ExtRegistration` not yet built), S02 (BREP-08 `TessMesh`, BREP-09
  `project_point_on_face`), S07 (`MeshSpatial`, `FaceGroups`, `Primitive`), S10 (RND-08
  `Shading::Field`, `Deformation`, `legend`), S11 (`UiModule`, `Registrar`, `Tool`), S14, S15
  (Draft), S16 (print-setup record pattern), S17 (`Elastic`, `Strength`, `VOIGT_ORDER`,
  `IsoPolicy`), S18 (SCR-12 blocked on S13), S08 (IMP-07). Code: `amg-fem` is an empty crate
  with the right deps (core, jobs, compute, mesh, materials); `amg-doc` has `ExtRecord`,
  `Command::{AddExt, SetExt}`, `ExtKey { kind, id }` (doc example `"fem.study"`), `Evaluated`,
  `EntityRef`; `amg-features::mesh` has `MeshEntityRef` and the `register_feature!`/`link()`
  pattern; `amg-jobs::Job`.
- **Research:**
  - Fusion behaviour: study types, "same as model" study materials, bonded default contacts with
    auto detection, mesh defaults (model-based 10 %, parabolic elements, min 20 %, ratio 1.5,
    curvature refinement), adaptive refinement, safety-factor targets, pre-check
    ([Autodesk blog: static stress simulations](https://www.autodesk.com/products/fusion-360/blog/static-stress-simulations-autodesk-fusion/),
    [AU class: Fusion simulation expert in 60 minutes](https://static.au-uw2-stg.autodesk.com/AUME717_-_Become_a_Fusion_Simulation_Expert_in_60_Minutes.pdf),
    [Autodesk learning: refining meshes](https://autodesk.com/learn/ondemand/course/fusion360-simulation-static-stress-analysis-cert-prep/unit/5QlagVblQKfvE2my5Zt0Qv),
    a published Fusion study report with the mesh settings block).
  - fTetWild ([repo](https://github.com/wildmeshing/fTetWild), [paper](https://arxiv.org/abs/1908.03581)):
    MPL-2.0, envelope ε = diag/1000 by default, needs GMP (MPIR on Windows), outputs `.msh`;
    its boundary is re-meshed, so patch tags need a transfer step (note §4.2). The in-process
    API and `--bg-mesh` sizing are to be confirmed by FEM-02 (note §12).
  - NAFEMS targets: LE1 σyy(D) = 92.7 MPa ([Altair OS-V 0010](https://help.altair.com/hwsolvers/os/topics/solvers/os/nafems_test_problem_le1_r.htm)),
    LE10 σyy(D) = −5.38 MPa ([Abaqus benchmark LE10](https://abaqus-docs.mit.edu/2017/English/SIMACAEBMKRefMap/simabmk-c-le10.htm),
    [COMSOL thick plate](https://doc.comsol.com/6.1/doc/com.comsol.help.models.sme.thick_plate/thick_plate.html));
    FV32, LE11 values from *The Standard NAFEMS Benchmarks* (TNSB Rev. 3, 1990).
  - Solvers: faer supernodal Cholesky (ADR-006); shift-invert Lanczos (Ericsson–Ruhe);
    p-multigrid for quadratic elements as the GPU preconditioner because CMP-05's geometric
    multigrid assumes structured grids; precision follows the ADR-004 note (f64 kernels where
    `SHADER_F64`, else f32 with f64 refinement; S15 OQ 1). Tie constraints follow CalculiX/Abaqus `*TIE`; remote loads follow the RBE3
    distributing-coupling idea. `fenris` (Rust FEM, MIT/Apache-2.0) is a reference for TET10
    quadrature, not a dependency.
- **Decided:**
  - `amg-fem` is document-agnostic (patch ids, not `EntityRef`); `amg-features::sim` does the
    document side; geometry projection comes in through `SurfaceProjector`. This keeps the
    architecture's `amg-fem` dependency list unchanged.
  - New `-sys` crate `amg-tetwild-sys` (lane F). Tag transfer is backend-independent.
  - One constraint mechanism `u = T û + g` for every linear constraint and bonded ties.
  - Study = `fem.study` `ExtRecord` with whole-record `SetExt` (same as S16). Results in `ext/`
    blobs (`AMGFEMR1`).
  - Mesh defaults copy Fusion. Default 8 modes (Fusion's default).
- **WPs:** FEM-01..13 keep their IDs and meaning. Splits: FEM-02 → FEM-02 (fTetWild, tags,
  global size) + FEM-14 (TET10, local sizing, quality, cache); FEM-03 → FEM-03 (target mapping)
  + FEM-15 (discretisation); FEM-06 → FEM-06 (results core, field export, fringe) + FEM-19
  (results tools UI); FEM-09 → FEM-09 (thermal) + FEM-20 (thermal stress); FEM-11 → FEM-11
  (plasticity, large deflection) + FEM-21 (nonlinear contact). New: FEM-16 (bonded contacts),
  FEM-17 (run job, persistence), FEM-18 (mesh/solve UI), FEM-22 (printed materials, from OQ 2/4),
  FEM-23 (adaptive refinement), FEM-24 (simplification), FEM-25 (CalculiX export).
  Dependency changes: FEM-13 now depends on FEM-01 and DOC-06 (it defines the types FEM-03
  uses), and FEM-03 on FEM-13. FEM-01 adds DOC-05/DOC-09/MAT-06. GEN-01 (FEM-01), GEN-05
  (FEM-13), GEN-07 (FEM-04), IMP-07 (FEM-06), AM-09 (FEM-04) and SCR-12 (FEM-01) keep valid
  meanings; SCR-12 should add FEM-17 (request to S18).
- **Questions:** #70 (budget scope), #71 (default mesher), #72 (Gmsh/HXT). Resolved here: result
  storage location, remote-force coupling type (OQ 8, 9).
- **Roadmap:** Phase 3 F gains FEM-13, FEM-14; Phase 4 F gains FEM-15..19; Phase 5 F gains
  FEM-20..25; lane F crate list gains `amg-tetwild-sys`.

### 2026-11-09 - owner answers moved in
- QUESTIONS #81 (precision policy): FEM-08 and the p-multigrid are generic over `R: GpuReal`; f64 kernels where the device has `SHADER_F64`, else f32 plus f64 CPU iterative refinement via `solve_gpu`; FEM-08 Done-when 5 covers both paths. Wording "f32 only / DX12 has no f64" removed.
- QUESTIONS #70 (30 s = assembly + solve, cached mesh; meshing < 30 s own budget; vision budgets rewritten), #71 (`Auto` mesher: TetGen for clean closed input), #72 (no Gmsh/HXT now; FEM-02 Done-when 3 reports whether it is needed). Open questions 5-7 resolved.

## Interface changes
None yet (spec not implemented). Requests this spec makes of other specs are listed under
Interfaces → "Needed from other specs".
### 2026-11-08: from the S14 refinement (same lane, additive)
- The FEM-05 constraint and load dialogs take a `&mut LoadCaseDef` and a target filter, so
  `amg-ui::generative` (GEN-13) reuses them with "preserve bodies only".
- GEN-10 (shape optimisation, P2) adds a `sim.shape_optimization` command to `amg-ui::sim`; the
  study itself is a `gen.study` record (S14), not a new `StudyType`.
- GEN-17 builds a `SurfaceInput`/`FeModel` for converted outcome bodies directly with `amg-fem`
  types and `target_triangles` (FEM-13). No new API.
