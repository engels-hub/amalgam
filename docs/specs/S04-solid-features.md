# S04 — Solid modeling features

| | |
|---|---|
| **Status** | Refined |
| **Priority** | P0 / P1 |
| **Crates** | `amg-features::solid`, `amg-features::dialog` (the shared dialog-schema module S11 renders) |
| **Lane** | A |
| **Depends on** | S01 (DOC-06 naming framework, DOC-12 preview recompute, `Feature` trait); S02 (BREP-03/04/05/06/07/10/14/15/16, BREP-11 for threads and coils; requests R-S04-1..9); S03 (SK-06/SK-15 profiles, SK-19 `SketchGeom`, SK-10 text); S05 (CON-03 reference resolution); S20 (KH-03 fillet chain, KH-08/KH-18 loft builder, `HardenedKernel`); S06 (ASM-01/02 for component operations) |
| **Parity** | [Solid modeling](../03-fusion-parity.md#solid-modeling-s04) |

## Summary
Every Fusion "Solid" tab feature as a timeline `Feature` in `amg-features::solid`. A feature
takes inputs (profiles, faces, edges, bodies, construction geometry, `Expr` values), builds a
**tool** through `BrepKernel`, applies a **body operation** (new body, join, cut, intersect;
new component through the dialog), and returns bodies with persistent names built by the
rules of [persistent-naming.md](../notes/persistent-naming.md) §5. Each feature also
registers a declarative **dialog schema** (`amg_features::dialog`, the shape S11 asked for in
its Interfaces §B), so S11 builds the dialog, command, toolbar entry and timeline entry
generically, and S18 builds Python bindings from the same schema. Reasoning and algorithms
are in [docs/notes/solid-features.md](../notes/solid-features.md).

## Scope
**In:**
- The shared framework: `BodyOp` and participants, extents, profile → face conversion,
  tangent-chain helpers, the `SolidTool` trait (tools for pattern/mirror), error mapping to
  highlighted entities, preview behaviour, `amg_features::dialog`.
- Features: extrude (incl. thin), revolve, primitives (box, cylinder, sphere, torus, coil,
  pipe), fillet (constant, chord, variable, setback), chamfer, shell, draft, scale, thicken,
  combine, split body, split face, silhouette split, pattern (rectangular, circular, path;
  bodies, faces, features, components), mirror, move/copy, align, offset face, replace face,
  delete face, press-pull (command dispatch only), sweep, loft (dialog and feature on S20's
  builder), hole and thread (with standards tables), rib, web, emboss, boundary fill, patch,
  stitch, unstitch, reverse normal; later ruled/extend/trim/untrim, rule fillet, derive,
  sheet metal.
- Golden tests in `tests/golden/solid/`, the Fusion-parity harness and the recompute budget
  bench.

**Out:**
- Kernel operations themselves (S02) and their robustness strategies (S20). S04 only asks
  for additive kernel calls (Interfaces, "Kernel calls").
- The naming *scheme* and resolver (S01 DOC-06, the naming note). S04 applies the rules.
- Dialog rendering, manipulators and viewport highlight (S11 UI-05/UI-06, S10).
- Construction planes/axes/points (S05 CON-01/02); S04 resolves them through CON-03.
- Mesh features and mesh booleans (S07). Solid features reject mesh participants.
- Form/SubD (S09), "Create form" (S09).
- Physical material and appearance (S17), though Fusion lists them in the same toolbar.
- G2 (curvature-continuous) fillets: OCCT can't build them; QUESTIONS #55.
- Full-round fillet: not in the parity matrix; added when the owner asks for it.

## Design

### 1. Module layout and kinds
- `amg-features::solid` with one submodule per feature group (`common`, `extent`, `extrude`,
  `revolve`, `primitive`, `fillet`, `shell`, `draft`, `transform`, `combine`, `split`,
  `pattern`, `face_ops`, `sweep`, `loft`, `hole`, `thread`, `rib`, `emboss`, `surface`,
  `standards`). It follows the existing `amg-features::mesh` pattern: plain serde structs with
  `#[serde(deny_unknown_fields)]`, `amg_doc::register_feature!(Type, "solid.<name>", version
  = 1)`, and a `pub fn link() {}`.
- `amg-features::dialog` is a sibling top-level module (S11 §B: the schema types must live
  below `amg-ui`). Lane A owns it through SOL-01. Other lanes (mesh, construct, sim, AM)
  register their own schemas with it.
- Kinds: `solid.extrude`, `solid.revolve`, `solid.box`, `solid.cylinder`, `solid.sphere`,
  `solid.torus`, `solid.coil`, `solid.pipe`, `solid.fillet`, `solid.chamfer`, `solid.shell`,
  `solid.draft`, `solid.scale`, `solid.thicken`, `solid.combine`, `solid.split_body`,
  `solid.split_face`, `solid.silhouette_split`, `solid.pattern`, `solid.mirror`,
  `solid.move`, `solid.align`, `solid.offset_face`, `solid.replace_face`,
  `solid.delete_face`, `solid.sweep`, `solid.loft`, `solid.hole`, `solid.thread`,
  `solid.rib`, `solid.web`, `solid.emboss`, `solid.boundary_fill`, `solid.patch`,
  `solid.stitch`, `solid.unstitch`, `solid.reverse_normal`. Kinds are stable forever.
- A naming-rule change bumps the feature's `version` and adds a migration (naming note §12).

### 2. Inputs, references and resolution
- Profiles are `EntityRef::Profile { sketch, profile, fingerprint }` (resolved with
  `ctx.resolve_profile`) or planar faces `EntityRef::Topo` (kind `Face`).
- Picked faces, edges and vertices are `TopoRef`s with a selection key (naming note §5.1).
  Set-valued fields (fillet edges, shell faces, draft faces, pattern faces) resolve with
  `SetPolicy::DropMissing`; single-valued ones (axis, extent target, mirror plane, split
  tool) with `AllOrError`.
- Axes, planes and points are `construct::RefInput` (origin entity or `EntityRef`), resolved
  with S05's `construct::resolve::resolve_input` (CON-03) to `RefGeom`, then
  `construct::math::{line_of, plane_of, point_of, axis_of}`.
- `inputs()` lists every reference: `Dependency::Body` per participant,
  `Dependency::Entity` per ref, `Dependency::Sketch` per profile's sketch,
  `Dependency::Construction` per construction ref, `Dependency::Feature` for pattern seeds.
- `exprs()`/`exprs_mut()` list every `Expr` with its dimension (`Dim::LENGTH`, `Dim::ANGLE`,
  `Dim::NONE` for counts and fractions).

### 3. Body operations
Full rules in the note §2. Summary:
- `BodyOp` is `NewBody | Join | Cut | Intersect`. "New component" is a dialog option that
  wraps the feature in a transaction with `AddComponent` (SOL-24), not a stored value.
- `participants: Vec<BodyId>` is stored; the dialog fills it with the bodies the preview tool
  touches (Fusion's "Objects to cut"). Missing participants are followed through
  `Evaluated::successors` or dropped with a warning.
- Join keeps the first touched participant's `BodyId` and marks the other touched ones
  `Consumed { into }`; with nothing touched it makes a new body and warns. Cut splits pieces
  into `BodyId { self, k }` with `Created { derived_from }` (S02 `OpResult::per_solid`); with
  nothing touched it fails (`NothingToCut`). Intersect deletes empty participants and fails if
  all are empty.
- Every boolean result is simplified with `unify_same_domain` (faces + edges), composed into
  the history before naming (naming note §5.1, QUESTIONS #48).
- Participants must be solid bodies of the feature's context component (`WrongBodyKind`,
  `OtherComponent`).

### 4. Extents (extrude, revolve, hole, rib)
Full algorithm in the note §3.
- Direction: one side, two sides (independent extent and taper per side), symmetric (half or
  whole length).
- Extent: distance; to object (plane, face, body, vertex; offset; extend faces); all.
- Start: profile plane, offset, or a planar object.
- "To object" on a parallel plane or planar face is an exact distance. Otherwise S04 builds a
  finite long prism and splits it with the target (extended to its untrimmed surface if
  `extend`), keeping the piece that holds the profile. "All" uses the context component's
  bounding boxes. No infinite solid ever reaches a boolean.
- Revolve uses the same `Extent` enum with angles: `Distance` is an angle, `ToObject` splits a
  full revolution with the target, and `Full` is 360°.

### 5. Tools, pattern and mirror
- Each additive/subtractive feature implements `SolidTool`: it returns its tool solids, their
  names (by its own rules) and its `BodyOp` target. A `ToolRegistration` (inventory) lets
  `solid.pattern`/`solid.mirror` call any seed's tool from its `FeatureBox`.
- Feature pattern ("identical" compute): build the seed's tool once in the pattern's
  context, place instance `k` with `Shape::transformed`, rename it with
  `BodyNaming::instanced(self, k)`, then run **one** N-ary boolean. "Adjust" compute (seed
  inputs moved per instance) is P1 (SOL-23).
- Body pattern/mirror copies bodies as `BodyId { self, k }`. Mirror uses S02's `mirrored`
  (R-S04-6), because a reflection must flip face orientations.

### 6. Preview
`ComputeMode::Preview` runs the same code with S20's `RetryPolicy::interactive()` and no
BRepCheck validation, and it still builds names (note §5). Typical previews must finish in
< 100 ms: a 20-edge profile extrude-cut into a 200-face body, a 10-edge fillet on a 200-face
body, a shell of a 100-face body. Benches: `budget/sol-extrude-preview`,
`budget/sol-fillet-preview`, `budget/sol-shell-preview`.

### 7. Failures
- `SolidError` (thiserror) for every S04 failure; it converts to
  `FeatureError::InvalidValue` (input problems) or `FeatureError::Kernel` (kernel failures).
- Offending sub-shapes from `KernelError` become `EntityRef`s via `topo_ref_at` and go into
  `Diagnostic::entities`, so the failing edge or face is highlighted. Hints come from
  `amg_core::Diagnostic`.
- Fillet and chamfer use raw BREP-06 in SOL-05. SOL-20 moves them onto S20's
  `robust::fillet::fillet` and shows `FilletDiagnosis` (failing edges, largest workable
  radius).

### 8. Standards tables
`solid::standards` holds thread and hole data in `crates/amg-features/data/threads/*.toml`
(embedded, parsed once). ISO metric coarse and fine (ISO 261 sizes M1.6–M64; ISO 68-1
profile; tap drill; ISO 273 fine/medium/coarse clearance) come with SOL-12. UNC/UNF come with
SOL-25 (QUESTIONS #54). A unit test recomputes every derived column from the ISO 68-1
formulas (note §7). Print allowances: QUESTIONS #53.

### 9. Press-pull
Press-pull is not a feature. It is a command that starts another feature's dialog from the
selection: a face → `solid.offset_face`, an edge → `solid.fillet`, a profile →
`solid.extrude`. It is registered through `dialog::DispatchRegistration`, which UI-05
renders (S11 request, SOL-09).

## Interfaces
Public API of `amg-features::dialog` and `amg-features::solid`. Bodies omitted. Types from
S01 follow the naming note §10 (DOC-06's final shapes); types from S02 follow its Interfaces.

```rust
// ===== amg_features::dialog (SOL-01; S11 Interfaces §B, adopted as is) ======================
// Everything in S11 §B: EntityFilter, FieldKey, DialogSchema, FieldSpec, FieldKind, ChoiceOption,
// ChoiceStyle, Condition, FieldValue, FieldValues, FieldErrors, ManipKind, ManipAnchor,
// DialogInit, CommandMeta, DialogRegistration, inventory::collect!(DialogRegistration).
// Additive extensions owned by S04:
pub struct DialogRegistration {
    pub kind: &'static str,
    pub schema: fn() -> &'static DialogSchema,
    pub defaults: fn(&DialogInit<'_>) -> FieldValues,
    pub read: fn(&dyn Feature, &Design) -> Option<FieldValues>,
    pub build: fn(&FieldValues, Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors>,
    pub anchors: fn(&dyn Feature, &Design, &Evaluated) -> Vec<ManipAnchor>,
    pub command: CommandMeta,
    /// NEW (S04): `Some(name)` = the user chose "New component": UI-05 commits
    /// `AddComponent { name }` + `AddFeature { context: <new component> }` in one transaction.
    /// `None` for every registration until SOL-24.
    pub new_component: Option<fn(&FieldValues) -> Option<String>>,
}
/// NEW (S04, SOL-09): a command that starts another kind's dialog chosen from the selection
/// (press-pull). UI-05 renders it like a registration without a schema.
pub struct DispatchRegistration {
    pub command_id: &'static str,                        // "solid.press_pull"
    pub dispatch: fn(&[EntityRef], &Design) -> Option<&'static str /* feature kind */>,
    pub command: CommandMeta,
}
inventory::collect!(DispatchRegistration);
pub fn registration(kind: &str) -> Option<&'static DialogRegistration>;
pub fn registrations() -> impl Iterator<Item = &'static DialogRegistration>;

// ===== amg_features::solid (SOL-01) =========================================================
use amg_brep::{BrepKernel, Shape, OpResult, SubShape, EdgeId, FaceId, geom::{Tag, Curve2d}};
use amg_doc::{BodyId, BodyNaming, Dependency, EntityRef, Expr, Feature, FeatureError,
              FeatureId, FeatureOutput, RecomputeCtx, SketchId, TopoRef, SetPolicy};
use amg_features::construct::RefInput;

pub fn link() {}

// ---- body operations (common) ----
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BodyOp { #[default] NewBody, Join, Cut, Intersect }
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OpTarget { pub op: BodyOp, pub participants: Vec<BodyId> }
impl OpTarget {
    /// `Dependency::Body` per participant.
    pub fn inputs(&self) -> impl Iterator<Item = Dependency> + '_;
}
/// A feature's tool before its body operation.
#[derive(Clone, Debug)]
pub struct ToolBody {
    pub shape: Shape,          // one solid or a compound of solids, in tool order
    pub naming: BodyNaming,    // by the producing feature's rules (Gen.feature = that feature)
    pub target: OpTarget,
}
/// Implemented by every additive/subtractive feature (extrude, revolve, primitives, sweep,
/// loft, hole, rib, web, emboss, coil, pipe).
pub trait SolidTool {
    fn tool(&self, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError>;
}
pub struct ToolRegistration {
    pub kind: &'static str,
    pub tool: fn(&dyn Feature, &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError>,
}
inventory::collect!(ToolRegistration);
/// `register_tool!(Extrude, "solid.extrude")` next to `register_feature!`.
#[macro_export] macro_rules! register_tool { /* submits a ToolRegistration that downcasts */ }
/// The seed's tool; `SolidError::NotPatternable` if its kind has no ToolRegistration.
pub fn tool_of(seed: &dyn Feature, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError>;
/// §3 rules: per-op semantics, successors, unify + compose, naming via ctx.naming_builder(),
/// per-solid split, Consumed/Created{derived_from}/Deleted body changes, warnings.
pub fn apply_body_op(ctx: &RecomputeCtx<'_>, tool: ToolBody, out: &mut FeatureOutput)
    -> Result<(), FeatureError>;
/// `unify_same_domain { faces, edges }` on `r.shape`, history composed into `r.history`.
pub fn simplify(k: &dyn BrepKernel, r: OpResult) -> amg_brep::KResult<OpResult>;

// ---- inputs (common) ----
/// The solid and names of `body` just before this feature; `WrongBodyKind` for non-solids.
pub fn input_solid<'a>(ctx: &RecomputeCtx<'a>, body: BodyId)
    -> Result<(&'a Shape, &'a BodyNaming), FeatureError>;
/// Resolved picks grouped by body, in input order (policy per §2).
pub fn resolve_subs(ctx: &RecomputeCtx<'_>, refs: &[TopoRef], policy: SetPolicy)
    -> Result<Vec<(BodyId, Vec<(SubShape, u32 /* selection key */)>)>, FeatureError>;
#[derive(Clone, Debug)]
pub enum ProfileSource {
    Region { sketch: SketchId, profile: amg_doc::stub::ProfileKey },
    Face { body: BodyId, face: FaceId, key: u32 },
}
#[derive(Clone, Debug)]
pub struct ProfileFace {
    pub face: Shape,                          // planar face
    pub plane: amg_core::Plane,               // outward normal = extrude direction
    pub edge_tags: Vec<(EdgeId, Tag)>,        // Tag = sketch entity id (§3.4 of the note)
    pub source: ProfileSource,
    pub group: u32,                           // regions of one sketch sharing edges share a group
}
/// Resolves profile regions and planar faces, converts loops (`to_curve2d`) and calls
/// `make_planar_faces` once per sketch. `NonParallelProfiles`, `NotPlanar` on bad input.
pub fn profile_faces(ctx: &RecomputeCtx<'_>, refs: &[EntityRef]) -> Result<Vec<ProfileFace>, FeatureError>;
pub fn to_curve2d(c: &amg_sketch::Curve2) -> Curve2d;           // total match (S02 §4)
pub fn tag_of(e: amg_sketch::EntityId) -> Tag;                   // Tag(u64::from(e.0))

// ---- topology helpers (common) ----
/// Edges G1-continuous with `seeds` across shared vertices (angle ≤ `angular`, R-S04-1).
pub fn tangent_chain_edges(k: &dyn BrepKernel, s: &Shape, seeds: &[EdgeId], angular: f64)
    -> amg_brep::KResult<Vec<EdgeId>>;
/// Faces G1-continuous with `seeds` across shared edges (normals sampled at 3 edge points).
pub fn tangent_chain_faces(k: &dyn BrepKernel, s: &Shape, seeds: &[FaceId], angular: f64)
    -> amg_brep::KResult<Vec<FaceId>>;

// ---- errors (common) ----
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum SolidError {
    #[error("body is not a solid")] WrongBodyKind,
    #[error("body {0:?} is in another component")] OtherComponent(BodyId),
    #[error("nothing to cut: the tool touches no participant")] NothingToCut,
    #[error("nothing to intersect: every common part is empty")] NothingToIntersect,
    #[error("profiles are not parallel")] NonParallelProfiles,
    #[error("the target does not cover the profile")] TargetMissed,
    #[error("this face type can't be extended")] ExtendUnsupported,
    #[error("the profile crosses the axis")] AxisCrossesProfile,
    #[error("the tool does not split the body")] NoSplit,
    #[error("feature kind `{0}` can't be patterned")] NotPatternable(String),
    #[error("{what} is not available in this build yet")] NotYet { what: &'static str },
    #[error("unknown thread `{0}`")] UnknownThread(String),
    #[error("{field}: {msg}")] Invalid { field: &'static str, msg: String },
}
impl From<SolidError> for FeatureError { /* Invalid/NotYet/… → InvalidValue; rest → Kernel */ }
/// KernelError → FeatureError::Kernel plus a Diagnostic whose `entities` are `topo_ref_at`
/// refs of `offending` on `body` (§7).
pub fn kernel_failure(ctx: &RecomputeCtx<'_>, body: Option<BodyId>, e: amg_brep::KernelError,
                      out: &mut FeatureOutput) -> FeatureError;

// ---- extents (SOL-01 distance; SOL-02 the rest) ----
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Extent {
    Distance(Expr),                                           // mm (extrude) or rad (revolve)
    ToObject { target: EntityRef, offset: Expr, extend: bool },
    All,
    Full,                                                     // revolve only: 360°
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExtentSide { pub extent: Extent, pub taper: Expr /* rad; revolve ignores it */ }
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Direction { #[default] OneSide, TwoSides, Symmetric { whole_length: bool } }
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub enum Start { #[default] Profile, Offset(Expr), Object { target: EntityRef, offset: Expr } }
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum ThinSide { #[default] Side1, Side2, Center }
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThinSpec { pub thickness: Expr, pub side: ThinSide }

// ---- features (one struct per kind; all `#[serde(deny_unknown_fields)]`, version 1) -----
/// solid.extrude (SOL-01 minimal, SOL-02 full, SOL-18 thin).
pub struct Extrude {
    pub profiles: Vec<EntityRef>,      // Profile regions or planar Topo faces; open sketch curves only with `thin`
    pub start: Start,
    pub direction: Direction,
    pub side1: ExtentSide,
    pub side2: Option<ExtentSide>,     // Some iff direction == TwoSides
    pub flip: bool,
    pub thin: Option<ThinSpec>,
    pub target: OpTarget,
}
/// solid.revolve (SOL-03; thin in SOL-18).
pub struct Revolve { pub profiles: Vec<EntityRef>, pub axis: RefInput, pub direction: Direction,
                     pub side1: ExtentSide, pub side2: Option<ExtentSide>, pub flip: bool,
                     pub thin: Option<ThinSpec>, pub target: OpTarget }
/// Placement shared by primitives: a plane (origin plane, construction plane or planar face),
/// an in-plane origin and a rotation about the plane normal.
pub struct PrimitiveFrame { pub plane: RefInput, pub u: Expr, pub v: Expr, pub angle: Expr }
pub struct BoxPrim      { pub frame: PrimitiveFrame, pub size: [Expr; 3], pub centered: bool, pub target: OpTarget } // solid.box
pub struct CylinderPrim { pub frame: PrimitiveFrame, pub radius: Expr, pub height: Expr, pub target: OpTarget }      // solid.cylinder
pub struct SpherePrim   { pub frame: PrimitiveFrame, pub radius: Expr, pub target: OpTarget }                        // solid.sphere
pub struct TorusPrim    { pub frame: PrimitiveFrame, pub major: Expr, pub minor: Expr, pub target: OpTarget }        // solid.torus
/// solid.coil (SOL-19).
pub enum CoilSize { RevolutionsHeight { revolutions: Expr, height: Expr }, RevolutionsPitch { revolutions: Expr, pitch: Expr },
                    HeightPitch { height: Expr, pitch: Expr }, Spiral { revolutions: Expr, pitch: Expr } }
pub enum CoilSection { Circular, Square, TriangleExternal, TriangleInternal }
pub enum SectionPosition { Inside, OnCenter, Outside }
pub struct Coil { pub frame: PrimitiveFrame, pub diameter: Expr, pub size: CoilSize, pub angle: Expr /* taper */,
                  pub left_handed: bool, pub section: CoilSection, pub section_size: Expr,
                  pub position: SectionPosition, pub target: OpTarget }
/// solid.pipe (SOL-19).
pub struct Pipe { pub path: Vec<EntityRef>, pub chain: bool, pub section: CoilSection, pub size: Expr,
                  pub hollow: Option<Expr /* wall thickness */>, pub distance: Expr /* fraction 0..1 */, pub target: OpTarget }

/// solid.fillet (SOL-05; robust path SOL-20; setback SOL-21).
pub enum FilletRadius { Constant(Expr), Chord(Expr), Variable(Vec<VarRadius>) }
pub struct VarRadius { pub at: Expr /* fraction 0..1 along the chain */, pub radius: Expr }
pub struct FilletSet { pub edges: Vec<TopoRef>, pub radius: FilletRadius }
pub enum CornerType { RollingBall, Setback }
pub struct Fillet { pub sets: Vec<FilletSet>, pub tangent_chain: bool, pub corner: CornerType }
/// solid.chamfer (SOL-05).
pub enum ChamferKind { Equal(Expr), TwoDistances { d1: Expr, d2: Expr, face: TopoRef }, DistanceAngle { d: Expr, angle: Expr, face: TopoRef } }
pub struct ChamferSet { pub edges: Vec<TopoRef>, pub kind: ChamferKind }
pub struct Chamfer { pub sets: Vec<ChamferSet>, pub tangent_chain: bool }

/// solid.shell (SOL-06). No faces = hollow body (R-S04-8).
pub enum ShellDirection { Inside, Outside, Both }
pub struct Shell { pub bodies: Vec<BodyId>, pub remove: Vec<TopoRef>, pub inside: Expr, pub outside: Expr,
                   pub direction: ShellDirection, pub tangent_chain: bool }
/// solid.draft (SOL-06).
pub enum DraftSides { OneSide, TwoSides { angle2: Expr }, Symmetric }
pub struct Draft { pub plane: RefInput, pub faces: Vec<TopoRef>, pub angle: Expr, pub flip: bool,
                   pub sides: DraftSides, pub tangent_chain: bool }
/// solid.scale (SOL-06).
pub enum ScaleKind { Uniform(Expr), NonUniform([Expr; 3] /* world x, y, z */) }
pub struct Scale { pub bodies: Vec<BodyId>, pub point: RefInput, pub kind: ScaleKind }
/// solid.thicken (SOL-06).
pub struct Thicken { pub faces: Vec<TopoRef>, pub thickness: Expr, pub side: ThinSide, pub target: OpTarget }

/// solid.combine (SOL-07). `tools` are `Consumed { into: target }` unless `keep_tools`.
pub struct Combine { pub target: BodyId, pub tools: Vec<BodyId>, pub op: BodyOp /* Join | Cut | Intersect */, pub keep_tools: bool }
/// solid.split_body (SOL-07): pieces per the naming note (PieceSlot order).
pub struct SplitBody { pub body: BodyId, pub tool: RefInput /* plane, face or body */, pub extend_tool: bool }
/// solid.split_face (SOL-07).
pub enum SplitFaceTool { Surface(RefInput), CurvesAlong { curves: Vec<EntityRef>, dir: Option<RefInput> /* None = sketch normal */ } }
pub struct SplitFace { pub faces: Vec<TopoRef>, pub tool: SplitFaceTool, pub extend_tool: bool }
/// solid.silhouette_split (SOL-22).
pub struct SilhouetteSplit { pub faces: Vec<TopoRef>, pub view: RefInput /* plane or axis */, pub split_body: bool }

/// solid.pattern (SOL-08 rectangular + circular on bodies and features; SOL-23 path, faces,
/// adjust; SOL-24 components).
pub enum PatternObjects { Bodies(Vec<BodyId>), Features(Vec<FeatureId>), Faces(Vec<TopoRef>), Occurrences(Vec<amg_doc::OccurrenceId>) }
pub enum Spacing { Extent(Expr), Spacing(Expr) }
pub struct PatternDir { pub dir: RefInput, pub count: Expr, pub spacing: Spacing, pub flip: bool, pub symmetric: bool }
pub enum AngleSpan { Full, Extent(Expr), Spacing(Expr) }
pub enum PathOrientation { Identical, PathDirection }
pub enum PatternKind {
    Rectangular { d1: PatternDir, d2: Option<PatternDir> },
    Circular { axis: RefInput, count: Expr, span: AngleSpan, symmetric: bool },
    Path { path: Vec<EntityRef>, count: Expr, spacing: Spacing, start: Expr /* fraction */, orientation: PathOrientation, flip: bool },
}
pub enum ComputeType { Identical, Adjust }
pub struct Pattern { pub objects: PatternObjects, pub kind: PatternKind, pub suppressed: Vec<u32 /* instance k */>,
                     pub compute: ComputeType, pub target: Option<OpTarget> /* feature patterns; None = the seeds' targets */ }
/// solid.mirror (SOL-08; components SOL-24).
pub struct Mirror { pub objects: PatternObjects, pub plane: RefInput, pub op: BodyOp /* NewBody | Join, bodies */, pub target: Option<OpTarget> }

/// solid.move (SOL-09).
pub enum MoveSpec {
    Free { matrix: [[f64; 4]; 4] },                      // from a triad drag; not parametric
    Translate([Expr; 3]),
    Rotate { axis: RefInput, angle: Expr },
    PointToPoint { from: RefInput, to: RefInput },
}
pub struct Move { pub bodies: Vec<BodyId>, pub spec: MoveSpec, pub copy: bool }
/// solid.align (SOL-09).
pub struct Align { pub bodies: Vec<BodyId>, pub from: RefInput, pub to: RefInput, pub flip: bool, pub angle: Expr }
/// solid.offset_face (SOL-09; R-S04-5).
pub struct OffsetFace { pub faces: Vec<TopoRef>, pub distance: Expr }
/// solid.replace_face (SOL-09).
pub struct ReplaceFace { pub faces: Vec<TopoRef>, pub with: RefInput /* plane or face */, pub tangent_chain: bool }
/// solid.delete_face (SOL-09). `heal = false` leaves an open (sheet) body (Fusion "Remove").
pub struct DeleteFace { pub faces: Vec<TopoRef>, pub heal: bool }

/// solid.sweep (SOL-10; R-S04-7 for guide surfaces).
pub enum RailScaling { None, Scale, Stretch }
pub enum SweepMode { Path, GuideRail { rail: Vec<EntityRef>, scaling: RailScaling }, GuideSurface { faces: Vec<TopoRef> } }
pub enum SweepOrientation { Perpendicular, Parallel }
pub struct Sweep { pub profiles: Vec<EntityRef>, pub path: Vec<EntityRef>, pub chain: bool, pub mode: SweepMode,
                   pub distance: Expr /* fraction */, pub taper: Expr, pub twist: Expr,
                   pub orientation: SweepOrientation, pub target: OpTarget }
/// solid.loft (SOL-11): maps onto S20 `robust::loft::LoftSpec`.
pub enum LoftEnd { Free, Direction { angle: Expr, weight: Expr }, Tangent { weight: Expr }, Smooth { weight: Expr }, PointTangent { weight: Expr } }
pub struct LoftSection { pub entity: EntityRef /* profile, face, sketch curve chain head or sketch point */, pub end: LoftEnd }
pub enum LoftGuide { None, Rails(Vec<Vec<EntityRef>>), Centerline { path: Vec<EntityRef> } }
pub struct LoftMapPoint { pub section: u32, pub s_this: f64, pub s_next: f64 }
pub struct Loft { pub sections: Vec<LoftSection>, pub guide: LoftGuide, pub closed: bool, pub ruled: bool,
                  pub merge_tangent_edges: bool, pub mapping: Vec<LoftMapPoint>, pub target: OpTarget }

/// solid.hole (SOL-12). One hole per placement point; always a cut of `participants`.
pub enum HolePlacement {
    SketchPoints { sketch: SketchId, points: Vec<amg_doc::stub::SketchEntityKey> },
    OnFace { face: TopoRef, refs: [(TopoRef, Expr); 2] /* edge + distance */ },
    AtPoint { point: RefInput, dir: Option<RefInput> /* None = face normal / sketch normal */ },
}
pub enum HoleType { Simple, Counterbore { diameter: Expr, depth: Expr }, Countersink { diameter: Expr, angle: Expr } }
pub enum Fit { Close, Normal, Loose }                                  // ISO 273 fine / medium / coarse
pub enum TapType { Simple { diameter: Expr }, Clearance { size: ThreadSize, fit: Fit },
                   Tapped { size: ThreadSize, modeled: bool, thread_depth: Option<Expr> /* None = full */ },
                   TaperTapped { size: ThreadSize } }
pub enum DrillPoint { Flat, Angle(Expr /* default 118° */) }
pub struct Hole { pub placement: HolePlacement, pub hole: HoleType, pub tap: TapType, pub drill_point: DrillPoint,
                  pub extent: Extent /* Distance | ToObject | All */, pub flip: bool, pub participants: Vec<BodyId> }
/// solid.thread (SOL-25) on cylindrical faces; Tapped holes reuse its geometry code.
pub struct Thread { pub faces: Vec<TopoRef>, pub size: Option<ThreadSize> /* None = from the face diameter */,
                    pub modeled: bool, pub full_length: bool, pub length: Expr, pub offset: Expr, pub right_handed: bool }

/// solid.rib / solid.web (SOL-13).
pub enum RibDepth { ToNext, Distance(Expr) }
pub enum RibMode { NormalToSketch, ParallelToSketch }
pub struct Rib { pub curves: Vec<EntityRef>, pub thickness: Expr, pub side: ThinSide, pub depth: RibDepth,
                 pub mode: RibMode, pub flip: bool, pub participants: Vec<BodyId> }
pub struct Web { pub curves: Vec<EntityRef>, pub thickness: Expr, pub side: ThinSide, pub depth: RibDepth,
                 pub flip: bool, pub participants: Vec<BodyId> }
/// solid.emboss (SOL-26).
pub enum EmbossKind { Emboss, Deboss }
pub enum EmbossPlacement { Project, Wrap }
pub struct Emboss { pub profiles: Vec<EntityRef>, pub face: TopoRef, pub depth: Expr, pub kind: EmbossKind,
                    pub placement: EmbossPlacement, pub angle: Expr, pub offset: [Expr; 2] }

/// Surface tools (SOL-14 boundary fill, SOL-27 patch/stitch/unstitch/reverse normal).
pub struct BoundaryFill { pub tools: Vec<RefInput>, pub cells: Vec<CellRef>, pub target: OpTarget, pub remove_tools: bool }
pub struct CellRef { pub fingerprint: amg_doc::GeomFingerprint /* volume, centroid, bbox of the cell */ }
pub enum PatchContinuity { Connected, Tangent, Curvature }
pub struct Patch { pub boundary: Vec<EntityRef> /* edges or sketch curves */, pub continuity: PatchContinuity, pub merge: bool }
pub struct Stitch { pub bodies: Vec<BodyId>, pub tolerance: Expr, pub keep_tools: bool }
pub struct Unstitch { pub faces: Vec<TopoRef> }
pub struct ReverseNormal { pub faces: Vec<TopoRef> }

// ---- standards (SOL-12, SOL-25) ----
pub mod standards {
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub enum ThreadStandard { IsoMetricCoarse, IsoMetricFine, Unc, Unf }
    #[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
    pub struct ThreadSize { pub standard: ThreadStandard, pub designation: String /* "M6x1" */, pub class: String /* "6H" | "6g" */ }
    #[derive(Clone, Debug, PartialEq)]
    pub struct ThreadRow {
        pub designation: &'static str, pub nominal: f64, pub pitch: f64,
        pub minor_internal: f64 /* D1 */, pub pitch_diameter: f64 /* D2 */, pub minor_external: f64 /* d3 */,
        pub tap_drill: f64, pub clearance: [f64; 3] /* ISO 273 fine, medium, coarse */,
        pub counterbore: Option<(f64, f64)>, pub countersink: Option<(f64, f64)>, // (diameter, depth | angle) defaults
    }
    pub fn rows(std: ThreadStandard) -> &'static [ThreadRow];
    pub fn lookup(size: &ThreadSize) -> Option<&'static ThreadRow>;
    /// ISO 68-1 basic-profile check used by the table test.
    pub fn iso68_derived(nominal: f64, pitch: f64) -> (f64 /* D1 */, f64 /* D2 */, f64 /* d3 */);
}
pub use standards::ThreadSize;
/// Cosmetic threads of the evaluated design, for display (S10/S11) and export (S12).
pub fn cosmetic_threads(design: &amg_doc::Design, eval: &amg_doc::Evaluated) -> Vec<CosmeticThread>;
pub struct CosmeticThread { pub body: BodyId, pub face: FaceId, pub size: ThreadSize, pub length: f64 }

/// Solid → mesh conversion used by the Mesh workspace and AM (S02 §8; ~10 lines).
pub fn solid_to_mesh(k: &dyn BrepKernel, s: &Shape, naming: &BodyNaming, q: &amg_brep::TessQuality,
                     ctx: &amg_jobs::JobCtx) -> amg_brep::KResult<(amg_mesh::IndexedMesh, BodyNaming)>;
```

**`Feature` impls.** Every struct above implements `amg_doc::Feature`:
- `inputs()`: §2 rules (participants, refs, sketches, constructions, seeds).
- `exprs()`/`exprs_mut()`: every `Expr` field, with `Dim::LENGTH`, `Dim::ANGLE` or
  `Dim::NONE` (counts, fractions, scale factors, weights).
- `validate()`: finiteness and ranges (counts ≥ 1 and integral after evaluation are checked
  in `compute`; static checks here: non-empty selections, `side2` present iff `TwoSides`,
  `Extent::Full` only on revolve, a hole's `extent` is not `Full`).
- `compute()`: tool (`SolidTool::tool` where implemented) → `apply_body_op`, or the local op →
  naming (`ctx.naming_builder()`, `from_history`, `selection`, `tag`, roles per the naming
  note §5.2) → `BodyChange`s. Errors through `kernel_failure`.

Example (the pattern every feature follows):
```rust
impl Feature for Extrude {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d: Vec<Dependency> = self.profiles.iter().cloned().map(Dependency::Entity).collect();
        d.extend(extent_refs(&self.start, &self.side1, self.side2.as_ref()).map(Dependency::Entity));
        d.extend(self.target.inputs());
        d
    }
    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> { /* start offset, distances, offsets: LENGTH; tapers: ANGLE; thin: LENGTH */ }
    fn validate(&self) -> Result<(), FeatureError> { /* non-empty profiles; side2 iff TwoSides; no Full */ }
    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let tool = self.tool(ctx)?;
        let mut out = FeatureOutput::default();
        apply_body_op(ctx, tool, &mut out)?;
        Ok(out)
    }
}
impl SolidTool for Extrude { fn tool(&self, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError> { /* note §3 */ } }
amg_doc::register_feature!(Extrude, "solid.extrude", version = 1);
register_tool!(Extrude, "solid.extrude");
```

**Kernel calls.** S04 uses the S02 trait as specified. The table lists what each WP calls;
**R-S04-n** are additive requests recorded in S02's Interface changes (2026-11-08). The WP
that needs a request adds it to `amg-brep` itself if the S02 WP it belongs to is already Done
(both are lane A), with its `KernelOp` variant (S20 rule).

| WP | S02 calls |
|---|---|
| SOL-01 | `make_planar_faces`, `prism` (vector), `boolean`, `unify_same_domain`, `sub_geom_all`, `OpResult::per_solid` (R-S04-2) |
| SOL-02 | + `prism` (taper), `split`, `make_face` (extend), `classify`, `transformed`, `Shape::bbox` |
| SOL-03 | `revol`, `split` |
| SOL-04 | `make_box`/`make_cylinder`/`make_sphere`/`make_torus` |
| SOL-05 | `fillet`, `chamfer`, `edge_ends` (R-S04-1) |
| SOL-06 | `shell` (incl. no removed faces, R-S04-8), `offset`, `thicken`, `draft`, `split_faces`, `gtransformed` |
| SOL-07 | `boolean`, `split`, `split_faces`, `prism` (sheet from a wire) |
| SOL-08 | `transformed`, `mirrored` (R-S04-6), N-ary `boolean` |
| SOL-09 | `transformed`, `offset_faces` (R-S04-5), `replace_face`, `defeature`, `heal` |
| SOL-10 | `make_wire`, `pipe` (+ `PipeMode::Support`, R-S04-7) |
| SOL-11 | S20 `robust::loft::{prepare, build}` |
| SOL-12 | `revol` (hole tools), BREP-11 thread helper, `boolean` |
| SOL-18 | `make_thin_face` (R-S04-3) |
| SOL-20 | S20 `robust::fillet::{fillet, max_radius}` |
| SOL-22 | `silhouette_curves` (R-S04-4), `split_faces` |
| SOL-27 | `fill_patch`, `reverse_faces` (R-S04-9), `sew`, `sub_shape` |

```rust
// ===== requests to amg_brep (additive; S02 Interface changes 2026-11-08) ====================
/// R-S04-1 (BREP-09): end points and unit tangents of an edge, in edge orientation.
#[derive(Clone, Debug, PartialEq)]
pub struct EdgeEnds { pub p0: Point3, pub t0: Dir3, pub p1: Point3, pub t1: Dir3 }
fn edge_ends(&self, s: &Shape, e: EdgeId) -> KResult<EdgeEnds>;
/// R-S04-2 (BREP-05): = naming note §12 Q3. Splits a compound result into per-solid results
/// with restricted, reindexed histories, in S02 split order.
impl OpResult { pub fn per_solid(&self) -> KResult<Vec<OpResult>>; }
/// R-S04-3 (BREP-03): thin region of an open or closed planar chain (BRepOffsetAPI_MakeOffset,
/// open-result mode). Offset edges keep the source tags; end-cap edges get `end_tags`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThinProfile { pub plane: Plane, pub chain: Vec<geom::ProfileEdge2>, pub closed: bool,
                         pub left: f64, pub right: f64 /* mm each side, ≥ 0, sum > 0 */, pub end_tags: [geom::Tag; 2] }
fn make_thin_face(&self, p: &ThinProfile, tol: &Tolerance) -> KResult<FaceBuild>;
/// R-S04-4 (BREP-17): silhouette curves of faces seen along `dir` (Contap_Contour).
fn silhouette_curves(&self, s: &Shape, faces: &[FaceId], dir: Dir3) -> KResult<Vec<geom::CurveSeg>>;
/// R-S04-5 (BREP-16): move faces along their normals by per-face distances; neighbours
/// re-intersected; history Modified 1:1 for moved faces.
fn offset_faces(&self, s: &Shape, faces: &[(FaceId, f64)], ctx: &JobCtx) -> KResult<OpResult>;
/// R-S04-6 (BREP-14): reflection about a plane (BRepBuilderAPI_Transform with copy), face
/// orientations fixed, history Modified 1:1. `Shape::transformed` keeps rejecting improper
/// transforms with `NonRigid`.
fn mirrored(&self, s: &Shape, plane: &Plane) -> KResult<OpResult>;
/// R-S04-7 (BREP-15): sweep binormal from a support shell (MakePipeShell::SetMode(SpineSupport)).
// PipeMode gains: Support { shell: usize /* operand */ }
/// R-S04-8 (BREP-07, contract): `shell` with an empty `remove` returns a hollow solid (outer
/// shell + reversed inner shell) instead of an error; history roles as for removed faces.
/// R-S04-9 (BREP-10, used by SOL-27): N-sided patch on boundary edges (BRepOffsetAPI_MakeFilling)
/// and face orientation reversal.
#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Continuity { C0, G1, G2 }
fn fill_patch(&self, boundary: &[Shape], support: &[(Shape, FaceId)], cont: Continuity, ctx: &JobCtx) -> KResult<OpResult>;
fn reverse_faces(&self, s: &Shape, faces: &[FaceId]) -> KResult<OpResult>;
```

**Used from other specs (no change requested):**
- S01 (DOC-06, naming note §10): `RecomputeCtx::{kernel, naming_builder, resolve, resolve_all,
  resolve_profile, region_source, mode, eval_length, eval_angle}`, `NamingBuilder::{tag, role,
  from_history, selection, split_order, generated, finish}`, `BodyNaming::instanced`,
  `BodyChangeKind::{Consumed, Created { derived_from }}`, `Evaluated::successors`,
  `topo_ref_at`, `SetPolicy`.
- S03: `SketchGeom::profiles`, `Profile`, `Loop`, `ProfileEdge { source, curve }`, `Curve2`,
  `TextBlock::regions` (SK-10).
- S05: `construct::{RefInput, resolve::resolve_input, math}`, `CurveEval` (path patterns).
- S20: `HardenedKernel` behind `ctx.kernel()`, `robust::fillet`, `robust::loft`,
  `RetryPolicy::interactive()`.

**For other specs:**
- **S11 (UI-05)**: `amg_features::dialog` as above. Every S04 kind registers a
  `DialogRegistration`; press-pull registers a `DispatchRegistration`.
- **S18 (SCR-02)**: the same registrations drive the Python bindings.
- **S01 (DOC-13)**: SOL-02/03/05/07/08 are the real features the TNP corpus runs on.
- **S07, S16**: `solid_to_mesh` (with face names → mesh group names, naming note §5.3).
- **S12**: `cosmetic_threads` for export; bodies as `BodyGeom::Solid`.

## Work packages
All lane A, gated by ADR-002 (cleared 2026-10-20). Every WP's Done-when also includes:
golden cases in `tests/golden/solid/<wp>_*.toml` (volume, area, bbox, topology counts;
`Validate::Always` in tests), a `DialogRegistration` for each new kind (schema renders in the
UI-05 test harness once UI-05 exists, else a schema unit test), and naming per the naming
note §5.2 checked by at least one edit-and-resolve test per feature.

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| SOL-01 | Feature framework: `amg_features::dialog` (S11 §B + `new_component`, `DispatchRegistration`), `BodyOp`/`OpTarget`/`apply_body_op` (§3 incl. unify + compose, per-solid split, `Consumed`), `SolidTool` + `ToolRegistration`, `profile_faces`/`to_curve2d`, `SolidError` + `kernel_failure` highlight, preview behaviour; `solid.extrude` minimal (one side, distance, start at profile) end-to-end | DOC-06, BREP-03, BREP-04, BREP-05, BREP-10, SK-19 | P0 | Todo | 1. `solid.extrude` (one side, distance) on a sketch rectangle 10×20, h 30: NewBody volume 6000 within 1e-9 relative; Join onto a box, Cut through a box, Intersect with a box each give the analytic volume and the face count expected after unify (golden `sol01_*`). 2. BodyOp rules, one test each: join touching 2 participants keeps the first `BodyId` and marks the second `Consumed`; a cut that splits a bar keeps piece 0's id and creates `BodyId{self,1}` with `derived_from`; a cut touching nothing fails `NothingToCut`; a mesh participant fails `WrongBodyKind`; join touching nothing creates a body and warns. 3. Two adjacent regions of one sketch (a rectangle split by a line) extrude to one body with 6 faces. 4. DOC-11 corpus cases TNP-01, TNP-02 and TNP-03 run on `solid.extrude` (instead of `test.brep_prism`) and meet their `expect`. 5. A kernel failure with `offending` edges (injected failing kernel) gives `Diagnostic::entities` resolving to those edges. 6. Preview: an extrude-cut of a 20-edge profile into a 200-face body, `ComputeMode::Preview`, median < 100 ms (criterion `budget/sol-extrude-preview`); full mode recorded. 7. `dialog::registration("solid.extrude")` exists and its `build(defaults)` round-trips through `read`. |
| SOL-02 | Extrude: all starts, directions and extents (§4; note §3): offset start, start from a plane, two sides, symmetric (half/whole), taper per side, to object (plane, face, body, vertex; offset; extend faces), all; planar-face profiles; text profiles | SOL-01, CON-03, BREP-14, SK-06 | P0 | Todo | 1. ≥ 20 golden cases cover every start × direction × extent combination the dialog allows, incl. to-object on an inclined plane, a cylinder face, a body and a vertex, with and without offset and extend; analytic volumes within 1e-9 relative, curved targets within 1e-6. 2. A face that doesn't cover the profile fails `TargetMissed` with the "Extend faces" hint; extend on fixes it; a B-spline target with extend fails `ExtendUnsupported`. 3. "All" cuts through 3 stacked bodies in one direction and through both sides in two-sided mode. 4. Text "AgB" (SK-10 `regions().solid`) extrudes to 3 solids with 4 holes in total (closes SK-10 Done-when 2). 5. Editing distance, taper or extent type keeps the lateral and cap names (`Exact` resolution of a downstream fillet ref). 6. Full-mode extrude-cut of a 100-edge profile into a 200-face body, median < 80 ms (`budget/sol-extrude-cut`). |
| SOL-03 | Revolve: profiles, axis (sketch line, edge, construction or origin axis), angle, full, two sides, symmetric, to object; profile/axis validation | SOL-01, CON-03, BREP-14 | P0 | Todo | 1. ≥ 8 golden cases (rectangle 90° and 360°, triangle with an edge on the axis → cone, two sides 30°/60°, symmetric 90°, each axis kind) match Pappus volumes within 1e-9 relative. 2. A profile crossing the axis fails `AxisCrossesProfile` naming the profile. 3. DOC-11 TNP-22 (edit to 360° → cap refs `NotFound`) runs on `solid.revolve` and meets its `expect`. 4. Revolve to a plane through the axis at 75° equals the 75° angle case within 1e-9. |
| SOL-04 | Primitives: box, cylinder, sphere, torus on a `PrimitiveFrame` (origin plane, construction plane, planar face), with `OpTarget` | SOL-01, CON-03, BREP-14 | P0 | Todo | 1. Golden volume and area for each primitive (exact within 1e-9) and for a box joined to and cut from a body. 2. A box placed on a planar face follows the face after an upstream height edit (centroid within 1e-9). 3. Editing a box size keeps `XMin#0 … ZMax#0` names (`Exact`). 4. Each kind has a dialog with a distance manipulator anchor per size field. |
| SOL-05 | Fillet (constant, chord, variable) and chamfer (equal, two distances, distance-angle) on raw BREP-06: edge sets per body, tangent-chain propagation (R-S04-1), failing-edge highlight | SOL-01, BREP-06, BREP-09 | P0 | Todo | 1. ≥ 15 golden cases (cube edge, 4-edge tangent loop of a rounded rectangle, 3-edge vertex blend, variable 2→4 mm, chord 3 mm, all chamfer kinds, two bodies in one feature) within 1e-6 relative of reference volumes. 2. Picking one edge of a rounded-rectangle top selects all 8 tangent edges; with `tangent_chain = false` only that edge. 3. A radius larger than the adjacent face fails and `Diagnostic::entities` names exactly the failing edge(s). 4. DOC-11 TNP-11 and TNP-27 run on `solid.fillet` and meet their `expect`. 5. Preview of a 10-edge constant fillet on a 200-face body, median < 100 ms (`budget/sol-fillet-preview`). 6. KH-03 can start: S20's corpus harness drives `solid.fillet` through a public test entry point. |
| SOL-06 | Shell (remove faces, hollow, inside/outside/both, tangent chain), draft (one side, two sides, symmetric; tangent chain), scale (uniform, non-uniform), thicken (`OpTarget`) | SOL-01, BREP-07, BREP-16, BREP-14 | P0 | Todo | 1. Shelled box (top removed, 1 mm) inside, outside and both, and a hollow box (no faces removed, R-S04-8) match analytic volumes within 1e-9 relative. 2. Shelling a filleted box with tangent chain removes the top and its fillet ring (face count checked). 3. A thickness above the smallest concave radius fails with the offending faces highlighted. 4. Draft of 4 box sides at 3°: measured angles 3° ± 1e-9 rad, analytic volume; symmetric draft about a mid-plane matches the analytic volume. 5. Uniform scale 2 → volume × 8 within 1e-12 relative; non-uniform (1, 2, 3) of a sphere r 10 → ellipsoid volume within 1e-9. 6. Thicken of a B-spline sheet by 1 mm is valid and within 0.5 % of area × t. 7. TNP-X3 and TNP-X6 meet their `expect`. 8. Shell preview of a 100-face body, median < 100 ms (`budget/sol-shell-preview`). |
| SOL-07 | Combine (join/cut/intersect, keep tools), split body (plane, face, body; extend tool), split face (surface tool; sketch curves along a vector) | SOL-01, BREP-16, CON-03 | P0 | Todo | 1. Combine golden cases for each op, with and without keep tools; consumed tools are `Consumed { into }` (TNP-23 meets its `expect`). 2. Splitting a box by a plane gives pieces in the naming-note order (piece 0 keeps the `BodyId`); TNP-21 meets its `expect`. 3. Split face by a sketch line projected along the sketch normal and by a plane: two faces whose areas match within 1e-9. 4. A tool that doesn't cross fails `NoSplit` with the "extend the tool" hint. |
| SOL-08 | Pattern (rectangular, circular; bodies and features; suppressed instances; identical compute) + mirror (bodies new/join, features) on `SolidTool` and `mirrored` (R-S04-6) | SOL-01, SOL-02, CON-03, BREP-14 | P0 | Todo | 1. 3×4 rectangular and 6-instance circular body patterns (incl. symmetric and 2 suppressed instances): body count and total volume exact. 2. A 10×10 feature pattern of a Ø3 cut-extrude in a 100×100×5 plate: analytic volume within 1e-9 relative, **one** boolean call (kernel call counter), instances share TShapes (`Shape::transformed` only), full recompute < 300 ms (`budget/sol-pattern-100`). 3. Mirror of a body and of an extrude boss: mirrored volume equal, BRepCheck valid (orientation). 4. TNP-17 and TNP-18 meet their `expect`. |
| SOL-09 | Move/copy (free, translate, rotate, point-to-point), align, offset face (R-S04-5), replace face, delete face (heal / remove), press-pull dispatch | SOL-01, BREP-07, BREP-16, CON-03 | P0 | Todo | 1. Move keeps names: a fillet ref on a moved body resolves `Exact`; a copy is a new body named `instanced(self, slot)`. 2. Align face-to-face makes the faces coplanar within 1e-9 mm. 3. Offset of a box top by +2 mm changes the height by 2 within 1e-9; a cylinder face +1 mm changes the radius by 1 within 1e-9. 4. Replace face onto an offset plane matches the analytic volume. 5. Delete of a fillet face with heal gives the pre-fillet volume within 1e-6; without heal gives an open body with the expected free edges. 6. `dispatch` maps a face, an edge and a profile selection to `solid.offset_face`, `solid.fillet` and `solid.extrude`. |
| SOL-10 | Sweep: path (chain), guide rail (none/scale/stretch), guide surface (R-S04-7), distance, taper, twist, orientation | SOL-01, BREP-15, CON-03 | P1 | Todo | 1. Circle swept along a line and along an arc within 1e-6 of the analytic volume; along a 3D spline within 0.1 % of the reference. 2. Twist 90° and taper 5° match reference volumes within 1e-4 relative. 3. Guide-rail scale vs stretch give the reference volumes; a guide-surface sweep stays normal to the surface (sampled angle < 0.1°). 4. Distance 0.5 gives half the length (volume within 1e-6). 5. A self-intersecting sweep fails with a clean error. 6. Lateral faces of a multi-edge path have the naming-note names (`Split{self, i, m}` or spine tags). |
| SOL-11 | Loft feature + dialog (profiles, rails, centerline, end conditions, point mapping) on the S20 loft builder (`robust::loft::build`), not raw `ThruSections`; live correspondence lines from `loft::prepare`; `LoftDiagnostics` in the dialog | SOL-01, KH-08, KH-18, SK-06 | P1 | Todo | 1. ≥ 98 % success on the loft corpus (KH-05) driven through `solid.loft` (matches KH-18 Done-when 1). 2. Preview uses `RetryPolicy::interactive()`; correspondence lines come from `prepare` in < 20 ms on corpus median. 3. Warnings (`HighTwist`, `CrossingCorrespondence`) appear as feature warnings. 4. TNP-X4 meets its `expect`. |
| SOL-12 | Hole: placements (sketch points, on face with two edge distances, at point), types (simple, counterbore, countersink), tap types (simple, clearance, tapped, taper tapped), drill point, extents; ISO metric coarse + fine table (`standards`); cosmetic threads (`cosmetic_threads`) and modeled tapped threads via the BREP-11 helper | SOL-02, BREP-11 | P1 | Todo | 1. Golden cases for every hole type × {simple, clearance, tapped} × {flat, 118°}, on distance, to-object and all extents, within 1e-9 relative (analytic). 2. Holes at 3 sketch points; deleting point 1 keeps the names of the others (TNP-X5 meets its `expect`). 3. The ISO metric table covers ISO 261 M1.6–M64 coarse and fine; the test recomputes D1, D2, d3 from ISO 68-1 within 1e-3 mm and checks tap drills and ISO 273 clearances against the source table. 4. A modeled M6 tapped hole is valid and its minor diameter measures 4.917 ± 1e-3 mm. 5. `cosmetic_threads` lists a tapped hole with its size and length. |
| SOL-13 | Rib and web: open sketch curves, thickness and side, depth to next or distance, normal-to-sketch and parallel-to-sketch modes (parallel: lines only), curve ends extended to the body | SOL-18, CON-03 | P1 | Todo | 1. A straight rib between two walls of a U-channel fills to the floor (to next): analytic volume within 1e-9. 2. A curved rib in normal mode, 2 mm thick, matches the reference within 1e-6. 3. A web of 3 crossing lines gives one fused body with the expected face count. 4. Parallel mode with an arc fails with a clear `Invalid` error. |
| SOL-14 | Boundary fill: tools (bodies, planes, faces), cell enumeration (`split` of all tools), cell selection by fingerprint (`CellRef`), `OpTarget`, remove tools | SOL-07 | P1 | Todo | 1. Three intersecting cylinders give the expected cell count; filling the common cell gives the analytic Steinmetz volume within 1e-6. 2. A cell selection survives a 10 % tool dimension edit (fingerprint match, warning only above the naming thresholds). 3. A cell that no longer exists gives a clear error. |
| SOL-15 | Rule fillet (rule-based edge selection: all edges, between faces/features, rules for concave/convex) | SOL-05 | P2 | Todo | Refine before Phase 5: rule set, its naming (edges re-derived per recompute), and Done-when. |
| SOL-16 | Derive / insert design as component | DOC-07, ASM-01, SOL-01 | P2 | Todo | Refine before Phase 5: external design reference, update semantics, naming via `BodyNaming::for_import`. |
| SOL-17 | Sheet metal (flange, bend, unfold, flat pattern) | SOL-01, SOL-06 | P3 | Todo | Placeholder: split into 1–3-day WPs before Phase 6. |
| SOL-18 | Thin extrude and thin revolve: open and closed chains, side 1/2/center (R-S04-3), names `ThinInner`/`ThinOuter`/`ThinEnd` | SOL-02, SOL-03 | P0 | Todo | 1. Thin extrude of an open line+arc chain (length L, t 2 mm, h 10 mm) in each side mode: volume L·t·h within 1e-9 relative. 2. A closed rectangle thin extrude gives a ring with the analytic volume. 3. A thin revolve of an open chain matches Pappus within 1e-9. 4. Editing the thickness keeps all names (`Exact`). |
| SOL-19 | Coil (all size modes, sections, positions, taper) and pipe (path chain, sections, hollow, distance) | SOL-04, SOL-10, BREP-11 | P0 | Todo | 1. Circular-section coil (d 20, 5 turns, pitch 4, section 2): volume within 0.1 % of section area × helix length. 2. Square and triangle sections and inside/outside positions give valid solids with reference volumes within 0.1 %. 3. Hollow pipe along a 3-segment path: volume within 1e-6 of the analytic value. 4. A coil whose pitch is smaller than its section size fails with a self-intersection error, never an invalid body. |
| SOL-20 | Fillet/chamfer on S20's robust chain: `robust::fillet::fillet` with `FilletDiagnosis` (failing edges highlighted, largest workable radius in the message), `max_radius` as a background job, interactive policy in preview | SOL-05, KH-03 | P0 | Todo | 1. On the KH-02 fillet corpus driven through `solid.fillet`, the success rate equals KH-03's recorded rate (no loss in the feature layer). 2. 100 % of failures highlight ≥ 1 edge and show a max radius when `max_radius` returns one. 3. Preview never exceeds the interactive deadline (test with an injected slow strategy). |
| SOL-21 | Setback corners for fillets (S20 KH-19 corner patches): `CornerType::Setback` with setback distances | SOL-20, KH-19 | P1 | Todo | 1. The `corner3` subset driven through `solid.fillet` with setback reaches KH-19's success rate (≥ 95 %). 2. TNP-27 still meets its `expect` with setback on. |
| SOL-22 | Silhouette split (R-S04-4): faces split along the silhouette for a view direction, optional body split | SOL-07 | P1 | Todo | 1. A sphere r 10 split for +Z gives two faces of area 2π·100 within 1e-6 relative. 2. A cylinder seen side-on splits into two half-cylinders; with `split_body` the body splits into two solids of equal volume within 1e-9. 3. A B-spline face split matches the sampled silhouette within 1e-4 mm. |
| SOL-23 | Pattern on path (spacing/extent, start, orientation), face pattern (boss/pocket on a planar support), adjust compute type | SOL-08, CON-03 | P0 | Todo | 1. 8 instances along a spline path with path-direction orientation: instance frames within 1e-9 of `CurveEval` frames. 2. A face pattern of a boss (6 faces) on a plate gives the analytic volume; a pocket the same. 3. Adjust compute of a to-object extrude seed across a sloped target gives per-instance heights matching the reference within 1e-6. |
| SOL-24 | Component operations: "New component" body op (`new_component`, S11 request), component pattern and mirror (occurrences), cross-component participants | SOL-08, ASM-02 | P0 | Todo | 1. Extrude with "New component" creates one component and one body in one undo entry. 2. A circular pattern of an occurrence creates 6 occurrences with the right transforms; mirror creates a mirrored occurrence. 3. A cut whose participant is in a sub-component (assembly context) works and is undone in one step. |
| SOL-25 | Thread feature on cylindrical faces (cosmetic/modeled, full length or length+offset, handedness), shared geometry with tapped holes; UNC/UNF tables (scope per QUESTIONS #54) | SOL-12 | P1 | Todo | 1. ISO M3, M6, M10, M20 external modeled threads on cylinders: crest pitch within 1e-6 mm and diameters within 1e-4 mm of ISO 68-1 (BREP-11 checks reused). 2. A thread size is inferred from the face diameter (Ø6 → M6×1). 3. If UNC/UNF are in scope, ¼-20 UNC and ¼-28 UNF pass the same checks. |
| SOL-26 | Emboss / deboss of profiles and text on planar faces (project) and on cylindrical and conical faces (wrap by unrolling) | SOL-02, SK-10 | P1 | Todo | 1. Text "AgB" embossed 1 mm on a plane: volume = text area × 1 within 1e-6. 2. A 10 mm square wrapped on a cylinder r 20 and debossed 1 mm: volume within 1e-4 relative of the analytic value. 3. Editing the depth keeps glyph lateral names. 4. A freeform target face fails with `NotYet { what: "emboss on freeform faces" }`. |
| SOL-27 | Surface repair tools (QUESTIONS #1, P1): patch (planar and N-sided, R-S04-9), stitch (sew to a solid when closed), unstitch, reverse normal | SOL-01, BREP-10 | P1 | Todo | 1. Stitching the 6 faces of an imported cube with 1e-4 mm gaps gives a valid solid with volume within 1e-6. 2. A planar patch closes a hole in a sheet; an N-sided G1 patch on a 4-sided curved gap is valid with boundary G1 deviation ≤ 0.5°. 3. Unstitch then stitch round-trips volume and face count. 4. Reverse normal flips the chosen faces (face orientation test). |
| SOL-28 | Ruled, extend, trim, untrim surfaces | SOL-27 | P2 | Todo | Refine before Phase 5 (needs S02 requests for `BRepLib::ExtendFace` and ruled surfaces between edges). |
| SOL-29 | Fusion-parity harness and recompute budget: owner Fusion designs (`$AMG_TESTDATA_DIR/fusion/`) rebuilt by Rust test scripts and compared by volume/area/bbox with the STEP exports; scripted 50-feature part bench | SOL-02, SOL-03, SOL-04, SOL-05, SOL-06, SOL-07, SOL-08, SOL-09 | P0 | Todo | 1. Every supplied owner design whose features exist is rebuilt; volume and area match the Fusion STEP within 1e-4 relative and bbox within 1e-3 mm, or the difference is explained in the log. Skipped cleanly if the folder is missing. 2. `budget/sol-recompute-50`: a 50-feature part (sketches, extrudes, cuts, fillets, a shell, a pattern, holes) recomputes after an edit to the first sketch in < 500 ms on the reference machine (vision budget); the bench prints the 5 slowest features. |

**Dependency and scope changes in this refinement** (2026-11-08):
- SOL-01: DOC-02, BREP-04 → DOC-06, BREP-03, BREP-04, BREP-05, BREP-10, SK-19 (naming,
  planar faces, history, unify, sketch profiles). Done-when made measurable: "extrude end to
  end" is the minimal one-side distance extrude.
- SOL-02: + CON-03, BREP-14; thin moved to SOL-18.
- SOL-03: + CON-03, BREP-14. SOL-04: + CON-03, BREP-14; coil and pipe moved to SOL-19
  (they need BREP-11 and SOL-10; S04 Open question 2).
- SOL-05: + BREP-09 (R-S04-1); setback moved to SOL-21; the S20 chain and diagnosis moved to
  SOL-20 (S04 Open question 4).
- SOL-06: + BREP-16, BREP-14. SOL-07: BREP-04 → BREP-16, CON-03; silhouette split moved to
  SOL-22. SOL-08: + SOL-02, CON-03, BREP-14; path patterns, face patterns and adjust compute
  moved to SOL-23; components moved to SOL-24. SOL-09: + BREP-16, CON-03.
- SOL-10: BREP-03 → BREP-15, CON-03. SOL-11: KH-08 → KH-08, KH-18, SK-06 (S04 Open
  question 4).
- SOL-12: SOL-02, BREP-11 kept; the standalone thread feature and UNC/UNF moved to SOL-25.
  Tapped holes keep their modeled/cosmetic threads in SOL-12, so S02 BREP-11's "helper used
  by SOL-12" still holds.
- SOL-13: SOL-02, SK-10 → SOL-18, CON-03; emboss moved to SOL-26. The naming note's
  "Emboss (SOL-13)" and "Thread, modeled (SOL-12)" rows now apply to SOL-26 and
  SOL-12/SOL-25; the rules are unchanged.
- SOL-14: P2 → P1 and narrowed to boundary fill (parity matrix lists boundary fill at P1);
  patch/stitch/unstitch/reverse normal → SOL-27 (P1, QUESTIONS #1); ruled/extend/trim/untrim
  → SOL-28 (P2).
- SOL-16: + ASM-01, SOL-01. SOL-17: + SOL-06.
- New: SOL-18 … SOL-29.

## Testing
- **Golden geometry**: `tests/golden/solid/` with one TOML per case (`amg_testkit::assert_golden!`
  on the tessellated soup, plus exact `mass_props` compared at 1e-9 relative where analytic).
  Case names start with the WP id (`sol02_to_object_cylinder.toml`).
- **Validity**: every result passes BRepCheck (`Validate::Always` in the test kernel config).
- **Naming**: the DOC-11 corpus cases named in each WP's Done-when run on the real feature;
  DOC-13 runs the full corpus once SOL-02/03/05/07/08 are Done.
- **Proptests**: random extrude distances/tapers on random profiles (SOL-02), random fillet
  radii (SOL-05) and random pattern counts (SOL-08) give `Ok(valid)` or a clean error, never a
  panic.
- **Dialogs**: every registration's `build(defaults(..))` gives a valid feature and
  `read(build(v)) == v` (round-trip test over all registrations).
- **Fusion parity**: SOL-29 rebuilds the owner's Fusion designs (Open question 1) by Rust test
  scripts. Once S18's Python API exists (SCR-02), the same scripts are also run from Python.
- **Benchmarks** (criterion, reference machine): `budget/sol-extrude-preview`,
  `budget/sol-extrude-cut`, `budget/sol-fillet-preview`, `budget/sol-shell-preview`,
  `budget/sol-pattern-100`, `budget/sol-recompute-50`.

## Performance targets
| Target | Budget | WP |
|---|---|---|
| Preview: extrude-cut, 20-edge profile into a 200-face body | < 100 ms median | SOL-01 |
| Full extrude-cut, 100-edge profile into a 200-face body | < 80 ms median | SOL-02 |
| Preview: 10-edge constant fillet, 200-face body | < 100 ms median | SOL-05 |
| Preview: shell of a 100-face body | < 100 ms median | SOL-06 |
| Feature pattern, 100 holes in a plate (full recompute) | < 300 ms | SOL-08 |
| 50-feature part, recompute after an early sketch edit | < 500 ms (vision) | SOL-29 |

All of these run on `amg-jobs` through the recompute engine; nothing runs on the UI thread
(vision: UI never blocks > 16 ms).

## Open questions
1. Owner to supply 5–10 representative Fusion designs (STEP + screenshots of timeline) as parity references.
   **Answered 2026-10-20:** yes, private, supplied later (before Phase 2 S04 implementation). They go
   under `$AMG_TESTDATA_DIR/fusion/` (S00 Open question 2) with a short note per design (purpose,
   features used); not committed.
   **2026-11-08:** SOL-29 consumes them; the harness skips cleanly while they are missing.
2. (from S02, 2026-10-20) WP dependency repoints for the S02 WP split: SOL-01 also on BREP-05 (history) and BREP-03 (planar faces); SOL-02/03 → BREP-14; SOL-04 → BREP-14, and coil uses `helix` (BREP-11), so SOL-04 needs BREP-11 or coil moves to a later WP; SOL-06 → BREP-07 + BREP-16; SOL-07 → BREP-16 (split); SOL-09 → BREP-07 + BREP-16; SOL-10 → BREP-15 (was BREP-03); SOL-12 → BREP-11 (unchanged). SOL-02/03 pass profiles through `make_planar_faces` with a `geom::Tag` per edge (the sketch entity key).
   **Resolved 2026-11-08:** all applied; coil and pipe moved to SOL-19 (deps BREP-11, SOL-10).
3. (from S11, 2026-10-20) Adopt `amg_features::dialog` for SOL-01 (S11 Interfaces §B): `EntityFilter`, `DialogSchema`, `FieldSpec`/`FieldKind`/`Condition`/`FieldValue(s)`, `ManipAnchor`, `CommandMeta`, `DialogRegistration` via `inventory`, extendable additively. Expression fields carry `param: Option<ParamId>` in edit mode. Features registered this way get command, toolbar and timeline entries automatically, so lane A needs no `amg-ui::solid`.
   **Resolved 2026-11-08:** adopted as is, plus two additive pieces: `DialogRegistration::new_component` (SOL-24) and `DispatchRegistration` (press-pull, SOL-09). Recorded in S11's Interface changes.
4. (from S20, 2026-10-20) SOL-05 uses `robust::fillet::fillet` and `FilletDiagnosis` (failing-edge highlight, max radius) once KH-03 lands, raw BREP-06 before. SOL-11 deps should become KH-08 + KH-18 (rails/centerline) + SK-06; it renders `LoftDiagnostics`, uses `loft::prepare` for live correspondence lines and `RetryPolicy::interactive()` for preview. Its Done-when (≥ 98 % on the loft corpus) matches KH-18 Done-when 1.
   **Resolved 2026-11-08:** SOL-05 stays on raw BREP-06 (KH-03 depends on SOL-05); the switch is the new SOL-20 (deps SOL-05, KH-03). SOL-11 deps and Done-when applied.
5. (integration, 2026-10-20) Agreed earlier-wave repoint applied: SOL-02 now also depends on SK-19 (`amg-features::sketch` profiles).
   **2026-11-08:** SK-19 now enters through SOL-01 (SOL-02 depends on SOL-01).
6. (2026-11-08, owner, QUESTIONS #53) Print allowances for holes and modeled threads on FFF/resin parts.
7. (2026-11-08, owner, QUESTIONS #54) UNC/UNF tables: needed, or ISO metric only?
8. (2026-11-08, owner, QUESTIONS #55) G2 (curvature-continuous) fillets: needed at all, and if so only with the Phase 7 Rust fillet engine?

## Refinement log

### 2026-11-08 - S04-refine (lane A refinement)
- **Read:** AGENTS.md, vision, architecture, ADRs, roadmap, QUESTIONS, the parity matrix; S01
  (DOC-06/11/12/13 and the accepted naming note, especially §5 and §10), S02 (Refined;
  Interfaces, WPs, Interface changes), S03 (profiles, SK-10/15/19 status and code), S05
  (construct API, CON-03), S07 (MESH-14 extrude stand-in), S11 (§B dialog request, UI-05),
  S18 (SCR-02), S20 (robust fillet/loft API, KH-03/08/18); the code of `amg-doc`
  (`Feature`, `RecomputeCtx`, `FeatureOutput`, `naming.rs`), `amg-features::{mesh, sketch}`
  (registration pattern), `amg-sketch::profile` and `offset`, and `amg-brep` (still empty:
  S02's Interfaces are the contract).
- **Researched:** Autodesk Fusion help (Extrude reference, Extrude a solid body, Hole
  reference, Holes and threads) and the Fusion API (`ExtrudeFeatureInput.setTwoSidesExtent`);
  OCCT 8.0.0 reference manual for `BRepPrimAPI_MakePrism`/`MakeRevol`,
  `BRepOffsetAPI_MakeThickSolid` (simple mode can't remove faces; join modes arc/intersection),
  `BRepOffsetAPI_MakeOffset` (open-wire offsets), `MakePipeShell` (`SetMode` with a spine
  support), `DraftAngle`, `MakeFillet`/`MakeChamfer`, `BRepAlgoAPI_Splitter`,
  `BRepOffsetAPI_MakeFilling`, `Contap_Contour`; OCCT tracker #25916 (disconnected shell after
  face removal) and forum threads on thick solids; Onshape FeatureScript (`opExtrude`,
  `opPattern`), FreeCAD PartDesign, build123d `extrude(until=…)`; ISO 68-1, ISO 261, ISO 273
  and published tap-drill charts (M3 2.5, M4 3.3, M5 4.2, M6 5.0; clearance coarse column
  3.6/4.8/5.8/7.0). Details and the failure-mode table: the new note.
- **Decided:**
  - Tool-then-boolean structure with `SolidTool`, so feature patterns reuse each feature's own
    tool and run one N-ary boolean (note §4).
  - Participants stored explicitly (Fusion "Objects to cut"); join-with-nothing makes a new
    body with a warning, cut-with-nothing fails (to be confirmed against the owner's Fusion
    designs, Open question 1).
  - "New component" is a dialog-level transaction, not stored feature data (components are
    authored state).
  - "To object" and "All" never pass infinite solids to booleans (finite prism + split).
  - Preview still builds names; preview budget measured on three named typical cases.
  - Press-pull is a dispatch command, not a feature.
  - Standards data in embedded TOML checked against ISO 68-1 formulas.
  - Mesh participants are rejected (no implicit mesh fallback in solid features).
- **Changed:** status Draft → Refined; Summary, Scope, Design, Interfaces, WPs, Testing,
  Performance targets rewritten; Open questions 2–5 resolved; 3 owner questions added
  (QUESTIONS #53–#55). WPs re-split as listed under the WP table; new SOL-18 … SOL-29.
  Roadmap phase lists updated for the new and moved WPs. Additive requests recorded in S02
  (R-S04-1 … R-S04-9) and S11 (dialog extensions).
- **Design note:** [docs/notes/solid-features.md](../notes/solid-features.md) (new).
- **Leftovers:** full-round fillet and G2 fillets are not planned (QUESTIONS #55); emboss on
  freeform faces, cross-component BodyOps beyond SOL-24, and SOL-15/16/17/28 are refined before
  their phase.
