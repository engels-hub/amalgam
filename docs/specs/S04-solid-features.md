# S04 — Solid modeling features

| | |
|---|---|
| **Status** | Ready |
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
- G2 (curvature-continuous) fillets: needed (owner, QUESTIONS #55) but OCCT can't build them.
  SOL-05 ships G1 only; G2 is planned with the Phase 7 Rust fillet engine (S20 KH-10).
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
- Participants must be solid bodies of the feature's context component or of a component
  placed exactly once below it (SOL-24, Fusion's assembly context: the tool is moved into the
  participant's component space; `WrongBodyKind`, `OtherComponent`, `AmbiguousPlacement`,
  `MirroredPlacement`; a join fuses bodies of one component only, `MixedComponents`).

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
SOL-25: ISO metric + UNC/UNF; pipe threads (BSP/NPT) only on request (owner, QUESTIONS #54).
A unit test recomputes every derived column from the ISO 68-1 formulas (note §7). Print
allowances: `Hole` and `Thread` carry a per-feature `clearance` (radial mm, default 0, can be
driven by a parameter); pure ISO values otherwise; no per-design default until S16 printer
profiles exist (owner, QUESTIONS #53).

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

// ===== amg_features::solid::component (SOL-24): transaction builders, not features ===========
pub enum ComponentOpError { NoComponent(ComponentId), NotOccurrences, NoOccurrences,
    NoOccurrence(OccurrenceId), NotInContext { occ: OccurrenceId, context: ComponentId },
    Feature(FeatureError), Command(CommandError) }
/// `"Component"`, `"Component1"`, … (first free).
pub fn unique_component_name(design: &Design, base: &str) -> String;
/// [NewComponent { parent, name, identity }, AddFeature { context: <new component> }].
pub fn new_component_commands(design: &Design, parent: ComponentId, name: &str, feature: FeatureBox,
    feature_name: Option<String>) -> Result<Vec<Command>, ComponentOpError>;
/// Pattern layout of a `PatternObjects::Occurrences` pattern in `context` (seed and suppressed left out).
pub fn occurrence_pattern_transforms(design: &Design, eval: &Evaluated, pattern: &Pattern,
    context: ComponentId) -> Result<Vec<(u32, Transform)>, ComponentOpError>;
/// One `AddOccurrence { parent: context, component, transform: T_k · L }` per source and instance.
pub fn pattern_occurrences(design: &Design, eval: &Evaluated, pattern: &Pattern, context: ComponentId)
    -> Result<Vec<Command>, ComponentOpError>;
/// One `AddOccurrence` at `M · L` per source, `M` the reflection about the plane (det −1).
pub fn mirror_occurrences(design: &Design, eval: &Evaluated, mirror: &Mirror, context: ComponentId)
    -> Result<Vec<Command>, ComponentOpError>;
pub fn reflection(plane: &Plane) -> Transform;
// amg_features::dialog (SOL-24): AddFeature, or new_component_commands when
// `reg.new_component(values)` is `Some(name)`.
pub enum CommitError { Fields(FieldErrors), Component(ComponentOpError) }
pub fn new_feature_commands(reg: &DialogRegistration, values: &FieldValues, design: &Design,
    context: ComponentId) -> Result<Vec<Command>, CommitError>;
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
pub struct Combine { pub target: Option<BodyId> /* None until picked */, pub tools: Vec<BodyId>, pub op: BodyOp /* Join | Cut | Intersect */, pub keep_tools: bool }
/// solid.split_body (SOL-07): pieces per the naming note (PieceSlot order).
pub struct SplitBody { pub body: Option<BodyId> /* None until picked */, pub tool: RefInput /* plane, face or body */, pub extend_tool: bool }
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
    Free { matrix: [[f64; 4]; 4] },                      // from a triad drag; not parametric; row-major, translation in column 3
    Translate([Expr; 3]),
    Rotate { axis: RefInput, angle: Expr },
    PointToPoint { from: RefInput, to: RefInput },
}
pub struct Move { pub bodies: Vec<BodyId>, pub spec: MoveSpec, pub copy: bool }
/// solid.align (SOL-09).
pub struct Align { pub bodies: Vec<BodyId>, pub from: Option<RefInput> /* None until picked */, pub to: RefInput, pub flip: bool, pub angle: Expr }
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
                  pub clearance: Expr /* radial mm added to the hole radius, default 0 (QUESTIONS #53) */,
                  pub extent: Extent /* Distance | ToObject | All */, pub flip: bool, pub participants: Vec<BodyId> }
/// solid.thread (SOL-25) on cylindrical faces; Tapped holes reuse its geometry code.
pub struct Thread { pub faces: Vec<TopoRef>, pub size: Option<ThreadSize> /* None = from the face diameter */,
                    pub modeled: bool, pub full_length: bool, pub length: Expr, pub offset: Expr, pub right_handed: bool,
                    pub clearance: Expr /* radial mm, default 0 (QUESTIONS #53) */ }

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
/// R-S04-3 (BREP-03, added by SOL-18): thin region of an open or closed planar chain
/// (BRepOffsetAPI_MakeOffset, open-result mode, sharp corners). Offset edges keep the source
/// tags; end-cap edges get `end_tags`; `ThinFace::left` tells the two boundaries apart.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThinProfile { pub plane: Plane, pub chain: Vec<geom::ProfileEdge2>, pub closed: bool,
                         pub left: f64, pub right: f64 /* mm each side, ≥ 0, sum > 0 */, pub end_tags: [geom::Tag; 2] }
pub struct ThinFace { pub build: FaceBuild, pub left: Vec<EdgeId> }
fn make_thin_face(&self, p: &ThinProfile, tol: &Tolerance) -> KResult<ThinFace>;
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
| SOL-01 | Feature framework: `amg_features::dialog` (S11 §B + `new_component`, `DispatchRegistration`), `BodyOp`/`OpTarget`/`apply_body_op` (§3 incl. unify + compose, per-solid split, `Consumed`), `SolidTool` + `ToolRegistration`, `profile_faces`/`to_curve2d`, `SolidError` + `kernel_failure` highlight, preview behaviour; `solid.extrude` minimal (one side, distance, start at profile) end-to-end | DOC-06, BREP-03, BREP-04, BREP-05, BREP-10, SK-19 | P0 | Done | 1. `solid.extrude` (one side, distance) on a sketch rectangle 10×20, h 30: NewBody volume 6000 within 1e-9 relative; Join onto a box, Cut through a box, Intersect with a box each give the analytic volume and the face count expected after unify (golden `sol01_*`). 2. BodyOp rules, one test each: join touching 2 participants keeps the first `BodyId` and marks the second `Consumed`; a cut that splits a bar keeps piece 0's id and creates `BodyId{self,1}` with `derived_from`; a cut touching nothing fails `NothingToCut`; a mesh participant fails `WrongBodyKind`; join touching nothing creates a body and warns. 3. Two adjacent regions of one sketch (a rectangle split by a line) extrude to one body with 6 faces. 4. DOC-11 corpus cases TNP-01, TNP-02 and TNP-03 run on `solid.extrude` (instead of `test.brep_prism`) and meet their `expect`. 5. A kernel failure with `offending` edges (injected failing kernel) gives `Diagnostic::entities` resolving to those edges. 6. Preview: an extrude-cut of a 20-edge profile into a 200-face body, `ComputeMode::Preview`, median < 100 ms (criterion `budget/sol-extrude-preview`); full mode recorded. 7. `dialog::registration("solid.extrude")` exists and its `build(defaults)` round-trips through `read`. |
| SOL-02 | Extrude: all starts, directions and extents (§4; note §3): offset start, start from a plane, two sides, symmetric (half/whole), taper per side, to object (plane, face, body, vertex; offset; extend faces), all; planar-face profiles; text profiles | SOL-01, CON-03, BREP-14, SK-06 | P0 | Done | 1. ≥ 20 golden cases cover every start × direction × extent combination the dialog allows, incl. to-object on an inclined plane, a cylinder face, a body and a vertex, with and without offset and extend; analytic volumes within 1e-9 relative, curved targets within 1e-6. 2. A face that doesn't cover the profile fails `TargetMissed` with the "Extend faces" hint; extend on fixes it; a B-spline target with extend fails `ExtendUnsupported`. 3. "All" cuts through 3 stacked bodies in one direction and through both sides in two-sided mode. 4. Text "AgB" (SK-10 `regions().solid`) extrudes to 3 solids with 4 holes in total (closes SK-10 Done-when 2). 5. Editing distance, taper or extent type keeps the lateral and cap names (`Exact` resolution of a downstream fillet ref). 6. Full-mode extrude-cut of a 100-edge profile into a 200-face body, median < 80 ms (`budget/sol-extrude-cut`). |
| SOL-03 | Revolve: profiles, axis (sketch line, edge, construction or origin axis), angle, full, two sides, symmetric, to object; profile/axis validation | SOL-01, CON-03, BREP-14 | P0 | Done | 1. ≥ 8 golden cases (rectangle 90° and 360°, triangle with an edge on the axis → cone, two sides 30°/60°, symmetric 90°, each axis kind) match Pappus volumes within 1e-9 relative. 2. A profile crossing the axis fails `AxisCrossesProfile` naming the profile. 3. DOC-11 TNP-22 (edit to 360° → cap refs `NotFound`) runs on `solid.revolve` and meets its `expect`. 4. Revolve to a plane through the axis at 75° equals the 75° angle case within 1e-9. |
| SOL-04 | Primitives: box, cylinder, sphere, torus on a `PrimitiveFrame` (origin plane, construction plane, planar face), with `OpTarget` | SOL-01, CON-03, BREP-14 | P0 | Done | 1. Golden volume and area for each primitive (exact within 1e-9) and for a box joined to and cut from a body. 2. A box placed on a planar face follows the face after an upstream height edit (centroid within 1e-9). 3. Editing a box size keeps `XMin#0 … ZMax#0` names (`Exact`). 4. Each kind has a dialog with a distance manipulator anchor per size field. |
| SOL-05 | Fillet (constant, chord, variable; G1 only, G2 is KH-10 per QUESTIONS #55) and chamfer (equal, two distances, distance-angle) on raw BREP-06: edge sets per body, tangent-chain propagation (R-S04-1), failing-edge highlight | SOL-01, BREP-06, BREP-09 | P0 | Done | 1. ≥ 15 golden cases (cube edge, 4-edge tangent loop of a rounded rectangle, 3-edge vertex blend, variable 2→4 mm, chord 3 mm, all chamfer kinds, two bodies in one feature) within 1e-6 relative of reference volumes. 2. Picking one edge of a rounded-rectangle top selects all 8 tangent edges; with `tangent_chain = false` only that edge. 3. A radius larger than the adjacent face fails and `Diagnostic::entities` names exactly the failing edge(s). 4. DOC-11 TNP-11 and TNP-27 run on `solid.fillet` and meet their `expect`. 5. Preview of a 10-edge constant fillet on a 200-face body, median < 100 ms (`budget/sol-fillet-preview`). 6. KH-03 can start: S20's corpus harness drives `solid.fillet` through a public test entry point. |
| SOL-06 | Shell (remove faces, hollow, inside/outside/both, tangent chain), draft (one side, two sides, symmetric; tangent chain), scale (uniform, non-uniform), thicken (`OpTarget`) | SOL-01, BREP-07, BREP-16, BREP-14 | P0 | Done | 1. Shelled box (top removed, 1 mm) inside, outside and both, and a hollow box (no faces removed, R-S04-8) match analytic volumes within 1e-9 relative. 2. Shelling a filleted box with tangent chain removes the top and its fillet ring (face count checked). 3. A thickness above the smallest concave radius fails with the offending faces highlighted. 4. Draft of 4 box sides at 3°: measured angles 3° ± 1e-9 rad, analytic volume; symmetric draft about a mid-plane matches the analytic volume. 5. Uniform scale 2 → volume × 8 within 1e-12 relative; non-uniform (1, 2, 3) of a sphere r 10 → ellipsoid volume within 1e-9. 6. Thicken of a B-spline sheet by 1 mm is valid and within 0.5 % of area × t. 7. TNP-X3 and TNP-X6 meet their `expect`. 8. Shell preview of a 100-face body, median < 100 ms (`budget/sol-shell-preview`). |
| SOL-07 | Combine (join/cut/intersect, keep tools), split body (plane, face, body; extend tool), split face (surface tool; sketch curves along a vector) | SOL-01, BREP-16, CON-03 | P0 | Done | 1. Combine golden cases for each op, with and without keep tools; consumed tools are `Consumed { into }` (TNP-23 meets its `expect`). 2. Splitting a box by a plane gives pieces in the naming-note order (piece 0 keeps the `BodyId`); TNP-21 meets its `expect`. 3. Split face by a sketch line projected along the sketch normal and by a plane: two faces whose areas match within 1e-9. 4. A tool that doesn't cross fails `NoSplit` with the "extend the tool" hint. |
| SOL-08 | Pattern (rectangular, circular; bodies and features; suppressed instances; identical compute) + mirror (bodies new/join, features) on `SolidTool` and `mirrored` (R-S04-6) | SOL-01, SOL-02, CON-03, BREP-14 | P0 | Done | 1. 3×4 rectangular and 6-instance circular body patterns (incl. symmetric and 2 suppressed instances): body count and total volume exact. 2. A 10×10 feature pattern of a Ø3 cut-extrude in a 100×100×5 plate: analytic volume within 1e-9 relative, **one** boolean call (kernel call counter), instances share TShapes (`Shape::transformed` only), full recompute < 300 ms (`budget/sol-pattern-100`). 3. Mirror of a body and of an extrude boss: mirrored volume equal, BRepCheck valid (orientation). 4. TNP-17 and TNP-18 meet their `expect`. |
| SOL-09 | Move/copy (free, translate, rotate, point-to-point), align, offset face (R-S04-5), replace face, delete face (heal / remove), press-pull dispatch | SOL-01, BREP-07, BREP-16, CON-03 | P0 | Done | 1. Move keeps names: a fillet ref on a moved body resolves `Exact`; a copy is a new body named `instanced(self, slot)`. 2. Align face-to-face makes the faces coplanar within 1e-9 mm. 3. Offset of a box top by +2 mm changes the height by 2 within 1e-9; a cylinder face +1 mm changes the radius by 1 within 1e-9. 4. Replace face onto an offset plane matches the analytic volume. 5. Delete of a fillet face with heal gives the pre-fillet volume within 1e-6; without heal gives an open body with the expected free edges. 6. `dispatch` maps a face, an edge and a profile selection to `solid.offset_face`, `solid.fillet` and `solid.extrude`. |
| SOL-10 | Sweep: path (chain), guide rail (none/scale/stretch), guide surface (R-S04-7), distance, taper, twist, orientation | SOL-01, BREP-15, CON-03 | P1 | Todo | 1. Circle swept along a line and along an arc within 1e-6 of the analytic volume; along a 3D spline within 0.1 % of the reference. 2. Twist 90° and taper 5° match reference volumes within 1e-4 relative. 3. Guide-rail scale vs stretch give the reference volumes; a guide-surface sweep stays normal to the surface (sampled angle < 0.1°). 4. Distance 0.5 gives half the length (volume within 1e-6). 5. A self-intersecting sweep fails with a clean error. 6. Lateral faces of a multi-edge path have the naming-note names (`Split{self, i, m}` or spine tags). |
| SOL-11 | Loft feature + dialog (profiles, rails, centerline, end conditions, point mapping) on the S20 loft builder (`robust::loft::build`), not raw `ThruSections`; live correspondence lines from `loft::prepare`; `LoftDiagnostics` in the dialog | SOL-01, KH-08, KH-18, SK-06 | P1 | Todo | 1. ≥ 98 % success on the loft corpus (KH-05) driven through `solid.loft` (matches KH-18 Done-when 1). 2. Preview uses `RetryPolicy::interactive()`; correspondence lines come from `prepare` in < 20 ms on corpus median. 3. Warnings (`HighTwist`, `CrossingCorrespondence`) appear as feature warnings. 4. TNP-X4 meets its `expect`. |
| SOL-12 | Hole: placements (sketch points, on face with two edge distances, at point), types (simple, counterbore, countersink), tap types (simple, clearance, tapped, taper tapped), drill point, extents; ISO metric coarse + fine table (`standards`); cosmetic threads (`cosmetic_threads`) and modeled tapped threads via the BREP-11 helper | SOL-02, BREP-11 | P1 | Todo | 1. Golden cases for every hole type × {simple, clearance, tapped} × {flat, 118°}, on distance, to-object and all extents, within 1e-9 relative (analytic). 2. Holes at 3 sketch points; deleting point 1 keeps the names of the others (TNP-X5 meets its `expect`). 3. The ISO metric table covers ISO 261 M1.6–M64 coarse and fine; the test recomputes D1, D2, d3 from ISO 68-1 within 1e-3 mm and checks tap drills and ISO 273 clearances against the source table. 4. A modeled M6 tapped hole is valid and its minor diameter measures 4.917 ± 1e-3 mm. 5. `cosmetic_threads` lists a tapped hole with its size and length. 6. `clearance` = 0.2 on an M6 clearance hole and a modeled thread enlarges the hole radius / thread diameters by exactly 0.2 mm (measured); a parameter-driven `clearance` recomputes; default 0 reproduces pure ISO values (QUESTIONS #53). |
| SOL-13 | Rib and web: open sketch curves, thickness and side, depth to next or distance, normal-to-sketch and parallel-to-sketch modes (parallel: lines only), curve ends extended to the body | SOL-18, CON-03 | P1 | Todo | 1. A straight rib between two walls of a U-channel fills to the floor (to next): analytic volume within 1e-9. 2. A curved rib in normal mode, 2 mm thick, matches the reference within 1e-6. 3. A web of 3 crossing lines gives one fused body with the expected face count. 4. Parallel mode with an arc fails with a clear `Invalid` error. |
| SOL-14 | Boundary fill: tools (bodies, planes, faces), cell enumeration (`split` of all tools), cell selection by fingerprint (`CellRef`), `OpTarget`, remove tools | SOL-07 | P1 | Todo | 1. Three intersecting cylinders give the expected cell count; filling the common cell gives the analytic Steinmetz volume within 1e-6. 2. A cell selection survives a 10 % tool dimension edit (fingerprint match, warning only above the naming thresholds). 3. A cell that no longer exists gives a clear error. |
| SOL-15 | Rule fillet (rule-based edge selection: all edges, between faces/features, rules for concave/convex) | SOL-05 | P2 | Todo | Refine before Phase 5: rule set, its naming (edges re-derived per recompute), and Done-when. |
| SOL-16 | Derive / insert design as component | DOC-07, ASM-01, SOL-01 | P2 | Todo | Refine before Phase 5: external design reference, update semantics, naming via `BodyNaming::for_import`. |
| SOL-17 | Sheet metal (flange, bend, unfold, flat pattern) | SOL-01, SOL-06 | P3 | Todo | Placeholder: split into 1–3-day WPs before Phase 6. |
| SOL-18 | Thin extrude and thin revolve: open and closed chains, side 1/2/center (R-S04-3), names `ThinInner`/`ThinOuter`/`ThinEnd` | SOL-02, SOL-03 | P0 | Done | 1. Thin extrude of an open line+arc chain (length L, t 2 mm, h 10 mm) in each side mode: volume L·t·h within 1e-9 relative. 2. A closed rectangle thin extrude gives a ring with the analytic volume. 3. A thin revolve of an open chain matches Pappus within 1e-9. 4. Editing the thickness keeps all names (`Exact`). |
| SOL-19 | Coil (all size modes, sections, positions, taper) and pipe (path chain, sections, hollow, distance) | SOL-04, SOL-10, BREP-11 | P0 | Todo | 1. Circular-section coil (d 20, 5 turns, pitch 4, section 2): volume within 0.1 % of section area × helix length. 2. Square and triangle sections and inside/outside positions give valid solids with reference volumes within 0.1 %. 3. Hollow pipe along a 3-segment path: volume within 1e-6 of the analytic value. 4. A coil whose pitch is smaller than its section size fails with a self-intersection error, never an invalid body. |
| SOL-20 | Fillet/chamfer on S20's robust chain: `robust::fillet::fillet` with `FilletDiagnosis` (failing edges highlighted, largest workable radius in the message), `max_radius` as a background job, interactive policy in preview | SOL-05, KH-03 | P0 | Todo | 1. On the KH-02 fillet corpus driven through `solid.fillet`, the success rate equals KH-03's recorded rate (no loss in the feature layer). 2. 100 % of failures highlight ≥ 1 edge and show a max radius when `max_radius` returns one. 3. Preview never exceeds the interactive deadline (test with an injected slow strategy). |
| SOL-21 | Setback corners for fillets (S20 KH-19 corner patches): `CornerType::Setback` with setback distances | SOL-20, KH-19 | P1 | Todo | 1. The `corner3` subset driven through `solid.fillet` with setback reaches KH-19's success rate (≥ 95 %). 2. TNP-27 still meets its `expect` with setback on. |
| SOL-22 | Silhouette split (R-S04-4): faces split along the silhouette for a view direction, optional body split | SOL-07 | P1 | Todo | 1. A sphere r 10 split for +Z gives two faces of area 2π·100 within 1e-6 relative. 2. A cylinder seen side-on splits into two half-cylinders; with `split_body` the body splits into two solids of equal volume within 1e-9. 3. A B-spline face split matches the sampled silhouette within 1e-4 mm. |
| SOL-23 | Pattern on path (spacing/extent, start, orientation), face pattern (boss/pocket on a planar support), adjust compute type | SOL-08, CON-03 | P0 | Done | 1. 8 instances along a spline path with path-direction orientation: instance frames within 1e-9 of `CurveEval` frames. 2. A face pattern of a boss (6 faces) on a plate gives the analytic volume; a pocket the same. 3. Adjust compute of a to-object extrude seed across a sloped target gives per-instance heights matching the reference within 1e-6. |
| SOL-24 | Component operations: "New component" body op (`new_component`, S11 request), component pattern and mirror (occurrences), cross-component participants | SOL-08, ASM-02 | P0 | Done | 1. Extrude with "New component" creates one component and one body in one undo entry. 2. A circular pattern of an occurrence creates 6 occurrences with the right transforms; mirror creates a mirrored occurrence. 3. A cut whose participant is in a sub-component (assembly context) works and is undone in one step. |
| SOL-25 | Thread feature on cylindrical faces (cosmetic/modeled, full length or length+offset, handedness), shared geometry with tapped holes; ISO metric + UNC/UNF tables (BSP/NPT only on request; QUESTIONS #54); per-feature `clearance` (QUESTIONS #53) | SOL-12 | P1 | Todo | 1. ISO M3, M6, M10, M20 external modeled threads on cylinders: crest pitch within 1e-6 mm and diameters within 1e-4 mm of ISO 68-1 (BREP-11 checks reused). 2. A thread size is inferred from the face diameter (Ø6 → M6×1). 3. ¼-20 UNC and ¼-28 UNF pass the same checks. 4. `clearance` on a modeled thread shifts diameters by the given radial amount. |
| SOL-26 | Emboss / deboss of profiles and text on planar faces (project) and on cylindrical and conical faces (wrap by unrolling) | SOL-02, SK-10 | P1 | Todo | 1. Text "AgB" embossed 1 mm on a plane: volume = text area × 1 within 1e-6. 2. A 10 mm square wrapped on a cylinder r 20 and debossed 1 mm: volume within 1e-4 relative of the analytic value. 3. Editing the depth keeps glyph lateral names. 4. A freeform target face fails with `NotYet { what: "emboss on freeform faces" }`. |
| SOL-27 | Surface repair tools (QUESTIONS #1, P1): patch (planar and N-sided, R-S04-9), stitch (sew to a solid when closed), unstitch, reverse normal | SOL-01, BREP-10 | P1 | Todo | 1. Stitching the 6 faces of an imported cube with 1e-4 mm gaps gives a valid solid with volume within 1e-6. 2. A planar patch closes a hole in a sheet; an N-sided G1 patch on a 4-sided curved gap is valid with boundary G1 deviation ≤ 0.5°. 3. Unstitch then stitch round-trips volume and face count. 4. Reverse normal flips the chosen faces (face orientation test). |
| SOL-28 | Ruled, extend, trim, untrim surfaces | SOL-27 | P2 | Todo | Refine before Phase 5 (needs S02 requests for `BRepLib::ExtendFace` and ruled surfaces between edges). |
| SOL-29 | Fusion-parity harness and recompute budget: owner Fusion designs (`$AMG_TESTDATA_DIR/fusion/`) rebuilt by Rust test scripts and compared by volume/area/bbox with the STEP exports; scripted 50-feature part bench | SOL-02, SOL-03, SOL-04, SOL-05, SOL-06, SOL-07, SOL-08, SOL-09 | P0 | Review | 1. Every supplied owner design whose features exist is rebuilt; volume and area match the Fusion STEP within 1e-4 relative and bbox within 1e-3 mm, or the difference is explained in the log. Skipped cleanly if the folder is missing. 2. `budget/sol-recompute-50`: a 50-feature part (sketches, extrudes, cuts, fillets, a shell, a pattern, holes) recomputes after an edit to the first sketch in < 500 ms on the reference machine (vision budget); the bench prints the 5 slowest features. |

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
   **Resolved 2026-11-09 (owner):** per-feature `clearance` field on Hole and Thread, default 0; per-design technology defaults only once S16 printer profiles exist.
7. (2026-11-08, owner, QUESTIONS #54) UNC/UNF tables: needed, or ISO metric only?
   **Resolved 2026-11-09 (owner):** ISO metric + UNC/UNF (SOL-25); pipe threads only on request.
8. (2026-11-08, owner, QUESTIONS #55) G2 (curvature-continuous) fillets: needed at all, and if so only with the Phase 7 Rust fillet engine?
   **Resolved 2026-11-09 (owner):** needed; planned with the Phase 7 Rust fillet engine (S20 KH-10). SOL-05 ships G1.

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
- **Leftovers:** full-round fillet is not planned; G2 fillets wait for S20 KH-10 (QUESTIONS #55, 2026-11-09); emboss on
  freeform faces, cross-component BodyOps beyond SOL-24, and SOL-15/16/17/28 are refined before
  their phase.

### 2026-11-09 - owner answers moved in
- QUESTIONS #53 (`clearance` field on Hole/Thread, default 0; SOL-12 Done-when 6, SOL-25 Done-when 4), #54 (ISO metric + UNC/UNF, pipe threads on request; SOL-25), #55 (G2 fillets needed; SOL-05 ships G1, G2 planned in S20 KH-10). Open questions 6-8 marked resolved.

### 2026-11-25 - SOL-01 implemented (lane A)
- **Landed.** `amg_features::dialog` (S11 §B as specified plus `new_component`,
  `DispatchRegistration`, `registration`/`registrations`/`dispatches`, small value helpers);
  `amg_features::solid::{common, error, extent, profile, extrude}`: `BodyOp`, `OpTarget`,
  `ToolBody`, `SolidTool`, `ToolRegistration` + `register_tool!` + `tool_of`, `apply_body_op`
  (§3), `simplify` (`unify_same_domain` faces + edges, history composed, split order re-applied),
  `input_solid`, `resolve_subs`, `op_kernel`, `profile_faces`, `to_curve2d`, `tag_of`,
  `region_source`, `cap_source`, `SolidError` + `kernel_failure`, the `Extent`/`ExtentSide`/
  `Direction`/`Start`/`ThinSpec` data, and `solid.extrude` (feature, tool, dialog registration).
- **BodyOp rules as implemented.** Participants are followed through `Evaluated::successors`
  (dropped with a warning when gone), deduplicated, and must be B-rep solids of the context
  component. Join: participants the tool touches (box test, then `distance` ≤ linear
  tolerance) are fused with the tool in one N-ary boolean; each result solid keeps the id of its
  first contributing participant (by history), other contributors are `Consumed { into }`,
  tool-only solids become new bodies. Cut / Intersect: one boolean per participant (box-disjoint
  ones skipped / deleted); "untouched" = every face `Kept` and no tool output; piece 0 keeps the
  id, further pieces are `CreatedFrom { BodyId { self, k } }` with `k` counted from 1 across
  participants (so the first split body's piece `k` is slot `k`); an empty result is `Deleted`
  with a warning. NewBody with several tool solids: `per_solid` of an identity history.
- **Extrude (minimal).** One side, `Extent::Distance`, start at the profile, no taper, no thin;
  every other option fails with `NotYet` (SOL-02/SOL-18). A negative distance or `flip` reverses
  the direction. Regions of one sketch that share a boundary curve (source or `also`) form a
  group; each group's prisms are fused and unified into one tool solid; disjoint groups go into
  one compound named through an index-offset history (checked against the compound's counts).
  Naming per note §5.2: laterals `Gen { self, Lateral#0, SketchCurve }` from the builder tags,
  caps `StartCap#0` / `EndCap#0` from `Region(p)`.
- **Done-when evidence** (`crates/amg-features/tests/solid_extrude.rs`, 26 tests; goldens
  `tests/golden/solid/sol01_*.toml`):
  1. `dw1_*`: new body 10×20×30 = 6000 mm³ (1e-9 rel), 6 faces; join onto a 40×40×10 plate
     20000 mm³ / 11 faces; cut through 14000 / 10; intersect 2000 / 6 (goldens `new_body`,
     `join_box`, `cut_box`, `intersect_box`).
  2. `dw2_*`: join of 2 keeps the first id and `successors(second) == [first]`; a cut splitting a
     bar keeps piece 0 (6000) and creates `BodyId { self, 1 }` (3000) with `derived_from`;
     `NothingToCut`; a mesh participant fails `WrongBodyKind`; join touching nothing creates a
     body and warns.
  3. `dw3_*`: a rectangle split by a line gives 2 regions → 1 body, 6 faces, 6000 mm³.
  4. `dw4_tnp01/02/03_*`: the three cases rebuilt on sketches + `solid.extrude` (B1 as a
     100×60 extrude, height edit 40 → 80; B2 as a point/line polygon sketch, insert point 5,
     delete point 2) with `test.brep_fillet`; each fillet ref resolves `Exact` to the expected
     edge (centroid and length within 1e-6).
  5. `dw5_*`: a kernel that fails `boolean` with `OpFailed { offending: [edge 0, edge 3] }`
     (`tests/solid_common/failing.rs`); `ctx.highlights()` holds two edge `TopoRef`s that
     resolve `Exact` to indices 0 and 3. `the_engine_puts_highlights_into_the_error_diagnostic`
     checks the engine copies highlights into `Health::Error` entities.
  6. `benches/sol_extrude.rs` (`budget/sol-extrude-preview`): 20-gon cut through a 198-gon prism
     (200 faces), release profile on the shared dev machine: preview median 63.1 ms (25-run
     sample; criterion 60.9 ms), full mode 60.1 ms (criterion 59.1 ms). `AMG_STRICT_TIMING=1`
     asserts < 100 ms and passed.
  7. `dw7_*` and `every_dialog_registration_round_trips`: `registration("solid.extrude")` exists,
     `read(build(defaults)) == defaults` (also for edited values and over all registrations).
- **Deviations.**
  - `RecomputeCtx::resolve_profile` / `region_source` (naming note §10) still don't exist; SOL-01
    resolves profiles in `solid::profile` (id, else `match_fingerprint` with a `FingerprintOnly`
    warning) and builds `Region(p)` with `solid::region_source`.
  - `kernel_failure` also marks the entities with the new `RecomputeCtx::highlight`, because a
    failed compute returns no `FeatureOutput` (the diagnostic pushed to `out` alone would be
    lost). S01 Interface changes 2026-11-25.
  - Preview: the "no BRepCheck" rule is `op_kernel(ctx)`: in `ComputeMode::Preview` on the
    default kernel, a static copy with `Validate::Off`. S20's `RetryPolicy::interactive()`
    doesn't exist yet, so nothing else differs. In release builds validation is already off
    (`DebugOnly`), so preview and full times are equal.
  - `tangent_chain_edges` / `tangent_chain_faces` are not in SOL-01 (first used by SOL-05/06).
  - `ProfileSource::Face` exists, but planar-face profiles fail `NotYet` (SOL-02 scope).
  - Extra public items: `BodyPicks`, `op_kernel`, `region_source`, `cap_source`, `tag_source`,
    `ExtentSide::distance`, `Extrude::distance`.
- **Interface changes elsewhere.** S01: `RecomputeCtx::{before, warn, highlight, highlights}`
  and the engine copying highlights into the error diagnostic (additive, recorded in S01).
  S11: notes on `amg_features::dialog` and the `EntityFilter` bit-layout mismatch with
  `amg_ui::select::EntityFilter` (no `amg-ui` change). `amg-features` gains `inventory` (dep) and
  `toml` + `amg-doc/test-features` (dev-deps), all existing workspace dependencies
  (THIRD_PARTY.md rows updated).
- **Leftovers.** `amg_ui::select::EntityFilter` → re-export (lane D, UI-05); S20 interactive
  retry policy in preview; `resolve_profile`/`region_source` on `RecomputeCtx` if another lane
  needs them; SOL-02 builds on the `Extent` data (starts, two sides, symmetric, taper, to object,
  all, planar-face profiles).

### 2026-11-25 - SOL-05 implemented (lane A)
- **Landed.** `amg_features::solid::fillet`: `solid.fillet` (`Fillet`, `FilletSet`, `FilletRadius::
  {Constant, Chord, Variable}`, `VarRadius`, `CornerType`) and `solid.chamfer` (`Chamfer`,
  `ChamferSet`, `ChamferKind::{Equal, TwoDistances, DistanceAngle}`) with the Interfaces shapes,
  both `register_feature!` v1 with dialog registrations (`solid.fillet`: edges, radius type,
  radius / start + end radius, tangent chain; `solid.chamfer`: edges, type, distance, distance 2,
  angle, reference face, tangent chain). `solid::common::tangent_chain_edges` (R-S04-1 on
  `edge_ends`). Public `Fillet::plan` / `Chamfer::plan` (`BodyEdges`, `PlannedEdge`: the edges
  passed to the kernel per body, picks first with their key, then propagated edges) for the
  UI-05 preview highlight. `solid::fillet::harness::run_fillet` (`FilletCase`, `FilletRun`,
  `FilletRunError`) for S20's corpus harness.
- **Behaviour.** Sets resolve with `DropMissing` and are grouped per body; one BREP-06 call per
  body with all its edges in pick order (OCCT numbers contours that way). With `tangent_chain`
  each pick grows along edges whose tangents continue across a shared vertex within
  `CHAIN_ANGLE` = 1e-3 rad (only edges between two distinct faces); a propagated edge gets its
  seed's set. Variable radii: samples sorted, positions in 0..1, a missing end held at the
  nearest sample's radius. Chamfer reference faces resolve `AllOrError` and must border every
  edge of their set. Failures go through `kernel_failure`, so `LocalOpFailed::offending` edges
  become the error highlight. Naming: `from_history` of the input plus `selection(key)` for picked
  edges only, so blends of propagated edges are named from the edge (`Topo*`), as the naming
  note §5.2 says; corners and neighbours by the default rules.
- **Done-when evidence** (`crates/amg-features/tests/solid_fillet.rs`, 28 tests, 24 with OCCT;
  goldens `tests/golden/solid/sol05_*.toml`, 17 files, rel 1e-6; 2 unit tests in the module):
  1. 16 golden cases (17 files: two bodies give two), each BRepCheck-valid; 14 also match a closed
     form within 1e-6: cube edge, chord 3 mm (`r = 3/√2`), two sets 2 mm + 3 mm, two bodies in one
     feature, the 3-edge vertex blend (`(1−π/4)r²(A+B+C−3r) + r³(1−π/6)`), the box top loop
     (Pappus on the mitred loop), the slot's 4-edge tangent loop and the rounded rectangle's
     8-edge loop from one pick (Pappus with the spandrel centroid), a concave edge, a cylinder rim,
     chamfers equal / two distances / distance-angle (plus the top-face area showing `d1` is on the
     reference face) and the chamfered slot loop. Variable 2 → 4 mm and the 3-edge chamfer vertex
     are golden-only with bounds.
  2. `dw2_*`: one pick on the rounded-rectangle top plans 8 edges (pick first with key 1, 7
     propagated, all at z = 10); `tangent_chain = false` plans 1; a box edge stays alone.
  3. `dw3_*`: a 7 mm radius on the top edge of a 5 mm plate fails and `Health::Error` entities are
     exactly that edge (resolving `Exact`); with a second, workable set only the failing edge is
     highlighted.
  4. `dw4_tnp11_*` (inserted `solid.fillet` on the same edge → the downstream ref is
     `NotFound`, feature in error) and `dw4_tnp27_*` (3-edge `solid.fillet`, a probe on the corner
     patch, box height 40 → 50: `Exact`, sphere, centroid (97.5, 2.5, 47.5), area 39.26991 within
     1e-4), both on `test.brep_box` B1 as in the corpus. Also: a blend face referenced downstream
     stays `Exact` when the radius goes 2 → 4 mm.
  5. `benches/sol_fillet.rs` (`budget/sol-fillet-preview`): 10 vertical edges (every 19th) of a
     198-vertex star prism (200 faces), r 0.3 mm; release profile on the shared dev machine:
     preview median 56.7 ms (25-run sample; criterion 59.1 ms), full mode 55.7 ms (criterion
     62.8 ms). `AMG_STRICT_TIMING=1` asserts < 100 ms and passed. An earlier run while another
     worker was building gave 63.9 ms (criterion 105 ms), so the budget is load-sensitive.
  6. `dw6_*`: `harness::run_fillet` builds a one-body design from a `.brep`-style `Shape` and
     `(EdgeId, FilletSpec)` pairs (the `KernelOp::Fillet` contents), runs `solid.fillet` with the
     caller's kernel and mode, and returns the shape + naming, or the error with the highlighted
     edges in the case's index space (checked on a radius-too-large case). A proptest (16 random
     radii/edges on a box) gives a valid solid or a `Kernel` error, never a panic.
- **Deviations.**
  - A regular 198-gon (the SOL-01 bench body) can't be used for Done-when 5: OCCT refuses its
    1.8° vertical edges as smooth. The bench uses a star with the same face count.
  - `tangent_chain = false` changes only the edges we pass: OCCT's `MakeFillet` still extends a
    contour along its tangent chain, so the geometry is the same; the flag matters for the plan,
    the highlight and which blends carry `Selection` names.
  - The dialog edits the first set only (S11 §B has no repeated groups); further sets are kept on
    edit. An empty edge pick builds (the field's `min: 1` keeps OK off), so the all-registrations
    round-trip test still passes; such a feature fails to compute with `InvalidValue { edges }`.
  - Two-distance / distance-angle chamfers need one reference face bordering every edge of the
    set (no per-edge flip); otherwise `InvalidValue { face }`.
  - `CornerType::Setback` fails `NotYet` (SOL-21). Kernel warnings of the op are not copied into
    feature warnings (as in SOL-01).
  - `tangent_chain_faces` is still not written (first used by SOL-06).
  - `tests/solid_common` gains `check_golden_as(wp, rel, case, shape)`; `check_golden` is
    unchanged for SOL-01.
- **Interface changes elsewhere.** None: only public `amg-brep` / `amg-doc` APIs are used (the
  harness builds its input state with `Evaluated::set_body`). No new dependency.
- **Leftovers.** SOL-20 (S20 robust chain, `FilletDiagnosis`, max radius); SOL-21 (setback);
  G2 with KH-10; a per-edge chamfer flip or multi-set dialog UI if UI-05 wants one;
  `tangent_chain_faces` (SOL-06).

### 2026-11-26 - SOL-02 implemented (lane A)
- **Landed.** `solid::extrude` is now a module: `extrude/mod.rs` (feature, tool, start and side
  evaluation), `extrude/sides.rs` (one profile face on one side: distance, "All", to object),
  `extrude/dialog.rs` (the schema). `solid::extent` gains target resolution
  (`resolve_target`, `Target`, `BodyFace`; crate-private, reusable by SOL-03/12/13): B-rep faces,
  vertices and bodies through DOC-06, construction/origin planes and points through CON-03.
  `solid::profile` resolves planar `EntityRef::Topo` faces as profiles (`ProfileSource::Face`,
  each its own group; laterals take the body edge's current name via the new public
  `face_edge_source`, caps `Sel(key)`). `solid::error::with_hint` (crate-private) appends a
  `SolidError` hint to the `InvalidValue` message.
- **Behaviour.** Side 1 goes along the first profile's normal (flipped by `flip`), side 2 the
  other way; a negative distance reverses its side (as in SOL-01). Start: `Offset(e)` moves each
  profile face along the first profile's normal (not affected by `flip`); `Object` takes a plane
  or planar face parallel to the profile, or a point (vertex, sketch or construction point), plus
  an offset; anything else is `Invalid { start }`. Two sides and symmetric (half or whole length;
  side 1's distance and taper both ways) are two prisms, fused and unified with the other
  profiles of their group, so untapered sides merge into one block (6 faces for a rectangle).
  Taper per side through `PrismOpts::taper` (positive grows the profile along its side; a kernel
  failure with a taper adds "Reduce the taper angle."). To object (note §3.2): a plane (or planar
  face) parallel to the profile, or a point, gives the exact prism (offset = farther along the
  side); otherwise a prism 1 mm past the target's far side is split (`BrepKernel::split`) by the
  target: an inclined planar face itself, or (with `extend`, or for a construction plane) a square
  on its plane covering the swept region; a curved face itself, or with `extend` its untrimmed
  surface (cylinder and cone over the swept region's axial range, cones stopping short of the
  apex; sphere and torus whole; B-spline and other surfaces fail `ExtendUnsupported`); or the
  whole target body. The piece holding the prism's start cap is kept; one piece, or a kept piece
  that still holds the end cap, is `TargetMissed` (message "the target does not cover the
  profile. Turn on Extend faces.", target highlighted); a target behind the start is
  `Invalid { extent }`. Offsets on curved faces go along the face's outward normal, on bodies
  outwards (`BrepKernel::offset` of the sheet / solid). "All": 1 mm past the farthest bounding-box
  corner of every solid body of the context component along the side; `Invalid` if none lies
  ahead. Naming: `Lateral#side`, `StartCap#side`, `EndCap#side` (side 1 through
  `NamingBuilder::role` and explicit cap names); faces a target leaves on a split prism are
  `EndCap#side` too, so switching the extent type keeps the cap name.
- **Done-when evidence** (`crates/amg-features/tests/solid_extrude_sol02.rs` and
  `solid_extrude_sol02/occt.rs`, 21 tests, 3 of them without OCCT; goldens
  `tests/golden/solid/sol02_*.toml`, 34 files):
  1. 31 golden cases (34 files with the 3 text solids), each BRepCheck-valid and matched to a
     closed form: offset start; start from a face (with and without offset), a vertex and a
     construction plane; two sides; two sides with tapers 3° / −2° and one side ±5° (tapered
     rectangle `abh + (a+b)h²tanθ + 4/3 h³tan²θ`); symmetric half, whole and tapered; to a
     construction plane (also with offset −5), a parallel face, a vertex (+3), a plane with taper;
     to an inclined face (20°; with offset 2, i.e. 2/cos α higher; with extend on a face that
     misses; the body as target); to a cylinder face (radius 20; offset 2 → 22; an extended short
     cylinder; the cylinder body; chord-area closed form within 1e-6); to a B-spline face
     (z = 30 + 0.1x, 6100 mm³ within 1e-6); two sides to a plane and a vertex; offset start to a
     plane; a planar-face profile joined onto its box (19200 mm³, 6 faces); "All" cut plates.
     Analytic cases at 1e-9 relative, curved ones at 1e-6.
     `dw1_every_start_direction_extent_combination` runs all 42 combinations the dialog allows
     (3 starts × {one side × 3 extents, two sides × 3 × 3, symmetric half/whole}) with exact
     volume, z range and 6 faces each.
  2. `dw2_*`: a cylinder face 10 long under a 20 wide profile fails with "does not cover the
     profile … Turn on Extend faces" and the face in the `Health::Error` entities; `extend` fixes
     it (analytic volume within 1e-6); the same for a small inclined planar face (1e-9); a
     B-spline target with extend fails "can't be extended". A target behind the profile is
     refused; the flipped extrude reaches it.
  3. `dw3_*`: "All" cuts a 10 × 20 hole through 3 stacked plates (each 7000 mm³, 10 faces) and
     leaves a plate on the other side untouched; two-sided "All" cuts plates on both sides.
  4. `dw4_text_agb_*`: "AgB" (`TextBlock::regions().solid`) extrudes 2 mm to exactly 3 valid
     solids with 4 holes in total, volume 2 × net area within 1e-6 (closes SK-10 Done-when 2;
     S03 log entry 2026-11-26).
  5. `dw5_*`: a fillet on the front top edge (`Bound { Lateral, EndCap#0 }`) of a 100 × 60
     extrude stays `Exact` through distance 40 → 80, taper 0 → 3°, to a construction plane, to a
     parallel face, "All", tapered to the plane, and back (edge z checked each time). Two sides: a
     fillet on the bottom edge (`EndCap#1`) stays `Exact` when side 2's distance changes and when
     it becomes "All". Side-1 faces are named `#1` (5 of 10 names of a tapered two-sided block).
  6. **Not met.** `benches/sol_extrude_cut.rs` (`budget/sol-extrude-cut`): a regular 100-gon
     (r 20) cut 30 mm through the SOL-01 198-gon prism (200 faces), full mode, release profile on
     the shared dev machine (another worker building): median 120.9 ms (25-run sample; criterion
     144.8 ms), budget < 80 ms; `AMG_STRICT_TIMING=1` fails. Breakdown: the extrude tool (profile
     faces, prism, naming) 2.9 ms; `apply_body_op` (SOL-01) ~120 ms, of which the OCCT boolean
     `Cut` alone is 105–127 ms (`check_inverted: false` 104.9, `use_obb` 105.6, serial 154.8 ms;
     8 or 16 OCCT threads no faster), `unify_same_domain` 30–34 ms, naming `sub_geom_all` 0.6 ms.
     The boolean alone is over budget, so this needs kernel-side work (S02/S20 boolean
     performance) or a re-measurement on the reference machine; the SOL-01 preview bench is
     unchanged (preview median 63.2 ms, full 61.9 ms, criterion 82.7 / 81.2 ms under load).
  Also: a proptest (16 random rectangles, distances ±40 mm, tapers ±20°, one or two sides) gives
  a valid solid or a clean error; dialog tests (conditional fields hidden for the defaults,
  unique labels, option labels distinct from field labels, `read(build(v)) == v` for every start,
  extent and direction).
- **Deviations.**
  - A start object must be parallel to the profile (planar start faces are translated to, not
    projected onto); inclined or curved start faces are refused (note §3.1 allows this).
  - Editing only one side's taper of a two-sided extrude un-merges the coplanar laterals, so refs
    into them resolve with DOC-06's "merge that no longer happens" warning (tested). Done-when 5
    holds for one-sided edits and for two-sided distance and extent edits.
  - The SOL-01 test `unsupported_options_fail_with_not_yet` used symmetric as its `NotYet`
    example; it now uses thin (SOL-18), because symmetric is implemented.
  - Planar-face profiles share TShapes with their body (no copy); the join test shows the
    booleans accept it.
  - Dialog: option labels differ from field labels ("Fixed distance", "Offset plane", …) because
    the UI-05 harness finds widgets by label; every conditional field is hidden for the defaults
    (the rule of UI-05's `dw1_extrude_fields_*`, mirrored by a unit test here). Side 2 has its own
    `side2_*` fields; symmetric disables the extent chooser. The amg-ui tests were not run in this
    WP (lane D crate); the schema gains fields, so run `amg-ui`'s `feature_dialog` tests at merge.
- **Interface changes elsewhere.** None to shared crates or other lanes' code. S03 (lane B): the
  SK-10 row's Done-when 2 note and a log entry. New public item: `solid::face_edge_source`. No new
  dependency.
- **Leftovers.** Done-when 6 (`budget/sol-extrude-cut`, 121 ms vs 80 ms; the OCCT boolean alone
  takes ~105 ms): kernel boolean performance (S02/S20) or the reference-machine measurement.
  Projected starts on inclined or curved faces; the sign convention of offsets on
  curved targets (outward normal) against Fusion (check with the owner's designs in SOL-29); thin
  extrude (SOL-18); manipulator anchors for the start offset and side-2 taper (UI-06).

### 2026-11-26 - SOL-08 implemented (lane A)
- **Landed.** `amg_features::solid::pattern`: `solid.pattern` (`Pattern`, `PatternObjects`,
  `PatternKind`, `PatternDir`, `Spacing`, `AngleSpan`, `PathOrientation`, `ComputeType`, the
  Interfaces shapes) and `solid.mirror` (`Mirror`), both `register_feature!` v1, with dialog
  registrations (`pattern/dialog.rs`; group "create", order 60/61, not pinned). Extra public
  items: `Pattern::instances(ctx)` (the produced `(k, Transform)` list, used by tests and the
  bench) and `pattern::MAX_INSTANCES` (10 000).
- **Behaviour.** Instance numbers follow the naming note §5.2: rectangular `k = i + (j << 16)`,
  circular `k` = position; the seed is `k = 0` and is never produced again. Counts are
  integral and ≥ 1 (`Invalid` otherwise) and include the seed; `symmetric` puts `count`
  instances on each side (`2 count − 1` in all), the negative side setting the field's top
  bit (`0x8000 | p` per rectangular field, `0x8000_0000 | p` circular) so numbers stay stable
  under count edits. `Spacing::Extent` / `AngleSpan::Extent` is seed-to-last; `AngleSpan::Full`
  is `360°/count` (symmetric ignored). Directions and axes resolve through CON-03: `line_of`
  (straight edges, axes, sketch lines), else `axis_of` (cylinder/cone/torus faces, circles).
  Suppressed numbers are removed from the list (`0` ignored).
  - Body patterns: `Shape::transformed` per instance, `BodyNaming::instanced(self, k)`, new body
    `BodyId { self, k }` (several source bodies: slots in order instead, since `k` uses all 32
    bits). Body patterns take no `target`.
  - Feature patterns (identical compute): each seed's tool is built once through `tool_of` in a
    sub-`RecomputeCtx` whose feature is the seed (so tool names are the seed's) against the
    state just before the pattern; instances are `Shape::transformed` + `instanced(self, k)`,
    gathered in one compound (`compound_of`, no copy) and applied with **one**
    `apply_body_op` (one boolean per participant). Target: the override, else the seeds' own
    (which must agree). Highlights of a failing seed tool are copied to the pattern.
    `ComputeType::Adjust`, path patterns, faces and occurrences fail `NotYet` (SOL-23/24).
  - Mirror: `BrepKernel::mirrored` (R-S04-6), named by `from_history` over its 1:1 history
    then `instanced_names(self, 1)`. Bodies: `NewBody` (new bodies in order) or `Join` (one
    `apply_body_op` of the mirrored compound with the sources as participants; a copy that
    doesn't touch becomes a new body with SOL-01's warning). Features: the seeds' tools
    mirrored, then their target (or the override). The plane is any plane input
    (`resolve_plane_input`).
  - Empty selections build and validate (the dialog's `min: 1` keeps OK off, as for fillet
    edges) and fail in compute.
- **Done-when evidence** (`crates/amg-features/tests/solid_pattern.rs` + `solid_pattern/occt.rs`,
  25 tests, 19 of them with OCCT; 7 unit tests in the module; goldens
  `tests/golden/solid/sol08_*.toml`, 7 files, rel 1e-9; every result BRepCheck-valid):
  1. `dw1_*`: 3×4 rectangular (20 / 15 mm) of a 10 mm cube: 12 bodies, 12 000 mm³, each copy
     centred at its `(i, j)` position; 6-instance full circular: 6 bodies, 6000 mm³, centres
     rotated by `k·60°`; symmetric rectangular (3 per side × 2) with 2 suppressed: 8 bodies,
     8000 mm³, suppressed slots absent, instance `0x8001` at −20 mm; symmetric circular (40°,
     4 per side) with 2 suppressed: 5 bodies, 5000 mm³ (relative 1e-12 everywhere).
  2. `dw2_*`: 10×10 pattern of a Ø3 "All" cut in a 100×100×5 plate: 46 465.708 mm³ =
     50 000 − 100·π·1.5²·5 within 1e-9, 106 faces (golden). With a counting kernel
     (`tests/solid_common/counting.rs`, every trait method forwarded): **1** `boolean` call
     for 99 instances; its tool is one compound of 99 solids at the instance positions whose
     binary form is 13 573 B against 130 018 B for 99 geometry copies (shared TShapes are
     written once). `benches/sol_pattern.rs` (`budget/sol-pattern-100`, full-mode pattern
     compute, release profile on the shared dev machine): median 103.1 ms (25 runs; criterion
     110–120 ms), budget < 300 ms, `AMG_STRICT_TIMING=1` passed; OCCT boolean 89.1 ms, unify
     8.7 ms, the rest (seed tool, 99 transforms, names) 5.3 ms.
  3. `dw3_*`: mirror of an L prism about YZ as a new body: volume equal (1e-12) and positive
     (faces still outward), BRepCheck valid, bbox mirrored, every face name carries
     `Inst { self, 1 }`; mirror-join of an L touching the plane: one body, 2× volume, 10 faces
     (coplanar faces merged); mirror of an extrude boss (feature) joined to its plate:
     54 000 mm³, valid. Also a mirrored "All" cut cuts the other side (1e-9) and its edge
     resolves `Exact`.
  4. `dw4_tnp17_*` / `dw4_tnp18_*` on script S17 (plate, r 4 "All" hole at (30, 0), circular
     feature pattern of 6 about Z, 0.5 mm chamfer on instance 3's top edge): count 6 → 8 keeps
     the chamfer `Exact` on a circle centred (−21.2132, 21.2132, 10), length 25.13274 (1e-4);
     count 6 → 3 gives `NotFound` and the chamfer in error.
  Also: suppressed feature instances (2 holes left out, 1e-9), a seed-depth edit flowing into
  the pattern, a join boss pattern (golden), non-patternable seeds (`can't be patterned`), bad
  counts (0, 2.5, 20 000) and empty selections as clean errors, and a proptest (16 random
  counts incl. 0 / fractions, spacings ±30 mm / ±30°, rectangular or circular, symmetric):
  a valid solid or a clean error.
- **Deviations.**
  - Feature patterns and mirrors with `target: None` read the seeds' participants in compute,
    but `inputs()` can't list them (it doesn't see the seeds), so the cache key covers them
    only through `Dependency::Feature(seed)`. A feature between the seed and the pattern that
    changes a participant doesn't invalidate the pattern (the same gap as SOL-02's "All"
    extent, which reads context bodies it doesn't list). Setting `target` explicitly avoids
    it; the dialog keeps an existing `target` but can't fill it (`build` doesn't see the
    design). Leftover.
  - The seed's tool is built against the state just before the pattern (S04 §5 "in the
    pattern's context"), not the seed's own input state; its resolution records and naming
    warnings stay in the sub-context (only highlights are copied).
  - Dialog: S11 §B has no field kind for timeline features, so seed features and suppressed
    instances are text fields of ids (`"12, 15"`); directions, axes and the mirror plane are an
    origin choice plus a "Selected …" pick. No manipulator anchors yet (`anchors` is empty).
    Faces, occurrences and path patterns have no fields (`read` returns `None` for a path
    pattern).
  - Mirror of features ignores `op` (the dialog stores `NewBody`); the seeds' target applies.
  - `PatternKind` allows `clippy::large_enum_variant` (the Interfaces shape).
- **Interface changes elsewhere.** None to shared crates or other lanes' code; no new
  dependency. Requests (not blocking): S11/UI-05 a timeline-feature pick (`EntityFilter` bit or
  a field kind) and viewport instance suppression to replace the two text fields; S01 a way for
  a feature to depend on bodies chosen by another feature (first deviation). Two new dialog
  registrations (`solid.pattern`, `solid.mirror`) add commands to the Solid "create" group, so
  UI shell snapshots that list commands may change (amg-ui was not run in this WP).
- **Leftovers.** SOL-23 (path, faces, adjust compute), SOL-24 (occurrences, new component);
  manipulator anchors for spacing and angle (UI-06); the cache-key gap above; timeline-feature
  and instance picks in the dialog (S11).

### 2026-11-26 - SOL-03 implemented (lane A)
- **Landed.** `amg_features::solid::revolve`: `solid.revolve` (`Revolve`, the Interfaces shape:
  `profiles`, `axis: RefInput`, `direction`, `side1`, `side2`, `flip`, `thin`, `target`;
  constructors `Revolve::angle` / `Revolve::full`), `register_feature!` v1 + `register_tool!`
  (so it can be a pattern/mirror seed), and its dialog registration (`revolve/dialog.rs`:
  profiles; axis = X/Y/Z or "Selected axis" with a pick of a sketch line, edge, construction
  axis or revolved face; direction; extent per side "Full turn" / "Fixed angle" / "To object"
  with object, offset and extend; side 2 fields; flip; operation; objects; group "create",
  order 20, pinned; angle manipulator anchors about origin axes). Extra public item:
  `revolve::SPLIT_GAP`.
- **Behaviour.** The axis resolves through CON-03 (`line_of`, else `axis_of`, i.e. SOL-08's
  `line_input`), reversed by `flip`; side 1 turns counter-clockwise about it, side 2 the other
  way, a negative angle reverses its side. `Distance` is an angle in (0°, 360°]; 360° and
  `Extent::Full` are a full turn without caps; "All" is refused (validate and compute). Two
  sides together may not exceed 360°; `Full` is one-sided only; symmetric takes half the angle
  each way (or the whole with `whole_length`). Each profile face × side is one
  `BrepKernel::revol`; the solids of a region group are fused and unified with SOL-02's
  `fuse_group` (two sides of one profile merge into one block, 6 faces for a rectangle), groups
  go into one compound (`compound_of`). **Validation** per profile face, before any sweep: an
  axis along the profile normal fails "perpendicular"; for an axis in the profile plane the
  boundary (analytic edges sampled 64×, others `tessellate_edge`) must stay on one side
  (touching is fine: the cone case); for an oblique axis its plane intersection must not be
  inside the face (even-odd test). A crossing fails `InvalidValue { field: "axis" }` with
  `SolidError::AxisCrossesProfile`'s text plus "profile i of n" and a hint, and highlights that
  profile's reference (and the axis reference). **To object:** a point, or a plane through the
  axis (construction plane, or a planar face with `extend`), gives the exact angle from the
  profile's radial direction (the first of the plane's two half-planes); every other target (a
  plane not through the axis, a bounded planar face, a curved face, a body; offsets) splits a
  `360° − SPLIT_GAP` (0.5°) sweep with SOL-02's `split_sweep` and keeps the piece holding the
  start cap (`TargetMissed` with the "Extend faces" hint otherwise), with SOL-02's
  `plane_face` / `extended_face` / offset handling. Naming as the naming note's Revolve row:
  laterals `Lateral#side` from the sketch tags, caps `StartCap#side` / `EndCap#side` (shared
  `name_sweep`), target faces on a split sweep `EndCap#side`; a full turn has no caps.
  `inputs()` lists profiles, sketches, the axis (`Dependency::Construction` for construction
  axes), to-object targets, and `Dependency::Body` for a body target (read whole), so the
  SOL-02/08 cache-key gap does not apply here.
- **Done-when evidence** (`crates/amg-features/tests/solid_revolve.rs` + `solid_revolve/occt.rs`,
  23 tests, 18 with OCCT; 2 unit tests in the module; goldens `tests/golden/solid/sol03_*.toml`,
  12 files, rel 1e-9; every result BRepCheck-valid):
  1. 12 golden cases, each within 1e-9 relative of Pappus (`A · r̄ · θ`): rectangle 10..30 ×
     0..20 on XZ about Z at 90° (12566.3706 mm³, 6 faces, bbox in the +X+Y quadrant) and 360°
     (50265.4825, 4 faces; `Distance(360°)` gives the same); the triangle (0,0)-(10,0)-(0,20)
     with an edge on the axis → cone π·100·20/3 = 2094.3951 (2 faces); two sides 30°/60° (90°
     total, bbox −60°..+30°, 6 faces); symmetric 90° (±45°, 6 faces; whole angle = 180°); each
     axis kind: an origin axis (Z; X at 180° for a 20 × 10 profile at r̄ 7), a sketch line
     x = 5 in the profile's sketch (r̄ 15, 9424.7780), a B-rep edge of a box on the Z axis, a
     construction axis (YZ ∩ XZ, directed −Z, so 90° goes to −Y); flip; a revolve-cut groove
     into a revolved cylinder (47909.2880, 7 faces); to a body at 270°; to the 75° plane.
  2. `dw2_a_profile_crossing_the_axis_fails_naming_the_profile`: of two regions, the one
     spanning x −10..10 fails "the profile crosses the axis: profile k of 2 …", its
     `EntityRef::Profile` is in the `Health::Error` entities and the other region's is not; an
     axis moved out of the profile fixes it. Also: an axis along the profile normal fails
     "perpendicular"; the X axis along a profile edge is accepted.
  3. `dw3_tnp22_editing_to_360_makes_the_cap_edge_not_found`: TNP-22 rebuilt on sketch +
     `solid.revolve` (rectangle x 10..30, z 0..20 on XZ, origin Z, 270°, 1 mm `solid.fillet` on
     the edge at (0, −30, 10)); the edit to 360° gives `ResolveError::NotFound { Edge }` and the
     fillet in error, as `expect` says.
  4. `dw4_revolve_to_a_plane_through_the_axis_at_75_equals_the_75_angle`: to a construction
     plane `AtAngle { Z axis, zero XZ, 75° }` vs `Distance(75°)`: volume, area and bbox equal
     within 1e-9 (volume 10471.9755 = Pappus), same face count.
  Also: an angle edit 90° → 120° → 45° keeps a fillet on the end-cap edge `Exact`; two sides:
  the side-2 cap edge (`EndCap#1`) stays `Exact` when side 1 changes; to a body, its face and
  its vertex all stop at 270° (30159.2895 mm³, 1e-9); a 1 mm high face fails "does not cover
  the profile … Turn on Extend faces" with the face highlighted; zero, > 360° and overlapping
  two-sided angles, a negative angle (reverses), thin (`NotYet`); dialog tests (conditional
  fields hidden for the defaults, unique labels, option labels distinct from field labels, a
  preselected sketch line becomes the axis, `read(build(v)) == v` for every axis, extent and
  direction, field errors for a two-sided full turn and a missing axis pick); data tests
  (inputs incl. `Dependency::Body`, expression dimensions, JSON round trip, static checks).
- **Deviations.**
  - `SolidError::AxisCrossesProfile` stays a unit variant; the profile is named in the
    `InvalidValue` message ("profile i of n") and highlighted, not carried in the error value.
  - To-object angles are measured from the profile's radial direction: exact for profiles in
    the axis plane (the usual case); for profiles off the axis plane the mean radial direction
    of the boundary is used. A target within `SPLIT_GAP` (0.5°) before the profile's own
    half-plane is missed; a point target takes no offset (`Invalid`), and a plane through the
    axis with an offset uses the split path.
  - Two sides are two revolves fused and unified (as SOL-02 does for extrudes), not one sweep
    from −θ2; the names are SOL-02's (`EndCap#1` for side 2's cap, laterals merged under side 0).
  - Lane-A-internal reuse, no behaviour change: `extrude::sides` is `pub(crate)` with
    `name_sweep` and `split_sweep` factored out of `prism_named` / `split_keep`, and
    `plane_face`, `extended_face` and a few helpers `pub(crate)`; `extrude::fuse_group`, the
    `extrude::dialog` helpers (`OPS`, `choice`, `expr`, `is`, `not`, `target_filter`,
    `one_ref`, `take_expr`, `take_choice`), `pattern::line_input` and `pattern::dialog::{AXES,
    read_axis, axis_values}` are `pub(crate)`. The SOL-01/02/05/08 tests pass unchanged.
  - The angle manipulator anchors exist only for origin axes (others resolve only inside a
    recompute).
- **Interface changes elsewhere.** None: only public `amg-brep` (`revol`, `split`,
  `tessellate_edge`), `amg-doc` and `amg-features::construct` APIs are used; no new
  dependency. The new pinned `solid.revolve` registration adds a "Revolve" button to the Solid
  "create" group, so the UI shell snapshots (`crates/amg-ui/tests/snapshots/shell_*.png`) will
  likely change (amg-ui was not run in this WP; regenerate at merge).
- **Leftovers.** Thin revolve (SOL-18); manipulator anchors for non-origin axes and the
  side-2 / to-object fields (UI-06); a Fusion check of revolve offset semantics on to-object
  targets (SOL-29).

### 2026-11-26 - SOL-07 implemented (lane A)
- **Landed.** `amg_features::solid::combine` (`solid.combine`, `Combine`) and
  `amg_features::solid::split` (`solid.split_body` / `SplitBody`, `solid.split_face` /
  `SplitFace` + `SplitFaceTool::{Surface, CurvesAlong}`), all `register_feature!` v1, with dialog
  registrations (`combine/dialog.rs`, `split/dialog.rs`; group "modify", orders 40/42/43, not
  pinned, no manipulator anchors). Constructors `Combine::new`, `SplitBody::new`,
  `SplitFace::{by_surface, by_curves}`.
- **Behaviour.**
  - Combine: target and tools are followed through `Evaluated::successors` (SOL-01's
    participant rules, now `common::live_body`) and must be solids of the context component; a
    tool equal to the target is refused. One N-ary `boolean` (target = operand 0, tools 1..n),
    simplified with `simplify` (unify + compose), named by history only (target and tool faces
    keep their names; naming note "Combine" row). **Join** keeps the whole result in the
    target body, also when a tool doesn't touch it (a body of several lumps, as Fusion does,
    with a warning): TNP-23 expects B's edge in A's body. **Cut / intersect** use SOL-01's
    piece rule (piece 0 keeps the target id, piece k is `BodyId { self, k }` with
    `CreatedFrom { derived_from: target }`). A cut with no tool box meeting the target, or whose
    history leaves the target untouched, is `NothingToCut`; an empty common part
    `NothingToIntersect`; a cut that removes everything is `Invalid { tools }`. Tools are
    `Consumed { into: target }` unless `keep_tools`.
  - Split tool (`split::split_tool`, shared by both splits): origin and construction planes and
    (with `extend_tool`) planar faces become SOL-02's `plane_face` square over the body box; a
    planar or curved face is used as is, or extended with SOL-02's `extended_face` (cylinder and
    cone over the box, sphere / torus whole, B-spline `ExtendUnsupported`); a body is used whole.
    Points, axes and curves are `Invalid { tool }`.
  - Split body: `BrepKernel::split`; fewer than two solids is `NoSplit` ("the tool does not split
    the body. Extend the tool so it crosses the body.", tool highlighted). Piece order (note
    §5.2): signed centroid distance along a planar tool's normal (a face's outward normal, an
    origin or construction plane's normal), else volume descending then centroid x, y, z, all
    quantised to 1e-6 so round-off can't swap equal pieces. The whole compound is named once with
    `SplitOrder::PieceSlot` (each output face's slot from the per-solid restrictions); faces left
    by a plane or face tool get `Gen { self, SplitFace#0, Sel(key) | Construction(id) }` (no
    source for an origin plane); a body tool's faces keep the tool body's names. Each piece is
    then named from the compound through its per-solid identity history. Piece 0 keeps the
    `BodyId`, piece k is `BodyId { self, k }` with `CreatedFrom`. The tool body is kept. Split
    results are not unified (not a BodyOp).
  - Split face: picks resolve with `DropMissing`, grouped per body; the tool is built once over
    the union of the bodies' boxes. `Surface(r)` as above. `CurvesAlong { curves, dir }`: each
    sketch line or arc (resolved through CON-03 into world space, so SK-09 projected curves work
    too) becomes a wire (`make_wire`) moved back along `dir` and swept with `prism` (vector) into
    a sheet that crosses the work box both ways; several curves go into one compound. `dir` is
    any line input (`pattern::line_input`), `None` = the first curve's sketch normal. One
    `split_faces` per body; the kernel's `InvalidInput { faces }` ("does not meet or split") is
    `NoSplit` with the hint and the tool (or the curves) highlighted. Naming by history (split
    faces `name + Split { self, i, n }`, geometric order), picked faces registered as selections.
  - `inputs()` list every body the features read: combine target and tools
    (`Dependency::Body`), the split body and a body tool (`Dependency::Body`, read whole),
    construction tools (`Dependency::Construction`), face picks and sketch curves
    (`Dependency::Entity`, plus `Dependency::Sketch` per curve sketch). The SOL-02/08 cache-key
    gap doesn't apply.
- **Done-when evidence** (`crates/amg-features/tests/solid_combine.rs` + `solid_combine/occt.rs`,
  21 tests, 16 of them with OCCT; 13 unit tests in the modules; goldens
  `tests/golden/solid/sol07_*.toml`, 16 files, rel 1e-9; every result BRepCheck-valid):
  1. `dw1_combine_goldens_for_each_op_with_and_without_keep_tools`: A 0..20 and B 10..30 (each
     2000 mm³, 10 × 10 section): join 3000 mm³, cut 1000 (x 0..10), intersect 1000 (x 10..20),
     6 faces each after unify, 6 goldens (`join`, `cut`, `intersect`, each also `_keep_tools`);
     without keep tools B is gone and `successors(B) == [A]` (`Consumed { into }`), with keep
     tools B stays 2000 mm³ with no successor. Also: a cut through a bar gives 1000 + 1000 with
     `derived_from(BodyId { comb, 1 }) == bar` (goldens `cut_split_piece0/1`); two tools in one
     combine cut two pockets (3400 mm³, 14 faces, golden `cut_two_tools`), both consumed;
     errors "nothing to cut", "nothing to intersect", target as its own tool and no target, each
     leaving the bodies unchanged. `dw1_tnp23_*`: TNP-23 rebuilt on two extrudes (A 0..50 × 0..60
     × 40, B 60..100 × 0..60 × 20) and a 2 mm `solid.fillet` on B's edge near (80, 0, 20); the
     combine inserted before the fillet folds B into A (warning "separate lump", B gone); the
     fillet stays `Ok`, its ref resolves `Exact` in body A, edge centroid (80, 0, 20), length 40,
     and A holds 2 solids.
  2. `dw2_splitting_a_box_by_a_plane_orders_pieces_along_the_normal`: −50..50 × 0..60 × 40 by
     origin YZ: piece 0 (the original id) is x ≤ 0, `BodyId { split, 1 }` is x ≥ 0, 120 000 mm³
     each, `derived_from` the original (goldens `split_yz_piece0/1`); by origin XZ (normal −Y)
     piece 0 is the +Y part. `dw2_tnp21_*`: TNP-21 on a polygon sketch + extrude, split by YZ, a
     2 mm fillet on piece 1's edge near (25, 0, 40); the block grows to x = 80: the fillet
     resolves `Exact` in `BodyId { split, 1 }`, centroid (40, 0, 40), length 80, piece volume
     within 1e-9. Also split by a body: a bar 0..40 by a box 10..20 gives 3 pieces ordered
     2000 (x̄ 30), 1000 (x̄ 5), 1000 (x̄ 15), the tool kept (golden `split_by_body_piece0`).
  3. `dw3_split_face_by_a_projected_sketch_line_and_by_a_plane_match`: the top of 0..20 × 0..10
     × 10 split by a sketch line x = 5 on XY projected along the sketch normal, and another box's
     top by a construction plane (YZ offset 5): 7 faces, still one solid of 2000 mm³, top faces
     50 and 150 mm² within 1e-9 and equal between the two (goldens `split_face_by_line`,
     `split_face_by_plane`). Also: a line along an explicit Z axis splits top and bottom (8
     faces); a sketch circle splits off a half disc of 12.5π mm² (1e-9); a fillet on the small
     piece's edge stays `Exact` through a height edit 10 → 20 (edge at z 20, length 10).
  4. `dw4_*`: a 10 × 20 face inside the bar's x range fails "the tool does not split the body.
     Extend the tool so it crosses the body." with the face in the `Health::Error` entities;
     `extend_tool` splits the bar at z = 5 into 2000 + 2000 (piece 0 below, golden
     `split_extended_face_piece1`); a construction plane above the body and a sketch line off
     the face (split face: curve highlighted, body unchanged) fail the same way; a short
     cylinder face fails and, extended, cuts the r 5 core (250π mm³ within 1e-6, golden
     `split_extended_cylinder_core`).
  Data and dialog tests: inputs per feature, the TNP-21/23 corpus JSON deserializes, unpicked
  selections validate and fail in compute, `read(build(v)) == v` for every tool kind and op,
  preselected bodies fill target / tools / body, unique labels, conditional fields hidden for
  the defaults (except the split-face surface-tool chooser, whose tool type is the default).
- **Deviations.**
  - `Combine::target` and `SplitBody::body` are `Option<BodyId>` (Interfaces updated): the
    dialog must build with nothing picked (the all-registrations round-trip test); `None` fails
    in compute. The corpus JSON (`"target": {…}`) deserializes unchanged.
  - Join keeps disjoint tools as lumps of the target body (TNP-23) instead of SOL-01's
    per-solid split; cut and intersect still split into bodies.
  - `SolidError::NoSplit` maps to `InvalidValue { field: "tool" }` with its hint (it was a
    `Kernel` error without a field; nothing used it before).
  - A body tool's cut faces keep the tool body's names (distinct per tool face) instead of one
    `SplitFace#0`; plane and face tools get `SplitFace#0` as the note says.
  - Split face takes sketch lines and arcs; free-form sketch curves (splines, ellipses) fail
    `NotYet { what: "split face along free-form sketch curves" }`.
  - **Lineage-optional inputs** (needed for TNP-23): the engine refused to compute a feature
    whose referenced body was consumed upstream ("an input is not available: … did not produce
    it") before DOC-06 resolution could follow the lineage. `solid.fillet` and `solid.chamfer`
    (SOL-05 module, new `fillet::lineage_optional`) and the three SOL-07 kinds return
    `input_is_optional = true` for `Dependency::Body` and `Dependency::Entity(Topo)`; the engine
    then hashes the whole input state for that dependency (`hash_missing_input`). The SOL-05
    tests pass unchanged. Extrude, revolve and pattern inputs are not changed.
  - Lane-A-internal reuse, no behaviour change: `common::{follow, live_body, Participant,
    boxes_meet, touches, contributes, solid_geom, pieces, identity_result}` are `pub(crate)`
    (`participants` now calls `live_body`).
- **Interface changes elsewhere.** None to shared crates or other lanes' code; only public
  `amg-brep` (`boolean`, `split`, `split_faces`, `make_wire`, `prism`, `make_compound`,
  `mass_props`, `sub_geom_all`), `amg-doc` (`NamingBuilder::split_order` / `generated`,
  `CreatedFrom`, `Consumed`, `input_is_optional`) and `amg-features::construct` APIs. No new
  dependency. Three new unpinned "modify" commands (Combine, Split Body, Split Face): UI shell
  snapshots that list unpinned commands may change (amg-ui was not run in this WP; regenerate
  at merge).
- **Leftovers.** Free-form sketch curves for split face (lift `Curve2d` B-splines to
  `Curve3::BSpline` on the sketch plane); lineage-optional inputs for the other S04 kinds
  (extrude/revolve targets and participants), ideally as an engine default for B-rep
  references (S01); a Fusion check (SOL-29) that a combine cut which severs the target makes
  separate bodies; "New component" for combine (SOL-24).

### 2026-11-26 - SOL-06 implemented (lane A)
- **Landed.** `amg_features::solid::shell` (`solid.shell` / `Shell`, `ShellDirection`; module
  `shell::thicken` with `solid.thicken` / `Thicken`), `amg_features::solid::draft` (`solid.draft`
  / `Draft`, `DraftSides`) and `amg_features::solid::transform` (`solid.scale` / `Scale`,
  `ScaleKind`), all `register_feature!` v1 with the Interfaces shapes, and dialog registrations
  (`shell/dialog.rs`, `draft/dialog.rs`, `transform/dialog.rs`; group "modify", orders 24
  shell, 25 draft, 26 scale, 27 thicken, not pinned, no manipulator anchors). `solid.thicken`
  also registers a `ToolRegistration` (it is an `OpTarget` tool, so pattern/mirror can use it).
  `solid::common::tangent_chain_faces` (Interfaces: transitive, normals compared at the shared
  edge's two ends and middle) and the extra `tangent_blend_faces` (see Deviations). Extra
  public items: `Shell::plan` / `Draft::plan` with `BodyFaces` / `PlannedFace` (the faces passed
  to the kernel per body, picks first with their keys, for the UI-05 preview highlight),
  `shell::FACE_CHAIN_ANGLE` (1e-3 rad), `Scale::transform`, constructors `Shell::inside`,
  `Draft::one_side`, `Scale::{uniform, non_uniform}`, `Thicken::new_body`.
- **Behaviour.**
  - Shell: the bodies of the removed faces (`DropMissing`, grouped per body) plus `bodies`
    (hollowed, R-S04-8). Joins are sharp (`OffsetJoin::Intersection`). `Inside` / `Outside` are
    one `BrepKernel::shell` (−inside / +outside). `Both` moves every kept face out by `outside`
    with `offset_faces` (removed faces stay, neighbours re-intersected) and shells that body
    inward by `inside + outside`, so the opening stays on the original face and no coincident
    boolean is needed. Naming per the note: inner walls `ShellInner#0` from the offset face's
    name (`Topo*`), the rim `ShellRim#0` from the removed face (`Sel(key)` for picks), arc-join
    faces `ShellJoin#0` from their edge; the body's faces keep their names.
  - Draft: the neutral plane is any plane input (`resolve_plane_input`); pull = its normal,
    `flip` reverses it; a positive angle narrows the part along the pull (S02's convention).
    One side: one `draft` call with the picks. Two sides / symmetric: `split_faces` with
    SOL-02's `plane_face` over the body box (faces the plane misses stay whole), then the pieces
    on the pull side drafted by `angle` with the pull and the others by `angle2` (or `angle`)
    against it, so both halves narrow away from the parting plane; histories composed and the
    split order re-applied; offending faces of the later steps mapped back to the input
    (`error::remap_offending`). Names by history only (split faces `name + Split`).
  - Scale: `Shape::gtransformed` of `T(p) · S · T(−p)` about a point input (`point_of`), factors
    finite and > 0 (a negative factor is a mirror); bodies modified in place, names by the 1:1
    history.
  - Thicken: per body, the whole sheet when every face of a sheet body is picked, one face as
    itself (`sub_shape`), else a compound of the picked faces; their names are mapped from the
    body by TShape identity (`is_same`). `Side1` / `Side2` = `thicken(±t)`, `Center` = `offset`
    by −t/2 then `thicken(t)` (histories composed, the offset role dropped). Far faces
    `Offset#0`, side walls `ThickenSide#0` (both `Topo*`); then `apply_body_op` with the target.
  - `inputs()` list every body the features read (`Dependency::Body` once per body of a pick and
    per listed body, `Dependency::Entity` per face pick, the plane / point dependency, the
    thicken participants), so the SOL-02/08 cache-key gap doesn't apply. Picks are
    lineage-optional as SOL-07's (`fillet::lineage_optional`); thicken participants stay strict.
- **Done-when evidence** (`crates/amg-features/tests/solid_shell.rs` + `solid_shell/occt.rs`,
  18 tests, 13 of them with OCCT; 4 unit tests in the modules; goldens
  `tests/golden/solid/sol06_*.toml`, 14 files, rel 1e-9; every result BRepCheck-valid):
  1. `dw1_shelled_and_hollow_boxes_match_analytic_volumes`: 40 × 30 × 20 box, top removed,
     1 mm: inside 3784 (`abc − (a−2)(b−2)(c−1)`), outside 4224, both 8008 mm³; hollow (no face
     removed, `bodies`) inside 4848, outside 5568, both 10 416 mm³, 12 faces each; all within
     1e-9 relative (goldens `box_top_*`, `hollow_*`). Also both with 1 mm in / 2 mm out:
     `(a+4)(b+4)(c+2) − (a−2)(b−2)(c−1)`, opening still at z = 20.
  2. `dw2_*`: the box with its 4 top edges filleted (r 3, 10 faces): picking the top plans 5
     faces (top with key 1, then the 4 fillets); the shell has 14 faces (5 outer, 5 inner, 4 rim
     pieces), the only cylinders are the 4 rims, all named `ShellRim`, the top is open (golden
     `filleted_box_chain`). With `tangent_chain = false` the feature fails "a removed face is
     tangent to faces that stay. Turn on Tangent chain …" with exactly the 4 fillets
     highlighted.
  3. `dw3_*`: a plate with a boss and a concave 1 mm fillet at the boss base, shelled outward
     2 mm (bottom removed): `Health::Error` from the kernel's collapse check; the highlighted
     entities are faces that resolve to the concave fillet cylinder. 0.5 mm works.
  4. `dw4_*`: 4 sides of the box drafted 3° about XY: every side `asin(n·z)` = 3° within 1e-9
     rad, volume `abh − (a+b) t h² + 4t²h³/3` within 1e-9; symmetric about a construction plane
     at z = 10: 10 faces, upper pieces +3°, lower −3° (1e-9 rad), volume twice the half-height
     formula (1e-9); two sides 3° / 5° the sum of the two halves (1e-9); a flipped draft about
     z = 20 narrows downwards (goldens `draft_*`).
  5. `dw5_*`: uniform 2 → 8 × volume within 1e-12, bbox (80, 60, 40); sphere r 10 scaled
     (1, 2, 3) → `4/3 π·10·20·30` within 7.97e-11 relative, measured with
     `volume_adaptive(1e-14)` (the default `mass_props` quadrature gives 1.1e-9 on this
     rational B-spline; the sphere itself measures 6.5e-16). Also scale 0.5 about a picked
     vertex keeps it fixed; factors 0 and −1 fail.
  6. `dw6_*`: a bicubic Bézier sheet (40 × 40 mm, middle poles raised 2 mm, area
     1603.594 mm²) thickened 1 mm: valid single solids; side 1 +0.373 %, side 2 −0.373 %,
     centre 1.2e-7 of area × t (goldens `thicken_bspline_*`); the two sides' mean and the
     centred slab within 1e-5 (Steiner); 4 `ThickenSide` walls and one `Offset` face; the sheet
     is kept. Also a box top thickened 2 mm and joined: one 6-face body of `ab(c+2)`.
  7. `dw7_tnp_x3_*`: shell (top removed, 1 mm), 0.5 mm fillet on the inner floor edge
     (1, 15, 1); extrude height 20 → 30: `Exact`, centroid (1, 15, 1), length 28; 5
     `ShellInner` and 1 `ShellRim` names. `dw7_tnp_x6_*`: 4-side draft 3°, 2 mm fillet on the
     (0, 0) vertical edge; angle 3° → 5°: `Exact`, edge centroid on the 5° corner line.
  8. `benches/sol_shell.rs` (`budget/sol-shell-preview`): **not met**. Body: a 98-vertex star
     prism (radii 50 / 46, height 20; 100 faces, as SOL-05's bench), top removed, 0.3 mm
     inward; release profile on the shared dev machine (another worker building, 3.5–10 GB free
     RAM): preview median 369 ms over 25 runs (runs of 395–454 ms earlier; criterion
     [354, 372, 390] ms), full mode 390 ms. `AMG_STRICT_TIMING=1` asserts < 100 ms and fails.
     The bare `BrepKernel::shell` call alone takes 363 ms, so the kernel is the whole cost
     (naming, plan and resolution are within the run-to-run noise). Probes
     (`AMG_SOL06_PROBE=1`): the same body shelled outward 89 ms, hollowed inward 312 ms, arc
     join 465 ms, `offset` −0.3 mm 293 ms; a 120 × 120 × 10 plate with 94 holes (100 faces)
     933 ms. OCCT's `BRepOffset_MakeOffset` inward (gap and intersection handling,
     `BRepOffset_Inter3d`) dominates; meeting the budget needs kernel work (S02/S20: a faster
     offset path or caching for preview), not feature code.
  Data and dialog tests: the four kinds round-trip `read(build(defaults))` with and without a
  preselection (faces → shell / draft / thicken picks, bodies → shell / scale bodies), unique
  labels, inputs, JSON round trips, edge picks refused by `validate`; `tangent_chain_faces` is
  transitive (top + 4 fillets + 4 sides = 9 on the filleted box) and `tangent_blend_faces`
  stops at planes (5).
- **Deviations.**
  - Shell tangent chain grows through **curved faces only** (`tangent_blend_faces`): the fully
    transitive chain of the Interfaces helper also reaches the side walls the top fillets are
    tangent to and would leave only the bottom. `tangent_chain_faces` itself is as specified.
  - OCCT closes an opening on the removed faces' own surfaces: removing a fillet ring leaves 4
    rim pieces on the fillet cylinders (not a flat rim), so Done-when 2 counts 14 faces.
  - OCCT's `MakeThickSolid` returns the body **unchanged** (no error) when a removed planar face
    is tangent to fillets that stay. The feature detects a result without `ShellInner` faces
    and fails with the tangent neighbours highlighted (or a kernel error if there are none).
    Request for S02 (BREP-07, not blocking): make `shell` fail with `LocalOpFailed` naming those
    faces.
  - Draft `tangent_chain` only changes `plan` (OCCT always carries a draft on to tangent
    faces), as SOL-05's fillet flag; only the picks go to the kernel.
  - Done-when 5's ellipsoid volume is measured with adaptive integration (above).
  - Done-when 6 uses a flatter, larger sheet than BREP-07's 20 × 20 / 3 mm bump: on that one a
    one-sided slab differs from area × t by 2.2 % (the mean-curvature term), beyond 0.5 %.
  - The draft dialog edits `flip` with `FieldKind::FlipDirection`; the shell dialog's faces and
    bodies both have `min: 0` (either may be empty); no manipulator anchors yet.
- **Interface changes elsewhere.** None to shared crates or other lanes' code; only public
  `amg-brep` (`shell`, `offset`, `offset_faces`, `thicken`, `draft`, `split_faces`,
  `Shape::gtransformed`, `sub_shape`, `is_same`, `make_compound`, `eval_surface`,
  `project_point_on_face`, `edge_ends`), `amg-doc` and `amg-features::construct` APIs. No new
  dependency. Four new unpinned "modify" commands (Shell, Draft, Scale, Thicken): UI shell
  snapshots that list unpinned commands may change (amg-ui was not run in this WP; regenerate
  at merge).
- **Leftovers.** Done-when 8 (`budget/sol-shell-preview`, kernel-bound, above); the S02 request
  above (shell returning its input); shell per-face thickness
  (`ShellOpts::per_face`) has no field yet; draft by a parting line / split line; manipulator
  anchors (UI-06); thicken of a partial multi-face selection of a solid is a compound of loose
  faces (sew first if OCCT needs a shell); "New component" for thicken (SOL-24).

### 2026-11-26 - Owner: budget misses accepted (SOL-02, SOL-06)
- SOL-02 Done-when 6 (`budget/sol-extrude-cut` 121 ms vs 80 ms) and SOL-06 Done-when 8 (`budget/sol-shell-preview` 369 ms vs 100 ms) are kernel-bound (OCCT boolean and offset). The owner accepted both misses: the WPs are Done, and the budgets are tracked by S20 KH-20.
- The context-body cache-key gap (SOL-02 "All", SOL-08 target-less feature patterns) is fixed generically by S01 DOC-15.

### 2026-11-26 - SOL-09 implemented (lane A)
- **Landed.** `amg_features::solid::moves`: `solid.move` (`Move`, `MoveSpec::{Free, Translate,
  Rotate, PointToPoint}`) and `solid.align` (`Align`); `amg_features::solid::face_ops`:
  `solid.offset_face` (`OffsetFace`, R-S04-5), `solid.replace_face` (`ReplaceFace`) and
  `solid.delete_face` (`DeleteFace`), all `register_feature!` v1, with dialog registrations
  (`moves/dialog.rs`, `face_ops/dialog.rs`; group "modify": Move/Copy order 50 pinned with
  shortcut M, Align 51, Offset Face 53, Replace Face 54, Delete Face 55) and the press-pull
  `DispatchRegistration` `solid.press_pull` (order 10, pinned, shortcut Q; public
  `face_ops::press_pull`). Extra public items: `Move::{translate, rotate, copied, transform}`,
  `Align::{new, transform}`, `moves::IDENTITY_MATRIX`, `OffsetFace::new`, `ReplaceFace::new`,
  `DeleteFace::new`. The modules are named `moves` and `face_ops` (not `transform`) so they
  can't collide with SOL-06's scale module.
- **Behaviour.**
  - Move: bodies are followed through successors and deduplicated (`common::live_body`). The
    motion is `Shape::transformed` (shared TShapes, every index kept), so the body is
    `Modified` with its element map unchanged and every reference into it resolves `Exact`.
    `Free` is a row-major rigid matrix (translation in column 3; `validate` checks that it is
    finite and affine, compute refuses a scale, shear or reflection as "not rigid").
    `Translate` takes three lengths. `Rotate` takes an axis through `pattern::line_input`
    (edges, sketch lines, axes, axes of revolved faces and circles) and an angle.
    `PointToPoint` takes two points (vertex, sketch or construction point, or a circle, sphere
    or torus centre). `copy` makes `BodyId { self, i }` (`i` = position among the live selected
    bodies) as plain `Created` (not lineage), named `instanced(self, i)`; the sources stay.
  - Align: `from` and `to` resolve through CON-03 into a point, a directed line (straight
    edges, axes, circles and cylinder faces by their axis) or an outward plane (planar faces,
    origin and construction planes; a face's pivot is its bounding-box centre on the plane).
    Plane → plane turns `from` about its pivot by the shortest arc so the normals oppose
    (`flip`: agree), then shifts along the target normal only (no in-plane slide). Line → line
    turns the directions together (`flip`: reversed) and shifts onto the target line. A point
    goes to the nearest point of a line or plane, and a plane or line is moved through a target
    point. Plane ↔ line is refused. `angle` then turns about the target's normal or direction
    through the moved pivot (refused when neither element has a direction). Bodies as for Move.
  - Offset face: picks resolve with `DropMissing`, grouped per body; one `offset_faces` call per
    body with the same distance for every face (outward positive). A zero distance changes
    nothing and warns. Names come through the `Modified` history.
  - Replace face: `with` is a face (its untrimmed surface, `Shape::surface`) or any plane input
    (`resolve_plane_input`). Faces are replaced one `replace_face` call at a time, each later
    face found through the composed history. `tangent_chain` first grows the picks with the
    new crate-private `face_ops::tangent_faces` (neighbours across two-face edges whose outward
    normals agree within 1e-3 rad at both edge ends and the middle). The replaced face keeps its
    name (BREP-16 history).
  - Delete face: `heal` → `defeature` per body (neighbours keep their names, deleted faces get
    tombstones). Without heal (`face_ops::remove_faces`) the kept faces go into a compound
    (`make_compound`, shared TShapes, sub-shapes matched with `is_same`), which is sewn (`sew`,
    `make_solid: false`) into an open shell. The history (removed faces, and edges and vertices
    only they used, `Deleted`; the rest `Kept`) is composed with the sewing's. The body stays a
    `BodyGeom::Solid` holding a shell (a surface body).
  - Press-pull: a selection of only faces → `solid.offset_face`, only edges → `solid.fillet`,
    only profiles → `solid.extrude`; an empty, mixed or other selection disables it (`None`).
  - `inputs()` lists every body read (`Dependency::Body` per moved or aligned body, per body of
    a face pick, and the body of a `with` face) plus `Dependency::Entity` / `Construction` for
    every reference. Body and B-rep entity inputs are lineage-optional (the SOL-07 rule).
- **Done-when evidence** (`crates/amg-features/tests/solid_moves.rs` + `solid_moves/occt.rs`,
  16 tests, 12 of them with OCCT; 8 unit tests in the modules; goldens
  `tests/golden/solid/sol09_*.toml`, 12 files, rel 1e-9 (delete-face cases 1e-6); every solid
  result BRepCheck-valid):
  1. `dw1_a_moved_body_keeps_names_and_a_fillet_ref_resolves_exact`: a translate (30, 5, 0)
     inserted between a 20 × 10 × 10 box and a 2 mm fillet. The fillet ref resolves `Exact` in
     the same body, edge centroid (40, 5, 10) within 1e-9, volume unchanged (1e-9). Editing the
     move into a 90° rotation about Z keeps it `Exact` (centroid (0, 10, 10)); golden
     `move_rotate_filleted`. `dw1_a_copy_is_a_new_body_named_instanced`: the copy is
     `BodyId { move, 0 }`; its naming equals the source's `instanced_names(move, 0)` (every face
     name ends in `Inst { move, 0 }`); the source keeps its names; `successors(source)` is empty.
     A fillet picked on the copy resolves `Exact` in the copy (also after the copy distance goes
     30 → 50 mm), and the source stays 2000 mm³ (golden `copy_translate`). Also: a free matrix
     (90° about X + 5 mm) places the box exactly (golden `move_free`), a scale matrix fails "not
     rigid", and point to point moves vertex (10, 10, 10) onto (20, 0, 0).
  2. `dw2_align_face_to_face_makes_the_faces_coplanar`: a cube's +X face onto another box's top
     (z = 5), plain, flipped and with 30°. Every vertex of the aligned face is at z = 5 within
     1e-9 mm, its normal is −Z (flip: +Z) within 1e-12, and the cube sits on (flip: under) the
     face; golden `align_face_to_face`. Also: a top face onto the XY plane, a vertical edge onto
     the Z axis, and a plane onto an axis refused.
  3. `dw3_offset_of_a_box_top_and_a_cylinder_face`: box top +2 mm gives z max 12 within 1e-9,
     2400 mm³ and 6 faces (golden `offset_box_top`); a downstream fillet on the top front edge
     stays `Exact` (now at z 12); −3 mm gives 7 mm and 1400 mm³. Cylinder r 5 × 10, lateral
     face +1 mm: the cylinder surface's radius is 6 within 1e-9 and the volume 360π within 1e-9
     (golden `offset_cylinder`).
  4. `dw4_replace_face_onto_an_offset_plane_matches_the_analytic_volume`: the top of a
     20 × 10 × 10 box onto a construction plane XY + 15 → 3000 mm³ and XY + 4 → 800 mm³ (1e-9,
     6 faces, goldens `replace_up` / `replace_down`). A downstream offset of the top face, made
     before the replace was inserted, still resolves `Exact`. The surface of another body's face
     (z = 12) gives 2400 mm³. With tangent chain, a split top (two coplanar halves) is replaced
     whole: 3000 mm³ (golden `replace_tangent_chain`).
  5. `dw5_deleting_a_fillet_face_with_heal_restores_the_box`: 2 mm fillet (1982.83 mm³), blend
     deleted with heal → 2000 mm³ within 1e-6, 6 faces (golden `delete_fillet_heal`).
     `dw5_deleting_without_heal_leaves_an_open_body_with_free_edges`: the same blend removed →
     6 faces, no solid, exactly 4 free edges (two tangent lines, two quarter arcs), area =
     before − 20π within 1e-6 (golden `delete_fillet_remove`). A box without its top → 5 faces,
     4 free edges, 800 mm² (golden `delete_top_remove`); a later removal of a side face still
     resolves `Exact` (6 free edges). `a_reference_to_a_deleted_face_is_not_found_with_the_tombstone`:
     a ref to the healed-away blend fails `NotFound { removed_by: Some(delete) }`.
  6. `dw6_press_pull_dispatches_face_edge_and_profile` (every build) and
     `dw6_press_pull_on_real_picks_opens_the_right_dialogs` (OCCT: the dispatched dialogs build
     from the real picks and compute `Ok`).
  Dialog tests: the five registrations round-trip `read(build(defaults))` for empty, body and
  face preselections; labels are unique and option labels differ from field labels; every move
  type and align target round-trips; a free move keeps its matrix on edit.
- **Deviations.**
  - `Align::from` is `Option<RefInput>` (Interfaces updated), as SOL-07 did for
    `Combine::target`: the dialog must build with nothing picked; `None` fails in compute.
  - `MoveSpec::Free` is documented as row-major (translation in column 3; Interfaces updated).
  - The kernel-calls table lists `heal` for SOL-09, but it is unused: delete without heal needs
    no healing (compound + `sew`), and delete with heal is `defeature`.
  - Replace or offset of a face tangent to a fillet blend fails in the kernel ("offset_faces
    failed on 0 sub-shape(s)": BRepOffset with intersection join can't move a face whose tangent
    neighbour is a blend, nor offset the blend itself outwards). The tangent-chain test
    therefore uses two coplanar halves. This is a BREP-16 / S20 robustness item.
  - `tangent_faces` is crate-private in `face_ops`. SOL-06 may add the Interfaces'
    `common::tangent_chain_faces`; switch to it then. `moves::rotation_about` and
    `matrix_transform` are crate-private too. SOL-06's scale code was not on this branch, so
    nothing is shared with it.
  - No manipulator anchors yet (`anchors` is empty): the free-move triad and the distance and
    angle arrows are UI-06. The dialog can't edit a free matrix; it keeps the stored one.
- **Interface changes elsewhere.** None to shared crates or other lanes' code. Only public
  `amg-brep` (`Shape::transformed`, `offset_faces`, `replace_face`, `defeature`,
  `make_compound`, `sew`, `edge_ends`, `tessellate_edge`, `project_point_on_face`,
  `eval_surface`), `amg-doc` and `amg-features::construct` APIs are used. No new dependency.
  Five new dialog registrations and a dispatch add commands to the Solid "modify" group, two of
  them pinned (Move/Copy, Press Pull), so the UI shell snapshots
  (`crates/amg-ui/tests/snapshots/shell_*.png`) will likely change (amg-ui was not run in this
  WP; regenerate at merge).
- **Leftovers.** Kernel robustness of offset and replace next to blends (S02/S20); manipulator
  anchors and the free-move triad (UI-06); per-face distances for offset face (the kernel takes
  them, the dialog has one distance); moving faces (Fusion's Move on faces) is not in the
  Interfaces; consolidating `tangent_faces` with SOL-06's `tangent_chain_faces`.

### 2026-11-28 - DOC-15 closes the context-body cache-key gap (lane E)
- `Extrude` with an "All" extent, and `Pattern` / `Mirror` over features (with or without
  `target`), declare `Feature::context_reads() = Some(ContextBodies::Context)` (S01 DOC-15), so
  an upstream edit of a body in their component recomputes them. This resolves the first
  deviation of the SOL-08 entry (2026-11-26). Compute code is unchanged. Tests:
  `crates/amg-features/tests/solid_context_reads.rs`.
- A new feature that reads `RecomputeCtx::before()` bodies it doesn't list in `inputs()` must
  declare them the same way.
- Leftover: a pattern whose seed lives in another component (explicit `target` only) does not
  cover bodies that seed reads in its own component.

### 2026-11-28 - SOL-23 implemented (lane C)
- **Landed.** `amg_features::solid::pattern` extended (SOL-08 module, `solid.pattern` stays v1;
  data shapes unchanged, as in the Interfaces):
  - `pattern::path` (public; `PathChain`): `PatternKind::Path` resolves each path ref through
    CON-03 (`resolve_input` + `construct::curves::curve_of`), chains the curves end to end in
    order (reversing a curve where needed; ends meet within 1e-4 mm, else `Invalid { path }`),
    and places instance `k` at arc length `s_0 ± k·step` (`s_0 = start·L`, `start` ∈ [0, 1];
    `step` = spacing or extent/(count − 1); `−` when `flip`). `count` includes the seed (k = 0);
    instance numbers are positions (as circular). A closed path wraps around; on an open path,
    positions beyond the ends are dropped with a warning and their numbers stay reserved.
    `Identical` orientation translates by `C(s_k) − C(s_0)`; `PathDirection` maps the frame at
    `s_0` onto the frame at `s_k`, frames from `construct::math::plane_along_path` (normal =
    tangent, x = principal normal). Works for body and feature patterns.
  - `pattern::faces`: `PatternObjects::Faces` — the picked faces must belong to one solid and
    their boundary edges must all border one planar, unpicked face (the support; coplanar
    faces accepted). The tool is the picked faces (`Shape::sub_shape`) sewn (`sew`,
    `make_solid`) with a cap on the support plane (`make_face`, `FaceBounds::Loops` of the
    boundary edge curves, largest loop first). Faces reaching out along the support's outward
    normal make a boss (`Join` with the body), faces reaching into it a pocket (`Cut`); an
    explicit `target` overrides. Tool names `PatternFace#j` (source: the picked face's name)
    and `PatternCap#0`, then `instanced(self, k)`; one `apply_body_op` for all instances.
  - `pattern::adjust`: `ComputeType::Adjust` for feature patterns rebuilds every seed per
    instance in a sub-context whose `before` is a copy of the state just before the pattern
    with the seeds' sketches (`Dependency::Sketch` of the seed's `inputs()`) moved by the
    instance transform (`SketchGeom::plane`); everything else (extent targets, "All" bodies,
    start planes) stays, so extents are recomputed per instance. Tools named
    `instanced(self, k)`, one body operation, targets must agree across instances.
  - `seed_tools` split into `seed_tool_in(ctx, before, id)` (shared by identical and adjust).
  - Dialog (`pattern/dialog.rs`, pattern entry only): pattern type "Path" with fields `path`
    (edges/sketch curves), `path_count`, `path_spacing_type`, `path_distance`, `path_start`,
    `path_orientation`, `path_flip`; object type "Faces" with a `faces` selection (pattern
    only, the mirror dialog is unchanged); `compute` (Identical/Adjust, shown for features;
    `build` now reads it instead of keeping the old value). `read` returns values for path
    patterns now.
- **Done-when evidence** (`crates/amg-features/tests/solid_pattern_sol23.rs` +
  `solid_pattern_sol23/occt.rs`, 12 tests, 9 with OCCT; 4 unit tests in `path.rs`/`adjust.rs`;
  goldens `tests/golden/solid/sol23_*.toml`, 4 files, rel 1e-9; results BRepCheck-valid):
  1. `dw1_eight_instances_along_a_spline_follow_the_curve_frames`: 8 instances (seed + 7) of a
     4 mm block along a cubic sketch spline (6 poles), start 0.1, spacing 0.1·L, path direction:
     every instance transform maps the reference frame at `s_0` onto the reference frame at
     `s_k` (computed independently with `resolve_in` + `curve_of` + `path_param` +
     `plane_along_path`) within 1e-9 (worst origin deviation 1.8e-15 mm; axes within 1e-9);
     each copy's centroid within 1e-9 of the mapped seed centroid (worst 1.0e-16 mm); volume
     64 mm³ (1e-12). Identical orientation gives pure translations to `C(s_k)`. Also extent
     (last instance on the path end), flip from the end with two positions beyond the start
     (dropped with a warning), a disconnected path and start 1.5 as clean errors.
  2. `dw2_face_pattern_of_a_boss_*`: the 6 faces (5 sides + top) of a 5 mm pentagon boss on a
     120×60×10 plate, 3 along X at 35 mm: 76 440 mm³ = 72 000 + 3·296·5 (rel 3.8e-16), 24
     faces, one solid. `dw2_face_pattern_of_a_pocket_*`: the 6 faces of a 3 mm pentagon pocket
     cut up from the plate's bottom: 69 336 mm³ = 72 000 − 3·296·3 (rel 2.1e-16). Faces of a
     whole body or without one planar support fail with clear messages.
  3. `dw3_adjust_recomputes_a_to_object_extent_per_instance`: a 10×20 extrude to the bottom of
     a box tilted 20°, patterned 4 along X at 12 mm with adjust: mean heights
     32.10660 / 36.47424 / 40.84189 mm = 15 + tan 20°·(x0 + 35), worst deviation 7.1e-15 mm
     (limit 1e-6); each top at the slope height at its far edge within 1e-6. The same
     pattern with identical compute keeps the seed's height. A planar-face-profile seed with
     adjust fails ("… has none; use identical compute"); adjust along a straight path with
     an "All" hole seed matches the analytic plate volume (1e-9).
- **Gates.** `cargo fmt --all --check`; `cargo clippy -p amg-features --all-targets -D warnings`
  default and `--features occt`; `cargo nextest run -p amg-features` default (357 passed) and
  `--features occt` (566 passed), `-p amg-doc`. Existing tests unchanged.
- **Deviations.**
  - The path chain uses the curves in the given order (no automatic sorting of an unordered
    pick); Frenet frames flip at inflection points and are arbitrary (but deterministic) on
    straight stretches, as `plane_along_path` defines them.
  - Adjust moves only sketches; a seed without one (planar-face profile) is refused rather
    than moving that body. Sketch entities used as extent targets of the seed move too.
  - Face patterns need a planar support (Done-when scope); bosses/pockets on curved faces and
    faces spanning several supports are refused.
- **Interface changes elsewhere.** None: only public `amg-brep` (`sew`, `make_face`,
  `Shape::{sub_shape, curve, edge_faces, …}`, `tessellate_edge`) and `amg-doc`
  (`Evaluated::set_sketch`, `RecomputeCtx::new`) APIs; no new dependency. `context_reads`
  unchanged (feature patterns, identical or adjust, still declare `ContextBodies::Context`;
  face patterns list their faces, whose body is the participant). The pattern dialog gains
  fields (UI shell snapshots that render the pattern dialog would change; none found in
  `amg-ui`).
- **Leftovers.** Manipulator anchors for the path distance (UI-06); a viewport pick for the
  path chain order; face patterns on curved supports; adjust for face-profile seeds.

### 2026-11-28 - SOL-04 implemented (lane C)
- **Landed.** `amg_features::solid::primitive`: `PrimitiveFrame`, `BoxPrim` (`solid.box`),
  `CylinderPrim` (`solid.cylinder`), `SpherePrim` (`solid.sphere`) and `TorusPrim`
  (`solid.torus`) with the Interfaces' fields, all `register_feature!` v1 + `register_tool!`
  (so each can be a pattern/mirror seed), on `make_box`/`make_cylinder`/`make_sphere`/
  `make_torus` and `apply_body_op`. Dialog registrations in `primitive/dialog.rs` (plane =
  XY/YZ/XZ or "Selected plane" with a construction plane or planar face; Offset X/Y; Rotation;
  one length field per size; Centered for the box; Operation; Objects; group "create", orders
  40-43, not pinned). A preselected face or construction plane becomes the plane, preselected
  bodies the participants. Extra public items: `PrimitiveFrame::{on, xy, resolve}` and a `new`
  constructor per feature.
- **Behaviour.** The plane resolves through CON-03 `resolve_plane_input` (a face gives its
  outward normal); `u`, `v` are offsets along that plane's axes from its origin and `angle`
  turns the frame counter-clockwise about the normal. Box: `size` along frame x, y, normal
  with a corner at the frame origin; `centered` centres the footprint (x, y) only. Cylinder:
  base centre at the origin. Sphere and torus: centred at the origin, torus axis = normal,
  major > minor. Box height and cylinder height may be negative (below the plane, so a cut on
  a face makes a pocket); the other sizes must exceed the linear tolerance. Bad sizes fail
  `InvalidValue` naming the field. Names (naming note "Primitives"): `Gen { self, role, from:
  None }` with `XMin#0 … ZMax#0` by outward normal in the box frame (ZMin is the lower z in
  that frame, so a negative height puts `ZMax#0` on the plane); cylinder `Lateral#0`,
  `StartCap#0` (on the placement plane, also for a negative height), `EndCap#0`; sphere and
  torus `Surface#0`. `inputs()`: the plane (`Dependency::Construction` or `Entity`) and the
  participants.
- **Done-when evidence** (`crates/amg-features/tests/solid_primitive.rs` +
  `solid_primitive/occt.rs`, 20 tests with OCCT, 6 without; goldens
  `tests/golden/solid/sol04_*.toml`, 6 files, rel 1e-9; solids BRepCheck-valid):
  1. `dw1_*`, volume and area within 1e-9 relative of the analytic value: box 10×20×30
     6000 / 2200; cylinder r5 h10 250π / 150π; sphere r5 500π/3 / 100π; torus R10 r2
     80π² / 80π²; a 10 mm box joined onto a 40×40×10 plate 17000 / 5200, 11 faces; a
     10×10×4 pocket cut from it 15600 / 4960, 11 faces.
  2. `dw2_a_box_on_a_planar_face_follows_an_upstream_height_edit`: a 4×6×10 box on the top
     face of a plate; extrude height 20 → 80 → 12, the centroid is (unchanged x, y, h + 5)
     within 1e-9 and the face reference resolves `Exact`.
  3. `dw3_editing_a_box_size_keeps_the_face_names`: the six faces carry exactly the six role
     names; references to all six resolve `Exact` to the matching side after two size edits,
     and a downstream fillet edge stays `Exact`. `dw3_round_primitives_*` checks the other
     kinds' names.
  4. `dw4_every_primitive_dialog_*` and `dw4_anchors_*`: each kind's size fields (and only
     those) have a `ManipKind::Distance` manipulator, and `anchors` returns one anchor per
     size field, checked on an offset, rotated construction plane (box, centred box,
     cylinder, sphere, torus) and, in `dw2`, on a face.
- **Deviations.**
  - Anchors resolve the plane against the final evaluated state with the root component as
    context (`construct::resolve::resolve_in`; the registration's `anchors` gets no component
    path), so a feature in a placed component shows them in component coordinates, and a
    reference that no longer resolves there gives none. No anchors for the offsets or the
    rotation.
  - On a planar face, `u`/`v` are measured from the origin of the plane CON-03 returns (the
    face's surface origin). It is stable under the height edit tested here; a kernel result
    that re-parameterises the surface would shift the primitive in the plane.
  - No benchmark: the vision document sets no primitive budget.
- **Interface changes elsewhere.** None: only public `amg-brep`, `amg-doc` and
  `amg-features::construct` APIs, plus existing crate-private dialog helpers of lane-A solid
  modules (unchanged). No new dependency. amg-ui was not run; the four registrations are not
  pinned, so the shell snapshots are not expected to change.
- **Leftovers.** UI-06 manipulators for offsets and rotation; component-aware anchors; coil
  and pipe (SOL-19) reuse `PrimitiveFrame`.

### 2026-11-28 - SOL-24 implemented (lane C)
- **Landed.**
  - `amg_features::solid::component` (new, public): component operations are **transaction
    builders**, not features (components and occurrences are authored state, S01 §6; same
    pattern as S06 ASM-09's builders). Each returns the commands of one undo entry, checked on
    a copy of the design. `new_component_commands` (`NewComponent` at the identity in the
    active component + `AddFeature` with the new component as context; name made unique,
    `"Component"`, `"Component1"`, …), `pattern_occurrences` / `occurrence_pattern_transforms`
    (one `AddOccurrence` of the same component per source and instance, at `T_k · L`; `T_k`
    from the same layout code as body/feature patterns, `Pattern::instances` in a
    `RecomputeCtx::new` over the latest `Evaluated`; seed and suppressed instances left out;
    sources must be placed directly in the active component), `mirror_occurrences` (one
    `AddOccurrence` at `M · L`, `M = reflection(plane)`, determinant −1; the component is
    shared, not copied), `ComponentOpError`, `unique_component_name`, `reflection`.
  - `amg_features::dialog::new_feature_commands` + `CommitError`: commits a new feature from
    dialog values: `AddFeature`, or `new_component_commands` when `reg.new_component` returns a
    name. `solid.extrude` and `solid.revolve` register `new_component`; their "Operation" choice
    gains "New component" (id `new_component`, built as `BodyOp::NewBody`, "Objects" hidden).
  - Cross-component participants in `apply_body_op` (§3): a participant of a component placed
    exactly once below the feature's component is accepted; the tool is moved into that
    component's space (`Shape::transformed`, the product of the captured local transforms on
    the occurrence chain, `RecomputeCtx::occurrence_transform`). Cut / intersect per
    participant; a join fuses touched participants of one component (else `MixedComponents`)
    and tool-only solids go back to the feature's component. New `SolidError` variants
    `AmbiguousPlacement`, `MirroredPlacement`, `MixedComponents`. `live_body` (combine, split,
    moves) is unchanged: those still require the context component.
  - `solid.pattern` / `solid.mirror` stored with `PatternObjects::Occurrences` now fail with a
    clear `InvalidValue` pointing at the builders (was `NotYet`).
- **Done-when evidence** (`crates/amg-features/tests/solid_component.rs` +
  `solid_component/occt.rs`, 11 tests, 5 with OCCT; 2 unit tests in `component.rs`; goldens
  `tests/golden/solid/sol24_*.toml`, 2 files, rel 1e-9; results BRepCheck-valid):
  1. `dw1_extrude_with_new_component_commits_one_component_in_one_undo_entry` and
     `occt::dw1_new_component_extrude_*`: the extrude dialog with "New component" on a 10×20
     rectangle (default 10 mm) gives 2 commands; committed as one transaction: undo stack +1,
     components +1 (named `Component`), the feature's context is the new component, exactly one
     new body, in that component, volume 2000.0000000000005 mm³ (rel 1e-9), bbox (0,0,0)–
     (10,20,10) (identity placement). One undo removes component, feature and body. Also: the
     option exists in both dialogs and hides "Objects"; `new_component` is `None` for other
     operations; a second "New component" is named `Component1`; "New body" stays one
     `AddFeature`.
  2. `dw2_circular_pattern_of_an_occurrence_creates_six_occurrences`: full circular pattern of 6
     about Z of an occurrence at `T(40,0,5)·Rz(15°)`: 5 `AddOccurrence` in one undo entry, 6
     occurrences of the component, each local and evaluated world transform equal to
     `Rz(k·60°)·L` (worst element deviation 3.2e-14, limit 1e-12); one undo leaves 1.
     `dw2_mirror_creates_a_mirrored_occurrence`: mirror about YZ: one occurrence at
     `diag(−1,1,1)·L` (deviation 0), determinant −1, same component. With OCCT
     (`occt::dw2_pattern_and_mirror_*`): a 10×10×4 body in the component seen through all 7
     occurrences (seed, 5 instances, XZ mirror) sits at the expected world centres (worst
     4.3e-14 mm) and the design still has one body. Also a 3×2 rectangular pattern with one
     suppressed instance (4 new occurrences at `(20 i, 15 j)`), an empty layout giving no
     commands, and clean errors for occurrences outside the active component, empty or
     non-occurrence selections, missing occurrences, a count of 2.5, and a stored component
     pattern feature.
  3. `occt::dw3_cut_in_assembly_context_*`: a 40×40×10 box in sub-component "Sub" placed at
     `T(50,20,5)·Rz(90°)`; a root extrude cut (10×10, 30 mm) with the Sub body as participant:
     15 000 mm³ (rel 1e-9), 10 faces, body still in Sub, centroid in Sub's space
     (19.3333, 20, 5) within 4.0e-15 mm (hole at local (30, 20)); golden
     `sol24_cut_sub_component`; the cut is one undo entry and one undo restores 16 000 mm³.
     Moving the occurrence +5 mm along world X recomputes the cut (hole now at local (30, 25),
     centroid y 19.6667, exact). `occt::dw3_a_slot_*`: a slot across the box splits it; both
     7200 mm³ pieces stay in Sub (`successors`, golden `sol24_split_sub_component_piece1`).
     `occt::dw3_join_and_errors_*`: a root join boss fused into the Sub body (18 000 mm³, still
     in Sub); a second placement of Sub gives `AmbiguousPlacement` (2), undo heals it; a body of
     a sibling component gives `OtherComponent`.
- **Gates.** `cargo fmt --all --check`; `cargo clippy -p amg-features --all-targets -D warnings`
  default and `--features occt`; `cargo clippy -p amg-doc --all-targets -D warnings`;
  `cargo nextest run -p amg-features` default (369 passed) and `--features occt` (585 passed),
  `-p amg-doc` (279 passed).
- **Deviations.**
  - Component patterns and mirrors are not timeline features (no `solid.pattern` node is
    stored): editing a count later means deleting / adding occurrences. A parametric component
    pattern would need features that create occurrences in recompute (S01 change, not
    requested here). `PatternObjects::Occurrences` stays in the data shape as the builders'
    input.
  - "New component" is offered on extrude and revolve only (not thicken, combine or the
    primitives SOL-04 adds in parallel); editing such a feature reads back as "New body" (the
    component is authored state, as decided 2026-11-08).
  - One existing assertion changed: `solid_extrude.rs::dw7_*` checked `new_component.is_none()`
    "until SOL-24"; it now checks `is_some()`.
  - Join in an assembly context needs every touched participant in one component; cut and
    intersect take participants of several components. Participants seen through a mirrored
    occurrence are refused (a reflected tool would need `mirrored`, not a located copy).
- **Interface changes elsewhere.** S01 (`amg-doc/src/recompute.rs`, recorded in S01 Interface
  changes 2026-11-28): cache keys of features reading bodies of other components cover the
  occurrence placements; `CreatedFrom` pieces stay in their parent body's component. S11
  (recorded 2026-11-28): `dialog::new_feature_commands` for UI-05; requests for UI-05 to commit
  through it and for a selectable occurrence `EntityRef`. No `amg-ui` change, no new dependency.
  `context_reads` unchanged: cross-component participants are listed in `inputs()` (the
  placement part is the S01 change). The extrude and revolve "Operation" dropdowns have a fifth
  option, so a UI snapshot that renders an open operation dropdown would change (none found).
- **Leftovers.** UI-05: commit through `new_feature_commands`; an occurrence pick so the pattern
  and mirror dialogs can offer components (S11 request); "New component" for thicken / combine
  / primitives; parametric component patterns (S01); joins across components.

### 2026-11-28 - SOL-18 thin extrude and thin revolve (lane C)
- **Landed.** `thin` on `solid.extrude` and `solid.revolve` (was `NotYet`).
  - New `solid::extrude::thin` (crate-private, used by revolve too): the walls as
    `ProfileFace`s with the new source `ProfileSource::Wall(Wall { cap, edges, refs })` (new
    public types `Wall`, `WallEdge`, `WallBoundary`). Everything after the profile (starts,
    directions, extents, taper, to object, body operations, pattern tools) is the existing
    sweep code.
  - Selection: a sketch region gives one closed wall per loop (outer and holes); sketch curves
    (`EntityRef::SketchCurve`) are joined by end points within 1e-6 mm into open or closed
    chains; more than two curves at a point is an error; curves that only cross are separate
    chains. All walls of the feature are fused into one tool. Planar-face profiles are refused.
  - Orientation: closed chains run counter-clockwise about the sketch normal; an open chain
    starts at its end on the end curve with the lower entity id. `Side1` widens to the chain's
    left (into a closed chain), `Side2` to the right, `Center` half each. `flip` reverses the
    sweep only.
  - Naming: `ThinInner#side` (left boundary) and `ThinOuter#side` (right) from
    `SketchCurve`; `ThinEnd#(2*side + end)` from the `SketchPoint` at that chain end (`end` 0 =
    start), so side 0 has `ThinEnd#0/1` and side 1 `#2/#3`; caps `StartCap#side` /
    `EndCap#side` from `Profile { sketch, chain curve ids }` (for a region's outer loop this
    equals `Region(p)`). No name depends on thickness, side mode or pick order.
  - Dialogs (schema only): both gain "Thin", "Wall thickness" and "Wall side" (the last two
    hidden while off); "Profiles" also accepts sketch curves, and curves without Thin are a
    field error. The extrude defaults turn Thin on when only sketch curves are preselected; the
    revolve keeps taking a preselected curve as the axis. SOL-24's "New component" is
    unchanged.
  - `amg-brep`: R-S04-3 `make_thin_face` (S02 Interface changes 2026-11-28).
- **Done-when evidence** (`crates/amg-features/tests/solid_thin.rs` + `solid_thin/occt.rs`,
  18 tests, 14 with OCCT; 5 unit tests in `thin.rs`; `crates/amg-brep/tests/thin.rs`, 7 tests;
  23 goldens `tests/golden/solid/sol18_*.toml`, rel 1e-9; t 2 mm, h 10 mm):
  1. Line 10 + tangent quarter arc r 5 (L = 10 + 5π/2): centre volume = L·t·h = 357.079632679
     (rel 3.2e-16). A side mode moves the mid line t/2 off the chain, so its length is
     L ∓ (π/2)(t/2): side 1 325.663706144 (rel 1.8e-16), side 2 388.495559215 (rel 0). With no
     net turn L·t·h holds literally in every mode: the line + two opposite quarter arcs
     (514.159265359, rel ≤ 2.3e-16) and a single tilted line (260, rel ≤ 2.2e-16).
  2. 20 × 10 rectangle (region and four curves): 1040 / 1360 / 1200 mm³ for side 1 / side 2 /
     centre (rel ≤ 3.4e-16), 10 faces. Also circles, a region with a hole and two regions
     sharing an edge.
  3. Line + tangent arc in XZ revolved about Z, 90° and 360°, each side mode, against the
     closed-form first moment: rel ≤ 1.2e-15.
  4. Thickness and side edits (4 per case) keep every face, edge and vertex name, and every
     reference taken before resolves `Exact` (open extrude 50, closed extrude 50, open revolve
     38 references; also spline and ellipse walls and a downstream fillet).
- **Deviations.**
  - Done-when 1 as written ("volume L·t·h in each side mode") is not true for a chain with a
    net turn in a side mode; tested as above.
  - Existing assertions that expected `NotYet` for thin now expect "thickness must be positive"
    (`solid_extrude.rs`, renamed `a_thin_extrude_needs_a_positive_thickness`;
    `solid_revolve/occt.rs`).
  - `make_thin_face` returns `ThinFace` (a `FaceBuild` plus the left edges), not a bare
    `FaceBuild`: the feature cannot name the two boundaries otherwise.
- **Gates.** `cargo fmt --all --check`; `cargo clippy --all-targets -D warnings` for
  `amg-features` and `amg-brep` (default and `--features occt`) and `amg-doc`;
  `cargo nextest run` for `amg-features` (default and occt), `amg-brep` (default and occt) and
  `amg-doc` (counts in the WP report).
- **Interface changes elsewhere.** S02 (`amg-brep`, `amg-occt-sys`, lane A), additive. No
  `amg-ui`, `amg-doc` or dependency change. No new toolbar entry, so no new shell snapshot
  change is expected; the Extrude and Revolve dialogs show one more checkbox ("Thin"), so a
  snapshot of an open dialog would change (amg-ui was not run).
- **Leftovers.**
  - Kernel `mass_props` (BREP-09, lane A) integrates solids with free-form faces poorly: the
    closed 15 × 9 ellipse wall reads 740.13 mm³ for a true 765.81 mm³ (3.4 %), while its cap
    areas (rel 3e-9) and a fine mesh (rel 4e-7) are right; `Shape::volume` reads 765.34. A
    plain extrude between two analytic ellipses is exact. The value is the same with exact
    offset curves and with the B-splines.
  - `BRepOffsetAPI_MakeOffset` fails cleanly on a very acute outer corner (5.7° interior; 15°
    works) and on walls that close or touch themselves (no partial walls are built).
  - Thin walls from planar-face profiles; a thickness manipulator and distance anchors for
    curve selections (UI-06); picking priority between sketch curves and regions (UI-05, the
    "Profiles" filter now includes curves).

### 2026-11-28 - SOL-29 Fusion-parity harness and recompute budget (lane C)
- **Landed** (tests and a bench of `amg-features` only; no product code, no dependency, no
  change in another lane's code).
  - `tests/solid_fusion.rs` + `tests/solid_fusion/`: every `<name>.step`/`.stp` directly in
    `$AMG_TESTDATA_DIR/fusion/` is rebuilt by the Rust script of the same name (`designs.rs`)
    and compared: same solid count, volume and area within 1e-4 relative, bbox within 1e-3 mm.
    Above 1e-6 mm sub-shape tolerance the IO-06 rule widens the volume bound to
    area x tolerance and the bbox bound to the tolerance. A script can be `Pending(reason)` or
    carry an `explained` reason for an accepted difference. Missing folder, unset variable or
    a build without `occt`: skip. A STEP without a script, a script without a STEP and an LFS
    pointer are listed, and fail only with `AMG_REQUIRE_TESTDATA=1`. Folder layout and script
    format: `tests/solid_fusion/README.md`.
  - `tests/solid_recompute/part.rs`: the 50-feature part (20 sketches, 20 extrudes incl.
    cuts, 4 fillets, 3 chamfers, 1 shell, 2 feature patterns; one body, 150 faces). The first
    sketch carries a driving width dimension. `tests/solid_recompute_50.rs` checks health,
    validity, width and volume across three edits; `benches/sol_recompute.rs` is
    `budget/sol-recompute-50`.
- **Done-when evidence.**
  1. No owner design is supplied (`$AMG_TESTDATA_DIR/fusion/` does not exist), so the owner
     test skips and `DESIGNS` is empty. The self-made bracket (STEP written by our AP242
     exporter into a temp folder) matches: volume 7.8e-15 rel, area 2.5e-15 rel, bbox
     2.5e-29 mm, and its volume equals the analytic value within 1e-9. A 0.01 mm taller
     script fails on bbox, a 0.05 mm wider bore on volume and area; a failing feature and an
     unreadable file fail.
  2. **Budget missed.** Ryzen 7 5700X3D, shared with other builds, never idle. Lowest 15-edit
     median 665.6 ms (min 654.7, max 683.7); other runs 711.5 ms and 836.2 ms, and
     1.86-2.02 s at 100 % CPU load. 32 of 50 features recompute (the 18 sketches on origin
     planes stay cached) and their computes are almost the whole time (694.9 of 711.5 ms).
     5 slowest in that run: pin pattern 131.0 ms (18.9 %), pin 108.4 (15.6 %), rim notch
     105.0 (15.1 %), vent pattern 35.4 (5.1 %), rib 4 27.1 (3.9 %); the top three were the
     same in every table read. Asserted only with `AMG_STRICT_TIMING=1` (fails there).
- **Deviations.**
  - `solid.hole` (SOL-12) does not exist: the part's holes are circle cut-extrudes.
  - No owner design was rebuilt, so the Fusion checks other WPs left for SOL-29 (join with
    nothing, a severing combine cut, to-object on curved targets) stay open.
- **Leftovers.**
  - Owner: supply 5-10 designs as the README describes; each then needs its script.
  - Recompute speed (lane A, S02/S20, with KH-20): the three features after the blends and
    chamfers (pin, pin pattern, rim notch) are about half of the time; one boolean feature
    there costs about 105 ms against about 27 ms for a rib before the blends. Where that time
    goes (boolean, unify, validation or naming) was not measured. Re-measure on an idle
    machine.
