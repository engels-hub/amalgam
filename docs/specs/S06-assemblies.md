# S06 — Components, assemblies & joints

| | |
|---|---|
| **Status** | Ready |
| **Priority** | P0 (components) / P1 (joints) / P2 (motion) |
| **Crates** | `amg-doc` (component tree commands, placements, `amg-doc::joints` data + solver), `amg-features::assembly`, `amg-ui::assembly` (lane E, owner 2026-11-09, QUESTIONS #60) |
| **Lane** | E |
| **Depends on** | S01 (DOC-05 commands, DOC-06 naming, DOC-09 events, DOC-12 recompute job); S02 (BREP-02 `Shape::transformed`, BREP-08 tessellation for contact); S03 (SK-03 `solve::system` layer, `numeric_rank`); S05 (CON-03 `construct::resolve` for joint origins); S07 (MESH-03 BVH for contact); S11 (UI-05/06/07/21 framework) |
| **Parity** | Assemblies section of `03-fusion-parity.md` |

## Summary
Fusion-style multi-component designs inside one file: components with occurrences, an
active-component editing context, joints with DOF and limits, a 3D joint solver with drag,
as-built joints and rigid groups, motion links, drive joints, motion studies and contact sets.
Linked external designs come later (P3). The design note is
[`docs/notes/assemblies.md`](../notes/assemblies.md); this spec states the decisions and the
contract, the note has the reasoning, prior art, math and budgets.

## Scope
**In:**
- Component tree editing: new component, new occurrence of a component (copy/paste), delete
  with cascade, reparent, ground, rename, visibility; active component.
- Solved placements in evaluated state and the world-transform queries other lanes use
  (`Evaluated::body_to_world`, `instances`).
- "Create components from bodies" and moving bodies between components.
- Move/rotate occurrences (gizmo, point to point).
- Joint origins, the seven Fusion joint types, offsets, flip, limits and rest values, joint
  origin features.
- The joint solver, the assembly stage of recompute and joint-aware drag.
- As-built joints, rigid groups, motion links, drive joints, motion studies, contact sets.

**Out:**
- STEP assembly import/export: S12 IO-06 (uses ASM-01 commands).
- Component patterns and mirror, "New component" body op, cross-component participants: S04
  SOL-24 (uses ASM-01/02).
- Mass properties and interference over assemblies: S05 INS-10, INS-04 (use ASM-01 queries).
- Exploded views and animation: S19 LAT-03.
- Physics (dynamics, forces, sliding contact). Motion is kinematic only.
- "Make independent" copies of a component definition (needs feature duplication with
  reference remapping; not planned).

## Design
Full detail and reasoning in [`docs/notes/assemblies.md`](../notes/assemblies.md).

### 1. Data model
- **Geometry lives in component space** (note §1). A component's bodies are evaluated in its
  own coordinates; each occurrence places them. This is already how DOC-03 works
  (`EvalBody::component`, cache key hashes the context id, not transforms), so moving an
  occurrence never recomputes the moved component's features.
- **Occurrence transforms are rigid or mirrored** (orthonormal linear part, `|det| = 1` within
  1e-9). Scale is refused (`ModelError::InvalidValue`).
- **Authored vs evaluated** (note §2). Authored positions come in three layers (§2):
  `Occurrence::transform` (base), Capture Position timeline nodes (applied in timeline order)
  and pending moves (`Design::pending_positions`, on top at the marker). `Evaluated::placements`
  is the position after the assembly stage, seeded from all three. Display, picking, measure,
  INS-04/INS-10 and IO-06 export use placements. Features that read another component through
  `EntityRef::InContext` see the *captured* position at their own timeline index (base +
  capture nodes before them, never pending moves). That keeps recompute acyclic and
  deterministic, and a move never re-runs a feature.
- **Paths.** `OccPath = Vec<OccurrenceId>` from a component downward (the existing
  `Design::occurrence_paths` and `EntityRef::InContext` already use `Vec<OccurrenceId>`). World
  queries take paths from the root.
- **Active component** is session state of the `Document` (not undoable, not saved, root on
  open). The UI uses it as `AddFeature::context`. Fusion records the context per feature; so
  does `FeatureNode::context`.
- **Delete cascade.** Deleting the last occurrence of a component deletes the component, its
  sub-components that have no other occurrence, and every timeline feature whose context is
  one of them, in one command (through the existing `DeleteFeatures` path, so model parameters
  are converted as usual). Features elsewhere that referenced them get `Error` with an
  upstream/resolve cause.

### 2. Moving occurrences: pending positions, Capture Position, Revert (QUESTIONS #59)
Fusion's model (owner, 2026-11-09; detail and reasoning in note §3):
- **Pending.** Every interactive move (Move tool, joint-aware drag, Drive Joints, rest values,
  motion-study "use this frame") ends in one `Command::SetPendingPositions` (one undo entry per
  move). Pending moves are authored state (`Design::pending_positions`): undoable, saved in
  `design.json` and the crash journal, shown in the viewport (they seed the assembly stage), but
  invisible to features (in-context refs read captured positions only).
- **Capture Position** (`Command::CapturePositions`) adds one timeline node
  `assembly.capture_position` at the marker holding the solved placements and joint states of
  every pending (and out-of-date) occurrence, and clears pending. Features after the node see
  those positions; rolling the marker before it, suppressing or deleting it brings the earlier
  positions back (Fusion: editing a captured position affects everything after it).
- **Revert** (`Command::RevertPositions`) clears pending: occurrences go back to their last
  captured position (latest capture node before the marker, else the base transform) and the
  joints re-solve. Undoable, no dialog (as in Fusion).
- **Prompt.** Starting a position-dependent action while moves are pending (Joint Origin, a
  feature with an in-context pick, moving the timeline marker, reordering or editing a node)
  asks "Capture Position / Revert / Cancel" (Fusion: "Capture Position / Continue"). The
  `SetMarker` command itself drops pending moves in the same undo entry, so headless use is
  deterministic (Fusion also loses uncaptured positions on rollback).
- **Joints need no capture.** A joint is a timeline node and the solver places its
  occurrences. Joint creation (ASM-11) commits `AddFeature` plus `SetOccurrenceTransforms`,
  which writes the snap into the layer effective at the marker (latest capture node listing the
  occurrence, else the base transform), so no Position panel appears after creating a joint.
  Editing a joint or recomputing while moves are pending keeps them as seeds; the solver
  satisfies the joints from there.
- **Grounding.** Grounded occurrences cannot get pending moves. Grounding an occurrence that
  has one keeps it at the pending position (still pending until captured or reverted).
- **Out of date.** When a geometry edit moves a joint origin, solved placements differ from
  the captured seeds; the root shows "positions out of date", whose action is Capture Position.
- **Direct mode** has no timeline: moves commit at once (`SetOccurrenceTransforms` on the base
  layer) and there is no pending state.

### 3. Joints are timeline features
Joints, as-built joints, rigid groups, motion links and joint origins are features in
`amg-features::assembly` (note §4). They resolve their origins (DOC-06 + CON-03) to frames in
component space and return an `AsmItem` in the new `FeatureOutput::assembly`. They read no
placements, and list `Dependency::Entity(target)` for the inner target of each origin, so
moves never re-run them. `Component::joints` stays empty (kept so files load; removed at the
next `FORMAT_VERSION` bump).

### 4. Joint origins and joint types
- A joint origin is a frame on geometry: path, snap (`OriginAt`: vertex, edge start/mid/end,
  circle centre, face centre, axis point, construction entity, component origin, picked point),
  and `Align` (flip, angle, offset x/y/z as `Expr`, optional x-axis reference). z comes from the
  geometry; x follows the deterministic rule of note §5.1 (projection of component +X, else +Y).
- Joint types and their residual rows: note §5.2. Rigid 0 DOF, revolute 1, slider 1,
  cylindrical 2, pin-slot 2, planar 3, ball 3. Joint values (θ, d, …) are extracted from the
  relative frame and unwrapped against the stored joint state.
- Limits `{min, max, rest}` per value as `Expr`. Enforced by the solver; `rest` is applied when
  a drag of that joint ends.
- Joint states (authored): last unwrapped values, layered like positions (base
  `Design::joint_states`, capture nodes, pending) and written by the same command as the move.
  Needed for angles beyond ±π and for motion links (note §5.5).

### 5. Joint solver and assembly stage (`amg-doc::joints`)
- Maximal coordinates: one rigid pose per unknown, SE(3) increments applied on the left with a
  retraction after each step; analytic Jacobians (note §6.1).
- Built on `amg_sketch::solve::{EquationSystem, Residual, numeric_rank}`; the LM loop is our own
  because `NativeSolver`'s pipeline is crate-private and additive-only. `nalgebra` stays inside
  `amg-doc` (ADR-006).
- Decomposition (note §6.2): per context component, bottom-up; rigid joints and rigid groups
  substituted as clusters; independent connected components. A component placed twice is
  solved once (its copies move alike).
- Seeded from `Design::seed_transform` (captured at the marker, overridden by pending); damped min-norm LM (under-constrained parts move as little as
  possible); limits by active set; DOF by numeric rank; per-joint diagnostics become
  `Health::Warning` on the joint node (note §6.3).
- Assembly stage after the timeline pass: collect `AsmItem`s, key them, solve, store placements
  (note §7). Deterministic: 100 runs give bit-identical placements.
- Drag (`AsmDrag`, note §6.4): weighted min-norm step toward the cursor in the constraint
  null space, Newton projection, limit clamp, optional contact. Synchronous on the UI thread up
  to 2 000 unknowns, a job above.

### 6. Motion and contact
- Motion links: residual between two joint values with a ratio from two `Expr`s and a reverse
  flag (note §8.1).
- Drive joints: a tool that sets one value with a temporary row; OK gives a pending move.
- Motion study: `ExtRecord` `assembly.motion_study` with per-joint keyed values over steps;
  playback frames are computed by a job and not saved; "use this frame" turns the shown frame
  into a pending move (note §8.2).
- Contact sets: `ExtRecord` `assembly.contact` (Off / Sets / All). Pairs in a set are tested
  with BVH overlap on meshes (solids tessellated by BREP-08, "Fine" preset); a penetrating
  step is bisected to the contact and stopped (note §8.3). Off by default.

### 7. UI (`amg-ui::assembly`)
- Browser: activate radio per component, grounded pin icon, context menus (new component,
  activate, ground, create components from bodies, copy/paste, delete, move to component),
  pending marker on moved occurrences, "positions out of date" badge.
- Position panel in the toolbar (Capture Position, Revert) while moves are pending, and the
  Capture / Revert / Cancel prompt before position-dependent actions (§2).
- Capture Position nodes show in the timeline with a position icon; rename, suppress, delete
  and reorder work as for any node. No edit dialog in v1 (to change one: roll the marker after
  it, move, capture again, delete the old node).
- Tools: Move/Copy for occurrences (triad, point to point, fields), Joint, As-built Joint,
  Joint Origin, Rigid Group, Motion Link, Drive Joints, Motion Study panel, Contact Sets.
  Feature dialogs use S11's generic dialogs where they fit and hand-built tools
  (`EditAction::StartTool`) for picking origins and dragging.
- Overlays: joint glyphs (type icon at the origin, DOF arrows) via RND-07.
- The module is owned by lane E (owner, 2026-11-09, QUESTIONS #60) and uses only the S11
  framework API.

## Interfaces
Provisional Rust signatures. Names can still change during implementation, but other lanes
code against these shapes. Existing items are not repeated; everything here is additive to
the current `amg-doc` code.

```rust
// ===== amg-doc::model / command (ASM-01) =========================================
pub type OccPath = Vec<amg_doc::OccurrenceId>;     // from a component downward; empty = that component

impl Design {
    /// Effective joint state at the marker: pending, else the latest capture node before the
    /// marker, else base (ASM-04; layers from ASM-13).
    pub fn joint_state(&self, joint: FeatureId) -> Option<&JointState>;
    /// ASM-13. Empty after Capture Position, Revert, `SetMarker`, and always in Direct mode.
    pub fn pending_positions(&self) -> &PendingPositions;
    pub fn has_pending_positions(&self) -> bool;
    /// ASM-13: local transform of `o` as seen by timeline node `index`: the base transform
    /// overridden by the capture nodes before `index` (never pending). `index = nodes.len()`
    /// is the end of the timeline.
    pub fn captured_transform_at(&self, o: OccurrenceId, index: usize) -> Option<Transform>;
    /// ASM-13: what the assembly stage seeds from: captured at the marker, then pending.
    pub fn seed_transform(&self, o: OccurrenceId) -> Option<Transform>;
    /// Features whose context is `c`, in timeline order.
    pub fn features_in(&self, c: ComponentId) -> impl Iterator<Item = FeatureId> + '_;
    /// True if `path` walks parent → child starting in `from`.
    pub fn is_valid_path(&self, from: ComponentId, path: &[OccurrenceId]) -> bool;
}

// New `Command` variants (existing NewComponent, RenameComponent, SetOccurrenceTransform and
// SetOccurrenceVisible stay as they are).
pub enum Command {
    /* … existing … */
    /// Another placement of an existing component (copy/paste, STEP shared prototypes).
    AddOccurrence { parent: ComponentId, component: ComponentId, transform: Transform,
                    name: Option<String> },                                    // -> CreatedId::Occurrence
    /// Removes occurrences; a component left without occurrences is deleted with its
    /// features and orphaned sub-components (§1 cascade).
    DeleteOccurrences { occs: Vec<OccurrenceId> },
    /// Reparent; `keep_world` recomputes the transform so the world placement is unchanged.
    MoveOccurrence { occ: OccurrenceId, parent: ComponentId, keep_world: bool },
    SetOccurrenceGrounded { occ: OccurrenceId, grounded: bool },
    SetOccurrenceName { occ: OccurrenceId, name: Option<String> },
    /// A direct, non-pending placement (joint creation snap, scripts, Direct mode), with the
    /// joint states of the joints involved (ASM-04). Writes the layer effective at the marker:
    /// the latest capture node before the marker that lists the occurrence, else the base
    /// transform (ASM-13; base only before ASM-13). Clears pending entries of those occurrences.
    SetOccurrenceTransforms { moves: Vec<(OccurrenceId, Transform)>,
                              joint_states: Vec<(FeatureId, JointState)> },
    /// ASM-13: an interactive move (gizmo, drag, drive, rest, motion-study frame), merged into
    /// `Design::pending_positions`. One undo entry. Refused (`CommandError::Invalid`) for a
    /// grounded occurrence and in Direct mode.
    SetPendingPositions { moves: Vec<(OccurrenceId, Transform)>,
                          joint_states: Vec<(FeatureId, JointState)> },
    /// ASM-13: Capture Position. Adds an `assembly.capture_position` node at the marker (root
    /// context) with these values and clears pending. Built by `joints::capture_command`.
    /// -> CreatedId::Feature
    CapturePositions { moves: Vec<(OccurrenceId, Transform)>,
                       joint_states: Vec<(FeatureId, JointState)>, name: Option<String> },
    /// ASM-13: Revert. Clears pending. Refused (`Invalid`) when nothing is pending.
    RevertPositions,
}

/// ASM-13: pending moves (authored; `#[serde(default)]` on `Design`, saved in `design.json`).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PendingPositions {
    pub moves: PMap<OccurrenceId, Transform>,           // local, relative to the parent
    pub joint_states: PMap<FeatureId, JointState>,
}

impl Document {
    /// Session state: the component new features go into (root after open/new).
    pub fn active_component(&self) -> ComponentId;
    /// Emits `DocEvent::ActiveComponentChanged`. Fails if `c` does not exist.
    pub fn set_active_component(&mut self, c: ComponentId) -> Result<(), CommandError>;
}
pub enum DocEvent { /* … existing … */ ActiveComponentChanged(ComponentId),
                     PendingPositionsChanged /* ASM-13: set, captured, reverted or dropped */ }
// `OccurrenceTransformChanged(o)` is also emitted when the *solved* placement of `o` changes.

// ===== amg-doc::eval (ASM-01; solved by ASM-04) ===================================
/// Placements of every occurrence after the assembly stage (= authored before ASM-04).
#[derive(Clone, Debug, Default)]
pub struct Placements { /* occ -> (parent, component, local); component -> occs; crate-private */ }
impl Placements {
    pub fn local(&self, o: OccurrenceId) -> Option<Transform>;            // relative to parent
    /// Product of the locals along `path` (from the root). None if the path is invalid.
    pub fn world(&self, path: &[OccurrenceId]) -> Option<Transform>;
    pub fn len(&self) -> usize;
}
#[derive(Clone, Debug, PartialEq)]
pub struct BodyInstance { pub path: OccPath /* from root */, pub world: Transform }
impl Evaluated {
    pub fn placements(&self) -> &Arc<Placements>;
    pub fn world_transform(&self, path: &[OccurrenceId]) -> Option<Transform>;
    /// S05 request: world transform of `body` seen through `path` (from the root). None if
    /// the path does not end at the body's component.
    pub fn body_to_world(&self, body: BodyId, path: &[OccurrenceId]) -> Option<Transform>;
    /// Every placement of a body (one per occurrence path of its component), path order.
    pub fn instances(&self, body: BodyId) -> Vec<BodyInstance>;
    pub fn asm_diagnostics(&self) -> &[AsmDiagnostic];                    // empty before ASM-04
}

// ===== amg-doc::feature additions =================================================
pub struct FeatureOutput { /* … existing … */ pub assembly: Option<AsmItem> /* ASM-03 */ }
pub enum BodyChangeKind {
    /* … existing … */
    /// The body now belongs to `component` (ASM-09). Geometry is replaced by `geom`
    /// (already expressed in the new component's space).
    Moved { component: ComponentId, geom: BodyGeom, naming: Option<BodyNaming> },
}
pub enum ConstructionGeom { /* … existing … */ Frame(Transform) /* joint origin, ASM-03 */ }

// ===== amg-doc::joints (data: ASM-03; solver + drag: ASM-04; links: ASM-06; contact: ASM-07)
/// A rigid frame (orthonormal, right-handed): columns x, y, z; translation = origin. mm.
pub type Frame = Transform;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum JointKind { Rigid, Revolute, Slider, Cylindrical, PinSlot, Planar, Ball }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum JointAxis { X, Y, Z }
/// One joint value. Units: rad for rotations, mm for slides.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum ValueSel { Rotation, Slide, SlideX, SlideY, Pitch, Yaw, Roll }
impl JointKind {
    pub fn dof(self) -> u32;
    pub fn values(self) -> &'static [ValueSel];
}

/// Evaluated joint: frames in the component space of the leaf of each path.
#[derive(Clone, Debug, PartialEq)]
pub struct JointEval {
    pub kind: JointKind,
    pub motion_axis: JointAxis,             // rotation / slide axis of A (Z by default)
    pub slide_axis: Option<JointAxis>,      // pin-slot only; ≠ motion_axis
    pub a_path: OccPath, pub a: Frame,      // "to" side (paths from the joint's context)
    pub b_path: OccPath, pub b: Frame,      // "from" side
    pub limits: Vec<(ValueSel, Limits)>,
}
#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct Limits { pub min: Option<f64>, pub max: Option<f64>, pub rest: Option<f64> }
#[derive(Clone, Debug, PartialEq)]
pub struct RigidGroupEval { pub members: Vec<OccPath>, pub relative: Vec<Transform> /* to members[0], captured */ }
#[derive(Clone, Debug, PartialEq)]
pub struct MotionLinkEval { pub a: (FeatureId, ValueSel), pub b: (FeatureId, ValueSel),
                            pub k: f64, pub c: f64 }                   // v_b = k·v_a + c
#[derive(Clone, Debug, PartialEq)]
pub enum AsmItem { Joint(JointEval), RigidGroup(RigidGroupEval), MotionLink(MotionLinkEval) }

/// Unwrapped joint values (authored, `Design::joint_states`).
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
pub struct JointState { pub values: Vec<(ValueSel, f64)> }

/// Values of `kind` for frames given in one common space; angles unwrapped to `seed`.
pub fn joint_values(j: &JointEval, a_world: &Frame, b_world: &Frame, seed: Option<&JointState>) -> JointState;
/// Closed-form placement of the "from" occurrence for a tree joint (ASM-03 dialog snap):
/// the new local transform of `b_path[0]` relative to the context.
pub fn snap_placement(j: &JointEval, p: &Placements, ctx_world: &Transform, values: &JointState)
    -> Result<Transform, AsmError>;

#[derive(Clone, Debug, PartialEq)]
pub struct AsmDiagnostic { pub feature: FeatureId, pub issue: AsmIssue }
#[derive(Clone, Debug, PartialEq)]
pub enum AsmIssue {
    NotConverged { residual: f64 }, Overconstrained { residual: f64 }, Redundant, Flipped,
    LimitViolated { value: ValueSel, at: f64 }, BadPath,
}
#[derive(thiserror::Error, Debug, Clone, PartialEq)]
pub enum AsmError {
    #[error("occurrence path is not valid in this component")] BadPath,
    #[error("occurrence {0} is grounded")] Grounded(OccurrenceId),
    #[error("nothing to drag: the occurrence has no free motion")] NoFreedom,
    #[error("joint solve did not converge (residual {0} mm)")] NotConverged(f64),
    #[error("cancelled")] Cancelled,
}

#[derive(Clone, Debug)]
pub struct SolveOptions { pub tol: f64 /* 1e-9 mm */, pub max_iter: u32 /* 50 */, pub limit_rounds: u32 /* 8 */ }
#[derive(Clone, Debug)]
pub struct AsmSolution {
    pub placements: Placements,
    pub states: Vec<(FeatureId, JointState)>,
    pub diagnostics: Vec<AsmDiagnostic>,
    pub dof: Vec<(ComponentId, u32)>,            // remaining DOF per context
    pub stats: AsmStats,
}
#[derive(Copy, Clone, Debug, Default)]
pub struct AsmStats { pub systems: u32, pub unknowns: u32, pub clusters: u32, pub iterations: u32 }
/// The assembly stage (called by the recompute engine; public for tests and tools).
pub fn solve_assembly(design: &Design, items: &[(FeatureId, ComponentId, AsmItem)],
                      seed: &Placements, opts: &SolveOptions) -> AsmSolution;

/// Joint-aware drag (ASM-04) and drive (ASM-06).
pub struct AsmDrag { /* system of the grabbed occurrence's connected component */ }
#[derive(Copy, Clone, Debug)]
pub enum DragTarget { Point(Point3), Pose(Transform) }
#[derive(Clone, Debug)]
pub struct DragFrame { pub placements: Arc<Placements>, pub stopped: Option<Stop>, pub dof: u32 }
#[derive(Clone, Debug, PartialEq)]
pub enum Stop { Limit(FeatureId, ValueSel), Contact(OccPath, OccPath) }
impl AsmDrag {
    /// `grabbed` is a path from the root; `grab` the picked world point.
    pub fn start(design: &Design, eval: &Evaluated, grabbed: &[OccurrenceId], grab: Point3)
        -> Result<AsmDrag, AsmError>;
    pub fn update(&mut self, target: DragTarget) -> DragFrame;
    /// ASM-06: drive one joint value to `to` (rad or mm).
    pub fn drive(&mut self, joint: FeatureId, value: ValueSel, to: f64) -> DragFrame;
    /// ASM-07: stop at contact.
    pub fn set_contact(&mut self, check: Option<Arc<dyn ContactCheck>>);
    /// Applies rest values and returns the commit: `SetPendingPositions` in History mode,
    /// `SetOccurrenceTransforms` in Direct mode (ASM-13).
    pub fn finish(self) -> Command;
}
/// ASM-13: built-in timeline feature "assembly.capture_position" v1. It lives in `amg-doc`
/// (like `core.base`) because the recompute engine reads it directly; it has no inputs and
/// produces no bodies.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CapturePositionFeature { pub moves: Vec<(OccurrenceId, Transform)>,
                                    pub joint_states: Vec<(FeatureId, JointState)> }
/// ASM-13: the Capture Position command for the current state: solved placements (local) and
/// joint states of every pending occurrence and every out-of-date one. None if there is none.
pub fn capture_command(design: &Design, eval: &Evaluated, name: Option<String>) -> Option<Command>;
/// ASM-13: occurrences whose solved placement differs from `Design::seed_transform` by more
/// than the model tolerance (the "positions out of date" badge).
pub fn out_of_date(design: &Design, eval: &Evaluated) -> Vec<OccurrenceId>;

/// Implemented by `amg-features::assembly::contact` (ASM-07).
pub trait ContactCheck: Send + Sync {
    /// First penetrating pair under `p`, in deterministic order, or None.
    fn penetrating(&self, p: &Placements) -> Option<(OccPath, OccPath)>;
}

// ===== amg-features::assembly (ASM-03, ASM-05, ASM-06, ASM-07, ASM-09) =============
pub use amg_doc::joints::{JointKind, JointAxis, ValueSel};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointOrigin { pub path: OccPath, pub at: OriginAt, pub align: Align,
                         pub z_hint: Option<amg_core::Vec3> }
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum OriginAt {
    Vertex(EntityRef),
    EdgeStart(EntityRef), EdgeMid(EntityRef), EdgeEnd(EntityRef),
    CircleCenter(EntityRef),
    FaceCenter(EntityRef),
    AxisPoint { face: EntityRef, at: AxisPos },
    PointOnFace { face: EntityRef, uv: [f64; 2] },
    Construction(ConstructionId),
    ComponentOrigin,
}
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum AxisPos { Start, Mid, End }
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Align { pub flip: bool, pub angle: Expr, pub offset: [Expr; 3], pub x_axis: Option<EntityRef> }
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LimitSpec { pub min: Option<Expr>, pub max: Option<Expr>, pub rest: Option<Expr> }

/// Frame of an origin in the component space of the leaf of `o.path`.
pub fn origin_frame(ctx: &RecomputeCtx<'_>, o: &JointOrigin) -> Result<Frame, FeatureError>;
/// Same, for tool previews outside recompute.
pub fn origin_frame_in(design: &Design, eval: &Evaluated, o: &JointOrigin) -> Result<Frame, AsmPrepError>;
/// Keypoints offered while hovering a picked entity (dialog snapping).
pub fn keypoints(design: &Design, eval: &Evaluated, path: &[OccurrenceId], e: &EntityRef) -> Vec<(OriginAt, Frame)>;

/// "assembly.joint" v1.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointFeature { pub kind: JointKind, pub motion_axis: JointAxis, pub slide_axis: Option<JointAxis>,
                          pub from: JointOrigin, pub to: JointOrigin, pub limits: Vec<(ValueSel, LimitSpec)> }
/// "assembly.joint_origin" v1 → `ConstructionGeom::Frame`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointOriginFeature { pub origin: JointOrigin }
/// "assembly.as_built_joint" v1 (ASM-05): one origin; the other side's frame is captured.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AsBuiltJointFeature { pub kind: JointKind, pub motion_axis: JointAxis, pub slide_axis: Option<JointAxis>,
                                 pub origin: JointOrigin, pub other: OccPath, pub captured: Frame,
                                 pub limits: Vec<(ValueSel, LimitSpec)> }
/// "assembly.rigid_group" v1 (ASM-05).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RigidGroupFeature { pub members: Vec<OccPath>, pub relative: Vec<Transform> }
/// "assembly.motion_link" v1 (ASM-06).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MotionLinkFeature { pub a: FeatureId, pub a_value: ValueSel, pub b: FeatureId, pub b_value: ValueSel,
                               pub ratio_a: Expr, pub ratio_b: Expr, pub reverse: bool,
                               pub v_a0: f64, pub v_b0: f64 }
/// "assembly.bodies_to_components" v1 (ASM-09). `transform` maps the body from its old
/// component space into `to`'s, captured by the builder (`world(to)⁻¹ · world(old)`), so the
/// body moves with its new component afterwards and occurrence moves recompute nothing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodiesToComponentsFeature { pub moves: Vec<BodyMove> }
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyMove { pub body: BodyId, pub to: ComponentId, pub transform: Transform }
pub const BODIES_TO_COMPONENTS_KIND: &str = "assembly.bodies_to_components";

/// Commands for one "Create Components from Bodies" transaction (ASM-09).
pub fn components_from_bodies(design: &Design, eval: &Evaluated, bodies: &[BodyId])
    -> Result<Vec<Command>, AsmPrepError>;
/// Commands for "move body to component" (ASM-09).
pub fn move_bodies(design: &Design, eval: &Evaluated, bodies: &[BodyId], to: ComponentId)
    -> Result<Vec<Command>, AsmPrepError>;
#[derive(thiserror::Error, Debug, Clone, PartialEq)]
pub enum AsmPrepError {
    #[error("body {0:?} does not exist")] NoBody(BodyId),
    #[error("component {0} does not exist")] NoComponent(ComponentId),
    #[error("target component is placed {0} times; pick one occurrence")] AmbiguousTarget(usize),
    /// ASM-09: the body's own component is placed more than once.
    #[error("body {body:?} is shown {placements} times; its component must be placed once")]
    AmbiguousSource { body: BodyId, placements: usize },
    #[error(transparent)] Resolve(#[from] amg_doc::ResolveError),
    /// ASM-09: the design refused a built command (dry run on a copy).
    #[error(transparent)] Command(#[from] amg_doc::CommandError),
    #[error(transparent)] Asm(#[from] amg_doc::joints::AsmError),   // added with ASM-03/04
}                                                                   // #[non_exhaustive]

pub mod motion {           // ASM-06; ExtRecord kind "assembly.motion_study", v1
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct MotionStudy { pub name: String, pub steps: u32, pub tracks: Vec<Track> }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct Track { pub joint: FeatureId, pub value: ValueSel, pub keys: Vec<(f64 /*step*/, Expr)> }
    pub struct Playback { pub frames: Vec<Arc<amg_doc::Placements>>, pub stops: Vec<(u32, amg_doc::joints::Stop)> }
    pub fn play(jobs: &JobSystem, design: DesignSnapshot, eval: Arc<Evaluated>, study: MotionStudy,
                contact: Option<Arc<dyn ContactCheck>>) -> JobHandle<Playback, AsmError>;
}
pub mod contact {          // ASM-07; ExtRecord kind "assembly.contact", v1
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct ContactConfig { pub mode: ContactMode, pub sets: Vec<ContactSet> }
    #[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
    pub enum ContactMode { Off, Sets, All }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    pub struct ContactSet { pub name: String, pub members: Vec<OccPath> }
    /// Tessellates (BREP-08) and builds BVHs in a job; the result implements `ContactCheck`.
    pub fn prepare(jobs: &JobSystem, design: DesignSnapshot, eval: Arc<Evaluated>, cfg: &ContactConfig)
        -> JobHandle<Arc<MeshContact>, AsmError>;
    pub struct MeshContact { /* per-member meshes + BVHs, pair list */ }
    impl amg_doc::joints::ContactCheck for MeshContact { /* … */ }
}
pub fn link() {}
```

**Needed from other specs (requests, recorded in their Interface changes, 2026-11-08):**
- **S01 (lane E, same lane):** everything in the `amg-doc` blocks above is additive and is
  implemented by ASM-01/03/04/09/13 themselves; recorded in S01 Interface changes. The
  2026-11-09 capture-position changes (ASM-13) are listed under **Interface changes** below and
  still need to be copied into S01.
- **S05 / CON-03:** `resolve_input` on a ref that is not `InContext` returns geometry in the
  referenced body's component space (no occurrence transform applied). Joint origins resolve
  the inner target and apply paths themselves.
- **S07 / MESH-03:** `Bvh::overlapping_pairs_with(&self, other: &Bvh, other_to_self: &Transform,
  visit: &mut dyn FnMut(u32, u32))` (two trees, deterministic order) for contact sets. If S07
  declines, ASM-07 writes it inside `amg-features::assembly::contact` over the public `Bvh`
  nodes.
- **S11:**
  - The generic dialog and tools take `AddFeature::context` from `Document::active_component()`.
  - UI-20 instance transforms come from `Evaluated::instances` / `placements` (solved), not
    authored transforms; it listens to `OccurrenceTransformChanged`.
  - Picking a body instance must say which occurrence path it was: `PickTarget::Body` gains an
    instance index (`PickTarget` is `Copy`, so a `u32` into the scene's path table), and
    `SelItem::entity` wraps sub-entities picked outside the active component in
    `EntityRef::InContext { path, .. }`.
  - A transient placement overlay for drag frames: `SceneAdapter::set_placement_overlay(Option<Arc<Placements>>)`
    so dragged instances move without a recompute.
  - (2026-11-09, ASM-13) A position guard before framework actions: when
    `Design::has_pending_positions()`, the timeline (marker drag, reorder, edit of a node) and
    the generic dialog (when a pick is `EntityRef::InContext`) call a hook registered by
    `amg-ui::assembly` (`fn confirm_positions(&mut UiCtx) -> PositionChoice { Capture, Revert,
    Cancel }`) before acting. Without a registered hook they proceed (and `SetMarker` drops
    pending moves).
  - (2026-11-09, ASM-13) A timeline icon for `assembly.capture_position` nodes through the
    normal feature registration (no new API if registration already takes an icon).
- **S12 / IO-06:** uses `AddOccurrence` for shared prototypes and `BakedBody::component` for
  bodies of imported components. No change on S12's side.

## Work packages
| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| ASM-01 | Components/occurrences in `amg-doc`: commands `AddOccurrence`, `DeleteOccurrences` (cascade), `MoveOccurrence`, `SetOccurrenceGrounded`, `SetOccurrenceName`, `SetOccurrenceTransforms`; rigid/mirror transform validation; `Placements` + `Evaluated::{placements, world_transform, body_to_world, instances}` (authored placements until ASM-04); `Document::active_component`; events | DOC-05, DOC-09, DOC-12 | P0 | Done | 1. Proptest: ≤ 100 random valid tree commands, undo all → `design.json` equals the initial one; redo all → equals the final one. 2. Deleting the last occurrence of a component holding 3 features and one sub-component removes all of them in one undo entry; a root feature that read one of its bodies gets `Error` with an upstream cause. 3. On a 3-level tree with one component placed twice, `body_to_world` equals the hand-multiplied transforms within 1e-12 and `instances` lists exactly 2 entries. 4. A scaled transform is refused with `InvalidValue`; a mirror (det −1) is accepted. 5. `MoveOccurrence { keep_world: true }` leaves the world transform unchanged within 1e-12. 6. Exact `DocEvent` sets per new command and for undo/redo (DOC-09 style). 7. Moving an occurrence recomputes no feature (engine counter). 8. criterion: `world_transform` of a depth-10 path < 1 µs; building `Placements` for 10 000 occurrences < 2 ms. |
| ASM-09 | Create components from bodies, move bodies between components: `BodyChangeKind::Moved`, feature `assembly.bodies_to_components`, `components_from_bodies`/`move_bodies` transaction builders; geometry transformed into the target space (`Shape::transformed`, `IndexedMesh::transform`) | ASM-01, BREP-02 | P0 | Done | 1. Three bodies in the root → one transaction creates 3 components and one feature; every body keeps its `BodyId` and its world bbox within 1e-9 mm; one undo entry. 2. Moving a body into an existing component placed at a non-identity transform keeps its world bbox within 1e-9 mm and volume within 1e-9 relative (solid and mesh). 3. A later root feature that reads the moved body still resolves (same `BodyId`), and the body reports the new component. 4. `.amg` save/load keeps the tree and each body's component. |
| ASM-13 | Pending positions, Capture Position, Revert in `amg-doc` (QUESTIONS #59): `Design::pending_positions` + `PendingPositions`, commands `SetPendingPositions`, `CapturePositions`, `RevertPositions`, layered `SetOccurrenceTransforms`, built-in feature `assembly.capture_position`, `captured_transform_at`/`seed_transform`, running position table in the timeline pass (in-context resolution and DOC-03 keys use captured transforms at the node's index), `SetMarker` drops pending, `joints::{capture_command, out_of_date}`, `DocEvent::PendingPositionsChanged` | ASM-01 | P0 | Done | 1. Move → capture → move → revert: the occurrence ends at the captured transform (1e-12); 4 undo entries; undo of the revert restores the second move; undo of the capture removes the node and restores pending. 2. Pending moves survive save/load (`design.json` equal) and crash recovery (journal replay). 3. A pending move recomputes no feature (engine counter); capturing it re-runs only the in-context features after the node whose occurrence paths include the moved occurrence. 4. Timeline `[F1 in-context on o, C1 capture o→P1, F2 in-context on o]`: F1 sees the base transform, F2 sees P1; marker before C1 → placements use base; suppressing C1 likewise. 5. `SetMarker` with pending moves clears them in the same undo entry. 6. `SetPendingPositions` on a grounded occurrence and in Direct mode is refused; in Direct mode `SetOccurrenceTransforms` writes the base. 7. `SetOccurrenceTransforms` with a capture node listing the occurrence before the marker edits that node, not the base. 8. Deleting an occurrence drops its pending entry; a capture node that lists a deleted occurrence gets `Health::Warning` and ignores it. 9. Exact `DocEvent` sets for the new commands, undo and redo. 10. criterion: `SetPendingPositions` of 100 occurrences < 1 ms; `captured_transform_at` with 50 capture nodes < 2 µs. |
| ASM-10 | Assembly UI module `amg-ui::assembly`: browser activate radio and active styling, grounded icon, context menus (New Component, Activate, Ground/Unground, Create Components from Bodies, Copy/Paste occurrence, Delete, Move to Component), pending marker on moved occurrences, "positions out of date" badge whose action is Capture Position | ASM-01, ASM-09, ASM-13, UI-07, UI-21 | P0 | Todo | 1. Kittest: activate a sub-component, run a generic-dialog feature → its node has that context. 2. Copy/paste an occurrence → 2 render instances share 1 uploaded mesh (upload counter). 3. Ground toggle sends `SetOccurrenceGrounded`, undo restores it. 4. Delete of a component with features is one undo entry. 5. Every menu entry is reachable by label (kittest) and appears only for the right selection. 6. An occurrence with a pending move shows the pending marker; the badge appears when `out_of_date` is non-empty and its action sends one `CapturePositions`. |
| ASM-02 | Ground, move/rotate occurrences: Move/Copy tool for occurrences (triad free move, point to point, numeric fields) ending in one `SetPendingPositions` (Direct mode: `SetOccurrenceTransforms`); Position panel (Capture Position, Revert); Capture / Revert / Cancel prompt registered as the S11 position guard and used by Joint Origin | ASM-10, ASM-13, UI-06 | P0 | Todo | 1. Scripted triad drag of 12.5 mm along X and 90° about Z gives the exact pending transform (1e-12) in one undo entry. 2. A grounded occurrence cannot be moved (tool shows why). 3. Point to point puts a vertex on a vertex within 1e-9 mm. 4. Moving recomputes no feature in the moved component (counter). 5. With 100 occurrences on screen, UI frame time during the drag stays < 16 ms (tracing span). 6. Kittest: after a move the Position panel shows; Capture Position adds one timeline node and hides the panel; Revert puts the occurrence back (1e-12) and hides it. 7. Kittest: dragging the timeline marker or starting Joint Origin with a pending move shows the prompt; Cancel leaves design and pending unchanged, Revert and Capture do what their buttons say. |
| ASM-03 | Joint origins, joint types and limits: `amg-doc::joints` data (`JointKind`, `JointEval`, `AsmItem`, `JointState`, `joint_values`, `snap_placement`), `FeatureOutput::assembly`, `ConstructionGeom::Frame`; `amg-features::assembly` `JointOrigin` + `origin_frame(_in)` + `keypoints` (snaps, x rule, align), features `assembly.joint` and `assembly.joint_origin` | ASM-01, DOC-06, CON-03 | P1 | Todo | 1. On a box-with-hole + pin fixture, every `OriginAt` variant gives the expected frame within 1e-9 (table test, ≥ 14 cases incl. the x-rule fallback and flip/angle/offset). 2. For all 7 joint types, `joint_values` recovers the values of a constructed relative pose within 1e-12 and the note §5.2 rows are zero on it. 3. `snap_placement` puts the pin in the hole: axes coincide within 1e-9 mm. 4. Editing the hole position upstream moves the joint frame with it (DOC-06 naming); deleting the hole face gives the joint an `Error` naming the ref. 5. Moving an occurrence does not re-run joint features (counter). 6. Joint offsets are model parameters: editing one changes the frame. |
| ASM-04 | Joint solver (3D), assembly stage of recompute and drag: residuals per note §5.2 with analytic Jacobians, LM with SE(3) retraction on `amg_sketch::solve::system`, decomposition (contexts, rigid clusters, connected components), limits active set, DOF, `AsmDiagnostic` → health, `Design::joint_states` (layered per ASM-13), seed from `Design::seed_transform`, `AsmDrag`, `test.joint` feature, golden mechanisms `tests/golden/asm/` | ASM-03, ASM-13, SK-03 | P1 | Todo | 1. 4-bar linkage drags correctly: scripted crank drag through 360° in 1° steps keeps the rocker within 1e-9 rad of the closed-form solution at every step with no branch flip; drag frame p95 < 2 ms. 2. Golden mechanisms 1–6 of note §10 pass. 3. Every residual's Jacobian matches finite differences (relative error < 1e-6) on 100 random poses. 4. 100 runs give bit-identical placements. 5. Budgets of note §9 (assembly stage rows) met in criterion. 6. An over-constrained loop reports `Overconstrained` on the later joint and leaves the earlier result in place. 7. With a pending move on a jointed occurrence, editing a joint offset re-solves from the pending seed (same branch) and keeps the move pending; Revert re-solves from the captured seed. |
| ASM-11 | Joint tools UI: Joint dialog (type, two origin picks with keypoint hover snapping, align fields, limits tab, animate preview), Joint Origin dialog, edit joint, joint glyph overlay (RND-07) | ASM-03, ASM-02, UI-05, UI-06, RND-07 | P1 | Todo | 1. Kittest: pick the pin's circular edge, then the hole's → preview snaps; OK gives `AddFeature` + `SetOccurrenceTransforms` in one undo entry and leaves nothing pending. 2. Hover keypoints update within one frame. 3. Each joint shows one glyph at its origin with the type icon (overlay count test). 4. Editing a joint's type from revolute to slider keeps its origins. |
| ASM-12 | Joint-aware drag in the viewport: drag tool on `AsmDrag`, rest value on release, limit/stop feedback, DOF arrows on glyphs, scene placement overlay during drag; release gives a pending move | ASM-04, ASM-11 | P1 | Todo | 1. Kittest on the 4-bar: dragging the coupler moves crank and rocker consistently (positions vs ASM-04 golden within 1e-9) and commits one `SetPendingPositions` undo entry holding every occurrence that moved and the joint states; Capture Position then stores the same values (1e-12); Revert restores the pre-drag placements. 2. A revolute with limits 0..90° stops at 90° and the status bar says why. 3. A joint with `rest` returns to it on release. 4. No recompute runs during the drag (counter); UI frame < 16 ms. |
| ASM-05 | As-built joints, rigid groups: features `assembly.as_built_joint` (captured frame) and `assembly.rigid_group`, rigid-cluster substitution, dialogs | ASM-04, ASM-11 | P1 | Todo | 1. An as-built revolute between two occurrences changes no placement on creation (1e-12), then a drag rotates about the picked axis only. 2. A rigid group of 3 occurrences moves as one (relative transforms unchanged within 1e-12); `AsmStats::clusters` shows the substitution. 3. A rigid group that contradicts a rigid joint reports `Overconstrained`. |
| ASM-06 | Motion links, drive joints, motion study: `assembly.motion_link`, link residual, Drive Joints tool, `motion::MotionStudy` ExtRecord, playback job, Motion Study panel with "use this frame" (pending move) | ASM-04, ASM-12 | P2 | Todo | 1. Gear pair linked 2:1 driven through 5 turns: phase error < 1e-9 rad. 2. Rack and pinion (revolute ↔ slider, 360° ↔ 2π·10 mm) tracks within 1e-9 mm. 3. Drive Joints sets 45° exactly and commits one `SetPendingPositions` undo entry; a motion-study "use this frame" does the same for that frame. 4. 4-bar motion study, 360 steps: playback job < 200 ms; frames equal the ASM-04 golden values. 5. A study survives save/load and undo. |
| ASM-07 | Contact sets: `contact::ContactConfig` ExtRecord (Off/Sets/All), `prepare` job (BREP-08 tessellation, BVHs), `MeshContact: ContactCheck`, bisection stop in `AsmDrag` and playback, Contact Sets UI | ASM-04, ASM-12, MESH-03, BREP-08 | P2 | Todo | 1. A pin on a slider joint driven into a blind hole stops at the bottom within the tessellation deviation + 1e-3 mm. 2. With mode Off nothing stops. 3. Touching faces (within tolerance) do not count as contact. 4. Drag frame with one contact pair of 2 × 100 k triangles: p95 < 8 ms. 5. `All` on 20 occurrences tests only pairs whose boxes overlap (counter) and stops correctly. |
| ASM-08 | Linked external designs | DOC-07 | P3 | Todo | Refine before Phase 6: reference by document UUID + path hint, update/break-link semantics, naming through `BodyNaming::for_import`, interaction with SOL-16 (derive). |

Priorities follow the parity matrix: component tree and moving P0, joints P1, motion and
contact P2, linked designs P3. Phases (roadmap): ASM-01, ASM-13, ASM-09, ASM-10, ASM-02 in Phase 2;
ASM-03, ASM-04, ASM-11, ASM-12, ASM-05 in Phase 3; ASM-06, ASM-07 in Phase 5; ASM-08 in Phase 6.

## Testing
- Unit and property tests in `amg-doc` for every command, the cascade and the placements
  (ASM-01), the residuals (FD Jacobian checks) and the solver (ASM-04).
- Golden mechanisms in `tests/golden/asm/` (note §10), driven by the lane-E `test.joint`
  feature so they run without OCCT; values compared with closed-form solutions.
- Origin-frame tests on B-rep fixtures in `amg-features::assembly` (ASM-03), gated like other
  OCCT tests.
- Kittest UI tests for ASM-02, ASM-10, ASM-11, ASM-12.
- Capture/revert/undo sequences as proptests in `amg-doc` (ASM-13): random moves, captures,
  reverts, marker moves and undo/redo keep `design.json` round-trips exact.
- criterion benches for the budgets below.

## Performance targets
From note §9, inside the vision budgets (recompute < 500 ms for a typical part; UI never
blocks > 16 ms):

| Case | Budget |
|---|---|
| Assembly stage, 50 occurrences, 60 joints incl. one loop | < 5 ms median |
| Assembly stage, 500 occurrences, mostly grounded/rigid | < 20 ms median |
| Assembly stage, unchanged key | < 0.2 ms |
| Drag frame, 4-bar or 50-occurrence mechanism | p95 < 2 ms |
| Drag frame with one contact pair (2 × 100 k tris) | p95 < 8 ms |
| Motion study, 4-bar, 360 steps | < 200 ms |
| `world_transform`, depth-10 path | < 1 µs |

## Open questions
1. ~~(from S05, S12, 2026-10-20) INS-04, INS-10 and IO-06 need occurrence transforms
   (`Evaluated::body_to_world(BodyId, occurrence path)`, requested from S01) and component
   creation from STEP `ForeignModel` assemblies (S12 maps `ForeignNode`s to components).~~
   **Resolved 2026-11-08:** ASM-01 provides `Evaluated::{body_to_world, instances,
   world_transform}` over solved placements and `Command::AddOccurrence` for shared
   prototypes. S12 needs no other change.
2. ~~**(owner, QUESTIONS #59)** Pending positions + Capture Position (Fusion) or every move
   committed at once with undo as revert?~~ **Resolved 2026-11-09 (owner):** Fusion-style
   pending moves until Capture Position (a timeline node) or Revert (§2, note §3, ASM-13).
3. ~~**(owner, QUESTIONS #60)** Lane E owns a new `amg-ui::assembly` module?~~ **Resolved
   2026-11-09 (owner):** yes; roadmap Lanes table updated.
4. Joints as timeline features or as an authored joint table? **Resolved 2026-11-08:** timeline
   features (§3, note §4); `Component::joints` stays empty until the next format bump.
5. Reuse `NativeSolver` for joints? **Resolved 2026-11-08:** reuse the `system` layer and
   `numeric_rank` only; own LM with SE(3) retraction (§5, note §6.1).
6. Copies of a subassembly in different joint positions? **Resolved 2026-11-08:** not
   supported; a component is solved once, so all copies move alike (note §6.2). Revisit with
   "make independent" if the owner needs it.

## Refinement log
### 2026-11-08 - S06-refine (lane E refinement)
- **Read:** AGENTS.md, vision, architecture, ADRs, roadmap, QUESTIONS; S01 (Interfaces, Interface
  changes, DOC-01/05 logs), S03 (`solve::system`, SK-12), S04 (SOL-24, solid-features note
  §2.3), S05 (CON-03 `ResolvedRef`, INS-04/INS-10), S07 (MESH-03), S11 (registration API,
  `PickTarget`, browser), S12 (IO-06, `ImportBrepFeature`), S19 (LAT-03). Code: `amg-doc`
  (`model.rs`, `command.rs`, `feature.rs`, `recompute.rs` cache keys), `amg-features::{mesh,
  sketch}` registration, `amg_sketch::solve::{system, rank, native}`, `amg_mesh::spatial::pairs`,
  `amg_core::Transform` (= `DAffine3`).
- **Prior art** (note §0): Autodesk Fusion help on joint types (seven types; note that the help
  page's "Motion allowed" lines for planar and ball contradict its own text, we follow the
  standard DOF counts), Capture/Revert, motion links, motion studies and contact sets; Onshape
  mates and mate connectors; FreeCAD 1.0 Assembly workbench with OndselSolver (LGPL-2.1 per the
  Gentoo ebuild; C++); FreeCAD Assembly3 (SolveSpace `libslvs`), Assembly4 (attachment trees,
  no solver); our S03 solver. No dependency is added: the solver is ours on the S03 system layer.
- **Findings from the code:**
  - DOC-03's key hashes the feature context id but no occurrence transform, and `InContext`
    paths hash authored transforms. So geometry already lives in component space; S06 keeps
    that and puts solved positions in evaluated state.
  - `Transform` is a general affine; occurrence transforms get a rigid/mirror check (ASM-01).
  - `Component::joints` is `deny_unknown_fields`-serialized, so it stays (empty).
  - `NativeSolver`'s generic pipeline (`Problem`) is crate-private; `EquationSystem`,
    `Residual` and `numeric_rank` are public.
  - `amg_mesh` has only a self-overlap BVH query; contact needs a two-tree query (S07 request).
- **Decisions:** no pending positions (owner confirms, #59; superseded 2026-11-09); joints as timeline features with
  `FeatureOutput::assembly`; joint frames from CON-03 in component space; maximal-coordinate
  solver with SE(3) increments; per-context bottom-up solve; rigid substitution; limits by
  active set; joint states for unwrapped angles; motion studies and contact sets as
  `ExtRecord`s.
- **WPs:** ASM-01..08 keep their meaning and IDs (ASM-01 and ASM-02 are referenced by S04, S05
  and S12; ASM-06 by S19). "New component from bodies" moved from ASM-01 to new **ASM-09**
  (other specs use ASM-01 only for the component tree). New: **ASM-10** (browser/assembly UI
  module), **ASM-11** (joint dialogs, glyphs), **ASM-12** (joint-aware viewport drag). ASM-02
  now depends on ASM-10 and UI-06; ASM-03 adds CON-03; ASM-05 adds ASM-11; ASM-06 adds ASM-12;
  ASM-07 adds ASM-12 and BREP-08. Every WP has a measurable Done-when.
- **Questions:** #59 (capture model), #60 (`amg-ui::assembly` lane). Open question 1 resolved.
- **Roadmap:** Phase 2 lane E adds ASM-09, ASM-10; Phase 3 lane E adds ASM-11, ASM-12. Lane
  table unchanged pending #60.
- **Interface requests** recorded in S01, S05, S07 and S11 Interface changes (2026-11-08).

### 2026-11-09 - owner answers #59, #60
- **#59 (b), Fusion-style pending moves.** Replaces the 2026-11-08 "no pending positions"
  decision. Fusion's help ("Capture or revert position of components"): the Position panel
  appears after a move; Capture Position keeps the positions, Revert returns components to their
  most recently captured position with no dialog; position-dependent commands (Joint Origin)
  prompt "Capture Position / Continue". Autodesk's blog and forum add that a capture is a
  timeline entry, that editing it affects what comes after, and that uncaptured positions are
  lost when the marker is rolled. The help does not say whether uncaptured positions are saved;
  we save them (they are authored state, and the crash journal needs them).
- **Fit with ADR-014 / S01:** pending moves are authored (`Design::pending_positions`, undoable
  snapshots, saved); capture nodes are an `amg-doc` built-in feature read by the timeline pass;
  features see captured positions at their own index and never pending ones, so moves stay
  cheap and recompute stays acyclic. Solved placements remain evaluated state. Joint creation
  writes the effective layer directly, so joints never leave anything pending. "Update
  positions" is gone; the out-of-date badge's action is Capture Position.
- **WPs:** new **ASM-13** (amg-doc layer, P0, Phase 2). ASM-02 now depends on ASM-13 and adds
  the Position panel and prompt; ASM-10 depends on ASM-13 (pending marker, badge → capture);
  ASM-04 depends on ASM-13 (layered joint states, seed) and gains test 7; ASM-11 test 1, ASM-12
  and ASM-06 Done-when now commit pending moves. IDs unchanged.
- **#60 yes:** `amg-ui::assembly` belongs to lane E; roadmap Lanes table updated (with
  `amg-ui::script`, #56).
- **Interface changes:** S01 (below) and S11 (position guard, under "Needed from other specs").
- Open questions 2 and 3 resolved. Note §3 rewritten, §1, §2, §5.5, §6.3, §6.4, §7, §8.2 updated.

### 2026-11-10 - ASM-01 (components and occurrences in `amg-doc`, lane E)
- **Landed** (`crates/amg-doc/src/assembly.rs`, `command.rs`, `eval.rs`, `event.rs`, `document.rs`,
  `recompute.rs`, `model.rs`): commands `AddOccurrence`, `DeleteOccurrences` (cascade through
  `Design::remove_occurrences`: orphaned components and their inner occurrences, features whose
  context was deleted with model-param conversion, baked bodies and `BaseFeature` entries, all in
  one command/undo entry), `MoveOccurrence` (cycle check; `keep_world` needs old and new parent each
  placed exactly once, else `InvalidValue { field: "keep_world" }`), `SetOccurrenceGrounded`,
  `SetOccurrenceName`, `SetOccurrenceTransforms`. Occurrence transforms are validated rigid or mirrored
  (orthonormal within 1e-9, |det| = 1 within 1e-9) on every path including `validate()` on load.
  `Placements` (id-sorted vectors, binary search) is built at the end of every recompute and stored in
  `Evaluated`; `Evaluated::{placements, world_transform, body_to_world, instances}`;
  `Design::{is_valid_path, features_in, authored_world}`; `Document::{active_component,
  set_active_component}` (falls back to the root when the component is deleted);
  `DocEvent::ActiveComponentChanged`; `OccurrenceTransformChanged` is also emitted when a swapped-in
  evaluated state has a different placement (deduplicated against events already queued).
- **Deviations / leftovers:** `SetOccurrenceTransforms` has only `moves` for now; the `joint_states`
  field comes with ASM-04 (needs `JointState`, serde-additive). `Evaluated::asm_diagnostics` and
  `AsmDiagnostic` are ASM-04's. `SetOccurrenceTransforms` writes the base layer (layers are ASM-13).
  A recompute after a placement change emits `OccurrenceTransformChanged` again after the authored one
  if the authored event was drained first (consumers reading `placements` need that one).
- **Tests:** `tests/assembly.rs` (proptest of <= 100 tree commands undo/redo, cascade, world/instances,
  rigid validation, keep_world, exact events, active component, no-recompute counter);
  bench `benches/assembly.rs`.

### 2026-11-10 - ASM-13 (pending positions, Capture Position, Revert in `amg-doc`, lane E)
- **Landed** (`crates/amg-doc/src/positions.rs`, `joints.rs`, plus `model.rs`, `command.rs`, `event.rs`,
  `recompute.rs`, `feature.rs`, `assembly.rs`, `timeline.rs`, `direct.rs`): `Design::pending_positions`
  (`PendingPositions`, omitted from `design.json` while empty, so existing goldens are byte-identical and
  `FORMAT_VERSION` stays), commands `SetPendingPositions`, `CapturePositions`, `RevertPositions`
  (one undo entry each), built-in feature `assembly.capture_position` v1 (`CapturePositionFeature`,
  sorted unique entries, warns and ignores entries of deleted occurrences/joints), `Design::{captured_transform_at,
  seed_transform, joint_state, capture_table}`, `joints::{capture_command, out_of_date, JointState, ValueSel}`,
  `DocEvent::PendingPositionsChanged`. The timeline pass keeps a running `PositionTable`; cache-key
  occurrence bits and `RecomputeCtx::{occurrence_transform, path_transform}` use it (captured, never
  pending); `Placements` are seeded from pending over captured over base. `SetOccurrenceTransforms`
  writes the layer effective at the marker (edits the latest applying capture node that lists the
  occurrence, else the base) and drops those pending entries. `SetMarker` clears pending in the same
  undo entry when the marker moves.
- **Deviations / leftovers:**
  - `SetOccurrenceTransforms` keeps only `moves` (no `joint_states`): adding the field would break the
    unchanged ASM-01 tests; ASM-04 adds it together with the base layer `Design::joint_states`.
    `Design::joint_state` therefore returns pending, else the latest capture node (no base layer yet).
  - `joints.rs` holds only `ValueSel`, `JointState` and the two helpers; ASM-04 grows it. `ValueSel`
    / `JointState` are the spec's shapes. `PendingPositions` fields are crate-private with accessors
    (`moves()`, `get`, `joint_states()`, `joint_state`, `is_empty`) so `imbl` stays out of the public API.
  - Capture-node cache keys also hash which listed occurrences/joints no longer exist (otherwise a cached
    healthy output would hide the warning).
  - Entering direct mode folds the seeds into the base transforms first (positions stay as seen) and
    clears pending; `CapturePositions`/`SetPendingPositions` in direct mode are `CommandError::Invalid`.
  - `out_of_date` compares seed and solved placement; it is always empty until ASM-04 solves joints.
    The assembly-stage key of note §7 does not exist yet; pending already invalidates placements through
    the design revision.
  - Moving the marker onto itself keeps pending (only an actual move clears it).
- **Tests:** `tests/positions.rs` (15 tests: Done-when 1-9 each, journal replay via a killed child
  process, save/load, joint-state layers, a proptest of random move/capture/revert/marker/undo/redo/place
  sequences that checks `design.json` round-trip and placements = seeds). Full `cargo nextest run -p amg-doc`:
  224 passed. Bench `benches/positions.rs` (run with `--profile dev`, opt-level 1, `AMG_STRICT_TIMING=1`):
  `SetPendingPositions` of 100 occurrences 19.4 µs (budget 1 ms); `captured_transform_at` with 50 capture
  nodes, worst case 511 ns (budget 2 µs).

### 2026-11-11 - ASM-09 (create components from bodies, move bodies between components, lane E)
- **Landed:**
  - `amg-doc`: `BodyChangeKind::Moved { component, geom, naming }` (engine: refused with a node
    `Error` if the body or component is missing; otherwise the body keeps its id, gets the new
    component and the feature becomes its producer); cache-file wire variant `Moved` (appended;
    `CACHE_FORMAT`, `FORMAT_VERSION` and `design.json` unchanged); `stub::Shape` can carry a real
    `amg_brep::Shape` (`from_brep`, `brep`, `transformed`). Recorded in S01 Interface changes.
  - `amg-brep`: `Shape::volume()` over the existing BREP-01 shim function (S02 Interface changes).
    `Shape::transformed` (BREP-02) and `IndexedMesh::transform` (MESH) already existed.
  - `amg-features::assembly` (new module): feature `assembly.bodies_to_components` v1
    (`BodiesToComponentsFeature { moves: Vec<BodyMove { body, to, transform }> }`), builders
    `components_from_bodies` / `move_bodies`, `AsmPrepError`, `link()`. New cargo feature
    `occt = ["amg-brep/occt"]` on `amg-features` for the solid tests.
- **Deviations:**
  - Each move stores the old-space → new-space transform (`world(to)⁻¹ · world(old)`, from
    `Evaluated` placements at build time) instead of `(BodyId, ComponentId)` only. Computing it at
    recompute time from captured positions would make a later occurrence move re-run the node and
    leave the body behind in world space; stored, the body moves with its new component and moving
    an occurrence recomputes nothing (tested).
  - "Create Components from Bodies" places each new component once, at the identity, in the
    component the body is in now (Fusion), so the transform is the identity and the geometry
    allocation is shared (no geometry-change event, only `ComponentTreeChanged`). It works even when
    that component is placed several times (the body keeps all instances).
  - New ids are predicted by a dry run of the commands on a copy of the design (`apply_command`;
    id allocation is deterministic), so the builders return a plain `Vec<Command>` for one
    transaction, as the Interfaces say. The caller must apply them to the same design state.
  - `AsmPrepError` gains `AmbiguousSource { body, placements }` (the body's own component placed
    more than once) and `Command(CommandError)`; it is `#[non_exhaustive]`; the `Asm(AsmError)`
    variant waits for ASM-03/04 (`AsmError` doesn't exist yet).
  - The node lives in the root context and is inserted at the marker.
- **Limits / leftovers:**
  - A B-rep solid cannot move across a mirrored placement (`amg_brep::Shape::transformed` refuses
    reflections): the node gets `Error` and the body stays put (tested). Meshes mirror fine
    (winding flipped, volume kept). Implicit and form bodies only move between components placed
    alike (identity transform).
  - Fingerprints inside the carried-over `BodyNaming` stay in the old component's space (names are
    unchanged, the topology is the same). DOC-11/CON-03 should transform or re-derive them if
    fingerprint matching across a move turns out to matter.
  - Deleting the target component later makes the node fail and the body returns to its old
    component (the cascade does not delete moved-in bodies, unlike Fusion).
  - Direct mode is not supported (the builders return `Command(WrongMode)` from `AddFeature`).
  - Real B-rep solids are still written as the `stub:solid` tag by the stub codecs, so a solid's
    geometry does not survive the cache file (not new; TODO(BREP-02) stub swap).
- **Tests** (`crates/amg-features/tests/assembly_bodies.rs`, 11 mesh tests + 2 solid tests behind
  `occt`): Done-when 1 (3 root bodies → 4 commands, 3 components + 1 node, same `BodyId`s, world
  bbox ≤ 1e-9 mm, volume ≤ 1e-9 rel, one undo entry restoring `design.json` exactly, redo);
  Done-when 2 (mesh into a two-level off-axis placement and into a mirrored one; solids
  `box`, `plate_with_holes`, `filleted_box` fixtures into a two-level off-axis placement: measured
  bbox drift ≤ 1.8e-15 mm, volume relative error ≤ 1.5e-16); Done-when 3 (a later root reader
  feature is `Ok` on the same `BodyId`, body reports the target, instance path has 2 occurrences);
  Done-when 4 (save/load with and without the persisted cache: `design.json` identical, every
  body's component identical, the cached run recomputes nothing). Plus: moving the target
  occurrence moves the body and recomputes no feature; builder errors; invalid feature data;
  deleted target. `cargo nextest run -p amg-doc -p amg-features -p amg-brep`: 314 passed,
  2 skipped; `-p amg-features --features occt --test assembly_bodies`: 13 passed.

## Interface changes
Changes this spec needs in S01 (lane E; the ASM-13 ones are copied to S01, 2026-11-10; to be copied into S01's Interface changes by the
owning agent; all additive except where marked):
- **2026-11-09, ASM-13 (QUESTIONS #59):**
  - `Design` gains `pending_positions: PendingPositions` (`#[serde(default)]`, saved; no
    `FORMAT_VERSION` bump) and the accessors `pending_positions`, `has_pending_positions`,
    `captured_transform_at`, `seed_transform`.
  - `Command` gains `SetPendingPositions`, `CapturePositions`, `RevertPositions`.
    `SetOccurrenceTransforms` (S06, not yet in S01) writes the layer effective at the marker.
  - **Behaviour change:** `SetMarker` clears pending positions in the same undo entry.
  - **Behaviour change (DOC-03):** "occurrence transform bits" in the key and the transforms
    used to resolve `EntityRef::InContext` / `Dependency::Occurrence` become
    `captured_transform_at(o, index)` of the node being computed, read from a position table
    the timeline pass updates at each computed (not suppressed) `assembly.capture_position`
    node, instead of `Occurrence::transform`. Without capture nodes the hashed bits are the
    same, so existing caches stay valid. Pending moves are never hashed into feature keys.
  - Built-in feature kind `assembly.capture_position` v1 registered by `amg-doc` (like
    `core.base`), so files with captures load without `amg-features`.
  - `DocEvent::PendingPositionsChanged`.
  - The assembly stage key (S06 note §7) includes the pending layer.
- **2026-11-11, ASM-09** (all additive, recorded in S01 and S02 Interface changes the same day):
  `BodyChangeKind::Moved { component, geom, naming }` and its cache-file wire variant;
  `stub::Shape::{from_brep, brep, transformed}`; `amg_brep::Shape::volume()`.
Requests this spec makes of other specs are listed under Interfaces → "Needed from other specs".
