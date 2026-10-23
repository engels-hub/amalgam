# S03 — Sketching & constraint solver

| | |
|---|---|
| **Status** | Ready |
| **Priority** | P0 |
| **Crates** | `amg-sketch`, `amg-features::sketch` (sketch-mode UX in `amg-ui` is lane D's UI-09) |
| **Lane** | B |
| **Depends on** | S00 (FND-03 core types); S01 for `Feature`, `Expr`, `EntityRef`, commands (SK-19, SK-09); S02 for projection/intersection queries (SK-09) |
| **Parity** | Sketch section of parity matrix |
| **Design notes** | [`docs/notes/sketch-solver.md`](../notes/sketch-solver.md) (solver design, ADR-007 options, SK-13 benchmark plan) |

## Summary
2D parametric sketches on planes and planar faces. This covers entities, geometric
constraints, dimensions, a robust and fast constraint solver with DOF feedback, and
profile/region detection that produces faces with holes and stable ids for
extrude/revolve.

## Scope
**In:**
- Entity model and parameter vector.
- Constraints and dimensions (driving/driven).
- Solver: solve, drag, DOF status, conflict diagnosis.
- Profile/region detection with stable profile ids.
- Geometry queries and inference hooks for the UI.
- Composite builders (rectangles, slots, polygons).
- Sketch edit ops (trim, extend, break, offset, fillet, chamfer, mirror, patterns, move/scale).
- Ellipses, splines, conics.
- Projection/include/intersect of 3D geometry.
- Text.
- DXF/SVG import.
- The timeline feature `amg-features::sketch` (Expr-driven dimensions, edit commands).
- 3D sketch (P2).

**Out:**
- UI tool interaction, rendering of glyphs, inference *rules* (S11 UI-09 uses SK-20 hooks).
- DXF/SVG export (S12 IO-08).
- Turning profiles into B-rep faces (S02/S04; `amg-brep` builds faces from the 2D loops given
  here).
- Mesh section sketch (S07 MESH-14, built on SK-08).
- Assembly joint solving (S06 ASM-04 may reuse `amg_sketch::solve::system`).

## Design

### Layering
- `amg-sketch` is a domain crate. It depends only on `amg-core` (and optionally `amg-jobs`
  for cancellation), plus `nalgebra`, `faer`, `serde` and `smallvec`.
- It knows nothing about `amg-doc`. Dimension values are plain `f64` (mm / rad) inside
  `amg-sketch`. Their **source of truth is an S01 `Expr`** stored by the timeline feature
  `amg-features::sketch::SketchFeature`, which evaluates the expressions in the recompute
  context and writes the values before solving.
- Projected geometry is identified inside `amg-sketch` only by an opaque `ExternalLinkId`.
  The `EntityRef` behind it lives in `SketchFeature`.

### Entities and parameter vector
- All solver unknowns are `f64` entries of one `ParamStore` vector. Lengths are in mm in
  sketch-plane coordinates, angles and curve parameters in radians.
- Entities reference parameters (`ParamId`) or other entities (points). Parameters are
  allocated when an entity is created and never move. Deleted entities free their slots
  lazily, and `Sketch::compact()` (called on save) renumbers.

| Entity | Own params | Sub-entities | Internal equations | DOF |
|---|---|---|---|---|
| Point | `x, y` | — | — | 2 |
| Line | — | start, end points | — | 4 |
| Circle | `r` | centre point | — | 3 |
| Arc (CCW start→end) | — | centre, start, end points | `|s−c| − |e−c| = 0` | 5 |
| Ellipse (SK-22) | `b` (minor radius) | centre point, major-axis end point | — | 5 |
| Elliptical arc (SK-22) | `b, t0, t1` | centre, major-axis end, start, end points | start/end lie at `t0`/`t1` (4 eqs) | 7 |
| B-spline, control-point form (SK-08) | weights (if rational) | pole points | — | 2·poles (+ weights) |
| B-spline, fit-point form (SK-08) | — | fit points, optional tangent handles | poles are a linear map of fit points (not unknowns) | 2·fit points (+ handles) |
| Conic (SK-08) | `rho` | start, end, apex points | — | 7 |

- Arcs use SolveSpace's centre/start/end form, not PlaneGCS's angle form. Endpoints are then
  real points, so coincident/tangent constraints are uniform and there is no angle
  wrap-around in the unknowns.
- Entity flags: `construction` (excluded from profiles) and `origin` (`User`,
  `Projected(link)` with all params locked, `Imported`).
- Text (SK-10) is a *text block*: an anchor point plus a direction point (2 points, so the
  block moves and rotates rigidly), a height and a string. The block generates locked spline
  outlines.
- **Ids are stable and never reused.** `EntityId(u32)` and `ConstraintId(u32)` are allocated
  monotonically per sketch and serialized. S01's `TopoName` examples
  (`from SketchCurve(S3, Line#12)`) and profile ids rely on them. Collections are
  `BTreeMap` for deterministic iteration.
- **Sketch plane.** It is `amg_core::Plane` (S00 FND-03: right-handed frame with
  `to_local`/`to_world`/`to_transform`). Sketch x/y are the plane's `x`/`y`. The plane is
  resolved by `SketchFeature` (origin plane, construction plane or planar face) at every
  recompute.

### Constraints and dimensions
Each constraint expands to one or more residual equations with hand-written analytic
Jacobians. Signed forms, with the orientation captured at creation, preserve chirality
(arcs don't flip).

| Constraint | Applies to | Rows | Residual (sketch; `d = b − a` of a line) | Substitutable |
|---|---|---|---|---|
| Coincident | point–point | 2 | `p − q` | yes |
| PointOnCurve | point–line / circle / arc / ellipse / spline | 1 (+aux) | line: `cross(d, p−a)/|d|`; circle: `|p−c| − r`; ellipse/spline: `p − C(t)` with aux param `t` (2 rows) | — |
| Midpoint | point–line | 2 | `2p − a − b` | — |
| Horizontal / Vertical | line or point pair | 1 | `a_y − b_y` / `a_x − b_x` | yes |
| Parallel / Perpendicular | line–line | 1 | `cross(d1,d2)/(|d1||d2|)` / `dot(d1,d2)/(|d1||d2|)` | — |
| Collinear | line–line | 2 | both points of line 2 on line 1 | — |
| Tangent | line–circle/arc, circle/arc–circle/arc, curve–curve at shared endpoint | 1 | line: signed `dist(c, line) − r`; circles: `|c1−c2| − (r1 ± r2)` (`TangentKind`); endpoint: `cross(t1, t2)` with a stored direction sign | — |
| Curvature (G2) (SK-08) | curve–curve at shared endpoint | 2 | tangent + equal signed curvature | — |
| Concentric | circle/arc/ellipse pair | 2 | centres coincident | yes |
| Equal | line–line (length), circle/arc (radius), line–arc (length) | 1 | `|d1| − |d2|`, `r1 − r2` | radius: yes |
| Symmetric | point pair / like curves about a line | 2 | midpoint on axis + `dot(q−p, axis_dir)` | — |
| Fix | any entity | — | locks params (removed from unknowns) | yes |
| **Dimensions** (`value`, driving/driven) | | | | |
| Distance | point–point, point–line, line–line (parallel), point/line–circle edge | 1 | `|p−q| − v` / signed perpendicular distance − `v` | — |
| Horizontal / Vertical distance | point pair | 1 | `side · (q_x − p_x) − v` | — |
| Length | line | 1 | `|d| − v` | — |
| Angle | line–line | 1 | `atan2(cross, dot)` of oriented directions − `v`, wrapped to (−π, π] | — |
| Radius / Diameter | circle, arc | 1 | `r − v` / `2r − v` | — |
| ArcLength | arc | 1 | `r · sweep − v` | — |
| Ellipse major/minor (SK-22) | ellipse | 1 | `|m − c| − v` / `b − v` | — |

- **Driven** dimensions add no equations. They are measured after the solve and reported in
  `SolveReport::driven`.
- **Angular residuals** are scaled by the characteristic length `L_c` (bbox diagonal, ≥ 1 mm)
  so one tolerance applies to all rows.
- **Auxiliary params** (curve parameter `t` for point-on-ellipse/spline) are owned by the
  constraint and stored in `Constraint::aux`.

### Solver (ADR-007; full design in the note)
Pipeline: build system → substitution (union-find aliasing) → connected components →
Dulmage–Mendelsohn blocks → per-block damped weighted minimum-norm Gauss–Newton/LM (dense
`nalgebra` ≤ 64 unknowns, sparse `faer` above) → optional DOF analysis and diagnosis.
Details:

- **Caching.** The decomposition and sparsity pattern are cached per
  `Sketch::structure_rev`, so value-only edits and drags skip the symbolic work.
- **DOF analysis.** Rank-revealing column-pivoted QR / SVD of `J` at the solution gives
  `DOF = n − rank`. Null-space rows give per-entity `Under` / `Full`. A dependency group puts
  its members to `Over`.
- **Diagnosis.** Dependent rows from QR of `Jᵀ` are mapped to constraint groups and
  classified as redundant (consistent), conflicting, or `DegenerateConfiguration`
  (witness-perturbation tie-break). Large groups are refined SolveSpace-style by
  leave-one-out rank tests.
- **Drag.** Dragged params are set to the cursor target and given a large movement cost in the
  weighted min-norm step. The solve warm-starts from the previous frame, and the cache is kept
  for the session. PlaneGCS-style SQP is benchmarked as an alternative in SK-13.
- **Backends.** `SolverBackend` trait. `NativeSolver` is always built. `SlvsSolver` exists only
  with cargo feature `slvs` (GPL-3, dev/test oracle per ADR-012; see the note §4).
- **Gate.** SK-03 (production solver) is gated by the ADR-007 decision. SK-13 produces the
  data for that decision. SK-01, SK-02, SK-06, SK-14 and SK-21 do not depend on the decision:
  they only use the data model and residual definitions, which both options share.
- **Threading.** All solver entry points are synchronous, `Send`, and take a cancellation
  callback. UI-09 runs solves and drag steps on an `amg-jobs` worker with latest-wins
  coalescing of cursor events. Nothing here runs on the UI thread by contract.

### Profiles (regions)
1. **Input:** non-construction curves (user + projected), as trimmed `Curve2`.
2. **Intersections:** broad phase with AABB sweep-and-prune, narrow phase analytic for
   line/arc/circle (SK-06), plus ellipse/spline/conic intersectors (SK-22, SK-08: Bézier
   clipping + Newton refinement).
3. **Split and weld:** split curves at intersection params. Weld vertices within
   `weld_tol` (default 1e-6 mm) with a grid hash. Merge collinear/co-circular overlapping
   spans into one edge carrying all source ids.
4. **Graph:** build a half-edge graph. Around each vertex, sort outgoing half-edges by tangent
   angle, breaking ties by signed curvature (for tangent arcs). `next` = the clockwise-adjacent
   twin. Prune dangling edges iteratively and report them as `open_chains`.
5. **Faces:** each face cycle's signed area comes from the exact Green integral for lines and
   arcs (Gauss–Legendre for ellipse/spline). CCW bounded cycles are outer loops, CW cycles are
   holes. Holes go to the smallest enclosing outer loop (containment tree). Every bounded face
   of the arrangement is one `Profile`. An island inside a hole is its own profile, as in
   Fusion.
6. **Issues** (feeds "sketch doctor"): gaps between endpoints in `(weld_tol, gap_warn_tol]`
   (default 0.01 mm), overlapping curves, self-intersecting splines, zero-length entities.
   Gaps are **not** healed automatically (open question 3).
7. **Stable ids (SK-15):** `ProfileId = hash64(sorted boundary edge keys)`. An edge key is
   `(source EntityId, start VertexKey, end VertexKey)`, and a `VertexKey` is
   `Endpoint(entity, End)`, `Intersection { a, b, ordinal }` (`a < b`, ordinal by parameter
   along `a`) or `Seam(entity)`. Dimension edits that keep the arrangement topology keep every
   `ProfileId`. When topology changes, `ProfileFingerprint` (area, centroid, bbox, source-set)
   lets S01 persistent naming match old to new (`ProfileSet::match_previous`). Profile
   `EntityRef`s are `Sketch#S / Profile / <ProfileId>`, with the fingerprint as fallback (input
   for DOC-11).

### Feature integration (`amg-features::sketch`)
- `SketchFeature` implements S01's `Feature`. Its inputs are the plane reference, the
  parameters referenced by its dimension `Expr`s, and projection `EntityRef`s.
- `compute` does, in order: resolve the plane, evaluate dimension `Expr`s (unit-checked:
  length → mm, angle → rad, ratio → dimensionless), refresh projected geometry (SK-09),
  `NativeSolver::solve` warm-started from the stored values, detect profiles, and output
  `FeatureOutput { sketch: Some(Arc<SketchGeom>) }`.
- A failed solve sets health `Warning` with the diagnosis and keeps the last good geometry
  (Fusion behaviour). It never aborts downstream recompute.
- Each driving dimension `Expr` is returned by `Feature::exprs_mut`. S01 then creates the
  model parameter (`d12`…) and rewrites it to `Expr::Param`. Driven dimensions should be
  exposed read-only (S01 request, open question 5).
- Sketch-mode edits: `SketchFeature::apply(SketchEdit)` on a clone, then S01
  `Command::EditFeature { id, feature }`. Undo/redo and the Python API come for free. Drag
  previews don't create undo entries. Only the final `SketchEdit::CommitDrag` does.

## Interfaces

All types are in `amg-sketch` unless prefixed. Lengths are in mm in sketch coordinates,
angles in radians, all `f64`. `Point2`/`Vec2`/`Plane`/`Aabb2`/`Segment2`/`Transform`/`Tolerance` are
the `amg-core` aliases (S00 FND-03). Collections are deterministic. Signatures marked
`// SK-xx` land with that WP; their exact shape may still change during it.

```rust
// ───────────────────────── amg_sketch::ids ─────────────────────────
use serde::{Deserialize, Serialize};

macro_rules! id { ($n:ident) => {
    #[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
    pub struct $n(pub u32);
}}
id!(EntityId); id!(ConstraintId); id!(ParamId); id!(ExternalLinkId);

// ───────────────────────── amg_sketch::model ───────────────────────
use std::collections::BTreeMap;
use smallvec::SmallVec;
use amg_core::{Aabb2, Plane, Point2, Segment2, Tolerance, Transform, Vec2};

// The sketch plane is `amg_core::Plane` (S00): `to_local`, `to_world`, `to_transform`.

#[derive(Copy, Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ParamFlags { pub locked: bool }

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ParamStore { values: Vec<f64>, flags: Vec<ParamFlags> }
impl ParamStore {
    pub fn get(&self, p: ParamId) -> f64;
    pub fn set(&mut self, p: ParamId, v: f64);
    pub fn flags(&self, p: ParamId) -> ParamFlags;
    pub fn as_slice(&self) -> &[f64];
    pub fn len(&self) -> usize;
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EntityOrigin { User, Projected(ExternalLinkId), Imported }

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum EntityKind {
    Point { x: ParamId, y: ParamId },
    Line { start: EntityId, end: EntityId },
    Circle { center: EntityId, radius: ParamId },
    /// CCW from `start` to `end`; |start−center| = |end−center| is an internal equation.
    Arc { center: EntityId, start: EntityId, end: EntityId },
    Ellipse { center: EntityId, major: EntityId, minor_radius: ParamId },            // SK-22
    EllipticArc { center: EntityId, major: EntityId, minor_radius: ParamId,
                  start: EntityId, end: EntityId, t0: ParamId, t1: ParamId },        // SK-22
    Spline(SplineDef),                                                              // SK-08
    /// Quadratic rational Bezier start->end, shoulder point `apex`, weights (1, rho/(1-rho), 1);
    /// passes through S + rho (apex - S), S = (start + end)/2; parameter range [0, 1].
    Conic { start: EntityId, end: EntityId, apex: EntityId, rho: ParamId },          // SK-08
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]                          // SK-08
pub enum SplineDef {
    Control { degree: u8, poles: Vec<EntityId>, weights: Option<Vec<ParamId>>,
              knots: Vec<f64>, periodic: bool },
    /// Cubic interpolation at frozen chord-length parameters `params` (one per point, 0..1).
    Fit { points: Vec<EntityId>, start_tangent: Option<EntityId>,
          end_tangent: Option<EntityId>, params: Vec<f64>, periodic: bool },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Entity { pub kind: EntityKind, pub construction: bool, pub origin: EntityOrigin }

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HvTarget { Line(EntityId), Points(EntityId, EntityId) }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TangentKind { External, Internal, AtEndpoint { reversed: bool } }
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Side { Positive, Negative }                       // captured at creation (chirality)
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AngleBranch { pub reverse_a: bool, pub reverse_b: bool }

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ConstraintKind {
    Coincident { a: EntityId, b: EntityId },
    PointOnCurve { point: EntityId, curve: EntityId },
    Midpoint { point: EntityId, line: EntityId },
    Horizontal(HvTarget),
    Vertical(HvTarget),
    Parallel { a: EntityId, b: EntityId },
    Perpendicular { a: EntityId, b: EntityId },
    Collinear { a: EntityId, b: EntityId },
    Tangent { a: EntityId, b: EntityId, kind: TangentKind },
    /// Tangent + equal signed curvature at the named shared ends (SK-08).
    Curvature { a: EntityId, b: EntityId, a_end: End, b_end: End, reversed: bool },
    Concentric { a: EntityId, b: EntityId },
    Equal { a: EntityId, b: EntityId },
    Symmetric { a: EntityId, b: EntityId, axis: EntityId },
    Fix { entity: EntityId },
    Dimension(Dimension),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum DimensionKind {
    Distance { a: EntityId, b: EntityId, side: Side },
    HorizontalDistance { a: EntityId, b: EntityId, side: Side },
    VerticalDistance { a: EntityId, b: EntityId, side: Side },
    Length { line: EntityId },
    Angle { a: EntityId, b: EntityId, branch: AngleBranch },
    Radius { curve: EntityId },
    Diameter { curve: EntityId },
    ArcLength { arc: EntityId },
    EllipseMajor { ellipse: EntityId },                                             // SK-22
    EllipseMinor { ellipse: EntityId },                                             // SK-22
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Quantity { Length, Angle }                        // what `value` means (mm | rad)

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Dimension {
    pub kind: DimensionKind,
    /// mm or rad, written by `amg-features::sketch` from the S01 `Expr` before each solve.
    pub value: f64,
    pub driving: bool,
    pub label_pos: Point2,                                 // UI placement only
}
impl DimensionKind { pub fn quantity(&self) -> Quantity; }

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Constraint { pub kind: ConstraintKind, pub aux: SmallVec<[ParamId; 1]> }

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sketch { /* plane, ParamStore, BTreeMap<EntityId, Entity>,
                       BTreeMap<ConstraintId, Constraint>, id counters, revisions */ }

impl Sketch {
    pub fn new(plane: Plane) -> Self;
    pub fn plane(&self) -> &Plane;
    pub fn set_plane(&mut self, plane: Plane);
    pub fn params(&self) -> &ParamStore;
    pub fn params_mut(&mut self) -> &mut ParamStore;       // bumps values_rev
    pub fn entity(&self, id: EntityId) -> Option<&Entity>;
    pub fn entities(&self) -> impl Iterator<Item = (EntityId, &Entity)> + '_;
    pub fn constraint(&self, id: ConstraintId) -> Option<&Constraint>;
    pub fn constraints(&self) -> impl Iterator<Item = (ConstraintId, &Constraint)> + '_;

    pub fn add_point(&mut self, p: Point2) -> EntityId;
    pub fn add_line(&mut self, a: Point2, b: Point2) -> EntityId;      // creates 2 points
    pub fn add_line_between(&mut self, a: EntityId, b: EntityId) -> Result<EntityId, SketchError>;
    pub fn add_circle(&mut self, center: Point2, r: f64) -> Result<EntityId, SketchError>;
    pub fn add_arc(&mut self, center: Point2, start: Point2, end: Point2) -> Result<EntityId, SketchError>;
    pub fn add_entity(&mut self, kind: EntityKind, construction: bool,
                      origin: EntityOrigin) -> Result<EntityId, SketchError>;  // generic
    pub fn alloc_param(&mut self, value: f64, flags: ParamFlags) -> ParamId;
    pub fn set_construction(&mut self, id: EntityId, on: bool) -> Result<(), SketchError>;
    /// Removes the entities, their dependent sub-entities, and every constraint touching them.
    pub fn remove_entities(&mut self, ids: &[EntityId]) -> Result<RemoveReport, SketchError>;

    /// Validates entity kinds and stores `kind` as given.
    pub fn add_constraint(&mut self, kind: ConstraintKind) -> Result<ConstraintId, SketchError>;
    /// Like `add_constraint`, but first recomputes `Side`/`TangentKind`/`AngleBranch` in `kind`
    /// from the current geometry so the new residual is zero-signed correctly (SK-02).
    pub fn add_constraint_oriented(&mut self, kind: ConstraintKind) -> Result<ConstraintId, SketchError>;
    pub fn remove_constraint(&mut self, id: ConstraintId) -> Result<(), SketchError>;
    pub fn set_dimension_value(&mut self, id: ConstraintId, value: f64) -> Result<(), SketchError>;
    pub fn set_driving(&mut self, id: ConstraintId, driving: bool) -> Result<(), SketchError>;

    pub fn curve(&self, id: EntityId) -> Option<geom::Curve2>;      // evaluated geometry
    pub fn structure_rev(&self) -> u64;                             // add/remove/kind change
    pub fn values_rev(&self) -> u64;                                // any param/value change
    pub fn compact(&mut self);                                      // renumber ParamIds only
}

#[derive(Clone, Debug, Default)]
pub struct RemoveReport { pub entities: Vec<EntityId>, pub constraints: Vec<ConstraintId> }

// ───────────────────────── amg_sketch::error ───────────────────────
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SketchError {
    #[error("unknown entity {0:?}")]
    UnknownEntity(EntityId),
    #[error("unknown constraint {0:?}")]
    UnknownConstraint(ConstraintId),
    #[error("constraint needs {expected}, but entity {entity:?} is a {found}")]
    WrongEntityKind { entity: EntityId, expected: &'static str, found: &'static str },
    #[error("constraint {0:?} is not a dimension")]
    NotADimension(ConstraintId),
    #[error("invalid value {value} for {what}")]
    InvalidValue { what: &'static str, value: f64 },
    #[error("entity {0:?} is locked (projected or fixed)")]
    Locked(EntityId),
    #[error("degenerate geometry: {0}")]
    Degenerate(&'static str),
    #[error("operation not applicable: {0}")]
    NotApplicable(&'static str),
    #[error("cancelled")]
    Cancelled,
}

// ───────────────────────── amg_sketch::geom ────────────────────────
// `Aabb2` and `Segment2` are the amg-core (S00) types; the rest are sketch-specific.
#[derive(Copy, Clone, Debug, PartialEq)] pub struct Circle2 { pub center: Point2, pub radius: f64 }
/// `sweep` > 0 is CCW, |sweep| ∈ (0, 2π].
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Arc2 { pub center: Point2, pub radius: f64, pub start_angle: f64, pub sweep: f64 }
/// Full ellipse when `t1 − t0 = 2π`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EllipseArc2 { pub center: Point2, pub major: Vec2, pub minor_radius: f64, pub t0: f64, pub t1: f64 }
#[derive(Clone, Debug, PartialEq)]
pub struct BSpline2 { pub degree: u8, pub knots: Vec<f64>, pub poles: Vec<Point2>, pub weights: Option<Vec<f64>> }

#[derive(Clone, Debug, PartialEq)]
pub enum Curve2 { Line(Segment2), Circle(Circle2), Arc(Arc2), Ellipse(EllipseArc2), BSpline(BSpline2) }
impl Curve2 {
    pub fn param_range(&self) -> (f64, f64);
    pub fn eval(&self, t: f64) -> Point2;
    pub fn derivative(&self, t: f64) -> Vec2;
    pub fn closest(&self, p: Point2) -> (f64, Point2);
    pub fn bbox(&self) -> Aabb2;
    pub fn reversed(&self) -> Curve2;
    pub fn trimmed(&self, t0: f64, t1: f64) -> Curve2;
    pub fn is_closed(&self) -> bool;
    pub fn length(&self) -> f64;
}
pub fn intersect(a: &Curve2, b: &Curve2, tol: &Tolerance) -> SmallVec<[CurveHit; 4]>;
#[derive(Copy, Clone, Debug)] pub struct CurveHit { pub ta: f64, pub tb: f64, pub point: Point2, pub tangential: bool }

// ───────────────────────── amg_sketch::solve ───────────────────────
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Algorithm { Auto, GaussNewtonMinNorm, LevenbergMarquardt, DogLeg }
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DofAnalysis { None, Total, PerEntity }

#[derive(Clone, Debug)]
pub struct SolveOptions {
    pub algorithm: Algorithm,     // default Auto (chosen by SK-13/SK-03)
    pub tolerance: f64,           // max |residual| in mm (scaled), default 1e-10
    pub max_iterations: u32,      // per block, default 50
    pub dof: DofAnalysis,         // default None
    pub diagnose: bool,           // run diagnosis when not Converged; default true
}
impl Default for SolveOptions { fn default() -> Self; }

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SolveStatus { Converged, ConvergedRedundant, Inconsistent, DidNotConverge, Cancelled }

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConstraintState { Under, Full, Over }
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct EntityStatus { pub state: ConstraintState, pub free_dof: u8 }
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DofReport { pub dof: u32, pub entities: BTreeMap<EntityId, EntityStatus> }

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GroupKind { Redundant, Conflicting, DegenerateConfiguration }
#[derive(Clone, Debug, PartialEq)]
pub struct DependencyGroup { pub kind: GroupKind, pub constraints: Vec<ConstraintId> }   // sorted
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Diagnosis { pub groups: Vec<DependencyGroup> }

#[derive(Clone, Debug)]
pub struct SolveReport {
    pub status: SolveStatus,
    pub iterations: u32,
    pub max_residual: f64,
    pub unsatisfied: Vec<ConstraintId>,
    pub driven: BTreeMap<ConstraintId, f64>,  // measured driven dimension values
    pub dof: Option<DofReport>,
    pub diagnosis: Option<Diagnosis>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ConstraintCheck {
    Independent,
    Redundant { with: Vec<ConstraintId> },
    Conflicting { with: Vec<ConstraintId> },
}

#[derive(Copy, Clone, Debug)]
pub struct DragSpec { pub entity: EntityId, pub grab: Point2 } // grab point in sketch coords
#[derive(Copy, Clone, Debug)]
pub struct DragStep { pub status: SolveStatus, pub iterations: u32, pub max_residual: f64 }

pub trait DragSession: Send {
    /// Moves the grabbed point toward `target` (sketch coords) and re-solves (SK-05).
    fn step(&mut self, sketch: &mut Sketch, target: Point2, cancel: &dyn Fn() -> bool) -> DragStep;
    /// Final full solve; caller turns the parameter delta into `SketchEdit::CommitDrag`.
    fn finish(self: Box<Self>, sketch: &mut Sketch) -> SolveReport;
}

pub trait SolverBackend: Send {
    fn name(&self) -> &'static str;
    fn solve(&mut self, sketch: &mut Sketch, opts: &SolveOptions,
             cancel: &dyn Fn() -> bool) -> SolveReport;                                     // SK-03
    fn analyze(&mut self, sketch: &Sketch) -> (DofReport, Diagnosis);                       // SK-04
    fn check_constraint(&mut self, sketch: &Sketch, candidate: &ConstraintKind)
        -> Result<ConstraintCheck, SketchError>;                                            // SK-04
    fn begin_drag(&mut self, sketch: &Sketch, spec: DragSpec)
        -> Result<Box<dyn DragSession>, SketchError>;                                       // SK-05
}

pub struct NativeSolver { /* cached decomposition keyed by structure_rev */ }
impl NativeSolver { pub fn new() -> Self; }
impl SolverBackend for NativeSolver { /* … */ }

#[cfg(feature = "slvs")]   // GPL-3.0 (ADR-012): dev/test oracle only unless the owner decides otherwise
pub struct SlvsSolver { /* … */ }

/// Dimension-agnostic layer reusable by SK-12 and S06 ASM-04.
pub mod system {
    pub trait Residual: Send + Sync {
        fn rows(&self) -> usize;
        fn params(&self) -> &[super::ParamId];
        fn eval(&self, x: &[f64], out: &mut [f64]);
        /// Row-major dense block, `rows × params().len()`.
        fn jacobian(&self, x: &[f64], out: &mut [f64]);
    }
    pub struct EquationSystem { /* Vec<Box<dyn Residual>>, owners: Vec<ConstraintId> */ }
}

// ───────────────────────── amg_sketch::profile ─────────────────────
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ProfileId(pub u64);
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum End { Start, End }
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum VertexKey {
    Endpoint(EntityId, End),
    Intersection { a: EntityId, b: EntityId, ordinal: u16 },   // a < b
    Seam(EntityId),                                            // closed curve with no split
}
#[derive(Clone, Debug)]
pub struct ProfileEdge {
    pub source: EntityId,
    pub also: SmallVec<[EntityId; 1]>,   // overlapping coincident sources merged into this edge
    pub curve: geom::Curve2,             // trimmed, oriented along the loop
    pub start: VertexKey,
    pub end: VertexKey,
}
#[derive(Clone, Debug)] pub struct Loop { pub edges: Vec<ProfileEdge> }   // outer CCW, holes CW
#[derive(Clone, Debug, PartialEq)]
pub struct ProfileFingerprint { pub area: f64, pub centroid: Point2, pub bbox: Aabb2, pub sources: Vec<EntityId> }
#[derive(Clone, Debug)]
pub struct Profile { pub id: ProfileId, pub outer: Loop, pub holes: Vec<Loop>, pub fingerprint: ProfileFingerprint }
#[derive(Clone, Debug)]
pub enum ProfileIssue {
    NearGap { a: VertexKey, b: VertexKey, distance: f64 },
    Overlap { a: EntityId, b: EntityId },
    SelfIntersection(EntityId),
    ZeroLength(EntityId),
}
#[derive(Clone, Debug)]
pub struct ProfileOptions {
    pub weld_tol: f64,             // default 1e-6 mm
    pub gap_warn_tol: f64,         // default 1e-2 mm
    pub include_projected: bool,   // default true
}
impl Default for ProfileOptions { fn default() -> Self; }
#[derive(Clone, Debug, Default)]
pub struct ProfileSet { pub profiles: Vec<Profile>, pub open_chains: Vec<Vec<EntityId>>, pub issues: Vec<ProfileIssue> }
impl ProfileSet {
    pub fn get(&self, id: ProfileId) -> Option<&Profile>;
    pub fn at_point(&self, p: Point2) -> Option<ProfileId>;              // innermost profile
    /// SK-15: map old ids to new ones after a topology change, by key, then fingerprint.
    pub fn match_previous(&self, previous: &ProfileSet) -> BTreeMap<ProfileId, ProfileMatch>;
    /// Pure fingerprint match (persistent-naming note, 5.4); never `Same`.
    pub fn match_fingerprint(&self, fp: &ProfileFingerprint) -> ProfileMatch;
}
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ProfileMatch { Same(ProfileId), Fingerprint { new: ProfileId, score: f64 }, Lost }
pub fn detect_profiles(sketch: &Sketch, opts: &ProfileOptions) -> ProfileSet;

// ───────────────────────── amg_sketch (re-exported by amg-doc, S01) ─────
/// Replaces the S01 stubs `amg_doc::stub::{SketchGeom, SketchEntityKey, ProfileKey}`.
pub type SketchEntityKey = EntityId;
pub type ProfileKey = ProfileId;
/// Evaluated sketch carried in S01 `FeatureOutput::sketch` and read via `RecomputeCtx::sketch`.
#[derive(Clone, Debug)]
pub struct SketchGeom {
    pub plane: Plane,                    // world placement at this recompute
    pub sketch: std::sync::Arc<Sketch>,  // solved geometry
    pub report: SolveReport,
    pub profiles: std::sync::Arc<ProfileSet>,
}

// ───────────────────────── amg_sketch::query (SK-20) ───────────────
#[derive(Copy, Clone, Debug, Default)]
pub struct PickFilter { pub points: bool, pub curves: bool, pub construction: bool, pub projected: bool }
#[derive(Copy, Clone, Debug)] pub struct PickHit { pub entity: EntityId, pub distance: f64, pub param: f64 }
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SnapKind { Endpoint, Midpoint, Center, Quadrant, OnCurve, Intersection, AlignHorizontal, AlignVertical, Tangent, Perpendicular }
#[derive(Clone, Debug)]
pub struct SnapCandidate { pub kind: SnapKind, pub point: Point2, pub entities: SmallVec<[EntityId; 2]>, pub suggested: Option<ConstraintKind> }
pub fn pick(sketch: &Sketch, p: Point2, radius: f64, filter: PickFilter) -> Vec<PickHit>;   // sorted by distance
pub fn snap_candidates(sketch: &Sketch, p: Point2, radius: f64, from: Option<Point2>) -> Vec<SnapCandidate>;

// ───────────────────────── amg_sketch::build (SK-21) ───────────────
#[derive(Clone, Debug, Default)]
pub struct Built { pub entities: Vec<EntityId>, pub constraints: Vec<ConstraintId> }
pub mod build {
    use super::*;
    pub fn rectangle_2pt(s: &mut Sketch, a: Point2, b: Point2) -> Result<Built, SketchError>;
    pub fn rectangle_3pt(s: &mut Sketch, a: Point2, b: Point2, c: Point2) -> Result<Built, SketchError>;
    pub fn rectangle_center(s: &mut Sketch, center: Point2, corner: Point2) -> Result<Built, SketchError>;
    pub fn polygon(s: &mut Sketch, center: Point2, vertex: Point2, sides: u32, inscribed: bool) -> Result<Built, SketchError>;
    pub fn slot_center_to_center(s: &mut Sketch, a: Point2, b: Point2, width: f64) -> Result<Built, SketchError>;
    pub fn slot_overall(s: &mut Sketch, a: Point2, b: Point2, width: f64) -> Result<Built, SketchError>;
    pub fn slot_center_point(s: &mut Sketch, center: Point2, end: Point2, width: f64) -> Result<Built, SketchError>;
    pub fn slot_arc(s: &mut Sketch, center: Point2, start: Point2, end: Point2, width: f64) -> Result<Built, SketchError>;
    pub fn circle_3pt(s: &mut Sketch, a: Point2, b: Point2, c: Point2) -> Result<Built, SketchError>;
    pub fn circle_2_tangent(s: &mut Sketch, a: EntityId, b: EntityId, r: f64, near: Point2) -> Result<Built, SketchError>;
    pub fn arc_3pt(s: &mut Sketch, a: Point2, through: Point2, b: Point2) -> Result<Built, SketchError>;
    pub fn arc_tangent(s: &mut Sketch, from: EntityId, at: End, end: Point2) -> Result<Built, SketchError>;
}

// ───────────────────────── amg_sketch::edit (SK-07, SK-16..18) ─────
#[derive(Clone, Debug, Default)]
pub struct EditResult {
    pub created: Vec<EntityId>, pub removed: Vec<EntityId>,
    pub constraints_added: Vec<ConstraintId>, pub constraints_removed: Vec<ConstraintId>,
}
#[derive(Copy, Clone, Debug)]
pub enum ChamferSpec { Equal(f64), TwoDistances(f64, f64), DistanceAngle(f64, f64) }
pub mod edit {
    use super::*;
    pub fn trim(s: &mut Sketch, entity: EntityId, pick: Point2) -> Result<EditResult, SketchError>;     // SK-07
    pub fn extend(s: &mut Sketch, entity: EntityId, pick: Point2) -> Result<EditResult, SketchError>;   // SK-07
    pub fn break_at(s: &mut Sketch, entity: EntityId, at: Point2) -> Result<EditResult, SketchError>;   // SK-07
    pub fn fillet(s: &mut Sketch, a: EntityId, b: EntityId, near: Point2, r: f64) -> Result<EditResult, SketchError>;          // SK-16
    pub fn chamfer(s: &mut Sketch, a: EntityId, b: EntityId, near: Point2, spec: ChamferSpec) -> Result<EditResult, SketchError>; // SK-16
    pub fn offset(s: &mut Sketch, chain: &[EntityId], distance: f64, side: Point2, constrain: bool) -> Result<EditResult, SketchError>; // SK-17
    pub fn mirror(s: &mut Sketch, items: &[EntityId], axis: EntityId, constrain: bool) -> Result<EditResult, SketchError>;        // SK-18
    pub fn pattern_rect(s: &mut Sketch, items: &[EntityId], dir_a: Vec2, n_a: u32, d_a: f64,
                        dir_b: Option<(Vec2, u32, f64)>, constrain: bool) -> Result<EditResult, SketchError>;             // SK-18
    pub fn pattern_circ(s: &mut Sketch, items: &[EntityId], center: EntityId, n: u32, total_angle: f64,
                        constrain: bool) -> Result<EditResult, SketchError>;                                               // SK-18
    pub fn transform(s: &mut Sketch, items: &[EntityId], t: amg_core::Transform /* in-plane */,
                     copy: bool) -> Result<EditResult, SketchError>;                                                       // SK-18 (move/copy/rotate/scale)
}
```

`amg-features::sketch` (lane B module). It uses S01 types (`Feature`, `RecomputeCtx`,
`FeatureOutput`, `Expr`, `EntityRef`, `Command::EditFeature`) as the S01 draft of 2026-10-19
defines them, and invents none:

```rust
use std::{collections::BTreeMap, sync::Arc};
use amg_doc::{Expr, EntityRef};                       // S01 amg-doc::param / ::naming
use amg_sketch::{Sketch, ConstraintId, EntityId, ExternalLinkId, ConstraintKind, EntityKind,
                 ParamId, EditResult, SketchError};

/// `Ref` is an `EntityRef::Construction(..)` plane or an `EntityRef::Topo(..)` planar face.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum PlaneInput { OriginXY, OriginYZ, OriginXZ, Ref(EntityRef) }

#[derive(Copy, Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ProjectionMode { Project, Include, Intersect, IntersectionCurve }        // SK-09

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ProjectionLink { pub source: EntityRef, pub mode: ProjectionMode, pub entities: Vec<EntityId> }

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SketchFeature {
    pub plane: PlaneInput,
    pub sketch: Sketch,                          // authored geometry + last solved values (warm start)
    /// Driving dimension values. `exprs_mut()` exposes them, so S01 turns each `Expr::Inline`
    /// into a model parameter (`d<n>`) and rewrites it to `Expr::Param`.
    pub dims: BTreeMap<ConstraintId, Expr>,
    pub links: BTreeMap<ExternalLinkId, ProjectionLink>,
}
/// `kind() == "sketch.sketch"`; `compute` returns `FeatureOutput { sketch: Some(Arc<SketchGeom>), .. }`.
impl amg_doc::Feature for SketchFeature { /* … */ }

impl SketchFeature {
    /// Applies one edit to this (cloned) feature. The UI then issues
    /// `Command::EditFeature { id, feature: FeatureBox(Arc::new(edited)) }`, so one
    /// `SketchEdit` (or `Batch`) = one undo entry.
    pub fn apply(&mut self, edit: SketchEdit) -> Result<EditResult, SketchError>;
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum SketchEdit {
    AddEntity { kind: EntityKind, construction: bool },
    RemoveEntities(Vec<EntityId>),
    AddConstraint { kind: ConstraintKind, expr: Option<Expr> },
    RemoveConstraint(ConstraintId),
    SetDimension { id: ConstraintId, expr: Expr },
    SetDriving { id: ConstraintId, driving: bool },
    SetConstruction { id: EntityId, on: bool },
    CommitDrag { values: Vec<(ParamId, f64)> },
    Batch(Vec<SketchEdit>),                      // builders / edit ops produce a batch
}
```

**Consumer contracts:**
- **SOL-02/03 (S04):** read `SketchGeom::profiles` (via `RecomputeCtx::sketch`), select by
  `EntityRef::Profile { sketch, profile: ProfileId }`, and pass `Profile` loops (`Curve2` +
  `plane`) to an `amg-brep` planar-face builder (lane A, BREP-03).
- **UI-09 (S11):** uses `NativeSolver` (solve/analyze/check/drag), `query`, `build` and
  `edit`, and emits `SketchEdit`s.
- **IO-08 (S12):** reads `Sketch::curve` per entity.

## Work packages
All lane B. Work packages SK-13 to SK-22 are new from this refinement; the phase placement
the coordinator should add to the roadmap is in the Refinement log.

| ID | Title | Deps | Pri | Status | Done when |
|---|---|---|---|---|---|
| SK-01 | Entity model, `ParamStore` layout, sketch plane (`amg_core::Plane`), constraint/dimension data types, ids, validation, serde | FND-03 | P0 | Done | 1. All P0 entity kinds (point, line, circle, arc) and every `ConstraintKind`/`DimensionKind` variant can be created through the API. 2. Kind validation returns `WrongEntityKind` (≥ 1 negative test per variant). 3. Three fixture sketches round-trip through serde bit-identically. 4. `remove_entities` cascades to constraints (unit test). |
| SK-02 | Residuals + analytic Jacobians for all P0 constraints and dimensions; `system::Residual`; chirality capture | SK-01 | P0 | Done | 1. Every P0 constraint and dimension passes a central-difference Jacobian check (relative error < 1e-6) on 1000 random configurations each (proptest). 2. Captured `Side`/`TangentKind`/`AngleBranch` make the residual zero at creation (unit test). |
| SK-14 | Solver test corpus + runner: corpus file format (JSON), `.slvs` 2D importer, ezpz text importer, hand-ported PlaneGCS cases, expected-result schema | SK-01 | P0 | Done | 1. `testdata/sketch/corpus/` holds ≥ 120 cases: ≥ 60 from SolveSpace 2D (`test/constraint/*`), ≥ 20 PlaneGCS, ≥ 20 ezpz, ≥ 20 own. 2. Each case has expected DOF and status. 3. The runner reports pass/fail per case. 4. Licence notices are in each source subfolder. |
| SK-13 | Solver prototype + ADR-007 benchmark: native prototype vs `libslvs` (feature `slvs`) vs ezpz on decision cases D1–D10 (note §5) | SK-02, SK-14 | P0 | Done | 1. `docs/notes/sketch-solver-benchmark.md` reports every §5.3 metric for every option on D1–D10. 2. Proposed ADR-007 wording is in the PR. 3. The owner's decision is recorded. *This WP produces the decision, so it is not behind the ADR-007 gate.* |
| SK-03 | Production solver core: substitution, connected components + Dulmage–Mendelsohn blocks, dense/sparse damped min-norm GN/LM, structure cache, `SolveStatus` / unsatisfied reporting, cancellation, driven-dimension measurement | SK-13 (ADR-007 gate cleared 2026-10-23) | P0 | Todo | 1. ≥ 95 % of the SK-14 corpus passes, and 100 % of its non-degenerate well-constrained cases. 2. D9 warm solve < 5 ms median on the reference machine (criterion). 3. The same input solved 100× gives bit-identical params. 4. Cancellation returns `Cancelled` within 1 ms. |
| SK-04 | DOF analysis (rank-revealing QR/SVD, per-entity status), diagnosis (redundant / conflicting / degenerate groups, leave-one-out refinement), `check_constraint` | SK-03 | P0 | Todo | 1. All 50 hand-made DOF cases (§Testing) match exactly: total DOF, every entity's `ConstraintState`, and dependency groups as sets. 2. `check_constraint` agrees with full re-analysis on every case. 3. Analysis of a 500-entity sketch < 10 ms. |
| SK-05 | Drag solving: `DragSession` for points, lines (translate), circle rim (radius), arc endpoints; weighted min-norm; warm start; SQP alternative only if SK-13 selected it | SK-03 | P0 | Todo | 1. On D4, D7 and a 200-entity generated sketch, 1000 scripted frames each: p95 step < 2 ms. 2. The jitter metric (note §5.3) is ≤ the SK-13 baseline. 3. No chirality flip on D7. 4. A fully constrained dragged point stays put. |
| SK-06 | Profile/region detection (lines, arcs, circles): intersections, split/weld, merge overlaps, half-edge faces, holes and islands, open chains, issues | SK-01 | P0 | Done | 1. ≥ 25 golden fixtures pass: profile count, outer/hole counts and areas (relative 1e-9). Fixtures cover nested islands, T-junctions, tangent circles, overlapping collinear lines, near gaps, zero-length lines. 2. A fuzz test of 10 000 random line/arc soups never panics, satisfies Euler `V − E + F = 1 + C`, and the profile area sum equals the raster-union area within 0.5 %. 3. 500-curve sketch < 5 ms. |
| SK-15 | Stable `ProfileId`s, `ProfileFingerprint`, `ProfileSet::match_previous` | SK-06 | P0 | Done | 1. 20 scripted dimension edits that keep topology keep every `ProfileId`. 2. 10 topology-changing edits give deterministic new ids, and `match_previous` maps each surviving region to the intended successor (fixtures list the intent). |
| SK-21 | Composite builders: rectangles (2pt, 3pt, centre), polygon, slots (4 variants), 3-point/tangent circles and arcs | SK-02 | P0 | Done | 1. Each builder's output has zero residual. 2. Each builder has the expected constraint set and DOF (e.g. 2-pt rectangle: DOF 4, slot: DOF 5), checked with the SK-02 rank helper. 3. Degenerate input returns `Degenerate`. |
| SK-20 | Queries & inference hooks for UI-09: `pick`, `snap_candidates` (endpoint/mid/centre/quadrant/intersection/alignment/tangent/perpendicular with suggested constraint), filtered by `check_constraint` | SK-04 | P0 | Todo | 1. Unit tests per `SnapKind`. 2. Redundant suggestions are suppressed (test on a fully constrained rectangle). 3. `pick` + `snap_candidates` < 0.1 ms on a 500-entity sketch. |
| SK-19 | `amg-features::sketch`: `SketchFeature` (plane resolution, `Expr` evaluation with unit check, solve, profiles, health), `SketchEdit` via `EditFeature`, model params for driving dims | SK-03, SK-06, DOC-02, DOC-04, DOC-05 | P0 | Todo | 1. Changing a user parameter referenced by a dimension recomputes the sketch and its profiles. 2. Wrong-unit `Expr` → feature error. 3. Driving dims appear as `dN` model params. 4. An inconsistent edit → `Warning` with diagnosis and previous geometry kept. 5. Random `SketchEdit` sequence → undo all → initial state (property test). 6. Save/load round-trip. |
| SK-07 | Trim, extend, break, with constraint migration rules (coincident/tangent kept on surviving pieces, dimensions on deleted pieces removed and reported) | SK-06, SK-02 | P0 | Done | 1. ≥ 30 per-op fixtures (each op × line/arc/circle × intersection kinds) give the expected entities and constraints. 2. The result solves with residual 0. |
| SK-16 | Sketch fillet and chamfer (equal, two-distance, distance-angle): trims both curves, adds tangent/coincident constraints and a radius/length dimension | SK-07 | P0 | Done | 1. Fixtures for line–line, line–arc, arc–arc, including corners where a fillet does not fit (→ `NotApplicable`). 2. Result DOF matches each fixture's hand-computed value, and the fillet arc is fully determined by its neighbours plus the radius dimension. |
| SK-17 | Offset of curve chains (lines/arcs/circles; splines approximated in SK-08) with gap closing (arc/extend) and optional offset constraint (equal-distance) | SK-06, SK-02 | P0 | Done | 1. Open/closed chain fixtures, including self-intersecting offsets that get trimmed. 2. With `constrain = true`, editing the offset dimension moves the copy. |
| SK-18 | Mirror, rectangular/circular pattern, move/copy/rotate/scale, with optional symmetric/pattern constraints | SK-02 | P0 | Done | 1. A constrained mirror/pattern follows its source when the source is edited (re-solve test). 2. An unconstrained copy is independent. 3. Scale about a point scales dimension values. |
| SK-22 | Ellipse & elliptical arc: entities, internal equations, point-on/tangent/axis dimensions, `Curve2::Ellipse` intersections for profiles | SK-02, SK-06 | P0 | Done | 1. FD Jacobian checks pass for all ellipse rows. 2. An ellipse-in-rectangle (tangent ×4) solves. 3. A profile fixture with an ellipse hole has area within 1e-9 relative. |
| SK-08 | Splines (control-point + fit-point, rational weights) and conics: entities, point-on-curve / tangent / curvature (G2) constraints, intersectors, offset approximation | SK-02, SK-06 | P1 | Done | 1. FD checks for spline/conic rows. 2. Spline tangent-to-line and G2-to-arc fixtures solve. 3. A spline profile area agrees with a 10⁴-segment polyline within 1e-6 relative. |
| SK-09 | Projection / include / intersect / intersection curve: locked `Projected` entities refreshed from `EntityRef` at recompute | SK-19, BREP-17, DOC-06 | P0 | Todo | 1. Editing the upstream extrude height/sketch moves the projected edges and the dependent constrained geometry (golden test on 3 models). 2. An unresolvable ref → `Warning`, and the entities keep their last geometry. |
| SK-10 | Text (font, size, bold/italic, along path) as text blocks of locked spline outlines | SK-08 | P1 | Todo | 1. A text string produces profiles whose glyph count and hole count (e.g. "AgB" → 3 outer, 4 holes) match. 2. Extrudes via SOL-02 in a golden test. |
| SK-11 | DXF/SVG import → entities (scale, units, layers → construction toggle, near-coincident endpoint merge with coincident constraints) | SK-08 | P1 | Review | 1. 10 sample files (≥ 3 DXF R12/2000+, ≥ 3 SVG with transforms) import at the correct scale (bbox within 1e-6 relative). 2. The expected profile counts are produced. |
| SK-12 | 3D sketch: 3D points/lines/splines, 3D constraints on the shared `system` layer | SK-04, SK-19 | P2 | Todo | 1. A 3D polyline with lengths/angles/coincidence to model vertices solves. 2. DOF analysis is correct on 10 3D cases. |

## Testing
- **Jacobians (SK-02, SK-08, SK-22).** Central-difference check for every residual row on
  random configurations (proptest), with relative error < 1e-6 and a 1e-9 absolute floor.
  There is also a symbolic spot check against the SolveSpace equation forms listed in the
  note.
- **Solver corpus (SK-14).** Source sets and how they are ported:
  1. **SolveSpace** `test/constraint/*` (36 dirs, `.slvs` text files). The SK-14 importer
     reads `Param`/`Request`/`Entity`/`Constraint` records of cases that lie in one workplane
     and skips 3D-only cases (`pt_face_distance`, `pt_on_face`, `pt_plane_distance`,
     `pt_in_plane`, `free_in_3d*`). Expected result: converges with the stored parameter
     values (from the `.slvs` file) within 1e-6 mm, plus the DOF given in `test.cpp`
     assertions where present. GPL-3 data, kept in `testdata/sketch/corpus/solvespace/` with
     upstream `COPYING` + `NOTICE`, test-only.
  2. **FreeCAD PlaneGCS:** the gtests in `tests/src/Mod/Sketcher/App/planegcs/` and the
     Python Sketcher solver tests. These are hand-ported (geometry + constraints + expected
     redundant/conflicting sets), LGPL-2.1+, in `corpus/planegcs/`.
  3. **ezpz** `test_cases/` (MIT). Automatic text-format importer, in `corpus/ezpz/`.
  4. **Own:** D1–D10 (note §5.2), generated grids (D9 family), Fusion-tutorial sketches
     rebuilt by hand.
- **Differential testing.** With `--features slvs`, the runner also solves every case with
  libslvs and flags disagreements in convergence or DOF. This is a nightly/optional CI job and
  never a default build.
- **The 50 hand-made DOF cases (SK-04)**, in `testdata/sketch/dof/`, each with expected total
  DOF, per-entity state and dependency groups:
  - **10 under-constrained:**
    - free point (2)
    - free line (4)
    - line + horizontal (3)
    - circle (3)
    - arc (5)
    - two lines sharing an endpoint (6)
    - polyline of 5 (12)
    - rectangle with only H/V (4)
    - circle centre fixed (1)
    - line with length only (3)
  - **10 fully constrained:**
    - fixed point
    - triangle with 3 lengths + fix + H
    - rectangle with 2 dims + fix
    - slot with width/length + fix + H
    - circle with fixed centre + radius
    - arc with fixed centre/radius + 2 angles
    - L-bracket with 6 lines
    - concentric pair with radii
    - symmetric trapezoid
    - tangent arc chain of 3
  - **10 redundant (consistent):**
    - rectangle + extra parallel
    - rectangle + 4th perpendicular
    - midpoint + symmetric about the same line
    - equal ×3 in a triangle with all lengths
    - double coincident
    - H on both lines of a collinear pair
    - concentric + coincident centres
    - fix + dims on a fixed entity
    - tangent + coincident duplicate
    - driven-vs-driving duplicate after toggling
  - **10 conflicting:**
    - two different distances on the same pair
    - triangle inequality violated
    - H + V on a line of non-zero length
    - perpendicular + parallel on the same pair
    - radius ≠ fixed rim point distance
    - length vs fixed endpoints
    - angle 30° + parallel
    - equal radius with different radius dims
    - tangent to two non-reachable circles
    - symmetric with an off-axis fixed point
  - **5 mixed** (rigid sub-cluster + free part, checking per-entity status boundaries).
  - **5 degenerate/witness:**
    - generically well-constrained but initially collinear
    - zero-length line
    - coincident circle centres before concentric
    - an arc with start = end
    - a near-singular four-bar toggle

    These expect *generic* status, with `DegenerateConfiguration` where applicable.
- **Property tests.**
  - Build random well-constrained sketches by measuring dimensions from random geometry
    (always consistent), perturb parameters by ≤ 10 %, and check that the solver recovers
    residual < tol with DOF = 0.
  - Random edit sequences never panic.
  - Profile fuzz (SK-06).
- **Golden.** Profile fixtures (SK-06/15/22/08). Sketch-feature recompute goldens in
  `tests/golden/sketch/` (SK-19, SK-09, SK-10).
- **Benchmarks (criterion, `amg-sketch/benches/`):** `solve_cold`, `solve_warm`, `drag_step`,
  `dof_analysis`, `profiles` on the D-cases and generated sketches of 50/200/500/2000
  entities.

## Performance targets
Reference machine (vision doc), single thread.

| Operation | Size | Target |
|---|---|---|
| Warm solve after one dimension edit (cached structure) | 500 entities | < 5 ms median, < 10 ms p95 |
| Cold solve (new structure) | 500 entities | < 20 ms |
| Drag step | 200 entities | < 2 ms p95, so 120 Hz with headroom |
| DOF analysis (per entity) | 500 entities | < 10 ms |
| Diagnosis after failure | 500 entities | < 100 ms |
| `check_constraint` | 500 entities | < 1 ms |
| Profile detection | 500 curves / 5 000 curves (DXF) | < 5 ms / < 100 ms |
| Pick + snap query | 500 entities | < 0.1 ms |
| `SketchFeature::compute` total (typical 50-entity sketch) | — | < 10 ms, so a 50-feature part stays within the 500 ms recompute budget |

## Open questions
1. **(owner, answered)** ADR-007: do you accept the SK-13 decision rule in
   `docs/notes/sketch-solver.md` §5.4? The recommended outcome is our own Rust solver as
   primary, with `libslvs` only as a GPL test oracle behind the `slvs` feature. You decide
   after the SK-13 report. SK-03 waits for this.
   **Answered 2026-10-20:** the SK-13 decision rule (note §5.4) is accepted. ADR-007 is finalized once the SK-13 report meets it (recommended).
2. **(owner, answered)** Over-constraint UX: should adding an over-constraining dimension be refused
   with an offer to make it driven (Fusion behaviour, recommended), or allowed and shown red
   (SolveSpace behaviour)? The solver supports both through `check_constraint`.
   **Answered 2026-10-20:** refuse and offer to make the dimension driven (Fusion behaviour, recommended).
3. **(owner, answered)** Near-closed gaps (≤ 0.01 mm, common in DXF imports): only warn in "sketch
   doctor" (Fusion, default), or offer an opt-in "heal gaps" that inserts coincident
   constraints? This affects SK-06/SK-11.
   **Answered 2026-10-20:** warn only in "sketch doctor" (recommended default).
4. **(owner, answered)** Should `libslvs` ever ship as a *runtime* fallback solver in a GPL build, or
   stay test-only? Recommended: test-only unless SK-13 shows a > 2 % corpus gap.
   **Answered 2026-10-20:** test-only, unless SK-13 shows a > 2 % corpus gap (recommended).
5. S01 coordination (lane E, not owner):
   - (a) Driven dimensions need read-only model parameters (`ParamKind` has no read-only
     variant in the S01 draft).
   - (b) S01's stubs `SketchGeom` / `SketchEntityKey` / `ProfileKey` are to be replaced by the
     `amg_sketch` items of the same names defined here (SK-01 / SK-06 / SK-15).
   - (c) DOC-11 should resolve `EntityRef::Profile` by `ProfileId`, then by
     `ProfileSet::match_previous` (fingerprint).
6. S00 coordination (lane E): `amg_sketch` uses `amg_core::{Plane, Aabb2, Segment2, Point2,
   Vec2, Tolerance}` from the S00 draft. If FND-03 adds further 2D curve primitives (arcs,
   circles), `amg_sketch::geom` re-exports them instead of defining its own.
7. ~~Auto-constraints while drawing~~. Resolved: the rules stay in S11 UI-09, and the solver
   hooks are `query::snap_candidates` + `SolverBackend::check_constraint` (SK-20).

## Refinement log
### 2026-10-19 — refinement (lane B agent)
**Researched:**
- SolveSpace / `libslvs`: C API, result codes, substitution, minimum-norm least-squares
  Newton step with column scaling and Eigen `SparseQR` + COLAMD in current `system.cpp`,
  leave-one-out failed-constraint search, the 36 `test/constraint` case directories and their
  `.slvs` + `test.cpp` layout.
- The `slvs` Rust crate (GPL-3.0, 0.6.0, June 2023, `cc` + bindgen/libclang).
- FreeCAD PlaneGCS (`GCS.cpp`, `SubSystem.cpp`, `qp_eq.cpp`): DogLeg default, LM, BFGS, SQP
  with temporary constraints for drag, QR-based redundant/conflicting/partially-redundant
  detection, subsystem split; gtests under `tests/src/Mod/Sketcher/App/planegcs/`.
- Zoo `ezpz` (MIT, pure Rust, text test-case format).
- Literature: Pothen & Fan 1990 (block triangular form / Dulmage–Mendelsohn);
  Hopcroft–Karp 1973; Ait-Aoudia, Jegou & Michelucci 1993; Latham & Middleditch 1996;
  Hoffmann, Lomonosov & Sitharam 2001 (DR-planning); Jermann et al. 2006 (survey);
  Michelucci & Foufou 2006 (witness configurations); Nocedal & Wright (LM/dogleg);
  Bettig & Hoffmann 2011.

**Decided:**
- SolveSpace-style centre/start/end arcs.
- Monotonic, never-reused `u32` ids in `BTreeMap`s (determinism, persistent naming).
- Dimensions are `f64` in `amg-sketch`, with the S01 `Expr` held by `SketchFeature`, so
  layering stays intact (`amg-sketch` never depends on `amg-doc`).
- Signed residuals with orientation captured at creation, to preserve chirality.
- Substitution + DM decomposition. DR-planning is deferred until measured.
- Weighted min-norm GN/LM, dense ≤ 64 unknowns / sparse above.
- QR-based DOF and diagnosis with a witness tie-break and leave-one-out refinement.
- Drag by weighted min-norm; SQP stays an SK-13 alternative.
- Half-edge profile detection with exact areas.
- `ProfileId` from vertex-keyed edge sets plus a fingerprint fallback.
- `SolverBackend` trait so `libslvs` can plug in behind the `slvs` feature.
- ADR-012 implies `libslvs` can never be the only solver, so the real ADR-007 choice is the
  native solver (or ezpz) as primary, with libslvs as an optional oracle. Full comparison and
  the SK-13 plan are in `docs/notes/sketch-solver.md` (new).

**Changed:**
- Status Draft → Refined.
- Added Interfaces, Performance targets and full Testing (corpus sources, 50 DOF cases).
- **New WPs:**
  - SK-13: solver prototype + ADR-007 benchmark; ungated, it feeds the gate.
  - SK-14: corpus + runner.
  - SK-15: stable profile ids, split out of SK-06.
  - SK-16: fillet/chamfer, split out of SK-07.
  - SK-17: offset, split out of SK-07.
  - SK-18: mirror/pattern/transform, split out of SK-07.
  - SK-19: `amg-features::sketch` feature integration; this was missing.
  - SK-20: queries/inference hooks for UI-09.
  - SK-21: composite builders (rectangles/slots/polygons are P0 in parity).
  - SK-22: ellipse/elliptical arc, split out of SK-08 because the parity matrix lists
    ellipse as P0.
- **Changed WPs:**
  - SK-03 now depends on SK-13. It is still gated by ADR-007, and its Done-when is
    measurable.
  - SK-07 narrowed to trim/extend/break and adds SK-02 as a dep.
  - SK-08 narrowed to splines/conics, deps SK-02 + SK-06.
  - SK-09 deps SK-01 → SK-19 (projection lives in the feature).
- Interfaces were aligned with the in-flight S00/S01 drafts (same day): `amg_core::Plane`,
  `Aabb2`, `Segment2`; S01 `Expr` (`Inline` / `Param`), `Feature::exprs_mut`,
  `Command::EditFeature { feature: FeatureBox }`, `FeatureOutput::sketch`, and the
  `EntityRef::{SketchCurve, Profile}` stubs. `SketchGeom`, `SketchEntityKey` and `ProfileKey`
  are now concrete in `amg-sketch`.
  - SK-12 deps SK-03 → SK-04, SK-19.
- SK-01, SK-02, SK-06, SK-14 and SK-21 stay independent of the ADR-007 decision.

**Roadmap phase placement (for the coordinator; roadmap not edited):**
- **Phase 1, lane B:** SK-13 and SK-14 must be in Phase 1, *before* SK-03 (Phase 1);
  otherwise SK-03 would be scheduled before a dependency. SK-15, SK-20 and SK-21 also belong
  in Phase 1 (deps all Phase 0/1).
- **Phase 2, lane B:** SK-19 (deps DOC-02/04/05 are Phase 1), SK-16, SK-17, SK-18 (with
  SK-07) and SK-22 (P0 ellipse).
- No existing WP moves earlier than its deps.

**Cross-spec dependency suggestions (other specs not edited):**
- SOL-02 should depend on SK-19, not only SK-06, since it consumes `SketchGeom`.
- UI-09 should also depend on SK-04 and SK-20.
- DOC-11 should consider SK-15's profile-ref scheme.
- BREP-03 should provide a planar-face builder from `Curve2` loops on a plane.

**Recommended dependencies (coordinator updates THIRD_PARTY.md):**

| Dependency | Licence | Purpose |
|---|---|---|
| `slvs` 0.6 | GPL-3.0 | Rust binding for the existing libslvs row. Dev/test only behind feature `slvs`; pulls `bindgen` (BSD-3-Clause) and needs libclang |
| `kcl-ezpz` | MIT | Dev-only, feature `ezpz-bench`, for SK-13 comparison |
| `smallvec` | MIT / Apache-2.0 | Small inline vecs in public types |
| `fontdb` | MIT | System font discovery for SK-10 |
| `faer` | MIT | Already listed; add `amg-sketch` as a user (sparse blocks) |
| `nalgebra` | Apache-2.0 | Already listed for the sketch solver |

**Sources:**
- github.com/solvespace/solvespace (`src/system.cpp`, `test/constraint/`)
- docs.rs/crate/slvs and github.com/thekakkun/rust_slvs
- github.com/FreeCAD/FreeCAD (`src/Mod/Sketcher/App/planegcs/`,
  `tests/src/Mod/Sketcher/App/planegcs/`)
- npm `@salusoft89/planegcs` README
- github.com/KittyCAD/ezpz and zoo.dev/blog/announcing-solver
- The papers listed above (full citations in the note §2.5)

## Interface changes

### 2026-10-20: requests from the wave-2 refinements (S02, S05, S10, S11, S12, S16, S17, S20)

Additive requests recorded by the integration pass; the owning lane confirms or adjusts them when it implements the affected WP.

- **From S02 (B-rep kernel).**
  - SK-09 now depends on BREP-17 (`plane_section`, `project_edges`) instead of BREP-09; applied in
    the WP table. SK-08's section sketch on B-rep can use `plane_section` too.
  - SOL-02/03 pass profiles through `make_planar_faces` with a `geom::Tag` per edge; the tag is
    the sketch entity key, so the profile API must expose the entity key of each profile edge.
- **From S05 (construction).** Optional: CON-03 offers `amg_features::construct::resolve` for
  sketch-plane resolution (planar faces and construction planes), which avoids duplicated
  plane-from-face code in SK-19/UI-09.
- **From S11 (UI shell).**
  - R5: `SketchEdit::SetDimensionLabel { id, pos }`.
  - Clarify that `query::*` may run on the UI thread; the "not on the UI thread" rule covers the
    solver entry points only.
  - R6: label positions are UI-only and must not change the sketch feature's cache key (see S01
    Interface changes, `Feature::key_data()`).
- **From S12 (import/export).** SK-11 uses `dxf` 0.6.2 (MIT) and `usvg` 0.48.1 (Apache-2.0 OR
  MIT) and maps `$INSUNITS` with the same table as IO-08 (`docs/notes/import-export-formats.md`).

### 2026-10-20 — SK-01 (data model, lane B; updated after review the same day)
- **Changed signatures:**
  - `TangentKind::AtEndpoint { a_end: End, b_end: End, reversed: bool }` (was
    `{ reversed }`). It names which end of each curve is tangent, so two arcs sharing both
    endpoints (a lens) are unambiguous. `End { Start, End }` now lives in `amg_sketch::model`
    (re-exported at the root, with serde) and SK-06's `profile` reuses it.
  - `Sketch::add_point`, `Sketch::add_line` and `Sketch::alloc_param` return
    `Result<_, SketchError>` and reject non-finite input with `InvalidValue` in every build.
- **Deferred to their WPs (additive later; new serde variants do not break old files):**
  `EntityKind::{Ellipse, EllipticArc}` and `DimensionKind::{EllipseMajor, EllipseMinor}`
  (SK-22); `EntityKind::{Spline, Conic}`, `SplineDef` and `ConstraintKind::Curvature`
  (SK-08). `geom::Curve2` has only `Line`/`Circle`/`Arc` for now (`Ellipse`/`BSpline` with
  SK-22/SK-08). `Curve2` methods and `geom::intersect`/`CurveHit` land with SK-06;
  `Sketch::add_constraint_oriented` with SK-02; `SketchGeom`/`ProfileKey` with SK-06/SK-15
  (they need `SolveReport`/`ProfileSet`). `SketchEntityKey = EntityId` exists now.
- **Additive:**
  - `SketchError::UnknownParam(ParamId)`; `SketchError` implements `amg_core::Diagnostic`
    and `From<amg_core::Cancelled>` (S00 error convention).
  - Ids are `#[serde(transparent)]` and have `index() -> usize`.
  - `ParamStore::{set_flags, contains, is_empty}`.
  - `EntityKind::{name, children, own_params, endpoint(End)}`, `ConstraintKind::{name,
    entities, as_dimension}`, `DimensionKind::{name, entities}`, `Sketch::point(EntityId) ->
    Option<Point2>`, `Arc2::from_center_start_end`.
- **Tangent sign conventions (for SK-02):**
  - Line–circle/arc, either argument order (an arc uses its circle): orient the line
    `start → end`, `d = end − start`, `s = cross(d, center − start) / |d|` (positive = centre
    left of the line). `External` means `s = +r` (circle left of the oriented line);
    `Internal` means `s = −r` (circle right).
  - Circle/arc–circle/arc: `External` means `|c1 − c2| = r1 + r2`; `Internal` means
    `|c1 − c2| = |r1 − r2|`.
  - `AtEndpoint`: `t_a`, `t_b` are the directions of travel at the named ends (line:
    `end − start`; arc: CCW tangent). The residual is `cross(t_a, t_b)`, and
    `reversed == false` means `dot(t_a, t_b) > 0`.
- **Behaviour the interface left open:**
  - Revisions are unique process-wide (one global counter), are not serialised, and are fresh
    after a load. Compare them only for equality. `structure_rev` changes on entity/constraint
    add/remove, `set_driving`, `alloc_param` and `compact`. `values_rev` changes on those and on
    `params_mut`, `set_dimension_value`, `set_construction` and `set_plane`.
  - **Non-finite values.** Construction rejects them. A non-finite value written later through
    `params_mut()` (for example by a diverged solve) makes *serialising* fail ("sketch
    parameter i is not finite"), so a saved sketch always loads.
  - **Load validation.** Deserialising a `Sketch` checks dangling ids, wrong kinds, parameters
    out of range or owned twice, retired slots, and id counters behind existing ids. It also
    runs cheap value checks: finite values and label positions, circle radius > 0, arc radius
    > 0, and arc end on the circle through its start within
    `Tolerance::DEFAULT.linear · max(1, r)`. The same entity checks run in `add_entity`.
    Dimension value domains are enforced on add and `set_dimension_value`; on load only
    finiteness.
  - **Removal.** `remove_entities` removes dependents (curves built on a removed point) and
    orphaned points (points of a removed curve that no surviving entity uses), then every
    constraint touching any of them. `Projected` points are never swept as orphans; removing
    one explicitly still works. It is atomic: an unknown id removes nothing.
  - **Retired slots.** Param slots of removed entities and constraints are *retired*: kept in
    the sketch (serialised as `retired`, omitted when empty), never accepted by `add_entity`
    again (`NotApplicable`), and dropped by `compact`. `compact` also drops `alloc_param` slots
    no entity or constraint owns.
  - `add_arc` projects `end` radially onto the circle through `start`, so the arc's internal
    equation holds at creation. It returns `Degenerate` when the start or end coincides with
    the centre, or when the start and end directions coincide within
    `Tolerance::DEFAULT.angular` (the sweep would be ambiguous between ~0 and 2π). `add_entity`
    still accepts an arc whose start and end coincide in position (distinct point entities):
    the SK-04 degenerate case.
  - `add_entity` with `EntityOrigin::Projected` locks the entity's own params and those of its
    child points.
  - Kind validation (`WrongEntityKind`) follows the constraint table. `Tangent::AtEndpoint`
    needs line/arc pairs with at least one arc. When it is added, the named ends must be the
    same point entity or be joined by a chain of `Coincident` constraints; otherwise it fails
    with `NotApplicable`. If the joining `Coincident` is removed later, the tangent stays, since
    its residual is defined by the stored ends. `Distance` accepts every pair except
    circle/arc–circle/arc. `Equal` accepts line–line, line–arc and circle/arc–circle/arc.
    `Symmetric` needs two entities of the same kind and a line axis. `NotApplicable` covers the
    same entity referenced twice, a point constrained onto its own curve, and a midpoint at the
    line's own endpoint. Dimension values: distances/length ≥ 0; radius, diameter and arc length
    > 0; angles finite; label position finite (`InvalidValue`).
- **For S01 / DOC (`.amg` writer, ADR-010):** a bit-identical JSON round trip needs
  `serde_json`'s `float_roundtrip` feature when parsing. `amg-sketch` enables it for its own
  tests; the document loader should enable it too.

### 2026-10-20 — SK-02 (residuals + analytic Jacobians, lane B)
- **`system::Residual` as specified, with one change:** `owners` is not `Vec<ConstraintId>`.
  Arc internal equations need an entity owner, so `EquationOwner { Entity(EntityId),
  Constraint(ConstraintId) }` is used. `x` is the whole parameter vector, indexed by
  `ParamId::index`; `jacobian` overwrites a rows x params row-major block.
- **New public API in `amg_sketch::solve`:** `EquationSystem`, `SketchResidual`,
  `SketchSystem { system, fixed, free, length_scale }`, `build_system[_scaled]`,
  `constraint_residual`, `entity_residual`, `fixed_params`, `characteristic_length`,
  `orient_constraint`, `Sketch::add_constraint_oriented`, `numeric_rank`, `dof`.
  `dof`/`numeric_rank` (full-pivot elimination) are the rank helper SK-21 uses; SK-04 replaces
  them with QR.
- **Deviation, `AtEndpoint`:** the residual is `Lc * wrap(angle(t_a, t_b) - (reversed ? pi : 0))`
  rather than the raw `cross(t_a, t_b)` from the SK-01 note. It has the same root to first
  order, but the flipped branch is not a root, so chirality is kept. Angular rows are scaled by
  the characteristic length `Lc` (Parallel, Perpendicular, AtEndpoint, Angle dimension).
- **Deviation, angle dimension:** the residual is `Lc * wrap(angle(u, w) - v)`.
  `orient_constraint` may swap `a` and `b`, because the `AngleBranch` flags alone only
  distinguish `theta` from `theta + pi`.
- **Conventions:** dimensions are `sigma * measure - v` (rows in mm). Line-line distance uses
  the signed distance of `b`'s midpoint from `a`; point-point ignores side. `Symmetric` rows:
  points 2, lines 4 (start-start, end-end), circles 3, arcs 5. Equal line-arc uses the arc
  length `r * sweep`. Degenerate input gives value 0 and a zero gradient (no NaN).
- **Driven dimensions** produce no rows; they are measured later (SK-03).

### 2026-10-20 — SK-06 (profile/region detection, lane B)
- **Landed:** `geom::{intersect, CurveHit}` and the `Curve2` methods (`param_range`, `eval`,
  `derivative`, `curvature`, `closest`, `bbox`, `reversed`, `trimmed`, `is_closed`, `length`);
  `amg_sketch::profile` (`detect_profiles`, `ProfileSet`, `Profile`, `Loop`, `ProfileEdge`,
  `VertexKey`, `ProfileId`, `ProfileFingerprint`, `ProfileIssue`, `ProfileOptions`) re-exported at
  the crate root, plus `pub type ProfileKey = ProfileId`. Curve parameters: line `[0, 1]`,
  circle `[0, 2π]` (angle), arc `[0, |sweep|]` along the direction of travel.
- **Additive (not in the Interfaces text):** `Curve2::curvature`; `Loop::{signed_area, contains,
  bbox}`; `Profile::contains`; `ProfileSet::{get, at_point}`.
- **Behaviour the interface left open:**
  - `intersect` reports an overlap of two collinear/co-circular curves as tangential hits at the
    overlap's end points (the profile code also gets an overlap flag internally and reports
    `ProfileIssue::Overlap`). A tangent contact within the tolerance is one tangential hit.
  - `ProfileSet::open_chains` also holds bridges (edges whose removal disconnects the graph and
    that bound no face on either side), not only dangling trees.
  - `Loop::signed_area` integrates the curves as stored, while `ProfileFingerprint::area` and
    the area used for ordering integrate through the welded vertices. They differ by at most
    `weld_tol · perimeter` (each vertex may sit up to the weld tolerance from a curve end);
    goldens compare the fingerprint area, and the test helper checks the difference against that
    scale-aware bound.
  - Faces whose area is at most `0.5 · weld_tol · perimeter` are dropped as slivers.
  - `ProfileId` hashes (FNV-1a, 64 bit) the sorted `(source entity, start vertex key, end vertex
    key, direction)` of all outer and hole edges. Unrelated edits and dimension changes keep it.
- **Deferred:** `SketchGeom` until `SolveReport` exists (SK-03); `ProfileSet::match_previous`
  stays with SK-15. `amg-sketch` gains `criterion` as a dev-dependency (bench
  `benches/profiles.rs`); no new third-party crates.
- **Tests and numbers:** 38 golden fixtures (`tests/profiles.rs`); 10 000 fuzz cases
  (`src/profile/tests.rs`: 7000 lattice-snapped soups with the Euler relation and a 0.5 % raster
  area oracle, 3000 generic float soups with the structural checks only, since a near-gap
  narrower than a raster cell can seal a pocket the oracle does not see). The area check allows
  0.5 % plus a raster-discretisation term (boundary cells and pinched wedges). Bench
  (`benches/profiles.rs`, 2026-10-21): 500 curves 1.02 ms (budget 5 ms), 5000 curves 16.1 ms.

### 2026-10-20 — SK-14 (solver corpus and runner, lane B)
- **New public API in `amg_sketch::corpus`** (additive; all test infrastructure):
  `Case` (JSON format 1), `CaseEntity`/`CaseGeom`, `Expected { status, also_accept, dof, solution, tolerance, groups }`,
  `Solution::{Start, Target, Witness}`, `Case::{validate, build}` -> `BuiltCase`, `Sidecar` (the `expected.json`
  of imported sets), `load_corpus`, importers `corpus::slvs::import` (SolveSpace 2D text files) and
  `corpus::ezpz::import`, the oracle `corpus::{ReferenceCheck, ReferenceSolver}`, and the runner
  `corpus::runner::{CorpusBackend, run_case, run_corpus, CorpusReport}`.
- **SK-03 / SK-13 adapter plan.** A production solver or libslvs plugs in by implementing
  `CorpusBackend { name, solve(&Case, &mut BuiltCase) -> Result<BackendOutcome, BackendSkip> }`:
  `solve` runs the solver on `built.sketch` from the start configuration; `analyze` fills `dof` and
  `groups` (constraint indices into `Case::constraints`, mapped with `BuiltCase::constraint_index`).
  A backend that reports no DOF/groups gets those checks reported as `unchecked`, not failed.
- **Corpus composition** (`testdata/sketch/corpus/`, 135 cases): SolveSpace 61, PlaneGCS 23, ezpz 27, own 24.
  SolveSpace: 33 distinct 2D cases from `test/constraint/*` plus their `_v22` re-saves (tag `format-v22`,
  28 files) as file-format coverage; the `_v20` duplicates and the empty `comment` case are not kept.
  The 3D-only cases and ezpz `massive_parallel_system` (1201 entities, too large for the dense oracle)
  are not included.
- **Deviations from the SK-14 text.**
  - PlaneGCS cases are scenario ports: the geometry and numbers are authored for Amalgam after the
    FreeCAD Sketcher test scenarios (no FreeCAD data copied verbatim); they are still attributed and
    labelled LGPL-2.1-or-later in `planegcs/NOTICE`.
  - Expected DOF/status for the imported sets come from the Amalgam reference oracle (SolveSpace's
    `test.cpp` asserts solve success only, ezpz stores no DOF) and were spot-checked by hand; own and
    PlaneGCS cases state them by hand and `tests/corpus_gen.rs` fails if the oracle disagrees.
  - `large_dimensions/perpendicular_4m.slvs` is saved unsolved, so it has no `Start` solution.
  - ezpz fully constrained cases use `Target` = the root the oracle reaches from ezpz's guesses.
  - D8 is the reduced form (24 holes on a bolt circle, 73 entities, not ~200); D9 is 341 entities.
- **Conventions SK-02 must confirm** (the oracle in `corpus/reference.rs` uses them; a mismatch shows
  up as corpus failures): signed distance positive on the left of the oriented line; point-circle
  `Distance` `Positive` = outside, `Negative` = inside, line-circle `side * s - r = v`; `Angle` is the
  signed CCW angle `atan2(cross, dot)` between the oriented directions; angular rows are scaled by the
  characteristic length; `Fix` of a circle locks only its radius (the centre is a separate point);
  a `Fix`ed arc whose three points are all fixed makes its internal equation a redundant row.
- **Dependencies.** `serde_json` (feature `float_roundtrip`) moved from dev-dependency to a normal
  dependency of `amg-sketch` for the corpus loader. The `.slvs` and ezpz importers are written for
  Amalgam; no SolveSpace or ezpz code is used. Test data licences are in `THIRD_PARTY.md`.

### 2026-10-21 - SK-15 (stable profile ids and matching, lane B)
- **Landed:** `profile::matching` with `ProfileMatch { Same, Fingerprint { new, score }, Lost }`
  (re-exported at the crate root), `ProfileSet::match_previous` and the pure
  `ProfileSet::match_fingerprint` requested by the persistent-naming note (section 12), and
  `Profile::outer_sources()`. `ProfileFingerprint` gains `outer_sources` (sorted, unique source
  ids of the outer loop only): additive field, needed for cap names (note section 5.4) and for
  scoring; no change to the `ProfileId` scheme.
- **Algorithm:** identical ids are `Same`. The remaining old profiles are assigned one-to-one,
  best score first (ties: old id, then new id), to the remaining new ones. Score = 0.3 outer-source
  Jaccard + 0.1 all-source Jaccard + 0.3 area ratio + 0.3 centroid proximity (distance over the
  old bbox diagonal); below 0.5 is `Lost`. A region redrawn with fresh entities in the same place
  scores 0.6 (rescued, TNP-13, but never exact: consumers must warn). Splits map the old id to the
  best-scoring piece; merges give the merged region to the best old region and lose the other.
- **Known limit:** a region whose bounding curves are all replaced and that also moves far
  (a circle dragged by more than about two of its diameters) is `Lost`; that is intended, since a
  fingerprint cannot distinguish it from a new region.
- **Tests** (`tests/profile_ids.rs`): 20 scripted dimension edits (size, hole position/radius,
  crossing line offset) keep every id and match `Same`; 10 topology edits (split, hole added or
  removed, disjoint curve, redrawn outer loop, vanishing lens, divider deleted, corner chopped,
  annulus to disc, split undone) check determinism and the intended successor by probe point;
  `match_fingerprint` purity; hole add changes the id but keeps `outer_sources`.

### 2026-10-21 — SK-21 (composite builders, lane B)
- **Landed:** `amg_sketch::build` (`src/build.rs`) with the 12 functions of the Interfaces
  section and `Built { entities, constraints }` (re-exported at the crate root). `Built.entities`
  lists everything created, points included, in creation order; reused entities (the `from` curve
  of `arc_tangent`) are not listed. On error the builder removes what it created.
- **Constraint sets and DOF** (whole sketch, SK-02 rank helper; tests in `tests/builders.rs`):
  `rectangle_2pt` 2 H + 2 V, DOF 4; `rectangle_3pt` 3 Perpendicular, DOF 5; `rectangle_center`
  H/V plus a construction diagonal and the centre point as its Midpoint, DOF 4; `polygon` a
  construction circle, n-1 Equal between consecutive sides and n PointOnCurve (inscribed) or n
  oriented Tangent (circumscribed), DOF 4; the three straight slots 4 AtEndpoint tangencies +
  Equal radii, DOF 5 (`slot_center_point` adds a construction centreline and a midpoint centre
  point); `slot_arc` 4 AtEndpoint tangencies, DOF 6; `circle_3pt` / `arc_3pt` no constraints
  (DOF 3 / 5, the clicked points are not kept); `circle_2_tangent` 2 Tangent + a driving Radius
  dimension, DOF 0 for the new circle; `arc_tangent` 1 AtEndpoint tangency, new arc DOF 2.
- **Choices the spec left open.** Circumscribed polygon: `vertex` is the midpoint of the first
  side, the construction circle is the incircle. `slot_center_point`: `end` is the centre of one
  end arc. `slot_overall`: `a`, `b` are the extreme points. Tangent kinds are chosen with
  `add_constraint_oriented`, so chirality matches the geometry. `circle_2_tangent` picks the
  centre nearest `near` among the offset-curve intersections and treats lines and arcs as full
  lines and circles. `arc_tangent` shares the existing endpoint entity of `from` as its start
  (CCW) or end (CW).
- **Known redundancy:** `slot_arc`'s centreline arc radius equation is implied by the caps (one
  dependent Jacobian row); SK-04 must tolerate it.

### 2026-10-21 — SK-13 (solver prototype + ADR-007 benchmark, lane B)
- **Result:** the §5.4 decision rule selects option A (own solver). Report:
  `docs/notes/sketch-solver-benchmark.md`; summary in `docs/notes/sketch-solver.md` §5.5; proposed
  ADR-007 text in `docs/QUESTIONS.md` #50 (the owner decides; ADR-007 not edited).
- **Native prototype** `amg_sketch::solve::proto` (throwaway quality; SK-03 replaces it): `solve`,
  `ProtoOptions` (`ProtoAlgorithm::{LevenbergMarquardt, DogLeg}`, substitution, analysis, diagnosis),
  `ProtoReport`/`ProtoStatus`/`ProtoGroup`, `ProtoDrag` with `DragMethod::{WeightedMinNorm, Sqp}`.
  Substitution covers coincident, H/V, concentric, circle equal-radius and fixed params; components;
  dense hand-written semidefinite Cholesky (no `nalgebra`/`faer` yet); DOF per component via
  `numeric_rank`; dependency groups by row-wise orthogonalisation of the unsubstituted Jacobian, one
  group per circuit (not merged), falling back to the unsatisfied rows.
- **Comparison harness** `amg_sketch::corpus::compare` (test infrastructure): `Contender`/`Loaded`,
  `ContenderBackend` (a `CorpusBackend`), `judge`, `metrics::{measure, markdown, csv}`; adapters
  `native`, `slvs` (feature `slvs`, GPL-3.0, test-only) and `ezpz` (feature `ezpz-bench`, MIT).
  Report tool `examples/sk13_report.rs`; criterion bench `benches/solver_compare.rs`
  (`sk13/<cold|warm|drag>/<Dn>/<option>`); acceptance tests `tests/sk13_compare.rs`.
- **Numbers (release, 200 runs):** native 10/10 decision cases, 135/135 corpus; D9 warm 0.09 ms
  median; drag p95 D4 1.3 µs, D7 27 µs; D7 jitter 66 (libslvs 106, feasible SQP 0.85). libslvs 10/10,
  118/125 corpus (10 inexpressible), 0 cases it passes and native fails. ezpz 9/10, 121/128.
- **Findings for later WPs:** SK-03 `Algorithm::Auto` = LM; the rank test is required for
  `ConvergedRedundant`. SK-05: weighted min-norm flips the D7 linkage at the toggle (as libslvs and
  ezpz do); a feasible-SQP drag (KKT step + min-norm restoration + step halving) does not, at the same
  cost. The D7 cursor path lies outside the linkage's reach (`|b − o1|` up to 16 > 12 mm), so D7
  measures behaviour at the toggle, not through it.
- **libslvs build:** the `slvs` 0.6 crate's bindgen 0.64 needs **libclang ≤ 15** (libclang 18 gives
  empty bindings and the crate does not compile); built here with the PyPI `libclang==15.0.6.1`
  wheel in a VS 2022 x64 developer environment. `amg-sketch/build.rs` links `advapi32` on Windows
  when `slvs` is on (mimalloc needs it). Sign conventions measured against the corpus:
  `PT_LINE_DISTANCE` and `PROJ_PT_DISTANCE` are the negative of ours. libslvs reports redundant but
  satisfied systems as `INCONSISTENT`; the adapter re-judges them with SK-02 residuals.
- **Dependencies:** optional `slvs` 0.6.0 (GPL-3.0; feature `slvs`) and `kcl-ezpz` 0.2.18 (MIT;
  feature `ezpz-bench`; pulls `faer` 0.23, `winnow`, `indexmap`, `libm`, `mutants`, all MIT /
  Apache-2.0). `deny.toml` gets a `GPL-3.0-only` exception for crate `slvs` (shared file; flagged).
  THIRD_PARTY.md rows updated.
- **Interface changes:** none to shared crates. Two one-line module declarations in `amg-sketch`
  (`solve::proto`, `corpus::compare`). Lane D: CI's `clippy --all-features` job will build `slvs` and
  needs libclang ≤ 15 (or must exclude the feature) — see QUESTIONS #50.
### 2026-10-22 - SK-18 (mirror, patterns, move/copy/rotate/scale, lane B)
- **Landed:** `amg_sketch::transform` (`src/transform.rs`): `mirror`, `pattern_rect`,
  `pattern_circ`, `transform`, plus `translation`, `rotation_about`, `scale_about` constructors and
  the module's own `EditResult` (same fields as the spec's; not re-exported at the crate root
  because SK-07's `edit::EditResult` exists - unify at merge, and the functions can be re-exported
  as `edit::{mirror, ..}`). Atomic: on error everything created is removed.
- **Deviation:** `transform` takes `amg_core::Transform2` (a 2D affine), not the 3D `Transform`.
  It must be rotation + translation + positive uniform scale; reflections (`NotApplicable`, use
  `mirror`), shear and non-uniform scale are rejected, zero scale is `Degenerate`.
- **Semantics.** Selection = items plus the points of selected curves. Pattern counts include the
  source (`n_a * n_b - 1` copies; circular: full turn spaces by `total/n`, otherwise `total/(n-1)`).
  A point whose image is itself (on the mirror axis, at the pattern centre) is shared, and a line or
  circle that would map onto itself is not duplicated. A mirrored arc swaps start and end.
- **Unconstrained copies** get copies of the internal constraints (all entities selected) that hold
  on the copy (dimension lengths scaled; `Fix` never; orientation re-captured by
  `orient_constraint`; residual checked). **In-place** move/rotate/scale scales internal length
  dimensions and circle radii, and removes (and reports) internal constraints that held before and
  no longer hold (H/V after a rotation). Constraints reaching outside the selection are kept.
  Locked/fixed points give `Locked` before anything changes.
- **Constrained results use existing constraint kinds** (no new kind): mirror = `Symmetric` per
  point pair + `Equal` per circle (+ `PointOnCurve` for shared axis points); rect pattern = H/V
  distance dimensions per point (Vertical/Horizontal where the offset is zero) + `Equal`; circular
  pattern = construction lines centre-point with `Equal` + `Angle` dimension per point + `Equal`.
  DOF of the whole sketch is unchanged by a constrained result (tests, SK-02 rank helper); the
  prototype solver re-solve test moves the copies with the source.
- **Leftovers:** pattern spacing is per-point dimensions (cluttered; a dedicated pattern
  constraint/offset parameter would need an SK-01/02 kind). Axis points already implied on the axis
  give a redundant `PointOnCurve`. No benchmark: the spec sets no budget for these ops.


### 2026-10-22 — SK-07 (trim, extend, break, lane B)
- **Landed:** `amg_sketch::edit` (`src/edit/{mod,cuts,rebuild}.rs`) with `trim`, `extend`,
  `break_at` and `EditResult` (re-exported at the crate root), plus `trim_detailed` /
  `extend_detailed` / `break_at_detailed` returning an `EditOutcome` with a `MigrationReport`
  (`migrated` old→new constraint ids, `dropped` with kind and reason, `dropped_dimensions()`).
  `edit/mod.rs` and `EditResult` are shared with SK-16..18; the coordinator resolves the merge.
- **Model.** Editing replaces the curve by new entities (ids change) and re-creates the
  constraints that still hold; the sketch is restored if anything fails. Trim removes the part
  between the two intersections nearest the pick (everything if there are none; a circle needs
  two and becomes one arc). Extend lengthens the end nearest the pick to the nearest curve in
  the way (lines straight, arcs along the circle, never past their other end; circles are
  `NotApplicable`). Break splits a line/arc into two pieces sharing the new point (circles are
  `NotApplicable`, an end point is `Degenerate`). New points are pinned with `PointOnCurve`
  on the cutter, or `Coincident` with a cutter end point lying exactly there.
- **Migration rules** (table in `rebuild.rs`): curve-level constraints (H/V, parallel,
  perpendicular, collinear, tangent to a circle, concentric, radius, diameter, distance,
  angle, symmetric axis) go to the first piece; a second piece is tied to it with `Collinear`
  (lines), `Equal` radius (arcs) or `Parallel` (break, shared joint), which keeps the DOF.
  `AtEndpoint` tangents follow the piece that still owns that original end, else are dropped;
  `PointOnCurve` goes to the piece containing the point; `Fix` goes to every piece. Dropped and
  reported: Length, ArcLength, Midpoint, Equal-length, mirrored operands, anything on a point
  removed with the curve.
- **Solver path:** residual 0 via `build_system`/`max_abs_residual` (SK-02) and DOF via the
  SK-02 `dof` rank helper; each fixture also runs `solve::proto::solve` (SK-13 prototype; SK-03
  is not built) and requires `Converged` with no parameter movement.
- **Tests:** `tests/edit_trim.rs`, 63 fixtures (trim 33, extend 15, break 10, errors 5).
- **Choices:** tangent contacts count as cuts; overlapping (collinear/co-circular) curves are
  ignored as cutters; construction curves do cut.

### 2026-10-23 - SK-17 (offset, lane B)
- **Landed:** `amg_sketch::offset` (`src/offset.rs`): `offset`, `offset_with(.., &OffsetOptions)`,
  `set_offset_distance`, `CornerStyle { Extend, Round }`, `OffsetOptions { corner, miter_limit }`,
  `OffsetResult { created, removed, constraints_added, constraints_removed, copies, dimensions }`
  (re-exported at the root except the functions). **Deviation:** the module is `offset`, not
  `edit`, and the result is `OffsetResult`, not `EditResult`: `edit::EditResult` is introduced by
  SK-07 in parallel. The merge should add `edit::offset` as a re-export and convert the result.
- **Solve helpers used:** the SK-02 residual/rank helpers (`build_system`, `dof`,
  `Sketch::add_constraint_oriented`) for chirality and in tests, and `solve::proto::solve`
  (SK-13 prototype) for the dimension-edit tests. SK-03 is not built yet; swap when it lands.
- **Semantics:** `chain` is any set of line/arc/circle entities; each connected simple chain
  (open or closed, ordered by coincident end points, shared or not) or circle is offset on its own.
  `side` picks the side of the chain's travel direction at the chain point nearest to it (a point
  nearest to a vertex is ambiguous: pick one near the middle of an edge). Outer corners are closed
  by extension (miter; falls back to a round arc when carriers miss or the miter exceeds
  `miter_limit * distance`, default 10) or by an arc about the original corner (`Round`).
- **Trimming:** raw offsets are split at all mutual intersections and a piece is kept only if every
  sample is at least `distance` from the source chain; duplicates merge, end points weld (5e-7).
  Result: self-intersecting loops (inner corners, vanished tabs, short steps) disappear. An empty
  result is `Degenerate`. Collinear pieces produced by splitting are not re-merged.
- **Constraints (`constrain = true`):** no "equal distance" constraint exists in the model, so each
  copy piece gets its own driving `Distance` dimension, all with the same value, listed in
  `OffsetResult::dimensions`; `set_offset_distance` edits them together (SK-19 should bind them to
  one parameter). Per piece: line = Parallel + line-line Distance; arc = Concentric + Distance from
  its start point to the source arc; circle = Concentric + a construction point (PointOnCurve,
  Horizontal, Distance); round corner arc = centred on the source corner point, AtEndpoint tangents
  to its neighbours. Open chain ends and smooth (tangent/collinear) junctions get a construction
  line (perpendicular to a source line, or the radius line of a source arc with PointOnCurve).
  With the source points fixed, the copy has DOF 0 and zero residual in every constrained fixture.
- **Known redundancy:** round corners and smooth junctions add consistent but dependent rows (arc
  internal equations, junction feet); SK-04 must tolerate them. Topology is fixed at creation: an
  edit that makes a short element vanish or appear is not tracked.
- **Tests** (`tests/offset.rs`, 29): closed rect (extend, round, inward, thin, collapsed), L-shape
  areas, tab trimmed, stadium, sector, circle; open polyline outer/inner corner, step chain with
  trimmed loop, tangent arc chain, arc collapse, round and miter-limit; two chains; errors;
  determinism; constrained rect/inward/round/open/tangent-arc/trimmed/stadium/circle edits solved
  with `solve::proto`; copy follows a moved source vs independent unconstrained copy. No budget in
  the vision doc for offset, so no bench.

### 2026-10-23 - SK-16 (fillet and chamfer, lane B)
- **Landed:** `amg_sketch::edit::{fillet, chamfer}` and `fillet_detailed` / `chamfer_detailed`
  (return SK-07's `EditOutcome`) in `src/edit/corner.rs`; `ChamferSpec` (`Equal`, `TwoDistances`,
  `DistanceAngle`) is re-exported at the crate root with `EditResult`. Signatures are the spec's.
  Atomic: on error the sketch is restored exactly (tests compare serialised sketches).
- **Trimming** reuses SK-07's `rebuild::apply` (so the migration table there applies): each curve
  is replaced by one piece whose cut boundary is the new fillet/chamfer point itself (`Bnd::Keep`),
  so the pieces and the new arc/line **share** points and no `Coincident` is needed. Created and
  removed ids are the sketch diff; the migration report chains the two edits (old -> new).
- **Fillet:** centres come from the intersections of the offset curves (line: two parallels; arc:
  circles of radius R+r and |R-r|); tangent points must lie on the actual curves and each curve
  must keep a part of positive length, else `NotApplicable` (radius too large, parallel curves,
  curves that do not reach). `near` selects the candidate arc nearest to it. Constraints: two
  `Tangent` `AtEndpoint` (chirality via `add_constraint_oriented`) and a driving `Radius`.
- **Chamfer:** corner = intersection of the carriers (nearest chord to `near` wins). Distances are
  straight-line distances from the corner. Constraints: a construction point at the old corner,
  `PointOnCurve` on both pieces, construction legs corner->cut points; `Equal(d)` = `Equal` legs +
  one `Length`; `TwoDistances` = a `Length` per leg; `DistanceAngle(d, a)` = `Length` on leg A +
  an `Angle` (chamfer line vs leg A, the interior angle at the cut point, `(0, pi)`).
- **DOF (SK-02 `dof`, whole sketch):** a joined corner keeps its DOF for every variant: 2 lines
  6 -> 6, rectangle corner 4 -> 4 (all four corners too), line-arc 7 -> 7, arc-arc 8 -> 8. Curves
  that were not joined lose the two DOF of their free ends: disjoint lines 8 -> 6, separate arcs
  10 -> 8. With the far ends fixed and H/V on the lines, DOF is 0 before and after, and editing the
  radius/length dimension re-solves (`solve::proto`) to the new geometry.
- **Choices:** circles are `NotApplicable`; the kept side is the one away from the corner, so for
  a T or X junction the part of a through curve on the corner side of the cut is removed
  (documented in the module). Dimensions on the trimmed curves that depend on length are dropped
  and reported (SK-07 rules). The corner point of a chamfer survives as a construction point.
- **Tests:** `tests/edit_corner.rs`, 32 fixtures (fillet 17, chamfer 13, result lists 1,
  incl. line-line/line-arc/arc-arc, shared and disjoint corners, X junction, rectangle corners,
  `NotApplicable` and input errors with the sketch unchanged). Each runs the residual check and
  the SK-13 prototype solver (no movement). SK-03 is not built; swap when it lands.
- **Interface changes:** none to other lanes. `edit/mod.rs` gains a `mod corner` and re-exports.
- **Leftovers:** chamfer distances on arcs are chords, not arc lengths; a fillet between a curve
  and a circle (full) is rejected; no fillet-of-a-chain / multi-corner command (UI loops).

### 2026-10-23 - SK-22 (ellipse and elliptical arc, lane B)
- **Landed:** `EntityKind::{Ellipse, EllipticArc}` (as in Interfaces), `DimensionKind::{EllipseMajor,
  EllipseMinor}`, `geom::EllipseArc2` + `Curve2::Ellipse` (re-exported at the root),
  `Sketch::{add_ellipse, add_elliptic_arc}`, `solve::constraint_residual_aux`. `Curve2` gains
  `curvature_at(t)`; `curvature()` of an ellipse is the curvature at the start.
- **Conventions.** `P(t) = c + m cos t + perp(m)(b/|m|) sin t`, `m = major - center`, `t` the eccentric
  angle; the elliptical arc runs CCW from `t0` to `t0 + ((t1 - t0) mod 2 pi)`. `b` may exceed `|m|`
  (the "major" point only names the axis the dimension measures). Curve parameter of `Curve2::Ellipse`
  is the angle travelled from the start, `[0, |t1 - t0|]`; `t1 < t0` is CW, span `2 pi` a full ellipse.
- **Equations.** Elliptical arc: `s - P(t0)`, `e - P(t1)` (4 rows, `EquationOwner::Entity`). Point on
  ellipse/elliptical arc (the full ellipse): `p - P(t)` (2 rows) with `t` the constraint's one aux param,
  allocated by `Sketch::add_constraint` (start value: nearest ellipse point); load validation requires
  exactly that aux count. Tangent line-ellipse: `s -/+ h`, `s` signed centre distance, `h` the half-width
  of the ellipse normal to the line (`h^2 = b^2 + (1 - b^2/a^2) q^2`), either argument order, External /
  Internal chosen by `orient`. Concentric accepts ellipses. `Equal` ellipse-ellipse (and arcs): both axes
  (2 rows, `|M-c|` and `b`). `Symmetric` of full ellipses: centres, major ends, `b` (5 rows).
  Axis dimensions `|M - c| - v` and `b - v` (values must be > 0).
- **Not supported (clear errors, never silent):** tangent circle/arc-ellipse and ellipse-ellipse, tangent
  `AtEndpoint` with elliptical arcs (`WrongEntityKind`), distance dimensions to ellipses, `Symmetric` of
  elliptical arcs, `Radius`/`Diameter`/`ArcLength` on ellipses (`WrongEntityKind`). `circle_2_tangent`,
  `offset` (offset of an ellipse is not an ellipse) and trim/extend/break *of* an ellipse return
  `NotApplicable` / `WrongEntityKind`. An ellipse is a valid *cutter* and extension target for lines,
  arcs and circles (tested). The corpus reference oracle and the slvs/ezpz adapters decline ellipses.
- **Intersections** (`geom::ellipse`): ellipse vs line, circle/arc and ellipse by root finding of the
  trigonometric polynomial `F(P(t))` on monotone pieces between the roots of its derivative; extrema within
  tolerance are tangential hits; end-point touches as for the other curves; same-conic pairs report an
  overlap and only their ends. Profile detection treats an ellipse like a `2 pi` arc (seam end-point
  vertex); areas and moments are exact (affine image of the circle segment), containment bisects the
  monotone pieces. Half-edge ordering uses the curvature at each end.
- **Transform (SK-18):** ellipses and elliptical arcs mirror (a reflection reverses the eccentric angle:
  image `t0 = -t1`, `t1 = -t0`), copy, pattern, rotate and scale (minor radius scaled); constrained results
  use the existing kinds (`Symmetric` per point, `Equal` per ellipse). Known redundancy: the image's
  `|M - c|` row and its end points are implied by the point symmetries; SK-04 must tolerate it.
- **Tests:** `tests/ellipse.rs` (28) and `geom::ellipse::tests` (13): FD Jacobians of every ellipse row on
  120 random configurations (~2500 rows), consistency of the equations, ellipse-in-rectangle (tangent x4)
  solve and edit, axis dimensions and point-on-ellipse solve, profile fixtures (hole area, segment, lens,
  inscribed ellipse, elliptical-arc D shapes, annulus; relative error 1e-9), validation and serde,
  trim/extend/offset/mirror/pattern/transform ellipse paths.
- **Leftovers:** trim/extend/break and offset of ellipses; ellipse tangent to circles or ellipses; tangent
  `AtEndpoint` for elliptical arcs; symmetric elliptical arcs; distance to an ellipse; `Curve2::BSpline`
  still with SK-08.

### 2026-10-23 - SK-08 (splines and conics, lane B)
- **Landed:** `EntityKind::{Spline(SplineDef), Conic}`, `SplineDef::{Control, Fit}`, `ConstraintKind::Curvature`
  (model), `geom::BSpline2` + `Curve2::BSpline`, `Sketch::{add_spline, add_spline_fit, add_conic}`,
  `geom::{fit_spline, FitMap, clamped_uniform_knots, check_knots, chord_params, conic_weight}`,
  `offset::OffsetOptions::spline_tol`, `solve/spline.rs` (rows with variable arity; `SketchResidual`
  wraps them), `geom/spline_x.rs` (intersectors). `BSpline2` is a clamped rational B-spline of degree
  1..=10, domain shifted to `[0, L]`; every conic is the same type.
- **Interface changes (additive, all in `amg-sketch`):** `SplineDef::Fit` gains `params` (frozen chord-length
  parameters, so the pole map stays linear); `Curvature` carries `a_end, b_end, reversed` like
  `TangentKind::AtEndpoint` (a bare `{a, b}` cannot name the shared ends); `Tangent AtEndpoint` accepts
  line/arc/spline/conic pairs (line-line is still rejected); `PointOnCurve` accepts splines and conics with
  one aux param (curve parameter); `Equal` accepts conic-conic (`rho`) and rational control spline pairs of
  equal pole count (weights); `Curve2` gains a variant, so downstream exhaustive matches need an arm.
- **Equations.** Poles are a linear map of point params (identity for control splines/conics, the interpolation
  map of `FitMap` for fit splines); weights are params, `rho/(1-rho)` for a conic, or 1. Point on spline:
  `p - C(t)` (2 rows, `t` aux; the curve continues polynomially beyond `[0, L]` so Newton can recover).
  Tangent: `L_c * wrap(angle(t_a, s t_b))` with direction `P1 - P0` at a start and `Pn - Pn-1` at an end.
  G2: that row plus `0.5 L_c^2 (k_a - s k_b)` (`s = -1` if reversed; the scale is the deviation at distance
  `L_c` of two curves whose curvatures differ). The end curvature depends only on the first three homogeneous
  poles; its gradient is exact (forward-mode dual numbers in `geom::spline`), then the pole map.
  Natural fit-spline ends have `C'' = 0`, i.e. zero curvature there: G2 with a fit spline needs a tangent handle.
- **Intersections** (deviation from "Bezier clipping"): spline vs line/circle/arc/ellipse by the implicit
  function along the knot spans (sample grid, bisection of `(F o C)'` and `F o C`); spline vs spline by
  control-polygon box subdivision of the Bezier pieces (hull property) and a Levenberg-Marquardt polish on the
  exact curves, grazing contacts accepted within the tolerance. Overlap = a run of samples on the other curve;
  only the run ends are reported. Self-intersection: 12 pieces per span intersected pairwise (a loop shorter
  than 1/12 of a span is missed); the closing point of a closed curve is not reported. Profile detection
  turns a self-crossing into a vertex and the loop into an edge, with `ProfileIssue::SelfIntersection`.
- **Profiles.** Area and first moment use the exact segment between curve and chord (Gauss-Legendre, 4 panels
  per span for rational curves); containment finds y-monotone pieces and bisects.
- **Not supported (clear errors, tested):** periodic splines (`NotApplicable`, also on load), tangent
  External/Internal with splines, curvature without a spline/conic, `Concentric`, `Symmetric`, `Midpoint`,
  `Parallel`, distance/radius/length dimensions on splines or conics, `Equal` of other combinations, trim /
  extend / break / fillet / chamfer *of* a spline (`WrongEntityKind`; a spline is a valid cutter), `circle_2_tangent`
  to a spline, corpus reference/ezpz/slvs adapters (skip). **Offset:** a selection of splines/conics only; each becomes
  a new unconstrained fit spline within `spline_tol` (default 1e-4 mm, measured against the source), refused when
  the distance reaches the radius of curvature; mixing with lines/arcs or `constrain = true` is `NotApplicable`.
  **Transform:** copy/move/rotate/scale/mirror/patterns work; constrained results use point constraints plus
  `Equal` for weights/`rho`.
- **Tests:** `src/geom/spline_tests.rs` (8), `tests/spline.rs` (16), `tests/spline_ops.rs` (4).
- **Leftovers:** periodic splines; spline trim/extend/break and constrained/chain offset with corner closing;
  rho/weight dimensions; G2 helper for fit splines with natural ends; slvs adapter untested (cannot build here).

### 2026-10-23 - SK-22 / SK-16 integration (merge review)
- SK-22 branched before SK-16 landed; `edit::corner` gains `Curve2::Ellipse` arms. Fillet and chamfer
  reject ellipses and elliptical arcs with `WrongEntityKind` (from the shared `edit::checked`, as trim
  does); test `fillet_with_ellipse_is_wrong_kind`.

### 2026-10-23 - ADR-007 accepted (owner, QUESTIONS #50)
- The owner accepted the SK-13 result: ADR-007 is Accepted (own Rust solver; libslvs as test oracle
  behind `slvs`; ezpz as benchmark reference behind `ezpz-bench`). SK-13 is Done and the SK-03 gate is
  cleared.

### 2026-10-23 - SK-11 (DXF/SVG import, lane B)
- **Landed:** `amg_sketch::import` (`src/import/{mod,dxf_in,svg_in}.rs`): `import_dxf(sketch, bytes, &ImportOptions)`,
  `import_dxf_file`, `import_svg(sketch, &str, &ImportOptions)`, `import_svg_file`, each returning an `ImportReport`
  (`entities`, `constraints`, `layers`, `unit: UnitInfo {source, declared_code, scale_to_mm}`, `bbox` in mm, `merged`,
  `snapped`, `skipped` counts by kind, `warnings`). Errors: `ImportError` (`Dxf`, `Svg`, `Io`, `Options`, `Sketch`).
  Every curve gets `EntityOrigin::Imported`. Parsers: `dxf` 0.6.2 (MIT) and `usvg` 0.48.1 (`default-features = false`,
  Apache-2.0 OR MIT); transitive crates are permissive (ADR-012; THIRD_PARTY.md updated).
- **Units/scale.** DXF: `$INSUNITS` through the IO-08 table (codes 1-24 as mm factors); 0/unknown = mm
  (`UnitSource::AssumedMm`); `ImportOptions::unit` overrides, `scale` multiplies on top. SVG: usvg resolves
  `width/height/viewBox`/units to CSS px; px to mm is `25.4 / svg_dpi` (96); an override reads px as the given unit.
  Nothing is scaled silently; the choice is reported. SVG y is flipped as `H - y` (document height).
- **Layers.** DXF layer names / SVG outermost named `<g id>`. `construction_layers` (ASCII case-insensitive) set
  `construction`, `skip_layers` drop geometry, DXF layers that are off or have a negative colour become construction
  (`hidden_layers_construction`, default on).
- **Endpoint merge.** Free ends within `merge_tol` (default 1e-4 mm) are clustered (single linkage, grid hash), snapped
  to a representative before the entities are built (an elliptical-arc end if present, else the first) and tied with
  `Coincident` constraints, so profiles close at once. Bigger gaps stay (QUESTIONS #17: warn only); opt-in heal is
  `merge_tol = 1e-2`. Lines shorter than the tolerance are dropped (`skipped["degenerate"]`).
- **DXF mapping.** LINE, CIRCLE, ARC (a -Z extrusion mirrors x and flips the sense), ELLIPSE (full: `Ellipse`; partial:
  `EllipticArc`), LWPOLYLINE and POLYLINE/VERTEX (bulge to `Arc`, closed flag), SPLINE (clamped control-point form:
  `SplineDef::Control` with weights and knots; fit-point form: `SplineDef::Fit` with tangent handles; unclamped or
  periodic: sampled by de Boor into a fit spline, with a warning). TEXT, HATCH, DIMENSION, INSERT and tilted extrusions
  are skipped and counted.
- **SVG mapping.** usvg flattens transforms, shapes and arcs to lines, quadratics and cubics. Closed four-cubic rings
  that are affine circles become `Circle` or `Ellipse` (exact for rotated/skewed circles); a cubic that is a circular
  arc (3.5e-4 r / 5e-4 chord deviation) becomes `Arc`; other cubics and quadratics become degree 3/2 `Control` splines.
- **Tests:** `tests/import.rs` (16) with 10 fixtures in `tests/fixtures/import/` (5 DXF: R12, R2000 x3, R2010; 5 SVG, 3
  with group/matrix/rotate transforms) and `src/import/tests.rs` (4). Done-when 1: every fixture's bbox is within 1e-6
  relative of the analytic value. Done-when 2: profile counts and areas match (1e-9 DXF, 1e-5 SVG, whose coordinates pass
  through f32 in usvg).
- **Deviations / leftovers:** `INSERT` blocks, HATCH, text and dimensions are not imported; POLYLINE spline-fit flags
  import the control polygon; SVG arcs wider than 90 deg arrive as several `Arc` entities, SVG partial ellipses become
  splines, consecutive Beziers are not merged into one spline; no progress/cancel hook; the import dialog (unit
  suggestion) is S11/S12 work on top of `ImportReport`. The fixtures are small hand-sized ASCII files written with a
  throw-away generator script; redistributable.
