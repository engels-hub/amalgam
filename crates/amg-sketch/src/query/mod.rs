//! Picking and snap inference for the sketch UI (SK-20, S11 UI-09).
//!
//! Everything here is pure and read-only and fits the UI thread (budget: `pick` +
//! `snap_candidates` < 0.1 ms on a 500-entity sketch).
//!
//! - [`pick`]: entities within a radius of the cursor, nearest first.
//! - [`snap_candidates`]: [`SnapKind`] candidates (end/centre/mid/quadrant/on-curve/
//!   intersection points, horizontal and vertical alignment, and, with a rubber-band anchor
//!   `from`, tangent and perpendicular points), each with the constraint the tool would add.
//! - [`suppress_redundant`] / [`snap_candidates_checked`]: drop what
//!   [`SolverBackend::check_constraint`] says would over-constrain the sketch. They run the
//!   solver and are not part of the 0.1 ms budget.
//!
//! **Index.** The functions take a [`Sketch`], build a [`QueryIndex`] (evaluated curves and
//! their boxes in flat arrays) and cache it per thread keyed by the sketch revisions, so
//! hovering over an unchanged sketch does not re-evaluate it. A caller that wants to own the
//! cache holds a [`QueryIndex`] itself.
//!
//! **Suggested constraints.** The geometry a tool is about to create does not exist yet, so a
//! suggestion names it with the placeholder [`NEW_ENTITY`]: the new *point* for point snaps and
//! alignments, the new *line* (rubber-band, `from` → candidate) for tangent and perpendicular.
//! [`SnapOptions::moving`] / [`rubber_band`](SnapOptions::rubber_band) name existing entities
//! instead (the point being dragged, the line being drawn); the suggestions then reference
//! them and can be checked by [`suppress_redundant`]. [`SnapCandidate::constraints`] resolves
//! the placeholders.
//!
//! All curve kinds are covered: lines, circles, arcs, ellipses and elliptical arcs (SK-22),
//! splines and conics (SK-08). Splines have no quadrant points and, being tangent only at
//! shared ends, no tangent suggestion.

use std::cell::RefCell;
use std::rc::Rc;

use amg_core::Point2;
use smallvec::SmallVec;

use crate::ids::EntityId;
use crate::model::{ConstraintKind, HvTarget, Sketch};
use crate::solve::{ConstraintCheck, SolverBackend};

mod index;
mod snap;

pub use index::QueryIndex;

/// Placeholder in suggested constraints for the entity the tool is about to create.
pub const NEW_ENTITY: EntityId = EntityId(u32::MAX);

/// What [`pick`] considers.
#[derive(Copy, Clone, Debug, Default)]
pub struct PickFilter {
    /// Point entities.
    pub points: bool,
    /// Curve entities.
    pub curves: bool,
    /// Construction geometry.
    pub construction: bool,
    /// Entities projected from 3D geometry.
    pub projected: bool,
}

impl PickFilter {
    /// Everything pickable.
    pub const ALL: PickFilter = PickFilter {
        points: true,
        curves: true,
        construction: true,
        projected: true,
    };
}

/// One entity found by [`pick`].
#[derive(Copy, Clone, Debug)]
pub struct PickHit {
    /// The entity.
    pub entity: EntityId,
    /// Distance from the cursor to the entity, mm.
    pub distance: f64,
    /// Curve parameter of the closest point (see [`Curve2::eval`](crate::Curve2::eval)); 0 for
    /// points.
    pub param: f64,
}

/// Kind of a snap.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SnapKind {
    /// A point entity (end of a curve, or a free point).
    Endpoint,
    /// Midpoint of a line (arc length midpoint of an arc, spline or conic).
    Midpoint,
    /// Centre of a circle, arc or ellipse.
    Center,
    /// Axis extreme of a circle, arc or ellipse (0, 90, 180, 270 degrees, or the ellipse axis
    /// ends).
    Quadrant,
    /// Nearest point of a curve.
    OnCurve,
    /// Crossing of two curves.
    Intersection,
    /// Same y as another point (or the rubber-band anchor).
    AlignHorizontal,
    /// Same x as another point (or the rubber-band anchor).
    AlignVertical,
    /// A rubber-band line from the anchor would be tangent to the curve here.
    Tangent,
    /// A rubber-band line from the anchor would be perpendicular to the curve here.
    Perpendicular,
}

/// One snap target.
#[derive(Clone, Debug)]
pub struct SnapCandidate {
    /// Kind.
    pub kind: SnapKind,
    /// Snapped position, sketch coordinates.
    pub point: Point2,
    /// Entities the snap refers to (the point, the curve(s), or the aligned point).
    pub entities: SmallVec<[EntityId; 2]>,
    /// Constraint to add when the snap is used (see the module docs for the placeholder).
    pub suggested: Option<ConstraintKind>,
    /// A second constraint: an intersection also lies on its second curve.
    pub secondary: Option<ConstraintKind>,
}

impl SnapCandidate {
    /// The suggested constraints with [`NEW_ENTITY`] replaced: `point` for the new point,
    /// `line` for the new rubber-band line.
    pub fn constraints(&self, point: EntityId, line: EntityId) -> SmallVec<[ConstraintKind; 2]> {
        [&self.suggested, &self.secondary]
            .into_iter()
            .flatten()
            .map(|k| resolve(k, point, line))
            .collect()
    }
}

/// Context of a snap query.
#[derive(Copy, Clone, Debug, Default)]
pub struct SnapOptions<'a> {
    /// Rubber-band anchor: enables alignment with it and tangent / perpendicular snaps.
    pub from: Option<Point2>,
    /// The point entity at `from`, if known (found by position otherwise).
    pub from_entity: Option<EntityId>,
    /// The existing point entity that is being placed or dragged: never a snap target, and
    /// used instead of [`NEW_ENTITY`] for the new point in suggestions.
    pub moving: Option<EntityId>,
    /// The existing line being drawn from `from`: never a snap target, and used instead of
    /// [`NEW_ENTITY`] for the new line in tangent / perpendicular suggestions.
    pub rubber_band: Option<EntityId>,
    /// More entities that are not snap targets.
    pub exclude: &'a [EntityId],
}

fn sub(e: EntityId, point: EntityId, line: EntityId, is_line: bool) -> EntityId {
    if e == NEW_ENTITY {
        if is_line { line } else { point }
    } else {
        e
    }
}

fn resolve(k: &ConstraintKind, point: EntityId, line: EntityId) -> ConstraintKind {
    use ConstraintKind as K;
    let p = |e| sub(e, point, line, false);
    let l = |e| sub(e, point, line, true);
    match *k {
        K::Coincident { a, b } => K::Coincident { a: p(a), b: p(b) },
        K::PointOnCurve { point: q, curve } => K::PointOnCurve { point: p(q), curve },
        K::Midpoint { point: q, line: m } => K::Midpoint {
            point: p(q),
            line: m,
        },
        K::Horizontal(HvTarget::Points(a, b)) => K::Horizontal(HvTarget::Points(p(a), p(b))),
        K::Vertical(HvTarget::Points(a, b)) => K::Vertical(HvTarget::Points(p(a), p(b))),
        K::Perpendicular { a, b } => K::Perpendicular { a: l(a), b },
        K::Tangent { a, b, kind } => K::Tangent { a: l(a), b, kind },
        ref other => other.clone(),
    }
}

thread_local! {
    static CACHE: RefCell<Option<Rc<QueryIndex>>> = const { RefCell::new(None) };
}

/// The cached index of `sketch` on this thread (rebuilt when the sketch changed).
fn cached(sketch: &Sketch) -> Rc<QueryIndex> {
    CACHE.with(|c| {
        let mut c = c.borrow_mut();
        match &*c {
            Some(ix) if ix.matches(sketch) => Rc::clone(ix),
            _ => {
                let ix = Rc::new(QueryIndex::new(sketch));
                *c = Some(Rc::clone(&ix));
                ix
            }
        }
    })
}

/// Entities within `radius` mm of `p`, sorted by distance (ties: points before curves).
pub fn pick(sketch: &Sketch, p: Point2, radius: f64, filter: PickFilter) -> Vec<PickHit> {
    cached(sketch).pick(p, radius, filter)
}

/// Snap candidates within `radius` mm of `p`, best first: by kind (endpoint, centre, midpoint,
/// intersection, quadrant, tangent, perpendicular, on-curve, alignment), then distance.
/// `from` is the rubber-band anchor. Suggestions use the [`NEW_ENTITY`] placeholder; see
/// [`snap_candidates_with`] for the options.
pub fn snap_candidates(
    sketch: &Sketch,
    p: Point2,
    radius: f64,
    from: Option<Point2>,
) -> Vec<SnapCandidate> {
    snap_candidates_with(
        sketch,
        p,
        radius,
        &SnapOptions {
            from,
            ..SnapOptions::default()
        },
    )
}

/// [`snap_candidates`] with the full [`SnapOptions`].
pub fn snap_candidates_with(
    sketch: &Sketch,
    p: Point2,
    radius: f64,
    opts: &SnapOptions<'_>,
) -> Vec<SnapCandidate> {
    cached(sketch).snap_candidates(p, radius, opts)
}

/// Applies [`SolverBackend::check_constraint`] to the suggestions of `candidates`.
///
/// - A suggestion that is `Redundant` (or invalid) is cleared: the position stays a valid
///   snap, there is just nothing to add.
/// - A candidate with a `Conflicting` suggestion is removed.
///
/// Suggestions that name [`NEW_ENTITY`] are kept as they are: a constraint on a new entity
/// removes freedom by construction. Pass [`SnapOptions::moving`] / `rubber_band` to check
/// suggestions for an entity that already exists (a drag, or a tool that creates its
/// geometry first).
pub fn suppress_redundant(
    sketch: &Sketch,
    solver: &mut dyn SolverBackend,
    candidates: Vec<SnapCandidate>,
) -> Vec<SnapCandidate> {
    let mut verdict = |k: &mut Option<ConstraintKind>| -> bool {
        let Some(kind) = k.as_ref() else { return true };
        if kind.entities().contains(&NEW_ENTITY) {
            return true;
        }
        match solver.check_constraint(sketch, kind) {
            Ok(ConstraintCheck::Independent) => true,
            Ok(ConstraintCheck::Conflicting { .. }) => false,
            Ok(ConstraintCheck::Redundant { .. }) | Err(_) => {
                *k = None;
                true
            }
        }
    };
    let mut out = Vec::with_capacity(candidates.len());
    for mut c in candidates {
        if verdict(&mut c.suggested) && verdict(&mut c.secondary) {
            if c.suggested.is_none() {
                c.suggested = c.secondary.take();
            }
            out.push(c);
        }
    }
    out
}

/// [`snap_candidates_with`] followed by [`suppress_redundant`].
pub fn snap_candidates_checked(
    sketch: &Sketch,
    solver: &mut dyn SolverBackend,
    p: Point2,
    radius: f64,
    opts: &SnapOptions<'_>,
) -> Vec<SnapCandidate> {
    let c = snap_candidates_with(sketch, p, radius, opts);
    suppress_redundant(sketch, solver, c)
}
