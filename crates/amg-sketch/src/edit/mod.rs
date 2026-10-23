//! Sketch editing operations: trim, extend and break (SK-07).
//!
//! Each operation replaces the edited curve by new pieces (new entity ids), keeps the
//! constraints that still hold on the pieces and reports the ones that cannot (see the
//! migration table in `rebuild`). On error the sketch is left exactly as it was.
//!
//! - [`trim`]: removes the part of a line/arc/circle under the pick point between its two
//!   nearest intersections with other curves (all of it if there are none).
//! - [`extend`]: lengthens the end of a line/arc nearest to the pick up to the nearest curve
//!   it would meet.
//! - [`break_at`]: splits a line/arc at a point into two pieces that share the new point.
//!
//! - [`fillet`] / [`chamfer`] (SK-16, in `corner`): round or cut the corner of two curves,
//!   trimming both with the same machinery and adding tangent/coincident constraints and a
//!   radius or length dimension.
//!
//! The `*_detailed` variants also return the [`MigrationReport`].

use std::f64::consts::TAU;

use amg_core::Point2;

use crate::error::SketchError;
use crate::ids::{ConstraintId, EntityId};
use crate::model::{ConstraintKind, End, EntityKind, Sketch};

mod corner;
mod cuts;
mod rebuild;

pub use corner::{ChamferSpec, chamfer, chamfer_detailed, fillet, fillet_detailed};

use cuts::{MERGE, cuts as find_cuts, extension_target, is_cuttable, place};
use rebuild::{Bnd, Link, NewPt, PieceSpec, Plan, apply, editable};

/// What an edit changed.
#[derive(Clone, Debug, Default)]
pub struct EditResult {
    /// Entities created (new points and new pieces).
    pub created: Vec<EntityId>,
    /// Entities removed (the edited curve and the points only it used).
    pub removed: Vec<EntityId>,
    /// Constraints created (new pins and links, and migrated constraints).
    pub constraints_added: Vec<ConstraintId>,
    /// Constraints removed (everything that touched a removed entity, migrated ones too).
    pub constraints_removed: Vec<ConstraintId>,
}

/// A constraint that did not survive an edit.
#[derive(Clone, Debug)]
pub struct DroppedConstraint {
    /// Its id before the edit.
    pub id: ConstraintId,
    /// What it was (ids refer to the sketch before the edit).
    pub kind: ConstraintKind,
    /// Why it was dropped.
    pub reason: &'static str,
}

/// Where the constraints of the edited curve went.
#[derive(Clone, Debug, Default)]
pub struct MigrationReport {
    /// `(old id, new id)` of constraints re-created on the new pieces.
    pub migrated: Vec<(ConstraintId, ConstraintId)>,
    /// Constraints that were removed without a replacement.
    pub dropped: Vec<DroppedConstraint>,
}

impl MigrationReport {
    /// The dropped dimensions.
    pub fn dropped_dimensions(&self) -> impl Iterator<Item = &DroppedConstraint> + '_ {
        self.dropped
            .iter()
            .filter(|d| matches!(d.kind, ConstraintKind::Dimension(_)))
    }
}

/// An edit's changes plus the constraint migration report.
#[derive(Clone, Debug, Default)]
pub struct EditOutcome {
    /// Created and removed ids.
    pub result: EditResult,
    /// Migrated and dropped constraints.
    pub migration: MigrationReport,
}

fn checked(
    s: &Sketch,
    e: EntityId,
) -> Result<(crate::model::Entity, crate::geom::Curve2), SketchError> {
    let ent = s.entity(e).ok_or(SketchError::UnknownEntity(e))?.clone();
    if !is_cuttable(&ent.kind) {
        return Err(SketchError::WrongEntityKind {
            entity: e,
            expected: "line, circle or arc",
            found: ent.kind.name(),
        });
    }
    editable(&ent, e)?;
    let curve = s.curve(e).ok_or(SketchError::UnknownEntity(e))?;
    Ok((ent, curve))
}

fn new_pt(s: &Sketch, curve: &crate::geom::Curve2, c: &cuts::Cut) -> NewPt {
    NewPt {
        loc: place(s, curve, c),
        attach: Some((c.cutter, c.snap)),
    }
}

/// Trims `entity` at `pick`.
///
/// The removed part runs between the two intersections (with any other curve) nearest to the
/// pick on either side; a line or arc keeps up to two pieces, a circle keeps one arc. An
/// entity without intersections is deleted. Hits at the entity's own ends do not count;
/// overlapping curves are ignored as cutters.
///
/// Errors: `UnknownEntity`, `WrongEntityKind` (points), `Locked` (projected), `NotApplicable`
/// (a circle with fewer than two intersections).
pub fn trim(s: &mut Sketch, entity: EntityId, pick: Point2) -> Result<EditResult, SketchError> {
    trim_detailed(s, entity, pick).map(|o| o.result)
}

/// [`trim`] with the constraint migration report.
pub fn trim_detailed(
    s: &mut Sketch,
    entity: EntityId,
    pick: Point2,
) -> Result<EditOutcome, SketchError> {
    let (ent, curve) = checked(s, entity)?;
    let found = find_cuts(s, entity, &curve);
    let (t_pick, _) = curve.closest(pick);
    let plan = match ent.kind {
        EntityKind::Circle { .. } => {
            let n = found.len();
            if n < 2 {
                return Err(SketchError::NotApplicable(
                    "a circle needs two intersections to be trimmed",
                ));
            }
            let hi = found.iter().position(|c| c.t > t_pick).unwrap_or(0);
            let lo = (hi + n - 1) % n;
            Plan {
                points: vec![new_pt(s, &curve, &found[hi]), new_pt(s, &curve, &found[lo])],
                pieces: vec![PieceSpec {
                    start: Bnd::New(0),
                    end: Bnd::New(1),
                    t0: found[hi].t,
                    len: (found[lo].t - found[hi].t).rem_euclid(TAU),
                }],
                link: None,
            }
        }
        EntityKind::Line { start, end } | EntityKind::Arc { start, end, .. } => {
            let t_end = curve.param_range().1;
            let lo = found.iter().rev().find(|c| c.t < t_pick);
            let hi = found.iter().find(|c| c.t >= t_pick);
            let mut points = Vec::new();
            let mut pieces = Vec::new();
            if let Some(c) = lo {
                points.push(new_pt(s, &curve, c));
                pieces.push(PieceSpec {
                    start: Bnd::Keep(start),
                    end: Bnd::New(points.len() - 1),
                    t0: 0.0,
                    len: c.t,
                });
            }
            if let Some(c) = hi {
                points.push(new_pt(s, &curve, c));
                pieces.push(PieceSpec {
                    start: Bnd::New(points.len() - 1),
                    end: Bnd::Keep(end),
                    t0: c.t,
                    len: t_end - c.t,
                });
            }
            let link = match ent.kind {
                EntityKind::Line { .. } => Link::Collinear,
                _ => Link::EqualRadius,
            };
            Plan {
                points,
                pieces,
                link: Some(link),
            }
        }
        // `checked()` rejects these kinds before this point.
        EntityKind::Point { .. } | EntityKind::Ellipse { .. } | EntityKind::EllipticArc { .. } => {
            return Err(SketchError::NotApplicable(
                "only lines, circles and arcs can be trimmed",
            ));
        }
    };
    apply(s, entity, &plan)
}

/// Extends the end of `entity` nearest to `pick` to the nearest curve in its way.
///
/// Lines extend straight, arcs along their circle. The extended entity is replaced by a new
/// one whose end is pinned to the target (`PointOnCurve`, or `Coincident` with an end point
/// of the target that lies exactly there).
///
/// Errors: `UnknownEntity`, `WrongEntityKind`, `Locked`, `NotApplicable` (circles, or nothing
/// to extend to).
pub fn extend(s: &mut Sketch, entity: EntityId, pick: Point2) -> Result<EditResult, SketchError> {
    extend_detailed(s, entity, pick).map(|o| o.result)
}

/// [`extend`] with the constraint migration report.
pub fn extend_detailed(
    s: &mut Sketch,
    entity: EntityId,
    pick: Point2,
) -> Result<EditOutcome, SketchError> {
    let (ent, curve) = checked(s, entity)?;
    let (start, end) = match ent.kind {
        EntityKind::Line { start, end } | EntityKind::Arc { start, end, .. } => (start, end),
        _ => {
            return Err(SketchError::NotApplicable("a circle has no end to extend"));
        }
    };
    let (t_end_range, pa, pb) = (
        curve.param_range().1,
        curve.eval(0.0),
        curve.eval(curve.param_range().1),
    );
    let which = if pick.distance_squared(pa) <= pick.distance_squared(pb) {
        End::Start
    } else {
        End::End
    };
    let cut = extension_target(s, entity, &curve, which)
        .ok_or(SketchError::NotApplicable("nothing to extend to"))?;
    let points = vec![new_pt(s, &curve, &cut)];
    let piece = match which {
        End::End => PieceSpec {
            start: Bnd::Keep(start),
            end: Bnd::New(0),
            t0: 0.0,
            len: cut.t,
        },
        End::Start => PieceSpec {
            start: Bnd::New(0),
            end: Bnd::Keep(end),
            t0: cut.t,
            len: t_end_range - cut.t,
        },
    };
    let plan = Plan {
        points,
        pieces: vec![piece],
        link: None,
    };
    apply(s, entity, &plan)
}

/// Splits a line or arc at the point of `entity` nearest to `at`.
///
/// The two new pieces share the new point; lines are tied with `Parallel` (so they stay
/// collinear), arcs share their centre and so their radius. Constraints that describe the
/// whole curve (orientation, tangency to a circle, radius, distances to it) go to the first
/// piece.
///
/// Errors: `UnknownEntity`, `WrongEntityKind`, `Locked`, `NotApplicable` (circles),
/// `Degenerate` (the nearest point is an end of the curve).
pub fn break_at(s: &mut Sketch, entity: EntityId, at: Point2) -> Result<EditResult, SketchError> {
    break_at_detailed(s, entity, at).map(|o| o.result)
}

/// [`break_at`] with the constraint migration report.
pub fn break_at_detailed(
    s: &mut Sketch,
    entity: EntityId,
    at: Point2,
) -> Result<EditOutcome, SketchError> {
    let (ent, curve) = checked(s, entity)?;
    let (start, end) = match ent.kind {
        EntityKind::Line { start, end } | EntityKind::Arc { start, end, .. } => (start, end),
        _ => {
            return Err(SketchError::NotApplicable(
                "a circle has no ends; trim it between two curves instead",
            ));
        }
    };
    let (t, _) = curve.closest(at);
    let t_end = curve.param_range().1;
    let scale = match &curve {
        crate::geom::Curve2::Line(seg) => seg.a.distance(seg.b),
        crate::geom::Curve2::Arc(a) => a.radius,
        crate::geom::Curve2::Circle(c) => c.radius,
        crate::geom::Curve2::Ellipse(_) => {
            return Err(SketchError::NotApplicable(
                "only lines and arcs can be broken",
            ));
        }
    };
    let eps = MERGE / scale.max(1e-9);
    if t <= eps || t >= t_end - eps {
        return Err(SketchError::Degenerate(
            "the break point is an end of the curve",
        ));
    }
    let plan = Plan {
        points: vec![NewPt {
            loc: curve.eval(t),
            attach: None,
        }],
        pieces: vec![
            PieceSpec {
                start: Bnd::Keep(start),
                end: Bnd::New(0),
                t0: 0.0,
                len: t,
            },
            PieceSpec {
                start: Bnd::New(0),
                end: Bnd::Keep(end),
                t0: t,
                len: t_end - t,
            },
        ],
        link: matches!(ent.kind, EntityKind::Line { .. }).then_some(Link::Parallel),
    };
    apply(s, entity, &plan)
}
