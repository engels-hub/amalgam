//! Replacing a curve by new pieces and migrating its constraints (SK-07).
//!
//! The model has no "move this end point" operation that keeps other constraints sound, and
//! constraints cannot be re-targeted, so every edit creates the surviving pieces as new
//! entities, re-creates the constraints that still hold on them, and removes the original
//! entity. Ids therefore change; [`MigrationReport`] maps old constraint ids to new ones.
//!
//! # Migration rules
//!
//! The original curve `E` is replaced by one to two pieces. The first piece (lowest
//! parameter) carries everything that depends on the *infinite* line or the full circle;
//! when there is a second piece, one extra constraint ties it to the first (`Collinear` for
//! lines, `Equal` radius for arcs, `Parallel` when the pieces share their joint point), so
//! the pieces together keep the degrees of freedom of the original.
//!
//! | Constraint on `E` | Result |
//! |---|---|
//! | `Horizontal`, `Vertical`, `Parallel`, `Perpendicular`, `Collinear` | first piece |
//! | `Tangent` line–circle / circle–circle (`External`, `Internal`) | first piece |
//! | `Tangent` `AtEndpoint` at an end | the piece that still has that original end; dropped if that end was cut off |
//! | `Concentric`, `Equal` radius, `Radius`, `Diameter`, `Distance`, `Angle` | first piece |
//! | `PointOnCurve` | the piece containing the point; dropped if it lay on the removed part, or is implied by a new `Coincident` |
//! | `Fix` | every piece |
//! | `Symmetric` with `E` as axis | first piece; with `E` as a mirrored curve: dropped |
//! | `Midpoint`, `Equal` length, `Length`, `ArcLength` | dropped (the extent changed) |
//! | anything on a point that is removed with `E` | dropped |

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;

use amg_core::Point2;

use super::cuts::{MERGE, raw_param};
use super::{DroppedConstraint, EditOutcome, EditResult, MigrationReport};
use crate::error::SketchError;
use crate::geom::Curve2;
use crate::ids::{ConstraintId, EntityId};
use crate::model::{
    ConstraintKind, DimensionKind, End, Entity, EntityKind, EntityOrigin, Sketch, TangentKind,
};

/// One boundary of a piece.
#[derive(Copy, Clone, Debug)]
pub(crate) enum Bnd {
    /// An existing point entity (an original end point).
    Keep(EntityId),
    /// The `n`-th new point of the plan.
    New(usize),
}

/// A piece of the original curve, along its direction of travel.
#[derive(Copy, Clone, Debug)]
pub(crate) struct PieceSpec {
    pub start: Bnd,
    pub end: Bnd,
    /// Parameter (original curve's domain) where the piece starts.
    pub t0: f64,
    /// Parameter length of the piece.
    pub len: f64,
}

/// A point to create, and what pins it to the cutter.
#[derive(Copy, Clone, Debug)]
pub(crate) struct NewPt {
    pub loc: Point2,
    /// `(cutter, snapped end point of the cutter)`; `None` for a break point.
    pub attach: Option<(EntityId, Option<EntityId>)>,
}

/// The constraint that ties two pieces together.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum Link {
    Parallel,
    Collinear,
    EqualRadius,
}

/// What to do with the curve.
#[derive(Clone, Debug)]
pub(crate) struct Plan {
    pub pieces: Vec<PieceSpec>,
    pub points: Vec<NewPt>,
    pub link: Option<Link>,
}

/// Applies `plan` to `e`. On error the sketch is restored exactly.
pub(crate) fn apply(s: &mut Sketch, e: EntityId, plan: &Plan) -> Result<EditOutcome, SketchError> {
    let backup = s.clone();
    let out = apply_inner(s, e, plan);
    if out.is_err() {
        *s = backup;
    }
    out
}

struct Ctx<'a> {
    e: EntityId,
    ekind: EntityKind,
    curve: Curve2,
    plan: &'a Plan,
    /// New piece ids, parallel to `plan.pieces`.
    ids: Vec<EntityId>,
    /// Point entities that were glued to a cutter end point.
    implied: BTreeSet<EntityId>,
    orig: [Option<EntityId>; 2],
}

fn apply_inner(s: &mut Sketch, e: EntityId, plan: &Plan) -> Result<EditOutcome, SketchError> {
    let ent: Entity = s.entity(e).ok_or(SketchError::UnknownEntity(e))?.clone();
    let curve = s.curve(e).ok_or(SketchError::UnknownEntity(e))?;
    let all: BTreeMap<ConstraintId, ConstraintKind> = s
        .constraints()
        .map(|(id, c)| (id, c.kind.clone()))
        .collect();
    let touching: Vec<ConstraintId> = all
        .iter()
        .filter(|(_, k)| k.entities().contains(&e))
        .map(|(&id, _)| id)
        .collect();

    let mut res = EditResult::default();
    let mut mig = MigrationReport::default();

    // New points, each pinned to its cutter.
    let mut pts: Vec<EntityId> = Vec::new();
    let mut implied = BTreeSet::new();
    for np in &plan.points {
        let id = s.add_point(np.loc)?;
        res.created.push(id);
        pts.push(id);
        if let Some((cutter, snap)) = np.attach {
            let kind = match snap {
                Some(q) => {
                    implied.insert(q);
                    ConstraintKind::Coincident { a: id, b: q }
                }
                None => ConstraintKind::PointOnCurve {
                    point: id,
                    curve: cutter,
                },
            };
            res.constraints_added.push(s.add_constraint(kind)?);
        }
    }

    // Pieces.
    let resolve = |b: Bnd| match b {
        Bnd::Keep(id) => id,
        Bnd::New(k) => pts[k],
    };
    let mut ids = Vec::new();
    for spec in &plan.pieces {
        let (start, end) = (resolve(spec.start), resolve(spec.end));
        let kind = match ent.kind {
            EntityKind::Line { .. } => EntityKind::Line { start, end },
            EntityKind::Circle { center, .. } | EntityKind::Arc { center, .. } => {
                EntityKind::Arc { center, start, end }
            }
            EntityKind::Point { .. } => return Err(SketchError::NotApplicable("a point")),
            EntityKind::Ellipse { .. } | EntityKind::EllipticArc { .. } => {
                return Err(SketchError::NotApplicable(
                    "ellipses cannot be replaced by pieces yet",
                ));
            }
        };
        let id = s.add_entity(kind, ent.construction, ent.origin)?;
        res.created.push(id);
        ids.push(id);
    }
    if let (Some(link), [first, second]) = (plan.link, &ids[..]) {
        let (first, second) = (*first, *second);
        let kind = match link {
            Link::Parallel => ConstraintKind::Parallel {
                a: first,
                b: second,
            },
            Link::Collinear => ConstraintKind::Collinear {
                a: first,
                b: second,
            },
            Link::EqualRadius => ConstraintKind::Equal {
                a: first,
                b: second,
            },
        };
        res.constraints_added.push(s.add_constraint(kind)?);
    }

    // Migrate constraints on `E`.
    let ctx = Ctx {
        e,
        ekind: ent.kind.clone(),
        curve,
        plan,
        ids,
        implied,
        orig: [ent.kind.endpoint(End::Start), ent.kind.endpoint(End::End)],
    };
    for &old in &touching {
        let kind = &all[&old];
        match migrate(s, &ctx, kind) {
            Ok(list) => {
                for k in list {
                    match s.add_constraint(k) {
                        Ok(new) => {
                            res.constraints_added.push(new);
                            mig.migrated.push((old, new));
                        }
                        Err(_) => mig.dropped.push(DroppedConstraint {
                            id: old,
                            kind: kind.clone(),
                            reason: "could not be re-applied to the new piece",
                        }),
                    }
                }
            }
            Err(reason) => mig.dropped.push(DroppedConstraint {
                id: old,
                kind: kind.clone(),
                reason,
            }),
        }
    }

    // Remove the original; its orphaned points and their constraints go with it.
    let rep = s.remove_entities(&[e])?;
    for id in &rep.constraints {
        if !touching.contains(id) {
            mig.dropped.push(DroppedConstraint {
                id: *id,
                kind: all[id].clone(),
                reason: "attached to a point that was removed with the curve",
            });
        }
    }
    res.removed = rep.entities;
    res.constraints_removed = rep.constraints;
    // Added ids that were removed again (cannot happen: new entities never touch `E`).
    Ok(EditOutcome {
        result: res,
        migration: mig,
    })
}

fn subst(kind: &ConstraintKind, from: EntityId, to: EntityId) -> ConstraintKind {
    let r = |x: EntityId| if x == from { to } else { x };
    let mut k = kind.clone();
    match &mut k {
        ConstraintKind::Coincident { a, b }
        | ConstraintKind::Parallel { a, b }
        | ConstraintKind::Perpendicular { a, b }
        | ConstraintKind::Collinear { a, b }
        | ConstraintKind::Tangent { a, b, .. }
        | ConstraintKind::Concentric { a, b }
        | ConstraintKind::Equal { a, b } => {
            *a = r(*a);
            *b = r(*b);
        }
        ConstraintKind::PointOnCurve { point, curve } => {
            *point = r(*point);
            *curve = r(*curve);
        }
        ConstraintKind::Midpoint { point, line } => {
            *point = r(*point);
            *line = r(*line);
        }
        ConstraintKind::Horizontal(t) | ConstraintKind::Vertical(t) => {
            if let crate::model::HvTarget::Line(l) = t {
                *l = r(*l);
            }
        }
        ConstraintKind::Symmetric { a, b, axis } => {
            *a = r(*a);
            *b = r(*b);
            *axis = r(*axis);
        }
        ConstraintKind::Fix { entity } => *entity = r(*entity),
        ConstraintKind::Dimension(d) => match &mut d.kind {
            DimensionKind::Distance { a, b, .. }
            | DimensionKind::HorizontalDistance { a, b, .. }
            | DimensionKind::VerticalDistance { a, b, .. }
            | DimensionKind::Angle { a, b, .. } => {
                *a = r(*a);
                *b = r(*b);
            }
            DimensionKind::Length { line: x }
            | DimensionKind::Radius { curve: x }
            | DimensionKind::Diameter { curve: x }
            | DimensionKind::ArcLength { arc: x }
            | DimensionKind::EllipseMajor { ellipse: x }
            | DimensionKind::EllipseMinor { ellipse: x } => *x = r(*x),
        },
    }
    k
}

/// Piece (index) that contains the original-curve parameter `t`.
fn piece_containing(ctx: &Ctx<'_>, t: f64) -> Option<usize> {
    let scale = match &ctx.curve {
        Curve2::Line(seg) => seg.a.distance(seg.b),
        Curve2::Circle(c) => c.radius,
        Curve2::Arc(a) => a.radius,
        Curve2::Ellipse(e) => e.semi_major().min(e.minor_radius),
    };
    let eps = MERGE / scale.max(1e-9);
    ctx.plan.pieces.iter().position(|p| {
        let inside = |t: f64| t >= p.t0 - eps && t <= p.t0 + p.len + eps;
        match ctx.curve {
            Curve2::Line(_) => inside(t),
            Curve2::Arc(_) | Curve2::Ellipse(_) => inside(t) || inside(t - TAU),
            Curve2::Circle(_) => {
                let u = (t - p.t0).rem_euclid(TAU);
                u <= p.len + eps || u >= TAU - eps
            }
        }
    })
}

/// The constraints `kind` becomes on the new pieces, or why it is dropped.
fn migrate(
    s: &Sketch,
    ctx: &Ctx<'_>,
    kind: &ConstraintKind,
) -> Result<Vec<ConstraintKind>, &'static str> {
    let first = ctx
        .ids
        .first()
        .copied()
        .ok_or("the whole curve was removed")?;
    let one = |id: EntityId| Ok(vec![subst(kind, ctx.e, id)]);
    match kind {
        ConstraintKind::Coincident { .. } => Err("not a curve constraint"),
        ConstraintKind::PointOnCurve { point, .. } => {
            if ctx.implied.contains(point) {
                return Err("implied by the new coincident point");
            }
            let p = s.point(*point).ok_or("point no longer exists")?;
            let t = raw_param(&ctx.curve, p);
            piece_containing(ctx, t)
                .map_or(Err("point lay on the removed part"), |i| one(ctx.ids[i]))
        }
        ConstraintKind::Midpoint { .. } => Err("the midpoint of a piece is not the old midpoint"),
        ConstraintKind::Horizontal(_)
        | ConstraintKind::Vertical(_)
        | ConstraintKind::Parallel { .. }
        | ConstraintKind::Perpendicular { .. }
        | ConstraintKind::Collinear { .. }
        | ConstraintKind::Concentric { .. } => one(first),
        ConstraintKind::Tangent { a, kind: tk, .. } => match *tk {
            TangentKind::AtEndpoint { a_end, b_end, .. } => {
                let end = if *a == ctx.e { a_end } else { b_end };
                let slot = usize::from(end == End::End);
                let want = ctx.orig[slot];
                let hit = ctx.plan.pieces.iter().position(|p| {
                    let bnd = if end == End::Start { p.start } else { p.end };
                    matches!((bnd, want), (Bnd::Keep(k), Some(w)) if k == w)
                });
                hit.map_or(Err("the tangent end was cut off"), |i| one(ctx.ids[i]))
            }
            _ => one(first),
        },
        ConstraintKind::Equal { a, b } => {
            let other = if *a == ctx.e { *b } else { *a };
            let other_is_line = matches!(
                s.entity(other).map(|x| &x.kind),
                Some(EntityKind::Line { .. })
            );
            if matches!(ctx.ekind, EntityKind::Line { .. }) || other_is_line {
                Err("a length changed")
            } else {
                one(first)
            }
        }
        ConstraintKind::Symmetric { axis, .. } => {
            if *axis == ctx.e {
                one(first)
            } else {
                Err("a mirrored curve changed extent")
            }
        }
        ConstraintKind::Fix { .. } => {
            Ok(ctx.ids.iter().map(|&id| subst(kind, ctx.e, id)).collect())
        }
        ConstraintKind::Dimension(d) => match d.kind {
            DimensionKind::Length { .. } | DimensionKind::ArcLength { .. } => {
                Err("its length changed")
            }
            DimensionKind::Radius { .. }
            | DimensionKind::Diameter { .. }
            | DimensionKind::Distance { .. }
            | DimensionKind::Angle { .. } => one(first),
            DimensionKind::HorizontalDistance { .. } | DimensionKind::VerticalDistance { .. } => {
                Err("not a curve dimension")
            }
            // Ellipse axes belong to an ellipse, which is never the curve being replaced.
            DimensionKind::EllipseMajor { .. } | DimensionKind::EllipseMinor { .. } => {
                Err("an ellipse dimension cannot move to a line or arc piece")
            }
        },
    }
}

/// Convenience for the origin check shared by all operations.
pub(crate) fn editable(ent: &Entity, id: EntityId) -> Result<(), SketchError> {
    if matches!(ent.origin, EntityOrigin::Projected(_)) {
        return Err(SketchError::Locked(id));
    }
    Ok(())
}
