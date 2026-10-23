//! Option C: Zoo `ezpz` (`kcl-ezpz` 0.2, MIT) behind feature `ezpz-bench` (dev-only comparison).
//!
//! ezpz is stateless: every solve gets the constraint requests and the initial guesses (our
//! current values). Each sketch parameter is one ezpz variable; fixed params become
//! highest-priority `Fixed` constraints. Mapping:
//!
//! | Amalgam | ezpz |
//! |---|---|
//! | arc internal equation | `Arc` |
//! | coincident, concentric | `PointsCoincident` (centres) |
//! | point on line / circle / arc | `PointLineDistance(…, 0)` / `DistanceVar(centre, p, r)` / `LinesEqualLength(c–p, c–start)` |
//! | midpoint, symmetric (points, lines, circles) | `Midpoint`, `Symmetric` (+ `ScalarEqual` radii) |
//! | horizontal, vertical (line or points) | `Horizontal`, `Vertical` |
//! | parallel, perpendicular, collinear | `LinesAtAngle(Parallel / Perpendicular)`, two `PointLineDistance(…, 0)` |
//! | tangent line–circle (side), circle–circle | `LineTangentToCircle` (line reversed for `Internal`), `CircleTangentToCircle` |
//! | tangent at a shared endpoint | emulated: line ⟂ radius / radii parallel (`LinesAtAngle`), unsigned |
//! | equal lines / circles / arcs | `LinesEqualLength` / `ScalarEqual` / helper radius variable + `DistanceVar` |
//! | distance point–point / point–line, h/v distance | `Distance`, `PointLineDistance` (same sign), `HorizontalDistance` / `VerticalDistance` |
//! | length, angle, radius, diameter, arc length | `Distance`, `LinesAtAngle(Other)` (signed), `CircleRadius` / `ArcRadius`, `ArcLength` |
//! | equal line–arc, symmetric arcs, distances to circle edges, line–line distance | not expressible: case skipped |
//!
//! Status: no unsatisfied constraints → `converged`; otherwise `did_not_converge` (ezpz has no
//! redundancy or DOF report; its "underconstrained variables" list is not a DOF count). The
//! unsatisfied constraints are reported as one conflicting set. Dragging: guesses at the
//! cursor plus a lower-priority `Fixed` on the dragged point (ezpz's priority mechanism).

use std::collections::BTreeMap;

use kcl_ezpz::datatypes::inputs::{
    DatumCircle, DatumCircularArc, DatumDistance, DatumLineSegment, DatumPoint,
};
use kcl_ezpz::datatypes::{Angle, AngleKind};
use kcl_ezpz::{Config, Constraint, ConstraintRequest, Id};

use super::{Contender, Loaded, Outcome};
use crate::End;
use crate::corpus::runner::BackendSkip;
use crate::corpus::{BuiltCase, CaseStatus, ExpectedGroup, GroupKind, Values};
use crate::ids::{EntityId, ParamId};
use crate::model::{
    ConstraintKind, DimensionKind, EntityKind, HvTarget, Side, Sketch, TangentKind,
};
use crate::solve::fixed_params;

/// ezpz as a contender.
#[derive(Copy, Clone, Debug, Default)]
pub struct Ezpz;

#[derive(Copy, Clone, Debug)]
enum Ent {
    Point(DatumPoint),
    Line(DatumLineSegment),
    Circle(DatumCircle),
    Arc(DatumCircularArc),
}

/// A dimension request whose value can be edited (index into `reqs`, value transform).
#[derive(Copy, Clone, Debug)]
struct DimRef {
    req: usize,
    scale: f64,
}

struct EzLoaded {
    built: BuiltCase,
    reqs: Vec<ConstraintRequest>,
    /// Request index → case constraint index (`None` for internal/fixed requests).
    owner: Vec<Option<u32>>,
    dims: BTreeMap<u32, DimRef>,
    /// ezpz variable of each sketch param (`None` if unused).
    var: Vec<Option<Id>>,
    /// Current values, by ezpz id.
    values: Vec<f64>,
    ents: BTreeMap<EntityId, Ent>,
}

fn skip(why: impl Into<String>) -> BackendSkip {
    BackendSkip(format!("ezpz: {}", why.into()))
}

fn top(c: Constraint) -> ConstraintRequest {
    ConstraintRequest::highest_priority(c)
}

struct B<'a> {
    sketch: &'a Sketch,
    var: Vec<Option<Id>>,
    values: Vec<f64>,
    ents: BTreeMap<EntityId, Ent>,
}

impl B<'_> {
    fn id(&mut self, p: ParamId) -> Id {
        if let Some(Some(id)) = self.var.get(p.index()) {
            return *id;
        }
        let id = self.values.len() as Id;
        self.values.push(self.sketch.params().get(p));
        self.var[p.index()] = Some(id);
        id
    }
    fn helper(&mut self, v: f64) -> Id {
        let id = self.values.len() as Id;
        self.values.push(v);
        id
    }
    fn get(&self, id: EntityId) -> Result<Ent, BackendSkip> {
        self.ents
            .get(&id)
            .copied()
            .ok_or_else(|| skip("unknown entity"))
    }
    fn point(&self, id: EntityId) -> Result<DatumPoint, BackendSkip> {
        match self.get(id)? {
            Ent::Point(p) => Ok(p),
            _ => Err(skip("expected a point")),
        }
    }
    fn line(&self, id: EntityId) -> Result<DatumLineSegment, BackendSkip> {
        match self.get(id)? {
            Ent::Line(l) => Ok(l),
            _ => Err(skip("expected a line")),
        }
    }
    fn centre(&self, id: EntityId) -> Result<DatumPoint, BackendSkip> {
        match self.get(id)? {
            Ent::Circle(c) => Ok(c.center),
            Ent::Arc(a) => Ok(a.center),
            _ => Err(skip("expected a circle or arc")),
        }
    }
    /// Endpoint datum of a line/arc end and the radius segment for arcs.
    fn end(
        &self,
        id: EntityId,
        end: End,
    ) -> Result<(DatumPoint, Option<DatumLineSegment>), BackendSkip> {
        match self.get(id)? {
            Ent::Line(l) => Ok((if end == End::Start { l.p0 } else { l.p1 }, None)),
            Ent::Arc(a) => {
                let p = if end == End::Start { a.start } else { a.end };
                Ok((p, Some(DatumLineSegment::new(a.center, p))))
            }
            _ => Err(skip("endpoint of a non-curve")),
        }
    }
}

fn rev(l: DatumLineSegment) -> DatumLineSegment {
    DatumLineSegment::new(l.p1, l.p0)
}

/// Requests of one case constraint and, for a dimension, `(request offset, value scale)`.
type Mapped = (Vec<Constraint>, Option<(usize, f64)>);

/// Requests of one case constraint; the first value-carrying one is returned for warm edits.
fn constrain(b: &mut B<'_>, kind: &ConstraintKind) -> Result<Mapped, BackendSkip> {
    let mut out = Vec::new();
    let mut dim = None;
    match kind {
        ConstraintKind::Fix { .. } => {}
        ConstraintKind::Coincident { a, b: q } => {
            out.push(Constraint::PointsCoincident(b.point(*a)?, b.point(*q)?))
        }
        ConstraintKind::PointOnCurve { point, curve } => {
            let p = b.point(*point)?;
            match b.get(*curve)? {
                Ent::Line(l) => out.push(Constraint::PointLineDistance(p, l, 0.0)),
                Ent::Circle(c) => out.push(Constraint::DistanceVar(c.center, p, c.radius)),
                Ent::Arc(a) => out.push(Constraint::LinesEqualLength(
                    DatumLineSegment::new(a.center, p),
                    DatumLineSegment::new(a.center, a.start),
                )),
                Ent::Point(_) => return Err(skip("point on a point")),
            }
        }
        ConstraintKind::Midpoint { point, line } => {
            out.push(Constraint::Midpoint(b.line(*line)?, b.point(*point)?))
        }
        ConstraintKind::Horizontal(t) | ConstraintKind::Vertical(t) => {
            let l = match *t {
                HvTarget::Line(l) => b.line(l)?,
                HvTarget::Points(p, q) => DatumLineSegment::new(b.point(p)?, b.point(q)?),
            };
            out.push(if matches!(kind, ConstraintKind::Horizontal(_)) {
                Constraint::Horizontal(l)
            } else {
                Constraint::Vertical(l)
            });
        }
        ConstraintKind::Parallel { a, b: q } => {
            out.push(Constraint::LinesAtAngle(
                b.line(*a)?,
                b.line(*q)?,
                AngleKind::Parallel,
            ));
        }
        ConstraintKind::Perpendicular { a, b: q } => {
            out.push(Constraint::LinesAtAngle(
                b.line(*a)?,
                b.line(*q)?,
                AngleKind::Perpendicular,
            ));
        }
        ConstraintKind::Collinear { a, b: q } => {
            let (la, lb) = (b.line(*a)?, b.line(*q)?);
            out.push(Constraint::PointLineDistance(lb.p0, la, 0.0));
            out.push(Constraint::PointLineDistance(lb.p1, la, 0.0));
        }
        ConstraintKind::Tangent { a, b: q, kind } => match *kind {
            TangentKind::External | TangentKind::Internal => match (b.get(*a)?, b.get(*q)?) {
                (Ent::Line(l), Ent::Circle(c)) | (Ent::Circle(c), Ent::Line(l)) => {
                    let l = if *kind == TangentKind::External {
                        l
                    } else {
                        rev(l)
                    };
                    out.push(Constraint::LineTangentToCircle(l, c));
                }
                (Ent::Circle(x), Ent::Circle(y)) => {
                    out.push(Constraint::CircleTangentToCircle(x, y))
                }
                _ => return Err(skip("tangency with arcs without a shared endpoint")),
            },
            TangentKind::AtEndpoint { a_end, b_end, .. } => {
                let (_, ra) = b.end(*a, a_end)?;
                let (_, rb) = b.end(*q, b_end)?;
                match (ra, rb) {
                    (Some(r), None) => out.push(Constraint::LinesAtAngle(
                        r,
                        b.line(*q)?,
                        AngleKind::Perpendicular,
                    )),
                    (None, Some(r)) => out.push(Constraint::LinesAtAngle(
                        b.line(*a)?,
                        r,
                        AngleKind::Perpendicular,
                    )),
                    (Some(r1), Some(r2)) => {
                        out.push(Constraint::LinesAtAngle(r1, r2, AngleKind::Parallel))
                    }
                    (None, None) => return Err(skip("line–line endpoint tangency")),
                }
            }
        },
        ConstraintKind::Curvature { .. } => {
            return Err(skip("curvature continuity is not expressible"));
        }
        ConstraintKind::Concentric { a, b: q } => {
            out.push(Constraint::PointsCoincident(b.centre(*a)?, b.centre(*q)?))
        }
        ConstraintKind::Equal { a, b: q } => match (b.get(*a)?, b.get(*q)?) {
            (Ent::Line(x), Ent::Line(y)) => out.push(Constraint::LinesEqualLength(x, y)),
            (Ent::Circle(x), Ent::Circle(y)) => {
                out.push(Constraint::ScalarEqual(x.radius.id, y.radius.id))
            }
            (Ent::Circle(c), Ent::Arc(arc)) | (Ent::Arc(arc), Ent::Circle(c)) => {
                out.push(Constraint::DistanceVar(arc.center, arc.start, c.radius));
            }
            (Ent::Arc(x), Ent::Arc(y)) => {
                let at = |id: Id| b.values[id as usize];
                let r0 = (at(x.start.id_x()) - at(x.center.id_x()))
                    .hypot(at(x.start.id_y()) - at(x.center.id_y()));
                let r = DatumDistance::new(b.helper(r0));
                out.push(Constraint::DistanceVar(x.center, x.start, r));
                out.push(Constraint::DistanceVar(y.center, y.start, r));
            }
            _ => return Err(skip("equal line–arc")),
        },
        ConstraintKind::Symmetric { a, b: q, axis } => {
            let ax = b.line(*axis)?;
            match (b.get(*a)?, b.get(*q)?) {
                (Ent::Point(p), Ent::Point(r)) => out.push(Constraint::Symmetric(ax, p, r)),
                (Ent::Line(x), Ent::Line(y)) => {
                    out.push(Constraint::Symmetric(ax, x.p0, y.p0));
                    out.push(Constraint::Symmetric(ax, x.p1, y.p1));
                }
                (Ent::Circle(x), Ent::Circle(y)) => {
                    out.push(Constraint::Symmetric(ax, x.center, y.center));
                    out.push(Constraint::ScalarEqual(x.radius.id, y.radius.id));
                }
                _ => return Err(skip("symmetric arcs")),
            }
        }
        ConstraintKind::Dimension(d) => {
            if !d.driving {
                return Ok((out, None));
            }
            let v = d.value;
            let sign = |s: Side| if s == Side::Positive { 1.0 } else { -1.0 };
            let c = match d.kind {
                DimensionKind::Distance { a, b: q, side } => match (b.get(a)?, b.get(q)?) {
                    (Ent::Point(p), Ent::Point(r)) => {
                        dim = Some((0, 1.0));
                        Constraint::Distance(p, r, v)
                    }
                    (Ent::Point(p), Ent::Line(l)) | (Ent::Line(l), Ent::Point(p)) => {
                        dim = Some((0, sign(side)));
                        Constraint::PointLineDistance(p, l, sign(side) * v)
                    }
                    _ => return Err(skip("distance between these entities")),
                },
                DimensionKind::HorizontalDistance { a, b: q, side } => {
                    dim = Some((0, sign(side)));
                    Constraint::HorizontalDistance(b.point(q)?, b.point(a)?, sign(side) * v)
                }
                DimensionKind::VerticalDistance { a, b: q, side } => {
                    dim = Some((0, sign(side)));
                    Constraint::VerticalDistance(b.point(q)?, b.point(a)?, sign(side) * v)
                }
                DimensionKind::Length { line } => {
                    let l = b.line(line)?;
                    dim = Some((0, 1.0));
                    Constraint::Distance(l.p0, l.p1, v)
                }
                DimensionKind::Angle { a, b: q, branch } => {
                    let (mut la, mut lb) = (b.line(a)?, b.line(q)?);
                    if branch.reverse_a {
                        la = rev(la);
                    }
                    if branch.reverse_b {
                        lb = rev(lb);
                    }
                    dim = Some((0, 1.0));
                    Constraint::LinesAtAngle(la, lb, AngleKind::Other(Angle::from_radians(v)))
                }
                DimensionKind::Radius { curve } | DimensionKind::Diameter { curve } => {
                    let k = if matches!(d.kind, DimensionKind::Radius { .. }) {
                        1.0
                    } else {
                        0.5
                    };
                    dim = Some((0, k));
                    match b.get(curve)? {
                        Ent::Circle(c) => Constraint::CircleRadius(c, k * v),
                        Ent::Arc(arc) => Constraint::ArcRadius(arc, k * v),
                        _ => return Err(skip("radius of a non-circle")),
                    }
                }
                DimensionKind::ArcLength { arc } => match b.get(arc)? {
                    Ent::Arc(arc) => {
                        dim = Some((0, 1.0));
                        Constraint::ArcLength(arc, v)
                    }
                    _ => return Err(skip("arc length of a non-arc")),
                },
                DimensionKind::EllipseMajor { .. } | DimensionKind::EllipseMinor { .. } => {
                    return Err(skip("ellipse dimensions are not expressible"));
                }
            };
            out.push(c);
        }
    }
    Ok((out, dim))
}

/// Rebuilds a value-carrying constraint with a new value.
fn with_value(c: &Constraint, v: f64) -> Option<Constraint> {
    Some(match *c {
        Constraint::Distance(p, q, _) => Constraint::Distance(p, q, v),
        Constraint::PointLineDistance(p, l, _) => Constraint::PointLineDistance(p, l, v),
        Constraint::HorizontalDistance(p, q, _) => Constraint::HorizontalDistance(p, q, v),
        Constraint::VerticalDistance(p, q, _) => Constraint::VerticalDistance(p, q, v),
        Constraint::LinesAtAngle(a, b, AngleKind::Other(_)) => {
            Constraint::LinesAtAngle(a, b, AngleKind::Other(Angle::from_radians(v)))
        }
        Constraint::CircleRadius(c, _) => Constraint::CircleRadius(c, v),
        Constraint::ArcRadius(a, _) => Constraint::ArcRadius(a, v),
        Constraint::ArcLength(a, _) => Constraint::ArcLength(a, v),
        _ => return None,
    })
}

impl Contender for Ezpz {
    fn name(&self) -> String {
        "ezpz".into()
    }

    fn load(&self, built: &BuiltCase) -> Result<Box<dyn Loaded>, BackendSkip> {
        let sketch = &built.sketch;
        let mut b = B {
            sketch,
            var: vec![None; sketch.params().len()],
            values: Vec::new(),
            ents: BTreeMap::new(),
        };
        let mut reqs = Vec::new();
        let mut owner = Vec::new();
        for (id, ent) in sketch.entities() {
            if let EntityKind::Point { x, y } = ent.kind {
                let p = DatumPoint::new_xy(b.id(x), b.id(y));
                b.ents.insert(id, Ent::Point(p));
            }
        }
        for (id, ent) in sketch.entities() {
            let e = match ent.kind {
                EntityKind::Point { .. } => continue,
                EntityKind::Line { start, end } => {
                    Ent::Line(DatumLineSegment::new(b.point(start)?, b.point(end)?))
                }
                EntityKind::Circle { center, radius } => Ent::Circle(DatumCircle {
                    center: b.point(center)?,
                    radius: DatumDistance::new(b.id(radius)),
                }),
                EntityKind::Arc { center, start, end } => {
                    let arc = DatumCircularArc {
                        center: b.point(center)?,
                        start: b.point(start)?,
                        end: b.point(end)?,
                    };
                    reqs.push(top(Constraint::Arc(arc)));
                    owner.push(None);
                    Ent::Arc(arc)
                }
                EntityKind::Ellipse { .. } | EntityKind::EllipticArc { .. } => {
                    return Err(skip("ellipses are not expressible"));
                }
                EntityKind::Spline(_) | EntityKind::Conic { .. } => {
                    return Err(skip("splines and conics are not expressible"));
                }
            };
            b.ents.insert(id, e);
        }
        for p in fixed_params(sketch) {
            if let Some(Some(id)) = b.var.get(p.index()) {
                reqs.push(top(Constraint::Fixed(*id, sketch.params().get(p))));
                owner.push(None);
            }
        }
        let mut dims = BTreeMap::new();
        for (i, &cid) in built.constraints.iter().enumerate() {
            let c = sketch
                .constraint(cid)
                .ok_or_else(|| skip("missing constraint"))?;
            let i = u32::try_from(i).map_err(|_| skip("too many constraints"))?;
            let (cs, dim) = constrain(&mut b, &c.kind)?;
            if let Some((k, scale)) = dim {
                dims.insert(
                    i,
                    DimRef {
                        req: reqs.len() + k,
                        scale,
                    },
                );
            }
            for c in cs {
                reqs.push(top(c));
                owner.push(Some(i));
            }
        }
        Ok(Box::new(EzLoaded {
            built: built.clone(),
            reqs,
            owner,
            dims,
            var: b.var,
            values: b.values,
            ents: b.ents,
        }))
    }
}

impl EzLoaded {
    fn run(&mut self, reqs: &[ConstraintRequest]) -> Outcome {
        let guesses: Vec<(Id, f64)> = self
            .values
            .iter()
            .enumerate()
            .map(|(i, v)| (i as Id, *v))
            .collect();
        let config = Config::default()
            .with_convergence_tolerance(1e-10)
            .with_max_iterations(100);
        match kcl_ezpz::solve(reqs, guesses, config) {
            Ok(out) => {
                self.values.copy_from_slice(out.final_values());
                let mut cs: Vec<u32> = out
                    .unsatisfied()
                    .iter()
                    .filter_map(|&r| self.owner.get(r).copied().flatten())
                    .collect();
                cs.sort_unstable();
                cs.dedup();
                let ok = out.unsatisfied().is_empty();
                Outcome {
                    status: if ok {
                        CaseStatus::Converged
                    } else {
                        CaseStatus::DidNotConverge
                    },
                    dof: None,
                    groups: Some(if cs.is_empty() {
                        Vec::new()
                    } else {
                        vec![ExpectedGroup {
                            kind: GroupKind::Conflicting,
                            constraints: cs,
                        }]
                    }),
                    iterations: u32::try_from(out.iterations()).ok(),
                }
            }
            Err(_) => Outcome {
                status: CaseStatus::DidNotConverge,
                dof: None,
                groups: None,
                iterations: None,
            },
        }
    }
}

impl Loaded for EzLoaded {
    fn solve(&mut self) -> Outcome {
        let reqs = std::mem::take(&mut self.reqs);
        let out = self.run(&reqs);
        self.reqs = reqs;
        out
    }

    fn values(&self) -> Values {
        let mut v = Values::default();
        let get = |p: ParamId| {
            self.var
                .get(p.index())
                .copied()
                .flatten()
                .map(|id| self.values[id as usize])
        };
        for (i, id) in self.built.entities.iter().enumerate() {
            let Ok(i) = u32::try_from(i) else { break };
            match self.built.sketch.entity(*id).map(|e| &e.kind) {
                Some(&EntityKind::Point { x, y }) => {
                    if let (Some(x), Some(y)) = (get(x), get(y)) {
                        v.points.insert(i, [x, y]);
                    }
                }
                Some(&EntityKind::Circle { radius, .. }) => {
                    if let Some(r) = get(radius) {
                        v.radii.insert(i, r);
                    }
                }
                _ => {}
            }
        }
        v
    }

    fn set_dimension(&mut self, constraint: u32, value: f64) -> Result<(), String> {
        let d = *self
            .dims
            .get(&constraint)
            .ok_or("not a driving dimension")?;
        let c = with_value(self.reqs[d.req].constraint(), d.scale * value).ok_or("not editable")?;
        self.reqs[d.req] = top(c);
        Ok(())
    }

    fn drag(&mut self, point: u32, target: [f64; 2]) -> Outcome {
        let Some(Ent::Point(p)) = self
            .built
            .entities
            .get(point as usize)
            .and_then(|id| self.ents.get(id).copied())
        else {
            return self.solve();
        };
        self.values[p.id_x() as usize] = target[0];
        self.values[p.id_y() as usize] = target[1];
        let mut reqs = self.reqs.clone();
        reqs.push(ConstraintRequest::new(
            Constraint::Fixed(p.id_x(), target[0]),
            1,
        ));
        reqs.push(ConstraintRequest::new(
            Constraint::Fixed(p.id_y(), target[1]),
            1,
        ));
        self.run(&reqs)
    }
}
