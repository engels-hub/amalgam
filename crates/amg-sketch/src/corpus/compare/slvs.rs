//! Option B: SolveSpace `libslvs` through the `slvs` 0.6 crate. **GPL-3.0**: compiled only with
//! cargo feature `slvs`, a dev/test oracle (ADR-012, S03 OQ 4). Never a default feature.
//!
//! Build requirements: a C++ compiler (MSVC works) and **libclang ≤ 15** for `bindgen` 0.64
//! (`LIBCLANG_PATH`; newer libclang generates empty bindings for `slvs.h`). On Windows the
//! crate also needs `advapi32` (linked by `build.rs`).
//!
//! Mapping (everything on one XY workplane; fixed params go to group 1, the rest to group 2):
//!
//! | Amalgam | libslvs |
//! |---|---|
//! | point, line, circle (+ `Distance` radius), arc | `POINT_IN_2D`, `LINE_SEGMENT`, `CIRCLE`, `ARC_OF_CIRCLE` |
//! | `Fix` / locked params | the entity lives in group 1 (constant) |
//! | coincident, concentric | `POINTS_COINCIDENT` (centres) |
//! | point on line / circle / arc | `PT_ON_LINE` / `PT_ON_CIRCLE` |
//! | midpoint, horizontal, vertical, parallel, perpendicular | `AT_MIDPOINT`, `HORIZONTAL`, `VERTICAL`, `PARALLEL`, `PERPENDICULAR` |
//! | collinear | two `PT_ON_LINE` |
//! | tangent at endpoint | `ARC_LINE_TANGENT` / `CURVE_CURVE_TANGENT` (unsigned: chirality can flip) |
//! | equal | `EQUAL_LENGTH_LINES`, `EQUAL_RADIUS`, `EQUAL_LINE_ARC_LEN` |
//! | symmetric (points, lines, circles) | `SYMMETRIC_LINE` (+ `EQUAL_RADIUS`) |
//! | distance point–point / point–line | `PT_PT_DISTANCE` / `PT_LINE_DISTANCE` (signed, opposite sign convention) |
//! | horizontal / vertical distance | `PROJ_PT_DISTANCE` onto a constant x / y axis line (sign flipped, as measured) |
//! | length, radius, diameter, angle | `PT_PT_DISTANCE`, `DIAMETER`, `ANGLE` (unsigned, `other` = supplementary) |
//! | line–circle / circle–circle tangency, arc length, distance to a circle edge, line–line distance, symmetric arcs | not expressible: case skipped |
//!
//! Status: `SLVS_RESULT_OKAY` → converged. This libslvs reports `INCONSISTENT` for redundant
//! but satisfied systems too (it writes the solved values back), so the adapter checks the SK-02
//! residuals: satisfied → `converged_redundant`, otherwise `inconsistent`.

use std::collections::BTreeMap;

use slvs::System;
use slvs::constraint::ConstraintHandle;
use slvs::constraint::{
    Angle, ArcLineTangent, AtMidpoint, CurveCurveTangent, Diameter, EqualLengthLines,
    EqualLineArcLen, EqualRadius, Horizontal, Parallel, Perpendicular, PointsCoincident,
    ProjPtDistance, PtLineDistance, PtOnCircle, PtOnLine, PtPtDistance, SymmetricLine, Vertical,
};
use slvs::entity::{
    ArcOfCircle, Circle, Distance, EntityHandle, LineSegment, Normal, Point, Workplane,
};
use slvs::group::Group;
use slvs::system::{FailReason, SolveResult};
use slvs::utils::make_quaternion;

use super::{Contender, Loaded, Outcome, judge};
use crate::corpus::runner::BackendSkip;
use crate::corpus::{BuiltCase, CaseStatus, ExpectedGroup, GroupKind, Values};
use crate::ids::EntityId;
use crate::model::{
    ConstraintKind, DimensionKind, EntityKind, HvTarget, Side, Sketch, TangentKind,
};
use crate::solve::fixed_params;
use crate::{End, ParamId};

/// libslvs as a contender.
#[derive(Copy, Clone, Debug, Default)]
pub struct Slvs;

#[derive(Copy, Clone, Debug)]
enum Ent {
    Point(EntityHandle<Point>),
    Line(EntityHandle<LineSegment>),
    Circle(EntityHandle<Circle>, EntityHandle<Distance>),
    Arc(EntityHandle<ArcOfCircle>),
}

/// Handle of a driving dimension, for warm edits.
#[derive(Copy, Clone, Debug)]
enum Dim {
    PtPt(ConstraintHandle<PtPtDistance>),
    /// Value = `sign · v`.
    PtLine(ConstraintHandle<PtLineDistance>, f64),
    Proj(ConstraintHandle<ProjPtDistance<LineSegment>>, f64),
    DiaCircle(ConstraintHandle<Diameter<Circle>>, f64),
    DiaArc(ConstraintHandle<Diameter<ArcOfCircle>>, f64),
    Angle(ConstraintHandle<Angle>),
}

struct SlvsLoaded {
    sys: System,
    g2: Group,
    built: BuiltCase,
    ents: BTreeMap<EntityId, Ent>,
    /// libslvs constraint handle → case constraint index.
    owner: BTreeMap<u32, u32>,
    dims: BTreeMap<u32, Dim>,
    fixed_points: Vec<EntityId>,
}

fn skip(why: impl Into<String>) -> BackendSkip {
    BackendSkip(format!("libslvs: {}", why.into()))
}

fn e<T>(r: Result<T, &'static str>) -> Result<T, BackendSkip> {
    r.map_err(skip)
}

struct Builder<'a> {
    sketch: &'a Sketch,
    sys: System,
    g1: Group,
    g2: Group,
    wp: EntityHandle<Workplane>,
    normal: EntityHandle<Normal>,
    fixed: std::collections::BTreeSet<ParamId>,
    ents: BTreeMap<EntityId, Ent>,
    x_axis: Option<EntityHandle<LineSegment>>,
    y_axis: Option<EntityHandle<LineSegment>>,
    fixed_points: Vec<EntityId>,
}

impl Builder<'_> {
    fn group_of(&self, params: &[ParamId]) -> Result<Group, BackendSkip> {
        let n = params.iter().filter(|p| self.fixed.contains(p)).count();
        match n {
            0 => Ok(self.g2),
            k if k == params.len() => Ok(self.g1),
            _ => Err(skip("partly fixed entity")),
        }
    }

    fn point(&self, id: EntityId) -> Result<EntityHandle<Point>, BackendSkip> {
        match self.ents.get(&id) {
            Some(Ent::Point(h)) => Ok(*h),
            _ => Err(skip("expected a point")),
        }
    }

    fn line(&self, id: EntityId) -> Result<EntityHandle<LineSegment>, BackendSkip> {
        match self.ents.get(&id) {
            Some(Ent::Line(h)) => Ok(*h),
            _ => Err(skip("expected a line")),
        }
    }

    fn line_points(&self, id: EntityId) -> Result<(EntityId, EntityId), BackendSkip> {
        match self.sketch.entity(id).map(|e| &e.kind) {
            Some(EntityKind::Line { start, end }) => Ok((*start, *end)),
            _ => Err(skip("expected a line")),
        }
    }

    fn axis(&mut self, horizontal: bool) -> Result<EntityHandle<LineSegment>, BackendSkip> {
        let slot = if horizontal { self.x_axis } else { self.y_axis };
        if let Some(h) = slot {
            return Ok(h);
        }
        let end = if horizontal { [1.0, 0.0] } else { [0.0, 1.0] };
        let a = e(self
            .sys
            .sketch(Point::new_on_workplane(self.g1, self.wp, [0.0, 0.0])))?;
        let b = e(self
            .sys
            .sketch(Point::new_on_workplane(self.g1, self.wp, end)))?;
        let l = e(self.sys.sketch(LineSegment::new(self.g1, a, b)))?;
        if horizontal {
            self.x_axis = Some(l);
        } else {
            self.y_axis = Some(l);
        }
        Ok(l)
    }

    fn entities(&mut self) -> Result<(), BackendSkip> {
        let sketch = self.sketch;
        // Points first (curves reference them).
        for (id, ent) in sketch.entities() {
            if let EntityKind::Point { x, y } = ent.kind {
                let g = self.group_of(&[x, y])?;
                let p = sketch.params();
                let h =
                    e(self
                        .sys
                        .sketch(Point::new_on_workplane(g, self.wp, [p.get(x), p.get(y)])))?;
                if g == self.g1 {
                    self.fixed_points.push(id);
                }
                self.ents.insert(id, Ent::Point(h));
            }
        }
        for (id, ent) in sketch.entities() {
            let h = match ent.kind {
                EntityKind::Point { .. } => continue,
                EntityKind::Line { start, end } => {
                    let (a, b) = (self.point(start)?, self.point(end)?);
                    let g =
                        if self.fixed_points.contains(&start) && self.fixed_points.contains(&end) {
                            self.g1
                        } else {
                            self.g2
                        };
                    Ent::Line(e(self.sys.sketch(LineSegment::new(g, a, b)))?)
                }
                EntityKind::Circle { center, radius } => {
                    let c = self.point(center)?;
                    let g = self.group_of(&[radius])?;
                    let r = e(self
                        .sys
                        .sketch(Distance::new(g, sketch.params().get(radius))))?;
                    let gc = if g == self.g1 && self.fixed_points.contains(&center) {
                        self.g1
                    } else {
                        self.g2
                    };
                    Ent::Circle(e(self.sys.sketch(Circle::new(gc, self.normal, c, r)))?, r)
                }
                EntityKind::Arc { center, start, end } => {
                    let (c, s, t) = (self.point(center)?, self.point(start)?, self.point(end)?);
                    let all = [center, start, end]
                        .iter()
                        .all(|p| self.fixed_points.contains(p));
                    let g = if all { self.g1 } else { self.g2 };
                    Ent::Arc(e(self.sys.sketch(ArcOfCircle::new(g, self.wp, c, s, t)))?)
                }
                EntityKind::Ellipse { .. } | EntityKind::EllipticArc { .. } => {
                    return Err(skip("ellipses are not expressible"));
                }
            };
            self.ents.insert(id, h);
        }
        Ok(())
    }
}

/// Adds the libslvs constraints of one case constraint; returns their handles and the
/// dimension handle for warm edits.
fn constrain(
    b: &mut Builder<'_>,
    kind: &ConstraintKind,
) -> Result<(Vec<u32>, Option<Dim>), BackendSkip> {
    let (g, wp) = (b.g2, b.wp);
    let mut hs = Vec::new();
    let mut dim = None;
    macro_rules! add {
        ($c:expr) => {{
            let h = e(b.sys.constrain($c))?;
            hs.push(h.handle);
            h
        }};
    }
    let ent = |b: &Builder<'_>, id: EntityId| {
        b.ents
            .get(&id)
            .copied()
            .ok_or_else(|| skip("unknown entity"))
    };
    match kind {
        ConstraintKind::Fix { .. } => {}
        ConstraintKind::Coincident { a, b: q } => {
            let (p, q) = (b.point(*a)?, b.point(*q)?);
            add!(PointsCoincident::new(g, p, q, Some(wp)));
        }
        ConstraintKind::PointOnCurve { point, curve } => {
            let p = b.point(*point)?;
            match ent(b, *curve)? {
                Ent::Line(l) => {
                    add!(PtOnLine::new(g, p, l, Some(wp)));
                }
                Ent::Circle(c, _) => {
                    add!(PtOnCircle::new(g, p, c));
                }
                Ent::Arc(a) => {
                    add!(PtOnCircle::new(g, p, a));
                }
                Ent::Point(_) => return Err(skip("point on a point")),
            }
        }
        ConstraintKind::Midpoint { point, line } => {
            let (p, l) = (b.point(*point)?, b.line(*line)?);
            add!(AtMidpoint::new(g, p, l, Some(wp)));
        }
        ConstraintKind::Horizontal(t) | ConstraintKind::Vertical(t) => {
            let horiz = matches!(kind, ConstraintKind::Horizontal(_));
            match *t {
                HvTarget::Line(l) => {
                    let l = b.line(l)?;
                    if horiz {
                        add!(Horizontal::from_line(g, wp, l));
                    } else {
                        add!(Vertical::from_line(g, wp, l));
                    }
                }
                HvTarget::Points(p, q) => {
                    let (p, q) = (b.point(p)?, b.point(q)?);
                    if horiz {
                        add!(Horizontal::from_points(g, wp, p, q));
                    } else {
                        add!(Vertical::from_points(g, wp, p, q));
                    }
                }
            }
        }
        ConstraintKind::Parallel { a, b: q } => {
            let (la, lb) = (b.line(*a)?, b.line(*q)?);
            add!(Parallel::new(g, la, lb, Some(wp)));
        }
        ConstraintKind::Perpendicular { a, b: q } => {
            let (la, lb) = (b.line(*a)?, b.line(*q)?);
            add!(Perpendicular::new(g, la, lb, Some(wp)));
        }
        ConstraintKind::Collinear { a, b: q } => {
            let la = b.line(*a)?;
            let (s, t) = b.line_points(*q)?;
            let (s, t) = (b.point(s)?, b.point(t)?);
            add!(PtOnLine::new(g, s, la, Some(wp)));
            add!(PtOnLine::new(g, t, la, Some(wp)));
        }
        ConstraintKind::Tangent { a, b: q, kind } => {
            let TangentKind::AtEndpoint { a_end, b_end, .. } = *kind else {
                return Err(skip("line/circle tangency without a shared endpoint"));
            };
            match (ent(b, *a)?, ent(b, *q)?) {
                (Ent::Arc(arc), Ent::Line(l)) => {
                    add!(ArcLineTangent::new(g, wp, arc, l, a_end == End::End));
                }
                (Ent::Line(l), Ent::Arc(arc)) => {
                    add!(ArcLineTangent::new(g, wp, arc, l, b_end == End::End));
                }
                (Ent::Arc(x), Ent::Arc(y)) => {
                    add!(CurveCurveTangent::new(
                        g,
                        wp,
                        x,
                        y,
                        a_end == End::End,
                        b_end == End::End
                    ));
                }
                _ => return Err(skip("endpoint tangency between these curves")),
            }
        }
        ConstraintKind::Concentric { a, b: q } => {
            let centre = |id: EntityId| match b.sketch.entity(id).map(|e| &e.kind) {
                Some(EntityKind::Circle { center, .. } | EntityKind::Arc { center, .. }) => {
                    Ok(*center)
                }
                _ => Err(skip("concentric needs circles/arcs")),
            };
            let (p, q) = (b.point(centre(*a)?)?, b.point(centre(*q)?)?);
            add!(PointsCoincident::new(g, p, q, Some(wp)));
        }
        ConstraintKind::Equal { a, b: q } => match (ent(b, *a)?, ent(b, *q)?) {
            (Ent::Line(x), Ent::Line(y)) => {
                add!(EqualLengthLines::new(g, x, y, Some(wp)));
            }
            (Ent::Circle(x, _), Ent::Circle(y, _)) => {
                add!(EqualRadius::new(g, x, y));
            }
            (Ent::Circle(x, _), Ent::Arc(y)) => {
                add!(EqualRadius::new(g, x, y));
            }
            (Ent::Arc(x), Ent::Circle(y, _)) => {
                add!(EqualRadius::new(g, x, y));
            }
            (Ent::Arc(x), Ent::Arc(y)) => {
                add!(EqualRadius::new(g, x, y));
            }
            (Ent::Line(l), Ent::Arc(arc)) | (Ent::Arc(arc), Ent::Line(l)) => {
                add!(EqualLineArcLen::new(g, l, arc));
            }
            _ => return Err(skip("equal between these entities")),
        },
        ConstraintKind::Symmetric { a, b: q, axis } => {
            let axis = b.line(*axis)?;
            match (ent(b, *a)?, ent(b, *q)?) {
                (Ent::Point(p), Ent::Point(r)) => {
                    add!(SymmetricLine::new(g, wp, p, r, axis));
                }
                (Ent::Line(_), Ent::Line(_)) => {
                    let (s1, e1) = b.line_points(*a)?;
                    let (s2, e2) = b.line_points(*q)?;
                    let (s1, e1, s2, e2) = (b.point(s1)?, b.point(e1)?, b.point(s2)?, b.point(e2)?);
                    add!(SymmetricLine::new(g, wp, s1, s2, axis));
                    add!(SymmetricLine::new(g, wp, e1, e2, axis));
                }
                (Ent::Circle(x, _), Ent::Circle(y, _)) => {
                    let centre = |id: EntityId| match b.sketch.entity(id).map(|e| &e.kind) {
                        Some(EntityKind::Circle { center, .. }) => Ok(*center),
                        _ => Err(skip("circle")),
                    };
                    let (c1, c2) = (b.point(centre(*a)?)?, b.point(centre(*q)?)?);
                    add!(SymmetricLine::new(g, wp, c1, c2, axis));
                    add!(EqualRadius::new(g, x, y));
                }
                _ => return Err(skip("symmetric arcs")),
            }
        }
        ConstraintKind::Dimension(d) => {
            if !d.driving {
                return Ok((hs, None));
            }
            let v = d.value;
            let sign = |s: Side| if s == Side::Positive { 1.0 } else { -1.0 };
            match d.kind {
                DimensionKind::Distance { a, b: q, side } => match (ent(b, a)?, ent(b, q)?) {
                    (Ent::Point(p), Ent::Point(r)) => {
                        dim = Some(Dim::PtPt(add!(PtPtDistance::new(g, p, r, v, Some(wp)))));
                    }
                    (Ent::Point(p), Ent::Line(l)) | (Ent::Line(l), Ent::Point(p)) => {
                        // libslvs signed distance = −(ours).
                        let k = -sign(side);
                        dim = Some(Dim::PtLine(
                            add!(PtLineDistance::new(g, p, l, k * v, Some(wp))),
                            k,
                        ));
                    }
                    _ => return Err(skip("distance between these entities")),
                },
                DimensionKind::HorizontalDistance { a, b: q, side }
                | DimensionKind::VerticalDistance { a, b: q, side } => {
                    let horiz = matches!(d.kind, DimensionKind::HorizontalDistance { .. });
                    let (p, r) = (b.point(a)?, b.point(q)?);
                    let axis = b.axis(horiz)?;
                    // Measured against the corpus: libslvs's projected distance has the
                    // opposite sign of `side · (b − a)` for this argument order.
                    let k = -sign(side);
                    dim = Some(Dim::Proj(
                        add!(ProjPtDistance::new(g, p, r, axis, k * v)),
                        k,
                    ));
                }
                DimensionKind::Length { line } => {
                    let (s, t) = b.line_points(line)?;
                    let (s, t) = (b.point(s)?, b.point(t)?);
                    dim = Some(Dim::PtPt(add!(PtPtDistance::new(g, s, t, v, Some(wp)))));
                }
                DimensionKind::Angle { a, b: q, branch } => {
                    let (la, lb) = (b.line(a)?, b.line(q)?);
                    let supp = branch.reverse_a != branch.reverse_b;
                    dim = Some(Dim::Angle(add!(Angle::new(
                        g,
                        la,
                        lb,
                        v.abs().to_degrees(),
                        Some(wp),
                        supp
                    ))));
                }
                DimensionKind::Radius { curve } | DimensionKind::Diameter { curve } => {
                    let k = if matches!(d.kind, DimensionKind::Radius { .. }) {
                        2.0
                    } else {
                        1.0
                    };
                    dim = Some(match ent(b, curve)? {
                        Ent::Circle(c, _) => Dim::DiaCircle(add!(Diameter::new(g, c, k * v)), k),
                        Ent::Arc(a) => Dim::DiaArc(add!(Diameter::new(g, a, k * v)), k),
                        _ => return Err(skip("radius of a non-circle")),
                    });
                }
                DimensionKind::ArcLength { .. } => return Err(skip("arc length dimension")),
                DimensionKind::EllipseMajor { .. } | DimensionKind::EllipseMinor { .. } => {
                    return Err(skip("ellipse dimensions are not expressible"));
                }
            }
        }
    }
    Ok((hs, dim))
}

impl Contender for Slvs {
    fn name(&self) -> String {
        "libslvs".into()
    }

    fn load(&self, built: &BuiltCase) -> Result<Box<dyn Loaded>, BackendSkip> {
        let sketch = &built.sketch;
        let mut sys = System::new();
        let g1 = sys.add_group();
        let g2 = sys.add_group();
        let origin = e(sys.sketch(Point::new_in_3d(g1, [0.0, 0.0, 0.0])))?;
        let normal = e(sys.sketch(Normal::new_in_3d(
            g1,
            make_quaternion([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        )))?;
        let wp = e(sys.sketch(Workplane::new(g1, origin, normal)))?;
        let mut b = Builder {
            sketch,
            sys,
            g1,
            g2,
            wp,
            normal,
            fixed: fixed_params(sketch),
            ents: BTreeMap::new(),
            x_axis: None,
            y_axis: None,
            fixed_points: Vec::new(),
        };
        b.entities()?;
        let mut owner = BTreeMap::new();
        let mut dims = BTreeMap::new();
        for (i, &cid) in built.constraints.iter().enumerate() {
            let Some(c) = sketch.constraint(cid) else {
                return Err(skip("missing constraint"));
            };
            let (hs, dim) = constrain(&mut b, &c.kind)?;
            let i = u32::try_from(i).map_err(|_| skip("too many constraints"))?;
            for h in hs {
                owner.insert(h, i);
            }
            if let Some(d) = dim {
                dims.insert(i, d);
            }
        }
        Ok(Box::new(SlvsLoaded {
            sys: b.sys,
            g2,
            built: built.clone(),
            ents: b.ents,
            owner,
            dims,
            fixed_points: b.fixed_points,
        }))
    }
}

/// Handle number from the boxed handle's `Debug` output (the handle trait is sealed in `slvs`).
fn handle_of(dbg: &str) -> Option<u32> {
    let rest = dbg.split("handle: ").nth(1)?;
    rest.split(|c: char| !c.is_ascii_digit())
        .next()?
        .parse()
        .ok()
}

impl SlvsLoaded {
    fn outcome(&mut self, r: SolveResult) -> Outcome {
        let (dof, failed, reason) = match r {
            SolveResult::Ok { dof } => (dof, Vec::new(), None),
            SolveResult::Fail {
                dof,
                reason,
                failed_constraints,
            } => (
                dof,
                failed_constraints
                    .iter()
                    .filter_map(|h| handle_of(&format!("{h:?}")))
                    .collect(),
                Some(reason),
            ),
        };
        let status = match reason {
            None => CaseStatus::Converged,
            Some(FailReason::Inconsistent) => {
                let mut b = self.built.clone();
                let satisfied = b.apply(&self.values()).is_ok() && {
                    let (w, tol) = judge(&b.sketch);
                    w <= tol.max(1e-8)
                };
                if satisfied {
                    CaseStatus::ConvergedRedundant
                } else {
                    CaseStatus::Inconsistent
                }
            }
            Some(_) => CaseStatus::DidNotConverge,
        };
        let mut cs: Vec<u32> = failed
            .iter()
            .filter_map(|h| self.owner.get(h).copied())
            .collect();
        cs.sort_unstable();
        cs.dedup();
        let groups = if reason.is_none() || cs.is_empty() {
            Some(Vec::new())
        } else {
            Some(vec![ExpectedGroup {
                kind: if status == CaseStatus::ConvergedRedundant {
                    GroupKind::Redundant
                } else {
                    GroupKind::Conflicting
                },
                constraints: cs,
            }])
        };
        Outcome {
            status,
            dof: u32::try_from(dof).ok(),
            groups,
            iterations: None,
        }
    }
}

impl Loaded for SlvsLoaded {
    fn solve(&mut self) -> Outcome {
        self.sys.clear_dragged();
        let r = self.sys.solve(&self.g2);
        self.outcome(r)
    }

    fn values(&self) -> Values {
        let mut v = Values::default();
        for (i, id) in self.built.entities.iter().enumerate() {
            let Ok(i) = u32::try_from(i) else { break };
            match self.ents.get(id) {
                Some(Ent::Point(h)) => {
                    if let Ok(Point::OnWorkplane { coords, .. }) = self.sys.entity_data(h) {
                        v.points.insert(i, coords);
                    }
                }
                Some(Ent::Circle(_, r)) => {
                    if let Ok(d) = self.sys.entity_data(r) {
                        v.radii.insert(i, d.val);
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
        let r = match d {
            Dim::PtPt(h) => self
                .sys
                .update_constraint(&h, |c| c.distance = value)
                .map(|_| ()),
            Dim::PtLine(h, k) => self
                .sys
                .update_constraint(&h, |c| c.distance = k * value)
                .map(|_| ()),
            Dim::Proj(h, k) => self
                .sys
                .update_constraint(&h, |c| c.distance = k * value)
                .map(|_| ()),
            Dim::DiaCircle(h, k) => self
                .sys
                .update_constraint(&h, |c| c.diameter = k * value)
                .map(|_| ()),
            Dim::DiaArc(h, k) => self
                .sys
                .update_constraint(&h, |c| c.diameter = k * value)
                .map(|_| ()),
            Dim::Angle(h) => self
                .sys
                .update_constraint(&h, |c| c.angle = value.abs().to_degrees())
                .map(|_| ()),
        };
        r.map_err(str::to_string)?;
        if let Some(&cid) = self.built.constraints.get(constraint as usize) {
            let _ = self.built.sketch.set_dimension_value(cid, value);
        }
        Ok(())
    }

    fn drag(&mut self, point: u32, target: [f64; 2]) -> Outcome {
        let Some(&id) = self.built.entities.get(point as usize) else {
            return self.solve();
        };
        if let Some(Ent::Point(h)) = self.ents.get(&id).copied()
            && !self.fixed_points.contains(&id)
        {
            let _ = self.sys.update_entity(&h, |p| {
                if let Point::OnWorkplane { coords, .. } = p {
                    *coords = target;
                }
            });
            let _ = self.sys.set_dragged(&h);
        }
        let r = self.sys.solve(&self.g2);
        self.outcome(r)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn parses_debug_handles() {
        assert_eq!(
            super::handle_of(
                "ConstraintHandle { handle: 17, type: \"slvs::constraint::PtPtDistance\" }"
            ),
            Some(17)
        );
        assert_eq!(super::handle_of("nothing"), None);
    }
}
