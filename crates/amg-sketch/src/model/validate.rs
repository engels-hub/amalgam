//! Kind validation for entities and constraints, and whole-sketch validation on load.

use std::collections::{BTreeMap, BTreeSet};

use amg_core::{Point2, Tolerance};

use crate::error::SketchError;
use crate::geom::{EllipseArc2, check_knots, conic_weight};
use crate::ids::{ConstraintId, EntityId, ParamId};
use crate::model::{
    Constraint, ConstraintKind, Dimension, DimensionKind, End, Entity, EntityKind, HvTarget,
    ParamStore, SplineDef, TangentKind,
};

/// Read-only view used by validation.
pub(super) struct View<'a> {
    pub params: &'a ParamStore,
    pub entities: &'a BTreeMap<EntityId, Entity>,
    pub constraints: &'a BTreeMap<ConstraintId, Constraint>,
}

/// When a constraint is validated.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(super) enum Mode {
    /// Being added: full value domain and the `AtEndpoint` contact check.
    Add,
    /// Being loaded: structure plus cheap value checks (finite values). Geometry left by the
    /// last solve is accepted as is.
    Load,
}

const POINT: &str = "point";
const LINE: &str = "line";
const ARC: &str = "arc";
const ROUND: &str = "circle or arc";
const CURVE: &str = "line, circle, arc, ellipse, spline or conic";
const ENDED: &str = "line, arc, spline or conic";
const LINE_OR_ARC: &str = "line or arc";
const ELLIPSE: &str = "ellipse or elliptical arc";
const CENTERED: &str = "circle, arc or ellipse";

fn wrong(entity: EntityId, expected: &'static str, found: &EntityKind) -> SketchError {
    SketchError::WrongEntityKind {
        entity,
        expected,
        found: found.name(),
    }
}

fn distinct(ids: &[EntityId]) -> Result<(), SketchError> {
    for (i, a) in ids.iter().enumerate() {
        if ids[i + 1..].contains(a) {
            return Err(SketchError::NotApplicable(
                "the same entity is referenced twice",
            ));
        }
    }
    Ok(())
}

fn finite(what: &'static str, value: f64) -> Result<(), SketchError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(SketchError::InvalidValue { what, value })
    }
}

/// `Err(InvalidValue)` unless both coordinates are finite.
pub(super) fn finite_point(what: &'static str, p: Point2) -> Result<(), SketchError> {
    finite(what, p.x)?;
    finite(what, p.y)
}

impl View<'_> {
    fn kind(&self, id: EntityId) -> Result<&EntityKind, SketchError> {
        self.entities
            .get(&id)
            .map(|e| &e.kind)
            .ok_or(SketchError::UnknownEntity(id))
    }

    fn expect(
        &self,
        id: EntityId,
        expected: &'static str,
        ok: fn(&EntityKind) -> bool,
    ) -> Result<&EntityKind, SketchError> {
        let k = self.kind(id)?;
        if ok(k) {
            Ok(k)
        } else {
            Err(wrong(id, expected, k))
        }
    }

    fn point(&self, id: EntityId) -> Result<(), SketchError> {
        self.expect(id, POINT, EntityKind::is_point).map(|_| ())
    }

    fn line(&self, id: EntityId) -> Result<(), SketchError> {
        self.expect(id, LINE, EntityKind::is_line).map(|_| ())
    }

    fn round(&self, id: EntityId) -> Result<(), SketchError> {
        self.expect(id, ROUND, EntityKind::is_round).map(|_| ())
    }

    fn param(&self, p: ParamId) -> Result<(), SketchError> {
        if self.params.contains(p) {
            Ok(())
        } else {
            Err(SketchError::UnknownParam(p))
        }
    }

    /// Position of a point entity that validation already checked.
    fn pos(&self, id: EntityId) -> Point2 {
        match self.entities.get(&id).map(|e| &e.kind) {
            Some(&EntityKind::Point { x, y }) => {
                Point2::new(self.params.get(x), self.params.get(y))
            }
            _ => Point2::NAN,
        }
    }

    /// Validates an entity kind (same rules on add and on load): params exist, children exist,
    /// are points and are distinct; point coordinates are finite; a circle radius is positive;
    /// an arc has a non-zero radius and its end lies on the circle through its start (within
    /// `Tolerance::DEFAULT.linear · max(1, r)`).
    pub fn entity(&self, kind: &EntityKind) -> Result<(), SketchError> {
        let own = kind.own_params();
        for &p in &own {
            self.param(p)?;
        }
        if own
            .iter()
            .enumerate()
            .any(|(i, p)| own[i + 1..].contains(p))
        {
            return Err(SketchError::NotApplicable(
                "the same parameter is used twice",
            ));
        }
        let children = kind.children();
        for &c in &children {
            self.point(c)?;
        }
        distinct(&children)?;
        match *kind {
            EntityKind::Point { x, y } => {
                finite("point coordinate", self.params.get(x))?;
                finite("point coordinate", self.params.get(y))?;
            }
            EntityKind::Circle { radius, .. } => {
                positive("circle radius", self.params.get(radius))?;
            }
            EntityKind::Arc { center, start, end } => {
                let tol = Tolerance::DEFAULT;
                let c = self.pos(center);
                let r = self.pos(start).distance(c);
                if tol.is_zero_len(r) {
                    return Err(SketchError::Degenerate(
                        "arc start coincides with its centre",
                    ));
                }
                if (self.pos(end).distance(c) - r).abs() > tol.linear * r.max(1.0) {
                    return Err(SketchError::Degenerate(
                        "arc start and end are at different distances from the centre",
                    ));
                }
            }
            EntityKind::Ellipse {
                center,
                major,
                minor_radius,
            } => {
                self.ellipse(center, major, minor_radius)?;
            }
            EntityKind::EllipticArc {
                center,
                major,
                minor_radius,
                start,
                end,
                t0,
                t1,
            } => {
                let e = self.ellipse(center, major, minor_radius)?;
                let tol = Tolerance::DEFAULT;
                let reach = tol.linear * e.semi_major().max(e.minor_radius).max(1.0);
                for (p, t, what) in [
                    (start, t0, "elliptical arc start is not at t0"),
                    (end, t1, "elliptical arc end is not at t1"),
                ] {
                    finite("elliptical arc parameter", self.params.get(t))?;
                    if e.point_at(self.params.get(t)).distance(self.pos(p)) > reach {
                        return Err(SketchError::Degenerate(what));
                    }
                }
            }
            EntityKind::Spline(ref d) => self.spline(d)?,
            EntityKind::Conic { rho, .. } => {
                conic_weight(self.params.get(rho))?;
            }
            EntityKind::Line { .. } => {}
        }
        Ok(())
    }

    /// Spline definition checks (same rules on add and on load): not periodic, a valid clamped
    /// knot vector, positive weights, increasing fit parameters.
    fn spline(&self, d: &SplineDef) -> Result<(), SketchError> {
        if d.is_periodic() {
            return Err(SketchError::NotApplicable(
                "periodic splines are not supported (make the end points coincident)",
            ));
        }
        match d {
            SplineDef::Control {
                degree,
                poles,
                weights,
                knots,
                ..
            } => {
                check_knots(usize::from(*degree), poles.len(), knots)?;
                if let Some(w) = weights {
                    if w.len() != poles.len() {
                        return Err(SketchError::NotApplicable("one weight per pole is needed"));
                    }
                    for &p in w {
                        positive("spline weight", self.params.get(p))?;
                    }
                }
            }
            SplineDef::Fit { points, params, .. } => {
                if points.len() < 2 {
                    return Err(SketchError::Degenerate(
                        "a fit-point spline needs at least two points",
                    ));
                }
                let ok = params.len() == points.len()
                    && params.iter().all(|v| v.is_finite())
                    && params[0].abs() <= 1e-9
                    && (params[params.len() - 1] - 1.0).abs() <= 1e-9
                    && params.windows(2).all(|w| w[1] > w[0]);
                if !ok {
                    return Err(SketchError::NotApplicable(
                        "fit parameters must increase strictly from 0 to 1, one per fit point",
                    ));
                }
            }
        }
        Ok(())
    }

    /// The full ellipse of an ellipse entity's centre, major end point and minor radius param;
    /// `Degenerate` / `InvalidValue` unless the semi-major length and the minor radius are
    /// positive.
    fn ellipse(
        &self,
        center: EntityId,
        major: EntityId,
        minor_radius: ParamId,
    ) -> Result<EllipseArc2, SketchError> {
        let c = self.pos(center);
        let m = self.pos(major) - c;
        if Tolerance::DEFAULT.is_zero_len(m.length()) {
            return Err(SketchError::Degenerate(
                "ellipse major axis has zero length",
            ));
        }
        let b = self.params.get(minor_radius);
        positive("ellipse minor radius", b)?;
        Ok(EllipseArc2::full(c, m, b))
    }

    /// True if points `a` and `b` are the same entity or joined by a chain of `Coincident`
    /// constraints.
    fn joined(&self, a: EntityId, b: EntityId) -> bool {
        if a == b {
            return true;
        }
        let mut seen = BTreeSet::from([a]);
        let mut stack = vec![a];
        while let Some(p) = stack.pop() {
            for c in self.constraints.values() {
                if let ConstraintKind::Coincident { a: u, b: v } = c.kind {
                    let other = if u == p {
                        v
                    } else if v == p {
                        u
                    } else {
                        continue;
                    };
                    if other == b {
                        return true;
                    }
                    if seen.insert(other) {
                        stack.push(other);
                    }
                }
            }
        }
        false
    }

    /// Number of auxiliary params the constraint owns: one (the curve parameter) for a point
    /// on an ellipse, elliptical arc, spline or conic, none otherwise.
    pub fn aux_count(&self, kind: &ConstraintKind) -> usize {
        match *kind {
            ConstraintKind::PointOnCurve { curve, .. }
                if self
                    .entities
                    .get(&curve)
                    .is_some_and(|e| e.kind.is_ellipse() || e.kind.is_bspline()) =>
            {
                1
            }
            _ => 0,
        }
    }

    /// Validates the entity kinds a constraint references, and values per `mode`.
    pub fn constraint(&self, kind: &ConstraintKind, mode: Mode) -> Result<(), SketchError> {
        for id in kind.entities() {
            self.kind(id)?;
        }
        match *kind {
            ConstraintKind::Coincident { a, b } => {
                self.point(a)?;
                self.point(b)?;
            }
            ConstraintKind::PointOnCurve { point, curve } => {
                self.point(point)?;
                let k = self.expect(curve, CURVE, EntityKind::is_curve)?;
                if k.children().contains(&point) {
                    return Err(SketchError::NotApplicable(
                        "the point is part of the curve itself",
                    ));
                }
            }
            ConstraintKind::Midpoint { point, line } => {
                self.point(point)?;
                let k = self.expect(line, LINE, EntityKind::is_line)?;
                if k.children().contains(&point) {
                    return Err(SketchError::NotApplicable(
                        "the point is an endpoint of the line",
                    ));
                }
            }
            ConstraintKind::Horizontal(t) | ConstraintKind::Vertical(t) => match t {
                HvTarget::Line(l) => self.line(l)?,
                HvTarget::Points(a, b) => {
                    self.point(a)?;
                    self.point(b)?;
                }
            },
            ConstraintKind::Parallel { a, b }
            | ConstraintKind::Perpendicular { a, b }
            | ConstraintKind::Collinear { a, b } => {
                self.line(a)?;
                self.line(b)?;
            }
            ConstraintKind::Tangent { a, b, kind } => self.tangent(a, b, kind, mode)?,
            ConstraintKind::Curvature {
                a, b, a_end, b_end, ..
            } => self.at_endpoint(a, b, a_end, b_end, mode, true)?,
            ConstraintKind::Concentric { a, b } => {
                self.expect(a, CENTERED, EntityKind::has_center)?;
                self.expect(b, CENTERED, EntityKind::has_center)?;
            }
            ConstraintKind::Equal { a, b } => {
                let ka = self.expect(a, CURVE, EntityKind::is_curve)?;
                let kb = self.kind(b)?;
                if ka.is_bspline() {
                    return match (ka, kb) {
                        (
                            EntityKind::Spline(SplineDef::Control {
                                poles: pa,
                                weights: Some(_),
                                ..
                            }),
                            EntityKind::Spline(SplineDef::Control {
                                poles: pb,
                                weights: Some(_),
                                ..
                            }),
                        ) if pa.len() == pb.len() => distinct(&[a, b]),
                        (EntityKind::Conic { .. }, EntityKind::Conic { .. }) => distinct(&[a, b]),
                        (_, kb) if kb.is_bspline() => Err(SketchError::NotApplicable(
                            "equal needs two conics or two rational control-point splines with the same pole count",
                        )),
                        (_, kb) => Err(wrong(b, "spline or conic", kb)),
                    };
                }
                let ok = match ka {
                    // line–line, line–arc: equal length
                    EntityKind::Line { .. } => kb.is_line() || kb.is_arc(),
                    // circle/arc–circle/arc: equal radius
                    EntityKind::Circle { .. } => kb.is_round(),
                    EntityKind::Arc { .. } => kb.is_line() || kb.is_round(),
                    // ellipse–ellipse: equal axes
                    EntityKind::Ellipse { .. } | EntityKind::EllipticArc { .. } => kb.is_ellipse(),
                    EntityKind::Point { .. } | EntityKind::Spline(_) | EntityKind::Conic { .. } => {
                        false
                    }
                };
                if !ok {
                    let expected = match ka {
                        EntityKind::Line { .. } => LINE_OR_ARC,
                        EntityKind::Circle { .. } => ROUND,
                        EntityKind::Ellipse { .. } | EntityKind::EllipticArc { .. } => ELLIPSE,
                        _ => "line, circle or arc",
                    };
                    return Err(wrong(b, expected, kb));
                }
            }
            ConstraintKind::Symmetric { a, b, axis } => {
                let ka = self.kind(a)?;
                let kb = self.kind(b)?;
                if std::mem::discriminant(ka) != std::mem::discriminant(kb) {
                    return Err(wrong(b, ka.name(), kb));
                }
                if matches!(ka, EntityKind::EllipticArc { .. }) {
                    return Err(SketchError::NotApplicable(
                        "symmetry of elliptical arcs is not supported; mirror the full ellipse",
                    ));
                }
                if ka.is_bspline() {
                    return Err(SketchError::NotApplicable(
                        "symmetry of splines and conics is not supported; make their points symmetric",
                    ));
                }
                self.line(axis)?;
            }
            ConstraintKind::Fix { .. } => {}
            ConstraintKind::Dimension(ref d) => self.dimension(d, mode)?,
        }
        distinct(&kind.entities())
    }

    fn tangent(
        &self,
        a: EntityId,
        b: EntityId,
        kind: TangentKind,
        mode: Mode,
    ) -> Result<(), SketchError> {
        let ka = self.kind(a)?;
        let kb = self.kind(b)?;
        match kind {
            TangentKind::AtEndpoint { a_end, b_end, .. } => {
                self.at_endpoint(a, b, a_end, b_end, mode, false)?;
            }
            TangentKind::External | TangentKind::Internal => {
                if ka.is_bspline() || kb.is_bspline() {
                    return Err(SketchError::NotApplicable(
                        "splines and conics are tangent only at a shared end point (TangentKind::AtEndpoint)",
                    ));
                }
                // Line–ellipse is supported in either order; ellipse–circle/ellipse is not.
                let (line_side, other) = if ka.is_line() {
                    (true, kb)
                } else if kb.is_line() && ka.is_ellipse() {
                    (true, ka)
                } else {
                    (false, kb)
                };
                if line_side && other.is_ellipse() {
                    // fine
                } else if ka.is_line() {
                    self.round(b)?;
                } else if ka.is_round() {
                    if !(kb.is_line() || kb.is_round()) {
                        if kb.is_ellipse() {
                            return Err(SketchError::NotApplicable(
                                "tangency between a circle or arc and an ellipse is not supported",
                            ));
                        }
                        return Err(wrong(b, "line, circle or arc", kb));
                    }
                } else if ka.is_ellipse() {
                    return Err(if kb.is_curve() {
                        SketchError::NotApplicable(
                            "an ellipse can only be tangent to a line (curve-curve tangency with ellipses is not supported)",
                        )
                    } else {
                        wrong(b, "line", kb)
                    });
                } else {
                    return Err(wrong(a, "line, circle, arc or ellipse", ka));
                }
            }
        }
        Ok(())
    }

    /// Shared checks of `Tangent::AtEndpoint` and `Curvature`: line, arc, spline or conic pairs
    /// (line–line is collinearity, so at least one arc, spline or conic; `Curvature` needs a
    /// spline or conic), and on add the named ends must be joined.
    fn at_endpoint(
        &self,
        a: EntityId,
        b: EntityId,
        a_end: End,
        b_end: End,
        mode: Mode,
        curvature: bool,
    ) -> Result<(), SketchError> {
        let ended = |k: &EntityKind| k.is_line() || k.is_arc() || k.is_bspline();
        let ka = self.expect(a, ENDED, ended)?;
        let kb = self.expect(b, ENDED, ended)?;
        if curvature {
            if !(ka.is_bspline() || kb.is_bspline()) {
                return Err(SketchError::NotApplicable(
                    "curvature continuity needs a spline or conic",
                ));
            }
        } else if ka.is_line() && kb.is_line() {
            // line–line tangency at an endpoint is collinearity; needs a curved partner.
            return Err(wrong(b, "arc, spline or conic", kb));
        }
        if mode == Mode::Add {
            let touch = matches!(
                (ka.endpoint(a_end), kb.endpoint(b_end)),
                (Some(pa), Some(pb)) if self.joined(pa, pb)
            );
            if !touch {
                return Err(SketchError::NotApplicable(
                    "the chosen endpoints are neither the same point nor coincident",
                ));
            }
        }
        Ok(())
    }

    fn dimension(&self, d: &Dimension, mode: Mode) -> Result<(), SketchError> {
        match d.kind {
            DimensionKind::Distance { a, b, .. } => {
                // point–point, point–line, line–line, and point/line to a circle or arc edge:
                // every pair except circle/arc–circle/arc.
                let ka = self.kind(a)?;
                let kb = self.kind(b)?;
                if ka.is_round() && kb.is_round() {
                    return Err(wrong(b, "point or line", kb));
                }
                // Distances to an ellipse, spline or conic are not supported.
                if ka.is_ellipse() || ka.is_bspline() {
                    return Err(wrong(a, "point, line, circle or arc", ka));
                }
                if kb.is_ellipse() || kb.is_bspline() {
                    return Err(wrong(b, "point, line, circle or arc", kb));
                }
            }
            DimensionKind::HorizontalDistance { a, b, .. }
            | DimensionKind::VerticalDistance { a, b, .. } => {
                self.point(a)?;
                self.point(b)?;
            }
            DimensionKind::Length { line } => self.line(line)?,
            DimensionKind::Angle { a, b, .. } => {
                self.line(a)?;
                self.line(b)?;
            }
            DimensionKind::Radius { curve } | DimensionKind::Diameter { curve } => {
                self.round(curve)?;
            }
            DimensionKind::ArcLength { arc } => {
                self.expect(arc, ARC, EntityKind::is_arc)?;
            }
            DimensionKind::EllipseMajor { ellipse } | DimensionKind::EllipseMinor { ellipse } => {
                self.expect(ellipse, ELLIPSE, EntityKind::is_ellipse)?;
            }
        }
        finite_point("dimension label position", d.label_pos)?;
        match mode {
            Mode::Add => check_dimension_value(&d.kind, d.value),
            Mode::Load => finite(d.kind.name(), d.value),
        }
    }
}

fn positive(what: &'static str, value: f64) -> Result<(), SketchError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(SketchError::InvalidValue { what, value })
    }
}

/// Domain of a dimension value: distances ≥ 0, radius/diameter/arc length > 0, angles finite.
pub(super) fn check_dimension_value(kind: &DimensionKind, value: f64) -> Result<(), SketchError> {
    let what = kind.name();
    match kind {
        DimensionKind::Distance { .. }
        | DimensionKind::HorizontalDistance { .. }
        | DimensionKind::VerticalDistance { .. }
        | DimensionKind::Length { .. } => {
            if value.is_finite() && value >= 0.0 {
                Ok(())
            } else {
                Err(SketchError::InvalidValue { what, value })
            }
        }
        DimensionKind::Radius { .. }
        | DimensionKind::Diameter { .. }
        | DimensionKind::ArcLength { .. }
        | DimensionKind::EllipseMajor { .. }
        | DimensionKind::EllipseMinor { .. } => positive(what, value),
        DimensionKind::Angle { .. } => finite(what, value),
    }
}

/// Validates a deserialised sketch: structure, parameter ownership, retired slots, and cheap
/// value checks (finite values, positive circle radii, consistent arc radii).
pub(super) fn whole(
    view: &View<'_>,
    retired: &BTreeSet<ParamId>,
    next_entity: u32,
    next_constraint: u32,
    flags_len: usize,
) -> Result<(), SketchError> {
    if flags_len != view.params.len() {
        return Err(SketchError::NotApplicable(
            "parameter values and flags differ in length",
        ));
    }
    if let Some(&v) = view.params.as_slice().iter().find(|v| !v.is_finite()) {
        return Err(SketchError::InvalidValue {
            what: "sketch parameter",
            value: v,
        });
    }
    if view
        .entities
        .keys()
        .next_back()
        .is_some_and(|id| id.0 >= next_entity)
        || view
            .constraints
            .keys()
            .next_back()
            .is_some_and(|id| id.0 >= next_constraint)
    {
        return Err(SketchError::NotApplicable(
            "id counter is behind an existing id",
        ));
    }
    let mut owned = vec![false; view.params.len()];
    let mut claim = |p: ParamId| -> Result<(), SketchError> {
        let slot = owned
            .get_mut(p.index())
            .ok_or(SketchError::UnknownParam(p))?;
        if *slot {
            return Err(SketchError::NotApplicable(
                "a parameter is owned twice (or owned and retired)",
            ));
        }
        *slot = true;
        Ok(())
    };
    for e in view.entities.values() {
        view.entity(&e.kind)?;
        for p in e.kind.own_params() {
            claim(p)?;
        }
    }
    for c in view.constraints.values() {
        view.constraint(&c.kind, Mode::Load)?;
        if c.aux.len() != view.aux_count(&c.kind) {
            return Err(SketchError::NotApplicable(
                "a constraint has the wrong number of auxiliary parameters",
            ));
        }
        for &p in &c.aux {
            claim(p)?;
        }
    }
    for &p in retired {
        claim(p)?;
    }
    Ok(())
}
