//! Kind validation for entities and constraints, and whole-sketch validation on load.

use std::collections::{BTreeMap, BTreeSet};

use amg_core::{Point2, Tolerance};

use crate::error::SketchError;
use crate::ids::{ConstraintId, EntityId, ParamId};
use crate::model::{
    Constraint, ConstraintKind, Dimension, DimensionKind, Entity, EntityKind, HvTarget, ParamStore,
    TangentKind,
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
const CURVE: &str = "line, circle or arc";
const LINE_OR_ARC: &str = "line or arc";

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
        if own.len() == 2 && own[0] == own[1] {
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
            EntityKind::Line { .. } => {}
        }
        Ok(())
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
            ConstraintKind::Concentric { a, b } => {
                self.round(a)?;
                self.round(b)?;
            }
            ConstraintKind::Equal { a, b } => {
                let ka = self.expect(a, CURVE, EntityKind::is_curve)?;
                let kb = self.kind(b)?;
                let ok = match ka {
                    // line–line, line–arc: equal length
                    EntityKind::Line { .. } => kb.is_line() || kb.is_arc(),
                    // circle/arc–circle/arc: equal radius
                    EntityKind::Circle { .. } => kb.is_round(),
                    EntityKind::Arc { .. } => kb.is_curve(),
                    EntityKind::Point { .. } => false,
                };
                if !ok {
                    let expected = match ka {
                        EntityKind::Line { .. } => LINE_OR_ARC,
                        EntityKind::Circle { .. } => ROUND,
                        _ => CURVE,
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
                let ka = self.expect(a, LINE_OR_ARC, |k| k.is_line() || k.is_arc())?;
                if ka.is_line() {
                    // line–line tangency at an endpoint is collinearity; needs an arc.
                    self.expect(b, ARC, EntityKind::is_arc)?;
                } else {
                    self.expect(b, LINE_OR_ARC, |k| k.is_line() || k.is_arc())?;
                }
                if mode == Mode::Add {
                    let pa = ka.endpoint(a_end);
                    let pb = kb.endpoint(b_end);
                    let touch = matches!((pa, pb), (Some(pa), Some(pb)) if self.joined(pa, pb));
                    if !touch {
                        return Err(SketchError::NotApplicable(
                            "the chosen endpoints are neither the same point nor coincident",
                        ));
                    }
                }
            }
            TangentKind::External | TangentKind::Internal => {
                if ka.is_line() {
                    self.round(b)?;
                } else if ka.is_round() {
                    if !kb.is_curve() {
                        return Err(wrong(b, CURVE, kb));
                    }
                } else {
                    return Err(wrong(a, CURVE, ka));
                }
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
        | DimensionKind::ArcLength { .. } => positive(what, value),
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
        for &p in &c.aux {
            claim(p)?;
        }
    }
    for &p in retired {
        claim(p)?;
    }
    Ok(())
}
