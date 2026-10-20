//! Kind validation for entities and constraints, and whole-sketch validation on load.

use std::collections::BTreeMap;

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

    /// Structural validation of an entity kind: params exist, children exist and are points,
    /// children are distinct. With `check_values`, also that coordinates are finite and a
    /// circle radius is positive.
    pub fn entity(&self, kind: &EntityKind, check_values: bool) -> Result<(), SketchError> {
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
        if check_values {
            match *kind {
                EntityKind::Point { x, y } => {
                    for v in [self.params.get(x), self.params.get(y)] {
                        if !v.is_finite() {
                            return Err(SketchError::InvalidValue {
                                what: "point coordinate",
                                value: v,
                            });
                        }
                    }
                }
                EntityKind::Circle { radius, .. } => {
                    positive("circle radius", self.params.get(radius))?;
                }
                EntityKind::Line { .. } | EntityKind::Arc { .. } => {}
            }
        }
        Ok(())
    }

    /// Validates the entity kinds a constraint references. With `check_values`, also the
    /// dimension value.
    pub fn constraint(&self, kind: &ConstraintKind, check_values: bool) -> Result<(), SketchError> {
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
            ConstraintKind::Tangent { a, b, kind } => self.tangent(a, b, kind)?,
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
            ConstraintKind::Dimension(ref d) => self.dimension(d, check_values)?,
        }
        distinct(&kind.entities())
    }

    fn tangent(&self, a: EntityId, b: EntityId, kind: TangentKind) -> Result<(), SketchError> {
        let ka = self.kind(a)?;
        let kb = self.kind(b)?;
        match kind {
            TangentKind::AtEndpoint { .. } => {
                let ka = self.expect(a, LINE_OR_ARC, |k| k.is_line() || k.is_arc())?;
                if ka.is_line() {
                    // line–line tangency at an endpoint is collinearity; needs an arc.
                    self.expect(b, ARC, EntityKind::is_arc)?;
                } else {
                    self.expect(b, LINE_OR_ARC, |k| k.is_line() || k.is_arc())?;
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

    fn dimension(&self, d: &Dimension, check_values: bool) -> Result<(), SketchError> {
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
        if check_values {
            check_dimension_value(&d.kind, d.value)?;
        }
        Ok(())
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
        DimensionKind::Angle { .. } => {
            if value.is_finite() {
                Ok(())
            } else {
                Err(SketchError::InvalidValue { what, value })
            }
        }
    }
}

/// Validates a deserialised sketch: structure only (values are whatever the last solve left).
pub(super) fn whole(
    view: &View<'_>,
    constraints: &BTreeMap<ConstraintId, Constraint>,
    next_entity: u32,
    next_constraint: u32,
    flags_len: usize,
) -> Result<(), SketchError> {
    if flags_len != view.params.len() {
        return Err(SketchError::NotApplicable(
            "parameter values and flags differ in length",
        ));
    }
    if view
        .entities
        .keys()
        .next_back()
        .is_some_and(|id| id.0 >= next_entity)
        || constraints
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
                "a parameter is owned by two entities or constraints",
            ));
        }
        *slot = true;
        Ok(())
    };
    for e in view.entities.values() {
        view.entity(&e.kind, false)?;
        for p in e.kind.own_params() {
            claim(p)?;
        }
    }
    for c in constraints.values() {
        view.constraint(&c.kind, false)?;
        for &p in &c.aux {
            claim(p)?;
        }
    }
    Ok(())
}
