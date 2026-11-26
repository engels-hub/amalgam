//! Projected entities (SK-09): creating, refreshing, removing and releasing them.

use amg_core::{Point2, Vec2};
use smallvec::SmallVec;

use super::Sketch;
use crate::error::SketchError;
use crate::geom::EllipseArc2;
use crate::ids::{EntityId, ExternalLinkId, ParamId};
use crate::model::{EntityKind, EntityOrigin, ParamFlags, SplineDef};
use crate::project::{ProjectedGeom, ccw_angles, ellipse_form, is_full};

/// The writes that move one projected entity to new geometry.
#[derive(Default)]
struct Writes {
    params: SmallVec<[(ParamId, f64); 8]>,
    knots: Option<(EntityId, Vec<f64>)>,
}

impl Writes {
    fn param(&mut self, p: ParamId, v: f64) {
        self.params.push((p, v));
    }
}

const SHAPE: SketchError =
    SketchError::NotApplicable("the projected geometry changed to another kind of curve");

fn on_circle(c: Point2, r: f64, a: f64) -> Point2 {
    c + Vec2::new(a.cos(), a.sin()) * r
}

impl Sketch {
    /// Adds the point or curve `geom` as geometry projected by `link`: the entity and its new
    /// child points get the origin [`EntityOrigin::Projected`] and locked parameters. Arcs are
    /// stored counter-clockwise; an arc or ellipse sweeping a full turn becomes a circle or a
    /// full ellipse.
    ///
    /// # Errors
    /// As [`ProjectedGeom::validate`]; the sketch is unchanged.
    pub fn add_projected(
        &mut self,
        link: ExternalLinkId,
        geom: &ProjectedGeom,
    ) -> Result<EntityId, SketchError> {
        geom.validate()?;
        // On a copy, so that a failure leaves nothing behind.
        let mut work = self.clone();
        let id = work.insert_projected(link, geom)?;
        *self = work;
        Ok(id)
    }

    fn insert_projected(
        &mut self,
        link: ExternalLinkId,
        geom: &ProjectedGeom,
    ) -> Result<EntityId, SketchError> {
        let origin = EntityOrigin::Projected(link);
        let kind = match geom {
            ProjectedGeom::Point(p) => return self.locked_point(origin, *p),
            ProjectedGeom::Line { a, b } => EntityKind::Line {
                start: self.locked_point(origin, *a)?,
                end: self.locked_point(origin, *b)?,
            },
            ProjectedGeom::Circle { center, radius } => {
                self.locked_circle(origin, *center, *radius)?
            }
            ProjectedGeom::Arc {
                center,
                radius,
                sweep,
                ..
            } if is_full(*sweep) => self.locked_circle(origin, *center, *radius)?,
            ProjectedGeom::Arc {
                center,
                radius,
                start_angle,
                sweep,
            } => {
                let (a0, a1) = ccw_angles(*start_angle, *sweep);
                EntityKind::Arc {
                    center: self.locked_point(origin, *center)?,
                    start: self.locked_point(origin, on_circle(*center, *radius, a0))?,
                    end: self.locked_point(origin, on_circle(*center, *radius, a1))?,
                }
            }
            ProjectedGeom::Ellipse {
                center,
                major,
                minor_radius,
                t0,
                t1,
            } => {
                let e = ellipse_form(*center, *major, *minor_radius, *t0, *t1, None);
                let center_id = self.locked_point(origin, e.center)?;
                let major_id = self.locked_point(origin, e.center + e.major)?;
                let minor = self.locked_param(e.minor_radius)?;
                if is_full(e.t1 - e.t0) {
                    EntityKind::Ellipse {
                        center: center_id,
                        major: major_id,
                        minor_radius: minor,
                    }
                } else {
                    EntityKind::EllipticArc {
                        center: center_id,
                        major: major_id,
                        minor_radius: minor,
                        start: self.locked_point(origin, e.point_at(e.t0))?,
                        end: self.locked_point(origin, e.point_at(e.t1))?,
                        t0: self.locked_param(e.t0)?,
                        t1: self.locked_param(e.t1)?,
                    }
                }
            }
            ProjectedGeom::Spline {
                degree,
                knots,
                poles,
                weights,
            } => {
                let poles = poles
                    .iter()
                    .map(|&p| self.locked_point(origin, p))
                    .collect::<Result<Vec<_>, _>>()?;
                let weights = match weights {
                    Some(w) => Some(
                        w.iter()
                            .map(|&v| self.locked_param(v))
                            .collect::<Result<Vec<_>, _>>()?,
                    ),
                    None => None,
                };
                EntityKind::Spline(SplineDef::Control {
                    degree: *degree,
                    poles,
                    weights,
                    knots: knots.clone(),
                    periodic: false,
                })
            }
        };
        self.add_entity(kind, false, origin)
    }

    fn locked_param(&mut self, v: f64) -> Result<ParamId, SketchError> {
        self.alloc_param(v, ParamFlags { locked: true })
    }

    fn locked_point(&mut self, origin: EntityOrigin, p: Point2) -> Result<EntityId, SketchError> {
        let x = self.locked_param(p.x)?;
        let y = self.locked_param(p.y)?;
        self.add_entity(EntityKind::Point { x, y }, false, origin)
    }

    fn locked_circle(
        &mut self,
        origin: EntityOrigin,
        center: Point2,
        radius: f64,
    ) -> Result<EntityKind, SketchError> {
        let center = self.locked_point(origin, center)?;
        Ok(EntityKind::Circle {
            center,
            radius: self.locked_param(radius)?,
        })
    }

    /// Moves projected entities to new geometry, keeping every id. All of them change or none:
    /// each `(entity, geom)` must keep the entity's kind (a spline also its degree, pole count
    /// and rationality). Lines keep the order of their ends; arcs are counter-clockwise; an
    /// ellipse is expressed on the axis frame closest to its present one.
    ///
    /// Only values that differ are written, so an unchanged projection leaves the revisions
    /// alone.
    ///
    /// # Errors
    /// `UnknownEntity`; `NotApplicable` for an entity that is not projected or changed kind; the
    /// errors of [`ProjectedGeom::validate`].
    pub fn refresh_projected(
        &mut self,
        items: &[(EntityId, &ProjectedGeom)],
    ) -> Result<(), SketchError> {
        let plans = items
            .iter()
            .map(|&(id, g)| self.plan_refresh(id, g))
            .collect::<Result<Vec<_>, _>>()?;
        for w in plans {
            for (p, v) in w.params {
                if self.params.get(p).to_bits() != v.to_bits() {
                    self.params.set(p, v);
                    self.touch_values();
                }
            }
            if let Some((id, new)) = w.knots
                && let Some(EntityKind::Spline(SplineDef::Control { knots, .. })) =
                    self.entities.get_mut(&id).map(|e| &mut e.kind)
                && *knots != new
            {
                *knots = new;
                self.touch_structure();
            }
        }
        Ok(())
    }

    fn plan_refresh(&self, id: EntityId, geom: &ProjectedGeom) -> Result<Writes, SketchError> {
        geom.validate()?;
        let entity = self
            .entities
            .get(&id)
            .ok_or(SketchError::UnknownEntity(id))?;
        if !matches!(entity.origin, EntityOrigin::Projected(_)) {
            return Err(SketchError::NotApplicable("the entity is not projected"));
        }
        let mut w = Writes::default();
        match (&entity.kind, geom) {
            (EntityKind::Point { .. }, ProjectedGeom::Point(p)) => {
                self.plan_point(&mut w, id, *p)?;
            }
            (&EntityKind::Line { start, end }, ProjectedGeom::Line { a, b }) => {
                self.plan_point(&mut w, start, *a)?;
                self.plan_point(&mut w, end, *b)?;
            }
            (
                &EntityKind::Circle { center, radius },
                &ProjectedGeom::Circle {
                    center: c,
                    radius: r,
                },
            ) => {
                self.plan_point(&mut w, center, c)?;
                w.param(radius, r);
            }
            (
                &EntityKind::Circle { center, radius },
                &ProjectedGeom::Arc {
                    center: c,
                    radius: r,
                    sweep,
                    ..
                },
            ) if is_full(sweep) => {
                self.plan_point(&mut w, center, c)?;
                w.param(radius, r);
            }
            (
                &EntityKind::Arc { center, start, end },
                &ProjectedGeom::Arc {
                    center: c,
                    radius,
                    start_angle,
                    sweep,
                },
            ) if !is_full(sweep) => {
                let (a0, a1) = ccw_angles(start_angle, sweep);
                self.plan_point(&mut w, center, c)?;
                self.plan_point(&mut w, start, on_circle(c, radius, a0))?;
                self.plan_point(&mut w, end, on_circle(c, radius, a1))?;
            }
            (
                &EntityKind::Ellipse {
                    center,
                    major,
                    minor_radius,
                },
                &ProjectedGeom::Ellipse {
                    center: c,
                    major: m,
                    minor_radius: b,
                    t0,
                    t1,
                },
            ) if is_full(t1 - t0) => {
                let e = self.refreshed_ellipse(center, major, (c, m, b, t0, t1))?;
                self.plan_point(&mut w, center, e.center)?;
                self.plan_point(&mut w, major, e.center + e.major)?;
                w.param(minor_radius, e.minor_radius);
            }
            (
                &EntityKind::EllipticArc {
                    center,
                    major,
                    minor_radius,
                    start,
                    end,
                    t0: p0,
                    t1: p1,
                },
                &ProjectedGeom::Ellipse {
                    center: c,
                    major: m,
                    minor_radius: b,
                    t0,
                    t1,
                },
            ) if !is_full(t1 - t0) => {
                let e = self.refreshed_ellipse(center, major, (c, m, b, t0, t1))?;
                self.plan_point(&mut w, center, e.center)?;
                self.plan_point(&mut w, major, e.center + e.major)?;
                w.param(minor_radius, e.minor_radius);
                self.plan_point(&mut w, start, e.point_at(e.t0))?;
                self.plan_point(&mut w, end, e.point_at(e.t1))?;
                w.param(p0, e.t0);
                w.param(p1, e.t1);
            }
            (
                EntityKind::Spline(SplineDef::Control {
                    degree,
                    poles,
                    weights,
                    ..
                }),
                ProjectedGeom::Spline {
                    degree: d,
                    knots,
                    poles: at,
                    weights: wv,
                },
            ) if degree == d && poles.len() == at.len() && weights.is_some() == wv.is_some() => {
                for (&pole, &p) in poles.iter().zip(at) {
                    self.plan_point(&mut w, pole, p)?;
                }
                if let (Some(ids), Some(values)) = (weights, wv) {
                    for (&p, &v) in ids.iter().zip(values) {
                        w.param(p, v);
                    }
                }
                w.knots = Some((id, knots.clone()));
            }
            _ => return Err(SHAPE),
        }
        Ok(w)
    }

    fn plan_point(&self, w: &mut Writes, id: EntityId, p: Point2) -> Result<(), SketchError> {
        match self.entities.get(&id).map(|e| &e.kind) {
            Some(&EntityKind::Point { x, y }) => {
                w.param(x, p.x);
                w.param(y, p.y);
                Ok(())
            }
            _ => Err(SketchError::UnknownEntity(id)),
        }
    }

    fn refreshed_ellipse(
        &self,
        center: EntityId,
        major: EntityId,
        (c, m, b, t0, t1): (Point2, Vec2, f64, f64, f64),
    ) -> Result<EllipseArc2, SketchError> {
        let old_center = self
            .point(center)
            .ok_or(SketchError::UnknownEntity(center))?;
        let old_major = self.point(major).ok_or(SketchError::UnknownEntity(major))?;
        Ok(ellipse_form(c, m, b, t0, t1, Some(old_major - old_center)))
    }

    /// Cuts an entity loose from its projection: it becomes user geometry with unlocked
    /// parameters (its child points are not touched). Entities of other origins are left alone.
    ///
    /// # Errors
    /// `UnknownEntity`.
    pub fn release_projected(&mut self, id: EntityId) -> Result<(), SketchError> {
        let e = self
            .entities
            .get_mut(&id)
            .ok_or(SketchError::UnknownEntity(id))?;
        if !matches!(e.origin, EntityOrigin::Projected(_)) {
            return Ok(());
        }
        e.origin = EntityOrigin::User;
        for p in e.kind.own_params() {
            self.params.set_flags(p, ParamFlags::default());
        }
        self.touch_structure();
        Ok(())
    }
}
