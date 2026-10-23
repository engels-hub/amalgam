//! The [`Sketch`] container.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;
use std::sync::atomic::{AtomicU64, Ordering};

use amg_core::{Plane, Point2, Segment2, Tolerance};
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use super::validate::{self, Mode, View, finite_point};
use super::{Constraint, ConstraintKind, Entity, EntityKind, EntityOrigin, ParamFlags, ParamStore};
use crate::error::SketchError;
use crate::geom::{Arc2, Circle2, Curve2, EllipseArc2};
use crate::ids::{ConstraintId, EntityId, ParamId};

/// Process-wide revision counter: every bump of any sketch gets a fresh number, so a revision
/// never repeats across sketches, clones that diverge, or reloads.
static NEXT_REV: AtomicU64 = AtomicU64::new(1);

fn next_rev() -> u64 {
    NEXT_REV.fetch_add(1, Ordering::Relaxed)
}

/// A 2D parametric sketch: plane, parameter vector, entities and constraints.
///
/// - Ids are allocated monotonically and never reused, also across save/load.
/// - Collections are `BTreeMap`s, so iteration and serialised output are deterministic.
/// - Deserialisation validates the whole structure (dangling ids, wrong kinds, parameter
///   ownership, retired slots) and cheap values (finite, circle radius > 0, consistent arc
///   radii), so a loaded sketch upholds the same invariants as one built through the API.
///   Serialisation fails on non-finite parameter values, so a saved sketch always loads.
/// - Revisions ([`structure_rev`](Sketch::structure_rev), [`values_rev`](Sketch::values_rev))
///   are runtime cache keys, not serialised. They are unique within the process: compare them
///   only for equality.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(try_from = "SketchRepr")]
pub struct Sketch {
    plane: Plane,
    params: ParamStore,
    entities: BTreeMap<EntityId, Entity>,
    constraints: BTreeMap<ConstraintId, Constraint>,
    /// Param slots of removed entities/constraints; never handed out again, dropped by
    /// `compact`.
    #[serde(default, skip_serializing_if = "BTreeSet::is_empty")]
    retired: BTreeSet<ParamId>,
    next_entity: u32,
    next_constraint: u32,
    #[serde(skip)]
    structure_rev: u64,
    #[serde(skip)]
    values_rev: u64,
}

/// Serialised form; validated into a [`Sketch`].
#[derive(Deserialize)]
struct SketchRepr {
    plane: Plane,
    params: ParamStore,
    entities: BTreeMap<EntityId, Entity>,
    constraints: BTreeMap<ConstraintId, Constraint>,
    #[serde(default)]
    retired: BTreeSet<ParamId>,
    next_entity: u32,
    next_constraint: u32,
}

impl TryFrom<SketchRepr> for Sketch {
    type Error = SketchError;
    fn try_from(r: SketchRepr) -> Result<Self, SketchError> {
        let view = View {
            params: &r.params,
            entities: &r.entities,
            constraints: &r.constraints,
        };
        validate::whole(
            &view,
            &r.retired,
            r.next_entity,
            r.next_constraint,
            r.params.flags.len(),
        )?;
        Ok(Sketch {
            plane: r.plane,
            params: r.params,
            entities: r.entities,
            constraints: r.constraints,
            retired: r.retired,
            next_entity: r.next_entity,
            next_constraint: r.next_constraint,
            structure_rev: next_rev(),
            values_rev: next_rev(),
        })
    }
}

/// What [`Sketch::remove_entities`] removed, sorted by id.
#[derive(Clone, Debug, Default)]
pub struct RemoveReport {
    /// Removed entities (requested ones, their dependents and orphaned points).
    pub entities: Vec<EntityId>,
    /// Removed constraints (every constraint touching a removed entity).
    pub constraints: Vec<ConstraintId>,
}

impl Sketch {
    /// Empty sketch on `plane`.
    pub fn new(plane: Plane) -> Self {
        Sketch {
            plane,
            params: ParamStore::default(),
            entities: BTreeMap::new(),
            constraints: BTreeMap::new(),
            retired: BTreeSet::new(),
            next_entity: 0,
            next_constraint: 0,
            structure_rev: next_rev(),
            values_rev: next_rev(),
        }
    }

    /// The sketch plane; sketch x/y are the plane's `x`/`y`.
    pub fn plane(&self) -> &Plane {
        &self.plane
    }

    /// Replaces the sketch plane (resolved by `SketchFeature` at every recompute). Bumps
    /// `values_rev`.
    pub fn set_plane(&mut self, plane: Plane) {
        self.plane = plane;
        self.touch_values();
    }

    /// The parameter vector.
    pub fn params(&self) -> &ParamStore {
        &self.params
    }

    /// Mutable parameter vector. Bumps `values_rev`.
    pub fn params_mut(&mut self) -> &mut ParamStore {
        self.touch_values();
        &mut self.params
    }

    /// The entity `id`, if it exists.
    pub fn entity(&self, id: EntityId) -> Option<&Entity> {
        self.entities.get(&id)
    }

    /// All entities in id order.
    pub fn entities(&self) -> impl Iterator<Item = (EntityId, &Entity)> + '_ {
        self.entities.iter().map(|(&id, e)| (id, e))
    }

    /// The constraint `id`, if it exists.
    pub fn constraint(&self, id: ConstraintId) -> Option<&Constraint> {
        self.constraints.get(&id)
    }

    /// All constraints in id order.
    pub fn constraints(&self) -> impl Iterator<Item = (ConstraintId, &Constraint)> + '_ {
        self.constraints.iter().map(|(&id, c)| (id, c))
    }

    /// Position of point entity `id`; `None` if it does not exist or is not a point.
    pub fn point(&self, id: EntityId) -> Option<Point2> {
        match self.entities.get(&id)?.kind {
            EntityKind::Point { x, y } => Some(Point2::new(self.params.get(x), self.params.get(y))),
            _ => None,
        }
    }

    // ---- Entities ---------------------------------------------------------------------

    /// Adds a free user point. `Err(InvalidValue)` if a coordinate is not finite.
    pub fn add_point(&mut self, p: Point2) -> Result<EntityId, SketchError> {
        finite_point("point coordinate", p)?;
        Ok(self.push_point(p))
    }

    /// Adds a line from `a` to `b`, creating its two end points. `Err(InvalidValue)` if a
    /// coordinate is not finite.
    pub fn add_line(&mut self, a: Point2, b: Point2) -> Result<EntityId, SketchError> {
        finite_point("point coordinate", a)?;
        finite_point("point coordinate", b)?;
        let start = self.push_point(a);
        let end = self.push_point(b);
        Ok(self.insert_entity(EntityKind::Line { start, end }, false, EntityOrigin::User))
    }

    /// Adds a line between two existing, distinct point entities.
    pub fn add_line_between(&mut self, a: EntityId, b: EntityId) -> Result<EntityId, SketchError> {
        self.add_entity(
            EntityKind::Line { start: a, end: b },
            false,
            EntityOrigin::User,
        )
    }

    /// Adds a circle, creating its centre point. `Err(InvalidValue)` unless `r > 0`, finite.
    pub fn add_circle(&mut self, center: Point2, r: f64) -> Result<EntityId, SketchError> {
        finite_point("point coordinate", center)?;
        if !(r.is_finite() && r > 0.0) {
            return Err(SketchError::InvalidValue {
                what: "circle radius",
                value: r,
            });
        }
        let center = self.push_point(center);
        let radius = self.params.push(r, ParamFlags::default());
        Ok(self.insert_entity(
            EntityKind::Circle { center, radius },
            false,
            EntityOrigin::User,
        ))
    }

    /// Adds a CCW arc from `start` to `end` about `center`, creating its three points.
    ///
    /// The radius is `|start − center|`; `end` is projected radially onto that circle, so the
    /// arc's internal equation holds exactly at creation. `Err(Degenerate)` if `start` or
    /// `end` coincides with `center` (within `Tolerance::DEFAULT.linear`), or if the start and
    /// end directions coincide within `Tolerance::DEFAULT.angular` (the sweep would be
    /// ambiguous between ~0 and 2π; use a circle). `Err(InvalidValue)` for non-finite input.
    pub fn add_arc(
        &mut self,
        center: Point2,
        start: Point2,
        end: Point2,
    ) -> Result<EntityId, SketchError> {
        for p in [center, start, end] {
            finite_point("point coordinate", p)?;
        }
        let tol = Tolerance::DEFAULT;
        let r = start.distance(center);
        if tol.is_zero_len(r) {
            return Err(SketchError::Degenerate(
                "arc start coincides with its centre",
            ));
        }
        let de = end - center;
        let de_len = de.length();
        if tol.is_zero_len(de_len) {
            return Err(SketchError::Degenerate("arc end coincides with its centre"));
        }
        let ds = start - center;
        if ds.perp_dot(de).atan2(ds.dot(de)).abs() <= tol.angular {
            return Err(SketchError::Degenerate(
                "arc start and end directions coincide",
            ));
        }
        let end = center + de * (r / de_len);
        let c = self.push_point(center);
        let s = self.push_point(start);
        let e = self.push_point(end);
        Ok(self.insert_entity(
            EntityKind::Arc {
                center: c,
                start: s,
                end: e,
            },
            false,
            EntityOrigin::User,
        ))
    }

    /// Adds an ellipse with centre `center`, the end of its semi-major axis at `major_end` and
    /// minor radius `minor_radius` (SK-22). Creates the two points.
    ///
    /// `Err(Degenerate)` if `major_end` coincides with `center`; `Err(InvalidValue)` unless the
    /// minor radius is positive and finite, or for non-finite input. The minor radius may
    /// exceed the semi-major length.
    pub fn add_ellipse(
        &mut self,
        center: Point2,
        major_end: Point2,
        minor_radius: f64,
    ) -> Result<EntityId, SketchError> {
        self.check_ellipse_input(center, major_end, minor_radius)?;
        let c = self.push_point(center);
        let m = self.push_point(major_end);
        let minor_radius = self.params.push(minor_radius, ParamFlags::default());
        Ok(self.insert_entity(
            EntityKind::Ellipse {
                center: c,
                major: m,
                minor_radius,
            },
            false,
            EntityOrigin::User,
        ))
    }

    /// Adds an elliptical arc on the ellipse of [`add_ellipse`](Sketch::add_ellipse), running
    /// CCW from eccentric angle `t0` to `t1` (rad), creating its start point `P(t0)` and end
    /// point `P(t1)` (SK-22).
    ///
    /// `Err(Degenerate)` if the sweep `(t1 − t0) mod 2π` is within `Tolerance::DEFAULT.angular`
    /// of 0 or 2π (use a full ellipse), plus the errors of `add_ellipse`.
    pub fn add_elliptic_arc(
        &mut self,
        center: Point2,
        major_end: Point2,
        minor_radius: f64,
        t0: f64,
        t1: f64,
    ) -> Result<EntityId, SketchError> {
        self.check_ellipse_input(center, major_end, minor_radius)?;
        for t in [t0, t1] {
            if !t.is_finite() {
                return Err(SketchError::InvalidValue {
                    what: "elliptical arc parameter",
                    value: t,
                });
            }
        }
        let sweep = (t1 - t0).rem_euclid(TAU);
        let tol = Tolerance::DEFAULT.angular;
        if sweep <= tol || sweep >= TAU - tol {
            return Err(SketchError::Degenerate(
                "elliptical arc start and end coincide",
            ));
        }
        let e = EllipseArc2::full(center, major_end - center, minor_radius);
        let c = self.push_point(center);
        let m = self.push_point(major_end);
        let s = self.push_point(e.point_at(t0));
        let en = self.push_point(e.point_at(t1));
        let minor_radius = self.params.push(minor_radius, ParamFlags::default());
        let t0 = self.params.push(t0, ParamFlags::default());
        let t1 = self.params.push(t1, ParamFlags::default());
        Ok(self.insert_entity(
            EntityKind::EllipticArc {
                center: c,
                major: m,
                minor_radius,
                start: s,
                end: en,
                t0,
                t1,
            },
            false,
            EntityOrigin::User,
        ))
    }

    fn check_ellipse_input(
        &self,
        center: Point2,
        major_end: Point2,
        minor_radius: f64,
    ) -> Result<(), SketchError> {
        finite_point("point coordinate", center)?;
        finite_point("point coordinate", major_end)?;
        if !(minor_radius.is_finite() && minor_radius > 0.0) {
            return Err(SketchError::InvalidValue {
                what: "ellipse minor radius",
                value: minor_radius,
            });
        }
        if Tolerance::DEFAULT.is_zero_len(major_end.distance(center)) {
            return Err(SketchError::Degenerate(
                "ellipse major axis has zero length",
            ));
        }
        Ok(())
    }

    /// Adds an entity of any kind referencing existing params and points.
    ///
    /// Validates that the params exist, were issued by [`alloc_param`](Sketch::alloc_param)
    /// and are owned by no other entity or constraint (slots of removed entities are retired
    /// and never reused), that children exist, are points and are distinct, that point
    /// coordinates are finite, a circle radius is positive, and an arc's end lies on the circle
    /// through its start (`Degenerate` otherwise; start and end may coincide).
    ///
    /// `Projected` entities get their own params *and their child points' params* locked.
    pub fn add_entity(
        &mut self,
        kind: EntityKind,
        construction: bool,
        origin: EntityOrigin,
    ) -> Result<EntityId, SketchError> {
        self.view().entity(&kind)?;
        let own = kind.own_params();
        if own.iter().any(|p| self.retired.contains(p)) {
            return Err(SketchError::NotApplicable(
                "a parameter belongs to a removed entity or constraint",
            ));
        }
        if own.iter().any(|&p| self.param_in_use(p)) {
            return Err(SketchError::NotApplicable(
                "a parameter is already owned by another entity or constraint",
            ));
        }
        if matches!(origin, EntityOrigin::Projected(_)) {
            let child_params: Vec<ParamId> = kind
                .children()
                .iter()
                .flat_map(|c| self.entities[c].kind.own_params())
                .collect();
            for p in own.iter().chain(&child_params) {
                self.params.set_flags(*p, ParamFlags { locked: true });
            }
        }
        Ok(self.insert_entity(kind, construction, origin))
    }

    /// Allocates a new parameter slot (for [`add_entity`](Sketch::add_entity)). Slots no
    /// entity or constraint owns are dropped by [`compact`](Sketch::compact).
    /// `Err(InvalidValue)` if `value` is not finite.
    pub fn alloc_param(&mut self, value: f64, flags: ParamFlags) -> Result<ParamId, SketchError> {
        if !value.is_finite() {
            return Err(SketchError::InvalidValue {
                what: "parameter",
                value,
            });
        }
        self.touch_structure();
        Ok(self.params.push(value, flags))
    }

    /// Marks an entity as construction geometry (or not). Bumps `values_rev` (profiles change,
    /// the equation system does not).
    pub fn set_construction(&mut self, id: EntityId, on: bool) -> Result<(), SketchError> {
        let e = self
            .entities
            .get_mut(&id)
            .ok_or(SketchError::UnknownEntity(id))?;
        e.construction = on;
        self.touch_values();
        Ok(())
    }

    /// Removes the entities, their dependents and sub-entities, and every constraint touching
    /// any of them.
    ///
    /// - Dependents: an entity built on a removed entity is removed (removing a point removes
    ///   the lines, circles and arcs using it).
    /// - Sub-entities: points of a removed entity that no surviving entity uses are removed
    ///   (removing a line removes its end points unless another curve shares them), except
    ///   `Projected` points, which belong to their projection link and stay.
    ///
    /// Nothing is removed if any id is unknown (`Err(UnknownEntity)`). Param slots are retired
    /// (never reused) until [`compact`](Sketch::compact) drops them.
    pub fn remove_entities(&mut self, ids: &[EntityId]) -> Result<RemoveReport, SketchError> {
        if let Some(&id) = ids.iter().find(|id| !self.entities.contains_key(id)) {
            return Err(SketchError::UnknownEntity(id));
        }
        let mut gone: BTreeSet<EntityId> = ids.iter().copied().collect();
        loop {
            let before = gone.len();
            // Dependents of removed entities.
            for (&id, e) in &self.entities {
                if !gone.contains(&id) && e.kind.children().iter().any(|c| gone.contains(c)) {
                    gone.insert(id);
                }
            }
            // Children no surviving entity uses.
            let used_by_survivors: BTreeSet<EntityId> = self
                .entities
                .iter()
                .filter(|(id, _)| !gone.contains(id))
                .flat_map(|(_, e)| e.kind.children())
                .collect();
            let orphans: Vec<EntityId> = gone
                .iter()
                .flat_map(|id| self.entities[id].kind.children())
                .filter(|c| {
                    !used_by_survivors.contains(c)
                        && !matches!(self.entities[c].origin, EntityOrigin::Projected(_))
                })
                .collect();
            gone.extend(orphans);
            if gone.len() == before {
                break;
            }
        }
        let constraints: Vec<ConstraintId> = self
            .constraints
            .iter()
            .filter(|(_, c)| c.kind.entities().iter().any(|e| gone.contains(e)))
            .map(|(&id, _)| id)
            .collect();
        for id in &gone {
            if let Some(e) = self.entities.remove(id) {
                self.retired.extend(e.kind.own_params());
            }
        }
        for id in &constraints {
            if let Some(c) = self.constraints.remove(id) {
                self.retired.extend(c.aux);
            }
        }
        self.touch_structure();
        Ok(RemoveReport {
            entities: gone.into_iter().collect(),
            constraints,
        })
    }

    // ---- Constraints ------------------------------------------------------------------

    /// Validates entity kinds (and the dimension value) and stores `kind` as given.
    ///
    /// Errors: `UnknownEntity`, `WrongEntityKind`, `NotApplicable` (an entity referenced
    /// twice, a point constrained onto its own curve), `InvalidValue` (dimension value out of
    /// its domain: distances ≥ 0, radius/diameter/arc length > 0, angles finite).
    pub fn add_constraint(&mut self, kind: ConstraintKind) -> Result<ConstraintId, SketchError> {
        self.view().constraint(&kind, Mode::Add)?;
        let mut aux = SmallVec::new();
        if let ConstraintKind::PointOnCurve { point, curve } = kind
            && self.view().aux_count(&kind) == 1
        {
            // The curve parameter of the point on the (full) ellipse, as the starting value.
            let t = match (self.point(point), self.curve(curve)) {
                (Some(p), Some(Curve2::Ellipse(e))) => {
                    EllipseArc2::full(e.center, e.major, e.minor_radius)
                        .closest(p)
                        .0
                }
                _ => 0.0,
            };
            aux.push(self.params.push(t, ParamFlags::default()));
        }
        let id = ConstraintId(self.next_constraint);
        self.next_constraint += 1;
        self.constraints.insert(id, Constraint { kind, aux });
        self.touch_structure();
        Ok(id)
    }

    /// Removes one constraint. Its aux param slots are retired until
    /// [`compact`](Sketch::compact).
    pub fn remove_constraint(&mut self, id: ConstraintId) -> Result<(), SketchError> {
        let c = self
            .constraints
            .remove(&id)
            .ok_or(SketchError::UnknownConstraint(id))?;
        self.retired.extend(c.aux);
        self.touch_structure();
        Ok(())
    }

    /// Sets a dimension's value (mm or rad). Bumps `values_rev`.
    pub fn set_dimension_value(&mut self, id: ConstraintId, value: f64) -> Result<(), SketchError> {
        let d = self.dimension_mut(id)?;
        validate::check_dimension_value(&d.kind, value)?;
        d.value = value;
        self.touch_values();
        Ok(())
    }

    /// Makes a dimension driving or driven. Bumps `structure_rev` (the equation set changes).
    pub fn set_driving(&mut self, id: ConstraintId, driving: bool) -> Result<(), SketchError> {
        self.dimension_mut(id)?.driving = driving;
        self.touch_structure();
        Ok(())
    }

    // ---- Geometry and revisions -------------------------------------------------------

    /// Evaluated geometry of curve entity `id` at the current parameter values; `None` for
    /// points and unknown ids.
    pub fn curve(&self, id: EntityId) -> Option<Curve2> {
        let pt = |e| self.point(e);
        match self.entities.get(&id)?.kind {
            EntityKind::Point { .. } => None,
            EntityKind::Line { start, end } => Some(Curve2::Line(Segment2 {
                a: pt(start)?,
                b: pt(end)?,
            })),
            EntityKind::Circle { center, radius } => Some(Curve2::Circle(Circle2 {
                center: pt(center)?,
                radius: self.params.get(radius),
            })),
            EntityKind::Arc { center, start, end } => Some(Curve2::Arc(
                Arc2::from_center_start_end(pt(center)?, pt(start)?, pt(end)?),
            )),
            EntityKind::Ellipse {
                center,
                major,
                minor_radius,
            } => {
                let c = pt(center)?;
                Some(Curve2::Ellipse(EllipseArc2::full(
                    c,
                    pt(major)? - c,
                    self.params.get(minor_radius),
                )))
            }
            EntityKind::EllipticArc {
                center,
                major,
                minor_radius,
                t0,
                t1,
                ..
            } => {
                let c = pt(center)?;
                let (t0, t1) = (self.params.get(t0), self.params.get(t1));
                let mut sweep = (t1 - t0).rem_euclid(TAU);
                if sweep <= 0.0 {
                    sweep = TAU;
                }
                Some(Curve2::Ellipse(EllipseArc2 {
                    center: c,
                    major: pt(major)? - c,
                    minor_radius: self.params.get(minor_radius),
                    t0,
                    t1: t0 + sweep,
                }))
            }
        }
    }

    /// Changes on every add/remove of entities or constraints, driving toggle, param
    /// allocation and `compact`.
    pub fn structure_rev(&self) -> u64 {
        self.structure_rev
    }

    /// Changes on every structure change and on any param, dimension value, construction or
    /// plane change.
    pub fn values_rev(&self) -> u64 {
        self.values_rev
    }

    /// Renumbers `ParamId`s densely, dropping slots no entity or constraint owns (removed
    /// entities, unused [`alloc_param`](Sketch::alloc_param) slots). Kept slots keep their
    /// relative order. Entity and constraint ids never change.
    pub fn compact(&mut self) {
        let mut used = vec![false; self.params.len()];
        for e in self.entities.values() {
            for p in e.kind.own_params() {
                used[p.index()] = true;
            }
        }
        for c in self.constraints.values() {
            for p in &c.aux {
                used[p.index()] = true;
            }
        }
        let mut remap = vec![ParamId(u32::MAX); used.len()];
        let mut store = ParamStore::default();
        for (i, keep) in used.iter().enumerate() {
            if *keep {
                let old = ParamId(u32::try_from(i).unwrap_or(u32::MAX));
                remap[i] = store.push(self.params.get(old), self.params.flags(old));
            }
        }
        for e in self.entities.values_mut() {
            e.kind.for_each_param_mut(|p| *p = remap[p.index()]);
        }
        for c in self.constraints.values_mut() {
            for p in &mut c.aux {
                *p = remap[p.index()];
            }
        }
        self.params = store;
        self.retired.clear();
        self.touch_structure();
    }

    // ---- Internals --------------------------------------------------------------------

    fn view(&self) -> View<'_> {
        View {
            params: &self.params,
            entities: &self.entities,
            constraints: &self.constraints,
        }
    }

    fn push_point(&mut self, p: Point2) -> EntityId {
        let x = self.params.push(p.x, ParamFlags::default());
        let y = self.params.push(p.y, ParamFlags::default());
        self.insert_entity(EntityKind::Point { x, y }, false, EntityOrigin::User)
    }

    fn insert_entity(
        &mut self,
        kind: EntityKind,
        construction: bool,
        origin: EntityOrigin,
    ) -> EntityId {
        let id = EntityId(self.next_entity);
        self.next_entity += 1;
        self.entities.insert(
            id,
            Entity {
                kind,
                construction,
                origin,
            },
        );
        self.touch_structure();
        id
    }

    fn param_in_use(&self, p: ParamId) -> bool {
        self.entities
            .values()
            .any(|e| e.kind.own_params().contains(&p))
            || self.constraints.values().any(|c| c.aux.contains(&p))
    }

    fn dimension_mut(&mut self, id: ConstraintId) -> Result<&mut super::Dimension, SketchError> {
        match self.constraints.get_mut(&id) {
            None => Err(SketchError::UnknownConstraint(id)),
            Some(Constraint {
                kind: ConstraintKind::Dimension(d),
                ..
            }) => Ok(d),
            Some(_) => Err(SketchError::NotADimension(id)),
        }
    }

    fn touch_structure(&mut self) {
        self.structure_rev = next_rev();
        self.values_rev = next_rev();
    }

    fn touch_values(&mut self) {
        self.values_rev = next_rev();
    }
}
