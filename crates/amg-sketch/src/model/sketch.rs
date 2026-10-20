//! The [`Sketch`] container.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicU64, Ordering};

use amg_core::{Plane, Point2, Segment2, Tolerance};
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use super::validate::{self, View};
use super::{Constraint, ConstraintKind, Entity, EntityKind, EntityOrigin, ParamFlags, ParamStore};
use crate::error::SketchError;
use crate::geom::{Arc2, Circle2, Curve2};
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
///   ownership), so a loaded sketch upholds the same invariants as one built through the API.
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
    next_entity: u32,
    next_constraint: u32,
}

impl TryFrom<SketchRepr> for Sketch {
    type Error = SketchError;
    fn try_from(r: SketchRepr) -> Result<Self, SketchError> {
        let view = View {
            params: &r.params,
            entities: &r.entities,
        };
        validate::whole(
            &view,
            &r.constraints,
            r.next_entity,
            r.next_constraint,
            r.params.flags.len(),
        )?;
        Ok(Sketch {
            plane: r.plane,
            params: r.params,
            entities: r.entities,
            constraints: r.constraints,
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

fn finite_point(p: Point2) -> Result<(), SketchError> {
    for v in [p.x, p.y] {
        if !v.is_finite() {
            return Err(SketchError::InvalidValue {
                what: "point coordinate",
                value: v,
            });
        }
    }
    Ok(())
}

impl Sketch {
    /// Empty sketch on `plane`.
    pub fn new(plane: Plane) -> Self {
        Sketch {
            plane,
            params: ParamStore::default(),
            entities: BTreeMap::new(),
            constraints: BTreeMap::new(),
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

    /// Adds a free user point. `p` should be finite (debug-asserted).
    pub fn add_point(&mut self, p: Point2) -> EntityId {
        debug_assert!(p.is_finite(), "Sketch::add_point: non-finite {p:?}");
        let x = self.params.push(p.x, ParamFlags::default());
        let y = self.params.push(p.y, ParamFlags::default());
        self.insert_entity(EntityKind::Point { x, y }, false, EntityOrigin::User)
    }

    /// Adds a line from `a` to `b`, creating its two end points.
    pub fn add_line(&mut self, a: Point2, b: Point2) -> EntityId {
        let start = self.add_point(a);
        let end = self.add_point(b);
        self.insert_entity(EntityKind::Line { start, end }, false, EntityOrigin::User)
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
        finite_point(center)?;
        if !(r.is_finite() && r > 0.0) {
            return Err(SketchError::InvalidValue {
                what: "circle radius",
                value: r,
            });
        }
        let center = self.add_point(center);
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
    /// `end` coincides with `center` (within `Tolerance::DEFAULT.linear`).
    pub fn add_arc(
        &mut self,
        center: Point2,
        start: Point2,
        end: Point2,
    ) -> Result<EntityId, SketchError> {
        for p in [center, start, end] {
            finite_point(p)?;
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
        let end = center + de * (r / de_len);
        let c = self.add_point(center);
        let s = self.add_point(start);
        let e = self.add_point(end);
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

    /// Adds an entity of any kind referencing existing params and points.
    ///
    /// Validates that the params exist and are not owned by another entity or constraint,
    /// that children exist, are points and are distinct, that point coordinates are finite
    /// and a circle radius is positive. `Projected` entities get their own params locked.
    pub fn add_entity(
        &mut self,
        kind: EntityKind,
        construction: bool,
        origin: EntityOrigin,
    ) -> Result<EntityId, SketchError> {
        self.view().entity(&kind, true)?;
        let own = kind.own_params();
        if own.iter().any(|&p| self.param_in_use(p)) {
            return Err(SketchError::NotApplicable(
                "a parameter is already owned by another entity or constraint",
            ));
        }
        if matches!(origin, EntityOrigin::Projected(_)) {
            for &p in &own {
                self.params.set_flags(p, ParamFlags { locked: true });
            }
        }
        Ok(self.insert_entity(kind, construction, origin))
    }

    /// Allocates a new parameter slot (for [`add_entity`](Sketch::add_entity)). Slots no
    /// entity or constraint owns are dropped by [`compact`](Sketch::compact).
    pub fn alloc_param(&mut self, value: f64, flags: ParamFlags) -> ParamId {
        self.touch_structure();
        self.params.push(value, flags)
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
    ///   (removing a line removes its end points unless another curve shares them).
    ///
    /// Nothing is removed if any id is unknown (`Err(UnknownEntity)`). Param slots stay until
    /// [`compact`](Sketch::compact).
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
                .filter(|c| !used_by_survivors.contains(c))
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
            self.entities.remove(id);
        }
        for id in &constraints {
            self.constraints.remove(id);
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
        self.view().constraint(&kind, true)?;
        let id = ConstraintId(self.next_constraint);
        self.next_constraint += 1;
        self.constraints.insert(
            id,
            Constraint {
                kind,
                aux: SmallVec::new(),
            },
        );
        self.touch_structure();
        Ok(id)
    }

    /// Removes one constraint. Its aux param slots stay until [`compact`](Sketch::compact).
    pub fn remove_constraint(&mut self, id: ConstraintId) -> Result<(), SketchError> {
        self.constraints
            .remove(&id)
            .ok_or(SketchError::UnknownConstraint(id))?;
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
        self.touch_structure();
    }

    // ---- Internals --------------------------------------------------------------------

    fn view(&self) -> View<'_> {
        View {
            params: &self.params,
            entities: &self.entities,
        }
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
