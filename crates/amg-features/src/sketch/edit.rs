//! Sketch-mode edits: [`SketchEdit`] and [`SketchFeature::apply`].
//!
//! An edit runs on a clone of the feature and replaces it only on success, so a failed edit
//! (including a failed [`SketchEdit::Batch`]) changes nothing. The caller then issues
//! `Command::EditFeature` with the edited feature: one edit, one undo entry.

use std::collections::BTreeSet;

use amg_core::{Dimension as Dim, Point2};
use amg_doc::{EntityRef, Expr, ExprSrc, ParamTable, TopoRef, UnitPrefs};
use amg_sketch::{
    ConstraintId, ConstraintKind, EditResult, EntityId, EntityKind, EntityOrigin, ExternalLinkId,
    ParamFlags, ParamId, ProjectedGeom, Quantity, Sketch, SketchError, SplineDef,
};
use serde::{Deserialize, Serialize};

use super::SketchGeom;
use super::{PlaneInput, ProjectionLink, ProjectionMode, SketchFeature};

/// One undoable change of a sketch feature. Ids are the sketch's stable ids.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum SketchEdit {
    /// Reserves a parameter slot for an entity added by [`AddEntity`](Self::AddEntity).
    AllocParam {
        /// Initial value (finite).
        value: f64,
        /// The solver never moves a locked parameter.
        locked: bool,
    },
    /// Adds an entity whose parameters were allocated by [`AllocParam`](Self::AllocParam).
    AddEntity {
        /// Geometry.
        kind: EntityKind,
        /// Construction geometry is excluded from profiles.
        construction: bool,
    },
    /// Adds a free point.
    AddPoint {
        /// Position, mm.
        at: Point2,
    },
    /// Adds a line with two new end points.
    AddLine {
        /// Start, mm.
        a: Point2,
        /// End, mm.
        b: Point2,
    },
    /// Adds a circle with a new centre point.
    AddCircle {
        /// Centre, mm.
        center: Point2,
        /// Radius, mm.
        radius: f64,
    },
    /// Removes entities, their dependents and the constraints touching them.
    RemoveEntities(Vec<EntityId>),
    /// Adds a constraint. A driving dimension takes `expr` as its value, or its stored value as
    /// a literal when `expr` is `None`.
    AddConstraint {
        /// The constraint.
        kind: ConstraintKind,
        /// Value expression (dimensions only).
        expr: Option<Expr>,
    },
    /// Removes a constraint.
    RemoveConstraint(ConstraintId),
    /// Replaces the value expression of a driving dimension.
    SetDimension {
        /// The dimension.
        id: ConstraintId,
        /// New value.
        expr: Expr,
    },
    /// Makes a dimension driving (value from an expression) or driven (measured).
    SetDriving {
        /// The dimension.
        id: ConstraintId,
        /// New state.
        driving: bool,
    },
    /// Marks an entity as construction geometry, or not.
    SetConstruction {
        /// The entity.
        id: EntityId,
        /// New state.
        on: bool,
    },
    /// Moves a dimension label (UI only).
    SetDimensionLabel {
        /// The dimension.
        id: ConstraintId,
        /// New label position in sketch coordinates.
        pos: Point2,
    },
    /// Moves the sketch to another plane or reference.
    SetPlane(PlaneInput),
    /// Projects model geometry into the sketch (SK-09): adds one locked entity per piece of
    /// `geometry` (computed by [`project_reference`](super::project_reference)) and a
    /// [`ProjectionLink`] that refreshes them at every recompute.
    AddProjection {
        /// The projected model entity.
        source: EntityRef,
        /// How it is projected.
        mode: ProjectionMode,
        /// Its present geometry in sketch coordinates.
        geometry: Vec<ProjectedGeom>,
    },
    /// Removes a projection link with its entities (and what is built on them).
    RemoveProjection(ExternalLinkId),
    /// Stores new geometry in a projected entity of the same kind
    /// ([`SketchFeature::commit_solved`]; recompute overwrites it while its source resolves).
    SetProjected {
        /// The projected entity.
        entity: EntityId,
        /// Its geometry.
        geometry: ProjectedGeom,
    },
    /// Writes final parameter values, the result of a drag (the only drag step that is undoable).
    CommitDrag {
        /// New values.
        values: Vec<(ParamId, f64)>,
    },
    /// Edits applied in order as one.
    Batch(Vec<SketchEdit>),
}

impl SketchFeature {
    /// Applies `edit`. On `Err` the feature is unchanged.
    pub fn apply(&mut self, edit: SketchEdit) -> Result<EditResult, SketchError> {
        let mut work = self.clone();
        let mut res = work.apply_inner(edit)?;
        work.reconcile_links(&mut res)?;
        work.reconcile();
        *self = work;
        Ok(res)
    }

    /// Runs `f` on the sketch (a builder or an `amg_sketch::edit` operation such as `trim` or
    /// `fillet`) and keeps the dimension expressions in step. On `Err` the feature is unchanged.
    pub fn edit_sketch<R>(
        &mut self,
        f: impl FnOnce(&mut Sketch) -> Result<R, SketchError>,
    ) -> Result<R, SketchError> {
        let mut work = self.clone();
        let out = f(&mut work.sketch)?;
        work.reconcile_links(&mut EditResult::default())?;
        work.reconcile();
        *self = work;
        Ok(out)
    }

    /// The edit that stores the values of a solved sketch (`geom` from the last recompute) as
    /// this feature's warm start and last good geometry, including the refreshed projections
    /// (what the sketch keeps if a source is lost later).
    pub fn commit_solved(&self, geom: &SketchGeom) -> SketchEdit {
        let old = self.sketch.params();
        let new = geom.sketch.params();
        let values = (0..old.len().min(new.len()))
            .filter_map(|i| {
                let id = ParamId(u32::try_from(i).ok()?);
                (old.get(id).to_bits() != new.get(id).to_bits()).then(|| (id, new.get(id)))
            })
            .collect();
        let commit = SketchEdit::CommitDrag { values };
        // A refreshed projected spline may also have new knots, which are not parameters.
        let mut edits: Vec<SketchEdit> = geom
            .sketch
            .entities()
            .filter_map(|(id, e)| {
                let old = self.sketch.entity(id)?;
                if !matches!(e.origin, EntityOrigin::Projected(_))
                    || spline_knots(&old.kind) == spline_knots(&e.kind)
                {
                    return None;
                }
                Some(SketchEdit::SetProjected {
                    entity: id,
                    geometry: spline_geom(&geom.sketch, &e.kind)?,
                })
            })
            .collect();
        if edits.is_empty() {
            return commit;
        }
        edits.push(commit);
        SketchEdit::Batch(edits)
    }

    fn apply_inner(&mut self, edit: SketchEdit) -> Result<EditResult, SketchError> {
        let mut res = EditResult::default();
        match edit {
            SketchEdit::AllocParam { value, locked } => {
                self.sketch.alloc_param(value, ParamFlags { locked })?;
            }
            SketchEdit::AddEntity { kind, construction } => {
                let id = self
                    .sketch
                    .add_entity(kind, construction, EntityOrigin::User)?;
                res.created.push(id);
            }
            SketchEdit::AddPoint { at } => res.created.push(self.sketch.add_point(at)?),
            SketchEdit::AddLine { a, b } => res.created.push(self.sketch.add_line(a, b)?),
            SketchEdit::AddCircle { center, radius } => {
                res.created.push(self.sketch.add_circle(center, radius)?);
            }
            SketchEdit::RemoveEntities(ids) => {
                let rep = self.sketch.remove_entities(&ids)?;
                res.removed = rep.entities;
                res.constraints_removed = rep.constraints;
            }
            SketchEdit::AddConstraint { kind, expr } => {
                let is_dim = matches!(kind, ConstraintKind::Dimension(_));
                if expr.is_some() && !is_dim {
                    return Err(SketchError::NotApplicable(
                        "only a dimension takes a value expression",
                    ));
                }
                let id = self.sketch.add_constraint(kind)?;
                if let Some(expr) = expr {
                    self.set_expr(id, expr)?;
                }
                res.constraints_added.push(id);
            }
            SketchEdit::RemoveConstraint(id) => {
                self.sketch.remove_constraint(id)?;
                res.constraints_removed.push(id);
            }
            SketchEdit::SetDimension { id, expr } => {
                if self.quantity(id).is_none() {
                    return Err(SketchError::NotADimension(id));
                }
                self.set_expr(id, expr)?;
            }
            SketchEdit::SetDriving { id, driving } => self.sketch.set_driving(id, driving)?,
            SketchEdit::SetConstruction { id, on } => self.sketch.set_construction(id, on)?,
            SketchEdit::SetDimensionLabel { id, pos } => self.set_label(id, pos)?,
            SketchEdit::SetPlane(plane) => self.plane = plane,
            SketchEdit::AddProjection {
                mut source,
                mode,
                geometry,
            } => {
                if geometry.is_empty() {
                    return Err(SketchError::NotApplicable("there is nothing to project"));
                }
                let link = self.next_link()?;
                let key = self.next_selection_key();
                if let Some(r) = topo_mut(&mut source) {
                    r.key = key;
                }
                for g in &geometry {
                    res.created.push(self.sketch.add_projected(link, g)?);
                }
                self.links.insert(
                    link,
                    ProjectionLink {
                        source,
                        mode,
                        entities: res.created.clone(),
                    },
                );
            }
            SketchEdit::RemoveProjection(link) => {
                let Some(l) = self.links.remove(&link) else {
                    return Err(SketchError::NotApplicable("there is no such projection"));
                };
                let ids: Vec<EntityId> = l
                    .entities
                    .into_iter()
                    .filter(|&e| self.sketch.entity(e).is_some())
                    .collect();
                let rep = self.sketch.remove_entities(&ids)?;
                res.removed = rep.entities;
                res.constraints_removed = rep.constraints;
                self.sweep_points(link, &BTreeSet::new(), &mut res)?;
            }
            SketchEdit::SetProjected { entity, geometry } => {
                self.sketch.refresh_projected(&[(entity, &geometry)])?;
            }
            SketchEdit::CommitDrag { values } => {
                for (p, v) in values {
                    if !self.sketch.params().contains(p) {
                        return Err(SketchError::UnknownParam(p));
                    }
                    if !v.is_finite() {
                        return Err(SketchError::InvalidValue {
                            what: "parameter",
                            value: v,
                        });
                    }
                    self.sketch.params_mut().set(p, v);
                }
                // The values must still describe valid geometry (a circle radius stays
                // positive, ...): the sketch's own loader checks that.
                let valid = serde_json::to_value(&self.sketch)
                    .ok()
                    .and_then(|j| serde_json::from_value::<Sketch>(j).ok());
                if valid.is_none() {
                    return Err(SketchError::NotApplicable(
                        "the values violate the sketch's invariants",
                    ));
                }
            }
            SketchEdit::Batch(edits) => {
                for e in edits {
                    let r = self.apply_inner(e)?;
                    res.created.extend(r.created);
                    res.removed.extend(r.removed);
                    res.constraints_added.extend(r.constraints_added);
                    res.constraints_removed.extend(r.constraints_removed);
                }
            }
        }
        Ok(res)
    }

    /// Stores `expr` for dimension `id`, and its value in the sketch when it needs no
    /// parameters (a stored value that still follows its expression keeps the warm start close).
    fn set_expr(&mut self, id: ConstraintId, expr: Expr) -> Result<(), SketchError> {
        let q = self.quantity(id).ok_or(SketchError::NotADimension(id))?;
        if let Some(v) = literal_value(&expr, q) {
            self.sketch.set_dimension_value(id, v)?;
        }
        self.dims.insert(id, expr);
        Ok(())
    }

    fn set_label(&mut self, id: ConstraintId, pos: Point2) -> Result<(), SketchError> {
        if !(pos.x.is_finite() && pos.y.is_finite()) {
            return Err(SketchError::InvalidValue {
                what: "dimension label position",
                value: if pos.x.is_finite() { pos.y } else { pos.x },
            });
        }
        if self.quantity(id).is_none() {
            return Err(SketchError::NotADimension(id));
        }
        // `amg-sketch` has no label setter yet: go through its serde form (labels are UI-only).
        let mut json = serde_json::to_value(&self.sketch).map_err(|_| {
            SketchError::NotApplicable("the sketch has non-finite values and can't be edited")
        })?;
        let label = json
            .pointer_mut(&format!("/constraints/{}/kind/Dimension/label_pos", id.0))
            .ok_or(SketchError::NotADimension(id))?;
        *label = serde_json::to_value(pos)
            .map_err(|_| SketchError::NotApplicable("label position can't be stored"))?;
        self.sketch = serde_json::from_value(json)
            .map_err(|_| SketchError::NotApplicable("label position can't be stored"))?;
        Ok(())
    }

    /// An id no link or projected entity uses.
    fn next_link(&self) -> Result<ExternalLinkId, SketchError> {
        let used = self
            .sketch
            .entities()
            .filter_map(|(_, e)| match e.origin {
                EntityOrigin::Projected(l) => Some(l),
                _ => None,
            })
            .chain(self.links.keys().copied())
            .max();
        match used {
            None => Ok(ExternalLinkId(0)),
            Some(l) => {
                l.0.checked_add(1)
                    .map(ExternalLinkId)
                    .ok_or(SketchError::NotApplicable("no projection id is left"))
            }
        }
    }

    /// The next selection key of the feature's topological references (naming note §5.1):
    /// one past the largest in use, so that re-picking one link can't touch another.
    fn next_selection_key(&self) -> u32 {
        let plane = match &self.plane {
            PlaneInput::Ref(r) => Some(r),
            _ => None,
        };
        self.links
            .values()
            .map(|l| &l.source)
            .chain(plane)
            .filter_map(topo)
            .map(|r| r.key)
            .max()
            .unwrap_or(0)
            .saturating_add(1)
    }

    /// Cleans up after edits that removed projected geometry: the points of a removed projected
    /// curve go with it, and a link with no entity left is dropped.
    fn reconcile_links(&mut self, res: &mut EditResult) -> Result<(), SketchError> {
        let links: Vec<(ExternalLinkId, BTreeSet<EntityId>)> = self
            .links
            .iter()
            .map(|(&link, l)| {
                // The listed entities and their points.
                let listed = l
                    .entities
                    .iter()
                    .flat_map(|&id| {
                        let children = self.sketch.entity(id).map(|e| e.kind.children());
                        children.into_iter().flatten().chain([id])
                    })
                    .collect();
                (link, listed)
            })
            .collect();
        for (link, listed) in links {
            self.sweep_points(link, &listed, res)?;
        }
        let sketch = &self.sketch;
        self.links
            .retain(|_, l| l.entities.iter().any(|&e| sketch.entity(e).is_some()));
        Ok(())
    }

    /// Removes the points of `link` that are not in `keep`. A point that a curve or constraint
    /// still uses stays and becomes ordinary user geometry, since nothing would refresh it.
    fn sweep_points(
        &mut self,
        link: ExternalLinkId,
        keep: &BTreeSet<EntityId>,
        res: &mut EditResult,
    ) -> Result<(), SketchError> {
        let orphans: Vec<EntityId> = self
            .sketch
            .entities()
            .filter(|(id, e)| {
                e.origin == EntityOrigin::Projected(link)
                    && matches!(e.kind, EntityKind::Point { .. })
                    && !keep.contains(id)
            })
            .map(|(id, _)| id)
            .collect();
        for id in orphans {
            let used = self
                .sketch
                .entities()
                .any(|(_, e)| e.kind.children().contains(&id))
                || self
                    .sketch
                    .constraints()
                    .any(|(_, c)| c.kind.entities().contains(&id));
            if used {
                self.sketch.release_projected(id)?;
            } else {
                res.removed
                    .extend(self.sketch.remove_entities(&[id])?.entities);
            }
        }
        Ok(())
    }

    /// Makes `dims` hold exactly the driving dimensions of the sketch: drops the expressions of
    /// removed and driven dimensions, and gives a driving dimension without one a literal of its
    /// current value.
    fn reconcile(&mut self) {
        let sketch = &self.sketch;
        let driving = |id: &ConstraintId| {
            matches!(
                sketch.constraint(*id).map(|c| &c.kind),
                Some(ConstraintKind::Dimension(d)) if d.driving
            )
        };
        self.dims.retain(|id, _| driving(id));
        for (id, c) in sketch.constraints() {
            if let ConstraintKind::Dimension(d) = &c.kind
                && d.driving
                && !self.dims.contains_key(&id)
            {
                let unit = match d.kind.quantity() {
                    Quantity::Length => "mm",
                    Quantity::Angle => "rad",
                };
                self.dims
                    .insert(id, Expr::Inline(ExprSrc(format!("{} {unit}", d.value))));
            }
        }
    }
}

/// The topological reference of `r`, also when seen through occurrences.
fn topo(r: &EntityRef) -> Option<&TopoRef> {
    match r {
        EntityRef::Topo(t) => Some(t),
        EntityRef::InContext { target, .. } => topo(target),
        _ => None,
    }
}

fn topo_mut(r: &mut EntityRef) -> Option<&mut TopoRef> {
    match r {
        EntityRef::Topo(t) => Some(t),
        EntityRef::InContext { target, .. } => topo_mut(target),
        _ => None,
    }
}

/// The knot vector of a control-point spline.
fn spline_knots(kind: &EntityKind) -> Option<&[f64]> {
    match kind {
        EntityKind::Spline(SplineDef::Control { knots, .. }) => Some(knots),
        _ => None,
    }
}

/// A control-point spline entity of `sketch` as projected geometry.
fn spline_geom(sketch: &Sketch, kind: &EntityKind) -> Option<ProjectedGeom> {
    let EntityKind::Spline(SplineDef::Control {
        degree,
        poles,
        weights,
        knots,
        ..
    }) = kind
    else {
        return None;
    };
    Some(ProjectedGeom::Spline {
        degree: *degree,
        knots: knots.clone(),
        poles: poles
            .iter()
            .map(|&p| sketch.point(p))
            .collect::<Option<_>>()?,
        weights: weights
            .as_ref()
            .map(|w| w.iter().map(|&p| sketch.params().get(p)).collect()),
    })
}

/// Value of an inline expression that needs no parameters, in mm or rad.
fn literal_value(expr: &Expr, q: Quantity) -> Option<f64> {
    let Expr::Inline(_) = expr else { return None };
    let dim = match q {
        Quantity::Length => Dim::LENGTH,
        Quantity::Angle => Dim::ANGLE,
    };
    ParamTable::default()
        .resolve(expr, dim, &UnitPrefs::default())
        .ok()
        .map(|q| q.value)
}
