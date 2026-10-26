//! Sketch-mode edits: [`SketchEdit`] and [`SketchFeature::apply`].
//!
//! An edit runs on a clone of the feature and replaces it only on success, so a failed edit
//! (including a failed [`SketchEdit::Batch`]) changes nothing. The caller then issues
//! `Command::EditFeature` with the edited feature: one edit, one undo entry.

use amg_core::{Dimension as Dim, Point2};
use amg_doc::{Expr, ExprSrc, ParamTable, UnitPrefs};
use amg_sketch::{
    ConstraintId, ConstraintKind, EditResult, EntityId, EntityKind, EntityOrigin, ParamFlags,
    ParamId, Quantity, Sketch, SketchError,
};
use serde::{Deserialize, Serialize};

use super::SketchGeom;
use super::{PlaneInput, SketchFeature};

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
        let res = work.apply_inner(edit)?;
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
        work.reconcile();
        *self = work;
        Ok(out)
    }

    /// The edit that stores the values of a solved sketch (`geom` from the last recompute) as
    /// this feature's warm start and last good geometry.
    pub fn commit_solved(&self, geom: &SketchGeom) -> SketchEdit {
        let old = self.sketch.params();
        let new = geom.sketch.params();
        let values = (0..old.len().min(new.len()))
            .filter_map(|i| {
                let id = ParamId(u32::try_from(i).ok()?);
                (old.get(id).to_bits() != new.get(id).to_bits()).then(|| (id, new.get(id)))
            })
            .collect();
        SketchEdit::CommitDrag { values }
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
