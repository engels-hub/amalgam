//! 3D sketches (SK-12): points, lines and clamped B-splines in model space, with 3D constraints
//! solved by the SK-03 solver and analysed by the SK-04 DOF code through the
//! dimension-agnostic [`system`](crate::solve::system) layer.
//!
//! - Entities ([`Entity3`]): free points, **model vertices** (points whose coordinates are
//!   fixed: projected corners of solid bodies, refreshed with [`Sketch3d::move_model_vertex`]),
//!   lines between two points and clamped non-rational B-splines through point poles.
//! - Constraints ([`Constraint3`]): coincidence (also to a model vertex), point on line, point
//!   on spline, distance and length, angle, parallel, perpendicular, equal length, parallel to
//!   a model axis and line–spline end tangency. Every one is a [`Residual`] with an analytic
//!   Jacobian; rows are mm (angular rows multiplied by `L_c`, as in 2D).
//! - Coincidence and axis constraints are exactly `p − q = 0` and are substituted (aliased)
//!   like 2D coincidence / horizontal / vertical; a point coincident with a model vertex
//!   becomes a constant.
//! - [`Sketch3d::solve`] runs the [`NativeSolver`] pipeline (components, Dulmage–Mendelsohn
//!   blocks, damped minimum-norm LM, rank test) and [`Sketch3d::analyze`] the SK-04 null-space
//!   DOF analysis with per-entity status and dependency circuits. Not available in 3D (they
//!   re-solve a 2D [`Sketch`](crate::Sketch)): the witness tie-break and leave-one-out
//!   refinement; see [`solve::native`](crate::solve::NativeSolver) `generic`.
//!
//! Lengths are mm in model coordinates, angles radians, all `f64`. Ids are never reused.
//!
//! [`Residual`]: crate::solve::Residual

mod residual;

use std::collections::BTreeMap;

use amg_core::Point3;
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use crate::error::SketchError;
use crate::geom::{check_knots, clamped_uniform_knots};
use crate::ids::{ConstraintId, EntityId, ExternalLinkId, ParamId};
use crate::solve::system::{EquationOwner, EquationSystem};
use crate::solve::{Diagnosis, DofReport, NativeSolver, SketchSystem, SolveOptions, SolveReport};

use self::residual::Res3;

/// A 3D sketch entity.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Entity3 {
    /// A free point `(x, y, z)`.
    Point([ParamId; 3]),
    /// A model vertex: a point whose coordinates are fixed (not solved).
    ModelVertex {
        /// Its coordinates.
        at: [ParamId; 3],
        /// The model topology it was taken from (resolved by the owning feature).
        link: Option<ExternalLinkId>,
    },
    /// A straight segment between two points (or model vertices).
    Line {
        /// Start point.
        start: EntityId,
        /// End point.
        end: EntityId,
    },
    /// A clamped non-rational B-spline whose poles are points (or model vertices); it starts at
    /// the first pole and ends at the last.
    Spline {
        /// Degree, 1..=10.
        degree: usize,
        /// Pole points, at least `degree + 1`.
        poles: Vec<EntityId>,
        /// Clamped knot vector (`poles.len() + degree + 1` values).
        knots: Vec<f64>,
    },
}

impl Entity3 {
    fn kind_name(&self) -> &'static str {
        match self {
            Entity3::Point(_) => "point",
            Entity3::ModelVertex { .. } => "model vertex",
            Entity3::Line { .. } => "line",
            Entity3::Spline { .. } => "spline",
        }
    }
}

/// A model axis.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Axis {
    /// The X axis.
    X,
    /// The Y axis.
    Y,
    /// The Z axis.
    Z,
}

/// Which end of a spline.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SplineEnd {
    /// The first pole.
    Start,
    /// The last pole.
    End,
}

/// A 3D constraint. "Point" means a point or a model vertex.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Constraint3 {
    /// Two points coincide (3 equations, substituted).
    Coincident {
        /// First point.
        a: EntityId,
        /// Second point.
        b: EntityId,
    },
    /// A point lies on the infinite line through a line's ends (3 equations, one auxiliary
    /// line parameter).
    PointOnLine {
        /// The point.
        point: EntityId,
        /// The line.
        line: EntityId,
    },
    /// A point lies on a spline (3 equations, one auxiliary curve parameter).
    PointOnSpline {
        /// The point.
        point: EntityId,
        /// The spline.
        spline: EntityId,
    },
    /// Distance between two points, mm (> 0).
    Distance {
        /// First point.
        a: EntityId,
        /// Second point.
        b: EntityId,
        /// The distance.
        value: f64,
    },
    /// Length of a line, mm (> 0).
    Length {
        /// The line.
        line: EntityId,
        /// The length.
        value: f64,
    },
    /// Angle between the directions (start → end) of two lines, radians in `(0, π)`; use
    /// [`Parallel`](Constraint3::Parallel) for 0 or π.
    Angle {
        /// First line.
        a: EntityId,
        /// Second line.
        b: EntityId,
        /// The angle.
        value: f64,
    },
    /// Two lines are parallel or anti-parallel (3 equations, one auxiliary ratio).
    Parallel {
        /// First line.
        a: EntityId,
        /// Second line.
        b: EntityId,
    },
    /// Two lines are perpendicular (1 equation).
    Perpendicular {
        /// First line.
        a: EntityId,
        /// Second line.
        b: EntityId,
    },
    /// Two lines have the same length (1 equation).
    EqualLength {
        /// First line.
        a: EntityId,
        /// Second line.
        b: EntityId,
    },
    /// A line is parallel to a model axis (2 equations, substituted).
    AlongAxis {
        /// The line.
        line: EntityId,
        /// The axis.
        axis: Axis,
    },
    /// A line is tangent to a spline at one end: the line is parallel to the end leg of the
    /// control polygon (3 equations, one auxiliary ratio). Coincidence is a separate constraint.
    Tangent {
        /// The line.
        line: EntityId,
        /// The spline.
        spline: EntityId,
        /// Which end.
        end: SplineEnd,
    },
}

impl Constraint3 {
    /// The entities the constraint references.
    pub fn entities(&self) -> SmallVec<[EntityId; 4]> {
        use Constraint3::*;
        match *self {
            Coincident { a, b }
            | Distance { a, b, .. }
            | Angle { a, b, .. }
            | Parallel { a, b }
            | Perpendicular { a, b }
            | EqualLength { a, b } => SmallVec::from_slice(&[a, b]),
            PointOnLine { point, line } => SmallVec::from_slice(&[point, line]),
            PointOnSpline { point, spline } => SmallVec::from_slice(&[point, spline]),
            Length { line, .. } | AlongAxis { line, .. } => SmallVec::from_slice(&[line]),
            Tangent { line, spline, .. } => SmallVec::from_slice(&[line, spline]),
        }
    }

    /// Number of auxiliary parameters (unknowns owned by the constraint).
    fn aux_count(&self) -> usize {
        match self {
            Constraint3::PointOnLine { .. }
            | Constraint3::PointOnSpline { .. }
            | Constraint3::Parallel { .. }
            | Constraint3::Tangent { .. } => 1,
            _ => 0,
        }
    }
}

/// A constraint with its auxiliary parameters.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Constraint3Entry {
    /// The constraint.
    pub kind: Constraint3,
    /// Auxiliary unknowns (line / curve parameter, parallel ratio).
    pub aux: SmallVec<[ParamId; 1]>,
}

/// A 3D sketch: parameters, entities and constraints.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Sketch3d {
    params: Vec<f64>,
    /// Per parameter: kept constant by the solver.
    fixed: Vec<bool>,
    entities: BTreeMap<EntityId, Entity3>,
    constraints: BTreeMap<ConstraintId, Constraint3Entry>,
    next_entity: u32,
    next_constraint: u32,
}

impl Sketch3d {
    /// An empty sketch.
    pub fn new() -> Self {
        Self::default()
    }

    fn alloc(&mut self, v: f64, fixed: bool) -> ParamId {
        self.params.push(v);
        self.fixed.push(fixed);
        ParamId((self.params.len() - 1) as u32)
    }

    fn alloc3(&mut self, p: Point3, fixed: bool) -> Result<[ParamId; 3], SketchError> {
        for v in [p.x, p.y, p.z] {
            if !v.is_finite() {
                return Err(SketchError::InvalidValue {
                    what: "coordinate",
                    value: v,
                });
            }
        }
        Ok([
            self.alloc(p.x, fixed),
            self.alloc(p.y, fixed),
            self.alloc(p.z, fixed),
        ])
    }

    fn insert(&mut self, e: Entity3) -> EntityId {
        let id = EntityId(self.next_entity);
        self.next_entity += 1;
        self.entities.insert(id, e);
        id
    }

    /// Adds a free point.
    pub fn add_point(&mut self, p: Point3) -> Result<EntityId, SketchError> {
        let ps = self.alloc3(p, false)?;
        Ok(self.insert(Entity3::Point(ps)))
    }

    /// Adds a model vertex: a point at `at` that the solver keeps fixed.
    pub fn add_model_vertex(
        &mut self,
        at: Point3,
        link: Option<ExternalLinkId>,
    ) -> Result<EntityId, SketchError> {
        let ps = self.alloc3(at, true)?;
        Ok(self.insert(Entity3::ModelVertex { at: ps, link }))
    }

    /// Moves a model vertex (the model changed); the next solve follows it.
    pub fn move_model_vertex(&mut self, id: EntityId, at: Point3) -> Result<(), SketchError> {
        let Some(Entity3::ModelVertex { at: ps, .. }) = self.entities.get(&id) else {
            return Err(self.wrong_kind(id, "model vertex"));
        };
        let ps = *ps;
        for (p, v) in ps.iter().zip([at.x, at.y, at.z]) {
            if !v.is_finite() {
                return Err(SketchError::InvalidValue {
                    what: "coordinate",
                    value: v,
                });
            }
            self.params[p.index()] = v;
        }
        Ok(())
    }

    /// Adds a line between two existing points.
    pub fn add_line(&mut self, start: EntityId, end: EntityId) -> Result<EntityId, SketchError> {
        self.point_params(start)?;
        self.point_params(end)?;
        if start == end {
            return Err(SketchError::Degenerate("a line needs two different points"));
        }
        Ok(self.insert(Entity3::Line { start, end }))
    }

    /// Adds a line with two new free points.
    pub fn add_line_points(&mut self, a: Point3, b: Point3) -> Result<EntityId, SketchError> {
        let s = self.add_point(a)?;
        let e = self.add_point(b)?;
        self.add_line(s, e)
    }

    /// Adds a clamped B-spline of `degree` through existing pole points, with uniform knots.
    pub fn add_spline(
        &mut self,
        degree: usize,
        poles: &[EntityId],
    ) -> Result<EntityId, SketchError> {
        let knots = clamped_uniform_knots(poles.len(), degree);
        self.add_spline_knots(degree, poles, knots)
    }

    /// Adds a clamped B-spline with an explicit knot vector.
    pub fn add_spline_knots(
        &mut self,
        degree: usize,
        poles: &[EntityId],
        knots: Vec<f64>,
    ) -> Result<EntityId, SketchError> {
        check_knots(degree, poles.len(), &knots)?;
        for &p in poles {
            self.point_params(p)?;
        }
        Ok(self.insert(Entity3::Spline {
            degree,
            poles: poles.to_vec(),
            knots,
        }))
    }

    /// Keeps a point's coordinates fixed (`true`) or solves them again (`false`).
    pub fn set_fixed(&mut self, point: EntityId, fixed: bool) -> Result<(), SketchError> {
        match self.entities.get(&point) {
            Some(Entity3::Point(ps)) => {
                for p in *ps {
                    self.fixed[p.index()] = fixed;
                }
                Ok(())
            }
            Some(Entity3::ModelVertex { .. }) => Err(SketchError::Locked(point)),
            _ => Err(self.wrong_kind(point, "point")),
        }
    }

    /// Adds a constraint (validated, auxiliary parameters initialised from the current
    /// geometry).
    pub fn add_constraint(&mut self, kind: Constraint3) -> Result<ConstraintId, SketchError> {
        self.validate(&kind)?;
        let mut aux = SmallVec::new();
        if kind.aux_count() == 1 {
            let v = residual::aux_start(self, &kind);
            aux.push(self.alloc(v, false));
        }
        let id = ConstraintId(self.next_constraint);
        self.next_constraint += 1;
        self.constraints.insert(id, Constraint3Entry { kind, aux });
        Ok(id)
    }

    /// Removes a constraint (its auxiliary parameters stay allocated and unused).
    pub fn remove_constraint(&mut self, id: ConstraintId) -> Result<Constraint3, SketchError> {
        self.constraints
            .remove(&id)
            .map(|c| c.kind)
            .ok_or(SketchError::UnknownConstraint(id))
    }

    /// Changes the value of a distance, length or angle constraint.
    pub fn set_value(&mut self, id: ConstraintId, v: f64) -> Result<(), SketchError> {
        let mut kind = self
            .constraints
            .get(&id)
            .ok_or(SketchError::UnknownConstraint(id))?
            .kind
            .clone();
        match &mut kind {
            Constraint3::Distance { value, .. }
            | Constraint3::Length { value, .. }
            | Constraint3::Angle { value, .. } => *value = v,
            _ => return Err(SketchError::NotADimension(id)),
        }
        self.validate(&kind)?;
        if let Some(c) = self.constraints.get_mut(&id) {
            c.kind = kind;
        }
        Ok(())
    }

    /// The entities, by id.
    pub fn entities(&self) -> impl Iterator<Item = (EntityId, &Entity3)> + '_ {
        self.entities.iter().map(|(&id, e)| (id, e))
    }

    /// One entity.
    pub fn entity(&self, id: EntityId) -> Option<&Entity3> {
        self.entities.get(&id)
    }

    /// The constraints, by id.
    pub fn constraints(&self) -> impl Iterator<Item = (ConstraintId, &Constraint3Entry)> + '_ {
        self.constraints.iter().map(|(&id, c)| (id, c))
    }

    /// One constraint.
    pub fn constraint(&self, id: ConstraintId) -> Option<&Constraint3Entry> {
        self.constraints.get(&id)
    }

    /// The parameter vector.
    pub fn params(&self) -> &[f64] {
        &self.params
    }

    /// True if the solver keeps `p` constant.
    pub fn is_fixed(&self, p: ParamId) -> bool {
        self.fixed.get(p.index()).copied().unwrap_or(false)
    }

    /// Position of a point or model vertex.
    pub fn point(&self, id: EntityId) -> Option<Point3> {
        let ps = self.point_params(id).ok()?;
        Some(Point3::new(
            self.params[ps[0].index()],
            self.params[ps[1].index()],
            self.params[ps[2].index()],
        ))
    }

    /// End points of a line.
    pub fn line(&self, id: EntityId) -> Option<(Point3, Point3)> {
        match self.entities.get(&id)? {
            Entity3::Line { start, end } => Some((self.point(*start)?, self.point(*end)?)),
            _ => None,
        }
    }

    /// Point of a spline at parameter `t` (clamped to the knot range).
    pub fn spline_point(&self, id: EntityId, t: f64) -> Option<Point3> {
        let Entity3::Spline {
            degree,
            poles,
            knots,
        } = self.entities.get(&id)?
        else {
            return None;
        };
        let pts: Vec<Point3> = poles
            .iter()
            .map(|&p| self.point(p))
            .collect::<Option<_>>()?;
        let t = t.clamp(knots[0], knots[knots.len() - 1]);
        Some(residual::spline_eval(*degree, knots, &pts, t).0)
    }

    /// Characteristic length `L_c` (mm) for the angular rows: the diagonal of the bounding box
    /// of all points, at least 1 mm.
    pub fn characteristic_length(&self) -> f64 {
        let (mut lo, mut hi) = (
            Point3::splat(f64::INFINITY),
            Point3::splat(f64::NEG_INFINITY),
        );
        for (&id, e) in &self.entities {
            if matches!(e, Entity3::Point(_) | Entity3::ModelVertex { .. })
                && let Some(p) = self.point(id)
            {
                lo = lo.min(p);
                hi = hi.max(p);
            }
        }
        if lo.x > hi.x {
            return 1.0;
        }
        (hi - lo).length().max(1.0)
    }

    /// The equation system: one block per constraint (by id); fixed parameters are model
    /// vertices and fixed points; free ones every other point coordinate and the auxiliary
    /// parameters.
    pub fn system(&self) -> Result<SketchSystem, SketchError> {
        let lc = self.characteristic_length();
        let mut system = EquationSystem::new();
        for (&id, c) in &self.constraints {
            system.push(
                EquationOwner::Constraint(id),
                Box::new(Res3::build(self, &c.kind, &c.aux, lc)?),
            );
        }
        let mut free = Vec::new();
        let mut fixed = std::collections::BTreeSet::new();
        let mut used = vec![false; self.params.len()];
        for e in self.entities.values() {
            if let Entity3::Point(ps) | Entity3::ModelVertex { at: ps, .. } = e {
                for p in ps {
                    used[p.index()] = true;
                }
            }
        }
        for c in self.constraints.values() {
            for p in &c.aux {
                used[p.index()] = true;
            }
        }
        for (i, &u) in used.iter().enumerate() {
            if !u {
                continue;
            }
            if self.fixed[i] {
                fixed.insert(ParamId(i as u32));
            } else {
                free.push(ParamId(i as u32));
            }
        }
        Ok(SketchSystem {
            system,
            fixed,
            free,
            length_scale: lc,
        })
    }

    /// Solves the sketch with `solver` (SK-03 pipeline). With `opts.dof` set the report carries
    /// the DOF; on failure (and `opts.diagnose`) the dependency groups. A cancelled solve
    /// leaves the sketch unchanged.
    pub fn solve(
        &mut self,
        solver: &mut NativeSolver,
        opts: &SolveOptions,
        cancel: &dyn Fn() -> bool,
    ) -> SolveReport {
        let sys = match self.system() {
            Ok(s) => s,
            Err(_) => return failed_report(self),
        };
        let mut x = self.params.clone();
        let report = {
            let entities = self.entity_params();
            let subst = |o: EquationOwner| residual::subst_pairs(self, o);
            let owners = |id: ConstraintId| self.constraint_entities(id);
            let problem = crate::solve::Problem {
                sys: &sys,
                n: self.params.len(),
                subst: &subst,
                entities: &entities,
                constraint_entities: &owners,
            };
            solver.solve_problem(&problem, &mut x, opts, cancel)
        };
        if report.status != crate::solve::SolveStatus::Cancelled {
            self.params = x;
        }
        report
    }

    /// DOF analysis (SK-04) at the current values: total DOF, per-entity status and
    /// dependency groups. Meaningful at a solved configuration.
    pub fn analyze(&self, solver: &NativeSolver) -> (DofReport, Diagnosis) {
        let Ok(sys) = self.system() else {
            return (DofReport::default(), Diagnosis::default());
        };
        let entities = self.entity_params();
        let subst = |o: EquationOwner| residual::subst_pairs(self, o);
        let owners = |id: ConstraintId| self.constraint_entities(id);
        let problem = crate::solve::Problem {
            sys: &sys,
            n: self.params.len(),
            subst: &subst,
            entities: &entities,
            constraint_entities: &owners,
        };
        solver.analyze_problem(&problem, &self.params)
    }

    // ---- internals --------------------------------------------------------------------

    fn wrong_kind(&self, id: EntityId, expected: &'static str) -> SketchError {
        match self.entities.get(&id) {
            None => SketchError::UnknownEntity(id),
            Some(e) => SketchError::WrongEntityKind {
                entity: id,
                expected,
                found: e.kind_name(),
            },
        }
    }

    /// Coordinates of a point or model vertex.
    fn point_params(&self, id: EntityId) -> Result<[ParamId; 3], SketchError> {
        match self.entities.get(&id) {
            Some(Entity3::Point(ps) | Entity3::ModelVertex { at: ps, .. }) => Ok(*ps),
            _ => Err(self.wrong_kind(id, "point")),
        }
    }

    fn line_ends(&self, id: EntityId) -> Result<(EntityId, EntityId), SketchError> {
        match self.entities.get(&id) {
            Some(Entity3::Line { start, end }) => Ok((*start, *end)),
            _ => Err(self.wrong_kind(id, "line")),
        }
    }

    fn spline(&self, id: EntityId) -> Result<(usize, &[EntityId], &[f64]), SketchError> {
        match self.entities.get(&id) {
            Some(Entity3::Spline {
                degree,
                poles,
                knots,
            }) => Ok((*degree, poles, knots)),
            _ => Err(self.wrong_kind(id, "spline")),
        }
    }

    fn validate(&self, kind: &Constraint3) -> Result<(), SketchError> {
        use Constraint3::*;
        let positive = |what: &'static str, value: f64| {
            if value.is_finite() && value > 0.0 {
                Ok(())
            } else {
                Err(SketchError::InvalidValue { what, value })
            }
        };
        match *kind {
            Coincident { a, b } => {
                self.point_params(a)?;
                self.point_params(b)?;
                if a == b {
                    return Err(SketchError::Degenerate("a point is coincident with itself"));
                }
            }
            PointOnLine { point, line } => {
                self.point_params(point)?;
                let (s, e) = self.line_ends(line)?;
                if point == s || point == e {
                    return Err(SketchError::NotApplicable(
                        "the point is an end of the line",
                    ));
                }
            }
            PointOnSpline { point, spline } => {
                self.point_params(point)?;
                self.spline(spline)?;
            }
            Distance { a, b, value } => {
                self.point_params(a)?;
                self.point_params(b)?;
                positive("distance", value)?;
            }
            Length { line, value } => {
                self.line_ends(line)?;
                positive("length", value)?;
            }
            Angle { a, b, value } => {
                self.line_ends(a)?;
                self.line_ends(b)?;
                if !(value > 1e-9 && value < std::f64::consts::PI - 1e-9) {
                    return Err(SketchError::InvalidValue {
                        what: "angle (use Parallel for 0 or π)",
                        value,
                    });
                }
            }
            Parallel { a, b } | Perpendicular { a, b } | EqualLength { a, b } => {
                self.line_ends(a)?;
                self.line_ends(b)?;
                if a == b {
                    return Err(SketchError::Degenerate("a line is related to itself"));
                }
            }
            AlongAxis { line, .. } => {
                self.line_ends(line)?;
            }
            Tangent { line, spline, .. } => {
                self.line_ends(line)?;
                self.spline(spline)?;
            }
        }
        Ok(())
    }

    /// Every entity with the parameters that move it.
    fn entity_params(&self) -> Vec<(EntityId, SmallVec<[ParamId; 8]>)> {
        let pts = |ids: &[EntityId]| -> SmallVec<[ParamId; 8]> {
            ids.iter()
                .filter_map(|&p| self.point_params(p).ok())
                .flatten()
                .collect()
        };
        self.entities
            .iter()
            .map(|(&id, e)| {
                let ps = match e {
                    Entity3::Point(ps) | Entity3::ModelVertex { at: ps, .. } => {
                        SmallVec::from_slice(ps)
                    }
                    Entity3::Line { start, end } => pts(&[*start, *end]),
                    Entity3::Spline { poles, .. } => pts(poles),
                };
                (id, ps)
            })
            .collect()
    }

    fn constraint_entities(&self, id: ConstraintId) -> SmallVec<[EntityId; 4]> {
        self.constraints
            .get(&id)
            .map(|c| c.kind.entities())
            .unwrap_or_default()
    }
}

/// Report for a sketch whose equations cannot be built (should not happen: constraints are
/// validated when added).
fn failed_report(s: &Sketch3d) -> SolveReport {
    SolveReport {
        status: crate::solve::SolveStatus::DidNotConverge,
        iterations: 0,
        max_residual: f64::INFINITY,
        unsatisfied: s.constraints.keys().copied().collect(),
        driven: BTreeMap::new(),
        dof: None,
        diagnosis: None,
    }
}
