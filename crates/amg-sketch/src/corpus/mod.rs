//! Solver test corpus (SK-14): case format, importers, a reference checker and the runner.
//!
//! The corpus lives in `testdata/sketch/corpus/<set>/` (one folder per source, each with its
//! licence notice). It is test infrastructure: SK-03 measures itself against it, SK-13 runs
//! the ADR-007 benchmark on it, and with `--features slvs` libslvs plugs in as one more
//! [`CorpusBackend`].
//!
//! - [`Case`] is one problem: entities with their start configuration, constraints, and the
//!   [`Expected`] result (status, DOF, optionally the solution and dependency groups). It is
//!   stored as JSON (`*.json`, format [`FORMAT_VERSION`]); entity references inside
//!   constraints are indices into [`Case::entities`], so a constraint is a plain
//!   [`ConstraintKind`].
//! - Imported sets keep their original files: [`slvs`] reads SolveSpace `.slvs` files and
//!   [`ezpz`] reads ezpz `problem.md` files. Their expectations come from an `expected.json`
//!   sidecar ([`Sidecar`]) in the set folder.
//! - [`load_corpus`] loads every case below a corpus root, deterministically ordered.
//! - [`reference`] is an independent oracle (residuals with forward-mode derivatives,
//!   rank-revealing QR, a damped minimum-norm Gauss–Newton). It is slow and dense and is
//!   not a solver candidate; it validates the corpus and lets the runner work before SK-03.
//! - [`runner`] runs a [`CorpusBackend`] over cases and reports pass/fail per case.
//! - [`compare`] is the SK-13 solver comparison (ADR-007): the native prototype, libslvs
//!   (feature `slvs`) and ezpz (feature `ezpz-bench`) behind one interface, plus metrics.
//! - [`drag`] holds the SK-05 scripted-drag fixtures (200-entity truss) and harness.
//! - [`solver`] adapts a production `SolverBackend` (SK-03 `NativeSolver`) to the runner.
//!
//! Sign conventions follow the S03 constraint table and SK-01's `TangentKind` notes:
//! signed distances are positive on the **left** of an oriented line (`Side::Positive`), and
//! a point/line–circle distance with `Side::Positive` is measured outside the circle.

pub mod compare;
pub mod drag;
pub mod ezpz;
pub mod reference;
pub mod runner;
pub mod slvs;
pub mod solver;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use amg_core::{Plane, Point2};
use serde::{Deserialize, Serialize};

use crate::error::SketchError;
use crate::ids::{ConstraintId, EntityId};
use crate::model::{
    ConstraintKind, DimensionKind, EntityKind, EntityOrigin, HvTarget, ParamFlags, Sketch,
};

pub use reference::{ReferenceCheck, ReferenceSolver};
pub use runner::{
    BackendOutcome, BackendSkip, CaseResult, CorpusBackend, CorpusReport, Verdict, run_case,
    run_corpus,
};
pub use solver::SolverRunner;

/// Version of the JSON case format written by this module.
pub const FORMAT_VERSION: u32 = 1;

/// Default tolerance for comparing a solve against [`Expected::solution`], mm.
pub const DEFAULT_SOLUTION_TOL: f64 = 1e-6;

/// Errors raised while reading, importing or building corpus cases.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum CorpusError {
    /// The source uses something the importer or the sketch model does not support (3D
    /// geometry, cubics, a constraint type without an equivalent). The case is skipped.
    #[error("unsupported: {0}")]
    Unsupported(String),
    /// The file is malformed.
    #[error("parse error at line {line}: {message}")]
    Parse {
        /// 1-based line number (0 when unknown).
        line: usize,
        /// What went wrong.
        message: String,
    },
    /// The case is structurally invalid (bad index, inconsistent expectation, ...).
    #[error("invalid case {case}: {message}")]
    Invalid {
        /// Case id.
        case: String,
        /// What went wrong.
        message: String,
    },
    /// The sketch model rejected an entity or constraint.
    #[error("case {case}: {source}")]
    Sketch {
        /// Case id.
        case: String,
        /// The model error.
        source: SketchError,
    },
    /// File system or JSON error while loading the corpus.
    #[error("{path}: {message}")]
    Io {
        /// File or folder.
        path: String,
        /// What went wrong.
        message: String,
    },
}

/// Where a case comes from.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SourceSet {
    /// SolveSpace `test/constraint/*` (GPL-3.0-or-later data, test-only).
    SolveSpace,
    /// Hand-ported from FreeCAD Sketcher / PlaneGCS tests (LGPL-2.1-or-later).
    PlaneGcs,
    /// KittyCAD ezpz `test_cases/` (MIT).
    Ezpz,
    /// Written for Amalgam (decision cases D1–D10 and others).
    Own,
}

impl SourceSet {
    /// All sets, in report order.
    pub const ALL: [SourceSet; 4] = [
        SourceSet::SolveSpace,
        SourceSet::PlaneGcs,
        SourceSet::Ezpz,
        SourceSet::Own,
    ];

    /// Folder name below the corpus root.
    pub fn dir(self) -> &'static str {
        match self {
            SourceSet::SolveSpace => "solvespace",
            SourceSet::PlaneGcs => "planegcs",
            SourceSet::Ezpz => "ezpz",
            SourceSet::Own => "own",
        }
    }
}

/// Provenance of a case.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CaseSource {
    /// Source set.
    pub set: SourceSet,
    /// Upstream location (file path in the upstream repository, test name, ...).
    pub origin: String,
    /// SPDX licence expression of the case data.
    pub licence: String,
}

/// Geometry of one case entity. References are indices into [`Case::entities`] and must point
/// to *earlier* point entities.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum CaseGeom {
    /// Point at `at` (start configuration).
    Point {
        /// Start position, mm.
        at: [f64; 2],
    },
    /// Line between two points.
    Line {
        /// Start point index.
        start: u32,
        /// End point index.
        end: u32,
    },
    /// Circle with a centre point and a start radius.
    Circle {
        /// Centre point index.
        center: u32,
        /// Start radius, mm.
        radius: f64,
    },
    /// CCW arc; the start configuration must satisfy the arc's internal equation.
    Arc {
        /// Centre point index.
        center: u32,
        /// Start point index.
        start: u32,
        /// End point index.
        end: u32,
    },
}

/// One entity of a case.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CaseEntity {
    /// Human-readable name (unique within the case when non-empty).
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub name: String,
    /// Geometry.
    pub geom: CaseGeom,
    /// Construction geometry.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub construction: bool,
}

/// Solver status a case expects. Mirrors the S03 `SolveStatus` minus `Cancelled`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaseStatus {
    /// All constraints satisfied, equations independent.
    Converged,
    /// All constraints satisfied, but some equations are redundant.
    ConvergedRedundant,
    /// No solution: dependent equations that contradict each other.
    Inconsistent,
    /// The solver gave up (no solution found).
    DidNotConverge,
}

impl CaseStatus {
    /// True for the two converged statuses.
    pub fn is_converged(self) -> bool {
        matches!(self, CaseStatus::Converged | CaseStatus::ConvergedRedundant)
    }
}

/// Kind of an expected dependency group (S03 `GroupKind`).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GroupKind {
    /// Consistent redundancy.
    Redundant,
    /// Contradiction.
    Conflicting,
    /// Dependency caused only by a degenerate configuration.
    DegenerateConfiguration,
}

/// An expected dependency group: constraint indices into [`Case::constraints`].
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExpectedGroup {
    /// Kind.
    pub kind: GroupKind,
    /// Constraint indices, sorted ascending.
    pub constraints: Vec<u32>,
}

/// Parameter values of a configuration, keyed by entity index.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Values {
    /// Point positions, mm.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub points: BTreeMap<u32, [f64; 2]>,
    /// Circle radii, mm.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub radii: BTreeMap<u32, f64>,
}

/// A known configuration satisfying the constraints.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum Solution {
    /// The start configuration already is the solution, and a solver must keep it (within
    /// [`Expected::tolerance`]): minimum movement from a solved state. SolveSpace files are
    /// saved solved.
    Start,
    /// The root a solver must reach from the start configuration (chirality: tangent arcs and
    /// signed distances must not flip). Listed values are compared.
    Target(Values),
    /// Some configuration that satisfies the constraints (under-constrained cases); used by
    /// [`ReferenceCheck`], never compared against a solve.
    Witness(Values),
}

fn default_tol() -> f64 {
    DEFAULT_SOLUTION_TOL
}

fn is_default_tol(t: &f64) -> bool {
    t.to_bits() == DEFAULT_SOLUTION_TOL.to_bits()
}

/// Expected result of solving a case from its start configuration.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Expected {
    /// Expected status.
    pub status: CaseStatus,
    /// Other statuses that also pass (e.g. `did_not_converge` for an infeasible system that a
    /// solver may not recognise as inconsistent).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub also_accept: Vec<CaseStatus>,
    /// Remaining degrees of freedom at the solution (`free params − rank J`). `None` only
    /// for cases without a solution (inconsistent / infeasible).
    pub dof: Option<u32>,
    /// Known solution, if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub solution: Option<Solution>,
    /// Tolerance for comparing against a `Start`/`Target` solution, mm.
    #[serde(default = "default_tol", skip_serializing_if = "is_default_tol")]
    pub tolerance: f64,
    /// Expected dependency groups (diagnosis, SK-04); empty = not specified.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub groups: Vec<ExpectedGroup>,
}

impl Expected {
    /// True if `status` passes.
    pub fn accepts(&self, status: CaseStatus) -> bool {
        self.status == status || self.also_accept.contains(&status)
    }
}

/// Scripted drag for drag benchmarks (SK-05, SK-13): the cursor visits `path` one frame per
/// entry. The runner does not execute it yet.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DragScript {
    /// Dragged point (entity index).
    pub entity: u32,
    /// Cursor position per frame, mm.
    pub path: Vec<[f64; 2]>,
}

/// One corpus case.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Case {
    /// Format version ([`FORMAT_VERSION`]).
    pub format: u32,
    /// Unique id, `<set>/<path without extension>`.
    pub id: String,
    /// Provenance.
    pub source: CaseSource,
    /// What the case is about.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub description: String,
    /// Free-form tags (`decision:D1`, `chirality`, `large`, ...).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tags: Vec<String>,
    /// Entities in creation order (points before the curves using them).
    pub entities: Vec<CaseEntity>,
    /// Constraints; entity ids inside are indices into `entities`.
    pub constraints: Vec<ConstraintKind>,
    /// Expected result.
    pub expected: Expected,
    /// Optional drag script.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub drag: Option<DragScript>,
}

/// A case turned into a [`Sketch`], with the maps between case indices and sketch ids.
#[derive(Clone, Debug)]
pub struct BuiltCase {
    /// The sketch in its start configuration.
    pub sketch: Sketch,
    /// Sketch entity of each case entity (index = case index).
    pub entities: Vec<EntityId>,
    /// Sketch constraint of each case constraint (index = case index).
    pub constraints: Vec<ConstraintId>,
}

impl BuiltCase {
    /// Case index of a sketch constraint.
    pub fn constraint_index(&self, id: ConstraintId) -> Option<u32> {
        self.constraints
            .iter()
            .position(|&c| c == id)
            .and_then(|i| u32::try_from(i).ok())
    }

    /// Current values of every point and circle radius, keyed by case index.
    pub fn values(&self) -> Values {
        let mut v = Values::default();
        for (i, &id) in self.entities.iter().enumerate() {
            let Ok(i) = u32::try_from(i) else { break };
            match self.sketch.entity(id).map(|e| &e.kind) {
                Some(&EntityKind::Point { x, y }) => {
                    let p = self.sketch.params();
                    v.points.insert(i, [p.get(x), p.get(y)]);
                }
                Some(&EntityKind::Circle { radius, .. }) => {
                    v.radii.insert(i, self.sketch.params().get(radius));
                }
                _ => {}
            }
        }
        v
    }

    /// Writes `values` into the sketch (entries for other kinds or unknown indices are
    /// errors).
    pub fn apply(&mut self, values: &Values) -> Result<(), String> {
        for (&i, &[x, y]) in &values.points {
            let id = *self
                .entities
                .get(i as usize)
                .ok_or_else(|| format!("solution point {i} out of range"))?;
            match self.sketch.entity(id).map(|e| e.kind.clone()) {
                Some(EntityKind::Point { x: px, y: py }) => {
                    let p = self.sketch.params_mut();
                    p.set(px, x);
                    p.set(py, y);
                }
                _ => return Err(format!("solution entry {i} is not a point")),
            }
        }
        for (&i, &r) in &values.radii {
            let id = *self
                .entities
                .get(i as usize)
                .ok_or_else(|| format!("solution radius {i} out of range"))?;
            match self.sketch.entity(id).map(|e| e.kind.clone()) {
                Some(EntityKind::Circle { radius, .. }) => self.sketch.params_mut().set(radius, r),
                _ => return Err(format!("solution entry {i} is not a circle")),
            }
        }
        Ok(())
    }
}

impl Case {
    /// Number of entities and constraints, for reports.
    pub fn size(&self) -> (usize, usize) {
        (self.entities.len(), self.constraints.len())
    }

    fn invalid(&self, message: impl Into<String>) -> CorpusError {
        CorpusError::Invalid {
            case: self.id.clone(),
            message: message.into(),
        }
    }

    /// Structural checks that do not need a sketch: format, unique names, forward references,
    /// solution and group indices, expectation consistency.
    pub fn validate(&self) -> Result<(), CorpusError> {
        if self.format != FORMAT_VERSION {
            return Err(self.invalid(format!(
                "format {} (expected {FORMAT_VERSION})",
                self.format
            )));
        }
        let mut names = std::collections::BTreeSet::new();
        for (i, e) in self.entities.iter().enumerate() {
            if !e.name.is_empty() && !names.insert(e.name.as_str()) {
                return Err(self.invalid(format!("duplicate entity name {:?}", e.name)));
            }
            let refs: &[u32] = match &e.geom {
                CaseGeom::Point { .. } => &[],
                CaseGeom::Line { start, end } => &[*start, *end],
                CaseGeom::Circle { center, .. } => &[*center],
                CaseGeom::Arc { center, start, end } => &[*center, *start, *end],
            };
            for &r in refs {
                let ok = (r as usize) < i
                    && matches!(self.entities[r as usize].geom, CaseGeom::Point { .. });
                if !ok {
                    return Err(self.invalid(format!(
                        "entity {i} references {r}, which is not an earlier point"
                    )));
                }
            }
        }
        let n = self.entities.len();
        for (ci, c) in self.constraints.iter().enumerate() {
            if let Some(bad) = c.entities().into_iter().find(|e| e.index() >= n) {
                return Err(self.invalid(format!(
                    "constraint {ci} references entity {} (only {n})",
                    bad.0
                )));
            }
        }
        let ex = &self.expected;
        if ex.status.is_converged() != ex.dof.is_some() {
            return Err(self.invalid("`dof` must be given exactly for converged statuses"));
        }
        if !(ex.tolerance.is_finite() && ex.tolerance > 0.0) {
            return Err(self.invalid("tolerance must be positive"));
        }
        if let Some(Solution::Target(v) | Solution::Witness(v)) = &ex.solution {
            for &i in v.points.keys() {
                if !matches!(
                    self.entities.get(i as usize).map(|e| &e.geom),
                    Some(CaseGeom::Point { .. })
                ) {
                    return Err(self.invalid(format!("solution point {i} is not a point")));
                }
            }
            for &i in v.radii.keys() {
                if !matches!(
                    self.entities.get(i as usize).map(|e| &e.geom),
                    Some(CaseGeom::Circle { .. })
                ) {
                    return Err(self.invalid(format!("solution radius {i} is not a circle")));
                }
            }
        }
        if ex.solution.is_some() && !ex.status.is_converged() {
            return Err(self.invalid("an unsolvable case cannot have a solution"));
        }
        for g in &ex.groups {
            if g.constraints.windows(2).any(|w| w[0] >= w[1]) {
                return Err(self.invalid("group constraints must be sorted and unique"));
            }
            if g.constraints
                .iter()
                .any(|&c| c as usize >= self.constraints.len())
            {
                return Err(self.invalid("group references a missing constraint"));
            }
        }
        if let Some(d) = &self.drag
            && !matches!(
                self.entities.get(d.entity as usize).map(|e| &e.geom),
                Some(CaseGeom::Point { .. })
            )
        {
            return Err(self.invalid("drag entity is not a point"));
        }
        Ok(())
    }

    /// Builds the sketch (on `Plane::XY`) in the start configuration.
    ///
    /// Entity `i` becomes sketch entity `EntityId(i)`. Constraints are added in order; an
    /// endpoint tangency whose ends are joined only by a later `Coincident` is retried after
    /// the others, so the constraint ids may differ from the case indices (see
    /// [`BuiltCase::constraints`]).
    pub fn build(&self) -> Result<BuiltCase, CorpusError> {
        self.validate()?;
        let err = |source: SketchError| CorpusError::Sketch {
            case: self.id.clone(),
            source,
        };
        let mut sketch = Sketch::new(Plane::XY);
        let mut entities = Vec::with_capacity(self.entities.len());
        for e in &self.entities {
            let eid = |i: u32| entities_get(&entities, i);
            let kind = match e.geom {
                CaseGeom::Point { at: [x, y] } => {
                    let x = sketch.alloc_param(x, ParamFlags::default()).map_err(err)?;
                    let y = sketch.alloc_param(y, ParamFlags::default()).map_err(err)?;
                    EntityKind::Point { x, y }
                }
                CaseGeom::Line { start, end } => EntityKind::Line {
                    start: eid(start),
                    end: eid(end),
                },
                CaseGeom::Circle { center, radius } => {
                    let radius = sketch
                        .alloc_param(radius, ParamFlags::default())
                        .map_err(err)?;
                    EntityKind::Circle {
                        center: eid(center),
                        radius,
                    }
                }
                CaseGeom::Arc { center, start, end } => EntityKind::Arc {
                    center: eid(center),
                    start: eid(start),
                    end: eid(end),
                },
            };
            let id = sketch
                .add_entity(kind, e.construction, EntityOrigin::Imported)
                .map_err(err)?;
            entities.push(id);
        }
        let map = |k: &ConstraintKind| map_entities(k, &|e: EntityId| entities_get(&entities, e.0));
        let mut constraints = vec![None; self.constraints.len()];
        let mut deferred = Vec::new();
        for (i, c) in self.constraints.iter().enumerate() {
            match sketch.add_constraint(map(c)) {
                Ok(id) => constraints[i] = Some(id),
                Err(SketchError::NotApplicable(_))
                    if matches!(
                        c,
                        ConstraintKind::Tangent {
                            kind: crate::model::TangentKind::AtEndpoint { .. },
                            ..
                        }
                    ) =>
                {
                    deferred.push(i);
                }
                Err(e) => return Err(err(e)),
            }
        }
        for i in deferred {
            constraints[i] = Some(
                sketch
                    .add_constraint(map(&self.constraints[i]))
                    .map_err(err)?,
            );
        }
        let constraints = constraints.into_iter().flatten().collect();
        Ok(BuiltCase {
            sketch,
            entities,
            constraints,
        })
    }
}

/// Sketch id of case entity `i`. `validate` guarantees the index exists; a dangling id only
/// makes the model report `UnknownEntity`.
fn entities_get(entities: &[EntityId], i: u32) -> EntityId {
    entities
        .get(i as usize)
        .copied()
        .unwrap_or(EntityId(u32::MAX))
}

/// `kind` with every entity id passed through `f`.
pub fn map_entities(kind: &ConstraintKind, f: &dyn Fn(EntityId) -> EntityId) -> ConstraintKind {
    use ConstraintKind as C;
    let hv = |t: &HvTarget| match *t {
        HvTarget::Line(l) => HvTarget::Line(f(l)),
        HvTarget::Points(a, b) => HvTarget::Points(f(a), f(b)),
    };
    match kind {
        C::Coincident { a, b } => C::Coincident { a: f(*a), b: f(*b) },
        C::PointOnCurve { point, curve } => C::PointOnCurve {
            point: f(*point),
            curve: f(*curve),
        },
        C::Midpoint { point, line } => C::Midpoint {
            point: f(*point),
            line: f(*line),
        },
        C::Horizontal(t) => C::Horizontal(hv(t)),
        C::Vertical(t) => C::Vertical(hv(t)),
        C::Parallel { a, b } => C::Parallel { a: f(*a), b: f(*b) },
        C::Perpendicular { a, b } => C::Perpendicular { a: f(*a), b: f(*b) },
        C::Collinear { a, b } => C::Collinear { a: f(*a), b: f(*b) },
        C::Tangent { a, b, kind } => C::Tangent {
            a: f(*a),
            b: f(*b),
            kind: *kind,
        },
        C::Curvature {
            a,
            b,
            a_end,
            b_end,
            reversed,
        } => C::Curvature {
            a: f(*a),
            b: f(*b),
            a_end: *a_end,
            b_end: *b_end,
            reversed: *reversed,
        },
        C::Concentric { a, b } => C::Concentric { a: f(*a), b: f(*b) },
        C::Equal { a, b } => C::Equal { a: f(*a), b: f(*b) },
        C::Symmetric { a, b, axis } => C::Symmetric {
            a: f(*a),
            b: f(*b),
            axis: f(*axis),
        },
        C::Fix { entity } => C::Fix { entity: f(*entity) },
        C::Dimension(d) => {
            use DimensionKind as D;
            let kind = match d.kind {
                D::Distance { a, b, side } => D::Distance {
                    a: f(a),
                    b: f(b),
                    side,
                },
                D::HorizontalDistance { a, b, side } => D::HorizontalDistance {
                    a: f(a),
                    b: f(b),
                    side,
                },
                D::VerticalDistance { a, b, side } => D::VerticalDistance {
                    a: f(a),
                    b: f(b),
                    side,
                },
                D::Length { line } => D::Length { line: f(line) },
                D::Angle { a, b, branch } => D::Angle {
                    a: f(a),
                    b: f(b),
                    branch,
                },
                D::Radius { curve } => D::Radius { curve: f(curve) },
                D::Diameter { curve } => D::Diameter { curve: f(curve) },
                D::ArcLength { arc } => D::ArcLength { arc: f(arc) },
                D::EllipseMajor { ellipse } => D::EllipseMajor {
                    ellipse: f(ellipse),
                },
                D::EllipseMinor { ellipse } => D::EllipseMinor {
                    ellipse: f(ellipse),
                },
            };
            C::Dimension(crate::model::Dimension {
                kind,
                value: d.value,
                driving: d.driving,
                label_pos: d.label_pos,
            })
        }
    }
}

// ---- Imported sets: sidecar expectations ------------------------------------------------

/// `expected.json` of an imported set: expectations for the original files.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sidecar {
    /// SPDX licence of the set's data.
    pub licence: String,
    /// Upstream repository and pinned commit.
    pub upstream: String,
    /// One entry per case group; every importable file must be covered exactly once.
    pub cases: Vec<SidecarEntry>,
}

/// Expectation for one or more files of an imported set (format-version variants of the same
/// geometry share one entry).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SidecarEntry {
    /// Paths relative to the set folder, `/`-separated.
    pub files: Vec<String>,
    /// What the case tests and how the expectation was derived.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub description: String,
    /// Tags added to the case.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tags: Vec<String>,
    /// Expected result.
    pub expected: Expected,
}

// ---- Loading ------------------------------------------------------------------------------

fn io_err(path: &Path, e: impl std::fmt::Display) -> CorpusError {
    CorpusError::Io {
        path: path.display().to_string(),
        message: e.to_string(),
    }
}

/// Parses a JSON case file.
pub fn parse_case_json(text: &str) -> Result<Case, CorpusError> {
    serde_json::from_str(text).map_err(|e| CorpusError::Parse {
        line: e.line(),
        message: e.to_string(),
    })
}

/// Serialises a case as pretty JSON with a trailing newline (the checked-in form).
pub fn case_to_json(case: &Case) -> Result<String, CorpusError> {
    let mut s = serde_json::to_string_pretty(case).map_err(|e| CorpusError::Invalid {
        case: case.id.clone(),
        message: e.to_string(),
    })?;
    s.push('\n');
    Ok(s)
}

/// All files below `dir`, relative and `/`-separated, sorted.
fn walk(dir: &Path) -> Result<Vec<String>, CorpusError> {
    fn rec(root: &Path, dir: &Path, out: &mut Vec<String>) -> Result<(), CorpusError> {
        let rd = std::fs::read_dir(dir).map_err(|e| io_err(dir, e))?;
        for entry in rd {
            let entry = entry.map_err(|e| io_err(dir, e))?;
            let path = entry.path();
            if path.is_dir() {
                rec(root, &path, out)?;
            } else if let Ok(rel) = path.strip_prefix(root) {
                let parts: Vec<_> = rel
                    .components()
                    .map(|c| c.as_os_str().to_string_lossy().into_owned())
                    .collect();
                out.push(parts.join("/"));
            }
        }
        Ok(())
    }
    let mut out = Vec::new();
    rec(dir, dir, &mut out)?;
    out.sort();
    Ok(out)
}

/// What [`load_corpus`] found.
#[derive(Clone, Debug, Default)]
pub struct LoadedCorpus {
    /// Loaded cases, sorted by id.
    pub cases: Vec<Case>,
    /// Source files that were not imported (`(path, reason)`), e.g. unsupported constraint
    /// types. Imported sets keep only importable files, so this is normally empty.
    pub skipped: Vec<(String, String)>,
}

impl LoadedCorpus {
    /// Number of cases from `set`.
    pub fn count(&self, set: SourceSet) -> usize {
        self.cases.iter().filter(|c| c.source.set == set).count()
    }
}

/// Loads every case below `root` (`<root>/<set>/...`).
///
/// - `*.json` files other than `expected.json` are [`Case`]s.
/// - `*.slvs` files are imported with [`slvs::import`] and `problem.md` files with
///   [`ezpz::import`]; both take their expectation from the set's `expected.json`, which must
///   cover every such file exactly once and name no missing file.
/// - Other files (`NOTICE`, `COPYING`, `README.md`, ...) are ignored.
///
/// Every case is [validated](Case::validate) and ids must be unique.
pub fn load_corpus(root: &Path) -> Result<LoadedCorpus, CorpusError> {
    let mut out = LoadedCorpus::default();
    for set in SourceSet::ALL {
        let dir = root.join(set.dir());
        if !dir.is_dir() {
            continue;
        }
        let files = walk(&dir)?;
        let sidecar_path = dir.join("expected.json");
        let sidecar: Option<Sidecar> = if sidecar_path.is_file() {
            let text =
                std::fs::read_to_string(&sidecar_path).map_err(|e| io_err(&sidecar_path, e))?;
            Some(serde_json::from_str(&text).map_err(|e| io_err(&sidecar_path, e))?)
        } else {
            None
        };
        let mut by_file: BTreeMap<&str, &SidecarEntry> = BTreeMap::new();
        if let Some(sc) = &sidecar {
            for entry in &sc.cases {
                for f in &entry.files {
                    if by_file.insert(f.as_str(), entry).is_some() {
                        return Err(io_err(&sidecar_path, format!("{f} is listed twice")));
                    }
                    if !files.iter().any(|x| x == f) {
                        return Err(io_err(&sidecar_path, format!("{f} does not exist")));
                    }
                }
            }
        }
        for rel in &files {
            let path = dir.join(rel);
            let stem = rel.rsplit_once('.').map_or(rel.as_str(), |(s, _)| s);
            let imported = if rel.ends_with(".json") {
                if rel == "expected.json" {
                    continue;
                }
                let text = std::fs::read_to_string(&path).map_err(|e| io_err(&path, e))?;
                let case = parse_case_json(&text).map_err(|e| io_err(&path, e))?;
                let want = format!("{}/{stem}", set.dir());
                if case.id != want {
                    return Err(io_err(
                        &path,
                        format!("case id {:?} must be {want:?}", case.id),
                    ));
                }
                Some(Ok(case))
            } else if rel.ends_with(".slvs") {
                let bytes = std::fs::read(&path).map_err(|e| io_err(&path, e))?;
                Some(slvs::import(
                    &bytes,
                    &format!("{}/{stem}", set.dir()),
                    &format!("test/constraint/{rel}"),
                ))
            } else if rel.ends_with("problem.md") {
                let text = std::fs::read_to_string(&path).map_err(|e| io_err(&path, e))?;
                let name = stem.trim_end_matches("/problem");
                Some(ezpz::import(
                    &text,
                    &format!("{}/{name}", set.dir()),
                    &format!("test_cases/{rel}"),
                ))
            } else {
                None
            };
            let Some(imported) = imported else { continue };
            let mut case = match imported {
                Ok(c) => c,
                Err(CorpusError::Unsupported(why)) => {
                    out.skipped.push((format!("{}/{rel}", set.dir()), why));
                    continue;
                }
                Err(e) => return Err(io_err(&path, e)),
            };
            if !rel.ends_with(".json") {
                let entry = by_file
                    .get(rel.as_str())
                    .ok_or_else(|| io_err(&path, "no entry in expected.json"))?;
                case.expected = entry.expected.clone();
                case.description.clone_from(&entry.description);
                case.tags.extend(entry.tags.iter().cloned());
                if let Some(sc) = &sidecar {
                    case.source.licence.clone_from(&sc.licence);
                }
            }
            case.validate().map_err(|e| io_err(&path, e))?;
            out.cases.push(case);
        }
    }
    out.cases.sort_by(|a, b| a.id.cmp(&b.id));
    if let Some(w) = out.cases.windows(2).find(|w| w[0].id == w[1].id) {
        return Err(io_err(root, format!("duplicate case id {}", w[0].id)));
    }
    Ok(out)
}

/// `<root>/<set>/<id without set>.json` for a JSON case.
pub fn case_path(root: &Path, case: &Case) -> PathBuf {
    root.join(format!("{}.json", case.id))
}

/// A point helper for case builders: `[p.x, p.y]`.
pub fn xy(p: Point2) -> [f64; 2] {
    [p.x, p.y]
}

/// Placeholder expectation used by importers until the sidecar provides the real one.
pub(crate) fn placeholder_expected() -> Expected {
    Expected {
        status: CaseStatus::Converged,
        also_accept: Vec::new(),
        dof: Some(0),
        solution: None,
        tolerance: DEFAULT_SOLUTION_TOL,
        groups: Vec::new(),
    }
}
