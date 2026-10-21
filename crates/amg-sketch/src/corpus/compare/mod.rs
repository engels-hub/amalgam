//! SK-13 solver comparison for the ADR-007 decision (note `docs/notes/sketch-solver.md` §5).
//!
//! Every option implements [`Contender`]: it maps a [`BuiltCase`] (our sketch model in its start
//! configuration) into its own model with [`Contender::load`], then solves, edits a driving
//! dimension and drags through the [`Loaded`] handle. One interface lets the same code measure
//! correctness, cold/warm solve time, drag time and jitter, and diagnosis ([`metrics`]), and
//! run the whole corpus ([`ContenderBackend`] is a [`CorpusBackend`]).
//!
//! | Option | Module | Feature |
//! |---|---|---|
//! | A. native prototype ([`crate::solve::proto`]) | [`native`] | always |
//! | B. SolveSpace `libslvs` (GPL-3.0, test oracle only) | `slvs` | `slvs` |
//! | C. Zoo `ezpz` (MIT) | `ezpz` | `ezpz-bench` |
//!
//! All options are judged by the same residuals (SK-02 [`build_system`]) on the values they
//! return, so "converged" means the same thing for each.

pub mod metrics;
pub mod native;

#[cfg(feature = "ezpz-bench")]
pub mod ezpz;
#[cfg(feature = "slvs")]
pub mod slvs;

use super::runner::{BackendOutcome, BackendSkip, CorpusBackend};
use super::{BuiltCase, Case, CaseStatus, ExpectedGroup, Values};
use crate::model::Sketch;
use crate::solve::build_system;

/// What one solve or drag frame reports.
#[derive(Clone, Debug, PartialEq)]
pub struct Outcome {
    /// Status as the option reports it (mapped to the corpus statuses; see each adapter).
    pub status: CaseStatus,
    /// Remaining DOF, if the option computes it.
    pub dof: Option<u32>,
    /// Reported failing / dependent constraint sets (case constraint indices), if any.
    pub groups: Option<Vec<ExpectedGroup>>,
    /// Iterations, if reported.
    pub iterations: Option<u32>,
}

/// A solver option under comparison.
pub trait Contender {
    /// Name used in reports.
    fn name(&self) -> String;
    /// Maps the case into the option's model (structure + start values). `Err` = the option
    /// cannot express the case (missing constraint type, ...).
    fn load(&self, built: &BuiltCase) -> Result<Box<dyn Loaded>, BackendSkip>;
}

impl Contender for Box<dyn Contender> {
    fn name(&self) -> String {
        self.as_ref().name()
    }
    fn load(&self, built: &BuiltCase) -> Result<Box<dyn Loaded>, BackendSkip> {
        self.as_ref().load(built)
    }
}

/// A loaded case.
pub trait Loaded {
    /// Solves from the current values.
    fn solve(&mut self) -> Outcome;
    /// Current values of every point and circle radius, keyed by case entity index.
    fn values(&self) -> Values;
    /// Sets driving dimension `constraint` (case index) to `value`; the next solve starts from
    /// the current values (warm).
    fn set_dimension(&mut self, constraint: u32, value: f64) -> Result<(), String>;
    /// One drag frame: point `point` (case entity index) toward `target`, then solve.
    fn drag(&mut self, point: u32, target: [f64; 2]) -> Outcome;
}

/// Largest SK-02 residual of `sketch` at its current values, and the solve tolerance
/// `max(1e-10, 64 ε L_c)` used by every judge.
pub fn judge(sketch: &Sketch) -> (f64, f64) {
    match build_system(sketch) {
        Ok(sys) => {
            let x = sketch.params().as_slice();
            let mut f = vec![0.0; sys.system.rows()];
            sys.system.eval(x, &mut f);
            let worst = f.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
            let worst = if worst.is_nan() { f64::INFINITY } else { worst };
            (worst, (64.0 * f64::EPSILON * sys.length_scale).max(1e-10))
        }
        Err(_) => (f64::INFINITY, 0.0),
    }
}

/// Runs a [`Contender`] as a corpus backend: load, solve, write the values back.
#[derive(Debug)]
pub struct ContenderBackend<C>(pub C);

impl<C: Contender> CorpusBackend for ContenderBackend<C> {
    fn name(&self) -> String {
        self.0.name()
    }

    fn solve(
        &mut self,
        _case: &Case,
        built: &mut BuiltCase,
    ) -> Result<BackendOutcome, BackendSkip> {
        let mut loaded = self.0.load(built)?;
        let out = loaded.solve();
        built.apply(&loaded.values()).map_err(BackendSkip)?;
        Ok(BackendOutcome {
            status: out.status,
            dof: out.dof,
            max_residual: Some(judge(&built.sketch).0),
            iterations: out.iterations,
            groups: out.groups,
        })
    }
}

/// The contenders built with the current features, in report order.
pub fn contenders() -> Vec<Box<dyn Contender>> {
    #[allow(unused_mut, reason = "pushed to only with optional features")]
    let mut v: Vec<Box<dyn Contender>> = vec![
        Box::new(native::Native::default()),
        Box::new(native::Native::fast()),
        Box::new(native::Native::dogleg()),
        Box::new(native::Native::sqp_drag()),
    ];
    #[cfg(feature = "slvs")]
    v.push(Box::new(slvs::Slvs));
    #[cfg(feature = "ezpz-bench")]
    v.push(Box::new(ezpz::Ezpz));
    v
}
