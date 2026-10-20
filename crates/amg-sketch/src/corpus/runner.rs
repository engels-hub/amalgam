//! Corpus runner: runs a [`CorpusBackend`] on cases and reports pass/fail per case.
//!
//! The runner is solver-agnostic. A backend gets the case and its [`BuiltCase`] (sketch in the
//! start configuration), solves it in place and reports a [`BackendOutcome`]. The runner then
//! compares against [`Expected`](super::Expected):
//!
//! 1. **Status** must be accepted (`status` or one of `also_accept`).
//! 2. **DOF** must match when the backend reports one (a backend without DOF analysis, e.g.
//!    SK-03 before SK-04, leaves it `None` and the check is reported as unchecked).
//! 3. **Solution**: for a converged `Start`/`Target` expectation, every listed value (all
//!    points and radii for `Start`) must be within `tolerance` of the result.
//! 4. **Groups**: when the case lists dependency groups and the backend reports groups, they
//!    must match as a set of sets.
//!
//! Panics inside a backend are caught and reported as failures of that case.
//!
//! The SK-03 adapter will be `impl CorpusBackend for` a wrapper around `SolverBackend`
//! (S03 Interfaces): `solve` → status/residual, `analyze` → DOF and groups, with
//! `BuiltCase::constraint_index` mapping `ConstraintId`s back to case indices.

use std::collections::BTreeSet;
use std::fmt;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::time::{Duration, Instant};

use super::{BuiltCase, Case, CaseStatus, ExpectedGroup, Solution, SourceSet, Values};

/// What a backend reports for one case.
#[derive(Clone, Debug, PartialEq)]
pub struct BackendOutcome {
    /// Final status.
    pub status: CaseStatus,
    /// Remaining DOF, if the backend analyses it.
    pub dof: Option<u32>,
    /// Largest absolute residual at the end, if known.
    pub max_residual: Option<f64>,
    /// Iterations used, if known.
    pub iterations: Option<u32>,
    /// Dependency groups (constraint indices into the case), if diagnosed.
    pub groups: Option<Vec<ExpectedGroup>>,
}

/// A backend declined a case (unsupported construct, missing data). Not a failure.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BackendSkip(pub String);

/// A solver (or checker) the corpus can be run against.
pub trait CorpusBackend {
    /// Name used in reports.
    fn name(&self) -> String;

    /// Solves `built.sketch` in place, starting from the case's start configuration.
    fn solve(&mut self, case: &Case, built: &mut BuiltCase) -> Result<BackendOutcome, BackendSkip>;
}

/// Outcome of one case.
#[derive(Clone, Debug, PartialEq)]
pub enum Verdict {
    /// Every check that could run passed.
    Pass,
    /// At least one check failed (reasons listed).
    Fail(Vec<String>),
    /// The backend declined the case.
    Skipped(String),
}

/// Result of one case.
#[derive(Clone, Debug)]
pub struct CaseResult {
    /// Case id.
    pub id: String,
    /// Source set.
    pub set: SourceSet,
    /// Verdict.
    pub verdict: Verdict,
    /// Backend outcome (if it ran to completion).
    pub outcome: Option<BackendOutcome>,
    /// Checks that could not run (e.g. `dof` when the backend does not report it).
    pub unchecked: Vec<&'static str>,
    /// Wall time of the backend call.
    pub elapsed: Duration,
}

/// Results of a corpus run.
#[derive(Clone, Debug)]
pub struct CorpusReport {
    /// Backend name.
    pub backend: String,
    /// Per-case results, in case order.
    pub results: Vec<CaseResult>,
}

impl CorpusReport {
    /// Number of passing cases.
    pub fn passed(&self) -> usize {
        self.results
            .iter()
            .filter(|r| r.verdict == Verdict::Pass)
            .count()
    }

    /// Failing cases.
    pub fn failures(&self) -> Vec<&CaseResult> {
        self.results
            .iter()
            .filter(|r| matches!(r.verdict, Verdict::Fail(_)))
            .collect()
    }

    /// Number of skipped cases.
    pub fn skipped(&self) -> usize {
        self.results
            .iter()
            .filter(|r| matches!(r.verdict, Verdict::Skipped(_)))
            .count()
    }

    /// Pass rate over the cases that were not skipped (1.0 when none ran).
    pub fn pass_rate(&self) -> f64 {
        let ran = self.results.len() - self.skipped();
        if ran == 0 {
            1.0
        } else {
            self.passed() as f64 / ran as f64
        }
    }
}

impl fmt::Display for CorpusReport {
    /// One line per case, then a per-set summary.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "corpus run, backend `{}`", self.backend)?;
        for r in &self.results {
            let (tag, detail) = match &r.verdict {
                Verdict::Pass => ("PASS", String::new()),
                Verdict::Fail(why) => ("FAIL", why.join("; ")),
                Verdict::Skipped(why) => ("SKIP", why.clone()),
            };
            let got = r.outcome.as_ref().map_or(String::new(), |o| {
                let dof = o.dof.map_or("-".to_string(), |d| d.to_string());
                let res = o
                    .max_residual
                    .map_or("-".to_string(), |x| format!("{x:.1e}"));
                format!("{:?} dof={dof} res={res}", o.status)
            });
            writeln!(
                f,
                "{tag} {:<50} {got:<40} {:>8.2}ms {detail}",
                r.id,
                r.elapsed.as_secs_f64() * 1e3
            )?;
        }
        for set in SourceSet::ALL {
            let rs: Vec<_> = self.results.iter().filter(|r| r.set == set).collect();
            if rs.is_empty() {
                continue;
            }
            let pass = rs.iter().filter(|r| r.verdict == Verdict::Pass).count();
            let skip = rs
                .iter()
                .filter(|r| matches!(r.verdict, Verdict::Skipped(_)))
                .count();
            writeln!(
                f,
                "{:<10} {pass}/{} pass, {skip} skipped",
                set.dir(),
                rs.len()
            )?;
        }
        write!(
            f,
            "total      {}/{} pass, {} skipped, {} fail",
            self.passed(),
            self.results.len(),
            self.skipped(),
            self.failures().len()
        )
    }
}

/// Largest difference between `got` and the listed `want` values (`None` if a listed entity
/// is missing from `got`).
fn max_diff(want: &Values, got: &Values) -> Option<f64> {
    let mut worst = 0.0_f64;
    for (i, w) in &want.points {
        let g = got.points.get(i)?;
        worst = worst.max((g[0] - w[0]).abs()).max((g[1] - w[1]).abs());
    }
    for (i, w) in &want.radii {
        worst = worst.max((got.radii.get(i)? - w).abs());
    }
    Some(worst)
}

/// Runs `backend` on one case.
pub fn run_case(backend: &mut dyn CorpusBackend, case: &Case) -> CaseResult {
    let mut result = CaseResult {
        id: case.id.clone(),
        set: case.source.set,
        verdict: Verdict::Pass,
        outcome: None,
        unchecked: Vec::new(),
        elapsed: Duration::ZERO,
    };
    let mut built = match case.build() {
        Ok(b) => b,
        Err(e) => {
            result.verdict = Verdict::Fail(vec![format!("build failed: {e}")]);
            return result;
        }
    };
    let start = built.values();
    let t0 = Instant::now();
    let ran = catch_unwind(AssertUnwindSafe(|| backend.solve(case, &mut built)));
    result.elapsed = t0.elapsed();
    let outcome = match ran {
        Ok(Ok(o)) => o,
        Ok(Err(BackendSkip(why))) => {
            result.verdict = Verdict::Skipped(why);
            return result;
        }
        Err(_) => {
            result.verdict = Verdict::Fail(vec!["backend panicked".into()]);
            return result;
        }
    };
    let ex = &case.expected;
    let mut why = Vec::new();
    if !ex.accepts(outcome.status) {
        why.push(format!(
            "status {:?}, expected {:?}",
            outcome.status, ex.status
        ));
    }
    match (ex.dof, outcome.dof) {
        (Some(want), Some(got)) if want != got => why.push(format!("dof {got}, expected {want}")),
        (Some(_), None) => result.unchecked.push("dof"),
        _ => {}
    }
    if outcome.status.is_converged() {
        let target = match &ex.solution {
            Some(Solution::Start) => Some(&start),
            Some(Solution::Target(v)) => Some(v),
            _ => None,
        };
        if let Some(want) = target {
            match max_diff(want, &built.values()) {
                Some(d) if d <= ex.tolerance => {}
                Some(d) => why.push(format!(
                    "solution off by {d:.3e} mm (tolerance {:.1e})",
                    ex.tolerance
                )),
                None => why.push("solution entity missing".into()),
            }
        }
    }
    if !ex.groups.is_empty() {
        match &outcome.groups {
            Some(got) => {
                let a: BTreeSet<_> = ex.groups.iter().collect();
                let b: BTreeSet<_> = got.iter().collect();
                if a != b {
                    why.push(format!("groups {got:?}, expected {:?}", ex.groups));
                }
            }
            None => result.unchecked.push("groups"),
        }
    }
    result.verdict = if why.is_empty() {
        Verdict::Pass
    } else {
        Verdict::Fail(why)
    };
    result.outcome = Some(outcome);
    result
}

/// Runs `backend` on every case, in order.
pub fn run_corpus<'a>(
    backend: &mut dyn CorpusBackend,
    cases: impl IntoIterator<Item = &'a Case>,
) -> CorpusReport {
    CorpusReport {
        backend: backend.name(),
        results: cases.into_iter().map(|c| run_case(backend, c)).collect(),
    }
}
