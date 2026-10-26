//! The production native solver (SK-03, ADR-007): substitution → connected components →
//! Dulmage–Mendelsohn blocks → per block damped weighted minimum-norm Gauss–Newton / LM (dense
//! up to [`DENSE_MAX`] unknowns, sparse `faer` Cholesky above) → status, rank test,
//! unsatisfied constraints, driven-dimension values and (on failure) diagnosis.
//!
//! - The symbolic part ([`structure`]) is cached per `Sketch::structure_rev` (plus the fixed
//!   set and residual shapes), so value-only edits and drags skip it. It does not depend on
//!   values, so cached and fresh solves are bit-identical.
//! - A component split into several blocks that ends unsatisfied is re-solved as one block
//!   from its start values (blocks fix earlier unknowns; a structurally determined unknown can
//!   be numerically free). The better result is kept.
//! - Cancellation is checked before each block, every iteration, every rank test and every 16
//!   diagnosis rows. A cancelled solve leaves the sketch unchanged.
//! - DOF analysis and diagnosis ([`analysis`], SK-04): per-entity status from the null space of
//!   the decomposed system, circuits with a witness tie-break and leave-one-out refinement of
//!   conflicts, and an incremental `check_constraint` on the cached analysis.
//! - Drag sessions ([`drag`], SK-05): weighted minimum norm per frame on the components that
//!   contain the grabbed params, with a feasible-SQP fallback near singular configurations.

mod analysis;
mod block;
mod dense;
mod diagnose;
mod drag;
mod sparse;
mod structure;
#[cfg(test)]
mod tests;

use std::collections::BTreeMap;
use std::sync::Arc;

use self::analysis::{Base, Settings, analyze_at};
use self::block::{BlockCtx, BlockOutcome, solve_block, to_dense};
pub use self::drag::NativeDrag;
use self::structure::{Slot, Structure, Sub, SubKind};
use super::api::{
    ConstraintCheck, Diagnosis, DofAnalysis, DofReport, DragMode, DragSession, DragSpec,
    SolveOptions, SolveReport, SolveStatus, SolverBackend,
};
use super::rank::numeric_rank;
use super::residual::{SketchSystem, build, build_system_polled, characteristic_length};
use super::system::{EquationOwner, EquationSystem, Residual};
use crate::error::SketchError;
use crate::ids::{ConstraintId, ParamId};
use crate::model::{ConstraintKind, DimensionKind, Sketch};

/// Blocks with at most this many unknowns use the dense step; larger ones the sparse one.
pub const DENSE_MAX: usize = 64;
/// Blocks with at most this many unknowns (and rows) get a dense numeric rank test; larger
/// ones use their structural rank (SK-04: sparse rank-revealing QR).
const RANK_DENSE_MAX: usize = 400;

/// Scratch buffers reused across blocks.
#[derive(Default)]
pub(crate) struct Workspace {
    pub tmp: Vec<f64>,
}

/// Structure statistics of the last solve (tests, benches, tuning).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct SolveStats {
    /// The structure came from the cache.
    pub cache_hit: bool,
    /// Unknowns after substitution.
    pub unknowns: usize,
    /// Equation rows removed by substitution.
    pub substituted_rows: usize,
    /// Connected components.
    pub components: usize,
    /// Dulmage–Mendelsohn blocks over all components.
    pub blocks: usize,
    /// Unknowns of the largest block.
    pub largest_block: usize,
    /// Blocks solved with the sparse step.
    pub sparse_blocks: usize,
    /// Components re-solved as one block after their blocks failed.
    pub fallbacks: usize,
}

/// The native solver backend. Holds the structure cache of the last sketch it saw.
pub struct NativeSolver {
    cache: Option<Arc<Structure>>,
    /// Systems of cancelled solves, dropped after the next completed solve so a cancel never
    /// waits for a large drop.
    retired: Vec<SketchSystem>,
    dense_max: usize,
    decompose: bool,
    stats: SolveStats,
    /// Analysis of the last sketch `analyze` / `check_constraint` saw (SK-04).
    analysis: Option<Arc<Base>>,
    /// How drag frames are solved (SK-05).
    drag_mode: DragMode,
}

impl std::fmt::Debug for NativeSolver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NativeSolver")
            .field("cached_rev", &self.cache.as_ref().map(|s| s.rev))
            .field("dense_max", &self.dense_max)
            .field("decompose", &self.decompose)
            .finish()
    }
}

impl Default for NativeSolver {
    fn default() -> Self {
        Self::new()
    }
}

impl Clone for NativeSolver {
    /// Shares the cached structure.
    fn clone(&self) -> Self {
        NativeSolver {
            cache: self.cache.clone(),
            retired: Vec::new(),
            dense_max: self.dense_max,
            decompose: self.decompose,
            stats: self.stats,
            analysis: self.analysis.clone(),
            drag_mode: self.drag_mode,
        }
    }
}

/// What a run of the block solves produced.
struct Run {
    x: Vec<f64>,
    iterations: u32,
}

impl NativeSolver {
    /// A solver with an empty cache and the default settings.
    pub fn new() -> Self {
        NativeSolver {
            cache: None,
            retired: Vec::new(),
            dense_max: DENSE_MAX,
            decompose: true,
            stats: SolveStats::default(),
            analysis: None,
            drag_mode: DragMode::Auto,
        }
    }

    /// How drag frames are solved (default [`DragMode::Auto`]).
    pub fn with_drag_mode(mut self, mode: DragMode) -> Self {
        self.drag_mode = mode;
        self
    }

    /// Blocks with more than `n` unknowns use the sparse step (default [`DENSE_MAX`]).
    pub fn with_dense_limit(mut self, n: usize) -> Self {
        self.dense_max = n;
        self
    }

    /// With `false`, every component is solved as one block (no Dulmage–Mendelsohn split).
    pub fn with_decomposition(mut self, on: bool) -> Self {
        self.decompose = on;
        self
    }

    /// Statistics of the last solve, analysis or drag frame.
    pub fn stats(&self) -> SolveStats {
        self.stats
    }

    /// The equation system and (cached) structure of `sketch`; `None` if cancelled.
    fn prepare(
        &mut self,
        sketch: &Sketch,
        cancel: &dyn Fn() -> bool,
    ) -> Option<Result<(SketchSystem, Arc<Structure>), SketchError>> {
        let sys = match build_system_polled(sketch, characteristic_length(sketch), cancel)? {
            Ok(sys) => sys,
            Err(e) => return Some(Err(e)),
        };
        if cancel() {
            self.retire(sys);
            return None;
        }
        let (st, hit) = match &self.cache {
            Some(s) if s.matches(sketch, &sys, self.dense_max, self.decompose) => (s.clone(), true),
            _ => {
                let s = Arc::new(Structure::new(sketch, &sys, self.dense_max, self.decompose));
                self.cache = Some(s.clone());
                (s, false)
            }
        };
        let (blocks, largest) = st.block_stats();
        self.stats = SolveStats {
            cache_hit: hit,
            unknowns: st.unknowns(),
            substituted_rows: st.substituted_rows,
            components: st.components.len(),
            blocks,
            largest_block: largest,
            sparse_blocks: st
                .components
                .iter()
                .flat_map(|c| &c.blocks)
                .filter(|b| b.sparse.is_some())
                .count(),
            fallbacks: 0,
        };
        Some(Ok((sys, st)))
    }

    /// Keeps a cancelled solve's system alive so the cancel returns without dropping it.
    fn retire(&mut self, sys: SketchSystem) {
        if self.retired.len() < 4 {
            self.retired.push(sys);
        }
    }

    /// Full solve with per-param movement weights.
    fn solve_weighted(
        &mut self,
        sketch: &mut Sketch,
        x0: &[f64],
        weight: &dyn Fn(ParamId) -> f64,
        opts: &SolveOptions,
        cancel: &dyn Fn() -> bool,
    ) -> SolveReport {
        if cancel() {
            return SolveReport::cancelled();
        }
        let (sys, st) = match self.prepare(sketch, cancel) {
            None => return SolveReport::cancelled(),
            Some(Ok(v)) => v,
            Some(Err(_)) => return failed_report(sketch),
        };
        let tol = effective_tol(opts.tolerance, sys.length_scale);
        let Some(run) = self.run(&sys, &st, x0, weight, opts, tol, cancel) else {
            self.retire(sys);
            return SolveReport::cancelled();
        };
        let settings = self.settings();
        let Some(mut report) = evaluate(sketch, &sys, &st, &run, opts, tol, settings, cancel)
        else {
            self.retire(sys);
            return SolveReport::cancelled();
        };
        self.retired.clear();
        let params = sketch.params_mut();
        for (i, s) in st.slot.iter().enumerate() {
            if *s != Slot::Unused {
                params.set(ParamId(i as u32), run.x[i]);
            }
        }
        report.driven = measure_driven(sketch, sys.length_scale);
        report
    }

    /// Solves every component from `x0`; `None` if cancelled.
    #[allow(clippy::too_many_arguments, reason = "internal pipeline step")]
    fn run(
        &mut self,
        sys: &SketchSystem,
        st: &Structure,
        x0: &[f64],
        weight: &dyn Fn(ParamId) -> f64,
        opts: &SolveOptions,
        tol: f64,
        cancel: &dyn Fn() -> bool,
    ) -> Option<Run> {
        let mut x = x0.to_vec();
        for (i, s) in st.slot.iter().enumerate() {
            if let Slot::Fixed { src } = *s
                && st.fixed.binary_search(&ParamId(i as u32)).is_err()
            {
                x[i] = x0[src as usize];
            }
        }
        // Class values: the weighted mean of the members (minimum weighted movement that
        // satisfies the substituted equations).
        let mut u = Vec::with_capacity(st.members.len());
        let mut w = Vec::with_capacity(st.members.len());
        for ms in &st.members {
            let (mut sw, mut sx) = (0.0, 0.0);
            for &p in ms {
                let wp = weight(p);
                sw += wp;
                sx += wp * x0[p.index()];
            }
            u.push(sx / sw);
            w.push(sw);
            for &p in ms {
                x[p.index()] = sx / sw;
            }
        }
        let ctx = BlockCtx {
            sys: &sys.system,
            st,
            tol,
            length_scale: sys.length_scale,
            algorithm: opts.algorithm,
            max_iterations: opts.max_iterations,
            cancel,
        };
        let mut ws = Workspace::default();
        let mut iterations = 0;
        for comp in &st.components {
            if cancel() {
                return None;
            }
            let start: Vec<f64> = comp.unknowns.iter().map(|&k| u[k as usize]).collect();
            for b in &comp.blocks {
                iterations += solve_sub(&ctx, b, &mut x, &mut u, &w, &mut ws)?;
            }
            if comp.blocks.len() > 1 {
                let whole = comp.whole();
                let mut f = vec![0.0; whole.rows];
                block::eval(&sys.system, whole, &x, &mut f, &mut ws);
                let r_split = max_abs(&f);
                if r_split > tol {
                    self.stats.fallbacks += 1;
                    let split: Vec<f64> = comp.unknowns.iter().map(|&k| u[k as usize]).collect();
                    for (i, &k) in comp.unknowns.iter().enumerate() {
                        u[k as usize] = start[i];
                    }
                    iterations += solve_sub(&ctx, whole, &mut x, &mut u, &w, &mut ws)?;
                    block::eval(&sys.system, whole, &x, &mut f, &mut ws);
                    if max_abs(&f) >= r_split {
                        for (i, &k) in comp.unknowns.iter().enumerate() {
                            u[k as usize] = split[i];
                        }
                        block::scatter(st, whole, &split, &mut x);
                    }
                }
            }
        }
        Some(Run { x, iterations })
    }
}

/// Solves one subsystem, updating the global class values `u`; `None` if cancelled.
fn solve_sub(
    ctx: &BlockCtx<'_>,
    sub: &Sub,
    x: &mut [f64],
    u: &mut [f64],
    w: &[f64],
    ws: &mut Workspace,
) -> Option<u32> {
    let mut ul: Vec<f64> = sub.unknowns.iter().map(|&k| u[k as usize]).collect();
    let wl: Vec<f64> = sub.unknowns.iter().map(|&k| w[k as usize]).collect();
    match solve_block(ctx, sub, x, &mut ul, &wl, ws) {
        BlockOutcome::Cancelled => None,
        BlockOutcome::Done(it) => {
            for (i, &k) in sub.unknowns.iter().enumerate() {
                u[k as usize] = ul[i];
            }
            Some(it)
        }
    }
}

fn max_abs(v: &[f64]) -> f64 {
    v.iter().fold(0.0_f64, |m, x| m.max(x.abs()))
}

/// `max(tolerance, 64 ε L_c)`: the requested tolerance, but never below what `f64` can reach
/// at the sketch's size.
fn effective_tol(tolerance: f64, length_scale: f64) -> f64 {
    tolerance.max(64.0 * f64::EPSILON * length_scale)
}

/// Report for a sketch whose equations cannot be built (should not happen for a validated
/// sketch).
fn failed_report(sketch: &Sketch) -> SolveReport {
    SolveReport {
        status: SolveStatus::DidNotConverge,
        iterations: 0,
        max_residual: f64::INFINITY,
        unsatisfied: sketch.constraints().map(|(id, _)| id).collect(),
        driven: BTreeMap::new(),
        dof: None,
        diagnosis: None,
    }
}

/// Numeric rank of a subsystem at `x` (dense up to [`RANK_DENSE_MAX`], structural above).
fn sub_rank(sys: &EquationSystem, sub: &Sub, x: &[f64], ws: &mut Workspace) -> usize {
    let (m, n) = (sub.rows, sub.n());
    if n > RANK_DENSE_MAX || m > RANK_DENSE_MAX {
        return match sub.kind {
            SubKind::Over => n,
            SubKind::Square | SubKind::Under => m,
            SubKind::Whole => m.min(n),
        };
    }
    let mut vals = vec![0.0; sub.nnz()];
    block::jacobian(sys, sub, x, &mut vals, ws);
    numeric_rank(&to_dense(sub, &vals), m, n)
}

/// Rank of the substituted system at `x` per component: sum of block ranks when every block
/// has the rank its shape allows (then the block-triangular rank is exactly the sum), else the
/// rank of the whole component. `None` if cancelled.
fn ranks(
    sys: &EquationSystem,
    st: &Structure,
    x: &[f64],
    cancel: &dyn Fn() -> bool,
) -> Option<(usize, bool)> {
    let mut ws = Workspace::default();
    let (mut total, mut deficient) = (0, false);
    for comp in &st.components {
        let mut sum = 0;
        let mut nice = true;
        for b in &comp.blocks {
            if cancel() {
                return None;
            }
            let r = sub_rank(sys, b, x, &mut ws);
            nice &= match b.kind {
                SubKind::Over => r == b.n(),
                SubKind::Square | SubKind::Under => r == b.rows,
                SubKind::Whole => true,
            };
            sum += r;
        }
        let rank = if nice || comp.blocks.len() == 1 {
            sum
        } else {
            if cancel() {
                return None;
            }
            sub_rank(sys, comp.whole(), x, &mut ws)
        };
        deficient |= rank < comp.rows;
        total += rank;
    }
    Some((total, deficient))
}

/// Status, residuals, rank/DOF and diagnosis at the end of a run; `None` if cancelled.
#[allow(clippy::too_many_arguments, reason = "internal pipeline step")]
fn evaluate(
    sketch: &Sketch,
    sys: &SketchSystem,
    st: &Structure,
    run: &Run,
    opts: &SolveOptions,
    tol: f64,
    settings: Settings,
    cancel: &dyn Fn() -> bool,
) -> Option<SolveReport> {
    let x = &run.x;
    let system = &sys.system;
    let mut f = vec![0.0; system.rows()];
    system.eval(x, &mut f);
    let max_residual = max_abs(&f);
    let mut unsatisfied: Vec<ConstraintId> = system
        .iter()
        .filter(|(_, _, rows)| f[rows.clone()].iter().any(|v| v.abs() > tol || v.is_nan()))
        .filter_map(|(owner, _, _)| match owner {
            EquationOwner::Constraint(id) => Some(id),
            EquationOwner::Entity(_) => None,
        })
        .collect();
    unsatisfied.sort_unstable();
    unsatisfied.dedup();
    let satisfied = max_residual <= tol;
    let conflict_join = st
        .fixed_joins
        .iter()
        .any(|j| (x[j.a as usize] - x[j.b as usize]).abs() > tol);
    let mut deficient =
        !st.redundant_subst.is_empty() || !st.const_blocks.is_empty() || conflict_join;
    let rank_test = opts.diagnose || opts.dof != DofAnalysis::None;
    let structural_deficit = st.components.iter().any(|c| {
        c.blocks
            .iter()
            .any(|b| b.kind == SubKind::Over && b.rows > b.n())
    });
    deficient |= structural_deficit;
    let mut dof = None;
    if rank_test {
        let (rank, def) = ranks(system, st, x, cancel)?;
        deficient |= def;
        dof = Some(u32::try_from(st.unknowns() - rank.min(st.unknowns())).unwrap_or(u32::MAX));
    }
    let status = match (satisfied, deficient) {
        (true, false) => SolveStatus::Converged,
        (true, true) => SolveStatus::ConvergedRedundant,
        (false, true) => SolveStatus::Inconsistent,
        (false, false) => SolveStatus::DidNotConverge,
    };
    let want_groups = opts.diagnose && status != SolveStatus::Converged;
    let analysis = if want_groups || opts.dof == DofAnalysis::PerEntity {
        Some(analyze_at(sketch, x, sys, st, settings, cancel)?)
    } else {
        None
    };
    let (dof, diagnosis) = match (opts.dof, analysis) {
        (DofAnalysis::None, a) => (None, a.filter(|_| want_groups).map(|a| a.diagnosis)),
        (DofAnalysis::Total, a) => (
            dof.map(|dof| DofReport {
                dof,
                entities: BTreeMap::new(),
            }),
            a.filter(|_| want_groups).map(|a| a.diagnosis),
        ),
        (DofAnalysis::PerEntity, Some(a)) => (Some(a.dof), want_groups.then_some(a.diagnosis)),
        (DofAnalysis::PerEntity, None) => (None, None),
    };
    Some(SolveReport {
        status,
        iterations: run.iterations,
        max_residual,
        unsatisfied,
        driven: BTreeMap::new(),
        dof,
        diagnosis,
    })
}

/// Measured values of the driven dimensions at the sketch's current values.
fn measure_driven(sketch: &Sketch, length_scale: f64) -> BTreeMap<ConstraintId, f64> {
    let x = sketch.params().as_slice();
    let mut out = BTreeMap::new();
    for (id, c) in sketch.constraints() {
        let ConstraintKind::Dimension(d) = &c.kind else {
            continue;
        };
        if d.driving {
            continue;
        }
        let Ok(Some(r)) = build(sketch, &c.kind, &c.aux, length_scale, true) else {
            continue;
        };
        let mut v = vec![0.0; r.rows()];
        r.eval(x, &mut v);
        // Every dimension row is `measured − value` (mm), angles scaled by `L_c`.
        let scale = match d.kind {
            DimensionKind::Angle { .. } => length_scale,
            _ => 1.0,
        };
        out.insert(id, d.value + v[0] / scale);
    }
    out
}

impl SolverBackend for NativeSolver {
    fn name(&self) -> &'static str {
        "native"
    }

    fn solve(
        &mut self,
        sketch: &mut Sketch,
        opts: &SolveOptions,
        cancel: &dyn Fn() -> bool,
    ) -> SolveReport {
        let x0 = sketch.params().as_slice().to_vec();
        self.solve_weighted(sketch, &x0, &|_| 1.0, opts, cancel)
    }

    fn analyze(&mut self, sketch: &Sketch) -> (DofReport, Diagnosis) {
        match self.base_analysis(sketch, &|| false) {
            Some(b) => (b.analysis.dof.clone(), b.analysis.diagnosis.clone()),
            None => (DofReport::default(), Diagnosis::default()),
        }
    }

    fn check_constraint(
        &mut self,
        sketch: &Sketch,
        candidate: &ConstraintKind,
    ) -> Result<ConstraintCheck, SketchError> {
        self.check(sketch, candidate)
    }

    fn begin_drag(
        &mut self,
        sketch: &Sketch,
        spec: DragSpec,
    ) -> Result<Box<dyn DragSession>, SketchError> {
        Ok(Box::new(self.drag(sketch, spec)?))
    }
}
