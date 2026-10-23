//! SK-13 prototype native solver (ADR-007 option A). **Throwaway quality, but correct.**
//!
//! Scope (note `docs/notes/sketch-solver.md` §5.1): substitution (union-find aliasing of
//! coincident / horizontal / vertical / concentric / equal-radius equations and fixed params) →
//! connected components → per component a damped, weighted minimum-norm Gauss–Newton
//! (Levenberg–Marquardt on `J W⁻¹ Jᵀ`, first step pure Gauss–Newton) or Powell dogleg, dense,
//! with hand-written linear algebra. Optional rank/DOF per component and a QR-style
//! dependency-group diagnosis on the unsubstituted Jacobian. Dragging (weighted min-norm and a
//! PlaneGCS-style SQP alternative) is in [`ProtoDrag`].
//!
//! Not here (SK-03/04/05): Dulmage–Mendelsohn blocks, sparse factorisations, a structure cache
//! across value edits, cancellation, per-entity DOF status, witness tie-breaks.
//!
//! The residuals are SK-02's ([`build_system`](super::build_system)), so chirality (sides,
//! tangent kinds, angle branches) is part of the equations.

mod diagnose;
mod drag;
pub(crate) mod linalg;
mod plan;

use std::time::Instant;

pub use drag::{DragFrame, DragMethod, ProtoDrag};

use self::linalg::{chol_semidef, chol_solve, gram, j_mul, jt_mul, max_abs, norm2};
use self::plan::{Component, Plan};
use super::rank::numeric_rank;
use super::system::EquationOwner;
use crate::error::SketchError;
use crate::model::Sketch;

/// Nonlinear iteration of the prototype.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ProtoAlgorithm {
    /// Damped weighted min-norm Gauss–Newton; `λ` starts at 0 and adapts (×10 / ÷10).
    LevenbergMarquardt,
    /// Powell dogleg (PlaneGCS default) with the min-norm Gauss–Newton step.
    DogLeg,
}

/// Options of [`solve`].
#[derive(Clone, Debug)]
pub struct ProtoOptions {
    /// Iteration scheme.
    pub algorithm: ProtoAlgorithm,
    /// Alias `a − b = 0` equations instead of solving them (default on).
    pub substitution: bool,
    /// Residual tolerance in mm; `None` = `max(1e-10, 64 ε L_c)`.
    pub tolerance: Option<f64>,
    /// Linear solves per component (accepted + rejected steps), default 100.
    pub max_iterations: u32,
    /// Rank test per component after the solve: DOF and `ConvergedRedundant` (default on).
    pub analyze: bool,
    /// Dependency groups when the result is redundant or failed (default on).
    pub diagnose: bool,
}

impl Default for ProtoOptions {
    fn default() -> Self {
        ProtoOptions {
            algorithm: ProtoAlgorithm::LevenbergMarquardt,
            substitution: true,
            tolerance: None,
            max_iterations: 100,
            analyze: true,
            diagnose: true,
        }
    }
}

impl ProtoOptions {
    /// Solve only: no rank test and no diagnosis (the inner loop of warm solves and drags).
    pub fn fast() -> Self {
        ProtoOptions {
            analyze: false,
            diagnose: false,
            ..Self::default()
        }
    }
}

/// Result status (S03 `SolveStatus` without `Cancelled`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ProtoStatus {
    /// All equations satisfied and independent (or not analysed).
    Converged,
    /// All equations satisfied, some are dependent.
    ConvergedRedundant,
    /// Not satisfied and dependent: contradictory constraints.
    Inconsistent,
    /// Not satisfied, no dependency found.
    DidNotConverge,
}

/// Kind of a dependency group.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ProtoGroupKind {
    /// Dependent and satisfied.
    Redundant,
    /// Dependent and contradictory.
    Conflicting,
}

/// A set of equation owners whose rows are linearly dependent (one circuit).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct ProtoGroup {
    /// Kind.
    pub kind: ProtoGroupKind,
    /// Owners, sorted and distinct.
    pub owners: Vec<EquationOwner>,
}

/// Result of [`solve`].
#[derive(Clone, Debug)]
pub struct ProtoReport {
    /// Status.
    pub status: ProtoStatus,
    /// Linear solves, summed over components.
    pub iterations: u32,
    /// Largest residual over the whole unsubstituted system (mm).
    pub max_residual: f64,
    /// Tolerance used (mm).
    pub tolerance: f64,
    /// Remaining DOF (`free params − rank J`), when analysed.
    pub dof: Option<u32>,
    /// Dependency groups (diagnosis), sorted.
    pub groups: Vec<ProtoGroup>,
    /// Unknowns after substitution.
    pub unknowns: usize,
    /// Rows removed by substitution.
    pub substituted_rows: usize,
    /// Connected components solved.
    pub components: usize,
    /// Unknowns of the largest component.
    pub largest_component: usize,
    /// Wall time split: building the plan, iterating, analysis + diagnosis (seconds).
    pub timing: [f64; 3],
}

/// Outcome of one component solve.
pub(crate) struct CompOutcome {
    pub iterations: u32,
}

/// Solves `sketch` in place from its current values.
pub fn solve(sketch: &mut Sketch, opts: &ProtoOptions) -> Result<ProtoReport, SketchError> {
    let t0 = Instant::now();
    let plan = Plan::new(sketch, opts)?;
    let t1 = Instant::now();
    let x0 = sketch.params().as_slice().to_vec();
    let (u, w) = plan.gather(&x0, &|_| 1.0);
    let mut x = plan.expand(&x0, &u);
    let mut iterations = 0;
    for c in &plan.components {
        let mut uc: Vec<f64> = c.unknowns.iter().map(|&k| u[k as usize]).collect();
        let wc: Vec<f64> = c.unknowns.iter().map(|&k| w[k as usize]).collect();
        let out = solve_component(&plan, c, &mut x, &mut uc, &wc, opts);
        iterations += out.iterations;
    }
    let t2 = Instant::now();
    write_back(&plan, sketch, &x);
    let mut report = finish(&plan, &x, opts, iterations);
    report.timing = [
        (t1 - t0).as_secs_f64(),
        (t2 - t1).as_secs_f64(),
        t2.elapsed().as_secs_f64(),
    ];
    Ok(report)
}

/// Writes every system parameter of `x` into the sketch.
pub(crate) fn write_back(plan: &Plan, sketch: &mut Sketch, x: &[f64]) {
    let params = sketch.params_mut();
    for (i, s) in plan.slot.iter().enumerate() {
        if *s != plan::Slot::Unused {
            params.set(crate::ids::ParamId(i as u32), x[i]);
        }
    }
}

/// Status, DOF and diagnosis at the final `x`.
pub(crate) fn finish(plan: &Plan, x: &[f64], opts: &ProtoOptions, iterations: u32) -> ProtoReport {
    let max_residual = plan.max_residual(x);
    let satisfied = max_residual <= plan.tol;
    let mut deficient = !plan.redundant_subst.is_empty()
        || !plan.conflict_subst.is_empty()
        || !plan.const_blocks.is_empty();
    let mut dof = None;
    if opts.analyze {
        let mut rank_total = 0;
        for c in &plan.components {
            let n = c.unknowns.len();
            let mut j = vec![0.0; c.rows * n];
            plan.jacobian(c, x, &mut j);
            let r = numeric_rank(&j, c.rows, n);
            if r < c.rows {
                deficient = true;
            }
            rank_total += r;
        }
        dof = Some(u32::try_from(plan.unknowns() - rank_total).unwrap_or(u32::MAX));
    }
    let status = match (satisfied, deficient) {
        (true, false) => ProtoStatus::Converged,
        (true, true) => ProtoStatus::ConvergedRedundant,
        (false, true) => ProtoStatus::Inconsistent,
        (false, false) => ProtoStatus::DidNotConverge,
    };
    let groups = if opts.diagnose && status != ProtoStatus::Converged {
        diagnose::groups(plan, x)
    } else {
        Vec::new()
    };
    ProtoReport {
        status,
        iterations,
        max_residual,
        tolerance: plan.tol,
        dof,
        groups,
        unknowns: plan.unknowns(),
        substituted_rows: plan.substituted_rows,
        components: plan.components.len(),
        largest_component: plan
            .components
            .iter()
            .map(|c| c.unknowns.len())
            .max()
            .unwrap_or(0),
        timing: [0.0; 3],
    }
}

/// Relative pivot threshold of the step factorisation.
pub(crate) const PIVOT_REL: f64 = 1e-11;

/// Scaled Jacobian `J W^{-1/2}` (in place) for weights `w`.
fn scale_columns(j: &mut [f64], m: usize, n: usize, inv_sqrt_w: &[f64]) {
    for r in 0..m {
        for (v, s) in j[r * n..(r + 1) * n].iter_mut().zip(inv_sqrt_w) {
            *v *= s;
        }
    }
}

/// Minimum-norm Gauss–Newton step in scaled variables: `dv = −Jsᵀ (Js Jsᵀ + λI)⁺ F`.
fn min_norm_step(js: &[f64], m: usize, n: usize, f: &[f64], lambda: f64) -> Vec<f64> {
    let mut a = gram(js, m, n, lambda);
    let dep = chol_semidef(&mut a, m, PIVOT_REL);
    let z = chol_solve(&a, &dep, m, f);
    jt_mul(js, m, n, &z).into_iter().map(|v| -v).collect()
}

/// Solves one component in place (`x` and the local unknowns `uc`).
pub(crate) fn solve_component(
    plan: &Plan,
    c: &Component,
    x: &mut [f64],
    uc: &mut [f64],
    wc: &[f64],
    opts: &ProtoOptions,
) -> CompOutcome {
    let (m, n) = (c.rows, c.unknowns.len());
    let inv_sqrt_w: Vec<f64> = wc.iter().map(|w| 1.0 / w.sqrt()).collect();
    let mut f = vec![0.0; m];
    let mut j = vec![0.0; m * n];
    plan.scatter(c, uc, x);
    plan.eval(c, x, &mut f);
    let mut iterations = 0;
    let max_it = opts.max_iterations;
    let mut trial_x = x.to_vec();
    let mut fnew = vec![0.0; m];
    let try_step = |uc: &[f64], dv: &[f64], fnew: &mut [f64], trial_x: &mut Vec<f64>| -> Vec<f64> {
        let un: Vec<f64> = uc
            .iter()
            .zip(dv)
            .zip(&inv_sqrt_w)
            .map(|((u, d), s)| u + d * s)
            .collect();
        plan.scatter(c, &un, trial_x);
        plan.eval(c, trial_x, fnew);
        un
    };
    let mut slow = 0;
    match opts.algorithm {
        ProtoAlgorithm::LevenbergMarquardt => {
            let mut lambda = 0.0_f64;
            while max_abs(&f) > plan.tol && iterations < max_it {
                plan.jacobian(c, x, &mut j);
                scale_columns(&mut j, m, n, &inv_sqrt_w);
                let dmax = (0..m)
                    .map(|r| norm2(&j[r * n..(r + 1) * n]))
                    .fold(0.0_f64, f64::max)
                    .max(1e-300);
                let floor = 1e-9 * dmax;
                let mut accepted = false;
                while iterations < max_it {
                    iterations += 1;
                    let dv = min_norm_step(&j, m, n, &f, lambda);
                    let un = try_step(uc, &dv, &mut fnew, &mut trial_x);
                    let (n0, n1) = (norm2(&f), norm2(&fnew));
                    if n1.is_finite() && (n1 < n0 || max_abs(&fnew) <= plan.tol) {
                        slow = if n1 > 0.98 * n0 { slow + 1 } else { 0 };
                        uc.copy_from_slice(&un);
                        x.copy_from_slice(&trial_x);
                        std::mem::swap(&mut f, &mut fnew);
                        lambda = if lambda <= floor { 0.0 } else { lambda / 10.0 };
                        accepted = true;
                        break;
                    }
                    trial_x.copy_from_slice(x);
                    lambda = (lambda * 10.0).max(floor);
                    if lambda > 1e12 * dmax {
                        break;
                    }
                }
                if !accepted || slow >= 8 {
                    break;
                }
            }
        }
        ProtoAlgorithm::DogLeg => {
            let mut delta = plan.sys.length_scale.max(1.0);
            while max_abs(&f) > plan.tol && iterations < max_it {
                iterations += 1;
                plan.jacobian(c, x, &mut j);
                scale_columns(&mut j, m, n, &inv_sqrt_w);
                let h_gn = min_norm_step(&j, m, n, &f, 0.0);
                let g = jt_mul(&j, m, n, &f);
                let jg = j_mul(&j, m, n, &g);
                let (gg, jgg) = (norm2(&g), norm2(&jg));
                let alpha = if jgg > 0.0 { gg / jgg } else { 0.0 };
                let h_sd: Vec<f64> = g.iter().map(|v| -alpha * v).collect();
                let (ngn, nsd) = (norm2(&h_gn).sqrt(), norm2(&h_sd).sqrt());
                let h: Vec<f64> = if ngn <= delta {
                    h_gn
                } else if nsd >= delta {
                    h_sd.iter().map(|v| v * delta / nsd).collect()
                } else {
                    // β with ‖h_sd + β (h_gn − h_sd)‖ = Δ.
                    let d: Vec<f64> = h_gn.iter().zip(&h_sd).map(|(a, b)| a - b).collect();
                    let (a2, b1, c0) = (
                        norm2(&d),
                        2.0 * h_sd.iter().zip(&d).map(|(a, b)| a * b).sum::<f64>(),
                        nsd * nsd - delta * delta,
                    );
                    let beta = (-b1 + (b1 * b1 - 4.0 * a2 * c0).max(0.0).sqrt()) / (2.0 * a2);
                    h_sd.iter().zip(&d).map(|(s, d)| s + beta * d).collect()
                };
                let jh = j_mul(&j, m, n, &h);
                let n0 = norm2(&f);
                let pred = n0
                    - f.iter()
                        .zip(&jh)
                        .map(|(a, b)| (a + b) * (a + b))
                        .sum::<f64>();
                let un = try_step(uc, &h, &mut fnew, &mut trial_x);
                let n1 = norm2(&fnew);
                let rho = if pred > 0.0 { (n0 - n1) / pred } else { -1.0 };
                let hn = norm2(&h).sqrt();
                if n1.is_finite() && (n1 < n0 || max_abs(&fnew) <= plan.tol) {
                    slow = if n1 > 0.98 * n0 { slow + 1 } else { 0 };
                    uc.copy_from_slice(&un);
                    x.copy_from_slice(&trial_x);
                    std::mem::swap(&mut f, &mut fnew);
                } else {
                    trial_x.copy_from_slice(x);
                }
                if rho > 0.75 {
                    delta = delta.max(3.0 * hn);
                } else if rho < 0.25 {
                    delta = 0.5 * hn.min(delta);
                }
                let scale = uc.iter().fold(1.0_f64, |a, v| a.max(v.abs()));
                if delta < 1e-14 * scale || slow >= 8 {
                    break;
                }
            }
        }
    }
    CompOutcome { iterations }
}

#[cfg(test)]
mod tests;
