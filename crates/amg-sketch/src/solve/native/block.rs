//! Per-block numerics: evaluation and Jacobian assembly into the cached CSR pattern, the damped
//! weighted minimum-norm step (dense or sparse) and the nonlinear iterations (note §3.2).
//!
//! In scaled variables `v = W^{1/2} u` the step is `dv = −Jsᵀ (Js Jsᵀ + λI)⁺ F` with
//! `Js = J W^{-1/2}`: the minimum weighted-norm Gauss–Newton step when `λ = 0`. Dependent rows
//! of a redundant block are dropped by the semidefinite factorisation, so consistent redundant
//! systems still converge quadratically.

use super::Workspace;
use super::structure::{NONE, Structure, Sub};
use crate::solve::api::Algorithm;
use crate::solve::proto::linalg::{chol_semidef, chol_solve, gram, max_abs, norm2};
use crate::solve::system::EquationSystem;

/// Relative pivot threshold of the step factorisation (on `J Jᵀ`).
pub(crate) const PIVOT_REL: f64 = 1e-11;

/// Settings of one block solve.
pub(crate) struct BlockCtx<'a> {
    pub sys: &'a EquationSystem,
    pub st: &'a Structure,
    pub tol: f64,
    pub length_scale: f64,
    pub algorithm: Algorithm,
    pub max_iterations: u32,
    pub cancel: &'a dyn Fn() -> bool,
}

/// Result of one block solve.
pub(crate) enum BlockOutcome {
    /// Finished (converged or not); linear solves used.
    Done(u32),
    /// The cancel callback fired.
    Cancelled,
}

/// Residuals of `sub` at `x` into `f[..sub.rows]`.
pub(crate) fn eval(sys: &EquationSystem, sub: &Sub, x: &[f64], f: &mut [f64], ws: &mut Workspace) {
    for part in &sub.parts {
        let res = sys.residual(part.res as usize);
        ws.tmp.clear();
        ws.tmp.resize(res.rows(), 0.0);
        res.eval(x, &mut ws.tmp);
        for (k, &r) in part.rows.iter().enumerate() {
            f[part.first_row as usize + k] = ws.tmp[r as usize];
        }
    }
}

/// Jacobian values of `sub` at `x` in its CSR pattern (`vals.len() == sub.nnz()`).
pub(crate) fn jacobian(
    sys: &EquationSystem,
    sub: &Sub,
    x: &[f64],
    vals: &mut [f64],
    ws: &mut Workspace,
) {
    vals.fill(0.0);
    for part in &sub.parts {
        let res = sys.residual(part.res as usize);
        let np = part.np as usize;
        ws.tmp.clear();
        ws.tmp.resize(res.rows() * np, 0.0);
        res.jacobian(x, &mut ws.tmp);
        let mut s = part.scatter_off as usize;
        for &r in &part.rows {
            let row = &ws.tmp[r as usize * np..(r as usize + 1) * np];
            for &v in row {
                let idx = sub.scatter[s];
                if idx != NONE {
                    vals[idx as usize] += v;
                }
                s += 1;
            }
        }
    }
}

/// Dense row-major copy of a CSR matrix.
pub(crate) fn to_dense(sub: &Sub, vals: &[f64]) -> Vec<f64> {
    let n = sub.n();
    let mut d = vec![0.0; sub.rows * n];
    for r in 0..sub.rows {
        for k in sub.row_ptr[r] as usize..sub.row_ptr[r + 1] as usize {
            d[r * n + sub.col_idx[k] as usize] += vals[k];
        }
    }
    d
}

/// `y = Jᵀ z` (CSR).
fn jt_mul_csr(sub: &Sub, vals: &[f64], z: &[f64]) -> Vec<f64> {
    let mut y = vec![0.0; sub.n()];
    for (r, &zr) in z.iter().enumerate().take(sub.rows) {
        if zr != 0.0 {
            for k in sub.row_ptr[r] as usize..sub.row_ptr[r + 1] as usize {
                y[sub.col_idx[k] as usize] += vals[k] * zr;
            }
        }
    }
    y
}

/// `y = J v` (CSR).
fn j_mul_csr(sub: &Sub, vals: &[f64], v: &[f64]) -> Vec<f64> {
    (0..sub.rows)
        .map(|r| {
            (sub.row_ptr[r] as usize..sub.row_ptr[r + 1] as usize)
                .map(|k| vals[k] * v[sub.col_idx[k] as usize])
                .sum()
        })
        .collect()
}

/// Minimum-norm step `dv = −Jsᵀ (Js Jsᵀ + λI)⁺ f` for scaled CSR values `js`.
fn min_norm_step(sub: &Sub, js: &[f64], f: &[f64], lambda: f64) -> Vec<f64> {
    let m = sub.rows;
    let z = match &sub.sparse {
        Some(sp) => sp
            .solve(js, lambda, PIVOT_REL, f)
            .unwrap_or_else(|| vec![0.0; m]),
        None => {
            let n = sub.n();
            let d = to_dense(sub, js);
            let mut a = gram(&d, m, n, lambda);
            let dep = chol_semidef(&mut a, m, PIVOT_REL);
            chol_solve(&a, &dep, m, f)
        }
    };
    jt_mul_csr(sub, js, &z).into_iter().map(|v| -v).collect()
}

/// Writes local unknowns `u` into `x` (all members of each class).
pub(crate) fn scatter(st: &Structure, sub: &Sub, u: &[f64], x: &mut [f64]) {
    for (i, &k) in sub.unknowns.iter().enumerate() {
        for p in &st.members[k as usize] {
            x[p.index()] = u[i];
        }
    }
}

/// Solves `sub` in place: local unknowns `u` (scattered into `x`), weights `w`.
pub(crate) fn solve_block(
    ctx: &BlockCtx<'_>,
    sub: &Sub,
    x: &mut [f64],
    u: &mut [f64],
    w: &[f64],
    ws: &mut Workspace,
) -> BlockOutcome {
    let (m, n) = (sub.rows, sub.n());
    let inv_sqrt_w: Vec<f64> = w.iter().map(|w| 1.0 / w.sqrt()).collect();
    let mut f = vec![0.0; m];
    let mut fnew = vec![0.0; m];
    let mut js = vec![0.0; sub.nnz()];
    scatter(ctx.st, sub, u, x);
    eval(ctx.sys, sub, x, &mut f, ws);
    if max_abs(&f) <= ctx.tol {
        return BlockOutcome::Done(0);
    }
    let mut trial_x = x.to_vec();
    let mut un = vec![0.0; n];
    let mut iterations = 0u32;
    let mut accepted_steps = 0u32;
    let mut slow = 0;
    // Trial point `u + dv / sqrt(w)`; returns ‖F‖² there.
    let try_step = |u: &[f64],
                    dv: &[f64],
                    un: &mut [f64],
                    trial_x: &mut [f64],
                    fnew: &mut [f64],
                    ws: &mut Workspace| {
        for i in 0..n {
            un[i] = u[i] + dv[i] * inv_sqrt_w[i];
        }
        scatter(ctx.st, sub, un, trial_x);
        eval(ctx.sys, sub, trial_x, fnew, ws);
        norm2(fnew)
    };
    let scale_cols = |js: &mut [f64]| {
        for (v, c) in js.iter_mut().zip(&sub.col_idx) {
            *v *= inv_sqrt_w[*c as usize];
        }
    };
    let accept =
        |n0: f64, n1: f64, fnew: &[f64]| n1.is_finite() && (n1 < n0 || max_abs(fnew) <= ctx.tol);

    match ctx.algorithm {
        Algorithm::Auto | Algorithm::LevenbergMarquardt | Algorithm::GaussNewtonMinNorm => {
            let gn = ctx.algorithm == Algorithm::GaussNewtonMinNorm;
            let mut lambda = 0.0_f64;
            while max_abs(&f) > ctx.tol && accepted_steps < ctx.max_iterations {
                if (ctx.cancel)() {
                    return BlockOutcome::Cancelled;
                }
                jacobian(ctx.sys, sub, x, &mut js, ws);
                scale_cols(&mut js);
                let dmax = row_norm_max(sub, &js).max(1e-300);
                let floor = 1e-9 * dmax;
                let n0 = norm2(&f);
                let mut accepted = false;
                if gn {
                    // Pure Gauss–Newton with step halving.
                    let dv = min_norm_step(sub, &js, &f, 0.0);
                    iterations += 1;
                    let mut t = 1.0;
                    for _ in 0..12 {
                        let dvt: Vec<f64> = dv.iter().map(|d| d * t).collect();
                        let n1 = try_step(u, &dvt, &mut un, &mut trial_x, &mut fnew, ws);
                        if accept(n0, n1, &fnew) {
                            slow = if n1 > 0.98 * n0 { slow + 1 } else { 0 };
                            accepted = true;
                            break;
                        }
                        trial_x.copy_from_slice(x);
                        t *= 0.5;
                    }
                } else {
                    while iterations < ctx.max_iterations.saturating_mul(8) {
                        iterations += 1;
                        let dv = min_norm_step(sub, &js, &f, lambda);
                        let n1 = try_step(u, &dv, &mut un, &mut trial_x, &mut fnew, ws);
                        if accept(n0, n1, &fnew) {
                            slow = if n1 > 0.98 * n0 { slow + 1 } else { 0 };
                            lambda = if lambda <= floor { 0.0 } else { lambda / 10.0 };
                            accepted = true;
                            break;
                        }
                        trial_x.copy_from_slice(x);
                        lambda = (lambda * 10.0).max(floor);
                        if lambda > 1e12 * dmax || (ctx.cancel)() {
                            break;
                        }
                    }
                }
                if !accepted {
                    break;
                }
                accepted_steps += 1;
                u.copy_from_slice(&un);
                x.copy_from_slice(&trial_x);
                std::mem::swap(&mut f, &mut fnew);
                if slow >= 8 {
                    break;
                }
            }
        }
        Algorithm::DogLeg => {
            let mut delta = ctx.length_scale.max(1.0);
            while max_abs(&f) > ctx.tol && accepted_steps < ctx.max_iterations {
                if (ctx.cancel)() {
                    return BlockOutcome::Cancelled;
                }
                iterations += 1;
                jacobian(ctx.sys, sub, x, &mut js, ws);
                scale_cols(&mut js);
                let h_gn = min_norm_step(sub, &js, &f, 0.0);
                let g = jt_mul_csr(sub, &js, &f);
                let jg = j_mul_csr(sub, &js, &g);
                let (gg, jgg) = (norm2(&g), norm2(&jg));
                let alpha = if jgg > 0.0 { gg / jgg } else { 0.0 };
                let h_sd: Vec<f64> = g.iter().map(|v| -alpha * v).collect();
                let (ngn, nsd) = (norm2(&h_gn).sqrt(), norm2(&h_sd).sqrt());
                let h: Vec<f64> = if ngn <= delta {
                    h_gn
                } else if nsd >= delta {
                    h_sd.iter().map(|v| v * delta / nsd).collect()
                } else {
                    let d: Vec<f64> = h_gn.iter().zip(&h_sd).map(|(a, b)| a - b).collect();
                    let (a2, b1, c0) = (
                        norm2(&d),
                        2.0 * h_sd.iter().zip(&d).map(|(a, b)| a * b).sum::<f64>(),
                        nsd * nsd - delta * delta,
                    );
                    let beta = (-b1 + (b1 * b1 - 4.0 * a2 * c0).max(0.0).sqrt()) / (2.0 * a2);
                    h_sd.iter().zip(&d).map(|(s, d)| s + beta * d).collect()
                };
                let jh = j_mul_csr(sub, &js, &h);
                let n0 = norm2(&f);
                let pred = n0
                    - f.iter()
                        .zip(&jh)
                        .map(|(a, b)| (a + b) * (a + b))
                        .sum::<f64>();
                let n1 = try_step(u, &h, &mut un, &mut trial_x, &mut fnew, ws);
                let rho = if pred > 0.0 { (n0 - n1) / pred } else { -1.0 };
                let hn = norm2(&h).sqrt();
                if accept(n0, n1, &fnew) {
                    slow = if n1 > 0.98 * n0 { slow + 1 } else { 0 };
                    accepted_steps += 1;
                    u.copy_from_slice(&un);
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
                let scale = u.iter().fold(1.0_f64, |a, v| a.max(v.abs()));
                if delta < 1e-14 * scale || slow >= 8 || iterations > 4 * ctx.max_iterations {
                    break;
                }
            }
        }
    }
    BlockOutcome::Done(iterations)
}

/// Largest row 2-norm squared of a CSR matrix (the Gram diagonal maximum).
fn row_norm_max(sub: &Sub, vals: &[f64]) -> f64 {
    (0..sub.rows)
        .map(|r| norm2(&vals[sub.row_ptr[r] as usize..sub.row_ptr[r + 1] as usize]))
        .fold(0.0_f64, f64::max)
}
