//! Prototype drag session (note §3.5): the structure is planned once per drag, every frame
//! warm-starts from the previous one.
//!
//! - [`DragMethod::WeightedMinNorm`] (default, SolveSpace-style): the dragged point is put at
//!   the cursor and gets a large movement cost `W`; the weighted min-norm Gauss–Newton then
//!   moves everything else first.
//! - [`DragMethod::Sqp`] (PlaneGCS-style): minimise `½‖p − cursor‖² + ½μ‖x_others − x_prev‖²`
//!   subject to `F(x) = 0` with Gauss–Newton SQP steps (KKT solved through `J H⁻¹ Jᵀ`).

use amg_core::Point2;

use super::linalg::{chol_semidef, chol_solve, gram, j_mul, jt_mul, max_abs};
use super::plan::{Component, Plan, Slot};
use super::{PIVOT_REL, ProtoOptions, ProtoStatus, solve_component, write_back};
use crate::error::SketchError;
use crate::ids::{EntityId, ParamId};
use crate::model::{EntityKind, Sketch};

/// Movement cost of the dragged params in the weighted min-norm step.
const DRAG_WEIGHT: f64 = 1e4;
/// SQP regularisation `μ` on the non-dragged params.
const SQP_MU: f64 = 1e-4;
/// SQP iterations per frame.
const SQP_ITERS: u32 = 30;

/// How a drag frame is solved.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DragMethod {
    /// Cursor as start value + heavy weight in the min-norm step.
    WeightedMinNorm,
    /// Cursor as objective, constraints as equalities (SQP).
    Sqp,
}

/// Result of one drag frame.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct DragFrame {
    /// `Converged` when all equations hold after the frame.
    pub status: ProtoStatus,
    /// Linear solves used.
    pub iterations: u32,
    /// Largest residual after the frame (mm).
    pub max_residual: f64,
}

/// A drag of one point.
pub struct ProtoDrag {
    plan: Plan,
    method: DragMethod,
    params: [ParamId; 2],
    /// Components containing a dragged class.
    comps: Vec<usize>,
    opts: ProtoOptions,
}

impl std::fmt::Debug for ProtoDrag {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ProtoDrag")
            .field("method", &self.method)
            .field("params", &self.params)
            .field("comps", &self.comps)
            .finish_non_exhaustive()
    }
}

impl ProtoDrag {
    /// Starts dragging point `point`. The sketch structure must not change during the drag.
    pub fn begin(
        sketch: &Sketch,
        point: EntityId,
        method: DragMethod,
        opts: &ProtoOptions,
    ) -> Result<ProtoDrag, SketchError> {
        let Some(EntityKind::Point { x, y }) = sketch.entity(point).map(|e| e.kind.clone()) else {
            return Err(SketchError::NotApplicable("drag target is not a point"));
        };
        let plan = Plan::new(sketch, opts)?;
        let classes: Vec<u32> = [x, y]
            .iter()
            .filter_map(|p| match plan.slot.get(p.index()) {
                Some(Slot::Unknown(k)) => Some(*k),
                _ => None,
            })
            .collect();
        let comps = plan
            .components
            .iter()
            .enumerate()
            .filter(|(_, c)| c.unknowns.iter().any(|k| classes.contains(k)))
            .map(|(i, _)| i)
            .collect();
        Ok(ProtoDrag {
            plan,
            method,
            params: [x, y],
            comps,
            opts: ProtoOptions {
                analyze: false,
                diagnose: false,
                ..opts.clone()
            },
        })
    }

    /// Moves the point toward `target` and re-solves.
    pub fn step(&mut self, sketch: &mut Sketch, target: Point2) -> DragFrame {
        let plan = &self.plan;
        let [px, py] = self.params;
        let mut x = sketch.params().as_slice().to_vec();
        let mut iterations = 0;
        let unknown = |p: ParamId| matches!(plan.slot.get(p.index()), Some(Slot::Unknown(_)));
        match self.method {
            DragMethod::WeightedMinNorm => {
                let prev = x.clone();
                if unknown(px) {
                    x[px.index()] = target.x;
                }
                if unknown(py) {
                    x[py.index()] = target.y;
                }
                let start = x.clone();
                // 1. Heavy dragged params; 2. if that stalls (cursor out of reach, near a
                // toggle), unit weights from the same start; 3. else keep the last frame.
                let mut done = false;
                for weight in [DRAG_WEIGHT, 1.0] {
                    let wf = |p: ParamId| if p == px || p == py { weight } else { 1.0 };
                    let (u, w) = plan.gather(&start, &wf);
                    x = plan.expand(&start, &u);
                    for &ci in &self.comps {
                        let c = &plan.components[ci];
                        let mut uc: Vec<f64> = c.unknowns.iter().map(|&k| u[k as usize]).collect();
                        let wc: Vec<f64> = c.unknowns.iter().map(|&k| w[k as usize]).collect();
                        iterations +=
                            solve_component(plan, c, &mut x, &mut uc, &wc, &self.opts).iterations;
                    }
                    if plan.max_residual(&x) <= plan.tol {
                        done = true;
                        break;
                    }
                }
                if !done {
                    x = prev;
                }
            }
            DragMethod::Sqp => {
                let (mut u, _) = plan.gather(&x, &|_| 1.0);
                // A dragged class in no component is unconstrained: it follows the cursor.
                for (p, t) in [(px, target.x), (py, target.y)] {
                    if let Some(Slot::Unknown(k)) = plan.slot.get(p.index())
                        && !self
                            .comps
                            .iter()
                            .any(|&ci| plan.components[ci].unknowns.contains(k))
                    {
                        u[*k as usize] = t;
                    }
                }
                x = plan.expand(&x, &u);
                for &ci in &self.comps {
                    let c = &plan.components[ci];
                    iterations += sqp_component(plan, c, &mut x, [px, py], target, &self.opts);
                }
            }
        }
        write_back(plan, sketch, &x);
        let max_residual = plan.max_residual(&x);
        DragFrame {
            status: if max_residual <= plan.tol {
                ProtoStatus::Converged
            } else {
                ProtoStatus::DidNotConverge
            },
            iterations,
            max_residual,
        }
    }
}

/// Feasible SQP for one component: Gauss–Newton KKT steps on the objective, each followed by a
/// min-norm restoration onto `F = 0` and accepted only if the objective decreases (step
/// halving). Returns the linear solves used.
fn sqp_component(
    plan: &Plan,
    c: &Component,
    x: &mut [f64],
    dragged: [ParamId; 2],
    target: Point2,
    opts: &ProtoOptions,
) -> u32 {
    let (m, n) = (c.rows, c.unknowns.len());
    // Reference value and cost per local unknown.
    let mut reference = Vec::with_capacity(n);
    let mut cost = Vec::with_capacity(n);
    for &k in &c.unknowns {
        let ms = &plan.members[k as usize];
        if ms.contains(&dragged[0]) {
            reference.push(target.x);
            cost.push(1.0);
        } else if ms.contains(&dragged[1]) {
            reference.push(target.y);
            cost.push(1.0);
        } else {
            reference.push(x[ms[0].index()]);
            cost.push(SQP_MU);
        }
    }
    let phi = |u: &[f64]| -> f64 {
        (0..n)
            .map(|i| 0.5 * cost[i] * (u[i] - reference[i]).powi(2))
            .sum()
    };
    let ones = vec![1.0; n];
    let read = |x: &[f64]| -> Vec<f64> {
        c.unknowns
            .iter()
            .map(|&k| x[plan.members[k as usize][0].index()])
            .collect()
    };
    let mut it = 0;
    // Start on the constraint manifold.
    let mut uc = read(x);
    it += solve_component(plan, c, x, &mut uc, &ones, opts).iterations;
    let s: Vec<f64> = cost.iter().map(|h| 1.0 / f64::sqrt(*h)).collect();
    let mut f = vec![0.0; m];
    let mut j = vec![0.0; m * n];
    let step_cap = plan.sys.length_scale.max(1.0);
    let mut outer = 0;
    while outer < SQP_ITERS {
        outer += 1;
        it += 1;
        plan.eval(c, x, &mut f);
        plan.jacobian(c, x, &mut j);
        for r in 0..m {
            for (v, sk) in j[r * n..(r + 1) * n].iter_mut().zip(&s) {
                *v *= sk;
            }
        }
        // gs = H^{-1/2} ∇φ, ∇φ = H (u − ref).
        let gs: Vec<f64> = (0..n)
            .map(|i| s[i] * cost[i] * (uc[i] - reference[i]))
            .collect();
        let jg = j_mul(&j, m, n, &gs);
        let rhs: Vec<f64> = f.iter().zip(&jg).map(|(a, b)| a - b).collect();
        let mut a = gram(&j, m, n, 0.0);
        let dep = chol_semidef(&mut a, m, PIVOT_REL);
        let lam = chol_solve(&a, &dep, m, &rhs);
        let jl = jt_mul(&j, m, n, &lam);
        let mut du: Vec<f64> = (0..n).map(|i| -s[i] * (gs[i] + jl[i])).collect();
        let big = max_abs(&du);
        if big > step_cap {
            du.iter_mut().for_each(|d| *d *= step_cap / big);
        }
        let scale = uc.iter().fold(1.0_f64, |a, v| a.max(v.abs()));
        if max_abs(&du) <= 1e-10 * scale {
            break;
        }
        let phi0 = phi(&uc);
        let mut t = 1.0;
        let mut accepted = false;
        for _ in 0..8 {
            let mut trial: Vec<f64> = uc.iter().zip(&du).map(|(u, d)| u + t * d).collect();
            let mut xt = x.to_vec();
            plan.scatter(c, &trial, &mut xt);
            it += solve_component(plan, c, &mut xt, &mut trial, &ones, opts).iterations;
            plan.eval(c, &xt, &mut f);
            if max_abs(&f) <= plan.tol && phi(&trial) < phi0 {
                uc = trial;
                x.copy_from_slice(&xt);
                accepted = true;
                break;
            }
            t *= 0.5;
        }
        if !accepted || (phi0 - phi(&uc)) <= 1e-14 * phi0.max(1e-300) {
            break;
        }
    }
    it
}
