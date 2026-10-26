//! Drag sessions (SK-05, note §3.5, ADR-007 near-singular decision).
//!
//! A session grabs one entity and moves *handle params* toward targets derived from the cursor:
//!
//! | grabbed | handles | target of a handle |
//! |---|---|---|
//! | point | its `x, y` | start + (cursor − grab) |
//! | line | both end points | start + (cursor − grab): a translation request |
//! | circle | the radius | `|cursor − centre|` + the grab offset (rim drag) |
//! | arc | the end point nearest to the grab | as a point |
//!
//! The structure and equation system are built once per session and every frame warm-starts
//! from the previous frame. Only the connected components that contain a handle are re-solved,
//! each as one block (a Dulmage–Mendelsohn block split would freeze a handle that sits in a
//! square block).
//!
//! - **Weighted minimum norm** (default frame): handles start at their targets with movement
//!   weight [`DRAG_WEIGHT`]; the damped minimum-norm Gauss–Newton then moves everything else
//!   first and needs no extra equations.
//! - **Feasible SQP** (fallback): minimise `½‖handles − targets‖² + ½μ‖Δx‖²` subject to
//!   `F(x) = 0` starting from the previous (feasible) frame. Each step is the KKT step of the
//!   same Gram system as the minimum-norm step, followed by a minimum-norm restoration and step
//!   halving, so the iterate stays on the constraint manifold and cannot jump branches.
//!
//! In [`DragMode::Auto`] a frame falls back to SQP when minimum norm does not converge, or moves
//! the non-dragged params more than [`JUMP_RATIO`] times the cursor step (a branch jump: near a
//! singular configuration the minimum-norm solve restarts from the cursor and lands anywhere on
//! the singular set, SK-13 finding 4).

use std::sync::Arc;

use amg_core::Point2;

use super::block::{
    BlockCtx, BlockOutcome, eval, j_mul_csr, jacobian, min_norm_step, scatter, solve_block,
};
use super::structure::{Component, Slot, Structure};
use super::{NativeSolver, Workspace, effective_tol};
use crate::error::SketchError;
use crate::ids::ParamId;
use crate::model::{EntityKind, Sketch};
use crate::solve::api::{
    Algorithm, DragMode, DragSession, DragSpec, DragStats, DragStep, SolveOptions, SolveReport,
    SolveStatus, SolverBackend,
};
use crate::solve::residual::SketchSystem;

/// Movement cost of the handle params in the minimum-norm step (note §3.5).
pub(crate) const DRAG_WEIGHT: f64 = 1e4;
/// SQP movement regularisation `μ` of the non-handle params (`1 / DRAG_WEIGHT`).
const SQP_MU: f64 = 1e-4;
/// Accepted Gauss–Newton steps per block in a drag frame: a warm frame needs a few, and a
/// frame that needs more is handed to the SQP fallback.
const DRAG_MAX_ITERATIONS: u32 = 12;
/// SQP outer iterations per frame.
const SQP_ITERS: u32 = 16;
/// Step halvings per SQP iteration.
const SQP_HALVINGS: u32 = 6;
/// A minimum-norm frame that moves a non-handle param by more than this many times the cursor
/// step is treated as a branch jump and redone by SQP.
const JUMP_RATIO: f64 = 3.0;
/// Movement allowed on top of the ratio, in units of the effective tolerance.
const JUMP_SLACK: f64 = 1e3;

/// What the cursor moves.
#[derive(Clone, Debug)]
enum Shape {
    /// Points translated by the cursor delta; `start` are their grab-time positions.
    Translate {
        grab: Point2,
        points: Vec<[ParamId; 2]>,
        start: Vec<Point2>,
    },
    /// Circle rim: the radius follows the cursor's distance from the centre.
    Radius {
        center: [ParamId; 2],
        radius: ParamId,
        offset: f64,
    },
}

/// Where a handle param lives in the cached structure.
#[derive(Copy, Clone, Debug)]
enum Place {
    /// In an equation class of connected component `comp`.
    Class { class: u32, comp: usize },
    /// In no equation: follows its target; `class` members (if aliased) follow too.
    Free { class: Option<u32> },
    /// A constant of the system (fixed param): cannot be dragged.
    Pinned,
}

#[derive(Copy, Clone, Debug)]
struct Handle {
    param: ParamId,
    place: Place,
}

/// A drag of one entity with the native solver.
///
/// Created by [`NativeSolver::begin_drag`] (boxed as [`DragSession`]) or
/// [`NativeSolver::drag`] (concrete, with [`stats`](NativeDrag::stats)). The sketch's structure
/// and constraint values must not change during the drag.
pub struct NativeDrag {
    solver: NativeSolver,
    sys: SketchSystem,
    st: Arc<Structure>,
    mode: DragMode,
    shape: Shape,
    handles: Vec<Handle>,
    /// Handle flags per param (movement weight [`DRAG_WEIGHT`]).
    is_handle: Vec<bool>,
    /// Components that contain a handle, ascending.
    comps: Vec<usize>,
    tol: f64,
    /// Handle targets of the previous frame (for the cursor step).
    last_targets: Vec<f64>,
    ws: Workspace,
    x: Vec<f64>,
    stats: DragStats,
}

impl std::fmt::Debug for NativeDrag {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NativeDrag")
            .field("mode", &self.mode)
            .field("handles", &self.handles.len())
            .field("components", &self.comps)
            .field("stats", &self.stats)
            .finish_non_exhaustive()
    }
}

impl NativeSolver {
    /// Starts dragging with the concrete session type (see [`NativeDrag`]).
    ///
    /// `Err(UnknownEntity)` if the entity does not exist; `Err(NotApplicable)` for entities
    /// that cannot be dragged (ellipses, splines, ...) and for a sketch whose equations cannot
    /// be built.
    pub fn drag(&mut self, sketch: &Sketch, spec: DragSpec) -> Result<NativeDrag, SketchError> {
        let ent = sketch
            .entity(spec.entity)
            .ok_or(SketchError::UnknownEntity(spec.entity))?;
        let point_of = |id| -> Result<[ParamId; 2], SketchError> {
            match sketch.entity(id).map(|e| &e.kind) {
                Some(EntityKind::Point { x, y }) => Ok([*x, *y]),
                _ => Err(SketchError::UnknownEntity(id)),
            }
        };
        let pos =
            |ps: [ParamId; 2]| Point2::new(sketch.params().get(ps[0]), sketch.params().get(ps[1]));
        let shape = match ent.kind.clone() {
            EntityKind::Point { x, y } => Shape::Translate {
                grab: spec.grab,
                points: vec![[x, y]],
                start: vec![pos([x, y])],
            },
            EntityKind::Line { start, end } => {
                let pts = [point_of(start)?, point_of(end)?];
                Shape::Translate {
                    grab: spec.grab,
                    start: pts.iter().map(|&p| pos(p)).collect(),
                    points: pts.to_vec(),
                }
            }
            EntityKind::Circle { center, radius } => {
                let c = point_of(center)?;
                let r = sketch.params().get(radius);
                Shape::Radius {
                    center: c,
                    radius,
                    offset: r - spec.grab.distance(pos(c)),
                }
            }
            EntityKind::Arc { start, end, .. } => {
                let (s, e) = (point_of(start)?, point_of(end)?);
                let nearest =
                    if pos(s).distance_squared(spec.grab) <= pos(e).distance_squared(spec.grab) {
                        s
                    } else {
                        e
                    };
                Shape::Translate {
                    grab: spec.grab,
                    points: vec![nearest],
                    start: vec![pos(nearest)],
                }
            }
            _ => {
                return Err(SketchError::NotApplicable(
                    "only points, lines, circles and arcs can be dragged",
                ));
            }
        };
        let Some(Ok((sys, st))) = self.prepare(sketch, &|| false) else {
            return Err(SketchError::NotApplicable(
                "the sketch's equations cannot be built",
            ));
        };
        let params: Vec<ParamId> = match &shape {
            Shape::Translate { points, .. } => points.iter().flatten().copied().collect(),
            Shape::Radius { radius, .. } => vec![*radius],
        };
        let mut class_comp = vec![usize::MAX; st.unknowns()];
        for (ci, c) in st.components.iter().enumerate() {
            for &k in &c.unknowns {
                class_comp[k as usize] = ci;
            }
        }
        let handles: Vec<Handle> = params
            .iter()
            .map(|&param| {
                let place = match st.slot.get(param.index()).copied() {
                    Some(Slot::Unknown(k)) if class_comp[k as usize] != usize::MAX => {
                        Place::Class {
                            class: k,
                            comp: class_comp[k as usize],
                        }
                    }
                    Some(Slot::Unknown(k)) => Place::Free { class: Some(k) },
                    Some(Slot::Fixed { .. }) => Place::Pinned,
                    Some(Slot::Unused) | None => Place::Free { class: None },
                };
                Handle { param, place }
            })
            .collect();
        let mut comps: Vec<usize> = handles
            .iter()
            .filter_map(|h| match h.place {
                Place::Class { comp, .. } => Some(comp),
                _ => None,
            })
            .collect();
        comps.sort_unstable();
        comps.dedup();
        let mut is_handle = vec![false; sketch.params().len()];
        for h in &handles {
            is_handle[h.param.index()] = true;
        }
        let tol = effective_tol(SolveOptions::fast().tolerance, sys.length_scale);
        let mut d = NativeDrag {
            solver: self.clone(),
            sys,
            st,
            mode: self.drag_mode,
            shape,
            handles,
            is_handle,
            comps,
            tol,
            last_targets: Vec::new(),
            ws: Workspace::default(),
            x: Vec::new(),
            stats: DragStats::default(),
        };
        // The "previous cursor" of the first frame is the grab itself.
        let x0 = sketch.params().as_slice();
        d.last_targets = d.targets(spec.grab, x0);
        Ok(d)
    }
}

impl NativeDrag {
    /// Counters since the session began.
    pub fn stats(&self) -> DragStats {
        self.stats
    }

    /// Target value of each handle for a cursor position (`x` = current params).
    fn targets(&self, cursor: Point2, x: &[f64]) -> Vec<f64> {
        match &self.shape {
            Shape::Translate {
                grab,
                points,
                start,
            } => {
                let d = cursor - *grab;
                points
                    .iter()
                    .zip(start)
                    .flat_map(|(_, s)| [s.x + d.x, s.y + d.y])
                    .collect()
            }
            Shape::Radius { center, offset, .. } => {
                let c = Point2::new(x[center[0].index()], x[center[1].index()]);
                vec![(cursor.distance(c) + offset).max(0.0)]
            }
        }
    }

    fn ctx<'a>(
        sys: &'a SketchSystem,
        st: &'a Structure,
        tol: f64,
        cancel: &'a dyn Fn() -> bool,
    ) -> BlockCtx<'a> {
        BlockCtx {
            sys: &sys.system,
            st,
            tol,
            length_scale: sys.length_scale,
            algorithm: Algorithm::Auto,
            max_iterations: DRAG_MAX_ITERATIONS,
            cancel,
        }
    }

    /// Largest residual of the touched components at `x`.
    fn residual(&mut self, x: &[f64]) -> f64 {
        let mut worst = 0.0_f64;
        let mut f = Vec::new();
        for &ci in &self.comps {
            let sub = self.st.components[ci].whole();
            f.clear();
            f.resize(sub.rows, 0.0);
            eval(&self.sys.system, sub, x, &mut f, &mut self.ws);
            worst = f.iter().fold(worst, |m, v| {
                if v.is_nan() {
                    f64::INFINITY
                } else {
                    m.max(v.abs())
                }
            });
        }
        worst
    }

    /// Weighted minimum-norm frame on `x` (current params with the handle targets already
    /// written). `None` if cancelled.
    fn min_norm_frame(&mut self, x: &mut [f64], cancel: &dyn Fn() -> bool) -> Option<u32> {
        let st = self.st.clone();
        let mut iterations = 0;
        let mut u = vec![0.0; st.unknowns()];
        let mut w = vec![0.0; st.unknowns()];
        for &ci in &self.comps {
            for &k in &st.components[ci].unknowns {
                let (mut sw, mut sx) = (0.0, 0.0);
                for &p in &st.members[k as usize] {
                    let wp = if self.is_handle[p.index()] {
                        DRAG_WEIGHT
                    } else {
                        1.0
                    };
                    sw += wp;
                    sx += wp * x[p.index()];
                }
                u[k as usize] = sx / sw;
                w[k as usize] = sw;
                for &p in &st.members[k as usize] {
                    x[p.index()] = u[k as usize];
                }
            }
        }
        let ctx = Self::ctx(&self.sys, &st, self.tol, cancel);
        let mut ws = std::mem::take(&mut self.ws);
        for &ci in &self.comps {
            let sub = st.components[ci].whole();
            let mut ul: Vec<f64> = sub.unknowns.iter().map(|&k| u[k as usize]).collect();
            let wl: Vec<f64> = sub.unknowns.iter().map(|&k| w[k as usize]).collect();
            match solve_block(&ctx, sub, x, &mut ul, &wl, &mut ws) {
                BlockOutcome::Cancelled => {
                    self.ws = ws;
                    return None;
                }
                BlockOutcome::Done(it) => iterations += it,
            }
        }
        self.ws = ws;
        Some(iterations)
    }

    /// Feasible SQP frame from the feasible `x` toward `targets` (written into `x`).
    /// Returns `(iterations, feasible)`; `None` if cancelled.
    fn sqp_frame(
        &mut self,
        x: &mut [f64],
        targets: &[f64],
        cancel: &dyn Fn() -> bool,
    ) -> Option<(u32, bool)> {
        let st = self.st.clone();
        let mut iterations = 0;
        let mut feasible = true;
        let mut ws = std::mem::take(&mut self.ws);
        let ctx = Self::ctx(&self.sys, &st, self.tol, cancel);
        for &ci in &self.comps {
            // Handle classes of this component with their targets (a class shared by several
            // handles gets their mean).
            let mut refs: Vec<(u32, f64, u32)> = Vec::new();
            for (h, &t) in self.handles.iter().zip(targets) {
                if let Place::Class { class, comp } = h.place
                    && comp == ci
                {
                    match refs.iter_mut().find(|r| r.0 == class) {
                        Some(r) => {
                            r.1 += t;
                            r.2 += 1;
                        }
                        None => refs.push((class, t, 1)),
                    }
                }
            }
            for r in &mut refs {
                r.1 /= f64::from(r.2);
            }
            match sqp_component(&ctx, &st.components[ci], x, &refs, &mut ws) {
                None => {
                    self.ws = ws;
                    return None;
                }
                Some((it, ok)) => {
                    iterations += it;
                    feasible &= ok;
                }
            }
        }
        self.ws = ws;
        Some((iterations, feasible))
    }

    /// Writes the solved params of the touched components and the free handles to `sketch`.
    fn write_back(&self, sketch: &mut Sketch, x: &[f64], targets: &[f64]) {
        let params = sketch.params_mut();
        for &ci in &self.comps {
            for &k in &self.st.components[ci].unknowns {
                for &p in &self.st.members[k as usize] {
                    params.set(p, x[p.index()]);
                }
            }
        }
        for (h, &t) in self.handles.iter().zip(targets) {
            if let Place::Free { class } = h.place {
                params.set(h.param, t);
                if let Some(k) = class {
                    for &p in &self.st.members[k as usize] {
                        params.set(p, t);
                    }
                }
            }
        }
    }

    fn cancelled() -> DragStep {
        DragStep {
            status: SolveStatus::Cancelled,
            iterations: 0,
            max_residual: f64::NAN,
        }
    }
}

impl DragSession for NativeDrag {
    fn step(&mut self, sketch: &mut Sketch, target: Point2, cancel: &dyn Fn() -> bool) -> DragStep {
        if cancel() {
            return Self::cancelled();
        }
        let mut prev = std::mem::take(&mut self.x);
        prev.clear();
        prev.extend_from_slice(sketch.params().as_slice());
        let targets = self.targets(target, &prev);
        // Cursor step in handle space and the largest allowed non-handle movement.
        let cursor_step = targets
            .iter()
            .zip(&self.last_targets)
            .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()));
        self.last_targets.clone_from(&targets);
        let allowed = JUMP_RATIO * cursor_step + JUMP_SLACK * self.tol;
        self.stats.frames += 1;

        let mut iterations = 0;
        // Minimum-norm frame: (params, residual, branch jump?).
        let mut min_norm: Option<(Vec<f64>, f64, bool)> = None;
        if self.mode != DragMode::Sqp {
            let mut x = prev.clone();
            for (h, &t) in self.handles.iter().zip(&targets) {
                if matches!(h.place, Place::Class { .. }) {
                    x[h.param.index()] = t;
                }
            }
            let Some(it) = self.min_norm_frame(&mut x, cancel) else {
                self.x = prev;
                return Self::cancelled();
            };
            iterations += it;
            let res = self.residual(&x);
            let jump = self.moved(&prev, &x) > allowed;
            min_norm = Some((x, res, jump));
        }
        let take_min_norm = match &min_norm {
            Some((_, res, jump)) => self.mode == DragMode::MinNorm || (*res <= self.tol && !*jump),
            None => false,
        };
        let mut chosen = None;
        if take_min_norm {
            chosen = min_norm.take().map(|(x, res, _)| (x, res));
        } else {
            self.stats.sqp_frames += 1;
            let mut x = prev.clone();
            let Some((it, _)) = self.sqp_frame(&mut x, &targets, cancel) else {
                self.x = prev;
                return Self::cancelled();
            };
            iterations += it;
            let res = self.residual(&x);
            if res <= self.tol {
                chosen = Some((x, res));
            } else if let Some((x, res, _)) = min_norm
                && res <= self.tol
            {
                // SQP could not start from a feasible frame: keep the converged jump.
                chosen = Some((x, res));
            }
        }
        let Some((x, res)) = chosen else {
            // Nothing converged: keep the last good state.
            self.stats.failed_frames += 1;
            let res = self.residual(&prev);
            self.x = prev;
            return DragStep {
                status: SolveStatus::DidNotConverge,
                iterations,
                max_residual: res,
            };
        };
        if res > self.tol {
            self.stats.failed_frames += 1;
        }
        self.write_back(sketch, &x, &targets);
        self.x = x;
        DragStep {
            status: if res <= self.tol {
                SolveStatus::Converged
            } else {
                SolveStatus::DidNotConverge
            },
            iterations,
            max_residual: res,
        }
    }

    fn finish(mut self: Box<Self>, sketch: &mut Sketch) -> SolveReport {
        self.solver
            .solve(sketch, &SolveOptions::default(), &|| false)
    }
}

impl NativeDrag {
    /// Largest change of a param (handles included: a constrained handle that jumps is a branch flip too) of the touched components between `a` and `b`.
    fn moved(&self, a: &[f64], b: &[f64]) -> f64 {
        let mut m = 0.0_f64;
        for &ci in &self.comps {
            for &k in &self.st.components[ci].unknowns {
                for &p in &self.st.members[k as usize] {
                    let d = (a[p.index()] - b[p.index()]).abs();
                    m = m.max(if d.is_nan() { f64::INFINITY } else { d });
                }
            }
        }
        m
    }
}

fn max_abs(v: &[f64]) -> f64 {
    v.iter().fold(0.0_f64, |m, x| m.max(x.abs()))
}

/// Feasible SQP for one component (see the module docs). `refs` are `(class, target, _)` of the
/// handle classes. Returns `(linear solves, feasible at the end)`; `None` if cancelled. `x` is
/// only changed to feasible, objective-decreasing iterates.
fn sqp_component(
    ctx: &BlockCtx<'_>,
    comp: &Component,
    x: &mut [f64],
    refs: &[(u32, f64, u32)],
    ws: &mut Workspace,
) -> Option<(u32, bool)> {
    let sub = comp.whole();
    let (m, n) = (sub.rows, sub.n());
    let read = |x: &[f64]| -> Vec<f64> {
        sub.unknowns
            .iter()
            .map(|&k| x[ctx.st.members[k as usize][0].index()])
            .collect()
    };
    let mut u = read(x);
    let mut reference = u.clone();
    let mut cost = vec![SQP_MU; n];
    for &(class, t, _) in refs {
        if let Ok(i) = sub.unknowns.binary_search(&class) {
            reference[i] = t;
            cost[i] = 1.0;
        }
    }
    let sqrt_cost: Vec<f64> = cost.iter().map(|c| c.sqrt()).collect();
    let phi = |u: &[f64]| -> f64 {
        (0..n)
            .map(|i| 0.5 * cost[i] * (u[i] - reference[i]).powi(2))
            .sum()
    };
    let mut iterations = 0;
    let mut f = vec![0.0; m];
    // Start on the constraint manifold (a no-op from a converged frame).
    match solve_block(ctx, sub, x, &mut u, &cost, ws) {
        BlockOutcome::Cancelled => return None,
        BlockOutcome::Done(it) => iterations += it,
    }
    eval(ctx.sys, sub, x, &mut f, ws);
    if max_abs(&f) > ctx.tol {
        return Some((iterations, false));
    }
    let mut js = vec![0.0; sub.nnz()];
    let step_cap = ctx.length_scale.max(1.0);
    let mut xt = x.to_vec();
    for _ in 0..SQP_ITERS {
        if (ctx.cancel)() {
            return None;
        }
        jacobian(ctx.sys, sub, x, &mut js, ws);
        for (v, c) in js.iter_mut().zip(&sub.col_idx) {
            *v /= sqrt_cost[*c as usize];
        }
        eval(ctx.sys, sub, x, &mut f, ws);
        // dv = −gs − Jsᵀ λ with Js Jsᵀ λ = F − Js gs, in v = H^{1/2} u.
        let gs: Vec<f64> = (0..n)
            .map(|i| sqrt_cost[i] * (u[i] - reference[i]))
            .collect();
        let jg = j_mul_csr(sub, &js, &gs);
        let rhs: Vec<f64> = f.iter().zip(&jg).map(|(a, b)| a - b).collect();
        let proj = min_norm_step(sub, &js, &rhs, 0.0);
        iterations += 1;
        let mut du: Vec<f64> = (0..n).map(|i| (proj[i] - gs[i]) / sqrt_cost[i]).collect();
        let big = max_abs(&du);
        if big > step_cap {
            du.iter_mut().for_each(|d| *d *= step_cap / big);
        }
        let scale = u.iter().fold(1.0_f64, |a, v| a.max(v.abs()));
        if max_abs(&du) <= 1e-10 * scale {
            break;
        }
        let phi0 = phi(&u);
        let mut t = 1.0;
        let mut accepted = false;
        for _ in 0..SQP_HALVINGS {
            let mut trial: Vec<f64> = u.iter().zip(&du).map(|(a, d)| a + t * d).collect();
            xt.copy_from_slice(x);
            scatter(ctx.st, sub, &trial, &mut xt);
            let pre = trial.clone();
            match solve_block(ctx, sub, &mut xt, &mut trial, &cost, ws) {
                BlockOutcome::Cancelled => return None,
                BlockOutcome::Done(it) => iterations += it,
            }
            eval(ctx.sys, sub, &xt, &mut f, ws);
            // Continuation: on a smooth manifold the restoration moves O(step²); a restoration
            // that moves a large part of the step landed on another branch (near a fold or
            // toggle) and is rejected, so the iterate cannot jump.
            let rest = trial
                .iter()
                .zip(&pre)
                .fold(0.0_f64, |m, (a, b)| m.max((a - b).abs()));
            let reach = max_abs(&du) * t;
            if max_abs(&f) <= ctx.tol && phi(&trial) < phi0 && rest <= 0.5 * reach + 1e-12 * scale {
                u = trial;
                x.copy_from_slice(&xt);
                accepted = true;
                break;
            }
            t *= 0.5;
        }
        if !accepted || (phi0 - phi(&u)) <= 1e-14 * phi0.max(1e-300) {
            break;
        }
    }
    Some((iterations, true))
}
