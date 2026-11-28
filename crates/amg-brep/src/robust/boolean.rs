//! `robust_boolean` / `robust_split` (design note §2.4): the exact attempt, the fuzzy ladder with
//! gluing for coincident faces, then the op on healed operands.

use amg_core::{Aabb3, Vec3};
use amg_jobs::JobCtx;

use super::chain::{Chain, Strategy, record_failure};
use super::heal::{compose_operands, heal_capped};
use super::{
    FUZZY_WARN_ABOVE, RetryPolicy, RobustCtx, RobustError, RobustOutput, StrategyId, Validation,
};
use crate::capture::{InputRef, KernelOp};
use crate::error::{KResult, KernelError};
use crate::geom::Surface;
use crate::history::OpResult;
use crate::kernel::BrepKernel;
use crate::opts::{BoolOp, BoolOpts, Glue};
use crate::shape::{Shape, SubKind, SubShape};

/// Relative slack of the boolean volume bounds (fixed-order volume quadrature is good to about
/// 1e-4 relative; design note §2.2(4)).
const VOLUME_SLACK: f64 = 1e-3;

#[derive(Clone, Copy)]
enum Op {
    Bool(BoolOp),
    Split,
}

impl Op {
    fn name(self) -> &'static str {
        match self {
            Op::Bool(_) => "boolean",
            Op::Split => "split",
        }
    }

    fn call(
        self,
        k: &dyn BrepKernel,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        match self {
            Op::Bool(op) => k.boolean(op, objects, tools, opts, ctx),
            Op::Split => k.split(objects, tools, opts, ctx),
        }
    }
}

/// [`BrepKernel::boolean`] through the strategy chain (S20 design §3.4), in this fixed order:
/// 1. `Direct`: `base` as given;
/// 2. `Fuzzy { rung }` for each `p.fuzzy_ladder[rung]` above `base.fuzzy` and at most
///    `min(p.tol_cap, 1e-3 × smallest input edge)`; from the second rung on, when `base.glue`
///    is off and the operands have coincident planar or cylindrical faces (within the rung),
///    also `Glue { rung }` (`Glue::Shift`);
/// 3. `Heal(level)` for `Light ..= p.heal`: every operand healed at that level (on copies), the
///    op with `base`, and the healing history composed in, so the history maps the caller's
///    operands.
///
/// Each result passes the gate: `p.validation` with `require_solid` when every operand is a
/// solid body, a tolerance cap of `max(input tolerance, min(cap, 10 × fuzzy))` and, for solid
/// operands, volume bounds (`V(∪) ≤ ΣV`, `V(A \ B) ≥ V(A) − V(B)`, `V(∩) ≤ min`, …) and
/// [`Validation::disjoint_solids`] (an unmerged fuse returns its overlapping operands). Every
/// non-direct acceptance adds a report warning; rungs above 1e-5 mm add a second one.
/// OCCT's non-destructive mode is always on (S02 `boolean`): the inputs are never modified.
///
/// # Errors
/// [`RobustError`]: `Timeout` when an attempt's deadline expires (never a later strategy's
/// result), `Cancelled`, the class of an argument error (no retry), or the direct attempt's
/// class when every strategy failed. With `cx.recorder` the failure is dumped (not for
/// `Cancelled`).
pub fn robust_boolean(
    k: &dyn BrepKernel,
    op: BoolOp,
    objects: &[Shape],
    tools: &[Shape],
    base: &BoolOpts,
    p: &RetryPolicy,
    cx: &RobustCtx<'_>,
) -> Result<RobustOutput, RobustError> {
    run(k, Op::Bool(op), objects, tools, base, p, cx)
}

/// [`BrepKernel::split`] through the same chain as [`robust_boolean`].
///
/// # Errors
/// As [`robust_boolean`].
pub fn robust_split(
    k: &dyn BrepKernel,
    objects: &[Shape],
    tools: &[Shape],
    base: &BoolOpts,
    p: &RetryPolicy,
    cx: &RobustCtx<'_>,
) -> Result<RobustOutput, RobustError> {
    run(k, Op::Split, objects, tools, base, p, cx)
}

fn run(
    k: &dyn BrepKernel,
    op: Op,
    objects: &[Shape],
    tools: &[Shape],
    base: &BoolOpts,
    p: &RetryPolicy,
    cx: &RobustCtx<'_>,
) -> Result<RobustOutput, RobustError> {
    let gate = Gate::new(op, objects, tools, p);
    let chain = Chain::new(k, op.name(), p, cx);

    let direct = Strategy::new(
        StrategyId::Direct,
        gate.validation(base.fuzzy),
        "the exact op",
        move |ctx| op.call(k, objects, tools, base, ctx),
    );
    // Built only when the direct attempt did not succeed (the edge scan costs).
    let rest = std::iter::once_with(|| later_strategies(k, op, objects, tools, base, p, cx, &gate))
        .flatten();
    let result = chain.run(std::iter::once(direct).chain(rest));
    if let Err(e) = &result {
        let inputs: Vec<&Shape> = objects.iter().chain(tools).collect();
        record_failure(
            cx,
            &dump_op(op, objects.len(), tools.len(), base),
            &inputs,
            e,
        );
    }
    result
}

#[allow(clippy::too_many_arguments, reason = "the chain's whole context")]
fn later_strategies<'s>(
    k: &'s dyn BrepKernel,
    op: Op,
    objects: &'s [Shape],
    tools: &'s [Shape],
    base: &'s BoolOpts,
    p: &'s RetryPolicy,
    cx: &'s RobustCtx<'_>,
    gate: &Gate,
) -> Vec<Strategy<'s>> {
    let mut out = Vec::new();
    if !p.fuzzy_ladder.is_empty() {
        let cap = ladder_cap(k, objects, tools, p.tol_cap);
        for (i, &rung) in p.fuzzy_ladder.iter().enumerate() {
            let Ok(index) = u8::try_from(i) else { break };
            if !(rung > base.fuzzy && rung <= cap) {
                continue;
            }
            let warnings = if rung > FUZZY_WARN_ABOVE {
                vec![format!(
                    "{} built with {rung:e} mm fuzzy tolerance (above {FUZZY_WARN_ABOVE:e} mm)",
                    op.name()
                )]
            } else {
                Vec::new()
            };
            let fuzzy = BoolOpts {
                fuzzy: rung,
                ..base.clone()
            };
            let mut s = Strategy::new(
                StrategyId::Fuzzy { rung: index },
                gate.validation(rung),
                format!("fuzzy {rung:e} mm"),
                move |ctx| op.call(k, objects, tools, &fuzzy, ctx),
            );
            s.warnings.clone_from(&warnings);
            out.push(s);
            if i >= 1 && base.glue == Glue::Off && coincident(objects, tools, rung) {
                let glued = BoolOpts {
                    fuzzy: rung,
                    glue: Glue::Shift,
                    ..base.clone()
                };
                let mut s = Strategy::new(
                    StrategyId::Glue { rung: index },
                    gate.validation(rung),
                    format!("fuzzy {rung:e} mm and glue shift"),
                    move |ctx| op.call(k, objects, tools, &glued, ctx),
                );
                s.warnings = warnings;
                out.push(s);
            }
        }
    }
    for level in p.heal.ladder() {
        let tol = cx.tol;
        let cap = p.tol_cap;
        out.push(Strategy::new(
            StrategyId::Heal(level),
            gate.validation(base.fuzzy),
            format!("operands healed at level {level:?}"),
            move |ctx| {
                let hcx = RobustCtx {
                    job: ctx,
                    recorder: None,
                    tol,
                };
                let mut healed = Vec::with_capacity(objects.len() + tools.len());
                for s in objects.iter().chain(tools) {
                    let h = heal_capped(k, s, level, cap, &hcx).map_err(|e| match e.error {
                        Some(err) => *err,
                        None => KernelError::OpFailed {
                            op: "heal",
                            message: e.message,
                            offending: Vec::new(),
                        },
                    })?;
                    healed.push(h.result);
                }
                let (hobj, htools) = healed.split_at(objects.len());
                let shapes = |r: &[OpResult]| r.iter().map(|r| r.shape.clone()).collect::<Vec<_>>();
                let r = op.call(k, &shapes(hobj), &shapes(htools), base, ctx)?;
                let pre: Vec<_> = healed.iter().map(|h| h.history.clone()).collect();
                let mut history = compose_operands(&pre, &r.history);
                history.order_splits(&r.shape)?;
                let mut warnings: Vec<_> = healed.into_iter().flat_map(|h| h.warnings).collect();
                warnings.extend(r.warnings);
                Ok(OpResult {
                    shape: r.shape,
                    history,
                    warnings,
                })
            },
        ));
    }
    out
}

/// The capture form of the op (inputs numbered objects first, then tools).
fn dump_op(op: Op, objects: usize, tools: usize, base: &BoolOpts) -> KernelOp {
    let refs = |from: usize, n: usize| -> Vec<InputRef> {
        (from..from + n)
            .map(|i| InputRef(u32::try_from(i).unwrap_or(u32::MAX)))
            .collect()
    };
    match op {
        Op::Bool(kind) => KernelOp::Boolean {
            kind,
            objects: refs(0, objects),
            tools: refs(objects, tools),
            opts: base.clone(),
        },
        Op::Split => KernelOp::Split {
            objects: refs(0, objects),
            tools: refs(objects, tools),
            opts: base.clone(),
        },
    }
}

/// `min(tol_cap, 1e-3 × the shortest non-degenerated input edge)`.
fn ladder_cap(k: &dyn BrepKernel, objects: &[Shape], tools: &[Shape], tol_cap: f64) -> f64 {
    let mut shortest = f64::INFINITY;
    for s in objects.iter().chain(tools) {
        for e in s.edges() {
            let sub = SubShape {
                kind: SubKind::Edge,
                index: e.0,
            };
            if let Ok(g) = k.sub_geom(s, sub)
                && g.measure > 0.0
            {
                shortest = shortest.min(g.measure);
            }
        }
    }
    tol_cap.min(1e-3 * shortest)
}

/// What the gate needs to know about the operands.
struct Gate {
    validation: Validation,
    solid_operands: bool,
    input_tol: f64,
    /// `(lo, hi)` before slack, for solid operands.
    volume: Option<(f64, f64)>,
    /// Total face area of the operands, mm² (fuzzy slack of the volume bounds); computed for
    /// the first fuzzy attempt only, so the direct path does not pay for it.
    area: std::cell::OnceCell<f64>,
    operands: Vec<Shape>,
}

impl Gate {
    fn new(op: Op, objects: &[Shape], tools: &[Shape], p: &RetryPolicy) -> Gate {
        let all = || objects.iter().chain(tools);
        let solid_operands = all().all(solid_body);
        let input_tol = all().map(Shape::max_tolerance).fold(0.0, f64::max);
        let volume = if solid_operands {
            volume_bounds(op, objects, tools)
        } else {
            None
        };
        Gate {
            validation: p.validation.clone(),
            solid_operands,
            input_tol,
            volume,
            area: std::cell::OnceCell::new(),
            operands: all().cloned().collect(),
        }
    }

    /// The gate of an attempt with fuzzy value `fuzzy`.
    fn validation(&self, fuzzy: f64) -> Validation {
        let mut v = self.validation.clone();
        v.require_solid |= self.solid_operands;
        v.disjoint_solids |= self.solid_operands;
        let cap = if fuzzy > 0.0 {
            v.max_tolerance.min(10.0 * fuzzy)
        } else {
            v.max_tolerance
        };
        v.max_tolerance = cap.max(self.input_tol);
        if v.volume_bounds.is_none() {
            v.volume_bounds = self.volume.map(|(lo, hi)| {
                let area = if fuzzy > 0.0 {
                    *self.area.get_or_init(|| {
                        self.operands
                            .iter()
                            .map(|s| s.faces().map(|f| s.face_area(f)).sum::<f64>())
                            .sum()
                    })
                } else {
                    0.0
                };
                let abs = 2.0 * fuzzy * area + 1e-9;
                (
                    (lo * (1.0 - VOLUME_SLACK) - abs).max(0.0),
                    hi * (1.0 + VOLUME_SLACK) + abs,
                )
            });
        }
        v
    }
}

/// Whether `s` has solids and every face of it is in one.
fn solid_body(s: &Shape) -> bool {
    let solids = s.solids();
    if solids.is_empty() {
        return false;
    }
    let in_solids: u32 = solids.iter().map(|x| x.count(SubKind::Face)).sum();
    in_solids >= s.count(SubKind::Face)
}

/// Volume bounds of the result from the operands' volumes (design note §2.2(4)).
fn volume_bounds(op: Op, objects: &[Shape], tools: &[Shape]) -> Option<(f64, f64)> {
    let vol = |s: &Shape| s.volume().ok().filter(|v| v.is_finite() && *v > 0.0);
    let vo: Vec<f64> = objects.iter().map(vol).collect::<Option<_>>()?;
    let vt: Vec<f64> = tools.iter().map(vol).collect::<Option<_>>()?;
    let sum = |v: &[f64]| v.iter().sum::<f64>();
    let max = |v: &[f64]| v.iter().copied().fold(0.0, f64::max);
    Some(match op {
        Op::Bool(BoolOp::Fuse) => (max(&vo).max(max(&vt)), sum(&vo) + sum(&vt)),
        Op::Bool(BoolOp::Cut) => {
            let lo = if vo.len() == 1 {
                (vo[0] - sum(&vt)).max(0.0)
            } else {
                0.0
            };
            (lo, sum(&vo))
        }
        Op::Bool(BoolOp::Common) => (0.0, sum(&vo).min(sum(&vt))),
        Op::Split => (max(&vo), sum(&vo)),
    })
}

/// A face's surface, reduced to what coincidence needs.
enum Carrier {
    Plane {
        origin: Vec3,
        normal: Vec3,
    },
    Cylinder {
        origin: Vec3,
        axis: Vec3,
        radius: f64,
    },
}

fn carriers(s: &Shape) -> Vec<(Carrier, Aabb3)> {
    s.faces()
        .filter_map(|f| {
            let carrier = match s.surface(f) {
                Surface::Plane(p) => Carrier::Plane {
                    origin: p.origin,
                    normal: p.normal.get(),
                },
                Surface::Cylinder { frame, radius } => Carrier::Cylinder {
                    origin: frame.origin,
                    axis: frame.normal.get(),
                    radius,
                },
                _ => return None,
            };
            let bbox = s
                .sub_shape(SubShape {
                    kind: SubKind::Face,
                    index: f.0,
                })
                .ok()?
                .bbox();
            Some((carrier, bbox))
        })
        .collect()
}

fn boxes_meet(a: &Aabb3, b: &Aabb3, tol: f64) -> bool {
    a.min.x <= b.max.x + tol
        && b.min.x <= a.max.x + tol
        && a.min.y <= b.max.y + tol
        && b.min.y <= a.max.y + tol
        && a.min.z <= b.max.z + tol
        && b.min.z <= a.max.z + tol
}

fn same_carrier(a: &Carrier, b: &Carrier, tol: f64) -> bool {
    const PARALLEL: f64 = 1e-9;
    match (a, b) {
        (
            Carrier::Plane {
                origin: o1,
                normal: n1,
            },
            Carrier::Plane {
                origin: o2,
                normal: n2,
            },
        ) => n1.dot(*n2).abs() >= 1.0 - PARALLEL && (*o2 - *o1).dot(*n1).abs() <= tol,
        (
            Carrier::Cylinder {
                origin: o1,
                axis: a1,
                radius: r1,
            },
            Carrier::Cylinder {
                origin: o2,
                axis: a2,
                radius: r2,
            },
        ) => {
            let d = *o2 - *o1;
            a1.dot(*a2).abs() >= 1.0 - PARALLEL
                && (r1 - r2).abs() <= tol
                && (d - *a1 * d.dot(*a1)).length() <= tol
        }
        _ => false,
    }
}

/// Whether two operands (objects × tools; objects among each other without tools) have faces on
/// the same plane or cylinder within `tol` whose boxes meet: what gluing is for.
fn coincident(objects: &[Shape], tools: &[Shape], tol: f64) -> bool {
    let co: Vec<_> = objects.iter().map(carriers).collect();
    let pair = |a: &[(Carrier, Aabb3)], b: &[(Carrier, Aabb3)]| {
        a.iter().any(|(ca, ba)| {
            b.iter()
                .any(|(cb, bb)| boxes_meet(ba, bb, tol) && same_carrier(ca, cb, tol))
        })
    };
    if tools.is_empty() {
        (0..co.len()).any(|i| (i + 1..co.len()).any(|j| pair(&co[i], &co[j])))
    } else {
        let ct: Vec<_> = tools.iter().map(carriers).collect();
        co.iter().any(|a| ct.iter().any(|b| pair(a, b)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn carriers_coincide_within_tolerance() {
        let z = Vec3::Z;
        let p = |h: f64| Carrier::Plane {
            origin: Vec3::new(1.0, 2.0, h),
            normal: z,
        };
        assert!(same_carrier(&p(10.0), &p(10.0 + 5e-5), 1e-4));
        assert!(!same_carrier(&p(10.0), &p(10.0 + 2e-4), 1e-4));
        let flipped = Carrier::Plane {
            origin: Vec3::new(0.0, 0.0, 10.0),
            normal: -z,
        };
        assert!(same_carrier(&p(10.0), &flipped, 1e-6));
        let c = |x: f64, r: f64| Carrier::Cylinder {
            origin: Vec3::new(x, 0.0, 3.0),
            axis: z,
            radius: r,
        };
        assert!(same_carrier(&c(0.0, 5.0), &c(5e-5, 5.0 + 5e-5), 1e-4));
        assert!(!same_carrier(&c(0.0, 5.0), &c(0.0, 5.1), 1e-4));
        assert!(!same_carrier(&c(0.0, 5.0), &p(0.0), 1.0));
    }

    #[test]
    fn boxes_meet_with_slack() {
        let a = Aabb3::new(Vec3::ZERO, Vec3::splat(1.0));
        let b = Aabb3::new(Vec3::new(1.0 + 1e-5, 0.0, 0.0), Vec3::new(2.0, 1.0, 1.0));
        assert!(!boxes_meet(&a, &b, 1e-6));
        assert!(boxes_meet(&a, &b, 1e-4));
    }
}
