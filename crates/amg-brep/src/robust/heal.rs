//! Healing levels with composed history (design note §2.5).

use super::{
    Attempt, AttemptOutcome, HealLevel, RobustCtx, RobustError, RobustOutput, RobustReport,
    StrategyId, Validation, validate,
};
use crate::capture::FailureInfo;
use crate::history::{InputSub, OpResult, Relation, ShapeHistory};
use crate::kernel::BrepKernel;
use crate::opts::{HealOpts, SewOpts, UnifyOpts};
use crate::shape::{Shape, SubKind, SubShape};

const KINDS: [SubKind; 3] = [SubKind::Face, SubKind::Edge, SubKind::Vertex];

/// The history of an op that returns `s` unchanged: every sub-shape `Kept`.
pub fn identity_history(s: &Shape) -> ShapeHistory {
    let mut entries = Vec::new();
    let mut counts = [0u32; 3];
    for (k, kind) in KINDS.into_iter().enumerate() {
        counts[k] = s.count(kind);
        for index in 0..counts[k] {
            let sub = SubShape { kind, index };
            entries.push((InputSub { operand: 0, sub }, Relation::Kept(sub)));
        }
    }
    entries.sort_by_key(|(input, _)| *input);
    ShapeHistory {
        entries,
        operand_counts: vec![counts],
        ..ShapeHistory::default()
    }
}

/// Composes per-operand pre-steps with a multi-operand op: `pre[i]` maps the caller's operand
/// `i` (A_i) to the shape the op got as operand `i` (B_i); `next` maps B_0..B_n to C. The result
/// maps A_0..A_n to C, as if the op had run on the caller's shapes ([`ShapeHistory::compose`]
/// applied per operand). Roles and tags are `next`'s; `operand_counts` the callers'.
///
/// # Panics
/// When `pre` has fewer histories than `next` has operands.
pub fn compose_operands(pre: &[ShapeHistory], next: &ShapeHistory) -> ShapeHistory {
    let mut entries = Vec::new();
    let mut operand_counts = Vec::with_capacity(pre.len());
    for (i, before) in pre.iter().enumerate() {
        let Ok(op) = u16::try_from(i) else { break };
        let slice = ShapeHistory {
            entries: next
                .entries
                .iter()
                .filter(|(input, _)| input.operand == op)
                .map(|(input, r)| {
                    (
                        InputSub {
                            operand: 0,
                            sub: input.sub,
                        },
                        r.clone(),
                    )
                })
                .collect(),
            operand_counts: next.operand_counts.get(i).copied().into_iter().collect(),
            ..ShapeHistory::default()
        };
        let composed = before.compose(&slice);
        entries.extend(composed.entries.into_iter().map(|(input, r)| {
            (
                InputSub {
                    operand: op,
                    sub: input.sub,
                },
                r,
            )
        }));
        operand_counts.push(before.operand_counts.first().copied().unwrap_or([0; 3]));
    }
    entries.sort_by_key(|(input, _)| *input);
    ShapeHistory {
        entries,
        roles: next.roles.clone(),
        tags: next.tags.clone(),
        operand_counts,
    }
}

/// `s` healed at `level` on a copy (design note §2.5), with one history from `s`:
/// - `None`: `s` itself, every sub-shape `Kept`;
/// - `Light`: [`BrepKernel::heal`] at `cx.tol.linear`, tolerances capped at 1e-3 mm;
/// - `Unify`: `Light`, then [`BrepKernel::unify_same_domain`] with the default options;
/// - `Full`: `Unify`, then [`BrepKernel::sew`] at 1e-3 mm (into solids when `s` has solids).
///
/// Each step is one attempt in the report (`Heal(Light)`, `UnifySameDomain`, `Heal(Full)`);
/// the result passes the gate (BRepCheck, tolerance cap) and the report's warning names the
/// level. Warnings of the steps are kept in `result.warnings`.
///
/// # Errors
/// The first failing step's class (`Cancelled` when cancelled), or `InvalidResult` /
/// `ToleranceTooLarge` when the healed shape fails the gate.
pub fn heal(
    k: &dyn BrepKernel,
    s: &Shape,
    level: HealLevel,
    cx: &RobustCtx<'_>,
) -> Result<RobustOutput, RobustError> {
    heal_capped(k, s, level, 1e-3, cx)
}

pub(crate) fn heal_capped(
    k: &dyn BrepKernel,
    s: &Shape,
    level: HealLevel,
    cap: f64,
    cx: &RobustCtx<'_>,
) -> Result<RobustOutput, RobustError> {
    let mut report = RobustReport::default();
    let mut current = OpResult {
        shape: s.clone(),
        history: identity_history(s),
        warnings: Vec::new(),
    };
    let had_solids = !s.solids().is_empty();
    let steps: Vec<StrategyId> = level
        .ladder()
        .map(|l| match l {
            HealLevel::Unify => StrategyId::UnifySameDomain,
            other => StrategyId::Heal(other),
        })
        .collect();
    for step in steps {
        let start = std::time::Instant::now();
        let shape = &current.shape;
        let out = match step {
            StrategyId::UnifySameDomain => k.unify_same_domain(shape, &UnifyOpts::default()),
            StrategyId::Heal(HealLevel::Full) => k.sew(
                std::slice::from_ref(shape),
                &SewOpts {
                    tol: cap,
                    non_manifold: false,
                    make_solid: had_solids,
                },
                cx.job,
            ),
            _ => k.heal(
                shape,
                &HealOpts {
                    tol: cx.tol.linear,
                    max_tol: cap.max(cx.tol.linear),
                    ..HealOpts::default()
                },
                cx.job,
            ),
        };
        let millis = u32::try_from(start.elapsed().as_millis()).unwrap_or(u32::MAX);
        match out {
            Ok(next) => {
                report.attempts.push(Attempt {
                    strategy: step,
                    outcome: AttemptOutcome::Accepted,
                    millis,
                });
                let mut history = current.history.compose(&next.history);
                let mut warnings = std::mem::take(&mut current.warnings);
                warnings.extend(next.warnings);
                if let Err(e) = history.order_splits(&next.shape) {
                    tracing::debug!("robust heal: split order not applied: {e}");
                }
                current = OpResult {
                    shape: next.shape,
                    history,
                    warnings,
                };
            }
            Err(e) => {
                let info = FailureInfo::from_kernel_error(&e, Some(shape));
                report.attempts.push(Attempt {
                    strategy: step,
                    outcome: AttemptOutcome::Failed(info.clone()),
                    millis,
                });
                return Err(RobustError {
                    class: info.class,
                    message: format!("heal {level:?} failed at {step}: {}", info.message),
                    report,
                    error: Some(Box::new(e)),
                });
            }
        }
    }
    let gate = Validation::with_cap(cap.max(s.max_tolerance()));
    match validate(k, &current.shape, &gate) {
        Ok(max) => report.max_tolerance = max,
        Err(rejection) => {
            if let Some(last) = report.attempts.last_mut() {
                last.outcome = AttemptOutcome::Rejected(rejection.clone());
            }
            return Err(RobustError {
                class: rejection.class(),
                message: format!("heal {level:?} gave a shape the gate refused: {rejection}"),
                report,
                error: None,
            });
        }
    }
    report.accepted = Some(StrategyId::Heal(level));
    if level != HealLevel::None {
        report
            .warnings
            .push(format!("shape healed at level {level:?}"));
    }
    Ok(RobustOutput {
        result: current,
        report,
    })
}

#[cfg(test)]
mod tests {
    use smallvec::smallvec;

    use super::*;

    fn sub(kind: SubKind, index: u32) -> SubShape {
        SubShape { kind, index }
    }

    #[test]
    fn operands_compose_independently() {
        let f = |i| sub(SubKind::Face, i);
        // Operand 0: face 0 → B face 1 (renumbered), face 1 → B faces 0 and 2 (split).
        let pre0 = ShapeHistory {
            entries: vec![
                (
                    InputSub {
                        operand: 0,
                        sub: f(0),
                    },
                    Relation::Modified(smallvec![f(1)]),
                ),
                (
                    InputSub {
                        operand: 0,
                        sub: f(1),
                    },
                    Relation::Modified(smallvec![f(0), f(2)]),
                ),
            ],
            operand_counts: vec![[2, 0, 0]],
            ..ShapeHistory::default()
        };
        // Operand 1: face 0 → B face 0.
        let pre1 = ShapeHistory {
            entries: vec![(
                InputSub {
                    operand: 0,
                    sub: f(0),
                },
                Relation::Modified(smallvec![f(0)]),
            )],
            operand_counts: vec![[1, 0, 0]],
            ..ShapeHistory::default()
        };
        // The op: B0 face 0 kept as C 0, B0 face 1 deleted, B0 face 2 → C 1; B1 face 0 → C 2.
        let next = ShapeHistory {
            entries: vec![
                (
                    InputSub {
                        operand: 0,
                        sub: f(0),
                    },
                    Relation::Kept(f(0)),
                ),
                (
                    InputSub {
                        operand: 0,
                        sub: f(1),
                    },
                    Relation::Deleted,
                ),
                (
                    InputSub {
                        operand: 0,
                        sub: f(2),
                    },
                    Relation::Modified(smallvec![f(1)]),
                ),
                (
                    InputSub {
                        operand: 1,
                        sub: f(0),
                    },
                    Relation::Modified(smallvec![f(2)]),
                ),
            ],
            operand_counts: vec![[3, 0, 0], [1, 0, 0]],
            ..ShapeHistory::default()
        };
        let c = compose_operands(&[pre0, pre1], &next);
        assert_eq!(c.operand_counts, vec![[2, 0, 0], [1, 0, 0]]);
        let of = |operand, i| {
            c.of_input(InputSub { operand, sub: f(i) })
                .cloned()
                .collect::<Vec<_>>()
        };
        assert_eq!(of(0, 0), vec![Relation::Deleted]);
        assert_eq!(of(0, 1), vec![Relation::Modified(smallvec![f(0), f(1)])]);
        assert_eq!(of(1, 0), vec![Relation::Modified(smallvec![f(2)])]);
    }
}
