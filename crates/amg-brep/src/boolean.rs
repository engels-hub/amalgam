//! Booleans, split and section (S02 §5, BREP-04): input validation, option lowering and the
//! conversion of the shim's status in plain Rust (so the unit tests run without OCCT); the OCCT
//! calls are in `backend/occt/boolean.rs`.

use crate::backend::Occ;
use crate::error::{KResult, KernelError};
use crate::history::{KernelWarning, OpRole};
use crate::ops::RawHistory;
use crate::opts::{BoolOp, BoolOpts, Glue};
use crate::shape::{Shape, SubKind, SubShape};

/// The shim's operation codes (`boolean_op`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum BoolCode {
    Fuse = 0,
    Cut = 1,
    Common = 2,
    Split = 3,
}

impl From<BoolOp> for BoolCode {
    fn from(op: BoolOp) -> Self {
        match op {
            BoolOp::Fuse => BoolCode::Fuse,
            BoolOp::Cut => BoolCode::Cut,
            BoolOp::Common => BoolCode::Common,
        }
    }
}

/// Lowered options for the shim (`OcBoolOpts`).
#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct BoolIn {
    pub fuzzy: f64,
    /// 0 off, 1 shift, 2 full.
    pub glue: u8,
    pub check_inverted: bool,
    pub use_obb: bool,
    pub parallel: bool,
}

/// How a boolean ended, as the shim reports it (`OcBoolStatus`).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct BoolStatus {
    pub cancelled: bool,
    /// BOPAlgo error alert names; non-empty means the op failed.
    pub errors: Vec<String>,
    /// BOPAlgo warning alert names.
    pub warnings: Vec<String>,
    /// `(operand, sub-shape)` named by the error alerts.
    pub offending: Vec<(u16, SubShape)>,
}

/// What one boolean run gave: the result with its history and status (warnings), or the
/// status of a failed or cancelled run.
pub(crate) type BoolOutcome = Result<(Occ, RawHistory, BoolStatus), BoolStatus>;

fn invalid(what: &'static str, reason: impl Into<String>) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason: reason.into(),
        offending: Vec::new(),
    }
}

/// Checks the operand lists and options of `boolean`/`split` and lowers the options;
/// `default_parallel` is the kernel's default for `BoolOpts::parallel`.
pub(crate) fn lower(
    code: BoolCode,
    objects: &[Shape],
    tools: &[Shape],
    opts: &BoolOpts,
    default_parallel: bool,
) -> KResult<BoolIn> {
    if objects.is_empty() {
        return Err(invalid("objects", "at least one object is needed"));
    }
    match code {
        BoolCode::Fuse if objects.len() + tools.len() < 2 => {
            return Err(invalid("tools", "a fuse needs at least two shapes in all"));
        }
        BoolCode::Cut | BoolCode::Common if tools.is_empty() => {
            return Err(invalid("tools", "at least one tool is needed"));
        }
        _ => {}
    }
    if objects.len() + tools.len() > usize::from(u16::MAX) {
        return Err(invalid("objects", "more than 65535 operands"));
    }
    for (i, s) in objects.iter().chain(tools).enumerate() {
        if s.is_null() {
            return Err(invalid("objects", format!("operand {i} is a null shape")));
        }
    }
    if !opts.fuzzy.is_finite() || opts.fuzzy < 0.0 {
        return Err(invalid(
            "fuzzy",
            format!("must be finite and not negative, got {}", opts.fuzzy),
        ));
    }
    Ok(BoolIn {
        fuzzy: opts.fuzzy,
        glue: match opts.glue {
            Glue::Off => 0,
            Glue::Shift => 1,
            Glue::Full => 2,
        },
        check_inverted: opts.check_inverted,
        use_obb: opts.use_obb,
        parallel: opts.parallel.unwrap_or(default_parallel),
    })
}

/// The operand split that the shim gets: a fuse without tools fuses the first object with the
/// others. Operand numbering (objects first, then tools) is unchanged by it.
pub(crate) fn operand_split<'a>(
    code: BoolCode,
    objects: &'a [Shape],
    tools: &'a [Shape],
) -> (Vec<&'a Shape>, Vec<&'a Shape>) {
    if code == BoolCode::Fuse && tools.is_empty() {
        let (first, rest) = objects.split_at(1);
        (first.iter().collect(), rest.iter().collect())
    } else {
        (objects.iter().collect(), tools.iter().collect())
    }
}

/// The error for a failed status, `None` when the op succeeded.
pub(crate) fn status_error(op: &'static str, status: &BoolStatus) -> Option<KernelError> {
    if status.cancelled {
        return Some(KernelError::Cancelled);
    }
    if status.errors.is_empty() {
        return None;
    }
    let mut message = status.errors.join(", ");
    if !status.offending.is_empty() {
        let named: Vec<String> = status
            .offending
            .iter()
            .map(|(operand, sub)| format!("operand {operand} {:?} {}", sub.kind, sub.index))
            .collect();
        message.push_str(&format!(" (on {})", named.join(", ")));
    }
    Some(KernelError::OpFailed {
        op,
        message,
        offending: status.offending.iter().map(|(_, sub)| *sub).collect(),
    })
}

/// Non-fatal notes: the fuzzy value used, and tolerance growth past every input's and the
/// kernel's linear tolerance.
pub(crate) fn warnings(
    fuzzy: f64,
    operands: &[&Shape],
    result: &Shape,
    linear: f64,
) -> Vec<KernelWarning> {
    let mut out = Vec::new();
    if fuzzy > 0.0 {
        out.push(KernelWarning::FuzzyUsed(fuzzy));
    }
    let before = operands
        .iter()
        .map(|s| s.max_tolerance())
        .fold(linear, f64::max);
    let after = result.max_tolerance();
    if after > before {
        out.push(KernelWarning::ToleranceGrown { max: after });
    }
    out
}

/// `SectionEdge` on every edge of a section result.
pub(crate) fn section_roles(result: &Shape) -> Vec<(SubShape, OpRole)> {
    (0..result.count(SubKind::Edge))
        .map(|index| {
            (
                SubShape {
                    kind: SubKind::Edge,
                    index,
                },
                OpRole::SectionEdge,
            )
        })
        .collect()
}

/// `SplitPiece { index }` on every face of a split result: `index` is the position of the
/// face's solid in the result's solid order ([`Shape::solids`], the order of
/// [`crate::OpResult::per_solid`]). A face shared by two pieces (the cut face of a body split by
/// a face) takes the lower index; faces outside any solid get none.
pub(crate) fn split_roles(result: &Shape) -> KResult<Vec<(SubShape, OpRole)>> {
    let mut piece: Vec<Option<u32>> = vec![None; result.count(SubKind::Face) as usize];
    for (k, solid) in result.solids().iter().enumerate() {
        let index = u32::try_from(k).unwrap_or(u32::MAX);
        let [faces, _, _] = result.occ().sub_index_map(solid.occ())?;
        for whole in faces.into_iter().flatten() {
            if let Some(slot) = piece.get_mut(whole as usize) {
                slot.get_or_insert(index);
            }
        }
    }
    Ok(piece
        .into_iter()
        .enumerate()
        .filter_map(|(f, k)| {
            let face = SubShape {
                kind: SubKind::Face,
                index: u32::try_from(f).ok()?,
            };
            Some((face, OpRole::SplitPiece { index: k? }))
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn options_are_lowered_and_checked() {
        let opts = BoolOpts {
            fuzzy: 1e-5,
            glue: Glue::Full,
            check_inverted: false,
            use_obb: true,
            parallel: None,
        };
        // No shapes exist without the backend, so only the list checks that come first and the
        // option checks on an empty fuse are reachable here.
        assert!(matches!(
            lower(BoolCode::Fuse, &[], &[], &opts, true),
            Err(KernelError::InvalidInput {
                what: "objects",
                ..
            })
        ));
        assert!(matches!(
            lower(BoolCode::Split, &[], &[], &opts, true),
            Err(KernelError::InvalidInput {
                what: "objects",
                ..
            })
        ));
    }

    #[test]
    fn status_becomes_the_right_error() {
        assert!(status_error("boolean", &BoolStatus::default()).is_none());
        let cancelled = BoolStatus {
            cancelled: true,
            ..BoolStatus::default()
        };
        assert!(matches!(
            status_error("boolean", &cancelled),
            Some(KernelError::Cancelled)
        ));
        let face = SubShape {
            kind: SubKind::Face,
            index: 4,
        };
        let failed = BoolStatus {
            errors: vec!["BOPAlgo_AlertBOPNotAllowed".into()],
            offending: vec![(1, face)],
            ..BoolStatus::default()
        };
        match status_error("boolean", &failed) {
            Some(KernelError::OpFailed {
                op,
                message,
                offending,
            }) => {
                assert_eq!(op, "boolean");
                assert!(
                    message.starts_with("BOPAlgo_AlertBOPNotAllowed"),
                    "{message}"
                );
                assert!(message.contains("operand 1 Face 4"), "{message}");
                assert_eq!(offending, vec![face]);
            }
            other => panic!("expected OpFailed, got {other:?}"),
        }
    }
}
