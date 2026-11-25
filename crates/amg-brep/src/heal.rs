//! Healing and assembly (S02 §9, BREP-10): input validation, lowering to the shim's parameters
//! and the conversion of the shim's status, in plain Rust (so the unit tests run without OCCT);
//! the OCCT calls are in `backend/occt/heal.rs` and `cpp/occt_heal.cpp`.

use crate::boolean::BoolStatus;
use crate::error::{KResult, KernelError};
use crate::opts::{HealOpts, SewOpts, UnifyOpts};
use crate::shape::{Shape, SubKind, SubShape};

/// A lowered `heal` (`OcHealParams`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct HealIn {
    pub tol: f64,
    pub max_tol: f64,
    pub fix_small_edges: bool,
    pub fix_orientation: bool,
}

/// A lowered `unify_same_domain` (`OcUnifyParams`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct UnifyIn {
    pub faces: bool,
    pub edges: bool,
    pub concat_bsplines: bool,
    pub lin_tol: f64,
    pub ang_tol: f64,
}

/// A lowered `sew` (`OcSewParams`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SewIn {
    pub tol: f64,
    pub non_manifold: bool,
    pub make_solid: bool,
}

/// What a heal, sew or make-solid run gave: the result, or the status of a failed run with the
/// shape it got to (when it has one).
pub(crate) type HealOutcome<T> = Result<T, (BoolStatus, Option<Shape>)>;

fn invalid(what: &'static str, reason: impl Into<String>) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason: reason.into(),
        offending: Vec::new(),
    }
}

fn not_null(what: &'static str, s: &Shape) -> KResult<()> {
    if s.is_null() {
        return Err(invalid(what, "the shape is null"));
    }
    Ok(())
}

/// A tolerance (mm) that must be finite and positive.
fn positive(name: &str, v: f64) -> KResult<()> {
    if !v.is_finite() || v <= 0.0 {
        return Err(invalid(
            "opts",
            format!("{name} must be finite and positive, got {v}"),
        ));
    }
    Ok(())
}

/// The heal options alone: `tol` finite and positive, `max_tol` finite and at least `tol`.
pub(crate) fn heal_params(opts: &HealOpts) -> KResult<HealIn> {
    positive("the healing tolerance", opts.tol)?;
    if !opts.max_tol.is_finite() || opts.max_tol < opts.tol {
        return Err(invalid(
            "opts",
            format!(
                "the largest tolerance must be finite and at least the healing tolerance {}, \
                 got {}",
                opts.tol, opts.max_tol
            ),
        ));
    }
    Ok(HealIn {
        tol: opts.tol,
        max_tol: opts.max_tol,
        fix_small_edges: opts.fix_small_edges,
        fix_orientation: opts.fix_orientation,
    })
}

/// Lowers a heal of `s` (any non-null shape).
pub(crate) fn lower_heal(s: &Shape, opts: &HealOpts) -> KResult<HealIn> {
    not_null("shape", s)?;
    heal_params(opts)
}

/// The unify options alone: `lin_tol` finite and ≥ 0, `ang_tol` in `[0, π)`.
pub(crate) fn unify_params(opts: &UnifyOpts) -> KResult<UnifyIn> {
    if !opts.lin_tol.is_finite() || opts.lin_tol < 0.0 {
        return Err(invalid(
            "opts",
            format!(
                "the linear tolerance must be finite and ≥ 0, got {}",
                opts.lin_tol
            ),
        ));
    }
    if !opts.ang_tol.is_finite() || !(0.0..std::f64::consts::PI).contains(&opts.ang_tol) {
        return Err(invalid(
            "opts",
            format!(
                "the angular tolerance must be in [0, π) rad, got {}",
                opts.ang_tol
            ),
        ));
    }
    Ok(UnifyIn {
        faces: opts.faces,
        edges: opts.edges,
        concat_bsplines: opts.concat_bsplines,
        lin_tol: opts.lin_tol,
        ang_tol: opts.ang_tol,
    })
}

/// Lowers a unify of `s` (any non-null shape).
pub(crate) fn lower_unify(s: &Shape, opts: &UnifyOpts) -> KResult<UnifyIn> {
    not_null("shape", s)?;
    unify_params(opts)
}

/// The sewing options alone: `tol` finite and positive.
pub(crate) fn sew_params(opts: &SewOpts) -> KResult<SewIn> {
    positive("the sewing tolerance", opts.tol)?;
    Ok(SewIn {
        tol: opts.tol,
        non_manifold: opts.non_manifold,
        make_solid: opts.make_solid,
    })
}

/// Lowers a sew: at least one shape, at most `u16::MAX + 1` (operand numbers are `u16`), each
/// with a face.
pub(crate) fn lower_sew(faces: &[Shape], opts: &SewOpts) -> KResult<SewIn> {
    if faces.is_empty() {
        return Err(invalid("faces", "sewing needs at least one shape"));
    }
    if faces.len() > usize::from(u16::MAX) + 1 {
        return Err(invalid(
            "faces",
            format!(
                "at most {} shapes can be sewn at once, got {}",
                usize::from(u16::MAX) + 1,
                faces.len()
            ),
        ));
    }
    for (i, s) in faces.iter().enumerate() {
        if s.is_null() || s.count(SubKind::Face) == 0 {
            return Err(invalid("faces", format!("shape {i} has no faces")));
        }
    }
    sew_params(opts)
}

/// `make_solid` takes shells (or faces, or a compound of them) without solids.
pub(crate) fn check_make_solid(s: &Shape) -> KResult<()> {
    not_null("shell", s)?;
    if s.count(SubKind::Face) == 0 {
        return Err(invalid("shell", "the shape has no faces"));
    }
    let solids = s.solids().len();
    if solids > 0 {
        return Err(invalid(
            "shell",
            format!("the shape already has {solids} solid(s)"),
        ));
    }
    Ok(())
}

/// `make_compound` takes non-null parts.
pub(crate) fn check_parts(parts: &[Shape]) -> KResult<()> {
    if let Some(i) = parts.iter().position(Shape::is_null) {
        return Err(invalid("parts", format!("part {i} is null")));
    }
    Ok(())
}

/// The error of a failed heal, sew or make-solid run. An error starting with `invalid: ` is a bad
/// input (`what`); any other is `LocalOpFailed` with the shape the run got to as `partial`.
pub(crate) fn failure(
    op: &'static str,
    what: &'static str,
    status: BoolStatus,
    partial: Option<Shape>,
) -> KernelError {
    if status.cancelled {
        return KernelError::Cancelled;
    }
    let mut offending: Vec<SubShape> = status.offending.iter().map(|(_, s)| *s).collect();
    offending.sort();
    offending.dedup();
    if let Some(reason) = status
        .errors
        .iter()
        .find_map(|e| e.strip_prefix("invalid: "))
    {
        return KernelError::InvalidInput {
            what,
            reason: reason.to_owned(),
            offending,
        };
    }
    let message = if status.errors.is_empty() {
        "NotDone".to_owned()
    } else {
        status.errors.join("; ")
    };
    KernelError::LocalOpFailed {
        op,
        message,
        offending,
        partial,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heal_tolerances_are_checked() {
        let ok = HealOpts::default();
        assert!(heal_params(&ok).is_ok());
        for (tol, max_tol) in [
            (0.0, 1.0),
            (-1e-6, 1.0),
            (f64::NAN, 1.0),
            (1e-3, 1e-4),
            (1e-6, f64::INFINITY),
        ] {
            let opts = HealOpts {
                tol,
                max_tol,
                ..ok.clone()
            };
            assert!(heal_params(&opts).is_err(), "{tol} {max_tol}");
        }
        let equal = HealOpts {
            tol: 1e-3,
            max_tol: 1e-3,
            ..ok
        };
        assert_eq!(
            heal_params(&equal).unwrap().max_tol.to_bits(),
            1e-3_f64.to_bits()
        );
    }

    #[test]
    fn unify_and_sew_tolerances_are_checked() {
        let ok = UnifyOpts::default();
        assert!(unify_params(&ok).is_ok());
        for (lin_tol, ang_tol) in [(-1.0, 0.0), (f64::NAN, 0.0), (0.0, -0.1), (0.0, 4.0)] {
            let opts = UnifyOpts {
                lin_tol,
                ang_tol,
                ..ok.clone()
            };
            assert!(unify_params(&opts).is_err(), "{lin_tol} {ang_tol}");
        }
        assert!(sew_params(&SewOpts::default()).is_ok());
        for tol in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let opts = SewOpts {
                tol,
                ..SewOpts::default()
            };
            assert!(sew_params(&opts).is_err(), "{tol}");
        }
    }

    #[test]
    fn failures_map_to_invalid_input_or_local_op_failed() {
        let status = |errors: &[&str], cancelled: bool| BoolStatus {
            cancelled,
            errors: errors.iter().map(|e| (*e).to_owned()).collect(),
            warnings: Vec::new(),
            offending: Vec::new(),
        };
        assert!(matches!(
            failure("sew", "faces", status(&[], true), None),
            KernelError::Cancelled
        ));
        assert!(matches!(
            failure(
                "make_solid",
                "shell",
                status(&["invalid: open"], false),
                None
            ),
            KernelError::InvalidInput { what: "shell", .. }
        ));
        match failure("sew", "faces", status(&["not closed"], false), None) {
            KernelError::LocalOpFailed { op, message, .. } => {
                assert_eq!(op, "sew");
                assert_eq!(message, "not closed");
            }
            other => panic!("{other:?}"),
        }
    }
}
