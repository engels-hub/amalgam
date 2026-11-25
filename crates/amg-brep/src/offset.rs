//! Shell (thick solid), offset and thicken (S02 §5, BREP-07): input validation, lowering to the
//! shim's parameters and the conversion of the shim's status, in plain Rust (so the unit tests
//! run without OCCT); the OCCT calls are in `backend/occt/offset.rs` and `cpp/occt_offset.cpp`.

use std::collections::BTreeSet;

use amg_core::Tolerance;

use crate::boolean::BoolStatus;
use crate::error::{KResult, KernelError};
use crate::opts::{OffsetJoin, ShellOpts};
use crate::shape::{FaceId, Shape, ShapeKind, SubKind, SubShape};

/// The shim's modes (`OcOffsetParams::mode`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum OffsetMode {
    Shell = 0,
    Offset = 1,
    Thicken = 2,
}

/// One lowered call (`OcOffsetParams` plus the face lists).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct OffsetIn {
    pub mode: OffsetMode,
    /// Offset along the outward normal, mm (negative is inward).
    pub offset: f64,
    /// The offset algorithm's confusion tolerance, mm.
    pub tol: f64,
    /// 0 arc, 1 intersection.
    pub join: u8,
    /// Removed (opening) faces of a shell.
    pub remove: Vec<u32>,
    /// Faces with their own offset, and those offsets.
    pub faces: Vec<u32>,
    pub offsets: Vec<f64>,
}

/// What an offset run gave: the result with its history, or the status of a failed run with
/// OCCT's partial result (when it has one).
pub(crate) type OffsetOutcome<T> = Result<T, (BoolStatus, Option<Shape>)>;

fn invalid(what: &'static str, reason: impl Into<String>, face: Option<FaceId>) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason: reason.into(),
        offending: face.map(face_sub).into_iter().collect(),
    }
}

pub(crate) fn face_sub(f: FaceId) -> SubShape {
    SubShape {
        kind: SubKind::Face,
        index: f.0,
    }
}

fn join_code(join: OffsetJoin) -> u8 {
    match join {
        OffsetJoin::Arc => 0,
        OffsetJoin::Intersection => 1,
    }
}

/// A distance must be finite and longer than the linear tolerance (either sign).
fn distance(what: &'static str, name: &str, v: f64, tol: &Tolerance) -> KResult<()> {
    if !v.is_finite() || v.abs() <= tol.linear {
        return Err(invalid(
            what,
            format!(
                "{name} must be finite and longer than {:e} mm (either sign), got {v}",
                tol.linear
            ),
            None,
        ));
    }
    Ok(())
}

/// The confusion tolerance: finite and ≥ 0; 0 takes the kernel's linear tolerance.
fn confusion(v: f64, tol: &Tolerance) -> KResult<f64> {
    if !v.is_finite() || v < 0.0 {
        return Err(invalid(
            "tol",
            format!("the tolerance must be finite and ≥ 0, got {v}"),
            None,
        ));
    }
    Ok(if v == 0.0 { tol.linear } else { v })
}

fn has_faces(s: &Shape) -> KResult<()> {
    if s.is_null() || s.count(SubKind::Face) == 0 {
        return Err(invalid("shape", "the shape has no faces", None));
    }
    Ok(())
}

fn face_in_range(what: &'static str, f: FaceId, n: u32) -> KResult<()> {
    if f.0 >= n {
        return Err(invalid(
            what,
            format!("face {} is out of range (the shape has {n} faces)", f.0),
            None,
        ));
    }
    Ok(())
}

/// Lowers a shell: `s` holds exactly one solid; `remove` lists distinct faces, not all of them;
/// `per_face` lists distinct faces that are not removed, each with a thickness of the sign of
/// `opts.thickness`.
pub(crate) fn lower_shell(
    s: &Shape,
    remove: &[FaceId],
    opts: &ShellOpts,
    tol: &Tolerance,
) -> KResult<OffsetIn> {
    has_faces(s)?;
    let solids = s.solids().len();
    if solids != 1 {
        return Err(invalid(
            "shape",
            format!("a shell needs exactly one solid, the shape has {solids}"),
            None,
        ));
    }
    distance("opts", "the thickness", opts.thickness, tol)?;
    let n = s.count(SubKind::Face);
    let mut removed = BTreeSet::new();
    for &f in remove {
        face_in_range("remove", f, n)?;
        if !removed.insert(f) {
            return Err(invalid(
                "remove",
                format!("face {} is listed twice", f.0),
                Some(f),
            ));
        }
    }
    if removed.len() as u32 == n {
        return Err(invalid("remove", "every face is removed", None));
    }
    let mut seen = BTreeSet::new();
    let (mut faces, mut offsets) = (Vec::new(), Vec::new());
    for &(f, t) in &opts.per_face {
        face_in_range("opts", f, n)?;
        if !seen.insert(f) {
            return Err(invalid(
                "opts",
                format!("face {} has two thicknesses", f.0),
                Some(f),
            ));
        }
        if removed.contains(&f) {
            return Err(invalid(
                "opts",
                format!("face {} is removed and has a thickness", f.0),
                Some(f),
            ));
        }
        distance("opts", &format!("the thickness of face {}", f.0), t, tol)?;
        if t.signum() != opts.thickness.signum() {
            return Err(invalid(
                "opts",
                format!(
                    "face {} has thickness {t}, of the other side than the shell's {}",
                    f.0, opts.thickness
                ),
                Some(f),
            ));
        }
        faces.push(f.0);
        offsets.push(t);
    }
    Ok(OffsetIn {
        mode: OffsetMode::Shell,
        offset: opts.thickness,
        tol: confusion(opts.tol, tol)?,
        join: join_code(opts.join),
        remove: removed.into_iter().map(|f| f.0).collect(),
        faces,
        offsets,
    })
}

/// Lowers an offset of a face, shell, solid or compound of them.
pub(crate) fn lower_offset(
    s: &Shape,
    d: f64,
    join: OffsetJoin,
    op_tol: f64,
    tol: &Tolerance,
) -> KResult<OffsetIn> {
    has_faces(s)?;
    distance("distance", "the distance", d, tol)?;
    Ok(OffsetIn {
        mode: OffsetMode::Offset,
        offset: d,
        tol: confusion(op_tol, tol)?,
        join: join_code(join),
        remove: Vec::new(),
        faces: Vec::new(),
        offsets: Vec::new(),
    })
}

/// Lowers a thicken: `s` is a face, a shell or a compound of them, without solids.
pub(crate) fn lower_thicken(
    s: &Shape,
    thickness: f64,
    join: OffsetJoin,
    tol: &Tolerance,
) -> KResult<OffsetIn> {
    has_faces(s)?;
    let kind = s.kind();
    if !matches!(
        kind,
        ShapeKind::Face | ShapeKind::Shell | ShapeKind::Compound
    ) || !s.solids().is_empty()
    {
        return Err(invalid(
            "shape",
            format!("thicken takes a face or a shell without solids, not a {kind:?}"),
            None,
        ));
    }
    distance("thickness", "the thickness", thickness, tol)?;
    Ok(OffsetIn {
        mode: OffsetMode::Thicken,
        offset: thickness,
        tol: tol.linear,
        join: join_code(join),
        remove: Vec::new(),
        faces: Vec::new(),
        offsets: Vec::new(),
    })
}

/// The error of a failed offset: `InvalidInput` for the shim's `invalid: ` errors, else
/// `LocalOpFailed` with the offending input faces and the partial result.
pub(crate) fn failure(op: &'static str, status: BoolStatus, partial: Option<Shape>) -> KernelError {
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
            what: "faces",
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

    const TOL: Tolerance = Tolerance::DEFAULT;

    #[test]
    fn distances_and_tolerances_are_checked() {
        assert!(distance("d", "d", -0.5, &TOL).is_ok());
        assert!(distance("d", "d", 0.0, &TOL).is_err());
        assert!(distance("d", "d", 1e-9, &TOL).is_err());
        assert!(distance("d", "d", f64::NAN, &TOL).is_err());
        assert_eq!(
            confusion(0.0, &TOL).unwrap().to_bits(),
            TOL.linear.to_bits()
        );
        assert_eq!(confusion(1e-4, &TOL).unwrap().to_bits(), 1e-4f64.to_bits());
        assert!(confusion(-1.0, &TOL).is_err());
        assert!(confusion(f64::INFINITY, &TOL).is_err());
        assert_eq!(join_code(OffsetJoin::Arc), 0);
        assert_eq!(join_code(OffsetJoin::Intersection), 1);
    }

    #[test]
    fn failures_carry_sorted_faces_or_become_invalid_input() {
        let status = BoolStatus {
            errors: vec!["face 4: offset -6 mm reaches its centre of curvature".into()],
            offending: vec![(0, face_sub(FaceId(4))), (0, face_sub(FaceId(1)))],
            ..BoolStatus::default()
        };
        match failure("shell", status, None) {
            KernelError::LocalOpFailed {
                op,
                message,
                offending,
                partial,
            } => {
                assert_eq!(op, "shell");
                assert!(message.contains("centre of curvature"));
                assert_eq!(offending, vec![face_sub(FaceId(1)), face_sub(FaceId(4))]);
                assert!(partial.is_none());
            }
            other => panic!("{other:?}"),
        }
        let status = BoolStatus {
            errors: vec!["invalid: face 3 is removed and has a thickness".into()],
            ..BoolStatus::default()
        };
        assert!(matches!(
            failure("shell", status, None),
            KernelError::InvalidInput { what: "faces", .. }
        ));
        let cancelled = BoolStatus {
            cancelled: true,
            ..BoolStatus::default()
        };
        assert!(matches!(
            failure("offset", cancelled, None),
            KernelError::Cancelled
        ));
    }
}
