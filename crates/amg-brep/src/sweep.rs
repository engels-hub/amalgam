//! Pipe-shell sweeps and the raw ThruSections loft (S02 §5, BREP-15): input validation, lowering
//! and the failure mapping in plain Rust (unit-tested without OCCT); the OCCT calls are in
//! `backend/occt/sweep.rs` and `amg-occt-sys/cpp/occt_sweep.cpp`.

use amg_core::{Dir3, Point3, Vec3};

use crate::boolean::BoolStatus;
use crate::error::{KResult, KernelError};
use crate::opts::{LoftSection, PipeMode, PipeOpts, ThruSectionsOpts, Transition};
use crate::shape::{Shape, ShapeKind, SubKind};

/// A direction is accepted when its length is 1 within this.
const UNIT_EPS: f64 = 1e-9;

/// Law parameters must start at 0 and end at 1 within this.
const LAW_END_EPS: f64 = 1e-12;

/// Smallest accepted scale factor.
const MIN_SCALE: f64 = 1e-6;

/// Largest B-spline degree OCCT builds (`Geom_BSplineSurface::MaxDegree`).
const MAX_DEGREE: u8 = 25;

/// The shim's `OcLoftSection::wire` of a point section.
pub(crate) const POINT_SECTION: u32 = u32::MAX;

/// A validated pipe, for the shim (`OcPipeParams` plus the laws).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PipeIn {
    pub profiles: u32,
    /// 0 corrected Frenet, 1 Frenet, 2 fixed, 3 binormal, 4 guide, 5 support.
    pub mode: u8,
    pub binormal: Vec3,
    /// Operand of the guide or support (0 for none).
    pub aux: u32,
    pub contact: bool,
    /// 0 transformed, 1 right corner, 2 round corner.
    pub transition: u8,
    pub solid: bool,
    /// Flat `(t, rad)` pairs; empty for none.
    pub twist: Vec<f64>,
    /// Flat `(t, factor)` pairs; empty for none.
    pub scale: Vec<f64>,
}

/// A validated loft, for the shim.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct LoftIn {
    /// Per section: the wire operand (`POINT_SECTION` for a point) and the point.
    pub sections: Vec<(u32, Point3)>,
    pub solid: bool,
    pub ruled: bool,
    pub check_compatibility: bool,
    pub smoothing: bool,
    pub max_degree: u8,
}

fn invalid(what: &'static str, reason: impl Into<String>) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason: reason.into(),
        offending: Vec::new(),
    }
}

fn unit(what: &'static str, d: Dir3) -> KResult<Vec3> {
    let v = d.get();
    if !v.is_finite() || (v.length() - 1.0).abs() > UNIT_EPS {
        return Err(invalid(what, "the direction must be a unit vector"));
    }
    Ok(v)
}

/// A wire or an edge with at least one edge (spines, guides, profiles, loft sections).
fn check_wire(what: &'static str, name: &str, s: &Shape) -> KResult<()> {
    if s.is_null() {
        return Err(invalid(what, format!("{name} is null")));
    }
    match s.kind() {
        ShapeKind::Wire | ShapeKind::Edge if s.count(SubKind::Edge) > 0 => Ok(()),
        ShapeKind::Wire | ShapeKind::Edge => Err(invalid(what, format!("{name} has no edges"))),
        other => Err(invalid(
            what,
            format!("{name} must be a wire or an edge, not a {other:?}"),
        )),
    }
}

/// A law of `(t, value)` samples: at least two, `t` strictly increasing from 0 to 1, values
/// finite and above `min` when given. Returned flat.
fn law(what: &'static str, samples: &[(f64, f64)], min: Option<f64>) -> KResult<Vec<f64>> {
    if samples.len() < 2 {
        return Err(invalid(what, "a law needs at least two (t, value) samples"));
    }
    let mut flat = Vec::with_capacity(samples.len() * 2);
    for (k, &(t, v)) in samples.iter().enumerate() {
        if !t.is_finite() || !v.is_finite() {
            return Err(invalid(what, format!("sample {k} is not finite")));
        }
        if let Some(min) = min
            && v < min
        {
            return Err(invalid(
                what,
                format!("sample {k}: the factor must be at least {min:e}, got {v}"),
            ));
        }
        if k > 0 && t <= samples[k - 1].0 {
            return Err(invalid(
                what,
                format!("sample {k}: the parameters must increase strictly"),
            ));
        }
        flat.extend([t, v]);
    }
    let (first, last) = (samples[0].0, samples[samples.len() - 1].0);
    if first.abs() > LAW_END_EPS || (last - 1.0).abs() > LAW_END_EPS {
        return Err(invalid(
            what,
            format!("the parameters must run from 0 to 1, got {first} to {last}"),
        ));
    }
    // Snap the ends so the shim maps them exactly onto the spine's ends.
    flat[0] = 0.0;
    let n = flat.len();
    flat[n - 2] = 1.0;
    Ok(flat)
}

/// Validates a pipe and lowers it for the shim. `operands` are `[profiles..., spine, aux?]`.
pub(crate) fn lower_pipe(operands: &[Shape], profiles: usize, opts: &PipeOpts) -> KResult<PipeIn> {
    if profiles == 0 {
        return Err(invalid("profiles", "a pipe needs at least one profile"));
    }
    let aux_operand = match opts.mode {
        PipeMode::Guide { wire, .. } => Some(("guide", wire)),
        PipeMode::Support { shell } => Some(("support", shell)),
        _ => None,
    };
    let want = profiles + 1 + usize::from(aux_operand.is_some());
    if operands.len() != want {
        return Err(invalid(
            "operands",
            format!(
                "expected {want} operands ({profiles} profile(s), the spine{}), got {}",
                if aux_operand.is_some() {
                    ", the guide or support"
                } else {
                    ""
                },
                operands.len()
            ),
        ));
    }
    if let Some((what, index)) = aux_operand
        && index != profiles + 1
    {
        return Err(invalid(
            what,
            format!(
                "the {what} must be the operand after the spine ({}), got {index}",
                profiles + 1
            ),
        ));
    }
    let mut wires = 0;
    for (i, p) in operands[..profiles].iter().enumerate() {
        if p.is_null() {
            return Err(invalid("profiles", format!("profile {i} is null")));
        }
        if p.kind() == ShapeKind::Vertex {
            if i != 0 && i + 1 != profiles {
                return Err(invalid(
                    "profiles",
                    format!("profile {i}: only the first and last sections may be points"),
                ));
            }
            continue;
        }
        check_wire("profiles", &format!("profile {i}"), p)?;
        wires += 1;
    }
    if wires == 0 {
        return Err(invalid("profiles", "a pipe needs a wire profile"));
    }
    check_wire("spine", "the spine", &operands[profiles])?;
    let mut binormal = Vec3::ZERO;
    let mut contact = false;
    let mode = match opts.mode {
        PipeMode::CorrectedFrenet => 0,
        PipeMode::Frenet => 1,
        PipeMode::Fixed { binormal: b } => {
            binormal = unit("mode", b)?;
            2
        }
        PipeMode::Binormal(b) => {
            binormal = unit("mode", b)?;
            3
        }
        PipeMode::Guide { contact: c, .. } => {
            check_wire("guide", "the guide", &operands[profiles + 1])?;
            contact = c;
            4
        }
        PipeMode::Support { .. } => {
            let s = &operands[profiles + 1];
            if s.is_null() || s.count(SubKind::Face) == 0 {
                return Err(invalid("support", "the support must have faces"));
            }
            5
        }
    };
    let twist = match &opts.twist {
        Some(samples) => {
            if !matches!(mode, 0 | 1 | 3) {
                return Err(invalid(
                    "twist",
                    "a twist law needs the corrected Frenet, Frenet or binormal mode",
                ));
            }
            law("twist", samples, None)?
        }
        None => Vec::new(),
    };
    let scale = match &opts.scale {
        Some(samples) => {
            if profiles != 1 || operands[0].kind() == ShapeKind::Vertex {
                return Err(invalid(
                    "scale",
                    "a scale law needs exactly one wire profile",
                ));
            }
            if aux_operand.is_some() || !twist.is_empty() {
                return Err(invalid(
                    "scale",
                    "OCCT cannot combine a scale law with a guide, a support or a twist law \
                     (all three use an auxiliary spine)",
                ));
            }
            law("scale", samples, Some(MIN_SCALE))?
        }
        None => Vec::new(),
    };
    let transition = match opts.transition {
        Transition::Transformed => 0,
        Transition::RightCorner => 1,
        Transition::RoundCorner => 2,
    };
    let as_u32 = |n: usize| u32::try_from(n).unwrap_or(u32::MAX);
    Ok(PipeIn {
        profiles: as_u32(profiles),
        mode,
        binormal,
        aux: aux_operand.map_or(0, |(_, i)| as_u32(i)),
        contact,
        transition,
        solid: opts.solid,
        twist,
        scale,
    })
}

/// Validates a loft and lowers it for the shim. A wire may appear once, except that the first
/// section's wire may be repeated as the last one (a closed loft).
pub(crate) fn lower_thru_sections(
    wires: &[Shape],
    sections: &[LoftSection],
    opts: &ThruSectionsOpts,
) -> KResult<LoftIn> {
    if sections.len() < 2 {
        return Err(invalid("sections", "a loft needs at least two sections"));
    }
    if opts.max_degree > MAX_DEGREE {
        return Err(invalid(
            "opts",
            format!(
                "max_degree must be 0 (OCCT's default) to {MAX_DEGREE}, got {}",
                opts.max_degree
            ),
        ));
    }
    for (i, w) in wires.iter().enumerate() {
        check_wire("wires", &format!("wire {i}"), w)?;
    }
    let last = sections.len() - 1;
    let mut used = vec![false; wires.len()];
    let mut out = Vec::with_capacity(sections.len());
    let mut wire_sections = 0;
    for (k, s) in sections.iter().enumerate() {
        match *s {
            LoftSection::Point(p) => {
                if k != 0 && k != last {
                    return Err(invalid(
                        "sections",
                        format!("section {k}: only the first and last sections may be points"),
                    ));
                }
                if !p.is_finite() {
                    return Err(invalid(
                        "sections",
                        format!("section {k}: the point is not finite"),
                    ));
                }
                out.push((POINT_SECTION, p));
            }
            LoftSection::Wire(i) => {
                if i >= wires.len() {
                    return Err(invalid(
                        "sections",
                        format!(
                            "section {k}: wire {i} is out of range ({} wires)",
                            wires.len()
                        ),
                    ));
                }
                let closes = k == last && sections[0] == LoftSection::Wire(i) && last >= 2;
                if used[i] && !closes {
                    return Err(invalid(
                        "sections",
                        format!(
                            "section {k}: wire {i} is used twice (only the first wire may be \
                             repeated last, to close the loft)"
                        ),
                    ));
                }
                used[i] = true;
                wire_sections += 1;
                out.push((u32::try_from(i).unwrap_or(u32::MAX), Point3::ZERO));
            }
        }
    }
    if wire_sections == 0 {
        return Err(invalid("sections", "a loft needs a wire section"));
    }
    Ok(LoftIn {
        sections: out,
        solid: opts.solid,
        ruled: opts.ruled,
        check_compatibility: opts.check_compatibility,
        smoothing: opts.smoothing,
        max_degree: opts.max_degree,
    })
}

/// The error of a failed or cancelled sweep: `Cancelled`, `InvalidInput { what }` for a shim
/// error starting with `invalid: `, else `OpFailed` with OCCT's reason.
pub(crate) fn failure(op: &'static str, what: &'static str, status: &BoolStatus) -> KernelError {
    if status.cancelled {
        return KernelError::Cancelled;
    }
    if let Some(reason) = status
        .errors
        .iter()
        .find_map(|e| e.strip_prefix("invalid: "))
    {
        return invalid(what, reason);
    }
    let message = if status.errors.is_empty() {
        "NotDone".to_owned()
    } else {
        status.errors.join("; ")
    };
    KernelError::OpFailed {
        op,
        message,
        offending: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn laws_are_checked_and_flattened() {
        assert_eq!(
            law("twist", &[(0.0, 0.0), (1.0, 1.5)], None).unwrap(),
            vec![0.0, 0.0, 1.0, 1.5]
        );
        assert_eq!(
            law(
                "scale",
                &[(1e-13, 1.0), (0.5, 3.0), (1.0, 2.0)],
                Some(MIN_SCALE)
            )
            .unwrap(),
            vec![0.0, 1.0, 0.5, 3.0, 1.0, 2.0]
        );
        assert!(law("twist", &[(0.0, 0.0)], None).is_err());
        assert!(law("twist", &[(0.0, 0.0), (0.5, 1.0)], None).is_err());
        assert!(law("twist", &[(0.1, 0.0), (1.0, 1.0)], None).is_err());
        assert!(
            law(
                "twist",
                &[(0.0, 0.0), (0.5, 1.0), (0.5, 2.0), (1.0, 0.0)],
                None
            )
            .is_err()
        );
        assert!(law("twist", &[(0.0, f64::NAN), (1.0, 1.0)], None).is_err());
        assert!(law("scale", &[(0.0, 1.0), (1.0, 0.0)], Some(MIN_SCALE)).is_err());
    }

    #[test]
    fn failures_map_to_op_failed_invalid_input_or_cancelled() {
        let status = |cancelled, errors: &[&str]| BoolStatus {
            cancelled,
            errors: errors.iter().map(|e| (*e).to_owned()).collect(),
            warnings: Vec::new(),
            offending: Vec::new(),
        };
        assert!(matches!(
            failure("pipe", "operands", &status(true, &[])),
            KernelError::Cancelled
        ));
        assert!(matches!(
            failure("pipe", "spine", &status(false, &["invalid: no wire"])),
            KernelError::InvalidInput { what: "spine", .. }
        ));
        match failure(
            "thru_sections",
            "sections",
            &status(false, &["Standard_Failure: x"]),
        ) {
            KernelError::OpFailed { op, message, .. } => {
                assert_eq!(op, "thru_sections");
                assert_eq!(message, "Standard_Failure: x");
            }
            other => panic!("{other:?}"),
        }
        assert!(matches!(
            failure("pipe", "operands", &status(false, &[])),
            KernelError::OpFailed { .. }
        ));
    }
}
