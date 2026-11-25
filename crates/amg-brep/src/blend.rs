//! Fillets and chamfers (S02 §5, BREP-06): input validation, spec lowering, the chord-length to
//! radius conversion and the conversion of the shim's status, in plain Rust (so the unit tests run
//! without OCCT); the OCCT calls are in `backend/occt/blend.rs` and `cpp/occt_blend.cpp`.

use std::collections::BTreeSet;
use std::f64::consts::FRAC_PI_2;

use amg_core::Tolerance;

use crate::boolean::BoolStatus;
use crate::error::{KResult, KernelError};
use crate::opts::{ChamferSpec, FilletOpts, FilletShape, FilletSpec};
use crate::shape::{EdgeId, FaceId, Shape, SubKind, SubShape};

/// Variable-radius law parameters must start at 0 and end at 1 within this.
const LAW_ENDS_EPS: f64 = 1e-9;

/// Below this dihedral angle (rad) the two faces of an edge are tangent: no chord conversion.
const MIN_CHORD_ANGLE: f64 = 1e-6;

/// The shim's spec kinds (`OcBlendEdge::kind`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum BlendKind {
    FilletConstant = 0,
    FilletVariable = 1,
    ChamferEqual = 2,
    ChamferTwoDistances = 3,
    ChamferDistanceAngle = 4,
}

/// One lowered edge spec (`OcBlendEdge`, with its own law).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BlendEdge {
    pub edge: u32,
    pub kind: BlendKind,
    pub a: f64,
    pub b: f64,
    pub face: u32,
    /// `(t, r)` pairs, flattened.
    pub law: Vec<f64>,
}

/// Lowered fillet options (`OcFilletParams`; 0 = OCCT default).
#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct FilletParams {
    pub shape: u8,
    pub tol_app_3d: f64,
    pub fleche: f64,
}

/// What a blend run gave: the result with its history, or the status of a failed run with
/// OCCT's partial result (when it has one).
pub(crate) type BlendOutcome<T> = Result<T, (BoolStatus, Option<Shape>)>;

fn invalid(what: &'static str, reason: impl Into<String>, edge: Option<EdgeId>) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason: reason.into(),
        offending: edge.map(edge_sub).into_iter().collect(),
    }
}

pub(crate) fn edge_sub(e: EdgeId) -> SubShape {
    SubShape {
        kind: SubKind::Edge,
        index: e.0,
    }
}

fn positive(what: &'static str, name: &str, v: f64, e: EdgeId, tol: &Tolerance) -> KResult<()> {
    if !v.is_finite() || v <= tol.linear {
        return Err(invalid(
            what,
            format!(
                "{name} of edge {} must be finite and larger than {:e} mm, got {v}",
                e.0, tol.linear
            ),
            Some(e),
        ));
    }
    Ok(())
}

/// The shape must be non-null with faces; every edge must be in range, listed once, and have
/// exactly two distinct faces. Returns each edge's two faces.
pub(crate) fn check_edges(s: &Shape, edges: &[EdgeId]) -> KResult<Vec<[FaceId; 2]>> {
    if s.is_null() || s.count(SubKind::Face) == 0 {
        return Err(invalid("shape", "the shape has no faces", None));
    }
    if edges.is_empty() {
        return Err(invalid("edges", "no edges to blend", None));
    }
    let n = s.count(SubKind::Edge);
    let mut seen = BTreeSet::new();
    let mut out = Vec::with_capacity(edges.len());
    for &e in edges {
        if e.0 >= n {
            return Err(invalid(
                "edges",
                format!("edge {} is out of range (the shape has {n} edges)", e.0),
                None,
            ));
        }
        if !seen.insert(e) {
            return Err(invalid(
                "edges",
                format!("edge {} is listed twice", e.0),
                Some(e),
            ));
        }
        match *s.edge_faces(e) {
            [f1, f2] if f1 != f2 => out.push([f1, f2]),
            _ => {
                return Err(invalid(
                    "edges",
                    format!(
                        "edge {} is not between two faces (it has {} faces, or is a seam)",
                        e.0,
                        s.edge_faces(e).len()
                    ),
                    Some(e),
                ));
            }
        }
    }
    Ok(out)
}

/// The constant radius of a circular fillet whose chord (between its two tangent lines on the
/// faces) is `chord`, where the faces' outward normals are `angle` rad apart: the arc spans
/// `angle`, so `chord = 2 r sin(angle / 2)`.
pub(crate) fn chord_radius(chord: f64, angle: f64) -> f64 {
    chord / (2.0 * (0.5 * angle).sin())
}

fn check_law(law: &[(f64, f64)], e: EdgeId, tol: &Tolerance) -> KResult<Vec<f64>> {
    let bad = |reason: String| Err(invalid("edges", reason, Some(e)));
    if law.len() < 2 {
        return bad(format!(
            "the radius law of edge {} needs at least 2 samples",
            e.0
        ));
    }
    let (first, last) = (law[0].0, law[law.len() - 1].0);
    if first.abs() > LAW_ENDS_EPS || (last - 1.0).abs() > LAW_ENDS_EPS {
        return bad(format!(
            "the radius law of edge {} must run from t = 0 to t = 1, got {first} to {last}",
            e.0
        ));
    }
    let mut flat = Vec::with_capacity(2 * law.len());
    let mut prev = f64::NEG_INFINITY;
    for (i, &(t, r)) in law.iter().enumerate() {
        if !t.is_finite() || t <= prev {
            return bad(format!(
                "the law parameters of edge {} must increase strictly",
                e.0
            ));
        }
        prev = t;
        positive("edges", &format!("law radius {i}"), r, e, tol)?;
        // Snap the ends exactly onto 0 and 1.
        let t = if i == 0 {
            0.0
        } else if i + 1 == law.len() {
            1.0
        } else {
            t
        };
        flat.extend([t, r]);
    }
    Ok(flat)
}

/// Lowers fillet specs; `angle(e, f1, f2)` is the dihedral angle the chord conversion needs.
pub(crate) fn lower_fillet(
    s: &Shape,
    edges: &[(EdgeId, FilletSpec)],
    tol: &Tolerance,
    mut angle: impl FnMut(EdgeId, FaceId, FaceId) -> KResult<f64>,
) -> KResult<Vec<BlendEdge>> {
    let ids: Vec<EdgeId> = edges.iter().map(|(e, _)| *e).collect();
    let faces = check_edges(s, &ids)?;
    edges
        .iter()
        .zip(faces)
        .map(|((e, spec), [f1, f2])| {
            let e = *e;
            let constant = |r: f64| BlendEdge {
                edge: e.0,
                kind: BlendKind::FilletConstant,
                a: r,
                b: 0.0,
                face: 0,
                law: Vec::new(),
            };
            match spec {
                FilletSpec::Constant(r) => {
                    positive("edges", "the radius", *r, e, tol)?;
                    Ok(constant(*r))
                }
                FilletSpec::Variable(law) => Ok(BlendEdge {
                    edge: e.0,
                    kind: BlendKind::FilletVariable,
                    a: 0.0,
                    b: 0.0,
                    face: 0,
                    law: check_law(law, e, tol)?,
                }),
                FilletSpec::Chord(c) => {
                    positive("edges", "the chord length", *c, e, tol)?;
                    let phi = angle(e, f1, f2)?;
                    if !phi.is_finite() || phi < MIN_CHORD_ANGLE {
                        return Err(invalid(
                            "edges",
                            format!(
                                "the faces of edge {} are tangent there: a chord length gives no \
                                 radius",
                                e.0
                            ),
                            Some(e),
                        ));
                    }
                    Ok(constant(chord_radius(*c, phi)))
                }
            }
        })
        .collect()
}

/// Lowers chamfer specs: the reference face of an asymmetric chamfer must be one of the edge's
/// two faces, and a distance-angle chamfer's angle must be in `(0, π/2)`.
pub(crate) fn lower_chamfer(
    s: &Shape,
    edges: &[(EdgeId, ChamferSpec)],
    tol: &Tolerance,
) -> KResult<Vec<BlendEdge>> {
    let ids: Vec<EdgeId> = edges.iter().map(|(e, _)| *e).collect();
    let faces = check_edges(s, &ids)?;
    let on_edge = |e: EdgeId, f: FaceId, adjacent: [FaceId; 2]| {
        if adjacent.contains(&f) {
            Ok(f.0)
        } else {
            Err(invalid(
                "edges",
                format!(
                    "face {} is not a face of edge {} (its faces are {} and {})",
                    f.0, e.0, adjacent[0].0, adjacent[1].0
                ),
                Some(e),
            ))
        }
    };
    edges
        .iter()
        .zip(faces)
        .map(|((e, spec), adjacent)| {
            let e = *e;
            let (kind, a, b, face) = match *spec {
                ChamferSpec::Equal(d) => {
                    positive("edges", "the distance", d, e, tol)?;
                    (BlendKind::ChamferEqual, d, 0.0, 0)
                }
                ChamferSpec::TwoDistances { d1, d2, face } => {
                    positive("edges", "d1", d1, e, tol)?;
                    positive("edges", "d2", d2, e, tol)?;
                    (
                        BlendKind::ChamferTwoDistances,
                        d1,
                        d2,
                        on_edge(e, face, adjacent)?,
                    )
                }
                ChamferSpec::DistanceAngle { d, angle, face } => {
                    positive("edges", "the distance", d, e, tol)?;
                    if !angle.is_finite()
                        || angle <= tol.angular
                        || angle >= FRAC_PI_2 - tol.angular
                    {
                        return Err(invalid(
                            "edges",
                            format!(
                                "the chamfer angle of edge {} must be in (0, π/2) rad, got {angle}",
                                e.0
                            ),
                            Some(e),
                        ));
                    }
                    (
                        BlendKind::ChamferDistanceAngle,
                        d,
                        angle,
                        on_edge(e, face, adjacent)?,
                    )
                }
            };
            Ok(BlendEdge {
                edge: e.0,
                kind,
                a,
                b,
                face,
                law: Vec::new(),
            })
        })
        .collect()
}

/// Lowers `FilletOpts`: approximation overrides must be finite and positive.
pub(crate) fn lower_opts(opts: &FilletOpts) -> KResult<FilletParams> {
    let positive_or_default = |name: &'static str, v: Option<f64>| match v {
        None => Ok(0.0),
        Some(v) if v.is_finite() && v > 0.0 => Ok(v),
        Some(v) => Err(invalid(
            "opts",
            format!("{name} must be finite and positive, got {v}"),
            None,
        )),
    };
    Ok(FilletParams {
        shape: match opts.shape {
            FilletShape::Rational => 0,
            FilletShape::QuasiAngular => 1,
            FilletShape::Polynomial => 2,
        },
        tol_app_3d: positive_or_default("tol_app_3d", opts.tol_app_3d)?,
        fleche: positive_or_default("fleche", opts.fleche)?,
    })
}

/// The error of a failed blend: `InvalidInput` for the shim's `invalid: ` errors, else
/// `LocalOpFailed` with the offending input edges and the partial result.
pub(crate) fn failure(op: &'static str, status: BoolStatus, partial: Option<Shape>) -> KernelError {
    if status.cancelled {
        return KernelError::Cancelled;
    }
    if let Some(reason) = status
        .errors
        .iter()
        .find_map(|e| e.strip_prefix("invalid: "))
    {
        return KernelError::InvalidInput {
            what: "edges",
            reason: reason.to_owned(),
            offending: status.offending.iter().map(|(_, s)| *s).collect(),
        };
    }
    let mut offending: Vec<SubShape> = status.offending.iter().map(|(_, s)| *s).collect();
    offending.sort();
    offending.dedup();
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
    use std::f64::consts::PI;

    use super::*;

    const TOL: Tolerance = Tolerance::DEFAULT;

    #[test]
    fn chord_of_a_right_angle_edge() {
        // A quarter circle of radius r has a chord of r·√2.
        let r = chord_radius(2f64.sqrt() * 3.0, FRAC_PI_2);
        assert!((r - 3.0).abs() < 1e-12, "{r}");
        // A half circle (faces folded back): chord = diameter.
        assert!((chord_radius(4.0, PI) - 2.0).abs() < 1e-12);
    }

    #[test]
    fn laws_are_checked_and_snapped() {
        let e = EdgeId(4);
        let flat = check_law(&[(1e-12, 1.0), (0.5, 2.0), (1.0 - 1e-12, 1.5)], e, &TOL).unwrap();
        assert_eq!(flat, vec![0.0, 1.0, 0.5, 2.0, 1.0, 1.5]);
        assert!(check_law(&[(0.0, 1.0)], e, &TOL).is_err());
        assert!(check_law(&[(0.1, 1.0), (1.0, 1.0)], e, &TOL).is_err());
        assert!(check_law(&[(0.0, 1.0), (0.6, 1.0), (0.5, 1.0), (1.0, 1.0)], e, &TOL).is_err());
        let err = check_law(&[(0.0, 1.0), (1.0, -1.0)], e, &TOL).unwrap_err();
        assert_eq!(err.offending(), &[edge_sub(e)]);
    }

    #[test]
    fn options_lower_to_the_shim_codes() {
        let p = lower_opts(&FilletOpts::default()).unwrap();
        assert_eq!((p.shape, p.tol_app_3d, p.fleche), (0, 0.0, 0.0));
        let p = lower_opts(&FilletOpts {
            shape: FilletShape::Polynomial,
            tol_app_3d: Some(1e-5),
            fleche: None,
        })
        .unwrap();
        assert_eq!((p.shape, p.tol_app_3d), (2, 1e-5));
        assert!(
            lower_opts(&FilletOpts {
                fleche: Some(-1.0),
                ..FilletOpts::default()
            })
            .is_err()
        );
    }

    #[test]
    fn failures_carry_sorted_edges_or_become_invalid_input() {
        let status = BoolStatus {
            errors: vec!["contour 1: ChFiDS_StartsolFailure".into()],
            offending: vec![(0, edge_sub(EdgeId(7))), (0, edge_sub(EdgeId(2)))],
            ..BoolStatus::default()
        };
        match failure("fillet", status, None) {
            KernelError::LocalOpFailed {
                op,
                message,
                offending,
                partial,
            } => {
                assert_eq!(op, "fillet");
                assert!(message.contains("StartsolFailure"));
                assert_eq!(offending, vec![edge_sub(EdgeId(2)), edge_sub(EdgeId(7))]);
                assert!(partial.is_none());
            }
            other => panic!("{other:?}"),
        }
        let status = BoolStatus {
            errors: vec!["invalid: edge 3 is on the tangent chain of edge 1".into()],
            ..BoolStatus::default()
        };
        assert!(matches!(
            failure("fillet", status, None),
            KernelError::InvalidInput { what: "edges", .. }
        ));
        let cancelled = BoolStatus {
            cancelled: true,
            ..BoolStatus::default()
        };
        assert!(matches!(
            failure("chamfer", cancelled, None),
            KernelError::Cancelled
        ));
    }
}
