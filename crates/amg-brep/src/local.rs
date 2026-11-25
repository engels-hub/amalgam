//! Draft, defeature, replace face, split faces and offset faces (S02 §5, BREP-16): input
//! validation and lowering in plain Rust (so the unit tests run without OCCT); the OCCT calls
//! are in `backend/occt/local.rs`, `cpp/occt_local.cpp` and (offset faces) `cpp/occt_offset.cpp`.
//! Shim failures map through [`crate::offset::failure`].

use std::collections::BTreeSet;
use std::f64::consts::{FRAC_PI_2, TAU};

use amg_core::{Aabb3, Plane, Tolerance, Vec3};

use crate::error::{KResult, KernelError};
use crate::geom::{FaceBounds, Surface};
use crate::offset::{OffsetIn, OffsetMode, face_sub};
use crate::opts::DraftOpts;
use crate::shape::{FaceId, Shape, SubKind};

/// One lowered draft (`OcDraftParams` plus the faces).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct DraftIn {
    pub faces: Vec<u32>,
    pub pull: Vec3,
    pub angle: f64,
    pub neutral: Plane,
}

fn invalid(what: &'static str, reason: impl Into<String>, face: Option<FaceId>) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason: reason.into(),
        offending: face.map(face_sub).into_iter().collect(),
    }
}

/// A non-empty list of distinct faces of `s`, each in range.
pub(crate) fn face_list(what: &'static str, s: &Shape, faces: &[FaceId]) -> KResult<Vec<u32>> {
    if s.is_null() || s.count(SubKind::Face) == 0 {
        return Err(invalid("shape", "the shape has no faces", None));
    }
    if faces.is_empty() {
        return Err(invalid(what, "no face is given", None));
    }
    let n = s.count(SubKind::Face);
    let mut seen = BTreeSet::new();
    for &f in faces {
        if f.0 >= n {
            return Err(invalid(
                what,
                format!("face {} is out of range (the shape has {n} faces)", f.0),
                None,
            ));
        }
        if !seen.insert(f) {
            return Err(invalid(
                what,
                format!("face {} is listed twice", f.0),
                Some(f),
            ));
        }
    }
    Ok(faces.iter().map(|f| f.0).collect())
}

fn has_solids(s: &Shape, op: &str) -> KResult<()> {
    if s.solids().is_empty() {
        return Err(invalid(
            "shape",
            format!("{op} needs a solid, the shape has none"),
            None,
        ));
    }
    Ok(())
}

/// Lowers a draft: distinct faces in range, an angle in (−π/2, π/2), a valid neutral frame.
pub(crate) fn lower_draft(s: &Shape, faces: &[FaceId], opts: &DraftOpts) -> KResult<DraftIn> {
    let faces = face_list("faces", s, faces)?;
    if !opts.angle.is_finite() || opts.angle.abs() >= FRAC_PI_2 {
        return Err(invalid(
            "opts",
            format!(
                "the draft angle must be finite and within (−π/2, π/2) rad, got {}",
                opts.angle
            ),
            None,
        ));
    }
    crate::ops::check_frame(&opts.neutral).map_err(|_| {
        invalid(
            "opts",
            "the neutral plane must be finite, orthonormal and right-handed",
            None,
        )
    })?;
    let pull = opts.pull.get();
    if pull.dot(opts.neutral.normal.get()).abs() < 1e-9 {
        return Err(invalid(
            "opts",
            "the pull direction lies in the neutral plane",
            None,
        ));
    }
    Ok(DraftIn {
        faces,
        pull,
        angle: opts.angle,
        neutral: opts.neutral,
    })
}

/// Lowers a defeature: distinct faces of a solid, not all of them.
pub(crate) fn lower_defeature(s: &Shape, faces: &[FaceId]) -> KResult<Vec<u32>> {
    let list = face_list("faces", s, faces)?;
    has_solids(s, "defeature")?;
    if list.len() as u32 == s.count(SubKind::Face) {
        return Err(invalid("faces", "every face is removed", None));
    }
    Ok(list)
}

/// Lowers an offset of single faces (R-S04-5): distinct faces, each with a finite offset longer
/// than the linear tolerance; the rest stay where they are.
pub(crate) fn lower_offset_faces(
    s: &Shape,
    faces: &[(FaceId, f64)],
    tol: &Tolerance,
) -> KResult<OffsetIn> {
    let ids: Vec<FaceId> = faces.iter().map(|(f, _)| *f).collect();
    let list = face_list("faces", s, &ids)?;
    let mut offsets = Vec::with_capacity(faces.len());
    for &(f, d) in faces {
        if !d.is_finite() || d.abs() <= tol.linear {
            return Err(invalid(
                "faces",
                format!(
                    "the offset of face {} must be finite and longer than {:e} mm, got {d}",
                    f.0, tol.linear
                ),
                Some(f),
            ));
        }
        offsets.push(d);
    }
    Ok(OffsetIn {
        mode: OffsetMode::Faces,
        offset: 0.0,
        tol: tol.linear,
        join: 1,
        remove: Vec::new(),
        faces: list,
        offsets,
    })
}

/// Checks a replace-face call: a solid, a face in range, a surface the kernel can build.
pub(crate) fn check_replace(s: &Shape, face: FaceId, with: &Surface) -> KResult<()> {
    face_list("face", s, &[face])?;
    has_solids(s, "replace_face")?;
    if matches!(with, Surface::Other { .. }) {
        return Err(invalid(
            "with",
            "the new surface must be a plane, cylinder, cone, sphere, torus or B-spline",
            None,
        ));
    }
    Ok(())
}

/// The parameter box of the tool face on `with` for a body in `bbox`: wide enough (twice the
/// box diagonal each way of the box centre) to cross the whole body; closed surfaces and
/// B-splines take their natural domain.
pub(crate) fn tool_bounds(with: &Surface, bbox: &Aabb3) -> KResult<FaceBounds> {
    let centre = bbox.min + (bbox.max - bbox.min) * 0.5;
    let half = 2.0 * (bbox.max - bbox.min).length().max(1.0);
    if !centre.is_finite() || !half.is_finite() {
        return Err(invalid(
            "shape",
            "the shape has no finite bounding box",
            None,
        ));
    }
    Ok(match with {
        Surface::Plane(p) => {
            let d = centre - p.origin;
            let (u, v) = (d.dot(p.x.get()), d.dot(p.y.get()));
            FaceBounds::UvRect {
                u0: u - half,
                u1: u + half,
                v0: v - half,
                v1: v + half,
            }
        }
        Surface::Cylinder { frame, .. } => {
            let v = (centre - frame.origin).dot(frame.normal.get());
            FaceBounds::UvRect {
                u0: 0.0,
                u1: TAU,
                v0: v - half,
                v1: v + half,
            }
        }
        Surface::Cone {
            frame,
            ref_radius,
            half_angle,
        } => {
            // Points are `origin + (r + v sin a) dir(u) + v cos a normal`; stay short of the apex.
            let (sin, cos) = half_angle.sin_cos();
            let v = (centre - frame.origin).dot(frame.normal.get()) / cos;
            let reach = half / cos.abs();
            let (mut v0, mut v1) = (v - reach, v + reach);
            if sin.abs() > 1e-12 {
                let apex = -ref_radius / sin;
                if sin > 0.0 {
                    v0 = v0.max(apex + 1e-3 * reach);
                } else {
                    v1 = v1.min(apex - 1e-3 * reach);
                }
            }
            if v0 >= v1 {
                return Err(invalid(
                    "with",
                    "the cone's apex lies on the far side of the whole body",
                    None,
                ));
            }
            FaceBounds::UvRect {
                u0: 0.0,
                u1: TAU,
                v0,
                v1,
            }
        }
        _ => FaceBounds::Natural,
    })
}

/// How far past the new surface the face is pushed before the trim: 5 % of the reach plus 1 %
/// of the body's size, so the extended body crosses the surface everywhere.
pub(crate) fn overshoot(reach: f64, bbox: &Aabb3, tol: &Tolerance) -> f64 {
    let size = (bbox.max - bbox.min).length();
    reach + 0.05 * reach + (0.01 * size).max(10.0 * tol.linear)
}

#[cfg(test)]
mod tests {
    use amg_core::Point3;

    use super::*;

    fn bbox() -> Aabb3 {
        Aabb3 {
            min: Point3::new(0.0, 0.0, 0.0),
            max: Point3::new(20.0, 16.0, 12.0),
        }
    }

    #[test]
    fn tool_bounds_cover_the_body() {
        let b = bbox();
        let diag = (b.max - b.min).length();
        match tool_bounds(&Surface::Plane(Plane::XY), &b).unwrap() {
            FaceBounds::UvRect { u0, u1, v0, v1 } => {
                assert!(u0 < 0.0 && u1 > 20.0 && v0 < 0.0 && v1 > 16.0);
                assert!((u1 - u0 - 4.0 * diag).abs() < 1e-9);
            }
            other => panic!("{other:?}"),
        }
        let cone = Surface::Cone {
            frame: Plane::XY,
            ref_radius: 5.0,
            half_angle: 0.3,
        };
        match tool_bounds(&cone, &b).unwrap() {
            FaceBounds::UvRect { v0, .. } => assert!(5.0 + v0 * 0.3f64.sin() > 0.0),
            other => panic!("{other:?}"),
        }
        assert_eq!(
            tool_bounds(
                &Surface::Sphere {
                    frame: Plane::XY,
                    radius: 3.0
                },
                &b
            )
            .unwrap(),
            FaceBounds::Natural
        );
        let tol = Tolerance::DEFAULT;
        assert!(overshoot(3.0, &b, &tol) > 3.0);
        assert!(overshoot(0.0, &b, &tol) > 0.0);
    }

    #[test]
    #[cfg(feature = "occt")]
    fn draft_options_are_checked() {
        use amg_core::Dir3;

        let s = Shape::test_box(1.0, 1.0, 1.0);
        let ok = DraftOpts {
            pull: Dir3::Z,
            angle: 0.05,
            neutral: Plane::XY,
        };
        assert!(lower_draft(&s, &[FaceId(0)], &ok).is_ok());
        assert!(lower_draft(&s, &[], &ok).is_err());
        assert!(lower_draft(&s, &[FaceId(0), FaceId(0)], &ok).is_err());
        assert!(lower_draft(&s, &[FaceId(6)], &ok).is_err());
        for angle in [FRAC_PI_2, -2.0, f64::NAN] {
            let bad = DraftOpts {
                angle,
                ..ok.clone()
            };
            assert!(lower_draft(&s, &[FaceId(0)], &bad).is_err(), "{angle}");
        }
        let flat = DraftOpts {
            pull: Dir3::X,
            ..ok.clone()
        };
        assert!(lower_draft(&s, &[FaceId(0)], &flat).is_err());
        let tol = Tolerance::DEFAULT;
        assert!(lower_offset_faces(&s, &[(FaceId(1), 2.0)], &tol).is_ok());
        assert!(lower_offset_faces(&s, &[(FaceId(1), 0.0)], &tol).is_err());
        assert!(lower_defeature(&s, &[FaceId(0)]).is_ok());
        let all: Vec<FaceId> = (0..6).map(FaceId).collect();
        assert!(lower_defeature(&s, &all).is_err());
    }
}
