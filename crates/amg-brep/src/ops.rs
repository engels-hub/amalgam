//! Primitives, prism, revol and affine transforms (S02 §5, BREP-14): input validation and the
//! history conversion in plain Rust (so the unit tests run without OCCT); the OCCT calls are in
//! `backend/occt/ops.rs`.

use std::f64::consts::{FRAC_PI_2, TAU};

use amg_core::{Line3, Mat3, Plane, Point3, Tolerance, Transform, Vec3};
use smallvec::SmallVec;

use crate::error::{KResult, KernelError};
use crate::geom::Surface;
use crate::history::{InputSub, OpRole, Relation, ShapeHistory};
use crate::opts::{PrismExtent, PrismOpts};
use crate::shape::{FaceId, Shape, ShapeKind, SubKind, SubShape};

/// A frame is accepted when its axes are unit, orthogonal and right-handed within this.
const FRAME_EPS: f64 = 1e-9;

/// Sweep angles up to `2π` plus this are clamped to a full turn.
const FULL_TURN_SLACK: f64 = 1e-12;

/// Largest accepted `|taper|`: the side faces must not lie in the profile plane.
const MAX_TAPER: f64 = FRAC_PI_2 - 1e-6;

/// The shim's primitive kinds (`make_primitive`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum PrimitiveKind {
    Box = 0,
    Cylinder = 1,
    Cone = 2,
    Sphere = 3,
    Torus = 4,
}

/// How a prism extends, for the shim (`prism`).
#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) struct PrismIn {
    /// The sweep vector (finite), or the unit direction (infinite).
    pub vec: Vec3,
    /// 0 finite, 1 infinite one way, 2 infinite both ways.
    pub mode: u8,
    /// rad, positive grows the profile along the sweep.
    pub taper: f64,
    pub merge_coplanar: bool,
}

/// An op history as the shim reports it (see `amg_occt_sys::ffi::OcOpHistory`).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct RawHistory {
    pub in_kinds: Vec<u8>,
    pub in_indices: Vec<u32>,
    pub relations: Vec<u8>,
    pub out_starts: Vec<u32>,
    pub out_kinds: Vec<u8>,
    pub out_indices: Vec<u32>,
    pub role_faces: Vec<u32>,
    pub roles: Vec<u8>,
}

fn invalid(what: &'static str, reason: impl Into<String>) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason: reason.into(),
        offending: Vec::new(),
    }
}

/// `frame` must have unit, orthogonal axes with `normal = x × y`, and a finite origin.
pub(crate) fn check_frame(frame: &Plane) -> KResult<()> {
    let (x, y, n) = (frame.x.get(), frame.y.get(), frame.normal.get());
    let unit = [x, y, n]
        .iter()
        .all(|v| v.is_finite() && (v.length() - 1.0).abs() <= FRAME_EPS);
    let orthogonal =
        x.dot(y).abs() <= FRAME_EPS && x.dot(n).abs() <= FRAME_EPS && y.dot(n).abs() <= FRAME_EPS;
    let right_handed = (x.cross(y) - n).length() <= FRAME_EPS;
    if !frame.origin.is_finite() || !unit || !orthogonal || !right_handed {
        return Err(invalid(
            "frame",
            "the frame must be finite, orthonormal and right-handed (normal = x × y)",
        ));
    }
    Ok(())
}

fn positive(what: &'static str, name: &str, value: f64, tol: &Tolerance) -> KResult<()> {
    if !value.is_finite() || value <= tol.linear {
        return Err(invalid(
            what,
            format!(
                "{name} must be finite and larger than {:e} mm, got {value}",
                tol.linear
            ),
        ));
    }
    Ok(())
}

/// A sweep angle in `(0, 2π]`; values just above `2π` become `2π`.
pub(crate) fn sweep_angle(what: &'static str, angle: f64, tol: &Tolerance) -> KResult<f64> {
    if !angle.is_finite() || angle <= tol.angular || angle > TAU + FULL_TURN_SLACK {
        return Err(invalid(
            what,
            format!("the angle must be in (0, 2π] rad, got {angle}"),
        ));
    }
    Ok(angle.min(TAU))
}

/// The shim parameters of a primitive, validated.
pub(crate) fn box_params(frame: &Plane, size: Vec3, tol: &Tolerance) -> KResult<[f64; 3]> {
    check_frame(frame)?;
    positive("size", "the box size x", size.x, tol)?;
    positive("size", "the box size y", size.y, tol)?;
    positive("size", "the box size z", size.z, tol)?;
    Ok(size.to_array())
}

pub(crate) fn cylinder_params(
    frame: &Plane,
    radius: f64,
    height: f64,
    angle: f64,
    tol: &Tolerance,
) -> KResult<[f64; 3]> {
    check_frame(frame)?;
    positive("radius", "the radius", radius, tol)?;
    positive("height", "the height", height, tol)?;
    Ok([radius, height, sweep_angle("angle", angle, tol)?])
}

pub(crate) fn cone_params(
    frame: &Plane,
    r1: f64,
    r2: f64,
    height: f64,
    angle: f64,
    tol: &Tolerance,
) -> KResult<[f64; 4]> {
    check_frame(frame)?;
    for (name, r) in [("r1", r1), ("r2", r2)] {
        if !r.is_finite() || r < 0.0 {
            return Err(invalid(
                "radius",
                format!("{name} must be finite and ≥ 0, got {r}"),
            ));
        }
    }
    if (r1 - r2).abs() <= tol.linear {
        return Err(invalid(
            "radius",
            "r1 and r2 are equal: that is a cylinder (make_cylinder)",
        ));
    }
    positive("height", "the height", height, tol)?;
    Ok([r1, r2, height, sweep_angle("angle", angle, tol)?])
}

pub(crate) fn sphere_params(frame: &Plane, radius: f64, tol: &Tolerance) -> KResult<[f64; 1]> {
    check_frame(frame)?;
    positive("radius", "the radius", radius, tol)?;
    Ok([radius])
}

pub(crate) fn torus_params(
    frame: &Plane,
    major: f64,
    minor: f64,
    angle: f64,
    tol: &Tolerance,
) -> KResult<[f64; 3]> {
    check_frame(frame)?;
    positive("radius", "the minor radius", minor, tol)?;
    if !major.is_finite() || major <= minor + tol.linear {
        return Err(invalid(
            "radius",
            format!(
                "the major radius must exceed the minor radius {minor} (no self-intersecting \
                 torus), got {major}"
            ),
        ));
    }
    Ok([major, minor, sweep_angle("angle", angle, tol)?])
}

/// Prism and revol take faces, shells, wires and edges, or compounds of them.
pub(crate) fn check_profile(profile: &Shape) -> KResult<()> {
    match profile.kind() {
        ShapeKind::Face
        | ShapeKind::Shell
        | ShapeKind::Wire
        | ShapeKind::Edge
        | ShapeKind::Compound => {}
        other => {
            return Err(invalid(
                "profile",
                format!("a profile is a face, shell, wire, edge or compound, not a {other:?}"),
            ));
        }
    }
    if profile.count(SubKind::Edge) == 0 {
        return Err(invalid("profile", "the profile has no edges"));
    }
    Ok(())
}

/// The plane of a one-face profile, if it is planar.
fn single_plane(profile: &Shape) -> Option<Plane> {
    if profile.kind() != ShapeKind::Face {
        return None;
    }
    match profile.surface(FaceId(0)) {
        Surface::Plane(p) => Some(p),
        _ => None,
    }
}

/// Validates a prism and lowers it for the shim.
pub(crate) fn lower_prism(
    profile: &Shape,
    extent: &PrismExtent,
    opts: &PrismOpts,
    tol: &Tolerance,
) -> KResult<PrismIn> {
    check_profile(profile)?;
    let (vec, mode) = match *extent {
        PrismExtent::Vector(v) => {
            if !v.is_finite() || v.length() <= tol.linear {
                return Err(invalid(
                    "extent",
                    format!(
                        "the prism vector must be finite and longer than {:e} mm",
                        tol.linear
                    ),
                ));
            }
            (v, 0)
        }
        PrismExtent::Infinite { dir, both } => {
            let d = dir.get();
            if !d.is_finite() || (d.length() - 1.0).abs() > FRAME_EPS {
                return Err(invalid(
                    "extent",
                    "the prism direction must be a unit vector",
                ));
            }
            (d, if both { 2 } else { 1 })
        }
    };
    // A sweep in the plane of a planar face would give a flat, degenerate solid.
    if let Some(plane) = single_plane(profile) {
        let along = vec.normalize().dot(plane.normal.get()).abs();
        if along <= tol.angular {
            return Err(invalid(
                "extent",
                "the prism direction lies in the profile plane",
            ));
        }
    }
    let taper = opts.taper;
    if !taper.is_finite() || taper.abs() > MAX_TAPER {
        return Err(invalid(
            "taper",
            format!("the taper must be finite and within ±(π/2 − 1e-6) rad, got {taper}"),
        ));
    }
    if taper != 0.0 {
        if mode != 0 {
            return Err(invalid("taper", "an infinite prism cannot be tapered"));
        }
        if single_plane(profile).is_none() {
            return Err(invalid(
                "taper",
                "a tapered prism needs one planar face as its profile",
            ));
        }
    }
    Ok(PrismIn {
        vec,
        mode,
        taper,
        merge_coplanar: opts.merge_coplanar,
    })
}

/// Validates a revolution; returns the axis point, unit axis direction and the angle.
pub(crate) fn lower_revol(
    profile: &Shape,
    axis: &Line3,
    angle: f64,
    tol: &Tolerance,
) -> KResult<(Point3, Vec3, f64)> {
    check_profile(profile)?;
    let d = axis.dir.get();
    if !axis.point.is_finite() || !d.is_finite() || (d.length() - 1.0).abs() > FRAME_EPS {
        return Err(invalid(
            "axis",
            "the axis must be finite with a unit direction",
        ));
    }
    let angle = sweep_angle("angle", angle, tol)?;
    Ok((axis.point, d, angle))
}

/// `t` must be finite and invertible.
pub(crate) fn check_affine(t: &Transform) -> KResult<()> {
    let finite =
        t.matrix3.to_cols_array().iter().all(|v| v.is_finite()) && t.translation.is_finite();
    if !finite {
        return Err(invalid("transform", "the transform is not finite"));
    }
    let m = t.matrix3;
    let scale = m.x_axis.length() * m.y_axis.length() * m.z_axis.length();
    if scale == 0.0 || m.determinant().abs() <= 1e-12 * scale {
        return Err(invalid("transform", "the transform is singular"));
    }
    Ok(())
}

/// The reflection about `plane`.
pub(crate) fn reflection(plane: &Plane) -> Transform {
    let n = plane.normal.get();
    let linear = Mat3::IDENTITY - 2.0 * Mat3::from_cols(n * n.x, n * n.y, n * n.z);
    let offset = 2.0 * plane.origin.dot(n) * n;
    Transform::from_mat3_translation(linear, offset)
}

fn sub(kind: u8, index: u32) -> SubShape {
    let kind = match kind {
        0 => SubKind::Face,
        1 => SubKind::Edge,
        _ => SubKind::Vertex,
    };
    SubShape { kind, index }
}

/// The `ShapeHistory` of a one-operand op from the shim's flat form.
pub(crate) fn history(raw: &RawHistory, operand_counts: [u32; 3]) -> ShapeHistory {
    let mut entries: Vec<(InputSub, Relation)> = Vec::with_capacity(raw.in_kinds.len());
    for i in 0..raw.in_kinds.len() {
        let input = InputSub {
            operand: 0,
            sub: sub(raw.in_kinds[i], raw.in_indices[i]),
        };
        let range = raw.out_starts[i] as usize..raw.out_starts[i + 1] as usize;
        let mut outs: SmallVec<[SubShape; 2]> = range
            .map(|k| sub(raw.out_kinds[k], raw.out_indices[k]))
            .collect();
        outs.sort();
        let relation = match raw.relations[i] {
            0 => Relation::Modified(outs),
            1 => Relation::Generated(outs),
            _ => Relation::Deleted,
        };
        entries.push((input, relation));
    }
    // Stable: Modified before Generated for one input, as the shim lists them.
    entries.sort_by_key(|(input, _)| *input);
    let mut roles: Vec<(SubShape, OpRole)> = raw
        .role_faces
        .iter()
        .zip(&raw.roles)
        .map(|(&f, &r)| {
            let role = match r {
                0 => OpRole::StartCap,
                1 => OpRole::EndCap,
                _ => OpRole::Lateral,
            };
            (sub(0, f), role)
        })
        .collect();
    roles.sort_by_key(|(s, _)| *s);
    ShapeHistory {
        entries,
        roles,
        tags: Vec::new(),
        operand_counts: vec![operand_counts],
    }
}

/// Faces, edges and vertices of `s` (`ShapeHistory::operand_counts`).
pub(crate) fn counts(s: &Shape) -> [u32; 3] {
    [
        s.count(SubKind::Face),
        s.count(SubKind::Edge),
        s.count(SubKind::Vertex),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::Dir3;

    const TOL: Tolerance = Tolerance::DEFAULT;

    #[test]
    fn frames_must_be_right_handed_and_orthonormal() {
        check_frame(&Plane::XY).unwrap();
        let mut left = Plane::XY;
        left.normal = Dir3::NEG_Z;
        assert!(check_frame(&left).is_err());
        let mut skew = Plane::XY;
        skew.y = Dir3::new_unchecked(Vec3::new(0.6, 0.8, 0.0));
        assert!(check_frame(&skew).is_err());
        let mut far = Plane::XY;
        far.origin = Point3::new(f64::NAN, 0.0, 0.0);
        assert!(check_frame(&far).is_err());
    }

    #[test]
    fn primitive_parameters_are_checked() {
        let f = Plane::XY;
        assert_eq!(
            box_params(&f, Vec3::new(1.0, 2.0, 3.0), &TOL)
                .unwrap()
                .map(f64::to_bits),
            [1.0, 2.0, 3.0].map(f64::to_bits)
        );
        assert!(box_params(&f, Vec3::new(1.0, 0.0, 3.0), &TOL).is_err());
        assert!(box_params(&f, Vec3::new(1.0, f64::INFINITY, 3.0), &TOL).is_err());
        assert_eq!(
            cylinder_params(&f, 2.0, 3.0, TAU + 1e-13, &TOL)
                .unwrap()
                .map(f64::to_bits),
            [2.0, 3.0, TAU].map(f64::to_bits)
        );
        assert!(cylinder_params(&f, 2.0, 3.0, TAU + 1e-6, &TOL).is_err());
        assert!(cylinder_params(&f, 2.0, 3.0, 0.0, &TOL).is_err());
        assert!(cylinder_params(&f, -2.0, 3.0, 1.0, &TOL).is_err());
        cone_params(&f, 0.0, 2.0, 3.0, 1.0, &TOL).unwrap();
        assert!(cone_params(&f, 2.0, 2.0, 3.0, 1.0, &TOL).is_err());
        assert!(cone_params(&f, -1.0, 2.0, 3.0, 1.0, &TOL).is_err());
        sphere_params(&f, 1.0, &TOL).unwrap();
        assert!(sphere_params(&f, 0.0, &TOL).is_err());
        torus_params(&f, 5.0, 1.0, TAU, &TOL).unwrap();
        assert!(torus_params(&f, 1.0, 1.0, TAU, &TOL).is_err());
        assert!(torus_params(&f, 5.0, 0.0, TAU, &TOL).is_err());
    }

    #[test]
    fn reflection_mirrors_points_about_the_plane() {
        let plane = Plane {
            origin: Point3::new(0.0, 0.0, 2.0),
            ..Plane::XY
        };
        let t = reflection(&plane);
        let p = t.transform_point3(Point3::new(1.0, 2.0, 5.0));
        assert!((p - Point3::new(1.0, 2.0, -1.0)).length() < 1e-15);
        assert!((t.matrix3.determinant() + 1.0).abs() < 1e-15);
        let tilted = Plane {
            origin: Point3::new(1.0, 1.0, 0.0),
            x: Dir3::new_unchecked(Vec3::new(0.0, 0.0, 1.0)),
            y: Dir3::new_unchecked(Vec3::new(-1.0, 1.0, 0.0).normalize()),
            normal: Dir3::new_unchecked(Vec3::new(-1.0, -1.0, 0.0).normalize()),
        };
        let t = reflection(&tilted);
        let q = Point3::new(3.0, 3.0, 7.0);
        let r = t.transform_point3(q);
        assert!((r - Point3::new(-1.0, -1.0, 7.0)).length() < 1e-14, "{r}");
        // Twice is the identity.
        assert!((t.transform_point3(r) - q).length() < 1e-14);
    }

    #[test]
    fn affine_maps_must_be_finite_and_invertible() {
        check_affine(&Transform::from_scale(Vec3::new(1.0, 2.0, 3.0))).unwrap();
        assert!(check_affine(&Transform::from_scale(Vec3::new(1.0, 0.0, 3.0))).is_err());
        assert!(check_affine(&Transform::from_translation(Vec3::new(f64::NAN, 0.0, 0.0))).is_err());
    }

    #[test]
    fn history_is_converted_and_sorted() {
        let raw = RawHistory {
            in_kinds: vec![1, 0, 0, 2],
            in_indices: vec![0, 0, 0, 3],
            relations: vec![1, 0, 1, 2],
            out_starts: vec![0, 1, 2, 4, 4],
            out_kinds: vec![0, 0, 0, 0],
            out_indices: vec![2, 0, 5, 1],
            role_faces: vec![1, 0, 2],
            roles: vec![1, 0, 2],
        };
        let h = history(&raw, [1, 4, 4]);
        let face0 = InputSub {
            operand: 0,
            sub: sub(0, 0),
        };
        assert_eq!(h.entries[0].0, face0);
        assert_eq!(
            h.entries[0].1,
            Relation::Modified(SmallVec::from_slice(&[sub(0, 0)]))
        );
        assert_eq!(
            h.entries[1].1,
            Relation::Generated(SmallVec::from_slice(&[sub(0, 1), sub(0, 5)]))
        );
        assert_eq!(h.entries[2].0.sub, sub(1, 0));
        assert_eq!(
            h.entries[3],
            (
                InputSub {
                    operand: 0,
                    sub: sub(2, 3)
                },
                Relation::Deleted
            )
        );
        assert_eq!(h.role_of(sub(0, 0)), Some(&OpRole::StartCap));
        assert_eq!(h.role_of(sub(0, 1)), Some(&OpRole::EndCap));
        assert_eq!(h.role_of(sub(0, 2)), Some(&OpRole::Lateral));
        assert_eq!(h.operand_counts, vec![[1, 4, 4]]);
        assert_eq!(h.sources_of(sub(0, 5)).len(), 1);
    }
}
