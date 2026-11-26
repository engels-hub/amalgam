//! The solid of one profile face on one side (note §3): a prism by a distance, up to the far
//! side of the component's bodies ("All"), or up to a target ("To object": an exact prism for
//! a parallel plane or a point, else a long prism split by the target).
//!
//! Naming per the naming note §5.2: `Lateral#side` from the profile edge tags,
//! `StartCap#side` / `EndCap#side` from the profile's cap source; the faces a target leaves on
//! a split prism are its `EndCap#side` too, so changing the extent type keeps the cap name.

use amg_brep::geom::{FaceBounds, Surface};
use amg_brep::{
    BoolOpts, BrepKernel, InputSub, OffsetJoin, OpResult, OpRole, PrismExtent, PrismOpts, Relation,
    Shape, ShapeHistory, SubKind, SubShape,
};
use amg_core::{Aabb3, Plane, Point3, Transform, Vec3};
use amg_doc::naming::{OperandNaming, Role, TopoKind};
use amg_doc::{BodyGeom, BodyNaming, EntityRef, FeatureError, FeatureOutput, RecomputeCtx};

use crate::solid::common::{finish_naming, with_job};
use crate::solid::error::{SolidError, kerr, with_hint};
use crate::solid::extent::{BodyFace, Target};
use crate::solid::profile::{ProfileFace, ProfileSource, cap_source, face_edge_source, tag_source};

/// Below this `|cos|` between a target plane and the extrude direction the plane is taken as
/// parallel to the direction (it never meets the prism).
const MIN_CROSSING: f64 = 1e-6;
/// `|cos|` above which a target plane is parallel to the profile (exact distance).
const PARALLEL: f64 = 1.0 - 1e-9;

/// One side of an extrude, evaluated.
#[derive(Clone, Debug)]
pub(super) struct Side<'a> {
    /// The naming side: 0, or 1 for the second side of two-sided and symmetric extrudes.
    pub index: u32,
    /// Unit direction.
    pub dir: Vec3,
    /// How far.
    pub extent: SideExtent<'a>,
    /// Taper, rad (positive grows the profile along the side).
    pub taper: f64,
}

/// The extent of a [`Side`], evaluated (a few per compute, so the large variant stays inline).
#[derive(Clone, Debug)]
#[allow(clippy::large_enum_variant, reason = "short-lived, a few per compute")]
pub(super) enum SideExtent<'a> {
    /// A positive length, mm.
    Length(f64),
    /// Up to a target.
    ToObject {
        /// The resolved target.
        target: Target<'a>,
        /// The stored reference (highlighted on failure).
        r: EntityRef,
        /// Offset, mm: farther along the side for planes and points, along the face's outward
        /// normal for curved faces, outwards for bodies.
        offset: f64,
        /// Use the untrimmed surface of a face target.
        extend: bool,
    },
    /// Through every solid body of the context component.
    All,
}

/// The solid of `f` (already at its start plane) on side `s`, named.
pub(super) fn side_solid(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    f: &ProfileFace,
    s: &Side<'_>,
    out: &mut FeatureOutput,
) -> Result<(Shape, BodyNaming), FeatureError> {
    let (r, naming) = match &s.extent {
        SideExtent::Length(d) => prism_named(ctx, k, f, s, *d, out)?,
        SideExtent::All => {
            let l = all_length(ctx, f, s.dir)?;
            prism_named(ctx, k, f, s, l, out)?
        }
        SideExtent::ToObject {
            target,
            r,
            offset,
            extend,
        } => return to_object(ctx, k, f, s, target, r, *offset, *extend, out),
    };
    Ok((r.shape, naming))
}

/// The prism of `f` along `s.dir` by `length`, tapered by `s.taper`, named for side `s.index`.
fn prism_named(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    f: &ProfileFace,
    s: &Side<'_>,
    length: f64,
    out: &mut FeatureOutput,
) -> Result<(OpResult, BodyNaming), FeatureError> {
    let r = with_job(ctx, |job| {
        k.prism(
            &f.face,
            &PrismExtent::Vector(s.dir * length),
            &PrismOpts {
                taper: s.taper,
                merge_coplanar: false,
            },
            job,
        )
    })
    .map_err(|e| {
        let fe = kerr(e);
        match fe {
            FeatureError::Kernel(m) if s.taper != 0.0 => {
                FeatureError::Kernel(format!("{m}. Reduce the taper angle."))
            }
            other => other,
        }
    })?;
    let mut b = ctx.naming_builder();
    b.from_history(&[(0, OperandNaming::Tagged(&f.edge_tags))], &r.history);
    match &f.source {
        ProfileSource::Region { sketch, .. } => {
            for (_, tag) in &f.edge_tags {
                b.tag(*tag, tag_source(*sketch, *tag));
            }
        }
        ProfileSource::Face { body, .. } => {
            for (_, tag) in &f.edge_tags {
                if let Some(src) = face_edge_source(ctx, *body, *tag) {
                    b.tag(*tag, src);
                }
            }
        }
    }
    if s.index != 0 {
        b.role(OpRole::Lateral, Role::new("Lateral", s.index));
    }
    let cap = cap_source(ctx, f)?;
    for (sub, role) in &r.history.roles {
        let role = match role {
            OpRole::StartCap => Role::new("StartCap", s.index),
            OpRole::EndCap => Role::new("EndCap", s.index),
            _ => continue,
        };
        if sub.kind == SubKind::Face {
            b.generated(TopoKind::Face, sub.index, role, Some(cap.clone()));
        }
    }
    let naming = finish_naming(b, k, &r.shape, out)?;
    Ok((r, naming))
}

fn corners(b: &Aabb3) -> [Point3; 8] {
    let (a, c) = (b.min, b.max);
    [
        Point3::new(a.x, a.y, a.z),
        Point3::new(c.x, a.y, a.z),
        Point3::new(a.x, c.y, a.z),
        Point3::new(c.x, c.y, a.z),
        Point3::new(a.x, a.y, c.z),
        Point3::new(c.x, a.y, c.z),
        Point3::new(a.x, c.y, c.z),
        Point3::new(c.x, c.y, c.z),
    ]
}

/// The farthest point of `b` along `dir` from `o` (−∞ for an empty box).
fn far_along(b: &Aabb3, o: Point3, dir: Vec3) -> f64 {
    if b.is_empty() {
        return f64::NEG_INFINITY;
    }
    corners(b)
        .iter()
        .map(|c| (*c - o).dot(dir))
        .fold(f64::NEG_INFINITY, f64::max)
}

/// Whether `x` is a finite distance beyond `tol` (NaN is not).
fn ahead(x: f64, tol: f64) -> bool {
    x.is_finite() && x > tol
}

fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

/// "All" (note §3.3): from the start plane to 1 mm past the farthest corner of the bounding
/// boxes of every solid body of the context component, along `dir`.
fn all_length(ctx: &RecomputeCtx<'_>, f: &ProfileFace, dir: Vec3) -> Result<f64, FeatureError> {
    let o = f.plane.origin;
    let mut far = f64::NEG_INFINITY;
    for (_, b) in ctx.before().bodies() {
        if b.component != ctx.context() {
            continue;
        }
        let BodyGeom::Solid(s) = &b.geom else {
            continue;
        };
        if let Some(shape) = s.brep() {
            far = far.max(far_along(&shape.bbox(), o, dir));
        }
    }
    if far.is_finite() && far > ctx.tolerance().linear {
        Ok(far + 1.0)
    } else {
        Err(invalid(
            "extent",
            "there is no body to extrude through on this side",
        ))
    }
}

/// `TargetMissed` with its hint, the target highlighted.
fn missed(ctx: &RecomputeCtx<'_>, r: &EntityRef) -> FeatureError {
    ctx.highlight([r.clone()]);
    with_hint(&SolidError::TargetMissed)
}

fn behind(ctx: &RecomputeCtx<'_>, r: &EntityRef) -> FeatureError {
    ctx.highlight([r.clone()]);
    invalid(
        "extent",
        "the target is behind the profile; flip the direction or pick another target",
    )
}

#[allow(
    clippy::too_many_arguments,
    reason = "the side's pieces, unpacked once by the caller"
)]
fn to_object(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    f: &ProfileFace,
    s: &Side<'_>,
    target: &Target<'_>,
    r: &EntityRef,
    offset: f64,
    extend: bool,
    out: &mut FeatureOutput,
) -> Result<(Shape, BodyNaming), FeatureError> {
    let o = f.plane.origin;
    let u = s.dir;
    let tol = ctx.tolerance().linear;
    let exact = |h: f64, out: &mut FeatureOutput| {
        if !(h.is_finite() && h > tol) {
            return Err(behind(ctx, r));
        }
        prism_named(ctx, k, f, s, h, out).map(|(r, n)| (r.shape, n))
    };
    let fb = f.face.bbox();
    match target {
        Target::Point(p) => exact((*p - o).dot(u) + offset, out),
        Target::Plane { plane, .. } if plane.normal.get().dot(u).abs() >= PARALLEL => {
            exact((plane.origin - o).dot(u) + offset, out)
        }
        Target::Plane { plane, face } => {
            let mut nt = plane.normal.get();
            if nt.dot(u) < 0.0 {
                nt = -nt;
            }
            let dot = nt.dot(u);
            if dot < MIN_CROSSING {
                return Err(missed(ctx, r));
            }
            let shift = nt * offset;
            let pt = plane.origin + shift;
            let tmax = corners(&fb)
                .iter()
                .map(|c| (pt - *c).dot(nt) / dot)
                .fold(f64::NEG_INFINITY, f64::max);
            if !ahead(tmax, tol) {
                return Err(behind(ctx, r));
            }
            let l = tmax + 1.0;
            let tool = match face {
                Some(bf) if !extend => bf
                    .sheet()?
                    .transformed(&Transform::from_translation(shift))
                    .map_err(kerr)?,
                _ => plane_face(
                    k,
                    &Plane::from_normal(pt, amg_core::Dir3::new_unchecked(nt)),
                    &work_box(&fb, u, l),
                    tol,
                )?,
            };
            split_keep(ctx, k, f, s, l, tool, r, out)
        }
        Target::Face(bf) => {
            let sheet = bf.sheet()?;
            let far = far_along(&sheet.bbox(), o, u);
            if !ahead(far, tol) {
                return Err(behind(ctx, r));
            }
            let mut l = far + offset.abs() + 1.0;
            let mut tool = sheet;
            if extend {
                l += fb.diagonal() + bf.shape.bbox().diagonal();
                tool = extended_face(ctx, k, bf, &work_box(&fb, u, l), r)?;
                let need = far_along(&tool.bbox(), o, u) + offset.abs() + 1.0;
                if need > l {
                    l = need;
                    tool = extended_face(ctx, k, bf, &work_box(&fb, u, l), r)?;
                }
            }
            if offset != 0.0 {
                // The sheet keeps the face's orientation; an extended face is built on the bare
                // surface, whose normal opposes the face's on a reversed face.
                let o_eff = if extend && bf.shape.face_reversed(bf.face) {
                    -offset
                } else {
                    offset
                };
                tool = with_job(ctx, |job| {
                    k.offset(&tool, o_eff, OffsetJoin::Intersection, tol.max(1e-7), job)
                })
                .map_err(kerr)?
                .shape;
            }
            split_keep(ctx, k, f, s, l, tool, r, out)
        }
        Target::Body { shape, .. } => {
            let far = far_along(&shape.bbox(), o, u);
            if !ahead(far, tol) {
                return Err(behind(ctx, r));
            }
            let l = far + offset.abs() + 1.0;
            let tool = if offset != 0.0 {
                with_job(ctx, |job| {
                    k.offset(shape, offset, OffsetJoin::Intersection, tol.max(1e-7), job)
                })
                .map_err(kerr)?
                .shape
            } else {
                (*shape).clone()
            };
            split_keep(ctx, k, f, s, l, tool, r, out)
        }
    }
}

/// The region a long prism of `fb` along `u` by `l` sweeps, grown by 1 mm.
fn work_box(fb: &Aabb3, u: Vec3, l: f64) -> Aabb3 {
    fb.union(&fb.transformed(&Transform::from_translation(u * l)))
        .expanded(1.0)
}

/// A square face on `plane` that covers `w` (seen along the plane normal).
fn plane_face(
    k: &dyn BrepKernel,
    plane: &Plane,
    w: &Aabb3,
    tol: f64,
) -> Result<Shape, FeatureError> {
    let c = plane.project(w.center());
    let pl = Plane::from_normal(c, plane.normal);
    let r = w.diagonal() + 1.0;
    k.make_face(
        &Surface::Plane(pl),
        &FaceBounds::UvRect {
            u0: -r,
            u1: r,
            v0: -r,
            v1: r,
        },
        tol.max(1e-7),
    )
    .map(|b| b.face)
    .map_err(kerr)
}

/// The axial range of `w` along `frame.normal`, grown by a margin.
fn axial_range(frame: &Plane, w: &Aabb3) -> (f64, f64) {
    let n = frame.normal.get();
    let (lo, hi) = corners(w)
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), c| {
            let z = (*c - frame.origin).dot(n);
            (lo.min(z), hi.max(z))
        });
    let m = 1.0 + 0.1 * w.diagonal();
    (lo - m, hi + m)
}

/// The untrimmed surface of a face target over the region `w` (note §3.2, "extend faces"):
/// planes, cylinders and cones over `w`, spheres and tori whole. B-spline and other surfaces
/// fail [`SolidError::ExtendUnsupported`].
fn extended_face(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    bf: &BodyFace<'_>,
    w: &Aabb3,
    r: &EntityRef,
) -> Result<Shape, FeatureError> {
    let tol = ctx.tolerance().linear.max(1e-7);
    let surf = bf.shape.surface(bf.face);
    let bounds = match &surf {
        Surface::Plane(p) => return plane_face(k, p, w, tol),
        Surface::Cylinder { frame, .. } => {
            let (v0, v1) = axial_range(frame, w);
            FaceBounds::UvRect {
                u0: 0.0,
                u1: std::f64::consts::TAU,
                v0,
                v1,
            }
        }
        Surface::Cone {
            frame,
            ref_radius,
            half_angle,
        } => {
            let (z0, z1) = axial_range(frame, w);
            let (sin, cos) = half_angle.sin_cos();
            let (mut v0, mut v1) = (z0 / cos, z1 / cos);
            // Stop short of the apex (v where the radius is 0): one nappe only.
            if sin.abs() > 1e-12 {
                let apex = -ref_radius / sin;
                let eps = 1e-3 * (1.0 + apex.abs());
                if sin > 0.0 {
                    v0 = v0.max(apex + eps);
                } else {
                    v1 = v1.min(apex - eps);
                }
            }
            if v1 <= v0 {
                ctx.highlight([r.clone()]);
                return Err(SolidError::ExtendUnsupported.into());
            }
            FaceBounds::UvRect {
                u0: 0.0,
                u1: std::f64::consts::TAU,
                v0,
                v1,
            }
        }
        Surface::Sphere { .. } | Surface::Torus { .. } => FaceBounds::Natural,
        Surface::BSpline(_) | Surface::Other { .. } => {
            ctx.highlight([r.clone()]);
            return Err(SolidError::ExtendUnsupported.into());
        }
    };
    k.make_face(&surf, &bounds, tol)
        .map(|b| b.face)
        .map_err(kerr)
}

/// The face of `h` with `role`, if any.
fn role_face(h: &ShapeHistory, role: &OpRole) -> Option<SubShape> {
    h.roles
        .iter()
        .find(|(s, r)| s.kind == SubKind::Face && r == role)
        .map(|(s, _)| *s)
}

/// Whether input `sub` of operand 0 has an image in `p`.
fn holds(p: &OpResult, sub: Option<SubShape>) -> bool {
    sub.is_some_and(|sub| {
        p.history
            .of_input(InputSub { operand: 0, sub })
            .any(|rel| !matches!(rel, Relation::Deleted))
    })
}

/// Note §3.2 steps 2–4: the prism of length `l` split by `tool`; the piece holding the start
/// cap is kept. One piece, or a kept piece that still holds the end cap, means the target does
/// not cover the profile (`TargetMissed`).
#[allow(
    clippy::too_many_arguments,
    reason = "the side's pieces, unpacked once by the caller"
)]
fn split_keep(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    f: &ProfileFace,
    s: &Side<'_>,
    l: f64,
    tool: Shape,
    r: &EntityRef,
    out: &mut FeatureOutput,
) -> Result<(Shape, BodyNaming), FeatureError> {
    let (prism, naming) = prism_named(ctx, k, f, s, l, out)?;
    let start = role_face(&prism.history, &OpRole::StartCap);
    let end = role_face(&prism.history, &OpRole::EndCap);
    let split = with_job(ctx, |job| {
        k.split(
            std::slice::from_ref(&prism.shape),
            &[tool],
            &BoolOpts::default(),
            job,
        )
    })
    .map_err(kerr)?;
    let pieces = split.per_solid().map_err(kerr)?;
    let with_start: Vec<&OpResult> = pieces.iter().filter(|p| holds(p, start)).collect();
    if with_start.len() > 1 {
        ctx.highlight([r.clone()]);
        return Err(invalid(
            "extent",
            "the target crosses the profile; move the start or pick another target",
        ));
    }
    let piece = match with_start.first() {
        Some(p) if pieces.len() >= 2 && !holds(p, end) => *p,
        _ => return Err(missed(ctx, r)),
    };
    let mut b = ctx.naming_builder();
    b.from_history(
        &[
            (0, OperandNaming::Named(&naming)),
            (1, OperandNaming::Anonymous),
        ],
        &piece.history,
    );
    let cap = cap_source(ctx, f)?;
    for face in 0..piece.shape.count(SubKind::Face) {
        let o = SubShape {
            kind: SubKind::Face,
            index: face,
        };
        let from_target = piece.history.sources_of(o).iter().any(|(i, rel)| {
            i.operand == 1
                && i.sub.kind == SubKind::Face
                && matches!(rel, Relation::Modified(_) | Relation::Kept(_))
        });
        if from_target {
            b.generated(
                TopoKind::Face,
                face,
                Role::new("EndCap", s.index),
                Some(cap.clone()),
            );
        }
    }
    let n = finish_naming(b, k, &piece.shape, out)?;
    Ok((piece.shape.clone(), n))
}
