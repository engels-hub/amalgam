//! `solid.revolve` (S04 §4, SOL-03): profiles revolved about an axis (sketch line, straight
//! edge, construction or origin axis; also the axis of a revolved face or circular edge) by an
//! angle, a full turn, two sides, symmetric, or up to an object; thin revolves sweep the walls
//! of the thin extrude (`extrude::thin`) instead of the profile regions.
//!
//! - **Axis.** Resolved through CON-03 (`line_of`, else `axis_of`), reversed by `flip`. Side 1
//!   turns counter-clockwise about the (flipped) axis direction, side 2 the other way; a
//!   negative angle reverses its side.
//! - **Validation.** The axis must not be perpendicular to the profile plane, and no profile
//!   may cross it ([`SolidError::AxisCrossesProfile`], the profile highlighted and named in the
//!   message). Profiles touching the axis (an edge on it, a cone) are fine.
//! - **Extents.** `Distance` is an angle in `(0°, 360°]` (360° = `Full`, no caps); "All" is not
//!   a revolve extent. "To object": a point or a plane through the axis gives the exact angle
//!   (the first half-plane the profile reaches); every other plane, face or body splits an
//!   almost full turn (`360° − SPLIT_GAP`) and keeps the piece holding the start cap, as the
//!   extrude does with its long prism (note §3.2; `extend` and `offset` as for the extrude).
//!   Two sides together may not exceed 360°.
//! - **Naming** (naming note §5.2, "like extrude"): laterals `Gen { self, Lateral#side,
//!   SketchCurve }`, caps `StartCap#side` / `EndCap#side`; a full turn has no caps (so refs to
//!   caps fail `NotFound` after an edit to 360°, TNP-22). Edges on the axis generate no face.

mod dialog;

use std::f64::consts::TAU;

use amg_brep::geom::Curve3;
use amg_brep::{BrepKernel, OffsetJoin, OpResult, Shape, TessQuality};
use amg_core::{Dir3, Line3, Plane, Point3, Transform, Vec3};
use amg_doc::{
    BodyNaming, Dependency, Dim, EntityRef, Expr, ExprSrc, Feature, FeatureError, FeatureOutput,
    RecomputeCtx, register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{OpTarget, SolidTool, ToolBody, apply_body_op, op_kernel, with_job};
use super::error::{SolidError, kerr};
use super::extent::{Direction, Extent, ExtentSide, Target, ThinSpec, resolve_target, side_exprs};
use super::extrude::sides::{extended_face, name_sweep, plane_face, split_sweep};
use super::extrude::thin::thin_faces;
use super::extrude::{compound_of, fuse_group};
use super::pattern::{line_input, ref_dependency};
use super::profile::{ProfileFace, ProfileSource, profile_faces};
use crate::construct::RefInput;

/// The gap left open by the almost full turn that "to object" splits (rad, 0.5°): a target
/// within this angle before the profile's own half-plane is missed.
pub const SPLIT_GAP: f64 = 0.5 * std::f64::consts::PI / 180.0;

/// Samples per analytic edge for the axis check.
const EDGE_SAMPLES: u32 = 64;

/// `solid.revolve`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Revolve {
    /// Profile regions (`EntityRef::Profile`) or planar faces (`EntityRef::Topo` faces); with
    /// `thin`, profile regions or sketch curves (`EntityRef::SketchCurve`).
    pub profiles: Vec<EntityRef>,
    /// The axis: an origin axis, a construction axis, a sketch line or a straight edge.
    pub axis: RefInput,
    /// One side, two sides or symmetric.
    #[serde(default)]
    pub direction: Direction,
    /// The first (or only) side; `Distance` is an angle. For symmetric revolves its angle
    /// applies to both sides (half each, or whole with `whole_length`).
    pub side1: ExtentSide,
    /// The second side; `Some` iff `direction == TwoSides`.
    #[serde(default)]
    pub side2: Option<ExtentSide>,
    /// Revolve about the reversed axis (side 1 clockwise).
    #[serde(default)]
    pub flip: bool,
    /// Thin revolve (SOL-18).
    #[serde(default)]
    pub thin: Option<ThinSpec>,
    /// Body operation.
    #[serde(default)]
    pub target: OpTarget,
}

fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

impl Revolve {
    /// A one-sided revolve of `profiles` about `axis` by `angle` (an expression such as
    /// `"90 deg"`).
    pub fn angle(
        profiles: Vec<EntityRef>,
        axis: RefInput,
        angle: &str,
        target: OpTarget,
    ) -> Revolve {
        Revolve {
            profiles,
            axis,
            direction: Direction::OneSide,
            side1: ExtentSide::distance(ex(angle)),
            side2: None,
            flip: false,
            thin: None,
            target,
        }
    }

    /// A full-turn revolve of `profiles` about `axis`.
    pub fn full(profiles: Vec<EntityRef>, axis: RefInput, target: OpTarget) -> Revolve {
        Revolve {
            side1: ExtentSide {
                extent: Extent::Full,
                taper: ex("0 deg"),
            },
            ..Revolve::angle(profiles, axis, "360 deg", target)
        }
    }

    /// The evaluated sides about `axis` (side 1's orientation).
    fn sides<'a>(
        &self,
        ctx: &RecomputeCtx<'a>,
        axis: Line3,
    ) -> Result<Vec<RSide<'a>>, FeatureError> {
        let back = reversed(axis);
        let sides = match self.direction {
            Direction::OneSide => vec![side_of(ctx, &self.side1, 0, axis)?],
            Direction::TwoSides => {
                let s2 = self
                    .side2
                    .as_ref()
                    .ok_or_else(|| invalid("side2", "a two-sided revolve needs a second side"))?;
                vec![
                    side_of(ctx, &self.side1, 0, axis)?,
                    side_of(ctx, s2, 1, back)?,
                ]
            }
            Direction::Symmetric { whole_length } => {
                let Extent::Distance(e) = &self.side1.extent else {
                    return Err(invalid("extent", "a symmetric revolve needs an angle"));
                };
                let a = angle_value(ctx, e)?.abs();
                let h = if whole_length { a } else { a / 2.0 };
                vec![
                    RSide {
                        index: 0,
                        axis,
                        sweep: Sweep::Angle(h),
                    },
                    RSide {
                        index: 1,
                        axis: back,
                        sweep: Sweep::Angle(h),
                    },
                ]
            }
        };
        let total: f64 = sides
            .iter()
            .map(|s| match s.sweep {
                Sweep::Angle(a) => a,
                Sweep::ToObject { .. } => 0.0,
            })
            .sum();
        if sides.len() > 1 && total > TAU + ctx.tolerance().angular.max(1e-12) {
            return Err(invalid(
                "extent",
                "the two sides together exceed 360°; reduce the angles",
            ));
        }
        Ok(sides)
    }
}

/// One side of a revolve, evaluated.
#[derive(Clone, Debug)]
struct RSide<'a> {
    /// The naming side: 0, or 1 for the second side of two-sided and symmetric revolves.
    index: u32,
    /// The axis, oriented so the side turns counter-clockwise about it.
    axis: Line3,
    /// How far.
    sweep: Sweep<'a>,
}

#[derive(Clone, Debug)]
#[allow(clippy::large_enum_variant, reason = "short-lived, a few per compute")]
enum Sweep<'a> {
    /// An angle in `(0, 2π]`, rad.
    Angle(f64),
    /// Up to a target.
    ToObject {
        target: Target<'a>,
        r: EntityRef,
        offset: f64,
        extend: bool,
    },
}

fn reversed(l: Line3) -> Line3 {
    Line3 {
        point: l.point,
        dir: Dir3::new_unchecked(-l.dir.get()),
    }
}

fn angle_value(ctx: &RecomputeCtx<'_>, e: &Expr) -> Result<f64, FeatureError> {
    let a = ctx.eval_angle(e)?;
    let tol = ctx.tolerance().angular.max(1e-12);
    if !a.is_finite() || a.abs() <= tol {
        return Err(invalid("angle", "the angle must not be zero"));
    }
    if a.abs() > TAU + tol {
        return Err(invalid("angle", "the angle must be at most 360°"));
    }
    Ok(a.clamp(-TAU, TAU))
}

/// One stored side, evaluated: a negative angle reverses the side.
fn side_of<'a>(
    ctx: &RecomputeCtx<'a>,
    s: &ExtentSide,
    index: u32,
    axis: Line3,
) -> Result<RSide<'a>, FeatureError> {
    let (axis, sweep) = match &s.extent {
        Extent::Distance(e) => {
            let a = angle_value(ctx, e)?;
            if a < 0.0 {
                (reversed(axis), Sweep::Angle(-a))
            } else {
                (axis, Sweep::Angle(a))
            }
        }
        Extent::Full => (axis, Sweep::Angle(TAU)),
        Extent::ToObject {
            target,
            offset,
            extend,
        } => (
            axis,
            Sweep::ToObject {
                target: resolve_target(ctx, target, "extent")?,
                r: target.clone(),
                offset: ctx.eval_length(offset)?,
                extend: *extend,
            },
        ),
        Extent::All => {
            return Err(invalid(
                "extent",
                "\"All\" is not a revolve extent; use a full turn",
            ));
        }
    };
    Ok(RSide { index, axis, sweep })
}

// ---- profile / axis validation ----------------------------------------------------------------

/// Points along every edge of a profile face (analytic edges sampled, others tessellated).
fn boundary_points(k: &dyn BrepKernel, face: &Shape) -> Result<Vec<Vec<Point3>>, FeatureError> {
    let q = TessQuality {
        linear: 1e-3,
        angular: 0.05,
        relative: false,
        min_size: 0.0,
        max_linear_abs: None,
    };
    let mut out = Vec::new();
    for e in face.edges() {
        let sampled = face.curve(e).and_then(|(c, t0, t1)| {
            if matches!(c, Curve3::Other) {
                return None;
            }
            (0..=EDGE_SAMPLES)
                .map(|i| c.point_at(t0 + (t1 - t0) * f64::from(i) / f64::from(EDGE_SAMPLES)))
                .collect::<Option<Vec<Point3>>>()
        });
        let pts = match sampled {
            Some(p) => p,
            None => k.tessellate_edge(face, e, &q).map_err(kerr)?,
        };
        out.push(pts);
    }
    Ok(out)
}

/// Even-odd test of `p` (in plane coordinates) against the polylines; `None` if `p` is within
/// `tol` of the boundary.
fn inside(polys: &[Vec<amg_core::Point2>], p: amg_core::Point2, tol: f64) -> Option<bool> {
    let mut odd = false;
    for poly in polys {
        for w in poly.windows(2) {
            let (a, b) = (w[0], w[1]);
            let ab = b - a;
            let t = if ab.length_squared() > 0.0 {
                ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
            } else {
                0.0
            };
            if (a + ab * t - p).length() <= tol {
                return None;
            }
            if (a.y > p.y) != (b.y > p.y) {
                let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
                if x > p.x {
                    odd = !odd;
                }
            }
        }
    }
    Some(odd)
}

/// Why a profile can't be revolved about an axis.
enum AxisProblem {
    Perpendicular,
    Crosses,
    OnAxis,
}

/// Checks `f` against `axis` and returns the profile's reference direction: the unit radial
/// direction from the axis to the profile (angle 0 of a to-object angle).
fn check_profile(
    k: &dyn BrepKernel,
    f: &ProfileFace,
    axis: &Line3,
    tol: f64,
) -> Result<Result<Vec3, AxisProblem>, FeatureError> {
    let a = axis.dir.get();
    let n = f.plane.normal.get();
    if a.dot(n).abs() >= 1.0 - 1e-9 {
        return Ok(Err(AxisProblem::Perpendicular));
    }
    let polys = boundary_points(k, &f.face)?;
    let radial = |p: Point3| {
        let d = p - axis.point;
        d - a * d.dot(a)
    };
    let mut mean = Vec3::ZERO;
    let mut count = 0.0_f64;
    for p in polys.iter().flatten() {
        mean += radial(*p);
        count += 1.0;
    }
    let coplanar = a.dot(n).abs() < 1e-9 && f.plane.signed_distance(axis.point).abs() <= tol;
    if coplanar {
        // Signed distance from the axis within the plane.
        let side = n.cross(a).normalize();
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in polys.iter().flatten() {
            let s = (*p - axis.point).dot(side);
            lo = lo.min(s);
            hi = hi.max(s);
        }
        if lo < -tol && hi > tol {
            return Ok(Err(AxisProblem::Crosses));
        }
        if hi <= tol && lo >= -tol {
            return Ok(Err(AxisProblem::OnAxis));
        }
        return Ok(Ok(if hi > tol { side } else { -side }));
    }
    if a.dot(n).abs() > 1e-12 {
        // The axis meets the profile plane at one point: inside the face means crossing.
        let t = (f.plane.origin - axis.point).dot(n) / a.dot(n);
        let x = f.plane.to_local(axis.point + a * t);
        let local: Vec<Vec<amg_core::Point2>> = polys
            .iter()
            .map(|p| p.iter().map(|q| f.plane.to_local(*q)).collect())
            .collect();
        if inside(&local, x, tol) == Some(true) {
            return Ok(Err(AxisProblem::Crosses));
        }
    }
    match (mean / count.max(1.0)).try_normalize() {
        Some(r0) => Ok(Ok(r0)),
        None => Ok(Err(AxisProblem::OnAxis)),
    }
}

/// The stored reference of profile face `f` (for the crossing highlight).
fn profile_ref<'r>(refs: &'r [EntityRef], f: &ProfileFace) -> Vec<&'r EntityRef> {
    if let ProfileSource::Wall(w) = &f.source {
        let hit: Vec<&EntityRef> = refs.iter().filter(|r| w.refs.contains(r)).collect();
        return if hit.is_empty() {
            refs.iter().collect()
        } else {
            hit
        };
    }
    let hit: Vec<&EntityRef> = refs
        .iter()
        .filter(|r| match (r, &f.source) {
            (
                EntityRef::Profile {
                    sketch, profile, ..
                },
                ProfileSource::Region {
                    sketch: s,
                    profile: p,
                },
            ) => sketch == s && profile == p,
            (EntityRef::Topo(t), ProfileSource::Face { key, .. }) => t.key == *key,
            _ => false,
        })
        .collect();
    if hit.is_empty() {
        refs.iter().collect()
    } else {
        hit
    }
}

// ---- sweeps -----------------------------------------------------------------------------------

/// The signed angle from `from` to `to` about `a`, in `(−π, π]`.
fn angle_about(a: Vec3, from: Vec3, to: Vec3) -> f64 {
    a.dot(from.cross(to)).atan2(from.dot(to))
}

/// `angle_about` mapped to `[0, 2π)`.
fn ccw(a: Vec3, from: Vec3, to: Vec3) -> f64 {
    let t = angle_about(a, from, to);
    if t < 0.0 { t + TAU } else { t }
}

fn revol_named(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    f: &ProfileFace,
    s: &RSide<'_>,
    angle: f64,
    out: &mut FeatureOutput,
) -> Result<(OpResult, BodyNaming), FeatureError> {
    let r = with_job(ctx, |job| k.revol(&f.face, &s.axis, angle, job)).map_err(kerr)?;
    let naming = name_sweep(ctx, k, f, s.index, &r, out)?;
    Ok((r, naming))
}

/// The solid of `f` on side `s`, named. `r0` is the profile's reference direction.
fn side_solid(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    f: &ProfileFace,
    r0: Vec3,
    s: &RSide<'_>,
    out: &mut FeatureOutput,
) -> Result<(Shape, BodyNaming), FeatureError> {
    match &s.sweep {
        Sweep::Angle(a) => revol_named(ctx, k, f, s, *a, out).map(|(r, n)| (r.shape, n)),
        Sweep::ToObject {
            target,
            r,
            offset,
            extend,
        } => to_object(ctx, k, f, r0, s, target, r, *offset, *extend, out),
    }
}

fn on_profile(ctx: &RecomputeCtx<'_>, r: &EntityRef) -> FeatureError {
    ctx.highlight([r.clone()]);
    invalid(
        "extent",
        "the target lies in the profile's half-plane; pick another target",
    )
}

/// "To object" for a revolve (S04 §4): the exact angle for a point or a plane through the axis
/// (no offset); otherwise an almost full turn split by the target, keeping the piece that holds
/// the start cap.
#[allow(
    clippy::too_many_arguments,
    reason = "the side's pieces, unpacked once by the caller"
)]
fn to_object(
    ctx: &RecomputeCtx<'_>,
    k: &dyn BrepKernel,
    f: &ProfileFace,
    r0: Vec3,
    s: &RSide<'_>,
    target: &Target<'_>,
    r: &EntityRef,
    offset: f64,
    extend: bool,
    out: &mut FeatureOutput,
) -> Result<(Shape, BodyNaming), FeatureError> {
    let a = s.axis.dir.get();
    let tol = ctx.tolerance().linear;
    let ang_tol = 1e-9;
    let exact = |t: f64, out: &mut FeatureOutput| {
        revol_named(ctx, k, f, s, t, out).map(|(r, n)| (r.shape, n))
    };
    match target {
        Target::Point(p) => {
            if offset != 0.0 {
                ctx.highlight([r.clone()]);
                return Err(invalid(
                    "extent",
                    "a point target takes no offset; use a plane, face or body",
                ));
            }
            let d = *p - s.axis.point;
            let rp = d - a * d.dot(a);
            if rp.length() <= tol {
                ctx.highlight([r.clone()]);
                return Err(invalid("extent", "the target point lies on the axis"));
            }
            let t = ccw(a, r0, rp);
            if t <= ang_tol {
                return Err(on_profile(ctx, r));
            }
            exact(t, out)
        }
        Target::Plane { plane, face }
            if offset == 0.0
                && (face.is_none() || extend)
                && plane.normal.get().dot(a).abs() < 1e-9
                && plane.signed_distance(s.axis.point).abs() <= tol =>
        {
            // A plane through the axis: the first of its two half-planes.
            let d = a.cross(plane.normal.get()).normalize();
            let t = [ccw(a, r0, d), ccw(a, r0, -d)]
                .into_iter()
                .filter(|t| *t > ang_tol)
                .fold(f64::INFINITY, f64::min);
            if !t.is_finite() {
                return Err(on_profile(ctx, r));
            }
            exact(t, out)
        }
        Target::Plane { plane, face } => {
            let sweep = revol_named(ctx, k, f, s, TAU - SPLIT_GAP, out)?;
            let n = plane.normal.get();
            let shift = n * offset;
            let tool = match face {
                Some(bf) if !extend => bf
                    .sheet()?
                    .transformed(&Transform::from_translation(shift))
                    .map_err(kerr)?,
                _ => plane_face(
                    k,
                    &Plane::from_normal(plane.origin + shift, plane.normal),
                    &sweep.0.shape.bbox().expanded(1.0),
                    tol,
                )?,
            };
            split_sweep(ctx, k, f, s.index, sweep, tool, r, out)
        }
        Target::Face(bf) => {
            let sweep = revol_named(ctx, k, f, s, TAU - SPLIT_GAP, out)?;
            let mut tool = if extend {
                let w = sweep
                    .0
                    .shape
                    .bbox()
                    .union(&bf.shape.bbox())
                    .expanded(1.0 + offset.abs());
                extended_face(ctx, k, bf, &w, r)?
            } else {
                bf.sheet()?
            };
            if offset != 0.0 {
                // As in the extrude: an extended face is built on the bare surface, whose
                // normal opposes the face's on a reversed face.
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
            split_sweep(ctx, k, f, s.index, sweep, tool, r, out)
        }
        Target::Body { shape } => {
            let sweep = revol_named(ctx, k, f, s, TAU - SPLIT_GAP, out)?;
            let tool = if offset != 0.0 {
                with_job(ctx, |job| {
                    k.offset(shape, offset, OffsetJoin::Intersection, tol.max(1e-7), job)
                })
                .map_err(kerr)?
                .shape
            } else {
                (*shape).clone()
            };
            split_sweep(ctx, k, f, s.index, sweep, tool, r, out)
        }
    }
}

impl SolidTool for Revolve {
    fn tool(&self, ctx: &RecomputeCtx<'_>) -> Result<ToolBody, FeatureError> {
        let faces = match &self.thin {
            Some(spec) => thin_faces(ctx, &self.profiles, spec)?,
            None => profile_faces(ctx, &self.profiles)?,
        };
        let axis = line_input(ctx, &self.axis, "axis")?;
        let axis = if self.flip { reversed(axis) } else { axis };
        let k = op_kernel(ctx);
        let tol = ctx.tolerance().linear.max(1e-6);
        let mut refs = Vec::with_capacity(faces.len());
        for (i, f) in faces.iter().enumerate() {
            match check_profile(k, f, &axis, tol)? {
                Ok(r0) => refs.push(r0),
                Err(problem) => {
                    let which = profile_ref(&self.profiles, f);
                    let n = self
                        .profiles
                        .iter()
                        .position(|r| which.first().is_some_and(|w| *w == r))
                        .unwrap_or(i)
                        + 1;
                    ctx.highlight(which.into_iter().cloned());
                    if let Some(a) = self.axis.entity() {
                        ctx.highlight([a.clone()]);
                    }
                    let e = SolidError::AxisCrossesProfile;
                    return Err(match problem {
                        AxisProblem::Crosses => FeatureError::InvalidValue {
                            field: "axis",
                            msg: format!(
                                "{e}: profile {n} of {}. Move the axis or split the profile at it.",
                                self.profiles.len()
                            ),
                        },
                        AxisProblem::Perpendicular => invalid(
                            "axis",
                            format!("the axis is perpendicular to the plane of profile {n}"),
                        ),
                        AxisProblem::OnAxis => {
                            invalid("axis", format!("profile {n} lies on the axis"))
                        }
                    });
                }
            }
        }
        let sides = self.sides(ctx, axis)?;
        let mut scratch = FeatureOutput::default();
        let mut groups: Vec<Vec<(Shape, BodyNaming)>> = Vec::new();
        for (f, r0) in faces.iter().zip(refs) {
            ctx.check_cancelled()?;
            let g = f.group as usize;
            if groups.len() <= g {
                groups.resize_with(g + 1, Vec::new);
            }
            for s in &sides {
                groups[g].push(side_solid(ctx, k, f, r0, s, &mut scratch)?);
            }
        }
        let mut solids = Vec::with_capacity(groups.len());
        for g in groups {
            if !g.is_empty() {
                solids.push(fuse_group(ctx, k, g, &mut scratch)?);
            }
        }
        let (shape, naming) = compound_of(ctx, k, solids, &mut scratch)?;
        Ok(ToolBody {
            shape,
            naming,
            target: self.target.clone(),
        })
    }
}

/// The entity references of the sides (to-object targets).
fn side_refs<'a>(
    sides: impl Iterator<Item = &'a ExtentSide> + 'a,
) -> impl Iterator<Item = EntityRef> + 'a {
    sides.filter_map(|s| match &s.extent {
        Extent::ToObject { target, .. } => Some(target.clone()),
        _ => None,
    })
}

impl Feature for Revolve {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d: Vec<Dependency> = Vec::new();
        for p in &self.profiles {
            d.push(Dependency::Entity(p.clone()));
            if let EntityRef::Profile { sketch, .. } | EntityRef::SketchCurve { sketch, .. } = p {
                let s = Dependency::Sketch(*sketch);
                if !d.contains(&s) {
                    d.push(s);
                }
            }
        }
        if let Some(a) = ref_dependency(&self.axis) {
            d.push(a);
        }
        for r in side_refs(std::iter::once(&self.side1).chain(&self.side2)) {
            // A body target is read whole: list the body itself, not only the reference.
            if let EntityRef::Body(b) = &r {
                d.push(Dependency::Body(*b));
            }
            let dep = match r {
                EntityRef::Construction(c) => Dependency::Construction(c),
                other => Dependency::Entity(other),
            };
            d.push(dep);
        }
        d.extend(self.target.inputs());
        d
    }

    fn exprs(&self) -> Vec<&Expr> {
        // The fields of `exprs_mut`, in the same order.
        let mut v: Vec<&Expr> = Vec::new();
        for s in std::iter::once(&self.side1).chain(&self.side2) {
            match &s.extent {
                Extent::Distance(e) | Extent::ToObject { offset: e, .. } => v.push(e),
                Extent::All | Extent::Full => {}
            }
            v.push(&s.taper);
        }
        if let Some(t) = &self.thin {
            v.push(&t.thickness);
        }
        v
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        let mut v = side_exprs(&mut self.side1, Dim::ANGLE);
        if let Some(s) = &mut self.side2 {
            v.extend(side_exprs(s, Dim::ANGLE));
        }
        if let Some(t) = &mut self.thin {
            v.push((&mut t.thickness, Dim::LENGTH));
        }
        v
    }

    fn validate(&self) -> Result<(), FeatureError> {
        if self.profiles.is_empty() {
            return Err(invalid("profiles", "select at least one profile"));
        }
        if self.side2.is_some() != (self.direction == Direction::TwoSides) {
            return Err(invalid(
                "side2",
                "a second side is set exactly for two-sided revolves",
            ));
        }
        let sides = || std::iter::once(&self.side1).chain(&self.side2);
        if sides().any(|s| s.extent == Extent::All) {
            return Err(invalid(
                "extent",
                "\"All\" is not a revolve extent; use a full turn",
            ));
        }
        if self.direction != Direction::OneSide && sides().any(|s| s.extent == Extent::Full) {
            return Err(invalid(
                "extent",
                "a full turn is one-sided; choose One side",
            ));
        }
        if matches!(self.direction, Direction::Symmetric { .. })
            && !matches!(self.side1.extent, Extent::Distance(_))
        {
            return Err(invalid("extent", "a symmetric revolve needs an angle"));
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let tool = self.tool(ctx)?;
        let mut out = FeatureOutput::default();
        apply_body_op(ctx, tool, &mut out)?;
        Ok(out)
    }
}

register_feature!(Revolve, "solid.revolve", version = 1);
crate::register_tool!(Revolve, "solid.revolve");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ccw_angles_are_in_zero_to_two_pi() {
        let z = Vec3::Z;
        let x = Vec3::X;
        assert!((ccw(z, x, Vec3::Y) - std::f64::consts::FRAC_PI_2).abs() < 1e-15);
        assert!((ccw(z, x, -Vec3::Y) - 1.5 * std::f64::consts::PI).abs() < 1e-15);
        assert!(ccw(z, x, x).abs() < 1e-15);
        // About −Z the same target is a quarter turn the other way.
        assert!((ccw(-z, x, -Vec3::Y) - std::f64::consts::FRAC_PI_2).abs() < 1e-15);
    }

    #[test]
    fn even_odd_inside_test_with_a_hole() {
        let p = amg_core::Point2::new;
        let sq = |a: f64| vec![p(-a, -a), p(a, -a), p(a, a), p(-a, a), p(-a, -a)];
        let polys = vec![sq(10.0), sq(2.0)];
        assert_eq!(inside(&polys, p(5.0, 0.0), 1e-6), Some(true));
        assert_eq!(inside(&polys, p(0.0, 0.0), 1e-6), Some(false));
        assert_eq!(inside(&polys, p(20.0, 0.0), 1e-6), Some(false));
        assert_eq!(inside(&polys, p(10.0, 0.0), 1e-6), None);
    }
}
