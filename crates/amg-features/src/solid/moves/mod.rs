//! `solid.move` and `solid.align` (S04 SOL-09): rigid motions of whole bodies.
//!
//! - **Move** places the bodies by a free matrix (a triad drag, not parametric), a
//!   translation, a rotation about an axis, or a point-to-point vector.
//! - **Align** moves the bodies so that a `from` element (planar face or plane, straight edge,
//!   axis or circle, point) lands on a `to` element of the same dimension: planes become
//!   coplanar and face each other (`flip`: the same way), lines coincide (`flip`: reversed),
//!   points coincide. A point can also go onto a line or a plane and vice versa. `angle` then
//!   turns the bodies about the target's normal or direction.
//!
//! Naming (naming note §5.2, "Move / copy / align" row): a moved body keeps its element map
//! as is (`Shape::transformed` keeps every index, S02 Q1), so every reference into it resolves
//! `Exact`. A copy (`copy = true`) is a new body `BodyId { self, i }` (`i` = the position of
//! the source in `bodies`) named `instanced(self, i)`; a copy is not lineage (plain `Created`).

pub(crate) mod dialog;

use amg_brep::{Shape, SubKind, SubShape};
use amg_core::{Dir3, Line3, Mat4, Plane, Point3, Quat, Transform, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, Dependency, Dim, EntityRef, Expr, ExprSrc, Feature, FeatureError,
    FeatureOutput, RecomputeCtx, TopoKind, register_feature,
};
use serde::{Deserialize, Serialize};

use super::common::{Participant, live_body, solid_geom, warning};
use super::error::kerr;
use super::pattern::{line_input, ref_dependency};
use crate::construct::math::{center_of, line_of, point_of};
use crate::construct::resolve::resolve_input;
use crate::construct::{RefGeom, RefInput};

// ---- data -------------------------------------------------------------------------------------

/// How `solid.move` places its bodies.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum MoveSpec {
    /// A rigid 4×4 matrix from a triad drag, row-major (`matrix[r][c]`, translation in column
    /// 3, last row `0 0 0 1`); not parametric.
    Free {
        /// The matrix.
        matrix: [[f64; 4]; 4],
    },
    /// A translation along world X, Y, Z (mm).
    Translate([Expr; 3]),
    /// A rotation about an axis (origin axis, construction axis, straight edge, sketch line, or
    /// the axis of a revolved face or circle), counter-clockwise looking against the axis.
    Rotate {
        /// The axis.
        axis: RefInput,
        /// The angle (rad).
        angle: Expr,
    },
    /// The translation from one point to another (vertices, sketch points, construction
    /// points, or the centre of a circle, sphere or torus).
    PointToPoint {
        /// Start point.
        from: RefInput,
        /// End point.
        to: RefInput,
    },
}

/// The identity as a row-major matrix.
pub const IDENTITY_MATRIX: [[f64; 4]; 4] = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

/// `solid.move`: bodies moved (or copied) rigidly.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Move {
    /// The bodies, in order (a copy of body `i` is `BodyId { self, i }`).
    #[serde(default)]
    pub bodies: Vec<amg_doc::BodyId>,
    /// The motion.
    pub spec: MoveSpec,
    /// Leave the bodies where they are and make moved copies.
    #[serde(default)]
    pub copy: bool,
}

/// `solid.align`: bodies moved so `from` lands on `to`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Align {
    /// The bodies, in order.
    #[serde(default)]
    pub bodies: Vec<amg_doc::BodyId>,
    /// The element that moves with the bodies (`None` until picked: the dialog builds
    /// without one and the feature then fails to compute).
    #[serde(default)]
    pub from: Option<RefInput>,
    /// The element it lands on.
    pub to: RefInput,
    /// Planes: face the same way instead of opposite; lines: reversed.
    #[serde(default)]
    pub flip: bool,
    /// Turn about the target's normal or direction afterwards (rad).
    pub angle: Expr,
}

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

impl Move {
    /// A translation of `bodies` by `[dx, dy, dz]` (expression texts, e.g. `"10 mm"`).
    pub fn translate(bodies: Vec<amg_doc::BodyId>, d: [&str; 3]) -> Move {
        Move {
            bodies,
            spec: MoveSpec::Translate(d.map(ex)),
            copy: false,
        }
    }

    /// A rotation of `bodies` about `axis` by `angle` (expression text, e.g. `"90 deg"`).
    pub fn rotate(bodies: Vec<amg_doc::BodyId>, axis: RefInput, angle: &str) -> Move {
        Move {
            bodies,
            spec: MoveSpec::Rotate {
                axis,
                angle: ex(angle),
            },
            copy: false,
        }
    }

    /// The same motion as copies.
    #[must_use]
    pub fn copied(mut self) -> Move {
        self.copy = true;
        self
    }
}

impl Align {
    /// An alignment of `bodies` from `from` onto `to`, no flip, no angle.
    pub fn new(bodies: Vec<amg_doc::BodyId>, from: RefInput, to: RefInput) -> Align {
        Align {
            bodies,
            from: Some(from),
            to,
            flip: false,
            angle: ex("0 deg"),
        }
    }
}

pub(crate) fn invalid(field: &'static str, msg: impl Into<String>) -> FeatureError {
    super::error::SolidError::Invalid {
        field,
        msg: msg.into(),
    }
    .into()
}

// ---- transforms -------------------------------------------------------------------------------

/// The affine map of a row-major matrix, or why it is not one.
pub(crate) fn matrix_transform(m: &[[f64; 4]; 4]) -> Result<Transform, &'static str> {
    if m.iter().flatten().any(|x| !x.is_finite()) {
        return Err("the matrix is not finite");
    }
    let last = m[3];
    if last[0].abs() > 1e-12 || last[1].abs() > 1e-12 || last[2].abs() > 1e-12 {
        return Err("the matrix is not affine (its last row must be 0 0 0 1)");
    }
    if (last[3] - 1.0).abs() > 1e-12 {
        return Err("the matrix is not affine (its last row must be 0 0 0 1)");
    }
    // glam is column-major.
    let cols = [
        [m[0][0], m[1][0], m[2][0], m[3][0]],
        [m[0][1], m[1][1], m[2][1], m[3][1]],
        [m[0][2], m[1][2], m[2][2], m[3][2]],
        [m[0][3], m[1][3], m[2][3], m[3][3]],
    ];
    Ok(Transform::from_mat4(Mat4::from_cols_array_2d(&cols)))
}

/// A rotation by `angle` about `line`.
pub(crate) fn rotation_about(line: &Line3, angle: f64) -> Transform {
    let p = line.point;
    Transform::from_translation(p)
        * Transform::from_axis_angle(line.dir.get(), angle)
        * Transform::from_translation(-p)
}

/// A rotation taking direction `a` to `b` about `pivot` (the shortest arc).
fn rotation_onto(a: Vec3, b: Vec3, pivot: Point3) -> Transform {
    let q = Quat::from_rotation_arc(a.normalize(), b.normalize());
    Transform::from_translation(pivot)
        * Transform::from_quat(q)
        * Transform::from_translation(-pivot)
}

/// `v` as a unit direction (`None` for a zero or non-finite vector).
fn unit(v: Vec3) -> Option<Dir3> {
    let n = v.normalize_or_zero();
    (n != Vec3::ZERO).then(|| Dir3::new_unchecked(n))
}

/// One element of an alignment.
#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) enum Elem {
    /// A point.
    Point(Point3),
    /// A directed line (its point is the pivot).
    Line(Line3),
    /// An oriented plane (its origin is the pivot).
    Plane(Plane),
}

impl Elem {
    fn moved(self, t: &Transform) -> Elem {
        match self {
            Elem::Point(p) => Elem::Point(t.transform_point3(p)),
            Elem::Line(l) => Elem::Line(Line3 {
                point: t.transform_point3(l.point),
                dir: unit(t.transform_vector3(l.dir.get())).unwrap_or(l.dir),
            }),
            Elem::Plane(p) => {
                let n = t.transform_vector3(p.normal.get());
                Elem::Plane(Plane::from_normal(
                    t.transform_point3(p.origin),
                    unit(n).unwrap_or(p.normal),
                ))
            }
        }
    }

    /// The pivot: the point, the line's point, the plane's origin.
    fn pivot(&self) -> Point3 {
        match self {
            Elem::Point(p) => *p,
            Elem::Line(l) => l.point,
            Elem::Plane(p) => p.origin,
        }
    }

    /// The point of this element nearest to `q`.
    fn closest(&self, q: Point3) -> Point3 {
        match self {
            Elem::Point(p) => *p,
            Elem::Line(l) => {
                let d = l.dir.get();
                l.point + d * (q - l.point).dot(d)
            }
            Elem::Plane(p) => q - p.normal.get() * p.signed_distance(q),
        }
    }

    /// The direction an angle turns about (plane normal, line direction).
    fn axis(&self) -> Option<Vec3> {
        match self {
            Elem::Point(_) => None,
            Elem::Line(l) => Some(l.dir.get()),
            Elem::Plane(p) => Some(p.normal.get()),
        }
    }
}

/// The rigid motion of an alignment (module docs). `Err` names the problem.
pub(crate) fn align_transform(
    from: Elem,
    to: Elem,
    flip: bool,
    angle: f64,
) -> Result<Transform, &'static str> {
    let rot = match (from, to) {
        (Elem::Plane(a), Elem::Plane(b)) => {
            let target = if flip {
                b.normal.get()
            } else {
                -b.normal.get()
            };
            rotation_onto(a.normal.get(), target, a.origin)
        }
        (Elem::Line(a), Elem::Line(b)) => {
            let target = if flip { -b.dir.get() } else { b.dir.get() };
            rotation_onto(a.dir.get(), target, a.point)
        }
        (Elem::Plane(_), Elem::Line(_)) | (Elem::Line(_), Elem::Plane(_)) => {
            return Err("align a plane to a plane, a line to a line, or a point to anything");
        }
        _ => Transform::IDENTITY,
    };
    let turned = from.moved(&rot);
    // The shortest translation that puts the lower-dimensional element onto the other.
    let shift = match (turned, to) {
        (_, Elem::Point(p)) => p - turned.closest(p),
        (f, t) => {
            let q = f.pivot();
            t.closest(q) - q
        }
    };
    let placed = Transform::from_translation(shift) * rot;
    if angle == 0.0 {
        return Ok(placed);
    }
    let Some(axis) = to.axis().or_else(|| from.moved(&placed).axis()) else {
        return Err("an angle needs a plane or a line to turn about");
    };
    let pivot = from.moved(&placed).pivot();
    let line = Line3 {
        point: pivot,
        dir: unit(axis).ok_or("the turn axis is degenerate")?,
    };
    Ok(rotation_about(&line, angle) * placed)
}

/// The world point of a point input: a point, or the centre of a circle, sphere or torus.
fn point_input(
    ctx: &RecomputeCtx<'_>,
    r: &RefInput,
    field: &'static str,
) -> Result<Point3, FeatureError> {
    let g = resolve_input(ctx, r)?.geom;
    point_of(&g).or_else(|_| center_of(&g)).map_err(|e| {
        ctx.highlight(r.entity().cloned());
        invalid(field, format!("not a point: {e}"))
    })
}

/// The centre of a B-rep face's bounding box, for a face reference (the pivot of a planar
/// face, so an angle turns about the face and not about its surface's origin).
fn face_center(ctx: &RecomputeCtx<'_>, r: &RefInput) -> Option<Point3> {
    let RefInput::Ref(EntityRef::Topo(t)) = r else {
        return None;
    };
    if t.kind != TopoKind::Face {
        return None;
    }
    let res = ctx.resolve(t).ok()?;
    let (shape, _) = super::common::input_solid(ctx, res.body).ok()?;
    let face = shape
        .sub_shape(SubShape {
            kind: SubKind::Face,
            index: res.index,
        })
        .ok()?;
    Some(face.bbox().center())
}

/// The alignment element of an input.
fn elem_input(
    ctx: &RecomputeCtx<'_>,
    r: &RefInput,
    field: &'static str,
) -> Result<Elem, FeatureError> {
    let g = resolve_input(ctx, r)?.geom;
    let tol = ctx.tolerance();
    let e = match &g {
        RefGeom::Point(p) => Elem::Point(*p),
        RefGeom::Line(_) | RefGeom::Segment(_) | RefGeom::Curve(_) => match line_of(&g, tol) {
            Ok(l) => Elem::Line(l),
            Err(e) => {
                ctx.highlight(r.entity().cloned());
                return Err(invalid(field, format!("can't align to this: {e}")));
            }
        },
        RefGeom::Plane(p) => {
            let pivot =
                face_center(ctx, r).map_or(p.origin, |c| c - p.normal.get() * p.signed_distance(c));
            Elem::Plane(Plane::from_normal(pivot, p.normal))
        }
        RefGeom::Circle { center, normal, .. } => Elem::Line(Line3 {
            point: *center,
            dir: *normal,
        }),
        RefGeom::Cylinder { axis, .. } => Elem::Line(*axis),
        RefGeom::Sphere { center, .. } => Elem::Point(*center),
        other => {
            ctx.highlight(r.entity().cloned());
            return Err(invalid(
                field,
                format!(
                    "can't align to a {}; pick a planar face, plane, straight edge, axis, circle or point",
                    other.kind_name()
                ),
            ));
        }
    };
    Ok(e)
}

// ---- shared compute ---------------------------------------------------------------------------

/// The live, deduplicated bodies (successors followed; gone ones dropped with a warning).
fn live_bodies<'a>(
    ctx: &RecomputeCtx<'a>,
    bodies: &[amg_doc::BodyId],
    out: &mut FeatureOutput,
) -> Result<Vec<Participant<'a>>, FeatureError> {
    if bodies.is_empty() {
        return Err(invalid("bodies", "select at least one body"));
    }
    let mut v: Vec<Participant<'a>> = Vec::new();
    for &b in bodies {
        let Some(p) = live_body(ctx, b, out)? else {
            continue;
        };
        if !v.iter().any(|q| q.id == p.id) {
            v.push(p);
        }
    }
    if v.is_empty() {
        return Err(invalid("bodies", "no selected body is left"));
    }
    Ok(v)
}

/// Moves (or copies) every body by `t` and appends the body changes.
pub(crate) fn place_bodies(
    ctx: &RecomputeCtx<'_>,
    bodies: &[amg_doc::BodyId],
    t: &Transform,
    copy: bool,
    field: &'static str,
    out: &mut FeatureOutput,
) -> Result<(), FeatureError> {
    let parts = live_bodies(ctx, bodies, out)?;
    let me = ctx.feature_id();
    for (slot, p) in (0u32..).zip(&parts) {
        ctx.check_cancelled()?;
        let moved: Shape = p.shape.transformed(t).map_err(|e| match e {
            amg_brep::KernelError::NonRigid => invalid(
                field,
                "the motion is not rigid (scale, shear or reflection)",
            ),
            e => kerr(e),
        })?;
        let change = if copy {
            let naming = p
                .naming
                .instanced(me, slot, &moved)
                .map_err(|e| FeatureError::Kernel(format!("naming: {e}")))?;
            BodyChange {
                id: ctx.new_body_id(slot),
                change: BodyChangeKind::Created {
                    geom: solid_geom(moved),
                    naming: Some(naming),
                },
            }
        } else {
            BodyChange {
                id: p.id,
                change: BodyChangeKind::Modified {
                    geom: solid_geom(moved),
                    naming: Some(p.naming.clone()),
                },
            }
        };
        out.bodies.push(change);
    }
    if !copy && t.abs_diff_eq(Transform::IDENTITY, 0.0) {
        out.warnings.push(warning(
            "The motion is zero; the bodies stay where they are.".into(),
        ));
    }
    Ok(())
}

fn body_inputs(bodies: &[amg_doc::BodyId]) -> impl Iterator<Item = Dependency> + '_ {
    bodies.iter().map(|b| Dependency::Body(*b))
}

// ---- Move ---------------------------------------------------------------------------------------

impl Move {
    /// The evaluated motion.
    ///
    /// # Errors
    /// Expression, reference and matrix errors.
    pub fn transform(&self, ctx: &RecomputeCtx<'_>) -> Result<Transform, FeatureError> {
        match &self.spec {
            MoveSpec::Free { matrix } => matrix_transform(matrix).map_err(|m| invalid("matrix", m)),
            MoveSpec::Translate(d) => {
                let v = Vec3::new(
                    ctx.eval_length(&d[0])?,
                    ctx.eval_length(&d[1])?,
                    ctx.eval_length(&d[2])?,
                );
                Ok(Transform::from_translation(v))
            }
            MoveSpec::Rotate { axis, angle } => {
                let line = line_input(ctx, axis, "axis")?;
                Ok(rotation_about(&line, ctx.eval_angle(angle)?))
            }
            MoveSpec::PointToPoint { from, to } => {
                let a = point_input(ctx, from, "from")?;
                let b = point_input(ctx, to, "to")?;
                Ok(Transform::from_translation(b - a))
            }
        }
    }
}

impl Feature for Move {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d: Vec<Dependency> = body_inputs(&self.bodies).collect();
        match &self.spec {
            MoveSpec::Free { .. } | MoveSpec::Translate(_) => {}
            MoveSpec::Rotate { axis, .. } => d.extend(ref_dependency(axis)),
            MoveSpec::PointToPoint { from, to } => {
                d.extend(ref_dependency(from));
                d.extend(ref_dependency(to));
            }
        }
        d
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        matches!(dep, Dependency::Body(_))
    }

    fn exprs(&self) -> Vec<&Expr> {
        match &self.spec {
            MoveSpec::Free { .. } | MoveSpec::PointToPoint { .. } => Vec::new(),
            MoveSpec::Translate(d) => d.iter().collect(),
            MoveSpec::Rotate { angle, .. } => vec![angle],
        }
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        match &mut self.spec {
            MoveSpec::Free { .. } | MoveSpec::PointToPoint { .. } => Vec::new(),
            MoveSpec::Translate(d) => d.iter_mut().map(|e| (e, Dim::LENGTH)).collect(),
            MoveSpec::Rotate { angle, .. } => vec![(angle, Dim::ANGLE)],
        }
    }

    fn validate(&self) -> Result<(), FeatureError> {
        // An empty body pick builds and validates (the dialog's `min: 1` keeps OK off) and
        // fails in `compute`, as for fillet edges.
        if let MoveSpec::Free { matrix } = &self.spec {
            matrix_transform(matrix).map_err(|m| invalid("matrix", m))?;
        }
        Ok(())
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        let t = self.transform(ctx)?;
        let field = match self.spec {
            MoveSpec::Free { .. } => "matrix",
            _ => "spec",
        };
        place_bodies(ctx, &self.bodies, &t, self.copy, field, &mut out)?;
        Ok(out)
    }
}

register_feature!(Move, "solid.move", version = 1);

// ---- Align --------------------------------------------------------------------------------------

impl Align {
    /// The evaluated motion.
    ///
    /// # Errors
    /// Reference errors and elements that can't be aligned.
    pub fn transform(&self, ctx: &RecomputeCtx<'_>) -> Result<Transform, FeatureError> {
        let Some(from) = &self.from else {
            return Err(invalid("from", "select what to align"));
        };
        let a = elem_input(ctx, from, "from")?;
        let b = elem_input(ctx, &self.to, "to")?;
        let angle = ctx.eval_angle(&self.angle)?;
        align_transform(a, b, self.flip, angle).map_err(|m| {
            ctx.highlight(
                from.entity()
                    .cloned()
                    .into_iter()
                    .chain(self.to.entity().cloned()),
            );
            invalid("to", m)
        })
    }
}

impl Feature for Align {
    fn inputs(&self) -> Vec<Dependency> {
        let mut d: Vec<Dependency> = body_inputs(&self.bodies).collect();
        if let Some(f) = &self.from {
            d.extend(ref_dependency(f));
        }
        d.extend(ref_dependency(&self.to));
        d
    }

    fn input_is_optional(&self, dep: &Dependency) -> bool {
        matches!(dep, Dependency::Body(_))
    }

    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.angle]
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dim)> {
        vec![(&mut self.angle, Dim::ANGLE)]
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let mut out = FeatureOutput::default();
        let t = self.transform(ctx)?;
        place_bodies(ctx, &self.bodies, &t, false, "to", &mut out)?;
        Ok(out)
    }
}

register_feature!(Align, "solid.align", version = 1);

#[cfg(test)]
mod tests {
    use super::*;
    fn dir(x: f64, y: f64, z: f64) -> Dir3 {
        unit(Vec3::new(x, y, z)).unwrap()
    }

    fn plane(o: [f64; 3], n: Dir3) -> Elem {
        Elem::Plane(Plane::from_normal(Point3::from_array(o), n))
    }

    #[track_caller]
    fn near(a: Vec3, b: Vec3) {
        assert!((a - b).length() < 1e-12, "{a:?} vs {b:?}");
    }

    #[test]
    fn row_major_matrices_put_the_translation_in_column_3() {
        let mut m = IDENTITY_MATRIX;
        m[0][3] = 5.0;
        m[2][3] = -2.0;
        let t = matrix_transform(&m).unwrap();
        near(t.transform_point3(Point3::ZERO), Vec3::new(5.0, 0.0, -2.0));
        let mut bad = IDENTITY_MATRIX;
        bad[3][0] = 1.0;
        assert!(matrix_transform(&bad).is_err());
        bad = IDENTITY_MATRIX;
        bad[1][1] = f64::NAN;
        assert!(matrix_transform(&bad).is_err());
    }

    #[test]
    fn planes_end_coplanar_and_opposed_or_flipped() {
        // +X face of a cube at x = 10 onto a top face at z = 30 (normal +Z).
        let from = plane([10.0, 5.0, 5.0], dir(1.0, 0.0, 0.0));
        let to = plane([50.0, 0.0, 30.0], dir(0.0, 0.0, 1.0));
        for flip in [false, true] {
            let t = align_transform(from, to, flip, 0.0).unwrap();
            let Elem::Plane(p) = from.moved(&t) else {
                unreachable!()
            };
            let want = if flip { 1.0 } else { -1.0 };
            near(p.normal.get(), Vec3::new(0.0, 0.0, want));
            assert!((p.origin.z - 30.0).abs() < 1e-12);
            // No slide: the pivot keeps its in-plane position after the turn.
            assert!((p.origin.x - 10.0).abs() < 1e-12 && (p.origin.y - 5.0).abs() < 1e-12);
        }
    }

    #[test]
    fn lines_coincide_points_meet_and_angles_turn_about_the_target() {
        let from = Elem::Line(Line3 {
            point: Point3::new(0.0, 0.0, 0.0),
            dir: dir(1.0, 0.0, 0.0),
        });
        let to = Elem::Line(Line3 {
            point: Point3::new(0.0, 5.0, 0.0),
            dir: dir(0.0, 0.0, 1.0),
        });
        let t = align_transform(from, to, false, 0.0).unwrap();
        let Elem::Line(l) = from.moved(&t) else {
            unreachable!()
        };
        near(l.dir.get(), Vec3::Z);
        near(to.closest(l.point), l.point);

        let p = Elem::Point(Point3::new(1.0, 2.0, 3.0));
        let q = Elem::Point(Point3::new(-4.0, 0.0, 9.0));
        let t = align_transform(p, q, false, 0.0).unwrap();
        near(
            t.transform_point3(Point3::new(1.0, 2.0, 3.0)),
            Vec3::new(-4.0, 0.0, 9.0),
        );
        assert!(
            align_transform(p, q, false, 0.5).is_err(),
            "no axis for an angle"
        );

        // A point onto a plane, then turned 90° about its normal through the landed point.
        let pl = plane([0.0, 0.0, 10.0], dir(0.0, 0.0, 1.0));
        let t = align_transform(p, pl, false, std::f64::consts::FRAC_PI_2).unwrap();
        near(
            t.transform_point3(Point3::new(1.0, 2.0, 3.0)),
            Vec3::new(1.0, 2.0, 10.0),
        );
        near(
            t.transform_point3(Point3::new(2.0, 2.0, 3.0)),
            Vec3::new(1.0, 3.0, 10.0),
        );
        assert!(
            align_transform(pl, from, false, 0.0).is_err(),
            "plane onto line"
        );
    }
}
