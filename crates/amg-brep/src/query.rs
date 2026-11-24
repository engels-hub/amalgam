//! Measurement queries (S02 §3, BREP-09): mass properties, point classification, distances,
//! ray casts, surface evaluation with principal curvatures, point projection onto a face, and
//! edge end tangents (S04 R-S04-1).
//!
//! The backend returns OCCT's raw values; this module applies the conventions (outward
//! normals, curvature signs, principal axes) in plain Rust, so they are unit-tested without
//! OCCT.

use amg_core::{Aabb3, Dir3, Mat3, Point2, Point3, Ray3, Tolerance, Vec3};

use crate::error::{KResult, KernelError};
use crate::geom::GeomClass;
use crate::shape::{EdgeId, FaceId, Shape, SubKind, SubShape};

/// Relative error bound of the adaptive `BRepGProp` integration behind
/// [`crate::BrepKernel::mass_props`]. Analytic primitives come out at round-off (≤ 1e-14
/// relative); faces bounded by curves that are not iso-lines (boolean results, B-spline
/// faces) converge to this bound instead of the fixed-order quadrature's 1e-4 (BREP-04
/// finding).
pub const MASS_PROPS_EPS: f64 = 1e-12;

/// Mass properties at unit density (S02 Interfaces).
///
/// For a shape with solids, `volume` (mm³) is over the closed shells only, and `centroid`,
/// `inertia` (mm⁵) and the principal values belong to the volume. A shape without solids has
/// `volume = 0` and the values of its faces at unit areal density (inertia in mm⁴), or, with
/// no faces either, of its edges at unit linear density (mm³). `area` is the summed face area.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MassProps {
    /// Volume in mm³ (0 without solids).
    pub volume: f64,
    /// Summed area of all faces in mm².
    pub area: f64,
    /// Centre of mass.
    pub centroid: Point3,
    /// Inertia tensor about `centroid` at unit density (`I_xx = ∫(y² + z²)`,
    /// `I_xy = −∫xy`); scale by the density in S05.
    pub inertia: Mat3,
    /// Eigenvalues of `inertia`, ascending.
    pub principal_moments: [f64; 3],
    /// Unit eigenvectors in the order of `principal_moments`, right-handed
    /// (`axes[2] = axes[0] × axes[1]`), each with its largest component positive (axes 0
    /// and 1). Within a repeated moment (a sphere, a cylinder's two radial axes) any
    /// orthonormal pair is correct; the result is still deterministic.
    pub principal_axes: [Dir3; 3],
}

/// Where a point lies relative to the solids of a shape.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Classification {
    /// Inside some solid.
    Inside,
    /// Outside every solid.
    Outside,
    /// On a boundary, within the given tolerance.
    OnBoundary,
}

/// A minimum distance and where it is attained.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DistanceResult {
    /// The distance in mm (0 where the shapes touch or overlap).
    pub distance: f64,
    /// Nearest point on the first shape.
    pub on_a: Point3,
    /// Nearest point on the second shape (or the query point).
    pub on_b: Point3,
    /// The face, edge or vertex of the first shape `on_a` lies on, when known.
    pub sub_a: Option<SubShape>,
    /// As `sub_a`, for the second shape.
    pub sub_b: Option<SubShape>,
}

/// The nearest ray hit.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RayHit {
    /// Distance along the ray (its direction is unit), in mm.
    pub t: f64,
    /// The hit point.
    pub point: Point3,
    /// The face hit.
    pub face: FaceId,
    /// Face parameters of the hit.
    pub uv: Point2,
    /// Outward face normal at the hit (flipped for a reversed face).
    pub normal: Dir3,
}

/// A face's surface at a parameter point, with principal curvatures (S05 INS-06, S20 R13).
///
/// `normal` points out of the material (flipped for a reversed face). Curvatures are signed
/// against it, **positive where the face is convex** (it bends away from its normal): a
/// sphere's outside face has `k = 1/r`, a hole's wall `−1/r`. `k_min ≤ k_max`; `dir_min` and
/// `dir_max` are unit tangents. At an umbilic point (sphere, plane) `dir_min` is the
/// normalised `d1u` and `dir_max = normal × dir_min`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SurfacePoint {
    /// The surface point.
    pub point: Point3,
    /// Outward unit normal.
    pub normal: Dir3,
    /// ∂S/∂u of the surface (not flipped).
    pub d1u: Vec3,
    /// ∂S/∂v of the surface (not flipped).
    pub d1v: Vec3,
    /// Smaller principal curvature in 1/mm.
    pub k_min: f64,
    /// Larger principal curvature in 1/mm.
    pub k_max: f64,
    /// Principal direction of `k_min`.
    pub dir_min: Vec3,
    /// Principal direction of `k_max`.
    pub dir_max: Vec3,
}

/// End points and unit tangents of an edge in its curve's parameter direction (the order of
/// [`crate::Shape::edge_vertices`] and [`crate::Shape::curve`]); S04 R-S04-1.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct EdgeEnds {
    /// Start point.
    pub p0: Point3,
    /// Unit tangent at the start.
    pub t0: Dir3,
    /// End point.
    pub p1: Point3,
    /// Unit tangent at the end.
    pub t1: Dir3,
}

// ----- raw backend values -------------------------------------------------------------------

/// What the mass properties were computed over.
#[cfg_attr(
    not(feature = "occt"),
    allow(dead_code, reason = "built by the OCCT backend")
)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum MassBasis {
    Volume,
    Surface,
    Linear,
    Nothing,
}

#[derive(Clone, Debug)]
pub(crate) struct RawMass {
    pub(crate) basis: MassBasis,
    pub(crate) volume: f64,
    pub(crate) area: f64,
    pub(crate) centroid: Point3,
    /// Row-major.
    pub(crate) inertia: [f64; 9],
}

#[derive(Clone, Debug)]
pub(crate) struct RawDistance {
    pub(crate) distance: f64,
    pub(crate) on_a: Point3,
    pub(crate) on_b: Point3,
    pub(crate) sub_a: Option<SubShape>,
    pub(crate) sub_b: Option<SubShape>,
    /// Face parameters of `on_a` when `sub_a` is a face (or after a face projection).
    pub(crate) uv_a: Point2,
}

#[derive(Clone, Debug)]
pub(crate) struct RawHit {
    pub(crate) t: f64,
    pub(crate) point: Point3,
    pub(crate) face: u32,
    pub(crate) uv: Point2,
}

#[derive(Clone, Debug)]
pub(crate) struct RawCurvature {
    pub(crate) k_min: f64,
    pub(crate) k_max: f64,
    /// `None` at an umbilic point.
    pub(crate) dirs: Option<(Vec3, Vec3)>,
}

/// OCCT's natural surface properties (normal `d1u × d1v`, curvatures signed so that a positive
/// value bends toward that normal).
#[derive(Clone, Debug)]
pub(crate) struct RawSurfacePoint {
    pub(crate) point: Point3,
    pub(crate) d1u: Vec3,
    pub(crate) d1v: Vec3,
    pub(crate) normal: Option<Vec3>,
    pub(crate) curvature: Option<RawCurvature>,
}

// ----- conventions ---------------------------------------------------------------------------

pub(crate) fn mass_props_from(raw: &RawMass) -> KResult<MassProps> {
    if raw.basis == MassBasis::Nothing {
        return Err(KernelError::InvalidInput {
            what: "shape",
            reason: "the shape has no solid, face or edge to measure".into(),
            offending: Vec::new(),
        });
    }
    let m = &raw.inertia;
    // Symmetrise (GProp's tensor is symmetric up to round-off).
    let a = [
        [m[0], 0.5 * (m[1] + m[3]), 0.5 * (m[2] + m[6])],
        [0.5 * (m[1] + m[3]), m[4], 0.5 * (m[5] + m[7])],
        [0.5 * (m[2] + m[6]), 0.5 * (m[5] + m[7]), m[8]],
    ];
    let inertia = Mat3::from_cols(
        Vec3::new(a[0][0], a[1][0], a[2][0]),
        Vec3::new(a[0][1], a[1][1], a[2][1]),
        Vec3::new(a[0][2], a[1][2], a[2][2]),
    );
    let (principal_moments, principal_axes) = principal(a);
    Ok(MassProps {
        volume: if raw.basis == MassBasis::Volume {
            raw.volume
        } else {
            0.0
        },
        area: raw.area,
        centroid: raw.centroid,
        inertia,
        principal_moments,
        principal_axes,
    })
}

/// Eigen-decomposition of a symmetric 3×3 matrix by cyclic Jacobi rotations: ascending
/// eigenvalues and right-handed unit eigenvectors (see [`MassProps::principal_axes`]).
pub(crate) fn principal(mut a: [[f64; 3]; 3]) -> ([f64; 3], [Dir3; 3]) {
    let scale = a[0][0].abs().max(a[1][1].abs()).max(a[2][2].abs());
    // Off-diagonal round-off of a tensor that is diagonal in theory (an axis-aligned body) would
    // otherwise turn its axes by an arbitrary angle within a repeated moment.
    for (p, q) in [(0, 1), (0, 2), (1, 2)] {
        if a[p][q].abs() <= 1e-13 * scale {
            a[p][q] = 0.0;
            a[q][p] = 0.0;
        }
    }
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _sweep in 0..64 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off <= f64::EPSILON * 1e-3 * scale || off == 0.0 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            // `signum(0.0)` is 1, so equal diagonal entries rotate by 45°.
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            // A ← Jᵀ A J for the rotation J in the (p, q) plane: columns, then rows.
            for row in &mut a {
                let (akp, akq) = (row[p], row[q]);
                row[p] = c * akp - s * akq;
                row[q] = s * akp + c * akq;
            }
            let (row_p, row_q) = (a[p], a[q]);
            for k in 0..3 {
                a[p][k] = c * row_p[k] - s * row_q[k];
                a[q][k] = s * row_p[k] + c * row_q[k];
            }
            for row in &mut v {
                let vp = row[p];
                let vq = row[q];
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    let mut order = [0usize, 1, 2];
    order.sort_by(|&i, &j| a[i][i].total_cmp(&a[j][j]).then(i.cmp(&j)));
    let moments = order.map(|i| a[i][i]);
    let column = |i: usize| Vec3::new(v[0][i], v[1][i], v[2][i]).normalize();
    let canonical = |axis: Vec3| {
        let largest = if axis.x.abs() >= axis.y.abs() && axis.x.abs() >= axis.z.abs() {
            axis.x
        } else if axis.y.abs() >= axis.z.abs() {
            axis.y
        } else {
            axis.z
        };
        if largest < 0.0 { -axis } else { axis }
    };
    let a0 = canonical(column(order[0]));
    let a1 = canonical(column(order[1]));
    let a2 = a0.cross(a1).normalize();
    (
        moments,
        [
            Dir3::new_unchecked(a0),
            Dir3::new_unchecked(a1),
            Dir3::new_unchecked(a2),
        ],
    )
}

/// Applies the face orientation and the convex-positive sign rule to OCCT's raw values.
pub(crate) fn surface_point_from(
    raw: &RawSurfacePoint,
    reversed: bool,
    f: FaceId,
    uv: Point2,
) -> KResult<SurfacePoint> {
    let undefined = |what: &str| KernelError::OpFailed {
        op: "eval_surface",
        message: format!("the {what} of {f:?} is undefined at ({}, {})", uv.x, uv.y),
        offending: vec![SubShape {
            kind: SubKind::Face,
            index: f.0,
        }],
    };
    let natural = raw.normal.ok_or_else(|| undefined("normal"))?;
    let curvature = raw
        .curvature
        .as_ref()
        .ok_or_else(|| undefined("curvature"))?;
    let sign = if reversed { -1.0 } else { 1.0 };
    let normal = (natural * sign).normalize();
    // OCCT: positive bends toward its natural normal. Here: positive bends away from the
    // outward normal. So k = −sign · k_occt, which swaps min and max when it negates.
    let ka = -sign * curvature.k_max;
    let kb = -sign * curvature.k_min;
    let (dir_occt_max, dir_occt_min) = match curvature.dirs {
        Some(dirs) => dirs,
        None => {
            let d = if raw.d1u.length() > 0.0 {
                raw.d1u
            } else {
                raw.d1v
            };
            let d = (d - normal * d.dot(normal)).normalize();
            (d, normal.cross(d))
        }
    };
    let (k_min, k_max, dir_min, dir_max) = if ka <= kb {
        (ka, kb, dir_occt_max, dir_occt_min)
    } else {
        (kb, ka, dir_occt_min, dir_occt_max)
    };
    Ok(SurfacePoint {
        point: raw.point,
        normal: Dir3::new_unchecked(normal),
        d1u: raw.d1u,
        d1v: raw.d1v,
        k_min,
        k_max,
        dir_min,
        dir_max,
    })
}

// ----- the queries ---------------------------------------------------------------------------

fn invalid(what: &'static str, reason: impl Into<String>) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason: reason.into(),
        offending: Vec::new(),
    }
}

fn face_in_range(s: &Shape, f: FaceId) -> KResult<u32> {
    let n = s.count(SubKind::Face);
    if f.0 < n {
        Ok(f.0)
    } else {
        Err(invalid(
            "face",
            format!("{f:?} is out of range ({n} faces)"),
        ))
    }
}

fn not_null(what: &'static str, s: &Shape) -> KResult<()> {
    if s.is_null() {
        Err(invalid(what, "the shape is null"))
    } else {
        Ok(())
    }
}

fn result_from(raw: RawDistance) -> DistanceResult {
    DistanceResult {
        distance: raw.distance,
        on_a: raw.on_a,
        on_b: raw.on_b,
        sub_a: raw.sub_a,
        sub_b: raw.sub_b,
    }
}

pub(crate) fn mass_props(s: &Shape) -> KResult<MassProps> {
    not_null("shape", s)?;
    mass_props_from(&s.occ().mass_props(MASS_PROPS_EPS)?)
}

pub(crate) fn classify(s: &Shape, p: Point3, tol: f64) -> KResult<Classification> {
    not_null("shape", s)?;
    if !(tol.is_finite() && tol >= 0.0) {
        return Err(invalid(
            "tol",
            format!("{tol} is not a finite tolerance ≥ 0"),
        ));
    }
    if !p.is_finite() {
        return Err(invalid("point", "the point is not finite"));
    }
    if !s.occ().has_solids() {
        return Err(invalid("shape", "classify needs a shape with a solid"));
    }
    s.occ().classify(p, tol)
}

pub(crate) fn distance(a: &Shape, b: &Shape) -> KResult<DistanceResult> {
    not_null("shapes", a)?;
    not_null("shapes", b)?;
    Ok(result_from(a.occ().distance(b.occ())?))
}

pub(crate) fn closest_point(s: &Shape, p: Point3) -> KResult<DistanceResult> {
    not_null("shape", s)?;
    if !p.is_finite() {
        return Err(invalid("point", "the point is not finite"));
    }
    if s.count(SubKind::Vertex) == 0 {
        return Err(invalid("shape", "the shape is empty"));
    }
    let raw = s.occ().closest_point(p)?;
    Ok(DistanceResult {
        on_b: p,
        sub_b: None,
        ..result_from(raw)
    })
}

pub(crate) fn ray_cast(
    s: &Shape,
    ray: &Ray3,
    max_t: f64,
    tol: &Tolerance,
) -> KResult<Option<RayHit>> {
    not_null("shape", s)?;
    if max_t.is_nan() || max_t < 0.0 {
        return Err(invalid("max_t", format!("{max_t} is not ≥ 0")));
    }
    if !ray.origin.is_finite() || !ray.dir.get().is_finite() {
        return Err(invalid("ray", "the ray is not finite"));
    }
    // IntCurvesFace takes a finite range; past the shape's bbox there is nothing to hit.
    let bbox = s.bbox();
    if bbox.is_empty() {
        return Ok(None);
    }
    let reach = (bbox.center() - ray.origin).length() + bbox.diagonal() + 1.0;
    let max_t = max_t.min(reach);
    let Some(hit) = s
        .occ()
        .ray_cast(ray.origin, ray.dir.get(), max_t, tol.linear)?
    else {
        return Ok(None);
    };
    let face = FaceId(hit.face);
    let raw = s.occ().eval_face(hit.face, hit.uv, tol.linear)?;
    let sign = if s.face_reversed(face) { -1.0 } else { 1.0 };
    let normal = raw
        .normal
        .map_or(-ray.dir.get(), |n| (n * sign).normalize());
    Ok(Some(RayHit {
        t: hit.t,
        point: hit.point,
        face,
        uv: hit.uv,
        normal: Dir3::new_unchecked(normal),
    }))
}

pub(crate) fn eval_surface(
    s: &Shape,
    f: FaceId,
    uv: Point2,
    tol: &Tolerance,
) -> KResult<SurfacePoint> {
    not_null("shape", s)?;
    let face = face_in_range(s, f)?;
    if !uv.is_finite() {
        return Err(invalid("uv", "the parameters are not finite"));
    }
    let raw = s.occ().eval_face(face, uv, tol.linear)?;
    surface_point_from(&raw, s.face_reversed(f), f, uv)
}

pub(crate) fn project_point_on_face(
    s: &Shape,
    f: FaceId,
    p: Point3,
    tol: &Tolerance,
) -> KResult<(Point2, Point3)> {
    not_null("shape", s)?;
    let face = face_in_range(s, f)?;
    if !p.is_finite() {
        return Err(invalid("point", "the point is not finite"));
    }
    let raw = s.occ().project_on_face(face, p, tol.linear)?;
    Ok((raw.uv_a, raw.on_a))
}

pub(crate) fn edge_ends(s: &Shape, e: EdgeId) -> KResult<EdgeEnds> {
    not_null("shape", s)?;
    let n = s.count(SubKind::Edge);
    if e.0 >= n {
        return Err(invalid(
            "edge",
            format!("{e:?} is out of range ({n} edges)"),
        ));
    }
    s.occ()
        .edge_ends(e.0)?
        .ok_or_else(|| KernelError::InvalidInput {
            what: "edge",
            reason: format!("{e:?} is degenerated or has no tangent at an end"),
            offending: vec![SubShape {
                kind: SubKind::Edge,
                index: e.0,
            }],
        })
}

/// A unit tangent from a first derivative, or `None` where it vanishes.
#[cfg_attr(
    not(feature = "occt"),
    allow(dead_code, reason = "used by the OCCT backend")
)]
pub(crate) fn tangent(d: Vec3) -> Option<Dir3> {
    let len = d.length();
    (len > 1e-300 && len.is_finite()).then(|| Dir3::new_unchecked(d / len))
}

/// Geometric fingerprint of one face, edge or vertex (S02 §3, BREP-05): the input of S01's
/// `GeomFingerprint` and of the split order ([`crate::split_order_key`]).
///
/// Values come from one deterministic kernel build: fixed-order `BRepGProp` quadrature for
/// `measure` and `centroid` (fast, and the same digits on every run; not the adaptive
/// [`crate::BrepKernel::mass_props`] accuracy), and `BRepBndLib::AddOptimal` without the
/// tolerances for `bbox`.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SubShapeGeom {
    /// Surface class (faces), curve class (edges: `Line`, `Circle`, `Ellipse`, else `Curve`) or
    /// `Point` (vertices); the same classes as [`crate::geom::Surface::class`].
    pub class: GeomClass,
    /// Face area mm², edge length mm, 0 for vertices and degenerated edges.
    pub measure: f64,
    /// Area- or length-weighted centre; a vertex's point; a degenerated edge's vertex.
    pub centroid: Point3,
    /// The outward normal of a plane face (flipped on a reversed face), the axis of a cylinder,
    /// cone or torus face, the direction of a line edge, the normal of a circle or ellipse edge;
    /// `None` otherwise (spheres, B-splines, other curves, vertices).
    pub axis: Option<Vec3>,
    /// Axis-aligned bounds, without the sub-shape's tolerance.
    pub bbox: Aabb3,
}

/// Fingerprints of the sub-shapes `indices` of `kind`, in that order.
pub(crate) fn sub_geoms(s: &Shape, kind: SubKind, indices: &[u32]) -> KResult<Vec<SubShapeGeom>> {
    not_null("shape", s)?;
    let n = s.count(kind);
    if let Some(&bad) = indices.iter().find(|&&i| i >= n) {
        return Err(KernelError::InvalidInput {
            what: "sub",
            reason: format!("{kind:?} {bad} is out of range ({n} in the shape)"),
            offending: vec![SubShape { kind, index: bad }],
        });
    }
    if indices.is_empty() {
        return Ok(Vec::new());
    }
    s.occ().sub_geoms(kind, indices)
}

pub(crate) fn sub_geom(s: &Shape, sub: SubShape) -> KResult<SubShapeGeom> {
    let mut one = sub_geoms(s, sub.kind, &[sub.index])?;
    Ok(one.remove(0))
}

/// Sub-shapes per fingerprint call of `sub_geom_all`, per kind. A face costs 5–100 µs (its
/// fixed-order `BRepGProp` integration grows with its edges: a cap with 1024 holes takes
/// ~10 ms), an edge ~1.5 µs, a vertex ~0.2 µs; the FFI call itself ~1 µs.
const GEOM_CHUNK: [usize; 3] = [4, 128, 512];

/// Fingerprints of every sub-shape. The work is spread over the current rayon pool: faces in
/// small groups, most edges first (so a face with many hole loops starts at once instead of
/// ending a long chunk), edges and vertices in larger chunks. The values do not depend on the
/// schedule.
pub(crate) fn sub_geom_all(s: &Shape) -> KResult<[Vec<SubShapeGeom>; 3]> {
    use rayon::prelude::*;
    not_null("shape", s)?;
    let kinds = [SubKind::Face, SubKind::Edge, SubKind::Vertex];
    let mut faces: Vec<u32> = (0..s.count(SubKind::Face)).collect();
    faces.sort_by_key(|&f| std::cmp::Reverse(s.face_edges(FaceId(f)).len()));
    let lists = [
        faces,
        (0..s.count(SubKind::Edge)).collect(),
        (0..s.count(SubKind::Vertex)).collect::<Vec<u32>>(),
    ];
    let tasks: Vec<(usize, &[u32])> = lists
        .iter()
        .enumerate()
        .flat_map(|(k, list)| list.chunks(GEOM_CHUNK[k]).map(move |c| (k, c)))
        .collect();
    let parts: Vec<KResult<Vec<SubShapeGeom>>> = tasks
        .par_iter()
        .with_max_len(1)
        .map(|(k, indices)| s.occ().sub_geoms(kinds[*k], indices))
        .collect();
    let mut slots: [Vec<Option<SubShapeGeom>>; 3] =
        std::array::from_fn(|k| vec![None; lists[k].len()]);
    for ((k, indices), part) in tasks.iter().zip(parts) {
        for (&i, g) in indices.iter().zip(part?) {
            slots[*k][i as usize] = Some(g);
        }
    }
    let mut out: [Vec<SubShapeGeom>; 3] = Default::default();
    for (k, slot) in slots.into_iter().enumerate() {
        out[k] = slot.into_iter().flatten().collect();
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "test code")]

    use super::*;

    fn rot(axis: Vec3, angle: f64) -> Mat3 {
        Mat3::from_axis_angle(axis.normalize(), angle)
    }

    fn check_decomposition(m: Mat3) {
        let a = [
            [m.x_axis.x, m.y_axis.x, m.z_axis.x],
            [m.x_axis.y, m.y_axis.y, m.z_axis.y],
            [m.x_axis.z, m.y_axis.z, m.z_axis.z],
        ];
        let (moments, axes) = principal(a);
        assert!(moments[0] <= moments[1] && moments[1] <= moments[2]);
        let scale = moments[2].abs().max(1.0);
        for (k, axis) in axes.iter().enumerate() {
            let v = axis.get();
            assert!((v.length() - 1.0).abs() < 1e-14);
            let r = m * v - v * moments[k];
            assert!(r.length() <= 1e-12 * scale, "residual {r:?}");
        }
        let det = axes[0].get().cross(axes[1].get()).dot(axes[2].get());
        assert!((det - 1.0).abs() < 1e-12);
    }

    #[test]
    fn diagonal_tensor_keeps_world_axes() {
        let (moments, axes) = principal([[3.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 2.0]]);
        assert_eq!(moments.map(f64::to_bits), [1.0, 2.0, 3.0].map(f64::to_bits));
        assert_eq!(axes[0].get(), Vec3::Y);
        assert_eq!(axes[1].get(), Vec3::Z);
        assert_eq!(axes[2].get(), Vec3::X);
    }

    #[test]
    fn repeated_moments_with_round_off_keep_world_axes() {
        let e = 1e-15 * 400.0;
        let (moments, axes) = principal([[400.0, e, 0.0], [e, 400.0, -e], [0.0, -e, 400.0]]);
        assert_eq!(moments.map(f64::to_bits), [400.0_f64; 3].map(f64::to_bits));
        assert_eq!(axes.map(Dir3::get), [Vec3::X, Vec3::Y, Vec3::Z]);
    }

    #[test]
    fn rotated_tensors_decompose() {
        let d = Mat3::from_diagonal(Vec3::new(1.0, 5.0, 9.0));
        for (axis, angle) in [
            (Vec3::new(1.0, 2.0, 3.0), 0.7),
            (Vec3::new(-1.0, 0.5, 0.2), 2.9),
            (Vec3::Z, 1e-9),
        ] {
            let r = rot(axis, angle);
            check_decomposition(r * d * r.transpose());
        }
        // A repeated pair, rotated.
        let d = Mat3::from_diagonal(Vec3::new(2.0, 2.0, 7.0));
        let r = rot(Vec3::new(0.3, -0.4, 1.0), 1.1);
        check_decomposition(r * d * r.transpose());
    }

    #[test]
    fn mass_props_need_something_to_measure() {
        let raw = RawMass {
            basis: MassBasis::Nothing,
            volume: 0.0,
            area: 0.0,
            centroid: Point3::ZERO,
            inertia: [0.0; 9],
        };
        assert!(matches!(
            mass_props_from(&raw),
            Err(KernelError::InvalidInput { what: "shape", .. })
        ));
    }

    fn sphere_raw(k_occt: f64) -> RawSurfacePoint {
        RawSurfacePoint {
            point: Point3::new(10.0, 0.0, 0.0),
            d1u: Vec3::new(0.0, 10.0, 0.0),
            d1v: Vec3::new(0.0, 0.0, 10.0),
            normal: Some(Vec3::X),
            curvature: Some(RawCurvature {
                k_min: k_occt,
                k_max: k_occt,
                dirs: None,
            }),
        }
    }

    #[test]
    fn reversed_faces_flip_normal_and_curvature() {
        // OCCT: a sphere's natural normal points out and its curvature (toward the normal) is
        // −1/r. Seen from outside it is convex: +1/r.
        let uv = Point2::ZERO;
        let out = surface_point_from(&sphere_raw(-0.1), false, FaceId(0), uv).unwrap();
        assert_eq!(out.normal.get(), Vec3::X);
        assert!((out.k_min - 0.1).abs() < 1e-15 && (out.k_max - 0.1).abs() < 1e-15);
        assert_eq!(out.dir_min, Vec3::Y);
        assert_eq!(out.dir_max, Vec3::Z);
        // The same surface as the wall of a spherical cavity (reversed face).
        let cavity = surface_point_from(&sphere_raw(-0.1), true, FaceId(0), uv).unwrap();
        assert_eq!(cavity.normal.get(), -Vec3::X);
        assert!((cavity.k_max + 0.1).abs() < 1e-15);
    }

    #[test]
    fn principal_directions_follow_the_sign_flip() {
        let raw = RawSurfacePoint {
            point: Point3::ZERO,
            d1u: Vec3::X,
            d1v: Vec3::Y,
            normal: Some(Vec3::Z),
            curvature: Some(RawCurvature {
                k_min: -0.5,
                k_max: 0.25,
                dirs: Some((Vec3::X, Vec3::Y)),
            }),
        };
        let p = surface_point_from(&raw, false, FaceId(3), Point2::ZERO).unwrap();
        assert_eq!((p.k_min, p.k_max), (-0.25, 0.5));
        assert_eq!((p.dir_min, p.dir_max), (Vec3::X, Vec3::Y));
        let undefined = RawSurfacePoint {
            normal: None,
            ..raw
        };
        assert!(matches!(
            surface_point_from(&undefined, false, FaceId(3), Point2::ZERO),
            Err(KernelError::OpFailed {
                op: "eval_surface",
                ..
            })
        ));
    }
}
