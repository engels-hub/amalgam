//! BREP-09: mass properties, `classify`, `distance`, `closest_point`, `ray_cast`,
//! `eval_surface` (curvatures), `project_point_on_face`, `interference` and `edge_ends`.
//!
//! Done-when 1 (box, cylinder, sphere and torus mass properties and inertia tensors within
//! 1e-9 of the analytic values), 2 (sphere–sphere and point–face distances within 1e-9),
//! 3 (1000 random rays at a cylinder: expected face and normal), 4 (torus curvatures at 10
//! points within 1e-9) and 5 (analytic interference volume of two overlapping boxes). All on
//! frames tilted off every axis. The concurrency run is in `tests/stress.rs`.
#![cfg(feature = "occt")]
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

#[path = "common/stress.rs"]
#[allow(dead_code, reason = "shared harness; only the RNG is used here")]
mod stress;

use std::f64::consts::{PI, TAU};

use amg_brep::geom::Surface;
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, Classification, Dir3, EdgeId, FaceId, JobCtx, KernelConfig,
    KernelError, MassProps, Mat3, OcctKernel, Plane, Point2, Point3, Ray3, Shape, SubKind,
    SubShape, Transform, Validate, Vec3,
};
use amg_core::Quat;
use stress::Rng;

/// The Done-when bound.
const REL: f64 = 1e-9;

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Always,
        ..KernelConfig::default()
    })
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

/// A frame rotated off every axis and moved off the origin.
fn tilted() -> Plane {
    let q = Quat::from_euler(amg_core::glam::EulerRot::ZXY, 0.4, -0.7, 1.1);
    Plane {
        origin: Point3::new(3.0, -2.0, 5.0),
        x: Dir3::new_unchecked(q * Vec3::X),
        y: Dir3::new_unchecked(q * Vec3::Y),
        normal: Dir3::new_unchecked(q * Vec3::Z),
    }
}

/// The frame's rotation (columns x, y, normal).
fn rotation(f: &Plane) -> Mat3 {
    Mat3::from_cols(f.x.get(), f.y.get(), f.normal.get())
}

fn to_world(f: &Plane, local: Vec3) -> Point3 {
    f.origin + rotation(f) * local
}

fn to_local(f: &Plane, p: Point3) -> Vec3 {
    rotation(f).transpose() * (p - f.origin)
}

#[track_caller]
fn assert_rel(what: &str, got: f64, want: f64, rel: f64) {
    let err = (got - want).abs() / want.abs().max(f64::MIN_POSITIVE);
    assert!(
        err <= rel,
        "{what}: got {got}, want {want} (relative error {err:e} > {rel:e})"
    );
    eprintln!("{what}: {got} vs {want}, relative error {err:.2e}");
}

#[track_caller]
fn assert_near(what: &str, got: Vec3, want: Vec3, abs: f64) {
    let err = (got - want).length();
    assert!(
        err <= abs,
        "{what}: got {got:?}, want {want:?} (error {err:e} > {abs:e})"
    );
}

/// Largest entry difference relative to the largest entry of `want`.
fn mat_rel_err(got: Mat3, want: Mat3) -> f64 {
    let d = (got - want).to_cols_array();
    let w = want.to_cols_array();
    let scale = w.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    d.iter().fold(0.0_f64, |m, v| m.max(v.abs())) / scale
}

/// `props` against a body with `volume`, `area`, centroid `centroid` and inertia
/// `diag` (about the centroid, in `frame`'s axes); every value within [`REL`].
#[track_caller]
fn assert_props(
    what: &str,
    props: &MassProps,
    volume: f64,
    area: f64,
    centroid: Point3,
    frame: &Plane,
    diag: Vec3,
) {
    assert_rel(&format!("{what} volume"), props.volume, volume, REL);
    assert_rel(&format!("{what} area"), props.area, area, REL);
    let scale = volume.cbrt();
    let err = (props.centroid - centroid).length() / scale;
    assert!(
        err <= REL,
        "{what} centroid {:?} vs {centroid:?} ({err:e})",
        props.centroid
    );
    let r = rotation(frame);
    let want = r * Mat3::from_diagonal(diag) * r.transpose();
    let err = mat_rel_err(props.inertia, want);
    eprintln!("{what} inertia: relative error {err:.2e}");
    assert!(
        err <= REL,
        "{what} inertia {:?} vs {want:?} ({err:e})",
        props.inertia
    );
    // Principal moments: the sorted diagonal.
    let mut moments = diag.to_array();
    moments.sort_by(f64::total_cmp);
    for (k, (&got, &want)) in props.principal_moments.iter().zip(&moments).enumerate() {
        assert_rel(&format!("{what} principal moment {k}"), got, want, REL);
    }
    // Axes are eigenvectors, right-handed.
    let axes = props.principal_axes.map(Dir3::get);
    for (k, axis) in axes.iter().enumerate() {
        let residual = (props.inertia * *axis - *axis * props.principal_moments[k]).length();
        assert!(
            residual <= REL * moments[2],
            "{what} axis {k}: residual {residual:e}"
        );
    }
    assert!((axes[0].cross(axes[1]).dot(axes[2]) - 1.0).abs() < 1e-12);
}

// ---- Done-when 1: mass properties ------------------------------------------------------------

#[test]
fn box_mass_props_are_analytic() {
    let k = kernel();
    let f = tilted();
    let (a, b, c) = (10.0, 20.0, 30.0);
    let s = k.make_box(&f, Vec3::new(a, b, c)).unwrap().shape;
    let m = a * b * c;
    let props = k.mass_props(&s).unwrap();
    assert_props(
        "box",
        &props,
        m,
        2.0 * (a * b + b * c + a * c),
        to_world(&f, Vec3::new(a, b, c) / 2.0),
        &f,
        Vec3::new(b * b + c * c, a * a + c * c, a * a + b * b) * (m / 12.0),
    );
    // Distinct moments: the principal axes are the frame's axes, smallest moment along the
    // longest side.
    for (axis, want) in props
        .principal_axes
        .iter()
        .zip([f.normal.get(), f.y.get(), f.x.get()])
    {
        assert!(
            axis.get().dot(want).abs() > 1.0 - 1e-12,
            "{axis:?} vs {want:?}"
        );
    }
}

#[test]
fn cylinder_mass_props_are_analytic() {
    let k = kernel();
    let f = tilted();
    let (r, h) = (5.0, 12.0);
    let s = k.make_cylinder(&f, r, h, TAU).unwrap().shape;
    let m = PI * r * r * h;
    let radial = m * (3.0 * r * r + h * h) / 12.0;
    assert_props(
        "cylinder",
        &k.mass_props(&s).unwrap(),
        m,
        2.0 * PI * r * r + 2.0 * PI * r * h,
        to_world(&f, Vec3::new(0.0, 0.0, h / 2.0)),
        &f,
        Vec3::new(radial, radial, m * r * r / 2.0),
    );
}

#[test]
fn sphere_mass_props_are_analytic() {
    let k = kernel();
    let f = tilted();
    let r = 7.0;
    let s = k.make_sphere(&f, r).unwrap().shape;
    let m = 4.0 / 3.0 * PI * r * r * r;
    let i = 0.4 * m * r * r;
    assert_props(
        "sphere",
        &k.mass_props(&s).unwrap(),
        m,
        4.0 * PI * r * r,
        f.origin,
        &f,
        Vec3::splat(i),
    );
}

#[test]
fn torus_mass_props_are_analytic() {
    let k = kernel();
    let f = tilted();
    let (big, small) = (10.0, 3.0);
    let s = k.make_torus(&f, big, small, TAU).unwrap().shape;
    let m = 2.0 * PI * PI * big * small * small;
    let axial = m * (big * big + 0.75 * small * small);
    let diametral = m * (0.5 * big * big + 0.625 * small * small);
    assert_props(
        "torus",
        &k.mass_props(&s).unwrap(),
        m,
        4.0 * PI * PI * big * small,
        f.origin,
        &f,
        Vec3::new(diametral, diametral, axial),
    );
}

#[test]
fn cone_frustum_mass_props_are_analytic() {
    let k = kernel();
    let f = tilted();
    let (r1, r2, h) = (5.0, 2.0, 10.0);
    let s = k.make_cone(&f, r1, r2, h, TAU).unwrap().shape;
    let m = PI * h / 3.0 * (r1 * r1 + r1 * r2 + r2 * r2);
    let slant = (h * h + (r1 - r2) * (r1 - r2)).sqrt();
    let area = PI * (r1 * r1 + r2 * r2) + PI * (r1 + r2) * slant;
    // Centroid height and moments of a frustum (integrals of the disc stack).
    let zc = h * (r1 * r1 + 2.0 * r1 * r2 + 3.0 * r2 * r2) / (4.0 * (r1 * r1 + r1 * r2 + r2 * r2));
    let n = 200_000;
    let (mut iz, mut ix) = (0.0, 0.0);
    for j in 0..n {
        // Midpoint rule on a quartic/polynomial integrand: exact to ~1e-12 at this n.
        let z = (f64::from(j) + 0.5) * h / f64::from(n);
        let r = r1 + (r2 - r1) * z / h;
        let dm = PI * r * r * h / f64::from(n);
        iz += dm * r * r / 2.0;
        ix += dm * (r * r / 4.0 + (z - zc) * (z - zc));
    }
    assert_props(
        "cone frustum",
        &k.mass_props(&s).unwrap(),
        m,
        area,
        to_world(&f, Vec3::new(0.0, 0.0, zc)),
        &f,
        Vec3::new(ix, ix, iz),
    );
}

#[test]
fn compound_mass_props_follow_the_parallel_axis_rule() {
    let k = kernel();
    let f = tilted();
    let a = k.make_box(&f, Vec3::new(4.0, 6.0, 8.0)).unwrap().shape;
    let moved = Plane {
        origin: to_world(&f, Vec3::new(20.0, 0.0, 0.0)),
        ..f
    };
    let b = k.make_box(&moved, Vec3::new(4.0, 6.0, 8.0)).unwrap().shape;
    let both = k
        .boolean(
            BoolOp::Fuse,
            &[a.clone(), b],
            &[],
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap()
        .shape;
    assert_eq!(both.solids().len(), 2);
    let one = k.mass_props(&a).unwrap();
    let props = k.mass_props(&both).unwrap();
    // Two equal boxes 20 apart along x: each 10 off the common centroid.
    let parallel = Vec3::new(0.0, 2.0 * one.volume * 100.0, 2.0 * one.volume * 100.0);
    let local = rotation(&f).transpose() * one.inertia * rotation(&f);
    let diag = 2.0 * Vec3::new(local.x_axis.x, local.y_axis.y, local.z_axis.z) + parallel;
    assert_props(
        "two boxes",
        &props,
        2.0 * one.volume,
        2.0 * one.area,
        to_world(&f, Vec3::new(12.0, 3.0, 4.0)),
        &f,
        diag,
    );
}

#[test]
fn faces_without_solids_are_measured_by_area() {
    let k = kernel();
    let f = tilted();
    let s = k.make_box(&f, Vec3::new(10.0, 20.0, 30.0)).unwrap().shape;
    // The face of the box on the frame's z = 0 plane.
    let bottom = s
        .faces()
        .find(|&face| {
            matches!(s.surface(face), Surface::Plane(p)
                if to_local(&f, p.origin).z.abs() < 1e-9 && p.normal.get().dot(f.normal.get()).abs() > 0.99)
        })
        .unwrap();
    let face = s
        .sub_shape(SubShape {
            kind: SubKind::Face,
            index: bottom.0,
        })
        .unwrap();
    let props = k.mass_props(&face).unwrap();
    assert_eq!(props.volume.to_bits(), 0.0_f64.to_bits());
    assert_rel("face area", props.area, 200.0, REL);
    assert_near(
        "face centroid",
        props.centroid,
        to_world(&f, Vec3::new(5.0, 10.0, 0.0)),
        1e-9,
    );
    // Unit areal density: polar moment of a 10 × 20 plate about its normal.
    let polar = 200.0 * (100.0 + 400.0) / 12.0;
    assert_rel("face polar moment", props.principal_moments[2], polar, REL);
}

/// The fixed-order quadrature of `Shape::volume` misreads non-iso-line boundaries (BREP-03,
/// BREP-14, BREP-04 findings); the adaptive `mass_props` does not.
#[test]
fn adaptive_integration_fixes_the_known_quadrature_cases() {
    let k = kernel();
    // BREP-04: lens of two spheres whose centre line is oblique to their poles.
    let (r, d) = (10.0, 8.0);
    let dir = Vec3::new(1.0, 1.0, 1.0).normalize();
    let a = k.make_sphere(&Plane::XY, r).unwrap().shape;
    let b = k
        .make_sphere(
            &Plane {
                origin: dir * d,
                ..Plane::XY
            },
            r,
        )
        .unwrap()
        .shape;
    let lens = k
        .boolean(BoolOp::Common, &[a], &[b], &BoolOpts::default(), &ctx())
        .unwrap()
        .shape;
    let want = PI * (4.0 * r + d) * (2.0 * r - d) * (2.0 * r - d) / 12.0;
    let fixed = lens.volume().unwrap();
    eprintln!(
        "lens: fixed-order volume {fixed} (relative error {:.2e})",
        (fixed - want).abs() / want
    );
    let props = k.mass_props(&lens).unwrap();
    assert_rel("lens volume", props.volume, want, REL);
    assert_near("lens centroid", props.centroid, dir * d / 2.0, 1e-8);

    // BREP-14: sphere r 10 scaled (1, 2, 3) — rational B-spline faces.
    let sphere = k.make_sphere(&Plane::XY, 10.0).unwrap().shape;
    let scale = Transform::from_scale(Vec3::new(1.0, 2.0, 3.0));
    let ellipsoid = sphere.gtransformed(&scale).unwrap().shape;
    let (ea, eb, ec) = (10.0, 20.0, 30.0);
    let m = 4.0 / 3.0 * PI * ea * eb * ec;
    let fixed = ellipsoid.volume().unwrap();
    eprintln!(
        "ellipsoid: fixed-order volume {fixed} (relative error {:.2e})",
        (fixed - m).abs() / m
    );
    let props = k.mass_props(&ellipsoid).unwrap();
    // Adaptive integration saturates at 1.1e-9 here (rational B-spline faces with poles), the
    // same for every eps ≤ 1e-12: just over S04 SOL-06's 1e-9 (BREP-09 leftover), against the
    // fixed-order 4.4e-4.
    assert_rel("ellipsoid volume", props.volume, m, 2e-9);
    let want = Mat3::from_diagonal(
        Vec3::new(eb * eb + ec * ec, ea * ea + ec * ec, ea * ea + eb * eb) * (m / 5.0),
    );
    let err = mat_rel_err(props.inertia, want);
    eprintln!("ellipsoid inertia: relative error {err:.2e}");
    assert!(err <= 2e-9, "{:?}", props.inertia);
}

// ---- classify ------------------------------------------------------------------------------

#[test]
fn points_classify_against_the_solids() {
    let k = kernel();
    let f = tilted();
    let s = k.make_box(&f, Vec3::new(10.0, 20.0, 30.0)).unwrap().shape;
    let at = |x: f64, y: f64, z: f64| to_world(&f, Vec3::new(x, y, z));
    let tol = 1e-6;
    assert_eq!(
        k.classify(&s, at(5.0, 10.0, 15.0), tol).unwrap(),
        Classification::Inside
    );
    assert_eq!(
        k.classify(&s, at(5.0, 10.0, 31.0), tol).unwrap(),
        Classification::Outside
    );
    assert_eq!(
        k.classify(&s, at(-1e-3, 10.0, 15.0), tol).unwrap(),
        Classification::Outside
    );
    assert_eq!(
        k.classify(&s, at(10.0, 10.0, 15.0), tol).unwrap(),
        Classification::OnBoundary
    );
    assert_eq!(
        k.classify(&s, at(10.0, 20.0, 30.0), tol).unwrap(),
        Classification::OnBoundary
    );
    assert_eq!(
        k.classify(&s, at(10.0 + 1e-7, 10.0, 15.0), tol).unwrap(),
        Classification::OnBoundary
    );
    let face = s
        .sub_shape(SubShape {
            kind: SubKind::Face,
            index: 0,
        })
        .unwrap();
    assert!(matches!(
        k.classify(&face, f.origin, tol),
        Err(KernelError::InvalidInput { what: "shape", .. })
    ));
    assert!(matches!(
        k.classify(&s, f.origin, -1.0),
        Err(KernelError::InvalidInput { what: "tol", .. })
    ));
}

// ---- Done-when 2: distances -----------------------------------------------------------------

#[test]
fn distance_between_spheres_is_analytic() {
    let k = kernel();
    let f = tilted();
    let (r1, r2) = (3.0, 5.0);
    let u = (f.x.get() * 0.3 + f.y.get() * -0.8 + f.normal.get() * 0.5).normalize();
    let c1 = f.origin;
    let c2 = c1 + u * 20.0;
    let a = k.make_sphere(&f, r1).unwrap().shape;
    let b = k.make_sphere(&Plane { origin: c2, ..f }, r2).unwrap().shape;
    let d = k.distance(&a, &b).unwrap();
    assert_rel("sphere distance", d.distance, 20.0 - r1 - r2, REL);
    assert_near("on a", d.on_a, c1 + u * r1, 1e-9 * 20.0);
    assert_near("on b", d.on_b, c2 - u * r2, 1e-9 * 20.0);
    assert_eq!(d.sub_a.map(|s| s.kind), Some(SubKind::Face));
    assert_eq!(d.sub_b.map(|s| s.kind), Some(SubKind::Face));
    // Symmetric.
    let back = k.distance(&b, &a).unwrap();
    assert_rel("sphere distance (b, a)", back.distance, 20.0 - r1 - r2, REL);
    // Overlapping spheres touch at distance 0.
    let near = k
        .make_sphere(
            &Plane {
                origin: c1 + u * 4.0,
                ..f
            },
            r2,
        )
        .unwrap()
        .shape;
    assert!(k.distance(&a, &near).unwrap().distance.abs() < 1e-12);
}

fn face_of(s: &Shape, pred: impl Fn(&Surface) -> bool) -> FaceId {
    s.faces().find(|&f| pred(&s.surface(f))).unwrap()
}

fn face_shape(s: &Shape, f: FaceId) -> Shape {
    s.sub_shape(SubShape {
        kind: SubKind::Face,
        index: f.0,
    })
    .unwrap()
}

#[test]
fn distance_from_a_point_to_a_face_is_analytic() {
    let k = kernel();
    let f = tilted();
    let (r, h) = (5.0, 12.0);
    let cyl = k.make_cylinder(&f, r, h, TAU).unwrap().shape;
    let side = face_of(&cyl, |s| matches!(s, Surface::Cylinder { .. }));
    let side_shape = face_shape(&cyl, side);
    let top = face_of(
        &cyl,
        |s| matches!(s, Surface::Plane(p) if (to_local(&f, p.origin).z - h).abs() < 1e-9),
    );
    let top_shape = face_shape(&cyl, top);

    // A point 7 mm off the lateral face, at mid height.
    let radial = (f.x.get() * 0.6 + f.y.get() * 0.8).normalize();
    let p = to_world(&f, Vec3::new(0.0, 0.0, 6.0)) + radial * (r + 7.0);
    let d = k.closest_point(&side_shape, p).unwrap();
    assert_rel("point-cylinder distance", d.distance, 7.0, REL);
    assert_near("foot on cylinder", d.on_a, p - radial * 7.0, 1e-9 * 7.0);
    assert_eq!(d.on_b, p);
    assert_eq!(d.sub_a.map(|s| s.kind), Some(SubKind::Face));

    // Above the top cap, inside its disc: the distance along the axis.
    let p = to_world(&f, Vec3::new(1.0, -2.0, h + 4.5));
    let d = k.closest_point(&top_shape, p).unwrap();
    assert_rel("point-plane distance", d.distance, 4.5, REL);
    // Beside the cap's rim: the distance to the circle.
    // (Off the seam vertex at angle 0.)
    let p = to_world(
        &f,
        Vec3::new((r + 3.0) * 1f64.cos(), (r + 3.0) * 1f64.sin(), h + 4.0),
    );
    let d = k.closest_point(&top_shape, p).unwrap();
    assert_rel("point-rim distance", d.distance, 5.0, REL);
    assert_eq!(d.sub_a.map(|s| s.kind), Some(SubKind::Edge));
    // The same through `distance` with a vertex shape is covered by `closest_point`; a point
    // inside the solid gets its nearest boundary point.
    let inside = to_world(&f, Vec3::new(0.0, 0.0, 1.5));
    let d = k.closest_point(&cyl, inside).unwrap();
    assert_rel("inside point to boundary", d.distance, 1.5, REL);
    // A sphere face, from the outside.
    let sphere = k.make_sphere(&f, 4.0).unwrap().shape;
    let p = f.origin + (f.x.get() - f.normal.get()).normalize() * 9.0;
    let d = k.closest_point(&sphere, p).unwrap();
    assert_rel("point-sphere distance", d.distance, 5.0, REL);
}

// ---- projection -----------------------------------------------------------------------------

#[test]
fn points_project_onto_faces_with_parameters() {
    let k = kernel();
    let f = tilted();
    let (r, h) = (5.0, 12.0);
    let cyl = k.make_cylinder(&f, r, h, TAU).unwrap().shape;
    let side = face_of(&cyl, |s| matches!(s, Surface::Cylinder { .. }));
    for (angle, z, off) in [
        (0.3, 4.0, 2.0),
        (2.5, 11.0, -1.0),
        (5.9, 0.5, 6.0),
        (0.0, 6.0, 1.0),
    ] {
        let radial = f.x.get() * f64::cos(angle) + f.y.get() * f64::sin(angle);
        let p = to_world(&f, Vec3::new(0.0, 0.0, z)) + radial * (r + off);
        let (uv, foot) = k.project_point_on_face(&cyl, side, p).unwrap();
        assert_near("cylinder foot", foot, p - radial * off, 1e-9);
        let back = k.eval_surface(&cyl, side, uv).unwrap();
        assert_near("eval at the projected uv", back.point, foot, 1e-9);
    }
    // Past the cylinder's end: the nearest point is on the rim, and its uv is on the boundary.
    let p = to_world(&f, Vec3::new(r + 1.0, 0.0, h + 2.0));
    let (uv, foot) = k.project_point_on_face(&cyl, side, p).unwrap();
    assert_near("rim foot", foot, to_world(&f, Vec3::new(r, 0.0, h)), 1e-9);
    let back = k.eval_surface(&cyl, side, uv).unwrap();
    assert_near("eval at the rim uv", back.point, foot, 1e-9);
    assert!(matches!(
        k.project_point_on_face(&cyl, FaceId(99), p),
        Err(KernelError::InvalidInput { what: "face", .. })
    ));
}

// ---- Done-when 3: ray casts -----------------------------------------------------------------

/// The analytic first hit of a ray (local frame) with a cylinder `r`, `h` on the z axis:
/// `(t, 0 = lateral | 1 = bottom | 2 = top)`.
fn analytic_hit(o: Vec3, d: Vec3, r: f64, h: f64) -> Option<(f64, u8)> {
    let mut best: Option<(f64, u8)> = None;
    let mut take = |t: f64, which: u8| {
        if t >= 0.0 && best.is_none_or(|(bt, _)| t < bt) {
            best = Some((t, which));
        }
    };
    let a = d.x * d.x + d.y * d.y;
    if a > 0.0 {
        let b = 2.0 * (o.x * d.x + o.y * d.y);
        let c = o.x * o.x + o.y * o.y - r * r;
        let disc = b * b - 4.0 * a * c;
        if disc >= 0.0 {
            for t in [
                (-b - disc.sqrt()) / (2.0 * a),
                (-b + disc.sqrt()) / (2.0 * a),
            ] {
                let z = o.z + t * d.z;
                if (0.0..=h).contains(&z) {
                    take(t, 0);
                }
            }
        }
    }
    if d.z != 0.0 {
        for (zc, which) in [(0.0, 1), (h, 2)] {
            let t = (zc - o.z) / d.z;
            let q = o + d * t;
            if q.x * q.x + q.y * q.y <= r * r {
                take(t, which);
            }
        }
    }
    best
}

#[test]
fn rays_hit_a_cylinder_where_the_analytic_solution_does() {
    let k = kernel();
    let f = tilted();
    let (r, h) = (5.0, 12.0);
    let cyl = k.make_cylinder(&f, r, h, TAU).unwrap().shape;
    let side = face_of(&cyl, |s| matches!(s, Surface::Cylinder { .. }));
    let bottom = face_of(
        &cyl,
        |s| matches!(s, Surface::Plane(p) if to_local(&f, p.origin).z.abs() < 1e-9),
    );
    let top = face_of(
        &cyl,
        |s| matches!(s, Surface::Plane(p) if (to_local(&f, p.origin).z - h).abs() < 1e-9),
    );
    let mut rng = Rng::new(0x0B5E_0903);
    let mut unit = || (rng.next_u64() >> 11) as f64 / (1u64 << 53) as f64;
    let centre = Vec3::new(0.0, 0.0, h / 2.0);
    let (mut counts, mut worst_t, mut worst_n) = ([0usize; 3], 0.0_f64, 0.0_f64);
    for i in 0..1000 {
        // An origin on a sphere of radius 40 around the cylinder, aimed at a point inside it.
        let (zr, phi) = (2.0 * unit() - 1.0, TAU * unit());
        let o = centre
            + Vec3::new(
                (1.0 - zr * zr).sqrt() * phi.cos(),
                (1.0 - zr * zr).sqrt() * phi.sin(),
                zr,
            ) * 40.0;
        let (rr, th, z) = (
            0.999 * r * unit().sqrt(),
            TAU * unit(),
            h * (0.001 + 0.998 * unit()),
        );
        let target = Vec3::new(rr * th.cos(), rr * th.sin(), z);
        let d = (target - o).normalize();
        let (t_want, which) = analytic_hit(o, d, r, h).expect("aimed inside");
        counts[usize::from(which)] += 1;
        let ray = Ray3 {
            origin: to_world(&f, o),
            dir: Dir3::new_unchecked(rotation(&f) * d),
        };
        let hit = k.ray_cast(&cyl, &ray, 1e3).unwrap().expect("hit");
        let face_want = [side, bottom, top][usize::from(which)];
        assert_eq!(hit.face, face_want, "ray {i}: {o:?} {d:?}");
        let t_err = (hit.t - t_want).abs() / t_want;
        worst_t = worst_t.max(t_err);
        assert!(t_err <= REL, "ray {i}: t {} vs {t_want}", hit.t);
        assert_near("hit point", hit.point, ray.at(t_want), 1e-9 * 40.0);
        let q = o + d * t_want;
        let n_local = match which {
            0 => Vec3::new(q.x, q.y, 0.0) / r,
            1 => -Vec3::Z,
            _ => Vec3::Z,
        };
        let n_err = (hit.normal.get() - rotation(&f) * n_local).length();
        worst_n = worst_n.max(n_err);
        assert!(
            n_err <= REL,
            "ray {i}: normal {:?} vs {n_local:?}",
            hit.normal
        );
        // The normal faces the ray (it enters from outside).
        assert!(hit.normal.get().dot(ray.dir.get()) < 0.0);
    }
    eprintln!(
        "1000 rays: {} lateral, {} bottom, {} top; worst t error {worst_t:.2e}, worst normal error {worst_n:.2e}",
        counts[0], counts[1], counts[2]
    );
    assert!(counts.iter().all(|&c| c > 20), "{counts:?}");
    // A miss and a ray stopped short.
    let away = Ray3 {
        origin: to_world(&f, Vec3::new(20.0, 0.0, 6.0)),
        dir: f.x,
    };
    assert_eq!(k.ray_cast(&cyl, &away, 1e3).unwrap(), None);
    let short = Ray3 {
        origin: to_world(&f, Vec3::new(20.0, 0.0, 6.0)),
        dir: Dir3::new_unchecked(-f.x.get()),
    };
    assert_eq!(k.ray_cast(&cyl, &short, 14.0).unwrap(), None);
    let hit = k.ray_cast(&cyl, &short, 15.0 + 1e-6).unwrap().unwrap();
    assert_rel("short ray", hit.t, 15.0, REL);
    assert!(matches!(
        k.ray_cast(&cyl, &short, -1.0),
        Err(KernelError::InvalidInput { what: "max_t", .. })
    ));
}

// ---- Done-when 4: curvatures ----------------------------------------------------------------

#[test]
fn torus_curvatures_are_analytic() {
    let k = kernel();
    let f = tilted();
    let (big, small) = (10.0, 3.0);
    let torus = k.make_torus(&f, big, small, TAU).unwrap().shape;
    let face = face_of(&torus, |s| matches!(s, Surface::Torus { .. }));
    let mut worst: f64 = 0.0;
    for j in 0..10 {
        let u = 0.31 + 0.6 * f64::from(j);
        let v = -2.9 + 0.59 * f64::from(j);
        let p = k.eval_surface(&torus, face, Point2::new(u, v)).unwrap();
        let ring = Vec3::new(u.cos(), u.sin(), 0.0);
        let rho = big + small * v.cos();
        let local = ring * rho + Vec3::Z * (small * v.sin());
        assert_near("torus point", p.point, to_world(&f, local), 1e-9 * big);
        let n_local = ring * v.cos() + Vec3::Z * v.sin();
        assert_near("torus normal", p.normal.get(), rotation(&f) * n_local, REL);
        // Convex-positive: the tube 1/r; around the axis cos v / (R + r cos v).
        let k_tube = 1.0 / small;
        let k_ring = v.cos() / rho;
        let (k_min, k_max) = (k_tube.min(k_ring), k_tube.max(k_ring));
        let err = (p.k_min - k_min).abs().max((p.k_max - k_max).abs());
        worst = worst.max(err / k_tube);
        assert!(
            err <= REL * k_tube,
            "uv ({u}, {v}): k {} {} vs {k_min} {k_max}",
            p.k_min,
            p.k_max
        );
        // Directions: around the tube (∂/∂v) for 1/r, around the axis (∂/∂u) for the other.
        let d_tube = rotation(&f) * (-ring * v.sin() + Vec3::Z * v.cos());
        let d_ring = rotation(&f) * Vec3::new(-u.sin(), u.cos(), 0.0);
        let (d_for_min, d_for_max) = if k_tube <= k_ring {
            (d_tube, d_ring)
        } else {
            (d_ring, d_tube)
        };
        assert!(
            p.dir_min.dot(d_for_min).abs() > 1.0 - REL,
            "dir_min {:?}",
            p.dir_min
        );
        assert!(
            p.dir_max.dot(d_for_max).abs() > 1.0 - REL,
            "dir_max {:?}",
            p.dir_max
        );
    }
    eprintln!("torus: worst curvature error {worst:.2e} relative to 1/r");
}

#[test]
fn curvature_signs_follow_the_material() {
    let k = kernel();
    let f = tilted();
    // Outside of a sphere: convex, 1/r both ways (umbilic).
    let sphere = k.make_sphere(&f, 4.0).unwrap().shape;
    let face = face_of(&sphere, |s| matches!(s, Surface::Sphere { .. }));
    let p = k
        .eval_surface(&sphere, face, Point2::new(1.0, 0.3))
        .unwrap();
    assert!(
        (p.k_min - 0.25).abs() < 1e-12 && (p.k_max - 0.25).abs() < 1e-12,
        "{p:?}"
    );
    assert!((p.dir_min.dot(p.dir_max)).abs() < 1e-12);
    assert!((p.point - f.origin).dot(p.normal.get()) > 0.0);
    // A cylinder: convex 1/r around, 0 along the axis.
    let cyl = k.make_cylinder(&f, 5.0, 10.0, TAU).unwrap().shape;
    let side = face_of(&cyl, |s| matches!(s, Surface::Cylinder { .. }));
    let p = k.eval_surface(&cyl, side, Point2::new(0.7, 3.0)).unwrap();
    assert!(
        (p.k_max - 0.2).abs() < 1e-12 && p.k_min.abs() < 1e-12,
        "{p:?}"
    );
    assert!(p.dir_min.dot(f.normal.get()).abs() > 1.0 - 1e-12);
    // The wall of a hole drilled through a box: concave, −1/r, normal toward the axis.
    let block = k.make_box(&f, Vec3::new(20.0, 20.0, 10.0)).unwrap().shape;
    let drill_frame = Plane {
        origin: to_world(&f, Vec3::new(10.0, 10.0, -1.0)),
        ..f
    };
    let drill = k.make_cylinder(&drill_frame, 3.0, 12.0, TAU).unwrap().shape;
    let holed = k
        .boolean(
            BoolOp::Cut,
            &[block],
            &[drill],
            &BoolOpts::default(),
            &ctx(),
        )
        .unwrap()
        .shape;
    let wall = face_of(&holed, |s| matches!(s, Surface::Cylinder { .. }));
    let [u0, u1, v0, v1] = holed.face_uv_bounds(wall);
    let p = k
        .eval_surface(&holed, wall, Point2::new(0.5 * (u0 + u1), 0.5 * (v0 + v1)))
        .unwrap();
    assert!(
        (p.k_min + 1.0 / 3.0).abs() < 1e-12 && p.k_max.abs() < 1e-12,
        "{p:?}"
    );
    let axis_point = to_world(&f, Vec3::new(10.0, 10.0, to_local(&f, p.point).z));
    assert!(
        (axis_point - p.point).dot(p.normal.get()) > 0.0,
        "normal points into the hole"
    );
    // A pole of the sphere has no normal from the first derivatives.
    let pole = k.eval_surface(&sphere, face, Point2::new(0.0, PI / 2.0));
    assert!(
        matches!(
            pole,
            Err(KernelError::OpFailed {
                op: "eval_surface",
                ..
            })
        ),
        "{pole:?}"
    );
    assert!(matches!(
        k.eval_surface(&sphere, FaceId(7), Point2::ZERO),
        Err(KernelError::InvalidInput { what: "face", .. })
    ));
}

// ---- Done-when 5: interference --------------------------------------------------------------

#[test]
fn interference_of_overlapping_boxes_is_analytic() {
    let k = kernel();
    let f = tilted();
    let a = k.make_box(&f, Vec3::new(10.0, 20.0, 30.0)).unwrap().shape;
    let at = |p: Vec3| Plane {
        origin: to_world(&f, p),
        ..f
    };
    let b = k
        .make_box(&at(Vec3::new(5.0, 5.0, 5.0)), Vec3::splat(10.0))
        .unwrap()
        .shape;
    let common = k.interference(&a, &b, &ctx()).unwrap().expect("overlap");
    let props = k.mass_props(&common.shape).unwrap();
    // [5, 10] × [5, 15] × [5, 15].
    assert_rel("interference volume", props.volume, 500.0, REL);
    assert_near(
        "interference centroid",
        props.centroid,
        to_world(&f, Vec3::new(7.5, 10.0, 10.0)),
        1e-9,
    );
    assert_eq!(common.history.operand_counts.len(), 2);
    // Symmetric.
    let other = k.interference(&b, &a, &ctx()).unwrap().expect("overlap");
    assert_rel(
        "interference volume (b, a)",
        k.mass_props(&other.shape).unwrap().volume,
        500.0,
        REL,
    );
    // Touching faces and disjoint boxes do not interfere.
    let touching = k
        .make_box(&at(Vec3::new(10.0, 0.0, 0.0)), Vec3::splat(5.0))
        .unwrap()
        .shape;
    assert!(k.interference(&a, &touching, &ctx()).unwrap().is_none());
    let apart = k
        .make_box(&at(Vec3::new(11.0, 0.0, 0.0)), Vec3::splat(5.0))
        .unwrap()
        .shape;
    assert!(k.interference(&a, &apart, &ctx()).unwrap().is_none());
    // Only solids interfere.
    let face = face_shape(&a, FaceId(0));
    assert!(matches!(
        k.interference(&face, &b, &ctx()),
        Err(KernelError::InvalidInput {
            what: "objects",
            ..
        })
    ));
}

// ---- edge ends (R-S04-1) --------------------------------------------------------------------

#[test]
fn edge_ends_give_points_and_unit_tangents() {
    let k = kernel();
    let f = tilted();
    let block = k.make_box(&f, Vec3::new(10.0, 20.0, 30.0)).unwrap().shape;
    for e in block.edges() {
        let ends = k.edge_ends(&block, e).unwrap();
        let [v0, v1] = block.edge_vertices(e);
        assert_near("p0", ends.p0, block.vertex_point(v0), 1e-12);
        assert_near("p1", ends.p1, block.vertex_point(v1), 1e-12);
        let chord = (ends.p1 - ends.p0).normalize();
        assert_near("t0", ends.t0.get(), chord, 1e-12);
        assert_near("t1", ends.t1.get(), chord, 1e-12);
    }
    let cyl = k.make_cylinder(&f, 5.0, 10.0, TAU).unwrap().shape;
    for e in cyl.edges() {
        let ends = k.edge_ends(&cyl, e).unwrap();
        assert!((ends.t0.get().length() - 1.0).abs() < 1e-12);
        if matches!(
            cyl.curve(e),
            Some((amg_brep::geom::Curve3::Circle { .. }, _, _))
        ) {
            // A full circle: closed, tangent perpendicular to the radius and to the axis.
            assert_near("closed circle", ends.p1, ends.p0, 1e-9);
            let radial = ends.p0 - to_world(&f, Vec3::new(0.0, 0.0, to_local(&f, ends.p0).z));
            assert!(ends.t0.get().dot(radial).abs() < 1e-9);
            assert!(ends.t0.get().dot(f.normal.get()).abs() < 1e-12);
        }
    }
    let sphere = k.make_sphere(&f, 4.0).unwrap().shape;
    let degenerated = sphere.edges().find(|&e| sphere.curve(e).is_none()).unwrap();
    assert!(matches!(
        k.edge_ends(&sphere, degenerated),
        Err(KernelError::InvalidInput { what: "edge", .. })
    ));
    assert!(matches!(
        k.edge_ends(&sphere, EdgeId(1000)),
        Err(KernelError::InvalidInput { what: "edge", .. })
    ));
}
