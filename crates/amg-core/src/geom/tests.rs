use super::intersect::*;
use super::*;
use crate::math::Quat;
use approx::assert_relative_eq;
use proptest::prelude::*;

const T: Tolerance = Tolerance::DEFAULT;

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}
fn p2(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}
fn dir(x: f64, y: f64, z: f64) -> Dir3 {
    Dir3::new(Vec3::new(x, y, z), &T).unwrap()
}
fn ray(o: Point3, d: Dir3) -> Ray3 {
    Ray3 { origin: o, dir: d }
}
fn unit_box() -> Aabb3 {
    Aabb3::new(p3(0.0, 0.0, 0.0), p3(1.0, 1.0, 1.0))
}

// ---- Aabb3 ------------------------------------------------------------------------------

#[test]
fn aabb3_new_orders_corners() {
    let b = Aabb3::new(p3(1.0, -1.0, 2.0), p3(-1.0, 1.0, 0.0));
    assert_eq!(b.min, p3(-1.0, -1.0, 0.0));
    assert_eq!(b.max, p3(1.0, 1.0, 2.0));
}

#[test]
fn aabb3_empty_and_from_points() {
    assert!(Aabb3::EMPTY.is_empty());
    assert!(Aabb3::from_points([]).is_empty());
    let b = Aabb3::from_points([p3(1.0, 2.0, 3.0), p3(-1.0, 5.0, 0.0)]);
    assert!(!b.is_empty());
    assert_eq!(b, Aabb3::new(p3(-1.0, 2.0, 0.0), p3(1.0, 5.0, 3.0)));
    // A single point is a non-empty degenerate box.
    assert!(!Aabb3::from_points([Point3::ONE]).is_empty());
}

#[test]
fn aabb3_center_size_diagonal() {
    let b = Aabb3::new(p3(0.0, 0.0, 0.0), p3(2.0, 4.0, 4.0));
    assert_eq!(b.center(), p3(1.0, 2.0, 2.0));
    assert_eq!(b.size(), Vec3::new(2.0, 4.0, 4.0));
    assert_relative_eq!(b.diagonal(), 6.0);
    assert_eq!(Aabb3::EMPTY.size(), Vec3::ZERO);
    assert_relative_eq!(Aabb3::EMPTY.diagonal(), 0.0);
    assert!(Aabb3::EMPTY.center().is_nan());
}

#[test]
fn aabb3_union_and_include_point() {
    let a = unit_box();
    let b = Aabb3::new(p3(2.0, 2.0, 2.0), p3(3.0, 3.0, 3.0));
    assert_eq!(a.union(&b), Aabb3::new(Point3::ZERO, p3(3.0, 3.0, 3.0)));
    assert_eq!(Aabb3::EMPTY.union(&a), a);
    let mut c = Aabb3::EMPTY;
    c.include_point(p3(1.0, 1.0, 1.0));
    c.include_point(p3(-1.0, 0.0, 2.0));
    assert_eq!(c, Aabb3::new(p3(-1.0, 0.0, 1.0), p3(1.0, 1.0, 2.0)));
}

#[test]
fn aabb3_expanded() {
    let b = unit_box().expanded(0.5);
    assert_eq!(b, Aabb3::new(Point3::splat(-0.5), Point3::splat(1.5)));
    assert!(Aabb3::EMPTY.expanded(10.0).is_empty());
}

#[test]
fn aabb3_contains_with_tolerance() {
    let b = unit_box();
    assert!(b.contains(p3(0.5, 0.5, 0.5), &T));
    assert!(b.contains(p3(1.0 + 5e-7, 0.5, 0.5), &T));
    assert!(!b.contains(p3(1.0 + 2e-6, 0.5, 0.5), &T));
    assert!(!Aabb3::EMPTY.contains(Point3::ZERO, &T));
}

#[test]
fn aabb3_intersects() {
    let a = unit_box();
    assert!(a.intersects(&Aabb3::new(Point3::splat(0.5), Point3::splat(2.0)), &T));
    assert!(a.intersects(
        &Aabb3::new(p3(1.0 + 5e-7, 0.0, 0.0), Point3::splat(2.0)),
        &T
    ));
    assert!(!a.intersects(&Aabb3::new(p3(1.1, 0.0, 0.0), Point3::splat(2.0)), &T));
    assert!(!a.intersects(&Aabb3::EMPTY, &T));
}

#[test]
fn aabb3_transformed_rotation() {
    let t = Transform::from_rotation_z(std::f64::consts::FRAC_PI_4);
    let b = unit_box().transformed(&t);
    let h = std::f64::consts::FRAC_1_SQRT_2;
    assert_relative_eq!(b.min, p3(-h, 0.0, 0.0), epsilon = 1e-12);
    assert_relative_eq!(b.max, p3(h, 2.0 * h, 1.0), epsilon = 1e-12);
    assert!(Aabb3::EMPTY.transformed(&t).is_empty());
}

// ---- Aabb2 ------------------------------------------------------------------------------

#[test]
fn aabb2_basics() {
    let b = Aabb2::new(p2(2.0, 0.0), p2(0.0, 4.0));
    assert_eq!(b.min, p2(0.0, 0.0));
    assert_eq!(b.center(), p2(1.0, 2.0));
    assert_eq!(b.size(), Vec2::new(2.0, 4.0));
    assert_relative_eq!(b.diagonal(), 20.0_f64.sqrt());
    assert!(Aabb2::EMPTY.is_empty());
    assert!(!b.is_empty());
    assert_eq!(Aabb2::EMPTY.size(), Vec2::ZERO);
    assert!(Aabb2::EMPTY.center().is_nan());
}

#[test]
fn aabb2_from_points_union_include_expand() {
    let b = Aabb2::from_points([p2(1.0, 1.0), p2(-1.0, 3.0)]);
    assert_eq!(b, Aabb2::new(p2(-1.0, 1.0), p2(1.0, 3.0)));
    assert!(Aabb2::from_points([]).is_empty());
    let u = b.union(&Aabb2::new(p2(5.0, 5.0), p2(6.0, 6.0)));
    assert_eq!(u, Aabb2::new(p2(-1.0, 1.0), p2(6.0, 6.0)));
    let mut c = Aabb2::EMPTY;
    c.include_point(p2(3.0, -2.0));
    assert_eq!(c, Aabb2::new(p2(3.0, -2.0), p2(3.0, -2.0)));
    assert_eq!(c.expanded(1.0), Aabb2::new(p2(2.0, -3.0), p2(4.0, -1.0)));
}

#[test]
fn aabb2_contains_intersects() {
    let b = Aabb2::new(p2(0.0, 0.0), p2(1.0, 1.0));
    assert!(b.contains(p2(1.0 + 5e-7, 0.0), &T));
    assert!(!b.contains(p2(1.1, 0.0), &T));
    assert!(b.intersects(&Aabb2::new(p2(1.0, 1.0), p2(2.0, 2.0)), &T));
    assert!(!b.intersects(&Aabb2::new(p2(1.1, 1.1), p2(2.0, 2.0)), &T));
    assert!(!b.intersects(&Aabb2::EMPTY, &T));
}

#[test]
fn aabb2_transformed2() {
    let t = Transform2::from_angle_translation(std::f64::consts::FRAC_PI_2, Vec2::new(10.0, 0.0));
    let b = Aabb2::new(p2(0.0, 0.0), p2(2.0, 1.0)).transformed2(&t);
    assert_relative_eq!(b.min, p2(9.0, 0.0), epsilon = 1e-12);
    assert_relative_eq!(b.max, p2(10.0, 2.0), epsilon = 1e-12);
    assert!(Aabb2::EMPTY.transformed2(&t).is_empty());
}

// ---- Plane ------------------------------------------------------------------------------

fn assert_frame(p: &Plane) {
    let (x, y, n) = (p.x.get(), p.y.get(), p.normal.get());
    assert!(x.dot(y).abs() < 1e-12 && x.dot(n).abs() < 1e-12 && y.dot(n).abs() < 1e-12);
    assert_relative_eq!(x.cross(y), n, epsilon = 1e-12);
}

#[test]
fn plane_constants_are_right_handed() {
    for p in [Plane::XY, Plane::YZ, Plane::XZ] {
        assert_frame(&p);
        assert_eq!(p.origin, Point3::ZERO);
    }
    assert_eq!(Plane::XZ.normal, Dir3::NEG_Y);
}

#[test]
fn plane_from_normal_is_deterministic() {
    let n = dir(1.0, 2.0, 3.0);
    let a = Plane::from_normal(p3(1.0, 0.0, 0.0), n);
    assert_frame(&a);
    assert_eq!(a, Plane::from_normal(p3(1.0, 0.0, 0.0), n));
    assert_eq!(a.normal, n);
}

#[test]
fn plane_from_normal_x() {
    let p = Plane::from_normal_x(Point3::ZERO, Dir3::Z, Vec3::new(2.0, 0.0, 5.0), &T).unwrap();
    assert_frame(&p);
    assert_relative_eq!(p.x.get(), Vec3::X, epsilon = 1e-15);
    assert_relative_eq!(p.y.get(), Vec3::Y, epsilon = 1e-15);
    assert!(Plane::from_normal_x(Point3::ZERO, Dir3::Z, Vec3::new(0.0, 0.0, 3.0), &T).is_err());
    assert!(Plane::from_normal_x(Point3::ZERO, Dir3::Z, Vec3::ZERO, &T).is_err());
}

#[test]
fn plane_from_points() {
    let p =
        Plane::from_points(p3(0.0, 0.0, 1.0), p3(2.0, 0.0, 1.0), p3(0.0, 3.0, 1.0), &T).unwrap();
    assert_frame(&p);
    assert_eq!(p.normal, Dir3::Z);
    assert_eq!(p.x, Dir3::X);
    assert!(matches!(
        Plane::from_points(Point3::ZERO, Point3::ZERO, Point3::X, &T),
        Err(Error::Degenerate { .. })
    ));
    assert!(matches!(
        Plane::from_points(Point3::ZERO, Point3::X, p3(2.0, 1e-8, 0.0), &T),
        Err(Error::Degenerate { .. })
    ));
}

#[test]
fn plane_signed_distance_and_project() {
    let p = Plane::from_normal(p3(0.0, 0.0, 2.0), Dir3::Z);
    assert_relative_eq!(p.signed_distance(p3(5.0, 1.0, 5.0)), 3.0);
    assert_relative_eq!(p.signed_distance(p3(5.0, 1.0, -1.0)), -3.0);
    assert_relative_eq!(p.project(p3(5.0, 1.0, 7.0)), p3(5.0, 1.0, 2.0));
}

#[test]
fn plane_local_world_and_transform() {
    let p = Plane::XZ;
    assert_eq!(p.to_local(p3(3.0, 9.0, 4.0)), p2(3.0, 4.0));
    assert_eq!(p.to_world(p2(3.0, 4.0)), p3(3.0, 0.0, 4.0));
    let q = Plane::from_normal(p3(1.0, 2.0, 3.0), dir(1.0, 1.0, 0.0));
    let t = q.to_transform();
    let local = Vec3::new(0.5, -2.0, 1.5);
    let w = t.transform_point3(local);
    assert_relative_eq!(
        w,
        q.to_world(p2(0.5, -2.0)) + q.normal.get() * 1.5,
        epsilon = 1e-12
    );
}

#[test]
fn plane_flipped() {
    let p = Plane::XY.flipped();
    assert_frame(&p);
    assert_eq!(p.normal, Dir3::NEG_Z);
    assert_eq!(p.x, Dir3::X);
}

#[test]
fn plane_serde_round_trip() {
    let p = Plane::from_normal(p3(1.0, 2.0, 3.0), dir(1.0, 2.0, 3.0));
    let s = serde_json::to_string(&p).unwrap();
    assert_eq!(serde_json::from_str::<Plane>(&s).unwrap(), p);
}

// ---- Ray, segment, triangle -------------------------------------------------------------

#[test]
fn ray3_at() {
    let r = ray(p3(1.0, 0.0, 0.0), Dir3::Y);
    assert_eq!(r.at(2.5), p3(1.0, 2.5, 0.0));
}

#[test]
fn segment3_length_and_closest_point() {
    let s = Segment3 {
        a: Point3::ZERO,
        b: p3(4.0, 0.0, 0.0),
    };
    assert_relative_eq!(s.length(), 4.0);
    assert_eq!(
        s.closest_point(p3(1.0, 5.0, 0.0)),
        (0.25, p3(1.0, 0.0, 0.0))
    );
    assert_eq!(s.closest_point(p3(-3.0, 1.0, 0.0)), (0.0, Point3::ZERO));
    assert_eq!(s.closest_point(p3(9.0, 1.0, 0.0)), (1.0, s.b));
    let z = Segment3 {
        a: Point3::ONE,
        b: Point3::ONE,
    };
    assert_eq!(z.closest_point(Point3::ZERO), (0.0, Point3::ONE));
}

#[test]
fn triangle3_normal_and_area() {
    let t = Triangle3 {
        a: Point3::ZERO,
        b: p3(2.0, 0.0, 0.0),
        c: p3(0.0, 2.0, 0.0),
    };
    assert_eq!(t.normal(&T), Some(Dir3::Z));
    assert_relative_eq!(t.area(), 2.0);
    let flat = Triangle3 {
        a: Point3::ZERO,
        b: p3(2.0, 0.0, 0.0),
        c: p3(1.0, 1e-9, 0.0),
    };
    assert_eq!(flat.normal(&T), None);
}

#[test]
fn line_and_segment2_types_serde() {
    let l = Line2 {
        point: p2(1.0, 2.0),
        dir: Vec2::X,
    };
    let s = Segment2 {
        a: p2(0.0, 0.0),
        b: p2(1.0, 1.0),
    };
    let l3 = Line3 {
        point: Point3::ONE,
        dir: Dir3::Z,
    };
    assert_eq!(
        serde_json::from_str::<Line2>(&serde_json::to_string(&l).unwrap()).unwrap(),
        l
    );
    assert_eq!(
        serde_json::from_str::<Segment2>(&serde_json::to_string(&s).unwrap()).unwrap(),
        s
    );
    assert_eq!(
        serde_json::from_str::<Line3>(&serde_json::to_string(&l3).unwrap()).unwrap(),
        l3
    );
}

// ---- intersect: unit tests --------------------------------------------------------------

#[test]
fn ray_plane_hits_misses_and_parallel() {
    let p = Plane::from_normal(p3(0.0, 0.0, 5.0), Dir3::Z);
    let h = ray_plane(&ray(Point3::ZERO, dir(0.0, 1.0, 1.0)), &p, &T).unwrap();
    assert_relative_eq!(h.t, 50.0_f64.sqrt(), epsilon = 1e-12);
    assert_relative_eq!(h.point, p3(0.0, 5.0, 5.0), epsilon = 1e-12);
    // Plane behind the ray.
    assert!(ray_plane(&ray(Point3::ZERO, Dir3::NEG_Z), &p, &T).is_none());
    // Parallel.
    assert!(ray_plane(&ray(Point3::ZERO, Dir3::X), &p, &T).is_none());
    // Back side hits too.
    assert!(ray_plane(&ray(p3(0.0, 0.0, 9.0), Dir3::NEG_Z), &p, &T).is_some());
}

#[test]
fn ray_aabb_cases() {
    let b = unit_box();
    let (t0, t1) = ray_aabb(&ray(p3(-1.0, 0.5, 0.5), Dir3::X), &b).unwrap();
    assert_relative_eq!(t0, 1.0);
    assert_relative_eq!(t1, 2.0);
    // Origin inside: entry clipped to 0.
    let (t0, t1) = ray_aabb(&ray(p3(0.5, 0.5, 0.5), Dir3::NEG_X), &b).unwrap();
    assert_relative_eq!(t0, 0.0);
    assert_relative_eq!(t1, 0.5);
    // Behind.
    assert!(ray_aabb(&ray(p3(2.0, 0.5, 0.5), Dir3::X), &b).is_none());
    // Zero components with origin on the slab boundary: no NaN, still a hit.
    let (t0, t1) = ray_aabb(&ray(p3(-1.0, 0.0, 1.0), Dir3::X), &b).unwrap();
    assert!(t0.is_finite() && t1.is_finite());
    // Zero component outside the slab.
    assert!(ray_aabb(&ray(p3(-1.0, 2.0, 0.5), Dir3::X), &b).is_none());
    assert!(ray_aabb(&ray(p3(-1.0, 0.5, 0.5), Dir3::X), &Aabb3::EMPTY).is_none());
}

#[test]
fn ray_triangle_hit_bary_and_both_sides() {
    let tri = Triangle3 {
        a: p3(0.0, 0.0, 0.0),
        b: p3(1.0, 0.0, 0.0),
        c: p3(0.0, 1.0, 0.0),
    };
    let q = p3(0.2, 0.3, 0.0);
    let h = ray_triangle(&ray(p3(0.2, 0.3, 5.0), Dir3::NEG_Z), &tri).unwrap();
    assert_relative_eq!(h.t, 5.0, epsilon = 1e-12);
    let pt = tri.a * h.bary[0] + tri.b * h.bary[1] + tri.c * h.bary[2];
    assert_relative_eq!(pt, q, epsilon = 1e-12);
    assert_relative_eq!(h.bary[0] + h.bary[1] + h.bary[2], 1.0, epsilon = 1e-12);
    // From below (back face).
    let h2 = ray_triangle(&ray(p3(0.2, 0.3, -2.0), Dir3::Z), &tri).unwrap();
    assert_relative_eq!(h2.t, 2.0, epsilon = 1e-12);
    // Miss, behind and parallel.
    assert!(ray_triangle(&ray(p3(0.8, 0.8, 5.0), Dir3::NEG_Z), &tri).is_none());
    assert!(ray_triangle(&ray(p3(0.2, 0.3, 5.0), Dir3::Z), &tri).is_none());
    assert!(ray_triangle(&ray(p3(-1.0, 0.1, 0.0), Dir3::X), &tri).is_none());
    // Degenerate triangle.
    let deg = Triangle3 {
        a: Point3::ZERO,
        b: Point3::X,
        c: p3(2.0, 0.0, 0.0),
    };
    assert!(ray_triangle(&ray(p3(0.5, 0.0, 1.0), Dir3::NEG_Z), &deg).is_none());
}

#[test]
fn plane_plane_line() {
    let a = Plane::from_normal(p3(0.0, 0.0, 1.0), Dir3::Z);
    let b = Plane::from_normal(p3(2.0, 0.0, 0.0), Dir3::X);
    let l = plane_plane(&a, &b, &T).unwrap();
    assert!(T.is_parallel(l.dir, Dir3::Y));
    assert_relative_eq!(l.point, p3(2.0, 0.0, 1.0), epsilon = 1e-12);
    assert!(plane_plane(&a, &a.flipped(), &T).is_none());
}

#[test]
fn line_plane_point() {
    let p = Plane::from_normal(p3(0.0, 0.0, 3.0), Dir3::Z);
    let l = Line3 {
        point: p3(1.0, 1.0, 10.0),
        dir: Dir3::Z, // pointing away: lines extend both ways
    };
    assert_relative_eq!(line_plane(&l, &p, &T).unwrap(), p3(1.0, 1.0, 3.0));
    let par = Line3 {
        point: Point3::ZERO,
        dir: Dir3::X,
    };
    assert!(line_plane(&par, &p, &T).is_none());
}

#[test]
fn segment_segment_cases() {
    let a = Segment3 {
        a: p3(-1.0, 0.0, 0.0),
        b: p3(1.0, 0.0, 0.0),
    };
    let b = Segment3 {
        a: p3(0.0, -1.0, 1.0),
        b: p3(0.0, 1.0, 1.0),
    };
    let (s, t, pa, pb) = segment_segment(&a, &b);
    assert_relative_eq!(s, 0.5);
    assert_relative_eq!(t, 0.5);
    assert_relative_eq!(pa, Point3::ZERO);
    assert_relative_eq!(pb, p3(0.0, 0.0, 1.0));
    // Endpoint clamping.
    let c = Segment3 {
        a: p3(3.0, 1.0, 0.0),
        b: p3(5.0, 1.0, 0.0),
    };
    let (s, t, _, _) = segment_segment(&a, &c);
    assert_relative_eq!(s, 1.0);
    assert_relative_eq!(t, 0.0);
    // Parallel and degenerate.
    let par = Segment3 {
        a: p3(0.0, 1.0, 0.0),
        b: p3(2.0, 1.0, 0.0),
    };
    let (_, _, pa, pb) = segment_segment(&a, &par);
    assert_relative_eq!(pa.distance(pb), 1.0, epsilon = 1e-12);
    let pt = Segment3 {
        a: p3(0.0, 2.0, 0.0),
        b: p3(0.0, 2.0, 0.0),
    };
    let (s, t, pa, _) = segment_segment(&a, &pt);
    assert_relative_eq!(s, 0.5);
    assert_relative_eq!(t, 0.0);
    assert_relative_eq!(pa, Point3::ZERO);
    let (_, _, pa, pb) = segment_segment(&pt, &pt);
    assert_eq!(pa, pb);
}

#[test]
fn line_line_closest() {
    let a = Line3 {
        point: Point3::ZERO,
        dir: Dir3::X,
    };
    let b = Line3 {
        point: p3(5.0, 0.0, 2.0),
        dir: Dir3::Y,
    };
    let (pa, pb) = line_line(&a, &b, &T).unwrap();
    assert_relative_eq!(pa, p3(5.0, 0.0, 0.0), epsilon = 1e-12);
    assert_relative_eq!(pb, p3(5.0, 0.0, 2.0), epsilon = 1e-12);
    let c = Line3 {
        point: Point3::ONE,
        dir: Dir3::NEG_X,
    };
    assert!(line_line(&a, &c, &T).is_none());
}

#[test]
fn point_segment_distance_cases() {
    let s = Segment3 {
        a: Point3::ZERO,
        b: p3(2.0, 0.0, 0.0),
    };
    assert_relative_eq!(point_segment_distance(p3(1.0, 3.0, 4.0), &s), 5.0);
    assert_relative_eq!(point_segment_distance(p3(5.0, 4.0, 0.0), &s), 5.0);
}

#[test]
fn point_triangle_closest_regions() {
    let tri = Triangle3 {
        a: p3(0.0, 0.0, 0.0),
        b: p3(2.0, 0.0, 0.0),
        c: p3(0.0, 2.0, 0.0),
    };
    // Face, three vertices, three edges.
    assert_relative_eq!(
        point_triangle_closest(p3(0.5, 0.5, 3.0), &tri),
        p3(0.5, 0.5, 0.0)
    );
    assert_eq!(point_triangle_closest(p3(-1.0, -1.0, 0.0), &tri), tri.a);
    assert_eq!(point_triangle_closest(p3(3.0, -1.0, 0.0), &tri), tri.b);
    assert_eq!(point_triangle_closest(p3(-1.0, 3.0, 0.0), &tri), tri.c);
    assert_relative_eq!(
        point_triangle_closest(p3(1.0, -1.0, 0.0), &tri),
        p3(1.0, 0.0, 0.0)
    );
    assert_relative_eq!(
        point_triangle_closest(p3(-1.0, 1.0, 0.0), &tri),
        p3(0.0, 1.0, 0.0)
    );
    assert_relative_eq!(
        point_triangle_closest(p3(2.0, 2.0, 0.0), &tri),
        p3(1.0, 1.0, 0.0),
        epsilon = 1e-12
    );
    // Degenerate triangle does not produce NaN.
    let deg = Triangle3 {
        a: Point3::ZERO,
        b: p3(1.0, 0.0, 0.0),
        c: p3(2.0, 0.0, 0.0),
    };
    let q = point_triangle_closest(p3(1.5, 1.0, 0.0), &deg);
    assert!(!q.is_nan());
    assert_relative_eq!(q, p3(1.5, 0.0, 0.0), epsilon = 1e-12);
}

#[test]
fn segment2_segment2_cases() {
    let s = |ax, ay, bx, by| Segment2 {
        a: p2(ax, ay),
        b: p2(bx, by),
    };
    // Crossing.
    match segment2_segment2(&s(0.0, 0.0, 2.0, 2.0), &s(0.0, 2.0, 2.0, 0.0), &T) {
        Seg2Intersection::Point { t, u, p } => {
            assert_relative_eq!(t, 0.5);
            assert_relative_eq!(u, 0.5);
            assert_relative_eq!(p, p2(1.0, 1.0));
        }
        other => panic!("{other:?}"),
    }
    // Disjoint, non-parallel.
    assert_eq!(
        segment2_segment2(&s(0.0, 0.0, 1.0, 0.0), &s(2.0, -1.0, 2.0, 1.0), &T),
        Seg2Intersection::None
    );
    // Touching at an endpoint within tolerance.
    assert!(matches!(
        segment2_segment2(&s(0.0, 0.0, 1.0, 0.0), &s(1.0 + 5e-7, -1.0, 1.0 + 5e-7, 1.0), &T),
        Seg2Intersection::Point { t, .. } if (t - 1.0).abs() < 1e-12
    ));
    // Parallel, not collinear.
    assert_eq!(
        segment2_segment2(&s(0.0, 0.0, 1.0, 0.0), &s(0.0, 1.0, 1.0, 1.0), &T),
        Seg2Intersection::None
    );
    // Collinear overlap (reversed second segment).
    assert_eq!(
        segment2_segment2(&s(0.0, 0.0, 4.0, 0.0), &s(3.0, 0.0, 1.0, 0.0), &T),
        Seg2Intersection::Overlap { t0: 0.25, t1: 0.75 }
    );
    // Collinear touching end to end.
    assert!(matches!(
        segment2_segment2(&s(0.0, 0.0, 1.0, 0.0), &s(1.0, 0.0, 2.0, 0.0), &T),
        Seg2Intersection::Point { t, u, .. } if (t - 1.0).abs() < 1e-9 && u.abs() < 1e-9
    ));
    // Collinear, disjoint.
    assert_eq!(
        segment2_segment2(&s(0.0, 0.0, 1.0, 0.0), &s(2.0, 0.0, 3.0, 0.0), &T),
        Seg2Intersection::None
    );
    // Degenerate segments.
    assert!(matches!(
        segment2_segment2(&s(1.0, 0.0, 1.0, 0.0), &s(0.0, 0.0, 2.0, 0.0), &T),
        Seg2Intersection::Point { u, .. } if (u - 0.5).abs() < 1e-12
    ));
    assert!(matches!(
        segment2_segment2(&s(0.0, 0.0, 2.0, 0.0), &s(1.0, 0.0, 1.0, 0.0), &T),
        Seg2Intersection::Point { t, .. } if (t - 0.5).abs() < 1e-12
    ));
    assert!(matches!(
        segment2_segment2(&s(1.0, 0.0, 1.0, 0.0), &s(1.0, 0.0, 1.0, 0.0), &T),
        Seg2Intersection::Point { .. }
    ));
    assert_eq!(
        segment2_segment2(&s(1.0, 0.0, 1.0, 0.0), &s(1.0, 1.0, 1.0, 1.0), &T),
        Seg2Intersection::None
    );
    assert_eq!(
        segment2_segment2(&s(1.0, 1.0, 1.0, 1.0), &s(0.0, 0.0, 2.0, 0.0), &T),
        Seg2Intersection::None
    );
}

// ---- property tests ---------------------------------------------------------------------

fn arb_point(r: f64) -> impl Strategy<Value = Point3> {
    (-r..r, -r..r, -r..r).prop_map(|(x, y, z)| p3(x, y, z))
}

fn arb_dir() -> impl Strategy<Value = Dir3> {
    // Include axis-aligned directions (zero components) to exercise the NaN-safe slab path.
    prop_oneof![
        3 => (-1.0..1.0f64, -1.0..1.0f64, -1.0..1.0f64)
            .prop_filter_map("non-zero", |(x, y, z)| Dir3::new(Vec3::new(x, y, z), &T)),
        1 => prop::sample::select(vec![
            Dir3::X, Dir3::Y, Dir3::Z, Dir3::NEG_X, Dir3::NEG_Y, Dir3::NEG_Z,
        ]),
        1 => (-1.0..1.0f64, -1.0..1.0f64)
            .prop_filter_map("non-zero", |(x, y)| Dir3::new(Vec3::new(x, y, 0.0), &T)),
    ]
}

fn arb_box() -> impl Strategy<Value = Aabb3> {
    (arb_point(5.0), arb_point(5.0)).prop_map(|(a, b)| Aabb3::new(a, b))
}

fn arb_transform() -> impl Strategy<Value = Transform> {
    (
        arb_point(10.0),
        (-3.0..3.0f64, -3.0..3.0f64, -3.0..3.0f64),
        (0.1..3.0f64, 0.1..3.0f64, 0.1..3.0f64),
    )
        .prop_map(|(tr, (rx, ry, rz), (sx, sy, sz))| {
            Transform::from_scale_rotation_translation(
                Vec3::new(sx, sy, sz),
                Quat::from_euler(glam::EulerRot::XYZ, rx, ry, rz),
                tr,
            )
        })
}

fn arb_plane() -> impl Strategy<Value = Plane> {
    (arb_point(100.0), arb_dir()).prop_map(|(o, n)| Plane::from_normal(o, n))
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(512))]

    /// `ray_aabb` agrees with brute-force sampling along the ray.
    #[test]
    fn prop_ray_aabb_matches_sampling(o in arb_point(10.0), d in arb_dir(), b in arb_box()) {
        let r = ray(o, d);
        let hit = ray_aabb(&r, &b);
        let tol = Tolerance { linear: 1e-9, angular: 1e-9 };
        let t_max = 40.0;
        let n = 4000;
        let mut first_inside = None;
        for i in 0..=n {
            let t = t_max * f64::from(i) / f64::from(n);
            if b.contains(r.at(t), &tol) {
                first_inside = Some(t);
                break;
            }
        }
        if let Some(t) = first_inside {
            let (t0, t1) = hit.expect("sample inside the box but ray_aabb missed");
            prop_assert!(t0 <= t + 1e-9 && t <= t1 + 1e-9, "t={t} not in [{t0}, {t1}]");
        }
        if let Some((t0, t1)) = hit {
            prop_assert!(0.0 <= t0 && t0 <= t1);
            let loose = Tolerance { linear: 1e-9 * (1.0 + t1), angular: 1e-9 };
            prop_assert!(b.contains(r.at(t0), &loose));
            prop_assert!(b.contains(r.at(0.5 * (t0 + t1)), &loose));
            prop_assert!(b.contains(r.at(t1), &loose));
        }
    }

    /// Rays through shared edges and the shared vertex of a closed 1000-triangle fan never
    /// slip through (watertightness).
    #[test]
    fn prop_ray_triangle_fan_is_watertight(
        edge in 0usize..1000,
        s in 0.0f64..0.999,
        origin_xy in (-30.0f64..30.0, -30.0f64..30.0),
        height in 5.0f64..60.0,
        apex_z in -3.0f64..3.0,
    ) {
        let fan = fan(1000, 10.0, apex_z);
        let apex = fan[0].a;
        let rim = fan[edge].b; // shared by triangles edge-1 and edge
        let target = apex + (rim - apex) * s;
        let o = p3(origin_xy.0, origin_xy.1, height);
        let d = Dir3::new(target - o, &T).unwrap();
        let r = ray(o, d);
        let hits = fan.iter().filter(|t| ray_triangle(&r, t).is_some()).count();
        prop_assert!(hits >= 1, "ray through edge {edge} at s={s} missed the fan");
    }

    /// `Plane::to_world(to_local(p))` round-trips within 1e-9 mm for points on the plane.
    #[test]
    fn prop_plane_local_world_round_trip(pl in arb_plane(), uv in (-1000.0f64..1000.0, -1000.0f64..1000.0)) {
        let p = pl.to_world(p2(uv.0, uv.1));
        let back = pl.to_world(pl.to_local(p));
        prop_assert!(back.distance(p) <= 1e-9, "error {}", back.distance(p));
        let q = pl.to_local(p);
        prop_assert!((q - p2(uv.0, uv.1)).length() <= 1e-9);
    }

    /// Off-plane points round-trip to their projection.
    #[test]
    fn prop_plane_round_trip_projects(pl in arb_plane(), p in arb_point(1000.0)) {
        let back = pl.to_world(pl.to_local(p));
        prop_assert!(back.distance(pl.project(p)) <= 1e-9);
    }

    /// `Aabb3::transformed` contains all 8 transformed corners.
    #[test]
    fn prop_aabb_transformed_contains_corners(b in arb_box(), t in arb_transform()) {
        let tb = b.transformed(&t);
        let tol = Tolerance { linear: 1e-9, angular: 1e-9 };
        for i in 0..8 {
            let c = p3(
                if i & 1 == 0 { b.min.x } else { b.max.x },
                if i & 2 == 0 { b.min.y } else { b.max.y },
                if i & 4 == 0 { b.min.z } else { b.max.z },
            );
            prop_assert!(tb.contains(t.transform_point3(c), &tol));
        }
    }

    /// `segment_segment` is no worse than a brute-force sample of both segments.
    #[test]
    fn prop_segment_segment_is_minimal(a0 in arb_point(5.0), a1 in arb_point(5.0), b0 in arb_point(5.0), b1 in arb_point(5.0)) {
        let a = Segment3 { a: a0, b: a1 };
        let b = Segment3 { a: b0, b: b1 };
        let (s, t, pa, pb) = segment_segment(&a, &b);
        prop_assert!((0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t));
        let d = pa.distance(pb);
        let n = 60;
        for i in 0..=n {
            for j in 0..=n {
                let x = a0 + (a1 - a0) * (f64::from(i) / f64::from(n));
                let y = b0 + (b1 - b0) * (f64::from(j) / f64::from(n));
                prop_assert!(d <= x.distance(y) + 1e-9);
            }
        }
    }

    /// `point_triangle_closest` is no worse than sampling the triangle.
    #[test]
    fn prop_point_triangle_closest_is_minimal(p in arb_point(5.0), a in arb_point(5.0), b in arb_point(5.0), c in arb_point(5.0)) {
        let tri = Triangle3 { a, b, c };
        let q = point_triangle_closest(p, &tri);
        let d = q.distance(p);
        let n = 40;
        for i in 0..=n {
            for j in 0..=(n - i) {
                let u = f64::from(i) / f64::from(n);
                let v = f64::from(j) / f64::from(n);
                let x = a + (b - a) * u + (c - a) * v;
                prop_assert!(d <= x.distance(p) + 1e-9);
            }
        }
    }

    /// `ray_triangle` hit points lie on the ray and inside the triangle.
    #[test]
    fn prop_ray_triangle_hit_is_consistent(o in arb_point(10.0), a in arb_point(5.0), b in arb_point(5.0), c in arb_point(5.0), w in (0.0f64..1.0, 0.0f64..1.0)) {
        let tri = Triangle3 { a, b, c };
        prop_assume!(tri.normal(&T).is_some() && tri.area() > 1e-2);
        let (u, v) = if w.0 + w.1 > 1.0 { (1.0 - w.0, 1.0 - w.1) } else { w };
        let target = a + (b - a) * u + (c - a) * v;
        let Some(d) = Dir3::new(target - o, &T) else { return Ok(()); };
        let r = ray(o, d);
        let h = ray_triangle(&r, &tri);
        let Some(h) = h else {
            // A miss is only allowed for rays grazing the triangle plane.
            let n = tri.normal(&T).unwrap();
            prop_assert!(d.get().dot(n.get()).abs() < 1e-6);
            return Ok(());
        };
        let on_tri = a * h.bary[0] + b * h.bary[1] + c * h.bary[2];
        prop_assert!(on_tri.distance(r.at(h.t)) <= 1e-7 * (1.0 + h.t));
        prop_assert!(h.bary.iter().all(|&x| x >= -1e-12));
    }

    /// `plane_plane` gives a line lying in both planes.
    #[test]
    fn prop_plane_plane_line_in_both(a in arb_plane(), b in arb_plane()) {
        if let Some(l) = plane_plane(&a, &b, &T) {
            for t in [-50.0, 0.0, 50.0] {
                let p = l.point + l.dir.get() * t;
                let scale = 1.0 + a.origin.length() + b.origin.length();
                let s = (a.normal.get().cross(b.normal.get())).length();
                prop_assert!(a.signed_distance(p).abs() <= 1e-9 * scale / s.min(1.0));
                prop_assert!(b.signed_distance(p).abs() <= 1e-9 * scale / s.min(1.0));
            }
        }
    }
}

/// Closed fan of `n` triangles: apex at `(0, 0, apex_z)`, rim on a circle of `radius` at z = 0.
/// Triangle `i` is `(apex, rim[i], rim[i+1])`, so triangle `i-1` and `i` share `rim[i]`.
fn fan(n: usize, radius: f64, apex_z: f64) -> Vec<Triangle3> {
    let rim: Vec<Point3> = (0..n)
        .map(|i| {
            let a = std::f64::consts::TAU * (i as f64) / (n as f64);
            p3(radius * a.cos(), radius * a.sin(), 0.0)
        })
        .collect();
    let apex = p3(0.0, 0.0, apex_z);
    (0..n)
        .map(|i| Triangle3 {
            a: apex,
            b: rim[i],
            c: rim[(i + 1) % n],
        })
        .collect()
}

/// Deterministic sweep: every shared edge (at several points) and the shared apex.
#[test]
fn ray_triangle_fan_sweep_has_no_misses() {
    for apex_z in [0.0, 2.0] {
        let fan = fan(1000, 10.0, apex_z);
        let apex = fan[0].a;
        let origins = [p3(0.3, -0.7, 20.0), p3(-13.0, 4.0, 7.0), p3(0.0, 0.0, 50.0)];
        for o in origins {
            // Through the apex shared by all 1000 triangles.
            let r = ray(o, Dir3::new(apex - o, &T).unwrap());
            assert!(
                fan.iter().any(|t| ray_triangle(&r, t).is_some()),
                "apex missed"
            );
            for (i, tri) in fan.iter().enumerate() {
                for s in [1e-6, 0.1, 0.5, 0.9, 0.99] {
                    let target = apex + (tri.b - apex) * s;
                    let r = ray(o, Dir3::new(target - o, &T).unwrap());
                    let prev = &fan[(i + fan.len() - 1) % fan.len()];
                    assert!(
                        ray_triangle(&r, tri).is_some() || ray_triangle(&r, prev).is_some(),
                        "edge {i} s={s} apex_z={apex_z} origin={o:?} missed"
                    );
                }
            }
        }
    }
}
