//! Surface and curve types the document-level tests don't reach: cones, spheres, tori,
//! mirrored (left-handed) frames, elliptic and B-spline edges and free-form faces.

#![allow(clippy::unwrap_used, clippy::float_cmp, reason = "test code")]

use amg_brep::geom::{BSplineCurveData, BSplineSurfaceData, CurveSeg, FaceBounds, Tag};
use amg_brep::{BoolOp, BoolOpts, KernelConfig, OcctKernel};
use amg_jobs::JobCtx;

use super::super::super::CurveEval;
use super::*;

const EPS: f64 = 1e-9;

fn kernel() -> &'static OcctKernel {
    static KERNEL: std::sync::OnceLock<OcctKernel> = std::sync::OnceLock::new();
    KERNEL.get_or_init(|| OcctKernel::new(KernelConfig::default()))
}

fn tol() -> Tolerance {
    Tolerance::DEFAULT
}

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn close(a: Point3, b: Point3) -> bool {
    a.distance(b) < EPS
}

/// A frame at `origin` tilted off every axis.
fn tilted(origin: Point3) -> Plane {
    let n = Dir3::new(Vec3::new(1.0, 2.0, 3.0), &tol()).unwrap();
    Plane::from_normal(origin, n)
}

/// Resolves every face of `shape`.
fn faces(shape: &Shape) -> Vec<(ResolvedRef, bool)> {
    (0..shape.count(SubKind::Face))
        .map(|i| {
            sub_entity(
                kernel(),
                &tol(),
                shape,
                TopoKind::Face,
                i,
                &Transform::IDENTITY,
            )
            .unwrap()
        })
        .collect()
}

fn edges(shape: &Shape) -> Vec<ResolvedRef> {
    (0..shape.count(SubKind::Edge))
        .map(|i| {
            sub_entity(
                kernel(),
                &tol(),
                shape,
                TopoKind::Edge,
                i,
                &Transform::IDENTITY,
            )
            .unwrap()
            .0
        })
        .collect()
}

/// A point map.
type PointMap = Box<dyn Fn(Point3) -> Point3>;

/// The shape, its mirror image in a tilted plane, and the mirror map for points.
fn with_mirror(shape: Shape) -> [(Shape, PointMap); 2] {
    let plane = tilted(p(-3.0, 1.0, 2.0));
    let image = kernel().mirrored(&shape, &plane).unwrap().shape;
    [
        (shape, Box::new(|q| q)),
        (
            image,
            Box::new(move |q| q - plane.normal.get() * (2.0 * plane.signed_distance(q))),
        ),
    ]
}

/// `outer` minus `inner`.
fn cut(outer: &Shape, inner: &Shape) -> Shape {
    kernel()
        .boolean(
            BoolOp::Cut,
            std::slice::from_ref(outer),
            std::slice::from_ref(inner),
            &BoolOpts::default(),
            &JobCtx::detached(),
        )
        .unwrap()
        .shape
}

#[test]
fn cones_have_their_apex_axis_and_orientation_also_mirrored() {
    let frame = tilted(p(1.0, 2.0, 3.0));
    let n = frame.normal.get();
    // Radius 5 at the base, 2 at height 4: the apex is 20/3 up and the cone opens downwards.
    let solid = kernel()
        .make_cone(&frame, 5.0, 2.0, 4.0, TAU)
        .unwrap()
        .shape;
    // The same cone as a hole in a block.
    let block = kernel()
        .make_box(
            &Plane {
                origin: frame.origin - frame.x.get() * 10.0 - frame.y.get() * 10.0,
                ..frame
            },
            Vec3::new(20.0, 20.0, 4.0),
        )
        .unwrap()
        .shape;
    let holed = cut(&block, &solid);
    for (shape, hole) in [(solid, false), (holed, true)] {
        for (shape, map) in with_mirror(shape) {
            let cones: Vec<_> = faces(&shape)
                .into_iter()
                .filter_map(|(r, reversed)| match r.geom {
                    RefGeom::Cone {
                        apex,
                        axis,
                        half_angle,
                    } => Some((apex, axis, half_angle, reversed)),
                    _ => None,
                })
                .collect();
            assert_eq!(cones.len(), 1);
            let (apex, axis, half_angle, reversed) = cones[0];
            let want_apex = map(frame.origin + n * (20.0 / 3.0));
            assert!(close(apex, want_apex), "{apex} vs {want_apex}");
            let want_axis = map(frame.origin) - want_apex;
            assert!(close(axis.get(), want_axis.normalize()));
            assert!((half_angle - 0.75_f64.atan()).abs() < EPS);
            assert_eq!(reversed, hole);
        }
    }
}

#[test]
fn spheres_tori_and_cylinders_report_holes_as_reversed_also_mirrored() {
    let frame = tilted(p(2.0, -1.0, 4.0));
    let full = TAU;
    let sphere = kernel().make_sphere(&frame, 3.0).unwrap().shape;
    let torus = kernel().make_torus(&frame, 8.0, 2.0, full).unwrap().shape;
    let cylinder = kernel()
        .make_cylinder(&frame, 2.5, 6.0, full)
        .unwrap()
        .shape;
    let block = kernel()
        .make_box(
            &Plane {
                origin: frame.origin
                    - frame.x.get() * 20.0
                    - frame.y.get() * 20.0
                    - frame.normal.get() * 20.0,
                ..frame
            },
            Vec3::new(40.0, 40.0, 40.0),
        )
        .unwrap()
        .shape;
    for tool in [sphere, torus, cylinder] {
        for (shape, hole) in [(tool.clone(), false), (cut(&block, &tool), true)] {
            for (shape, map) in with_mirror(shape) {
                let centre = map(frame.origin);
                let mut curved = 0;
                for (r, reversed) in faces(&shape) {
                    match r.geom {
                        RefGeom::Sphere { center, radius } => {
                            assert!(close(center, centre));
                            assert_eq!(radius, 3.0);
                        }
                        RefGeom::Torus { axis, major, minor } => {
                            assert!(close(axis.point, centre));
                            assert_eq!((major, minor), (8.0, 2.0));
                        }
                        RefGeom::Cylinder { axis, radius } => {
                            assert_eq!(radius, 2.5);
                            let d = axis.point - centre;
                            let a = axis.dir.get();
                            assert!((d - a * a.dot(d)).length() < EPS);
                        }
                        RefGeom::Plane(_) => continue,
                        other => panic!("unexpected {}", other.kind_name()),
                    }
                    curved += 1;
                    assert_eq!(reversed, hole);
                }
                assert!(curved >= 1);
            }
        }
    }
}

#[test]
fn planar_face_normals_point_out_of_the_material_also_mirrored() {
    let block = kernel()
        .make_box(&Plane::XY, Vec3::new(4.0, 5.0, 6.0))
        .unwrap()
        .shape;
    for (shape, map) in with_mirror(block) {
        let centre = map(p(2.0, 2.5, 3.0));
        let planes = faces(&shape);
        assert_eq!(planes.len(), 6);
        for (r, reversed) in planes {
            let RefGeom::Plane(plane) = r.geom else {
                panic!("not a plane");
            };
            assert!(!reversed);
            assert!(plane.signed_distance(centre) < -1.9);
            let (x, y) = (plane.x.get(), plane.y.get());
            assert!(close(x.cross(y), plane.normal.get()), "right-handed");
        }
    }
}

/// A wire of one edge.
fn edge_of(curve: Curve3, t0: f64, t1: f64) -> Shape {
    kernel()
        .make_wire(
            &[CurveSeg {
                curve,
                t0,
                t1,
                tag: Tag(1),
            }],
            &tol(),
        )
        .unwrap()
        .shape
}

#[test]
fn circular_arcs_start_and_end_on_the_edge() {
    let frame = tilted(p(1.0, 1.0, 1.0));
    let curve = Curve3::Circle { frame, radius: 4.0 };
    let wire = edge_of(curve, 0.4, 2.1);
    let r = &edges(&wire)[0];
    let RefGeom::Circle {
        center,
        normal,
        radius,
        arc: Some((a, b)),
    } = r.geom
    else {
        panic!("not an arc: {:?}", r.geom);
    };
    assert_eq!(radius, 4.0);
    assert!(close(center, frame.origin));
    assert!(r.context_plane.is_none(), "a wire has no faces");
    assert!((b - a - 1.7).abs() < EPS);
    let path = super::super::super::curves::CircleCurve::new(center, normal, radius, Some((a, b)))
        .unwrap();
    let on_circle = |t: f64| frame.to_world(Point2::new(4.0 * t.cos(), 4.0 * t.sin()));
    assert!(close(path.eval(a), on_circle(0.4)));
    assert!(close(path.eval(b), on_circle(2.1)));
}

#[test]
fn elliptic_and_bspline_edges_become_curves_on_the_edge() {
    let frame = tilted(p(0.0, 2.0, 0.0));
    let ellipse = Curve3::Ellipse {
        frame,
        major_radius: 6.0,
        minor_radius: 2.0,
    };
    let spline = Curve3::BSpline(BSplineCurveData {
        degree: 3,
        knots: vec![0.0, 0.5, 1.0],
        mults: vec![4, 1, 4],
        poles: vec![
            p(0.0, 0.0, 0.0),
            p(2.0, 5.0, 1.0),
            p(5.0, -3.0, 2.0),
            p(8.0, 4.0, 0.0),
            p(10.0, 0.0, 3.0),
        ],
        weights: None,
        periodic: false,
    });
    for (curve, t0, t1) in [(ellipse, 0.3, 4.0), (spline, 0.1, 0.9)] {
        let wire = edge_of(curve, t0, t1);
        let r = &edges(&wire)[0];
        let RefGeom::Curve(c) = &r.geom else {
            panic!("not a curve: {:?}", r.geom);
        };
        let (curve, a, b) = wire.curve(EdgeId(0)).unwrap();
        assert_eq!(c.range(), (a, b));
        // The kernel's own tessellation of the edge lies on the adapter.
        let points = kernel()
            .tessellate_edge(&wire, EdgeId(0), &amg_brep::TessQuality::display())
            .unwrap();
        for q in points {
            assert!(c.eval(c.closest_param(q)).distance(q) < 1e-7);
        }
        for i in 0..=10 {
            let t = a + (b - a) * f64::from(i) / 10.0;
            if let Some(want) = curve.point_at(t) {
                assert!(close(c.eval(t), want));
            }
        }
    }
}

#[test]
fn a_bspline_face_becomes_a_surface_with_outward_normals() {
    // A bicubic bump over [0, 10]².
    let mut poles = Vec::new();
    for i in 0..4 {
        for j in 0..4 {
            let bump = if (1..=2).contains(&i) && (1..=2).contains(&j) {
                3.0
            } else {
                0.0
            };
            poles.push(p(
                f64::from(i) * 10.0 / 3.0,
                f64::from(j) * 10.0 / 3.0,
                bump,
            ));
        }
    }
    let surface = Surface::BSpline(BSplineSurfaceData {
        u_degree: 3,
        v_degree: 3,
        u_knots: vec![0.0, 1.0],
        u_mults: vec![4, 4],
        v_knots: vec![0.0, 1.0],
        v_mults: vec![4, 4],
        poles,
        nu: 4,
        nv: 4,
        weights: None,
        u_periodic: false,
        v_periodic: false,
    });
    let face = kernel()
        .make_face(&surface, &FaceBounds::Natural, 1e-7)
        .unwrap()
        .face;
    let (r, reversed) = faces(&face).remove(0);
    assert!(!reversed);
    let RefGeom::Surface(s) = r.geom else {
        panic!("not a surface: {:?}", r.geom);
    };
    // The centre of the bump: z = 3 · (B₁ + B₂)² at t = ½ = 3 · (3/4)².
    let mid = Point2::new(0.5, 0.5);
    assert!(close(s.eval(mid), p(5.0, 5.0, 27.0 / 16.0)));
    assert!(close(s.normal(mid).unwrap().get(), Vec3::Z));
    let uv = s.closest_uv(p(5.0, 5.0, 20.0)).unwrap();
    assert!((uv - mid).length() < 1e-6);
}

#[test]
fn a_transform_moves_every_sub_entity_and_a_bad_index_is_an_error() {
    let block = kernel()
        .make_box(&Plane::XY, Vec3::new(1.0, 2.0, 3.0))
        .unwrap()
        .shape;
    let t = Transform::from_translation(Vec3::new(10.0, 0.0, 0.0));
    for i in 0..8 {
        let at = |t: &Transform| {
            let (r, _) = sub_entity(kernel(), &tol(), &block, TopoKind::Vertex, i, t).unwrap();
            let RefGeom::Point(q) = r.geom else {
                panic!("not a point");
            };
            q
        };
        assert!(close(
            at(&t),
            at(&Transform::IDENTITY) + Vec3::new(10.0, 0.0, 0.0)
        ));
    }
    let missing = sub_entity(
        kernel(),
        &tol(),
        &block,
        TopoKind::Face,
        6,
        &Transform::IDENTITY,
    );
    assert!(matches!(missing, Err(Fail::Missing(_))));
}
