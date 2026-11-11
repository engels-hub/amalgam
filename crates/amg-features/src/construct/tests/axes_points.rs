//! CON-02 unit tests: every axis and point variant on analytic inputs through the CON-01 test
//! resolver (pre-resolved `RefGeom`), plus every error the axis and point constructions can
//! raise (note §6).

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

use super::*;

// ---- runners -------------------------------------------------------------------------

/// What a successful axis case must produce.
#[derive(Clone, Copy, Debug)]
struct AxisWant {
    dir: Vec3,
    /// A point the axis must contain.
    through: Point3,
    /// Exact `Line3::point`, when the convention fixes it.
    point: Option<Point3>,
}

fn axis_want(dir: Vec3, through: Point3) -> AxisWant {
    AxisWant {
        dir,
        through,
        point: None,
    }
}

impl AxisWant {
    fn at(mut self, p: Point3) -> AxisWant {
        self.point = Some(p);
        self
    }
}

struct AxisCase {
    name: &'static str,
    env: Env,
    def: AxisDef,
    want: Result<AxisWant, ConstructError>,
}

fn axis_ok(name: &'static str, env: Env, def: AxisDef, w: AxisWant) -> AxisCase {
    AxisCase {
        name,
        env,
        def,
        want: Ok(w),
    }
}

fn axis_err(name: &'static str, env: Env, def: AxisDef, e: ConstructError) -> AxisCase {
    AxisCase {
        name,
        env,
        def,
        want: Err(e),
    }
}

fn line_bits(l: &Line3) -> [u64; 6] {
    let [a, b, c] = l.point.to_array().map(f64::to_bits);
    let [d, e, f] = l.dir.get().to_array().map(f64::to_bits);
    [a, b, c, d, e, f]
}

/// Runs an axis table: each case twice (bit-identical), checking the expectation.
fn run_axes(cases: Vec<AxisCase>, variant: &str, min_ok: usize) {
    let n_ok = cases.iter().filter(|c| c.want.is_ok()).count();
    assert!(n_ok >= min_ok, "{variant}: only {n_ok} analytic cases");
    for c in cases {
        let got = c.def.evaluate(&c.env);
        match c.want {
            Ok(w) => {
                let (l, warnings) = got.unwrap_or_else(|e| panic!("{}: {e}", c.name));
                assert!(warnings.is_empty(), "{}", c.name);
                let d = l.dir.get();
                assert!((d.length() - 1.0).abs() <= EPS, "{}: |dir|", c.name);
                assert!(close(d, w.dir), "{}: dir {d:?}, want {:?}", c.name, w.dir);
                let off = (w.through - l.point).cross(d).length();
                assert!(off <= EPS, "{}: {:?} is {off} off", c.name, w.through);
                if let Some(p) = w.point {
                    assert!(close(l.point, p), "{}: point {:?}", c.name, l.point);
                }
                let (again, _) = c.def.evaluate(&c.env).unwrap();
                assert_eq!(
                    line_bits(&l),
                    line_bits(&again),
                    "{}: not bit-identical",
                    c.name
                );
            }
            Err(e) => {
                assert_eq!(got.map(|_| ()), Err(FeatureError::from(e)), "{}", c.name);
            }
        }
    }
}

struct PointCase {
    name: &'static str,
    env: Env,
    def: PointDef,
    want: Result<Point3, ConstructError>,
}

fn pt_ok(name: &'static str, env: Env, def: PointDef, p: Point3) -> PointCase {
    PointCase {
        name,
        env,
        def,
        want: Ok(p),
    }
}

fn pt_err(name: &'static str, env: Env, def: PointDef, e: ConstructError) -> PointCase {
    PointCase {
        name,
        env,
        def,
        want: Err(e),
    }
}

/// Runs a point table: each case twice (bit-identical), checking the expectation.
fn run_points(cases: Vec<PointCase>, variant: &str, min_ok: usize) {
    let n_ok = cases.iter().filter(|c| c.want.is_ok()).count();
    assert!(n_ok >= min_ok, "{variant}: only {n_ok} analytic cases");
    for c in cases {
        let got = c.def.evaluate(&c.env);
        match c.want {
            Ok(w) => {
                let (p, warnings) = got.unwrap_or_else(|e| panic!("{}: {e}", c.name));
                assert!(warnings.is_empty(), "{}", c.name);
                assert!(close(p, w), "{}: {p:?}, want {w:?}", c.name);
                let (again, _) = c.def.evaluate(&c.env).unwrap();
                assert_eq!(
                    p.to_array().map(f64::to_bits),
                    again.to_array().map(f64::to_bits),
                    "{}: not bit-identical",
                    c.name
                );
            }
            Err(e) => {
                assert_eq!(got.map(|_| ()), Err(FeatureError::from(e)), "{}", c.name);
            }
        }
    }
}

fn z_axis() -> Line3 {
    Line3 {
        point: Point3::ZERO,
        dir: Dir3::Z,
    }
}

fn seg(a: Point3, b: Point3) -> RefGeom {
    RefGeom::Segment(Segment3 { a, b })
}

fn circle(center: Point3, normal: Dir3, radius: f64, arc: Option<(f64, f64)>) -> RefGeom {
    RefGeom::Circle {
        center,
        normal,
        radius,
        arc,
    }
}

/// Env with two inputs.
fn pair(a: RefGeom, b: RefGeom) -> (Env, RefInput, RefInput) {
    let mut env = Env::default();
    let ra = env.add(a);
    let rb = env.add(b);
    (env, ra, rb)
}

// ---- axes ----------------------------------------------------------------------------

#[test]
fn axis_through_revolved() {
    let rev = |face_or_edge| AxisDef::ThroughRevolved { face_or_edge };
    let oblique = dir(Vec3::new(1.0, 1.0, 0.0));
    let case = |name, g: RefGeom, w: Result<AxisWant, ConstructError>| {
        let (env, r) = single(g);
        AxisCase {
            name,
            env,
            def: rev(r),
            want: w,
        }
    };
    let cases = vec![
        case(
            "cylinder on Z",
            RefGeom::Cylinder {
                axis: z_axis(),
                radius: 3.0,
            },
            Ok(axis_want(Vec3::Z, Point3::ZERO).at(Point3::ZERO)),
        ),
        case(
            "oblique cylinder keeps its stored axis",
            RefGeom::Cylinder {
                axis: Line3 {
                    point: p3(1.0, 2.0, 3.0),
                    dir: oblique,
                },
                radius: 1.0,
            },
            Ok(axis_want(oblique.get(), p3(1.0, 2.0, 3.0)).at(p3(1.0, 2.0, 3.0))),
        ),
        case(
            "cone opening along −Z: axis through the apex",
            RefGeom::Cone {
                apex: p3(0.0, 0.0, 5.0),
                axis: Dir3::NEG_Z,
                half_angle: 0.3,
            },
            Ok(axis_want(Vec3::NEG_Z, p3(0.0, 0.0, 5.0)).at(p3(0.0, 0.0, 5.0))),
        ),
        case(
            "torus on X",
            RefGeom::Torus {
                axis: Line3 {
                    point: p3(1.0, 0.0, 0.0),
                    dir: Dir3::X,
                },
                major: 5.0,
                minor: 1.0,
            },
            Ok(axis_want(Vec3::X, p3(1.0, 0.0, 0.0)).at(p3(1.0, 0.0, 0.0))),
        ),
        case(
            "circular edge: its normal through the centre",
            circle(p3(0.0, 0.0, 2.0), Dir3::Y, 4.0, None),
            Ok(axis_want(Vec3::Y, p3(0.0, 0.0, 2.0)).at(p3(0.0, 0.0, 2.0))),
        ),
        case(
            "arc",
            circle(p3(1.0, 1.0, 1.0), Dir3::NEG_X, 2.0, Some((0.0, FRAC_PI_2))),
            Ok(axis_want(Vec3::NEG_X, p3(1.0, 1.0, 1.0)).at(p3(1.0, 1.0, 1.0))),
        ),
        // Errors.
        case(
            "sphere has no axis",
            RefGeom::Sphere {
                center: Point3::ZERO,
                radius: 1.0,
            },
            Err(ConstructError::Unsupported),
        ),
        case(
            "plane has no axis",
            RefGeom::Plane(Plane::XY),
            Err(ConstructError::Unsupported),
        ),
        case(
            "free-form surface",
            RefGeom::Surface(Arc::new(TestFace {
                closest_fails: false,
                singular: false,
            })),
            Err(ConstructError::Unsupported),
        ),
        case(
            "a point",
            RefGeom::Point(Point3::ZERO),
            Err(ConstructError::WrongInput {
                expected: "cylinder, cone or torus face, or circular edge",
                got: "point",
            }),
        ),
        case(
            "zero-radius cylinder",
            RefGeom::Cylinder {
                axis: z_axis(),
                radius: 0.0,
            },
            Err(ConstructError::Degenerate("radius is not positive")),
        ),
        case(
            "flat cone",
            RefGeom::Cone {
                apex: Point3::ZERO,
                axis: Dir3::Z,
                half_angle: FRAC_PI_2,
            },
            Err(ConstructError::Degenerate(
                "cone half angle outside (0, π/2)",
            )),
        ),
    ];
    run_axes(cases, "through revolved", 5);
}

/// Axis cases on a face and a point, for both perpendicular variants.
fn perp(
    on_face: bool,
    name: &'static str,
    g: RefGeom,
    p: Point3,
    w: Result<AxisWant, ConstructError>,
) -> AxisCase {
    let (env, face, point) = pair(g, RefGeom::Point(p));
    let def = if on_face {
        AxisDef::PerpendicularAtPoint { face, point }
    } else {
        AxisDef::PerpendicularToFaceAtPoint { face, point }
    };
    AxisCase {
        name,
        env,
        def,
        want: w,
    }
}

fn sphere5() -> RefGeom {
    RefGeom::Sphere {
        center: Point3::ZERO,
        radius: 5.0,
    }
}

fn cyl2() -> RefGeom {
    RefGeom::Cylinder {
        axis: z_axis(),
        radius: 2.0,
    }
}

fn torus51() -> RefGeom {
    RefGeom::Torus {
        axis: z_axis(),
        major: 5.0,
        minor: 1.0,
    }
}

fn test_face(closest_fails: bool, singular: bool) -> RefGeom {
    RefGeom::Surface(Arc::new(TestFace {
        closest_fails,
        singular,
    }))
}

#[test]
fn axis_perpendicular_at_point() {
    let c = |name, g, p, w| perp(true, name, g, p, w);
    let cases = vec![
        c(
            "planar face",
            RefGeom::Plane(Plane::XY),
            p3(1.0, 2.0, 0.0),
            Ok(axis_want(Vec3::Z, p3(1.0, 2.0, 0.0)).at(p3(1.0, 2.0, 0.0))),
        ),
        c(
            "point within tolerance of the face: the axis keeps the picked point",
            RefGeom::Plane(Plane::XY),
            p3(1.0, 1.0, 5e-7),
            Ok(axis_want(Vec3::Z, p3(1.0, 1.0, 0.0)).at(p3(1.0, 1.0, 5e-7))),
        ),
        c(
            "sphere",
            sphere5(),
            p3(0.0, 5.0, 0.0),
            Ok(axis_want(Vec3::Y, Point3::ZERO).at(p3(0.0, 5.0, 0.0))),
        ),
        c(
            "cylinder",
            cyl2(),
            p3(0.0, -2.0, 4.0),
            Ok(axis_want(Vec3::NEG_Y, p3(0.0, 0.0, 4.0)).at(p3(0.0, -2.0, 4.0))),
        ),
        c(
            "45° cone",
            RefGeom::Cone {
                apex: Point3::ZERO,
                axis: Dir3::Z,
                half_angle: FRAC_PI_4,
            },
            p3(1.0, 0.0, 1.0),
            Ok(axis_want(
                Vec3::new(FRAC_1_SQRT_2, 0.0, -FRAC_1_SQRT_2),
                p3(1.0, 0.0, 1.0),
            )
            .at(p3(1.0, 0.0, 1.0))),
        ),
        c(
            "torus outer equator",
            torus51(),
            p3(6.0, 0.0, 0.0),
            Ok(axis_want(Vec3::X, p3(6.0, 0.0, 0.0)).at(p3(6.0, 0.0, 0.0))),
        ),
        c(
            "free-form face",
            test_face(false, false),
            p3(3.0, 4.0, 1.0),
            Ok(axis_want(Vec3::Z, p3(3.0, 4.0, 1.0)).at(p3(3.0, 4.0, 1.0))),
        ),
        // Errors.
        c(
            "point 0.5 mm off the face",
            RefGeom::Plane(Plane::XY),
            p3(1.0, 2.0, 0.5),
            Err(ConstructError::NotOnFace { distance: 0.5 }),
        ),
        c(
            "point 1 mm outside the sphere",
            sphere5(),
            p3(0.0, 0.0, 6.0),
            Err(ConstructError::NotOnFace { distance: 1.0 }),
        ),
        c(
            "sphere centre",
            sphere5(),
            Point3::ZERO,
            Err(ConstructError::Degenerate(
                "point on the surface axis or centre",
            )),
        ),
        c(
            "closest-point query fails",
            test_face(true, false),
            p3(0.0, 0.0, 1.0),
            Err(ConstructError::NotOnFace {
                distance: f64::INFINITY,
            }),
        ),
        c(
            "singular face point",
            test_face(false, true),
            p3(0.0, 0.0, 1.0),
            Err(ConstructError::Degenerate("singular point on the face")),
        ),
        c(
            "a line is not a face",
            RefGeom::Line(z_axis()),
            Point3::ZERO,
            Err(ConstructError::WrongInput {
                expected: "face",
                got: "line",
            }),
        ),
    ];
    run_axes(cases, "perpendicular at point", 5);

    // A non-point second input.
    let (env, face, point) = pair(RefGeom::Plane(Plane::XY), RefGeom::Plane(Plane::XY));
    assert_eq!(
        AxisDef::PerpendicularAtPoint { face, point }
            .evaluate(&env)
            .map(|_| ()),
        Err(FeatureError::from(ConstructError::WrongInput {
            expected: "point",
            got: "plane",
        }))
    );
}

#[test]
fn axis_perpendicular_to_face_at_point() {
    let c = |name, g, p, w| perp(false, name, g, p, w);
    let ring = p3(5.0, 0.0, 0.0);
    let tn = Vec3::new(3.0, 0.0, 1.0).normalize();
    let oblique = Plane::from_normal(p3(1.0, 1.0, 1.0), dir(Vec3::ONE));
    let cases = vec![
        c(
            "planar face from above",
            RefGeom::Plane(Plane::XY),
            p3(1.0, 2.0, 3.0),
            Ok(axis_want(Vec3::Z, p3(1.0, 2.0, 0.0)).at(p3(1.0, 2.0, 0.0))),
        ),
        c(
            "oblique plane from the origin",
            RefGeom::Plane(oblique),
            Point3::ZERO,
            Ok(axis_want(Vec3::ONE.normalize(), Point3::ZERO).at(p3(1.0, 1.0, 1.0))),
        ),
        c(
            "sphere from outside",
            sphere5(),
            p3(0.0, 0.0, 10.0),
            Ok(axis_want(Vec3::Z, p3(0.0, 0.0, 5.0)).at(p3(0.0, 0.0, 5.0))),
        ),
        c(
            "cylinder from outside",
            cyl2(),
            p3(3.0, 0.0, 7.0),
            Ok(axis_want(Vec3::X, p3(2.0, 0.0, 7.0)).at(p3(2.0, 0.0, 7.0))),
        ),
        c(
            "cylinder from inside: normal still away from the axis",
            cyl2(),
            p3(1.0, 0.0, 7.0),
            Ok(axis_want(Vec3::X, p3(2.0, 0.0, 7.0)).at(p3(2.0, 0.0, 7.0))),
        ),
        c(
            "torus",
            torus51(),
            p3(8.0, 0.0, 1.0),
            Ok(axis_want(tn, ring).at(ring + tn)),
        ),
        c(
            "free-form face",
            test_face(false, false),
            p3(3.0, 4.0, 9.0),
            Ok(axis_want(Vec3::Z, p3(3.0, 4.0, 1.0)).at(p3(3.0, 4.0, 1.0))),
        ),
        // Errors.
        c(
            "point on the cylinder axis",
            cyl2(),
            p3(0.0, 0.0, 3.0),
            Err(ConstructError::Degenerate(
                "point on the surface axis or centre",
            )),
        ),
        c(
            "closest point is the cone apex",
            RefGeom::Cone {
                apex: Point3::ZERO,
                axis: Dir3::Z,
                half_angle: FRAC_PI_4,
            },
            p3(0.1, 0.0, -1.0),
            Err(ConstructError::Degenerate("closest point is the cone apex")),
        ),
        c(
            "closest-point query fails",
            test_face(true, false),
            p3(0.0, 0.0, 3.0),
            Err(ConstructError::NotOnFace {
                distance: f64::INFINITY,
            }),
        ),
        c(
            "a circle is not a face",
            circle(Point3::ZERO, Dir3::Z, 1.0, None),
            Point3::ZERO,
            Err(ConstructError::WrongInput {
                expected: "face",
                got: "circle",
            }),
        ),
    ];
    run_axes(cases, "perpendicular to face at point", 5);
}

#[test]
fn axis_through_two_planes() {
    let two = |a, b| AxisDef::ThroughTwoPlanes { a, b };
    let tilted = Plane::from_normal(Point3::ZERO, dir(Vec3::new(1.0, 0.0, 1.0)));
    let mut cases = vec![
        axis_ok(
            "XY ∩ YZ: Z × X = +Y",
            Env::default(),
            two(XY, YZ),
            axis_want(Vec3::Y, Point3::ZERO).at(Point3::ZERO),
        ),
        axis_ok(
            "XY ∩ XZ: Z × (−Y) = +X",
            Env::default(),
            two(XY, XZ),
            axis_want(Vec3::X, Point3::ZERO).at(Point3::ZERO),
        ),
        axis_ok(
            "YZ ∩ XY: X × Z = −Y",
            Env::default(),
            two(YZ, XY),
            axis_want(Vec3::NEG_Y, Point3::ZERO).at(Point3::ZERO),
        ),
    ];
    let mut env = Env::default();
    let a = env.plane(plane_z(5.0));
    let b = env.plane(plane_offset(&Plane::YZ, 2.0));
    cases.push(axis_ok(
        "z = 5 ∩ x = 2: point closest to the origin",
        env,
        two(a, b),
        axis_want(Vec3::Y, p3(2.0, 7.0, 5.0)).at(p3(2.0, 0.0, 5.0)),
    ));
    let mut env = Env::default();
    let a = env.plane(tilted);
    cases.push(axis_ok(
        "45° plane ∩ XY",
        env,
        two(a, XY),
        axis_want(Vec3::NEG_Y, Point3::ZERO).at(Point3::ZERO),
    ));
    let mut env = Env::default();
    let a = env.add(circle(p3(0.0, 0.0, 3.0), Dir3::Z, 1.0, None));
    cases.push(axis_ok(
        "plane of a circular edge ∩ YZ",
        env,
        two(a, YZ),
        axis_want(Vec3::Y, p3(0.0, 0.0, 3.0)).at(p3(0.0, 0.0, 3.0)),
    ));
    // Errors.
    let mut env = Env::default();
    let a = env.plane(plane_z(5.0));
    cases.push(axis_err(
        "parallel planes",
        env,
        two(XY, a),
        ConstructError::Parallel,
    ));
    cases.push(axis_err(
        "the same plane",
        Env::default(),
        two(XY, XY),
        ConstructError::Parallel,
    ));
    let mut env = Env::default();
    let a = env.plane(Plane::XY.flipped());
    cases.push(axis_err(
        "opposite normals",
        env,
        two(XY, a),
        ConstructError::Parallel,
    ));
    let mut env = Env::default();
    let a = env.point(0.0, 0.0, 0.0);
    cases.push(axis_err(
        "a point",
        env,
        two(XY, a),
        ConstructError::WrongInput {
            expected: "plane",
            got: "point",
        },
    ));
    run_axes(cases, "through two planes", 5);
}

#[test]
fn axis_through_two_points() {
    let case = |name, a: Point3, b: Point3, w| {
        let (env, ra, rb) = pair(RefGeom::Point(a), RefGeom::Point(b));
        AxisCase {
            name,
            env,
            def: AxisDef::ThroughTwoPoints { a: ra, b: rb },
            want: w,
        }
    };
    let mut cases = vec![
        case(
            "origin → (1, 0, 0)",
            Point3::ZERO,
            p3(1.0, 0.0, 0.0),
            Ok(axis_want(Vec3::X, Point3::ZERO).at(Point3::ZERO)),
        ),
        case(
            "diagonal",
            p3(1.0, 1.0, 1.0),
            p3(2.0, 2.0, 2.0),
            Ok(axis_want(Vec3::ONE.normalize(), Point3::ZERO).at(p3(1.0, 1.0, 1.0))),
        ),
        case(
            "downwards",
            p3(0.0, 0.0, 5.0),
            p3(0.0, 0.0, -5.0),
            Ok(axis_want(Vec3::NEG_Z, Point3::ZERO).at(p3(0.0, 0.0, 5.0))),
        ),
        case(
            "vertical through (−1, 2, ·)",
            p3(-1.0, 2.0, 3.0),
            p3(-1.0, 2.0, 10.0),
            Ok(axis_want(Vec3::Z, p3(-1.0, 2.0, 0.0)).at(p3(-1.0, 2.0, 3.0))),
        ),
        // Errors.
        case(
            "the same point",
            p3(1.0, 2.0, 3.0),
            p3(1.0, 2.0, 3.0),
            Err(ConstructError::Coincident),
        ),
        case(
            "points 5e-7 mm apart",
            p3(1.0, 2.0, 3.0),
            p3(1.0, 2.0, 3.0 + 5e-7),
            Err(ConstructError::Coincident),
        ),
    ];
    let mut env = Env::default();
    let b = env.point(3.0, 4.0, 0.0);
    cases.push(axis_ok(
        "origin point → construction point",
        env,
        AxisDef::ThroughTwoPoints {
            a: RefInput::Origin(OriginEntity::Point),
            b,
        },
        axis_want(Vec3::new(0.6, 0.8, 0.0), p3(6.0, 8.0, 0.0)).at(Point3::ZERO),
    ));
    let mut env = Env::default();
    let b = env.plane(Plane::XY);
    cases.push(axis_err(
        "a plane",
        env,
        AxisDef::ThroughTwoPoints {
            a: RefInput::Origin(OriginEntity::Point),
            b,
        },
        ConstructError::WrongInput {
            expected: "point",
            got: "plane",
        },
    ));
    run_axes(cases, "through two points", 5);
}

#[test]
fn axis_through_edge() {
    let case = |name, g: RefGeom, w| {
        let (env, edge) = single(g);
        AxisCase {
            name,
            env,
            def: AxisDef::ThroughEdge { edge },
            want: w,
        }
    };
    let oblique = dir(Vec3::new(1.0, 2.0, 2.0));
    let cases = vec![
        case(
            "straight edge, directed start → end",
            seg(Point3::ZERO, p3(10.0, 0.0, 0.0)),
            Ok(axis_want(Vec3::X, Point3::ZERO).at(Point3::ZERO)),
        ),
        case(
            "reversed edge",
            seg(p3(10.0, 0.0, 0.0), Point3::ZERO),
            Ok(axis_want(Vec3::NEG_X, Point3::ZERO).at(p3(10.0, 0.0, 0.0))),
        ),
        case(
            "oblique edge",
            seg(p3(1.0, 1.0, 1.0), p3(2.0, 3.0, 3.0)),
            Ok(axis_want(oblique.get(), p3(3.0, 5.0, 5.0)).at(p3(1.0, 1.0, 1.0))),
        ),
        case(
            "construction axis",
            RefGeom::Line(Line3 {
                point: p3(0.0, 1.0, 0.0),
                dir: Dir3::Z,
            }),
            Ok(axis_want(Vec3::Z, p3(0.0, 1.0, 9.0)).at(p3(0.0, 1.0, 0.0))),
        ),
        case(
            "straight free-form curve",
            RefGeom::Curve(Arc::new(
                SegmentCurve::new(&Segment3 {
                    a: p3(1.0, 1.0, 0.0),
                    b: p3(1.0, 5.0, 0.0),
                })
                .unwrap(),
            )),
            Ok(axis_want(Vec3::Y, p3(1.0, 1.0, 0.0)).at(p3(1.0, 1.0, 0.0))),
        ),
        // Errors.
        case(
            "circular edge",
            circle(Point3::ZERO, Dir3::Z, 1.0, None),
            Err(ConstructError::NotStraight),
        ),
        case(
            "curved free-form edge",
            RefGeom::Curve(Arc::new(
                CircleCurve::new(Point3::ZERO, Dir3::Z, 5.0, Some((0.0, FRAC_PI_2))).unwrap(),
            )),
            Err(ConstructError::NotStraight),
        ),
        case(
            "zero-length edge",
            seg(p3(1.0, 1.0, 1.0), p3(1.0, 1.0, 1.0)),
            Err(ConstructError::Degenerate("zero-length edge")),
        ),
        case(
            "a plane",
            RefGeom::Plane(Plane::XY),
            Err(ConstructError::WrongInput {
                expected: "line or straight edge",
                got: "plane",
            }),
        ),
    ];
    run_axes(cases, "through edge", 5);
}

// ---- points --------------------------------------------------------------------------

#[test]
fn point_at_vertex() {
    let case = |name, g: RefGeom, w| {
        let (env, point) = single(g);
        PointCase {
            name,
            env,
            def: PointDef::AtVertex { point },
            want: w,
        }
    };
    let mut cases = vec![
        case(
            "vertex",
            RefGeom::Point(p3(1.0, 2.0, 3.0)),
            Ok(p3(1.0, 2.0, 3.0)),
        ),
        case(
            "negative coordinates",
            RefGeom::Point(p3(-4.5, 0.0, -1e-3)),
            Ok(p3(-4.5, 0.0, -1e-3)),
        ),
        case(
            "large coordinates",
            RefGeom::Point(p3(1e4, -2e4, 3e4)),
            Ok(p3(1e4, -2e4, 3e4)),
        ),
        case(
            "origin as a vertex",
            RefGeom::Point(Point3::ZERO),
            Ok(Point3::ZERO),
        ),
        // Errors.
        case(
            "a line",
            RefGeom::Line(z_axis()),
            Err(ConstructError::WrongInput {
                expected: "point",
                got: "line",
            }),
        ),
        case(
            "a circle (use at centre)",
            circle(Point3::ZERO, Dir3::Z, 1.0, None),
            Err(ConstructError::WrongInput {
                expected: "point",
                got: "circle",
            }),
        ),
    ];
    cases.push(pt_ok(
        "origin point",
        Env::default(),
        PointDef::AtVertex {
            point: RefInput::Origin(OriginEntity::Point),
        },
        Point3::ZERO,
    ));
    run_points(cases, "at vertex", 5);
}

#[test]
fn point_through_two_edges() {
    let two = |a, b| PointDef::ThroughTwoEdges { a, b };
    let line = |p: Point3, d: Vec3| {
        RefGeom::Line(Line3 {
            point: p,
            dir: dir(d),
        })
    };
    let mut cases = vec![pt_ok(
        "X axis × Y axis",
        Env::default(),
        two(X_AXIS, Y_AXIS),
        Point3::ZERO,
    )];
    let (env, a, b) = pair(
        seg(p3(0.0, -1.0, 2.0), p3(0.0, 1.0, 2.0)),
        line(p3(-3.0, 0.0, 2.0), Vec3::X),
    );
    cases.push(pt_ok(
        "edge × line at z = 2",
        env,
        two(a, b),
        p3(0.0, 0.0, 2.0),
    ));
    let x = p3(1.0, 2.0, 3.0);
    let (env, a, b) = pair(
        line(x - Vec3::new(1.0, 2.0, 2.0), Vec3::new(1.0, 2.0, 2.0)),
        line(
            x + Vec3::new(2.0, -1.0, 4.0) * 3.0,
            Vec3::new(2.0, -1.0, 4.0),
        ),
    );
    cases.push(pt_ok(
        "oblique lines meeting at (1, 2, 3)",
        env,
        two(a, b),
        x,
    ));
    let mut env = Env::default();
    let b = env.add(seg(p3(5.0, -1.0, 0.0), p3(5.0, 1.0, 0.0)));
    cases.push(pt_ok(
        "edges extend as lines",
        env,
        two(X_AXIS, b),
        p3(5.0, 0.0, 0.0),
    ));
    let mut env = Env::default();
    let b = env.add(line(p3(0.0, 0.0, 5e-7), Vec3::Y));
    cases.push(pt_ok(
        "a gap within tolerance: the midpoint",
        env,
        two(X_AXIS, b),
        p3(0.0, 0.0, 2.5e-7),
    ));
    let mut env = Env::default();
    let b = env.add(RefGeom::Curve(Arc::new(
        SegmentCurve::new(&Segment3 {
            a: p3(2.0, 3.0, 0.0),
            b: p3(2.0, 3.0, 4.0),
        })
        .unwrap(),
    )));
    let a = env.add(line(p3(0.0, 3.0, 1.0), Vec3::X));
    cases.push(pt_ok(
        "straight free-form curve",
        env,
        two(a, b),
        p3(2.0, 3.0, 1.0),
    ));
    // Errors.
    let mut env = Env::default();
    let b = env.add(line(p3(0.0, 1.0, 0.0), Vec3::NEG_X));
    cases.push(pt_err(
        "parallel lines",
        env,
        two(X_AXIS, b),
        ConstructError::Parallel,
    ));
    let mut env = Env::default();
    let b = env.add(seg(p3(3.0, 0.0, 0.0), p3(1.0, 0.0, 0.0)));
    cases.push(pt_err(
        "the same line",
        env,
        two(X_AXIS, b),
        ConstructError::Coincident,
    ));
    let mut env = Env::default();
    let b = env.add(line(p3(0.0, 0.0, 0.01), Vec3::Y));
    cases.push(pt_err(
        "skew lines 0.01 mm apart",
        env,
        two(X_AXIS, b),
        ConstructError::Skew { distance: 0.01 },
    ));
    let mut env = Env::default();
    let b = env.add(circle(Point3::ZERO, Dir3::Z, 1.0, None));
    cases.push(pt_err(
        "a circle",
        env,
        two(X_AXIS, b),
        ConstructError::NotStraight,
    ));
    let mut env = Env::default();
    let b = env.point(1.0, 0.0, 0.0);
    cases.push(pt_err(
        "a point",
        env,
        two(X_AXIS, b),
        ConstructError::WrongInput {
            expected: "line or straight edge",
            got: "point",
        },
    ));
    run_points(cases, "through two edges", 5);
}

#[test]
fn skew_lines_report_the_gap_to_1e_12() {
    let tol = Tolerance::DEFAULT;
    let mut rng = Rng(7);
    for i in 0..200 {
        // Two random directions, offset along their common normal by exactly 0.01.
        let da = rng.unit();
        let db = rng.unit();
        let n = da.cross(db);
        if n.length() < 0.1 {
            continue;
        }
        let n = n.normalize();
        let base = rng.vec(50.0);
        let a = Line3 {
            point: base + da * rng.range(-20.0, 20.0),
            dir: dir(da),
        };
        let b = Line3 {
            point: base + n * 0.01 + db * rng.range(-20.0, 20.0),
            dir: dir(db),
        };
        let got = math::point_two_lines(&a, &b, &tol);
        let Err(ConstructError::Skew { distance }) = got else {
            panic!("case {i}: {got:?}")
        };
        assert!((distance - 0.01).abs() <= EPS, "case {i}: {distance}");
    }
    // The Done-when example: oblique skew lines with a 0.01 mm gap.
    let got = math::point_two_lines(
        &Line3 {
            point: Point3::ZERO,
            dir: Dir3::X,
        },
        &Line3 {
            point: p3(0.0, 0.0, 0.01),
            dir: dir(Vec3::new(1.0, 1.0, 0.0)),
        },
        &tol,
    );
    let Err(ConstructError::Skew { distance }) = got else {
        panic!("{got:?}")
    };
    assert!((distance - 0.01).abs() <= EPS, "{distance}");
}

#[test]
fn point_through_three_planes() {
    let three = |a, b, c| PointDef::ThroughThreePlanes { a, b, c };
    let at = |o: Point3, n: Vec3| Plane::from_normal(o, dir(n));
    let mut cases = vec![pt_ok(
        "the three origin planes",
        Env::default(),
        three(XY, YZ, XZ),
        Point3::ZERO,
    )];
    let mut env = Env::default();
    let a = env.plane(plane_z(5.0));
    let b = env.plane(plane_offset(&Plane::YZ, 2.0));
    let c = env.plane(plane_offset(&Plane::XZ, 3.0));
    cases.push(pt_ok(
        "offset planes",
        env,
        three(a, b, c),
        p3(2.0, -3.0, 5.0),
    ));
    let x = p3(1.0, 2.0, 3.0);
    let mut env = Env::default();
    let a = env.plane(at(x + Vec3::Y * 4.0, Vec3::X));
    let b = env.plane(at(x + Vec3::new(1.0, -1.0, 7.0), Vec3::new(1.0, 1.0, 0.0)));
    let c = env.plane(at(x + Vec3::new(1.0, -2.0, 1.0), Vec3::ONE));
    cases.push(pt_ok(
        "oblique planes through (1, 2, 3)",
        env,
        three(a, b, c),
        x,
    ));
    let mut env = Env::default();
    let a = env.plane(plane_z(5.0).flipped());
    let b = env.plane(Plane::YZ.flipped());
    cases.push(pt_ok(
        "flipped normals do not matter",
        env,
        three(a, b, XZ),
        p3(0.0, 0.0, 5.0),
    ));
    let mut env = Env::default();
    let a = env.add(circle(p3(0.0, 0.0, -2.0), Dir3::Z, 1.0, None));
    cases.push(pt_ok(
        "plane of a circular edge",
        env,
        three(a, YZ, XZ),
        p3(0.0, 0.0, -2.0),
    ));
    // Errors.
    let degenerate = ConstructError::Degenerate("the planes do not meet in a single point");
    let mut env = Env::default();
    let a = env.plane(plane_z(5.0));
    cases.push(pt_err(
        "two parallel planes",
        env,
        three(XY, a, YZ),
        degenerate.clone(),
    ));
    let mut env = Env::default();
    let a = env.plane(at(Point3::ZERO, Vec3::new(0.0, 1.0, 1.0)));
    cases.push(pt_err(
        "three planes sharing the X axis",
        env,
        three(XY, XZ, a),
        degenerate.clone(),
    ));
    let mut env = Env::default();
    let a = env.plane(plane_z(1.0));
    let b = env.plane(plane_z(2.0));
    cases.push(pt_err(
        "three parallel planes",
        env,
        three(XY, a, b),
        degenerate,
    ));
    let mut env = Env::default();
    let a = env.point(0.0, 0.0, 0.0);
    cases.push(pt_err(
        "a point",
        env,
        three(XY, YZ, a),
        ConstructError::WrongInput {
            expected: "plane",
            got: "point",
        },
    ));
    run_points(cases, "through three planes", 5);
}

/// Deterministic SplitMix64 for the randomised tests.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform in [lo, hi).
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        // 52 random mantissa bits in [1, 2), shifted to [0, 1).
        let u = f64::from_bits((self.next() >> 12) | 0x3FF0_0000_0000_0000) - 1.0;
        lo + (hi - lo) * u
    }
    fn vec(&mut self, r: f64) -> Vec3 {
        Vec3::new(self.range(-r, r), self.range(-r, r), self.range(-r, r))
    }
    fn unit(&mut self) -> Vec3 {
        loop {
            let v = self.vec(1.0);
            let l = v.length();
            if l > 0.1 && l <= 1.0 {
                return v / l;
            }
        }
    }
}

/// `M x = r` by Gaussian elimination with partial pivoting (independent reference solve).
fn solve3(mut m: [[f64; 3]; 3], mut r: [f64; 3]) -> [f64; 3] {
    for col in 0..3 {
        let piv = (col..3)
            .max_by(|&i, &j| m[i][col].abs().total_cmp(&m[j][col].abs()))
            .unwrap();
        m.swap(col, piv);
        r.swap(col, piv);
        for row in col + 1..3 {
            let f = m[row][col] / m[col][col];
            let pivot_row = m[col];
            for (a, b) in m[row].iter_mut().zip(pivot_row).skip(col) {
                *a -= f * b;
            }
            r[row] -= f * r[col];
        }
    }
    let mut x = [0.0; 3];
    for row in (0..3).rev() {
        let s: f64 = (row + 1..3).map(|k| m[row][k] * x[k]).sum();
        x[row] = (r[row] - s) / m[row][row];
    }
    x
}

#[test]
fn three_planes_match_a_3x3_solve_on_random_triples() {
    let tol = Tolerance::DEFAULT;
    let mut rng = Rng(42);
    let mut checked = 0;
    let mut worst = 0.0f64;
    while checked < 1000 {
        let planes: [Plane; 3] = std::array::from_fn(|_| {
            Plane::from_normal(Point3::from(rng.vec(100.0)), dir(rng.unit()))
        });
        let [a, b, c] = &planes;
        let (n1, n2, n3) = (a.normal.get(), b.normal.get(), c.normal.get());
        // Non-degenerate: the normals span space well.
        if n1.dot(n2.cross(n3)).abs() < 0.05 {
            continue;
        }
        let m = [n1.to_array(), n2.to_array(), n3.to_array()];
        let r = [n1.dot(a.origin), n2.dot(b.origin), n3.dot(c.origin)];
        let want = Vec3::from(solve3(m, r));
        let got = math::point_three_planes(a, b, c, &tol).unwrap();
        let again = math::point_three_planes(a, b, c, &tol).unwrap();
        assert_eq!(
            got.to_array().map(f64::to_bits),
            again.to_array().map(f64::to_bits)
        );
        let e = (got - want).abs().max_element();
        worst = worst.max(e);
        assert!(e <= 1e-9, "triple {checked}: {got:?} vs {want:?} ({e})");
        // On all three planes.
        for p in &planes {
            assert!(p.signed_distance(got).abs() <= 1e-9);
        }
        checked += 1;
    }
    eprintln!("three planes: worst deviation from the 3×3 solve over 1000 triples = {worst:e} mm");
}

#[test]
fn point_at_center() {
    let case = |name, g: RefGeom, w| {
        let (env, entity) = single(g);
        PointCase {
            name,
            env,
            def: PointDef::AtCenter { entity },
            want: w,
        }
    };
    let cases = vec![
        case(
            "circle",
            circle(p3(1.0, 2.0, 3.0), Dir3::Z, 4.0, None),
            Ok(p3(1.0, 2.0, 3.0)),
        ),
        case(
            "arc",
            circle(p3(-1.0, 0.0, 5.0), Dir3::X, 0.5, Some((0.0, 1.0))),
            Ok(p3(-1.0, 0.0, 5.0)),
        ),
        case(
            "sphere",
            RefGeom::Sphere {
                center: p3(0.0, 7.0, 0.0),
                radius: 2.0,
            },
            Ok(p3(0.0, 7.0, 0.0)),
        ),
        case(
            "torus",
            RefGeom::Torus {
                axis: Line3 {
                    point: p3(3.0, 3.0, 3.0),
                    dir: Dir3::Y,
                },
                major: 4.0,
                minor: 1.0,
            },
            Ok(p3(3.0, 3.0, 3.0)),
        ),
        case(
            "circle at the origin",
            circle(Point3::ZERO, dir(Vec3::ONE), 1.0, None),
            Ok(Point3::ZERO),
        ),
        // Errors.
        case("cylinder", cyl2(), Err(ConstructError::Unsupported)),
        case(
            "plane",
            RefGeom::Plane(Plane::XY),
            Err(ConstructError::Unsupported),
        ),
        case(
            "free-form face",
            test_face(false, false),
            Err(ConstructError::Unsupported),
        ),
        case(
            "a point",
            RefGeom::Point(Point3::ZERO),
            Err(ConstructError::WrongInput {
                expected: "circle, arc, sphere or torus",
                got: "point",
            }),
        ),
        case(
            "a straight edge",
            seg(Point3::ZERO, p3(1.0, 0.0, 0.0)),
            Err(ConstructError::WrongInput {
                expected: "circle, arc, sphere or torus",
                got: "straight edge",
            }),
        ),
    ];
    run_points(cases, "at center", 5);
}

/// The point at angle `t` of the circle `(center, normal, r)` in the `RefGeom` frame.
fn circle_at(center: Point3, normal: Dir3, r: f64, t: f64) -> Point3 {
    let f = Plane::from_normal(center, normal);
    center + (f.x.get() * t.cos() + f.y.get() * t.sin()) * r
}

/// Arc parameter of `p` on the circle `(center, normal)`, in [0, 2π).
fn circle_param(center: Point3, normal: Dir3, p: Point3) -> f64 {
    let uv = Plane::from_normal(center, normal).to_local(p);
    uv.y.atan2(uv.x).rem_euclid(TAU)
}

/// Of two candidate points on a circle, the one with the smaller arc parameter.
fn first_on_circle(center: Point3, normal: Dir3, a: Point3, b: Point3) -> Point3 {
    if circle_param(center, normal, a) <= circle_param(center, normal, b) {
        a
    } else {
        b
    }
}

#[test]
fn point_at_edge_and_plane() {
    let ep = |edge, plane| PointDef::EdgeAndPlane { edge, plane };
    let mut cases = Vec::new();
    let mut env = Env::default();
    let p = env.plane(plane_z(5.0));
    cases.push(pt_ok(
        "Z axis × z = 5",
        env,
        ep(Z_AXIS, p),
        p3(0.0, 0.0, 5.0),
    ));
    let mut env = Env::default();
    let e = env.add(seg(p3(1.0, 1.0, -1.0), p3(1.0, 1.0, 1.0)));
    cases.push(pt_ok("edge × XY", env, ep(e, XY), p3(1.0, 1.0, 0.0)));
    let mut env = Env::default();
    let e = env.add(seg(p3(1.0, 1.0, 0.0), p3(1.0, 1.0, 1.0)));
    cases.push(pt_ok(
        "edge ending on XY",
        env,
        ep(e, XY),
        p3(1.0, 1.0, 0.0),
    ));
    let mut env = Env::default();
    let e = env.line(p3(0.0, 0.0, 4.0), Vec3::new(1.0, 1.0, -2.0));
    cases.push(pt_ok(
        "oblique line × XY",
        env,
        ep(e, XY),
        p3(2.0, 2.0, 0.0),
    ));
    // Circle r = 2 in the XZ plane crosses XY at (±2, 0, 0): the first by arc parameter.
    let o = Point3::ZERO;
    let first = first_on_circle(o, Dir3::Y, p3(2.0, 0.0, 0.0), p3(-2.0, 0.0, 0.0));
    let mut env = Env::default();
    let e = env.add(circle(o, Dir3::Y, 2.0, None));
    cases.push(pt_ok(
        "circle × XY: smallest parameter",
        env,
        ep(e, XY),
        first,
    ));
    // The half of that circle starting at the other crossing contains it at its start.
    let other = if first.x > 0.0 {
        p3(-2.0, 0.0, 0.0)
    } else {
        p3(2.0, 0.0, 0.0)
    };
    let t_other = circle_param(o, Dir3::Y, other);
    let mut env = Env::default();
    let e = env.add(circle(o, Dir3::Y, 2.0, Some((t_other, t_other + 0.5 * PI))));
    cases.push(pt_ok("arc starting on the plane", env, ep(e, XY), other));
    // Circle r = 2 on XY tangent to x = 2.
    let mut env = Env::default();
    let e = env.add(circle(o, Dir3::Z, 2.0, None));
    let p = env.plane(plane_offset(&Plane::YZ, 2.0));
    cases.push(pt_ok(
        "circle tangent to the plane",
        env,
        ep(e, p),
        p3(2.0, 0.0, 0.0),
    ));
    // Free-form circle r = 5 on XY × x = 3: (3, ±4, 0), first by parameter.
    let first = first_on_circle(o, Dir3::Z, p3(3.0, 4.0, 0.0), p3(3.0, -4.0, 0.0));
    let mut env = Env::default();
    let e = env.add(RefGeom::Curve(Arc::new(
        CircleCurve::new(o, Dir3::Z, 5.0, None).unwrap(),
    )));
    let p = env.plane(plane_offset(&Plane::YZ, 3.0));
    cases.push(pt_ok("free-form circle × x = 3", env, ep(e, p), first));
    let mut env = Env::default();
    let e = env.add(RefGeom::Curve(Arc::new(
        SegmentCurve::new(&Segment3 {
            a: p3(0.0, 0.0, -3.0),
            b: p3(6.0, 0.0, 3.0),
        })
        .unwrap(),
    )));
    cases.push(pt_ok(
        "straight free-form curve × XY",
        env,
        ep(e, XY),
        p3(3.0, 0.0, 0.0),
    ));
    // Errors.
    let mut env = Env::default();
    let p = env.plane(plane_z(5.0));
    cases.push(pt_err(
        "line parallel to the plane",
        env,
        ep(X_AXIS, p),
        ConstructError::NoIntersection,
    ));
    cases.push(pt_err(
        "line in the plane",
        Env::default(),
        ep(X_AXIS, XY),
        ConstructError::Coincident,
    ));
    let mut env = Env::default();
    let e = env.add(seg(p3(0.0, 0.0, 1.0), p3(0.0, 0.0, 2.0)));
    cases.push(pt_err(
        "edge short of the plane",
        env,
        ep(e, XY),
        ConstructError::NoIntersection,
    ));
    let mut env = Env::default();
    let e = env.add(circle(p3(0.0, 0.0, 3.0), Dir3::Y, 2.0, None));
    cases.push(pt_err(
        "circle above the plane",
        env,
        ep(e, XY),
        ConstructError::NoIntersection,
    ));
    let mut env = Env::default();
    let e = env.add(circle(o, Dir3::Z, 2.0, None));
    cases.push(pt_err(
        "circle in the plane",
        env,
        ep(e, XY),
        ConstructError::Coincident,
    ));
    let t_first = circle_param(
        o,
        Dir3::Y,
        first_on_circle(o, Dir3::Y, p3(2.0, 0.0, 0.0), p3(-2.0, 0.0, 0.0)),
    );
    let mut env = Env::default();
    let e = env.add(circle(
        o,
        Dir3::Y,
        2.0,
        Some((t_first + 0.1, t_first + PI - 0.1)),
    ));
    cases.push(pt_err(
        "arc between the crossings",
        env,
        ep(e, XY),
        ConstructError::NoIntersection,
    ));
    let mut env = Env::default();
    let e = env.add(RefGeom::Curve(Arc::new(
        CircleCurve::new(o, Dir3::Z, 5.0, None).unwrap(),
    )));
    let p = env.plane(plane_offset(&Plane::YZ, 6.0));
    cases.push(pt_err(
        "free-form curve missing the plane",
        env,
        ep(e, p),
        ConstructError::NoIntersection,
    ));
    cases.push(pt_err(
        "a plane as the edge",
        Env::default(),
        ep(YZ, XY),
        ConstructError::WrongInput {
            expected: "edge, sketch curve or axis",
            got: "plane",
        },
    ));
    let mut env = Env::default();
    let p = env.point(0.0, 0.0, 0.0);
    cases.push(pt_err(
        "a point as the plane",
        env,
        ep(Z_AXIS, p),
        ConstructError::WrongInput {
            expected: "plane",
            got: "point",
        },
    ));
    run_points(cases, "edge and plane", 5);

    // Every analytic circle crossing lies on both the circle and the plane.
    let mut rng = Rng(3);
    for i in 0..200 {
        let (c, n, r) = (
            Point3::from(rng.vec(10.0)),
            dir(rng.unit()),
            rng.range(0.5, 5.0),
        );
        let pl = Plane::from_normal(c + rng.unit() * rng.range(0.0, 0.9 * r), dir(rng.unit()));
        let g = circle(c, n, r, None);
        let Ok(x) = math::point_curve_plane(&g, &pl, &Tolerance::DEFAULT) else {
            continue;
        };
        assert!(pl.signed_distance(x).abs() <= 1e-9, "case {i}");
        assert!(((x - c).length() - r).abs() <= 1e-9, "case {i}");
        assert!((x - c).dot(n.get()).abs() <= 1e-9, "case {i}");
    }
}

#[test]
fn point_along_path() {
    let frac = |path, v: f64| PointDef::AlongPath {
        path,
        at: PathPosition::Fraction(num(v)),
    };
    let dist = |path, v: f64| PointDef::AlongPath {
        path,
        at: PathPosition::Distance(num(v)),
    };
    let edge = || seg(Point3::ZERO, p3(10.0, 0.0, 0.0));
    let o = Point3::ZERO;
    let mut cases = Vec::new();
    let (env, p) = single(edge());
    cases.push(pt_ok("edge at half", env, frac(p, 0.5), p3(5.0, 0.0, 0.0)));
    let (env, p) = single(edge());
    cases.push(pt_ok(
        "edge at 2.5 mm",
        env,
        dist(p, 2.5),
        p3(2.5, 0.0, 0.0),
    ));
    let (env, p) = single(edge());
    cases.push(pt_ok("edge start", env, frac(p, 0.0), Point3::ZERO));
    let (env, p) = single(circle(o, Dir3::Z, 5.0, None));
    cases.push(pt_ok(
        "circle at a quarter",
        env,
        frac(p, 0.25),
        circle_at(o, Dir3::Z, 5.0, FRAC_PI_2),
    ));
    let (env, p) = single(circle(o, Dir3::Z, 2.0, Some((0.0, PI))));
    cases.push(pt_ok(
        "half arc r = 2 at arc length π",
        env,
        dist(p, PI),
        circle_at(o, Dir3::Z, 2.0, FRAC_PI_2),
    ));
    let (env, p) = single(RefGeom::Curve(Arc::new(
        CircleCurve::new(p3(0.0, 0.0, 3.0), Dir3::Z, 4.0, None).unwrap(),
    )));
    cases.push(pt_ok(
        "free-form circle at its end",
        env,
        frac(p, 1.0),
        circle_at(p3(0.0, 0.0, 3.0), Dir3::Z, 4.0, TAU),
    ));
    // Errors.
    let (env, p) = single(edge());
    cases.push(pt_err(
        "fraction above 1",
        env,
        frac(p, 1.5),
        ConstructError::OutOfRange,
    ));
    let (env, p) = single(edge());
    cases.push(pt_err(
        "negative fraction",
        env,
        frac(p, -0.1),
        ConstructError::OutOfRange,
    ));
    let (env, p) = single(edge());
    cases.push(pt_err(
        "distance past the end",
        env,
        dist(p, 10.5),
        ConstructError::OutOfRange,
    ));
    cases.push(pt_err(
        "an infinite axis is not a path",
        Env::default(),
        frac(X_AXIS, 0.5),
        ConstructError::WrongInput {
            expected: "edge or sketch curve",
            got: "line",
        },
    ));
    run_points(cases, "along path", 5);
}

// ---- framework -----------------------------------------------------------------------

#[test]
fn axis_and_point_inputs_and_exprs() {
    let a = EntityRef::Construction(ConstructionId(FeatureId(5)));
    let b = EntityRef::Construction(ConstructionId(FeatureId(6)));
    let axis = ConstructAxis {
        def: AxisDef::ThroughTwoPlanes {
            a: RefInput::Ref(a.clone()),
            b: XY,
        },
    };
    assert_eq!(
        axis.inputs(),
        vec![Dependency::Construction(ConstructionId(FeatureId(5)))]
    );
    assert!(axis.exprs().is_empty());
    let point = ConstructPoint {
        def: PointDef::ThroughThreePlanes {
            a: RefInput::Ref(a.clone()),
            b: RefInput::Ref(b),
            c: YZ,
        },
    };
    assert_eq!(point.inputs().len(), 2);
    assert!(point.exprs().is_empty());
    let mut along = ConstructPoint {
        def: PointDef::AlongPath {
            path: RefInput::Ref(a),
            at: PathPosition::Distance(num(2.0)),
        },
    };
    assert_eq!(along.exprs().len(), 1);
    let dims: Vec<_> = along.exprs_mut().into_iter().map(|(_, d)| d).collect();
    assert_eq!(dims, vec![amg_core::Dimension::LENGTH]);
    for def in [
        AxisDef::ThroughRevolved { face_or_edge: XY },
        AxisDef::PerpendicularAtPoint {
            face: XY,
            point: XY,
        },
        AxisDef::ThroughTwoPlanes { a: XY, b: YZ },
        AxisDef::ThroughTwoPoints { a: XY, b: YZ },
        AxisDef::ThroughEdge { edge: XY },
        AxisDef::PerpendicularToFaceAtPoint {
            face: XY,
            point: XY,
        },
    ] {
        assert!(!def.refs().is_empty());
        assert!(
            ConstructAxis { def }.inputs().is_empty(),
            "origin inputs need no dependency"
        );
    }
}

#[test]
fn a_fingerprint_only_axis_or_point_input_is_a_warning() {
    let mut env = Env::default();
    let p = env.add_resolved(ResolvedRef {
        geom: RefGeom::Plane(plane_z(2.0)),
        context_plane: None,
        quality: Some(MatchQuality::FingerprintOnly),
    });
    let (axis, warnings) = AxisDef::ThroughTwoPlanes {
        a: p.clone(),
        b: YZ,
    }
    .evaluate(&env)
    .unwrap();
    assert!(close(axis.point, p3(0.0, 0.0, 2.0)));
    assert_eq!(warnings.len(), 1);
    assert_eq!(warnings[0].entities, vec![p.entity().unwrap().clone()]);
    let (point, warnings) = PointDef::EdgeAndPlane {
        edge: Z_AXIS,
        plane: p,
    }
    .evaluate(&env)
    .unwrap();
    assert!(close(point, p3(0.0, 0.0, 2.0)));
    assert_eq!(warnings.len(), 1);
}
