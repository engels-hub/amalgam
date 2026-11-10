//! CON-01 unit tests: every plane variant on analytic inputs through a test resolver
//! (pre-resolved `RefGeom`), plus every error the plane constructions can raise (note §6).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    reason = "tests"
)]

use std::collections::HashMap;
use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4, PI};
use std::sync::Arc;

use amg_core::{Dir3, Line3, Plane, Point2, Point3, Segment3, Tolerance, Transform, Vec3};
use amg_doc::{
    ConstructionId, Dependency, EntityRef, Expr, ExprSrc, Feature, FeatureError, FeatureId,
    MatchQuality,
};

use super::curves::{CircleCurve, SegmentCurve};
use super::*;

const EPS: f64 = 1e-12;

// ---- test resolver -------------------------------------------------------------------

/// Pre-resolved inputs: each `add` registers geometry under a fresh construction reference.
/// Expressions are plain numbers in canonical units (mm, rad, dimensionless).
#[derive(Default)]
struct Env {
    refs: HashMap<RefInput, ResolvedRef>,
    next: u64,
}

impl Env {
    fn add_resolved(&mut self, r: ResolvedRef) -> RefInput {
        self.next += 1;
        let key = RefInput::Ref(EntityRef::Construction(ConstructionId(FeatureId(
            1000 + self.next,
        ))));
        self.refs.insert(key.clone(), r);
        key
    }
    fn add(&mut self, g: RefGeom) -> RefInput {
        self.add_resolved(ResolvedRef::new(g))
    }
    fn plane(&mut self, p: Plane) -> RefInput {
        self.add(RefGeom::Plane(p))
    }
    fn point(&mut self, x: f64, y: f64, z: f64) -> RefInput {
        self.add(RefGeom::Point(Point3::new(x, y, z)))
    }
    fn line(&mut self, p: Point3, d: Vec3) -> RefInput {
        self.add(RefGeom::Line(Line3 {
            point: p,
            dir: dir(d),
        }))
    }
}

fn num(v: f64) -> Expr {
    Expr::Inline(ExprSrc(format!("{v:?}")))
}

fn value(e: &Expr) -> Result<f64, FeatureError> {
    match e {
        Expr::Inline(s) => Ok(s.0.parse().unwrap()),
        Expr::Param(_) => unreachable!(),
    }
}

impl ConstructInputs for Env {
    fn resolve(&self, r: &RefInput) -> Result<ResolvedRef, FeatureError> {
        match r {
            RefInput::Origin(e) => Ok(ResolvedRef::new(math::origin_geom(
                *e,
                &Transform::IDENTITY,
            ))),
            other => Ok(self.refs.get(other).cloned().expect("registered input")),
        }
    }
    fn length(&self, e: &Expr) -> Result<f64, FeatureError> {
        value(e)
    }
    fn angle(&self, e: &Expr) -> Result<f64, FeatureError> {
        value(e)
    }
    fn scalar(&self, e: &Expr) -> Result<f64, FeatureError> {
        value(e)
    }
    fn tolerance(&self) -> Tolerance {
        Tolerance::DEFAULT
    }
}

fn dir(v: Vec3) -> Dir3 {
    Dir3::new(v, &Tolerance::DEFAULT).unwrap()
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

const XY: RefInput = RefInput::Origin(OriginEntity::XYPlane);
const YZ: RefInput = RefInput::Origin(OriginEntity::YZPlane);
const XZ: RefInput = RefInput::Origin(OriginEntity::XZPlane);
const X_AXIS: RefInput = RefInput::Origin(OriginEntity::XAxis);
const Y_AXIS: RefInput = RefInput::Origin(OriginEntity::YAxis);
const Z_AXIS: RefInput = RefInput::Origin(OriginEntity::ZAxis);

// ---- expectations --------------------------------------------------------------------

/// What a successful case must produce.
#[derive(Clone, Copy, Debug)]
struct Want {
    normal: Vec3,
    /// A point the plane must contain.
    through: Point3,
    /// Exact origin, when the convention fixes it.
    origin: Option<Point3>,
    /// Exact in-plane x axis, when the convention fixes it.
    x: Option<Vec3>,
}

fn want(normal: Vec3, through: Point3) -> Want {
    Want {
        normal,
        through,
        origin: None,
        x: None,
    }
}

impl Want {
    fn at(mut self, o: Point3) -> Want {
        self.origin = Some(o);
        self
    }
    fn x(mut self, x: Vec3) -> Want {
        self.x = Some(x);
        self
    }
}

enum Outcome {
    Ok(Want),
    Err(ConstructError),
}

struct Case {
    name: &'static str,
    env: Env,
    def: PlaneDef,
    outcome: Outcome,
}

fn ok(name: &'static str, env: Env, def: PlaneDef, w: Want) -> Case {
    Case {
        name,
        env,
        def,
        outcome: Outcome::Ok(w),
    }
}

fn err(name: &'static str, env: Env, def: PlaneDef, e: ConstructError) -> Case {
    Case {
        name,
        env,
        def,
        outcome: Outcome::Err(e),
    }
}

fn close(a: Vec3, b: Vec3) -> bool {
    (a - b).abs().max_element() <= EPS
}

fn bits(p: &Plane) -> [u64; 12] {
    let v = [p.origin, p.x.get(), p.y.get(), p.normal.get()];
    let mut out = [0; 12];
    for (i, c) in v.iter().flat_map(|v| v.to_array()).enumerate() {
        out[i] = c.to_bits();
    }
    out
}

fn check_frame(name: &str, p: &Plane) {
    let (x, y, n) = (p.x.get(), p.y.get(), p.normal.get());
    for (what, v) in [("x", x), ("y", y), ("normal", n)] {
        assert!(
            (v.length() - 1.0).abs() <= EPS,
            "{name}: |{what}| = {}",
            v.length()
        );
    }
    assert!(x.dot(n).abs() <= EPS, "{name}: x·n = {}", x.dot(n));
    assert!(close(x.cross(y), n), "{name}: frame not right-handed");
}

/// Runs a table: each case twice (bit-identical), checking the frame and the expectation.
fn run(cases: Vec<Case>, variant: &str, min_ok: usize) {
    let n_ok = cases
        .iter()
        .filter(|c| matches!(c.outcome, Outcome::Ok(_)))
        .count();
    assert!(n_ok >= min_ok, "{variant}: only {n_ok} analytic cases");
    for c in cases {
        let got = c.def.evaluate(&c.env);
        match c.outcome {
            Outcome::Ok(w) => {
                let (p, warnings) = got.unwrap_or_else(|e| panic!("{}: {e}", c.name));
                assert!(warnings.is_empty(), "{}", c.name);
                check_frame(c.name, &p);
                assert!(
                    close(p.normal.get(), w.normal),
                    "{}: normal {:?}, want {:?}",
                    c.name,
                    p.normal.get(),
                    w.normal
                );
                let d = p.signed_distance(w.through);
                assert!(d.abs() <= EPS, "{}: {:?} is {d} off", c.name, w.through);
                if let Some(o) = w.origin {
                    assert!(close(p.origin, o), "{}: origin {:?}", c.name, p.origin);
                }
                if let Some(x) = w.x {
                    assert!(close(p.x.get(), x), "{}: x {:?}", c.name, p.x.get());
                }
                let (again, _) = c.def.evaluate(&c.env).unwrap();
                assert_eq!(bits(&p), bits(&again), "{}: not bit-identical", c.name);
            }
            Outcome::Err(e) => {
                assert_eq!(got.map(|_| ()), Err(FeatureError::from(e)), "{}", c.name);
            }
        }
    }
}

fn offset(base: RefInput, d: f64) -> PlaneDef {
    PlaneDef::Offset {
        base,
        distance: num(d),
    }
}

fn at_angle(line: RefInput, zero: Option<RefInput>, deg: f64) -> PlaneDef {
    PlaneDef::AtAngle {
        line,
        zero,
        angle: num(deg.to_radians()),
    }
}

fn tangent(face: RefInput, reference: RefInput, deg: f64) -> PlaneDef {
    PlaneDef::Tangent {
        face,
        reference,
        angle: num(deg.to_radians()),
    }
}

fn single(g: RefGeom) -> (Env, RefInput) {
    let mut env = Env::default();
    let r = env.add(g);
    (env, r)
}

fn plane_z(z: f64) -> Plane {
    plane_offset(&Plane::XY, z)
}

use super::math::plane_offset;

// ---- the variants --------------------------------------------------------------------

#[test]
fn offset_plane() {
    let mut cases = vec![
        ok(
            "XY by 5 → z = 5",
            Env::default(),
            offset(XY, 5.0),
            want(Vec3::Z, p3(0.0, 0.0, 5.0))
                .at(p3(0.0, 0.0, 5.0))
                .x(Vec3::X),
        ),
        ok(
            "XY by −2.5",
            Env::default(),
            offset(XY, -2.5),
            want(Vec3::Z, p3(0.0, 0.0, -2.5)).at(p3(0.0, 0.0, -2.5)),
        ),
        ok(
            "YZ by 3",
            Env::default(),
            offset(YZ, 3.0),
            want(Vec3::X, p3(3.0, 0.0, 0.0)).x(Vec3::Y),
        ),
        ok(
            "XZ by 4 goes along −Y",
            Env::default(),
            offset(XZ, 4.0),
            want(Vec3::NEG_Y, p3(0.0, -4.0, 0.0)),
        ),
    ];
    let mut env = Env::default();
    let n = Vec3::ONE.normalize();
    let base = env.plane(Plane::from_normal(p3(1.0, 2.0, 3.0), dir(n)));
    cases.push(ok(
        "oblique plane by √3",
        env,
        offset(base, 3f64.sqrt()),
        want(n, p3(2.0, 3.0, 4.0)),
    ));
    let (env, circle) = single(RefGeom::Circle {
        center: p3(0.0, 0.0, 1.0),
        normal: Dir3::Z,
        radius: 2.0,
        arc: None,
    });
    cases.push(ok(
        "plane of a circular edge by 1",
        env,
        offset(circle, 1.0),
        want(Vec3::Z, p3(0.0, 0.0, 2.0)).at(p3(0.0, 0.0, 2.0)),
    ));
    let (env, pt) = single(RefGeom::Point(Point3::ZERO));
    cases.push(err(
        "a point is not a plane",
        env,
        offset(pt, 1.0),
        ConstructError::WrongInput {
            expected: "plane",
            got: "point",
        },
    ));
    run(cases, "offset", 5);
}

#[test]
fn plane_at_angle() {
    let (s30, c30) = 30f64.to_radians().sin_cos();
    let mut cases = vec![
        ok(
            "30° about +X from XY (right-hand rule)",
            Env::default(),
            at_angle(X_AXIS, Some(XY), 30.0),
            want(Vec3::new(0.0, -s30, c30), Point3::ZERO)
                .at(Point3::ZERO)
                .x(Vec3::X),
        ),
        ok(
            "90° about +X from XY",
            Env::default(),
            at_angle(X_AXIS, Some(XY), 90.0),
            want(Vec3::NEG_Y, Point3::ZERO),
        ),
        ok(
            "X axis, default zero plane (world Z): X × Z = −Y",
            Env::default(),
            at_angle(X_AXIS, None, 0.0),
            want(Vec3::NEG_Y, Point3::ZERO),
        ),
        ok(
            "90° about +Z from XZ",
            Env::default(),
            at_angle(Z_AXIS, Some(XZ), 90.0),
            want(Vec3::X, Point3::ZERO).x(Vec3::Z),
        ),
        ok(
            "Y axis, default zero (Y × Z = +X), −90°",
            Env::default(),
            at_angle(Y_AXIS, None, -90.0),
            want(Vec3::Z, Point3::ZERO),
        ),
        ok(
            "Z axis, default zero falls back to world Y: Z × Y = −X",
            Env::default(),
            at_angle(Z_AXIS, None, 0.0),
            want(Vec3::NEG_X, Point3::ZERO),
        ),
    ];
    // A sketch line whose context plane is z = 5 is the zero plane when none is picked.
    let mut env = Env::default();
    let (s45, c45) = FRAC_PI_4.sin_cos();
    let line = env.add_resolved(ResolvedRef {
        geom: RefGeom::Segment(Segment3 {
            a: p3(0.0, 0.0, 5.0),
            b: p3(10.0, 0.0, 5.0),
        }),
        context_plane: Some(plane_z(5.0)),
        quality: None,
    });
    cases.push(ok(
        "sketch line, context plane is the zero plane, 45°",
        env,
        at_angle(line, None, 45.0),
        want(Vec3::new(0.0, -s45, c45), p3(3.0, 0.0, 5.0)).at(p3(0.0, 0.0, 5.0)),
    ));
    cases.push(err(
        "the zero plane must contain the line (perpendicular)",
        Env::default(),
        at_angle(Z_AXIS, Some(XY), 10.0),
        ConstructError::Parallel,
    ));
    let mut env = Env::default();
    let above = env.line(p3(0.0, 0.0, 1.0), Vec3::X);
    cases.push(err(
        "the zero plane must contain the line (offset)",
        env,
        at_angle(above, Some(XY), 10.0),
        ConstructError::Parallel,
    ));
    let (env, arc) = single(RefGeom::Circle {
        center: Point3::ZERO,
        normal: Dir3::Z,
        radius: 1.0,
        arc: None,
    });
    cases.push(err(
        "a circular edge is not straight",
        env,
        at_angle(arc, Some(XY), 0.0),
        ConstructError::NotStraight,
    ));
    let (env, pt) = single(RefGeom::Point(Point3::ZERO));
    cases.push(err(
        "a point is not a line",
        env,
        at_angle(pt, Some(XY), 0.0),
        ConstructError::WrongInput {
            expected: "line or straight edge",
            got: "point",
        },
    ));
    run(cases, "at angle", 5);
}

#[test]
fn tangent_plane() {
    let cyl_z = |r: f64| RefGeom::Cylinder {
        axis: Line3 {
            point: Point3::ZERO,
            dir: Dir3::Z,
        },
        radius: r,
    };
    let mut cases = Vec::new();
    // Note §6: u₀ is the reference normal projected ⟂ to the axis. `Plane::XZ` has normal −Y,
    // so the contact is at −Y; with the XZ plane facing +Y it is the Done-when's (0, 3, 0).
    let (env, cyl) = single(cyl_z(3.0));
    cases.push(ok(
        "cylinder r = 3 on Z, 0° from XZ (normal −Y)",
        env,
        tangent(cyl, XZ, 0.0),
        want(Vec3::NEG_Y, p3(0.0, -3.0, 0.0))
            .at(p3(0.0, -3.0, 0.0))
            .x(Vec3::Z),
    ));
    let mut env = Env::default();
    let cyl = env.add(cyl_z(3.0));
    let xz_up = env.plane(Plane::XZ.flipped());
    cases.push(ok(
        "cylinder r = 3 on Z, 0° from XZ facing +Y → normal +Y through (0, 3, 0)",
        env,
        tangent(cyl, xz_up, 0.0),
        want(Vec3::Y, p3(0.0, 3.0, 0.0)).at(p3(0.0, 3.0, 0.0)),
    ));
    let mut env = Env::default();
    let cyl = env.add(cyl_z(3.0));
    let xz_up = env.plane(Plane::XZ.flipped());
    cases.push(ok(
        "cylinder r = 3 on Z, 90° from +Y about +Z → −X",
        env,
        tangent(cyl, xz_up, 90.0),
        want(Vec3::NEG_X, p3(-3.0, 0.0, 0.0)),
    ));
    let (env, cyl) = single(RefGeom::Cylinder {
        axis: Line3 {
            point: Point3::ZERO,
            dir: Dir3::X,
        },
        radius: 2.0,
    });
    cases.push(ok(
        "cylinder r = 2 on X, 0° from XY",
        env,
        tangent(cyl, XY, 0.0),
        want(Vec3::Z, p3(0.0, 0.0, 2.0)).x(Vec3::X),
    ));
    let (env, cyl) = single(RefGeom::Cylinder {
        axis: Line3 {
            point: p3(5.0, 5.0, 0.0),
            dir: Dir3::Z,
        },
        radius: 1.0,
    });
    cases.push(ok(
        "off-origin cylinder, 180° from YZ",
        env,
        tangent(cyl, YZ, 180.0),
        want(Vec3::NEG_X, p3(4.0, 5.0, 0.0)),
    ));
    let mut env = Env::default();
    let cyl = env.add(cyl_z(3.0));
    let oblique = env.plane(Plane::from_normal(
        Point3::ZERO,
        dir(Vec3::new(1.0, 0.0, 1.0)),
    ));
    cases.push(ok(
        "oblique reference projects to +X",
        env,
        tangent(cyl, oblique, 0.0),
        want(Vec3::X, p3(3.0, 0.0, 0.0)),
    ));
    let sphere = || RefGeom::Sphere {
        center: p3(1.0, 2.0, 3.0),
        radius: 2.0,
    };
    let (env, s) = single(sphere());
    cases.push(ok(
        "sphere, 0° from XY",
        env,
        tangent(s, XY, 0.0),
        want(Vec3::Z, p3(1.0, 2.0, 5.0)).x(Vec3::X),
    ));
    let (env, s) = single(sphere());
    cases.push(ok(
        "sphere, 90° about XY's x",
        env,
        tangent(s, XY, 90.0),
        want(Vec3::NEG_Y, p3(1.0, 0.0, 3.0)).at(p3(1.0, 0.0, 3.0)),
    ));
    let cone = || RefGeom::Cone {
        apex: Point3::ZERO,
        axis: Dir3::Z,
        half_angle: FRAC_PI_4,
    };
    let mut env = Env::default();
    let c = env.add(cone());
    let r = env.plane(Plane::from_normal(p3(0.0, 0.0, 2.0), Dir3::Y));
    cases.push(ok(
        "45° cone, contact at the reference height",
        env,
        tangent(c, r, 0.0),
        want(Vec3::new(0.0, FRAC_1_SQRT_2, -FRAC_1_SQRT_2), Point3::ZERO)
            .at(p3(0.0, 2.0, 2.0))
            .x(Vec3::new(0.0, FRAC_1_SQRT_2, FRAC_1_SQRT_2)),
    ));
    let mut env = Env::default();
    let c = env.add(cone());
    let r = env.plane(Plane::from_normal(Point3::ZERO, Dir3::Y));
    cases.push(ok(
        "45° cone, reference at the apex height",
        env,
        tangent(c, r, 0.0),
        want(
            Vec3::new(0.0, FRAC_1_SQRT_2, -FRAC_1_SQRT_2),
            p3(0.0, 1.0, 1.0),
        )
        .at(Point3::ZERO),
    ));
    // Errors.
    let (env, cyl) = single(cyl_z(3.0));
    cases.push(err(
        "reference normal parallel to the axis",
        env,
        tangent(cyl, XY, 0.0),
        ConstructError::Parallel,
    ));
    let (env, torus) = single(RefGeom::Torus {
        axis: Line3 {
            point: Point3::ZERO,
            dir: Dir3::Z,
        },
        major: 10.0,
        minor: 2.0,
    });
    cases.push(err(
        "torus",
        env,
        tangent(torus, XZ, 0.0),
        ConstructError::Unsupported,
    ));
    let (env, pl) = single(RefGeom::Plane(plane_z(1.0)));
    cases.push(err(
        "planar face",
        env,
        tangent(pl, XZ, 0.0),
        ConstructError::Unsupported,
    ));
    let (env, pt) = single(RefGeom::Point(Point3::ZERO));
    cases.push(err(
        "a point is not a face",
        env,
        tangent(pt, XZ, 0.0),
        ConstructError::WrongInput {
            expected: "cylinder, cone or sphere face",
            got: "point",
        },
    ));
    let (env, cyl) = single(cyl_z(0.0));
    cases.push(err(
        "zero radius",
        env,
        tangent(cyl, XZ, 0.0),
        ConstructError::Degenerate("radius is not positive"),
    ));
    run(cases, "tangent", 5);
}

#[test]
fn midplane() {
    let mid = |a, b| PlaneDef::Midplane { a, b };
    let mut cases = Vec::new();
    let mut env = Env::default();
    let top = env.plane(plane_z(10.0));
    cases.push(ok(
        "z = 0 and z = 10 → z = 5",
        env,
        mid(XY, top),
        want(Vec3::Z, p3(0.0, 0.0, 5.0))
            .at(p3(0.0, 0.0, 5.0))
            .x(Vec3::X),
    ));
    let mut env = Env::default();
    let top = env.plane(plane_z(10.0).flipped());
    cases.push(ok(
        "opposite parallel planes keep the first normal",
        env,
        mid(XY, top),
        want(Vec3::Z, p3(0.0, 0.0, 5.0)),
    ));
    let mut env = Env::default();
    let below = env.plane(plane_z(-4.0));
    cases.push(ok(
        "second plane below the first",
        env,
        mid(XY, below),
        want(Vec3::Z, p3(0.0, 0.0, -2.0)),
    ));
    cases.push(ok(
        "XY and YZ (n₁·n₂ = 0) → bisector n₁ + n₂ through the Y axis",
        Env::default(),
        mid(XY, YZ),
        want(
            Vec3::new(FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2),
            p3(0.0, 7.0, 0.0),
        )
        .at(Point3::ZERO)
        .x(Vec3::Y),
    ));
    // Facing planes: z = 0 (up) and x = 0 facing −X: n₁·n₂ = 0, so n₁ + n₂ as well; take an
    // obtuse pair for n₁ − n₂.
    let mut env = Env::default();
    let n2 = Vec3::new(1.0, 0.0, -1.0).normalize();
    let b = env.plane(Plane::from_normal(Point3::ZERO, dir(n2)));
    let expect = (Vec3::Z - n2).normalize();
    cases.push(ok(
        "planes facing each other (n₁·n₂ < 0) → n₁ − n₂",
        env,
        mid(XY, b),
        want(expect, p3(0.0, 3.0, 0.0)),
    ));
    let mut env = Env::default();
    let (a, b) = (env.point(0.0, 0.0, 0.0), env.point(10.0, 0.0, 0.0));
    cases.push(ok(
        "two points on X",
        env,
        mid(a, b),
        want(Vec3::X, p3(5.0, 3.0, -1.0)).at(p3(5.0, 0.0, 0.0)),
    ));
    let mut env = Env::default();
    let (a, b) = (env.point(1.0, 1.0, 1.0), env.point(3.0, 3.0, 3.0));
    cases.push(ok(
        "two points on the diagonal",
        env,
        mid(a, b),
        want(Vec3::ONE.normalize(), p3(2.0, 2.0, 2.0)).at(p3(2.0, 2.0, 2.0)),
    ));
    // Errors.
    cases.push(err(
        "the same plane",
        Env::default(),
        mid(XY, XY),
        ConstructError::Coincident,
    ));
    let mut env = Env::default();
    let flipped = env.plane(Plane::XY.flipped());
    cases.push(err(
        "the same plane facing the other way",
        env,
        mid(XY, flipped),
        ConstructError::Coincident,
    ));
    let mut env = Env::default();
    let (a, b) = (env.point(1.0, 2.0, 3.0), env.point(1.0, 2.0, 3.0));
    cases.push(err(
        "the same point",
        env,
        mid(a, b),
        ConstructError::Coincident,
    ));
    let mut env = Env::default();
    let pt = env.point(0.0, 0.0, 1.0);
    cases.push(err(
        "a plane and a point",
        env,
        mid(XY, pt),
        ConstructError::WrongInput {
            expected: "plane",
            got: "point",
        },
    ));
    cases.push(err(
        "an axis",
        Env::default(),
        mid(X_AXIS, XY),
        ConstructError::WrongInput {
            expected: "two planes or two points",
            got: "line",
        },
    ));
    run(cases, "midplane", 5);

    // The bisector is equidistant from both planes.
    let mut env = Env::default();
    let b = env.plane(Plane::from_normal(p3(1.0, 2.0, 0.5), dir(n2)));
    let (p, _) = mid(XY, b.clone()).evaluate(&env).unwrap();
    let q = env.refs[&b].geom.clone();
    let RefGeom::Plane(q) = q else { panic!() };
    for uv in [Point2::new(3.0, -1.0), Point2::new(-7.0, 2.5)] {
        let w = p.to_world(uv);
        let (da, db) = (Plane::XY.signed_distance(w), q.signed_distance(w));
        assert!((da.abs() - db.abs()).abs() <= EPS, "{da} vs {db}");
    }
}

#[test]
fn plane_through_two_edges() {
    let two = |a, b| PlaneDef::TwoEdges { a, b };
    let mut cases = vec![ok(
        "X and Y axes → +Z",
        Env::default(),
        two(X_AXIS, Y_AXIS),
        want(Vec3::Z, Point3::ZERO).at(Point3::ZERO).x(Vec3::X),
    )];
    let mut env = Env::default();
    let b = env.line(p3(0.0, 5.0, 0.0), Vec3::X);
    cases.push(ok(
        "parallel lines: d × (p₂ − p₁)",
        env,
        two(X_AXIS, b),
        want(Vec3::Z, p3(4.0, 5.0, 0.0)),
    ));
    let mut env = Env::default();
    let a = env.add(RefGeom::Segment(Segment3 {
        a: Point3::ZERO,
        b: p3(1.0, 0.0, 0.0),
    }));
    let b = env.add(RefGeom::Segment(Segment3 {
        a: Point3::ZERO,
        b: p3(0.0, 0.0, 1.0),
    }));
    cases.push(ok(
        "two straight edges: X × Z = −Y",
        env,
        two(a, b),
        want(Vec3::NEG_Y, Point3::ZERO),
    ));
    let mut env = Env::default();
    let a = env.line(p3(1.0, 1.0, 1.0), Vec3::X);
    let b = env.line(p3(1.0, 1.0, 1.0), Vec3::new(0.0, 1.0, 1.0));
    cases.push(ok(
        "lines meeting off the origin",
        env,
        two(a, b),
        want(
            Vec3::new(0.0, -FRAC_1_SQRT_2, FRAC_1_SQRT_2),
            p3(1.0, 1.0, 1.0),
        ),
    ));
    let mut env = Env::default();
    let b = env.line(p3(3.0, 0.0, 0.0), Vec3::Z);
    cases.push(ok(
        "parallel lines along Z",
        env,
        two(Z_AXIS, b),
        want(Vec3::Y, p3(3.0, 0.0, 9.0)),
    ));
    let mut env = Env::default();
    let straight = env.add(RefGeom::Curve(Arc::new(
        SegmentCurve::new(&Segment3 {
            a: p3(0.0, 2.0, 0.0),
            b: p3(0.0, 6.0, 0.0),
        })
        .unwrap(),
    )));
    cases.push(ok(
        "a straight free-form curve and the X axis",
        env,
        two(straight, X_AXIS),
        want(Vec3::NEG_Z, p3(0.0, 2.0, 0.0)),
    ));
    // Errors.
    cases.push(err(
        "the same line",
        Env::default(),
        two(X_AXIS, X_AXIS),
        ConstructError::Coincident,
    ));
    let mut env = Env::default();
    let b = env.line(p3(0.0, 0.0, 0.01), Vec3::Y);
    cases.push(err(
        "skew lines 0.01 mm apart",
        env,
        two(X_AXIS, b),
        ConstructError::Skew { distance: 0.01 },
    ));
    let mut env = Env::default();
    let arc = env.add(RefGeom::Curve(Arc::new(
        CircleCurve::new(Point3::ZERO, Dir3::Z, 5.0, Some((0.0, FRAC_PI_2))).unwrap(),
    )));
    cases.push(err(
        "a curved free-form edge",
        env,
        two(arc, X_AXIS),
        ConstructError::NotStraight,
    ));
    let mut env = Env::default();
    let pt = env.point(1.0, 0.0, 0.0);
    cases.push(err(
        "a point",
        env,
        two(X_AXIS, pt),
        ConstructError::WrongInput {
            expected: "line or straight edge",
            got: "point",
        },
    ));
    run(cases, "two edges", 5);

    // The skew gap is exact to 1e-12.
    let skew = math::plane_two_lines(
        &Line3 {
            point: Point3::ZERO,
            dir: Dir3::X,
        },
        &Line3 {
            point: p3(0.0, 0.0, 0.01),
            dir: dir(Vec3::new(1.0, 1.0, 0.0)),
        },
        &Tolerance::DEFAULT,
    );
    let Err(ConstructError::Skew { distance }) = skew else {
        panic!("{skew:?}")
    };
    assert!((distance - 0.01).abs() <= EPS, "{distance}");
}

#[test]
fn plane_through_three_points() {
    let three = |env: &mut Env, a: Point3, b: Point3, c: Point3| PlaneDef::ThreePoints {
        a: env.add(RefGeom::Point(a)),
        b: env.add(RefGeom::Point(b)),
        c: env.add(RefGeom::Point(c)),
    };
    let mut cases = Vec::new();
    let table = vec![
        (
            "XY, counter-clockwise",
            [Point3::ZERO, p3(1.0, 0.0, 0.0), p3(0.0, 1.0, 0.0)],
            Some(want(Vec3::Z, Point3::ZERO).at(Point3::ZERO).x(Vec3::X)),
        ),
        (
            "XY, clockwise",
            [Point3::ZERO, p3(0.0, 1.0, 0.0), p3(1.0, 0.0, 0.0)],
            Some(want(Vec3::NEG_Z, Point3::ZERO).x(Vec3::Y)),
        ),
        (
            "unit triangle",
            [p3(1.0, 0.0, 0.0), p3(0.0, 1.0, 0.0), p3(0.0, 0.0, 1.0)],
            Some(want(Vec3::ONE.normalize(), p3(1.0, 0.0, 0.0)).x(Vec3::new(
                -FRAC_1_SQRT_2,
                FRAC_1_SQRT_2,
                0.0,
            ))),
        ),
        (
            "raised XY",
            [p3(0.0, 0.0, 5.0), p3(1.0, 0.0, 5.0), p3(0.0, 1.0, 5.0)],
            Some(want(Vec3::Z, p3(9.0, 9.0, 5.0))),
        ),
        (
            "XZ points: Z × X = +Y",
            [Point3::ZERO, p3(0.0, 0.0, 1.0), p3(1.0, 0.0, 0.0)],
            Some(want(Vec3::Y, Point3::ZERO)),
        ),
        (
            "collinear",
            [Point3::ZERO, p3(1.0, 0.0, 0.0), p3(2.0, 0.0, 0.0)],
            None,
        ),
        (
            "two coincident points",
            [p3(1.0, 1.0, 1.0), p3(1.0, 1.0, 1.0), p3(2.0, 0.0, 0.0)],
            None,
        ),
    ];
    for (name, [a, b, c], w) in table {
        let mut env = Env::default();
        let def = three(&mut env, a, b, c);
        cases.push(match w {
            Some(w) => ok(name, env, def, w),
            None => err(name, env, def, ConstructError::Collinear),
        });
    }
    let mut env = Env::default();
    let (a, b) = (env.point(0.0, 0.0, 0.0), env.point(1.0, 0.0, 0.0));
    cases.push(err(
        "an axis",
        env,
        PlaneDef::ThreePoints { a, b, c: Z_AXIS },
        ConstructError::WrongInput {
            expected: "point",
            got: "line",
        },
    ));
    run(cases, "three points", 5);
}

/// A free-form test face: the plane z = 1 (outward +Z), or a face whose queries fail.
#[derive(Debug)]
struct TestFace {
    closest_fails: bool,
    singular: bool,
}

impl SurfaceEval for TestFace {
    fn eval(&self, uv: Point2) -> Point3 {
        p3(uv.x, uv.y, 1.0)
    }
    fn normal(&self, _uv: Point2) -> Option<Dir3> {
        (!self.singular).then_some(Dir3::Z)
    }
    fn closest_uv(&self, p: Point3) -> Option<Point2> {
        (!self.closest_fails).then_some(Point2::new(p.x, p.y))
    }
}

#[test]
fn plane_tangent_to_face_at_point() {
    let case = |name, g: RefGeom, p: Point3, w: Option<Want>, e: Option<ConstructError>| {
        let mut env = Env::default();
        let face = env.add(g);
        let point = env.add(RefGeom::Point(p));
        let def = PlaneDef::TangentAtPoint { face, point };
        match (w, e) {
            (Some(w), _) => ok(name, env, def, w),
            (None, Some(e)) => err(name, env, def, e),
            (None, None) => unreachable!(),
        }
    };
    let z_axis = Line3 {
        point: Point3::ZERO,
        dir: Dir3::Z,
    };
    let from_normal_x = |n: Dir3| Plane::from_normal(Point3::ZERO, n).x.get();
    let cases = vec![
        case(
            "planar face",
            RefGeom::Plane(Plane::XY),
            p3(1.0, 2.0, 3.0),
            Some(
                want(Vec3::Z, Point3::ZERO)
                    .at(p3(1.0, 2.0, 0.0))
                    .x(from_normal_x(Dir3::Z)),
            ),
            None,
        ),
        case(
            "sphere from outside",
            RefGeom::Sphere {
                center: Point3::ZERO,
                radius: 5.0,
            },
            p3(0.0, 0.0, 10.0),
            Some(want(Vec3::Z, p3(0.0, 0.0, 5.0)).at(p3(0.0, 0.0, 5.0))),
            None,
        ),
        case(
            "cylinder",
            RefGeom::Cylinder {
                axis: z_axis,
                radius: 2.0,
            },
            p3(3.0, 0.0, 7.0),
            Some(want(Vec3::X, p3(2.0, 0.0, 7.0)).at(p3(2.0, 0.0, 7.0))),
            None,
        ),
        case(
            "45° cone, point on the surface",
            RefGeom::Cone {
                apex: Point3::ZERO,
                axis: Dir3::Z,
                half_angle: FRAC_PI_4,
            },
            p3(1.0, 0.0, 1.0),
            Some(
                want(Vec3::new(FRAC_1_SQRT_2, 0.0, -FRAC_1_SQRT_2), Point3::ZERO)
                    .at(p3(1.0, 0.0, 1.0)),
            ),
            None,
        ),
        case(
            "torus outer equator",
            RefGeom::Torus {
                axis: z_axis,
                major: 10.0,
                minor: 2.0,
            },
            p3(13.0, 0.0, 0.0),
            Some(want(Vec3::X, p3(12.0, 0.0, 0.0)).at(p3(12.0, 0.0, 0.0))),
            None,
        ),
        case(
            "free-form face",
            RefGeom::Surface(Arc::new(TestFace {
                closest_fails: false,
                singular: false,
            })),
            p3(4.0, -2.0, 6.0),
            Some(want(Vec3::Z, p3(4.0, -2.0, 1.0)).at(p3(4.0, -2.0, 1.0))),
            None,
        ),
        case(
            "closest-point query fails",
            RefGeom::Surface(Arc::new(TestFace {
                closest_fails: true,
                singular: false,
            })),
            p3(4.0, -2.0, 6.0),
            None,
            Some(ConstructError::NotOnFace {
                distance: f64::INFINITY,
            }),
        ),
        case(
            "singular point of a free-form face",
            RefGeom::Surface(Arc::new(TestFace {
                closest_fails: false,
                singular: true,
            })),
            Point3::ZERO,
            None,
            Some(ConstructError::Degenerate("singular point on the face")),
        ),
        case(
            "point at the sphere centre",
            RefGeom::Sphere {
                center: Point3::ZERO,
                radius: 5.0,
            },
            Point3::ZERO,
            None,
            Some(ConstructError::Degenerate(
                "point on the surface axis or centre",
            )),
        ),
        case(
            "a line is not a face",
            RefGeom::Line(z_axis),
            Point3::ZERO,
            None,
            Some(ConstructError::WrongInput {
                expected: "face",
                got: "line",
            }),
        ),
    ];
    run(cases, "tangent at point", 5);
}

#[test]
fn plane_along_path() {
    let seg = || {
        RefGeom::Segment(Segment3 {
            a: Point3::ZERO,
            b: p3(10.0, 0.0, 0.0),
        })
    };
    let frac = |path, v: f64| PlaneDef::AlongPath {
        path,
        at: PathPosition::Fraction(num(v)),
    };
    let dist = |path, v: f64| PlaneDef::AlongPath {
        path,
        at: PathPosition::Distance(num(v)),
    };
    let circle_frame = Plane::from_normal(Point3::ZERO, Dir3::Z);
    let (fx, fy) = (circle_frame.x.get(), circle_frame.y.get());
    let mut cases = Vec::new();
    let (env, p) = single(seg());
    cases.push(ok(
        "segment at half",
        env,
        frac(p, 0.5),
        want(Vec3::X, p3(5.0, 0.0, 0.0)).at(p3(5.0, 0.0, 0.0)),
    ));
    let (env, p) = single(seg());
    cases.push(ok(
        "segment at 2.5 mm",
        env,
        dist(p, 2.5),
        want(Vec3::X, p3(2.5, 0.0, 0.0)).at(p3(2.5, 0.0, 0.0)),
    ));
    let (env, p) = single(seg());
    cases.push(ok(
        "segment end",
        env,
        frac(p, 1.0),
        want(Vec3::X, p3(10.0, 0.0, 0.0)).at(p3(10.0, 0.0, 0.0)),
    ));
    let (env, p) = single(RefGeom::Circle {
        center: Point3::ZERO,
        normal: Dir3::Z,
        radius: 5.0,
        arc: None,
    });
    cases.push(ok(
        "full circle at its start: x = principal normal (towards the centre)",
        env,
        frac(p, 0.0),
        want(fy, fx * 5.0).at(fx * 5.0).x(-fx),
    ));
    let (env, p) = single(RefGeom::Circle {
        center: Point3::ZERO,
        normal: Dir3::Z,
        radius: 2.0,
        arc: Some((0.0, PI)),
    });
    cases.push(ok(
        "half arc r = 2 at arc length π (90°)",
        env,
        dist(p, PI),
        want(-fx, fy * 2.0).at(fy * 2.0).x(-fy),
    ));
    let (env, p) = single(RefGeom::Curve(Arc::new(
        CircleCurve::new(p3(0.0, 0.0, 3.0), Dir3::Z, 4.0, None).unwrap(),
    )));
    cases.push(ok(
        "free-form circle at a quarter",
        env,
        frac(p, 0.25),
        want(-fx, p3(0.0, 0.0, 3.0) + fy * 4.0).at(p3(0.0, 0.0, 3.0) + fy * 4.0),
    ));
    // Errors.
    let (env, p) = single(seg());
    cases.push(err(
        "fraction above 1",
        env,
        frac(p, 1.5),
        ConstructError::OutOfRange,
    ));
    let (env, p) = single(seg());
    cases.push(err(
        "negative distance",
        env,
        dist(p, -1.0),
        ConstructError::OutOfRange,
    ));
    let (env, p) = single(seg());
    cases.push(err(
        "distance past the end",
        env,
        dist(p, 10.5),
        ConstructError::OutOfRange,
    ));
    cases.push(err(
        "an infinite axis is not a path",
        Env::default(),
        frac(X_AXIS, 0.5),
        ConstructError::WrongInput {
            expected: "edge or sketch curve",
            got: "line",
        },
    ));
    run(cases, "along path", 5);

    // `plane_along_path` refuses parameters outside the range on its own.
    let c = SegmentCurve::new(&Segment3 {
        a: Point3::ZERO,
        b: p3(1.0, 0.0, 0.0),
    })
    .unwrap();
    assert_eq!(
        math::plane_along_path(&c, 1.5, &Tolerance::DEFAULT),
        Err(ConstructError::OutOfRange)
    );
    assert_eq!(
        math::plane_along_path(&c, f64::NAN, &Tolerance::DEFAULT),
        Err(ConstructError::OutOfRange)
    );
}

// ---- framework -----------------------------------------------------------------------

#[test]
fn origin_entities_follow_the_component_transform() {
    let t = Transform::from_rotation_translation(
        amg_core::Quat::from_rotation_z(FRAC_PI_2),
        Vec3::new(1.0, 2.0, 3.0),
    );
    let RefGeom::Plane(xy) = math::origin_geom(OriginEntity::XYPlane, &t) else {
        panic!()
    };
    assert!(close(xy.origin, p3(1.0, 2.0, 3.0)));
    assert!(close(xy.x.get(), Vec3::Y));
    assert!(close(xy.normal.get(), Vec3::Z));
    let RefGeom::Plane(xz) = math::origin_geom(OriginEntity::XZPlane, &Transform::IDENTITY) else {
        panic!()
    };
    assert_eq!(xz, Plane::XZ);
    let RefGeom::Line(x) = math::origin_geom(OriginEntity::XAxis, &t) else {
        panic!()
    };
    assert!(close(x.dir.get(), Vec3::Y));
    let RefGeom::Point(o) = math::origin_geom(OriginEntity::Point, &t) else {
        panic!()
    };
    assert!(close(o, p3(1.0, 2.0, 3.0)));
}

#[test]
fn a_fingerprint_only_match_is_a_warning_naming_the_reference() {
    let mut env = Env::default();
    let base = env.add_resolved(ResolvedRef {
        geom: RefGeom::Plane(Plane::XY),
        context_plane: None,
        quality: Some(MatchQuality::FingerprintOnly),
    });
    let (p, warnings) = offset(base.clone(), 1.0).evaluate(&env).unwrap();
    assert!(close(p.origin, p3(0.0, 0.0, 1.0)));
    assert_eq!(warnings.len(), 1);
    assert_eq!(warnings[0].entities, vec![base.entity().unwrap().clone()]);
}

#[test]
fn inputs_list_every_reference_and_exprs_every_parameter() {
    let a = EntityRef::Construction(ConstructionId(FeatureId(5)));
    let f = ConstructPlane {
        def: PlaneDef::AtAngle {
            line: X_AXIS,
            zero: Some(RefInput::Ref(a.clone())),
            angle: num(0.5),
        },
    };
    assert_eq!(
        f.inputs(),
        vec![Dependency::Construction(ConstructionId(FeatureId(5)))]
    );
    assert_eq!(f.exprs().len(), 1);
    let mut g = ConstructPlane {
        def: PlaneDef::AlongPath {
            path: RefInput::Ref(a.clone()),
            at: PathPosition::Fraction(num(0.5)),
        },
    };
    let dims: Vec<_> = g.exprs_mut().into_iter().map(|(_, d)| d).collect();
    assert_eq!(dims, vec![amg_core::Dimension::NONE]);
    let three = ConstructPlane {
        def: PlaneDef::ThreePoints {
            a: RefInput::Origin(OriginEntity::Point),
            b: RefInput::Ref(a.clone()),
            c: RefInput::Ref(a),
        },
    };
    assert_eq!(three.inputs().len(), 2);
    assert!(three.exprs().is_empty());
}

#[test]
fn construct_error_maps_to_an_invalid_inputs_feature_error() {
    assert_eq!(
        FeatureError::from(ConstructError::Collinear),
        FeatureError::InvalidValue {
            field: "inputs",
            msg: "points are collinear".to_owned(),
        }
    );
}

#[test]
fn circle_curve_closest_param_wraps_and_clamps() {
    let c = CircleCurve::new(Point3::ZERO, Dir3::Z, 1.0, Some((0.0, FRAC_PI_2))).unwrap();
    let f = Plane::from_normal(Point3::ZERO, Dir3::Z);
    let at = |a: f64| f.to_world(Point2::new(a.cos(), a.sin()));
    assert!((c.closest_param(at(0.3)) - 0.3).abs() <= EPS);
    // Just past the end → end; just before the start → start.
    assert_eq!(c.closest_param(at(FRAC_PI_2 + 0.1)), FRAC_PI_2);
    assert_eq!(c.closest_param(at(-0.1)), 0.0);
    assert!((c.length() - FRAC_PI_2).abs() <= EPS);
}
