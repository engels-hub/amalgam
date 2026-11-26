//! The OCCT half of the SOL-02 tests: goldens `tests/golden/solid/sol02_*.toml`, the start ×
//! direction × extent grid, target failures, "All" cuts, text, naming and a proptest.

use std::f64::consts::TAU;

use amg_brep::geom::{BSplineSurfaceData, FaceBounds, Surface};
use amg_brep::{BrepKernel, Shape as BShape, SubKind, SubShape};
use amg_core::{Dir3, Plane, Point2, Point3, Tolerance, Vec3};
use amg_doc::naming::{GeomSource, KernelGeom, NamingView, Role, default_kernel, topo_ref_at};
use amg_doc::stub::ProfileKey;
use amg_doc::testing::brep::TestBrepFillet;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, ConstructionId, Dependency, Evaluated,
    FeatureError, FeatureOutput, GeomFingerprint, Health, MatchQuality, RecomputeCtx, SetPolicy,
    SketchId, TopoKind, TopoRef,
};
use amg_features::construct::{ConstructPlane, OriginEntity, PlaneDef, RefInput};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::BodyOp;
use serde::{Deserialize, Serialize};

use super::*;

// ---- a test feature for target bodies ---------------------------------------------------------

/// A body from a kernel primitive (S02), faces named role-only `F#i`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sol02Prim {
    pub prim: Prim,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Prim {
    /// A box on the frame (`origin`, `normal`, `x` hint) with edge lengths `size`.
    Box {
        origin: [f64; 3],
        normal: [f64; 3],
        x: [f64; 3],
        size: [f64; 3],
    },
    /// A full cylinder with base centre `origin` and axis `normal`.
    Cylinder {
        origin: [f64; 3],
        normal: [f64; 3],
        radius: f64,
        height: f64,
    },
    /// A bilinear B-spline sheet over x, y ∈ [−50, 50] at z = z0 + slope·x.
    Sheet { z0: f64, slope: f64 },
}

fn p3(a: [f64; 3]) -> Point3 {
    Point3::from_array(a)
}

fn frame(origin: [f64; 3], normal: [f64; 3], x: Option<[f64; 3]>) -> Plane {
    let n = Dir3::new_unchecked(Vec3::from_array(normal).normalize());
    match x {
        Some(x) => {
            Plane::from_normal_x(p3(origin), n, Vec3::from_array(x), &Tolerance::default()).unwrap()
        }
        None => Plane::from_normal(p3(origin), n),
    }
}

impl Feature for Sol02Prim {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let k = ctx.kernel();
        let kerr = |e: amg_brep::KernelError| FeatureError::Kernel(e.to_string());
        let shape = match &self.prim {
            Prim::Box {
                origin,
                normal,
                x,
                size,
            } => {
                k.make_box(&frame(*origin, *normal, Some(*x)), Vec3::from_array(*size))
                    .map_err(kerr)?
                    .shape
            }
            Prim::Cylinder {
                origin,
                normal,
                radius,
                height,
            } => {
                k.make_cylinder(&frame(*origin, *normal, None), *radius, *height, TAU)
                    .map_err(kerr)?
                    .shape
            }
            Prim::Sheet { z0, slope } => {
                let z = |x: f64| z0 + slope * x;
                let s = Surface::BSpline(BSplineSurfaceData {
                    u_degree: 1,
                    v_degree: 1,
                    u_knots: vec![0.0, 1.0],
                    u_mults: vec![2, 2],
                    v_knots: vec![0.0, 1.0],
                    v_mults: vec![2, 2],
                    poles: vec![
                        Point3::new(-50.0, -50.0, z(-50.0)),
                        Point3::new(-50.0, 50.0, z(-50.0)),
                        Point3::new(50.0, -50.0, z(50.0)),
                        Point3::new(50.0, 50.0, z(50.0)),
                    ],
                    nu: 2,
                    nv: 2,
                    weights: None,
                    u_periodic: false,
                    v_periodic: false,
                });
                k.make_face(&s, &FaceBounds::Natural, 1e-6)
                    .map_err(kerr)?
                    .face
            }
        };
        let mut b = ctx.naming_builder();
        for f in 0..shape.count(SubKind::Face) {
            b.generated(TopoKind::Face, f, Role::new("F", f), None);
        }
        let naming = b
            .finish(k, &shape)
            .map_err(|e| FeatureError::Kernel(e.to_string()))?;
        let mut out = FeatureOutput::default();
        out.bodies.push(BodyChange {
            id: ctx.new_body_id(0),
            change: BodyChangeKind::Created {
                geom: BodyGeom::Solid(amg_doc::stub::Shape::from_brep(shape)),
                naming: Some(naming),
            },
        });
        Ok(out)
    }
}

amg_doc::register_feature!(Sol02Prim, "sol02test.prim", version = 1);

fn prim(doc: &mut Document, p: Prim) -> BodyId {
    body0(add(doc, FeatureBox::new(Sol02Prim { prim: p })))
}

/// An axis-aligned box with its min corner at `min`.
fn abox(doc: &mut Document, min: [f64; 3], size: [f64; 3]) -> BodyId {
    prim(
        doc,
        Prim::Box {
            origin: min,
            normal: [0.0, 0.0, 1.0],
            x: [1.0, 0.0, 0.0],
            size,
        },
    )
}

/// A construction plane: XY offset by `z`.
fn xy_plane(doc: &mut Document, z: f64) -> EntityRef {
    let f = add(
        doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: ex(&format!("{z} mm")),
            },
        }),
    );
    EntityRef::Construction(ConstructionId(f))
}

fn shape_of(doc: &Document, b: BodyId) -> BShape {
    brep(doc, b)
}

/// The face of `b` whose fingerprint centroid is nearest to `near`.
fn face_near(doc: &Document, b: BodyId, near: [f64; 3]) -> EntityRef {
    EntityRef::Topo(pick(doc.evaluated(), b, TopoKind::Face, near))
}

fn vertex_near(doc: &Document, b: BodyId, near: [f64; 3]) -> EntityRef {
    EntityRef::Topo(pick(doc.evaluated(), b, TopoKind::Vertex, near))
}

fn pick(ev: &Evaluated, body: BodyId, kind: TopoKind, near: [f64; 3]) -> TopoRef {
    let geom = KernelGeom::new(ev, default_kernel());
    let near = Point3::from_array(near);
    let mut found: Vec<(f64, u32)> = GeomSource::all(&geom, body, kind)
        .into_iter()
        .map(|(i, fp)| ((fp.centroid - near).length(), i))
        .collect();
    found.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert!(
        found.len() == 1 || found[1].0 - found[0].0 > 1e-6,
        "tied pick"
    );
    let mut r = topo_ref_at(ev, default_kernel(), body, kind, found[0].1).unwrap();
    r.key = 1;
    r
}

const RECT: [f64; 4] = [0.0, 0.0, 10.0, 20.0];
const AREA: f64 = 200.0;

/// The 10 × 20 rectangle sketch on XY and its profile.
fn rect(doc: &mut Document) -> Vec<EntityRef> {
    let s = add(doc, FeatureBox::new(rect_sketch(&[RECT])));
    profiles(doc, s)
}

fn side(extent: Extent, taper: &str) -> ExtentSide {
    ExtentSide {
        extent,
        taper: ex(taper),
    }
}

fn dist(d: f64) -> Extent {
    Extent::Distance(ex(&format!("{d} mm")))
}

fn to(target: &EntityRef, offset: f64, extend: bool) -> Extent {
    Extent::ToObject {
        target: target.clone(),
        offset: ex(&format!("{offset} mm")),
        extend,
    }
}

fn extrude(profiles: Vec<EntityRef>) -> Extrude {
    Extrude::distance(profiles, "10 mm", OpTarget::default())
}

/// Adds `e`, asserts it computed, returns its body 0.
#[track_caller]
fn ok(doc: &mut Document, e: Extrude) -> (FeatureId, BShape) {
    let id = add(doc, FeatureBox::new(e));
    assert_eq!(health(doc, id), Health::Ok, "{:?}", health(doc, id));
    (id, shape_of(doc, body0(id)))
}

#[track_caller]
fn err(doc: &mut Document, e: Extrude) -> String {
    let id = add(doc, FeatureBox::new(e));
    match health(doc, id) {
        Health::Error(d) => d.message,
        h => panic!("expected an error, got {h:?}"),
    }
}

#[track_caller]
fn valid(s: &BShape) {
    let r = kernel().check(s, &amg_brep::CheckOpts::default());
    assert!(r.valid, "{r:?}");
}

#[track_caller]
fn golden(case: &str, s: &BShape, rel: f64) {
    valid(s);
    check_golden_as("sol02", rel, case, s);
}

#[track_caller]
fn zrange(s: &BShape, lo: f64, hi: f64) {
    let b = s.bbox();
    assert!(
        (b.min.z - lo).abs() < 1e-6 && (b.max.z - hi).abs() < 1e-6,
        "z range {} .. {}, want {lo} .. {hi}",
        b.min.z,
        b.max.z
    );
}

/// Volume of the 10 × 20 rectangle prism of height `h` tapered by `deg` (positive grows).
fn tapered(h: f64, deg: f64) -> f64 {
    let t = deg.to_radians().tan();
    AREA * h + 30.0 * h * h * t + 4.0 / 3.0 * h.powi(3) * t * t
}

// ---- Done-when 1: goldens, analytic volumes ---------------------------------------------------

#[test]
fn dw1_offset_start() {
    let mut doc = Document::new("offset");
    let mut e = extrude(rect(&mut doc));
    e.start = Start::Offset(ex("5 mm"));
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), 2000.0, 1e-9);
    zrange(&s, 5.0, 15.0);
    golden("offset_start", &s, 1e-9);
}

#[test]
fn dw1_start_from_a_face_plane_and_vertex() {
    let mut doc = Document::new("start-object");
    let block = abox(&mut doc, [-15.0, -10.0, 0.0], [40.0, 40.0, 7.0]);
    let pr = rect(&mut doc);
    let top = face_near(&doc, block, [5.0, 10.0, 7.0]);
    for (case, offset, lo) in [("start_face", 0.0, 7.0), ("start_face_offset", 2.0, 9.0)] {
        let mut e = extrude(pr.clone());
        e.start = Start::Object {
            target: top.clone(),
            offset: ex(&format!("{offset} mm")),
        };
        let (_, s) = ok(&mut doc, e);
        assert_rel(volume(&s), 2000.0, 1e-9);
        zrange(&s, lo, lo + 10.0);
        golden(case, &s, 1e-9);
    }
    let corner = vertex_near(&doc, block, [25.0, 30.0, 7.0]);
    let mut e = extrude(pr.clone());
    e.start = Start::Object {
        target: corner,
        offset: ex("0 mm"),
    };
    e.side1 = side(dist(5.0), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), 1000.0, 1e-9);
    zrange(&s, 7.0, 12.0);
    golden("start_vertex", &s, 1e-9);
    let plane = xy_plane(&mut doc, 12.0);
    let mut e = extrude(pr.clone());
    e.start = Start::Object {
        target: plane,
        offset: ex("0 mm"),
    };
    let (_, s) = ok(&mut doc, e);
    zrange(&s, 12.0, 22.0);
    golden("start_plane", &s, 1e-9);
    // A start plane that is not parallel to the profile is refused.
    let side_face = face_near(&doc, block, [-15.0, 10.0, 3.5]);
    let mut e = extrude(pr);
    e.start = Start::Object {
        target: side_face,
        offset: ex("0 mm"),
    };
    let m = err(&mut doc, e);
    assert!(m.contains("parallel"), "{m}");
}

#[test]
fn dw1_two_sides_symmetric_and_tapers() {
    let mut doc = Document::new("sides");
    let pr = rect(&mut doc);
    // Two sides 10 / 5: one unified block.
    let mut e = extrude(pr.clone());
    e.direction = Direction::TwoSides;
    e.side2 = Some(side(dist(5.0), "0 deg"));
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), 3000.0, 1e-9);
    zrange(&s, -5.0, 10.0);
    assert_eq!(faces(&s), 6, "the sides' laterals merge");
    golden("two_sides", &s, 1e-9);
    // Two sides, each with its own taper.
    let mut e = extrude(pr.clone());
    e.direction = Direction::TwoSides;
    e.side1 = side(dist(10.0), "3 deg");
    e.side2 = Some(side(dist(5.0), "-2 deg"));
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), tapered(10.0, 3.0) + tapered(5.0, -2.0), 1e-9);
    golden("two_sides_taper", &s, 1e-9);
    // Symmetric half and whole length.
    for (case, whole, d) in [
        ("symmetric_half", false, 20.0),
        ("symmetric_whole", true, 10.0),
    ] {
        let mut e = extrude(pr.clone());
        e.direction = Direction::Symmetric {
            whole_length: whole,
        };
        e.side1 = side(dist(d), "0 deg");
        let (_, s) = ok(&mut doc, e);
        assert_rel(volume(&s), 4000.0, 1e-9);
        zrange(&s, -10.0, 10.0);
        assert_eq!(faces(&s), 6);
        golden(case, &s, 1e-9);
    }
    // Symmetric with a taper: both halves grow from the profile plane.
    let mut e = extrude(pr.clone());
    e.direction = Direction::Symmetric {
        whole_length: false,
    };
    e.side1 = side(dist(20.0), "4 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), 2.0 * tapered(10.0, 4.0), 1e-9);
    golden("symmetric_taper", &s, 1e-9);
    // One side, tapered out and in.
    for (case, deg) in [("taper_out", 5.0), ("taper_in", -5.0)] {
        let mut e = extrude(pr.clone());
        e.side1 = side(dist(10.0), &format!("{deg} deg"));
        let (_, s) = ok(&mut doc, e);
        assert_rel(volume(&s), tapered(10.0, deg), 1e-9);
        golden(case, &s, 1e-9);
    }
}

#[test]
fn dw1_to_parallel_plane_face_and_vertex() {
    let mut doc = Document::new("to-parallel");
    let pr = rect(&mut doc);
    let plane = xy_plane(&mut doc, 25.0);
    for (case, offset, h) in [("to_plane", 0.0, 25.0), ("to_plane_offset", -5.0, 20.0)] {
        let mut e = extrude(pr.clone());
        e.side1 = side(to(&plane, offset, false), "0 deg");
        let (_, s) = ok(&mut doc, e);
        assert_rel(volume(&s), AREA * h, 1e-9);
        zrange(&s, 0.0, h);
        golden(case, &s, 1e-9);
    }
    // A planar face parallel to the profile: the exact distance, even where the face is
    // smaller than the profile (it is a plane extent, note §3.2).
    let block = abox(&mut doc, [100.0, 0.0, 30.0], [5.0, 5.0, 7.0]);
    let bottom = face_near(&doc, block, [102.5, 2.5, 30.0]);
    let mut e = extrude(pr.clone());
    e.side1 = side(to(&bottom, 0.0, false), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), AREA * 30.0, 1e-9);
    golden("to_face_parallel", &s, 1e-9);
    let v = vertex_near(&doc, block, [105.0, 5.0, 37.0]);
    let mut e = extrude(pr.clone());
    e.side1 = side(to(&v, 3.0, false), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), AREA * 40.0, 1e-9);
    zrange(&s, 0.0, 40.0);
    golden("to_vertex_offset", &s, 1e-9);
    // To object with a taper: the exact tapered prism.
    let mut e = extrude(pr);
    e.side1 = side(to(&plane, 0.0, false), "2 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), tapered(25.0, 2.0), 1e-9);
    golden("to_plane_taper", &s, 1e-9);
}

const ALPHA: f64 = 20.0;

/// A box tilted by `ALPHA` about Y whose bottom face is the plane through (−30, ·, 15) with
/// `z = 15 + tan α (x + 30)`, `size[0]` long along its tilted x.
fn tilted(doc: &mut Document, size_x: f64) -> BodyId {
    let a = ALPHA.to_radians();
    prim(
        doc,
        Prim::Box {
            origin: [-30.0, -30.0, 15.0],
            normal: [-a.sin(), 0.0, a.cos()],
            x: [a.cos(), 0.0, a.sin()],
            size: [size_x, 80.0, 20.0],
        },
    )
}

/// Volume under `z = 15 + tan α (x + 30) + dz` over the rectangle.
fn under_tilted(dz: f64) -> f64 {
    let t = ALPHA.to_radians().tan();
    20.0 * (10.0 * (15.0 + dz) + t * (50.0 + 300.0))
}

#[test]
fn dw1_to_inclined_face_with_offset_and_extend() {
    let mut doc = Document::new("inclined");
    let pr = rect(&mut doc);
    let big = tilted(&mut doc, 80.0);
    let a = ALPHA.to_radians();
    let bottom = face_near(&doc, big, [10.0, 10.0, 15.0 + 40.0 * a.tan()]);
    let mut e = extrude(pr.clone());
    e.side1 = side(to(&bottom, 0.0, false), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), under_tilted(0.0), 1e-9);
    golden("to_inclined_face", &s, 1e-9);
    // Offset 2 mm along the extrude side moves the plane up by 2 / cos α.
    let mut e = extrude(pr.clone());
    e.side1 = side(to(&bottom, 2.0, false), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), under_tilted(2.0 / a.cos()), 1e-9);
    golden("to_inclined_face_offset", &s, 1e-9);
    // The body as the target gives the same piece.
    let mut e = extrude(pr.clone());
    e.side1 = side(to(&EntityRef::Body(big), 0.0, false), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), under_tilted(0.0), 1e-9);
    golden("to_body_inclined", &s, 1e-9);
}

/// ∫ sqrt(R² − s²) ds over s ∈ [−5, 5].
fn chord_area(r: f64) -> f64 {
    5.0 * (r * r - 25.0).sqrt() + r * r * (5.0 / r).asin()
}

/// A cylinder along Y with its axis at x = 5, z = 40, radius 20, from y = `y0`, `len` long.
fn cylinder(doc: &mut Document, y0: f64, len: f64) -> BodyId {
    prim(
        doc,
        Prim::Cylinder {
            origin: [5.0, y0, 40.0],
            normal: [0.0, 1.0, 0.0],
            radius: 20.0,
            height: len,
        },
    )
}

fn cyl_face(doc: &Document, c: BodyId) -> EntityRef {
    let s = shape_of(doc, c);
    let f = s
        .faces()
        .find(|f| matches!(s.surface(*f), Surface::Cylinder { .. }))
        .unwrap();
    EntityRef::Topo(
        topo_ref_at(&**doc.evaluated(), default_kernel(), c, TopoKind::Face, f.0).unwrap(),
    )
}

#[test]
fn dw1_to_cylinder_face_with_offset_and_to_body() {
    let mut doc = Document::new("cylinder");
    let pr = rect(&mut doc);
    let c = cylinder(&mut doc, -10.0, 40.0);
    let face = cyl_face(&doc, c);
    let below = |r: f64| 20.0 * (10.0 * 40.0 - chord_area(r));
    let mut e = extrude(pr.clone());
    e.side1 = side(to(&face, 0.0, false), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), below(20.0), 1e-6);
    golden("to_cylinder_face", &s, 1e-6);
    // Offset 2 mm along the face's outward normal: radius 22.
    let mut e = extrude(pr.clone());
    e.side1 = side(to(&face, 2.0, false), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), below(22.0), 1e-6);
    golden("to_cylinder_face_offset", &s, 1e-6);
    // Up to the body: the first faces the prism meets.
    let mut e = extrude(pr);
    e.side1 = side(to(&EntityRef::Body(c), 0.0, false), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), below(20.0), 1e-6);
    golden("to_body_cylinder", &s, 1e-6);
}

#[test]
fn dw1_to_bspline_face() {
    let mut doc = Document::new("bspline");
    let pr = rect(&mut doc);
    let sheet = prim(
        &mut doc,
        Prim::Sheet {
            z0: 30.0,
            slope: 0.1,
        },
    );
    let f = face_near(&doc, sheet, [0.0, 0.0, 30.0]);
    let mut e = extrude(pr);
    e.side1 = side(to(&f, 0.0, false), "0 deg");
    let (_, s) = ok(&mut doc, e);
    // z = 30 + 0.1 x over the rectangle.
    assert_rel(volume(&s), 20.0 * (300.0 + 5.0), 1e-6);
    golden("to_bspline_face", &s, 1e-6);
}

#[test]
fn dw1_combinations_and_offset_start_with_to_object() {
    let mut doc = Document::new("combos");
    let pr = rect(&mut doc);
    let plane = xy_plane(&mut doc, 25.0);
    let low = abox(&mut doc, [60.0, 0.0, -8.0], [5.0, 5.0, 3.0]);
    let v = vertex_near(&doc, low, [65.0, 5.0, -8.0]);
    // Two sides up to a plane and down to a vertex.
    let mut e = extrude(pr.clone());
    e.direction = Direction::TwoSides;
    e.side1 = side(to(&plane, 0.0, false), "0 deg");
    e.side2 = Some(side(to(&v, 0.0, false), "0 deg"));
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), AREA * 33.0, 1e-9);
    zrange(&s, -8.0, 25.0);
    golden("two_sides_to_objects", &s, 1e-9);
    // An offset start up to the plane.
    let mut e = extrude(pr);
    e.start = Start::Offset(ex("5 mm"));
    e.side1 = side(to(&plane, 0.0, false), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), AREA * 20.0, 1e-9);
    zrange(&s, 5.0, 25.0);
    golden("offset_start_to_plane", &s, 1e-9);
}

/// Every start × direction × extent the dialog allows (3 × 14 = 42 cases), on box geometry
/// with analytic z ranges: start at the profile, at +5 or at a plane z = 3; side 1 by 10, up to
/// a face at z = 40 or through all (z = 41); side 2 by 4, down to a vertex at z = −30 or
/// through all (z = −31); symmetric 10, half or whole.
#[test]
fn dw1_every_start_direction_extent_combination() {
    // A fresh document per case keeps "All" independent of earlier cases' bodies; the
    // references are stable across documents (same feature ids).
    let base = || {
        let mut doc = Document::new("grid");
        let pr = rect(&mut doc);
        let fence = abox(&mut doc, [50.0, 0.0, -30.0], [5.0, 5.0, 70.0]);
        let plane3 = xy_plane(&mut doc, 3.0);
        (doc, pr, fence, plane3)
    };
    let (doc, pr, fence, plane3) = base();
    let top = face_near(&doc, fence, [52.5, 2.5, 40.0]);
    let bottom = vertex_near(&doc, fence, [55.0, 5.0, -30.0]);
    let starts: [(Start, f64); 3] = [
        (Start::Profile, 0.0),
        (Start::Offset(ex("5 mm")), 5.0),
        (
            Start::Object {
                target: plane3,
                offset: ex("0 mm"),
            },
            3.0,
        ),
    ];
    let ext1 = |z0: f64| -> [(Extent, f64); 3] {
        [
            (dist(10.0), z0 + 10.0),
            (to(&top, 0.0, false), 40.0),
            (Extent::All, 41.0),
        ]
    };
    let ext2 = |z0: f64| -> [(Extent, f64); 3] {
        [
            (dist(4.0), z0 - 4.0),
            (to(&bottom, 0.0, false), -30.0),
            (Extent::All, -31.0),
        ]
    };
    let mut n = 0;
    for (start, z0) in &starts {
        let mut cases: Vec<(Extrude, f64, f64)> = Vec::new();
        for (x1, hi) in ext1(*z0) {
            let mut e = extrude(pr.clone());
            e.start = start.clone();
            e.side1 = side(x1.clone(), "0 deg");
            cases.push((e, *z0, hi));
            for (x2, lo) in ext2(*z0) {
                let mut e = extrude(pr.clone());
                e.start = start.clone();
                e.direction = Direction::TwoSides;
                e.side1 = side(x1.clone(), "0 deg");
                e.side2 = Some(side(x2, "0 deg"));
                cases.push((e, lo, hi));
            }
        }
        for (whole, h) in [(false, 5.0), (true, 10.0)] {
            let mut e = extrude(pr.clone());
            e.start = start.clone();
            e.direction = Direction::Symmetric {
                whole_length: whole,
            };
            cases.push((e, z0 - h, z0 + h));
        }
        for (e, lo, hi) in cases {
            let label = format!(
                "{:?} {:?} {:?} {:?}",
                e.start, e.direction, e.side1, e.side2
            );
            let (mut doc, ..) = base();
            let id = add(&mut doc, FeatureBox::new(e));
            assert_eq!(health(&doc, id), Health::Ok, "{label}");
            let s = shape_of(&doc, body0(id));
            assert_rel(volume(&s), AREA * (hi - lo), 1e-9);
            let b = s.bbox();
            assert!(
                (b.min.z - lo).abs() < 1e-6 && (b.max.z - hi).abs() < 1e-6,
                "{label}: z {} .. {}",
                b.min.z,
                b.max.z
            );
            assert_eq!(faces(&s), 6, "{label}");
            n += 1;
        }
    }
    assert_eq!(n, 42);
}

#[test]
fn dw1_planar_face_profile_joins_onto_its_body() {
    let mut doc = Document::new("face-profile");
    let block = abox(&mut doc, [-15.0, -10.0, 0.0], [40.0, 40.0, 7.0]);
    let top = face_near(&doc, block, [5.0, 10.0, 7.0]);
    let mut e = Extrude::distance(
        vec![top],
        "5 mm",
        OpTarget {
            op: BodyOp::Join,
            participants: vec![block],
        },
    );
    e.side1.taper = ex("0 deg");
    let id = add(&mut doc, FeatureBox::new(e));
    assert_eq!(health(&doc, id), Health::Ok, "{:?}", health(&doc, id));
    let s = shape_of(&doc, block);
    assert_rel(volume(&s), 1600.0 * 12.0, 1e-9);
    assert_eq!(faces(&s), 6, "the join is unified");
    golden("face_profile_join", &s, 1e-9);
    // A non-planar face is refused.
    let c = cylinder(&mut doc, -10.0, 40.0);
    let f = cyl_face(&doc, c);
    let m = err(
        &mut doc,
        Extrude::distance(vec![f], "5 mm", OpTarget::default()),
    );
    assert!(m.contains("planar"), "{m}");
}

// ---- Done-when 2: targets that miss the profile --------------------------------------------

#[test]
fn dw2_a_face_that_misses_fails_target_missed_and_extend_fixes_it() {
    let mut doc = Document::new("missed");
    let pr = rect(&mut doc);
    // A cylinder only 10 long in y (profile spans 0..20).
    let short = cylinder(&mut doc, 5.0, 10.0);
    let face = cyl_face(&doc, short);
    let mut e = extrude(pr.clone());
    e.side1 = side(to(&face, 0.0, false), "0 deg");
    let id = add(&mut doc, FeatureBox::new(e.clone()));
    let Health::Error(d) = health(&doc, id) else {
        panic!("{:?}", health(&doc, id));
    };
    assert!(
        d.message.contains("does not cover the profile") && d.message.contains("Extend faces"),
        "{}",
        d.message
    );
    assert!(
        d.entities.contains(&face),
        "the target is highlighted: {:?}",
        d.entities
    );
    e.side1 = side(to(&face, 0.0, true), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), 20.0 * (400.0 - chord_area(20.0)), 1e-6);
    golden("to_cylinder_face_extend", &s, 1e-6);
    // A small inclined face, extended to its plane.
    let small = tilted(&mut doc, 6.0);
    let a = ALPHA.to_radians();
    let bottom = face_near(&doc, small, [-27.0 * 1.0, 10.0, 15.0 + 3.0 * a.tan()]);
    let mut e = extrude(pr.clone());
    e.side1 = side(to(&bottom, 0.0, false), "0 deg");
    let m = err(&mut doc, e.clone());
    assert!(m.contains("Extend faces"), "{m}");
    e.side1 = side(to(&bottom, 0.0, true), "0 deg");
    let (_, s) = ok(&mut doc, e);
    assert_rel(volume(&s), under_tilted(0.0), 1e-9);
    golden("to_inclined_face_extend", &s, 1e-9);
    // A B-spline target can't be extended.
    let sheet = prim(
        &mut doc,
        Prim::Sheet {
            z0: 30.0,
            slope: 0.1,
        },
    );
    let f = face_near(&doc, sheet, [0.0, 0.0, 30.0]);
    let mut e = extrude(pr);
    e.side1 = side(to(&f, 0.0, true), "0 deg");
    let m = err(&mut doc, e);
    assert!(m.contains("can't be extended"), "{m}");
}

#[test]
fn dw2_a_target_behind_or_along_the_profile_is_refused() {
    let mut doc = Document::new("behind");
    let pr = rect(&mut doc);
    let plane = xy_plane(&mut doc, -10.0);
    let mut e = extrude(pr.clone());
    e.side1 = side(to(&plane, 0.0, false), "0 deg");
    let m = err(&mut doc, e.clone());
    assert!(m.contains("behind"), "{m}");
    // Flipped, the same plane works.
    e.flip = true;
    let (_, s) = ok(&mut doc, e);
    zrange(&s, -10.0, 0.0);
}

// ---- Done-when 3: "All" -------------------------------------------------------------------

fn plates(doc: &mut Document, zs: &[f64]) -> Vec<BodyId> {
    zs.iter()
        .map(|z| abox(doc, [-15.0, -10.0, *z], [40.0, 40.0, 5.0]))
        .collect()
}

#[test]
fn dw3_all_cuts_through_three_stacked_bodies() {
    let mut doc = Document::new("all-one");
    let pr = rect(&mut doc);
    let ps = plates(&mut doc, &[5.0, 15.0, 25.0]);
    let below = abox(&mut doc, [-15.0, -10.0, -15.0], [40.0, 40.0, 5.0]);
    let mut participants = ps.clone();
    participants.push(below);
    let mut e = Extrude::distance(
        pr,
        "1 mm",
        OpTarget {
            op: BodyOp::Cut,
            participants,
        },
    );
    e.side1.extent = Extent::All;
    let id = add(&mut doc, FeatureBox::new(e));
    assert_eq!(health(&doc, id), Health::Ok, "{:?}", health(&doc, id));
    for p in &ps {
        let s = shape_of(&doc, *p);
        assert_rel(volume(&s), 8000.0 - 1000.0, 1e-9);
        assert_eq!(faces(&s), 10, "a through hole");
    }
    golden("all_cut_top_plate", &shape_of(&doc, ps[2]), 1e-9);
    // One direction only: the plate below is untouched.
    assert_rel(volume(&shape_of(&doc, below)), 8000.0, 1e-9);
}

#[test]
fn dw3_all_two_sided_cuts_both_ways() {
    let mut doc = Document::new("all-two");
    let pr = rect(&mut doc);
    let ps = plates(&mut doc, &[-20.0, 5.0, 15.0]);
    let mut e = Extrude::distance(
        pr,
        "1 mm",
        OpTarget {
            op: BodyOp::Cut,
            participants: ps.clone(),
        },
    );
    e.direction = Direction::TwoSides;
    e.side1.extent = Extent::All;
    e.side2 = Some(side(Extent::All, "0 deg"));
    let id = add(&mut doc, FeatureBox::new(e));
    assert_eq!(health(&doc, id), Health::Ok, "{:?}", health(&doc, id));
    for p in &ps {
        let s = shape_of(&doc, *p);
        assert_rel(volume(&s), 7000.0, 1e-9);
        assert_eq!(faces(&s), 10);
    }
    golden("all_cut_two_sides_bottom", &shape_of(&doc, ps[0]), 1e-9);
}

// ---- Done-when 4: text ----------------------------------------------------------------------

fn font(name: &str) -> amg_sketch::text::Font {
    let path: std::path::PathBuf = [
        env!("CARGO_MANIFEST_DIR"),
        "..",
        "amg-sketch",
        "tests",
        "fixtures",
        "fonts",
        name,
    ]
    .iter()
    .collect();
    amg_sketch::text::Font::from_bytes(std::fs::read(path).unwrap()).unwrap()
}

#[test]
fn dw4_text_agb_extrudes_to_three_solids_with_four_holes() {
    use amg_sketch::text::{FontSet, Placement, TextBlock, TextSpec};
    let set = FontSet {
        regular: font("NotoSans-Regular-ascii.ttf"),
        bold: None,
        italic: None,
        bold_italic: None,
    };
    let mut doc = Document::new("text");
    let mut sk = SketchFeature::new(PlaneInput::OriginXY);
    let block = sk
        .edit_sketch(|s| {
            Ok(TextBlock::create(
                s,
                &set,
                TextSpec::new("AgB", 10.0, "Noto Sans"),
                &Placement::Free {
                    anchor: Point2::new(0.0, 0.0),
                    angle: 0.0,
                },
            )
            .unwrap())
        })
        .unwrap();
    let sid = add(&mut doc, FeatureBox::new(sk));
    let g = doc.evaluated().sketch(SketchId(sid)).unwrap();
    let regions = block.regions(&g.profiles);
    assert_eq!((regions.solid.len(), regions.counters.len()), (3, 4));
    let refs: Vec<EntityRef> = regions
        .solid
        .iter()
        .map(|id| EntityRef::Profile {
            sketch: SketchId(sid),
            profile: ProfileKey(id.0),
            fingerprint: Some(amg_doc::naming::ProfileFp(
                g.profiles.get(*id).unwrap().fingerprint.clone(),
            )),
        })
        .collect();
    let net: f64 = regions
        .solid
        .iter()
        .map(|id| g.profiles.get(*id).unwrap().fingerprint.area)
        .sum();
    let id = add(
        &mut doc,
        FeatureBox::new(Extrude::distance(refs, "2 mm", OpTarget::default())),
    );
    assert_eq!(health(&doc, id), Health::Ok, "{:?}", health(&doc, id));
    let bodies: Vec<BShape> = (0..3)
        .map(|k| {
            shape_of(
                &doc,
                BodyId {
                    origin: id,
                    slot: k,
                },
            )
        })
        .collect();
    assert!(
        doc.evaluated()
            .body(BodyId {
                origin: id,
                slot: 3
            })
            .is_none(),
        "exactly three solids"
    );
    let mut holes = 0;
    let mut vol = 0.0;
    for (k, s) in bodies.iter().enumerate() {
        valid(s);
        vol += volume(s);
        // Inner wires of the bottom cap (the face at z = 0 with the largest area).
        // Inner wires of the bottom cap (the one face at z = 0).
        let caps: Vec<amg_brep::FaceId> = s
            .faces()
            .filter(|f| {
                let bb = s
                    .sub_shape(SubShape {
                        kind: SubKind::Face,
                        index: f.0,
                    })
                    .unwrap()
                    .bbox();
                bb.max.z.abs() < 1e-6
            })
            .collect();
        assert_eq!(caps.len(), 1);
        holes += s.face_wires(caps[0]).len() - 1;
        golden(&format!("text_agb_{k}"), s, 1e-6);
    }
    assert_eq!(holes, 4, "A 1, g 1, B 2");
    assert_rel(vol, net * 2.0, 1e-6);
}

// ---- Done-when 5: naming ------------------------------------------------------------------

fn fillet(doc: &mut Document, body: BodyId, near: [f64; 3]) -> FeatureId {
    let r = pick(doc.evaluated(), body, TopoKind::Edge, near);
    add(
        doc,
        FeatureBox::new(TestBrepFillet {
            body,
            edges: vec![r],
            radius: 2.0,
            policy: SetPolicy::DropMissing,
        }),
    )
}

fn fillet_resolution(doc: &Document, fil: FeatureId) -> (MatchQuality, GeomFingerprint) {
    let node = doc.evaluated().node(fil).unwrap();
    let rec = &node.resolutions[0];
    let q = rec.result.as_ref().expect("resolved")[0].quality;
    let Some(EntityRef::Topo(t)) = rec.current.first() else {
        panic!("{rec:?}");
    };
    (q, t.fingerprint.clone().unwrap())
}

#[test]
fn dw5_edits_of_distance_taper_and_extent_type_keep_lateral_and_cap_names() {
    let mut doc = Document::new("names");
    let s = add(
        &mut doc,
        FeatureBox::new(rect_sketch(&[[0.0, 0.0, 100.0, 60.0]])),
    );
    let pr = profiles(&doc, s);
    let fence = abox(&mut doc, [200.0, 0.0, -50.0], [5.0, 5.0, 120.0]);
    let plane = xy_plane(&mut doc, 55.0);
    let e0 = Extrude::distance(pr.clone(), "40 mm", OpTarget::default());
    let e = add(&mut doc, FeatureBox::new(e0.clone()));
    // The front top edge: Bound { Lateral(front line), EndCap#0 }.
    let fil = fillet(&mut doc, body0(e), [50.0, 0.0, 40.0]);
    assert_eq!(health(&doc, fil), Health::Ok);
    let edits: Vec<(&str, ExtentSide, f64)> = vec![
        ("distance", side(dist(80.0), "0 deg"), 80.0),
        ("taper", side(dist(40.0), "3 deg"), 40.0),
        ("to plane", side(to(&plane, 0.0, false), "0 deg"), 55.0),
        (
            "to face",
            side(
                to(&face_near(&doc, fence, [202.5, 2.5, 70.0]), 0.0, false),
                "0 deg",
            ),
            70.0,
        ),
        ("all", side(Extent::All, "0 deg"), 71.0),
        (
            "tapered to plane",
            side(to(&plane, 0.0, false), "2 deg"),
            55.0,
        ),
        ("back", side(dist(40.0), "0 deg"), 40.0),
    ];
    for (what, s1, z) in edits {
        let mut ed = e0.clone();
        ed.side1 = s1;
        edit(&mut doc, e, FeatureBox::new(ed));
        assert_eq!(health(&doc, e), Health::Ok, "{what}");
        assert_eq!(
            health(&doc, fil),
            Health::Ok,
            "{what}: {:?}",
            health(&doc, fil)
        );
        let (q, fp) = fillet_resolution(&doc, fil);
        assert_eq!(q, MatchQuality::Exact, "{what}");
        assert!((fp.centroid.z - z).abs() < 1e-6, "{what}: {fp:?}");
        assert!(fp.centroid.y.abs() < 3.0, "{what}: the front edge: {fp:?}");
    }
    // Two sides: the bottom front edge is on EndCap#1; editing side 2 keeps it.
    let mut two = e0.clone();
    two.direction = Direction::TwoSides;
    two.side2 = Some(side(dist(10.0), "0 deg"));
    edit(&mut doc, e, FeatureBox::new(two.clone()));
    let fil2 = fillet(&mut doc, body0(e), [50.0, 0.0, -10.0]);
    assert_eq!(health(&doc, fil2), Health::Ok);
    for (s2, z) in [
        (side(dist(20.0), "0 deg"), -20.0),
        (side(Extent::All, "0 deg"), -51.0),
    ] {
        let mut ed = two.clone();
        ed.side2 = Some(s2);
        edit(&mut doc, e, FeatureBox::new(ed));
        assert_eq!(health(&doc, fil2), Health::Ok, "{:?}", health(&doc, fil2));
        let (q, fp) = fillet_resolution(&doc, fil2);
        assert_eq!(q, MatchQuality::Exact);
        assert!((fp.centroid.z - z).abs() < 1e-6, "{fp:?}");
    }
    // Tapering only side 2 un-merges the two sides' coplanar laterals (merged names, primary
    // side 0, naming note §5.2): DOC-06 resolves to the closest part with a warning.
    let mut ed = two.clone();
    ed.side2 = Some(side(dist(20.0), "-3 deg"));
    edit(&mut doc, e, FeatureBox::new(ed));
    let Health::Warning(w) = health(&doc, fil2) else {
        panic!("{:?}", health(&doc, fil2));
    };
    assert!(w[0].message.contains("merge"), "{w:?}");
    let (_, fp) = fillet_resolution(&doc, fil2);
    assert!((fp.centroid.z + 20.0).abs() < 1e-6, "{fp:?}");
}

#[test]
fn dw5_second_side_faces_are_named_side_1() {
    let mut doc = Document::new("side-names");
    let pr = rect(&mut doc);
    let mut e = extrude(pr);
    e.direction = Direction::TwoSides;
    e.side1 = side(dist(10.0), "2 deg");
    e.side2 = Some(side(dist(5.0), "2 deg"));
    let (id, _) = ok(&mut doc, e);
    let n = doc.evaluated().naming(body0(id)).unwrap();
    let text: Vec<String> = n
        .list(TopoKind::Face)
        .unwrap()
        .iter()
        .map(|t| format!("{:?}", t.segments()[0]))
        .collect();
    let joined = text.join("\n");
    for want in ["EndCap", "Lateral"] {
        assert!(joined.contains(want), "{joined}");
    }
    // Tapered sides don't merge: 4 laterals per side + 2 end caps.
    assert_eq!(text.len(), 10, "{joined}");
    assert_eq!(
        text.iter().filter(|t| t.contains("index: 1")).count(),
        5,
        "side-1 roles (4 laterals, 1 end cap): {joined}"
    );
}

// ---- proptest (S04 Testing) -------------------------------------------------------------------

mod prop {
    use proptest::prelude::*;

    use super::*;

    proptest! {
        #![proptest_config(ProptestConfig { cases: 16, ..ProptestConfig::default() })]
        #[test]
        fn random_distances_and_tapers_compute_or_fail_cleanly(
            w in 1.0f64..30.0,
            h in 1.0f64..30.0,
            d in -40.0f64..40.0,
            t1 in -20.0f64..20.0,
            t2 in -20.0f64..20.0,
            two in any::<bool>(),
        ) {
            let mut doc = Document::new("prop");
            let s = add(&mut doc, FeatureBox::new(rect_sketch(&[[0.0, 0.0, w, h]])));
            let mut e = extrude(profiles(&doc, s));
            e.side1 = side(dist(d), &format!("{t1} deg"));
            if two {
                e.direction = Direction::TwoSides;
                e.side2 = Some(side(dist(d.abs() / 2.0 + 1.0), &format!("{t2} deg")));
            }
            let id = add(&mut doc, FeatureBox::new(e));
            match health(&doc, id) {
                Health::Ok | Health::Warning(_) => {
                    let s = shape_of(&doc, body0(id));
                    valid(&s);
                    prop_assert!(volume(&s) > 0.0);
                }
                Health::Error(d) => prop_assert!(!d.message.is_empty()),
                other => prop_assert!(false, "{other:?}"),
            }
        }
    }
}
