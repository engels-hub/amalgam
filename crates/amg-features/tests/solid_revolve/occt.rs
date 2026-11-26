//! The OCCT half of the SOL-03 tests: goldens `tests/golden/solid/sol03_*.toml` (Pappus
//! volumes), axis kinds, profile/axis validation, TNP-22, to-object and naming.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_brep::{BrepKernel, CheckOpts};
use amg_core::{Point2, Point3};
use amg_doc::naming::{GeomSource, KernelGeom, default_kernel, topo_ref_at};
use amg_doc::stub::SketchEntityKey;
use amg_doc::{
    BodyId, ConstructionId, Evaluated, Health, MatchQuality, ResolveError, SketchId, TopoKind,
    TopoRef,
};
use amg_features::construct::{AxisDef, ConstructAxis, ConstructPlane, PlaneDef};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{BodyOp, Fillet};
use amg_sketch::EntityId;

use super::*;

const REL: f64 = 1e-9;

/// Pappus: area × centroid radius × angle.
fn pappus(area: f64, r: f64, angle: f64) -> f64 {
    area * r * angle
}

/// The rectangle x 10..30, z 0..20 of TNP-22 (area 400, centroid radius 20).
const RECT: [f64; 4] = [10.0, 0.0, 30.0, 20.0];
const RECT_A: f64 = 400.0;
const RECT_R: f64 = 20.0;

/// A sketch on XZ (u = X, v = Z) with axis-aligned rectangles and extra lines; returns the
/// sketch feature id and the line ids.
fn xz_sketch(
    doc: &mut Document,
    rects: &[[f64; 4]],
    lines: &[[f64; 4]],
) -> (FeatureId, Vec<EntityId>) {
    let mut f = SketchFeature::new(PlaneInput::OriginXZ);
    let ids = f
        .edit_sketch(|s| {
            for r in rects {
                amg_sketch::build::rectangle_2pt(
                    s,
                    Point2::new(r[0], r[1]),
                    Point2::new(r[2], r[3]),
                )?;
            }
            lines
                .iter()
                .map(|l| s.add_line(Point2::new(l[0], l[1]), Point2::new(l[2], l[3])))
                .collect::<Result<Vec<_>, _>>()
        })
        .unwrap();
    (add(doc, FeatureBox::new(f)), ids)
}

/// A closed polygon on XZ.
fn xz_polygon(doc: &mut Document, pts: &[[f64; 2]]) -> FeatureId {
    let mut f = SketchFeature::new(PlaneInput::OriginXZ);
    f.edit_sketch(|s| {
        let n = pts.len();
        for i in 0..n {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            s.add_line(Point2::new(a[0], a[1]), Point2::new(b[0], b[1]))?;
        }
        Ok(())
    })
    .unwrap();
    add(doc, FeatureBox::new(f))
}

fn rect_profiles(doc: &mut Document) -> Vec<EntityRef> {
    let (s, _) = xz_sketch(doc, &[RECT], &[]);
    profiles(doc, s)
}

#[track_caller]
fn ok(doc: &Document, f: FeatureId) {
    assert_eq!(health(doc, f), Health::Ok, "feature {f:?}");
}

/// Adds a revolve and returns it with its body (checked valid).
#[track_caller]
fn revolve(doc: &mut Document, r: Revolve) -> (FeatureId, amg_brep::Shape) {
    let f = add(doc, FeatureBox::new(r));
    ok(doc, f);
    let s = brep(doc, body0(f));
    assert!(kernel().check(&s, &CheckOpts::default()).valid, "valid");
    (f, s)
}

#[track_caller]
fn golden(case: &str, s: &amg_brep::Shape) {
    check_golden_as("sol03", REL, case, s);
}

fn angle(profiles: Vec<EntityRef>, axis: RefInput, a: &str) -> Revolve {
    Revolve::angle(profiles, axis, a, OpTarget::default())
}

fn two_sides(profiles: Vec<EntityRef>, a1: &str, a2: &str) -> Revolve {
    let mut r = angle(profiles, z_axis(), a1);
    r.direction = Direction::TwoSides;
    r.side2 = Some(ExtentSide::distance(ex(a2)));
    r
}

fn to_object(profiles: Vec<EntityRef>, target: EntityRef, offset: &str) -> Revolve {
    let mut r = angle(profiles, z_axis(), "90 deg");
    r.side1.extent = Extent::ToObject {
        target,
        offset: ex(offset),
        extend: false,
    };
    r
}

#[track_caller]
fn assert_bbox(s: &amg_brep::Shape, min: [f64; 3], max: [f64; 3]) {
    let b = s.bbox();
    for i in 0..3 {
        assert!(
            (b.min.to_array()[i] - min[i]).abs() < 1e-6,
            "bbox min {b:?}"
        );
        assert!(
            (b.max.to_array()[i] - max[i]).abs() < 1e-6,
            "bbox max {b:?}"
        );
    }
}

/// `$pick`: the entity of `kind` whose centroid is nearest to `near` (no ties).
fn pick(ev: &Evaluated, body: BodyId, kind: TopoKind, near: [f64; 3], key: u32) -> TopoRef {
    let geom = KernelGeom::new(ev, default_kernel());
    let near = Point3::from_array(near);
    let mut found: Vec<(f64, u32)> = GeomSource::all(&geom, body, kind)
        .into_iter()
        .map(|(i, fp)| ((fp.centroid - near).length(), i))
        .collect();
    found.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert!(
        found.len() > 1 && found[1].0 - found[0].0 > 1e-6,
        "tied pick near {near:?}"
    );
    let mut r = topo_ref_at(ev, default_kernel(), body, kind, found[0].1).unwrap();
    r.key = key;
    r
}

fn inputs(doc: &Document, f: FeatureId) -> Vec<Dependency> {
    doc.design()
        .timeline()
        .node(f)
        .unwrap()
        .1
        .feature
        .get()
        .inputs()
}

fn construction(id: FeatureId) -> EntityRef {
    EntityRef::Construction(ConstructionId(id))
}

// ---- Done-when 1: Pappus goldens --------------------------------------------------------------

#[test]
fn dw1_rectangle_90() {
    let mut doc = Document::new("rect90");
    let p = rect_profiles(&mut doc);
    let (_, s) = revolve(&mut doc, angle(p, z_axis(), "90 deg"));
    assert_rel(volume(&s), pappus(RECT_A, RECT_R, FRAC_PI_2), REL);
    assert_eq!(faces(&s), 6);
    // Counter-clockwise about +Z: from +X towards +Y.
    assert_bbox(&s, [0.0, 0.0, 0.0], [30.0, 30.0, 20.0]);
    golden("rect_90", &s);
}

#[test]
fn dw1_rectangle_360_full_and_angle() {
    let mut doc = Document::new("rect360");
    let p = rect_profiles(&mut doc);
    let (_, s) = revolve(
        &mut doc,
        Revolve::full(p.clone(), z_axis(), OpTarget::default()),
    );
    assert_rel(volume(&s), pappus(RECT_A, RECT_R, TAU), REL);
    assert_eq!(faces(&s), 4, "no caps");
    golden("rect_360", &s);
    // `Distance(360°)` is the same full turn.
    let (_, s2) = revolve(&mut doc, angle(p, z_axis(), "360 deg"));
    assert_rel(volume(&s2), pappus(RECT_A, RECT_R, TAU), REL);
    assert_eq!(faces(&s2), 4);
}

#[test]
fn dw1_triangle_with_an_edge_on_the_axis_is_a_cone() {
    let mut doc = Document::new("cone");
    let s = xz_polygon(&mut doc, &[[0.0, 0.0], [10.0, 0.0], [0.0, 20.0]]);
    let p = profiles(&doc, s);
    let (_, sh) = revolve(&mut doc, Revolve::full(p, z_axis(), OpTarget::default()));
    // π r² h / 3 = Pappus with area 100 at radius 10/3.
    let cone = PI * 100.0 * 20.0 / 3.0;
    assert_rel(volume(&sh), cone, REL);
    assert_rel(pappus(100.0, 10.0 / 3.0, TAU), cone, 1e-12);
    assert_eq!(
        faces(&sh),
        2,
        "base disc and cone; the edge on the axis sweeps nothing"
    );
    golden("cone", &sh);
}

#[test]
fn dw1_two_sides_30_60() {
    let mut doc = Document::new("two");
    let p = rect_profiles(&mut doc);
    let (_, s) = revolve(&mut doc, two_sides(p, "30 deg", "60 deg"));
    assert_rel(volume(&s), pappus(RECT_A, RECT_R, FRAC_PI_2), REL);
    // From −60° to +30°.
    let (s30, c30) = 30f64.to_radians().sin_cos();
    let s60 = 60f64.to_radians().sin();
    let b = s.bbox();
    assert!(
        (b.max.y - 30.0 * s30).abs() < 1e-6 && (b.min.y + 30.0 * s60).abs() < 1e-6,
        "{b:?}"
    );
    assert!((b.min.x - 10.0 * c30.min(0.5)).abs() < 1e-6, "{b:?}");
    assert_eq!(faces(&s), 6, "the two sides merge into one block");
    golden("two_sides_30_60", &s);
}

#[test]
fn dw1_symmetric_90_half_and_whole() {
    let mut doc = Document::new("sym");
    let p = rect_profiles(&mut doc);
    let mut r = angle(p.clone(), z_axis(), "90 deg");
    r.direction = Direction::Symmetric {
        whole_length: false,
    };
    let (_, s) = revolve(&mut doc, r.clone());
    assert_rel(volume(&s), pappus(RECT_A, RECT_R, FRAC_PI_2), REL);
    let b = s.bbox();
    assert!(
        (b.max.y + b.min.y).abs() < 1e-9,
        "symmetric about XZ: {b:?}"
    );
    assert!((b.max.y - 30.0 * 45f64.to_radians().sin()).abs() < 1e-6);
    assert_eq!(faces(&s), 6);
    golden("symmetric_90", &s);
    r.direction = Direction::Symmetric { whole_length: true };
    let (_, w) = revolve(&mut doc, r);
    assert_rel(volume(&w), pappus(RECT_A, RECT_R, PI), REL);
}

#[test]
fn dw1_axis_sketch_line() {
    // A line x = 5 in the profile's own sketch, directed +Z: centroid radius 15.
    let mut doc = Document::new("line");
    let (s, lines) = xz_sketch(&mut doc, &[RECT], &[[5.0, -10.0, 5.0, 30.0]]);
    let p = profiles(&doc, s);
    let axis = RefInput::Ref(EntityRef::SketchCurve {
        sketch: SketchId(s),
        curve: SketchEntityKey(u64::from(lines[0].0)),
    });
    let (rev, sh) = revolve(&mut doc, angle(p, axis, "90 deg"));
    assert_rel(volume(&sh), pappus(RECT_A, 15.0, FRAC_PI_2), REL);
    assert!(
        inputs(&doc, rev).contains(&Dependency::Entity(EntityRef::SketchCurve {
            sketch: SketchId(s),
            curve: SketchEntityKey(u64::from(lines[0].0)),
        }))
    );
    golden("axis_sketch_line", &sh);
}

#[test]
fn dw1_axis_body_edge() {
    // A 5 × 5 × 40 box whose vertical edge lies on the Z axis.
    let mut doc = Document::new("edge");
    let (_, bx) = extrude_rects(
        &mut doc,
        &[[-5.0, 0.0, 0.0, 5.0]],
        40.0,
        BodyOp::NewBody,
        vec![],
    );
    let edge = pick(
        doc.evaluated(),
        body0(bx),
        TopoKind::Edge,
        [0.0, 0.0, 20.0],
        1,
    );
    let p = rect_profiles(&mut doc);
    let (_, sh) = revolve(
        &mut doc,
        angle(p, RefInput::Ref(EntityRef::Topo(edge)), "90 deg"),
    );
    assert_rel(volume(&sh), pappus(RECT_A, RECT_R, FRAC_PI_2), REL);
    golden("axis_body_edge", &sh);
}

#[test]
fn dw1_axis_construction_axis() {
    // The intersection of YZ and XZ: directed YZ.n × XZ.n = X × −Y = −Z, so 90° goes to −Y.
    let mut doc = Document::new("con");
    let ax = add(
        &mut doc,
        FeatureBox::new(ConstructAxis {
            def: AxisDef::ThroughTwoPlanes {
                a: RefInput::Origin(OriginEntity::YZPlane),
                b: RefInput::Origin(OriginEntity::XZPlane),
            },
        }),
    );
    ok(&doc, ax);
    let p = rect_profiles(&mut doc);
    let (_, sh) = revolve(
        &mut doc,
        angle(p, RefInput::Ref(construction(ax)), "90 deg"),
    );
    assert_rel(volume(&sh), pappus(RECT_A, RECT_R, FRAC_PI_2), REL);
    assert_bbox(&sh, [0.0, -30.0, 0.0], [30.0, 0.0, 20.0]);
    golden("axis_construction", &sh);
}

#[test]
fn dw1_origin_x_axis_and_flip() {
    // About X: the rectangle x 10..30, z 2..12 sweeps a ring of radius 2..12 around X.
    let mut doc = Document::new("x");
    let (s, _) = xz_sketch(&mut doc, &[[10.0, 2.0, 30.0, 12.0]], &[]);
    let p = profiles(&doc, s);
    let x = RefInput::Origin(OriginEntity::XAxis);
    let (_, sh) = revolve(&mut doc, angle(p.clone(), x.clone(), "180 deg"));
    assert_rel(volume(&sh), pappus(200.0, 7.0, PI), REL);
    golden("axis_origin_x_180", &sh);
    // Flip about Z: clockwise, into −Y.
    let p = rect_profiles(&mut doc);
    let mut r = angle(p, z_axis(), "90 deg");
    r.flip = true;
    let (_, f) = revolve(&mut doc, r);
    assert_rel(volume(&f), pappus(RECT_A, RECT_R, FRAC_PI_2), REL);
    assert_bbox(&f, [0.0, -30.0, 0.0], [30.0, 0.0, 20.0]);
}

#[test]
fn dw1_revolve_cut_groove_into_a_revolved_cylinder() {
    let mut doc = Document::new("cut");
    let (s, _) = xz_sketch(&mut doc, &[[0.0, 0.0, 30.0, 20.0]], &[]);
    let p = profiles(&doc, s);
    let (cyl, _) = revolve(&mut doc, Revolve::full(p, z_axis(), OpTarget::default()));
    let (g, _) = xz_sketch(&mut doc, &[[25.0, 5.0, 30.0, 15.0]], &[]);
    let gp = profiles(&doc, g);
    let cut = add(
        &mut doc,
        FeatureBox::new(Revolve::full(
            gp,
            z_axis(),
            OpTarget {
                op: BodyOp::Cut,
                participants: vec![body0(cyl)],
            },
        )),
    );
    ok(&doc, cut);
    let sh = brep(&doc, body0(cyl));
    assert!(kernel().check(&sh, &CheckOpts::default()).valid);
    let want = PI * 900.0 * 20.0 - pappus(50.0, 27.5, TAU);
    assert_rel(volume(&sh), want, REL);
    golden("cut_groove", &sh);
}

// ---- Done-when 2: a profile crossing the axis -------------------------------------------------

#[track_caller]
fn error(doc: &Document, f: FeatureId) -> amg_doc::Diagnostic {
    let Health::Error(d) = health(doc, f) else {
        panic!("{:?}", health(doc, f));
    };
    d
}

#[test]
fn dw2_a_profile_crossing_the_axis_fails_naming_the_profile() {
    let mut doc = Document::new("cross");
    let (s, _) = xz_sketch(
        &mut doc,
        &[[40.0, 0.0, 60.0, 10.0], [-10.0, 0.0, 10.0, 20.0]],
        &[],
    );
    let p = profiles(&doc, s);
    assert_eq!(p.len(), 2);
    let r = add(
        &mut doc,
        FeatureBox::new(angle(p.clone(), z_axis(), "90 deg")),
    );
    let d = error(&doc, r);
    assert!(d.message.contains("crosses the axis"), "{}", d.message);
    // The crossing region is the one around the origin.
    let g = doc.evaluated().sketch(SketchId(s)).unwrap();
    let crossing = p
        .iter()
        .position(|r| {
            let EntityRef::Profile { profile, .. } = r else {
                return false;
            };
            let c = g
                .profiles
                .get(amg_sketch::ProfileId(profile.0))
                .unwrap()
                .fingerprint
                .centroid;
            c.x.abs() < 1.0
        })
        .unwrap();
    assert!(
        d.message
            .contains(&format!("profile {} of 2", crossing + 1)),
        "{}",
        d.message
    );
    assert!(d.entities.contains(&p[crossing]), "{:?}", d.entities);
    assert!(!d.entities.contains(&p[1 - crossing]), "{:?}", d.entities);
    // Moving the axis out of the profile (an X-offset sketch line) fixes it.
    let (s2, lines) = xz_sketch(
        &mut doc,
        &[[-10.0, 0.0, 10.0, 20.0]],
        &[[-15.0, 0.0, -15.0, 20.0]],
    );
    let p2 = profiles(&doc, s2);
    let axis = RefInput::Ref(EntityRef::SketchCurve {
        sketch: SketchId(s2),
        curve: SketchEntityKey(u64::from(lines[0].0)),
    });
    let (_, sh) = revolve(&mut doc, angle(p2, axis, "90 deg"));
    assert_rel(volume(&sh), pappus(400.0, 15.0, FRAC_PI_2), REL);
}

#[test]
fn dw2_an_axis_through_the_profile_or_perpendicular_to_it_fails() {
    let mut doc = Document::new("perp");
    // A rectangle on XY revolved about Z: the axis is perpendicular to the profile plane.
    let (s, _) = extrude_rects(
        &mut doc,
        &[[10.0, 0.0, 30.0, 20.0]],
        1.0,
        BodyOp::NewBody,
        vec![],
    );
    let p = profiles(&doc, s);
    let r = add(
        &mut doc,
        FeatureBox::new(angle(p.clone(), z_axis(), "90 deg")),
    );
    let d = error(&doc, r);
    assert!(d.message.contains("perpendicular"), "{}", d.message);
    // The X axis lies in the profile plane along the rectangle's edge y = 0: valid.
    let r2 = add(
        &mut doc,
        FeatureBox::new(angle(p, RefInput::Origin(OriginEntity::XAxis), "90 deg")),
    );
    ok(&doc, r2);
}

// ---- Done-when 3: TNP-22 ----------------------------------------------------------------------

#[test]
fn dw3_tnp22_editing_to_360_makes_the_cap_edge_not_found() {
    let mut doc = Document::new("tnp22");
    let p = rect_profiles(&mut doc);
    let (rev, _) = revolve(&mut doc, angle(p.clone(), z_axis(), "270 deg"));
    // The edge between the end cap and the outer lateral, at (0, −30, z).
    let e = pick(
        doc.evaluated(),
        body0(rev),
        TopoKind::Edge,
        [0.0, -30.0, 10.0],
        1,
    );
    let fil = add(&mut doc, FeatureBox::new(Fillet::constant(vec![e], "1 mm")));
    ok(&doc, fil);
    edit(
        &mut doc,
        rev,
        FeatureBox::new(angle(p, z_axis(), "360 deg")),
    );
    ok(&doc, rev);
    let node = doc.evaluated().node(fil).unwrap();
    assert!(
        matches!(
            node.resolutions[0].result,
            Err(ResolveError::NotFound {
                kind: TopoKind::Edge,
                ..
            })
        ),
        "{:?}",
        node.resolutions[0]
    );
    assert!(matches!(node.health, Health::Error(_)), "{:?}", node.health);
}

#[test]
fn an_angle_edit_keeps_the_cap_edge_exact() {
    let mut doc = Document::new("names");
    let p = rect_profiles(&mut doc);
    let (rev, _) = revolve(&mut doc, angle(p.clone(), z_axis(), "90 deg"));
    // The outer edge of the end cap at 90° is at (0, 30, z).
    let e = pick(
        doc.evaluated(),
        body0(rev),
        TopoKind::Edge,
        [0.0, 30.0, 10.0],
        1,
    );
    let fil = add(&mut doc, FeatureBox::new(Fillet::constant(vec![e], "1 mm")));
    ok(&doc, fil);
    for a in ["120 deg", "45 deg"] {
        edit(
            &mut doc,
            rev,
            FeatureBox::new(angle(p.clone(), z_axis(), a)),
        );
        ok(&doc, fil);
        let rec = &doc.evaluated().node(fil).unwrap().resolutions[0];
        let res = rec.result.as_ref().expect("resolved");
        assert_eq!(res[0].quality, MatchQuality::Exact, "{a}: {rec:?}");
    }
    // Two sides: the side-2 end cap is `EndCap#1`, its edge survives a side-1 edit.
    let (rev2, _) = revolve(&mut doc, two_sides(p.clone(), "30 deg", "60 deg"));
    let (s60, c60) = 60f64.to_radians().sin_cos();
    let e2 = pick(
        doc.evaluated(),
        body0(rev2),
        TopoKind::Edge,
        [30.0 * c60, -30.0 * s60, 10.0],
        1,
    );
    let fil2 = add(
        &mut doc,
        FeatureBox::new(Fillet::constant(vec![e2], "1 mm")),
    );
    ok(&doc, fil2);
    edit(
        &mut doc,
        rev2,
        FeatureBox::new(two_sides(p, "50 deg", "60 deg")),
    );
    ok(&doc, fil2);
    let rec = &doc.evaluated().node(fil2).unwrap().resolutions[0];
    assert_eq!(rec.result.as_ref().unwrap()[0].quality, MatchQuality::Exact);
}

// ---- Done-when 4: to a plane through the axis -------------------------------------------------

#[test]
fn dw4_revolve_to_a_plane_through_the_axis_at_75_equals_the_75_angle() {
    let mut doc = Document::new("to75");
    let plane = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::AtAngle {
                line: z_axis(),
                zero: Some(RefInput::Origin(OriginEntity::XZPlane)),
                angle: ex("75 deg"),
            },
        }),
    );
    ok(&doc, plane);
    let p = rect_profiles(&mut doc);
    let (_, by_angle) = revolve(&mut doc, angle(p.clone(), z_axis(), "75 deg"));
    let (_, by_plane) = revolve(&mut doc, to_object(p, construction(plane), "0 mm"));
    let want = pappus(RECT_A, RECT_R, 75f64.to_radians());
    assert_rel(volume(&by_angle), want, REL);
    assert_rel(volume(&by_plane), volume(&by_angle), REL);
    assert_rel(
        kernel().mass_props(&by_plane).unwrap().area,
        kernel().mass_props(&by_angle).unwrap().area,
        REL,
    );
    let (a, b) = (by_angle.bbox(), by_plane.bbox());
    assert!((a.min - b.min).length() < 1e-9 && (a.max - b.max).length() < 1e-9);
    assert_eq!(faces(&by_plane), faces(&by_angle));
    golden("to_plane_75", &by_plane);
}

// ---- to object: split targets -----------------------------------------------------------------

/// The rectangle x 10..30, z 2..18 (area 320) and a box x 0..40, y −40..−5, z 0..20 that the
/// counter-clockwise turn about Z enters at its face x = 0, i.e. at 270°.
fn box_scene(doc: &mut Document, h: f64) -> (Vec<EntityRef>, BodyId) {
    let (_, bx) = extrude_rects(doc, &[[0.0, -40.0, 40.0, -5.0]], h, BodyOp::NewBody, vec![]);
    let (s, _) = xz_sketch(doc, &[[10.0, 2.0, 30.0, 18.0]], &[]);
    (profiles(doc, s), body0(bx))
}

#[test]
fn to_object_body_face_and_vertex_stop_at_270() {
    let mut doc = Document::new("to-body");
    let (p, bx) = box_scene(&mut doc, 20.0);
    let want = pappus(320.0, 20.0, 1.5 * PI);
    let (rb, by_body) = revolve(&mut doc, to_object(p.clone(), EntityRef::Body(bx), "0 mm"));
    assert_rel(volume(&by_body), want, REL);
    assert!(inputs(&doc, rb).contains(&Dependency::Body(bx)));
    golden("to_body_270", &by_body);
    let face = pick(doc.evaluated(), bx, TopoKind::Face, [0.0, -22.5, 10.0], 1);
    let (_, by_face) = revolve(
        &mut doc,
        to_object(p.clone(), EntityRef::Topo(face), "0 mm"),
    );
    assert_rel(volume(&by_face), want, REL);
    let v = pick(doc.evaluated(), bx, TopoKind::Vertex, [0.0, -40.0, 0.0], 1);
    let (_, by_vertex) = revolve(&mut doc, to_object(p, EntityRef::Topo(v), "0 mm"));
    assert_rel(volume(&by_vertex), want, REL);
}

#[test]
fn to_object_a_face_that_misses_the_profile_fails_target_missed() {
    let mut doc = Document::new("missed");
    // The box is only 1 mm high: its face x = 0 doesn't cover the profile's z 2..18.
    let (p, bx) = box_scene(&mut doc, 1.0);
    let face = pick(doc.evaluated(), bx, TopoKind::Face, [0.0, -22.5, 0.5], 1);
    let r = add(
        &mut doc,
        FeatureBox::new(to_object(p, EntityRef::Topo(face.clone()), "0 mm")),
    );
    let d = error(&doc, r);
    assert!(d.message.contains("does not cover"), "{}", d.message);
    assert!(
        d.entities.contains(&EntityRef::Topo(face)),
        "{:?}",
        d.entities
    );
}

#[test]
fn bad_angles_and_options_fail_cleanly() {
    let mut doc = Document::new("bad");
    let p = rect_profiles(&mut doc);
    for (a, what) in [("0 deg", "zero"), ("400 deg", "at most 360")] {
        let r = add(&mut doc, FeatureBox::new(angle(p.clone(), z_axis(), a)));
        let d = error(&doc, r);
        assert!(d.message.contains(what), "{a}: {}", d.message);
    }
    let r = add(
        &mut doc,
        FeatureBox::new(two_sides(p.clone(), "200 deg", "200 deg")),
    );
    assert!(error(&doc, r).message.contains("exceed 360"));
    // A negative angle reverses the side.
    let (_, neg) = revolve(&mut doc, angle(p.clone(), z_axis(), "-90 deg"));
    assert_bbox(&neg, [0.0, -30.0, 0.0], [30.0, 0.0, 20.0]);
    let mut thin = angle(p, z_axis(), "90 deg");
    thin.thin = Some(amg_features::solid::ThinSpec {
        thickness: ex("1 mm"),
        side: amg_features::solid::ThinSide::Side1,
    });
    let r = add(&mut doc, FeatureBox::new(thin));
    assert!(error(&doc, r).message.contains("thin revolve"));
}
