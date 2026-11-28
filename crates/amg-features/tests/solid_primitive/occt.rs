//! The OCCT half of the SOL-04 tests: goldens `tests/golden/solid/sol04_*.toml` (analytic
//! volume and area), placement on planar faces, names, validation and pattern seeds.

use std::f64::consts::PI;

use amg_brep::{BrepKernel, CheckOpts};
use amg_doc::naming::{GeomSource, KernelGeom, NameSeg, default_kernel, topo_ref_at};
use amg_doc::{Evaluated, Health, MatchQuality};
use amg_features::solid::{
    Extrude, Fillet, Pattern, PatternDir, PatternKind, PatternObjects, Spacing,
};

use super::*;

const REL: f64 = 1e-9;

#[track_caller]
fn ok(doc: &Document, f: FeatureId) {
    assert_eq!(health(doc, f), Health::Ok, "feature {f:?}");
}

/// Adds a feature and returns it with its first body (checked valid).
#[track_caller]
fn solid(doc: &mut Document, f: FeatureBox) -> (FeatureId, amg_brep::Shape) {
    let id = add(doc, f);
    ok(doc, id);
    let s = brep(doc, body0(id));
    assert_valid(&s);
    (id, s)
}

#[track_caller]
fn assert_valid(s: &amg_brep::Shape) {
    assert!(kernel().check(s, &CheckOpts::default()).valid, "valid");
}

/// Analytic volume, area and face count, then the golden.
#[track_caller]
fn check(case: &str, s: &amg_brep::Shape, v: f64, a: f64, n: u32) {
    let m = kernel().mass_props(s).unwrap();
    assert_rel(m.volume, v, REL);
    assert_rel(m.area, a, REL);
    assert_eq!(faces(s), n, "{case}: faces");
    check_golden_as("sol04", REL, case, s);
}

fn centroid(s: &amg_brep::Shape) -> Point3 {
    kernel().mass_props(s).unwrap().centroid
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

/// The entity of `kind` whose centroid is nearest to `near` (no ties).
#[track_caller]
fn pick(ev: &Evaluated, body: BodyId, kind: TopoKind, near: [f64; 3], key: u32) -> TopoRef {
    let geom = KernelGeom::new(ev, default_kernel());
    let near = Point3::from_array(near);
    let mut found: Vec<(f64, u32)> = GeomSource::all(&geom, body, kind)
        .into_iter()
        .map(|(i, fp)| ((fp.centroid - near).length(), i))
        .collect();
    found.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert!(
        found.len() == 1 || found[1].0 - found[0].0 > 1e-6,
        "tied pick near {near:?}"
    );
    let mut r = topo_ref_at(ev, default_kernel(), body, kind, found[0].1).unwrap();
    r.key = key;
    r
}

/// The role (`tag#index`) of a face name that is one `Gen` segment of `feature` without a
/// source.
#[track_caller]
fn role_of(name: &TopoName, feature: FeatureId) -> String {
    match name.segments() {
        [
            NameSeg::Gen {
                feature: f,
                role,
                from: None,
            },
        ] if *f == feature => format!("{}#{}", role.tag, role.index),
        other => panic!("not a primitive face name: {other:?}"),
    }
}

/// Every face of `body` as `(role, centroid)`, sorted by role.
fn face_roles(doc: &Document, feature: FeatureId) -> Vec<(String, Point3)> {
    let body = body0(feature);
    let ev = doc.evaluated();
    let geom = KernelGeom::new(ev.as_ref(), default_kernel());
    let mut v: Vec<(String, Point3)> = GeomSource::all(&geom, body, TopoKind::Face)
        .into_iter()
        .map(|(i, fp)| {
            let r = topo_ref_at(ev.as_ref(), default_kernel(), body, TopoKind::Face, i).unwrap();
            (role_of(&r.name, feature), fp.centroid)
        })
        .collect();
    v.sort_by(|a, b| a.0.cmp(&b.0));
    v
}

fn origin_plane(e: OriginEntity) -> PrimitiveFrame {
    PrimitiveFrame::on(RefInput::Origin(e))
}

// ---- Done-when 1: goldens ---------------------------------------------------------------------

#[test]
fn dw1_box() {
    let mut doc = Document::new("box");
    let f = BoxPrim::new(
        PrimitiveFrame::xy(),
        ["10 mm", "20 mm", "30 mm"],
        OpTarget::default(),
    );
    let (_, s) = solid(&mut doc, FeatureBox::new(f));
    check("box", &s, 6000.0, 2200.0, 6);
    assert_bbox(&s, [0.0, 0.0, 0.0], [10.0, 20.0, 30.0]);
}

#[test]
fn dw1_cylinder() {
    let mut doc = Document::new("cylinder");
    let f = CylinderPrim::new(PrimitiveFrame::xy(), "5 mm", "10 mm", OpTarget::default());
    let (_, s) = solid(&mut doc, FeatureBox::new(f));
    check("cylinder", &s, 250.0 * PI, 150.0 * PI, 3);
    assert_bbox(&s, [-5.0, -5.0, 0.0], [5.0, 5.0, 10.0]);
}

#[test]
fn dw1_sphere() {
    let mut doc = Document::new("sphere");
    let f = SpherePrim::new(PrimitiveFrame::xy(), "5 mm", OpTarget::default());
    let (_, s) = solid(&mut doc, FeatureBox::new(f));
    check("sphere", &s, 500.0 * PI / 3.0, 100.0 * PI, 1);
    assert_bbox(&s, [-5.0, -5.0, -5.0], [5.0, 5.0, 5.0]);
}

#[test]
fn dw1_torus() {
    let mut doc = Document::new("torus");
    let f = TorusPrim::new(PrimitiveFrame::xy(), "10 mm", "2 mm", OpTarget::default());
    let (_, s) = solid(&mut doc, FeatureBox::new(f));
    // 2π²Rr² and 4π²Rr.
    check("torus", &s, 80.0 * PI * PI, 80.0 * PI * PI, 1);
    assert_bbox(&s, [-12.0, -12.0, -2.0], [12.0, 12.0, 2.0]);
}

/// A 40 × 40 × `h` plate on XY.
fn plate(doc: &mut Document, h: f64) -> (FeatureId, FeatureId, BodyId) {
    let (s, e) = extrude_rects(doc, &[[0.0, 0.0, 40.0, 40.0]], h, BodyOp::NewBody, vec![]);
    ok(doc, e);
    (s, e, body0(e))
}

/// The construction plane `z = d`.
fn plane_at(doc: &mut Document, d: &str) -> RefInput {
    let f = add(
        doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: ex(d),
            },
        }),
    );
    ok(doc, f);
    RefInput::Ref(EntityRef::Construction(ConstructionId(f)))
}

fn op(op: BodyOp, body: BodyId) -> OpTarget {
    OpTarget {
        op,
        participants: vec![body],
    }
}

#[test]
fn dw1_box_joined_to_a_body() {
    let mut doc = Document::new("join");
    let (_, _, body) = plate(&mut doc, 10.0);
    let top = plane_at(&mut doc, "10 mm");
    let f = BoxPrim::new(
        frame(top, "5 mm", "5 mm", "0 deg"),
        ["10 mm", "10 mm", "10 mm"],
        op(BodyOp::Join, body),
    );
    let id = add(&mut doc, FeatureBox::new(f));
    ok(&doc, id);
    assert!(doc.evaluated().body(body0(id)).is_none(), "no new body");
    let s = brep(&doc, body);
    assert_valid(&s);
    // 16000 + 1000; the plate's area plus the box's four sides (its bottom and the covered
    // part of the plate cancel its top).
    check("box_join", &s, 17_000.0, 5200.0, 11);
}

#[test]
fn dw1_box_cut_from_a_body() {
    let mut doc = Document::new("cut");
    let (_, _, body) = plate(&mut doc, 10.0);
    let top = plane_at(&mut doc, "10 mm");
    // A negative height goes below the plane: a 10 × 10 × 4 pocket.
    let f = BoxPrim::new(
        frame(top, "5 mm", "5 mm", "0 deg"),
        ["10 mm", "10 mm", "-4 mm"],
        op(BodyOp::Cut, body),
    );
    let id = add(&mut doc, FeatureBox::new(f));
    ok(&doc, id);
    let s = brep(&doc, body);
    assert_valid(&s);
    check("box_cut", &s, 15_600.0, 4960.0, 11);
}

#[test]
fn a_cylinder_intersects_and_a_missed_cut_fails() {
    let mut doc = Document::new("intersect");
    let (_, _, body) = plate(&mut doc, 10.0);
    let f = CylinderPrim::new(
        frame(
            RefInput::Origin(OriginEntity::XYPlane),
            "20 mm",
            "20 mm",
            "0 deg",
        ),
        "5 mm",
        "30 mm",
        op(BodyOp::Intersect, body),
    );
    let id = add(&mut doc, FeatureBox::new(f));
    ok(&doc, id);
    let s = brep(&doc, body);
    assert_valid(&s);
    assert_rel(volume(&s), 250.0 * PI, REL);

    let miss = SpherePrim::new(
        frame(
            RefInput::Origin(OriginEntity::XYPlane),
            "200 mm",
            "0 mm",
            "0 deg",
        ),
        "5 mm",
        op(BodyOp::Cut, body),
    );
    let id = add(&mut doc, FeatureBox::new(miss));
    let h = format!("{:?}", health(&doc, id));
    assert!(h.contains("nothing to cut"), "{h}");
}

// ---- placement ----------------------------------------------------------------------------------

#[test]
fn offsets_rotation_centering_and_negative_heights_place_the_solid() {
    let mut doc = Document::new("place");
    // XY, origin (1, 2), turned 90°: length goes along +Y, width along −X.
    let on = frame(
        RefInput::Origin(OriginEntity::XYPlane),
        "1 mm",
        "2 mm",
        "90 deg",
    );
    let mut b = BoxPrim::new(on, ["10 mm", "4 mm", "3 mm"], OpTarget::default());
    let (_, s) = solid(&mut doc, FeatureBox::new(b.clone()));
    assert_bbox(&s, [-3.0, 2.0, 0.0], [1.0, 12.0, 3.0]);
    b.centered = true;
    let (_, s) = solid(&mut doc, FeatureBox::new(b));
    assert_bbox(&s, [-1.0, -3.0, 0.0], [3.0, 7.0, 3.0]);
    // XZ: normal −Y, u = X, v = Z.
    let xz = origin_plane(OriginEntity::XZPlane);
    let b = BoxPrim::new(xz.clone(), ["10 mm", "20 mm", "30 mm"], OpTarget::default());
    let (_, s) = solid(&mut doc, FeatureBox::new(b));
    assert_bbox(&s, [0.0, -30.0, 0.0], [10.0, 0.0, 20.0]);
    let c = CylinderPrim::new(xz, "5 mm", "-10 mm", OpTarget::default());
    let (_, s) = solid(&mut doc, FeatureBox::new(c));
    assert_bbox(&s, [-5.0, 0.0, -5.0], [5.0, 10.0, 5.0]);
    assert_rel(volume(&s), 250.0 * PI, REL);
    let yz = origin_plane(OriginEntity::YZPlane);
    let t = TorusPrim::new(yz, "10 mm", "2 mm", OpTarget::default());
    let (_, s) = solid(&mut doc, FeatureBox::new(t));
    assert_bbox(&s, [-2.0, -12.0, -12.0], [2.0, 12.0, 12.0]);
}

// ---- Done-when 2: on a planar face --------------------------------------------------------------

#[test]
fn dw2_a_box_on_a_planar_face_follows_an_upstream_height_edit() {
    let mut doc = Document::new("follow");
    let (sketch, ext, body) = plate(&mut doc, 20.0);
    let top = pick(doc.evaluated(), body, TopoKind::Face, [20.0, 20.0, 20.0], 1);
    let on = frame(RefInput::Ref(EntityRef::Topo(top)), "5 mm", "7 mm", "0 deg");
    let f = FeatureBox::new(BoxPrim::new(
        on.clone(),
        ["4 mm", "6 mm", "10 mm"],
        OpTarget::default(),
    ));
    assert!(matches!(
        f.get().inputs().as_slice(),
        [Dependency::Entity(EntityRef::Topo(_))]
    ));
    let (b, s) = solid(&mut doc, f);
    let c0 = centroid(&s);
    // It sits on the face, outwards.
    assert!((c0.z - 25.0).abs() < 1e-9, "{c0:?}");
    // The anchors follow the face too (height from the footprint centre).
    let reg_anchors = anchors(&doc, &doc.design().timeline().node(b).unwrap().1.feature);
    assert!((reg_anchors[2].1.z - 20.0).abs() < 1e-9);
    assert!((reg_anchors[2].1.x - c0.x).abs() < 1e-9 && (reg_anchors[2].1.y - c0.y).abs() < 1e-9);

    for h in [80.0, 12.0] {
        let p = profiles(&doc, sketch);
        edit(
            &mut doc,
            ext,
            FeatureBox::new(Extrude::distance(
                p,
                &format!("{h} mm"),
                OpTarget::default(),
            )),
        );
        ok(&doc, ext);
        ok(&doc, b);
        let s = brep(&doc, body0(b));
        let c = centroid(&s);
        let want = Point3::new(c0.x, c0.y, h + 5.0);
        assert!((c - want).length() < 1e-9, "h {h}: {c:?}, want {want:?}");
        let rec = &doc.evaluated().node(b).unwrap().resolutions[0];
        assert_eq!(
            rec.result.as_ref().expect("resolved")[0].quality,
            MatchQuality::Exact
        );
    }
}

#[test]
fn a_cylinder_on_a_side_face_points_outwards_and_joins() {
    let mut doc = Document::new("side");
    let (_, _, body) = plate(&mut doc, 20.0);
    let side = pick(doc.evaluated(), body, TopoKind::Face, [40.0, 20.0, 10.0], 1);
    let f = CylinderPrim::new(
        PrimitiveFrame::on(RefInput::Ref(EntityRef::Topo(side))),
        "2 mm",
        "5 mm",
        OpTarget::default(),
    );
    let (_, s) = solid(&mut doc, FeatureBox::new(f));
    let b = s.bbox();
    assert!(
        (b.min.x - 40.0).abs() < 1e-6 && (b.max.x - 45.0).abs() < 1e-6,
        "{b:?}"
    );
}

#[test]
fn a_non_planar_face_is_refused_and_highlighted() {
    let mut doc = Document::new("curved");
    let c = CylinderPrim::new(PrimitiveFrame::xy(), "5 mm", "10 mm", OpTarget::default());
    let (c, _) = solid(&mut doc, FeatureBox::new(c));
    let lateral = pick(
        doc.evaluated(),
        body0(c),
        TopoKind::Face,
        [0.0, 0.0, 5.0],
        1,
    );
    let r = EntityRef::Topo(lateral);
    let f = SpherePrim::new(
        PrimitiveFrame::on(RefInput::Ref(r)),
        "1 mm",
        OpTarget::default(),
    );
    let id = add(&mut doc, FeatureBox::new(f));
    let Health::Error(d) = health(&doc, id) else {
        panic!("{:?}", health(&doc, id));
    };
    assert!(d.message.contains("plane"), "{d:?}");
    assert_eq!(d.entities.len(), 1, "{d:?}");
}

#[test]
fn bad_sizes_fail_naming_the_field() {
    let mut doc = Document::new("bad");
    let t = OpTarget::default;
    let xy = PrimitiveFrame::xy;
    let cases = [
        (
            FeatureBox::new(BoxPrim::new(xy(), ["0 mm", "1 mm", "1 mm"], t())),
            "length",
        ),
        (
            FeatureBox::new(BoxPrim::new(xy(), ["1 mm", "-1 mm", "1 mm"], t())),
            "width",
        ),
        (
            FeatureBox::new(BoxPrim::new(xy(), ["1 mm", "1 mm", "0 mm"], t())),
            "height",
        ),
        (
            FeatureBox::new(CylinderPrim::new(xy(), "-1 mm", "1 mm", t())),
            "radius",
        ),
        (
            FeatureBox::new(CylinderPrim::new(xy(), "1 mm", "0 mm", t())),
            "height",
        ),
        (
            FeatureBox::new(SpherePrim::new(xy(), "0 mm", t())),
            "radius",
        ),
        (
            FeatureBox::new(TorusPrim::new(xy(), "2 mm", "2 mm", t())),
            "minor_radius",
        ),
        (
            FeatureBox::new(TorusPrim::new(xy(), "0 mm", "1 mm", t())),
            "major_radius",
        ),
    ];
    for (f, field) in cases {
        let id = add(&mut doc, f);
        let Health::Error(d) = health(&doc, id) else {
            panic!("{field}: {:?}", health(&doc, id));
        };
        assert!(d.message.contains(field), "{field}: {d:?}");
        assert!(doc.evaluated().body(body0(id)).is_none());
    }
}

// ---- Done-when 3: names -------------------------------------------------------------------------

/// Where the centre of the box face `role` is for a box of `size` with a corner at the
/// origin.
fn face_centre(role: &str, size: [f64; 3]) -> Point3 {
    let mut c = Vec3::from_array(size) / 2.0;
    match role {
        "XMin#0" => c.x = 0.0,
        "XMax#0" => c.x = size[0],
        "YMin#0" => c.y = 0.0,
        "YMax#0" => c.y = size[1],
        "ZMin#0" => c.z = 0.0,
        "ZMax#0" => c.z = size[2],
        other => panic!("{other}"),
    }
    c
}

const BOX_ROLES: [&str; 6] = ["XMax#0", "XMin#0", "YMax#0", "YMin#0", "ZMax#0", "ZMin#0"];

#[test]
fn dw3_editing_a_box_size_keeps_the_face_names() {
    let mut doc = Document::new("names");
    let size = [10.0, 20.0, 30.0];
    let boxed =
        |s: [&str; 3]| FeatureBox::new(BoxPrim::new(PrimitiveFrame::xy(), s, OpTarget::default()));
    let (b, _) = solid(&mut doc, boxed(["10 mm", "20 mm", "30 mm"]));
    let roles = face_roles(&doc, b);
    assert_eq!(
        roles.iter().map(|r| r.0.as_str()).collect::<Vec<_>>(),
        BOX_ROLES
    );
    for (role, c) in &roles {
        assert!(
            (*c - face_centre(role, size)).length() < 1e-9,
            "{role}: {c:?}"
        );
    }
    let refs: Vec<(String, TopoRef)> = BOX_ROLES
        .iter()
        .map(|role| {
            let at = face_centre(role, size).to_array();
            let r = pick(doc.evaluated(), body0(b), TopoKind::Face, at, 1);
            ((*role).to_owned(), r)
        })
        .collect();
    // A fillet on the edge ZMax ∩ XMax, as a downstream reference.
    let edge = pick(
        doc.evaluated(),
        body0(b),
        TopoKind::Edge,
        [10.0, 10.0, 30.0],
        1,
    );
    let fil = add(
        &mut doc,
        FeatureBox::new(Fillet::constant(vec![edge.clone()], "1 mm")),
    );
    ok(&doc, fil);

    for (s, size) in [
        (["40 mm", "5 mm", "7 mm"], [40.0, 5.0, 7.0]),
        (["3 mm", "50 mm", "12 mm"], [3.0, 50.0, 12.0]),
    ] {
        edit(&mut doc, b, boxed(s));
        ok(&doc, b);
        ok(&doc, fil);
        let rec = &doc.evaluated().node(fil).unwrap().resolutions[0];
        assert_eq!(
            rec.result.as_ref().expect("the edge")[0].quality,
            MatchQuality::Exact
        );
        // The box itself, before the fillet.
        let node = doc.evaluated().node(b).unwrap();
        assert_eq!(node.health, Health::Ok);
        doc.apply(amg_doc::Command::SetSuppressed {
            ids: vec![fil],
            suppressed: true,
        })
        .unwrap();
        doc.recompute_blocking(jobs()).unwrap();
        for (role, r) in &refs {
            let got = doc.evaluated().resolve(r).expect("resolved");
            assert_eq!(got.quality, MatchQuality::Exact, "{role}");
            let geom = KernelGeom::new(doc.evaluated().as_ref(), default_kernel());
            let c = geom
                .fingerprint(got.body, got.kind, got.index)
                .unwrap()
                .centroid;
            assert!(
                (c - face_centre(role, size)).length() < 1e-9,
                "{role}: {c:?}"
            );
        }
        doc.apply(amg_doc::Command::SetSuppressed {
            ids: vec![fil],
            suppressed: false,
        })
        .unwrap();
        doc.recompute_blocking(jobs()).unwrap();
    }
}

#[test]
fn dw3_round_primitives_have_their_fixed_role_names() {
    let mut doc = Document::new("round-names");
    for height in ["10 mm", "-10 mm"] {
        let c = CylinderPrim::new(PrimitiveFrame::xy(), "5 mm", height, OpTarget::default());
        let (c, _) = solid(&mut doc, FeatureBox::new(c));
        let roles = face_roles(&doc, c);
        assert_eq!(
            roles.iter().map(|r| r.0.as_str()).collect::<Vec<_>>(),
            ["EndCap#0", "Lateral#0", "StartCap#0"]
        );
        // The start cap stays on the placement plane.
        assert!(roles[2].1.z.abs() < 1e-9, "{roles:?}");
        assert!((roles[0].1.z.abs() - 10.0).abs() < 1e-9, "{roles:?}");
    }
    // A box with a negative height names its faces in the box frame: ZMax is on the plane.
    let b = BoxPrim::new(
        PrimitiveFrame::xy(),
        ["1 mm", "1 mm", "-5 mm"],
        OpTarget::default(),
    );
    let (b, _) = solid(&mut doc, FeatureBox::new(b));
    let roles = face_roles(&doc, b);
    assert_eq!(roles[4].0, "ZMax#0");
    assert!(roles[4].1.z.abs() < 1e-9, "{roles:?}");
    assert!((roles[5].1.z + 5.0).abs() < 1e-9, "{roles:?}");
    for f in [
        FeatureBox::new(SpherePrim::new(
            PrimitiveFrame::xy(),
            "5 mm",
            OpTarget::default(),
        )),
        FeatureBox::new(TorusPrim::new(
            PrimitiveFrame::xy(),
            "10 mm",
            "2 mm",
            OpTarget::default(),
        )),
    ] {
        let (id, _) = solid(&mut doc, f);
        let roles = face_roles(&doc, id);
        assert_eq!(roles.len(), 1);
        assert_eq!(roles[0].0, "Surface#0");
    }
}

// ---- tools --------------------------------------------------------------------------------------

#[test]
fn a_primitive_cut_is_a_pattern_seed() {
    let mut doc = Document::new("seed");
    let (_, _, body) = plate(&mut doc, 10.0);
    let hole = CylinderPrim::new(
        frame(
            RefInput::Origin(OriginEntity::XYPlane),
            "5 mm",
            "5 mm",
            "0 deg",
        ),
        "2 mm",
        "10 mm",
        op(BodyOp::Cut, body),
    );
    let h = add(&mut doc, FeatureBox::new(hole));
    ok(&doc, h);
    let p = add(
        &mut doc,
        FeatureBox::new(Pattern {
            objects: PatternObjects::Features(vec![h]),
            kind: PatternKind::Rectangular {
                d1: PatternDir {
                    dir: RefInput::Origin(OriginEntity::XAxis),
                    count: ex("3"),
                    spacing: Spacing::Spacing(ex("10 mm")),
                    flip: false,
                    symmetric: false,
                },
                d2: None,
            },
            suppressed: Vec::new(),
            compute: Default::default(),
            target: None,
        }),
    );
    ok(&doc, p);
    let s = brep(&doc, body);
    assert_valid(&s);
    assert_rel(volume(&s), 16_000.0 - 3.0 * PI * 4.0 * 10.0, REL);
}
