//! The OCCT half of the SOL-09 tests: goldens `tests/golden/solid/sol09_*.toml`, names through
//! moves and copies, align coplanarity, offset / replace / delete face geometry. Small parts
//! only (BREP-16's local ops take seconds on 1000-face plates).

use amg_brep::{BrepKernel, CheckOpts, FaceId, SubKind, geom::Surface};
use amg_core::{Point2, Point3};
use amg_doc::naming::{GeomSource, KernelGeom, NameSeg, default_kernel, topo_ref_at};
use amg_doc::{
    Command, ConstructionId, Evaluated, Expr, ExprSrc, Health, MatchQuality, TopoKind, TopoRef,
};
use amg_features::construct::{ConstructPlane, PlaneDef};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::moves::IDENTITY_MATRIX;
use amg_features::solid::{BodyOp, Extrude, Fillet, OpTarget};

use super::*;

const REL: f64 = 1e-9;

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

#[track_caller]
fn golden(case: &str, s: &amg_brep::Shape) {
    check_golden_as("sol09", REL, case, s);
}

#[track_caller]
fn ok(doc: &Document, f: FeatureId) {
    assert_eq!(health(doc, f), Health::Ok, "feature {f:?}");
}

#[track_caller]
fn valid(s: &amg_brep::Shape) {
    assert!(kernel().check(s, &CheckOpts::default()).valid, "valid");
}

/// A box `[x0, y0, x1, y1]` × `0..h` as a new body.
fn block(doc: &mut Document, r: [f64; 4], h: f64) -> BodyId {
    body0(extrude_rects(doc, &[r], h, BodyOp::NewBody, vec![]).1)
}

/// A cylinder of radius `r` about Z, `0..h`, as a new body.
fn cylinder(doc: &mut Document, r: f64, h: f64) -> BodyId {
    let mut s = SketchFeature::new(PlaneInput::OriginXY);
    s.edit_sketch(|k| k.add_circle(Point2::new(0.0, 0.0), r))
        .unwrap();
    let s = add(doc, FeatureBox::new(s));
    let e = add(
        doc,
        FeatureBox::new(Extrude::distance(
            profiles(doc, s),
            &format!("{h} mm"),
            OpTarget::default(),
        )),
    );
    body0(e)
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

/// Adds `f` at the timeline position of `before` and recomputes.
fn insert_before(doc: &mut Document, before: FeatureId, f: FeatureBox) -> FeatureId {
    let at = doc.design().timeline().node(before).unwrap().0;
    let root = doc.design().root();
    let o = doc
        .apply(Command::AddFeature {
            at: Some(at),
            feature: f,
            context: root,
            name: None,
        })
        .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    created_feature(&o)
}

/// The first reference of `f`: its quality, body and current fingerprint.
fn first_resolution(
    doc: &Document,
    f: FeatureId,
) -> (MatchQuality, BodyId, amg_doc::GeomFingerprint) {
    let node = doc.evaluated().node(f).unwrap();
    let rec = &node.resolutions[0];
    let res = &rec.result.as_ref().expect("resolved")[0];
    let Some(EntityRef::Topo(t)) = rec.current.first() else {
        panic!("{rec:?}");
    };
    (res.quality, res.body, t.fingerprint.clone().unwrap())
}

#[track_caller]
fn near3(got: Point3, want: [f64; 3], tol: f64) {
    assert!(
        (got - Point3::from_array(want)).length() <= tol,
        "{got:?} vs {want:?}"
    );
}

/// The outward plane of planar face `f`.
fn face_plane(s: &amg_brep::Shape, f: FaceId) -> amg_core::Plane {
    match s.surface(f) {
        Surface::Plane(p) if s.face_reversed(f) => p.flipped(),
        Surface::Plane(p) => p,
        other => panic!("not planar: {other:?}"),
    }
}

/// The points of every vertex of face `f`.
fn face_vertices(s: &amg_brep::Shape, f: FaceId) -> Vec<Point3> {
    let mut v = Vec::new();
    for &e in s.face_edges(f) {
        for x in s.edge_vertices(e) {
            v.push(s.vertex_point(x));
        }
    }
    v
}

/// Edges used by exactly one face (the free boundary of an open body).
fn free_edges(s: &amg_brep::Shape) -> usize {
    s.edges().filter(|e| s.edge_faces(*e).len() == 1).count()
}

fn area(s: &amg_brep::Shape) -> f64 {
    kernel().mass_props(s).unwrap().area
}

// ---- Done-when 1: move keeps names, copy is instanced -----------------------------------------

#[test]
fn dw1_a_moved_body_keeps_names_and_a_fillet_ref_resolves_exact() {
    let mut doc = Document::new("move");
    let b = block(&mut doc, [0.0, 0.0, 20.0, 10.0], 10.0);
    let e = pick(doc.evaluated(), b, TopoKind::Edge, [10.0, 0.0, 10.0], 1);
    let fil = add(&mut doc, FeatureBox::new(Fillet::constant(vec![e], "2 mm")));
    let before = volume(&brep(&doc, b));
    let names_before = doc.evaluated().body(b).unwrap().naming.clone().unwrap();
    // A move inserted between the box and the fillet: the fillet still finds its edge.
    let mv = insert_before(
        &mut doc,
        fil,
        FeatureBox::new(Move::translate(vec![b], ["30 mm", "5 mm", "0 mm"])),
    );
    ok(&doc, mv);
    ok(&doc, fil);
    let (q, rb, fp) = first_resolution(&doc, fil);
    assert_eq!((q, rb), (MatchQuality::Exact, b));
    near3(fp.centroid, [40.0, 5.0, 10.0], 1e-9);
    let s = brep(&doc, b);
    assert_rel(volume(&s), before, REL);
    near3(s.bbox().min, [30.0, 5.0, 0.0], 1e-9);
    valid(&s);
    // The move itself keeps the element map (the fillet's own changes come after it).
    let names_after = doc.evaluated().body(b).unwrap().naming.clone().unwrap();
    assert_eq!(names_after.faces.len(), names_before.faces.len());
    // Editing the move into a rotation keeps the reference `Exact` too.
    let z = RefInput::Origin(OriginEntity::ZAxis);
    edit(
        &mut doc,
        mv,
        FeatureBox::new(Move::rotate(vec![b], z, "90 deg")),
    );
    ok(&doc, fil);
    let (q, _, fp) = first_resolution(&doc, fil);
    assert_eq!(q, MatchQuality::Exact);
    // (10, 0, 10) turned 90° about Z is (0, 10, 10).
    near3(fp.centroid, [0.0, 10.0, 10.0], 1e-9);
    let s = brep(&doc, b);
    assert_rel(volume(&s), before, REL);
    golden("move_rotate_filleted", &s);
}

#[test]
fn dw1_a_copy_is_a_new_body_named_instanced() {
    let mut doc = Document::new("copy");
    let b = block(&mut doc, [0.0, 0.0, 20.0, 10.0], 10.0);
    let names = doc.evaluated().body(b).unwrap().naming.clone().unwrap();
    let mv = add(
        &mut doc,
        FeatureBox::new(Move::translate(vec![b], ["30 mm", "0 mm", "0 mm"]).copied()),
    );
    ok(&doc, mv);
    let copy = BodyId {
        origin: mv,
        slot: 0,
    };
    let orig = brep(&doc, b);
    let s = brep(&doc, copy);
    assert_rel(volume(&orig), 2000.0, REL);
    assert_rel(volume(&s), 2000.0, REL);
    near3(orig.bbox().min, [0.0, 0.0, 0.0], 1e-12);
    near3(s.bbox().min, [30.0, 0.0, 0.0], 1e-9);
    golden("copy_translate", &s);
    // Names: instanced(self, slot 0) of the source's; the source keeps its own.
    let cn = doc.evaluated().body(copy).unwrap().naming.clone().unwrap();
    assert_eq!(*cn, names.instanced_names(mv, 0));
    assert!(cn.faces.iter().all(|n| {
        n.segments()
            .iter()
            .any(|s| matches!(s, NameSeg::Inst { feature, instance: 0 } if *feature == mv))
    }));
    assert_eq!(
        doc.evaluated().body(b).unwrap().naming.clone().unwrap(),
        names
    );
    // Not lineage: the copy has no predecessor.
    assert!(doc.evaluated().successors(b).is_empty());
    // A fillet on the copy resolves `Exact` in the copy and leaves the original alone.
    let e = pick(doc.evaluated(), copy, TopoKind::Edge, [40.0, 0.0, 10.0], 1);
    let fil = add(&mut doc, FeatureBox::new(Fillet::constant(vec![e], "2 mm")));
    ok(&doc, fil);
    let (q, rb, _) = first_resolution(&doc, fil);
    assert_eq!((q, rb), (MatchQuality::Exact, copy));
    assert_rel(volume(&brep(&doc, b)), 2000.0, REL);
    assert!(volume(&brep(&doc, copy)) < 2000.0);
    // Editing the copy distance keeps the copy's fillet `Exact`.
    edit(
        &mut doc,
        mv,
        FeatureBox::new(Move::translate(vec![b], ["50 mm", "0 mm", "0 mm"]).copied()),
    );
    let (q, rb, fp) = first_resolution(&doc, fil);
    assert_eq!((q, rb), (MatchQuality::Exact, copy));
    near3(fp.centroid, [60.0, 0.0, 10.0], 1e-9);
}

#[test]
fn free_and_point_to_point_moves_place_the_body() {
    let mut doc = Document::new("free");
    let b = block(&mut doc, [0.0, 0.0, 10.0, 10.0], 10.0);
    // A free matrix: 90° about X, then (5, 0, 0).
    let mut m = IDENTITY_MATRIX;
    m[1] = [0.0, 0.0, -1.0, 0.0];
    m[2] = [0.0, 1.0, 0.0, 0.0];
    m[0][3] = 5.0;
    let mv = add(
        &mut doc,
        FeatureBox::new(Move {
            bodies: vec![b],
            spec: MoveSpec::Free { matrix: m },
            copy: false,
        }),
    );
    ok(&doc, mv);
    let s = brep(&doc, b);
    near3(s.bbox().min, [5.0, -10.0, 0.0], 1e-9);
    near3(s.bbox().max, [15.0, 0.0, 10.0], 1e-9);
    golden("move_free", &s);
    // A scaling matrix is refused at compute (validate only checks it is affine).
    let mut scale = IDENTITY_MATRIX;
    scale[0][0] = 2.0;
    edit(
        &mut doc,
        mv,
        FeatureBox::new(Move {
            bodies: vec![b],
            spec: MoveSpec::Free { matrix: scale },
            copy: false,
        }),
    );
    let Health::Error(d) = health(&doc, mv) else {
        panic!("a scale is not a move");
    };
    assert!(d.message.contains("not rigid"), "{}", d.message);
    // Point to point: vertex (10, 10, 10) of the box onto vertex (20, 0, 0) of another one.
    let mut doc = Document::new("p2p");
    let b = block(&mut doc, [0.0, 0.0, 10.0, 10.0], 10.0);
    let c = block(&mut doc, [20.0, 0.0, 30.0, 10.0], 5.0);
    let from = pick(doc.evaluated(), b, TopoKind::Vertex, [10.0, 10.0, 10.0], 1);
    let to = pick(doc.evaluated(), c, TopoKind::Vertex, [20.0, 0.0, 0.0], 2);
    let mv = add(
        &mut doc,
        FeatureBox::new(Move {
            bodies: vec![b],
            spec: MoveSpec::PointToPoint {
                from: RefInput::Ref(EntityRef::Topo(from)),
                to: RefInput::Ref(EntityRef::Topo(to)),
            },
            copy: false,
        }),
    );
    ok(&doc, mv);
    near3(brep(&doc, b).bbox().min, [10.0, -10.0, -10.0], 1e-9);
}

// ---- Done-when 2: align face to face ------------------------------------------------------------

#[test]
fn dw2_align_face_to_face_makes_the_faces_coplanar() {
    for (flip, angle) in [(false, "0 deg"), (true, "0 deg"), (false, "30 deg")] {
        let mut doc = Document::new("align");
        let a = block(&mut doc, [0.0, 0.0, 10.0, 10.0], 10.0);
        let b = block(&mut doc, [30.0, 0.0, 50.0, 20.0], 5.0);
        // A's +X face onto B's top face.
        let from = pick(doc.evaluated(), a, TopoKind::Face, [10.0, 5.0, 5.0], 1);
        let to = pick(doc.evaluated(), b, TopoKind::Face, [40.0, 10.0, 5.0], 2);
        let fi = doc
            .evaluated()
            .resolve(&from)
            .map(|r| FaceId(r.index))
            .unwrap();
        let mut al = Align::new(
            vec![a],
            RefInput::Ref(EntityRef::Topo(from)),
            RefInput::Ref(EntityRef::Topo(to)),
        );
        al.flip = flip;
        al.angle = ex(angle);
        let f = add(&mut doc, FeatureBox::new(al));
        ok(&doc, f);
        let s = brep(&doc, a);
        valid(&s);
        assert_rel(volume(&s), 1000.0, REL);
        // Same face index (a move keeps indices): its vertices all at z = 5, normal ∓Z.
        for p in face_vertices(&s, fi) {
            assert!((p.z - 5.0).abs() <= 1e-9, "{p:?}");
        }
        let n = face_plane(&s, fi).normal.get();
        let want = if flip { 1.0 } else { -1.0 };
        assert!((n.z - want).abs() <= 1e-12, "{n:?}");
        let bb = s.bbox();
        if flip {
            assert!((bb.max.z - 5.0).abs() <= 1e-9, "A below the top face");
        } else {
            assert!((bb.min.z - 5.0).abs() <= 1e-9, "A sits on the top face");
        }
        if angle == "0 deg" && !flip {
            golden("align_face_to_face", &s);
        }
    }
}

#[test]
fn align_a_face_to_an_origin_plane_and_an_edge_to_an_axis() {
    let mut doc = Document::new("align2");
    let a = block(&mut doc, [10.0, 10.0, 20.0, 30.0], 10.0);
    let top = pick(doc.evaluated(), a, TopoKind::Face, [15.0, 20.0, 10.0], 1);
    let f = add(
        &mut doc,
        FeatureBox::new(Align::new(
            vec![a],
            RefInput::Ref(EntityRef::Topo(top)),
            RefInput::Origin(OriginEntity::XYPlane),
        )),
    );
    ok(&doc, f);
    let bb = brep(&doc, a).bbox();
    // The top face (normal +Z) faces the XY plane's +Z normal: A turns over onto z ≥ 0.
    assert!(
        (bb.min.z - 0.0).abs() <= 1e-9 && (bb.max.z - 10.0).abs() <= 1e-9,
        "{bb:?}"
    );
    // A vertical edge of a box onto the Z axis.
    let mut doc = Document::new("align3");
    let a = block(&mut doc, [10.0, 10.0, 20.0, 30.0], 10.0);
    let edge = pick(doc.evaluated(), a, TopoKind::Edge, [20.0, 30.0, 5.0], 1);
    let f = add(
        &mut doc,
        FeatureBox::new(Align::new(
            vec![a],
            RefInput::Ref(EntityRef::Topo(edge)),
            RefInput::Origin(OriginEntity::ZAxis),
        )),
    );
    ok(&doc, f);
    let s = brep(&doc, a);
    let bb = s.bbox();
    assert!(bb.min.x.abs() <= 1e-9 || bb.max.x.abs() <= 1e-9, "{bb:?}");
    assert!(bb.min.y.abs() <= 1e-9 || bb.max.y.abs() <= 1e-9, "{bb:?}");
    // A plane onto a line is refused.
    let mut doc = Document::new("align4");
    let a = block(&mut doc, [0.0, 0.0, 10.0, 10.0], 10.0);
    let top = pick(doc.evaluated(), a, TopoKind::Face, [5.0, 5.0, 10.0], 1);
    let f = add(
        &mut doc,
        FeatureBox::new(Align::new(
            vec![a],
            RefInput::Ref(EntityRef::Topo(top)),
            RefInput::Origin(OriginEntity::XAxis),
        )),
    );
    let Health::Error(d) = health(&doc, f) else {
        panic!("plane onto line")
    };
    assert!(d.message.contains("plane to a plane"), "{}", d.message);
}

// ---- Done-when 3: offset face ---------------------------------------------------------------------

#[test]
fn dw3_offset_of_a_box_top_and_a_cylinder_face() {
    let mut doc = Document::new("offset");
    let b = block(&mut doc, [0.0, 0.0, 20.0, 10.0], 10.0);
    let top = pick(doc.evaluated(), b, TopoKind::Face, [10.0, 5.0, 10.0], 1);
    let e = pick(doc.evaluated(), b, TopoKind::Edge, [10.0, 0.0, 10.0], 1);
    let fil = add(&mut doc, FeatureBox::new(Fillet::constant(vec![e], "1 mm")));
    let off = insert_before(
        &mut doc,
        fil,
        FeatureBox::new(OffsetFace::new(vec![top], "2 mm")),
    );
    ok(&doc, off);
    ok(&doc, fil);
    // The fillet's edge (top front) is still found `Exact`, now at z = 12.
    let (q, _, fp) = first_resolution(&doc, fil);
    assert_eq!(q, MatchQuality::Exact);
    near3(fp.centroid, [10.0, 0.0, 12.0], 1e-9);
    // Remove the fillet to measure the plain offset.
    doc.apply(Command::DeleteFeatures { ids: vec![fil] })
        .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    let s = brep(&doc, b);
    valid(&s);
    let bb = s.bbox();
    assert!((bb.max.z - 12.0).abs() <= 1e-9, "height + 2: {bb:?}");
    assert!(bb.min.z.abs() <= 1e-9);
    assert_rel(volume(&s), 2400.0, REL);
    assert_eq!(faces(&s), 6);
    golden("offset_box_top", &s);
    // Inwards.
    let top = pick(doc.evaluated(), b, TopoKind::Face, [10.0, 5.0, 12.0], 1);
    edit(
        &mut doc,
        off,
        FeatureBox::new(OffsetFace::new(vec![top], "-3 mm")),
    );
    ok(&doc, off);
    let s = brep(&doc, b);
    assert!((s.bbox().max.z - 7.0).abs() <= 1e-9);
    assert_rel(volume(&s), 1400.0, REL);

    // A cylinder face + 1 mm: radius 5 → 6.
    let mut doc = Document::new("offset_cyl");
    let c = cylinder(&mut doc, 5.0, 10.0);
    let side = pick(doc.evaluated(), c, TopoKind::Face, [0.0, 0.0, 5.0], 1);
    let off = add(
        &mut doc,
        FeatureBox::new(OffsetFace::new(vec![side], "1 mm")),
    );
    ok(&doc, off);
    let s = brep(&doc, c);
    valid(&s);
    let r = (0..s.count(SubKind::Face))
        .find_map(|i| match s.surface(FaceId(i)) {
            Surface::Cylinder { radius, .. } => Some(radius),
            _ => None,
        })
        .expect("a cylinder face");
    assert!((r - 6.0).abs() <= 1e-9, "radius {r}");
    let bb = s.bbox();
    assert!(
        (bb.max.x - 6.0).abs() <= 1e-7 && (bb.min.y + 6.0).abs() <= 1e-7,
        "{bb:?}"
    );
    assert_rel(volume(&s), std::f64::consts::PI * 36.0 * 10.0, REL);
    golden("offset_cylinder", &s);
}

// ---- Done-when 4: replace face --------------------------------------------------------------------

fn offset_plane(doc: &mut Document, z: f64) -> RefInput {
    let p = add(
        doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: ex(&format!("{z} mm")),
            },
        }),
    );
    RefInput::Ref(EntityRef::Construction(ConstructionId(p)))
}

#[test]
fn dw4_replace_face_onto_an_offset_plane_matches_the_analytic_volume() {
    for (z, want, case) in [(15.0, 3000.0, "replace_up"), (4.0, 800.0, "replace_down")] {
        let mut doc = Document::new(case);
        let b = block(&mut doc, [0.0, 0.0, 20.0, 10.0], 10.0);
        let top = pick(doc.evaluated(), b, TopoKind::Face, [10.0, 5.0, 10.0], 1);
        let plane = offset_plane(&mut doc, z);
        // A downstream offset of the top face: its reference survives the inserted replace.
        let off = add(
            &mut doc,
            FeatureBox::new(OffsetFace::new(vec![top.clone()], "1 mm")),
        );
        let rep = insert_before(
            &mut doc,
            off,
            FeatureBox::new(ReplaceFace::new(vec![top], plane)),
        );
        ok(&doc, rep);
        ok(&doc, off);
        let (q, _, _) = first_resolution(&doc, off);
        assert_eq!(q, MatchQuality::Exact, "the replaced face keeps its name");
        let s = brep(&doc, b);
        assert_rel(volume(&s), (want / 200.0 + 1.0) * 200.0, REL);
        doc.apply(Command::DeleteFeatures { ids: vec![off] })
            .unwrap();
        doc.recompute_blocking(jobs()).unwrap();
        let s = brep(&doc, b);
        valid(&s);
        assert_rel(volume(&s), want, REL);
        assert!((s.bbox().max.z - z).abs() <= 1e-9);
        assert_eq!(faces(&s), 6);
        golden(case, &s);
    }
    // The surface of a face of another body (z = 12).
    let mut doc = Document::new("replace_face");
    let b = block(&mut doc, [0.0, 0.0, 20.0, 10.0], 10.0);
    let c = block(&mut doc, [40.0, 0.0, 50.0, 10.0], 12.0);
    let top = pick(doc.evaluated(), b, TopoKind::Face, [10.0, 5.0, 10.0], 1);
    let other = pick(doc.evaluated(), c, TopoKind::Face, [45.0, 5.0, 12.0], 2);
    let rep = add(
        &mut doc,
        FeatureBox::new(ReplaceFace::new(
            vec![top],
            RefInput::Ref(EntityRef::Topo(other)),
        )),
    );
    ok(&doc, rep);
    assert_rel(volume(&brep(&doc, b)), 2400.0, REL);
    assert_rel(volume(&brep(&doc, c)), 1200.0, REL);
}

#[test]
fn replace_face_with_tangent_chain_takes_the_coplanar_neighbour_too() {
    // A box top split in two along x = 0: the halves are tangent (coplanar) neighbours, so
    // replacing one half with tangent chain replaces both and the part is a block to z = 15.
    let mut doc = Document::new("chain");
    let b = block(&mut doc, [-10.0, 0.0, 10.0, 10.0], 10.0);
    let top = pick(doc.evaluated(), b, TopoKind::Face, [0.0, 5.0, 10.0], 1);
    let split = add(
        &mut doc,
        FeatureBox::new(amg_features::solid::SplitFace::by_surface(
            vec![top],
            RefInput::Origin(OriginEntity::YZPlane),
        )),
    );
    ok(&doc, split);
    assert_eq!(faces(&brep(&doc, b)), 7);
    let half = pick(doc.evaluated(), b, TopoKind::Face, [-5.0, 5.0, 10.0], 1);
    let plane = offset_plane(&mut doc, 15.0);
    let mut r = ReplaceFace::new(vec![half], plane);
    r.tangent_chain = true;
    let rep = add(&mut doc, FeatureBox::new(r));
    ok(&doc, rep);
    let s = brep(&doc, b);
    valid(&s);
    assert!((s.bbox().max.z - 15.0).abs() <= 1e-9);
    assert_rel(volume(&s), 3000.0, REL);
    golden("replace_tangent_chain", &s);
}

// ---- Done-when 5: delete face -------------------------------------------------------------------

/// Box 20 × 10 × 10 with a 2 mm fillet on the top front edge; the blend face.
fn filleted_box(doc: &mut Document) -> (BodyId, TopoRef) {
    let b = block(doc, [0.0, 0.0, 20.0, 10.0], 10.0);
    let e = pick(doc.evaluated(), b, TopoKind::Edge, [10.0, 0.0, 10.0], 1);
    add(doc, FeatureBox::new(Fillet::constant(vec![e], "2 mm")));
    // Arc centroid at 2 − 2·2√2/π·cos45° from the corner.
    let c = 2.0 - 2.0 * 2.0 / std::f64::consts::PI;
    let blend = pick(doc.evaluated(), b, TopoKind::Face, [10.0, c, 10.0 - c], 1);
    (b, blend)
}

#[test]
fn dw5_deleting_a_fillet_face_with_heal_restores_the_box() {
    let mut doc = Document::new("heal");
    let (b, blend) = filleted_box(&mut doc);
    let filleted = volume(&brep(&doc, b));
    assert!(filleted < 2000.0);
    let del = add(
        &mut doc,
        FeatureBox::new(DeleteFace::new(vec![blend], true)),
    );
    ok(&doc, del);
    let s = brep(&doc, b);
    valid(&s);
    assert_rel(volume(&s), 2000.0, 1e-6);
    assert_eq!(faces(&s), 6);
    check_golden_as("sol09", 1e-6, "delete_fillet_heal", &s);
}

#[test]
fn dw5_deleting_without_heal_leaves_an_open_body_with_free_edges() {
    let mut doc = Document::new("remove");
    let (b, blend) = filleted_box(&mut doc);
    let before = brep(&doc, b);
    let blend_area = std::f64::consts::PI * 2.0 / 2.0 * 20.0;
    let del = add(
        &mut doc,
        FeatureBox::new(DeleteFace::new(vec![blend], false)),
    );
    ok(&doc, del);
    let s = brep(&doc, b);
    assert_eq!(faces(&s), faces(&before) - 1);
    // The blend's boundary: two straight tangent edges and two quarter arcs.
    assert_eq!(free_edges(&s), 4);
    assert!(s.solids().is_empty(), "an open shell, not a solid");
    assert_rel(area(&s), area(&before) - blend_area, 1e-6);
    check_golden_as("sol09", 1e-6, "delete_fillet_remove", &s);

    // A plain box without its top: 5 faces, 4 free edges, area 1000 − 200.
    let mut doc = Document::new("remove_top");
    let b = block(&mut doc, [0.0, 0.0, 20.0, 10.0], 10.0);
    let top = pick(doc.evaluated(), b, TopoKind::Face, [10.0, 5.0, 10.0], 1);
    let side = pick(doc.evaluated(), b, TopoKind::Face, [10.0, 0.0, 5.0], 2);
    // A downstream reference to a side face keeps resolving `Exact` (names kept).
    let off = add(&mut doc, FeatureBox::new(DeleteFace::new(vec![top], false)));
    ok(&doc, off);
    let s = brep(&doc, b);
    assert_eq!((faces(&s), free_edges(&s)), (5, 4));
    assert_rel(area(&s), 800.0, REL);
    golden("delete_top_remove", &s);
    let probe = add(
        &mut doc,
        FeatureBox::new(DeleteFace::new(vec![side], false)),
    );
    ok(&doc, probe);
    let (q, _, _) = first_resolution(&doc, probe);
    assert_eq!(q, MatchQuality::Exact);
    assert_eq!(free_edges(&brep(&doc, b)), 6, "the U of four faces");
}

#[test]
fn a_reference_to_a_deleted_face_is_not_found_with_the_tombstone() {
    let mut doc = Document::new("tomb");
    let (b, blend) = filleted_box(&mut doc);
    let probe = add(
        &mut doc,
        FeatureBox::new(DeleteFace::new(vec![blend.clone()], false)),
    );
    ok(&doc, probe);
    let del = insert_before(
        &mut doc,
        probe,
        FeatureBox::new(DeleteFace::new(vec![blend], true)),
    );
    ok(&doc, del);
    assert!(
        matches!(health(&doc, probe), Health::Error(_)),
        "the blend is gone"
    );
    // The tombstone names the delete as the feature that removed it.
    let rec = &doc.evaluated().node(probe).unwrap().resolutions[0];
    assert_eq!(
        rec.result,
        Err(amg_doc::ResolveError::NotFound {
            kind: TopoKind::Face,
            removed_by: Some(del),
        })
    );
    assert_rel(volume(&brep(&doc, b)), 2000.0, 1e-6);
}

// ---- Done-when 6 (end to end) ------------------------------------------------------------------

#[test]
fn dw6_press_pull_on_real_picks_opens_the_right_dialogs() {
    let mut doc = Document::new("pp");
    let (s, e) = extrude_rects(
        &mut doc,
        &[[0.0, 0.0, 20.0, 10.0]],
        10.0,
        BodyOp::NewBody,
        vec![],
    );
    let b = body0(e);
    let face = pick(doc.evaluated(), b, TopoKind::Face, [10.0, 5.0, 10.0], 1);
    let edge = pick(doc.evaluated(), b, TopoKind::Edge, [10.0, 0.0, 10.0], 1);
    let prof = profiles(&doc, s);
    let d = dialog::dispatches()
        .find(|d| d.command_id == "solid.press_pull")
        .unwrap();
    for (sel, kind) in [
        (vec![EntityRef::Topo(face)], "solid.offset_face"),
        (vec![EntityRef::Topo(edge)], "solid.fillet"),
        (prof, "solid.extrude"),
    ] {
        assert_eq!((d.dispatch)(&sel, doc.design()), Some(kind));
        // The dialog it opens takes the selection and builds a feature that computes.
        let reg = dialog::registration(kind).unwrap();
        let init = DialogInit {
            preselection: &sel,
            design: doc.design(),
            evaluated: doc.evaluated(),
        };
        let f = (reg.build)(&(reg.defaults)(&init), None).unwrap();
        let id = add(&mut doc, f);
        ok(&doc, id);
    }
}
