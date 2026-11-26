//! The OCCT half of the SOL-07 tests: goldens `tests/golden/solid/sol07_*.toml`, body lineage
//! (`Consumed`, `CreatedFrom`), piece order, TNP-21, TNP-23, split face areas and `NoSplit`.

use amg_brep::{BrepKernel, CheckOpts, SubKind};
use amg_core::{Point2, Point3};
use amg_doc::naming::{GeomSource, KernelGeom, NamingView, default_kernel, topo_ref_at};
use amg_doc::stub::SketchEntityKey;
use amg_doc::{
    Command, ConstructionId, Evaluated, Expr, ExprSrc, Health, MatchQuality, SketchId, TopoKind,
    TopoRef,
};
use amg_features::construct::{ConstructPlane, PlaneDef};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{Extrude, Fillet, OpTarget};

use super::*;

const REL: f64 = 1e-9;

fn ex(s: &str) -> Expr {
    Expr::Inline(ExprSrc(s.to_owned()))
}

#[track_caller]
fn golden(case: &str, s: &amg_brep::Shape) {
    check_golden_as("sol07", REL, case, s);
}

#[track_caller]
fn ok(doc: &Document, f: FeatureId) {
    assert_eq!(health(doc, f), Health::Ok, "feature {f:?}");
}

#[track_caller]
fn valid(s: &amg_brep::Shape) {
    assert!(kernel().check(s, &CheckOpts::default()).valid, "valid");
}

/// The error message of a failed feature.
#[track_caller]
fn error_of(doc: &Document, f: FeatureId) -> String {
    match health(doc, f) {
        Health::Error(d) => d.message,
        h => panic!("expected an error, got {h:?}"),
    }
}

/// A box `[x0, y0, x1, y1]` × `0..h` as a new body.
fn block(doc: &mut Document, r: [f64; 4], h: f64) -> BodyId {
    body0(extrude_rects(doc, &[r], h, BodyOp::NewBody, vec![]).1)
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

/// Areas of the planar faces whose centroid is at height `z`, ascending.
fn areas_at(s: &amg_brep::Shape, z: f64) -> Vec<f64> {
    let [faces, _, _] = kernel().sub_geom_all(s).unwrap();
    let mut a: Vec<f64> = faces
        .iter()
        .filter(|g| (g.centroid.z - z).abs() < 1e-9)
        .map(|g| g.measure)
        .collect();
    a.sort_by(f64::total_cmp);
    a
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
fn expect_entity(fp: &amg_doc::GeomFingerprint, centroid: [f64; 3], measure: f64) {
    let c = Point3::from_array(centroid);
    assert!((fp.centroid - c).length() < 1e-6, "{fp:?} vs {centroid:?}");
    assert!((fp.measure - measure).abs() < 1e-6, "{fp:?} vs {measure}");
}

// ---- Done-when 1: combine ---------------------------------------------------------------------

/// A = 0..20 × 0..10 × 0..10 (2000 mm³), B = 10..30 × 0..10 × 0..10 (half overlapping).
fn two_blocks(doc: &mut Document) -> (BodyId, BodyId) {
    let a = block(doc, [0.0, 0.0, 20.0, 10.0], 10.0);
    let b = block(doc, [10.0, 0.0, 30.0, 10.0], 10.0);
    (a, b)
}

#[test]
fn dw1_combine_goldens_for_each_op_with_and_without_keep_tools() {
    for (op, name, want, min_x, max_x) in [
        (BodyOp::Join, "join", 3000.0, 0.0, 30.0),
        (BodyOp::Cut, "cut", 1000.0, 0.0, 10.0),
        (BodyOp::Intersect, "intersect", 1000.0, 10.0, 20.0),
    ] {
        for keep in [false, true] {
            let mut doc = Document::new(name);
            let (a, b) = two_blocks(&mut doc);
            let mut c = Combine::new(a, vec![b], op);
            c.keep_tools = keep;
            let f = add(&mut doc, FeatureBox::new(c));
            ok(&doc, f);
            let s = brep(&doc, a);
            valid(&s);
            assert_rel(volume(&s), want, REL);
            assert_eq!(faces(&s), 6, "{name}: unified to a box");
            let bb = s.bbox();
            assert!((bb.min.x - min_x).abs() < 1e-9 && (bb.max.x - max_x).abs() < 1e-9);
            let ev = doc.evaluated();
            if keep {
                assert_rel(volume(&brep(&doc, b)), 2000.0, REL);
                assert!(
                    ev.successors(b).is_empty(),
                    "{name}: a kept tool has no successor"
                );
                golden(&format!("{name}_keep_tools"), &s);
            } else {
                assert!(ev.body(b).is_none(), "{name}: the tool is consumed");
                assert_eq!(
                    ev.successors(b),
                    &[a],
                    "{name}: Consumed {{ into: target }}"
                );
                golden(name, &s);
            }
        }
    }
}

#[test]
fn dw1_a_combine_cut_through_a_bar_creates_piece_1_from_the_target() {
    let mut doc = Document::new("cut-split");
    let bar = block(&mut doc, [0.0, 0.0, 30.0, 10.0], 10.0);
    let tool = block(&mut doc, [10.0, -5.0, 20.0, 15.0], 20.0);
    let f = add(
        &mut doc,
        FeatureBox::new(Combine::new(bar, vec![tool], BodyOp::Cut)),
    );
    ok(&doc, f);
    let piece1 = BodyId { origin: f, slot: 1 };
    let (p0, p1) = (brep(&doc, bar), brep(&doc, piece1));
    assert_rel(volume(&p0), 1000.0, REL);
    assert_rel(volume(&p1), 1000.0, REL);
    valid(&p0);
    valid(&p1);
    let ev: &Evaluated = doc.evaluated();
    assert_eq!(NamingView::derived_from(ev, piece1), Some(bar));
    assert_eq!(ev.successors(tool), &[bar]);
    golden("cut_split_piece0", &p0);
    golden("cut_split_piece1", &p1);
}

#[test]
fn dw1_several_tools_in_one_combine() {
    let mut doc = Document::new("multi");
    let plate = block(&mut doc, [0.0, 0.0, 40.0, 10.0], 10.0);
    let t1 = block(&mut doc, [5.0, 2.0, 10.0, 8.0], 20.0);
    let t2 = block(&mut doc, [25.0, 2.0, 30.0, 8.0], 20.0);
    let f = add(
        &mut doc,
        FeatureBox::new(Combine::new(plate, vec![t1, t2], BodyOp::Cut)),
    );
    ok(&doc, f);
    let s = brep(&doc, plate);
    valid(&s);
    assert_rel(volume(&s), 4000.0 - 2.0 * 300.0, REL);
    assert_eq!(faces(&s), 14, "two pockets through the plate");
    golden("cut_two_tools", &s);
    let ev = doc.evaluated();
    assert_eq!(ev.successors(t1), &[plate]);
    assert_eq!(ev.successors(t2), &[plate]);
}

#[test]
fn dw1_combine_errors_nothing_to_cut_and_nothing_to_intersect() {
    let mut doc = Document::new("errors");
    let a = block(&mut doc, [0.0, 0.0, 10.0, 10.0], 10.0);
    let far = block(&mut doc, [50.0, 0.0, 60.0, 10.0], 10.0);
    let cut = add(
        &mut doc,
        FeatureBox::new(Combine::new(a, vec![far], BodyOp::Cut)),
    );
    assert!(error_of(&doc, cut).contains("nothing to cut"));
    let int = add(
        &mut doc,
        FeatureBox::new(Combine::new(a, vec![far], BodyOp::Intersect)),
    );
    assert!(error_of(&doc, int).contains("nothing to intersect"));
    let own = add(
        &mut doc,
        FeatureBox::new(Combine::new(a, vec![a], BodyOp::Join)),
    );
    assert!(error_of(&doc, own).contains("its own tool"));
    let none = add(
        &mut doc,
        FeatureBox::new(Combine {
            target: None,
            tools: vec![far],
            op: BodyOp::Join,
            keep_tools: false,
        }),
    );
    assert!(error_of(&doc, none).contains("target"));
    // The failed features changed nothing.
    assert_rel(volume(&brep(&doc, a)), 1000.0, REL);
    assert_rel(volume(&brep(&doc, far)), 1000.0, REL);
}

#[test]
fn dw1_tnp23_a_combine_inserted_upstream_folds_b_into_a() {
    let mut doc = Document::new("tnp23");
    let a = block(&mut doc, [0.0, 0.0, 50.0, 60.0], 40.0);
    let b = block(&mut doc, [60.0, 0.0, 100.0, 60.0], 20.0);
    let e = pick(doc.evaluated(), b, TopoKind::Edge, [80.0, 0.0, 20.0], 1);
    let fil = add(&mut doc, FeatureBox::new(Fillet::constant(vec![e], "2 mm")));
    ok(&doc, fil);
    let comb = insert_before(
        &mut doc,
        fil,
        FeatureBox::new(Combine::new(a, vec![b], BodyOp::Join)),
    );
    // B doesn't touch A: one body of two lumps, with a warning.
    let Health::Warning(w) = health(&doc, comb) else {
        panic!("{:?}", health(&doc, comb));
    };
    assert!(
        w.iter().any(|d| d.message.contains("separate lump")),
        "{w:?}"
    );
    assert!(doc.evaluated().body(b).is_none());
    ok(&doc, fil);
    let (q, body, fp) = first_resolution(&doc, fil);
    assert_eq!(q, MatchQuality::Exact);
    assert_eq!(body, a, "the fillet follows B into A");
    expect_entity(&fp, [80.0, 0.0, 20.0], 40.0);
    let s = brep(&doc, a);
    valid(&s);
    assert_eq!(s.solids().len(), 2);
}

// ---- Done-when 2: split body ------------------------------------------------------------------

#[test]
fn dw2_splitting_a_box_by_a_plane_orders_pieces_along_the_normal() {
    let mut doc = Document::new("split-yz");
    let b = block(&mut doc, [-50.0, 0.0, 50.0, 60.0], 40.0);
    let split = add(
        &mut doc,
        FeatureBox::new(SplitBody::new(b, RefInput::Origin(OriginEntity::YZPlane))),
    );
    ok(&doc, split);
    let piece1 = BodyId {
        origin: split,
        slot: 1,
    };
    let (p0, p1) = (brep(&doc, b), brep(&doc, piece1));
    valid(&p0);
    valid(&p1);
    assert_rel(volume(&p0), 120_000.0, REL);
    assert_rel(volume(&p1), 120_000.0, REL);
    assert!(p0.bbox().max.x.abs() < 1e-9, "piece 0 is on the −X side");
    assert!(
        p1.bbox().min.x.abs() < 1e-9,
        "piece 1 is on the +X side (YZ normal)"
    );
    let ev: &Evaluated = doc.evaluated();
    assert_eq!(NamingView::derived_from(ev, piece1), Some(b));
    assert_eq!(ev.bodies().count(), 2);
    golden("split_yz_piece0", &p0);
    golden("split_yz_piece1", &p1);

    // XZ's normal is −Y, so piece 0 is the +Y half.
    let mut doc = Document::new("split-xz");
    let b = block(&mut doc, [0.0, -10.0, 10.0, 30.0], 10.0);
    let split = add(
        &mut doc,
        FeatureBox::new(SplitBody::new(b, RefInput::Origin(OriginEntity::XZPlane))),
    );
    ok(&doc, split);
    let p0 = brep(&doc, b);
    assert!(p0.bbox().min.y.abs() < 1e-9, "piece 0 is y 0..30");
    assert_rel(volume(&p0), 3000.0, REL);
    assert_rel(
        volume(&brep(
            &doc,
            BodyId {
                origin: split,
                slot: 1,
            },
        )),
        1000.0,
        REL,
    );
}

/// TNP-21 on a polygon sketch (so the block can grow) + extrude.
fn tnp21_block(doc: &mut Document, x1: f64) -> (FeatureId, FeatureId) {
    let (sk, _, _) = polygon_sketch(&[[-50.0, 0.0], [x1, 0.0], [x1, 60.0], [-50.0, 60.0]]);
    let s = add(doc, FeatureBox::new(sk));
    let e = add(
        doc,
        FeatureBox::new(Extrude::distance(
            profiles(doc, s),
            "40 mm",
            OpTarget::default(),
        )),
    );
    (s, e)
}

#[test]
fn dw2_tnp21_a_fillet_on_the_positive_piece_survives_the_block_growing() {
    let mut doc = Document::new("tnp21");
    let (s, e) = tnp21_block(&mut doc, 50.0);
    let split = add(
        &mut doc,
        FeatureBox::new(SplitBody::new(
            body0(e),
            RefInput::Origin(OriginEntity::YZPlane),
        )),
    );
    ok(&doc, split);
    let piece1 = BodyId {
        origin: split,
        slot: 1,
    };
    let edge = pick(
        doc.evaluated(),
        piece1,
        TopoKind::Edge,
        [25.0, 0.0, 40.0],
        1,
    );
    let fil = add(
        &mut doc,
        FeatureBox::new(Fillet::constant(vec![edge], "2 mm")),
    );
    ok(&doc, fil);
    // The block grows to x = 80.
    let (sk, _, _) = polygon_sketch(&[[-50.0, 0.0], [80.0, 0.0], [80.0, 60.0], [-50.0, 60.0]]);
    edit(&mut doc, s, FeatureBox::new(sk));
    let p = profiles(&doc, s);
    edit(
        &mut doc,
        e,
        FeatureBox::new(Extrude::distance(p, "40 mm", OpTarget::default())),
    );
    ok(&doc, split);
    ok(&doc, fil);
    let (q, body, fp) = first_resolution(&doc, fil);
    assert_eq!(q, MatchQuality::Exact);
    assert_eq!(body, piece1);
    expect_entity(&fp, [40.0, 0.0, 40.0], 80.0);
    assert_rel(
        volume(&brep(&doc, piece1)),
        80.0 * 60.0 * 40.0 - 4.0 * 80.0 * (1.0 - std::f64::consts::FRAC_PI_4),
        1e-9,
    );
}

#[test]
fn dw2_split_by_a_body_orders_pieces_by_volume_and_keeps_the_tool() {
    let mut doc = Document::new("split-body");
    let bar = block(&mut doc, [0.0, 0.0, 40.0, 10.0], 10.0);
    let tool = block(&mut doc, [10.0, -5.0, 20.0, 15.0], 20.0);
    let split = add(
        &mut doc,
        FeatureBox::new(SplitBody::new(bar, RefInput::Ref(EntityRef::Body(tool)))),
    );
    ok(&doc, split);
    let p = |k| {
        if k == 0 {
            bar
        } else {
            BodyId {
                origin: split,
                slot: k,
            }
        }
    };
    let want = [(2000.0, 30.0), (1000.0, 5.0), (1000.0, 15.0)];
    for (k, (v, cx)) in (0u32..).zip(want) {
        let s = brep(&doc, p(k));
        valid(&s);
        assert_rel(volume(&s), v, REL);
        let c = kernel().mass_props(&s).unwrap().centroid;
        assert!((c.x - cx).abs() < 1e-9, "piece {k}: centroid {c:?}");
    }
    assert_rel(volume(&brep(&doc, tool)), 4000.0, REL);
    golden("split_by_body_piece0", &brep(&doc, bar));
}

// ---- Done-when 4 (split body): NoSplit and extend --------------------------------------------

#[test]
fn dw4_a_face_that_doesnt_cross_fails_no_split_and_extend_fixes_it() {
    let mut doc = Document::new("nosplit");
    let bar = block(&mut doc, [0.0, 0.0, 40.0, 10.0], 10.0);
    // A 10 × 20 face at z = 5 (the top of a small block) inside the bar's x range only.
    let small = block(&mut doc, [15.0, -5.0, 25.0, 15.0], 5.0);
    let top = pick(doc.evaluated(), small, TopoKind::Face, [20.0, 5.0, 5.0], 1);
    let tool = RefInput::Ref(EntityRef::Topo(top.clone()));
    let split = add(&mut doc, FeatureBox::new(SplitBody::new(bar, tool.clone())));
    let msg = error_of(&doc, split);
    assert!(msg.contains("does not split"), "{msg}");
    assert!(msg.contains("Extend the tool"), "{msg}");
    let node = doc.evaluated().node(split).unwrap();
    let Health::Error(d) = &node.health else {
        unreachable!()
    };
    assert!(
        d.entities
            .iter()
            .any(|e| matches!(e, EntityRef::Topo(t) if t.key == top.key)),
        "the tool is highlighted: {:?}",
        d.entities
    );
    let mut ext = SplitBody::new(bar, tool);
    ext.extend_tool = true;
    edit(&mut doc, split, FeatureBox::new(ext));
    ok(&doc, split);
    let (p0, p1) = (
        brep(&doc, bar),
        brep(
            &doc,
            BodyId {
                origin: split,
                slot: 1,
            },
        ),
    );
    assert_rel(volume(&p0), 2000.0, REL);
    assert_rel(volume(&p1), 2000.0, REL);
    assert!(
        p0.bbox().max.z < 5.0 + 1e-9,
        "piece 0 is below the face (normal +Z)"
    );
    golden("split_extended_face_piece1", &p1);
}

#[test]
fn dw4_a_plane_that_misses_the_body_fails_no_split() {
    let mut doc = Document::new("miss");
    let bar = block(&mut doc, [0.0, 0.0, 40.0, 10.0], 10.0);
    let plane = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: ex("50 mm"),
            },
        }),
    );
    let split = add(
        &mut doc,
        FeatureBox::new(SplitBody::new(
            bar,
            RefInput::Ref(EntityRef::Construction(ConstructionId(plane))),
        )),
    );
    let msg = error_of(&doc, split);
    assert!(msg.contains("Extend the tool"), "{msg}");
    assert_rel(volume(&brep(&doc, bar)), 4000.0, REL);
}

// ---- Done-when 3: split face --------------------------------------------------------------------

/// A sketch on XY with one line; returns the sketch and the line's reference.
fn line_sketch(doc: &mut Document, a: [f64; 2], b: [f64; 2]) -> (FeatureId, EntityRef) {
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    let id = f
        .edit_sketch(|s| s.add_line(Point2::new(a[0], a[1]), Point2::new(b[0], b[1])))
        .unwrap();
    let s = add(doc, FeatureBox::new(f));
    (
        s,
        EntityRef::SketchCurve {
            sketch: SketchId(s),
            curve: SketchEntityKey(u64::from(id.0)),
        },
    )
}

#[test]
fn dw3_split_face_by_a_projected_sketch_line_and_by_a_plane_match() {
    // The top face of 0..20 × 0..10 × 0..10 split at x = 5: 50 + 150 mm².
    let mut doc = Document::new("split-face");
    let a = block(&mut doc, [0.0, 0.0, 20.0, 10.0], 10.0);
    let b = block(&mut doc, [0.0, 20.0, 20.0, 30.0], 10.0);
    let (_, line) = line_sketch(&mut doc, [5.0, -5.0], [5.0, 15.0]);
    let top_a = pick(doc.evaluated(), a, TopoKind::Face, [10.0, 5.0, 10.0], 1);
    let by_curve = add(
        &mut doc,
        FeatureBox::new(SplitFace::by_curves(vec![top_a], vec![line])),
    );
    ok(&doc, by_curve);
    let plane = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::YZPlane),
                distance: ex("5 mm"),
            },
        }),
    );
    let top_b = pick(doc.evaluated(), b, TopoKind::Face, [10.0, 25.0, 10.0], 1);
    let by_plane = add(
        &mut doc,
        FeatureBox::new(SplitFace::by_surface(
            vec![top_b],
            RefInput::Ref(EntityRef::Construction(ConstructionId(plane))),
        )),
    );
    ok(&doc, by_plane);
    let (sa, sb) = (brep(&doc, a), brep(&doc, b));
    for s in [&sa, &sb] {
        valid(s);
        assert_eq!(faces(s), 7);
        assert_eq!(s.solids().len(), 1, "still one solid");
        assert_rel(volume(s), 2000.0, REL);
    }
    let (aa, ab) = (areas_at(&sa, 10.0), areas_at(&sb, 10.0));
    assert_eq!(aa.len(), 2);
    assert_rel(aa[0], 50.0, REL);
    assert_rel(aa[1], 150.0, REL);
    for (x, y) in aa.iter().zip(&ab) {
        assert_rel(*x, *y, REL);
    }
    golden("split_face_by_line", &sa);
    golden("split_face_by_plane", &sb);
}

#[test]
fn dw3_curves_along_an_explicit_direction_and_a_face_set() {
    // Two faces (top and bottom) split along Z by one sketch line.
    let mut doc = Document::new("along");
    let a = block(&mut doc, [0.0, 0.0, 20.0, 10.0], 10.0);
    let (_, line) = line_sketch(&mut doc, [15.0, -5.0], [15.0, 15.0]);
    let top = pick(doc.evaluated(), a, TopoKind::Face, [10.0, 5.0, 10.0], 1);
    let bottom = pick(doc.evaluated(), a, TopoKind::Face, [10.0, 5.0, 0.0], 2);
    let f = add(
        &mut doc,
        FeatureBox::new(SplitFace {
            faces: vec![top, bottom],
            tool: SplitFaceTool::CurvesAlong {
                curves: vec![line],
                dir: Some(RefInput::Origin(OriginEntity::ZAxis)),
            },
            extend_tool: false,
        }),
    );
    ok(&doc, f);
    let s = brep(&doc, a);
    valid(&s);
    assert_eq!(faces(&s), 8);
    for z in [0.0, 10.0] {
        let ar = areas_at(&s, z);
        assert_rel(ar[0], 50.0, REL);
        assert_rel(ar[1], 150.0, REL);
    }
}

#[test]
fn dw3_a_split_face_piece_keeps_its_name_through_a_height_edit() {
    let mut doc = Document::new("split-face-edit");
    let (s, e) = extrude_rects(
        &mut doc,
        &[[0.0, 0.0, 20.0, 10.0]],
        10.0,
        BodyOp::NewBody,
        vec![],
    );
    let a = body0(e);
    let (_, line) = line_sketch(&mut doc, [5.0, -5.0], [5.0, 15.0]);
    let top = pick(doc.evaluated(), a, TopoKind::Face, [10.0, 5.0, 10.0], 1);
    let sf = add(
        &mut doc,
        FeatureBox::new(SplitFace::by_curves(vec![top], vec![line])),
    );
    ok(&doc, sf);
    // The edge between the small piece (x 0..5) and the left side face.
    let edge = pick(doc.evaluated(), a, TopoKind::Edge, [0.0, 5.0, 10.0], 1);
    let fil = add(
        &mut doc,
        FeatureBox::new(Fillet::constant(vec![edge], "1 mm")),
    );
    ok(&doc, fil);
    let p = profiles(&doc, s);
    edit(
        &mut doc,
        e,
        FeatureBox::new(Extrude::distance(p, "20 mm", OpTarget::default())),
    );
    ok(&doc, sf);
    ok(&doc, fil);
    let (q, _, fp) = first_resolution(&doc, fil);
    assert_eq!(q, MatchQuality::Exact);
    expect_entity(&fp, [0.0, 5.0, 20.0], 10.0);
}

#[test]
fn dw4_split_face_by_a_curve_off_the_face_fails_no_split() {
    let mut doc = Document::new("face-miss");
    let a = block(&mut doc, [0.0, 0.0, 20.0, 10.0], 10.0);
    let (_, line) = line_sketch(&mut doc, [50.0, -5.0], [50.0, 15.0]);
    let top = pick(doc.evaluated(), a, TopoKind::Face, [10.0, 5.0, 10.0], 1);
    let f = add(
        &mut doc,
        FeatureBox::new(SplitFace::by_curves(vec![top], vec![line.clone()])),
    );
    let msg = error_of(&doc, f);
    assert!(msg.contains("Extend the tool"), "{msg}");
    let node = doc.evaluated().node(f).unwrap();
    let Health::Error(d) = &node.health else {
        unreachable!()
    };
    assert!(d.entities.contains(&line), "the curve is highlighted");
    assert_eq!(faces(&brep(&doc, a)), 6, "nothing changed");
    assert!(brep(&doc, a).count(SubKind::Face) == 6);
}

/// A cylinder r 5 about Z, `0..h`, as a new body.
fn cylinder(doc: &mut Document, h: f64) -> BodyId {
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    f.edit_sketch(|s| s.add_circle(Point2::new(0.0, 0.0), 5.0))
        .unwrap();
    let s = add(doc, FeatureBox::new(f));
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

#[test]
fn dw4_a_short_cylinder_face_fails_no_split_and_extended_cuts_a_core() {
    let mut doc = Document::new("cyl");
    let bx = block(&mut doc, [-20.0, -20.0, 20.0, 20.0], 10.0);
    let cyl = cylinder(&mut doc, 5.0);
    let side = pick(doc.evaluated(), cyl, TopoKind::Face, [0.0, 0.0, 2.5], 1);
    let tool = RefInput::Ref(EntityRef::Topo(side));
    let split = add(&mut doc, FeatureBox::new(SplitBody::new(bx, tool.clone())));
    let msg = error_of(&doc, split);
    assert!(msg.contains("Extend the tool"), "{msg}");
    let mut ext = SplitBody::new(bx, tool);
    ext.extend_tool = true;
    edit(&mut doc, split, FeatureBox::new(ext));
    ok(&doc, split);
    let core = std::f64::consts::PI * 25.0 * 10.0;
    let (p0, p1) = (
        brep(&doc, bx),
        brep(
            &doc,
            BodyId {
                origin: split,
                slot: 1,
            },
        ),
    );
    valid(&p0);
    valid(&p1);
    assert_rel(volume(&p0), 16_000.0 - core, 1e-6);
    assert_rel(volume(&p1), core, 1e-6);
    golden("split_extended_cylinder_core", &p1);
}

#[test]
fn dw3_split_face_by_a_projected_sketch_arc() {
    // A half circle r 5 about (10, 0) on the top of 0..20 × 0..10: a half disc of 12.5π.
    let mut doc = Document::new("arc");
    let a = block(&mut doc, [0.0, 0.0, 20.0, 10.0], 10.0);
    let mut f = SketchFeature::new(PlaneInput::OriginXY);
    let id = f
        .edit_sketch(|s| s.add_circle(Point2::new(10.0, 0.0), 5.0))
        .unwrap();
    let s = add(&mut doc, FeatureBox::new(f));
    let circle = EntityRef::SketchCurve {
        sketch: SketchId(s),
        curve: SketchEntityKey(u64::from(id.0)),
    };
    let top = pick(doc.evaluated(), a, TopoKind::Face, [10.0, 5.0, 10.0], 1);
    let sf = add(
        &mut doc,
        FeatureBox::new(SplitFace::by_curves(vec![top], vec![circle])),
    );
    ok(&doc, sf);
    let sa = brep(&doc, a);
    valid(&sa);
    let ar = areas_at_any(&sa, 10.0);
    let half = 12.5 * std::f64::consts::PI;
    assert_eq!(ar.len(), 2, "{ar:?}");
    assert_rel(ar[0], half, 1e-9);
    assert_rel(ar[1], 200.0 - half, 1e-9);
}

/// Areas of the faces lying in the plane z = `z` (by bbox), ascending.
fn areas_at_any(s: &amg_brep::Shape, z: f64) -> Vec<f64> {
    let [faces, _, _] = kernel().sub_geom_all(s).unwrap();
    let mut a: Vec<f64> = faces
        .iter()
        .filter(|g| (g.bbox.min.z - z).abs() < 1e-9 && (g.bbox.max.z - z).abs() < 1e-9)
        .map(|g| g.measure)
        .collect();
    a.sort_by(f64::total_cmp);
    a
}
