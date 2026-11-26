//! The OCCT half of the SOL-08 tests: body patterns, the 10×10 hole pattern (one boolean,
//! shared TShapes), mirrors, TNP-17/18, goldens `tests/golden/solid/sol08_*.toml` and a
//! proptest over random counts.

use std::f64::consts::{PI, TAU};

use amg_brep::{BrepKernel, KernelConfig, OcctKernel, Shape as BShape, Transform};
use amg_core::{Point2, Point3};
use amg_doc::naming::{GeomClass, KernelGeom, NameSeg, NamingView, default_kernel, topo_ref_at};
use amg_doc::{
    BodyChangeKind, BodyGeom, ComputeMode, Evaluated, GeomFingerprint, Health, MatchQuality,
    RecomputeCtx, ResolveError, TopoKind, TopoRef,
};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{Chamfer, Extent, Extrude};

use super::*;
use solid_common::counting::Counting;

// ---- helpers ----------------------------------------------------------------------------------

#[track_caller]
fn valid(s: &BShape) {
    let r = kernel().check(s, &amg_brep::CheckOpts::default());
    assert!(r.valid, "{r:?}");
}

#[track_caller]
fn golden(case: &str, s: &BShape, rel: f64) {
    valid(s);
    check_golden_as("sol08", rel, case, s);
}

#[track_caller]
fn ok(doc: &Document, f: FeatureId) {
    assert_eq!(health(doc, f), Health::Ok, "feature {f:?}");
}

/// Every live solid body: id and B-rep.
fn solids(doc: &Document) -> Vec<(BodyId, BShape)> {
    let mut v: Vec<(BodyId, BShape)> = doc
        .evaluated()
        .bodies()
        .filter_map(|(id, b)| match &b.geom {
            BodyGeom::Solid(s) => s.brep().map(|s| (id, s.clone())),
            _ => None,
        })
        .collect();
    v.sort_by_key(|(id, _)| *id);
    v
}

fn total_volume(doc: &Document) -> f64 {
    solids(doc).iter().map(|(_, s)| volume(s)).sum()
}

/// A 10 mm cube at `[x0, y0]` on XY.
fn cube(doc: &mut Document, x0: f64, y0: f64) -> BodyId {
    let (_, e) = extrude_rects(
        doc,
        &[[x0, y0, x0 + 10.0, y0 + 10.0]],
        10.0,
        BodyOp::NewBody,
        Vec::new(),
    );
    body0(e)
}

fn circle_sketch(center: [f64; 2], r: f64) -> SketchFeature {
    let mut sk = SketchFeature::new(PlaneInput::OriginXY);
    sk.edit_sketch(|s| {
        s.add_circle(Point2::new(center[0], center[1]), r)
            .map(|_| ())
    })
    .unwrap();
    sk
}

/// A circular hole cut "All" up from XY into `plate`.
fn hole(doc: &mut Document, plate: BodyId, center: [f64; 2], r: f64) -> FeatureId {
    let s = add(doc, FeatureBox::new(circle_sketch(center, r)));
    let mut e = Extrude::distance(
        profiles(doc, s),
        "1 mm",
        OpTarget {
            op: BodyOp::Cut,
            participants: vec![plate],
        },
    );
    e.side1.extent = Extent::All;
    let f = add(doc, FeatureBox::new(e));
    ok(doc, f);
    f
}

/// `$pick`: the entity of `kind` whose centroid is nearest to `near` (no ties).
fn pick(ev: &Evaluated, body: BodyId, kind: TopoKind, near: [f64; 3]) -> TopoRef {
    let geom = KernelGeom::new(ev, default_kernel());
    let near = Point3::from_array(near);
    let mut found: Vec<(f64, u32)> = amg_doc::naming::GeomSource::all(&geom, body, kind)
        .into_iter()
        .map(|(i, fp)| ((fp.centroid - near).length(), i))
        .collect();
    found.sort_by(|a, b| a.0.total_cmp(&b.0));
    assert!(
        found.len() > 1 && found[1].0 - found[0].0 > 1e-6,
        "tied pick near {near:?}"
    );
    let mut r = topo_ref_at(ev, default_kernel(), body, kind, found[0].1).unwrap();
    r.key = 1;
    r
}

fn resolution(doc: &Document, f: FeatureId) -> (MatchQuality, GeomFingerprint) {
    let node = doc.evaluated().node(f).unwrap();
    let rec = &node.resolutions[0];
    let q = rec.result.as_ref().expect("resolved")[0].quality;
    let Some(EntityRef::Topo(t)) = rec.current.first() else {
        panic!("{rec:?}");
    };
    (q, t.fingerprint.clone().unwrap())
}

fn centre(s: &BShape) -> Point3 {
    let b = s.bbox();
    (b.min + b.max) * 0.5
}

// ---- Done-when 1: body patterns -----------------------------------------------------------------

#[test]
fn dw1_rectangular_3x4_body_pattern() {
    let mut doc = Document::new("rect");
    let b = cube(&mut doc, 0.0, 0.0);
    let p = rect(
        PatternObjects::Bodies(vec![b]),
        dir(OriginEntity::XAxis, "3", "20 mm"),
        Some(dir(OriginEntity::YAxis, "4", "15 mm")),
    );
    let p = add(&mut doc, FeatureBox::new(p));
    ok(&doc, p);
    let all = solids(&doc);
    assert_eq!(all.len(), 12);
    assert_rel(total_volume(&doc), 12_000.0, 1e-12);
    for (id, s) in &all {
        valid(s);
        assert_eq!(faces(s), 6);
        if id.origin == p {
            let (i, j) = (id.slot & 0xffff, id.slot >> 16);
            let c = centre(s);
            let want = Point3::new(5.0 + 20.0 * f64::from(i), 5.0 + 15.0 * f64::from(j), 5.0);
            assert!((c - want).length() < 1e-9, "{id:?}: {c} vs {want}");
        }
    }
    let last = all
        .iter()
        .find(|(id, _)| {
            *id == BodyId {
                origin: p,
                slot: 2 + (3 << 16),
            }
        })
        .unwrap();
    golden("rect_body_last", &last.1, 1e-9);
}

#[test]
fn dw1_circular_6_instance_body_pattern() {
    let mut doc = Document::new("circ");
    let b = cube(&mut doc, 20.0, 0.0);
    let p = add(
        &mut doc,
        FeatureBox::new(circular(
            PatternObjects::Bodies(vec![b]),
            "6",
            AngleSpan::Full,
            false,
        )),
    );
    ok(&doc, p);
    let all = solids(&doc);
    assert_eq!(all.len(), 6);
    assert_rel(total_volume(&doc), 6000.0, 1e-12);
    for (id, s) in &all {
        valid(s);
        if id.origin == p {
            let a = TAU / 6.0 * f64::from(id.slot);
            let c = centre(s);
            // The box of a rotated cube is centred on the rotated centre.
            let want = Point3::new(
                25.0 * a.cos() - 5.0 * a.sin(),
                25.0 * a.sin() + 5.0 * a.cos(),
                5.0,
            );
            assert!((c - want).length() < 1e-9, "{id:?}: {c} vs {want}");
        }
    }
    let half = all
        .iter()
        .find(|(id, _)| *id == BodyId { origin: p, slot: 3 })
        .unwrap();
    golden("circ_body_k3", &half.1, 1e-9);
}

#[test]
fn dw1_symmetric_patterns_with_two_suppressed_instances() {
    // Rectangular: direction 1 symmetric (count 3 per side: 5), direction 2 count 2: 10
    // positions, 2 suppressed: 8 bodies.
    let mut doc = Document::new("sym-rect");
    let b = cube(&mut doc, 0.0, 0.0);
    let mut d1 = dir(OriginEntity::XAxis, "3", "20 mm");
    d1.symmetric = true;
    let mut p = rect(
        PatternObjects::Bodies(vec![b]),
        d1,
        Some(dir(OriginEntity::YAxis, "2", "20 mm")),
    );
    let gone = [0x8002, 1 + (1 << 16)];
    p.suppressed = gone.to_vec();
    let p = add(&mut doc, FeatureBox::new(p));
    ok(&doc, p);
    let all = solids(&doc);
    assert_eq!(all.len(), 8);
    assert_rel(total_volume(&doc), 8000.0, 1e-12);
    for k in gone {
        assert!(
            doc.evaluated()
                .body(BodyId { origin: p, slot: k })
                .is_none()
        );
    }
    let neg = doc
        .evaluated()
        .body(BodyId {
            origin: p,
            slot: 0x8001,
        })
        .expect("instance −1");
    let BodyGeom::Solid(s) = &neg.geom else {
        panic!()
    };
    assert!((centre(s.brep().unwrap()).x + 15.0).abs() < 1e-9);

    // Circular: 40° spacing, symmetric, count 4 per side (7), 2 suppressed: 5 bodies.
    let mut doc = Document::new("sym-circ");
    let b = cube(&mut doc, 20.0, 0.0);
    let mut p = circular(
        PatternObjects::Bodies(vec![b]),
        "4",
        AngleSpan::Spacing(ex("40 deg")),
        true,
    );
    p.suppressed = vec![2, 0x8000_0003];
    let p = add(&mut doc, FeatureBox::new(p));
    ok(&doc, p);
    assert_eq!(solids(&doc).len(), 5);
    assert_rel(total_volume(&doc), 5000.0, 1e-12);
}

#[test]
fn body_copies_are_named_per_instance() {
    let mut doc = Document::new("names");
    let b = cube(&mut doc, 0.0, 0.0);
    let p = add(
        &mut doc,
        FeatureBox::new(rect(
            PatternObjects::Bodies(vec![b]),
            dir(OriginEntity::XAxis, "2", "20 mm"),
            None,
        )),
    );
    let copy = BodyId { origin: p, slot: 1 };
    let n = NamingView::naming(&**doc.evaluated(), copy).unwrap();
    for f in &n.faces {
        assert!(
            f.segments().iter().any(|s| matches!(
                s,
                NameSeg::Inst { feature, instance: 1 } if feature == &p
            )),
            "{f:?}"
        );
    }
}

#[test]
fn a_bad_count_is_a_clean_error() {
    let mut doc = Document::new("bad");
    let b = cube(&mut doc, 0.0, 0.0);
    for count in ["0", "2.5", "20000"] {
        let p = add(
            &mut doc,
            FeatureBox::new(rect(
                PatternObjects::Bodies(vec![b]),
                dir(OriginEntity::XAxis, count, "20 mm"),
                None,
            )),
        );
        assert!(
            matches!(health(&doc, p), Health::Error(_)),
            "{count}: {:?}",
            health(&doc, p)
        );
    }
}

#[test]
fn an_empty_selection_fails_in_compute() {
    let mut doc = Document::new("empty");
    cube(&mut doc, 0.0, 0.0);
    let p = add(
        &mut doc,
        FeatureBox::new(rect(
            PatternObjects::Bodies(Vec::new()),
            dir(OriginEntity::XAxis, "2", "20 mm"),
            None,
        )),
    );
    let Health::Error(d) = health(&doc, p) else {
        panic!("{:?}", health(&doc, p));
    };
    assert!(d.message.contains("select at least one body"), "{d:?}");
}

// ---- Done-when 2: a 10×10 feature pattern of a Ø3 hole ------------------------------------------

/// A 100×100×5 plate with a Ø3 hole at (5, 5).
fn hole_plate(doc: &mut Document) -> (BodyId, FeatureId) {
    let (_, e) = extrude_rects(
        doc,
        &[[0.0, 0.0, 100.0, 100.0]],
        5.0,
        BodyOp::NewBody,
        Vec::new(),
    );
    let plate = body0(e);
    let h = hole(doc, plate, [5.0, 5.0], 1.5);
    (plate, h)
}

fn hole_pattern(h: FeatureId) -> Pattern {
    rect(
        PatternObjects::Features(vec![h]),
        dir(OriginEntity::XAxis, "10", "10 mm"),
        Some(dir(OriginEntity::YAxis, "10", "10 mm")),
    )
}

const HOLES_VOLUME: f64 = 50_000.0 - 100.0 * PI * 1.5 * 1.5 * 5.0;

#[test]
fn dw2_a_10x10_hole_pattern_has_the_analytic_volume() {
    let mut doc = Document::new("holes");
    let (plate, h) = hole_plate(&mut doc);
    let p = add(&mut doc, FeatureBox::new(hole_pattern(h)));
    ok(&doc, p);
    let s = brep(&doc, plate);
    assert_eq!(solids(&doc).len(), 1);
    assert_rel(volume(&s), HOLES_VOLUME, 1e-9);
    // 6 plate faces + one lateral face per hole.
    assert_eq!(faces(&s), 106);
    golden("hole_pattern_10x10", &s, 1e-9);
}

#[test]
fn dw2_one_boolean_call_and_instances_share_tshapes() {
    let mut doc = Document::new("holes-count");
    let (plate, h) = hole_plate(&mut doc);
    let pat = hole_pattern(h);
    let counting = Counting::new(OcctKernel::new(KernelConfig::default()));
    let mut ctx = RecomputeCtx::new(
        doc.design(),
        doc.evaluated(),
        FeatureId(9_999),
        doc.design().root(),
        ComputeMode::Full,
    )
    .with_kernel(&counting);
    let inst = pat.instances(&ctx).unwrap();
    assert_eq!(inst.len(), 99);
    let out = pat.compute(&mut ctx).unwrap();
    assert_eq!(counting.boolean_calls(), 1, "one boolean for 99 instances");
    assert_eq!(out.bodies.len(), 1);
    let BodyChangeKind::Modified {
        geom: BodyGeom::Solid(s),
        ..
    } = &out.bodies[0].change
    else {
        panic!("{:?}", out.bodies[0]);
    };
    assert_eq!(out.bodies[0].id, plate);
    assert_rel(volume(s.brep().unwrap()), HOLES_VOLUME, 1e-9);

    // The tool is one compound of 99 located copies: the instances share the seed tool's
    // TShapes, so the compound's binary form (shared TShapes written once) is a small fraction
    // of a compound of 99 geometry copies.
    let tools = counting.tools.lock().unwrap();
    assert_eq!(tools[0].len(), 1);
    let compound = &tools[0][0];
    let parts = compound.solids();
    assert_eq!(parts.len(), 99);
    for ((_, t), part) in inst.iter().zip(&parts) {
        let p = t.transform_point3(Point3::new(5.0, 5.0, 0.0));
        let c = centre(part);
        assert!(
            (c.x - p.x).abs() < 1e-9 && (c.y - p.y).abs() < 1e-9,
            "{c} vs {p}"
        );
    }
    let bytes = |s: &BShape| {
        let mut v = Vec::new();
        s.write_bin(&mut v).unwrap();
        v.len()
    };
    let copies: Vec<BShape> = parts
        .iter()
        .map(|p| p.gtransformed(&Transform::IDENTITY).unwrap().shape)
        .collect();
    let copied = kernel().make_compound(&copies).unwrap();
    let (located, apart) = (bytes(compound), bytes(&copied));
    println!("tool compound: located {located} B, geometry copies {apart} B");
    assert!(
        located * 4 < apart,
        "located {located} B vs copies {apart} B: the instances don't share TShapes"
    );
}

#[test]
fn suppressed_feature_instances_leave_their_holes_out() {
    let mut doc = Document::new("holes-suppressed");
    let (plate, h) = hole_plate(&mut doc);
    let mut p = hole_pattern(h);
    p.suppressed = vec![1, 3 + (4 << 16), 0];
    let p = add(&mut doc, FeatureBox::new(p));
    ok(&doc, p);
    let s = brep(&doc, plate);
    assert_rel(volume(&s), 50_000.0 - 98.0 * PI * 1.5 * 1.5 * 5.0, 1e-9);
}

#[test]
fn a_feature_pattern_follows_seed_edits() {
    let mut doc = Document::new("seed-edit");
    let (_, e) = extrude_rects(
        &mut doc,
        &[[0.0, 0.0, 100.0, 20.0]],
        5.0,
        BodyOp::NewBody,
        Vec::new(),
    );
    let plate = body0(e);
    let s = add(&mut doc, FeatureBox::new(circle_sketch([10.0, 10.0], 2.0)));
    let cut = |d: &str, doc: &Document| {
        Extrude::distance(
            profiles(doc, s),
            d,
            OpTarget {
                op: BodyOp::Cut,
                participants: vec![plate],
            },
        )
    };
    let first = FeatureBox::new(cut("2 mm", &doc));
    let h = add(&mut doc, first);
    let p = add(
        &mut doc,
        FeatureBox::new(rect(
            PatternObjects::Features(vec![h]),
            dir(OriginEntity::XAxis, "4", "20 mm"),
            None,
        )),
    );
    ok(&doc, p);
    let pocket = PI * 4.0;
    assert_rel(
        volume(&brep(&doc, plate)),
        10_000.0 - 4.0 * pocket * 2.0,
        1e-9,
    );
    let deeper = FeatureBox::new(cut("3 mm", &doc));
    edit(&mut doc, h, deeper);
    ok(&doc, p);
    assert_rel(
        volume(&brep(&doc, plate)),
        10_000.0 - 4.0 * pocket * 3.0,
        1e-9,
    );
}

#[test]
fn a_join_feature_pattern_fuses_the_instances() {
    // Bosses 10 high on a 100×20×5 plate, one N-ary fuse.
    let mut doc = Document::new("bosses");
    let (_, e) = extrude_rects(
        &mut doc,
        &[[0.0, 0.0, 100.0, 20.0]],
        5.0,
        BodyOp::NewBody,
        Vec::new(),
    );
    let plate = body0(e);
    let (_, boss) = extrude_rects(
        &mut doc,
        &[[5.0, 5.0, 15.0, 15.0]],
        10.0,
        BodyOp::Join,
        vec![plate],
    );
    let p = add(
        &mut doc,
        FeatureBox::new(rect(
            PatternObjects::Features(vec![boss]),
            dir(OriginEntity::XAxis, "5", "20 mm"),
            None,
        )),
    );
    ok(&doc, p);
    assert_eq!(solids(&doc).len(), 1);
    let s = brep(&doc, plate);
    assert_rel(volume(&s), 10_000.0 + 5.0 * 500.0, 1e-9);
    golden("boss_pattern_join", &s, 1e-9);
}

#[test]
fn a_non_patternable_seed_fails_cleanly() {
    let mut doc = Document::new("np");
    let s = add(
        &mut doc,
        FeatureBox::new(rect_sketch(&[[0.0, 0.0, 10.0, 10.0]])),
    );
    let p = add(
        &mut doc,
        FeatureBox::new(rect(
            PatternObjects::Features(vec![s]),
            dir(OriginEntity::XAxis, "2", "20 mm"),
            None,
        )),
    );
    let Health::Error(d) = health(&doc, p) else {
        panic!("{:?}", health(&doc, p));
    };
    assert!(d.message.contains("can't be patterned"), "{d:?}");
}

// ---- Done-when 3: mirror -----------------------------------------------------------------------

/// An L-shaped prism (not symmetric about any origin plane) at x ∈ [10, 40].
fn l_body(doc: &mut Document, x0: f64) -> BodyId {
    let pts = [
        [x0, 0.0],
        [x0 + 30.0, 0.0],
        [x0 + 30.0, 10.0],
        [x0 + 10.0, 10.0],
        [x0 + 10.0, 25.0],
        [x0, 25.0],
    ];
    let (sk, _, _) = polygon_sketch(&pts);
    let s = add(doc, FeatureBox::new(sk));
    let e = add(
        doc,
        FeatureBox::new(Extrude::distance(
            profiles(doc, s),
            "8 mm",
            OpTarget::default(),
        )),
    );
    ok(doc, e);
    body0(e)
}

const L_VOLUME: f64 = (30.0 * 10.0 + 10.0 * 15.0) * 8.0;

#[test]
fn dw3_mirror_of_a_body_as_a_new_body() {
    let mut doc = Document::new("mirror-new");
    let b = l_body(&mut doc, 10.0);
    let m = add(
        &mut doc,
        FeatureBox::new(Mirror {
            objects: PatternObjects::Bodies(vec![b]),
            plane: RefInput::Origin(OriginEntity::YZPlane),
            op: BodyOp::NewBody,
            target: None,
        }),
    );
    ok(&doc, m);
    let copy = brep(&doc, BodyId { origin: m, slot: 0 });
    valid(&copy);
    let props = kernel().mass_props(&copy).unwrap();
    assert!(props.volume > 0.0, "faces still face out of the material");
    assert_rel(props.volume, L_VOLUME, 1e-12);
    let src = brep(&doc, b);
    let (a, c) = (src.bbox(), copy.bbox());
    assert!((c.min.x + a.max.x).abs() < 1e-9 && (c.max.x + a.min.x).abs() < 1e-9);
    assert!((c.max.y - a.max.y).abs() < 1e-9);
    let n = NamingView::naming(&**doc.evaluated(), BodyId { origin: m, slot: 0 }).unwrap();
    assert!(n.faces.iter().all(|f| {
        f.segments()
            .iter()
            .any(|s| matches!(s, NameSeg::Inst { feature, instance: 1 } if feature == &m))
    }));
    golden("mirror_body_new", &copy, 1e-9);
}

#[test]
fn dw3_mirror_join_fuses_with_the_source() {
    // The L touches the YZ plane, so the mirror joins into one T-like body.
    let mut doc = Document::new("mirror-join");
    let b = l_body(&mut doc, 0.0);
    let m = add(
        &mut doc,
        FeatureBox::new(Mirror {
            objects: PatternObjects::Bodies(vec![b]),
            plane: RefInput::Origin(OriginEntity::YZPlane),
            op: BodyOp::Join,
            target: None,
        }),
    );
    ok(&doc, m);
    assert_eq!(solids(&doc).len(), 1);
    let s = brep(&doc, b);
    valid(&s);
    assert_rel(volume(&s), 2.0 * L_VOLUME, 1e-12);
    // The coplanar faces on x = 0 are gone and the halves' coplanar faces merged: a prism
    // over an 8-gon (T profile): 8 laterals + 2 caps.
    assert_eq!(faces(&s), 10);
    golden("mirror_body_join", &s, 1e-9);
}

#[test]
fn dw3_mirror_of_an_extrude_boss() {
    let mut doc = Document::new("mirror-boss");
    let (_, e) = extrude_rects(
        &mut doc,
        &[[-50.0, -50.0, 50.0, 50.0]],
        5.0,
        BodyOp::NewBody,
        Vec::new(),
    );
    let plate = body0(e);
    let (_, boss) = extrude_rects(
        &mut doc,
        &[[10.0, 10.0, 20.0, 30.0]],
        15.0,
        BodyOp::Join,
        vec![plate],
    );
    let before = volume(&brep(&doc, plate));
    assert_rel(before, 50_000.0 + 2000.0, 1e-12);
    let m = add(
        &mut doc,
        FeatureBox::new(Mirror {
            objects: PatternObjects::Features(vec![boss]),
            plane: RefInput::Origin(OriginEntity::YZPlane),
            op: BodyOp::NewBody,
            target: None,
        }),
    );
    ok(&doc, m);
    assert_eq!(solids(&doc).len(), 1);
    let s = brep(&doc, plate);
    valid(&s);
    assert_rel(volume(&s), 50_000.0 + 4000.0, 1e-12);
    let b = s.bbox();
    assert!((b.max.z - 15.0).abs() < 1e-9);
    golden("mirror_feature_boss", &s, 1e-9);
}

#[test]
fn a_mirrored_cut_feature_cuts_the_other_side() {
    let mut doc = Document::new("mirror-cut");
    let (_, e) = extrude_rects(
        &mut doc,
        &[[-50.0, -20.0, 50.0, 20.0]],
        10.0,
        BodyOp::NewBody,
        Vec::new(),
    );
    let plate = body0(e);
    let h = hole(&mut doc, plate, [30.0, 0.0], 4.0);
    let m = add(
        &mut doc,
        FeatureBox::new(Mirror {
            objects: PatternObjects::Features(vec![h]),
            plane: RefInput::Origin(OriginEntity::YZPlane),
            op: BodyOp::NewBody,
            target: None,
        }),
    );
    ok(&doc, m);
    let s = brep(&doc, plate);
    valid(&s);
    assert_rel(volume(&s), 40_000.0 - 2.0 * PI * 16.0 * 10.0, 1e-9);
    let c = pick(doc.evaluated(), plate, TopoKind::Edge, [-30.0, 0.0, 10.0]);
    let r = doc.evaluated().resolve(&c).unwrap();
    assert_eq!(r.quality, MatchQuality::Exact);
}

// ---- Done-when 4: TNP-17 and TNP-18 (shared script S17) ------------------------------------------

/// S17: a plate (−50,−50)-(50,50) 10 mm, a through-hole r 4 at (30, 0), a circular feature
/// pattern of the hole about Z (count 6), and a 0.5 mm chamfer on instance 3's top edge at
/// (−30, 0, 10). Returns (pattern, chamfer).
fn s17(doc: &mut Document) -> (FeatureId, FeatureId) {
    let (_, e) = extrude_rects(
        doc,
        &[[-50.0, -50.0, 50.0, 50.0]],
        10.0,
        BodyOp::NewBody,
        Vec::new(),
    );
    let plate = body0(e);
    let h = hole(doc, plate, [30.0, 0.0], 4.0);
    let pat = add(
        doc,
        FeatureBox::new(circular(
            PatternObjects::Features(vec![h]),
            "6",
            AngleSpan::Full,
            false,
        )),
    );
    ok(doc, pat);
    let edge = pick(doc.evaluated(), plate, TopoKind::Edge, [-30.0, 0.0, 10.0]);
    let cha = add(doc, FeatureBox::new(Chamfer::equal(vec![edge], "0.5 mm")));
    ok(doc, cha);
    (pat, cha)
}

fn with_count(doc: &Document, pat: FeatureId, count: &str) -> FeatureBox {
    let node = doc.design().timeline().node(pat).unwrap().1;
    let mut p = node.feature.downcast_ref::<Pattern>().unwrap().clone();
    let PatternKind::Circular { count: c, .. } = &mut p.kind else {
        panic!()
    };
    *c = ex(count);
    FeatureBox::new(p)
}

#[test]
fn dw4_tnp17_pattern_count_6_to_8_follows_instance_3() {
    let mut doc = Document::new("tnp17");
    let (pat, cha) = s17(&mut doc);
    let (q, fp) = resolution(&doc, cha);
    assert_eq!(q, MatchQuality::Exact);
    assert!(
        (fp.centroid - Point3::new(-30.0, 0.0, 10.0)).length() < 1e-6,
        "{fp:?}"
    );
    let f = with_count(&doc, pat, "8");
    edit(&mut doc, pat, f);
    ok(&doc, pat);
    ok(&doc, cha);
    let (q, fp) = resolution(&doc, cha);
    assert_eq!(q, MatchQuality::Exact);
    assert_eq!(fp.class, GeomClass::Circle);
    let want = Point3::new(-21.21320, 21.21320, 10.0);
    assert!((fp.centroid - want).length() < 1e-4, "{fp:?}");
    assert!((fp.measure - 25.13274).abs() < 1e-4, "{fp:?}");
}

#[test]
fn dw4_tnp18_pattern_count_6_to_3_loses_instance_3() {
    let mut doc = Document::new("tnp18");
    let (pat, cha) = s17(&mut doc);
    let f = with_count(&doc, pat, "3");
    edit(&mut doc, pat, f);
    ok(&doc, pat);
    let node = doc.evaluated().node(cha).unwrap();
    assert!(
        matches!(
            node.resolutions[0].result,
            Err(ResolveError::NotFound { .. })
        ),
        "{:?}",
        node.resolutions[0]
    );
    assert!(matches!(node.health, Health::Error(_)), "{:?}", node.health);
}

// ---- proptest: random counts ---------------------------------------------------------------------

mod random {
    use proptest::prelude::*;

    use super::*;

    proptest! {
        #![proptest_config(ProptestConfig { cases: 16, ..ProptestConfig::default() })]

        /// Random counts (incl. 0 and fractions) and spacings on a body and a feature
        /// pattern: valid solids or a clean error, never a panic.
        #[test]
        fn random_counts_give_valid_solids_or_clean_errors(
            n1 in -1i32..6,
            n2 in 0i32..4,
            frac in prop::bool::weighted(0.15),
            spacing in -30.0f64..30.0,
            circular_kind in any::<bool>(),
        ) {
            let mut doc = Document::new("random");
            let (_, e) = extrude_rects(&mut doc, &[[0.0, 0.0, 100.0, 100.0]], 5.0, BodyOp::NewBody, Vec::new());
            let plate = body0(e);
            let h = hole(&mut doc, plate, [50.0, 50.0], 2.0);
            let count = if frac { format!("{}.5", n1) } else { n1.to_string() };
            let sp = format!("{spacing} mm");
            let p = if circular_kind {
                circular(PatternObjects::Features(vec![h]), &count, AngleSpan::Spacing(ex(&format!("{spacing} deg"))), n2 % 2 == 1)
            } else {
                rect(
                    PatternObjects::Features(vec![h]),
                    dir(OriginEntity::XAxis, &count, &sp),
                    Some(dir(OriginEntity::YAxis, &n2.max(1).to_string(), &sp)),
                )
            };
            let p = add(&mut doc, FeatureBox::new(p));
            match health(&doc, p) {
                Health::Ok | Health::Warning(_) => {
                    let s = brep(&doc, plate);
                    prop_assert!(kernel().check(&s, &amg_brep::CheckOpts::default()).valid);
                }
                Health::Error(d) => prop_assert!(!d.message.is_empty()),
                other => prop_assert!(false, "{other:?}"),
            }
        }
    }
}
