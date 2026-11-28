//! The OCCT half of the SOL-24 tests: the body of a "New component" extrude, the bodies of a
//! component pattern and mirror in world space, and cut / join / split of a body in a
//! sub-component from the root (assembly context), goldens `tests/golden/solid/sol24_*.toml`.

use amg_brep::{BrepKernel, Shape as BShape};
use amg_core::Point3;
use amg_doc::{BodyId, FeatureBox, FeatureId, Health};
use amg_features::solid::{Extrude, OpTarget, SolidError};

use super::*;

#[track_caller]
fn valid(s: &BShape) {
    let r = kernel().check(s, &amg_brep::CheckOpts::default());
    assert!(r.valid, "{r:?}");
}

#[track_caller]
fn ok(doc: &Document, f: FeatureId) {
    assert_eq!(health(doc, f), Health::Ok, "feature {f:?}");
}

/// Adds `f` in component `context` and recomputes.
fn add_in(doc: &mut Document, f: FeatureBox, context: ComponentId) -> FeatureId {
    let o = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context,
            name: None,
        })
        .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    created_feature(&o)
}

/// A `w × w × h` box at the origin of `context` (sketch + extrude there); its body.
fn box_in(doc: &mut Document, context: ComponentId, w: f64, h: f64) -> BodyId {
    let s = add_in(
        doc,
        FeatureBox::new(rect_sketch(&[[0.0, 0.0, w, w]])),
        context,
    );
    let e = Extrude::distance(profiles(doc, s), &format!("{h} mm"), OpTarget::default());
    let e = add_in(doc, FeatureBox::new(e), context);
    ok(doc, e);
    body0(e)
}

/// A root extrude of the rectangle `r` (XY, z = 0) up by 30 mm with `op` on `participants`.
fn root_extrude(
    doc: &mut Document,
    r: [f64; 4],
    op: BodyOp,
    participants: Vec<BodyId>,
) -> FeatureId {
    let root = doc.design().root();
    let s = add_in(doc, FeatureBox::new(rect_sketch(&[r])), root);
    let e = Extrude::distance(profiles(doc, s), "30 mm", OpTarget { op, participants });
    add_in(doc, FeatureBox::new(e), root)
}

fn centroid(s: &BShape) -> Point3 {
    kernel().mass_props(s).unwrap().centroid
}

#[track_caller]
fn near(got: Point3, want: Point3, tol: f64) -> f64 {
    let d = (got - want).length();
    assert!(d <= tol, "got {got:?}, want {want:?} ({d:e})");
    d
}

// ---- Done-when 1 ------------------------------------------------------------------------------

#[test]
fn dw1_new_component_extrude_makes_one_body_in_the_new_component() {
    let mut doc = Document::new("sol24-dw1-occt");
    let root = doc.design().root();
    let s = add(
        &mut doc,
        FeatureBox::new(rect_sketch(&[[0.0, 0.0, 10.0, 20.0]])),
    );
    let profs = profiles(&doc, s);
    let reg = dialog::registration("solid.extrude").unwrap();
    let v = extrude_values(&doc, &profs, "new_component");
    let bodies_before = doc.evaluated().bodies().count();
    let undo_before = doc.history().stack().applied();
    let cmds = dialog::new_feature_commands(reg, &v, doc.design(), root).unwrap();
    let created = commit(&mut doc, "Extrude", cmds);
    doc.recompute_blocking(jobs()).unwrap();
    assert_eq!(doc.history().stack().applied(), undo_before + 1);
    let c = created
        .iter()
        .find_map(|x| match x {
            CreatedId::Component(c) => Some(*c),
            _ => None,
        })
        .unwrap();
    let f = created
        .iter()
        .find_map(|x| match x {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .unwrap();
    ok(&doc, f);
    assert_eq!(
        doc.evaluated().bodies().count(),
        bodies_before + 1,
        "one body"
    );
    let b = body0(f);
    assert_eq!(doc.evaluated().body(b).unwrap().component, c);
    let in_c: Vec<_> = doc
        .evaluated()
        .bodies()
        .filter(|(_, x)| x.component == c)
        .collect();
    assert_eq!(in_c.len(), 1, "one body in the new component");
    let shape = brep(&doc, b);
    valid(&shape);
    let vol = volume(&shape);
    assert_rel(vol, 2000.0, 1e-9);
    // Placed at the identity: the body sits on its sketch.
    let bb = shape.bbox();
    assert!((bb.min - Point3::ZERO).length() < 1e-9);
    assert!((bb.max - Point3::new(10.0, 20.0, 10.0)).length() < 1e-9);
    println!("dw1: component {c}, body {b:?}, volume {vol} mm³, undo entries +1");
    assert!(doc.undo());
    doc.recompute_blocking(jobs()).unwrap();
    assert!(doc.evaluated().body(b).is_none());
    assert_eq!(doc.evaluated().bodies().count(), bodies_before);
}

// ---- Done-when 2 ------------------------------------------------------------------------------

#[test]
fn dw2_pattern_and_mirror_instances_place_the_component_body() {
    let mut doc = Document::new("sol24-dw2-occt");
    let root = doc.design().root();
    let (c, occ) = new_component(&mut doc, root, "Blade", seed_placement());
    let b = box_in(&mut doc, c, 10.0, 4.0);
    let local = centroid(&brep(&doc, b));
    let cmds =
        pattern_occurrences(doc.design(), doc.evaluated(), &circular_of(occ, "6"), root).unwrap();
    commit(&mut doc, "Pattern", cmds);
    let m = Mirror {
        objects: PatternObjects::Occurrences(vec![occ]),
        plane: RefInput::Origin(OriginEntity::XZPlane),
        op: BodyOp::NewBody,
        target: None,
    };
    let cmds = mirror_occurrences(doc.design(), doc.evaluated(), &m, root).unwrap();
    commit(&mut doc, "Mirror", cmds);
    doc.recompute_blocking(jobs()).unwrap();
    let occs = occurrences_of(&doc, c);
    assert_eq!(occs.len(), 7, "seed + 5 pattern instances + 1 mirror");
    let seed_world = seed_placement().transform_point3(local);
    let mut want: Vec<Point3> = (0..6u32)
        .map(|k| Transform::from_rotation_z(f64::from(k) * PI / 3.0).transform_point3(seed_world))
        .collect();
    want.push(Point3::new(seed_world.x, -seed_world.y, seed_world.z));
    let mut worst: f64 = 0.0;
    for (id, _) in &occs {
        let w = doc.evaluated().body_to_world(b, &[*id]).unwrap();
        let got = w.transform_point3(local);
        let d = want
            .iter()
            .map(|p| (*p - got).length())
            .fold(f64::INFINITY, f64::min);
        assert!(
            d < 1e-9,
            "occurrence {id}: body centre {got:?} not at an instance"
        );
        worst = worst.max(d);
    }
    // One component definition: the pattern added no body.
    assert_eq!(doc.evaluated().bodies().count(), 1);
    println!("dw2 world body centres: worst deviation {worst:e} mm over 7 occurrences");
}

// ---- Done-when 3 ------------------------------------------------------------------------------

/// The sub-component placement of the DW3 tests: 90° about Z, then (50, 20, 5). Local
/// `(x, y, z)` is world `(50 − y, 20 + x, 5 + z)`.
fn sub_placement(dx: f64) -> Transform {
    Transform::from_translation(Vec3::new(50.0 + dx, 20.0, 5.0))
        * Transform::from_rotation_z(PI / 2.0)
}

#[test]
fn dw3_cut_in_assembly_context_cuts_the_sub_component_body_and_undoes_in_one_step() {
    let mut doc = Document::new("sol24-dw3");
    let root = doc.design().root();
    let (sub, occ) = new_component(&mut doc, root, "Sub", sub_placement(0.0));
    let b = box_in(&mut doc, sub, 40.0, 10.0);
    assert_rel(volume(&brep(&doc, b)), 16_000.0, 1e-12);
    // A 10 × 10 hole at local (30, 20) = world (30, 50), cut from a root sketch.
    let sketch = add_in(
        &mut doc,
        FeatureBox::new(rect_sketch(&[[25.0, 45.0, 35.0, 55.0]])),
        root,
    );
    let undo_before = doc.history().stack().applied();
    let e = Extrude::distance(
        profiles(&doc, sketch),
        "30 mm",
        OpTarget {
            op: BodyOp::Cut,
            participants: vec![b],
        },
    );
    let cut = add_in(&mut doc, FeatureBox::new(e), root);
    ok(&doc, cut);
    assert_eq!(
        doc.history().stack().applied(),
        undo_before + 1,
        "one entry"
    );
    let s = brep(&doc, b);
    valid(&s);
    assert_eq!(
        doc.evaluated().body(b).unwrap().component,
        sub,
        "still in Sub"
    );
    let vol = volume(&s);
    assert_rel(vol, 15_000.0, 1e-9);
    assert_eq!(faces(&s), 10);
    // Centroid in Sub's space: x = (16000·20 − 1000·30) / 15000, y = 20.
    let d1 = near(centroid(&s), Point3::new(290.0 / 15.0, 20.0, 5.0), 1e-9);
    check_golden_as("sol24", 1e-9, "cut_sub_component", &s);
    // Moving the occurrence (+5 mm along world X) recomputes the cut: the world hole is now
    // at local (30, 25), so y = (16000·20 − 1000·25) / 15000.
    doc.apply(Command::SetOccurrenceTransform {
        occ,
        transform: sub_placement(5.0),
    })
    .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    ok(&doc, cut);
    let moved = brep(&doc, b);
    let d2 = near(
        centroid(&moved),
        Point3::new(290.0 / 15.0, 295.0 / 15.0, 5.0),
        1e-9,
    );
    assert_rel(volume(&moved), 15_000.0, 1e-9);
    assert!(doc.undo(), "undo the move");
    assert!(doc.undo(), "undo the cut in one step");
    doc.recompute_blocking(jobs()).unwrap();
    assert!(doc.design().timeline().node(cut).is_none());
    assert_rel(volume(&brep(&doc, b)), 16_000.0, 1e-12);
    println!(
        "dw3: cut volume {vol} mm³ (15000), centroid deviation {d1:e} / after the move {d2:e}"
    );
}

#[test]
fn dw3_a_slot_through_the_sub_component_body_keeps_both_pieces_there() {
    let mut doc = Document::new("sol24-dw3-split");
    let root = doc.design().root();
    let (sub, _) = new_component(&mut doc, root, "Sub", sub_placement(0.0));
    let b = box_in(&mut doc, sub, 40.0, 10.0);
    // Local x ∈ [18, 22] across the whole box = world y ∈ [38, 42].
    let cut = root_extrude(&mut doc, [5.0, 38.0, 55.0, 42.0], BodyOp::Cut, vec![b]);
    ok(&doc, cut);
    let piece = BodyId {
        origin: cut,
        slot: 1,
    };
    for id in [b, piece] {
        let x = doc.evaluated().body(id).expect("both pieces");
        assert_eq!(x.component, sub, "{id:?} stays in Sub");
        let s = brep(&doc, id);
        valid(&s);
        assert_rel(volume(&s), 7200.0, 1e-9);
    }
    assert_eq!(doc.evaluated().successors(b), &[piece][..]);
    check_golden_as(
        "sol24",
        1e-9,
        "split_sub_component_piece1",
        &brep(&doc, piece),
    );
}

#[test]
fn dw3_join_and_errors_in_assembly_context() {
    let mut doc = Document::new("sol24-dw3-join");
    let root = doc.design().root();
    let (sub, _) = new_component(&mut doc, root, "Sub", sub_placement(0.0));
    let b = box_in(&mut doc, sub, 40.0, 10.0);
    // A 10 × 10 × 30 boss from z = 0 overlapping the box (world z 5..15) is fused into it:
    // 16000 + 2000 (the parts below and above the box).
    let join = root_extrude(&mut doc, [25.0, 45.0, 35.0, 55.0], BodyOp::Join, vec![b]);
    ok(&doc, join);
    let s = brep(&doc, b);
    valid(&s);
    assert_rel(volume(&s), 16_000.0 + 2000.0, 1e-9);
    assert_eq!(doc.evaluated().body(b).unwrap().component, sub);
    // Placed twice below the root: ambiguous.
    doc.apply(Command::AddOccurrence {
        parent: root,
        component: sub,
        transform: Transform::from_translation(Vec3::new(200.0, 0.0, 0.0)),
        name: None,
    })
    .unwrap();
    doc.recompute_blocking(jobs()).unwrap();
    let Health::Error(d) = health(&doc, join) else {
        panic!("ambiguous placement must fail: {:?}", health(&doc, join));
    };
    let want = SolidError::AmbiguousPlacement {
        body: b,
        placements: 2,
    };
    assert!(d.message.contains(&want.to_string()), "{}", d.message);
    assert!(doc.undo());
    doc.recompute_blocking(jobs()).unwrap();
    ok(&doc, join);
    // A body outside the active component's subtree is refused.
    let (other, _) = new_component(&mut doc, root, "Other", Transform::IDENTITY);
    let ob = box_in(&mut doc, other, 5.0, 5.0);
    let in_sub = add_in(
        &mut doc,
        FeatureBox::new(rect_sketch(&[[0.0, 0.0, 1.0, 1.0]])),
        sub,
    );
    let e = Extrude::distance(
        profiles(&doc, in_sub),
        "3 mm",
        OpTarget {
            op: BodyOp::Cut,
            participants: vec![ob],
        },
    );
    let e = add_in(&mut doc, FeatureBox::new(e), sub);
    let Health::Error(d) = health(&doc, e) else {
        panic!("a sibling's body must fail");
    };
    assert!(
        d.message
            .contains(&SolidError::OtherComponent(ob).to_string()),
        "{}",
        d.message
    );
}
