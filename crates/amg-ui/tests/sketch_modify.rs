//! UI-16 sketch modify tools, kittest on the real app with the `NullViewport` and
//! `CpuPickService`.
//!
//! Done-when 1: one test per tool. Each loads a small fixture, drives the tool with clicks,
//! keys and dialog fields, and checks that
//! - the session's newest [`ModifyOp`] names the picked entities and positions,
//! - replaying that op with the S03 edit operation on the sketch as it was gives the same
//!   sketch as the tool (entities, constraints, values within 1e-6), and
//! - the outcome is the one the S03 fixture of that op describes (pieces, constraints,
//!   dimensions), in exactly one undo entry labelled with the tool.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::time::{Duration, Instant};

use amg_core::{Point2, Point3};
use amg_doc::{Command, FeatureBox};
use amg_features::sketch::SketchFeature;
use amg_render::{Camera, Projection};
use amg_sketch::{ConstraintKind, Dimension, DimensionKind, EntityId, EntityKind, Sketch, build};
use amg_ui::CommitBatch;
use amg_ui::sketch::modify::field;
use amg_ui::sketch::{
    CREATE_SKETCH_CMD, ModifyKind, ModifyOp, SKETCH_SELECT_CMD, SketchLayer, SketchSession,
};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable as _;

const PRIM: egui::PointerButton = egui::PointerButton::Primary;
const NONE: egui::Modifiers = egui::Modifiers::NONE;

fn pt(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn session(h: &AppHarness) -> &SketchSession {
    h.app()
        .doc_layer::<SketchLayer>()
        .expect("sketch layer")
        .session()
        .expect("an open sketch session")
}

fn sketch(h: &AppHarness) -> Sketch {
    session(h).working().sketch.clone()
}

fn applied(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

/// A new XY sketch seen from the top.
fn open() -> AppHarness {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h.run_command(CREATE_SKETCH_CMD);
    h.kittest().get_by_label("XY").click();
    h.drive_until_idle();
    h.app_mut().active_document_mut().camera = Camera {
        eye: Point3::new(10.0, 10.0, 150.0),
        target: Point3::new(10.0, 10.0, 0.0),
        up: amg_core::Dir3::Y,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    h.drive_until_idle();
    h
}

/// Steps until the session has no solver work left (a focused field keeps asking for frames).
fn settle(h: &mut AppHarness) {
    let start = Instant::now();
    loop {
        h.kittest().step();
        let s = session(h);
        if !s.is_solving() && h.app().jobs().jobs().is_empty() && h.document().is_evaluated() {
            for _ in 0..3 {
                h.kittest().step();
            }
            return;
        }
        assert!(
            start.elapsed() < Duration::from_secs(20),
            "settle: still solving"
        );
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn click(h: &mut AppHarness, p: Point2) {
    let s = h.world_to_screen(Point3::new(p.x, p.y, 0.0));
    h.click_at(s, PRIM, NONE);
    settle(h);
}

fn key(h: &mut AppHarness, k: egui::Key) {
    h.key(k);
    settle(h);
}

/// Replaces the text of dialog field `label` (without committing; the next click does).
fn type_into(h: &mut AppHarness, label: &str, text: &str) {
    h.kittest().get_by_label(label).focus();
    h.kittest().step();
    h.kittest()
        .key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
    h.kittest().step();
    h.kittest().get_by_label(label).type_text(text);
    h.kittest().step();
}

/// Loads the sketch `build` makes (one undo entry) and returns what it returned.
fn load<R>(h: &mut AppHarness, build: impl FnOnce(&mut Sketch) -> R) -> R {
    let mut f = session(h).working().clone();
    let r = build(&mut f.sketch);
    let n = f.sketch.entities().count();
    let id = session(h).feature();
    h.app_mut().submit(CommitBatch {
        label: "Fixture".into(),
        commands: vec![Command::EditFeature {
            id,
            feature: FeatureBox::new(f),
        }],
    });
    h.drive_until_idle();
    settle(h);
    assert_eq!(sketch(h).entities().count(), n, "fixture loaded");
    r
}

/// Starts `kind` and returns the feature as it is before the tool acts.
fn start(h: &mut AppHarness, kind: ModifyKind) -> SketchFeature {
    h.run_command(kind.def().id);
    settle(h);
    assert_eq!(h.app().active_tool(), Some(kind.def().id));
    session(h).working().clone()
}

/// JSON values equal, numbers within `tol`.
fn same_json(
    a: &serde_json::Value,
    b: &serde_json::Value,
    tol: f64,
    path: &str,
) -> Result<(), String> {
    use serde_json::Value as V;
    match (a, b) {
        (V::Number(x), V::Number(y)) => {
            let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
            if (x - y).abs() <= tol * (1.0 + x.abs()) {
                Ok(())
            } else {
                Err(format!("{path}: {x} != {y}"))
            }
        }
        (V::Array(x), V::Array(y)) => {
            if x.len() != y.len() {
                return Err(format!("{path}: length {} != {}", x.len(), y.len()));
            }
            for (i, (p, q)) in x.iter().zip(y).enumerate() {
                same_json(p, q, tol, &format!("{path}[{i}]"))?;
            }
            Ok(())
        }
        (V::Object(x), V::Object(y)) => {
            let kx: Vec<_> = x.keys().collect();
            let ky: Vec<_> = y.keys().collect();
            if kx != ky {
                return Err(format!("{path}: keys {kx:?} != {ky:?}"));
            }
            for (k, p) in x {
                same_json(p, &y[k], tol, &format!("{path}.{k}"))?;
            }
            Ok(())
        }
        _ if a == b => Ok(()),
        _ => Err(format!("{path}: {a} != {b}")),
    }
}

/// The tool made exactly one undo entry, and replaying its op with the S03 operation on
/// `before` gives the working copy. Returns the op.
fn check_replay(
    h: &AppHarness,
    kind: ModifyKind,
    before: &SketchFeature,
    undo_before: usize,
) -> ModifyOp {
    assert_eq!(applied(h), undo_before + 1, "one undo entry");
    assert_eq!(h.document().undo_label(), Some(kind.def().label));
    let op = session(h).last_modify.clone().expect("an op");
    let mut expected = before.clone();
    op.apply(&mut expected).expect("the op replays");
    let got = serde_json::to_value(sketch(h)).unwrap();
    let want = serde_json::to_value(&expected.sketch).unwrap();
    if let Err(e) = same_json(&want, &got, 1e-6, "sketch") {
        panic!("{kind:?}: the tool's sketch differs from the S03 op's: {e}");
    }
    let got_dims: Vec<_> = session(h).working().dims.keys().copied().collect();
    let want_dims: Vec<_> = expected.dims.keys().copied().collect();
    assert_eq!(got_dims, want_dims, "driving dimensions");
    assert_eq!(
        h.app().active_tool(),
        Some(kind.def().id),
        "the tool stays active"
    );
    op
}

fn lines(s: &Sketch) -> Vec<(EntityId, Point2, Point2)> {
    s.entities()
        .filter_map(|(id, e)| match e.kind {
            EntityKind::Line { start, end } => Some((id, s.point(start)?, s.point(end)?)),
            _ => None,
        })
        .collect()
}

fn count(s: &Sketch, f: impl Fn(&EntityKind) -> bool) -> usize {
    s.entities().filter(|(_, e)| f(&e.kind)).count()
}

fn circles(s: &Sketch) -> Vec<(Point2, f64)> {
    s.entities()
        .filter_map(|(id, e)| match e.kind {
            EntityKind::Circle { center, .. } => {
                let c = s.curve(id)?;
                let amg_sketch::Curve2::Circle(k) = c else {
                    return None;
                };
                Some((s.point(center)?, k.radius))
            }
            _ => None,
        })
        .collect()
}

fn near(a: Point2, b: Point2) -> bool {
    a.distance(b) < 1e-6
}

fn dims(s: &Sketch) -> Vec<Dimension> {
    s.constraints()
        .filter_map(|(_, c)| c.kind.as_dimension().cloned())
        .collect()
}

/// Two crossing lines: horizontal (0,0)–(40,0) and vertical (20,-10)–(20,10).
fn cross(s: &mut Sketch) -> (EntityId, EntityId) {
    let a = s.add_line(pt(0.0, 0.0), pt(40.0, 0.0)).unwrap();
    let b = s.add_line(pt(20.0, -10.0), pt(20.0, 10.0)).unwrap();
    (a, b)
}

/// The 2-point rectangle (0,0)–(40,30) of the SK-21 builder.
fn rectangle(s: &mut Sketch) -> Vec<EntityId> {
    build::rectangle_2pt(s, pt(0.0, 0.0), pt(40.0, 30.0))
        .unwrap()
        .entities
}

// ---- Done-when 1: one test per tool ----------------------------------------------------------

#[test]
fn trim_removes_the_clicked_part_between_intersections() {
    let mut h = open();
    let (a, b) = load(&mut h, cross);
    let before = start(&mut h, ModifyKind::Trim);
    let undo = applied(&h);
    click(&mut h, pt(30.0, 0.0));
    let op = check_replay(&h, ModifyKind::Trim, &before, undo);
    assert!(matches!(op, ModifyOp::Trim { entity, .. } if entity == a));
    let s = sketch(&h);
    assert!(s.entity(a).is_none(), "the trimmed line is replaced");
    assert!(s.entity(b).is_some());
    let ls = lines(&s);
    assert_eq!(ls.len(), 2);
    let piece = ls.iter().find(|l| l.0 != b).unwrap();
    assert!(
        near(piece.1, pt(0.0, 0.0)) && near(piece.2, pt(20.0, 0.0)),
        "{piece:?}"
    );
    // The new end is pinned on the cutter (SK-07).
    assert!(s.constraints().any(|(_, c)| matches!(
        c.kind,
        ConstraintKind::PointOnCurve { curve, .. } if curve == b
    )));
}

#[test]
fn extend_lengthens_the_clicked_end_to_the_next_curve() {
    let mut h = open();
    let (a, b) = load(&mut h, |s| {
        let a = s.add_line(pt(0.0, 0.0), pt(15.0, 0.0)).unwrap();
        let b = s.add_line(pt(20.0, -10.0), pt(20.0, 10.0)).unwrap();
        (a, b)
    });
    let before = start(&mut h, ModifyKind::Extend);
    let undo = applied(&h);
    click(&mut h, pt(13.0, 0.0));
    let op = check_replay(&h, ModifyKind::Extend, &before, undo);
    assert!(matches!(op, ModifyOp::Extend { entity, .. } if entity == a));
    let s = sketch(&h);
    let ext = lines(&s).into_iter().find(|l| l.0 != b).unwrap();
    assert!(
        near(ext.1, pt(0.0, 0.0)) && near(ext.2, pt(20.0, 0.0)),
        "{ext:?}"
    );
}

#[test]
fn break_splits_a_line_at_the_snapped_intersection() {
    let mut h = open();
    let (a, b) = load(&mut h, cross);
    let before = start(&mut h, ModifyKind::Break);
    let undo = applied(&h);
    click(&mut h, pt(20.3, 0.0));
    let op = check_replay(&h, ModifyKind::Break, &before, undo);
    match op {
        ModifyOp::Break { entity, at } => {
            assert_eq!(entity, a);
            assert!(
                near(at, pt(20.0, 0.0)),
                "snapped to the intersection: {at:?}"
            );
        }
        other => panic!("{other:?}"),
    }
    let s = sketch(&h);
    let pieces: Vec<_> = lines(&s).into_iter().filter(|l| l.0 != b).collect();
    assert_eq!(pieces.len(), 2);
    assert!(
        pieces
            .iter()
            .any(|l| near(l.2, pt(20.0, 0.0)) || near(l.1, pt(20.0, 0.0)))
    );
}

#[test]
fn fillet_rounds_a_corner_and_binds_the_radius_expression() {
    let mut h = open();
    load(&mut h, rectangle);
    let dof = session(&h).dof().unwrap().dof;
    let before = start(&mut h, ModifyKind::Fillet);
    type_into(&mut h, field::RADIUS, "5 mm");
    let undo = applied(&h);
    click(&mut h, pt(0.0, 0.0));
    let op = check_replay(&h, ModifyKind::Fillet, &before, undo);
    assert!(matches!(op, ModifyOp::Fillet { radius, .. } if (radius - 5.0).abs() < 1e-12));
    let s = sketch(&h);
    assert_eq!(count(&s, |k| matches!(k, EntityKind::Arc { .. })), 1);
    let ds = dims(&s);
    assert_eq!(ds.len(), 1);
    assert!(matches!(ds[0].kind, DimensionKind::Radius { .. }));
    assert!((ds[0].value - 5.0).abs() < 1e-9);
    let tangents = s
        .constraints()
        .filter(|(_, c)| matches!(c.kind, ConstraintKind::Tangent { .. }))
        .count();
    assert_eq!(tangents, 2);
    assert_eq!(
        session(&h).dof().unwrap().dof,
        dof,
        "a joined corner keeps its DOF"
    );
    // A second corner from two curves.
    click(&mut h, pt(39.0, 0.0));
    click(&mut h, pt(40.0, 1.0));
    assert_eq!(
        count(&sketch(&h), |k| matches!(k, EntityKind::Arc { .. })),
        2
    );
    assert_eq!(applied(&h), undo + 2);
}

#[test]
fn chamfer_with_two_distances_cuts_the_corner() {
    let mut h = open();
    load(&mut h, rectangle);
    let before = start(&mut h, ModifyKind::Chamfer);
    h.kittest().get_by_label("Two distances").click();
    h.kittest().step();
    type_into(&mut h, field::DISTANCE, "3 mm");
    type_into(&mut h, field::DISTANCE2, "4 mm");
    let undo = applied(&h);
    click(&mut h, pt(0.0, 0.0));
    let op = check_replay(&h, ModifyKind::Chamfer, &before, undo);
    assert!(matches!(
        op,
        ModifyOp::Chamfer { spec: amg_sketch::ChamferSpec::TwoDistances(a, b), .. }
            if (a - 3.0).abs() < 1e-12 && (b - 4.0).abs() < 1e-12
    ));
    let s = sketch(&h);
    let ds = dims(&s);
    assert_eq!(ds.len(), 2, "a length per leg");
    let mut vals: Vec<f64> = ds.iter().map(|d| d.value).collect();
    vals.sort_by(f64::total_cmp);
    assert!(
        (vals[0] - 3.0).abs() < 1e-9 && (vals[1] - 4.0).abs() < 1e-9,
        "{vals:?}"
    );
    // 4 sides + the chamfer line + 2 construction legs.
    assert_eq!(count(&s, |k| matches!(k, EntityKind::Line { .. })), 7);
}

#[test]
fn offset_copies_the_clicked_chain_to_the_clicked_side() {
    let mut h = open();
    load(&mut h, rectangle);
    let before = start(&mut h, ModifyKind::Offset);
    type_into(&mut h, field::OFFSET, "2 mm");
    let undo = applied(&h);
    click(&mut h, pt(20.0, 0.0));
    assert_eq!(applied(&h), undo, "the chain is picked first");
    click(&mut h, pt(20.0, 15.0));
    let op = check_replay(&h, ModifyKind::Offset, &before, undo);
    match &op {
        ModifyOp::Offset {
            chain,
            distance,
            constrain,
            ..
        } => {
            assert_eq!(chain.len(), 4, "the rectangle's chain");
            assert!((distance - 2.0).abs() < 1e-12);
            assert!(*constrain);
        }
        other => panic!("{other:?}"),
    }
    let s = sketch(&h);
    let inner: Vec<_> = lines(&s)
        .into_iter()
        .filter(|l| !s.entity(l.0).unwrap().construction)
        .filter(|l| {
            [l.1, l.2]
                .iter()
                .all(|p| p.x > 1.0 && p.x < 39.0 && p.y > 1.0 && p.y < 29.0)
        })
        .collect();
    assert_eq!(inner.len(), 4, "the inner copy");
    let ds = dims(&s);
    assert!(!ds.is_empty());
    assert!(ds.iter().all(|d| (d.value - 2.0).abs() < 1e-9));
}

#[test]
fn mirror_selects_objects_then_the_line_and_adds_symmetric_constraints() {
    let mut h = open();
    let (axis, c) = load(&mut h, |s| {
        let axis = s.add_line(pt(10.0, -10.0), pt(10.0, 30.0)).unwrap();
        s.set_construction(axis, true).unwrap();
        let c = s.add_circle(pt(0.0, 10.0), 4.0).unwrap();
        (axis, c)
    });
    let before = start(&mut h, ModifyKind::Mirror);
    let undo = applied(&h);
    click(&mut h, pt(-4.0, 10.0));
    assert_eq!(
        session(&h).selected.iter().copied().collect::<Vec<_>>(),
        vec![c]
    );
    key(&mut h, egui::Key::Enter);
    click(&mut h, pt(10.0, 25.0));
    let op = check_replay(&h, ModifyKind::Mirror, &before, undo);
    assert!(
        matches!(&op, ModifyOp::Mirror { items, axis: a, constrain: true } if items == &[c] && *a == axis)
    );
    let s = sketch(&h);
    let cs = circles(&s);
    assert_eq!(cs.len(), 2);
    assert!(
        cs.iter()
            .any(|(p, r)| near(*p, pt(20.0, 10.0)) && (r - 4.0).abs() < 1e-9)
    );
    assert!(
        s.constraints()
            .any(|(_, k)| matches!(k.kind, ConstraintKind::Symmetric { .. }))
    );
}

#[test]
fn rectangular_pattern_copies_in_columns_with_the_typed_spacing() {
    let mut h = open();
    let c = load(&mut h, |s| s.add_circle(pt(0.0, 0.0), 2.0).unwrap());
    let before = start(&mut h, ModifyKind::RectPattern);
    type_into(&mut h, field::COLUMN_SPACING, "10 mm");
    let undo = applied(&h);
    click(&mut h, pt(2.0, 0.0));
    key(&mut h, egui::Key::Enter);
    let op = check_replay(&h, ModifyKind::RectPattern, &before, undo);
    assert!(
        matches!(&op, ModifyOp::RectPattern { items, n_a: 2, dir_b: None, .. } if items == &[c])
    );
    let cs = circles(&sketch(&h));
    assert_eq!(cs.len(), 2);
    assert!(cs.iter().any(|(p, _)| near(*p, pt(10.0, 0.0))));
}

#[test]
fn circular_pattern_copies_around_the_clicked_centre() {
    let mut h = open();
    let (centre, c) = load(&mut h, |s| {
        let p = s.add_point(pt(0.0, 0.0)).unwrap();
        let c = s.add_circle(pt(20.0, 0.0), 3.0).unwrap();
        (p, c)
    });
    let before = start(&mut h, ModifyKind::CircPattern);
    let undo = applied(&h);
    click(&mut h, pt(23.0, 0.0));
    key(&mut h, egui::Key::Enter);
    click(&mut h, pt(0.0, 0.0));
    let op = check_replay(&h, ModifyKind::CircPattern, &before, undo);
    assert!(
        matches!(&op, ModifyOp::CircPattern { items, center, n: 3, .. } if items == &[c] && *center == centre)
    );
    let cs = circles(&sketch(&h));
    assert_eq!(cs.len(), 3);
    let r120 = pt(
        20.0 * (120f64).to_radians().cos(),
        20.0 * (120f64).to_radians().sin(),
    );
    assert!(cs.iter().any(|(p, _)| p.distance(r120) < 1e-6), "{cs:?}");
}

#[test]
fn move_translates_the_selection_from_base_point_to_target() {
    let mut h = open();
    let l = load(&mut h, |s| {
        let l = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
        s.add_point(pt(20.0, 10.0)).unwrap();
        l
    });
    let before = start(&mut h, ModifyKind::Move);
    let undo = applied(&h);
    click(&mut h, pt(5.0, 0.0));
    key(&mut h, egui::Key::Enter);
    click(&mut h, pt(0.2, 0.1));
    click(&mut h, pt(20.1, 9.9));
    let op = check_replay(&h, ModifyKind::Move, &before, undo);
    match &op {
        ModifyOp::Transform {
            items,
            transform,
            copy,
        } => {
            assert_eq!(items, &[l]);
            assert!(!copy);
            let d = transform.translation;
            assert!(
                (d.x - 20.0).abs() < 1e-12 && (d.y - 10.0).abs() < 1e-12,
                "snapped: {d:?}"
            );
        }
        other => panic!("{other:?}"),
    }
    let ls = lines(&sketch(&h));
    assert_eq!(ls.len(), 1);
    assert!(near(ls[0].1, pt(20.0, 10.0)) && near(ls[0].2, pt(30.0, 10.0)));
}

#[test]
fn copy_leaves_the_source_and_makes_an_independent_copy() {
    let mut h = open();
    load(&mut h, |s| {
        let l = s.add_line(pt(0.0, 0.0), pt(10.0, 0.0)).unwrap();
        s.add_constraint(ConstraintKind::Horizontal(amg_sketch::HvTarget::Line(l)))
            .unwrap();
        s.add_point(pt(0.0, 15.0)).unwrap();
    });
    let before = start(&mut h, ModifyKind::Copy);
    let undo = applied(&h);
    click(&mut h, pt(5.0, 0.0));
    key(&mut h, egui::Key::Enter);
    click(&mut h, pt(0.0, 0.0));
    click(&mut h, pt(0.0, 15.0));
    let op = check_replay(&h, ModifyKind::Copy, &before, undo);
    assert!(matches!(op, ModifyOp::Transform { copy: true, .. }));
    let s = sketch(&h);
    let ls = lines(&s);
    assert_eq!(ls.len(), 2);
    assert!(ls.iter().any(|l| near(l.1, pt(0.0, 15.0))));
    let hs = s
        .constraints()
        .filter(|(_, c)| matches!(c.kind, ConstraintKind::Horizontal(_)))
        .count();
    assert_eq!(hs, 2, "the internal constraint is copied");
}

#[test]
fn rotate_turns_the_selection_about_the_clicked_centre() {
    let mut h = open();
    let l = load(&mut h, |s| {
        s.add_point(pt(0.0, 0.0)).unwrap();
        s.add_line(pt(10.0, 0.0), pt(20.0, 0.0)).unwrap()
    });
    let before = start(&mut h, ModifyKind::Rotate);
    let undo = applied(&h);
    click(&mut h, pt(15.0, 0.0));
    key(&mut h, egui::Key::Enter);
    click(&mut h, pt(0.1, 0.1));
    let op = check_replay(&h, ModifyKind::Rotate, &before, undo);
    assert!(matches!(&op, ModifyOp::Transform { items, copy: false, .. } if items == &[l]));
    let ls = lines(&sketch(&h));
    assert!(
        near(ls[0].1, pt(0.0, 10.0)) && near(ls[0].2, pt(0.0, 20.0)),
        "{ls:?}"
    );
}

#[test]
fn scale_about_a_point_scales_the_dimension_values() {
    let mut h = open();
    let c = load(&mut h, |s| {
        s.add_point(pt(0.0, 0.0)).unwrap();
        let c = s.add_circle(pt(10.0, 0.0), 2.0).unwrap();
        s.add_constraint(ConstraintKind::Dimension(Dimension {
            kind: DimensionKind::Diameter { curve: c },
            value: 4.0,
            driving: true,
            label_pos: pt(10.0, 5.0),
        }))
        .unwrap();
        c
    });
    let before = start(&mut h, ModifyKind::Scale);
    let undo = applied(&h);
    click(&mut h, pt(12.0, 0.0));
    key(&mut h, egui::Key::Enter);
    click(&mut h, pt(0.0, 0.0));
    let op = check_replay(&h, ModifyKind::Scale, &before, undo);
    assert!(matches!(&op, ModifyOp::Transform { items, copy: false, .. } if items == &[c]));
    let s = sketch(&h);
    let cs = circles(&s);
    assert!(
        near(cs[0].0, pt(20.0, 0.0)) && (cs[0].1 - 4.0).abs() < 1e-9,
        "{cs:?}"
    );
    assert!(
        (dims(&s)[0].value - 8.0).abs() < 1e-9,
        "the diameter doubles"
    );
}

// ---- reachability, refusals, Esc -------------------------------------------------------------

#[test]
fn every_modify_tool_runs_and_fusion_shortcuts_t_o_m() {
    let mut h = open();
    let map = h.app().shortcuts();
    assert!(map.conflicts().is_empty(), "{:?}", map.conflicts());
    for (k, s) in [
        (ModifyKind::Trim, "T"),
        (ModifyKind::Offset, "O"),
        (ModifyKind::Move, "M"),
    ] {
        assert_eq!(map.display_of(k.def().id).as_deref(), Some(s));
    }
    for k in ModifyKind::ALL {
        h.run_command(k.def().id);
        h.drive_until_idle();
        assert_eq!(h.app().active_tool(), Some(k.def().id), "{k:?}");
    }
    for (k, key) in [
        (ModifyKind::Trim, egui::Key::T),
        (ModifyKind::Offset, egui::Key::O),
        (ModifyKind::Move, egui::Key::M),
    ] {
        h.key(egui::Key::Escape);
        h.drive_until_idle();
        h.key(key);
        h.drive_until_idle();
        assert_eq!(h.app().active_tool(), Some(k.def().id), "{k:?}");
    }
}

#[test]
fn refused_picks_change_nothing_and_esc_steps_back() {
    let mut h = open();
    load(&mut h, |s| {
        s.add_circle(pt(0.0, 0.0), 3.0).unwrap();
        s.add_line(pt(10.0, 0.0), pt(20.0, 0.0)).unwrap();
    });
    let json = serde_json::to_string(&sketch(&h)).unwrap();
    let undo = applied(&h);
    // Extend refuses a circle; break refuses a circle.
    for k in [ModifyKind::Extend, ModifyKind::Break] {
        start(&mut h, k);
        click(&mut h, pt(3.0, 0.0));
        assert_eq!(applied(&h), undo, "{k:?}");
    }
    // A fillet on a free line end: one curve only.
    start(&mut h, ModifyKind::Fillet);
    click(&mut h, pt(20.0, 0.0));
    assert_eq!(applied(&h), undo);
    // Mirror: objects, Enter, then Esc twice leaves the objects, then the tool.
    start(&mut h, ModifyKind::Mirror);
    click(&mut h, pt(15.0, 0.0));
    key(&mut h, egui::Key::Enter);
    key(&mut h, egui::Key::Escape);
    assert_eq!(h.app().active_tool(), Some(ModifyKind::Mirror.def().id));
    key(&mut h, egui::Key::Escape);
    assert_eq!(h.app().active_tool(), Some(ModifyKind::Mirror.def().id));
    assert!(session(&h).selected.is_empty());
    key(&mut h, egui::Key::Escape);
    assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD));
    assert_eq!(serde_json::to_string(&sketch(&h)).unwrap(), json);
    assert_eq!(applied(&h), undo);
}
