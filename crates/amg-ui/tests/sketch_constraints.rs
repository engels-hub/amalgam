//! UI-14 constraint and dimension UX, kittest on the real app with the `NullViewport` and
//! `CpuPickService`.
//!
//! - Done-when 1: dimensioning a side of a fully constrained rectangle offers "driven" and
//!   creates a driven dimension on Yes.
//! - Done-when 2: double-click a dimension, type `w/2` (user parameter `w`): the expression is
//!   set, the sketch re-solves and the DOF is unchanged.
//! - Done-when 3: each constraint tool applies to a valid pick set and refuses invalid kinds
//!   with a status hint.
//! - Done-when 4: hovering a glyph highlights exactly the constrained entities.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::collections::BTreeSet;
use std::time::{Duration, Instant};

use amg_core::{Point2, Point3, Unit};
use amg_doc::{Command, Expr, FeatureBox};
use amg_render::{Camera, Projection};
use amg_sketch::{ConstraintId, ConstraintKind, DimensionKind, End, EntityId, EntityKind, Sketch};
use amg_ui::CommitBatch;
use amg_ui::sketch::dimension::{DRIVEN_TITLE, EDIT_FIELD_NAME};
use amg_ui::sketch::glyphs;
use amg_ui::sketch::{
    CREATE_SKETCH_CMD, ConstraintTool, DIMENSION_DEF, DimensionEditDialog, DrivenPromptDialog,
    LINE_CMD, SKETCH_SELECT_CMD, SketchLayer, SketchSession,
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

fn dof(h: &AppHarness) -> u32 {
    session(h).dof().expect("solved").dof
}

/// A new XY sketch seen from the top.
fn open() -> AppHarness {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h.run_command(CREATE_SKETCH_CMD);
    h.kittest().get_by_label("XY").click();
    h.drive_until_idle();
    h.app_mut().active_document_mut().camera = Camera {
        eye: Point3::new(20.0, 15.0, 150.0),
        target: Point3::new(20.0, 15.0, 0.0),
        up: amg_core::Dir3::Y,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    h.drive_until_idle();
    h
}

fn screen(h: &AppHarness, p: Point2) -> egui::Pos2 {
    h.world_to_screen(Point3::new(p.x, p.y, 0.0))
}

fn hover(h: &mut AppHarness, p: Point2) {
    let s = screen(h, p);
    h.hover_at(s);
    h.drive_until_idle();
}

fn click(h: &mut AppHarness, p: Point2) {
    let s = screen(h, p);
    h.click_at(s, PRIM, NONE);
    h.drive_until_idle();
}

/// Steps until the session has no solver work left (a focused text field keeps asking for
/// frames, so `drive_until_idle` cannot be used while the dimension editor is open).
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

/// A click whose consequences may open the dimension editor.
fn click_settle(h: &mut AppHarness, p: Point2) {
    let s = screen(h, p);
    h.click_at(s, PRIM, NONE);
    settle(h);
}

fn status_shows(h: &mut AppHarness, text: &str) -> bool {
    h.kittest()
        .query_all_by_label_contains(text)
        .next()
        .is_some()
}

fn dimensions(s: &Sketch) -> Vec<(ConstraintId, amg_sketch::Dimension)> {
    s.constraints()
        .filter_map(|(id, c)| c.kind.as_dimension().map(|d| (id, d.clone())))
        .collect()
}

/// The rectangle (0,0) → (40,0) → (40,30) → (0,30) drawn with the Line tool (DOF 4).
fn draw_rectangle(h: &mut AppHarness) {
    h.run_command(LINE_CMD);
    for p in [
        pt(0.0, 0.0),
        pt(40.0, 0.0),
        pt(40.0, 30.0),
        pt(0.0, 30.0),
        pt(0.0, 0.0),
    ] {
        click(h, p);
    }
    to_select(h);
    assert_eq!(dof(h), 4);
}

/// Esc until the session's select tool runs (one more Esc would leave the sketch).
fn to_select(h: &mut AppHarness) {
    for _ in 0..3 {
        if h.app().active_tool() == Some(SKETCH_SELECT_CMD) {
            return;
        }
        h.key(egui::Key::Escape);
        h.drive_until_idle();
    }
    assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD));
}

/// Closes the dimension editor without a change (Enter on the unchanged value).
fn close_editor(h: &mut AppHarness) {
    assert!(
        h.app().dialog::<DimensionEditDialog>().is_some(),
        "the editor opens on a new dimension"
    );
    h.key(egui::Key::Enter);
    h.drive_until_idle();
    assert!(h.app().dialog::<DimensionEditDialog>().is_none());
}

/// Sketch Dimension: pick `on`, place the label at `at`, close the editor.
fn dimension(h: &mut AppHarness, on: Point2, at: Point2) {
    if h.app().active_tool() != Some(DIMENSION_DEF.id) {
        h.run_command(DIMENSION_DEF.id);
    }
    click(h, on);
    click_settle(h, at);
}

/// The rectangle with its (0,0) corner fixed, a 40 mm width and a 30 mm height: DOF 0.
fn constrained_rectangle(h: &mut AppHarness) {
    draw_rectangle(h);
    h.run_command(ConstraintTool::Fix.def().id);
    click(h, pt(0.0, 0.0));
    assert_eq!(dof(h), 2, "fixed corner");
    dimension(h, pt(20.0, 0.0), pt(20.0, -8.0));
    close_editor(h);
    dimension(h, pt(40.0, 15.0), pt(48.0, 15.0));
    close_editor(h);
    h.key(egui::Key::Escape);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD));
    assert_eq!(dof(h), 0, "fully constrained");
    let dims = dimensions(&sketch(h));
    assert_eq!(dims.len(), 2);
    assert!(dims.iter().all(|(_, d)| d.driving));
    assert!(matches!(dims[0].1.kind, DimensionKind::Length { .. }));
    assert!((dims[0].1.value - 40.0).abs() < 1e-3);
    assert!((dims[1].1.value - 30.0).abs() < 1e-3, "{dims:?}");
}

// ---- Done-when 1 ---------------------------------------------------------------------------

#[test]
fn dimensioning_a_fully_constrained_side_offers_driven_and_yes_creates_it() {
    let mut h = open();
    constrained_rectangle(&mut h);
    let before = applied(&h);
    // The top side: its length follows from the width already.
    dimension(&mut h, pt(20.0, 30.0), pt(20.0, 38.0));
    assert!(
        h.app().dialog::<DrivenPromptDialog>().is_some(),
        "prompt open"
    );
    assert!(h.app().dialog::<DimensionEditDialog>().is_none());
    assert_eq!(
        h.app()
            .dialog_titles()
            .iter()
            .filter(|t| *t == DRIVEN_TITLE)
            .count(),
        1
    );
    let offer = session(&h).driven_offer.clone().expect("an offer");
    assert!(!offer.conflicting, "redundant, not conflicting");
    assert!(!offer.with.is_empty());
    assert_eq!(applied(&h), before, "nothing added yet");
    assert_eq!(dimensions(&sketch(&h)).len(), 2);
    assert!(status_shows(&mut h, "over-constrain"));

    h.kittest().get_by_label("Yes").click();
    h.drive_until_idle();
    assert!(h.app().dialog::<DrivenPromptDialog>().is_none());
    assert_eq!(applied(&h), before + 1, "one undo entry");
    assert_eq!(h.document().undo_label(), Some(DIMENSION_DEF.label));
    let dims = dimensions(&sketch(&h));
    assert_eq!(dims.len(), 3);
    let (id, d) = &dims[2];
    assert!(!d.driving, "driven");
    assert!(matches!(d.kind, DimensionKind::Length { .. }));
    assert!(
        !session(&h).working().dims.contains_key(id),
        "no expression"
    );
    // The driven value is measured by the solve.
    let shown = session(&h).display().constraint(*id).unwrap();
    assert!((shown.kind.as_dimension().unwrap().value - 40.0).abs() < 1e-6);
    assert_eq!(dof(&h), 0, "DOF unchanged");

    // No drops a second offer.
    dimension(&mut h, pt(0.0, 15.0), pt(-8.0, 15.0));
    assert!(h.app().dialog::<DrivenPromptDialog>().is_some());
    h.kittest().get_by_label("No").click();
    h.drive_until_idle();
    assert!(h.app().dialog::<DrivenPromptDialog>().is_none());
    assert_eq!(dimensions(&sketch(&h)).len(), 3);
    assert_eq!(applied(&h), before + 1);
}

// ---- Done-when 2 ---------------------------------------------------------------------------

/// Two clicks in one frame: kittest runs one 0.25 s frame per queued event, so the events go
/// straight into the next frame's raw input to stay inside the 0.3 s double-click window.
fn double_click(h: &mut AppHarness, p: Point2) {
    let pos = screen(h, p);
    h.hover_at(pos);
    for pressed in [true, false, true, false] {
        h.kittest()
            .input_mut()
            .events
            .push(egui::Event::PointerButton {
                pos,
                button: PRIM,
                pressed,
                modifiers: NONE,
            });
    }
    h.kittest().step();
    settle(h);
}

#[test]
fn double_click_a_dimension_and_type_w_over_2() {
    let mut h = open();
    constrained_rectangle(&mut h);
    h.app_mut().submit(CommitBatch {
        label: "Add w".into(),
        commands: vec![Command::AddParam {
            name: "w".into(),
            expr: "100 mm".into(),
            unit: Some(Unit::Millimetre),
            comment: String::new(),
        }],
    });
    h.drive_until_idle();
    let (width, _) = dimensions(&sketch(&h))[0].clone();
    let Some(Expr::Param(param)) = session(&h).working().dims.get(&width).cloned() else {
        panic!("the dimension is a model parameter");
    };
    let before = applied(&h);

    double_click(&mut h, pt(20.0, -8.0));
    let editor = h
        .app()
        .dialog::<DimensionEditDialog>()
        .expect("double click opens the editor");
    assert_eq!(editor.id, width);
    h.kittest()
        .key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
    h.kittest().step();
    h.kittest().get_by_label(EDIT_FIELD_NAME).type_text("w/2");
    h.kittest().step();
    h.key(egui::Key::Enter);
    h.drive_until_idle();
    assert!(
        h.app().dialog::<DimensionEditDialog>().is_none(),
        "Enter commits"
    );

    assert_eq!(applied(&h), before + 1, "one undo entry");
    let p = h.document().design().params().get(param).unwrap();
    assert_eq!(
        p.expr.0, "w/2",
        "the model parameter carries the expression"
    );
    let s = session(&h);
    let d = s
        .display()
        .constraint(width)
        .unwrap()
        .kind
        .as_dimension()
        .unwrap()
        .clone();
    assert!((d.value - 50.0).abs() < 1e-9, "{}", d.value);
    // Re-solved: the far corners moved to x = 50.
    let xs: Vec<f64> = s
        .display()
        .entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Point { .. }))
        .filter_map(|(id, _)| s.display().point(id))
        .map(|p| p.x)
        .collect();
    assert!(xs.iter().any(|x| (x - 50.0).abs() < 1e-4), "{xs:?}");
    assert!(!xs.iter().any(|x| (x - 40.0).abs() < 1e-3), "{xs:?}");
    assert_eq!(dof(&h), 0, "DOF unchanged");

    // Changing `w` drives the sketch.
    let w = h.document().design().params().by_name("w").unwrap();
    h.app_mut().submit(CommitBatch {
        label: "Set w".into(),
        commands: vec![Command::SetParamExpr {
            id: w,
            expr: "60 mm".into(),
        }],
    });
    h.drive_until_idle();
    let d = session(&h)
        .display()
        .constraint(width)
        .unwrap()
        .kind
        .as_dimension()
        .unwrap()
        .clone();
    assert!((d.value - 30.0).abs() < 1e-9);
    assert_eq!(dof(&h), 0);

    // An invalid expression stays in the editor with its message.
    double_click(&mut h, pt(15.0, -8.0));
    assert!(h.app().dialog::<DimensionEditDialog>().is_some());
    h.kittest()
        .key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
    h.kittest().step();
    h.kittest()
        .get_by_label(EDIT_FIELD_NAME)
        .type_text("nope*2");
    h.kittest().step();
    h.key(egui::Key::Enter);
    settle(&mut h);
    let ed = h.app().dialog::<DimensionEditDialog>().expect("still open");
    assert!(ed.error.is_some(), "the error is shown");
    assert_eq!(applied(&h), before + 2, "nothing submitted");
}

// ---- Done-when 3 ---------------------------------------------------------------------------

struct Fixture {
    l1: EntityId,
    l2: EntityId,
    l3: EntityId,
    c1: EntityId,
    c2: EntityId,
    p: EntityId,
    l4: EntityId,
    spline: EntityId,
}

/// Loads a sketch with lines, circles, a free point and a spline joined to a line's end.
fn load_fixture(h: &mut AppHarness) -> Fixture {
    let mut f = session(h).working().clone();
    let s = &mut f.sketch;
    let l1 = s.add_line(pt(0.0, 0.0), pt(30.0, 2.0)).unwrap();
    let l2 = s.add_line(pt(0.0, 10.0), pt(30.0, 14.0)).unwrap();
    let l3 = s.add_line(pt(50.0, 0.0), pt(52.0, 30.0)).unwrap();
    let c1 = s.add_circle(pt(-30.0, 10.0), 5.0).unwrap();
    let c2 = s.add_circle(pt(-30.0, 40.0), 8.0).unwrap();
    let p = s.add_point(pt(15.0, 25.0)).unwrap();
    let l4 = s.add_line(pt(60.0, 40.0), pt(70.0, 40.0)).unwrap();
    let spline = s
        .add_spline(
            3,
            &[
                pt(70.0, 40.0),
                pt(75.0, 41.0),
                pt(80.0, 45.0),
                pt(85.0, 50.0),
            ],
            None,
            None,
        )
        .unwrap();
    let a = s.entity(l4).unwrap().kind.endpoint(End::End).unwrap();
    let b = s.entity(spline).unwrap().kind.endpoint(End::Start).unwrap();
    s.add_constraint(ConstraintKind::Coincident { a, b })
        .unwrap();
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
    assert_eq!(sketch(h).entities().count(), n, "fixture loaded");
    Fixture {
        l1,
        l2,
        l3,
        c1,
        c2,
        p,
        l4,
        spline,
    }
}

/// A point on entity `e` that picks it (a line's middle, a circle's rim, the point).
fn on(s: &Sketch, e: EntityId) -> Point2 {
    if let Some(p) = s.point(e) {
        return p;
    }
    match s.entity(e).unwrap().kind {
        EntityKind::Circle { center, radius } => {
            s.point(center).unwrap() + amg_core::Vec2::new(s.params().get(radius), 0.0)
        }
        _ => {
            let c = s.curve(e).unwrap();
            let (t0, t1) = c.param_range();
            c.eval(t0 + 0.4 * (t1 - t0))
        }
    }
}

/// Tool, valid picks, the constraint it adds, an invalid pick set, words of the hint.
type Case = (
    ConstraintTool,
    Vec<EntityId>,
    &'static str,
    Vec<EntityId>,
    &'static str,
);

#[test]
fn each_constraint_tool_applies_to_valid_picks_and_refuses_invalid_kinds() {
    let mut h = open();
    let f = load_fixture(&mut h);
    let s0 = sketch(&h);
    // (tool, valid picks, the constraint it adds, an invalid pick set, words of the hint)
    let cases: Vec<Case> = vec![
        (
            ConstraintTool::HorizontalVertical,
            vec![f.l1],
            "horizontal",
            vec![f.c1],
            "needs a line or two points",
        ),
        (
            ConstraintTool::Coincident,
            vec![f.p, f.l2],
            "point on curve",
            vec![f.l1, f.l2],
            "needs a point",
        ),
        (
            ConstraintTool::Tangent,
            vec![f.l2, f.c1],
            "tangent",
            vec![f.p],
            "needs a curve",
        ),
        (
            ConstraintTool::Equal,
            vec![f.c1, f.c2],
            "equal",
            vec![f.l1, f.c1],
            "same kind",
        ),
        (
            ConstraintTool::Parallel,
            vec![f.l1, f.l2],
            "parallel",
            vec![f.c1],
            "needs a line",
        ),
        (
            ConstraintTool::Perpendicular,
            vec![f.l1, f.l3],
            "perpendicular",
            vec![f.l1, f.c2],
            "needs a line",
        ),
        (ConstraintTool::Fix, vec![f.p], "fix", vec![], ""),
        (
            ConstraintTool::Midpoint,
            vec![f.p, f.l1],
            "midpoint",
            vec![f.c1],
            "needs a point or a line",
        ),
        (
            ConstraintTool::Concentric,
            vec![f.c1, f.c2],
            "concentric",
            vec![f.l3],
            "needs a circle, arc or ellipse",
        ),
        (
            ConstraintTool::Collinear,
            vec![f.l1, f.l2],
            "collinear",
            vec![f.p],
            "needs a line",
        ),
        (
            ConstraintTool::Symmetric,
            vec![f.c1, f.c2, f.l3],
            "symmetric",
            vec![f.c1, f.l1],
            "to mirror",
        ),
        (
            ConstraintTool::Curvature,
            vec![f.spline, f.l4],
            "curvature",
            vec![f.l1, f.l2],
            "spline",
        ),
    ];
    let mut tested = BTreeSet::new();
    for (tool, valid, name, invalid, hint) in cases {
        tested.insert(tool.def().id);
        let id = tool.def().id;
        h.run_command(id);
        assert_eq!(h.app().active_tool(), Some(id), "{tool:?}");
        // Invalid kinds: refused with a hint, nothing added.
        if !invalid.is_empty() {
            let before = applied(&h);
            for e in &invalid {
                click(&mut h, on(&s0, *e));
            }
            assert_eq!(applied(&h), before, "{tool:?}: refused");
            assert!(
                status_shows(&mut h, hint),
                "{tool:?}: hint '{hint}' in the status bar"
            );
            h.key(egui::Key::Escape);
            h.drive_until_idle();
            if h.app().active_tool() != Some(id) {
                h.run_command(id);
            }
            assert!(!status_shows(&mut h, hint), "{tool:?}: Esc clears the hint");
        }
        if tool == ConstraintTool::Curvature {
            // A spline and a line that do not share an end point are refused too.
            let before = applied(&h);
            click(&mut h, on(&s0, f.spline));
            click(&mut h, on(&s0, f.l3));
            assert_eq!(applied(&h), before, "{tool:?}");
            assert!(status_shows(&mut h, "share an end point"), "{tool:?}");
            h.key(egui::Key::Escape);
            h.drive_until_idle();
        }
        let before = applied(&h);
        let count = sketch(&h).constraints().count();
        for e in &valid {
            click(&mut h, on(&s0, *e));
        }
        assert_eq!(
            applied(&h),
            before + 1,
            "{tool:?}: one undo entry ({:?}, {:?}, {:?})",
            session(&h).notice,
            valid
                .iter()
                .map(|e| amg_sketch::query::pick(
                    &s0,
                    on(&s0, *e),
                    1.0,
                    amg_sketch::query::PickFilter::ALL
                )
                .into_iter()
                .map(|x| x.entity)
                .collect::<Vec<_>>())
                .collect::<Vec<_>>(),
            session(&h).selected
        );
        assert_eq!(
            h.document().undo_label(),
            Some(tool.def().label),
            "{tool:?}"
        );
        let s = sketch(&h);
        assert_eq!(s.constraints().count(), count + 1, "{tool:?}");
        let (_, last) = s.constraints().last().unwrap();
        assert_eq!(last.kind.name(), name, "{tool:?}");
        let mut want: Vec<EntityId> = valid.clone();
        want.sort();
        let mut got: Vec<EntityId> = last.kind.entities().to_vec();
        got.sort();
        assert_eq!(got, want, "{tool:?}: the picked entities");
        assert_eq!(h.app().active_tool(), Some(id), "{tool:?}: stays active");
        // Back to the fixture for the next tool.
        h.key(egui::Key::Escape);
        h.drive_until_idle();
        h.run_command(amg_ui::command::CmdId("edit.undo"));
        h.drive_until_idle();
        assert_eq!(sketch(&h).constraints().count(), count, "{tool:?}: undone");
    }
    assert_eq!(tested.len(), 12, "all twelve tools");
}

#[test]
fn fix_toggles_and_a_redundant_constraint_is_refused() {
    let mut h = open();
    let f = load_fixture(&mut h);
    let s0 = sketch(&h);
    let fix = ConstraintTool::Fix.def().id;
    h.run_command(fix);
    click(&mut h, on(&s0, f.p));
    let fixed = |h: &AppHarness| {
        sketch(h)
            .constraints()
            .any(|(_, c)| matches!(c.kind, ConstraintKind::Fix { entity } if entity == f.p))
    };
    assert!(fixed(&h));
    click(&mut h, on(&s0, f.p));
    assert!(!fixed(&h), "a second pick unfixes");
    assert_eq!(h.document().undo_label(), Some("Fix/Unfix"));

    // Parallel twice: the second one is redundant and not added.
    let par = ConstraintTool::Parallel.def().id;
    h.run_command(par);
    click(&mut h, on(&s0, f.l1));
    click(&mut h, on(&s0, f.l2));
    let before = applied(&h);
    let n = sketch(&h).constraints().count();
    click(&mut h, on(&s0, f.l2));
    click(&mut h, on(&s0, f.l1));
    assert_eq!(applied(&h), before);
    assert_eq!(sketch(&h).constraints().count(), n);
    assert!(status_shows(&mut h, "already implied"));

    // A pre-selection applies when the tool starts: Horizontal/Vertical on two lines.
    to_select(&mut h);
    assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD));
    click(&mut h, on(&s0, f.l3));
    let shift = egui::Modifiers::SHIFT;
    let p = screen(&h, on(&s0, f.c1));
    h.click_at(p, PRIM, shift);
    h.drive_until_idle();
    assert_eq!(session(&h).selected.len(), 2);
    h.run_command(ConstraintTool::HorizontalVertical.def().id);
    h.drive_until_idle();
    assert!(
        session(&h).selected.is_empty(),
        "an invalid pre-selection is dropped"
    );
}

// ---- Done-when 4 ---------------------------------------------------------------------------

/// The glyphs as the session lays them out.
fn layout(h: &AppHarness) -> Vec<glyphs::Glyph> {
    let s = session(h);
    let reveal: BTreeSet<EntityId> = s.selected.iter().chain(&s.hovered).copied().collect();
    glyphs::layout(s.display(), &reveal)
}

#[test]
fn glyph_hover_highlights_exactly_the_constrained_entities_and_delete_removes_it() {
    let mut h = open();
    let f = load_fixture(&mut h);
    let s0 = sketch(&h);
    h.run_command(ConstraintTool::Parallel.def().id);
    click(&mut h, on(&s0, f.l1));
    click(&mut h, on(&s0, f.l2));
    h.run_command(ConstraintTool::Concentric.def().id);
    click(&mut h, on(&s0, f.c1));
    click(&mut h, on(&s0, f.c2));
    to_select(&mut h);
    assert_eq!(h.app().active_tool(), Some(SKETCH_SELECT_CMD));

    let s = sketch(&h);
    let find = |name: &str| {
        s.constraints()
            .find(|(_, c)| c.kind.name() == name)
            .map(|(id, c)| {
                (
                    id,
                    c.kind.entities().iter().copied().collect::<BTreeSet<_>>(),
                )
            })
            .unwrap()
    };
    let view = h.camera_view();
    let plane = *session(&h).plane();
    for name in ["parallel", "concentric"] {
        let (id, ents) = find(name);
        let gs = layout(&h);
        let mine: Vec<_> = gs.iter().filter(|g| g.constraint == id).collect();
        assert_eq!(mine.len(), 2, "{name}: one glyph per entity");
        for g in mine {
            let at = g.screen(&view, &plane).unwrap();
            h.hover_at(at);
            h.drive_until_idle();
            let ses = session(&h);
            assert_eq!(ses.hovered_constraint, Some(id), "{name}");
            assert_eq!(ses.highlighted(), ents, "{name}: exactly its entities");
            assert_eq!(ses.hovered, None, "the glyph wins over the geometry below");
        }
    }
    // Away from every glyph nothing is highlighted.
    hover(&mut h, pt(40.0, -20.0));
    assert!(session(&h).highlighted().is_empty());

    // Click a glyph, Delete: the constraint goes (one undo entry), the geometry stays.
    let (id, _) = find("parallel");
    let g = layout(&h).into_iter().find(|g| g.constraint == id).unwrap();
    let at = g.screen(&view, &plane).unwrap();
    h.click_at(at, PRIM, NONE);
    h.drive_until_idle();
    assert_eq!(
        session(&h).selected_constraints,
        BTreeSet::from([id]),
        "selected"
    );
    let before = applied(&h);
    let ents = sketch(&h).entities().count();
    h.key(egui::Key::Delete);
    h.drive_until_idle();
    assert_eq!(applied(&h), before + 1);
    assert_eq!(h.document().undo_label(), Some("Delete constraint"));
    assert!(sketch(&h).constraint(id).is_none());
    assert_eq!(sketch(&h).entities().count(), ents);
}

#[test]
fn dragging_a_dimension_label_is_one_undo_entry() {
    let mut h = open();
    draw_rectangle(&mut h);
    dimension(&mut h, pt(20.0, 0.0), pt(20.0, -8.0));
    close_editor(&mut h);
    h.key(egui::Key::Escape);
    h.drive_until_idle();
    let (id, d) = dimensions(&sketch(&h))[0].clone();
    assert!((d.label_pos - pt(20.0, -8.0)).length() < 1e-3);
    let before = applied(&h);
    let (from, to) = (screen(&h, pt(20.0, -8.0)), screen(&h, pt(25.0, -15.0)));
    h.drag(PRIM, NONE, from, to, 20);
    h.drive_until_idle();
    assert_eq!(applied(&h), before + 1);
    assert_eq!(h.document().undo_label(), Some("Move dimension label"));
    let d = sketch(&h)
        .constraint(id)
        .unwrap()
        .kind
        .as_dimension()
        .unwrap()
        .clone();
    assert!(
        (d.label_pos - pt(25.0, -15.0)).length() < 1e-3,
        "{:?}",
        d.label_pos
    );
    assert!((d.value - 40.0).abs() < 1e-9, "the value is unchanged");
}

#[test]
fn sketch_dimension_has_fusion_shortcut_d_and_picks_radius_and_angle() {
    let mut h = open();
    let f = load_fixture(&mut h);
    let s0 = sketch(&h);
    hover(&mut h, pt(100.0, 100.0));
    h.key(egui::Key::D);
    h.drive_until_idle();
    assert_eq!(
        h.app().active_tool(),
        Some(DIMENSION_DEF.id),
        "D starts Sketch Dimension"
    );
    // A circle gets its diameter.
    click(&mut h, on(&s0, f.c1));
    click_settle(&mut h, pt(-45.0, 0.0));
    h.key(egui::Key::Enter);
    h.drive_until_idle();
    // Two lines that meet get an angle.
    click(&mut h, on(&s0, f.l1));
    click(&mut h, on(&s0, f.l3));
    click_settle(&mut h, pt(45.0, 3.0));
    h.key(egui::Key::Enter);
    h.drive_until_idle();
    let dims = dimensions(&sketch(&h));
    assert!(matches!(dims[0].1.kind, DimensionKind::Diameter { .. }));
    assert!((dims[0].1.value - 10.0).abs() < 1e-6);
    assert!(matches!(dims[1].1.kind, DimensionKind::Angle { .. }));
    let v = dims[1].1.value.to_degrees();
    assert!(v > 0.0 && v < 180.0, "{v}");
    assert_eq!(dof(&h), session(&h).dof().unwrap().dof);
}
