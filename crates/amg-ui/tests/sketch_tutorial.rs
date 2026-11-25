//! UI-16 Done-when 2: the scripted "Fusion sketch basics" scenario.
//!
//! A bracket plate drawn only through UI actions (clicks, keys, typed text) on the real app
//! (kittest, `NullViewport`, `CpuPickService`):
//!
//! 1. Change Parameters: user parameters `width = 80 mm`, `height = 50 mm`, `fillet = 8 mm`.
//! 2. Create Sketch on XY; 2-point rectangle (0,0)–(80,50).
//! 3. Fillet the four corners with radius `fillet` (four radius dimensions bound to it).
//! 4. Sketch Dimension: the straight bottom side `width - 2*fillet`, the right side
//!    `height - 2*fillet`.
//! 5. A vertical construction centreline between the midpoints of the bottom and top sides.
//! 6. Two holes Ø6 concentric with the left fillets (centres snapped to the fillet centres).
//! 7. Mirror the two holes about the centreline (symmetric constraints): two concentric hole
//!    pairs.
//! 8. A centre point slot at the centreline's midpoint (half length 10, width 8), made
//!    horizontal.
//! 9. Fix the bottom-left hole centre: DOF 0, every entity `Full`.
//!
//! The profile areas are checked against the closed-form values within 1e-9 relative, then
//! `width` is changed to 100 mm in the parameters dialog: still DOF 0, areas follow. Every UI
//! action goes through [`Script`], which counts them (at most 60).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::f64::consts::PI;
use std::time::{Duration, Instant};

use amg_core::{Point2, Point3};
use amg_sketch::solve::ConstraintState;
use amg_sketch::{ConstraintKind, DimensionKind, EntityKind, Sketch};
use amg_ui::params::{DIALOG_TITLE, PARAMETERS_CMD};
use amg_ui::sketch::modify::field;
use amg_ui::sketch::{
    CREATE_SKETCH_CMD, ConstraintTool, CreateKind, LINE_CMD, ModifyKind, SketchLayer, SketchSession,
};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable as _;

/// The action budget of the row.
const MAX_ACTIONS: u32 = 60;

fn pt(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

/// The app plus a count of the UI actions sent to it.
struct Script {
    h: AppHarness,
    actions: u32,
    log: Vec<String>,
}

impl Script {
    fn new() -> Script {
        let mut h = AppHarness::new();
        h.drive_until_idle();
        Script {
            h,
            actions: 0,
            log: Vec::new(),
        }
    }

    fn count(&mut self, what: String) {
        self.actions += 1;
        self.log.push(what);
    }

    fn session(&self) -> &SketchSession {
        self.h
            .app()
            .doc_layer::<SketchLayer>()
            .expect("sketch layer")
            .session()
            .expect("an open sketch session")
    }

    fn in_sketch(&self) -> bool {
        self.h
            .app()
            .doc_layer::<SketchLayer>()
            .is_some_and(|l| l.session().is_some())
    }

    fn sketch(&self) -> Sketch {
        self.session().working().sketch.clone()
    }

    /// Steps until no solver or document work is left (a focused text field keeps asking for
    /// frames, so `drive_until_idle` is not enough).
    fn settle(&mut self) {
        let start = Instant::now();
        loop {
            self.h.kittest().step();
            let solving = self.in_sketch() && self.session().is_solving();
            if !solving && self.h.app().jobs().jobs().is_empty() && self.h.document().is_evaluated()
            {
                for _ in 0..3 {
                    self.h.kittest().step();
                }
                return;
            }
            assert!(
                start.elapsed() < Duration::from_secs(30),
                "settle: still busy"
            );
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// Runs a command (a toolbar click, an S-key pick or a shortcut).
    fn command(&mut self, id: amg_ui::command::CmdId) {
        self.count(format!("command {}", id.0));
        self.h.run_command(id);
        self.settle();
    }

    /// Presses a key.
    fn key(&mut self, k: egui::Key) {
        self.count(format!("key {k:?}"));
        self.h.key(k);
        self.settle();
    }

    /// Clicks the sketch point `p` in the viewport.
    fn click(&mut self, p: Point2) {
        self.count(format!("click ({}, {})", p.x, p.y));
        let s = self.h.world_to_screen(Point3::new(p.x, p.y, 0.0));
        self.h
            .click_at(s, egui::PointerButton::Primary, egui::Modifiers::NONE);
        self.settle();
    }

    /// Clicks the widget labelled `label`.
    fn click_widget(&mut self, label: &str) {
        self.count(format!("click \"{label}\""));
        self.h.kittest().get_by_label(label).click();
        self.settle();
    }

    /// Types `text` into the field labelled `label` (replacing its text).
    fn type_into(&mut self, label: &str, text: &str) {
        self.count(format!("type \"{text}\" into \"{label}\""));
        self.h.kittest().get_by_label(label).focus();
        self.h.kittest().step();
        self.h
            .kittest()
            .key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
        self.h.kittest().step();
        self.h.kittest().get_by_label(label).type_text(text);
        self.h.kittest().step();
    }

    /// Types `text` with no text field focused (the heads-up input of a creation tool).
    fn type_text(&mut self, text: &str) {
        self.count(format!("type \"{text}\""));
        self.h.kittest().event(egui::Event::Text(text.to_owned()));
        self.h.kittest().step();
    }

    /// Types `text` into the focused dimension editor (it selects its value when it opens).
    fn type_editor(&mut self, text: &str) {
        self.count(format!("type \"{text}\" into the dimension editor"));
        self.h
            .kittest()
            .get_by_label(amg_ui::sketch::dimension::EDIT_FIELD_NAME)
            .type_text(text);
        self.h.kittest().step();
    }

    fn dof(&self) -> u32 {
        self.session().dof().expect("solved").dof
    }
}

/// Net profile areas, sorted.
fn areas(s: &SketchSession) -> Vec<f64> {
    let mut a: Vec<f64> = s
        .profiles()
        .profiles
        .iter()
        .map(|p| p.fingerprint.area)
        .collect();
    a.sort_by(f64::total_cmp);
    a
}

/// The closed-form areas for width `w`, height `h`, fillet radius `r` (holes Ø6, slot 20 × 8).
fn expected_areas(w: f64, h: f64, r: f64) -> Vec<f64> {
    let hole = PI * 3.0 * 3.0;
    let slot = 20.0 * 8.0 + PI * 4.0 * 4.0;
    let outer = w * h - 4.0 * (r * r - PI * r * r / 4.0);
    let plate = outer - 4.0 * hole - slot;
    let mut a = vec![hole, hole, hole, hole, slot, plate];
    a.sort_by(f64::total_cmp);
    a
}

fn assert_areas(got: &[f64], want: &[f64]) {
    assert_eq!(got.len(), want.len(), "{got:?} vs {want:?}");
    for (g, w) in got.iter().zip(want) {
        assert!(((g - w) / w).abs() < 1e-9, "area {g} vs {w} (all: {got:?})");
    }
}

fn all_full(s: &SketchSession) -> Vec<String> {
    let sk = s.working().sketch.clone();
    sk.entities()
        .filter(|(id, _)| s.entity_state(*id) != Some(ConstraintState::Full))
        .map(|(id, e)| format!("{id:?} {} {:?}", e.kind.name(), s.entity_state(id)))
        .collect()
}

#[test]
#[allow(clippy::too_many_lines, reason = "one scenario")]
fn fusion_sketch_basics_bracket_reaches_dof_zero_within_sixty_actions() {
    let mut s = Script::new();
    let (w, h, r) = (80.0, 50.0, 8.0);

    // 1. User parameters.
    s.command(PARAMETERS_CMD);
    for (name, expr) in [("width", "80 mm"), ("height", "50 mm"), ("fillet", "8 mm")] {
        s.type_into("New parameter name", name);
        s.type_into("New parameter expression", expr);
        s.click_widget("Add parameter");
    }
    assert_eq!(s.h.document().design().params().len(), 3);
    s.click_widget("Close window");
    assert!(!s.h.app().dialog_titles().contains(&DIALOG_TITLE.to_owned()));

    // 2. Sketch and rectangle.
    s.command(CREATE_SKETCH_CMD);
    s.click_widget("XY");
    assert!(s.in_sketch());
    // The look-at animation of Create Sketch runs to its end (no input needed).
    s.h.drive_until_idle();
    s.key(egui::Key::R);
    s.click(pt(0.0, 0.0));
    s.click(pt(w, h));
    assert_eq!(s.dof(), 4, "rectangle");

    // 3. Fillets bound to `fillet`.
    s.command(ModifyKind::Fillet.def().id);
    s.type_into(field::RADIUS, "fillet");
    for c in [pt(0.0, 0.0), pt(w, 0.0), pt(w, h), pt(0.0, h)] {
        s.click(c);
    }
    let sk = s.sketch();
    assert_eq!(
        sk.entities()
            .filter(|(_, e)| matches!(e.kind, EntityKind::Arc { .. }))
            .count(),
        4
    );
    assert_eq!(s.dof(), 4, "fillets keep the DOF");

    // 4. Dimensions of the straight sides.
    s.key(egui::Key::D);
    s.click(pt(w / 2.0, 0.0));
    s.click(pt(w / 2.0, -8.0));
    s.type_editor("width - 2*fillet");
    // Clicking the next side commits the value (as in Fusion) and picks the side.
    s.click(pt(w, h / 2.0));
    s.click(pt(w + 8.0, h / 2.0));
    s.type_editor("height - 2*fillet");
    s.key(egui::Key::Enter);
    assert_eq!(s.dof(), 2, "only the position is left");

    // 5. Construction centreline between the side midpoints.
    s.command(LINE_CMD);
    s.key(egui::Key::X);
    s.click(pt(w / 2.0, 0.0));
    s.click(pt(w / 2.0, h));
    s.key(egui::Key::X);
    assert_eq!(s.dof(), 2, "the centreline is fully determined");

    // 6. Two holes on the left fillet centres.
    s.key(egui::Key::C);
    for c in [pt(r, r), pt(r, h - r)] {
        s.click(c);
        s.type_text("6");
        s.key(egui::Key::Enter);
    }
    assert_eq!(s.dof(), 2, "holes on the fillet centres with a diameter");

    // 7. Mirror them about the centreline.
    s.command(ModifyKind::Mirror.def().id);
    s.click(pt(r + 3.0, r));
    s.click(pt(r + 3.0, h - r));
    s.key(egui::Key::Enter);
    s.click(pt(w / 2.0, h * 0.8));
    assert_eq!(s.dof(), 2, "the mirrored holes follow the source");

    // 8. The slot at the centreline's midpoint, horizontal.
    s.command(CreateKind::SlotCenterPoint.def().id);
    s.click(pt(w / 2.0, h / 2.0));
    s.type_text("10");
    s.key(egui::Key::Enter);
    s.type_text("8");
    s.key(egui::Key::Enter);
    assert_eq!(s.dof(), 3, "the slot's direction is free");
    s.command(ConstraintTool::HorizontalVertical.def().id);
    s.click(pt(w / 2.0 + 5.0, h / 2.0 + 4.0));
    assert_eq!(s.dof(), 2, "horizontal slot");

    // 9. Anchor.
    s.command(ConstraintTool::Fix.def().id);
    s.click(pt(r, r));
    let profiles = s.session().profiles().profiles.len();
    assert_eq!(profiles, 6, "plate, four holes and the slot");

    // ---- checks -------------------------------------------------------------------------
    let sess = s.session();
    assert_eq!(sess.dof().unwrap().dof, 0, "fully constrained");
    let not_full = all_full(sess);
    assert!(not_full.is_empty(), "not Full: {not_full:?}");
    let sk = sess.working().sketch.clone();
    let circles = sk
        .entities()
        .filter(|(_, e)| matches!(e.kind, EntityKind::Circle { .. }))
        .count();
    assert_eq!(circles, 4, "two concentric hole pairs");
    let symmetric = sk
        .constraints()
        .filter(|(_, c)| matches!(c.kind, ConstraintKind::Symmetric { .. }))
        .count();
    assert!(symmetric >= 2, "mirrored with symmetric constraints");
    let radii = sk
        .constraints()
        .filter(|(_, c)| {
            matches!(
                c.kind.as_dimension().map(|d| &d.kind),
                Some(DimensionKind::Radius { .. })
            )
        })
        .count();
    assert_eq!(radii, 4);
    // Dimensions driven by the user parameters.
    let design = s.h.document().design().clone();
    let sources: Vec<String> = sess
        .working()
        .dims
        .keys()
        .filter_map(|id| sess.dimension_source(&design, *id))
        .collect();
    for p in ["width", "height", "fillet"] {
        assert!(
            sources.iter().any(|t| t.contains(p)),
            "{p} drives a dimension: {sources:?}"
        );
    }
    assert_areas(&areas(sess), &expected_areas(w, h, r));
    let actions_before_edit = s.actions;

    // ---- changing a parameter keeps DOF 0 -------------------------------------------------
    s.command(PARAMETERS_CMD);
    s.type_into("Expression width", "100 mm");
    s.key(egui::Key::Enter);
    let sess = s.session();
    assert_eq!(sess.dof().unwrap().dof, 0, "still fully constrained");
    assert!(all_full(sess).is_empty());
    assert_areas(&areas(sess), &expected_areas(100.0, h, r));

    eprintln!(
        "UI-16 tutorial: {} UI actions ({} before the parameter edit):\n  {}",
        s.actions,
        actions_before_edit,
        s.log.join("\n  ")
    );
    assert!(
        s.actions <= MAX_ACTIONS,
        "{} actions > {MAX_ACTIONS}",
        s.actions
    );
}
