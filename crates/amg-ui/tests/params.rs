//! UI-12 tests: the Parameters dialog in the real app (`AppHarness`), driven through
//! AccessKit labels. Done-when 1 (user parameter, model parameter, rename), 2 (cycle shown
//! inline, document unchanged), 3 (one undo entry per edit) and 4 (1 000 parameters; the
//! strict timing assertion needs `AMG_STRICT_TIMING=1`, the criterion bench is
//! `benches/params.rs`).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::time::{Duration, Instant};

use amg_core::{Dimension, Unit};
use amg_doc::model::Design;
use amg_doc::{
    Command, Dependency, Expr, ExprSrc, FeatureBox, FeatureError, FeatureOutput, ParamId,
    RecomputeCtx, Revision,
};
use amg_ui::CommitBatch;
use amg_ui::UiRegistry;
use amg_ui::params::{
    DIALOG_TITLE, PARAMETERS_CMD, ParameterRow, ParametersDialog, ParametersView,
};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::{NodeT, Queryable};
use serde::{Deserialize, Serialize};

/// A feature with one length field, so the command layer creates a model parameter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Boxy {
    length: Expr,
}

impl amg_doc::Feature for Boxy {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.length]
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, Dimension)> {
        vec![(&mut self.length, Dimension::LENGTH)]
    }

    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput::default())
    }
}

amg_doc::register_feature!(Boxy, "uitest.params_box", version = 1);

// ---- helpers --------------------------------------------------------------------------------

fn submit(h: &mut AppHarness, commands: Vec<Command>) {
    h.app_mut().submit(CommitBatch {
        label: "Setup".into(),
        commands,
    });
    h.drive_until_idle();
}

fn add_box(h: &mut AppHarness, length: &str) {
    let root = h.document().design().root();
    submit(
        h,
        vec![Command::AddFeature {
            at: None,
            feature: FeatureBox::new(Boxy {
                length: Expr::Inline(ExprSrc::from(length)),
            }),
            context: root,
            name: None,
        }],
    );
}

fn add_user(name: &str, expr: &str) -> Command {
    Command::AddParam {
        name: name.into(),
        expr: expr.into(),
        unit: Some(Unit::Millimetre),
        comment: String::new(),
    }
}

fn open(h: &mut AppHarness) {
    h.run_command(PARAMETERS_CMD);
    assert_eq!(h.app().dialog_titles(), [DIALOG_TITLE]);
}

fn click(h: &mut AppHarness, label: &str) {
    h.kittest().get_by_label(label).click();
    h.drive_until_idle();
}

/// Focuses the text field `label` and replaces its text (without committing).
fn type_over(h: &mut AppHarness, label: &str, text: &str) {
    h.kittest().get_by_label(label).focus();
    h.kittest().step();
    h.kittest()
        .key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
    h.kittest().step();
    h.kittest().get_by_label(label).type_text(text);
    h.kittest().step();
}

/// [`type_over`] and `Enter`.
fn edit(h: &mut AppHarness, label: &str, text: &str) {
    type_over(h, label, text);
    h.key(egui::Key::Enter);
    h.drive_until_idle();
}

fn field(h: &mut AppHarness, label: &str) -> String {
    h.kittest()
        .get_by_label(label)
        .accesskit_node()
        .value()
        .unwrap_or_default()
}

fn shown(h: &mut AppHarness, label: &str) -> bool {
    h.kittest().query_all_by_label(label).next().is_some()
}

fn id(h: &AppHarness, name: &str) -> ParamId {
    h.document()
        .design()
        .params()
        .by_name(name)
        .unwrap_or_else(|| panic!("no parameter `{name}`"))
}

fn source(h: &AppHarness, name: &str) -> String {
    let p = h.document().design().params().get(id(h, name)).unwrap();
    p.expr.as_str().to_owned()
}

fn value(h: &AppHarness, name: &str) -> f64 {
    let params = h.document().design().params();
    params.value(id(h, name)).unwrap().value
}

fn json(h: &AppHarness) -> String {
    serde_json::to_string(h.document().design()).unwrap()
}

fn rows(h: &AppHarness) -> Vec<String> {
    let dialog = h.app().dialog::<ParametersDialog>().expect("open dialog");
    let params = h.document().design().params();
    dialog
        .view()
        .rows()
        .iter()
        .map(|r| match r {
            ParameterRow::Group(g) => format!("[{g}]"),
            ParameterRow::Param(id) => params.get(*id).unwrap().name.clone(),
        })
        .collect()
}

fn revision(h: &AppHarness) -> Revision {
    h.document().history().revision()
}

// ---- Done-when 1 ----------------------------------------------------------------------------

#[test]
fn user_parameter_drives_a_model_parameter_and_rename_rewrites_it() {
    let mut h = AppHarness::new();
    add_box(&mut h, "10 mm");
    open(&mut h);

    type_over(&mut h, "New parameter name", "w");
    type_over(&mut h, "New parameter expression", "20 mm");
    click(&mut h, "Add parameter");
    assert_eq!(source(&h, "w"), "20 mm");
    assert_eq!(field(&mut h, "New parameter name"), "", "the row clears");
    assert!(shown(&mut h, "20.00 mm"));

    edit(&mut h, "Expression d1", "w * 2");
    assert_eq!(source(&h, "d1"), "w * 2");
    assert!((value(&h, "d1") - 40.0).abs() < 1e-12);
    assert!(shown(&mut h, "40.00 mm"), "the value column");

    edit(&mut h, "Name w", "width");
    assert_eq!(source(&h, "d1"), "width * 2");
    assert_eq!(field(&mut h, "Expression d1"), "width * 2");
    assert_eq!(field(&mut h, "Name width"), "width");
    assert!(shown(&mut h, "40.00 mm"));
    assert_eq!(
        rows(&h),
        [
            "[User Parameters]",
            "width",
            "[Model Parameters: params_box1]",
            "d1"
        ]
    );
}

// ---- Done-when 2 ----------------------------------------------------------------------------

#[test]
fn a_cycle_is_shown_inline_and_leaves_the_document_unchanged() {
    let mut h = AppHarness::new();
    submit(&mut h, vec![add_user("a", "1 mm"), add_user("b", "2 mm")]);
    open(&mut h);

    edit(&mut h, "Expression a", "b");
    assert_eq!(source(&h, "a"), "b");
    let (before, rev) = (json(&h), revision(&h));

    // Shown while typing already.
    type_over(&mut h, "Expression b", "a");
    assert!(shown(&mut h, "circular reference: b → a → b"));
    h.key(egui::Key::Enter);
    h.drive_until_idle();

    assert!(shown(&mut h, "circular reference: b → a → b"));
    assert_eq!(field(&mut h, "Expression b"), "a", "the text stays to fix");
    assert_eq!(json(&h), before, "the document is unchanged");
    assert_eq!(revision(&h), rev);
    assert_eq!(source(&h, "b"), "2 mm");
    assert!(h.app().notifications().is_empty(), "no toast");

    // Esc in the field gives the document text back.
    h.kittest().get_by_label("Expression b").focus();
    h.kittest().step();
    h.key(egui::Key::Escape);
    h.drive_until_idle();
    // The first Esc may only close the completion list.
    if field(&mut h, "Expression b") != "2 mm" {
        h.key(egui::Key::Escape);
        h.drive_until_idle();
    }
    assert_eq!(field(&mut h, "Expression b"), "2 mm");
    assert!(!shown(&mut h, "circular reference: b → a → b"));
    assert_eq!(revision(&h), rev);
}

#[test]
fn unit_name_and_in_use_errors_are_shown_inline() {
    let mut h = AppHarness::new();
    add_box(&mut h, "10 mm");
    submit(&mut h, vec![add_user("w", "20 mm"), add_user("v", "w / 2")]);
    open(&mut h);
    let rev = revision(&h);

    edit(&mut h, "Expression d1", "5 kg");
    assert!(shown(&mut h, "unit mismatch: expected mm, got t"));
    edit(&mut h, "Expression d1", "5 mm + nope");
    assert!(shown(&mut h, "unknown parameter `nope`"));
    edit(&mut h, "Expression d1", "12,5");
    assert!(
        h.kittest()
            .query_all_by_label_contains("decimal separator")
            .next()
            .is_some()
    );
    edit(&mut h, "Name v", "w");
    assert!(shown(&mut h, "name `w` already in use"));
    click(&mut h, "Delete w");
    assert!(shown(&mut h, "in use by v"));
    assert_eq!(revision(&h), rev, "nothing was submitted");

    // A parameter a feature reads can't be deleted either; model parameters have no button.
    assert!(!shown(&mut h, "Delete d1"));

    // A rejected edit goes away once the document text it replaced changes.
    let d1 = id(&h, "d1");
    submit(
        &mut h,
        vec![Command::SetParamExpr {
            id: d1,
            expr: "w".into(),
        }],
    );
    assert_eq!(field(&mut h, "Expression d1"), "w");
    assert!(!shown(&mut h, "decimal separator"));
}

// ---- Done-when 3 ----------------------------------------------------------------------------

#[test]
fn each_edit_is_one_undo_entry() {
    let mut h = AppHarness::new();
    let initial = json(&h);
    open(&mut h);
    let mut states = vec![initial];
    let mut step = |h: &mut AppHarness, label: &str, rev: Revision| {
        assert_eq!(revision(h).0, rev.0 + 1, "{label}: one command");
        assert_eq!(h.document().undo_label(), Some(label));
        let now = json(h);
        assert_ne!(states.last(), Some(&now), "{label}");
        states.push(now);
    };

    // Add, with Enter in the add row.
    type_over(&mut h, "New parameter name", "w");
    let rev = revision(&h);
    edit(&mut h, "New parameter expression", "20 mm");
    step(&mut h, "Add parameter", rev);

    let rev = revision(&h);
    edit(&mut h, "Expression w", "25 mm");
    step(&mut h, "Edit parameter", rev);

    let rev = revision(&h);
    edit(&mut h, "Name w", "width");
    step(&mut h, "Rename parameter", rev);

    // A text cell also commits when another widget takes the focus.
    let rev = revision(&h);
    type_over(&mut h, "Comment width", "outer size");
    assert_eq!(revision(&h), rev, "not committed while typing");
    h.kittest().get_by_label("Filter parameters").focus();
    // The focused filter's cursor keeps asking for frames, so step instead of waiting for idle.
    for _ in 0..4 {
        h.kittest().step();
    }
    step(&mut h, "Edit parameter comment", rev);

    let rev = revision(&h);
    click(&mut h, "Favourite width");
    step(&mut h, "Favourite parameter", rev);
    assert_eq!(rows(&h), ["[Favourites]", "width"]);

    // Visiting a field without changing it, and typing the same text, add nothing.
    let rev = revision(&h);
    edit(&mut h, "Expression width", "25 mm");
    edit(&mut h, "Name width", "  width  ");
    assert_eq!(revision(&h), rev);

    let rev = revision(&h);
    click(&mut h, "Delete width");
    step(&mut h, "Delete parameter", rev);
    assert!(h.document().design().params().is_empty());

    // Undo walks back through exactly these states, then there is nothing left.
    assert_eq!(states.len(), 7);
    states.pop();
    let doc = h.app().active_document().id;
    while let Some(expected) = states.pop() {
        h.app_mut().request_undo(doc);
        h.drive_until_idle();
        let now = json(&h);
        assert_eq!(
            params_of(&now),
            params_of(&expected),
            "{} states left",
            states.len()
        );
    }
    assert!(h.document().undo_label().is_none());
}

/// The parameter entries of a serialized design (id counters don't matter for undo).
fn params_of(json: &str) -> serde_json::Value {
    let v: serde_json::Value = serde_json::from_str(json).unwrap();
    v["params"].clone()
}

// ---- table, filter, window ------------------------------------------------------------------

#[test]
fn groups_favourites_and_filter() {
    let mut h = AppHarness::new();
    add_box(&mut h, "10 mm");
    add_box(&mut h, "30 mm");
    submit(
        &mut h,
        vec![add_user("w", "20 mm"), add_user("depth", "3 mm")],
    );
    open(&mut h);
    assert_eq!(
        rows(&h),
        [
            "[User Parameters]",
            "w",
            "depth",
            "[Model Parameters: params_box1]",
            "d1",
            "[Model Parameters: params_box2]",
            "d2",
        ]
    );
    click(&mut h, "Favourite d2");
    assert_eq!(rows(&h)[..2], ["[Favourites]", "d2"]);
    assert!(!rows(&h).contains(&"[Model Parameters: params_box2]".to_owned()));

    edit(&mut h, "Comment depth", "Wall Thickness");
    let rev = revision(&h);
    edit(&mut h, "Filter parameters", "thick");
    assert_eq!(rows(&h), ["[User Parameters]", "depth"]);
    edit(&mut h, "Filter parameters", "30 MM");
    assert_eq!(rows(&h), ["[Favourites]", "d2"]);
    edit(&mut h, "Filter parameters", "zzz");
    assert!(rows(&h).is_empty());
    assert!(shown(&mut h, "No parameter matches the filter."));
    assert_eq!(revision(&h), rev, "filtering edits nothing");

    // Rows are rebuilt only on a document or filter change.
    let rebuilds = |h: &AppHarness| {
        h.app()
            .dialog::<ParametersDialog>()
            .unwrap()
            .view()
            .rebuilds()
    };
    let n = rebuilds(&h);
    for _ in 0..5 {
        h.kittest().step();
    }
    assert_eq!(rebuilds(&h), n);
}

#[test]
fn new_parameter_row_previews_and_refuses() {
    let mut h = AppHarness::new();
    open(&mut h);
    let add_disabled = |h: &mut AppHarness| {
        h.kittest()
            .get_by_label("Add parameter")
            .accesskit_node()
            .is_disabled()
    };
    assert!(add_disabled(&mut h));
    type_over(&mut h, "New parameter name", "mm");
    type_over(&mut h, "New parameter expression", "7");
    h.kittest().step();
    assert!(add_disabled(&mut h));
    assert!(shown(&mut h, "`mm` is a reserved name"));
    type_over(&mut h, "New parameter name", "gap");
    h.kittest().step();
    // A bare number takes the unit of the new parameter.
    assert!(shown(&mut h, "= 7.00 mm"));
    type_over(&mut h, "New parameter comment", "clearance");
    click(&mut h, "Add parameter");
    assert_eq!(source(&h, "gap"), "7");
    let p = h.document().design().params().get(id(&h, "gap")).unwrap();
    assert_eq!(p.comment, "clearance");
    assert_eq!(p.unit, Some(Unit::Millimetre));
}

#[test]
fn the_command_opens_one_dialog_and_it_can_be_reopened() {
    let mut h = AppHarness::new();
    open(&mut h);
    h.run_command(PARAMETERS_CMD);
    assert_eq!(h.app().dialog_titles(), [DIALOG_TITLE]);
    // The title bar's close button.
    h.kittest().get_by_label("Close window").click();
    h.drive_until_idle();
    assert!(h.app().dialog_titles().is_empty());
    open(&mut h);
}

// ---- Done-when 4 ----------------------------------------------------------------------------

/// 1 000 parameters: user parameters, a chain of references and model parameters.
fn thousand() -> Design {
    let mut d = Design::new("params");
    d.add_param("p0", "1 mm", Some(Unit::Millimetre), "base")
        .unwrap();
    for i in 1..1000 {
        let expr = if i % 4 == 0 {
            format!("p{} + 1 mm", i - 1)
        } else {
            format!("{i} mm")
        };
        d.add_param(&format!("p{i}"), &expr, Some(Unit::Millimetre), "note")
            .unwrap();
    }
    d
}

#[test]
fn timing_a_thousand_parameters_render_only_the_visible_rows() {
    let design = thousand();
    let registry = UiRegistry::build();
    let mut view = ParametersView::default();
    let ctx = egui::Context::default();
    let frame = |view: &mut ParametersView, rev: u64| {
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(880.0, 440.0),
            )),
            ..egui::RawInput::default()
        };
        let mut out = ctx.run_ui(input, |ui| {
            let batches = view.show(ui, &design, Revision(rev), &registry);
            assert!(batches.is_empty());
        });
        // There is no renderer to apply the font texture.
        out.textures_delta.clear();
        out.shapes.len()
    };
    frame(&mut view, 1);
    let shapes = frame(&mut view, 1);
    assert_eq!(view.rows().len(), 1001);
    assert!(shapes < 600, "rows out of view are not drawn: {shapes}");

    let median = |mut t: Vec<Duration>| {
        t.sort();
        t[t.len() / 2]
    };
    let steady = median(
        (0..200)
            .map(|_| {
                let t = Instant::now();
                frame(&mut view, 1);
                t.elapsed()
            })
            .collect(),
    );
    // A new document revision every frame: the rows are rebuilt too.
    let rebuild = median(
        (0..200)
            .map(|i| {
                let t = Instant::now();
                frame(&mut view, i + 2);
                t.elapsed()
            })
            .collect(),
    );
    println!("1000 parameters: steady frame median {steady:?}, with rebuild {rebuild:?}");
    if std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1") {
        assert!(steady < Duration::from_millis(2), "steady {steady:?}");
        assert!(rebuild < Duration::from_millis(2), "rebuild {rebuild:?}");
    }
}
