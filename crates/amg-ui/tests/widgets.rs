//! UI-15 tests: the dialog widgets. `ExprInput` (live value, unit-mismatch span with OK
//! disabled, autocomplete with Tab, the decimal-comma hint, Esc / Enter, nudge), the
//! single active `SelectionField` setting the tool's pick filter (in the real app, with UI-04's
//! select rules feeding the field), and AccessKit labels of every widget.
//!
//! Widget tests run in a plain kittest `Harness`; the selection-field test runs the real app in
//! `AppHarness` with `CpuPickService` and a mesh body seen from above.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::borrow::Cow;
use std::cell::RefCell;
use std::sync::Arc;

use amg_core::{Dimension, Dir3, Point3, Unit};
use amg_doc::model::Design;
use amg_doc::stub::IndexedMesh;
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, Command, Dependency, Document, FeatureBox, FeatureError,
    FeatureOutput, ParamId, RecomputeCtx, UnitPrefs,
};
use amg_render::{Camera, Projection};
use amg_ui::CommitBatch;
use amg_ui::command::{CmdAction, CmdId, CommandDef, Enablement};
use amg_ui::input::ViewportEvent;
use amg_ui::module::{Registrar, UiModule};
use amg_ui::pick::PickTarget;
use amg_ui::select::{EntityFilter, SelItem, SelectTool};
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::Harness;
use amg_ui::testing::egui_kittest::kittest::{NodeT, Queryable};
use amg_ui::tool::{FinishReason, InputResult, Tool, ToolCx, ToolInit};
use amg_ui::widgets::{
    ChoiceField, DialogAction, DialogFrame, ExprInput, IntField, SelectionField, check_expr, flip,
    group, toggle,
};
use serde::{Deserialize, Serialize};

const L: Dimension = Dimension::LENGTH;

// ---- ExprInput in a dialog ------------------------------------------------------------------

struct ExprDialog {
    design: Design,
    prefs: UnitPrefs,
    src: String,
    bound: Option<ParamId>,
    value: Option<Result<amg_core::Quantity, amg_doc::ExprError>>,
    span: Option<std::ops::Range<usize>>,
    committed: u32,
    action: DialogAction,
}

impl ExprDialog {
    fn new() -> ExprDialog {
        let mut design = Design::new("widgets");
        design
            .add_param("d1", "10 mm", Some(Unit::Millimetre), "")
            .unwrap();
        design
            .add_param("depth", "3 mm", Some(Unit::Millimetre), "")
            .unwrap();
        design
            .add_param("width", "40 mm", Some(Unit::Millimetre), "")
            .unwrap();
        ExprDialog {
            design,
            prefs: UnitPrefs::default(),
            src: String::new(),
            bound: None,
            value: None,
            span: None,
            committed: 0,
            action: DialogAction::None,
        }
    }
}

fn expr_ui(ui: &mut egui::Ui, st: &mut ExprDialog) {
    // OK follows the field's validity (checked before drawing, as a dialog would).
    let ok = match check_expr(&st.src, L, st.design.params(), &st.prefs).message {
        None => Enablement::Enabled,
        Some(m) => Enablement::Disabled(Cow::Owned(format!("Distance: {m}"))),
    };
    let mut out = None;
    let action = DialogFrame::new("Extrude")
        .ok_enabled(ok)
        .show_inside(ui, |ui| {
            out = Some(
                ExprInput::new(&mut st.src, L, st.design.params(), &st.prefs)
                    .label("Distance")
                    .bound_param(st.bound)
                    .show(ui),
            );
        });
    let out = out.unwrap();
    if action != DialogAction::None {
        st.action = action;
    }
    if out.committed {
        st.committed += 1;
    }
    st.span = out.error_span;
    st.value = Some(out.value);
}

fn expr_harness() -> Harness<'static, ExprDialog> {
    Harness::builder()
        .with_size(egui::vec2(500.0, 300.0))
        .build_ui_state(expr_ui, ExprDialog::new())
}

fn type_into(h: &mut Harness<'_, ExprDialog>, label: &str, text: &str) {
    h.get_by_label(label).focus();
    h.run();
    h.get_by_label(label).type_text(text);
    h.run();
}

#[test]
fn length_field_shows_the_value_and_a_unit_mismatch_disables_ok() {
    let mut h = expr_harness();
    h.run();
    type_into(&mut h, "Distance", "d1 + 5");
    assert_eq!(h.state().src, "d1 + 5");
    assert_eq!(h.state().value, Some(Ok(amg_core::Quantity::new(15.0, L))));
    assert!(h.query_by_label("15.00 mm").is_some(), "value shown");
    assert!(!h.get_by_label("OK").accesskit_node().is_disabled());

    // Replace the text with a mass.
    h.state_mut().src.clear();
    h.run();
    h.get_by_label("Distance").type_text("12 kg");
    h.run();
    assert_eq!(h.state().src, "12 kg");
    assert!(matches!(
        h.state().value,
        Some(Err(amg_doc::ExprError::WrongDimension { .. }))
    ));
    // The whole result has the wrong dimension, so the whole expression is underlined.
    assert_eq!(h.state().span, Some(0..5));
    assert!(
        h.query_by_label("unit mismatch: expected mm, got t")
            .is_some()
    );
    let ok = h.get_by_label("OK");
    assert!(ok.accesskit_node().is_disabled(), "OK disabled");
    // The reason is shown next to the buttons.
    assert!(
        h.query_by_label_contains("Distance: unit mismatch")
            .is_some()
    );
    h.get_by_label("OK").click();
    h.run();
    assert_eq!(
        h.state().action,
        DialogAction::None,
        "a disabled OK does nothing"
    );

    // A mismatch inside the expression underlines the operation's span.
    h.state_mut().src.clear();
    h.run();
    // Clicking OK moved the focus away: click back into the field first.
    type_into(&mut h, "Distance", "2 * (d1 + 3 kg)");
    assert!(
        matches!(
            h.state().value,
            Some(Err(amg_doc::ExprError::UnitMismatch { .. }))
        ),
        "{:?} -> {:?}",
        h.state().src,
        h.state().value
    );
    let span = h.state().span.clone().unwrap();
    // amg-doc's span covers the parenthesised operation.
    assert_eq!(&h.state().src[span], "(d1 + 3 kg)");
}

#[test]
fn typing_d_offers_every_d_parameter_and_tab_completes() {
    let mut h = expr_harness();
    h.run();
    type_into(&mut h, "Distance", "d");
    // Both parameters starting with `d` are offered (and `width` is not).
    assert!(h.query_by_label("d1").is_some());
    assert!(h.query_by_label("depth").is_some());
    assert!(h.query_by_label("width").is_none());
    // Units and functions too.
    assert!(h.query_by_label("deg").is_some());

    // Down moves to the second entry, Tab completes it.
    h.key_press(egui::Key::ArrowDown);
    h.run();
    h.key_press(egui::Key::Tab);
    h.run();
    assert_eq!(h.state().src, "depth");
    assert!(
        h.get_by_label("Distance").is_focused(),
        "Tab kept the focus"
    );
    assert!(h.query_by_label("3.00 mm").is_some());
    assert!(h.query_by_label("d1").is_none(), "list closed");

    // Completion in the middle of an expression replaces only the word.
    h.get_by_label("Distance").type_text(" * 2 + wi");
    h.run();
    h.key_press(egui::Key::Tab);
    h.run();
    assert_eq!(h.state().src, "depth * 2 + width");
    assert!(h.query_by_label("46.00 mm").is_some());
}

#[test]
fn decimal_comma_shows_the_hint() {
    let mut h = expr_harness();
    h.run();
    type_into(&mut h, "Distance", "12,5");
    assert_eq!(h.state().span, Some(2..3));
    assert!(h.query_by_label("Write 12.5, not 12,5.").is_some());
    let msg = check_expr("12,5", L, h.state().design.params(), &h.state().prefs)
        .message
        .unwrap();
    assert!(msg.contains("decimal separator"), "{msg}");
    assert!(h.query_by_label(&msg).is_some(), "message shown");
    assert!(h.get_by_label("OK").accesskit_node().is_disabled());
}

#[test]
fn enter_commits_esc_reverts_and_arrows_nudge() {
    let mut h = expr_harness();
    h.state_mut().src = "12 mm".into();
    h.state_mut().bound = h.state().design.params().by_name("d1");
    h.run();
    // Bound to a model parameter: its name is shown with the value.
    assert!(h.query_by_label("d1 = 12.00 mm").is_some());
    h.get_by_label("Distance").focus();
    h.run();
    h.key_press(egui::Key::ArrowUp);
    h.run();
    assert_eq!(h.state().src, "13 mm");
    h.key_press(egui::Key::ArrowDown);
    h.key_press(egui::Key::ArrowDown);
    h.run();
    assert_eq!(h.state().src, "11 mm");
    h.key_press(egui::Key::Escape);
    h.run();
    assert_eq!(
        h.state().src,
        "12 mm",
        "Esc restores the text from focus time"
    );
    assert_eq!(h.state().committed, 0, "Esc does not commit");
    assert!(!h.get_by_label("Distance").is_focused());

    h.get_by_label("Distance").focus();
    h.run();
    h.get_by_label("Distance").type_text("0");
    h.key_press(egui::Key::Enter);
    h.run();
    assert_eq!(h.state().src, "12 mm0");
    assert_eq!(h.state().committed, 1, "Enter commits once");
}

// ---- every widget by label ------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq)]
enum Extent {
    Distance,
    ToObject,
}

struct AllWidgets {
    design: Design,
    prefs: UnitPrefs,
    expr: String,
    items: Vec<SelItem>,
    active: bool,
    extent: Extent,
    side: u8,
    chain: bool,
    count: i64,
    flipped: bool,
    health: amg_doc::Health,
}

fn all_ui(ui: &mut egui::Ui, st: &mut AllWidgets) {
    DialogFrame::new("Everything")
        .help("https://example.invalid/help")
        .banner(Some(&st.health))
        .ok_enabled(Enablement::Disabled("Select a profile".into()))
        .show_inside(ui, |ui| {
            ExprInput::new(&mut st.expr, L, st.design.params(), &st.prefs)
                .label("Distance")
                .show(ui);
            SelectionField::new(
                "Profile",
                &mut st.items,
                EntityFilter::PROFILE,
                &mut st.active,
            )
            .show(ui);
            group(ui, "Options", |ui| {
                ChoiceField::new(
                    "Extent",
                    &mut st.extent,
                    &[
                        (Extent::Distance, "Distance"),
                        (Extent::ToObject, "To object"),
                    ],
                )
                .show(ui);
                ChoiceField::new("Direction", &mut st.side, &[(0, "⬆"), (1, "⬍"), (2, "⬇")])
                    .buttons()
                    .show(ui);
                toggle(ui, "Chain faces", &mut st.chain);
                IntField::new("Count", &mut st.count).range(1, 10).show(ui);
                flip(ui, "Flip", &mut st.flipped);
            });
        });
}

#[test]
fn every_widget_is_queryable_by_label() {
    let st = AllWidgets {
        design: Design::new("w"),
        prefs: UnitPrefs::default(),
        expr: "5".into(),
        items: Vec::new(),
        active: false,
        extent: Extent::Distance,
        side: 0,
        chain: false,
        count: 3,
        flipped: false,
        health: amg_doc::Health::Warning(vec![amg_doc::Diagnostic {
            message: "Preview is approximate".into(),
            cause: None,
            entities: Vec::new(),
        }]),
    };
    let mut h = Harness::builder()
        .with_size(egui::vec2(500.0, 500.0))
        .build_ui_state(all_ui, st);
    h.run();
    for label in [
        "Distance",
        "Profile",
        "Clear Profile",
        "Extent",
        "Chain faces",
        "Count",
        "Flip",
        "OK",
        "Cancel",
        "Help",
        "Options",
        "⬆",
        "⬇",
    ] {
        assert!(
            h.query_by_label(label).is_some(),
            "no widget labelled {label:?}"
        );
    }
    assert!(
        h.query_by_label("⚠ Preview is approximate").is_some(),
        "banner"
    );
    assert!(h.query_by_label("Select a profile").is_some(), "OK reason");
    assert!(h.query_by_label("0 selected").is_some());
    assert!(h.get_by_label("OK").accesskit_node().is_disabled());
    // The label nodes drive the widgets.
    h.get_by_label("Chain faces").click();
    h.get_by_label("⬇").click();
    h.get_by_label("Flip").click();
    h.run();
    assert!(h.state().chain);
    assert_eq!(h.state().side, 2);
    assert!(h.state().flipped);
    h.get_by_label("Extent").click();
    h.run();
    h.get_by_label("To object").click();
    h.run();
    assert_eq!(h.state().extent, Extent::ToObject);
    h.get_by_label("Profile").click();
    h.run();
    assert!(h.state().active, "clicking the field activates it");
}

// ---- SelectionField in a real tool ------------------------------------------------------------

/// A flat `n × n` grid at `z = 0` from `(x0, 0)`, 1 mm cells.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct WidgetGrid {
    n: u32,
    x0: f64,
}

fn grid_mesh(n: u32, x0: f64) -> IndexedMesh {
    let mut pos = Vec::new();
    for j in 0..=n {
        for i in 0..=n {
            pos.push(Point3::new(x0 + f64::from(i), f64::from(j), 0.0));
        }
    }
    let w = n + 1;
    let mut tris = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let a = j * w + i;
            tris.push([a, a + 1, a + w + 1]);
            tris.push([a, a + w + 1, a + w]);
        }
    }
    IndexedMesh::new(pos, tris).expect("valid grid")
}

impl amg_doc::Feature for WidgetGrid {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(Arc::new(grid_mesh(self.n, self.x0))),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(WidgetGrid, "uitest.widget_grid", version = 1);

const FIELDS_TOOL: CmdId = CmdId("uitest.fields_tool");

/// (effective pick filter, faces active, bodies active, face items, body items) per frame.
type Seen = (EntityFilter, bool, bool, usize, usize);

thread_local! {
    static SEEN: RefCell<Vec<Seen>> = const { RefCell::new(Vec::new()) };
}

fn last_seen() -> Seen {
    SEEN.with(|s| *s.borrow().last().expect("tool ran"))
}

/// Two selection fields; UI-04's select rules (a forwarded `SelectTool`) feed the active one.
#[derive(Default)]
struct FieldsTool {
    picker: SelectTool,
    faces: Vec<SelItem>,
    faces_active: bool,
    bodies: Vec<SelItem>,
    bodies_active: bool,
}

impl Tool for FieldsTool {
    fn id(&self) -> CmdId {
        FIELDS_TOOL
    }

    fn label(&self) -> Cow<'static, str> {
        "Fields tool".into()
    }

    fn input(&mut self, ev: &ViewportEvent, cx: &mut ToolCx<'_>) -> InputResult {
        self.picker.input(ev, cx)
    }

    fn update(&mut self, cx: &mut ToolCx<'_>) {
        let f = cx.pick_filter();
        SEEN.with(|s| {
            s.borrow_mut().push((
                f,
                self.faces_active,
                self.bodies_active,
                self.faces.len(),
                self.bodies.len(),
            ));
        });
    }

    fn dialog_ui(&mut self, ui: &mut egui::Ui, cx: &mut ToolCx<'_>) -> bool {
        SelectionField::new(
            "Target faces",
            &mut self.faces,
            EntityFilter::FACE,
            &mut self.faces_active,
        )
        .mirror(cx.selection_mut())
        .show(ui);
        SelectionField::new(
            "Target body",
            &mut self.bodies,
            EntityFilter::BODY,
            &mut self.bodies_active,
        )
        .limits(1, Some(1))
        .mirror(cx.selection_mut())
        .show(ui);
        true
    }

    fn finish(&mut self, _reason: FinishReason, _cx: &mut ToolCx<'_>) -> Option<CommitBatch> {
        None
    }
}

fn start_fields(_init: &ToolInit<'_>) -> Box<dyn Tool> {
    Box::new(FieldsTool {
        // Auto-focus the first field, as UI-05 does.
        faces_active: true,
        ..FieldsTool::default()
    })
}

inventory::submit! { UiModule { id: "zz_widgets_test", register } }

fn register(r: &mut Registrar<'_>) {
    r.command(CommandDef::new(
        FIELDS_TOOL.0,
        "Fields tool",
        CmdAction::StartTool(start_fields),
    ));
}

fn add_feature(f: FeatureBox, doc: &Document) -> Command {
    Command::AddFeature {
        at: None,
        feature: f,
        context: doc.design().root(),
        name: None,
    }
}

fn top_down(x: f64, y: f64, height: f64) -> Camera {
    Camera {
        eye: Point3::new(x, y, height),
        target: Point3::new(x, y, 0.0),
        up: Dir3::Y,
        projection: Projection::Perspective { fov_y: 0.8 },
    }
}

#[test]
fn only_one_selection_field_is_active_and_it_sets_the_pick_filter() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let cmd = add_feature(FeatureBox::new(WidgetGrid { n: 4, x0: 0.0 }), h.document());
    h.app_mut().submit(CommitBatch {
        label: "Grid".into(),
        commands: vec![cmd],
    });
    h.drive_until_idle();
    h.app_mut().active_document_mut().camera = top_down(2.0, 2.0, 12.0);
    h.drive_until_idle();
    let body = h.scene().bodies().next().expect("one body");

    h.run_command(FIELDS_TOOL);
    h.drive_until_idle();
    h.kittest().run();
    // The auto-focused Faces field is the active one and narrows the tool's filter.
    let (f, faces, bodies, _, _) = last_seen();
    assert!(faces && !bodies);
    assert_eq!(f, EntityFilter::FACE);

    // Clicking Bodies activates it and deactivates Faces.
    h.kittest().get_by_label("Target body").click();
    h.kittest().run();
    h.drive_until_idle();
    let (f, faces, bodies, _, _) = last_seen();
    assert!(!faces && bodies, "exactly one field active");
    assert_eq!(f, EntityFilter::BODY);

    // A click on the grid goes through UI-04's select rules under the field's filter: the
    // whole body lands in the Bodies field.
    h.click_world(Point3::new(1.3, 2.6, 0.0));
    h.drive_until_idle();
    h.kittest().run();
    let (_, _, _, nf, nb) = last_seen();
    assert_eq!((nf, nb), (0, 1));
    let sel: Vec<PickTarget> = h
        .app()
        .active_document()
        .selection
        .items()
        .iter()
        .map(|i| i.target)
        .collect();
    assert_eq!(
        sel,
        [PickTarget::Body {
            body,
            instance: 0,
            sub: None
        }]
    );
    assert!(h.kittest().query_by_label("1 selected").is_some());

    // Back to Faces: the selection becomes the (empty) face list, and a click picks a face.
    h.kittest().get_by_label("Target faces").click();
    h.kittest().run();
    h.drive_until_idle();
    let (f, faces, bodies, _, nb) = last_seen();
    assert!(faces && !bodies);
    assert_eq!(f, EntityFilter::FACE);
    assert_eq!(nb, 1, "the inactive field keeps its items");
    assert!(h.app().active_document().selection.is_empty());
    h.click_world(Point3::new(1.3, 2.6, 0.0));
    h.drive_until_idle();
    h.kittest().run();
    let (_, _, _, nf, nb) = last_seen();
    assert_eq!((nf, nb), (1, 1));
    let face = h.app().active_document().selection.items()[0].target;
    assert!(matches!(
        face,
        PickTarget::Body {
            sub: Some((amg_doc::TopoKind::Face, _)),
            ..
        }
    ));

    // After the tool ends, nothing narrows the filter any more.
    h.kittest().key_press(egui::Key::Escape);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None);
}
