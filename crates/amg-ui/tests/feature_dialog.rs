//! UI-05 tests: the generic feature dialog (`amg_ui::feature_dialog`), kittest on the real app
//! with the `NullViewport`.
//!
//! - Done-when 1: the Extrude dialog comes from SOL-01's schema alone (grep test over the
//!   renderer's sources), with its generated command, toolbar placement and timeline info;
//!   changing the operation shows / hides the dependent field (the SOL-01 schema has no extent
//!   type yet, see the S11 log); `enabled_if`, groups and every widget kind on a test
//!   registration.
//! - Done-when 2 (`occt`): pick a profile in the viewport, type a distance, the preview is ready
//!   within debounce + 100 ms (asserted under `AMG_STRICT_TIMING=1`, printed always); OK is one
//!   undo entry holding `AddFeature`.
//! - Done-when 3: editing the distance of an existing extrude keeps its model parameter `dN`
//!   and changes its expression (the commands apply without a kernel; with `occt` the volume
//!   follows).
//! - Done-when 4: an invalid input disables OK with the field error; Cancel leaves the
//!   document revision unchanged.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

#[cfg(feature = "occt")]
use std::time::{Duration, Instant};

use amg_core::{Point2, Point3};
use amg_doc::stub::ProfileKey;
use amg_doc::{
    Command, CreatedId, Dependency, Design, Document, EntityRef, Evaluated, Expr, Feature,
    FeatureBox, FeatureError, FeatureId, FeatureOutput, RecomputeCtx, SketchId,
};
use amg_features::dialog::{
    ChoiceOption, ChoiceStyle, CommandMeta, Condition, DialogInit, DialogRegistration,
    DialogSchema, EntityFilter as DialogFilter, FieldErrors, FieldKey, FieldKind, FieldSpec,
    FieldValue, FieldValues, ManipAnchor,
};
use amg_features::sketch::{PlaneInput, SketchFeature};
use amg_features::solid::{Extrude, OpTarget};
use amg_render::{Camera, Projection};
use amg_sketch::build;
use amg_ui::command::{CmdAction, CmdId, Placement};
use amg_ui::feature_dialog::{self, GenericFeatureTool};
use amg_ui::module::GroupId;
use amg_ui::preview::PreviewState;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::{NodeT as _, Queryable as _};
use amg_ui::timeline::EditAction;
use amg_ui::{FinishReason, UiRegistry};
use serde::{Deserialize, Serialize};

const EXTRUDE: CmdId = CmdId("feature.solid.extrude");

// ---- helpers --------------------------------------------------------------------------------

fn created_feature(out: &amg_doc::CommandOutcome) -> FeatureId {
    out.created
        .iter()
        .find_map(|c| match c {
            CreatedId::Feature(f) => Some(*f),
            _ => None,
        })
        .expect("a feature was created")
}

fn add(doc: &mut Document, f: FeatureBox) -> FeatureId {
    let root = doc.design().root();
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context: root,
            name: None,
        })
        .unwrap();
    created_feature(&out)
}

/// A document with a 20 × 10 rectangle sketch on XY at the origin.
fn rect_doc() -> (Document, FeatureId) {
    let mut doc = Document::new("ui05");
    let mut s = SketchFeature::new(PlaneInput::OriginXY);
    s.edit_sketch(|sk| build::rectangle_2pt(sk, Point2::new(0.0, 0.0), Point2::new(20.0, 10.0)))
        .unwrap();
    let id = add(&mut doc, FeatureBox::new(s));
    (doc, id)
}

/// Every profile of sketch feature `s` as `EntityRef::Profile`.
fn profiles(eval: &Evaluated, s: FeatureId) -> Vec<EntityRef> {
    let g = eval.sketch(SketchId(s)).expect("the sketch is evaluated");
    g.profiles
        .profiles
        .iter()
        .map(|p| EntityRef::Profile {
            sketch: SketchId(s),
            profile: ProfileKey(p.id.0),
            fingerprint: Some(amg_doc::naming::ProfileFp(p.fingerprint.clone())),
        })
        .collect()
}

/// The app on `doc`, recomputed, seen from above the rectangle.
fn harness(doc: Document) -> AppHarness {
    let mut h = AppHarness::builder()
        .size(egui::vec2(1400.0, 900.0))
        .build();
    let id = h.app_mut().add_document("ui05".into(), doc);
    h.app_mut().activate(id);
    h.drive_until_idle();
    h.app_mut().active_document_mut().camera = Camera {
        eye: Point3::new(10.0, 5.0, 150.0),
        target: Point3::new(10.0, 5.0, 0.0),
        up: amg_core::Dir3::Y,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    h.drive_until_idle();
    h
}

fn tool(h: &mut AppHarness) -> &mut GenericFeatureTool {
    h.app_mut()
        .active_tool_as::<GenericFeatureTool>()
        .expect("the generic dialog is active")
}

fn shown(h: &mut AppHarness, label: &str) -> bool {
    h.kittest().query_all_by_label(label).next().is_some()
}

fn applied(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

/// Replaces the text of the expression field `label`.
fn retype(h: &mut AppHarness, label: &str, text: &str) {
    h.kittest().get_by_label(label).focus();
    h.kittest().step();
    h.kittest()
        .key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::A);
    h.kittest().step();
    h.kittest().get_by_label(label).type_text(text);
    h.kittest().step();
    // Leave the field (as Tab / a click elsewhere would): the caret blink would keep the
    // harness from idling.
    let ctx = h.kittest().ctx.clone();
    ctx.memory_mut(|m| {
        if let Some(id) = m.focused() {
            m.surrender_focus(id);
        }
    });
    h.kittest().step();
}

fn ok_disabled(h: &mut AppHarness) -> bool {
    h.kittest()
        .get_by_label("OK")
        .accesskit_node()
        .is_disabled()
}

#[cfg(feature = "occt")]
fn timing_strict() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

// ---- Done-when 1: generated from the schema --------------------------------------------------

#[test]
fn dw1_the_renderer_has_no_kind_specific_code() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/feature_dialog");
    let mut files = 0;
    for entry in std::fs::read_dir(&dir).unwrap() {
        let path = entry.unwrap().path();
        let text = std::fs::read_to_string(&path).unwrap().to_lowercase();
        files += 1;
        for word in ["extrude", "solid.", "profiles\"", "distance\"", "new_body"] {
            assert!(
                !text.contains(word),
                "{} mentions `{word}`: the renderer must stay generic",
                path.display()
            );
        }
    }
    assert!(files >= 3, "the renderer's sources were found");
}

#[test]
fn dw1_every_registration_gets_a_command_a_toolbar_slot_and_timeline_info() {
    let reg = UiRegistry::build();
    for d in amg_features::dialog::registrations() {
        let id = feature_dialog::command_id(d.kind);
        let cmd = reg
            .command(id)
            .unwrap_or_else(|| panic!("no command for {}", d.kind));
        assert!(matches!(cmd.action, CmdAction::StartTool(_)));
        assert_eq!(reg.command_owner(id), Some(feature_dialog::MODULE));
        assert_eq!(cmd.label, (d.schema)().title.as_str());
        let info = reg.feature_ui(d.kind).expect("timeline info");
        assert!(matches!(info.edit, EditAction::GenericDialog));
        assert_eq!(info.display_name, (d.schema)().title);
    }
    let cmd = reg.command(EXTRUDE).expect("feature.solid.extrude");
    assert_eq!(
        cmd.placements,
        vec![Placement::Toolbar {
            group: GroupId("design.solid.create"),
            pinned: true,
            order: 10,
        }]
    );
    assert!(cmd.shortcut.is_some(), "the registration's shortcut E");
    assert!(cmd.keywords.contains(&"boss"));
    assert!(
        reg.group_commands(GroupId("design.solid.create"))
            .any(|(pinned, c)| pinned && c.id == EXTRUDE),
        "a pinned button in SOLID ▸ CREATE"
    );
    assert_eq!(
        reg.feature_ui("solid.extrude").unwrap().display_name,
        "Extrude"
    );
}

#[test]
fn dw1_extrude_fields_come_from_the_schema_and_follow_the_conditions() {
    let (doc, _) = rect_doc();
    let mut h = harness(doc);
    h.run_command(EXTRUDE);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(EXTRUDE));
    let schema = (amg_features::dialog::registration("solid.extrude")
        .unwrap()
        .schema)();
    for f in &schema.fields {
        let want = f.visible_if.is_none();
        assert_eq!(shown(&mut h, &f.label), want, "field {}", f.label);
    }
    // Auto-focus: the first empty selection field is active.
    assert_eq!(tool(&mut h).active_field(), Some(FieldKey("profiles")));

    // Operation → Join shows the bodies field ("Objects"); back to New body hides it.
    h.kittest().get_by_label("Operation").click();
    h.kittest().run();
    h.kittest().get_by_label("Join").click();
    h.drive_until_idle();
    assert_eq!(
        tool(&mut h).values().get("operation"),
        Some(&FieldValue::Choice("join"))
    );
    assert!(shown(&mut h, "Objects"), "Join shows the bodies field");
    h.kittest().get_by_label("Operation").click();
    h.kittest().run();
    h.kittest().get_by_label("New body").click();
    h.drive_until_idle();
    assert!(!shown(&mut h, "Objects"), "New body hides it");
    h.app_mut().finish_tool(FinishReason::Cancel);
}

// A registration of a test feature with the widget kinds Extrude does not use.

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Ui05Widgets {
    count: i64,
    name: String,
    smooth: bool,
    mode: String,
    radius: Expr,
}

impl Feature for Ui05Widgets {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, _ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        Ok(FeatureOutput::default())
    }

    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.radius]
    }

    fn exprs_mut(&mut self) -> Vec<(&mut Expr, amg_core::Dimension)> {
        vec![(&mut self.radius, amg_core::Dimension::LENGTH)]
    }
}

amg_doc::register_feature!(Ui05Widgets, "uitest.ui05_widgets", version = 1);

fn widgets_schema() -> &'static DialogSchema {
    static S: std::sync::OnceLock<DialogSchema> = std::sync::OnceLock::new();
    S.get_or_init(|| {
        let mut radius = FieldSpec::new(
            "radius",
            "Radius",
            FieldKind::Expr {
                dim: amg_doc::Dim::LENGTH,
                default: "2 mm",
            },
        );
        radius.enabled_if = Some(Condition::Is(FieldKey("smooth"), FieldValue::Bool(true)));
        radius.group = Some("Smoothing".into());
        let mut smooth = FieldSpec::new("smooth", "Smooth", FieldKind::Bool);
        smooth.group = Some("Smoothing".into());
        DialogSchema {
            kind: "uitest.ui05_widgets",
            title: "UI05 Widgets".into(),
            help: None,
            fields: vec![
                FieldSpec::new("count", "Count", FieldKind::Int { min: 1, max: 9 }),
                smooth,
                FieldSpec::new("name", "Name", FieldKind::Text),
                FieldSpec::new(
                    "mode",
                    "Mode",
                    FieldKind::Choice {
                        options: vec![
                            ChoiceOption::new("a", "Mode A"),
                            ChoiceOption::new("b", "Mode B"),
                        ],
                        style: ChoiceStyle::IconRow,
                    },
                ),
                radius,
                FieldSpec::new(
                    "faces",
                    "Faces",
                    FieldKind::Selection {
                        filter: DialogFilter::FACE,
                        min: 0,
                        max: None,
                    },
                ),
            ],
        }
    })
}

fn widgets_defaults(_: &DialogInit<'_>) -> FieldValues {
    FieldValues::default()
        .with("count", FieldValue::Int(3))
        .with("smooth", FieldValue::Bool(false))
        .with("name", FieldValue::Text("w".into()))
        .with("mode", FieldValue::Choice("a"))
        .with("radius", FieldValue::expr("2 mm"))
}

fn widgets_read(f: &dyn Feature, design: &Design) -> Option<FieldValues> {
    let w = f.as_any().downcast_ref::<Ui05Widgets>()?;
    Some(
        FieldValues::default()
            .with("count", FieldValue::Int(w.count))
            .with("smooth", FieldValue::Bool(w.smooth))
            .with("name", FieldValue::Text(w.name.clone()))
            .with(
                "mode",
                FieldValue::Choice(if w.mode == "b" { "b" } else { "a" }),
            )
            .with("radius", FieldValue::from_expr(&w.radius, design)),
    )
}

fn widgets_build(v: &FieldValues, _old: Option<&dyn Feature>) -> Result<FeatureBox, FieldErrors> {
    let name = match v.get("name") {
        Some(FieldValue::Text(s)) if !s.is_empty() => s.clone(),
        _ => return Err((FieldKey("name"), "a name is required".to_owned()).into()),
    };
    let count = match v.get("count") {
        Some(FieldValue::Int(i)) => *i,
        _ => 1,
    };
    Ok(FeatureBox::new(Ui05Widgets {
        count,
        name,
        smooth: v.flag("smooth").map_err(FieldErrors::from)?,
        mode: v.choice("mode", "a").map_err(FieldErrors::from)?.to_owned(),
        radius: v.expr("radius").map_err(FieldErrors::from)?,
    }))
}

fn widgets_anchors(_: &dyn Feature, _: &Design, _: &Evaluated) -> Vec<ManipAnchor> {
    Vec::new()
}

inventory::submit! {
    DialogRegistration {
        kind: "uitest.ui05_widgets",
        schema: widgets_schema,
        defaults: widgets_defaults,
        read: widgets_read,
        build: widgets_build,
        anchors: widgets_anchors,
        command: CommandMeta {
            workspace: "design",
            tab: "solid",
            group: "create",
            order: 900,
            pinned: false,
            icon_svg: &[],
            shortcut: None,
            keywords: &[],
        },
        new_component: None,
    }
}

const WIDGETS: CmdId = CmdId("feature.uitest.ui05_widgets");

#[test]
fn dw1_every_field_kind_renders_with_groups_and_enabled_if() {
    let (doc, _) = rect_doc();
    let mut h = harness(doc);
    h.run_command(WIDGETS);
    h.drive_until_idle();
    for label in [
        "Count",
        "Smooth",
        "Name",
        "Mode A",
        "Mode B",
        "Radius",
        "Faces",
        "Smoothing",
    ] {
        assert!(shown(&mut h, label), "{label} shown");
    }
    // `enabled_if`: Radius is disabled until Smooth is on.
    assert!(
        h.kittest()
            .get_by_label("Radius")
            .accesskit_node()
            .is_disabled()
    );
    h.kittest().get_by_label("Smooth").click();
    h.drive_until_idle();
    assert!(
        !h.kittest()
            .get_by_label("Radius")
            .accesskit_node()
            .is_disabled()
    );
    // An icon-row choice.
    h.kittest().get_by_label("Mode B").click();
    h.drive_until_idle();
    assert_eq!(
        tool(&mut h).values().get("mode"),
        Some(&FieldValue::Choice("b"))
    );
    // A build error disables OK with the field's message.
    tool(&mut h)
        .values_mut()
        .0
        .insert(FieldKey("name"), FieldValue::Text(String::new()));
    h.drive_until_idle();
    assert!(ok_disabled(&mut h));
    assert!(
        h.kittest()
            .query_by_label_contains("Name: a name is required")
            .is_some()
    );
    tool(&mut h)
        .values_mut()
        .0
        .insert(FieldKey("name"), FieldValue::Text("w2".into()));
    h.drive_until_idle();
    assert!(!ok_disabled(&mut h));
    let before = applied(&h);
    h.kittest().get_by_label("OK").click();
    h.drive_until_idle();
    assert_eq!(applied(&h), before + 1);
    let doc = h.document();
    let (_, node) = doc
        .design()
        .timeline()
        .nodes()
        .last()
        .map(|n| (n.id, n))
        .unwrap();
    let w = node.feature.downcast_ref::<Ui05Widgets>().unwrap();
    assert_eq!(
        (w.count, w.smooth, w.mode.as_str(), w.name.as_str()),
        (3, true, "b", "w2")
    );
}

// ---- Done-when 3: edit mode keeps the model parameter ------------------------------------------

/// A document with the rectangle and a 10 mm extrude of it (recomputed).
fn extruded() -> (Document, FeatureId, FeatureId) {
    let (mut doc, s) = rect_doc();
    let jobs = amg_jobs::JobSystem::new(amg_jobs::JobSystemConfig::default()).unwrap();
    doc.recompute_blocking(&jobs).unwrap();
    let ext = Extrude::distance(profiles(doc.evaluated(), s), "10 mm", OpTarget::default());
    let e = add(&mut doc, FeatureBox::new(ext));
    let _ = doc.recompute_blocking(&jobs);
    (doc, s, e)
}

fn extrude_param(doc: &Document, e: FeatureId) -> amg_doc::ParamId {
    let (_, node) = doc.design().timeline().node(e).unwrap();
    let x = node.feature.downcast_ref::<Extrude>().unwrap();
    match x.side1.extent {
        amg_features::solid::Extent::Distance(Expr::Param(p)) => p,
        ref other => panic!("the distance is not a model parameter: {other:?}"),
    }
}

#[test]
fn dw3_editing_the_distance_keeps_the_model_parameter() {
    let (doc, _, e) = extruded();
    let p = extrude_param(&doc, e);
    let name = doc.design().params().get(p).unwrap().name.clone();
    let mut h = harness(doc);
    h.app_mut().edit_feature(e);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(EXTRUDE));
    assert_eq!(tool(&mut h).editing(), Some(e));
    assert_eq!(
        tool(&mut h).values().get("distance"),
        Some(&FieldValue::Expr {
            src: "10 mm".into(),
            param: Some(p),
        })
    );
    // The bound parameter's name is shown with the value.
    assert!(
        h.kittest()
            .query_by_label_contains(&format!("{name} ="))
            .is_some(),
        "`{name} = …` shown"
    );
    let before = applied(&h);
    retype(&mut h, "Distance", "25 mm");
    h.drive_until_idle();
    assert_eq!(
        tool(&mut h).values().get("distance"),
        Some(&FieldValue::Expr {
            src: "25 mm".into(),
            param: Some(p),
        })
    );
    h.kittest().get_by_label("OK").click();
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None);
    assert_eq!(applied(&h), before + 1, "one undo entry");
    let doc = h.document();
    assert_eq!(extrude_param(doc, e), p, "still bound to {name}");
    let param = doc.design().params().get(p).unwrap();
    assert_eq!(param.name, name);
    assert_eq!(param.expr.as_str(), "25 mm");
    // Undo restores the old expression in one step.
    h.app_mut().active_document_mut().document.undo();
    assert_eq!(
        h.document().design().params().get(p).unwrap().expr.as_str(),
        "10 mm"
    );
}

#[test]
fn dw3_edit_without_changes_commits_nothing() {
    let (doc, _, e) = extruded();
    let mut h = harness(doc);
    let before = (applied(&h), h.document().history().revision());
    h.app_mut().edit_feature(e);
    h.drive_until_idle();
    assert!(matches!(
        h.app_mut().preview_driver().state(),
        PreviewState::Idle
    ));
    h.kittest().get_by_label("OK").click();
    h.drive_until_idle();
    assert_eq!((applied(&h), h.document().history().revision()), before);
}

// ---- Done-when 4: invalid input, Cancel ---------------------------------------------------------

#[test]
fn dw4_invalid_input_disables_ok_and_cancel_changes_nothing() {
    let (doc, s) = rect_doc();
    let mut h = harness(doc);
    let profs = profiles(h.document().evaluated(), s);
    assert_eq!(profs.len(), 1);
    // A profile in the preselection fills the field (SOL-01 `defaults`).
    let item = amg_ui::SelItem {
        target: amg_ui::PickTarget::Sketch(SketchId(s)),
        entity: profs[0].clone(),
    };
    h.app_mut().active_document_mut().selection.set(vec![item]);
    let before = h.document().history().revision();
    h.run_command(EXTRUDE);
    h.drive_until_idle();
    assert_eq!(
        tool(&mut h).values().get("profiles"),
        Some(&FieldValue::Selection(profs.clone()))
    );
    assert_eq!(
        tool(&mut h).active_field(),
        None,
        "no empty selection field"
    );
    assert!(!ok_disabled(&mut h), "valid defaults enable OK");

    retype(&mut h, "Distance", "12 kg");
    h.drive_until_idle();
    assert!(ok_disabled(&mut h), "a unit mismatch disables OK");
    assert!(
        h.kittest()
            .query_by_label_contains("Distance: unit mismatch")
            .is_some(),
        "the field error is OK's reason"
    );
    assert!(!tool(&mut h).ok_state().is_enabled());
    // Enter does not commit an invalid dialog.
    h.key(egui::Key::Enter);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(EXTRUDE));

    // Clearing the profiles: OK names the selection field.
    retype(&mut h, "Distance", "12 mm");
    h.drive_until_idle();
    h.kittest().get_by_label("Clear Profiles").click();
    h.drive_until_idle();
    assert!(ok_disabled(&mut h));
    assert!(
        h.kittest()
            .query_by_label_contains("Profiles: select at least 1")
            .is_some()
    );

    h.kittest().get_by_label("Cancel").click();
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None);
    assert_eq!(
        h.document().history().revision(),
        before,
        "Cancel leaves the document unchanged"
    );
    assert!(matches!(
        h.app_mut().preview_driver().state(),
        PreviewState::Idle
    ));
}

#[test]
fn profiles_are_picked_in_the_viewport() {
    let (doc, s) = rect_doc();
    let mut h = harness(doc);
    h.run_command(EXTRUDE);
    h.drive_until_idle();
    assert_eq!(tool(&mut h).active_field(), Some(FieldKey("profiles")));
    // Shaded profiles are drawn while the field is shown.
    assert!(!h.app().tool_overlays().is_empty());
    h.click_world(Point3::new(5.0, 5.0, 0.0));
    h.drive_until_idle();
    let want = profiles(h.document().evaluated(), s);
    assert_eq!(
        tool(&mut h).values().get("profiles"),
        Some(&FieldValue::Selection(want))
    );
    assert!(shown(&mut h, "1 selected"));
    // A second click on it removes it; a click outside picks nothing.
    h.click_world(Point3::new(5.0, 5.0, 0.0));
    h.drive_until_idle();
    h.click_world(Point3::new(30.0, 5.0, 0.0));
    h.drive_until_idle();
    assert_eq!(
        tool(&mut h).values().get("profiles"),
        Some(&FieldValue::Selection(Vec::new()))
    );
    h.app_mut().finish_tool(FinishReason::Cancel);
}

// ---- Done-when 2: preview and commit (needs a kernel) --------------------------------------------

#[cfg(feature = "occt")]
#[test]
fn dw2_pick_type_preview_ok() {
    let (doc, s) = rect_doc();
    let mut h = harness(doc);
    h.run_command(EXTRUDE);
    h.drive_until_idle();
    h.click_world(Point3::new(5.0, 5.0, 0.0));
    h.drive_until_idle();
    let want = profiles(h.document().evaluated(), s);
    assert_eq!(
        tool(&mut h).values().get("profiles"),
        Some(&FieldValue::Selection(want))
    );
    // The default distance already previews; time a typed change.
    let debounce = h.app_mut().preview_driver().debounce();
    let mut samples = Vec::new();
    for (i, text) in ["15 mm", "22 mm", "7.5 mm"].iter().enumerate() {
        // From before the typing: the debounce starts with the first changed frame.
        let t0 = Instant::now();
        retype(&mut h, "Distance", text);
        let ready = loop {
            h.kittest().step();
            if let PreviewState::Ready(ev) = h.app_mut().preview_driver().state().clone()
                && tool(&mut h).values().get("distance") == Some(&FieldValue::expr(text))
                && !h.app_mut().preview_driver().is_busy()
            {
                break ev;
            }
            assert!(
                t0.elapsed() < Duration::from_secs(10),
                "preview {i} never arrived"
            );
            std::thread::sleep(Duration::from_millis(1));
        };
        let dt = t0.elapsed();
        samples.push(dt);
        // The preview has the new body.
        let committed = h.document().evaluated().clone();
        let new: Vec<_> = ready
            .bodies()
            .filter(|(id, _)| committed.body(*id).is_none())
            .collect();
        assert_eq!(new.len(), 1, "one previewed body");
    }
    let worst = samples.iter().max().copied().unwrap();
    println!(
        "UI-05 preview latency after typing: {:?} (debounce {debounce:?}, worst {worst:?})",
        samples
    );
    if timing_strict() {
        assert!(
            worst <= debounce + Duration::from_millis(100),
            "preview after {worst:?} > debounce {debounce:?} + 100 ms"
        );
    }
    // OK: one undo entry holding the AddFeature.
    let before = applied(&h);
    let nodes = h.document().design().timeline().nodes().count();
    h.kittest().get_by_label("OK").click();
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None);
    assert_eq!(applied(&h), before + 1, "one undo entry");
    assert_eq!(h.document().undo_label(), Some("Extrude"));
    assert_eq!(h.document().design().timeline().nodes().count(), nodes + 1);
    let ext = h
        .document()
        .design()
        .timeline()
        .nodes()
        .last()
        .unwrap()
        .feature
        .downcast_ref::<Extrude>()
        .cloned()
        .expect("an extrude was added");
    assert_eq!(ext.profiles.len(), 1);
    let ev = h.document().evaluated();
    let vol: f64 = ev
        .bodies()
        .filter_map(|(_, b)| match &b.geom {
            amg_doc::BodyGeom::Solid(s) => s.brep().map(|shape| {
                use amg_brep::BrepKernel as _;
                amg_doc::naming::default_kernel()
                    .mass_props(shape)
                    .unwrap()
                    .volume
            }),
            _ => None,
        })
        .sum();
    assert!((vol - 20.0 * 10.0 * 7.5).abs() < 1e-6, "volume {vol}");
    // Undo removes it in one step.
    h.app_mut().active_document_mut().document.undo();
    assert_eq!(h.document().design().timeline().nodes().count(), nodes);
}
