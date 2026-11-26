//! INS-11: Section Analysis on solids and reference bases in the real app (`AppHarness`,
//! kittest, CPU scene backend).
//!
//! - The offset arrow (UI-06) and the offset field follow each other; `Ctrl` snaps, `Esc`
//!   restores, OK commits what was dragged.
//! - A construction-plane base follows a parameter edit; deleting the plane keeps the stored
//!   plane with a warning.
//! - With the `occt` feature: caps of solid bodies, a picked solid face as the base (also in a
//!   placed component), which follows its face and, once the face is deleted, keeps the last
//!   plane with a warning.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "test code"
)]

use std::time::{Duration, Instant};

use amg_core::{Dir3, Point3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, Command, ComponentId, ConstructionId, CreatedId,
    Dependency, Document, EntityRef, Expr, ExprSrc, Feature, FeatureBox, FeatureError, FeatureId,
    FeatureOutput, RecomputeCtx,
};
use amg_features::construct::{ConstructPlane, OriginEntity, PlaneDef, RefInput};
use amg_features::inspect::doc::{SectionAnalysis, SectionBase, analyses};
use amg_jobs::{JobSystem, JobSystemConfig};
use amg_mesh::IndexedMesh;
use amg_render::{Camera, Projection};
use amg_ui::CommitBatch;
use amg_ui::inspect::section::SectionSnapshot;
use amg_ui::inspect::section_tool::SectionToolSnapshot;
use amg_ui::inspect::{SECTION_CMD, SECTION_STATS_CMD, SectionLayer, section_tool_snapshot};
use amg_ui::manip;
use amg_ui::pick::PickTarget;
use amg_ui::select::SelItem;
#[cfg(feature = "occt")]
use amg_ui::select::SelectPriority;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable;
use egui::accesskit::Role;
use serde::{Deserialize, Serialize};

// ---- fixtures ---------------------------------------------------------------------------------

/// A mesh cube `[0, size]³`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Cube {
    size: u32,
}

impl Feature for Cube {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let s = f64::from(self.size);
        let v = (0..8)
            .map(|i| {
                Point3::new(
                    f64::from(i & 1),
                    f64::from((i >> 1) & 1),
                    f64::from((i >> 2) & 1),
                ) * s
            })
            .collect();
        let t = vec![
            [0, 2, 3],
            [0, 3, 1],
            [4, 5, 7],
            [4, 7, 6],
            [0, 1, 5],
            [0, 5, 4],
            [2, 6, 7],
            [2, 7, 3],
            [0, 4, 6],
            [0, 6, 2],
            [1, 3, 7],
            [1, 7, 5],
        ];
        let mesh = IndexedMesh::new(v, t).expect("a cube");
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(std::sync::Arc::new(mesh)),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(Cube, "uitest.ins11_cube", version = 1);

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

fn add_in(doc: &mut Document, f: FeatureBox, context: ComponentId) -> FeatureId {
    let out = doc
        .apply(Command::AddFeature {
            at: None,
            feature: f,
            context,
            name: None,
        })
        .unwrap();
    doc.recompute_blocking(&jobs()).unwrap();
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
    add_in(doc, f, root)
}

/// The app on `doc`.
fn harness(doc: Document) -> AppHarness {
    amg_features::construct::link();
    amg_features::sketch::link();
    let mut h = AppHarness::builder()
        .size(egui::vec2(1600.0, 1000.0))
        .build();
    let id = h.app_mut().add_document("ins11".into(), doc);
    h.app_mut().activate(id);
    h.drive_until_idle();
    h
}

fn look(h: &mut AppHarness, eye: Point3, target: Point3, up: Dir3) {
    h.app_mut().active_document_mut().camera = Camera {
        eye,
        target,
        up,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    h.drive_until_idle();
}

/// Looks down the Z axis at (x, y) from `height`.
fn look_down(h: &mut AppHarness, x: f64, y: f64, height: f64) {
    look(
        h,
        Point3::new(x, y, height),
        Point3::new(x, y, 0.0),
        Dir3::Y,
    );
}

// ---- helpers ------------------------------------------------------------------------------------

fn tool(h: &mut AppHarness) -> SectionToolSnapshot {
    h.kittest().step();
    section_tool_snapshot(&h.kittest().ctx).expect("the dialog was drawn")
}

fn layer(h: &AppHarness) -> SectionSnapshot {
    h.app()
        .doc_layer::<SectionLayer>()
        .expect("the section layer")
        .snapshot()
}

/// Steps until the caps match the planes and no caps job runs.
fn caps_current(h: &mut AppHarness) -> SectionSnapshot {
    let t0 = Instant::now();
    loop {
        h.drive_until_idle();
        let s = layer(h);
        if s.current && s.running.is_none() {
            assert!(s.error.is_none(), "{:?}", s.error);
            return s;
        }
        assert!(
            t0.elapsed() < Duration::from_secs(120),
            "caps never caught up: {s:?}"
        );
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn start(h: &mut AppHarness) {
    h.run_command(SECTION_CMD);
    h.kittest().step();
    h.kittest().step();
    assert_eq!(h.app().active_tool(), Some(SECTION_CMD));
}

fn click_button(h: &mut AppHarness, label: &str) {
    h.kittest().step();
    h.kittest()
        .get_by_role_and_label(Role::Button, label)
        .click();
    h.kittest().step();
    h.drive_until_idle();
}

fn has_label_containing(h: &mut AppHarness, text: &str) -> bool {
    h.kittest().step();
    h.kittest()
        .query_all_by_label_contains(text)
        .next()
        .is_some()
}

/// Replaces the text of the Offset expression field and commits it with Enter.
fn type_offset(h: &mut AppHarness, text: &str) {
    h.kittest().step();
    h.kittest()
        .get_by_role_and_label(Role::TextInput, "Offset")
        .click();
    h.kittest().run_steps(2);
    let old = tool(h).offset_text;
    for _ in 0..old.chars().count() + 2 {
        h.kittest().key_press(egui::Key::Backspace);
    }
    h.kittest().run_steps(1);
    h.kittest()
        .get_by_role_and_label(Role::TextInput, "Offset")
        .type_text(text);
    h.kittest().run_steps(2);
    h.kittest().key_press(egui::Key::Enter);
    h.kittest().run_steps(2);
}

fn submit(h: &mut AppHarness, command: Command) {
    h.app_mut().submit(CommitBatch {
        label: "Test".into(),
        commands: vec![command],
    });
    h.drive_until_idle();
}

fn applied(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

fn record(h: &AppHarness) -> SectionAnalysis {
    analyses(h.document().design())
        .into_iter()
        .next()
        .expect("one record")
        .1
        .unwrap()
}

fn plane_z(h: &AppHarness) -> f64 {
    h.app().view_sections().planes[0].plane.origin.z
}

// ---- manipulator --------------------------------------------------------------------------------

/// The screen points of the arrow's middle and of the same spot `mm` further along it.
fn arrow_drag(h: &mut AppHarness, mm: f64) -> (egui::Pos2, egui::Pos2) {
    let (origin, dir) = tool(h).gizmo.expect("the offset arrow");
    let cam = h.camera_view();
    let grab = origin + dir.get() * (0.6 * manip::world_size(&cam, origin));
    (
        h.world_to_screen(grab),
        h.world_to_screen(grab + dir.get() * mm),
    )
}

fn drag_arrow(h: &mut AppHarness, mm: f64, mods: egui::Modifiers) {
    let (from, to) = arrow_drag(h, mm);
    h.drag(egui::PointerButton::Primary, mods, from, to, 6);
    h.drive_until_idle();
}

/// The draft offset, its field text, the clip plane and the arrow all show `offset`.
#[track_caller]
fn shows_offset(h: &mut AppHarness, offset: f64, tolerance: f64) -> f64 {
    let t = tool(h);
    assert!(
        (t.draft.offset - offset).abs() <= tolerance,
        "offset {} instead of {offset}",
        t.draft.offset
    );
    let number: f64 = t
        .offset_text
        .strip_suffix(" mm")
        .unwrap_or_else(|| panic!("{}", t.offset_text))
        .parse()
        .unwrap();
    assert_eq!(number, t.draft.offset, "the field shows the offset");
    h.drive_until_idle();
    assert_eq!(plane_z(h), t.draft.offset, "the view is cut there");
    let (origin, dir) = tool(h).gizmo.expect("the offset arrow");
    assert_eq!(dir, Dir3::Z);
    assert!((origin.z - t.draft.offset).abs() < 1e-12, "{origin:?}");
    t.draft.offset
}

#[test]
fn the_offset_arrow_and_the_offset_field_follow_each_other() {
    let mut doc = Document::new("manip");
    add(&mut doc, FeatureBox::new(Cube { size: 20 }));
    let mut h = harness(doc);
    look(
        &mut h,
        Point3::new(10.0, -110.0, 25.0),
        Point3::new(10.0, 10.0, 10.0),
        Dir3::Z,
    );
    let n0 = applied(&h);
    start(&mut h);
    // The arrow sits on the section plane, over the middle of the body, along the base normal.
    let (origin, dir) = tool(&mut h).gizmo.expect("the offset arrow");
    assert!(
        origin.distance(Point3::new(10.0, 10.0, 0.0)) < 1e-9,
        "{origin:?}"
    );
    assert_eq!(dir, Dir3::Z);

    // Field → arrow.
    type_offset(&mut h, "7 mm");
    shows_offset(&mut h, 7.0, 0.0);
    click_button(&mut h, "Offset up");
    shows_offset(&mut h, 8.0, 0.0);

    // Arrow → field.
    drag_arrow(&mut h, 5.0, egui::Modifiers::NONE);
    shows_offset(&mut h, 13.0, 0.05);
    drag_arrow(&mut h, -9.3, egui::Modifiers::NONE);
    let free = shows_offset(&mut h, 3.7, 0.1);
    // Ctrl snaps to 1 mm.
    drag_arrow(&mut h, 8.2, egui::Modifiers::CTRL);
    shows_offset(&mut h, (free + 8.2).round(), 0.0);
    let s = caps_current(&mut h);
    assert!((s.stats[0].area - 400.0).abs() < 1e-9, "{:?}", s.stats);

    // Esc during a drag restores the field and keeps the tool.
    type_offset(&mut h, "2 mm + 3 mm");
    let (from, to) = arrow_drag(&mut h, 6.0);
    let primary = egui::PointerButton::Primary;
    h.hover_at(from);
    h.button(from, primary, true, egui::Modifiers::NONE);
    h.hover_at(from.lerp(to, 0.5));
    h.hover_at(to);
    assert!((tool(&mut h).draft.offset - 11.0).abs() < 0.05);
    h.key(egui::Key::Escape);
    h.button(to, primary, false, egui::Modifiers::NONE);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(SECTION_CMD));
    let t = tool(&mut h);
    assert_eq!(
        (t.draft.offset, t.offset_text.as_str()),
        (5.0, "2 mm + 3 mm")
    );

    // A flip keeps the arrow along the base normal; with a tilt it stays on the plane.
    click_button(&mut h, "Flip");
    let (_, dir) = tool(&mut h).gizmo.expect("the offset arrow");
    assert_eq!(dir, Dir3::Z);
    click_button(&mut h, "Flip");

    // No arrow while a base is being picked.
    click_button(&mut h, "Pick face");
    assert_eq!(tool(&mut h).gizmo, None);
    click_button(&mut h, "Pick face");

    drag_arrow(&mut h, 4.0, egui::Modifiers::NONE);
    let dragged = shows_offset(&mut h, 9.0, 0.05);
    assert_eq!(applied(&h), n0, "dragging commits nothing");
    click_button(&mut h, "OK");
    assert_eq!(applied(&h), n0 + 1);
    assert_eq!(record(&h).offset, dragged);

    // Editing the record puts the arrow at its offset again.
    let key = analyses(h.document().design())[0].0.clone();
    h.app_mut()
        .with_cx(|cx| amg_ui::inspect::section::edit(cx, key));
    h.drive_until_idle();
    assert!(tool(&mut h).editing.is_some());
    shows_offset(&mut h, dragged, 0.0);
}

// ---- construction-plane bases --------------------------------------------------------------------

#[test]
fn a_construction_base_follows_a_parameter_edit_and_warns_when_deleted() {
    amg_features::construct::link();
    let mut doc = Document::new("param");
    let out = doc
        .apply(Command::AddParam {
            name: "height".into(),
            expr: "25 mm".into(),
            unit: None,
            comment: String::new(),
        })
        .unwrap();
    let param = out
        .created
        .iter()
        .find_map(|c| match c {
            CreatedId::Param(p) => Some(*p),
            _ => None,
        })
        .expect("a parameter");
    add(&mut doc, FeatureBox::new(Cube { size: 20 }));
    let plane = add(
        &mut doc,
        FeatureBox::new(ConstructPlane {
            def: PlaneDef::Offset {
                base: RefInput::Origin(OriginEntity::XYPlane),
                distance: Expr::Inline(ExprSrc("height".to_owned())),
            },
        }),
    );
    let mut h = harness(doc);
    look_down(&mut h, 10.0, 10.0, 200.0);
    let c = ConstructionId(plane);
    h.app_mut()
        .active_document_mut()
        .selection
        .set(vec![SelItem {
            target: PickTarget::Construction(c),
            entity: EntityRef::Construction(c),
        }]);
    h.drive_until_idle();
    start(&mut h);
    let (origin, _) = tool(&mut h)
        .gizmo
        .expect("an arrow on the referenced plane");
    assert!((origin.z - 25.0).abs() < 1e-12);
    click_button(&mut h, "OK");
    assert!(matches!(record(&h).base, SectionBase::Ref { .. }));
    assert_eq!(plane_z(&h), 25.0);

    submit(
        &mut h,
        Command::SetParamExpr {
            id: param,
            expr: "12 mm".into(),
        },
    );
    assert_eq!(plane_z(&h), 12.0);
    let s = caps_current(&mut h);
    assert!((s.stats[0].area - 400.0).abs() < 1e-9, "{s:?}");
    assert!(s.warnings.is_empty());

    // Deleted: the stored plane (CON-04), with a warning in the panel.
    submit(&mut h, Command::DeleteFeatures { ids: vec![plane] });
    assert_eq!(plane_z(&h), 25.0);
    let s = layer(&h);
    assert_eq!(s.warnings.len(), 1, "{s:?}");
    assert!(s.warnings[0].fallback);
    assert!(s.warnings[0].message.contains("construction plane is gone"));
    h.run_command(SECTION_STATS_CMD);
    assert!(has_label_containing(&mut h, "construction plane is gone"));
}

// ---- solids ---------------------------------------------------------------------------------

#[cfg(feature = "occt")]
mod solid {
    use amg_core::{Point2, Transform, Vec3};
    use amg_doc::stub::ProfileKey;
    use amg_doc::{BodyId, OccurrenceId, SketchId, TopoKind, naming::ProfileFp};
    use amg_features::sketch::{PlaneInput, SketchFeature};
    use amg_features::solid::{BodyOp, Extrude, OpTarget};
    use amg_sketch::build;

    use super::*;

    /// A sketch on XY with the rectangle `r` and its extrude, both in `context`.
    fn extruded(
        doc: &mut Document,
        context: ComponentId,
        r: [f64; 4],
        height: &str,
        join: Option<BodyId>,
    ) -> (FeatureId, FeatureId) {
        let mut f = SketchFeature::new(PlaneInput::OriginXY);
        f.edit_sketch(|s| {
            build::rectangle_2pt(s, Point2::new(r[0], r[1]), Point2::new(r[2], r[3]))
        })
        .unwrap();
        let sketch = add_in(doc, FeatureBox::new(f), context);
        let feature = extrude_of(doc, sketch, height, join);
        (sketch, add_in(doc, feature, context))
    }

    fn extrude_of(
        doc: &Document,
        sketch: FeatureId,
        height: &str,
        join: Option<BodyId>,
    ) -> FeatureBox {
        let g = doc.evaluated().sketch(SketchId(sketch)).unwrap();
        let profiles = g
            .profiles
            .profiles
            .iter()
            .map(|p| EntityRef::Profile {
                sketch: SketchId(sketch),
                profile: ProfileKey(p.id.0),
                fingerprint: Some(ProfileFp(p.fingerprint.clone())),
            })
            .collect();
        let target = OpTarget {
            op: if join.is_some() {
                BodyOp::Join
            } else {
                BodyOp::NewBody
            },
            participants: join.into_iter().collect(),
        };
        FeatureBox::new(Extrude::distance(profiles, height, target))
    }

    /// A 40 × 30 × 10 slab with a 10 × 12 boss reaching z = 16: the boss feature and its sketch.
    fn slab_with_boss(doc: &mut Document) -> (FeatureId, FeatureId) {
        let root = doc.design().root();
        let (_, slab) = extruded(doc, root, [0.0, 0.0, 40.0, 30.0], "10 mm", None);
        let body = BodyId {
            origin: slab,
            slot: 0,
        };
        extruded(doc, root, [5.0, 5.0, 15.0, 17.0], "16 mm", Some(body))
    }

    /// Picks the face at `at` as the base and waits for it.
    fn pick_base(h: &mut AppHarness, at: Point3) -> SectionToolSnapshot {
        click_button(h, "Pick face");
        h.click_world(at);
        h.drive_until_idle();
        let t = tool(h);
        assert!(!t.picking && !t.resolving, "{t:?}");
        assert_eq!(t.error, None);
        t
    }

    #[test]
    fn a_solid_face_base_follows_its_face_and_keeps_the_last_plane_when_deleted() {
        let mut doc = Document::new("solid");
        amg_features::sketch::link();
        let (sketch, boss) = slab_with_boss(&mut doc);
        let slab = BodyId {
            origin: doc.design().timeline().nodes().nth(1).unwrap().id,
            slot: 0,
        };
        let mut h = harness(doc);
        look_down(&mut h, 20.0, 15.0, 120.0);
        // Solid bodies alone enable the tool, and are capped on their tessellation.
        start(&mut h);
        type_offset(&mut h, "4 mm");
        let s = caps_current(&mut h);
        assert!((s.stats[0].area - 1200.0).abs() < 1e-9, "{s:?}");
        assert_eq!((s.stats[0].bodies, s.stats[0].loops), (1, 1));
        assert!(s.outlines_drawn >= 1);
        type_offset(&mut h, "0 mm");

        // The boss top as the base.
        let t = pick_base(&mut h, Point3::new(11.0, 13.0, 16.0));
        let SectionBase::Ref { entity, last } = &t.draft.base else {
            panic!("a reference base: {:?}", t.draft.base);
        };
        assert!(
            matches!(entity, EntityRef::Topo(r) if r.kind == TopoKind::Face),
            "{entity:?}"
        );
        assert!((last.origin.z - 16.0).abs() < 1e-9);
        assert!((last.normal.get() - Vec3::Z).length() < 1e-12);
        let (origin, _) = t.gizmo.expect("an arrow on the face plane");
        assert!((origin.z - 16.0).abs() < 1e-9);
        type_offset(&mut h, "-2 mm");
        let s = caps_current(&mut h);
        assert!((s.planes[0].plane.origin.z - 14.0).abs() < 1e-9);
        assert!((s.stats[0].area - 120.0).abs() < 1e-9, "{s:?}");
        // The arrow drags a face-based section too (seen from the side: from above it points
        // at the viewer).
        look(
            &mut h,
            Point3::new(20.0, -120.0, 40.0),
            Point3::new(20.0, 15.0, 10.0),
            Dir3::Z,
        );
        drag_arrow(&mut h, -1.0, egui::Modifiers::CTRL);
        assert_eq!(tool(&mut h).draft.offset, -3.0);
        type_offset(&mut h, "-2 mm");
        click_button(&mut h, "OK");
        let stored = record(&h);
        let n = applied(&h);

        // The section follows the face.
        let higher = extrude_of(h.document(), sketch, "22.5 mm", Some(slab));
        submit(
            &mut h,
            Command::EditFeature {
                id: boss,
                feature: higher,
            },
        );
        let s = caps_current(&mut h);
        assert!((plane_z(&h) - 20.5).abs() < 1e-9, "{}", plane_z(&h));
        assert!((s.stats[0].area - 120.0).abs() < 1e-9);
        assert!(s.warnings.is_empty());
        assert_eq!(record(&h), stored, "following a face edits no record");

        // Deleting the boss deletes the face: the last plane stays, with a warning.
        // (A warning toast hides itself after a few seconds, so it is caught frame by frame.)
        h.app_mut().submit(CommitBatch {
            label: "Delete".into(),
            commands: vec![Command::DeleteFeatures { ids: vec![boss] }],
        });
        let t0 = Instant::now();
        let toasts = loop {
            h.kittest().step();
            let toasts = h
                .app()
                .notifications()
                .items()
                .iter()
                .filter(|t| t.notification.body.contains("base face is gone"))
                .count();
            if toasts > 0 {
                break toasts;
            }
            assert!(t0.elapsed() < Duration::from_secs(60), "no toast");
            std::thread::sleep(Duration::from_millis(2));
        };
        assert_eq!(toasts, 1, "one toast per new warning");
        let s = caps_current(&mut h);
        assert!((plane_z(&h) - 20.5).abs() < 1e-9, "{}", plane_z(&h));
        assert_eq!(s.stats[0].area, 0.0, "the plane is above the slab");
        assert_eq!(s.warnings.len(), 1, "{s:?}");
        assert!(s.warnings[0].fallback);
        assert!(s.warnings[0].message.contains("base face is gone"));
        assert_eq!(applied(&h), n + 2, "the warning is not an edit");
        h.run_command(SECTION_STATS_CMD);
        assert!(has_label_containing(&mut h, "Warning, Section 1"));

        // The dialog shows it, and OK stores the kept plane.
        let key = analyses(h.document().design())[0].0.clone();
        let app = h.app_mut();
        app.with_cx(|cx| amg_ui::inspect::section::edit(cx, key));
        h.drive_until_idle();
        let t = tool(&mut h);
        assert!(
            t.warning.as_deref().is_some_and(|w| w.contains("gone")),
            "{t:?}"
        );
        click_button(&mut h, "OK");
        let SectionBase::Ref { last, .. } = record(&h).base else {
            panic!("still a reference");
        };
        assert!((last.origin.z - 22.5).abs() < 1e-9, "{last:?}");

        // Undoing back to the boss clears the warning and follows the face again.
        let id = h.app().active_document().id;
        h.app_mut().request_undo(id);
        h.drive_until_idle();
        h.app_mut().request_undo(id);
        h.drive_until_idle();
        let s = caps_current(&mut h);
        assert!(s.warnings.is_empty(), "{s:?}");
        assert!((plane_z(&h) - 20.5).abs() < 1e-9);
    }

    #[test]
    fn a_face_in_a_placed_component_is_cut_where_it_is_seen() {
        let mut doc = Document::new("placed");
        amg_features::sketch::link();
        let root = doc.design().root();
        let out = doc
            .apply(Command::NewComponent {
                parent: root,
                name: "Part".into(),
                transform: Transform::from_translation(Vec3::new(50.0, 0.0, 30.0)),
            })
            .unwrap();
        let component = out
            .created
            .iter()
            .find_map(|c| match c {
                CreatedId::Component(c) => Some(*c),
                _ => None,
            })
            .expect("a component");
        let (_, extrude) = extruded(&mut doc, component, [0.0, 0.0, 10.0, 8.0], "6 mm", None);
        let body = BodyId {
            origin: extrude,
            slot: 0,
        };
        let path: Vec<OccurrenceId> = doc.design().occurrence_paths(component)[0].to_vec();
        let mut h = harness(doc);
        look_down(&mut h, 55.0, 4.0, 120.0);
        // The preselected top face starts the section on it.
        h.app_mut()
            .active_document_mut()
            .selection
            .set_priority(SelectPriority::Face);
        let at = Point3::new(54.0, 3.0, 36.0);
        h.hover_at(h.world_to_screen(at));
        h.drive_until_idle();
        h.click_world(at);
        h.drive_until_idle();
        let selected = h.app().active_document().selection.items().to_vec();
        assert!(
            matches!(
                selected.as_slice(),
                [SelItem {
                    target: PickTarget::Body {
                        body: b,
                        sub: Some((TopoKind::Face, _)),
                        ..
                    },
                    ..
                }] if *b == body
            ),
            "{selected:?}"
        );
        start(&mut h);
        h.drive_until_idle();
        let t = tool(&mut h);
        let SectionBase::Ref { entity, last } = &t.draft.base else {
            panic!("a reference base: {t:?}");
        };
        assert!(
            matches!(entity, EntityRef::InContext { path: p, .. } if *p == path),
            "{entity:?}"
        );
        assert!((last.origin.z - 36.0).abs() < 1e-9, "{last:?}");
        type_offset(&mut h, "-1 mm");
        let s = caps_current(&mut h);
        assert!((s.stats[0].area - 80.0).abs() < 1e-9, "{s:?}");
        assert!(s.warnings.is_empty());
        let outline = s.stats[0].extent;
        assert!((outline.size().x * outline.size().y - 80.0).abs() < 1e-9);
    }
}
