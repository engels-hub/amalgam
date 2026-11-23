//! INS-09: Section Analysis in the real app (`AppHarness`, kittest, CPU scene backend).
//!
//! - The command is in INSPECT ▸ Inspect and S-key search.
//! - The dialog: offset by expression and by the ± buttons, flip, hatch, cap colour, base from
//!   a picked planar face; OK is one undo step; the draft clips the view live.
//! - The browser "Analysis" folder: hide, rename, edit (double-click) and delete, each one undo
//!   step.
//! - Two visible analyses combine (box-corner cut: both planes reach RND-06, caps clipped by
//!   each other); scope and cap colour apply per body; open meshes report open curves.
//! - The Phase 1 script (open STL → XY + 10 mm → area → flip → hatch → save → reopen) on a
//!   synthetic part and on a corpus STL (skipped for an un-fetched LFS pointer unless
//!   `AMG_REQUIRE_TESTDATA=1`).
//! - Timing (asserted only with `AMG_STRICT_TIMING=1`): UI-thread frames while dragging the
//!   offset slider over a 1 M-tri body, and the caps catching up after the drag stops.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stderr,
    clippy::float_cmp,
    missing_docs,
    reason = "test code"
)]

use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_core::{Dir3, Point3, Tolerance, Transform, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, Dependency, Evaluated, ExtKey,
    FeatureBox, FeatureError, FeatureOutput, RecomputeCtx,
};
use amg_features::inspect::doc::{
    CapColor, HatchPattern, OriginPlane, SectionAnalysis, SectionBase, SectionScope, analyses,
    is_kept,
};
use amg_features::inspect::section::{CapOpts, SectionPlanes, section_mesh};
use amg_jobs::JobCtx;
use amg_mesh::{IndexedMesh, MeshBody, Rgba8};
use amg_render::{Camera, Projection};
use amg_ui::CommitBatch;
use amg_ui::command::{CmdId, Placement};
use amg_ui::design::tabs::INSPECT;
use amg_ui::file::FileOp;
use amg_ui::inspect::section::SectionSnapshot;
use amg_ui::inspect::{
    SECTION_CMD, SECTION_JOB, SECTION_STATS_CMD, SectionLayer, section_tool_snapshot,
};
use amg_ui::scene::CpuSceneBackend;
use amg_ui::search::{Recency, SearchIndex};
use amg_ui::testing::egui_kittest::kittest::Queryable;
use amg_ui::testing::{AppHarness, FakeFileDialog};
use egui::accesskit::Role;
use serde::{Deserialize, Serialize};

// ---- fixtures ---------------------------------------------------------------------------------

static MESHES: Mutex<Vec<Arc<IndexedMesh>>> = Mutex::new(Vec::new());

/// A mesh source whose mesh lives in a process-wide table.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct TestMesh {
    key: usize,
}

impl amg_doc::Feature for TestMesh {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let m = Arc::clone(&MESHES.lock().unwrap()[self.key]);
        Ok(FeatureOutput {
            bodies: vec![BodyChange {
                id: ctx.new_body_id(0),
                change: BodyChangeKind::Created {
                    geom: BodyGeom::Mesh(m),
                    naming: None,
                },
            }],
            ..FeatureOutput::default()
        })
    }
}

amg_doc::register_feature!(TestMesh, "uitest.ins09_source", version = 1);

fn link() {
    amg_features::mesh::link();
    amg_features::io::link();
}

fn add_mesh(h: &mut AppHarness, m: IndexedMesh) -> BodyId {
    link();
    let key = {
        let mut t = MESHES.lock().unwrap();
        t.push(Arc::new(m));
        t.len() - 1
    };
    let root = h.document().design().root();
    h.app_mut().submit(CommitBatch {
        label: "Mesh".into(),
        commands: vec![Command::AddFeature {
            at: None,
            feature: FeatureBox::new(TestMesh { key }),
            context: root,
            name: None,
        }],
    });
    h.drive_until_idle();
    newest_mesh_body(h.document().evaluated()).expect("a mesh body")
}

fn newest_mesh_body(ev: &Evaluated) -> Option<BodyId> {
    ev.bodies()
        .filter(|(_, b)| matches!(b.geom, BodyGeom::Mesh(_)))
        .map(|(id, _)| id)
        .max()
}

/// An axis-aligned cube with outward, counter-clockwise triangles.
fn cube_tris(min: Point3, s: f64) -> (Vec<Point3>, Vec<[u32; 3]>) {
    let v = (0..8)
        .map(|i| {
            min + Vec3::new(
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
    (v, t)
}

fn cube(min: Point3, s: f64) -> IndexedMesh {
    let (v, t) = cube_tris(min, s);
    IndexedMesh::new(v, t).unwrap()
}

// ---- helpers ------------------------------------------------------------------------------------

fn layer(h: &AppHarness) -> SectionSnapshot {
    h.app()
        .doc_layer::<SectionLayer>()
        .expect("the section layer of the active document")
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
            "caps never caught up"
        );
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn undo(h: &mut AppHarness) {
    let doc = h.app().active_document().id;
    h.app_mut().request_undo(doc);
    h.drive_until_idle();
}

fn applied(h: &AppHarness) -> usize {
    h.document().history().stack().applied()
}

fn records(h: &AppHarness) -> Vec<(ExtKey, SectionAnalysis)> {
    analyses(h.document().design())
        .into_iter()
        .map(|(k, r)| (k, r.unwrap()))
        .collect()
}

fn add_record(h: &mut AppHarness, a: &SectionAnalysis) -> ExtKey {
    h.app_mut().submit(CommitBatch {
        label: "Section".into(),
        commands: vec![a.add_command()],
    });
    h.drive_until_idle();
    records(h).last().unwrap().0.clone()
}

fn xy(offset: f64) -> SectionAnalysis {
    let mut a = SectionAnalysis::new("Section 1", SectionBase::Origin(OriginPlane::XY));
    a.offset = offset;
    a
}

fn start(h: &mut AppHarness) {
    h.run_command(SECTION_CMD);
    h.kittest().step();
    h.kittest().step();
    assert_eq!(
        h.app().active_tool(),
        Some(SECTION_CMD),
        "Section Analysis started"
    );
}

fn tool(h: &mut AppHarness) -> amg_ui::inspect::section_tool::SectionToolSnapshot {
    h.kittest().step();
    section_tool_snapshot(&h.kittest().ctx).expect("the dialog was drawn")
}

fn click_button(h: &mut AppHarness, label: &str) {
    h.kittest().step();
    h.kittest()
        .get_by_role_and_label(Role::Button, label)
        .click();
    h.kittest().step();
    h.drive_until_idle();
}

fn click_label(h: &mut AppHarness, label: &str) {
    h.kittest().step();
    h.kittest().get_by_label(label).click();
    h.kittest().step();
    h.drive_until_idle();
}

fn has_label(h: &mut AppHarness, label: &str) -> bool {
    h.kittest().step();
    h.kittest().query_all_by_label(label).next().is_some()
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

/// Instance descriptions of `body` in the CPU scene backend.
fn instance_of(h: &AppHarness, body: BodyId) -> amg_ui::scene::InstanceDesc {
    let scene = h.scene();
    let cpu = scene
        .backend()
        .as_any()
        .downcast_ref::<CpuSceneBackend>()
        .unwrap();
    let keys = scene.pick_map().body_keys(body);
    cpu.instances()
        .find(|(_, i)| keys.contains(&i.key))
        .map(|(_, i)| i.clone())
        .expect("instance in the scene")
}

fn look(h: &mut AppHarness, target: Point3, from: Vec3, dist: f64) {
    let dir = from.normalize();
    let up = if dir.z.abs() > 0.9 { Dir3::Y } else { Dir3::Z };
    h.app_mut().active_document_mut().camera = Camera {
        eye: target + dir * dist,
        target,
        up,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    h.drive_until_idle();
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

fn temp_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!(
        "amg-ui-ins09-{tag}-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

// ---- command ------------------------------------------------------------------------------------

#[test]
fn section_analysis_is_in_the_inspect_tab_and_search() {
    let h = AppHarness::new();
    let reg = h.app().registry();
    let def = reg.command(SECTION_CMD).expect("registered");
    assert!(def.placements.iter().any(|p| matches!(
        p,
        Placement::Toolbar { group, pinned: true, .. }
            if reg.group(*group).map(|g| g.tab) == Some(INSPECT)
    )));
    assert!(reg.command(SECTION_STATS_CMD).is_some());
    let index = SearchIndex::build(reg);
    for q in ["Section Analysis", "cross section", "clip"] {
        assert!(
            index
                .search(q, &Recency::new())
                .iter()
                .any(|x| x.id == SECTION_CMD),
            "search {q}"
        );
    }
}

// ---- dialog -------------------------------------------------------------------------------------

/// OK adds one record (one undo step); the draft clips the view while the dialog is open;
/// the caps give the section area; Esc after edits changes nothing; undo removes the record.
#[test]
fn the_dialog_adds_one_record_and_clips_the_view_live() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let body = add_mesh(&mut h, cube(Point3::ZERO, 20.0));
    let n0 = applied(&h);
    start(&mut h);
    let t = tool(&mut h);
    assert_eq!(t.editing, None);
    assert_eq!(t.draft.name, "Section 1");
    assert_eq!(t.draft.base, SectionBase::Origin(OriginPlane::XY));
    assert_eq!(t.offset_text, "0 mm");

    // ± buttons.
    for _ in 0..5 {
        click_button(&mut h, "Offset up");
    }
    click_button(&mut h, "Offset down");
    let t = tool(&mut h);
    assert_eq!(t.draft.offset, 4.0);
    assert_eq!(t.offset_text, "4 mm");
    // The expression field (units and arithmetic).
    type_offset(&mut h, "0.25 in + 3.65 mm");
    let t = tool(&mut h);
    assert!((t.draft.offset - 10.0).abs() < 1e-12, "{}", t.draft.offset);
    // The draft clips the view now, before OK.
    h.drive_until_idle();
    let planes = &h.app().view_sections().planes;
    assert_eq!(planes.len(), 1);
    assert!((planes[0].plane.origin.z - 10.0).abs() < 1e-12);
    assert_eq!(planes[0].plane.normal, Dir3::Z);
    assert!(instance_of(&h, body).sectioned);
    let s = caps_current(&mut h);
    assert_eq!(s.stats.len(), 1);
    assert!((s.stats[0].area - 400.0).abs() < 1e-9, "{:?}", s.stats);
    assert!((s.stats[0].perimeter - 80.0).abs() < 1e-9);
    assert_eq!(s.stats[0].loops, 1);
    assert!(s.outlines_drawn >= 1);
    assert!(has_label(&mut h, "Section area: 400 mm²"));
    assert_eq!(applied(&h), n0, "nothing committed yet");

    // Flip, hatch, uniform cap colour.
    click_button(&mut h, "Flip");
    click_label(&mut h, "Cross");
    click_label(&mut h, "Uniform");
    let t = tool(&mut h);
    assert!(t.draft.flip);
    assert_eq!(t.draft.hatch.pattern, HatchPattern::Cross);
    assert!(matches!(t.draft.cap_color, CapColor::Uniform(_)));
    h.drive_until_idle();
    let sec = h.app().view_sections().clone();
    assert_eq!(sec.planes[0].plane.normal.get(), -Vec3::Z);
    assert!(sec.planes[0].hatch.cross);
    assert!(sec.style.cap_colors.contains_key(&body));
    assert!(instance_of(&h, body).material.cap_color.is_some());

    // OK: one undo step with exactly the draft.
    click_button(&mut h, "OK");
    assert_eq!(h.app().active_tool(), None);
    assert_eq!(applied(&h), n0 + 1);
    let recs = records(&h);
    assert_eq!(recs.len(), 1);
    assert_eq!(recs[0].1, t.draft);
    assert_eq!(h.app().view_sections().planes.len(), 1);

    // A second dialog cancelled with Esc changes nothing and drops its draft.
    start(&mut h);
    click_button(&mut h, "Offset up");
    h.drive_until_idle();
    assert_eq!(
        h.app().view_sections().planes.len(),
        2,
        "draft joins the record"
    );
    h.key(egui::Key::Escape);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None);
    assert_eq!(applied(&h), n0 + 1);
    assert_eq!(h.app().view_sections().planes.len(), 1);

    // Undo removes the record and the clip.
    undo(&mut h);
    assert!(records(&h).is_empty());
    assert!(h.app().view_sections().planes.is_empty());
    assert!(!h.app().view_sections().style.cap_colors.contains_key(&body));
    assert_eq!(instance_of(&h, body).material.cap_color, None);
}

/// "Pick face" + a click on the top face of a cube: the coplanar region's plane is the base.
#[test]
fn a_picked_planar_face_is_the_base() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    start(&mut h);
    look(
        &mut h,
        Point3::new(5.0, 5.0, 10.0),
        Vec3::new(0.3, 0.2, 1.0),
        60.0,
    );
    click_button(&mut h, "Pick face");
    assert!(tool(&mut h).picking);
    let pos = h.world_to_screen(Point3::new(3.0, 6.0, 10.0));
    h.click_at(pos, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.drive_until_idle();
    let t = tool(&mut h);
    assert!(!t.picking && !t.resolving, "{t:?}");
    let SectionBase::Free(p) = t.draft.base else {
        panic!("free base expected: {:?}", t.draft.base)
    };
    assert!((p.origin.z - 10.0).abs() < 1e-9, "{p:?}");
    assert!((p.normal.get() - Vec3::Z).length() < 1e-9, "{p:?}");
    // Offset −3 mm into the cube: the cap is the full 10 × 10 square at z = 7.
    type_offset(&mut h, "-3 mm");
    let s = caps_current(&mut h);
    assert!((s.planes[0].plane.origin.z - 7.0).abs() < 1e-9);
    assert!((s.stats[0].area - 100.0).abs() < 1e-9);
    click_button(&mut h, "OK");
    assert!(matches!(records(&h)[0].1.base, SectionBase::Free(_)));
}

// ---- browser ------------------------------------------------------------------------------------

/// The Analysis folder: hide (eye), rename, edit (double-click → dialog → OK) and delete are
/// each one undo step, and undo restores the previous record.
#[test]
fn browser_hide_rename_edit_delete_are_one_undo_step_each() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    let key = add_record(&mut h, &xy(5.0));
    let get = |h: &AppHarness| {
        h.document()
            .design()
            .ext(&key)
            .map(|r| SectionAnalysis::from_record(r).unwrap())
    };
    assert!(has_label(&mut h, "Analysis"));
    click_button(&mut h, "Expand Analysis");
    assert!(has_label(&mut h, "Section 1"));
    assert_eq!(h.app().view_sections().planes.len(), 1);

    // Hide.
    let n = applied(&h);
    click_button(&mut h, "Hide Section 1");
    assert_eq!(applied(&h), n + 1);
    assert!(!get(&h).unwrap().visible);
    assert!(h.app().view_sections().planes.is_empty());
    assert!(has_label(&mut h, "Show Section 1"));
    undo(&mut h);
    assert!(get(&h).unwrap().visible);
    assert_eq!(h.app().view_sections().planes.len(), 1);

    // Rename (context menu → Rename → type → Enter).
    let n = applied(&h);
    h.kittest().get_by_label("Section 1").click_secondary();
    h.drive_until_idle();
    h.kittest()
        .get_by_role_and_label(Role::Button, "Rename")
        .click();
    h.kittest().run_steps(3);
    let field = h.kittest().get_by_role(Role::TextInput);
    field.type_text("Mid cut");
    h.kittest().run_steps(2);
    h.kittest().key_press(egui::Key::Enter);
    h.drive_until_idle();
    assert_eq!(applied(&h), n + 1);
    assert_eq!(get(&h).unwrap().name, "Mid cut");
    assert!(has_label(&mut h, "Mid cut"));

    // Edit: double-click opens the dialog on the record; OK replaces it (one step).
    let n = applied(&h);
    // kittest runs one frame per queued pointer event and egui's clock advances with every
    // pass, so widen the double-click window (as the UI-07 browser test does).
    h.kittest().run_steps(60);
    h.kittest()
        .ctx
        .options_mut(|o| o.input_options.max_double_click_delay = 2.0);
    let node = h.kittest().get_by_label("Mid cut");
    node.click();
    node.click();
    h.kittest().run_steps(3);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(SECTION_CMD));
    let t = tool(&mut h);
    assert_eq!(t.editing, Some(key.clone()));
    assert_eq!(t.draft.offset, 5.0);
    click_button(&mut h, "Offset up");
    // While editing, the draft replaces the record (still one plane).
    let planes = &h.app().view_sections().planes;
    assert_eq!(planes.len(), 1);
    assert!((planes[0].plane.origin.z - 6.0).abs() < 1e-12);
    click_button(&mut h, "OK");
    assert_eq!(applied(&h), n + 1);
    assert_eq!(get(&h).unwrap().offset, 6.0);
    assert_eq!(get(&h).unwrap().name, "Mid cut");
    undo(&mut h);
    assert_eq!(get(&h).unwrap().offset, 5.0);

    // Delete (context menu).
    let n = applied(&h);
    h.kittest().get_by_label("Mid cut").click_secondary();
    h.drive_until_idle();
    h.kittest()
        .get_by_role_and_label(Role::Button, "Delete")
        .click();
    h.drive_until_idle();
    assert_eq!(applied(&h), n + 1);
    assert!(get(&h).is_none());
    assert!(
        !has_label(&mut h, "Analysis"),
        "an empty folder is not shown"
    );
    assert!(h.app().view_sections().planes.is_empty());
    undo(&mut h);
    assert_eq!(get(&h).unwrap().name, "Mid cut");
    assert!(has_label(&mut h, "Analysis"));
}

// ---- combining, scope, open meshes --------------------------------------------------------------

/// Two visible analyses (XY at z = 5, YZ at x = 5) on a 10 mm cube: both planes go to RND-06
/// (the GPU clip keeps their intersection), and the caps are the two 5 × 10 halves of the
/// box-corner cut, each inside the other plane's kept side. The same planes give the same caps
/// through INS-02 directly. Hiding one leaves one full cap.
#[test]
fn two_visible_analyses_combine_into_a_box_corner_cut() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let body = add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    add_record(&mut h, &xy(5.0));
    let mut b = SectionAnalysis::new("Section 2", SectionBase::Origin(OriginPlane::YZ));
    b.offset = 5.0;
    let kb = add_record(&mut h, &b);
    let s = caps_current(&mut h);
    let planes = h.app().view_sections().planes.clone();
    assert_eq!(planes.len(), 2);
    assert_eq!(planes[0].plane.normal, Dir3::Z);
    assert_eq!(planes[1].plane.normal, Dir3::X);
    assert!(planes.iter().all(|p| p.enabled && p.caps));
    assert_eq!(s.stats.len(), 2);
    for st in &s.stats {
        assert!((st.area - 50.0).abs() < 1e-9, "{st:?}");
    }
    // The combined kept region is the box corner.
    let sp = SectionPlanes(planes.iter().map(|p| p.plane).collect());
    let tol = Tolerance::mesh();
    assert!(is_kept(&sp, Point3::new(2.0, 5.0, 2.0), &tol));
    assert!(!is_kept(&sp, Point3::new(7.0, 5.0, 2.0), &tol));
    assert!(!is_kept(&sp, Point3::new(2.0, 5.0, 7.0), &tol));
    assert!(!is_kept(&sp, Point3::new(7.0, 5.0, 7.0), &tol));
    // INS-02 directly on the same planes: the same areas.
    let m = match &h.document().evaluated().body(body).unwrap().geom {
        BodyGeom::Mesh(m) => Arc::clone(m),
        _ => unreachable!(),
    };
    let direct = section_mesh(
        &MeshBody::from_arc(m),
        &Transform::IDENTITY,
        &sp,
        &CapOpts::default(),
        &JobCtx::detached(),
    )
    .unwrap();
    for cap in &direct.caps {
        for p in &cap.positions {
            assert!(
                is_kept(
                    &sp,
                    *p,
                    &Tolerance {
                        linear: 1e-9,
                        ..tol
                    }
                ),
                "{p:?}"
            );
        }
    }
    assert_eq!(direct.stats[0].area.get(), s.stats[0].area);
    assert_eq!(direct.stats[1].area.get(), s.stats[1].area);
    // Hide the second: one plane, a full cap.
    let mut hidden = b.clone();
    hidden.visible = false;
    h.app_mut().submit(CommitBatch {
        label: "Hide".into(),
        commands: vec![hidden.command(kb)],
    });
    let s = caps_current(&mut h);
    assert_eq!(h.app().view_sections().planes.len(), 1);
    assert!((s.stats[0].area - 100.0).abs() < 1e-9);
}

/// A scope of listed bodies clips only those (others stay whole), with the uniform cap colour.
#[test]
fn scope_and_cap_colour_apply_per_body() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let a = add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    let b = add_mesh(&mut h, cube(Point3::new(20.0, 0.0, 0.0), 10.0));
    let mut s = xy(5.0);
    s.scope = SectionScope::Bodies(vec![a]);
    s.cap_color = CapColor::Uniform(Rgba8([255, 0, 0, 255]));
    add_record(&mut h, &s);
    let snap = caps_current(&mut h);
    assert!(instance_of(&h, a).sectioned);
    assert!(!instance_of(&h, b).sectioned);
    let c = instance_of(&h, a).material.cap_color.unwrap();
    assert!((c[0] - 1.0).abs() < 1e-6 && c[1] == 0.0 && c[2] == 0.0);
    assert_eq!(instance_of(&h, b).material.cap_color, None);
    // Only body a is capped.
    assert!((snap.stats[0].area - 100.0).abs() < 1e-9);
    assert_eq!(snap.stats[0].bodies, 1);
}

/// A cube without its top face: the section is an open curve, no cap, and the statistics
/// panel says so.
#[test]
fn an_open_mesh_reports_open_section_curves() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let (v, mut t) = cube_tris(Point3::ZERO, 10.0);
    t.retain(|tri| *tri != [4, 5, 7] && *tri != [4, 7, 6]);
    add_mesh(&mut h, IndexedMesh::new(v, t).unwrap());
    let mut s = SectionAnalysis::new("Side", SectionBase::Origin(OriginPlane::YZ));
    s.offset = 5.0;
    add_record(&mut h, &s);
    let snap = caps_current(&mut h);
    assert_eq!(snap.stats[0].open_loops, 1);
    assert_eq!(snap.stats[0].area, 0.0);
    h.run_command(SECTION_STATS_CMD);
    h.drive_until_idle();
    assert!(has_label(&mut h, "Side"));
    assert!(has_label_containing(
        &mut h,
        "body is not closed: 1 open section curve"
    ));
}

// ---- Phase 1 script --------------------------------------------------------------------------

/// Imports an STL file as Insert Mesh does (S12 `io.import_mesh`).
fn import_stl(h: &mut AppHarness, bytes: &[u8], name: &str) -> BodyId {
    use amg_features::io::{ImportChoices, ImportSource, import_commands, raw_options};
    use amg_io::{Format, ImportOptions, Source};
    link();
    let ctx = JobCtx::detached();
    let opts = ImportOptions::default();
    let imported = amg_io::import(
        Source::Bytes(bytes),
        Some(Format::Stl),
        &raw_options(&opts),
        &ctx,
    )
    .unwrap();
    let doc: &mut amg_doc::Document = &mut h.app_mut().active_document_mut().document;
    let choices = ImportChoices::from_options(doc.design().root(), &opts, &imported.report);
    let source = ImportSource::from_bytes(name, Format::Stl, imported.report.declared_unit, bytes);
    let plan = import_commands(imported, source, &choices).unwrap();
    plan.apply(doc).unwrap();
    h.drive_until_idle();
    newest_mesh_body(h.document().evaluated()).expect("imported mesh body")
}

fn stl_bytes(m: &IndexedMesh) -> Vec<u8> {
    let mut out = vec![0u8; 80];
    out.extend_from_slice(&u32::try_from(m.face_count()).unwrap().to_le_bytes());
    for t in &m.tris {
        out.extend_from_slice(&[0u8; 12]);
        for &i in t {
            let p = m.positions[i as usize];
            for c in [p.x, p.y, p.z] {
                #[allow(clippy::cast_possible_truncation, reason = "STL is f32")]
                out.extend_from_slice(&(c as f32).to_le_bytes());
            }
        }
        out.extend_from_slice(&[0u8; 2]);
    }
    out
}

/// The scripted part of the owner's Phase 1 check: open STL → Section Analysis at XY + 10 mm →
/// read the area → flip → hatch on → OK → save → close → reopen: the record, the planes and
/// the area are back. Returns the area at z = 10.
fn phase1_script(bytes: &[u8], name: &str) -> f64 {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let body = import_stl(&mut h, bytes, name);
    start(&mut h);
    type_offset(&mut h, "10 mm");
    let s = caps_current(&mut h);
    let area = s.stats[0].area;
    // The same area straight from INS-02.
    let m = match &h.document().evaluated().body(body).unwrap().geom {
        BodyGeom::Mesh(m) => Arc::clone(m),
        _ => unreachable!(),
    };
    let world = h.document().evaluated().instances(body)[0].world;
    let plane = amg_core::Plane {
        origin: Point3::new(0.0, 0.0, 10.0),
        ..amg_core::Plane::XY
    };
    let direct = section_mesh(
        &MeshBody::from_arc(m),
        &world,
        &SectionPlanes(vec![plane]),
        &CapOpts::default(),
        &JobCtx::detached(),
    )
    .unwrap();
    assert_eq!(direct.stats[0].area.get(), area, "{name}");
    let shown = format!("Section area: {} mm²", amg_ui::inspect::props::sig(area, 7));
    assert!(has_label(&mut h, &shown), "{name}: {shown}");
    click_button(&mut h, "Flip");
    click_label(&mut h, "Lines");
    let t = tool(&mut h);
    assert!(t.draft.flip);
    assert_eq!(t.draft.hatch.pattern, HatchPattern::Lines45);
    // Flipping keeps the same section curve, so the same area (other side kept).
    let s = caps_current(&mut h);
    assert!(
        (s.stats[0].area - area).abs() <= 1e-9 * area.max(1.0),
        "{name}"
    );
    click_button(&mut h, "OK");
    let saved = records(&h);
    assert_eq!(saved.len(), 1);
    let planes_before = h.app().view_sections().planes.clone();

    let fake = FakeFileDialog::new();
    h.app_mut().set_file_dialog_backend(fake.clone());
    let dir = temp_dir("script");
    let path = dir.join("section.amg");
    fake.answer(Some(path.clone()));
    h.run_command(CmdId("file.save_as"));
    h.drive_until_idle();
    assert!(path.exists());
    h.run_command(CmdId("file.close"));
    h.drive_until_idle();
    h.app_mut().file_op(None, FileOp::OpenPath(path.clone()));
    h.drive_until_idle();
    assert_eq!(
        records(&h),
        saved,
        "{name}: the record survives save / reopen"
    );
    let s = caps_current(&mut h);
    assert_eq!(h.app().view_sections().planes, planes_before);
    assert!((s.stats[0].area - area).abs() <= 1e-9 * area.max(1.0));
    click_button(&mut h, "Expand Analysis");
    assert!(has_label(&mut h, "Section 1"));
    let _ = std::fs::remove_dir_all(dir);
    area
}

#[test]
fn phase1_script_on_a_synthetic_stl() {
    let area = phase1_script(&stl_bytes(&cube(Point3::ZERO, 20.0)), "cube20.stl");
    assert!((area - 400.0).abs() < 1e-9, "{area}");
}

#[test]
fn phase1_script_on_a_corpus_stl() {
    // A watertight CAD part (thingi10k, CC licence per `testdata`); skipped for an un-fetched
    // LFS pointer unless AMG_REQUIRE_TESTDATA=1 (amg_testkit::testdata).
    let rel = "mesh/real/thingi10k/232912.stl";
    let Some(path) = amg_testkit::testdata(rel) else {
        eprintln!("skipped: {rel} is not fetched");
        return;
    };
    let bytes = std::fs::read(&path).unwrap();
    let area = phase1_script(&bytes, rel);
    eprintln!("INS-09 Phase 1 script on {rel}: section area at z = 10 mm: {area} mm²");
}

// ---- timing ---------------------------------------------------------------------------------

/// Records how long each `amg_ui.frame` span was entered (the UI thread's frame time).
#[derive(Clone, Default)]
struct FrameTimes(Arc<Mutex<Vec<Duration>>>);

struct Entered(Instant);

impl<S> tracing_subscriber::Layer<S> for FrameTimes
where
    S: tracing::Subscriber + for<'a> tracing_subscriber::registry::LookupSpan<'a>,
{
    fn on_enter(&self, id: &tracing::span::Id, ctx: tracing_subscriber::layer::Context<'_, S>) {
        if let Some(span) = ctx.span(id)
            && span.name() == "amg_ui.frame"
        {
            span.extensions_mut().replace(Entered(Instant::now()));
        }
    }

    fn on_exit(&self, id: &tracing::span::Id, ctx: tracing_subscriber::layer::Context<'_, S>) {
        if let Some(span) = ctx.span(id)
            && let Some(Entered(t)) = span.extensions().get::<Entered>()
        {
            self.0.lock().unwrap().push(t.elapsed());
        }
    }
}

/// Dragging the offset slider over a 1 M-tri closed body: every frame moves the plane (the
/// GPU clip's input) and supersedes the caps job; the UI thread's frames stay short, and the
/// caps of the final position arrive soon after the drag stops. Measured on the CPU side
/// (kittest has no GPU); asserted (< 16 ms per frame, ≤ 100 ms catch-up) only under
/// `AMG_STRICT_TIMING=1`.
#[test]
fn timing_slider_drag_on_a_1m_tri_body() {
    use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
    use tracing_subscriber::layer::SubscriberExt as _;

    let mut h = AppHarness::new();
    h.drive_until_idle();
    let body = add_mesh(&mut h, benchmark_sphere(BenchSize::M1, 1));
    let faces = match &h.document().evaluated().body(body).unwrap().geom {
        BodyGeom::Mesh(m) => m.face_count(),
        _ => unreachable!(),
    };
    start(&mut h);
    // First caps (warms the closed-and-oriented flag, cached per mesh).
    let first = caps_current(&mut h);
    assert!(first.stats[0].area > 0.0);
    let slider = h
        .kittest()
        .get_by_role_and_label(Role::Slider, "Offset slider")
        .rect();
    let from = egui::pos2(slider.left() + slider.width() * 0.3, slider.center().y);
    let to = egui::pos2(slider.left() + slider.width() * 0.7, slider.center().y);
    let before = layer(&h);

    let times = FrameTimes::default();
    let subscriber = tracing_subscriber::registry().with(times.clone());
    let guard = tracing::subscriber::set_default(subscriber);
    let steps = 120;
    h.drag(
        egui::PointerButton::Primary,
        egui::Modifiers::NONE,
        from,
        to,
        steps,
    );
    let released = Instant::now();
    let s = caps_current(&mut h);
    let waited = released.elapsed();
    drop(guard);

    let after = layer(&h);
    let moved = tool(&mut h).draft.offset;
    let jobs = after.jobs_started - before.jobs_started;
    let superseded = after.superseded.len() - before.superseded.len();
    let catch_up = *after.catch_up.last().unwrap();
    let mut t = times.0.lock().unwrap().clone();
    t.sort();
    let max = *t.last().unwrap();
    let p50 = t[t.len() / 2];
    let p99 = t[t.len() * 99 / 100];
    let over = t.iter().filter(|d| **d > Duration::from_millis(16)).count();
    eprintln!(
        "INS-09 slider drag on {faces} faces: {} frames, p50 {p50:?}, p99 {p99:?}, max {max:?}, \
         {over} over 16 ms; {jobs} '{SECTION_JOB}' jobs, {superseded} superseded; final offset \
         {moved:.3} mm, area {:.3} mm²; caps caught up {catch_up:?} after the last change \
         ({waited:?} after release incl. polling)",
        t.len(),
        s.stats[0].area
    );
    assert!(jobs >= 2, "the drag moved the plane: {jobs} jobs");
    assert!(superseded >= 1, "newer positions supersede older caps jobs");
    if strict_timing() {
        assert!(max < Duration::from_millis(16), "max frame {max:?}");
        assert!(
            catch_up <= Duration::from_millis(100),
            "catch-up {catch_up:?}"
        );
    }
}
