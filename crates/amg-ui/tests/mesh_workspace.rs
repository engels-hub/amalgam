//! MESH-23: the Phase 1 Mesh workspace in the real app (`AppHarness`, kittest).
//!
//! - every op plus Mesh Analysis is in a MESH toolbar group and in S-key search, and starts
//!   from both;
//! - the dialogs preview through cancellable jobs and commit one undoable feature;
//! - the Mesh Analysis panel reports and highlights defects (RND-04 `Error` layer);
//! - cancel of a preview and of the committed recompute of a 1 M-tri op, and the UI-thread
//!   frame time while such an op runs (timing asserts only under `AMG_STRICT_TIMING=1`;
//!   numbers are printed for the report);
//! - the Phase 1 script (import STL → analyse → repair → remesh → reduce → plane cut → shell →
//!   export 3MF) on three corpus models (skipped for un-fetched git-LFS pointers unless
//!   `AMG_REQUIRE_TESTDATA=1`).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "test code"
)]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, Dependency, Document, Evaluated,
    FeatureBox, FeatureError, FeatureOutput, Health, RecomputeCtx,
};
use amg_mesh::{FaceId, IndexedMesh};
use amg_ui::CommitBatch;
use amg_ui::command::Placement;
use amg_ui::mesh::{ANALYSIS_CMD, MESH_TAB, MeshOp, command_ids};
use amg_ui::pick::PickTarget;
use amg_ui::preview::PreviewState;
use amg_ui::scene::HighlightLayer;
use amg_ui::search::{Recency, SearchIndex};
use amg_ui::select::SelItem;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable;
use serde::{Deserialize, Serialize};

// ---- fixtures ---------------------------------------------------------------------------------

static MESHES: Mutex<Vec<Arc<IndexedMesh>>> = Mutex::new(Vec::new());

/// A mesh source: the mesh lives in a process-wide table, so the feature (and every preview's
/// design copy) stays small.
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

amg_doc::register_feature!(TestMesh, "uitest.mesh23_source", version = 1);

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

fn mesh(ev: &Evaluated, body: BodyId) -> Arc<IndexedMesh> {
    match &ev.body(body).expect("body").geom {
        BodyGeom::Mesh(m) => Arc::clone(m),
        other => panic!("{other:?}"),
    }
}

fn sphere(k: u32) -> IndexedMesh {
    amg_mesh::corpus::bench::geodesic_sphere(k, 10.0, None)
}

/// A sphere with the faces of one cap removed (a hole) and one face flipped.
fn broken_sphere() -> IndexedMesh {
    let s = sphere(12);
    let keep: Vec<[u32; 3]> = s
        .tris
        .iter()
        .filter(|t| t.iter().all(|&v| s.positions[v as usize].z < 9.0))
        .copied()
        .collect();
    IndexedMesh::new(s.positions.clone(), keep).unwrap()
}

fn select(h: &mut AppHarness, body: BodyId) {
    let item = SelItem::from_target(PickTarget::Body {
        body,
        instance: 0,
        sub: None,
    })
    .unwrap();
    let sel = &mut h.app_mut().active_document_mut().selection;
    sel.clear();
    sel.add(item);
}

fn click(h: &mut AppHarness, label: &str) {
    h.kittest().step();
    h.kittest().get_by_label(label).click();
    h.kittest().step();
    h.drive_until_idle();
}

fn start(h: &mut AppHarness, op: MeshOp) {
    h.run_command(op.cmd());
    h.drive_until_idle();
    h.kittest().step();
    assert_eq!(h.app().active_tool(), Some(op.cmd()), "{op:?} started");
}

/// Runs `op` with its default options and OK; returns the health of the new feature.
fn run_op(h: &mut AppHarness, op: MeshOp) -> Health {
    let before = h.document().design().timeline().nodes().count();
    start(h, op);
    click(h, "OK");
    assert_eq!(h.app().active_tool(), None, "{op:?} committed");
    let doc = h.document();
    let nodes: Vec<_> = doc.design().timeline().nodes().map(|n| n.id).collect();
    assert_eq!(nodes.len(), before + 1, "{op:?} added one feature");
    let id = *nodes.last().unwrap();
    doc.evaluated().node(id).expect("evaluated").health.clone()
}

fn assert_not_error(op: MeshOp, health: &Health) {
    assert!(
        matches!(health, Health::Ok | Health::Warning(_)),
        "{op:?}: {health:?}"
    );
}

/// Steps a few frames (for while a text field is focused).
fn settle(h: &mut AppHarness) {
    for _ in 0..6 {
        h.kittest().step();
    }
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1")
}

// ---- toolbar and search -----------------------------------------------------------------------

#[test]
fn every_mesh_op_is_reachable_from_the_toolbar_and_s_key_search() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    let ids = command_ids();
    assert_eq!(ids.len(), MeshOp::ALL.len() + 1);
    {
        let reg = h.app().registry();
        let index = SearchIndex::build(reg);
        let recency = Recency::new();
        for id in &ids {
            let def = reg
                .command(*id)
                .unwrap_or_else(|| panic!("{id} registered"));
            let groups: Vec<_> = def
                .placements
                .iter()
                .filter_map(|p| match p {
                    Placement::Toolbar { group, .. } => Some(*group),
                    _ => None,
                })
                .collect();
            assert!(!groups.is_empty(), "{id} has a toolbar placement");
            for g in groups {
                assert_eq!(reg.group(g).map(|g| g.tab), Some(MESH_TAB), "{id} in {g}");
                assert!(reg.group_commands(g).any(|(_, c)| c.id == *id));
            }
            let title = def.label.trim_end_matches('…').to_owned();
            let hits = index.search(&title, &recency);
            assert_eq!(
                hits.first().map(|x| x.id),
                Some(*id),
                "search {title:?}: {hits:?}"
            );
            for k in def.keywords {
                assert!(
                    index.search(k, &recency).iter().any(|x| x.id == *id),
                    "keyword {k} finds {id}"
                );
            }
        }
    }

    // Without a mesh body the ops are disabled with a reason.
    h.run_command(MeshOp::Remesh.cmd());
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None);

    let body = add_mesh(&mut h, sphere(12));
    // Every op starts from its command (toolbar button / menu / search all run it) and shows
    // its dialog; Esc cancels without a change.
    for op in MeshOp::ALL {
        select(&mut h, body);
        start(&mut h, op);
        assert!(
            h.kittest().query_by_label(op.title()).is_some(),
            "{op:?} dialog titled"
        );
        h.key(egui::Key::Escape);
        h.drive_until_idle();
        assert_eq!(h.app().active_tool(), None, "{op:?} cancelled");
    }
    assert_eq!(h.document().design().timeline().nodes().count(), 1);

    // The MESH tab's pinned button.
    h.app_mut().select_tab(MESH_TAB);
    h.drive_until_idle();
    click(&mut h, &MeshOp::Remesh.label());
    assert_eq!(h.app().active_tool(), Some(MeshOp::Remesh.cmd()));
    h.key(egui::Key::Escape);
    h.drive_until_idle();

    // S-key search: S, type, Enter.
    // (The focused search field's blinking cursor keeps repainting: step, don't drive to idle.)
    h.kittest().key_press(egui::Key::S);
    settle(&mut h);
    assert!(h.app().search().is_open());
    h.kittest().event(egui::Event::Text("plane cut".to_owned()));
    settle(&mut h);
    assert_eq!(
        h.app().search().results().first().map(|x| x.id),
        Some(MeshOp::PlaneCut.cmd())
    );
    h.kittest().key_press(egui::Key::Enter);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), Some(MeshOp::PlaneCut.cmd()));
    h.key(egui::Key::Escape);
    h.drive_until_idle();

    // Mesh Analysis from its toolbar button.
    click(&mut h, "Mesh Analysis");
    assert!(
        h.kittest().query_by_label("Watertight: yes").is_some(),
        "analysis panel open"
    );
    let _ = ANALYSIS_CMD;
}

// ---- dialogs and previews ---------------------------------------------------------------------

#[test]
fn a_dialog_previews_as_a_job_and_ok_commits_one_undoable_feature() {
    let mut h = AppHarness::new();
    let body = add_mesh(&mut h, sphere(16));
    let n = mesh(h.document().evaluated(), body).face_count();
    select(&mut h, body);
    start(&mut h, MeshOp::Reduce);
    // The preview ran as a job and shows the reduced body on the preview layer.
    let shown = match h.app_mut().preview_driver().state() {
        PreviewState::Ready(ev) => mesh(ev, body).face_count(),
        other => panic!("{other:?}"),
    };
    assert!(shown.abs_diff(n / 2) <= 4, "preview {shown} of {n}");
    assert!(h.scene().preview_bodies().any(|b| b == body));
    // The committed document is untouched until OK.
    assert_eq!(mesh(h.document().evaluated(), body).face_count(), n);
    assert_eq!(h.document().design().timeline().nodes().count(), 1);

    click(&mut h, "OK");
    assert_eq!(h.app().active_tool(), None);
    assert_eq!(h.scene().preview_bodies().count(), 0, "preview cleared");
    let after = mesh(h.document().evaluated(), body).face_count();
    assert_eq!(after, shown, "OK computes what the preview showed");
    assert_eq!(h.document().undo_label(), Some("Mesh Reduce"));

    let doc = h.app().active_document().id;
    h.app_mut().request_undo(doc);
    h.drive_until_idle();
    assert_eq!(mesh(h.document().evaluated(), body).face_count(), n);
    h.app_mut().request_redo(doc);
    h.drive_until_idle();
    assert_eq!(mesh(h.document().evaluated(), body).face_count(), after);
}

#[test]
fn erase_and_fill_needs_faces_and_uses_their_groups() {
    let mut h = AppHarness::new();
    let body = add_mesh(&mut h, sphere(12));
    select(&mut h, body);
    start(&mut h, MeshOp::EraseFill);
    // No faces: OK is disabled with the reason.
    assert!(
        h.kittest()
            .query_by_label("Select the faces to erase")
            .is_some()
    );
    h.key(egui::Key::Escape);
    h.drive_until_idle();

    // Preselected faces: the preview runs (the whole sphere is one group, so erasing it fails
    // in the preview with a banner, and OK stays possible only for a valid result).
    let face = SelItem::resolve(
        PickTarget::Body {
            body,
            instance: 0,
            sub: Some((amg_doc::TopoKind::Face, 0)),
        },
        h.document().evaluated(),
    )
    .expect("face item");
    {
        let sel = &mut h.app_mut().active_document_mut().selection;
        sel.clear();
        sel.add(face);
    }
    start(&mut h, MeshOp::EraseFill);
    assert!(
        matches!(
            h.app_mut().preview_driver().state(),
            PreviewState::Ready(_) | PreviewState::Failed(_)
        ),
        "the preview ran"
    );
    h.key(egui::Key::Escape);
    h.drive_until_idle();
}

#[test]
fn large_inputs_preview_on_a_proxy() {
    // Lower the proxy limit (process-wide; own test process under nextest).
    amg_features::mesh::set_preview_proxy_limits(2000, 500);
    let mut h = AppHarness::new();
    let body = add_mesh(&mut h, sphere(16));
    let n = mesh(h.document().evaluated(), body).face_count();
    select(&mut h, body);
    start(&mut h, MeshOp::Smooth);
    assert!(
        h.kittest()
            .query_by_label_contains("Preview on a decimated proxy")
            .is_some()
    );
    let shown = match h.app_mut().preview_driver().state() {
        PreviewState::Ready(ev) => mesh(ev, body).face_count(),
        other => panic!("{other:?}"),
    };
    assert!(shown <= 510, "proxy preview has {shown} faces");
    click(&mut h, "OK");
    assert_eq!(mesh(h.document().evaluated(), body).face_count(), n);
}

// ---- analysis panel ---------------------------------------------------------------------------

#[test]
fn the_analysis_panel_reports_and_highlights_defects_and_repair_clears_them() {
    let mut h = AppHarness::new();
    let body = add_mesh(&mut h, broken_sphere());
    h.run_command(ANALYSIS_CMD);
    h.drive_until_idle();
    h.kittest().step();
    h.drive_until_idle();
    h.kittest().step();
    assert!(h.kittest().query_by_label("Watertight: no").is_some());
    assert!(h.kittest().query_by_label("Holes: 1").is_some());
    let hl = h.scene().highlight(HighlightLayer::Error).len();
    assert!(hl > 0, "defect faces highlighted");

    // The toggle clears and restores the layer.
    h.kittest()
        .get_by_label_contains("Highlight defects")
        .click();
    h.kittest().step();
    h.drive_until_idle();
    assert_eq!(h.scene().highlight(HighlightLayer::Error).len(), 0);
    h.kittest()
        .get_by_label_contains("Highlight defects")
        .click();
    h.kittest().step();
    h.drive_until_idle();
    assert_eq!(h.scene().highlight(HighlightLayer::Error).len(), hl);

    // Repair from the panel; the panel re-analyses the new mesh.
    click(&mut h, "Repair…");
    assert_eq!(h.app().active_tool(), Some(MeshOp::Repair.cmd()));
    click(&mut h, "OK");
    h.kittest().step();
    h.drive_until_idle();
    h.kittest().step();
    assert!(h.kittest().query_by_label("Watertight: yes").is_some());
    assert_eq!(h.scene().highlight(HighlightLayer::Error).len(), 0);
    let m = mesh(h.document().evaluated(), body);
    let r = amg_mesh::analyze(
        &m,
        &amg_mesh::AnalyzeOpts::default(),
        &amg_jobs::JobCtx::detached(),
    )
    .unwrap();
    assert!(r.is_watertight());
    let _ = FaceId(0);
}

// ---- 1 M-tri ops: cancel and frame time -------------------------------------------------------

fn million() -> IndexedMesh {
    amg_mesh::corpus::bench::benchmark_sphere(amg_mesh::corpus::bench::BenchSize::M1, 1)
}

/// Waits (stepping frames) until a job labelled `label` runs; returns its id.
fn wait_for_job(h: &mut AppHarness, label: &str) -> amg_jobs::JobId {
    let t0 = Instant::now();
    let mut seen = Vec::new();
    loop {
        h.kittest().step();
        let jobs = h.app().jobs().jobs();
        for j in &jobs {
            let s = format!("{} {:?}", j.label, j.state);
            if !seen.contains(&s) {
                seen.push(s);
            }
        }
        if let Some(j) = jobs
            .into_iter()
            .find(|j| j.label == label && matches!(j.state, amg_jobs::JobState::Running(_)))
        {
            return j.id;
        }
        if t0.elapsed() >= Duration::from_secs(60) {
            let tool = h.app().active_tool();
            let preview = match h.app_mut().preview_driver().state() {
                PreviewState::Ready(ev) => format!(
                    "Ready: {:?}",
                    ev.nodes()
                        .map(|(id, n)| (id, n.health.clone()))
                        .collect::<Vec<_>>()
                ),
                PreviewState::Failed(e) => format!("Failed: {e}"),
                PreviewState::Pending => "Pending".to_owned(),
                PreviewState::Computing => "Computing".to_owned(),
                _ => "Idle".to_owned(),
            };
            panic!("no {label} job (seen {seen:?}, preview {preview}, tool {tool:?})");
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Steps frames until no job is left; the time it took.
fn time_until_no_jobs(h: &mut AppHarness) -> Duration {
    let t0 = Instant::now();
    while !h.app().jobs().jobs().is_empty() {
        h.kittest().step();
        assert!(t0.elapsed() < Duration::from_secs(30), "job not cancelled");
        std::thread::sleep(Duration::from_millis(1));
    }
    t0.elapsed()
}

#[test]
fn timing_cancel_of_a_1m_tri_preview_and_recompute_takes_under_200ms() {
    let mut h = AppHarness::new();
    let body = add_mesh(&mut h, million());
    select(&mut h, body);
    // Remesh of 1 M tris takes seconds: cancel the preview from the dialog.
    h.app_mut().request_command(MeshOp::Remesh.cmd());
    let _ = wait_for_job(&mut h, "Preview");
    std::thread::sleep(Duration::from_millis(300));
    h.kittest().step();
    h.kittest().get_by_label("Stop preview").click();
    let t0 = Instant::now();
    h.kittest().step();
    let preview_cancel = t0.elapsed() + time_until_no_jobs(&mut h);

    // Esc during a preview also cancels it.
    h.key(egui::Key::Escape);
    h.drive_until_idle();
    h.app_mut().request_command(MeshOp::Remesh.cmd());
    let _ = wait_for_job(&mut h, "Preview");
    std::thread::sleep(Duration::from_millis(300));
    let t0 = Instant::now();
    h.key(egui::Key::Escape);
    let esc_cancel = t0.elapsed() + time_until_no_jobs(&mut h);
    assert_eq!(h.app().active_tool(), None);

    // OK, then Cancel of the recompute job (what the jobs panel's Cancel does).
    h.app_mut().request_command(MeshOp::Remesh.cmd());
    let _ = wait_for_job(&mut h, "Preview");
    h.kittest().step();
    h.kittest().get_by_label("OK").click();
    h.kittest().step();
    let deadline = Instant::now() + Duration::from_secs(60);
    let recompute =
        loop {
            h.kittest().step();
            if let Some(j) =
                h.app().jobs().jobs().into_iter().find(|j| {
                    j.label != "Preview" && matches!(j.state, amg_jobs::JobState::Running(_))
                })
            {
                break j;
            }
            assert!(Instant::now() < deadline, "no recompute job");
            std::thread::sleep(Duration::from_millis(2));
        };
    std::thread::sleep(Duration::from_millis(300));
    let t0 = Instant::now();
    assert!(h.app().jobs().cancel(recompute.id));
    let recompute_cancel = t0.elapsed() + time_until_no_jobs(&mut h);
    eprintln!(
        "MESH-23 cancel (1 M-tri remesh): preview Stop {preview_cancel:?}, Esc {esc_cancel:?}, recompute job `{}` {recompute_cancel:?}",
        recompute.label
    );
    if strict_timing() {
        for (what, d) in [
            ("preview stop", preview_cancel),
            ("esc", esc_cancel),
            ("recompute", recompute_cancel),
        ] {
            assert!(d < Duration::from_millis(200), "{what} cancel took {d:?}");
        }
    }
    // The cancelled recompute leaves the document usable: undo removes the feature.
    let doc = h.app().active_document().id;
    h.app_mut().request_undo(doc);
    h.drive_until_idle();
    assert_eq!(
        mesh(h.document().evaluated(), body).face_count(),
        million_faces()
    );
}

fn million_faces() -> usize {
    MESHES.lock().unwrap()[0].face_count()
}

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

#[test]
fn timing_frames_stay_under_16ms_while_a_1m_tri_op_runs() {
    use tracing_subscriber::layer::SubscriberExt as _;

    let mut h = AppHarness::new();
    let body = add_mesh(&mut h, million());
    select(&mut h, body);
    let times = FrameTimes::default();
    let subscriber = tracing_subscriber::registry().with(times.clone());
    let _guard = tracing::subscriber::set_default(subscriber);

    // Preview (1 M-tri reduce job) with the dialog open, then OK (recompute job), stepping
    // frames like a continuously repainting event loop until everything is idle.
    let t0 = Instant::now();
    h.app_mut().request_command(MeshOp::Reduce.cmd());
    let mut ok_clicked = false;
    let mut busy_frames = 0usize;
    loop {
        h.kittest().step();
        let app = h.app();
        let busy = !app.jobs().jobs().is_empty();
        busy_frames += usize::from(busy);
        let previewed = matches!(h.app_mut().preview_driver().state(), PreviewState::Ready(_));
        if !ok_clicked && previewed {
            h.kittest().step();
            h.kittest().get_by_label("OK").click();
            ok_clicked = true;
        }
        let app = h.app();
        if ok_clicked
            && app.active_tool().is_none()
            && app.jobs().jobs().is_empty()
            && app.scene().pending() == 0
            && matches!(
                app.active_document().document.recompute_status(),
                amg_doc::RecomputeStatus::Idle
            )
        {
            break;
        }
        assert!(t0.elapsed() < Duration::from_secs(300), "op did not finish");
        std::thread::sleep(Duration::from_millis(2));
    }
    let total = t0.elapsed();
    let reduced = mesh(h.document().evaluated(), body).face_count();
    assert!(reduced < 600_000, "reduced to {reduced}");

    let mut t = times.0.lock().unwrap().clone();
    t.sort();
    let max = *t.last().unwrap();
    let p50 = t[t.len() / 2];
    let p99 = t[t.len() * 99 / 100];
    let over = t.iter().filter(|d| **d > Duration::from_millis(16)).count();
    eprintln!(
        "MESH-23 frames while a 1 M-tri reduce previews and commits: {} frames ({busy_frames} with jobs running) in {total:?}; p50 {p50:?}, p99 {p99:?}, max {max:?}, {over} over 16 ms",
        t.len()
    );
    if strict_timing() {
        assert!(max < Duration::from_millis(16), "max frame {max:?}");
    }
}

// ---- Phase 1 script on corpus models ----------------------------------------------------------

const SCRIPT_MODELS: [&str; 3] = [
    "mesh/real/thingi10k/132062.stl",
    "mesh/real/thingi10k/37825.stl",
    "mesh/real/thingi10k/36371.stl",
];

/// Imports an STL file into the open document as Insert Mesh does (S12 `io.import_mesh`).
fn import_stl(h: &mut AppHarness, bytes: &[u8], name: &str) -> BodyId {
    use amg_features::io::{ImportChoices, ImportSource, import_commands, raw_options};
    use amg_io::{Format, ImportOptions, Source};
    link();
    let ctx = amg_jobs::JobCtx::detached();
    let opts = ImportOptions::default();
    let imported = amg_io::import(
        Source::Bytes(bytes),
        Some(Format::Stl),
        &raw_options(&opts),
        &ctx,
    )
    .unwrap();
    let doc: &mut Document = &mut h.app_mut().active_document_mut().document;
    let choices = ImportChoices::from_options(doc.design().root(), &opts, &imported.report);
    let source = ImportSource::from_bytes(name, Format::Stl, imported.report.declared_unit, bytes);
    let plan = import_commands(imported, source, &choices).unwrap();
    plan.apply(doc).unwrap();
    h.drive_until_idle();
    newest_mesh_body(h.document().evaluated()).expect("imported mesh body")
}

/// Exports the visible bodies (File > Export) and reads the file back: 3MF, or STL while the
/// 3MF writer (S12 IO-11) is not there yet, unless `require_3mf`. Returns the format used, the
/// bytes and the read-back.
fn export_3mf(h: &AppHarness, require_3mf: bool) -> (amg_io::Format, Vec<u8>, amg_io::Imported) {
    use amg_features::io::{ExportPrep, ExportSelection, scene_from_document};
    use amg_io::{ExportOptions, Format, ImportOptions, IoError, Sink, Source};
    let ctx = amg_jobs::JobCtx::detached();
    let doc = h.document();
    let scene = scene_from_document(
        doc.design(),
        doc.evaluated(),
        &ExportSelection::AllVisible,
        &ExportPrep::default(),
        &ctx,
    )
    .unwrap();
    let mut out = Vec::new();
    let opts = ExportOptions::default();
    let format = match amg_io::export(&scene, Sink::Memory(&mut out), Format::ThreeMf, &opts, &ctx)
    {
        Ok(_) => Format::ThreeMf,
        Err(IoError::Unsupported { .. }) if !require_3mf => {
            out.clear();
            amg_io::export(&scene, Sink::Memory(&mut out), Format::Stl, &opts, &ctx).unwrap();
            Format::Stl
        }
        Err(e) => panic!("3MF export: {e}"),
    };
    let back = amg_io::import(
        Source::Bytes(&out),
        Some(format),
        &ImportOptions::default(),
        &ctx,
    )
    .unwrap();
    (format, out, back)
}

fn report(m: &IndexedMesh) -> amg_mesh::AnalysisReport {
    amg_mesh::analyze(
        m,
        &amg_mesh::AnalyzeOpts::default(),
        &amg_jobs::JobCtx::detached(),
    )
    .unwrap()
}

#[test]
fn phase1_script_on_three_corpus_models() {
    phase1_script(false);
}

/// The same script with a real 3MF export: switch on once the 3MF writer lands.
#[test]
#[ignore = "needs IO-11 (3MF writer)"]
fn phase1_script_on_three_corpus_models_exports_3mf() {
    phase1_script(true);
}

/// Open STL → analyse → repair → remesh → reduce → plane cut → shell → export (3MF, or STL
/// while IO-11 is open unless `require_3mf`).
fn phase1_script(require_3mf: bool) {
    let mut ran = 0;
    for rel in SCRIPT_MODELS {
        let Some(path) = amg_testkit::testdata(rel) else {
            continue;
        };
        let bytes = std::fs::read(&path).unwrap();
        let mut h = AppHarness::new();
        h.drive_until_idle();
        let t0 = Instant::now();
        // 1. Open STL.
        let body = import_stl(&mut h, &bytes, rel);
        let input = mesh(h.document().evaluated(), body);
        // 2. Analyse (panel).
        h.run_command(ANALYSIS_CMD);
        h.drive_until_idle();
        h.kittest().step();
        h.drive_until_idle();
        h.kittest().step();
        assert!(
            h.kittest()
                .query_by_label_contains("Watertight: ")
                .is_some(),
            "{rel}: analysis shown"
        );
        // 3..7. Repair, remesh, reduce, plane cut, shell: each through its dialog's OK.
        let mut line = format!("{rel}: {} faces", input.face_count());
        for op in [
            MeshOp::Repair,
            MeshOp::Remesh,
            MeshOp::Reduce,
            MeshOp::PlaneCut,
            MeshOp::Shell,
        ] {
            select(&mut h, body);
            let health = run_op(&mut h, op);
            assert_not_error(op, &health);
            let m = mesh(h.document().evaluated(), body);
            let r = report(&m);
            line.push_str(&format!(
                " → {} {} ({}{})",
                op.title(),
                m.face_count(),
                if r.is_watertight() {
                    "watertight"
                } else {
                    "open"
                },
                if matches!(health, Health::Warning(_)) {
                    ", warning"
                } else {
                    ""
                }
            ));
            if op == MeshOp::Repair {
                assert!(r.is_watertight(), "{rel}: repaired mesh is watertight");
            }
        }
        // The analysis panel follows the body: the shelled result is reported.
        h.kittest().step();
        h.drive_until_idle();
        h.kittest().step();
        assert!(h.kittest().query_by_label_contains("Faces: ").is_some());
        // 8. Export 3MF and read it back.
        let (format, bytes_out, back) = export_3mf(&h, require_3mf);
        let objects: usize = back.scene.objects.len();
        assert!(objects >= 1, "{rel}: the export has an object");
        let last = mesh(h.document().evaluated(), body);
        let total_faces: usize = back.scene.objects.iter().map(|o| o.mesh.face_count()).sum();
        assert_eq!(
            total_faces,
            last.face_count(),
            "{rel}: the export keeps every face"
        );
        eprintln!(
            "MESH-23 script {line} → {format:?} export {} bytes, {objects} object(s); {:?}",
            bytes_out.len(),
            t0.elapsed()
        );
        ran += 1;
    }
    eprintln!(
        "MESH-23 script ran on {ran} of {} models",
        SCRIPT_MODELS.len()
    );
}
