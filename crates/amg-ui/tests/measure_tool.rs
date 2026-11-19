//! INS-08: the Measure tool in the real app (`AppHarness`, kittest, CPU picking).
//!
//! - Measure is in the INSPECT tab and in S-key search; Esc closes it.
//! - Two faces of a cube give distance, Δ, angle and the RND-07 overlay; a pick 4 px from a
//!   vertex snaps to it; Pin keeps the overlay; a third pick starts over.
//! - Results use the document display unit and the chosen precision (a 1/4 in part in an
//!   inch document shows inches); Copy writes the displayed text.
//! - 3-point circle mode.
//! - A newer selection cancels the running job (timing assert only under
//!   `AMG_STRICT_TIMING=1`), and frames stay short while two 1 M-tri bodies are measured.
//! - The Phase 1 script on three corpus models: hole diameter (one click), wall distance,
//!   angle between two faces and body volume equal the INS-01 API values for the same picks
//!   (skipped for un-fetched git-LFS pointers unless `AMG_REQUIRE_TESTDATA=1`).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "test code"
)]

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_core::{Dir3, LengthUnit, Point3, Transform, Vec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, Dependency, Document, Evaluated,
    FeatureBox, FeatureError, FeatureOutput, RecomputeCtx, UnitPrefs,
};
use amg_features::inspect::measure::{
    self as m, MeasureOpts, MeasureReport, MeshElement, MeshOperand, Operand, OperandKind,
    RegionOpts,
};
use amg_jobs::JobCtx;
use amg_mesh::{FaceId, IndexedMesh, MeshBody};
use amg_render::{Camera, Projection};
use amg_ui::CommitBatch;
use amg_ui::command::Placement;
use amg_ui::design::tabs::INSPECT;
use amg_ui::inspect::{MEASURE_CMD, MeasureFilter, MeasureSnapshot, measure_snapshot};
use amg_ui::pick::{PickMode, PickTarget};
use amg_ui::search::{Recency, SearchIndex};
use amg_ui::select::SelItem;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable;
use serde::{Deserialize, Serialize};

// ---- fixtures ---------------------------------------------------------------------------------

static MESHES: Mutex<Vec<Arc<IndexedMesh>>> = Mutex::new(Vec::new());

/// A mesh source whose mesh lives in a process-wide table (small feature, cheap previews).
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

amg_doc::register_feature!(TestMesh, "uitest.ins08_source", version = 1);

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

/// An axis-aligned cube with outward, counter-clockwise triangles.
fn cube(min: Point3, s: f64) -> IndexedMesh {
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
    IndexedMesh::new(v, t).unwrap()
}

fn translated(mut m: IndexedMesh, d: Vec3) -> IndexedMesh {
    for p in &mut m.positions {
        *p += d;
    }
    IndexedMesh::new(m.positions.clone(), m.tris.clone()).unwrap()
}

fn select(h: &mut AppHarness, bodies: &[BodyId]) {
    let sel = &mut h.app_mut().active_document_mut().selection;
    sel.clear();
    for &body in bodies {
        sel.add(
            SelItem::from_target(PickTarget::Body {
                body,
                instance: 0,
                sub: None,
            })
            .unwrap(),
        );
    }
}

fn start(h: &mut AppHarness) {
    h.run_command(MEASURE_CMD);
    h.kittest().step();
    assert_eq!(h.app().active_tool(), Some(MEASURE_CMD), "Measure started");
}

/// The tool's state after one more frame.
fn snap(h: &mut AppHarness) -> MeasureSnapshot {
    h.kittest().step();
    measure_snapshot(&h.kittest().ctx).expect("the Measure dialog was drawn")
}

/// Steps until the measurement finished; its snapshot.
fn result(h: &mut AppHarness) -> MeasureSnapshot {
    h.drive_until_idle();
    let s = snap(h);
    assert!(s.running.is_none(), "no job left: {s:?}");
    assert!(s.error.is_none(), "no error: {:?}", s.error);
    s
}

/// A label with exactly `text` is shown (several may be).
fn shown(h: &mut AppHarness, text: &str) -> bool {
    h.kittest().query_all_by_label(text).next().is_some()
}

fn row<'a>(s: &'a MeasureSnapshot, key: &str) -> &'a m::MeasureRow {
    s.rows
        .iter()
        .find(|r| r.key == key)
        .unwrap_or_else(|| panic!("row {key} in {:?}", s.rows))
}

/// Puts the camera `dist` mm from `target` along `from`, looking at it.
fn look(h: &mut AppHarness, target: Point3, from: Vec3, dist: f64) {
    let dir = from.normalize();
    let up = if dir.z.abs() > 0.9 { Dir3::Y } else { Dir3::Z };
    h.app_mut().active_document_mut().camera = Camera {
        eye: target + dir * dist,
        target,
        up,
        projection: Projection::Perspective { fov_y: FOV },
    };
    h.drive_until_idle();
}

const FOV: f64 = 0.8;

fn click_pos(h: &mut AppHarness, pos: egui::Pos2) {
    h.click_at(pos, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.drive_until_idle();
}

fn click(h: &mut AppHarness, p: Point3) {
    let pos = h.world_to_screen(p);
    click_pos(h, pos);
}

fn click_label(h: &mut AppHarness, label: &str) {
    h.kittest().step();
    h.kittest().get_by_label(label).click();
    h.kittest().step();
    h.drive_until_idle();
}

/// Sets the dialog's filter check boxes to `want`.
fn set_filter(h: &mut AppHarness, want: MeasureFilter) {
    type Get = fn(&MeasureFilter) -> bool;
    let boxes: [(&str, Get); 5] = [
        ("Body", |f| f.body),
        ("Face", |f| f.face),
        ("Edge", |f| f.edge),
        ("Vertex", |f| f.vertex),
        ("Point", |f| f.point),
    ];
    for (label, get) in boxes {
        if get(&snap(h).filter) != get(&want) {
            // The check box (the browser may show a "Body" row too).
            h.kittest().step();
            h.kittest()
                .get_by_role_and_label(egui::accesskit::Role::CheckBox, label)
                .click();
            h.kittest().step();
            h.drive_until_idle();
        }
    }
    assert_eq!(snap(h).filter, want);
}

fn body_only() -> MeasureFilter {
    MeasureFilter {
        body: true,
        face: false,
        edge: false,
        vertex: false,
        point: false,
    }
}

fn opts() -> MeasureOpts {
    MeasureOpts::default()
}

fn ins01(a: &Operand, b: Option<&Operand>) -> MeasureReport {
    m::measure(a, b, &opts(), &JobCtx::detached()).unwrap()
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1")
}

// ---- command --------------------------------------------------------------------------------

#[test]
fn measure_is_in_the_inspect_tab_and_search_and_esc_closes_it() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    {
        let reg = h.app().registry();
        let def = reg.command(MEASURE_CMD).expect("registered");
        let groups: Vec<_> = def
            .placements
            .iter()
            .filter_map(|p| match p {
                Placement::Toolbar { group, .. } => Some(*group),
                _ => None,
            })
            .collect();
        assert!(
            groups
                .iter()
                .any(|g| reg.group(*g).map(|g| g.tab) == Some(INSPECT)),
            "in the INSPECT tab: {groups:?}"
        );
        let index = SearchIndex::build(reg);
        let hits = index.search("Measure", &Recency::new());
        assert_eq!(hits.first().map(|x| x.id), Some(MEASURE_CMD));
        assert!(
            index
                .search("diameter", &Recency::new())
                .iter()
                .any(|x| x.id == MEASURE_CMD)
        );
    }
    // Disabled without a mesh body.
    h.run_command(MEASURE_CMD);
    assert_eq!(h.app().active_tool(), None);
    add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    start(&mut h);
    assert!(h.kittest().query_by_label("Selection 1").is_some());
    h.key(egui::Key::Escape);
    h.drive_until_idle();
    assert_eq!(h.app().active_tool(), None, "Esc closes Measure");
    assert!(h.app().tool_overlays().is_empty());
}

// ---- cube -----------------------------------------------------------------------------------

#[test]
fn two_faces_of_a_cube_give_distance_angle_and_the_overlay() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    add_mesh(&mut h, cube(Point3::ZERO, 10.0));
    start(&mut h);

    // Top face (smart: the 2-triangle square), away from its diagonal.
    look(&mut h, Point3::new(5.0, 5.0, 10.0), Vec3::Z, 60.0);
    click(&mut h, Point3::new(2.0, 5.0, 10.0));
    let s = result(&mut h);
    assert_eq!(s.kinds, vec![Some(OperandKind::Faces)]);
    let r = s.report.as_ref().unwrap();
    assert!((r.first.area.unwrap().to_mm2() - 100.0).abs() < 1e-9);
    assert_eq!(row(&s, "first.area").text, "100 mm^2");

    // Bottom face from below: distance 10, angle 180°.
    look(&mut h, Point3::new(5.0, 5.0, 0.0), -Vec3::Z, 60.0);
    click(&mut h, Point3::new(2.0, 5.0, 0.0));
    let s = result(&mut h);
    assert_eq!(s.kinds.len(), 2);
    let r = s.report.clone().unwrap();
    let pair = r.pair.as_ref().unwrap();
    assert!((pair.min_distance.distance.to_mm() - 10.0).abs() < 1e-12);
    let angle = pair.angle.unwrap().angle.to_rad();
    assert!((angle - std::f64::consts::PI).abs() < 1e-12, "{angle}");
    // The same as INS-01 for the same operands.
    assert_eq!(r, ins01(&s.operands[0], Some(&s.operands[1])));
    let dist = row(&s, "distance").text.clone();
    assert_eq!(dist, "10 mm");
    assert!(shown(&mut h, &dist), "row shown");
    // RND-07 overlay: witness line, points, Δ leg (z only), angle arc and labels.
    let o = h.app().tool_overlays();
    assert!(o.primitive_count() >= 4, "{}", o.primitive_count());
    assert!(o.label_count() >= 3, "{}", o.label_count());
    let before = o.primitive_count();
    click_label(&mut h, "Pin");
    assert_eq!(snap(&mut h).pinned, 1);
    assert!(h.app().tool_overlays().primitive_count() > before);

    // A third pick starts over: +X side face, then the top again → 90°.
    look(&mut h, Point3::new(10.0, 5.0, 5.0), Vec3::X, 60.0);
    click(&mut h, Point3::new(10.0, 3.0, 6.0));
    let s = result(&mut h);
    assert_eq!(s.kinds.len(), 1, "selection 2 cleared");
    look(&mut h, Point3::new(5.0, 5.0, 10.0), Vec3::Z, 60.0);
    click(&mut h, Point3::new(2.0, 5.0, 10.0));
    let s = result(&mut h);
    let angle = s
        .report
        .as_ref()
        .unwrap()
        .pair
        .as_ref()
        .unwrap()
        .angle
        .unwrap();
    assert!((angle.angle.to_deg() - 90.0).abs() < 1e-9, "{angle:?}");
    assert_eq!(row(&s, "angle").text, "90 deg");

    // Snap: 4 px from the corner (10, 10, 10), inside the top face → that vertex.
    click_label(&mut h, "Restart");
    let corner = h.world_to_screen(Point3::new(10.0, 10.0, 10.0));
    click_pos(&mut h, corner + egui::vec2(-3.0, 3.0));
    let s = result(&mut h);
    assert_eq!(s.kinds, vec![Some(OperandKind::Vertex)]);
    assert_eq!(row(&s, "first.x").text, "10 mm");
    // 10 px left and 24 px down (away from the edges and the diagonal) it is a face again.
    click_label(&mut h, "Restart");
    click_pos(&mut h, corner + egui::vec2(-10.0, 24.0));
    assert_eq!(result(&mut h).kinds, vec![Some(OperandKind::Faces)]);
    // Alt-click keeps the single triangle.
    click_label(&mut h, "Restart");
    let pos = h.world_to_screen(Point3::new(2.0, 5.0, 10.0));
    h.click_at(pos, egui::PointerButton::Primary, egui::Modifiers::ALT);
    h.drive_until_idle();
    let s = result(&mut h);
    let a = s.report.as_ref().unwrap().first.area.unwrap().to_mm2();
    assert!((a - 50.0).abs() < 1e-9, "one triangle: {a}");
}

// ---- units ----------------------------------------------------------------------------------

#[test]
fn results_use_the_document_unit_and_the_chosen_precision() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    h.app_mut().submit(CommitBatch {
        label: "Units".into(),
        commands: vec![Command::SetDesignMeta {
            name: None,
            default_units: Some(UnitPrefs {
                length: LengthUnit::Inch,
            }),
        }],
    });
    h.drive_until_idle();
    // A 1/4 in cube, preselected.
    let body = add_mesh(&mut h, cube(Point3::ZERO, 6.35));
    select(&mut h, &[body]);
    start(&mut h);
    let s = result(&mut h);
    assert_eq!(s.prefs.length, LengthUnit::Inch);
    assert_eq!(s.prefs.precision, 3);
    assert_eq!(s.kinds, vec![Some(OperandKind::Body)]);
    let report = s.report.clone().unwrap();
    assert_eq!(s.rows, m::report_rows(&report, &s.prefs));
    assert_eq!(row(&s, "first.volume").text, "0.016 in^3");
    assert_eq!(row(&s, "first.area").text, "0.375 in^2");
    assert!(shown(&mut h, "0.016 in^3"));

    click_label(&mut h, "Fewer decimals");
    let s = snap(&mut h);
    assert_eq!(s.prefs.precision, 2);
    assert_eq!(row(&s, "first.volume").text, "0.02 in^3");
    click_label(&mut h, "More decimals");
    click_label(&mut h, "More decimals");
    let s = snap(&mut h);
    assert_eq!(row(&s, "first.volume").text, "0.0156 in^3");
    assert_eq!(
        s.report.as_ref(),
        Some(&report),
        "re-formatting does not remeasure"
    );

    // Copy writes the displayed text.
    h.kittest().step();
    h.kittest().get_by_label("Copy Volume").click();
    h.kittest().step();
    let copied: Vec<String> = h
        .kittest()
        .output()
        .platform_output
        .commands
        .iter()
        .filter_map(|c| match c {
            egui::OutputCommand::CopyText(t) => Some(t.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(copied, vec!["0.0156 in^3".to_owned()]);
}

// ---- 3-point circle -------------------------------------------------------------------------

#[test]
fn three_point_circle_is_the_circumcircle_of_the_snapped_points() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    add_mesh(&mut h, cube(Point3::ZERO, 100.0));
    start(&mut h);
    click_label(&mut h, "3-point circle");
    assert!(snap(&mut h).three_point);
    look(&mut h, Point3::new(50.0, 50.0, 100.0), Vec3::Z, 250.0);
    for p in [(30.0, 60.0), (60.0, 75.0), (60.0, 35.0)] {
        click(&mut h, Point3::new(p.0, p.1, 100.0));
    }
    let s = result(&mut h);
    assert_eq!(s.operands.len(), 3);
    let pts: Vec<Point3> = s
        .operands
        .iter()
        .map(|o| match o {
            Operand::Point(p) => *p,
            other => panic!("{other:?}"),
        })
        .collect();
    let expected = m::circle_through(pts[0], pts[1], pts[2], &amg_core::Tolerance::mesh()).unwrap();
    let got = s.report.as_ref().unwrap().first.circle.unwrap().circle;
    assert_eq!(got, expected);
    let d = row(&s, "first.diameter");
    assert!((d.value.value - 2.0 * expected.radius).abs() < 1e-12);
    // The circle is drawn.
    assert!(h.app().tool_overlays().primitive_count() >= 2);
    // A fourth point starts a new circle.
    click(&mut h, Point3::new(40.0, 40.0, 100.0));
    let s = snap(&mut h);
    assert_eq!(s.operands.len(), 1);
    assert!(s.report.is_none());
}

// ---- supersede ------------------------------------------------------------------------------

#[test]
fn timing_a_newer_selection_cancels_the_running_job_within_50ms() {
    let mut h = AppHarness::new();
    h.drive_until_idle();
    // Nested spheres far apart: a long, cancellable minimum-distance job (S05 INS-01 log).
    let outer = add_mesh(
        &mut h,
        amg_mesh::corpus::bench::geodesic_sphere(60, 50.0, None),
    );
    let inner = add_mesh(
        &mut h,
        amg_mesh::corpus::bench::geodesic_sphere(60, 30.0, None),
    );
    select(&mut h, &[outer, inner]);
    h.app_mut().request_command(MEASURE_CMD);
    // Do not wait for idle: the measurement runs for a long time.
    let t0 = Instant::now();
    let first = loop {
        h.kittest().step();
        if let Some(s) = measure_snapshot(&h.kittest().ctx)
            && let Some(id) = s.running
        {
            break id;
        }
        assert!(t0.elapsed() < Duration::from_secs(30), "the job started");
        std::thread::sleep(Duration::from_millis(2));
    };
    // Let it run.
    for _ in 0..20 {
        h.kittest().step();
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(
        h.app().jobs().jobs().iter().any(|j| j.id == first),
        "still running"
    );
    // A newer selection: a click on the outer sphere.
    let cam_target = Point3::ZERO;
    h.app_mut().active_document_mut().camera = Camera {
        eye: Point3::new(0.0, 0.0, 200.0),
        target: cam_target,
        up: Dir3::Y,
        projection: Projection::Perspective { fov_y: FOV },
    };
    h.kittest().step();
    h.kittest().step();
    let pos = h.world_to_screen(Point3::new(3.0, 4.0, 49.0));
    h.click_at(pos, egui::PointerButton::Primary, egui::Modifiers::NONE);
    let s = loop {
        let s = snap(&mut h);
        if s.superseded.iter().any(|(id, _)| *id == first) {
            break s;
        }
        assert!(t0.elapsed() < Duration::from_secs(60), "superseded");
    };
    let cancelled_at = s
        .superseded
        .iter()
        .find(|(id, _)| *id == first)
        .map(|(_, t)| *t)
        .unwrap();
    while h.app().jobs().jobs().iter().any(|j| j.id == first) {
        assert!(cancelled_at.elapsed() < Duration::from_secs(5), "job gone");
        std::thread::sleep(Duration::from_micros(200));
    }
    let latency = cancelled_at.elapsed();
    eprintln!("INS-08 supersede: the running measure job finished {latency:?} after the cancel");
    if strict_timing() {
        assert!(latency < Duration::from_millis(50), "{latency:?}");
    }
    // The new selection is measured.
    let s = result(&mut h);
    assert_eq!(s.kinds.len(), 1, "selection 2 cleared");
    assert!(s.report.is_some());
}

// ---- frame time -----------------------------------------------------------------------------

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
fn timing_frames_stay_under_16ms_while_measuring_two_1m_tri_bodies() {
    use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
    use tracing_subscriber::layer::SubscriberExt as _;

    let mut h = AppHarness::new();
    h.drive_until_idle();
    let a = add_mesh(&mut h, benchmark_sphere(BenchSize::M1, 1));
    let b = add_mesh(
        &mut h,
        translated(
            benchmark_sphere(BenchSize::M1, 2),
            Vec3::new(25.0, 0.0, 0.0),
        ),
    );
    let faces = mesh(h.document().evaluated(), a).face_count()
        + mesh(h.document().evaluated(), b).face_count();
    start(&mut h);
    set_filter(&mut h, body_only());
    look(&mut h, Point3::new(12.5, 0.0, 0.0), Vec3::Z, 90.0);
    let pa = h.world_to_screen(Point3::new(0.0, 0.0, 10.0));
    let pb = h.world_to_screen(Point3::new(25.0, 0.0, 10.0));
    // The CPU pick service (tests only; the app picks on the GPU) builds its ray-cast BVH on
    // the first cast: warm it up outside the measured frames.
    let view = h.camera_view();
    for p in [pa, pb] {
        assert!(!h.pick_ray(view.ray(p), PickMode::Nearest).is_empty());
    }

    let times = FrameTimes::default();
    let subscriber = tracing_subscriber::registry().with(times.clone());
    let _guard = tracing::subscriber::set_default(subscriber);
    let t0 = Instant::now();
    h.click_at(pa, egui::PointerButton::Primary, egui::Modifiers::NONE);
    h.click_at(pb, egui::PointerButton::Primary, egui::Modifiers::NONE);
    let s = loop {
        h.kittest().step();
        let s = measure_snapshot(&h.kittest().ctx).unwrap();
        if s.running.is_none() && s.report.as_ref().is_some_and(|r| r.pair.is_some()) {
            break s;
        }
        assert!(s.error.is_none(), "{:?}", s.error);
        assert!(
            t0.elapsed() < Duration::from_secs(300),
            "measure did not finish"
        );
        std::thread::sleep(Duration::from_millis(2));
    };
    let total = t0.elapsed();
    let r = s.report.as_ref().unwrap();
    let d = r.pair.as_ref().unwrap().min_distance.distance.to_mm();
    assert!(d > 3.0 && d < 6.0, "gap {d}");
    assert!(r.first.volume.is_some() && r.second.as_ref().unwrap().volume.is_some());

    let mut t = times.0.lock().unwrap().clone();
    t.sort();
    let max = *t.last().unwrap();
    let p50 = t[t.len() / 2];
    let p99 = t[t.len() * 99 / 100];
    let over = t.iter().filter(|d| **d > Duration::from_millis(16)).count();
    eprintln!(
        "INS-08 frames while measuring two 1 M-tri bodies ({faces} faces, gap {d:.4} mm): {} frames in {total:?}; p50 {p50:?}, p99 {p99:?}, max {max:?}, {over} over 16 ms",
        t.len()
    );
    if strict_timing() {
        assert!(max < Duration::from_millis(16), "max frame {max:?}");
    }
}

// ---- Phase 1 script on corpus models ----------------------------------------------------------

/// Watertight CAD parts with round holes (thingi10k, CC licences per `testdata`).
const SCRIPT_MODELS: [&str; 3] = [
    "mesh/real/thingi10k/232912.stl",
    "mesh/real/thingi10k/36371.stl",
    "mesh/real/thingi10k/1750623.stl",
];

/// Imports an STL file into the open document as Insert Mesh does (S12 `io.import_mesh`).
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
    let doc: &mut Document = &mut h.app_mut().active_document_mut().document;
    let choices = ImportChoices::from_options(doc.design().root(), &opts, &imported.report);
    let source = ImportSource::from_bytes(name, Format::Stl, imported.report.declared_unit, bytes);
    let plan = import_commands(imported, source, &choices).unwrap();
    plan.apply(doc).unwrap();
    h.drive_until_idle();
    newest_mesh_body(h.document().evaluated()).expect("imported mesh body")
}

/// The model in world space, as the test sees it.
struct Model {
    body: BodyId,
    mesh: Arc<IndexedMesh>,
    to_world: Transform,
}

impl Model {
    fn corner(&self, f: u32) -> [Point3; 3] {
        self.mesh
            .triangle(FaceId(f))
            .map(|p| self.to_world.transform_point3(p))
    }

    fn normal(&self, f: u32) -> Vec3 {
        let [a, b, c] = self.corner(f);
        (b - a).cross(c - a).normalize_or_zero()
    }

    fn operand(&self, element: MeshElement) -> Operand {
        Operand::Mesh(MeshOperand {
            body: MeshBody::from_arc(Arc::clone(&self.mesh)),
            to_world: self.to_world,
            element,
        })
    }
}

/// Viewport height in physical pixels.
fn view_height_px(h: &AppHarness) -> f64 {
    f64::from(h.camera_view().size_px[1])
}

/// Camera distance at which one pixel is `mm` at the target (perspective, `FOV`).
fn dist_for(h: &AppHarness, mm_per_px: f64) -> f64 {
    mm_per_px * view_height_px(h) / (2.0 * (FOV / 2.0).tan())
}

/// Looks at `aim` along the normal of face `f` from `dist`, and checks that a click there
/// picks face `f` first (not occluded). Returns the screen position.
fn aim_at(h: &mut AppHarness, m: &Model, f: u32, aim: Point3, dist: f64) -> Option<egui::Pos2> {
    let n = m.normal(f);
    if n == Vec3::ZERO {
        return None;
    }
    look(h, aim, n, dist);
    let pos = h.camera_view().project(aim)?;
    let view = h.camera_view();
    let first = h
        .pick_ray(view.ray(pos), PickMode::Nearest)
        .into_iter()
        .next()?;
    (first.target
        == PickTarget::Body {
            body: m.body,
            instance: 0,
            sub: Some((amg_doc::TopoKind::Face, f)),
        })
    .then_some(pos)
}

/// The incentre and inradius of face `f`.
fn incircle(m: &Model, f: u32) -> (Point3, f64) {
    let [a, b, c] = m.corner(f);
    let (la, lb, lc) = ((b - c).length(), (c - a).length(), (a - b).length());
    let p = la + lb + lc;
    let area = 0.5 * (b - a).cross(c - a).length();
    ((a * la + b * lb + c * lc) / p, 2.0 * area / p)
}

/// A triangle of `region` that a click on its incentre, seen along its normal, picks first
/// (largest inradius first).
fn visible_face(h: &mut AppHarness, m: &Model, region: &[FaceId]) -> Option<u32> {
    let mut faces: Vec<(u32, f64)> = region.iter().map(|f| (f.0, incircle(m, f.0).1)).collect();
    faces.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
    faces.into_iter().take(200).find_map(|(f, r_in)| {
        let (aim, _) = incircle(m, f);
        aim_at(h, m, f, aim, dist_for(h, r_in / 8.0).max(1.0)).map(|_| f)
    })
}

/// Clicks the incentre of face `f` (8 px from its edges), seen along its normal.
fn click_face(h: &mut AppHarness, m: &Model, f: u32) {
    let (aim, r_in) = incircle(m, f);
    let pos = aim_at(h, m, f, aim, dist_for(h, r_in / 8.0).max(1.0)).expect("visible");
    click_pos(h, pos);
}

/// Planar regions from the largest triangles: `(faces, unit normal, a point, area)`.
fn planar_regions(m: &Model) -> Vec<(Arc<[FaceId]>, Vec3, Point3, f64)> {
    let ctx = JobCtx::detached();
    let mesh = &*m.mesh;
    let mut by_area: Vec<(u32, f64)> = (0..mesh.face_count() as u32)
        .map(|f| {
            let [a, b, c] = m.corner(f);
            (f, (b - a).cross(c - a).length())
        })
        .collect();
    by_area.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
    let mut taken = vec![false; mesh.face_count()];
    let mut out = Vec::new();
    for &(seed, _) in by_area.iter().take(60) {
        if taken[seed as usize] {
            continue;
        }
        let region =
            m::grow_planar_region(mesh, FaceId(seed), &RegionOpts::default(), &ctx).unwrap();
        for f in region.iter() {
            taken[f.0 as usize] = true;
        }
        let e = m::measure_one(
            &m.operand(MeshElement::Faces(Arc::clone(&region))),
            &opts(),
            &ctx,
        )
        .unwrap();
        if let Some(p) = e.plane {
            out.push((
                region,
                p.plane.normal.get(),
                p.plane.origin,
                e.area.unwrap().to_mm2(),
            ));
        }
    }
    out
}

/// A triangle edge on a hole: `(a, b, an adjacent face)`, from the longest feature edges
/// whose INS-01 chain is closed with an accepted circle fit.
fn hole_edges(m: &Model) -> Vec<(u32, u32, [u32; 2])> {
    let mesh = &*m.mesh;
    let mut edges: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
    for (f, t) in (0u32..).zip(&mesh.tris) {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(f);
        }
    }
    let cos30 = (std::f64::consts::PI / 6.0).cos();
    let mut feature: Vec<((u32, u32), [u32; 2], f64)> = edges
        .into_iter()
        .filter_map(|(e, fs)| {
            let [f0, f1] = fs[..] else { return None };
            (m.normal(f0).dot(m.normal(f1)) < cos30).then(|| {
                let l = (mesh.positions[e.0 as usize] - mesh.positions[e.1 as usize]).length();
                (e, [f0, f1], l)
            })
        })
        .collect();
    feature.sort_by(|a, b| b.2.total_cmp(&a.2).then(a.0.cmp(&b.0)));
    let ctx = JobCtx::detached();
    let mut seen_chains: Vec<Vec<u32>> = Vec::new();
    let mut out = Vec::new();
    for ((a, b), faces, _) in feature.into_iter().take(4000) {
        if seen_chains.iter().any(|c| c.contains(&a) && c.contains(&b)) {
            continue;
        }
        let el = m::grow_feature_chain(mesh, a, b, std::f64::consts::PI / 6.0, &ctx).unwrap();
        let MeshElement::Polyline { verts, closed } = &el else {
            continue;
        };
        seen_chains.push(verts.to_vec());
        if !*closed {
            continue;
        }
        let e = m::measure_one(&m.operand(el.clone()), &opts(), &ctx).unwrap();
        if e.circle.is_some() {
            out.push((a, b, faces));
            if out.len() >= 8 {
                break;
            }
        }
    }
    out
}

/// `p` is one of the edge ends `a`, `b`.
fn near(p: Point3, a: Point3, b: Point3) -> bool {
    (p - a).length() < 1e-9 || (p - b).length() < 1e-9
}

/// Distance from `p` to segment `a b`.
fn seg_dist(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
    (a + ab * t - p).length()
}

/// One click on a hole rim edge (2 px inside an adjacent face, ≥ 12 px from its ends).
fn click_hole(h: &mut AppHarness, m: &Model) -> (u32, u32) {
    for (a, b, faces) in hole_edges(m) {
        let pa = m.to_world.transform_point3(m.mesh.positions[a as usize]);
        let pb = m.to_world.transform_point3(m.mesh.positions[b as usize]);
        let mid = pa.lerp(pb, 0.5);
        let len = (pb - pa).length();
        for f in faces {
            let [c0, c1, c2] = m.corner(f);
            let centroid = (c0 + c1 + c2) / 3.0;
            let axis = (pb - pa) / len;
            let inward = (centroid - mid) - axis * (centroid - mid).dot(axis);
            let Some(inward) = inward.try_normalize() else {
                continue;
            };
            // 2 px inside the face; zoom in until the face's other edges are > 6 px away.
            let others: Vec<(Point3, Point3)> = [(c0, c1), (c1, c2), (c2, c0)]
                .into_iter()
                .filter(|(p, q)| !(near(*p, pa, pb) && near(*q, pa, pb)))
                .collect();
            let mut mm_px = len / 30.0;
            let mut aim = mid + inward * (2.0 * mm_px);
            for _ in 0..20 {
                let d = others
                    .iter()
                    .map(|(p, q)| seg_dist(aim, *p, *q))
                    .fold(f64::INFINITY, f64::min);
                if d > 6.0 * mm_px {
                    break;
                }
                mm_px *= 0.5;
                aim = mid + inward * (2.0 * mm_px);
            }
            if let Some(pos) = aim_at(h, m, f, aim, dist_for(h, mm_px)) {
                click_pos(h, pos);
                return (a, b);
            }
        }
    }
    panic!("no visible hole edge");
}

#[test]
fn phase1_measure_script_on_three_corpus_models() {
    let mut ran = 0;
    for rel in SCRIPT_MODELS {
        let Some(path) = amg_testkit::testdata(rel) else {
            continue;
        };
        let bytes = std::fs::read(&path).unwrap();
        let mut h = AppHarness::new();
        h.drive_until_idle();
        let t0 = Instant::now();
        let body = import_stl(&mut h, &bytes, rel);
        let model = Model {
            body,
            mesh: mesh(h.document().evaluated(), body),
            to_world: h.scene().instance_world(body, 0).expect("displayed"),
        };
        start(&mut h);
        let mut line = format!("{rel} ({} faces):", model.mesh.face_count());

        // 1. Hole diameter: one click on a rim edge (smart chain + circle fit).
        let (a, b) = click_hole(&mut h, &model);
        let s = result(&mut h);
        assert_eq!(
            s.kinds,
            vec![Some(OperandKind::Polyline)],
            "{rel}: one chain"
        );
        // The aimed rim edge's INS-01 chain (either direction; the loop may start elsewhere).
        let aimed = m::grow_feature_chain(
            &model.mesh,
            a,
            b,
            std::f64::consts::PI / 6.0,
            &JobCtx::detached(),
        )
        .unwrap();
        let verts = |e: &MeshElement| match e {
            MeshElement::Polyline {
                verts,
                closed: true,
            } => {
                let mut v = verts.to_vec();
                v.sort_unstable();
                v
            }
            other => panic!("{rel}: closed chain expected, got {other:?}"),
        };
        let picked = match &s.operands[0] {
            Operand::Mesh(op) => op.element.clone(),
            other => panic!("{other:?}"),
        };
        assert_eq!(
            verts(&picked),
            verts(&aimed),
            "{rel}: the aimed hole's chain"
        );
        let got = s.report.clone().unwrap();
        assert_eq!(got, ins01(&s.operands[0], None), "{rel}: hole = INS-01");
        let want = ins01(&model.operand(aimed), None);
        let (rg, rw) = (
            got.first.circle.unwrap().circle.radius,
            want.first.circle.unwrap().circle.radius,
        );
        assert!((rg - rw).abs() < 1e-9, "{rel}: radius {rg} vs {rw}");
        let dia = row(&s, "first.diameter").clone();
        assert!(shown(&mut h, &dia.text), "{rel}: shown");
        line.push_str(&format!(" hole ⌀ {}", dia.text));

        // 2. Wall distance between two opposite planar faces; 3. angle between two faces.
        // Only regions with a triangle a click can reach.
        let regions: Vec<_> = planar_regions(&model)
            .into_iter()
            .filter_map(|r| {
                let f = visible_face(&mut h, &model, &r.0)?;
                Some((r.0, r.1, r.2, r.3, f))
            })
            .collect();
        let (i, j) = (0..regions.len())
            .flat_map(|i| (0..regions.len()).map(move |j| (i, j)))
            .filter(|&(i, j)| {
                i < j && {
                    let (ri, rj) = (&regions[i], &regions[j]);
                    ri.1.dot(rj.1) < -0.9999 && (rj.2 - ri.2).dot(ri.1).abs() > 0.1
                }
            })
            .max_by(|x, y| {
                let s = |(i, j): (usize, usize)| regions[i].3.min(regions[j].3);
                s(*x).total_cmp(&s(*y))
            })
            .expect("two opposite planar faces");
        let k = (0..regions.len())
            .filter(|&k| k != i && regions[i].1.dot(regions[k].1).abs() < 0.99)
            .max_by(|x, y| regions[*x].3.total_cmp(&regions[*y].3))
            .expect("a face at an angle");
        let region_op = |f: u32| {
            let r = m::grow_planar_region(
                &model.mesh,
                FaceId(f),
                &RegionOpts::default(),
                &JobCtx::detached(),
            )
            .unwrap();
            model.operand(MeshElement::Faces(r))
        };

        click_label(&mut h, "Restart");
        let fa = regions[i].4;
        click_face(&mut h, &model, fa);
        let fb = regions[j].4;
        click_face(&mut h, &model, fb);
        let s = result(&mut h);
        let wall = ins01(&region_op(fa), Some(&region_op(fb)));
        assert_eq!(s.report.as_ref(), Some(&wall), "{rel}: wall = INS-01");
        let text = row(&s, "distance").text.clone();
        assert!(shown(&mut h, &text));
        line.push_str(&format!(", wall {text}"));

        click_label(&mut h, "Restart");
        let fa = regions[i].4;
        click_face(&mut h, &model, fa);
        let fc = regions[k].4;
        click_face(&mut h, &model, fc);
        let s = result(&mut h);
        let angle = ins01(&region_op(fa), Some(&region_op(fc)));
        assert_eq!(s.report.as_ref(), Some(&angle), "{rel}: angle = INS-01");
        assert!(angle.pair.as_ref().unwrap().angle.is_some());
        line.push_str(&format!(", angle {}", row(&s, "angle").text));

        // 4. Body volume.
        click_label(&mut h, "Restart");
        set_filter(&mut h, body_only());
        click_face(&mut h, &model, regions[i].4);
        let s = result(&mut h);
        let vol = ins01(&model.operand(MeshElement::Body), None);
        assert_eq!(s.report.as_ref(), Some(&vol), "{rel}: body = INS-01");
        assert!(vol.first.volume.is_some(), "{rel}: watertight");
        line.push_str(&format!(", volume {}", row(&s, "first.volume").text));
        set_filter(&mut h, MeasureFilter::default());

        eprintln!("INS-08 script {line}; {:?}", t0.elapsed());
        ran += 1;
    }
    eprintln!(
        "INS-08 script ran on {ran} of {} models",
        SCRIPT_MODELS.len()
    );
}
