//! AM-12: the overhang overlay and metrics of the Manufacture workspace (`AppHarness`, kittest).
//!
//! - the toggle colours the items with the RND-08 overhang mode built from AM-02
//!   `shader_params`, runs the fine analysis, keeps its per-face class buffer, marks the islands
//!   and shows item / plate metrics; turning it off restores the material;
//! - Done-when 1: rotating a 1 M-tri item in Move/Rotate (live preview) keeps the overlay's
//!   uniforms in sync with the drawn transform on every frame; coarse metrics rate, fine latency
//!   after release and frame times are printed (asserted under `AMG_STRICT_TIMING=1`);
//! - Done-when 2: the overlay's classes (the RND-08 rule evaluated on the CPU in f32 with the
//!   same uniforms) agree with the fine CPU class buffer on ≥ 99.9 % of the faces of the test set
//!   (synthetic shapes + corpus models; un-fetched git-LFS pointers are skipped unless
//!   `AMG_REQUIRE_TESTDATA=1`);
//! - Done-when 3 (UI side): with the overlay on, orbiting a 5 M-tri item costs the UI thread no
//!   per-vertex work (frame times printed, asserted under `AMG_STRICT_TIMING=1`); the GPU side is
//!   the RND-08 bench row (`benches/viewport.rs`, overhang on 5 M tris);
//! - Done-when 4: changing α in Overhang Settings recolours on the next frame and the fine
//!   metrics for the new α arrive within 500 ms (asserted under `AMG_STRICT_TIMING=1`); OK is one
//!   undo entry.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    missing_docs,
    reason = "test code"
)]

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_am::machine::{Machine, MachineKey, MachineLibrary};
use amg_am::overhang::{AnalysisSettings, FaceClass, analyze_overhang};
use amg_am::place::{Placement, ZMode, effective_transform};
use amg_core::Transform;
use amg_core::glam::{DQuat, DVec2, DVec3};
use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, Command, Dependency, Evaluated, ExtKey,
    FeatureBox, FeatureError, FeatureOutput, RecomputeCtx,
};
use amg_features::am::setups;
use amg_jobs::JobCtx;
use amg_mesh::corpus::{FileStatus, Manifest, corpus_root, read_mesh};
use amg_mesh::{BvhOpts, IndexedMesh, MeshSpatial};
use amg_render::{Analysis, Shading};
use amg_ui::CommitBatch;
use amg_ui::am::overhang::{
    self, Quality, analysis_opts, island_points, overlay_shading, shader_face_classes, z_min_of,
};
use amg_ui::am::settings::{self, AmSettings};
use amg_ui::am::{
    AM, AmLayer, ItemStatus, MOVE_ROTATE, MoveRotateDialog, NEW_SETUP, NewSetupDialog,
    OVERHANG_SETTINGS, OVERHANG_TOGGLE, OverhangSettingsDialog, active_setup, command_ids,
};
use amg_ui::pick::PickTarget;
use amg_ui::scene::{CpuSceneBackend, InstanceDesc};
use amg_ui::select::SelItem;
use amg_ui::testing::AppHarness;
use amg_ui::testing::egui_kittest::kittest::Queryable;
use serde::{Deserialize, Serialize};

const KCM: &str = "builtin:creality/ender-3-v3-ke-kcm";

// ---- fixtures ---------------------------------------------------------------------------------

static MESHES: Mutex<Vec<Arc<IndexedMesh>>> = Mutex::new(Vec::new());

/// A mesh source whose mesh lives in a process-wide table (small feature records).
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

amg_doc::register_feature!(TestMesh, "uitest.am12_source", version = 1);

fn link() {
    amg_features::mesh::link();
    amg_features::io::link();
    amg_features::am::link();
}

fn newest_mesh_body(ev: &Evaluated) -> Option<BodyId> {
    ev.bodies()
        .filter(|(_, b)| matches!(b.geom, BodyGeom::Mesh(_)))
        .map(|(id, _)| id)
        .max()
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

fn mesh(h: &AppHarness, body: BodyId) -> Arc<IndexedMesh> {
    match &h.document().evaluated().body(body).expect("body").geom {
        BodyGeom::Mesh(m) => Arc::clone(m),
        other => panic!("{other:?}"),
    }
}

fn temp_dir(tag: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("amg-am12-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

fn configure(dir: &Path) {
    settings::set_settings_path(Some(dir.join("am-settings.json")));
    settings::save(AmSettings {
        slicer: None,
        handoff_dir: Some(dir.join("handoff")),
        machines_dir: Some(dir.join("machines")),
    });
}

fn select(h: &mut AppHarness, bodies: &[BodyId]) {
    let sel = &mut h.app_mut().active_document_mut().selection;
    sel.clear();
    for b in bodies {
        sel.add(
            SelItem::from_target(PickTarget::Body {
                body: *b,
                instance: 0,
                sub: None,
            })
            .unwrap(),
        );
    }
}

fn click(h: &mut AppHarness, label: &str) {
    h.kittest().step();
    h.kittest().get_by_label(label).click();
    h.kittest().step();
    h.drive_until_idle();
}

fn new_setup(h: &mut AppHarness, bodies: &[BodyId]) -> ExtKey {
    select(h, bodies);
    h.run_command(NEW_SETUP);
    h.kittest().step();
    {
        let d = h.app_mut().dialog_mut::<NewSetupDialog>().expect("dialog");
        d.machine = MachineKey(KCM.to_owned());
    }
    click(h, "OK");
    setups(h.document().design()).last().unwrap().0.clone()
}

fn enter(h: &mut AppHarness) {
    h.app_mut().set_workspace(AM);
    h.drive_until_idle();
    h.kittest().step();
    h.drive_until_idle();
}

fn layer(h: &AppHarness) -> &AmLayer {
    h.app().doc_layer::<AmLayer>().expect("plate layer")
}

fn status(h: &AppHarness, body: BodyId) -> ItemStatus {
    layer(h)
        .status()
        .iter()
        .find(|s| s.body == body)
        .cloned()
        .expect("item status")
}

fn active_settings(h: &AppHarness) -> (AnalysisSettings, Machine) {
    let chosen = layer(h).shown_setup().cloned();
    let (_, s) = active_setup(h.document().design(), chosen.as_ref()).unwrap();
    (s.settings, s.machine)
}

fn instance_of(h: &AppHarness, body: BodyId) -> InstanceDesc {
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

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1")
}

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").as_deref() == Ok("1")
}

/// Steps frames like a continuously repainting event loop until the app is idle.
fn step_until_idle(h: &mut AppHarness) {
    let t0 = Instant::now();
    loop {
        h.kittest().step();
        let app = h.app();
        let idle = app.jobs().jobs().is_empty()
            && app.scene().pending() == 0
            && !app.input_busy()
            && matches!(
                app.active_document().document.recompute_status(),
                amg_doc::RecomputeStatus::Idle
            )
            && app.doc_layer::<AmLayer>().is_none_or(|l| !l.busy());
        if idle {
            h.kittest().step();
            return;
        }
        assert!(t0.elapsed() < Duration::from_secs(300), "did not settle");
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// Frame times of the `amg_ui.frame` span.
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

fn percentiles(t: &mut [Duration]) -> (Duration, Duration, Duration, usize) {
    t.sort();
    let max = *t.last().unwrap();
    let p50 = t[t.len() / 2];
    let p99 = t[t.len() * 99 / 100];
    let over = t.iter().filter(|d| **d > Duration::from_millis(16)).count();
    (p50, p99, max, over)
}

/// The overhang shading of an instance: (build_up, α, plate height, tolerance).
fn overhang_of(d: &InstanceDesc) -> Option<(DVec3, f64, f64, f64)> {
    match &d.shading {
        Shading::Analysis(Analysis::Overhang {
            build_up,
            critical,
            plate_height,
            plate_tol,
        }) => Some((build_up.get(), *critical, *plate_height, *plate_tol)),
        _ => None,
    }
}

/// An axis-aligned box `size` with its min corner at the origin, transformed by `t`.
fn cuboid(size: DVec3, t: Transform) -> IndexedMesh {
    let p = (0..8u32)
        .map(|i| {
            t.transform_point3(DVec3::new(
                if i & 1 == 0 { 0.0 } else { size.x },
                if i & 2 == 0 { 0.0 } else { size.y },
                if i & 4 == 0 { 0.0 } else { size.z },
            ))
        })
        .collect();
    let tris = vec![
        [0, 2, 1],
        [1, 2, 3],
        [4, 5, 6],
        [5, 7, 6],
        [0, 1, 4],
        [1, 5, 4],
        [2, 6, 3],
        [3, 6, 7],
        [0, 4, 2],
        [2, 4, 6],
        [1, 3, 5],
        [3, 7, 5],
    ];
    IndexedMesh::new(p, tris).unwrap()
}

/// Several closed boxes in one mesh (each a shell): a "T" (post + bar) and a floating block,
/// so there are overhangs, contact and an island.
fn t_and_block() -> IndexedMesh {
    let parts = [
        (DVec3::new(10.0, 10.0, 30.0), DVec3::new(0.0, 0.0, 0.0)),
        (DVec3::new(40.0, 10.0, 5.0), DVec3::new(-15.0, 0.0, 30.0)),
        (DVec3::new(8.0, 8.0, 8.0), DVec3::new(40.0, 30.0, 12.0)),
    ];
    let mut pos = Vec::new();
    let mut tris = Vec::new();
    for (size, at) in parts {
        let c = cuboid(size, Transform::from_translation(at));
        let base = u32::try_from(pos.len()).unwrap();
        pos.extend(c.positions.iter().copied());
        tris.extend(c.tris.iter().map(|t| t.map(|i| i + base)));
    }
    IndexedMesh::new(pos, tris).unwrap()
}

/// The 1 M-tri benchmark sphere squashed into an ellipsoid, so rotating it changes the classes.
fn ellipsoid(size: amg_mesh::corpus::bench::BenchSize) -> IndexedMesh {
    let mut m = amg_mesh::corpus::bench::benchmark_sphere(size, 1);
    for p in &mut m.positions {
        *p *= DVec3::new(2.0, 1.0, 0.6);
    }
    m
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    fn quat(&mut self) -> DQuat {
        let (u1, u2, u3) = (self.next(), self.next(), self.next());
        let tau = std::f64::consts::TAU;
        DQuat::from_xyzw(
            (1.0 - u1).sqrt() * (tau * u2).sin(),
            (1.0 - u1).sqrt() * (tau * u2).cos(),
            u1.sqrt() * (tau * u3).sin(),
            u1.sqrt() * (tau * u3).cos(),
        )
        .normalize()
    }
}

fn kcm() -> Machine {
    MachineLibrary::builtin()
        .get(&MachineKey(KCM.to_owned()))
        .cloned()
        .unwrap()
}

fn on_plate(rotation: DQuat) -> Placement {
    Placement {
        rotation,
        xy: DVec2::new(110.0, 110.0),
        z: ZMode::OnPlate { elevation: 0.0 },
    }
}

// ---- the toggle, the class buffer, islands, metrics ----------------------------------------------

#[test]
fn overhang_toggle_colours_items_runs_the_fine_analysis_and_marks_islands() {
    configure(&temp_dir("toggle"));
    let mut h = AppHarness::new();
    let body = add_mesh(&mut h, t_and_block());
    new_setup(&mut h, &[body]);
    enter(&mut h);
    step_until_idle(&mut h);
    assert!(command_ids().contains(&OVERHANG_SETTINGS));
    assert!(
        overhang_of(&instance_of(&h, body)).is_none(),
        "material before the toggle"
    );
    assert!(layer(&h).overhang().analyses().is_empty(), "no job before");

    h.run_command(OVERHANG_TOGGLE);
    step_until_idle(&mut h);
    assert!(overhang::overlay_enabled());
    let st = status(&h, body);
    let t = st.transform.unwrap();
    let m = mesh(&h, body);
    let (s, machine) = active_settings(&h);
    let opts = analysis_opts(&s, &machine, Quality::Fine);

    // Uniforms: AM-02 shader_params at the drawn transform.
    let inst = instance_of(&h, body);
    assert_eq!(inst.transform, t);
    let (up, alpha, plate, tol) = overhang_of(&inst).expect("overhang shading");
    assert!((up - DVec3::Z).length() < 1e-12);
    assert!((alpha - s.critical_angle).abs() < 1e-12);
    assert!(
        (plate - z_min_of(&m, &t)).abs() < 1e-9,
        "plate height = item min z"
    );
    assert_eq!(tol, opts.contact_tolerance);
    assert_eq!(inst.shading, overlay_shading(&t, z_min_of(&m, &t), &opts));

    // The fine per-face class buffer equals a direct AM-02 analysis.
    let a = layer(&h)
        .overhang()
        .analysis(st.item)
        .expect("analysis")
        .clone();
    assert_eq!(a.quality, Quality::Fine);
    let spatial =
        MeshSpatial::build(Arc::clone(&m), &BvhOpts::default(), &JobCtx::detached()).unwrap();
    let direct = analyze_overhang(&spatial, &t, &opts, &JobCtx::detached()).unwrap();
    assert_eq!(a.face_classes(), direct.face_class.as_slice());
    assert_eq!(a.face_classes().len(), m.face_count());
    assert_eq!(*a.metrics(), direct.metrics);
    assert!(a.metrics().overhang_area > 0.0 && a.metrics().contact_area > 0.0);
    // The bar's underside (40 × 10 mm², two triangles over the post too) and the floating
    // block's bottom (64 mm²).
    assert!(
        (a.metrics().overhang_area - (400.0 + 64.0)).abs() < 1e-6,
        "{:?}",
        a.metrics()
    );
    // The overlay agrees with the class buffer on every face (no near-threshold faces here).
    assert_eq!(
        shader_face_classes(&m, &inst.transform, &inst.shading),
        a.face_classes()
    );

    // Island markers at the drawn transform: the bar and the block are separate shells, so their
    // bottoms are local minima; the block's is one of them.
    let markers = layer(&h).island_markers();
    let (_, pts) = markers.iter().find(|(i, _)| *i == st.item).unwrap();
    assert_eq!(*pts, island_points(&m, &direct, &t));
    assert!(!pts.is_empty() && a.metrics().island_count >= 1);
    let block_bottom = t.transform_point3(DVec3::new(40.0, 30.0, 12.0)).z;
    assert!(
        pts.iter().any(|p| (p.z - block_bottom).abs() < 1e-9),
        "{pts:?}"
    );

    // Plate metrics: one item, the same numbers.
    let plate_m = layer(&h).overhang().plate_metrics(&[st.item]);
    assert_eq!(plate_m.analysed, 1);
    assert_eq!(plate_m.overhang_area, a.metrics().overhang_area);
    assert_eq!(plate_m.island_count, a.metrics().island_count);

    // The Plate panel shows the item's metrics (the only item, no selection needed).
    h.kittest().step();
    let _ = h.kittest().get_by_label("Overhang area");
    let _ = h.kittest().get_by_label("Islands");

    // Off: material again, nothing running.
    h.run_command(OVERHANG_TOGGLE);
    step_until_idle(&mut h);
    assert!(!overhang::overlay_enabled());
    assert!(overhang_of(&instance_of(&h, body)).is_none());
    assert!(!layer(&h).overhang().busy());
}

// ---- Done-when 2: overlay vs CPU class buffer --------------------------------------------------

/// Corpus meshes (≤ `max_faces`), skipping un-fetched LFS pointers unless required.
fn corpus_meshes(count: usize, max_faces: usize) -> Vec<(String, IndexedMesh)> {
    let manifest = Manifest::load().expect("corpus.toml");
    let root = corpus_root();
    let mut out = Vec::new();
    for e in manifest.meshes.iter().filter(|e| e.faces <= max_faces) {
        if out.len() >= count {
            break;
        }
        match e.locate(&root) {
            FileStatus::Ready(p) => {
                if let Ok(m) = read_mesh(&p) {
                    out.push((e.id.clone(), m));
                }
            }
            FileStatus::LfsPointer(p) => assert!(
                !require_testdata(),
                "{} is an un-fetched git-LFS pointer and AMG_REQUIRE_TESTDATA=1",
                p.display()
            ),
            FileStatus::NotFetched(_) => {}
            FileStatus::Missing(p) => panic!("committed corpus file {} is missing", p.display()),
        }
    }
    if out.is_empty() {
        eprintln!("SKIP corpus: no corpus mesh available (LFS pointers); synthetic meshes only");
    }
    out
}

#[test]
fn overlay_classes_agree_with_the_cpu_class_buffer_on_the_test_set() {
    let machine = kcm();
    let settings = AnalysisSettings::for_machine(&machine);
    let mut set: Vec<(String, IndexedMesh)> = vec![
        ("t_and_block".into(), t_and_block()),
        (
            "ellipsoid_100k".into(),
            ellipsoid(amg_mesh::corpus::bench::BenchSize::K100),
        ),
        (
            "tilted_box".into(),
            cuboid(
                DVec3::new(20.0, 30.0, 40.0),
                Transform::from_rotation_translation(
                    DQuat::from_rotation_y(0.4),
                    DVec3::new(3.0, 4.0, 5.0),
                ),
            ),
        ),
    ];
    set.extend(corpus_meshes(8, 400_000));
    let mut rng = Rng(12);
    let (mut total, mut agree) = (0usize, 0usize);
    let mut lines = Vec::new();
    for (name, m) in &set {
        let m = Arc::new(m.clone());
        let spatial =
            MeshSpatial::build(Arc::clone(&m), &BvhOpts::default(), &JobCtx::detached()).unwrap();
        for (k, alpha_deg) in [45.0f64, 30.0, 60.0, 45.0].into_iter().enumerate() {
            let rot = if k == 0 { DQuat::IDENTITY } else { rng.quat() };
            let s = AnalysisSettings {
                critical_angle: alpha_deg.to_radians(),
                ..settings.clone()
            };
            let opts = analysis_opts(&s, &machine, Quality::Fine);
            let t = effective_transform(&m, &on_plate(rot));
            let report = analyze_overhang(&spatial, &t, &opts, &JobCtx::detached()).unwrap();
            let shading = overlay_shading(&t, report.z_min, &opts);
            let gpu = shader_face_classes(&m, &t, &shading);
            let same = gpu
                .iter()
                .zip(&report.face_class)
                .filter(|(a, b)| a == b)
                .count();
            let mut diffs: std::collections::BTreeMap<String, usize> = Default::default();
            for (g, c) in gpu.iter().zip(&report.face_class) {
                if g != c {
                    *diffs
                        .entry(format!("cpu {c:?} / overlay {g:?}"))
                        .or_default() += 1;
                }
            }
            let n = report.face_class.len();
            let kinds = |c: FaceClass| report.face_class.iter().filter(|x| **x == c).count();
            lines.push(format!(
                "{name} #{k} α={alpha_deg}°: {same}/{n} ({:.4} %), overhang {}, contact {}, degenerate {}; differences {diffs:?}",
                100.0 * same as f64 / n as f64,
                kinds(FaceClass::Overhang),
                kinds(FaceClass::Contact),
                kinds(FaceClass::Degenerate)
            ));
            total += n;
            agree += same;
        }
    }
    let ratio = agree as f64 / total as f64;
    for l in &lines {
        eprintln!("AM-12 overlay vs class buffer: {l}");
    }
    eprintln!(
        "AM-12 overlay vs class buffer: {agree}/{total} faces agree ({:.5} %) over {} meshes × 4 orientations",
        100.0 * ratio,
        set.len()
    );
    assert!(ratio >= 0.999, "agreement {ratio}");
}

// ---- Done-when 4: changing α --------------------------------------------------------------------

#[test]
fn changing_alpha_recolours_at_once_and_fine_metrics_follow_within_500ms() {
    configure(&temp_dir("alpha"));
    let mut h = AppHarness::builder().job_threads(0, 0).build();
    let body = add_mesh(&mut h, ellipsoid(amg_mesh::corpus::bench::BenchSize::K100));
    new_setup(&mut h, &[body]);
    enter(&mut h);
    h.run_command(OVERHANG_TOGGLE);
    step_until_idle(&mut h);
    let item = status(&h, body).item;
    let before = layer(&h)
        .overhang()
        .analysis(item)
        .unwrap()
        .metrics()
        .clone();
    let undo0 = h.document().history().stack().applied();

    h.run_command(OVERHANG_SETTINGS);
    h.kittest().step();
    h.app_mut()
        .dialog_mut::<OverhangSettingsDialog>()
        .expect("dialog")
        .fields
        .critical_deg = 60.0;
    let t0 = Instant::now();
    // Colours: the dialog sets the preview while it draws (after the layers), so the frame
    // after the edit carries the new α in the instance's uniforms (no job involved).
    let mut frames = 0;
    let recolour = loop {
        h.kittest().step();
        frames += 1;
        let (_, alpha, _, _) = overhang_of(&instance_of(&h, body)).unwrap();
        if (alpha - 60f64.to_radians()).abs() < 1e-12 {
            break t0.elapsed();
        }
        assert!(frames < 2, "α still {alpha} after {frames} frames");
    };
    // Metrics: the fine analysis for α = 60° (the preview holds still, so it is fine).
    let fine_at = loop {
        h.kittest().step();
        if let Some(a) = layer(&h).overhang().analysis(item)
            && a.quality == Quality::Fine
            && (a.settings.critical_angle - 60f64.to_radians()).abs() < 1e-12
        {
            break a.arrived.duration_since(t0);
        }
        assert!(t0.elapsed() < Duration::from_secs(30), "no fine result");
        std::thread::sleep(Duration::from_millis(1));
    };
    let after = layer(&h)
        .overhang()
        .analysis(item)
        .unwrap()
        .metrics()
        .clone();
    assert!(
        after.overhang_area > before.overhang_area,
        "a larger α flags more faces: {} → {}",
        before.overhang_area,
        after.overhang_area
    );
    eprintln!(
        "AM-12 α 45° → 60° on 100 k tris: colours after {frames} frame(s) ({recolour:?}), fine metrics after {fine_at:?}"
    );
    if strict_timing() {
        assert!(fine_at < Duration::from_millis(500), "{fine_at:?}");
    }
    click(&mut h, "OK");
    assert_eq!(
        h.document().history().stack().applied(),
        undo0 + 1,
        "one undo entry"
    );
    let (s, _) = active_settings(&h);
    assert!((s.critical_angle - 60f64.to_radians()).abs() < 1e-12);
    assert!(layer(&h).settings_preview().is_none());
}

// ---- Done-when 1: rotating a 1 M-tri item -----------------------------------------------------

#[test]
fn timing_rotating_a_1m_tri_item_keeps_the_overlay_in_sync() {
    use tracing_subscriber::layer::SubscriberExt as _;

    configure(&temp_dir("rotate"));
    // The app sizes its pools from the machine (`JobSystemConfig::default`), not 2 / 1.
    let mut h = AppHarness::builder().job_threads(0, 0).build();
    let m = ellipsoid(amg_mesh::corpus::bench::BenchSize::M1);
    let tris = m.face_count();
    let body = add_mesh(&mut h, m);
    new_setup(&mut h, &[body]);
    enter(&mut h);
    h.run_command(OVERHANG_TOGGLE);
    step_until_idle(&mut h); // the first fine pass also builds the BVH
    let m = mesh(&h, body);
    let item = status(&h, body).item;
    assert_eq!(
        layer(&h).overhang().analysis(item).map(|a| a.quality),
        Some(Quality::Fine)
    );
    // The jobs' cost without the app around them (same machine, same moment).
    let (s, machine) = active_settings(&h);
    let spatial =
        MeshSpatial::build(Arc::clone(&m), &BvhOpts::default(), &JobCtx::detached()).unwrap();
    let place = on_plate(DQuat::from_rotation_x(0.4));
    let mut iso = Vec::new();
    for q in [Quality::Coarse, Quality::Fine] {
        let opts = analysis_opts(&s, &machine, q);
        let mut best = Duration::MAX;
        for _ in 0..3 {
            let t0 = Instant::now();
            let t = effective_transform(&m, &place);
            let r = analyze_overhang(&spatial, &t, &opts, &JobCtx::detached()).unwrap();
            best = best.min(t0.elapsed());
            assert!(r.metrics.support.is_some());
        }
        iso.push(best);
    }
    let fit_t0 = Instant::now();
    let t = effective_transform(&m, &place);
    let _ = amg_am::place::check_fit(&m, &t, &machine.volume, &JobCtx::detached()).unwrap();
    let fit_cost = fit_t0.elapsed();
    eprintln!(
        "AM-12 isolated on {tris} tris: coarse job {:?}, fine job {:?} (best of 3), fit check {fit_cost:?}",
        iso[0], iso[1]
    );
    drop(spatial);
    select(&mut h, &[body]);
    h.app_mut().request_command(MOVE_ROTATE);
    step_until_idle(&mut h);

    let times = FrameTimes::default();
    let subscriber = tracing_subscriber::registry().with(times.clone());
    let guard = tracing::subscriber::set_default(subscriber);
    let c0 = layer(&h).overhang().counters();
    let mut coarse_arrivals: Vec<Instant> = Vec::new();
    let mut coarse_jobs: Vec<Duration> = Vec::new();
    let mut last_arrival = None;
    let mut synced = 0u32;
    let drag_start = Instant::now();
    let frames = 150u32;
    for i in 0..frames {
        if let Some(d) = h.app_mut().dialog_mut::<MoveRotateDialog>() {
            d.fields.tilt_x = f64::from(i) * 0.8;
            d.fields.rotate_z = f64::from(i) * 1.5;
        }
        h.kittest().step();
        // Every frame: the drawn transform and the overlay's plate height belong together.
        let inst = instance_of(&h, body);
        let (up, _, plate, _) = overhang_of(&inst).expect("overlay on");
        assert!((up - DVec3::Z).length() < 1e-12);
        let zmin = z_min_of(&m, &inst.transform);
        assert!(
            (plate - zmin).abs() < 1e-9,
            "frame {i}: plate {plate} vs min z {zmin}"
        );
        synced += 1;
        if let Some(a) = layer(&h).overhang().analysis(item)
            && a.quality == Quality::Coarse
            && last_arrival != Some(a.arrived)
        {
            last_arrival = Some(a.arrived);
            coarse_arrivals.push(a.arrived);
            coarse_jobs.push(a.elapsed);
        }
        std::thread::sleep(Duration::from_millis(6));
    }
    let drag = drag_start.elapsed();
    // Release: the fields hold still; the fine pass for the final placement follows.
    let release = Instant::now();
    let final_placement = h
        .app()
        .dialog::<MoveRotateDialog>()
        .map(|d| d.placement())
        .unwrap();
    let fine_after = loop {
        h.kittest().step();
        if let Some(a) = layer(&h).overhang().analysis(item)
            && a.quality == Quality::Fine
            && a.placement == final_placement
        {
            break a.arrived.duration_since(release);
        }
        assert!(
            release.elapsed() < Duration::from_secs(60),
            "no fine result"
        );
        std::thread::sleep(Duration::from_millis(1));
    };
    step_until_idle(&mut h);
    h.kittest().get_by_label("OK").click();
    step_until_idle(&mut h);
    drop(guard);
    let c1 = layer(&h).overhang().counters();

    let rate = coarse_arrivals.len() as f64 / drag.as_secs_f64();
    let gaps: Vec<f64> = coarse_arrivals
        .windows(2)
        .map(|w| w[1].duration_since(w[0]).as_secs_f64() * 1e3)
        .collect();
    let max_gap = gaps.iter().copied().fold(0.0, f64::max);
    let mut t = times.0.lock().unwrap().clone();
    let n = t.len();
    let (p50, p99, max, over) = percentiles(&mut t);
    coarse_jobs.sort();
    let coarse_p50 = coarse_jobs
        .get(coarse_jobs.len() / 2)
        .copied()
        .unwrap_or_default();
    let coarse_ms = layer(&h)
        .overhang()
        .analysis(item)
        .map(|a| a.elapsed)
        .unwrap();
    eprintln!(
        "AM-12 rotating a {tris}-tri item ({frames}-frame Move/Rotate drag in {drag:?}): overlay in sync on {synced}/{frames} frames; \
         coarse metrics {} updates = {rate:.1} Hz (max gap {max_gap:.0} ms, coarse job p50 {coarse_p50:?}); fine metrics {fine_after:?} after release (fine job {coarse_ms:?}); \
         jobs: coarse {} spawned / {} done, fine {} / {}, superseded {}; {n} frames p50 {p50:?}, p99 {p99:?}, max {max:?}, {over} over 16 ms",
        coarse_arrivals.len(),
        c1.coarse_spawned - c0.coarse_spawned,
        c1.coarse_done - c0.coarse_done,
        c1.fine_spawned - c0.fine_spawned,
        c1.fine_done - c0.fine_done,
        c1.superseded - c0.superseded,
    );
    assert_eq!(synced, frames);
    assert!(
        !coarse_arrivals.is_empty(),
        "coarse metrics during the drag"
    );
    if strict_timing() {
        assert!(rate >= 10.0, "coarse rate {rate} Hz");
        assert!(fine_after < Duration::from_millis(500), "{fine_after:?}");
        assert!(max < Duration::from_millis(16), "max frame {max:?}");
    }
}

// ---- Done-when 3 (UI side): orbiting a 5 M-tri item with the overlay on ------------------------

#[test]
fn timing_orbiting_a_5m_tri_item_with_the_overlay_costs_no_vertex_work() {
    use tracing_subscriber::layer::SubscriberExt as _;

    configure(&temp_dir("orbit"));
    let mut h = AppHarness::builder().job_threads(0, 0).build();
    // ≈ 5 M triangles: five 1 M ellipsoids side by side in one mesh.
    let one = ellipsoid(amg_mesh::corpus::bench::BenchSize::M1);
    let mut pos = Vec::new();
    let mut tris = Vec::new();
    for k in 0..5 {
        let base = u32::try_from(pos.len()).unwrap();
        let dx = DVec3::new(f64::from(k) * 45.0, 0.0, 0.0);
        pos.extend(one.positions.iter().map(|p| *p + dx));
        tris.extend(one.tris.iter().map(|t| t.map(|i| i + base)));
    }
    drop(one);
    let big = IndexedMesh::new(pos, tris).unwrap();
    let n_tris = big.face_count();
    let body = add_mesh(&mut h, big);
    new_setup(&mut h, &[body]);
    enter(&mut h);
    h.run_command(OVERHANG_TOGGLE);
    step_until_idle(&mut h);
    assert!(overhang_of(&instance_of(&h, body)).is_some());
    let fine = layer(&h)
        .overhang()
        .analysis(status(&h, body).item)
        .map(|a| a.elapsed)
        .unwrap();

    let times = FrameTimes::default();
    let subscriber = tracing_subscriber::registry().with(times.clone());
    let guard = tracing::subscriber::set_default(subscriber);
    let cam0 = h.app().active_document().camera;
    let c0 = layer(&h).overhang().counters();
    let frames = 240;
    for i in 0..frames {
        let a = f64::from(i) * std::f64::consts::TAU / f64::from(frames);
        let mut cam = cam0;
        let off = cam0.eye - cam0.target;
        cam.eye = cam0.target + DQuat::from_rotation_z(a) * off;
        h.app_mut().active_document_mut().camera = cam;
        h.kittest().step();
    }
    drop(guard);
    let c1 = layer(&h).overhang().counters();
    let mut t = times.0.lock().unwrap().clone();
    let n = t.len();
    let (p50, p99, max, over) = percentiles(&mut t);
    eprintln!(
        "AM-12 orbit with the overlay on, {n_tris} tris ({frames} orbit steps, {n} frames, CPU scene backend): p50 {p50:?}, p99 {p99:?}, max {max:?}, {over} over 16 ms; \
         analysis jobs during the orbit: {} (fine pass {fine:?})",
        (c1.coarse_spawned + c1.fine_spawned) - (c0.coarse_spawned + c0.fine_spawned)
    );
    assert_eq!(
        c1, c0,
        "orbiting starts no analysis (the overlay is uniforms only)"
    );
    if strict_timing() {
        assert!(max < Duration::from_millis(16), "max frame {max:?}");
    }
}

// ---- cancel-and-supersede ---------------------------------------------------------------------

#[test]
fn a_move_during_the_fine_pass_supersedes_it() {
    configure(&temp_dir("supersede"));
    let mut h = AppHarness::new();
    let body = add_mesh(&mut h, ellipsoid(amg_mesh::corpus::bench::BenchSize::M1));
    new_setup(&mut h, &[body]);
    enter(&mut h);
    h.run_command(OVERHANG_TOGGLE);
    step_until_idle(&mut h);
    let item = status(&h, body).item;
    select(&mut h, &[body]);
    h.app_mut().request_command(MOVE_ROTATE);
    step_until_idle(&mut h);
    let c0 = layer(&h).overhang().counters();
    let set_tilt = |h: &mut AppHarness, deg: f64| {
        h.app_mut()
            .dialog_mut::<MoveRotateDialog>()
            .expect("dialog")
            .fields
            .tilt_x = deg;
    };
    // A move (a coarse pass), then hold still: the fine pass starts once the coarse one is in.
    set_tilt(&mut h, 20.0);
    let t0 = Instant::now();
    let c1 = loop {
        h.kittest().step();
        let c = layer(&h).overhang().counters();
        if c.fine_spawned > c0.fine_spawned {
            break c;
        }
        assert!(
            t0.elapsed() < Duration::from_secs(30),
            "no fine pass after holding still"
        );
    };
    assert!(
        c1.coarse_spawned > c0.coarse_spawned,
        "the move ran a coarse pass"
    );
    assert!(layer(&h).overhang().busy(), "fine pass running");
    // Move again while it runs: the fine job is cancelled and superseded.
    set_tilt(&mut h, 35.0);
    h.kittest().step();
    h.kittest().step();
    let c2 = layer(&h).overhang().counters();
    assert!(c2.superseded > c1.superseded, "{c1:?} → {c2:?}");
    step_until_idle(&mut h);
    let fin = h
        .app()
        .dialog::<MoveRotateDialog>()
        .map(|d| d.placement())
        .unwrap();
    let a = layer(&h).overhang().analysis(item).unwrap();
    assert_eq!(a.quality, Quality::Fine);
    assert_eq!(a.placement, fin, "the result is for the newest placement");
    eprintln!(
        "AM-12 supersede: {c0:?} → {:?}",
        layer(&h).overhang().counters()
    );
}
