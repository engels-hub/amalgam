//! IO-15: File ▸ Insert Mesh / Export, drag and drop, the import and export dialogs, in the
//! real app (`AppHarness`, egui_kittest). Drag and drop is simulated through egui's dropped-files
//! input.
//!
//! - Done-when 1: `drop_imports_each_format`, `drop_of_an_unsupported_file_shows_the_error_and_hint`.
//! - Done-when 2: `timing_big_stl_*` (`#[ignore]`; run command in the test docs; the 16 ms and
//!   200 ms bounds are asserted only under `AMG_STRICT_TIMING=1`).
//! - Done-when 3: `unit_change_updates_extents_without_rereading`.
//! - Done-when 4: `export_with_holes_*`.
//! - Done-when 5: `phase1_script_*` (corpus models, skipped for un-fetched git-LFS pointers
//!   unless `AMG_REQUIRE_TESTDATA=1`; the Orca CLI part needs `AMG_ORCA_SLICER` or the default
//!   path `D:/OrcaSlicer/orca-slicer.exe`, required with `AMG_REQUIRE_ORCA=1`).

#![allow(
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::expect_used,
    missing_docs,
    reason = "test code"
)]

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_core::Transform;
use amg_core::units::LengthUnit;
use amg_doc::{BodyGeom, BodyId, Evaluated};
use amg_io::{
    ExportOptions, Format, ImportOptions, Scene, SceneNode, SceneObject, Sink, SlicerFlavor,
    Source, ValidityPolicy,
};
use amg_mesh::IndexedMesh;
use amg_ui::io::{
    EXPORT_CMD, ExportDialog, INSERT_MESH_CMD, ImportDialog, UnitSel, prefs, set_prefs_path,
};
use amg_ui::testing::egui_kittest::kittest::Queryable;
use amg_ui::testing::{AppHarness, FakeFileDialog};

// ---- helpers ------------------------------------------------------------------------------

/// The prefs store is process-wide: serialise the tests of one process (`cargo test`).
static SERIAL: Mutex<()> = Mutex::new(());

struct Env {
    _guard: std::sync::MutexGuard<'static, ()>,
    dir: PathBuf,
}

impl Drop for Env {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

fn env(tag: &str) -> Env {
    let guard = SERIAL
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = std::env::temp_dir().join(format!("amg-io15-{tag}-{}-{nanos}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    set_prefs_path(Some(dir.join("io-prefs.json")));
    amg_features::io::link();
    amg_features::mesh::link();
    Env { _guard: guard, dir }
}

fn harness() -> (AppHarness, Arc<FakeFileDialog>) {
    let mut h = AppHarness::new();
    let fake = FakeFileDialog::new();
    h.app_mut().set_file_dialog_backend(fake.clone());
    h.drive_until_idle();
    (h, fake)
}

#[derive(Debug)]
struct Dropped(PathBuf);

impl egui::DroppedFile for Dropped {
    fn path(&self) -> &Path {
        &self.0
    }
    fn bytes(&self) -> Result<Vec<u8>, String> {
        std::fs::read(&self.0).map_err(|e| e.to_string())
    }
}

/// Drops `path` onto the window (one frame with the dropped-files input).
fn drop_file(h: &mut AppHarness, path: &Path) {
    h.kittest()
        .input_mut()
        .dropped_files
        .push(Arc::new(Dropped(path.to_owned())));
    h.kittest().step();
}

/// Steps frames until `f` holds (a repaint loop), panics after `secs`.
fn wait(h: &mut AppHarness, secs: u64, what: &str, mut f: impl FnMut(&mut AppHarness) -> bool) {
    let t = Instant::now();
    loop {
        h.kittest().step();
        if f(h) {
            return;
        }
        assert!(t.elapsed() < Duration::from_secs(secs), "timed out: {what}");
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn import_ready(h: &AppHarness) -> bool {
    h.app()
        .dialog::<ImportDialog>()
        .is_some_and(ImportDialog::is_ready)
}

fn click(h: &mut AppHarness, label: &str) {
    h.kittest().step();
    h.kittest().get_by_label(label).click();
    h.kittest().step();
}

/// The geodesic sphere (radius 10 mm, closed).
fn sphere() -> IndexedMesh {
    amg_mesh::corpus::bench::geodesic_sphere(4, 10.0, None)
}

/// The sphere without its upper cap: boundary edges.
fn holed_sphere() -> IndexedMesh {
    let s = sphere();
    let keep: Vec<[u32; 3]> = s
        .tris
        .iter()
        .filter(|t| t.iter().all(|&v| s.positions[v as usize].z < 8.0))
        .copied()
        .collect();
    IndexedMesh::new(s.positions.clone(), keep).unwrap()
}

fn scene_of(meshes: Vec<IndexedMesh>) -> Scene {
    let mut scene = Scene::default();
    for (i, m) in meshes.into_iter().enumerate() {
        let mut o = SceneObject::new(Arc::new(m));
        o.name = Some(format!("Part {}", i + 1));
        scene.objects.push(o);
        scene
            .roots
            .push(SceneNode::mesh(i as u32, Transform::IDENTITY));
    }
    scene
}

/// Writes `scene` as `format` (invalid meshes allowed).
fn write_scene(scene: &Scene, format: Format, path: &Path) {
    let opts = ExportOptions {
        validity: Some(ValidityPolicy::AllowInvalid),
        ..ExportOptions::default()
    };
    amg_io::export(
        scene,
        Sink::Path(path),
        format,
        &opts,
        &amg_jobs::JobCtx::detached(),
    )
    .unwrap();
}

fn mesh_bodies(ev: &Evaluated) -> Vec<(BodyId, Arc<IndexedMesh>)> {
    ev.bodies()
        .filter_map(|(id, b)| match &b.geom {
            BodyGeom::Mesh(m) => Some((id, Arc::clone(m))),
            _ => None,
        })
        .collect()
}

fn extents(m: &IndexedMesh) -> [f64; 3] {
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for p in &m.positions {
        for (k, v) in [p.x, p.y, p.z].into_iter().enumerate() {
            lo[k] = lo[k].min(v);
            hi[k] = hi[k].max(v);
        }
    }
    [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]]
}

fn timeline_kinds(h: &AppHarness) -> Vec<String> {
    h.document()
        .design()
        .timeline()
        .nodes()
        .map(|n| n.feature.kind().to_owned())
        .collect()
}

/// Drops `path`, waits for the dialog, presses OK and waits for the body.
fn insert_by_drop(h: &mut AppHarness, path: &Path) {
    drop_file(h, path);
    wait(h, 60, "import dialog ready", |h| import_ready(h));
    click(h, "OK");
    wait(h, 60, "inserted", |h| {
        h.app().dialog::<ImportDialog>().is_none() && h.document().is_evaluated()
    });
    h.drive_until_idle();
}

// ---- Done-when 1: drop ----------------------------------------------------------------------

#[test]
fn drop_imports_each_format() {
    let e = env("drop");
    let scene = scene_of(vec![sphere()]);
    let want = extents(&scene.objects[0].mesh);
    for (format, name) in [
        (Format::Stl, "a.stl"),
        (Format::ThreeMf, "b.3mf"),
        (Format::Obj, "c.obj"),
        (Format::Ply, "d.ply"),
    ] {
        let file = e.dir.join(name);
        write_scene(&scene, format, &file);
        let (mut h, _) = harness();
        insert_by_drop(&mut h, &file);
        let bodies = mesh_bodies(h.document().evaluated());
        assert_eq!(bodies.len(), 1, "{name}: one body");
        let got = extents(&bodies[0].1);
        for k in 0..3 {
            assert!(
                (got[k] - want[k]).abs() < 1e-4,
                "{name}: extent {k}: {} vs {}",
                got[k],
                want[k]
            );
        }
        assert!(
            timeline_kinds(&h).contains(&"io.import_mesh".to_owned()),
            "{name}: {:?}",
            timeline_kinds(&h)
        );
        assert_eq!(
            h.document().undo_label(),
            Some(format!("Insert {name}").as_str()),
            "{name}: one undo entry"
        );
    }
}

#[test]
fn drop_of_an_unsupported_file_shows_the_error_and_hint() {
    let e = env("unsupported");
    let file = e.dir.join("notes.xyz");
    std::fs::write(&file, "this is not a mesh file\n").unwrap();
    let (mut h, _) = harness();
    drop_file(&mut h, &file);
    wait(&mut h, 30, "error toast", |h| {
        h.app().dialog::<ImportDialog>().is_none() && !h.app().notifications().items().is_empty()
    });
    let toasts = h.app().notifications().items();
    let n = &toasts[0].notification;
    assert_eq!(n.severity, amg_core::Severity::Error);
    assert!(n.body.contains("not recognised"), "{}", n.body);
    assert!(n.hint.is_some(), "the IoError hint is shown: {n:?}");
    assert!(mesh_bodies(h.document().evaluated()).is_empty());
}

#[test]
fn a_dropped_design_file_opens_instead_of_importing() {
    let e = env("amgdrop");
    let file = e.dir.join("design.amg");
    let doc = amg_doc::Document::new("Dropped");
    amg_doc::save(&file, &doc.snapshot(), &amg_doc::io::SaveOptions::default()).unwrap();
    let (mut h, _) = harness();
    drop_file(&mut h, &file);
    wait(&mut h, 30, "design opened", |h| {
        h.app().documents().len() == 2
    });
    assert!(h.app().dialog::<ImportDialog>().is_none());
}

#[test]
fn insert_mesh_command_asks_for_a_file() {
    let e = env("command");
    let file = e.dir.join("p.stl");
    write_scene(&scene_of(vec![sphere()]), Format::Stl, &file);
    let (mut h, fake) = harness();
    fake.answer(Some(file.clone()));
    h.app_mut().request_command(INSERT_MESH_CMD);
    wait(&mut h, 60, "ready", |h| import_ready(h));
    let req = fake.requests();
    assert_eq!(req.len(), 1);
    assert_eq!(req[0].kind, amg_ui::file::FileDialogKind::Open);
    assert!(req[0].filters[0].extensions.iter().any(|e| e == "stl"));
    assert!(req[0].filters[0].extensions.iter().any(|e| e == "3mf"));
    click(&mut h, "Cancel");
    h.drive_until_idle();
    assert!(h.app().dialog::<ImportDialog>().is_none());
    assert!(mesh_bodies(h.document().evaluated()).is_empty());
}

// ---- Done-when 3: unit change -------------------------------------------------------------

#[test]
fn unit_change_updates_extents_without_rereading() {
    let e = env("unit");
    let file = e.dir.join("p.stl");
    let scene = scene_of(vec![sphere()]);
    write_scene(&scene, Format::Stl, &file);
    let raw = extents(&scene.objects[0].mesh);
    let (mut h, _) = harness();
    drop_file(&mut h, &file);
    wait(&mut h, 60, "ready", |h| import_ready(h));
    // The file is gone: anything that re-read it would fail.
    std::fs::remove_file(&file).unwrap();
    h.kittest().step();
    let size = |e: [f64; 3]| format!("Size: {:.3} × {:.3} × {:.3} mm", e[0], e[1], e[2]);
    assert!(h.kittest().query_by_label(&size(raw)).is_some(), "mm size");
    for (unit, factor) in [
        (LengthUnit::Inch, 25.4),
        (LengthUnit::Centimetre, 10.0),
        (LengthUnit::Millimetre, 1.0),
    ] {
        h.app_mut()
            .dialog_mut::<ImportDialog>()
            .unwrap()
            .set_unit(UnitSel::Unit(unit));
        h.kittest().step();
        let want = raw.map(|v| v * factor);
        assert!(
            h.kittest().query_by_label(&size(want)).is_some(),
            "{unit:?}: {}",
            size(want)
        );
        let d = h.app().dialog::<ImportDialog>().unwrap();
        assert_eq!(d.extents_mm().unwrap(), want);
        assert_eq!(d.reads(), 1, "the file was read once");
    }
    // The up axis turns the extents too.
    h.app_mut()
        .dialog_mut::<ImportDialog>()
        .unwrap()
        .set_unit(UnitSel::Unit(LengthUnit::Inch));
    click(&mut h, "OK");
    wait(&mut h, 60, "inserted", |h| {
        h.app().dialog::<ImportDialog>().is_none() && h.document().is_evaluated()
    });
    let bodies = mesh_bodies(h.document().evaluated());
    let got = extents(&bodies[0].1);
    for k in 0..3 {
        assert!((got[k] - raw[k] * 25.4).abs() < 1e-3, "inserted in inches");
    }
}

#[test]
fn y_up_and_bed_fit_follow_the_choices() {
    let e = env("bed");
    let file = e.dir.join("long.stl");
    // 300 mm long in X, 20 in Y, 40 in Z.
    let m = {
        let s = sphere();
        let pos = s
            .positions
            .iter()
            .map(|p| amg_core::Point3::new(p.x * 15.0, p.y, p.z * 2.0))
            .collect();
        IndexedMesh::new(pos, s.tris.clone()).unwrap()
    };
    write_scene(&scene_of(vec![m]), Format::Stl, &file);
    let (mut h, _) = harness();
    drop_file(&mut h, &file);
    wait(&mut h, 60, "ready", |h| import_ready(h));
    let d = h.app().dialog::<ImportDialog>().unwrap();
    let z_up = d.extents_mm().unwrap();
    assert!(z_up[0] > 290.0, "{z_up:?}");
    h.kittest().step();
    assert!(
        h.kittest()
            .query_by_label_contains("Does not fit the build volume (256 × 256 × 256 mm")
            .is_some(),
        "300 mm does not fit the default 256 mm bed"
    );
    h.app_mut()
        .dialog_mut::<ImportDialog>()
        .unwrap()
        .set_unit(UnitSel::Unit(LengthUnit::Centimetre));
    h.kittest().step();
    assert!(
        h.kittest()
            .query_by_label_contains("Does not fit")
            .is_some()
    );
    h.app_mut()
        .dialog_mut::<ImportDialog>()
        .unwrap()
        .set_unit(UnitSel::Unit(LengthUnit::Micrometre));
    h.app_mut()
        .dialog_mut::<ImportDialog>()
        .unwrap()
        .set_up_axis(amg_io::UpAxis::Y);
    h.kittest().step();
    let y_up = h
        .app()
        .dialog::<ImportDialog>()
        .unwrap()
        .extents_mm()
        .unwrap();
    assert!(
        (y_up[1] - z_up[2] / 1000.0).abs() < 1e-9,
        "Y up swaps y and z"
    );
    assert!(
        h.kittest()
            .query_by_label_contains("Fits the build volume")
            .is_some(),
        "0.3 mm fits"
    );
}

// ---- remembered choices -------------------------------------------------------------------

#[test]
fn choices_are_remembered_per_format() {
    let e = env("prefs");
    let scene = scene_of(vec![sphere()]);
    let stl = e.dir.join("a.stl");
    let obj = e.dir.join("b.obj");
    write_scene(&scene, Format::Stl, &stl);
    write_scene(&scene, Format::Obj, &obj);
    let (mut h, _) = harness();
    drop_file(&mut h, &stl);
    wait(&mut h, 60, "ready", |h| import_ready(h));
    {
        let d = h.app_mut().dialog_mut::<ImportDialog>().unwrap();
        d.set_unit(UnitSel::Unit(LengthUnit::Inch));
        d.set_up_axis(amg_io::UpAxis::Y);
    }
    click(&mut h, "OK");
    wait(&mut h, 60, "inserted", |h| {
        h.app().dialog::<ImportDialog>().is_none() && h.document().is_evaluated()
    });
    // Another STL starts with the same choices; an OBJ starts with its own (the defaults).
    drop_file(&mut h, &stl);
    wait(&mut h, 60, "ready", |h| import_ready(h));
    {
        let d = h.app().dialog::<ImportDialog>().unwrap();
        assert_eq!(d.unit(), UnitSel::Unit(LengthUnit::Inch));
        assert_eq!(d.up_axis(), amg_io::UpAxis::Y);
    }
    click(&mut h, "Cancel");
    drop_file(&mut h, &obj);
    wait(&mut h, 60, "ready", |h| import_ready(h));
    {
        let d = h.app().dialog::<ImportDialog>().unwrap();
        assert_eq!(d.unit(), UnitSel::FromFile);
        assert_eq!(d.up_axis(), amg_io::UpAxis::Z);
    }
    click(&mut h, "Cancel");
    // They survive a restart: the file is read again.
    set_prefs_path(Some(e.dir.join("io-prefs.json")));
    let p = prefs::load();
    assert_eq!(p.import_for(Format::Stl).unit, Some(LengthUnit::Inch));
    assert_eq!(p.import_for(Format::Obj).unit, None);
}

#[test]
fn merge_and_separate() {
    let e = env("merge");
    let file = e.dir.join("two.3mf");
    let mut b = sphere();
    for p in &mut b.positions {
        p.x += 50.0;
    }
    write_scene(&scene_of(vec![sphere(), b]), Format::ThreeMf, &file);
    let (mut h, _) = harness();
    insert_by_drop(&mut h, &file);
    assert_eq!(mesh_bodies(h.document().evaluated()).len(), 2, "separate");
    let (mut h, _) = harness();
    drop_file(&mut h, &file);
    wait(&mut h, 60, "ready", |h| import_ready(h));
    h.app_mut()
        .dialog_mut::<ImportDialog>()
        .unwrap()
        .set_merge(true);
    click(&mut h, "OK");
    wait(&mut h, 60, "inserted", |h| {
        h.app().dialog::<ImportDialog>().is_none() && h.document().is_evaluated()
    });
    let bodies = mesh_bodies(h.document().evaluated());
    assert_eq!(bodies.len(), 1, "merged");
    assert_eq!(
        bodies[0].1.face_count(),
        2 * sphere().face_count(),
        "every face kept"
    );
}

// ---- Done-when 4: export ------------------------------------------------------------------

/// Drops a holed sphere, then opens the Export dialog and answers its Save dialog with `out`.
fn start_export(h: &mut AppHarness, fake: &FakeFileDialog, out: &Path) {
    fake.answer(Some(out.to_owned()));
    h.app_mut().request_command(EXPORT_CMD);
    h.kittest().step();
    h.kittest().step();
    assert!(h.app().dialog::<ExportDialog>().is_some());
    click(h, "OK");
}

fn export_gone(h: &mut AppHarness) -> bool {
    h.app().dialog::<ExportDialog>().is_none()
}

#[test]
fn export_with_holes_shows_the_report_and_repair_first_exports() {
    let e = env("repair");
    let src = e.dir.join("holed.stl");
    write_scene(&scene_of(vec![holed_sphere()]), Format::Stl, &src);
    let (mut h, fake) = harness();
    insert_by_drop(&mut h, &src);
    let out = e.dir.join("out.3mf");
    start_export(&mut h, &fake, &out);
    wait(&mut h, 60, "validity report", |h| {
        h.app()
            .dialog::<ExportDialog>()
            .is_some_and(ExportDialog::is_blocked)
    });
    h.kittest().step();
    let report = h
        .app()
        .dialog::<ExportDialog>()
        .unwrap()
        .report()
        .unwrap()
        .clone();
    assert_eq!(report.objects.len(), 1);
    assert!(report.objects[0].boundary_edges > 0, "{report:?}");
    assert!(!out.exists(), "nothing was written while blocked");
    assert!(
        h.kittest()
            .query_by_label_contains("boundary (hole) edges")
            .is_some(),
        "the report is on screen"
    );
    assert!(h.kittest().query_by_label("Export anyway").is_some());
    let before = h.document().design().timeline().nodes().count();
    click(&mut h, "Repair first");
    wait(&mut h, 120, "repaired and exported", export_gone);
    // A toast says where it went (checked at once: toasts time out).
    assert!(
        h.app()
            .notifications()
            .items()
            .iter()
            .any(|t| t.notification.title == "Exported"),
        "{:?}",
        h.app().notifications().items()
    );
    h.drive_until_idle();
    assert!(out.exists(), "exported after the repair");
    assert_eq!(
        h.document().design().timeline().nodes().count(),
        before + 1,
        "one mesh.repair feature"
    );
    assert!(timeline_kinds(&h).iter().any(|k| k == "mesh.repair"));
    // The file reads back as a closed solid.
    let back = amg_io::import(
        Source::Path(&out),
        None,
        &ImportOptions::default(),
        &amg_jobs::JobCtx::detached(),
    )
    .unwrap();
    let check =
        amg_io::check_export(&back.scene, Format::ThreeMf, &amg_jobs::JobCtx::detached()).unwrap();
    assert!(check.is_valid(), "{check:?}");
}

#[test]
fn export_anyway_writes_the_invalid_mesh() {
    let e = env("anyway");
    let src = e.dir.join("holed.stl");
    write_scene(&scene_of(vec![holed_sphere()]), Format::Stl, &src);
    let (mut h, fake) = harness();
    insert_by_drop(&mut h, &src);
    let out = e.dir.join("anyway.3mf");
    start_export(&mut h, &fake, &out);
    wait(&mut h, 60, "report", |h| {
        h.app()
            .dialog::<ExportDialog>()
            .is_some_and(ExportDialog::is_blocked)
    });
    h.kittest().step();
    click(&mut h, "Export anyway");
    wait(&mut h, 60, "exported", export_gone);
    assert!(out.exists());
    let warned = h
        .app()
        .notifications()
        .items()
        .iter()
        .any(|t| t.notification.title == "Exported with warnings");
    assert!(warned, "the writer's NotManifold warning is shown");
    assert_eq!(
        h.document().design().timeline().nodes().count(),
        1,
        "no repair"
    );
}

#[test]
fn export_dialog_writes_each_format_with_its_options() {
    let e = env("formats");
    let src = e.dir.join("ok.stl");
    write_scene(&scene_of(vec![sphere()]), Format::Stl, &src);
    let (mut h, fake) = harness();
    insert_by_drop(&mut h, &src);
    for (format, name) in [
        (Format::Stl, "x.stl"),
        (Format::Obj, "x.obj"),
        (Format::Ply, "x.ply"),
        (Format::ThreeMf, "x.3mf"),
    ] {
        let out = e.dir.join(name);
        fake.answer(Some(out.clone()));
        h.app_mut().request_command(EXPORT_CMD);
        h.kittest().step();
        h.kittest().step();
        {
            let d = h.app_mut().dialog_mut::<ExportDialog>().unwrap();
            d.set_format(format);
            d.set_unit(LengthUnit::Inch);
            if format == Format::ThreeMf {
                d.set_flavor(SlicerFlavor::Orca);
            }
        }
        click(&mut h, "OK");
        wait(&mut h, 60, name, export_gone);
        assert!(out.exists(), "{name}");
        let back = amg_io::import(
            Source::Path(&out),
            None,
            &ImportOptions::default(),
            &amg_jobs::JobCtx::detached(),
        )
        .unwrap();
        let got = extents(&back.scene.objects[0].mesh);
        let want = extents(&sphere());
        // Written in inches; STL/OBJ/PLY readers assume mm, 3MF declares the unit.
        let factor = if format == Format::ThreeMf {
            1.0
        } else {
            1.0 / 25.4
        };
        assert!(
            (got[0] - want[0] * factor).abs() < 1e-4,
            "{name}: {} vs {}",
            got[0],
            want[0] * factor
        );
        if format == Format::ThreeMf {
            let mut z = zip::ZipArchive::new(std::fs::File::open(&out).unwrap()).unwrap();
            assert!(
                z.by_name("Metadata/model_settings.config").is_ok(),
                "the Orca flavour writes model_settings.config"
            );
        }
    }
    // The last export format and the per-format unit are remembered.
    let p = prefs::load();
    assert_eq!(p.last_export_format, Some(Format::ThreeMf));
    assert_eq!(p.export_for(Format::Stl).unit, LengthUnit::Inch);
    assert_eq!(p.export_for(Format::ThreeMf).flavor, SlicerFlavor::Orca);
    // Opening the dialog again starts with them.
    h.app_mut().request_command(EXPORT_CMD);
    h.kittest().step();
    h.kittest().step();
    let d = h.app().dialog::<ExportDialog>().unwrap();
    assert_eq!(d.format(), Format::ThreeMf);
    assert_eq!(d.flavor(), SlicerFlavor::Orca);
    assert_eq!(d.unit(), LengthUnit::Inch);
}

#[test]
fn export_is_disabled_without_bodies() {
    let _e = env("nobody");
    let (mut h, _) = harness();
    h.app_mut().request_command(EXPORT_CMD);
    h.drive_until_idle();
    assert!(
        h.app().dialog::<ExportDialog>().is_none(),
        "no body, no dialog"
    );
}

#[test]
fn cancel_stops_an_export() {
    let e = env("cancel-export");
    let src = e.dir.join("ok.stl");
    write_scene(&scene_of(vec![sphere()]), Format::Stl, &src);
    let (mut h, fake) = harness();
    insert_by_drop(&mut h, &src);
    fake.answer(None);
    h.app_mut().request_command(EXPORT_CMD);
    h.kittest().step();
    h.kittest().step();
    click(&mut h, "OK");
    // The user cancels the Save dialog: back to the form, nothing written.
    for _ in 0..30 {
        h.kittest().step();
        std::thread::sleep(Duration::from_millis(2));
    }
    assert!(h.app().dialog::<ExportDialog>().is_some());
    assert!(h.app().dialog::<ExportDialog>().unwrap().path().is_none());
    click(&mut h, "Cancel");
    h.drive_until_idle();
    assert!(h.app().dialog::<ExportDialog>().is_none());
}

// ---- Done-when 2: a 1 GB STL --------------------------------------------------------------

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

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").as_deref() == Ok("1")
}

/// Writes a binary STL of `megabytes` MB (a displaced grid, 50 bytes per triangle) in `path`,
/// streaming. Returns the triangle count.
fn write_big_stl(path: &Path, megabytes: u64) -> u64 {
    use std::io::Write as _;
    let tris_wanted = megabytes * 1024 * 1024 / 50;
    let n = ((tris_wanted / 2) as f64).sqrt() as u64; // n × n quads
    let tris = 2 * n * n;
    let mut w = std::io::BufWriter::with_capacity(8 << 20, std::fs::File::create(path).unwrap());
    w.write_all(&[0u8; 80]).unwrap();
    w.write_all(&(tris as u32).to_le_bytes()).unwrap();
    let z = |i: u64, j: u64| ((i as f32) * 0.07).sin() * ((j as f32) * 0.05).cos() * 3.0;
    let v = |i: u64, j: u64| [i as f32 * 0.1, j as f32 * 0.1, z(i, j)];
    let mut rec = [0u8; 50];
    let mut put =
        |w: &mut std::io::BufWriter<std::fs::File>, a: [f32; 3], b: [f32; 3], c: [f32; 3]| {
            rec[..12].fill(0);
            for (k, p) in [a, b, c].iter().enumerate() {
                for (m, x) in p.iter().enumerate() {
                    let o = 12 + k * 12 + m * 4;
                    rec[o..o + 4].copy_from_slice(&x.to_le_bytes());
                }
            }
            rec[48] = 0;
            rec[49] = 0;
            w.write_all(&rec).unwrap();
        };
    for i in 0..n {
        for j in 0..n {
            put(&mut w, v(i, j), v(i + 1, j), v(i, j + 1));
            put(&mut w, v(i + 1, j), v(i + 1, j + 1), v(i, j + 1));
        }
    }
    w.flush().unwrap();
    tris
}

/// Done-when 2. Generates the STL in a temp dir (never committed). Run with:
/// `cargo nextest run -p amg-ui --test io_dialogs --run-ignored only -E 'test(timing_big_stl)'`
/// (`AMG_IO15_STL_MB` sets the size, default 1024; `AMG_STRICT_TIMING=1` asserts the bounds).
#[test]
#[ignore = "generates and imports a 1 GB STL (several GB of RAM); see the doc comment"]
fn timing_big_stl_import_keeps_frames_short_and_cancels_fast() {
    use tracing_subscriber::layer::SubscriberExt as _;
    let e = env("big");
    let mb: u64 = std::env::var("AMG_IO15_STL_MB")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1024);
    let file = e.dir.join("big.stl");
    let t = Instant::now();
    let tris = write_big_stl(&file, mb);
    let size = std::fs::metadata(&file).unwrap().len();
    println!("generated {size} B ({tris} triangles) in {:?}", t.elapsed());

    let times = FrameTimes::default();
    let subscriber = tracing_subscriber::registry().with(times.clone());
    let _guard = tracing::subscriber::set_default(subscriber);

    // 1. Import to the ready dialog: frame times while the job runs.
    let (mut h, _) = harness();
    times.0.lock().unwrap().clear();
    let start = Instant::now();
    drop_file(&mut h, &file);
    let mut frames = 0u32;
    wait(&mut h, 900, "big import", |h| {
        frames += 1;
        import_ready(h)
    });
    let total = start.elapsed();
    let mut ft = times.0.lock().unwrap().clone();
    ft.sort();
    let max = *ft.last().unwrap();
    let p50 = ft[ft.len() / 2];
    let over = ft
        .iter()
        .filter(|d| **d > Duration::from_millis(16))
        .count();
    println!(
        "IO-15 import of {size} B ({tris} triangles) in {total:?}: {} frames, UI frame p50 {p50:?}, max {max:?}, {over} over 16 ms",
        ft.len()
    );
    assert!(frames > 5, "the import spans many frames");
    let ready = h.app().dialog::<ImportDialog>().unwrap();
    assert_eq!(ready.loaded().unwrap().imported.report.triangles_read, tris);
    if strict_timing() {
        assert!(max < Duration::from_millis(16), "a frame took {max:?}");
    }
    click(&mut h, "Cancel");
    h.drive_until_idle();
    drop(h);

    // 2. Cancel while reading: time from the click until the job and dialog are gone.
    let (mut h, _) = harness();
    drop_file(&mut h, &file);
    wait(&mut h, 120, "progress", |h| {
        h.app()
            .jobs()
            .jobs()
            .iter()
            .any(|j| matches!(&j.state, amg_jobs::JobState::Running(p) if p.fraction > 0.15))
    });
    let t = Instant::now();
    click(&mut h, "Cancel");
    let mut frames_after = 1;
    while !(h.app().dialog::<ImportDialog>().is_none() && h.app().jobs().jobs().is_empty()) {
        h.kittest().step();
        frames_after += 1;
        assert!(
            t.elapsed() < Duration::from_secs(30),
            "cancel did not stop the import"
        );
        std::thread::sleep(Duration::from_millis(1));
    }
    let cancel = t.elapsed();
    println!("IO-15 cancel of the {size} B import: {cancel:?} ({frames_after} frames)");
    assert!(
        h.app().notifications().items().is_empty(),
        "a cancel shows no error"
    );
    if strict_timing() {
        assert!(
            cancel < Duration::from_millis(200),
            "cancel took {cancel:?}"
        );
    }
}

// ---- Done-when 5: the Phase 1 script --------------------------------------------------------

const SCRIPT_MODELS: [&str; 3] = [
    "mesh/real/thingi10k/132062.stl",
    "mesh/real/thingi10k/37825.stl",
    "mesh/real/thingi10k/36371.stl",
];

fn orca_exe() -> Option<PathBuf> {
    if let Some(p) = std::env::var_os("AMG_ORCA_SLICER") {
        return Some(p.into());
    }
    let default = PathBuf::from("D:/OrcaSlicer/orca-slicer.exe");
    default.is_file().then_some(default)
}

/// Drag STL → analyse → repair → export 3MF (Orca flavour) → `orca-slicer --info` reads it.
/// The GUI part of the script is a manual owner check (handoff report).
#[test]
fn phase1_script_drag_analyse_repair_export_orca_3mf() {
    let e = env("script");
    let orca = orca_exe();
    if orca.is_none() {
        assert!(
            std::env::var("AMG_REQUIRE_ORCA").as_deref() != Ok("1"),
            "AMG_REQUIRE_ORCA=1 but no OrcaSlicer was found"
        );
        eprintln!("no OrcaSlicer: the --info check is skipped");
    }
    let mut ran = 0;
    for rel in SCRIPT_MODELS {
        let Some(path) = amg_testkit::testdata(rel) else {
            continue;
        };
        let (mut h, fake) = harness();
        // 1. Drag the STL onto the window and insert it.
        insert_by_drop(&mut h, &path);
        let bodies = mesh_bodies(h.document().evaluated());
        assert_eq!(bodies.len(), 1, "{rel}");
        let (body, input) = (bodies[0].0, Arc::clone(&bodies[0].1));
        // 2. Analyse (Mesh Analysis panel).
        h.run_command(amg_ui::mesh::ANALYSIS_CMD);
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
        // 3. Repair (MESH-04) as the Repair command's feature.
        let root = h.document().design().root();
        h.app_mut().submit(amg_ui::CommitBatch {
            label: "Repair".into(),
            commands: vec![amg_doc::Command::AddFeature {
                at: None,
                feature: amg_doc::FeatureBox::new(amg_features::mesh::MeshRepairFeature::new(body)),
                context: root,
                name: None,
            }],
        });
        h.drive_until_idle();
        let repaired = mesh_bodies(h.document().evaluated());
        assert_eq!(repaired.len(), 1, "{rel}");
        // 4. Export 3MF, Orca flavour, through the dialog.
        let out = e.dir.join(format!(
            "{}-orca.3mf",
            Path::new(rel).file_stem().unwrap().to_string_lossy()
        ));
        fake.answer(Some(out.clone()));
        h.app_mut().request_command(EXPORT_CMD);
        h.kittest().step();
        h.kittest().step();
        h.app_mut()
            .dialog_mut::<ExportDialog>()
            .unwrap()
            .set_flavor(SlicerFlavor::Orca);
        click(&mut h, "OK");
        wait(&mut h, 120, "export or report", |h| {
            export_gone(h)
                || h.app()
                    .dialog::<ExportDialog>()
                    .is_some_and(ExportDialog::is_blocked)
        });
        let mut anyway = false;
        if h.app().dialog::<ExportDialog>().is_some() {
            // The repaired model is still not a closed solid: say so and export anyway.
            anyway = true;
            h.kittest().step();
            click(&mut h, "Export anyway");
            wait(&mut h, 120, "export", export_gone);
        }
        assert!(out.exists(), "{rel}: exported");
        // 5. The file reads back with every face, and Orca reads it.
        let back = amg_io::import(
            Source::Path(&out),
            None,
            &ImportOptions::default(),
            &amg_jobs::JobCtx::detached(),
        )
        .unwrap();
        let faces: usize = back.scene.objects.iter().map(|o| o.mesh.face_count()).sum();
        assert_eq!(faces, repaired[0].1.face_count(), "{rel}: every face kept");
        let mut info_line = String::from("(no Orca)");
        if let Some(exe) = &orca {
            let name = out.file_name().unwrap().to_string_lossy().into_owned();
            let o = std::process::Command::new(exe)
                .arg("--info")
                .arg(&out)
                .output()
                .unwrap();
            let text = String::from_utf8_lossy(&o.stdout).into_owned();
            assert!(o.status.success(), "{rel}: orca --info: {text}");
            assert!(text.contains(&format!("[{name}]")), "{rel}: {text}");
            let parts: usize = text
                .lines()
                .filter_map(|l| l.strip_prefix("number_of_parts ="))
                .map(|v| v.trim().parse::<usize>().unwrap())
                .sum();
            assert_eq!(parts, 1, "{rel}: Orca sees one part:\n{text}");
            let size: Vec<f64> = ["size_x", "size_y", "size_z"]
                .iter()
                .map(|k| {
                    text.lines()
                        .find_map(|l| l.strip_prefix(&format!("{k} =")))
                        .and_then(|v| v.trim().parse().ok())
                        .unwrap_or(f64::NAN)
                })
                .collect();
            let want = extents(&repaired[0].1);
            for k in 0..3 {
                assert!(
                    (size[k] - want[k]).abs() < 0.01 * want[k].max(1.0),
                    "{rel}: Orca size {size:?} vs {want:?}"
                );
            }
            info_line = format!("Orca --info: 1 part, size {size:?}");
        }
        eprintln!(
            "IO-15 script {rel}: {} faces in, {} after repair, {} bytes out{}; {info_line}",
            input.face_count(),
            repaired[0].1.face_count(),
            std::fs::metadata(&out).unwrap().len(),
            if anyway { " (Export anyway)" } else { "" }
        );
        ran += 1;
    }
    eprintln!(
        "IO-15 script ran on {ran} of {} models",
        SCRIPT_MODELS.len()
    );
    if std::env::var("AMG_REQUIRE_TESTDATA").as_deref() == Ok("1") {
        assert_eq!(ran, SCRIPT_MODELS.len(), "corpus required");
    }
}
