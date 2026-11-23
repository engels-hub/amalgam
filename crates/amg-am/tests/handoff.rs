//! AM-06: slicer hand-off: registry value parsing, detection, plate export (round trip through
//! the IO-03 reader), atomic write, launch, prune, and the export + spawn job.
//!
//! Nothing here depends on what is installed, except the tests gated by env vars:
//! `AMG_ORCA_SLICER` (the OrcaSlicer CLI check; `AMG_REQUIRE_ORCA=1` makes it required) and
//! `AMG_REQUIRE_REGISTRY_ORCA=1` (the registry must list an OrcaSlicer).
//! Timing is asserted only under `AMG_STRICT_TIMING=1`.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::print_stderr,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    missing_docs,
    reason = "tests"
)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use amg_am::AmError;
use amg_am::handoff::{
    DetectOptions, DetectSource, HandoffFormat, HandoffItem, HandoffPlate, SendToSlicerJob,
    SlicerConfig, classify_exe, classify_name, detect_slicers, detect_slicers_with, export_plate,
    handoff_path, launch, parse_registry_exe, parse_registry_exe_with, prune_handoff_dir,
};
use amg_am::machine::{Machine, MachineKey, MachineLibrary, SlicerKind};
use amg_am::place::{Placement, effective_transform};
use amg_core::glam::{DQuat, DVec2, DVec3};
use amg_core::{Diagnostic, Transform};
use amg_io::{ImportOptions, Source};
use amg_jobs::{CancelToken, JobCtx, JobSystem, JobSystemConfig};
use amg_mesh::{IndexedMesh, Rgba8};

// ------------------------------------------------------------------------- helpers

fn machine() -> Machine {
    let mut m = MachineLibrary::builtin()
        .get(&MachineKey("builtin:creality/ender-3-v3-ke-kcm".into()))
        .unwrap()
        .clone();
    // A non-zero slicer origin makes the offset visible.
    m.volume.slicer_origin = DVec2::new(110.0, 110.0);
    m
}

fn cuboid(x: f64, y: f64, z: f64) -> IndexedMesh {
    let t = amg_testkit::shapes::cube(DVec3::ZERO, 1.0);
    let positions = t
        .positions
        .iter()
        .map(|p| DVec3::new(p.x * x, p.y * y, p.z * z))
        .collect();
    IndexedMesh::new(positions, t.triangles).unwrap()
}

fn item(
    name: &str,
    mesh: IndexedMesh,
    rot: DQuat,
    xy: (f64, f64),
    filament: Option<u8>,
) -> HandoffItem {
    let mesh = Arc::new(mesh);
    let transform = effective_transform(&mesh, &Placement::on_plate(rot, DVec2::new(xy.0, xy.1)));
    HandoffItem {
        name: name.into(),
        mesh,
        transform,
        filament,
        color: filament
            .map(|f| Rgba8([[220, 50, 40, 255], [40, 110, 220, 255]][usize::from(f - 1)])),
    }
}

/// Three items on two filaments (1, 2, 1), rotated, at distinct plate positions.
fn three_items() -> Vec<HandoffItem> {
    three_items_at([(100.0, 90.0), (140.5, 120.25), (60.0, 150.0)])
}

fn three_items_at(at: [(f64, f64); 3]) -> Vec<HandoffItem> {
    vec![
        item(
            "Alpha",
            cuboid(20.0, 10.0, 8.0),
            DQuat::from_axis_angle(DVec3::Z, 0.4),
            at[0],
            Some(1),
        ),
        item(
            "Beta",
            cuboid(12.0, 12.0, 15.0),
            DQuat::from_axis_angle(DVec3::X, std::f64::consts::FRAC_PI_2),
            at[1],
            Some(2),
        ),
        item(
            "Gamma",
            cuboid(8.0, 30.0, 5.0),
            DQuat::IDENTITY,
            at[2],
            Some(1),
        ),
    ]
}

fn plate(m: &Machine, items: Vec<HandoffItem>) -> HandoffPlate<'_> {
    HandoffPlate {
        design_name: "Test design".into(),
        plate_name: "Plate 1".into(),
        machine: m,
        items,
        thumbnail_png: None,
    }
}

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// A harmless stand-in for a slicer: it writes its `{file}` argument into `out` and exits.
fn stand_in(out: &Path) -> SlicerConfig {
    #[cfg(windows)]
    {
        let exe = PathBuf::from(std::env::var_os("ComSpec").unwrap_or_else(|| "cmd.exe".into()));
        let mut cfg = SlicerConfig::new(SlicerKind::Custom, exe);
        cfg.args = vec![
            "/C".into(),
            "echo".into(),
            "{file}".into(),
            ">".into(),
            out.to_string_lossy().into_owned(),
        ];
        cfg
    }
    #[cfg(not(windows))]
    {
        let mut cfg = SlicerConfig::new(SlicerKind::Custom, "/bin/sh");
        cfg.args = vec![
            "-c".into(),
            "echo \"$0\" > \"$1\"".into(),
            "{file}".into(),
            out.to_string_lossy().into_owned(),
        ];
        cfg
    }
}

fn wait_for(path: &Path) -> String {
    let t0 = Instant::now();
    loop {
        if let Ok(s) = std::fs::read_to_string(path)
            && !s.trim().is_empty()
        {
            return s;
        }
        assert!(
            t0.elapsed() < Duration::from_secs(20),
            "{} never appeared",
            path.display()
        );
        std::thread::sleep(Duration::from_millis(20));
    }
}

// ------------------------------------------------------------------------- Done-when 1

#[test]
fn registry_value_of_the_owner_and_other_forms() {
    let exe = r"D:\OrcaSlicer\orca-slicer.exe";
    let exists = |p: &Path| {
        p.to_string_lossy()
            .replace('/', "\\")
            .eq_ignore_ascii_case(exe)
    };
    let cases = [
        // The owner's value: the install path is repeated behind a second drive prefix.
        r"D:\OrcaSlicer\D:\OrcaSlicer\orca-slicer.exe",
        r#""D:\OrcaSlicer\D:\OrcaSlicer\orca-slicer.exe""#,
        r#""D:\OrcaSlicer\D:\OrcaSlicer\orca-slicer.exe",0"#,
        r"D:\OrcaSlicer\D:\OrcaSlicer\orca-slicer.exe,0",
        r"D:\OrcaSlicer\orca-slicer.exe",
        r#""D:\OrcaSlicer\orca-slicer.exe""#,
        r#""D:\OrcaSlicer\orca-slicer.exe",-1"#,
        r"D:\OrcaSlicer\orca-slicer.exe,1",
        r#"  "D:\OrcaSlicer\orca-slicer.exe" --some-arg "x"  "#,
        r"D:\OrcaSlicer\ORCA-SLICER.EXE",
        "D:/OrcaSlicer/orca-slicer.exe",
    ];
    for v in cases {
        let got = parse_registry_exe_with(v, exists).unwrap_or_else(|| panic!("{v}"));
        assert!(exists(&got), "{v} -> {got:?}");
        // Exactly one drive prefix survives.
        assert_eq!(
            got.to_string_lossy().matches(':').count(),
            1,
            "{v} -> {got:?}"
        );
    }
    assert_eq!(
        parse_registry_exe_with(cases[0], exists).unwrap(),
        PathBuf::from(exe)
    );
    assert_eq!(
        parse_registry_exe_with(cases[2], exists).unwrap(),
        PathBuf::from(exe)
    );
    assert_eq!(
        parse_registry_exe_with(cases[8], exists).unwrap(),
        PathBuf::from(exe)
    );
}

#[test]
fn registry_value_without_an_exe_or_file_gives_none() {
    let any = |_: &Path| true;
    for v in [
        "",
        "   ",
        r"D:\OrcaSlicer\",
        r"D:\OrcaSlicer\orca-slicer.dll,0",
        r"D:\OrcaSlicer\orca-slicer.exe.bak",
        ",0",
        ".exe",
    ] {
        assert_eq!(parse_registry_exe_with(v, any), None, "{v:?}");
    }
    assert_eq!(
        parse_registry_exe_with(r"D:\Nope\nope.exe", |_| false),
        None
    );
    assert_eq!(parse_registry_exe(r"Z:\definitely\not\here\app.exe"), None);
}

#[test]
fn registry_value_of_a_real_file_resolves() {
    let dir = tempfile::tempdir().unwrap();
    let dir_path = dir.path().join("Orca Slicer");
    std::fs::create_dir_all(&dir_path).unwrap();
    let exe = dir_path.join("orca-slicer.exe");
    std::fs::write(&exe, b"MZ").unwrap();
    let s = exe.to_string_lossy();
    assert_eq!(parse_registry_exe(&format!("\"{s}\",0")), Some(exe.clone()));
    assert_eq!(parse_registry_exe(&format!("{s},0")), Some(exe.clone()));
    // Doubled, with spaces in the path.
    let doubled = format!("{}\\{}", dir_path.display(), s);
    assert_eq!(parse_registry_exe(&doubled), Some(exe));
}

#[test]
fn names_and_exes_are_classified() {
    assert_eq!(classify_name("OrcaSlicer"), Some(SlicerKind::OrcaSlicer));
    assert_eq!(
        classify_name("Orca Slicer 2.4"),
        Some(SlicerKind::OrcaSlicer)
    );
    assert_eq!(classify_name("Bambu Studio"), Some(SlicerKind::BambuStudio));
    assert_eq!(classify_name("PrusaSlicer"), Some(SlicerKind::PrusaSlicer));
    assert_eq!(classify_name("CHITUBOX Basic"), Some(SlicerKind::Chitubox));
    assert_eq!(classify_name("Notepad++"), None);
    assert_eq!(classify_name("OrcaSlicer Uninstall"), None);
    assert_eq!(
        classify_exe(Path::new(r"C:\x\orca-slicer.exe")),
        Some(SlicerKind::OrcaSlicer)
    );
    assert_eq!(
        classify_exe(Path::new(r"C:\x\prusa-slicer-console.exe")),
        None
    );
}

// ------------------------------------------------------------------------- detection

#[test]
fn detection_finds_known_dirs_and_path_sorted_and_deduplicated() {
    let root = tempfile::tempdir().unwrap();
    let make = |rel: &str| -> PathBuf {
        let p = root.path().join(rel);
        std::fs::create_dir_all(p.parent().unwrap()).unwrap();
        std::fs::write(&p, b"MZ").unwrap();
        p
    };
    let orca = make("pf/OrcaSlicer/orca-slicer.exe");
    let chitubox = make("pf/CHITUBOX64/CHITUBOX.exe");
    let prusa_on_path = make("bin/prusa-slicer.exe");
    let orca_dir = orca.parent().unwrap().to_path_buf();
    let _ = make("pf/Notes/notes.exe");
    let path = std::env::join_paths([
        root.path().join("bin"),
        orca_dir,
        root.path().join("missing"),
    ])
    .unwrap();
    let found = detect_slicers_with(&DetectOptions {
        registry: false,
        known_roots: vec![root.path().join("pf"), root.path().join("nope")],
        path: Some(path),
    });
    let kinds: Vec<_> = found.iter().map(|i| (i.kind, i.source)).collect();
    assert_eq!(
        kinds,
        [
            // Found in a known dir and on PATH: one entry, the better source.
            (SlicerKind::OrcaSlicer, DetectSource::KnownDir),
            (SlicerKind::PrusaSlicer, DetectSource::Path),
            (SlicerKind::Chitubox, DetectSource::KnownDir),
        ]
    );
    assert_eq!(found[0].exe, orca);
    assert_eq!(found[1].exe, prusa_on_path);
    assert_eq!(found[2].exe, chitubox);
    assert!(found.iter().all(|i| i.version.is_none()));
}

#[test]
fn detection_with_nothing_configured_is_empty() {
    assert!(detect_slicers_with(&DetectOptions::default()).is_empty());
}

#[test]
fn system_detection_only_lists_existing_unique_files() {
    let found = detect_slicers();
    for i in &found {
        assert!(i.exe.is_file(), "{:?}", i.exe);
        // An uninstaller is never the slicer (the registry DisplayIcon of most installers).
        let name = i.exe.file_name().unwrap().to_string_lossy().to_lowercase();
        assert!(!name.contains("uninst"), "{:?}", i.exe);
    }
    let mut keys: Vec<String> = found
        .iter()
        .map(|i| i.exe.to_string_lossy().to_lowercase())
        .collect();
    let n = keys.len();
    keys.sort();
    keys.dedup();
    assert_eq!(keys.len(), n, "duplicates in {found:?}");
    eprintln!("detected on this machine: {found:#?}");
}

/// Owner's machine: `AMG_REQUIRE_REGISTRY_ORCA=1` demands an OrcaSlicer from the registry.
#[test]
fn owner_registry_lists_orcaslicer_when_required() {
    if std::env::var("AMG_REQUIRE_REGISTRY_ORCA").is_ok_and(|v| v == "1") {
        let found = detect_slicers();
        let orca = found
            .iter()
            .find(|i| i.kind == SlicerKind::OrcaSlicer && i.source == DetectSource::Registry)
            .unwrap_or_else(|| panic!("no registry OrcaSlicer in {found:#?}"));
        assert!(orca.exe.is_file());
        eprintln!("registry OrcaSlicer: {orca:?}");
    }
}

// ------------------------------------------------------------------------- paths and prune

#[test]
fn handoff_path_is_sanitised_and_timestamped() {
    let t = UNIX_EPOCH + Duration::from_millis(1_700_000_000_123);
    let p = handoff_path(
        Path::new("out"),
        "My: design/v2?",
        "Plate  1",
        t,
        HandoffFormat::ThreeMf,
    );
    assert_eq!(
        p,
        PathBuf::from("out").join("My__design_v2_Plate__1_20231114-221320-123.3mf")
    );
    let q = handoff_path(Path::new("o"), "", "...", t, HandoffFormat::Stl);
    assert_eq!(
        q,
        PathBuf::from("o").join("plate_plate_20231114-221320-123.stl")
    );
    // Leap day.
    let leap = UNIX_EPOCH + Duration::from_secs(1_709_164_800); // 2024-02-29 00:00:00
    assert!(
        handoff_path(Path::new("o"), "a", "b", leap, HandoffFormat::Stl)
            .to_string_lossy()
            .contains("20240229-000000-000")
    );
}

#[test]
fn prune_keeps_the_newest_and_only_touches_3mf_and_stl() {
    let dir = tempfile::tempdir().unwrap();
    let base = SystemTime::now() - Duration::from_secs(10_000);
    let mut names = Vec::new();
    for k in 0..25u64 {
        let ext = if k % 2 == 0 { "3mf" } else { "STL" };
        let p = dir.path().join(format!("f{k:02}.{ext}"));
        let f = std::fs::File::create(&p).unwrap();
        f.set_modified(base + Duration::from_secs(k * 10)).unwrap();
        names.push(p);
    }
    std::fs::write(dir.path().join("notes.txt"), b"x").unwrap();
    std::fs::write(dir.path().join("x.3mf.tmp"), b"x").unwrap();
    std::fs::create_dir(dir.path().join("sub.3mf")).unwrap();
    assert_eq!(prune_handoff_dir(dir.path(), 20).unwrap(), 5);
    for (k, p) in names.iter().enumerate() {
        assert_eq!(p.exists(), k >= 5, "{}", p.display());
    }
    assert!(dir.path().join("notes.txt").exists());
    assert!(dir.path().join("x.3mf.tmp").exists());
    assert!(dir.path().join("sub.3mf").exists());
    // Idempotent, and a missing directory is fine.
    assert_eq!(prune_handoff_dir(dir.path(), 20).unwrap(), 0);
    assert_eq!(prune_handoff_dir(&dir.path().join("gone"), 20).unwrap(), 0);
    assert_eq!(prune_handoff_dir(dir.path(), 0).unwrap(), 20);
}

// ------------------------------------------------------------------------- Done-when 2

/// Plate-frame vertex positions of every instance of a re-imported scene, per object name:
/// slicer coordinates plus the slicer origin.
fn plate_positions(scene: &amg_io::Scene, origin: DVec2) -> BTreeMap<String, Vec<DVec3>> {
    let shift = DVec3::new(origin.x, origin.y, 0.0);
    scene
        .instances()
        .into_iter()
        .map(|(i, t)| {
            let o = &scene.objects[i as usize];
            let pts = o
                .mesh
                .positions
                .iter()
                .map(|p| t.transform_point3(*p) + shift)
                .collect();
            (o.name.clone().unwrap_or_default(), pts)
        })
        .collect()
}

fn sorted_key(p: &DVec3) -> [i64; 3] {
    [p.x, p.y, p.z].map(|v| (v * 1e3).round() as i64)
}

fn assert_same_plate_frame(items: &[HandoffItem], scene: &amg_io::Scene, origin: DVec2, tol: f64) {
    let got = plate_positions(scene, origin);
    assert_eq!(got.len(), items.len());
    for it in items {
        let have = got
            .get(&it.name)
            .unwrap_or_else(|| panic!("no {} in {:?}", it.name, got.keys()));
        assert_eq!(have.len(), it.mesh.positions.len(), "{}", it.name);
        let mut want: Vec<DVec3> = it
            .mesh
            .positions
            .iter()
            .map(|p| it.transform.transform_point3(*p))
            .collect();
        let mut have = have.clone();
        want.sort_by_key(sorted_key);
        have.sort_by_key(sorted_key);
        for (a, b) in want.iter().zip(&have) {
            assert!(
                (*a - *b).abs().max_element() < tol,
                "{}: {a:?} vs {b:?}",
                it.name
            );
        }
    }
}

fn reimport(path: &Path) -> amg_io::Scene {
    amg_io::import(Source::Path(path), None, &ImportOptions::default(), &ctx())
        .unwrap()
        .scene
}

fn names_and_filaments(scene: &amg_io::Scene) -> Vec<(String, Option<u8>)> {
    let mut v: Vec<_> = scene
        .instances()
        .iter()
        .map(|(i, _)| {
            let o = &scene.objects[*i as usize];
            (o.name.clone().unwrap_or_default(), o.filament)
        })
        .collect();
    v.sort();
    v
}

#[test]
fn three_item_plate_round_trips_in_orca_flavour_with_filaments() {
    let m = machine();
    let items = three_items();
    let p = plate(&m, items.clone());
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("plate.3mf");
    let r = export_plate(
        &p,
        HandoffFormat::ThreeMf,
        SlicerKind::OrcaSlicer,
        &path,
        &ctx(),
    )
    .unwrap();
    assert_eq!((r.objects, r.triangles), (3, 36));
    assert_eq!(r.bytes, std::fs::metadata(&path).unwrap().len());
    assert_eq!(r.path, path);
    let scene = reimport(&path);
    assert_same_plate_frame(&items, &scene, m.volume.slicer_origin, 1e-6);
    assert_eq!(
        names_and_filaments(&scene),
        [
            ("Alpha".to_owned(), Some(1)),
            ("Beta".to_owned(), Some(2)),
            ("Gamma".to_owned(), Some(1))
        ]
    );
    // The reader keeps `plates` only for projects with more than one; a single plate is
    // the whole scene.
    assert!(scene.plates.len() <= 1);
}

#[test]
fn slicer_origin_offset_is_applied_to_x_and_y_only() {
    let mut m = machine();
    m.volume.slicer_origin = DVec2::new(-25.5, 40.0);
    let items = vec![item(
        "Solo",
        cuboid(10.0, 10.0, 10.0),
        DQuat::IDENTITY,
        (30.0, 50.0),
        None,
    )];
    let p = plate(&m, items);
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("solo.3mf");
    export_plate(
        &p,
        HandoffFormat::ThreeMf,
        SlicerKind::Custom,
        &path,
        &ctx(),
    )
    .unwrap();
    let scene = reimport(&path);
    let bb = scene.world_aabb();
    // Plate frame: x 25..35, y 45..55, z 0..10. Slicer frame: minus (-25.5, 40).
    assert!(
        (bb.min.x - 50.5).abs() < 1e-6 && (bb.max.x - 60.5).abs() < 1e-6,
        "{bb:?}"
    );
    assert!(
        (bb.min.y - 5.0).abs() < 1e-6 && (bb.max.y - 15.0).abs() < 1e-6,
        "{bb:?}"
    );
    assert!(
        bb.min.z.abs() < 1e-9 && (bb.max.z - 10.0).abs() < 1e-9,
        "{bb:?}"
    );
}

#[test]
fn core_3mf_and_stl_keep_positions_and_the_core_3mf_keeps_filaments() {
    let m = machine();
    let items = three_items();
    let p = plate(&m, items.clone());
    let dir = tempfile::tempdir().unwrap();
    // PrusaSlicer / Chitubox / custom get the core 3MF.
    for kind in [
        SlicerKind::PrusaSlicer,
        SlicerKind::Chitubox,
        SlicerKind::Custom,
    ] {
        let path = dir.path().join(format!("core-{kind:?}.3mf"));
        export_plate(&p, HandoffFormat::ThreeMf, kind, &path, &ctx()).unwrap();
        let scene = reimport(&path);
        assert_same_plate_frame(&items, &scene, m.volume.slicer_origin, 1e-6);
        let f: Vec<_> = names_and_filaments(&scene)
            .into_iter()
            .map(|x| x.1)
            .collect();
        assert_eq!(f, [Some(1), Some(2), Some(1)], "{kind:?}");
    }
    // STL: one merged solid, same plate-frame points (f32 on disk).
    let path = dir.path().join("plate.stl");
    let r = export_plate(&p, HandoffFormat::Stl, SlicerKind::Chitubox, &path, &ctx()).unwrap();
    assert_eq!(r.triangles, 36);
    let scene = reimport(&path);
    let shift = DVec3::new(m.volume.slicer_origin.x, m.volume.slicer_origin.y, 0.0);
    let mut have: Vec<DVec3> = scene.objects[0]
        .mesh
        .positions
        .iter()
        .map(|p| *p + shift)
        .collect();
    let mut want: Vec<DVec3> = items
        .iter()
        .flat_map(|it| {
            it.mesh
                .positions
                .iter()
                .map(|p| it.transform.transform_point3(*p))
        })
        .collect();
    have.sort_by_key(sorted_key);
    want.sort_by_key(sorted_key);
    assert_eq!(have.len(), want.len());
    for (a, b) in have.iter().zip(&want) {
        assert!((*a - *b).abs().max_element() < 1e-3, "{a:?} vs {b:?}");
    }
}

// ------------------------------------------------------------------------- atomic write

#[test]
fn failed_exports_leave_the_old_file_and_no_temp_file() {
    let m = machine();
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("keep.3mf");
    std::fs::write(&path, b"previous content").unwrap();

    let mut items = three_items();
    items[1].transform = Transform::from_scale(DVec3::new(2.0, 1.0, 1.0));
    let err = export_plate(
        &plate(&m, items),
        HandoffFormat::ThreeMf,
        SlicerKind::OrcaSlicer,
        &path,
        &ctx(),
    )
    .unwrap_err();
    assert!(matches!(err, AmError::NotRigid), "{err}");

    let err = export_plate(
        &plate(&m, vec![]),
        HandoffFormat::ThreeMf,
        SlicerKind::OrcaSlicer,
        &path,
        &ctx(),
    )
    .unwrap_err();
    assert!(matches!(err, AmError::EmptyPlate), "{err}");

    let token = CancelToken::new();
    token.cancel();
    let err = export_plate(
        &plate(&m, three_items()),
        HandoffFormat::ThreeMf,
        SlicerKind::OrcaSlicer,
        &path,
        &JobCtx::with_token(token),
    )
    .unwrap_err();
    assert!(matches!(err, AmError::Cancelled), "{err}");

    assert_eq!(std::fs::read(&path).unwrap(), b"previous content");
    let names: Vec<_> = std::fs::read_dir(dir.path())
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    assert_eq!(names, ["keep.3mf"]);

    // A good export replaces the file in one step and leaves no `.tmp`.
    export_plate(
        &plate(&m, three_items()),
        HandoffFormat::ThreeMf,
        SlicerKind::OrcaSlicer,
        &path,
        &ctx(),
    )
    .unwrap();
    assert!(std::fs::read(&path).unwrap().starts_with(b"PK"));
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
}

#[test]
fn export_creates_the_directory() {
    let m = machine();
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("a").join("b").join("p.3mf");
    export_plate(
        &plate(&m, three_items()),
        HandoffFormat::ThreeMf,
        SlicerKind::OrcaSlicer,
        &path,
        &ctx(),
    )
    .unwrap();
    assert!(path.is_file());
}

// ------------------------------------------------------------------------- Done-when 4

#[test]
fn a_missing_exe_gives_slicer_not_found_with_a_hint() {
    let missing = PathBuf::from(r"Z:\no\such\orca-slicer.exe");
    let cfg = SlicerConfig::new(SlicerKind::OrcaSlicer, &missing);
    let err = launch(&cfg, Path::new("x.3mf")).unwrap_err();
    assert!(
        matches!(&err, AmError::SlicerNotFound(p) if *p == missing),
        "{err}"
    );
    assert!(err.to_string().contains("not found"), "{err}");
    let hint = err.hint().unwrap();
    assert!(hint.contains("Preferences"), "{hint}");

    // Through the job as well, before any file is written.
    let m = machine();
    let dir = tempfile::tempdir().unwrap();
    let job = SendToSlicerJob::new(&plate(&m, three_items()), cfg, dir.path().join("out"));
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let err = jobs.spawn(job).wait_blocking().unwrap_err();
    assert!(format!("{err:?}").contains("SlicerNotFound"), "{err:?}");
    assert!(!dir.path().join("out").exists());
}

#[test]
fn a_file_that_cannot_run_gives_launch_error() {
    let dir = tempfile::tempdir().unwrap();
    let junk = dir.path().join("not-a-program.exe");
    std::fs::write(&junk, b"this is not an executable").unwrap();
    let cfg = SlicerConfig::new(SlicerKind::Custom, &junk);
    let err = launch(&cfg, Path::new("x.3mf")).unwrap_err();
    assert!(matches!(err, AmError::Launch(_)), "{err}");
    assert!(err.hint().is_some());
}

#[test]
fn launch_substitutes_the_file_and_returns_without_waiting() {
    let dir = tempfile::tempdir().unwrap();
    let out = dir.path().join("arg.txt");
    let cfg = stand_in(&out);
    let t0 = Instant::now();
    let pid = launch(&cfg, Path::new("Plate_1.3mf")).unwrap();
    eprintln!("launch returned after {} ms", t0.elapsed().as_millis());
    assert!(pid > 0);
    assert!(wait_for(&out).contains("Plate_1.3mf"));
}

#[test]
fn defaults_per_slicer() {
    let o = SlicerConfig::new(SlicerKind::OrcaSlicer, "a");
    assert_eq!(o.format, HandoffFormat::ThreeMf);
    assert_eq!(o.args, ["{file}"]);
    assert_eq!(
        SlicerConfig::new(SlicerKind::Chitubox, "a").format,
        HandoffFormat::Stl
    );
}

// ------------------------------------------------------------------------- Done-when 5

#[test]
fn send_job_exports_prunes_and_launches_without_blocking_the_caller() {
    let m = machine();
    let dir = tempfile::tempdir().unwrap();
    let out = dir.path().join("arg.txt");
    let hand = dir.path().join("handoff");
    std::fs::create_dir_all(&hand).unwrap();
    let base = SystemTime::now() - Duration::from_secs(10_000);
    for k in 0..22u64 {
        let f = std::fs::File::create(hand.join(format!("old{k:02}.3mf"))).unwrap();
        f.set_modified(base + Duration::from_secs(k)).unwrap();
    }
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let job = SendToSlicerJob::new(&plate(&m, three_items()), stand_in(&out), hand.clone());
    let t0 = Instant::now();
    let mut handle = jobs.spawn(job);
    let spawn_us = t0.elapsed().as_micros();
    // The caller stays free to poll while the job runs.
    let res = loop {
        if let Some(r) = handle.try_take() {
            break r.unwrap();
        }
        std::thread::sleep(Duration::from_millis(1));
    };
    assert!(spawn_us < 50_000, "spawn took {spawn_us} µs");
    assert!(res.pid > 0);
    assert_eq!(res.handoff.objects, 3);
    assert!(res.handoff.path.starts_with(&hand));
    assert!(res.handoff.path.extension().is_some_and(|e| e == "3mf"));
    // 22 old + 1 new = 23, newest 20 kept.
    assert_eq!(res.pruned, 3);
    assert_eq!(std::fs::read_dir(&hand).unwrap().count(), 20);
    assert!(res.handoff.path.is_file());
    let arg = wait_for(&out);
    let file_name = res.handoff.path.file_name().unwrap().to_string_lossy();
    assert!(arg.contains(&*file_name), "{arg}");
}

#[test]
fn cancelling_the_job_before_launch_starts_nothing() {
    let m = machine();
    let dir = tempfile::tempdir().unwrap();
    let out = dir.path().join("arg.txt");
    let token = CancelToken::new();
    token.cancel();
    let job = SendToSlicerJob::new(
        &plate(&m, three_items()),
        stand_in(&out),
        dir.path().join("h"),
    );
    let err = amg_jobs::Job::run(job, &JobCtx::with_token(token)).unwrap_err();
    assert!(matches!(err, AmError::Cancelled), "{err}");
    std::thread::sleep(Duration::from_millis(200));
    assert!(!out.exists());
}

/// Done-when 5: a 1 M-triangle plate exports and spawns in < 3 s as a job. The stand-in
/// replaces the slicer GUI. Asserted under `AMG_STRICT_TIMING=1`, printed always.
#[test]
fn one_million_triangle_plate_sends_in_under_three_seconds() {
    use amg_mesh::corpus::bench::{BenchSize, benchmark_sphere};
    let m = machine();
    let mesh = Arc::new(benchmark_sphere(BenchSize::M1, 7));
    assert!(mesh.face_count() > 1_000_000);
    let transform = effective_transform(
        &mesh,
        &Placement::on_plate(
            DQuat::from_axis_angle(DVec3::new(1.0, 2.0, 3.0).normalize(), 0.7),
            DVec2::new(110.0, 110.0),
        ),
    );
    let items = vec![HandoffItem {
        name: "Sphere".into(),
        mesh,
        transform,
        filament: Some(1),
        color: None,
    }];
    let dir = tempfile::tempdir().unwrap();
    let out = dir.path().join("arg.txt");
    let jobs = JobSystem::new(JobSystemConfig::default()).unwrap();
    let mut best = f64::INFINITY;
    for run in 0..3 {
        let _ = std::fs::remove_file(&out);
        let job = SendToSlicerJob::new(
            &plate(&m, items.clone()),
            stand_in(&out),
            dir.path().join("h"),
        );
        let t0 = Instant::now();
        let res = jobs.spawn(job).wait_blocking().unwrap();
        let secs = t0.elapsed().as_secs_f64();
        eprintln!(
            "run {run}: export + spawn of {} tris: {secs:.3} s ({} bytes, export {} ms, {} warnings)",
            res.handoff.triangles,
            res.handoff.bytes,
            res.handoff.millis,
            res.handoff.warnings.len()
        );
        wait_for(&out);
        best = best.min(secs);
    }
    eprintln!("1 M-triangle export + spawn, best of 3: {best:.3} s");
    if strict_timing() {
        assert!(best < 3.0, "{best:.3} s");
    }
}

// ------------------------------------------------------------------------- OrcaSlicer CLI

fn orca_exe() -> Option<PathBuf> {
    if let Some(p) = std::env::var_os("AMG_ORCA_SLICER") {
        return Some(p.into());
    }
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path)
        .flat_map(|d| [d.join("orca-slicer.exe"), d.join("orca-slicer")])
        .find(|p| p.is_file())
}

/// The CLI (never the GUI) reads the hand-off file: three objects with their names, and the
/// `--export-3mf` re-export keeps the objects and their extruders. Skipped without OrcaSlicer;
/// `AMG_REQUIRE_ORCA=1` makes it required.
#[test]
fn orca_cli_reads_the_hand_off_file() {
    let Some(exe) = orca_exe() else {
        assert!(
            std::env::var("AMG_REQUIRE_ORCA").map_or(true, |v| v != "1"),
            "AMG_REQUIRE_ORCA=1 but no OrcaSlicer found"
        );
        eprintln!("skipped: set AMG_ORCA_SLICER");
        return;
    };
    // Slicer origin 0: all items are inside Orca's default bed, so the CLI has no reason to
    // arrange them.
    let mut m = machine();
    m.volume.slicer_origin = DVec2::ZERO;
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("handoff.3mf");
    export_plate(
        &plate(&m, three_items()),
        HandoffFormat::ThreeMf,
        SlicerKind::OrcaSlicer,
        &path,
        &ctx(),
    )
    .unwrap();
    let info = std::process::Command::new(&exe)
        .arg("--info")
        .arg(&path)
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&info.stdout);
    assert_eq!(text.matches("[handoff.3mf]").count(), 3, "{text}");
    let outdir = dir.path().join("re");
    std::fs::create_dir_all(&outdir).unwrap();
    let st = std::process::Command::new(&exe)
        .arg("--outputdir")
        .arg(&outdir)
        .arg("--arrange")
        .arg("0")
        .arg("--export-3mf")
        .arg("re.3mf")
        .arg(&path)
        .output()
        .unwrap();
    assert!(st.status.success(), "{:?}", st.status);
    let re = reimport(&outdir.join("re.3mf"));
    let f = names_and_filaments(&re);
    eprintln!("Orca re-export: {f:?}");
    assert_eq!(
        f,
        [
            ("Alpha".to_owned(), Some(1)),
            ("Beta".to_owned(), Some(2)),
            ("Gamma".to_owned(), Some(1))
        ]
    );
    // Positions: the re-export keeps every object's bounding-box centre in the slicer frame
    // (± 0.1 mm, the manual-check tolerance); `--arrange 0` because the CLI arranges otherwise.
    let origin = m.volume.slicer_origin;
    let shift = DVec3::new(origin.x, origin.y, 0.0);
    let mut worst: f64 = 0.0;
    for it in three_items() {
        let want = bbox_centre(&it.mesh, &it.transform) - shift;
        let (i, t) = re
            .instances()
            .into_iter()
            .find(|(i, _)| re.objects[*i as usize].name.as_deref() == Some(it.name.as_str()))
            .unwrap();
        let got = bbox_centre(&re.objects[i as usize].mesh, &t);
        worst = worst.max((want - got).abs().max_element());
        eprintln!("{}: want {want:?} got {got:?}", it.name);
    }
    eprintln!("Orca re-export worst centre deviation: {worst:.6} mm");
    assert!(worst < 0.1, "{worst}");
}

fn bbox_centre(mesh: &IndexedMesh, t: &Transform) -> DVec3 {
    let mut lo = DVec3::splat(f64::INFINITY);
    let mut hi = DVec3::splat(f64::NEG_INFINITY);
    for p in &mesh.positions {
        let q = t.transform_point3(*p);
        lo = lo.min(q);
        hi = hi.max(q);
    }
    (lo + hi) * 0.5
}

// ------------------------------------------------------------------------- manual check fixtures

/// Writes the hand-off files for the owner's manual check (Done-when 3) when
/// `AMG_AM06_FIXTURE_DIR` is set: the three-item, two-filament plate on the Ender-3 V3 KE KCM
/// (3MF for OrcaSlicer) and on the Elegoo Mars 3 Pro (3MF and STL for Chitubox).
#[test]
fn write_manual_check_fixtures() {
    let Some(dir) = std::env::var_os("AMG_AM06_FIXTURE_DIR").map(PathBuf::from) else {
        return;
    };
    std::fs::create_dir_all(&dir).unwrap();
    let lib = MachineLibrary::builtin();
    let ke = lib
        .get(&MachineKey("builtin:creality/ender-3-v3-ke-kcm".into()))
        .unwrap();
    let mars = lib
        .get(&MachineKey("builtin:elegoo/mars-3-pro".into()))
        .unwrap();
    let write = |m: &Machine, name: &str, f: HandoffFormat, k: SlicerKind| {
        // Positions inside each bed.
        let at = if std::ptr::eq(m, mars) {
            [(30.0, 25.0), (70.0, 45.0), (110.0, 65.0)]
        } else {
            [(100.0, 90.0), (140.5, 120.25), (60.0, 150.0)]
        };
        let mut p = plate(m, three_items_at(at));
        p.design_name = "AM-06 check".into();
        export_plate(&p, f, k, &dir.join(name), &ctx()).unwrap();
    };
    write(
        ke,
        "am06-orca-3items.3mf",
        HandoffFormat::ThreeMf,
        SlicerKind::OrcaSlicer,
    );
    write(
        mars,
        "am06-chitubox-3items.3mf",
        HandoffFormat::ThreeMf,
        SlicerKind::Chitubox,
    );
    write(
        mars,
        "am06-chitubox-3items.stl",
        HandoffFormat::Stl,
        SlicerKind::Chitubox,
    );
    eprintln!("fixtures in {}", dir.display());
}
