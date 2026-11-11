//! IO-03 Done-when 2: slicer-produced fixtures (`testdata/io/slicer/fixtures.toml`, made by
//! `scripts/io/make_slicer_fixtures.ps1`) load with the right object count, names, build
//! transforms and per-face filament histogram.
//!
//! Un-fetched LFS pointers and an empty manifest skip, unless `AMG_REQUIRE_TESTDATA=1`.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::print_stderr,
    clippy::print_stdout,
    reason = "test code; a failed expectation is a test failure"
)]

use std::collections::BTreeMap;

use amg_io::{Format, ImportOptions, Imported, IoWarning, NO_SLOT, Source, import};
use amg_jobs::JobCtx;

#[derive(serde::Deserialize)]
struct Manifest {
    fixture: Vec<Fixture>,
}

#[derive(serde::Deserialize)]
struct Fixture {
    file: String,
    slicer: String,
    objects: usize,
    names: Vec<String>,
    translations: Vec<[f64; 3]>,
    #[serde(default)]
    painted: Vec<[u64; 2]>,
    #[serde(default)]
    approximated: u64,
}

fn require() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn manifest() -> Manifest {
    let p = amg_testkit::testdata_dir().join("io/slicer/fixtures.toml");
    toml::from_str(&std::fs::read_to_string(p).expect("manifest")).expect("manifest syntax")
}

fn load(file: &str) -> Option<Imported> {
    let path = amg_testkit::testdata(&format!("io/slicer/{file}"))?;
    Some(
        import(
            Source::Path(&path),
            Some(Format::ThreeMf),
            &ImportOptions::default(),
            &JobCtx::detached(),
        )
        .unwrap_or_else(|e| panic!("{file}: {e}")),
    )
}

/// Filament → painted face count over every placed object.
fn painted_histogram(r: &Imported) -> BTreeMap<u64, u64> {
    let mut h = BTreeMap::new();
    for (i, _) in r.scene.instances() {
        let o = &r.scene.objects[i as usize];
        if let Some(f) = &o.face_materials {
            for &s in f.iter().filter(|&&s| s != NO_SLOT) {
                if let Some(fil) = r.scene.materials[usize::from(s)].filament {
                    *h.entry(u64::from(fil)).or_default() += 1;
                }
            }
        }
    }
    h
}

#[test]
fn slicer_fixtures_load_with_expected_structure() {
    let m = manifest();
    if m.fixture.is_empty() {
        assert!(
            !require(),
            "testdata/io/slicer/fixtures.toml lists no fixture"
        );
        eprintln!("SKIP: no slicer fixtures listed (run scripts/io/make_slicer_fixtures.ps1)");
        return;
    }
    for f in &m.fixture {
        let Some(r) = load(&f.file) else { continue };
        let s = &r.scene;
        assert_eq!(
            s.roots.len(),
            f.objects,
            "{} ({}): objects",
            f.file,
            f.slicer
        );
        let names: Vec<String> = s
            .roots
            .iter()
            .map(|n| n.name.clone().unwrap_or_default())
            .collect();
        assert_eq!(names, f.names, "{}: names", f.file);
        for (k, (n, t)) in s.roots.iter().zip(&f.translations).enumerate() {
            let d = n.transform.translation - amg_core::Vec3::new(t[0], t[1], t[2]);
            assert!(
                d.length() < 1e-6,
                "{}: root {k} at {:?}, expected {t:?}",
                f.file,
                n.transform.translation
            );
        }
        if !f.painted.is_empty() {
            let want: BTreeMap<u64, u64> = f.painted.iter().map(|p| (p[0], p[1])).collect();
            assert_eq!(painted_histogram(&r), want, "{}: painted faces", f.file);
            let approx = r
                .report
                .warnings
                .iter()
                .find_map(|w| match w {
                    IoWarning::PaintApproximated { triangles } => Some(*triangles),
                    _ => None,
                })
                .unwrap_or(0);
            assert_eq!(approx, f.approximated, "{}: approximated triangles", f.file);
        }
    }
}

/// Prints manifest entries for every `*.3mf` in `testdata/io/slicer/` (a helper for
/// `make_slicer_fixtures.ps1`; check the values against the slicer before pasting them).
#[test]
fn print_manifest_entries() {
    let dir = amg_testkit::testdata_dir().join("io/slicer");
    let Ok(rd) = std::fs::read_dir(&dir) else {
        return;
    };
    let mut files: Vec<_> = rd
        .flatten()
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.ends_with(".3mf"))
        .collect();
    files.sort();
    for file in files {
        let Some(r) = load(&file) else { continue };
        let names: Vec<String> = r
            .scene
            .roots
            .iter()
            .map(|n| format!("{:?}", n.name.clone().unwrap_or_default()))
            .collect();
        let tr: Vec<String> = r
            .scene
            .roots
            .iter()
            .map(|n| {
                format!(
                    "[{}, {}, {}]",
                    n.transform.translation.x, n.transform.translation.y, n.transform.translation.z
                )
            })
            .collect();
        let painted: Vec<String> = painted_histogram(&r)
            .iter()
            .map(|(k, v)| format!("[{k}, {v}]"))
            .collect();
        println!(
            "[[fixture]]\nfile = {file:?}\nslicer = \"?\"\nobjects = {}\nnames = [{}]\ntranslations = [{}]\npainted = [{}]\n",
            r.scene.roots.len(),
            names.join(", "),
            tr.join(", "),
            painted.join(", ")
        );
    }
}
