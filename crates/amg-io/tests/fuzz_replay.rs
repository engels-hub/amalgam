//! IO-16 Done-when 3 and 5: stable regression replay of the fuzz findings.
//!
//! Every file in `testdata/io/fuzz/<target>/` (a minimised libFuzzer artifact, one per fixed
//! bug) and in `fuzz/seeds/<target>/` (the committed seeds) runs through the same target body
//! the fuzzer uses (`fuzz/src/harness.rs`, compiled here by path), so fixed bugs stay fixed
//! without nightly. Un-fetched git-LFS pointer files are skipped unless
//! `AMG_REQUIRE_TESTDATA=1` (the rule of the `amg-mesh` corpus tests).

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    reason = "test harness; a failed replay is a test failure"
)]

#[path = "../../../fuzz/src/harness.rs"]
#[allow(dead_code, reason = "shared with the fuzz crate, which uses all of it")]
mod harness;

use std::path::{Path, PathBuf};
use std::time::Instant;

const LFS_PREFIX: &[u8] = b"version https://git-lfs.github.com/spec/v1";

fn require_testdata() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// Input files of a directory (absent → none), sorted; dot files and READMEs are notes.
fn inputs(dir: &Path) -> Vec<PathBuf> {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut v: Vec<PathBuf> = rd
        .map(|e| e.unwrap().path())
        .filter(|p| p.is_file())
        .filter(|p| {
            let n = p.file_name().unwrap().to_string_lossy();
            !n.starts_with('.') && !n.starts_with("README")
        })
        .collect();
    v.sort();
    v
}

/// Replays every input of `dirs` through its target; returns (replayed, skipped).
fn replay(root: &Path) -> (usize, usize) {
    let mut failures = Vec::new();
    let (mut ran, mut skipped) = (0, 0);
    for &(name, target) in harness::TARGETS {
        for path in inputs(&root.join(name)) {
            let bytes = std::fs::read(&path).unwrap();
            if bytes.len() <= 1024 && bytes.starts_with(LFS_PREFIX) {
                assert!(
                    !require_testdata(),
                    "{} is an un-fetched git-LFS pointer and AMG_REQUIRE_TESTDATA=1",
                    path.display()
                );
                skipped += 1;
                continue;
            }
            let t0 = Instant::now();
            let r = std::panic::catch_unwind(|| target(&bytes));
            let dt = t0.elapsed();
            ran += 1;
            match r {
                Ok(()) => eprintln!(
                    "ok   {name:<16} {:>8.1} ms  {}",
                    dt.as_secs_f64() * 1e3,
                    path.display()
                ),
                Err(e) => {
                    let msg = e
                        .downcast_ref::<String>()
                        .cloned()
                        .or_else(|| e.downcast_ref::<&str>().map(|s| (*s).to_owned()))
                        .unwrap_or_default();
                    failures.push(format!("{name}: {}: {msg}", path.display()));
                }
            }
        }
    }
    assert!(
        failures.is_empty(),
        "replay failures:\n{}",
        failures.join("\n")
    );
    (ran, skipped)
}

/// Every directory under `testdata/io/fuzz/` is a known target (a typo would never replay).
fn known_dirs(root: &Path) {
    let Ok(rd) = std::fs::read_dir(root) else {
        return;
    };
    for e in rd {
        let p = e.unwrap().path();
        if p.is_dir() {
            let n = p.file_name().unwrap().to_string_lossy().into_owned();
            assert!(
                harness::TARGETS.iter().any(|(t, _)| *t == n),
                "{} is not a fuzz target directory",
                p.display()
            );
        }
    }
}

#[test]
fn fuzz_artifacts_replay() {
    let root = repo().join("testdata/io/fuzz");
    known_dirs(&root);
    let (ran, skipped) = replay(&root);
    eprintln!("replayed {ran} artifacts, skipped {skipped} LFS pointers");
}

#[test]
fn fuzz_seeds_replay() {
    let root = repo().join("fuzz/seeds");
    known_dirs(&root);
    let (ran, _) = replay(&root);
    // Every target has seeds; an empty run means the path broke.
    assert!(ran >= harness::TARGETS.len(), "only {ran} seeds replayed");
}

/// The harness itself: a target panics on what it should catch.
#[test]
fn harness_targets_take_empty_and_garbage_input() {
    for &(_, target) in harness::TARGETS {
        target(b"");
        target(b"\0\xff solid\nfacet\nPK\x03\x04<model>");
    }
}

#[test]
fn stub_package_reads_back_a_model() {
    let model = br#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources><object id="1" type="model"><mesh>
  <vertices><vertex x="0" y="0" z="0"/><vertex x="1" y="0" z="0"/><vertex x="0" y="1" z="0"/><vertex x="0" y="0" z="1"/></vertices>
  <triangles><triangle v1="0" v2="2" v3="1"/><triangle v1="0" v2="1" v3="3"/><triangle v1="1" v2="2" v3="3"/><triangle v1="2" v2="0" v3="3"/></triangles>
 </mesh></object></resources>
 <build><item objectid="1"/></build>
</model>"#;
    let pkg = harness::stub_package(model);
    let i = amg_io::import(
        amg_io::Source::Bytes(&pkg),
        Some(amg_io::Format::ThreeMf),
        &amg_io::ImportOptions::default(),
        &amg_jobs::JobCtx::detached(),
    )
    .unwrap();
    assert_eq!(i.scene.objects.len(), 1);
    assert_eq!(i.scene.objects[0].mesh.face_count(), 4);
}

/// `roundtrip_scene` builds varied scenes from arbitrary bytes and they survive the trip.
#[test]
fn roundtrip_scene_on_generated_inputs() {
    let mut x = 0x9E37_79B9_7F4A_7C15_u64;
    for len in [0usize, 1, 7, 64, 300, 2000] {
        for _ in 0..8 {
            let bytes: Vec<u8> = (0..len)
                .map(|_| {
                    x ^= x << 13;
                    x ^= x >> 7;
                    x ^= x << 17;
                    x as u8
                })
                .collect();
            harness::roundtrip_scene(&bytes);
        }
    }
}

/// `ply/inch-overflow`: a coordinate that overflows on the way to mm is refused, for every
/// unit-less format and for 3MF; millimetre files keep the same value.
#[test]
fn unit_conversion_overflow_is_refused() {
    use amg_core::units::LengthUnit;
    use amg_io::{Format, ImportOptions, IoError, Source, UnitChoice, import};
    let obj = b"v 1e308 0 0\nv 0 0 0\nv 1 0 0\nf 1 2 3\n";
    let ctx = amg_jobs::JobCtx::detached();
    let mm = import(
        Source::Bytes(obj),
        Some(Format::Obj),
        &ImportOptions::default(),
        &ctx,
    )
    .unwrap();
    assert_eq!(
        mm.scene.objects[0].mesh.positions[0].x.to_bits(),
        1e308_f64.to_bits()
    );
    let inch = ImportOptions {
        unit: UnitChoice::Assume(LengthUnit::Inch),
        ..ImportOptions::default()
    };
    let e = import(Source::Bytes(obj), Some(Format::Obj), &inch, &ctx).unwrap_err();
    assert!(matches!(e, IoError::InvalidScene { .. }), "{e:?}");
}
