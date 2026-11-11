//! IO-03 Done-when 1 and 4: the 3MF Consortium conformance suites 2, 3, 5 and 6.
//!
//! - Every `P_*` case in scope loads in strict mode; every `N_*` case fails with a specific
//!   `IoError`. Out-of-scope cases are listed with a reason in
//!   `testdata/io/3mf-conformance.toml`, and their count is printed.
//! - Done-when 4: the parallel scanner gives exactly the scene of the pure quick-xml path on
//!   every case that loads (strict and normal mode).
//!
//! The suites are fetch-only (about 600 MB): `testdata/io/fetch-3mf-suites.ps1` checks out the
//! pinned commit into `testdata/io/fetch/3mf-test-suites/`, or point `AMG_3MF_SUITES` at a
//! checkout. Without them the tests skip, unless `AMG_REQUIRE_TESTDATA=1`.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::print_stderr,
    reason = "test code; a failed expectation is a test failure"
)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use amg_io::threemf::{Tuning, read_with};
use amg_io::{ImportOptions, Imported, IoError, Scene, Source};
use amg_jobs::JobCtx;
use rayon::prelude::*;

/// Suite directories in scope (S12 IO-03: core, production, materials).
const SUITES: [&str; 4] = [
    "suite2_core_prod_matl",
    "suite3_core",
    "suite5_core_prod",
    "suite6_core_matl",
];

fn require() -> bool {
    std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1")
}

fn suites_dir() -> Option<PathBuf> {
    let d = std::env::var_os("AMG_3MF_SUITES")
        .map(PathBuf::from)
        .unwrap_or_else(|| amg_testkit::testdata_dir().join("io/fetch/3mf-test-suites"));
    if d.join("suite3_core").is_dir() {
        return Some(d);
    }
    assert!(
        !require(),
        "3MF conformance suites not found at {} (run testdata/io/fetch-3mf-suites.ps1)",
        d.display()
    );
    eprintln!(
        "SKIP: 3MF conformance suites not found at {} (run testdata/io/fetch-3mf-suites.ps1 or set AMG_3MF_SUITES)",
        d.display()
    );
    None
}

#[derive(serde::Deserialize)]
struct AllowList {
    case: Vec<Allowed>,
}

#[derive(serde::Deserialize)]
struct Allowed {
    file: String,
    reason: String,
}

fn allow_list() -> BTreeMap<String, String> {
    let p = amg_testkit::testdata_dir().join("io/3mf-conformance.toml");
    let s = std::fs::read_to_string(&p).expect("allow-list");
    let a: AllowList = toml::from_str(&s).expect("allow-list syntax");
    a.case.into_iter().map(|c| (c.file, c.reason)).collect()
}

/// Every case file: (path relative to the suites dir, positive?).
fn cases(root: &Path) -> Vec<(String, bool)> {
    let mut out = Vec::new();
    for s in SUITES {
        let Ok(dirs) = std::fs::read_dir(root.join(s)) else {
            continue;
        };
        let mut dirs: Vec<_> = dirs.flatten().collect();
        dirs.sort_by_key(|d| d.file_name());
        for d in dirs {
            let dn = d.file_name().to_string_lossy().into_owned();
            let positive = dn.starts_with("positive");
            if !positive && !dn.starts_with("negative") {
                continue;
            }
            let mut files: Vec<_> = std::fs::read_dir(d.path()).unwrap().flatten().collect();
            files.sort_by_key(|f| f.file_name());
            for f in files {
                let n = f.file_name().to_string_lossy().into_owned();
                if n.to_ascii_lowercase().ends_with(".3mf") {
                    out.push((format!("{s}/{dn}/{n}"), positive));
                }
            }
        }
    }
    out
}

fn opts(strict: bool) -> ImportOptions {
    let mut o = ImportOptions::default();
    o.threemf.strict = strict;
    o
}

fn read(path: &Path, strict: bool, reference_only: bool) -> Result<Imported, IoError> {
    let tuning = Tuning {
        reference_only,
        // Small chunks so that the chunk seams are exercised on these small files.
        chunk_bytes: if reference_only { 4 << 20 } else { 4096 },
    };
    read_with(
        Source::Path(path),
        &opts(strict),
        &JobCtx::detached(),
        &tuning,
    )
}

/// A comparable digest of a scene: everything the reader produces.
fn digest(s: &Scene) -> String {
    let mut out = String::new();
    for (k, o) in s.objects.iter().enumerate() {
        out.push_str(&format!(
            "o{k} {:?} {:?} {:?} {:?} {:?} {:?}\n",
            o.name, o.kind, o.material, o.filament, o.part_subtype, o.source_id
        ));
        out.push_str(&format!("{:?}\n{:?}\n", o.mesh.positions, o.mesh.tris));
        out.push_str(&format!(
            "{:?}\n{:?}\n",
            o.face_materials, o.mesh.attrs.corner_uvs
        ));
    }
    out.push_str(&format!("{:?}\n", s.materials));
    for (i, t) in s.instances() {
        out.push_str(&format!("i{i} {t:?}\n"));
    }
    out.push_str(&format!("{:?}\n", s.metadata));
    out
}

/// A "specific" error: not an I/O failure, a cancel or a panic-like catch-all.
fn specific(e: &IoError) -> bool {
    matches!(
        e,
        IoError::Malformed { .. }
            | IoError::LimitExceeded { .. }
            | IoError::UnsupportedRequiredExtension { .. }
            | IoError::Encrypted
            | IoError::Empty
            | IoError::Mesh(_)
    )
}

fn variant(e: &IoError) -> &'static str {
    match e {
        IoError::Malformed { .. } => "Malformed",
        IoError::LimitExceeded { .. } => "LimitExceeded",
        IoError::UnsupportedRequiredExtension { .. } => "UnsupportedRequiredExtension",
        IoError::Encrypted => "Encrypted",
        IoError::Empty => "Empty",
        IoError::Mesh(_) => "Mesh",
        _ => "other",
    }
}

/// Done-when 1.
#[test]
fn consortium_suites_positive_load_and_negative_fail_in_strict_mode() {
    let Some(root) = suites_dir() else { return };
    let allow = allow_list();
    let all = cases(&root);
    assert!(
        all.len() > 900,
        "only {} cases found under {}",
        all.len(),
        root.display()
    );
    let results: Vec<(String, bool, Result<(), String>)> = all
        .par_iter()
        .map(|(rel, positive)| {
            let r = read(&root.join(rel), true, false);
            let outcome = match (positive, r) {
                (true, Ok(_)) => Ok(()),
                (true, Err(e)) => Err(format!("{e}")),
                (false, Ok(_)) => Err("loaded".to_owned()),
                (false, Err(e)) if specific(&e) => Ok(()),
                (false, Err(e)) => Err(format!("unspecific error {e:?}")),
            };
            (rel.clone(), *positive, outcome)
        })
        .collect();
    let mut failures = Vec::new();
    let (mut pos, mut neg, mut allowed, mut stale) = (0, 0, 0, Vec::new());
    for (rel, positive, outcome) in &results {
        match (outcome, allow.get(rel)) {
            (Ok(()), None) => {
                if *positive {
                    pos += 1;
                } else {
                    neg += 1;
                }
            }
            (Ok(()), Some(_)) => stale.push(rel.clone()),
            (Err(_), Some(_)) => allowed += 1,
            (Err(m), None) => failures.push(format!("{rel}: {m}")),
        }
    }
    let mut hist: BTreeMap<&str, u32> = BTreeMap::new();
    for (rel, positive) in &all {
        if !positive
            && !allow.contains_key(rel)
            && let Err(e) = read(&root.join(rel), true, false)
        {
            *hist.entry(variant(&e)).or_default() += 1;
        }
    }
    eprintln!(
        "3MF conformance: {} cases; {pos} positive load, {neg} negative fail as expected; \
         {allowed} allow-listed (out of scope); negative error variants {hist:?}",
        all.len()
    );
    for s in &stale {
        eprintln!("NOTE: allow-listed case now behaves as expected: {s}");
    }
    assert!(
        failures.is_empty(),
        "{} unexpected results:\n{}",
        failures.len(),
        failures.join("\n")
    );
    let unknown: Vec<_> = allow
        .keys()
        .filter(|k| !all.iter().any(|(r, _)| r == *k))
        .collect();
    assert!(
        unknown.is_empty(),
        "allow-list names unknown cases: {unknown:?}"
    );
}

/// Done-when 4: scanner vs pure quick-xml parsing on every fixture that loads.
#[test]
fn scanner_matches_pure_quick_xml_on_every_suite_case() {
    let Some(root) = suites_dir() else { return };
    let all = cases(&root);
    let compared: usize = all
        .par_iter()
        .map(|(rel, _)| {
            let path = root.join(rel);
            let mut n = 0;
            for strict in [false, true] {
                let fast = read(&path, strict, false);
                let slow = read(&path, strict, true);
                match (&fast, &slow) {
                    (Ok(f), Ok(s)) => {
                        assert_eq!(digest(&f.scene), digest(&s.scene), "{rel} strict={strict}");
                        assert_eq!(
                            f.report.warnings, s.report.warnings,
                            "{rel} strict={strict}"
                        );
                        n += 1;
                    }
                    (Err(f), Err(s)) => {
                        assert_eq!(variant(f), variant(s), "{rel} strict={strict}: {f} vs {s}");
                    }
                    _ => panic!(
                        "{rel} strict={strict}: scanner {:?} vs quick-xml {:?}",
                        fast.as_ref().err(),
                        slow.as_ref().err()
                    ),
                }
            }
            n
        })
        .sum();
    eprintln!(
        "scanner vs quick-xml: {compared} loads compared over {} cases",
        all.len()
    );
    assert!(compared > 900);
}
