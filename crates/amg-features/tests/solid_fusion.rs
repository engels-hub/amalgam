//! SOL-29 Done-when 1: the Fusion-parity harness. The owner's designs in
//! `$AMG_TESTDATA_DIR/fusion/` are rebuilt by the scripts in `solid_fusion/designs.rs` and
//! compared with their STEP exports (volume and area within 1e-4 relative, bbox within
//! 1e-3 mm). Skipped when the folder is missing. A self-made design runs the same path on a
//! STEP file written into a temporary folder. Layout and script format:
//! `solid_fusion/README.md`. Geometry needs the `occt` feature (AMG_OCCT_DIR).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp,
    clippy::print_stdout,
    clippy::print_stderr,
    missing_docs,
    reason = "tests"
)]
#![cfg_attr(
    not(feature = "occt"),
    allow(dead_code, reason = "the rebuild path needs the kernel")
)]

mod solid_common;

mod solid_fusion {
    pub mod compare;
    pub mod designs;
    pub mod harness;
    #[cfg(feature = "occt")]
    pub mod selfmade;
}

use std::path::PathBuf;

use solid_fusion::compare::{Metrics, compare};
use solid_fusion::designs::DESIGNS;
use solid_fusion::harness::{Outcome, Report, step_files};

/// An empty scratch folder for one test.
fn scratch(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR"))
        .join("solid-fusion")
        .join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

// ---- the owner's designs ----------------------------------------------------------------------

#[test]
fn owner_fusion_designs_match_their_step_exports() {
    let Some(root) = std::env::var_os("AMG_TESTDATA_DIR").map(PathBuf::from) else {
        eprintln!("SKIP: AMG_TESTDATA_DIR is not set");
        return;
    };
    let dir = root.join("fusion");
    if step_files(&dir).is_none() {
        eprintln!("SKIP: {} is absent", dir.display());
        return;
    }
    #[cfg(feature = "occt")]
    {
        let report = solid_fusion::harness::run(&dir, DESIGNS).expect("the folder exists");
        for line in report.lines() {
            println!("{line}");
        }
        let failed = report.failures(solid_fusion::harness::strict());
        assert!(
            failed.is_empty(),
            "{failed:?}\n{}",
            report.lines().join("\n")
        );
    }
    #[cfg(not(feature = "occt"))]
    eprintln!("SKIP: rebuilding needs the `occt` feature");
}

#[test]
fn design_names_are_unique_file_stems() {
    let mut names: Vec<&str> = DESIGNS.iter().map(|d| d.name).collect();
    names.sort_unstable();
    for n in &names {
        assert!(
            !n.is_empty() && !n.contains(['/', '\\', '.']),
            "{n:?} is not a file stem"
        );
    }
    let count = names.len();
    names.dedup();
    assert_eq!(names.len(), count, "a design is listed twice");
}

// ---- folder and verdicts (every build) --------------------------------------------------------

#[test]
fn a_missing_folder_is_a_skip() {
    let dir = scratch("missing");
    assert_eq!(step_files(&dir.join("fusion")), None);
    #[cfg(feature = "occt")]
    assert!(solid_fusion::harness::run(&dir.join("fusion"), DESIGNS).is_none());
}

#[test]
fn only_step_files_directly_in_the_folder_are_designs() {
    let dir = scratch("listing");
    for name in ["b.STP", "a.step", "a.md", "a.png", "c.step.bak"] {
        std::fs::write(dir.join(name), "x").unwrap();
    }
    std::fs::create_dir_all(dir.join("sub.step")).unwrap();
    let names: Vec<String> = step_files(&dir).unwrap().into_iter().map(|f| f.0).collect();
    assert_eq!(names, ["a", "b"]);
}

#[test]
fn unscripted_and_missing_files_fail_only_when_strict() {
    let report = Report {
        designs: vec![
            ("a".into(), Outcome::Unscripted),
            ("b".into(), Outcome::MissingStep),
            ("c".into(), Outcome::NotFetched),
            ("d".into(), Outcome::Pending("needs solid.hole")),
            ("e".into(), Outcome::Failed("volume".into())),
        ],
    };
    assert_eq!(report.failures(false), ["e"]);
    assert_eq!(report.failures(true), ["a", "b", "c", "e"]);
    assert_eq!(report.lines().len(), 5);
}

// ---- the bounds (every build) -----------------------------------------------------------------

fn reference() -> Metrics {
    Metrics {
        solids: 1,
        volume: 1000.0,
        area: 600.0,
        min: [0.0; 3],
        max: [10.0; 3],
        tolerance: 1e-7,
    }
}

#[test]
fn bounds_are_1e4_relative_and_1e3_mm() {
    let r = reference();
    let inside = Metrics {
        volume: 1000.0 * (1.0 + 0.9e-4),
        area: 600.0 * (1.0 - 0.9e-4),
        max: [10.0009, 10.0, 10.0],
        ..r
    };
    let c = compare(&r, &inside);
    assert!(c.mismatches.is_empty() && !c.loose, "{c:?}");

    for (outside, what) in [
        (
            Metrics {
                volume: 1000.0 * (1.0 + 1.1e-4),
                ..r
            },
            "volume",
        ),
        (
            Metrics {
                area: 600.0 * (1.0 - 1.1e-4),
                ..r
            },
            "area",
        ),
        (
            Metrics {
                min: [0.0, -0.0011, 0.0],
                ..r
            },
            "bbox min.y",
        ),
        (Metrics { solids: 2, ..r }, "solids"),
        (
            Metrics {
                volume: f64::NAN,
                ..r
            },
            "volume",
        ),
    ] {
        let c = compare(&r, &outside);
        assert_eq!(c.mismatches.len(), 1, "{what}: {c:?}");
        assert!(c.mismatches[0].starts_with(what), "{what}: {c:?}");
    }
}

#[test]
fn loose_parts_are_held_to_area_times_tolerance() {
    // 0.01 mm tolerance: volume bound 600 × 0.01 = 6 mm³, bbox bound 0.01 mm; area unchanged.
    let r = Metrics {
        tolerance: 0.01,
        ..reference()
    };
    let near = Metrics {
        volume: 1005.9,
        max: [10.0099, 10.0, 10.0],
        tolerance: 1e-7,
        ..r
    };
    let c = compare(&r, &near);
    assert!(c.mismatches.is_empty() && c.loose, "{c:?}");
    let far = Metrics {
        volume: 1006.1,
        area: 600.1,
        max: [10.0101, 10.0, 10.0],
        ..r
    };
    assert_eq!(compare(&r, &far).mismatches.len(), 3);
}

// ---- the self-made design (OCCT) --------------------------------------------------------------

#[cfg(feature = "occt")]
mod end_to_end {
    use std::path::Path;

    use amg_doc::Document;
    use amg_features::io::{ExportSelection, brep_scene_from_document};
    use amg_io::brep::{StepSchema, export_step};
    use amg_jobs::JobCtx;

    use super::scratch;
    use super::solid_fusion::compare::compare;
    use super::solid_fusion::harness::{Design, Outcome, Script, run, step_metrics};
    use super::solid_fusion::selfmade;

    /// Writes `<dir>/<name>.step` from the script with our exporter.
    fn write_step(dir: &Path, name: &str, script: fn(&mut Document)) {
        amg_features::io::link();
        let mut doc = Document::new(name);
        script(&mut doc);
        let scene =
            brep_scene_from_document(doc.design(), doc.evaluated(), &ExportSelection::AllVisible)
                .unwrap();
        export_step(
            &scene,
            StepSchema::Ap242,
            &dir.join(format!("{name}.step")),
            &JobCtx::detached(),
        )
        .unwrap();
    }

    const fn design(name: &'static str, script: fn(&mut Document)) -> Design {
        Design {
            name,
            script: Script::Build(script),
            explained: None,
        }
    }

    #[test]
    fn the_selfmade_design_matches_its_step_export() {
        let dir = scratch("selfmade").join("fusion");
        std::fs::create_dir_all(&dir).unwrap();
        write_step(&dir, "bracket", selfmade::bracket);

        // The reference file holds the analytic part.
        let got = step_metrics(&dir.join("bracket.step")).unwrap();
        let want = selfmade::bracket_analytic();
        assert_eq!(got.solids, 1);
        assert!(
            (got.volume - want.volume).abs() <= 1e-9 * want.volume,
            "{} vs {}",
            got.volume,
            want.volume
        );
        let c = compare(
            &super::Metrics {
                area: got.area,
                tolerance: got.tolerance,
                ..want
            },
            &got,
        );
        assert!(c.mismatches.is_empty(), "{c:?}");

        let report = run(&dir, &[design("bracket", selfmade::bracket)]).unwrap();
        println!("{}", report.lines().join("\n"));
        let Some(Outcome::Matched(c)) = report.outcome("bracket") else {
            panic!("{report:?}");
        };
        assert!(
            c.volume_rel <= 1e-9 && c.area_rel <= 1e-9 && c.bbox_mm <= 1e-6,
            "{c:?}"
        );
        assert!(report.failures(true).is_empty());
    }

    #[test]
    fn wrong_scripts_fail_and_explained_differences_are_reported() {
        let dir = scratch("differences").join("fusion");
        std::fs::create_dir_all(&dir).unwrap();
        for name in ["taller", "bore", "broken", "known", "unscripted"] {
            write_step(&dir, name, selfmade::bracket);
        }
        std::fs::write(dir.join("garbage.step"), "not a STEP file").unwrap();
        std::fs::write(
            dir.join("pointer.step"),
            "version https://git-lfs.github.com/spec/v1\noid sha256:0\nsize 1\n",
        )
        .unwrap();
        let designs = [
            design("taller", selfmade::bracket_taller),
            design("bore", selfmade::bracket_wider_bore),
            design("broken", selfmade::bracket_broken),
            Design {
                explained: Some("the bore is deliberately wider"),
                ..design("known", selfmade::bracket_wider_bore)
            },
            design("garbage", selfmade::bracket),
            design("pointer", selfmade::bracket),
            design("absent", selfmade::bracket),
            Design {
                name: "later",
                script: Script::Pending("needs solid.hole (SOL-12)"),
                explained: None,
            },
        ];
        let report = run(&dir, &designs).unwrap();
        println!("{}", report.lines().join("\n"));
        let failed = |name: &str| match report.outcome(name) {
            Some(Outcome::Failed(why)) => why.clone(),
            other => panic!("{name}: {other:?}"),
        };
        let taller = failed("taller");
        assert!(taller.contains("bbox max.z"), "{taller}");
        let bore = failed("bore");
        assert!(bore.contains("volume") && !bore.contains("bbox"), "{bore}");
        assert!(failed("broken").contains("feature "));
        assert!(failed("garbage").contains("STEP import"));
        assert!(matches!(
            report.outcome("known"),
            Some(Outcome::Explained(c, _)) if !c.mismatches.is_empty()
        ));
        assert!(matches!(
            report.outcome("unscripted"),
            Some(Outcome::Unscripted)
        ));
        assert!(matches!(
            report.outcome("pointer"),
            Some(Outcome::NotFetched)
        ));
        assert!(matches!(
            report.outcome("absent"),
            Some(Outcome::MissingStep)
        ));
        assert!(matches!(report.outcome("later"), Some(Outcome::Pending(_))));
        assert_eq!(
            report.failures(false),
            ["bore", "broken", "garbage", "taller"]
        );
    }
}
