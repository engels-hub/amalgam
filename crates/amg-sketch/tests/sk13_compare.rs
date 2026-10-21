//! SK-13 acceptance for the comparison harness and the native prototype: the prototype
//! solves the ADR-007 decision cases, the corpus, drags D4/D7 without branch flips, and is
//! deterministic. With `--features slvs` / `ezpz-bench` the other options run too.

#![allow(clippy::unwrap_used, clippy::print_stdout, reason = "tests")]

use std::path::PathBuf;

use amg_sketch::corpus::compare::{self, ContenderBackend, metrics, native::Native};
use amg_sketch::corpus::{self, Case, CaseStatus, GroupKind, run_corpus};

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../testdata/sketch/corpus")
}

fn decision_cases() -> Vec<Case> {
    let mut v: Vec<(u32, Case)> = corpus::load_corpus(&root())
        .unwrap()
        .cases
        .into_iter()
        .filter_map(|c| {
            let n: u32 = c
                .tags
                .iter()
                .find_map(|t| t.strip_prefix("decision:D"))?
                .parse()
                .ok()?;
            Some((n, c))
        })
        .collect();
    v.sort_by_key(|(n, _)| *n);
    v.into_iter().map(|(_, c)| c).collect()
}

#[test]
fn native_solves_all_decision_cases() {
    let cases = decision_cases();
    assert_eq!(cases.len(), 10);
    let mut solved = 0;
    for case in &cases {
        let m = metrics::measure(&Native::default(), case, 3, 1);
        println!("{} {:?} {:?} {:?}", case.id, m.verdict, m.status, m.groups);
        if m.passed() {
            solved += 1;
        }
    }
    // Decision rule §5.4 (1): at least 9 of 10.
    assert!(solved >= 9, "native solved {solved}/10");
}

#[test]
fn native_drags_d4_and_d7_smoothly() {
    for case in decision_cases()
        .iter()
        .filter(|c| c.id.ends_with("d04_polyline50") || c.id.ends_with("d07_four_bar"))
    {
        for opt in [Native::default(), Native::sqp_drag()] {
            let m = metrics::measure(&opt, case, 1, 1);
            let d = m.drag.unwrap();
            println!("{} {} {d:?}", case.id, opt.label);
            assert_eq!(d.converged_frames, d.frames, "{}", case.id);
            assert!(d.jitter.is_finite(), "{}: {}", case.id, d.jitter);
            // D7's cursor path stays outside the reachable range (|b − o1| > 12), so the
            // linkage is pulled into the toggle on the first frame (where it may pass to the other
            // branch once) and stays there. Feasible SQP then keeps the branch; weighted min-norm
            // restarts from the cursor every frame and flips repeatedly (benchmark finding).
            if opt.drag == amg_sketch::solve::proto::DragMethod::Sqp {
                assert!(d.flips.unwrap_or(0) <= 1, "{}: branch flipped", case.id);
                assert!(d.jitter < 1.0, "{}: {}", case.id, d.jitter);
            }
        }
    }
}

#[test]
fn native_diagnoses_d5_and_d6() {
    let cases = decision_cases();
    let d5 = metrics::measure(&Native::default(), &cases[4], 1, 0);
    assert_eq!(d5.status, Some(CaseStatus::ConvergedRedundant));
    let g = d5.groups.unwrap();
    // Parallel (7) and equal (8) are each redundant with the two horizontals (1, 3).
    assert!(g.iter().all(|g| g.kind == GroupKind::Redundant), "{g:?}");
    let sets: Vec<Vec<u32>> = g.into_iter().map(|g| g.constraints).collect();
    assert!(
        sets.contains(&vec![1, 3, 7]) && sets.contains(&vec![1, 3, 8]),
        "{sets:?}"
    );

    let d6 = metrics::measure(&Native::default(), &cases[5], 1, 0);
    assert!(!d6.status.unwrap().is_converged());
    let g = d6.groups.unwrap();
    assert!(
        g.iter()
            .any(|g| g.kind == GroupKind::Conflicting && g.constraints == [2, 3, 4]),
        "{g:?}"
    );
}

#[test]
fn native_is_deterministic_on_d9() {
    let case = &decision_cases()[8];
    let run = || {
        let mut b = case.build().unwrap();
        let mut l = compare::Contender::load(&Native::fast(), &b).unwrap();
        l.solve();
        b.apply(&l.values()).unwrap();
        b.sketch
            .params()
            .as_slice()
            .iter()
            .map(|v| v.to_bits())
            .collect::<Vec<_>>()
    };
    let first = run();
    for _ in 0..100 {
        assert_eq!(run(), first);
    }
}

#[test]
fn native_passes_the_corpus() {
    let c = corpus::load_corpus(&root()).unwrap();
    for opt in [Native::default(), Native::dogleg()] {
        let report = run_corpus(&mut ContenderBackend(opt), &c.cases);
        println!("{report}");
        let ran = report.results.len() - report.skipped();
        assert_eq!(report.skipped(), 0);
        // Recorded in docs/notes/sketch-solver-benchmark.md; SK-03's bar is ≥ 95 %.
        assert!(
            report.pass_rate() >= 0.95,
            "{}: {}/{ran}",
            report.backend,
            report.passed()
        );
    }
}

#[cfg(feature = "slvs")]
#[test]
fn libslvs_runs_the_decision_cases() {
    let cases = decision_cases();
    let mut solved = 0;
    for case in &cases {
        let m = metrics::measure(&compare::slvs::Slvs, case, 1, 1);
        println!("{} {:?} {:?} {:?}", case.id, m.skipped, m.verdict, m.status);
        if m.passed() {
            solved += 1;
        }
    }
    assert!(solved >= 6, "libslvs solved {solved}/10");
}

#[cfg(feature = "ezpz-bench")]
#[test]
fn ezpz_runs_the_decision_cases() {
    let cases = decision_cases();
    let mut ran = 0;
    for case in &cases {
        let m = metrics::measure(&compare::ezpz::Ezpz, case, 1, 1);
        println!("{} {:?} {:?} {:?}", case.id, m.skipped, m.verdict, m.status);
        if m.skipped.is_none() {
            ran += 1;
        }
    }
    assert!(ran >= 6, "ezpz ran {ran}/10");
}
