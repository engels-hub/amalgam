//! SK-14 acceptance: the corpus is complete (size per set, expected status and DOF per case,
//! licence notices), valid (every stored solution satisfies its constraints, the stated DOF
//! matches the Jacobian rank there) and the runner reports pass/fail per case.

#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    reason = "test helpers; a failure is a test failure"
)]

use std::path::PathBuf;

use amg_sketch::corpus::{
    self, CorpusBackend, ReferenceCheck, ReferenceSolver, SourceSet, Verdict, run_corpus,
};

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../testdata/sketch/corpus")
}

fn load() -> corpus::LoadedCorpus {
    corpus::load_corpus(&root()).unwrap()
}

#[test]
fn corpus_has_the_required_size() {
    let c = load();
    assert!(c.skipped.is_empty(), "skipped files: {:?}", c.skipped);
    assert!(c.count(SourceSet::SolveSpace) >= 60);
    assert!(c.count(SourceSet::PlaneGcs) >= 20);
    assert!(c.count(SourceSet::Ezpz) >= 20);
    assert!(c.count(SourceSet::Own) >= 20);
    assert!(c.cases.len() >= 120, "{} cases", c.cases.len());
}

#[test]
fn every_case_has_status_and_dof() {
    for case in &load().cases {
        let ex = &case.expected;
        // `validate` ties `dof` to converged statuses; make the contract explicit here.
        assert_eq!(
            ex.dof.is_some(),
            ex.status.is_converged(),
            "{}: dof/status",
            case.id
        );
        assert!(!case.description.is_empty(), "{}: no description", case.id);
        assert!(!case.source.licence.is_empty(), "{}: no licence", case.id);
    }
}

#[test]
fn every_source_folder_has_notice_and_licence() {
    for set in SourceSet::ALL {
        let dir = root().join(set.dir());
        let notice = std::fs::read_to_string(dir.join("NOTICE")).unwrap();
        for word in ["Licence", "Use:"] {
            assert!(notice.contains(word), "{}/NOTICE lacks `{word}`", set.dir());
        }
        match set {
            SourceSet::SolveSpace => assert!(dir.join("COPYING").is_file()),
            SourceSet::Ezpz => assert!(dir.join("LICENSE").is_file()),
            SourceSet::PlaneGcs | SourceSet::Own => {}
        }
    }
}

#[test]
fn third_party_data_is_never_a_crate_source() {
    // The GPL data must stay data: no crate may reference the corpus through include_*!.
    let src = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut stack = vec![src];
    while let Some(dir) = stack.pop() {
        for e in std::fs::read_dir(dir).unwrap() {
            let p = e.unwrap().path();
            if p.is_dir() {
                stack.push(p);
            } else if p.extension().is_some_and(|x| x == "rs") {
                let text = std::fs::read_to_string(&p).unwrap();
                assert!(
                    !text.contains("include_str!") && !text.contains("include_bytes!"),
                    "{} embeds files",
                    p.display()
                );
            }
        }
    }
}

#[test]
fn stored_solutions_satisfy_their_constraints() {
    let c = load();
    let report = run_corpus(&mut ReferenceCheck::default(), &c.cases);
    println!("{report}");
    let bad: Vec<_> = report.failures().iter().map(|r| r.id.clone()).collect();
    assert!(bad.is_empty(), "stored solution fails: {bad:?}");
    // Only unsolvable cases and cases saved unsolved have nothing to check.
    for r in &report.results {
        if let Verdict::Skipped(why) = &r.verdict {
            let case = c.cases.iter().find(|x| x.id == r.id).unwrap();
            assert!(case.expected.solution.is_none(), "{} skipped: {why}", r.id);
        }
    }
}

#[test]
fn reference_solver_passes_the_whole_corpus() {
    let c = load();
    let report = run_corpus(&mut ReferenceSolver::default(), &c.cases);
    println!("{report}");
    assert_eq!(report.skipped(), 0, "{report}");
    let failed: Vec<_> = report
        .failures()
        .iter()
        .map(|r| (r.id.clone(), r.verdict.clone()))
        .collect();
    assert!(failed.is_empty(), "failures: {failed:?}");
}

#[test]
fn runner_reports_failures() {
    // A backend that always says "did not converge" must fail the solvable cases and pass the
    // unsolvable ones that accept it; it must never panic the runner.
    struct Never;
    impl CorpusBackend for Never {
        fn name(&self) -> String {
            "never".into()
        }
        fn solve(
            &mut self,
            _: &corpus::Case,
            _: &mut corpus::BuiltCase,
        ) -> Result<corpus::BackendOutcome, corpus::BackendSkip> {
            Ok(corpus::BackendOutcome {
                status: corpus::CaseStatus::DidNotConverge,
                dof: None,
                max_residual: None,
                iterations: None,
                groups: None,
            })
        }
    }
    let c = load();
    let report = run_corpus(&mut Never, &c.cases);
    assert!(report.failures().len() > 100);
    assert!(report.passed() >= 1);
    assert!(report.to_string().contains("FAIL own/d02_rectangle"));
}
