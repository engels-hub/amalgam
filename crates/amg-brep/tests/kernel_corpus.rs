//! S20 KH-02: the fillet corpus and the process-isolated harness.
//!
//! - `corpus_meets_the_size_targets` (Done-when 1): ≥ 200 cases, ≥ 120 synthetic from
//!   `seeds.toml`, ≥ 50 handmade, a smoke subset of about 40; owner/harvested cases under
//!   `$AMG_TESTDATA_DIR/kernel/fillet/` when that is set.
//! - `exact_volume_oracle_matches_tessellation` (Done-when 2): the `prism_vertical_edges` volume
//!   formula against a fine tessellation of OCCT's result on 20 cases.
//! - `golden_fillet_smoke` (Done-when 4): the smoke subset in worker processes (raw and, since
//!   KH-03, hardened) against
//!   `tests/golden/kernel/fillet-baseline.toml`, under 2 minutes; the `golden_` prefix puts it in
//!   CI's golden selection. `golden_fillet_smoke_catches_a_broken_case`: a baseline-passing case
//!   broken on purpose is reported as a regression.
//! - `harness_survives_crashes_and_hangs` (Done-when 5): a worker that dies with
//!   `STATUS_ACCESS_VIOLATION` (and one that aborts) inside a case gives a `Crash` record, a
//!   hanging one a `Timeout`, and the run completes. Runs without OCCT too.
//! - `full_fillet_corpus` (ignored, Done-when 3): the whole corpus, the baseline numbers.
//!
//! The worker processes are this test binary run again with `--exact worker_entry`
//! ([`WORKER_ENV`] set), which serves the harness protocol on stdin/stdout.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    reason = "test helpers outside #[test] functions"
)]

use std::path::PathBuf;
use std::process::Command;
use std::time::{Duration, Instant};

use amg_brep::robust::corpus::case::{
    Case, CaseInput, CorpusSource, ExpectOutcome, Inject, golden_kernel_dir,
};
use amg_brep::robust::corpus::fillet::{FilletGenCase, FilletGenerator};
use amg_brep::robust::corpus::harness::{
    Baseline, HarnessConfig, Outcome, Report, Strategy, Verdict, run_isolated, select, serve_stdio,
};
use amg_brep::{BrepKernel, JobCtx, KernelConfig, OCCT_BACKEND, OcctKernel, TessQuality, Validate};
use amg_testkit::{TriSoup, signed_volume};

/// Set in the worker processes this binary starts.
const WORKER_ENV: &str = "AMG_KERNEL_HARNESS_WORKER";

/// The worker side: idle in a normal test run.
#[test]
fn worker_entry() {
    if std::env::var(WORKER_ENV).is_ok_and(|v| v == "1") {
        serve_stdio().expect("serve");
    }
}

fn worker() -> Command {
    let mut c = Command::new(std::env::current_exe().expect("test binary"));
    c.args([
        "worker_entry",
        "--exact",
        "--nocapture",
        "--test-threads",
        "1",
    ])
    .env(WORKER_ENV, "1");
    c
}

fn baseline_path() -> PathBuf {
    golden_kernel_dir().join("fillet-baseline.toml")
}

/// Whether `path` is an un-fetched git-LFS pointer (the corpus `.brep` files are not in LFS
/// today; this keeps the test honest if they move there).
fn is_lfs_pointer(path: &std::path::Path) -> bool {
    std::fs::read(path).is_ok_and(|b| b.starts_with(b"version https://git-lfs.github.com/spec/v1"))
}

/// The fillet corpus, without cases whose inputs are un-fetched LFS pointers (which fail
/// instead with `AMG_REQUIRE_TESTDATA=1`).
fn corpus() -> Vec<Case> {
    let require = std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1");
    let source = CorpusSource::fillet();
    if std::env::var_os("AMG_TESTDATA_DIR").is_none() {
        eprintln!("SKIP: owner/harvested fillet cases need AMG_TESTDATA_DIR (unset)");
    }
    source
        .load()
        .expect("corpus")
        .into_iter()
        .filter(|c| match &c.input {
            CaseInput::Dir { dir, .. } => {
                let input = dir.join("inputs").join("0.brep");
                let pointer = is_lfs_pointer(&input);
                assert!(
                    !(pointer && require),
                    "{} is an un-fetched git-LFS pointer and AMG_REQUIRE_TESTDATA=1",
                    input.display()
                );
                if pointer {
                    eprintln!("SKIP: {} is an un-fetched git-LFS pointer", input.display());
                }
                !pointer
            }
            CaseInput::Fillet(_) => true,
        })
        .collect()
}

fn cfg(jobs: usize) -> HarnessConfig {
    HarnessConfig {
        jobs,
        ..HarnessConfig::default()
    }
}

#[test]
fn corpus_meets_the_size_targets() {
    let cases = corpus();
    let synthetic = cases.iter().filter(|c| c.source == "synthetic").count();
    let handmade = cases.iter().filter(|c| c.source == "handmade").count();
    let smoke = cases.iter().filter(|c| c.has_tag("smoke")).count();
    eprintln!(
        "fillet corpus: {} cases, {synthetic} synthetic, {handmade} handmade, {smoke} smoke",
        cases.len()
    );
    assert!(cases.len() >= 200, "{} cases", cases.len());
    assert!(synthetic >= 120, "{synthetic} synthetic");
    assert!(handmade >= 50, "{handmade} handmade");
    assert!((30..=50).contains(&smoke), "{smoke} smoke cases");
    for g in FilletGenerator::ALL {
        assert!(
            cases
                .iter()
                .any(|c| c.has_tag(g.name()) && c.has_tag("smoke")),
            "no smoke case of {}",
            g.name()
        );
    }
    for c in cases.iter().filter(|c| c.source == "handmade") {
        let CaseInput::Dir { dir, op } = &c.input else {
            panic!("{} is not a directory case", c.id);
        };
        let name = dir.file_name().unwrap().to_string_lossy();
        assert_eq!(c.id, format!("fillet/handmade/{name}"));
        assert!(dir.join(op).is_file() && dir.join("inputs").join("0.brep").is_file());
    }
    let clean = cases
        .iter()
        .filter(|c| c.expect.outcome == ExpectOutcome::CleanFailure)
        .count();
    assert!(clean >= 20, "{clean} clean-failure cases");
}

#[test]
fn handmade_cases_load_and_keep_their_edge_indices() {
    // Every op.json edge keeps its index after the `.brep` round trip (fingerprints match).
    if !OCCT_BACKEND {
        eprintln!("SKIP: needs the occt feature");
        return;
    }
    let k = OcctKernel::new(KernelConfig::default());
    for c in corpus().iter().filter(|c| c.source == "handmade") {
        let p = c.prepare(&k).unwrap_or_else(|e| panic!("{}: {e}", c.id));
        assert_eq!(p.inputs.len(), 1, "{}", c.id);
    }
}

#[test]
fn generated_cases_are_deterministic() {
    if !OCCT_BACKEND {
        eprintln!("SKIP: needs the occt feature");
        return;
    }
    let k = OcctKernel::new(KernelConfig::default());
    for generator in FilletGenerator::ALL {
        let case = Case::synthetic(FilletGenCase { generator, seed: 3 });
        let a = case.prepare(&k).unwrap();
        let b = case.prepare(&k).unwrap();
        assert_eq!(
            serde_json::to_string(&a.op).unwrap(),
            serde_json::to_string(&b.op).unwrap()
        );
        let text = |s: &amg_brep::Shape| {
            let mut v = Vec::new();
            s.write_text(&mut v).unwrap();
            v
        };
        assert_eq!(text(&a.inputs[0]), text(&b.inputs[0]), "{}", case.id);
    }
}

#[test]
fn exact_volume_oracle_matches_tessellation() {
    if !OCCT_BACKEND {
        eprintln!("SKIP: needs the occt feature");
        return;
    }
    let k = OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    });
    let q = TessQuality {
        linear: 2e-4,
        angular: 0.01,
        relative: false,
        min_size: 0.0,
        max_linear_abs: None,
    };
    let ctx = JobCtx::detached();
    let mut worst: f64 = 0.0;
    let mut worst_props: f64 = 0.0;
    for seed in 0..20 {
        let case = FilletGenCase {
            generator: FilletGenerator::PrismVerticalEdges,
            seed,
        };
        let built = case.build(&k).unwrap();
        let oracle = built.expect.volume.unwrap();
        let out = k
            .fillet(&built.input, &built.edges, &built.opts, &ctx)
            .unwrap_or_else(|e| panic!("seed {seed}: {e}"));
        let mesh = k.tessellate(&out.shape, &q, &ctx).unwrap();
        // The divergence theorem needs a geometrically closed surface, not shared vertices,
        // so the per-face triangles are used as they are.
        let tris = &mesh.tris;
        let v_mesh = signed_volume(TriSoup {
            positions: &mesh.positions,
            triangles: tris,
        });
        let v_props = k.mass_props(&out.shape).unwrap().volume;
        let rel = (v_mesh - oracle).abs() / oracle;
        let rel_props = (v_props - oracle).abs() / oracle;
        eprintln!(
            "seed {seed:2}: {} fillets, oracle {oracle:.6}, mesh {v_mesh:.6} ({rel:.2e}), \
             mass props {v_props:.9} ({rel_props:.2e}), {} triangles",
            built.edges.len(),
            tris.len()
        );
        assert!(
            rel <= 1e-4,
            "seed {seed}: tessellation {v_mesh} vs oracle {oracle}"
        );
        assert!(
            rel_props <= 1e-9,
            "seed {seed}: mass props {v_props} vs {oracle}"
        );
        worst = worst.max(rel);
        worst_props = worst_props.max(rel_props);
    }
    eprintln!("worst relative deviation: tessellation {worst:.2e}, mass props {worst_props:.2e}");
}

fn injected(id: &str, inject: Option<Inject>) -> Case {
    let mut c = Case::synthetic(FilletGenCase {
        generator: FilletGenerator::PrismVerticalEdges,
        seed: 0,
    });
    c.id = id.to_owned();
    c.inject = inject;
    c
}

#[test]
fn harness_survives_crashes_and_hangs() {
    let cases = vec![
        injected("before", None),
        injected("access-violation", Some(Inject::AccessViolation)),
        injected("after-crash", None),
        injected("abort", Some(Inject::Abort)),
        injected("hang", Some(Inject::Hang)),
        injected("last", None),
    ];
    let config = HarnessConfig {
        jobs: 1,
        timeout: Duration::from_secs(5),
        ..HarnessConfig::default()
    };
    let start = Instant::now();
    let records = run_isolated(&cases, &config, &worker).unwrap();
    eprintln!("{} records in {:?}", records.len(), start.elapsed());
    let ids: Vec<&str> = records.iter().map(|r| r.id.as_str()).collect();
    assert_eq!(
        ids,
        [
            "before",
            "access-violation",
            "after-crash",
            "abort",
            "hang",
            "last"
        ]
    );
    let av = &records[1];
    assert_eq!(
        (av.outcome, av.verdict),
        (Outcome::Crash, Verdict::Crash),
        "{av:?}"
    );
    let msg = av.message.as_deref().unwrap();
    assert!(msg.contains("during op"), "{msg}");
    if cfg!(windows) {
        assert!(msg.contains("0xc0000005"), "{msg}");
    }
    assert_eq!(records[3].outcome, Outcome::Crash);
    assert_eq!(records[4].outcome, Outcome::Timeout);
    // The cases after a crash or hang run in a fresh worker as normal.
    let normal = if OCCT_BACKEND {
        Verdict::ValidOk
    } else {
        Verdict::SetupError
    };
    for i in [0, 2, 5] {
        assert_eq!(records[i].verdict, normal, "{:?}", records[i]);
    }
    let report = Report::new(records, vec!["injected".to_owned()]);
    let all = &report.summary(Strategy::Raw).unwrap().all;
    assert_eq!((all.crashes, all.timeouts), (2, 1));
}

fn smoke_run(cases: &[Case]) -> (Vec<amg_brep::robust::corpus::harness::CaseRecord>, Duration) {
    let start = Instant::now();
    let records = run_isolated(cases, &cfg(4), &worker).unwrap();
    (records, start.elapsed())
}

#[test]
fn golden_fillet_smoke() {
    if !OCCT_BACKEND {
        eprintln!("SKIP: the fillet smoke subset needs the occt feature");
        return;
    }
    let cases = select(corpus(), &["smoke".to_owned()], None);
    // Both strategies: the baseline guards the KH-03 chain (`hardened`) as well as raw OCCT.
    let start = Instant::now();
    let both = HarnessConfig {
        strategies: Strategy::ALL.to_vec(),
        ..cfg(4)
    };
    let records = run_isolated(&cases, &both, &worker).unwrap();
    let elapsed = start.elapsed();
    let report = Report::new(records, vec!["fillet smoke".to_owned()]);
    eprintln!(
        "{}\n{} cases in {elapsed:?}",
        report.markdown(&[]),
        cases.len()
    );
    let base = Baseline::load(&baseline_path()).unwrap();
    let regressions = base.regressions(&report.cases);
    assert!(
        regressions.is_empty(),
        "baseline-passing cases now fail: {regressions:#?}"
    );
    // Every smoke case is in the baseline run, and most of them pass there.
    let passing = base.passing(Strategy::Raw);
    let in_base = cases
        .iter()
        .filter(|c| passing.contains(c.id.as_str()))
        .count();
    assert!(
        in_base * 10 >= cases.len() * 9,
        "{in_base} of {} smoke cases pass in the baseline",
        cases.len()
    );
    assert!(
        elapsed < Duration::from_secs(120),
        "smoke run took {elapsed:?}"
    );
}

#[test]
fn golden_fillet_smoke_catches_a_broken_case() {
    if !OCCT_BACKEND {
        eprintln!("SKIP: needs the occt feature");
        return;
    }
    let base = Baseline::load(&baseline_path()).unwrap();
    let passing = base.passing(Strategy::Raw);
    let good = |c: &Case| {
        c.has_tag("smoke")
            && passing.contains(c.id.as_str())
            && c.expect.outcome == ExpectOutcome::Success
    };
    let all = corpus();
    let synthetic: Vec<Case> = all
        .iter()
        .filter(|c| good(c) && c.source == "synthetic")
        .take(2)
        .cloned()
        .collect();
    let handmade: Vec<Case> = all
        .iter()
        .filter(|c| good(c) && c.source == "handmade" && c.expect.volume.is_some())
        .take(2)
        .cloned()
        .collect();
    let mut cases = [synthetic, handmade].concat();
    assert_eq!(cases.len(), 4);
    // Break two baseline-passing cases on purpose: the op of one fails, the other's result no
    // longer matches its (edited) volume oracle.
    cases[0].inject = Some(Inject::NotDone);
    let v = cases[2].expect.volume.unwrap();
    cases[2].expect.volume = Some(v * 1.01);
    let (records, _) = smoke_run(&cases);
    let regressions = base.regressions(&records);
    let ids: Vec<&str> = regressions.iter().map(|r| r.id.as_str()).collect();
    assert_eq!(
        ids,
        [cases[0].id.as_str(), cases[2].id.as_str()],
        "{regressions:#?}"
    );
    assert_eq!(regressions[0].verdict, Verdict::ClassifiedErr);
    assert_eq!(regressions[1].verdict, Verdict::InvalidOk);
    assert!(records[1].passed && records[3].passed);
}

/// The whole corpus (Done-when 3). Writes the report to `$AMG_HARNESS_REPORT` (default
/// `<target tmp>/fillet-harness.json`) and, with `AMG_HARNESS_WRITE_BASELINE=1`, rewrites the
/// baseline file.
#[test]
#[ignore = "full corpus (about 220 cases); run before closing a fillet hardening WP"]
fn full_fillet_corpus() {
    if !OCCT_BACKEND {
        panic!("the full corpus run needs the occt feature");
    }
    let cases = corpus();
    let start = Instant::now();
    let cfg = HarnessConfig {
        strategies: Strategy::ALL.to_vec(),
        ..cfg(4)
    };
    let records = run_isolated(&cases, &cfg, &worker).unwrap();
    let elapsed = start.elapsed();
    // KH-03 Done-when 2 and 3: the chain returns no invalid `Ok`, and every error it returns
    // names ≥ 1 failing edge and a meaningful class (a crash ends the worker before any answer).
    for r in records.iter().filter(|r| r.strategy == Strategy::Hardened) {
        assert_ne!(r.verdict, Verdict::InvalidOk, "{}: {:?}", r.id, r.problems);
        if r.outcome == Outcome::Err {
            let classified = r
                .failure_class
                .is_some_and(amg_brep::robust::corpus::harness::is_classified);
            assert!(
                classified && !r.offending.is_empty(),
                "{}: {:?} {:?}",
                r.id,
                r.failure_class,
                r.offending
            );
        }
    }
    let report = Report::new(
        records,
        vec!["fillet corpus (seeds.toml + handmade)".to_owned()],
    );
    eprintln!(
        "{}\n{} cases in {elapsed:?}",
        report.markdown(&[
            "synthetic",
            "handmade",
            "smoke",
            "prism_vertical_edges",
            "prism_mixed_edges",
            "boolean_stack",
            "radius_too_large",
            "corner3",
            "concave",
            "near_limit",
        ]),
        cases.len()
    );
    for r in report.cases.iter().filter(|r| !r.passed) {
        eprintln!(
            "not passing: {} {} {:?} {:?} {}",
            r.strategy.name(),
            r.id,
            r.verdict,
            r.failure_class,
            r.message.clone().unwrap_or_else(|| r.problems.join("; "))
        );
    }
    let path = std::env::var_os("AMG_HARNESS_REPORT").map_or_else(
        || PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("fillet-harness.json"),
        PathBuf::from,
    );
    report.write(&path).unwrap();
    eprintln!("report: {}", path.display());
    if std::env::var("AMG_HARNESS_WRITE_BASELINE").is_ok_and(|v| v == "1") {
        let text = Baseline::from_records(&report.cases).to_toml().unwrap();
        std::fs::write(baseline_path(), text).unwrap();
        eprintln!("baseline rewritten: {}", baseline_path().display());
    } else {
        let base = Baseline::load(&baseline_path()).unwrap();
        let regressions = base.regressions(&report.cases);
        assert!(regressions.is_empty(), "{regressions:#?}");
    }
}
