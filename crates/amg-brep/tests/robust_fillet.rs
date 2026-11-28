//! S20 KH-03: the fillet strategy chain, failing-edge isolation and `max_radius`.
//!
//! - `corpus_failures_name_edges_classes_and_max_radii` (Done-when 3, 4): every chain failure on
//!   the committed corpus names ≥ 1 requested edge and a meaningful class; on the 25 cases with an
//!   analytic max radius it is within 2 % and the class is `RadiusTooLarge`.
//! - `max_radius_is_consistent_on_inflated_radii` (Done-when 4): `r̂` works and `1.02·r̂` fails
//!   for ≥ 95 % of the failing edges of the synthetic cases with every radius ×4.
//! - `generated_parts_are_valid_or_classified` (Done-when 5): 256 proptest-drawn parts (fixed
//!   seed) through `HardenedKernel` in worker processes.
//! - `chain::*`: a shape variant and a sequential strategy accepted with warnings and composed
//!   history (injected faults), isolation by OCCT report, probe and ddmin, `exact_only`, argument
//!   errors, determinism over runs and 1 vs 8 threads.
//! - `print_reports` (ignored): the attempts and diagnosis of `AMG_KH03_IDS` cases.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::result_large_err,
    reason = "test helpers outside #[test] functions; they return the chain's own Result"
)]

use amg_brep::capture::KernelOp;
use amg_brep::robust::corpus::case::{Case, CaseInput, CorpusSource};
use amg_brep::robust::fillet::{FilletFailure, fillet};
use amg_brep::robust::{RetryPolicy, RobustCtx, RobustOutput};
use amg_brep::{
    EdgeId, FilletOpts, FilletSpec, JobCtx, KernelConfig, OCCT_BACKEND, OcctKernel, Shape, Validate,
};

/// Cases that crash OCCT itself inside `MakeFillet` (an access violation, KH-09): only the
/// process-isolated harness may run them.
const KNOWN_CRASHES: [&str; 1] = ["fillet/synthetic/boolean_stack-0005"];

fn raw() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    })
}

fn is_lfs_pointer(path: &std::path::Path) -> bool {
    std::fs::read(path).is_ok_and(|b| b.starts_with(b"version https://git-lfs.github.com/spec/v1"))
}

/// The committed fillet corpus without un-fetched LFS pointers (fail with
/// `AMG_REQUIRE_TESTDATA=1`) and without [`KNOWN_CRASHES`].
fn corpus() -> Vec<Case> {
    let require = std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1");
    CorpusSource::fillet()
        .load()
        .expect("corpus")
        .into_iter()
        .filter(|c| !KNOWN_CRASHES.contains(&c.id.as_str()))
        .filter(|c| match &c.input {
            CaseInput::Dir { dir, .. } => {
                let input = dir.join("inputs").join("0.brep");
                let pointer = is_lfs_pointer(&input);
                assert!(
                    !(pointer && require),
                    "{} is an LFS pointer",
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

/// The shape, edges and options of a fillet case.
fn fillet_input(k: &OcctKernel, c: &Case) -> (Shape, Vec<(EdgeId, FilletSpec)>, FilletOpts) {
    let p = c.prepare(k).unwrap();
    match p.op {
        KernelOp::Fillet { edges, opts, .. } => (
            p.inputs[0].clone(),
            edges.into_iter().map(|(r, s)| (r.id, s)).collect(),
            opts,
        ),
        other => panic!("{}: not a fillet: {other:?}", c.id),
    }
}

fn run(
    k: &OcctKernel,
    s: &Shape,
    edges: &[(EdgeId, FilletSpec)],
    opts: &FilletOpts,
    p: &RetryPolicy,
) -> Result<RobustOutput, FilletFailure> {
    let job = JobCtx::detached();
    let cx = RobustCtx::new(&job);
    fillet(k, s, edges, opts, p, &cx)
}

/// Prints the chain's attempts and the diagnosis for the cases in `AMG_KH03_IDS`
/// (comma-separated ids, or `all`).
#[test]
#[ignore = "diagnostics for development"]
fn print_reports() {
    if !OCCT_BACKEND {
        return;
    }
    let ids = std::env::var("AMG_KH03_IDS").unwrap_or_default();
    let k = raw();
    for c in corpus()
        .iter()
        .filter(|c| ids == "all" || ids.split(',').any(|i| c.id.ends_with(i)))
    {
        let (s, edges, opts) = fillet_input(&k, c);
        let r = run(&k, &s, &edges, &opts, &RetryPolicy::recompute());
        eprintln!("== {} ({} edges)", c.id, edges.len());
        let report = match &r {
            Ok(out) => &out.report,
            Err(f) => &f.diagnosis.report,
        };
        for a in &report.attempts {
            eprintln!("   {} {:?} {} ms", a.strategy, a.outcome, a.millis);
        }
        match &r {
            Ok(out) => eprintln!(
                "   accepted {:?} {:?}",
                out.report.accepted, out.report.warnings
            ),
            Err(f) => eprintln!(
                "   failure {:?}: {} {:?}",
                f.class, f.message, f.diagnosis.failing
            ),
        }
    }
}

/// Done-when 3 and 4 on the committed corpus (in process, [`KNOWN_CRASHES`] left out): every
/// failure of the chain names ≥ 1 failing edge and a meaningful class; on cases with an analytic
/// max radius (`radius_too_large`) the failing edge's `max_radius` is within 2 % of it and the
/// class is `RadiusTooLarge`; on the other failures, `r̂` works and `1.02·r̂` fails for ≥ 95 % of
/// the edges with a max radius below the requested one (the rest are flagged `approximate`).
#[test]
fn corpus_failures_name_edges_classes_and_max_radii() {
    if !OCCT_BACKEND {
        eprintln!("SKIP: needs the occt feature");
        return;
    }
    let k = raw();
    let (mut failures, mut limit_cases, mut worst_limit_dev) = (0usize, 0usize, 0.0f64);
    let (mut other_radii, mut other_exact) = (0usize, 0usize);
    let mut problems = Vec::new();
    for c in corpus() {
        let prepared = c.prepare(&k).unwrap();
        let (s, edges, opts) = fillet_input(&k, &c);
        let Err(f) = run(&k, &s, &edges, &opts, &RetryPolicy::recompute()) else {
            continue;
        };
        failures += 1;
        let classified = amg_brep::robust::corpus::harness::is_classified(f.class);
        if f.diagnosis.failing.is_empty() || !classified {
            problems.push(format!("{}: {:?} {:?}", c.id, f.class, f.diagnosis.failing));
        }
        if f.diagnosis
            .failing
            .iter()
            .any(|fe| !edges.iter().any(|(e, _)| *e == fe.edge))
        {
            problems.push(format!("{}: a failing edge was not requested", c.id));
        }
        let requested = |e: EdgeId| {
            edges
                .iter()
                .find(|(x, _)| *x == e)
                .map_or(0.0, |(_, sp)| match sp {
                    FilletSpec::Constant(r) | FilletSpec::Chord(r) => *r,
                    FilletSpec::Variable(law) => law.iter().map(|(_, r)| *r).fold(0.0, f64::max),
                })
        };
        if let Some(limit) = prepared.expect.max_radius {
            limit_cases += 1;
            match f.diagnosis.failing.as_slice() {
                [fe] if fe.max_radius.is_some() => {
                    let dev = (fe.max_radius.unwrap() - limit).abs() / limit;
                    worst_limit_dev = worst_limit_dev.max(dev);
                    if dev > 0.02 || fe.class != amg_brep::capture::FailureClass::RadiusTooLarge {
                        problems.push(format!("{}: {fe:?} vs limit {limit}", c.id));
                    }
                }
                other => problems.push(format!(
                    "{}: expected one edge with a radius: {other:?}",
                    c.id
                )),
            }
        } else {
            for fe in &f.diagnosis.failing {
                if let Some(r) = fe.max_radius
                    && r < requested(fe.edge)
                {
                    other_radii += 1;
                    if !fe.approximate {
                        other_exact += 1;
                    }
                }
            }
        }
        eprintln!("{}: {:?} — {}", c.id, f.class, f.message);
    }
    eprintln!(
        "{failures} failures; {limit_cases} with an analytic max radius, worst deviation {worst_limit_dev:.2e}; \
         other failures: {other_exact} of {other_radii} max radii exact"
    );
    assert!(problems.is_empty(), "{problems:#?}");
    assert!(limit_cases >= 20, "{limit_cases}");
    assert!(
        other_exact * 100 >= other_radii * 95,
        "{other_exact} of {other_radii}"
    );
}

/// Done-when 4, "other failures": the `prism_vertical_edges` and `boolean_stack` cases with
/// every radius ×4, most of which then fail on edges whose limit comes from their
/// neighbours rather than one analytic formula. For ≥ 95 % of the failing edges whose max
/// radius is below the requested one, `r̂` works and `1.02·r̂` fails.
#[test]
fn max_radius_is_consistent_on_inflated_radii() {
    if !OCCT_BACKEND {
        eprintln!("SKIP: needs the occt feature");
        return;
    }
    let k = raw();
    let pick = |c: &Case| c.id.contains("prism_vertical_edges") || c.id.contains("boolean_stack");
    let (mut failures, mut radii, mut exact) = (0usize, 0usize, 0usize);
    let mut approximate = Vec::new();
    for c in corpus().iter().filter(|c| pick(c)) {
        let (s, edges, opts) = fillet_input(&k, c);
        let big: Vec<(EdgeId, FilletSpec)> = edges
            .iter()
            .map(|(e, sp)| match sp {
                FilletSpec::Constant(r) => (*e, FilletSpec::Constant(4.0 * r)),
                other => (*e, other.clone()),
            })
            .collect();
        let Err(f) = run(&k, &s, &big, &opts, &RetryPolicy::recompute()) else {
            continue;
        };
        failures += 1;
        assert!(!f.diagnosis.failing.is_empty(), "{}", c.id);
        for fe in &f.diagnosis.failing {
            let requested =
                big.iter()
                    .find(|(x, _)| *x == fe.edge)
                    .map_or(0.0, |(_, sp)| match sp {
                        FilletSpec::Constant(r) => *r,
                        _ => 0.0,
                    });
            if let Some(r) = fe.max_radius
                && r < requested
            {
                radii += 1;
                if fe.approximate {
                    approximate.push(format!("{} edge {} r̂ {r}", c.id, fe.edge.0));
                } else {
                    exact += 1;
                }
            }
        }
    }
    eprintln!(
        "{failures} inflated failures; {exact} of {radii} max radii exact; approximate: {approximate:?}"
    );
    assert!(radii >= 10, "{radii}");
    assert!(
        exact * 100 >= radii * 95,
        "{exact} of {radii}: {approximate:?}"
    );
}

/// Set in the worker processes this binary starts.
const WORKER_ENV: &str = "AMG_KH03_HARNESS_WORKER";

/// The worker side of the harness: idle in a normal test run.
#[test]
fn worker_entry() {
    if std::env::var(WORKER_ENV).is_ok_and(|v| v == "1") {
        amg_brep::robust::corpus::harness::serve_stdio().expect("serve");
    }
}

fn worker() -> std::process::Command {
    let mut c = std::process::Command::new(std::env::current_exe().expect("test binary"));
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

/// The proptest's seed: fixed, so a run is reproducible; `AMG_KH03_SEED=<u64>` draws another
/// sample.
fn prop_seed() -> u64 {
    std::env::var("AMG_KH03_SEED")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0x4b48_3033)
}

/// Done-when 5: 256 parts drawn by proptest (fixed seed) from the four generators with seeds
/// outside the committed corpus give a valid `Ok` or a classified `Err` through the chain
/// (`HardenedKernel`, `recompute`): never a panic, an unclassified error, a timeout or an invalid
/// `Ok` (exact-volume cases within 1e-6). The parts run in worker processes, so an OCCT access
/// violation is a `Crash` record; one is accepted only where raw `MakeFillet` crashes on the same
/// part (an OCCT bug for KH-09, outside any strategy's reach).
#[test]
fn generated_parts_are_valid_or_classified() {
    use amg_brep::robust::corpus::fillet::{FilletGenCase, FilletGenerator};
    use amg_brep::robust::corpus::harness::{
        HarnessConfig, Strategy as HStrategy, Verdict, run_isolated,
    };
    use proptest::prelude::*;
    use proptest::test_runner::{RngSeed, TestRunner};

    if !OCCT_BACKEND {
        eprintln!("SKIP: needs the occt feature");
        return;
    }
    let drawn = std::cell::RefCell::new(Vec::new());
    let mut runner = TestRunner::new(ProptestConfig {
        cases: 256,
        failure_persistence: None,
        rng_seed: RngSeed::Fixed(prop_seed()),
        ..ProptestConfig::default()
    });
    runner
        .run(
            &(0..FilletGenerator::ALL.len(), 1_000u64..1_000_000),
            |(g, seed)| {
                drawn.borrow_mut().push(FilletGenCase {
                    generator: FilletGenerator::ALL[g],
                    seed,
                });
                Ok(())
            },
        )
        .unwrap();
    let mut gens = drawn.into_inner();
    gens.sort_by_key(|g| (g.generator.name(), g.seed));
    gens.dedup_by_key(|g| (g.generator.name(), g.seed));
    assert!(gens.len() >= 250, "{} distinct parts", gens.len());
    let cases: Vec<Case> = gens.into_iter().map(Case::synthetic).collect();
    let cfg = HarnessConfig {
        jobs: 4,
        strategies: HStrategy::ALL.to_vec(),
        ..HarnessConfig::default()
    };
    let records = run_isolated(&cases, &cfg, &worker).unwrap();
    let raw_crash = |id: &str| {
        records
            .iter()
            .any(|r| r.id == id && r.strategy == HStrategy::Raw && r.verdict == Verdict::Crash)
    };
    let mut tally = std::collections::BTreeMap::<(&str, String), usize>::new();
    let mut problems = Vec::new();
    for r in &records {
        *tally
            .entry((r.strategy.name(), format!("{:?}", r.verdict)))
            .or_default() += 1;
        if r.strategy != HStrategy::Hardened {
            continue;
        }
        match r.verdict {
            Verdict::ValidOk => {}
            Verdict::ClassifiedErr if !r.offending.is_empty() => {}
            Verdict::Crash if raw_crash(&r.id) => {
                eprintln!("OCCT crash (raw too): {}", r.id);
            }
            other => problems.push(format!(
                "{}: {other:?} {:?} {}",
                r.id,
                r.failure_class,
                r.message.clone().unwrap_or_else(|| r.problems.join("; "))
            )),
        }
    }
    eprintln!("seed {:#x}, {} parts: {tally:?}", prop_seed(), cases.len());
    assert!(problems.is_empty(), "{problems:#?}");
}

#[cfg(feature = "occt")]
#[path = "common/fault.rs"]
mod fault;

/// Chain behaviour on a 10 mm cube with injected faults (`fault::FilletFault`).
#[cfg(feature = "occt")]
mod chain {
    use amg_brep::capture::FailureClass;
    use amg_brep::robust::fillet::{FilletFailure, fillet};
    use amg_brep::robust::{HardenedKernel, RetryPolicy, RobustCtx, RobustOutput, StrategyId};
    use amg_brep::{
        BrepKernel, EdgeId, FilletOpts, FilletSpec, InputSub, JobCtx, KernelError, KernelWarning,
        OcctKernel, OpRole, Plane, Relation, Shape, SubKind, SubShape, Vec3,
    };

    use super::fault::{FaultKernel, FilletFault};
    use super::raw;

    const SIDE: f64 = 10.0;

    fn cube() -> Shape {
        raw().make_box(&Plane::XY, Vec3::splat(SIDE)).unwrap().shape
    }

    fn faulty(f: FilletFault) -> FaultKernel<OcctKernel> {
        FaultKernel {
            inner: raw(),
            abort: false,
            exact_boolean_fails: false,
            fillet_fault: Some(f),
        }
    }

    fn edge_sub(e: EdgeId) -> SubShape {
        SubShape {
            kind: SubKind::Edge,
            index: e.0,
        }
    }

    /// The cube's vertical edges.
    fn vertical(k: &dyn BrepKernel, s: &Shape) -> Vec<EdgeId> {
        s.edges()
            .filter(|&e| k.edge_ends(s, e).is_ok_and(|x| x.t0.get().z.abs() > 0.99))
            .collect()
    }

    /// Two vertical edges that share no face (diagonally opposite).
    fn opposite(k: &dyn BrepKernel, s: &Shape) -> (EdgeId, EdgeId) {
        let v = vertical(k, s);
        let p = |e: EdgeId| k.edge_ends(s, e).unwrap().p0;
        let a = v[0];
        let b = *v
            .iter()
            .max_by(|x, y| {
                (p(**x) - p(a))
                    .length()
                    .total_cmp(&(p(**y) - p(a)).length())
            })
            .unwrap();
        (a, b)
    }

    /// The cube's volume with vertical edges filleted at radii `rs`.
    fn expected(rs: &[f64]) -> f64 {
        SIDE.powi(3)
            - rs.iter()
                .map(|r| SIDE * r * r * (1.0 - std::f64::consts::FRAC_PI_4))
                .sum::<f64>()
    }

    fn run(
        k: &dyn BrepKernel,
        s: &Shape,
        edges: &[(EdgeId, FilletSpec)],
        p: &RetryPolicy,
    ) -> Result<RobustOutput, FilletFailure> {
        let job = JobCtx::detached();
        let cx = RobustCtx::new(&job);
        fillet(k, s, edges, &FilletOpts::default(), p, &cx)
    }

    #[test]
    fn a_shape_variant_is_accepted_with_a_warning() {
        let k = faulty(FilletFault::DefaultOpts);
        let s = cube();
        let e = vertical(&k, &s)[0];
        let edges = [(e, FilletSpec::Constant(2.0))];
        let out = run(&k, &s, &edges, &RetryPolicy::recompute()).unwrap();
        assert_eq!(out.report.accepted, Some(StrategyId::FilletShapeVariant(0)));
        assert_eq!(out.report.warnings.len(), 1, "{:?}", out.report.warnings);
        let v = out.result.shape.volume().unwrap();
        assert!((v - expected(&[2.0])).abs() < 1e-6 * v, "{v}");
        // Through the decorator the warning becomes a KernelWarning::Strategy.
        let h = HardenedKernel::new(
            faulty(FilletFault::DefaultOpts),
            RetryPolicy::recompute(),
            None,
        );
        let r = h
            .fillet(&s, &edges, &FilletOpts::default(), &JobCtx::detached())
            .unwrap();
        assert!(r.warnings.iter().any(|w| matches!(
            w,
            KernelWarning::Strategy {
                name: "fillet_shape_variant",
                ..
            }
        )));
        // A direct acceptance adds none.
        let plain = HardenedKernel::new(raw(), RetryPolicy::recompute(), None);
        let r = plain
            .fillet(&s, &edges, &FilletOpts::default(), &JobCtx::detached())
            .unwrap();
        assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    }

    #[test]
    fn sequential_contours_compose_their_history() {
        let k = faulty(FilletFault::Together);
        let s = cube();
        let (a, b) = opposite(&k, &s);
        let edges = [
            (a, FilletSpec::Constant(1.0)),
            (b, FilletSpec::Constant(2.0)),
        ];
        let out = run(&k, &s, &edges, &RetryPolicy::recompute()).unwrap();
        assert_eq!(
            out.report.accepted,
            Some(StrategyId::RegroupChains),
            "{:#?}",
            out.report
        );
        let v = out.result.shape.volume().unwrap();
        let want = expected(&[1.0, 2.0]);
        assert!((v - want).abs() < 1e-6 * v, "{v} vs {want}");
        // Both input edges map to blend faces of the final shape, through both steps.
        let h = &out.result.history;
        for e in [a, b] {
            let input = InputSub {
                operand: 0,
                sub: edge_sub(e),
            };
            let faces: Vec<SubShape> = h
                .of_input(input)
                .flat_map(|r| match r {
                    Relation::Generated(o) => o.to_vec(),
                    _ => Vec::new(),
                })
                .filter(|o| o.kind == SubKind::Face)
                .collect();
            assert_eq!(
                faces.len(),
                1,
                "edge {}: {:?}",
                e.0,
                h.of_input(input).collect::<Vec<_>>()
            );
            assert!(matches!(
                h.role_of(faces[0]),
                Some(OpRole::FilletBlend { .. })
            ));
        }
        // Every input face still resolves.
        for f in 0..s.count(SubKind::Face) {
            let input = InputSub {
                operand: 0,
                sub: SubShape {
                    kind: SubKind::Face,
                    index: f,
                },
            };
            assert!(h.of_input(input).next().is_some(), "face {f} lost");
        }
    }

    #[test]
    fn isolation_names_only_the_failing_edge() {
        let k = raw();
        let s = cube();
        let (a, b) = opposite(&k, &s);
        let edges = [
            (a, FilletSpec::Constant(1.0)),
            (b, FilletSpec::Constant(12.0)),
        ];
        let f = run(&k, &s, &edges, &RetryPolicy::recompute()).unwrap_err();
        assert_eq!(f.diagnosis.failing.len(), 1, "{:?}", f.diagnosis.failing);
        let fe = &f.diagnosis.failing[0];
        assert_eq!(fe.edge, b);
        assert_eq!(fe.class, FailureClass::RadiusTooLarge);
        let r = fe.max_radius.unwrap();
        assert!(r < SIDE && r > 0.99 * SIDE, "{r}");
        assert!(!fe.approximate);
        // Interactive: the same edge and class from one probe, no max radius.
        let f = run(&k, &s, &edges, &RetryPolicy::interactive()).unwrap_err();
        assert_eq!(f.diagnosis.failing.len(), 1);
        assert_eq!(f.diagnosis.failing[0].edge, b);
        assert_eq!(f.diagnosis.failing[0].class, FailureClass::RadiusTooLarge);
        assert_eq!(f.diagnosis.failing[0].max_radius, None);
        // As a kernel error: the class and the edge.
        let ke = f.to_kernel_error();
        assert_eq!(FailureClass::of(&ke), FailureClass::RadiusTooLarge);
        assert_eq!(ke.offending(), &[edge_sub(b)]);
    }

    #[test]
    fn delta_debugging_finds_edges_that_fail_together() {
        // Every edge works alone and any two fail together; the interactive budget (4 attempts:
        // the direct one and the three shape variants) runs out before the sequential ones.
        let k = faulty(FilletFault::Together);
        let s = cube();
        let v = vertical(&k, &s);
        let edges: Vec<(EdgeId, FilletSpec)> =
            v.iter().map(|e| (*e, FilletSpec::Constant(1.0))).collect();
        let f = run(&k, &s, &edges, &RetryPolicy::interactive()).unwrap_err();
        assert_eq!(f.diagnosis.report.attempts.len(), 4);
        assert_eq!(f.diagnosis.failing.len(), 2, "{:?}", f.diagnosis.failing);
        for fe in &f.diagnosis.failing {
            assert!(v.contains(&fe.edge));
            assert_eq!(fe.max_radius, None);
        }
    }

    #[test]
    fn exact_only_reports_without_probing() {
        let k = raw();
        let s = cube();
        let e = vertical(&k, &s)[0];
        let f = run(
            &k,
            &s,
            &[(e, FilletSpec::Constant(30.0))],
            &RetryPolicy::exact_only(),
        )
        .unwrap_err();
        assert_eq!(f.diagnosis.report.attempts.len(), 1);
        assert_eq!(f.diagnosis.failing.len(), 1);
        assert_eq!(f.diagnosis.failing[0].edge, e);
        assert_eq!(f.diagnosis.failing[0].max_radius, None);
    }

    #[test]
    fn invalid_input_stops_the_chain_and_comes_back_unchanged() {
        let s = cube();
        let h = HardenedKernel::new(raw(), RetryPolicy::recompute(), None);
        let e = h.fillet(
            &s,
            &[(EdgeId(999), FilletSpec::Constant(1.0))],
            &FilletOpts::default(),
            &JobCtx::detached(),
        );
        assert!(matches!(e, Err(KernelError::InvalidInput { .. })), "{e:?}");
    }

    /// The same inputs give identical bytes, histories and reports over runs and on 1 vs 8
    /// rayon threads (an accepted sequential result and an isolated failure).
    #[test]
    fn results_are_deterministic_across_runs_and_threads() {
        let s = cube();
        let k = faulty(FilletFault::Together);
        let (a, b) = opposite(&k, &s);
        let ok_edges = [
            (a, FilletSpec::Constant(1.0)),
            (b, FilletSpec::Constant(2.0)),
        ];
        let bad_edges = [
            (a, FilletSpec::Constant(1.0)),
            (b, FilletSpec::Constant(12.0)),
        ];
        let once = || {
            let ok = run(&k, &s, &ok_edges, &RetryPolicy::recompute()).unwrap();
            let mut bytes = Vec::new();
            ok.result.shape.write_text(&mut bytes).unwrap();
            let bad = run(&raw(), &s, &bad_edges, &RetryPolicy::recompute()).unwrap_err();
            let mut diag = bad.diagnosis.clone();
            diag.report = diag.report.without_timing();
            (
                bytes,
                ok.report.without_timing(),
                ok.result.history,
                diag,
                bad.message,
            )
        };
        let pool = |n| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(n)
                .build()
                .unwrap()
        };
        let first = once();
        for p in [pool(1), pool(8)] {
            for _ in 0..2 {
                let again = p.install(once);
                assert!(again == first, "results differ between runs");
            }
        }
    }
}
