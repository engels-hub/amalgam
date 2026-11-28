//! S20 KH-17: the robustness framework (`amg_brep::robust`).
//!
//! Done-when 1 (BREP-04 corpus + 100 generated near-coincident cases: `robust_boolean` valid
//! `Ok` on ≥ 95 % of the generated cases, zero invalid `Ok`; raw OCCT baseline printed for
//! `docs/notes/kernel-metrics.md`), 2 (`exact_only` reproduces raw OCCT bit for bit), 3
//! (identical `.brep` bytes and `RobustReport` across runs and 1 vs 8 rayon threads), 4 (a
//! deadline expiry yields `Timeout`, never a later strategy's result), 5 (healing history
//! composes: a fillet after `Unify` resolves names through the composed history) and 6 (every
//! non-direct acceptance adds a warning), plus the near-coincidence proptest.
//!
//! The metric test prints its table (`--nocapture`); the numbers are in the S20 Refinement log
//! and `docs/notes/kernel-metrics.md`.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use amg_brep::robust::{HardenedKernel, RetryPolicy, StrategyId};
use amg_brep::{BoolOp, BoolOpts, BrepKernel, JobCtx, KernelConfig, KernelError, OcctKernel};

/// Without OCCT the hardened kernel returns the backend's own error, and the policy version is
/// part of the kernel version.
#[test]
fn hardened_kernel_passes_backend_errors_through() {
    let k = HardenedKernel::new(
        OcctKernel::new(KernelConfig::default()),
        RetryPolicy::default(),
        None,
    );
    assert!(
        amg_brep::kernel_version()
            .ends_with(&format!("policy-{}", amg_brep::robust::POLICY_VERSION))
    );
    assert_eq!(amg_brep::robust::POLICY_VERSION, 3);
    if !amg_brep::OCCT_BACKEND {
        let r = k.boolean(
            BoolOp::Fuse,
            &[],
            &[],
            &BoolOpts::default(),
            &JobCtx::detached(),
        );
        assert!(matches!(r, Err(KernelError::BackendUnavailable)), "{r:?}");
        let r = k.split(&[], &[], &BoolOpts::default(), &JobCtx::detached());
        assert!(matches!(r, Err(KernelError::BackendUnavailable)), "{r:?}");
    }
    assert_eq!(StrategyId::Direct.name(), "direct");
}

#[cfg(feature = "occt")]
#[path = "common/bool_corpus.rs"]
mod bool_corpus;
#[cfg(feature = "occt")]
#[path = "common/fault.rs"]
mod fault;
#[cfg(feature = "occt")]
#[path = "common/parts.rs"]
mod parts;

#[cfg(feature = "occt")]
mod occt {
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    use amg_brep::capture::{
        CaptureConfig, CaptureMode, FailureClass, FailureRecorder, read_manifest,
    };
    use amg_brep::robust::chain::{Chain, Strategy as ChainStrategy};
    use amg_brep::robust::corpus::{ANGLES, NearCase, NearFamily, OFFSETS, TILTS, near_coincident};
    use amg_brep::robust::{
        self, AttemptOutcome, HardenedKernel, HealLevel, RetryPolicy, RobustCtx, RobustError,
        RobustOutput, StrategyId, Validation, robust_boolean, robust_split,
    };
    use amg_brep::{
        BoolOp, BoolOpts, BrepKernel, CheckOpts, FilletOpts, FilletSpec, InputSub, JobCtx,
        KernelConfig, KernelError, KernelWarning, OcctKernel, OpResult, OpRole, Plane, Point3,
        Relation, Shape, SubKind, SubShape, Validate, Vec3,
    };
    use proptest::prelude::*;
    use proptest::test_runner::RngSeed;

    use super::bool_corpus::{Case, corpus};
    use super::fault::FaultKernel;
    use super::parts;

    /// Seed of the generated near-coincident set (fixed: the set is part of the baseline).
    const SEED: u64 = 0x4b48_3137; // "KH17"

    fn kernel(validate: Validate) -> OcctKernel {
        OcctKernel::new(KernelConfig {
            validate,
            ..KernelConfig::default()
        })
    }

    fn text(s: &Shape) -> Vec<u8> {
        let mut bytes = Vec::new();
        s.write_text(&mut bytes).unwrap();
        bytes
    }

    fn volume(s: &Shape) -> f64 {
        if s.count(SubKind::Face) == 0 {
            0.0
        } else {
            s.volume_adaptive(1e-10).unwrap()
        }
    }

    fn face(index: u32) -> SubShape {
        SubShape {
            kind: SubKind::Face,
            index,
        }
    }

    fn edge(index: u32) -> SubShape {
        SubShape {
            kind: SubKind::Edge,
            index,
        }
    }

    /// The independent check of an `Ok`: BRepCheck, closed solids with positive volume, and the
    /// oracle volume within `slack`.
    fn valid_ok(k: &dyn BrepKernel, s: &Shape, oracle: Option<(f64, f64)>) -> Result<(), String> {
        let report = k.check(s, &CheckOpts::default());
        if !report.valid {
            return Err(format!("check: {:?}", report.problem_kinds()));
        }
        let gate = Validation {
            require_solid: true,
            ..Validation::with_cap(1.0)
        };
        robust::validate(k, s, &gate).map_err(|r| format!("gate: {r}"))?;
        if let Some((v, slack)) = oracle {
            let got = volume(s);
            if (got - v).abs() > slack {
                return Err(format!("volume {got} vs {v} ± {slack:e}"));
            }
        }
        Ok(())
    }

    #[derive(Default, Debug)]
    struct Tally {
        cases: usize,
        valid_ok: usize,
        invalid_ok: usize,
        err: usize,
        non_direct: usize,
        /// Valid `Ok`s whose largest tolerance is above the 1e-3 mm cap ("valid but sloppy").
        sloppy: usize,
        millis: Vec<f64>,
    }

    impl Tally {
        fn time(&mut self, t: Instant) {
            self.millis.push(t.elapsed().as_secs_f64() * 1e3);
        }

        fn pct(&self, q: f64) -> f64 {
            let mut m = self.millis.clone();
            m.sort_by(f64::total_cmp);
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_precision_loss,
                reason = "small counts"
            )]
            let i = ((m.len() as f64 - 1.0) * q).round() as usize;
            m.get(i).copied().unwrap_or(0.0)
        }

        fn row(&self, name: &str) -> String {
            #[allow(clippy::cast_precision_loss, reason = "small counts")]
            let rate = 100.0 * self.valid_ok as f64 / self.cases.max(1) as f64;
            let time = if self.millis.is_empty() {
                "| - | - |".to_owned()
            } else {
                format!("| {:.2} | {:.2} |", self.pct(0.5), self.pct(0.95))
            };
            format!(
                "| {name} | {} | {} ({rate:.1} %) | {} | {} | {} | {} {time}",
                self.cases, self.valid_ok, self.invalid_ok, self.sloppy, self.err, self.non_direct,
            )
        }
    }

    fn corpus_or_skip() -> Option<Vec<Case>> {
        let c = corpus();
        if c.is_none() {
            eprintln!("SKIP: testdata/brep/bool/cases.toml not found");
        }
        c
    }

    fn corpus_oracle(case: &Case) -> Option<(f64, f64)> {
        case.volume.map(|v| (v, 1e-6 * v.abs().max(1.0)))
    }

    fn run_case_robust(
        k: &dyn BrepKernel,
        case: &Case,
        objects: &[Shape],
        tools: &[Shape],
        p: &RetryPolicy,
    ) -> Result<RobustOutput, RobustError> {
        let job = JobCtx::detached();
        let cx = RobustCtx::new(&job);
        let opts = case.opts(None);
        match case.op.as_str() {
            "fuse" => robust_boolean(k, BoolOp::Fuse, objects, tools, &opts, p, &cx),
            "cut" => robust_boolean(k, BoolOp::Cut, objects, tools, &opts, p, &cx),
            "common" => robust_boolean(k, BoolOp::Common, objects, tools, &opts, p, &cx),
            "split" => robust_split(k, objects, tools, &opts, p, &cx),
            other => panic!("unknown op {other}"),
        }
    }

    fn near_robust(
        k: &dyn BrepKernel,
        c: &NearCase,
        objects: &[Shape],
        tools: &[Shape],
        p: &RetryPolicy,
    ) -> Result<RobustOutput, RobustError> {
        let job = JobCtx::detached();
        let cx = RobustCtx::new(&job);
        robust_boolean(k, c.op, objects, tools, &BoolOpts::default(), p, &cx)
    }

    fn tally_robust(
        t: &mut Tally,
        k: &dyn BrepKernel,
        r: &Result<RobustOutput, RobustError>,
        oracle: Option<(f64, f64)>,
        id: &str,
    ) {
        t.cases += 1;
        match r {
            Ok(out) => {
                if out.report.accepted != Some(StrategyId::Direct) {
                    t.non_direct += 1;
                    // Done-when 6 on the corpora: every non-direct acceptance warns.
                    assert!(!out.report.warnings.is_empty(), "{id}: no warning");
                }
                match valid_ok(k, &out.result.shape, oracle) {
                    Ok(()) => {
                        t.valid_ok += 1;
                        if out.result.shape.max_tolerance() > 1e-3 {
                            t.sloppy += 1;
                        }
                    }
                    Err(why) => {
                        eprintln!(
                            "INVALID OK robust {id}: {why} via {:?}",
                            out.report.accepted
                        );
                        t.invalid_ok += 1;
                    }
                }
            }
            Err(e) => {
                assert_ne!(e.class, FailureClass::Other, "{id}: unclassified {e}");
                t.err += 1;
            }
        }
    }

    fn tally_raw(
        t: &mut Tally,
        k: &dyn BrepKernel,
        r: &Result<OpResult, KernelError>,
        oracle: Option<(f64, f64)>,
        id: &str,
    ) {
        t.cases += 1;
        match r {
            Ok(r) => match valid_ok(k, &r.shape, oracle) {
                Ok(()) => {
                    t.valid_ok += 1;
                    if r.shape.max_tolerance() > 1e-3 {
                        t.sloppy += 1;
                    }
                }
                Err(why) => {
                    eprintln!("invalid ok raw {id}: {why}");
                    t.invalid_ok += 1;
                }
            },
            Err(_) => t.err += 1,
        }
    }

    /// Done-when 1 (and 6 on both sets): baseline raw OCCT vs `robust_boolean` (`recompute`).
    #[test]
    fn robust_boolean_beats_raw_occt_on_near_coincident_cases() {
        // Raw baseline: no validation policy, so OCCT's own invalid `Ok`s are counted.
        let raw_k = kernel(Validate::Off);
        let p = RetryPolicy::recompute();
        let mut rows = Vec::new();

        if let Some(cases) = corpus_or_skip() {
            let (mut raw, mut rob) = (Tally::default(), Tally::default());
            for case in &cases {
                let (objects, tools) = case.operands(&raw_k);
                let oracle = corpus_oracle(case);
                let t = Instant::now();
                let r = case.run(&raw_k, &objects, &tools, None);
                raw.time(t);
                if case.error.is_none() {
                    tally_raw(&mut raw, &raw_k, &r, oracle, &case.name);
                }
                let t = Instant::now();
                let r = run_case_robust(&raw_k, case, &objects, &tools, &p);
                rob.time(t);
                if case.error.is_some() {
                    assert!(r.is_err(), "{}: expected error, got Ok", case.name);
                    continue;
                }
                tally_robust(&mut rob, &raw_k, &r, oracle, &case.name);
            }
            assert_eq!(rob.invalid_ok, 0, "robust invalid Ok on the BREP-04 corpus");
            assert!(rob.valid_ok >= raw.valid_ok, "{rob:?} vs {raw:?}");
            rows.push(raw.row("BREP-04 corpus, raw OCCT"));
            rows.push(rob.row("BREP-04 corpus, robust recompute"));
        }

        let cases = near_coincident(100, SEED);
        let (mut raw, mut rob, mut ex) = (Tally::default(), Tally::default(), Tally::default());
        let exhaustive = RetryPolicy::exhaustive();
        let mut per_family = Vec::new();
        for f in NearFamily::ALL {
            per_family.push((f, Tally::default(), Tally::default()));
        }
        for c in &cases {
            let (objects, tools) = c.operands(&raw_k).unwrap();
            let oracle = Some(c.expected_volume());
            let fam = per_family
                .iter_mut()
                .find(|(f, ..)| *f == c.family)
                .unwrap();
            let t = Instant::now();
            let r = raw_k.boolean(
                c.op,
                &objects,
                &tools,
                &BoolOpts::default(),
                &JobCtx::detached(),
            );
            raw.time(t);
            tally_raw(&mut raw, &raw_k, &r, oracle, &c.id);
            tally_raw(&mut fam.1, &raw_k, &r, oracle, &c.id);
            let t = Instant::now();
            let r = near_robust(&raw_k, c, &objects, &tools, &p);
            rob.time(t);
            if let Err(e) = &r {
                eprintln!("robust Err {}: {e}", c.id);
            }
            tally_robust(&mut rob, &raw_k, &r, oracle, &c.id);
            tally_robust(&mut fam.2, &raw_k, &r, oracle, &c.id);
            let t = Instant::now();
            let r = near_robust(&raw_k, c, &objects, &tools, &exhaustive);
            ex.time(t);
            if let Err(e) = &r {
                eprintln!("robust exhaustive Err {}: {e}", c.id);
            }
            tally_robust(&mut ex, &raw_k, &r, oracle, &c.id);
        }
        rows.push(raw.row("near-coincident 100, raw OCCT"));
        rows.push(rob.row("near-coincident 100, robust recompute"));
        rows.push(ex.row("near-coincident 100, robust exhaustive"));
        for (f, r, b) in &per_family {
            rows.push(r.row(&format!("  {f:?}, raw")));
            rows.push(b.row(&format!("  {f:?}, robust")));
        }
        eprintln!(
            "| set | cases | valid Ok | invalid Ok | valid Ok above 1e-3 mm tol | Err | non-direct | p50 ms | p95 ms |\n|---|---|---|---|---|---|---|---|---|\n{}",
            rows.join("\n")
        );
        assert_eq!(rob.invalid_ok, 0, "robust invalid Ok on the generated set");
        assert_eq!(
            rob.sloppy, 0,
            "robust results stay within the tolerance cap"
        );
        assert_eq!(
            ex.invalid_ok, 0,
            "exhaustive invalid Ok on the generated set"
        );
        assert!(rob.valid_ok >= 95, "robust valid Ok {} / 100", rob.valid_ok);
    }

    /// Done-when 2: `exact_only` gives raw OCCT's result bit for bit (bytes and history), or the
    /// same error.
    #[test]
    fn exact_only_reproduces_raw_occt() {
        let k = kernel(Validate::Always);
        let p = RetryPolicy::exact_only();
        let (same, rejected) = (std::cell::Cell::new(0), std::cell::Cell::new(0));
        let check = |id: &str,
                     raw: Result<OpResult, KernelError>,
                     rob: Result<RobustOutput, RobustError>| {
            match (raw, rob) {
                (Ok(a), Ok(b)) => {
                    assert_eq!(text(&a.shape), text(&b.result.shape), "{id}: bytes");
                    assert_eq!(a.history, b.result.history, "{id}: history");
                    assert_eq!(a.warnings, b.result.warnings, "{id}: warnings");
                    assert_eq!(b.report.attempts.len(), 1);
                    assert!(b.report.warnings.is_empty());
                    same.set(same.get() + 1);
                }
                (Err(a), Err(b)) => {
                    let e = b.error.expect("the direct error is kept");
                    assert_eq!(a.to_string(), e.to_string(), "{id}: error");
                    same.set(same.get() + 1);
                }
                (Ok(_), Err(b)) => {
                    assert!(b.error.is_none(), "{id}: not a gate rejection: {b}");
                    // The gate refused a result OCCT's BRepCheck passed: never on the corpus.
                    eprintln!("{id}: exact_only rejected a raw Ok: {b}");
                    rejected.set(rejected.get() + 1);
                }
                (Err(a), Ok(_)) => panic!("{id}: raw failed ({a}) but exact_only succeeded"),
            }
        };
        if let Some(cases) = corpus_or_skip() {
            for case in &cases {
                let (objects, tools) = case.operands(&k);
                let raw = case.run(&k, &objects, &tools, None);
                let rob = run_case_robust(&k, case, &objects, &tools, &p);
                check(&case.name, raw, rob);
            }
            assert_eq!(rejected.get(), 0, "exact_only rejected corpus results");
        }
        for c in near_coincident(100, SEED) {
            let (objects, tools) = c.operands(&k).unwrap();
            let raw = k.boolean(
                c.op,
                &objects,
                &tools,
                &BoolOpts::default(),
                &JobCtx::detached(),
            );
            let rob = near_robust(&k, &c, &objects, &tools, &p);
            check(&c.id, raw, rob);
        }
        eprintln!(
            "exact_only: {} identical to raw OCCT, {} raw Ok refused by the gate",
            same.get(),
            rejected.get()
        );
    }

    /// Done-when 3: the same input gives identical `.brep` bytes, history and report (timing
    /// aside) on two runs and on a 1-thread and an 8-thread rayon pool.
    #[test]
    fn robust_results_are_deterministic_across_runs_and_threads() {
        let k = kernel(Validate::Off);
        let p = RetryPolicy::recompute();
        let pool = |n| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(n)
                .build()
                .unwrap()
        };
        let (one, eight) = (pool(1), pool(8));
        let mut inputs: Vec<(String, BoolOp, Vec<Shape>, Vec<Shape>)> = near_coincident(100, SEED)
            .into_iter()
            .map(|c| {
                let (o, t) = c.operands(&k).unwrap();
                (c.id, c.op, o, t)
            })
            .collect();
        if let Some(cases) = corpus_or_skip() {
            for case in cases.iter().filter(|c| c.determinism) {
                let op = match case.op.as_str() {
                    "fuse" => BoolOp::Fuse,
                    "cut" => BoolOp::Cut,
                    "common" => BoolOp::Common,
                    _ => continue,
                };
                let (o, t) = case.operands(&k);
                inputs.push((case.name.clone(), op, o, t));
            }
        }
        let run = |op: BoolOp, o: &[Shape], t: &[Shape]| {
            let job = JobCtx::detached();
            let cx = RobustCtx::new(&job);
            match robust_boolean(&k, op, o, t, &BoolOpts::default(), &p, &cx) {
                Ok(out) => Ok((
                    text(&out.result.shape),
                    out.result.history,
                    out.report.without_timing(),
                )),
                Err(e) => Err((e.class, e.report.without_timing())),
            }
        };
        let mut non_direct = 0;
        for (id, op, o, t) in &inputs {
            let a = one.install(|| run(*op, o, t));
            let b = eight.install(|| run(*op, o, t));
            let c = eight.install(|| run(*op, o, t));
            assert!(a == b, "{id}: 1 vs 8 threads differ");
            assert!(b == c, "{id}: two runs differ");
            if let Ok((_, _, r)) = &a
                && r.accepted != Some(StrategyId::Direct)
            {
                non_direct += 1;
            }
        }
        eprintln!(
            "determinism: {} cases ({non_direct} accepted a non-direct strategy) identical over 3 runs, 1 and 8 threads",
            inputs.len()
        );
    }

    /// Done-when 4: a deadline expiry yields `Timeout` and never a later strategy's result. The
    /// injected slow strategy is a real 1030-face cut (about 10 s) under a 300 ms deadline,
    /// followed by a strategy that would succeed at once.
    #[test]
    fn deadline_expiry_yields_timeout_never_a_later_result() {
        let k = kernel(Validate::Off);
        let (object, tool) = parts::plate_pair(&k, 32);
        let small = k.make_box(&Plane::XY, Vec3::splat(1.0)).unwrap().shape;
        let p = RetryPolicy {
            attempt_deadline: Duration::from_millis(300),
            ..RetryPolicy::recompute()
        };
        let job = JobCtx::detached();
        let cx = RobustCtx::new(&job);
        let later_ran = std::sync::atomic::AtomicBool::new(false);
        let started = Instant::now();
        let chain = Chain::new(&k, "boolean", &p, &cx);
        let slow = ChainStrategy::new(
            StrategyId::Direct,
            Validation::default(),
            "slow cut",
            |ctx| {
                k.boolean(
                    BoolOp::Cut,
                    std::slice::from_ref(&object),
                    std::slice::from_ref(&tool),
                    &BoolOpts::default(),
                    ctx,
                )
            },
        );
        let fast = ChainStrategy::new(
            StrategyId::Custom(1),
            Validation::default(),
            "fast",
            |ctx| {
                later_ran.store(true, std::sync::atomic::Ordering::Relaxed);
                k.make_compound(std::slice::from_ref(&small))
                    .map(|shape| OpResult {
                        shape,
                        history: amg_brep::ShapeHistory::default(),
                        warnings: Vec::new(),
                    })
                    .and_then(|r| ctx.checkpoint().map(|()| r).map_err(Into::into))
            },
        );
        let e = chain.run([slow, fast]).expect_err("timeout");
        let waited = started.elapsed();
        assert_eq!(e.class, FailureClass::Timeout, "{e}");
        assert!(!later_ran.load(std::sync::atomic::Ordering::Relaxed));
        assert!(e.report.accepted.is_none());
        assert_eq!(e.report.attempts.len(), 1);
        assert!(matches!(
            &e.report.attempts[0].outcome,
            AttemptOutcome::Failed(info) if info.class == FailureClass::Timeout
        ));
        eprintln!(
            "deadline: Timeout after {:.0} ms (deadline 300 ms, the cut alone takes seconds)",
            waited.as_secs_f64() * 1e3
        );
        assert!(waited < Duration::from_secs(5), "{waited:?}");

        // The same through `robust_boolean`: Timeout, not a fuzzy rung's result.
        let started = Instant::now();
        let r = robust_boolean(
            &k,
            BoolOp::Cut,
            std::slice::from_ref(&object),
            std::slice::from_ref(&tool),
            &BoolOpts::default(),
            &p,
            &cx,
        );
        let e = r.expect_err("timeout");
        assert_eq!(e.class, FailureClass::Timeout);
        assert_eq!(e.report.attempts.len(), 1);
        eprintln!(
            "deadline: robust_boolean Timeout after {:.0} ms",
            started.elapsed().as_secs_f64() * 1e3
        );
        // HardenedKernel reports it as a failed boolean.
        let hk = HardenedKernel::new(kernel(Validate::Off), p, None);
        let r = hk.boolean(
            BoolOp::Cut,
            std::slice::from_ref(&object),
            std::slice::from_ref(&tool),
            &BoolOpts::default(),
            &JobCtx::detached(),
        );
        assert!(matches!(r, Err(KernelError::OpFailed { .. })), "{r:?}");
    }

    fn find_face(k: &dyn BrepKernel, s: &Shape, centroid: Point3) -> u32 {
        (0..s.count(SubKind::Face))
            .find(|&i| (k.sub_geom(s, face(i)).unwrap().centroid - centroid).length() < 1e-6)
            .unwrap_or_else(|| panic!("no face at {centroid}"))
    }

    fn find_edge(k: &dyn BrepKernel, s: &Shape, centroid: Point3) -> u32 {
        (0..s.count(SubKind::Edge))
            .find(|&i| (k.sub_geom(s, edge(i)).unwrap().centroid - centroid).length() < 1e-6)
            .unwrap_or_else(|| panic!("no edge at {centroid}"))
    }

    /// Done-when 5: two boxes fused side by side have split top and front faces; `heal(Unify)`
    /// merges them and a fillet on the merged front-top edge follows. The composed history
    /// (fuse result → unify → fillet) resolves the original pieces: both top pieces into the
    /// trimmed top face, both front-top edge pieces into the fillet blend face.
    #[test]
    fn healing_history_composes_through_unify_and_a_fillet() {
        let k = kernel(Validate::Always);
        let job = JobCtx::detached();
        let cx = RobustCtx::new(&job);
        let a = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
        let b = a
            .transformed(&amg_brep::Transform::from_translation(Vec3::new(
                10.0, 0.0, 0.0,
            )))
            .unwrap();
        let fused = k
            .boolean(BoolOp::Fuse, &[a], &[b], &BoolOpts::default(), &job)
            .unwrap()
            .shape;
        assert_eq!(fused.count(SubKind::Face), 10, "fuse keeps the split faces");

        let healed = robust::heal(&k, &fused, HealLevel::Unify, &cx).unwrap();
        let ids: Vec<_> = healed.report.attempts.iter().map(|a| a.strategy).collect();
        assert_eq!(
            ids,
            vec![
                StrategyId::Heal(HealLevel::Light),
                StrategyId::UnifySameDomain
            ]
        );
        assert_eq!(
            healed.report.accepted,
            Some(StrategyId::Heal(HealLevel::Unify))
        );
        assert!(!healed.report.warnings.is_empty());
        let unified = &healed.result.shape;
        assert_eq!(unified.count(SubKind::Face), 6, "unify merges to a box");

        let long_edge = find_edge(&k, unified, Point3::new(10.0, 0.0, 10.0));
        let filleted = k
            .fillet(
                unified,
                &[(amg_brep::EdgeId(long_edge), FilletSpec::Constant(2.0))],
                &FilletOpts::default(),
                &job,
            )
            .unwrap();
        let composed = healed.result.history.compose(&filleted.history);
        let out = &filleted.shape;

        for x in [5.0, 15.0] {
            // A top piece of the fused shape → the (trimmed) top face.
            let top = find_face(&k, &fused, Point3::new(x, 5.0, 10.0));
            let rel: Vec<_> = composed
                .of_input(InputSub {
                    operand: 0,
                    sub: face(top),
                })
                .cloned()
                .collect();
            let [Relation::Modified(outs)] = rel.as_slice() else {
                panic!("top piece at x = {x}: {rel:?}");
            };
            assert_eq!(outs.len(), 1);
            let g = k.sub_geom(out, outs[0]).unwrap();
            assert!((g.centroid.z - 10.0).abs() < 1e-9, "{g:?}");
            assert!(g.axis.unwrap().dot(Vec3::Z) > 1.0 - 1e-12);
            // The top face lost the fillet's 2 mm strip: 20 × (10 − 2).
            assert!((g.measure - 160.0).abs() < 1e-6, "{}", g.measure);

            // A front-top edge piece → the blend face.
            let piece = find_edge(&k, &fused, Point3::new(x, 0.0, 10.0));
            let rel: Vec<_> = composed
                .of_input(InputSub {
                    operand: 0,
                    sub: edge(piece),
                })
                .cloned()
                .collect();
            let blend = rel
                .iter()
                .find_map(|r| match r {
                    Relation::Generated(outs) => outs.iter().find(|o| o.kind == SubKind::Face),
                    _ => None,
                })
                .unwrap_or_else(|| panic!("edge piece at x = {x}: {rel:?}"));
            assert!(
                matches!(composed.role_of(*blend), Some(OpRole::FilletBlend { .. })),
                "{:?}",
                composed.role_of(*blend)
            );
        }
        assert!(
            composed
                .range_error([
                    out.count(SubKind::Face),
                    out.count(SubKind::Edge),
                    out.count(SubKind::Vertex)
                ])
                .is_none()
        );
    }

    fn overlapping_boxes(k: &dyn BrepKernel) -> (Shape, Shape) {
        let a = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
        let b = a
            .transformed(&amg_brep::Transform::from_translation(Vec3::splat(5.0)))
            .unwrap();
        (a, b)
    }

    /// Done-when 6: a non-direct acceptance adds a report warning, a second one above 1e-5 mm,
    /// and `HardenedKernel` turns them into `KernelWarning::Strategy`; a direct one adds none.
    #[test]
    fn non_direct_acceptances_warn() {
        let faulty = FaultKernel {
            inner: kernel(Validate::Always),
            abort: false,
            exact_boolean_fails: true,
            fillet_fault: None,
        };
        let (a, b) = overlapping_boxes(&faulty);
        let job = JobCtx::detached();
        let cx = RobustCtx::new(&job);
        let objects = std::slice::from_ref(&a);
        let tools = std::slice::from_ref(&b);

        let out = robust_boolean(
            &faulty,
            BoolOp::Fuse,
            objects,
            tools,
            &BoolOpts::default(),
            &RetryPolicy::recompute(),
            &cx,
        )
        .unwrap();
        assert_eq!(out.report.accepted, Some(StrategyId::Fuzzy { rung: 0 }));
        assert_eq!(out.report.attempts.len(), 2);
        assert_eq!(out.report.warnings.len(), 1, "{:?}", out.report.warnings);
        assert!(out.report.warnings[0].contains("fuzzy 1e-5 mm"));
        assert!((volume(&out.result.shape) - 1875.0).abs() < 1e-6);

        let p = RetryPolicy {
            fuzzy_ladder: vec![1e-4],
            ..RetryPolicy::recompute()
        };
        let out = robust_boolean(
            &faulty,
            BoolOp::Fuse,
            objects,
            tools,
            &BoolOpts::default(),
            &p,
            &cx,
        )
        .unwrap();
        assert_eq!(out.report.warnings.len(), 2, "{:?}", out.report.warnings);
        assert!(out.report.warnings[1].contains("above 1e-5 mm"));

        let hk = HardenedKernel::new(faulty, RetryPolicy::recompute(), None);
        let r = hk
            .boolean(BoolOp::Fuse, objects, tools, &BoolOpts::default(), &job)
            .unwrap();
        let strategy: Vec<_> = r
            .warnings
            .iter()
            .filter_map(|w| match w {
                KernelWarning::Strategy { name, detail } => Some((*name, detail.clone())),
                _ => None,
            })
            .collect();
        assert_eq!(strategy.len(), 1, "{:?}", r.warnings);
        assert_eq!(strategy[0].0, "fuzzy");
        assert!(r.warnings.contains(&KernelWarning::FuzzyUsed(1e-5)));
        let r = hk
            .split(objects, tools, &BoolOpts::default(), &job)
            .unwrap();
        assert!(
            r.warnings
                .iter()
                .any(|w| matches!(w, KernelWarning::Strategy { name: "fuzzy", .. }))
        );

        // Direct acceptance: no warning at all.
        let plain = kernel(Validate::Always);
        let out = robust_boolean(
            &plain,
            BoolOp::Fuse,
            objects,
            tools,
            &BoolOpts::default(),
            &RetryPolicy::recompute(),
            &cx,
        )
        .unwrap();
        assert_eq!(out.report.accepted, Some(StrategyId::Direct));
        assert!(out.report.warnings.is_empty());
        let hk = HardenedKernel::new(plain, RetryPolicy::recompute(), None);
        let r = hk
            .boolean(BoolOp::Fuse, objects, tools, &BoolOpts::default(), &job)
            .unwrap();
        assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    }

    /// A final failure is dumped with the chain's attempts; argument errors stop the chain at
    /// once and come back unchanged through `HardenedKernel`.
    #[test]
    fn failures_are_classified_dumped_and_passed_through() {
        let k = kernel(Validate::Always);
        let dir = std::env::temp_dir().join(format!("amg-kh17-dump-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let recorder = Arc::new(
            FailureRecorder::new(CaptureConfig {
                mode: CaptureMode::OnFailure,
                ..CaptureConfig::in_dir(&dir)
            })
            .unwrap(),
        );
        let job = JobCtx::detached();
        let cx = RobustCtx {
            job: &job,
            recorder: Some(&recorder),
            tol: amg_brep::Tolerance::DEFAULT,
        };
        let solid = k.make_box(&Plane::XY, Vec3::splat(10.0)).unwrap().shape;
        let sheet = k
            .make_planar_polygon(
                &[
                    Point3::new(-5.0, -5.0, 5.0),
                    Point3::new(15.0, -5.0, 5.0),
                    Point3::new(15.0, 15.0, 5.0),
                ],
                &[],
                &amg_brep::Tolerance::DEFAULT,
            )
            .unwrap();
        // A fuse of a solid and a face is not allowed: every strategy fails the same way.
        let e = robust_boolean(
            &k,
            BoolOp::Fuse,
            std::slice::from_ref(&solid),
            std::slice::from_ref(&sheet),
            &BoolOpts::default(),
            &RetryPolicy::recompute(),
            &cx,
        )
        .expect_err("not allowed");
        assert_eq!(e.class, FailureClass::NotDone);
        let ids: Vec<_> = e.report.attempts.iter().map(|a| a.strategy).collect();
        assert_eq!(ids[0], StrategyId::Direct);
        assert!(ids.contains(&StrategyId::Fuzzy { rung: 0 }));
        assert!(ids.contains(&StrategyId::Heal(HealLevel::Unify)));
        let dumps = recorder.dumps().unwrap();
        assert_eq!(dumps.len(), 1);
        let m = read_manifest(&dumps[0]).unwrap();
        assert_eq!(m.attempts.len(), e.report.attempts.len());
        assert_eq!(m.attempts[0].strategy, "direct");
        assert!(
            m.attempts[0].outcome.starts_with("failed: NotDone"),
            "{:?}",
            m.attempts[0]
        );

        // An argument error: one attempt, the kernel's own error.
        let e = robust_boolean(
            &k,
            BoolOp::Cut,
            std::slice::from_ref(&solid),
            &[],
            &BoolOpts::default(),
            &RetryPolicy::recompute(),
            &RobustCtx::new(&job),
        )
        .expect_err("cut needs a tool");
        assert_eq!(e.class, FailureClass::InvalidInput);
        assert_eq!(e.report.attempts.len(), 1);
        let hk = HardenedKernel::new(kernel(Validate::Always), RetryPolicy::recompute(), None);
        let r = hk.boolean(
            BoolOp::Cut,
            std::slice::from_ref(&solid),
            &[],
            &BoolOpts::default(),
            &job,
        );
        assert!(matches!(r, Err(KernelError::InvalidInput { .. })), "{r:?}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The proptest case from its four indices (family, offset, tilt or angle, op).
    fn near_case_of(f: usize, o: usize, a: usize, op: u8) -> NearCase {
        let family = NearFamily::ALL[f];
        let angle = match family {
            NearFamily::BoxFaceOffset | NearFamily::FlushPocket | NearFamily::ShiftedCopy => {
                ANGLES[a % ANGLES.len()]
            }
            NearFamily::TiltedFace | NearFamily::SkewCoaxial => TILTS[a],
            _ => 0.0,
        };
        let op = match (family, op) {
            (NearFamily::BoxFaceOffset | NearFamily::TiltedFace, 0) => BoolOp::Fuse,
            (NearFamily::BoxFaceOffset | NearFamily::TiltedFace, 1) => BoolOp::Cut,
            (NearFamily::BoxFaceOffset | NearFamily::TiltedFace, _) => BoolOp::Common,
            (NearFamily::FlushPocket | NearFamily::TangentHole, _) => BoolOp::Cut,
            (NearFamily::TangentCylinders | NearFamily::ShiftedCopy, 1) => BoolOp::Cut,
            _ => BoolOp::Fuse,
        };
        NearCase {
            id: format!("prop-{f}-{o}-{a}-{op:?}"),
            family,
            op,
            offset: OFFSETS[o],
            angle,
        }
    }

    /// The proptest's seed: fixed, so a run is reproducible; `AMG_NEAR_SEED=<u64>` draws
    /// another sample (the KH-17 fix ran seeds 0..200).
    fn near_seed() -> u64 {
        std::env::var("AMG_NEAR_SEED")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(SEED)
    }

    /// Whether a robust result is a valid `Ok` or a classified `Err`; the message otherwise.
    fn valid_or_classified(
        k: &dyn BrepKernel,
        c: &NearCase,
        r: &Result<RobustOutput, RobustError>,
    ) -> Result<bool, String> {
        match r {
            Ok(out) => valid_ok(k, &out.result.shape, Some(c.expected_volume()))
                .map(|()| true)
                .map_err(|why| {
                    format!("{}: invalid Ok via {:?}: {why}", c.id, out.report.accepted)
                }),
            Err(e) if e.class == FailureClass::Other => Err(format!("{}: unclassified {e}", c.id)),
            Err(_) => Ok(false),
        }
    }

    /// KH-17 fix regressions (seen in KH-20's runs of the proptest): a skewed coaxial fuse at
    /// offset 1e-7 mm and tilt ±1e-3 rad, where OCCT's exact fuse returns both cylinders unmerged
    /// (two overlapping solids, volume `ΣV` = 1570.80 mm³ against 1178.10 mm³) and the volume
    /// bounds alone accepted it. The disjoint-solid check now refuses the direct result.
    #[test]
    fn skew_coaxial_unmerged_fuse_is_refused() {
        let k = kernel(Validate::Off);
        for (o, a) in [(1, 6), (1, 7)] {
            let c = near_case_of(9, o, a, 0);
            assert_eq!(c.id, format!("prop-9-{o}-{a}-Fuse"));
            let (objects, tools) = c.operands(&k).unwrap();
            let raw = k
                .boolean(
                    c.op,
                    &objects,
                    &tools,
                    &BoolOpts::default(),
                    &JobCtx::detached(),
                )
                .unwrap();
            let gate = Validation {
                require_solid: true,
                disjoint_solids: true,
                ..Validation::default()
            };
            let rejected = robust::validate(&k, &raw.shape, &gate);
            if raw.shape.solids().len() > 1 {
                assert!(
                    matches!(rejected, Err(robust::Rejection::OverlappingSolids(0, 1))),
                    "{}: {rejected:?}",
                    c.id
                );
            }
            let r = near_robust(&k, &c, &objects, &tools, &RetryPolicy::recompute());
            let outcome = valid_or_classified(&k, &c, &r);
            assert!(outcome.is_ok(), "{outcome:?}");
            let report = match &r {
                Ok(out) => &out.report,
                Err(e) => &e.report,
            };
            eprintln!("{}: {:?}", c.id, report.without_timing().attempts);
        }
    }

    /// The disjoint-solid check passes pieces that only share faces (a split) and solids that
    /// only touch along a line (tangent cylinders), so it adds no false refusals.
    #[test]
    fn touching_solids_pass_the_disjoint_check() {
        let k = kernel(Validate::Off);
        let gate = Validation {
            require_solid: true,
            disjoint_solids: true,
            ..Validation::default()
        };
        let (a, b) = overlapping_boxes(&k);
        let pieces = k
            .split(
                std::slice::from_ref(&a),
                std::slice::from_ref(&b),
                &BoolOpts::default(),
                &JobCtx::detached(),
            )
            .unwrap();
        assert!(pieces.shape.solids().len() >= 2);
        robust::validate(&k, &pieces.shape, &gate).unwrap();
        let c = NearCase {
            id: "tangent".into(),
            family: NearFamily::TangentCylinders,
            op: BoolOp::Fuse,
            offset: 0.0,
            angle: 0.0,
        };
        let (objects, tools) = c.operands(&k).unwrap();
        let fused = k
            .boolean(
                BoolOp::Fuse,
                &objects,
                &tools,
                &BoolOpts::default(),
                &JobCtx::detached(),
            )
            .unwrap();
        robust::validate(&k, &fused.shape, &gate).unwrap();
        // Two overlapping boxes side by side in one compound are refused.
        let both = k.make_compound(&[a, b]).unwrap();
        assert!(matches!(
            robust::validate(&k, &both, &gate),
            Err(robust::Rejection::OverlappingSolids(0, 1))
        ));
    }

    /// Every distinct case of the proptest's space (612 of them): no invalid `Ok`, ≥ 95 % valid.
    #[test]
    fn near_coincident_space_has_no_invalid_ok() {
        let k = kernel(Validate::Off);
        let mut seen = std::collections::BTreeSet::new();
        let (mut ok, mut err, mut bad) = (0usize, 0usize, Vec::new());
        for f in 0..NearFamily::ALL.len() {
            for o in 0..OFFSETS.len() {
                for a in 0..TILTS.len() {
                    for op in 0u8..3 {
                        let c = near_case_of(f, o, a, op);
                        let key = format!("{:?} {} {} {:?}", c.family, c.offset, c.angle, c.op);
                        if !seen.insert(key) {
                            continue;
                        }
                        let (objects, tools) = c.operands(&k).unwrap();
                        let r = near_robust(&k, &c, &objects, &tools, &RetryPolicy::recompute());
                        match valid_or_classified(&k, &c, &r) {
                            Ok(true) => ok += 1,
                            Ok(false) => {
                                eprintln!(
                                    "classified Err {}: {:?}",
                                    c.id,
                                    r.as_ref().err().map(|e| e.class)
                                );
                                err += 1;
                            }
                            Err(why) => bad.push(why),
                        }
                    }
                }
            }
        }
        let n = seen.len();
        eprintln!(
            "near-coincident space: {n} cases, {ok} valid Ok, {err} classified Err, {} invalid Ok",
            bad.len()
        );
        assert!(bad.is_empty(), "{bad:#?}");
        assert!(ok * 100 >= 95 * n, "{ok} of {n} valid");
    }

    /// The proptest's sampling over seeds `0..AMG_NEAR_SEEDS` (default 200), tallied: the
    /// KH-17 fix evidence (`cargo nextest run … --run-ignored all near_coincident_seed_sweep`).
    #[test]
    #[ignore = "slow: about 9600 robust booleans"]
    fn near_coincident_seed_sweep() {
        use proptest::test_runner::TestRunner;
        let seeds: u64 = std::env::var("AMG_NEAR_SEEDS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(200);
        let k = kernel(Validate::Off);
        let tally = std::cell::RefCell::new((0usize, 0usize, Vec::<String>::new()));
        for seed in 0..seeds {
            let mut runner = TestRunner::new(ProptestConfig {
                cases: 48,
                failure_persistence: None,
                rng_seed: RngSeed::Fixed(seed),
                ..ProptestConfig::default()
            });
            runner
                .run(&near_case(), |c| {
                    let (objects, tools) = c.operands(&k).unwrap();
                    let r = near_robust(&k, &c, &objects, &tools, &RetryPolicy::recompute());
                    let mut t = tally.borrow_mut();
                    match valid_or_classified(&k, &c, &r) {
                        Ok(true) => t.0 += 1,
                        Ok(false) => t.1 += 1,
                        Err(why) => t.2.push(format!("seed {seed}: {why}")),
                    }
                    Ok(())
                })
                .unwrap();
        }
        let (ok, err, bad) = tally.into_inner();
        let n = ok + err + bad.len();
        eprintln!(
            "seed sweep: {seeds} seeds, {n} cases, {ok} valid Ok, {err} classified Err, {} invalid Ok",
            bad.len()
        );
        assert!(bad.is_empty(), "{bad:#?}");
    }

    fn near_case() -> impl Strategy<Value = NearCase> {
        (
            0usize..NearFamily::ALL.len(),
            0usize..OFFSETS.len(),
            0usize..TILTS.len(),
            0u8..3,
        )
            .prop_map(|(f, o, a, op)| near_case_of(f, o, a, op))
    }

    proptest! {
        #![proptest_config(ProptestConfig {
            cases: 48,
            failure_persistence: None,
            rng_seed: RngSeed::Fixed(near_seed()),
            ..ProptestConfig::default()
        })]

        /// Design note §7: near-coincident inputs give a valid `Ok` (BRepCheck, closed solids,
        /// the oracle volume) or a classified `Err`; never a panic, never an invalid `Ok`.
        #[test]
        fn near_coincident_booleans_are_valid_or_classified(c in near_case()) {
            let k = kernel(Validate::Off);
            let (objects, tools) = c.operands(&k).unwrap();
            match near_robust(&k, &c, &objects, &tools, &RetryPolicy::recompute()) {
                Ok(out) => {
                    let check = valid_ok(&k, &out.result.shape, Some(c.expected_volume()));
                    prop_assert!(check.is_ok(), "{}: {:?}", c.id, check);
                }
                Err(e) => prop_assert_ne!(e.class, FailureClass::Other),
            }
        }
    }
}
