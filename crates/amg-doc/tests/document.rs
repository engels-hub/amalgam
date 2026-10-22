//! DOC-12: recompute as a job (S01 §3 "Job integration").

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_doc::testing::{TestSlow, compute_count};
use amg_doc::{
    Command, ComputeMode, Design, Document, Expr, FeatureBox, FeatureId, GeomCache, ParamId,
    RecomputeError, RecomputeStatus, recompute,
};
use amg_jobs::{JobCtx, JobSystem, JobSystemConfig};

fn jobs() -> JobSystem {
    JobSystem::new(JobSystemConfig::default()).unwrap()
}

/// `n` independent slow features, each reading its own parameter `ms<i>` (initially `ms`).
fn slow_design(n: usize, ms: &str, panic: bool) -> (Design, Vec<FeatureId>, Vec<ParamId>) {
    let mut d = Design::new("doc12");
    let root = d.root();
    let (mut fs, mut ps) = (Vec::new(), Vec::new());
    for i in 0..n {
        let p = d.add_param(&format!("ms{i}"), ms, None, "").unwrap();
        let f = FeatureBox::new(TestSlow {
            ms: Expr::Param(p),
            tag: i64::try_from(i).unwrap(),
            panic,
        });
        fs.push(d.add_feature(None, f, root, None).unwrap());
        ps.push(p);
    }
    (d, fs, ps)
}

fn set(doc: &mut Document, p: ParamId, expr: &str) {
    doc.apply(Command::SetParamExpr {
        id: p,
        expr: expr.to_owned(),
    })
    .unwrap();
}

/// Polls until the evaluated state is current and no job runs.
fn settle(doc: &mut Document, jobs: &JobSystem) {
    let end = Instant::now() + Duration::from_secs(30);
    loop {
        doc.poll(jobs);
        if doc.recompute_status() == RecomputeStatus::Idle {
            return;
        }
        assert!(Instant::now() < end, "recompute did not settle");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn counts(doc: &Document, fs: &[FeatureId]) -> Vec<u64> {
    let u = doc.design().meta().uuid;
    fs.iter().map(|f| compute_count(u, *f)).collect()
}

#[test]
fn fifty_rapid_edits_swap_in_only_the_last_revision() {
    let jobs = jobs();
    let (d, fs, ps) = slow_design(10, "20", false);
    let mut doc = Document::from_design(d);
    doc.recompute_blocking(&jobs).unwrap();
    assert!(doc.is_evaluated());
    let initial = Arc::clone(doc.evaluated());
    let base = counts(&doc, &fs);
    assert!(base.iter().all(|c| *c == 1));

    let mut worst = Duration::ZERO;
    let mut timed_poll = |doc: &mut Document| {
        let t = Instant::now();
        doc.poll(&jobs);
        worst = worst.max(t.elapsed());
    };
    for k in 0..50 {
        set(&mut doc, ps[0], &format!("{}", 400 + k));
        timed_poll(&mut doc);
        // Nothing finished yet, so the old evaluated state is still the one in place.
        assert!(
            Arc::ptr_eq(doc.evaluated(), &initial),
            "swapped at edit {k}"
        );
        assert!(matches!(
            doc.recompute_status(),
            RecomputeStatus::Running { .. }
        ));
    }
    assert!(
        worst < Duration::from_millis(1),
        "poll blocked for {worst:?}"
    );

    let last = doc.design().revision();
    settle(&mut doc, &jobs);
    assert!(!Arc::ptr_eq(doc.evaluated(), &initial));
    assert_eq!(doc.evaluated().revision, last);

    let after = counts(&doc, &fs);
    // Same result as a synchronous pass over the final design.
    let reference = recompute(
        doc.design(),
        &GeomCache::new(1 << 30),
        ComputeMode::Full,
        &JobCtx::detached(),
    )
    .unwrap();
    for f in &fs {
        assert_eq!(
            doc.evaluated().node(*f).unwrap().key,
            reference.evaluated.node(*f).unwrap().key
        );
    }
    // The nine untouched features were never recomputed.
    assert_eq!(after[1..], base[1..]);
    let run = doc.last_run().unwrap();
    assert_eq!(run.computed, vec![fs[0]]);
}

#[test]
fn cancelled_work_stays_cached() {
    let jobs = jobs();
    let (d, fs, ps) = slow_design(10, "40", false);
    let mut doc = Document::from_design(d);
    doc.poll(&jobs);
    std::thread::sleep(Duration::from_millis(220));
    set(&mut doc, ps[9], "41");
    doc.poll(&jobs);
    settle(&mut doc, &jobs);

    let c = counts(&doc, &fs);
    assert!(c.iter().all(|n| (1..=2).contains(n)), "{c:?}");
    // Only the feature in flight when the first job was cancelled can run twice.
    assert!(c.iter().filter(|n| **n == 2).count() <= 1, "{c:?}");
    let run = doc.last_run().unwrap();
    assert!(!run.cached.is_empty(), "nothing was reused: {run:?}");
    assert_eq!(run.cached.len() + run.computed.len(), 10);
}

#[test]
fn stale_finished_result_is_dropped() {
    let jobs = jobs();
    let (d, _fs, ps) = slow_design(3, "5", false);
    let mut doc = Document::from_design(d);
    doc.poll(&jobs);
    // Let the job finish without polling.
    let end = Instant::now() + Duration::from_secs(10);
    while !jobs.jobs().is_empty() {
        assert!(Instant::now() < end);
        std::thread::sleep(Duration::from_millis(1));
    }
    let before = Arc::clone(doc.evaluated());
    set(&mut doc, ps[1], "6");
    doc.poll(&jobs);
    assert!(
        Arc::ptr_eq(doc.evaluated(), &before),
        "a result for an old revision was swapped in"
    );
    assert!(!doc.is_evaluated());
    settle(&mut doc, &jobs);
    assert_eq!(doc.evaluated().revision, doc.design().revision());
}

#[test]
fn blocking_recompute_and_undo_hit_the_cache() {
    let jobs = jobs();
    let (d, fs, ps) = slow_design(4, "1", false);
    let mut doc = Document::from_design(d);
    assert_eq!(doc.recompute_status(), RecomputeStatus::Stale);
    doc.recompute_blocking(&jobs).unwrap();
    assert_eq!(doc.recompute_status(), RecomputeStatus::Idle);
    assert_eq!(doc.evaluated().bodies().count(), 4);
    let first = counts(&doc, &fs);
    doc.recompute_blocking(&jobs).unwrap();
    assert_eq!(counts(&doc, &fs), first, "no-op recompute computed");

    set(&mut doc, ps[2], "2");
    doc.recompute_blocking(&jobs).unwrap();
    assert_eq!(doc.last_run().unwrap().computed, vec![fs[2]]);
    assert!(doc.undo());
    doc.recompute_blocking(&jobs).unwrap();
    assert!(doc.last_run().unwrap().computed.is_empty(), "undo missed");
    assert_eq!(doc.last_run().unwrap().cached.len(), 4);
}

#[cfg(debug_assertions)]
#[test]
fn recompute_blocking_asserts_off_the_ui_thread_only() {
    let r = std::thread::spawn(|| {
        amg_jobs::mark_ui_thread();
        let mut doc = Document::new("ui");
        let _ = doc.recompute_blocking(&jobs());
    })
    .join();
    assert!(r.is_err(), "no debug assertion on the UI thread");

    // `poll` is the UI-thread API and must work there.
    std::thread::spawn(|| {
        amg_jobs::mark_ui_thread();
        let jobs = jobs();
        let mut doc = Document::new("ui");
        doc.poll(&jobs);
        settle(&mut doc, &jobs);
    })
    .join()
    .unwrap();
}

#[test]
fn preview_never_touches_cache_or_undo() {
    let jobs = jobs();
    let (d, _fs, ps) = slow_design(4, "1", false);
    let mut doc = Document::from_design(d);
    doc.recompute_blocking(&jobs).unwrap();
    let stats = doc.cache().stats();
    let (applied, revision, design_rev) = (
        doc.history().stack().applied(),
        doc.history().revision(),
        doc.design().revision(),
    );
    let evaluated = Arc::clone(doc.evaluated());

    let cmd = Command::SetParamExpr {
        id: ps[1],
        expr: "3".into(),
    };
    let preview = doc
        .preview(&jobs, &cmd)
        .unwrap()
        .wait_blocking()
        .map_err(RecomputeError::from)
        .unwrap();
    assert!(
        preview.revision != design_rev,
        "preview is of the edited design"
    );
    assert_eq!(preview.bodies().count(), 4);

    assert_eq!(doc.cache().stats(), stats, "preview touched the main cache");
    assert!(doc.preview_cache().stats().entries > 0);
    assert_eq!(doc.history().stack().applied(), applied);
    assert_eq!(doc.history().stack().redoable(), 0);
    assert_eq!(doc.history().revision(), revision);
    assert_eq!(doc.design().revision(), design_rev);
    assert!(Arc::ptr_eq(doc.evaluated(), &evaluated));
    assert!(doc.is_evaluated());

    let bad = Command::SetParamExpr {
        id: ParamId(999_999),
        expr: "1".into(),
    };
    assert!(doc.preview(&jobs, &bad).is_err());
}

#[test]
fn status_reports_progress_and_feature_name() {
    let jobs = jobs();
    let (d, _fs, _ps) = slow_design(6, "30", false);
    let mut doc = Document::from_design(d);
    assert_eq!(doc.recompute_status(), RecomputeStatus::Stale);
    doc.poll(&jobs);
    std::thread::sleep(Duration::from_millis(80));
    match doc.recompute_status() {
        RecomputeStatus::Running { progress, message } => {
            assert!((0.0..1.0).contains(&progress), "{progress}");
            assert_eq!(message.as_deref(), Some("test.slow"));
        }
        s => panic!("expected Running, got {s:?}"),
    }
    settle(&mut doc, &jobs);
}

#[test]
fn a_panicking_feature_fails_once_and_is_not_retried_every_frame() {
    let jobs = jobs();
    let (d, _fs, ps) = slow_design(2, "1", true);
    let mut doc = Document::from_design(d);
    let err = doc.recompute_blocking(&jobs).unwrap_err();
    assert!(matches!(err, RecomputeError::Panicked(_)), "{err}");
    assert!(matches!(doc.recompute_status(), RecomputeStatus::Failed(_)));
    doc.poll(&jobs);
    assert!(jobs.jobs().is_empty(), "poll respawned a failed recompute");
    // A new edit is a new revision: the next poll tries again.
    set(&mut doc, ps[0], "2");
    assert_eq!(doc.recompute_status(), RecomputeStatus::Stale);
    doc.poll(&jobs);
    assert!(matches!(
        doc.recompute_status(),
        RecomputeStatus::Running { .. } | RecomputeStatus::Failed(_)
    ));
}
