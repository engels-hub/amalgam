//! DOC-03: recompute engine core (S01 §3).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    missing_docs,
    reason = "tests"
)]

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Mutex;

use amg_doc::testing::{TestCounter, compute_count};
use amg_doc::{
    BodyId, CacheKey, ComputeMode, Dependency, Design, ErrorCause, Evaluated, Expr, Feature,
    FeatureBox, FeatureError, FeatureId, FeatureOutput, GeomCache, Health, RecomputeCtx, recompute,
};
use amg_jobs::{CancelToken, JobCtx};
use proptest::prelude::*;
use serde::{Deserialize, Serialize};

fn body(f: FeatureId) -> BodyId {
    BodyId { origin: f, slot: 0 }
}

fn counter(deps: &[FeatureId], value: i64, fail: bool) -> FeatureBox {
    FeatureBox::new(TestCounter {
        deps: deps.iter().map(|f| Dependency::Body(body(*f))).collect(),
        value,
        fail,
    })
}

fn add(d: &mut Design, f: FeatureBox) -> FeatureId {
    let root = d.root();
    d.add_feature(None, f, root, None).unwrap()
}

fn run(d: &Design, cache: &GeomCache) -> amg_doc::RecomputeResult {
    recompute(d, cache, ComputeMode::Full, &JobCtx::detached()).unwrap()
}

fn key_of(e: &Evaluated, f: FeatureId) -> CacheKey {
    e.node(f).unwrap().key.unwrap()
}

fn counts(d: &Design, ids: &[FeatureId]) -> Vec<u64> {
    let u = d.meta().uuid;
    ids.iter().map(|f| compute_count(u, *f)).collect()
}

// ---- Done-when 1: exact recompute set on random DAGs ---------------------------------------

/// For each feature, indices of earlier features whose body it reads.
fn dag() -> impl Strategy<Value = Vec<Vec<usize>>> {
    (1usize..14).prop_flat_map(|n| {
        (0..n)
            .map(|i| proptest::collection::btree_set(0..i.max(1), 0..4.min(i + 1)))
            .collect::<Vec<_>>()
            .prop_map(|sets| {
                sets.into_iter()
                    .enumerate()
                    .map(|(i, s)| s.into_iter().filter(|&j| j < i).collect())
                    .collect()
            })
    })
}

/// Oracle: nodes whose key must change when `k` is edited. Feature *i* reads the state of body
/// *j* left by its last writer before *i*; writers of body *j* are *j* itself and every later
/// feature that lists *j* as a dependency (test.counter passes its inputs through).
fn expected_dirty(deps: &[Vec<usize>], k: usize) -> BTreeSet<usize> {
    let n = deps.len();
    let mut writer: Vec<usize> = (0..n).collect(); // writer[j] = last writer of body j so far
    let mut dirty = BTreeSet::new();
    for (i, ds) in deps.iter().enumerate() {
        let reads_dirty = ds.iter().any(|&j| dirty.contains(&writer[j]));
        if i == k || reads_dirty {
            dirty.insert(i);
        }
        for &j in ds {
            writer[j] = i;
        }
    }
    dirty
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn editing_k_recomputes_exactly_its_dependents(deps in dag(), pick in any::<prop::sample::Index>()) {
        let mut d = Design::new("dag");
        let cache = GeomCache::new(1 << 30);
        let mut ids: Vec<FeatureId> = Vec::new();
        for (i, ds) in deps.iter().enumerate() {
            let from: Vec<FeatureId> = ds.iter().map(|&j| ids[j]).collect();
            ids.push(add(&mut d, counter(&from, i64::try_from(i).unwrap(), false)));
        }
        let first = run(&d, &cache);
        prop_assert_eq!(first.computed.len(), ids.len());
        prop_assert_eq!(counts(&d, &ids), vec![1; ids.len()]);

        // Unchanged document: everything hits.
        let again = run(&d, &cache);
        prop_assert!(again.computed.is_empty());
        prop_assert_eq!(counts(&d, &ids), vec![1; ids.len()]);

        let k = pick.index(ids.len());
        let from: Vec<FeatureId> = deps[k].iter().map(|&j| ids[j]).collect();
        d.replace_feature(ids[k], counter(&from, 1000, false)).unwrap();
        let after = run(&d, &cache);

        let want = expected_dirty(&deps, k);
        let got: BTreeSet<usize> = ids
            .iter()
            .enumerate()
            .filter(|(_, f)| after.computed.contains(f))
            .map(|(i, _)| i)
            .collect();
        prop_assert_eq!(&got, &want);
        // The test.counter's own counters agree with the engine's report.
        let c = counts(&d, &ids);
        for (i, n) in c.iter().enumerate() {
            prop_assert_eq!(*n, if want.contains(&i) { 2 } else { 1 }, "feature {}", i);
        }
        // Keys changed for exactly those nodes.
        for (i, f) in ids.iter().enumerate() {
            let changed = key_of(&first.evaluated, *f) != key_of(&after.evaluated, *f);
            prop_assert_eq!(changed, want.contains(&i));
        }
        // Reverting the edit is served from the cache entirely (undo is instant).
        d.replace_feature(ids[k], counter(&from, i64::try_from(k).unwrap(), false)).unwrap();
        let back = run(&d, &cache);
        prop_assert!(back.computed.is_empty());
        for f in &ids {
            prop_assert_eq!(key_of(&back.evaluated, *f), key_of(&first.evaluated, *f));
        }
    }
}

// ---- Done-when 2: failure semantics ---------------------------------------------------------

#[test]
fn failing_feature_has_no_effect_and_dependants_get_upstream() {
    let mut d = Design::new("fail");
    let cache = GeomCache::new(1 << 30);
    let a = add(&mut d, counter(&[], 0, false));
    let b = add(&mut d, counter(&[a], 1, true)); // reads A's body, then fails
    let c = add(&mut d, counter(&[b], 2, false)); // reads B's (never created) body
    let e = add(&mut d, counter(&[a], 3, false)); // reads A's body: must still see A's output

    let r = run(&d, &cache);
    let ev = &r.evaluated;
    assert_eq!(ev.node(a).unwrap().health, Health::Ok);
    let Health::Error(db) = &ev.node(b).unwrap().health else {
        panic!("B must fail");
    };
    assert_eq!(db.cause, None);
    let Health::Error(dc) = &ev.node(c).unwrap().health else {
        panic!("C must fail");
    };
    assert_eq!(dc.cause, Some(ErrorCause::Upstream(b)));
    assert_eq!(ev.node(e).unwrap().health, Health::Ok);

    // B had no effect: its body doesn't exist, A's body is exactly what A produced.
    assert!(ev.body(body(b)).is_none());
    assert!(ev.body(body(c)).is_none());
    let a_out = cache.get(&key_of(ev, a)).unwrap();
    let amg_doc::BodyChangeKind::Created { geom, .. } = &a_out.bodies[0].change else {
        panic!()
    };
    // E passed A's body through (modified), still the very same geometry allocation.
    assert!(ev.body(body(a)).unwrap().geom.same_as(geom));
    // C never computed (engine rejects it before compute).
    assert_eq!(counts(&d, &[c]), vec![0]);
    // Failed nodes aren't cached: B recomputes (and fails) on every pass.
    run(&d, &cache);
    assert_eq!(counts(&d, &[a, b, e]), vec![1, 2, 1]);
}

#[test]
fn missing_producer_is_an_upstream_error() {
    let mut d = Design::new("gone");
    let cache = GeomCache::new(1 << 30);
    let a = add(&mut d, counter(&[], 0, false));
    let b = add(&mut d, counter(&[a], 1, false));
    d.remove_feature(a).unwrap();
    let r = run(&d, &cache);
    let Health::Error(db) = &r.evaluated.node(b).unwrap().health else {
        panic!()
    };
    assert_eq!(db.cause, Some(ErrorCause::Upstream(a)));
    assert!(r.evaluated.body(body(b)).is_none());
}

// ---- suppression, marker --------------------------------------------------------------------

#[test]
fn suppression_and_unsuppression() {
    let mut d = Design::new("sup");
    let cache = GeomCache::new(1 << 30);
    let a = add(&mut d, counter(&[], 0, false));
    let b = add(&mut d, counter(&[a], 1, false));
    let c = add(&mut d, counter(&[], 2, false));
    let first = run(&d, &cache);

    d.set_suppressed(a, true).unwrap();
    let r = run(&d, &cache);
    let ev = &r.evaluated;
    assert_eq!(ev.node(a).unwrap().health, Health::Suppressed);
    assert_eq!(ev.node(a).unwrap().key, None);
    let Health::Error(db) = &ev.node(b).unwrap().health else {
        panic!()
    };
    assert_eq!(db.cause, Some(ErrorCause::Upstream(a)));
    assert!(ev.body(body(a)).is_none());
    // C is independent: still a hit.
    assert!(r.cached.contains(&c) && r.computed.is_empty());

    d.set_suppressed(a, false).unwrap();
    let back = run(&d, &cache);
    assert!(
        back.computed.is_empty(),
        "unsuppressing is served from cache"
    );
    for f in [a, b, c] {
        assert_eq!(key_of(&back.evaluated, f), key_of(&first.evaluated, f));
    }
}

#[test]
fn marker_rolls_back_later_nodes() {
    let mut d = Design::new("marker");
    let cache = GeomCache::new(1 << 30);
    let a = add(&mut d, counter(&[], 0, false));
    let b = add(&mut d, counter(&[a], 1, false));
    d.set_marker(1).unwrap();
    let r = run(&d, &cache);
    assert_eq!(r.computed, vec![a]);
    assert_eq!(r.evaluated.node(b).unwrap().health, Health::RolledBack);
    assert!(r.evaluated.body(body(b)).is_none());
    d.set_marker(2).unwrap();
    let r = run(&d, &cache);
    assert_eq!((r.cached, r.computed), (vec![a], vec![b]));
}

// ---- Done-when 3: keys are deterministic ----------------------------------------------------

fn fixed_design() -> (Design, Vec<FeatureId>) {
    let mut d = Design::new("keys");
    let a = add(&mut d, counter(&[], 7, false));
    let b = add(&mut d, counter(&[a], -3, false));
    let c = add(&mut d, counter(&[a, b], 0, false));
    (d, vec![a, b, c])
}

#[test]
fn keys_are_deterministic_and_independent_of_the_document_uuid() {
    let (d1, ids) = fixed_design();
    let (d2, _) = fixed_design();
    assert_ne!(d1.meta().uuid, d2.meta().uuid);
    let k1 = run(&d1, &GeomCache::new(1 << 30)).evaluated;
    let k2 = run(&d2, &GeomCache::new(1 << 30)).evaluated;
    let hex: Vec<String> = ids.iter().map(|f| key_of(&k1, *f).to_hex()).collect();
    for (f, h) in ids.iter().zip(&hex) {
        assert_eq!(&key_of(&k2, *f).to_hex(), h);
    }
    // Golden values: identical in every process and on every machine. If this fails after an
    // intended change to the key layout, bump ENGINE_FORMAT_VERSION and update the values.
    assert_eq!(hex, GOLDEN, "cache keys changed; see comment");
}

const GOLDEN: [&str; 3] = [
    "0dafe2e4e1ebf1782762010135fadbe9b7d3730606a72f3bef138687427b2c53",
    "c3f298ea0a8057ee0e67e922a9d83e606443fb4427f0faf52516dfc7be0b6f98",
    "855b530f1143ead506e85b7af9910f183d2f015e6a25cdb6075216acb3de7617",
];

#[test]
fn mode_and_tolerance_are_part_of_the_key() {
    let (d, ids) = fixed_design();
    let job = JobCtx::detached();
    let full = recompute(&d, &GeomCache::new(1 << 30), ComputeMode::Full, &job).unwrap();
    let prev = recompute(&d, &GeomCache::new(1 << 30), ComputeMode::Preview, &job).unwrap();
    assert_ne!(
        key_of(&full.evaluated, ids[0]),
        key_of(&prev.evaluated, ids[0])
    );
    let mut tol = d.meta().tolerance;
    let salt = amg_doc::engine_salt(&tol);
    tol.linear *= 2.0;
    assert_ne!(salt, amg_doc::engine_salt(&tol));
    // Preview and full outputs live side by side in one cache.
    let cache = GeomCache::new(1 << 30);
    recompute(&d, &cache, ComputeMode::Full, &job).unwrap();
    let p = recompute(&d, &cache, ComputeMode::Preview, &job).unwrap();
    assert_eq!(p.computed.len(), ids.len());
    let again = recompute(&d, &cache, ComputeMode::Full, &job).unwrap();
    assert!(again.computed.is_empty());
}

// ---- parameters -----------------------------------------------------------------------------

/// A feature with one expression field and no inputs, to test parameter dependencies.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct WithParam {
    w: Expr,
}

static PARAM_COMPUTES: Mutex<BTreeMap<u64, u64>> = Mutex::new(BTreeMap::new());

impl Feature for WithParam {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }
    fn exprs(&self) -> Vec<&Expr> {
        vec![&self.w]
    }
    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let w = ctx.eval_length(&self.w)?;
        assert!(w.is_finite());
        *PARAM_COMPUTES
            .lock()
            .unwrap()
            .entry(ctx.feature_id().0)
            .or_default() += 1;
        Ok(FeatureOutput::default())
    }
}

amg_doc::register_feature!(WithParam, "test.with_param", version = 1);

#[test]
fn parameter_values_are_part_of_the_key() {
    let mut d = Design::new("params");
    let cache = GeomCache::new(1 << 30);
    let p = d.add_param("w", "10 mm", None, "").unwrap();
    let q = d.add_param("other", "5 mm", None, "").unwrap();
    let f = add(&mut d, FeatureBox::new(WithParam { w: Expr::Param(p) }));
    let g = add(&mut d, counter(&[], 0, false));
    let n = |id: FeatureId| *PARAM_COMPUTES.lock().unwrap().get(&id.0).unwrap_or(&0);

    let r0 = run(&d, &cache);
    assert_eq!(n(f), 1);
    run(&d, &cache);
    assert_eq!(n(f), 1);

    // Editing an unrelated parameter keeps the cache.
    d.set_param_expr(q, "6 mm").unwrap();
    assert!(run(&d, &cache).computed.is_empty());

    // Editing the used parameter recomputes that feature only.
    d.set_param_expr(p, "12 mm").unwrap();
    let r1 = run(&d, &cache);
    assert_eq!(r1.computed, vec![f]);
    assert_eq!(n(f), 2);
    assert_ne!(key_of(&r0.evaluated, f), key_of(&r1.evaluated, f));
    assert_eq!(key_of(&r0.evaluated, g), key_of(&r1.evaluated, g));

    // The same value reached by a different expression (and unit) hits.
    d.set_param_expr(p, "1.2 cm").unwrap();
    assert!(run(&d, &cache).computed.is_empty());
    d.set_param_expr(p, "10 mm").unwrap();
    assert!(
        run(&d, &cache).computed.is_empty(),
        "back to the first value"
    );
}

#[test]
fn negative_zero_normalises() {
    let key = |expr: &str| {
        let mut d = Design::new("z");
        let p = d.add_param("w", expr, None, "").unwrap();
        let f = add(&mut d, FeatureBox::new(WithParam { w: Expr::Param(p) }));
        key_of(&run(&d, &GeomCache::new(1 << 20)).evaluated, f)
    };
    assert_eq!(key("0 mm"), key("-0 mm"));
}

#[test]
fn missing_parameter_is_an_expr_error() {
    let mut d = Design::new("noparam");
    let f = add(
        &mut d,
        FeatureBox::new(WithParam {
            w: Expr::Param(amg_doc::ParamId(999_999)),
        }),
    );
    let r = run(&d, &GeomCache::new(1 << 20));
    let Health::Error(diag) = &r.evaluated.node(f).unwrap().health else {
        panic!()
    };
    assert_eq!(
        diag.cause,
        Some(ErrorCause::Expr(amg_doc::ParamId(999_999)))
    );
}

// ---- cancellation ---------------------------------------------------------------------------

static CANCEL: Mutex<Option<CancelToken>> = Mutex::new(None);

/// Cancels the running job when computed (once armed), then returns normally.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Canceller {
    n: i64,
}

impl Feature for Canceller {
    fn inputs(&self) -> Vec<Dependency> {
        Vec::new()
    }
    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        if let Some(t) = CANCEL.lock().unwrap().as_ref() {
            t.cancel();
        }
        ctx.check_cancelled()?;
        Ok(FeatureOutput::default())
    }
}

amg_doc::register_feature!(Canceller, "test.canceller", version = 1);

#[test]
fn cancellation_keeps_finished_features_cached() {
    let mut d = Design::new("cancel");
    let cache = GeomCache::new(1 << 30);
    let a = add(&mut d, counter(&[], 0, false));
    let b = add(&mut d, counter(&[a], 1, false));
    let x = add(&mut d, FeatureBox::new(Canceller { n: 0 }));
    let c = add(&mut d, counter(&[b], 2, false));

    // Pre-cancelled token: nothing runs.
    let token = CancelToken::new();
    token.cancel();
    let pre = recompute(&d, &cache, ComputeMode::Full, &JobCtx::with_token(token));
    assert!(pre.is_err());
    assert_eq!(counts(&d, &[a]), vec![0]);

    // Cancelled while computing X: A and B are finished and stay cached; C never runs.
    let token = CancelToken::new();
    *CANCEL.lock().unwrap() = Some(token.clone());
    let r = recompute(&d, &cache, ComputeMode::Full, &JobCtx::with_token(token));
    *CANCEL.lock().unwrap() = None;
    assert!(r.is_err());
    assert_eq!(counts(&d, &[a, b, c]), vec![1, 1, 0]);

    // The superseding pass reuses A and B.
    let r = run(&d, &cache);
    assert_eq!(r.cached, vec![a, b]);
    assert_eq!(r.computed, vec![x, c]);
}
