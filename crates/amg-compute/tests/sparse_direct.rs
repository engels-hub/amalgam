//! CMP-03 sparse direct solver tests (CPU only). The 500 k and 1 M DOF cases (Done-when 1–3)
//! are in `benches/sparse_direct.rs`; here the same properties are checked on small problems.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]
#![allow(clippy::float_cmp, reason = "bit-identical results are the point")]

use std::sync::Arc;

use amg_compute::ComputeError;
use amg_compute::sparse::{
    Cholesky, FactorOpts, Ldlt, Ordering, SymCsr, SymbolicFactor, TripletBuilder,
    analyses_on_this_thread,
};
use amg_compute::testprob::{Support, q1_elasticity, q1_poisson, simp_field};
use amg_jobs::{CancelToken, JobCtx};
use faer::{Mat, Side};
use proptest::prelude::*;

fn job() -> JobCtx {
    JobCtx::detached()
}

fn norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

fn rel_residual(a: &SymCsr, x: &[f64], b: &[f64]) -> f64 {
    let mut r = vec![0.0; a.n()];
    a.spmv(x, &mut r);
    for (ri, bi) in r.iter_mut().zip(b) {
        *ri -= bi;
    }
    norm(&r) / norm(b)
}

fn bits(v: &[f64]) -> Vec<u64> {
    v.iter().map(|x| x.to_bits()).collect()
}

fn cholesky_solve(a: &SymCsr, b: &[f64], opts: &FactorOpts) -> Vec<f64> {
    let sym = SymbolicFactor::analyze(a, opts).unwrap();
    let f = Cholesky::factor(&sym, a, &job()).unwrap();
    let mut x = b.to_vec();
    f.solve_in_place(&mut x);
    x
}

#[test]
fn cholesky_solves_elasticity_and_poisson() {
    let (k, _, f) = q1_elasticity(10, Support::Clamped, 0.3);
    let x = cholesky_solve(&k, &f, &FactorOpts::default());
    let r = rel_residual(&k, &x, &f);
    assert!(r <= 1e-12, "elasticity residual {r:e}");

    let coeff = simp_field(11, 1e3, 3);
    let (p, f) = q1_poisson(12, Some(&coeff));
    let x = cholesky_solve(&p, &f, &FactorOpts::default());
    let r = rel_residual(&p, &x, &f);
    assert!(r <= 1e-11, "poisson residual {r:e}");
}

#[test]
fn every_ordering_gives_the_same_solution() {
    let (k, _, f) = q1_elasticity(6, Support::Clamped, 0.3);
    let amd = cholesky_solve(&k, &f, &FactorOpts::default());
    let ident = cholesky_solve(
        &k,
        &f,
        &FactorOpts {
            ordering: Ordering::Identity,
            ..Default::default()
        },
    );
    let reversed: Vec<u32> = (0..k.n() as u32).rev().collect();
    let custom = cholesky_solve(
        &k,
        &f,
        &FactorOpts {
            ordering: Ordering::Custom(reversed.clone()),
            ..Default::default()
        },
    );
    let scale = norm(&amd);
    for x in [&ident, &custom] {
        let d: Vec<f64> = x.iter().zip(&amd).map(|(a, b)| a - b).collect();
        assert!(norm(&d) <= 1e-10 * scale);
    }
    let sym = SymbolicFactor::analyze(
        &k,
        &FactorOpts {
            ordering: Ordering::Custom(reversed.clone()),
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(sym.permutation(), &reversed[..]);
}

#[test]
fn invalid_custom_ordering_is_rejected() {
    let (k, _) = q1_poisson(4, None);
    for p in [vec![0u32; k.n()], (0..k.n() as u32 - 1).collect()] {
        let opts = FactorOpts {
            ordering: Ordering::Custom(p),
            ..Default::default()
        };
        assert!(matches!(
            SymbolicFactor::analyze(&k, &opts),
            Err(ComputeError::InvalidMatrix(_))
        ));
    }
    let empty = TripletBuilder::new(0).build().unwrap();
    assert!(matches!(
        SymbolicFactor::analyze(&empty, &FactorOpts::default()),
        Err(ComputeError::Shape(_))
    ));
}

/// Done-when 3 (correctness part; the timing is in the bench).
#[test]
fn solve_many_matches_single_solves() {
    let (k, _, f) = q1_elasticity(8, Support::Clamped, 0.3);
    let n = k.n();
    let sym = SymbolicFactor::analyze(&k, &FactorOpts::default()).unwrap();
    let chol = Cholesky::factor(&sym, &k, &job()).unwrap();
    let rhs: Vec<Vec<f64>> = (0..6)
        .map(|c| {
            f.iter()
                .enumerate()
                .map(|(i, v)| v * (c + 1) as f64 + ((i * 7 + c) % 11) as f64 * 1e-3)
                .collect()
        })
        .collect();
    let mut many: Vec<f64> = rhs.concat();
    chol.solve_many(&mut many, 6);
    for (c, b) in rhs.iter().enumerate() {
        let mut x = b.clone();
        chol.solve_in_place(&mut x);
        let got = &many[c * n..(c + 1) * n];
        let d: Vec<f64> = x.iter().zip(got).map(|(a, b)| a - b).collect();
        assert!(norm(&d) <= 1e-13 * norm(&x), "rhs {c}");
        assert!(rel_residual(&k, got, b) <= 1e-12);
    }
}

/// Done-when 4: a free-free body gives an error with a row or a flagged pivot, never a panic.
#[test]
fn free_elasticity_is_reported_not_panicking() {
    for nodes in [2u32, 3, 4, 6, 9, 12] {
        let (k, _, _) = q1_elasticity(nodes, Support::Free, 0.3);
        let sym = SymbolicFactor::analyze(&k, &FactorOpts::default()).unwrap();
        match Cholesky::factor(&sym, &k, &job()) {
            Err(ComputeError::NotPositiveDefinite { row, .. }) => assert!(row < k.n()),
            Ok(c) => {
                let p = c.pivots();
                assert!(p.below_threshold, "nodes {nodes}: {p:?}");
                assert!(p.row < k.n());
            }
            Err(e) => panic!("nodes {nodes}: unexpected error {e}"),
        }
        // The same pattern clamped is well conditioned.
        let (kc, _, _) = q1_elasticity(nodes, Support::Clamped, 0.3);
        let c = Cholesky::factor(&sym, &kc, &job()).unwrap();
        assert!(
            !c.pivots().below_threshold,
            "nodes {nodes}: {:?}",
            c.pivots()
        );
    }
}

#[test]
fn indefinite_matrix_reports_its_row() {
    let mut t = TripletBuilder::new(3);
    t.push(0, 0, 4.0);
    t.push(1, 1, -1.0);
    t.push(2, 2, 2.0);
    t.push(0, 2, 1.0);
    let a = t.build().unwrap();
    let sym = SymbolicFactor::analyze(
        &a,
        &FactorOpts {
            ordering: Ordering::Identity,
            ..Default::default()
        },
    )
    .unwrap();
    match Cholesky::factor(&sym, &a, &job()) {
        Err(ComputeError::NotPositiveDefinite { row, pivot }) => {
            assert_eq!(row, 1);
            assert!(pivot.is_nan());
        }
        other => panic!("expected NotPositiveDefinite, got {other:?}"),
    }
    let ldlt = Ldlt::factor(&sym, &a, &job()).unwrap();
    let i = ldlt.inertia();
    assert_eq!((i.negative, i.zero, i.positive), (1, 0, 2));
}

/// Done-when 6: re-factoring on a fixed pattern does not analyse again.
#[test]
fn refactoring_reuses_the_analysis() {
    let (k, m, f) = q1_elasticity(7, Support::Clamped, 0.3);
    let before = analyses_on_this_thread();
    let sym = SymbolicFactor::analyze(&k, &FactorOpts::default()).unwrap();
    assert_eq!(analyses_on_this_thread(), before + 1);
    let shifted = k.add_scaled(-1e-3, &m).unwrap();
    assert!(shifted.same_pattern(&k));
    let mut stiffer = k.clone();
    for v in stiffer.vals_mut() {
        *v *= 2.0;
    }
    let c1 = Cholesky::factor(&sym, &k, &job()).unwrap();
    let c2 = Cholesky::factor(&sym, &stiffer, &job()).unwrap();
    let l = Ldlt::factor(&sym, &shifted, &job()).unwrap();
    assert_eq!(analyses_on_this_thread(), before + 1);
    assert_eq!(sym.numeric_factorizations(), 3);
    assert!(Arc::ptr_eq(c1.symbolic(), c2.symbolic()));
    assert!(Arc::ptr_eq(c1.symbolic(), l.symbolic()));
    let (mut x1, mut x2) = (f.clone(), f.clone());
    c1.solve_in_place(&mut x1);
    c2.solve_in_place(&mut x2);
    let d: Vec<f64> = x1.iter().zip(&x2).map(|(a, b)| a - 2.0 * b).collect();
    assert!(norm(&d) <= 1e-12 * norm(&x1));
}

#[test]
fn factor_rejects_a_different_pattern() {
    let (k, _, _) = q1_elasticity(4, Support::Clamped, 0.3);
    let (p, _) = q1_poisson(5, None);
    let sym = SymbolicFactor::analyze(&k, &FactorOpts::default()).unwrap();
    assert!(matches!(
        Cholesky::factor(&sym, &p, &job()),
        Err(ComputeError::InvalidMatrix(_))
    ));
    assert!(matches!(
        Ldlt::factor(&sym, &p, &job()),
        Err(ComputeError::InvalidMatrix(_))
    ));
}

#[test]
fn cancelled_job_stops_before_factoring() {
    let (k, _, _) = q1_elasticity(4, Support::Clamped, 0.3);
    let sym = SymbolicFactor::analyze(&k, &FactorOpts::default()).unwrap();
    let token = CancelToken::new();
    token.cancel();
    let ctx = JobCtx::with_token(token);
    let e = Cholesky::factor(&sym, &k, &ctx).unwrap_err();
    assert!(matches!(e, ComputeError::Cancelled(_)));
    assert!(matches!(
        Ldlt::factor(&sym, &k, &ctx),
        Err(ComputeError::Cancelled(_))
    ));
    assert_eq!(sym.numeric_factorizations(), 0);
}

#[test]
fn estimate_is_consistent() {
    let (k, _, _) = q1_elasticity(9, Support::Clamped, 0.3);
    let sym = SymbolicFactor::analyze(&k, &FactorOpts::default()).unwrap();
    let e = sym.estimate();
    assert!(e.factor_nnz >= k.nnz() as u64, "{e:?}");
    assert!(e.bytes >= e.factor_nnz * 8, "{e:?}");
    assert!(e.flops >= e.factor_nnz as f64, "{e:?}");
    let ident = SymbolicFactor::analyze(
        &k,
        &FactorOpts {
            ordering: Ordering::Identity,
            ..Default::default()
        },
    )
    .unwrap();
    // A banded grid ordering fills more than AMD on a 3-D grid.
    assert!(ident.estimate().factor_nnz > e.factor_nnz);
}

/// Done-when 7: factor and solve bit-identical over 2 runs (default threads and sequential).
#[test]
fn factor_and_solve_are_bit_identical_over_runs() {
    let (k, _, f) = q1_elasticity(14, Support::Clamped, 0.3);
    for threads in [None, Some(1), Some(4)] {
        let opts = FactorOpts {
            threads,
            ..Default::default()
        };
        let run = || {
            let sym = SymbolicFactor::analyze(&k, &opts).unwrap();
            let c = Cholesky::factor(&sym, &k, &job()).unwrap();
            let mut x = f.clone();
            c.solve_in_place(&mut x);
            let l = Ldlt::factor(&sym, &k, &job()).unwrap();
            let mut y = f.clone();
            l.solve_in_place(&mut y);
            (sym.permutation().to_vec(), c.pivots(), bits(&x), bits(&y))
        };
        let (a, b) = (run(), run());
        assert_eq!(a.0, b.0, "{threads:?}: permutation");
        assert_eq!(a.1, b.1, "{threads:?}: pivots");
        assert!(a.2 == b.2, "{threads:?}: Cholesky solution differs");
        assert!(a.3 == b.3, "{threads:?}: LBLT solution differs");
    }
}

/// Done-when 7: `spmv` bit-identical across 1, 4 and 16 threads.
#[test]
fn spmv_is_bit_identical_across_thread_counts() {
    let (k, _, _) = q1_elasticity(20, Support::Clamped, 0.3);
    let x: Vec<f64> = (0..k.n())
        .map(|i| ((i * 37) % 101) as f64 / 7.0 - 3.0)
        .collect();
    let mut results = Vec::new();
    for threads in [1, 4, 16] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        let mut y = vec![0.0; k.n()];
        pool.install(|| k.spmv(&x, &mut y));
        results.push(bits(&y));
    }
    assert!(results[0] == results[1] && results[1] == results[2]);
}

#[test]
fn ldlt_solves_an_indefinite_shifted_pencil() {
    let (k, m, _) = q1_elasticity(6, Support::Clamped, 0.3);
    // A shift inside the spectrum makes K − σM indefinite.
    let shifted = k.add_scaled(-50.0, &m).unwrap();
    let sym = SymbolicFactor::analyze(&shifted, &FactorOpts::default()).unwrap();
    let l = Ldlt::factor(&sym, &shifted, &job()).unwrap();
    let inertia = l.inertia();
    assert!(inertia.negative > 0 && inertia.positive > 0, "{inertia:?}");
    assert_eq!(inertia.negative + inertia.zero + inertia.positive, k.n());
    let b: Vec<f64> = (0..k.n()).map(|i| ((i % 13) as f64) - 6.0).collect();
    let mut x = b.clone();
    l.solve_in_place(&mut x);
    let r = rel_residual(&shifted, &x, &b);
    assert!(r <= 1e-9, "residual {r:e}");
    assert!(matches!(
        Cholesky::factor(&sym, &shifted, &job()),
        Err(ComputeError::NotPositiveDefinite { .. })
    ));
}

/// Eigenvalue sign counts of the dense symmetric matrix, or `None` if one is too close to 0
/// to be classified robustly.
fn dense_signs(a: &SymCsr) -> Option<(usize, usize)> {
    let n = a.n();
    let d = Mat::<f64>::from_fn(n, n, |i, j| a.get(i, j));
    let ev = d.self_adjoint_eigenvalues(Side::Lower).unwrap();
    let max = ev.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    if ev.iter().any(|v| v.abs() <= 1e-8 * max.max(1e-300)) {
        return None;
    }
    Some((
        ev.iter().filter(|&&v| v < 0.0).count(),
        ev.iter().filter(|&&v| v > 0.0).count(),
    ))
}

/// Random diagonal plus a low-rank update: `diag(d) + Σ s_k u_k u_kᵀ` with sparse `u_k`.
fn diag_plus_low_rank() -> impl Strategy<Value = SymCsr> {
    (2usize..60).prop_flat_map(|n| {
        (
            prop::collection::vec(-10.0f64..10.0, n),
            prop::collection::vec(
                (
                    prop::bool::ANY,
                    prop::collection::vec((0..n, -3.0f64..3.0), 1..6),
                ),
                0..4,
            ),
        )
            .prop_map(move |(d, updates)| {
                let mut dense = vec![vec![0.0f64; n]; n];
                for (i, v) in d.iter().enumerate() {
                    dense[i][i] = *v;
                }
                for (neg, entries) in updates {
                    let s = if neg { -1.0 } else { 1.0 };
                    let mut u = vec![0.0f64; n];
                    for (i, v) in entries {
                        u[i] += v;
                    }
                    for i in 0..n {
                        for j in 0..n {
                            dense[i][j] += s * u[i] * u[j];
                        }
                    }
                }
                let mut t = TripletBuilder::new(n);
                for (i, row) in dense.iter().enumerate() {
                    for (j, &v) in row.iter().enumerate().skip(i) {
                        if i == j || v != 0.0 {
                            t.push(i, j, v);
                        }
                    }
                }
                t.build().unwrap()
            })
    })
}

proptest! {
    #![proptest_config(ProptestConfig {
        failure_persistence: None,
        ..ProptestConfig::with_cases(96)
    })]

    /// Done-when 5: `Ldlt::inertia` equals the dense eigenvalue sign count.
    #[test]
    fn ldlt_inertia_matches_dense_eigenvalues(a in diag_plus_low_rank()) {
        let Some((neg, pos)) = dense_signs(&a) else {
            return Err(TestCaseError::reject("eigenvalue too close to zero"));
        };
        let sym = SymbolicFactor::analyze(&a, &FactorOpts::default()).unwrap();
        let l = Ldlt::factor(&sym, &a, &job()).unwrap();
        let i = l.inertia();
        prop_assert_eq!((i.negative, i.zero, i.positive), (neg, 0, pos));
    }

    /// Done-when 8 (property form): valid matrices are accepted; a swapped pair of columns,
    /// a lower entry or an out-of-range column is rejected.
    #[test]
    fn from_raw_accepts_valid_and_rejects_corrupted(
        n in 1usize..30,
        entries in prop::collection::vec((0usize..30, 0usize..30, -1.0f64..1.0), 0..80),
        which in 0usize..3,
    ) {
        let mut t = TripletBuilder::new(n);
        for (i, j, v) in entries {
            t.push(i % n, j % n, v);
        }
        let a = t.build().unwrap();
        let again = SymCsr::from_raw(n, a.row_ptr().to_vec(), a.cols().to_vec(), a.vals().to_vec());
        prop_assert!(again.is_ok());
        let mut cols = a.cols().to_vec();
        let rp = a.row_ptr();
        let long_row = (0..n).find(|&i| rp[i + 1] - rp[i] >= 2);
        let corrupted = match (which, long_row) {
            (0, Some(i)) => {
                cols.swap(rp[i], rp[i] + 1);
                true
            }
            (1, _) => {
                // A lower entry: a row > 0 whose first column is below the diagonal.
                match (1..n).find(|&i| rp[i + 1] > rp[i]) {
                    Some(i) => { cols[rp[i]] = (i - 1) as u32; true }
                    None => false,
                }
            }
            (2, _) if !cols.is_empty() => {
                let last = cols.len() - 1;
                cols[last] = n as u32;
                true
            }
            _ => false,
        };
        if corrupted {
            let r = SymCsr::from_raw(n, rp.to_vec(), cols, a.vals().to_vec());
            prop_assert!(matches!(r, Err(ComputeError::InvalidMatrix(_))));
        }
    }
}

/// Geometric nested dissection of an `m³` node grid, 3 DOF per node: both halves first, then
/// the separator plane (the input for the CMP-10 decision; estimate only).
fn grid_nested_dissection(m: usize) -> Vec<u32> {
    fn rec(lo: [usize; 3], hi: [usize; 3], m: usize, out: &mut Vec<u32>) {
        let ext = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
        if ext.contains(&0) {
            return;
        }
        let ax = (0..3).max_by_key(|&a| (ext[a], a)).unwrap();
        if ext[0] * ext[1] * ext[2] <= 64 || ext[ax] < 3 {
            for k in lo[2]..hi[2] {
                for j in lo[1]..hi[1] {
                    for i in lo[0]..hi[0] {
                        let p = i + m * (j + m * k);
                        out.extend((0..3).map(|c| (3 * p + c) as u32));
                    }
                }
            }
            return;
        }
        let mid = lo[ax] + ext[ax] / 2;
        let (mut h1, mut l2, mut slo, mut shi) = (hi, lo, lo, hi);
        h1[ax] = mid;
        l2[ax] = mid + 1;
        slo[ax] = mid;
        shi[ax] = mid + 1;
        rec(lo, h1, m, out);
        rec(l2, hi, m, out);
        rec(slo, shi, m, out);
    }
    let mut out = Vec::with_capacity(3 * m * m * m);
    rec([0; 3], [m; 3], m, &mut out);
    out
}

#[test]
fn grid_nested_dissection_is_a_permutation() {
    let (k, _, _) = q1_elasticity(9, Support::Clamped, 0.3);
    let p = grid_nested_dissection(9);
    let opts = FactorOpts {
        ordering: Ordering::Custom(p),
        ..Default::default()
    };
    let sym = SymbolicFactor::analyze(&k, &opts).unwrap();
    let c = Cholesky::factor(&sym, &k, &job()).unwrap();
    assert!(!c.pivots().below_threshold);
}

/// Analysis only (no factorisation) of the 500 k and 1 M DOF cases: prints `estimate()` for
/// AMD and for geometric nested dissection. Needs ≈ 3 GB; run explicitly with
/// `--run-ignored only -E 'test(large_analysis_estimates)'`.
#[test]
#[ignore = "large: analysis of 0.5 M and 1 M DOF problems"]
fn large_analysis_estimates() {
    let gib = |b: u64| b as f64 / f64::from(1u32 << 30);
    for nodes in [55u32, 70] {
        let (k, _, _) = q1_elasticity(nodes, Support::Clamped, 0.3);
        for (name, ordering) in [
            ("AMD", Ordering::Amd),
            (
                "ND",
                Ordering::Custom(grid_nested_dissection(nodes as usize)),
            ),
        ] {
            let t = std::time::Instant::now();
            let sym = SymbolicFactor::analyze(
                &k,
                &FactorOpts {
                    ordering,
                    ..Default::default()
                },
            )
            .unwrap();
            let e = sym.estimate();
            eprintln!(
                "nodes {nodes} (n = {}, nnz upper = {}), {name}: factor_nnz = {}, bytes = {} ({:.2} GiB), flops = {:.3e}, analysis {:.2} s",
                k.n(),
                k.nnz(),
                e.factor_nnz,
                e.bytes,
                gib(e.bytes),
                e.flops,
                t.elapsed().as_secs_f64()
            );
        }
    }
}
