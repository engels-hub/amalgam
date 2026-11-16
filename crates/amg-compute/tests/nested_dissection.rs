//! CMP-10 geometric nested-dissection ordering. The 500 k DOF cases (Done-when 1 and 2) are in
//! `large_nd_estimates` (ignored, analysis only) and `benches/sparse_direct.rs` (factor time).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]
#![allow(clippy::float_cmp, reason = "copied option values")]

use amg_compute::ComputeError;
use amg_compute::sparse::{
    Cholesky, FactorEstimate, FactorOpts, Ordering, SymCsr, SymbolicFactor, nested_dissection,
};
use amg_compute::testprob::{Support, grid_coords, q1_elasticity, tet10_bracket};
use amg_jobs::JobCtx;

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

fn opts(ordering: Ordering) -> FactorOpts {
    FactorOpts {
        ordering,
        ..Default::default()
    }
}

fn estimate(a: &SymCsr, ordering: Ordering) -> FactorEstimate {
    SymbolicFactor::analyze(a, &opts(ordering))
        .unwrap()
        .estimate()
}

fn solve(a: &SymCsr, b: &[f64], ordering: Ordering) -> Vec<f64> {
    let sym = SymbolicFactor::analyze(a, &opts(ordering)).unwrap();
    let c = Cholesky::factor(&sym, a, &JobCtx::detached()).unwrap();
    assert!(!c.pivots().below_threshold);
    let mut x = b.to_vec();
    c.solve_in_place(&mut x);
    x
}

fn is_permutation(p: &[u32], n: usize) -> bool {
    let mut seen = vec![false; n];
    p.len() == n
        && p.iter()
            .all(|&v| (v as usize) < n && !std::mem::replace(&mut seen[v as usize], true))
}

#[test]
fn nd_solves_like_amd() {
    let (k, _, f) = q1_elasticity(12, Support::Clamped, 0.3);
    let nd = Ordering::NestedDissection {
        coords: grid_coords(12, 3),
    };
    let (xa, xn) = (solve(&k, &f, Ordering::Amd), solve(&k, &f, nd));
    assert!(rel_residual(&k, &xn, &f) <= 1e-10);
    let diff: Vec<f64> = xa.iter().zip(&xn).map(|(a, b)| a - b).collect();
    assert!(norm(&diff) <= 1e-9 * norm(&xa));

    let b = tet10_bracket(3);
    let x = solve(&b.k, &b.f, Ordering::NestedDissection { coords: b.coords });
    assert!(rel_residual(&b.k, &x, &b.f) <= 1e-10);
}

/// The fill reduction already shows at moderate sizes (Done-when 1 at 500 k is measured by
/// `large_nd_estimates`).
#[test]
fn nd_has_less_fill_than_amd() {
    let (k, _, _) = q1_elasticity(24, Support::Clamped, 0.3);
    let amd = estimate(&k, Ordering::Amd);
    let nd = estimate(
        &k,
        Ordering::NestedDissection {
            coords: grid_coords(24, 3),
        },
    );
    eprintln!("q1_elasticity(24): AMD {amd:?}, ND {nd:?}");
    assert!(nd.factor_nnz < amd.factor_nnz);
    assert!(nd.flops < amd.flops);

    // On the thin-walled bracket ND overtakes AMD in non-zeros only at larger sizes (at
    // 12 k DOF it has 6 % more; at 0.5 M 38 % less), in flops already here.
    let b = tet10_bracket(6);
    let amd = estimate(&b.k, Ordering::Amd);
    let nd = estimate(&b.k, Ordering::NestedDissection { coords: b.coords });
    eprintln!("tet10_bracket(6), n = {}: AMD {amd:?}, ND {nd:?}", b.k.n());
    assert!(nd.flops < 0.9 * amd.flops);
}

/// The permutation depends only on the pattern and the coordinates, not on the thread count
/// (the dissection recurses in parallel above 20 000 rows).
#[test]
fn nd_is_deterministic_across_thread_counts() {
    let b = tet10_bracket(6);
    assert!(b.k.n() > 30_000);
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| nested_dissection(&b.k, &b.coords).unwrap())
    };
    let p1 = run(1);
    assert!(is_permutation(&p1, b.k.n()));
    for threads in [4, 16] {
        assert_eq!(run(threads), p1, "{threads} threads");
    }
    let sym = SymbolicFactor::analyze(
        &b.k,
        &opts(Ordering::NestedDissection {
            coords: b.coords.clone(),
        }),
    )
    .unwrap();
    assert_eq!(sym.permutation(), p1.as_slice());
}

#[test]
fn node_constructor_repeats_positions() {
    let nodes = [[0.0, 1.0, 2.0], [3.0, 4.0, 5.0]];
    let o = Ordering::nested_dissection(&nodes, 3);
    let Ordering::NestedDissection { coords } = &o else {
        panic!("wrong variant");
    };
    assert_eq!(coords.len(), 6);
    assert_eq!(coords[2], nodes[0]);
    assert_eq!(coords[3], nodes[1]);
    assert_eq!(o, o.clone());
    assert_ne!(o, Ordering::nested_dissection(&nodes, 1));
    assert_ne!(o, Ordering::Amd);
    let fo = FactorOpts::for_mesh(&nodes, 3);
    assert_eq!(fo.ordering, o);
    assert_eq!(fo.pivot_warn, FactorOpts::default().pivot_warn);
}

#[test]
fn bad_coordinates_are_rejected() {
    let (k, _, _) = q1_elasticity(3, Support::Clamped, 0.3);
    let mut coords = grid_coords(3, 3);
    coords.pop();
    let r = SymbolicFactor::analyze(&k, &opts(Ordering::NestedDissection { coords }));
    assert!(matches!(r, Err(ComputeError::InvalidMatrix(_))));
    let mut coords = grid_coords(3, 3);
    coords[7][0] = f64::INFINITY;
    let r = SymbolicFactor::analyze(&k, &opts(Ordering::NestedDissection { coords }));
    assert!(matches!(r, Err(ComputeError::InvalidMatrix(_))));
}

/// Done-when 1 (analysis only): factor non-zeros of AMD and ND on `q1_elasticity(55)` and the
/// 0.5 M DOF `tet10_bracket(15)`; ND must be ≥ 25 % lower. Needs ≈ 4 GB; run explicitly with
/// `--run-ignored only -E 'test(large_nd_estimates)'`.
#[test]
#[ignore = "large: analysis of two 0.5 M DOF problems"]
fn large_nd_estimates() {
    let gib = |b: u64| b as f64 / f64::from(1u32 << 30);
    let (k, _, _) = q1_elasticity(55, Support::Clamped, 0.3);
    let grid = ("q1_elasticity(55)", k, grid_coords(55, 3));
    let b = tet10_bracket(15);
    let bracket = ("tet10_bracket(15)", b.k, b.coords);
    for (name, k, coords) in [grid, bracket] {
        let mut est = Vec::new();
        for (oname, ordering) in [
            ("AMD", Ordering::Amd),
            ("ND", Ordering::NestedDissection { coords }),
        ] {
            let t = std::time::Instant::now();
            let e = estimate(&k, ordering);
            eprintln!(
                "{name} (n = {}, nnz upper = {}), {oname}: factor_nnz = {} ({:.2} GiB), flops = {:.3e}, analysis {:.2} s",
                k.n(),
                k.nnz(),
                e.factor_nnz,
                gib(e.bytes),
                e.flops,
                t.elapsed().as_secs_f64()
            );
            est.push(e);
        }
        let reduction = 1.0 - est[1].factor_nnz as f64 / est[0].factor_nnz as f64;
        eprintln!("{name}: ND factor nnz {:.1} % below AMD", reduction * 100.0);
        assert!(reduction >= 0.25, "{name}: only {:.1} %", reduction * 100.0);
    }
}
