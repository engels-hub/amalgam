//! CMP-03 Done-when 1–3: sparse direct solve of 3-D Q1 elasticity on the reference machine.
//!
//! 1. `q1_elasticity(55, Clamped)` (499 125 DOF): analyse + factor + solve ≤ 20 s (S13 note §8.1
//!    split of the vision 30 s budget), relative residual ≤ 1e-10 (always asserted).
//! 2. `q1_elasticity(70, Clamped)` (1 029 000 DOF), only with `AMG_BENCH_1M=1`: time, process
//!    memory before/after the factorisation, and `estimate().bytes` within 10 % of the measured
//!    factor storage (always asserted when run). Peak RSS is sampled from outside the process
//!    (see the Refinement log).
//! 3. `solve_many` with 6 right-hand sides ≤ 2× one `solve_in_place` (best of 5 each).
//!
//! Timing bars are asserted only under `AMG_STRICT_TIMING=1` (release build, idle machine).
//! The criterion part repeats analyse + factor + solve at 500 k DOF (10 samples). Not run in CI.
//! `AMG_BENCH_ORDERING=nd` replaces AMD with a geometric nested dissection of the node grid
//! (input for CMP-10; about 2.2x less factor memory at 500 k).

#![allow(clippy::unwrap_used, clippy::expect_used, reason = "benchmark code")]

use std::hint::black_box;
use std::time::{Duration, Instant};

use amg_compute::sparse::{Cholesky, FactorOpts, Ordering, SymCsr, SymbolicFactor};
use amg_compute::testprob::{Support, q1_elasticity};
use amg_jobs::JobCtx;
use criterion::{Criterion, criterion_group, criterion_main};

fn strict() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// Geometric nested dissection of an `m³` node grid, 3 DOF per node: both halves first, then
/// the separator plane (same as the helper in `tests/sparse_direct.rs`).
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

/// Factor options for an `nodes³` grid: AMD, or ND with `AMG_BENCH_ORDERING=nd`.
fn opts(nodes: usize) -> (FactorOpts, &'static str) {
    if std::env::var("AMG_BENCH_ORDERING").is_ok_and(|v| v == "nd") {
        let ordering = Ordering::Custom(grid_nested_dissection(nodes));
        (
            FactorOpts {
                ordering,
                ..Default::default()
            },
            "ND",
        )
    } else {
        (FactorOpts::default(), "AMD")
    }
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

fn gib(bytes: f64) -> f64 {
    bytes / f64::from(1u32 << 30)
}

fn mem() -> (f64, f64) {
    memory_stats::memory_stats().map_or((f64::NAN, f64::NAN), |m| {
        (m.physical_mem as f64, m.virtual_mem as f64)
    })
}

struct Run {
    analyse: Duration,
    factor: Duration,
    solve: Duration,
    residual: f64,
}

/// One analyse + factor + solve, timed per phase; keeps the factor for further solves.
fn run(k: &SymCsr, f: &[f64], opts: &FactorOpts) -> (Run, Cholesky, (f64, f64, f64, f64)) {
    let ctx = JobCtx::detached();
    let t0 = Instant::now();
    let sym = SymbolicFactor::analyze(k, opts).unwrap();
    let analyse = t0.elapsed();
    let before = mem();
    let t1 = Instant::now();
    let chol = Cholesky::factor(&sym, k, &ctx).unwrap();
    let factor = t1.elapsed();
    let after = mem();
    let mut x = f.to_vec();
    let t2 = Instant::now();
    chol.solve_in_place(&mut x);
    let solve = t2.elapsed();
    let residual = rel_residual(k, &x, f);
    (
        Run {
            analyse,
            factor,
            solve,
            residual,
        },
        chol,
        (before.0, before.1, after.0, after.1),
    )
}

fn best_of(reps: usize, mut f: impl FnMut()) -> Duration {
    (0..reps)
        .map(|_| {
            let t = Instant::now();
            f();
            t.elapsed()
        })
        .min()
        .unwrap()
}

fn report(label: &str, k: &SymCsr, r: &Run, sym: &SymbolicFactor) {
    let e = sym.estimate();
    let total = r.analyse + r.factor + r.solve;
    eprintln!(
        "{label}: n = {}, nnz(upper) = {}, factor nnz = {} ({:.2} GiB est.), flops = {:.3e}",
        k.n(),
        k.nnz(),
        e.factor_nnz,
        gib(e.bytes as f64),
        e.flops
    );
    eprintln!(
        "{label}: analyse {:.2} s + factor {:.2} s + solve {:.3} s = {:.2} s; residual {:.2e}; threads {}",
        r.analyse.as_secs_f64(),
        r.factor.as_secs_f64(),
        r.solve.as_secs_f64(),
        total.as_secs_f64(),
        r.residual,
        rayon::current_num_threads()
    );
}

fn elasticity_500k(c: &mut Criterion) {
    let (k, _, f) = q1_elasticity(55, Support::Clamped, 0.3);
    assert_eq!(k.n(), 499_125);
    let (o, name) = opts(55);
    eprintln!("500k: ordering {name}");
    let (r, chol, _) = run(&k, &f, &o);
    report("500k", &k, &r, chol.symbolic());
    assert!(r.residual <= 1e-10, "residual {:e} above 1e-10", r.residual);

    // Done-when 3: six right-hand sides at once vs one.
    let n = k.n();
    let six: Vec<f64> = (0..6)
        .flat_map(|c| f.iter().map(move |v| v * (c + 1) as f64))
        .collect();
    let one = best_of(5, || {
        let mut x = f.clone();
        chol.solve_in_place(&mut x);
        black_box(&x);
    });
    let many = best_of(5, || {
        let mut x = six.clone();
        chol.solve_many(&mut x, 6);
        black_box(&x);
    });
    let mut x = six.clone();
    chol.solve_many(&mut x, 6);
    for col in 0..6 {
        let rr = rel_residual(&k, &x[col * n..(col + 1) * n], &six[col * n..(col + 1) * n]);
        assert!(rr <= 1e-10, "solve_many column {col}: residual {rr:e}");
    }
    let ratio = many.as_secs_f64() / one.as_secs_f64();
    eprintln!(
        "500k: solve_in_place {:.1} ms, solve_many(6) {:.1} ms, ratio {ratio:.2} (bar 2.0)",
        one.as_secs_f64() * 1e3,
        many.as_secs_f64() * 1e3
    );
    drop(chol);

    let total = r.analyse + r.factor + r.solve;
    if strict() {
        assert!(
            total <= Duration::from_secs(20),
            "analyse + factor + solve took {total:?} (budget 20 s)"
        );
        assert!(
            ratio <= 2.0,
            "solve_many(6) is {ratio:.2}x one solve (bar 2x)"
        );
    }

    let mut g = c.benchmark_group("sparse_direct");
    g.sample_size(10)
        .warm_up_time(Duration::from_millis(1))
        .measurement_time(Duration::from_secs(1));
    g.bench_function("elasticity_55_clamped_analyse_factor_solve", |b| {
        b.iter(|| {
            let (r, chol, _) = run(&k, &f, &o);
            black_box((r.residual, chol.pivots()));
        });
    });
    g.finish();
}

/// Done-when 2; opt-in (`AMG_BENCH_1M=1`): needs ≈ 10–20 GB of free memory.
fn elasticity_1m(_: &mut Criterion) {
    if !std::env::var("AMG_BENCH_1M").is_ok_and(|v| v == "1") {
        eprintln!("1M: skipped (set AMG_BENCH_1M=1)");
        return;
    }
    let t = Instant::now();
    let (k, _, f) = q1_elasticity(70, Support::Clamped, 0.3);
    assert_eq!(k.n(), 1_029_000);
    let (o, name) = opts(70);
    eprintln!("1M: ordering {name}");
    eprintln!("1M: assembled in {:.2} s", t.elapsed().as_secs_f64());
    let (r, chol, (ws0, pv0, ws1, pv1)) = run(&k, &f, &o);
    report("1M", &k, &r, chol.symbolic());
    let est = chol.symbolic().estimate().bytes as f64;
    let (dws, dpv) = (ws1 - ws0, pv1 - pv0);
    eprintln!(
        "1M: process working set {:.2} -> {:.2} GiB, private {:.2} -> {:.2} GiB; factor storage measured {:.3} GiB (private delta), {:.3} GiB (working-set delta); estimate {:.3} GiB ({:+.2} % of private delta)",
        gib(ws0),
        gib(ws1),
        gib(pv0),
        gib(pv1),
        gib(dpv),
        gib(dws),
        gib(est),
        (est / dpv - 1.0) * 100.0
    );
    assert!(r.residual <= 1e-10, "residual {:e} above 1e-10", r.residual);
    assert!(
        (est - dpv).abs() <= 0.1 * dpv,
        "estimate {est} not within 10 % of the measured {dpv}"
    );
}

criterion_group!(benches, elasticity_500k, elasticity_1m);
criterion_main!(benches);
