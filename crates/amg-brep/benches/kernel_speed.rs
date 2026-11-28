//! S20 KH-20: the kernel calls behind the S04 budgets `budget/sol-extrude-cut` and
//! `budget/sol-shell-preview`, on the same inputs at kernel level
//! (`tests/common/speed_parts.rs`):
//!
//! - `cut-198-100-{plain,fast}`: the boolean cut of the 100-gon prism through the 198-gon prism
//!   (200 faces), then `unify_same_domain` as SOL-01's `simplify` runs it, with the KH-20 fast
//!   path (`KernelConfig::fast_paths`, the unify skip) off and on;
//! - `shell-star-100`: the star prism (100 faces), top removed, 0.3 mm inward, intersection join;
//! - `shell-plate-100`: the 120 × 120 × 10 plate with 94 holes, top removed, 0.3 mm inward.
//!
//! Before criterion runs, fixed 25-run samples print the medians of each call, and of the
//! boolean with OCCT's parallel mode off, with `check_inverted` off and with `use_obb` on, for
//! the record. With `AMG_STRICT_TIMING=1` the fast boolean + unify must take under 80 ms (the
//! whole extrude-cut budget). Validation is off, as in a release build's default policy.
#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::print_stdout,
    reason = "benchmark code"
)]

#[path = "../tests/common/speed_parts.rs"]
mod speed;

use std::hint::black_box;
use std::time::{Duration, Instant};

use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, JobCtx, KernelConfig, OcctKernel, OffsetJoin, OpResult, Shape,
    ShellOpts, UnifyOpts, Validate,
};
use criterion::{Criterion, criterion_group, criterion_main};

fn kernel(fast: bool) -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        fast_paths: fast,
        ..KernelConfig::default()
    })
}

fn median_of(mut f: impl FnMut()) -> Duration {
    f();
    let mut t: Vec<Duration> = (0..25)
        .map(|_| {
            let s = Instant::now();
            f();
            s.elapsed()
        })
        .collect();
    t.sort();
    t[t.len() / 2]
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1e3
}

fn cut(k: &OcctKernel, body: &Shape, tool: &Shape, opts: &BoolOpts) -> OpResult {
    k.boolean(
        BoolOp::Cut,
        std::slice::from_ref(body),
        std::slice::from_ref(tool),
        opts,
        &JobCtx::detached(),
    )
    .unwrap()
}

fn shell_opts() -> ShellOpts {
    ShellOpts {
        thickness: -0.3,
        join: OffsetJoin::Intersection,
        per_face: Vec::new(),
        tol: 0.0,
    }
}

fn bench(c: &mut Criterion) {
    let (plain, fast) = (kernel(false), kernel(true));
    let (body, tool) = speed::extrude_cut_pair(&fast);
    let star = speed::star_prism(&fast);
    let star_top = speed::cap_at(&fast, &star, 20.0);
    let plate = speed::holed_plate(&fast);
    let plate_top = speed::cap_at(&fast, &plate, 10.0);
    let opts = shell_opts();
    let ctx = JobCtx::detached();
    let unify = UnifyOpts::default();

    let cut_result = cut(&fast, &body, &tool, &BoolOpts::default());
    let boolean = median_of(|| {
        black_box(cut(&fast, &body, &tool, &BoolOpts::default()));
    });
    let serial = median_of(|| {
        black_box(cut(
            &fast,
            &body,
            &tool,
            &BoolOpts {
                parallel: Some(false),
                ..BoolOpts::default()
            },
        ));
    });
    let no_inverted = median_of(|| {
        black_box(cut(
            &fast,
            &body,
            &tool,
            &BoolOpts {
                check_inverted: false,
                ..BoolOpts::default()
            },
        ));
    });
    let obb = median_of(|| {
        black_box(cut(
            &fast,
            &body,
            &tool,
            &BoolOpts {
                use_obb: true,
                ..BoolOpts::default()
            },
        ));
    });
    let unify_plain = median_of(|| {
        black_box(plain.unify_same_domain(&cut_result.shape, &unify).unwrap());
    });
    let unify_fast = median_of(|| {
        black_box(fast.unify_same_domain(&cut_result.shape, &unify).unwrap());
    });
    let both_fast = median_of(|| {
        let r = cut(&fast, &body, &tool, &BoolOpts::default());
        black_box(fast.unify_same_domain(&r.shape, &unify).unwrap());
    });
    println!(
        "kh20 cut-198-100: boolean {:.2} ms (serial {:.2}, check_inverted off {:.2}, use_obb {:.2}); unify plain {:.2} ms, fast {:.2} ms; boolean + fast unify {:.2} ms (25 runs)",
        ms(boolean),
        ms(serial),
        ms(no_inverted),
        ms(obb),
        ms(unify_plain),
        ms(unify_fast),
        ms(both_fast)
    );
    for (name, s, top) in [("star", &star, star_top), ("plate", &plate, plate_top)] {
        let t = median_of(|| {
            black_box(fast.shell(s, &[top], &opts, &ctx).unwrap());
        });
        println!("kh20 shell-{name}-100: {:.2} ms (25 runs)", ms(t));
    }
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(
            both_fast < Duration::from_millis(80),
            "boolean + unify {both_fast:?} over 80 ms"
        );
    }

    let mut g = c.benchmark_group("kh20");
    g.sample_size(20);
    for (name, k) in [("plain", &plain), ("fast", &fast)] {
        g.bench_function(format!("cut-198-100-{name}"), |b| {
            b.iter(|| {
                let r = cut(k, &body, &tool, &BoolOpts::default());
                black_box(k.unify_same_domain(&r.shape, &unify).unwrap())
            });
        });
    }
    g.bench_function("shell-star-100", |b| {
        b.iter(|| black_box(fast.shell(&star, &[star_top], &opts, &ctx).unwrap()));
    });
    g.bench_function("shell-plate-100", |b| {
        b.iter(|| black_box(fast.shell(&plate, &[plate_top], &opts, &ctx).unwrap()));
    });
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
