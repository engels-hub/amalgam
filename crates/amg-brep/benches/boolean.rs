//! BREP-04 budgets (S02 "Performance targets"): a typical boolean of two 200-face parts
//! (< 50 ms, `budget/brep-bool-200`), and Done-when 5: a cut of two 1000-face parts with OCCT's
//! parallel mode on 1, half and all cores' threads, plus the serial baseline
//! (`occt-threads/cut-1030/*`). The OCCT pool is process-global, so the thread groups resize it
//! between runs (outside the timed loop). Validation is off, as for a release build's default
//! policy.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

#[path = "../tests/common/parts.rs"]
mod parts;

use std::hint::black_box;
use std::time::Duration;

use amg_brep::{BoolOp, BoolOpts, BrepKernel, JobCtx, KernelConfig, OcctKernel, Shape, Validate};
use criterion::{Criterion, criterion_group, criterion_main};

fn kernel() -> OcctKernel {
    OcctKernel::new(KernelConfig {
        validate: Validate::Off,
        ..KernelConfig::default()
    })
}

fn cut(k: &OcctKernel, object: &Shape, tool: &Shape, parallel: bool) -> Shape {
    k.boolean(
        BoolOp::Cut,
        std::slice::from_ref(object),
        std::slice::from_ref(tool),
        &BoolOpts {
            parallel: Some(parallel),
            ..BoolOpts::default()
        },
        &JobCtx::detached(),
    )
    .unwrap()
    .shape
}

fn set_pool(threads: usize) -> i32 {
    amg_occt_sys::ffi::set_default_thread_pool(i32::try_from(threads).unwrap()).unwrap()
}

fn bench(c: &mut Criterion) {
    let k = kernel();
    let cores = std::thread::available_parallelism().map_or(1, std::num::NonZeroUsize::get);
    let default_threads = k.config().effective_occt_threads().get();

    let (small_object, small_tool) = parts::plate_pair(&k, 14);
    assert_eq!(small_object.count(amg_brep::SubKind::Face), 202);
    // The kernel default (parallel on, the OCCT pool at half the cores).
    let parallel = k.config().bool_parallel && default_threads > 1;
    c.bench_function("budget/brep-bool-200", |b| {
        b.iter(|| black_box(cut(&k, &small_object, &small_tool, parallel)));
    });

    let (object, tool) = parts::plate_pair(&k, 32);
    assert_eq!(object.count(amg_brep::SubKind::Face), 1030);
    let mut group = c.benchmark_group("occt-threads/cut-1030");
    group
        .sample_size(10)
        .warm_up_time(Duration::from_millis(1))
        .measurement_time(Duration::from_secs(60));
    group.bench_function("serial", |b| {
        b.iter(|| black_box(cut(&k, &object, &tool, false)));
    });
    let mut counts = vec![1, (cores / 2).max(1), cores];
    counts.dedup();
    for threads in counts {
        let got = set_pool(threads);
        eprintln!("OCCT pool: {got} threads ({cores} cores)");
        group.bench_function(format!("parallel-{threads}"), |b| {
            b.iter(|| black_box(cut(&k, &object, &tool, true)));
        });
    }
    group.finish();
    set_pool(default_threads);
}

criterion_group!(benches, bench);
criterion_main!(benches);
