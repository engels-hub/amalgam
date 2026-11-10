//! IO-02 benches (Done-when 5 and 6): `io/stl_read_bin_1gb`, `io/stl_read_ascii_1gb`,
//! `io/stl_write_20m`, plus the cancel latency of the 1 GB import.
//!
//! The 1 GB inputs are generated into `CARGO_TARGET_DIR` on first use. Bounds (< 5 s, < 6 s,
//! < 2 s, 100 ms) are asserted only under `AMG_STRICT_TIMING=1`; the peak-heap bound is in
//! `stl_heap` (it needs the `dhat` allocator, which would distort these timings).
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "bench code")]

mod common;

use std::time::{Duration, Instant};

use amg_io::{Format, ImportOptions, IoError, Sink, Source, export, import};
use amg_jobs::{CancelToken, JobCtx};
use criterion::{Criterion, criterion_group, criterion_main};

fn read_once(path: &std::path::Path) -> (Duration, u64) {
    let t = Instant::now();
    let i = import(
        Source::Path(path),
        Some(Format::Stl),
        &ImportOptions::default(),
        &JobCtx::detached(),
    )
    .expect("import");
    (t.elapsed(), i.report.triangles_read)
}

fn bench_read(c: &mut Criterion, id: &str, path: &std::path::Path, bound: Duration) {
    let (t, tris) = read_once(path);
    eprintln!("{id}: {tris} triangles in {t:.2?} (bound {bound:?})");
    if common::strict() {
        assert!(t < bound, "{id}: {t:?} exceeds {bound:?}");
    }
    let mut g = c.benchmark_group("io");
    g.sample_size(10)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(20));
    g.bench_function(id, |b| b.iter(|| read_once(path)));
    g.finish();
}

fn stl_read_bin_1gb(c: &mut Criterion) {
    let p = common::bin_1gb();
    bench_read(c, "stl_read_bin_1gb", &p, Duration::from_secs(5));
}

fn stl_read_ascii_1gb(c: &mut Criterion) {
    let p = common::ascii_1gb();
    eprintln!(
        "ascii input: {} bytes",
        std::fs::metadata(&p).expect("meta").len()
    );
    bench_read(c, "stl_read_ascii_1gb", &p, Duration::from_secs(6));
}

fn stl_write_20m(c: &mut Criterion) {
    let scene = common::scene(common::mesh(common::BIN_FREQ));
    let opts = common::raw_options(false);
    let cap = 84 + 50 * 20_000_000usize;
    let run = || {
        let mut out: Vec<u8> = Vec::with_capacity(cap);
        let t = Instant::now();
        let r = export(
            &scene,
            Sink::Memory(&mut out),
            Format::Stl,
            &opts,
            &JobCtx::detached(),
        )
        .expect("export");
        (t.elapsed(), r.bytes)
    };
    let (t, bytes) = run();
    eprintln!("stl_write_20m: {bytes} bytes in {t:.2?} (bound 2 s)");
    if common::strict() {
        assert!(t < Duration::from_secs(2), "stl_write_20m: {t:?}");
    }
    let mut g = c.benchmark_group("io");
    g.sample_size(10)
        .warm_up_time(Duration::from_secs(1))
        .measurement_time(Duration::from_secs(15));
    g.bench_function("stl_write_20m", |b| b.iter(run));
    g.finish();
}

/// Done-when 6: cancelling the 1 GB import returns `Cancelled` within 100 ms.
fn stl_cancel_latency_1gb(_c: &mut Criterion) {
    let path = common::bin_1gb();
    let mut worst = Duration::ZERO;
    for delay_ms in [50u64, 400, 1200] {
        let token = CancelToken::new();
        let ctx = JobCtx::with_token(token.clone());
        let worker = {
            let path = path.clone();
            std::thread::spawn(move || {
                import(
                    Source::Path(&path),
                    Some(Format::Stl),
                    &ImportOptions::default(),
                    &ctx,
                )
                .map(|_| ())
            })
        };
        std::thread::sleep(Duration::from_millis(delay_ms));
        let t = Instant::now();
        token.cancel();
        let r = worker.join().expect("join");
        let latency = t.elapsed();
        eprintln!("cancel after {delay_ms} ms: {r:?}, returned {latency:.2?} later");
        assert!(
            matches!(r, Err(IoError::Cancelled)),
            "the import finished before the cancel (delay {delay_ms} ms): {r:?}"
        );
        worst = worst.max(latency);
    }
    eprintln!("stl_cancel_latency_1gb: worst {worst:.2?} (bound 100 ms)");
    if common::strict() {
        assert!(worst < Duration::from_millis(100), "{worst:?}");
    }
}

criterion_group!(
    benches,
    stl_read_bin_1gb,
    stl_read_ascii_1gb,
    stl_write_20m,
    stl_cancel_latency_1gb
);
criterion_main!(benches);
