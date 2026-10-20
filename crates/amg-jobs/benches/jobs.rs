//! FND-04 benchmarks (S00 Performance targets): non-blocking handle accessors (< 1 µs) and
//! spawn-to-start latency on an idle pool (< 50 µs median). Not run in CI.

#![allow(clippy::unwrap_used, reason = "benchmark code")]

use std::{
    hint::black_box,
    num::NonZeroUsize,
    time::{Duration, Instant},
};

use amg_jobs::{Cancelled, JobSystem, JobSystemConfig, SpawnOptions};
use criterion::{Criterion, criterion_group, criterion_main};

fn system() -> JobSystem {
    JobSystem::new(JobSystemConfig {
        interactive_threads: NonZeroUsize::new(2),
        background_threads: NonZeroUsize::new(1),
        notifier: None,
    })
    .unwrap()
}

fn accessors(c: &mut Criterion) {
    let js = system();
    let (release_tx, release_rx) = crossbeam_channel::bounded::<()>(1);
    let mut h = js.spawn_fn("blocked", SpawnOptions::default(), move |ctx| {
        ctx.progress_msg(0.5, "working");
        let _ = release_rx.recv();
        Ok::<_, Cancelled>(())
    });
    while !matches!(h.state(), amg_jobs::JobState::Running(_)) {
        std::thread::yield_now();
    }
    c.bench_function("JobHandle::try_take (unfinished)", |b| {
        b.iter(|| black_box(h.try_take().is_none()))
    });
    c.bench_function("JobHandle::state (running)", |b| {
        b.iter(|| black_box(h.state()))
    });
    c.bench_function("JobHandle::progress", |b| {
        b.iter(|| black_box(h.progress()))
    });
    release_tx.send(()).unwrap();
    let _ = h.wait_blocking();
}

fn spawn_latency(c: &mut Criterion) {
    let js = system();
    let mut g = c.benchmark_group("spawn");
    g.sample_size(20).measurement_time(Duration::from_secs(5));
    // Measures only spawn -> job start; workers are given time to go idle between iterations.
    g.bench_function("spawn_to_start (idle pool)", |b| {
        b.iter_custom(|iters| {
            let mut total = Duration::ZERO;
            for _ in 0..iters {
                std::thread::sleep(Duration::from_millis(1));
                let t0 = Instant::now();
                let h = js.spawn_fn("t", SpawnOptions::default(), |_| {
                    Ok::<_, Cancelled>(Instant::now())
                });
                total += h.wait_blocking().unwrap().saturating_duration_since(t0);
            }
            total
        })
    });
    g.bench_function("spawn + wait (warm)", |b| {
        b.iter(|| {
            js.spawn_fn("t", SpawnOptions::default(), |_| Ok::<_, Cancelled>(1u32))
                .wait_blocking()
                .unwrap()
        })
    });
    g.finish();
}

criterion_group!(benches, accessors, spawn_latency);
criterion_main!(benches);
