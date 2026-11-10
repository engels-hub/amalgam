//! CMP-01 Done-when 4: throughput of the GPU primitives on the reference GPU.
//!
//! - reduce f32 over 2^26 elements: at least 60 % of the device-to-device copy bandwidth;
//! - scan u32 over 2^26 elements: at least 30 %;
//! - sort of 2^24 key-value pairs (32-bit keys): at least 1 G pairs/s and at least 3x rayon
//!   `par_sort_by_key`;
//! - f64 reduce and dot (device with `SHADER_F64`, i.e. `AMG_WGPU_BACKEND=vulkan`): recorded.
//!
//! Bandwidth counts the bytes an ideal implementation moves: copy and scan read and write the
//! array (2 x 256 MiB), reduce reads it once. The bars are asserted only under
//! `AMG_STRICT_TIMING=1` on a hardware adapter. Not run in CI.

#![allow(clippy::unwrap_used, reason = "benchmark code")]

use std::cell::RefCell;
use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_compute::gpu::{ComputeCtx, GpuVec};
use amg_compute::prim::{Primitives, ReduceOp, ScanKind};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig, wgpu};
use criterion::{Criterion, criterion_group, criterion_main};
use rayon::slice::ParallelSliceMut;

const N_BIG: usize = 1 << 26;
const N_SORT: usize = 1 << 24;

struct Rng(u64);
impl Rng {
    fn u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        ((z ^ (z >> 31)) >> 32) as u32
    }
}

/// Mean seconds per repetition, recorded by name.
#[derive(Default)]
struct Results(RefCell<Vec<(String, f64)>>);

impl Results {
    fn get(&self, name: &str) -> f64 {
        self.0
            .borrow()
            .iter()
            .find(|(n, _)| n == name)
            .map_or(f64::NAN, |(_, s)| *s)
    }
}

/// Records `iters` repetitions of `record` in contexts of at most `batch`, waits for the GPU
/// and returns the elapsed time. Mean time per repetition is stored in `results[name]`.
fn timed(
    gpu: &Arc<Gpu>,
    iters: u64,
    batch: u64,
    totals: &mut (Duration, u64),
    mut record: impl FnMut(&mut ComputeCtx),
) -> Duration {
    let job = JobCtx::detached();
    let start = Instant::now();
    let mut left = iters;
    while left > 0 {
        let reps = left.min(batch);
        let mut cx = ComputeCtx::new(gpu.clone(), "bench");
        for _ in 0..reps {
            record(&mut cx);
        }
        cx.finish(&job).unwrap();
        left -= reps;
    }
    let elapsed = start.elapsed();
    totals.0 += elapsed;
    totals.1 += iters;
    elapsed
}

fn primitives(c: &mut Criterion) {
    if !amg_testkit::gpu_available() {
        return;
    }
    let gpu = Gpu::new_blocking(&GpuConfig::from_env()).unwrap();
    let info = gpu.info();
    eprintln!(
        "adapter: {} ({:?}, {:?})",
        info.name, info.backend, info.device_type
    );
    let hardware = info.device_type != wgpu::DeviceType::Cpu;
    let has_f64 = gpu.has(wgpu::Features::SHADER_F64);
    let p = Primitives::new(&gpu).unwrap();
    let results = Results::default();
    let mut rng = Rng(1);
    let mut group = c.benchmark_group("primitives");
    group.sample_size(10);
    group.warm_up_time(Duration::from_millis(500));
    group.measurement_time(Duration::from_secs(3));

    let mut run = |name: &str, batch: u64, record: &mut dyn FnMut(&mut ComputeCtx)| {
        let mut totals = (Duration::ZERO, 0u64);
        group.bench_function(name, |b| {
            b.iter_custom(|iters| timed(&gpu, iters, batch, &mut totals, &mut *record));
        });
        let secs = totals.0.as_secs_f64() / totals.1.max(1) as f64;
        results.0.borrow_mut().push((name.to_owned(), secs));
        eprintln!("{name}: {:.3} ms per call", secs * 1e3);
    };

    // 2^26 f32 / u32 elements.
    let data: Vec<f32> = (0..N_BIG)
        .map(|_| (rng.u32() >> 8) as f32 / 8_388_608.0 - 1.0)
        .collect();
    let src = GpuVec::from_slice(&gpu, "src", &data).unwrap();
    let dst = GpuVec::<f32>::zeroed(&gpu, "dst", N_BIG as u64).unwrap();
    let out = GpuVec::<f32>::zeroed(&gpu, "out", 1).unwrap();
    run("copy_f32_2^26", 8, &mut |cx| cx.copy(&src, &dst));
    run("reduce_f32_sum_2^26", 8, &mut |cx| {
        p.reduce(cx, ReduceOp::Sum, &src, &out, 0).unwrap();
    });
    let ints: Vec<u32> = (0..N_BIG).map(|_| rng.u32() & 3).collect();
    let in_u32 = GpuVec::from_slice(&gpu, "in", &ints).unwrap();
    let out_u32 = GpuVec::<u32>::zeroed(&gpu, "out", N_BIG as u64).unwrap();
    run("scan_u32_exclusive_2^26", 4, &mut |cx| {
        p.scan(cx, ScanKind::Exclusive, &in_u32, &out_u32, None)
            .unwrap();
    });
    drop((in_u32, out_u32, dst));

    // Sort 2^24 key-value pairs: a fresh copy of the unsorted pairs before every sort.
    let keys: Vec<u32> = (0..N_SORT).map(|_| rng.u32()).collect();
    let vals: Vec<u32> = (0..N_SORT as u32).collect();
    let (k0, v0) = (
        GpuVec::from_slice(&gpu, "k0", &keys).unwrap(),
        GpuVec::from_slice(&gpu, "v0", &vals).unwrap(),
    );
    let mut k = GpuVec::<u32>::zeroed(&gpu, "k", N_SORT as u64).unwrap();
    let mut v = GpuVec::<u32>::zeroed(&gpu, "v", N_SORT as u64).unwrap();
    run("copy_pairs_2^24", 8, &mut |cx| {
        cx.copy(&k0, &k);
        cx.copy(&v0, &v);
    });
    run("sort_pairs_2^24 (incl. copy)", 2, &mut |cx| {
        cx.copy(&k0, &k);
        cx.copy(&v0, &v);
        p.sort(cx, &mut k, Some(&mut v), 32).unwrap();
    });

    if has_f64 {
        let d64: Vec<f64> = data[..N_BIG / 2].iter().map(|&x| f64::from(x)).collect();
        let a = GpuVec::from_slice(&gpu, "a64", &d64).unwrap();
        let b = GpuVec::from_slice(&gpu, "b64", &d64).unwrap();
        let o = GpuVec::<f64>::zeroed(&gpu, "o64", 1).unwrap();
        run("reduce_f64_sum_2^25", 8, &mut |cx| {
            p.reduce(cx, ReduceOp::Sum, &a, &o, 0).unwrap();
        });
        run("dot_f64_2^25", 8, &mut |cx| {
            p.dot(cx, &a, &b, &o, 0).unwrap();
        });
        let f32a = GpuVec::from_slice(&gpu, "a32", &data[..N_BIG / 2]).unwrap();
        let f32b = GpuVec::from_slice(&gpu, "b32", &data[..N_BIG / 2]).unwrap();
        run("dot_f32_2^25", 8, &mut |cx| {
            p.dot(cx, &f32a, &f32b, &out, 0).unwrap();
        });
    }
    group.finish();

    // CPU baseline: rayon par_sort_by_key on (key, value) pairs.
    let pairs: Vec<(u32, u32)> = keys.iter().copied().zip(vals.iter().copied()).collect();
    let mut cpu_best = f64::INFINITY;
    for _ in 0..5 {
        let mut work = pairs.clone();
        let start = Instant::now();
        work.par_sort_by_key(|p| p.0);
        cpu_best = cpu_best.min(start.elapsed().as_secs_f64());
        std::hint::black_box(&work);
    }

    let bytes = (N_BIG * 4) as f64;
    let copy_bw = 2.0 * bytes / results.get("copy_f32_2^26");
    let reduce_bw = bytes / results.get("reduce_f32_sum_2^26");
    let scan_bw = 2.0 * bytes / results.get("scan_u32_exclusive_2^26");
    let copy_pairs = results.get("copy_pairs_2^24");
    let sort_total = results.get("sort_pairs_2^24 (incl. copy)");
    let sort_s = sort_total - copy_pairs;
    let sort_rate = N_SORT as f64 / sort_s;
    let gb = |x: f64| x / 1e9;
    eprintln!("copy bandwidth (read+write): {:.1} GB/s", gb(copy_bw));
    eprintln!(
        "reduce f32: {:.1} GB/s read = {:.0} % of copy ({:.0} % counting copy as one-way bytes)",
        gb(reduce_bw),
        100.0 * reduce_bw / copy_bw,
        100.0 * reduce_bw / (copy_bw / 2.0)
    );
    eprintln!(
        "scan u32: {:.1} GB/s (read+write) = {:.0} % of copy",
        gb(scan_bw),
        100.0 * scan_bw / copy_bw
    );
    eprintln!(
        "sort 2^24 pairs: {:.2} ms ({:.2} G pairs/s, copy of the unsorted input subtracted: {:.2} ms); rayon par_sort_by_key: {:.2} ms ({:.2} G pairs/s); speed-up {:.1}x",
        sort_s * 1e3,
        sort_rate / 1e9,
        copy_pairs * 1e3,
        cpu_best * 1e3,
        N_SORT as f64 / cpu_best / 1e9,
        cpu_best / sort_s
    );
    if has_f64 {
        eprintln!(
            "f64 reduce 2^25: {:.3} ms ({:.1} GB/s); f64 dot 2^25: {:.3} ms; f32 dot 2^25: {:.3} ms",
            results.get("reduce_f64_sum_2^25") * 1e3,
            gb((N_BIG / 2 * 8) as f64 / results.get("reduce_f64_sum_2^25")),
            results.get("dot_f64_2^25") * 1e3,
            results.get("dot_f32_2^25") * 1e3
        );
    }

    if std::env::var_os("AMG_STRICT_TIMING").is_some() && hardware {
        assert!(
            reduce_bw >= 0.6 * copy_bw,
            "reduce f32 below 60 % of copy bandwidth"
        );
        assert!(
            scan_bw >= 0.3 * copy_bw,
            "scan u32 below 30 % of copy bandwidth"
        );
        assert!(sort_rate >= 1e9, "sort below 1 G pairs/s");
        assert!(
            cpu_best / sort_s >= 3.0,
            "sort below 3x rayon par_sort_by_key"
        );
    }
}

criterion_group!(benches, primitives);
criterion_main!(benches);
