//! CMP-02 Done-when 6: at least 0.5 G `bg_trilinear` samples/s at random in-band points on the
//! reference GPU.
//!
//! Grid: the Done-when 1 sphere (`testprob::sphere_bricks::<f16>(100, 4)`, `CORNERS`, f16
//! packed). Points: 2^24 uniformly random points of the band shell (|d| ≤ 4 cells), in random
//! order, one thread each; the kernel writes one f32 per point. Rate = points / GPU time of one
//! dispatch (timestamp queries; wall clock of a batch when the adapter has none). A larger
//! sphere (r = 400 cells, about 120 MB of bricks, bigger than the L2 cache) is recorded without
//! a bar. The bar is asserted only under `AMG_STRICT_TIMING=1` on a hardware adapter. Not run in
//! CI.

#![allow(clippy::unwrap_used, reason = "benchmark code")]

use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_compute::f16;
use amg_compute::gpu::{ComputeCtx, GpuVec, Kernel};
use amg_compute::grid::{GpuBrickGrid, brick_grid_wgsl};
use amg_compute::testprob::sphere_bricks;
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig, wgpu};
use criterion::{Criterion, Throughput, criterion_group, criterion_main};

const POINTS: usize = 1 << 24;
const BAND: f64 = 4.0;
const BAR: f64 = 0.5e9;
/// Dispatches per timed batch.
const REPS: usize = 20;

struct Rng(u64);
impl Rng {
    fn unit(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Uniform random points of the shell `radius ± BAND`, as `vec4<f32>` (w unused).
fn shell_points(radius: f64, n: usize, seed: u64) -> Vec<[f32; 4]> {
    let mut rng = Rng(seed);
    (0..n)
        .map(|_| {
            let dir = loop {
                let v = [0, 1, 2].map(|_| rng.unit() * 2.0 - 1.0);
                let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
                if l > 1e-3 && l <= 1.0 {
                    break v.map(|c| c / l);
                }
            };
            // Uniform in volume: r^3 uniform between the shell radii.
            let (r0, r1) = ((radius - BAND).powi(3), (radius + BAND).powi(3));
            let r = (r0 + rng.unit() * (r1 - r0)).cbrt();
            [dir[0] * r, dir[1] * r, dir[2] * r, 0.0].map(|c| c as f32)
        })
        .collect()
}

/// Seconds per dispatch of `bg_trilinear` over `POINTS` random in-band points.
fn measure(gpu: &Arc<Gpu>, radius: f64, c: &mut Criterion) -> (f64, usize, usize) {
    let g = sphere_bricks::<f16>(radius, BAND);
    let grid = GpuBrickGrid::upload(gpu, &g).unwrap();
    let pts = shell_points(radius, POINTS, 5);
    let src = format!(
        "{}
@group(0) @binding(0) var<storage, read> pts: array<vec4<f32>>;
@group(0) @binding(1) var<storage, read_write> out: array<f32>;
@compute @workgroup_size(WG)
fn main(@builtin(global_invocation_id) gid: vec3<u32>, @builtin(num_workgroups) nwg: vec3<u32>) {{
    let i = amg_linear_id(gid, nwg);
    if (i >= arrayLength(&out)) {{ return; }}
    out[i] = bg_trilinear(pts[i].xyz);
}}
",
        brick_grid_wgsl(1, 0, grid.encoding(), false)
    );
    let k = Kernel::new(gpu, "bg_trilinear bench", &src, "main", &[]).unwrap();
    let p = GpuVec::from_slice(gpu, "points", &pts).unwrap();
    let out = GpuVec::<f32>::zeroed(gpu, "values", POINTS as u64).unwrap();
    let g0 = k.bind_group(gpu, 0, &[p.binding(), out.binding()]);
    let g1 = k.bind_group(gpu, 1, &grid.bindings());
    let job = JobCtx::detached();
    let timestamps = gpu.has(wgpu::Features::TIMESTAMP_QUERY);
    let mut best = f64::INFINITY;
    let mut group = c.benchmark_group("brick_grid");
    group.throughput(Throughput::Elements(POINTS as u64));
    group.sample_size(10);
    group.bench_function(format!("bg_trilinear_f16_r{radius}"), |b| {
        b.iter_custom(|iters| {
            let mut wall = Duration::ZERO;
            for _ in 0..iters {
                let mut cx = ComputeCtx::new(gpu.clone(), "bg_trilinear bench");
                let start = Instant::now();
                for _ in 0..REPS {
                    cx.dispatch_1d(&k, &[&g0, &g1], POINTS as u64);
                }
                cx.finish(&job).unwrap();
                let elapsed = start.elapsed();
                wall += elapsed / REPS as u32;
                let per = if timestamps {
                    cx.take_timings()
                        .iter()
                        .map(|(_, d)| d.as_secs_f64())
                        .fold(f64::INFINITY, f64::min)
                } else {
                    elapsed.as_secs_f64() / REPS as f64
                };
                best = best.min(per);
            }
            wall
        });
    });
    group.finish();
    (best, g.brick_count(), g.brick_bytes())
}

fn brick_grid(c: &mut Criterion) {
    if !amg_testkit::gpu_available() {
        return;
    }
    let gpu = Gpu::new_blocking(&GpuConfig::from_env()).unwrap();
    let hardware = gpu.info().device_type != wgpu::DeviceType::Cpu;
    eprintln!(
        "adapter: {} ({:?}), TIMESTAMP_QUERY: {}",
        gpu.info().name,
        gpu.info().backend,
        gpu.has(wgpu::Features::TIMESTAMP_QUERY)
    );
    let mut rates = Vec::new();
    for radius in [100.0, 400.0] {
        let (secs, bricks, bytes) = measure(&gpu, radius, c);
        let rate = POINTS as f64 / secs;
        eprintln!(
            "bg_trilinear f16, sphere r = {radius} cells ({bricks} bricks, {:.1} MB): \
             {POINTS} in-band points in {:.3} ms = {:.2} G samples/s (bar {:.1} G/s)",
            bytes as f64 / 1e6,
            secs * 1e3,
            rate / 1e9,
            BAR / 1e9
        );
        rates.push(rate);
    }
    if std::env::var_os("AMG_STRICT_TIMING").is_some() && hardware {
        assert!(
            rates[0] >= BAR,
            "bg_trilinear {:.2} G samples/s below 0.5 G/s",
            rates[0] / 1e9
        );
    }
}

criterion_group!(benches, brick_grid);
criterion_main!(benches);
