//! RND-03 budgets (spec S10):
//! - `encode` of 10 k instances over 100 meshes: CPU p99 <= 2 ms (asserted under
//!   `AMG_STRICT_TIMING=1`; the p50/p99 are printed always);
//! - `upload_mesh` of 5 M triangles <= 300 ms on a job thread (owner machine);
//! - `insert_mesh` <= 50 us.
//!
//! Needs a GPU (WARP via `AMG_WGPU_BACKEND=warp` works); without an adapter the benches skip.

#![allow(clippy::unwrap_used, clippy::expect_used, reason = "bench code")]

use std::hint::black_box;
use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_core::{Dir3, Point3, Quat, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::{
    Background, Camera, Instance, InstanceKey, MeshData, Projection, Renderer, RendererConfig,
    ViewId, ViewStyle, upload_mesh, wgpu,
};
use criterion::{Criterion, criterion_group, criterion_main};

const MESHES: usize = 100;
const INSTANCES: usize = 10_000;

fn gpu() -> Option<Arc<Gpu>> {
    match Gpu::new_blocking(&GpuConfig::from_env()) {
        Ok(gpu) => {
            eprintln!("adapter: {} ({:?})", gpu.info().name, gpu.info().backend);
            Some(gpu)
        }
        Err(e) => {
            eprintln!("no GPU, skipping the render benches: {e}");
            None
        }
    }
}

/// A small tetrahedron-like mesh; `k` varies the shape so the meshes differ.
fn small_mesh(k: usize) -> MeshData {
    let s = 1.0 + k as f32 * 0.01;
    MeshData {
        origin: Point3::ZERO,
        positions: vec![[0.0, 0.0, 0.0], [s, 0.0, 0.0], [0.0, s, 0.0], [0.0, 0.0, s]],
        normals: Vec::new(),
        indices: vec![0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3],
        closed: true,
    }
}

struct Fixture {
    renderer: Renderer,
    view: ViewId,
    instances: Vec<InstanceKey>,
}

/// 10 000 instances of 100 meshes on a 25 x 20 x 20 grid, most of them in view.
fn fixture(gpu: &Arc<Gpu>) -> Fixture {
    let mut renderer = Renderer::new(Arc::clone(gpu), &RendererConfig::default()).unwrap();
    let meshes: Vec<_> = (0..MESHES)
        .map(|k| {
            let mesh = upload_mesh(gpu, &small_mesh(k), &JobCtx::detached()).unwrap();
            renderer.scene_mut().insert_mesh(mesh)
        })
        .collect();
    let instances = (0..INSTANCES)
        .map(|i| {
            let (x, y, z) = (i % 25, (i / 25) % 20, i / 500);
            let t = Transform::from_scale_rotation_translation(
                Vec3::splat(0.8),
                Quat::from_rotation_z(i as f64 * 0.1),
                Vec3::new(x as f64 * 2.0, y as f64 * 2.0, z as f64 * 2.0),
            );
            renderer
                .scene_mut()
                .add_instance(Instance::new(meshes[i % MESHES], t))
        })
        .collect();
    let view = renderer.create_view([1280, 720]);
    let v = renderer.view_mut(view).unwrap();
    v.camera = Camera {
        eye: Point3::new(25.0, -90.0, 60.0),
        target: Point3::new(25.0, 20.0, 20.0),
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.9 },
    };
    v.style = ViewStyle {
        background: Background::Solid([0.02, 0.03, 0.05]),
        ..Default::default()
    };
    Fixture {
        renderer,
        view,
        instances,
    }
}

/// Times `iters` encodes. Only the `encode` call is timed; the finished buffer is submitted
/// outside the timer and the device is drained now and then so the queue stays short.
fn time_encodes(
    f: &mut Fixture,
    gpu: &Gpu,
    iters: u64,
    mut before: impl FnMut(&mut Fixture, u64),
) -> Vec<Duration> {
    let mut times = Vec::with_capacity(iters as usize);
    for i in 0..iters {
        before(f, i);
        let mut encoder = gpu.device().create_command_encoder(&Default::default());
        let start = Instant::now();
        let stats = f.renderer.encode(f.view, &mut encoder).unwrap();
        times.push(start.elapsed());
        black_box(stats);
        gpu.queue().submit([encoder.finish()]);
        if i % 16 == 15 {
            let _ = gpu.device().poll(wgpu_wait());
        }
    }
    times
}

fn wgpu_wait() -> wgpu::PollType {
    wgpu::PollType::wait_indefinitely()
}

fn percentile(sorted: &[Duration], p: f64) -> Duration {
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

fn encode_benches(c: &mut Criterion) {
    let Some(gpu) = gpu() else {
        return;
    };
    let mut f = fixture(&gpu);

    // The budget check: p50/p99 over 2000 frames after a warm-up.
    time_encodes(&mut f, &gpu, 50, |_, _| {});
    let mut times = time_encodes(&mut f, &gpu, 2000, |_, _| {});
    times.sort();
    let (p50, p99) = (percentile(&times, 0.5), percentile(&times, 0.99));
    let stats = {
        let mut encoder = gpu.device().create_command_encoder(&Default::default());
        f.renderer.encode(f.view, &mut encoder).unwrap()
    };
    eprintln!(
        "encode {INSTANCES} instances / {MESHES} meshes: p50 {p50:?}, p99 {p99:?} (budget p99 <= 2 ms); drawn {}, culled {}",
        stats.instances_drawn, stats.culled
    );
    if std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1") {
        assert!(p99 <= Duration::from_millis(2), "encode p99 {p99:?} > 2 ms");
    }

    let mut g = c.benchmark_group("scene");
    g.bench_function("encode_10k_instances_100_meshes", |b| {
        b.iter_custom(|iters| time_encodes(&mut f, &gpu, iters, |_, _| {}).iter().sum());
    });
    g.bench_function("encode_10k_one_dirty_instance", |b| {
        b.iter_custom(|iters| {
            time_encodes(&mut f, &gpu, iters, |f, i| {
                let key = f.instances[(i as usize * 7919) % INSTANCES];
                f.renderer.scene_mut().instance_mut(key).unwrap().visible = true;
            })
            .iter()
            .sum()
        });
    });
    g.finish();
}

fn mesh_benches(c: &mut Criterion) {
    let Some(gpu) = gpu() else {
        return;
    };
    let mut g = c.benchmark_group("mesh");
    g.bench_function("insert_mesh", |b| {
        b.iter_custom(|iters| {
            let mut renderer = Renderer::new(Arc::clone(&gpu), &RendererConfig::default()).unwrap();
            let meshes: Vec<_> = (0..iters)
                .map(|k| {
                    upload_mesh(&gpu, &small_mesh(k as usize % 7), &JobCtx::detached()).unwrap()
                })
                .collect();
            let start = Instant::now();
            for mesh in meshes {
                black_box(renderer.scene_mut().insert_mesh(mesh));
            }
            start.elapsed()
        });
    });

    // 5 M triangles: a displaced grid.
    let n = 1581_usize;
    let stride = n + 1;
    let positions: Vec<[f32; 3]> = (0..stride * stride)
        .map(|k| {
            let (x, y) = ((k % stride) as f32 * 0.01, (k / stride) as f32 * 0.01);
            [x, y, 0.3 * (x * 3.0).sin() * (y * 2.0).cos()]
        })
        .collect();
    let mut indices = Vec::with_capacity(n * n * 6);
    for j in 0..n {
        for i in 0..n {
            let a = (j * stride + i) as u32;
            let (b, c, d) = (a + 1, a + stride as u32, a + stride as u32 + 1);
            indices.extend_from_slice(&[a, b, d, a, d, c]);
        }
    }
    let data = MeshData {
        positions,
        indices,
        ..Default::default()
    };
    g.sample_size(10);
    g.bench_function("upload_mesh_5m_tris", |b| {
        b.iter(|| black_box(upload_mesh(&gpu, &data, &JobCtx::detached()).unwrap()));
    });
    g.finish();
}

criterion_group!(benches, encode_benches, mesh_benches);
criterion_main!(benches);
