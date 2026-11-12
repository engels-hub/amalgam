//! RND-02 viewport bench (spec S10, Done-when 1 and 2; vision row "viewport, 5 M-triangle mesh,
//! orbit >= 60 fps", shading-only half). Headless: no window, no egui.
//!
//! A displaced icosphere orbits for 600 frames at 2560x1440, 4x MSAA, Studio lighting, with at
//! most two frames in flight (like a swapchain). Reported per configuration:
//! - **wall**: time between the starts of consecutive frames (encode + submit + waiting for the
//!   frame before last), so it includes CPU work and GPU back-pressure;
//! - **GPU**: timestamp span of the frame's passes (`Renderer::drain_gpu_timings`; needs
//!   `TIMESTAMP_QUERY`).
//!
//! Budgets: 5 M tris GPU p99 <= 9 ms and wall p99 <= 16.6 ms; 1 M tris GPU p50 <= 3 ms. Each is
//! measured with and without the depth pre-pass (Done-when 2). The budgets are asserted only
//! with `AMG_STRICT_TIMING=1`; the numbers are printed always.
//!
//! `cargo bench -p amg-render --bench viewport -- --test` (CI on WARP) runs a tiny version.

#![allow(clippy::unwrap_used, clippy::expect_used, reason = "bench code")]

use std::collections::HashMap;
use std::hint::black_box;
use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_core::{Dir3, Point3, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::{
    Camera, Instance, Lighting, MeshData, Projection, Renderer, RendererConfig, ViewId, ViewStyle,
    upload_mesh, wgpu,
};
use criterion::{Criterion, criterion_group, criterion_main};

const FRAMES: usize = 600;
const SIZE: [u32; 2] = [2560, 1440];

fn test_mode() -> bool {
    std::env::args().any(|a| a == "--test")
}

fn strict() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

fn gpu() -> Option<Arc<Gpu>> {
    let mut cfg = GpuConfig::from_env();
    cfg.optional_features |= amg_render::OPTIONAL_FEATURES;
    match Gpu::new_blocking(&cfg) {
        Ok(gpu) => {
            eprintln!("adapter: {} ({:?})", gpu.info().name, gpu.info().backend);
            Some(gpu)
        }
        Err(e) => {
            eprintln!("no GPU, skipping the viewport benches: {e}");
            None
        }
    }
}

/// An icosahedron subdivided at frequency `f = ceil(sqrt(min_tris / 20))` (`20 f^2`
/// triangles), displaced radially by a few sine waves, radius about 100 mm, with area-weighted
/// vertex normals. The same generator as the `spin` example.
fn displaced_icosphere(min_tris: usize) -> MeshData {
    const T: f64 = 1.618_033_988_749_895;
    const CORNERS: [[f64; 3]; 12] = [
        [-1.0, T, 0.0],
        [1.0, T, 0.0],
        [-1.0, -T, 0.0],
        [1.0, -T, 0.0],
        [0.0, -1.0, T],
        [0.0, 1.0, T],
        [0.0, -1.0, -T],
        [0.0, 1.0, -T],
        [T, 0.0, -1.0],
        [T, 0.0, 1.0],
        [-T, 0.0, -1.0],
        [-T, 0.0, 1.0],
    ];
    const FACES: [[u32; 3]; 20] = [
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    let f = ((min_tris as f64 / 20.0).sqrt().ceil() as u32).max(1);
    let fu = f as usize;
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(10 * fu * fu + 2);
    let mut indices: Vec<u32> = Vec::with_capacity(20 * fu * fu * 3);
    let mut shared: HashMap<(u32, u32, u32), u32> = HashMap::new();
    let corner = |i: u32| Vec3::from_array(CORNERS[i as usize]);
    let edge_key = |a: u32, wa: u32, b: u32, wb: u32| if a < b { (a, b, wa) } else { (b, a, wb) };
    let displacement = |d: Vec3| {
        100.0
            * (1.0
                + 0.05 * (11.0 * d.x).sin() * (9.0 * d.y).sin()
                + 0.03 * (23.0 * d.z + 7.0 * d.x).sin()
                + 0.015 * (47.0 * d.y + 31.0 * d.z).sin())
    };
    let mut row: Vec<Vec<u32>> = Vec::with_capacity(fu + 1);
    for [a, b, c] in FACES {
        row.clear();
        for i in 0..=f {
            let mut ids = Vec::with_capacity(i as usize + 1);
            for j in 0..=i {
                let (wa, wb, wc) = (f - i, i - j, j);
                let key = match (wa > 0, wb > 0, wc > 0) {
                    (true, false, false) => Some((a, a, f)),
                    (false, true, false) => Some((b, b, f)),
                    (false, false, true) => Some((c, c, f)),
                    (true, true, false) => Some(edge_key(a, wa, b, wb)),
                    (false, true, true) => Some(edge_key(b, wb, c, wc)),
                    (true, false, true) => Some(edge_key(a, wa, c, wc)),
                    _ => None,
                };
                let new_point = |positions: &mut Vec<[f32; 3]>| {
                    let dir = (corner(a) * f64::from(wa)
                        + corner(b) * f64::from(wb)
                        + corner(c) * f64::from(wc))
                    .normalize();
                    positions.push((dir * displacement(dir)).as_vec3().to_array());
                    (positions.len() - 1) as u32
                };
                ids.push(match key {
                    Some(key) => *shared
                        .entry(key)
                        .or_insert_with(|| new_point(&mut positions)),
                    None => new_point(&mut positions),
                });
            }
            row.push(ids);
        }
        for i in 0..fu {
            for j in 0..=i {
                indices.extend([row[i][j], row[i + 1][j], row[i + 1][j + 1]]);
                if j < i {
                    indices.extend([row[i][j], row[i + 1][j + 1], row[i][j + 1]]);
                }
            }
        }
    }
    let mut sum = vec![Vec3::ZERO; positions.len()];
    let p = |i: u32| Vec3::from_array(positions[i as usize].map(f64::from));
    for t in indices.chunks_exact(3) {
        let n = (p(t[1]) - p(t[0])).cross(p(t[2]) - p(t[0]));
        for &v in t {
            sum[v as usize] += n;
        }
    }
    let normals = sum
        .iter()
        .map(|n| n.normalize_or_zero().as_vec3().to_array())
        .collect();
    MeshData {
        origin: Point3::ZERO,
        positions,
        normals,
        indices,
        closed: true,
    }
}

struct Fixture {
    renderer: Renderer,
    view: ViewId,
    triangles: u64,
}

fn fixture(gpu: &Arc<Gpu>, mesh: &MeshData, size: [u32; 2], prepass: bool) -> Fixture {
    let mut renderer = Renderer::new(
        Arc::clone(gpu),
        &RendererConfig {
            msaa_samples: 4,
            depth_prepass: prepass,
            jobs: None,
        },
    )
    .unwrap();
    let gpu_mesh = upload_mesh(gpu, mesh, &JobCtx::detached()).unwrap();
    let triangles = u64::from(gpu_mesh.triangle_count());
    let key = renderer.scene_mut().insert_mesh(gpu_mesh);
    renderer
        .scene_mut()
        .add_instance(Instance::new(key, Transform::IDENTITY));
    let view = renderer.create_view(size);
    renderer.view_mut(view).unwrap().style = ViewStyle {
        lighting: Lighting::Studio,
        ..ViewStyle::default()
    };
    assert!(renderer.prepare_view(view));
    Fixture {
        renderer,
        view,
        triangles,
    }
}

/// Camera of frame `i` of an `n`-frame orbit (one turn, slightly above the equator).
fn orbit(i: usize, n: usize) -> Camera {
    let a = std::f64::consts::TAU * i as f64 / n as f64;
    Camera {
        eye: Point3::new(330.0 * a.cos(), 330.0 * a.sin(), 110.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.7 },
    }
}

struct Run {
    wall_ms: Vec<f64>,
    gpu_ms: Vec<f64>,
    passes: HashMap<&'static str, Vec<f64>>,
}

/// Renders `frames` orbit frames with at most two in flight.
fn run(f: &mut Fixture, gpu: &Gpu, frames: usize) -> Run {
    let device = gpu.device();
    let queue = gpu.queue();
    let mut out = Run {
        wall_ms: Vec::with_capacity(frames),
        gpu_ms: Vec::with_capacity(frames),
        passes: HashMap::new(),
    };
    let mut in_flight: Option<wgpu::SubmissionIndex> = None;
    let mut last = Instant::now();
    for i in 0..=frames {
        let now = Instant::now();
        if i > 0 {
            out.wall_ms.push((now - last).as_secs_f64() * 1e3);
        }
        last = now;
        if i == frames {
            break;
        }
        f.renderer.view_mut(f.view).unwrap().camera = orbit(i, frames);
        let mut encoder = device.create_command_encoder(&Default::default());
        black_box(f.renderer.encode(f.view, &mut encoder).unwrap());
        let index = queue.submit([encoder.finish()]);
        if let Some(prev) = in_flight.replace(index) {
            device
                .poll(wgpu::PollType::Wait {
                    submission_index: Some(prev),
                    timeout: Some(Duration::from_secs(10)),
                })
                .unwrap();
        }
        for t in f.renderer.drain_gpu_timings(f.view) {
            out.gpu_ms.push(f64::from(t.total_ms));
            for (name, ms) in t.passes {
                out.passes.entry(name).or_default().push(f64::from(ms));
            }
        }
    }
    device.poll(wgpu::PollType::wait_indefinitely()).unwrap();
    for t in f.renderer.drain_gpu_timings(f.view) {
        out.gpu_ms.push(f64::from(t.total_ms));
        for (name, ms) in t.passes {
            out.passes.entry(name).or_default().push(f64::from(ms));
        }
    }
    out
}

fn percentile(values: &[f64], p: f64) -> f64 {
    if values.is_empty() {
        return f64::NAN;
    }
    let mut v = values.to_vec();
    v.sort_by(f64::total_cmp);
    v[((v.len() - 1) as f64 * p).round() as usize]
}

/// Done-when 1 and 2: the orbit budgets with and without the depth pre-pass.
fn orbit_budgets(c: &mut Criterion) {
    let Some(gpu) = gpu() else {
        return;
    };
    let (size, frames, sizes) = if test_mode() {
        ([64, 64], 5, vec![("20k", 20_000usize)])
    } else {
        (SIZE, FRAMES, vec![("1M", 1_000_000), ("5M", 5_000_000)])
    };
    for (name, tris) in sizes {
        let start = Instant::now();
        let mesh = displaced_icosphere(tris);
        eprintln!(
            "mesh {name}: {} triangles, generated in {:.1} s",
            mesh.indices.len() / 3,
            start.elapsed().as_secs_f64()
        );
        let mut p99s = Vec::new();
        for prepass in [false, true] {
            let mut f = fixture(&gpu, &mesh, size, prepass);
            run(&mut f, &gpu, frames.min(60)); // warm-up
            let r = run(&mut f, &gpu, frames);
            let (w50, w99) = (percentile(&r.wall_ms, 0.5), percentile(&r.wall_ms, 0.99));
            let (g50, g99) = (percentile(&r.gpu_ms, 0.5), percentile(&r.gpu_ms, 0.99));
            let mut passes: Vec<_> = r
                .passes
                .iter()
                .map(|(k, v)| format!("{k} p50 {:.3}", percentile(v, 0.5)))
                .collect();
            passes.sort();
            eprintln!(
                "viewport {name} ({} tris) {}x{} 4x MSAA Studio prepass={prepass}: {} frames, wall p50 {w50:.3} ms p99 {w99:.3} ms; GPU p50 {g50:.3} ms p99 {g99:.3} ms ({} timed; {})",
                f.triangles,
                size[0],
                size[1],
                r.wall_ms.len(),
                r.gpu_ms.len(),
                passes.join(", ")
            );
            p99s.push((prepass, g50, g99, w99));
            if strict() && !prepass && !test_mode() {
                if name == "5M" {
                    assert!(g99 <= 9.0, "5M GPU p99 {g99:.3} ms > 9 ms");
                    assert!(w99 <= 16.6, "5M wall p99 {w99:.3} ms > 16.6 ms");
                } else {
                    assert!(g50 <= 3.0, "1M GPU p50 {g50:.3} ms > 3 ms");
                }
            }
        }
        if let [(_, off50, off99, _), (_, on50, on99, _)] = p99s[..] {
            let faster = if on50 < off50 {
                "pre-pass"
            } else {
                "no pre-pass"
            };
            eprintln!(
                "depth pre-pass {name}: GPU p50 {off50:.3} -> {on50:.3} ms, p99 {off99:.3} -> {on99:.3} ms; faster: {faster}"
            );
        }
    }

    // Criterion: wall time of one synchronous frame (encode, submit, wait).
    let tris = if test_mode() { 20_000 } else { 5_000_000 };
    let mut f = fixture(&gpu, &displaced_icosphere(tris), size, false);
    let mut g = c.benchmark_group("viewport");
    g.sample_size(20);
    g.bench_function("frame_5m_studio_1440p_msaa4", |b| {
        let mut i = 0;
        b.iter_custom(|iters| {
            let start = Instant::now();
            for _ in 0..iters {
                f.renderer.view_mut(f.view).unwrap().camera = orbit(i, FRAMES);
                i += 1;
                let mut encoder = gpu.device().create_command_encoder(&Default::default());
                black_box(f.renderer.encode(f.view, &mut encoder).unwrap());
                gpu.queue().submit([encoder.finish()]);
                gpu.device()
                    .poll(wgpu::PollType::wait_indefinitely())
                    .unwrap();
            }
            start.elapsed()
        });
    });
    g.finish();
}

criterion_group!(benches, orbit_budgets);
criterion_main!(benches);
