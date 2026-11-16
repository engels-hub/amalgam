//! RND-07 overlay bench (spec S10 RND-07 Done-when 6; performance target "overlays <= 0.5 ms
//! incremental"). Headless: no window, no egui.
//!
//! A 1 M-triangle sphere orbits for 600 frames at 2560x1440, 4x MSAA, `ShadedVisibleEdges`,
//! with at most two frames in flight. Every frame the overlay list is refilled with 1000
//! primitives: 300 polylines (12 points), 200 arrows, 200 rings, 100 point sets (4 points),
//! 100 plane quads, 50 sprites, 49 filled polygons and one adaptive grid, in the three depth
//! modes. Reported: the GPU time of the `overlays` pass (`Renderer::drain_gpu_timings`; needs
//! `TIMESTAMP_QUERY`), the whole frame's GPU time, and the CPU time of refilling the list plus
//! `encode` (tessellation included), against the same orbit without overlays.
//!
//! Budget: `overlays` pass GPU p99 <= 0.5 ms, asserted only with `AMG_STRICT_TIMING=1`.
//! `cargo bench -p amg-render --bench overlay -- --test` runs a tiny version.

#![allow(clippy::unwrap_used, clippy::expect_used, reason = "bench code")]

use std::collections::HashMap;
use std::hint::black_box;
use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_core::{Dir3, Plane, Point3, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::{
    Camera, DepthMode, ImageKey, Instance, LabelStyle, LineStyle, MeshData, OverlayList,
    PointStyle, Projection, Renderer, RendererConfig, RgbaImage, ViewId, ViewStyle, VisualStyle,
    upload_image, upload_mesh, wgpu,
};
use criterion::{Criterion, criterion_group, criterion_main};

const FRAMES: usize = 600;
const SIZE: [u32; 2] = [2560, 1440];
const PRIMITIVES: usize = 1000;

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
            eprintln!("no GPU, skipping the overlay bench: {e}");
            None
        }
    }
}

/// A UV sphere of radius 100 mm with about `2 * n * n` triangles.
fn sphere(n: u32) -> MeshData {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    for i in 0..=n {
        let polar = std::f64::consts::PI * f64::from(i) / f64::from(n);
        for j in 0..=n {
            let az = std::f64::consts::TAU * f64::from(j) / f64::from(n);
            let d = Vec3::new(polar.sin() * az.cos(), polar.sin() * az.sin(), polar.cos());
            positions.push((d * 100.0).as_vec3().to_array());
            normals.push(d.as_vec3().to_array());
        }
    }
    let mut indices = Vec::new();
    let row = n + 1;
    for i in 0..n {
        for j in 0..n {
            let (a, b, c, d) = (
                i * row + j,
                i * row + j + 1,
                (i + 1) * row + j,
                (i + 1) * row + j + 1,
            );
            indices.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }
    MeshData {
        origin: Point3::ZERO,
        positions,
        normals,
        indices,
        closed: true,
    }
}

fn orbit(i: usize, n: usize) -> Camera {
    let a = std::f64::consts::TAU * i as f64 / n as f64;
    Camera {
        eye: Point3::new(330.0 * a.cos(), 330.0 * a.sin(), 110.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.7 },
    }
}

/// The 1000-primitive overlay list of frame `frame`.
fn fill(list: &mut OverlayList, frame: usize, icon: ImageKey) {
    let modes = [
        DepthMode::Test,
        DepthMode::AlwaysOnTop,
        DepthMode::XRay { hidden_alpha: 0.3 },
    ];
    let style = |k: usize| LineStyle {
        width_px: 1.0 + (k % 3) as f32,
        color: [0.9, 0.6 + 0.1 * (k % 4) as f32, 0.1, 1.0],
        dash: (k.is_multiple_of(5)).then_some((6.0, 3.0)),
        depth: modes[k % 3],
    };
    let t = frame as f64 * 0.01;
    let at = |k: usize| {
        let a = k as f64 * 2.399 + t;
        let r = 40.0 + (k % 7) as f64 * 15.0;
        Point3::new(r * a.cos(), r * a.sin(), ((k % 11) as f64 - 5.0) * 15.0)
    };
    let mut n = 0;
    for k in 0..300 {
        let p = at(k);
        let pts: Vec<Point3> = (0..12)
            .map(|i| p + Vec3::new(i as f64 * 3.0, (i as f64 * 0.7).sin() * 5.0, 0.0))
            .collect();
        list.polyline(&pts, style(k), None);
        n += 1;
    }
    for k in 0..200 {
        let d = Dir3::new_unchecked(Vec3::new((k as f64).cos(), (k as f64).sin(), 0.5).normalize());
        list.arrow(
            at(k + 300),
            d,
            60.0,
            style(k),
            Some((amg_render::PickId(1000), k as u32)),
        );
        n += 1;
    }
    for k in 0..200 {
        list.ring(at(k + 500), Dir3::Z, 25.0, style(k), None);
        n += 1;
    }
    for k in 0..100 {
        let p = at(k + 700);
        let pts = [p, p + Vec3::X * 4.0, p + Vec3::Y * 4.0, p + Vec3::Z * 4.0];
        list.points(
            &pts,
            PointStyle {
                size_px: 6.0,
                color: [0.1, 0.8, 0.9, 1.0],
                depth: modes[k % 3],
            },
            None,
        );
        n += 1;
    }
    for k in 0..100 {
        let plane = Plane {
            origin: at(k + 800),
            x: Dir3::X,
            y: Dir3::Y,
            normal: Dir3::Z,
        };
        let alpha = if k.is_multiple_of(2) { 0.35 } else { 1.0 };
        list.plane_quad(&plane, [6.0, 4.0], [0.2, 0.4, 0.9, alpha], style(k), None);
        n += 1;
    }
    for k in 0..50 {
        list.sprite(at(k + 900), icon, [[0.0, 0.0], [1.0, 1.0]], 16.0, None);
        n += 1;
    }
    for k in 0..49 {
        let p = at(k + 950);
        let pts = [
            p,
            p + Vec3::X * 8.0,
            p + Vec3::new(8.0, 8.0, 0.0),
            p + Vec3::Y * 8.0,
        ];
        list.polygon(
            &pts,
            &[[0, 1, 2], [0, 2, 3]],
            [0.8, 0.2, 0.2, 0.5],
            modes[k % 3],
            None,
        );
        n += 1;
    }
    list.grid(&Plane::XY, [300.0, 300.0], [0.6, 0.6, 0.6, 0.8]);
    n += 1;
    list.label(Point3::ZERO, "origin", LabelStyle::default());
    assert_eq!(n, PRIMITIVES);
    debug_assert_eq!(list.primitive_count(), PRIMITIVES);
}

struct Run {
    gpu_ms: Vec<f64>,
    passes: HashMap<&'static str, Vec<f64>>,
    cpu_ms: Vec<f64>,
}

#[allow(
    clippy::too_many_arguments,
    reason = "one measurement from its plain inputs"
)]
fn run(
    r: &mut Renderer,
    view: ViewId,
    gpu: &Gpu,
    frames: usize,
    overlays: bool,
    icon: ImageKey,
) -> Run {
    let (device, queue) = (gpu.device(), gpu.queue());
    let mut out = Run {
        gpu_ms: Vec::new(),
        passes: HashMap::new(),
        cpu_ms: Vec::new(),
    };
    let mut in_flight: Option<wgpu::SubmissionIndex> = None;
    let harvest = |r: &mut Renderer, out: &mut Run| {
        for t in r.drain_gpu_timings(view) {
            out.gpu_ms.push(f64::from(t.total_ms));
            for (name, ms) in t.passes {
                out.passes.entry(name).or_default().push(f64::from(ms));
            }
        }
    };
    for i in 0..frames {
        r.view_mut(view).unwrap().camera = orbit(i, frames);
        let start = Instant::now();
        let list = r.scene_mut().begin_overlays();
        if overlays {
            fill(list, i, icon);
        }
        let mut encoder = device.create_command_encoder(&Default::default());
        black_box(r.encode(view, &mut encoder).unwrap());
        out.cpu_ms.push(start.elapsed().as_secs_f64() * 1e3);
        let index = queue.submit([encoder.finish()]);
        if let Some(prev) = in_flight.replace(index) {
            device
                .poll(wgpu::PollType::Wait {
                    submission_index: Some(prev),
                    timeout: Some(Duration::from_secs(10)),
                })
                .unwrap();
        }
        harvest(r, &mut out);
    }
    device.poll(wgpu::PollType::wait_indefinitely()).unwrap();
    harvest(r, &mut out);
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

/// Done-when 6: 1000 overlay primitives cost <= 0.5 ms GPU.
fn overlay_budget(_c: &mut Criterion) {
    let Some(gpu) = gpu() else {
        return;
    };
    let (size, frames, n) = if test_mode() {
        ([64, 64], 5, 16)
    } else {
        (SIZE, FRAMES, 707)
    };
    let mut r = Renderer::new(
        Arc::clone(&gpu),
        &RendererConfig {
            msaa_samples: 4,
            ..Default::default()
        },
    )
    .unwrap();
    let mesh = upload_mesh(&gpu, &sphere(n), &JobCtx::detached()).unwrap();
    let tris = mesh.triangle_count();
    let key = r.scene_mut().insert_mesh(mesh);
    r.scene_mut()
        .add_instance(Instance::new(key, Transform::IDENTITY));
    let icon = r.scene_mut().insert_image(
        upload_image(
            &gpu,
            &RgbaImage {
                size: [16, 16],
                pixels: (0..16 * 16 * 4).map(|i| (i * 37 % 256) as u8).collect(),
            },
        )
        .unwrap(),
    );
    let view = r.create_view(size);
    r.view_mut(view).unwrap().style = ViewStyle {
        visual: VisualStyle::ShadedVisibleEdges,
        ..Default::default()
    };
    // Build the variants (overlay pipelines included) before measuring.
    fill(r.scene_mut().begin_overlays(), 0, icon);
    assert!(r.prepare_view(view));
    r.capture(view).unwrap().wait().unwrap();
    assert!(r.prepare_view(view));

    let base = run(&mut r, view, &gpu, frames, false, icon);
    let with = run(&mut r, view, &gpu, frames, true, icon);
    let pass = with.passes.get("overlays").cloned().unwrap_or_default();
    eprintln!(
        "overlay bench: {}x{}, 4x MSAA, {tris} tris, {PRIMITIVES} primitives, {frames} frames",
        size[0], size[1]
    );
    eprintln!(
        "  overlays pass GPU: p50 {:.3} ms, p99 {:.3} ms, max {:.3} ms ({} frames timed)",
        percentile(&pass, 0.5),
        percentile(&pass, 0.99),
        percentile(&pass, 1.0),
        pass.len()
    );
    eprintln!(
        "  frame GPU without / with overlays: p50 {:.3} / {:.3} ms, p99 {:.3} / {:.3} ms",
        percentile(&base.gpu_ms, 0.5),
        percentile(&with.gpu_ms, 0.5),
        percentile(&base.gpu_ms, 0.99),
        percentile(&with.gpu_ms, 0.99),
    );
    eprintln!(
        "  CPU refill + encode without / with overlays: p50 {:.3} / {:.3} ms, p99 {:.3} / {:.3} ms",
        percentile(&base.cpu_ms, 0.5),
        percentile(&with.cpu_ms, 0.5),
        percentile(&base.cpu_ms, 0.99),
        percentile(&with.cpu_ms, 0.99),
    );
    if strict() && !test_mode() {
        let p99 = percentile(&pass, 0.99);
        assert!(!pass.is_empty(), "no overlay timings (TIMESTAMP_QUERY?)");
        assert!(p99 <= 0.5, "overlays pass GPU p99 {p99:.3} ms > 0.5 ms");
    }
}

criterion_group!(benches, overlay_budget);
criterion_main!(benches);
