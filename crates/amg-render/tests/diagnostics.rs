//! RND-13 GPU tests: device loss, VRAM accounting and the uncaptured-error log. Named `gpu_*`
//! (nextest test-group `gpu`, serial); they skip without an adapter and fail under
//! `AMG_REQUIRE_GPU=1` (CI runs them on WARP).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use amg_core::{Point3, Transform};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig, GpuError};
use amg_render::{
    EdgeData, FaceRange, Instance, MeshData, PickFilter, PickRegion, PickRequest, PointData,
    Polyline, RenderError, Renderer, RendererConfig, RgbaImage, ViewId, upload_edges, upload_image,
    upload_mesh, upload_mesh_with_faces, upload_points, upload_scalar_field, upload_vector_field,
    wgpu,
};

fn gpu() -> Option<Arc<Gpu>> {
    if !amg_testkit::gpu_available() {
        return None;
    }
    let mut cfg = GpuConfig::from_env();
    cfg.optional_features |= amg_render::OPTIONAL_FEATURES;
    let gpu = Gpu::new_blocking(&cfg).expect("adapter probed, device opens");
    let info = gpu.info();
    eprintln!("adapter: {} ({:?})", info.name, info.backend);
    Some(gpu)
}

fn quad() -> MeshData {
    MeshData {
        positions: vec![
            [-20.0, 0.0, -20.0],
            [20.0, 0.0, -20.0],
            [20.0, 0.0, 20.0],
            [-20.0, 0.0, 20.0],
        ],
        normals: vec![[0.0, -1.0, 0.0]; 4],
        indices: vec![0, 1, 2, 0, 2, 3],
        ..Default::default()
    }
}

/// A renderer with one quad in one view, with every pipeline built.
fn scene(gpu: &Arc<Gpu>, samples: u32, size: [u32; 2]) -> (Renderer, ViewId) {
    let cfg = RendererConfig {
        msaa_samples: samples,
        ..Default::default()
    };
    let mut r = Renderer::new(Arc::clone(gpu), &cfg).unwrap();
    let mesh = upload_mesh(gpu, &quad(), &JobCtx::detached()).unwrap();
    let key = r.scene_mut().insert_mesh(mesh);
    r.scene_mut()
        .add_instance(Instance::new(key, Transform::IDENTITY));
    let view = r.create_view(size);
    assert!(r.prepare_view(view), "pipeline variants did not finish");
    (r, view)
}

fn encoder(gpu: &Gpu) -> wgpu::CommandEncoder {
    gpu.device().create_command_encoder(&Default::default())
}

fn is_lost(e: &RenderError) -> bool {
    matches!(e, RenderError::Gpu(GpuError::DeviceLost(_)))
}

fn point_pick() -> PickRequest {
    PickRequest {
        region: PickRegion::Point { px: [32.0, 32.0] },
        filter: PickFilter::FACES,
    }
}

/// Done-when 1: after `device.destroy()` the next `encode` returns `DeviceLost`, as does
/// everything after it, without a panic. No error log is installed, so any use of the dead
/// device that raised a wgpu error would panic here.
#[test]
fn gpu_encode_after_destroy_returns_device_lost() {
    let Some(gpu) = gpu() else { return };
    let (mut r, view) = scene(&gpu, 4, [64, 64]);
    r.capture(view).unwrap().wait().unwrap();
    let queued = r.request_pick(view, point_pick());

    gpu.device().destroy();
    let mut enc = encoder(&gpu);
    let err = r.encode(view, &mut enc).unwrap_err();
    assert!(is_lost(&err), "{err}");
    // And so on, including a resized view (which would rebuild targets).
    r.set_view_size(view, [128, 32]);
    assert!(is_lost(&r.encode(view, &mut enc).unwrap_err()));
    assert!(is_lost(&r.capture(view).unwrap_err()));

    let picks = r.poll_picks();
    assert_eq!(picks.len(), 1, "{picks:?}");
    assert_eq!(picks[0].0, queued);
    assert!(is_lost(picks[0].1.as_ref().unwrap_err()), "{picks:?}");
    assert_eq!(r.pending_picks(), 0);

    let ctx = JobCtx::detached();
    assert!(is_lost(&upload_mesh(&gpu, &quad(), &ctx).unwrap_err()));
    let image = RgbaImage {
        size: [1, 1],
        pixels: vec![0; 4],
    };
    assert!(is_lost(&upload_image(&gpu, &image).unwrap_err()));
    // The diagnostics stay readable.
    assert!(r.diagnostics(view).is_some());
}

/// A `destroy()` with frames still running: wgpu reports the loss once they finish. Until then
/// `encode` may succeed; it then returns `DeviceLost`, and nothing panics (the app installs the
/// error log, which takes whatever the dead device raises).
#[test]
fn gpu_destroy_with_frames_in_flight_ends_in_device_lost() {
    let Some(gpu) = gpu() else { return };
    let (mut r, view) = scene(&gpu, 4, [512, 512]);
    r.capture_errors();
    for _ in 0..8 {
        let mut enc = encoder(&gpu);
        r.encode(view, &mut enc).unwrap();
        gpu.queue().submit([enc.finish()]);
    }
    gpu.device().destroy();
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut early = 0u32;
    let err = loop {
        let mut enc = encoder(&gpu);
        match r.encode(view, &mut enc) {
            Err(e) => break e,
            Ok(_) => early += 1,
        }
        assert!(
            Instant::now() < deadline,
            "no DeviceLost after {early} frames"
        );
        std::thread::sleep(Duration::from_millis(1));
    };
    eprintln!("{early} encode(s) succeeded while the queue drained");
    assert!(is_lost(&err), "{err}");
}

/// Done-when 2: the VRAM counters equal the sums of the resources' `gpu_bytes`, category by
/// category, as resources come and go.
#[test]
fn gpu_vram_counters_equal_the_sum_of_gpu_bytes() {
    let Some(gpu) = gpu() else { return };
    let ctx = JobCtx::detached();
    let mut r = Renderer::new(Arc::clone(&gpu), &RendererConfig::default()).unwrap();
    let empty = r.vram_usage();
    assert_eq!(empty.scene.total(), 0);
    assert_eq!(
        (empty.view_targets, empty.pick_targets, empty.view_buffers),
        (0, 0, 0)
    );
    assert!(empty.instance_buffers > 0 && empty.other > 0, "{empty:?}");

    let plain = upload_mesh(&gpu, &quad(), &ctx).unwrap();
    let faces = [FaceRange {
        first_tri: 0,
        tri_count: 2,
        face: 0,
    }];
    let flat = MeshData {
        normals: Vec::new(),
        ..quad()
    };
    let faced = upload_mesh_with_faces(&gpu, &flat, &faces, &ctx).unwrap();
    let scalar = upload_scalar_field(&gpu, &plain, &[0.0, 1.0, 2.0, 3.0], &ctx).unwrap();
    let vector = upload_vector_field(&gpu, &plain, &[[1.0, 0.0, 0.0]; 4], &ctx).unwrap();
    let edges = upload_edges(
        &gpu,
        &EdgeData {
            origin: Point3::ZERO,
            points: quad().positions,
            polylines: vec![Polyline {
                first: 0,
                len: 4,
                id: 0,
                closed: true,
            }],
        },
        &ctx,
    )
    .unwrap();
    let points = upload_points(
        &gpu,
        &PointData {
            points: quad().positions,
            ..Default::default()
        },
        &ctx,
    )
    .unwrap();
    let image = upload_image(
        &gpu,
        &RgbaImage {
            size: [8, 4],
            pixels: vec![255; 128],
        },
    )
    .unwrap();

    let meshes = plain.gpu_bytes() + faced.gpu_bytes();
    let fields = scalar.gpu_bytes() + vector.gpu_bytes();
    let (edge_bytes, point_bytes, image_bytes) =
        (edges.gpu_bytes(), points.gpu_bytes(), image.gpu_bytes());
    assert_eq!(image_bytes, 128);
    assert!(meshes > 0 && fields > 0 && edge_bytes > 0 && point_bytes > 0);

    let s = r.scene_mut();
    let plain_bytes = plain.gpu_bytes();
    let plain = s.insert_mesh(plain);
    s.insert_mesh(faced);
    s.insert_field(scalar);
    s.insert_field(vector);
    s.insert_edges(edges);
    s.insert_points(points);
    s.insert_image(image);
    s.add_instance(Instance::new(plain, Transform::IDENTITY));

    let v = r.vram_usage();
    assert_eq!(v.scene.meshes, meshes);
    assert_eq!(v.scene.fields, fields);
    assert_eq!(v.scene.edges, edge_bytes);
    assert_eq!(v.scene.points, point_bytes);
    assert_eq!(v.scene.images, image_bytes);
    let sum = meshes + fields + edge_bytes + point_bytes + image_bytes;
    assert_eq!(v.scene.total(), sum);
    assert_eq!(r.scene().gpu_bytes(), sum);
    assert_eq!(v.total(), v.categories().iter().map(|c| c.1).sum::<u64>());

    assert!(r.scene_mut().remove_mesh(plain));
    assert_eq!(r.vram_usage().scene.meshes, meshes - plain_bytes);
    assert_eq!(r.scene().gpu_bytes(), sum - plain_bytes);
}

/// The renderer's own categories follow the views: targets by size and sample count, pick
/// targets and buffers on use, all released with the view.
#[test]
fn gpu_vram_counts_view_targets_and_buffers() {
    let Some(gpu) = gpu() else { return };
    // Per pixel: normal 4, depth 4 and (MSAA) colour 8, each per sample; overlay depth 4,
    // resolved colour 8 and the final image 4.
    for (samples, per_pixel) in [(1u32, 4 + 4 + 16), (4, (8 + 4 + 4) * 4 + 16)] {
        let (mut r, view) = scene(&gpu, samples, [64, 32]);
        assert_eq!(r.vram_usage().view_targets, 0, "no targets before a frame");
        r.capture(view).unwrap().wait().unwrap();
        let v = r.vram_usage();
        assert_eq!(v.view_targets, per_pixel * 64 * 32, "{samples}x");
        assert_eq!(v.pick_targets, 0);
        assert!(v.view_buffers > 0);

        r.request_pick(view, point_pick());
        r.capture(view).unwrap().wait().unwrap();
        assert!(r.vram_usage().pick_targets > 0);

        let d = r.diagnostics(view).unwrap();
        assert_eq!(d.frame.triangles, 2);
        assert_eq!(d.vram, r.vram_usage());
        assert_eq!(d.gpu_errors, 0);
        if gpu.has(wgpu::Features::TIMESTAMP_QUERY) {
            // Timings are harvested by the following frames.
            r.capture(view).unwrap().wait().unwrap();
            let t = r.diagnostics(view).unwrap().timings.expect("a timed frame");
            assert!(t.passes.iter().any(|p| p.0 == "opaque"), "{t:?}");
        }

        r.remove_view(view);
        let v = r.vram_usage();
        assert_eq!((v.view_targets, v.pick_targets, v.view_buffers), (0, 0, 0));
    }
}

/// Collects the text of every tracing event.
#[derive(Clone, Default)]
struct Events(Arc<Mutex<Vec<(tracing::Level, String)>>>);

struct Text(String);

impl tracing::field::Visit for Text {
    fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
        self.0 += &format!("{}={value:?} ", field.name());
    }
}

impl tracing::Subscriber for Events {
    fn enabled(&self, _: &tracing::Metadata<'_>) -> bool {
        true
    }
    fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::span::Id {
        tracing::span::Id::from_u64(1)
    }
    fn record(&self, _: &tracing::span::Id, _: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _: &tracing::span::Id, _: &tracing::span::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        let mut text = Text(String::new());
        event.record(&mut text);
        self.0
            .lock()
            .unwrap()
            .push((*event.metadata().level(), text.0));
    }
    fn enter(&self, _: &tracing::span::Id) {}
    fn exit(&self, _: &tracing::span::Id) {}
}

/// Done-when 3: a forced validation error appears in the log once, with its label, however
/// often it repeats, and does not panic.
#[test]
fn gpu_validation_error_is_logged_once_with_its_label() {
    let Some(gpu) = gpu() else { return };
    let (mut r, view) = scene(&gpu, 1, [16, 16]);
    let log = r.capture_errors();
    let events = Events::default();
    tracing::subscriber::with_default(events.clone(), || {
        for _ in 0..3 {
            // MAP_READ may only be combined with COPY_DST: a validation error.
            let _bad = gpu.device().create_buffer(&wgpu::BufferDescriptor {
                label: Some("amg-test-bad-buffer"),
                size: 16,
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::STORAGE,
                mapped_at_creation: false,
            });
        }
    });
    let seen = events.0.lock().unwrap().clone();
    let hits: Vec<_> = seen
        .iter()
        .filter(|(_, text)| text.contains("amg-test-bad-buffer"))
        .collect();
    assert_eq!(hits.len(), 1, "{seen:#?}");
    assert_eq!(hits[0].0, tracing::Level::ERROR);
    assert!(hits[0].1.contains("validation"), "{}", hits[0].1);
    eprintln!("logged: {}", hits[0].1);
    assert_eq!((log.total(), log.distinct()), (3, 1));
    // One log per renderer, and the overlay data counts the errors.
    assert_eq!(r.capture_errors().total(), 3);
    assert_eq!(r.diagnostics(view).unwrap().gpu_errors, 3);
    // The device is still usable.
    r.capture(view).unwrap().wait().unwrap();
}
