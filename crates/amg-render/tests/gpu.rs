//! RND-01 GPU tests. Named `gpu_*` (nextest test-group `gpu`, serial). They skip without an
//! adapter and fail under `AMG_REQUIRE_GPU=1` (CI runs them on WARP).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::Arc;

use amg_core::{Dir3, Point3, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::{
    Background, Camera, GpuMesh, Instance, MeshData, Projection, Renderer, RendererConfig,
    RgbaImage, ViewId, ViewStyle, upload_mesh,
};

const BACKGROUND: [f32; 3] = [0.02, 0.03, 0.05];

fn gpu() -> Option<Arc<Gpu>> {
    if !amg_testkit::gpu_available() {
        return None;
    }
    let gpu = Gpu::new_blocking(&GpuConfig::from_env()).expect("adapter probed, device opens");
    let info = gpu.info();
    eprintln!("adapter: {} ({:?})", info.name, info.backend);
    Some(gpu)
}

fn sphere_data(radius: f64, subdivisions: u32) -> MeshData {
    let mesh = amg_testkit::shapes::icosphere(Point3::ZERO, radius, subdivisions);
    MeshData {
        origin: Point3::ZERO,
        positions: mesh
            .positions
            .iter()
            .map(|p| p.as_vec3().to_array())
            .collect(),
        normals: mesh
            .positions
            .iter()
            .map(|p| p.normalize().as_vec3().to_array())
            .collect(),
        indices: mesh.triangles.iter().flatten().copied().collect(),
        closed: true,
    }
}

/// A 1 mm cube centred on its local origin, with the given world origin.
fn cube_data(origin: Point3) -> MeshData {
    let mesh = amg_testkit::shapes::cube(Point3::splat(-0.5), 1.0);
    MeshData {
        origin,
        positions: mesh
            .positions
            .iter()
            .map(|p| p.as_vec3().to_array())
            .collect(),
        normals: Vec::new(),
        indices: mesh.triangles.iter().flatten().copied().collect(),
        closed: true,
    }
}

fn upload(gpu: &Gpu, data: &MeshData) -> GpuMesh {
    upload_mesh(gpu, data, &JobCtx::detached()).unwrap()
}

fn solid_style() -> ViewStyle {
    ViewStyle {
        background: Background::Solid(BACKGROUND),
        ..Default::default()
    }
}

/// A renderer showing one instance of `mesh` through `camera`.
fn scene(
    gpu: &Arc<Gpu>,
    mesh: GpuMesh,
    transform: Transform,
    camera: Camera,
    size: [u32; 2],
) -> (Renderer, ViewId) {
    let mut renderer = Renderer::new(Arc::clone(gpu), &RendererConfig::default()).unwrap();
    let key = renderer.scene_mut().insert_mesh(mesh);
    renderer
        .scene_mut()
        .add_instance(Instance::new(key, transform));
    let view = renderer.create_view(size);
    let v = renderer.view_mut(view).unwrap();
    v.camera = camera;
    v.style = solid_style();
    (renderer, view)
}

fn perspective(eye: Point3, target: Point3) -> Camera {
    Camera {
        eye,
        target,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.7 },
    }
}

fn capture(renderer: &mut Renderer, view: ViewId) -> RgbaImage {
    renderer.capture(view).unwrap().wait().unwrap()
}

/// The sRGB bytes the background clears to.
fn background_bytes() -> [u8; 4] {
    let encode = |c: f32| {
        let s = if c <= 0.003_130_8 {
            12.92 * c
        } else {
            1.055 * c.powf(1.0 / 2.4) - 0.055
        };
        (s * 255.0).round() as u8
    };
    [
        encode(BACKGROUND[0]),
        encode(BACKGROUND[1]),
        encode(BACKGROUND[2]),
        255,
    ]
}

fn near(a: [u8; 4], b: [u8; 4], tolerance: u8) -> bool {
    a.iter().zip(&b).all(|(x, y)| x.abs_diff(*y) <= tolerance)
}

#[test]
fn gpu_spin_smoke() {
    let Some(gpu) = gpu() else {
        return;
    };
    let mesh = upload(&gpu, &sphere_data(50.0, 3));
    let cam = perspective(Point3::new(0.0, -300.0, 0.0), Point3::ZERO);
    let (mut renderer, view) = scene(&gpu, mesh, Transform::IDENTITY, cam, [256, 256]);

    let first = capture(&mut renderer, view);
    let second = capture(&mut renderer, view);
    assert_eq!(first, second, "two captures must be bit-identical");

    let background = background_bytes();
    let centre = first.pixel(128, 128);
    assert!(
        !near(centre, background, 8),
        "centre {centre:?} is background {background:?}"
    );
    for (x, y) in [(0, 0), (255, 0), (0, 255), (255, 255)] {
        let corner = first.pixel(x, y);
        assert!(
            near(corner, background, 1),
            "corner ({x},{y}) {corner:?} is not background {background:?}"
        );
    }
}

/// Far from the origin only RTE keeps the f32 positions small: a naive path would quantise a
/// 1 mm cube at 1e5 mm into visible steps.
#[test]
fn gpu_rte_cube_far_from_origin_matches_origin() {
    let Some(gpu) = gpu() else {
        return;
    };
    let offset = Vec3::new(1e5, 1e5, 0.0);
    let pose = Vec3::new(1.4, -2.2, 1.1);
    let size = [256, 256];

    let at_origin = {
        let mesh = upload(&gpu, &cube_data(Point3::ZERO));
        let cam = perspective(pose, Point3::ZERO);
        let (mut r, v) = scene(&gpu, mesh, Transform::IDENTITY, cam, size);
        capture(&mut r, v)
    };
    let by_mesh_origin = {
        let mesh = upload(&gpu, &cube_data(offset));
        let cam = perspective(offset + pose, offset);
        let (mut r, v) = scene(&gpu, mesh, Transform::IDENTITY, cam, size);
        capture(&mut r, v)
    };
    let by_transform = {
        let mesh = upload(&gpu, &cube_data(Point3::ZERO));
        let cam = perspective(offset + pose, offset);
        let t = Transform::from_translation(offset);
        let (mut r, v) = scene(&gpu, mesh, t, cam, size);
        capture(&mut r, v)
    };

    for (name, far) in [
        ("mesh origin", &by_mesh_origin),
        ("transform", &by_transform),
    ] {
        let worst = at_origin
            .pixels
            .iter()
            .zip(&far.pixels)
            .map(|(a, b)| a.abs_diff(*b))
            .max()
            .unwrap();
        assert!(worst <= 1, "{name}: worst channel difference {worst}");
    }
    // The cube is on screen, so the comparison is not vacuous.
    assert!(!near(at_origin.pixel(128, 128), background_bytes(), 8));
}

#[test]
fn gpu_flat_shading_matches_smooth_on_a_facing_surface() {
    let Some(gpu) = gpu() else {
        return;
    };
    // A quad facing the camera has the same Lambert term flat or smooth.
    let quad = MeshData {
        origin: Point3::ZERO,
        positions: vec![
            [-40.0, 0.0, -40.0],
            [40.0, 0.0, -40.0],
            [40.0, 0.0, 40.0],
            [-40.0, 0.0, 40.0],
        ],
        normals: vec![[0.0, -1.0, 0.0]; 4],
        indices: vec![0, 1, 2, 0, 2, 3],
        closed: false,
    };
    let cam = perspective(Point3::new(0.0, -200.0, 0.0), Point3::ZERO);

    let smooth = {
        let (mut r, v) = scene(
            &gpu,
            upload(&gpu, &quad),
            Transform::IDENTITY,
            cam,
            [64, 64],
        );
        capture(&mut r, v)
    };
    let from_empty_normals = {
        let no_normals = MeshData {
            normals: Vec::new(),
            ..quad.clone()
        };
        let (mut r, v) = scene(
            &gpu,
            upload(&gpu, &no_normals),
            Transform::IDENTITY,
            cam,
            [64, 64],
        );
        capture(&mut r, v)
    };
    let from_view_toggle = {
        let (mut r, v) = scene(
            &gpu,
            upload(&gpu, &quad),
            Transform::IDENTITY,
            cam,
            [64, 64],
        );
        r.view_mut(v).unwrap().style.flat_shading = true;
        capture(&mut r, v)
    };

    let centre = smooth.pixel(32, 32);
    assert!(!near(centre, background_bytes(), 8));
    for flat in [&from_empty_normals, &from_view_toggle] {
        assert!(
            near(flat.pixel(32, 32), centre, 2),
            "{:?} vs {centre:?}",
            flat.pixel(32, 32)
        );
    }
}

#[test]
fn gpu_msaa_toggle_and_resize_keep_rendering() {
    let Some(gpu) = gpu() else {
        return;
    };
    let mesh = upload(&gpu, &sphere_data(50.0, 2));
    let cam = perspective(Point3::new(0.0, -300.0, 0.0), Point3::ZERO);
    let (mut renderer, view) = scene(&gpu, mesh, Transform::IDENTITY, cam, [128, 128]);
    let four = capture(&mut renderer, view);

    renderer.set_msaa_samples(1).unwrap();
    renderer.set_view_size(view, [64, 96]);
    let one = capture(&mut renderer, view);
    assert_eq!(one.size, [64, 96]);
    assert!(!near(one.pixel(32, 48), background_bytes(), 8));

    renderer.set_msaa_samples(4).unwrap();
    renderer.set_view_size(view, [128, 128]);
    assert_eq!(capture(&mut renderer, view), four);

    assert!(renderer.set_msaa_samples(3).is_err());
}

#[test]
fn gpu_zero_sized_view_records_nothing() {
    let Some(gpu) = gpu() else {
        return;
    };
    let mut renderer = Renderer::new(Arc::clone(&gpu), &RendererConfig::default()).unwrap();
    let view = renderer.create_view([0, 16]);
    let mut encoder = gpu.device().create_command_encoder(&Default::default());
    renderer.encode(view, &mut encoder).unwrap();
    assert!(renderer.final_texture(view).is_none());
    assert!(renderer.capture(view).is_err());
}

#[test]
fn gpu_upload_rejects_malformed_data() {
    let Some(gpu) = gpu() else {
        return;
    };
    let ctx = JobCtx::detached();
    let bad_normals = MeshData {
        positions: vec![[0.0; 3]; 3],
        normals: vec![[0.0, 0.0, 1.0]],
        indices: vec![0, 1, 2],
        ..Default::default()
    };
    assert!(upload_mesh(&gpu, &bad_normals, &ctx).is_err());
    let bad_indices = MeshData {
        positions: vec![[0.0; 3]; 3],
        indices: vec![0, 1],
        ..Default::default()
    };
    assert!(upload_mesh(&gpu, &bad_indices, &ctx).is_err());
}

#[test]
fn timing_core_pipeline_creation_within_budget() {
    let Some(gpu) = gpu() else {
        return;
    };
    let renderer = Renderer::new(Arc::clone(&gpu), &RendererConfig::default()).unwrap();
    let total: f64 = renderer.core_pipeline_timings().iter().map(|t| t.ms).sum();
    eprintln!(
        "core pipelines: {total:.1} ms ({:?})",
        renderer.core_pipeline_timings()
    );
    if amg_testkit_strict_timing() {
        assert!(total <= 150.0, "core pipelines took {total:.1} ms");
    }
}

fn amg_testkit_strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}
