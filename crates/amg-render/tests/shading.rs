//! RND-02 GPU tests: lighting against the CPU references, octahedral normals and the normal
//! target, MSAA and FXAA antialiasing, gradient backgrounds, pipeline variants built on jobs,
//! the depth pre-pass, double-sided open meshes, materials and GPU timings. Named `gpu_*`
//! (nextest test-group `gpu`, serial); they skip without an adapter and fail under
//! `AMG_REQUIRE_GPU=1` (CI runs them on WARP).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]
#![allow(clippy::float_cmp, reason = "cleared texels are exactly zero")]

use std::sync::Arc;

use amg_core::{Dir3, Point3, Quat, Transform, Vec3};
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_jobs::{Cancelled, JobCtx, JobSystem, JobSystemConfig};
use amg_render::{
    Background, Camera, GpuMesh, Instance, Lighting, Material, MeshData, Projection, Renderer,
    RendererConfig, RgbaImage, ViewId, ViewStyle, oct_decode, shade_reference, upload_mesh, wgpu,
};

const BLACK: [f32; 3] = [0.0, 0.0, 0.0];

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

fn upload(gpu: &Gpu, data: &MeshData) -> GpuMesh {
    upload_mesh(gpu, data, &JobCtx::detached()).unwrap()
}

fn srgb_byte(linear: f32) -> u8 {
    let c = linear.clamp(0.0, 1.0);
    let s = if c <= 0.003_130_8 {
        12.92 * c
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    };
    (s * 255.0).round() as u8
}

fn srgb_to_linear(byte: u8) -> f32 {
    let s = f32::from(byte) / 255.0;
    if s <= 0.040_45 {
        s / 12.92
    } else {
        ((s + 0.055) / 1.055).powf(2.4)
    }
}

/// A square of side `side` in the local XY plane, normal +Z, counter-clockwise from +Z.
fn square(side: f32, normals: bool, closed: bool) -> MeshData {
    let h = side / 2.0;
    MeshData {
        origin: Point3::ZERO,
        positions: vec![[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]],
        normals: if normals {
            vec![[0.0, 0.0, 1.0]; 4]
        } else {
            Vec::new()
        },
        indices: vec![0, 1, 2, 0, 2, 3],
        closed,
    }
}

/// Orthographic camera on −Y looking at the origin: view x = world X, view y = world Z, view z
/// = world −Y.
fn front_ortho(height: f64) -> Camera {
    Camera {
        eye: Point3::new(0.0, -500.0, 0.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Orthographic { height },
    }
}

/// World direction of a view-space vector for [`front_ortho`].
fn world_of_view(n: Vec3) -> Vec3 {
    Vec3::new(n.x, -n.z, n.y)
}

fn new_renderer(gpu: &Arc<Gpu>, cfg: &RendererConfig) -> Renderer {
    Renderer::new(Arc::clone(gpu), cfg).unwrap()
}

fn add_view(r: &mut Renderer, camera: Camera, size: [u32; 2], style: ViewStyle) -> ViewId {
    let view = r.create_view(size);
    let v = r.view_mut(view).unwrap();
    v.camera = camera;
    v.style = style;
    view
}

fn solid(lighting: Lighting) -> ViewStyle {
    ViewStyle {
        background: Background::Solid(BLACK),
        lighting,
        ..Default::default()
    }
}

/// Prepares every variant the view needs, then captures it.
fn capture(r: &mut Renderer, view: ViewId) -> RgbaImage {
    assert!(r.prepare_view(view), "pipeline variants did not finish");
    r.capture(view).unwrap().wait().unwrap()
}

fn near(a: [u8; 4], b: [u8; 4], tolerance: u8) -> bool {
    a.iter().zip(&b).all(|(x, y)| x.abs_diff(*y) <= tolerance)
}

/// Test normals in view space: polar angles 0..70° in five azimuths.
fn test_normals() -> Vec<Vec3> {
    let mut out = vec![Vec3::Z];
    for theta in [20.0f64, 40.0, 55.0, 70.0] {
        for k in 0..5 {
            let phi = (k as f64 * 72.0 + theta).to_radians();
            let t = theta.to_radians();
            out.push(Vec3::new(t.sin() * phi.cos(), t.sin() * phi.sin(), t.cos()));
        }
    }
    out
}

/// One small square per normal on a 5-wide grid, each rotated so its normal is `n` (view).
fn place_squares(
    r: &mut Renderer,
    mesh: amg_render::MeshKey,
    material: Material,
) -> Vec<(Vec3, Point3)> {
    test_normals()
        .into_iter()
        .enumerate()
        .map(|(i, n)| {
            let centre = Point3::new(
                (i % 5) as f64 * 22.0 - 44.0,
                0.0,
                (i / 5) as f64 * 22.0 - 44.0,
            );
            let rotation = Quat::from_rotation_arc(Vec3::Z, world_of_view(n));
            let mut inst =
                Instance::new(mesh, Transform::from_rotation_translation(rotation, centre));
            inst.material = material;
            r.scene_mut().add_instance(inst);
            (n, centre)
        })
        .collect()
}

/// Done-when 4: lighting at known normals matches the CPU reference within 2/255, for every
/// lighting model, dielectric and metal, oct-encoded and derivative normals.
#[test]
fn gpu_lighting_matches_the_cpu_reference_at_known_normals() {
    let Some(gpu) = gpu() else {
        return;
    };
    let size = [240, 240];
    let materials = [
        Material {
            base_color: [0.8, 0.45, 0.2, 1.0],
            metallic: 0.0,
            roughness: 0.6,
            cap_color: None,
        },
        Material {
            base_color: [0.9, 0.9, 0.95, 1.0],
            metallic: 1.0,
            roughness: 0.35,
            cap_color: None,
        },
    ];
    let mut checked = 0;
    let mut worst = 0u8;
    for with_normals in [true, false] {
        for material in materials {
            let mut r = new_renderer(&gpu, &RendererConfig::default());
            let mesh = r
                .scene_mut()
                .insert_mesh(upload(&gpu, &square(12.0, with_normals, true)));
            let squares = place_squares(&mut r, mesh, material);
            let cam = front_ortho(120.0);
            let view = add_view(&mut r, cam, size, solid(Lighting::Studio));
            for lighting in [Lighting::Studio, Lighting::Matcap, Lighting::Flat] {
                r.view_mut(view).unwrap().style.lighting = lighting;
                let image = capture(&mut r, view);
                for &(n, centre) in &squares {
                    let p = cam.project(centre, size).unwrap();
                    let got = image.pixel(p[0] as u32, p[1] as u32);
                    let want = shade_reference(
                        lighting,
                        &material,
                        n.as_vec3().to_array(),
                        [0.0, 0.0, 1.0],
                        [0.0, 1.0, 0.0],
                    );
                    let want = [
                        srgb_byte(want[0]),
                        srgb_byte(want[1]),
                        srgb_byte(want[2]),
                        255,
                    ];
                    let diff = got
                        .iter()
                        .zip(&want)
                        .map(|(a, b)| a.abs_diff(*b))
                        .max()
                        .unwrap();
                    worst = worst.max(diff);
                    assert!(
                        diff <= 2,
                        "{lighting:?} normals={with_normals} {material:?} n={n:?}: got {got:?}, want {want:?}"
                    );
                    checked += 1;
                }
            }
        }
    }
    eprintln!(
        "lighting: {checked} pixels match the CPU reference, worst channel difference {worst}/255"
    );
    assert_eq!(checked, 2 * 2 * 3 * 21);
}

/// Done-when 3: a flat mesh without normals costs exactly 12 B/vertex + 12 B/triangle; with
/// normals, octahedral `Snorm16x2` adds 4 B/vertex.
#[test]
fn gpu_flat_mesh_vram_is_12_bytes_per_vertex_and_per_triangle() {
    let Some(gpu) = gpu() else {
        return;
    };
    let sphere = amg_testkit::shapes::icosphere(Point3::ZERO, 10.0, 4);
    let positions: Vec<[f32; 3]> = sphere
        .positions
        .iter()
        .map(|p| p.as_vec3().to_array())
        .collect();
    let indices: Vec<u32> = sphere.triangles.iter().flatten().copied().collect();
    let (v, t) = (positions.len() as u64, (indices.len() / 3) as u64);
    let flat = upload(
        &gpu,
        &MeshData {
            positions: positions.clone(),
            indices: indices.clone(),
            closed: true,
            ..Default::default()
        },
    );
    assert_eq!(
        flat.gpu_bytes(),
        12 * v + 12 * t,
        "{v} vertices, {t} triangles"
    );
    let smooth = upload(
        &gpu,
        &MeshData {
            normals: sphere
                .positions
                .iter()
                .map(|p| p.normalize().as_vec3().to_array())
                .collect(),
            positions,
            indices,
            closed: true,
            ..Default::default()
        },
    );
    assert_eq!(smooth.gpu_bytes(), 16 * v + 12 * t);
    eprintln!(
        "VRAM: flat {} B ({v} v, {t} t), smooth {} B, {:.2} B/tri smooth",
        flat.gpu_bytes(),
        smooth.gpu_bytes(),
        smooth.gpu_bytes() as f64 / t as f64
    );
}

/// Root-mean-square difference between rendered intensity and analytic pixel coverage over the
/// pixels the edge of a white rotated square crosses (on black): the staircase metric.
fn staircase_error(image: &RgbaImage, corners_px: &[[f64; 2]; 4]) -> (f64, usize) {
    let inside = |x: f64, y: f64| {
        (0..4).all(|i| {
            let a = corners_px[i];
            let b = corners_px[(i + 1) % 4];
            (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) <= 0.0
        }) || (0..4).all(|i| {
            let a = corners_px[i];
            let b = corners_px[(i + 1) % 4];
            (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) >= 0.0
        })
    };
    const N: usize = 16;
    let (mut sum, mut count) = (0.0, 0);
    for y in 0..image.size[1] {
        for x in 0..image.size[0] {
            let mut hits = 0;
            for j in 0..N {
                for i in 0..N {
                    let sx = f64::from(x) + (i as f64 + 0.5) / N as f64;
                    let sy = f64::from(y) + (j as f64 + 0.5) / N as f64;
                    hits += usize::from(inside(sx, sy));
                }
            }
            let coverage = hits as f64 / (N * N) as f64;
            if coverage > 0.0 && coverage < 1.0 {
                let got = f64::from(srgb_to_linear(image.pixel(x, y)[0]));
                sum += (got - coverage).powi(2);
                count += 1;
            }
        }
    }
    ((sum / count.max(1) as f64).sqrt(), count)
}

/// Done-when 4: MSAA 4 reduces edge aliasing on a tilted quad (and FXAA does at one sample).
#[test]
fn gpu_msaa4_and_fxaa_reduce_staircase_aliasing_on_a_tilted_quad() {
    let Some(gpu) = gpu() else {
        return;
    };
    let size = [96, 96];
    let cam = front_ortho(96.0);
    // Facing the camera, rotated 7° about the view axis; Flat lighting of white is 1.0.
    let rotation =
        Quat::from_rotation_y(-7f64.to_radians()) * Quat::from_rotation_arc(Vec3::Z, Vec3::NEG_Y);
    let transform = Transform::from_rotation_translation(rotation, Vec3::ZERO);
    let corners_px = [[-25.0, -25.0], [25.0, -25.0], [25.0, 25.0], [-25.0, 25.0]].map(|[x, y]| {
        let p = cam
            .project(transform.transform_point3(Point3::new(x, y, 0.0)), size)
            .unwrap();
        [p[0], p[1]]
    });
    let render = |samples: u32, fxaa: bool| {
        let mut r = new_renderer(
            &gpu,
            &RendererConfig {
                msaa_samples: samples,
                ..Default::default()
            },
        );
        let mesh = r
            .scene_mut()
            .insert_mesh(upload(&gpu, &square(50.0, true, true)));
        let mut inst = Instance::new(mesh, transform);
        inst.material.base_color = [1.0, 1.0, 1.0, 1.0];
        r.scene_mut().add_instance(inst);
        let style = ViewStyle {
            fxaa,
            ..solid(Lighting::Flat)
        };
        let view = add_view(&mut r, cam, size, style);
        capture(&mut r, view)
    };
    let one = render(1, false);
    let four = render(4, false);
    let fxaa = render(1, true);
    assert_eq!(one.pixel(48, 48), [255, 255, 255, 255], "the quad is white");
    let (e1, n) = staircase_error(&one, &corners_px);
    let (e4, _) = staircase_error(&four, &corners_px);
    let (ef, _) = staircase_error(&fxaa, &corners_px);
    eprintln!(
        "staircase RMS coverage error over {n} edge pixels: 1x {e1:.4}, 4x {e4:.4}, 1x+FXAA {ef:.4}"
    );
    assert!(n > 150, "{n} edge pixels");
    assert!(e1 > 0.2, "one sample is binary: {e1}");
    assert!(e4 < 0.5 * e1, "4x MSAA {e4} vs 1x {e1}");
    assert!(ef < 0.9 * e1, "FXAA {ef} vs 1x {e1}");
}

#[test]
fn gpu_gradient_background_runs_from_top_to_bottom() {
    let Some(gpu) = gpu() else {
        return;
    };
    let top = [0.8, 0.2, 0.05];
    let bottom = [0.05, 0.2, 0.8];
    let mut r = new_renderer(&gpu, &RendererConfig::default());
    let style = ViewStyle {
        background: Background::Gradient { top, bottom },
        ..Default::default()
    };
    let view = add_view(&mut r, front_ortho(10.0), [32, 64], style);
    let image = capture(&mut r, view);
    let want = |c: [f32; 3]| [srgb_byte(c[0]), srgb_byte(c[1]), srgb_byte(c[2]), 255];
    // Row centres sit at t = (y + 0.5) / 64.
    let lerp = |t: f32| [0, 1, 2].map(|k| top[k] + (bottom[k] - top[k]) * t);
    for y in [0u32, 21, 40, 63] {
        let t = (y as f32 + 0.5) / 64.0;
        let got = image.pixel(16, y);
        assert!(
            near(got, want(lerp(t)), 1),
            "row {y}: {got:?} vs {:?}",
            want(lerp(t))
        );
    }
    let reds: Vec<u8> = (0..64).map(|y| image.pixel(5, y)[0]).collect();
    assert!(
        reds.windows(2).all(|w| w[0] >= w[1]),
        "red falls monotonically: {reds:?}"
    );
}

fn f16_to_f32(bits: u16) -> f32 {
    let sign = if bits & 0x8000 != 0 { -1.0 } else { 1.0 };
    let exp = i32::from((bits >> 10) & 0x1f);
    let mant = f32::from(bits & 0x3ff);
    match exp {
        0 => sign * mant * 2f32.powi(-24),
        31 => f32::NAN,
        _ => sign * (1.0 + mant / 1024.0) * 2f32.powi(exp - 15),
    }
}

/// Reads the one-sample normal target of `view` (after an encode) as decoded unit vectors.
fn read_normals(r: &Renderer, view: ViewId) -> (Vec<[f32; 2]>, [u32; 2]) {
    let gpu = Arc::clone(r.gpu());
    let texture = r.normal_texture(view).unwrap();
    let (w, h) = (texture.width(), texture.height());
    let padded = (w * 4).next_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
    let bytes = u64::from(padded) * u64::from(h);
    let buffer = gpu.device().create_buffer(&wgpu::BufferDescriptor {
        label: Some("normal readback"),
        size: bytes,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let mut encoder = gpu.device().create_command_encoder(&Default::default());
    encoder.copy_texture_to_buffer(
        texture.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded),
                rows_per_image: Some(h),
            },
        },
        wgpu::Extent3d {
            width: w,
            height: h,
            depth_or_array_layers: 1,
        },
    );
    gpu.queue().submit([encoder.finish()]);
    let raw = gpu.readback::<u8>(&buffer, 0..bytes).wait().unwrap();
    let mut out = Vec::with_capacity((w * h) as usize);
    for row in raw.chunks_exact(padded as usize) {
        for px in row[..(w * 4) as usize].chunks_exact(4) {
            let x = f16_to_f32(u16::from_le_bytes([px[0], px[1]]));
            let y = f16_to_f32(u16::from_le_bytes([px[2], px[3]]));
            out.push([x, y]);
        }
    }
    (out, [w, h])
}

#[test]
fn gpu_normal_target_holds_octahedral_view_normals() {
    let Some(gpu) = gpu() else {
        return;
    };
    let size = [64, 64];
    let n = Vec3::new(0.3, -0.5, 0.812_403_8).normalize();
    for with_normals in [true, false] {
        let mut r = new_renderer(
            &gpu,
            &RendererConfig {
                msaa_samples: 1,
                ..Default::default()
            },
        );
        let mesh = r
            .scene_mut()
            .insert_mesh(upload(&gpu, &square(30.0, with_normals, true)));
        let rotation = Quat::from_rotation_arc(Vec3::Z, world_of_view(n));
        r.scene_mut().add_instance(Instance::new(
            mesh,
            Transform::from_rotation_translation(rotation, Vec3::ZERO),
        ));
        let view = add_view(&mut r, front_ortho(64.0), size, solid(Lighting::Studio));
        capture(&mut r, view);
        let (normals, [w, _]) = read_normals(&r, view);
        let at = |x: u32, y: u32| normals[(y * w + x) as usize];
        let decoded = Vec3::from_array(oct_decode(at(32, 32)).map(f64::from));
        assert!(
            decoded.angle_between(n) < 2e-3,
            "normals={with_normals}: decoded {decoded:?} vs {n:?}"
        );
        assert_eq!(at(0, 0), [0.0, 0.0], "nothing drawn: cleared to zero");
    }
}

/// Lighting and sample-count variants are built on the job system; until they are ready,
/// frames fall back to the core Studio pipelines at the previous sample count.
#[test]
fn gpu_variants_are_built_on_jobs_and_frames_fall_back_until_ready() {
    let Some(gpu) = gpu() else {
        return;
    };
    let jobs = JobSystem::new(JobSystemConfig {
        interactive_threads: std::num::NonZeroUsize::new(1),
        ..Default::default()
    })
    .unwrap();
    // Occupy the only Interactive thread so variant builds queue behind it.
    let (release, blocked) = std::sync::mpsc::channel::<()>();
    let blocker = jobs.spawn_fn("blocker", Default::default(), move |_ctx| {
        let _ = blocked.recv();
        Ok::<(), Cancelled>(())
    });
    let cfg = RendererConfig {
        jobs: Some(jobs.clone()),
        ..Default::default()
    };
    let sphere = amg_testkit::shapes::icosphere(Point3::ZERO, 40.0, 3);
    let data = MeshData {
        positions: sphere
            .positions
            .iter()
            .map(|p| p.as_vec3().to_array())
            .collect(),
        normals: sphere
            .positions
            .iter()
            .map(|p| p.normalize().as_vec3().to_array())
            .collect(),
        indices: sphere.triangles.iter().flatten().copied().collect(),
        closed: true,
        ..Default::default()
    };
    let cam = front_ortho(100.0);
    let mut r = new_renderer(&gpu, &cfg);
    let mesh = r.scene_mut().insert_mesh(upload(&gpu, &data));
    r.scene_mut()
        .add_instance(Instance::new(mesh, Transform::IDENTITY));
    let view = add_view(&mut r, cam, [96, 96], solid(Lighting::Studio));
    let plain = |r: &mut Renderer| r.capture(view).unwrap().wait().unwrap();
    let studio4 = plain(&mut r);

    r.view_mut(view).unwrap().style.lighting = Lighting::Matcap;
    r.set_msaa_samples(1).unwrap();
    let fallback = plain(&mut r);
    assert_eq!(
        fallback, studio4,
        "Matcap and 1x are not built yet: Studio at 4x"
    );
    assert_eq!(r.active_msaa_samples(), 4);
    assert_eq!(r.msaa_samples(), 1);
    assert!(r.pending_variants() > 0);
    assert!(r.variant_timings().is_empty());

    release.send(()).unwrap();
    blocker.wait_blocking().unwrap();
    assert!(r.prepare_view(view));
    assert_eq!(r.active_msaa_samples(), 1);
    let matcap1 = plain(&mut r);
    assert_ne!(matcap1, studio4);
    let built: Vec<_> = r
        .variant_timings()
        .iter()
        .map(|t| t.variant.clone())
        .collect();
    assert!(
        built
            .iter()
            .any(|v| v.contains("Matcap") && v.ends_with("x1")),
        "{built:?}"
    );
    assert!(
        r.core_pipeline_timings()
            .iter()
            .all(|t| !t.variant.contains("Matcap")),
        "Matcap is not a core pipeline"
    );

    // The same image as a renderer that had everything ready.
    let mut reference = new_renderer(
        &gpu,
        &RendererConfig {
            msaa_samples: 1,
            ..Default::default()
        },
    );
    let mesh = reference.scene_mut().insert_mesh(upload(&gpu, &data));
    reference
        .scene_mut()
        .add_instance(Instance::new(mesh, Transform::IDENTITY));
    let rv = add_view(&mut reference, cam, [96, 96], solid(Lighting::Matcap));
    assert_eq!(capture(&mut reference, rv), matcap1);
}

#[test]
fn gpu_material_change_rewrites_one_record_and_changes_shading() {
    let Some(gpu) = gpu() else {
        return;
    };
    let mut r = new_renderer(&gpu, &RendererConfig::default());
    let mesh = r
        .scene_mut()
        .insert_mesh(upload(&gpu, &square(40.0, true, true)));
    let rotation = Quat::from_rotation_arc(
        Vec3::Z,
        world_of_view(Vec3::new(-0.3, 0.4, 0.866).normalize()),
    );
    let key = r.scene_mut().add_instance(Instance::new(
        mesh,
        Transform::from_rotation_translation(rotation, Vec3::ZERO),
    ));
    let view = add_view(&mut r, front_ortho(60.0), [64, 64], solid(Lighting::Studio));
    let dielectric = capture(&mut r, view);
    let before = r.instance_bytes_uploaded();
    let m = &mut r.scene_mut().instance_mut(key).unwrap().material;
    m.metallic = 1.0;
    m.roughness = 0.2;
    let metal = capture(&mut r, view);
    assert_eq!(r.instance_bytes_uploaded() - before, 128);
    assert!(!near(metal.pixel(32, 32), dielectric.pixel(32, 32), 4));
}

/// Open meshes are drawn double-sided (back faces lit from the visible side); closed meshes
/// cull back faces.
#[test]
fn gpu_open_meshes_are_double_sided_and_closed_ones_cull_back_faces() {
    let Some(gpu) = gpu() else {
        return;
    };
    // Normal +Y (away from the camera on −Y): the camera sees the back face.
    let away =
        Transform::from_rotation_translation(Quat::from_rotation_arc(Vec3::Z, Vec3::Y), Vec3::ZERO);
    let material = Material::default();
    let back_lit = shade_reference(
        Lighting::Flat,
        &material,
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
    );
    let back_lit = [
        srgb_byte(back_lit[0]),
        srgb_byte(back_lit[1]),
        srgb_byte(back_lit[2]),
        255,
    ];
    for (closed, normals) in [(false, true), (false, false), (true, true)] {
        let mut r = new_renderer(&gpu, &RendererConfig::default());
        let mesh = r
            .scene_mut()
            .insert_mesh(upload(&gpu, &square(30.0, normals, closed)));
        r.scene_mut().add_instance(Instance::new(mesh, away));
        let view = add_view(&mut r, front_ortho(60.0), [64, 64], solid(Lighting::Flat));
        let got = capture(&mut r, view).pixel(32, 32);
        if closed {
            assert_eq!(got, [0, 0, 0, 255], "closed: the back face is culled");
        } else {
            assert!(
                near(got, back_lit, 2),
                "open, normals={normals}: {got:?} vs {back_lit:?}"
            );
        }
    }
}

fn overlapping_scene(gpu: &Arc<Gpu>, cfg: &RendererConfig) -> (Renderer, ViewId) {
    let mut r = new_renderer(gpu, cfg);
    let sphere = amg_testkit::shapes::icosphere(Point3::ZERO, 30.0, 4);
    let sphere = MeshData {
        positions: sphere
            .positions
            .iter()
            .map(|p| p.as_vec3().to_array())
            .collect(),
        normals: sphere
            .positions
            .iter()
            .map(|p| p.normalize().as_vec3().to_array())
            .collect(),
        indices: sphere.triangles.iter().flatten().copied().collect(),
        closed: true,
        ..Default::default()
    };
    let cube = amg_testkit::shapes::cube(Point3::splat(-20.0), 40.0);
    let cube = MeshData {
        positions: cube
            .positions
            .iter()
            .map(|p| p.as_vec3().to_array())
            .collect(),
        indices: cube.triangles.iter().flatten().copied().collect(),
        closed: true,
        ..Default::default()
    };
    let s = r.scene_mut().insert_mesh(upload(gpu, &sphere));
    let c = r.scene_mut().insert_mesh(upload(gpu, &cube));
    for (k, x) in [(s, -12.0), (c, 15.0)] {
        r.scene_mut().add_instance(Instance::new(
            k,
            Transform::from_rotation_translation(
                Quat::from_rotation_z(0.4),
                Vec3::new(x, 0.0, 0.0),
            ),
        ));
    }
    let cam = Camera {
        eye: Point3::new(60.0, -160.0, 70.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.7 },
    };
    let view = add_view(&mut r, cam, [128, 96], ViewStyle::default());
    (r, view)
}

/// Done-when 2 (correctness half; the bench measures the cost): the depth pre-pass draws the
/// same image as the single opaque pass.
#[test]
fn gpu_depth_prepass_renders_like_the_single_pass() {
    let Some(gpu) = gpu() else {
        return;
    };
    let (mut plain, v) = overlapping_scene(&gpu, &RendererConfig::default());
    let (mut pre, w) = overlapping_scene(
        &gpu,
        &RendererConfig {
            depth_prepass: true,
            ..Default::default()
        },
    );
    assert!(pre.depth_prepass_active() && !plain.depth_prepass_active());
    let a = capture(&mut plain, v);
    let b = capture(&mut pre, w);
    let worst = a
        .pixels
        .iter()
        .zip(&b.pixels)
        .map(|(x, y)| x.abs_diff(*y))
        .max()
        .unwrap();
    assert!(worst <= 1, "worst channel difference {worst}");
    // Switching at run time builds the pre-pass pipelines on demand.
    plain.set_depth_prepass(true);
    assert!(plain.prepare_view(v));
    assert!(plain.depth_prepass_active());
    let c = capture(&mut plain, v);
    assert_eq!(c, b);
}

#[test]
fn gpu_timings_name_every_pass() {
    let Some(gpu) = gpu() else {
        return;
    };
    if !gpu.has(wgpu::Features::TIMESTAMP_QUERY) {
        eprintln!("no TIMESTAMP_QUERY: skipped");
        return;
    }
    for prepass in [false, true] {
        let (mut r, view) = overlapping_scene(
            &gpu,
            &RendererConfig {
                depth_prepass: prepass,
                ..Default::default()
            },
        );
        for _ in 0..3 {
            capture(&mut r, view);
        }
        let _ = gpu.device().poll(wgpu::PollType::wait_indefinitely());
        let frames = r.drain_gpu_timings(view);
        assert!(!frames.is_empty(), "prepass={prepass}: no timed frame");
        let names: Vec<_> = frames[0].passes.iter().map(|p| p.0).collect();
        let want: &[&str] = if prepass {
            &["depth prepass", "opaque", "final"]
        } else {
            &["opaque", "final"]
        };
        assert_eq!(names, want);
        let t = &frames[0];
        let sum: f32 = t.passes.iter().map(|p| p.1).sum();
        assert!(t.total_ms >= 0.0 && sum <= t.total_ms + 1e-3, "{t:?}");
        assert!(r.gpu_timings(view).is_some());
    }
}
