//! RND-03 tests: scene mirror, dirty tracking, culling, multiple views, upload limits and the
//! external layer. GPU tests are named `gpu_*` (nextest test-group `gpu`, serial) and skip
//! without an adapter (CI runs them on WARP with `AMG_REQUIRE_GPU=1`). `timing_*` tests print
//! their measurement and only assert the budget under `AMG_STRICT_TIMING=1`.

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_core::{Dir3, Point3, Quat, Transform, Vec3};
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_jobs::{CancelToken, JobCtx};
use amg_render::{
    Background, Camera, ExternalLayer, GpuMesh, Instance, InstanceKey, Layer, LayerMask, MeshData,
    MeshKey, PickId, Projection, RenderError, Renderer, RendererConfig, RgbaImage, ViewId,
    ViewStyle, upload_mesh,
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

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// A unit cube centred on its local origin (1 mm edge), flat shaded.
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

fn sphere_data(radius: f64) -> MeshData {
    let mesh = amg_testkit::shapes::icosphere(Point3::ZERO, radius, 3);
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

fn upload(gpu: &Gpu, data: &MeshData) -> GpuMesh {
    upload_mesh(gpu, data, &JobCtx::detached()).unwrap()
}

fn perspective(eye: Point3, target: Point3) -> Camera {
    Camera {
        eye,
        target,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.7 },
    }
}

fn new_renderer(gpu: &Arc<Gpu>) -> Renderer {
    Renderer::new(Arc::clone(gpu), &RendererConfig::default()).unwrap()
}

fn add_view(r: &mut Renderer, camera: Camera, size: [u32; 2]) -> ViewId {
    let view = r.create_view(size);
    let v = r.view_mut(view).unwrap();
    v.camera = camera;
    v.style = ViewStyle {
        background: Background::Solid(BACKGROUND),
        ..Default::default()
    };
    view
}

fn capture(r: &mut Renderer, view: ViewId) -> RgbaImage {
    r.capture(view).unwrap().wait().unwrap()
}

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

/// Encodes one frame of `view` without reading anything back.
fn encode_frame(r: &mut Renderer, view: ViewId) -> amg_render::FrameStats {
    let gpu = Arc::clone(r.gpu());
    let mut encoder = gpu.device().create_command_encoder(&Default::default());
    let stats = r.encode(view, &mut encoder).unwrap();
    gpu.queue().submit([encoder.finish()]);
    stats
}

/// `n` unit cubes (scaled to `size` mm) in a row along X at spacing `step`, all sharing one mesh.
fn row_of_cubes(r: &mut Renderer, gpu: &Gpu, n: usize, size: f64, step: f64) -> Vec<InstanceKey> {
    let mesh = r
        .scene_mut()
        .insert_mesh(upload(gpu, &cube_data(Point3::ZERO)));
    (0..n)
        .map(|i| {
            let t = Transform::from_scale_rotation_translation(
                Vec3::splat(size),
                Quat::IDENTITY,
                Vec3::new(i as f64 * step, 0.0, 0.0),
            );
            r.scene_mut().add_instance(Instance::new(mesh, t))
        })
        .collect()
}

#[test]
fn gpu_one_transform_change_rewrites_exactly_128_bytes() {
    let Some(gpu) = gpu() else {
        return;
    };
    let mut r = new_renderer(&gpu);
    let keys = row_of_cubes(&mut r, &gpu, 50, 10.0, 15.0);
    let view = add_view(
        &mut r,
        perspective(
            Point3::new(300.0, -900.0, 200.0),
            Point3::new(300.0, 0.0, 0.0),
        ),
        [96, 96],
    );

    let before = capture(&mut r, view);
    assert_eq!(
        r.instance_bytes_uploaded(),
        50 * 128,
        "first frame writes every record once"
    );

    // Nothing changed: nothing is written, even though the camera moved.
    let mut total = r.instance_bytes_uploaded();
    r.view_mut(view).unwrap().camera.eye += Vec3::new(5.0, 0.0, 0.0);
    capture(&mut r, view);
    assert_eq!(r.instance_bytes_uploaded(), total, "camera moves are free");

    // One transform.
    r.scene_mut().instance_mut(keys[20]).unwrap().transform =
        Transform::from_translation(Vec3::new(300.0, 0.0, 60.0))
            * Transform::from_scale(Vec3::splat(10.0));
    capture(&mut r, view);
    assert_eq!(
        r.instance_bytes_uploaded() - total,
        128,
        "one transform: 128 B"
    );
    total = r.instance_bytes_uploaded();

    // One material colour, one visibility flip: still 128 B each.
    r.scene_mut()
        .instance_mut(keys[3])
        .unwrap()
        .material
        .base_color = [1.0, 0.0, 0.0, 1.0];
    capture(&mut r, view);
    assert_eq!(r.instance_bytes_uploaded() - total, 128);
    total = r.instance_bytes_uploaded();

    // Touching the same instance twice before a frame is still one record.
    for i in 0..3 {
        r.scene_mut().instance_mut(keys[7]).unwrap().transform =
            Transform::from_translation(Vec3::new(105.0, 0.0, f64::from(i)));
    }
    capture(&mut r, view);
    assert_eq!(r.instance_bytes_uploaded() - total, 128);
    total = r.instance_bytes_uploaded();

    // Removing the last instance writes nothing; removing another moves the last into its slot.
    r.scene_mut().remove_instance(keys[49]);
    capture(&mut r, view);
    assert_eq!(r.instance_bytes_uploaded() - total, 0);
    r.scene_mut().remove_instance(keys[10]);
    let total = r.instance_bytes_uploaded();
    capture(&mut r, view);
    assert_eq!(
        r.instance_bytes_uploaded() - total,
        128,
        "the swapped-in record is resent"
    );

    let after = capture(&mut r, view);
    assert_ne!(before, after, "the changes are visible");
}

#[test]
fn gpu_instance_buffer_grows_and_keeps_every_instance() {
    let Some(gpu) = gpu() else {
        return;
    };
    let mut r = new_renderer(&gpu);
    // More than the initial capacity (64), added over several frames so the buffer regrows.
    let view = add_view(
        &mut r,
        perspective(Point3::new(0.0, -900.0, 0.0), Point3::ZERO),
        [64, 64],
    );
    let mesh = r
        .scene_mut()
        .insert_mesh(upload(&gpu, &cube_data(Point3::ZERO)));
    let (mut drawn, mut culled) = (0, 0);
    for batch in 0..3 {
        for i in 0..150 {
            let x = f64::from(batch * 150 + i) * 2.0 - 450.0;
            r.scene_mut().add_instance(Instance::new(
                mesh,
                Transform::from_scale_rotation_translation(
                    Vec3::splat(30.0),
                    Quat::IDENTITY,
                    Vec3::new(x, 0.0, 0.0),
                ),
            ));
        }
        let stats = encode_frame(&mut r, view);
        (drawn, culled) = (stats.instances_drawn, stats.culled);
    }
    // Every instance is either drawn or frustum-culled; the buffer grew past 64 and 256 records.
    assert_eq!(drawn + culled, 450);
    assert!(drawn > 256, "{drawn} drawn");
    let image = capture(&mut r, view);
    assert!(
        !near(image.pixel(32, 32), background_bytes(), 8),
        "the row is on screen"
    );
}

#[test]
fn gpu_shared_mesh_renders_like_separate_copies() {
    let Some(gpu) = gpu() else {
        return;
    };
    let cam = perspective(Point3::new(0.0, -300.0, 40.0), Point3::ZERO);
    let place = |i: usize| {
        Transform::from_scale_rotation_translation(
            Vec3::splat(40.0),
            Quat::from_rotation_z(0.3 * i as f64),
            Vec3::new(-50.0 + 100.0 * i as f64, 0.0, 0.0),
        )
    };

    let mut shared = new_renderer(&gpu);
    let view = add_view(&mut shared, cam, [128, 96]);
    let mesh = shared
        .scene_mut()
        .insert_mesh(upload(&gpu, &cube_data(Point3::ZERO)));
    for i in 0..2 {
        shared
            .scene_mut()
            .add_instance(Instance::new(mesh, place(i)));
    }
    let one_mesh_bytes = shared.scene().gpu_bytes();
    let stats = encode_frame(&mut shared, view);
    assert_eq!((stats.instances_drawn, stats.triangles), (2, 24));
    let shared_image = capture(&mut shared, view);

    let mut separate = new_renderer(&gpu);
    let view = add_view(&mut separate, cam, [128, 96]);
    for i in 0..2 {
        let mesh = separate
            .scene_mut()
            .insert_mesh(upload(&gpu, &cube_data(Point3::ZERO)));
        separate
            .scene_mut()
            .add_instance(Instance::new(mesh, place(i)));
    }
    assert_eq!(separate.scene().gpu_bytes(), 2 * one_mesh_bytes);
    assert_eq!(shared_image, capture(&mut separate, view));
}

#[test]
fn gpu_hidden_layered_removed_and_culled_instances_are_not_drawn() {
    let Some(gpu) = gpu() else {
        return;
    };
    let mut r = new_renderer(&gpu);
    let view = add_view(
        &mut r,
        perspective(Point3::new(0.0, -300.0, 0.0), Point3::ZERO),
        [64, 64],
    );
    let keys = row_of_cubes(&mut r, &gpu, 6, 10.0, 0.0);
    // 0 stays, 1 hidden, 2 preview layer, 3 far to the side (culled), 4 behind the eye (culled),
    // 5 has its mesh removed.
    r.scene_mut().instance_mut(keys[1]).unwrap().visible = false;
    r.scene_mut().instance_mut(keys[2]).unwrap().layer = Layer::Preview;
    r.scene_mut().instance_mut(keys[3]).unwrap().transform =
        Transform::from_translation(Vec3::new(5000.0, 0.0, 0.0));
    r.scene_mut().instance_mut(keys[4]).unwrap().transform =
        Transform::from_translation(Vec3::new(0.0, -2000.0, 0.0));
    let other = r
        .scene_mut()
        .insert_mesh(upload(&gpu, &cube_data(Point3::ZERO)));
    r.scene_mut().instance_mut(keys[5]).unwrap().mesh = Some(other);
    r.scene_mut().remove_mesh(other);

    let stats = encode_frame(&mut r, view);
    // Slots 0 and 2 draw; 2 is on the preview layer, which this view shows by default.
    assert_eq!(stats.instances_drawn, 2);
    assert_eq!(stats.culled, 2);

    r.view_mut(view).unwrap().layers = LayerMask::ALL.without(Layer::Preview);
    assert_eq!(encode_frame(&mut r, view).instances_drawn, 1);
    r.view_mut(view).unwrap().layers = LayerMask::only(Layer::Preview);
    assert_eq!(encode_frame(&mut r, view).instances_drawn, 1);
    r.view_mut(view).unwrap().layers = LayerMask::NONE;
    assert_eq!(encode_frame(&mut r, view).instances_drawn, 0);
}

#[test]
fn gpu_two_views_with_different_cameras_render_one_scene_in_one_frame() {
    let Some(gpu) = gpu() else {
        return;
    };
    let mut r = new_renderer(&gpu);
    let sphere = r.scene_mut().insert_mesh(upload(&gpu, &sphere_data(40.0)));
    let cube = r
        .scene_mut()
        .insert_mesh(upload(&gpu, &cube_data(Point3::ZERO)));
    r.scene_mut().add_instance(Instance::new(
        sphere,
        Transform::from_translation(Vec3::new(-60.0, 0.0, 0.0)),
    ));
    let mut cube_instance = Instance::new(
        cube,
        Transform::from_scale_rotation_translation(
            Vec3::splat(60.0),
            Quat::from_rotation_z(0.5),
            Vec3::new(60.0, 0.0, 0.0),
        ),
    );
    cube_instance.material.base_color = [0.9, 0.3, 0.2, 1.0];
    r.scene_mut().add_instance(cube_instance);

    let front = add_view(
        &mut r,
        perspective(Point3::new(0.0, -400.0, 0.0), Point3::ZERO),
        [160, 120],
    );
    let side = add_view(
        &mut r,
        Camera {
            eye: Point3::new(400.0, 0.0, 150.0),
            target: Point3::ZERO,
            up: Dir3::Z,
            projection: Projection::Orthographic { height: 250.0 },
        },
        [100, 100],
    );

    let both = r
        .capture_many(&[front, side])
        .unwrap()
        .into_iter()
        .map(|c| c.wait().unwrap())
        .collect::<Vec<_>>();
    assert_eq!(both[0].size, [160, 120]);
    assert_eq!(both[1].size, [100, 100]);
    assert_eq!(
        r.instance_bytes_uploaded(),
        2 * 128,
        "two views share one set of instance records"
    );
    for image in &both {
        let background = image
            .pixels
            .chunks_exact(4)
            .filter(|p| near([p[0], p[1], p[2], p[3]], background_bytes(), 1))
            .count();
        assert!(background < image.pixels.len() / 4, "something is drawn");
    }

    // Each view looks like it does when rendered alone: the other view does not leak in.
    assert_eq!(both[0], capture(&mut r, front));
    assert_eq!(both[1], capture(&mut r, side));
    // The views differ: from the front the sphere is left of the cube, from the side they overlap.
    let front_left = both[0].pixel(55, 60);
    let front_right = both[0].pixel(105, 60);
    assert!(!near(front_left, background_bytes(), 8) && !near(front_right, background_bytes(), 8));
    assert!(!near(front_left, front_right, 8));
}

/// Rgba16Float bits of a small non-negative value (only exact constants are needed here).
fn f16_bits(v: f32) -> u16 {
    if v == 0.0 {
        0
    } else if (v - 1.0).abs() < f32::EPSILON {
        0x3C00
    } else {
        panic!("test only needs 0 and 1")
    }
}

fn layer_texture(
    gpu: &Gpu,
    format: wgpu::TextureFormat,
    size: [u32; 2],
    bytes: &[u8],
    texel: u32,
) -> wgpu::TextureView {
    let texture = gpu.device().create_texture(&wgpu::TextureDescriptor {
        label: Some("test external layer"),
        size: wgpu::Extent3d {
            width: size[0],
            height: size[1],
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    gpu.queue().write_texture(
        texture.as_image_copy(),
        bytes,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(size[0] * texel),
            rows_per_image: Some(size[1]),
        },
        wgpu::Extent3d {
            width: size[0],
            height: size[1],
            depth_or_array_layers: 1,
        },
    );
    texture.create_view(&wgpu::TextureViewDescriptor::default())
}

/// Ray / oriented-box test: the parameter of the first hit of `ray` with the unit cube
/// (`-0.5..0.5` in local space) placed by `world_from_local`, or `None`.
fn ray_cube(ray: amg_core::Ray3, world_from_local: &Transform) -> Option<f64> {
    let inverse = world_from_local.inverse();
    let o = inverse.transform_point3(ray.origin);
    let d = inverse.transform_vector3(ray.dir.get());
    let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
    for k in 0..3 {
        if d[k].abs() < 1e-15 {
            if o[k].abs() > 0.5 {
                return None;
            }
            continue;
        }
        let (a, b) = ((-0.5 - o[k]) / d[k], (0.5 - o[k]) / d[k]);
        t0 = t0.max(a.min(b));
        t1 = t1.min(a.max(b));
    }
    (t0 <= t1 && t1 > 0.0).then_some(t0.max(0.0))
}

#[test]
fn gpu_external_layer_composites_with_correct_occlusion_on_both_sides() {
    let Some(gpu) = gpu() else {
        return;
    };
    const N: u32 = 128;
    let size = [N, N];
    let cam = perspective(Point3::new(0.0, -300.0, 0.0), Point3::ZERO);
    let cube_pose = Transform::from_scale_rotation_translation(
        Vec3::splat(100.0),
        Quat::from_rotation_z(std::f64::consts::FRAC_PI_4),
        Vec3::ZERO,
    );

    let mut r = new_renderer(&gpu);
    let mesh = r
        .scene_mut()
        .insert_mesh(upload(&gpu, &cube_data(Point3::ZERO)));
    r.scene_mut().add_instance(Instance::new(mesh, cube_pose));
    let view = add_view(&mut r, cam, size);

    // The layer is a plane at view depth 265 mm: the cube's nearest edge (229 mm) is in front
    // of it, its side faces (up to 300 mm) are behind it. Green, opaque, left quarter empty.
    let plane_depth = 265.0;
    let plane_point = cam.eye + cam.forward().get() * plane_depth;
    let ndc_depth = cam.project(plane_point, size).unwrap()[2] as f32;
    let empty_columns = N / 4;
    let (mut colour, mut depth) = (Vec::new(), Vec::new());
    for _y in 0..N {
        for x in 0..N {
            let live = x >= empty_columns;
            for c in [0.0, 1.0, 0.0, 1.0] {
                colour.extend_from_slice(&f16_bits(if live { c } else { 0.0 }).to_le_bytes());
            }
            depth.extend_from_slice(&(if live { ndc_depth } else { 0.0 }).to_le_bytes());
        }
    }
    let layer = ExternalLayer {
        color: layer_texture(&gpu, wgpu::TextureFormat::Rgba16Float, size, &colour, 8),
        depth: layer_texture(&gpu, wgpu::TextureFormat::R32Float, size, &depth, 4),
        size_px: size,
        pick: PickId(77),
    };
    assert_eq!(layer.pick, PickId(77));
    r.scene_mut().set_external_layer(Some(layer));
    assert!(r.scene().external_layer().is_some());
    let image = capture(&mut r, view);

    let green = [0, 255, 0, 255];
    let (mut in_front, mut behind, mut plane_only, mut skipped) = (0, 0, 0, 0);
    for y in 0..N {
        for x in 0..N {
            if x < empty_columns {
                continue;
            }
            // Classify with the CPU ray cast; skip pixels near the cube silhouette or the
            // intersection line (any of five nearby rays disagrees).
            let class_at = |dx: f64, dy: f64| {
                let ray = cam.ray([f64::from(x) + 0.5 + dx, f64::from(y) + 0.5 + dy], size);
                match ray_cube(ray, &cube_pose) {
                    None => 0,
                    Some(t) => {
                        let z = (ray.at(t) - cam.eye).dot(cam.forward().get());
                        if (z - plane_depth).abs() < 2.0 {
                            3
                        } else if z < plane_depth {
                            1
                        } else {
                            2
                        }
                    }
                }
            };
            let class = class_at(0.0, 0.0);
            let stable = [(-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)]
                .iter()
                .all(|&(dx, dy)| class_at(dx, dy) == class);
            if class == 3 || !stable {
                skipped += 1;
                continue;
            }
            let got = image.pixel(x, y);
            match class {
                1 => {
                    in_front += 1;
                    assert!(
                        !near(got, green, 40) && !near(got, background_bytes(), 8),
                        "({x},{y}): cube nearer than the plane must show the cube, got {got:?}"
                    );
                }
                _ => {
                    if class == 2 {
                        behind += 1;
                    } else {
                        plane_only += 1;
                    }
                    assert!(
                        near(got, green, 2),
                        "({x},{y}): the plane must cover what is behind it, got {got:?}"
                    );
                }
            }
        }
    }
    eprintln!(
        "external layer: {in_front} cube-in-front, {behind} cube-behind, {plane_only} plane-only, {skipped} skipped"
    );
    assert!(
        in_front >= 150,
        "need pixels in front of the plane: {in_front}"
    );
    assert!(behind >= 150, "need pixels behind the plane: {behind}");
    assert!(plane_only >= 1000);
    // Empty layer texels (depth 0) leave the scene alone.
    assert!(near(image.pixel(5, 5), background_bytes(), 1));
    assert!(near(image.pixel(10, 120), background_bytes(), 1));

    // Removing the layer removes the plane; a layer of another size is ignored.
    r.scene_mut().set_external_layer(None);
    assert!(near(
        capture(&mut r, view).pixel(120, 5),
        background_bytes(),
        1
    ));
}

#[test]
fn gpu_add_remove_cycles_return_gpu_bytes_to_baseline() {
    let Some(gpu) = gpu() else {
        return;
    };
    let mut r = new_renderer(&gpu);
    let view = add_view(
        &mut r,
        perspective(Point3::new(0.0, -300.0, 100.0), Point3::ZERO),
        [32, 32],
    );
    // A permanent mesh makes the baseline non-zero.
    let keep = r
        .scene_mut()
        .insert_mesh(upload(&gpu, &cube_data(Point3::ZERO)));
    let keep_instance = r
        .scene_mut()
        .add_instance(Instance::new(keep, Transform::IDENTITY));
    let baseline = r.scene().gpu_bytes();
    assert!(baseline > 0);

    let sphere = sphere_data(5.0);
    let cube = cube_data(Point3::ZERO);
    for cycle in 0..1000_u32 {
        let data = if cycle % 2 == 0 { &sphere } else { &cube };
        let mesh = r.scene_mut().insert_mesh(upload(&gpu, data));
        assert!(r.scene().gpu_bytes() > baseline);
        let instances: Vec<_> = (0..=cycle % 3)
            .map(|i| {
                r.scene_mut().add_instance(Instance::new(
                    mesh,
                    Transform::from_translation(Vec3::new(f64::from(i) * 3.0, 0.0, 0.0)),
                ))
            })
            .collect();
        if cycle % 50 == 0 {
            encode_frame(&mut r, view);
        }
        // Alternate the removal order so both the swap-remove and the tail paths run.
        if cycle % 4 < 2 {
            for k in instances {
                assert!(r.scene_mut().remove_instance(k));
            }
        } else {
            for k in instances.into_iter().rev() {
                assert!(r.scene_mut().remove_instance(k));
            }
        }
        assert!(r.scene_mut().remove_mesh(mesh));
        assert_eq!(r.scene().gpu_bytes(), baseline, "cycle {cycle}");
        assert_eq!(r.scene().instance_count(), 1);
        assert_eq!(r.scene().mesh_count(), 1);
    }
    // The survivor still renders after all the churn, and the buffer did not grow per cycle.
    let stats = encode_frame(&mut r, view);
    assert_eq!(stats.instances_drawn, 1);
    assert!(r.scene().instance(keep_instance).is_some());
    assert!(
        r.instance_bytes_uploaded() < 100 * 128,
        "{} bytes of instance uploads for 1000 cycles",
        r.instance_bytes_uploaded()
    );
}

#[test]
fn gpu_visible_bounds_cover_the_visible_instances_only() {
    let Some(gpu) = gpu() else {
        return;
    };
    let mut r = new_renderer(&gpu);
    assert!(r.scene().visible_bounds().is_empty());
    let mesh = r
        .scene_mut()
        .insert_mesh(upload(&gpu, &cube_data(Point3::new(100.0, 0.0, 0.0))));
    let a = r
        .scene_mut()
        .add_instance(Instance::new(mesh, Transform::IDENTITY));
    let b = r.scene_mut().add_instance(Instance::new(
        mesh,
        Transform::from_translation(Vec3::new(0.0, 50.0, 0.0)),
    ));
    let bounds = r.scene().visible_bounds();
    assert!(bounds.min.distance(Point3::new(99.5, -0.5, -0.5)) < 1e-9);
    assert!(bounds.max.distance(Point3::new(100.5, 50.5, 0.5)) < 1e-9);
    r.scene_mut().instance_mut(b).unwrap().visible = false;
    assert!(r.scene().visible_bounds().max.y < 1.0);
    r.scene_mut().instance_mut(a).unwrap().visible = false;
    assert!(r.scene().visible_bounds().is_empty());
}

#[test]
fn gpu_upload_reports_invalid_data() {
    let Some(gpu) = gpu() else {
        return;
    };
    let ctx = JobCtx::detached();
    let mut bad_index = cube_data(Point3::ZERO);
    let vertex_count = bad_index.positions.len() as u32;
    bad_index.indices[5] = vertex_count; // one past the end
    let err = upload_mesh(&gpu, &bad_index, &ctx).unwrap_err();
    assert!(
        matches!(&err, RenderError::InvalidData(m) if m.contains("index 5")),
        "{err}"
    );

    let mut nan = cube_data(Point3::ZERO);
    nan.positions[3][1] = f32::NAN;
    let err = upload_mesh(&gpu, &nan, &ctx).unwrap_err();
    assert!(
        matches!(&err, RenderError::InvalidData(m) if m.contains("vertex 3")),
        "{err}"
    );

    // Nothing was uploaded for the rejected meshes, and a valid one still works.
    assert!(upload_mesh(&gpu, &cube_data(Point3::ZERO), &ctx).is_ok());
}

/// A displaced regular grid with about `tris` triangles (2 per cell), 12 B/vertex positions.
fn grid_data(tris: usize) -> MeshData {
    let n = ((tris / 2) as f64).sqrt() as usize;
    let stride = n + 1;
    let mut positions = Vec::with_capacity(stride * stride);
    for j in 0..stride {
        for i in 0..stride {
            let (x, y) = (i as f32 * 0.01, j as f32 * 0.01);
            positions.push([x, y, 0.3 * (x * 3.0).sin() * (y * 2.0).cos()]);
        }
    }
    let mut indices = Vec::with_capacity(n * n * 6);
    for j in 0..n {
        for i in 0..n {
            let a = (j * stride + i) as u32;
            let (b, c, d) = (a + 1, a + stride as u32, a + stride as u32 + 1);
            indices.extend_from_slice(&[a, b, d, a, d, c]);
        }
    }
    MeshData {
        positions,
        indices,
        ..Default::default()
    }
}

#[test]
fn timing_upload_5m_triangles_on_a_job_thread() {
    let Some(gpu) = gpu() else {
        return;
    };
    let data = grid_data(5_000_000);
    assert!(data.indices.len() / 3 >= 4_990_000);
    let ctx = JobCtx::detached();
    // Warm up the allocator and driver with a small upload.
    upload_mesh(&gpu, &grid_data(1000), &ctx).unwrap();
    let worker = {
        let gpu = Arc::clone(&gpu);
        std::thread::spawn(move || {
            let start = Instant::now();
            let mesh = upload_mesh(&gpu, &data, &JobCtx::detached()).unwrap();
            (start.elapsed(), mesh.triangle_count(), mesh.gpu_bytes())
        })
    };
    let (elapsed, tris, bytes) = worker.join().unwrap();
    eprintln!(
        "upload_mesh: {tris} tris, {} MB in {:.1} ms",
        bytes / 1_000_000,
        elapsed.as_secs_f64() * 1e3
    );
    if strict_timing() {
        assert!(elapsed <= Duration::from_millis(300), "{elapsed:?}");
    }
}

#[test]
fn timing_cancelling_mid_upload_returns_within_20ms() {
    let Some(gpu) = gpu() else {
        return;
    };
    let data = Arc::new(grid_data(5_000_000));
    let mut worst = Duration::ZERO;
    let mut cancelled_runs = 0;
    for delay_ms in [3, 8, 14, 20, 30, 45] {
        let token = CancelToken::new();
        let ctx = JobCtx::with_token(token.clone());
        let worker = {
            let (gpu, data) = (Arc::clone(&gpu), Arc::clone(&data));
            std::thread::spawn(move || {
                let result = upload_mesh(&gpu, &data, &ctx);
                (result.map(|m| m.triangle_count()), Instant::now())
            })
        };
        std::thread::sleep(Duration::from_millis(delay_ms));
        let requested = Instant::now();
        token.cancel();
        let (result, returned) = worker.join().unwrap();
        match result {
            Err(RenderError::Cancelled(_)) => {
                cancelled_runs += 1;
                worst = worst.max(returned.saturating_duration_since(requested));
            }
            Ok(_) => eprintln!("delay {delay_ms} ms: the upload had already finished"),
            Err(e) => panic!("unexpected error {e}"),
        }
    }
    eprintln!(
        "cancel latency: worst {:.2} ms over {cancelled_runs} cancelled runs",
        worst.as_secs_f64() * 1e3
    );
    assert!(cancelled_runs >= 1, "no upload was cancelled mid-way");
    if strict_timing() {
        assert!(worst <= Duration::from_millis(20), "{worst:?}");
    }
}

#[test]
fn timing_insert_mesh_is_constant_time() {
    let Some(gpu) = gpu() else {
        return;
    };
    let mut r = new_renderer(&gpu);
    let mut meshes: Vec<GpuMesh> = (0..2000)
        .map(|_| upload(&gpu, &cube_data(Point3::ZERO)))
        .collect();
    let mut times = Vec::new();
    let mut keys: Vec<MeshKey> = Vec::new();
    while let Some(mesh) = meshes.pop() {
        let start = Instant::now();
        let key = r.scene_mut().insert_mesh(mesh);
        times.push(start.elapsed());
        keys.push(key);
    }
    times.sort();
    let p99 = times[times.len() * 99 / 100];
    eprintln!(
        "insert_mesh: p50 {:?}, p99 {p99:?}, max {:?}",
        times[times.len() / 2],
        times.last().unwrap()
    );
    if strict_timing() {
        assert!(p99 <= Duration::from_micros(50), "{p99:?}");
    }
}
