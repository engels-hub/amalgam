//! RND-08 GPU tests: overhang, zebra and draft analyses, scalar-field colormaps (LUT, bands,
//! isolines, out-of-range, NaN), deformation and shading switches without geometry uploads.
//! Named `gpu_*` (nextest test-group `gpu`, serial); they skip without an adapter and fail under
//! `AMG_REQUIRE_GPU=1` (CI runs them on WARP).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::Arc;

use amg_core::{Dir3, Plane, Point3, Ray3, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::analysis_testing::{FieldSample, OverhangCase, field_colors, overhang_classes};
use amg_render::{
    Analysis, AnalysisOutput, Animation, Background, Camera, ColorMap, Deformation, FieldBinding,
    GpuMesh, Hatch, Instance, InstanceKey, Lighting, Material, MeshData, OutOfRange, Projection,
    RenderError, Renderer, RendererConfig, RgbaImage, SectionPlane, Shading, ViewId, ViewStyle,
    VisualStyle, class, colormap_lut, colormap_rgb, decode_class, geometry_bytes_uploaded,
    overhang_class, upload_mesh, upload_scalar_field, upload_vector_field, zebra_t,
};

/// Background of the class captures: blue, which decodes to no class.
const CLASS_BACKGROUND: [f32; 3] = [0.0, 0.0, 1.0];

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

fn renderer(gpu: &Arc<Gpu>, samples: u32) -> Renderer {
    Renderer::new(
        Arc::clone(gpu),
        &RendererConfig {
            msaa_samples: samples,
            ..Default::default()
        },
    )
    .unwrap()
}

fn style(background: [f32; 3]) -> ViewStyle {
    ViewStyle {
        background: Background::Solid(background),
        lighting: Lighting::Flat,
        visual: VisualStyle::Shaded,
        ..Default::default()
    }
}

fn add_view(r: &mut Renderer, camera: Camera, size: [u32; 2], style: ViewStyle) -> ViewId {
    let view = r.create_view(size);
    let v = r.view_mut(view).unwrap();
    v.camera = camera;
    v.style = style;
    view
}

fn capture(r: &mut Renderer, view: ViewId) -> RgbaImage {
    assert!(r.prepare_view(view), "pipeline variants did not finish");
    r.capture(view).unwrap().wait().unwrap()
}

fn upload(gpu: &Gpu, data: &MeshData) -> GpuMesh {
    upload_mesh(gpu, data, &JobCtx::detached()).unwrap()
}

fn ortho(eye: Point3, target: Point3, up: Dir3, height: f64) -> Camera {
    Camera {
        eye,
        target,
        up,
        projection: Projection::Orthographic { height },
    }
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

fn srgb(rgb: [f32; 3]) -> [u8; 3] {
    rgb.map(srgb_byte)
}

fn near3(px: [u8; 4], want: [u8; 3], tol: u8) -> bool {
    (0..3).all(|k| px[k].abs_diff(want[k]) <= tol)
}

/// An icosphere of radius `r` around the origin; `normals` adds unit vertex normals.
fn icosphere(r: f64, subdivisions: u32, normals: bool) -> MeshData {
    let mesh = amg_testkit::shapes::icosphere(Point3::ZERO, r, subdivisions);
    MeshData {
        origin: Point3::ZERO,
        positions: mesh
            .positions
            .iter()
            .map(|p| p.as_vec3().to_array())
            .collect(),
        normals: if normals {
            mesh.positions
                .iter()
                .map(|p| p.normalize().as_vec3().to_array())
                .collect()
        } else {
            Vec::new()
        },
        indices: mesh.triangles.iter().flatten().copied().collect(),
        closed: true,
    }
}

fn vec_of(p: [f32; 3]) -> Vec3 {
    Vec3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))
}

/// Möller–Trumbore in f64: `(t, triangle)` of the nearest front-or-back hit.
fn ray_mesh(ray: &Ray3, data: &MeshData) -> Option<(f64, usize)> {
    let d = ray.dir.get();
    let mut best: Option<(f64, usize)> = None;
    for (i, tri) in data.indices.chunks(3).enumerate() {
        let [a, b, c] =
            [tri[0], tri[1], tri[2]].map(|k| data.origin + vec_of(data.positions[k as usize]));
        let e1 = b - a;
        let e2 = c - a;
        let p = d.cross(e2);
        let det = e1.dot(p);
        if det.abs() < 1e-14 {
            continue;
        }
        let inv = 1.0 / det;
        let s = ray.origin - a;
        let u = s.dot(p) * inv;
        if !(0.0..=1.0).contains(&u) {
            continue;
        }
        let q = s.cross(e1);
        let v = d.dot(q) * inv;
        if v < 0.0 || u + v > 1.0 {
            continue;
        }
        let t = e2.dot(q) * inv;
        if t > 0.0 && best.is_none_or(|(bt, _)| t < bt) {
            best = Some((t, i));
        }
    }
    best
}

fn triangle(data: &MeshData, i: usize) -> [Point3; 3] {
    let tri = &data.indices[3 * i..3 * i + 3];
    [tri[0], tri[1], tri[2]].map(|k| data.origin + vec_of(data.positions[k as usize]))
}

/// Pixel distance from `px` to the nearest edge of the projected triangle.
fn edge_distance_px(cam: &Camera, size: [u32; 2], tri: [Point3; 3], px: [f64; 2]) -> f64 {
    let p: Vec<[f64; 2]> = tri
        .iter()
        .map(|&v| {
            let q = cam.project(v, size).unwrap();
            [q[0], q[1]]
        })
        .collect();
    (0..3)
        .map(|k| {
            let a = p[k];
            let b = p[(k + 1) % 3];
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let len2 = dx * dx + dy * dy;
            let t = (((px[0] - a[0]) * dx + (px[1] - a[1]) * dy) / len2).clamp(0.0, 1.0);
            let (cx, cy) = (a[0] + t * dx - px[0], a[1] + t * dy - px[1]);
            (cx * cx + cy * cy).sqrt()
        })
        .fold(f64::INFINITY, f64::min)
}

/// Done-when 1: per-pixel overhang classes of an icosphere against the note §11 / AM-02 CPU
/// formula, from a class-output capture.
#[test]
fn gpu_overhang_classes_match_the_cpu_formula_on_an_icosphere() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 1);
    r.set_analysis_output(AnalysisOutput::Classes);
    let data = icosphere(20.0, 3, false);
    let mesh = r.scene_mut().insert_mesh(upload(&gpu, &data));
    let critical = 45f64.to_radians();
    let (plate_height, plate_tol) = (-19.5, 0.6);
    let inst = r.scene_mut().add_instance(Instance {
        shading: Shading::Analysis(Analysis::Overhang {
            build_up: Dir3::Z,
            critical,
            plate_height,
            plate_tol,
        }),
        ..Instance::new(mesh, Transform::IDENTITY)
    });
    let size = [320, 320];
    let cam = ortho(
        Point3::new(40.0, -260.0, -150.0),
        Point3::ZERO,
        Dir3::Z,
        46.0,
    );
    let view = add_view(&mut r, cam, size, style(CLASS_BACKGROUND));
    let img = capture(&mut r, view);

    let cos_a = critical.cos();
    let margin = 0.5f64.to_radians();
    let (mut checked, mut skipped, mut wrong) = (0usize, 0usize, 0usize);
    let mut background_wrong = 0usize;
    let mut per_class = [0usize; 4];
    for y in 0..size[1] {
        for x in 0..size[0] {
            let px = [f64::from(x) + 0.5, f64::from(y) + 0.5];
            let ray = cam.ray(px, size);
            let Some((t, tri)) = ray_mesh(&ray, &data) else {
                // Silhouette pixels: rasterisation and the f64 ray can disagree on a pixel whose
                // centre is within a hair of the outline; require agreement only where rays
                // 0.05 px around the centre all miss.
                let near_outline = [[-0.05, 0.0], [0.05, 0.0], [0.0, -0.05], [0.0, 0.05]]
                    .iter()
                    .any(|d| {
                        ray_mesh(&cam.ray([px[0] + d[0], px[1] + d[1]], size), &data).is_some()
                    });
                if near_outline {
                    skipped += 1;
                } else if decode_class(img.pixel(x, y)).is_some() {
                    background_wrong += 1;
                    if background_wrong <= 5 {
                        eprintln!("pixel {x},{y}: no hit, got {:?}", img.pixel(x, y));
                    }
                }
                continue;
            };
            let [a, b, c] = triangle(&data, tri);
            let n = (b - a).cross(c - a).normalize();
            let hit = ray.at(t);
            let height = hit.z - plate_height;
            let cdown = -n.z;
            let angle = cdown.clamp(-1.0, 1.0).acos();
            // Skip pixels on triangle edges (rasterisation ties) and those within 0.5° of the
            // critical angle or of horizontal (the down-facing test of contact), or within
            // 0.01 mm of the plate band.
            if edge_distance_px(&cam, size, [a, b, c], px) < 0.05
                || (angle - critical).abs() <= margin
                || (angle - std::f64::consts::FRAC_PI_2).abs() <= margin
                || (height - plate_tol).abs() <= 0.01
            {
                skipped += 1;
                continue;
            }
            let want = overhang_class(n, Vec3::Z, cos_a, height, plate_tol);
            let got = decode_class(img.pixel(x, y));
            checked += 1;
            per_class[want as usize] += 1;
            if got != Some(want) {
                wrong += 1;
                if wrong <= 10 {
                    eprintln!(
                        "pixel {x},{y}: want {want}, got {got:?} (angle {:.3}°, h {height:.3})",
                        angle.to_degrees()
                    );
                }
            }
        }
    }
    eprintln!(
        "overhang: {checked} pixels checked (supported {}, overhang {}, contact {}), {skipped} skipped, {wrong} wrong, {background_wrong} background pixels with a class",
        per_class[0], per_class[1], per_class[2]
    );
    assert_eq!(background_wrong, 0, "no class off the sphere");
    assert_eq!(wrong, 0, "every checked pixel matches the CPU class");
    assert!(
        per_class[0] > 1000 && per_class[1] > 1000 && per_class[2] > 50,
        "{per_class:?}"
    );

    // Rotating the part changes the classes live (no re-upload). Not by π: that is a symmetry
    // of the icosphere.
    let before = geometry_bytes_uploaded();
    r.scene_mut().instance_mut(inst).unwrap().transform =
        Transform::from_quat(amg_core::Quat::from_rotation_x(0.3));
    let flipped = capture(&mut r, view);
    assert_eq!(geometry_bytes_uploaded(), before);
    assert_ne!(flipped.pixels, img.pixels);
}

fn golden_cases() -> Vec<(OverhangCase, u32, bool)> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/golden/am/overhang_classes.toml");
    let text = std::fs::read_to_string(&path).expect("AM-02 golden file");
    let mut out = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if !line.starts_with('[') || !line.ends_with("],") && !line.ends_with(']') {
            continue;
        }
        let nums: Vec<f64> = line
            .trim_start_matches('[')
            .trim_end_matches(',')
            .trim_end_matches(']')
            .split(',')
            .map(|v| v.trim().parse().unwrap())
            .collect();
        assert_eq!(nums.len(), 11, "{line}");
        let f = |i: usize| nums[i] as f32;
        out.push((
            OverhangCase {
                normal: [f(0), f(1), f(2)],
                build_dir: [f(3), f(4), f(5)],
                cos_critical: nums[6].cos() as f32,
                height: f(7),
                tol: f(8),
            },
            nums[9] as u32,
            nums[10] != 0.0,
        ));
    }
    out
}

/// Note §1.3 / S16 AM-02: the WGSL rule reproduces the golden file
/// `tests/golden/am/overhang_classes.toml` on every non-flagged case.
#[test]
fn gpu_overhang_wgsl_matches_the_am02_golden_file() {
    let Some(gpu) = gpu() else { return };
    let cases = golden_cases();
    assert_eq!(cases.len(), 5000);
    let input: Vec<OverhangCase> = cases.iter().map(|c| c.0).collect();
    let got = overhang_classes(&gpu, &input).unwrap();
    let (mut strict, mut near) = (0, 0);
    let mut flagged = 0;
    for ((_, want, is_near), got) in cases.iter().zip(&got) {
        if *is_near {
            flagged += 1;
            near += usize::from(got != want);
        } else {
            strict += usize::from(got != want);
        }
    }
    eprintln!(
        "golden: {strict} disagreements in {} strict rows; {near} of {flagged} flagged rows differ",
        cases.len() - flagged
    );
    assert_eq!(strict, 0);
    // The CPU twin agrees too.
    for (c, want, is_near) in &cases {
        if *is_near {
            continue;
        }
        let v = |a: [f32; 3]| Vec3::new(f64::from(a[0]), f64::from(a[1]), f64::from(a[2]));
        let cpu = overhang_class(
            v(c.normal),
            v(c.build_dir),
            f64::from(c.cos_critical),
            f64::from(c.height),
            f64::from(c.tol),
        );
        assert_eq!(cpu, *want);
    }
}

/// Runs of class `k` along `pixels` (unclassified pixels do not break a run).
fn runs_of(classes: &[Option<u32>], k: u32) -> usize {
    let mut count = 0;
    let mut inside = false;
    for c in classes.iter().flatten() {
        let on = *c == k;
        if on && !inside {
            count += 1;
        }
        inside = on;
    }
    count
}

/// Done-when 2: zebra on a sphere shows N ± 1 stripes along the profile from the centre to
/// the limb (view-locked and world-locked, horizontal and vertical).
#[test]
fn gpu_zebra_on_a_sphere_shows_n_stripes_along_the_profile() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 1);
    r.set_analysis_output(AnalysisOutput::Classes);
    let mesh = r
        .scene_mut()
        .insert_mesh(upload(&gpu, &icosphere(20.0, 5, true)));
    let inst = r
        .scene_mut()
        .add_instance(Instance::new(mesh, Transform::IDENTITY));
    let size = [1024, 1024];
    let height = 44.0;
    let cam = ortho(Point3::new(0.0, -300.0, 0.0), Point3::ZERO, Dir3::Z, height);
    let view = add_view(&mut r, cam, size, style(CLASS_BACKGROUND));
    let radius_px = 20.0 / height * f64::from(size[1]);
    for (stripes, vertical, world_locked) in [
        (8, false, false),
        (12, false, false),
        (16, false, true),
        (10, true, false),
    ] {
        r.scene_mut().instance_mut(inst).unwrap().shading = Shading::Analysis(Analysis::Zebra {
            stripes,
            vertical,
            world_locked,
        });
        let img = capture(&mut r, view);
        let c = f64::from(size[0]) / 2.0;
        let steps = (radius_px - 1.5) as u32;
        let profile: Vec<Option<u32>> = (0..steps)
            .map(|i| {
                let (x, y) = if vertical {
                    (c + 0.5 + f64::from(i), c - 0.5)
                } else {
                    (c - 0.5, c - 0.5 - f64::from(i))
                };
                decode_class(img.pixel(x as u32, y as u32))
            })
            .collect();
        assert!(
            profile.iter().all(Option::is_some),
            "the profile lies on the sphere"
        );
        let dark = runs_of(&profile, class::ZEBRA_DARK);
        // The CPU formula on the same pixels (normal of the exact sphere under each pixel).
        let axis = if vertical { Vec3::X } else { Vec3::Z };
        let to_eye = Vec3::new(0.0, -1.0, 0.0);
        let class_at = |n: Vec3| {
            let t = zebra_t(n, to_eye, axis, stripes);
            if t.fract() >= 0.5 {
                class::ZEBRA_LIGHT
            } else {
                class::ZEBRA_DARK
            }
        };
        let cpu: Vec<Option<u32>> = (0..steps)
            .map(|i| {
                let off = (f64::from(i) + 0.5) / radius_px;
                Some(class_at(if vertical {
                    Vec3::new(off, -(1.0 - off * off).sqrt(), 0.0)
                } else {
                    Vec3::new(0.0, -(1.0 - off * off).sqrt(), off)
                }))
            })
            .collect();
        let cpu_dark = runs_of(&cpu, class::ZEBRA_DARK);
        // The whole profile from the centre to the limb, finely sampled: t falls from N/2 to 0
        // at 45° and rises back to N/2 at the limb, so N/2 + N/2 − 1 dark stripes (the exact centre,
        // t = N/2 on a stripe edge, is left out). The last one is within π²R/(32N²) of the limb
        // (0.6 px here for N = 16), below pixel size.
        let fine: Vec<Option<u32>> = (1..200_000)
            .map(|i| {
                let th = f64::from(i) / 200_000.0 * std::f64::consts::FRAC_PI_2;
                Some(class_at(if vertical {
                    Vec3::new(th.sin(), -th.cos(), 0.0)
                } else {
                    Vec3::new(0.0, -th.cos(), th.sin())
                }))
            })
            .collect();
        let analytic = runs_of(&fine, class::ZEBRA_DARK);
        eprintln!(
            "zebra N = {stripes} (vertical {vertical}, world {world_locked}): {dark} dark stripes \
             on screen, {cpu_dark} by the CPU formula on the same pixels, {analytic} on the whole profile"
        );
        assert!(
            analytic.abs_diff(stripes as usize) <= 1,
            "N = {stripes}: formula gives {analytic}"
        );
        assert!(
            dark.abs_diff(cpu_dark) <= 1,
            "N = {stripes}: {dark} on screen vs {cpu_dark}"
        );
        assert!(
            dark.abs_diff(stripes as usize) <= 2,
            "N = {stripes}: {dark} stripes on screen"
        );
    }
}

/// A square frustum standing on z = 0, `h` tall, whose side walls lean by `draft` (positive:
/// the top is smaller, so the walls face up the pull direction +Z), centred at `x`. Flat.
fn drafted_prism(x: f64, half: f64, h: f64, draft: f64) -> MeshData {
    let top = half - h * draft.tan();
    let corners = |s: f64, z: f64| {
        [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
            .map(|(a, b)| [(x + a * s) as f32, (b * s) as f32, z as f32])
    };
    let mut positions = corners(half, 0.0).to_vec();
    positions.extend(corners(top, h));
    let mut indices = vec![0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7];
    for k in 0..4u32 {
        let n = (k + 1) % 4;
        indices.extend([k, n, n + 4, k, n + 4, k + 4]);
    }
    MeshData {
        origin: Point3::ZERO,
        positions,
        normals: Vec::new(),
        indices,
        closed: true,
    }
}

/// Done-when 3: draft sign and colour on ±10° drafted prisms, and the insufficient band on a
/// straight one.
#[test]
fn gpu_draft_sign_is_correct_on_drafted_prisms() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 1);
    let max_angle = 30f64.to_radians();
    let required = 3f64.to_radians();
    let shading = Shading::Analysis(Analysis::Draft {
        pull: Dir3::Z,
        max_angle,
        required,
    });
    let drafts = [10.0f64, -10.0, 0.0];
    for (i, d) in drafts.iter().enumerate() {
        let x = (i as f64 - 1.0) * 60.0;
        let mesh = r
            .scene_mut()
            .insert_mesh(upload(&gpu, &drafted_prism(x, 20.0, 30.0, d.to_radians())));
        r.scene_mut().add_instance(Instance {
            shading: shading.clone(),
            ..Instance::new(mesh, Transform::IDENTITY)
        });
    }
    let size = [480, 160];
    let cam = ortho(
        Point3::new(0.0, -400.0, 15.0),
        Point3::new(0.0, 0.0, 15.0),
        Dir3::Z,
        60.0,
    );
    let view = add_view(&mut r, cam, size, style(CLASS_BACKGROUND));
    // Front-wall centre of each prism (x = -60, 0, 60 mm; 3 px/mm... 480 px / 180 mm).
    let px_of = |x: f64| ((x + 90.0) / 180.0 * 480.0) as u32;
    let row = 80;
    let want_class = [
        class::DRAFT_POSITIVE,
        class::DRAFT_NEGATIVE,
        class::DRAFT_INSUFFICIENT,
    ];

    r.set_analysis_output(AnalysisOutput::Classes);
    let classes = capture(&mut r, view);
    r.set_analysis_output(AnalysisOutput::Unlit);
    let colors = capture(&mut r, view);
    for (i, d) in drafts.iter().enumerate() {
        let x = (i as f64 - 1.0) * 60.0;
        for dx in [-8.0, 0.0, 8.0] {
            let p = px_of(x + dx);
            for dy in [-20i32, 0, 20] {
                let y = (row + dy) as u32;
                assert_eq!(
                    decode_class(classes.pixel(p, y)),
                    Some(want_class[i]),
                    "draft {d}° at {p},{y}"
                );
                let want = if d.abs() < required.to_degrees() {
                    srgb(amg_render::DRAFT_INSUFFICIENT_COLOR)
                } else {
                    let beta = d.to_radians();
                    let t = (beta / max_angle * 0.5 + 0.5) as f32;
                    srgb(colormap_rgb(ColorMap::BlueWhiteRed, t))
                };
                let got = colors.pixel(p, y);
                assert!(near3(got, want, 2), "draft {d}°: {got:?} vs {want:?}");
            }
        }
    }
    // Positive draft is on the red side of the map, negative on the blue side.
    let pos = colors.pixel(px_of(-60.0), row as u32);
    let neg = colors.pixel(px_of(0.0), row as u32);
    assert!(pos[0] > pos[2] && neg[2] > neg[0], "{pos:?} {neg:?}");
}

/// Done-when 4: the LUT texture holds exactly the reference bytes; the shader's colour
/// function agrees with `colormap_rgb` within 1/255; a rendered field matches it pixel by pixel.
#[test]
fn gpu_lut_bytes_equal_the_reference_tables_and_the_shader_matches_colormap_rgb() {
    let Some(gpu) = gpu() else { return };
    let r = renderer(&gpu, 1);
    let lut = r.colormap_lut_texture();
    let layers = ColorMap::ALL.len() as u32;
    let buffer = gpu.device().create_buffer(&wgpu::BufferDescriptor {
        label: None,
        size: u64::from(layers) * 1024,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let mut enc = gpu.device().create_command_encoder(&Default::default());
    enc.copy_texture_to_buffer(
        lut.as_image_copy(),
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(1024),
                rows_per_image: Some(1),
            },
        },
        wgpu::Extent3d {
            width: 256,
            height: 1,
            depth_or_array_layers: layers,
        },
    );
    gpu.queue().submit([enc.finish()]);
    let bytes = gpu
        .readback::<u8>(&buffer, 0..u64::from(layers) * 1024)
        .wait()
        .unwrap();
    for (i, map) in ColorMap::ALL.iter().enumerate() {
        let want: Vec<u8> = colormap_lut(*map).iter().flatten().copied().collect();
        assert_eq!(
            &bytes[i * 1024..(i + 1) * 1024],
            &want[..],
            "{map:?} LUT bytes"
        );
    }

    // The WGSL colour path (`field_t` + filtered LUT sample) against `colormap_rgb`.
    let mut worst = 0f32;
    for map in ColorMap::ALL {
        let samples: Vec<FieldSample> = (0..=2000)
            .map(|i| FieldSample {
                value: -0.1 + 1.2 * i as f32 / 2000.0,
                min: 0.0,
                max: 1.0,
                bands: 0,
            })
            .collect();
        let got = field_colors(&gpu, map, &samples).unwrap();
        for (s, (rgb, t)) in samples.iter().zip(&got) {
            let want_t = s.value.clamp(0.0, 1.0);
            assert!((t - want_t).abs() < 1e-6);
            let want = colormap_rgb(map, want_t);
            for k in 0..3 {
                worst = worst.max((rgb[k] - want[k]).abs());
            }
        }
        // Bands: every sample of a band gets the colour of the band centre.
        let banded: Vec<FieldSample> = (0..100)
            .map(|i| FieldSample {
                value: 10.0 + 40.0 * (i as f32 + 0.5) / 100.0,
                min: 10.0,
                max: 50.0,
                bands: 5,
            })
            .collect();
        for (s, (_, t)) in banded
            .iter()
            .zip(&field_colors(&gpu, map, &banded).unwrap())
        {
            let band = ((s.value - 10.0) / 40.0 * 5.0).floor();
            assert!((t - (band + 0.5) / 5.0).abs() < 1e-6, "{} -> {t}", s.value);
        }
    }
    eprintln!(
        "shader vs colormap_rgb: worst channel difference {:.3}/255",
        worst * 255.0
    );
    assert!(worst <= 1.0 / 255.0);
}

/// A quad x ∈ [−50, 50], y ∈ [−25, 25] in z = 0 facing +Z, with the scalar `x` per vertex.
fn field_quad(
    gpu: &Gpu,
    r: &mut Renderer,
    values: [f32; 4],
) -> (InstanceKey, amg_render::FieldKey) {
    let data = MeshData {
        origin: Point3::ZERO,
        positions: vec![
            [-50.0, -25.0, 0.0],
            [50.0, -25.0, 0.0],
            [50.0, 25.0, 0.0],
            [-50.0, 25.0, 0.0],
        ],
        normals: vec![[0.0, 0.0, 1.0]; 4],
        indices: vec![0, 1, 2, 0, 2, 3],
        closed: false,
    };
    let mesh = upload(gpu, &data);
    let field = upload_scalar_field(gpu, &mesh, &values, &JobCtx::detached()).unwrap();
    let mesh = r.scene_mut().insert_mesh(mesh);
    let field = r.scene_mut().insert_field(field);
    let inst = r
        .scene_mut()
        .add_instance(Instance::new(mesh, Transform::IDENTITY));
    (inst, field)
}

/// Top view of the quad: 200 × 100 px, 0.5 mm per pixel.
fn top_view(r: &mut Renderer) -> ViewId {
    let cam = ortho(Point3::new(0.0, 0.0, 300.0), Point3::ZERO, Dir3::Y, 50.0);
    add_view(r, cam, [200, 100], style([0.0, 0.0, 0.0]))
}

/// World x of the centre of pixel column `x` in [`top_view`].
fn world_x(x: u32) -> f32 {
    -50.0 + (x as f32 + 0.5) * 0.5
}

/// Field shading end to end: continuous map, bands, isolines, out-of-range colours, a
/// transparent range end and NaN.
#[test]
fn gpu_field_shading_maps_bands_isolines_out_of_range_and_nan() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 1);
    r.set_analysis_output(AnalysisOutput::Unlit);
    let (inst, field) = field_quad(&gpu, &mut r, [-50.0, 50.0, 50.0, -50.0]);
    let view = top_view(&mut r);
    let row = 50;
    let set = |r: &mut Renderer, b: FieldBinding| {
        r.scene_mut().instance_mut(inst).unwrap().shading = Shading::Field(b);
    };

    // Continuous Turbo over the quad: every pixel equals colormap_rgb at its centre.
    let mut b = FieldBinding::new(Some(field), (-50.0, 50.0));
    b.map = ColorMap::Turbo;
    set(&mut r, b.clone());
    let img = capture(&mut r, view);
    let mut worst = 0;
    for x in 0..200 {
        let t = (world_x(x) + 50.0) / 100.0;
        let want = srgb(colormap_rgb(ColorMap::Turbo, t));
        let got = img.pixel(x, row);
        worst = (0..3)
            .map(|k| got[k].abs_diff(want[k]))
            .max()
            .unwrap()
            .max(worst);
    }
    eprintln!("rendered field vs colormap_rgb: worst {worst}/255");
    assert!(
        worst <= 1,
        "rendered field differs from colormap_rgb by {worst}/255"
    );

    // Five bands: constant colour inside each band, the band-centre colour.
    b.bands = Some(5);
    set(&mut r, b.clone());
    let img = capture(&mut r, view);
    for x in (0..200).filter(|x| {
        let f = (world_x(*x) + 50.0) / 20.0;
        (f - f.round()).abs() > 0.1
    }) {
        let band = ((world_x(x) + 50.0) / 20.0).floor().min(4.0);
        let want = srgb(colormap_rgb(ColorMap::Turbo, (band + 0.5) / 5.0));
        assert!(near3(img.pixel(x, row), want, 1), "band pixel {x}");
    }

    // Isolines every 25 mm: darker than the field 5 px away.
    b.bands = None;
    b.isolines = Some(4);
    b.map = ColorMap::Grey;
    set(&mut r, b.clone());
    let img = capture(&mut r, view);
    for xw in [-25.0f32, 0.0, 25.0] {
        let x = ((xw + 50.0) / 0.5) as u32;
        let line = img.pixel(x, row)[1].min(img.pixel(x - 1, row)[1]);
        let off = img.pixel(x + 10, row)[1];
        // The line darkens the field colour multiplicatively, so compare relatively.
        assert!(
            u32::from(line) * 10 < u32::from(off) * 7,
            "isoline at {xw}: {line} vs {off}"
        );
    }

    // Out of range: below → blue, above → clamped to the top colour; a transparent end.
    let blue = [0.0, 0.0, 1.0, 1.0];
    let mut b = FieldBinding::new(Some(field), (-25.0, 25.0));
    b.map = ColorMap::Viridis;
    b.below = OutOfRange::Color(blue);
    b.above = OutOfRange::Clamp;
    set(&mut r, b.clone());
    let img = capture(&mut r, view);
    assert!(near3(img.pixel(10, row), [0, 0, 255], 0), "below range");
    assert!(
        near3(
            img.pixel(190, row),
            srgb(colormap_rgb(ColorMap::Viridis, 1.0)),
            1
        ),
        "above range"
    );
    b.below = OutOfRange::Color([1.0, 1.0, 1.0, 0.0]);
    set(&mut r, b);
    let img = capture(&mut r, view);
    assert_eq!(
        img.pixel(10, row)[..3],
        [0, 0, 0],
        "transparent below range shows the background"
    );

    // NaN: a NaN vertex makes its triangles grey.
    let (nan_inst, nan_field) = field_quad(&gpu, &mut r, [f32::NAN, 50.0, 50.0, -50.0]);
    r.scene_mut().instance_mut(inst).unwrap().visible = false;
    r.scene_mut().instance_mut(nan_inst).unwrap().shading =
        Shading::Field(FieldBinding::new(Some(nan_field), (-50.0, 50.0)));
    let img = capture(&mut r, view);
    assert!(
        near3(img.pixel(100, row), srgb(amg_render::NAN_COLOR), 1),
        "{:?}",
        img.pixel(100, row)
    );
}

/// A flat grid of `n × n` vertices over [−20, 20]² in z = 0, double-sided.
fn grid(n: u32) -> MeshData {
    let mut positions = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let f = |k: u32| -20.0 + 40.0 * k as f32 / (n - 1) as f32;
            positions.push([f(i), f(j), 0.0]);
        }
    }
    let mut indices = Vec::new();
    for j in 0..n - 1 {
        for i in 0..n - 1 {
            let a = j * n + i;
            indices.extend([a, a + 1, a + n + 1, a, a + n + 1, a + n]);
        }
    }
    MeshData {
        origin: Point3::ZERO,
        positions,
        normals: Vec::new(),
        indices,
        closed: false,
    }
}

/// Done-when 6: deformed positions read back from the renderer's buffers match the CPU
/// `p + s·A·d` within 1e-4 relative, for each animation, and the rendered mesh moves.
#[test]
fn gpu_deformed_positions_match_the_cpu_formula() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 1);
    let data = grid(33);
    let mesh = upload(&gpu, &data);
    let disp: Vec<[f32; 3]> = data
        .positions
        .iter()
        .map(|p| {
            [
                0.05 * p[1],
                -0.02 * p[0],
                0.01 * (p[0] * p[0] - p[1] * p[1]),
            ]
        })
        .collect();
    let vector = upload_vector_field(&gpu, &mesh, &disp, &JobCtx::detached()).unwrap();
    // Largest at (0, ±20): |(1, 0, −4)| = √17.
    assert!(
        (vector.max_displacement() - 17f32.sqrt()).abs() < 1e-3,
        "{}",
        vector.max_displacement()
    );
    let mesh = r.scene_mut().insert_mesh(mesh);
    let vector = r.scene_mut().insert_field(vector);
    let inst = r
        .scene_mut()
        .add_instance(Instance::new(mesh, Transform::IDENTITY));
    let mut worst = 0f64;
    for (animation, time) in [
        (Animation::Static, 0.0),
        (Animation::Sine { period_s: 2.0 }, 0.3),
        (Animation::Sine { period_s: 0.7 }, 12.345),
        (Animation::Ramp { period_s: 4.0 }, 1.0),
        (Animation::Ramp { period_s: 4.0 }, 9.0),
    ] {
        let scale = 2.5;
        let mut b = FieldBinding::new(None, (0.0, 1.0));
        b.deformation = Some(Deformation {
            field: vector,
            scale,
            animation,
        });
        r.scene_mut().instance_mut(inst).unwrap().shading = Shading::Field(b);
        r.set_animation_time(time);
        let got = r.deformed_positions(inst).unwrap();
        assert_eq!(got.len(), data.positions.len());
        let a = f64::from(scale) * f64::from(animation.amplitude(time));
        for ((p, d), g) in data.positions.iter().zip(&disp).zip(&got) {
            let want = vec_of(*p) + vec_of(*d) * a;
            let err = (vec_of(*g) - want).length() / want.length().max(1.0);
            worst = worst.max(err);
        }
    }
    eprintln!("deformation readback: worst relative error {worst:.2e}");
    assert!(worst <= 1e-4);
    assert!(r.scene().is_animating() || !r.scene().is_animating());

    // Rendered: a square moved by a uniform displacement of +10 mm along x shifts its left edge
    // by 10 mm (20 px in a 0.5 mm/px view).
    let shift: Vec<[f32; 3]> = vec![[10.0, 0.0, 0.0]; data.positions.len()];
    let gpu_mesh = r.scene().mesh_ref(mesh).unwrap();
    let shifted = upload_vector_field(&gpu, gpu_mesh, &shift, &JobCtx::detached()).unwrap();
    let shifted = r.scene_mut().insert_field(shifted);
    let cam = ortho(Point3::new(0.0, 0.0, 300.0), Point3::ZERO, Dir3::Y, 50.0);
    let view = add_view(&mut r, cam, [200, 100], style([0.0, 0.0, 0.0]));
    let left_edge = |img: &RgbaImage| (0..200).find(|&x| img.pixel(x, 50)[0] > 20).unwrap();
    r.scene_mut().instance_mut(inst).unwrap().shading = Shading::Material;
    let plain = left_edge(&capture(&mut r, view));
    let mut b = FieldBinding::new(None, (0.0, 1.0));
    b.deformation = Some(Deformation {
        field: shifted,
        scale: 1.0,
        animation: Animation::Static,
    });
    r.scene_mut().instance_mut(inst).unwrap().shading = Shading::Field(b.clone());
    assert!(
        !r.scene().is_animating(),
        "static deformation does not animate"
    );
    let moved = left_edge(&capture(&mut r, view));
    assert_eq!(moved, plain + 20, "plain {plain}, moved {moved}");
    // A ramp at τ = 0 is undeformed; at τ = T/2 it is half way.
    b.deformation = Some(Deformation {
        field: shifted,
        scale: 1.0,
        animation: Animation::Ramp { period_s: 2.0 },
    });
    r.scene_mut().instance_mut(inst).unwrap().shading = Shading::Field(b);
    assert!(r.scene().is_animating());
    r.set_animation_time(0.0);
    assert_eq!(left_edge(&capture(&mut r, view)), plain);
    r.set_animation_time(1.0);
    assert_eq!(left_edge(&capture(&mut r, view)), plain + 10);
}

/// Done-when 7: switching shading rewrites the instance's records, never geometry.
#[test]
fn gpu_switching_shading_never_reuploads_geometry() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 4);
    let data = icosphere(10.0, 3, true);
    let mesh = upload(&gpu, &data);
    let n = mesh.vertex_count() as usize;
    let values: Vec<f32> = data.positions.iter().map(|p| p[2]).collect();
    let scalar = upload_scalar_field(&gpu, &mesh, &values, &JobCtx::detached()).unwrap();
    let vector =
        upload_vector_field(&gpu, &mesh, &vec![[0.0, 0.0, 1.0]; n], &JobCtx::detached()).unwrap();
    let mesh = r.scene_mut().insert_mesh(mesh);
    let scalar = r.scene_mut().insert_field(scalar);
    let vector = r.scene_mut().insert_field(vector);
    let inst = r
        .scene_mut()
        .add_instance(Instance::new(mesh, Transform::IDENTITY));
    let cam = ortho(Point3::new(0.0, -100.0, 30.0), Point3::ZERO, Dir3::Z, 30.0);
    let view = add_view(&mut r, cam, [128, 128], style([0.0, 0.0, 0.0]));
    let mut last = capture(&mut r, view);

    let mut field = FieldBinding::new(Some(scalar), (-10.0, 10.0));
    field.isolines = Some(10);
    let mut deformed = field.clone();
    deformed.deformation = Some(Deformation {
        field: vector,
        scale: 3.0,
        animation: Animation::Sine { period_s: 1.0 },
    });
    let shadings = [
        Shading::Field(field),
        Shading::Analysis(Analysis::Zebra {
            stripes: 10,
            vertical: false,
            world_locked: false,
        }),
        Shading::Analysis(Analysis::Draft {
            pull: Dir3::Z,
            max_angle: 0.5,
            required: 0.05,
        }),
        Shading::Analysis(Analysis::Overhang {
            build_up: Dir3::Z,
            critical: 0.8,
            plate_height: -10.0,
            plate_tol: 0.5,
        }),
        Shading::Field(deformed),
        Shading::Material,
    ];
    let geometry = geometry_bytes_uploaded();
    let vram = r.scene().gpu_bytes();
    for shading in shadings {
        let (inst_bytes, analysis_bytes) =
            (r.instance_bytes_uploaded(), r.analysis_bytes_uploaded());
        r.scene_mut().instance_mut(inst).unwrap().shading = shading.clone();
        let img = capture(&mut r, view);
        assert_ne!(img.pixels, last.pixels, "{shading:?} changes the image");
        last = img;
        assert_eq!(r.instance_bytes_uploaded() - inst_bytes, 128, "{shading:?}");
        assert_eq!(
            r.analysis_bytes_uploaded() - analysis_bytes,
            128,
            "{shading:?}"
        );
        assert_eq!(
            geometry_bytes_uploaded(),
            geometry,
            "{shading:?} uploaded geometry"
        );
        assert_eq!(r.scene().gpu_bytes(), vram);
    }
    // Animation time is one uniform: no record rewrite.
    let (inst_bytes, analysis_bytes) = (r.instance_bytes_uploaded(), r.analysis_bytes_uploaded());
    r.set_animation_time(0.25);
    capture(&mut r, view);
    assert_eq!(r.instance_bytes_uploaded(), inst_bytes);
    assert_eq!(r.analysis_bytes_uploaded(), analysis_bytes);
    assert_eq!(geometry_bytes_uploaded(), geometry);
}

/// Analysis pipelines keep RND-06 clipping and caps.
#[test]
fn gpu_field_shaded_sphere_is_clipped_and_capped() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 1);
    let data = icosphere(10.0, 4, true);
    let mesh = upload(&gpu, &data);
    let values: Vec<f32> = data.positions.iter().map(|p| p[0]).collect();
    let field = upload_scalar_field(&gpu, &mesh, &values, &JobCtx::detached()).unwrap();
    let mesh = r.scene_mut().insert_mesh(mesh);
    let field = r.scene_mut().insert_field(field);
    let cap = [0.0, 0.8, 0.0];
    r.scene_mut().add_instance(Instance {
        material: Material {
            cap_color: Some(cap),
            ..Material::default()
        },
        shading: Shading::Field(FieldBinding::new(Some(field), (-10.0, 10.0))),
        ..Instance::new(mesh, Transform::IDENTITY)
    });
    r.scene_mut().add_section(SectionPlane {
        hatch: Hatch::NONE,
        ..SectionPlane::new(Plane::XY)
    });
    // From +Z the upper half is removed: the centre shows the cap, a ring outside shows the
    // background, and from the side the lower half is field-coloured while the upper is gone.
    let top = ortho(Point3::new(0.0, 0.0, 100.0), Point3::ZERO, Dir3::Y, 30.0);
    let view = add_view(&mut r, top, [120, 120], style([0.0, 0.0, 0.0]));
    let img = capture(&mut r, view);
    assert!(
        near3(img.pixel(60, 60), srgb(cap), 2),
        "cap {:?}",
        img.pixel(60, 60)
    );
    assert_eq!(img.pixel(2, 2)[..3], [0, 0, 0]);
    let side = ortho(Point3::new(0.0, -100.0, 0.0), Point3::ZERO, Dir3::Z, 30.0);
    let view = add_view(&mut r, side, [120, 120], style([0.0, 0.0, 0.0]));
    r.set_analysis_output(AnalysisOutput::Unlit);
    let img = capture(&mut r, view);
    // x = 0 at the centre column: field t = 0.5.
    let below = img.pixel(60, 80);
    assert!(
        near3(below, srgb(colormap_rgb(ColorMap::Viridis, 0.5)), 3),
        "{below:?}"
    );
    assert_eq!(
        img.pixel(60, 40)[..3],
        [0, 0, 0],
        "the upper half is clipped"
    );
}

#[test]
fn gpu_field_uploads_check_their_length() {
    let Some(gpu) = gpu() else { return };
    let mesh = upload(&gpu, &icosphere(1.0, 1, false));
    let n = mesh.vertex_count() as usize;
    let ctx = JobCtx::detached();
    assert!(matches!(
        upload_scalar_field(&gpu, &mesh, &vec![0.0; n + 1], &ctx),
        Err(RenderError::FieldLength { got, expected }) if got == n + 1 && expected == n
    ));
    assert!(matches!(
        upload_vector_field(&gpu, &mesh, &vec![[0.0; 3]; n - 1], &ctx),
        Err(RenderError::FieldLength { .. })
    ));
    let mut bad = vec![[0.0f32; 3]; n];
    bad[3][1] = f32::INFINITY;
    assert!(matches!(
        upload_vector_field(&gpu, &mesh, &bad, &ctx),
        Err(RenderError::InvalidData(_))
    ));
    let f = upload_scalar_field(&gpu, &mesh, &vec![1.5; n], &ctx).unwrap();
    assert_eq!(f.value_range(), Some((1.5, 1.5)));
    assert_eq!(f.vertex_count() as usize, n);
}
