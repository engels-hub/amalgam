//! RND-06 GPU tests: section planes (clipping in the mesh, depth, edge and point pipelines),
//! back-face caps with per-instance cap colour, screen-constant hatching, `closed` gating and
//! the cap depth pass. Named `gpu_*` (nextest test-group `gpu`, serial); they skip without an
//! adapter and fail under `AMG_REQUIRE_GPU=1` (CI runs them on WARP).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::Arc;

use amg_core::{Dir3, Plane, Point3, Tolerance, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::{
    Background, Camera, EdgeData, Hatch, Instance, Lighting, LineStyle, MeshData, PointData,
    PointStyle, Polyline, Projection, Renderer, RendererConfig, RgbaImage, SectionPlane, ViewId,
    ViewStyle, VisualStyle, upload_edges, upload_mesh, upload_points,
};

const RED: [f32; 3] = [1.0, 0.0, 0.0];
const GREEN: [f32; 4] = [0.0, 0.6, 0.0, 1.0];
const BLUE: [f32; 4] = [0.0, 0.0, 1.0, 1.0];

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

fn style(visual: VisualStyle) -> ViewStyle {
    ViewStyle {
        background: Background::Solid([0.0, 0.0, 0.0]),
        lighting: Lighting::Flat,
        visual,
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

fn dir(v: Vec3) -> Dir3 {
    Dir3::new(v, &Tolerance::default()).unwrap()
}

/// A section plane through `origin` removing the side `normal` points to, with `hatch`.
fn section(origin: Point3, normal: Vec3, hatch: Hatch) -> SectionPlane {
    SectionPlane {
        hatch,
        ..SectionPlane::new(Plane::from_normal(origin, dir(normal)))
    }
}

/// The XY plane at the origin (x axis = world X), removing z > 0.
fn cut_z(hatch: Hatch) -> SectionPlane {
    SectionPlane {
        hatch,
        ..SectionPlane::new(Plane::XY)
    }
}

/// A UV sphere of `segs` meridians and `rings` (even) latitude bands, so the equator is a ring
/// of vertices; outward CCW winding, smooth normals.
fn uv_sphere(r: f32, segs: u32, rings: u32, closed: bool) -> MeshData {
    let mut positions = vec![[0.0, 0.0, r]];
    for i in 1..rings {
        let theta = std::f32::consts::PI * i as f32 / rings as f32;
        for j in 0..segs {
            let phi = std::f32::consts::TAU * j as f32 / segs as f32;
            positions.push([
                r * theta.sin() * phi.cos(),
                r * theta.sin() * phi.sin(),
                r * theta.cos(),
            ]);
        }
    }
    positions.push([0.0, 0.0, -r]);
    let south = positions.len() as u32 - 1;
    let v = |i: u32, j: u32| 1 + (i - 1) * segs + j % segs;
    let mut indices = Vec::new();
    for j in 0..segs {
        indices.extend([0, v(1, j), v(1, j + 1)]);
    }
    for i in 1..rings - 1 {
        for j in 0..segs {
            indices.extend([v(i, j), v(i + 1, j), v(i + 1, j + 1)]);
            indices.extend([v(i, j), v(i + 1, j + 1), v(i, j + 1)]);
        }
    }
    for j in 0..segs {
        indices.extend([v(rings - 1, j), south, v(rings - 1, j + 1)]);
    }
    let normals = positions
        .iter()
        .map(|p| [p[0] / r, p[1] / r, p[2] / r])
        .collect();
    MeshData {
        origin: Point3::ZERO,
        positions,
        normals,
        indices,
        closed,
    }
}

fn add_body(r: &mut Renderer, gpu: &Gpu, data: &MeshData, cap: Option<[f32; 3]>) {
    let key = r
        .scene_mut()
        .insert_mesh(upload_mesh(gpu, data, &JobCtx::detached()).unwrap());
    let mut inst = Instance::new(key, Transform::IDENTITY);
    inst.material.base_color = GREEN;
    inst.material.cap_color = cap;
    r.scene_mut().add_instance(inst);
}

/// Red cap colour, possibly mixed with blue hatch; never the green body or the background.
fn cap_or_hatch(px: [u8; 4]) -> bool {
    px[1] < 40 && (px[0] > 90 || px[2] > 90)
}

fn red(px: [u8; 4]) -> bool {
    px[0] > 200 && px[1] < 40 && px[2] < 40
}

fn top_ortho(height: f64) -> Camera {
    Camera {
        eye: Point3::new(0.0, 0.0, 100.0),
        target: Point3::ZERO,
        up: Dir3::Y,
        projection: Projection::Orthographic { height },
    }
}

fn top_perspective() -> Camera {
    Camera {
        eye: Point3::new(0.0, 0.0, 40.0),
        target: Point3::ZERO,
        up: Dir3::Y,
        projection: Projection::Perspective {
            fov_y: 40f64.to_radians(),
        },
    }
}

/// Projected centre and radius (px) of the circle of radius `r` in the plane z = 0.
fn projected_circle(cam: &Camera, size: [u32; 2], r: f64) -> ([f64; 2], f64) {
    let c = cam.project(Point3::ZERO, size).unwrap();
    let e = cam.project(Point3::new(r, 0.0, 0.0), size).unwrap();
    ([c[0], c[1]], (e[0] - c[0]).hypot(e[1] - c[1]))
}

/// Done-when 1: a closed sphere r = 10 cut by z = 0 and seen from +Z shows cap or hatch colour
/// at every pixel inside its projected circle shrunk by 1 px (watertight), orthographic and
/// perspective, 1× and 4× MSAA, and two captures are identical.
#[test]
fn gpu_closed_sphere_cut_shows_a_watertight_cap() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [256, 256];
    // 256 segments: the inscribed equator polygon is within 0.0008 mm (0.008 px) of the circle.
    let sphere = uv_sphere(10.0, 256, 128, true);
    let hatch = Hatch {
        color: BLUE,
        ..Hatch::default()
    };
    for samples in [1, 4] {
        for (name, cam) in [
            ("ortho", top_ortho(25.0)),
            ("perspective", top_perspective()),
        ] {
            let mut r = renderer(&gpu, samples);
            add_body(&mut r, &gpu, &sphere, Some(RED));
            r.scene_mut().add_section(cut_z(hatch));
            let view = add_view(&mut r, cam, size, style(VisualStyle::Shaded));
            let img = capture(&mut r, view);
            let again = r.capture(view).unwrap().wait().unwrap();
            assert_eq!(img, again, "captures are deterministic");
            let (c, radius) =
                projected_circle(&cam, size, 10.0 * (std::f64::consts::PI / 256.0).cos());
            let mut inside = 0;
            let mut bad = Vec::new();
            let mut hatch_px = 0;
            for y in 0..size[1] {
                for x in 0..size[0] {
                    let d = (f64::from(x) + 0.5 - c[0]).hypot(f64::from(y) + 0.5 - c[1]);
                    let px = img.pixel(x, y);
                    if d < radius - 1.0 {
                        inside += 1;
                        if !cap_or_hatch(px) {
                            bad.push((x, y, px));
                        }
                        if px[2] > 90 {
                            hatch_px += 1;
                        }
                    } else if d > radius + 1.5 {
                        assert!(
                            !cap_or_hatch(px),
                            "{name} x{samples}: cap outside at {x},{y}"
                        );
                    }
                }
            }
            eprintln!(
                "{name} x{samples}: radius {radius:.1} px, {inside} pixels inside, {} not cap, {hatch_px} hatched",
                bad.len()
            );
            assert!(inside > 15_000);
            assert!(
                bad.is_empty(),
                "{name} x{samples}: {} holes, first {:?}",
                bad.len(),
                bad.first()
            );
            assert!(hatch_px > inside / 20, "hatch lines are drawn");
        }
    }
}

/// A closed 100 mm cube centred on the origin, flat shaded.
fn cube_data() -> MeshData {
    let mesh = amg_testkit::shapes::cube(Point3::splat(-50.0), 100.0);
    MeshData {
        origin: Point3::ZERO,
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

/// Signed distance (px) of `p` inside the convex polygon `poly` (positive inside).
fn inside_distance(poly: &[[f64; 2]], p: [f64; 2]) -> f64 {
    let n = poly.len();
    let area: f64 = (0..n)
        .map(|i| {
            let (a, b) = (poly[i], poly[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    let sign = area.signum();
    (0..n)
        .map(|i| {
            let (a, b) = (poly[i], poly[(i + 1) % n]);
            let e = [b[0] - a[0], b[1] - a[1]];
            let len = e[0].hypot(e[1]);
            sign * (e[0] * (p[1] - a[1]) - e[1] * (p[0] - a[0])) / len
        })
        .fold(f64::INFINITY, f64::min)
}

fn corner_view() -> Camera {
    Camera {
        eye: Point3::new(400.0, 300.0, 250.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Orthographic { height: 260.0 },
    }
}

/// Done-when 2: a cube cut by two orthogonal planes (x > 0 and y > 0 removed) shows an L-shaped
/// cap (two rectangles meeting at the corner edge); every pixel 1.5 px inside the projected cap
/// rectangles is cap colour, and none 1.5 px outside them is, except isolated pixels on the
/// silhouette of the kept block (within 1.5 px of its outline): there rasterisation can cover a
/// back face but not the front face sharing its edge, and that back face is taken for a cap.
#[test]
fn gpu_cube_cut_by_two_planes_gives_an_l_shaped_cap() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [256, 256];
    let cam = corner_view();
    let proj = |p: Point3| {
        let q = cam.project(p, size).unwrap();
        [q[0], q[1]]
    };
    let cap_x: Vec<_> = [
        Point3::new(0.0, -50.0, -50.0),
        Point3::new(0.0, 0.0, -50.0),
        Point3::new(0.0, 0.0, 50.0),
        Point3::new(0.0, -50.0, 50.0),
    ]
    .into_iter()
    .map(proj)
    .collect();
    let cap_y: Vec<_> = [
        Point3::new(-50.0, 0.0, -50.0),
        Point3::new(0.0, 0.0, -50.0),
        Point3::new(0.0, 0.0, 50.0),
        Point3::new(-50.0, 0.0, 50.0),
    ]
    .into_iter()
    .map(proj)
    .collect();
    let mut block: Vec<[f64; 2]> = (0..8u32)
        .map(|i| {
            let c = cube_corner(i);
            proj(Point3::new(c.x.min(0.0), c.y.min(0.0), c.z))
        })
        .collect();
    let block = convex_hull(&mut block);
    for samples in [1, 4] {
        let mut r = renderer(&gpu, samples);
        add_body(&mut r, &gpu, &cube_data(), Some(RED));
        r.scene_mut()
            .add_section(section(Point3::ZERO, Vec3::X, Hatch::NONE));
        r.scene_mut()
            .add_section(section(Point3::ZERO, Vec3::Y, Hatch::NONE));
        let view = add_view(&mut r, cam, size, style(VisualStyle::Shaded));
        let img = capture(&mut r, view);
        let (mut inside, mut cap_pixels, mut specks) = (0, 0, 0);
        for y in 0..size[1] {
            for x in 0..size[0] {
                let p = [f64::from(x) + 0.5, f64::from(y) + 0.5];
                let d = inside_distance(&cap_x, p).max(inside_distance(&cap_y, p));
                let px = img.pixel(x, y);
                if red(px) {
                    cap_pixels += 1;
                }
                if d > 1.5 {
                    inside += 1;
                    assert!(red(px), "x{samples}: not cap at {x},{y}: {px:?}");
                } else if d < -1.5 && red(px) {
                    assert!(
                        inside_distance(&block, p).abs() <= 1.5,
                        "x{samples}: cap outside at {x},{y}: {px:?}"
                    );
                    specks += 1;
                }
            }
        }
        eprintln!(
            "x{samples}: {inside} pixels inside the L, {cap_pixels} cap pixels, {specks} silhouette specks"
        );
        assert!(inside > 5000, "the L is in view ({inside})");
        assert!(specks <= 8, "{specks} cap-coloured silhouette pixels");
    }
}

/// Done-when 3: an open mesh draws no cap; its inside shows shaded back faces.
#[test]
fn gpu_open_mesh_draws_no_cap() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [128, 128];
    let cam = top_ortho(25.0);
    for (closed, expect_cap) in [(false, false), (true, true)] {
        let mut r = renderer(&gpu, 4);
        add_body(&mut r, &gpu, &uv_sphere(10.0, 96, 48, closed), Some(RED));
        r.scene_mut().add_section(cut_z(Hatch::NONE));
        let view = add_view(&mut r, cam, size, style(VisualStyle::Shaded));
        let img = capture(&mut r, view);
        let caps = img
            .pixels
            .chunks_exact(4)
            .filter(|p| red([p[0], p[1], p[2], p[3]]))
            .count();
        let centre = img.pixel(64, 64);
        eprintln!("closed {closed}: {caps} cap pixels, centre {centre:?}");
        if expect_cap {
            assert!(caps > 3000);
        } else {
            assert_eq!(caps, 0, "an open mesh has no cap");
            assert!(
                centre[1] > 40 && centre[0] < 40,
                "the inside shows shaded back faces: {centre:?}"
            );
        }
    }
}

/// Centres (px along the column) of the hatch lines crossing column `x`.
fn line_centres(img: &RgbaImage, x: u32, rows: std::ops::Range<u32>) -> Vec<f64> {
    let mut out = Vec::new();
    let mut run: Option<(u32, f64, f64)> = None;
    for y in rows {
        let px = img.pixel(x, y);
        let w = f64::from(px[2]) / 255.0;
        if px[2] > 60 {
            let r = run.get_or_insert((y, 0.0, 0.0));
            r.1 += w;
            r.2 += w * (f64::from(y) + 0.5);
        } else if let Some((_, sum, moment)) = run.take() {
            out.push(moment / sum);
        }
    }
    out
}

/// Spacing (px, perpendicular to the lines) of the hatch lines at `angle` on a view of the cap
/// from +Z, measured along the central column.
fn measured_spacing(img: &RgbaImage, angle: f64) -> (f64, f64, f64, usize) {
    let h = img.size[1];
    let centres = line_centres(img, img.size[0] / 2, h / 8..h - h / 8);
    let gaps: Vec<f64> = centres
        .windows(2)
        .map(|w| (w[1] - w[0]) * angle.cos())
        .collect();
    let mean = gaps.iter().sum::<f64>() / gaps.len().max(1) as f64;
    let min = gaps.iter().copied().fold(f64::INFINITY, f64::min);
    let max = gaps.iter().copied().fold(0.0, f64::max);
    (mean, min, max, gaps.len())
}

/// Done-when 4: hatch spacing is 8 ± 1 px at zooms 0.01×–100×. Two families: the world scale
/// from 0.01× to 100× with the view framing the cap, and the camera zoomed into a fixed sphere
/// from 1× to 100×; orthographic and perspective; lines at 0° and 30°.
#[test]
fn gpu_hatch_spacing_is_constant_on_screen() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [256, 256];
    let mut cases = Vec::new();
    for scale in [0.01, 0.1, 1.0, 10.0, 100.0] {
        cases.push((format!("scale {scale}x ortho"), scale, 1.0, false));
        cases.push((format!("scale {scale}x perspective"), scale, 1.0, true));
    }
    for zoom in [3.0, 10.0, 100.0] {
        cases.push((format!("zoom {zoom}x ortho"), 1.0, zoom, false));
        cases.push((format!("zoom {zoom}x perspective"), 1.0, zoom, true));
    }
    for angle in [0.0f64, 30f64.to_radians()] {
        for (name, scale, zoom, perspective) in &cases {
            let radius = 10.0 * scale;
            let cam = if *perspective {
                // Eye at 4 r over the plane; zooming moves it closer (target stays on the cap).
                let h = 4.0 * radius / zoom;
                Camera {
                    eye: Point3::new(0.0, 0.0, h),
                    target: Point3::ZERO,
                    up: Dir3::Y,
                    projection: Projection::Perspective {
                        fov_y: 2.0 * (1.25 * radius / zoom / h).atan(),
                    },
                }
            } else {
                Camera {
                    eye: Point3::new(0.0, 0.0, 10.0 * radius),
                    target: Point3::ZERO,
                    up: Dir3::Y,
                    projection: Projection::Orthographic {
                        height: 2.5 * radius / zoom,
                    },
                }
            };
            let mut r = renderer(&gpu, 1);
            add_body(
                &mut r,
                &gpu,
                &uv_sphere(radius as f32, 128, 64, true),
                Some(RED),
            );
            r.scene_mut().add_section(cut_z(Hatch {
                angle,
                spacing_px: 8.0,
                color: BLUE,
                cross: false,
            }));
            let view = add_view(&mut r, cam, size, style(VisualStyle::Shaded));
            let img = capture(&mut r, view);
            let (mean, min, max, n) = measured_spacing(&img, angle);
            eprintln!(
                "{name}, {:.0}°: {n} gaps, mean {mean:.3} px, min {min:.3}, max {max:.3}",
                angle.to_degrees()
            );
            assert!(n >= 10, "{name}: too few hatch lines ({n})");
            assert!(
                (7.0..=9.0).contains(&min) && (7.0..=9.0).contains(&max),
                "{name}: spacing {min:.3}..{max:.3} px"
            );
        }
    }
}

/// A thick tube (outer r 40, inner r 20, height 100 along Z), closed and outward wound: the
/// inner wall faces the axis.
fn tube(segs: u32) -> MeshData {
    let mut positions = Vec::new();
    for i in 0..segs {
        let t = std::f32::consts::TAU * i as f32 / segs as f32;
        let (s, c) = t.sin_cos();
        for (r, z) in [(40.0, -50.0), (40.0, 50.0), (20.0, -50.0), (20.0, 50.0)] {
            positions.push([r * c, r * s, z]);
        }
    }
    let v = |i: u32, k: u32| 4 * (i % segs) + k;
    let mut indices = Vec::new();
    for i in 0..segs {
        let j = i + 1;
        // Outer wall (outward), inner wall (toward the axis), top and bottom rings.
        indices.extend([v(i, 0), v(j, 0), v(j, 1), v(i, 0), v(j, 1), v(i, 1)]);
        indices.extend([v(i, 2), v(i, 3), v(j, 3), v(i, 2), v(j, 3), v(j, 2)]);
        indices.extend([v(i, 1), v(j, 1), v(j, 3), v(i, 1), v(j, 3), v(i, 3)]);
        indices.extend([v(i, 0), v(i, 2), v(j, 2), v(i, 0), v(j, 2), v(j, 0)]);
    }
    MeshData {
        origin: Point3::ZERO,
        positions,
        normals: Vec::new(),
        indices,
        closed: true,
    }
}

/// The back-face cap is tested against the body's own depth first, so the bore of a tube cut
/// lengthwise shows its inner wall, not a cap in front of it; the cut wall shows the cap.
#[test]
fn gpu_tube_cut_lengthwise_caps_the_wall_and_not_the_bore() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [200, 200];
    let cam = Camera {
        eye: Point3::new(0.0, 300.0, 0.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Orthographic { height: 120.0 },
    };
    for samples in [1, 4] {
        let mut r = renderer(&gpu, samples);
        add_body(&mut r, &gpu, &tube(128), Some(RED));
        r.scene_mut()
            .add_section(section(Point3::ZERO, Vec3::Y, Hatch::NONE));
        let view = add_view(&mut r, cam, size, style(VisualStyle::ShadedVisibleEdges));
        let img = capture(&mut r, view);
        let (mut wall, mut bore) = (0, 0);
        for y in 30..170u32 {
            for x in 0..size[0] {
                // View x = world −X here; |x| is what matters.
                let wx = (f64::from(x) + 0.5 - 100.0) * 120.0 / 200.0;
                let px = img.pixel(x, y);
                if (23.0..37.0).contains(&wx.abs()) {
                    wall += 1;
                    assert!(red(px), "x{samples}: wall at {x},{y} is not cap: {px:?}");
                } else if wx.abs() < 17.0 {
                    bore += 1;
                    assert!(
                        !red(px) && px[1] > 30,
                        "x{samples}: bore at {x},{y} shows {px:?}, not the inner wall"
                    );
                }
            }
        }
        eprintln!("x{samples}: {wall} wall pixels capped, {bore} bore pixels show the inner wall");

        // A slab |y| <= 10: through the bore the ray leaves the slab in the air, then meets the
        // inner wall (a front face past the exit, drawn as an occluder) before the outer wall's
        // back face, so the bore shows no cap; the wall is capped on the y = 10 plane.
        let mut r = renderer(&gpu, samples);
        add_body(&mut r, &gpu, &tube(128), Some(RED));
        r.scene_mut()
            .add_section(section(Point3::new(0.0, 10.0, 0.0), Vec3::Y, Hatch::NONE));
        r.scene_mut()
            .add_section(section(Point3::new(0.0, -10.0, 0.0), -Vec3::Y, Hatch::NONE));
        let view = add_view(&mut r, cam, size, style(VisualStyle::Shaded));
        let img = capture(&mut r, view);
        let (mut wall, mut bore) = (0, 0);
        for y in 30..170u32 {
            for x in 0..size[0] {
                let wx = (f64::from(x) + 0.5 - 100.0) * 120.0 / 200.0;
                let px = img.pixel(x, y);
                if (25.0..37.0).contains(&wx.abs()) {
                    wall += 1;
                    assert!(
                        red(px),
                        "slab x{samples}: wall at {x},{y} is not cap: {px:?}"
                    );
                } else if wx.abs() < 17.0 {
                    bore += 1;
                    assert!(!red(px), "slab x{samples}: bore at {x},{y} shows a cap");
                }
            }
        }
        eprintln!("slab x{samples}: {wall} wall pixels capped, {bore} bore pixels uncapped");
    }
}

fn polylines(lines: &[Vec<Point3>]) -> EdgeData {
    let mut data = EdgeData::default();
    for (i, pts) in lines.iter().enumerate() {
        data.polylines.push(Polyline {
            first: data.points.len() as u32,
            len: pts.len() as u32,
            id: i as u32,
            closed: false,
        });
        data.points
            .extend(pts.iter().map(|p| p.as_vec3().to_array()));
    }
    data
}

fn cube_corner(i: u32) -> Point3 {
    let bit = |b: u32| if (i >> b) & 1 == 1 { 50.0 } else { -50.0 };
    Point3::new(bit(0), bit(1), bit(2))
}

fn cube_edge_lines() -> Vec<Vec<Point3>> {
    let mut out = Vec::new();
    for i in 0..8u32 {
        for b in 0..3 {
            if (i >> b) & 1 == 0 {
                out.push(vec![cube_corner(i), cube_corner(i | (1 << b))]);
            }
        }
    }
    out
}

fn yellow(px: [u8; 4]) -> bool {
    px[0] > 150 && px[1] > 150 && px[2] < 80
}

fn cube_camera() -> Camera {
    Camera {
        eye: Point3::new(260.0, -320.0, 220.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective {
            fov_y: 40f64.to_radians(),
        },
    }
}

/// Fraction of the samples along `a`–`b` (ends trimmed) whose pixel satisfies `f`.
fn fraction_along(
    img: &RgbaImage,
    cam: &Camera,
    a: Point3,
    b: Point3,
    f: impl Fn([u8; 4]) -> bool,
) -> f64 {
    let (mut hit, mut n) = (0, 0);
    for k in 2..=38 {
        let p = cam
            .project(a.lerp(b, f64::from(k) / 40.0), img.size)
            .unwrap();
        let (x, y) = (p[0].floor(), p[1].floor());
        if x < 0.0 || y < 0.0 || x >= f64::from(img.size[0]) || y >= f64::from(img.size[1]) {
            continue;
        }
        n += 1;
        if f(img.pixel(x as u32, y as u32)) {
            hit += 1;
        }
    }
    f64::from(hit) / f64::from(n.max(1))
}

/// Edges and points honour the planes: the parts above z = 0 are gone, the kept parts stay, and
/// a bottom edge behind the cap is hidden by the cap's depth (the cap depth pass). The triangle
/// wireframe is clipped too. Edges of an instance with `sectioned == false` are not clipped.
#[test]
fn gpu_edges_points_and_wireframe_are_clipped() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [256, 256];
    let cam = cube_camera();
    let build = |r: &mut Renderer, sectioned: bool| {
        let key = r
            .scene_mut()
            .insert_mesh(upload_mesh(&gpu, &cube_data(), &JobCtx::detached()).unwrap());
        let e = upload_edges(&gpu, &polylines(&cube_edge_lines()), &JobCtx::detached()).unwrap();
        let p = upload_points(
            &gpu,
            &PointData {
                origin: Point3::ZERO,
                points: vec![[0.0, -50.0, 40.0], [0.0, -50.0, -40.0]],
                ids: Vec::new(),
            },
            &JobCtx::detached(),
        )
        .unwrap();
        let mut inst = Instance::new(key, Transform::IDENTITY);
        inst.edges = Some(r.scene_mut().insert_edges(e));
        inst.points = Some(r.scene_mut().insert_points(p));
        inst.material.base_color = [0.3, 0.3, 0.35, 1.0];
        inst.material.cap_color = Some([0.0, 0.0, 0.8]);
        inst.edge_style = LineStyle {
            width_px: 3.0,
            color: [1.0, 1.0, 0.0, 1.0],
            ..LineStyle::default()
        };
        inst.point_style = PointStyle {
            size_px: 9.0,
            color: [1.0, 1.0, 0.0, 1.0],
            ..PointStyle::default()
        };
        inst.mesh_wireframe = true;
        inst.sectioned = sectioned;
        r.scene_mut().add_instance(inst);
        r.scene_mut().add_section(cut_z(Hatch::NONE));
    };
    // The top front edge (z = 50), the front vertical edge, and a bottom edge seen through the cap.
    let top = (
        Point3::new(50.0, -50.0, 50.0),
        Point3::new(-50.0, -50.0, 50.0),
    );
    let vertical_hi = (
        Point3::new(50.0, -50.0, 5.0),
        Point3::new(50.0, -50.0, 48.0),
    );
    let vertical_lo = (
        Point3::new(50.0, -50.0, -48.0),
        Point3::new(50.0, -50.0, -5.0),
    );
    // Seen through the cap for x < 2 (the sight line crosses z = 0 inside the cap square).
    let behind_cap = (
        Point3::new(-50.0, 50.0, -50.0),
        Point3::new(-5.0, 50.0, -50.0),
    );
    for samples in [1, 4] {
        let mut r = renderer(&gpu, samples);
        build(&mut r, true);
        let view = add_view(&mut r, cam, size, style(VisualStyle::ShadedVisibleEdges));
        let img = capture(&mut r, view);
        let f = |(a, b): (Point3, Point3)| fraction_along(&img, &cam, a, b, yellow);
        let (t, hi, lo, back) = (f(top), f(vertical_hi), f(vertical_lo), f(behind_cap));
        eprintln!(
            "x{samples}: top {t:.2}, vertical above {hi:.2}, below {lo:.2}, behind cap {back:.2}"
        );
        assert!(t < 0.01 && hi < 0.01, "edges above the plane are clipped");
        assert!(lo > 0.95, "the kept part of an edge is drawn");
        assert!(
            back < 0.01,
            "an edge behind the cap is hidden by the cap depth"
        );
        let point = |p: Point3| {
            let q = cam.project(p, size).unwrap();
            img.pixel(q[0] as u32, q[1] as u32)
        };
        assert!(
            !yellow(point(Point3::new(0.0, -50.0, 40.0))),
            "point above is clipped"
        );
        assert!(
            yellow(point(Point3::new(0.0, -50.0, -40.0))),
            "point below is drawn"
        );

        // Triangle wireframe (or its feature-edge fallback) is clipped as well.
        r.view_mut(view).unwrap().style = style(VisualStyle::Wireframe);
        let wire = capture(&mut r, view);
        let lit_above = fraction_along(&wire, &cam, top.0, top.1, |px| {
            px[0].max(px[1]).max(px[2]) > 60
        });
        // One sample may fall where a kept (hidden) wireframe line crosses the edge's path.
        assert!(
            lit_above <= 0.06,
            "wireframe above the plane is clipped ({lit_above:.2})"
        );

        // Not sectioned: nothing is clipped.
        let mut r = renderer(&gpu, samples);
        build(&mut r, false);
        let view = add_view(&mut r, cam, size, style(VisualStyle::ShadedVisibleEdges));
        let img = capture(&mut r, view);
        let t = fraction_along(&img, &cam, top.0, top.1, yellow);
        assert!(
            t > 0.95,
            "an unsectioned instance keeps its top edge ({t:.2})"
        );
    }
}

/// Switches: a disabled plane, a view with `sections_enabled == false` and an instance with
/// `sectioned == false` clip nothing; removing the plane restores the body; six planes make a
/// box section whose three visible caps cover the projected inner box.
#[test]
fn gpu_section_switches_and_a_six_plane_box() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [128, 128];
    let count = |img: &RgbaImage| {
        img.pixels
            .chunks_exact(4)
            .filter(|p| red([p[0], p[1], p[2], p[3]]))
            .count()
    };
    let mut r = renderer(&gpu, 4);
    add_body(&mut r, &gpu, &uv_sphere(10.0, 96, 48, true), Some(RED));
    let key = r.scene_mut().add_section(cut_z(Hatch::NONE));
    let view = add_view(&mut r, top_ortho(25.0), size, style(VisualStyle::Shaded));
    let on = count(&capture(&mut r, view));
    r.scene_mut().section_mut(key).unwrap().enabled = false;
    let disabled = count(&capture(&mut r, view));
    r.scene_mut().section_mut(key).unwrap().enabled = true;
    r.view_mut(view).unwrap().sections_enabled = false;
    let view_off = count(&capture(&mut r, view));
    r.view_mut(view).unwrap().sections_enabled = true;
    assert!(r.scene_mut().remove_section(key));
    let removed = count(&capture(&mut r, view));
    eprintln!("cap pixels: on {on}, disabled {disabled}, view off {view_off}, removed {removed}");
    assert!(on > 3000);
    assert_eq!((disabled, view_off, removed), (0, 0, 0));

    // Box section: keep |x|, |y|, |z| <= 30 of the 100 mm cube.
    let cam = corner_view();
    let mut r = renderer(&gpu, 4);
    add_body(&mut r, &gpu, &cube_data(), Some(RED));
    for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
        for s in [1.0, -1.0] {
            r.scene_mut()
                .add_section(section(axis * (30.0 * s), axis * s, Hatch::NONE));
        }
    }
    assert_eq!(r.scene().active_sections().len(), 6);
    let view = add_view(&mut r, cam, [256, 256], style(VisualStyle::Shaded));
    let img = capture(&mut r, view);
    // The projected inner box: convex hull of its corners.
    let mut pts: Vec<[f64; 2]> = (0..8u32)
        .map(|i| {
            let c = cube_corner(i) * 0.6;
            let q = cam.project(c, img.size).unwrap();
            [q[0], q[1]]
        })
        .collect();
    let hull = convex_hull(&mut pts);
    let (mut inside, mut outside_cap) = (0, 0);
    for y in 0..256 {
        for x in 0..256 {
            let p = [f64::from(x) + 0.5, f64::from(y) + 0.5];
            let d = inside_distance(&hull, p);
            let px = img.pixel(x, y);
            if d > 1.5 {
                inside += 1;
                assert!(red(px), "box cap missing at {x},{y}: {px:?}");
            } else if d < -1.5 && red(px) {
                outside_cap += 1;
            }
        }
    }
    eprintln!("box section: {inside} cap pixels inside the projected box, {outside_cap} outside");
    assert!(inside > 5000);
    assert_eq!(outside_cap, 0);
}

fn convex_hull(pts: &mut [[f64; 2]]) -> Vec<[f64; 2]> {
    pts.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    };
    let mut hull: Vec<[f64; 2]> = Vec::new();
    for pass in 0..2 {
        let start = hull.len();
        let iter: Box<dyn Iterator<Item = &[f64; 2]>> = if pass == 0 {
            Box::new(pts.iter())
        } else {
            Box::new(pts.iter().rev())
        };
        for &p in iter {
            while hull.len() >= start + 2
                && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0
            {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    hull
}

/// With the depth pre-pass, the clipped depth-only pipelines give the same image as the single
/// pass; silhouettes over caps decode the cap marks (the cap interior has no outline).
#[test]
fn gpu_sections_with_depth_prepass_and_silhouettes() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [128, 128];
    let mut images = Vec::new();
    for prepass in [false, true] {
        let mut r = Renderer::new(
            Arc::clone(&gpu),
            &RendererConfig {
                msaa_samples: 4,
                depth_prepass: prepass,
                ..Default::default()
            },
        )
        .unwrap();
        add_body(&mut r, &gpu, &cube_data(), Some(RED));
        r.scene_mut()
            .add_section(section(Point3::ZERO, Vec3::X, Hatch::NONE));
        let view = add_view(&mut r, corner_view(), size, style(VisualStyle::Shaded));
        images.push(capture(&mut r, view));
        if !prepass {
            let mut s = style(VisualStyle::Shaded);
            s.silhouettes = true;
            s.silhouette_color = [1.0, 1.0, 1.0];
            r.view_mut(view).unwrap().style = s;
            let img = capture(&mut r, view);
            // The centre of the x = 0 cap.
            let c = corner_view()
                .project(Point3::new(0.0, -25.0, 0.0), size)
                .unwrap();
            let px = img.pixel(c[0] as u32, c[1] as u32);
            assert!(red(px), "cap interior has no outline: {px:?}");
        }
    }
    let diff = images[0]
        .pixels
        .iter()
        .zip(&images[1].pixels)
        .filter(|(a, b)| a.abs_diff(**b) > 2)
        .count();
    eprintln!("pre-pass vs single pass: {diff} channel values differ by more than 2");
    assert!(diff < 40, "{diff}");
}
