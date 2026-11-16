//! RND-07 GPU tests: overlay labels, constant pixel size of arrows and rings across view
//! distances, handle picking, the adaptive grid spacing, image-plane orientation, depth modes
//! against the scene and insets. Named `gpu_*` (nextest test-group `gpu`, serial); they skip
//! without an adapter and fail under `AMG_REQUIRE_GPU=1` (CI runs them on WARP).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::collections::HashMap;
use std::sync::Arc;

use amg_core::{Dir3, Plane, Point3, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::{
    Background, Camera, DepthMode, HighlightItem, HighlightLayer, Instance, LabelStyle, Lighting,
    LineStyle, MeshData, OverlayInset, PickFilter, PickHit, PickId, PickRequest, PickResult,
    PointStyle, Projection, Range32, Renderer, RendererConfig, RgbaImage, SubKind, ViewId,
    ViewStyle, VisualStyle, upload_image, upload_mesh,
};

const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];

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

fn add_view(r: &mut Renderer, camera: Camera, size: [u32; 2]) -> ViewId {
    let view = r.create_view(size);
    let v = r.view_mut(view).unwrap();
    v.camera = camera;
    v.style = ViewStyle {
        background: Background::Solid([0.0, 0.0, 0.0]),
        lighting: Lighting::Flat,
        visual: VisualStyle::Shaded,
        ..Default::default()
    };
    assert!(r.prepare_view(view), "pipeline variants did not finish");
    view
}

fn capture(r: &mut Renderer, view: ViewId) -> RgbaImage {
    assert!(r.prepare_view(view), "pipeline variants did not finish");
    r.capture(view).unwrap().wait().unwrap()
}

fn line(width_px: f32, color: [f32; 4], depth: DepthMode) -> LineStyle {
    LineStyle {
        width_px,
        color,
        dash: None,
        depth,
    }
}

/// Small deterministic generator (xorshift64*), so the random cameras are reproducible.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next()
    }

    fn point(&mut self, r: f64) -> Point3 {
        Point3::new(self.range(-r, r), self.range(-r, r), self.range(-r, r))
    }
}

/// A square of side `2 h` in the plane y = `y` (x, z in `[-h, h]`), seen by the front cameras.
fn wall(y: f64, h: f64) -> MeshData {
    let p = |x: f64, z: f64| [x as f32, y as f32, z as f32];
    MeshData {
        origin: Point3::ZERO,
        positions: vec![p(-h, -h), p(h, -h), p(h, h), p(-h, h)],
        normals: Vec::new(),
        indices: vec![0, 1, 2, 0, 2, 3],
        closed: false,
    }
}

fn add_wall(r: &mut Renderer, gpu: &Gpu, y: f64, h: f64, color: [f32; 4], pick: u32) {
    let mesh = upload_mesh(gpu, &wall(y, h), &JobCtx::detached()).unwrap();
    let key = r.scene_mut().insert_mesh(mesh);
    let mut inst = Instance::new(key, Transform::IDENTITY);
    inst.material.base_color = color;
    inst.pick = PickId(pick);
    r.scene_mut().add_instance(inst);
}

/// Orthographic camera on −Y: view x = world X, view y = world Z.
fn front_ortho(height: f64) -> Camera {
    Camera {
        eye: Point3::new(0.0, -500.0, 0.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Orthographic { height },
    }
}

fn lum(img: &RgbaImage, x: u32, y: u32) -> u8 {
    let p = img.pixel(x, y);
    p[0].max(p[1]).max(p[2])
}

/// Done-when 1: label anchors are projected exactly like `Camera::project` (the text itself is
/// egui-rasterised: `egui_glue` unit test `paint_labels_emits_egui_text_at_the_projected_positions`).
#[test]
fn gpu_projected_labels_match_camera_project_for_20_random_cameras() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 1);
    let size = [640, 480];
    let view = add_view(&mut r, front_ortho(100.0), size);
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    let mut worst = 0.0f64;
    let mut checked = 0;
    for i in 0..20 {
        let target = rng.point(1e3);
        let dir = Vec3::new(
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(0.2, 1.0),
        )
        .normalize();
        let distance = 10f64.powf(rng.range(-1.0, 4.0));
        let projection = if i % 3 == 2 {
            Projection::Orthographic {
                height: distance * rng.range(0.3, 2.0),
            }
        } else {
            Projection::Perspective {
                fov_y: rng.range(20.0, 70.0).to_radians(),
            }
        };
        let cam = Camera {
            eye: target + dir * distance,
            target,
            up: Dir3::Z,
            projection,
        };
        r.view_mut(view).unwrap().camera = cam;
        let anchors: Vec<Point3> = (0..10)
            .map(|_| {
                target
                    + Vec3::new(
                        rng.range(-1.0, 1.0),
                        rng.range(-1.0, 1.0),
                        rng.range(-1.0, 1.0),
                    ) * distance
                        * 0.3
            })
            .collect();
        let overlays = r.scene_mut().begin_overlays();
        for (k, a) in anchors.iter().enumerate() {
            overlays.label(*a, format!("{k}"), LabelStyle::default());
        }
        let labels = r.project_labels(view);
        let expected: Vec<(usize, [f64; 3])> = anchors
            .iter()
            .enumerate()
            .filter_map(|(k, a)| cam.project(*a, size).map(|p| (k, p)))
            .collect();
        assert_eq!(labels.len(), expected.len(), "camera {i}");
        for (label, (k, p)) in labels.iter().zip(&expected) {
            assert_eq!(label.text, format!("{k}"));
            let d = (f64::from(label.pos_px[0]) - p[0]).hypot(f64::from(label.pos_px[1]) - p[1]);
            worst = worst.max(d);
            checked += 1;
            assert!((f64::from(label.depth) - p[2]).abs() < 1e-6);
        }
    }
    eprintln!("{checked} labels over 20 cameras, worst deviation {worst:.2e} px");
    assert!(checked >= 150);
    assert!(worst <= 0.5);

    // Inset labels are offset by the inset rectangle.
    let inset_cam = front_ortho(2.0);
    let mut list = amg_render::OverlayList::new();
    list.label(Point3::ZERO, "F", LabelStyle::default());
    r.view_mut(view).unwrap().insets = vec![OverlayInset {
        rect_px: [500, 20, 101, 101],
        camera: inset_cam,
        overlays: list,
    }];
    r.scene_mut().begin_overlays();
    let labels = r.project_labels(view);
    assert_eq!(labels.len(), 1);
    let [lx, ly] = labels[0].pos_px;
    assert!(
        (lx - 550.5).abs() < 1e-3 && (ly - 70.5).abs() < 1e-3,
        "{:?}",
        labels[0].pos_px
    );
}

/// Done-when 2: arrows and rings keep their requested pixel size from 1e-2 to 1e5 mm.
#[test]
fn gpu_arrow_and_ring_sizes_hold_across_view_distances() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 1);
    // Odd size: the target projects onto a pixel centre (200.5, 150.5).
    let size = [401, 301];
    let target = Point3::new(1000.0, -500.0, 200.0);
    let view = add_view(&mut r, front_ortho(10.0), size);
    let (cx, cy) = (200usize, 150usize);
    let mut worst_arrow = 0.0f64;
    let mut worst_ring = 0.0f64;
    for e in -2..=5 {
        let distance = 10f64.powi(e);
        let cam = Camera {
            eye: target + Vec3::new(0.0, -distance, 0.0),
            target,
            up: Dir3::Z,
            projection: Projection::Perspective {
                fov_y: 45f64.to_radians(),
            },
        };
        r.view_mut(view).unwrap().camera = cam;
        // Arrow along screen +x (world +X), 80 px.
        let overlays = r.scene_mut().begin_overlays();
        overlays.arrow(
            target,
            Dir3::X,
            80.0,
            line(1.0, WHITE, DepthMode::AlwaysOnTop),
            None,
        );
        let img = capture(&mut r, view);
        let last = (cx..size[0] as usize)
            .rev()
            .find(|&x| lum(&img, x as u32, cy as u32) > 64)
            .unwrap();
        // The tip lies in the last lit pixel (the axis runs through the pixel centres).
        let length = (last as f64 + 1.0) - 200.5;
        eprintln!("{distance} mm: arrow {length} px");
        worst_arrow = worst_arrow.max((length - 80.0).abs());
        assert!(
            (length - 80.0).abs() <= 1.0,
            "arrow at {distance} mm: {length} px"
        );
        // Ring facing the camera, radius 40 px: centre line from the coverage centroid.
        let overlays = r.scene_mut().begin_overlays();
        overlays.ring(
            target,
            Dir3::NEG_Y,
            40.0,
            line(1.5, WHITE, DepthMode::AlwaysOnTop),
            None,
        );
        let img = capture(&mut r, view);
        let centroid = |values: &[(f64, f64)]| {
            let w: f64 = values.iter().map(|v| v.1).sum();
            values.iter().map(|v| v.0 * v.1).sum::<f64>() / w
        };
        let mut radii = Vec::new();
        for (dx, dy) in [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)] {
            let samples: Vec<(f64, f64)> = (30..=50)
                .map(|t| {
                    let x = (cx as i64 + dx * t) as u32;
                    let y = (cy as i64 + dy * t) as u32;
                    (t as f64, f64::from(lum(&img, x, y)))
                })
                .collect();
            radii.push(centroid(&samples));
        }
        eprintln!("{distance} mm: ring radii {radii:?} px");
        for &radius in &radii {
            worst_ring = worst_ring.max((radius - 40.0).abs());
            assert!(
                (radius - 40.0).abs() <= 1.0,
                "ring at {distance} mm: radius {radius} px ({radii:?})"
            );
        }
    }
    eprintln!("worst arrow length error {worst_arrow:.3} px, ring radius {worst_ring:.3} px");
}

fn pick(r: &mut Renderer, view: ViewId, px: [f64; 2], filter: PickFilter) -> Vec<PickHit> {
    let ticket = r.request_pick(
        view,
        PickRequest {
            filter,
            ..PickRequest::point([px[0] as f32, px[1] as f32])
        },
    );
    let mut got: HashMap<_, _> = HashMap::new();
    for _ in 0..3 {
        r.capture(view).unwrap().wait().unwrap();
        got.extend(r.poll_picks());
        if got.contains_key(&ticket) {
            break;
        }
    }
    match got.remove(&ticket).expect("answered").unwrap() {
        PickResult::Point(h) => h,
        PickResult::Rect(_) => panic!("point result"),
    }
}

/// Done-when 3: overlay handles are picked as `SubKind::Handle` with their id.
#[test]
fn gpu_overlay_handles_pick_as_handle_with_their_id() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 4);
    let size = [400, 300];
    // 1 mm per pixel; view x = world X, view y = −world Z.
    let cam = front_ortho(300.0);
    add_wall(&mut r, &gpu, 0.0, 120.0, [0.5, 0.5, 0.5, 1.0], 9);
    let view = add_view(&mut r, cam, size);
    let px = |p: Point3| {
        let q = cam.project(p, size).unwrap();
        [q[0], q[1]]
    };
    let fill = |r: &mut Renderer| {
        let o = r.scene_mut().begin_overlays();
        // An always-on-top arrow in front of the wall: shaft and cone are handle (50, 7).
        o.arrow(
            Point3::new(-100.0, -10.0, 60.0),
            Dir3::X,
            80.0,
            line(2.0, WHITE, DepthMode::AlwaysOnTop),
            Some((PickId(50), 7)),
        );
        // A point handle (51, 3) and a plane quad handle (52, 9), depth tested, in front.
        o.points(
            &[Point3::new(60.0, -20.0, 60.0)],
            PointStyle {
                size_px: 8.0,
                color: WHITE,
                depth: DepthMode::Test,
            },
            Some((PickId(51), 3)),
        );
        o.plane_quad(
            &Plane {
                origin: Point3::new(0.0, -30.0, -60.0),
                x: Dir3::X,
                y: Dir3::Z,
                normal: Dir3::NEG_Y,
            },
            [20.0, 20.0],
            [0.2, 0.4, 0.9, 0.5],
            line(1.0, WHITE, DepthMode::Test),
            Some((PickId(52), 9)),
        );
        // A depth-tested handle behind the wall: not pickable through it.
        o.polyline(
            &[
                Point3::new(60.0, 50.0, -60.0),
                Point3::new(100.0, 50.0, -60.0),
            ],
            line(3.0, WHITE, DepthMode::Test),
            Some((PickId(53), 1)),
        );
        // A non-pickable polyline.
        o.polyline(
            &[
                Point3::new(-100.0, -10.0, -100.0),
                Point3::new(-40.0, -10.0, -100.0),
            ],
            line(3.0, WHITE, DepthMode::AlwaysOnTop),
            None,
        );
    };
    let check = |r: &mut Renderer, at: Point3, filter: PickFilter| {
        fill(r);
        let hits = pick(r, view, px(at), filter);
        hits.first().map(|h| (h.pick, h.kind, h.sub, h.world))
    };
    let all = PickFilter::ALL;
    // Shaft middle, cone, point, quad interior.
    for (at, want) in [
        (Point3::new(-70.0, -10.0, 60.0), (PickId(50), 7)),
        (Point3::new(-30.0, -10.0, 60.0), (PickId(50), 7)),
        (Point3::new(60.0, -20.0, 60.0), (PickId(51), 3)),
        (Point3::new(5.0, -30.0, -55.0), (PickId(52), 9)),
    ] {
        let (pick, kind, sub, world) = check(&mut r, at, all).expect("a hit");
        assert_eq!(
            (pick, kind, sub),
            (want.0, SubKind::Handle, want.1),
            "at {at:?}"
        );
        // The cone surface lies up to its radius (7 mm) in front of or behind the axis.
        let tol = if at.x > -40.0 && at.x < -20.0 {
            7.5
        } else {
            1e-3
        };
        assert!((world.y - at.y).abs() < tol, "world {world:?} for {at:?}");
    }
    // Behind the wall: the wall face, not handle 53.
    let (pick, kind, ..) = check(&mut r, Point3::new(80.0, 50.0, -60.0), all).unwrap();
    assert_eq!((pick, kind), (PickId(9), SubKind::Face));
    // The non-pickable polyline and a filter without handles give the wall.
    let (pick, kind, ..) = check(&mut r, Point3::new(-70.0, -10.0, -100.0), all).unwrap();
    assert_eq!((pick, kind), (PickId(9), SubKind::Face));
    let faces_only = PickFilter::BODIES | PickFilter::FACES;
    let (pick, kind, ..) = check(&mut r, Point3::new(-70.0, -10.0, 60.0), faces_only).unwrap();
    assert_eq!((pick, kind), (PickId(9), SubKind::Face));
    // Handle highlight: hovering handle (50, 7) recolours the arrow.
    let before = capture(&mut r, view);
    r.scene_mut()
        .set_highlight_color(HighlightLayer::Hover, [1.0, 0.0, 0.0, 1.0]);
    r.scene_mut().set_highlight_layer(
        HighlightLayer::Hover,
        &[HighlightItem {
            pick: PickId(50),
            sub: Some((SubKind::Handle, Range32 { start: 7, end: 8 })),
        }],
    );
    fill(&mut r);
    let after = capture(&mut r, view);
    let shaft = px(Point3::new(-70.0, -10.0, 60.0));
    let (x, y) = (shaft[0] as u32, shaft[1] as u32);
    assert!(
        before.pixel(x, y)[1] > 200,
        "white before: {:?}",
        before.pixel(x, y)
    );
    let p = after.pixel(x, y);
    assert!(p[0] > 200 && p[1] < 40, "red when hovered: {p:?}");
}

/// Line centres (intensity-weighted, px) of runs of lit pixels along a scan line; runs longer
/// than `max_run` (lines along the scan direction) are dropped.
fn line_centres(values: &[f64], threshold: f64, max_run: usize) -> Vec<f64> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < values.len() {
        if values[i] <= threshold {
            i += 1;
            continue;
        }
        let start = i;
        while i < values.len() && values[i] > threshold {
            i += 1;
        }
        if i - start <= max_run && start > 0 && i < values.len() {
            let w: f64 = values[start..i].iter().sum();
            let c: f64 = (start..i).map(|k| k as f64 * values[k]).sum::<f64>() / w;
            out.push(c);
        }
    }
    out
}

/// Done-when 4: minor grid lines are never closer than 8 px.
#[test]
fn gpu_grid_minor_lines_are_never_closer_than_8_px() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 4);
    let size = [480, 360];
    let view = add_view(&mut r, front_ortho(1.0), size);
    // (camera, scan only the columns near the centre). In the oblique views the lines of constant
    // x converge, so only the centre columns, where they are near vertical (long runs that are
    // dropped), are scanned for the lines of constant y; rows see the converging lines only.
    let mut cases = Vec::new();
    // Top views (orthographic): heights over seven decades.
    for h in [0.05, 0.73, 3.3, 47.0, 120.0, 999.0, 8000.0] {
        cases.push((
            Camera {
                eye: Point3::new(3.7, 1.9, 500.0),
                target: Point3::new(3.7, 1.9, 0.0),
                up: Dir3::Y,
                projection: Projection::Orthographic { height: h },
            },
            false,
        ));
    }
    // Oblique perspective along +Y, 35-70° down, at several distances.
    for (d, pitch) in [(0.5, 35.0f64), (20.0, 50.0), (300.0, 60.0), (5e3, 70.0)] {
        let p = pitch.to_radians();
        let target = Point3::new(0.3, 0.1, 0.0);
        cases.push((
            Camera {
                eye: target + Vec3::new(0.0, -p.cos(), p.sin()) * d,
                target,
                up: Dir3::Z,
                projection: Projection::Perspective {
                    fov_y: 45f64.to_radians(),
                },
            },
            true,
        ));
    }
    let mut min_spacing = f64::INFINITY;
    let mut measured = 0usize;
    for (i, (cam, centre_columns)) in cases.iter().enumerate() {
        r.view_mut(view).unwrap().camera = *cam;
        let extent = cam.eye.distance(cam.target) * 50.0 + 1e4;
        r.scene_mut()
            .begin_overlays()
            .grid(&Plane::XY, [extent, extent], WHITE);
        let img = capture(&mut r, view);
        let at = |x: u32, y: u32| f64::from(lum(&img, x, y)) / 255.0;
        let threshold = 4.0 / 255.0;
        let mut case_min = f64::INFINITY;
        // Rows: crossings of the lines that run up the screen; columns: the lines across it.
        let scans: Vec<Vec<f64>> = (0..size[1])
            .step_by(7)
            .map(|y| (0..size[0]).map(|x| at(x, y)).collect())
            .chain(
                (0..size[0])
                    .step_by(7)
                    .filter(|&x| !centre_columns || x.abs_diff(size[0] / 2) <= 24)
                    .map(|x| (0..size[1]).map(|y| at(x, y)).collect()),
            )
            .collect();
        for scan in &scans {
            let lit = scan.iter().filter(|&&v| v > threshold).count();
            if lit * 3 > scan.len() {
                continue;
            }
            let centres = line_centres(scan, threshold, 4);
            for w in centres.windows(2) {
                let d = w[1] - w[0];
                case_min = case_min.min(d);
                measured += 1;
            }
        }
        eprintln!("case {i}: closest lines {case_min:.2} px");
        min_spacing = min_spacing.min(case_min);
    }
    eprintln!("{measured} spacings, closest {min_spacing:.2} px");
    assert!(measured > 1000);
    assert!(min_spacing >= 8.0, "grid lines {min_spacing} px apart");
}

/// Encodes `img` as a PNG and decodes it again (the PNG path an IO-10 canvas takes).
fn png_roundtrip(img: &RgbaImage) -> RgbaImage {
    let mut bytes = Vec::new();
    {
        let mut enc = png::Encoder::new(&mut bytes, img.size[0], img.size[1]);
        enc.set_color(png::ColorType::Rgba);
        enc.set_depth(png::BitDepth::Eight);
        let mut w = enc.write_header().unwrap();
        w.write_image_data(&img.pixels).unwrap();
    }
    let dec = png::Decoder::new(std::io::Cursor::new(bytes));
    let mut reader = dec.read_info().unwrap();
    let mut pixels = vec![0; reader.output_buffer_size().unwrap()];
    let info = reader.next_frame(&mut pixels).unwrap();
    assert_eq!(info.color_type, png::ColorType::Rgba);
    pixels.truncate(info.buffer_size());
    RgbaImage {
        size: [info.width, info.height],
        pixels,
    }
}

/// Done-when 5: an image plane shows a PNG the right way round (corner pixels).
#[test]
fn gpu_image_plane_shows_a_png_with_correct_orientation() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 4);
    let size = [200, 200];
    // Quadrants: top-left red, top-right green, bottom-left blue, bottom-right white.
    let n = 32u32;
    let mut pixels = Vec::with_capacity((4 * n * n) as usize);
    for y in 0..n {
        for x in 0..n {
            pixels.extend_from_slice(match (x < n / 2, y < n / 2) {
                (true, true) => &[255, 0, 0, 255],
                (false, true) => &[0, 255, 0, 255],
                (true, false) => &[0, 0, 255, 255],
                (false, false) => &[255, 255, 255, 255],
            });
        }
    }
    let img = png_roundtrip(&RgbaImage {
        size: [n, n],
        pixels,
    });
    let key = r
        .scene_mut()
        .insert_image(upload_image(&gpu, &img).unwrap());
    // 1 mm per pixel; the plane fills x, z in [-80, 80] seen from −Y.
    let cam = front_ortho(200.0);
    let view = add_view(&mut r, cam, size);
    let corners = [
        Point3::new(-80.0, 0.0, 80.0),
        Point3::new(80.0, 0.0, 80.0),
        Point3::new(80.0, 0.0, -80.0),
        Point3::new(-80.0, 0.0, -80.0),
    ];
    r.scene_mut()
        .begin_overlays()
        .image_plane(key, corners, 1.0);
    let shot = capture(&mut r, view);
    // 10 px inside each corner of the projected plane (pixels 20..180).
    for (x, y, want) in [
        (30, 30, [255, 0, 0]),
        (170, 30, [0, 255, 0]),
        (30, 170, [0, 0, 255]),
        (170, 170, [255, 255, 255]),
    ] {
        let p = shot.pixel(x, y);
        assert_eq!([p[0], p[1], p[2]], want, "pixel ({x}, {y})");
    }
    assert_eq!(&shot.pixel(5, 5)[..3], &[0, 0, 0], "outside the plane");
    // Half opacity blends with the background.
    r.scene_mut()
        .begin_overlays()
        .image_plane(key, corners, 0.5);
    let p = capture(&mut r, view).pixel(170, 170);
    assert!((150..=220).contains(&p[0]), "half-transparent white: {p:?}");
    // A removed image draws nothing.
    assert!(r.scene_mut().remove_image(key));
    r.scene_mut()
        .begin_overlays()
        .image_plane(key, corners, 1.0);
    assert_eq!(&capture(&mut r, view).pixel(30, 30)[..3], &[0, 0, 0]);
}

/// Depth modes against the scene: `Test` hides behind a face, `XRay` dims, `AlwaysOnTop` shows;
/// overlays occlude each other through their own depth.
#[test]
fn gpu_depth_modes_hide_dim_or_show_overlays_behind_the_scene() {
    let Some(gpu) = gpu() else { return };
    for samples in [1, 4] {
        let mut r = renderer(&gpu, samples);
        let size = [300, 200];
        add_wall(&mut r, &gpu, 0.0, 60.0, [0.0, 0.0, 0.0, 1.0], 1);
        let view = add_view(&mut r, front_ortho(200.0), size);
        let green = [0.0, 1.0, 0.0, 1.0];
        let row = |z: f64| [Point3::new(-50.0, 10.0, z), Point3::new(50.0, 10.0, z)];
        let o = r.scene_mut().begin_overlays();
        // Three lines behind the wall (y = 10 > 0), one per mode.
        o.polyline(&row(40.0), line(3.0, green, DepthMode::Test), None);
        o.polyline(
            &row(0.0),
            line(3.0, green, DepthMode::XRay { hidden_alpha: 0.25 }),
            None,
        );
        o.polyline(&row(-40.0), line(3.0, green, DepthMode::AlwaysOnTop), None);
        // An opaque quad in front of a line, both overlays (always on top of the scene).
        o.polyline(
            &[
                Point3::new(80.0, -20.0, -80.0),
                Point3::new(80.0, -20.0, 80.0),
            ],
            line(3.0, green, DepthMode::AlwaysOnTop),
            None,
        );
        o.plane_quad(
            &Plane {
                origin: Point3::new(80.0, -40.0, 0.0),
                x: Dir3::X,
                y: Dir3::Z,
                normal: Dir3::NEG_Y,
            },
            [10.0, 10.0],
            [1.0, 0.0, 0.0, 1.0],
            line(1.0, [1.0, 0.0, 0.0, 1.0], DepthMode::AlwaysOnTop),
            None,
        );
        let img = capture(&mut r, view);
        let g = |x: u32, y: u32| img.pixel(x, y)[1];
        // 1 mm per pixel: z = 40 → y = 60, z = 0 → 100, z = −40 → 140; x = 0 → 150.
        assert!(g(150, 60) < 5, "Test hidden: {:?}", img.pixel(150, 60));
        let xray = g(150, 100);
        // 25 % of linear green over black is sRGB 137.
        assert!((110..=165).contains(&xray), "XRay dimmed: {xray}");
        assert!(g(150, 140) > 240, "AlwaysOnTop: {:?}", img.pixel(150, 140));
        // The quad hides the vertical line behind it; above the quad the line shows.
        assert!(
            img.pixel(230, 100)[0] > 240 && img.pixel(230, 100)[1] < 10,
            "quad over line"
        );
        assert!(g(230, 60) > 240, "line above the quad");
        eprintln!("x{samples}: x-ray green {xray}");
    }
}

/// The view-cube hook: an inset draws its own list with its own camera inside its rectangle,
/// over the scene, and nowhere else.
#[test]
fn gpu_insets_draw_in_their_rectangle_with_their_camera() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 4);
    let size = [320, 240];
    add_wall(&mut r, &gpu, 0.0, 500.0, [0.05, 0.05, 0.05, 1.0], 1);
    let view = add_view(&mut r, front_ortho(100.0), size);
    // An orthographic inset camera; a unit cube face as an opaque polygon filling half the
    // inset, and an edge loop around it.
    let mut list = amg_render::OverlayList::new();
    let face = [
        Point3::new(-1.0, -1.0, -1.0),
        Point3::new(1.0, -1.0, -1.0),
        Point3::new(1.0, -1.0, 1.0),
        Point3::new(-1.0, -1.0, 1.0),
    ];
    list.polygon(
        &face,
        &[[0, 1, 2], [0, 2, 3]],
        [0.9, 0.9, 0.9, 1.0],
        DepthMode::Test,
        None,
    );
    let rect = [200, 10, 100, 100];
    r.view_mut(view).unwrap().insets = vec![OverlayInset {
        rect_px: rect,
        camera: Camera {
            eye: Point3::new(0.0, -10.0, 0.0),
            target: Point3::ZERO,
            up: Dir3::Z,
            projection: Projection::Orthographic { height: 4.0 },
        },
        overlays: list,
    }];
    r.scene_mut().begin_overlays();
    let img = capture(&mut r, view);
    // The face spans half the inset height: pixels 25..75 of the inset.
    let centre = img.pixel(250, 60);
    assert!(centre[0] > 200, "face in the inset: {centre:?}");
    let inside_bg = img.pixel(205, 15);
    let outside = img.pixel(150, 60);
    assert!(
        inside_bg[0] < 100 && outside[0] < 100,
        "{inside_bg:?} {outside:?}"
    );
    let lit: usize = (0..size[1])
        .flat_map(|y| (0..size[0]).map(move |x| (x, y)))
        .filter(|&(x, y)| img.pixel(x, y)[0] > 200)
        .map(|(x, y)| {
            assert!(
                (rect[0]..rect[0] + rect[2]).contains(&x)
                    && (rect[1]..rect[1] + rect[3]).contains(&y),
                "lit pixel ({x}, {y}) outside the inset"
            );
            1
        })
        .sum();
    assert!(
        (2200..=2800).contains(&lit),
        "{lit} face pixels (50 x 50 expected)"
    );
}
