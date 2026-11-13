//! RND-12 GPU tests: wide lines with round joins, the view-space depth offset across zoom
//! levels, hidden-edge dashes in the six visual styles, silhouettes, point sprites and the
//! triangle wireframe with its feature-edge fallback. Named `gpu_*` (nextest test-group `gpu`,
//! serial); they skip without an adapter and fail under `AMG_REQUIRE_GPU=1` (CI runs them on
//! WARP).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::collections::{HashSet, VecDeque};
use std::sync::Arc;

use amg_core::{Dir3, Point3, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::{
    Background, Camera, DepthMode, EdgeData, Instance, Lighting, LineStyle, MeshData, PointData,
    PointStyle, Polyline, Projection, Renderer, RendererConfig, RgbaImage, ViewId, ViewStyle,
    VisualStyle, upload_edges, upload_mesh, upload_points,
};

const BLACK: [f32; 3] = [0.0, 0.0, 0.0];
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

fn style(visual: VisualStyle) -> ViewStyle {
    ViewStyle {
        background: Background::Solid(BLACK),
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

/// Orthographic camera on −Y: view x = world X, view y = world Z.
fn front_ortho(height: f64) -> Camera {
    Camera {
        eye: Point3::new(0.0, -500.0, 0.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Orthographic { height },
    }
}

fn edges(points: Vec<Point3>, closed: bool) -> EdgeData {
    polylines(&[(points, closed)])
}

fn polylines(lines: &[(Vec<Point3>, bool)]) -> EdgeData {
    let mut data = EdgeData::default();
    for (i, (pts, closed)) in lines.iter().enumerate() {
        data.polylines.push(Polyline {
            first: data.points.len() as u32,
            len: pts.len() as u32,
            id: i as u32,
            closed: *closed,
        });
        data.points
            .extend(pts.iter().map(|p| p.as_vec3().to_array()));
    }
    data
}

fn line_style(width_px: f32, color: [f32; 4]) -> LineStyle {
    LineStyle {
        width_px,
        color,
        ..LineStyle::default()
    }
}

/// Sample points every `step_px` (at most) along each segment of `pts`, skipping `skip` of the
/// length at both ends of every segment.
fn samples_along(
    cam: &Camera,
    size: [u32; 2],
    pts: &[Point3],
    closed: bool,
    skip: f64,
) -> Vec<[f64; 2]> {
    let mut out = Vec::new();
    let n = if closed { pts.len() } else { pts.len() - 1 };
    for i in 0..n {
        let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
        let pa = cam.project(a, size).unwrap();
        let pb = cam.project(b, size).unwrap();
        let len_px = ((pb[0] - pa[0]).hypot(pb[1] - pa[1])).max(1.0);
        let steps = (len_px * 2.0).ceil() as usize;
        for k in 0..=steps {
            let t = k as f64 / steps as f64;
            if t < skip || t > 1.0 - skip {
                continue;
            }
            let p = cam.project(a.lerp(b, t), size).unwrap();
            out.push([p[0], p[1]]);
        }
    }
    out
}

fn pixel_at(img: &RgbaImage, p: [f64; 2]) -> Option<[u8; 4]> {
    let (x, y) = (p[0].floor(), p[1].floor());
    (x >= 0.0 && y >= 0.0 && x < f64::from(img.size[0]) && y < f64::from(img.size[1]))
        .then(|| img.pixel(x as u32, y as u32))
}

fn lit(px: [u8; 4]) -> bool {
    px[0].max(px[1]).max(px[2]) > 60
}

/// 8-connected components of the lit pixels reached from `start`.
fn reachable(img: &RgbaImage, start: (u32, u32)) -> HashSet<(u32, u32)> {
    let mut seen = HashSet::new();
    let mut queue = VecDeque::from([start]);
    while let Some((x, y)) = queue.pop_front() {
        if !lit(img.pixel(x, y)) || !seen.insert((x, y)) {
            continue;
        }
        for dy in -1i64..=1 {
            for dx in -1i64..=1 {
                let (nx, ny) = (i64::from(x) + dx, i64::from(y) + dy);
                if nx >= 0 && ny >= 0 && nx < i64::from(img.size[0]) && ny < i64::from(img.size[1])
                {
                    queue.push_back((nx as u32, ny as u32));
                }
            }
        }
    }
    seen
}

/// Points of a polyline that starts at `start` heading along `angle0` and turns by
/// `turns[i % turns.len()]` (radians) after each segment of `len` mm, in the XZ plane.
fn turning_polyline(
    start: [f64; 2],
    angle0: f64,
    len: f64,
    turns: &[f64],
    n: usize,
) -> Vec<Point3> {
    let mut pts = vec![Point3::new(start[0], 0.0, start[1])];
    let mut angle = angle0;
    for i in 0..n {
        let last = pts[pts.len() - 1];
        pts.push(last + len * Vec3::new(angle.cos(), 0.0, angle.sin()));
        angle += turns[i % turns.len()];
    }
    pts
}

/// Done-when 1: 1-px and 3-px polylines with 170° turns (and with gentle 10° turns, whose joins
/// skip the cap) cover every pixel their centre line passes through and are one 8-connected
/// component.
#[test]
fn gpu_polylines_with_sharp_turns_have_no_gaps() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [256, 256];
    let cam = front_ortho(256.0);
    let hairpin = 170f64.to_radians();
    let shapes = [
        (
            "hairpins",
            turning_polyline([-60.0, -90.0], 0.0, 90.0, &[hairpin, -hairpin], 16),
        ),
        (
            "gentle",
            turning_polyline([-10.0, -100.0], 0.0, 12.0, &[10f64.to_radians()], 30),
        ),
    ];
    for samples in [1, 4] {
        for width in [1.0f32, 3.0] {
            for (name, pts) in &shapes {
                let mut r = renderer(&gpu, samples);
                let e =
                    upload_edges(&gpu, &edges(pts.clone(), false), &JobCtx::detached()).unwrap();
                let key = r.scene_mut().insert_edges(e);
                let mut inst = Instance::new_edges(key, Transform::IDENTITY);
                inst.edge_style = line_style(width, WHITE);
                r.scene_mut().add_instance(inst);
                let view = add_view(&mut r, cam, size, style(VisualStyle::Wireframe));
                let img = capture(&mut r, view);
                let centre = samples_along(&cam, size, pts, false, 0.0);
                let missing: Vec<_> = centre
                    .iter()
                    .filter(|&&p| !pixel_at(&img, p).is_some_and(lit))
                    .collect();
                assert!(
                    missing.is_empty(),
                    "{name} {width} px x{samples}: {} of {} centre-line pixels unlit, first {:?}",
                    missing.len(),
                    centre.len(),
                    missing.first()
                );
                let start = (centre[0][0].floor() as u32, centre[0][1].floor() as u32);
                let component = reachable(&img, start);
                let total = img
                    .pixels
                    .chunks_exact(4)
                    .filter(|p| lit([p[0], p[1], p[2], p[3]]))
                    .count();
                assert_eq!(
                    component.len(),
                    total,
                    "{name} {width} px x{samples}: the lit pixels are not one connected line"
                );
                eprintln!(
                    "{name} {width} px x{samples}: {} centre samples lit, {total} lit pixels, connected",
                    centre.len()
                );
            }
        }
    }
}

/// A square face of side `side` in the XY plane (normal +Z), double-sided.
fn face(side: f32) -> MeshData {
    let h = side / 2.0;
    MeshData {
        origin: Point3::ZERO,
        positions: vec![[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]],
        normals: vec![[0.0, 0.0, 1.0]; 4],
        indices: vec![0, 1, 2, 0, 2, 3],
        closed: false,
    }
}

/// Polylines lying in the plane z = `z` inside a face of side `s`: an inner square, both
/// diagonals of it and a circle.
fn face_edges(s: f64, z: f64) -> Vec<(Vec<Point3>, bool)> {
    let a = 0.3 * s;
    let sq = vec![
        Point3::new(-a, -a, z),
        Point3::new(a, -a, z),
        Point3::new(a, a, z),
        Point3::new(-a, a, z),
    ];
    let circle = (0..64)
        .map(|i| {
            let t = std::f64::consts::TAU * f64::from(i) / 64.0;
            Point3::new(0.2 * s * t.cos(), 0.2 * s * t.sin(), z)
        })
        .collect();
    vec![
        (sq.clone(), true),
        (vec![sq[0], sq[2]], false),
        (vec![sq[1], sq[3]], false),
        (circle, true),
    ]
}

/// Camera at `distance` from the origin, 40° off the face normal (+Z).
fn oblique(distance: f64) -> Camera {
    let tilt = 40f64.to_radians();
    Camera {
        eye: Point3::new(0.0, -distance * tilt.sin(), distance * tilt.cos()),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective {
            fov_y: 45f64.to_radians(),
        },
    }
}

/// Fraction of the centre-line samples of `lines` (inside the view, 3 px from its border)
/// whose pixel satisfies `is_edge`.
fn edge_fraction(
    img: &RgbaImage,
    cam: &Camera,
    lines: &[(Vec<Point3>, bool)],
    skip: f64,
    is_edge: impl Fn([u8; 4]) -> bool,
) -> (f64, usize) {
    let size = img.size;
    let mut hits = 0;
    let mut total = 0;
    for (pts, closed) in lines {
        for p in samples_along(cam, size, pts, *closed, skip) {
            let inside = p[0] >= 3.0
                && p[1] >= 3.0
                && p[0] < f64::from(size[0]) - 3.0
                && p[1] < f64::from(size[1]) - 3.0;
            if !inside {
                continue;
            }
            total += 1;
            if pixel_at(img, p).is_some_and(&is_edge) {
                hits += 1;
            }
        }
    }
    (hits as f64 / total.max(1) as f64, total)
}

fn red(px: [u8; 4]) -> bool {
    i32::from(px[0]) > i32::from(px[2]) + 40
}

/// Done-when 2: edges lying on their own face are visible for ≥ 99 % of their centre-line
/// pixels from 1e-2 to 1e5 mm view distance, and an edge 0.5 mm behind the face is hidden.
#[test]
fn gpu_edges_on_their_faces_are_visible_at_every_distance_and_hidden_behind() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [256, 256];
    for exp in -2..=5 {
        let d = 10f64.powi(exp);
        let mut r = renderer(&gpu, 4);
        let mesh = r
            .scene_mut()
            .insert_mesh(upload_mesh(&gpu, &face(d as f32), &JobCtx::detached()).unwrap());
        let lines = face_edges(d, 0.0);
        let e = upload_edges(&gpu, &polylines(&lines), &JobCtx::detached()).unwrap();
        let mut inst = Instance::new(mesh, Transform::IDENTITY);
        inst.edges = Some(r.scene_mut().insert_edges(e));
        inst.material.base_color = [0.1, 0.2, 0.9, 1.0];
        inst.edge_style = line_style(2.0, [1.0, 0.0, 0.0, 1.0]);
        r.scene_mut().add_instance(inst);
        let cam = oblique(d);
        let view = add_view(&mut r, cam, size, style(VisualStyle::ShadedVisibleEdges));
        let img = capture(&mut r, view);
        let (fraction, n) = edge_fraction(&img, &cam, &lines, 0.0, red);
        eprintln!(
            "view distance {d:e} mm: {:.2} % of {n} edge pixels visible",
            100.0 * fraction
        );
        assert!(n > 300, "the edges are in view ({n} samples)");
        assert!(
            fraction >= 0.99,
            "at {d:e} mm only {:.2} % of the edge pixels are visible",
            100.0 * fraction
        );
    }

    // 0.5 mm behind a 100 mm face seen at 250 mm in a 1440 px tall view (0.144 mm/px, the
    // RND-02 bench framing): the depth offset is (1.5 + 1) px = 0.36 mm along the view ray, the
    // gap 0.5 / cos 40° = 0.65 mm.
    let size = [256, 1440];
    let mut r = renderer(&gpu, 4);
    let mesh = r
        .scene_mut()
        .insert_mesh(upload_mesh(&gpu, &face(100.0), &JobCtx::detached()).unwrap());
    let behind = face_edges(100.0, -0.5);
    let e = upload_edges(&gpu, &polylines(&behind), &JobCtx::detached()).unwrap();
    let mut inst = Instance::new(mesh, Transform::IDENTITY);
    inst.edges = Some(r.scene_mut().insert_edges(e));
    inst.material.base_color = [0.1, 0.2, 0.9, 1.0];
    inst.edge_style = line_style(2.0, [1.0, 0.0, 0.0, 1.0]);
    r.scene_mut().add_instance(inst);
    let cam = oblique(250.0);
    let px = cam.pixel_size_at(Point3::ZERO, size);
    let view = add_view(&mut r, cam, size, style(VisualStyle::ShadedVisibleEdges));
    let img = capture(&mut r, view);
    let (fraction, n) = edge_fraction(&img, &cam, &behind, 0.0, red);
    eprintln!(
        "edge 0.5 mm behind the face at {px:.3} mm/px: {:.2} % of {n} pixels visible",
        100.0 * fraction
    );
    assert!(n > 300);
    assert!(
        fraction <= 0.01,
        "{:.2} % of the hidden edge shows",
        100.0 * fraction
    );
}

/// A closed 100 mm cube centred on the origin (flat shaded) and its 12 edges.
fn cube_scene(r: &mut Renderer, gpu: &Gpu) {
    let mesh = amg_testkit::shapes::cube(Point3::splat(-50.0), 100.0);
    let data = MeshData {
        origin: Point3::ZERO,
        positions: mesh
            .positions
            .iter()
            .map(|p| p.as_vec3().to_array())
            .collect(),
        normals: Vec::new(),
        indices: mesh.triangles.iter().flatten().copied().collect(),
        closed: true,
    };
    let key = r
        .scene_mut()
        .insert_mesh(upload_mesh(gpu, &data, &JobCtx::detached()).unwrap());
    let e = upload_edges(gpu, &polylines(&cube_edges()), &JobCtx::detached()).unwrap();
    let mut inst = Instance::new(key, Transform::IDENTITY);
    inst.edges = Some(r.scene_mut().insert_edges(e));
    inst.material.base_color = [0.5, 0.5, 0.5, 1.0];
    inst.edge_style = line_style(2.0, [0.0, 1.0, 0.0, 1.0]);
    r.scene_mut().add_instance(inst);
}

fn cube_edges() -> Vec<(Vec<Point3>, bool)> {
    let c = |i: u32| {
        let bit = |b: u32| if (i >> b) & 1 == 1 { 50.0 } else { -50.0 };
        Point3::new(bit(0), bit(1), bit(2))
    };
    let mut out = Vec::new();
    for i in 0..8u32 {
        for b in 0..3 {
            if (i >> b) & 1 == 0 {
                out.push((vec![c(i), c(i | (1 << b))], false));
            }
        }
    }
    out
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

/// Green edge colour (over grey faces or a black background).
fn green(px: [u8; 4]) -> bool {
    i32::from(px[1]) - i32::from(px[0]) > 25
}

/// The on/off runs of `is_edge` along the samples of one segment.
fn runs(img: &RgbaImage, samples: &[[f64; 2]], is_edge: impl Fn([u8; 4]) -> bool) -> (f64, usize) {
    let mut pixels: Vec<(u32, u32)> = Vec::new();
    for p in samples {
        let q = (p[0].floor() as u32, p[1].floor() as u32);
        if pixels.last() != Some(&q) {
            pixels.push(q);
        }
    }
    let states: Vec<bool> = pixels
        .iter()
        .map(|&(x, y)| is_edge(img.pixel(x, y)))
        .collect();
    let on = states.iter().filter(|&&s| s).count();
    let transitions = states.windows(2).filter(|w| w[0] != w[1]).count();
    (on as f64 / states.len().max(1) as f64, transitions)
}

/// Done-when 3 (and the six styles): the cube's hidden edge (−50, 50, −50)–(50, 50, −50) is
/// dashed in the hidden-edge styles, absent in the visible-edge styles, solid in `Wireframe`;
/// a front edge shows in every style but `Shaded`; faces are shaded only in the shaded styles.
#[test]
fn gpu_visual_styles_draw_visible_hidden_and_dashed_edges() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [320, 320];
    let cam = cube_camera();
    let hidden = [
        vec![
            Point3::new(-50.0, 50.0, -50.0),
            Point3::new(50.0, 50.0, -50.0),
        ],
        vec![
            Point3::new(-50.0, 50.0, -50.0),
            Point3::new(-50.0, 50.0, 50.0),
        ],
    ];
    let front = vec![
        Point3::new(50.0, -50.0, 50.0),
        Point3::new(50.0, -50.0, -50.0),
    ];
    let face_centre = cam.project(Point3::new(50.0, 0.0, 0.0), size).unwrap();
    for visual in [
        VisualStyle::Shaded,
        VisualStyle::ShadedVisibleEdges,
        VisualStyle::ShadedHiddenEdges,
        VisualStyle::Wireframe,
        VisualStyle::WireframeHiddenEdges,
        VisualStyle::WireframeVisibleEdges,
    ] {
        let mut r = renderer(&gpu, 4);
        cube_scene(&mut r, &gpu);
        let view = add_view(&mut r, cam, size, style(visual));
        let img = capture(&mut r, view);
        // The hidden edges cross visible ones on screen: leave out samples near any other edge.
        let others: Vec<[[f64; 3]; 2]> = cube_edges()
            .iter()
            .filter(|(pts, _)| !hidden.contains(pts))
            .map(|(pts, _)| {
                [
                    cam.project(pts[0], size).unwrap(),
                    cam.project(pts[1], size).unwrap(),
                ]
            })
            .collect();
        let clear_of_others = |p: &[f64; 2]| {
            others.iter().all(|[a, b]| {
                let (ab, ap) = ([b[0] - a[0], b[1] - a[1]], [p[0] - a[0], p[1] - a[1]]);
                let t = ((ap[0] * ab[0] + ap[1] * ab[1]) / (ab[0] * ab[0] + ab[1] * ab[1]))
                    .clamp(0.0, 1.0);
                (ap[0] - t * ab[0]).hypot(ap[1] - t * ab[1]) >= 4.0
            })
        };
        let mut hidden_stats = Vec::new();
        for h in &hidden {
            let s: Vec<_> = samples_along(&cam, size, h, false, 0.2)
                .into_iter()
                .filter(clear_of_others)
                .collect();
            assert!(
                s.len() > 40,
                "enough of the hidden edge is clear of the others"
            );
            hidden_stats.push(runs(&img, &s, green));
        }
        let (front_on, _) = runs(&img, &samples_along(&cam, size, &front, false, 0.2), green);
        let face = pixel_at(&img, [face_centre[0], face_centre[1]]).unwrap();
        eprintln!(
            "{visual:?}: hidden edges (on fraction, transitions) {hidden_stats:?}, front edge {front_on:.2}, face centre {face:?}"
        );
        let shaded_face = face[0] > 30;
        assert_eq!(
            shaded_face,
            visual.shades_faces(),
            "{visual:?} face shading"
        );
        if visual == VisualStyle::Shaded {
            assert!(front_on < 0.02, "Shaded draws no edges");
        } else {
            assert!(front_on > 0.98, "{visual:?}: the front edge is solid");
        }
        for &(on, transitions) in &hidden_stats {
            match visual {
                VisualStyle::ShadedHiddenEdges | VisualStyle::WireframeHiddenEdges => {
                    assert!(
                        (0.3..=0.85).contains(&on) && transitions >= 6,
                        "{visual:?}: hidden edge not dashed (on {on:.2}, {transitions} transitions)"
                    );
                }
                VisualStyle::Wireframe => {
                    assert!(on > 0.98, "Wireframe draws hidden edges solid ({on:.2})");
                }
                _ => assert!(on < 0.02, "{visual:?}: the hidden edge shows ({on:.2})"),
            }
        }
    }
}

/// An open cylinder side (axis Z, radius `r`, height `h`) with smooth normals.
fn cylinder(r: f32, h: f32, segments: u32) -> MeshData {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    for i in 0..segments {
        let t = std::f32::consts::TAU * i as f32 / segments as f32;
        let (s, c) = t.sin_cos();
        for z in [-h / 2.0, h / 2.0] {
            positions.push([r * c, r * s, z]);
            normals.push([c, s, 0.0]);
        }
    }
    let mut indices = Vec::new();
    for i in 0..segments {
        let j = (i + 1) % segments;
        let (a0, a1, b0, b1) = (2 * i, 2 * i + 1, 2 * j, 2 * j + 1);
        indices.extend([a0, b0, b1, a0, b1, a1]);
    }
    MeshData {
        origin: Point3::ZERO,
        positions,
        normals,
        indices,
        closed: true,
    }
}

fn magenta(px: [u8; 4]) -> bool {
    px[0] > 200 && px[2] > 200 && px[1] < 80
}

/// Done-when 4: the silhouettes of a cylinder seen side-on are present at both limbs, against
/// the background and against a plane behind it, and absent inside the cylinder.
#[test]
fn gpu_cylinder_silhouettes_are_present() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [256, 256];
    let cam = front_ortho(200.0);
    let mm_to_px = 256.0 / 200.0;
    for (backdrop, samples) in [(false, 4), (true, 4), (false, 1)] {
        for on in [true, false] {
            let mut r = renderer(&gpu, samples);
            let key = r.scene_mut().insert_mesh(
                upload_mesh(&gpu, &cylinder(30.0, 80.0, 96), &JobCtx::detached()).unwrap(),
            );
            r.scene_mut()
                .add_instance(Instance::new(key, Transform::IDENTITY));
            if backdrop {
                let plane = r
                    .scene_mut()
                    .insert_mesh(upload_mesh(&gpu, &face(400.0), &JobCtx::detached()).unwrap());
                let mut inst = Instance::new(
                    plane,
                    Transform::from_rotation_translation(
                        amg_core::Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
                        Vec3::new(0.0, 100.0, 0.0),
                    ),
                );
                inst.material.base_color = [1.0, 1.0, 1.0, 1.0];
                r.scene_mut().add_instance(inst);
            }
            let mut st = style(VisualStyle::Shaded);
            st.silhouettes = on;
            st.silhouette_color = [1.0, 0.0, 1.0];
            let view = add_view(&mut r, cam, size, st);
            let img = capture(&mut r, view);
            let limb = 30.0 * mm_to_px;
            let mut rows_with_both = 0;
            let mut interior_hits = 0;
            let rows: Vec<u32> = (100..156).collect();
            for &y in &rows {
                let has = |x0: f64| {
                    ((x0 - 2.5).floor() as u32..=(x0 + 2.5).ceil() as u32)
                        .any(|x| magenta(img.pixel(x, y)))
                };
                if has(128.0 - limb) && has(128.0 + limb) {
                    rows_with_both += 1;
                }
                interior_hits += (108..148).filter(|&x| magenta(img.pixel(x, y))).count();
            }
            let anywhere = img
                .pixels
                .chunks_exact(4)
                .filter(|p| magenta([p[0], p[1], p[2], p[3]]))
                .count();
            eprintln!(
                "backdrop {backdrop} x{samples} silhouettes {on}: {rows_with_both}/{} rows with both limbs, {interior_hits} interior, {anywhere} outline pixels",
                rows.len()
            );
            if on {
                assert_eq!(
                    rows_with_both,
                    rows.len(),
                    "both limbs outlined on every row"
                );
                assert_eq!(interior_hits, 0, "no outline inside the cylinder");
            } else {
                assert_eq!(anywhere, 0, "no outline with silhouettes off");
            }
        }
    }
}

/// Point sprites are round discs of the requested diameter; a point behind a face is hidden in
/// `Test` mode and dimmed in `XRay` mode.
#[test]
fn gpu_point_sprites_are_round_and_depth_tested() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [128, 128];
    let cam = front_ortho(128.0);
    let centres = [Point3::new(-30.0, 0.0, 20.0), Point3::new(30.0, 0.0, -20.0)];
    for depth in [DepthMode::Test, DepthMode::XRay { hidden_alpha: 0.5 }] {
        let mut r = renderer(&gpu, 4);
        let pts = upload_points(
            &gpu,
            &PointData {
                origin: Point3::ZERO,
                points: centres.iter().map(|p| p.as_vec3().to_array()).collect(),
                ids: Vec::new(),
            },
            &JobCtx::detached(),
        )
        .unwrap();
        let mut inst = Instance::new_points(r.scene_mut().insert_points(pts), Transform::IDENTITY);
        inst.point_style = PointStyle {
            size_px: 11.0,
            color: WHITE,
            depth,
        };
        r.scene_mut().add_instance(inst);
        // A small grey face in front of the second point only.
        let blocker = r
            .scene_mut()
            .insert_mesh(upload_mesh(&gpu, &face(20.0), &JobCtx::detached()).unwrap());
        let mut b = Instance::new(
            blocker,
            Transform::from_rotation_translation(
                amg_core::Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
                Vec3::new(30.0, -10.0, -20.0),
            ),
        );
        b.material.base_color = [0.02, 0.02, 0.02, 1.0];
        r.scene_mut().add_instance(b);
        let view = add_view(&mut r, cam, size, style(VisualStyle::ShadedVisibleEdges));
        let img = capture(&mut r, view);
        let c = cam.project(centres[0], size).unwrap();
        for y in 0..size[1] {
            for x in 0..size[0] {
                let d = (f64::from(x) + 0.5 - c[0]).hypot(f64::from(y) + 0.5 - c[1]);
                let v = img.pixel(x, y)[0];
                if d <= 4.0 {
                    assert!(v > 230, "inside the disc at {d:.2} px: {v}");
                } else if (7.0..20.0).contains(&d) {
                    assert_eq!(v, 0, "outside the disc at {d:.2} px: {v}");
                }
            }
        }
        let hidden = cam.project(centres[1], size).unwrap();
        let v = img.pixel(hidden[0] as u32, hidden[1] as u32);
        eprintln!("{depth:?}: point behind the face shows {v:?}");
        match depth {
            DepthMode::Test => assert!(v[0] < 80, "the hidden point is hidden: {v:?}"),
            _ => assert!(v[0] > 100 && v[0] < 230, "x-ray shows it dimmed: {v:?}"),
        }
    }
}

/// The triangle wireframe of a mesh body: `POLYGON_MODE_LINE` draws the interior diagonal of a
/// two-triangle square; without it (or disabled) the wireframe styles draw its feature edges
/// (the square's boundary) and never the diagonal.
#[test]
fn gpu_mesh_wireframe_uses_polygon_mode_line_or_feature_edges() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let size = [128, 128];
    let cam = front_ortho(128.0);
    let h = 40.0;
    let corners = [
        Point3::new(-h, 0.0, -h),
        Point3::new(h, 0.0, -h),
        Point3::new(h, 0.0, h),
        Point3::new(-h, 0.0, h),
    ];
    let diagonal = vec![corners[0], corners[2]];
    for enabled in [true, false] {
        let mut r = Renderer::new(
            Arc::clone(&gpu),
            &RendererConfig {
                polygon_mode_line: enabled,
                ..Default::default()
            },
        )
        .unwrap();
        if enabled && !r.triangle_wireframe() {
            eprintln!("the device has no POLYGON_MODE_LINE; only the fallback is tested");
            continue;
        }
        // The square in the XZ plane, facing the camera (−Y).
        let data = MeshData {
            origin: Point3::ZERO,
            positions: corners.iter().map(|p| p.as_vec3().to_array()).collect(),
            normals: Vec::new(),
            indices: vec![0, 2, 1, 0, 3, 2],
            closed: false,
        };
        let mesh = r
            .scene_mut()
            .insert_mesh(upload_mesh(&gpu, &data, &JobCtx::detached()).unwrap());
        let e = upload_edges(&gpu, &edges(corners.to_vec(), true), &JobCtx::detached()).unwrap();
        let mut inst = Instance::new(mesh, Transform::IDENTITY);
        inst.edges = Some(r.scene_mut().insert_edges(e));
        inst.mesh_wireframe = true;
        inst.edge_style = line_style(2.0, WHITE);
        r.scene_mut().add_instance(inst);
        for visual in [VisualStyle::Wireframe, VisualStyle::WireframeVisibleEdges] {
            let view = add_view(&mut r, cam, size, style(visual));
            let img = capture(&mut r, view);
            let near_lit = |p: [f64; 2]| {
                let (x, y) = (p[0].floor() as i64, p[1].floor() as i64);
                (-1..=1)
                    .any(|dy| (-1..=1).any(|dx| lit(img.pixel((x + dx) as u32, (y + dy) as u32))))
            };
            let diag = samples_along(&cam, size, &diagonal, false, 0.2);
            let diag_lit = diag.iter().filter(|&&p| near_lit(p)).count() as f64 / diag.len() as f64;
            let border = samples_along(&cam, size, &corners, true, 0.1);
            let border_lit =
                border.iter().filter(|&&p| near_lit(p)).count() as f64 / border.len() as f64;
            let centre = img.pixel(64 + 20, 64 - 8);
            eprintln!(
                "polygon_mode_line {enabled} {visual:?}: diagonal {diag_lit:.2}, border {border_lit:.2}, face {centre:?}"
            );
            assert!(border_lit > 0.98, "the boundary is drawn");
            assert!(!lit(centre), "no face shading in wireframe styles");
            if enabled {
                assert!(diag_lit > 0.95, "the triangle wireframe draws the diagonal");
            } else {
                assert!(diag_lit < 0.02, "the fallback draws feature edges only");
            }
        }
    }
}

/// Edge-only and point-only instances are culled with their own bounds, counted in
/// `FrameStats::edge_segments`, and their VRAM is returned on removal.
#[test]
fn gpu_edge_sets_are_culled_counted_and_freed() {
    let Some(gpu) = gpu() else {
        eprintln!("no GPU adapter; skipping");
        return;
    };
    let mut r = renderer(&gpu, 1);
    let base = r.scene().gpu_bytes();
    let line = edges(
        vec![
            Point3::new(-10.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 5.0),
            Point3::new(10.0, 0.0, 0.0),
        ],
        false,
    );
    let e = upload_edges(&gpu, &line, &JobCtx::detached()).unwrap();
    assert_eq!(e.segment_count(), 2);
    assert_eq!(e.gpu_bytes(), 2 * 32 + 2 * 4);
    let key = r.scene_mut().insert_edges(e);
    let near = r
        .scene_mut()
        .add_instance(Instance::new_edges(key, Transform::IDENTITY));
    let far = r.scene_mut().add_instance(Instance::new_edges(
        key,
        Transform::from_translation(Vec3::new(0.0, 0.0, 1.0e4)),
    ));
    let view = add_view(
        &mut r,
        front_ortho(100.0),
        [64, 64],
        style(VisualStyle::Wireframe),
    );
    assert!(r.prepare_view(view));
    let mut encoder = gpu.device().create_command_encoder(&Default::default());
    let stats = r.encode(view, &mut encoder).unwrap();
    gpu.queue().submit([encoder.finish()]);
    assert_eq!(stats.edge_segments, 2, "the far copy is culled");
    let b = r.scene().visible_bounds();
    assert!(
        b.max.z > 9_000.0,
        "visible bounds include edge-only instances"
    );
    r.view_mut(view).unwrap().style.visual = VisualStyle::Shaded;
    let mut encoder = gpu.device().create_command_encoder(&Default::default());
    let stats = r.encode(view, &mut encoder).unwrap();
    gpu.queue().submit([encoder.finish()]);
    assert_eq!(stats.edge_segments, 0, "Shaded draws no edges");
    assert!(r.scene_mut().remove_instance(near));
    assert!(r.scene_mut().remove_instance(far));
    assert!(r.scene_mut().remove_edges(key));
    assert_eq!(r.scene().gpu_bytes(), base);
    assert_eq!(r.scene().edges_count(), 0);
}
