//! RND-11 GPU tests: precision of a tiny feature far from the origin over a full orbit, the
//! view-cube inset, and the fitted orthographic depth range. Named `gpu_*` (nextest test-group
//! `gpu`, serial); they skip without an adapter and fail under `AMG_REQUIRE_GPU=1` (CI runs
//! them on WARP).

#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::sync::Arc;

use amg_core::{Aabb3, Dir3, Point3, Transform, Vec3};
use amg_jobs::JobCtx;
use amg_jobs::gpu::{Gpu, GpuConfig};
use amg_render::nav::{self, CubeRegion, NavController, NavMode, StandardView, ViewCube};
use amg_render::{
    Background, Camera, Instance, Lighting, MeshData, Projection, Renderer, RendererConfig,
    RgbaImage, ViewId, ViewStyle, VisualStyle, upload_mesh,
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

/// An icosphere of `radius` around the local origin (smooth normals).
fn sphere(radius: f64, subdivisions: u32) -> MeshData {
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

/// Centroid and equivalent radius (`sqrt(area / π)`) of the pixels brighter than the background.
fn blob(img: &RgbaImage) -> ([f64; 2], f64) {
    let (mut n, mut sx, mut sy) = (0.0, 0.0, 0.0);
    for y in 0..img.size[1] {
        for x in 0..img.size[0] {
            let p = img.pixel(x, y);
            if p[0].max(p[1]).max(p[2]) > 40 {
                n += 1.0;
                sx += f64::from(x) + 0.5;
                sy += f64::from(y) + 0.5;
            }
        }
    }
    assert!(n > 0.0, "nothing drawn");
    ([sx / n, sy / n], (n / std::f64::consts::PI).sqrt())
}

/// Done-when 9: a 0.01 mm feature 1e4 mm from the origin, zoomed to fill the view (fit), stays
/// within 0.5 px over a 360° orbit about it.
#[test]
fn gpu_tiny_feature_far_from_the_origin_is_stable_over_a_full_orbit() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 4);
    let size = [256, 256];
    let centre = Point3::new(7071.0678, -7071.0678, 0.0); // 1e4 mm from the origin
    let radius = 0.005; // 0.01 mm across
    let mesh = upload_mesh(&gpu, &sphere(radius, 4), &JobCtx::detached()).unwrap();
    let key = r.scene_mut().insert_mesh(mesh);
    r.scene_mut()
        .add_instance(Instance::new(key, Transform::from_translation(centre)));
    let bounds = r.scene().visible_bounds();
    assert!(bounds.center().distance(centre) < 1e-9);

    let mut cam = Camera {
        eye: centre + Vec3::new(30.0, -40.0, 20.0),
        target: centre,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    let mut nav = NavController::new();
    nav.fit(&mut cam, bounds, size, false);
    // Fit frames the box's bounding sphere (√3 × the feature); zoom in about the feature so the
    // feature itself fills the view.
    let steps = (1.0 / 3f64.sqrt()).ln() / nav::ZOOM_STEP.ln();
    nav.zoom(&mut cam, steps, [128.0, 128.0], Some(centre), size);
    eprintln!(
        "eye distance {:.5} mm, near {:.1e} mm",
        cam.eye.distance(centre),
        cam.default_near()
    );
    let view = add_view(&mut r, cam, size);

    let steps = 24;
    let h = f64::from(size[1]);
    nav.begin(NavMode::Orbit, &cam, [0.0, 0.0], Some(centre), bounds);
    let mut first: Option<([f64; 2], f64)> = None;
    let (mut worst_pos, mut worst_radius) = (0.0f64, 0.0f64);
    for k in 0..=steps {
        // 360° = two view heights of drag; a little pitch too, so the view axis moves in 3D.
        let dx = 2.0 * h * f64::from(k) / f64::from(steps);
        let dy = 0.2 * h * (f64::from(k) * 0.7).sin();
        nav.drag(&mut cam, [dx, dy], size);
        r.view_mut(view).unwrap().camera = cam;
        let img = capture(&mut r, view);
        let (c, rad) = blob(&img);
        // The feature centre's pixel from the f64 camera.
        let p = cam.project(centre, size).unwrap();
        worst_pos = worst_pos.max((c[0] - p[0]).hypot(c[1] - p[1]));
        let (c0, r0) = *first.get_or_insert((c, rad));
        worst_pos = worst_pos.max((c[0] - c0[0]).hypot(c[1] - c0[1]));
        worst_radius = worst_radius.max((rad - r0).abs());
        // Filled: the disc spans most of the view.
        assert!(rad * 2.0 > 0.8 * h, "radius {rad} px");
    }
    nav.end();
    eprintln!(
        "orbit at 1e4 mm: centroid within {worst_pos:.4} px, radius within {worst_radius:.4} px"
    );
    assert!(worst_pos <= 0.5, "centroid moved {worst_pos} px");
    assert!(worst_radius <= 0.5, "radius changed {worst_radius} px");
}

/// The view cube is drawn as an inset in the top-right corner with its letters, follows the
/// camera, and shows the hover region.
#[test]
fn gpu_view_cube_inset_draws_in_the_top_right_corner() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 4);
    let size = [400, 300];
    let front = Camera {
        eye: Point3::new(0.0, -100.0, 0.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    let view = add_view(&mut r, front, size);
    let plain = capture(&mut r, view);
    let cube = ViewCube::default();
    let [x, y, w, _] = cube.rect_px(size);
    assert_eq!([x, y, w], [400 - 120, 10, 110]);
    let (cx, cy) = (x + w / 2, y + w / 2);
    assert_eq!(plain.pixel(cx, cy)[..3], [0, 0, 0], "no cube unless asked");
    {
        let v = r.view_mut(view).unwrap();
        v.show_view_cube = true;
        v.view_cube = cube;
    }
    let img = capture(&mut r, view);
    let face = img.pixel(cx, cy);
    assert!(
        face[0] > 100 && face[0].abs_diff(face[2]) < 40,
        "grey face {face:?}"
    );
    // Off the cube inside the inset, and outside the inset: background.
    assert_eq!(img.pixel(x + 3, y + w - 3)[..3], [0, 0, 0]);
    assert_eq!(img.pixel(cx, y + w + 5)[..3], [0, 0, 0]);
    assert_eq!(img.pixel(100, 150)[..3], [0, 0, 0]);
    // The FRONT letter sits on the face centre (labels are egui-painted by the shell).
    let labels = r.project_labels(view);
    assert_eq!(labels.len(), 1, "face-on: one letter");
    assert_eq!(labels[0].text, "FRONT");
    let pos = labels[0].pos_px;
    assert!((f64::from(pos[0]) - f64::from(x) - 55.0).abs() < 1.0);
    assert!((f64::from(pos[1]) - f64::from(y) - 55.0).abs() < 1.0);
    // Hovered region in the hover colour (blue).
    r.view_mut(view).unwrap().view_cube.hover = Some(CubeRegion::Face(StandardView::Front));
    let hover = capture(&mut r, view);
    let p = hover.pixel(cx, cy);
    assert!(p[2] > p[0] + 60, "hover colour {p:?}");
    // From Home the cube shows three faces and three letters, around the same centre.
    let mut cam = front;
    NavController::new().set_view(&mut cam, StandardView::Home, Aabb3::EMPTY, size, false);
    {
        let v = r.view_mut(view).unwrap();
        v.camera = cam;
        v.view_cube.hover = None;
    }
    let home = capture(&mut r, view);
    assert!(home.pixel(cx, cy)[0] > 60);
    let mut names: Vec<String> = r.project_labels(view).into_iter().map(|l| l.text).collect();
    names.sort();
    assert_eq!(names, ["FRONT", "RIGHT", "TOP"]);
}

/// Near-plane policy: an orthographic view fits its depth range to the scene, so a part far in
/// front of a close eye (beyond the 1000 × distance placeholder) is drawn, and the same part
/// close to the eye renders identically.
#[test]
fn gpu_orthographic_view_fits_its_depth_range_to_the_scene() {
    let Some(gpu) = gpu() else { return };
    let mut r = renderer(&gpu, 1);
    let size = [128, 128];
    let mesh = upload_mesh(&gpu, &sphere(10.0, 3), &JobCtx::detached()).unwrap();
    let key = r.scene_mut().insert_mesh(mesh);
    r.scene_mut()
        .add_instance(Instance::new(key, Transform::from_translation(Vec3::ZERO)));
    // Eye 1 mm from its target, the sphere 5000 mm further along the view axis.
    let far = Camera {
        eye: Point3::new(0.0, -5001.0, 0.0),
        target: Point3::new(0.0, -5000.0, 0.0),
        up: Dir3::Z,
        projection: Projection::Orthographic { height: 30.0 },
    };
    let placeholder = far.default_depth_range();
    assert!(
        placeholder.far < 5000.0,
        "the old range would clip the part"
    );
    let fitted = far.depth_range_for(&r.scene().visible_bounds());
    assert!(
        fitted.near < 5001.0 - 10.0 && fitted.far > 5001.0 + 10.0,
        "{fitted:?}"
    );
    assert!(
        fitted.far - fitted.near < 40.0,
        "fitted to the part: {fitted:?}"
    );
    let view = add_view(&mut r, far, size);
    let img_far = capture(&mut r, view);
    let (c, rad) = blob(&img_far);
    assert!(
        (c[0] - 64.0).abs() < 0.5 && (c[1] - 64.0).abs() < 0.5,
        "{c:?}"
    );
    assert!((rad - 128.0 / 30.0 * 10.0).abs() < 1.5, "radius {rad}");
    let near = Camera {
        eye: Point3::new(0.0, -100.0, 0.0),
        target: Point3::ZERO,
        ..far
    };
    r.view_mut(view).unwrap().camera = near;
    let img_near = capture(&mut r, view);
    let diff = img_far
        .pixels
        .iter()
        .zip(&img_near.pixels)
        .map(|(a, b)| a.abs_diff(*b))
        .max()
        .unwrap();
    assert!(diff <= 2, "eye distance changed the image by {diff}");
}
