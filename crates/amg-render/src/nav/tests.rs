//! RND-11 Done-when 1-7 (no GPU).

#![allow(clippy::float_cmp, reason = "exact end points and pass-through values")]

use super::*;

const SIZE: [u32; 2] = [1280, 720];

/// Small deterministic generator (xorshift64*).
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

    fn unit(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            if (0.01..=1.0).contains(&v.length_squared()) {
                return v.normalize();
            }
        }
    }
}

/// A camera at `dist` from a target up to 1e3 mm off the origin, perspective or orthographic.
fn camera(rng: &mut Rng, dist: f64, ortho: bool) -> Camera {
    let target = Vec3::new(
        rng.range(-1e3, 1e3),
        rng.range(-1e3, 1e3),
        rng.range(-1e3, 1e3),
    );
    let mut dir = rng.unit();
    dir.z = dir.z.clamp(-0.9, 0.9);
    let projection = if ortho {
        Projection::Orthographic { height: dist * 0.8 }
    } else {
        Projection::Perspective {
            fov_y: rng.range(0.3, 1.2),
        }
    };
    Camera {
        eye: target + dir.normalize() * dist,
        target,
        up: Dir3::Z,
        projection,
    }
}

/// A world point at pixel `px` and `depth_frac` of the eye-to-target distance in front of the eye.
fn point_at(cam: &Camera, px: [f64; 2], depth_frac: f64) -> Point3 {
    let ray = cam.ray(px, SIZE);
    let forward = cam.forward().get();
    let depth = cam.eye.distance(cam.target) * depth_frac;
    let t = (depth - (ray.origin - cam.eye).dot(forward)) / ray.dir.get().dot(forward);
    ray.at(t)
}

fn pixel(cam: &Camera, p: Point3) -> [f64; 2] {
    let q = cam.project(p, SIZE).expect("in front of the eye");
    [q[0], q[1]]
}

fn px_err(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn random_px(rng: &mut Rng) -> [f64; 2] {
    [rng.range(40.0, 1240.0), rng.range(40.0, 680.0)]
}

/// Done-when 1: orbit keeps the pivot pixel fixed (≤ 1e-6 px), free and constrained.
#[test]
fn orbit_keeps_the_pivot_pixel_fixed() {
    let mut rng = Rng(0x1234_5678_9abc_def1);
    let mut worst = 0.0f64;
    for case in 0..400 {
        let mode = if case % 2 == 0 {
            OrbitMode::Free
        } else {
            OrbitMode::Constrained
        };
        let dist = 10f64.powf(rng.range(-2.0, 4.0));
        let mut cam = camera(&mut rng, dist, case % 4 >= 2);
        let start_px = random_px(&mut rng);
        let pivot = point_at(&cam, start_px, rng.range(0.5, 1.5));
        let before = pixel(&cam, pivot);
        let mut nav = NavController::new();
        nav.set_orbit_mode(mode);
        nav.begin(NavMode::Orbit, &cam, start_px, Some(pivot), Aabb3::EMPTY);
        for _ in 0..4 {
            let cursor = [
                start_px[0] + rng.range(-300.0, 300.0),
                start_px[1] + rng.range(-200.0, 200.0),
            ];
            nav.drag(&mut cam, cursor, SIZE);
            // A point behind the eye after a large orbit has no pixel; the pivot is never behind.
            let after = pixel(&cam, pivot);
            worst = worst.max(px_err(before, after));
        }
        nav.end();
        assert!(!nav.is_dragging());
    }
    eprintln!("orbit pivot: worst {worst:e} px");
    assert!(worst <= 1e-6, "pivot moved {worst} px");
}

/// Orbit about the scene centre on background, and 180° per viewport height.
#[test]
fn orbit_on_background_turns_180_degrees_per_view_height_about_the_scene_centre() {
    let mut cam = Camera {
        eye: Point3::new(0.0, -100.0, 0.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    let scene = Aabb3::new(Point3::splat(-10.0), Point3::splat(10.0));
    let mut nav = NavController::new();
    nav.begin(NavMode::Orbit, &cam, [100.0, 100.0], None, scene);
    assert_eq!(nav.pivot(), Some(Point3::ZERO));
    // Half a height to the right: 90° yaw, the camera moves to −X (model turns right).
    nav.drag(&mut cam, [100.0 + 360.0, 100.0], SIZE);
    assert!(
        cam.eye.distance(Point3::new(-100.0, 0.0, 0.0)) < 1e-9,
        "{}",
        cam.eye
    );
    // Dragging down raises the camera.
    nav.drag(&mut cam, [100.0, 160.0], SIZE);
    assert!(cam.eye.z > 0.0);
}

/// The turntable orbit keeps +Z up and clamps the pitch to ±89.9°.
#[test]
fn constrained_orbit_clamps_the_pitch_and_keeps_z_up() {
    let mut cam = Camera {
        eye: Point3::new(0.0, -100.0, 20.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    let mut nav = NavController::new();
    nav.set_orbit_mode(OrbitMode::Constrained);
    nav.begin(
        NavMode::Orbit,
        &cam,
        [0.0, 0.0],
        Some(Point3::ZERO),
        Aabb3::EMPTY,
    );
    nav.drag(&mut cam, [50.0, 5000.0], SIZE);
    assert_eq!(cam.up, Dir3::Z);
    let elevation = (-cam.forward().get().z).asin();
    assert!(
        (elevation - CONSTRAINED_PITCH_LIMIT).abs() < 1e-9,
        "{elevation}"
    );
    // The free orbit goes over the pole.
    let mut free = Camera {
        eye: Point3::new(0.0, -100.0, 20.0),
        ..cam
    };
    let mut nav = NavController::new();
    nav.begin(
        NavMode::Orbit,
        &free,
        [0.0, 0.0],
        Some(Point3::ZERO),
        Aabb3::EMPTY,
    );
    nav.drag(&mut free, [0.0, 600.0], SIZE);
    assert!(free.up.get().z < 0.0, "upside down past the pole");
}

/// Done-when 2: zoom to the cursor keeps the anchor pixel fixed (≤ 1e-6 px), perspective and
/// orthographic, over 1e-3..1e5 mm.
#[test]
fn zoom_keeps_the_anchor_pixel_fixed() {
    let mut rng = Rng(0x0bad_cafe_f00d_1234);
    let mut worst = 0.0f64;
    for ortho in [false, true] {
        for exp in -3..=5 {
            for _ in 0..20 {
                let dist = 10f64.powi(exp) * rng.range(1.0, 3.0);
                let mut cam = camera(&mut rng, dist, ortho);
                let cursor = random_px(&mut rng);
                let mut nav = NavController::new();
                // With a picked anchor at some depth.
                let anchor = point_at(&cam, cursor, rng.range(0.3, 2.0));
                for steps in [1.0, 3.5, -2.0, 0.25] {
                    nav.zoom(&mut cam, steps, cursor, Some(anchor), SIZE);
                    worst = worst.max(px_err(pixel(&cam, anchor), cursor));
                }
                // On background: the point on the target plane under the cursor.
                let on_plane = target_plane_point(&cam, cursor, SIZE);
                nav.zoom(&mut cam, 2.0, cursor, None, SIZE);
                worst = worst.max(px_err(pixel(&cam, on_plane), cursor));
            }
        }
    }
    eprintln!("zoom anchor: worst {worst:e} px");
    assert!(worst <= 1e-6, "anchor moved {worst} px");
}

/// Zoom scales the distance (perspective) or the height (orthographic) by 0.9 per step.
#[test]
fn zoom_steps_scale_distance_or_height() {
    let mut nav = NavController::new();
    let mut cam = Camera {
        eye: Point3::new(0.0, -100.0, 0.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    nav.zoom(&mut cam, 2.0, [640.0, 360.0], None, SIZE);
    assert!((cam.eye.distance(cam.target) - 81.0).abs() < 1e-9);
    // Near plane follows the distance (near-plane policy).
    assert!((cam.default_near() - 0.081).abs() < 1e-12);
    cam.projection = Projection::Orthographic { height: 50.0 };
    nav.zoom(&mut cam, -1.0, [640.0, 360.0], None, SIZE);
    let Projection::Orthographic { height } = cam.projection else {
        unreachable!()
    };
    assert!((height - 50.0 / 0.9).abs() < 1e-9);
}

/// Done-when 3: pan keeps the grabbed point under the cursor (≤ 1e-6 px).
#[test]
fn pan_keeps_the_grabbed_point_under_the_cursor() {
    let mut rng = Rng(0x5eed_5eed_1111_2222);
    let mut worst = 0.0f64;
    for case in 0..200 {
        let dist = 10f64.powf(rng.range(-3.0, 5.0));
        let mut cam = camera(&mut rng, dist, case % 2 == 1);
        let start_px = random_px(&mut rng);
        let picked = case % 3 != 0;
        let grab = if picked {
            point_at(&cam, start_px, rng.range(0.3, 2.0))
        } else {
            target_plane_point(&cam, start_px, SIZE)
        };
        let mut nav = NavController::new();
        nav.begin(
            NavMode::Pan,
            &cam,
            start_px,
            picked.then_some(grab),
            Aabb3::EMPTY,
        );
        for _ in 0..4 {
            let cursor = random_px(&mut rng);
            nav.drag(&mut cam, cursor, SIZE);
            worst = worst.max(px_err(pixel(&cam, grab), cursor));
        }
        nav.end();
    }
    eprintln!("pan grab: worst {worst:e} px");
    assert!(worst <= 1e-6, "grabbed point moved {worst} px");
}

/// Projected extent of the bounding sphere of `b` along x and y (sampled silhouette), px.
fn sphere_extent(cam: &Camera, b: &Aabb3, size: [u32; 2]) -> [f64; 2] {
    let (c, r) = (b.center(), b.diagonal() * 0.5);
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    let n = 200;
    for i in 0..n {
        let theta = PI * (f64::from(i) + 0.5) / f64::from(n);
        for j in 0..n {
            let phi = 2.0 * PI * f64::from(j) / f64::from(n);
            let p = c + Vec3::new(
                theta.sin() * phi.cos(),
                theta.sin() * phi.sin(),
                theta.cos(),
            ) * r;
            let q = cam.project(p, size).unwrap();
            for k in 0..2 {
                lo[k] = lo[k].min(q[k]);
                hi[k] = hi[k].max(q[k]);
            }
        }
    }
    [hi[0] - lo[0], hi[1] - lo[1]]
}

/// Done-when 4: after a fit all 8 corners are inside the view and the bounding sphere spans
/// ≥ 90 % of the smaller viewport dimension.
#[test]
fn fit_shows_all_corners_and_fills_the_smaller_dimension() {
    let mut rng = Rng(0xf17f_17f1_7f17_f17f);
    let mut least = f64::INFINITY;
    for case in 0..48 {
        let size = [[1280, 720], [720, 1280], [800, 800], [300, 900]][case % 4];
        let ortho = case % 8 >= 4;
        let dist = rng.range(1.0, 1e4);
        let mut cam = camera(&mut rng, dist, ortho);
        if !ortho {
            cam.projection = Projection::Perspective {
                fov_y: rng.range(0.4, 1.4),
            };
        }
        let centre = Vec3::new(
            rng.range(-1e4, 1e4),
            rng.range(-1e4, 1e4),
            rng.range(-1e3, 1e3),
        );
        let half = Vec3::new(
            rng.range(0.01, 500.0),
            rng.range(0.01, 500.0),
            rng.range(0.01, 500.0),
        );
        let b = Aabb3::new(centre - half, centre + half);
        let mut nav = NavController::new();
        nav.fit(&mut cam, b, size, false);
        for x in [b.min.x, b.max.x] {
            for y in [b.min.y, b.max.y] {
                for z in [b.min.z, b.max.z] {
                    let q = cam.project(Point3::new(x, y, z), size).unwrap();
                    assert!(
                        (0.0..=f64::from(size[0])).contains(&q[0])
                            && (0.0..=f64::from(size[1])).contains(&q[1])
                            && q[2] > 0.0,
                        "case {case}: corner at {q:?} outside {size:?}"
                    );
                }
            }
        }
        let ext = sphere_extent(&cam, &b, size);
        let (i, dim) = if size[0] <= size[1] {
            (0, size[0])
        } else {
            (1, size[1])
        };
        let frac = ext[i] / f64::from(dim);
        least = least.min(frac);
        assert!(frac >= 0.9, "case {case}: sphere spans {frac}");
        assert!(frac <= 1.0, "case {case}: sphere spans {frac}");
    }
    eprintln!("fit: smallest sphere span {least:.4}");
    assert!(least >= 0.9);
}

/// Done-when 5: switching projection keeps the apparent size at the target within 0.1 %.
#[test]
fn projection_switch_keeps_the_apparent_size_at_the_target() {
    let mut rng = Rng(0x0123_4567_89ab_cdef);
    for _ in 0..100 {
        let dist = 10f64.powf(rng.range(-2.0, 4.0));
        let mut cam = camera(&mut rng, dist, false);
        let mut nav = NavController::new();
        let target = cam.target;
        let size0 = cam.pixel_size_at(target, SIZE);
        nav.set_projection(&mut cam, Projection::Orthographic { height: 1.0 });
        assert!(matches!(cam.projection, Projection::Orthographic { .. }));
        let size1 = cam.pixel_size_at(target, SIZE);
        assert!((size1 / size0 - 1.0).abs() < 1e-3, "{size0} -> {size1}");
        let fov = rng.range(0.3, 1.3);
        nav.set_projection(&mut cam, Projection::Perspective { fov_y: fov });
        let size2 = cam.pixel_size_at(target, SIZE);
        assert!((size2 / size0 - 1.0).abs() < 1e-3, "{size0} -> {size2}");
        assert_eq!(cam.target, target);
        assert!(matches!(cam.projection, Projection::Perspective { fov_y } if fov_y == fov));
    }
}

/// Done-when 6: all 26 cube regions + Home map to their views, and a click on a region's centre
/// seen along the region's direction hits that region.
#[test]
fn cube_regions_table() {
    use CubeRegion as R;
    use StandardView as V;
    let iso = |x, y, z| V::Iso { x, y, z };
    #[rustfmt::skip]
    let table: [(CubeRegion, StandardView); 27] = [
        (R::Face(V::Front), V::Front),
        (R::Face(V::Back), V::Back),
        (R::Face(V::Left), V::Left),
        (R::Face(V::Right), V::Right),
        (R::Face(V::Top), V::Top),
        (R::Face(V::Bottom), V::Bottom),
        (R::Edge { a: V::Left, b: V::Front }, iso(-1, -1, 0)),
        (R::Edge { a: V::Left, b: V::Back }, iso(-1, 1, 0)),
        (R::Edge { a: V::Right, b: V::Front }, iso(1, -1, 0)),
        (R::Edge { a: V::Right, b: V::Back }, iso(1, 1, 0)),
        (R::Edge { a: V::Left, b: V::Bottom }, iso(-1, 0, -1)),
        (R::Edge { a: V::Left, b: V::Top }, iso(-1, 0, 1)),
        (R::Edge { a: V::Right, b: V::Bottom }, iso(1, 0, -1)),
        (R::Edge { a: V::Right, b: V::Top }, iso(1, 0, 1)),
        (R::Edge { a: V::Front, b: V::Bottom }, iso(0, -1, -1)),
        (R::Edge { a: V::Front, b: V::Top }, iso(0, -1, 1)),
        (R::Edge { a: V::Back, b: V::Bottom }, iso(0, 1, -1)),
        (R::Edge { a: V::Back, b: V::Top }, iso(0, 1, 1)),
        (R::Corner { x: -1, y: -1, z: -1 }, iso(-1, -1, -1)),
        (R::Corner { x: -1, y: -1, z: 1 }, iso(-1, -1, 1)),
        (R::Corner { x: -1, y: 1, z: -1 }, iso(-1, 1, -1)),
        (R::Corner { x: -1, y: 1, z: 1 }, iso(-1, 1, 1)),
        (R::Corner { x: 1, y: -1, z: -1 }, iso(1, -1, -1)),
        (R::Corner { x: 1, y: -1, z: 1 }, iso(1, -1, 1)),
        (R::Corner { x: 1, y: 1, z: -1 }, iso(1, 1, -1)),
        (R::Corner { x: 1, y: 1, z: 1 }, iso(1, 1, 1)),
        (R::Home, V::Home),
    ];
    let all = CubeRegion::all();
    assert_eq!(all.len(), 26);
    for (region, view) in table {
        assert_eq!(view_for_region(region), view, "{region:?}");
        if region == R::Home {
            assert!(!all.contains(&region));
            continue;
        }
        assert!(all.contains(&region), "{region:?} not in CubeRegion::all");
        // Seen along its own direction, the region's centre is the inset centre.
        let s = region.signs().unwrap();
        let dir = Vec3::new(f64::from(s[0]), f64::from(s[1]), f64::from(s[2]));
        let v = view_for_region(region);
        assert!(
            v.eye_direction().distance(dir.normalize()) < 1e-12,
            "{region:?}"
        );
        let cam = Camera {
            eye: dir.normalize() * 100.0,
            target: Point3::ZERO,
            up: Dir3::new_unchecked(v.up()),
            projection: Projection::Perspective { fov_y: 0.8 },
        };
        for inset in [110, 220] {
            let c = f64::from(inset) * 0.5;
            assert_eq!(
                view_cube_hit(&cam, [c, c], inset),
                Some(region),
                "{region:?} at {inset} px"
            );
        }
    }
    // Standard view orientations: Front looks along +Y, Top has +Y up, right = +X.
    for (v, right) in [
        (V::Front, Vec3::X),
        (V::Top, Vec3::X),
        (V::Bottom, Vec3::X),
        (V::Right, Vec3::Y),
    ] {
        let cam = Camera {
            eye: v.eye_direction() * 10.0,
            target: Point3::ZERO,
            up: Dir3::new_unchecked(v.up()),
            projection: Projection::Perspective { fov_y: 0.8 },
        };
        let (r, _, _) = cam.basis();
        assert!(r.distance(right) < 1e-12, "{v:?}: right {r}");
    }
    // The corner of the inset is off the cube; Home is never hit.
    let home = Camera {
        eye: Vec3::new(1.0, -1.0, 1.0) * 50.0,
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    assert_eq!(view_cube_hit(&home, [1.0, 1.0], 110), None);
}

/// Every pixel of the inset that hits the cube hits a region of a visible face, and all of them
/// are reachable from the Home view.
#[test]
fn cube_hit_from_home_reaches_the_three_visible_faces() {
    let cam = Camera {
        eye: StandardView::Home.eye_direction() * 50.0,
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    let mut seen = std::collections::HashSet::new();
    for y in 0..110 {
        for x in 0..110 {
            if let Some(r) = view_cube_hit(&cam, [f64::from(x) + 0.5, f64::from(y) + 0.5], 110) {
                let s = r.signs().unwrap();
                // Hits land on a visible face (+X, −Y or +Z); the border strips of a visible
                // face belong to the edges and corners it shares with hidden faces.
                assert!(s[0] > 0 || s[1] < 0 || s[2] > 0, "{r:?}");
                seen.insert(r);
            }
        }
    }
    // The 7 regions made of visible faces only (3 faces, 3 edges, 1 corner) …
    for s in [
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, 1],
        [1, -1, 0],
        [1, 0, 1],
        [0, -1, 1],
        [1, -1, 1],
    ] {
        assert!(seen.contains(&CubeRegion::from_signs(s).unwrap()), "{s:?}");
    }
    // … and every other region touching a visible face: 26 minus the 7 of the hidden faces.
    assert_eq!(seen.len(), 19, "{seen:?}");
}

/// Done-when 7: `look_at_plane` puts plane x to screen right and the normal toward the viewer
/// (±1e-9), animated or not.
#[test]
fn look_at_plane_puts_x_right_and_the_normal_toward_the_viewer() {
    let mut rng = Rng(0xabcd_ef01_2345_6789);
    let tol = amg_core::Tolerance::default();
    for case in 0..100 {
        let normal = Dir3::new(rng.unit(), &tol).unwrap();
        let origin = Vec3::new(rng.range(-1e4, 1e4), rng.range(-1e4, 1e4), 0.0);
        let plane = Plane::from_normal(origin, normal);
        let mut cam = camera(&mut rng, 200.0, case % 2 == 0);
        let bounds = if case % 3 == 0 {
            Aabb3::EMPTY
        } else {
            Aabb3::new(origin - Vec3::splat(5.0), origin + Vec3::splat(5.0))
        };
        let mut nav = NavController::new();
        let animate = case % 4 == 1;
        nav.look_at_plane(&mut cam, &plane, bounds, SIZE, animate);
        if animate {
            assert!(nav.is_animating());
            let mut frames = 0;
            while nav.tick(&mut cam, Duration::from_millis(16)) {
                frames += 1;
                assert!(frames < 100);
            }
        }
        let (right, up, forward) = cam.basis();
        assert!(
            right.distance(plane.x.get()) <= 1e-9,
            "right {right} vs {}",
            plane.x.get()
        );
        assert!(up.distance(plane.y.get()) <= 1e-9);
        assert!((-forward).distance(normal.get()) <= 1e-9);
        assert!((cam.eye - cam.target).normalize().distance(normal.get()) <= 1e-9);
        if bounds.is_empty() {
            assert!(cam.target.distance(plane.origin) < 1e-9);
        }
    }
}

/// Transitions take 300 ms with smoothstep easing and land exactly on the destination.
#[test]
fn transitions_take_300_ms_and_land_exactly() {
    let mut cam = Camera {
        eye: Point3::new(0.0, -100.0, 30.0),
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    let bounds = Aabb3::new(Point3::splat(-20.0), Point3::splat(20.0));
    let mut instant = cam;
    let mut nav = NavController::new();
    nav.set_view(&mut instant, StandardView::Top, bounds, SIZE, false);
    let start = cam;
    nav.set_view(&mut cam, StandardView::Top, bounds, SIZE, true);
    assert_eq!(cam, start, "nothing moves before the first tick");
    let mut elapsed = Duration::ZERO;
    let mut last_angle = 0.0;
    while nav.tick(&mut cam, Duration::from_millis(10)) {
        elapsed += Duration::from_millis(10);
        // Monotonic progress of the orientation.
        let angle = start.forward().get().angle_between(cam.forward().get());
        assert!(angle >= last_angle - 1e-12);
        last_angle = angle;
        // The target stays put (same bounds centre), the distance is between the two.
        assert!(cam.target.distance(Point3::ZERO) < 1e-9);
    }
    elapsed += Duration::from_millis(10);
    assert_eq!(elapsed, TRANSITION);
    assert_eq!(cam, instant, "lands exactly on the destination");
    assert!(!nav.is_animating());
    // A drag cancels a running transition where it is.
    nav.set_view(&mut cam, StandardView::Front, bounds, SIZE, true);
    nav.tick(&mut cam, Duration::from_millis(100));
    let mid = cam;
    nav.begin(NavMode::Orbit, &cam, [0.0, 0.0], None, bounds);
    assert!(!nav.is_animating());
    assert!(!nav.tick(&mut cam, Duration::from_millis(100)));
    assert_eq!(cam, mid);
}

/// Smoothstep eases in and out; interpolation of identical cameras is the identity.
#[test]
fn interpolation_endpoints() {
    assert_eq!(smoothstep(0.0), 0.0);
    assert_eq!(smoothstep(1.0), 1.0);
    assert!((smoothstep(0.5) - 0.5).abs() < 1e-15);
    let a = Camera {
        eye: Point3::new(3.0, -40.0, 12.0),
        target: Point3::new(1.0, 2.0, 3.0),
        up: Dir3::Z,
        projection: Projection::Orthographic { height: 30.0 },
    };
    let b = interpolate(&a, &a, 0.37);
    assert!(b.eye.distance(a.eye) < 1e-9 && b.target == a.target);
}

/// The view-cube inset is in the top-right corner, shades only visible faces, and labels the
/// faces that face the viewer.
#[test]
fn view_cube_inset_layout_and_labels() {
    let cube = ViewCube::for_scale(2.0);
    assert_eq!((cube.size_px, cube.margin_px), (220, 20));
    assert_eq!(cube.rect_px([1920, 1080]), [1920 - 240, 20, 220, 220]);
    assert_eq!(cube.rect_px([100, 100]), [0, 0, 100, 100]);
    let home = Camera {
        eye: StandardView::Home.eye_direction() * 50.0,
        target: Point3::ZERO,
        up: Dir3::Z,
        projection: Projection::Perspective { fov_y: 0.8 },
    };
    let inset = cube.inset(&home, [1920, 1080]);
    // Three visible faces × 9 quads, and the 9 visible cube edges.
    assert_eq!(inset.overlays.primitive_count(), 27 + 9);
    assert_eq!(inset.overlays.label_count(), 3);
    let mut labels = Vec::new();
    inset
        .overlays
        .project_labels(&inset.camera, [220, 220], [0.0, 0.0], &mut labels);
    let names: Vec<_> = labels.iter().map(|l| l.text.as_str()).collect();
    for n in ["FRONT", "RIGHT", "TOP"] {
        assert!(names.contains(&n), "{names:?}");
    }
    // The FRONT label sits on the front face's centre, which hits the Front region.
    let front = labels.iter().find(|l| l.text == "FRONT").unwrap();
    let px = [f64::from(front.pos_px[0]), f64::from(front.pos_px[1])];
    assert_eq!(
        view_cube_hit(&home, px, 220),
        Some(CubeRegion::Face(StandardView::Front))
    );
    // Face-on, only the front face is drawn and labelled.
    let front_cam = Camera {
        eye: Point3::new(0.0, -50.0, 0.0),
        ..home
    };
    let inset = cube.inset(&front_cam, [1920, 1080]);
    assert_eq!(inset.overlays.label_count(), 1);
}
