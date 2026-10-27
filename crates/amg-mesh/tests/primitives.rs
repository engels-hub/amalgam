//! MESH-24 tests: primitive fitting (RANSAC + Levenberg–Marquardt) on sampled surfaces and on
//! segmented tessellated parts.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::print_stderr
)]

#[path = "segment_common/mod.rs"]
mod parts;

use std::f64::consts::{PI, TAU};

use amg_core::{Point3, Vec3};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::{
    FitOpts, FitResult, IndexedMesh, MeshError, Primitive, PrimitiveKind, SegmentOpts,
    fit_group_primitives, fit_primitive, segment_face_groups,
};

// ------------------------------------------------------------------ sampling

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform in [0, 1).
    fn uniform(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.uniform()
    }
    /// Standard normal (Box–Muller).
    fn gauss(&mut self) -> f64 {
        let u = self.uniform().max(1e-300);
        let v = self.uniform();
        (-2.0 * u.ln()).sqrt() * (TAU * v).cos()
    }
    fn unit(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(self.gauss(), self.gauss(), self.gauss());
            if v.length() > 1e-6 {
                return v.normalize();
            }
        }
    }
}

/// A random pose: unit axis, frame (e1, e2) normal to it, and a centre in [-50, 50]^3.
fn pose(rng: &mut Rng) -> (Vec3, Vec3, Vec3, Point3) {
    let a = rng.unit();
    let (e1, e2) = a.any_orthonormal_pair();
    let c = Point3::new(
        rng.range(-50.0, 50.0),
        rng.range(-50.0, 50.0),
        rng.range(-50.0, 50.0),
    );
    (a, e1, e2, c)
}

/// The test primitive of each kind (mm): plane 50 x 50 mm patch; sphere r 20 mm, the cap
/// within 70 degrees of a pole; cylinder r 10 mm, 40 mm long, 270 degrees around; cone half
/// angle 25 degrees, 5 to 35 mm from the apex; torus R 25 mm, r 8 mm, full.
fn truth(kind: PrimitiveKind, rng: &mut Rng) -> Primitive {
    let (a, _, _, c) = pose(rng);
    match kind {
        PrimitiveKind::Plane => Primitive::Plane {
            origin: c,
            normal: a,
        },
        PrimitiveKind::Sphere => Primitive::Sphere {
            center: c,
            radius: 20.0,
        },
        PrimitiveKind::Cylinder => Primitive::Cylinder {
            origin: c,
            axis: a,
            radius: 10.0,
        },
        PrimitiveKind::Cone => Primitive::Cone {
            apex: c,
            axis: a,
            half_angle: 25f64.to_radians(),
        },
        PrimitiveKind::Torus => Primitive::Torus {
            center: c,
            axis: a,
            major: 25.0,
            minor: 8.0,
        },
    }
}

/// `n` points on `prim` with exact unit normals.
fn sample(prim: &Primitive, n: usize, rng: &mut Rng) -> (Vec<Point3>, Vec<Vec3>) {
    let mut pts = Vec::with_capacity(n);
    let mut nrm = Vec::with_capacity(n);
    for _ in 0..n {
        let (p, v) = match *prim {
            Primitive::Plane { origin, normal } => {
                let (e1, e2) = normal.any_orthonormal_pair();
                let p = origin + e1 * rng.range(-25.0, 25.0) + e2 * rng.range(-25.0, 25.0);
                (p, normal)
            }
            Primitive::Sphere { center, radius } => {
                let v = loop {
                    let v = rng.unit();
                    if v.z.acos() < 70f64.to_radians() {
                        break v;
                    }
                };
                (center + v * radius, v)
            }
            Primitive::Cylinder {
                origin,
                axis,
                radius,
            } => {
                let (e1, e2) = axis.any_orthonormal_pair();
                let th = rng.range(0.0, 1.5 * PI);
                let radial = e1 * th.cos() + e2 * th.sin();
                (
                    origin + axis * rng.range(-20.0, 20.0) + radial * radius,
                    radial,
                )
            }
            Primitive::Cone {
                apex,
                axis,
                half_angle,
            } => {
                let (e1, e2) = axis.any_orthonormal_pair();
                let th = rng.range(0.0, TAU);
                let radial = e1 * th.cos() + e2 * th.sin();
                let h = rng.range(5.0, 35.0);
                (
                    apex + axis * h + radial * (h * half_angle.tan()),
                    radial * half_angle.cos() - axis * half_angle.sin(),
                )
            }
            Primitive::Torus {
                center,
                axis,
                major,
                minor,
            } => {
                let (e1, e2) = axis.any_orthonormal_pair();
                let (u, v) = (rng.range(0.0, TAU), rng.range(0.0, TAU));
                let radial = e1 * u.cos() + e2 * u.sin();
                let n = radial * v.cos() + axis * v.sin();
                (center + radial * major + n * minor, n)
            }
        };
        pts.push(p);
        nrm.push(v);
    }
    (pts, nrm)
}

/// Angle between two lines (unsigned directions), accurate near zero.
fn angle(a: Vec3, b: Vec3) -> f64 {
    a.cross(b).length().atan2(a.dot(b).abs())
}

/// Distance of `p` from the line through `o` along unit `a`.
fn line_dist(p: Point3, o: Point3, a: Vec3) -> f64 {
    let v = p - o;
    (v - a * v.dot(a)).length()
}

/// Parameter errors of `fit` against `truth`: (largest radius / length error mm, axis angle
/// rad, largest position error mm).
fn errors(fit: &Primitive, truth: &Primitive) -> (f64, f64, f64) {
    match (fit, truth) {
        (
            Primitive::Plane { origin, normal },
            Primitive::Plane {
                origin: o2,
                normal: n2,
            },
        ) => (0.0, angle(*normal, *n2), (*o2 - *origin).dot(*normal).abs()),
        (
            Primitive::Sphere { center, radius },
            Primitive::Sphere {
                center: c2,
                radius: r2,
            },
        ) => ((radius - r2).abs(), 0.0, (*center - *c2).length()),
        (
            Primitive::Cylinder {
                origin,
                axis,
                radius,
            },
            Primitive::Cylinder {
                origin: o2,
                axis: a2,
                radius: r2,
            },
        ) => (
            (radius - r2).abs(),
            angle(*axis, *a2),
            line_dist(*o2, *origin, *axis),
        ),
        (
            Primitive::Cone {
                apex,
                axis,
                half_angle,
            },
            Primitive::Cone {
                apex: p2,
                axis: a2,
                half_angle: h2,
            },
        ) => (
            // The half-angle error as a radius error 35 mm from the apex.
            35.0 * (half_angle.tan() - h2.tan()).abs(),
            angle(*axis, *a2).max((half_angle - h2).abs()),
            (*apex - *p2).length(),
        ),
        (
            Primitive::Torus {
                center,
                axis,
                major,
                minor,
            },
            Primitive::Torus {
                center: c2,
                axis: a2,
                major: m2,
                minor: n2,
            },
        ) => (
            (major - m2).abs().max((minor - n2).abs()),
            angle(*axis, *a2),
            (*center - *c2).length(),
        ),
        _ => (f64::INFINITY, f64::INFINITY, f64::INFINITY),
    }
}

const KINDS: [PrimitiveKind; 5] = [
    PrimitiveKind::Plane,
    PrimitiveKind::Sphere,
    PrimitiveKind::Cylinder,
    PrimitiveKind::Cone,
    PrimitiveKind::Torus,
];

/// Characteristic size of the test primitive of `kind` (mm), for relative errors.
fn size(kind: PrimitiveKind) -> f64 {
    match kind {
        PrimitiveKind::Plane => 50.0,
        PrimitiveKind::Sphere => 20.0,
        PrimitiveKind::Cylinder => 10.0,
        PrimitiveKind::Cone => 35.0,
        PrimitiveKind::Torus => 25.0,
    }
}

// ------------------------------------------------------------------ sampled surfaces

/// Done-when: noise-free samples recover the parameters within 1e-6 relative (2 000 points,
/// 8 random poses per kind; lengths relative to the primitive's size, angles in radians).
#[test]
fn noise_free_samples_recover_parameters() {
    for kind in KINDS {
        let mut worst = (0.0f64, 0.0f64, 0.0f64);
        for trial in 0..8u64 {
            let mut rng = Rng(1000 + trial * 31 + kind as u64);
            let t = truth(kind, &mut rng);
            let (pts, nrm) = sample(&t, 2000, &mut rng);
            let r = fit_primitive(&pts, &nrm, kind, &FitOpts::default())
                .unwrap_or_else(|| panic!("{kind:?} trial {trial}: no fit"));
            assert_eq!(r.support, 2000, "{kind:?} trial {trial}");
            let (dr, da, dp) = errors(&r.primitive, &t);
            let s = size(kind);
            assert!(
                dr / s < 1e-6 && da < 1e-6 && dp / s < 1e-6,
                "{kind:?} trial {trial}: radius {dr:e} angle {da:e} position {dp:e}\n{:?}\n{t:?}",
                r.primitive
            );
            assert!(r.rms < 1e-6 * s && r.max_dev < 1e-6 * s, "{r:?}");
            worst = (worst.0.max(dr / s), worst.1.max(da), worst.2.max(dp / s));
        }
        eprintln!(
            "{kind:?} noise-free worst relative errors: radius {:.1e}, angle {:.1e} rad, position {:.1e}",
            worst.0, worst.1, worst.2
        );
    }
}

/// Done-when: at sigma = 0.05 mm noise (isotropic Gaussian per axis on the positions; normals
/// tilted by Gaussian noise of 2 degrees per component), radius errors are below 0.1 mm and axis
/// errors below 0.5 degrees. 5 000 points, 8 poses per kind, epsilon 0.2 mm (4 sigma).
#[test]
fn noisy_samples_are_within_tolerance() {
    let sigma = 0.05;
    let opts = FitOpts {
        epsilon: 4.0 * sigma,
        ..FitOpts::default()
    };
    for kind in KINDS {
        let mut worst = (0.0f64, 0.0f64);
        for trial in 0..8u64 {
            let mut rng = Rng(7000 + trial * 17 + kind as u64);
            let t = truth(kind, &mut rng);
            let (mut pts, mut nrm) = sample(&t, 5000, &mut rng);
            for (p, n) in pts.iter_mut().zip(&mut nrm) {
                *p += Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * sigma;
                let tilt = 2f64.to_radians();
                *n = (*n + Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * tilt).normalize();
            }
            let r = fit_primitive(&pts, &nrm, kind, &opts)
                .unwrap_or_else(|| panic!("{kind:?} trial {trial}: no fit"));
            let (dr, da, _) = errors(&r.primitive, &t);
            assert!(
                dr < 0.1 && da < 0.5f64.to_radians(),
                "{kind:?} trial {trial}: radius {dr} mm, axis {} deg\n{r:?}",
                da.to_degrees()
            );
            assert!(r.support >= 4500, "{kind:?} trial {trial}: {}", r.support);
            assert!(
                (r.rms - sigma).abs() < 0.2 * sigma,
                "{kind:?}: rms {}",
                r.rms
            );
            worst = (worst.0.max(dr), worst.1.max(da));
        }
        eprintln!(
            "{kind:?} sigma 0.05: worst radius error {:.4} mm, worst axis error {:.4} deg",
            worst.0,
            worst.1.to_degrees()
        );
    }
}

/// Done-when: the same seed gives the same result (bit for bit), on noisy input.
#[test]
fn same_seed_gives_same_result() {
    for kind in KINDS {
        let mut rng = Rng(42 + kind as u64);
        let t = truth(kind, &mut rng);
        let (mut pts, nrm) = sample(&t, 3000, &mut rng);
        for p in &mut pts {
            *p += Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * 0.05;
        }
        let opts = FitOpts {
            epsilon: 0.2,
            seed: 99,
            ..FitOpts::default()
        };
        let a = fit_primitive(&pts, &nrm, kind, &opts).unwrap();
        let b = fit_primitive(&pts, &nrm, kind, &opts).unwrap();
        assert_eq!(a, b, "{kind:?}");
        // Another seed lands on the same least-squares fit up to the convergence tolerance.
        let c = fit_primitive(&pts, &nrm, kind, &FitOpts { seed: 7, ..opts }).unwrap();
        let (dr, da, _) = errors(&a.primitive, &c.primitive);
        assert!(dr < 1e-3 && da < 1e-4, "{kind:?}: {dr} {da}");
    }
}

/// RANSAC finds the primitive among 30 % uniform outliers in the bounding box.
#[test]
fn outliers_are_rejected() {
    for kind in KINDS {
        let mut rng = Rng(555 + kind as u64);
        let t = truth(kind, &mut rng);
        let (mut pts, mut nrm) = sample(&t, 3500, &mut rng);
        let (lo, hi) = pts.iter().fold(
            (Vec3::splat(f64::INFINITY), Vec3::splat(f64::NEG_INFINITY)),
            |(lo, hi), p| (lo.min(*p), hi.max(*p)),
        );
        for _ in 0..1500 {
            pts.push(Point3::new(
                rng.range(lo.x, hi.x),
                rng.range(lo.y, hi.y),
                rng.range(lo.z, hi.z),
            ));
            nrm.push(rng.unit());
        }
        let r = fit_primitive(&pts, &nrm, kind, &FitOpts::default()).unwrap();
        let (dr, da, dp) = errors(&r.primitive, &t);
        // Outliers that land inside the epsilon band (and pass the normal test) are inliers
        // of the least-squares fit, so the result is close to, not exactly, the truth.
        assert!(
            dr < 1e-3 && da < 1e-4 && dp < 1e-3,
            "{kind:?}: {dr} {da} {dp}"
        );
        assert!((3500..3600).contains(&r.support), "{kind:?}: {}", r.support);
    }
}

#[test]
fn degenerate_input_gives_none() {
    let opts = FitOpts::default();
    let p = [Point3::ZERO; 3];
    let n = [Vec3::Z; 3];
    for kind in KINDS {
        assert!(fit_primitive(&[], &[], kind, &opts).is_none());
        assert!(fit_primitive(&p, &n[..2], kind, &opts).is_none());
        assert!(fit_primitive(&p, &n, kind, &opts).is_none());
    }
    // A plane is not a cylinder, cone or torus (parallel normals give no candidate).
    let mut rng = Rng(3);
    let t = truth(PrimitiveKind::Plane, &mut rng);
    let (pts, nrm) = sample(&t, 500, &mut rng);
    for kind in [PrimitiveKind::Cylinder, PrimitiveKind::Cone] {
        assert!(fit_primitive(&pts, &nrm, kind, &opts).is_none(), "{kind:?}");
    }
}

// ------------------------------------------------------------------ tessellated parts

fn segmented(i: usize, noise: bool) -> IndexedMesh {
    let mut m = parts::build(i, noise).mesh;
    segment_face_groups(&mut m, &SegmentOpts::default(), &JobCtx::detached()).unwrap();
    m
}

fn kinds_of(fits: &[(amg_mesh::FaceGroupId, FitResult)]) -> Vec<PrimitiveKind> {
    let mut k: Vec<PrimitiveKind> = fits.iter().map(|(_, r)| r.primitive.kind()).collect();
    k.sort_by_key(|k| *k as u8);
    k
}

/// Kinds of the true faces of the revolved parts (segment_common: profile segments of the
/// stepped shaft, cone on cylinder, domed cylinder, pill disc, V pulley, sphere zone).
fn expected(i: usize) -> Vec<PrimitiveKind> {
    use PrimitiveKind::*;
    let mut k = match i {
        0 => vec![Plane, Cylinder, Plane, Cylinder, Cone, Plane],
        2 => vec![Plane, Cylinder, Cone],
        3 => vec![Plane, Cylinder, Sphere],
        4 => vec![Plane, Torus, Plane],
        5 => vec![Plane, Cylinder, Cone, Cone, Cylinder, Plane],
        6 => vec![Plane, Sphere, Plane],
        _ => unreachable!(),
    };
    k.sort_by_key(|k| *k as u8);
    k
}

/// Segmented revolved parts (about 1 mm triangles, 72 steps around): every group gets the
/// right kind, with and without the 0.02 mm vertex noise; without noise the vertices are on the
/// surfaces, so the fits are exact.
#[test]
fn segmented_parts_get_the_right_primitives() {
    for noise in [false, true] {
        for i in [0, 2, 3, 4, 5, 6] {
            let mut m = segmented(i, noise);
            let fits =
                fit_group_primitives(&mut m, &FitOpts::default(), &JobCtx::detached()).unwrap();
            assert_eq!(kinds_of(&fits), expected(i), "part {i} noise {noise}");
            for (g, r) in &fits {
                assert_eq!(
                    m.groups.info(*g).unwrap().primitive.as_ref(),
                    Some(&r.primitive)
                );
                let n = m.groups.faces_of(*g).count();
                assert!(n > 0);
                if noise {
                    assert!(r.max_dev < 0.1, "part {i}: {r:?}");
                } else {
                    assert!(r.max_dev < 1e-6, "part {i}: {r:?}");
                }
            }
        }
    }
}

/// Stepped shaft: radii 10 and 7 mm, a 45 degree chamfer cone, all coaxial.
#[test]
fn stepped_shaft_parameters() {
    let mut m = segmented(0, false);
    let fits = fit_group_primitives(&mut m, &FitOpts::default(), &JobCtx::detached()).unwrap();
    let mut radii = Vec::new();
    let mut axes = Vec::new();
    for (_, r) in &fits {
        match r.primitive {
            Primitive::Cylinder { axis, radius, .. } => {
                radii.push(radius);
                axes.push(axis);
            }
            Primitive::Cone {
                axis, half_angle, ..
            } => {
                assert!((half_angle - PI / 4.0).abs() < 1e-9, "{half_angle}");
                axes.push(axis);
            }
            Primitive::Plane { normal, .. } => axes.push(normal),
            _ => panic!("{r:?}"),
        }
    }
    radii.sort_by(f64::total_cmp);
    assert!(
        (radii[0] - 7.0).abs() < 1e-9 && (radii[1] - 10.0).abs() < 1e-9,
        "{radii:?}"
    );
    for a in &axes {
        assert!(angle(*a, axes[0]) < 1e-9);
    }
}

/// Same input and seed: identical results and stored tables for 1, 4 and 13 threads.
#[test]
fn group_fitting_is_deterministic_across_thread_counts() {
    let base = segmented(5, true);
    let run = || {
        let mut m = base.clone();
        let f = fit_group_primitives(&mut m, &FitOpts::default(), &JobCtx::detached()).unwrap();
        (m, f)
    };
    let reference = run();
    for threads in [1, 4, 13] {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        let got = pool.install(run);
        assert_eq!(got.1, reference.1, "{threads} threads");
        assert_eq!(got.0, reference.0, "{threads} threads");
    }
}

#[test]
fn restricted_kinds_and_bad_options() {
    let mut m = segmented(0, false);
    let opts = FitOpts {
        kinds: vec![PrimitiveKind::Plane],
        ..FitOpts::default()
    };
    let fits = fit_group_primitives(&mut m, &opts, &JobCtx::detached()).unwrap();
    assert_eq!(kinds_of(&fits), vec![PrimitiveKind::Plane; 3]);
    // Groups without an acceptable fit have no primitive.
    let unfitted = m
        .groups
        .ids()
        .filter(|g| m.groups.faces_of(*g).next().is_some())
        .filter(|g| m.groups.info(*g).unwrap().primitive.is_none())
        .count();
    assert_eq!(unfitted, 3);
    for bad in [
        FitOpts {
            epsilon: 0.0,
            ..FitOpts::default()
        },
        FitOpts {
            normal_threshold: 2.0,
            ..FitOpts::default()
        },
        FitOpts {
            min_coverage: 1.5,
            ..FitOpts::default()
        },
    ] {
        assert!(matches!(
            fit_group_primitives(&mut m, &bad, &JobCtx::detached()),
            Err(MeshError::InvalidOption { .. })
        ));
    }
}

#[test]
fn cancellation_leaves_the_mesh_unchanged() {
    let mut m = segmented(0, false);
    let before = m.clone();
    let token = CancelToken::new();
    token.cancel();
    let r = fit_group_primitives(&mut m, &FitOpts::default(), &JobCtx::with_token(token));
    assert!(matches!(r, Err(MeshError::Cancelled)));
    assert_eq!(m, before);
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// No budget in the vision doc; reported for reference, and bounded at 10 s only with
/// `AMG_STRICT_TIMING=1`. Stepped shaft at about 1 M triangles, segmented, then fitted.
#[test]
fn timing_one_million_triangles() {
    let mut m = parts::shaft_fine(800, 0.1, false).mesh;
    segment_face_groups(&mut m, &SegmentOpts::default(), &JobCtx::detached()).unwrap();
    let t = std::time::Instant::now();
    let fits = fit_group_primitives(&mut m, &FitOpts::default(), &JobCtx::detached()).unwrap();
    let secs = t.elapsed().as_secs_f64();
    eprintln!(
        "fit {} tris: {} groups fitted in {secs:.2} s",
        m.face_count(),
        fits.len()
    );
    assert_eq!(kinds_of(&fits), expected(0));
    if strict_timing() {
        assert!(secs < 10.0, "fitting took {secs:.2} s");
    }
}
