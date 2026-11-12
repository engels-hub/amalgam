//! AM-02: overhang classification, metrics, islands, stability and the support estimate.
//! Golden values: `tests/golden/am/overhang_metrics.toml` (note §3.3) and the shared S10
//! classification file `tests/golden/am/overhang_classes.toml`.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::print_stderr,
    clippy::cast_precision_loss,
    missing_docs,
    reason = "tests"
)]

mod overhang_shapes;

use std::f64::consts::{FRAC_PI_4, PI};
use std::fmt::Write as _;
use std::sync::Arc;

use amg_am::AmError;
use amg_am::overhang::{
    FaceClass, OverhangOpts, OverhangReport, SupportMethod, SupportOpts, analyze_overhang,
    classify_face, classify_faces, severity, shader_params, vertex_severity,
};
use amg_am::place::{Placement, ZMode, effective_transform, rotate_about_build_axis};
use amg_core::Transform;
use amg_core::glam::{DQuat, DVec2, DVec3};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::{BvhOpts, IndexedMesh, MeshSpatial};
use overhang_shapes::{box_union, cuboid, shelf, t_shape, uv_sphere};

// ------------------------------------------------------------------------- helpers

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn f(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn unit(&mut self) -> DVec3 {
        let z = 2.0 * self.f() - 1.0;
        let ph = 2.0 * PI * self.f();
        let s = (1.0 - z * z).max(0.0).sqrt();
        DVec3::new(s * ph.cos(), s * ph.sin(), z)
    }
}

fn spatial(m: IndexedMesh) -> MeshSpatial {
    MeshSpatial::build(Arc::new(m), &BvhOpts::default(), &JobCtx::detached()).unwrap()
}

fn opts(alpha_deg: f64, tol: f64, cell: Option<f64>) -> OverhangOpts {
    OverhangOpts {
        critical_angle: alpha_deg.to_radians(),
        contact_tolerance: tol,
        support: cell.map(|cell| SupportOpts {
            cell,
            max_rays: 1 << 22,
        }),
    }
}

fn place(m: &IndexedMesh, rot: DQuat) -> Transform {
    effective_transform(m, &Placement::on_plate(rot, DVec2::splat(110.0)))
}

fn analyze(s: &MeshSpatial, t: &Transform, o: &OverhangOpts) -> OverhangReport {
    analyze_overhang(s, t, o, &JobCtx::detached()).unwrap()
}

fn rel(a: f64, b: f64) -> f64 {
    if b == 0.0 {
        a.abs()
    } else {
        (a - b).abs() / b.abs()
    }
}

// ------------------------------------------------------------------ golden metrics (§3.3)

#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct MetricsFile {
    case: Vec<MetricsCase>,
}

#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct MetricsCase {
    name: String,
    shape: String,
    /// Rotation about plate X, degrees.
    #[serde(default)]
    tilt_x_deg: f64,
    alpha_deg: f64,
    contact_tolerance: f64,
    cell: f64,
    rel_tol: f64,
    overhang_area: Option<f64>,
    overhang_projected_area: Option<f64>,
    contact_area: Option<f64>,
    footprint_area: Option<f64>,
    height: Option<f64>,
    island_count: Option<usize>,
    stable: Option<bool>,
    support_volume: Option<f64>,
    on_plate_volume: Option<f64>,
    on_part_volume: Option<f64>,
}

fn shape(name: &str) -> IndexedMesh {
    match name {
        "t_shape" => t_shape(),
        "shelf" => shelf(),
        "cube10" => cuboid(DVec3::ZERO, DVec3::splat(10.0)),
        other => panic!("unknown shape {other}"),
    }
}

fn golden_path(file: &str) -> std::path::PathBuf {
    amg_testkit::golden_dir().join("am").join(file)
}

#[test]
fn golden_overhang_metrics() {
    let text = std::fs::read_to_string(golden_path("overhang_metrics.toml")).unwrap();
    let file: MetricsFile = toml::from_str(&text).unwrap();
    assert!(file.case.len() >= 5);
    let mut failures = Vec::new();
    for c in &file.case {
        let m = shape(&c.shape);
        let rot = DQuat::from_rotation_x(c.tilt_x_deg.to_radians());
        let t = place(&m, rot);
        let s = spatial(m);
        let r = analyze(
            &s,
            &t,
            &opts(c.alpha_deg, c.contact_tolerance, Some(c.cell)),
        );
        let mm = &r.metrics;
        let sup = mm.support.as_ref().unwrap();
        assert_eq!(sup.method, SupportMethod::Columns, "{}", c.name);
        assert!(!mm.approximate, "{}", c.name);
        let mut check = |what: &str, got: f64, want: Option<f64>| {
            if let Some(w) = want {
                let ok = if w == 0.0 {
                    got.abs() <= 1e-9
                } else {
                    rel(got, w) <= c.rel_tol
                };
                eprintln!("{}: {what} = {got} (want {w})", c.name);
                if !ok {
                    failures.push(format!(
                        "{}: {what} = {got}, want {w} ± {}",
                        c.name, c.rel_tol
                    ));
                }
            }
        };
        check("overhang_area", mm.overhang_area, c.overhang_area);
        check(
            "overhang_projected_area",
            mm.overhang_projected_area,
            c.overhang_projected_area,
        );
        check("contact_area", mm.contact_area, c.contact_area);
        check("footprint_area", mm.footprint_area, c.footprint_area);
        check("height", mm.height, c.height);
        check("support_volume", sup.volume, c.support_volume);
        check("on_plate_volume", sup.on_plate_volume, c.on_plate_volume);
        check("on_part_volume", sup.on_part_volume, c.on_part_volume);
        if let Some(n) = c.island_count {
            assert_eq!(mm.island_count, n, "{}: islands", c.name);
        }
        if c.stable.is_some() {
            assert_eq!(
                mm.stable, c.stable,
                "{}: stable (margin {:?})",
                c.name, mm.com_margin
            );
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Note §3.3 sphere r = 10 at α = 45°: cap area, projected area and support volume converge
/// (error at least halves per 4× triangles and ½ cell) and are within 1 % at ≥ 320 k
/// triangles and 0.1 mm cells. Contact tolerance 1e-6 so the analytic values (no contact
/// patch) apply.
#[test]
fn golden_overhang_sphere_convergence() {
    let r = 10.0_f64;
    let a = r * FRAC_PI_4.sin();
    let cap = 2.0 * PI * r * r * (1.0 - FRAC_PI_4.cos());
    let proj = PI * a * a;
    let vol = PI * a * a * r - (2.0 * PI / 3.0) * (r.powi(3) - (r * r - a * a).powf(1.5));
    assert!((cap - 184.030).abs() < 1e-3 && (proj - 157.080).abs() < 1e-3);
    assert!((vol - 216.882).abs() < 1e-3, "{vol}");
    let mut prev: Option<[f64; 3]> = None;
    for (n_lat, cell) in [(36, 0.8), (72, 0.4), (144, 0.2), (288, 0.1)] {
        let m = uv_sphere(r, n_lat);
        let tris = m.face_count();
        let t = place(&m, DQuat::IDENTITY);
        let s = spatial(m);
        let rep = analyze(&s, &t, &opts(45.0, 1e-6, Some(cell)));
        let mm = &rep.metrics;
        let sup = mm.support.as_ref().unwrap();
        let err = [
            rel(mm.overhang_area, cap),
            rel(mm.overhang_projected_area, proj),
            rel(sup.volume, vol),
        ];
        eprintln!(
            "sphere {tris} tris, cell {cell}: cap {:.4} proj {:.4} vol {:.4}; rel err {:.3e} {:.3e} {:.3e}",
            mm.overhang_area, mm.overhang_projected_area, sup.volume, err[0], err[1], err[2]
        );
        assert_eq!(sup.on_part_volume, 0.0);
        assert_eq!(mm.island_count, 0);
        if let Some(p) = prev {
            for k in 0..3 {
                assert!(
                    err[k] <= 0.5 * p[k],
                    "metric {k}: {} → {} did not halve",
                    p[k],
                    err[k]
                );
            }
        }
        prev = Some(err);
        if n_lat == 288 {
            assert!(tris >= 320_000, "{tris}");
            assert!(err.iter().all(|&e| e < 0.01), "{err:?}");
        }
    }
}

// ------------------------------------------------------------------ cubes and threshold

#[test]
fn cube_on_face_is_exact() {
    let m = cuboid(DVec3::ZERO, DVec3::splat(10.0));
    let t = place(&m, DQuat::IDENTITY);
    let s = spatial(m);
    let r = analyze(&s, &t, &opts(45.0, 0.1, Some(0.5)));
    let mm = &r.metrics;
    assert_eq!(mm.overhang_area, 0.0);
    assert!((mm.contact_area - 100.0).abs() <= 1e-9);
    assert!((mm.footprint_area - 100.0).abs() <= 1e-9);
    assert!((mm.height - 10.0).abs() <= 1e-9);
    assert_eq!(mm.support.as_ref().unwrap().volume, 0.0);
    assert_eq!(mm.stable, Some(true));
    assert!((mm.com_margin.unwrap() - 5.0).abs() <= 1e-9);
    assert!(r.islands.is_empty());
    assert_eq!(
        r.face_class
            .iter()
            .filter(|c| **c == FaceClass::Contact)
            .count(),
        2
    );
}

/// Note §3.3: cube balanced on an edge (45° about X). The two lower faces are at exactly
/// ψ = 45°: `Supported` at α = 45° (strict rule), `Overhang` at α = 46°; not stable.
#[test]
fn edge_balanced_cube_strict_threshold() {
    let m = cuboid(DVec3::ZERO, DVec3::splat(10.0));
    let t = place(&m, DQuat::from_rotation_x(FRAC_PI_4));
    let s = spatial(m);
    let ctx = JobCtx::detached();
    let (c45, psi) = classify_faces(s.mesh(), &t, &opts(45.0, 0.1, None), &ctx).unwrap();
    let lower: Vec<usize> = (0..psi.len())
        .filter(|&f| (psi[f] - FRAC_PI_4).abs() < 1e-9)
        .collect();
    assert_eq!(lower.len(), 4, "two lower faces = four triangles");
    assert!(c45.iter().all(|c| *c != FaceClass::Overhang));
    let (c46, _) = classify_faces(s.mesh(), &t, &opts(46.0, 0.1, None), &ctx).unwrap();
    for &f in &lower {
        assert_eq!(c46[f], FaceClass::Overhang);
    }
    assert_eq!(c46.iter().filter(|c| **c == FaceClass::Overhang).count(), 4);

    let r45 = analyze(&s, &t, &opts(45.0, 0.1, Some(0.25)));
    assert_eq!(r45.metrics.overhang_area, 0.0);
    assert_eq!(r45.metrics.support.as_ref().unwrap().volume, 0.0);
    assert_eq!(r45.metrics.stable, Some(false));
    assert!(r45.metrics.com_margin.unwrap() <= 1e-9);
    let r46 = analyze(&s, &t, &opts(46.0, 0.1, Some(0.25)));
    assert!((r46.metrics.overhang_area - 200.0).abs() <= 1e-9);
    // Projected area of two 10×10 faces at 45°: 2·100·cos 45°.
    assert!((r46.metrics.overhang_projected_area - 200.0 * FRAC_PI_4.cos()).abs() <= 1e-9);
    assert!(r46.metrics.support.as_ref().unwrap().volume > 0.0);
}

// ------------------------------------------------------------------ rotation invariance

/// Rotating about the build axis by 37° keeps the area metrics within 1e-9 relative and the
/// support volume within the cell bound `√2 · s · P · g_max` (P = support-footprint
/// perimeter, g_max = largest gap), note §3.1.
#[test]
fn rotation_about_build_axis_invariance() {
    let m = t_shape();
    let p0 = Placement::on_plate(DQuat::IDENTITY, DVec2::splat(110.0));
    let p1 = rotate_about_build_axis(&p0, 37f64.to_radians());
    let t0 = effective_transform(&m, &p0);
    let t1 = effective_transform(&m, &p1);
    let s = spatial(m);
    let o = opts(45.0, 0.1, Some(0.25));
    let (a, b) = (analyze(&s, &t0, &o).metrics, analyze(&s, &t1, &o).metrics);
    for (x, y, what) in [
        (a.overhang_area, b.overhang_area, "overhang"),
        (
            a.overhang_projected_area,
            b.overhang_projected_area,
            "projected",
        ),
        (a.contact_area, b.contact_area, "contact"),
        (a.footprint_area, b.footprint_area, "footprint"),
        (a.height, b.height, "height"),
    ] {
        assert!(rel(y, x) < 1e-9, "{what}: {x} vs {y}");
    }
    assert_eq!(a.island_count, b.island_count);
    assert_eq!(a.stable, b.stable);
    let (va, vb) = (a.support.unwrap(), b.support.unwrap());
    let bound = 2f64.sqrt() * va.cell * 80.0 * 20.0;
    eprintln!(
        "T rotated 37°: support {} → {} (Δ {:.3} mm³, bound {bound:.1}); rays {} / {}",
        va.volume,
        vb.volume,
        (vb.volume - va.volume).abs(),
        va.rays,
        vb.rays
    );
    assert!((vb.volume - va.volume).abs() <= bound);
    assert!(rel(vb.volume, 4000.0) < 0.02, "{}", vb.volume);
}

// ------------------------------------------------------------------ open mesh fallback

#[test]
fn open_mesh_uses_face_centroids() {
    let mut m = t_shape();
    // Drop the top of the bar (z = 25): the mesh is open.
    let pos = m.positions.clone();
    m.tris
        .retain(|t| !t.iter().all(|&i| pos[i as usize].z == 25.0));
    let m = IndexedMesh::new(m.positions, m.tris).unwrap();
    let t = place(&m, DQuat::IDENTITY);
    let s = spatial(m);
    let r = analyze(&s, &t, &opts(45.0, 0.1, Some(0.5)));
    let mm = &r.metrics;
    assert!(mm.approximate);
    assert_eq!(mm.stable, None);
    let sup = mm.support.as_ref().unwrap();
    assert_eq!(sup.method, SupportMethod::FaceCentroids);
    assert!((mm.overhang_area - 200.0).abs() <= 1e-9);
    // Every bar-underside sample misses the part, so h = 20 mm: 200 mm² · 20 mm.
    assert!(rel(sup.volume, 4000.0) < 1e-9, "{}", sup.volume);
    assert!(sup.on_part_volume.abs() < 1e-9);
    assert!(
        sup.rays > 100,
        "large faces are subsampled: {} rays",
        sup.rays
    );
    // Without support the flag still reports the open mesh.
    let r = analyze(&s, &t, &opts(45.0, 0.1, None));
    assert!(r.metrics.approximate && r.metrics.support.is_none());
}

// ------------------------------------------------------------------ determinism

#[test]
fn identical_across_thread_counts() {
    let mut m = uv_sphere(10.0, 48);
    // Add a floating box so there are on-part columns and an island-free second shell.
    m.append(&box_union(&[(
        DVec3::new(-4.0, -4.0, 12.0),
        DVec3::new(4.0, 4.0, 14.0),
    )]))
    .unwrap();
    let rot = DQuat::from_axis_angle(DVec3::new(1.0, 2.0, 3.0).normalize(), 0.7);
    let t = place(&m, rot);
    let s = spatial(m);
    let o = opts(45.0, 0.1, Some(0.2));
    let run = |n: usize| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(n)
            .build()
            .unwrap();
        pool.install(|| analyze(&s, &t, &o))
    };
    let base = run(1);
    assert!(base.metrics.support.as_ref().unwrap().volume > 0.0);
    for n in [4, 16] {
        let r = run(n);
        assert_eq!(r.face_class, base.face_class, "{n} threads");
        assert_eq!(
            r.down_angle.iter().map(|x| x.to_bits()).collect::<Vec<_>>(),
            base.down_angle
                .iter()
                .map(|x| x.to_bits())
                .collect::<Vec<_>>()
        );
        assert_eq!(r.islands, base.islands);
        assert_eq!(
            format!("{:?}", r.metrics),
            format!("{:?}", base.metrics),
            "{n} threads"
        );
    }
}

// ------------------------------------------------------------------ islands, severity, shader

#[test]
fn floating_sphere_is_an_island() {
    let mut m = cuboid(
        DVec3::new(-20.0, -20.0, -30.0),
        DVec3::new(-10.0, -10.0, -20.0),
    );
    m.append(&uv_sphere(5.0, 24)).unwrap();
    let t = place(&m, DQuat::IDENTITY);
    let s = spatial(m);
    let r = analyze(&s, &t, &opts(30.0, 0.1, None));
    assert_eq!(r.metrics.island_count, 1);
    let v = r.islands[0].0 as usize;
    let p = t.transform_point3(s.mesh().positions[v]);
    assert!((p.z - 25.0).abs() < 1e-9, "south pole of the sphere: {p}");
    // Flat ceilings joined to lower geometry are not islands.
    for m in [t_shape(), shelf()] {
        let t = place(&m, DQuat::IDENTITY);
        let r = analyze(&spatial(m), &t, &opts(45.0, 0.1, None));
        assert_eq!(r.metrics.island_count, 0);
    }
}

#[test]
fn severity_and_vertex_severity() {
    let a = 45f64.to_radians();
    assert_eq!(severity(0.0, a), 1.0);
    assert!(severity(a, a).abs() < 1e-12);
    assert_eq!(severity(PI / 2.0, a), 0.0);
    assert!((severity(a / 2.0, a) - ((a / 2.0).cos() - a.cos()) / (1.0 - a.cos())).abs() < 1e-15);

    let m = t_shape();
    let t = place(&m, DQuat::IDENTITY);
    let s = spatial(m);
    let r = analyze(&s, &t, &opts(45.0, 0.1, None));
    let vs = vertex_severity(s.mesh(), &r, a);
    assert_eq!(vs.len(), s.mesh().vertex_count());
    assert!(vs.iter().all(|v| (0.0..=1.0).contains(v)));
    // Bar-underside vertices see the ceiling; plate-contact vertices are 0.
    for (i, p) in s.mesh().positions.iter().enumerate() {
        let z = t.transform_point3(*p).z;
        if (z - 20.0).abs() < 1e-9 && (p.x < 10.0 || p.x > 20.0) {
            assert!(vs[i] > 0.0, "vertex {i}");
        }
        if z.abs() < 1e-9 {
            assert_eq!(vs[i], 0.0);
        }
    }
}

/// The shader-side rule (uniforms from `shader_params`, per face: normal, highest corner)
/// gives the CPU classes on a rotated, elevated sphere.
#[test]
fn shader_params_reproduce_classes() {
    let m = uv_sphere(10.0, 40);
    let rot = DQuat::from_axis_angle(DVec3::new(-0.3, 0.8, 0.2).normalize(), 2.1);
    let tr = effective_transform(
        &m,
        &Placement {
            rotation: rot,
            xy: DVec2::splat(50.0),
            z: ZMode::OnPlate { elevation: 3.0 },
        },
    );
    let o = opts(40.0, 0.4, None);
    let s = spatial(m);
    let r = analyze(&s, &tr, &o);
    assert!((r.z_min - 3.0).abs() < 1e-9);
    let sp = shader_params(&tr, r.z_min, &o);
    assert!((sp.build_dir_local - rot.inverse() * DVec3::Z).length() < 1e-12);
    let (n, w) = sp.contact_plane;
    let mesh = s.mesh();
    let mut contact = 0;
    for (f, tri) in mesh.tris.iter().enumerate() {
        let h = tri
            .iter()
            .map(|&i| n.dot(mesh.positions[i as usize]) + w)
            .fold(f64::NEG_INFINITY, f64::max);
        let normal = mesh.face_normal(amg_mesh::FaceId(f as u32));
        let c = classify_face(
            normal,
            sp.build_dir_local,
            sp.cos_critical,
            h,
            sp.contact_tolerance,
        );
        assert_eq!(c, r.face_class[f], "face {f}");
        contact += usize::from(c == FaceClass::Contact);
    }
    assert!(contact > 0);
}

// ------------------------------------------------------------------ errors

#[test]
fn invalid_options_and_cancel() {
    let m = cuboid(DVec3::ZERO, DVec3::splat(10.0));
    let t = place(&m, DQuat::IDENTITY);
    let s = spatial(m);
    let ctx = JobCtx::detached();
    for o in [
        opts(0.0, 0.1, None),
        opts(91.0, 0.1, None),
        opts(45.0, -1.0, None),
        opts(45.0, f64::NAN, None),
        opts(45.0, 0.1, Some(0.0)),
        OverhangOpts {
            support: Some(SupportOpts {
                cell: 0.5,
                max_rays: 0,
            }),
            ..opts(45.0, 0.1, None)
        },
    ] {
        assert!(matches!(
            analyze_overhang(&s, &t, &o, &ctx),
            Err(AmError::InvalidOption(_))
        ));
    }
    let tok = CancelToken::new();
    tok.cancel();
    let ctx = JobCtx::with_token(tok);
    assert!(matches!(
        analyze_overhang(&s, &t, &opts(45.0, 0.1, Some(0.5)), &ctx),
        Err(AmError::Cancelled)
    ));
    assert!(matches!(
        classify_faces(s.mesh(), &t, &opts(45.0, 0.1, None), &ctx),
        Err(AmError::Cancelled)
    ));
}

#[test]
fn ray_budget_grows_the_cell() {
    let m = cuboid(DVec3::ZERO, DVec3::new(40.0, 20.0, 5.0));
    let t = place(&m, DQuat::IDENTITY);
    let s = spatial(m);
    let mut o = opts(45.0, 0.1, Some(0.1));
    o.support = Some(SupportOpts {
        cell: 0.1,
        max_rays: 1000,
    });
    let sup = analyze(&s, &t, &o).metrics.support.unwrap();
    assert!(sup.rays <= 1000 && sup.rays > 500, "{}", sup.rays);
    assert!(sup.cell > 0.1);
}

// ------------------------------------------------------------------ shared S10 class file

const CLASS_CASES: usize = 5000;
const NEAR: f64 = 1e-6;

/// One case of `overhang_classes.toml`, all values as written in the file.
struct ClassCase {
    n: DVec3,
    d: DVec3,
    alpha: f64,
    height: f64,
    tol: f64,
}

impl ClassCase {
    fn expected(&self) -> (FaceClass, bool) {
        let n = self.n.normalize_or_zero();
        let d = self.d.normalize();
        let ca = self.alpha.cos();
        let class = classify_face(n, d, ca, self.height, self.tol);
        let c = -n.dot(d);
        let near = n != DVec3::ZERO
            && ((c - ca).abs() <= NEAR
                || c.abs() <= NEAR
                || (self.height - self.tol).abs() <= NEAR);
        (class, near)
    }
}

fn fmt9(x: f64) -> String {
    let s = format!("{x:.9}");
    if s == "-0.000000000" {
        "0.000000000".to_owned()
    } else {
        s
    }
}

/// The deterministic case list and the file text it produces.
fn generate_class_file() -> String {
    let mut rng = Rng(0x0A02_0000_5000);
    let mut out = String::new();
    out.push_str(
        "# AM-02 shared overhang classification cases for the S10 RND-08 shader test.\n\
         # Generated by crates/amg-am/tests/overhang.rs (golden_overhang_classes); regenerate with\n\
         # AMG_BLESS=1. Each row: [nx, ny, nz, dx, dy, dz, alpha, height, tol, class, near].\n\
         # n = face normal, d = build direction (both body-local; normalise before use, n = 0 is\n\
         # degenerate), alpha = critical angle (rad), height = the face's highest point above the\n\
         # item's lowest point (mm), tol = contact tolerance (mm).\n\
         # class: 0 Supported, 1 Overhang, 2 Contact, 3 Degenerate, by the rule (in this order):\n\
         #   Degenerate if n = 0; c = -dot(n, d); Contact if c > 0 and height <= tol;\n\
         #   Overhang if c > cos(alpha); else Supported.\n\
         # near = 1 when |c - cos(alpha)|, |c| or |height - tol| <= 1e-6: only those rows may\n\
         # disagree with an f32 implementation.\n",
    );
    let _ = writeln!(out, "count = {CLASS_CASES}");
    out.push_str("cases = [\n");
    let tols = [0.025, 0.05, 0.1, 0.2];
    for i in 0..CLASS_CASES {
        let d = rng.unit();
        let alpha = (5.0 + 80.0 * rng.f()).to_radians();
        let tol = tols[(rng.next() % 4) as usize];
        let n = match i % 50 {
            0 => DVec3::ZERO,
            k if k % 4 == 1 => {
                // Near the critical angle: c = cos α + δ, |δ| log-uniform in [1e-9, 1e-2].
                let delta =
                    10f64.powf(-9.0 + 7.0 * rng.f()) * if rng.next() & 1 == 0 { 1.0 } else { -1.0 };
                let c = (alpha.cos() + delta).clamp(-1.0, 1.0);
                let (u, _) = d.any_orthonormal_pair();
                let q = DQuat::from_axis_angle(d, 2.0 * PI * rng.f());
                -c * d + (1.0 - c * c).sqrt() * (q * u)
            }
            _ => rng.unit(),
        };
        let height = match rng.next() % 10 {
            0..=3 => 2.0 * tol * rng.f(),
            4 => tol + (2.0 * rng.f() - 1.0) * 1e-7,
            _ => 50.0 * rng.f(),
        };
        let vals = [n.x, n.y, n.z, d.x, d.y, d.z, alpha, height];
        let parsed: Vec<f64> = vals.iter().map(|v| fmt9(*v).parse().unwrap()).collect();
        let case = ClassCase {
            n: DVec3::new(parsed[0], parsed[1], parsed[2]),
            d: DVec3::new(parsed[3], parsed[4], parsed[5]),
            alpha: parsed[6],
            height: parsed[7],
            tol,
        };
        let (class, near) = case.expected();
        let row: Vec<String> = vals.iter().map(|v| fmt9(*v)).collect();
        let _ = writeln!(
            out,
            "  [{}, {tol}, {}, {}],",
            row.join(", "),
            class as u8,
            u8::from(near)
        );
    }
    out.push_str("]\n");
    out
}

fn parse_class_file(text: &str) -> Vec<(ClassCase, u8, bool)> {
    /// `[nx, ny, nz, dx, dy, dz, alpha, height, tol, class, near]`.
    type Row = (f64, f64, f64, f64, f64, f64, f64, f64, f64, u8, u8);
    #[derive(serde::Deserialize)]
    #[serde(deny_unknown_fields)]
    struct File {
        count: usize,
        cases: Vec<Row>,
    }
    let f: File = toml::from_str(text).unwrap();
    assert_eq!(f.count, f.cases.len());
    f.cases
        .into_iter()
        .map(|r| {
            (
                ClassCase {
                    n: DVec3::new(r.0, r.1, r.2),
                    d: DVec3::new(r.3, r.4, r.5),
                    alpha: r.6,
                    height: r.7,
                    tol: r.8,
                },
                r.9,
                r.10 == 1,
            )
        })
        .collect()
}

/// A one-face mesh realising `c` through `classify_faces`: the face (normal `n`, tiny) has its
/// highest corner `height` above a degenerate anchor at the lowest point, and the transform
/// rotates `d` onto plate +Z.
fn class_via_mesh(c: &ClassCase) -> FaceClass {
    let d = c.d.normalize();
    let n = c.n.normalize_or_zero();
    let l = 1e-5;
    let corners = if n == DVec3::ZERO {
        [DVec3::ZERO; 3]
    } else {
        let (u, v) = n.any_orthonormal_pair();
        let (u, v) = if u.cross(v).dot(n) > 0.0 {
            (u, v)
        } else {
            (v, u)
        };
        [DVec3::ZERO, u * l, v * l]
    };
    let top = corners
        .iter()
        .map(|p| p.dot(d))
        .fold(f64::NEG_INFINITY, f64::max);
    let shift = d * (c.height - top);
    let mut pos: Vec<DVec3> = corners.iter().map(|p| *p + shift).collect();
    pos.extend([DVec3::ZERO; 3]);
    let m = IndexedMesh::new(pos, vec![[0, 1, 2], [3, 4, 5]]).unwrap();
    let t = Transform::from_quat(DQuat::from_rotation_arc(d, DVec3::Z));
    let o = OverhangOpts {
        critical_angle: c.alpha,
        contact_tolerance: c.tol,
        support: None,
    };
    classify_faces(&m, &t, &o, &JobCtx::detached()).unwrap().0[0]
}

/// The WGSL rule in f32 (no guards), as RND-08 evaluates it.
fn class_f32(c: &ClassCase) -> u8 {
    let n = c.n.as_vec3().normalize_or_zero();
    if n == amg_core::glam::Vec3::ZERO {
        return 3;
    }
    let d = c.d.as_vec3().normalize();
    let cc = -n.dot(d);
    if cc > 0.0 && (c.height as f32) <= c.tol as f32 {
        2
    } else if cc > (c.alpha as f32).cos() {
        1
    } else {
        0
    }
}

#[test]
fn golden_overhang_classes() {
    let path = golden_path("overhang_classes.toml");
    let text = generate_class_file();
    let bless = std::env::var("AMG_BLESS").is_ok_and(|v| v == "1");
    if bless {
        assert!(
            std::env::var_os("CI").is_none(),
            "blessing is refused on CI"
        );
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, &text).unwrap();
    }
    let on_disk = std::fs::read_to_string(&path)
        .unwrap()
        .replace("\r\n", "\n");
    assert!(
        on_disk == text,
        "{} is stale; regenerate with AMG_BLESS=1",
        path.display()
    );
    let cases = parse_class_file(&on_disk);
    assert_eq!(cases.len(), CLASS_CASES);
    let mut counts = [0usize; 4];
    let (mut near, mut mesh_off, mut f32_off) = (0, 0, 0);
    for (i, (c, class, is_near)) in cases.iter().enumerate() {
        let (want, want_near) = c.expected();
        assert_eq!(want as u8, *class, "row {i}");
        assert_eq!(want_near, *is_near, "row {i}");
        counts[*class as usize] += 1;
        near += usize::from(*is_near);
        let via_mesh = class_via_mesh(c);
        let via_f32 = class_f32(c);
        if *is_near {
            mesh_off += usize::from(via_mesh as u8 != *class);
            f32_off += usize::from(via_f32 != *class);
        } else {
            assert_eq!(via_mesh as u8, *class, "row {i}: classify_faces");
            assert_eq!(via_f32, *class, "row {i}: f32 shader rule");
        }
    }
    eprintln!(
        "classes: {counts:?} (S/O/C/D), near-threshold {near}, of which mesh path differs on {mesh_off}, f32 rule on {f32_off}"
    );
    assert!(counts.iter().all(|&n| n > 50), "{counts:?}");
    assert!(near > 100);
}
