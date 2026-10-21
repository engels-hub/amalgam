//! MESH-17: isosurface extraction (`amg_mesh::iso`).

#![allow(clippy::expect_used, clippy::unwrap_used)]

#[path = "common/mod.rs"]
mod common;

use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};

use amg_core::{Aabb3, Point3, Vec3};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::iso::{IsoOpts, ScalarField, extract_isosurface};
use amg_mesh::{IndexedMesh, MeshError};
use amg_testkit::{TriSoup, hausdorff, metrics, shapes};

fn ctx() -> JobCtx {
    JobCtx::detached()
}

// ── fields ─────────────────────────────────────────────────────────────────────────────

struct Sphere {
    c: Point3,
    r: f64,
}
impl ScalarField for Sphere {
    fn value(&self, p: Point3) -> f64 {
        (p - self.c).length() - self.r
    }
    fn may_cross(&self, b: &Aabb3, iso: f64) -> bool {
        // 1-Lipschitz: |f(centre)| <= half diagonal.
        self.value(b.center()).abs() - iso.abs() <= b.size().length() * 0.5
    }
}

struct Torus {
    major: f64,
    minor: f64,
}
impl ScalarField for Torus {
    fn value(&self, p: Point3) -> f64 {
        let q = ((p.x * p.x + p.y * p.y).sqrt() - self.major).hypot(p.z);
        q - self.minor
    }
}

#[derive(Clone)]
enum Prim {
    Sphere(Point3, f64),
    Box(Point3, Vec3),
}
impl Prim {
    fn value(&self, p: Point3) -> f64 {
        match *self {
            Prim::Sphere(c, r) => (p - c).length() - r,
            // Chebyshev form: exactly zero on the box faces for grid-aligned boxes.
            Prim::Box(c, h) => {
                let d = (p - c).abs() - h;
                d.x.max(d.y).max(d.z)
            }
        }
    }
}

#[derive(Clone)]
enum Op {
    Union,
    Intersect,
    Subtract,
}

struct Csg {
    prims: Vec<(Op, Prim)>,
}
impl ScalarField for Csg {
    fn value(&self, p: Point3) -> f64 {
        let mut it = self.prims.iter();
        let mut v = it.next().expect("non-empty").1.value(p);
        for (op, prim) in it {
            let w = prim.value(p);
            v = match op {
                Op::Union => v.min(w),
                Op::Intersect => v.max(w),
                Op::Subtract => v.max(-w),
            };
        }
        v
    }
}

/// SplitMix64.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.unit()
    }
    fn int(&mut self, a: i64, b: i64) -> i64 {
        a + (self.next() % (b - a + 1) as u64) as i64
    }
}

// ── topology checks ────────────────────────────────────────────────────────────────────

/// Closed, consistently oriented 2-manifold: every directed edge appears once and has its
/// reverse; every vertex link is a single cycle; no degenerate index triples.
fn check_manifold(m: &IndexedMesh) -> Result<(), String> {
    let mut directed: HashMap<(u32, u32), u32> = HashMap::new();
    for t in &m.tris {
        if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
            return Err(format!("degenerate triangle {t:?}"));
        }
        for k in 0..3 {
            let e = (t[k], t[(k + 1) % 3]);
            if directed.insert(e, 1).is_some() {
                return Err(format!("directed edge {e:?} used twice"));
            }
        }
    }
    for &(a, b) in directed.keys() {
        if !directed.contains_key(&(b, a)) {
            return Err(format!("edge {a}->{b} has no opposite"));
        }
    }
    // Vertex links: successor map of opposite edges must be a single cycle.
    let mut link: HashMap<u32, Vec<(u32, u32)>> = HashMap::new();
    for t in &m.tris {
        for k in 0..3 {
            link.entry(t[k])
                .or_default()
                .push((t[(k + 1) % 3], t[(k + 2) % 3]));
        }
    }
    for (v, edges) in &link {
        let next: HashMap<u32, u32> = edges.iter().copied().collect();
        if next.len() != edges.len() {
            return Err(format!("vertex {v}: link is not a function"));
        }
        let start = edges[0].0;
        let (mut cur, mut n) = (start, 0usize);
        loop {
            cur = *next
                .get(&cur)
                .ok_or_else(|| format!("vertex {v}: open link"))?;
            n += 1;
            if cur == start {
                break;
            }
            if n > edges.len() {
                return Err(format!("vertex {v}: link does not close"));
            }
        }
        if n != edges.len() {
            return Err(format!("vertex {v}: link has several cycles"));
        }
    }
    Ok(())
}

fn soup(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
}

fn cube_bounds(half: f64) -> Aabb3 {
    Aabb3::new(Point3::splat(-half), Point3::splat(half))
}

// ── tests ──────────────────────────────────────────────────────────────────────────────

#[test]
fn fuzz_random_csg_is_closed_oriented_manifold() {
    let mut rng = Rng(0xA11CE);
    let (mut empty, mut nonempty) = (0, 0);
    for case in 0..1000 {
        let grid_aligned = case % 2 == 0;
        let (lo, hi, h) = if grid_aligned {
            (-4.0, 4.0, 1.0)
        } else {
            let lo = rng.range(-5.0, -2.0);
            (lo, lo + rng.range(5.0, 9.0), rng.range(0.4, 0.9))
        };
        let n = rng.int(1, 5) as usize;
        let mut prims = Vec::new();
        for i in 0..n {
            let op = match rng.int(0, 3) {
                0 | 1 => Op::Union,
                2 => Op::Intersect,
                _ => Op::Subtract,
            };
            let op = if i == 0 { Op::Union } else { op };
            let prim = if grid_aligned {
                let c = Point3::new(
                    rng.int(-4, 4) as f64,
                    rng.int(-4, 4) as f64,
                    rng.int(-4, 4) as f64,
                );
                if rng.int(0, 1) == 0 {
                    Prim::Sphere(c, rng.int(0, 5) as f64)
                } else {
                    let e = Vec3::new(
                        rng.int(0, 5) as f64,
                        rng.int(0, 5) as f64,
                        rng.int(0, 5) as f64,
                    );
                    Prim::Box(c, e)
                }
            } else {
                let c = Point3::new(rng.range(lo, hi), rng.range(lo, hi), rng.range(lo, hi));
                if rng.int(0, 1) == 0 {
                    Prim::Sphere(c, rng.range(0.3, 5.0))
                } else {
                    Prim::Box(
                        c,
                        Vec3::new(
                            rng.range(0.3, 4.0),
                            rng.range(0.3, 4.0),
                            rng.range(0.3, 4.0),
                        ),
                    )
                }
            };
            prims.push((op, prim));
        }
        let f = Csg { prims };
        let mut opts = IsoOpts::new(Aabb3::new(Point3::splat(lo), Point3::splat(hi)), h);
        opts.iso = if case % 7 == 0 { 0.25 } else { 0.0 };
        opts.block_cells = [4, 8, 16][case % 3];
        opts.project_steps = (case % 4) as u32;
        let m = extract_isosurface(&f, &opts, &ctx()).expect("extract");
        if let Err(e) = check_manifold(&m) {
            panic!("case {case}: {e}");
        }
        assert!(
            m.signed_volume() >= -1e-9,
            "case {case}: negative volume {}",
            m.signed_volume()
        );
        let bb = Aabb3::new(Point3::splat(lo), Point3::splat(hi));
        for p in &m.positions {
            assert!(
                p.cmpge(bb.min).all() && p.cmple(bb.max).all(),
                "case {case}: vertex {p} outside bounds"
            );
        }
        if m.tris.is_empty() {
            empty += 1;
        } else {
            nonempty += 1;
        }
    }
    assert!(
        nonempty > 600 && empty > 0,
        "fuzz mix: {nonempty} / {empty}"
    );
}

#[test]
fn constant_fields() {
    struct Const(f64);
    impl ScalarField for Const {
        fn value(&self, _: Point3) -> f64 {
            self.0
        }
    }
    let b = cube_bounds(2.0);
    // value == iso is outside; positive is outside.
    for v in [0.0, 1.0, f64::NAN, f64::INFINITY] {
        let m = extract_isosurface(&Const(v), &IsoOpts::new(b, 1.0), &ctx()).unwrap();
        assert_eq!(m.face_count(), 0, "value {v}");
    }
    // Everywhere inside: the bounds box closes the surface exactly.
    let m = extract_isosurface(&Const(-1.0), &IsoOpts::new(b, 1.0), &ctx()).unwrap();
    check_manifold(&m).unwrap();
    assert!(
        (m.signed_volume() - 64.0).abs() < 1e-9,
        "{}",
        m.signed_volume()
    );
    assert_eq!(m.aabb().min, b.min);
    assert_eq!(m.aabb().max, b.max);
    // Bounds that are not a multiple of the cell size.
    let m = extract_isosurface(&Const(-1.0), &IsoOpts::new(b, 1.3), &ctx()).unwrap();
    check_manifold(&m).unwrap();
    assert!(
        (m.signed_volume() - 64.0).abs() < 1e-6,
        "{}",
        m.signed_volume()
    );
}

#[test]
fn nan_and_inf_regions_stay_manifold() {
    struct Weird;
    impl ScalarField for Weird {
        fn value(&self, p: Point3) -> f64 {
            if p.x > 1.0 && p.y > 0.0 {
                f64::NAN
            } else if p.z > 1.5 {
                f64::NEG_INFINITY
            } else {
                p.length() - 2.5
            }
        }
    }
    let mut o = IsoOpts::new(cube_bounds(4.0), 0.5);
    o.project_steps = 3;
    let m = extract_isosurface(&Weird, &o, &ctx()).unwrap();
    check_manifold(&m).unwrap();
    assert!(m.positions.iter().all(|p| p.is_finite()));
}

#[test]
fn invalid_options() {
    let f = Sphere {
        c: Point3::ZERO,
        r: 1.0,
    };
    let ok = IsoOpts::new(cube_bounds(2.0), 0.5);
    for bad in [
        IsoOpts {
            cell_size: 0.0,
            ..ok.clone()
        },
        IsoOpts {
            cell_size: f64::NAN,
            ..ok.clone()
        },
        IsoOpts {
            block_cells: 0,
            ..ok.clone()
        },
        IsoOpts {
            bounds: Aabb3::EMPTY,
            ..ok.clone()
        },
        IsoOpts {
            iso: f64::NAN,
            ..ok.clone()
        },
    ] {
        assert!(matches!(
            extract_isosurface(&f, &bad, &ctx()),
            Err(MeshError::InvalidOption { .. })
        ));
    }
}

#[test]
fn cancellation() {
    let t = CancelToken::new();
    t.cancel();
    let f = Sphere {
        c: Point3::ZERO,
        r: 5.0,
    };
    let r = extract_isosurface(
        &f,
        &IsoOpts::new(cube_bounds(8.0), 0.5),
        &JobCtx::with_token(t),
    );
    assert!(matches!(r, Err(MeshError::Cancelled)));
}

#[test]
fn sphere_volume_converges_quadratically() {
    let (r, exact) = (10.0, 4.0 / 3.0 * std::f64::consts::PI * 1000.0);
    let f = Sphere {
        c: Point3::new(0.13, -0.27, 0.05),
        r,
    };
    let mut errs = Vec::new();
    for h in [2.0, 1.0, 0.5, 0.25] {
        let m = extract_isosurface(&f, &IsoOpts::new(cube_bounds(12.0), h), &ctx()).unwrap();
        check_manifold(&m).unwrap();
        errs.push(((m.signed_volume() - exact) / exact).abs());
    }
    eprintln!("sphere volume rel errors: {errs:?}");
    for w in errs.windows(2) {
        assert!(w[1] < w[0] / 2.5, "not converging: {errs:?}");
    }
    assert!(errs[3] < 2e-3, "{errs:?}");
}

#[test]
fn torus_volume_converges() {
    let (major, minor) = (20.0, 6.0);
    let exact = 2.0 * std::f64::consts::PI.powi(2) * major * minor * minor;
    let f = Torus { major, minor };
    let b = Aabb3::new(
        Point3::new(-27.0, -27.0, -8.0),
        Point3::new(27.0, 27.0, 8.0),
    );
    let mut errs = Vec::new();
    for h in [1.5, 0.75, 0.375] {
        let m = extract_isosurface(&f, &IsoOpts::new(b, h), &ctx()).unwrap();
        check_manifold(&m).unwrap();
        errs.push(((m.signed_volume() - exact) / exact).abs());
        let mt = metrics(soup(&m));
        assert_eq!(mt.euler, 0, "torus genus 1");
    }
    eprintln!("torus volume rel errors: {errs:?}");
    assert!(
        errs[1] < errs[0] / 2.5 && errs[2] < errs[1] / 2.5,
        "{errs:?}"
    );
    assert!(errs[2] < 3e-3, "{errs:?}");
}

#[test]
fn hausdorff_to_analytic_shapes_within_cell() {
    let h = 0.5;
    let f = Sphere {
        c: Point3::ZERO,
        r: 10.0,
    };
    let m = extract_isosurface(&f, &IsoOpts::new(cube_bounds(12.0), h), &ctx()).unwrap();
    let reference = shapes::icosphere(Point3::ZERO, 10.0, 5);
    let d = hausdorff(soup(&m), reference.soup(), 50_000);
    assert!(d <= h, "sphere Hausdorff {d} > {h}");

    let (major, minor) = (20.0, 6.0);
    let b = Aabb3::new(
        Point3::new(-27.0, -27.0, -8.0),
        Point3::new(27.0, 27.0, 8.0),
    );
    let m = extract_isosurface(&Torus { major, minor }, &IsoOpts::new(b, h), &ctx()).unwrap();
    let reference = common::torus(256, 128, major, minor);
    let reference = soup_of(&reference);
    let d = hausdorff(soup(&m), reference, 50_000);
    assert!(d <= h, "torus Hausdorff {d} > {h}");
}

fn soup_of(m: &IndexedMesh) -> TriSoup<'_> {
    soup(m)
}

#[test]
fn project_steps_reduce_surface_error() {
    let f = Sphere {
        c: Point3::ZERO,
        r: 10.0,
    };
    let err = |steps: u32| {
        let mut o = IsoOpts::new(cube_bounds(12.0), 1.0);
        o.project_steps = steps;
        let m = extract_isosurface(&f, &o, &ctx()).unwrap();
        check_manifold(&m).unwrap();
        m.positions
            .iter()
            .map(|p| (p.length() - 10.0).abs())
            .fold(0.0, f64::max)
    };
    let (e0, e2) = (err(0), err(2));
    assert!(e2 < e0 * 0.2 && e2 < 5e-3, "{e0} -> {e2}");
}

#[test]
fn deterministic_across_thread_counts() {
    let f = Torus {
        major: 20.0,
        minor: 6.0,
    };
    let b = Aabb3::new(
        Point3::new(-27.0, -27.0, -8.0),
        Point3::new(27.0, 27.0, 8.0),
    );
    let mut o = IsoOpts::new(b, 0.7);
    o.project_steps = 2;
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| extract_isosurface(&f, &o, &ctx()).unwrap())
    };
    let (a, b, c) = (run(1), run(3), run(8));
    assert!(a.face_count() > 1000);
    assert_eq!(a.content_hash(), b.content_hash());
    assert_eq!(a.content_hash(), c.content_hash());
    assert_eq!(a.positions, c.positions);
    assert_eq!(a.tris, c.tris);
}

#[test]
fn sparse_skips_far_blocks() {
    struct Counting(Sphere, AtomicUsize);
    impl ScalarField for Counting {
        fn value(&self, p: Point3) -> f64 {
            self.1.fetch_add(1, Ordering::Relaxed);
            self.0.value(p)
        }
        fn may_cross(&self, b: &Aabb3, iso: f64) -> bool {
            self.0.may_cross(b, iso)
        }
    }
    let f = Counting(
        Sphere {
            c: Point3::ZERO,
            r: 10.0,
        },
        AtomicUsize::new(0),
    );
    let h = 0.5;
    let m = extract_isosurface(&f, &IsoOpts::new(cube_bounds(25.0), h), &ctx()).unwrap();
    check_manifold(&m).unwrap();
    let dense = (50.0f64 / h).powi(3);
    let used = f.1.load(Ordering::Relaxed) as f64;
    assert!(used < dense * 0.25, "evaluated {used} of {dense}");
}

#[test]
fn golden_iso_sphere_r10() {
    let f = Sphere {
        c: Point3::ZERO,
        r: 10.0,
    };
    let m = extract_isosurface(&f, &IsoOpts::new(cube_bounds(12.0), 1.0), &ctx()).unwrap();
    amg_testkit::assert_golden!("mesh", "iso_sphere_r10_h1", soup(&m));
}

#[test]
fn golden_iso_torus() {
    let f = Torus {
        major: 20.0,
        minor: 6.0,
    };
    let b = Aabb3::new(
        Point3::new(-27.0, -27.0, -8.0),
        Point3::new(27.0, 27.0, 8.0),
    );
    let m = extract_isosurface(&f, &IsoOpts::new(b, 1.0), &ctx()).unwrap();
    amg_testkit::assert_golden!("mesh", "iso_torus_R20_r6_h1", soup(&m));
}

#[test]
fn golden_iso_csg_box_minus_sphere() {
    // Grid-aligned box (exact zeros on lattice points) with a spherical bite.
    let f = Csg {
        prims: vec![
            (Op::Union, Prim::Box(Point3::ZERO, Vec3::splat(8.0))),
            (Op::Subtract, Prim::Sphere(Point3::new(8.0, 0.0, 0.0), 6.0)),
        ],
    };
    let m = extract_isosurface(&f, &IsoOpts::new(cube_bounds(10.0), 1.0), &ctx()).unwrap();
    check_manifold(&m).unwrap();
    amg_testkit::assert_golden!("mesh", "iso_box_minus_sphere_h1", soup(&m));
}
