//! MESH-22 tests: curvature-adaptive remeshing.

#![allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]

mod common;

use amg_core::Point3;
use amg_jobs::JobCtx;
use amg_mesh::{EdgeLength, IndexedMesh, MeshError, RemeshOpts, Sizing, remesh};
use amg_testkit::{TriSoup, hausdorff, metrics, shapes};

fn soup(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
}

fn icosphere(sub: u32) -> IndexedMesh {
    let s = shapes::icosphere(Point3::ZERO, 10.0, sub);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

/// Sphere with a smooth, localised bump: gentle elsewhere, strongly curved on the bump.
fn displaced_sphere(sub: u32) -> IndexedMesh {
    let mut m = icosphere(sub);
    for p in &mut m.positions {
        let u = *p / 10.0;
        let d = (u - Point3::new(1.0, 0.0, 0.0)).length_squared();
        *p *= 1.0 + 0.25 * (-d / 0.08).exp() + 0.03 * (3.0 * u.y).sin();
    }
    m
}

fn run(m: &IndexedMesh, o: &RemeshOpts) -> IndexedMesh {
    let out = remesh(m, o, &JobCtx::detached()).unwrap();
    out.validate().unwrap();
    out
}

fn adaptive(eps: f64, min: f64, max: f64) -> RemeshOpts {
    RemeshOpts {
        sizing: Sizing::Adaptive {
            tolerance: eps,
            min_length: min,
            max_length: max,
        },
        ..RemeshOpts::default()
    }
}

fn uniform(l: f64) -> RemeshOpts {
    RemeshOpts {
        sizing: Sizing::Uniform(EdgeLength::Absolute(l)),
        ..RemeshOpts::default()
    }
}

fn deviation(a: &IndexedMesh, b: &IndexedMesh) -> f64 {
    hausdorff(soup(a), soup(b), 200_000)
}

/// Triangle count of the uniform remesh with the largest length that deviates at most `dev`.
fn uniform_at_deviation(m: &IndexedMesh, dev: f64, hi: f64) -> (f64, usize) {
    let (mut lo, mut hi) = (0.01 * hi, hi);
    let mut best = None;
    for _ in 0..12 {
        let mid = 0.5 * (lo + hi);
        let out = run(m, &uniform(mid));
        if deviation(m, &out) <= dev {
            best = Some((mid, out.tris.len()));
            lo = mid;
        } else {
            hi = mid;
        }
    }
    best.unwrap()
}

fn check(name: &str, m: &IndexedMesh, eps: f64, min: f64, max: f64) {
    let out = run(m, &adaptive(eps, min, max));
    let dev = deviation(m, &out);
    let (mi, mo) = (metrics(soup(m)), metrics(soup(&out)));
    let (l, uni) = uniform_at_deviation(m, dev, max);
    eprintln!(
        "{name}: eps {eps}, deviation {dev:.4}, adaptive {} tris, uniform (L {l:.3}) {uni} tris, ratio {:.3}",
        out.tris.len(),
        out.tris.len() as f64 / uni as f64
    );
    assert!(dev <= eps, "{name}: deviation {dev} > eps {eps}");
    assert!(
        out.tris.len() as f64 <= 0.6 * uni as f64,
        "{name}: {} tris > 60 % of uniform {uni}",
        out.tris.len()
    );
    assert_eq!(
        (mo.boundary_edges, mo.non_manifold_edges, mo.euler),
        (mi.boundary_edges, mi.non_manifold_edges, mi.euler),
        "{name}: topology changed"
    );
}

#[test]
fn sphere_deviation_within_tolerance() {
    // Constant curvature: adaptive cannot beat uniform here, so only the tolerance is checked.
    let m = icosphere(5);
    let out = run(&m, &adaptive(0.05, 0.05, 5.0));
    let dev = deviation(&m, &out);
    eprintln!("sphere: deviation {dev:.4}, {} tris", out.tris.len());
    assert!(dev <= 0.05, "deviation {dev}");
    assert!(out.tris.len() < m.tris.len() / 2);
}

#[test]
fn torus_adaptive_beats_uniform() {
    check("torus", &common::torus(240, 100, 4.0, 3.5), 0.05, 0.05, 6.0);
}

#[test]
fn displaced_sphere_adaptive_beats_uniform() {
    check("displaced", &displaced_sphere(6), 0.05, 0.05, 6.0);
}

#[test]
fn bit_identical_across_thread_counts() {
    let m = displaced_sphere(5);
    let hash_with = |threads: usize| {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap()
            .install(|| run(&m, &adaptive(0.05, 0.05, 6.0)).content_hash())
    };
    let h1 = hash_with(1);
    assert_eq!(h1, hash_with(4));
    assert_eq!(h1, hash_with(13));
}

#[test]
fn bounds_are_respected_and_options_validated() {
    let m = icosphere(4);
    // A huge tolerance gives max_length everywhere, a tiny one min_length.
    let coarse = run(&m, &adaptive(100.0, 0.5, 4.0));
    let fine = run(&m, &adaptive(1e-6, 2.0, 4.0));
    assert!(coarse.tris.len() < fine.tris.len());
    let ctx = JobCtx::detached();
    for (eps, lo, hi) in [
        (0.0, 0.1, 1.0),
        (f64::NAN, 0.1, 1.0),
        (0.1, 0.0, 1.0),
        (0.1, 2.0, 1.0),
        (0.1, 0.1, f64::INFINITY),
    ] {
        assert!(
            matches!(
                remesh(&m, &adaptive(eps, lo, hi), &ctx),
                Err(MeshError::InvalidOption { .. })
            ),
            "{eps} {lo} {hi}"
        );
    }
}

#[test]
fn round_torus_deviation_within_tolerance() {
    let m = common::torus(160, 60, 30.0, 8.0);
    let out = run(&m, &adaptive(0.1, 0.05, 10.0));
    let dev = deviation(&m, &out);
    eprintln!(
        "round torus: deviation {dev:.4}, {} -> {} tris",
        m.tris.len(),
        out.tris.len()
    );
    assert!(dev <= 0.1, "deviation {dev}");
}

#[test]
fn flat_faces_and_cube_features_survive() {
    // A cube is flat between its creases: the field is max_length there, and the (constrained)
    // feature edges and corners must stay exact.
    let s = shapes::cube(Point3::ZERO, 10.0);
    let m = IndexedMesh::new(s.positions, s.triangles).unwrap();
    let out = run(&m, &adaptive(0.01, 0.1, 3.0));
    assert!(deviation(&m, &out) <= 1e-6);
    let v = metrics(soup(&out));
    assert!((v.volume - 1000.0).abs() < 1e-6, "{}", v.volume);
    assert!(out.tris.len() > m.tris.len());
}
