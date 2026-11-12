//! INS-01: `amg_features::inspect::measure` against brute force, across thread counts, and on
//! a CAD-exported corpus STL.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::print_stderr,
    missing_docs,
    reason = "tests"
)]

use std::f64::consts::TAU;
use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_core::{Aabb3, Point3, Quat, Transform, Vec3};
use amg_features::inspect::InspectError;
use amg_features::inspect::measure::prims::{PrimFeature, tri_tri};
use amg_features::inspect::measure::{
    MeasureOpts, MeshElement, MeshOperand, MinDistance, Operand, RegionOpts, WitnessFeature,
    grow_planar_region, min_distance, prepare_body,
};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::{FaceId, IndexedMesh, MeshBody};
use amg_testkit::shapes;

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// SplitMix64.
struct Rng(u64);
impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
    fn dir(&mut self) -> Vec3 {
        loop {
            let v = Vec3::new(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            let l = v.length();
            if l > 0.1 && l <= 1.0 {
                return v / l;
            }
        }
    }
    fn rotation(&mut self) -> Quat {
        let axis = self.dir();
        Quat::from_axis_angle(axis, self.range(0.0, TAU))
    }
}

/// A bumpy closed blob: icosphere (subdivision 3, 1 280 faces) with radial noise and an
/// anisotropic stretch, so faces are not all alike.
fn blob(rng: &mut Rng, radius: f64) -> MeshBody {
    let s = shapes::icosphere(Point3::ZERO, radius, 3);
    let stretch = Vec3::new(
        rng.range(0.6, 1.4),
        rng.range(0.6, 1.4),
        rng.range(0.6, 1.4),
    );
    let positions = s
        .positions
        .iter()
        .map(|p| *p * stretch * (1.0 + 0.15 * (rng.unit() - 0.5)))
        .collect();
    MeshBody::new(IndexedMesh::new(positions, s.triangles).unwrap())
}

fn body_op(b: &MeshBody, t: Transform) -> Operand {
    Operand::Mesh(MeshOperand {
        body: b.clone(),
        to_world: t,
        element: MeshElement::Body,
    })
}

/// Same mapping as the library (note: computed in A's frame, B mapped by `A⁻¹ · B`).
fn feature(m: &IndexedMesh, f: u32, pf: PrimFeature) -> WitnessFeature {
    let v = m.tris[f as usize];
    match pf {
        PrimFeature::Corner(k) => WitnessFeature::Vertex(v[usize::from(k)]),
        PrimFeature::Edge(k, l) => {
            let (a, b) = (v[usize::from(k)], v[usize::from(l)]);
            WitnessFeature::Edge(a.min(b), a.max(b))
        }
        PrimFeature::Interior => WitnessFeature::Face(FaceId(f)),
    }
}

/// O(n·m) oracle: every triangle pair, keeping the lexicographic minimum of
/// `(distance², a face, b face)`. A pair is skipped only when the gap between its two
/// triangle boxes exceeds the best distance so far (a box gap never exceeds the triangle
/// distance), so ties are all evaluated.
fn brute_force(
    a: &MeshBody,
    ta: &Transform,
    b: &MeshBody,
    tb: &Transform,
) -> (f64, WitnessFeature, WitnessFeature) {
    let t_ab = ta.inverse() * *tb;
    let (ma, mb) = (a.mesh(), b.mesh());
    let tris_b: Vec<([Point3; 3], Aabb3)> = (0..mb.face_count())
        .map(|f| {
            let c = mb
                .triangle(FaceId(f as u32))
                .map(|p| t_ab.transform_point3(p));
            (c, Aabb3::from_points(c))
        })
        .collect();
    let mut best = (
        f64::INFINITY,
        0u32,
        0u32,
        PrimFeature::Interior,
        PrimFeature::Interior,
    );
    for fa in 0..ma.face_count() as u32 {
        let ca = ma.triangle(FaceId(fa));
        let ba = Aabb3::from_points(ca);
        for (fb, (cb, bb)) in tris_b.iter().enumerate() {
            let gap = (ba.min - bb.max)
                .max(bb.min - ba.max)
                .max(Vec3::ZERO)
                .length();
            let gap = (gap - 1e-9).max(0.0);
            if gap * gap > best.0 {
                continue;
            }
            let d = tri_tri(&ca, cb);
            if d.dist_sq < best.0 {
                best = (d.dist_sq, fa, fb as u32, d.fa, d.fb);
            }
        }
    }
    (
        best.0.sqrt(),
        feature(ma, best.1, best.3),
        feature(mb, best.2, best.4),
    )
}

#[test]
fn min_distance_equals_brute_force() {
    let mut rng = Rng(20_261_111);
    let opts = MeasureOpts::default();
    let pairs = 1000;
    let mut kinds = [0usize; 4];
    let mut worst = 0.0f64;
    let start = Instant::now();
    for i in 0..pairs {
        let ra = rng.range(5.0, 15.0);
        let a = blob(&mut rng, ra);
        let kind = i % 4;
        let rb = match kind {
            1 => rng.range(0.25, 0.3) * ra,
            3 => rng.range(0.5, 1.5),
            _ => rng.range(3.0, 12.0),
        };
        let b = blob(&mut rng, rb);
        let ta = Transform::from_rotation_translation(
            rng.rotation(),
            Vec3::new(
                rng.range(-50.0, 50.0),
                rng.range(-50.0, 50.0),
                rng.range(-50.0, 50.0),
            ),
        );
        let dir = rng.dir();
        let centre = match kind {
            // Separated: well apart.
            0 => dir * (2.0 * (ra + rb) + rng.range(0.1, 20.0)),
            // Intersecting: B (smaller than A's thinnest half-width) centred on a vertex of A.
            1 => {
                let m = a.mesh();
                m.positions[(rng.next_u64() % m.vertex_count() as u64) as usize]
            }
            // Touching: placed apart, then moved onto A along the witness direction (below).
            2 => dir * (2.0 * (ra + rb) + 5.0),
            // Nested: a small blob near A's centre.
            _ => dir * rng.range(0.0, 0.2) * ra,
        };
        let mut tb =
            Transform::from_rotation_translation(rng.rotation(), ta.transform_point3(centre));
        if kind == 2 {
            let d = min_distance(&body_op(&a, ta), &body_op(&b, tb), &opts, &ctx()).unwrap();
            tb = Transform::from_translation(-d.delta()) * tb;
        }
        let got = min_distance(&body_op(&a, ta), &body_op(&b, tb), &opts, &ctx()).unwrap();
        let (d, fa, fb) = brute_force(&a, &ta, &b, &tb);
        let err = (got.distance.to_mm() - d).abs();
        worst = worst.max(err);
        assert!(
            err <= 1e-12,
            "pair {i} (kind {kind}): {} vs {d}",
            got.distance.to_mm()
        );
        assert_eq!(
            (got.a.feature, got.b.feature),
            (fa, fb),
            "pair {i} (kind {kind})"
        );
        assert!(((got.b.point - got.a.point).length() - d).abs() <= 1e-9);
        match kind {
            1 => assert!(got.intersecting, "pair {i}"),
            2 => assert!(
                got.distance.to_mm() <= 1e-9,
                "pair {i}: {}",
                got.distance.to_mm()
            ),
            3 => assert_eq!(got.contained, Some(true), "pair {i}"),
            _ => assert_eq!(got.contained, Some(false), "pair {i}"),
        }
        kinds[kind] += 1;
    }
    eprintln!(
        "brute force: {pairs} pairs ({kinds:?} separated/intersecting/touching/nested), worst |Δ| = {worst:e} mm, {:?}",
        start.elapsed()
    );
}

#[test]
fn min_distance_is_symmetric_and_rigid_invariant() {
    let mut rng = Rng(99);
    let opts = MeasureOpts::default();
    for _ in 0..20 {
        let a = blob(&mut rng, 8.0);
        let b = blob(&mut rng, 5.0);
        let ta = Transform::from_rotation_translation(rng.rotation(), rng.dir() * 10.0);
        let tb = Transform::from_rotation_translation(rng.rotation(), rng.dir() * 25.0);
        let d1 = min_distance(&body_op(&a, ta), &body_op(&b, tb), &opts, &ctx()).unwrap();
        let d2 = min_distance(&body_op(&b, tb), &body_op(&a, ta), &opts, &ctx()).unwrap();
        assert!((d1.distance.to_mm() - d2.distance.to_mm()).abs() <= 1e-12);
        let g = Transform::from_rotation_translation(rng.rotation(), rng.dir() * 100.0);
        let d3 = min_distance(&body_op(&a, g * ta), &body_op(&b, g * tb), &opts, &ctx()).unwrap();
        assert!((d1.distance.to_mm() - d3.distance.to_mm()).abs() <= 1e-11);
    }
}

fn icosphere_body(r: f64, sub: u32) -> MeshBody {
    let s = shapes::icosphere(Point3::ZERO, r, sub);
    MeshBody::new(IndexedMesh::new(s.positions, s.triangles).unwrap())
}

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .unwrap()
        .install(f)
}

fn bits(d: &MinDistance) -> [u64; 7] {
    [
        d.distance.to_mm().to_bits(),
        d.a.point.x.to_bits(),
        d.a.point.y.to_bits(),
        d.a.point.z.to_bits(),
        d.b.point.x.to_bits(),
        d.b.point.y.to_bits(),
        d.b.point.z.to_bits(),
    ]
}

#[test]
fn bit_identical_across_thread_counts() {
    // 327 680 faces each: above the parallel threshold, so the 4 × 4 split runs.
    let a = icosphere_body(50.0, 7);
    let b = icosphere_body(49.9, 7);
    let c = icosphere_body(30.0, 7);
    let opts = MeasureOpts::default();
    let cases = [
        // Concentric (nested, worst case).
        (
            Transform::IDENTITY,
            Transform::from_quat(Quat::from_rotation_z(0.3)),
        ),
        // Separated by ~5 mm.
        (
            Transform::IDENTITY,
            Transform::from_translation(Vec3::new(85.0, 1.0, -2.0)),
        ),
        // Intersecting.
        (
            Transform::from_quat(Quat::from_rotation_x(0.2)),
            Transform::from_translation(Vec3::new(60.0, 0.0, 0.0)),
        ),
    ];
    for (i, (ta, tb)) in cases.into_iter().enumerate() {
        // Concentric uses the 0.1 mm gap of the S05 worst case; the others the r = 30 sphere.
        let b = if i == 0 { &b } else { &c };
        let run = || {
            let d = min_distance(&body_op(&a, ta), &body_op(b, tb), &opts, &ctx()).unwrap();
            (bits(&d), d.a.feature, d.b.feature, d.contained)
        };
        let t0 = Instant::now();
        let one = in_pool(1, run);
        eprintln!(
            "1 thread: {:?} (distance bits {:x})",
            t0.elapsed(),
            one.0[0]
        );
        let one_again = in_pool(1, run);
        let many = in_pool(16, run);
        let many_again = in_pool(16, run);
        if i == 0 {
            assert_eq!(
                one.3,
                Some(true),
                "the r = 49.9 sphere lies inside the r = 50 one"
            );
        }
        assert_eq!(one, one_again);
        assert_eq!(one, many);
        assert_eq!(one, many_again);
    }
}

#[test]
fn concentric_spheres_cancel_quickly() {
    let a = icosphere_body(50.0, 6);
    let b = icosphere_body(49.9, 6);
    prepare_body(&a, &ctx()).unwrap();
    prepare_body(&b, &ctx()).unwrap();
    let token = CancelToken::new();
    let cx = JobCtx::with_token(token.clone());
    let opts = MeasureOpts::default();
    let (ta, tb) = (
        Transform::IDENTITY,
        Transform::from_quat(Quat::from_rotation_z(0.01)),
    );
    let handle = std::thread::spawn(move || {
        let t0 = Instant::now();
        let r = min_distance(&body_op(&a, ta), &body_op(&b, tb), &opts, &cx);
        (r, t0.elapsed())
    });
    std::thread::sleep(Duration::from_millis(20));
    let cancel_at = Instant::now();
    token.cancel();
    let (r, _) = handle.join().unwrap();
    let latency = cancel_at.elapsed();
    match r {
        Err(InspectError::Cancelled) => eprintln!("cancel latency {latency:?}"),
        Ok(d) => eprintln!("finished before the cancel ({} mm)", d.distance.to_mm()),
        Err(e) => panic!("{e}"),
    }
    if strict_timing() {
        assert!(latency < Duration::from_millis(50), "{latency:?}");
    }
}

// ---- Done-when 4: planar regions on a CAD-exported STL ----------------------------------

/// Done-when 4: on a CAD-exported binary (f32) STL from the corpus, growing a planar region
/// from *any* triangle of each of its 10 largest planar CAD faces returns exactly that face's
/// ground-truth triangle set. The ground truth in `tests/golden/inspect/` was computed
/// independently of this crate (a script reading the STL directly: union-find over manifold
/// edges whose two triangles are coplanar within 0.5° / 1e-4 mm, each component checked
/// planar within 1e-3 mm).
#[test]
fn planar_regions_on_corpus_stl() {
    use amg_mesh::corpus::{FileStatus, Manifest, corpus_root, read_mesh};
    let gt_path = amg_testkit::golden_dir().join("inspect/planar_regions_thingi10k-232912.json");
    let gt: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(gt_path).unwrap()).unwrap();
    let manifest = Manifest::load().unwrap();
    let entry = manifest
        .meshes
        .iter()
        .find(|e| e.id == gt["model"])
        .unwrap();
    let path = match entry.locate(&corpus_root()) {
        FileStatus::Ready(p) => p,
        FileStatus::LfsPointer(p) | FileStatus::NotFetched(p) | FileStatus::Missing(p) => {
            assert!(
                !std::env::var("AMG_REQUIRE_TESTDATA").is_ok_and(|v| v == "1"),
                "{} is not available and AMG_REQUIRE_TESTDATA=1",
                p.display()
            );
            eprintln!("skipping: {} not fetched", p.display());
            return;
        }
    };
    let m = read_mesh(&path).unwrap();
    assert_eq!(m.face_count() as u64, gt["triangles"].as_u64().unwrap());
    let faces = gt["planar_faces"].as_array().unwrap();
    assert_eq!(faces.len(), 10);
    let body = MeshBody::new(m.clone());
    let mut seeds = 0;
    for f in faces {
        let want: Vec<FaceId> = f["faces"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| FaceId(u32::try_from(v.as_u64().unwrap()).unwrap()))
            .collect();
        for &seed in &want {
            let got = grow_planar_region(&m, seed, &RegionOpts::default(), &ctx()).unwrap();
            assert_eq!(&*got, &want[..], "seed {}", seed.0);
            seeds += 1;
        }
        let e = amg_features::inspect::measure::measure_one(
            &Operand::Mesh(MeshOperand {
                body: body.clone(),
                to_world: Transform::IDENTITY,
                element: MeshElement::Faces(want.clone().into()),
            }),
            &MeasureOpts::default(),
            &ctx(),
        )
        .unwrap();
        let area = f["area_mm2"].as_f64().unwrap();
        assert!((e.area.unwrap().get() - area).abs() <= 1e-6 * area);
        let plane = e.plane.expect("planar");
        let n = f["normal"].as_array().unwrap();
        let n = Vec3::new(
            n[0].as_f64().unwrap(),
            n[1].as_f64().unwrap(),
            n[2].as_f64().unwrap(),
        );
        assert!((plane.plane.normal.get() - n).length() <= 1e-5);
    }
    eprintln!("planar regions: 10 faces, {seeds} seeds, all exact");
}

/// Discovery helper (run with `--ignored`): planar segmentation of every corpus model.
#[test]
#[ignore = "corpus survey for picking the Done-when 4 model"]
fn survey_corpus_planar_regions() {
    use amg_mesh::corpus::{FileStatus, Manifest, corpus_root, read_mesh};
    let manifest = Manifest::load().unwrap();
    let root = corpus_root();
    for e in &manifest.meshes {
        let FileStatus::Ready(path) = e.locate(&root) else {
            continue;
        };
        let Ok(m) = read_mesh(&path) else { continue };
        if m.face_count() > 20_000 {
            continue;
        }
        let mut seen = vec![false; m.face_count()];
        let mut sizes = Vec::new();
        for f in 0..m.face_count() {
            if seen[f] {
                continue;
            }
            let Ok(r) = grow_planar_region(&m, FaceId(f as u32), &RegionOpts::default(), &ctx())
            else {
                seen[f] = true;
                sizes.push(0);
                continue;
            };
            for g in r.iter() {
                seen[g.0 as usize] = true;
            }
            sizes.push(r.len());
        }
        let mut big: Vec<usize> = sizes.iter().copied().filter(|&s| s >= 2).collect();
        big.sort_unstable_by(|a, b| b.cmp(a));
        let covered: usize = big.iter().sum();
        eprintln!(
            "SURVEY {} ({}): {} faces, {} regions, {} multi-face covering {:.3}, top {:?}",
            e.id,
            e.title,
            m.face_count(),
            sizes.len(),
            big.len(),
            covered as f64 / m.face_count() as f64,
            &big[..big.len().min(14)]
        );
    }
    let _ = Arc::new(());
}
