//! MESH-05 tests: booleans via Manifold (feature `manifold`).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::print_stderr
)]

use amg_core::{Point3, Vec3};
use amg_jobs::{CancelToken, JobCtx};
use amg_mesh::corpus::bench::{BenchSize, boolean_pair};
use amg_mesh::{
    AnalysisReport, AnalyzeOpts, BoolOp, BoolOpts, FaceGroupId, FaceGroupInfo, FaceId, GroupOrigin,
    IndexedMesh, MeshError, analyze, boolean, boolean_n,
};
use amg_testkit::{TriSoup, shapes};
use proptest::prelude::*;

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn report(m: &IndexedMesh) -> AnalysisReport {
    analyze(m, &AnalyzeOpts::default(), &ctx()).unwrap()
}

fn soup(m: &IndexedMesh) -> TriSoup<'_> {
    TriSoup {
        positions: &m.positions,
        triangles: &m.tris,
    }
}

fn icosphere(center: Point3, radius: f64, sub: u32) -> IndexedMesh {
    let s = shapes::icosphere(center, radius, sub);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

fn cube(min: Point3, size: f64) -> IndexedMesh {
    let s = shapes::cube(min, size);
    IndexedMesh::new(s.positions, s.triangles).unwrap()
}

/// Puts every face whose normal points along `+axis` into a new group named `name`.
fn group_side(m: &mut IndexedMesh, axis: Vec3, name: &str) -> FaceGroupId {
    let mut info = FaceGroupInfo::new(GroupOrigin::User);
    info.name = Some(name.to_owned());
    let g = m.groups.alloc(info);
    for f in 0..m.face_count() as u32 {
        if m.face_normal(FaceId(f)).dot(axis) > 0.9 {
            m.groups.set(FaceId(f), g);
        }
    }
    g
}

fn run(op: BoolOp, a: &IndexedMesh, b: &IndexedMesh) -> IndexedMesh {
    boolean(op, a, b, &BoolOpts::default(), &ctx()).unwrap().0
}

fn assert_watertight(m: &IndexedMesh) {
    let r = report(m);
    assert!(
        r.is_watertight(),
        "not watertight: {} boundary edges, {} non-manifold edges, {} orientation conflicts",
        r.boundary_edges,
        r.non_manifold_edges.len(),
        r.orientation_conflicts
    );
    assert!(m.signed_volume() >= 0.0);
}

#[test]
fn cube_union_difference_intersection() {
    let a = cube(Point3::ZERO, 2.0);
    let b = cube(Point3::splat(1.0), 2.0);
    let u = run(BoolOp::Union, &a, &b);
    let d = run(BoolOp::Difference, &a, &b);
    let i = run(BoolOp::Intersection, &a, &b);
    for (m, v) in [(&u, 15.0), (&d, 7.0), (&i, 1.0)] {
        assert_watertight(m);
        assert!(
            (m.signed_volume() - v).abs() < 1e-12,
            "{}",
            m.signed_volume()
        );
        assert!(m.attrs.is_empty());
        m.validate().unwrap();
    }
}

/// Done-when: group ids survive (A kept, B re-allocated).
#[test]
fn group_ids_survive() {
    let mut a = cube(Point3::ZERO, 2.0);
    let a_top = group_side(&mut a, Vec3::Z, "a top");
    let a_side = group_side(&mut a, Vec3::X, "a +x");
    let mut b = cube(Point3::new(1.0, 0.5, 0.5), 2.0);
    // Burn ids so B's ids collide with A's.
    let b_top = group_side(&mut b, Vec3::Z, "b top");
    let b_side = group_side(&mut b, Vec3::X, "b +x");
    assert_eq!((a_top, a_side), (b_top, b_side));
    let a_next = a.groups.next_id();

    let u = run(BoolOp::Union, &a, &b);
    let ids: Vec<FaceGroupId> = u.groups.ids().collect();
    // A's table is kept as is.
    for g in a.groups.ids() {
        assert_eq!(u.groups.info(g), a.groups.info(g));
    }
    // B's groups present in the result got fresh ids with a Boolean origin and kept the name.
    let fresh: Vec<FaceGroupId> = ids.iter().copied().filter(|g| g.0 >= a_next).collect();
    assert_eq!(
        fresh.len(),
        3,
        "B default, top and +x are all on the union surface"
    );
    let mut by_source = std::collections::BTreeMap::new();
    for g in &fresh {
        let info = u.groups.info(*g).unwrap();
        let GroupOrigin::Boolean { operand, source } = info.origin else {
            panic!("unexpected origin {:?}", info.origin);
        };
        assert_eq!(operand, 1);
        by_source.insert(source, (*g, info.name.clone()));
    }
    assert_eq!(by_source[&b_top].1.as_deref(), Some("b top"));
    assert_eq!(by_source[&b_side].1.as_deref(), Some("b +x"));
    // Every face sits in the group its geometry says: +x faces at x = 3 come from B's +x group,
    // top faces at z = 2 from A's top group, at z = 2.5 from B's top group.
    for f in 0..u.face_count() as u32 {
        let f = FaceId(f);
        let g = u.groups.group_of(f);
        let n = u.face_normal(f);
        let c = u.triangle(f).iter().fold(Vec3::ZERO, |s, p| s + *p) / 3.0;
        if n.dot(Vec3::X) > 0.9 && (c.x - 3.0).abs() < 1e-9 {
            assert_eq!(g, by_source[&b_side].0);
        } else if n.dot(Vec3::Z) > 0.9 && (c.z - 2.0).abs() < 1e-9 {
            assert_eq!(g, a_top);
        } else if n.dot(Vec3::Z) > 0.9 && (c.z - 2.5).abs() < 1e-9 {
            assert_eq!(g, by_source[&b_top].0);
        }
    }
    assert_watertight(&u);
    // A's +x face is partly covered by B: the uncovered L-shaped rest keeps A's group id.
    let rest: Vec<FaceId> = u.groups.faces_of(a_side).collect();
    assert!(!rest.is_empty());
    for f in rest {
        assert!(u.triangle(f).iter().all(|p| (p.x - 2.0).abs() < 1e-12));
        assert!(u.face_normal(f).dot(Vec3::X) > 0.99);
    }
    let area: f64 = u.groups.faces_of(a_side).map(|f| u.face_area(f)).sum();
    assert!((area - (4.0 - 1.5 * 1.5)).abs() < 1e-12, "{area}");
}

#[test]
fn boolean_n_batch() {
    let a = icosphere(Point3::ZERO, 10.0, 3);
    let b = icosphere(Point3::new(8.0, 0.0, 0.0), 10.0, 3);
    let c = icosphere(Point3::new(0.0, 8.0, 0.0), 10.0, 3);
    let opts = BoolOpts::default();
    let (u, _) = boolean_n(BoolOp::Union, &[&a, &b, &c], &opts, &ctx()).unwrap();
    assert_watertight(&u);
    let operands: std::collections::BTreeSet<u8> = u
        .groups
        .ids()
        .filter_map(|g| match u.groups.info(g)?.origin {
            GroupOrigin::Boolean { operand, .. } => Some(operand),
            _ => None,
        })
        .collect();
    assert_eq!(operands, [1, 2].into());
    // Batch result equals the pairwise chain up to rounding.
    let ab = run(BoolOp::Union, &a, &b);
    let abc = run(BoolOp::Union, &ab, &c);
    assert!((u.signed_volume() - abc.signed_volume()).abs() < 1e-9 * abc.signed_volume());
    // Difference: a minus the union of the others.
    let (d, _) = boolean_n(BoolOp::Difference, &[&a, &b, &c], &opts, &ctx()).unwrap();
    assert_watertight(&d);
    let expect = a.signed_volume()
        - run(BoolOp::Intersection, &a, &run(BoolOp::Union, &b, &c)).signed_volume();
    assert!((d.signed_volume() - expect).abs() < 1e-9 * expect);
    // One operand: a normalised copy.
    let (one, _) = boolean_n(BoolOp::Union, &[&a], &opts, &ctx()).unwrap();
    assert!((one.signed_volume() - a.signed_volume()).abs() < 1e-9);
}

#[test]
fn disjoint_and_empty_results() {
    let a = cube(Point3::ZERO, 1.0);
    let b = cube(Point3::splat(5.0), 1.0);
    let u = run(BoolOp::Union, &a, &b);
    assert_eq!(report(&u).shells.len(), 2);
    assert!((u.signed_volume() - 2.0).abs() < 1e-12);
    let i = run(BoolOp::Intersection, &a, &b);
    assert_eq!(i.face_count(), 0);
    i.validate().unwrap();
    let d = run(BoolOp::Difference, &a, &a);
    assert_eq!(d.face_count(), 0);
}

#[test]
fn open_input_is_rejected_or_repaired() {
    let a = icosphere(Point3::ZERO, 10.0, 2);
    let mut b = icosphere(Point3::new(5.0, 0.0, 0.0), 10.0, 2);
    b.tris.pop();
    b.groups = amg_mesh::FaceGroups::new(b.tris.len());
    let r = boolean(BoolOp::Union, &a, &b, &BoolOpts::default(), &ctx());
    assert!(
        matches!(r, Err(MeshError::NotClosed { boundary_loops: 1 })),
        "{r:?}"
    );
    let opts = BoolOpts {
        repair_inputs: true,
    };
    let (u, rep) = boolean(BoolOp::Union, &a, &b, &opts, &ctx()).unwrap();
    assert_eq!(rep.repaired_inputs, vec![1]);
    assert_watertight(&u);
}

#[test]
fn cancelled() {
    let a = cube(Point3::ZERO, 1.0);
    let t = CancelToken::new();
    t.cancel();
    let c = JobCtx::with_token(t);
    let r = boolean(BoolOp::Union, &a, &a, &BoolOpts::default(), &c);
    assert!(matches!(r, Err(MeshError::Cancelled)));
}

fn bits(m: &IndexedMesh) -> (Vec<[u64; 3]>, Vec<[u32; 3]>, Vec<FaceGroupId>) {
    (
        m.positions
            .iter()
            .map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
            .collect(),
        m.tris.clone(),
        m.groups.per_face().to_vec(),
    )
}

/// Done-when: 10 runs with TBB are bit-identical.
#[test]
fn determinism_10_runs() {
    let (mut a, mut b) = boolean_pair(BenchSize::K100, 3);
    group_side(&mut a, Vec3::Z, "a top");
    group_side(&mut b, Vec3::Y, "b side");
    for op in [BoolOp::Union, BoolOp::Difference, BoolOp::Intersection] {
        let first = bits(&run(op, &a, &b));
        for _ in 1..10 {
            assert!(
                bits(&run(op, &a, &b)) == first,
                "{op:?} output differs between runs"
            );
        }
    }
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 200, ..ProptestConfig::default() })]

    /// Done-when: vol(A∪B) + vol(A∩B) = vol(A) + vol(B) within 1e-6 relative, on 200 random
    /// pairs; every output is watertight.
    #[test]
    fn volume_identity(
        ra in 2.0f64..12.0, rb in 2.0f64..12.0,
        off in prop::array::uniform3(-15.0f64..15.0),
        sub_a in 1u32..4, sub_b in 1u32..4,
        a_box in any::<bool>(), b_box in any::<bool>(),
    ) {
        let shape = |is_box: bool, c: Point3, r: f64, sub: u32| {
            if is_box { cube(c - Vec3::splat(r * 0.8), r * 1.6) } else { icosphere(c, r, sub) }
        };
        let a = shape(a_box, Point3::ZERO, ra, sub_a);
        let b = shape(b_box, Point3::from(off), rb, sub_b);
        let u = run(BoolOp::Union, &a, &b);
        let i = run(BoolOp::Intersection, &a, &b);
        let d = run(BoolOp::Difference, &a, &b);
        let lhs = u.signed_volume() + i.signed_volume();
        let rhs = a.signed_volume() + b.signed_volume();
        prop_assert!(((lhs - rhs) / rhs).abs() < 1e-6, "{lhs} vs {rhs}");
        prop_assert!(((d.signed_volume() + i.signed_volume() - a.signed_volume()) / rhs).abs() < 1e-6);
        for m in [&u, &i, &d] {
            if m.face_count() > 0 {
                let r = report(m);
                prop_assert!(r.is_watertight());
            }
        }
    }
}

fn strict_timing() -> bool {
    std::env::var("AMG_STRICT_TIMING").is_ok_and(|v| v == "1")
}

/// Done-when: two 1 M-triangle offset spheres union in < 1 s end-to-end including conversion
/// (asserted with `AMG_STRICT_TIMING=1`; measured in `benches/mesh_boolean.rs`).
#[test]
fn timing_union_1m_spheres() {
    let (a, b) = boolean_pair(BenchSize::M1, 7);
    assert!(a.face_count() > 1_000_000 && b.face_count() > 1_000_000);
    // Warm-up (TBB thread pool start).
    run(BoolOp::Union, &a, &b);
    let started = std::time::Instant::now();
    let (u, rep) = boolean(BoolOp::Union, &a, &b, &BoolOpts::default(), &ctx()).unwrap();
    let ms = started.elapsed().as_secs_f64() * 1e3;
    eprintln!(
        "1M x 1M union: {ms:.0} ms (report {} ms), {} tris",
        rep.millis,
        u.face_count()
    );
    assert_watertight(&u);
    if strict_timing() {
        assert!(ms < 1000.0, "union took {ms:.0} ms");
    }
}

#[test]
fn golden_boolean_spheres() {
    let a = icosphere(Point3::ZERO, 10.0, 4);
    let b = icosphere(Point3::new(7.0, 3.0, 1.0), 8.0, 4);
    amg_testkit::assert_golden!(
        "mesh",
        "boolean_spheres_union",
        soup(&run(BoolOp::Union, &a, &b))
    );
    amg_testkit::assert_golden!(
        "mesh",
        "boolean_spheres_difference",
        soup(&run(BoolOp::Difference, &a, &b))
    );
    amg_testkit::assert_golden!(
        "mesh",
        "boolean_spheres_intersection",
        soup(&run(BoolOp::Intersection, &a, &b))
    );
}

#[test]
fn golden_boolean_box_minus_spheres() {
    let a = cube(Point3::splat(-10.0), 20.0);
    let s1 = icosphere(Point3::new(10.0, 10.0, 10.0), 6.0, 3);
    let s2 = icosphere(Point3::new(-10.0, 0.0, 0.0), 5.0, 3);
    let (m, _) = boolean_n(
        BoolOp::Difference,
        &[&a, &s1, &s2],
        &BoolOpts::default(),
        &ctx(),
    )
    .unwrap();
    amg_testkit::assert_golden!("mesh", "boolean_box_minus_spheres", soup(&m));
}
