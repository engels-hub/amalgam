//! Unit tests (INS-01): analytic cube values, fits, conventions, errors, snapping, rows.

#![allow(clippy::float_cmp, reason = "exact expected values")]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_core::{Dir3, LengthUnit, Plane, Point3, Quat, Ray3, Tolerance, Transform, Unit, Vec3};
use amg_jobs::JobCtx;
use amg_mesh::{FaceId, IndexedMesh, MeshBody};
use amg_testkit::shapes;

use super::prims::{Prim, PrimFeature, prim_distance};
use super::*;
use crate::inspect::InspectError;

fn ctx() -> JobCtx {
    JobCtx::detached()
}

fn opts() -> MeasureOpts {
    MeasureOpts::default()
}

/// 10 mm cube on [0, 10]³ (testkit: vertex i has x = bit 0, y = bit 1, z = bit 2).
fn cube() -> MeshBody {
    let s = shapes::cube(Point3::ZERO, 10.0);
    MeshBody::new(IndexedMesh::new(s.positions, s.triangles).unwrap())
}

fn op(body: &MeshBody, to_world: Transform, element: MeshElement) -> Operand {
    Operand::Mesh(MeshOperand {
        body: body.clone(),
        to_world,
        element,
    })
}

fn region(body: &MeshBody, seed: u32) -> MeshElement {
    MeshElement::Faces(
        grow_planar_region(body.mesh(), FaceId(seed), &RegionOpts::default(), &ctx()).unwrap(),
    )
}

/// A rigid transform that is not trivial.
fn rigid() -> Transform {
    Transform::from_rotation_translation(
        Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0).normalize(), 0.7),
        Vec3::new(3.0, -7.0, 11.0),
    )
}

#[test]
fn cube_analytic_values() {
    let c = cube();
    for t in [Transform::IDENTITY, rigid()] {
        // Faces 0, 1: z = 0; 2, 3: z = 10; 4, 5: y = 0.
        let bottom = region(&c, 0);
        assert_eq!(
            bottom,
            MeshElement::Faces(vec![FaceId(0), FaceId(1)].into())
        );
        let top = region(&c, 2);
        let side = region(&c, 4);
        let e = measure_one(&op(&c, t, bottom.clone()), &opts(), &ctx()).unwrap();
        assert!((e.area.unwrap().get() - 100.0).abs() <= 1e-9);
        assert_eq!(e.face_count, Some(2));
        assert!(e.plane.is_some());

        let edge = measure_one(
            &op(&c, t, MeshElement::Edge { a: 0, b: 1 }),
            &opts(),
            &ctx(),
        )
        .unwrap();
        assert!((edge.length.unwrap().to_mm() - 10.0).abs() <= 1e-12);

        let r = measure(
            &op(&c, t, bottom.clone()),
            Some(&op(&c, t, top.clone())),
            &opts(),
            &ctx(),
        )
        .unwrap();
        let p = r.pair.unwrap();
        assert!((p.min_distance.distance.to_mm() - 10.0).abs() <= 1e-12);
        let a = p.angle.unwrap();
        assert_eq!(a.kind, AngleKind::PlanePlane);
        assert!((a.angle.to_rad() - PI).abs() <= 1e-12, "{a:?}");
        assert!(a.supplement.to_rad().abs() <= 1e-12);
        assert!((p.center_distance.to_mm() - 10.0).abs() <= 1e-12);

        let r = measure(
            &op(&c, t, bottom.clone()),
            Some(&op(&c, t, side)),
            &opts(),
            &ctx(),
        )
        .unwrap();
        let p = r.pair.unwrap();
        assert!((p.angle.unwrap().angle.to_rad() - FRAC_PI_2).abs() <= 1e-12);
        assert!(p.min_distance.intersecting);

        let body = measure_one(&op(&c, t, MeshElement::Body), &opts(), &ctx()).unwrap();
        assert!((body.volume.unwrap().get() - 1000.0).abs() <= 1e-9);
        assert!((body.area.unwrap().get() - 600.0).abs() <= 1e-9);
        assert_eq!(body.face_count, Some(12));

        let above = Operand::Point(t.transform_point3(Point3::new(5.0, 5.0, 15.0)));
        for el in [top.clone(), MeshElement::Body] {
            let d = min_distance(&above, &op(&c, t, el), &opts(), &ctx()).unwrap();
            assert!((d.distance.to_mm() - 5.0).abs() <= 1e-12, "{d:?}");
            assert!(matches!(
                d.b.feature,
                WitnessFeature::Face(_) | WitnessFeature::Edge(..)
            ));
            assert_eq!(d.a.feature, WitnessFeature::Point);
        }
    }
}

#[test]
fn body_bbox_and_containment() {
    let c = cube();
    let e = measure_one(
        &op(&c, Transform::from_translation(Vec3::X), MeshElement::Body),
        &opts(),
        &ctx(),
    )
    .unwrap();
    let b = e.bbox.unwrap();
    assert_eq!(b.min, Point3::new(1.0, 0.0, 0.0));
    assert_eq!(b.max, Point3::new(11.0, 10.0, 10.0));
    // A 2 mm cube inside the 10 mm cube.
    let small = MeshBody::new({
        let s = shapes::cube(Point3::splat(4.0), 2.0);
        IndexedMesh::new(s.positions, s.triangles).unwrap()
    });
    let d = min_distance(
        &op(&small, Transform::IDENTITY, MeshElement::Body),
        &op(&c, Transform::IDENTITY, MeshElement::Body),
        &opts(),
        &ctx(),
    )
    .unwrap();
    assert!((d.distance.to_mm() - 4.0).abs() <= 1e-12);
    assert_eq!(d.contained, Some(true));
    let d = min_distance(
        &op(
            &small,
            Transform::from_translation(Vec3::new(20.0, 0.0, 0.0)),
            MeshElement::Body,
        ),
        &op(&c, Transform::IDENTITY, MeshElement::Body),
        &opts(),
        &ctx(),
    )
    .unwrap();
    assert_eq!(d.contained, Some(false));
    assert!((d.distance.to_mm() - 14.0).abs() <= 1e-12);
    // Not bodies → None.
    let d = min_distance(
        &Operand::Point(Point3::ZERO),
        &op(&c, Transform::IDENTITY, MeshElement::Body),
        &opts(),
        &ctx(),
    )
    .unwrap();
    assert_eq!(d.contained, None);
    assert!(d.intersecting);
}

#[test]
fn non_rigid_transform_is_rejected() {
    let c = cube();
    for t in [
        Transform::from_scale(Vec3::splat(2.0)),
        Transform::from_scale(Vec3::new(1.0, 1.0, -1.0)),
        Transform::from_mat3(amg_core::Mat3::from_cols(
            Vec3::X,
            Vec3::new(0.1, 1.0, 0.0),
            Vec3::Z,
        )),
    ] {
        let o = op(&c, t, MeshElement::Body);
        assert!(matches!(
            measure_one(&o, &opts(), &ctx()),
            Err(InspectError::NonRigidTransform)
        ));
        assert!(matches!(
            min_distance(&Operand::Point(Point3::ZERO), &o, &opts(), &ctx()),
            Err(InspectError::NonRigidTransform)
        ));
        assert!(matches!(
            measure(&o, None, &opts(), &ctx()),
            Err(InspectError::NonRigidTransform)
        ));
    }
}

#[test]
fn bad_elements() {
    let c = cube();
    let id = Transform::IDENTITY;
    for el in [
        MeshElement::Vertex(8),
        MeshElement::Edge { a: 0, b: 0 },
        MeshElement::Edge { a: 0, b: 99 },
        MeshElement::Faces(vec![FaceId(12)].into()),
        MeshElement::Faces(Vec::new().into()),
        MeshElement::Polyline {
            verts: vec![1].into(),
            closed: false,
        },
        MeshElement::SurfacePoint {
            face: FaceId(12),
            bary: [1.0, 0.0, 0.0],
        },
        MeshElement::FaceGroup(FaceGroupId(7)),
    ] {
        let o = op(&c, id, el.clone());
        assert!(
            matches!(
                measure_one(&o, &opts(), &ctx()),
                Err(InspectError::BadElement(_))
            ),
            "{el:?}"
        );
        assert!(matches!(
            min_distance(&o, &Operand::Point(Point3::ZERO), &opts(), &ctx()),
            Err(InspectError::BadElement(_))
        ));
    }
    assert!(matches!(
        grow_planar_region(c.mesh(), FaceId(12), &RegionOpts::default(), &ctx()),
        Err(InspectError::BadElement(_))
    ));
    // 0–7 is a space diagonal, not an edge.
    assert!(matches!(
        grow_feature_chain(c.mesh(), 0, 7, 0.5, &ctx()),
        Err(InspectError::BadElement(_))
    ));
}

#[test]
fn group_and_point_elements() {
    let c = cube();
    let g = c.mesh().groups.group_of(FaceId(0));
    let e = measure_one(
        &op(&c, Transform::IDENTITY, MeshElement::FaceGroup(g)),
        &opts(),
        &ctx(),
    )
    .unwrap();
    assert!((e.area.unwrap().get() - 600.0).abs() <= 1e-9);
    assert!(e.plane.is_none(), "a closed body is not planar");
    let sp = MeshElement::SurfacePoint {
        face: FaceId(2),
        bary: [0.25, 0.25, 0.5],
    };
    let e = measure_one(&op(&c, Transform::IDENTITY, sp), &opts(), &ctx()).unwrap();
    assert_eq!(e.position.unwrap().z, 10.0);
    let v = measure_one(&op(&c, rigid(), MeshElement::Vertex(7)), &opts(), &ctx()).unwrap();
    assert!((v.position.unwrap() - rigid().transform_point3(Point3::splat(10.0))).length() < 1e-12);
}

/// Block [-10, 10]² × [0, 10] with a through hole r = 5 on Z, `n` segments (n divisible by 8,
/// so the square corners are ring vertices).
pub(crate) fn holed_block(n: usize, r: f64) -> IndexedMesh {
    let mut p = Vec::new();
    for z in [10.0, 0.0] {
        for i in 0..n {
            let th = TAU * i as f64 / n as f64;
            p.push(Point3::new(r * th.cos(), r * th.sin(), z));
        }
        for i in 0..n {
            let th = TAU * i as f64 / n as f64;
            let (c, s) = (th.cos(), th.sin());
            let k = 10.0 / c.abs().max(s.abs());
            p.push(Point3::new(k * c, k * s, z));
        }
    }
    let n32 = n as u32;
    let (ct, st, cb, sb) = (0, n32, 2 * n32, 3 * n32);
    let mut t = Vec::new();
    for i in 0..n32 {
        let j = (i + 1) % n32;
        t.push([ct + i, st + i, st + j]);
        t.push([ct + i, st + j, ct + j]);
        t.push([cb + i, sb + j, sb + i]);
        t.push([cb + i, cb + j, sb + j]);
        t.push([sb + i, sb + j, st + j]);
        t.push([sb + i, st + j, st + i]);
        t.push([cb + i, ct + i, ct + j]);
        t.push([cb + i, ct + j, cb + j]);
    }
    IndexedMesh::new(p, t).unwrap()
}

#[test]
fn hole_chain_and_circle_fit() {
    let m = holed_block(128, 5.0);
    let body = MeshBody::new(m.clone());
    assert!(
        measure_one(
            &op(&body, Transform::IDENTITY, MeshElement::Body),
            &opts(),
            &ctx()
        )
        .unwrap()
        .volume
        .is_some()
    );
    let chain = grow_feature_chain(&m, 0, 1, 30f64.to_radians(), &ctx()).unwrap();
    let MeshElement::Polyline { verts, closed } = &chain else {
        panic!("{chain:?}")
    };
    assert!(closed);
    assert_eq!(verts.len(), 128);
    assert_eq!(&verts[..2], &[0, 1]);
    for t in [Transform::IDENTITY, rigid()] {
        let e = measure_one(&op(&body, t, chain.clone()), &opts(), &ctx()).unwrap();
        let c = e.circle.expect("circle accepted");
        assert!((c.circle.radius - 5.0).abs() <= 1e-9, "{c:?}");
        assert!(
            (c.circle.center - t.transform_point3(Point3::new(0.0, 0.0, 10.0))).length() <= 1e-9
        );
        // Vertices 0 → 1 run counter-clockwise seen from +Z.
        assert!((c.circle.normal.get() - t.transform_vector3(Vec3::Z)).length() <= 1e-9);
        assert!(e.direction.is_none());
        let len = e.length.unwrap().to_mm();
        assert!((len - 128.0 * 2.0 * 5.0 * (PI / 128.0).sin()).abs() <= 1e-9);
    }
    // The top annulus is one planar region (2 × 128 faces); the rim stops it.
    let top = grow_planar_region(&m, FaceId(0), &RegionOpts::default(), &ctx()).unwrap();
    assert_eq!(top.len(), 256);
    assert!(top.iter().all(|f| f.0 % 8 < 2));
    // Clicking an outer wall edge (between two coplanar wall quads) is not a feature edge.
    let one = grow_feature_chain(&m, 385, 130, 30f64.to_radians(), &ctx()).unwrap();
    assert_eq!(
        one,
        MeshElement::Polyline {
            verts: vec![385, 130].into(),
            closed: false
        }
    );
}

/// SplitMix64 for reproducible noise.
pub(crate) struct Rng(pub u64);
impl Rng {
    pub(crate) fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    pub(crate) fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub(crate) fn gauss(&mut self) -> f64 {
        let u = self.unit().max(1e-300);
        let v = self.unit();
        (-2.0 * u.ln()).sqrt() * (TAU * v).cos()
    }
}

#[test]
fn noisy_circle_fit() {
    let mut rng = Rng(7);
    let frame = rigid();
    for _ in 0..20 {
        // In-plane noise σ = 0.002 mm (out-of-plane noise of that size exceeds the 1e-3 mm
        // planarity gate of note §2.4 and is rejected, tested below).
        let pts: Vec<Point3> = (0..64)
            .map(|i| {
                let th = TAU * f64::from(i) / 64.0;
                let p = Point3::new(
                    5.0 * th.cos() + 0.002 * rng.gauss(),
                    5.0 * th.sin() + 0.002 * rng.gauss(),
                    0.0,
                );
                frame.transform_point3(p)
            })
            .collect();
        let f = fit_circle(&pts, &opts()).expect("accepted");
        assert!((f.circle.radius - 5.0).abs() <= 0.005, "{f:?}");
        assert!(f.rms > 0.0 && f.rms < 0.004);
    }
    let pts: Vec<Point3> = (0..64)
        .map(|i| {
            let th = TAU * f64::from(i) / 64.0;
            Point3::new(5.0 * th.cos(), 5.0 * th.sin(), 0.002 * rng.gauss())
        })
        .collect();
    assert!(fit_circle(&pts, &opts()).is_none());
    // A square is not a circle; too few points.
    let sq = [
        Point3::ZERO,
        Point3::X,
        Point3::new(1.0, 1.0, 0.0),
        Point3::Y,
        Point3::new(0.5, 0.0, 0.0),
    ];
    assert!(fit_circle(&sq, &opts()).is_none());
    assert!(fit_circle(&sq[..4], &opts()).is_none());
}

#[test]
fn circumcircle() {
    let t = rigid();
    let on =
        |th: f64| t.transform_point3(Point3::new(1.0 + 3.0 * th.cos(), 2.0 + 3.0 * th.sin(), 4.0));
    let c = circle_through(on(0.1), on(1.9), on(4.0), &Tolerance::DEFAULT).unwrap();
    assert!((c.radius - 3.0).abs() < 1e-12);
    assert!((c.center - t.transform_point3(Point3::new(1.0, 2.0, 4.0))).length() < 1e-12);
    assert!((c.normal.get() - t.transform_vector3(Vec3::Z)).length() < 1e-12);
    assert!(matches!(
        circle_through(
            Point3::ZERO,
            Point3::X,
            Point3::new(2.0, 0.0, 0.0),
            &Tolerance::DEFAULT
        ),
        Err(InspectError::Degenerate(_))
    ));
    assert!(circle_through(Point3::ZERO, Point3::ZERO, Point3::X, &Tolerance::DEFAULT).is_err());
}

fn entity_line(d: Vec3) -> EntityMeasure {
    let mut e = EntityMeasure::empty(OperandKind::Edge);
    e.direction = Dir3::new(d, &Tolerance::DEFAULT);
    e
}

fn entity_plane(n: Vec3) -> EntityMeasure {
    let mut e = EntityMeasure::empty(OperandKind::Faces);
    e.plane = Some(PlaneFit {
        plane: Plane::from_normal(Point3::ZERO, Dir3::new(n, &Tolerance::DEFAULT).unwrap()),
        max_deviation: 0.0,
    });
    e
}

#[test]
fn angle_conventions() {
    let ll = angle_between(&entity_line(Vec3::X), &entity_line(-Vec3::X)).unwrap();
    assert_eq!((ll.kind, ll.angle.to_rad()), (AngleKind::LineLine, 0.0));
    assert_eq!(ll.supplement.to_rad(), PI);
    let ll = angle_between(
        &entity_line(Vec3::X),
        &entity_line(Vec3::new(-1.0, 1.0, 0.0)),
    )
    .unwrap();
    assert!((ll.angle.to_rad() - PI / 4.0).abs() < 1e-15);
    let pp = angle_between(
        &entity_plane(Vec3::Z),
        &entity_plane(Vec3::new(0.0, 1.0, 1.0)),
    )
    .unwrap();
    assert!((pp.angle.to_rad() - PI / 4.0).abs() < 1e-15);
    assert!((pp.supplement.to_rad() - 3.0 * PI / 4.0).abs() < 1e-15);
    // A tiny angle keeps full precision (atan2, not acos).
    let tiny = angle_between(
        &entity_plane(Vec3::Z),
        &entity_plane(Vec3::new(1e-10, 0.0, 1.0)),
    )
    .unwrap();
    assert!((tiny.angle.to_rad() - 1e-10).abs() < 1e-24);
    let lp = angle_between(
        &entity_line(Vec3::new(1.0, 0.0, 1.0)),
        &entity_plane(Vec3::Z),
    )
    .unwrap();
    assert_eq!(lp.kind, AngleKind::LinePlane);
    assert!((lp.angle.to_rad() - PI / 4.0).abs() < 1e-15);
    let pl = angle_between(&entity_plane(Vec3::Z), &entity_line(Vec3::Z)).unwrap();
    assert!((pl.angle.to_rad() - FRAC_PI_2).abs() < 1e-15 && pl.supplement.to_rad().abs() < 1e-15);
    assert!(
        angle_between(
            &entity_line(Vec3::X),
            &EntityMeasure::empty(OperandKind::Point)
        )
        .is_none()
    );
}

#[test]
fn primitive_distances() {
    let t = [Point3::ZERO, Point3::X, Point3::Y];
    let d = prim_distance(&Prim::Point(Point3::new(0.25, 0.25, 2.0)), &Prim::Tri(t));
    assert_eq!((d.dist_sq, d.fb), (4.0, PrimFeature::Interior));
    let d = prim_distance(&Prim::Point(Point3::new(-1.0, -1.0, 0.0)), &Prim::Tri(t));
    assert_eq!(d.fb, PrimFeature::Corner(0));
    let d = prim_distance(&Prim::Tri(t), &Prim::Point(Point3::new(1.0, 1.0, 0.0)));
    assert_eq!(d.fa, PrimFeature::Edge(1, 2));
    assert!((d.dist_sq - 0.5).abs() < 1e-15);
    // Crossing triangles.
    let u = [
        Point3::new(0.2, 0.2, -1.0),
        Point3::new(0.2, 0.2, 1.0),
        Point3::new(5.0, 5.0, 0.0),
    ];
    let d = prim_distance(&Prim::Tri(t), &Prim::Tri(u));
    assert_eq!(d.dist_sq, 0.0);
    assert!(
        matches!(
            (d.fa, d.fb),
            (PrimFeature::Edge(..), PrimFeature::Interior)
                | (PrimFeature::Interior, PrimFeature::Edge(..))
        ),
        "{d:?}"
    );
    // Skew edges.
    let v = [
        Point3::new(0.5, -1.0, 1.0),
        Point3::new(0.5, 1.0, 1.0),
        Point3::new(0.5, 0.0, 3.0),
    ];
    let d = prim_distance(
        &Prim::Seg([Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]),
        &Prim::Tri(v),
    );
    assert!((d.dist_sq - 1.0).abs() < 1e-15);
    assert_eq!(
        (d.fa, d.fb),
        (PrimFeature::Edge(0, 1), PrimFeature::Edge(0, 1))
    );
    // Random pairs against dense sampling: never larger than any sampled pair distance.
    let mut rng = Rng(3);
    let mut p = || Point3::new(rng.unit(), rng.unit(), rng.unit()) * 4.0;
    for _ in 0..200 {
        let a = [p(), p(), p()];
        let b = [p(), p(), p()];
        let d = prim_distance(&Prim::Tri(a), &Prim::Tri(b));
        assert!((d.a.distance_squared(d.b) - d.dist_sq).abs() < 1e-12);
        for i in 0..=8 {
            for j in 0..=(8 - i) {
                let (s, tt) = (f64::from(i) / 8.0, f64::from(j) / 8.0);
                let q = a[0] + (a[1] - a[0]) * s + (a[2] - a[0]) * tt;
                let r = prim_distance(&Prim::Point(q), &Prim::Tri(b));
                assert!(d.dist_sq <= r.dist_sq + 1e-12);
            }
        }
    }
}

#[test]
fn snapping() {
    let c = cube();
    let t = rigid();
    let down = |x: f64, y: f64| Ray3 {
        origin: t.transform_point3(Point3::new(x, y, 20.0)),
        dir: Dir3::new(t.transform_vector3(-Vec3::Z), &Tolerance::DEFAULT).unwrap(),
    };
    let so = SnapOpts {
        vertex_radius: 0.5,
        edge_radius: 0.3,
    };
    // Face 2 = [4, 5, 7] on z = 10.
    let (s, p) = snap_on_face(c.mesh(), &t, FaceId(2), &down(9.8, 9.7), &so).unwrap();
    assert_eq!(s, Snapped::Vertex(7));
    assert!((p - t.transform_point3(Point3::splat(10.0))).length() < 1e-12);
    let (s, p) = snap_on_face(c.mesh(), &t, FaceId(2), &down(5.0, 0.2), &so).unwrap();
    let Snapped::Edge { a, b, t: k } = s else {
        panic!("{s:?}")
    };
    assert_eq!((a, b), (4, 5));
    assert!((k - 0.5).abs() < 1e-12);
    assert!((p - t.transform_point3(Point3::new(5.0, 0.0, 10.0))).length() < 1e-12);
    let (s, p) = snap_on_face(c.mesh(), &t, FaceId(2), &down(7.0, 3.0), &so).unwrap();
    let Snapped::Face { face, bary } = s else {
        panic!("{s:?}")
    };
    assert_eq!(face, FaceId(2));
    assert!((bary.iter().sum::<f64>() - 1.0).abs() < 1e-12);
    assert!((p - t.transform_point3(Point3::new(7.0, 3.0, 10.0))).length() < 1e-12);
    // A ray just outside the triangle still lands on its closest point.
    let (_, p) = snap_on_face(
        c.mesh(),
        &t,
        FaceId(2),
        &down(3.0, 5.0),
        &SnapOpts {
            vertex_radius: 0.0,
            edge_radius: 0.0,
        },
    )
    .unwrap();
    assert!((t.inverse().transform_point3(p).z - 10.0).abs() < 1e-12);
    assert!(snap_on_face(c.mesh(), &t, FaceId(99), &down(1.0, 1.0), &so).is_none());
}

#[test]
fn cost_and_rows() {
    let c = cube();
    let id = Transform::IDENTITY;
    let pt = Operand::Point(Point3::ZERO);
    assert_eq!(estimated_cost(&pt, None), Cost::Inline);
    assert_eq!(
        estimated_cost(&pt, Some(&op(&c, id, MeshElement::Edge { a: 0, b: 1 }))),
        Cost::Inline
    );
    assert_eq!(
        estimated_cost(&pt, Some(&op(&c, id, MeshElement::Body))),
        Cost::Job
    );
    assert_eq!(
        estimated_cost(
            &op(
                &c,
                id,
                MeshElement::Faces((0..65).map(FaceId).collect::<Vec<_>>().into())
            ),
            None
        ),
        Cost::Job
    );

    let r = measure(
        &op(&c, id, region(&c, 0)),
        Some(&op(&c, id, MeshElement::Edge { a: 4, b: 5 })),
        &opts(),
        &ctx(),
    )
    .unwrap();
    let rows = report_rows(&r, &DisplayPrefs::default());
    let keys: Vec<&str> = rows.iter().map(|r| r.key).collect();
    assert_eq!(
        keys,
        [
            "distance",
            "dx",
            "dy",
            "dz",
            "angle",
            "center_distance",
            "first.area",
            "second.length"
        ]
    );
    assert_eq!(rows[0].text, "10 mm");
    assert_eq!(rows[4].text, "0 deg");
    assert_eq!(rows[6].text, "100 mm^2");
    let inch = DisplayPrefs {
        length: LengthUnit::Inch,
        angle: Unit::Degree,
        precision: 4,
    };
    let rows = report_rows(&r, &inch);
    assert_eq!(rows[6].text, "0.155 in^2");
    assert_eq!(rows[7].text, "0.3937 in");
    assert_eq!(area_unit(LengthUnit::Foot), Unit::SquareInch);
    assert_eq!(volume_unit(LengthUnit::Centimetre), Unit::CubicCentimetre);
    // Single entity: unnumbered labels; a circle gives diameter then radius.
    let m = holed_block(64, 5.0);
    let body = MeshBody::new(m.clone());
    let chain = grow_feature_chain(&m, 0, 1, 0.5, &ctx()).unwrap();
    let r = measure(&op(&body, id, chain), None, &opts(), &ctx()).unwrap();
    let rows = report_rows(&r, &DisplayPrefs::default());
    let keys: Vec<&str> = rows.iter().map(|r| r.key).collect();
    assert_eq!(keys, ["first.length", "first.diameter", "first.radius"]);
    assert_eq!(rows[1].text, "10 mm");
    assert_eq!(rows[2].label, "Radius");
}

#[test]
fn cancelled_jobs_stop() {
    let c = cube();
    let token = amg_jobs::CancelToken::new();
    token.cancel();
    let cx = JobCtx::with_token(token);
    assert!(matches!(
        grow_planar_region(c.mesh(), FaceId(0), &RegionOpts::default(), &cx),
        Err(InspectError::Cancelled)
    ));
    assert!(matches!(
        min_distance(
            &Operand::Point(Point3::ZERO),
            &op(&c, Transform::IDENTITY, MeshElement::Body),
            &opts(),
            &cx
        ),
        Err(InspectError::Cancelled | InspectError::Mesh(_))
    ));
}
