//! BREP-12 Done-when 3: every op parameter type is `Clone + Debug + PartialEq + Serialize +
//! DeserializeOwned` (S20 KH-01 prerequisite), and a forwarding decorator implementing
//! `BrepKernel` compiles (object safety).
//!
//! Runs with and without the `occt` feature. Each op WP that adds a trait method adds its
//! parameter types to `op_parameter_types_are_capturable`, a forwarding arm to `Forward` and a
//! call to `the_hardened_kernel_forwards_every_method` (S20 KH-17's `HardenedKernel`).
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::fmt::Debug;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use amg_brep::geom::{
    BSpline2dData, BSplineCurveData, BSplineSurfaceData, Curve2d, Curve3, CurveSeg, FaceBounds,
    GeomClass, Loop2, PlanarProfile, ProfileEdge2, Surface, Tag, ThinProfile,
};
use amg_brep::robust::{HardenedKernel, RetryPolicy};
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, ChamferSpec, CheckOpts, CheckReport, Classification,
    DistanceResult, DraftOpts, EdgeEnds, EdgeId, FaceBuild, FaceId, FilletOpts, FilletShape,
    FilletSpec, Glue, HealOpts, HelixSpec, JobCtx, KResult, KernelConfig, LoftSection, MassProps,
    OcctKernel, OffsetJoin, OpResult, PipeMode, PipeOpts, PrismExtent, PrismOpts, RayHit, SewOpts,
    Shape, ShapeHistory, ShapeKind, ShellOpts, SubKind, SubShape, SubShapeGeom, SurfacePoint,
    TessMesh, TessQuality, ThinFace, ThruSectionsOpts, Tolerance, Transition, UnifyOpts, VertexId,
};
use amg_core::{Dir3, Line3, Plane, Point2, Point3, Ray3, Transform, Vec2, Vec3};
use serde::Serialize;
use serde::de::DeserializeOwned;

/// What S20's `KernelOp` capture needs of a parameter type.
fn assert_op_param<T: Clone + Debug + PartialEq + Serialize + DeserializeOwned>() {}

/// Serialises `value`, reads it back, and checks it is unchanged.
fn round_trip<T: Clone + Debug + PartialEq + Serialize + DeserializeOwned>(value: &T) {
    let json = serde_json::to_string(value).unwrap();
    let back: T = serde_json::from_str(&json).unwrap();
    assert_eq!(&back, value, "{json}");
}

#[test]
fn op_parameter_types_are_capturable() {
    // Sub-shape references and ids.
    assert_op_param::<FaceId>();
    assert_op_param::<EdgeId>();
    assert_op_param::<VertexId>();
    assert_op_param::<SubShape>();
    assert_op_param::<SubKind>();
    assert_op_param::<ShapeKind>();
    assert_op_param::<Tolerance>();
    assert_op_param::<Plane>();
    // BREP-14 primitive, revol and transform parameters.
    assert_op_param::<Vec3>();
    assert_op_param::<Line3>();
    assert_op_param::<Transform>();
    assert_op_param::<f64>();
    // BREP-09 query parameters.
    assert_op_param::<Point2>();
    assert_op_param::<Point3>();
    assert_op_param::<Ray3>();
    // geom, including the BREP-03 builder inputs.
    assert_op_param::<Tag>();
    assert_op_param::<Curve2d>();
    assert_op_param::<BSpline2dData>();
    assert_op_param::<ProfileEdge2>();
    assert_op_param::<Loop2>();
    assert_op_param::<PlanarProfile>();
    assert_op_param::<Vec<PlanarProfile>>();
    assert_op_param::<CurveSeg>();
    assert_op_param::<Vec<CurveSeg>>();
    assert_op_param::<FaceBounds>();
    assert_op_param::<Vec<Vec<Point3>>>();
    assert_op_param::<ShapeHistory>();
    assert_op_param::<Curve3>();
    assert_op_param::<BSplineCurveData>();
    assert_op_param::<Surface>();
    assert_op_param::<BSplineSurfaceData>();
    assert_op_param::<GeomClass>();
    // Op options.
    assert_op_param::<BoolOp>();
    assert_op_param::<Glue>();
    assert_op_param::<BoolOpts>();
    assert_op_param::<PrismExtent>();
    assert_op_param::<PrismOpts>();
    assert_op_param::<PipeMode>();
    assert_op_param::<PipeOpts>();
    assert_op_param::<Transition>();
    assert_op_param::<ThruSectionsOpts>();
    assert_op_param::<LoftSection>();
    assert_op_param::<FilletSpec>();
    assert_op_param::<FilletShape>();
    assert_op_param::<FilletOpts>();
    assert_op_param::<ChamferSpec>();
    assert_op_param::<OffsetJoin>();
    assert_op_param::<ShellOpts>();
    assert_op_param::<DraftOpts>();
    assert_op_param::<HealOpts>();
    assert_op_param::<UnifyOpts>();
    assert_op_param::<SewOpts>();
    assert_op_param::<HelixSpec>();
    assert_op_param::<CheckOpts>();
    assert_op_param::<TessQuality>();
    // Op-shaped composites as a `KernelOp` would hold them.
    assert_op_param::<Vec<(EdgeId, FilletSpec)>>();
    assert_op_param::<Vec<(EdgeId, ChamferSpec)>>();
    assert_op_param::<Vec<LoftSection>>();
}

#[test]
fn op_parameters_survive_a_serde_round_trip() {
    let frame = Plane::XY;
    let dir = Dir3::new_unchecked(Vec3::Z);
    round_trip(&BoolOpts::default());
    round_trip(&BoolOpts {
        fuzzy: 1e-4,
        glue: Glue::Shift,
        check_inverted: false,
        use_obb: true,
        parallel: Some(true),
    });
    round_trip(&BoolOp::Cut);
    round_trip(&PrismExtent::Vector(Vec3::new(0.0, 0.0, 5.0)));
    round_trip(&PrismExtent::Infinite { dir, both: true });
    round_trip(&PrismOpts {
        taper: 0.1,
        merge_coplanar: true,
    });
    round_trip(&PipeOpts {
        mode: PipeMode::Guide {
            wire: 2,
            contact: true,
        },
        twist: Some(vec![(0.0, 0.0), (1.0, 1.57)]),
        scale: None,
        solid: true,
        transition: Transition::RoundCorner,
    });
    round_trip(&PipeMode::Fixed { binormal: dir });
    round_trip(&PipeMode::Binormal(dir));
    round_trip(&ThruSectionsOpts {
        solid: true,
        ruled: false,
        check_compatibility: true,
        smoothing: false,
        max_degree: 8,
    });
    round_trip(&vec![
        LoftSection::Wire(0),
        LoftSection::Point(Point3::new(1.0, 2.0, 3.0)),
    ]);
    round_trip(&vec![
        (EdgeId(0), FilletSpec::Constant(1.0)),
        (
            EdgeId(4),
            FilletSpec::Variable(vec![(0.0, 1.0), (1.0, 2.0)]),
        ),
        (EdgeId(5), FilletSpec::Chord(0.5)),
    ]);
    round_trip(&FilletOpts {
        shape: FilletShape::Polynomial,
        tol_app_3d: Some(1e-5),
        fleche: None,
    });
    round_trip(&vec![
        ChamferSpec::Equal(1.0),
        ChamferSpec::TwoDistances {
            d1: 1.0,
            d2: 2.0,
            face: FaceId(3),
        },
        ChamferSpec::DistanceAngle {
            d: 1.0,
            angle: 0.5,
            face: FaceId(1),
        },
    ]);
    round_trip(&ShellOpts {
        thickness: -1.0,
        join: OffsetJoin::Intersection,
        per_face: vec![(FaceId(2), 2.0)],
        tol: 1e-6,
    });
    round_trip(&DraftOpts {
        pull: dir,
        angle: 0.05,
        neutral: frame,
    });
    round_trip(&HealOpts {
        tol: 1e-6,
        fix_small_edges: true,
        fix_orientation: true,
        max_tol: 0.1,
    });
    round_trip(&UnifyOpts {
        faces: true,
        edges: true,
        concat_bsplines: false,
        lin_tol: 1e-6,
        ang_tol: 1e-9,
    });
    round_trip(&SewOpts {
        tol: 1e-4,
        non_manifold: false,
        make_solid: true,
    });
    round_trip(&HelixSpec {
        axis: frame,
        radius: 3.0,
        pitch: 0.5,
        height: 10.0,
        taper: 0.0,
        left_handed: true,
    });
    round_trip(&CheckOpts {
        self_interference: true,
    });
    for q in [
        TessQuality::preview(),
        TessQuality::display(),
        TessQuality::export(),
    ] {
        round_trip(&q);
    }
    round_trip(&Surface::Cylinder { frame, radius: 2.0 });
    let profile = PlanarProfile {
        plane: frame,
        outer: Loop2 {
            edges: vec![
                ProfileEdge2 {
                    curve: Curve2d::Arc {
                        center: Point2::ZERO,
                        radius: 1.0,
                        start_angle: 0.0,
                        sweep: std::f64::consts::PI,
                    },
                    tag: Tag(1),
                },
                ProfileEdge2 {
                    curve: Curve2d::BSpline(BSpline2dData {
                        degree: 1,
                        knots: vec![0.0, 0.0, 1.0, 1.0],
                        poles: vec![Point2::new(-1.0, 0.0), Point2::new(1.0, 0.0)],
                        weights: Some(vec![1.0, 2.0]),
                    }),
                    tag: Tag(u64::MAX),
                },
            ],
        },
        holes: vec![Loop2 {
            edges: vec![ProfileEdge2 {
                curve: Curve2d::Ellipse {
                    center: Point2::new(0.0, 0.5),
                    major: Vec2::new(0.2, 0.0),
                    minor_radius: 0.1,
                    t0: std::f64::consts::TAU,
                    t1: 0.0,
                },
                tag: Tag(3),
            }],
        }],
    };
    round_trip(&vec![profile]);
    let seg = CurveSeg {
        curve: Curve3::Circle { frame, radius: 2.0 },
        t0: 1.0,
        t1: 0.0,
        tag: Tag(9),
    };
    round_trip(&FaceBounds::Loops(vec![vec![seg.clone()]]));
    round_trip(&FaceBounds::Polylines(vec![vec![
        Point3::ZERO,
        Point3::X,
        Point3::Y,
    ]]));
    round_trip(&FaceBounds::UvRect {
        u0: 0.0,
        u1: 1.0,
        v0: -1.0,
        v1: 2.0,
    });
    round_trip(&FaceBounds::Natural);
    round_trip(&vec![seg]);
}

#[test]
fn tessellation_presets_match_the_spec_table() {
    let (p, d, e) = (
        TessQuality::preview(),
        TessQuality::display(),
        TessQuality::export(),
    );
    assert!(p.relative && d.relative && e.relative);
    assert_eq!(
        [p.linear, d.linear, e.linear].map(f64::to_bits),
        [0.005, 0.001, 0.0001].map(f64::to_bits)
    );
    assert_eq!(
        [p.angular, d.angular, e.angular].map(f64::to_bits),
        [0.5, 0.35, 0.2].map(f64::to_bits)
    );
    assert_eq!(e.max_linear_abs, Some(0.01));
    assert_eq!(p.max_linear_abs, None);
}

/// A decorator in the shape of S20's `RecordingKernel<K>`: forwards every method unchanged.
/// There are no default methods, so a new trait method fails to compile here until forwarded.
struct Forward<K: BrepKernel> {
    inner: K,
    calls: AtomicUsize,
}

impl<K: BrepKernel> BrepKernel for Forward<K> {
    fn check(&self, s: &Shape, opts: &CheckOpts) -> CheckReport {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.check(s, opts)
    }

    fn make_planar_faces(
        &self,
        profiles: &[PlanarProfile],
        tol: &Tolerance,
    ) -> KResult<Vec<FaceBuild>> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_planar_faces(profiles, tol)
    }

    fn make_planar_polygon(
        &self,
        outer: &[Point3],
        holes: &[Vec<Point3>],
        tol: &Tolerance,
    ) -> KResult<Shape> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_planar_polygon(outer, holes, tol)
    }

    fn make_face(&self, surface: &Surface, bounds: &FaceBounds, tol: f64) -> KResult<FaceBuild> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_face(surface, bounds, tol)
    }

    fn make_wire(&self, segs: &[CurveSeg], tol: &Tolerance) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_wire(segs, tol)
    }

    fn make_thin_face(&self, p: &ThinProfile, tol: &Tolerance) -> KResult<ThinFace> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_thin_face(p, tol)
    }

    fn make_box(&self, frame: &Plane, size: Vec3) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_box(frame, size)
    }

    fn make_cylinder(
        &self,
        frame: &Plane,
        radius: f64,
        height: f64,
        angle: f64,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_cylinder(frame, radius, height, angle)
    }

    fn make_cone(
        &self,
        frame: &Plane,
        r1: f64,
        r2: f64,
        height: f64,
        angle: f64,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_cone(frame, r1, r2, height, angle)
    }

    fn make_sphere(&self, frame: &Plane, radius: f64) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_sphere(frame, radius)
    }

    fn make_torus(&self, frame: &Plane, major: f64, minor: f64, angle: f64) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_torus(frame, major, minor, angle)
    }

    fn prism(
        &self,
        profile: &Shape,
        extent: &PrismExtent,
        opts: &PrismOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.prism(profile, extent, opts, ctx)
    }

    fn revol(&self, profile: &Shape, axis: &Line3, angle: f64, ctx: &JobCtx) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.revol(profile, axis, angle, ctx)
    }

    fn pipe(
        &self,
        operands: &[Shape],
        profiles: usize,
        opts: &PipeOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.pipe(operands, profiles, opts, ctx)
    }

    fn thru_sections(
        &self,
        wires: &[Shape],
        sections: &[LoftSection],
        opts: &ThruSectionsOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.thru_sections(wires, sections, opts, ctx)
    }

    fn mirrored(&self, s: &Shape, plane: &Plane) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.mirrored(s, plane)
    }

    fn boolean(
        &self,
        op: BoolOp,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.boolean(op, objects, tools, opts, ctx)
    }

    fn split(
        &self,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.split(objects, tools, opts, ctx)
    }

    fn section(&self, a: &Shape, b: &Shape, ctx: &JobCtx) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.section(a, b, ctx)
    }

    fn fillet(
        &self,
        s: &Shape,
        edges: &[(EdgeId, FilletSpec)],
        opts: &FilletOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.fillet(s, edges, opts, ctx)
    }

    fn chamfer(
        &self,
        s: &Shape,
        edges: &[(EdgeId, ChamferSpec)],
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.chamfer(s, edges, ctx)
    }

    fn fillet_max_radius(
        &self,
        s: &Shape,
        edges: &[EdgeId],
        hi: f64,
        iterations: u32,
        ctx: &JobCtx,
    ) -> KResult<f64> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.fillet_max_radius(s, edges, hi, iterations, ctx)
    }

    fn shell(
        &self,
        s: &Shape,
        remove: &[FaceId],
        opts: &ShellOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.shell(s, remove, opts, ctx)
    }

    fn offset(
        &self,
        s: &Shape,
        distance: f64,
        join: OffsetJoin,
        tol: f64,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.offset(s, distance, join, tol, ctx)
    }

    fn thicken(
        &self,
        s: &Shape,
        thickness: f64,
        join: OffsetJoin,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.thicken(s, thickness, join, ctx)
    }

    fn draft(
        &self,
        s: &Shape,
        faces: &[FaceId],
        opts: &DraftOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.draft(s, faces, opts, ctx)
    }

    fn defeature(&self, s: &Shape, faces: &[FaceId], ctx: &JobCtx) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.defeature(s, faces, ctx)
    }

    fn replace_face(
        &self,
        s: &Shape,
        face: FaceId,
        with: &Surface,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.replace_face(s, face, with, ctx)
    }

    fn split_faces(
        &self,
        s: &Shape,
        faces: &[FaceId],
        tool: &Shape,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.split_faces(s, faces, tool, ctx)
    }

    fn offset_faces(&self, s: &Shape, faces: &[(FaceId, f64)], ctx: &JobCtx) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.offset_faces(s, faces, ctx)
    }

    fn heal(&self, s: &Shape, opts: &HealOpts, ctx: &JobCtx) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.heal(s, opts, ctx)
    }

    fn unify_same_domain(&self, s: &Shape, opts: &UnifyOpts) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.unify_same_domain(s, opts)
    }

    fn sew(&self, faces: &[Shape], opts: &SewOpts, ctx: &JobCtx) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.sew(faces, opts, ctx)
    }

    fn make_solid(&self, shell: &Shape) -> KResult<OpResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_solid(shell)
    }

    fn make_compound(&self, parts: &[Shape]) -> KResult<Shape> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.make_compound(parts)
    }

    fn plane_section(&self, s: &Shape, plane: &Plane) -> KResult<Vec<CurveSeg>> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.plane_section(s, plane)
    }

    fn project_edges(&self, s: &Shape, edges: &[EdgeId], plane: &Plane) -> KResult<Vec<CurveSeg>> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.project_edges(s, edges, plane)
    }

    fn tessellate(&self, s: &Shape, q: &TessQuality, ctx: &JobCtx) -> KResult<TessMesh> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.tessellate(s, q, ctx)
    }

    fn tessellate_many(
        &self,
        shapes: &[Shape],
        q: &TessQuality,
        ctx: &JobCtx,
    ) -> Vec<KResult<TessMesh>> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.tessellate_many(shapes, q, ctx)
    }

    fn tessellate_edge(&self, s: &Shape, e: EdgeId, q: &TessQuality) -> KResult<Vec<Point3>> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.tessellate_edge(s, e, q)
    }

    fn mass_props(&self, s: &Shape) -> KResult<MassProps> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.mass_props(s)
    }

    fn classify(&self, s: &Shape, p: Point3, tol: f64) -> KResult<Classification> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.classify(s, p, tol)
    }

    fn distance(&self, a: &Shape, b: &Shape) -> KResult<DistanceResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.distance(a, b)
    }

    fn closest_point(&self, s: &Shape, p: Point3) -> KResult<DistanceResult> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.closest_point(s, p)
    }

    fn ray_cast(&self, s: &Shape, ray: &Ray3, max_t: f64) -> KResult<Option<RayHit>> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.ray_cast(s, ray, max_t)
    }

    fn interference(&self, a: &Shape, b: &Shape, ctx: &JobCtx) -> KResult<Option<OpResult>> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.interference(a, b, ctx)
    }

    fn eval_surface(&self, s: &Shape, f: FaceId, uv: Point2) -> KResult<SurfacePoint> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.eval_surface(s, f, uv)
    }

    fn project_point_on_face(&self, s: &Shape, f: FaceId, p: Point3) -> KResult<(Point2, Point3)> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.project_point_on_face(s, f, p)
    }

    fn edge_ends(&self, s: &Shape, e: EdgeId) -> KResult<EdgeEnds> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.edge_ends(s, e)
    }

    fn sub_geom(&self, s: &Shape, sub: SubShape) -> KResult<SubShapeGeom> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.sub_geom(s, sub)
    }

    fn sub_geom_all(&self, s: &Shape) -> KResult<[Vec<SubShapeGeom>; 3]> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        self.inner.sub_geom_all(s)
    }
}

/// Features hold the kernel as a trait object.
fn as_dyn(kernel: Arc<dyn BrepKernel>) -> Arc<dyn BrepKernel> {
    kernel
}

#[test]
fn a_forwarding_decorator_is_a_kernel() {
    let base = OcctKernel::new(KernelConfig::default());
    let decorated = Forward {
        inner: base.clone(),
        calls: AtomicUsize::new(0),
    };
    // Decorators stack and erase to `dyn BrepKernel`.
    let stacked = Forward {
        inner: decorated,
        calls: AtomicUsize::new(0),
    };
    // S20 KH-17: the hardened kernel stacks with other decorators too.
    let hardened = HardenedKernel::new(stacked, RetryPolicy::default(), None);
    let kernel = as_dyn(Arc::new(hardened));
    // `dyn BrepKernel` is `Send + Sync`: it moves to and is shared across threads.
    let other = Arc::clone(&kernel);
    std::thread::spawn(move || drop(other)).join().unwrap();
    assert_eq!(Arc::strong_count(&kernel), 1);
    assert_eq!(base.config(), &KernelConfig::default());
}

#[cfg(feature = "occt")]
#[test]
fn the_decorator_forwards_calls() {
    let path = amg_testkit::testdata("brep/box.brep");
    let Some(path) = path else { return };
    let shape = Shape::read_text(std::fs::File::open(path).unwrap()).unwrap();
    let forward = Forward {
        inner: OcctKernel::new(KernelConfig::default()),
        calls: AtomicUsize::new(0),
    };
    let kernel: &dyn BrepKernel = &forward;
    let report = kernel.check(&shape, &CheckOpts::default());
    assert!(report.valid, "{report:?}");
    assert_eq!(forward.calls.load(Ordering::Relaxed), 1);
}

#[cfg(not(feature = "occt"))]
#[test]
fn builders_need_the_backend() {
    let kernel = OcctKernel::new(KernelConfig::default());
    let tol = Tolerance::DEFAULT;
    let square = [
        Point3::ZERO,
        Point3::X,
        Point3::new(1.0, 1.0, 0.0),
        Point3::Y,
    ];
    assert!(matches!(
        kernel.make_planar_polygon(&square, &[], &tol),
        Err(amg_brep::KernelError::BackendUnavailable)
    ));
    assert!(matches!(
        kernel.make_planar_faces(&[], &tol),
        Err(amg_brep::KernelError::BackendUnavailable)
    ));
    let sphere = Surface::Sphere {
        frame: Plane::XY,
        radius: 1.0,
    };
    assert!(matches!(
        kernel.make_face(&sphere, &FaceBounds::Natural, 1e-6),
        Err(amg_brep::KernelError::BackendUnavailable)
    ));
    assert!(matches!(
        kernel.make_wire(&[], &tol),
        Err(amg_brep::KernelError::BackendUnavailable)
    ));
}

#[cfg(not(feature = "occt"))]
#[test]
fn primitives_need_the_backend() {
    let kernel = OcctKernel::new(KernelConfig::default());
    let unavailable =
        |r: KResult<OpResult>| matches!(r, Err(amg_brep::KernelError::BackendUnavailable));
    let f = Plane::XY;
    assert!(unavailable(kernel.make_box(&f, Vec3::ONE)));
    assert!(unavailable(kernel.make_cylinder(&f, 1.0, 1.0, 1.0)));
    assert!(unavailable(kernel.make_cone(&f, 1.0, 0.0, 1.0, 1.0)));
    assert!(unavailable(kernel.make_sphere(&f, 1.0)));
    assert!(unavailable(kernel.make_torus(&f, 2.0, 1.0, 1.0)));
    // Checked before validation, as for every op: bad parameters still say "no backend".
    assert!(unavailable(kernel.make_box(&f, Vec3::ZERO)));
}

#[cfg(not(feature = "occt"))]
#[test]
fn booleans_need_the_backend() {
    let kernel = OcctKernel::new(KernelConfig::default());
    let unavailable =
        |r: KResult<OpResult>| matches!(r, Err(amg_brep::KernelError::BackendUnavailable));
    let ctx = JobCtx::detached();
    let opts = BoolOpts::default();
    // No shape exists without the backend, so empty lists: still "no backend", not InvalidInput.
    for op in [BoolOp::Fuse, BoolOp::Cut, BoolOp::Common] {
        assert!(unavailable(kernel.boolean(op, &[], &[], &opts, &ctx)));
    }
    assert!(unavailable(kernel.split(&[], &[], &opts, &ctx)));
}

/// The trait methods `Forward` implements, read from this file: a new trait method fails to
/// compile in `Forward` until forwarded, and this count makes the next test fail until it calls
/// the new method through `HardenedKernel` too.
#[cfg(feature = "occt")]
fn trait_method_count() -> usize {
    let src = include_str!("api_contract.rs");
    let start = src
        .find("impl<K: BrepKernel> BrepKernel for Forward<K> {")
        .unwrap();
    let end = start
        + src[start..]
            .find(
                "
}
",
            )
            .unwrap();
    src[start..end]
        .lines()
        .filter(|l| l.starts_with("    fn "))
        .count()
}

/// S20 KH-17: `HardenedKernel` reaches the inner kernel from every trait method (booleans and
/// splits through the robust chain, everything else forwarded).
#[cfg(feature = "occt")]
#[test]
fn the_hardened_kernel_forwards_every_method() {
    let inner = Forward {
        inner: OcctKernel::new(KernelConfig::default()),
        calls: AtomicUsize::new(0),
    };
    let hardened = HardenedKernel::new(inner, RetryPolicy::exact_only(), None);
    let k: &dyn BrepKernel = &hardened;
    let ctx = JobCtx::detached();
    let tol = Tolerance::DEFAULT;
    let frame = Plane::XY;
    let dir = Dir3::new_unchecked(Vec3::Z);
    let axis = Line3 {
        point: Point3::new(-5.0, 0.0, 0.0),
        dir: Dir3::new_unchecked(Vec3::Y),
    };
    let a = k.make_box(&frame, Vec3::splat(10.0)).unwrap().shape;
    let b = k
        .make_box(&frame, Vec3::splat(5.0))
        .unwrap()
        .shape
        .transformed(&Transform::from_translation(Vec3::splat(5.0)))
        .unwrap();
    let face = k
        .make_planar_polygon(
            &[Point3::ZERO, Point3::X, Point3::new(1.0, 1.0, 0.0)],
            &[],
            &tol,
        )
        .unwrap();
    let seg = CurveSeg {
        curve: Curve3::Line {
            origin: Point3::ZERO,
            dir,
        },
        t0: 0.0,
        t1: 10.0,
        tag: Tag(1),
    };
    let wire = k.make_wire(std::slice::from_ref(&seg), &tol).unwrap().shape;
    let q = TessQuality::preview();
    let calls = || hardened.inner().calls.load(Ordering::Relaxed);
    let mut n = 0;
    macro_rules! fwd {
        ($e:expr) => {{
            let before = calls();
            let _ = $e;
            assert!(calls() > before, "not forwarded: {}", stringify!($e));
            n += 1;
        }};
    }
    fwd!(k.check(&a, &CheckOpts::default()));
    fwd!(k.make_planar_faces(&[], &tol));
    fwd!(k.make_planar_polygon(&[], &[], &tol));
    fwd!(k.make_thin_face(
        &ThinProfile {
            plane: frame,
            chain: Vec::new(),
            closed: false,
            left: 1.0,
            right: 0.0,
            end_tags: [Tag(1), Tag(2)],
        },
        &tol
    ));
    fwd!(k.make_face(&Surface::Plane(frame), &FaceBounds::Natural, 1e-6));
    fwd!(k.make_wire(std::slice::from_ref(&seg), &tol));
    fwd!(k.make_box(&frame, Vec3::ONE));
    fwd!(k.make_cylinder(&frame, 1.0, 2.0, 6.0));
    fwd!(k.make_cone(&frame, 1.0, 0.5, 2.0, 6.0));
    fwd!(k.make_sphere(&frame, 1.0));
    fwd!(k.make_torus(&frame, 2.0, 1.0, 6.0));
    fwd!(k.prism(
        &face,
        &PrismExtent::Vector(Vec3::Z),
        &PrismOpts {
            taper: 0.0,
            merge_coplanar: false
        },
        &ctx
    ));
    fwd!(k.revol(&face, &axis, 1.0, &ctx));
    fwd!(k.pipe(&[face.clone(), wire.clone()], 1, &PipeOpts::default(), &ctx));
    fwd!(k.thru_sections(
        std::slice::from_ref(&wire),
        &[LoftSection::Wire(0)],
        &ThruSectionsOpts::default(),
        &ctx
    ));
    fwd!(k.mirrored(&a, &frame));
    fwd!(k.boolean(
        BoolOp::Fuse,
        std::slice::from_ref(&a),
        std::slice::from_ref(&b),
        &BoolOpts::default(),
        &ctx
    ));
    fwd!(k.split(
        std::slice::from_ref(&a),
        std::slice::from_ref(&b),
        &BoolOpts::default(),
        &ctx
    ));
    fwd!(k.section(&a, &b, &ctx));
    fwd!(k.fillet(
        &a,
        &[(EdgeId(0), FilletSpec::Constant(1.0))],
        &FilletOpts::default(),
        &ctx
    ));
    fwd!(k.chamfer(&a, &[(EdgeId(0), ChamferSpec::Equal(1.0))], &ctx));
    fwd!(k.fillet_max_radius(&a, &[EdgeId(0)], 1.0, 2, &ctx));
    fwd!(k.shell(
        &a,
        &[FaceId(0)],
        &ShellOpts {
            thickness: -1.0,
            join: OffsetJoin::Arc,
            per_face: Vec::new(),
            tol: 0.0
        },
        &ctx
    ));
    fwd!(k.offset(&a, 1.0, OffsetJoin::Arc, 0.0, &ctx));
    fwd!(k.thicken(&face, 1.0, OffsetJoin::Arc, &ctx));
    fwd!(k.draft(
        &a,
        &[FaceId(0)],
        &DraftOpts {
            pull: dir,
            angle: 0.05,
            neutral: frame
        },
        &ctx
    ));
    fwd!(k.defeature(&a, &[FaceId(0)], &ctx));
    fwd!(k.replace_face(&a, FaceId(0), &Surface::Plane(frame), &ctx));
    fwd!(k.split_faces(&a, &[FaceId(0)], &wire, &ctx));
    fwd!(k.offset_faces(&a, &[(FaceId(0), 1.0)], &ctx));
    fwd!(k.heal(&a, &HealOpts::default(), &ctx));
    fwd!(k.unify_same_domain(&a, &UnifyOpts::default()));
    fwd!(k.sew(std::slice::from_ref(&face), &SewOpts::default(), &ctx));
    fwd!(k.make_solid(&a));
    fwd!(k.make_compound(std::slice::from_ref(&a)));
    fwd!(k.plane_section(&a, &frame));
    fwd!(k.project_edges(&a, &[EdgeId(0)], &frame));
    fwd!(k.tessellate(&a, &q, &ctx));
    fwd!(k.tessellate_many(std::slice::from_ref(&a), &q, &ctx));
    fwd!(k.tessellate_edge(&a, EdgeId(0), &q));
    fwd!(k.mass_props(&a));
    fwd!(k.classify(&a, Point3::ONE, 1e-6));
    fwd!(k.distance(&a, &b));
    fwd!(k.closest_point(&a, Point3::splat(20.0)));
    fwd!(k.ray_cast(
        &a,
        &Ray3 {
            origin: Point3::new(5.0, 5.0, -5.0),
            dir
        },
        100.0
    ));
    fwd!(k.interference(&a, &b, &ctx));
    fwd!(k.eval_surface(&a, FaceId(0), Point2::ZERO));
    fwd!(k.project_point_on_face(&a, FaceId(0), Point3::ZERO));
    fwd!(k.edge_ends(&a, EdgeId(0)));
    fwd!(k.sub_geom(
        &a,
        SubShape {
            kind: SubKind::Face,
            index: 0
        }
    ));
    fwd!(k.sub_geom_all(&a));
    assert_eq!(n, trait_method_count(), "call every trait method above");
}
