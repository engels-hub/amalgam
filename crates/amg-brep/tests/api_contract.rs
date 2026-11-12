//! BREP-12 Done-when 3: every op parameter type is `Clone + Debug + PartialEq + Serialize +
//! DeserializeOwned` (S20 KH-01 prerequisite), and a forwarding decorator implementing
//! `BrepKernel` compiles (object safety).
//!
//! Runs with and without the `occt` feature. Each op WP that adds a trait method adds its
//! parameter types to `op_parameter_types_are_capturable` and a forwarding arm to `Forward`.
#![allow(clippy::expect_used, clippy::unwrap_used, reason = "test code")]

use std::fmt::Debug;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use amg_brep::geom::{
    BSpline2dData, BSplineCurveData, BSplineSurfaceData, Curve2d, Curve3, CurveSeg, FaceBounds,
    GeomClass, Loop2, PlanarProfile, ProfileEdge2, Surface, Tag,
};
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, ChamferSpec, CheckOpts, CheckReport, DraftOpts, EdgeId,
    FaceBuild, FaceId, FilletOpts, FilletShape, FilletSpec, Glue, HealOpts, HelixSpec, JobCtx,
    KResult, KernelConfig, LoftSection, OcctKernel, OffsetJoin, OpResult, PipeMode, PipeOpts,
    PrismExtent, PrismOpts, SewOpts, Shape, ShapeHistory, ShapeKind, ShellOpts, SubKind, SubShape,
    TessMesh, TessQuality, ThruSectionsOpts, Tolerance, Transition, UnifyOpts, VertexId,
};
use amg_core::{Dir3, Plane, Point2, Point3, Vec2, Vec3};
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
    let kernel = as_dyn(Arc::new(stacked));
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
