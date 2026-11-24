//! `RecordingKernel`: the `BrepKernel` decorator that records failures.

use std::panic::{AssertUnwindSafe, catch_unwind, resume_unwind};
use std::sync::Arc;
use std::time::{Duration, Instant};

use amg_core::{Line3, Plane, Point2, Point3, Ray3, Tolerance, Vec3};
use amg_jobs::JobCtx;

use crate::capture::info::{FailureClass, FailureInfo};
use crate::capture::op::{EdgeRecord, InputRef, KernelOp, refs};
use crate::capture::recorder::{CaptureMode, Captured, FailureRecorder};
use crate::check::{CheckOpts, CheckReport};
use crate::error::{KResult, KernelError};
use crate::geom;
use crate::history::{FaceBuild, OpResult};
use crate::kernel::BrepKernel;
use crate::opts::{BoolOp, BoolOpts, PrismExtent, PrismOpts, TessQuality};
use crate::query::{
    Classification, DistanceResult, EdgeEnds, MassProps, RayHit, SubShapeGeom, SurfacePoint,
};
use crate::shape::{EdgeId, FaceId, Shape, SubShape};
use crate::tess::TessMesh;

/// [`BrepKernel`] decorator: forwards every call to `inner` and records failed
/// shape-producing calls through its [`FailureRecorder`] (see [`KernelOp`] for which ops are
/// recorded). `Cancelled` and `BackendUnavailable` are not failures and are never recorded; a
/// panic is recorded as `Panic` and then resumed.
///
/// In `OnFailure` mode a successful call costs a mode check, a clock read and a
/// `catch_unwind` frame: no allocation and no I/O (the inputs are borrowed for the call, so
/// nothing has to be kept). Recording errors never change the op's result; they are logged.
#[derive(Debug)]
pub struct RecordingKernel<K: BrepKernel> {
    inner: K,
    recorder: Arc<FailureRecorder>,
    tolerance: Option<Tolerance>,
}

const fn _assert_send_sync<T: Send + Sync>() {}
const _: () = _assert_send_sync::<RecordingKernel<crate::OcctKernel>>();

fn should_record(e: &KernelError) -> bool {
    !matches!(e, KernelError::Cancelled | KernelError::BackendUnavailable)
}

fn panic_message(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|s| (*s).to_owned())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "panic with a non-string payload".into())
}

impl<K: BrepKernel> RecordingKernel<K> {
    /// Wraps `inner`.
    pub fn new(inner: K, recorder: Arc<FailureRecorder>) -> Self {
        RecordingKernel {
            inner,
            recorder,
            tolerance: None,
        }
    }

    /// Records `tol` (the inner kernel's tolerance) in every manifest.
    #[must_use]
    pub fn with_tolerance(mut self, tol: Tolerance) -> Self {
        self.tolerance = Some(tol);
        self
    }

    /// The wrapped kernel.
    pub fn inner(&self) -> &K {
        &self.inner
    }

    /// The recorder.
    pub fn recorder(&self) -> &Arc<FailureRecorder> {
        &self.recorder
    }

    fn write(&self, captured: &Captured, info: FailureInfo, duration: Duration) {
        let mode = self.recorder.config().mode;
        let manifest = captured.manifest(info, &[], mode, duration, self.tolerance);
        match self.recorder.store(captured, manifest) {
            Ok(id) => tracing::info!("kernel failure captured: {id}"),
            Err(e) => tracing::warn!("kernel capture failed: {e}"),
        }
    }

    /// Runs `call` under the capture policy. `op` and `inputs` are only evaluated when a dump
    /// is written (`OnFailure`) or before the call (`WriteAhead`). Public for the overhead
    /// bench; ops use it through the trait.
    #[doc(hidden)]
    pub fn guarded<'s, T>(
        &self,
        op: impl FnOnce() -> KernelOp,
        inputs: impl FnOnce() -> Vec<&'s Shape>,
        call: impl FnOnce() -> KResult<T>,
    ) -> KResult<T> {
        match self.recorder.config().mode {
            CaptureMode::Off => call(),
            CaptureMode::OnFailure => {
                let start = Instant::now();
                let result = catch_unwind(AssertUnwindSafe(call));
                if matches!(result, Ok(Ok(_))) {
                    return result.unwrap_or_else(|p| resume_unwind(p));
                }
                let failure = match &result {
                    Ok(Ok(_)) => None,
                    Ok(Err(e)) if !should_record(e) => None,
                    Ok(Err(e)) => Some(Err(e)),
                    Err(p) => Some(Ok(panic_message(p.as_ref()))),
                };
                if let Some(failure) = failure {
                    let inputs = inputs();
                    match Captured::new(op(), &inputs) {
                        Ok(captured) => {
                            let info = match failure {
                                Err(e) => {
                                    FailureInfo::from_kernel_error(e, inputs.first().copied())
                                }
                                Ok(msg) => FailureInfo::new(FailureClass::Panic, msg),
                            };
                            self.write(&captured, info, start.elapsed());
                        }
                        Err(e) => tracing::warn!("kernel capture failed: {e}"),
                    }
                }
                result.unwrap_or_else(|p| resume_unwind(p))
            }
            CaptureMode::WriteAhead => {
                let inputs = inputs();
                let pending = Captured::new(op(), &inputs).and_then(|c| {
                    let m = c.manifest(
                        FailureInfo::new(
                            FailureClass::Crash,
                            "the process ended inside the op (write-ahead dump)",
                        ),
                        &[],
                        CaptureMode::WriteAhead,
                        Duration::ZERO,
                        self.tolerance,
                    );
                    self.recorder.begin(c, m)
                });
                let pending = match pending {
                    Ok(p) => Some(p),
                    Err(e) => {
                        tracing::warn!("kernel capture: write-ahead failed: {e}");
                        None
                    }
                };
                let start = Instant::now();
                let result = catch_unwind(AssertUnwindSafe(call));
                if let Some(pending) = pending {
                    let info = match &result {
                        Ok(Err(e)) if should_record(e) => {
                            Some(FailureInfo::from_kernel_error(e, inputs.first().copied()))
                        }
                        Err(p) => Some(FailureInfo::new(
                            FailureClass::Panic,
                            panic_message(p.as_ref()),
                        )),
                        _ => None,
                    };
                    if let Some(info) = info {
                        self.write(&pending.captured, info, start.elapsed());
                    }
                    pending.finish();
                }
                result.unwrap_or_else(|p| resume_unwind(p))
            }
        }
    }
}

fn one(i: u32) -> InputRef {
    InputRef(i)
}

impl<K: BrepKernel> BrepKernel for RecordingKernel<K> {
    fn check(&self, s: &Shape, opts: &CheckOpts) -> CheckReport {
        self.inner.check(s, opts)
    }

    fn make_planar_faces(
        &self,
        profiles: &[geom::PlanarProfile],
        tol: &Tolerance,
    ) -> KResult<Vec<FaceBuild>> {
        self.guarded(
            || KernelOp::MakePlanarFaces {
                profiles: profiles.to_vec(),
                tol: *tol,
            },
            Vec::new,
            || self.inner.make_planar_faces(profiles, tol),
        )
    }

    fn make_planar_polygon(
        &self,
        outer: &[Point3],
        holes: &[Vec<Point3>],
        tol: &Tolerance,
    ) -> KResult<Shape> {
        self.guarded(
            || KernelOp::MakePlanarPolygon {
                outer: outer.to_vec(),
                holes: holes.to_vec(),
                tol: *tol,
            },
            Vec::new,
            || self.inner.make_planar_polygon(outer, holes, tol),
        )
    }

    fn make_face(
        &self,
        surface: &geom::Surface,
        bounds: &geom::FaceBounds,
        tol: f64,
    ) -> KResult<FaceBuild> {
        self.guarded(
            || KernelOp::MakeFace {
                surface: surface.clone(),
                bounds: bounds.clone(),
                tol,
            },
            Vec::new,
            || self.inner.make_face(surface, bounds, tol),
        )
    }

    fn make_wire(&self, segs: &[geom::CurveSeg], tol: &Tolerance) -> KResult<OpResult> {
        self.guarded(
            || KernelOp::MakeWire {
                segs: segs.to_vec(),
                tol: *tol,
            },
            Vec::new,
            || self.inner.make_wire(segs, tol),
        )
    }

    fn make_box(&self, frame: &Plane, size: Vec3) -> KResult<OpResult> {
        self.guarded(
            || KernelOp::MakeBox {
                frame: *frame,
                size,
            },
            Vec::new,
            || self.inner.make_box(frame, size),
        )
    }

    fn make_cylinder(
        &self,
        frame: &Plane,
        radius: f64,
        height: f64,
        angle: f64,
    ) -> KResult<OpResult> {
        self.guarded(
            || KernelOp::MakeCylinder {
                frame: *frame,
                radius,
                height,
                angle,
            },
            Vec::new,
            || self.inner.make_cylinder(frame, radius, height, angle),
        )
    }

    fn make_cone(
        &self,
        frame: &Plane,
        r1: f64,
        r2: f64,
        height: f64,
        angle: f64,
    ) -> KResult<OpResult> {
        self.guarded(
            || KernelOp::MakeCone {
                frame: *frame,
                r1,
                r2,
                height,
                angle,
            },
            Vec::new,
            || self.inner.make_cone(frame, r1, r2, height, angle),
        )
    }

    fn make_sphere(&self, frame: &Plane, radius: f64) -> KResult<OpResult> {
        self.guarded(
            || KernelOp::MakeSphere {
                frame: *frame,
                radius,
            },
            Vec::new,
            || self.inner.make_sphere(frame, radius),
        )
    }

    fn make_torus(&self, frame: &Plane, major: f64, minor: f64, angle: f64) -> KResult<OpResult> {
        self.guarded(
            || KernelOp::MakeTorus {
                frame: *frame,
                major,
                minor,
                angle,
            },
            Vec::new,
            || self.inner.make_torus(frame, major, minor, angle),
        )
    }

    fn prism(
        &self,
        profile: &Shape,
        extent: &PrismExtent,
        opts: &PrismOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.guarded(
            || KernelOp::Prism {
                profile: one(0),
                extent: extent.clone(),
                opts: opts.clone(),
            },
            || vec![profile],
            || self.inner.prism(profile, extent, opts, ctx),
        )
    }

    fn revol(&self, profile: &Shape, axis: &Line3, angle: f64, ctx: &JobCtx) -> KResult<OpResult> {
        self.guarded(
            || KernelOp::Revol {
                profile: one(0),
                axis: *axis,
                angle,
            },
            || vec![profile],
            || self.inner.revol(profile, axis, angle, ctx),
        )
    }

    fn mirrored(&self, s: &Shape, plane: &Plane) -> KResult<OpResult> {
        self.guarded(
            || KernelOp::Mirrored {
                shape: one(0),
                plane: *plane,
            },
            || vec![s],
            || self.inner.mirrored(s, plane),
        )
    }

    fn boolean(
        &self,
        op: BoolOp,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        let n = objects.len();
        self.guarded(
            || KernelOp::Boolean {
                kind: op,
                objects: refs(0..n),
                tools: refs(n..n + tools.len()),
                opts: opts.clone(),
            },
            || objects.iter().chain(tools).collect(),
            || self.inner.boolean(op, objects, tools, opts, ctx),
        )
    }

    fn split(
        &self,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        let n = objects.len();
        self.guarded(
            || KernelOp::Split {
                objects: refs(0..n),
                tools: refs(n..n + tools.len()),
                opts: opts.clone(),
            },
            || objects.iter().chain(tools).collect(),
            || self.inner.split(objects, tools, opts, ctx),
        )
    }

    fn section(&self, a: &Shape, b: &Shape, ctx: &JobCtx) -> KResult<OpResult> {
        self.guarded(
            || KernelOp::Section {
                a: one(0),
                b: one(1),
            },
            || vec![a, b],
            || self.inner.section(a, b, ctx),
        )
    }

    fn plane_section(&self, s: &Shape, plane: &Plane) -> KResult<Vec<geom::CurveSeg>> {
        self.guarded(
            || KernelOp::PlaneSection {
                shape: one(0),
                plane: *plane,
            },
            || vec![s],
            || self.inner.plane_section(s, plane),
        )
    }

    fn project_edges(
        &self,
        s: &Shape,
        edges: &[EdgeId],
        plane: &Plane,
    ) -> KResult<Vec<geom::CurveSeg>> {
        self.guarded(
            || KernelOp::ProjectEdges {
                shape: one(0),
                edges: edges.iter().map(|e| EdgeRecord::of(s, *e)).collect(),
                plane: *plane,
            },
            || vec![s],
            || self.inner.project_edges(s, edges, plane),
        )
    }

    fn tessellate(&self, s: &Shape, q: &TessQuality, ctx: &JobCtx) -> KResult<TessMesh> {
        self.guarded(
            || KernelOp::Tessellate {
                shape: one(0),
                quality: *q,
            },
            || vec![s],
            || self.inner.tessellate(s, q, ctx),
        )
    }

    fn tessellate_many(
        &self,
        shapes: &[Shape],
        q: &TessQuality,
        ctx: &JobCtx,
    ) -> Vec<KResult<TessMesh>> {
        let results = self.inner.tessellate_many(shapes, q, ctx);
        if self.recorder.config().mode != CaptureMode::Off {
            for (s, r) in shapes.iter().zip(&results) {
                if let Err(e) = r
                    && should_record(e)
                {
                    let op = KernelOp::Tessellate {
                        shape: one(0),
                        quality: *q,
                    };
                    match Captured::new(op, &[s]) {
                        Ok(c) => self.write(
                            &c,
                            FailureInfo::from_kernel_error(e, Some(s)),
                            Duration::ZERO,
                        ),
                        Err(e) => tracing::warn!("kernel capture failed: {e}"),
                    }
                }
            }
        }
        results
    }

    fn tessellate_edge(&self, s: &Shape, e: EdgeId, q: &TessQuality) -> KResult<Vec<Point3>> {
        self.inner.tessellate_edge(s, e, q)
    }

    fn mass_props(&self, s: &Shape) -> KResult<MassProps> {
        self.inner.mass_props(s)
    }

    fn classify(&self, s: &Shape, p: Point3, tol: f64) -> KResult<Classification> {
        self.inner.classify(s, p, tol)
    }

    fn distance(&self, a: &Shape, b: &Shape) -> KResult<DistanceResult> {
        self.inner.distance(a, b)
    }

    fn closest_point(&self, s: &Shape, p: Point3) -> KResult<DistanceResult> {
        self.inner.closest_point(s, p)
    }

    fn ray_cast(&self, s: &Shape, ray: &Ray3, max_t: f64) -> KResult<Option<RayHit>> {
        self.inner.ray_cast(s, ray, max_t)
    }

    fn interference(&self, a: &Shape, b: &Shape, ctx: &JobCtx) -> KResult<Option<OpResult>> {
        self.guarded(
            || KernelOp::Interference {
                a: one(0),
                b: one(1),
            },
            || vec![a, b],
            || self.inner.interference(a, b, ctx),
        )
    }

    fn eval_surface(&self, s: &Shape, f: FaceId, uv: Point2) -> KResult<SurfacePoint> {
        self.inner.eval_surface(s, f, uv)
    }

    fn project_point_on_face(&self, s: &Shape, f: FaceId, p: Point3) -> KResult<(Point2, Point3)> {
        self.inner.project_point_on_face(s, f, p)
    }

    fn edge_ends(&self, s: &Shape, e: EdgeId) -> KResult<EdgeEnds> {
        self.inner.edge_ends(s, e)
    }

    fn sub_geom(&self, s: &Shape, sub: SubShape) -> KResult<SubShapeGeom> {
        self.inner.sub_geom(s, sub)
    }

    fn sub_geom_all(&self, s: &Shape) -> KResult<[Vec<SubShapeGeom>; 3]> {
        self.inner.sub_geom_all(s)
    }
}
