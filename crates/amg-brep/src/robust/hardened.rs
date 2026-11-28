//! `HardenedKernel`: the opt-in decorator that routes booleans and fillets through the robust
//! chains (S02 §10, S20 design §3–§4).

use std::sync::Arc;

use amg_core::{Line3, Plane, Point2, Point3, Ray3, Tolerance, Vec3};
use amg_jobs::JobCtx;

use super::{
    RetryPolicy, RobustCtx, RobustError, RobustOutput, StrategyId, robust_boolean, robust_split,
};
use crate::capture::{FailureClass, FailureRecorder};
use crate::check::{CheckOpts, CheckReport};
use crate::error::{KResult, KernelError};
use crate::geom;
use crate::history::{FaceBuild, KernelWarning, OpResult, ThinFace};
use crate::kernel::BrepKernel;
use crate::opts::{
    BoolOp, BoolOpts, ChamferSpec, DraftOpts, FilletOpts, FilletSpec, HealOpts, LoftSection,
    OffsetJoin, PipeOpts, PrismExtent, PrismOpts, SewOpts, ShellOpts, TessQuality,
    ThruSectionsOpts, UnifyOpts,
};
use crate::query::{
    Classification, DistanceResult, EdgeEnds, MassProps, RayHit, SubShapeGeom, SurfacePoint,
};
use crate::shape::{EdgeId, FaceId, Shape, SubShape};
use crate::tess::TessMesh;

/// [`BrepKernel`] decorator (S02 §10): routes `boolean` and `split` through
/// [`robust_boolean`] / [`robust_split`] and `fillet` through [`super::fillet::fillet`] under
/// `policy`; every other method forwards to `inner` unchanged. Opt-in: features keep taking
/// `&dyn BrepKernel`, and the recompute context decides which kernel they get.
///
/// A non-direct acceptance adds [`KernelWarning::Strategy`] to `OpResult::warnings` (S20 request
/// R17), one per report warning. A boolean chain that fails returns the kernel's own error of the
/// direct attempt (argument errors, `BackendUnavailable` and `Cancelled` unchanged), or
/// `OpFailed { op, message }` for a timeout or a result the gate refused. A failed fillet returns
/// [`super::fillet::FilletFailure::to_kernel_error`]: argument errors unchanged, otherwise
/// `LocalOpFailed { op: "fillet" }` naming the failing edges, with the class first in the
/// message. Callers that need the whole [`super::RobustReport`] or the
/// [`super::fillet::FilletDiagnosis`] call [`robust_boolean`] / [`super::fillet::fillet`]
/// directly.
#[derive(Debug)]
pub struct HardenedKernel<K: BrepKernel> {
    inner: K,
    policy: RetryPolicy,
    recorder: Option<Arc<FailureRecorder>>,
    tol: Tolerance,
}

const fn _assert_send_sync<T: Send + Sync>() {}
const _: () = _assert_send_sync::<HardenedKernel<crate::OcctKernel>>();

impl<K: BrepKernel> HardenedKernel<K> {
    /// Wraps `inner`; final failures are dumped through `recorder` when given.
    pub fn new(inner: K, policy: RetryPolicy, recorder: Option<Arc<FailureRecorder>>) -> Self {
        HardenedKernel {
            inner,
            policy,
            recorder,
            tol: Tolerance::DEFAULT,
        }
    }

    /// Sets the model tolerance used for healing (default [`Tolerance::DEFAULT`]).
    #[must_use]
    pub fn with_tolerance(mut self, tol: Tolerance) -> Self {
        self.tol = tol;
        self
    }

    /// The wrapped kernel.
    pub fn inner(&self) -> &K {
        &self.inner
    }

    /// The policy.
    pub fn policy(&self) -> &RetryPolicy {
        &self.policy
    }

    fn cx<'a>(&'a self, ctx: &'a JobCtx) -> RobustCtx<'a> {
        RobustCtx {
            job: ctx,
            recorder: self.recorder.as_deref(),
            tol: self.tol,
        }
    }

    fn hardened(
        &self,
        op: &'static str,
        r: Result<RobustOutput, RobustError>,
    ) -> KResult<OpResult> {
        match r {
            Ok(out) => {
                let mut result = out.result;
                if let Some(id) = out.report.accepted.filter(|id| *id != StrategyId::Direct) {
                    for detail in out.report.warnings {
                        result.warnings.push(KernelWarning::Strategy {
                            name: id.name(),
                            detail,
                        });
                    }
                }
                Ok(result)
            }
            Err(e) => Err(kernel_error(op, e)),
        }
    }
}

/// The kernel error a failed chain is reported as.
fn kernel_error(op: &'static str, e: RobustError) -> KernelError {
    if e.class == FailureClass::Cancelled {
        return KernelError::Cancelled;
    }
    match e.error {
        Some(err) if e.class != FailureClass::Timeout => *err,
        _ => KernelError::OpFailed {
            op,
            message: e.message,
            offending: Vec::new(),
        },
    }
}

impl<K: BrepKernel> BrepKernel for HardenedKernel<K> {
    fn check(&self, s: &Shape, opts: &CheckOpts) -> CheckReport {
        self.inner.check(s, opts)
    }

    fn make_planar_faces(
        &self,
        profiles: &[geom::PlanarProfile],
        tol: &Tolerance,
    ) -> KResult<Vec<FaceBuild>> {
        self.inner.make_planar_faces(profiles, tol)
    }

    fn make_planar_polygon(
        &self,
        outer: &[Point3],
        holes: &[Vec<Point3>],
        tol: &Tolerance,
    ) -> KResult<Shape> {
        self.inner.make_planar_polygon(outer, holes, tol)
    }

    fn make_face(
        &self,
        surface: &geom::Surface,
        bounds: &geom::FaceBounds,
        tol: f64,
    ) -> KResult<FaceBuild> {
        self.inner.make_face(surface, bounds, tol)
    }

    fn make_wire(&self, segs: &[geom::CurveSeg], tol: &Tolerance) -> KResult<OpResult> {
        self.inner.make_wire(segs, tol)
    }

    fn make_thin_face(&self, p: &geom::ThinProfile, tol: &Tolerance) -> KResult<ThinFace> {
        self.inner.make_thin_face(p, tol)
    }

    fn make_box(&self, frame: &Plane, size: Vec3) -> KResult<OpResult> {
        self.inner.make_box(frame, size)
    }

    fn make_cylinder(
        &self,
        frame: &Plane,
        radius: f64,
        height: f64,
        angle: f64,
    ) -> KResult<OpResult> {
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
        self.inner.make_cone(frame, r1, r2, height, angle)
    }

    fn make_sphere(&self, frame: &Plane, radius: f64) -> KResult<OpResult> {
        self.inner.make_sphere(frame, radius)
    }

    fn make_torus(&self, frame: &Plane, major: f64, minor: f64, angle: f64) -> KResult<OpResult> {
        self.inner.make_torus(frame, major, minor, angle)
    }

    fn prism(
        &self,
        profile: &Shape,
        extent: &PrismExtent,
        opts: &PrismOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.inner.prism(profile, extent, opts, ctx)
    }

    fn revol(&self, profile: &Shape, axis: &Line3, angle: f64, ctx: &JobCtx) -> KResult<OpResult> {
        self.inner.revol(profile, axis, angle, ctx)
    }

    fn pipe(
        &self,
        operands: &[Shape],
        profiles: usize,
        opts: &PipeOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.inner.pipe(operands, profiles, opts, ctx)
    }

    fn thru_sections(
        &self,
        wires: &[Shape],
        sections: &[LoftSection],
        opts: &ThruSectionsOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.inner.thru_sections(wires, sections, opts, ctx)
    }

    fn mirrored(&self, s: &Shape, plane: &Plane) -> KResult<OpResult> {
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
        self.hardened(
            "boolean",
            robust_boolean(
                &self.inner,
                op,
                objects,
                tools,
                opts,
                &self.policy,
                &self.cx(ctx),
            ),
        )
    }

    fn split(
        &self,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.hardened(
            "split",
            robust_split(
                &self.inner,
                objects,
                tools,
                opts,
                &self.policy,
                &self.cx(ctx),
            ),
        )
    }

    fn section(&self, a: &Shape, b: &Shape, ctx: &JobCtx) -> KResult<OpResult> {
        self.inner.section(a, b, ctx)
    }

    fn fillet(
        &self,
        s: &Shape,
        edges: &[(EdgeId, FilletSpec)],
        opts: &FilletOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        match super::fillet::fillet(&self.inner, s, edges, opts, &self.policy, &self.cx(ctx)) {
            Ok(out) => self.hardened("fillet", Ok(out)),
            Err(f) => Err(f.to_kernel_error()),
        }
    }

    fn chamfer(
        &self,
        s: &Shape,
        edges: &[(EdgeId, ChamferSpec)],
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
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
        self.inner.fillet_max_radius(s, edges, hi, iterations, ctx)
    }

    fn shell(
        &self,
        s: &Shape,
        remove: &[FaceId],
        opts: &ShellOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
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
        self.inner.offset(s, distance, join, tol, ctx)
    }

    fn thicken(
        &self,
        s: &Shape,
        thickness: f64,
        join: OffsetJoin,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.inner.thicken(s, thickness, join, ctx)
    }

    fn draft(
        &self,
        s: &Shape,
        faces: &[FaceId],
        opts: &DraftOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.inner.draft(s, faces, opts, ctx)
    }

    fn defeature(&self, s: &Shape, faces: &[FaceId], ctx: &JobCtx) -> KResult<OpResult> {
        self.inner.defeature(s, faces, ctx)
    }

    fn replace_face(
        &self,
        s: &Shape,
        face: FaceId,
        with: &geom::Surface,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.inner.replace_face(s, face, with, ctx)
    }

    fn split_faces(
        &self,
        s: &Shape,
        faces: &[FaceId],
        tool: &Shape,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.inner.split_faces(s, faces, tool, ctx)
    }

    fn offset_faces(&self, s: &Shape, faces: &[(FaceId, f64)], ctx: &JobCtx) -> KResult<OpResult> {
        self.inner.offset_faces(s, faces, ctx)
    }

    fn heal(&self, s: &Shape, opts: &HealOpts, ctx: &JobCtx) -> KResult<OpResult> {
        self.inner.heal(s, opts, ctx)
    }

    fn unify_same_domain(&self, s: &Shape, opts: &UnifyOpts) -> KResult<OpResult> {
        self.inner.unify_same_domain(s, opts)
    }

    fn sew(&self, faces: &[Shape], opts: &SewOpts, ctx: &JobCtx) -> KResult<OpResult> {
        self.inner.sew(faces, opts, ctx)
    }

    fn make_solid(&self, shell: &Shape) -> KResult<OpResult> {
        self.inner.make_solid(shell)
    }

    fn make_compound(&self, parts: &[Shape]) -> KResult<Shape> {
        self.inner.make_compound(parts)
    }

    fn plane_section(&self, s: &Shape, plane: &Plane) -> KResult<Vec<geom::CurveSeg>> {
        self.inner.plane_section(s, plane)
    }

    fn project_edges(
        &self,
        s: &Shape,
        edges: &[EdgeId],
        plane: &Plane,
    ) -> KResult<Vec<geom::CurveSeg>> {
        self.inner.project_edges(s, edges, plane)
    }

    fn tessellate(&self, s: &Shape, q: &TessQuality, ctx: &JobCtx) -> KResult<TessMesh> {
        self.inner.tessellate(s, q, ctx)
    }

    fn tessellate_many(
        &self,
        shapes: &[Shape],
        q: &TessQuality,
        ctx: &JobCtx,
    ) -> Vec<KResult<TessMesh>> {
        self.inner.tessellate_many(shapes, q, ctx)
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
        self.inner.interference(a, b, ctx)
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
