//! A fault-injecting `BrepKernel` decorator for the capture tests (S20 KH-01): deterministic, so
//! a replay in another process with the same faults reproduces the failure.
#![allow(dead_code)]

use amg_brep::geom;
use amg_brep::{
    BoolOp, BoolOpts, BrepKernel, ChamferSpec, CheckOpts, CheckReport, Classification,
    DistanceResult, DraftOpts, EdgeEnds, EdgeId, FaceBuild, FaceId, FilletOpts, FilletSpec,
    HealOpts, JobCtx, KResult, KernelError, Line3, LoftSection, MassProps, OffsetJoin, OpResult,
    PipeOpts, Plane, Point2, Point3, PrismExtent, PrismOpts, Ray3, RayHit, SewOpts, Shape,
    ShellOpts, SubShape, SubShapeGeom, SurfacePoint, TessMesh, TessQuality, ThruSectionsOpts,
    Tolerance, UnifyOpts, Vec3,
};

/// The injected "not done" failure.
pub fn not_done(op: &'static str) -> KernelError {
    KernelError::OpFailed {
        op,
        message: "injected NotDone".into(),
        offending: Vec::new(),
    }
}

/// `inner` with faults:
/// - `mirrored` and `section` always fail with an injected `OpFailed` (class `NotDone`);
/// - `make_torus` with `major == 42` panics;
/// - with `abort`, `make_sphere` aborts the process (a hard crash inside the op);
/// - with `exact_boolean_fails` (S20 KH-17), `boolean` and `split` with `fuzzy == 0` fail with
///   an injected `OpFailed`, so a robust chain has to use the fuzzy ladder.
#[derive(Debug)]
pub struct FaultKernel<K> {
    pub inner: K,
    pub abort: bool,
    pub exact_boolean_fails: bool,
}

impl<K: BrepKernel> BrepKernel for FaultKernel<K> {
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
    fn make_box(&self, frame: &Plane, size: Vec3) -> KResult<OpResult> {
        self.inner.make_box(frame, size)
    }
    fn make_cylinder(&self, frame: &Plane, r: f64, h: f64, a: f64) -> KResult<OpResult> {
        self.inner.make_cylinder(frame, r, h, a)
    }
    fn make_cone(&self, frame: &Plane, r1: f64, r2: f64, h: f64, a: f64) -> KResult<OpResult> {
        self.inner.make_cone(frame, r1, r2, h, a)
    }
    fn make_sphere(&self, frame: &Plane, radius: f64) -> KResult<OpResult> {
        if self.abort {
            std::process::abort();
        }
        self.inner.make_sphere(frame, radius)
    }
    fn make_torus(&self, frame: &Plane, major: f64, minor: f64, angle: f64) -> KResult<OpResult> {
        assert!((major - 42.0).abs() > 1e-12, "injected panic in make_torus");
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

    fn mirrored(&self, _s: &Shape, _plane: &Plane) -> KResult<OpResult> {
        Err(not_done("mirrored"))
    }
    fn boolean(
        &self,
        op: BoolOp,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        if self.exact_boolean_fails && opts.fuzzy == 0.0 {
            return Err(not_done("boolean"));
        }
        self.inner.boolean(op, objects, tools, opts, ctx)
    }
    fn split(
        &self,
        objects: &[Shape],
        tools: &[Shape],
        opts: &BoolOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        if self.exact_boolean_fails && opts.fuzzy == 0.0 {
            return Err(not_done("split"));
        }
        self.inner.split(objects, tools, opts, ctx)
    }
    fn section(&self, _a: &Shape, _b: &Shape, _ctx: &JobCtx) -> KResult<OpResult> {
        Err(not_done("section"))
    }
    fn fillet(
        &self,
        s: &Shape,
        edges: &[(EdgeId, FilletSpec)],
        opts: &FilletOpts,
        ctx: &JobCtx,
    ) -> KResult<OpResult> {
        self.inner.fillet(s, edges, opts, ctx)
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
