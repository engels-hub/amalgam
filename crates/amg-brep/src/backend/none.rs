//! The backend of a build without OCCT. `Occ` has no values, so the query bodies are unreachable.

use std::num::NonZeroUsize;

use amg_core::{Aabb3, Plane, Point3, Transform};
use amg_jobs::JobCtx;

use crate::KResult;
use crate::blend::{BlendEdge, BlendOutcome, FilletParams};
use crate::boolean::{BoolCode, BoolIn, BoolOutcome};
use crate::builders::{BoundsIn, LoopsIn, PlanarIn};
use crate::check::CheckReport;
use crate::error::KernelError;
use crate::extract::RawSeg;
use crate::geom::{Curve3, Surface};
use crate::heal::{HealIn, HealOutcome, SewIn, UnifyIn};
use crate::index::RawTopology;
use crate::offset::{OffsetIn, OffsetOutcome};
use crate::ops::{PrimitiveKind, PrismIn, RawHistory};
use crate::query::{RawDistance, RawHit, RawMass, RawSurfacePoint};
use crate::shape::{ShapeKind, SubKind, SubShape};
use crate::tess::{TessMesh, TessParams, TessProfile};
use crate::{Classification, EdgeEnds, SubShapeGeom};
use amg_core::{Point2, Vec3};

pub(crate) fn occt_version() -> String {
    "occt-none".to_owned()
}

pub(crate) fn init_process(_threads: NonZeroUsize) {}

pub(crate) fn progress_probe(_steps: u32, _step_micros: u32, _ctx: &JobCtx) -> KResult<()> {
    Err(KernelError::BackendUnavailable)
}

/// The shape and history of a finished sweep, or its status (BREP-15).
pub(crate) type SweepOutcome = Result<(Occ, RawHistory), crate::boolean::BoolStatus>;

/// The builders' backend check: no OCCT in this build.
pub(crate) fn require() -> KResult<()> {
    Err(KernelError::BackendUnavailable)
}

pub(crate) enum Occ {}

impl Occ {
    pub(crate) fn planar_faces(_inputs: &[PlanarIn], _tol: f64) -> KResult<Vec<(Occ, Vec<u32>)>> {
        Err(KernelError::BackendUnavailable)
    }

    pub(crate) fn face_on_surface(
        _surface: &Surface,
        _bounds: &BoundsIn,
        _tol: f64,
    ) -> KResult<(Occ, Vec<u32>)> {
        Err(KernelError::BackendUnavailable)
    }

    pub(crate) fn wire(_edges: &LoopsIn, _closed: bool, _tol: f64) -> KResult<(Occ, Vec<u32>)> {
        Err(KernelError::BackendUnavailable)
    }

    pub(crate) fn primitive(_kind: PrimitiveKind, _frame: &Plane, _params: &[f64]) -> KResult<Occ> {
        Err(KernelError::BackendUnavailable)
    }

    pub(crate) fn read_bin(_data: &[u8]) -> KResult<Occ> {
        Err(KernelError::BackendUnavailable)
    }

    pub(crate) fn read_text(_data: &[u8]) -> KResult<Occ> {
        Err(KernelError::BackendUnavailable)
    }

    pub(crate) fn kind(&self) -> ShapeKind {
        match *self {}
    }

    pub(crate) fn is_null(&self) -> bool {
        match *self {}
    }

    pub(crate) fn is_same(&self, _other: &Occ) -> bool {
        match *self {}
    }

    pub(crate) fn topology(&self) -> RawTopology {
        match *self {}
    }

    pub(crate) fn solids(&self) -> Vec<Occ> {
        match *self {}
    }

    pub(crate) fn sub_shape(&self, _kind: SubKind, _index: u32) -> KResult<Occ> {
        match *self {}
    }

    pub(crate) fn bbox(&self) -> Aabb3 {
        match *self {}
    }

    pub(crate) fn surface(&self, _face: u32) -> Surface {
        match *self {}
    }

    pub(crate) fn face_reversed(&self, _face: u32) -> bool {
        match *self {}
    }

    pub(crate) fn face_uv_bounds(&self, _face: u32) -> [f64; 4] {
        match *self {}
    }

    pub(crate) fn face_area(&self, _face: u32) -> f64 {
        match *self {}
    }

    pub(crate) fn curve(&self, _edge: u32) -> Option<(Curve3, f64, f64)> {
        match *self {}
    }

    pub(crate) fn is_seam(&self, _edge: u32, _face: u32) -> bool {
        match *self {}
    }

    pub(crate) fn vertex_point(&self, _vertex: u32) -> Point3 {
        match *self {}
    }

    pub(crate) fn tolerance(&self, _kind: SubKind, _index: u32) -> f64 {
        match *self {}
    }

    pub(crate) fn max_tolerance(&self) -> f64 {
        match *self {}
    }

    pub(crate) fn check(&self) -> CheckReport {
        match *self {}
    }

    pub(crate) fn estimated_bytes(&self) -> u64 {
        match *self {}
    }

    pub(crate) fn volume(&self) -> KResult<f64> {
        match *self {}
    }

    pub(crate) fn volume_adaptive(&self, _eps: f64) -> KResult<f64> {
        match *self {}
    }

    pub(crate) fn moved(&self, _transform: &Transform) -> KResult<Occ> {
        match *self {}
    }

    pub(crate) fn write_bin(&self) -> KResult<Vec<u8>> {
        match *self {}
    }

    pub(crate) fn write_text(&self) -> KResult<Vec<u8>> {
        match *self {}
    }

    pub(crate) fn tessellate(
        &self,
        _params: TessParams,
        _ctx: &JobCtx,
    ) -> KResult<(TessMesh, TessProfile)> {
        match *self {}
    }

    pub(crate) fn tessellate_edge(
        &self,
        _edge: u32,
        _linear: f64,
        _angular: f64,
    ) -> KResult<Vec<Point3>> {
        match *self {}
    }

    pub(crate) fn boolean(
        _code: BoolCode,
        _objects: &[&Occ],
        _tools: &[&Occ],
        _opts: &BoolIn,
        _ctx: &JobCtx,
    ) -> KResult<BoolOutcome> {
        Err(KernelError::BackendUnavailable)
    }

    pub(crate) fn section(
        &self,
        _other: &Occ,
        _parallel: bool,
        _ctx: &JobCtx,
    ) -> KResult<BoolOutcome> {
        match *self {}
    }

    pub(crate) fn plane_section(&self, _plane: &Plane, _tol: f64) -> KResult<Vec<RawSeg>> {
        match *self {}
    }

    pub(crate) fn prism(&self, _p: &PrismIn) -> KResult<(Occ, RawHistory)> {
        match *self {}
    }

    pub(crate) fn revol(
        &self,
        _origin: Point3,
        _axis: Vec3,
        _angle: f64,
    ) -> KResult<(Occ, RawHistory)> {
        match *self {}
    }

    pub(crate) fn gtransformed(&self, _t: &Transform) -> KResult<(Occ, RawHistory)> {
        match *self {}
    }

    pub(crate) fn curve_recognised(
        &self,
        _edge: u32,
        _tol: f64,
    ) -> KResult<Option<(Curve3, f64, f64)>> {
        match *self {}
    }
}

impl Occ {
    pub(crate) fn sub_geoms(&self, _kind: SubKind, _indices: &[u32]) -> KResult<Vec<SubShapeGeom>> {
        match *self {}
    }

    pub(crate) fn sub_index_map(&self, _part: &Occ) -> KResult<[Vec<Option<u32>>; 3]> {
        match *self {}
    }

    pub(crate) fn unify(&self) -> KResult<(Occ, RawHistory)> {
        match *self {}
    }

    pub(crate) fn heal(
        &self,
        _p: &HealIn,
        _ctx: &JobCtx,
    ) -> KResult<HealOutcome<(Occ, RawHistory, u32)>> {
        match *self {}
    }

    pub(crate) fn unify_same_domain(&self, _p: &UnifyIn) -> KResult<(Occ, RawHistory)> {
        match *self {}
    }

    pub(crate) fn sew(
        _shapes: &[&Occ],
        _p: &SewIn,
        _ctx: &JobCtx,
    ) -> KResult<HealOutcome<(Occ, RawHistory)>> {
        Err(KernelError::BackendUnavailable)
    }

    pub(crate) fn pipe(
        _operands: &[&Occ],
        _p: &crate::sweep::PipeIn,
        _ctx: &JobCtx,
    ) -> KResult<SweepOutcome> {
        Err(KernelError::BackendUnavailable)
    }

    pub(crate) fn thru_sections(
        _wires: &[&Occ],
        _p: &crate::sweep::LoftIn,
        _ctx: &JobCtx,
    ) -> KResult<SweepOutcome> {
        Err(KernelError::BackendUnavailable)
    }

    pub(crate) fn make_solid(&self) -> KResult<HealOutcome<(Occ, RawHistory)>> {
        match *self {}
    }

    pub(crate) fn compound(_parts: &[&Occ]) -> KResult<Occ> {
        Err(KernelError::BackendUnavailable)
    }

    pub(crate) fn analyze(&self) -> KResult<Vec<(Option<SubShape>, String)>> {
        match *self {}
    }

    pub(crate) fn blend(
        &self,
        _chamfer: bool,
        _edges: &[BlendEdge],
        _params: &FilletParams,
        _with_history: bool,
    ) -> KResult<BlendOutcome<(Occ, RawHistory)>> {
        match *self {}
    }

    pub(crate) fn offset(
        &self,
        _input: &OffsetIn,
        _ctx: &JobCtx,
    ) -> KResult<OffsetOutcome<(Occ, RawHistory)>> {
        match *self {}
    }

    pub(crate) fn edge_face_angle(&self, _edge: u32, _f1: u32, _f2: u32) -> KResult<f64> {
        match *self {}
    }

    pub(crate) fn draft(
        &self,
        _input: &crate::local::DraftIn,
        _ctx: &JobCtx,
    ) -> KResult<OffsetOutcome<(Occ, RawHistory)>> {
        match *self {}
    }

    pub(crate) fn defeature(
        &self,
        _faces: &[u32],
        _parallel: bool,
        _ctx: &JobCtx,
    ) -> KResult<OffsetOutcome<(Occ, RawHistory)>> {
        match *self {}
    }

    pub(crate) fn face_reach(&self, _face: u32, _tool: &Occ) -> KResult<f64> {
        match *self {}
    }

    pub(crate) fn trim_face(
        &self,
        _face: u32,
        _tool: &Occ,
        _ctx: &JobCtx,
    ) -> KResult<OffsetOutcome<(Occ, RawHistory)>> {
        match *self {}
    }

    pub(crate) fn split_faces(
        &self,
        _faces: &[u32],
        _tool: &Occ,
        _ctx: &JobCtx,
    ) -> KResult<OffsetOutcome<(Occ, RawHistory)>> {
        match *self {}
    }

    pub(crate) fn mass_props(&self, _eps: f64) -> KResult<RawMass> {
        match *self {}
    }

    pub(crate) fn has_solids(&self) -> bool {
        match *self {}
    }

    pub(crate) fn classify(&self, _p: Point3, _tol: f64) -> KResult<Classification> {
        match *self {}
    }

    pub(crate) fn distance(&self, _other: &Occ) -> KResult<RawDistance> {
        match *self {}
    }

    pub(crate) fn closest_point(&self, _p: Point3) -> KResult<RawDistance> {
        match *self {}
    }

    pub(crate) fn ray_cast(
        &self,
        _origin: Point3,
        _dir: Vec3,
        _max_t: f64,
        _tol: f64,
    ) -> KResult<Option<RawHit>> {
        match *self {}
    }

    pub(crate) fn eval_face(&self, _face: u32, _uv: Point2, _tol: f64) -> KResult<RawSurfacePoint> {
        match *self {}
    }

    pub(crate) fn project_on_face(
        &self,
        _face: u32,
        _p: Point3,
        _tol: f64,
    ) -> KResult<RawDistance> {
        match *self {}
    }

    pub(crate) fn edge_ends(&self, _edge: u32) -> KResult<Option<EdgeEnds>> {
        match *self {}
    }
}
