//! The backend of a build without OCCT. `Occ` has no values, so the query bodies are unreachable.

use std::num::NonZeroUsize;

use amg_core::{Aabb3, Plane, Point3, Transform};
use amg_jobs::JobCtx;

use crate::KResult;
use crate::builders::{BoundsIn, LoopsIn, PlanarIn};
use crate::check::CheckReport;
use crate::error::KernelError;
use crate::extract::RawSeg;
use crate::geom::{Curve3, Surface};
use crate::index::RawTopology;
use crate::ops::{PrimitiveKind, PrismIn, RawHistory};
use crate::shape::{ShapeKind, SubKind};
use crate::tess::{TessMesh, TessParams, TessProfile};

pub(crate) fn occt_version() -> String {
    "occt-none".to_owned()
}

pub(crate) fn init_process(_threads: NonZeroUsize) {}

pub(crate) fn progress_probe(_steps: u32, _step_micros: u32, _ctx: &JobCtx) -> KResult<()> {
    Err(KernelError::BackendUnavailable)
}

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

    pub(crate) fn boolean_cut_probe(&self, _tool: &Occ) -> KResult<Occ> {
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
        _axis: amg_core::Vec3,
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
