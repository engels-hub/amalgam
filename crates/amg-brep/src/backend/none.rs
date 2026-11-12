//! The backend of a build without OCCT. `Occ` has no values, so the query bodies are unreachable.

use std::num::NonZeroUsize;

use amg_core::{Aabb3, Point3, Transform};
use amg_jobs::JobCtx;

use crate::KResult;
use crate::check::CheckReport;
use crate::error::KernelError;
use crate::geom::{Curve3, Surface};
use crate::index::RawTopology;
use crate::shape::{ShapeKind, SubKind};
use crate::tess::{TessMesh, TessParams, TessProfile};

pub(crate) fn occt_version() -> String {
    "occt-none".to_owned()
}

pub(crate) fn init_process(_threads: NonZeroUsize) {}

pub(crate) fn progress_probe(_steps: u32, _step_micros: u32, _ctx: &JobCtx) -> KResult<()> {
    Err(KernelError::BackendUnavailable)
}

pub(crate) enum Occ {}

impl Occ {
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
}
