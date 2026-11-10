//! The backend of a build without OCCT. `Occ` has no values, so the query bodies are unreachable.

use amg_core::{Aabb3, Point3, Transform};

use crate::KResult;
use crate::error::KernelError;
use crate::geom::{Curve3, Surface};
use crate::index::RawTopology;
use crate::shape::{ShapeKind, SubKind};

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

    pub(crate) fn estimated_bytes(&self) -> u64 {
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
}
