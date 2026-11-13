//! The OCCT side of primitives, prism, revol and affine transforms (BREP-14).

use amg_core::{Plane, Point3, Transform, Vec3};
use amg_occt_sys::ffi;

use super::build::oc_frame;
use super::{Occ, mat34, to_kernel_error};
use crate::KResult;
use crate::ops::{PrimitiveKind, PrismIn, RawHistory};

fn v3(p: Vec3) -> ffi::V3 {
    ffi::V3 {
        x: p.x,
        y: p.y,
        z: p.z,
    }
}

fn take(op: &ffi::OcOp) -> KResult<(Occ, RawHistory)> {
    let shape = ffi::op_shape(op).map_err(|e| to_kernel_error(&e))?;
    let raw = ffi::op_history(op).map_err(|e| to_kernel_error(&e))?;
    let history = RawHistory {
        in_kinds: raw.in_kinds,
        in_indices: raw.in_indices,
        relations: raw.relations,
        out_starts: raw.out_starts,
        out_kinds: raw.out_kinds,
        out_indices: raw.out_indices,
        role_faces: raw.role_faces,
        roles: raw.roles,
    };
    Ok((Occ::new(shape), history))
}

impl Occ {
    /// A primitive solid on `frame` (parameters validated by `crate::ops`).
    pub(crate) fn primitive(kind: PrimitiveKind, frame: &Plane, params: &[f64]) -> KResult<Occ> {
        ffi::make_primitive(kind as u8, &oc_frame(frame), params)
            .map(Occ::new)
            .map_err(|e| to_kernel_error(&e))
    }

    /// `self` swept as a prism.
    pub(crate) fn prism(&self, p: &PrismIn) -> KResult<(Occ, RawHistory)> {
        let op = ffi::prism(&self.shape, &v3(p.vec), p.mode, p.taper, p.merge_coplanar)
            .map_err(|e| to_kernel_error(&e))?;
        take(&op)
    }

    /// `self` revolved by `angle` about the axis through `origin` along `axis`.
    pub(crate) fn revol(
        &self,
        origin: Point3,
        axis: Vec3,
        angle: f64,
    ) -> KResult<(Occ, RawHistory)> {
        let op = ffi::revol(&self.shape, &v3(origin), &v3(axis), angle)
            .map_err(|e| to_kernel_error(&e))?;
        take(&op)
    }

    /// `self` mapped by an affine transform into new geometry.
    pub(crate) fn gtransformed(&self, t: &Transform) -> KResult<(Occ, RawHistory)> {
        let op =
            ffi::shape_gtransformed(&self.shape, &mat34(t)).map_err(|e| to_kernel_error(&e))?;
        take(&op)
    }
}
