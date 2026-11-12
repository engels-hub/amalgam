//! The OCCT side of geometry extraction (BREP-17): plane sections and recognised edge curves.

use amg_core::Plane;
use amg_occt_sys::ffi;

use super::build::oc_frame;
use super::{Occ, curve_from, to_kernel_error};
use crate::KResult;
use crate::extract::RawSeg;
use crate::geom::Curve3;

impl Occ {
    /// The section edges of `self` cut by `plane`, each with the face it lies on.
    pub(crate) fn plane_section(&self, plane: &Plane, tol: f64) -> KResult<Vec<RawSeg>> {
        let raw = ffi::plane_section(&self.shape, self.index(), &oc_frame(plane), tol)
            .map_err(|e| to_kernel_error(&e))?;
        Ok(raw
            .curves
            .into_iter()
            .zip(raw.faces)
            .filter_map(|(curve, face)| {
                let (curve, t0, t1) = curve_from(curve)?;
                Some(RawSeg {
                    curve,
                    t0,
                    t1,
                    face: (face != u32::MAX).then_some(face),
                })
            })
            .collect())
    }

    /// Edge `edge`'s curve as a line, circle, ellipse or B-spline (approximated within `tol`
    /// where OCCT has no exact form). `None` for a degenerated edge.
    pub(crate) fn curve_recognised(
        &self,
        edge: u32,
        tol: f64,
    ) -> KResult<Option<(Curve3, f64, f64)>> {
        let raw =
            ffi::edge_curve_recognised(self.index(), edge, tol).map_err(|e| to_kernel_error(&e))?;
        Ok(curve_from(raw))
    }
}
