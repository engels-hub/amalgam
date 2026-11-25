//! The OCCT side of fillets and chamfers (BREP-06).

use amg_occt_sys::ffi;

use super::boolean::status;
use super::ops::take;
use super::{Occ, to_kernel_error};
use crate::KResult;
use crate::blend::{BlendEdge, BlendOutcome, FilletParams};
use crate::ops::RawHistory;

impl Occ {
    /// Fillets (`chamfer` false) or chamfers `edges` on a private copy (category C). Without
    /// `with_history` the history of a successful run is empty.
    pub(crate) fn blend(
        &self,
        chamfer: bool,
        edges: &[BlendEdge],
        params: &FilletParams,
        with_history: bool,
    ) -> KResult<BlendOutcome<(Occ, RawHistory)>> {
        let mut laws = Vec::new();
        let specs: Vec<ffi::OcBlendEdge> = edges
            .iter()
            .map(|e| {
                let law_start = laws.len();
                laws.extend_from_slice(&e.law);
                ffi::OcBlendEdge {
                    edge: e.edge,
                    kind: e.kind as u8,
                    a: e.a,
                    b: e.b,
                    face: e.face,
                    law_start,
                    law_end: laws.len(),
                }
            })
            .collect();
        let op = ffi::blend_op(
            &self.shape,
            self.index(),
            chamfer,
            &specs,
            &laws,
            &ffi::OcFilletParams {
                shape: params.shape,
                tol_app_3d: params.tol_app_3d,
                fleche: params.fleche,
            },
            with_history,
        )
        .map_err(|e| to_kernel_error(&e))?;
        let st = status(&op)?;
        if st.cancelled || !st.errors.is_empty() {
            let partial = ffi::op_shape(&op).map_err(|e| to_kernel_error(&e))?;
            let partial = Occ::new(partial);
            let partial = (!partial.is_null()).then_some(partial);
            return Ok(Err((st, partial.map(crate::Shape::from_occ))));
        }
        take(&op).map(Ok)
    }

    /// The angle between the outward normals of faces `f1` and `f2` at the middle of `edge`.
    pub(crate) fn edge_face_angle(&self, edge: u32, f1: u32, f2: u32) -> KResult<f64> {
        ffi::edge_face_angle(self.index(), edge, f1, f2).map_err(|e| to_kernel_error(&e))
    }
}
