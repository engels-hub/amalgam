//! The OCCT side of shell, offset and thicken (BREP-07).

use amg_jobs::JobCtx;
use amg_occt_sys::ffi;

use super::boolean::status;
use super::ops::take;
use super::{Occ, progress_sink, to_kernel_error};
use crate::KResult;
use crate::offset::{OffsetIn, OffsetOutcome};
use crate::ops::RawHistory;

impl Occ {
    /// Shells, offsets or thickens on a private copy (category C), with progress and cancel.
    pub(crate) fn offset(
        &self,
        input: &OffsetIn,
        ctx: &JobCtx,
    ) -> KResult<OffsetOutcome<(Occ, RawHistory)>> {
        let sink = progress_sink(ctx);
        let op = ffi::offset_op(
            &self.shape,
            self.index(),
            &ffi::OcOffsetParams {
                mode: input.mode as u8,
                offset: input.offset,
                tol: input.tol,
                join: input.join,
            },
            &input.remove,
            &input.faces,
            &input.offsets,
            &sink,
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
}
