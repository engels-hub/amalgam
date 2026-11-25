//! The OCCT side of draft, defeature, replace face and split faces (BREP-16).

use amg_jobs::JobCtx;
use amg_occt_sys::ffi;

use super::boolean::status;
use super::build::oc_frame;
use super::ops::take;
use super::{Occ, progress_sink, to_kernel_error};
use crate::KResult;
use crate::local::DraftIn;
use crate::offset::OffsetOutcome;
use crate::ops::RawHistory;

/// The result of a finished op, or its status when it failed or was cancelled.
fn outcome(op: &ffi::OcOp) -> KResult<OffsetOutcome<(Occ, RawHistory)>> {
    let st = status(op)?;
    if st.cancelled || !st.errors.is_empty() {
        return Ok(Err((st, None)));
    }
    take(op).map(Ok)
}

impl Occ {
    /// Drafts faces on a private copy (category C).
    pub(crate) fn draft(
        &self,
        input: &DraftIn,
        ctx: &JobCtx,
    ) -> KResult<OffsetOutcome<(Occ, RawHistory)>> {
        let sink = progress_sink(ctx);
        let p = input.pull;
        let op = ffi::draft_op(
            &self.shape,
            self.index(),
            &input.faces,
            &ffi::OcDraftParams {
                pull: ffi::V3 {
                    x: p.x,
                    y: p.y,
                    z: p.z,
                },
                angle: input.angle,
                neutral: oc_frame(&input.neutral),
            },
            &sink,
        )
        .map_err(|e| to_kernel_error(&e))?;
        outcome(&op)
    }

    /// Removes faces on a private copy (category C).
    pub(crate) fn defeature(
        &self,
        faces: &[u32],
        parallel: bool,
        ctx: &JobCtx,
    ) -> KResult<OffsetOutcome<(Occ, RawHistory)>> {
        let sink = progress_sink(ctx);
        let op = ffi::defeature_op(&self.shape, self.index(), faces, parallel, &sink)
            .map_err(|e| to_kernel_error(&e))?;
        outcome(&op)
    }

    /// How far the surface of the face `tool` lies beyond face `face` along its outward normals
    /// (NaN when no normal line meets it).
    pub(crate) fn face_reach(&self, face: u32, tool: &Occ) -> KResult<f64> {
        ffi::face_reach(self.index(), face, &tool.shape).map_err(|e| to_kernel_error(&e))
    }

    /// `self` split by the face `tool`, keeping the material side (category C).
    pub(crate) fn trim_face(
        &self,
        face: u32,
        tool: &Occ,
        ctx: &JobCtx,
    ) -> KResult<OffsetOutcome<(Occ, RawHistory)>> {
        let sink = progress_sink(ctx);
        let op = ffi::trim_face_op(&self.shape, self.index(), face, &tool.shape, &sink)
            .map_err(|e| to_kernel_error(&e))?;
        outcome(&op)
    }

    /// `faces` split by the curves of `tool` on them (category C).
    pub(crate) fn split_faces(
        &self,
        faces: &[u32],
        tool: &Occ,
        ctx: &JobCtx,
    ) -> KResult<OffsetOutcome<(Occ, RawHistory)>> {
        let sink = progress_sink(ctx);
        let op = ffi::split_faces_op(&self.shape, self.index(), faces, &tool.shape, &sink)
            .map_err(|e| to_kernel_error(&e))?;
        outcome(&op)
    }
}
