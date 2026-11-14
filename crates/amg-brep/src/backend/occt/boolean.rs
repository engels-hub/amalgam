//! The OCCT side of booleans, split and section (BREP-04).

use amg_jobs::JobCtx;
use amg_occt_sys::cxx::UniquePtr;
use amg_occt_sys::ffi;

use super::ops::take;
use super::{Occ, progress_sink, to_kernel_error};
use crate::KResult;
use crate::boolean::{BoolCode, BoolIn, BoolOutcome, BoolStatus};
use crate::shape::{SubKind, SubShape};

fn list(shapes: &[&Occ]) -> KResult<UniquePtr<ffi::OcShapeList>> {
    let mut out = ffi::new_shape_list().map_err(|e| to_kernel_error(&e))?;
    for s in shapes {
        ffi::shape_list_push(out.pin_mut(), &s.shape).map_err(|e| to_kernel_error(&e))?;
    }
    Ok(out)
}

fn status(op: &ffi::OcOp) -> KResult<BoolStatus> {
    let raw = ffi::op_bool_status(op).map_err(|e| to_kernel_error(&e))?;
    let offending = raw
        .offending_operands
        .iter()
        .zip(&raw.offending_kinds)
        .zip(&raw.offending_indices)
        .map(|((&operand, &kind), &index)| {
            let kind = match kind {
                0 => SubKind::Face,
                1 => SubKind::Edge,
                _ => SubKind::Vertex,
            };
            (operand, SubShape { kind, index })
        })
        .collect();
    Ok(BoolStatus {
        cancelled: raw.cancelled,
        errors: raw.errors,
        warnings: raw.warnings,
        offending,
    })
}

/// The shape and history of a finished op, or its status when it failed or was cancelled.
fn outcome(op: &ffi::OcOp) -> KResult<BoolOutcome> {
    let status = status(op)?;
    if status.cancelled || !status.errors.is_empty() {
        return Ok(Err(status));
    }
    let (occ, raw) = take(op)?;
    Ok(Ok((occ, raw, status)))
}

impl Occ {
    /// `objects` op `tools` (or split), non-destructive (category N), with progress and cancel.
    pub(crate) fn boolean(
        code: BoolCode,
        objects: &[&Occ],
        tools: &[&Occ],
        opts: &BoolIn,
        ctx: &JobCtx,
    ) -> KResult<BoolOutcome> {
        let objects = list(objects)?;
        let tools = list(tools)?;
        let sink = progress_sink(ctx);
        let op = ffi::boolean_op(
            code as u8,
            &objects,
            &tools,
            &ffi::OcBoolOpts {
                fuzzy: opts.fuzzy,
                glue: opts.glue,
                check_inverted: opts.check_inverted,
                use_obb: opts.use_obb,
                parallel: opts.parallel,
            },
            &sink,
        )
        .map_err(|e| to_kernel_error(&e))?;
        outcome(&op)
    }

    /// The section edges of `self` and `other` (non-destructive, category N).
    pub(crate) fn section(
        &self,
        other: &Occ,
        parallel: bool,
        ctx: &JobCtx,
    ) -> KResult<BoolOutcome> {
        let sink = progress_sink(ctx);
        let op = ffi::section_op(&self.shape, &other.shape, parallel, &sink)
            .map_err(|e| to_kernel_error(&e))?;
        outcome(&op)
    }
}
