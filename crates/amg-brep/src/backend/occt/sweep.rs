//! The OCCT side of pipe and thru_sections (BREP-15).

use amg_core::Vec3;
use amg_jobs::JobCtx;
use amg_occt_sys::ffi;

use super::boolean::{list, status};
use super::ops::take;
use super::{Occ, progress_sink, to_kernel_error};
use crate::KResult;
use crate::boolean::BoolStatus;
use crate::ops::RawHistory;
use crate::sweep::{LoftIn, PipeIn};

/// The shape and history of a finished sweep, or its status when it failed or was cancelled.
pub(crate) type SweepOutcome = Result<(Occ, RawHistory), BoolStatus>;

fn v3(p: Vec3) -> ffi::V3 {
    ffi::V3 {
        x: p.x,
        y: p.y,
        z: p.z,
    }
}

fn outcome(op: &ffi::OcOp) -> KResult<SweepOutcome> {
    let st = status(op)?;
    if st.cancelled || !st.errors.is_empty() {
        return Ok(Err(st));
    }
    take(op).map(Ok)
}

impl Occ {
    /// Pipe-shell of `operands` on private copies (category C), with progress and cancel.
    pub(crate) fn pipe(operands: &[&Occ], p: &PipeIn, ctx: &JobCtx) -> KResult<SweepOutcome> {
        let shapes = list(operands)?;
        let sink = progress_sink(ctx);
        let op = ffi::pipe_op(
            &shapes,
            &ffi::OcPipeParams {
                profiles: p.profiles,
                mode: p.mode,
                binormal: v3(p.binormal),
                aux: p.aux,
                contact: p.contact,
                transition: p.transition,
                solid: p.solid,
            },
            &p.twist,
            &p.scale,
            &sink,
        )
        .map_err(|e| to_kernel_error(&e))?;
        outcome(&op)
    }

    /// ThruSections through the sections of `p` on private copies of `wires` (category C).
    pub(crate) fn thru_sections(wires: &[&Occ], p: &LoftIn, ctx: &JobCtx) -> KResult<SweepOutcome> {
        let shapes = list(wires)?;
        let sink = progress_sink(ctx);
        let sections: Vec<ffi::OcLoftSection> = p
            .sections
            .iter()
            .map(|&(wire, point)| ffi::OcLoftSection {
                wire,
                point: v3(point),
            })
            .collect();
        let op = ffi::thru_sections_op(
            &shapes,
            &sections,
            &ffi::OcLoftParams {
                solid: p.solid,
                ruled: p.ruled,
                check_compatibility: p.check_compatibility,
                smoothing: p.smoothing,
                max_degree: p.max_degree,
            },
            &sink,
        )
        .map_err(|e| to_kernel_error(&e))?;
        outcome(&op)
    }
}
