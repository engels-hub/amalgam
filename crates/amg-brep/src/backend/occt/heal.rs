//! The OCCT side of heal, unify same domain, sew, make solid, make compound and the
//! `BOPAlgo_ArgumentAnalyzer` pass of `check` (BREP-10).

use amg_jobs::JobCtx;
use amg_occt_sys::ffi;

use super::boolean::{list, status};
use super::ops::take;
use super::{Occ, progress_sink, to_kernel_error};
use crate::KResult;
use crate::heal::{HealIn, HealOutcome, SewIn, UnifyIn};
use crate::ops::RawHistory;
use crate::shape::{SubKind, SubShape};

/// The result of a finished op, or its status with the shape it holds (if any) when it failed.
fn outcome(op: &ffi::OcOp) -> KResult<HealOutcome<(Occ, RawHistory)>> {
    let st = status(op)?;
    if st.cancelled || !st.errors.is_empty() {
        let partial = Occ::new(ffi::op_shape(op).map_err(|e| to_kernel_error(&e))?);
        let partial = (!partial.is_null()).then(|| crate::Shape::from_occ(partial));
        return Ok(Err((st, partial)));
    }
    take(op).map(Ok)
}

impl Occ {
    /// `ShapeFix_Shape` + `ShapeFix_Wireframe` on a private copy (category C); also the number of
    /// fixes.
    pub(crate) fn heal(
        &self,
        p: &HealIn,
        ctx: &JobCtx,
    ) -> KResult<HealOutcome<(Occ, RawHistory, u32)>> {
        let sink = progress_sink(ctx);
        let op = ffi::heal_op(
            &self.shape,
            &ffi::OcHealParams {
                tol: p.tol,
                max_tol: p.max_tol,
                fix_small_edges: p.fix_small_edges,
                fix_orientation: p.fix_orientation,
            },
            &sink,
        )
        .map_err(|e| to_kernel_error(&e))?;
        let fixes = ffi::op_fix_count(&op).map_err(|e| to_kernel_error(&e))?;
        Ok(outcome(&op)?.map(|(occ, raw)| (occ, raw, fixes)))
    }

    /// `ShapeUpgrade_UnifySameDomain` with options on a private copy (category C).
    pub(crate) fn unify_same_domain(&self, p: &UnifyIn) -> KResult<(Occ, RawHistory)> {
        let op = ffi::unify_same_domain_op(
            &self.shape,
            &ffi::OcUnifyParams {
                faces: p.faces,
                edges: p.edges,
                concat_bsplines: p.concat_bsplines,
                lin_tol: p.lin_tol,
                ang_tol: p.ang_tol,
            },
        )
        .map_err(|e| to_kernel_error(&e))?;
        take(&op)
    }

    /// `shapes` sewn on one private copy (category C), with progress and cancel.
    pub(crate) fn sew(
        shapes: &[&Occ],
        p: &SewIn,
        ctx: &JobCtx,
    ) -> KResult<HealOutcome<(Occ, RawHistory)>> {
        let shapes = list(shapes)?;
        let sink = progress_sink(ctx);
        let op = ffi::sew_op(
            &shapes,
            &ffi::OcSewParams {
                tol: p.tol,
                non_manifold: p.non_manifold,
                make_solid: p.make_solid,
            },
            &sink,
        )
        .map_err(|e| to_kernel_error(&e))?;
        outcome(&op)
    }

    /// A solid per closed shell, on a private copy (category C).
    pub(crate) fn make_solid(&self) -> KResult<HealOutcome<(Occ, RawHistory)>> {
        let op = ffi::make_solid_op(&self.shape).map_err(|e| to_kernel_error(&e))?;
        outcome(&op)
    }

    /// A compound sharing the parts' TShapes (category N).
    pub(crate) fn compound(parts: &[&Occ]) -> KResult<Occ> {
        let parts = list(parts)?;
        ffi::make_compound_of(&parts)
            .map(Occ::new)
            .map_err(|e| to_kernel_error(&e))
    }

    /// The `BOPAlgo_ArgumentAnalyzer` problems of the shape (on a private copy, category C).
    pub(crate) fn analyze(&self) -> KResult<Vec<(Option<SubShape>, String)>> {
        let raw = ffi::analyze_shape(&self.shape, self.index()).map_err(|e| to_kernel_error(&e))?;
        Ok(raw
            .into_iter()
            .map(|p| {
                let kind = match p.kind {
                    0 => Some(SubKind::Face),
                    1 => Some(SubKind::Edge),
                    2 => Some(SubKind::Vertex),
                    _ => None,
                };
                let sub = kind.map(|kind| SubShape {
                    kind,
                    index: p.index,
                });
                (sub, p.status)
            })
            .collect())
    }
}
