//! Mesh booleans via Manifold (S07 MESH-05, ADR-003).
//!
//! `IndexedMesh` → `MeshGL64` → Manifold boolean / batch boolean → `MeshGL64` → `IndexedMesh`.
//! The backend sits behind the cargo feature `manifold` (off by default, because it needs the
//! vcpkg-built native library); without it every call returns
//! [`MeshError::BackendUnavailable`].
//!
//! - **Groups:** faces from `inputs[0]` keep their [`FaceGroupId`](crate::FaceGroupId)s and its
//!   whole group table; every group of a later operand that appears in the result is
//!   re-allocated to a fresh id with origin [`GroupOrigin::Boolean`](crate::GroupOrigin), in
//!   (operand, source id) order.
//! - **Attributes:** vertex normals / colours, corner UVs and 3MF properties are not carried
//!   through; the result has empty [`MeshAttrs`](crate::MeshAttrs).
//! - **Input checks:** an operand Manifold rejects (open, non-manifold) is repaired with
//!   [`crate::repair`] when [`BoolOpts::repair_inputs`] is set, else the call fails with
//!   `NotClosed` / `NonManifold`.
//! - **Determinism:** Manifold with oneTBB gives bit-identical output for the same input (MESH-05
//!   test `determinism_10_runs`) as long as the operands keep their internal mesh-id order. The
//!   operands are imported in parallel, which can swap those ids, so the output is put in a
//!   canonical order: Manifold's triangle runs sorted by operand, triangle order inside a run
//!   kept, vertices numbered by first use.
//! - **Cancellation:** Manifold calls are not interruptible; `ctx` is checked between stages.

#[cfg(feature = "manifold")]
mod backend;

use amg_jobs::JobCtx;

use crate::IndexedMesh;
use crate::error::{MeshError, MeshResult};

/// Boolean operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum BoolOp {
    /// A ∪ B.
    Union,
    /// A − B (for [`boolean_n`]: `inputs[0]` minus all others).
    Difference,
    /// A ∩ B.
    Intersection,
}

/// Boolean options.
#[derive(Clone, Debug, Default)]
pub struct BoolOpts {
    /// Repair operands that are not closed oriented 2-manifolds (default repair options)
    /// instead of failing.
    pub repair_inputs: bool,
}

/// What a boolean did besides producing the mesh.
#[derive(Clone, Debug, Default)]
pub struct BoolReport {
    /// Indices of the operands that had to be repaired first.
    pub repaired_inputs: Vec<usize>,
    /// Wall time, conversion included, in milliseconds.
    pub millis: u32,
}

/// `a op b`. Faces from `a` keep their group ids; groups of `b` are re-allocated.
///
/// Errors: [`MeshError::BackendUnavailable`] without the `manifold` feature; `NotClosed` /
/// `NonManifold` for a bad operand when `repair_inputs` is off; [`MeshError::Boolean`] when
/// Manifold fails; [`MeshError::Cancelled`].
pub fn boolean(
    op: BoolOp,
    a: &IndexedMesh,
    b: &IndexedMesh,
    opts: &BoolOpts,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, BoolReport)> {
    boolean_n(op, &[a, b], opts, ctx)
}

/// `inputs[0]` op all others (Manifold batch boolean); group ids of `inputs[0]` are preserved,
/// groups of `inputs[k]`, `k ≥ 1`, are re-allocated with `GroupOrigin::Boolean { operand: k }`.
///
/// Needs 1 to 256 inputs (`InvalidOption` otherwise). Errors as for [`boolean`].
pub fn boolean_n(
    op: BoolOp,
    inputs: &[&IndexedMesh],
    opts: &BoolOpts,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, BoolReport)> {
    if inputs.is_empty() || inputs.len() > 256 {
        return Err(MeshError::InvalidOption {
            name: "inputs",
            reason: format!("expected 1 to 256 operands, got {}", inputs.len()),
        });
    }
    for m in inputs {
        m.validate()?;
    }
    ctx.checkpoint()?;
    run(op, inputs, opts, ctx)
}

#[cfg(feature = "manifold")]
fn run(
    op: BoolOp,
    inputs: &[&IndexedMesh],
    opts: &BoolOpts,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, BoolReport)> {
    backend::run(op, inputs, opts, ctx)
}

#[cfg(not(feature = "manifold"))]
fn run(
    _op: BoolOp,
    _inputs: &[&IndexedMesh],
    _opts: &BoolOpts,
    _ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, BoolReport)> {
    Err(MeshError::BackendUnavailable)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn operand_count_is_checked() {
        let ctx = JobCtx::detached();
        let r = boolean_n(BoolOp::Union, &[], &BoolOpts::default(), &ctx);
        assert!(matches!(r, Err(MeshError::InvalidOption { .. })));
    }

    #[cfg(not(feature = "manifold"))]
    #[test]
    fn without_feature_backend_is_unavailable() {
        let ctx = JobCtx::detached();
        let p = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let c = IndexedMesh::new(
            p.iter().map(|&q| amg_core::Point3::from(q)).collect(),
            vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
        )
        .unwrap();
        let r = boolean(BoolOp::Union, &c, &c, &BoolOpts::default(), &ctx);
        assert!(matches!(r, Err(MeshError::BackendUnavailable)));
    }
}
