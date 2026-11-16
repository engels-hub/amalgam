//! Shared plumbing of mesh features: input lookup, output mode, result emission.

use std::sync::Arc;

use amg_doc::{
    BodyChange, BodyChangeKind, BodyGeom, BodyId, FeatureError, FeatureOutput, RecomputeCtx,
};
use amg_mesh::IndexedMesh;
use serde::{Deserialize, Serialize};

use super::MeshFeatureError;

/// What a mesh feature does with its result.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum OutputMode {
    /// The input body gets the new geometry (default).
    #[default]
    Replace,
    /// The result is a new body (slot 0 of the feature); the input stays.
    NewBody,
}

/// The mesh of `body` just before the feature.
pub(super) fn input_mesh(
    ctx: &RecomputeCtx<'_>,
    body: BodyId,
) -> Result<Arc<IndexedMesh>, FeatureError> {
    match ctx.body(body)? {
        BodyGeom::Mesh(m) => Ok(Arc::clone(m)),
        _ => Err(MeshFeatureError::NotAMesh.into()),
    }
}

/// Adds the body change for `mesh` according to `mode`.
pub(super) fn emit(
    ctx: &RecomputeCtx<'_>,
    body: BodyId,
    mode: OutputMode,
    mesh: Arc<IndexedMesh>,
    out: &mut FeatureOutput,
) {
    let geom = BodyGeom::Mesh(mesh);
    out.bodies.push(match mode {
        OutputMode::Replace => BodyChange {
            id: body,
            change: BodyChangeKind::Modified { geom, naming: None },
        },
        OutputMode::NewBody => BodyChange {
            id: ctx.new_body_id(0),
            change: BodyChangeKind::Created { geom, naming: None },
        },
    });
}

/// A warning diagnostic without a cause or entities.
pub(super) fn warning(message: String) -> amg_doc::Diagnostic {
    amg_doc::Diagnostic {
        message,
        cause: None,
        entities: Vec::new(),
    }
}

/// The faces of the referenced groups of `mesh` (sorted, unique). A group found only by its
/// fingerprint adds a warning to `out`; one that cannot be found fails the feature.
pub(super) fn group_faces(
    ctx: &RecomputeCtx<'_>,
    mesh: &IndexedMesh,
    refs: &[super::MeshEntityRef],
    out: &mut FeatureOutput,
) -> Result<Vec<amg_mesh::FaceId>, FeatureError> {
    let mut faces = std::collections::BTreeSet::new();
    for r in refs {
        ctx.check_cancelled()?;
        let res = super::resolve_group(mesh, r)?;
        if res.quality != amg_doc::MatchQuality::Exact {
            out.warnings.push(warning(format!(
                "face group {:?} was found again by its shape as {:?}",
                r.group, res.group
            )));
        }
        faces.extend(mesh.groups.faces_of(res.group));
    }
    Ok(faces.into_iter().collect())
}

/// The job of the compute, or a detached one (never cancelled).
pub(super) fn with_job<T>(ctx: &RecomputeCtx<'_>, f: impl FnOnce(&amg_jobs::JobCtx) -> T) -> T {
    match ctx.job() {
        Some(job) => f(job),
        None => f(&amg_jobs::JobCtx::detached()),
    }
}

/// `field` must be finite and inside `(lo, hi)` (`lo_incl` / `hi_incl` make an end inclusive).
pub(super) fn check_range(
    field: &'static str,
    v: f64,
    lo: f64,
    lo_incl: bool,
    hi: f64,
    hi_incl: bool,
) -> Result<f64, FeatureError> {
    let lo_ok = if lo_incl { v >= lo } else { v > lo };
    let hi_ok = if hi_incl { v <= hi } else { v < hi };
    if v.is_finite() && lo_ok && hi_ok {
        Ok(v)
    } else {
        let (l, h) = (
            if lo_incl { '[' } else { '(' },
            if hi_incl { ']' } else { ')' },
        );
        Err(FeatureError::InvalidValue {
            field,
            msg: format!("must be in {l}{lo}, {hi}{h}, got {v}"),
        })
    }
}

/// A finite length `> 0`.
pub(super) fn positive_length(
    ctx: &RecomputeCtx<'_>,
    e: &amg_doc::Expr,
    field: &'static str,
) -> Result<f64, FeatureError> {
    check_range(field, ctx.eval_length(e)?, 0.0, false, f64::INFINITY, false)
}

/// The mesh, or the input allocation itself when the content is identical (no change event).
pub(super) fn same_or_new(input: &Arc<IndexedMesh>, m: IndexedMesh) -> Arc<IndexedMesh> {
    if m.content_hash() == input.content_hash() && m == **input {
        Arc::clone(input)
    } else {
        Arc::new(m)
    }
}
