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
