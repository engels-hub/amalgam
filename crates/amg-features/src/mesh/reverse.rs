//! `mesh.reverse_normal`: flip the winding of a whole mesh body or of some face groups.

use std::collections::BTreeSet;

use amg_doc::{
    BodyId, Dependency, Diagnostic, Feature, FeatureError, FeatureOutput, MatchQuality,
    RecomputeCtx,
};
use amg_mesh::{FaceId, IndexedMesh};
use serde::{Deserialize, Serialize};

use super::base::{emit, input_mesh};
use super::{MeshEntityRef, OutputMode, resolve_group};

/// A copy of `m` with the winding of `faces` flipped (`None`: every face).
///
/// Face order, vertices and face groups are unchanged. Corner UVs and 3MF triangle properties
/// follow the swapped corners. Vertex normals are negated when every face flips and dropped
/// when only some do (a shared vertex has no single right answer; recompute them).
pub fn reverse_normals(m: &IndexedMesh, faces: Option<&[FaceId]>) -> IndexedMesh {
    let mut out = m.clone();
    let all = faces.is_none();
    let flip: Vec<usize> = match faces {
        None => (0..out.tris.len()).collect(),
        Some(f) => f
            .iter()
            .map(|f| f.0 as usize)
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect(),
    };
    for &f in &flip {
        out.tris[f].swap(1, 2);
        if let Some(uv) = &mut out.attrs.corner_uvs {
            uv[f].swap(1, 2);
        }
        if let Some(tp) = &mut out.attrs.tri_properties {
            tp[f].index.swap(1, 2);
        }
    }
    if all {
        for n in out.attrs.vertex_normals.iter_mut().flatten() {
            *n = -*n;
        }
    } else if !flip.is_empty() {
        out.attrs.vertex_normals = None;
    }
    out
}

/// Reverses the normals of a mesh body: all faces, or the faces of the listed groups.
///
/// A group that no longer exists is found again by its fingerprint (with a warning); one that
/// cannot be found fails the feature.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshReverseNormalFeature {
    /// The mesh body.
    pub body: BodyId,
    /// Groups to flip; `None`: the whole body.
    pub groups: Option<Vec<MeshEntityRef>>,
}

impl Feature for MeshReverseNormalFeature {
    fn inputs(&self) -> Vec<Dependency> {
        vec![Dependency::Body(self.body)]
    }

    fn validate(&self) -> Result<(), FeatureError> {
        match &self.groups {
            Some(g) if g.is_empty() => Err(FeatureError::InvalidValue {
                field: "groups",
                msg: "an empty list flips nothing; use None for the whole body".to_owned(),
            }),
            _ => Ok(()),
        }
    }

    fn compute(&self, ctx: &mut RecomputeCtx<'_>) -> Result<FeatureOutput, FeatureError> {
        let input = input_mesh(ctx, self.body)?;
        let mut out = FeatureOutput::default();
        let mesh = match &self.groups {
            None => reverse_normals(&input, None),
            Some(refs) => {
                let mut faces = Vec::new();
                for r in refs {
                    ctx.check_cancelled()?;
                    let res = resolve_group(&input, r)?;
                    if res.quality != MatchQuality::Exact {
                        out.warnings.push(Diagnostic {
                            message: format!(
                                "face group {:?} was found again by its shape as {:?}",
                                r.group, res.group
                            ),
                            cause: None,
                            entities: Vec::new(),
                        });
                    }
                    faces.extend(input.groups.faces_of(res.group));
                }
                reverse_normals(&input, Some(&faces))
            }
        };
        emit(
            ctx,
            self.body,
            OutputMode::Replace,
            std::sync::Arc::new(mesh),
            &mut out,
        );
        Ok(out)
    }
}

amg_doc::register_feature!(MeshReverseNormalFeature, "mesh.reverse_normal", version = 1);
