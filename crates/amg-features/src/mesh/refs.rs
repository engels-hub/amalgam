//! Mesh sub-entity references: face group id first, geometric fingerprint as fallback.

use amg_core::Vec3;
use amg_doc::MatchQuality;
use amg_mesh::{FaceGroupId, GroupFingerprint, IndexedMesh};
use serde::{Deserialize, Serialize};

use super::MeshFeatureError;

/// Mesh sub-entity reference (the S01 `EntityRef` payload for mesh bodies).
///
/// `group` is the persistent name: ids are never reused within a body lineage and survive
/// repair, remesh and boolean operations. `fingerprint` is the geometric fallback for the
/// case that an upstream edit removed the id.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshEntityRef {
    /// The face group.
    pub group: FaceGroupId,
    /// What the group looked like when it was picked.
    pub fingerprint: GroupFingerprint,
}

impl MeshEntityRef {
    /// A reference to `group` of `mesh`, with its current fingerprint. `None` if the group is
    /// unknown or has no faces.
    pub fn new(mesh: &IndexedMesh, group: FaceGroupId) -> Option<MeshEntityRef> {
        Some(MeshEntityRef {
            group,
            fingerprint: mesh.groups.fingerprint(mesh, group)?,
        })
    }
}

/// How a reference was resolved (`Exact` or `FingerprintOnly`).
pub type GroupMatch = MatchQuality;

/// A resolved reference.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ResolvedGroup {
    /// The group in the mesh now.
    pub group: FaceGroupId,
    /// `Exact`: the id exists. `FingerprintOnly`: found by geometry (callers report a warning).
    pub quality: GroupMatch,
}

/// Relative tolerance of the fingerprint match.
const AREA_REL: f64 = 0.25;
/// Centroid distance as a fraction of the reference group's bounding-box diagonal.
const CENTROID_REL: f64 = 0.15;
const MIN_NORMAL_DOT: f64 = 0.9;
/// The best candidate's score must be at most this fraction of the second best's.
const UNIQUE_MARGIN: f64 = 0.5;

/// Resolves `r` against `mesh`: the group id if it still has faces, otherwise the group whose
/// fingerprint matches `r.fingerprint` (area, centroid, mean normal, primitive kind) clearly
/// better than every other group.
///
/// # Errors
/// [`MeshFeatureError::UnresolvedGroup`] if the id is gone and no group matches uniquely.
pub fn resolve_group(
    mesh: &IndexedMesh,
    r: &MeshEntityRef,
) -> Result<ResolvedGroup, MeshFeatureError> {
    if mesh.groups.faces_of(r.group).next().is_some() {
        return Ok(ResolvedGroup {
            group: r.group,
            quality: MatchQuality::Exact,
        });
    }
    let want = &r.fingerprint;
    let size = want.aabb.size().length().max(f64::EPSILON);
    let mut best: Option<(f64, FaceGroupId)> = None;
    let mut second = f64::INFINITY;
    for g in mesh.groups.ids() {
        let Some(fp) = mesh.groups.fingerprint(mesh, g) else {
            continue;
        };
        let Some(s) = score(want, &fp, size) else {
            continue;
        };
        match best {
            Some((b, _)) if s >= b => second = second.min(s),
            Some((b, _)) => {
                second = second.min(b);
                best = Some((s, g));
            }
            None => best = Some((s, g)),
        }
    }
    match best {
        Some((s, g)) if second.is_infinite() || s <= second * UNIQUE_MARGIN => Ok(ResolvedGroup {
            group: g,
            quality: MatchQuality::FingerprintOnly,
        }),
        _ => Err(MeshFeatureError::UnresolvedGroup(r.group)),
    }
}

/// Lower is better; `None` if `got` is not a plausible match for `want`.
fn score(want: &GroupFingerprint, got: &GroupFingerprint, size: f64) -> Option<f64> {
    if want.primitive.is_some() && got.primitive.is_some() && want.primitive != got.primitive {
        return None;
    }
    let area = (got.area - want.area).abs() / want.area.max(f64::EPSILON);
    let centroid = (got.centroid - want.centroid).length() / size;
    if area > AREA_REL || centroid > CENTROID_REL {
        return None;
    }
    let dot = want.mean_normal.dot(got.mean_normal);
    let both = want.mean_normal != Vec3::ZERO && got.mean_normal != Vec3::ZERO;
    if both && dot < MIN_NORMAL_DOT {
        return None;
    }
    let n = if both { 1.0 - dot } else { 0.0 };
    Some(area / AREA_REL + centroid / CENTROID_REL + n)
}
