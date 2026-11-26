//! Reference bases of section analyses (INS-11, S05 §3): `SectionBase::Ref` → the world plane,
//! through CON-03.

use amg_core::Plane;
use amg_doc::{Design, EntityRef, Evaluated, MatchQuality};

use crate::construct::resolve::{InspectResolveError, resolve_in};
use crate::construct::{RefGeom, RefInput};

/// A reference base that resolved.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResolvedBase {
    /// The world plane; a face's normal points out of the material.
    pub plane: Plane,
    /// Only the fingerprint matched (DOC-06): the entity may not be the one that was picked.
    pub uncertain: bool,
}

/// Why a reference gives no base plane.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum BaseError {
    /// The reference does not resolve (deleted, no geometry, unsupported entity).
    #[error(transparent)]
    Resolve(#[from] InspectResolveError),
    /// It resolves to something that is not a plane, such as a curved face.
    #[error("the referenced entity is not a construction plane or a planar face")]
    NotPlanar,
}

/// Whether `e` is construction geometry, possibly seen through occurrences. Such a base resolves
/// without a kernel query, so it is cheap enough for the UI thread; every other reference should
/// be resolved in a job.
pub fn is_construction(e: &EntityRef) -> bool {
    match e {
        EntityRef::Construction(_) => true,
        EntityRef::InContext { target, .. } => is_construction(target),
        _ => false,
    }
}

/// The world plane of reference base `e`: a construction plane, a planar B-rep face or a planar
/// mesh face group.
///
/// # Errors
/// [`BaseError`] when the reference does not resolve or is not planar.
pub fn resolve_base(
    eval: &Evaluated,
    design: &Design,
    e: &EntityRef,
) -> Result<ResolvedBase, BaseError> {
    let r = resolve_in(eval, design, &RefInput::Ref(e.clone()))?;
    match r.geom {
        RefGeom::Plane(plane) => Ok(ResolvedBase {
            plane,
            uncertain: r.quality == Some(MatchQuality::FingerprintOnly),
        }),
        _ => Err(BaseError::NotPlanar),
    }
}
