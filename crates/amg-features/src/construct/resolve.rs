//! Reference resolution: [`RefInput`] → [`ResolvedRef`] inside a recompute.
//!
//! CON-01 resolves what the document already holds without a geometry kernel:
//!
//! - origin entities of the context component, in component space (no occurrence transform;
//!   see the S06 note under S05 Interface changes);
//! - construction planes, axes and points (`EntityRef::Construction`).
//!
//! B-rep faces, edges and vertices, sketch entities and mesh face groups resolve with CON-03;
//! until then they are a feature error naming the reference kind.

use amg_core::Transform;
use amg_doc::{ConstructionGeom, EntityRef, FeatureError, RecomputeCtx};

use super::{RefGeom, RefInput, ResolvedRef, math};

/// Resolves one construction input against the state just before the computing feature.
pub fn resolve_input(ctx: &RecomputeCtx<'_>, r: &RefInput) -> Result<ResolvedRef, FeatureError> {
    match r {
        RefInput::Origin(e) => Ok(ResolvedRef::new(math::origin_geom(
            *e,
            &Transform::IDENTITY,
        ))),
        RefInput::Ref(EntityRef::Construction(c)) => {
            let geom = match ctx.construction(*c)? {
                ConstructionGeom::Plane(p) => RefGeom::Plane(*p),
                ConstructionGeom::Axis(l) => RefGeom::Line(*l),
                ConstructionGeom::Point(p) => RefGeom::Point(*p),
            };
            Ok(ResolvedRef::new(geom))
        }
        RefInput::Ref(other) => Err(FeatureError::InvalidValue {
            field: "inputs",
            msg: format!(
                "{} references cannot be resolved yet (CON-03)",
                kind_name(other)
            ),
        }),
    }
}

fn kind_name(r: &EntityRef) -> &'static str {
    match r {
        EntityRef::Body(_) => "body",
        EntityRef::Topo(_) => "face, edge or vertex",
        EntityRef::SketchCurve { .. } => "sketch curve",
        EntityRef::SketchPoint { .. } => "sketch point",
        EntityRef::Profile { .. } => "profile",
        EntityRef::Construction(_) => "construction",
        EntityRef::InContext { .. } => "in-context",
    }
}
