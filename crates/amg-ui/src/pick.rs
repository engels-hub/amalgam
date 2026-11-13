//! Display-side identities of pickable things (spec §7, Interfaces §A `amg_ui::pick`).
//!
//! UI-20 defines the types the scene adapter's [`PickMap`](crate::scene::PickMap) translates
//! to. UI-03 adds the `PickService` trait (screen-space requests with radius and filter) on
//! top; until then [`testing::CpuPickService`](crate::testing) takes world rays.

use amg_doc::{BodyId, ConstructionId, OccurrenceId, SketchId, TopoKind};

/// Display-side identity of a pickable thing in the current `Evaluated`.
///
/// `instance` indexes the scene adapter's occurrence-path table
/// ([`SceneAdapter::occurrence_path`](crate::scene::SceneAdapter::occurrence_path)): the same
/// body seen through two occurrences is two targets (S06 request). A mesh body's
/// `(TopoKind::Face, i)` is triangle `i` (an `amg_mesh::FaceId`).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum PickTarget {
    /// A body, or one of its faces, edges or vertices (`sub == None`: the whole body).
    Body {
        /// The body.
        body: BodyId,
        /// Index into the adapter's occurrence-path table.
        instance: u32,
        /// Sub-entity.
        sub: Option<(TopoKind, u32)>,
    },
    /// A whole sketch (sketch entities and profiles come with the RND-07 overlays, UI-09).
    Sketch(SketchId),
    /// Construction geometry.
    Construction(ConstructionId),
    /// An occurrence (component pick priority, UI-04).
    Occurrence(OccurrenceId),
}

impl PickTarget {
    /// The body, for body targets.
    pub fn body(&self) -> Option<BodyId> {
        match self {
            PickTarget::Body { body, .. } => Some(*body),
            _ => None,
        }
    }

    /// The same target without its sub-entity (the whole body).
    #[must_use]
    pub fn whole(self) -> PickTarget {
        match self {
            PickTarget::Body { body, instance, .. } => PickTarget::Body {
                body,
                instance,
                sub: None,
            },
            other => other,
        }
    }
}

/// How many hits a point pick returns.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PickMode {
    /// The front-most hit.
    Nearest,
    /// Every pickable thing along the ray, front to back (select-through, UI-04).
    AllAlongRay,
}

/// Identifies a pick request until its result is polled.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct PickTicket(pub u64);

/// One pick result in document terms.
#[derive(Clone, Debug, PartialEq)]
pub struct PickHitUi {
    /// What was hit.
    pub target: PickTarget,
    /// World position of the hit, mm.
    pub world: amg_core::Point3,
    /// Distance along the pick ray, mm (f32 like S10's depth).
    pub depth: f32,
}
