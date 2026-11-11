//! Assembly features and command builders (S06, lane E).
//!
//! ASM-09 lands the first part: moving bodies between components.
//!
//! - [`BodiesToComponentsFeature`] (`assembly.bodies_to_components` v1) is the timeline node
//!   that re-homes bodies. Each [`BodyMove`] stores the rigid (or mirrored) transform from the
//!   body's old component space to the new one, captured when the command was built, so the
//!   body keeps its world placement at that moment and afterwards moves with its new
//!   component. Moving an occurrence later therefore recomputes nothing (S06 §1).
//! - [`components_from_bodies`] and [`move_bodies`] build the commands of one transaction
//!   ("Create Components from Bodies", "Move to Component").
//!
//! Call [`link`] from a binary or test so the registrations are linked in.

mod bodies;

pub use bodies::{
    AsmPrepError, BODIES_TO_COMPONENTS_KIND, BodiesToComponentsFeature, BodyMove,
    components_from_bodies, move_bodies,
};

/// Forces this module's feature registrations to be linked (registry `link()` convention).
pub fn link() {}
