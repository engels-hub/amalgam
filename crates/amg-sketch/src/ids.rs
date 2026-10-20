//! Stable sketch ids.
//!
//! Unlike `amg_core::define_id!` (runtime slotmap keys), these ids are persistent: they are
//! allocated monotonically per sketch, never reused, and serialised. S01 persistent names
//! (`from SketchCurve(S3, Line#12)`) and profile ids rely on that.

use serde::{Deserialize, Serialize};

macro_rules! id {
    ($(#[$m:meta])* $n:ident) => {
        $(#[$m])*
        #[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $n(pub u32);

        impl $n {
            /// The id as a `usize` index.
            pub fn index(self) -> usize {
                self.0 as usize
            }
        }
    };
}

id!(
    /// A sketch entity (point, line, circle, arc, ...). Never reused within a sketch.
    EntityId
);
id!(
    /// A constraint or dimension. Never reused within a sketch.
    ConstraintId
);
id!(
    /// Index of one `f64` unknown in the sketch's [`ParamStore`](crate::ParamStore).
    /// Stable until [`Sketch::compact`](crate::Sketch::compact) renumbers.
    ParamId
);
id!(
    /// Opaque link to projected 3D geometry; the `EntityRef` behind it lives in
    /// `amg-features::sketch::SketchFeature`.
    ExternalLinkId
);
