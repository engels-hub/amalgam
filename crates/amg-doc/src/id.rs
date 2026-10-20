//! Authored and derived ids (S01 §1 Identity).
//!
//! - Authored ids are typed `u64`s allocated from the design's monotonic counter. They are
//!   never reused, persisted as plain numbers, and never random, so anything that feeds a cache
//!   key is deterministic.
//! - Derived ids ([`BodyId`], [`SketchId`], [`ConstructionId`]) name recompute outputs by the
//!   feature that created them, so recompute never allocates.
//! - The document itself is identified by an [`amg_core::PersistentId`] (UUID v7).

use serde::{Deserialize, Serialize};

macro_rules! typed_id {
    ($($(#[$m:meta])* $name:ident),* $(,)?) => {$(
        $(#[$m])*
        #[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(pub u64);

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "{}#{}", stringify!($name), self.0)
            }
        }
    )*};
}

typed_id!(
    /// A timeline feature.
    FeatureId,
    /// A user or model parameter.
    ParamId,
    /// A component (part definition) in the component tree.
    ComponentId,
    /// One placement of a component inside a parent component.
    OccurrenceId,
    /// A timeline group.
    GroupId,
    /// A configuration (DOC-10).
    ConfigId,
    /// A joint between occurrences (S06). Allocated like every other authored id.
    JointId,
);

/// A body, named by the feature that created it and that feature's output slot.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BodyId {
    /// Feature that created the body.
    pub origin: FeatureId,
    /// Output slot of that feature (0 for single-body features).
    pub slot: u32,
}

/// Name used by S04 for a body input.
pub type BodyRef = BodyId;

/// A sketch, named by its creating feature.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct SketchId(pub FeatureId);

/// A construction plane/axis/point, named by its creating feature.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ConstructionId(pub FeatureId);

/// Key of an extension record (studies, print setups, ...): owning kind plus an authored id.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExtKey {
    /// Registered extension kind, e.g. `"fem.study"`.
    pub kind: String,
    /// Id allocated from the design's counter.
    pub id: u64,
}

/// Authored-state revision; bumps on every authored change. Runtime only, never persisted.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Revision(pub u64);

impl Revision {
    /// The next revision.
    #[must_use]
    pub fn next(self) -> Revision {
        Revision(self.0 + 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_serialize_as_plain_numbers() {
        assert_eq!(serde_json::to_string(&FeatureId(7)).unwrap(), "7");
        assert_eq!(serde_json::to_string(&SketchId(FeatureId(3))).unwrap(), "3");
        let b = BodyId {
            origin: FeatureId(4),
            slot: 1,
        };
        let s = serde_json::to_string(&b).unwrap();
        assert_eq!(s, r#"{"origin":4,"slot":1}"#);
        assert_eq!(serde_json::from_str::<BodyId>(&s).unwrap(), b);
        assert!(serde_json::from_str::<BodyId>(r#"{"origin":4,"slot":1,"x":0}"#).is_err());
    }

    #[test]
    fn ordering_and_display() {
        let a = BodyId {
            origin: FeatureId(1),
            slot: 5,
        };
        let b = BodyId {
            origin: FeatureId(2),
            slot: 0,
        };
        assert!(a < b, "bodies order by origin feature first");
        assert_eq!(ComponentId(9).to_string(), "ComponentId#9");
        assert_eq!(Revision(1).next(), Revision(2));
    }
}
