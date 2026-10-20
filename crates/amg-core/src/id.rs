//! Identifiers.
//!
//! - Runtime ids are `slotmap` generational keys, declared per crate with
//!   [`define_id!`](crate::define_id). They are never written to files.
//! - File-persistent ids are [`PersistentId`]s (UUIDs).

use crate::error::Error;

pub use slotmap::{DenseSlotMap, Key, KeyData, SecondaryMap, SlotMap, SparseSecondaryMap};

/// Declares one or more runtime generational ids (slotmap keys). Never serialised to files.
///
/// ```
/// amg_core::define_id! {
///     /// A body in the document.
///     pub struct BodyId;
/// }
/// let mut bodies = amg_core::SlotMap::<BodyId, &str>::with_key();
/// let id = bodies.insert("cube");
/// assert_eq!(bodies[id], "cube");
/// ```
#[macro_export]
macro_rules! define_id {
    ($($(#[$m:meta])* $vis:vis struct $name:ident;)+) => {
        $crate::__private::slotmap::new_key_type! { $($(#[$m])* $vis struct $name;)+ }
    };
}

/// Stable id stored in `.amg` files and exposed to Python.
///
/// Displays and parses as a hyphenated lowercase UUID; serialises as that string in
/// human-readable formats.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(transparent)]
pub struct PersistentId(pub uuid::Uuid);

impl PersistentId {
    /// The all-zero id.
    pub const NIL: PersistentId = PersistentId(uuid::Uuid::nil());

    /// A fresh UUID v7 (time-ordered). Only commands may call this, never recompute, so
    /// recompute stays deterministic.
    #[allow(
        clippy::new_without_default,
        reason = "a random id must be asked for explicitly; Default would hide non-determinism"
    )]
    pub fn new() -> PersistentId {
        PersistentId(uuid::Uuid::now_v7())
    }

    /// UUID v5 (SHA-1) of `role` in namespace `parent`: a deterministic id for recompute
    /// outputs. The same inputs give the same id on every run and machine.
    pub fn derive(parent: PersistentId, role: &[u8]) -> PersistentId {
        PersistentId(uuid::Uuid::new_v5(&parent.0, role))
    }
}

impl std::fmt::Display for PersistentId {
    /// Hyphenated lowercase form, e.g. `67e55044-10b1-426f-9247-bb680e5fe0c8`.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Display::fmt(&self.0.hyphenated(), f)
    }
}

impl std::str::FromStr for PersistentId {
    type Err = Error;
    /// Parses any form `uuid` accepts (hyphenated, simple, braced, urn); `Err(InvalidId)`
    /// otherwise.
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        uuid::Uuid::parse_str(s)
            .map(PersistentId)
            .map_err(|_| Error::InvalidId(s.to_owned()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    crate::define_id! {
        /// Test id.
        pub struct TestId;
        /// Second test id in the same invocation.
        pub(crate) struct OtherId;
    }

    #[test]
    fn define_id_creates_slotmap_keys() {
        let mut m = SlotMap::<TestId, i32>::with_key();
        let a = m.insert(1);
        m.remove(a);
        let b = m.insert(2);
        assert_ne!(a, b, "generational keys differ after reuse");
        assert!(m.get(a).is_none());
        let mut s = SecondaryMap::<TestId, &str>::new();
        s.insert(b, "b");
        assert_eq!(s[b], "b");
        assert!(OtherId::default().is_null());
    }

    #[test]
    fn new_is_v7_and_time_ordered() {
        let a = PersistentId::new();
        let b = PersistentId::new();
        assert_eq!(a.0.get_version_num(), 7);
        assert_ne!(a, b);
        assert!(a < b, "v7 ids from one process are monotone");
    }

    #[test]
    fn derive_is_stable_fixed_vector() {
        let parent: PersistentId = "67e55044-10b1-426f-9247-bb680e5fe0c8".parse().unwrap();
        let d = PersistentId::derive(parent, b"extrude/side/0");
        assert_eq!(d.0.get_version_num(), 5);
        // Fixed vector (cross-checked with Python uuid.uuid5): must never change.
        assert_eq!(d.to_string(), "0bc2cd3c-a596-544c-b7cb-29fbb7f486f6");
        // RFC 4122 Appendix B vector: v5 of "www.example.com" in the DNS namespace.
        let dns = PersistentId(uuid::Uuid::NAMESPACE_DNS);
        assert_eq!(
            PersistentId::derive(dns, b"www.example.com").to_string(),
            "2ed6657d-e927-568b-95e1-2665a8aea6a2"
        );
        assert_ne!(d, PersistentId::derive(parent, b"extrude/side/1"));
    }

    #[test]
    fn nil() {
        assert!(PersistentId::NIL.0.is_nil());
        assert_eq!(
            PersistentId::NIL.to_string(),
            "00000000-0000-0000-0000-000000000000"
        );
    }

    #[test]
    fn display_from_str_round_trip() {
        let a = PersistentId::new();
        let s = a.to_string();
        assert_eq!(s.len(), 36);
        assert_eq!(s.parse::<PersistentId>().unwrap(), a);
        assert!(matches!(
            "not-a-uuid".parse::<PersistentId>(),
            Err(Error::InvalidId(s)) if s == "not-a-uuid"
        ));
    }

    #[test]
    fn serde_is_transparent_string() {
        let id: PersistentId = "67e55044-10b1-426f-9247-bb680e5fe0c8".parse().unwrap();
        let s = serde_json::to_string(&id).unwrap();
        assert_eq!(s, "\"67e55044-10b1-426f-9247-bb680e5fe0c8\"");
        assert_eq!(serde_json::from_str::<PersistentId>(&s).unwrap(), id);
    }
}
