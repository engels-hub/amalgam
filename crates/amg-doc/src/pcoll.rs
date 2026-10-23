//! Persistent collections behind crate-private aliases (S01 §1, open question 1).
//!
//! `imbl` (MPL-2.0) backs authored state. Swapping the backend means changing these aliases
//! and the serde helpers below; nothing outside this module names `imbl` directly. The DOC-01
//! benchmark (`benches/snapshot.rs`) compares it with `rpds` and per-entity `Arc` COW.
//!
//! Ordered maps only: iteration order is deterministic, so `design.json` and cache keys are
//! byte-stable.

use std::fmt;
use std::marker::PhantomData;
use std::sync::Arc;

use serde::de::{Deserialize, Deserializer, Error as _, SeqAccess, Visitor};
use serde::ser::{SerializeSeq, Serializer};

/// Persistent ordered map; `clone()` is O(1), mutation copies O(log n) nodes.
pub(crate) type PMap<K, V> = imbl::OrdMap<K, V>;

/// Persistent vector (RRB tree: cheap split/concat for timeline reorder).
pub(crate) type PVec<T> = imbl::Vector<T>;

/// Persistent ordered set.
pub(crate) type PSet<T> = imbl::OrdSet<T>;

/// One difference between two [`PMap`]s (`PMap::diff`).
pub(crate) type PDiff<'a, 'b, K, V> = imbl::ordmap::DiffItem<'a, 'b, K, V>;

/// Serde for `PMap<K, Arc<V>>` as a JSON array of `[key, value]` pairs in key order.
///
/// An array (not a JSON object) because most keys are structs (`BodyId`, `ExtKey`) and JSON
/// object keys must be strings. Duplicate keys are rejected on load.
pub(crate) mod entries {
    use super::*;

    pub(crate) fn serialize<K, V, S>(map: &PMap<K, Arc<V>>, s: S) -> Result<S::Ok, S::Error>
    where
        K: serde::Serialize + Ord + Clone,
        V: serde::Serialize,
        S: Serializer,
    {
        let mut seq = s.serialize_seq(Some(map.len()))?;
        for (k, v) in map {
            seq.serialize_element(&(k, &**v))?;
        }
        seq.end()
    }

    pub(crate) fn deserialize<'de, K, V, D>(d: D) -> Result<PMap<K, Arc<V>>, D::Error>
    where
        K: Deserialize<'de> + Ord + Clone + fmt::Debug,
        V: Deserialize<'de>,
        D: Deserializer<'de>,
    {
        struct V_<K, V>(PhantomData<(K, V)>);
        impl<'de, K, V> Visitor<'de> for V_<K, V>
        where
            K: Deserialize<'de> + Ord + Clone + fmt::Debug,
            V: Deserialize<'de>,
        {
            type Value = PMap<K, Arc<V>>;
            fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("an array of [key, value] pairs")
            }
            fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<Self::Value, A::Error> {
                let mut map = PMap::new();
                while let Some((k, v)) = seq.next_element::<(K, V)>()? {
                    if map.contains_key(&k) {
                        return Err(A::Error::custom(format_args!("duplicate key {k:?}")));
                    }
                    map.insert(k, Arc::new(v));
                }
                Ok(map)
            }
        }
        d.deserialize_seq(V_(PhantomData))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(serde::Serialize, serde::Deserialize, Debug)]
    struct W(#[serde(with = "entries")] PMap<u64, Arc<String>>);

    #[test]
    fn entries_round_trip_in_key_order() {
        let mut m = PMap::new();
        m.insert(3, Arc::new("c".to_owned()));
        m.insert(1, Arc::new("a".to_owned()));
        let s = serde_json::to_string(&W(m)).unwrap();
        assert_eq!(s, r#"[[1,"a"],[3,"c"]]"#);
        let back: W = serde_json::from_str(&s).unwrap();
        assert_eq!(back.0.len(), 2);
        assert_eq!(*back.0[&3], "c");
    }

    #[test]
    fn duplicate_keys_are_rejected() {
        let e = serde_json::from_str::<W>(r#"[[1,"a"],[1,"b"]]"#).unwrap_err();
        assert!(e.to_string().contains("duplicate key 1"), "{e}");
    }
}
