//! The geometry cache (S01 §3): `CacheKey → Arc<FeatureOutput>`, LRU by estimated bytes.
//!
//! Shared by the UI thread and recompute jobs. Entries are immutable `Arc`s, so the lock is
//! held only for map bookkeeping, never while a feature computes or while a reader uses an
//! output. Preview-mode outputs use keys of their own (the compute mode is part of the key).

use std::collections::{BTreeMap, HashMap};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use crate::eval::CacheKey;
use crate::feature::FeatureOutput;

/// Default cache budget, bytes (S01 §3: 2 GiB, a preference).
pub const DEFAULT_BUDGET_BYTES: u64 = 2 << 30;

impl CacheKey {
    /// Lowercase hex of the 32 bytes (file names in `cache/`, diagnostics).
    pub fn to_hex(&self) -> String {
        use std::fmt::Write as _;
        self.0.iter().fold(String::with_capacity(64), |mut s, b| {
            let _ = write!(s, "{b:02x}");
            s
        })
    }
}

/// Estimated heap size of an output, for the LRU budget.
///
/// TODO(BREP-02/MESH-02): add `Shape::estimated_bytes()` / mesh sizes (S02 refinement); until
/// then geometry payloads are placeholders and only the fixed per-body overhead counts.
pub fn estimate_bytes(out: &FeatureOutput) -> u64 {
    const BASE: usize = 256;
    const PER_BODY: usize = 192;
    const PER_WARNING: usize = 160;
    let n = BASE + out.bodies.len() * PER_BODY + out.warnings.len() * PER_WARNING;
    u64::try_from(n).unwrap_or(u64::MAX)
}

struct Entry {
    value: Arc<FeatureOutput>,
    bytes: u64,
    tick: u64,
}

#[derive(Default)]
struct Inner {
    map: HashMap<CacheKey, Entry>,
    /// tick → key, oldest first.
    order: BTreeMap<u64, CacheKey>,
    tick: u64,
    used: u64,
    budget: u64,
}

impl Inner {
    fn next_tick(&mut self) -> u64 {
        self.tick += 1;
        self.tick
    }

    fn evict_to(&mut self, limit: u64) {
        while self.used > limit {
            let Some((_, key)) = self.order.pop_first() else {
                break;
            };
            if let Some(e) = self.map.remove(&key) {
                self.used = self.used.saturating_sub(e.bytes);
            }
        }
    }
}

/// Hit/miss/size counters of a [`GeomCache`].
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct CacheStats {
    /// Successful `get`s.
    pub hits: u64,
    /// Failed `get`s.
    pub misses: u64,
    /// Entries now.
    pub entries: usize,
    /// Estimated bytes now.
    pub bytes: u64,
}

/// In-memory feature-output cache, LRU by estimated bytes.
pub struct GeomCache {
    inner: Mutex<Inner>,
    hits: AtomicU64,
    misses: AtomicU64,
}

impl std::fmt::Debug for GeomCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = self.stats();
        f.debug_struct("GeomCache")
            .field("entries", &s.entries)
            .field("bytes", &s.bytes)
            .field("budget", &self.budget())
            .finish()
    }
}

impl GeomCache {
    /// A cache holding at most `budget_bytes` of estimated output size.
    pub fn new(budget_bytes: u64) -> Arc<GeomCache> {
        Arc::new(GeomCache {
            inner: Mutex::new(Inner {
                budget: budget_bytes,
                ..Inner::default()
            }),
            hits: AtomicU64::new(0),
            misses: AtomicU64::new(0),
        })
    }

    fn lock(&self) -> MutexGuard<'_, Inner> {
        // The map stays consistent under a panic elsewhere (every mutation is a short,
        // non-panicking sequence), so a poisoned lock is still usable.
        self.inner.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// The cached output for `key`, marking it most recently used.
    pub fn get(&self, key: &CacheKey) -> Option<Arc<FeatureOutput>> {
        let mut g = self.lock();
        let tick = g.next_tick();
        let found = match g.map.get_mut(key) {
            Some(e) => {
                let old = std::mem::replace(&mut e.tick, tick);
                Some((Arc::clone(&e.value), old))
            }
            None => None,
        };
        match found {
            Some((value, old)) => {
                g.order.remove(&old);
                g.order.insert(tick, *key);
                self.hits.fetch_add(1, Ordering::Relaxed);
                Some(value)
            }
            None => {
                self.misses.fetch_add(1, Ordering::Relaxed);
                None
            }
        }
    }

    /// True if `key` is cached (does not touch recency or counters).
    pub fn contains(&self, key: &CacheKey) -> bool {
        self.lock().map.contains_key(key)
    }

    /// Stores an output, evicting least recently used entries over budget. An output larger
    /// than the whole budget is not stored.
    pub fn insert(&self, key: CacheKey, value: Arc<FeatureOutput>) {
        let bytes = estimate_bytes(&value);
        let mut g = self.lock();
        if let Some(old) = g.map.remove(&key) {
            g.order.remove(&old.tick);
            g.used = g.used.saturating_sub(old.bytes);
        }
        if bytes > g.budget {
            return;
        }
        let tick = g.next_tick();
        g.order.insert(tick, key);
        g.used += bytes;
        g.map.insert(key, Entry { value, bytes, tick });
        let budget = g.budget;
        g.evict_to(budget);
    }

    /// Changes the budget (a preference), evicting as needed.
    pub fn set_budget(&self, budget_bytes: u64) {
        let mut g = self.lock();
        g.budget = budget_bytes;
        g.evict_to(budget_bytes);
    }

    /// Current budget, bytes.
    pub fn budget(&self) -> u64 {
        self.lock().budget
    }

    /// Drops every entry.
    pub fn clear(&self) {
        let mut g = self.lock();
        g.map.clear();
        g.order.clear();
        g.used = 0;
    }

    /// Counters and sizes.
    pub fn stats(&self) -> CacheStats {
        let g = self.lock();
        CacheStats {
            hits: self.hits.load(Ordering::Relaxed),
            misses: self.misses.load(Ordering::Relaxed),
            entries: g.map.len(),
            bytes: g.used,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::feature::{BodyChange, BodyChangeKind};
    use crate::id::{BodyId, FeatureId};

    fn key(n: u8) -> CacheKey {
        CacheKey([n; 32])
    }

    fn out(bodies: usize) -> Arc<FeatureOutput> {
        Arc::new(FeatureOutput {
            bodies: (0..bodies)
                .map(|i| BodyChange {
                    id: BodyId {
                        origin: FeatureId(1),
                        slot: u32::try_from(i).unwrap(),
                    },
                    change: BodyChangeKind::Deleted,
                })
                .collect(),
            ..FeatureOutput::default()
        })
    }

    #[test]
    fn lru_evicts_least_recently_used_by_bytes() {
        let one = estimate_bytes(&out(0));
        let cache = GeomCache::new(one * 3);
        cache.insert(key(1), out(0));
        cache.insert(key(2), out(0));
        cache.insert(key(3), out(0));
        // Touch 1 so 2 is the oldest.
        assert!(cache.get(&key(1)).is_some());
        cache.insert(key(4), out(0));
        assert!(cache.contains(&key(1)));
        assert!(!cache.contains(&key(2)));
        assert!(cache.contains(&key(3)));
        assert!(cache.contains(&key(4)));
        let s = cache.stats();
        assert_eq!((s.entries, s.hits, s.misses), (3, 1, 0));
        assert_eq!(s.bytes, one * 3);
    }

    #[test]
    fn oversized_entry_is_not_stored_and_replace_keeps_accounting() {
        let one = estimate_bytes(&out(0));
        let cache = GeomCache::new(one + 10);
        cache.insert(key(1), out(5));
        assert_eq!(cache.stats().entries, 0);
        cache.insert(key(2), out(0));
        cache.insert(key(2), out(0));
        let s = cache.stats();
        assert_eq!((s.entries, s.bytes), (1, one));
        assert!(cache.get(&key(9)).is_none());
        assert_eq!(cache.stats().misses, 1);
    }

    #[test]
    fn shrinking_the_budget_evicts_and_clear_empties() {
        let one = estimate_bytes(&out(0));
        let cache = GeomCache::new(one * 4);
        for i in 0..4 {
            cache.insert(key(i), out(0));
        }
        cache.set_budget(one * 2);
        assert_eq!(cache.stats().entries, 2);
        assert!(cache.contains(&key(3)));
        cache.clear();
        assert_eq!(cache.stats().bytes, 0);
    }

    #[test]
    fn hex_is_64_lowercase_chars() {
        let h = CacheKey([0xab; 32]).to_hex();
        assert_eq!(h.len(), 64);
        assert!(h.starts_with("abab"));
    }
}
