//! Streaming exact weld (note §5.3): vertices are merged on the bit patterns of their f32
//! coordinates (`-0.0` already normalised to `+0.0` by the parsers).
//!
//! The result does not depend on the number of threads and equals the sequential
//! "first seen, first numbered" order:
//! 1. corners of a chunk are cut into blocks; each block is counting-sorted by shard (the top
//!    bits of the key hash), keeping corner order inside a shard;
//! 2. shards run in parallel, each walking the blocks in order, so a shard sees its corners in
//!    file order and numbers its new vertices `0, 1, 2, ...` locally;
//! 3. the new vertices of all shards are merged by corner index (one sort) and get their global
//!    ids in file order;
//! 4. blocks translate their corners back to global ids, each writing its own slice.
//!
//! No `unsafe` and no locks: every parallel step owns its output.

use amg_core::Point3;
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::error::IoError;

/// Bit patterns of the three coordinates of a corner.
pub(super) type Key = [u32; 3];

const SHARD_BITS: u32 = 6;
const SHARDS: usize = 1 << SHARD_BITS;
const EMPTY: u32 = u32::MAX;
/// Smallest number of corners worth a block of their own.
const MIN_BLOCK: usize = 4096;
const MAX_BLOCKS: usize = 64;

#[inline]
fn hash(k: &Key) -> u64 {
    const M: u64 = 0x9E37_79B9_7F4A_7C15;
    let mut h = u64::from(k[0]).wrapping_mul(M);
    h = (h.rotate_left(26) ^ u64::from(k[1])).wrapping_mul(M);
    h = (h.rotate_left(26) ^ u64::from(k[2])).wrapping_mul(M);
    // splitmix64 finalizer: top bits pick the shard, low bits the slot.
    h ^= h >> 30;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 27;
    h = h.wrapping_mul(0x94D0_49BB_1331_11EB);
    h ^ (h >> 31)
}

#[inline]
fn shard_of(h: u64) -> usize {
    (h >> (64 - SHARD_BITS)) as usize
}

#[derive(Clone, Copy)]
struct Slot {
    key: Key,
    val: u32,
}

/// Open-addressing table `Key -> local id`, linear probing, load factor at most 1/2.
struct Table {
    slots: Vec<Slot>,
    mask: usize,
    len: usize,
}

impl Table {
    fn new() -> Self {
        Self::with_slots(1 << 10)
    }

    fn with_slots(n: usize) -> Self {
        debug_assert!(n.is_power_of_two());
        Table {
            slots: vec![
                Slot {
                    key: [0; 3],
                    val: EMPTY
                };
                n
            ],
            mask: n - 1,
            len: 0,
        }
    }

    fn grow(&mut self) {
        let mut bigger = Table::with_slots(self.slots.len() * 2);
        for s in self.slots.iter().filter(|s| s.val != EMPTY) {
            let mut i = hash(&s.key) as usize & bigger.mask;
            while bigger.slots[i].val != EMPTY {
                i = (i + 1) & bigger.mask;
            }
            bigger.slots[i] = *s;
        }
        bigger.len = self.len;
        *self = bigger;
    }

    /// The local id of `key`; a new key gets `next` and the second value is `true`.
    #[inline]
    fn get_or_insert(&mut self, key: &Key, h: u64, next: u32) -> (u32, bool) {
        if (self.len + 1) * 2 > self.slots.len() {
            self.grow();
        }
        let mut i = h as usize & self.mask;
        loop {
            let s = &mut self.slots[i];
            if s.val == EMPTY {
                *s = Slot {
                    key: *key,
                    val: next,
                };
                self.len += 1;
                return (next, true);
            }
            if s.key == *key {
                return (s.val, false);
            }
            i = (i + 1) & self.mask;
        }
    }
}

struct Shard {
    table: Table,
    /// Local id -> global id (filled once the chunk's new vertices are numbered).
    global: Vec<u32>,
    /// Corner indices (in the current chunk) that created a vertex, ascending.
    news: Vec<u32>,
    /// Per block: the local id of every entry this shard got from that block.
    res: Vec<Vec<u32>>,
}

/// Corners of one block, grouped by shard.
struct Block {
    /// Corner indices (chunk-wide), shard-major, ascending within a shard.
    idx: Vec<u32>,
    hs: Vec<u64>,
    starts: [u32; SHARDS + 1],
}

/// The welder of one object.
pub(super) struct Welder {
    shards: Vec<Shard>,
    positions: Vec<Point3>,
    tris: Vec<[u32; 3]>,
    max_vertices: u64,
}

#[inline]
fn point(k: &Key) -> Point3 {
    Point3::new(
        f64::from(f32::from_bits(k[0])),
        f64::from(f32::from_bits(k[1])),
        f64::from(f32::from_bits(k[2])),
    )
}

impl Welder {
    /// `expected_tris` sizes the output arrays (a hint, not a limit).
    pub(super) fn new(expected_tris: usize, max_vertices: u64) -> Self {
        let shards = (0..SHARDS)
            .map(|_| Shard {
                table: Table::new(),
                global: Vec::new(),
                news: Vec::new(),
                res: Vec::new(),
            })
            .collect();
        // A closed triangle mesh has about half as many vertices as faces.
        let positions = Vec::with_capacity(expected_tris / 2 + 16);
        let tris = Vec::with_capacity(expected_tris);
        Welder {
            shards,
            positions,
            tris,
            max_vertices,
        }
    }

    pub(super) fn triangle_count(&self) -> usize {
        self.tris.len()
    }

    pub(super) fn finish(self) -> (Vec<Point3>, Vec<[u32; 3]>) {
        (self.positions, self.tris)
    }

    /// Welds the triangles whose corners are `corners` (three per triangle, in order).
    pub(super) fn push(&mut self, corners: &[Key], ctx: &JobCtx) -> Result<(), IoError> {
        debug_assert_eq!(corners.len() % 3, 0);
        let c = corners.len();
        if c == 0 {
            return Ok(());
        }
        ctx.checkpoint()?;
        let nb = (c / MIN_BLOCK).clamp(1, MAX_BLOCKS);
        let bs = c.div_ceil(nb);

        let blocks: Vec<Block> = corners
            .par_chunks(bs)
            .enumerate()
            .map(|(b, chunk)| {
                let base = b * bs;
                let mut counts = [0u32; SHARDS];
                let hs_orig: Vec<u64> = chunk
                    .iter()
                    .map(|k| {
                        let h = hash(k);
                        counts[shard_of(h)] += 1;
                        h
                    })
                    .collect();
                let mut starts = [0u32; SHARDS + 1];
                for s in 0..SHARDS {
                    starts[s + 1] = starts[s] + counts[s];
                }
                let mut cursor = starts;
                let mut idx = vec![0u32; chunk.len()];
                let mut hs = vec![0u64; chunk.len()];
                for (i, &h) in hs_orig.iter().enumerate() {
                    let s = shard_of(h);
                    let p = cursor[s] as usize;
                    idx[p] = (base + i) as u32;
                    hs[p] = h;
                    cursor[s] += 1;
                }
                Block { idx, hs, starts }
            })
            .collect();
        ctx.checkpoint()?;

        let nblocks = blocks.len();
        self.shards.par_iter_mut().enumerate().for_each(|(s, sh)| {
            sh.news.clear();
            sh.res.resize_with(nblocks, Vec::new);
            for (b, blk) in blocks.iter().enumerate() {
                let (lo, hi) = (blk.starts[s] as usize, blk.starts[s + 1] as usize);
                let out = &mut sh.res[b];
                out.clear();
                out.reserve(hi - lo);
                if ctx.is_cancelled() {
                    continue;
                }
                for k in lo..hi {
                    let ci = blk.idx[k];
                    let next = sh.global.len() as u32;
                    let (local, inserted) =
                        sh.table
                            .get_or_insert(&corners[ci as usize], blk.hs[k], next);
                    if inserted {
                        sh.global.push(EMPTY);
                        sh.news.push(ci);
                    }
                    out.push(local);
                }
            }
        });
        ctx.checkpoint()?;

        // New vertices in file order.
        let total_new: usize = self.shards.iter().map(|s| s.news.len()).sum();
        let after = self.positions.len() as u64 + total_new as u64;
        if after > self.max_vertices {
            return Err(IoError::LimitExceeded {
                what: "vertices",
                limit: self.max_vertices,
                actual: after,
            });
        }
        let mut packed: Vec<u64> = Vec::with_capacity(total_new);
        let mut next_local = [0u32; SHARDS];
        for (s, sh) in self.shards.iter().enumerate() {
            next_local[s] = (sh.global.len() - sh.news.len()) as u32;
            packed.extend(sh.news.iter().map(|&ci| (u64::from(ci) << 8) | s as u64));
        }
        packed.par_sort_unstable();
        self.positions.reserve(total_new);
        for p in packed {
            let s = (p & 0xFF) as usize;
            let ci = (p >> 8) as usize;
            let local = next_local[s] as usize;
            next_local[s] += 1;
            self.shards[s].global[local] = self.positions.len() as u32;
            self.positions.push(point(&corners[ci]));
        }
        ctx.checkpoint()?;

        // Corners back to global ids, block by block.
        let shards = &self.shards;
        let mut gids = vec![0u32; c];
        gids.par_chunks_mut(bs)
            .zip(blocks.par_iter())
            .enumerate()
            .for_each(|(b, (out, blk))| {
                let base = b * bs;
                for (s, sh) in shards.iter().enumerate() {
                    let (lo, hi) = (blk.starts[s] as usize, blk.starts[s + 1] as usize);
                    for (j, k) in (lo..hi).enumerate() {
                        out[blk.idx[k] as usize - base] = sh.global[sh.res[b][j] as usize];
                    }
                }
            });
        self.tris
            .extend(gids.chunks_exact(3).map(|t| [t[0], t[1], t[2]]));
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    fn run(keys: &[Key], threads: usize, pieces: usize) -> (Vec<Point3>, Vec<[u32; 3]>) {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("pool");
        pool.install(|| {
            let ctx = JobCtx::detached();
            let mut w = Welder::new(keys.len() / 3, u64::MAX);
            let per = (keys.len() / 3).div_ceil(pieces).max(1) * 3;
            for part in keys.chunks(per) {
                w.push(part, &ctx).expect("push");
            }
            w.finish()
        })
    }

    #[test]
    fn first_seen_order_and_dedup() {
        let a = [1, 2, 3];
        let b = [4, 5, 6];
        let c = [7, 8, 9];
        let (pos, tris) = run(&[a, b, c, c, b, a], 2, 1);
        assert_eq!(pos.len(), 3);
        assert_eq!(tris, vec![[0, 1, 2], [2, 1, 0]]);
    }

    #[test]
    fn matches_a_btreeset_oracle_for_any_split_and_thread_count() {
        // Pseudo-random keys from a small alphabet so that duplicates are common.
        let mut x = 0x1234_5678_9ABC_DEF0u64;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            x
        };
        let keys: Vec<Key> = (0..60_000)
            .map(|_| {
                let v = next();
                [
                    (v % 17) as u32,
                    ((v >> 8) % 19) as u32 * 7,
                    ((v >> 16) % 23) as u32 * 1_000_003,
                ]
            })
            .collect();
        let distinct: BTreeSet<Key> = keys.iter().copied().collect();
        let reference = run(&keys, 1, 1);
        assert_eq!(reference.0.len(), distinct.len());
        for (threads, pieces) in [(1, 7), (4, 1), (16, 13)] {
            assert_eq!(run(&keys, threads, pieces), reference, "{threads}/{pieces}");
        }
        // Ids are first-seen: vertex i is the i-th distinct key in input order.
        let mut seen = BTreeSet::new();
        let first_seen: Vec<Key> = keys.iter().copied().filter(|k| seen.insert(*k)).collect();
        for (p, k) in reference.0.iter().zip(&first_seen) {
            assert_eq!(*p, point(k));
        }
    }

    #[test]
    fn vertex_limit_is_enforced() {
        let ctx = JobCtx::detached();
        let mut w = Welder::new(2, 3);
        let keys: Vec<Key> = (0..6).map(|i| [i, 0, 0]).collect();
        assert!(matches!(
            w.push(&keys, &ctx),
            Err(IoError::LimitExceeded {
                what: "vertices",
                ..
            })
        ));
    }
}
