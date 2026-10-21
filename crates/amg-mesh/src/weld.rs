//! Vertex welding: cluster points that are at most `tolerance` apart (transitively).
//!
//! Algorithm (deterministic, independent of the rayon thread count):
//! 1. **Exact pass.** Each point gets a 64-bit entry `(32-bit hash of its bits, index)`; the
//!    entries are sorted with a parallel unstable sort (keys are unique, so the order is fully
//!    determined). Runs of equal hash are scanned in parallel; inside a run every point joins
//!    the first bit-equal point. Equality is transitive, so that point is the cluster root.
//! 2. **Tolerance pass** (only for `tolerance > 0`, over the exact-pass representatives).
//!    Every point gets a *home* entry keyed by its grid cell (cell size 8 × tolerance, so a cell of
//!    clustered data holds a bounded number of points) and a *ghost* entry in each neighbouring
//!    cell its tolerance ball may reach (only points within `GHOST_MARGIN` = 1/8 + 1/64 of a
//!    cell of a cell face; the 1/64 absorbs rounding of the cell coordinate). Inside each run
//!    of equal cell hash, pairs with at least one home entry are compared exactly
//!    (`|p − q|² ≤ tol²`) and joined in a union-find whose root is the smallest index. Any two
//!    points within tolerance share at least one cell where one of them is at home, so no pair
//!    is missed; hash collisions only add comparisons.
//!
//! Exact duplicates are within any tolerance, so pass 1 does not change the transitive closure
//! computed by pass 2; it only shrinks pass 2's input (an STL soup has ≈ 6 copies of each
//! vertex). With `tolerance == 0` only bit-equal points merge (`-0.0` equals `0.0`, NaN never
//! merges).
//!
//! The representative of each cluster is its smallest index, and its position is kept
//! bit-for-bit.

use std::sync::atomic::{AtomicU32, Ordering};

use amg_core::Point3;
use rayon::prelude::*;

use crate::error::{MeshError, MeshResult};

/// Grid cell edge in units of the tolerance. Smaller cells bound the pairs compared inside a
/// cell for clustered input, larger ones cut the ghost entries (`(1 + 2·GHOST_MARGIN)³`
/// entries per point on average): 2 would ghost every point, 8 gives about 2.2 entries.
const CELL_FACTOR: f64 = 8.0;
/// A point closer than this (in cell units) to a cell face gets a ghost in the neighbour. Must
/// be at least `1 / CELL_FACTOR` (the tolerance ball's reach) plus rounding slack.
const GHOST_MARGIN: f64 = 1.0 / CELL_FACTOR + 1.0 / 64.0;

#[inline]
fn fmix64(mut k: u64) -> u64 {
    k ^= k >> 33;
    k = k.wrapping_mul(0xff51_afd7_ed55_8ccd);
    k ^= k >> 33;
    k = k.wrapping_mul(0xc4ce_b9fe_1a85_ec53);
    k ^= k >> 33;
    k
}

/// 32-bit hash of three words (the high half of a 64-bit mix).
#[inline]
fn hash3(x: u64, y: u64, z: u64) -> u64 {
    fmix64(
        x.wrapping_mul(0x9e37_79b9_7f4a_7c15)
            ^ y.wrapping_mul(0xc2b2_ae3d_27d4_eb4f).rotate_left(21)
            ^ z.wrapping_mul(0x1656_67b1_9e37_79f9).rotate_left(42),
    ) >> 32
}

/// Entry: hash in the high 32 bits, then the point index, then the ghost flag.
#[inline]
fn entry(key: u64, idx: u32, ghost: bool) -> u64 {
    (key << 32) | (u64::from(idx) << 1) | u64::from(ghost)
}

#[inline]
fn entry_key(e: u64) -> u64 {
    e >> 32
}

#[inline]
fn entry_idx(e: u64) -> (usize, bool) {
    (((e & 0xffff_ffff) >> 1) as usize, e & 1 == 1)
}

/// Converts a count to `u32`, failing with `TooLarge` at `u32::MAX` (reserved for `INVALID`).
pub(crate) fn to_u32(count: usize) -> MeshResult<u32> {
    u32::try_from(count)
        .ok()
        .filter(|&c| c != u32::MAX)
        .ok_or(MeshError::TooLarge { count })
}

fn find(parent: &mut [u32], mut i: u32) -> u32 {
    let mut r = i;
    while parent[r as usize] != r {
        r = parent[r as usize];
    }
    while parent[i as usize] != r {
        let next = parent[i as usize];
        parent[i as usize] = r;
        i = next;
    }
    r
}

/// Splits sorted `entries` into about `parts` chunks that never cut a run of equal keys.
fn run_chunks(entries: &[u64], parts: usize) -> Vec<std::ops::Range<usize>> {
    let n = entries.len();
    let mut out = Vec::with_capacity(parts);
    let mut start = 0;
    for c in 1..=parts {
        let mut end = (n * c / parts).max(start);
        while end < n && end > 0 && entry_key(entries[end]) == entry_key(entries[end - 1]) {
            end += 1;
        }
        if end > start {
            out.push(start..end);
            start = end;
        }
    }
    out
}

/// For each point, the smallest index of its weld cluster.
pub(crate) fn weld_roots(p: &[Point3], tolerance: f64) -> MeshResult<Vec<u32>> {
    // Indices are stored shifted by one bit next to the ghost flag.
    if p.len() >= (1usize << 31) {
        return Err(MeshError::TooLarge { count: p.len() });
    }
    let roots = weld_exact(p);
    if !(tolerance > 0.0 && tolerance.is_finite()) {
        return Ok(roots);
    }
    // Representatives in ascending order, so their local order matches the global one.
    let reps: Vec<u32> = (0..p.len() as u32)
        .filter(|&i| roots[i as usize] == i)
        .collect();
    let q: Vec<Point3> = reps.par_iter().map(|&i| p[i as usize]).collect();
    let local = weld_tolerant(&q, tolerance);
    let mut rep_pos = vec![0u32; p.len()];
    for (k, &i) in reps.iter().enumerate() {
        rep_pos[i as usize] = k as u32;
    }
    Ok(roots
        .par_iter()
        .map(|&r| reps[local[rep_pos[r as usize] as usize] as usize])
        .collect())
}

fn weld_exact(p: &[Point3]) -> Vec<u32> {
    let mut entries: Vec<u64> = p
        .par_iter()
        .enumerate()
        .map(|(i, q)| {
            // `+ 0.0` maps -0.0 to +0.0 so equal values hash equally.
            let k = hash3(
                (q.x + 0.0).to_bits(),
                (q.y + 0.0).to_bits(),
                (q.z + 0.0).to_bits(),
            );
            entry(k, i as u32, false)
        })
        .collect();
    entries.par_sort_unstable();

    let roots: Vec<AtomicU32> = (0..p.len() as u32).map(AtomicU32::new).collect();
    let parts = rayon::current_num_threads() * 8;
    run_chunks(&entries, parts)
        .into_par_iter()
        .for_each(|range| {
            let e = &entries[range];
            let mut start = 0;
            while start < e.len() {
                let key = entry_key(e[start]);
                let mut end = start + 1;
                while end < e.len() && entry_key(e[end]) == key {
                    end += 1;
                }
                // Runs are tiny (≈ 6 for a soup); collisions only add comparisons.
                for j in start + 1..end {
                    let (pj, _) = entry_idx(e[j]);
                    for &ei in &e[start..j] {
                        let (pi, _) = entry_idx(ei);
                        // Every point of a run belongs to this chunk only, so the loads see this
                        // thread's own stores.
                        if roots[pi].load(Ordering::Relaxed) as usize == pi && p[pi] == p[pj] {
                            roots[pj].store(pi as u32, Ordering::Relaxed);
                            break;
                        }
                    }
                }
                start = end;
            }
        });
    roots.into_iter().map(AtomicU32::into_inner).collect()
}

fn weld_tolerant(p: &[Point3], tol: f64) -> Vec<u32> {
    let cell = tol * CELL_FACTOR;
    let inv = 1.0 / cell;
    let tol2 = tol * tol;

    let mut entries: Vec<u64> = p
        .par_iter()
        .enumerate()
        .flat_map_iter(|(i, q)| {
            let s = *q * inv;
            let f = s.floor();
            let frac = s - f;
            // Saturating casts: non-finite coordinates land in some cell and never compare
            // within tolerance.
            #[allow(clippy::cast_possible_truncation)]
            let c = [f.x as i64, f.y as i64, f.z as i64];
            let mut opts = [[0i64; 2]; 3];
            let mut n_opts = [1usize; 3];
            for a in 0..3 {
                if frac[a] < GHOST_MARGIN {
                    opts[a][1] = -1;
                    n_opts[a] = 2;
                } else if frac[a] > 1.0 - GHOST_MARGIN {
                    opts[a][1] = 1;
                    n_opts[a] = 2;
                }
            }
            let mut out = ArrayVec8 {
                buf: [0; 8],
                len: 0,
            };
            for ix in 0..n_opts[0] {
                for iy in 0..n_opts[1] {
                    for iz in 0..n_opts[2] {
                        let k = hash3(
                            c[0].wrapping_add(opts[0][ix]) as u64,
                            c[1].wrapping_add(opts[1][iy]) as u64,
                            c[2].wrapping_add(opts[2][iz]) as u64,
                        );
                        out.push(entry(k, i as u32, ix + iy + iz > 0));
                    }
                }
            }
            out
        })
        .collect();
    entries.par_sort_unstable();

    // Union pairs per chunk in parallel, then join them in order (the root is the minimum
    // index whatever the union order, so the result is deterministic).
    let parts = rayon::current_num_threads() * 8;
    let pairs: Vec<Vec<(u32, u32)>> = run_chunks(&entries, parts)
        .into_par_iter()
        .map(|range| {
            let e = &entries[range];
            let mut out = Vec::new();
            let mut start = 0;
            while start < e.len() {
                let key = entry_key(e[start]);
                let mut end = start + 1;
                while end < e.len() && entry_key(e[end]) == key {
                    end += 1;
                }
                for j in start + 1..end {
                    let (pj, gj) = entry_idx(e[j]);
                    for &ei in &e[start..j] {
                        let (pi, gi) = entry_idx(ei);
                        if !(gi && gj) && p[pi].distance_squared(p[pj]) <= tol2 {
                            out.push((pi as u32, pj as u32));
                        }
                    }
                }
                start = end;
            }
            out
        })
        .collect();
    let mut parent: Vec<u32> = (0..p.len() as u32).collect();
    for (a, b) in pairs.into_iter().flatten() {
        let ra = find(&mut parent, a);
        let rb = find(&mut parent, b);
        if ra != rb {
            let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
            parent[hi as usize] = lo;
        }
    }
    for i in 0..parent.len() {
        let r = find(&mut parent, i as u32);
        parent[i] = r;
    }
    parent
}

/// Up to 8 entries per point; a fixed array avoids a heap allocation per point.
struct ArrayVec8 {
    buf: [u64; 8],
    len: usize,
}

impl ArrayVec8 {
    fn push(&mut self, v: u64) {
        self.buf[self.len] = v;
        self.len += 1;
    }
}

impl IntoIterator for ArrayVec8 {
    type Item = u64;
    type IntoIter = std::iter::Take<std::array::IntoIter<u64, 8>>;
    fn into_iter(self) -> Self::IntoIter {
        self.buf.into_iter().take(self.len)
    }
}

/// Assigns compact ids to cluster roots in order of first appearance: returns
/// (`new_id` per input point, root indices in new-id order).
pub(crate) fn compact_roots(roots: &[u32]) -> (Vec<u32>, Vec<u32>) {
    let mut new_id = vec![0u32; roots.len()];
    let mut kept = Vec::new();
    for (i, &r) in roots.iter().enumerate() {
        if r as usize == i {
            new_id[i] = kept.len() as u32;
            kept.push(i as u32);
        } else {
            // r < i, so it already has its id.
            new_id[i] = new_id[r as usize];
        }
    }
    (new_id, kept)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn brute(p: &[Point3], tol: f64) -> Vec<u32> {
        let mut parent: Vec<u32> = (0..p.len() as u32).collect();
        for i in 0..p.len() {
            for j in i + 1..p.len() {
                // `==` covers tol = 0 with -0.0 == 0.0; NaN never matches.
                if p[i] == p[j] || p[i].distance_squared(p[j]) <= tol * tol {
                    let ri = find(&mut parent, i as u32);
                    let rj = find(&mut parent, j as u32);
                    let (lo, hi) = if ri < rj { (ri, rj) } else { (rj, ri) };
                    parent[hi as usize] = lo;
                }
            }
        }
        (0..p.len() as u32).map(|i| find(&mut parent, i)).collect()
    }

    #[test]
    fn exact_merges_bit_equal_only() {
        let p = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(-0.0, 0.0, 0.0),
            Point3::new(1e-300, 0.0, 0.0),
            Point3::new(f64::NAN, 0.0, 0.0),
            Point3::new(f64::NAN, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        ];
        assert_eq!(weld_roots(&p, 0.0).unwrap(), vec![0, 0, 2, 3, 4, 0]);
    }

    #[test]
    fn tolerant_matches_brute_force() {
        // Points clustered near cell boundaries, with chains and exact duplicates.
        let tol = 1e-3;
        let mut s = 0x1234_5678_u64;
        let mut rnd = || {
            s = s.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            ((s >> 11) as f64) / ((1u64 << 53) as f64)
        };
        for _ in 0..20 {
            let mut p: Vec<Point3> = Vec::new();
            for k in 0..400 {
                if k % 5 == 4 {
                    let dup = p[(rnd() * p.len() as f64) as usize];
                    p.push(dup);
                    continue;
                }
                // Coarse lattice near multiples of the cell size, plus jitter up to 2 tol.
                let base = Point3::new(
                    (rnd() * 4.0).floor() * tol * CELL_FACTOR,
                    (rnd() * 4.0).floor() * tol * CELL_FACTOR,
                    (rnd() * 2.0).floor() * tol,
                );
                let j = Point3::new(rnd() - 0.5, rnd() - 0.5, rnd() - 0.5) * (2.0 * tol);
                p.push(base + j);
            }
            assert_eq!(weld_roots(&p, tol).unwrap(), brute(&p, tol));
            assert_eq!(weld_roots(&p, 0.0).unwrap(), brute(&p, 0.0));
        }
    }

    #[test]
    fn independent_of_thread_count() {
        let p: Vec<Point3> = (0..20_000)
            .map(|i| {
                let k = (i * 7919) % 5000;
                Point3::new(k as f64 * 1e-3, (k % 17) as f64, 1e-6 * (i % 3) as f64)
            })
            .collect();
        let run = |threads: usize| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap()
                .install(|| (weld_roots(&p, 0.0).unwrap(), weld_roots(&p, 1e-5).unwrap()))
        };
        let a = run(1);
        assert_eq!(a, run(4));
        assert_eq!(a, run(16));
    }

    #[test]
    fn chunks_respect_runs() {
        let e: Vec<u64> = [1u64, 1, 1, 2, 3, 3, 3, 3, 4]
            .iter()
            .enumerate()
            .map(|(i, &k)| entry(k, i as u32, false))
            .collect();
        for parts in 1..12 {
            let c = run_chunks(&e, parts);
            assert_eq!(c.first().unwrap().start, 0);
            assert_eq!(c.last().unwrap().end, e.len());
            for w in c.windows(2) {
                assert_eq!(w[0].end, w[1].start);
                assert_ne!(entry_key(e[w[1].start]), entry_key(e[w[1].start - 1]));
            }
        }
    }

    #[test]
    fn compact_assigns_first_appearance() {
        let (id, kept) = compact_roots(&[0, 1, 0, 3, 1]);
        assert_eq!(id, vec![0, 1, 0, 2, 1]);
        assert_eq!(kept, vec![0, 1, 3]);
    }

    #[test]
    fn dense_lattice_is_linear_and_exact() {
        let tol = 1e-3;
        let lattice = |step: f64| -> Vec<Point3> {
            let mut p = Vec::new();
            for x in 0..20 {
                for y in 0..20 {
                    for z in 0..20 {
                        p.push(Point3::new(x as f64, y as f64, z as f64) * (step * tol));
                    }
                }
            }
            p
        };
        // Spacing above the tolerance: nothing merges. Below: one chain-connected cluster.
        let apart = lattice(1.5);
        assert_eq!(
            weld_roots(&apart, tol).unwrap(),
            (0..apart.len() as u32).collect::<Vec<_>>()
        );
        let close = lattice(0.9);
        assert!(weld_roots(&close, tol).unwrap().iter().all(|&r| r == 0));
        assert_eq!(weld_roots(&apart, tol).unwrap(), brute(&apart, tol));
    }
}
