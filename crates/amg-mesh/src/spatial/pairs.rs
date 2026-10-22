//! Self-traversal of a [`Bvh`]: all pairs of primitives whose boxes overlap (MESH-19).
//!
//! Each leaf `L` (in parallel) walks the tree once and meets only the leaves that come after
//! it in depth-first order, so every unordered pair is produced exactly once. The walk skips a
//! left subtree when `L` lies in the right subtree (depth-first order makes the left subtree
//! entirely earlier). The result is a pure function of the tree and the boxes.

use amg_core::Aabb3;
use amg_jobs::JobCtx;
use rayon::prelude::*;

use super::bvh::{Bvh, RawNode, STACK};
use crate::error::MeshResult;

/// Leaves handled per parallel batch (cancellation and `max_items` are checked between
/// batches).
const BATCH_LEAVES: usize = 4096;

#[inline]
fn nodes_overlap(a: &RawNode, b: &RawNode) -> bool {
    a.min[0] <= b.max[0]
        && a.min[1] <= b.max[1]
        && a.min[2] <= b.max[2]
        && b.min[0] <= a.max[0]
        && b.min[1] <= a.max[1]
        && b.min[2] <= a.max[2]
}

#[inline]
fn boxes_overlap(a: &Aabb3, b: &Aabb3) -> bool {
    a.min.cmple(b.max).all() && b.min.cmple(a.max).all()
}

impl Bvh {
    /// Indices of all leaf nodes in depth-first order.
    fn leaf_nodes(&self) -> Vec<u32> {
        (0..self.nodes.len() as u32)
            .filter(|&i| self.nodes[i as usize].is_leaf())
            .collect()
    }

    /// Walks the tree for leaf `li` and calls `visit(a, b)` for every primitive pair `(a, b)`
    /// with `a < b` whose exact boxes overlap, where one primitive is in leaf `li` and the
    /// other in `li` or a later leaf.
    fn leaf_pairs(&self, li: u32, prim_bounds: &[Aabb3], visit: &mut dyn FnMut(u32, u32)) {
        let leaf = &self.nodes[li as usize];
        let first = leaf.left_or_first as usize;
        let count = leaf.count as usize;
        let mine = &self.prim_indices[first..first + count];
        let mut emit = |p: u32, q: u32| {
            if boxes_overlap(&prim_bounds[p as usize], &prim_bounds[q as usize]) {
                visit(p.min(q), p.max(q));
            }
        };
        // Pairs inside the leaf.
        for (k, &p) in mine.iter().enumerate() {
            for &q in &mine[k + 1..] {
                emit(p, q);
            }
        }
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let i = stack[sp];
            let n = &self.nodes[i as usize];
            if i > li {
                if !nodes_overlap(n, leaf) {
                    continue;
                }
                if n.is_leaf() {
                    let f = n.left_or_first as usize;
                    for &q in &self.prim_indices[f..f + n.count as usize] {
                        for &p in mine {
                            emit(p, q);
                        }
                    }
                    continue;
                }
            } else if i == li || n.is_leaf() {
                continue;
            }
            // Here `n` is an inner node that is an ancestor of the leaf, or a later inner
            // node that overlaps it.
            let right = n.left_or_first;
            stack[sp] = right;
            sp += 1;
            if li < right {
                stack[sp] = i + 1;
                sp += 1;
            }
        }
    }

    /// Calls `visit(a, b)` once for every unordered pair of primitives `a < b` whose boxes in
    /// `prim_bounds` overlap (touching counts). Sequential; the order is deterministic.
    ///
    /// `prim_bounds` must be the boxes the tree was built from.
    pub fn overlapping_pairs(&self, prim_bounds: &[Aabb3], visit: &mut dyn FnMut(u32, u32)) {
        for li in self.leaf_nodes() {
            self.leaf_pairs(li, prim_bounds, visit);
        }
    }

    /// Parallel [`Bvh::overlapping_pairs`]: `f(a, b, out)` is called for every overlapping pair
    /// (`a < b`) and appends its results to `out`. The results come back in leaf order, so the
    /// output does not depend on the thread count.
    ///
    /// With `max_items = Some(n)` work stops after the first batch of leaves that brings the
    /// total to `n` or more and the result is truncated to `n` (a deterministic prefix).
    ///
    /// # Errors
    /// [`crate::MeshError::Cancelled`].
    pub fn par_overlapping_pairs<T, F>(
        &self,
        prim_bounds: &[Aabb3],
        max_items: Option<usize>,
        ctx: &JobCtx,
        f: F,
    ) -> MeshResult<Vec<T>>
    where
        T: Send,
        F: Fn(u32, u32, &mut Vec<T>) + Sync,
    {
        let leaves = self.leaf_nodes();
        let mut out: Vec<T> = Vec::new();
        for (bi, batch) in leaves.chunks(BATCH_LEAVES).enumerate() {
            ctx.checkpoint()?;
            let parts: Vec<Vec<T>> = batch
                .par_iter()
                .map(|&li| {
                    let mut v = Vec::new();
                    self.leaf_pairs(li, prim_bounds, &mut |a, b| f(a, b, &mut v));
                    v
                })
                .collect();
            for p in parts {
                out.extend(p);
            }
            if max_items.is_some_and(|m| out.len() >= m) {
                break;
            }
            ctx.progress(((bi + 1) * BATCH_LEAVES).min(leaves.len()) as f32 / leaves.len() as f32);
        }
        if let Some(m) = max_items {
            out.truncate(m);
        }
        Ok(out)
    }
}
