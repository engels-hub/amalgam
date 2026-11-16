//! Geometric nested dissection (CMP-10): a fill-reducing ordering from the coordinates of the
//! unknowns.
//!
//! Each subset of rows is cut at the median coordinate along one axis of its bounding box. The
//! vertex separator is the one-sided boundary of the cut: the rows of one side that have a
//! neighbour (in the matrix graph, restricted to the subset) on the other side; the smaller of
//! the two boundaries is taken. The axes whose extent is close to the largest are all tried and
//! the smallest separator wins. Both halves are ordered first (recursively), the separator last.
//! Subsets of at most [`LEAF_ROWS`] rows keep their input order.
//!
//! Rows with equal coordinates (the DOFs of one node) always land on the same side, so a vector
//! problem is dissected node by node. On a structured `m³` grid this reproduces the classic
//! plane-separator dissection. The result is deterministic: it depends only on the pattern and
//! the coordinates, not on the thread count.

use std::sync::atomic::{AtomicU32, Ordering as AtomicOrdering};

use super::SymCsr;
use crate::{ComputeError, ComputeResult};

/// Subsets with at most this many rows are not dissected further.
pub(crate) const LEAF_ROWS: usize = 192;
/// An axis is tried when its extent is at least this fraction of the largest one.
const AXIS_RATIO: f64 = 0.7;
/// Subsets larger than this are dissected in parallel (rayon `join`).
const PAR_ROWS: usize = 20_000;

/// Label of rows that are already ordered (in a separator of an enclosing subset).
const DONE: u32 = 0;

/// The full (both triangles, no diagonal) adjacency of the pattern of `a`.
struct Graph {
    ptr: Vec<usize>,
    adj: Vec<u32>,
}

impl Graph {
    fn new(a: &SymCsr) -> Self {
        let n = a.n();
        let (rp, cols) = (a.row_ptr(), a.cols());
        let mut deg = vec![0usize; n];
        for i in 0..n {
            for &j in &cols[rp[i]..rp[i + 1]] {
                let j = j as usize;
                if j != i {
                    deg[i] += 1;
                    deg[j] += 1;
                }
            }
        }
        let mut ptr = Vec::with_capacity(n + 1);
        ptr.push(0usize);
        for d in &deg {
            ptr.push(ptr[ptr.len() - 1] + d);
        }
        let mut next: Vec<usize> = ptr[..n].to_vec();
        let mut adj = vec![0u32; ptr[n]];
        for i in 0..n {
            for &j in &cols[rp[i]..rp[i + 1]] {
                let ju = j as usize;
                if ju != i {
                    adj[next[i]] = j;
                    next[i] += 1;
                    adj[next[ju]] = i as u32;
                    next[ju] += 1;
                }
            }
        }
        Self { ptr, adj }
    }

    fn neighbours(&self, v: u32) -> &[u32] {
        let v = v as usize;
        &self.adj[self.ptr[v]..self.ptr[v + 1]]
    }
}

struct Ctx<'a> {
    graph: Graph,
    coords: &'a [[f64; 3]],
    /// Subset id of every row; [`DONE`] once a row is ordered.
    label: Vec<AtomicU32>,
    next_id: AtomicU32,
}

impl Ctx<'_> {
    fn fresh_id(&self) -> u32 {
        self.next_id.fetch_add(1, AtomicOrdering::Relaxed)
    }

    fn label(&self, v: u32) -> u32 {
        self.label[v as usize].load(AtomicOrdering::Relaxed)
    }

    fn set_label(&self, vs: &[u32], id: u32) {
        for &v in vs {
            self.label[v as usize].store(id, AtomicOrdering::Relaxed);
        }
    }
}

/// A cut of a subset: rows with `coord[axis] < split` (or `<=` when `inclusive`) are side A.
#[derive(Clone, Copy)]
struct Cut {
    axis: usize,
    split: f64,
    inclusive: bool,
}

impl Cut {
    fn side_a(&self, c: &[f64; 3]) -> bool {
        if self.inclusive {
            c[self.axis] <= self.split
        } else {
            c[self.axis] < self.split
        }
    }
}

/// Nested-dissection permutation (`perm[new] = old`) of `a` from per-row coordinates.
///
/// # Errors
/// [`ComputeError::InvalidMatrix`] when `coords` does not have one entry per row or holds a
/// non-finite value.
pub fn nested_dissection(a: &SymCsr, coords: &[[f64; 3]]) -> ComputeResult<Vec<u32>> {
    let n = a.n();
    if coords.len() != n {
        return Err(ComputeError::InvalidMatrix(format!(
            "nested dissection has {} coordinates, expected one per row ({n})",
            coords.len()
        )));
    }
    if coords.iter().flatten().any(|c| !c.is_finite()) {
        return Err(ComputeError::InvalidMatrix(
            "nested dissection coordinates must be finite".into(),
        ));
    }
    let ctx = Ctx {
        graph: Graph::new(a),
        coords,
        label: (0..n).map(|_| AtomicU32::new(1)).collect(),
        next_id: AtomicU32::new(2),
    };
    let rows: Vec<u32> = (0..n as u32).collect();
    let mut out = vec![0u32; n];
    dissect(&ctx, rows, 1, &mut out);
    Ok(out)
}

/// Orders `rows` (all labelled `id`) into `out` (same length).
fn dissect(ctx: &Ctx<'_>, rows: Vec<u32>, id: u32, out: &mut [u32]) {
    debug_assert_eq!(rows.len(), out.len());
    if rows.len() <= LEAF_ROWS {
        out.copy_from_slice(&rows);
        ctx.set_label(&rows, DONE);
        return;
    }
    let Some((a, b, sep)) = best_split(ctx, &rows, id) else {
        // All rows share one point (or no axis splits them): keep the input order.
        out.copy_from_slice(&rows);
        ctx.set_label(&rows, DONE);
        return;
    };
    drop(rows);
    ctx.set_label(&sep, DONE);
    let (id_a, id_b) = (ctx.fresh_id(), ctx.fresh_id());
    ctx.set_label(&a, id_a);
    ctx.set_label(&b, id_b);
    let (out_a, rest) = out.split_at_mut(a.len());
    let (out_b, out_sep) = rest.split_at_mut(b.len());
    out_sep.copy_from_slice(&sep);
    drop(sep);
    if a.len() + b.len() > PAR_ROWS {
        rayon::join(
            || dissect(ctx, a, id_a, out_a),
            || dissect(ctx, b, id_b, out_b),
        );
    } else {
        dissect(ctx, a, id_a, out_a);
        dissect(ctx, b, id_b, out_b);
    }
}

/// The cut with the smallest separator over the candidate axes: `(A \ sep, B \ sep, sep)`, in
/// input order. `None` when no axis splits the subset.
fn best_split(ctx: &Ctx<'_>, rows: &[u32], id: u32) -> Option<(Vec<u32>, Vec<u32>, Vec<u32>)> {
    let coords = ctx.coords;
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for &v in rows {
        let c = &coords[v as usize];
        for ax in 0..3 {
            lo[ax] = lo[ax].min(c[ax]);
            hi[ax] = hi[ax].max(c[ax]);
        }
    }
    let ext = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    let max_ext = ext[0].max(ext[1]).max(ext[2]);
    if max_ext <= 0.0 {
        return None;
    }
    // Longest axis first; ties go to the lower axis index.
    let mut axes = [0usize, 1, 2];
    axes.sort_by(|&x, &y| ext[y].total_cmp(&ext[x]).then(x.cmp(&y)));

    let mut best: Option<(Cut, bool, usize)> = None; // (cut, separator on side A, size)
    let mut vals: Vec<f64> = Vec::with_capacity(rows.len());
    for &axis in &axes {
        if ext[axis] <= 0.0 || ext[axis] < AXIS_RATIO * max_ext {
            continue;
        }
        let Some(cut) = median_cut(coords, rows, axis, &mut vals) else {
            continue;
        };
        let (sep_a, sep_b) = boundary_sizes(ctx, rows, id, cut);
        let (on_a, size) = if sep_a < sep_b {
            (true, sep_a)
        } else {
            (false, sep_b)
        };
        if best.is_none_or(|(_, _, s)| size < s) {
            best = Some((cut, on_a, size));
        }
    }
    let (cut, on_a, _) = best?;

    let (mut a, mut b, mut sep) = (Vec::new(), Vec::new(), Vec::new());
    for &v in rows {
        let in_a = cut.side_a(&coords[v as usize]);
        if in_a == on_a && touches_other_side(ctx, v, id, cut, in_a) {
            sep.push(v);
        } else if in_a {
            a.push(v);
        } else {
            b.push(v);
        }
    }
    Some((a, b, sep))
}

/// The median cut of `rows` along `axis`, or `None` if all rows have one coordinate there.
fn median_cut(coords: &[[f64; 3]], rows: &[u32], axis: usize, vals: &mut Vec<f64>) -> Option<Cut> {
    vals.clear();
    vals.extend(rows.iter().map(|&v| coords[v as usize][axis]));
    let mid = vals.len() / 2;
    let (_, &mut split, _) = vals.select_nth_unstable_by(mid, f64::total_cmp);
    let below = rows
        .iter()
        .filter(|&&v| coords[v as usize][axis] < split)
        .count();
    let cut = if below > 0 {
        Cut {
            axis,
            split,
            inclusive: false,
        }
    } else {
        // The median is the minimum: put the rows at the minimum on side A.
        let at_or_below = rows
            .iter()
            .filter(|&&v| coords[v as usize][axis] <= split)
            .count();
        if at_or_below == rows.len() {
            return None;
        }
        Cut {
            axis,
            split,
            inclusive: true,
        }
    };
    Some(cut)
}

/// Whether `v` (in the subset `id`, on side `in_a` of `cut`) has a neighbour in the subset on
/// the other side.
fn touches_other_side(ctx: &Ctx<'_>, v: u32, id: u32, cut: Cut, in_a: bool) -> bool {
    ctx.graph
        .neighbours(v)
        .iter()
        .any(|&u| ctx.label(u) == id && cut.side_a(&ctx.coords[u as usize]) != in_a)
}

/// Sizes of the two one-sided boundaries of `cut` (rows of A touching B, rows of B touching A).
fn boundary_sizes(ctx: &Ctx<'_>, rows: &[u32], id: u32, cut: Cut) -> (usize, usize) {
    let (mut sa, mut sb) = (0, 0);
    for &v in rows {
        let in_a = cut.side_a(&ctx.coords[v as usize]);
        if touches_other_side(ctx, v, id, cut, in_a) {
            if in_a {
                sa += 1;
            } else {
                sb += 1;
            }
        }
    }
    (sa, sb)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, reason = "tests")]

    use super::*;
    use crate::sparse::TripletBuilder;

    fn path(n: usize) -> (SymCsr, Vec<[f64; 3]>) {
        let mut t = TripletBuilder::new(n);
        for i in 0..n {
            t.push(i, i, 2.0);
            if i + 1 < n {
                t.push(i, i + 1, -1.0);
            }
        }
        let coords = (0..n).map(|i| [i as f64, 0.0, 0.0]).collect();
        (t.build().unwrap(), coords)
    }

    fn is_permutation(p: &[u32], n: usize) -> bool {
        let mut seen = vec![false; n];
        p.len() == n
            && p.iter()
                .all(|&v| (v as usize) < n && !std::mem::replace(&mut seen[v as usize], true))
    }

    #[test]
    fn path_separator_is_last() {
        let n = 4 * LEAF_ROWS + 1;
        let (a, coords) = path(n);
        let p = nested_dissection(&a, &coords).unwrap();
        assert!(is_permutation(&p, n));
        // The top-level separator of a path is a single vertex near the middle.
        let last = p[n - 1] as usize;
        assert!(last.abs_diff(n / 2) <= 1, "last = {last}");
    }

    #[test]
    fn small_and_degenerate_inputs_keep_input_order() {
        let (a, coords) = path(10);
        assert_eq!(
            nested_dissection(&a, &coords).unwrap(),
            (0..10).collect::<Vec<_>>()
        );
        let n = 2 * LEAF_ROWS;
        let (a, _) = path(n);
        let same = vec![[1.0, 2.0, 3.0]; n];
        assert_eq!(
            nested_dissection(&a, &same).unwrap(),
            (0..n as u32).collect::<Vec<_>>()
        );
    }

    #[test]
    fn rejects_bad_coordinates() {
        let (a, mut coords) = path(5);
        assert!(matches!(
            nested_dissection(&a, &coords[..4]),
            Err(ComputeError::InvalidMatrix(_))
        ));
        coords[2][1] = f64::NAN;
        assert!(matches!(
            nested_dissection(&a, &coords),
            Err(ComputeError::InvalidMatrix(_))
        ));
    }
}
