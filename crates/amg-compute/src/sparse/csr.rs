//! [`SymCsr`] (symmetric, upper triangle in CSR) and [`TripletBuilder`].

use std::sync::{Arc, OnceLock};

use rayon::prelude::*;

use crate::{ComputeError, ComputeResult};

/// Rows per rayon task in [`SymCsr::spmv`]. Each row is summed sequentially in a fixed order,
/// so the chunking only affects scheduling, never the result.
const SPMV_ROWS_PER_TASK: usize = 1024;

/// Symmetric matrix, upper triangle (diagonal included) in CSR, columns sorted per row.
///
/// Row `i` holds the entries `a_ij` with `j >= i`. Read as CSC, the same arrays are the lower
/// triangle of `A`, which is how the factorisations hand it to faer.
#[derive(Clone, Debug)]
pub struct SymCsr {
    n: usize,
    row_ptr: Vec<usize>,
    cols: Vec<u32>,
    vals: Vec<f64>,
    /// The strictly lower triangle by rows (the transpose of the strictly upper part), built on
    /// the first [`SymCsr::spmv`]. Depends on the pattern only, so [`SymCsr::vals_mut`] keeps it.
    lower: OnceLock<Arc<Lower>>,
}

/// Strictly lower triangle in CSR: for row `i`, the columns `j < i` and the position of
/// `a_ji` in [`SymCsr::vals`].
#[derive(Debug)]
struct Lower {
    row_ptr: Vec<usize>,
    cols: Vec<u32>,
    src: Vec<usize>,
}

impl PartialEq for SymCsr {
    fn eq(&self, other: &Self) -> bool {
        self.n == other.n
            && self.row_ptr == other.row_ptr
            && self.cols == other.cols
            && self.vals == other.vals
    }
}

impl SymCsr {
    /// Builds a matrix from raw upper-CSR arrays.
    ///
    /// # Errors
    /// [`ComputeError::InvalidMatrix`] unless: `row_ptr` has `n + 1` entries, starts at 0, is
    /// non-decreasing and ends at `cols.len() == vals.len()`; the columns of each row are
    /// strictly increasing (sorted, unique), `>= row` and `< n`; `n` fits in `u32`.
    pub fn from_raw(
        n: usize,
        row_ptr: Vec<usize>,
        cols: Vec<u32>,
        vals: Vec<f64>,
    ) -> ComputeResult<Self> {
        let bad = |msg: String| Err(ComputeError::InvalidMatrix(msg));
        if u32::try_from(n).is_err() {
            return bad(format!("{n} rows do not fit in u32 column indices"));
        }
        if row_ptr.len() != n + 1 {
            return bad(format!(
                "row_ptr has {} entries, expected n + 1 = {}",
                row_ptr.len(),
                n + 1
            ));
        }
        if row_ptr[0] != 0 {
            return bad(format!("row_ptr[0] is {}, expected 0", row_ptr[0]));
        }
        if cols.len() != vals.len() {
            return bad(format!(
                "{} column indices but {} values",
                cols.len(),
                vals.len()
            ));
        }
        if row_ptr[n] != cols.len() {
            return bad(format!(
                "row_ptr[n] is {}, expected the entry count {}",
                row_ptr[n],
                cols.len()
            ));
        }
        for i in 0..n {
            let (start, end) = (row_ptr[i], row_ptr[i + 1]);
            if end < start || end > cols.len() {
                return bad(format!("row_ptr is not non-decreasing at row {i}"));
            }
            let row = &cols[start..end];
            for (k, &c) in row.iter().enumerate() {
                let c = c as usize;
                if c >= n {
                    return bad(format!("column {c} in row {i} is out of range (n = {n})"));
                }
                if c < i {
                    return bad(format!(
                        "column {c} in row {i} is below the diagonal (upper triangle expected)"
                    ));
                }
                if k > 0 && row[k - 1] as usize >= c {
                    return bad(format!("columns of row {i} are not sorted and unique"));
                }
            }
        }
        Ok(Self {
            n,
            row_ptr,
            cols,
            vals,
            lower: OnceLock::new(),
        })
    }

    /// Number of rows (and columns).
    pub fn n(&self) -> usize {
        self.n
    }

    /// Number of stored (upper) entries.
    pub fn nnz(&self) -> usize {
        self.cols.len()
    }

    /// Row offsets, `n + 1` entries.
    pub fn row_ptr(&self) -> &[usize] {
        &self.row_ptr
    }

    /// Column indices, sorted within each row.
    pub fn cols(&self) -> &[u32] {
        &self.cols
    }

    /// Values, in the order of [`SymCsr::cols`].
    pub fn vals(&self) -> &[f64] {
        &self.vals
    }

    /// Mutable values, to refill a fixed pattern (Newton steps, shifts).
    pub fn vals_mut(&mut self) -> &mut [f64] {
        &mut self.vals
    }

    /// The diagonal; 0 where the diagonal entry is not stored.
    pub fn diagonal(&self) -> Vec<f64> {
        (0..self.n)
            .map(|i| {
                let k = self.row_ptr[i];
                if k < self.row_ptr[i + 1] && self.cols[k] as usize == i {
                    self.vals[k]
                } else {
                    0.0
                }
            })
            .collect()
    }

    /// The value at `(i, j)` (either triangle); 0 if not stored.
    pub fn get(&self, i: usize, j: usize) -> f64 {
        let (r, c) = if i <= j { (i, j) } else { (j, i) };
        if r >= self.n {
            return 0.0;
        }
        let range = self.row_ptr[r]..self.row_ptr[r + 1];
        match self.cols[range.clone()].binary_search(&(c as u32)) {
            Ok(k) => self.vals[range.start + k],
            Err(_) => 0.0,
        }
    }

    fn lower(&self) -> &Lower {
        self.lower.get_or_init(|| Arc::new(self.build_lower()))
    }

    fn build_lower(&self) -> Lower {
        let n = self.n;
        let mut counts = vec![0usize; n + 1];
        for i in 0..n {
            for &c in &self.cols[self.row_ptr[i]..self.row_ptr[i + 1]] {
                if c as usize != i {
                    counts[c as usize + 1] += 1;
                }
            }
        }
        for i in 0..n {
            counts[i + 1] += counts[i];
        }
        let row_ptr = counts;
        let total = row_ptr[n];
        let mut next = row_ptr.clone();
        let mut cols = vec![0u32; total];
        let mut src = vec![0usize; total];
        // Rows i ascending, so the columns of every lower row come out sorted.
        for i in 0..n {
            for k in self.row_ptr[i]..self.row_ptr[i + 1] {
                let c = self.cols[k] as usize;
                if c != i {
                    let slot = next[c];
                    next[c] += 1;
                    cols[slot] = i as u32;
                    src[slot] = k;
                }
            }
        }
        Lower { row_ptr, cols, src }
    }

    /// `y = A·x` on the rayon pool.
    ///
    /// Deterministic: each `y_i` is summed sequentially over the columns of row `i` of the full
    /// matrix in increasing order, so the result is bit-identical for any thread count. The
    /// first call builds the lower-triangle index (O(nnz) memory, kept across calls).
    ///
    /// # Panics
    /// If `x` or `y` does not have `n` entries.
    pub fn spmv(&self, x: &[f64], y: &mut [f64]) {
        assert_eq!(x.len(), self.n, "spmv: x has the wrong length");
        assert_eq!(y.len(), self.n, "spmv: y has the wrong length");
        let lower = self.lower();
        y.par_chunks_mut(SPMV_ROWS_PER_TASK)
            .enumerate()
            .for_each(|(chunk, ys)| {
                let base = chunk * SPMV_ROWS_PER_TASK;
                for (r, yi) in ys.iter_mut().enumerate() {
                    let i = base + r;
                    let mut acc = 0.0;
                    for k in lower.row_ptr[i]..lower.row_ptr[i + 1] {
                        acc += self.vals[lower.src[k]] * x[lower.cols[k] as usize];
                    }
                    for k in self.row_ptr[i]..self.row_ptr[i + 1] {
                        acc += self.vals[k] * x[self.cols[k] as usize];
                    }
                    *yi = acc;
                }
            });
    }

    /// `self + alpha·other` on the union of the two patterns (e.g. `K − σM`).
    ///
    /// # Errors
    /// [`ComputeError::Shape`] if the sizes differ.
    pub fn add_scaled(&self, alpha: f64, other: &SymCsr) -> ComputeResult<SymCsr> {
        if self.n != other.n {
            return Err(ComputeError::Shape(format!(
                "add_scaled of a {0}×{0} and a {1}×{1} matrix",
                self.n, other.n
            )));
        }
        let n = self.n;
        let mut row_ptr = Vec::with_capacity(n + 1);
        row_ptr.push(0);
        let cap = self.nnz().max(other.nnz());
        let mut cols = Vec::with_capacity(cap);
        let mut vals = Vec::with_capacity(cap);
        for i in 0..n {
            let (mut a, a_end) = (self.row_ptr[i], self.row_ptr[i + 1]);
            let (mut b, b_end) = (other.row_ptr[i], other.row_ptr[i + 1]);
            while a < a_end || b < b_end {
                let ca = if a < a_end { self.cols[a] } else { u32::MAX };
                let cb = if b < b_end { other.cols[b] } else { u32::MAX };
                if ca == cb {
                    cols.push(ca);
                    vals.push(self.vals[a] + alpha * other.vals[b]);
                    a += 1;
                    b += 1;
                } else if ca < cb {
                    cols.push(ca);
                    vals.push(self.vals[a]);
                    a += 1;
                } else {
                    cols.push(cb);
                    vals.push(alpha * other.vals[b]);
                    b += 1;
                }
            }
            row_ptr.push(cols.len());
        }
        Ok(SymCsr {
            n,
            row_ptr,
            cols,
            vals,
            lower: OnceLock::new(),
        })
    }

    /// True if both matrices have the same size and the same stored pattern.
    pub fn same_pattern(&self, other: &SymCsr) -> bool {
        self.n == other.n && self.row_ptr == other.row_ptr && self.cols == other.cols
    }
}

/// Small/test assembly: sums duplicates; an entry `(i, j)` with `j < i` is stored as `(j, i)`.
///
/// Because entries are swapped into the upper triangle, push each off-diagonal pair once (either
/// `(i, j)` or `(j, i)`); pushing both doubles it.
#[derive(Clone, Debug, Default)]
pub struct TripletBuilder {
    n: usize,
    entries: Vec<(u32, u32, f64)>,
    out_of_range: Option<(usize, usize)>,
}

impl TripletBuilder {
    /// An empty `n × n` builder.
    pub fn new(n: usize) -> Self {
        Self {
            n,
            entries: Vec::new(),
            out_of_range: None,
        }
    }

    /// Adds `v` at `(i, j)`; out-of-range indices make [`TripletBuilder::build`] fail.
    pub fn push(&mut self, i: usize, j: usize, v: f64) {
        let (r, c) = if i <= j { (i, j) } else { (j, i) };
        match (u32::try_from(r), u32::try_from(c)) {
            (Ok(r32), Ok(c32)) if c < self.n => self.entries.push((r32, c32, v)),
            _ => {
                self.out_of_range.get_or_insert((i, j));
            }
        }
    }

    /// Sorts, sums duplicates and builds the matrix.
    ///
    /// # Errors
    /// [`ComputeError::InvalidMatrix`] if an entry was out of range or `n` does not fit in u32.
    pub fn build(self) -> ComputeResult<SymCsr> {
        if let Some((i, j)) = self.out_of_range {
            return Err(ComputeError::InvalidMatrix(format!(
                "entry ({i}, {j}) is out of range (n = {})",
                self.n
            )));
        }
        let n = self.n;
        let mut entries = self.entries;
        // Stable, so duplicates are summed in push order (deterministic).
        entries.sort_by_key(|&(r, c, _)| (r, c));
        let mut row_ptr = vec![0usize; n + 1];
        let mut cols: Vec<u32> = Vec::with_capacity(entries.len());
        let mut vals: Vec<f64> = Vec::with_capacity(entries.len());
        let mut last: Option<(u32, u32)> = None;
        for (r, c, v) in entries {
            if last == Some((r, c)) {
                if let Some(x) = vals.last_mut() {
                    *x += v;
                }
            } else {
                cols.push(c);
                vals.push(v);
                row_ptr[r as usize + 1] += 1;
                last = Some((r, c));
            }
        }
        for i in 0..n {
            row_ptr[i + 1] += row_ptr[i];
        }
        SymCsr::from_raw(n, row_ptr, cols, vals)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dense_upper(a: &SymCsr) -> Vec<Vec<f64>> {
        let n = a.n();
        (0..n)
            .map(|i| (0..n).map(|j| a.get(i, j)).collect())
            .collect()
    }

    #[test]
    fn triplets_sum_duplicates_and_swap_to_upper() {
        let mut t = TripletBuilder::new(3);
        t.push(0, 0, 1.0);
        t.push(0, 0, 2.0);
        t.push(2, 0, 5.0); // stored as (0, 2)
        t.push(1, 1, 4.0);
        t.push(2, 2, 6.0);
        let a = t.build().unwrap();
        assert_eq!(a.row_ptr(), &[0, 2, 3, 4]);
        assert_eq!(a.cols(), &[0, 2, 1, 2]);
        assert_eq!(a.vals(), &[3.0, 5.0, 4.0, 6.0]);
        assert_eq!(a.diagonal(), vec![3.0, 4.0, 6.0]);
        assert_eq!(a.get(2, 0).to_bits(), 5.0f64.to_bits());
    }

    #[test]
    fn triplet_out_of_range_is_invalid() {
        let mut t = TripletBuilder::new(2);
        t.push(0, 2, 1.0);
        assert!(matches!(t.build(), Err(ComputeError::InvalidMatrix(_))));
    }

    #[test]
    fn spmv_matches_dense() {
        let mut t = TripletBuilder::new(4);
        for (i, j, v) in [
            (0, 0, 4.0),
            (0, 1, -1.0),
            (1, 1, 4.0),
            (1, 3, 0.5),
            (2, 2, 3.0),
            (0, 3, 2.0),
            (3, 3, 5.0),
        ] {
            t.push(i, j, v);
        }
        let a = t.build().unwrap();
        let d = dense_upper(&a);
        let x = [1.0, -2.0, 3.0, 0.25];
        let mut y = [0.0; 4];
        a.spmv(&x, &mut y);
        for i in 0..4 {
            let e: f64 = (0..4).map(|j| d[i][j] * x[j]).sum();
            assert!((y[i] - e).abs() < 1e-14, "row {i}: {} vs {e}", y[i]);
        }
    }

    #[test]
    fn add_scaled_unions_patterns() {
        let mut t = TripletBuilder::new(3);
        t.push(0, 0, 1.0);
        t.push(0, 2, 1.0);
        let a = t.build().unwrap();
        let mut t = TripletBuilder::new(3);
        t.push(0, 0, 1.0);
        t.push(1, 2, 2.0);
        t.push(2, 2, 1.0);
        let b = t.build().unwrap();
        let c = a.add_scaled(-2.0, &b).unwrap();
        assert_eq!(c.cols(), &[0, 2, 2, 2]);
        assert_eq!(c.vals(), &[-1.0, 1.0, -4.0, -2.0]);
        assert!(!c.same_pattern(&a));
        assert!(c.same_pattern(&c.clone()));
        let wrong = TripletBuilder::new(2).build().unwrap();
        assert!(matches!(
            a.add_scaled(1.0, &wrong),
            Err(ComputeError::Shape(_))
        ));
    }

    #[test]
    fn from_raw_rejects_bad_input() {
        let ok = SymCsr::from_raw(2, vec![0, 2, 3], vec![0, 1, 1], vec![1.0, 0.5, 1.0]);
        assert!(ok.is_ok());
        type Raw = (usize, Vec<usize>, Vec<u32>, Vec<f64>);
        let cases: [Raw; 7] = [
            // unsorted
            (2, vec![0, 2, 3], vec![1, 0, 1], vec![1.0, 1.0, 1.0]),
            // duplicate
            (2, vec![0, 2, 3], vec![1, 1, 1], vec![1.0, 1.0, 1.0]),
            // lower
            (2, vec![0, 1, 3], vec![0, 0, 1], vec![1.0, 1.0, 1.0]),
            // out of range
            (2, vec![0, 1, 2], vec![0, 2], vec![1.0, 1.0]),
            // row_ptr length
            (2, vec![0, 1], vec![0], vec![1.0]),
            // decreasing row_ptr
            (2, vec![0, 2, 1], vec![0, 1], vec![1.0, 1.0]),
            // value count
            (2, vec![0, 1, 2], vec![0, 1], vec![1.0]),
        ];
        for (k, (n, rp, c, v)) in cases.into_iter().enumerate() {
            assert!(
                matches!(
                    SymCsr::from_raw(n, rp, c, v),
                    Err(ComputeError::InvalidMatrix(_))
                ),
                "case {k}"
            );
        }
    }
}
