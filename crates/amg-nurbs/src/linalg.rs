//! Small banded linear solver for the fitting systems (KH-13).
//!
//! Interpolation (A9.1) and least-squares normal equations (§9.4) are banded with bandwidth
//! about `p`, so one Gaussian elimination with partial pivoting on a band does all of them; a
//! dense system is the band with `kl = ku = n − 1` (closed interpolation). The right-hand side
//! is a slice of control points, so every coordinate is solved in one pass.

use crate::{CtrlPt, NurbsError};

/// Square `n × n` matrix with lower bandwidth `kl` and upper bandwidth `ku`.
///
/// Row `i` stores columns `i − kl ..= i + kl + ku` (the extra `kl` columns hold the fill-in
/// that row swaps create).
#[derive(Debug, Clone)]
pub(crate) struct Band {
    n: usize,
    kl: usize,
    ku: usize,
    w: usize,
    a: Vec<f64>,
}

impl Band {
    pub(crate) fn new(n: usize, kl: usize, ku: usize) -> Band {
        let kl = kl.min(n.saturating_sub(1));
        let ku = ku.min(n.saturating_sub(1));
        let w = 2 * kl + ku + 1;
        Band {
            n,
            kl,
            ku,
            w,
            a: vec![0.0; n * w],
        }
    }

    #[inline]
    fn idx(&self, i: usize, j: usize) -> usize {
        debug_assert!(
            j + self.kl >= i && j <= i + self.kl + self.ku,
            "({i}, {j}) off band"
        );
        i * self.w + (j + self.kl - i)
    }

    /// `A[i][j] += v`. `(i, j)` must lie in the band (`i − kl <= j <= i + ku`).
    #[inline]
    pub(crate) fn add(&mut self, i: usize, j: usize, v: f64) {
        let k = self.idx(i, j);
        self.a[k] += v;
    }

    /// Solves `A x = rhs` in place (Gaussian elimination with partial pivoting on the band).
    /// `Singular(what)` if a pivot falls below `1e-14` of the largest entry.
    pub(crate) fn solve<P: CtrlPt>(
        mut self,
        rhs: &mut [P],
        what: &'static str,
    ) -> Result<(), NurbsError> {
        let n = self.n;
        if rhs.len() != n {
            return Err(NurbsError::Singular(what));
        }
        if n == 0 {
            return Ok(());
        }
        let amax = self.a.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        if !(amax.is_finite() && amax > 0.0) {
            return Err(NurbsError::Singular(what));
        }
        let tiny = amax * 1e-14;
        let (kl, ku) = (self.kl, self.ku);
        for k in 0..n {
            let last_row = (k + kl).min(n - 1);
            let last_col = (k + kl + ku).min(n - 1);
            // Pivot: largest |A[r][k]|, lowest index on ties.
            let mut r = k;
            let mut best = self.a[self.idx(k, k)].abs();
            for i in k + 1..=last_row {
                let v = self.a[self.idx(i, k)].abs();
                if v > best {
                    best = v;
                    r = i;
                }
            }
            if best.is_nan() || best <= tiny {
                return Err(NurbsError::Singular(what));
            }
            if r != k {
                for j in k..=last_col {
                    let (x, y) = (self.idx(k, j), self.idx(r, j));
                    self.a.swap(x, y);
                }
                rhs.swap(k, r);
            }
            let piv = self.a[self.idx(k, k)];
            for i in k + 1..=last_row {
                let f = self.a[self.idx(i, k)] / piv;
                if f == 0.0 {
                    continue;
                }
                for j in k + 1..=last_col {
                    let v = self.a[self.idx(k, j)];
                    let t = self.idx(i, j);
                    self.a[t] -= f * v;
                }
                rhs[i] = rhs[i] - rhs[k] * f;
            }
        }
        for k in (0..n).rev() {
            let last_col = (k + kl + ku).min(n - 1);
            let s = rhs[k + 1..=last_col]
                .iter()
                .enumerate()
                .fold(rhs[k], |s, (i, &x)| s - x * self.a[self.idx(k, k + 1 + i)]);
            rhs[k] = s * (1.0 / self.a[self.idx(k, k)]);
        }
        if rhs.iter().all(|p| p.is_finite_pt()) {
            Ok(())
        } else {
            Err(NurbsError::Singular(what))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::Point2;

    #[test]
    fn solves_tridiagonal_and_dense_with_pivoting() {
        // [[0, 1, 0], [1, 0, 1], [0, 1, 2]] needs a row swap at k = 0.
        let mut b = Band::new(3, 1, 1);
        b.add(0, 1, 1.0);
        b.add(1, 0, 1.0);
        b.add(1, 2, 1.0);
        b.add(2, 1, 1.0);
        b.add(2, 2, 2.0);
        // x = (1, 2, 3): rhs = (2, 4, 8)
        let mut rhs = vec![
            Point2::new(2.0, 0.0),
            Point2::new(4.0, 0.0),
            Point2::new(8.0, 0.0),
        ];
        b.solve(&mut rhs, "test").unwrap();
        for (x, want) in rhs.iter().zip([1.0, 2.0, 3.0]) {
            assert!((x.x - want).abs() < 1e-14, "{rhs:?}");
        }
        // Dense 4×4 (kl = ku = 3).
        let m = [
            [4.0, 1.0, 2.0, 0.5],
            [1.0, 0.0, 1.0, 3.0],
            [2.0, 1.0, 5.0, 1.0],
            [0.5, 3.0, 1.0, 0.1],
        ];
        let x = [1.0, -2.0, 0.5, 3.0];
        let mut b = Band::new(4, 3, 3);
        let mut rhs = vec![Point2::ZERO; 4];
        for i in 0..4 {
            for j in 0..4 {
                b.add(i, j, m[i][j]);
                rhs[i].y += m[i][j] * x[j];
            }
        }
        b.solve(&mut rhs, "test").unwrap();
        for i in 0..4 {
            assert!((rhs[i].y - x[i]).abs() < 1e-13, "{rhs:?}");
        }
    }

    #[test]
    fn singular_is_an_error() {
        let mut b = Band::new(2, 1, 1);
        b.add(0, 0, 1.0);
        b.add(0, 1, 1.0);
        b.add(1, 0, 1.0);
        b.add(1, 1, 1.0);
        let mut rhs = vec![Point2::ONE; 2];
        assert!(matches!(
            b.solve(&mut rhs, "t"),
            Err(NurbsError::Singular("t"))
        ));
        assert!(Band::new(1, 0, 0).solve(&mut [Point2::ONE], "t").is_err());
    }
}
