//! [`Grid`]: row-major 2D array of poles or weights.

use crate::NurbsError;

/// Row-major grid: `get(i, j)` with `i` along u (`0..nu`), `j` along v (`0..nv`), stored at
/// `i·nv + j`.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub struct Grid<T> {
    nu: usize,
    nv: usize,
    data: Vec<T>,
}

#[derive(serde::Deserialize)]
struct GridRaw<T> {
    nu: usize,
    nv: usize,
    data: Vec<T>,
}

impl<'de, T: Copy + serde::Deserialize<'de>> serde::Deserialize<'de> for Grid<T> {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let r = GridRaw::<T>::deserialize(d)?;
        Grid::new(r.nu, r.nv, r.data).map_err(serde::de::Error::custom)
    }
}

impl<T: Copy> Grid<T> {
    /// `data.len()` must equal `nu · nv`, and both sizes must be ≥ 1.
    pub fn new(nu: usize, nv: usize, data: Vec<T>) -> Result<Self, NurbsError> {
        if nu == 0 || nv == 0 {
            return Err(NurbsError::Degenerate("grid has an empty dimension"));
        }
        if nu.checked_mul(nv) != Some(data.len()) {
            return Err(NurbsError::Degenerate(
                "grid dimensions do not match the data length",
            ));
        }
        Ok(Grid { nu, nv, data })
    }

    /// Builds a grid from `f(i, j)`.
    pub(crate) fn from_fn(nu: usize, nv: usize, mut f: impl FnMut(usize, usize) -> T) -> Self {
        let mut data = Vec::with_capacity(nu * nv);
        for i in 0..nu {
            for j in 0..nv {
                data.push(f(i, j));
            }
        }
        Grid { nu, nv, data }
    }

    /// Size along u.
    #[inline]
    pub fn nu(&self) -> usize {
        self.nu
    }

    /// Size along v.
    #[inline]
    pub fn nv(&self) -> usize {
        self.nv
    }

    /// Element `(i, j)`. Panics if out of range (like slice indexing).
    #[inline]
    pub fn get(&self, i: usize, j: usize) -> T {
        self.data[i * self.nv + j]
    }

    /// Row `i` (fixed i, all j).
    #[inline]
    pub fn row(&self, i: usize) -> &[T] {
        &self.data[i * self.nv..(i + 1) * self.nv]
    }

    /// Column `j` (fixed j, all i).
    pub fn column(&self, j: usize) -> Vec<T> {
        (0..self.nu).map(|i| self.get(i, j)).collect()
    }

    /// The row-major data.
    #[inline]
    pub fn as_slice(&self) -> &[T] {
        &self.data
    }

    /// Element-wise map.
    pub(crate) fn map<U: Copy>(&self, f: impl FnMut(&T) -> U) -> Grid<U> {
        Grid {
            nu: self.nu,
            nv: self.nv,
            data: self.data.iter().map(f).collect(),
        }
    }

    /// Swaps u and v.
    pub(crate) fn transposed(&self) -> Grid<T> {
        Grid::from_fn(self.nv, self.nu, |i, j| self.get(j, i))
    }

    /// Reverses the order along u (`UV::U`) or v.
    pub(crate) fn reversed(&self, along_u: bool) -> Grid<T> {
        let (nu, nv) = (self.nu, self.nv);
        if along_u {
            Grid::from_fn(nu, nv, |i, j| self.get(nu - 1 - i, j))
        } else {
            Grid::from_fn(nu, nv, |i, j| self.get(i, nv - 1 - j))
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact expected values")]
mod tests {
    use super::*;

    #[test]
    fn layout() {
        let g = Grid::new(2, 3, vec![0, 1, 2, 10, 11, 12]).unwrap();
        assert_eq!(g.get(1, 2), 12);
        assert_eq!(g.row(1), &[10, 11, 12]);
        assert_eq!(g.column(1), vec![1, 11]);
        assert_eq!(g.transposed().get(2, 1), 12);
        assert_eq!(g.reversed(true).get(0, 0), 10);
        assert_eq!(g.reversed(false).get(0, 0), 2);
        assert!(Grid::new(2, 2, vec![0; 3]).is_err());
        assert!(Grid::new(0, 2, Vec::<i32>::new()).is_err());
        assert!(Grid::new(usize::MAX, 2, vec![0; 2]).is_err());
    }
}
