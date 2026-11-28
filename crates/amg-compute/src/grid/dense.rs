//! `DenseGrid<T>`: a dense box of values, x fastest.

/// Dense grid, x fastest. Cell- or node-centred by the caller's convention.
#[derive(Clone, Debug, PartialEq)]
pub struct DenseGrid<T> {
    pub dims: [u32; 3],
    pub data: Vec<T>,
}

impl<T: Copy + Default> DenseGrid<T> {
    /// A grid of `T::default()`.
    pub fn new(dims: [u32; 3]) -> Self {
        let n = dims.iter().map(|&d| d as usize).product();
        DenseGrid {
            dims,
            data: vec![T::default(); n],
        }
    }

    /// A grid with `data[index(ijk)] = f(ijk)`.
    pub fn from_fn(dims: [u32; 3], mut f: impl FnMut([u32; 3]) -> T) -> Self {
        let mut data = Vec::with_capacity(dims.iter().map(|&d| d as usize).product());
        for z in 0..dims[2] {
            for y in 0..dims[1] {
                for x in 0..dims[0] {
                    data.push(f([x, y, z]));
                }
            }
        }
        DenseGrid { dims, data }
    }

    /// `x + dims.x (y + dims.y z)`. Debug builds check the bounds.
    pub fn index(&self, ijk: [u32; 3]) -> usize {
        debug_assert!(
            (0..3).all(|a| ijk[a] < self.dims[a]),
            "{ijk:?} outside {:?}",
            self.dims
        );
        let [dx, dy, _] = self.dims.map(|d| d as usize);
        ijk[0] as usize + dx * (ijk[1] as usize + dy * ijk[2] as usize)
    }

    pub fn get(&self, ijk: [u32; 3]) -> T {
        self.data[self.index(ijk)]
    }

    pub fn set(&mut self, ijk: [u32; 3], v: T) {
        let i = self.index(ijk);
        self.data[i] = v;
    }

    /// Number of values.
    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
}

impl DenseGrid<f32> {
    /// Trilinear interpolation at `p` in sample units (`data[index(ijk)]` at `p = ijk`),
    /// clamped to the grid (a point outside reads the nearest boundary value). An empty grid
    /// gives 0.
    pub fn trilinear(&self, p: [f64; 3]) -> f64 {
        if self.data.is_empty() {
            return 0.0;
        }
        let mut i0 = [0u32; 3];
        let mut i1 = [0u32; 3];
        let mut t = [0.0f64; 3];
        for a in 0..3 {
            let hi = f64::from(self.dims[a] - 1);
            let q = if p[a].is_nan() {
                0.0
            } else {
                p[a].clamp(0.0, hi)
            };
            let f = q.floor();
            i0[a] = f as u32;
            i1[a] = (i0[a] + 1).min(self.dims[a] - 1);
            t[a] = q - f;
        }
        let mut c = [0.0f64; 8];
        for (k, ck) in c.iter_mut().enumerate() {
            let pick = |a: usize| if (k >> a) & 1 == 0 { i0[a] } else { i1[a] };
            *ck = f64::from(self.get([pick(0), pick(1), pick(2)]));
        }
        super::brick::lerp8(c, t)
    }
}

#[cfg(test)]
#[allow(
    clippy::float_cmp,
    reason = "exact values are the point of these tests"
)]
mod tests {
    use super::*;

    #[test]
    fn index_get_set() {
        let mut g = DenseGrid::<u8>::new([3, 4, 5]);
        assert_eq!(g.len(), 60);
        assert_eq!(g.index([2, 3, 4]), 59);
        assert_eq!(g.index([1, 2, 0]), 7);
        g.set([1, 2, 3], 9);
        assert_eq!(g.get([1, 2, 3]), 9);
        assert_eq!(g.data[g.index([1, 2, 3])], 9);
    }

    #[test]
    fn trilinear_is_exact_on_linear_fields_and_clamps() {
        let g = DenseGrid::from_fn([4, 5, 6], |[x, y, z]| x as f32 + 2.0 * y as f32 - z as f32);
        for p in [[0.5, 0.5, 0.5], [2.9, 3.1, 4.75], [3.0, 4.0, 5.0]] {
            let want = p[0] + 2.0 * p[1] - p[2];
            assert!((g.trilinear(p) - want).abs() < 1e-9, "{p:?}");
        }
        assert!((g.trilinear([-3.0, 10.0, 2.0]) - (0.0 + 8.0 - 2.0)).abs() < 1e-9);
        assert_eq!(DenseGrid::<f32>::new([0, 1, 1]).trilinear([0.0; 3]), 0.0);
    }
}
