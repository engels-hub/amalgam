//! Deterministic random curves and surfaces for the amg-nurbs tests.
#![allow(dead_code, reason = "shared by several test binaries")]

use amg_nurbs::{Grid, KnotVector, NurbsCurve3, NurbsSurface3, Point3};

/// SplitMix64: tiny, deterministic, good enough for test data.
pub struct Rng(pub u64);

impl Rng {
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    /// Uniform in [0, 1).
    pub fn f(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.f()
    }
    pub fn usize(&mut self, a: usize, b_inclusive: usize) -> usize {
        a + (self.next_u64() % (b_inclusive - a + 1) as u64) as usize
    }
    pub fn point(&mut self, s: f64) -> Point3 {
        Point3::new(self.range(-s, s), self.range(-s, s), self.range(-s, s))
    }
}

/// Random clamped knot vector for `n` poles of degree `p` on a random domain, with some
/// repeated interior knots (multiplicity ≤ p).
pub fn knots(r: &mut Rng, p: usize, n: usize) -> KnotVector {
    let a = r.range(-5.0, 5.0);
    let len = r.range(0.5, 20.0);
    let interior = n - p - 1;
    let mut inner: Vec<f64> = Vec::with_capacity(interior);
    while inner.len() < interior {
        let k = a + len * r.range(0.02, 0.98);
        let reps = r.usize(1, p.min(interior - inner.len()).clamp(1, 2));
        for _ in 0..reps {
            inner.push(k);
        }
    }
    inner.sort_by(f64::total_cmp);
    let mut v = vec![a; p + 1];
    v.extend(inner);
    v.extend(std::iter::repeat_n(a + len, p + 1));
    KnotVector::new(v).unwrap()
}

pub fn curve(r: &mut Rng, p: usize, rational: bool) -> NurbsCurve3 {
    let n = r.usize(p + 1, p + 8);
    let k = knots(r, p, n);
    let poles = (0..n).map(|_| r.point(100.0)).collect();
    let w = rational.then(|| (0..n).map(|_| r.range(0.2, 5.0)).collect());
    NurbsCurve3::new(p, k, poles, w).unwrap()
}

pub fn surface(r: &mut Rng, p: usize, q: usize, rational: bool) -> NurbsSurface3 {
    let nu = r.usize(p + 1, p + 4);
    let nv = r.usize(q + 1, q + 4);
    let ku = knots(r, p, nu);
    let kv = knots(r, q, nv);
    let poles = Grid::new(nu, nv, (0..nu * nv).map(|_| r.point(100.0)).collect()).unwrap();
    let w = rational
        .then(|| Grid::new(nu, nv, (0..nu * nv).map(|_| r.range(0.2, 5.0)).collect()).unwrap());
    NurbsSurface3::new([p, q], [ku, kv], poles, w).unwrap()
}

/// Bounding-box diagonal of a point set (the `D` of the accuracy contract).
pub fn diag(pts: &[Point3]) -> f64 {
    amg_core::Aabb3::from_points(pts.iter().copied()).diagonal()
}

/// 1000 stratified parameters in `[a, b]` with a fixed-seed jitter, plus both ends.
pub fn samples(a: f64, b: f64, n: usize, seed: u64) -> Vec<f64> {
    let mut r = Rng(seed);
    let mut v: Vec<f64> = (0..n)
        .map(|i| a + (b - a) * ((i as f64 + r.f()) / n as f64))
        .collect();
    v[0] = a;
    v[n - 1] = b;
    v
}
