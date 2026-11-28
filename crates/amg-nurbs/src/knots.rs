//! [`KnotVector`]: validated, finite, non-decreasing knots.

use crate::{MAX_DEGREE, NurbsError};

/// Validated, finite, non-decreasing knot vector with a non-empty range
/// (`first < last`).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(try_from = "Vec<f64>", into = "Vec<f64>")]
pub struct KnotVector(Vec<f64>);

impl TryFrom<Vec<f64>> for KnotVector {
    type Error = NurbsError;
    fn try_from(v: Vec<f64>) -> Result<Self, NurbsError> {
        KnotVector::new(v)
    }
}

impl From<KnotVector> for Vec<f64> {
    fn from(k: KnotVector) -> Vec<f64> {
        k.0
    }
}

impl KnotVector {
    /// Validates `knots`: at least two values, all finite, non-decreasing, `first < last`.
    pub fn new(knots: Vec<f64>) -> Result<KnotVector, NurbsError> {
        if knots.len() < 2 {
            return Err(NurbsError::InvalidKnots("fewer than two knots"));
        }
        if knots.iter().any(|k| !k.is_finite()) {
            return Err(NurbsError::InvalidKnots("knot is not finite"));
        }
        if knots.windows(2).any(|w| w[1] < w[0]) {
            return Err(NurbsError::InvalidKnots("knots decrease"));
        }
        if knots[knots.len() - 1] <= knots[0] {
            return Err(NurbsError::InvalidKnots("knot range is empty"));
        }
        Ok(KnotVector(knots))
    }

    /// Crate-internal constructor for knot vectors that are valid by construction.
    pub(crate) fn new_unchecked(knots: Vec<f64>) -> KnotVector {
        debug_assert!(
            KnotVector::new(knots.clone()).is_ok(),
            "invalid knots {knots:?}"
        );
        KnotVector(knots)
    }

    /// Bézier knots on `[0, 1]`: `[0; p+1] ++ [1; p+1]`.
    pub fn bezier(degree: usize) -> KnotVector {
        let mut k = vec![0.0; degree + 1];
        k.resize(2 * (degree + 1), 1.0);
        KnotVector(k)
    }

    /// Clamped uniform knots on `[0, 1]` for `n_poles` poles of `degree`.
    pub fn clamped_uniform(degree: usize, n_poles: usize) -> Result<KnotVector, NurbsError> {
        if degree == 0 || degree > MAX_DEGREE {
            return Err(NurbsError::Degree(degree));
        }
        if n_poles <= degree {
            return Err(NurbsError::CountMismatch {
                degree,
                poles: n_poles,
                knots: n_poles + degree + 1,
            });
        }
        let spans = n_poles - degree;
        let mut k = vec![0.0; degree + 1];
        k.extend((1..spans).map(|i| i as f64 / spans as f64));
        k.extend(std::iter::repeat_n(1.0, degree + 1));
        Ok(KnotVector(k))
    }

    /// From distinct knots and multiplicities (OCCT form). Knots must be strictly increasing
    /// and every multiplicity ≥ 1.
    pub fn from_knots_mults(knots: &[f64], mults: &[usize]) -> Result<KnotVector, NurbsError> {
        if knots.len() != mults.len() {
            return Err(NurbsError::InvalidKnots(
                "knots and multiplicities differ in length",
            ));
        }
        if mults.contains(&0) {
            return Err(NurbsError::InvalidKnots("zero multiplicity"));
        }
        if knots.windows(2).any(|w| w[1] <= w[0]) {
            return Err(NurbsError::InvalidKnots(
                "distinct knots must strictly increase",
            ));
        }
        let total = mults
            .iter()
            .try_fold(0usize, |a, &m| a.checked_add(m))
            .filter(|&t| t <= 1 << 24)
            .ok_or(NurbsError::InvalidKnots("total multiplicity too large"))?;
        let mut v = Vec::with_capacity(total);
        for (&k, &m) in knots.iter().zip(mults) {
            v.extend(std::iter::repeat_n(k, m));
        }
        KnotVector::new(v)
    }

    /// Distinct knots + multiplicities (OCCT form); knots closer than `tol` to the first knot
    /// of their group merge into it.
    pub fn to_knots_mults(&self, tol: f64) -> (Vec<f64>, Vec<usize>) {
        let mut ks: Vec<f64> = Vec::new();
        let mut ms: Vec<usize> = Vec::new();
        for &k in &self.0 {
            match ks.last() {
                Some(&last) if (k - last).abs() <= tol => {
                    if let Some(m) = ms.last_mut() {
                        *m += 1;
                    }
                }
                _ => {
                    ks.push(k);
                    ms.push(1);
                }
            }
        }
        (ks, ms)
    }

    /// The knot values.
    #[inline]
    pub fn as_slice(&self) -> &[f64] {
        &self.0
    }

    /// Number of knots.
    #[inline]
    #[allow(clippy::len_without_is_empty, reason = "a knot vector is never empty")]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Parameter domain `(u_p, u_{m-p})` for `degree`. Falls back to the full range if the
    /// vector is too short for `degree`.
    pub fn domain(&self, degree: usize) -> (f64, f64) {
        let m = self.0.len();
        if 2 * degree + 1 < m {
            (self.0[degree], self.0[m - degree - 1])
        } else {
            (self.0[0], self.0[m - 1])
        }
    }

    /// A2.1 FindSpan: index `i` with `u_i <= u < u_{i+1}`; `u == u_max` maps to the last
    /// non-empty span, parameters outside the domain to the first/last span.
    /// Inconsistent `degree`/`n_poles` are clamped to the vector length (no panic).
    pub fn find_span(&self, degree: usize, n_poles: usize, u: f64) -> usize {
        find_span(&self.0, degree, n_poles, u)
    }

    /// Number of knots within `tol` of `u`.
    pub fn multiplicity(&self, u: f64, tol: f64) -> usize {
        self.0.iter().filter(|&&k| (k - u).abs() <= tol).count()
    }

    /// True if the first and last `degree + 1` knots are equal.
    pub fn is_clamped(&self, degree: usize) -> bool {
        let m = self.0.len();
        if m < 2 * (degree + 1) {
            return false;
        }
        let (a, b) = (self.0[0], self.0[m - 1]);
        self.0[..=degree].iter().all(|k| k.to_bits() == a.to_bits())
            && self.0[m - degree - 1..]
                .iter()
                .all(|k| k.to_bits() == b.to_bits())
    }

    /// Affine map of the whole knot range onto `range`. The end knots map exactly onto
    /// `range.0` / `range.1`. If `range` is not finite and increasing, an unchanged copy is
    /// returned.
    pub fn reparametrized(&self, range: (f64, f64)) -> KnotVector {
        let (r0, r1) = range;
        if !(r0.is_finite() && r1.is_finite() && r0 < r1) {
            return self.clone();
        }
        let m = self.0.len();
        let (a, b) = (self.0[0], self.0[m - 1]);
        let s = (r1 - r0) / (b - a);
        let mut v: Vec<f64> = self
            .0
            .iter()
            .map(|&k| {
                if k.to_bits() == a.to_bits() {
                    r0
                } else if k.to_bits() == b.to_bits() {
                    r1
                } else {
                    (r0 + (k - a) * s).clamp(r0, r1)
                }
            })
            .collect();
        // Rounding can never reorder (monotone map + clamp), but keep the invariant explicit.
        for i in 1..v.len() {
            if v[i] < v[i - 1] {
                v[i] = v[i - 1];
            }
        }
        if v[m - 1] <= v[0] {
            return self.clone();
        }
        KnotVector(v)
    }

    /// Greville abscissae of `degree` (one per pole): averages of `degree` consecutive knots.
    /// For degree 0 the span midpoints.
    pub fn greville(&self, degree: usize) -> Vec<f64> {
        let m = self.0.len();
        if degree == 0 {
            return self.0.windows(2).map(|w| 0.5 * (w[0] + w[1])).collect();
        }
        if m < degree + 2 {
            return Vec::new();
        }
        let n = m - degree - 1;
        (0..n)
            .map(|i| self.0[i + 1..=i + degree].iter().sum::<f64>() / degree as f64)
            .collect()
    }

    /// Merge keeping the maximum multiplicity of each distinct knot. Knots within `tol` of
    /// each other are the same knot; the value of `self` is kept for them. Determines the
    /// common refinement of two knot vectors (curve compatibility, KH-14).
    pub fn union(&self, other: &KnotVector, tol: f64) -> KnotVector {
        let tol = tol.max(0.0);
        let (ka, ma) = self.to_knots_mults(tol);
        let (kb, mb) = other.to_knots_mults(tol);
        let mut v = Vec::with_capacity(self.len() + other.len());
        let (mut i, mut j) = (0, 0);
        while i < ka.len() || j < kb.len() {
            let take = match (ka.get(i), kb.get(j)) {
                (Some(&a), Some(&b)) if (a - b).abs() <= tol => {
                    let m = ma[i].max(mb[j]);
                    i += 1;
                    j += 1;
                    (a, m)
                }
                (Some(&a), Some(&b)) if a < b => {
                    i += 1;
                    (a, ma[i - 1])
                }
                (Some(_), Some(&b)) | (None, Some(&b)) => {
                    j += 1;
                    (b, mb[j - 1])
                }
                (Some(&a), None) => {
                    i += 1;
                    (a, ma[i - 1])
                }
                (None, None) => break,
            };
            v.extend(std::iter::repeat_n(take.0, take.1));
        }
        KnotVector::new_unchecked(v)
    }

    /// Knots that must be inserted into `self` to obtain `target` (`target ⊇ self`), sorted.
    /// A target knot within `tol` of a knot of `self` is emitted with `self`'s value, so
    /// inserting the result raises that knot's multiplicity instead of adding a near-duplicate.
    /// Knots of `self` missing from `target` are ignored.
    pub fn missing_from(&self, target: &KnotVector, tol: f64) -> Vec<f64> {
        let tol = tol.max(0.0);
        let (ks, ms) = self.to_knots_mults(tol);
        let (kt, mt) = target.to_knots_mults(tol);
        let mut out = Vec::new();
        let mut i = 0;
        for (&t, &m) in kt.iter().zip(&mt) {
            while i < ks.len() && ks[i] < t - tol {
                i += 1;
            }
            let (val, have) = match ks.get(i) {
                Some(&k) if (k - t).abs() <= tol => (k, ms[i]),
                _ => (t, 0),
            };
            if m > have {
                out.extend(std::iter::repeat_n(val, m - have));
            }
        }
        out
    }

    /// Mirrored knots `a + b − u_{m−i}` (used by curve/surface reversal).
    pub(crate) fn reversed(&self) -> KnotVector {
        let m = self.0.len();
        let (a, b) = (self.0[0], self.0[m - 1]);
        let mut v: Vec<f64> = self.0.iter().rev().map(|&k| (a + b) - k).collect();
        v[0] = a;
        v[m - 1] = b;
        for i in 1..m {
            v[i] = v[i].clamp(a, b);
            if v[i] < v[i - 1] {
                v[i] = v[i - 1];
            }
        }
        KnotVector(v)
    }
}

/// A2.1 on a raw slice; never panics for `knots.len() >= 2`.
#[inline]
pub(crate) fn find_span(knots: &[f64], degree: usize, n_poles: usize, u: f64) -> usize {
    let m = knots.len();
    if m < 2 {
        return 0;
    }
    // n = index of the last pole; clamp so that n + 1 <= m - 1 and degree <= n.
    let n = n_poles.saturating_sub(1).min(m - 2);
    let p = degree.min(n);
    if u >= knots[n + 1] {
        // Last non-empty span at or below n.
        let mut i = n;
        while i > p && knots[i] >= knots[n + 1] {
            i -= 1;
        }
        return i;
    }
    if u <= knots[p] {
        // First non-empty span at or above p.
        let mut i = p;
        while i < n && knots[i + 1] <= knots[p] {
            i += 1;
        }
        return i;
    }
    let (mut low, mut high) = (p, n + 1);
    let mut mid = (low + high) / 2;
    // Bounded loop: the interval halves each step (also for NaN `u`, which exits at once).
    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        let next = (low + high) / 2;
        if next == mid {
            break;
        }
        mid = next;
    }
    mid
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact expected values")]
mod tests {
    use super::*;

    #[test]
    fn find_span_examples() {
        // P&T Ex. 2.3 knot vector, p = 2, n + 1 = 8 poles.
        let k = KnotVector::new(vec![0., 0., 0., 1., 2., 3., 4., 4., 5., 5., 5.]).unwrap();
        assert_eq!(k.find_span(2, 8, 2.5), 4);
        assert_eq!(k.find_span(2, 8, 0.0), 2);
        assert_eq!(k.find_span(2, 8, 5.0), 7);
        assert_eq!(k.find_span(2, 8, 4.0), 7);
        assert_eq!(k.find_span(2, 8, 3.999), 5);
        assert_eq!(k.find_span(2, 8, -1.0), 2);
        assert_eq!(k.find_span(2, 8, 9.0), 7);
        let _ = k.find_span(2, 8, f64::NAN);
        let _ = k.find_span(40, 1000, 1.0);
    }

    #[test]
    fn knots_mults_round_trip() {
        let k = KnotVector::from_knots_mults(&[0., 1., 2.], &[3, 1, 3]).unwrap();
        assert_eq!(k.as_slice(), &[0., 0., 0., 1., 2., 2., 2.]);
        assert_eq!(k.to_knots_mults(1e-12), (vec![0., 1., 2.], vec![3, 1, 3]));
        assert!(k.is_clamped(2));
        assert!(!k.is_clamped(3));
        assert_eq!(k.domain(2), (0.0, 2.0));
        assert_eq!(k.multiplicity(2.0, 0.0), 3);
    }

    #[test]
    fn clamped_uniform_and_greville() {
        let k = KnotVector::clamped_uniform(3, 6).unwrap();
        assert_eq!(k.len(), 10);
        assert_eq!(k.as_slice()[4], 1.0 / 3.0);
        let g = k.greville(3);
        assert_eq!(g.len(), 6);
        assert_eq!(g[0], 0.0);
        assert_eq!(g[5], 1.0);
        assert_eq!(KnotVector::bezier(2).as_slice(), &[0., 0., 0., 1., 1., 1.]);
    }

    #[test]
    fn union_and_missing() {
        let a = KnotVector::new(vec![0., 0., 0., 0.5, 1., 1., 1.]).unwrap();
        let b = KnotVector::new(vec![0., 0., 0.25, 0.5, 0.5, 1., 1.]).unwrap();
        let u = a.union(&b, 1e-12);
        assert_eq!(u.as_slice(), &[0., 0., 0., 0.25, 0.5, 0.5, 1., 1., 1.]);
        assert_eq!(a.missing_from(&u, 1e-12), vec![0.25, 0.5]);
        assert_eq!(b.missing_from(&u, 1e-12), vec![0., 1.]);
        assert!(u.missing_from(&u, 0.0).is_empty());
        // Near-equal knots keep self's value.
        let c = KnotVector::new(vec![0., 0., 0.5 + 1e-14, 1., 1.]).unwrap();
        assert_eq!(a.union(&c, 1e-12).as_slice(), a.as_slice());
        assert_eq!(
            a.missing_from(
                &KnotVector::new(vec![0., 0., 0., 0.5, 0.5 + 1e-14, 1., 1., 1.]).unwrap(),
                1e-12
            ),
            vec![0.5]
        );
        // Union is commutative up to which near-equal value is kept.
        assert_eq!(b.union(&a, 0.0), u);
    }

    #[test]
    fn reparam_and_reverse() {
        let k = KnotVector::new(vec![0., 0., 0.3, 1., 1.]).unwrap();
        let r = k.reparametrized((2.0, 4.0));
        assert_eq!(r.as_slice()[0], 2.0);
        assert_eq!(r.as_slice()[4], 4.0);
        assert!((r.as_slice()[2] - 2.6).abs() < 1e-15);
        let rev = k.reversed();
        assert!((rev.as_slice()[2] - 0.7).abs() < 1e-15);
        assert!((rev.reversed().as_slice()[2] - 0.3).abs() < 1e-15);
        assert_eq!(k.reparametrized((1.0, 1.0)), k);
    }
}
