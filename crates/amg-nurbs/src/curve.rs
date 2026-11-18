//! [`NurbsCurve`]: clamped B-spline / NURBS curves (P&T ch. 2–4).

use amg_core::{Aabb3, Dir3, Plane, Point2, Point3, Tolerance, Transform, Vec3};
use smallvec::SmallVec;

use crate::basis::{basis_funs_n, ders_basis_funs_n, with_cap};
use crate::knots::find_span;
use crate::{CtrlPt, KnotVector, MAX_CURVE_DERIV, MAX_DEGREE, NurbsError};

/// Clamped B-spline/NURBS curve.
///
/// Invariants (checked by [`NurbsCurve::new`] and on deserialisation):
/// `1 <= degree <= MAX_DEGREE`, `knots.len() == poles.len() + degree + 1`, clamped ends
/// (multiplicity exactly `degree + 1`), interior multiplicities `<= degree`, finite poles,
/// and weights (if any) finite, `> 0` and one per pole.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
#[serde(bound(serialize = "P: CtrlPt"))]
pub struct NurbsCurve<P: CtrlPt = Point3> {
    degree: usize,
    knots: KnotVector,
    poles: Vec<P>,
    weights: Option<Vec<f64>>,
}

/// 2D curve.
pub type NurbsCurve2 = NurbsCurve<Point2>;
/// 3D curve.
pub type NurbsCurve3 = NurbsCurve<Point3>;

#[derive(serde::Deserialize)]
#[serde(bound(deserialize = "P: CtrlPt"))]
struct CurveRaw<P: CtrlPt> {
    degree: usize,
    knots: KnotVector,
    poles: Vec<P>,
    weights: Option<Vec<f64>>,
}

impl<'de, P: CtrlPt> serde::Deserialize<'de> for NurbsCurve<P> {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let r = CurveRaw::<P>::deserialize(d)?;
        NurbsCurve::new(r.degree, r.knots, r.poles, r.weights).map_err(serde::de::Error::custom)
    }
}

/// A point on a curve with its parameter and distance to a query point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurvePoint<P> {
    /// Curve parameter.
    pub t: f64,
    /// Curve point at `t`.
    pub point: P,
    /// Distance from the query point.
    pub distance: f64,
}

/// Geometric / parametric continuity class at a knot.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
pub enum Continuity {
    /// Positional only.
    C0,
    /// Tangent direction.
    G1,
    /// First derivative.
    C1,
    /// Curvature.
    G2,
    /// Second derivative.
    C2,
    /// Smooth beyond second derivative.
    Cn,
}

/// Binomial coefficients up to 4.
const BIN: [[f64; MAX_CURVE_DERIV + 1]; MAX_CURVE_DERIV + 1] = [
    [1.0, 0.0, 0.0, 0.0, 0.0],
    [1.0, 1.0, 0.0, 0.0, 0.0],
    [1.0, 2.0, 1.0, 0.0, 0.0],
    [1.0, 3.0, 3.0, 1.0, 0.0],
    [1.0, 4.0, 6.0, 4.0, 1.0],
];

/// Validates degree, counts, clamping and multiplicities for one parametric direction.
pub(crate) fn check_knots(
    degree: usize,
    knots: &KnotVector,
    n_poles: usize,
) -> Result<(), NurbsError> {
    if degree == 0 || degree > MAX_DEGREE {
        return Err(NurbsError::Degree(degree));
    }
    if n_poles <= degree || knots.len() != n_poles + degree + 1 {
        return Err(NurbsError::CountMismatch {
            degree,
            poles: n_poles,
            knots: knots.len(),
        });
    }
    if !knots.is_clamped(degree) {
        return Err(NurbsError::InvalidKnots("not clamped"));
    }
    let (_, mults) = knots.to_knots_mults(0.0);
    let last = mults.len() - 1;
    if mults[0] != degree + 1 || mults[last] != degree + 1 {
        return Err(NurbsError::InvalidKnots(
            "end knot multiplicity exceeds degree + 1",
        ));
    }
    if mults[1..last].iter().any(|&m| m > degree) {
        return Err(NurbsError::InvalidKnots(
            "interior knot multiplicity exceeds the degree",
        ));
    }
    Ok(())
}

/// Validates a weight slice (finite, positive).
pub(crate) fn check_weights(w: &[f64]) -> Result<(), NurbsError> {
    match w.iter().position(|&x| !(x.is_finite() && x > 0.0)) {
        Some(index) => Err(NurbsError::Weight { index }),
        None => Ok(()),
    }
}

impl<P: CtrlPt> NurbsCurve<P> {
    /// Validated constructor (see the type docs for the invariants).
    pub fn new(
        degree: usize,
        knots: KnotVector,
        poles: Vec<P>,
        weights: Option<Vec<f64>>,
    ) -> Result<Self, NurbsError> {
        check_knots(degree, &knots, poles.len())?;
        if poles.iter().any(|p| !p.is_finite_pt()) {
            return Err(NurbsError::Degenerate("control point is not finite"));
        }
        if let Some(w) = &weights {
            if w.len() != poles.len() {
                return Err(NurbsError::Degenerate("weights and poles differ in length"));
            }
            check_weights(w)?;
        }
        Ok(NurbsCurve {
            degree,
            knots,
            poles,
            weights,
        })
    }

    /// Crate-internal constructor for data that is valid by construction.
    pub(crate) fn new_unchecked(
        degree: usize,
        knots: KnotVector,
        poles: Vec<P>,
        weights: Option<Vec<f64>>,
    ) -> Self {
        // Structural invariants only: derived poles of near-`f64::MAX` input may overflow.
        debug_assert!(
            check_knots(degree, &knots, poles.len()).is_ok()
                && weights.as_ref().is_none_or(|w| w.len() == poles.len()),
            "invalid curve data"
        );
        NurbsCurve {
            degree,
            knots,
            poles,
            weights,
        }
    }

    /// Degree.
    #[inline]
    pub fn degree(&self) -> usize {
        self.degree
    }

    /// Knot vector.
    #[inline]
    pub fn knots(&self) -> &KnotVector {
        &self.knots
    }

    /// Control points.
    #[inline]
    pub fn poles(&self) -> &[P] {
        &self.poles
    }

    /// Rational weights, if stored.
    #[inline]
    pub fn weights(&self) -> Option<&[f64]> {
        self.weights.as_deref()
    }

    /// True if weights are stored and not all equal.
    pub fn is_rational(&self) -> bool {
        match &self.weights {
            None => false,
            Some(w) => w.iter().any(|x| x.to_bits() != w[0].to_bits()),
        }
    }

    /// Parameter domain `(t_min, t_max)`.
    #[inline]
    pub fn domain(&self) -> (f64, f64) {
        self.knots.domain(self.degree)
    }

    /// Point at `t` (A4.1 / A4.2 point part). `t` is clamped into the domain.
    #[inline]
    pub fn eval(&self, t: f64) -> P {
        with_cap!(self.degree, N => self.eval_n::<N>(t))
    }

    #[inline(always)]
    fn eval_n<const N: usize>(&self, t: f64) -> P {
        let p = self.degree;
        let k = self.knots.as_slice();
        let (a, b) = self.domain();
        let t = t.clamp(a, b);
        let span = find_span(k, p, self.poles.len(), t);
        let mut nb = [0.0; N];
        basis_funs_n::<N>(k, span, t, p, &mut nb);
        let base = span - p;
        let poles = &self.poles[base..=span];
        match &self.weights {
            None => {
                let mut c = P::zero();
                for (pt, &n) in poles.iter().zip(&nb[..=p]) {
                    c = c + *pt * n;
                }
                c
            }
            Some(w) => {
                let mut c = P::zero();
                let mut den = 0.0;
                for ((pt, &wi), &n) in poles.iter().zip(&w[base..=span]).zip(&nb[..=p]) {
                    let nw = n * wi;
                    c = c + *pt * nw;
                    den += nw;
                }
                c * (1.0 / den)
            }
        }
    }

    /// `[C, C', …, C^(n)]`, rational-correct (A3.2 / A4.2). `n` is clamped to
    /// [`MAX_CURVE_DERIV`] (= 4); `t` is clamped into the domain. Derivatives at a knot are
    /// taken from the span to the right of it (the left one at `t_max`).
    pub fn derivs(&self, t: f64, n: usize) -> SmallVec<[P; 5]> {
        let n = n.min(MAX_CURVE_DERIV);
        with_cap!(self.degree, N => self.derivs_n::<N>(t, n))
    }

    #[inline(always)]
    fn derivs_n<const N: usize>(&self, t: f64, n: usize) -> SmallVec<[P; 5]> {
        let p = self.degree;
        let k = self.knots.as_slice();
        let (a, b) = self.domain();
        let t = t.clamp(a, b);
        let span = find_span(k, p, self.poles.len(), t);
        let mut ders = [[0.0; N]; MAX_CURVE_DERIV + 1];
        ders_basis_funs_n::<N>(k, span, t, p, n, &mut ders[..=n]);
        let base = span - p;
        let poles = &self.poles[base..=span];
        let mut ck: SmallVec<[P; 5]> = SmallVec::new();
        match &self.weights {
            None => {
                for row in ders.iter().take(n + 1) {
                    let mut c = P::zero();
                    for (pt, &d) in poles.iter().zip(&row[..=p]) {
                        c = c + *pt * d;
                    }
                    ck.push(c);
                }
            }
            Some(w) => {
                let w = &w[base..=span];
                let mut aders = [P::zero(); MAX_CURVE_DERIV + 1];
                let mut wders = [0.0; MAX_CURVE_DERIV + 1];
                for kk in 0..=n {
                    for j in 0..=p {
                        let dw = ders[kk][j] * w[j];
                        aders[kk] = aders[kk] + poles[j] * dw;
                        wders[kk] += dw;
                    }
                }
                let inv = 1.0 / wders[0];
                for kk in 0..=n {
                    let mut v = aders[kk];
                    for i in 1..=kk {
                        v = v - ck[kk - i] * (BIN[kk][i] * wders[i]);
                    }
                    ck.push(v * inv);
                }
            }
        }
        ck
    }

    /// First point (the first pole: curves are clamped).
    #[inline]
    pub fn start(&self) -> P {
        self.poles[0]
    }

    /// Last point (the last pole).
    #[inline]
    pub fn end(&self) -> P {
        self.poles[self.poles.len() - 1]
    }

    /// True if start and end coincide within `tol.linear`.
    pub fn is_closed(&self, tol: &Tolerance) -> bool {
        (self.end() - self.start()).length() <= tol.linear
    }

    /// Same curve traversed backwards: `C'(t) = C(a + b − t)` on the same domain.
    pub fn reversed(&self) -> Self {
        let mut poles = self.poles.clone();
        poles.reverse();
        let weights = self.weights.as_ref().map(|w| {
            let mut w = w.clone();
            w.reverse();
            w
        });
        NurbsCurve {
            degree: self.degree,
            knots: self.knots.reversed(),
            poles,
            weights,
        }
    }

    /// Same shape on the new domain `range` (affine knot map). Invalid ranges (not finite
    /// and increasing) leave the curve unchanged.
    pub fn reparametrized(&self, range: (f64, f64)) -> Self {
        NurbsCurve {
            knots: self.knots.reparametrized(range),
            ..self.clone()
        }
    }
}

impl NurbsCurve3 {
    /// Curve with every pole transformed (affine invariance; weights unchanged).
    pub fn transformed(&self, t: &Transform) -> Self {
        NurbsCurve {
            poles: self.poles.iter().map(|&p| t.transform_point3(p)).collect(),
            ..self.clone()
        }
    }

    /// Control-hull bound (conservative; contains the curve by the convex-hull property).
    pub fn bbox(&self) -> Aabb3 {
        Aabb3::from_points(self.poles.iter().copied())
    }

    /// Curvature `|C' × C''| / |C'|³`; 0 where the tangent vanishes.
    pub fn curvature(&self, t: f64) -> f64 {
        let d = self.derivs(t, 2);
        let s = d[1].length();
        if s == 0.0 || !s.is_finite() {
            return 0.0;
        }
        d[1].cross(d[2]).length() / (s * s * s)
    }

    /// Torsion `(C' × C'') · C''' / |C' × C''|²`; 0 where the curvature vanishes.
    pub fn torsion(&self, t: f64) -> f64 {
        let d = self.derivs(t, 3);
        let c = d[1].cross(d[2]);
        let c2 = c.length_squared();
        if c2 == 0.0 || !c2.is_finite() {
            return 0.0;
        }
        c.dot(d[3]) / c2
    }

    /// `Some(plane)` if all poles lie within `tol.linear` of one plane (then so does the
    /// curve). The plane is the least-squares plane of the poles through their centroid.
    /// Collinear (or coincident) poles give a plane containing the line.
    pub fn planarity(&self, tol: &Tolerance) -> Option<Plane> {
        planarity_of(&self.poles, tol)
    }
}

/// Least-squares plane of `pts` if every point is within `tol.linear` of it.
pub(crate) fn planarity_of(pts: &[Point3], tol: &Tolerance) -> Option<Plane> {
    let n = pts.len() as f64;
    let c = pts.iter().fold(Vec3::ZERO, |a, &p| a + p) / n;
    if !c.is_finite() {
        return None;
    }
    // Seed: farthest point from the centroid, then farthest from that line.
    let a = pts.iter().copied().max_by(|x, y| {
        (*x - c)
            .length_squared()
            .total_cmp(&(*y - c).length_squared())
    })?;
    let ax = a - c;
    if ax.length() <= tol.linear {
        // All points coincide: any plane through them.
        return Some(Plane::from_normal(c, Dir3::Z));
    }
    let axd = ax.normalize();
    if !axd.is_finite() {
        return None;
    }
    let b = pts.iter().copied().max_by(|x, y| {
        let dx = (*x - c) - axd * (*x - c).dot(axd);
        let dy = (*y - c) - axd * (*y - c).dot(axd);
        dx.length_squared().total_cmp(&dy.length_squared())
    })?;
    let bperp = (b - c) - axd * (b - c).dot(axd);
    let mut normal = if bperp.length() <= tol.linear {
        // Collinear: deterministic normal perpendicular to the line.
        Dir3::new_unchecked(axd).any_orthogonal().get()
    } else {
        axd.cross(bperp).normalize()
    };
    if bperp.length() > tol.linear {
        // Refine to the least-squares normal: power iteration on tr(C)·I − C, whose dominant
        // eigenvector is C's smallest one.
        let mut cov = [[0.0f64; 3]; 3];
        for &p in pts {
            let d = (p - c).to_array();
            for (i, row) in cov.iter_mut().enumerate() {
                for (j, v) in row.iter_mut().enumerate() {
                    *v += d[i] * d[j];
                }
            }
        }
        let tr = cov[0][0] + cov[1][1] + cov[2][2];
        let m = amg_core::Mat3::from_cols_array_2d(&cov);
        let shifted = amg_core::Mat3::from_diagonal(Vec3::splat(tr)) - m;
        for _ in 0..64 {
            let next = shifted * normal;
            let l = next.length();
            if !(l.is_finite() && l > 0.0) {
                break;
            }
            normal = next / l;
        }
    }
    if !(normal.is_finite() && (normal.length() - 1.0).abs() <= 1e-9) {
        return None;
    }
    let nd = Dir3::new_unchecked(normal);
    let plane = Plane::from_normal(c, nd);
    pts.iter()
        .all(|&p| plane.signed_distance(p).abs() <= tol.linear)
        .then_some(plane)
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact expected values")]
mod tests {
    use super::*;

    fn quad() -> NurbsCurve3 {
        NurbsCurve::new(
            2,
            KnotVector::new(vec![0., 0., 0., 1., 1., 1.]).unwrap(),
            vec![
                Point3::ZERO,
                Point3::new(1., 1., 0.),
                Point3::new(2., 0., 0.),
            ],
            None,
        )
        .unwrap()
    }

    #[test]
    fn bezier_quadratic() {
        let c = quad();
        let p = c.eval(0.5);
        assert!((p - Point3::new(1.0, 0.5, 0.0)).length() < 1e-15);
        let d = c.derivs(0.5, 3);
        assert!((d[1] - Point3::new(2.0, 0.0, 0.0)).length() < 1e-14);
        assert!((d[2] - Point3::new(0.0, -4.0, 0.0)).length() < 1e-14);
        assert_eq!(d[3], Point3::ZERO);
        assert_eq!(c.derivs(0.5, 99).len(), MAX_CURVE_DERIV + 1);
        assert_eq!(c.start(), Point3::ZERO);
        assert_eq!(c.end(), Point3::new(2., 0., 0.));
        assert!(!c.is_rational());
    }

    #[test]
    fn reverse_and_reparam() {
        let c = quad();
        let r = c.reversed();
        assert!((r.eval(0.25) - c.eval(0.75)).length() < 1e-15);
        let rp = c.reparametrized((10.0, 12.0));
        assert_eq!(rp.domain(), (10.0, 12.0));
        assert!((rp.eval(11.0) - c.eval(0.5)).length() < 1e-15);
    }

    #[test]
    fn planarity() {
        let c = quad().transformed(&Transform::from_rotation_x(0.3));
        let tol = Tolerance::DEFAULT;
        let pl = c.planarity(&tol).unwrap();
        assert!(
            pl.normal
                .get()
                .dot(Vec3::new(0., -0.3f64.sin(), 0.3f64.cos()))
                .abs()
                > 1.0 - 1e-12
        );
        let helixish = NurbsCurve::new(
            1,
            KnotVector::new(vec![0., 0., 1., 2., 3., 3.]).unwrap(),
            vec![
                Point3::ZERO,
                Point3::X,
                Point3::new(1., 1., 0.),
                Point3::new(0., 1., 1.),
            ],
            None,
        )
        .unwrap();
        assert!(helixish.planarity(&tol).is_none());
        let line = NurbsCurve::new(
            1,
            KnotVector::new(vec![0., 0., 1., 1.]).unwrap(),
            vec![Point3::ZERO, Point3::X],
            None,
        )
        .unwrap();
        assert!(line.planarity(&tol).is_some());
    }
}
