//! [`NurbsSurface`]: clamped tensor-product B-spline / NURBS surfaces (P&T ch. 3–4).

use amg_core::{Aabb3, Dir3, Point3, Tolerance, Transform, Vec3};

use crate::basis::{basis_funs_n, ders_basis_funs_n, with_cap};
use crate::curve::{check_knots, check_weights};
use crate::knots::find_span;
use crate::{CtrlPt, Grid, KnotVector, MAX_SURFACE_DERIV, NurbsCurve, NurbsError};

/// Parametric direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum UV {
    /// The u direction (grid index `i`).
    U,
    /// The v direction (grid index `j`).
    V,
}

/// Boundary of a surface's parameter rectangle.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Side {
    /// `u = u_min` (pole row `i = 0`).
    UMin,
    /// `u = u_max` (pole row `i = nu − 1`).
    UMax,
    /// `v = v_min` (pole column `j = 0`).
    VMin,
    /// `v = v_max` (pole column `j = nv − 1`).
    VMax,
}

/// Clamped tensor-product NURBS surface. Poles are a row-major [`Grid`] (`i` along u).
///
/// Invariants (checked by [`NurbsSurface::new`] and on deserialisation): per direction the
/// curve invariants of [`NurbsCurve`]; the weight grid (if any) has the pole grid's size and
/// positive finite weights.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
#[serde(bound(serialize = "P: CtrlPt"))]
pub struct NurbsSurface<P: CtrlPt = Point3> {
    degree: [usize; 2],
    knots: [KnotVector; 2],
    poles: Grid<P>,
    weights: Option<Grid<f64>>,
}

/// 3D surface.
pub type NurbsSurface3 = NurbsSurface<Point3>;

#[derive(serde::Deserialize)]
#[serde(bound(deserialize = "P: CtrlPt"))]
struct SurfaceRaw<P: CtrlPt> {
    degree: [usize; 2],
    knots: [KnotVector; 2],
    poles: Grid<P>,
    weights: Option<Grid<f64>>,
}

impl<'de, P: CtrlPt> serde::Deserialize<'de> for NurbsSurface<P> {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let r = SurfaceRaw::<P>::deserialize(d)?;
        NurbsSurface::new(r.degree, r.knots, r.poles, r.weights).map_err(serde::de::Error::custom)
    }
}

/// `d[k][l]` = ∂^{k+l} S / ∂u^k ∂v^l for `k + l <= order`; other entries are zero.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceDerivs {
    /// Derivative table.
    pub d: [[Vec3; 3]; 3],
    /// Highest total order computed.
    pub order: usize,
}

/// A point on a surface with its parameters and distance to a query point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfacePoint {
    /// u parameter.
    pub u: f64,
    /// v parameter.
    pub v: f64,
    /// Surface point at `(u, v)`.
    pub point: Point3,
    /// Distance from the query point.
    pub distance: f64,
}

const BIN: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [1.0, 2.0, 1.0]];

impl<P: CtrlPt> NurbsSurface<P> {
    /// Validated constructor (see the type docs for the invariants).
    pub fn new(
        degree: [usize; 2],
        knots: [KnotVector; 2],
        poles: Grid<P>,
        weights: Option<Grid<f64>>,
    ) -> Result<Self, NurbsError> {
        check_knots(degree[0], &knots[0], poles.nu())?;
        check_knots(degree[1], &knots[1], poles.nv())?;
        if poles.as_slice().iter().any(|p| !p.is_finite_pt()) {
            return Err(NurbsError::Degenerate("control point is not finite"));
        }
        if let Some(w) = &weights {
            if w.nu() != poles.nu() || w.nv() != poles.nv() {
                return Err(NurbsError::Degenerate(
                    "weight grid and pole grid differ in size",
                ));
            }
            check_weights(w.as_slice())?;
        }
        Ok(NurbsSurface {
            degree,
            knots,
            poles,
            weights,
        })
    }

    /// Crate-internal constructor for data that is valid by construction.
    pub(crate) fn new_unchecked(
        degree: [usize; 2],
        knots: [KnotVector; 2],
        poles: Grid<P>,
        weights: Option<Grid<f64>>,
    ) -> Self {
        // Structural invariants only (see `NurbsCurve::new_unchecked`).
        debug_assert!(
            check_knots(degree[0], &knots[0], poles.nu()).is_ok()
                && check_knots(degree[1], &knots[1], poles.nv()).is_ok()
                && weights
                    .as_ref()
                    .is_none_or(|w| w.nu() == poles.nu() && w.nv() == poles.nv()),
            "invalid surface data"
        );
        NurbsSurface {
            degree,
            knots,
            poles,
            weights,
        }
    }

    #[inline]
    fn idx(d: UV) -> usize {
        match d {
            UV::U => 0,
            UV::V => 1,
        }
    }

    /// Degree in direction `d`.
    #[inline]
    pub fn degree(&self, d: UV) -> usize {
        self.degree[Self::idx(d)]
    }

    /// Knot vector in direction `d`.
    #[inline]
    pub fn knots(&self, d: UV) -> &KnotVector {
        &self.knots[Self::idx(d)]
    }

    /// Pole grid.
    #[inline]
    pub fn poles(&self) -> &Grid<P> {
        &self.poles
    }

    /// Weight grid, if stored.
    #[inline]
    pub fn weights(&self) -> Option<&Grid<f64>> {
        self.weights.as_ref()
    }

    /// True if weights are stored and not all equal.
    pub fn is_rational(&self) -> bool {
        match &self.weights {
            None => false,
            Some(w) => {
                let s = w.as_slice();
                s.iter().any(|x| x.to_bits() != s[0].to_bits())
            }
        }
    }

    /// `[(u_min, u_max), (v_min, v_max)]`.
    #[inline]
    pub fn domain(&self) -> [(f64, f64); 2] {
        [
            self.knots[0].domain(self.degree[0]),
            self.knots[1].domain(self.degree[1]),
        ]
    }

    /// Clamps `(u, v)` and finds both spans.
    #[inline(always)]
    fn locate(&self, u: f64, v: f64) -> (f64, f64, usize, usize) {
        let [(u0, u1), (v0, v1)] = self.domain();
        let u = u.clamp(u0, u1);
        let v = v.clamp(v0, v1);
        let su = find_span(self.knots[0].as_slice(), self.degree[0], self.poles.nu(), u);
        let sv = find_span(self.knots[1].as_slice(), self.degree[1], self.poles.nv(), v);
        (u, v, su, sv)
    }

    /// Point at `(u, v)` (A4.3); parameters are clamped into the domain.
    #[inline]
    pub fn eval(&self, u: f64, v: f64) -> P {
        with_cap!(self.degree[0].max(self.degree[1]), N => self.eval_n::<N>(u, v))
    }

    #[inline(always)]
    fn eval_n<const N: usize>(&self, u: f64, v: f64) -> P {
        let [p, q] = self.degree;
        let (u, v, su, sv) = self.locate(u, v);
        let mut nu = [0.0; N];
        let mut nv = [0.0; N];
        basis_funs_n::<N>(self.knots[0].as_slice(), su, u, p, &mut nu);
        basis_funs_n::<N>(self.knots[1].as_slice(), sv, v, q, &mut nv);
        let (iu, jv) = (su - p, sv - q);
        match &self.weights {
            None => {
                let mut s = P::zero();
                for (k, &bu) in nu[..=p].iter().enumerate() {
                    let row = &self.poles.row(iu + k)[jv..=sv];
                    let mut t = P::zero();
                    for (pt, &bv) in row.iter().zip(&nv[..=q]) {
                        t = t + *pt * bv;
                    }
                    s = s + t * bu;
                }
                s
            }
            Some(w) => {
                let mut s = P::zero();
                let mut den = 0.0;
                for (k, &bu) in nu[..=p].iter().enumerate() {
                    let row = &self.poles.row(iu + k)[jv..=sv];
                    let wrow = &w.row(iu + k)[jv..=sv];
                    let mut t = P::zero();
                    let mut tw = 0.0;
                    for ((pt, &wi), &bv) in row.iter().zip(wrow).zip(&nv[..=q]) {
                        let bw = bv * wi;
                        t = t + *pt * bw;
                        tw += bw;
                    }
                    s = s + t * bu;
                    den += tw * bu;
                }
                s * (1.0 / den)
            }
        }
    }

    /// Iso-parametric curve with direction `d` held at `t`: `iso_curve(UV::U, t)` is the
    /// curve `v ↦ S(t, v)` (degree and knots of v), `iso_curve(UV::V, t)` is `u ↦ S(u, t)`.
    /// `t` is clamped into the domain. (Only for coordinates near `f64::MAX` can the derived
    /// poles overflow.)
    pub fn iso_curve(&self, d: UV, t: f64) -> NurbsCurve<P> {
        with_cap!(self.degree[Self::idx(d)], N => self.iso_curve_n::<N>(d, t))
    }

    fn iso_curve_n<const N: usize>(&self, d: UV, t: f64) -> NurbsCurve<P> {
        let di = Self::idx(d);
        let other = 1 - di;
        let p = self.degree[di];
        let k = self.knots[di].as_slice();
        let n_fixed = if di == 0 {
            self.poles.nu()
        } else {
            self.poles.nv()
        };
        let n_free = if di == 0 {
            self.poles.nv()
        } else {
            self.poles.nu()
        };
        let (a, b) = self.knots[di].domain(p);
        let t = t.clamp(a, b);
        let span = find_span(k, p, n_fixed, t);
        let mut nb = [0.0; N];
        basis_funs_n::<N>(k, span, t, p, &mut nb);
        let at = |fixed: usize, free: usize| -> (P, f64) {
            let (i, j) = if di == 0 {
                (fixed, free)
            } else {
                (free, fixed)
            };
            let w = self.weights.as_ref().map_or(1.0, |w| w.get(i, j));
            (self.poles.get(i, j), w)
        };
        let mut poles = Vec::with_capacity(n_free);
        let mut weights = Vec::with_capacity(n_free);
        for free in 0..n_free {
            let mut s = P::zero();
            let mut ws = 0.0;
            for (r, &bn) in nb[..=p].iter().enumerate() {
                let (pt, w) = at(span - p + r, free);
                s = s + pt * (bn * w);
                ws += bn * w;
            }
            poles.push(s * (1.0 / ws));
            weights.push(ws);
        }
        let weights = self.weights.as_ref().map(|_| weights);
        NurbsCurve::new_unchecked(
            self.degree[other],
            self.knots[other].clone(),
            poles,
            weights,
        )
    }

    /// Boundary curve on `side`, copied exactly from the first/last pole row or column.
    /// `UMin`/`UMax` run along v, `VMin`/`VMax` along u.
    pub fn boundary(&self, side: Side) -> NurbsCurve<P> {
        let (nu, nv) = (self.poles.nu(), self.poles.nv());
        let (poles, weights, d) = match side {
            Side::UMin | Side::UMax => {
                let i = if side == Side::UMin { 0 } else { nu - 1 };
                (
                    self.poles.row(i).to_vec(),
                    self.weights.as_ref().map(|w| w.row(i).to_vec()),
                    1,
                )
            }
            Side::VMin | Side::VMax => {
                let j = if side == Side::VMin { 0 } else { nv - 1 };
                (
                    self.poles.column(j),
                    self.weights.as_ref().map(|w| w.column(j)),
                    0,
                )
            }
        };
        NurbsCurve::new_unchecked(self.degree[d], self.knots[d].clone(), poles, weights)
    }

    /// Same surface with direction `d` reversed (knots mirrored, poles flipped).
    pub fn reversed(&self, d: UV) -> Self {
        let along_u = d == UV::U;
        let mut knots = self.knots.clone();
        let di = Self::idx(d);
        knots[di] = self.knots[di].reversed();
        NurbsSurface {
            degree: self.degree,
            knots,
            poles: self.poles.reversed(along_u),
            weights: self.weights.as_ref().map(|w| w.reversed(along_u)),
        }
    }

    /// Swaps u and v (`S'(v, u) = S(u, v)`; the normal flips).
    pub fn transposed(&self) -> Self {
        NurbsSurface {
            degree: [self.degree[1], self.degree[0]],
            knots: [self.knots[1].clone(), self.knots[0].clone()],
            poles: self.poles.transposed(),
            weights: self.weights.as_ref().map(Grid::transposed),
        }
    }
}

impl NurbsSurface3 {
    /// Partial derivatives up to total order `order` (A3.6 / A4.4). `order` is clamped to
    /// [`MAX_SURFACE_DERIV`] (= 2); parameters are clamped into the domain.
    pub fn derivs(&self, u: f64, v: f64, order: usize) -> SurfaceDerivs {
        let order = order.min(MAX_SURFACE_DERIV);
        with_cap!(self.degree[0].max(self.degree[1]), N => self.derivs_n::<N>(u, v, order))
    }

    #[inline(always)]
    fn derivs_n<const N: usize>(&self, u: f64, v: f64, d: usize) -> SurfaceDerivs {
        let [p, q] = self.degree;
        let (u, v, su, sv) = self.locate(u, v);
        let mut nu = [[0.0; N]; MAX_SURFACE_DERIV + 1];
        let mut nv = [[0.0; N]; MAX_SURFACE_DERIV + 1];
        ders_basis_funs_n::<N>(self.knots[0].as_slice(), su, u, p, d, &mut nu[..=d]);
        ders_basis_funs_n::<N>(self.knots[1].as_slice(), sv, v, q, d, &mut nv[..=d]);
        let (iu, jv) = (su - p, sv - q);
        let mut a = [[Vec3::ZERO; 3]; 3];
        let mut wd = [[0.0f64; 3]; 3];
        let w = self.weights.as_ref();
        // One pass over the (p+1) × (q+1) pole block accumulates every u-order at once.
        let mut temp = [[Vec3::ZERO; N]; MAX_SURFACE_DERIV + 1];
        let mut tw = [[0.0; N]; MAX_SURFACE_DERIV + 1];
        #[allow(
            clippy::needless_range_loop,
            reason = "r indexes pole rows and every basis row"
        )]
        for r in 0..=p {
            let row = &self.poles.row(iu + r)[jv..=sv];
            match w {
                None => {
                    for (s, &pt) in row.iter().enumerate() {
                        for k in 0..=d {
                            temp[k][s] += pt * nu[k][r];
                        }
                    }
                }
                Some(w) => {
                    let wrow = &w.row(iu + r)[jv..=sv];
                    for (s, (&pt, &wi)) in row.iter().zip(wrow).enumerate() {
                        let pw = pt * wi;
                        for k in 0..=d {
                            temp[k][s] += pw * nu[k][r];
                            tw[k][s] += wi * nu[k][r];
                        }
                    }
                }
            }
        }
        for k in 0..=d {
            for l in 0..=(d - k) {
                let mut acc = Vec3::ZERO;
                for s in 0..=q {
                    acc += temp[k][s] * nv[l][s];
                }
                a[k][l] = acc;
                if w.is_some() {
                    let mut accw = 0.0;
                    for s in 0..=q {
                        accw += tw[k][s] * nv[l][s];
                    }
                    wd[k][l] = accw;
                }
            }
        }
        if w.is_none() {
            return SurfaceDerivs { d: a, order: d };
        }
        let mut skl = [[Vec3::ZERO; 3]; 3];
        let inv = 1.0 / wd[0][0];
        for k in 0..=d {
            for l in 0..=(d - k) {
                let mut val = a[k][l];
                for j in 1..=l {
                    val -= skl[k][l - j] * (BIN[l][j] * wd[0][j]);
                }
                for i in 1..=k {
                    val -= skl[k - i][l] * (BIN[k][i] * wd[i][0]);
                    let mut v2 = Vec3::ZERO;
                    for j in 1..=l {
                        v2 += skl[k - i][l - j] * (BIN[l][j] * wd[i][j]);
                    }
                    val -= v2 * BIN[k][i];
                }
                skl[k][l] = val * inv;
            }
        }
        SurfaceDerivs { d: skl, order: d }
    }

    /// Unit normal `S_u × S_v`. At a degenerate point (collapsed edge such as a sphere pole,
    /// where one partial is below 1e-12 of the other, or `S_u × S_v` is below `tol.angular`
    /// relative to `|S_u||S_v|`) the normal is taken from a nearby point stepped toward the
    /// domain centre (relative steps 1e-9 … 1e-3); `None` if that fails too.
    pub fn normal(&self, u: f64, v: f64, tol: &Tolerance) -> Option<Dir3> {
        let [(u0, u1), (v0, v1)] = self.domain();
        let (uc, vc) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
        for h in [0.0, 1e-9, 1e-7, 1e-5, 1e-3] {
            let uu = u.clamp(u0, u1) + (uc - u.clamp(u0, u1)) * h;
            let vv = v.clamp(v0, v1) + (vc - v.clamp(v0, v1)) * h;
            let d = self.derivs(uu, vv, 1);
            let (su, sv) = (d.d[1][0], d.d[0][1]);
            let n = su.cross(sv);
            let len = n.length();
            let (lu, lv) = (su.length(), sv.length());
            // A collapsed edge leaves rounding noise in one partial: require both to be
            // significant relative to each other, then a non-parallel pair.
            let significant = lu > 1e-12 * lv && lv > 1e-12 * lu;
            if len.is_finite() && significant && len > tol.angular * lu * lv {
                return Some(Dir3::new_unchecked(n / len));
            }
        }
        None
    }

    /// Surface with every pole transformed (affine invariance; weights unchanged).
    pub fn transformed(&self, t: &Transform) -> Self {
        NurbsSurface {
            poles: self.poles.map(|&p| t.transform_point3(p)),
            ..self.clone()
        }
    }

    /// Control-hull bound (conservative).
    pub fn bbox(&self) -> Aabb3 {
        Aabb3::from_points(self.poles.as_slice().iter().copied())
    }
}

/// Builds a surface from validated-by-construction data (used by [`crate::analytic`]).
pub(crate) fn surface_unchecked(
    degree: [usize; 2],
    knots: [KnotVector; 2],
    poles: Grid<Point3>,
    weights: Option<Grid<f64>>,
) -> NurbsSurface3 {
    NurbsSurface::new_unchecked(degree, knots, poles, weights)
}

#[cfg(test)]
#[allow(clippy::float_cmp, reason = "exact expected values")]
mod tests {
    use super::*;

    fn bilinear() -> NurbsSurface3 {
        NurbsSurface::new(
            [1, 1],
            [
                KnotVector::new(vec![0., 0., 1., 1.]).unwrap(),
                KnotVector::new(vec![0., 0., 1., 1.]).unwrap(),
            ],
            Grid::new(
                2,
                2,
                vec![
                    Point3::ZERO,
                    Point3::Y,
                    Point3::X,
                    Point3::new(1.0, 1.0, 1.0),
                ],
            )
            .unwrap(),
            None,
        )
        .unwrap()
    }

    #[test]
    fn bilinear_eval_and_derivs() {
        let s = bilinear();
        let p = s.eval(0.5, 0.25);
        assert!((p - Point3::new(0.5, 0.25, 0.125)).length() < 1e-15);
        let d = s.derivs(0.5, 0.25, 2);
        assert!((d.d[1][0] - Vec3::new(1.0, 0.0, 0.25)).length() < 1e-15);
        assert!((d.d[0][1] - Vec3::new(0.0, 1.0, 0.5)).length() < 1e-15);
        assert!((d.d[1][1] - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-15);
        assert_eq!(d.d[2][0], Vec3::ZERO);
        assert!(s.normal(0.0, 0.0, &Tolerance::DEFAULT).is_some());
    }

    #[test]
    fn iso_boundary_reverse_transpose() {
        let s = bilinear();
        let iso = s.iso_curve(UV::U, 0.5);
        assert!((iso.eval(0.25) - s.eval(0.5, 0.25)).length() < 1e-15);
        let iso = s.iso_curve(UV::V, 0.25);
        assert!((iso.eval(0.5) - s.eval(0.5, 0.25)).length() < 1e-15);
        assert_eq!(
            s.boundary(Side::UMax).poles(),
            &[Point3::X, Point3::new(1., 1., 1.)]
        );
        assert_eq!(s.boundary(Side::VMin).poles(), &[Point3::ZERO, Point3::X]);
        let r = s.reversed(UV::V);
        assert!((r.eval(0.5, 0.75) - s.eval(0.5, 0.25)).length() < 1e-15);
        let t = s.transposed();
        assert!((t.eval(0.25, 0.5) - s.eval(0.5, 0.25)).length() < 1e-15);
    }
}
