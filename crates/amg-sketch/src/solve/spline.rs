//! Residual rows of splines and conics (SK-08).
//!
//! A spline's poles are a **linear map** of point params, `P_i = Σ_j m_ij q_j` (the identity for
//! control-point splines and conics, the interpolation map of
//! [`FitMap`](crate::geom::FitMap) for fit-point splines), and its weights are params (rational
//! control-point splines), the conic weight `ρ/(1 − ρ)` of the `rho` param, or 1. Every row is a
//! function of the poles and weights, so its Jacobian is the curve derivative followed by this
//! map.
//!
//! | Constraint | Rows |
//! |---|---|
//! | Point on spline / conic | `p − C(t)` (x, y), `t` = the constraint's aux param (`C` extends polynomially beyond `[0, L]`) |
//! | Tangent `AtEndpoint` with a spline or conic | `L_c · wrap(angle(t_a, σ t_b))`, the direction at a spline start is `P₁ − P₀`, at its end `Pₙ − Pₙ₋₁` |
//! | Curvature (G2) | the tangent row and `½ L_c² (κ_a − σ κ_b)`, `κ` the signed curvature at the shared end |
//! | Equal conic–conic / rational spline–spline | `ρ_a − ρ_b` / `w_a,i − w_b,i` |
//!
//! The end curvature of a clamped rational spline depends only on its first three homogeneous
//! poles ([`start_curvature`](crate::geom)); the rows use its forward-mode gradient.

#![allow(
    clippy::needless_range_loop,
    reason = "numeric kernels index several arrays in lockstep"
)]

use amg_core::{Point2, Vec2};
use smallvec::SmallVec;

use super::func::wrap_pi;
use super::system::Residual;
use crate::geom::{BSpline2, start_curvature};
use crate::ids::ParamId;
use crate::model::End;

/// A point entity's `x, y` params.
pub(crate) type P2 = [ParamId; 2];

/// Where the pole weights come from.
#[derive(Clone, Debug)]
pub(crate) enum WSrc {
    /// All weights are 1.
    Ones,
    /// One weight param per pole.
    Params(Vec<ParamId>),
    /// A conic: weights `(1, ρ/(1 − ρ), 1)` from the `rho` param.
    Rho(ParamId),
}

/// A spline or conic as the solver sees it.
#[derive(Clone, Debug)]
pub(crate) struct Spl {
    pub degree: usize,
    /// Knots on `[0, L]`.
    pub knots: Vec<f64>,
    /// Knots of the reversed curve (`L − k`, reversed).
    pub knots_rev: Vec<f64>,
    /// Input points (fit points / poles / conic points, then fit tangent handles).
    pub pts: Vec<P2>,
    /// `m[i][j]`: pole `i` = `Σ_j m[i][j] · pts[j]`.
    pub m: Vec<Vec<f64>>,
    pub w: WSrc,
}

impl Spl {
    pub fn new(degree: usize, knots: Vec<f64>, pts: Vec<P2>, m: Vec<Vec<f64>>, w: WSrc) -> Spl {
        let k0 = knots[0];
        let knots: Vec<f64> = knots.into_iter().map(|k| k - k0).collect();
        let span = knots[knots.len() - 1];
        let knots_rev = knots.iter().rev().map(|&k| span - k).collect();
        Spl {
            degree,
            knots,
            knots_rev,
            pts,
            m,
            w,
        }
    }

    fn n_poles(&self) -> usize {
        self.m.len()
    }

    /// Nonzero `(point index, coefficient)` of pole `i`.
    fn deps(&self, i: usize) -> impl Iterator<Item = (usize, f64)> + '_ {
        self.m[i]
            .iter()
            .copied()
            .enumerate()
            .filter(|(_, c)| c.abs() > 0.0)
    }

    /// Every param the pole set `poles` (indices) depends on.
    fn params_of(&self, poles: impl Iterator<Item = usize>, out: &mut Vec<ParamId>) {
        for i in poles {
            for (j, _) in self.deps(i) {
                out.extend(self.pts[j]);
            }
            match &self.w {
                WSrc::Params(ps) => out.push(ps[i]),
                WSrc::Rho(r) if i == 1 => out.push(*r),
                _ => {}
            }
        }
    }

    fn all_params(&self, out: &mut Vec<ParamId>) {
        self.params_of(0..self.n_poles(), out);
    }

    fn pole(&self, i: usize, x: &[f64]) -> Point2 {
        let mut p = Vec2::ZERO;
        for (j, c) in self.deps(i) {
            p += Vec2::new(x[self.pts[j][0].index()], x[self.pts[j][1].index()]) * c;
        }
        p
    }

    fn weight(&self, i: usize, x: &[f64]) -> f64 {
        match &self.w {
            WSrc::Ones => 1.0,
            WSrc::Params(ps) => x[ps[i].index()],
            WSrc::Rho(r) => {
                if i == 1 {
                    let rho = x[r.index()];
                    rho / (1.0 - rho)
                } else {
                    1.0
                }
            }
        }
    }

    /// `(param, d weight_i / d param)`, if the weight depends on a param.
    fn dweight(&self, i: usize, x: &[f64]) -> Option<(ParamId, f64)> {
        match &self.w {
            WSrc::Ones => None,
            WSrc::Params(ps) => Some((ps[i], 1.0)),
            WSrc::Rho(r) => (i == 1).then(|| {
                let rho = x[r.index()];
                (*r, 1.0 / ((1.0 - rho) * (1.0 - rho)))
            }),
        }
    }

    fn curve(&self, x: &[f64]) -> BSpline2 {
        let n = self.n_poles();
        BSpline2::unchecked(
            self.degree,
            self.knots.clone(),
            (0..n).map(|i| self.pole(i, x)).collect(),
            (0..n).map(|i| self.weight(i, x)).collect(),
        )
    }
}

// ---- Dense local Jacobian assembly -------------------------------------------------------------

/// Sorted distinct params and the column of each.
#[derive(Clone, Debug)]
struct Cols(SmallVec<[ParamId; 8]>);

impl Cols {
    fn new(mut v: Vec<ParamId>) -> Cols {
        v.sort_unstable();
        v.dedup();
        Cols(v.into_iter().collect())
    }
    fn of(&self, p: ParamId) -> usize {
        self.0.binary_search(&p).unwrap_or(0)
    }
}

// ---- Point on spline ---------------------------------------------------------------------------

/// What a spline residual block computes.
#[derive(Clone, Debug)]
enum Kind {
    /// `p − C(t)`.
    PointOn { spl: Spl, p: P2, t: ParamId },
    /// Tangent (and with `curvature` also G2) at shared ends.
    Ends {
        a: Side,
        b: Side,
        reversed: bool,
        curvature: bool,
        lc: f64,
    },
}

/// One side of an end-point tangent / curvature constraint.
#[derive(Clone, Debug)]
pub(crate) enum Side {
    /// A line: direction `end − start`, curvature 0.
    Line { a: P2, b: P2 },
    /// A CCW arc at `p`: direction `perp(p − c)`, curvature `1/|p − c|`.
    Arc { c: P2, p: P2 },
    /// A spline or conic at one of its ends.
    Spline { spl: Spl, end: End },
}

/// A spline/conic residual block (see the [module docs](self)).
#[derive(Clone, Debug)]
pub(crate) struct SplineRes {
    cols: Cols,
    kind: Kind,
}

impl SplineRes {
    /// `p − C(t)` (2 rows).
    pub fn point_on(spl: Spl, p: P2, t: ParamId) -> SplineRes {
        let mut v = vec![p[0], p[1], t];
        spl.all_params(&mut v);
        SplineRes {
            cols: Cols::new(v),
            kind: Kind::PointOn { spl, p, t },
        }
    }

    /// Tangent row (and the curvature row if `curvature`) at the named ends.
    pub fn ends(a: Side, b: Side, reversed: bool, curvature: bool, lc: f64) -> SplineRes {
        let mut v = Vec::new();
        a.params(&mut v);
        b.params(&mut v);
        SplineRes {
            cols: Cols::new(v),
            kind: Kind::Ends {
                a,
                b,
                reversed,
                curvature,
                lc,
            },
        }
    }
}

impl Side {
    fn params(&self, out: &mut Vec<ParamId>) {
        match self {
            Side::Line { a, b } => {
                out.extend(*a);
                out.extend(*b);
            }
            Side::Arc { c, p } => {
                out.extend(*c);
                out.extend(*p);
            }
            Side::Spline { spl, end } => {
                let (idx, _) = end_poles(spl, *end);
                spl.params_of(idx.into_iter(), out);
            }
        }
    }
}

/// Pole indices `[e0, e1, e2]` counted from the named end, and the sign of the direction
/// `P[e1] − P[e0]` along the direction of travel.
fn end_poles(spl: &Spl, end: End) -> ([usize; 3], f64) {
    let n = spl.n_poles();
    match end {
        End::Start => ([0, 1, 2.min(n - 1)], 1.0),
        End::End => ([n - 1, n - 2, n.saturating_sub(3)], -1.0),
    }
}

/// Direction of travel and signed curvature at a side, with their sparse gradients.
struct SideEval {
    dir: Vec2,
    ddir: SmallVec<[(ParamId, Vec2); 12]>,
    k: f64,
    dk: SmallVec<[(ParamId, f64); 12]>,
}

impl Side {
    fn eval(&self, x: &[f64]) -> SideEval {
        let get = |p: ParamId| x[p.index()];
        match self {
            Side::Line { a, b } => SideEval {
                dir: Vec2::new(get(b[0]) - get(a[0]), get(b[1]) - get(a[1])),
                ddir: SmallVec::from_slice(&[
                    (a[0], Vec2::new(-1.0, 0.0)),
                    (a[1], Vec2::new(0.0, -1.0)),
                    (b[0], Vec2::new(1.0, 0.0)),
                    (b[1], Vec2::new(0.0, 1.0)),
                ]),
                k: 0.0,
                dk: SmallVec::new(),
            },
            Side::Arc { c, p } => {
                let (dx, dy) = (get(p[0]) - get(c[0]), get(p[1]) - get(c[1]));
                let r = dx.hypot(dy);
                let (k, gx, gy) = if r > 0.0 {
                    (1.0 / r, -dx / (r * r * r), -dy / (r * r * r))
                } else {
                    (0.0, 0.0, 0.0)
                };
                SideEval {
                    // perp(p − c) = (−dy, dx)
                    dir: Vec2::new(-dy, dx),
                    ddir: SmallVec::from_slice(&[
                        (c[0], Vec2::new(0.0, -1.0)),
                        (c[1], Vec2::new(1.0, 0.0)),
                        (p[0], Vec2::new(0.0, 1.0)),
                        (p[1], Vec2::new(-1.0, 0.0)),
                    ]),
                    k,
                    dk: SmallVec::from_slice(&[(p[0], gx), (p[1], gy), (c[0], -gx), (c[1], -gy)]),
                }
            }
            Side::Spline { spl, end } => {
                let (idx, sign) = end_poles(spl, *end);
                let pole = |i: usize| spl.pole(i, x);
                let dir = (pole(idx[1]) - pole(idx[0])) * sign;
                let mut ddir: SmallVec<[(ParamId, Vec2); 12]> = SmallVec::new();
                for (i, s) in [(idx[1], sign), (idx[0], -sign)] {
                    for (j, c) in spl.deps(i) {
                        ddir.push((spl.pts[j][0], Vec2::new(s * c, 0.0)));
                        ddir.push((spl.pts[j][1], Vec2::new(0.0, s * c)));
                    }
                }
                let mut dk: SmallVec<[(ParamId, f64); 12]> = SmallVec::new();
                let mut k = 0.0;
                if spl.degree >= 2 && spl.n_poles() >= 3 {
                    let mut v = [0.0; 9];
                    for (s, &i) in idx.iter().enumerate() {
                        let p = pole(i);
                        v[3 * s] = p.x;
                        v[3 * s + 1] = p.y;
                        v[3 * s + 2] = spl.weight(i, x);
                    }
                    let (knots, flip) = match end {
                        End::Start => (&spl.knots, 1.0),
                        End::End => (&spl.knots_rev, -1.0),
                    };
                    let (kv, g) = start_curvature(spl.degree, knots, &v);
                    k = flip * kv;
                    for (s, &i) in idx.iter().enumerate() {
                        let (gx, gy, gw) =
                            (flip * g[3 * s], flip * g[3 * s + 1], flip * g[3 * s + 2]);
                        for (j, c) in spl.deps(i) {
                            dk.push((spl.pts[j][0], gx * c));
                            dk.push((spl.pts[j][1], gy * c));
                        }
                        if let Some((pid, dw)) = spl.dweight(i, x) {
                            dk.push((pid, gw * dw));
                        }
                    }
                }
                SideEval { dir, ddir, k, dk }
            }
        }
    }
}

/// `θ = atan2(u × v, u · v)` and its gradient with respect to `u` and `v`.
fn angle(u: Vec2, v: Vec2) -> (f64, Vec2, Vec2) {
    let (c, d) = (u.perp_dot(v), u.dot(v));
    let n = u.length_squared() * v.length_squared();
    if n <= 0.0 {
        return (0.0, Vec2::ZERO, Vec2::ZERO);
    }
    let du = (Vec2::new(v.y, -v.x) * d - v * c) / n;
    let dv = (Vec2::new(-u.y, u.x) * d - u * c) / n;
    (c.atan2(d), du, dv)
}

impl Residual for SplineRes {
    fn rows(&self) -> usize {
        match &self.kind {
            Kind::PointOn { .. } => 2,
            Kind::Ends { curvature, .. } => 1 + usize::from(*curvature),
        }
    }

    fn params(&self) -> &[ParamId] {
        &self.cols.0
    }

    fn eval(&self, x: &[f64], out: &mut [f64]) {
        match &self.kind {
            Kind::PointOn { spl, p, t } => {
                let c = spl.curve(x).derivs_ext(x[t.index()]).0;
                out[0] = x[p[0].index()] - c.x;
                out[1] = x[p[1].index()] - c.y;
            }
            Kind::Ends {
                a,
                b,
                reversed,
                curvature,
                lc,
            } => {
                let (ea, eb) = (a.eval(x), b.eval(x));
                let (th, _, _) = angle(ea.dir, eb.dir);
                let offset = if *reversed {
                    -std::f64::consts::PI
                } else {
                    0.0
                };
                out[0] = lc * wrap_pi(th + offset);
                if *curvature {
                    let kb = if *reversed { -eb.k } else { eb.k };
                    out[1] = curv_scale(*lc) * (ea.k - kb);
                }
            }
        }
    }

    fn jacobian(&self, x: &[f64], out: &mut [f64]) {
        let n = self.cols.0.len();
        out[..self.rows() * n].fill(0.0);
        match &self.kind {
            Kind::PointOn { spl, p, t } => {
                let curve = spl.curve(x);
                let tv = x[t.index()];
                let (c, d1, _) = curve.derivs_ext(tv);
                let (span, nb) = curve.basis_at(tv);
                let wsum: f64 = (0..=spl.degree)
                    .map(|j| nb[j] * curve.weights()[span - spl.degree + j])
                    .sum();
                out[self.cols.of(p[0])] += 1.0;
                out[n + self.cols.of(p[1])] += 1.0;
                out[self.cols.of(*t)] -= d1.x;
                out[n + self.cols.of(*t)] -= d1.y;
                for j in 0..=spl.degree {
                    let i = span - spl.degree + j;
                    let wi = curve.weights()[i];
                    let r = nb[j] * wi / wsum;
                    for (q, coef) in spl.deps(i) {
                        out[self.cols.of(spl.pts[q][0])] -= r * coef;
                        out[n + self.cols.of(spl.pts[q][1])] -= r * coef;
                    }
                    if let Some((pid, dw)) = spl.dweight(i, x) {
                        let dc = (curve.poles()[i] - c) * (nb[j] / wsum);
                        out[self.cols.of(pid)] -= dc.x * dw;
                        out[n + self.cols.of(pid)] -= dc.y * dw;
                    }
                }
            }
            Kind::Ends {
                a,
                b,
                reversed,
                curvature,
                lc,
            } => {
                let (ea, eb) = (a.eval(x), b.eval(x));
                let (_, du, dv) = angle(ea.dir, eb.dir);
                for (p, d) in &ea.ddir {
                    out[self.cols.of(*p)] += lc * du.dot(*d);
                }
                for (p, d) in &eb.ddir {
                    out[self.cols.of(*p)] += lc * dv.dot(*d);
                }
                if *curvature {
                    let s = curv_scale(*lc);
                    let sb = if *reversed { 1.0 } else { -1.0 };
                    for (p, g) in &ea.dk {
                        out[n + self.cols.of(*p)] += s * g;
                    }
                    for (p, g) in &eb.dk {
                        out[n + self.cols.of(*p)] += s * sb * g;
                    }
                }
            }
        }
    }
}

/// Scale of the curvature row: `½ L_c²` is the deviation (mm) of two curves with curvatures
/// differing by `δκ` a characteristic length from the joint.
fn curv_scale(lc: f64) -> f64 {
    0.5 * lc * lc
}
