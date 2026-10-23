//! Ellipses and elliptical arcs (SK-22): evaluation, queries and intersection.
//!
//! **Parameterisation.** An [`EllipseArc2`] is the set
//! `P(θ) = c + m·cos θ + w·sin θ`, `θ` running from `t0` to `t1`, where `m` is the semi-major
//! vector, `w = perp(m)·(b / |m|)` the semi-minor vector (CCW from `m`, `b` the minor radius)
//! and `θ` the *eccentric* angle (not the polar angle). `t1 < t0` traverses the arc CW; a span
//! of `2π` is the full ellipse.
//!
//! As for every [`Curve2`], the curve parameter `s` used by `eval`, `trimmed` and `intersect`
//! runs from `0` to `|t1 − t0|` in the direction of travel: `θ(s) = t0 + sign(t1 − t0)·s`.
//!
//! The minor radius may exceed `|m|` (the "major" axis is then the shorter one): the curve is
//! still a valid ellipse, and the dimension rows measure `|m|` and `b` as they are.

use std::f64::consts::{PI, TAU};
use std::sync::OnceLock;

use amg_core::{Aabb2, Point2, Tolerance, Vec2};
use smallvec::SmallVec;

use super::query::Intersection;
use super::{Curve2, CurveHit};

/// Ellipse or elliptical arc (see the [module docs](self) for the parameterisation).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EllipseArc2 {
    /// Centre, mm.
    pub center: Point2,
    /// Semi-major vector `m` (centre → major-axis end point), mm.
    pub major: Vec2,
    /// Minor radius `b`, mm.
    pub minor_radius: f64,
    /// Eccentric angle at the start, rad.
    pub t0: f64,
    /// Eccentric angle at the end, rad (`t1 − t0 = ±2π` is a full ellipse).
    pub t1: f64,
}

impl EllipseArc2 {
    /// The full ellipse (`t0 = 0`, `t1 = 2π`).
    pub fn full(center: Point2, major: Vec2, minor_radius: f64) -> EllipseArc2 {
        EllipseArc2 {
            center,
            major,
            minor_radius,
            t0: 0.0,
            t1: TAU,
        }
    }

    /// Semi-major length `|m|`, mm.
    pub fn semi_major(&self) -> f64 {
        self.major.length()
    }

    /// Semi-minor vector `w` (`perp(m)` scaled to length `b`); zero if `m` is zero.
    pub fn minor(&self) -> Vec2 {
        let a = self.major.length();
        if a > 0.0 {
            self.major.perp() * (self.minor_radius / a)
        } else {
            Vec2::ZERO
        }
    }

    /// Point at eccentric angle `theta`.
    pub fn point_at(&self, theta: f64) -> Point2 {
        self.center + self.major * theta.cos() + self.minor() * theta.sin()
    }

    /// `dP/dθ` at eccentric angle `theta`.
    pub fn tangent_at(&self, theta: f64) -> Vec2 {
        self.minor() * theta.cos() - self.major * theta.sin()
    }

    /// Eccentric angle `θ` of a point `p` on the ellipse (`cos θ = (p − c)·m / a²`,
    /// `sin θ = (p − c)·w / b²`; for a point off the curve, the angle of its radial projection).
    pub fn angle_of(&self, p: Point2) -> f64 {
        let d = p - self.center;
        let (a, b) = (self.semi_major(), self.minor_radius);
        if a <= 0.0 || b <= 0.0 {
            return 0.0;
        }
        let w = self.minor();
        (d.dot(w) / (b * b)).atan2(d.dot(self.major) / (a * a))
    }

    /// Signed sweep `t1 − t0`, rad.
    pub fn sweep(&self) -> f64 {
        self.t1 - self.t0
    }

    /// Direction of travel in `θ`: `+1` (CCW) or `−1`.
    pub fn sign(&self) -> f64 {
        if self.t1 < self.t0 { -1.0 } else { 1.0 }
    }

    /// Parameter span `|t1 − t0|`, rad.
    pub fn span(&self) -> f64 {
        self.sweep().abs()
    }

    /// True for a full ellipse (span at least `2π` within 1e-12).
    pub fn is_full(&self) -> bool {
        self.span() >= TAU - 1e-12
    }

    /// Eccentric angle at curve parameter `s`.
    pub(crate) fn angle(&self, s: f64) -> f64 {
        self.t0 + self.sign() * s
    }

    /// Curve parameter of eccentric angle `theta`, accepting angles up to `eps` rad outside
    /// the arc (clamped onto it).
    pub(crate) fn param_of_angle(&self, theta: f64, eps: f64) -> Option<f64> {
        let span = self.span();
        let s = (self.sign() * (theta - self.t0)).rem_euclid(TAU);
        if s <= span + eps {
            Some(s.min(span))
        } else if s >= TAU - eps {
            Some(0.0)
        } else {
            None
        }
    }

    /// Smallest of the two radii: a lower bound of the speed `|dP/dθ|`.
    fn min_radius(&self) -> f64 {
        self.semi_major().min(self.minor_radius)
    }

    pub(crate) fn eval(&self, s: f64) -> Point2 {
        self.point_at(self.angle(s))
    }

    pub(crate) fn derivative(&self, s: f64) -> Vec2 {
        self.tangent_at(self.angle(s)) * self.sign()
    }

    /// Signed curvature at curve parameter `s` (positive = turning left), 1/mm.
    pub(crate) fn curvature_at(&self, s: f64) -> f64 {
        let th = self.angle(s);
        let (a, b) = (self.semi_major(), self.minor_radius);
        let speed2 = a * a * th.sin().powi(2) + b * b * th.cos().powi(2);
        if speed2 <= 0.0 {
            return 0.0;
        }
        self.sign() * a * b / (speed2 * speed2.sqrt())
    }

    /// Closest point to `p`: `(s, point)`. Samples the arc, then bisects every sign change of
    /// the squared-distance derivative, so the global minimum is found.
    pub(crate) fn closest(&self, p: Point2) -> (f64, Point2) {
        let span = self.span();
        let sg = self.sign();
        let pt = |s: f64| self.point_at(self.t0 + sg * s);
        if span <= 0.0 {
            return (0.0, pt(0.0));
        }
        let g = |s: f64| {
            let th = self.t0 + sg * s;
            (self.point_at(th) - p).dot(self.tangent_at(th)) * sg
        };
        let n = ((span / (PI / 64.0)).ceil() as usize).clamp(8, 256);
        let mut best = (f64::INFINITY, 0.0);
        let mut consider = |s: f64| {
            let d = pt(s).distance_squared(p);
            if d < best.0 {
                best = (d, s);
            }
        };
        consider(0.0);
        consider(span);
        let (mut s_prev, mut g_prev) = (0.0, g(0.0));
        for i in 1..=n {
            let s = span * i as f64 / n as f64;
            let gi = g(s);
            if g_prev < 0.0 && gi >= 0.0 {
                let (mut lo, mut hi) = (s_prev, s);
                for _ in 0..70 {
                    let mid = 0.5 * (lo + hi);
                    if g(mid) < 0.0 {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                consider(0.5 * (lo + hi));
            }
            s_prev = s;
            g_prev = gi;
        }
        (best.1, pt(best.1))
    }

    /// Tight bounding box.
    pub(crate) fn bbox(&self) -> Aabb2 {
        let span = self.span();
        let mut b = Aabb2::from_points([self.eval(0.0), self.eval(span)]);
        let w = self.minor();
        for base in [w.x.atan2(self.major.x), w.y.atan2(self.major.y)] {
            for k in 0..2 {
                let th = base + f64::from(k) * PI;
                if self.param_of_angle(th, 0.0).is_some() {
                    b.include_point(self.point_at(th));
                }
            }
        }
        b
    }

    /// Arc length, mm (Gauss–Legendre, refined until it converges).
    pub(crate) fn length(&self) -> f64 {
        let (a, b) = (self.semi_major(), self.minor_radius);
        let speed = |t: f64| (a * a * t.sin().powi(2) + b * b * t.cos().powi(2)).sqrt();
        let span = self.span();
        let (lo, hi) = (self.t0.min(self.t1), self.t0.max(self.t1));
        let mut panels = ((span / (PI / 8.0)).ceil() as usize).max(2);
        let mut prev = integrate(speed, lo, hi, panels);
        while panels < 8192 {
            panels *= 2;
            let next = integrate(speed, lo, hi, panels);
            let done = (next - prev).abs() <= 1e-13 * next.abs();
            prev = next;
            if done {
                break;
            }
        }
        prev
    }
}

/// 12-point Gauss–Legendre nodes and weights on `[-1, 1]`.
fn gl_nodes() -> &'static [(f64, f64)] {
    static NODES: OnceLock<Vec<(f64, f64)>> = OnceLock::new();
    NODES.get_or_init(|| {
        const N: usize = 12;
        let legendre = |x: f64| {
            let (mut p0, mut p1) = (1.0, x);
            for k in 2..=N {
                let kf = k as f64;
                let p2 = ((2.0 * kf - 1.0) * x * p1 - (kf - 1.0) * p0) / kf;
                p0 = p1;
                p1 = p2;
            }
            (p1, N as f64 * (x * p1 - p0) / (x * x - 1.0))
        };
        (0..N)
            .map(|i| {
                let mut x = (PI * (i as f64 + 0.75) / (N as f64 + 0.5)).cos();
                for _ in 0..100 {
                    let (p, dp) = legendre(x);
                    let dx = p / dp;
                    x -= dx;
                    if dx.abs() < 1e-15 {
                        break;
                    }
                }
                let (_, dp) = legendre(x);
                (x, 2.0 / ((1.0 - x * x) * dp * dp))
            })
            .collect()
    })
}

/// Composite Gauss–Legendre integral of `f` over `[lo, hi]` with `panels` panels.
pub(crate) fn integrate(f: impl Fn(f64) -> f64, lo: f64, hi: f64, panels: usize) -> f64 {
    let h = (hi - lo) / panels as f64;
    let mut sum = 0.0;
    for k in 0..panels {
        let mid = lo + (k as f64 + 0.5) * h;
        for &(x, w) in gl_nodes() {
            sum += w * f(mid + 0.5 * h * x);
        }
    }
    sum * 0.5 * h
}

// ---- Intersection --------------------------------------------------------------------------

/// The other curve as an implicit function `F(p)` with a gradient, such that `|F|/|∇F|` is
/// (to first order) the distance from the curve.
enum Implicit {
    Line {
        a: Point2,
        u: Vec2,
    },
    Circle {
        c: Point2,
        r: f64,
    },
    Ellipse {
        c: Point2,
        u: Vec2,
        v: Vec2,
        a: f64,
        b: f64,
    },
}

impl Implicit {
    fn of(curve: &Curve2) -> Option<Implicit> {
        match curve {
            Curve2::Line(s) => {
                let d = s.b - s.a;
                let len = d.length();
                (len > 0.0).then(|| Implicit::Line { a: s.a, u: d / len })
            }
            Curve2::Circle(c) => Some(Implicit::Circle {
                c: c.center,
                r: c.radius,
            }),
            Curve2::Arc(a) => Some(Implicit::Circle {
                c: a.center,
                r: a.radius,
            }),
            Curve2::Ellipse(e) => {
                let a = e.semi_major();
                (a > 0.0 && e.minor_radius > 0.0).then(|| {
                    let u = e.major / a;
                    Implicit::Ellipse {
                        c: e.center,
                        u,
                        v: u.perp(),
                        a,
                        b: e.minor_radius,
                    }
                })
            }
        }
    }

    fn value(&self, p: Point2) -> (f64, Vec2) {
        match *self {
            Implicit::Line { a, u } => (u.perp_dot(p - a), u.perp()),
            Implicit::Circle { c, r } => {
                let d = p - c;
                (d.length_squared() - r * r, d * 2.0)
            }
            Implicit::Ellipse { c, u, v, a, b } => {
                let d = p - c;
                let (x, y) = (d.dot(u), d.dot(v));
                let f = (x / a).powi(2) + (y / b).powi(2) - 1.0;
                (f, u * (2.0 * x / (a * a)) + v * (2.0 * y / (b * b)))
            }
        }
    }

    fn is_conic(&self) -> bool {
        !matches!(self, Implicit::Line { .. })
    }
}

/// Parameter of the point `p` (on the curve's carrier) on `curve`, if it lies within the
/// curve's range up to the tolerance `lin`.
fn param_on(curve: &Curve2, p: Point2, lin: f64) -> Option<f64> {
    match curve {
        Curve2::Line(s) => {
            let d = s.b - s.a;
            let len = d.length();
            if len <= 0.0 {
                return None;
            }
            let t = (p - s.a).dot(d) / (len * len);
            let et = lin / len;
            (-et..=1.0 + et).contains(&t).then(|| t.clamp(0.0, 1.0))
        }
        Curve2::Ellipse(e) => {
            let eps = lin / e.min_radius().max(f64::MIN_POSITIVE);
            e.param_of_angle(e.angle_of(p), eps)
        }
        _ => {
            let r = curve.round()?;
            let v = p - r.c;
            r.param_of_angle(v.y.atan2(v.x), lin / r.r)
        }
    }
}

fn bisect(f: impl Fn(f64) -> f64, mut lo: f64, mut hi: f64) -> f64 {
    let flo = f(lo);
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if (f(mid) < 0.0) == (flo < 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// Eccentric angles in `[0, 2π)` where the full ellipse of `e` meets `imp`, with a flag for
/// tangential contacts (touching within `lin`).
///
/// `f(θ) = F(P(θ))` is a trigonometric polynomial of degree 2. The roots of `f'` split the
/// circle into monotone pieces (at most one root each); an extremum of `f` within `lin` of the
/// curve is a tangential contact.
fn roots(e: &EllipseArc2, imp: &Implicit, lin: f64) -> SmallVec<[(f64, bool); 8]> {
    const N: usize = 720;
    let f = |t: f64| imp.value(e.point_at(t)).0;
    let g = |t: f64| imp.value(e.point_at(t)).1.dot(e.tangent_at(t));
    let mut crit: SmallVec<[f64; 8]> = SmallVec::new();
    let (mut t_prev, mut g_prev) = (0.0, g(0.0));
    for i in 1..=N {
        let t = TAU * i as f64 / N as f64;
        let gi = g(t);
        if (g_prev < 0.0 && gi > 0.0) || (g_prev > 0.0 && gi < 0.0) {
            crit.push(bisect(g, t_prev, t));
        } else if gi == 0.0 && i < N {
            crit.push(t);
        }
        t_prev = t;
        g_prev = gi;
    }
    let mut cuts: SmallVec<[f64; 10]> = SmallVec::new();
    cuts.push(0.0);
    cuts.extend(crit.iter().copied());
    cuts.push(TAU);
    cuts.sort_by(f64::total_cmp);
    let mut out: SmallVec<[(f64, bool); 8]> = SmallVec::new();
    for w in cuts.windows(2) {
        let (flo, fhi) = (f(w[0]), f(w[1]));
        if flo == 0.0 {
            out.push((w[0], false));
        } else if flo * fhi < 0.0 {
            out.push((bisect(f, w[0], w[1]), false));
        }
    }
    for &c in &crit {
        let (fv, gr) = imp.value(e.point_at(c));
        let gl = gr.length();
        if gl > 0.0 && fv.abs() / gl <= lin {
            out.push((c, true));
        }
    }
    out
}

fn push_unique(hits: &mut SmallVec<[CurveHit; 4]>, h: CurveHit, tol: f64) {
    if !(h.ta.is_finite() && h.tb.is_finite() && h.point.is_finite()) {
        return;
    }
    if hits.iter().all(|o| o.point.distance(h.point) > tol) {
        hits.push(h);
    }
}

/// Intersection of the ellipse `e` (the curve `ecurve`) with `other`. With `swap`, `ecurve`
/// is the second curve of the pair (hits get `ta` on `other`, `tb` on the ellipse).
pub(crate) fn intersect(
    e: &EllipseArc2,
    ecurve: &Curve2,
    other: &Curve2,
    tol: &Tolerance,
    swap: bool,
) -> Intersection {
    let lin = tol.linear;
    let mut out = Intersection::default();
    let (a, b) = (e.semi_major(), e.minor_radius);
    if !(a > 0.0 && b > 0.0 && a.is_finite() && b.is_finite()) {
        return out;
    }
    let mk = |te: f64, to: f64, point: Point2, tangential: bool| {
        if swap {
            CurveHit {
                ta: to,
                tb: te,
                point,
                tangential,
            }
        } else {
            CurveHit {
                ta: te,
                tb: to,
                point,
                tangential,
            }
        }
    };
    let imp = Implicit::of(other);
    // Both on the same conic: only the ends can be hits (and the stretch they bound overlaps).
    let same_conic = imp.as_ref().is_some_and(|i| {
        i.is_conic()
            && (0..24).all(|k| {
                let p = e.point_at(TAU * f64::from(k) / 24.0);
                let (fv, gr) = i.value(p);
                let gl = gr.length();
                gl > 0.0 && fv.abs() / gl <= lin
            })
    });
    let eps_e = lin / e.min_radius();
    if let Some(imp) = imp.as_ref().filter(|_| !same_conic) {
        for (theta, tangential) in roots(e, imp, lin) {
            let Some(te) = e.param_of_angle(theta, eps_e) else {
                continue;
            };
            let p = e.point_at(theta);
            if let Some(to) = param_on(other, p, lin) {
                push_unique(&mut out.hits, mk(te, to, p, tangential), lin);
            }
        }
    }
    // End points touching the other curve (T-junctions and tangential approaches).
    let mut ends: SmallVec<[CurveHit; 4]> = SmallVec::new();
    if !e.is_full() {
        for te in [0.0, e.span()] {
            let p = e.eval(te);
            let (to, q) = other.closest(p);
            if q.distance(p) <= lin {
                push_unique(&mut ends, mk(te, to, p, false), lin);
            }
        }
    }
    if !other.is_closed() {
        let t_end = other.param_range().1;
        for to in [0.0, t_end] {
            let p = other.eval(to);
            let (te, q) = ecurve.closest(p);
            if q.distance(p) <= lin {
                push_unique(&mut ends, mk(te, to, p, false), lin);
            }
        }
    }
    if same_conic {
        const M: usize = 64;
        let span = e.span();
        let on = |s: f64| {
            let p = e.eval(s);
            other.closest(p).1.distance(p) <= lin
        };
        let at = |i: usize| span * i as f64 / M as f64;
        out.overlap = span > 0.0
            && (0..M).any(|i| on(at(i)) && on(at(i + 1)) && on(0.5 * (at(i) + at(i + 1))));
        for mut h in ends {
            h.tangential = true;
            out.hits.push(h);
        }
        return out;
    }
    for h in ends {
        push_unique(&mut out.hits, h, lin);
    }
    out
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_PI_2;

    use amg_core::Segment2;

    use super::*;
    use crate::geom::{Arc2, Circle2, intersect, intersect_detailed};

    const TOL: Tolerance = Tolerance::DEFAULT;

    fn ell(cx: f64, cy: f64, mx: f64, my: f64, b: f64) -> Curve2 {
        Curve2::Ellipse(EllipseArc2::full(Point2::new(cx, cy), Vec2::new(mx, my), b))
    }

    fn line(ax: f64, ay: f64, bx: f64, by: f64) -> Curve2 {
        Curve2::Line(Segment2 {
            a: Point2::new(ax, ay),
            b: Point2::new(bx, by),
        })
    }

    fn near(a: Point2, b: Point2) -> bool {
        a.distance(b) < 1e-9
    }

    /// Perimeter of the ellipse with semi-axes 2 and 1.
    const PERIMETER_2_1: f64 = 9.688_448_220_547_672;

    #[test]
    fn eval_derivative_and_orientation() {
        let c = ell(1.0, 2.0, 0.0, 3.0, 1.0); // major along +y
        assert!((c.param_range().1 - TAU).abs() < 1e-15);
        assert!(near(c.eval(0.0), Point2::new(1.0, 5.0)));
        // w = perp(m)·(b/a) = (-1, 0): at t = pi/2 the point is c + w.
        assert!(near(c.eval(FRAC_PI_2), Point2::new(0.0, 2.0)));
        assert!(c.curvature() > 0.0, "CCW travel turns left");
        // The tangent at t = 0 is w.
        assert!(near(c.derivative(0.0), Point2::new(-1.0, 0.0)));
        assert!(c.is_closed());
    }

    #[test]
    fn length_matches_circle_and_known_perimeter() {
        let circle_like = ell(0.0, 0.0, 3.0, 0.0, 3.0);
        assert!((circle_like.length() - 6.0 * PI).abs() < 1e-12);
        let e = ell(0.0, 0.0, 2.0, 0.0, 1.0);
        assert!((e.length() - PERIMETER_2_1).abs() < 1e-11, "{}", e.length());
        // A quarter of a symmetric ellipse is a quarter of the perimeter.
        let q = e.trimmed(0.0, FRAC_PI_2);
        assert!((q.length() - PERIMETER_2_1 / 4.0).abs() < 1e-11);
        // Very flat ellipse: the perimeter tends to 4a.
        let flat = ell(0.0, 0.0, 100.0, 0.0, 0.01);
        assert!((flat.length() - 400.0).abs() < 1e-3);
    }

    #[test]
    fn curvature_at_the_axis_ends() {
        let e = EllipseArc2::full(Point2::ZERO, Vec2::new(2.0, 0.0), 1.0);
        // kappa = a b / (a² sin² + b² cos²)^(3/2): a/b² at the major end, b/a² at the minor.
        assert!((e.curvature_at(0.0) - 2.0).abs() < 1e-12);
        assert!((e.curvature_at(FRAC_PI_2) - 0.25).abs() < 1e-12);
        let rev = Curve2::Ellipse(e).reversed();
        assert!((rev.curvature_at(0.0) + 2.0).abs() < 1e-12);
    }

    #[test]
    fn bbox_of_rotated_ellipse_and_arc() {
        // a = 2, b = 1 rotated by 90 degrees: x extent ±1, y extent ±2.
        let c = ell(1.0, 1.0, 0.0, 2.0, 1.0);
        let b = c.bbox();
        assert!((b.min.x - 0.0).abs() < 1e-12 && (b.max.x - 2.0).abs() < 1e-12);
        assert!((b.min.y + 1.0).abs() < 1e-12 && (b.max.y - 3.0).abs() < 1e-12);
        // The arc from t = 0 to pi/4 does not reach the x extremes.
        let arc = c.trimmed(0.0, FRAC_PI_2 / 2.0);
        let bb = arc.bbox();
        assert!(bb.max.y <= 3.0 + 1e-12 && bb.max.y > 2.9);
        assert!(bb.min.x > 0.2);
    }

    #[test]
    fn closest_point_matches_brute_force() {
        let c = ell(0.5, -0.5, 3.0, 1.0, 1.2);
        for k in 0..40 {
            let ang = f64::from(k) * 0.37;
            let p = Point2::new(0.5 + 6.0 * ang.cos(), -0.5 + 4.0 * ang.sin() * 0.7);
            let (t, q) = c.closest(p);
            assert!(near(c.eval(t), q));
            let brute = (0..20000)
                .map(|i| c.eval(TAU * f64::from(i) / 20000.0).distance(p))
                .fold(f64::INFINITY, f64::min);
            assert!(
                q.distance(p) <= brute + 1e-12,
                "{k}: {} vs {brute}",
                q.distance(p)
            );
            assert!(brute - q.distance(p) < 1e-6);
        }
        // A point at the centre is nearest to a minor-axis end.
        let (_, q) = ell(0.0, 0.0, 2.0, 0.0, 1.0).closest(Point2::ZERO);
        assert!((q.length() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn closest_respects_the_arc_range() {
        let arc = ell(0.0, 0.0, 2.0, 0.0, 1.0).trimmed(0.0, FRAC_PI_2);
        // The nearest ellipse point is near (-2, 0), outside the arc: the arc's end wins.
        let (t, q) = arc.closest(Point2::new(-3.0, 0.0));
        assert!(near(q, Point2::new(0.0, 1.0)) && (t - FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn reversed_and_trimmed_round_trip() {
        let c = ell(0.0, 0.0, 2.0, 0.0, 1.0);
        let r = c.reversed();
        assert!(near(r.eval(0.0), c.eval(TAU)));
        assert!(near(r.eval(1.0), c.eval(TAU - 1.0)));
        let t = c.trimmed(1.0, 0.25);
        assert!(near(t.eval(0.0), c.eval(1.0)));
        assert!(near(t.eval(0.75), c.eval(0.25)));
        assert!((t.param_range().1 - 0.75).abs() < 1e-15);
        assert!(!t.is_closed());
        let u = r.trimmed(0.5, 1.5);
        assert!(near(u.eval(0.0), r.eval(0.5)) && near(u.eval(1.0), r.eval(1.5)));
    }

    #[test]
    fn line_ellipse_secant_tangent_miss() {
        let c = ell(0.0, 0.0, 2.0, 0.0, 1.0);
        let h = intersect(&line(-3.0, 0.0, 3.0, 0.0), &c, &TOL);
        assert_eq!(h.len(), 2);
        assert!(near(h[0].point, Point2::new(-2.0, 0.0)));
        assert!(near(h[1].point, Point2::new(2.0, 0.0)));
        let h = intersect(&line(-3.0, 1.0, 3.0, 1.0), &c, &TOL);
        assert_eq!(h.len(), 1, "tangent at the minor-axis end");
        assert!(h[0].tangential && near(h[0].point, Point2::new(0.0, 1.0)));
        assert!(intersect(&line(-3.0, 1.1, 3.0, 1.1), &c, &TOL).is_empty());
        // Swapped argument order: the same two points, ta on the ellipse.
        let h2 = intersect(&c, &line(-3.0, 0.0, 3.0, 0.0), &TOL);
        assert_eq!(h2.len(), 2);
        assert!(
            h2.iter()
                .all(|h| (h.tb - 1.0 / 6.0).abs() < 1e-9 || (h.tb - 5.0 / 6.0).abs() < 1e-9)
        );
        // The line stops short of the ellipse: no hit; ending on it: T-junction hit.
        assert!(intersect(&line(-3.0, 0.0, -2.1, 0.0), &c, &TOL).is_empty());
        let h = intersect(&line(-3.0, 0.0, -2.0 - 5e-7, 0.0), &c, &TOL);
        assert_eq!(h.len(), 1);
    }

    #[test]
    fn line_ellipse_respects_both_ranges() {
        let arc = ell(0.0, 0.0, 2.0, 0.0, 1.0).trimmed(0.0, PI); // upper half
        let h = intersect(&line(-3.0, 0.5, 3.0, 0.5), &arc, &TOL);
        assert_eq!(h.len(), 2);
        assert!(intersect(&line(-3.0, -0.5, 3.0, -0.5), &arc, &TOL).is_empty());
        // Only one of the two crossings lies on the segment.
        let h = intersect(&line(0.0, 0.5, 3.0, 0.5), &arc, &TOL);
        assert_eq!(h.len(), 1);
        assert!(h[0].point.x > 0.0);
        // The arc end points on the x axis are hit by a line through them.
        let h = intersect(&line(-3.0, 0.0, 3.0, 0.0), &arc, &TOL);
        assert_eq!(h.len(), 2);
    }

    #[test]
    fn circle_and_ellipse_cross_four_times() {
        let c = ell(0.0, 0.0, 3.0, 0.0, 1.0);
        let k = Curve2::Circle(Circle2 {
            center: Point2::ZERO,
            radius: 2.0,
        });
        let h = intersect(&c, &k, &TOL);
        assert_eq!(h.len(), 4);
        for x in &h {
            assert!((x.point.length() - 2.0).abs() < 1e-9);
            let (_, q) = c.closest(x.point);
            assert!(q.distance(x.point) < 1e-9);
        }
        // Concentric circle inside the minor radius: no contact.
        let inner = Curve2::Circle(Circle2 {
            center: Point2::ZERO,
            radius: 0.9,
        });
        assert!(intersect(&c, &inner, &TOL).is_empty());
        // Circle of the minor radius touches at the two minor-axis ends.
        let touch = Curve2::Circle(Circle2 {
            center: Point2::ZERO,
            radius: 1.0,
        });
        let h = intersect(&c, &touch, &TOL);
        assert_eq!(h.len(), 2);
        assert!(h.iter().all(|x| x.tangential));
        // An arc of the circle only keeps the crossings in its range.
        let arc = Curve2::Arc(Arc2 {
            center: Point2::ZERO,
            radius: 2.0,
            start_angle: 0.0,
            sweep: PI,
        });
        assert_eq!(intersect(&arc, &c, &TOL).len(), 2);
    }

    #[test]
    fn two_ellipses_cross_and_overlap() {
        let a = ell(0.0, 0.0, 2.0, 0.0, 1.0);
        let b = ell(0.0, 0.0, 0.0, 2.0, 1.0); // rotated by 90 degrees
        let h = intersect(&a, &b, &TOL);
        assert_eq!(h.len(), 4);
        let s = (4.0f64 / 5.0).sqrt();
        for x in &h {
            assert!((x.point.x.abs() - s).abs() < 1e-9 && (x.point.y.abs() - s).abs() < 1e-9);
        }
        // Shifted copy: two crossings.
        let c = ell(1.0, 0.0, 2.0, 0.0, 1.0);
        assert_eq!(intersect(&a, &c, &TOL).len(), 2);
        // The same ellipse: overlap, no crossings.
        let d = intersect_detailed(&a, &a, &TOL);
        assert!(d.overlap && d.hits.is_empty());
        // A circle-like ellipse equals the circle.
        let same = ell(0.0, 0.0, 1.0, 0.0, 1.0);
        let circ = Curve2::Circle(Circle2 {
            center: Point2::ZERO,
            radius: 1.0,
        });
        assert!(intersect_detailed(&same, &circ, &TOL).overlap);
        assert!(intersect_detailed(&circ, &same, &TOL).overlap);
    }

    #[test]
    fn overlapping_elliptic_arcs_report_the_ends() {
        let full = ell(0.0, 0.0, 2.0, 0.0, 1.0);
        let a = full.trimmed(0.0, PI);
        let b = full.trimmed(FRAC_PI_2, 1.5 * PI);
        let det = intersect_detailed(&a, &b, &TOL);
        assert!(det.overlap);
        assert_eq!(det.hits.len(), 2);
        assert!(det.hits.iter().all(|h| h.tangential));
        // Meeting end to end only: no overlap, one hit.
        let c = full.trimmed(PI, 1.5 * PI);
        let det = intersect_detailed(&a, &c, &TOL);
        assert!(!det.overlap);
        assert_eq!(det.hits.len(), 1);
    }
}
