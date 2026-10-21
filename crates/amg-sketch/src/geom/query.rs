//! Curve queries and tolerance-aware curve–curve intersection (SK-06).
//!
//! **Parameterisation** (`param_range`):
//! - `Line`: `t ∈ [0, 1]`, `eval(t) = a + t·(b − a)`.
//! - `Circle`: `t ∈ [0, 2π]` is the polar angle, CCW from `+x`.
//! - `Arc`: `t ∈ [0, |sweep|]` is the angle travelled from the start point along the arc's
//!   direction, `eval(t) = c + r·(cos θ, sin θ)` with `θ = start_angle + sign(sweep)·t`.
//!
//! The parameter therefore always increases in the curve's direction of travel.

use std::f64::consts::{FRAC_PI_2, TAU};

use amg_core::intersect::{Seg2Intersection, segment2_segment2};
use amg_core::{Aabb2, Point2, Segment2, Tolerance, Vec2};
use smallvec::SmallVec;

use super::{Arc2, Circle2, Curve2};

/// One intersection of two curves: `point ≈ a.eval(ta) ≈ b.eval(tb)`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct CurveHit {
    /// Parameter on the first curve.
    pub ta: f64,
    /// Parameter on the second curve.
    pub tb: f64,
    /// The intersection point, mm.
    pub point: Point2,
    /// The curves touch tangentially here (or this is an end of an overlap).
    pub tangential: bool,
}

/// Intersection result with overlap information (used by profile detection).
#[derive(Clone, Debug, Default)]
pub(crate) struct Intersection {
    /// Hits sorted by `ta`. For overlapping curves these include the overlap's ends.
    pub hits: SmallVec<[CurveHit; 4]>,
    /// The curves share a stretch of positive length (collinear lines, co-circular arcs).
    pub overlap: bool,
}

fn dir(a: f64) -> Vec2 {
    Vec2::new(a.cos(), a.sin())
}

/// A circle or arc as centre, radius and a signed angular span (`±2π` for a circle).
#[derive(Copy, Clone, Debug)]
pub(crate) struct Round {
    pub c: Point2,
    pub r: f64,
    pub start: f64,
    pub sweep: f64,
    /// A `Circle2` (the parameter wraps; no end points).
    pub circle: bool,
}

impl Round {
    pub fn sign(&self) -> f64 {
        if self.sweep < 0.0 { -1.0 } else { 1.0 }
    }
    pub fn span(&self) -> f64 {
        self.sweep.abs()
    }
    pub fn angle(&self, t: f64) -> f64 {
        self.start + self.sign() * t
    }
    pub fn eval(&self, t: f64) -> Point2 {
        self.c + dir(self.angle(t)) * self.r
    }
    pub fn derivative(&self, t: f64) -> Vec2 {
        dir(self.angle(t)).perp() * (self.r * self.sign())
    }
    /// Parameter of polar angle `theta`, accepting angles up to `eps` rad outside the span
    /// (clamped onto it).
    pub fn param_of_angle(&self, theta: f64, eps: f64) -> Option<f64> {
        let span = self.span();
        let t = (self.sign() * (theta - self.start)).rem_euclid(TAU);
        if t <= span + eps {
            Some(t.min(span))
        } else if t >= TAU - eps {
            Some(0.0)
        } else {
            None
        }
    }
    pub fn closest(&self, p: Point2) -> (f64, Point2) {
        let d = p - self.c;
        if d.length_squared() == 0.0 {
            return (0.0, self.eval(0.0));
        }
        match self.param_of_angle(d.y.atan2(d.x), 0.0) {
            Some(t) => (t, self.c + d * (self.r / d.length())),
            None => {
                let (p0, p1) = (self.eval(0.0), self.eval(self.span()));
                if p0.distance_squared(p) <= p1.distance_squared(p) {
                    (0.0, p0)
                } else {
                    (self.span(), p1)
                }
            }
        }
    }
    fn bbox(&self) -> Aabb2 {
        let mut b = Aabb2::from_points([self.eval(0.0), self.eval(self.span())]);
        for k in 0..4 {
            let phi = f64::from(k) * FRAC_PI_2;
            if self.circle || self.param_of_angle(phi, 0.0).is_some() {
                b.include_point(self.c + dir(phi) * self.r);
            }
        }
        b
    }
    /// CCW interval `(start ∈ [0, 2π), length)`.
    fn ccw_interval(&self) -> (f64, f64) {
        let s = if self.sweep >= 0.0 {
            self.start
        } else {
            self.start + self.sweep
        };
        (s.rem_euclid(TAU), self.span())
    }
    /// End points (none for a full circle).
    fn ends(&self) -> SmallVec<[(f64, Point2); 2]> {
        if self.circle {
            SmallVec::new()
        } else {
            [(0.0, self.eval(0.0)), (self.span(), self.eval(self.span()))]
                .into_iter()
                .collect()
        }
    }
}

/// Angular overlap length of two co-circular spans, rad.
fn overlap_angle(a: &Round, b: &Round) -> f64 {
    let (s1, l1) = a.ccw_interval();
    let (s2, l2) = b.ccw_interval();
    let mut total = 0.0;
    for k in [-1.0, 0.0, 1.0] {
        let s2k = s2 + k * TAU;
        total += ((s1 + l1).min(s2k + l2) - s1.max(s2k)).max(0.0);
    }
    total.min(l1.min(l2))
}

impl Curve2 {
    pub(crate) fn round(&self) -> Option<Round> {
        match *self {
            Curve2::Line(_) => None,
            Curve2::Circle(Circle2 { center, radius }) => Some(Round {
                c: center,
                r: radius,
                start: 0.0,
                sweep: TAU,
                circle: true,
            }),
            Curve2::Arc(Arc2 {
                center,
                radius,
                start_angle,
                sweep,
            }) => Some(Round {
                c: center,
                r: radius,
                start: start_angle,
                sweep,
                circle: false,
            }),
        }
    }

    /// Parameter interval (see the module docs).
    pub fn param_range(&self) -> (f64, f64) {
        match self {
            Curve2::Line(_) => (0.0, 1.0),
            Curve2::Circle(_) => (0.0, TAU),
            Curve2::Arc(a) => (0.0, a.sweep.abs()),
        }
    }

    /// Point at parameter `t`.
    pub fn eval(&self, t: f64) -> Point2 {
        match (self, self.round()) {
            (Curve2::Line(s), _) => s.a + (s.b - s.a) * t,
            (_, Some(r)) => r.eval(t),
            _ => Point2::NAN,
        }
    }

    /// First derivative `d eval / dt` (direction of travel).
    pub fn derivative(&self, t: f64) -> Vec2 {
        match (self, self.round()) {
            (Curve2::Line(s), _) => s.b - s.a,
            (_, Some(r)) => r.derivative(t),
            _ => Vec2::NAN,
        }
    }

    /// Signed curvature (positive = turning left / CCW), 1/mm.
    pub fn curvature(&self) -> f64 {
        match self.round() {
            None => 0.0,
            Some(r) => r.sign() / r.r,
        }
    }

    /// Closest point to `p`: `(t, point)`.
    pub fn closest(&self, p: Point2) -> (f64, Point2) {
        match (self, self.round()) {
            (Curve2::Line(s), _) => {
                let d = s.b - s.a;
                let ll = d.length_squared();
                let t = if ll > 0.0 {
                    ((p - s.a).dot(d) / ll).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                (t, s.a + d * t)
            }
            (_, Some(r)) => r.closest(p),
            _ => (0.0, Point2::NAN),
        }
    }

    /// Tight axis-aligned bounding box.
    pub fn bbox(&self) -> Aabb2 {
        match (self, self.round()) {
            (Curve2::Line(s), _) => Aabb2::new(s.a, s.b),
            (_, Some(r)) => r.bbox(),
            _ => Aabb2::EMPTY,
        }
    }

    /// The same point set traversed the other way. A reversed circle is an arc with
    /// `sweep = −2π` starting at angle 0.
    pub fn reversed(&self) -> Curve2 {
        match *self {
            Curve2::Line(s) => Curve2::Line(Segment2 { a: s.b, b: s.a }),
            Curve2::Circle(c) => Curve2::Arc(Arc2 {
                center: c.center,
                radius: c.radius,
                start_angle: 0.0,
                sweep: -TAU,
            }),
            Curve2::Arc(a) => Curve2::Arc(Arc2 {
                start_angle: a.start_angle + a.sweep,
                sweep: -a.sweep,
                ..a
            }),
        }
    }

    /// The piece from `t0` to `t1` (reversed if `t1 < t0`). Circles and arcs give an `Arc`.
    pub fn trimmed(&self, t0: f64, t1: f64) -> Curve2 {
        match (self, self.round()) {
            (Curve2::Line(_), _) => Curve2::Line(Segment2 {
                a: self.eval(t0),
                b: self.eval(t1),
            }),
            (_, Some(r)) => Curve2::Arc(Arc2 {
                center: r.c,
                radius: r.r,
                start_angle: r.angle(t0),
                sweep: r.sign() * (t1 - t0),
            }),
            _ => self.clone(),
        }
    }

    /// Circle, or arc with `|sweep| = 2π`.
    pub fn is_closed(&self) -> bool {
        match self {
            Curve2::Line(_) => false,
            Curve2::Circle(_) => true,
            Curve2::Arc(a) => a.sweep.abs() >= TAU - 1e-12,
        }
    }

    /// Arc length, mm.
    pub fn length(&self) -> f64 {
        match (self, self.round()) {
            (Curve2::Line(s), _) => s.a.distance(s.b),
            (_, Some(r)) => r.r * r.span(),
            _ => f64::NAN,
        }
    }
}

/// Intersections of `a` and `b` with tolerance `tol.linear`, sorted by `ta`.
///
/// End points within `tol.linear` of the other curve count as touching (T-junctions, tiny
/// gaps); near-tangent contacts within `tol.linear` give one `tangential` hit. Overlapping
/// curves (collinear lines, co-circular arcs/circles) report the ends of each overlapping
/// stretch as `tangential` hits; two identical full circles give no hits.
pub fn intersect(a: &Curve2, b: &Curve2, tol: &Tolerance) -> SmallVec<[CurveHit; 4]> {
    intersect_detailed(a, b, tol).hits
}

/// [`intersect`] plus the overlap flag.
pub(crate) fn intersect_detailed(a: &Curve2, b: &Curve2, tol: &Tolerance) -> Intersection {
    let mut out = match (a, b, a.round(), b.round()) {
        (Curve2::Line(sa), Curve2::Line(sb), _, _) => line_line(sa, sb, tol),
        (Curve2::Line(s), _, _, Some(r)) => line_round(s, &r, tol, false),
        (_, Curve2::Line(s), Some(r), _) => line_round(s, &r, tol, true),
        (_, _, Some(ra), Some(rb)) => round_round(&ra, &rb, tol),
        _ => Intersection::default(),
    };
    for h in &mut out.hits {
        if !h.tangential {
            let (da, db) = (a.derivative(h.ta), b.derivative(h.tb));
            let s = da.perp_dot(db).abs() / (da.length() * db.length());
            h.tangential = s <= tol.angular.max(1e-12);
        }
    }
    out.hits.sort_by(|x, y| x.ta.total_cmp(&y.ta));
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

fn line_line(a: &Segment2, b: &Segment2, tol: &Tolerance) -> Intersection {
    let mut out = Intersection::default();
    match segment2_segment2(a, b, tol) {
        Seg2Intersection::None => {}
        Seg2Intersection::Point { t, u, p } => {
            // `p` lies on `a`; when an end point of one segment only touches the other within
            // tolerance, the crossing of the supporting lines lies on both and is the better
            // vertex position.
            let (r, sv) = (a.b - a.a, b.b - b.a);
            let denom = r.perp_dot(sv);
            let mut point = p;
            if denom.abs() > tol.angular * r.length() * sv.length() {
                let x = a.a + r * ((b.a - a.a).perp_dot(sv) / denom);
                if x.distance(p) <= 2.0 * tol.linear {
                    point = x;
                }
            }
            out.hits.push(CurveHit {
                ta: t,
                tb: u,
                point,
                tangential: false,
            });
        }
        Seg2Intersection::Overlap { t0, t1 } => {
            out.overlap = true;
            let lb = Curve2::Line(*b);
            for t in [t0, t1] {
                let p = a.a + (a.b - a.a) * t;
                let (u, _) = lb.closest(p);
                // Prefer an exact end point of either segment as the hit point.
                #[allow(clippy::float_cmp, reason = "exact end-point parameters")]
                let p = if t == 0.0 || t == 1.0 {
                    p
                } else if u == 0.0 {
                    b.a
                } else if u == 1.0 {
                    b.b
                } else {
                    p
                };
                push_unique(
                    &mut out.hits,
                    CurveHit {
                        ta: t,
                        tb: u,
                        point: p,
                        tangential: true,
                    },
                    tol.linear,
                );
            }
        }
    }
    out
}

/// Line `s` against circle/arc `r`. `swap` returns hits with `ta` on the round.
fn line_round(s: &Segment2, r: &Round, tol: &Tolerance, swap: bool) -> Intersection {
    let lin = tol.linear;
    let mut hits: SmallVec<[CurveHit; 4]> = SmallVec::new();
    let line = Curve2::Line(*s);
    let d = s.b - s.a;
    let len = d.length();
    let mk = |tl: f64, tr: f64, p: Point2, tangential: bool| {
        if swap {
            CurveHit {
                ta: tr,
                tb: tl,
                point: p,
                tangential,
            }
        } else {
            CurveHit {
                ta: tl,
                tb: tr,
                point: p,
                tangential,
            }
        }
    };
    if len > 0.0 {
        let u = d / len;
        let h = u.perp_dot(r.c - s.a);
        let foot = s.a + u * u.dot(r.c - s.a);
        let et = lin / len;
        let ea = lin / r.r;
        let mut accept = |p: Point2, tangential: bool| {
            let tl = (p - s.a).dot(u) / len;
            if tl < -et || tl > 1.0 + et {
                return;
            }
            let v = p - r.c;
            if let Some(tr) = r.param_of_angle(v.y.atan2(v.x), ea) {
                push_unique(&mut hits, mk(tl.clamp(0.0, 1.0), tr, p, tangential), lin);
            }
        };
        if (h.abs() - r.r).abs() <= lin {
            // Tangent within tolerance: one contact at the foot of the perpendicular.
            accept(foot, true);
        } else if h.abs() < r.r {
            let w = (r.r * r.r - h * h).max(0.0).sqrt();
            accept(foot - u * w, false);
            accept(foot + u * w, false);
        }
    }
    // End points touching the other curve (tangential approaches the analytic part misses).
    for (tl, p) in [(0.0, s.a), (1.0, s.b)] {
        let (tr, q) = r.closest(p);
        if q.distance(p) <= lin {
            push_unique(&mut hits, mk(tl, tr, p, false), lin);
        }
    }
    for (tr, p) in r.ends() {
        let (tl, q) = line.closest(p);
        if q.distance(p) <= lin {
            push_unique(&mut hits, mk(tl, tr, p, false), lin);
        }
    }
    Intersection {
        hits,
        overlap: false,
    }
}

fn round_round(a: &Round, b: &Round, tol: &Tolerance) -> Intersection {
    let lin = tol.linear;
    let mut out = Intersection::default();
    // End points touching the other curve.
    let mut ends: SmallVec<[CurveHit; 4]> = SmallVec::new();
    for (ta, p) in a.ends() {
        let (tb, q) = b.closest(p);
        if q.distance(p) <= lin {
            push_unique(
                &mut ends,
                CurveHit {
                    ta,
                    tb,
                    point: p,
                    tangential: false,
                },
                lin,
            );
        }
    }
    for (tb, p) in b.ends() {
        let (ta, q) = a.closest(p);
        if q.distance(p) <= lin {
            push_unique(
                &mut ends,
                CurveHit {
                    ta,
                    tb,
                    point: p,
                    tangential: false,
                },
                lin,
            );
        }
    }
    let dv = b.c - a.c;
    let d = dv.length();
    if d <= lin && (a.r - b.r).abs() <= lin {
        // Co-circular: only the end points can be hits.
        out.overlap = overlap_angle(a, b) * a.r.max(b.r) > lin;
        for mut h in ends {
            h.tangential = true;
            out.hits.push(h);
        }
        return out;
    }
    if !(d <= lin || d > a.r + b.r + lin || d < (a.r - b.r).abs() - lin) {
        let u = dv / d;
        let (ea, eb) = (lin / a.r, lin / b.r);
        let mut accept = |p: Point2, tangential: bool| {
            let (va, vb) = (p - a.c, p - b.c);
            if let (Some(ta), Some(tb)) = (
                a.param_of_angle(va.y.atan2(va.x), ea),
                b.param_of_angle(vb.y.atan2(vb.x), eb),
            ) {
                push_unique(
                    &mut out.hits,
                    CurveHit {
                        ta,
                        tb,
                        point: p,
                        tangential,
                    },
                    lin,
                );
            }
        };
        if (d - (a.r + b.r)).abs() <= lin {
            accept(((a.c + u * a.r) + (b.c - u * b.r)) * 0.5, true);
        } else if (d - (a.r - b.r).abs()).abs() <= lin {
            let p = if a.r >= b.r {
                ((a.c + u * a.r) + (b.c + u * b.r)) * 0.5
            } else {
                ((a.c - u * a.r) + (b.c - u * b.r)) * 0.5
            };
            accept(p, true);
        } else {
            let x = (d * d + a.r * a.r - b.r * b.r) / (2.0 * d);
            let h = (a.r * a.r - x * x).max(0.0).sqrt();
            let m = a.c + u * x;
            accept(m - u.perp() * h, false);
            accept(m + u.perp() * h, false);
        }
    }
    for h in ends {
        push_unique(&mut out.hits, h, lin);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: Tolerance = Tolerance::DEFAULT;

    fn line(ax: f64, ay: f64, bx: f64, by: f64) -> Curve2 {
        Curve2::Line(Segment2 {
            a: Point2::new(ax, ay),
            b: Point2::new(bx, by),
        })
    }
    fn circle(x: f64, y: f64, r: f64) -> Curve2 {
        Curve2::Circle(Circle2 {
            center: Point2::new(x, y),
            radius: r,
        })
    }
    fn arc(x: f64, y: f64, r: f64, a0: f64, sweep: f64) -> Curve2 {
        Curve2::Arc(Arc2 {
            center: Point2::new(x, y),
            radius: r,
            start_angle: a0,
            sweep,
        })
    }
    fn near(a: Point2, b: Point2) -> bool {
        a.distance(b) < 1e-9
    }

    #[test]
    fn eval_derivative_param_conventions() {
        let a = arc(0.0, 0.0, 2.0, 0.0, -PI);
        assert_eq!(a.param_range(), (0.0, PI));
        assert!(near(a.eval(FRAC_PI_2), Point2::new(0.0, -2.0)));
        assert!(near(a.derivative(0.0), Point2::new(0.0, -2.0)));
        assert!(a.curvature() < 0.0);
        let c = circle(1.0, 1.0, 1.0);
        assert!(near(c.eval(PI), Point2::new(0.0, 1.0)));
        assert!(c.is_closed() && !a.is_closed());
        assert!((c.length() - TAU).abs() < 1e-12);
    }

    #[test]
    fn reversed_and_trimmed() {
        let a = arc(0.0, 0.0, 1.0, 0.0, FRAC_PI_2);
        let r = a.reversed();
        assert!(near(r.eval(0.0), a.eval(FRAC_PI_2)));
        assert!(near(r.eval(FRAC_PI_2), a.eval(0.0)));
        let t = a.trimmed(1.0, 0.25);
        assert!(near(t.eval(0.0), a.eval(1.0)));
        assert!(near(t.eval(0.75), a.eval(0.25)));
        let c = circle(0.0, 0.0, 1.0).reversed();
        assert!(near(c.eval(FRAC_PI_2), Point2::new(0.0, -1.0)));
    }

    #[test]
    fn bbox_and_closest() {
        let a = arc(0.0, 0.0, 1.0, -FRAC_PI_2 / 2.0, FRAC_PI_2);
        let b = a.bbox();
        assert!((b.max.x - 1.0).abs() < 1e-12);
        assert!((b.max.y - (0.5f64).sqrt()).abs() < 1e-12);
        let (t, p) = a.closest(Point2::new(-5.0, 0.1));
        assert!(t == 0.0 || (t - FRAC_PI_2).abs() < 1e-12);
        assert!((p.length() - 1.0).abs() < 1e-12);
        let (t, p) = line(0.0, 0.0, 2.0, 0.0).closest(Point2::new(1.5, 3.0));
        assert!((t - 0.75).abs() < 1e-12 && near(p, Point2::new(1.5, 0.0)));
    }

    #[test]
    fn line_circle_secant_tangent_miss() {
        let c = circle(0.0, 0.0, 1.0);
        let h = intersect(&line(-2.0, 0.0, 2.0, 0.0), &c, &TOL);
        assert_eq!(h.len(), 2);
        assert!(near(h[0].point, Point2::new(-1.0, 0.0)));
        assert!((h[1].tb - 0.0).abs() < 1e-12 || (h[1].tb - TAU).abs() < 1e-12);
        let h = intersect(&line(-2.0, 1.0, 2.0, 1.0), &c, &TOL);
        assert_eq!(h.len(), 1);
        assert!(h[0].tangential && near(h[0].point, Point2::new(0.0, 1.0)));
        let h = intersect(&line(-2.0, 1.0 + 5e-7, 2.0, 1.0 + 5e-7), &c, &TOL);
        assert_eq!(h.len(), 1, "tangent within tolerance");
        assert!(intersect(&line(-2.0, 1.1, 2.0, 1.1), &c, &TOL).is_empty());
    }

    #[test]
    fn line_arc_range_and_t_junction() {
        let a = arc(0.0, 0.0, 1.0, 0.0, PI);
        // Only the upper half: a horizontal line through the centre hits both end points.
        let h = intersect(&line(-2.0, 0.0, 2.0, 0.0), &a, &TOL);
        assert_eq!(h.len(), 2);
        let h = intersect(&line(-2.0, -0.5, 2.0, -0.5), &a, &TOL);
        assert!(h.is_empty());
        // Line ending 5e-7 short of the arc counts as touching.
        let h = intersect(&line(0.0, 0.0, 0.0, 1.0 - 5e-7), &a, &TOL);
        assert_eq!(h.len(), 1);
        assert!((h[0].ta - 1.0).abs() < 1e-12);
        // ... with swapped arguments too.
        let h = intersect(&a, &line(0.0, 0.0, 0.0, 1.0 - 5e-7), &TOL);
        assert_eq!(h.len(), 1);
        assert!((h[0].tb - 1.0).abs() < 1e-12 && (h[0].ta - FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn circle_circle_cases() {
        let a = circle(0.0, 0.0, 1.0);
        assert_eq!(intersect(&a, &circle(1.0, 0.0, 1.0), &TOL).len(), 2);
        let h = intersect(&a, &circle(3.0, 0.0, 2.0), &TOL);
        assert_eq!(h.len(), 1);
        assert!(h[0].tangential && near(h[0].point, Point2::new(1.0, 0.0)));
        let h = intersect(&a, &circle(0.5, 0.0, 0.5), &TOL);
        assert_eq!(h.len(), 1);
        assert!(near(h[0].point, Point2::new(1.0, 0.0)));
        let h = intersect(&circle(0.5, 0.0, 0.5), &a, &TOL);
        assert!(near(h[0].point, Point2::new(1.0, 0.0)));
        assert!(intersect(&a, &circle(0.1, 0.0, 0.5), &TOL).is_empty());
        assert!(intersect(&a, &circle(5.0, 0.0, 1.0), &TOL).is_empty());
        let d = intersect_detailed(&a, &circle(0.0, 0.0, 1.0), &TOL);
        assert!(d.overlap && d.hits.is_empty());
    }

    #[test]
    fn cocircular_arcs_overlap() {
        let a = arc(0.0, 0.0, 1.0, 0.0, PI);
        let b = arc(0.0, 0.0, 1.0, FRAC_PI_2, PI);
        let d = intersect_detailed(&a, &b, &TOL);
        assert!(d.overlap);
        assert_eq!(d.hits.len(), 2);
        // Touching end to end only: no overlap.
        let c = arc(0.0, 0.0, 1.0, PI, FRAC_PI_2);
        let d = intersect_detailed(&a, &c, &TOL);
        assert!(!d.overlap);
        assert_eq!(d.hits.len(), 1);
        // Opposite orientation still overlaps.
        let e = arc(0.0, 0.0, 1.0, PI, -FRAC_PI_2);
        assert!(intersect_detailed(&a, &e, &TOL).overlap);
    }

    #[test]
    fn collinear_lines_overlap() {
        let d = intersect_detailed(&line(0.0, 0.0, 7.0, 0.0), &line(3.0, 0.0, 10.0, 0.0), &TOL);
        assert!(d.overlap);
        assert_eq!(d.hits.len(), 2);
        assert!(near(d.hits[0].point, Point2::new(3.0, 0.0)));
        assert!(near(d.hits[1].point, Point2::new(7.0, 0.0)));
    }
}
