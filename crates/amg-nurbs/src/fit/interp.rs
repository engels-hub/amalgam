//! Global interpolation (P&T A9.1, §9.2.2 end derivatives, periodic / closed).

#![allow(clippy::float_cmp, reason = "knot runs are compared exactly")]

use crate::basis::{basis_funs, ders_basis_funs};
use crate::knot_ops::{self, H};
use crate::knots::find_span;
use crate::linalg::Band;
use crate::{CtrlPt, KnotVector, MAX_DEGREE, NurbsCurve, NurbsError};

use super::{InterpOpts, averaging_knots, parametrize, strictly_increasing};

/// A9.1 GlobalCurveInterp with optional first / second end-derivative rows (§9.2.2) or
/// periodic closure.
///
/// - **Open:** `m + 1` points and `s` / `e` derivative constraints at the start / end give
///   `m + 1 + s + e` poles. The knots average the parameter list with `ū_0` repeated `s` and
///   `ū_m` repeated `e` extra times (eq. 9.8; for one first derivative per end this is
///   eq. 9.22). The rows (point, `C'`, `C''` at the start, interior points, `C''`, `C'`,
///   point at the end) form a banded system solved by banded LU with partial pivoting.
/// - **Closed:** the points (a repeated first point at the end is dropped) are interpolated by
///   a periodic B-spline whose knots are the periodic averages of the parameters (closing
///   chord included), then knots are inserted at the seam until the curve is clamped there.
///   The result is C^{p−1} at the seam and starts and ends exactly at the first point.
///
/// Data points are reproduced to rounding and end derivatives exactly as given (KH-13 tests:
/// ≤ 1e-12·D and 1e-10 relative).
///
/// Errors: `Degree`; `Degenerate` for too few or non-finite points, coincident consecutive
/// points, explicit parameters that are not strictly increasing or of the wrong length, a
/// degree too low for the derivative constraints (`degree >= constraints at an end + 1`),
/// fewer poles than `degree + 1`, or end derivatives combined with `closed`; `Singular` if
/// the system cannot be solved.
pub fn interpolate<P: CtrlPt>(pts: &[P], o: &InterpOpts<P>) -> Result<NurbsCurve<P>, NurbsError> {
    let p = o.degree;
    if p == 0 || p > MAX_DEGREE {
        return Err(NurbsError::Degree(p));
    }
    if pts.len() < 2 {
        return Err(NurbsError::Degenerate(
            "interpolation needs at least two points",
        ));
    }
    if pts.iter().any(|q| !q.is_finite_pt()) {
        return Err(NurbsError::Degenerate("data point is not finite"));
    }
    let ends = [o.ends.start, o.ends.start2, o.ends.end, o.ends.end2];
    if ends.iter().flatten().any(|d| !d.is_finite_pt()) {
        return Err(NurbsError::Degenerate("end derivative is not finite"));
    }
    if o.closed {
        if !o.ends.is_empty() {
            return Err(NurbsError::Degenerate(
                "end derivatives cannot be combined with closed interpolation",
            ));
        }
        return interpolate_closed(pts, o);
    }
    let m = pts.len() - 1;
    let params = match &o.params {
        Some(u) => {
            if u.len() != pts.len() || !strictly_increasing(u) {
                return Err(NurbsError::Degenerate(
                    "parameters must be strictly increasing, one per point",
                ));
            }
            u.clone()
        }
        None => {
            let u = parametrize(pts, o.parametrization);
            if !strictly_increasing(&u) {
                return Err(NurbsError::Degenerate("consecutive data points coincide"));
            }
            u
        }
    };
    let start = o.ends.start_orders();
    let end = o.ends.end_orders();
    let s = start.iter().flatten().count();
    let e = end.iter().flatten().count();
    if (s > 0 && s + 1 > p) || (e > 0 && e + 1 > p) {
        return Err(NurbsError::Degenerate(
            "degree too low for the end-derivative constraints",
        ));
    }
    let n = m + 1 + s + e;
    if n < p + 1 {
        return Err(NurbsError::Degenerate("too few points for the degree"));
    }
    // Extended parameter list: ū_0 and ū_m repeated once per derivative constraint.
    let mut ext = Vec::with_capacity(n);
    ext.extend(std::iter::repeat_n(params[0], 1 + s));
    ext.extend_from_slice(&params[1..m]);
    ext.extend(std::iter::repeat_n(params[m], 1 + e));
    let knots = averaging_knots(&ext, p);
    let k = knots.as_slice();
    if knots.len() != n + p + 1 {
        return Err(NurbsError::Degenerate("too few points for the degree"));
    }

    // Rows: (parameter, derivative order, column start, values, rhs).
    struct Row<P> {
        col: usize,
        vals: [f64; MAX_DEGREE + 1],
        rhs: P,
    }
    let mut rows: Vec<Row<P>> = Vec::with_capacity(n);
    let mut d = [[0.0; MAX_DEGREE + 1]; 3];
    let mut push_ders = |rows: &mut Vec<Row<P>>, t: f64, order: usize, rhs: P| {
        let span = find_span(k, p, n, t);
        ders_basis_funs(k, span, t, p, order, &mut d);
        let mut vals = [0.0; MAX_DEGREE + 1];
        vals[..=p].copy_from_slice(&d[order][..=p]);
        rows.push(Row {
            col: span - p,
            vals,
            rhs,
        });
    };
    push_ders(&mut rows, params[0], 0, pts[0]);
    for (order, c) in start.iter().enumerate() {
        if let Some(v) = *c {
            push_ders(&mut rows, params[0], order, v);
        }
    }
    for i in 1..m {
        push_ders(&mut rows, params[i], 0, pts[i]);
    }
    for (order, c) in end.iter().enumerate().rev() {
        if let Some(v) = *c {
            push_ders(&mut rows, params[m], order, v);
        }
    }
    push_ders(&mut rows, params[m], 0, pts[m]);

    let (mut kl, mut ku) = (0, 0);
    for (i, r) in rows.iter().enumerate() {
        let lo = r.col;
        let hi = r.col + p;
        if i > lo {
            kl = kl.max(i - lo);
        }
        if hi > i {
            ku = ku.max(hi - i);
        }
    }
    let mut a = Band::new(n, kl, ku);
    let mut rhs = Vec::with_capacity(n);
    for (i, r) in rows.iter().enumerate() {
        for (j, &v) in r.vals[..=p].iter().enumerate() {
            if v != 0.0 {
                a.add(i, r.col + j, v);
            }
        }
        rhs.push(r.rhs);
    }
    a.solve(&mut rhs, "interpolation")?;
    // Clamped ends interpolate exactly: store the data points bit-identically, then resolve
    // the end poles touched by derivative rows from their constraint
    // (`C^(k)(a) = Σ_{i<=k} N_i^(k)(a) P_i`), so the end derivatives hold to rounding of
    // that one formula instead of the whole solve.
    rhs[0] = pts[0];
    rhs[n - 1] = pts[m];
    let mut d = [[0.0; MAX_DEGREE + 1]; 3];
    ders_basis_funs(k, p, params[0], p, 2.min(p), &mut d);
    for (order, c) in start.iter().enumerate() {
        if let Some(v) = *c {
            let acc = (0..order).fold(v, |acc, i| acc - rhs[i] * d[order][i]);
            rhs[order] = acc * (1.0 / d[order][order]);
        }
    }
    let span = find_span(k, p, n, params[m]);
    ders_basis_funs(k, span, params[m], p, 2.min(p), &mut d);
    let base = span - p;
    for (order, c) in end.iter().enumerate() {
        if let Some(v) = *c {
            let me = n - 1 - order;
            let acc = (me + 1..n).fold(v, |acc, i| acc - rhs[i] * d[order][i - base]);
            rhs[me] = acc * (1.0 / d[order][me - base]);
        }
    }
    NurbsCurve::new(p, knots, rhs, None)
}

/// Floor division for the periodic knot index.
fn div_floor(a: i64, b: i64) -> i64 {
    a.div_euclid(b)
}

/// Periodic interpolation, converted to clamped form at the first point.
fn interpolate_closed<P: CtrlPt>(
    pts: &[P],
    o: &InterpOpts<P>,
) -> Result<NurbsCurve<P>, NurbsError> {
    let p = o.degree;
    let scale = pts.iter().fold(0.0f64, |s, q| s.max(q.max_abs())).max(1.0);
    let repeated = (pts[pts.len() - 1] - pts[0]).length() <= 1e-12 * scale;
    let q: &[P] = if repeated { &pts[..pts.len() - 1] } else { pts };
    let nq = q.len();
    if nq < 2 || nq < p + 1 {
        return Err(NurbsError::Degenerate(
            "closed interpolation needs at least degree + 1 distinct points",
        ));
    }
    let ubar: Vec<f64> = match &o.params {
        Some(u) => {
            let want = if repeated { pts.len() } else { pts.len() + 1 };
            if u.len() != want || !strictly_increasing(u) {
                return Err(NurbsError::Degenerate(
                    "closed parameters must be strictly increasing, one per point plus the closing one",
                ));
            }
            u.clone()
        }
        None => {
            let mut loop_pts = q.to_vec();
            loop_pts.push(q[0]);
            let u = parametrize(&loop_pts, o.parametrization);
            if !strictly_increasing(&u) {
                return Err(NurbsError::Degenerate("consecutive data points coincide"));
            }
            u
        }
    };
    let nn = nq as i64;
    let a = ubar[0];
    let period = ubar[nq] - a;
    let ub = |i: i64| -> f64 {
        let r = i.rem_euclid(nn) as usize;
        ubar[r] + div_floor(i, nn) as f64 * period
    };
    let pi = p as i64;
    let inv = 1.0 / p as f64;
    // One period of averaged knots t_j, j = j0 .. j0 + N − 1, extended by ± period.
    let j0 = -pi - 1;
    let base: Vec<f64> = (0..nn)
        .map(|r| (0..pi).map(|i| ub(j0 + r + i)).sum::<f64>() * inv)
        .collect();
    let t = |j: i64| -> f64 {
        let r = (j - j0).rem_euclid(nn) as usize;
        base[r] + div_floor(j - j0, nn) as f64 * period
    };
    // Span index J with t_J <= u < t_{J+1}.
    let span_of = |u: f64, guess: i64| -> i64 {
        let mut j = guess;
        while t(j + 1) <= u {
            j += 1;
        }
        while t(j) > u {
            j -= 1;
        }
        j
    };
    let mut a_mat = Band::new(nq, nq - 1, nq - 1);
    let mut rhs = q.to_vec();
    let mut local = vec![0.0; 2 * p + 2];
    let mut nb = [0.0; MAX_DEGREE + 1];
    let mut guess = j0;
    for (row, &u) in ubar[..nq].iter().enumerate() {
        let jj = span_of(u, guess);
        guess = jj;
        for (i, l) in local.iter_mut().enumerate() {
            *l = t(jj - pi + i as i64);
        }
        basis_funs(&local, p, u, p, &mut nb);
        for (r, &v) in nb[..=p].iter().enumerate() {
            if v != 0.0 {
                let col = (jj - pi + r as i64).rem_euclid(nn) as usize;
                a_mat.add(row, col, v);
            }
        }
    }
    a_mat.solve(&mut rhs, "closed interpolation")?;

    // Unclamped curve over one period: knots t_{J−p} … t_{J+N+p+1}, poles J−p … J+N.
    let ja = span_of(a, j0);
    let mut u: Vec<f64> = (0..(nq + 2 * p + 2) as i64)
        .map(|i| t(ja - pi + i))
        .collect();
    let mut pw: Vec<H<P>> = (0..(nq + p + 1) as i64)
        .map(|i| H {
            p: rhs[(ja - pi + i).rem_euclid(nn) as usize],
            w: 1.0,
        })
        .collect();
    let b = a + period;
    for x in [a, b] {
        let s = u.iter().filter(|&&k| k == x).count();
        if s < p {
            let k = u.partition_point(|&k| k <= x) - 1;
            let (u2, pw2) = knot_ops::insert(p, &u, &pw, x, k, s, p - s);
            u = u2;
            pw = pw2;
        }
    }
    let fa = u.partition_point(|&k| k < a);
    let lb = u.partition_point(|&k| k <= b) - 1;
    if fa == 0 || lb < fa {
        return Err(NurbsError::Singular("closed interpolation"));
    }
    let mut kv = Vec::with_capacity(lb - fa + 3);
    kv.push(a);
    kv.extend_from_slice(&u[fa..=lb]);
    kv.push(b);
    let count = kv.len() - p - 1;
    if fa - 1 + count > pw.len() {
        return Err(NurbsError::Singular("closed interpolation"));
    }
    let mut poles: Vec<P> = pw[fa - 1..fa - 1 + count].iter().map(|h| h.p).collect();
    poles[0] = q[0];
    poles[count - 1] = q[0];
    let knots = KnotVector::new(kv)?;
    NurbsCurve::new(p, knots, poles, None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fit::{EndDerivs, Parametrization};
    use amg_core::{Point2, Point3};

    fn opts(p: usize) -> InterpOpts<Point2> {
        InterpOpts {
            degree: p,
            parametrization: Parametrization::ChordLength,
            ..InterpOpts::default()
        }
    }

    /// P&T Ex. 9.1: 5 points, cubic, chord length; the curve passes through every point at
    /// its parameter.
    #[test]
    fn example_9_1_interpolation() {
        let q = [
            Point2::new(0., 0.),
            Point2::new(3., 4.),
            Point2::new(-1., 4.),
            Point2::new(-4., 0.),
            Point2::new(-4., -3.),
        ];
        let c = interpolate(&q, &opts(3)).unwrap();
        assert_eq!(c.poles().len(), 5);
        let u = [0.0, 5.0 / 17.0, 9.0 / 17.0, 14.0 / 17.0, 1.0];
        for (pt, t) in q.iter().zip(u) {
            assert!((c.eval(t) - *pt).length() < 1e-14, "{t}");
        }
    }

    #[test]
    fn end_derivatives_and_errors() {
        let q: Vec<Point3> = (0..6)
            .map(|i| {
                let t = i as f64;
                Point3::new(t, (t * 0.7).sin(), 0.1 * t * t)
            })
            .collect();
        let d0 = Point3::new(1.0, 2.0, -1.0);
        let d1 = Point3::new(0.5, -1.0, 3.0);
        let e0 = Point3::new(0.0, 4.0, 1.0);
        let e1 = Point3::new(-2.0, 0.0, 1.0);
        let o = InterpOpts {
            degree: 3,
            ends: EndDerivs {
                start: Some(d0),
                end: Some(d1),
                start2: Some(e0),
                end2: Some(e1),
            },
            ..InterpOpts::default()
        };
        let c = interpolate(&q, &o).unwrap();
        assert_eq!(c.poles().len(), 10);
        let (a, b) = c.domain();
        let da = c.derivs(a, 2);
        let db = c.derivs(b, 2);
        assert!((da[1] - d0).length() < 1e-10 * d0.length());
        assert!((db[1] - d1).length() < 1e-10 * d1.length());
        assert!((da[2] - e0).length() < 1e-10 * e0.length());
        assert!((db[2] - e1).length() < 1e-10 * e1.length());
        // Degree 2 cannot take two constraints per end.
        let o2 = InterpOpts {
            degree: 2,
            ..o.clone()
        };
        assert!(matches!(
            interpolate(&q, &o2),
            Err(NurbsError::Degenerate(_))
        ));
        let o3 = InterpOpts { closed: true, ..o };
        assert!(matches!(
            interpolate(&q, &o3),
            Err(NurbsError::Degenerate(_))
        ));
        assert!(matches!(
            interpolate(&q[..1], &InterpOpts::default()),
            Err(NurbsError::Degenerate(_))
        ));
        assert!(matches!(
            interpolate(
                &[Point3::ZERO, Point3::ZERO, Point3::X],
                &InterpOpts::default()
            ),
            Err(NurbsError::Degenerate(_))
        ));
        assert!(matches!(
            interpolate(
                &q,
                &InterpOpts {
                    degree: 0,
                    ..InterpOpts::default()
                }
            ),
            Err(NurbsError::Degree(0))
        ));
        assert!(matches!(
            interpolate(&q[..3], &InterpOpts::default()),
            Err(NurbsError::Degenerate(_))
        ));
    }

    #[test]
    fn closed_interpolation_is_periodic() {
        for p in 1..=5 {
            for repeat in [false, true] {
                let mut q: Vec<Point3> = (0..9)
                    .map(|i| {
                        let a = i as f64 / 9.0 * std::f64::consts::TAU;
                        Point3::new(3.0 * a.cos(), 2.0 * a.sin(), (2.0 * a).sin())
                    })
                    .collect();
                if repeat {
                    q.push(q[0]);
                }
                let o = InterpOpts {
                    degree: p,
                    closed: true,
                    ..InterpOpts::default()
                };
                let c = interpolate(&q, &o).unwrap();
                let mut loop_pts = q[..9].to_vec();
                loop_pts.push(q[0]);
                let u = parametrize(&loop_pts, Parametrization::Centripetal);
                for (pt, &t) in q.iter().zip(&u) {
                    assert!((c.eval(t) - *pt).length() < 1e-12, "p={p} t={t}");
                }
                assert_eq!(c.start(), q[0]);
                assert_eq!(c.end(), q[0]);
                let (a, b) = c.domain();
                let da = c.derivs(a, 4);
                let db = c.derivs(b, 4);
                for k in 1..p.min(5) {
                    let s = da[k].length().max(1.0);
                    assert!((da[k] - db[k]).length() < 1e-9 * s, "p={p} k={k}");
                }
            }
        }
    }
}
