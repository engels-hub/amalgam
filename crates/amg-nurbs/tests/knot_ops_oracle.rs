//! KH-12 Done-when 4: knot refinement and degree elevation agree with curvo 0.3.3
//! (`try_refine_knot` / `try_elevate_degree`, dev-only oracle) within 1e-12·max(1, D) on
//! 1000 random curves each (degree 1–7, rational and non-rational): same knots, poles within
//! 1e-12·max(1, D), weights within 1e-12 relative.
//!
//! curvo 0.3.3's `try_elevate_degree` (P&T A5.9) allocates `n + t + 3` output poles and never
//! truncates: the result is padded with zero-weight points, and inputs that need more poles
//! panic (index out of bounds); for `t >= 2` it also underflows `i - degree_inc` (a panic with
//! overflow checks). The oracle therefore elevates with curvo one degree at a time (the
//! elevated curve on a given knot vector is unique, so `t` steps of 1 equal one step of `t`),
//! keeps only the valid prefix (`n + 1 + spans` poles) of each step, and uses curves within
//! curvo's capacity (≤ 4 spans). With interior multiplicities `<= p − 2` curvo additionally
//! returns non-positive weights or leaves the input shape in part of the cases (cause not
//! traced; `elevate_vs_curvo_general_knots` prints the counts and checks that every valid
//! curvo result matches ours). The strict oracle set therefore uses interior multiplicities
//! `p − 1` or `p`; general knot vectors are covered by the shape proptests in `knot_props`.
//! curvo has no blossom fallback for uneven knots, so cases where curvo's own result drifts
//! from the input by more than ε are excluded (counted and bounded).
#![allow(clippy::unwrap_used, clippy::float_cmp, reason = "test code")]

mod common;

use amg_nurbs::{NurbsCurve3, Point3};
use common::{Rng, curve, diag, samples};
use curvo::prelude::NurbsCurve3D;

fn to_curvo(c: &NurbsCurve3) -> NurbsCurve3D<f64> {
    let w = c.weights();
    let pts = c
        .poles()
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let wi = w.map_or(1.0, |w| w[i]);
            [p.x * wi, p.y * wi, p.z * wi, wi].into()
        })
        .collect();
    NurbsCurve3D::try_new(c.degree(), pts, c.knots().as_slice().to_vec()).unwrap()
}

/// The first `n` curvo control points dehomogenized; `None` if a weight is not positive.
fn curvo_poles(o: &NurbsCurve3D<f64>, n: usize) -> Option<(Vec<Point3>, Vec<f64>)> {
    let mut ps = Vec::new();
    let mut ws = Vec::new();
    for h in o.control_points().iter().take(n) {
        if h.w.is_nan() || h.w <= 0.0 {
            return None;
        }
        ps.push(Point3::new(h.x / h.w, h.y / h.w, h.z / h.w));
        ws.push(h.w);
    }
    Some((ps, ws))
}

/// Max deviation from `c` of curvo's curve truncated to `n` poles (None if degenerate).
fn curvo_shape_error(c: &NurbsCurve3, o: &NurbsCurve3D<f64>, n: usize) -> Option<f64> {
    let (ps, ws) = curvo_poles(o, n)?;
    let oc = NurbsCurve3::new(
        o.degree(),
        amg_nurbs::KnotVector::new(o.knots().as_slice()[..n + o.degree() + 1].to_vec()).ok()?,
        ps,
        Some(ws),
    )
    .ok()?;
    let (a, b) = c.domain();
    Some(
        samples(a, b, 200, 5)
            .into_iter()
            .map(|t| (oc.eval(t) - c.eval(t)).length())
            .fold(0.0, f64::max),
    )
}

/// Pole / weight / knot differences between ours and curvo's, relative to `scale`.
fn compare(ours: &NurbsCurve3, o: &NurbsCurve3D<f64>, eps: f64, what: &str, i: usize) -> f64 {
    let n = ours.poles().len();
    let (ps, ws) = curvo_poles(o, n).unwrap();
    assert_eq!(ours.degree(), o.degree(), "{what} {i}: degree");
    let ok = &o.knots().as_slice()[..n + o.degree() + 1];
    let uk = ours.knots().as_slice();
    assert_eq!(uk.len(), ok.len(), "{what} {i}: knot count");
    let span = uk[uk.len() - 1] - uk[0];
    for (x, y) in uk.iter().zip(ok) {
        assert!(
            (x - y).abs() <= 1e-14 * span.max(1.0),
            "{what} {i}: knot {x} vs {y}"
        );
    }
    assert_eq!(ours.poles().len(), ps.len(), "{what} {i}: pole count");
    let mut worst = 0.0f64;
    for (k, (p, q)) in ours.poles().iter().zip(&ps).enumerate() {
        let e = (*p - *q).length();
        assert!(e <= eps, "{what} {i}: pole {k} differs by {e:e} > {eps:e}");
        worst = worst.max(e / eps * 1e-12);
    }
    let w = ours
        .weights()
        .map_or_else(|| vec![1.0; ps.len()], <[f64]>::to_vec);
    for (x, y) in w.iter().zip(&ws) {
        assert!(
            (x - y).abs() <= 1e-12 * x.abs(),
            "{what} {i}: weight {x} vs {y}"
        );
    }
    worst
}

/// Random curve with at most `max_spans` non-empty spans and interior multiplicities in
/// `min_mult..=p`.
fn small_curve(
    r: &mut Rng,
    p: usize,
    max_spans: usize,
    min_mult: usize,
    rational: bool,
) -> NurbsCurve3 {
    let spans = r.usize(1, max_spans);
    let a = r.range(-5.0, 5.0);
    let len = r.range(0.5, 20.0);
    let mut inner: Vec<f64> = (1..spans).map(|_| a + len * r.range(0.05, 0.95)).collect();
    inner.sort_by(f64::total_cmp);
    let mut v = vec![a; p + 1];
    for k in inner {
        v.extend(std::iter::repeat_n(k, r.usize(min_mult.clamp(1, p), p)));
    }
    v.extend(std::iter::repeat_n(a + len, p + 1));
    let n = v.len() - p - 1;
    let poles = (0..n).map(|_| r.point(100.0)).collect();
    let w = rational.then(|| (0..n).map(|_| r.range(0.2, 5.0)).collect());
    NurbsCurve3::new(p, amg_nurbs::KnotVector::new(v).unwrap(), poles, w).unwrap()
}

/// curvo elevation by `by` as `by` single steps, each truncated to its valid prefix.
fn curvo_elevate(o: &NurbsCurve3D<f64>, by: usize) -> Option<NurbsCurve3D<f64>> {
    let mut cur = o.clone();
    for _ in 0..by {
        let spans = {
            let k = cur.knots().as_slice();
            k.windows(2).filter(|w| w[1] > w[0]).count()
        };
        let n = cur.control_points().len() + spans;
        let e = cur.try_elevate_degree(cur.degree() + 1).ok()?;
        let pts = e.control_points()[..n].to_vec();
        let knots = e.knots().as_slice()[..n + e.degree() + 1].to_vec();
        cur = NurbsCurve3D::try_new(e.degree(), pts, knots).ok()?;
    }
    Some(cur)
}

#[test]
fn refine_matches_curvo() {
    let mut r = Rng(0x0c12_cafe);
    let mut worst = 0.0f64;
    for i in 0..1000 {
        let p = 1 + i % 7;
        let c = curve(&mut r, p, (i / 7) % 2 == 1);
        let eps = 1e-12 * diag(c.poles()).max(1.0);
        let (a, b) = c.domain();
        let nk = r.usize(1, 20);
        let mut xs: Vec<f64> = (0..nk)
            .map(|_| a + (b - a) * r.range(0.001, 0.999))
            .collect();
        xs.sort_by(f64::total_cmp);
        let ours = c.refine_knots(&xs).unwrap();
        let mut o = to_curvo(&c);
        o.try_refine_knot(xs).unwrap();
        worst = worst.max(compare(&ours, &o, eps, "refine", i));
    }
    eprintln!("curvo refine: worst pole difference {worst:e}·max(1,D) over 1000 curves");
}

#[test]
fn elevate_matches_curvo() {
    let mut r = Rng(0x0c12_beef);
    let mut worst = 0.0f64;
    // Silence the oracle's panic messages for the duration of this test.
    let hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let (mut skipped_degenerate, mut skipped_inaccurate) = (0usize, 0usize);
    for i in 0..1000 {
        let p = 1 + i % 7;
        let by = 1 + (i / 7) % 3;
        let c = small_curve(&mut r, p, 4, p - 1, (i / 21) % 2 == 1);
        let eps = 1e-12 * diag(c.poles()).max(1.0);
        let ours = c.elevate_degree(by).unwrap();
        let hook_free = std::panic::catch_unwind(|| curvo_elevate(&to_curvo(&c), by));
        let Ok(Some(o)) = hook_free else {
            skipped_degenerate += 1;
            continue;
        };
        match curvo_shape_error(&c, &o, ours.poles().len()) {
            None => skipped_degenerate += 1,
            Some(e) if e > eps => skipped_inaccurate += 1,
            Some(_) => worst = worst.max(compare(&ours, &o, eps, "elevate", i)),
        }
    }
    std::panic::set_hook(hook);
    eprintln!(
        "curvo elevate (1000 curves, ≤ 4 spans, t = 1..3 as t curvo steps): worst pole difference {worst:e}·max(1,D); excluded {skipped_degenerate} \
         cases with degenerate curvo output and {skipped_inaccurate} where curvo drifts > ε \
         (ours is checked against the input shape in knot_props)"
    );
    assert!(
        skipped_degenerate + skipped_inaccurate <= 10,
        "too many excluded cases"
    );
}

/// Measures curvo on general knot vectors (interior multiplicities 1..=p, ≤ 4 spans, degree
/// 1–7, rational and not): cases where curvo returns positive weights and keeps the input
/// shape must match ours within ε; the rest are counted. Ours always keeps the shape.
#[test]
fn elevate_vs_curvo_general_knots() {
    let mut r = Rng(0x0c12_5151);
    let (mut matched, mut degenerate, mut drift) = (0usize, 0usize, 0usize);
    let mut worst = 0.0f64;
    let hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    for i in 0..1000 {
        let p = 1 + i % 7;
        let c = small_curve(&mut r, p, 4, 1, (i / 7) % 2 == 1);
        let eps = 1e-12 * diag(c.poles()).max(1.0);
        let ours = c.elevate_degree(1).unwrap();
        let (a, b) = c.domain();
        let e = samples(a, b, 200, 3)
            .into_iter()
            .map(|t| (ours.eval(t) - c.eval(t)).length())
            .fold(0.0, f64::max);
        assert!(e <= eps, "ours {e:e} > {eps:e}");
        let n = ours.poles().len();
        let res = std::panic::catch_unwind(|| curvo_elevate(&to_curvo(&c), 1))
            .ok()
            .flatten();
        match res.as_ref().and_then(|o| curvo_shape_error(&c, o, n)) {
            None => degenerate += 1,
            Some(ce) if ce > eps => drift += 1,
            Some(_) => {
                matched += 1;
                worst = worst.max(compare(&ours, res.as_ref().unwrap(), eps, "general", i));
            }
        }
    }
    std::panic::set_hook(hook);
    eprintln!(
        "curvo elevate on general knots (1000 curves): {matched} match ours (worst pole difference {worst:e}·max(1,D)), {degenerate} panic or have non-positive weights, {drift} leave the input shape by > ε"
    );
}
