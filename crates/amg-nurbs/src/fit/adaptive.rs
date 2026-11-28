//! Approximation to a tolerance (adaptive knots + Hoschek parameter correction) and
//! re-approximation onto a given knot vector (KH-13, nurbs note §4).

#![allow(clippy::float_cmp, reason = "knot runs are compared exactly")]

use crate::knots::find_span;
use crate::{CtrlPt, KnotVector, MAX_DEGREE, NurbsCurve, NurbsError};

use super::lsq::{BasisCache, EndCons, check_degree_knots, param_errors, pole_map, solve_poles};
use super::{Fit, OntoOpts, Parametrization, parametrize};

/// Hoschek corrections after each solve of [`approximate_to_tolerance`].
const CORRECTIONS_PER_SOLVE: usize = 3;
/// Knot-insertion rounds of [`approximate_to_tolerance`].
const MAX_ROUNDS: usize = 64;
/// Newton iterations of the final projection.
const FINAL_NEWTON: usize = 8;
/// Orthogonality (cosine) criterion of the final projection: the distance is then exact to
/// a relative `1e-18`, far below the 1 % reporting contract.
const FINAL_ANGULAR: f64 = 1e-9;

/// Hoschek (1988) intrinsic parameter correction, one step per interior point:
/// `dt = <Q - C(t), C'(t)> / |C'(t)|^2`, clamped to `domain`. `C` and `C'` come from the
/// basis values cached by the last solve (`cache`, `poles` at `params`). The end parameters
/// stay at the domain ends. Reads `params`, writes the corrected values into `out` (same
/// length) and returns the largest step relative to the smaller neighbouring parameter gap
/// (a coarse-fit signal, see `to_tolerance`).
fn hoschek<P: CtrlPt>(
    cache: &BasisCache,
    p: usize,
    poles: &[P],
    domain: (f64, f64),
    pts: &[P],
    params: &[f64],
    out: &mut [f64],
) -> f64 {
    out.copy_from_slice(params);
    let m = params.len();
    let mut ratio = 0.0f64;
    if m < 3 {
        return ratio;
    }
    for k in 1..m - 1 {
        let t = params[k];
        let (c, d) = cache.point_tangent(k, p, poles);
        let l2 = d.dot(d);
        if l2.is_nan() || l2 <= 0.0 {
            continue;
        }
        let tn = (t + (pts[k] - c).dot(d) / l2).clamp(domain.0, domain.1);
        out[k] = tn;
        let gap = (t - params[k - 1]).min(params[k + 1] - t);
        if gap > 0.0 {
            ratio = ratio.max((tn - t).abs() / gap);
        }
    }
    ratio
}

/// Final projection of every interior data point onto `c` (Newton from its parameter, never
/// increasing the deviation). Returns `(max, rms)` of the point-to-curve distances.
fn project_all<P: CtrlPt>(c: &NurbsCurve<P>, pts: &[P], params: &mut [f64]) -> (f64, f64) {
    let m = params.len();
    let mut max = 0.0f64;
    let mut sq = 0.0;
    for k in 0..m {
        let e = if k == 0 || k == m - 1 {
            (c.eval(params[k]) - pts[k]).length()
        } else {
            let (t, d) = c.newton_foot(pts[k], params[k], FINAL_ANGULAR, FINAL_NEWTON);
            params[k] = t;
            d
        };
        max = max.max(e);
        sq += e * e;
    }
    (max, (sq / m.max(1) as f64).sqrt())
}

fn fit_once<P: CtrlPt>(
    p: usize,
    knots: &KnotVector,
    pts: &[P],
    params: &[f64],
    start: &EndCons<P>,
    end: &EndCons<P>,
    cache: &mut BasisCache,
) -> Result<NurbsCurve<P>, NurbsError> {
    let k = knots.as_slice();
    let n = k.len() - p - 1;
    let map = pole_map(p, k, n, start, end)?;
    let poles = solve_poles(p, k, &map, pts, params, None, cache)?;
    NurbsCurve::new(p, knots.clone(), poles, None)
}

/// Problem data shared by the solves of one `to_tolerance` run.
struct Problem<'a, P> {
    p: usize,
    pts: &'a [P],
    start: EndCons<P>,
    end: EndCons<P>,
}

impl<P: CtrlPt> Problem<'_, P> {
    /// One solve plus its per-point deviations (into `err`); returns the curve and the max.
    fn fit_err(
        &self,
        knots: &KnotVector,
        params: &[f64],
        cache: &mut BasisCache,
        err: &mut Vec<f64>,
    ) -> Result<(NurbsCurve<P>, f64), NurbsError> {
        let c = fit_once(
            self.p,
            knots,
            self.pts,
            params,
            &self.start,
            &self.end,
            cache,
        )?;
        let max = cache.errors(self.p, c.poles(), self.pts, err);
        Ok((c, max))
    }
}

/// Approximates `pts` with a non-rational curve of `degree` whose distance to every data point
/// is at most `tol` (mm), interpolating the first and last point.
///
/// Starts from one Bézier segment (`degree + 1` poles) on parameters from `m`, then repeats:
/// least-squares solve (§9.4.1 with fixed ends), Hoschek (1988) parameter correction and
/// re-solve (up to three times, each kept only if it lowers the largest deviation; the
/// corrected parameters carry over to the next round, and while the corrections still move
/// parameters by more than half a neighbouring gap the initial parametrisation competes),
/// and bisection of every knot span whose largest parametric deviation
/// exceeds `tol` (worst spans first, ties by lowest index, while the pole count stays
/// `<= max_poles`; a span is only split if each half keeps two data parameters). When all
/// deviations are within `tol` the data points are projected onto the curve (Newton) and
/// `max_error` / `rms_error` are the point-to-curve distances, `params` the foot-point
/// parameters.
///
/// Errors: `Degree`; `Degenerate` for fewer than `degree + 1` points, non-finite data or a
/// non-positive `tol`; `TooManyPoles` if `max_poles < degree + 1`; `ToleranceNotMet` if the
/// pole limit is reached (or no span can be split) before `tol`; `Singular` from the solves.
pub fn approximate_to_tolerance<P: CtrlPt>(
    pts: &[P],
    degree: usize,
    tol: f64,
    max_poles: usize,
    m: Parametrization,
) -> Result<Fit<P>, NurbsError> {
    to_tolerance(pts, degree, tol, max_poles, m, true)
}

/// The KH-13 Done-when 2 reference: [`approximate_to_tolerance`] without Hoschek parameter
/// correction and without the final projection (`max_error` is the parametric deviation).
#[doc(hidden)]
pub fn approximate_to_tolerance_uncorrected<P: CtrlPt>(
    pts: &[P],
    degree: usize,
    tol: f64,
    max_poles: usize,
    m: Parametrization,
) -> Result<Fit<P>, NurbsError> {
    to_tolerance(pts, degree, tol, max_poles, m, false)
}

fn to_tolerance<P: CtrlPt>(
    pts: &[P],
    degree: usize,
    tol: f64,
    max_poles: usize,
    m: Parametrization,
    correct: bool,
) -> Result<Fit<P>, NurbsError> {
    let p = degree;
    if p == 0 || p > MAX_DEGREE {
        return Err(NurbsError::Degree(p));
    }
    if !(tol.is_finite() && tol > 0.0) {
        return Err(NurbsError::Degenerate("tolerance must be positive"));
    }
    if pts.len() < p + 1 {
        return Err(NurbsError::Degenerate("too few points for the degree"));
    }
    if pts.iter().any(|q| !q.is_finite_pt()) {
        return Err(NurbsError::Degenerate("data point is not finite"));
    }
    if max_poles < p + 1 {
        return Err(NurbsError::TooManyPoles {
            needed: p + 1,
            limit: max_poles,
        });
    }
    let base = parametrize(pts, m);
    let mut params = base.clone();
    let mut trial = params.clone();
    let pb = Problem {
        p,
        pts,
        start: [Some(pts[0]), None, None],
        end: [Some(pts[pts.len() - 1]), None, None],
    };
    let mut knots = KnotVector::bezier(p);
    let mut span_err: Vec<f64> = Vec::new();
    let mut span_cnt: Vec<usize> = Vec::new();
    let (mut cache, mut cache_t) = (BasisCache::default(), BasisCache::default());
    let (mut err, mut err_t) = (Vec::new(), Vec::new());
    // Compete with the initial parametrisation while the corrections take large steps.
    let mut compete = false;
    for _ in 0..MAX_ROUNDS {
        let (mut c, mut max) = pb.fit_err(&knots, &params, &mut cache, &mut err)?;
        if correct {
            // Corrected parameters carry over between rounds. Foot points of a coarse fit
            // can cluster them and spoil the finer rounds, so while the last correction moved
            // a parameter by more than half a neighbouring gap the initial parametrisation is
            // refitted too and the better of the two (largest deviation) continues.
            if compete {
                let (cb, eb) = pb.fit_err(&knots, &base, &mut cache_t, &mut err_t)?;
                if eb < max {
                    params.copy_from_slice(&base);
                    c = cb;
                    max = eb;
                    std::mem::swap(&mut cache, &mut cache_t);
                    std::mem::swap(&mut err, &mut err_t);
                }
            }
            // A correction is kept only if it lowers the largest deviation.
            compete = false;
            for _ in 0..CORRECTIONS_PER_SOLVE {
                let ratio = hoschek(&cache, p, c.poles(), c.domain(), pts, &params, &mut trial);
                let (ct, mt) = pb.fit_err(&knots, &trial, &mut cache_t, &mut err_t)?;
                if mt < max {
                    std::mem::swap(&mut params, &mut trial);
                    std::mem::swap(&mut cache, &mut cache_t);
                    std::mem::swap(&mut err, &mut err_t);
                    c = ct;
                    max = mt;
                    compete = ratio > 0.5;
                } else {
                    break;
                }
            }
        }
        // Per-span parametric deviation.
        let ks = knots.as_slice();
        let n = ks.len() - p - 1;
        span_err.clear();
        span_err.resize(n, 0.0);
        for (k, &e) in err.iter().enumerate() {
            let s = cache.span(k);
            span_err[s] = span_err[s].max(e);
        }
        if max <= tol {
            let (max_error, rms_error) = if correct {
                project_all(&c, pts, &mut params)
            } else {
                param_errors(&c, pts, &params)
            };
            return Ok(Fit {
                curve: c,
                max_error,
                rms_error,
                params,
            });
        }
        let room = max_poles.saturating_sub(n);
        if room == 0 {
            return Err(NurbsError::ToleranceNotMet { tol, achieved: max });
        }
        // Candidate spans, worst first (ties: lowest index).
        let mut cand: Vec<usize> = (p..n)
            .filter(|&s| span_err[s] > tol && ks[s] < ks[s + 1])
            .collect();
        cand.sort_by(|&x, &y| span_err[y].total_cmp(&span_err[x]).then(x.cmp(&y)));
        // Data counts on each side of every span midpoint.
        span_cnt.clear();
        span_cnt.resize(2 * n, 0);
        for &t in &params {
            let s = find_span(ks, p, n, t);
            let mid = 0.5 * (ks[s] + ks[s + 1]);
            span_cnt[2 * s + usize::from(t >= mid)] += 1;
        }
        let mut new_knots: Vec<f64> = Vec::new();
        for s in cand {
            if new_knots.len() >= room {
                break;
            }
            if span_cnt[2 * s] < 2 || span_cnt[2 * s + 1] < 2 {
                continue;
            }
            let mid = 0.5 * (ks[s] + ks[s + 1]);
            if ks[s] < mid && mid < ks[s + 1] {
                new_knots.push(mid);
            }
        }
        if new_knots.is_empty() {
            return Err(NurbsError::ToleranceNotMet { tol, achieved: max });
        }
        let mut v = ks.to_vec();
        v.extend(new_knots);
        v.sort_by(f64::total_cmp);
        knots = KnotVector::new(v)?;
    }
    Err(NurbsError::ToleranceNotMet {
        tol,
        achieved: f64::INFINITY,
    })
}

/// Samples per target span of [`approximate_onto_knots`] (at least `2 (degree + 1)`).
const ONTO_SAMPLES: usize = 8;

/// Re-approximates `c` on `degree` + `knots` (shared-knot compatibility, KH-14).
///
/// The source domain is mapped linearly onto the target domain. Data: `max(8, 2 (p + 1))`
/// samples per non-empty target span plus every interior source knot; the end points are
/// interpolated, the end tangents optionally constrained (`fix_end_tangents`: the source
/// derivative times `d s / d u`; explicit `ends` take precedence). After the first solve
/// `o.corrections` Hoschek passes re-solve with corrected parameters.
///
/// `max_error` is the largest distance from the source curve to the fit: every data sample
/// and every midpoint between samples is projected onto the fit (Newton), and the largest
/// local maxima are refined by golden-section search over the source parameter.
///
/// Errors: `Degree`, `CountMismatch` / `InvalidKnots` for the target knots,
/// `Degenerate` for constraints that do not fit, `Singular`, and `ToleranceNotMet` if
/// `max_error > o.tol`.
pub fn approximate_onto_knots<P: CtrlPt>(
    c: &NurbsCurve<P>,
    degree: usize,
    knots: &KnotVector,
    o: &OntoOpts<P>,
) -> Result<Fit<P>, NurbsError> {
    let p = degree;
    let n = check_degree_knots(p, knots)?;
    let ks = knots.as_slice();
    let (ta, tb) = knots.domain(p);
    let (sa, sb) = c.domain();
    let to_src = |u: f64| -> f64 {
        if u >= tb {
            sb
        } else {
            (sa + (u - ta) * ((sb - sa) / (tb - ta))).clamp(sa, sb)
        }
    };
    let to_tgt = |s: f64| -> f64 {
        if s >= sb {
            tb
        } else {
            (ta + (s - sa) * ((tb - ta) / (sb - sa))).clamp(ta, tb)
        }
    };
    let per = ONTO_SAMPLES.max(2 * (p + 1));
    let mut params: Vec<f64> = Vec::with_capacity((n - p) * per + c.knots().len() + 1);
    for s in p..n {
        let (k0, k1) = (ks[s], ks[s + 1]);
        if k0 < k1 {
            for i in 0..per {
                params.push(k0 + (k1 - k0) * (i as f64 / per as f64));
            }
        }
    }
    params.push(tb);
    for &k in c.knots().as_slice() {
        if k > sa && k < sb {
            params.push(to_tgt(k));
        }
    }
    params.sort_by(f64::total_cmp);
    params.dedup();
    let src: Vec<f64> = params.iter().map(|&u| to_src(u)).collect();
    let pts: Vec<P> = src.iter().map(|&s| c.eval(s)).collect();

    let scale = (sb - sa) / (tb - ta);
    let mut start: EndCons<P> = [Some(c.start()), None, None];
    let mut end: EndCons<P> = [Some(c.end()), None, None];
    if o.fix_end_tangents {
        start[1] = Some(c.derivs(sa, 1)[1] * scale);
        end[1] = Some(c.derivs(sb, 1)[1] * scale);
    }
    let given = [o.ends.start, o.ends.start2, o.ends.end, o.ends.end2];
    for (i, v) in given.into_iter().enumerate() {
        if v.is_some() {
            let side = if i < 2 { &mut start } else { &mut end };
            side[1 + i % 2] = v;
        }
    }
    if start
        .iter()
        .chain(&end)
        .flatten()
        .any(|d| !d.is_finite_pt())
    {
        return Err(NurbsError::Degenerate("end derivative is not finite"));
    }
    let mut cache = BasisCache::default();
    let mut fit = fit_once(p, knots, &pts, &params, &start, &end, &mut cache)?;
    let mut trial = params.clone();
    for _ in 0..o.corrections {
        hoschek(
            &cache,
            p,
            fit.poles(),
            fit.domain(),
            &pts,
            &params,
            &mut trial,
        );
        std::mem::swap(&mut params, &mut trial);
        fit = fit_once(p, knots, &pts, &params, &start, &end, &mut cache)?;
    }
    let (max_error, rms_error) = deviation(c, &fit, &src, &params);
    if max_error.is_nan() || max_error > o.tol {
        return Err(NurbsError::ToleranceNotMet {
            tol: o.tol,
            achieved: max_error,
        });
    }
    Ok(Fit {
        curve: fit,
        max_error,
        rms_error,
        params,
    })
}

/// Max / rms distance from the source curve `c` to `fit`, measured at the sample parameters
/// `src` (foot points near `tgt`) and their midpoints, refining the largest local maxima.
fn deviation<P: CtrlPt>(
    c: &NurbsCurve<P>,
    fit: &NurbsCurve<P>,
    src: &[f64],
    tgt: &[f64],
) -> (f64, f64) {
    let dist_at = |s: f64, guess: f64| -> (f64, f64) {
        let (t, d) = fit.newton_foot(c.eval(s), guess, 1e-12, FINAL_NEWTON);
        (d, t)
    };
    // Samples and midpoints, in order.
    let mut ss: Vec<(f64, f64)> = Vec::with_capacity(2 * src.len());
    for i in 0..src.len() {
        ss.push((src[i], tgt[i]));
        if i + 1 < src.len() {
            ss.push((0.5 * (src[i] + src[i + 1]), 0.5 * (tgt[i] + tgt[i + 1])));
        }
    }
    let ds: Vec<(f64, f64)> = ss.iter().map(|&(s, t)| dist_at(s, t)).collect();
    let mut max = ds.iter().fold(0.0f64, |m, d| m.max(d.0));
    let sq: f64 = ds.iter().map(|d| d.0 * d.0).sum();
    let rms = (sq / ds.len().max(1) as f64).sqrt();
    // Golden-section refinement of the local maxima above half the sampled maximum.
    let g = 0.5 * (5f64.sqrt() - 1.0);
    let thr = 0.5 * max;
    for i in 1..ds.len().saturating_sub(1) {
        if !(ds[i].0 >= ds[i - 1].0 && ds[i].0 >= ds[i + 1].0 && ds[i].0 > thr) {
            continue;
        }
        let (mut lo, mut hi) = (ss[i - 1].0, ss[i + 1].0);
        let guess = ds[i].1;
        let mut x1 = hi - g * (hi - lo);
        let mut x2 = lo + g * (hi - lo);
        let mut f1 = dist_at(x1, guess).0;
        let mut f2 = dist_at(x2, guess).0;
        for _ in 0..40 {
            if f1 > f2 {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - g * (hi - lo);
                f1 = dist_at(x1, guess).0;
            } else {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + g * (hi - lo);
                f2 = dist_at(x2, guess).0;
            }
            if hi - lo <= 1e-14 * (hi.abs() + lo.abs() + 1e-300) {
                break;
            }
        }
        max = max.max(f1).max(f2);
    }
    (max, rms)
}
