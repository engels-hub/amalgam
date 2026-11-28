//! Constrained least squares (P&T §9.4.1 / §9.4.2, A9.6).
//!
//! The end constraints of A9.6 that this crate exposes (end point, first and second end
//! derivative) each fix one end pole as an affine combination of the poles before it:
//! `C^(k)(a) = Σ_{i<=k} N_i^(k)(a) P_i`. Substituting them into the objective removes the
//! Lagrange multipliers of A9.6 without changing the minimiser (the constraint set is the same
//! affine subspace), and the reduced normal equations stay banded (bandwidth about `p`), so
//! one banded solve gives the result.

use smallvec::SmallVec;

use crate::basis::ders_basis_funs;
use crate::curve::check_knots;
use crate::knots::find_span;
use crate::linalg::Band;
use crate::{CtrlPt, KnotVector, MAX_DEGREE, NurbsCurve, NurbsError};

use super::{Fit, LsqOpts};

/// A pole as an affine combination of the free variables: `Σ c_v · x_v + k`.
#[derive(Clone, Debug)]
pub(crate) enum PoleExpr<P> {
    Free(usize),
    Fixed {
        terms: SmallVec<[(usize, f64); 4]>,
        k: P,
    },
}

/// Pole map of an LSQ problem.
#[derive(Clone, Debug)]
pub(crate) struct PoleMap<P> {
    pub(crate) poles: Vec<PoleExpr<P>>,
    pub(crate) n_free: usize,
}

/// `[point, C', C'']` constraints at one end.
pub(crate) type EndCons<P> = [Option<P>; 3];

/// Builds the pole map for `n` poles of degree `p` on clamped `knots` with the given end
/// constraints (index = derivative order).
pub(crate) fn pole_map<P: CtrlPt>(
    p: usize,
    knots: &[f64],
    n: usize,
    start: &EndCons<P>,
    end: &EndCons<P>,
) -> Result<PoleMap<P>, NurbsError> {
    let top = |c: &EndCons<P>| c.iter().rposition(Option::is_some);
    let (ks, ke) = (top(start), top(end));
    for k in [ks, ke].into_iter().flatten() {
        if k > p {
            return Err(NurbsError::Degenerate(
                "a derivative constraint exceeds the degree",
            ));
        }
    }
    let used_start = ks.map_or(0, |k| k + 1);
    let used_end = ke.map_or(0, |k| k + 1);
    if used_start + used_end > n {
        return Err(NurbsError::Degenerate(
            "too few poles for the end constraints",
        ));
    }
    // Free variables in pole order; constrained poles get placeholders first.
    let mut poles: Vec<Option<PoleExpr<P>>> = vec![None; n];
    let mut is_fixed = vec![false; n];
    for (k, c) in start.iter().enumerate() {
        if c.is_some() {
            is_fixed[k] = true;
        }
    }
    for (k, c) in end.iter().enumerate() {
        if c.is_some() {
            is_fixed[n - 1 - k] = true;
        }
    }
    let mut n_free = 0;
    for i in 0..n {
        if !is_fixed[i] {
            poles[i] = Some(PoleExpr::Free(n_free));
            n_free += 1;
        }
    }
    let (a, b) = (knots[p], knots[n]);
    let mut d = [[0.0; MAX_DEGREE + 1]; 3];
    // Start: N_i^(k)(a) is non-zero for i <= k only.
    ders_basis_funs(knots, find_span(knots, p, n, a), a, p, 2.min(p), &mut d);
    for (k, c) in start.iter().enumerate() {
        if let Some(target) = *c {
            let coef: SmallVec<[(usize, f64); 3]> = (0..k).map(|i| (i, d[k][i])).collect();
            poles[k] = Some(resolve(&poles, target, d[k][k], &coef));
        }
        if k == 0 {
            exact_end_point(&mut poles[0], *c);
        }
    }
    // End: pole n − 1 − p + j has basis index j in the last span; order k touches j >= p − k.
    let span = find_span(knots, p, n, b);
    ders_basis_funs(knots, span, b, p, 2.min(p), &mut d);
    let base = span - p;
    for (k, c) in end.iter().enumerate() {
        if let Some(target) = *c {
            let me = n - 1 - k;
            let coef: SmallVec<[(usize, f64); 3]> =
                (me + 1..n).map(|i| (i, d[k][i - base])).collect();
            poles[me] = Some(resolve(&poles, target, d[k][me - base], &coef));
        }
        if k == 0 {
            exact_end_point(&mut poles[n - 1], *c);
        }
    }
    let poles = poles
        .into_iter()
        .map(|e| e.ok_or(NurbsError::Degenerate("unresolved end constraint")))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(PoleMap { poles, n_free })
}

/// A clamped end pole equals the end point (`N(a) = 1` exactly; the basis recurrence can
/// round it to `1 ± ulp`), so a point constraint stores the point bit-identically.
fn exact_end_point<P: CtrlPt>(slot: &mut Option<PoleExpr<P>>, c: Option<P>) {
    if let Some(k) = c {
        *slot = Some(PoleExpr::Fixed {
            terms: SmallVec::new(),
            k,
        });
    }
}

/// `P_k = (target − Σ c_i P_i) / dk` as an expression of the free variables.
fn resolve<P: CtrlPt>(
    poles: &[Option<PoleExpr<P>>],
    target: P,
    dk: f64,
    coef: &[(usize, f64)],
) -> PoleExpr<P> {
    let inv = 1.0 / dk;
    let mut k = target * inv;
    let mut terms: SmallVec<[(usize, f64); 4]> = SmallVec::new();
    for &(i, c) in coef {
        let s = -c * inv;
        match &poles[i] {
            Some(PoleExpr::Free(v)) => push_term(&mut terms, *v, s),
            Some(PoleExpr::Fixed { terms: t, k: kk }) => {
                for &(v, cv) in t {
                    push_term(&mut terms, v, s * cv);
                }
                k = k + *kk * s;
            }
            None => {}
        }
    }
    PoleExpr::Fixed { terms, k }
}

fn push_term<A: smallvec::Array<Item = (usize, f64)>>(t: &mut SmallVec<A>, v: usize, c: f64) {
    match t.iter_mut().find(|e| e.0 == v) {
        Some(e) => e.1 += c,
        None => t.push((v, c)),
    }
}

/// Variable range `(min, max)` touched by pole `i`.
fn var_range<P>(e: &PoleExpr<P>) -> Option<(usize, usize)> {
    match e {
        PoleExpr::Free(v) => Some((*v, *v)),
        PoleExpr::Fixed { terms, .. } => terms.iter().fold(None, |r, &(v, _)| match r {
            None => Some((v, v)),
            Some((lo, hi)) => Some((lo.min(v), hi.max(v))),
        }),
    }
}

/// Spans and basis values of one least-squares assembly, reused for the deviation pass.
#[derive(Clone, Debug, Default)]
pub(crate) struct BasisCache {
    spans: Vec<usize>,
    vals: Vec<f64>,
    ders: Vec<f64>,
}

impl BasisCache {
    /// `|C(t_k) - Q_k|` for every point from the cached basis values (same `p` and
    /// parameters as the assembly) into `out`; returns the maximum.
    pub(crate) fn errors<P: CtrlPt>(
        &self,
        p: usize,
        poles: &[P],
        pts: &[P],
        out: &mut Vec<f64>,
    ) -> f64 {
        out.clear();
        let mut max = 0.0f64;
        for (k, (&s, &q)) in self.spans.iter().zip(pts).enumerate() {
            let mut c = P::zero();
            for (j, &nj) in self.vals[k * (p + 1)..(k + 1) * (p + 1)].iter().enumerate() {
                c = c + poles[s - p + j] * nj;
            }
            let e = (c - q).length();
            max = max.max(e);
            out.push(e);
        }
        max
    }

    /// Span of point `k`.
    pub(crate) fn span(&self, k: usize) -> usize {
        self.spans[k]
    }

    /// `(C(t_k), C'(t_k))` from the cached basis values.
    pub(crate) fn point_tangent<P: CtrlPt>(&self, k: usize, p: usize, poles: &[P]) -> (P, P) {
        let s = self.spans[k];
        let r = k * (p + 1)..(k + 1) * (p + 1);
        let (mut c, mut d) = (P::zero(), P::zero());
        for ((&n0, &n1), &pt) in self.vals[r.clone()]
            .iter()
            .zip(&self.ders[r])
            .zip(&poles[s - p..=s])
        {
            c = c + pt * n0;
            d = d + pt * n1;
        }
        (c, d)
    }
}

/// Solves the (weighted) least-squares problem for the poles. `params` must lie in the
/// domain; `weights` (if any) have one positive entry per point. The spans and basis values
/// are left in `cache`.
pub(crate) fn solve_poles<P: CtrlPt>(
    p: usize,
    knots: &[f64],
    map: &PoleMap<P>,
    pts: &[P],
    params: &[f64],
    weights: Option<&[f64]>,
    cache: &mut BasisCache,
) -> Result<Vec<P>, NurbsError> {
    cache.spans.clear();
    cache.vals.clear();
    cache.ders.clear();
    let n = map.poles.len();
    let nf = map.n_free;
    let mut out = vec![P::zero(); n];
    if nf > 0 {
        // Bandwidth over every span's pole window.
        let mut bw = 0;
        for s in p..n {
            let r = map.poles[s - p..=s].iter().filter_map(var_range).fold(
                None,
                |r: Option<(usize, usize)>, (lo, hi)| match r {
                    None => Some((lo, hi)),
                    Some((a, b)) => Some((a.min(lo), b.max(hi))),
                },
            );
            if let Some((lo, hi)) = r {
                bw = bw.max(hi - lo);
            }
        }
        let mut a = Band::new(nf, bw, bw);
        let mut rhs = vec![P::zero(); nf];
        let mut nb = [0.0; MAX_DEGREE + 1];
        let mut nd = [0.0; MAX_DEGREE + 1];
        let mut row: SmallVec<[(usize, f64); 16]> = SmallVec::new();
        for (k, (&q, &t)) in pts.iter().zip(params).enumerate() {
            let s = find_span(knots, p, n, t);
            crate::basis::ders1_into(knots, s, t, p, &mut nb, &mut nd);
            cache.spans.push(s);
            cache.vals.extend_from_slice(&nb[..=p]);
            cache.ders.extend_from_slice(&nd[..=p]);
            row.clear();
            let mut c = P::zero();
            for (j, &nj) in nb[..=p].iter().enumerate() {
                if nj == 0.0 {
                    continue;
                }
                match &map.poles[s - p + j] {
                    PoleExpr::Free(v) => push_term(&mut row, *v, nj),
                    PoleExpr::Fixed { terms, k } => {
                        for &(v, cv) in terms {
                            push_term(&mut row, v, nj * cv);
                        }
                        c = c + *k * nj;
                    }
                }
            }
            let w = weights.map_or(1.0, |w| w[k]);
            let r = q - c;
            for &(va, ca) in &row {
                let wa = w * ca;
                rhs[va] = rhs[va] + r * wa;
                for &(vb, cb) in &row {
                    a.add(va, vb, wa * cb);
                }
            }
        }
        a.solve(&mut rhs, "least squares")?;
        for (o, e) in out.iter_mut().zip(&map.poles) {
            if let PoleExpr::Free(v) = e {
                *o = rhs[*v];
            }
        }
        // Fixed poles from the solved free ones.
        for (i, e) in map.poles.iter().enumerate() {
            if let PoleExpr::Fixed { terms, k } = e {
                out[i] = terms.iter().fold(*k, |acc, &(v, cv)| acc + rhs[v] * cv);
            }
        }
    } else {
        for (o, e) in out.iter_mut().zip(&map.poles) {
            if let PoleExpr::Fixed { k, .. } = e {
                *o = *k;
            }
        }
    }
    if out.iter().all(|q| q.is_finite_pt()) {
        Ok(out)
    } else {
        Err(NurbsError::Singular("least squares"))
    }
}

/// `(max, rms)` of `|C(t_k) − Q_k|`.
pub(crate) fn param_errors<P: CtrlPt>(c: &NurbsCurve<P>, pts: &[P], params: &[f64]) -> (f64, f64) {
    let mut max = 0.0f64;
    let mut sq = 0.0;
    for (&q, &t) in pts.iter().zip(params) {
        let e = (c.eval(t) - q).length();
        max = max.max(e);
        sq += e * e;
    }
    let rms = if pts.is_empty() {
        0.0
    } else {
        (sq / pts.len() as f64).sqrt()
    };
    (max, rms)
}

/// Validates degree + knots and returns the pole count.
pub(crate) fn check_degree_knots(degree: usize, knots: &KnotVector) -> Result<usize, NurbsError> {
    if degree == 0 || degree > MAX_DEGREE {
        return Err(NurbsError::Degree(degree));
    }
    let n = knots.len().saturating_sub(degree + 1);
    check_knots(degree, knots, n)?;
    Ok(n)
}

/// §9.4.1 / A9.6: least-squares fit of `pts` at the given parameters onto the given degree and
/// knots, with optional exact end points (`fix_ends`), exact end derivatives and data weights.
///
/// `max_error` / `rms_error` are the parametric deviations `|C(t_k) − Q_k|` at `o.params`
/// (an upper bound of the point-to-curve distances). Errors: `Degree`, `CountMismatch` /
/// `InvalidKnots` for the knots, `Degenerate` for mismatched lengths, non-finite data or
/// constraints that do not fit the pole count, `OutOfDomain` for a parameter outside the knot
/// domain, `Weight` for a non-positive data weight, `Singular` if the data does not determine
/// the poles (a knot span without data, P&T §9.4.1 Schoenberg–Whitney).
pub fn approximate_lsq<P: CtrlPt>(pts: &[P], o: &LsqOpts<P>) -> Result<Fit<P>, NurbsError> {
    let p = o.degree;
    let n = check_degree_knots(p, &o.knots)?;
    if pts.is_empty() {
        return Err(NurbsError::Degenerate("no data points"));
    }
    if o.params.len() != pts.len() {
        return Err(NurbsError::Degenerate(
            "one parameter per data point is required",
        ));
    }
    if pts.iter().any(|q| !q.is_finite_pt()) {
        return Err(NurbsError::Degenerate("data point is not finite"));
    }
    let (a, b) = o.knots.domain(p);
    if let Some(&t) = o.params.iter().find(|t| !(**t >= a && **t <= b)) {
        return Err(NurbsError::OutOfDomain(t));
    }
    if let Some(w) = &o.data_weights {
        if w.len() != pts.len() {
            return Err(NurbsError::Degenerate(
                "one weight per data point is required",
            ));
        }
        if let Some(index) = w.iter().position(|x| !(x.is_finite() && *x > 0.0)) {
            return Err(NurbsError::Weight { index });
        }
    }
    let ends = [o.ends.start, o.ends.start2, o.ends.end, o.ends.end2];
    if ends.iter().flatten().any(|d| !d.is_finite_pt()) {
        return Err(NurbsError::Degenerate("end derivative is not finite"));
    }
    let mut start = o.ends.start_orders();
    let mut end = o.ends.end_orders();
    if o.fix_ends {
        start[0] = Some(pts[0]);
        end[0] = Some(pts[pts.len() - 1]);
    }
    let k = o.knots.as_slice();
    let map = pole_map(p, k, n, &start, &end)?;
    let mut cache = BasisCache::default();
    let poles = solve_poles(
        p,
        k,
        &map,
        pts,
        &o.params,
        o.data_weights.as_deref(),
        &mut cache,
    )?;
    let curve = NurbsCurve::new(p, o.knots.clone(), poles, None)?;
    let (max_error, rms_error) = param_errors(&curve, pts, &o.params);
    Ok(Fit {
        curve,
        max_error,
        rms_error,
        params: o.params.clone(),
    })
}
