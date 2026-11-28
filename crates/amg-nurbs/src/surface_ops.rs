//! Surface knot operations per parametric direction (KH-12): A5.3 knot insertion, A5.5 knot
//! refinement, A5.7 Bézier decomposition into strips, A5.10 degree elevation, split.
//!
//! Each operation applies the curve kernel of `knot_ops` to every pole row / column along the
//! chosen direction (the knot arithmetic is identical for all of them, which is exactly what
//! A5.3 / A5.5 / A5.7 / A5.10 do).

use crate::curve_ops::{mult_last, prepare_refine, snap, snap_eps};
use crate::knot_ops::{self, H};
use crate::knots::find_span;
use crate::{CtrlPt, Grid, KnotVector, MAX_DEGREE, NurbsError, NurbsSurface, UV};

impl<P: CtrlPt> NurbsSurface<P> {
    #[inline]
    fn di(d: UV) -> usize {
        match d {
            UV::U => 0,
            UV::V => 1,
        }
    }

    /// Homogeneous pole lines along `d`: for `UV::U` one line per column `j` (varying `i`),
    /// for `UV::V` one line per row `i`.
    fn lines(&self, d: UV) -> Vec<Vec<H<P>>> {
        let g = self.poles();
        let w = self.weights();
        let (nu, nv) = (g.nu(), g.nv());
        let h = |i: usize, j: usize| {
            let p = g.get(i, j);
            match w {
                None => H { p, w: 1.0 },
                Some(w) => {
                    let wi = w.get(i, j);
                    H { p: p * wi, w: wi }
                }
            }
        };
        match d {
            UV::U => (0..nv)
                .map(|j| (0..nu).map(|i| h(i, j)).collect())
                .collect(),
            UV::V => (0..nu)
                .map(|i| (0..nv).map(|j| h(i, j)).collect())
                .collect(),
        }
    }

    /// Rebuilds a surface from new lines along `d` with new degree / knots in that direction.
    fn rebuild_lines(&self, d: UV, degree: usize, knots: Vec<f64>, lines: &[Vec<H<P>>]) -> Self {
        let rational = self.weights().is_some();
        let len = lines[0].len();
        let (nu, nv) = match d {
            UV::U => (len, lines.len()),
            UV::V => (lines.len(), len),
        };
        let at = |i: usize, j: usize| match d {
            UV::U => lines[j][i],
            UV::V => lines[i][j],
        };
        let poles = Grid::from_fn(nu, nv, |i, j| {
            let h = at(i, j);
            if rational { h.p * (1.0 / h.w) } else { h.p }
        });
        let weights = rational.then(|| Grid::from_fn(nu, nv, |i, j| at(i, j).w));
        let k = Self::di(d);
        let mut deg = [self.degree(UV::U), self.degree(UV::V)];
        deg[k] = degree;
        let mut kn = [self.knots(UV::U).clone(), self.knots(UV::V).clone()];
        kn[k] = KnotVector::new_unchecked(knots);
        NurbsSurface::new_unchecked(deg, kn, poles, weights)
    }

    /// Applies one line kernel to every line along `d`.
    fn map_lines(
        &self,
        d: UV,
        degree: usize,
        f: impl Fn(&[H<P>]) -> (Vec<f64>, Vec<H<P>>),
    ) -> Self {
        let mut knots = Vec::new();
        let lines: Vec<Vec<H<P>>> = self
            .lines(d)
            .iter()
            .map(|l| {
                let (k, q) = f(l);
                knots = k;
                q
            })
            .collect();
        self.rebuild_lines(d, degree, knots, &lines)
    }

    /// Inserts the knot `t` in direction `d` up to `times` times (A5.3); same rules as
    /// [`crate::NurbsCurve::insert_knot`].
    pub fn insert_knot(&self, d: UV, t: f64, times: usize) -> Result<Self, NurbsError> {
        let (a, b) = self.domain()[Self::di(d)];
        let u = self.knots(d).as_slice();
        let eps = snap_eps(a, b);
        if !(t.is_finite() && t >= a - eps && t <= b + eps) {
            return Err(NurbsError::OutOfDomain(t));
        }
        let t = snap(u, t, eps);
        if t - a <= eps || b - t <= eps {
            return Ok(self.clone());
        }
        let p = self.degree(d);
        let (s, _) = mult_last(u, t);
        let r = times.min(p.saturating_sub(s));
        if r == 0 {
            return Ok(self.clone());
        }
        let n = u.len() - p - 1;
        let k = find_span(u, p, n, t);
        Ok(self.map_lines(d, p, |l| knot_ops::insert(p, u, l, t, k, s, r)))
    }

    /// Inserts all of `ts` in direction `d` (A5.5); same rules as
    /// [`crate::NurbsCurve::refine_knots`].
    pub fn refine_knots(&self, d: UV, ts: &[f64]) -> Result<Self, NurbsError> {
        let p = self.degree(d);
        let u = self.knots(d).as_slice();
        let x = prepare_refine(u, p, self.domain()[Self::di(d)], ts)?;
        if x.is_empty() {
            return Ok(self.clone());
        }
        Ok(self.map_lines(d, p, |l| knot_ops::refine(p, u, l, &x)))
    }

    /// Raises the degree in direction `d` by `by` (A5.10: A5.9 per pole line, with the same
    /// blossom fallback for uneven knots as [`crate::NurbsCurve::elevate_degree`]).
    /// `Degree` above [`MAX_DEGREE`].
    pub fn elevate_degree(&self, d: UV, by: usize) -> Result<Self, NurbsError> {
        let p = self.degree(d);
        if by == 0 {
            return Ok(self.clone());
        }
        let ph = p.saturating_add(by);
        if ph > MAX_DEGREE {
            return Err(NurbsError::Degree(ph));
        }
        let u = self.knots(d).as_slice();
        Ok(self.map_lines(d, ph, |l| knot_ops::elevate_any(p, u, l, by)))
    }

    /// Bézier strips along `d` (A5.7): one surface per non-empty span of direction `d`, each
    /// with knots `[u_i; p+1] ++ [u_{i+1}; p+1]` in `d` (parameters kept) and the other
    /// direction unchanged. Decompose both directions with
    /// `s.bezier_strips(UV::U)` then `strip.bezier_strips(UV::V)`.
    pub fn bezier_strips(&self, d: UV) -> Vec<Self> {
        let p = self.degree(d);
        let u = self.knots(d).as_slice();
        let per_line: Vec<Vec<knot_ops::BezierSeg<P>>> = self
            .lines(d)
            .iter()
            .map(|l| knot_ops::decompose(p, u, l))
            .collect();
        let nseg = per_line[0].len();
        (0..nseg)
            .map(|s| {
                let (a, b) = per_line[0][s].range;
                let mut k = vec![a; p + 1];
                k.resize(2 * (p + 1), b);
                let lines: Vec<Vec<H<P>>> =
                    per_line.iter().map(|segs| segs[s].pts.clone()).collect();
                self.rebuild_lines(d, p, k, &lines)
            })
            .collect()
    }

    /// Splits at `t` in direction `d` (insertion to multiplicity `degree`); the two halves
    /// share the boundary poles bit-identically. `OutOfDomain` unless `t` is strictly inside.
    pub fn split(&self, d: UV, t: f64) -> Result<(Self, Self), NurbsError> {
        let (a, b) = self.domain()[Self::di(d)];
        let eps = snap_eps(a, b);
        let p = self.degree(d);
        if !(t.is_finite() && t - a > eps && b - t > eps) {
            return Err(NurbsError::OutOfDomain(t));
        }
        let t = snap(self.knots(d).as_slice(), t, eps);
        if !(t - a > eps && b - t > eps) {
            return Err(NurbsError::OutOfDomain(t));
        }
        let full = self.insert_knot(d, t, p)?;
        let u = full.knots(d).as_slice();
        let f = u.partition_point(|&k| k < t);
        let mut lk = u[..f + p].to_vec();
        lk.push(t);
        let mut rk = vec![t];
        rk.extend_from_slice(&u[f..]);
        let lines = full.lines(d);
        let left: Vec<Vec<H<P>>> = lines.iter().map(|l| l[..f].to_vec()).collect();
        let right: Vec<Vec<H<P>>> = lines.iter().map(|l| l[f - 1..].to_vec()).collect();
        Ok((
            full.rebuild_lines(d, p, lk, &left),
            full.rebuild_lines(d, p, rk, &right),
        ))
    }
}
