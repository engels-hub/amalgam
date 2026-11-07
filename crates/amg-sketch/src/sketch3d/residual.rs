//! Residual blocks of the 3D constraints, with analytic Jacobians.
//!
//! Each block reads a fixed list of *logical* coordinates (for example the six end
//! coordinates of two lines); the same parameter may appear twice (two lines sharing a point),
//! so [`Res3::params`] lists each distinct parameter once and the Jacobian sums the logical
//! columns into it.

use amg_core::{Point2, Point3, Vec3};

use super::{Axis, Constraint3, Sketch3d, SplineEnd};
use crate::error::SketchError;
use crate::geom::BSpline2;
use crate::ids::{EntityId, ParamId};
use crate::solve::system::{EquationOwner, Residual};

/// What a block computes. Logical layouts are documented per variant.
enum Kind {
    /// `a(3), b(3)`: `a − b`.
    Coincident,
    /// `p(3), s(3), e(3), t`: `p − s − t (e − s)`.
    OnLine,
    /// `p(3), poles(3 n), t`: `p − C(t)`. `basis` evaluates the basis functions.
    OnSpline { basis: BSpline2, n: usize },
    /// `a(3), b(3)`: `|b − a| − value`.
    Distance(f64),
    /// `u0, u1, v0, v1`: `L_c (∠(u, v) − value)`.
    Angle { value: f64, lc: f64 },
    /// `u0, u1, v0, v1, s`: `v − s u`.
    Parallel,
    /// `u0, u1, v0, v1`: `L_c cos ∠(u, v)`.
    Perpendicular { lc: f64 },
    /// `u0, u1, v0, v1`: `|u| − |v|`.
    EqualLength,
    /// `s(3), e(3)`: the two components of `e − s` off the axis.
    Axis(Axis),
}

/// One 3D constraint's equations.
pub(super) struct Res3 {
    kind: Kind,
    /// Logical coordinates.
    slots: Vec<ParamId>,
    /// Distinct parameters, in first-use order.
    params: Vec<ParamId>,
    /// Column in `params` of each slot.
    col: Vec<usize>,
}

fn v3(x: &[f64], ps: &[ParamId], i: usize) -> Vec3 {
    Vec3::new(x[ps[i].index()], x[ps[i + 1].index()], x[ps[i + 2].index()])
}

/// `j[r][c..c + 3] = v` in a row-major matrix with `ns` columns.
fn put(j: &mut [f64], ns: usize, r: usize, c: usize, v: Vec3) {
    j[r * ns + c..r * ns + c + 3].copy_from_slice(&v.to_array());
}

fn safe_unit(v: Vec3) -> Vec3 {
    let l = v.length();
    if l > 0.0 { v / l } else { Vec3::ZERO }
}

/// Point and derivative of a clamped B-spline at `t` (the end spans extend beyond the
/// range).
pub(super) fn spline_eval(degree: usize, knots: &[f64], pts: &[Point3], t: f64) -> (Point3, Vec3) {
    let basis = basis_spline(degree, knots, pts.len());
    let span = basis.find_span(t);
    let d = basis.basis_derivs(span, t);
    let (mut c, mut dc) = (Point3::ZERO, Vec3::ZERO);
    for j in 0..=degree {
        let p = pts[span - degree + j];
        c += d[0][j] * p;
        dc += d[1][j] * p;
    }
    (c, dc)
}

fn basis_spline(degree: usize, knots: &[f64], n: usize) -> BSpline2 {
    BSpline2::unchecked(degree, knots.to_vec(), vec![Point2::ZERO; n], vec![1.0; n])
}

impl Res3 {
    fn new(kind: Kind, slots: Vec<ParamId>) -> Res3 {
        let mut params: Vec<ParamId> = Vec::with_capacity(slots.len());
        let col = slots
            .iter()
            .map(|p| match params.iter().position(|q| q == p) {
                Some(i) => i,
                None => {
                    params.push(*p);
                    params.len() - 1
                }
            })
            .collect();
        Res3 {
            kind,
            slots,
            params,
            col,
        }
    }

    /// The block of `kind` with auxiliary parameters `aux`; `lc` scales angular rows.
    pub(super) fn build(
        s: &Sketch3d,
        kind: &Constraint3,
        aux: &[ParamId],
        lc: f64,
    ) -> Result<Res3, SketchError> {
        use Constraint3 as C;
        let pt = |id: EntityId| s.point_params(id);
        let line = |id: EntityId| -> Result<Vec<ParamId>, SketchError> {
            let (a, b) = s.line_ends(id)?;
            Ok([pt(a)?, pt(b)?].concat())
        };
        let aux0 = || {
            aux.first()
                .copied()
                .ok_or(SketchError::NotApplicable("missing auxiliary parameter"))
        };
        Ok(match *kind {
            C::Coincident { a, b } => Res3::new(Kind::Coincident, [pt(a)?, pt(b)?].concat()),
            C::PointOnLine { point, line: l } => {
                let mut slots = [pt(point)?.to_vec(), line(l)?].concat();
                slots.push(aux0()?);
                Res3::new(Kind::OnLine, slots)
            }
            C::PointOnSpline { point, spline } => {
                let (degree, poles, knots) = s.spline(spline)?;
                let mut slots = pt(point)?.to_vec();
                for &p in poles {
                    slots.extend(pt(p)?);
                }
                slots.push(aux0()?);
                let basis = basis_spline(degree, knots, poles.len());
                Res3::new(
                    Kind::OnSpline {
                        basis,
                        n: poles.len(),
                    },
                    slots,
                )
            }
            C::Distance { a, b, value } => {
                Res3::new(Kind::Distance(value), [pt(a)?, pt(b)?].concat())
            }
            C::Length { line: l, value } => Res3::new(Kind::Distance(value), line(l)?),
            C::Angle { a, b, value } => {
                Res3::new(Kind::Angle { value, lc }, [line(a)?, line(b)?].concat())
            }
            C::Parallel { a, b } => {
                let mut slots = [line(a)?, line(b)?].concat();
                slots.push(aux0()?);
                Res3::new(Kind::Parallel, slots)
            }
            C::Tangent {
                line: l,
                spline,
                end,
            } => {
                let (_, poles, _) = s.spline(spline)?;
                let n = poles.len();
                let (p0, p1) = match end {
                    SplineEnd::Start => (poles[0], poles[1]),
                    SplineEnd::End => (poles[n - 2], poles[n - 1]),
                };
                let mut slots = [line(l)?, pt(p0)?.to_vec(), pt(p1)?.to_vec()].concat();
                slots.push(aux0()?);
                Res3::new(Kind::Parallel, slots)
            }
            C::Perpendicular { a, b } => {
                Res3::new(Kind::Perpendicular { lc }, [line(a)?, line(b)?].concat())
            }
            C::EqualLength { a, b } => Res3::new(Kind::EqualLength, [line(a)?, line(b)?].concat()),
            C::AlongAxis { line: l, axis } => Res3::new(Kind::Axis(axis), line(l)?),
        })
    }

    /// Row-major logical Jacobian `rows × slots.len()`.
    fn logical_jacobian(&self, x: &[f64], j: &mut [f64]) {
        let ns = self.slots.len();
        let ps = &self.slots;
        j.fill(0.0);
        match &self.kind {
            Kind::Coincident => {
                for r in 0..3 {
                    put(j, ns, r, 0, Vec3::AXES[r]);
                    put(j, ns, r, 3, -Vec3::AXES[r]);
                }
            }
            Kind::OnLine => {
                let (s, e, t) = (v3(x, ps, 3), v3(x, ps, 6), x[ps[9].index()]);
                let d = e - s;
                for r in 0..3 {
                    put(j, ns, r, 0, Vec3::AXES[r]);
                    put(j, ns, r, 3, -(1.0 - t) * Vec3::AXES[r]);
                    put(j, ns, r, 6, -t * Vec3::AXES[r]);
                    j[r * ns + 9] = -d[r];
                }
            }
            Kind::OnSpline { basis, n } => {
                let t = x[ps[3 + 3 * n].index()];
                let p = basis.degree();
                let span = basis.find_span(t);
                let d = basis.basis_derivs(span, t);
                let mut dc = Vec3::ZERO;
                for (k, dk) in d[1].iter().enumerate().take(p + 1) {
                    dc += *dk * v3(x, ps, 3 + 3 * (span - p + k));
                }
                for r in 0..3 {
                    put(j, ns, r, 0, Vec3::AXES[r]);
                    for (k, nk) in d[0].iter().enumerate().take(p + 1) {
                        j[r * ns + 3 + 3 * (span - p + k) + r] = -nk;
                    }
                    j[r * ns + 3 + 3 * n] = -dc[r];
                }
            }
            Kind::Distance(_) => {
                let u = safe_unit(v3(x, ps, 3) - v3(x, ps, 0));
                put(j, ns, 0, 0, -u);
                put(j, ns, 0, 3, u);
            }
            Kind::Angle { lc, .. } => {
                let (u, v) = (v3(x, ps, 3) - v3(x, ps, 0), v3(x, ps, 9) - v3(x, ps, 6));
                let c = u.cross(v);
                let (cn, d) = (c.length(), u.dot(v));
                let den = cn * cn + d * d;
                if den > 0.0 {
                    let ch = safe_unit(c);
                    let du = (d * v.cross(ch) - cn * v) / den * *lc;
                    let dv = (d * ch.cross(u) - cn * u) / den * *lc;
                    put(j, ns, 0, 0, -du);
                    put(j, ns, 0, 3, du);
                    put(j, ns, 0, 6, -dv);
                    put(j, ns, 0, 9, dv);
                }
            }
            Kind::Parallel => {
                let u = v3(x, ps, 3) - v3(x, ps, 0);
                let s = x[ps[12].index()];
                for r in 0..3 {
                    put(j, ns, r, 0, s * Vec3::AXES[r]);
                    put(j, ns, r, 3, -s * Vec3::AXES[r]);
                    put(j, ns, r, 6, -Vec3::AXES[r]);
                    put(j, ns, r, 9, Vec3::AXES[r]);
                    j[r * ns + 12] = -u[r];
                }
            }
            Kind::Perpendicular { lc } => {
                let (u, v) = (v3(x, ps, 3) - v3(x, ps, 0), v3(x, ps, 9) - v3(x, ps, 6));
                let (lu, lv) = (u.length(), v.length());
                if lu > 0.0 && lv > 0.0 {
                    let d = u.dot(v);
                    let du = (v / (lu * lv) - d * u / (lu * lu * lu * lv)) * *lc;
                    let dv = (u / (lu * lv) - d * v / (lv * lv * lv * lu)) * *lc;
                    put(j, ns, 0, 0, -du);
                    put(j, ns, 0, 3, du);
                    put(j, ns, 0, 6, -dv);
                    put(j, ns, 0, 9, dv);
                }
            }
            Kind::EqualLength => {
                let (u, v) = (v3(x, ps, 3) - v3(x, ps, 0), v3(x, ps, 9) - v3(x, ps, 6));
                let (uh, vh) = (safe_unit(u), safe_unit(v));
                put(j, ns, 0, 0, -uh);
                put(j, ns, 0, 3, uh);
                put(j, ns, 0, 6, vh);
                put(j, ns, 0, 9, -vh);
            }
            Kind::Axis(axis) => {
                for (r, k) in off_axis(*axis).into_iter().enumerate() {
                    j[r * ns + k] = -1.0;
                    j[r * ns + 3 + k] = 1.0;
                }
            }
        }
    }
}

/// The two coordinate indices that a line along `axis` keeps equal at both ends.
fn off_axis(axis: Axis) -> [usize; 2] {
    match axis {
        Axis::X => [1, 2],
        Axis::Y => [0, 2],
        Axis::Z => [0, 1],
    }
}

impl Residual for Res3 {
    fn rows(&self) -> usize {
        match self.kind {
            Kind::Coincident | Kind::OnLine | Kind::OnSpline { .. } | Kind::Parallel => 3,
            Kind::Axis(_) => 2,
            Kind::Distance(_)
            | Kind::Angle { .. }
            | Kind::Perpendicular { .. }
            | Kind::EqualLength => 1,
        }
    }

    fn params(&self) -> &[ParamId] {
        &self.params
    }

    fn eval(&self, x: &[f64], out: &mut [f64]) {
        let ps = &self.slots;
        let out3 = |out: &mut [f64], v: Vec3| out[..3].copy_from_slice(&v.to_array());
        match &self.kind {
            Kind::Coincident => out3(out, v3(x, ps, 0) - v3(x, ps, 3)),
            Kind::OnLine => {
                let (p, s, e, t) = (v3(x, ps, 0), v3(x, ps, 3), v3(x, ps, 6), x[ps[9].index()]);
                out3(out, p - s - t * (e - s));
            }
            Kind::OnSpline { basis, n } => {
                let t = x[ps[3 + 3 * n].index()];
                let p = basis.degree();
                let (span, d) = basis.basis_at(t);
                let mut c = Vec3::ZERO;
                for (k, dk) in d.iter().enumerate().take(p + 1) {
                    c += *dk * v3(x, ps, 3 + 3 * (span - p + k));
                }
                out3(out, v3(x, ps, 0) - c);
            }
            Kind::Distance(value) => out[0] = (v3(x, ps, 3) - v3(x, ps, 0)).length() - value,
            Kind::Angle { value, lc } => {
                let (u, v) = (v3(x, ps, 3) - v3(x, ps, 0), v3(x, ps, 9) - v3(x, ps, 6));
                out[0] = (u.cross(v).length().atan2(u.dot(v)) - value) * lc;
            }
            Kind::Parallel => {
                let (u, v) = (v3(x, ps, 3) - v3(x, ps, 0), v3(x, ps, 9) - v3(x, ps, 6));
                out3(out, v - x[ps[12].index()] * u);
            }
            Kind::Perpendicular { lc } => {
                let (u, v) = (v3(x, ps, 3) - v3(x, ps, 0), v3(x, ps, 9) - v3(x, ps, 6));
                let l = u.length() * v.length();
                out[0] = if l > 0.0 { u.dot(v) / l * lc } else { 0.0 };
            }
            Kind::EqualLength => {
                let (u, v) = (v3(x, ps, 3) - v3(x, ps, 0), v3(x, ps, 9) - v3(x, ps, 6));
                out[0] = u.length() - v.length();
            }
            Kind::Axis(axis) => {
                for (r, k) in off_axis(*axis).into_iter().enumerate() {
                    out[r] = x[ps[3 + k].index()] - x[ps[k].index()];
                }
            }
        }
    }

    fn jacobian(&self, x: &[f64], out: &mut [f64]) {
        let (rows, ns, np) = (self.rows(), self.slots.len(), self.params.len());
        let mut j = vec![0.0; rows * ns];
        self.logical_jacobian(x, &mut j);
        out[..rows * np].fill(0.0);
        for r in 0..rows {
            for (c, &k) in self.col.iter().enumerate() {
                out[r * np + k] += j[r * ns + c];
            }
        }
    }
}

/// Start value of a constraint's auxiliary parameter from the current geometry: the
/// projection parameter on a line, the closest sampled spline parameter, or the ratio
/// `v·u / u·u` of a parallel pair.
pub(super) fn aux_start(s: &Sketch3d, kind: &Constraint3) -> f64 {
    let p = |id: EntityId| s.point(id).unwrap_or(Point3::ZERO);
    let dir = |l: EntityId| {
        s.line(l)
            .map(|(a, b)| (a, b - a))
            .unwrap_or((Point3::ZERO, Vec3::ZERO))
    };
    let ratio = |u: Vec3, v: Vec3| {
        let uu = u.length_squared();
        if uu > 0.0 { v.dot(u) / uu } else { 1.0 }
    };
    match *kind {
        Constraint3::PointOnLine { point, line } => {
            let (a, d) = dir(line);
            ratio(d, p(point) - a)
        }
        Constraint3::PointOnSpline { point, spline } => {
            let Ok((_, _, knots)) = s.spline(spline) else {
                return 0.0;
            };
            let (t0, t1) = (knots[0], knots[knots.len() - 1]);
            let q = p(point);
            (0..=256)
                .map(|i| t0 + (t1 - t0) * f64::from(i) / 256.0)
                .map(|t| {
                    (
                        s.spline_point(spline, t)
                            .map_or(f64::INFINITY, |c| c.distance(q)),
                        t,
                    )
                })
                .fold((f64::INFINITY, t0), |m, c| if c.0 < m.0 { c } else { m })
                .1
        }
        Constraint3::Parallel { a, b } => ratio(dir(a).1, dir(b).1),
        Constraint3::Tangent { line, spline, end } => {
            let Ok((_, poles, _)) = s.spline(spline) else {
                return 1.0;
            };
            let n = poles.len();
            let leg = match end {
                SplineEnd::Start => p(poles[1]) - p(poles[0]),
                SplineEnd::End => p(poles[n - 1]) - p(poles[n - 2]),
            };
            ratio(dir(line).1, leg)
        }
        _ => 0.0,
    }
}

/// Parameter pairs of a block that is exactly `p − q = 0` (coincidence, parallel to an axis).
pub(super) fn subst_pairs(s: &Sketch3d, owner: EquationOwner) -> Option<crate::solve::SubstPairs> {
    let EquationOwner::Constraint(id) = owner else {
        return None;
    };
    let mut out = crate::solve::SubstPairs::new();
    match s.constraint(id)?.kind {
        Constraint3::Coincident { a, b } => {
            let (p, q) = (s.point_params(a).ok()?, s.point_params(b).ok()?);
            out.extend(p.into_iter().zip(q));
        }
        Constraint3::AlongAxis { line, axis } => {
            let (a, b) = s.line_ends(line).ok()?;
            let (p, q) = (s.point_params(a).ok()?, s.point_params(b).ok()?);
            out.extend(off_axis(axis).into_iter().map(|k| (p[k], q[k])));
        }
        _ => return None,
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Central differences of `eval` against `jacobian` at the sketch's values, every block.
    fn check_jacobians(s: &Sketch3d) {
        let sys = s.system().unwrap();
        let x0 = s.params().to_vec();
        for (_, r, rows) in sys.system.iter() {
            let (m, np) = (rows.len(), r.params().len());
            let mut j = vec![0.0; m * np];
            r.jacobian(&x0, &mut j);
            for (c, p) in r.params().iter().enumerate() {
                let h = 1e-6;
                let (mut xp, mut xm) = (x0.clone(), x0.clone());
                xp[p.index()] += h;
                xm[p.index()] -= h;
                let (mut fp, mut fm) = (vec![0.0; m], vec![0.0; m]);
                r.eval(&xp, &mut fp);
                r.eval(&xm, &mut fm);
                for i in 0..m {
                    let fd = (fp[i] - fm[i]) / (2.0 * h);
                    let an = j[i * np + c];
                    assert!(
                        (fd - an).abs() <= 1e-6 * (1.0 + an.abs()),
                        "row {i} col {c}: fd {fd} vs analytic {an}"
                    );
                }
            }
        }
    }

    #[test]
    fn jacobians_match_finite_differences() {
        let mut s = Sketch3d::new();
        let p = |x, y, z| Point3::new(x, y, z);
        let a = s.add_point(p(0.0, 0.0, 0.0)).unwrap();
        let b = s.add_point(p(10.0, 1.0, 2.0)).unwrap();
        let c = s.add_point(p(12.0, 9.0, -3.0)).unwrap();
        let d = s.add_point(p(3.0, 7.0, 5.0)).unwrap();
        let l1 = s.add_line(a, b).unwrap();
        let l2 = s.add_line(b, c).unwrap();
        let l3 = s.add_line(c, d).unwrap();
        let v = s.add_model_vertex(p(1.0, 2.0, 3.0), None).unwrap();
        let free = s.add_point(p(5.0, 5.0, 1.0)).unwrap();
        let sp = s.add_spline(3, &[a, b, c, d, v]).unwrap();
        use Constraint3 as C;
        for k in [
            C::Coincident { a: d, b: v },
            C::PointOnLine {
                point: free,
                line: l1,
            },
            C::PointOnSpline {
                point: free,
                spline: sp,
            },
            C::Distance {
                a,
                b: c,
                value: 4.0,
            },
            C::Length {
                line: l2,
                value: 3.0,
            },
            C::Angle {
                a: l1,
                b: l2,
                value: 1.0,
            },
            C::Angle {
                a: l1,
                b: l3,
                value: 2.0,
            },
            C::Parallel { a: l1, b: l3 },
            C::Parallel { a: l1, b: l2 },
            C::Perpendicular { a: l2, b: l3 },
            C::EqualLength { a: l1, b: l2 },
            C::AlongAxis {
                line: l3,
                axis: Axis::Y,
            },
            C::Tangent {
                line: l1,
                spline: sp,
                end: SplineEnd::End,
            },
        ] {
            s.add_constraint(k).unwrap();
        }
        check_jacobians(&s);
    }

    #[test]
    fn spline_ends_are_the_end_poles() {
        let pts = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(3.0, 2.0, 1.0),
            Point3::new(4.0, 0.0, 2.0),
        ];
        let knots = crate::geom::clamped_uniform_knots(4, 2);
        let (c0, _) = spline_eval(2, &knots, &pts, 0.0);
        let (c1, _) = spline_eval(2, &knots, &pts, knots[knots.len() - 1]);
        assert!(c0.distance(pts[0]) < 1e-12);
        assert!(c1.distance(pts[3]) < 1e-12);
    }
}
