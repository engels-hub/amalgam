//! `amg-nurbs` ↔ kernel geometry (S20 KH-16): conversions between [`NurbsCurve3`] /
//! [`NurbsSurface3`] and S02's plain B-spline data (OCCT's knots-and-multiplicities form),
//! exact NURBS of analytic curves and surfaces, edges and wires as NURBS, and faces on NURBS
//! surfaces. No OCCT type is public: everything goes through the [`geom`] data.
//!
//! Conventions:
//! - [`From`] (NURBS → data) is exact and never periodic. Knots group by exact equality.
//!   Weights are kept when the curve is rational and dropped when they are all equal (the same
//!   curve), because OCCT stores no weights for such a curve either.
//! - [`TryFrom`] (data → NURBS) is exact for clamped data. Periodic data is unwrapped and
//!   clamped by knot insertion (Boehm), which is exact up to rounding.
//! - So a round trip NURBS → data → OCCT → data → NURBS is bit-identical (KH-16 Done-when 1).
//! - Analytic conversions use `amg_nurbs::analytic` (rational quadratic arcs of at most 90°).
//!   Lines and planes keep the analytic parameter; arcs keep the angle at segment boundaries
//!   (the rational parametrisation differs in between). Every result's domain is the requested
//!   parameter range.
//!
//! [`oracle`] holds the OCCT evaluation, `SetNotPeriodic` and `GeomFill_Gordon` oracles used by
//! the differential tests here and by KH-15.

use std::f64::consts::{FRAC_PI_2, TAU};

use amg_core::{Dir3, Line3, Plane, Point3, Vec3};
use amg_nurbs::{Grid, KnotVector, NurbsCurve3, NurbsError, NurbsSurface3, UV};

use crate::error::{KResult, KernelError};
use crate::geom::{BSplineCurveData, BSplineSurfaceData, Curve3, CurveSeg, FaceBounds, Surface};
use crate::history::FaceBuild;
use crate::kernel::BrepKernel;
use crate::shape::{EdgeId, Shape, ShapeKind};

/// Slack on angle ranges (a full turn may exceed 2π by this much).
const ANGLE_EPS: f64 = 1e-12;

fn invalid(what: &'static str, reason: impl Into<String>) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason: reason.into(),
        offending: Vec::new(),
    }
}

fn nurbs_err(what: &'static str) -> impl Fn(NurbsError) -> KernelError {
    move |e| invalid(what, e.to_string())
}

// ---------------------------------------------------------------------------------------------
// NURBS → data
// ---------------------------------------------------------------------------------------------

fn knots_mults(k: &KnotVector) -> (Vec<f64>, Vec<u32>) {
    let (knots, mults) = k.to_knots_mults(0.0);
    // A knot vector holds at most 2^24 knots (`from_knots_mults`), so a multiplicity fits.
    (knots, mults.into_iter().map(|m| m as u32).collect())
}

fn degree_u8(d: usize) -> u8 {
    // `amg_nurbs::MAX_DEGREE` is 25.
    u8::try_from(d).unwrap_or(u8::MAX)
}

impl From<&NurbsCurve3> for BSplineCurveData {
    fn from(c: &NurbsCurve3) -> Self {
        let (knots, mults) = knots_mults(c.knots());
        BSplineCurveData {
            degree: degree_u8(c.degree()),
            knots,
            mults,
            poles: c.poles().to_vec(),
            weights: c.weights().filter(|_| c.is_rational()).map(<[f64]>::to_vec),
            periodic: false,
        }
    }
}

impl From<&NurbsSurface3> for BSplineSurfaceData {
    fn from(s: &NurbsSurface3) -> Self {
        let (u_knots, u_mults) = knots_mults(s.knots(UV::U));
        let (v_knots, v_mults) = knots_mults(s.knots(UV::V));
        let poles = s.poles();
        BSplineSurfaceData {
            u_degree: degree_u8(s.degree(UV::U)),
            v_degree: degree_u8(s.degree(UV::V)),
            u_knots,
            u_mults,
            v_knots,
            v_mults,
            poles: poles.as_slice().to_vec(),
            nu: poles.nu() as u32,
            nv: poles.nv() as u32,
            weights: s
                .weights()
                .filter(|_| s.is_rational())
                .map(|w| w.as_slice().to_vec()),
            u_periodic: false,
            v_periodic: false,
        }
    }
}

// ---------------------------------------------------------------------------------------------
// data → NURBS
// ---------------------------------------------------------------------------------------------

fn usize_mults(m: &[u32]) -> Vec<usize> {
    m.iter().map(|&m| m as usize).collect()
}

/// Homogeneous control point `(w·x, w·y, w·z, w)`.
type H = [f64; 4];

fn lerp(a: H, b: H, t: f64) -> H {
    // `(1 − t)·a + t·b`, the form of P&T A5.1.
    let s = 1.0 - t;
    [
        t * b[0] + s * a[0],
        t * b[1] + s * a[1],
        t * b[2] + s * a[2],
        t * b[3] + s * a[3],
    ]
}

/// Inserts `u` `r` times into every line of `lines` (P&T A5.1; any knot vector, clamped or
/// not). `k` is the span (`knots[k] <= u < knots[k + 1]`), `s` the multiplicity of `u`.
fn insert(
    p: usize,
    knots: &[f64],
    lines: &[Vec<H>],
    u: f64,
    k: usize,
    s: usize,
    r: usize,
) -> (Vec<f64>, Vec<Vec<H>>) {
    let mut uq = Vec::with_capacity(knots.len() + r);
    uq.extend_from_slice(&knots[..=k]);
    uq.extend(std::iter::repeat_n(u, r));
    uq.extend_from_slice(&knots[k + 1..]);
    let out = lines
        .iter()
        .map(|pw| {
            let np = pw.len() - 1;
            let mut qw = vec![[0.0; 4]; pw.len() + r];
            qw[..=k - p].copy_from_slice(&pw[..=k - p]);
            qw[k - s + r..=np + r].copy_from_slice(&pw[k - s..=np]);
            let mut rw: Vec<H> = pw[k - p..=k - s].to_vec();
            let mut l = k - p;
            for j in 1..=r {
                l = k - p + j;
                for i in 0..=p - j - s {
                    let alpha = (u - knots[l + i]) / (knots[i + k + 1] - knots[l + i]);
                    rw[i] = lerp(rw[i], rw[i + 1], alpha);
                }
                qw[l] = rw[0];
                qw[k + r - j - s] = rw[p - j - s];
            }
            if l + 1 < k - s {
                qw[l + 1..k - s].copy_from_slice(&rw[1..k - s - l]);
            }
            qw
        })
        .collect();
    (uq, out)
}

/// Unwraps periodic B-spline lines (OCCT form: `knots` with equal end multiplicities
/// `m0 <= p`, `N = m0 + Σ interior` poles per line) into clamped flat knots on
/// `[k_0, k_last]` and the matching lines.
///
/// OCCT's periodic flat knots start `p + 1 − m0` knots before `k_0`, and the span after `k_0`
/// uses poles `0..=p` (`BSplCLib::KnotSequence`, `BSplCLib::PoleIndex`): the floating form has
/// poles `Q_i = P_{i mod N}`, `i < N + p`. Clamping inserts `k_0` and `k_last` up to
/// multiplicity `p` and cuts the curve there.
fn unwrap_periodic(
    p: usize,
    knots: &[f64],
    mults: &[usize],
    lines: &[Vec<H>],
) -> Result<(Vec<f64>, Vec<Vec<H>>), NurbsError> {
    let n = knots.len();
    if n < 2 || mults.len() != n {
        return Err(NurbsError::InvalidKnots(
            "periodic knots and multiplicities differ in length",
        ));
    }
    if p == 0 || p > amg_nurbs::MAX_DEGREE {
        return Err(NurbsError::Degree(p));
    }
    if knots.windows(2).any(|w| w[1] <= w[0]) || knots.iter().any(|k| !k.is_finite()) {
        return Err(NurbsError::InvalidKnots(
            "distinct knots must strictly increase",
        ));
    }
    let m0 = mults[0];
    if m0 == 0 || m0 > p || mults[n - 1] != m0 || mults[1..n - 1].iter().any(|&m| m == 0 || m > p) {
        return Err(NurbsError::InvalidKnots(
            "periodic multiplicities must be in 1..=degree with equal ends",
        ));
    }
    let count: usize = mults[..n - 1].iter().sum();
    if count < 2 || lines.iter().any(|l| l.len() != count) {
        return Err(NurbsError::CountMismatch {
            degree: p,
            poles: lines.first().map_or(0, Vec::len),
            knots: count,
        });
    }
    let period = knots[n - 1] - knots[0];
    let base: Vec<f64> = knots[..n - 1]
        .iter()
        .zip(&mults[..n - 1])
        .flat_map(|(&k, &m)| std::iter::repeat_n(k, m))
        .collect();
    let s0 = (p + 1 - m0) as isize;
    let total = count + 2 * p + 1;
    let nn = count as isize;
    let flat: Vec<f64> = (0..total as isize)
        .map(|j| {
            let q = j - s0;
            let wraps = q.div_euclid(nn);
            let k = base[q.rem_euclid(nn) as usize];
            // `k_0 + T` must be `k_last` exactly (the sum can round away from it).
            if wraps == 1 && k.to_bits() == knots[0].to_bits() {
                knots[n - 1]
            } else {
                k + wraps as f64 * period
            }
        })
        .collect();
    let floating: Vec<Vec<H>> = lines
        .iter()
        .map(|l| (0..count + p).map(|i| l[i % count]).collect())
        .collect();
    let (k0, kl) = (knots[0], knots[n - 1]);
    // A5.1 inserts up to multiplicity p; the curve then passes through a pole at the knot,
    // and the last copy is added by cutting there (as `NurbsCurve::split` does).
    let r = p - m0;
    let (flat, floating) = if r > 0 {
        // The run of `k_0` ends at flat index p.
        let (flat, floating) = insert(p, &flat, &floating, k0, p, m0, r);
        let k = flat.partition_point(|&t| t <= kl) - 1;
        insert(p, &flat, &floating, kl, k, m0, r)
    } else {
        (flat, floating)
    };
    // Cut at k_0: its run starts at f >= 1 (the floating form has knots before it).
    let f = flat.partition_point(|&t| t < k0);
    let mut knots_out = vec![k0];
    knots_out.extend_from_slice(&flat[f..]);
    let lines: Vec<&[H]> = floating.iter().map(|q| &q[f - 1..]).collect();
    // Cut at k_last.
    let l = knots_out.partition_point(|&t| t < kl);
    knots_out.truncate(l + p);
    knots_out.push(kl);
    let lines_out = lines.iter().map(|q| q[..l].to_vec()).collect();
    Ok((knots_out, lines_out))
}

fn homogeneous(poles: &[Point3], weights: Option<&[f64]>) -> Vec<H> {
    poles
        .iter()
        .enumerate()
        .map(|(i, q)| {
            let w = weights.map_or(1.0, |w| w[i]);
            if weights.is_some() {
                [q.x * w, q.y * w, q.z * w, w]
            } else {
                [q.x, q.y, q.z, 1.0]
            }
        })
        .collect()
}

fn split_homogeneous(h: &[H], rational: bool) -> (Vec<Point3>, Option<Vec<f64>>) {
    let poles = h
        .iter()
        .map(|q| {
            if rational {
                Point3::new(q[0] / q[3], q[1] / q[3], q[2] / q[3])
            } else {
                Point3::new(q[0], q[1], q[2])
            }
        })
        .collect();
    (poles, rational.then(|| h.iter().map(|q| q[3]).collect()))
}

/// Periodic data is unwrapped and clamped (P&T A5.1 knot insertion); exact up to rounding.
impl TryFrom<&BSplineCurveData> for NurbsCurve3 {
    type Error = NurbsError;
    fn try_from(c: &BSplineCurveData) -> Result<Self, NurbsError> {
        let p = usize::from(c.degree);
        let mults = usize_mults(&c.mults);
        if c.weights.as_ref().is_some_and(|w| w.len() != c.poles.len()) {
            return Err(NurbsError::Degenerate("weights and poles differ in length"));
        }
        if !c.periodic {
            let knots = KnotVector::from_knots_mults(&c.knots, &mults)?;
            return NurbsCurve3::new(p, knots, c.poles.clone(), c.weights.clone());
        }
        let rational = c.weights.is_some();
        let line = homogeneous(&c.poles, c.weights.as_deref());
        let (knots, lines) = unwrap_periodic(p, &c.knots, &mults, &[line])?;
        let (poles, weights) = split_homogeneous(&lines[0], rational);
        NurbsCurve3::new(p, KnotVector::new(knots)?, poles, weights)
    }
}

/// Periodic data is unwrapped and clamped per direction; exact up to rounding.
impl TryFrom<&BSplineSurfaceData> for NurbsSurface3 {
    type Error = NurbsError;
    fn try_from(s: &BSplineSurfaceData) -> Result<Self, NurbsError> {
        let (pu, pv) = (usize::from(s.u_degree), usize::from(s.v_degree));
        let (nu, nv) = (s.nu as usize, s.nv as usize);
        let poles = Grid::new(nu, nv, s.poles.clone())?;
        let weights = match &s.weights {
            Some(w) => Some(Grid::new(nu, nv, w.clone())?),
            None => None,
        };
        let (um, vm) = (usize_mults(&s.u_mults), usize_mults(&s.v_mults));
        if !s.u_periodic && !s.v_periodic {
            let ku = KnotVector::from_knots_mults(&s.u_knots, &um)?;
            let kv = KnotVector::from_knots_mults(&s.v_knots, &vm)?;
            return NurbsSurface3::new([pu, pv], [ku, kv], poles, weights);
        }
        let rational = weights.is_some();
        // Homogeneous grid, row-major (u outer).
        let mut h: Vec<H> = homogeneous(poles.as_slice(), weights.as_ref().map(Grid::as_slice));
        let (mut nu, mut nv) = (nu, nv);
        let flat = |knots: &[f64], mults: &[usize]| -> Vec<f64> {
            knots
                .iter()
                .zip(mults)
                .flat_map(|(&k, &m)| std::iter::repeat_n(k, m))
                .collect()
        };
        let ku = if s.u_periodic {
            // Lines along u: one per column j.
            let cols: Vec<Vec<H>> = (0..nv)
                .map(|j| (0..nu).map(|i| h[i * nv + j]).collect())
                .collect();
            let (k, cols) = unwrap_periodic(pu, &s.u_knots, &um, &cols)?;
            nu = cols[0].len();
            h = (0..nu)
                .flat_map(|i| cols.iter().map(move |c| c[i]))
                .collect();
            k
        } else {
            flat(&s.u_knots, &um)
        };
        let kv = if s.v_periodic {
            let rows: Vec<Vec<H>> = (0..nu).map(|i| h[i * nv..(i + 1) * nv].to_vec()).collect();
            let (k, rows) = unwrap_periodic(pv, &s.v_knots, &vm, &rows)?;
            nv = rows[0].len();
            h = rows.into_iter().flatten().collect();
            k
        } else {
            flat(&s.v_knots, &vm)
        };
        let (poles, weights) = split_homogeneous(&h, rational);
        let weights = match weights {
            Some(w) => Some(Grid::new(nu, nv, w)?),
            None => None,
        };
        NurbsSurface3::new(
            [pu, pv],
            [KnotVector::new(ku)?, KnotVector::new(kv)?],
            Grid::new(nu, nv, poles)?,
            weights,
        )
    }
}

// ---------------------------------------------------------------------------------------------
// Analytic and trimmed geometry → NURBS
// ---------------------------------------------------------------------------------------------

/// `(min, max)` of a parameter range, checked finite and non-empty.
fn range(a: f64, b: f64, what: &'static str) -> KResult<(f64, f64)> {
    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
    if !(lo.is_finite() && hi.is_finite() && hi > lo) {
        return Err(invalid(
            what,
            format!("empty or non-finite range [{a}, {b}]"),
        ));
    }
    Ok((lo, hi))
}

fn check_turn(lo: f64, hi: f64, what: &'static str) -> KResult<()> {
    if hi - lo > TAU + ANGLE_EPS {
        return Err(invalid(what, "the angular range exceeds a full turn"));
    }
    Ok(())
}

/// The piece of `c` on `[lo, hi]`, keeping `c` itself when that is its whole domain. Ends
/// within 1e-9 of the domain length snap onto it.
fn trim_curve(c: NurbsCurve3, lo: f64, hi: f64) -> KResult<NurbsCurve3> {
    let (a, b) = c.domain();
    let eps = 1e-9 * (b - a).max(1.0);
    if lo < a - eps || hi > b + eps {
        return Err(invalid(
            "curve",
            format!("range [{lo}, {hi}] is outside the B-spline domain [{a}, {b}]"),
        ));
    }
    let (lo, hi) = (lo.max(a), hi.min(b));
    if lo - a <= eps && b - hi <= eps {
        return Ok(c);
    }
    c.segment(lo, hi).map_err(nurbs_err("curve"))
}

/// A periodic B-spline's NURBS on `[lo, hi]`, which may lie anywhere on the periodic line and
/// cross the seam once.
fn periodic_piece(b: &BSplineCurveData, lo: f64, hi: f64) -> KResult<NurbsCurve3> {
    let c = NurbsCurve3::try_from(b).map_err(nurbs_err("curve"))?;
    let (k0, kl) = c.domain();
    let period = kl - k0;
    if hi - lo > period * (1.0 + 1e-12) {
        return Err(invalid("curve", "the range exceeds the period"));
    }
    let shift = ((lo - k0) / period).floor() * period;
    let (a, z) = (lo - shift, hi - shift);
    let eps = 1e-9 * period.max(1.0);
    let piece = if z <= kl + eps {
        trim_curve(c, a, z.min(kl))?
    } else {
        let first = trim_curve(c.clone(), a, kl)?;
        let second = trim_curve(c, k0, z - period)?;
        first
            .join(&second, &amg_core::Tolerance::DEFAULT)
            .map_err(nurbs_err("curve"))?
    };
    Ok(piece.reparametrized((lo, hi)))
}

/// Exact NURBS of a trimmed analytic or B-spline curve (`Curve3::Other` → Err). The result
/// runs from `c.t0` to `c.t1` on the domain `[min, max]` of the two (reversed when
/// `t1 < t0`).
///
/// # Errors
/// `InvalidInput` for `Curve3::Other`, an empty or non-finite range, an arc over more than a
/// full turn, a range outside a non-periodic B-spline's domain, or malformed B-spline data.
pub fn curve_seg_to_nurbs(c: &CurveSeg) -> KResult<NurbsCurve3> {
    let (lo, hi) = range(c.t0, c.t1, "curve")?;
    let curve = match &c.curve {
        Curve3::Line { origin, dir } => {
            let at = |t: f64| *origin + dir.get() * t;
            let k = KnotVector::new(vec![lo, lo, hi, hi]).map_err(nurbs_err("curve"))?;
            NurbsCurve3::new(1, k, vec![at(lo), at(hi)], None).map_err(nurbs_err("curve"))?
        }
        Curve3::Circle { frame, radius } => {
            check_turn(lo, hi, "curve")?;
            amg_nurbs::analytic::circle_arc(frame, *radius, lo, (hi - lo).min(TAU))
                .map_err(nurbs_err("curve"))?
                .reparametrized((lo, hi))
        }
        Curve3::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => {
            check_turn(lo, hi, "curve")?;
            amg_nurbs::analytic::ellipse_arc(
                frame,
                *major_radius,
                *minor_radius,
                lo,
                lo + (hi - lo).min(TAU),
            )
            .map_err(nurbs_err("curve"))?
            .reparametrized((lo, hi))
        }
        Curve3::BSpline(b) if b.periodic => periodic_piece(b, lo, hi)?,
        Curve3::BSpline(b) => {
            let n = NurbsCurve3::try_from(b).map_err(nurbs_err("curve"))?;
            trim_curve(n, lo, hi)?
        }
        Curve3::Other => {
            return Err(invalid(
                "curve",
                "an offset, hyperbola, parabola or other curve has no exact NURBS here",
            ));
        }
    };
    Ok(if c.t1 < c.t0 { curve.reversed() } else { curve })
}

/// `s` with its knots mapped affinely onto `uv`.
fn with_domain(s: &NurbsSurface3, uv: [(f64, f64); 2]) -> KResult<NurbsSurface3> {
    NurbsSurface3::new(
        [s.degree(UV::U), s.degree(UV::V)],
        [
            s.knots(UV::U).reparametrized(uv[0]),
            s.knots(UV::V).reparametrized(uv[1]),
        ],
        s.poles().clone(),
        s.weights().cloned(),
    )
    .map_err(nurbs_err("surface"))
}

/// The profile at angle `u` of a surface of revolution about `frame.normal`, revolved over
/// `[u0, u1]`, on the domain `uv`.
fn revolved(
    frame: &Plane,
    profile: &NurbsCurve3,
    (u0, u1): (f64, f64),
    v: (f64, f64),
) -> KResult<NurbsSurface3> {
    let axis = Line3 {
        point: frame.origin,
        dir: frame.normal,
    };
    let s = amg_nurbs::analytic::revolution(profile, &axis, (u1 - u0).min(TAU))
        .map_err(nurbs_err("surface"))?;
    with_domain(&s, [(u0, u1), v])
}

/// The radial direction at angle `u` and the meridian plane through it (`x` radial, `y` the
/// axis), for profiles of revolved surfaces.
fn meridian(frame: &Plane, u: f64) -> (Vec3, Plane) {
    let d = frame.x.get() * u.cos() + frame.y.get() * u.sin();
    let z = frame.normal.get();
    let plane = Plane {
        origin: frame.origin,
        x: Dir3::new_unchecked(d),
        y: frame.normal,
        normal: Dir3::new_unchecked(d.cross(z)),
    };
    (d, plane)
}

fn segment_line(a: Point3, b: Point3, (lo, hi): (f64, f64)) -> KResult<NurbsCurve3> {
    let k = KnotVector::new(vec![lo, lo, hi, hi]).map_err(nurbs_err("surface"))?;
    NurbsCurve3::new(1, k, vec![a, b], None).map_err(nurbs_err("surface"))
}

/// The NURBS surface `s` restricted to `uv` (within the domain; ends within 1e-9 of the
/// domain length snap onto it).
fn trim_surface(mut s: NurbsSurface3, uv: [(f64, f64); 2]) -> KResult<NurbsSurface3> {
    for (d, (lo, hi)) in [(UV::U, uv[0]), (UV::V, uv[1])] {
        let (a, b) = s.domain()[usize::from(d == UV::V)];
        let eps = 1e-9 * (b - a).max(1.0);
        if lo < a - eps || hi > b + eps {
            return Err(invalid(
                "surface",
                format!("range [{lo}, {hi}] is outside the B-spline domain [{a}, {b}]"),
            ));
        }
        if lo - a > eps {
            s = s.split(d, lo).map_err(nurbs_err("surface"))?.1;
        }
        if b - hi > eps {
            s = s.split(d, hi).map_err(nurbs_err("surface"))?.0;
        }
    }
    Ok(s)
}

/// A periodic direction's range shifted into the base period; `Err` if it crosses the seam.
fn into_period(lo: f64, hi: f64, (k0, kl): (f64, f64)) -> KResult<(f64, f64)> {
    let period = kl - k0;
    let shift = ((lo - k0) / period).floor() * period;
    let (a, b) = (lo - shift, hi - shift);
    if b > kl + 1e-9 * period.max(1.0) {
        return Err(invalid(
            "surface",
            "a range across the seam of a periodic B-spline surface is not supported",
        ));
    }
    Ok((a, b.min(kl)))
}

/// Exact NURBS of an analytic/B-spline surface over a UV box (`Surface::Other` → Err).
///
/// The result's domain is `uv`, its `u` and `v` run as the surface's, so `S_u × S_v` points
/// the same way. Planes keep the analytic parametrisation; revolved surfaces (cylinder, cone,
/// sphere, torus) are rational quadratic in `u` (and in `v` for spheres and tori), exact at
/// 90° steps.
///
/// # Errors
/// `InvalidInput` for `Surface::Other`, an empty or non-finite range, an angular range over a
/// full turn, a sphere latitude outside `[-π/2, π/2]`, a range outside a B-spline's domain (or
/// across a periodic B-spline surface's seam), or malformed data.
pub fn surface_to_nurbs(s: &Surface, uv: [(f64, f64); 2]) -> KResult<NurbsSurface3> {
    let (u0, u1) = range(uv[0].0, uv[0].1, "surface")?;
    let (v0, v1) = range(uv[1].0, uv[1].1, "surface")?;
    let u = (u0, u1);
    let v = (v0, v1);
    match s {
        Surface::Plane(p) => {
            let patch = amg_nurbs::analytic::plane_patch(p, u, v);
            with_domain(&patch, [u, v])
        }
        Surface::Cylinder { frame, radius } => {
            check_turn(u0, u1, "surface")?;
            let (d, _) = meridian(frame, u0);
            let z = frame.normal.get();
            let at = |h: f64| frame.origin + d * *radius + z * h;
            revolved(frame, &segment_line(at(v0), at(v1), v)?, u, v)
        }
        Surface::Cone {
            frame,
            ref_radius,
            half_angle,
        } => {
            check_turn(u0, u1, "surface")?;
            let (d, _) = meridian(frame, u0);
            let z = frame.normal.get();
            let (sin, cos) = half_angle.sin_cos();
            let at = |h: f64| frame.origin + d * (*ref_radius + h * sin) + z * (h * cos);
            revolved(frame, &segment_line(at(v0), at(v1), v)?, u, v)
        }
        Surface::Sphere { frame, radius } => {
            check_turn(u0, u1, "surface")?;
            if v0 < -FRAC_PI_2 - ANGLE_EPS || v1 > FRAC_PI_2 + ANGLE_EPS {
                return Err(invalid("surface", "sphere latitude outside [-π/2, π/2]"));
            }
            let (_, plane) = meridian(frame, u0);
            let profile = amg_nurbs::analytic::circle_arc(&plane, *radius, v0, v1 - v0)
                .map_err(nurbs_err("surface"))?
                .reparametrized(v);
            revolved(frame, &profile, u, v)
        }
        Surface::Torus {
            frame,
            major,
            minor,
        } => {
            check_turn(u0, u1, "surface")?;
            check_turn(v0, v1, "surface")?;
            let (d, mut plane) = meridian(frame, u0);
            plane.origin = frame.origin + d * *major;
            let profile = amg_nurbs::analytic::circle_arc(&plane, *minor, v0, (v1 - v0).min(TAU))
                .map_err(nurbs_err("surface"))?
                .reparametrized(v);
            revolved(frame, &profile, u, v)
        }
        Surface::BSpline(b) => {
            let n = NurbsSurface3::try_from(b).map_err(nurbs_err("surface"))?;
            let dom = n.domain();
            let u = if b.u_periodic {
                into_period(u0, u1, dom[0])?
            } else {
                u
            };
            let v = if b.v_periodic {
                into_period(v0, v1, dom[1])?
            } else {
                v
            };
            let t = trim_surface(n, [u, v])?;
            if b.u_periodic || b.v_periodic {
                with_domain(&t, [(u0, u1), (v0, v1)])
            } else {
                Ok(t)
            }
        }
        Surface::Other { .. } => Err(invalid(
            "surface",
            "a revolution, extrusion, offset or other surface has no exact NURBS here",
        )),
    }
}

// ---------------------------------------------------------------------------------------------
// Shapes
// ---------------------------------------------------------------------------------------------

/// Edge `e` of `s` as a NURBS over the edge's parameter range, in the curve's direction.
///
/// # Errors
/// `InvalidInput` for an edge index out of range, a degenerated edge (no 3D curve) or a curve
/// [`curve_seg_to_nurbs`] rejects.
pub fn edge_to_nurbs(s: &Shape, e: EdgeId) -> KResult<NurbsCurve3> {
    if e.0 >= s.edges().len() as u32 {
        return Err(invalid("edge", format!("edge {} is out of range", e.0)));
    }
    let (curve, t0, t1) = s
        .curve(e)
        .ok_or_else(|| invalid("edge", "a degenerated edge has no 3D curve"))?;
    curve_seg_to_nurbs(&CurveSeg {
        curve,
        t0,
        t1,
        tag: crate::geom::Tag(0),
    })
}

/// Edges of a wire shape in wire order, each oriented along the wire (S02 request R16). An
/// edge shape gives its single edge.
///
/// # Errors
/// `InvalidInput` when `wire` is not a wire or an edge, or for an edge
/// [`edge_to_nurbs`] rejects; `Occt` when the wire is not a simple chain.
pub fn wire_to_chain(wire: &Shape) -> KResult<Vec<NurbsCurve3>> {
    if !matches!(wire.kind(), ShapeKind::Wire | ShapeKind::Edge) {
        return Err(invalid(
            "wire",
            format!("expected a wire or an edge, got {:?}", wire.kind()),
        ));
    }
    wire.occ()
        .wire_order()?
        .into_iter()
        .map(|(e, reversed)| {
            let c = edge_to_nurbs(wire, EdgeId(e))?;
            Ok(if reversed { c.reversed() } else { c })
        })
        .collect()
}

/// `k.make_face(&Surface::BSpline(s.into()), bounds, tol)`.
///
/// # Errors
/// As [`BrepKernel::make_face`].
pub fn nurbs_face(
    k: &dyn BrepKernel,
    s: &NurbsSurface3,
    bounds: &FaceBounds,
    tol: f64,
) -> KResult<FaceBuild> {
    k.make_face(&Surface::BSpline(s.into()), bounds, tol)
}

// ---------------------------------------------------------------------------------------------
// OCCT oracles
// ---------------------------------------------------------------------------------------------

/// OCCT as an oracle for differential tests (S20 KH-16, KH-15), on plain data. Each call builds
/// fresh `Geom_*` objects, so all are thread-safe (category R). Without the `occt` feature
/// every call returns [`KernelError::BackendUnavailable`].
pub mod oracle {
    use amg_core::{Point3, Vec3};
    use amg_nurbs::{NurbsCurve3, NurbsSurface3};

    use crate::backend::nurbs;
    use crate::error::{KResult, KernelError};
    use crate::geom::{BSplineCurveData, BSplineSurfaceData};

    /// `Geom_BSplineSurface::D2` at one `(u, v)`.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct SurfaceD2 {
        /// The point.
        pub p: Point3,
        /// `∂S/∂u`.
        pub du: Vec3,
        /// `∂S/∂v`.
        pub dv: Vec3,
        /// `∂²S/∂u²`.
        pub duu: Vec3,
        /// `∂²S/∂v²`.
        pub dvv: Vec3,
        /// `∂²S/∂u∂v`.
        pub duv: Vec3,
    }

    /// What one `GeomFill_Gordon` run gave (OCCT ≥ 8.0.0p1).
    #[derive(Clone, Debug)]
    pub struct GordonOracle {
        /// `GeomFill_Gordon::ResultStatus` as a number: 1 = `Done`.
        pub status: u8,
        /// Whether OCCT fell back to an approximation (never in the exact mode used here).
        pub approximate: bool,
        /// Largest profile/guide contact gap OCCT saw, mm.
        pub max_contact_gap: f64,
        /// Largest distance of the result from a profile, mm (OCCT's own check).
        pub max_profile_deviation: f64,
        /// Largest distance of the result from a guide, mm (OCCT's own check).
        pub max_guide_deviation: f64,
        /// The surface when `status == 1`.
        pub surface: Option<NurbsSurface3>,
    }

    impl GordonOracle {
        /// `GeomFill_Gordon::ResultStatus::Done`.
        pub const DONE: u8 = 1;
    }

    /// `c` built as a `Geom_BSplineCurve` and read back.
    ///
    /// # Errors
    /// `Occt` when OCCT rejects the data; `BackendUnavailable` without OCCT.
    pub fn round_trip_curve(c: &BSplineCurveData) -> KResult<BSplineCurveData> {
        nurbs::curve_round_trip(c)
    }

    /// `s` built as a `Geom_BSplineSurface` and read back.
    ///
    /// # Errors
    /// As [`round_trip_curve`].
    pub fn round_trip_surface(s: &BSplineSurfaceData) -> KResult<BSplineSurfaceData> {
        nurbs::surface_round_trip(s)
    }

    /// `c` after OCCT's `SetNotPeriodic`.
    ///
    /// # Errors
    /// As [`round_trip_curve`].
    pub fn curve_not_periodic(c: &BSplineCurveData) -> KResult<BSplineCurveData> {
        nurbs::curve_not_periodic(c)
    }

    /// `s` after OCCT's `SetUNotPeriodic` and `SetVNotPeriodic`.
    ///
    /// # Errors
    /// As [`round_trip_curve`].
    pub fn surface_not_periodic(s: &BSplineSurfaceData) -> KResult<BSplineSurfaceData> {
        nurbs::surface_not_periodic(s)
    }

    /// `Geom_BSplineCurve::D2` (point, first and second derivative) at each of `ts`.
    ///
    /// # Errors
    /// As [`round_trip_curve`].
    pub fn curve_d2(c: &BSplineCurveData, ts: &[f64]) -> KResult<Vec<(Point3, Vec3, Vec3)>> {
        nurbs::curve_d2(c, ts)
    }

    /// `Geom_BSplineSurface::D2` at each of `uvs`.
    ///
    /// # Errors
    /// As [`round_trip_curve`].
    pub fn surface_d2(s: &BSplineSurfaceData, uvs: &[(f64, f64)]) -> KResult<Vec<SurfaceD2>> {
        Ok(nurbs::surface_d2(s, uvs)?
            .into_iter()
            .map(|(p, [du, dv, duu, dvv, duv])| SurfaceD2 {
                p,
                du,
                dv,
                duu,
                dvv,
                duv,
            })
            .collect())
    }

    /// OCCT's `GeomFill_Gordon` (exact mode, serial) through `profiles` and `guides` with
    /// intersection tolerance `tol` mm: the KH-15 cross-check.
    ///
    /// # Errors
    /// `InvalidInput` when OCCT's result cannot be read as a clamped NURBS; `Occt` when OCCT
    /// raises; `BackendUnavailable` without OCCT. A Gordon failure is a status, not an error.
    pub fn gordon(
        profiles: &[NurbsCurve3],
        guides: &[NurbsCurve3],
        tol: f64,
    ) -> KResult<GordonOracle> {
        let p: Vec<BSplineCurveData> = profiles.iter().map(BSplineCurveData::from).collect();
        let g: Vec<BSplineCurveData> = guides.iter().map(BSplineCurveData::from).collect();
        let raw = nurbs::gordon(&p, &g, tol)?;
        let surface = match &raw.surface {
            Some(s) => Some(
                NurbsSurface3::try_from(s).map_err(|e| KernelError::InvalidInput {
                    what: "gordon",
                    reason: e.to_string(),
                    offending: Vec::new(),
                })?,
            ),
            None => None,
        };
        Ok(GordonOracle {
            status: raw.status,
            approximate: raw.approximate,
            max_contact_gap: raw.max_contact_gap,
            max_profile_deviation: raw.max_profile_deviation,
            max_guide_deviation: raw.max_guide_deviation,
            surface,
        })
    }
}
