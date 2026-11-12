//! Geometry extraction for sketches and drawings (BREP-17, S02 §3): plane sections and
//! orthogonal projection of edges, as [`CurveSeg`]s.
//!
//! Orthogonal projection onto a plane is an affine map, so it is done exactly here in plain
//! Rust: a line stays a line, a circle or ellipse becomes an ellipse (a circle when the plane is
//! parallel to it, a segment when it is edge-on), and a B-spline's poles are projected with their
//! weights kept. Only curves with no exact data form (offset curves, parabolas, curves on
//! surfaces) go to OCCT first, which recognises lines, circles and ellipses in B-splines and
//! approximates the rest. Plane sections are OCCT's `BRepAlgoAPI_Section`; this module chains
//! the section edges into loops and merges pieces of one carrier curve.

use amg_core::{Dir3, Plane, Point3, Tolerance, Vec3};

use crate::builders::eval::point3;
use crate::error::{KResult, KernelError};
use crate::geom::{BSplineCurveData, Curve3, CurveSeg, Tag};
use crate::shape::{EdgeId, Shape, SubKind, SubShape};

/// The [`CurveSeg::tag`] of a section segment whose face OCCT did not name.
pub(crate) const NO_FACE_TAG: Tag = Tag(u64::MAX);

/// Carrier curves closer than this (relative to their size) are the same curve when merging
/// section pieces.
const SAME_CARRIER: f64 = 1e-9;

/// One section edge as the backend returns it.
#[derive(Clone, Debug)]
pub(crate) struct RawSeg {
    pub(crate) curve: Curve3,
    pub(crate) t0: f64,
    pub(crate) t1: f64,
    /// The input face the edge lies on.
    pub(crate) face: Option<u32>,
}

fn invalid(what: &'static str, reason: impl Into<String>) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason: reason.into(),
        offending: Vec::new(),
    }
}

/// Rejects planes with non-finite values, a non-unit normal or an `x` axis along the normal.
pub(crate) fn check_plane(plane: &Plane) -> KResult<()> {
    let n = plane.normal.get();
    let x = plane.x.get();
    if !(plane.origin.is_finite() && n.is_finite() && x.is_finite()) {
        return Err(invalid("plane", "the plane has non-finite values"));
    }
    if (n.length() - 1.0).abs() > 1e-9 {
        return Err(invalid("plane", "the plane normal is not a unit vector"));
    }
    if x.cross(n).length() < 1e-9 {
        return Err(invalid(
            "plane",
            "the plane's x axis is parallel to its normal",
        ));
    }
    Ok(())
}

fn check_tolerance(tol: &Tolerance) -> KResult<()> {
    if tol.linear > 0.0 && tol.linear.is_finite() {
        Ok(())
    } else {
        Err(invalid(
            "tol",
            "the linear tolerance must be positive and finite",
        ))
    }
}

/// The distance within which segment ends are joined: the model tolerance or the largest
/// sub-shape tolerance of `s`, whichever is larger, with a safety factor.
fn link_tolerance(s: &Shape, tol: &Tolerance) -> f64 {
    10.0 * tol.linear.max(s.max_tolerance())
}

// ---- projection -------------------------------------------------------------------------------

/// Orthogonal projection onto a plane.
#[derive(Clone, Copy, Debug)]
struct Projector {
    origin: Point3,
    normal: Vec3,
}

impl Projector {
    fn new(plane: &Plane) -> Projector {
        Projector {
            origin: plane.origin,
            normal: plane.normal.get(),
        }
    }

    fn point(&self, p: Point3) -> Point3 {
        p - self.normal * (p - self.origin).dot(self.normal)
    }

    fn vector(&self, v: Vec3) -> Vec3 {
        v - self.normal * v.dot(self.normal)
    }
}

/// A trimmed curve of a projection result.
type Piece = (Curve3, f64, f64);

/// The image of `curve` on `[t0, t1]` under orthogonal projection onto `plane`, as exact curves.
///
/// Lines, circles and ellipses come out as lines, circles or ellipses; an ellipse image thinner
/// than `tol` becomes line segments (two or more when it folds back on itself); a part that
/// collapses to a point is dropped. B-splines keep their knots and weights; one whose image lies
/// on a line and runs one way becomes a line. Circle and ellipse frames get `normal =
/// plane.normal`, so `t1 > t0` runs counter-clockwise seen from the normal. `Curve3::Other` gives
/// nothing (the caller converts it first).
pub(crate) fn project_curve(
    curve: &Curve3,
    t0: f64,
    t1: f64,
    plane: &Plane,
    tol: f64,
) -> Vec<Piece> {
    let proj = Projector::new(plane);
    match curve {
        Curve3::Line { origin, dir } => {
            let d = proj.vector(dir.get());
            let scale = d.length();
            if scale * (t1 - t0).abs() <= tol {
                return Vec::new();
            }
            let line = Curve3::Line {
                origin: proj.point(*origin),
                dir: Dir3::new_unchecked(d / scale),
            };
            vec![(line, t0 * scale, t1 * scale)]
        }
        Curve3::Circle { frame, radius } => conic(
            &proj,
            plane,
            frame.origin,
            frame.x.get() * *radius,
            frame.y.get() * *radius,
            t0,
            t1,
            tol,
        ),
        Curve3::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => conic(
            &proj,
            plane,
            frame.origin,
            frame.x.get() * *major_radius,
            frame.y.get() * *minor_radius,
            t0,
            t1,
            tol,
        ),
        Curve3::BSpline(b) => project_bspline(&proj, b, t0, t1, tol),
        Curve3::Other => Vec::new(),
    }
}

/// The image of `c + A cos t + B sin t` for `t ∈ [t0, t1]`. The images of `A` and `B` are
/// conjugate semi-diameters of the image ellipse; its axes are at the eccentric angle `t*` where
/// `A' cos t + B' sin t` is longest.
#[allow(
    clippy::too_many_arguments,
    reason = "the conic's parts, unpacked by the caller"
)]
fn conic(
    proj: &Projector,
    plane: &Plane,
    center: Point3,
    a: Vec3,
    b: Vec3,
    t0: f64,
    t1: f64,
    tol: f64,
) -> Vec<Piece> {
    let c = proj.point(center);
    let (a, b) = (proj.vector(a), proj.vector(b));
    let (aa, bb, ab) = (a.dot(a), b.dot(b), a.dot(b));
    // Already principal (a circle seen face-on, or the ellipse's own axes): keep `x`.
    let t_star = if (aa - bb).abs() + 2.0 * ab.abs() <= 1e-12 * (aa + bb) {
        0.0
    } else {
        0.5 * (2.0 * ab).atan2(aa - bb)
    };
    let (sin, cos) = t_star.sin_cos();
    let u = a * cos + b * sin;
    let v = b * cos - a * sin;
    let (major, minor) = (u.length(), v.length());
    if major <= tol {
        return Vec::new();
    }
    let x = u / major;
    if minor <= tol {
        return folded_segment(c, x, major, t_star, t0, t1, tol);
    }
    let n = plane.normal.get();
    let y = n.cross(x);
    let sign = if v.dot(y) >= 0.0 { 1.0 } else { -1.0 };
    let frame = Plane {
        origin: c,
        x: Dir3::new_unchecked(x),
        y: Dir3::new_unchecked(y),
        normal: plane.normal,
    };
    let (s0, s1) = (sign * (t0 - t_star), sign * (t1 - t_star));
    let curve = if major - minor <= 1e-9 * major.max(1.0) {
        Curve3::Circle {
            frame,
            radius: 0.5 * (major + minor),
        }
    } else {
        Curve3::Ellipse {
            frame,
            major_radius: major,
            minor_radius: minor,
        }
    };
    vec![(curve, s0, s1)]
}

/// An edge-on conic: `c + x · major cos(t − t*)`, split where it turns back (`t − t* = kπ`).
fn folded_segment(
    c: Point3,
    x: Vec3,
    major: f64,
    t_star: f64,
    t0: f64,
    t1: f64,
    tol: f64,
) -> Vec<Piece> {
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    let pi = std::f64::consts::PI;
    let mut cuts = vec![t0];
    let mut k = ((lo - t_star) / pi).floor() + 1.0;
    let mut inner = Vec::new();
    while t_star + k * pi < hi {
        inner.push(t_star + k * pi);
        k += 1.0;
    }
    if t1 < t0 {
        inner.reverse();
    }
    cuts.extend(inner);
    cuts.push(t1);
    let line = Curve3::Line {
        origin: c,
        dir: Dir3::new_unchecked(x),
    };
    cuts.windows(2)
        .filter_map(|w| {
            let (sa, sb) = (major * (w[0] - t_star).cos(), major * (w[1] - t_star).cos());
            ((sb - sa).abs() > tol).then(|| (line.clone(), sa, sb))
        })
        .collect()
}

fn project_bspline(
    proj: &Projector,
    b: &BSplineCurveData,
    t0: f64,
    t1: f64,
    tol: f64,
) -> Vec<Piece> {
    let poles: Vec<Point3> = b.poles.iter().map(|p| proj.point(*p)).collect();
    let image = BSplineCurveData { poles, ..b.clone() };
    let curve = Curve3::BSpline(image);
    let (Some(p0), Some(p1)) = (point3(&curve, t0), point3(&curve, t1)) else {
        // Periodic: no evaluation here, keep the exact image.
        return vec![(curve, t0, t1)];
    };
    let Curve3::BSpline(image) = &curve else {
        unreachable!("built above")
    };
    let first = image.poles[0];
    let spread = image
        .poles
        .iter()
        .map(|p| (*p - first).length())
        .fold(0.0, f64::max);
    if spread <= tol {
        return Vec::new();
    }
    if let Some(line) = straight(&image.poles, p0, p1, tol) {
        return vec![line];
    }
    vec![(curve, t0, t1)]
}

/// A line through `p0` and `p1` when every pole is within `tol` of it and the poles run one way
/// along it (so the curve does too: B-splines are variation diminishing).
fn straight(poles: &[Point3], p0: Point3, p1: Point3, tol: f64) -> Option<Piece> {
    let chord = p1 - p0;
    let len = chord.length();
    if len <= tol {
        return None;
    }
    let dir = chord / len;
    let mut last = f64::NEG_INFINITY;
    let mut forward = true;
    let mut backward = true;
    for p in poles {
        let d = *p - p0;
        let along = d.dot(dir);
        if (d - dir * along).length() > tol {
            return None;
        }
        if last.is_finite() {
            forward &= along >= last - tol;
            backward &= along <= last + tol;
        }
        last = along;
    }
    (forward || backward).then(|| {
        let line = Curve3::Line {
            origin: p0,
            dir: Dir3::new_unchecked(dir),
        };
        (line, 0.0, len)
    })
}

// ---- segment ends, chaining, merging ----------------------------------------------------------

fn start(s: &CurveSeg) -> Option<Point3> {
    point3(&s.curve, s.t0)
}

fn end(s: &CurveSeg) -> Option<Point3> {
    point3(&s.curve, s.t1)
}

fn near(a: Option<Point3>, b: Option<Point3>, tol: f64) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => (a - b).length() <= tol,
        _ => false,
    }
}

fn reverse(s: &mut CurveSeg) {
    std::mem::swap(&mut s.t0, &mut s.t1);
}

/// Flips segments so that consecutive ones sharing an end point run head to tail. The first is
/// flipped when only its start touches the second.
pub(crate) fn orient_in_sequence(segs: &mut [CurveSeg], tol: f64) {
    if segs.len() < 2 {
        return;
    }
    let second = (start(&segs[1]), end(&segs[1]));
    let touches = |p| near(p, second.0, tol) || near(p, second.1, tol);
    if !touches(end(&segs[0])) && touches(start(&segs[0])) {
        reverse(&mut segs[0]);
    }
    for i in 1..segs.len() {
        let prev = end(&segs[i - 1]);
        if !near(prev, start(&segs[i]), tol) && near(prev, end(&segs[i]), tol) {
            reverse(&mut segs[i]);
        }
    }
}

/// Orders `segs` into chains: each chain runs head to tail, closed chains end where they start.
/// Chains come in the order of their first segment; a chain grows at its end first, then at its
/// start.
pub(crate) fn chain(segs: Vec<CurveSeg>, tol: f64) -> Vec<Vec<CurveSeg>> {
    let ends: Vec<(Option<Point3>, Option<Point3>)> =
        segs.iter().map(|s| (start(s), end(s))).collect();
    let mut used = vec![false; segs.len()];
    let mut chains = Vec::new();
    for first in 0..segs.len() {
        if used[first] {
            continue;
        }
        used[first] = true;
        let mut chain = std::collections::VecDeque::from([(first, false)]);
        let tail = |c: &std::collections::VecDeque<(usize, bool)>| {
            let (i, flipped) = c[c.len() - 1];
            if flipped { ends[i].0 } else { ends[i].1 }
        };
        let head = |c: &std::collections::VecDeque<(usize, bool)>| {
            let (i, flipped) = c[0];
            if flipped { ends[i].1 } else { ends[i].0 }
        };
        loop {
            if near(tail(&chain), head(&chain), tol) && chain.len() > 1 {
                break;
            }
            let at = tail(&chain);
            let next = (0..segs.len()).filter(|j| !used[*j]).find_map(|j| {
                if near(at, ends[j].0, tol) {
                    Some((j, false))
                } else if near(at, ends[j].1, tol) {
                    Some((j, true))
                } else {
                    None
                }
            });
            let Some((j, flipped)) = next else { break };
            used[j] = true;
            chain.push_back((j, flipped));
        }
        if !(chain.len() > 1 && near(tail(&chain), head(&chain), tol)) {
            loop {
                let at = head(&chain);
                let prev = (0..segs.len()).filter(|j| !used[*j]).find_map(|j| {
                    if near(at, ends[j].1, tol) {
                        Some((j, false))
                    } else if near(at, ends[j].0, tol) {
                        Some((j, true))
                    } else {
                        None
                    }
                });
                let Some((j, flipped)) = prev else { break };
                used[j] = true;
                chain.push_front((j, flipped));
            }
        }
        chains.push(
            chain
                .into_iter()
                .map(|(i, flipped)| {
                    let mut s = segs[i].clone();
                    if flipped {
                        reverse(&mut s);
                    }
                    s
                })
                .collect(),
        );
    }
    chains
}

fn close_points(a: Point3, b: Point3, scale: f64) -> bool {
    (a - b).length() <= SAME_CARRIER * scale.max(1.0)
}

fn close_dirs(a: Dir3, b: Dir3) -> bool {
    (a.get() - b.get()).length() <= SAME_CARRIER
}

fn same_frame(a: &Plane, b: &Plane, scale: f64) -> bool {
    close_points(a.origin, b.origin, scale)
        && close_dirs(a.x, b.x)
        && close_dirs(a.normal, b.normal)
}

/// `b` continuing `a` on the same carrier, as `a` extended to `b`'s end.
fn merged(a: &CurveSeg, b: &CurveSeg) -> Option<CurveSeg> {
    if a.tag != b.tag {
        return None;
    }
    let tau = std::f64::consts::TAU;
    let (periodic, scale) = match (&a.curve, &b.curve) {
        (
            Curve3::Line {
                origin: oa,
                dir: da,
            },
            Curve3::Line {
                origin: ob,
                dir: db,
            },
        ) => {
            if !(close_points(*oa, *ob, a.t1.abs()) && close_dirs(*da, *db)) {
                return None;
            }
            (false, a.t1.abs().max(1.0))
        }
        (
            Curve3::Circle {
                frame: fa,
                radius: ra,
            },
            Curve3::Circle {
                frame: fb,
                radius: rb,
            },
        ) => {
            if !(same_frame(fa, fb, *ra) && (ra - rb).abs() <= SAME_CARRIER * ra.max(1.0)) {
                return None;
            }
            (true, 1.0)
        }
        (
            Curve3::Ellipse {
                frame: fa,
                major_radius: ma,
                minor_radius: na,
            },
            Curve3::Ellipse {
                frame: fb,
                major_radius: mb,
                minor_radius: nb,
            },
        ) => {
            let s = ma.max(1.0);
            if !(same_frame(fa, fb, *ma)
                && (ma - mb).abs() <= SAME_CARRIER * s
                && (na - nb).abs() <= SAME_CARRIER * s)
            {
                return None;
            }
            (true, 1.0)
        }
        _ => return None,
    };
    let forward_a = a.t1 > a.t0;
    let forward_b = b.t1 > b.t0;
    if forward_a != forward_b {
        return None;
    }
    let mut gap = b.t0 - a.t1;
    if periodic {
        gap -= tau * (gap / tau).round();
    }
    if gap.abs() > SAME_CARRIER * scale {
        return None;
    }
    let t1 = a.t1 + (b.t1 - b.t0);
    if periodic && (t1 - a.t0).abs() > tau + 1e-9 {
        return None;
    }
    Some(CurveSeg { t1, ..a.clone() })
}

/// Merges neighbours of a chain that continue one carrier curve (and lie on one face); for a
/// closed chain also its last and first segment.
pub(crate) fn merge_chain(chain: Vec<CurveSeg>, closed: bool) -> Vec<CurveSeg> {
    let mut out: Vec<CurveSeg> = Vec::with_capacity(chain.len());
    for seg in chain {
        match out.last_mut() {
            Some(last) => match merged(last, &seg) {
                Some(m) => *last = m,
                None => out.push(seg),
            },
            None => out.push(seg),
        }
    }
    if closed
        && out.len() > 1
        && let Some(m) = merged(&out[out.len() - 1], &out[0])
    {
        out[0] = m;
        out.pop();
    }
    out
}

// ---- the queries ------------------------------------------------------------------------------

/// [`crate::BrepKernel::plane_section`].
pub(crate) fn plane_section(s: &Shape, plane: &Plane, tol: &Tolerance) -> KResult<Vec<CurveSeg>> {
    check_plane(plane)?;
    check_tolerance(tol)?;
    let raw = s.occ().plane_section(plane, tol.linear)?;
    let segs: Vec<CurveSeg> = raw
        .into_iter()
        .map(|r| CurveSeg {
            curve: r.curve,
            t0: r.t0,
            t1: r.t1,
            tag: r.face.map_or(NO_FACE_TAG, |f| Tag(u64::from(f))),
        })
        .collect();
    let link = link_tolerance(s, tol);
    Ok(chain(segs, link)
        .into_iter()
        .flat_map(|c| {
            let closed = near(end(&c[c.len() - 1]), start(&c[0]), link);
            merge_chain(c, closed)
        })
        .collect())
}

/// [`crate::BrepKernel::project_edges`].
pub(crate) fn project_edges(
    s: &Shape,
    edges: &[EdgeId],
    plane: &Plane,
    tol: &Tolerance,
) -> KResult<Vec<CurveSeg>> {
    check_plane(plane)?;
    check_tolerance(tol)?;
    let n = s.count(SubKind::Edge);
    if let Some(bad) = edges.iter().find(|e| e.0 >= n) {
        return Err(KernelError::InvalidInput {
            what: "edges",
            reason: format!("{bad:?} is out of range: the shape has {n} edges"),
            offending: Vec::new(),
        });
    }
    let mut out = Vec::new();
    for &e in edges {
        let curve = match s.curve(e) {
            Some(
                c @ (Curve3::Line { .. } | Curve3::Circle { .. } | Curve3::Ellipse { .. }, _, _),
            ) => Some(c),
            // B-splines may be lines, circles or ellipses; other curves need a B-spline form.
            _ => s
                .occ()
                .curve_recognised(e.0, tol.linear)
                .map_err(|error| with_edge(error, e))?,
        };
        let Some((curve, t0, t1)) = curve else {
            continue; // degenerated
        };
        out.extend(
            project_curve(&curve, t0, t1, plane, tol.linear)
                .into_iter()
                .map(|(curve, t0, t1)| CurveSeg {
                    curve,
                    t0,
                    t1,
                    tag: Tag(u64::from(e.0)),
                }),
        );
    }
    orient_in_sequence(&mut out, link_tolerance(s, tol));
    Ok(out)
}

fn with_edge(error: KernelError, e: EdgeId) -> KernelError {
    match error {
        KernelError::Occt { kind, message } => KernelError::OpFailed {
            op: "project_edges",
            message: format!("{kind}: {message}"),
            offending: vec![SubShape {
                kind: SubKind::Edge,
                index: e.0,
            }],
        },
        other => other,
    }
}

#[cfg(test)]
mod tests;
