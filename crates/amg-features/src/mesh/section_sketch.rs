//! MESH-14: mesh section → sketch curves.
//!
//! [`section_to_sketch_curves`] projects the polylines of `amg_mesh::section` onto the plane's
//! frame and replaces each with a short chain of lines, circular arcs, circles and cubic
//! splines that stays within `tolerance` of the polyline:
//!
//! 1. a closed loop that is one circle becomes a [`Curve2::Circle`];
//! 2. otherwise a run of points is tried as one line, one arc, one spline (in that order); the
//!    first that is within `tolerance` of the run in both directions wins (every run vertex is
//!    within `tolerance` of the curve, every point of the curve is within `tolerance` of the
//!    run);
//! 3. a run nothing fits is split at the vertex farthest from its chord (Douglas–Peucker) and
//!    both halves are fitted again;
//! 4. neighbouring lines (arcs) that one line (arc) also fits are merged.
//!
//! Pieces meet at polyline vertices exactly: lines end there, arcs are constrained to pass
//! through both ends of their run, splines interpolate them. So the chain closes without gaps
//! and [`section_to_sketch`] gives a sketch whose profiles (SK-06) are the section's regions.
//!
//! Arcs come back counter-clockwise (the sketch convention), so a clockwise run is an arc
//! with the run's end as its start. Splines are non-periodic cubic fit splines with natural
//! ends (SK-08 `fit_spline`).

use amg_core::{Plane, Point2, Point3, Vec2};
use amg_mesh::Polyline3;
use amg_sketch::{Arc2, BSpline2, Circle2, Curve2, Sketch, SketchError, geom::fit_spline};

use amg_core::Segment2;
use std::f64::consts::TAU;

/// Samples of a candidate curve checked per run segment.
const SAMPLES_PER_SEGMENT: usize = 6;
/// Points closer than this are one point, mm.
const DEDUP: f64 = 1e-9;
/// Splines are not tried through a run with a turn sharper than this at an interior vertex, rad.
const SPLINE_MAX_TURN: f64 = 0.7;
/// A closed loop starts at its sharpest vertex when that turn exceeds this, rad.
const CORNER_TURN: f64 = 0.3;
/// Most interpolation points of one spline; longer runs are split first.
const MAX_SPLINE_POINTS: usize = 400;
/// An arc whose radius exceeds this many chord lengths is a line.
const MAX_RADIUS_RATIO: f64 = 1e5;

/// Fits lines, arcs, circles and splines to the section polylines `polys` of a mesh cut by
/// `plane` (see the [module docs](self)).
///
/// The curves are in the plane's frame (`plane.x`, `plane.y`, origin `plane.origin`, the frame
/// a [`Sketch::new`] on the same plane uses), mm. Every returned curve is within `tolerance`
/// (mm, clamped to at least 1e-9) of the polyline piece it replaces. Polylines with fewer than
/// two distinct points give nothing; a closed polyline with fewer than three is treated as
/// open.
pub fn section_to_sketch_curves(polys: &[Polyline3], plane: &Plane, tolerance: f64) -> Vec<Curve2> {
    let tol = tolerance.max(1e-9);
    let mut out = Vec::new();
    for poly in polys {
        let (pts, closed) = project(poly, plane);
        out.extend(fit_polyline(&pts, closed, tol));
    }
    out
}

/// A sketch on `plane` holding `curves` (as produced by [`section_to_sketch_curves`]).
///
/// Splines become control-point splines. `Err` if the sketch rejects a curve (a degenerate arc);
/// ellipses are not produced by the fit and give `NotApplicable`.
pub fn curves_to_sketch(curves: &[Curve2], plane: &Plane) -> Result<Sketch, SketchError> {
    let mut sketch = Sketch::new(*plane);
    for c in curves {
        match c {
            Curve2::Line(s) => {
                sketch.add_line(s.a, s.b)?;
            }
            Curve2::Circle(c) => {
                sketch.add_circle(c.center, c.radius)?;
            }
            Curve2::Arc(a) => {
                let (_, t1) = c.param_range();
                sketch.add_arc(a.center, c.eval(0.0), c.eval(t1))?;
            }
            Curve2::BSpline(b) => {
                let weights = b.is_rational().then(|| b.weights());
                sketch.add_spline(b.degree(), b.poles(), weights, Some(b.knots().to_vec()))?;
            }
            Curve2::Ellipse(_) => {
                return Err(SketchError::NotApplicable(
                    "section fitting does not produce ellipses",
                ));
            }
        }
    }
    Ok(sketch)
}

/// The section polylines as a sketch on `plane`: [`section_to_sketch_curves`] then
/// [`curves_to_sketch`].
pub fn section_to_sketch(
    polys: &[Polyline3],
    plane: &Plane,
    tolerance: f64,
) -> Result<Sketch, SketchError> {
    curves_to_sketch(&section_to_sketch_curves(polys, plane, tolerance), plane)
}

/// The largest distance between `curves` and the polylines `polys` on `plane`, in either
/// direction: the farthest polyline vertex from every curve, and the farthest curve sample
/// (`SAMPLES_PER_SEGMENT` per polyline segment, at least 64 per curve) from every polyline.
/// `f64::INFINITY` if there are polylines but no curves. Brute force, for tests and
/// diagnostics.
pub fn section_deviation(curves: &[Curve2], polys: &[Polyline3], plane: &Plane) -> f64 {
    let lines: Vec<(Vec<Point2>, bool)> = polys.iter().map(|p| project(p, plane)).collect();
    let mut worst = 0.0_f64;
    let mut segs = 0;
    for (pts, closed) in &lines {
        segs += pts.len().saturating_sub(1) + usize::from(*closed);
        for &p in pts {
            let d = curves
                .iter()
                .map(|c| dist_curve(c, p))
                .fold(f64::INFINITY, f64::min);
            worst = worst.max(d);
        }
    }
    let m = (SAMPLES_PER_SEGMENT * segs).max(64);
    for c in curves {
        let (t0, t1) = c.param_range();
        for k in 0..=m {
            let p = c.eval(t0 + (t1 - t0) * k as f64 / m as f64);
            let d = lines
                .iter()
                .flat_map(|(pts, closed)| segments(pts, *closed))
                .map(|(a, b)| dist_segment(p, a, b))
                .fold(f64::INFINITY, f64::min);
            worst = worst.max(d);
        }
    }
    worst
}

/// Distance from `p` to `c`; splines through a 4096-point sampling (their closest point is
/// costly).
fn dist_curve(c: &Curve2, p: Point2) -> f64 {
    if !matches!(c, Curve2::BSpline(_)) {
        return c.closest(p).1.distance(p);
    }
    let (t0, t1) = c.param_range();
    let pts: Vec<Point2> = (0..=4096)
        .map(|k| c.eval(t0 + (t1 - t0) * f64::from(k) / 4096.0))
        .collect();
    pts.windows(2)
        .map(|w| dist_segment(p, w[0], w[1]))
        .fold(f64::INFINITY, f64::min)
}

// ───────────────────────────── projection ─────────────────────────────

/// The polyline in plane coordinates without consecutive duplicates (also across the closing
/// edge); a closed polyline of fewer than three points is open.
fn project(poly: &Polyline3, plane: &Plane) -> (Vec<Point2>, bool) {
    let to2 = |p: &Point3| {
        let d = *p - plane.origin;
        Point2::new(d.dot(plane.x.get()), d.dot(plane.y.get()))
    };
    let mut pts: Vec<Point2> = Vec::with_capacity(poly.points.len());
    for p in poly.points.iter().map(to2) {
        if pts.last().is_none_or(|&q| q.distance(p) > DEDUP) {
            pts.push(p);
        }
    }
    let mut closed = poly.closed;
    if closed {
        while pts.len() > 1 && pts[0].distance(pts[pts.len() - 1]) <= DEDUP {
            pts.pop();
        }
        closed = pts.len() >= 3;
    }
    (pts, closed)
}

fn segments(pts: &[Point2], closed: bool) -> impl Iterator<Item = (Point2, Point2)> + '_ {
    let n = pts.len();
    let wrap = (closed && n > 1).then(|| (pts[n - 1], pts[0]));
    pts.windows(2).map(|w| (w[0], w[1])).chain(wrap)
}

fn dist_segment(p: Point2, a: Point2, b: Point2) -> f64 {
    let d = b - a;
    let ll = d.length_squared();
    let t = if ll > 0.0 {
        ((p - a).dot(d) / ll).clamp(0.0, 1.0)
    } else {
        0.0
    };
    p.distance(a + d * t)
}

// ───────────────────────────── polyline fitting ─────────────────────────────

/// One fitted curve covering `ext[a..=b]`.
struct Piece {
    a: usize,
    b: usize,
    curve: Curve2,
}

fn kind(c: &Curve2) -> u8 {
    match c {
        Curve2::Line(_) => 0,
        Curve2::Arc(_) | Curve2::Circle(_) => 1,
        _ => 2,
    }
}

fn fit_polyline(pts: &[Point2], closed: bool, tol: f64) -> Vec<Curve2> {
    if pts.len() < 2 {
        return Vec::new();
    }
    if !closed {
        let mut pieces = fit_greedy(pts, tol);
        merge_adjacent(pts, &mut pieces, tol);
        return pieces.into_iter().map(|p| p.curve).collect();
    }
    let n = pts.len();
    let mut ext: Vec<Point2> = pts.to_vec();
    ext.push(pts[0]);
    if let Some(c @ Curve2::Circle(_)) = fit_arc(&ext, tol) {
        return vec![c];
    }
    let start = sharpest_vertex(pts);
    if start > 0 {
        ext.rotate_left(start);
        // rotate_left moved the duplicate seam point into the interior; rebuild the cycle.
        ext.truncate(n);
        ext.push(ext[0]);
    }
    let mut pieces = fit_greedy(&ext, tol);
    merge_adjacent(&ext, &mut pieces, tol);
    if pieces.len() == 1 {
        // One curve closing on itself (a spline): two halves keep profile detection cheap.
        let h = n / 2;
        pieces = fit_greedy(&ext[..=h], tol);
        let second = fit_greedy(&ext[h..], tol);
        pieces.extend(second.into_iter().map(|p| Piece {
            a: p.a + h,
            b: p.b + h,
            curve: p.curve,
        }));
    }
    let mut curves: Vec<Curve2> = pieces.iter().map(|p| p.curve.clone()).collect();
    if let [first, .., last] = pieces.as_slice() {
        let k = kind(&first.curve);
        if k == kind(&last.curve) && k < 2 {
            let mut run: Vec<Point2> = ext[last.a..=n].to_vec();
            run.extend_from_slice(&ext[1..=first.b]);
            let merged = if k == 0 {
                fit_line(&run, tol)
            } else {
                fit_arc(&run, tol)
            };
            if let Some(m) = merged {
                curves.pop();
                curves.remove(0);
                curves.push(m);
            }
        }
    }
    curves
}

/// Index of the vertex with the largest turn when that turn is a corner, else 0.
fn sharpest_vertex(pts: &[Point2]) -> usize {
    let n = pts.len();
    let mut best = (0, CORNER_TURN);
    for i in 0..n {
        let t = turn(pts[(i + n - 1) % n], pts[i], pts[(i + 1) % n]);
        if t > best.1 {
            best = (i, t);
        }
    }
    best.0
}

/// Unsigned turning angle at `b` of the path `a → b → c`, rad.
fn turn(a: Point2, b: Point2, c: Point2) -> f64 {
    let (u, v) = (b - a, c - b);
    u.perp_dot(v).atan2(u.dot(v)).abs()
}

/// Covers `ext` by pieces, each as long as one curve fits (greedy from the start).
fn fit_greedy(ext: &[Point2], tol: f64) -> Vec<Piece> {
    let last = ext.len() - 1;
    let mut out = Vec::new();
    let mut a = 0;
    while a < last {
        let (b, curve) = best_piece(ext, a, tol);
        out.push(Piece { a, b, curve });
        a = b;
    }
    out
}

/// The longest line, arc or spline starting at `ext[a]`: a line or arc, whichever reaches
/// farther (line on ties), unless a spline reaches at least twice as many segments.
fn best_piece(ext: &[Point2], a: usize, tol: f64) -> (usize, Curve2) {
    let last = ext.len() - 1;
    let line = longest(ext, a, tol, fit_line).unwrap_or_else(|| {
        let seg = Segment2 {
            a: ext[a],
            b: ext[a + 1],
        };
        (a + 1, Curve2::Line(seg))
    });
    let mut best = line;
    if let Some(arc) = longest(ext, a, tol, fit_arc)
        && arc.0 > best.0
    {
        best = arc;
    }
    if best.0 < last
        && let Some(spline) = longest(ext, a, tol, fit_bspline)
        && spline.0 - a >= 6
        && spline.0 - a >= 2 * (best.0 - a)
    {
        best = spline;
    }
    best
}

/// Farthest end `b` such that `fit(ext[a..=b])` succeeds: the best of the first few ends
/// (three collinear points have no arc), then galloping and bisection (the feasible ends are
/// taken to be an interval).
fn longest(
    ext: &[Point2],
    a: usize,
    tol: f64,
    fit: fn(&[Point2], f64) -> Option<Curve2>,
) -> Option<(usize, Curve2)> {
    let last = ext.len() - 1;
    let mut good: Option<(usize, Curve2)> = None;
    for k in 2..=4 {
        let b = (a + k).min(last);
        if let Some(c) = fit(&ext[a..=b], tol) {
            good = Some((b, c));
        }
        if b == last {
            break;
        }
    }
    let mut lo = good.as_ref()?.0;
    if lo == last {
        return good;
    }
    let mut hi;
    loop {
        let b = (a + 2 * (lo - a)).min(last);
        match fit(&ext[a..=b], tol) {
            Some(c) => {
                good = Some((b, c));
                if b == last {
                    return good;
                }
                lo = b;
            }
            None => {
                hi = b;
                break;
            }
        }
    }
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        match fit(&ext[a..=mid], tol) {
            Some(c) => {
                good = Some((mid, c));
                lo = mid;
            }
            None => hi = mid,
        }
    }
    good
}

/// Merges neighbouring lines and neighbouring arcs of `pieces` that one line / arc also fits.
fn merge_adjacent(ext: &[Point2], pieces: &mut Vec<Piece>, tol: f64) {
    let mut i = 0;
    while i + 1 < pieces.len() {
        let (p, q) = (&pieces[i], &pieces[i + 1]);
        let k = kind(&p.curve);
        let merged = if k == kind(&q.curve) && k < 2 {
            let run = &ext[p.a..=q.b];
            if k == 0 {
                fit_line(run, tol)
            } else {
                fit_arc(run, tol).filter(|c| matches!(c, Curve2::Arc(_)))
            }
        } else {
            None
        };
        match merged {
            Some(curve) => {
                let b = pieces[i + 1].b;
                pieces[i].b = b;
                pieces[i].curve = curve;
                pieces.remove(i + 1);
                i = i.saturating_sub(1);
            }
            None => i += 1,
        }
    }
}

// ───────────────────────────── candidates ─────────────────────────────

fn fit_line(run: &[Point2], tol: f64) -> Option<Curve2> {
    let (a, b) = (run[0], run[run.len() - 1]);
    if a.distance(b) <= DEDUP {
        return None;
    }
    let curve = Curve2::Line(Segment2 { a, b });
    within(&curve, run, tol).then_some(curve)
}

/// A circle (closed run) or the arc through both ends of an open run that best fits it.
fn fit_arc(run: &[Point2], tol: f64) -> Option<Curve2> {
    if run.len() < 3 {
        return None;
    }
    let (first, last) = (run[0], run[run.len() - 1]);
    let closed = first.distance(last) <= DEDUP;
    let pts = if closed { &run[..run.len() - 1] } else { run };
    let (mut center, mut radius) = kasa(pts)?;
    let chord = first.distance(last);
    if closed {
        for _ in 0..8 {
            (center, radius) = refine_free(pts, center, radius);
        }
    } else {
        let mid = first.midpoint(last);
        let n = (last - first).perp() / chord;
        let mut s = (center - mid).dot(n);
        if !s.is_finite() || s.abs() > MAX_RADIUS_RATIO * chord {
            return None;
        }
        for _ in 0..30 {
            let ds = newton_offset(pts, mid, n, s);
            s += ds;
            if ds.abs() < 1e-14 * (1.0 + s.abs()) {
                break;
            }
        }
        center = mid + n * s;
        radius = first.distance(center);
    }
    if !(radius.is_finite() && radius > DEDUP) || !center.is_finite() {
        return None;
    }
    if !closed && radius > MAX_RADIUS_RATIO * chord {
        return None;
    }
    let sweep = monotone_sweep(run, center)?;
    let curve = if closed {
        if sweep.abs() < TAU - 1e-3 {
            return None;
        }
        Curve2::Circle(Circle2 { center, radius })
    } else if sweep > 0.0 {
        Curve2::Arc(Arc2 {
            center,
            radius,
            start_angle: angle_of(first - center),
            sweep,
        })
    } else {
        Curve2::Arc(Arc2 {
            center,
            radius,
            start_angle: angle_of(last - center),
            sweep: -sweep,
        })
    };
    within(&curve, run, tol).then_some(curve)
}

fn angle_of(v: Vec2) -> f64 {
    v.y.atan2(v.x)
}

/// Signed total angle about `center` along `run`, `None` unless every step turns the same
/// way and the total stays within one turn (a closed run: exactly one).
fn monotone_sweep(run: &[Point2], center: Point2) -> Option<f64> {
    let mut total = 0.0;
    let mut sign = 0.0_f64;
    for w in run.windows(2) {
        let (u, v) = (w[0] - center, w[1] - center);
        let step = u.perp_dot(v).atan2(u.dot(v));
        if step == 0.0 {
            continue;
        }
        if sign == 0.0 {
            sign = step.signum();
        } else if step * sign < 0.0 {
            return None;
        }
        total += step;
    }
    (total.abs() > 1e-9 && total.abs() <= TAU + 1e-9).then_some(total)
}

/// Kåsa algebraic circle fit (centred coordinates): `(centre, radius)`.
fn kasa(pts: &[Point2]) -> Option<(Point2, f64)> {
    let m = pts.iter().fold(Point2::ZERO, |s, &p| s + p) / pts.len() as f64;
    let (mut sxx, mut sxy, mut syy, mut sx, mut sy) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let (mut sxz, mut syz, mut sz) = (0.0, 0.0, 0.0);
    for &p in pts {
        let (x, y) = (p.x - m.x, p.y - m.y);
        let z = x * x + y * y;
        sxx += x * x;
        sxy += x * y;
        syy += y * y;
        sx += x;
        sy += y;
        sxz += x * z;
        syz += y * z;
        sz += z;
    }
    let n = pts.len() as f64;
    // z = 2 a x + 2 b y + c  →  normal equations in (2a, 2b, c).
    let sol = solve3(
        [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]],
        [sxz, syz, sz],
    )?;
    let (a, b) = (sol[0] / 2.0, sol[1] / 2.0);
    let r2 = sol[2] + a * a + b * b;
    (r2.is_finite() && r2 > 0.0).then(|| (m + Vec2::new(a, b), r2.sqrt()))
}

fn solve3(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = |m: [[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let d = det(a);
    let scale = a
        .iter()
        .flatten()
        .fold(0.0_f64, |s, v| s.max(v.abs()))
        .powi(3);
    if !d.is_finite() || d.abs() <= 1e-14 * scale {
        return None;
    }
    let mut out = [0.0; 3];
    for (k, o) in out.iter_mut().enumerate() {
        let mut m = a;
        for (row, v) in m.iter_mut().zip(b) {
            row[k] = v;
        }
        *o = det(m) / d;
    }
    Some(out)
}

/// One Gauss–Newton step on the geometric circle fit, centre and radius free.
fn refine_free(pts: &[Point2], c: Point2, r: f64) -> (Point2, f64) {
    let n = pts.len() as f64;
    let (mut jtj, mut jtr) = ([[0.0; 3]; 3], [0.0; 3]);
    for &p in pts {
        let d = p - c;
        let len = d.length();
        if len == 0.0 {
            continue;
        }
        let u = d / len;
        let j = [-u.x, -u.y, -1.0];
        let res = len - r;
        for i in 0..3 {
            jtr[i] += j[i] * res;
            for k in 0..3 {
                jtj[i][k] += j[i] * j[k];
            }
        }
    }
    let _ = n;
    match solve3(jtj, jtr) {
        Some(s) => (c - Vec2::new(s[0], s[1]), r - s[2]),
        None => (c, r),
    }
}

/// Gauss–Newton step for the offset `s` of a centre `mid + s n` constrained to the
/// perpendicular bisector of the chord (the radius is the distance to the run's first point).
fn newton_offset(pts: &[Point2], mid: Point2, n: Vec2, s: f64) -> f64 {
    let c = mid + n * s;
    let first = pts[0];
    let (d0, l0) = (first - c, first.distance(c));
    let (mut num, mut den) = (0.0, 0.0);
    for &p in pts {
        let d = p - c;
        let len = d.length();
        if len == 0.0 || l0 == 0.0 {
            continue;
        }
        let j = -d.dot(n) / len + d0.dot(n) / l0;
        num += j * (len - l0);
        den += j * j;
    }
    if den > 0.0 { -num / den } else { 0.0 }
}

/// A cubic fit spline through a Douglas–Peucker thinning of the run (`tol / 4`).
fn fit_bspline(run: &[Point2], tol: f64) -> Option<Curve2> {
    if run.len() < 4 {
        return None;
    }
    if run
        .windows(3)
        .any(|w| turn(w[0], w[1], w[2]) > SPLINE_MAX_TURN)
    {
        return None;
    }
    let keep = douglas_peucker(run, tol / 4.0);
    if keep.len() < 3 || keep.len() > MAX_SPLINE_POINTS {
        return None;
    }
    let through: Vec<Point2> = keep.iter().map(|&i| run[i]).collect();
    let spline: BSpline2 = fit_spline(&through, None, None).ok()?;
    let curve = Curve2::BSpline(spline);
    within(&curve, run, tol).then_some(curve)
}

/// Indices of the Douglas–Peucker simplification of `run` (ends always kept).
fn douglas_peucker(run: &[Point2], eps: f64) -> Vec<usize> {
    let mut keep = vec![false; run.len()];
    keep[0] = true;
    keep[run.len() - 1] = true;
    let mut stack = vec![(0, run.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        let mut best = (0, eps);
        for (i, &p) in run.iter().enumerate().take(b).skip(a + 1) {
            let d = dist_segment(p, run[a], run[b]);
            if d > best.1 {
                best = (i, d);
            }
        }
        if best.0 > 0 {
            keep[best.0] = true;
            stack.push((a, best.0));
            stack.push((best.0, b));
        }
    }
    (0..run.len()).filter(|&i| keep[i]).collect()
}

// ───────────────────────────── deviation check ─────────────────────────────

/// Whether `p` is within `tol` of the polyline `poly`; `cursor` (the segment of the previous
/// hit) is where the search starts, so walking along the polyline is cheap.
fn near_polyline(p: Point2, poly: &[Point2], cursor: &mut usize, tol: f64) -> bool {
    let segs = poly.len() - 1;
    let hit = (0..=segs).find_map(|off| {
        [*cursor + off, cursor.wrapping_sub(off)]
            .into_iter()
            .find(|&i| i < segs && dist_segment(p, poly[i], poly[i + 1]) <= tol)
    });
    if let Some(i) = hit {
        *cursor = i;
    }
    hit.is_some()
}

/// Whether `curve` and the polyline `run` are within `tol` of each other in both directions.
/// Splines are compared through a dense sampling (their closest point is costly).
fn within(curve: &Curve2, run: &[Point2], tol: f64) -> bool {
    let segs = run.len() - 1;
    let (t0, t1) = curve.param_range();
    let per = if matches!(curve, Curve2::BSpline(_)) {
        4 * SAMPLES_PER_SEGMENT
    } else {
        SAMPLES_PER_SEGMENT
    };
    let m = (per * segs).max(32);
    let samples: Vec<Point2> = (0..=m)
        .map(|k| curve.eval(t0 + (t1 - t0) * k as f64 / m as f64))
        .collect();
    if samples.iter().any(|p| !p.is_finite()) {
        return false;
    }
    let exact = !matches!(curve, Curve2::BSpline(_));
    let mut cursor = 0;
    for &p in run {
        let ok = if exact {
            curve.closest(p).1.distance(p) <= tol
        } else {
            near_polyline(p, &samples, &mut cursor, tol)
        };
        if !ok {
            return false;
        }
    }
    let mut cursor = 0;
    samples
        .iter()
        .all(|&p| near_polyline(p, run, &mut cursor, tol))
}

#[cfg(test)]
mod tests {
    use super::*;
    use amg_core::Dir3;
    use amg_mesh::{IndexedMesh, section};
    use amg_sketch::{ProfileOptions, detect_profiles};
    use std::f64::consts::PI;

    const TOL: f64 = 0.01;

    /// A closed, outward-oriented prism over the convex CCW polygon `ring` between `z0` and
    /// `z1`, with the side quads split along alternating diagonals and fan caps.
    fn prism(ring: &[Point2], z0: f64, z1: f64) -> IndexedMesh {
        let n = ring.len() as u32;
        let mut pos: Vec<Point3> = Vec::new();
        for &z in &[z0, z1] {
            pos.extend(ring.iter().map(|p| Point3::new(p.x, p.y, z)));
        }
        let (cb, ct) = (pos.len() as u32, pos.len() as u32 + 1);
        let centroid = ring.iter().fold(Point2::ZERO, |s, &p| s + p) / ring.len() as f64;
        pos.push(Point3::new(centroid.x, centroid.y, z0));
        pos.push(Point3::new(centroid.x, centroid.y, z1));
        let mut tris = Vec::new();
        for i in 0..n {
            let j = (i + 1) % n;
            let (b0, b1, t0, t1) = (i, j, i + n, j + n);
            if i % 2 == 0 {
                tris.push([b0, b1, t1]);
                tris.push([b0, t1, t0]);
            } else {
                tris.push([b0, b1, t0]);
                tris.push([b1, t1, t0]);
            }
            tris.push([cb, j, i]);
            tris.push([ct, t0, t1]);
        }
        IndexedMesh::new(pos, tris).unwrap()
    }

    fn circle_ring(r: f64, n: usize) -> Vec<Point2> {
        (0..n)
            .map(|i| {
                let a = TAU * i as f64 / n as f64;
                Point2::new(r * a.cos(), r * a.sin())
            })
            .collect()
    }

    fn plane_at(z: f64) -> Plane {
        Plane {
            origin: Point3::new(0.0, 0.0, z),
            ..Plane::XY
        }
    }

    fn kinds(curves: &[Curve2]) -> (usize, usize, usize, usize) {
        let mut k = (0, 0, 0, 0);
        for c in curves {
            match c {
                Curve2::Line(_) => k.0 += 1,
                Curve2::Arc(_) => k.1 += 1,
                Curve2::Circle(_) => k.2 += 1,
                _ => k.3 += 1,
            }
        }
        k
    }

    fn profile_areas(curves: &[Curve2], plane: &Plane) -> Vec<f64> {
        let sketch = curves_to_sketch(curves, plane).unwrap();
        let set = detect_profiles(&sketch, &ProfileOptions::default());
        assert!(set.open_chains.is_empty(), "{:?}", set.open_chains);
        set.profiles
            .iter()
            .map(|p| {
                p.outer.signed_area() - p.holes.iter().map(|h| h.signed_area().abs()).sum::<f64>()
            })
            .collect()
    }

    #[test]
    fn cylinder_section_is_a_circle() {
        let r = 10.0;
        let m = prism(&circle_ring(r, 128), 0.0, 20.0);
        // Mid-height sections hit the quad diagonals: the points sit inside the circle.
        for z in [3.0, 7.7, 10.0, 16.1] {
            let plane = plane_at(z);
            let polys = section(&m, &plane, 0.0);
            assert_eq!(polys.len(), 1);
            let curves = section_to_sketch_curves(&polys, &plane, TOL);
            assert_eq!(kinds(&curves), (0, 0, 1, 0), "z = {z}");
            let Curve2::Circle(c) = &curves[0] else {
                panic!()
            };
            let err = (c.radius - r).abs() / r;
            assert!(err < 0.005, "z = {z}: radius error {err}");
            assert!(c.center.length() < 0.01);
            let dev = section_deviation(&curves, &polys, &plane);
            assert!(dev <= TOL, "z = {z}: deviation {dev}");
            println!("cylinder z={z}: radius error {err:.2e}, deviation {dev:.2e}");
        }
    }

    #[test]
    fn box_section_is_four_lines_one_profile() {
        let ring = [
            Point2::new(-5.0, -3.0),
            Point2::new(5.0, -3.0),
            Point2::new(5.0, 3.0),
            Point2::new(-5.0, 3.0),
        ];
        let m = prism(&ring, 0.0, 4.0);
        let plane = plane_at(2.0);
        let polys = section(&m, &plane, 0.0);
        let curves = section_to_sketch_curves(&polys, &plane, TOL);
        assert_eq!(kinds(&curves), (4, 0, 0, 0));
        assert!(section_deviation(&curves, &polys, &plane) <= TOL);
        let areas = profile_areas(&curves, &plane);
        assert_eq!(areas.len(), 1);
        assert!((areas[0] - 60.0).abs() < 1e-9);
    }

    #[test]
    fn stadium_section_is_lines_and_arcs() {
        // Two semicircles (r = 4, centres (±6, 0)) joined by straight sides.
        let mut ring = Vec::new();
        let k = 48;
        for i in 0..=k {
            let a = -PI / 2.0 + PI * i as f64 / k as f64;
            ring.push(Point2::new(6.0 + 4.0 * a.cos(), 4.0 * a.sin()));
        }
        for i in 0..=k {
            let a = PI / 2.0 + PI * i as f64 / k as f64;
            ring.push(Point2::new(-6.0 + 4.0 * a.cos(), 4.0 * a.sin()));
        }
        let m = prism(&ring, 0.0, 5.0);
        let plane = plane_at(2.5);
        let polys = section(&m, &plane, 0.0);
        let curves = section_to_sketch_curves(&polys, &plane, TOL);
        let (lines, arcs, circles, splines) = kinds(&curves);
        assert_eq!((circles, splines), (0, 0), "{curves:?}");
        assert_eq!((lines, arcs), (2, 2), "{curves:?}");
        for c in &curves {
            if let Curve2::Arc(a) = c {
                assert!((a.radius - 4.0).abs() / 4.0 < 0.005);
            }
        }
        let dev = section_deviation(&curves, &polys, &plane);
        assert!(dev <= TOL, "deviation {dev}");
        let areas = profile_areas(&curves, &plane);
        assert_eq!(areas.len(), 1);
        let exact = 12.0 * 8.0 + PI * 16.0;
        assert!((areas[0] - exact).abs() / exact < 0.005, "{}", areas[0]);
        println!("stadium: {lines} lines, {arcs} arcs, deviation {dev:.2e}");
    }

    fn tilted_ellipse() -> (Vec<Polyline3>, Plane, f64, f64) {
        let r = 8.0;
        let m = prism(&circle_ring(r, 256), -30.0, 30.0);
        // Plane tilted 60° about the y axis, through (0, 0, 1).
        let th = 60.0_f64.to_radians();
        let normal = Dir3::new_unchecked(amg_core::Vec3::new(-th.sin(), 0.0, th.cos()));
        let plane = Plane::from_normal(Point3::new(0.0, 0.0, 1.0), normal);
        let polys = section(&m, &plane, 0.0);
        assert_eq!(polys.len(), 1);
        assert!(polys[0].closed);
        (polys, plane, PI * r * r / th.cos(), r)
    }

    #[test]
    fn tilted_section_of_a_cylinder_is_ellipse_splines() {
        let (polys, plane, _, _) = tilted_ellipse();
        let curves = section_to_sketch_curves(&polys, &plane, TOL);
        let (lines, arcs, circles, splines) = kinds(&curves);
        assert_eq!((lines, circles), (0, 0), "{curves:?}");
        assert!(splines >= 2, "{curves:?}");
        assert!(curves.len() <= 8, "{} pieces ({arcs} arcs)", curves.len());
        let dev = section_deviation(&curves, &polys, &plane);
        assert!(dev <= TOL, "deviation {dev}");
        println!(
            "ellipse: {} pieces ({splines} splines, {arcs} arcs), deviation {dev:.2e}",
            curves.len()
        );
    }

    /// SK-06 on spline boundaries is very slow (minutes for this ellipse), hence ignored.
    #[test]
    #[ignore = "amg-sketch profile detection on splines takes minutes"]
    fn tilted_ellipse_closes_into_one_profile() {
        let (polys, plane, exact, _) = tilted_ellipse();
        let curves = section_to_sketch_curves(&polys, &plane, TOL);
        let areas = profile_areas(&curves, &plane);
        assert_eq!(areas.len(), 1);
        assert!((areas[0] - exact).abs() / exact < 0.005, "{}", areas[0]);
    }

    #[test]
    fn open_chain_stays_open() {
        // Open half circle plus a straight tail.
        let mut points: Vec<Point3> = (0..=60)
            .map(|i| {
                let a = PI * i as f64 / 60.0;
                Point3::new(5.0 * a.cos(), 5.0 * a.sin(), 0.0)
            })
            .collect();
        points.push(Point3::new(-5.0, -10.0, 0.0));
        let poly = Polyline3 {
            points,
            closed: false,
        };
        let curves = section_to_sketch_curves(std::slice::from_ref(&poly), &Plane::XY, TOL);
        assert_eq!(kinds(&curves), (1, 1, 0, 0), "{curves:?}");
        let dev = section_deviation(&curves, &[poly], &Plane::XY);
        assert!(dev <= TOL, "deviation {dev}");
        let sketch = curves_to_sketch(&curves, &Plane::XY).unwrap();
        let set = detect_profiles(&sketch, &ProfileOptions::default());
        assert!(set.profiles.is_empty());
    }

    #[test]
    fn degenerate_input_gives_nothing() {
        let p = Point3::new(1.0, 2.0, 0.0);
        let one = Polyline3 {
            points: vec![p, p, p],
            closed: true,
        };
        assert!(section_to_sketch_curves(&[one], &Plane::XY, TOL).is_empty());
        assert!(section_to_sketch_curves(&[], &Plane::XY, TOL).is_empty());
    }

    #[test]
    fn coarse_tessellation_stays_lines() {
        // A 12-gon of radius 10 has a 0.34 mm sagitta: no arc fits within 0.01 mm.
        let m = prism(&circle_ring(10.0, 12), 0.0, 5.0);
        let plane = plane_at(2.5);
        let polys = section(&m, &plane, 0.0);
        let curves = section_to_sketch_curves(&polys, &plane, TOL);
        assert_eq!(kinds(&curves), (12, 0, 0, 0));
        assert!(section_deviation(&curves, &polys, &plane) <= 1e-9);
        // A tolerance above the sagitta recovers the circle.
        let curves = section_to_sketch_curves(&polys, &plane, 0.5);
        assert_eq!(kinds(&curves), (0, 0, 1, 0));
    }
}
