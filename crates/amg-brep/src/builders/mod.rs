//! Face and wire builders (S02 §4, BREP-03): input validation in plain Rust, then one backend
//! call that builds the topology.
//!
//! Validation errors name the offending profile, loop and edge. Loop indices are 0 for the outer
//! loop and `1 + i` for hole `i`.

mod eval;

use amg_core::{Dir3, Plane, Point2, Point3, Tolerance, Vec3};

use crate::error::{KResult, KernelError};
use crate::geom::{BSpline2dData, BSplineSurfaceData, Curve2d, Curve3, CurveSeg, FaceBounds};
use crate::geom::{Loop2, PlanarProfile, Surface, Tag};

use eval::distinct_knots;

/// One edge for the backend: `curve` between `first < last`, run backwards when `reversed`.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct EdgeIn {
    pub curve: Curve3,
    pub first: f64,
    pub last: f64,
    pub reversed: bool,
}

/// Edges of several loops; loop `i` is `edges[starts[i]..starts[i + 1]]`.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct LoopsIn {
    pub edges: Vec<EdgeIn>,
    pub starts: Vec<u32>,
}

impl LoopsIn {
    fn push_loop(&mut self, edges: impl IntoIterator<Item = EdgeIn>) {
        if self.starts.is_empty() {
            self.starts.push(0);
        }
        self.edges.extend(edges);
        self.starts.push(len_u32(self.edges.len()));
    }
}

/// The bounds of a face on a surface, lowered for the backend.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BoundsIn {
    Natural,
    UvRect([f64; 4]),
    Loops(LoopsIn),
    /// Points of all polylines; polyline `i` is `points[starts[i]..starts[i + 1]]`.
    Polylines {
        points: Vec<Point3>,
        starts: Vec<u32>,
    },
}

/// A planar face for the backend: the plane and its loops (outer first, correctly oriented).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PlanarIn {
    pub plane: Plane,
    pub loops: LoopsIn,
}

fn len_u32(n: usize) -> u32 {
    u32::try_from(n).unwrap_or(u32::MAX)
}

fn invalid(what: &'static str, reason: String) -> KernelError {
    KernelError::InvalidInput {
        what,
        reason,
        offending: Vec::new(),
    }
}

fn finite2(p: Point2) -> bool {
    p.x.is_finite() && p.y.is_finite()
}

fn check_tol(tol: &Tolerance) -> KResult<()> {
    if tol.linear.is_finite() && tol.linear > 0.0 {
        Ok(())
    } else {
        Err(invalid(
            "tol",
            format!("linear tolerance {} is not positive", tol.linear),
        ))
    }
}

/// The map from the plane's local `(u, v)` to world points.
fn on_plane(plane: &Plane, p: Point2) -> Point3 {
    plane.origin + plane.x.get() * p.x + plane.y.get() * p.y
}

/// A local frame at `center` with the plane's normal and `x` along the local direction `u`.
fn frame_at(plane: &Plane, center: Point2, u: amg_core::Vec2) -> Plane {
    let x = (plane.x.get() * u.x + plane.y.get() * u.y).normalize();
    let y = plane.normal.get().cross(x);
    Plane {
        origin: on_plane(plane, center),
        x: Dir3::new_unchecked(x),
        y: Dir3::new_unchecked(y),
        normal: plane.normal,
    }
}

// ---- profile validation ------------------------------------------------------------------

/// Checks one profile curve's own data. `Err(reason)` on malformed input.
fn check_curve2(c: &Curve2d, tol: &Tolerance) -> Result<(), String> {
    let pos = |r: f64| r.is_finite() && r > tol.linear;
    match c {
        Curve2d::Line { a, b } => {
            if !finite2(*a) || !finite2(*b) {
                return Err("line has a non-finite end point".into());
            }
            if (*b - *a).length() <= tol.linear {
                return Err("line is shorter than the tolerance".into());
            }
        }
        Curve2d::Circle { center, radius } => {
            if !finite2(*center) || !pos(*radius) {
                return Err(format!("circle radius {radius} is not positive"));
            }
        }
        Curve2d::Arc {
            center,
            radius,
            start_angle,
            sweep,
        } => {
            if !finite2(*center) || !pos(*radius) || !start_angle.is_finite() {
                return Err(format!("arc radius {radius} is not positive"));
            }
            if !sweep.is_finite()
                || sweep.abs() * radius <= tol.linear
                || sweep.abs() > std::f64::consts::TAU + 1e-12
            {
                return Err(format!("arc sweep {sweep} is not in (0, 2π]"));
            }
        }
        Curve2d::Ellipse {
            center,
            major,
            minor_radius,
            t0,
            t1,
        } => {
            if !finite2(*center) || !finite2(*major) || !pos(major.length()) || !pos(*minor_radius)
            {
                return Err("ellipse has a zero or non-finite radius".into());
            }
            let span = t1 - t0;
            if !span.is_finite() || span == 0.0 || span.abs() > std::f64::consts::TAU + 1e-12 {
                return Err(format!("ellipse span t1 - t0 = {span} is not in (0, 2π]"));
            }
        }
        Curve2d::BSpline(b) => check_bspline2(b)?,
    }
    Ok(())
}

fn check_bspline2(b: &BSpline2dData) -> Result<(), String> {
    let p = usize::from(b.degree);
    let n = b.poles.len();
    if p == 0 {
        return Err("B-spline degree is 0".into());
    }
    if n < p + 1 {
        return Err(format!(
            "B-spline has {n} poles, degree {p} needs at least {}",
            p + 1
        ));
    }
    if b.knots.len() != n + p + 1 {
        return Err(format!(
            "B-spline has {} knots, {n} poles of degree {p} need {}",
            b.knots.len(),
            n + p + 1
        ));
    }
    if b.knots.iter().any(|k| !k.is_finite()) || b.knots.windows(2).any(|w| w[1] < w[0]) {
        return Err("B-spline knots are not finite and non-decreasing".into());
    }
    if b.knots[n] <= b.knots[p] {
        return Err("B-spline has an empty parameter range".into());
    }
    let (_, mults) = distinct_knots(&b.knots);
    if mults.iter().any(|&m| m as usize > p + 1) {
        return Err("a B-spline knot repeats more than degree + 1 times".into());
    }
    if b.poles.iter().any(|q| !finite2(*q)) {
        return Err("B-spline has a non-finite pole".into());
    }
    if let Some(w) = &b.weights
        && (w.len() != n || w.iter().any(|w| !w.is_finite() || *w <= 0.0))
    {
        return Err("B-spline weights must be one positive weight per pole".into());
    }
    Ok(())
}

/// Samples a loop as a closed polygon (no repeated closing point) and returns the largest
/// distance between a chord midpoint and the curve (the polygon's error bound).
fn sample_loop(edges: &[&Curve2d]) -> (Vec<Point2>, f64) {
    let mut points = Vec::new();
    let mut max_dev: f64 = 0.0;
    for c in edges {
        let n = eval::segments2(c);
        #[allow(clippy::cast_precision_loss, reason = "small counts")]
        let nf = n as f64;
        let mut prev = eval::point2(c, 0.0);
        for i in 0..n {
            points.push(prev);
            #[allow(clippy::cast_precision_loss, reason = "small counts")]
            let s1 = (i + 1) as f64 / nf;
            #[allow(clippy::cast_precision_loss, reason = "small counts")]
            let sm = (i as f64 + 0.5) / nf;
            let next = eval::point2(c, s1);
            let mid = eval::point2(c, sm);
            max_dev = max_dev.max((mid - (prev + next) * 0.5).length());
            prev = next;
        }
    }
    (points, max_dev)
}

/// Twice the signed area of a closed polygon (positive when counter-clockwise).
fn signed_area2(points: &[Point2]) -> f64 {
    let n = points.len();
    (0..n)
        .map(|i| {
            let (a, b) = (points[i], points[(i + 1) % n]);
            a.x * b.y - b.x * a.y
        })
        .sum()
}

/// Winding number of `polygon` around `p`.
fn winding(polygon: &[Point2], p: Point2) -> i32 {
    let n = polygon.len();
    let mut w = 0;
    for i in 0..n {
        let (a, b) = (polygon[i], polygon[(i + 1) % n]);
        let side = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
        if a.y <= p.y {
            if b.y > p.y && side > 0.0 {
                w += 1;
            }
        } else if b.y <= p.y && side < 0.0 {
            w -= 1;
        }
    }
    w
}

fn distance_to_polygon(polygon: &[Point2], p: Point2) -> f64 {
    let n = polygon.len();
    (0..n)
        .map(|i| {
            let (a, b) = (polygon[i], polygon[(i + 1) % n]);
            let ab = b - a;
            let len2 = ab.length_squared();
            let t = if len2 > 0.0 {
                ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            (p - (a + ab * t)).length()
        })
        .fold(f64::INFINITY, f64::min)
}

/// Whether `p` is inside `polygon` or within `slack` of its boundary.
fn inside_or_on(polygon: &[Point2], p: Point2, slack: f64) -> bool {
    winding(polygon, p) != 0 || distance_to_polygon(polygon, p) <= slack
}

/// One validated, sampled loop.
struct CheckedLoop {
    samples: Vec<Point2>,
    max_dev: f64,
}

/// Checks a profile loop: curve data, closure, orientation. `role_ccw` is the required
/// direction (outer CCW, holes CW).
fn check_loop(
    profile: usize,
    loop_index: u32,
    lp: &Loop2,
    role_ccw: bool,
    tol: &Tolerance,
) -> KResult<CheckedLoop> {
    let at = |e: usize| format!("profile {profile}, loop {loop_index}, edge {e}");
    if lp.edges.is_empty() {
        return Err(invalid(
            "profiles",
            format!("profile {profile}, loop {loop_index}: the loop has no edges"),
        ));
    }
    for (e, edge) in lp.edges.iter().enumerate() {
        check_curve2(&edge.curve, tol)
            .map_err(|r| invalid("profiles", format!("{}: {r}", at(e))))?;
        if matches!(edge.curve, Curve2d::Circle { .. }) && lp.edges.len() > 1 {
            return Err(invalid(
                "profiles",
                format!("{}: a full circle must be a loop by itself", at(e)),
            ));
        }
    }
    let n = lp.edges.len();
    for e in 0..n {
        let end = eval::point2(&lp.edges[e].curve, 1.0);
        let next = eval::point2(&lp.edges[(e + 1) % n].curve, 0.0);
        let gap = (next - end).length();
        if gap > tol.linear || !gap.is_finite() {
            return Err(KernelError::OpenLoop {
                loop_index,
                edge_index: len_u32(e),
                gap,
            });
        }
    }
    let curves: Vec<&Curve2d> = lp.edges.iter().map(|e| &e.curve).collect();
    let (samples, max_dev) = sample_loop(&curves);
    let lone_circle = matches!(lp.edges[0].curve, Curve2d::Circle { .. });
    let area2 = if lone_circle {
        0.0
    } else {
        signed_area2(&samples)
    };
    if !lone_circle {
        let perimeter: f64 = (0..samples.len())
            .map(|i| (samples[(i + 1) % samples.len()] - samples[i]).length())
            .sum();
        if area2.abs() <= tol.linear * perimeter {
            return Err(invalid(
                "profiles",
                format!("profile {profile}, loop {loop_index}: the loop encloses no area"),
            ));
        }
        if (area2 > 0.0) != role_ccw {
            let (want, got) = if role_ccw {
                ("CCW", "CW")
            } else {
                ("CW", "CCW")
            };
            let role = if loop_index == 0 {
                "the outer loop"
            } else {
                "a hole"
            };
            return Err(invalid(
                "profiles",
                format!(
                    "profile {profile}, loop {loop_index}: {role} must run {want}, it runs {got}"
                ),
            ));
        }
    }
    Ok(CheckedLoop { samples, max_dev })
}

/// Lowers one profile curve onto `plane`. `hole` orients a lone circle clockwise.
fn lower_curve2(plane: &Plane, c: &Curve2d, hole: bool) -> EdgeIn {
    let tau = std::f64::consts::TAU;
    match c {
        Curve2d::Line { a, b } => {
            let (pa, pb) = (on_plane(plane, *a), on_plane(plane, *b));
            let d = pb - pa;
            EdgeIn {
                curve: Curve3::Line {
                    origin: pa,
                    dir: Dir3::new_unchecked(d.normalize()),
                },
                first: 0.0,
                last: d.length(),
                reversed: false,
            }
        }
        Curve2d::Circle { center, radius } => EdgeIn {
            curve: Curve3::Circle {
                frame: frame_at(plane, *center, amg_core::Vec2::X),
                radius: *radius,
            },
            first: 0.0,
            last: tau,
            reversed: hole,
        },
        Curve2d::Arc {
            center,
            radius,
            start_angle,
            sweep,
        } => {
            let curve = Curve3::Circle {
                frame: frame_at(plane, *center, amg_core::Vec2::X),
                radius: *radius,
            };
            let (first, last) = if *sweep > 0.0 {
                (*start_angle, start_angle + sweep)
            } else {
                (start_angle + sweep, *start_angle)
            };
            EdgeIn {
                curve,
                first,
                last,
                reversed: *sweep < 0.0,
            }
        }
        Curve2d::Ellipse {
            center,
            major,
            minor_radius,
            t0,
            t1,
        } => {
            let (c, u, a, b, shift) = eval::ellipse_axes(*center, *major, *minor_radius);
            let curve = Curve3::Ellipse {
                frame: frame_at(plane, c, u),
                major_radius: a,
                minor_radius: b,
            };
            let (s0, s1) = (t0 + shift, t1 + shift);
            EdgeIn {
                curve,
                first: s0.min(s1),
                last: s0.max(s1),
                reversed: s1 < s0,
            }
        }
        Curve2d::BSpline(b) => {
            let (knots, mults) = distinct_knots(&b.knots);
            let (first, last) = eval::bspline2_range(b);
            EdgeIn {
                curve: Curve3::BSpline(crate::geom::BSplineCurveData {
                    degree: b.degree,
                    knots,
                    mults,
                    poles: b.poles.iter().map(|p| on_plane(plane, *p)).collect(),
                    weights: b.weights.clone(),
                    periodic: false,
                }),
                first,
                last,
                reversed: false,
            }
        }
    }
}

fn check_plane(plane: &Plane, what: &'static str) -> KResult<()> {
    let (x, y, n) = (plane.x.get(), plane.y.get(), plane.normal.get());
    let ok = plane.origin.is_finite()
        && [x, y, n].iter().all(|v| (v.length() - 1.0).abs() < 1e-9)
        && x.dot(y).abs() < 1e-9
        && (x.cross(y) - n).length() < 1e-9;
    if ok {
        Ok(())
    } else {
        Err(invalid(
            what,
            "the plane frame is not right-handed orthonormal".into(),
        ))
    }
}

/// Validates `profiles` and lowers them for the backend, keeping each edge's tag in input order
/// (outer loop, then holes).
pub(crate) fn lower_profiles(
    profiles: &[PlanarProfile],
    tol: &Tolerance,
) -> KResult<(Vec<PlanarIn>, Vec<Vec<Tag>>)> {
    check_tol(tol)?;
    let mut lowered = Vec::with_capacity(profiles.len());
    let mut tags = Vec::with_capacity(profiles.len());
    for (pi, profile) in profiles.iter().enumerate() {
        check_plane(&profile.plane, "profiles")?;
        let outer = check_loop(pi, 0, &profile.outer, true, tol)?;
        let slack = outer.max_dev + tol.linear;
        for (hi, hole) in profile.holes.iter().enumerate() {
            let loop_index = len_u32(hi + 1);
            let checked = check_loop(pi, loop_index, hole, false, tol)?;
            if let Some(i) = checked
                .samples
                .iter()
                .position(|p| !inside_or_on(&outer.samples, *p, slack))
            {
                // The edge that owns sample i.
                let mut count = 0;
                let mut edge = 0;
                for (e, h) in hole.edges.iter().enumerate() {
                    count += eval::segments2(&h.curve);
                    if i < count {
                        edge = e;
                        break;
                    }
                }
                return Err(invalid(
                    "profiles",
                    format!(
                        "profile {pi}, loop {loop_index}, edge {edge}: the hole is outside the \
                         outer loop"
                    ),
                ));
            }
        }
        let mut loops = LoopsIn::default();
        let mut profile_tags = Vec::new();
        for (li, lp) in std::iter::once(&profile.outer)
            .chain(&profile.holes)
            .enumerate()
        {
            loops.push_loop(
                lp.edges
                    .iter()
                    .map(|e| lower_curve2(&profile.plane, &e.curve, li > 0)),
            );
            profile_tags.extend(lp.edges.iter().map(|e| e.tag));
        }
        lowered.push(PlanarIn {
            plane: profile.plane,
            loops,
        });
        tags.push(profile_tags);
    }
    Ok((lowered, tags))
}

// ---- polygons ----------------------------------------------------------------------------

/// Drops non-finite input and consecutive points within `tol` (cyclically).
fn clean_polygon(points: &[Point3], tol: f64, what: &'static str) -> KResult<Vec<Point3>> {
    if points.iter().any(|p| !p.is_finite()) {
        return Err(invalid(what, "a point is not finite".into()));
    }
    let mut out: Vec<Point3> = Vec::with_capacity(points.len());
    for p in points {
        if out.last().is_none_or(|q| (*p - *q).length() > tol) {
            out.push(*p);
        }
    }
    while out.len() > 1 && (out[out.len() - 1] - out[0]).length() <= tol {
        out.pop();
    }
    Ok(out)
}

/// Newell normal (twice the vector area) of a closed polygon.
fn newell(points: &[Point3]) -> Vec3 {
    let n = points.len();
    let c = points.iter().copied().sum::<Point3>() / points.len().max(1) as f64;
    (0..n)
        .map(|i| (points[i] - c).cross(points[(i + 1) % n] - c))
        .sum()
}

fn line_edge(a: Point3, b: Point3) -> EdgeIn {
    let d = b - a;
    EdgeIn {
        curve: Curve3::Line {
            origin: a,
            dir: Dir3::new_unchecked(d.normalize()),
        },
        first: 0.0,
        last: d.length(),
        reversed: false,
    }
}

/// Validates a planar polygon with holes and lowers it to a planar face: vertices merged within
/// `tol.linear`, the plane fitted (Newell normal through the outer centroid), every point
/// projected onto it. Holes are re-oriented clockwise as needed.
pub(crate) fn lower_polygon(
    outer: &[Point3],
    holes: &[Vec<Point3>],
    tol: &Tolerance,
) -> KResult<PlanarIn> {
    check_tol(tol)?;
    let outer = clean_polygon(outer, tol.linear, "outer")?;
    if outer.len() < 3 {
        return Err(invalid(
            "outer",
            format!("{} distinct points, a polygon needs 3", outer.len()),
        ));
    }
    let holes: Vec<Vec<Point3>> = holes
        .iter()
        .map(|h| clean_polygon(h, tol.linear, "holes"))
        .collect::<KResult<_>>()?;
    if let Some(i) = holes.iter().position(|h| h.len() < 3) {
        return Err(invalid(
            "holes",
            format!(
                "hole {i} has {} distinct points, it needs 3",
                holes[i].len()
            ),
        ));
    }
    let n = newell(&outer);
    let perimeter: f64 = (0..outer.len())
        .map(|i| (outer[(i + 1) % outer.len()] - outer[i]).length())
        .sum();
    if n.length() <= tol.linear * perimeter {
        return Err(invalid("outer", "the polygon encloses no area".into()));
    }
    let normal = Dir3::new_unchecked(n.normalize());
    #[allow(clippy::cast_precision_loss, reason = "point counts")]
    let origin = outer.iter().copied().sum::<Point3>() / outer.len() as f64;
    let plane = Plane::from_normal_x(origin, normal, outer[1] - outer[0], tol)
        .unwrap_or_else(|_| Plane::from_normal(origin, normal));
    let max_dev = outer
        .iter()
        .chain(holes.iter().flatten())
        .map(|p| (*p - origin).dot(n.normalize()).abs())
        .fold(0.0, f64::max);
    if max_dev > tol.linear {
        return Err(KernelError::NotPlanar { max_dev });
    }
    let local = |p: &Point3| {
        let d = *p - origin;
        Point2::new(d.dot(plane.x.get()), d.dot(plane.y.get()))
    };
    let outer2: Vec<Point2> = outer.iter().map(local).collect();
    let mut loops = LoopsIn::default();
    let ring = |pts: &[Point2]| -> Vec<EdgeIn> {
        let m = pts.len();
        (0..m)
            .map(|i| line_edge(on_plane(&plane, pts[i]), on_plane(&plane, pts[(i + 1) % m])))
            .collect()
    };
    loops.push_loop(ring(&outer2));
    for (i, hole) in holes.iter().enumerate() {
        let mut h2: Vec<Point2> = hole.iter().map(local).collect();
        let area2 = signed_area2(&h2);
        let hp: f64 = (0..h2.len())
            .map(|k| (h2[(k + 1) % h2.len()] - h2[k]).length())
            .sum();
        if area2.abs() <= tol.linear * hp {
            return Err(invalid("holes", format!("hole {i} encloses no area")));
        }
        if area2 > 0.0 {
            h2.reverse();
        }
        if h2.iter().any(|p| !inside_or_on(&outer2, *p, tol.linear)) {
            return Err(invalid(
                "holes",
                format!("hole {i} is outside the outer loop"),
            ));
        }
        loops.push_loop(ring(&h2));
    }
    Ok(PlanarIn { plane, loops })
}

// ---- faces on surfaces and wires ---------------------------------------------------------

fn check_bspline_surface(b: &BSplineSurfaceData) -> Result<(), String> {
    let total = |m: &[u32]| m.iter().map(|&m| m as usize).sum::<usize>();
    let (nu, nv) = (b.nu as usize, b.nv as usize);
    let (pu, pv) = (usize::from(b.u_degree), usize::from(b.v_degree));
    if pu == 0 || pv == 0 {
        return Err("B-spline surface degree is 0".into());
    }
    if b.poles.len() != nu * nv {
        return Err(format!("{} poles, nu × nv = {}", b.poles.len(), nu * nv));
    }
    if b.u_knots.len() != b.u_mults.len() || b.v_knots.len() != b.v_mults.len() {
        return Err("knot and multiplicity counts differ".into());
    }
    if !b.u_periodic && total(&b.u_mults) != nu + pu + 1 {
        return Err(format!(
            "u multiplicities sum to {}, need {}",
            total(&b.u_mults),
            nu + pu + 1
        ));
    }
    if !b.v_periodic && total(&b.v_mults) != nv + pv + 1 {
        return Err(format!(
            "v multiplicities sum to {}, need {}",
            total(&b.v_mults),
            nv + pv + 1
        ));
    }
    for k in [&b.u_knots, &b.v_knots] {
        if k.iter().any(|x| !x.is_finite()) || k.windows(2).any(|w| w[1] <= w[0]) {
            return Err("knots are not finite and increasing".into());
        }
    }
    if b.poles.iter().any(|p| !p.is_finite()) {
        return Err("a pole is not finite".into());
    }
    if let Some(w) = &b.weights
        && (w.len() != b.poles.len() || w.iter().any(|w| !w.is_finite() || *w <= 0.0))
    {
        return Err("weights must be one positive weight per pole".into());
    }
    Ok(())
}

fn check_surface(s: &Surface) -> KResult<()> {
    let pos = |r: f64| r.is_finite() && r > 0.0;
    let frame = |p: &Plane| check_plane(p, "surface");
    match s {
        Surface::Plane(p) => frame(p),
        Surface::Cylinder { frame: f, radius } | Surface::Sphere { frame: f, radius } => {
            frame(f)?;
            if pos(*radius) {
                Ok(())
            } else {
                Err(invalid(
                    "surface",
                    format!("radius {radius} is not positive"),
                ))
            }
        }
        Surface::Cone {
            frame: f,
            ref_radius,
            half_angle,
        } => {
            frame(f)?;
            let limit = std::f64::consts::FRAC_PI_2;
            if ref_radius.is_finite()
                && *ref_radius >= 0.0
                && half_angle.is_finite()
                && half_angle.abs() > 1e-12
                && half_angle.abs() < limit
            {
                Ok(())
            } else {
                Err(invalid(
                    "surface",
                    "cone radius or half angle out of range".into(),
                ))
            }
        }
        Surface::Torus {
            frame: f,
            major,
            minor,
        } => {
            frame(f)?;
            if pos(*major) && pos(*minor) {
                Ok(())
            } else {
                Err(invalid("surface", "torus radii must be positive".into()))
            }
        }
        Surface::BSpline(b) => check_bspline_surface(b).map_err(|r| invalid("surface", r)),
        Surface::Other { .. } => Err(invalid(
            "surface",
            "only plane, cylinder, cone, sphere, torus and B-spline surfaces can be built".into(),
        )),
    }
}

/// Checks a curve segment's own data and lowers it.
fn lower_seg(seg: &CurveSeg) -> Result<EdgeIn, String> {
    let (t0, t1) = (seg.t0, seg.t1);
    if !t0.is_finite() || !t1.is_finite() || (t1 - t0).abs() <= 0.0 {
        return Err(format!("parameter range [{t0}, {t1}] is empty"));
    }
    match &seg.curve {
        Curve3::Line { origin, .. } => {
            if !origin.is_finite() {
                return Err("line origin is not finite".into());
            }
        }
        Curve3::Circle { frame, radius } => {
            check_plane(frame, "segs").map_err(|e| e.to_string())?;
            if !(radius.is_finite() && *radius > 0.0) {
                return Err(format!("circle radius {radius} is not positive"));
            }
        }
        Curve3::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => {
            check_plane(frame, "segs").map_err(|e| e.to_string())?;
            if !(minor_radius.is_finite() && *minor_radius > 0.0 && major_radius >= minor_radius) {
                return Err("ellipse radii must satisfy major >= minor > 0".into());
            }
        }
        Curve3::BSpline(b) => {
            if b.periodic {
                return Err("periodic B-spline segments are not supported".into());
            }
            let p = usize::from(b.degree);
            let total: usize = b.mults.iter().map(|&m| m as usize).sum();
            if p == 0
                || b.knots.len() != b.mults.len()
                || total != b.poles.len() + p + 1
                || b.knots.windows(2).any(|w| w[1] <= w[0])
                || b.poles.iter().any(|q| !q.is_finite())
                || b.weights
                    .as_ref()
                    .is_some_and(|w| w.len() != b.poles.len() || w.iter().any(|w| *w <= 0.0))
            {
                return Err("malformed B-spline".into());
            }
            let (lo, hi) = eval::bspline3_range(b);
            let eps = 1e-12 * (hi - lo).abs().max(1.0);
            if t0.min(t1) < lo - eps || t0.max(t1) > hi + eps {
                return Err(format!(
                    "range [{t0}, {t1}] is outside the knots [{lo}, {hi}]"
                ));
            }
        }
        Curve3::Other => return Err("`Curve3::Other` cannot be built".into()),
    }
    Ok(EdgeIn {
        curve: seg.curve.clone(),
        first: t0.min(t1),
        last: t0.max(t1),
        reversed: t1 < t0,
    })
}

fn seg_ends(seg: &CurveSeg) -> (Point3, Point3) {
    let at = |t| eval::point3(&seg.curve, t).unwrap_or(Point3::NAN);
    (at(seg.t0), at(seg.t1))
}

/// Validates and lowers `make_face` input. Returns the tag of every loop edge in input order.
pub(crate) fn lower_face(
    surface: &Surface,
    bounds: &FaceBounds,
    tol: f64,
) -> KResult<(BoundsIn, Vec<Tag>)> {
    if !(tol.is_finite() && tol > 0.0) {
        return Err(invalid("tol", format!("tolerance {tol} is not positive")));
    }
    check_surface(surface)?;
    match bounds {
        FaceBounds::Natural => {
            if matches!(
                surface,
                Surface::Plane(_) | Surface::Cylinder { .. } | Surface::Cone { .. }
            ) {
                return Err(invalid(
                    "bounds",
                    "the surface is infinite; give UvRect, Loops or Polylines bounds".into(),
                ));
            }
            Ok((BoundsIn::Natural, Vec::new()))
        }
        FaceBounds::UvRect { u0, u1, v0, v1 } => {
            let ok = [u0, u1, v0, v1].iter().all(|x| x.is_finite()) && u0 < u1 && v0 < v1;
            if !ok {
                return Err(invalid("bounds", "UvRect needs u0 < u1 and v0 < v1".into()));
            }
            Ok((BoundsIn::UvRect([*u0, *u1, *v0, *v1]), Vec::new()))
        }
        FaceBounds::Loops(loops) => {
            if loops.is_empty() {
                return Err(invalid("bounds", "no loops".into()));
            }
            let mut out = LoopsIn::default();
            let mut tags = Vec::new();
            for (li, lp) in loops.iter().enumerate() {
                if lp.is_empty() {
                    return Err(invalid("bounds", format!("loop {li} has no edges")));
                }
                let edges = lp
                    .iter()
                    .enumerate()
                    .map(|(e, s)| {
                        lower_seg(s)
                            .map_err(|r| invalid("bounds", format!("loop {li}, edge {e}: {r}")))
                    })
                    .collect::<KResult<Vec<_>>>()?;
                for e in 0..lp.len() {
                    let (_, end) = seg_ends(&lp[e]);
                    let (start, _) = seg_ends(&lp[(e + 1) % lp.len()]);
                    let gap = (start - end).length();
                    if gap.is_nan() || gap > tol {
                        return Err(KernelError::OpenLoop {
                            loop_index: len_u32(li),
                            edge_index: len_u32(e),
                            gap,
                        });
                    }
                }
                out.push_loop(edges);
                tags.extend(lp.iter().map(|s| s.tag));
            }
            Ok((BoundsIn::Loops(out), tags))
        }
        FaceBounds::Polylines(lines) => {
            if lines.is_empty() {
                return Err(invalid("bounds", "no polylines".into()));
            }
            let mut points = Vec::new();
            let mut starts = vec![0];
            for (i, line) in lines.iter().enumerate() {
                let clean = clean_polygon(line, tol * 1e-3, "bounds")?;
                if clean.len() < 3 {
                    return Err(invalid(
                        "bounds",
                        format!(
                            "polyline {i} has {} distinct points, it needs 3",
                            clean.len()
                        ),
                    ));
                }
                points.extend(clean);
                starts.push(len_u32(points.len()));
            }
            Ok((BoundsIn::Polylines { points, starts }, Vec::new()))
        }
    }
}

/// Validates and lowers `make_wire` input. Returns the edges and whether the wire closes.
pub(crate) fn lower_wire(segs: &[CurveSeg], tol: &Tolerance) -> KResult<(LoopsIn, bool)> {
    check_tol(tol)?;
    if segs.is_empty() {
        return Err(invalid("segs", "no segments".into()));
    }
    let edges = segs
        .iter()
        .enumerate()
        .map(|(i, s)| lower_seg(s).map_err(|r| invalid("segs", format!("segment {i}: {r}"))))
        .collect::<KResult<Vec<_>>>()?;
    for i in 0..segs.len() - 1 {
        let (_, end) = seg_ends(&segs[i]);
        let (start, _) = seg_ends(&segs[i + 1]);
        let gap = (start - end).length();
        if gap.is_nan() || gap > tol.linear {
            return Err(invalid(
                "segs",
                format!("gap of {gap:.3e} mm after segment {i}"),
            ));
        }
    }
    let (first, _) = seg_ends(&segs[0]);
    let (_, last) = seg_ends(&segs[segs.len() - 1]);
    let closed = (first - last).length() <= tol.linear;
    let mut loops = LoopsIn::default();
    loops.push_loop(edges);
    Ok((loops, closed))
}

#[cfg(test)]
mod tests;
