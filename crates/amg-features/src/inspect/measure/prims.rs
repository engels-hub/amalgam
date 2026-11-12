//! Exact distances between points, segments and triangles (note §2.2), with the closest
//! features of each side.

use amg_core::intersect::{ray_triangle, segment_segment};
use amg_core::{Dir3, Point3, Ray3, Segment3, Transform, Triangle3};

/// Which part of a primitive a witness point lies on, by local corner index.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PrimFeature {
    /// Corner `i`: 0 for a point, 0 or 1 for a segment, 0 to 2 for a triangle.
    Corner(u8),
    /// The edge from corner `i` to corner `j` (a segment's interior is `Edge(0, 1)`).
    Edge(u8, u8),
    /// The interior of a triangle.
    Interior,
}

/// A point, segment or triangle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Prim {
    /// A point.
    Point(Point3),
    /// A segment.
    Seg([Point3; 2]),
    /// A triangle.
    Tri([Point3; 3]),
}

impl Prim {
    /// `t` applied to every corner.
    pub fn transformed(&self, t: &Transform) -> Prim {
        match *self {
            Prim::Point(p) => Prim::Point(t.transform_point3(p)),
            Prim::Seg(s) => Prim::Seg(s.map(|p| t.transform_point3(p))),
            Prim::Tri(c) => Prim::Tri(c.map(|p| t.transform_point3(p))),
        }
    }

    /// Bounds `(min, max)`.
    pub fn bounds(&self) -> (Point3, Point3) {
        match self {
            Prim::Point(p) => (*p, *p),
            Prim::Seg([a, b]) => (a.min(*b), a.max(*b)),
            Prim::Tri([a, b, c]) => (a.min(*b).min(*c), a.max(*b).max(*c)),
        }
    }
}

/// Closest pair between two primitives.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrimDistance {
    /// Squared distance, mm².
    pub dist_sq: f64,
    /// Point on the first primitive.
    pub a: Point3,
    /// Point on the second primitive.
    pub b: Point3,
    /// Feature of the first primitive holding `a`.
    pub fa: PrimFeature,
    /// Feature of the second primitive holding `b`.
    pub fb: PrimFeature,
}

impl PrimDistance {
    fn new(a: Point3, b: Point3, fa: PrimFeature, fb: PrimFeature) -> Self {
        PrimDistance {
            dist_sq: a.distance_squared(b),
            a,
            b,
            fa,
            fb,
        }
    }

    fn swapped(self) -> Self {
        PrimDistance {
            dist_sq: self.dist_sq,
            a: self.b,
            b: self.a,
            fa: self.fb,
            fb: self.fa,
        }
    }
}

/// Keeps the first of the strictly closest candidates.
#[derive(Default)]
struct Best(Option<PrimDistance>);

impl Best {
    fn keep(&mut self, d: PrimDistance) {
        if self.0.is_none_or(|b| d.dist_sq < b.dist_sq) {
            self.0 = Some(d);
        }
    }
}

/// Exact closest points of two primitives. Ties between feature pairs go to the first pair in
/// a fixed order, so the result is deterministic.
pub fn prim_distance(a: &Prim, b: &Prim) -> PrimDistance {
    match (a, b) {
        (Prim::Point(p), Prim::Point(q)) => {
            PrimDistance::new(*p, *q, PrimFeature::Corner(0), PrimFeature::Corner(0))
        }
        (Prim::Point(p), Prim::Seg(s)) => {
            let (q, f) = closest_on_segment(*p, s);
            PrimDistance::new(*p, q, PrimFeature::Corner(0), f)
        }
        (Prim::Point(p), Prim::Tri(t)) => {
            let (q, f) = closest_on_triangle(*p, t);
            PrimDistance::new(*p, q, PrimFeature::Corner(0), f)
        }
        (Prim::Seg(s), Prim::Seg(t)) => seg_seg(s, t),
        (Prim::Seg(s), Prim::Tri(t)) => seg_tri(s, t),
        (Prim::Tri(s), Prim::Tri(t)) => tri_tri(s, t),
        (Prim::Seg(_) | Prim::Tri(_), Prim::Point(_)) | (Prim::Tri(_), Prim::Seg(_)) => {
            prim_distance(b, a).swapped()
        }
    }
}

const EDGES: [(u8, u8); 3] = [(0, 1), (1, 2), (2, 0)];

#[inline]
fn corner<const N: usize>(p: &[Point3; N], i: u8) -> Point3 {
    p[usize::from(i)]
}

/// Exact triangle–triangle distance (note §2.2): an edge crossing the other triangle gives
/// distance 0; otherwise the minimum over 6 vertex–triangle and 9 edge–edge pairs.
pub fn tri_tri(s: &[Point3; 3], t: &[Point3; 3]) -> PrimDistance {
    for &(i, j) in &EDGES {
        if let Some(p) = edge_crosses(corner(s, i), corner(s, j), t) {
            return PrimDistance::new(p, p, PrimFeature::Edge(i, j), PrimFeature::Interior);
        }
    }
    for &(i, j) in &EDGES {
        if let Some(p) = edge_crosses(corner(t, i), corner(t, j), s) {
            return PrimDistance::new(p, p, PrimFeature::Interior, PrimFeature::Edge(i, j));
        }
    }
    let mut best = Best::default();
    for i in 0..3u8 {
        let p = corner(s, i);
        let (q, f) = closest_on_triangle(p, t);
        best.keep(PrimDistance::new(p, q, PrimFeature::Corner(i), f));
    }
    for i in 0..3u8 {
        let p = corner(t, i);
        let (q, f) = closest_on_triangle(p, s);
        best.keep(PrimDistance::new(q, p, f, PrimFeature::Corner(i)));
    }
    for &(i, j) in &EDGES {
        for &(k, l) in &EDGES {
            let d = seg_seg(&[corner(s, i), corner(s, j)], &[corner(t, k), corner(t, l)]);
            best.keep(PrimDistance {
                fa: lift_edge(d.fa, i, j),
                fb: lift_edge(d.fb, k, l),
                ..d
            });
        }
    }
    best.0.unwrap_or_else(|| {
        PrimDistance::new(s[0], t[0], PrimFeature::Corner(0), PrimFeature::Corner(0))
    })
}

/// Segment–triangle distance: 0 if the segment crosses the triangle, else the minimum over the
/// 2 endpoint–triangle and 3 segment–edge pairs.
fn seg_tri(s: &[Point3; 2], t: &[Point3; 3]) -> PrimDistance {
    if let Some(p) = edge_crosses(s[0], s[1], t) {
        return PrimDistance::new(p, p, PrimFeature::Edge(0, 1), PrimFeature::Interior);
    }
    let mut best = Best::default();
    for i in 0..2u8 {
        let p = corner(s, i);
        let (q, f) = closest_on_triangle(p, t);
        best.keep(PrimDistance::new(p, q, PrimFeature::Corner(i), f));
    }
    for &(k, l) in &EDGES {
        let d = seg_seg(s, &[corner(t, k), corner(t, l)]);
        best.keep(PrimDistance {
            fb: lift_edge(d.fb, k, l),
            ..d
        });
    }
    best.0.unwrap_or_else(|| {
        PrimDistance::new(s[0], t[0], PrimFeature::Corner(0), PrimFeature::Corner(0))
    })
}

/// A segment feature (`Corner(0 | 1)`, `Edge(0, 1)`) mapped onto triangle edge `(i, j)`.
fn lift_edge(f: PrimFeature, i: u8, j: u8) -> PrimFeature {
    match f {
        PrimFeature::Corner(0) => PrimFeature::Corner(i),
        PrimFeature::Corner(_) => PrimFeature::Corner(j),
        _ => PrimFeature::Edge(i, j),
    }
}

#[allow(
    clippy::float_cmp,
    reason = "segment parameters are clamped to exactly 0 and 1"
)]
fn seg_feature(s: f64) -> PrimFeature {
    if s == 0.0 {
        PrimFeature::Corner(0)
    } else if s == 1.0 {
        PrimFeature::Corner(1)
    } else {
        PrimFeature::Edge(0, 1)
    }
}

fn seg_seg(s: &[Point3; 2], t: &[Point3; 2]) -> PrimDistance {
    let (u, v, p, q) = segment_segment(
        &Segment3 { a: s[0], b: s[1] },
        &Segment3 { a: t[0], b: t[1] },
    );
    PrimDistance::new(p, q, seg_feature(u), seg_feature(v))
}

fn closest_on_segment(p: Point3, s: &[Point3; 2]) -> (Point3, PrimFeature) {
    let (t, q) = Segment3 { a: s[0], b: s[1] }.closest_point(p);
    (q, seg_feature(t))
}

/// Point where segment `a → b` crosses triangle `t` (`amg_core::intersect::ray_triangle`
/// with `t ∈ [0, |b − a|]`), as the barycentric point on the triangle.
fn edge_crosses(a: Point3, b: Point3, t: &[Point3; 3]) -> Option<Point3> {
    let d = b - a;
    let len = d.length();
    if len.is_nan() || len <= 0.0 || !len.is_finite() {
        return None;
    }
    let ray = Ray3 {
        origin: a,
        dir: Dir3::new_unchecked(d / len),
    };
    let tri = Triangle3 {
        a: t[0],
        b: t[1],
        c: t[2],
    };
    let hit = ray_triangle(&ray, &tri)?;
    if hit.t > len {
        return None;
    }
    Some(t[0] * hit.bary[0] + t[1] * hit.bary[1] + t[2] * hit.bary[2])
}

/// Relative height below which a triangle counts as a sliver (as in
/// `amg_core::intersect::point_triangle_closest`).
const SLIVER: f64 = 1e-8;

/// Closest point of the filled triangle to `p`, with the feature it lies on (Ericson,
/// *Real-Time Collision Detection* §5.1.5). Slivers use their edges.
pub fn closest_on_triangle(p: Point3, t: &[Point3; 3]) -> (Point3, PrimFeature) {
    let [a, b, c] = *t;
    let ab = b - a;
    let ac = c - a;
    let max_edge2 = ab
        .length_squared()
        .max(ac.length_squared())
        .max((c - b).length_squared());
    let n2 = ab.cross(ac).length_squared();
    if n2.is_nan() || n2 <= SLIVER * SLIVER * max_edge2 * max_edge2 {
        return closest_on_edges(p, t);
    }
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, PrimFeature::Corner(0));
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, PrimFeature::Corner(1));
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (a + ab * v, PrimFeature::Edge(0, 1));
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, PrimFeature::Corner(2));
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (a + ac * w, PrimFeature::Edge(2, 0));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (b + (c - b) * w, PrimFeature::Edge(1, 2));
    }
    let sum = va + vb + vc;
    if sum > 0.0 && sum.is_finite() {
        let v = vb / sum;
        let w = vc / sum;
        return (a + ab * v + ac * w, PrimFeature::Interior);
    }
    closest_on_edges(p, t)
}

fn closest_on_edges(p: Point3, t: &[Point3; 3]) -> (Point3, PrimFeature) {
    let mut best = (t[0], PrimFeature::Corner(0));
    let mut best_d = f64::INFINITY;
    for &(i, j) in &EDGES {
        let (q, f) = closest_on_segment(p, &[corner(t, i), corner(t, j)]);
        let d = q.distance_squared(p);
        if d < best_d {
            best_d = d;
            best = (q, lift_edge(f, i, j));
        }
    }
    best
}
