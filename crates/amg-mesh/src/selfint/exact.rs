//! Exact closed triangle-triangle intersection built on Shewchuk's adaptive predicates
//! (`robust::orient3d` / `robust::orient2d`). No predicate result is ever approximated: every
//! decision below is the sign of an exactly evaluated determinant.
//!
//! Triangles are closed sets. Pairs that share vertices *by index* only count as intersecting
//! when they meet somewhere other than the shared vertices / edge (see [`pair_intersects`]).

use amg_core::Point3;
use robust::{Coord, Coord3D, orient2d, orient3d};

/// Sign of an exact predicate value.
#[inline]
fn sign(v: f64) -> i8 {
    if v > 0.0 {
        1
    } else if v < 0.0 {
        -1
    } else {
        0
    }
}

#[inline]
fn c3(p: Point3) -> Coord3D<f64> {
    Coord3D {
        x: p.x,
        y: p.y,
        z: p.z,
    }
}

/// Exact orientation of `d` against the plane `abc` (sign convention of `robust`).
#[inline]
fn o3(a: Point3, b: Point3, c: Point3, d: Point3) -> i8 {
    sign(orient3d(c3(a), c3(b), c3(c), c3(d)))
}

/// Projects onto the plane that drops axis `drop` (0, 1 or 2).
#[inline]
fn pr(p: Point3, drop: u8) -> Coord<f64> {
    match drop {
        0 => Coord { x: p.y, y: p.z },
        1 => Coord { x: p.z, y: p.x },
        _ => Coord { x: p.x, y: p.y },
    }
}

/// Exact 2D orientation of the projections.
#[inline]
fn o2(a: Point3, b: Point3, c: Point3, drop: u8) -> i8 {
    sign(orient2d(pr(a, drop), pr(b, drop), pr(c, drop)))
}

/// Axis flag of a degenerate (zero-area) triangle.
pub(crate) const DEGENERATE: u8 = 3;

/// Projection axis for a triangle: the dropped axis whose projected triangle has non-zero
/// exact area (the one with the largest approximate normal component first), or
/// [`DEGENERATE`] when the three points are exactly collinear.
pub(crate) fn drop_axis(a: Point3, b: Point3, c: Point3) -> u8 {
    let n = (b - a).cross(c - a).abs();
    let mut order = [0u8, 1, 2];
    if n.y >= n.x && n.y >= n.z {
        order = [1, 0, 2];
    } else if n.z >= n.x && n.z >= n.y {
        order = [2, 0, 1];
    }
    for d in order {
        if o2(a, b, c, d) != 0 {
            return d;
        }
    }
    DEGENERATE
}

/// A non-degenerate triangle with its projection axis.
#[derive(Clone, Copy)]
pub(crate) struct Tri {
    pub v: [Point3; 3],
    pub drop: u8,
}

/// Closed point-in-triangle test in the projection (exact).
fn point_in_tri_2d(p: Point3, t: &Tri) -> bool {
    let [a, b, c] = t.v;
    let orient = o2(a, b, c, t.drop);
    let s1 = o2(a, b, p, t.drop) * orient;
    let s2 = o2(b, c, p, t.drop) * orient;
    let s3 = o2(c, a, p, t.drop) * orient;
    s1 >= 0 && s2 >= 0 && s3 >= 0
}

/// `r` is collinear with `p`, `q`; is it within the closed segment (projected)?
fn on_segment_2d(p: Point3, q: Point3, r: Point3, drop: u8) -> bool {
    let (p, q, r) = (pr(p, drop), pr(q, drop), pr(r, drop));
    r.x >= p.x.min(q.x) && r.x <= p.x.max(q.x) && r.y >= p.y.min(q.y) && r.y <= p.y.max(q.y)
}

/// Closed segment-segment test in the projection (exact).
fn seg_seg_2d(p: Point3, q: Point3, a: Point3, b: Point3, drop: u8) -> bool {
    let d1 = o2(p, q, a, drop);
    let d2 = o2(p, q, b, drop);
    let d3 = o2(a, b, p, drop);
    let d4 = o2(a, b, q, drop);
    if d1 * d2 < 0 && d3 * d4 < 0 {
        return true;
    }
    (d1 == 0 && on_segment_2d(p, q, a, drop))
        || (d2 == 0 && on_segment_2d(p, q, b, drop))
        || (d3 == 0 && on_segment_2d(a, b, p, drop))
        || (d4 == 0 && on_segment_2d(a, b, q, drop))
}

/// Closed segment against a triangle that lies in the segment's plane.
fn coplanar_seg_tri(p: Point3, q: Point3, t: &Tri) -> bool {
    if point_in_tri_2d(p, t) || point_in_tri_2d(q, t) {
        return true;
    }
    let [a, b, c] = t.v;
    seg_seg_2d(p, q, a, b, t.drop)
        || seg_seg_2d(p, q, b, c, t.drop)
        || seg_seg_2d(p, q, c, a, t.drop)
}

/// Closed segment `pq` against closed triangle `t`, given the exact plane sides `sp`, `sq` of
/// the endpoints.
fn seg_tri(p: Point3, q: Point3, sp: i8, sq: i8, t: &Tri) -> bool {
    if sp * sq > 0 {
        return false;
    }
    if sp == 0 && sq == 0 {
        return coplanar_seg_tri(p, q, t);
    }
    // The line crosses the plane in one point; it is inside the closed triangle iff the three
    // edge side-products do not have strictly opposite signs.
    let [a, b, c] = t.v;
    let s1 = o3(p, q, a, b);
    let s2 = o3(p, q, b, c);
    let s3 = o3(p, q, c, a);
    !((s1 > 0 || s2 > 0 || s3 > 0) && (s1 < 0 || s2 < 0 || s3 < 0))
}

/// Exact plane sides of the three vertices of `x` against triangle `t`.
#[inline]
fn sides(x: &Tri, t: &Tri) -> [i8; 3] {
    [
        o3(t.v[0], t.v[1], t.v[2], x.v[0]),
        o3(t.v[0], t.v[1], t.v[2], x.v[1]),
        o3(t.v[0], t.v[1], t.v[2], x.v[2]),
    ]
}

#[inline]
fn all_same_side(s: [i8; 3]) -> bool {
    (s[0] > 0 && s[1] > 0 && s[2] > 0) || (s[0] < 0 && s[1] < 0 && s[2] < 0)
}

/// Do the closed triangles `a` and `b` intersect (no shared vertices)?
fn disjoint_indices(a: &Tri, b: &Tri) -> bool {
    let sa = sides(a, b); // a's vertices against b's plane
    if all_same_side(sa) {
        return false;
    }
    let sb = sides(b, a);
    if all_same_side(sb) {
        return false;
    }
    // Non-coplanar: the intersection is a segment whose ends lie on edges of a or b; the
    // coplanar case reduces to the same six segment tests.
    for k in 0..3 {
        let k1 = (k + 1) % 3;
        if seg_tri(a.v[k], a.v[k1], sa[k], sa[k1], b) || seg_tri(b.v[k], b.v[k1], sb[k], sb[k1], a)
        {
            return true;
        }
    }
    false
}

/// Do two triangles sharing exactly the vertex `a.v[i] == b.v[j]` touch anywhere else?
fn share_vertex(a: &Tri, i: usize, b: &Tri, j: usize) -> bool {
    // The intersection is convex and contains the shared vertex; if it contains another point
    // then the shorter of the two rays from the vertex ends on the opposite edge of its own
    // triangle and inside the other triangle.
    let (a1, a2) = (a.v[(i + 1) % 3], a.v[(i + 2) % 3]);
    let (b1, b2) = (b.v[(j + 1) % 3], b.v[(j + 2) % 3]);
    let pl = |t: &Tri, x: Point3| o3(t.v[0], t.v[1], t.v[2], x);
    let (sa1, sa2) = (pl(b, a1), pl(b, a2));
    if seg_tri(a1, a2, sa1, sa2, b) {
        return true;
    }
    let (sb1, sb2) = (pl(a, b1), pl(a, b2));
    seg_tri(b1, b2, sb1, sb2, a)
}

/// Do two triangles sharing the edge `a.v[i] a.v[i+1]` overlap beyond it? Only possible when
/// coplanar with the third vertices on the same side of the edge.
fn share_edge(a: &Tri, i: usize, b: &Tri, j: usize) -> bool {
    let (u, v) = (a.v[i], a.v[(i + 1) % 3]);
    let c = a.v[(i + 2) % 3];
    // `b`'s edge may run in the opposite direction; its third vertex is the one not shared.
    let d = b.v[(j + 2) % 3];
    if o3(a.v[0], a.v[1], a.v[2], d) != 0 {
        return false;
    }
    let sc = o2(u, v, c, a.drop);
    let sd = o2(u, v, d, a.drop);
    sc == sd
}

/// Exact intersection test for two non-degenerate triangles of an indexed mesh.
///
/// Triangles are closed point sets. `ia` / `ib` are the vertex indices; vertices are shared
/// *by index* (two vertices at the same position but with different indices are not shared, so
/// an unwelded mesh reports its seams). Result by number of shared vertices:
/// - 0: closed triangles share a point;
/// - 1: they share a point other than the shared vertex;
/// - 2: they are coplanar and overlap with positive area (a fold-over);
/// - 3: always `true` (a duplicate face).
pub(crate) fn pair_intersects(ta: &Tri, ia: [u32; 3], tb: &Tri, ib: [u32; 3]) -> bool {
    let mut shared = [(0usize, 0usize); 3];
    let mut n = 0;
    for (i, x) in ia.iter().enumerate() {
        for (j, y) in ib.iter().enumerate() {
            if x == y {
                shared[n] = (i, j);
                n += 1;
            }
        }
    }
    match n {
        0 => disjoint_indices(ta, tb),
        1 => share_vertex(ta, shared[0].0, tb, shared[0].1),
        2 => {
            // The shared edge of `a` runs between the two shared corners; find its start.
            let (i0, i1) = (shared[0].0, shared[1].0);
            let i = if (i0 + 1) % 3 == i1 { i0 } else { i1 };
            let (j0, j1) = (shared[0].1, shared[1].1);
            let j = if (j0 + 1) % 3 == j1 { j0 } else { j1 };
            share_edge(ta, i, tb, j)
        }
        _ => true,
    }
}
