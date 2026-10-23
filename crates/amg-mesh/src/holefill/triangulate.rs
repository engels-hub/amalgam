//! Initial triangulations of a hole polygon: Liepa's minimum-weight dynamic program, a
//! constrained Delaunay triangulation for planar loops, and ear clipping for large loops.
//!
//! The polygon is given as positions `pos[0..n]` in the order that makes every new triangle
//! `(i, j, k)` with `i < j < k` counter-clockwise, i.e. the reverse of the boundary half-edge
//! order. `outer[i]` is the unit normal of the mesh face across the edge `i -> i + 1`
//! (`outer[n - 1]` belongs to the closing edge `n - 1 -> 0`); zero when that face is degenerate.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use amg_core::{Point2, Point3, Vec3};
use amg_jobs::JobCtx;
use spade::{ConstrainedDelaunayTriangulation, Triangulation};

use crate::MeshResult;

/// Chord predicate: `true` when the edge between two loop vertices already exists in the mesh.
pub(super) type Forbidden<'a> = &'a dyn Fn(usize, usize) -> bool;

/// Tolerance when comparing dihedral cosines (angles of about 1e-6 rad).
const COS_EPS: f64 = 1e-12;

/// Cosine of the angle between two unit normals; a zero normal (degenerate neighbour) is
/// neutral.
fn cos_between(a: Vec3, b: Vec3) -> f64 {
    if a == Vec3::ZERO || b == Vec3::ZERO {
        1.0
    } else {
        a.dot(b)
    }
}

/// Liepa's O(n^3) dynamic program: among all triangulations of the polygon, the one that
/// minimises (largest dihedral angle between adjacent triangles, including the surrounding
/// faces; then total area). Chords that already exist in the mesh are forbidden.
///
/// `None` when no triangulation avoids the forbidden chords.
pub(super) fn dp_triangulate(
    pos: &[Point3],
    outer: &[Vec3],
    forbidden: Forbidden<'_>,
    ctx: &JobCtx,
) -> MeshResult<Option<Vec<[u32; 3]>>> {
    let n = pos.len();
    debug_assert!(n >= 3 && outer.len() == n);
    let at = |i: usize, j: usize| i * n + j;
    // Worst (smallest) dihedral cosine and total area of the best sub-triangulation of i..=j.
    let mut worst = vec![1.0f64; n * n];
    let mut area = vec![0.0f64; n * n];
    let mut piv = vec![u32::MAX; n * n];
    let mut nrm = vec![Vec3::ZERO; n * n];
    for i in 0..n - 1 {
        nrm[at(i, i + 1)] = outer[i];
    }
    for span in 2..n {
        ctx.checkpoint()?;
        for i in 0..n - span {
            let j = i + span;
            let root = i == 0 && j == n - 1;
            if !root && forbidden(i, j) {
                area[at(i, j)] = f64::INFINITY;
                continue;
            }
            let mut best: Option<(f64, f64, usize, Vec3)> = None;
            for k in i + 1..j {
                let (a1, a2) = (area[at(i, k)], area[at(k, j)]);
                if !a1.is_finite() || !a2.is_finite() {
                    continue;
                }
                let c = (pos[k] - pos[i]).cross(pos[j] - pos[i]);
                let len = c.length();
                let scale = (pos[k] - pos[i])
                    .length_squared()
                    .max((pos[j] - pos[i]).length_squared());
                let (normal, cos_here) = if len > 1e-12 * scale && len > 0.0 {
                    let nn = c / len;
                    let mut cs = cos_between(nn, nrm[at(i, k)]).min(cos_between(nn, nrm[at(k, j)]));
                    if root {
                        cs = cs.min(cos_between(nn, outer[n - 1]));
                    }
                    (nn, cs)
                } else {
                    (Vec3::ZERO, -1.0)
                };
                let w = worst[at(i, k)].min(worst[at(k, j)]).min(cos_here);
                let a = a1 + a2 + 0.5 * len;
                let better = match best {
                    None => true,
                    Some((bw, ba, _, _)) => w > bw + COS_EPS || (w >= bw - COS_EPS && a < ba),
                };
                if better {
                    best = Some((w, a, k, normal));
                }
            }
            match best {
                Some((w, a, k, normal)) => {
                    worst[at(i, j)] = w;
                    area[at(i, j)] = a;
                    piv[at(i, j)] = k as u32;
                    nrm[at(i, j)] = normal;
                }
                None => area[at(i, j)] = f64::INFINITY,
            }
        }
    }
    if !area[at(0, n - 1)].is_finite() {
        return Ok(None);
    }
    let mut tris = Vec::with_capacity(n - 2);
    let mut stack = vec![(0usize, n - 1)];
    while let Some((i, j)) = stack.pop() {
        if j <= i + 1 {
            continue;
        }
        let k = piv[at(i, j)] as usize;
        tris.push([i as u32, k as u32, j as u32]);
        stack.push((i, k));
        stack.push((k, j));
    }
    tris.sort_unstable();
    Ok(Some(tris))
}

/// Newell normal (twice the signed area vector) of the closed polygon.
pub(super) fn newell(pos: &[Point3]) -> Vec3 {
    let mut s = Vec3::ZERO;
    for i in 0..pos.len() {
        s += pos[i].cross(pos[(i + 1) % pos.len()]);
    }
    s
}

/// Orthonormal basis `(u, v)` of the plane with unit normal `n`, `u x v = n`.
fn plane_basis(n: Vec3) -> (Vec3, Vec3) {
    let helper = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let u = helper.cross(n).normalize();
    (u, n.cross(u))
}

/// The polygon's best-fit plane when it is planar to within `rel_tol` of its extent:
/// `(centroid, unit normal)`.
pub(super) fn planar_frame(pos: &[Point3], rel_tol: f64) -> Option<(Point3, Vec3)> {
    let nrm = newell(pos);
    let len = nrm.length();
    if !(len > 0.0 && len.is_finite()) {
        return None;
    }
    let nrm = nrm / len;
    let c = pos.iter().copied().sum::<Point3>() / pos.len() as f64;
    let (mut dev, mut ext) = (0.0f64, 0.0f64);
    for p in pos {
        let d = *p - c;
        dev = dev.max(d.dot(nrm).abs());
        ext = ext.max(d.length());
    }
    (ext > 0.0 && dev <= rel_tol * ext).then_some((c, nrm))
}

/// Projects the polygon into its plane (2D coordinates in the basis `u`, `v`).
fn project(pos: &[Point3], c: Point3, n: Vec3) -> Vec<Point2> {
    let (u, v) = plane_basis(n);
    pos.iter()
        .map(|p| Point2::new((*p - c).dot(u), (*p - c).dot(v)))
        .collect()
}

fn winding_inside(poly: &[Point2], p: Point2) -> bool {
    let mut inside = false;
    let n = poly.len();
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (poly[i], poly[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Constrained Delaunay triangulation of a planar polygon (no Steiner points). `None` when the
/// polygon degenerates in 2D (coincident or crossing points) or a chord already exists.
pub(super) fn cdt_triangulate(
    pos: &[Point3],
    frame: (Point3, Vec3),
    forbidden: Forbidden<'_>,
) -> Option<Vec<[u32; 3]>> {
    let n = pos.len();
    let pts = project(pos, frame.0, frame.1);
    let mut cdt = ConstrainedDelaunayTriangulation::<spade::Point2<f64>>::new();
    let mut handles = Vec::with_capacity(n);
    for p in &pts {
        handles.push(cdt.insert(spade::Point2::new(p.x, p.y)).ok()?);
    }
    let mut seen: Vec<usize> = handles.iter().map(|h| h.index()).collect();
    seen.sort_unstable();
    seen.dedup();
    if seen.len() != n {
        return None;
    }
    for i in 0..n {
        let (a, b) = (handles[i], handles[(i + 1) % n]);
        if !cdt.can_add_constraint(a, b) {
            return None;
        }
        cdt.add_constraint(a, b);
    }
    let mut local = vec![u32::MAX; cdt.num_vertices()];
    for (i, h) in handles.iter().enumerate() {
        local[h.index()] = i as u32;
    }
    let mut tris = Vec::with_capacity(n - 2);
    for f in cdt.inner_faces() {
        let vs = f.vertices();
        let t = [
            local[vs[0].fix().index()],
            local[vs[1].fix().index()],
            local[vs[2].fix().index()],
        ];
        if t.contains(&u32::MAX) {
            return None; // a Steiner vertex from a split constraint
        }
        let (p0, p1, p2) = (pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]);
        let centroid = (p0 + p1 + p2) / 3.0;
        if !winding_inside(&pts, centroid) {
            continue;
        }
        // Spade returns counter-clockwise faces; the polygon is counter-clockwise too.
        let mut t = t;
        if (p1 - p0).perp_dot(p2 - p0) < 0.0 {
            t.swap(1, 2);
        }
        tris.push(t);
    }
    if tris.len() != n - 2 {
        return None;
    }
    // Every triangle edge between loop vertices that is not a polygon edge is a chord.
    for t in &tris {
        for k in 0..3 {
            let (a, b) = (t[k] as usize, t[(k + 1) % 3] as usize);
            let adjacent = (a + 1) % n == b || (b + 1) % n == a;
            if !adjacent && forbidden(a, b) {
                return None;
            }
        }
    }
    // Rotate each triangle to start at its smallest index for a stable order.
    for t in &mut tris {
        let m = (0..3).min_by_key(|&k| t[k]).unwrap_or(0);
        t.rotate_left(m);
    }
    tris.sort_unstable();
    Some(tris)
}

/// Ear clipping in the best-fit plane, always taking the sharpest valid ear first. Meant for
/// loops too long for the dynamic program. `None` if no valid ear remains (a loop that folds
/// over itself in projection).
pub(super) fn ear_clip(
    pos: &[Point3],
    frame: (Point3, Vec3),
    forbidden: Forbidden<'_>,
) -> Option<Vec<[u32; 3]>> {
    let n = pos.len();
    let pts = project(pos, frame.0, frame.1);
    let mut prev: Vec<u32> = (0..n as u32)
        .map(|i| (i + n as u32 - 1) % n as u32)
        .collect();
    let mut next: Vec<u32> = (0..n as u32).map(|i| (i + 1) % n as u32).collect();
    let mut alive = vec![true; n];
    let mut version = vec![0u32; n];
    let mut remaining = n;
    let angle_of = |p: usize, v: usize, x: usize| -> Option<f64> {
        let (a, b) = (pts[p] - pts[v], pts[x] - pts[v]);
        let ang = b.perp_dot(a).atan2(b.dot(a));
        (ang > 1e-12 && ang < std::f64::consts::PI - 1e-12).then_some(ang)
    };
    let mut heap: BinaryHeap<Reverse<(u64, u32, u32)>> = BinaryHeap::new();
    let push = |heap: &mut BinaryHeap<_>, v: usize, prev: &[u32], next: &[u32], version: &[u32]| {
        if let Some(a) = angle_of(prev[v] as usize, v, next[v] as usize) {
            heap.push(Reverse((a.to_bits(), v as u32, version[v])));
        }
    };
    for v in 0..n {
        push(&mut heap, v, &prev, &next, &version);
    }
    let mut tris = Vec::with_capacity(n - 2);
    let mut rebuilt = false;
    while remaining > 3 {
        let Some(Reverse((_, v, ver))) = heap.pop() else {
            if rebuilt {
                return None;
            }
            rebuilt = true;
            for v in 0..n {
                if alive[v] {
                    version[v] += 1;
                    push(&mut heap, v, &prev, &next, &version);
                }
            }
            continue;
        };
        let v = v as usize;
        if !alive[v] || version[v] != ver {
            continue;
        }
        let (p, x) = (prev[v] as usize, next[v] as usize);
        if forbidden(p, x) {
            continue;
        }
        // No other remaining vertex may lie in the ear.
        let (a, b, c) = (pts[p], pts[v], pts[x]);
        let mut blocked = false;
        let mut w = next[x] as usize;
        while w != p {
            let q = pts[w];
            let (d1, d2, d3) = (
                (b - a).perp_dot(q - a),
                (c - b).perp_dot(q - b),
                (a - c).perp_dot(q - c),
            );
            if d1 >= 0.0 && d2 >= 0.0 && d3 >= 0.0 {
                blocked = true;
                break;
            }
            w = next[w] as usize;
        }
        if blocked {
            continue;
        }
        tris.push([p as u32, v as u32, x as u32]);
        alive[v] = false;
        remaining -= 1;
        next[p] = x as u32;
        prev[x] = p as u32;
        for u in [p, x] {
            version[u] += 1;
            push(&mut heap, u, &prev, &next, &version);
        }
        rebuilt = false;
    }
    let first = (0..n).find(|&i| alive[i])?;
    tris.push([prev[first], first as u32, next[first]]);
    // Normalise: start at the smallest index (cyclic rotation keeps the orientation).
    for t in &mut tris {
        let m = (0..3).min_by_key(|&k| t[k]).unwrap_or(0);
        t.rotate_left(m);
    }
    tris.sort_unstable();
    Some(tris)
}

/// Fan from a new centre vertex (index `n`) to every boundary edge. Always valid.
pub(super) fn centroid_fan(n: usize) -> Vec<[u32; 3]> {
    (0..n)
        .map(|i| [i as u32, ((i + 1) % n) as u32, n as u32])
        .collect()
}

/// Checks a triangulation of an `n`-gon (plus `extra` interior vertices): every polygon edge
/// `i -> i + 1` occurs once, every other directed edge at most once and with its twin.
pub(super) fn is_valid_patch(tris: &[[u32; 3]], n: usize, extra: usize) -> bool {
    use std::collections::HashMap;
    let total = n + extra;
    let mut edges: HashMap<(u32, u32), u32> = HashMap::with_capacity(tris.len() * 3);
    for t in tris {
        if t.iter().any(|&v| v as usize >= total) || t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
            return false;
        }
        for k in 0..3 {
            if edges.insert((t[k], t[(k + 1) % 3]), 1).is_some() {
                return false;
            }
        }
    }
    for &(a, b) in edges.keys() {
        let is_loop = (a as usize) < n && (b as usize) < n && (a as usize + 1) % n == b as usize;
        if is_loop {
            if edges.contains_key(&(b, a)) {
                return false;
            }
        } else if !edges.contains_key(&(b, a)) {
            return false;
        }
    }
    (0..n).all(|i| edges.contains_key(&(i as u32, ((i + 1) % n) as u32)))
}
