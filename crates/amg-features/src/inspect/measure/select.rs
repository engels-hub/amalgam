//! Smart selection (note §2.3) and snapping (S05 §2).

use std::collections::VecDeque;
use std::sync::Arc;

use amg_core::intersect::ray_triangle;
use amg_core::{Dir3, Point3, Ray3, Segment3, Transform, Triangle3, Vec3};
use amg_jobs::JobCtx;
use amg_mesh::{FaceId, IndexedMesh};

use super::{MeshElement, RegionOpts, SnapOpts, Snapped};
use crate::inspect::InspectError;

/// Faces are processed between cancellation checks.
const CANCEL_EVERY: usize = 1 << 14;

impl Default for RegionOpts {
    /// 0.5°, 1e-3 mm, no group borders, 5 M faces (note §2.3).
    fn default() -> Self {
        RegionOpts {
            max_normal_angle: 0.5f64.to_radians(),
            planar_tolerance: 1e-3,
            stop_at_group_borders: false,
            max_faces: 5_000_000,
        }
    }
}

/// Vertex → incident faces in CSR form, faces ascending per vertex.
struct VertexFaces {
    offsets: Vec<u32>,
    faces: Vec<u32>,
}

impl VertexFaces {
    fn build(m: &IndexedMesh) -> Self {
        let mut offsets = vec![0u32; m.vertex_count() + 1];
        for t in &m.tris {
            for &v in t {
                offsets[v as usize + 1] += 1;
            }
        }
        for i in 0..m.vertex_count() {
            offsets[i + 1] += offsets[i];
        }
        let mut fill = offsets.clone();
        let mut faces = vec![0u32; m.tris.len() * 3];
        for (f, t) in m.tris.iter().enumerate() {
            for &v in t {
                let slot = &mut fill[v as usize];
                faces[*slot as usize] = f as u32;
                *slot += 1;
            }
        }
        VertexFaces { offsets, faces }
    }

    fn of(&self, v: u32) -> &[u32] {
        &self.faces[self.offsets[v as usize] as usize..self.offsets[v as usize + 1] as usize]
    }

    /// Faces using both `u` and `v` (ascending).
    fn of_edge<'a>(&'a self, m: &'a IndexedMesh, u: u32, v: u32) -> impl Iterator<Item = u32> + 'a {
        let tris = &m.tris;
        self.of(u)
            .iter()
            .copied()
            .filter(move |&f| tris[f as usize].contains(&v))
            .scan(None, |last, f| {
                // A face listed twice for a repeated vertex is counted once.
                let dup = *last == Some(f);
                *last = Some(f);
                Some((f, dup))
            })
            .filter_map(|(f, dup)| (!dup).then_some(f))
    }
}

fn unit_normal(m: &IndexedMesh, f: u32) -> Option<Vec3> {
    let [a, b, c] = m.triangle(FaceId(f));
    let n = (b - a).cross(c - a);
    let l = n.length();
    (l > 0.0 && l.is_finite()).then(|| n / l)
}

fn angle(u: Vec3, v: Vec3) -> f64 {
    u.cross(v).length().atan2(u.dot(v))
}

/// Grows a planar region from `seed` (note §2.3): breadth-first over manifold edges in
/// ascending face order, adding a neighbour iff its normal is within `max_normal_angle` of the
/// seed normal and all its vertices are within `planar_tolerance` of the seed plane (through
/// the seed's centroid). Optionally stops at face-group borders; capped at `max_faces`. The
/// result is sorted.
///
/// # Errors
/// [`InspectError::BadElement`] for a seed out of range, [`InspectError::Degenerate`] for a
/// zero-area seed, [`InspectError::Cancelled`].
pub fn grow_planar_region(
    m: &IndexedMesh,
    seed: FaceId,
    opts: &RegionOpts,
    ctx: &JobCtx,
) -> Result<Arc<[FaceId]>, InspectError> {
    if seed.0 as usize >= m.face_count() {
        return Err(InspectError::BadElement(format!("face {}", seed.0)));
    }
    let n0 = unit_normal(m, seed.0).ok_or(InspectError::Degenerate("the seed face has no area"))?;
    let [a, b, c] = m.triangle(seed);
    let o = (a + b + c) / 3.0;
    let group = m.groups.group_of(seed);
    let vf = VertexFaces::build(m);
    ctx.checkpoint()?;
    let mut seen = vec![false; m.face_count()];
    seen[seed.0 as usize] = true;
    let mut out = vec![seed.0];
    let mut queue = VecDeque::from([seed.0]);
    let mut nbrs = Vec::with_capacity(3);
    let accept = |f: u32| {
        if opts.stop_at_group_borders && m.groups.group_of(FaceId(f)) != group {
            return false;
        }
        let Some(n) = unit_normal(m, f) else {
            return false;
        };
        angle(n0, n) <= opts.max_normal_angle
            && m.triangle(FaceId(f))
                .iter()
                .all(|p| n0.dot(*p - o).abs() <= opts.planar_tolerance)
    };
    let max = opts.max_faces.max(1);
    'bfs: while let Some(f) = queue.pop_front() {
        if out.len() % CANCEL_EVERY == 0 {
            ctx.checkpoint()?;
        }
        let t = m.tris[f as usize];
        nbrs.clear();
        for (u, v) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let mut it = vf.of_edge(m, u, v).filter(|&g| g != f);
            if let (Some(g), None) = (it.next(), it.next()) {
                nbrs.push(g);
            }
        }
        nbrs.sort_unstable();
        for &g in &nbrs {
            if seen[g as usize] {
                continue;
            }
            seen[g as usize] = true;
            if accept(g) {
                if out.len() >= max {
                    break 'bfs;
                }
                out.push(g);
                queue.push_back(g);
            }
        }
    }
    out.sort_unstable();
    Ok(out.into_iter().map(FaceId).collect())
}

/// Grows the feature-edge chain through the picked edge `a–b` (note §2.3). A feature edge is a
/// boundary edge, a non-manifold edge, or one whose dihedral angle (between face normals) is
/// ≥ `feature_angle`. The walk continues while the current vertex has exactly one other
/// incident feature edge; the chain is closed when it returns to its start. The result keeps
/// the direction `a → b`. A picked edge that is not a feature edge gives the 2-vertex open
/// polyline `[a, b]`.
///
/// # Errors
/// [`InspectError::BadElement`] if `a–b` is not an edge of the mesh,
/// [`InspectError::Cancelled`].
pub fn grow_feature_chain(
    m: &IndexedMesh,
    a: u32,
    b: u32,
    feature_angle: f64,
    ctx: &JobCtx,
) -> Result<MeshElement, InspectError> {
    let nv = m.vertex_count() as u32;
    if a >= nv || b >= nv || a == b {
        return Err(InspectError::BadElement(format!("edge {a}–{b}")));
    }
    let vf = VertexFaces::build(m);
    ctx.checkpoint()?;
    if vf.of_edge(m, a, b).next().is_none() {
        return Err(InspectError::BadElement(format!("edge {a}–{b}")));
    }
    let is_feature = |u: u32, v: u32| {
        let faces: Vec<u32> = vf.of_edge(m, u, v).collect();
        match faces.as_slice() {
            [_, _] => match (unit_normal(m, faces[0]), unit_normal(m, faces[1])) {
                (Some(n0), Some(n1)) => angle(n0, n1) >= feature_angle,
                _ => true,
            },
            _ => !faces.is_empty(),
        }
    };
    let single = |v: Vec<u32>| MeshElement::Polyline {
        verts: v.into(),
        closed: false,
    };
    if !is_feature(a, b) {
        return Ok(single(vec![a, b]));
    }
    // The next vertex after `prev → cur`, if `cur` has exactly one other feature edge.
    let next = |prev: u32, cur: u32| -> Option<u32> {
        let mut others: Vec<u32> = vf
            .of(cur)
            .iter()
            .flat_map(|&f| m.tris[f as usize])
            .filter(|&w| w != cur && w != prev)
            .collect();
        others.sort_unstable();
        others.dedup();
        let mut feat = others.into_iter().filter(|&w| is_feature(cur, w));
        match (feat.next(), feat.next()) {
            (Some(w), None) => Some(w),
            _ => None,
        }
    };
    let limit = m.vertex_count() + 1;
    let mut fwd = vec![a, b];
    let mut closed = false;
    let (mut prev, mut cur) = (a, b);
    while let Some(w) = next(prev, cur) {
        if fwd.len() % CANCEL_EVERY == 0 {
            ctx.checkpoint()?;
        }
        if w == a {
            closed = true;
            break;
        }
        if fwd.contains(&w) || fwd.len() > limit {
            break; // a lasso: stop before revisiting
        }
        fwd.push(w);
        (prev, cur) = (cur, w);
    }
    if !closed {
        let mut back = Vec::new();
        let (mut prev, mut cur) = (b, a);
        while let Some(w) = next(prev, cur) {
            if back.len() % CANCEL_EVERY == 0 {
                ctx.checkpoint()?;
            }
            if fwd.contains(&w) || back.contains(&w) || back.len() > limit {
                break;
            }
            back.push(w);
            (prev, cur) = (cur, w);
        }
        back.reverse();
        back.extend(fwd);
        fwd = back;
    }
    Ok(MeshElement::Polyline {
        verts: fwd.into(),
        closed,
    })
}

/// Snaps a pick on `face` (S05 §2): intersects the world `ray` with the face in body-local
/// f64 (a ray that just misses the triangle uses the closest triangle point to its plane
/// crossing), then snaps to the nearest face vertex within `vertex_radius` mm, else to the
/// nearest face edge within `edge_radius` mm, else keeps the surface point. Returns the snap
/// and its world position; `None` if the face is out of range, `to_world` is singular, or
/// the ray is parallel to the face plane and misses it.
pub fn snap_on_face(
    m: &IndexedMesh,
    to_world: &Transform,
    face: FaceId,
    ray: &Ray3,
    opts: &SnapOpts,
) -> Option<(Snapped, Point3)> {
    if face.0 as usize >= m.face_count() {
        return None;
    }
    let inv = to_world.inverse();
    if !inv.is_finite() {
        return None;
    }
    let origin = inv.transform_point3(ray.origin);
    let dir = Dir3::new(
        inv.transform_vector3(ray.dir.get()),
        &amg_core::Tolerance::DEFAULT,
    )?;
    let local = Ray3 { origin, dir };
    let corners = m.triangle(face);
    let tri = Triangle3 {
        a: corners[0],
        b: corners[1],
        c: corners[2],
    };
    let hit = match ray_triangle(&local, &tri) {
        Some(h) => tri.a * h.bary[0] + tri.b * h.bary[1] + tri.c * h.bary[2],
        None => {
            let n = (tri.b - tri.a).cross(tri.c - tri.a);
            let denom = n.dot(dir.get());
            if denom == 0.0 || !denom.is_finite() {
                return None;
            }
            let t = n.dot(tri.a - origin) / denom;
            let p = local.at(t.max(0.0));
            amg_core::intersect::point_triangle_closest(p, &tri)
        }
    };
    let v = m.tris[face.0 as usize];
    let world = |p: Point3| to_world.transform_point3(p);
    // Vertex snap: nearest corner (lowest corner on ties).
    let (k, dv) = (0..3)
        .map(|k| (k, corners[k].distance(hit)))
        .fold((0, f64::INFINITY), |b, x| if x.1 < b.1 { x } else { b });
    if dv <= opts.vertex_radius {
        return Some((Snapped::Vertex(v[k]), world(corners[k])));
    }
    let mut best: Option<(usize, f64, f64)> = None;
    for (e, (i, j)) in [(0usize, 1usize), (1, 2), (2, 0)].into_iter().enumerate() {
        let (t, q) = Segment3 {
            a: corners[i],
            b: corners[j],
        }
        .closest_point(hit);
        let d = q.distance(hit);
        if best.is_none_or(|b| d < b.2) {
            best = Some((e, t, d));
        }
    }
    if let Some((e, t, d)) = best
        && d <= opts.edge_radius
    {
        let (i, j) = [(0usize, 1usize), (1, 2), (2, 0)][e];
        let p = corners[i] + (corners[j] - corners[i]) * t;
        return Some((
            Snapped::Edge {
                a: v[i],
                b: v[j],
                t,
            },
            world(p),
        ));
    }

    let bary = barycentric(&corners, hit);
    Some((Snapped::Face { face, bary }, world(hit)))
}

/// Barycentric weights of `p` (on the triangle's plane) by sub-triangle areas.
fn barycentric(t: &[Point3; 3], p: Point3) -> [f64; 3] {
    let n = (t[1] - t[0]).cross(t[2] - t[0]);
    let n2 = n.length_squared();
    if n2.is_nan() || n2 <= 0.0 {
        return [1.0, 0.0, 0.0];
    }
    let w = |a: Point3, b: Point3| (b - a).cross(p - a).dot(n) / n2;
    let u = w(t[1], t[2]);
    let v = w(t[2], t[0]);
    [u, v, 1.0 - u - v]
}
