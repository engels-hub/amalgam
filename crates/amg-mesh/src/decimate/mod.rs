//! QEM decimation (MESH-07): Garland & Heckbert 1997 edge collapse with optimal placement.
//!
//! Boundary edges, group borders, feature edges and UV seams are protected by penalty
//! quadrics on the planes through the edge perpendicular to the adjacent face (weight
//! [`DecimateOpts::boundary_weight`]) and by combinatorial rules: an unconstrained edge
//! between two constrained vertices is never collapsed, and a constrained vertex absorbs a
//! free one without moving. A collapse is rejected when it violates the link condition,
//! flips a face normal, or drops a triangle below [`DecimateOpts::min_triangle_quality`].
//!
//! The queue is sequential with the tie-break `(cost, min vertex id, max vertex id)`, and the
//! only parallel stages (sort, initial costs) are pure, so the result is bit-identical across
//! rayon thread counts.

mod quadric;
mod queue;

use std::collections::HashSet;
use std::hash::{BuildHasherDefault, Hasher};

use amg_core::Point3;
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::{IndexedMesh, MeshError, MeshResult};
use quadric::Quadric;
use queue::{Best, VertexQueue};

/// What [`decimate`] aims for.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DecimateTarget {
    /// Stop at this many faces (at least 4 remain; the result may be one face below).
    FaceCount(usize),
    /// Stop at `ratio x` the input face count, `0 < ratio <= 1`.
    Ratio(f64),
    /// Collapse while the error stays below this many mm (see [`DecimateOpts::max_error`]).
    MaxError(f64),
}

/// Options for [`decimate`].
#[derive(Clone, Debug)]
pub struct DecimateOpts {
    /// Goal.
    pub target: DecimateTarget,
    /// Hard cap in mm on the area-weighted RMS distance of a collapse, combined with `target`.
    pub max_error: Option<f64>,
    /// Keep open boundaries in place (penalty quadrics, weight `boundary_weight`).
    pub preserve_boundary: bool,
    /// Keep borders between face groups in place.
    pub preserve_group_borders: bool,
    /// Edges whose dihedral angle exceeds this (radians) are protected; `None` = no feature
    /// edges.
    pub feature_angle: Option<f64>,
    /// Penalty-quadric weight for protected edges, default 1e3.
    pub boundary_weight: f64,
    /// Reject collapses that create a triangle below this quality (`4 sqrt(3) A / sum l^2`,
    /// in 0..1) and worse than before; default 0.1.
    pub min_triangle_quality: f64,
    /// Keep vertex colours and corner UVs (UV seams are protected like group borders).
    /// Vertex normals are always dropped (stale). Face groups and 3MF properties are always
    /// kept.
    pub preserve_attributes: bool,
}

impl Default for DecimateOpts {
    fn default() -> Self {
        DecimateOpts {
            target: DecimateTarget::Ratio(0.5),
            max_error: None,
            preserve_boundary: true,
            preserve_group_borders: true,
            feature_angle: Some(45f64.to_radians()),
            boundary_weight: 1e3,
            min_triangle_quality: 0.1,
            preserve_attributes: true,
        }
    }
}

/// Outcome of [`decimate`].
#[derive(Clone, Debug)]
pub struct DecimateReport {
    /// Input face count.
    pub faces_before: usize,
    /// Output face count.
    pub faces_after: usize,
    /// Largest area-weighted RMS plane distance (mm) over the accepted collapses.
    pub max_quadric_error: f64,
    /// Attribute names that were dropped.
    pub dropped_attributes: Vec<&'static str>,
}

#[derive(Default)]
struct IdHasher(u64);

impl Hasher for IdHasher {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
        }
    }
    fn write_u64(&mut self, v: u64) {
        self.0 = (v ^ (v >> 29)).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }
}

type EdgeSet = HashSet<u64, BuildHasherDefault<IdHasher>>;

fn key(a: u32, b: u32) -> u64 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    (u64::from(lo) << 32) | u64::from(hi)
}

#[derive(Default)]
struct Scratch {
    na: Vec<u32>,
    nb: Vec<u32>,
}

struct State {
    pos: Vec<Point3>,
    tris: Vec<[u32; 3]>,
    face_alive: Vec<bool>,
    vf: Vec<Vec<u32>>,
    q: Vec<Quadric>,
    cons: Vec<bool>,
    bnd: Vec<bool>,
    locked: Vec<bool>,
    max_err2: f64,
    moved: Vec<bool>,
    cons_edges: EdgeSet,
    min_quality: f64,
}

/// Cost marker of an edge that must not be collapsed (also an impossible `f32` bit pattern).
const NO_COST: u32 = u32::MAX;

fn sorted_neighbors(vf: &[Vec<u32>], tris: &[[u32; 3]], v: u32, out: &mut Vec<u32>) {
    out.clear();
    for &f in &vf[v as usize] {
        out.extend(tris[f as usize].iter().copied().filter(|&x| x != v));
    }
    out.sort_unstable();
    out.dedup();
}

/// Cheapest candidate of `v` from its costed neighbour list (ties: smallest `(lo, hi)`).
fn best_of_list(v: u32, list: &[(u32, u32)]) -> Best {
    let mut best = Best::NONE;
    for &(n, c) in list {
        if c != NO_COST {
            let cand = Best::new(c, v, n);
            if best.is_none() || cand.sort_key() < best.sort_key() {
                best = cand;
            }
        }
    }
    best
}

fn quality(n_len: f64, l2_sum: f64) -> f64 {
    if l2_sum <= 0.0 {
        0.0
    } else {
        2.0 * 3f64.sqrt() * n_len / l2_sum
    }
}

fn tri_normal(a: Point3, b: Point3, c: Point3) -> (Point3, f64) {
    let n = (b - a).cross(c - a);
    let l2 = (b - a).length_squared() + (c - b).length_squared() + (a - c).length_squared();
    (n, l2)
}

impl State {
    fn plan(&self, a: u32, b: u32) -> Option<(f64, Point3)> {
        let (ia, ib) = (a as usize, b as usize);
        if self.locked[ia] || self.locked[ib] {
            return None;
        }
        let (ca, cb) = (self.cons[ia], self.cons[ib]);
        if ca && cb && !self.cons_edges.contains(&key(a, b)) {
            return None;
        }
        let q = self.q[ia].sum(&self.q[ib]);
        let (pa, pb) = (self.pos[ia], self.pos[ib]);
        let mut best: Option<(f64, Point3)> = None;
        let mut consider = |p: Point3| {
            let c = q.eval(p);
            if c.is_finite() && best.is_none_or(|(bc, _)| c < bc) {
                best = Some((c, p));
            }
        };
        if ca && !cb {
            consider(pa);
        } else if cb && !ca {
            consider(pb);
        } else {
            let mid = (pa + pb) * 0.5;
            if let Some(o) = q.optimum()
                && (o - mid).length() <= (pb - pa).length()
            {
                consider(o);
            }
            consider(pa);
            consider(pb);
            consider(mid);
        }
        best
    }

    /// Queue cost (f32 bits) of collapsing `(a, b)`; `NO_COST` if it is not a candidate.
    fn cand(&self, a: u32, b: u32) -> u32 {
        let (a, b) = if a < b { (a, b) } else { (b, a) };
        let Some((cost, _)) = self.plan(a, b) else {
            return NO_COST;
        };
        let w = self.q[a as usize].w + self.q[b as usize].w;
        if cost > self.max_err2 * w {
            return NO_COST;
        }
        (cost as f32).to_bits()
    }

    /// Costed neighbour list of `v`. `buf` is scratch.
    fn costed_neighbors(&self, v: u32, buf: &mut Vec<u32>) -> Vec<(u32, u32)> {
        sorted_neighbors(&self.vf, &self.tris, v, buf);
        buf.iter().map(|&n| (n, self.cand(v, n))).collect()
    }

    /// Flip and quality test of face `f` when its vertex `v` moves to `p`.
    fn face_ok(&self, f: u32, v: u32, p: Point3) -> bool {
        let t = self.tris[f as usize];
        let old = t.map(|i| self.pos[i as usize]);
        let new = [0, 1, 2].map(|k| if t[k] == v { p } else { old[k] });
        let (no, l2o) = tri_normal(old[0], old[1], old[2]);
        let (nn, l2n) = tri_normal(new[0], new[1], new[2]);
        let (lo, ln) = (no.length(), nn.length());
        if lo <= 0.0 {
            return true;
        }
        if ln <= 1e-9 * lo || no.dot(nn) < 0.05 * lo * ln {
            return false;
        }
        let qn = quality(ln, l2n);
        qn >= self.min_quality || qn >= quality(lo, l2o)
    }

    /// Link condition + geometric checks. Returns the number of faces the collapse removes.
    fn legal(&self, a: u32, b: u32, p: Point3, sc: &mut Scratch) -> Option<usize> {
        let (ia, ib) = (a as usize, b as usize);
        if self.bnd[ia] && self.bnd[ib] && !self.is_boundary_edge(a, b) {
            return None;
        }
        sc.na.clear();
        sc.nb.clear();
        let mut shared = 0usize;
        for &f in &self.vf[ia] {
            let t = self.tris[f as usize];
            if t.contains(&b) {
                shared += 1;
            } else if !self.face_ok(f, a, p) {
                return None;
            }
            sc.na
                .extend(t.iter().copied().filter(|&v| v != a && v != b));
        }
        for &f in &self.vf[ib] {
            let t = self.tris[f as usize];
            if !t.contains(&a) && !self.face_ok(f, b, p) {
                return None;
            }
            sc.nb
                .extend(t.iter().copied().filter(|&v| v != a && v != b));
        }
        if shared == 0 || shared > 2 {
            return None;
        }
        sc.na.sort_unstable();
        sc.na.dedup();
        sc.nb.sort_unstable();
        sc.nb.dedup();
        let (mut i, mut j, mut common) = (0, 0, 0usize);
        while i < sc.na.len() && j < sc.nb.len() {
            match sc.na[i].cmp(&sc.nb[j]) {
                std::cmp::Ordering::Less => i += 1,
                std::cmp::Ordering::Greater => j += 1,
                std::cmp::Ordering::Equal => {
                    common += 1;
                    i += 1;
                    j += 1;
                }
            }
        }
        (common == shared).then_some(shared)
    }

    fn is_boundary_edge(&self, a: u32, b: u32) -> bool {
        self.vf[a as usize]
            .iter()
            .filter(|&&f| self.tris[f as usize].contains(&b))
            .count()
            == 1
    }

    /// Collapses `b` into `a` at `p`.
    fn apply(&mut self, a: u32, b: u32, p: Point3) {
        let (ia, ib) = (a as usize, b as usize);
        let fb = std::mem::take(&mut self.vf[ib]);
        // Constrained edges of b become edges of a.
        if self.cons[ib] {
            let mut nbrs = Vec::new();
            for &f in &fb {
                nbrs.extend(self.tris[f as usize].iter().copied().filter(|&v| v != b));
            }
            for n in nbrs {
                if n != a && self.cons_edges.remove(&key(b, n)) {
                    self.cons_edges.insert(key(a, n));
                }
            }
            self.cons_edges.remove(&key(a, b));
        }
        for &f in &fb {
            let t = self.tris[f as usize];
            if t.contains(&a) {
                self.face_alive[f as usize] = false;
                for v in t {
                    if v != b {
                        self.vf[v as usize].retain(|&x| x != f);
                    }
                }
            } else {
                for c in &mut self.tris[f as usize] {
                    if *c == b {
                        *c = a;
                    }
                }
                self.vf[ia].push(f);
            }
        }
        self.pos[ia] = p;
        self.moved[ia] = true;
        let qb = self.q[ib];
        self.q[ia].add(&qb);
        self.cons[ia] |= self.cons[ib];
        self.bnd[ia] |= self.bnd[ib];
    }
}

/// Reduces `m`. See the module docs and [`DecimateOpts`].
///
/// # Errors
/// [`MeshError::Empty`] without faces, [`MeshError::InvalidOption`] for a non-finite or
/// out-of-range option, [`MeshError::Cancelled`] when `ctx` is cancelled.
#[allow(clippy::too_many_lines)]
pub fn decimate(
    m: &IndexedMesh,
    opts: &DecimateOpts,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, DecimateReport)> {
    let nf = m.tris.len();
    let nv = m.positions.len();
    if nf == 0 {
        return Err(MeshError::Empty);
    }
    let bad = |name: &'static str, reason: &str| MeshError::InvalidOption {
        name,
        reason: reason.to_string(),
    };
    let (target_faces, mut max_error) = match opts.target {
        DecimateTarget::FaceCount(n) => (n.max(4), f64::INFINITY),
        DecimateTarget::Ratio(r) => {
            if !(r > 0.0 && r.is_finite()) {
                return Err(bad("target", "ratio must be finite and > 0"));
            }
            (((nf as f64 * r).ceil() as usize).max(4), f64::INFINITY)
        }
        DecimateTarget::MaxError(e) => {
            if !(e >= 0.0 && e.is_finite()) {
                return Err(bad("target", "max error must be finite and >= 0"));
            }
            (4, e)
        }
    };
    if let Some(e) = opts.max_error {
        if !(e >= 0.0 && e.is_finite()) {
            return Err(bad("max_error", "must be finite and >= 0"));
        }
        max_error = max_error.min(e);
    }
    if !(opts.boundary_weight >= 0.0 && opts.boundary_weight.is_finite()) {
        return Err(bad("boundary_weight", "must be finite and >= 0"));
    }
    if !opts.min_triangle_quality.is_finite() {
        return Err(bad("min_triangle_quality", "must be finite"));
    }
    let max_err2 = max_error * max_error;

    let mut dropped: Vec<&'static str> = Vec::new();
    let mut out_attrs = m.attrs.clone();
    if out_attrs.vertex_normals.take().is_some() {
        dropped.push("vertex_normals");
    }
    if !opts.preserve_attributes {
        if out_attrs.vertex_colors.take().is_some() {
            dropped.push("vertex_colors");
        }
        if out_attrs.corner_uvs.take().is_some() {
            dropped.push("corner_uvs");
        }
    }

    if nf <= target_faces {
        let mut out = m.clone();
        out.attrs = out_attrs;
        return Ok((
            out,
            DecimateReport {
                faces_before: nf,
                faces_after: nf,
                max_quadric_error: 0.0,
                dropped_attributes: dropped,
            },
        ));
    }

    // Work relative to the bounding-box centre: quadric evaluation loses digits far from 0.
    let bb = m.aabb();
    let center = (bb.min + bb.max) * 0.5;
    let pos: Vec<Point3> = m.positions.par_iter().map(|&p| p - center).collect();

    let mut locked = vec![false; nv];
    let mut he: Vec<(u64, u32)> = Vec::with_capacity(3 * nf);
    for (f, t) in m.tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if a == b {
                locked[a as usize] = true;
            } else {
                he.push((key(a, b), f as u32));
            }
        }
    }
    he.par_sort_unstable();

    let normals: Vec<(Point3, f64)> = m
        .tris
        .par_iter()
        .map(|t| {
            let n = (pos[t[1] as usize] - pos[t[0] as usize])
                .cross(pos[t[2] as usize] - pos[t[0] as usize]);
            let l = n.length();
            if l > 0.0 {
                (n / l, l * 0.5)
            } else {
                (Point3::ZERO, 0.0)
            }
        })
        .collect();

    let mut vf: Vec<Vec<u32>> = {
        let mut deg = vec![0u32; nv];
        for t in &m.tris {
            for &v in t {
                deg[v as usize] += 1;
            }
        }
        deg.iter()
            .map(|&d| Vec::with_capacity(d as usize + 2))
            .collect()
    };
    for (f, t) in m.tris.iter().enumerate() {
        for &v in t {
            vf[v as usize].push(f as u32);
        }
    }
    // Per-vertex sums in ascending face order: parallel, yet independent of the thread count.
    let mut q: Vec<Quadric> = vf
        .par_iter()
        .map(|fs| {
            let mut acc = Quadric::default();
            for &f in fs {
                let (n, area) = normals[f as usize];
                if area > 0.0 {
                    let d = -n.dot(pos[m.tris[f as usize][0] as usize]);
                    let mut fq = Quadric::plane(n, d, area);
                    fq.w = area;
                    acc.add(&fq);
                }
            }
            acc
        })
        .collect();

    let cos_feature = opts.feature_angle.map(f64::cos);
    let uvs = if opts.preserve_attributes {
        m.attrs.corner_uvs.as_deref()
    } else {
        None
    };
    let mut cons = vec![false; nv];
    let mut bnd = vec![false; nv];
    let mut cons_edges = EdgeSet::default();
    let mut i = 0;
    while i < he.len() {
        let k = he[i].0;
        let mut j = i + 1;
        while j < he.len() && he[j].0 == k {
            j += 1;
        }
        let (a, b) = ((k >> 32) as u32, k as u32);
        let run = j - i;
        let faces = [he[i].1, he[(i + 1).min(j - 1)].1];
        let mut protect = false;
        let mut sides = 0;
        if run > 2 {
            locked[a as usize] = true;
            locked[b as usize] = true;
        } else if run == 1 {
            bnd[a as usize] = true;
            bnd[b as usize] = true;
            protect = opts.preserve_boundary;
            sides = 1;
        } else {
            sides = 2;
            let [f1, f2] = faces;
            if opts.preserve_group_borders
                && m.groups.group_of(crate::FaceId(f1)) != m.groups.group_of(crate::FaceId(f2))
            {
                protect = true;
            }
            if let Some(c) = cos_feature
                && normals[f1 as usize].0.dot(normals[f2 as usize].0) < c
            {
                protect = true;
            }
            if let Some(uv) = uvs {
                let corner = |f: u32, v: u32| {
                    let p = m.tris[f as usize].iter().position(|&x| x == v);
                    p.map(|p| uv[f as usize][p])
                };
                if corner(f1, a) != corner(f2, a) || corner(f1, b) != corner(f2, b) {
                    protect = true;
                }
            }
        }
        // Protected edges get the full penalty; unprotected boundary edges a soft one.
        if sides > 0 && (protect || run == 1) {
            let (pa, pb) = (pos[a as usize], pos[b as usize]);
            let e = pb - pa;
            let len = e.length();
            if len > 0.0 {
                let dir = e / len;
                let w = if protect { opts.boundary_weight } else { 1.0 } * len * len;
                for &f in &faces[..sides] {
                    let mp = dir.cross(normals[f as usize].0);
                    let ml = mp.length();
                    if ml > 0.0 {
                        let mp = mp / ml;
                        let pq = Quadric::plane(mp, -mp.dot(pa), w);
                        q[a as usize].add(&pq);
                        q[b as usize].add(&pq);
                    }
                }
            }
        }
        if protect {
            cons[a as usize] = true;
            cons[b as usize] = true;
            cons_edges.insert(k);
        }
        i = j;
    }
    drop(he);

    let mut st = State {
        pos,
        tris: m.tris.clone(),
        face_alive: vec![true; nf],
        vf,
        q,
        cons,
        bnd,
        locked,
        max_err2,
        moved: vec![false; nv],
        cons_edges,
        min_quality: opts.min_triangle_quality,
    };

    let mut adj: Vec<Vec<(u32, u32)>> = (0..nv as u32)
        .into_par_iter()
        .map_init(Vec::new, |buf, v| st.costed_neighbors(v, buf))
        .collect();
    let best: Vec<Best> = adj
        .par_iter()
        .enumerate()
        .map(|(v, l)| best_of_list(v as u32, l))
        .collect();
    let mut queue = VertexQueue::build(best);

    let mut alive = nf;
    let mut sc = Scratch::default();
    let mut ring: Vec<u32> = Vec::new();
    let mut max_seen = 0.0f64;
    let span = (nf - target_faces).max(1) as f32;
    let mut iter = 0u32;
    while alive > target_faces {
        let Some((_, top)) = queue.peek() else { break };
        iter = iter.wrapping_add(1);
        if iter.is_multiple_of(2048) {
            ctx.checkpoint()?;
            ctx.progress((nf - alive) as f32 / span);
        }
        let (a, b) = (top.lo, top.hi);
        let planned = st.plan(a, b);
        let removed = planned.and_then(|(_, p)| st.legal(a, b, p, &mut sc));
        let (Some((cost, p)), Some(removed)) = (planned, removed.filter(|r| alive - r >= 4)) else {
            // Rejected until an endpoint changes (its edges are re-costed then).
            for (x, y) in [(a, b), (b, a)] {
                if let Some(e) = adj[x as usize].iter_mut().find(|e| e.0 == y) {
                    e.1 = NO_COST;
                }
                let nb = best_of_list(x, &adj[x as usize]);
                queue.set(x, nb);
            }
            continue;
        };
        let w = st.q[a as usize].w + st.q[b as usize].w;
        if w > 0.0 {
            max_seen = max_seen.max((cost / w).sqrt());
        }
        st.apply(a, b, p);
        alive -= removed;
        queue.remove(b);
        adj[b as usize] = Vec::new();

        sorted_neighbors(&st.vf, &st.tris, a, &mut ring);
        let list_a: Vec<(u32, u32)> = ring.iter().map(|&n| (n, st.cand(a, n))).collect();
        for &(n, c) in &list_a {
            let l = &mut adj[n as usize];
            if let Some(i) = l.iter().position(|e| e.0 == b) {
                l.swap_remove(i);
            }
            match l.iter_mut().find(|e| e.0 == a) {
                Some(e) => e.1 = c,
                None => l.push((a, c)),
            }
            let nb = best_of_list(n, l);
            if nb != queue.best(n) {
                queue.set(n, nb);
            }
        }
        let nb = best_of_list(a, &list_a);
        adj[a as usize] = list_a;
        queue.set(a, nb);
    }
    ctx.checkpoint()?;
    ctx.progress(1.0);

    // Assemble the output.
    let keep_f: Vec<bool> = st.face_alive.clone();
    let mut keep_v = vec![false; nv];
    for (t, _) in st.tris.iter().zip(&keep_f).filter(|(_, k)| **k) {
        for &v in t {
            keep_v[v as usize] = true;
        }
    }
    let mut map = vec![crate::INVALID; nv];
    let mut positions = Vec::with_capacity(alive / 2 + 2);
    for v in 0..nv {
        if keep_v[v] {
            map[v] = positions.len() as u32;
            positions.push(if st.moved[v] {
                st.pos[v] + center
            } else {
                m.positions[v]
            });
        }
    }
    let tris: Vec<[u32; 3]> = st
        .tris
        .iter()
        .zip(&keep_f)
        .filter(|(_, k)| **k)
        .map(|(t, _)| t.map(|v| map[v as usize]))
        .collect();
    out_attrs.retain(&keep_v, &keep_f);
    let mut groups = m.groups.clone();
    groups.retain(&keep_f);
    let out = IndexedMesh {
        positions,
        tris,
        attrs: out_attrs,
        groups,
    };
    let faces_after = out.tris.len();
    Ok((
        out,
        DecimateReport {
            faces_before: nf,
            faces_after,
            max_quadric_error: max_seen,
            dropped_attributes: dropped,
        },
    ))
}
