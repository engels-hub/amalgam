//! Split, collapse and flip passes.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

use amg_core::Point3;
use amg_jobs::JobCtx;
use rayon::prelude::*;

use super::{State, key};
use crate::{FaceId, HalfEdgeId, MeshResult, VertexId};

/// Checkpoint interval in edits.
const CHECK_EVERY: usize = 4096;
/// Largest turn (cos of the angle between the two curve directions) a vertex of a constrained
/// curve may have and still slide: 45 degrees.
const CURVE_COS: f64 = -std::f64::consts::FRAC_1_SQRT_2;
/// A triangle may rotate by at most `acos(MIN_NORMAL_COS)` when a vertex moves or an edge flips.
const MIN_NORMAL_COS: f64 = 0.25;

fn unit_dot(a: Point3, b: Point3) -> f64 {
    let (la, lb) = (a.length(), b.length());
    if la > 0.0 && lb > 0.0 {
        a.dot(b) / (la * lb)
    } else {
        -1.0
    }
}

fn corner_angle(p: Point3, q: Point3, r: Point3) -> f64 {
    let (u, v) = ((q - p).normalize_or_zero(), (r - p).normalize_or_zero());
    u.dot(v).clamp(-1.0, 1.0).acos()
}

fn min_angle(a: Point3, b: Point3, c: Point3) -> f64 {
    corner_angle(a, b, c)
        .min(corner_angle(b, c, a))
        .min(corner_angle(c, a, b))
}

impl State {
    pub(super) fn pos(&self, v: u32) -> Point3 {
        self.he.position(VertexId(v))
    }

    fn find(&self, a: u32, b: u32) -> Option<HalfEdgeId> {
        self.he.find_halfedge(VertexId(a), VertexId(b))
    }

    /// The two constrained neighbours of a vertex with exactly two constrained edges.
    pub(super) fn cons_neighbors(&self, v: u32) -> Option<[u32; 2]> {
        if self.ncons[v as usize] != 2 {
            return None;
        }
        let mut out = [u32::MAX; 2];
        let mut n = 0;
        for w in self.he.neighbors(VertexId(v)) {
            if self.cons.contains_key(&key(v, w.0)) {
                if n < 2 {
                    out[n] = w.0;
                }
                n += 1;
            }
        }
        (n == 2).then_some(out)
    }

    /// The neighbours of a vertex that slides along a gently turning constrained curve.
    pub(super) fn curve(&self, v: u32) -> Option<[u32; 2]> {
        if self.fixed[v as usize] {
            return None;
        }
        let [a, b] = self.cons_neighbors(v)?;
        let p = self.pos(v);
        let (d1, d2) = (
            (self.pos(a) - p).normalize_or_zero(),
            (self.pos(b) - p).normalize_or_zero(),
        );
        (d1 != Point3::ZERO && d2 != Point3::ZERO && d1.dot(d2) <= CURVE_COS).then_some([a, b])
    }

    /// Both faces of `h` are inside the selection.
    fn splittable(&self, h: HalfEdgeId) -> bool {
        self.face_selected(self.he.face(h))
            && self
                .he
                .twin(h)
                .is_none_or(|t| self.face_selected(self.he.face(t)))
    }

    /// Squared split threshold (4/3 of the local target) of the edge `h`.
    fn high2_of(&self, h: HalfEdgeId) -> f64 {
        self.hi2(self.he.origin(h).0, self.he.target(h).0)
    }

    /// Squared split threshold of the edge between vertices `a` and `b`.
    pub(super) fn hi2(&self, a: u32, b: u32) -> f64 {
        let h = 4.0 / 3.0 * self.vs[a as usize].min(self.vs[b as usize]);
        h * h
    }

    /// Squared collapse threshold (4/5 of the local target) of the edge between `a` and `b`.
    fn lo2(&self, a: u32, b: u32) -> f64 {
        let l = 0.8 * self.vs[a as usize].min(self.vs[b as usize]);
        l * l
    }

    fn edge_len2(&self, e: HalfEdgeId) -> f64 {
        (self.pos(self.he.target(e).0) - self.pos(self.he.origin(e).0)).length_squared()
    }

    /// `e` is too long and will stay so: it borders the frozen part of the mesh, or (up to
    /// `depth` levels) a triangle whose longer edge is itself stuck.
    fn stuck(&self, e: HalfEdgeId, depth: u32) -> bool {
        self.edge_len2(e) > self.high2_of(e)
            && (!self.splittable(e) || (depth > 0 && self.split_blocked(e, depth - 1)))
    }

    /// A triangle next to `h` has a longer edge that cannot be split. Splitting `h` first
    /// would only bisect that triangle into ever thinner slivers (longest-edge-first rule).
    fn split_blocked(&self, h: HalfEdgeId, depth: u32) -> bool {
        if self.sel.is_none() {
            return false;
        }
        let l2 = self.edge_len2(h);
        let side = |e: HalfEdgeId| {
            [self.he.next(e), self.he.prev(e)]
                .into_iter()
                .any(|o| self.edge_len2(o) > l2 && self.stuck(o, depth))
        };
        side(h) || self.he.twin(h).is_some_and(side)
    }

    // ---- split ------------------------------------------------------------------------

    pub(super) fn split_pass(&mut self, ctx: &JobCtx) -> MeshResult<usize> {
        let he = &self.he;
        let cands: Vec<(u64, Reverse<u64>)> = (0..he.face_slots() as u32)
            .into_par_iter()
            .flat_map_iter(|f| (0..3).map(move |k| (FaceId(f), k)))
            .filter_map(|(f, k)| {
                if !he.is_face_alive(f) {
                    return None;
                }
                let h = he.halfedge(f, k);
                if he.twin(h).is_some_and(|t| t.0 < h.0) {
                    return None;
                }
                let (a, b) = (he.origin(h), he.target(h));
                let l2 = (he.position(b) - he.position(a)).length_squared();
                (l2 > self.hi2(a.0, b.0)).then(|| (l2.to_bits(), Reverse(key(a.0, b.0))))
            })
            .collect();
        let mut heap = BinaryHeap::from(cands);
        let mut done = 0usize;
        // Safety net; a pass never needs more than a few splits per initial long edge.
        let cap = 32 * (heap.len() + 1024);
        while let Some((_, Reverse(k))) = heap.pop() {
            if done >= cap {
                break;
            }
            let (a, b) = ((k >> 32) as u32, k as u32);
            if !self.he.is_vertex_alive(VertexId(a)) || !self.he.is_vertex_alive(VertexId(b)) {
                continue;
            }
            let Some(h) = self.find(a, b).or_else(|| self.find(b, a)) else {
                continue;
            };
            if !self.splittable(h) || self.split_blocked(h, 4) {
                continue;
            }
            let (pa, pb) = (self.pos(a), self.pos(b));
            if (pb - pa).length_squared() <= self.hi2(a, b) {
                continue;
            }
            let slots_before = self.he.face_slots();
            let Ok(m) = self.he.split_edge(h, (pa + pb) * 0.5) else {
                continue;
            };
            let m = m.0;
            if let Some(sel) = &mut self.sel {
                sel.resize(self.he.face_slots(), true);
            }
            debug_assert!(self.he.face_slots() > slots_before);
            self.ncons.push(0);
            self.fixed.push(false);
            self.vs
                .push(0.5 * (self.vs[a as usize] + self.vs[b as usize]));
            if let Some(kind) = self.cons.remove(&k) {
                self.cons.insert(key(a, m), kind);
                self.cons.insert(key(m, b), kind);
                self.ncons[m as usize] = 2;
            }
            let pm = self.pos(m);
            for w in self.he.neighbors(VertexId(m)) {
                let l2 = (self.pos(w.0) - pm).length_squared();
                if l2 > self.hi2(m, w.0) {
                    heap.push((l2.to_bits(), Reverse(key(m, w.0))));
                }
            }
            done += 1;
            if done.is_multiple_of(CHECK_EVERY) {
                ctx.checkpoint()?;
            }
        }
        ctx.checkpoint()?;
        Ok(done)
    }

    // ---- collapse ---------------------------------------------------------------------

    pub(super) fn collapse_pass(&mut self, ctx: &JobCtx) -> MeshResult<usize> {
        let he = &self.he;
        let mut cands: Vec<(u64, u64)> = (0..he.face_slots() as u32)
            .into_par_iter()
            .flat_map_iter(|f| (0..3).map(move |k| (FaceId(f), k)))
            .filter_map(|(f, k)| {
                if !he.is_face_alive(f) {
                    return None;
                }
                let h = he.halfedge(f, k);
                if he.twin(h).is_some_and(|t| t.0 < h.0) {
                    return None;
                }
                let (a, b) = (he.origin(h), he.target(h));
                let l2 = (he.position(b) - he.position(a)).length_squared();
                (l2 < self.lo2(a.0, b.0)).then(|| (l2.to_bits(), key(a.0, b.0)))
            })
            .collect();
        cands.par_sort_unstable();
        let mut done = 0usize;
        for (i, &(_, k)) in cands.iter().enumerate() {
            let (a, b) = ((k >> 32) as u32, k as u32);
            if !self.he.is_vertex_alive(VertexId(a)) || !self.he.is_vertex_alive(VertexId(b)) {
                continue;
            }
            if (self.pos(a) - self.pos(b)).length_squared() >= self.lo2(a, b) {
                continue;
            }
            let hab = self.find(a, b);
            let hba = self.find(b, a);
            for (x, y, h) in [(a, b, hab), (b, a, hba)] {
                if let Some(h) = h
                    && self.try_collapse(x, y, h)
                {
                    done += 1;
                    break;
                }
            }
            if i.is_multiple_of(CHECK_EVERY) {
                ctx.checkpoint()?;
            }
        }
        ctx.checkpoint()?;
        Ok(done)
    }

    /// Removes `x` into `y` along `h` (`origin(h) == x`) if every rule allows it.
    fn try_collapse(&mut self, x: u32, y: u32, h: HalfEdgeId) -> bool {
        if self.fixed[x as usize] {
            return false;
        }
        let edge_cons = self.cons.contains_key(&key(x, y));
        let nx = self.ncons[x as usize];
        let mut slide = None;
        if nx > 0 {
            // A curve vertex may only leave along its own curve.
            if !edge_cons {
                return false;
            }
            let Some([c1, c2]) = self.curve(x) else {
                return false;
            };
            let w = if c1 == y { c2 } else { c1 };
            if w == y || self.he.neighbors(VertexId(y)).iter().any(|n| n.0 == w) {
                return false;
            }
            slide = Some(w);
        } else if edge_cons {
            return false;
        }
        let (px, py) = (self.pos(x), self.pos(y));
        let p = if nx > 0 || self.ncons[y as usize] > 0 || self.fixed[y as usize] {
            py
        } else {
            (px + py) * 0.5
        };
        if !self.he.can_collapse(h) {
            return false;
        }
        for w in self.he.neighbors(VertexId(x)) {
            if w.0 != y && (self.pos(w.0) - p).length_squared() > self.hi2(w.0, y) {
                return false;
            }
        }
        // No flipped or collapsed triangle around x.
        for o in self.he.outgoing(VertexId(x)) {
            let (t1, t2) = (self.he.target(o), self.he.target(self.he.next(o)));
            if t1.0 == y || t2.0 == y {
                continue;
            }
            let (q1, q2) = (self.pos(t1.0), self.pos(t2.0));
            let old = (q1 - px).cross(q2 - px);
            let new = (q1 - p).cross(q2 - p);
            if unit_dot(old, new) < MIN_NORMAL_COS {
                return false;
            }
        }
        if self.he.collapse_edge(h, p).is_err() {
            return false;
        }
        if let Some(w) = slide {
            let kind = self
                .cons
                .remove(&key(x, w))
                .unwrap_or(super::features::HARD);
            self.cons.remove(&key(x, y));
            self.cons.insert(key(y, w), kind);
            self.ncons[x as usize] = 0;
        }
        true
    }

    // ---- flip -------------------------------------------------------------------------

    pub(super) fn flip_pass(&mut self, ctx: &JobCtx) -> MeshResult<usize> {
        let mut done = 0usize;
        for f in 0..self.he.face_slots() as u32 {
            let f = FaceId(f);
            if !self.he.is_face_alive(f) {
                continue;
            }
            for k in 0..3 {
                let h = self.he.halfedge(f, k);
                if let Some(t) = self.he.twin(h)
                    && t.0 > h.0
                    && self.try_flip(h, t)
                {
                    done += 1;
                }
            }
            if (f.0 as usize).is_multiple_of(CHECK_EVERY) {
                ctx.checkpoint()?;
            }
        }
        ctx.checkpoint()?;
        Ok(done)
    }

    fn valence_dev(&self, v: VertexId) -> i64 {
        let target = if self.he.is_boundary_vertex(v) { 4 } else { 6 };
        self.he.valence(v) as i64 - target
    }

    fn try_flip(&mut self, h: HalfEdgeId, t: HalfEdgeId) -> bool {
        let (f, g) = (self.he.face(h), self.he.face(t));
        if !self.face_selected(f) || !self.face_selected(g) {
            return false;
        }
        let (a, b) = (self.he.origin(h), self.he.target(h));
        if !self.cons.is_empty() && self.cons.contains_key(&key(a.0, b.0)) {
            return false;
        }
        let c = self.he.origin(self.he.prev(h));
        let d = self.he.origin(self.he.prev(t));
        let before: i64 = [a, b, c, d]
            .iter()
            .map(|&v| self.valence_dev(v).pow(2))
            .sum();
        let after = (self.valence_dev(a) - 1).pow(2)
            + (self.valence_dev(b) - 1).pow(2)
            + (self.valence_dev(c) + 1).pow(2)
            + (self.valence_dev(d) + 1).pow(2);
        if after >= before {
            return false;
        }
        let (pa, pb, pc, pd) = (self.pos(a.0), self.pos(b.0), self.pos(c.0), self.pos(d.0));
        // Old faces (a, b, c) and (b, a, d); new faces (c, a, d) and (d, b, c).
        let (nf, ng) = ((pb - pa).cross(pc - pa), (pa - pb).cross(pd - pb));
        let (n1, n2) = ((pa - pc).cross(pd - pc), (pb - pd).cross(pc - pd));
        for n in [n1, n2] {
            if unit_dot(n, nf) < MIN_NORMAL_COS || unit_dot(n, ng) < MIN_NORMAL_COS {
                return false;
            }
        }
        // Do not trade a good triangle pair for a clearly worse one.
        let old_min = min_angle(pa, pb, pc).min(min_angle(pb, pa, pd));
        let new_min = min_angle(pc, pa, pd).min(min_angle(pd, pb, pc));
        if new_min < 0.8 * old_min {
            return false;
        }
        self.he.flip_edge(h).is_ok()
    }
}
