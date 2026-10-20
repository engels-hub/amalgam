//! [`HalfEdgeMesh`]: compact *directed-edges* structure (Campagna, Kobbelt & Seidel 1998).
//!
//! - Face `f` owns half-edges `3f, 3f+1, 3f+2`; half-edge `3f+k` starts at corner `k` of the
//!   face and ends at corner `k+1`, so `next` / `prev` / `face` are index arithmetic.
//! - The only stored per-half-edge datum is `twin` (`INVALID` on a boundary). There are no
//!   explicit boundary half-edges: a boundary edge is a face half-edge without twin.
//! - Each live vertex stores one outgoing half-edge. On a boundary vertex it is the boundary
//!   one (twin-less), so the counter-clockwise one-ring walk starts at the right place.
//! - Every live vertex has a single fan of faces (the structure is a manifold with boundary).
//! - Deletions are tombstones; [`HalfEdgeMesh::garbage_collect`] compacts.
//!
//! Rotation around a vertex `v`: for an outgoing `h`, `twin(prev(h))` is the next outgoing
//! half-edge counter-clockwise, and `next(twin(h))` the next one clockwise.

mod build;
mod edit;

use amg_core::Point3;

use crate::attrs::MeshAttrs;
use crate::groups::FaceGroups;
use crate::indexed::{IndexedMesh, Remap};
use crate::{FaceId, HalfEdgeId, INVALID, VertexId};

pub use build::{ConversionReport, NonManifoldPolicy};

/// Half-edge mesh for topology-changing operations. Build it with
/// [`HalfEdgeMesh::from_indexed`]; convert back with [`HalfEdgeMesh::to_indexed`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HalfEdgeMesh {
    positions: Vec<Point3>,
    tris: Vec<[u32; 3]>,
    twin: Vec<u32>,
    vert_he: Vec<u32>,
    vert_alive: Vec<bool>,
    face_alive: Vec<bool>,
    attrs: MeshAttrs,
    groups: FaceGroups,
    dead_vertices: usize,
    dead_faces: usize,
}

#[inline]
fn next_raw(h: u32) -> u32 {
    if h % 3 == 2 { h - 2 } else { h + 1 }
}

#[inline]
fn prev_raw(h: u32) -> u32 {
    if h.is_multiple_of(3) { h + 2 } else { h - 1 }
}

impl HalfEdgeMesh {
    // ---- elements ---------------------------------------------------------------------

    /// Number of live vertices (including isolated ones).
    pub fn vertex_count(&self) -> usize {
        self.positions.len() - self.dead_vertices
    }

    /// Number of live faces.
    pub fn face_count(&self) -> usize {
        self.tris.len() - self.dead_faces
    }

    /// Vertex slots, including tombstones (valid ids are `0..vertex_slots()`).
    pub fn vertex_slots(&self) -> usize {
        self.positions.len()
    }

    /// Face slots, including tombstones.
    pub fn face_slots(&self) -> usize {
        self.tris.len()
    }

    /// Half-edge slots (`3 × face_slots()`).
    pub fn halfedge_slots(&self) -> usize {
        self.twin.len()
    }

    /// True if `v` exists and was not deleted.
    pub fn is_vertex_alive(&self, v: VertexId) -> bool {
        self.vert_alive.get(v.0 as usize).copied().unwrap_or(false)
    }

    /// True if `f` exists and was not deleted.
    pub fn is_face_alive(&self, f: FaceId) -> bool {
        self.face_alive.get(f.0 as usize).copied().unwrap_or(false)
    }

    /// Live vertex ids, ascending.
    pub fn vertices(&self) -> impl Iterator<Item = VertexId> + '_ {
        (0..self.positions.len() as u32)
            .filter(|&v| self.vert_alive[v as usize])
            .map(VertexId)
    }

    /// Live face ids, ascending.
    pub fn faces(&self) -> impl Iterator<Item = FaceId> + '_ {
        (0..self.tris.len() as u32)
            .filter(|&f| self.face_alive[f as usize])
            .map(FaceId)
    }

    /// Corner vertices of `f`, counter-clockwise.
    pub fn face_vertices(&self, f: FaceId) -> [VertexId; 3] {
        self.tris[f.0 as usize].map(VertexId)
    }

    /// Half-edge `k` (0..3) of face `f`; it starts at corner `k`.
    pub fn halfedge(&self, f: FaceId, k: u32) -> HalfEdgeId {
        debug_assert!(k < 3);
        HalfEdgeId(3 * f.0 + k)
    }

    /// The stored outgoing half-edge of `v` (the boundary one on a boundary vertex); `None`
    /// for an isolated or deleted vertex.
    pub fn vertex_halfedge(&self, v: VertexId) -> Option<HalfEdgeId> {
        let h = self.vert_he[v.0 as usize];
        (h != INVALID).then_some(HalfEdgeId(h))
    }

    /// Attributes (arrays indexed by vertex / face slot).
    pub fn attrs(&self) -> &MeshAttrs {
        &self.attrs
    }

    /// Face groups (indexed by face slot).
    pub fn groups(&self) -> &FaceGroups {
        &self.groups
    }

    /// Mutable face groups (to allocate groups for [`HalfEdgeMesh::add_face`]).
    pub fn groups_mut(&mut self) -> &mut FaceGroups {
        &mut self.groups
    }

    // ---- navigation -------------------------------------------------------------------

    /// Position of `v`.
    pub fn position(&self, v: VertexId) -> Point3 {
        self.positions[v.0 as usize]
    }

    /// Moves `v`.
    pub fn set_position(&mut self, v: VertexId, p: Point3) {
        self.positions[v.0 as usize] = p;
    }

    /// Next half-edge in the same face.
    pub fn next(&self, h: HalfEdgeId) -> HalfEdgeId {
        HalfEdgeId(next_raw(h.0))
    }

    /// Previous half-edge in the same face.
    pub fn prev(&self, h: HalfEdgeId) -> HalfEdgeId {
        HalfEdgeId(prev_raw(h.0))
    }

    /// Opposite half-edge; `None` on a boundary edge.
    pub fn twin(&self, h: HalfEdgeId) -> Option<HalfEdgeId> {
        let t = self.twin[h.0 as usize];
        (t != INVALID).then_some(HalfEdgeId(t))
    }

    /// Start vertex.
    pub fn origin(&self, h: HalfEdgeId) -> VertexId {
        VertexId(self.org(h.0))
    }

    /// End vertex.
    pub fn target(&self, h: HalfEdgeId) -> VertexId {
        VertexId(self.org(next_raw(h.0)))
    }

    /// Face the half-edge belongs to.
    pub fn face(&self, h: HalfEdgeId) -> FaceId {
        FaceId(h.0 / 3)
    }

    /// True for a vertex on a boundary, and for isolated vertices (like PMP / OpenMesh).
    pub fn is_boundary_vertex(&self, v: VertexId) -> bool {
        let h = self.vert_he[v.0 as usize];
        h == INVALID || self.twin[h as usize] == INVALID
    }

    /// Outgoing face half-edges of `v`, counter-clockwise, starting at the stored one. On a
    /// boundary vertex the last neighbour (reached only by an incoming boundary edge) is
    /// `origin(prev(last))`.
    pub fn outgoing(&self, v: VertexId) -> impl Iterator<Item = HalfEdgeId> + '_ {
        let start = self.vert_he[v.0 as usize];
        let mut cur = start;
        std::iter::from_fn(move || {
            if cur == INVALID {
                return None;
            }
            let out = cur;
            let t = self.twin[prev_raw(cur) as usize];
            cur = if t == start { INVALID } else { t };
            Some(HalfEdgeId(out))
        })
    }

    /// Number of neighbouring vertices (edges at `v`).
    pub fn valence(&self, v: VertexId) -> usize {
        let n = self.outgoing(v).count();
        if n > 0 && self.is_boundary_vertex(v) {
            n + 1
        } else {
            n
        }
    }

    /// Neighbouring vertices, counter-clockwise.
    pub fn neighbors(&self, v: VertexId) -> Vec<VertexId> {
        let mut out = Vec::new();
        let mut last = None;
        for h in self.outgoing(v) {
            out.push(self.target(h));
            last = Some(h);
        }
        if let Some(l) = last
            && self.is_boundary_vertex(v)
        {
            out.push(self.origin(self.prev(l)));
        }
        out
    }

    /// Half-edge from `a` to `b`, if it exists.
    pub fn find_halfedge(&self, a: VertexId, b: VertexId) -> Option<HalfEdgeId> {
        self.outgoing(a).find(|&h| self.target(h) == b)
    }

    /// Boundary loops as twin-less half-edges in loop order (faces on the left), ordered by
    /// their smallest half-edge.
    pub fn boundary_loops(&self) -> Vec<Vec<HalfEdgeId>> {
        let mut seen = vec![false; self.twin.len()];
        let mut loops = Vec::new();
        for h in 0..self.twin.len() as u32 {
            if seen[h as usize]
                || self.twin[h as usize] != INVALID
                || !self.face_alive[h as usize / 3]
            {
                continue;
            }
            let mut lp = Vec::new();
            let mut cur = h;
            // Bounded so a broken invariant cannot loop forever.
            for _ in 0..self.twin.len() {
                seen[cur as usize] = true;
                lp.push(HalfEdgeId(cur));
                let nxt = self.vert_he[self.org(next_raw(cur)) as usize];
                if nxt == h || nxt == INVALID || seen[nxt as usize] {
                    break;
                }
                cur = nxt;
            }
            loops.push(lp);
        }
        loops
    }

    // ---- conversion and compaction ----------------------------------------------------

    /// Back to an [`IndexedMesh`], dropping deleted elements (order preserved). Isolated live
    /// vertices are kept.
    pub fn to_indexed(&self) -> IndexedMesh {
        if self.dead_vertices == 0 && self.dead_faces == 0 {
            return IndexedMesh {
                positions: self.positions.clone(),
                tris: self.tris.clone(),
                attrs: self.attrs.clone(),
                groups: self.groups.clone(),
            };
        }
        let mut c = self.clone();
        c.garbage_collect();
        IndexedMesh {
            positions: c.positions,
            tris: c.tris,
            attrs: c.attrs,
            groups: c.groups,
        }
    }

    /// Removes deleted vertices and faces, renumbering everything (order preserved).
    pub fn garbage_collect(&mut self) -> Remap {
        let mut vmap = vec![INVALID; self.positions.len()];
        let mut n = 0u32;
        for (i, &a) in self.vert_alive.iter().enumerate() {
            if a {
                vmap[i] = n;
                n += 1;
            }
        }
        let mut fmap = vec![INVALID; self.tris.len()];
        let mut n = 0u32;
        for (i, &a) in self.face_alive.iter().enumerate() {
            if a {
                fmap[i] = n;
                n += 1;
            }
        }
        let he_map = |h: u32| -> u32 {
            if h == INVALID {
                INVALID
            } else {
                let f = fmap[h as usize / 3];
                if f == INVALID { INVALID } else { 3 * f + h % 3 }
            }
        };
        let mut tris = Vec::with_capacity(self.face_count());
        let mut twin = Vec::with_capacity(3 * self.face_count());
        for (f, t) in self.tris.iter().enumerate() {
            if self.face_alive[f] {
                tris.push(t.map(|v| vmap[v as usize]));
                for k in 0..3 {
                    twin.push(he_map(self.twin[3 * f + k]));
                }
            }
        }
        let mut positions = Vec::with_capacity(self.vertex_count());
        let mut vert_he = Vec::with_capacity(self.vertex_count());
        for (v, &a) in self.vert_alive.iter().enumerate() {
            if a {
                positions.push(self.positions[v]);
                vert_he.push(he_map(self.vert_he[v]));
            }
        }
        self.attrs.retain(&self.vert_alive, &self.face_alive);
        self.groups.retain(&self.face_alive);
        self.vert_alive = vec![true; positions.len()];
        self.face_alive = vec![true; tris.len()];
        self.positions = positions;
        self.tris = tris;
        self.twin = twin;
        self.vert_he = vert_he;
        self.dead_vertices = 0;
        self.dead_faces = 0;
        Remap {
            vertices: vmap,
            faces: fmap,
        }
    }

    // ---- invariants -------------------------------------------------------------------

    /// Checks every structural invariant; `Err` describes the first violation. Intended for
    /// tests and debug assertions (O(n log n)).
    pub fn check_invariants(&self) -> Result<(), String> {
        let nf = self.tris.len();
        let nv = self.positions.len();
        if self.twin.len() != 3 * nf || self.face_alive.len() != nf {
            return Err("face array lengths disagree".into());
        }
        if self.vert_he.len() != nv || self.vert_alive.len() != nv {
            return Err("vertex array lengths disagree".into());
        }
        if self.dead_faces != self.face_alive.iter().filter(|a| !**a).count()
            || self.dead_vertices != self.vert_alive.iter().filter(|a| !**a).count()
        {
            return Err("tombstone counters are wrong".into());
        }
        self.attrs
            .validate(nv, nf)
            .map_err(|e| format!("attributes: {e}"))?;
        self.groups
            .validate(nf)
            .map_err(|e| format!("groups: {e}"))?;
        let mut corners = vec![0usize; nv];
        let mut directed = Vec::with_capacity(3 * nf);
        for f in 0..nf {
            if !self.face_alive[f] {
                continue;
            }
            let t = self.tris[f];
            if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                return Err(format!("face {f} has repeated vertices {t:?}"));
            }
            for (k, &v) in t.iter().enumerate() {
                if v as usize >= nv || !self.vert_alive[v as usize] {
                    return Err(format!("face {f} uses dead or missing vertex {v}"));
                }
                corners[v as usize] += 1;
                let h = (3 * f + k) as u32;
                directed.push((v, self.org(next_raw(h)), h));
                let tw = self.twin[h as usize];
                if tw != INVALID {
                    if tw as usize >= self.twin.len() || !self.face_alive[tw as usize / 3] {
                        return Err(format!("half-edge {h} twins a dead half-edge {tw}"));
                    }
                    if self.twin[tw as usize] != h {
                        return Err(format!("twin of twin of {h} is not {h}"));
                    }
                    if self.org(tw) != self.org(next_raw(h)) || self.org(next_raw(tw)) != v {
                        return Err(format!("half-edge {h} and twin {tw} are not opposite"));
                    }
                }
            }
        }
        directed.sort_unstable();
        for w in directed.windows(2) {
            if w[0].0 == w[1].0 && w[0].1 == w[1].1 {
                return Err(format!(
                    "duplicate directed edge {}->{} (half-edges {} and {})",
                    w[0].0, w[0].1, w[0].2, w[1].2
                ));
            }
        }
        // A twin-less half-edge must not have a reverse partner (it would be a missed twin).
        for &(a, b, h) in &directed {
            if self.twin[h as usize] == INVALID
                && directed
                    .binary_search_by(|e| (e.0, e.1).cmp(&(b, a)))
                    .is_ok()
            {
                return Err(format!(
                    "half-edge {h} ({a}->{b}) has an unlinked reverse edge"
                ));
            }
        }
        for (v, &n_corners) in corners.iter().enumerate() {
            let h = self.vert_he[v];
            if !self.vert_alive[v] {
                if h != INVALID {
                    return Err(format!("dead vertex {v} has an outgoing half-edge"));
                }
                continue;
            }
            if h == INVALID {
                if n_corners != 0 {
                    return Err(format!("vertex {v} is used by faces but has no half-edge"));
                }
                continue;
            }
            if !self.face_alive[h as usize / 3] || self.org(h) != v as u32 {
                return Err(format!("outgoing half-edge {h} of vertex {v} is invalid"));
            }
            let fan = self
                .outgoing(VertexId(v as u32))
                .take(n_corners + 1)
                .count();
            if fan != n_corners {
                return Err(format!(
                    "vertex {v} has {} corners but its fan has {fan} (non-manifold vertex)",
                    n_corners
                ));
            }
            if self.twin[h as usize] != INVALID
                && self
                    .outgoing(VertexId(v as u32))
                    .any(|o| self.twin[prev_raw(o.0) as usize] == INVALID)
            {
                return Err(format!(
                    "boundary vertex {v} does not store its boundary half-edge"
                ));
            }
        }
        Ok(())
    }

    // ---- internals --------------------------------------------------------------------

    #[inline]
    fn org(&self, h: u32) -> u32 {
        self.tris[h as usize / 3][h as usize % 3]
    }

    /// Re-points `vert_he[v]` to its clockwise-most outgoing half-edge (the boundary one).
    fn adjust_outgoing(&mut self, v: u32) {
        let s = self.vert_he[v as usize];
        if s == INVALID {
            return;
        }
        let mut h = s;
        loop {
            let t = self.twin[h as usize];
            if t == INVALID {
                break;
            }
            let h2 = next_raw(t);
            if h2 == s {
                h = s;
                break;
            }
            h = h2;
        }
        self.vert_he[v as usize] = h;
    }
}

#[cfg(test)]
mod tests;
