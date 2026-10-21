//! Topology edits: split, collapse, flip, add / delete face, add vertex.
//!
//! Every edit keeps all invariants of [`HalfEdgeMesh::check_invariants`] and leaves the mesh
//! unchanged when it returns an error. Attribute policy: new vertices interpolate vertex
//! attributes along the edge, new corners interpolate UVs inside the parent face (property
//! indices take the nearer endpoint's), and new faces inherit the parent face's group and
//! property group.

use amg_core::{Point2, Point3};

use super::{HalfEdgeMesh, next_raw, prev_raw};
use crate::error::TopoError;
use crate::groups::FaceGroupId;
use crate::{FaceId, HalfEdgeId, INVALID, VertexId};

/// Parameter of `p` projected onto segment `a`–`b`, clamped to [0, 1] (0.5 if degenerate).
fn edge_param(a: Point3, b: Point3, p: Point3) -> f64 {
    let d = b - a;
    let l2 = d.length_squared();
    if l2 > 0.0 && l2.is_finite() {
        ((p - a).dot(d) / l2).clamp(0.0, 1.0)
    } else {
        0.5
    }
}

fn idx(n: usize) -> u32 {
    assert!(
        n < INVALID as usize,
        "half-edge mesh exceeds the u32 index range"
    );
    n as u32
}

impl HalfEdgeMesh {
    fn check_he(&self, h: HalfEdgeId) -> Result<u32, TopoError> {
        let f = h.0 as usize / 3;
        if f < self.face_alive.len() && self.face_alive[f] {
            Ok(h.0)
        } else {
            Err(TopoError::Dead)
        }
    }

    /// `TooLarge` unless `verts` more vertices and `faces` more faces (3 half-edges each) stay
    /// below `INVALID`.
    fn room(&self, verts: usize, faces: usize) -> Result<(), TopoError> {
        let v_ok = self.positions.len() + verts < INVALID as usize;
        let f_ok = 3 * (self.tris.len() + faces) < INVALID as usize;
        if v_ok && f_ok {
            Ok(())
        } else {
            Err(TopoError::TooLarge)
        }
    }

    fn push_face_raw(&mut self, t: [u32; 3], group: FaceGroupId) -> u32 {
        let f = idx(self.tris.len());
        idx(3 * self.tris.len() + 2);
        self.tris.push(t);
        self.twin.extend_from_slice(&[INVALID; 3]);
        self.face_alive.push(true);
        self.groups.push(group);
        f
    }

    #[inline]
    fn link(&mut self, a: u32, b: u32) {
        self.twin[a as usize] = b;
        if b != INVALID {
            self.twin[b as usize] = a;
        }
    }

    /// Adds an isolated vertex (zero normal, white colour). Panics past `u32::MAX - 1`
    /// vertices.
    pub fn add_vertex(&mut self, p: Point3) -> VertexId {
        let v = idx(self.positions.len());
        self.positions.push(p);
        self.vert_he.push(INVALID);
        self.vert_alive.push(true);
        self.attrs.push_vertex_default();
        VertexId(v)
    }

    /// Splits the edge of `h` at `p` (one new vertex; two new faces on an interior edge, one
    /// on a boundary edge). Returns the new vertex. The faces of `h` and of its twin keep their
    /// ids and become the halves next to `origin(h)` and `origin(twin)` respectively.
    pub fn split_edge(&mut self, h: HalfEdgeId, p: Point3) -> Result<VertexId, TopoError> {
        let h = self.check_he(h)?;
        let f = h as usize / 3;
        let k = h as usize % 3;
        let (h_bc, h_ca) = (next_raw(h), prev_raw(h));
        let a = self.org(h);
        let b = self.org(h_bc);
        let c = self.org(h_ca);
        let t = self.twin[h as usize];
        if t != INVALID && self.org(prev_raw(t)) == c {
            // Both faces share all three vertices (a two-face shell): the new edges to the
            // opposite vertex would coincide.
            return Err(TopoError::NonManifold);
        }
        self.room(1, if t == INVALID { 1 } else { 2 })?;
        let s = edge_param(self.positions[a as usize], self.positions[b as usize], p);

        let m = idx(self.positions.len());
        self.positions.push(p);
        self.vert_he.push(INVALID);
        self.vert_alive.push(true);
        self.attrs.push_vertex_lerp(a as usize, b as usize, s);

        // Face f = (a, b, c) → (a, m, c); new f2 = (m, b, c).
        let (uv, pi) = self.attrs.corners(f);
        let (ka, kb, kc) = (k, (k + 1) % 3, (k + 2) % 3);
        let uv_m = uv[ka].lerp(uv[kb], s);
        let pi_m = if s < 0.5 { pi[ka] } else { pi[kb] };
        let mut uv_f = uv;
        let mut pi_f = pi;
        uv_f[kb] = uv_m;
        pi_f[kb] = pi_m;
        self.attrs.set_corners(f, uv_f, pi_f);
        self.tris[f][kb] = m;
        let g_f = self.groups.per_face()[f];
        let f2 = self.push_face_raw([m, b, c], g_f);
        self.attrs
            .push_face(Some(f), [uv_m, uv[kb], uv[kc]], [pi_m, pi[kb], pi[kc]]);
        let (e0, e1, e2) = (3 * f2, 3 * f2 + 1, 3 * f2 + 2);
        let x = self.twin[h_bc as usize];
        self.link(e1, x);
        self.link(h_bc, e2);
        if self.vert_he[b as usize] == h_bc {
            self.vert_he[b as usize] = e1;
        }

        if t == INVALID {
            self.twin[e0 as usize] = INVALID;
        } else {
            // Face g = (b, a, d) → (b, m, d); new g2 = (m, a, d).
            let g = t as usize / 3;
            let j = t as usize % 3;
            let (t_ad, t_db) = (next_raw(t), prev_raw(t));
            let d = self.org(t_db);
            let (uvg, pig) = self.attrs.corners(g);
            let (jb, ja, jd) = (j, (j + 1) % 3, (j + 2) % 3);
            let uv_mg = uvg[ja].lerp(uvg[jb], s);
            let pi_mg = if s < 0.5 { pig[ja] } else { pig[jb] };
            let mut uv_g = uvg;
            let mut pi_g = pig;
            uv_g[ja] = uv_mg;
            pi_g[ja] = pi_mg;
            self.attrs.set_corners(g, uv_g, pi_g);
            self.tris[g][ja] = m;
            let g_g = self.groups.per_face()[g];
            let g2 = self.push_face_raw([m, a, d], g_g);
            self.attrs.push_face(
                Some(g),
                [uv_mg, uvg[ja], uvg[jd]],
                [pi_mg, pig[ja], pig[jd]],
            );
            let (q0, q1, q2) = (3 * g2, 3 * g2 + 1, 3 * g2 + 2);
            let y = self.twin[t_ad as usize];
            self.link(q1, y);
            self.link(t_ad, q2);
            self.link(h, q0);
            self.link(t, e0);
            if self.vert_he[a as usize] == t_ad {
                self.vert_he[a as usize] = q1;
            }
        }
        self.vert_he[m as usize] = e0;
        self.adjust_outgoing(m);
        Ok(VertexId(m))
    }

    /// True if collapsing `h` (moving `origin(h)` into `target(h)`) keeps the mesh a manifold
    /// (link condition, plus PMP's boundary rules).
    pub fn can_collapse(&self, h: HalfEdgeId) -> bool {
        let Ok(h) = self.check_he(h) else {
            return false;
        };
        let v0 = self.org(h);
        let v1 = self.org(next_raw(h));
        let t = self.twin[h as usize];
        // The other two edges of a removed face must not both be boundary.
        if self.twin[next_raw(h) as usize] == INVALID && self.twin[prev_raw(h) as usize] == INVALID
        {
            return false;
        }
        let vl = self.org(prev_raw(h));
        let mut vr = INVALID;
        if t != INVALID {
            if self.twin[next_raw(t) as usize] == INVALID
                && self.twin[prev_raw(t) as usize] == INVALID
            {
                return false;
            }
            vr = self.org(prev_raw(t));
            if vl == vr {
                return false;
            }
            // An interior edge between two boundary vertices would pinch the surface.
            if self.is_boundary_vertex(VertexId(v0)) && self.is_boundary_vertex(VertexId(v1)) {
                return false;
            }
        }
        let n0 = self.neighbors(VertexId(v0));
        let n1 = self.neighbors(VertexId(v1));
        n0.iter()
            .filter(|n| n1.contains(n))
            .all(|n| n.0 == vl || n.0 == vr)
    }

    /// Collapses the edge of `h`: `origin(h)` is removed, `target(h)` survives at `p` (with
    /// vertex attributes interpolated along the edge), and the one or two faces of the edge are
    /// deleted. Returns the surviving vertex.
    pub fn collapse_edge(&mut self, h: HalfEdgeId, p: Point3) -> Result<VertexId, TopoError> {
        let h = self.check_he(h)?;
        if !self.can_collapse(HalfEdgeId(h)) {
            return Err(TopoError::LinkCondition);
        }
        let v0 = self.org(h);
        let v1 = self.org(next_raw(h));
        let vl = self.org(prev_raw(h));
        let t = self.twin[h as usize];
        let s = edge_param(self.positions[v0 as usize], self.positions[v1 as usize], p);

        let fan0: Vec<u32> = self.outgoing(VertexId(v0)).map(|x| x.0).collect();
        let f = h / 3;
        let g = if t == INVALID { INVALID } else { t / 3 };

        // External twins of the removed faces get linked to each other.
        let a_ = self.twin[next_raw(h) as usize]; // vl→v1
        let b_ = self.twin[prev_raw(h) as usize]; // v0→vl
        let (mut c_, mut d_, mut vr) = (INVALID, INVALID, INVALID);
        if t != INVALID {
            c_ = self.twin[next_raw(t) as usize]; // vr→v0
            d_ = self.twin[prev_raw(t) as usize]; // v1→vr
            vr = self.org(prev_raw(t));
        }
        for &x in &fan0 {
            let fx = x / 3;
            if fx != f && fx != g {
                self.tris[fx as usize][x as usize % 3] = v1;
            }
        }
        for dead in [f, g] {
            if dead == INVALID {
                continue;
            }
            for k in 0..3 {
                self.twin[(3 * dead + k) as usize] = INVALID;
            }
            self.face_alive[dead as usize] = false;
            self.dead_faces += 1;
        }
        if a_ != INVALID {
            self.twin[a_ as usize] = b_;
        }
        if b_ != INVALID {
            self.twin[b_ as usize] = a_;
        }
        if c_ != INVALID {
            self.twin[c_ as usize] = d_;
        }
        if d_ != INVALID {
            self.twin[d_ as usize] = c_;
        }
        self.vert_alive[v0 as usize] = false;
        self.vert_he[v0 as usize] = INVALID;
        self.dead_vertices += 1;
        self.positions[v1 as usize] = p;
        self.attrs
            .set_vertex_lerp(v1 as usize, v0 as usize, v1 as usize, s);

        let alive = |s: &Self, x: u32| x != INVALID && s.face_alive[x as usize / 3];
        // v1: any live outgoing half-edge (its own, or one inherited from v0).
        let cand = std::iter::once(self.vert_he[v1 as usize])
            .chain([b_, d_])
            .chain(fan0.iter().copied())
            .find(|&x| alive(self, x) && self.org(x) == v1)
            .unwrap_or(INVALID);
        self.vert_he[v1 as usize] = cand;
        self.adjust_outgoing(v1);
        for (w, inc, out) in [(vl, a_, b_), (vr, c_, d_)] {
            if w == INVALID {
                continue;
            }
            if !alive(self, self.vert_he[w as usize]) {
                self.vert_he[w as usize] = if inc != INVALID { inc } else { next_raw(out) };
            }
            self.adjust_outgoing(w);
        }
        Ok(VertexId(v1))
    }

    /// Flips the interior edge of `h` to connect the two opposite vertices. Both faces keep
    /// their ids and groups.
    pub fn flip_edge(&mut self, h: HalfEdgeId) -> Result<(), TopoError> {
        let h = self.check_he(h)?;
        let t = self.twin[h as usize];
        if t == INVALID {
            return Err(TopoError::BoundaryEdge);
        }
        let a = self.org(h);
        let b = self.org(next_raw(h));
        let c = self.org(prev_raw(h));
        let d = self.org(prev_raw(t));
        if c == d
            || self.find_halfedge(VertexId(c), VertexId(d)).is_some()
            || self.find_halfedge(VertexId(d), VertexId(c)).is_some()
        {
            return Err(TopoError::DuplicateEdge);
        }
        let f = h / 3;
        let g = t / 3;
        let e_bc = self.twin[next_raw(h) as usize];
        let e_ca = self.twin[prev_raw(h) as usize];
        let e_ad = self.twin[next_raw(t) as usize];
        let e_db = self.twin[prev_raw(t) as usize];

        let corner = |s: &Self, face: u32, v: u32| -> (Point2, u32) {
            let (uv, pi) = s.attrs.corners(face as usize);
            let k = s.tris[face as usize]
                .iter()
                .position(|&x| x == v)
                .unwrap_or(0);
            (uv[k], pi[k])
        };
        let (fc, fa, gd, gb) = (
            corner(self, f, c),
            corner(self, f, a),
            corner(self, g, d),
            corner(self, g, b),
        );
        self.tris[f as usize] = [c, a, d];
        self.tris[g as usize] = [d, b, c];
        self.attrs
            .set_corners(f as usize, [fc.0, fa.0, gd.0], [fc.1, fa.1, gd.1]);
        self.attrs
            .set_corners(g as usize, [gd.0, gb.0, fc.0], [gd.1, gb.1, fc.1]);
        let (f0, f1, f2) = (3 * f, 3 * f + 1, 3 * f + 2);
        let (g0, g1, g2) = (3 * g, 3 * g + 1, 3 * g + 2);
        self.link(f0, e_ca);
        self.link(f1, e_ad);
        self.link(g0, e_db);
        self.link(g1, e_bc);
        self.link(f2, g2);
        for (v, he) in [(a, f1), (b, g1), (c, f0), (d, g0)] {
            self.vert_he[v as usize] = he;
            self.adjust_outgoing(v);
        }
        Ok(())
    }

    /// Adds face `v` (counter-clockwise) in group `group`. Rejected if a vertex is dead, the
    /// face repeats a vertex, a directed edge already exists, or a vertex would get a second
    /// fan (the face must attach to each used corner's boundary).
    pub fn add_face(&mut self, v: [VertexId; 3], group: FaceGroupId) -> Result<FaceId, TopoError> {
        let vs = v.map(|x| x.0);
        for &x in &vs {
            if !self.is_vertex_alive(VertexId(x)) {
                return Err(TopoError::Dead);
            }
        }
        if vs[0] == vs[1] || vs[1] == vs[2] || vs[0] == vs[2] {
            return Err(TopoError::NonManifold);
        }
        let mut opp = [INVALID; 3];
        for k in 0..3 {
            let (x, y) = (vs[k], vs[(k + 1) % 3]);
            if self.find_halfedge(VertexId(x), VertexId(y)).is_some() {
                return Err(TopoError::DuplicateEdge);
            }
            opp[k] = self
                .find_halfedge(VertexId(y), VertexId(x))
                .map_or(INVALID, |hh| hh.0);
        }
        for k in 0..3 {
            let x = vs[k];
            if self.vert_he[x as usize] == INVALID {
                continue;
            }
            if !self.is_boundary_vertex(VertexId(x)) {
                return Err(TopoError::NonManifold);
            }
            // Edges at corner k: out-edge k and in-edge k−1.
            if opp[k] == INVALID && opp[(k + 2) % 3] == INVALID {
                return Err(TopoError::NonManifold);
            }
        }
        self.room(0, 1)?;
        let f = self.push_face_raw(vs, group);
        self.attrs.push_face(None, [Point2::ZERO; 3], [INVALID; 3]);
        for (k, &o) in opp.iter().enumerate() {
            self.link(3 * f + k as u32, o);
        }
        for (k, &x) in vs.iter().enumerate() {
            if self.vert_he[x as usize] == INVALID {
                self.vert_he[x as usize] = 3 * f + k as u32;
            }
            self.adjust_outgoing(x);
        }
        Ok(FaceId(f))
    }

    /// Deletes face `f` (tombstone; no-op if already dead). Vertices left without faces stay
    /// as isolated vertices. A boundary vertex in the middle of whose fan `f` sits would become
    /// a bow-tie; it is split instead (the faces counter-clockwise of `f` move to a copy of the
    /// vertex, appended at the end).
    ///
    /// # Panics
    /// If that vertex split would exceed the `u32` vertex-id space (like [`Self::add_vertex`]).
    pub fn delete_face(&mut self, f: FaceId) {
        let fi = f.0;
        if !self.is_face_alive(f) {
            return;
        }
        let old_twin = [0u32, 1, 2].map(|k| self.twin[(3 * fi + k) as usize]);
        let verts = self.tris[fi as usize];
        let mut extra = Vec::new();
        for k in 0..3u32 {
            let h_out = 3 * fi + k;
            let h_in = prev_raw(h_out);
            let x = verts[k as usize];
            let t_out = self.twin[h_out as usize];
            let t_in = self.twin[h_in as usize];
            if t_out == INVALID || t_in == INVALID || !self.is_boundary_vertex(VertexId(x)) {
                continue;
            }
            // Split: half-edges counter-clockwise of f move to a copy of x.
            let mut part = Vec::new();
            let mut h = t_in;
            for _ in 0..self.twin.len() {
                part.push(h);
                let tt = self.twin[prev_raw(h) as usize];
                if tt == INVALID {
                    break;
                }
                h = tt;
            }
            let xn = idx(self.positions.len());
            self.positions.push(self.positions[x as usize]);
            self.vert_alive.push(true);
            self.vert_he.push(t_in);
            self.attrs.push_vertex_copy(x as usize);
            for &e in &part {
                self.tris[e as usize / 3][e as usize % 3] = xn;
            }
            extra.push(xn);
        }
        for k in 0..3u32 {
            let h = 3 * fi + k;
            let t = self.twin[h as usize];
            if t != INVALID {
                self.twin[t as usize] = INVALID;
            }
            self.twin[h as usize] = INVALID;
        }
        self.face_alive[fi as usize] = false;
        self.dead_faces += 1;
        for k in 0..3usize {
            let x = verts[k];
            let cur = self.vert_he[x as usize];
            if cur == INVALID || cur / 3 == fi || self.org(cur) != x {
                // Old twin of the out-edge is y→x; its next is x's clockwise neighbour.
                // Otherwise the old twin of the in-edge is x's counter-clockwise neighbour
                // (unless it moved to a split copy).
                let t_out = old_twin[k];
                let t_in = old_twin[(k + 2) % 3];
                self.vert_he[x as usize] = if t_out != INVALID {
                    next_raw(t_out)
                } else if t_in != INVALID && self.org(t_in) == x {
                    t_in
                } else {
                    INVALID
                };
            }
            self.adjust_outgoing(x);
        }
        for xn in extra {
            self.adjust_outgoing(xn);
        }
    }

    /// Deletes a vertex and all its faces.
    ///
    /// # Panics
    /// Like [`Self::delete_face`].
    pub fn delete_vertex(&mut self, v: VertexId) {
        if !self.is_vertex_alive(v) {
            return;
        }
        loop {
            let h = self.vert_he[v.0 as usize];
            if h == INVALID {
                break;
            }
            self.delete_face(FaceId(h / 3));
        }
        self.vert_alive[v.0 as usize] = false;
        self.dead_vertices += 1;
    }
}
