//! `IndexedMesh` → `HalfEdgeMesh` conversion.
//!
//! 1. Faces with a repeated vertex index are dropped (they have no directed-edge form).
//! 2. Outgoing half-edges are bucketed per origin vertex (CSR, O(n)).
//! 3. Half-edges are sorted by undirected edge key `(min, max, h)`; inside each run `u→w` is
//!    paired with `w→u` iff each direction occurs exactly once (O(n log n), independent of
//!    vertex valence). Edges
//!    with more than two faces, or two faces of the same direction (orientation conflict), are
//!    cut: all their half-edges become boundary.
//! 4. Each vertex's half-edges are split into fans (maximal twin-connected umbrellas). A vertex
//!    with more than one fan (bow-tie, or a consequence of a cut) is non-manifold; under
//!    `SplitVertices` every extra fan gets its own copy of the vertex.
//! 5. Only if edges were cut: a fix-up loop re-links cut half-edges that ended up as the two
//!    ends of one slit fan (`u→w` and `w→u` with no other copies), and unlinks duplicate
//!    directed edges left inside one fan (a fan winding around twice), re-splitting fans until
//!    nothing changes. Manifold input never enters this loop.
//!
//! Steps 3 and 4 run in parallel; output is independent of the thread count.

use rayon::prelude::*;

use super::{HalfEdgeMesh, next_raw, prev_raw};
use crate::INVALID;
use crate::attrs::MeshAttrs;
use crate::error::{MeshError, MeshResult};
use crate::indexed::IndexedMesh;
use crate::weld::to_u32;

/// What to do with non-manifold input.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NonManifoldPolicy {
    /// Return [`MeshError::NonManifold`].
    Fail,
    /// Cut unpairable edges and duplicate non-manifold vertices (one copy per extra fan).
    SplitVertices,
}

/// What the conversion had to change.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ConversionReport {
    /// Vertex copies created (one per extra fan of a non-manifold vertex). New vertices are
    /// appended after the input's vertices.
    pub split_vertices: usize,
    /// Undirected input edges cut because they could not be paired (more than two faces, or
    /// two faces with the same direction).
    pub cut_edges: usize,
    /// Faces dropped because they repeat a vertex index.
    pub dropped_degenerate: usize,
}

/// Outgoing half-edges per vertex: `out[offs[v]..offs[v + 1]]`, ascending.
fn csr(corners: &[u32], nv: usize) -> (Vec<u32>, Vec<u32>) {
    let mut offs = vec![0u32; nv + 1];
    for &v in corners {
        offs[v as usize + 1] += 1;
    }
    for i in 0..nv {
        offs[i + 1] += offs[i];
    }
    let mut fill = offs.clone();
    let mut out = vec![0u32; corners.len()];
    for (h, &v) in corners.iter().enumerate() {
        out[fill[v as usize] as usize] = h as u32;
        fill[v as usize] += 1;
    }
    (offs, out)
}

/// Clockwise-most half-edge of the fan containing `s0`, and the fan size.
fn fan_of(twin: &[u32], s0: u32) -> (u32, usize) {
    let mut h = s0;
    loop {
        let t = twin[h as usize];
        if t == INVALID {
            break;
        }
        let h2 = next_raw(t);
        if h2 == s0 {
            h = s0;
            break;
        }
        h = h2;
    }
    let start = h;
    let mut n = 0usize;
    loop {
        n += 1;
        let t = twin[prev_raw(h) as usize];
        if t == INVALID || t == start {
            break;
        }
        h = t;
    }
    (start, n)
}

/// Pairs half-edges by sorting on the undirected edge key. Inside each run of equal keys:
/// one `u→w` and one `w→u` become twins; a lone half-edge is boundary; anything else is cut
/// (all its half-edges stay unpaired; counted once per run). Returns (twin, cut edge count).
/// Deterministic: the sort key `(edge, h)` is a total order.
fn pair_twins(corners: &[u32], n_he: usize) -> (Vec<u32>, usize) {
    let mut keys: Vec<(u64, u32)> = (0..n_he as u32)
        .into_par_iter()
        .map(|h| {
            let u = corners[h as usize];
            let w = corners[next_raw(h) as usize];
            let (lo, hi) = if u < w { (u, w) } else { (w, u) };
            (((lo as u64) << 32) | hi as u64, h)
        })
        .collect();
    keys.par_sort_unstable();
    let mut twin = vec![INVALID; n_he];
    let mut cut = 0usize;
    let mut i = 0;
    while i < keys.len() {
        let mut j = i + 1;
        while j < keys.len() && keys[j].0 == keys[i].0 {
            j += 1;
        }
        if j - i == 2 {
            let (a, b) = (keys[i].1, keys[i + 1].1);
            // Opposite directions iff the origins differ.
            if corners[a as usize] != corners[b as usize] {
                twin[a as usize] = b;
                twin[b as usize] = a;
            } else {
                cut += 1;
            }
        } else if j - i > 2 {
            cut += 1;
        }
        i = j;
    }
    (twin, cut)
}

struct Parts {
    positions: Vec<amg_core::Point3>,
    tris: Vec<[u32; 3]>,
    twin: Vec<u32>,
    vert_he: Vec<u32>,
    attrs: MeshAttrs,
}

impl Parts {
    /// Recomputes `vert_he` and gives every extra fan of a vertex its own vertex copy.
    /// Returns (vertices that had more than one fan, copies created). With `count_only`, only
    /// counts (and sets `vert_he` to each vertex's first fan).
    /// `csr_in` is the current CSR, if the caller has it.
    fn split_fans(
        &mut self,
        count_only: bool,
        csr_in: Option<(Vec<u32>, Vec<u32>)>,
    ) -> MeshResult<(usize, usize)> {
        let nv = self.positions.len();
        let (offs, out) = csr_in.unwrap_or_else(|| csr(self.tris.as_flattened(), nv));
        let outgoing = |v: usize| &out[offs[v] as usize..offs[v + 1] as usize];
        let twin = &self.twin;
        let first: Vec<(u32, bool)> = (0..nv)
            .into_par_iter()
            .map(|v| {
                let o = outgoing(v);
                if o.is_empty() {
                    return (INVALID, true);
                }
                let (s, n) = fan_of(twin, o[0]);
                (s, n == o.len())
            })
            .collect();
        self.vert_he = first.iter().map(|f| f.0).collect();
        let bad: Vec<usize> = (0..nv).filter(|&v| !first[v].1).collect();
        let mut copies = 0;
        if !bad.is_empty() && !count_only {
            let mut seen = vec![false; self.twin.len()];
            for &v in &bad {
                let mut first_fan = true;
                for &h0 in outgoing(v) {
                    if seen[h0 as usize] {
                        continue;
                    }
                    let (start, _) = fan_of(&self.twin, h0);
                    let mut fan = Vec::new();
                    let mut h = start;
                    loop {
                        seen[h as usize] = true;
                        fan.push(h);
                        let t = self.twin[prev_raw(h) as usize];
                        if t == INVALID || t == start {
                            break;
                        }
                        h = t;
                    }
                    if first_fan {
                        self.vert_he[v] = start;
                        first_fan = false;
                        continue;
                    }
                    let id = to_u32(self.positions.len())?;
                    self.positions.push(self.positions[v]);
                    self.attrs.push_vertex_copy(v);
                    self.vert_he.push(start);
                    // Relabelling corners never changes `twin`, nor other vertices' lists.
                    for &x in &fan {
                        self.tris[x as usize / 3][x as usize % 3] = id;
                    }
                    copies += 1;
                }
            }
        }
        Ok((bad.len(), copies))
    }

    /// One fix-up round after cuts (see module docs). Returns true if a twin link changed.
    fn fix_cuts(&mut self) -> bool {
        let corners = self.tris.as_flattened();
        let org = |h: u32| corners[h as usize];
        let mut d: Vec<(u32, u32, u32)> = (0..self.twin.len() as u32)
            .map(|h| (org(h), org(next_raw(h)), h))
            .collect();
        d.sort_unstable();
        let mut changed = false;
        let group = |d: &[(u32, u32, u32)], u: u32, w: u32| -> std::ops::Range<usize> {
            let lo = d.partition_point(|e| (e.0, e.1) < (u, w));
            let hi = d.partition_point(|e| (e.0, e.1) <= (u, w));
            lo..hi
        };
        let mut i = 0;
        while i < d.len() {
            let (u, w, _) = d[i];
            let g = group(&d, u, w);
            if g.len() > 1 {
                // Duplicates inside one fan: unlink all but the first.
                for &(_, _, h) in &d[g.start + 1..g.end] {
                    let t = self.twin[h as usize];
                    if t != INVALID {
                        self.twin[t as usize] = INVALID;
                        self.twin[h as usize] = INVALID;
                        changed = true;
                    }
                }
            } else if u < w && self.twin[d[i].2 as usize] == INVALID {
                let r = group(&d, w, u);
                if r.len() == 1 && self.twin[d[r.start].2 as usize] == INVALID {
                    // A slit: the two ends of one fan at both u and w.
                    let (a, b) = (d[i].2, d[r.start].2);
                    self.twin[a as usize] = b;
                    self.twin[b as usize] = a;
                    changed = true;
                }
            }
            i = g.end;
        }
        changed
    }
}

impl HalfEdgeMesh {
    /// Builds the half-edge structure. Vertex ids are kept (split copies are appended); faces
    /// keep their order after degenerate ones are dropped: face `k` of the result is the `k`-th
    /// face of `m` with three distinct indices, and its groups and attributes follow it.
    /// Fails with
    /// [`MeshError::NonManifold`] under [`NonManifoldPolicy::Fail`] if any edge must be cut or
    /// any vertex split.
    pub fn from_indexed(
        m: &IndexedMesh,
        policy: NonManifoldPolicy,
    ) -> MeshResult<(Self, ConversionReport)> {
        m.validate()?;
        let mut report = ConversionReport::default();

        // 1. Drop faces with repeated indices.
        let keep_f: Vec<bool> = m
            .tris
            .par_iter()
            .map(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2])
            .collect();
        report.dropped_degenerate = keep_f.iter().filter(|k| !**k).count();
        let mut attrs = m.attrs.clone();
        let mut groups = m.groups.clone();
        let tris: Vec<[u32; 3]> = if report.dropped_degenerate == 0 {
            m.tris.clone()
        } else {
            attrs.retain(&vec![true; m.positions.len()], &keep_f);
            groups.retain(&keep_f);
            m.tris
                .iter()
                .zip(&keep_f)
                .filter(|(_, k)| **k)
                .map(|(t, _)| *t)
                .collect()
        };
        let n_he = tris
            .len()
            .checked_mul(3)
            .ok_or(MeshError::TooLarge { count: tris.len() })?;
        to_u32(n_he)?;
        let nv = m.positions.len();

        // 2.–3. CSR and twins.
        let (offs, out) = csr(tris.as_flattened(), nv);
        let (twin, cut_edges) = pair_twins(tris.as_flattened(), n_he);
        report.cut_edges = cut_edges;

        // 4. Fans.
        let mut parts = Parts {
            positions: m.positions.clone(),
            tris,
            twin,
            vert_he: Vec::new(),
            attrs,
        };
        let (bad, copies) =
            parts.split_fans(policy == NonManifoldPolicy::Fail, Some((offs, out)))?;
        if policy == NonManifoldPolicy::Fail && (report.cut_edges > 0 || bad > 0) {
            return Err(MeshError::NonManifold {
                edges: report.cut_edges,
                vertices: bad,
            });
        }
        report.split_vertices = copies;

        // 5. Fix-up after cuts.
        if report.cut_edges > 0 {
            while parts.fix_cuts() {
                let (_, copies) = parts.split_fans(false, None)?;
                report.split_vertices += copies;
            }
        }

        let nf = parts.tris.len();
        let nv_out = parts.positions.len();
        Ok((
            HalfEdgeMesh {
                positions: parts.positions,
                tris: parts.tris,
                twin: parts.twin,
                vert_he: parts.vert_he,
                vert_alive: vec![true; nv_out],
                face_alive: vec![true; nf],
                attrs: parts.attrs,
                groups,
                dead_vertices: 0,
                dead_faces: 0,
            },
            report,
        ))
    }
}
