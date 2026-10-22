//! Seams that only welding closes.
//!
//! A boundary edge `e = (u, v)` is *unwelded* when another edge `e' = (a, b)` has its endpoints
//! near `e`'s (either way round: `a ~ u, b ~ v` or `a ~ v, b ~ u`) without being the same vertex
//! pair, and `e` is longer than `tol`. Welding the near vertices would pair `e` up. "Near" means
//! within `tol` when `e'` is a boundary edge too (two open edges facing each other), and within
//! `tol / 1000` (numerically coincident) when `e'` has two faces. The looser distance is not
//! used for interior partners because a boundary next to a sliver or a very thin wall would
//! otherwise count. The length condition keeps tiny holes (and sliver edges) that a weld would
//! merely collapse from counting as seams.

use rayon::prelude::*;

use crate::indexed::IndexedMesh;

/// Spatial hash of all vertices.
struct Grid<'a> {
    m: &'a IndexedMesh,
    tol: f64,
    /// `(cell, vertex)` sorted.
    cells: Vec<([i64; 3], u32)>,
}

impl<'a> Grid<'a> {
    fn new(m: &'a IndexedMesh, tol: f64) -> Self {
        let mut g = Grid {
            m,
            tol,
            cells: Vec::new(),
        };
        g.cells = (0..m.positions.len() as u32)
            .into_par_iter()
            .map(|v| (g.cell(v), v))
            .collect();
        g.cells.par_sort_unstable();
        g
    }

    fn cell(&self, v: u32) -> [i64; 3] {
        let p = self.m.positions[v as usize] / self.tol;
        [p.x.floor() as i64, p.y.floor() as i64, p.z.floor() as i64]
    }

    /// Sorted vertices within `tol` of `v` (including `v`).
    fn near(&self, v: u32, radius: f64) -> Vec<u32> {
        let c = self.cell(v);
        let pv = self.m.positions[v as usize];
        let mut out = Vec::new();
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let n = [c[0] + dx, c[1] + dy, c[2] + dz];
                    let start = self.cells.partition_point(|e| e.0 < n);
                    for e in self.cells[start..].iter().take_while(|e| e.0 == n) {
                        if (self.m.positions[e.1 as usize] - pv).length() <= radius {
                            out.push(e.1);
                        }
                    }
                }
            }
        }
        out.sort_unstable();
        out
    }
}

/// Number of boundary edges `(from, to, face)` with a partner as described in the module docs,
/// for merge distance `tol` (mm).
pub(super) fn count(m: &IndexedMesh, boundary: &[(u32, u32, u32)], tol: f64) -> usize {
    if tol <= 0.0 || !tol.is_finite() {
        return 0;
    }
    let mut verts: Vec<u32> = boundary.iter().flat_map(|e| [e.0, e.1]).collect();
    verts.par_sort_unstable();
    verts.dedup();
    let grid = Grid::new(m, tol);

    // Vertices near a boundary vertex, and every edge that starts at one of them.
    let mut near_all: Vec<u32> = verts
        .par_iter()
        .flat_map_iter(|&v| grid.near(v, tol))
        .collect();
    near_all.par_sort_unstable();
    near_all.dedup();
    let mut in_near = vec![false; m.positions.len()];
    for &v in &near_all {
        in_near[v as usize] = true;
    }
    let mut inc: Vec<(u32, u32)> = m
        .tris
        .par_iter()
        .flat_map_iter(|t| {
            let live = t[0] != t[1] && t[1] != t[2] && t[0] != t[2];
            let mut out = Vec::new();
            if live {
                for k in 0..3 {
                    let (a, b) = (t[k], t[(k + 1) % 3]);
                    if in_near[a as usize] {
                        out.push((a, b));
                    }
                    if in_near[b as usize] {
                        out.push((b, a));
                    }
                }
            }
            out
        })
        .collect();
    inc.par_sort_unstable();
    inc.dedup();

    let mut boundary_pairs: Vec<(u32, u32)> = boundary
        .iter()
        .map(|e| (e.0.min(e.1), e.0.max(e.1)))
        .collect();
    boundary_pairs.par_sort_unstable();
    let tight = tol * 1e-3;

    boundary
        .par_iter()
        .filter(|&&(u, v, _)| {
            let len = (m.positions[u as usize] - m.positions[v as usize]).length();
            if len <= tol {
                return false;
            }
            let (nu, nv) = (grid.near(u, tol), grid.near(v, tol));
            let (tu, tv) = (grid.near(u, tight), grid.near(v, tight));
            let has = |s: &[u32], x: u32| s.binary_search(&x).is_ok();
            nu.iter().chain(&nv).any(|&a| {
                let start = inc.partition_point(|e| e.0 < a);
                inc[start..].iter().take_while(|e| e.0 == a).any(|&(_, b)| {
                    let same_pair = (a == u && b == v) || (a == v && b == u);
                    if same_pair {
                        return false;
                    }
                    let coincident = (has(&tu, a) && has(&tv, b)) || (has(&tv, a) && has(&tu, b));
                    let within = (has(&nu, a) && has(&nv, b)) || (has(&nv, a) && has(&nu, b));
                    coincident
                        || (within && boundary_pairs.binary_search(&(a.min(b), a.max(b))).is_ok())
                })
            })
        })
        .count()
}
