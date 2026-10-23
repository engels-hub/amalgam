//! Cleaning steps on an [`IndexedMesh`]: boundary weld, degenerate and duplicate faces, small
//! shells.

use amg_core::{Point3, Vec3};
use amg_jobs::JobCtx;
use rayon::prelude::*;

use super::SmallShellFilter;
use crate::analysis::AnalysisReport;
use crate::error::MeshResult;
use crate::indexed::IndexedMesh;
use crate::weld::weld_roots;

/// Keeps the faces where `keep` is true (attributes and groups follow). Vertices stay.
pub(crate) fn retain_faces(m: &mut IndexedMesh, keep: &[bool]) {
    let nv = m.positions.len();
    m.attrs.retain(&vec![true; nv], keep);
    m.groups.retain(keep);
    let mut i = 0;
    m.tris.retain(|_| {
        i += 1;
        keep[i - 1]
    });
}

/// Merges vertices within `tolerance` that lie on a boundary edge (the vertices of
/// `report.boundary_loops`); the smallest index of a cluster survives. Interior vertices are
/// left alone, so coincident copies made by non-manifold splitting are not merged back.
/// Returns the number of vertices merged away. Merged-away vertices stay unreferenced.
pub(crate) fn weld_boundary(
    m: &mut IndexedMesh,
    report: &AnalysisReport,
    tolerance: f64,
) -> MeshResult<usize> {
    let mut ids: Vec<u32> = report.boundary_loops.iter().flatten().copied().collect();
    ids.sort_unstable();
    ids.dedup();
    if ids.len() < 2 {
        return Ok(0);
    }
    let pts: Vec<Point3> = ids.iter().map(|&i| m.positions[i as usize]).collect();
    let roots = weld_roots(&pts, tolerance)?;
    let mut remap: Vec<u32> = (0..m.positions.len() as u32).collect();
    let mut merged = 0;
    for (i, &r) in roots.iter().enumerate() {
        if r as usize != i {
            remap[ids[i] as usize] = ids[r as usize];
            merged += 1;
        }
    }
    if merged > 0 {
        m.tris
            .par_iter_mut()
            .for_each(|t| *t = t.map(|v| remap[v as usize]));
    }
    Ok(merged)
}

/// Vertex-to-face lists over the live faces, used by
/// [`remove_degenerate`].
struct Adjacency {
    offs: Vec<u32>,
    faces: Vec<u32>,
}

impl Adjacency {
    fn build(tris: &[[u32; 3]], dead: &[bool], nv: usize) -> Adjacency {
        let mut offs = vec![0u32; nv + 1];
        for (f, t) in tris.iter().enumerate() {
            if !dead[f] {
                for &v in t {
                    offs[v as usize + 1] += 1;
                }
            }
        }
        for i in 0..nv {
            offs[i + 1] += offs[i];
        }
        let mut fill = offs.clone();
        let mut faces = vec![0u32; offs[nv] as usize];
        for (f, t) in tris.iter().enumerate() {
            if !dead[f] {
                for &v in t {
                    faces[fill[v as usize] as usize] = f as u32;
                    fill[v as usize] += 1;
                }
            }
        }
        Adjacency { offs, faces }
    }

    fn of(&self, v: u32) -> &[u32] {
        &self.faces[self.offs[v as usize] as usize..self.offs[v as usize + 1] as usize]
    }
}

fn normal(p: &[Point3], t: [u32; 3]) -> Vec3 {
    let [a, b, c] = t.map(|i| p[i as usize]);
    (b - a).cross(c - a)
}

/// Plans collapsing the edge `v -> u` (`v` is merged into `u`): returns the faces on the edge,
/// which go, and the other faces of `v`, which are rewritten to `u`; or `None` if the link
/// condition fails, the edge is not manifold or a neighbouring face would flip.
fn plan_collapse(
    tris: &[[u32; 3]],
    positions: &[Point3],
    adj: &Adjacency,
    u: u32,
    v: u32,
) -> Option<(Vec<u32>, Vec<u32>)> {
    let fu = adj.of(u);
    let fv = adj.of(v);
    let shared: Vec<u32> = fu.iter().copied().filter(|f| fv.contains(f)).collect();
    if shared.is_empty() || shared.len() > 2 {
        return None;
    }
    let neighbours = |fs: &[u32], x: u32| {
        let mut n: Vec<u32> = fs
            .iter()
            .flat_map(|&f| tris[f as usize])
            .filter(|&w| w != x)
            .collect();
        n.sort_unstable();
        n.dedup();
        n
    };
    let (nu, nv) = (neighbours(fu, u), neighbours(fv, v));
    let common: Vec<u32> = nu
        .iter()
        .copied()
        .filter(|w| *w != v && nv.binary_search(w).is_ok())
        .collect();
    let mut opposite: Vec<u32> = shared
        .iter()
        .flat_map(|&f| tris[f as usize])
        .filter(|&w| w != u && w != v)
        .collect();
    opposite.sort_unstable();
    opposite.dedup();
    if common != opposite {
        return None;
    }
    let rewrite: Vec<u32> = fv.iter().copied().filter(|f| !shared.contains(f)).collect();
    for &f in &rewrite {
        let old = tris[f as usize];
        let new = old.map(|w| if w == v { u } else { w });
        let (n_old, n_new) = (normal(positions, old), normal(positions, new));
        let l = n_old.length();
        if l > 0.0 && n_new.dot(n_old) <= 0.05 * n_new.length() * l {
            return None;
        }
    }
    Some((shared, rewrite))
}

/// Removes degenerate faces. A face with a repeated vertex index goes. A zero-area face with
/// three distinct vertices is removed by collapsing its shortest edge that keeps the surface
/// a manifold without flipping a neighbour (so a closed mesh stays closed), and dropped when
/// no such edge exists. Returns the number of faces removed.
pub(crate) fn remove_degenerate(
    m: &mut IndexedMesh,
    degenerate: &[crate::FaceId],
    ctx: &JobCtx,
) -> MeshResult<usize> {
    let before = m.tris.len();
    let mut dead = vec![false; before];
    let mut pending: Vec<u32> = Vec::new();
    for f in degenerate {
        let t = m.tris[f.0 as usize];
        if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
            dead[f.0 as usize] = true;
        } else {
            pending.push(f.0);
        }
    }
    let nv = m.positions.len();
    for _pass in 0..8 {
        if pending.is_empty() {
            break;
        }
        ctx.checkpoint()?;
        let adj = Adjacency::build(&m.tris, &dead, nv);
        let mut locked = vec![false; nv];
        let mut progress = false;
        let mut next: Vec<u32> = Vec::new();
        for &f in &pending {
            if dead[f as usize] {
                continue;
            }
            let t = m.tris[f as usize];
            if t.iter().any(|&v| locked[v as usize]) {
                next.push(f);
                continue;
            }
            let mut edges = [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])];
            edges.sort_by(|a, b| {
                let la = m.positions[a.0 as usize].distance_squared(m.positions[a.1 as usize]);
                let lb = m.positions[b.0 as usize].distance_squared(m.positions[b.1 as usize]);
                la.total_cmp(&lb)
            });
            let mut done = false;
            for &(a, b) in &edges {
                // Merge the higher index into the lower one.
                let (u, v) = (a.min(b), a.max(b));
                if let Some((del, rew)) = plan_collapse(&m.tris, &m.positions, &adj, u, v) {
                    for &d in &del {
                        dead[d as usize] = true;
                    }
                    for &r in &rew {
                        for w in &mut m.tris[r as usize] {
                            if *w == v {
                                *w = u;
                            }
                        }
                    }
                    for x in [u, v] {
                        for &g in adj.of(x) {
                            for w in m.tris[g as usize] {
                                locked[w as usize] = true;
                            }
                        }
                    }
                    locked[u as usize] = true;
                    locked[v as usize] = true;
                    done = true;
                    break;
                }
            }
            if done {
                progress = true;
            } else {
                // Cannot be collapsed: try again only if the neighbourhood changes.
                next.push(f);
            }
        }
        if !progress {
            break;
        }
        pending = next;
    }
    // What is left cannot be collapsed. Dropping a face whose three edges all have a partner
    // would open a three-edge hole that the hole filler closes with the same face again, so
    // such a face stays; the others are dropped (their removal shrinks a hole or a fin).
    if !pending.is_empty() {
        let adj = Adjacency::build(&m.tris, &dead, nv);
        for &f in &pending {
            if dead[f as usize] {
                continue;
            }
            let t = m.tris[f as usize];
            let interior = (0..3).all(|k| {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                adj.of(a)
                    .iter()
                    .filter(|&&g| m.tris[g as usize].contains(&b))
                    .count()
                    == 2
            });
            if !interior {
                dead[f as usize] = true;
            }
        }
    }
    let removed = dead.iter().filter(|d| **d).count();
    if removed > 0 {
        let keep: Vec<bool> = dead.iter().map(|d| !d).collect();
        retain_faces(m, &keep);
    }
    Ok(removed)
}

/// Removes repeated faces (same vertex set). Of a set with `p` faces of one winding and `n` of
/// the other, the pairs of opposite windings cancel (a zero-thickness fold) and at most one
/// face of the remaining winding stays. Returns the number of faces removed.
pub(crate) fn remove_duplicates(m: &mut IndexedMesh) -> usize {
    let mut keyed: Vec<([u32; 3], bool, u32)> = m
        .tris
        .par_iter()
        .enumerate()
        .map(|(f, t)| {
            let mut s = *t;
            s.sort_unstable();
            // Parity of the permutation that sorts the corners.
            let odd = (t[0] > t[1]) ^ (t[1] > t[2]) ^ (t[0] > t[2]);
            (s, odd, f as u32)
        })
        .collect();
    keyed.par_sort_unstable();
    let mut keep = vec![true; m.tris.len()];
    let mut removed = 0;
    let mut i = 0;
    while i < keyed.len() {
        let mut j = i + 1;
        while j < keyed.len() && keyed[j].0 == keyed[i].0 {
            j += 1;
        }
        if j - i > 1 && keyed[i].0[0] != keyed[i].0[1] && keyed[i].0[1] != keyed[i].0[2] {
            let group = &keyed[i..j];
            let even: Vec<u32> = group.iter().filter(|g| !g.1).map(|g| g.2).collect();
            let odd: Vec<u32> = group.iter().filter(|g| g.1).map(|g| g.2).collect();
            let survivor = match even.len().cmp(&odd.len()) {
                std::cmp::Ordering::Greater => Some(even[0]),
                std::cmp::Ordering::Less => Some(odd[0]),
                std::cmp::Ordering::Equal => None,
            };
            for g in group {
                if Some(g.2) != survivor {
                    keep[g.2 as usize] = false;
                    removed += 1;
                }
            }
        }
        i = j;
    }
    if removed > 0 {
        retain_faces(m, &keep);
    }
    removed
}

/// Removes the shells `filter` calls small, except the largest one. A shell is small if it has
/// at most `max_faces` faces and its extent (absolute signed volume if closed, else bounding
/// box volume) is at most `max_volume_fraction` of that of the largest shell. Returns the
/// number of faces removed.
pub(crate) fn remove_small_shells(
    m: &mut IndexedMesh,
    report: &AnalysisReport,
    filter: &SmallShellFilter,
) -> usize {
    if report.shells.len() < 2 {
        return 0;
    }
    let extent: Vec<f64> = report
        .shells
        .iter()
        .map(|s| {
            if s.closed && s.signed_volume != 0.0 {
                s.signed_volume.abs()
            } else {
                let d = s.aabb.max - s.aabb.min;
                d.x * d.y * d.z
            }
        })
        .collect();
    let largest = (0..extent.len())
        .max_by(|&a, &b| extent[a].total_cmp(&extent[b]).then(b.cmp(&a)))
        .unwrap_or(0);
    let drop_shell: Vec<bool> = report
        .shells
        .iter()
        .enumerate()
        .map(|(i, s)| {
            i != largest
                && s.faces <= filter.max_faces
                && extent[i] <= filter.max_volume_fraction * extent[largest]
        })
        .collect();
    if !drop_shell.iter().any(|d| *d) {
        return 0;
    }
    let keep: Vec<bool> = report
        .shell_of_face
        .iter()
        .map(|&s| s == crate::INVALID || !drop_shell[s as usize])
        .collect();
    let removed = keep.iter().filter(|k| !**k).count();
    retain_faces(m, &keep);
    removed
}
