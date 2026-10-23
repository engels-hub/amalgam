//! Non-manifold resolution by unzipping: faces are paired across every edge, and the corners
//! of each vertex are grouped into fans along the pairs; each fan gets its own vertex copy.
//!
//! Edges with two faces pair those two. Edges with more faces are sorted by the angle of their
//! faces around the edge, with every face oriented outward from the material it bounds, and a
//! face is paired with its neighbour on the material side. So the solids that meet at an edge
//! become separate sheets again (touching, not joined), and a zero-thickness fin becomes a
//! shell of its own. Faces that cannot be paired stay single and become boundary. A closed
//! mesh stays closed unless an edge has an odd number of faces.

use amg_core::{Point3, Vec3};
use amg_jobs::JobCtx;

use super::edges::EdgeTable;
use super::orient::oriented_flips;
use crate::error::MeshResult;
use crate::indexed::IndexedMesh;

/// What [`split_non_manifold`] did.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct SplitStats {
    /// Edges with more than two faces that were re-paired.
    pub edges: usize,
    /// Vertex copies added.
    pub vertices_added: usize,
}

fn find(parent: &mut [u32], mut i: u32) -> u32 {
    while parent[i as usize] != i {
        let p = parent[i as usize];
        parent[i as usize] = parent[p as usize];
        i = parent[i as usize];
    }
    i
}

fn union(parent: &mut [u32], a: u32, b: u32) {
    let (ra, rb) = (find(parent, a), find(parent, b));
    if ra != rb {
        let (lo, hi) = (ra.min(rb), ra.max(rb));
        parent[hi as usize] = lo;
    }
}

/// Splits non-manifold edges and vertices so that every edge has at most two faces and every
/// vertex one fan. Faces are never removed or reoriented.
pub(crate) fn split_non_manifold(m: &mut IndexedMesh, ctx: &JobCtx) -> MeshResult<SplitStats> {
    let table = EdgeTable::build(&m.tris)?;
    // Directions of the faces after orienting every component outward; the pairing uses only
    // which faces run `lo -> hi`, i.e. on which side of the face the material lies.
    let flips = oriented_flips(m, &table, ctx)?;
    let mut parent: Vec<u32> = (0..(3 * m.tris.len()) as u32).collect();
    let mut stats = SplitStats::default();
    // `pairs` per edge: corners paired with each other.
    let mut pairs: Vec<(u32, u32)> = Vec::new();
    for g in 0..table.len() {
        let grp = table.group(g);
        match grp.len() {
            0 | 1 => continue,
            2 => pairs.push((grp[0].1, grp[1].1)),
            _ => {
                stats.edges += 1;
                pair_fan(m, &table, g, &flips, &mut pairs);
            }
        }
        if g % 65_536 == 0 {
            ctx.checkpoint()?;
        }
    }
    for &(c1, c2) in &pairs {
        let (f1, k1) = ((c1 / 3) as usize, (c1 % 3) as usize);
        let (f2, k2) = ((c2 / 3) as usize, (c2 % 3) as usize);
        let (t1, t2) = (m.tris[f1], m.tris[f2]);
        for (slot1, v) in [(k1, t1[k1]), ((k1 + 1) % 3, t1[(k1 + 1) % 3])] {
            let slot2 = if t2[k2] == v { k2 } else { (k2 + 1) % 3 };
            union(
                &mut parent,
                (3 * f1 + slot1) as u32,
                (3 * f2 + slot2) as u32,
            );
        }
    }
    ctx.checkpoint()?;
    // One vertex per fan; the first fan met keeps the original index.
    let n0 = m.positions.len();
    let mut new_id = vec![crate::INVALID; parent.len()];
    let mut used = vec![false; m.positions.len()];
    for c in 0..parent.len() {
        let root = find(&mut parent, c as u32) as usize;
        if new_id[root] == crate::INVALID {
            let (f, k) = (c / 3, c % 3);
            let v = m.tris[f][k];
            if used[v as usize] {
                new_id[root] = m.positions.len() as u32;
                m.positions.push(m.positions[v as usize]);
                m.attrs.push_vertex_copy(v as usize);
                stats.vertices_added += 1;
            } else {
                used[v as usize] = true;
                new_id[root] = v;
            }
        }
    }
    for c in 0..parent.len() {
        let root = find(&mut parent, c as u32) as usize;
        m.tris[c / 3][c % 3] = new_id[root];
    }
    separate_copies(m, n0);
    Ok(stats)
}

/// How far a vertex copy moves towards its own fan, as a fraction of the distance to the mean
/// of its neighbours. Copies sit exactly on the vertex they were split from, and two sheets
/// that touch there would count as intersecting; this opens a gap far below any printing or
/// modelling tolerance (a thousandth of an edge) and keeps the original vertex where it was.
const SEPARATION: f64 = 1e-3;

/// Moves the vertex copies `n0..` slightly into their own fans.
fn separate_copies(m: &mut IndexedMesh, n0: usize) {
    if m.positions.len() == n0 {
        return;
    }
    let start = m.positions.clone();
    let mut sum = vec![(Point3::ZERO, 0u32); m.positions.len() - n0];
    for t in &m.tris {
        for k in 0..3 {
            let v = t[k] as usize;
            if v >= n0 {
                let (a, b) = (
                    start[t[(k + 1) % 3] as usize],
                    start[t[(k + 2) % 3] as usize],
                );
                let s = &mut sum[v - n0];
                s.0 += a + b;
                s.1 += 2;
            }
        }
    }
    for (i, (s, n)) in sum.iter().enumerate() {
        if *n > 0 {
            let p = start[n0 + i];
            let centre = *s / f64::from(*n);
            m.positions[n0 + i] = p + (centre - p) * SEPARATION;
        }
    }
}

/// Pairs the faces around the non-manifold edge `g` (see the module docs).
fn pair_fan(
    m: &IndexedMesh,
    table: &EdgeTable,
    g: usize,
    flips: &[bool],
    pairs: &mut Vec<(u32, u32)>,
) {
    let (lo, hi) = table.ends(g);
    let (p_lo, p_hi) = (m.positions[lo as usize], m.positions[hi as usize]);
    let axis = (p_hi - p_lo).normalize_or_zero();
    // Any unit vector perpendicular to the axis.
    let seed = if axis.x.abs() < 0.6 { Vec3::X } else { Vec3::Y };
    let e1 = (seed - axis * seed.dot(axis)).normalize_or_zero();
    let e2 = axis.cross(e1);
    // (angle, runs lo->hi after consistent orientation, corner)
    let mut fan: Vec<(f64, bool, u32)> = table
        .group(g)
        .iter()
        .map(|&(_, c)| {
            let (f, k) = ((c / 3) as usize, (c % 3) as usize);
            let t = m.tris[f];
            let forward = (t[k] == lo) ^ flips[f];
            let w: Point3 = m.positions[t[(k + 2) % 3] as usize];
            let r = w - p_lo;
            let r = r - axis * r.dot(axis);
            (r.dot(e2).atan2(r.dot(e1)), forward, c)
        })
        .collect();
    // Equal angles: `hi -> lo` faces first (a wedge of material opens with a `hi -> lo` face).
    fan.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.1.cmp(&b.1))
            .then(a.2.cmp(&b.2))
    });
    let mut left: Vec<(bool, u32)> = fan.iter().map(|&(_, fw, c)| (fw, c)).collect();
    loop {
        let n = left.len();
        if n < 2 {
            break;
        }
        let mut used = vec![false; n];
        let mut any = false;
        for i in 0..n {
            let j = (i + 1) % n;
            if !used[i] && !used[j] && !left[i].0 && left[j].0 {
                used[i] = true;
                used[j] = true;
                pairs.push((left[i].1, left[j].1));
                any = true;
            }
        }
        if !any {
            break;
        }
        let mut k = 0;
        left.retain(|_| {
            k += 1;
            !used[k - 1]
        });
    }
}
