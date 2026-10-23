//! Curvature sizing field for adaptive remeshing (MESH-22).
//!
//! Per input edge the curvature is `kappa = theta / d`, with `theta` the angle between the two
//! adjacent face normals and `d` the distance between the face centroids (exact for a sphere).
//! Edges across a feature crease (`theta` above the feature angle) are skipped: they are kept
//! as constrained edges and have no curvature in the smooth sense. A vertex takes the largest
//! `kappa` of its edges and of its one-ring (so a high-curvature spot is never missed between
//! samples). The edge length follows Dunyach et al. 2013, `L = sqrt(6 eps / kappa - 3 eps^2)`,
//! the edge of an equilateral triangle whose centre deviates by `eps` from a sphere of radius
//! `1 / kappa`. The remesher lets edges grow to 4/3 of the target, so the target is
//! [`SAFETY`] times that length. The result is clamped to `[min_length, max_length]` and made
//! Lipschitz ([`GRADATION`]) with a deterministic Dijkstra sweep.

use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::sync::Arc;

use amg_core::Point3;

use crate::{IndexedMesh, MeshSpatial};

/// Fraction of the Dunyach length used as the target (the longest allowed edge is 4/3 of it).
pub(crate) const SAFETY: f64 = 0.75;
/// Largest growth of the edge length per unit distance on the surface.
pub(crate) const GRADATION: f64 = 0.5;

/// Per-input-vertex target edge lengths.
pub(crate) struct SizeField {
    pub(crate) mesh: Arc<IndexedMesh>,
    pub(crate) sizes: Vec<f64>,
}

impl SizeField {
    /// Size at the point of the input surface closest to `p`.
    pub(crate) fn at(&self, spatial: &MeshSpatial, p: Point3, max_dist: f64) -> Option<f64> {
        let cp = spatial.closest_point(p, max_dist)?;
        let t = self.mesh.tris[cp.face.0 as usize];
        let w = [cp.bary.x, cp.bary.y, cp.bary.z];
        Some(
            (0..3)
                .map(|k| w[k] * self.sizes[t[k] as usize])
                .sum::<f64>(),
        )
    }
}

fn centroid(m: &IndexedMesh, f: usize) -> Point3 {
    let t = m.tris[f];
    (m.positions[t[0] as usize] + m.positions[t[1] as usize] + m.positions[t[2] as usize]) / 3.0
}

fn normal(m: &IndexedMesh, f: usize) -> Point3 {
    let t = m.tris[f];
    let (a, b, c) = (
        m.positions[t[0] as usize],
        m.positions[t[1] as usize],
        m.positions[t[2] as usize],
    );
    (b - a).cross(c - a).normalize_or_zero()
}

/// Sizes per vertex of `m` for tolerance `eps` and bounds `[min_len, max_len]`.
pub(crate) fn sizes(
    m: &IndexedMesh,
    eps: f64,
    min_len: f64,
    max_len: f64,
    feature_angle: Option<f64>,
) -> Vec<f64> {
    let nv = m.positions.len();
    // (edge key, face) pairs; sorting groups the faces of an edge deterministically.
    let mut inc: Vec<(u64, u32)> = Vec::with_capacity(m.tris.len() * 3);
    for (f, t) in m.tris.iter().enumerate() {
        for k in 0..3 {
            inc.push((super::key(t[k], t[(k + 1) % 3]), f as u32));
        }
    }
    inc.sort_unstable();
    let mut edges: Vec<(u32, u32, f64)> = Vec::new();
    let mut i = 0;
    while i < inc.len() {
        let mut j = i;
        while j < inc.len() && inc[j].0 == inc[i].0 {
            j += 1;
        }
        let k = inc[i].0;
        let (a, b) = ((k >> 32) as u32, k as u32);
        let mut kappa = 0.0;
        if j - i == 2 {
            let (f1, f2) = (inc[i].1 as usize, inc[i + 1].1 as usize);
            let theta = normal(m, f1).dot(normal(m, f2)).clamp(-1.0, 1.0).acos();
            // Distance across the edge: the centroid offset without its component along the
            // edge (slivers have centroids far apart along the edge, which says nothing about
            // the curvature across it).
            let e = (m.positions[b as usize] - m.positions[a as usize]).normalize_or_zero();
            let off = centroid(m, f2) - centroid(m, f1);
            let d = (off - e * off.dot(e)).length();
            if feature_angle.is_none_or(|fa| theta <= fa) && d > 0.0 {
                kappa = theta / d;
            }
        }
        edges.push((a, b, kappa));
        i = j;
    }
    let mut kv = vec![0.0f64; nv];
    for &(a, b, k) in &edges {
        kv[a as usize] = kv[a as usize].max(k);
        kv[b as usize] = kv[b as usize].max(k);
    }
    let mut kd = kv.clone();
    for &(a, b, _) in &edges {
        kd[a as usize] = kd[a as usize].max(kv[b as usize]);
        kd[b as usize] = kd[b as usize].max(kv[a as usize]);
    }
    let mut s: Vec<f64> = kd
        .iter()
        .map(|&k| {
            let rad = 6.0 * eps / k - 3.0 * eps * eps;
            // rad <= 0 means eps >= 2 / kappa: the tolerance exceeds the curvature diameter,
            // so no length can violate it.
            let l = if k <= 0.0 || rad <= 0.0 {
                max_len
            } else {
                SAFETY * rad.sqrt()
            };
            l.clamp(min_len, max_len)
        })
        .collect();
    // Gradation: s_v <= s_w + g |p_v - p_w|, by Dijkstra from the smallest sizes.
    let mut adj: Vec<Vec<(u32, f64)>> = vec![Vec::new(); nv];
    for &(a, b, _) in &edges {
        let d = m.positions[a as usize].distance(m.positions[b as usize]);
        adj[a as usize].push((b, d));
        adj[b as usize].push((a, d));
    }
    let mut heap: BinaryHeap<Reverse<(u64, u32)>> = s
        .iter()
        .enumerate()
        .map(|(v, &x)| Reverse((x.to_bits(), v as u32)))
        .collect();
    while let Some(Reverse((bits, v))) = heap.pop() {
        if f64::from_bits(bits) > s[v as usize] {
            continue;
        }
        for &(w, d) in &adj[v as usize] {
            let cand = s[v as usize] + GRADATION * d;
            if cand < s[w as usize] {
                s[w as usize] = cand;
                heap.push(Reverse((cand.to_bits(), w)));
            }
        }
    }
    s
}
