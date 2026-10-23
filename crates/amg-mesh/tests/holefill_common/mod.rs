//! Mesh generators for the MESH-20 tests and bench (shared via `#[path]`).
#![allow(dead_code, clippy::expect_used)]

use amg_core::Point3;
use amg_mesh::{FaceId, IndexedMesh};

/// Open tube with `n` segments around and `rings` vertex rings, both ends open (two holes of
/// exactly `n` edges). Segment length is about `edge`. With `wobble > 0` the two end rings
/// leave their plane (a non-planar rim); `bumps` modulates the radius so the wall is not a
/// perfect cylinder.
pub fn tube(n: u32, rings: u32, edge: f64, wobble: f64, bumps: f64) -> IndexedMesh {
    let r0 = edge / (2.0 * (std::f64::consts::PI / f64::from(n)).sin());
    let mut p = Vec::with_capacity((n * rings) as usize);
    for j in 0..rings {
        for i in 0..n {
            let th = f64::from(i) / f64::from(n) * std::f64::consts::TAU;
            let z = f64::from(j) * edge;
            let r = r0 * (1.0 + bumps * (3.0 * th + 0.3 * z).sin() * (0.2 * z).cos());
            let end = if j == 0 || j == rings - 1 { 1.0 } else { 0.0 };
            let dz = wobble * edge * end * ((2.0 * th).sin() + 0.5 * (3.0 * th).cos());
            p.push(Point3::new(r * th.cos(), r * th.sin(), z + dz));
        }
    }
    let id = |i: u32, j: u32| j * n + (i % n);
    let mut t = Vec::new();
    for j in 0..rings - 1 {
        for i in 0..n {
            let (a, b, c, d) = (id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
            t.push([a, b, c]);
            t.push([a, c, d]);
        }
    }
    IndexedMesh::new(p, t).expect("valid tube")
}

/// `m` without the faces for which `pick` returns true (vertices are kept; the unused ones are
/// harmless for analysis).
pub fn without_faces(m: &IndexedMesh, pick: impl Fn(FaceId) -> bool) -> IndexedMesh {
    let tris = (0..m.face_count() as u32)
        .filter(|&f| !pick(FaceId(f)))
        .map(|f| m.tris[f as usize])
        .collect();
    IndexedMesh::new(m.positions.clone(), tris).expect("valid cut mesh")
}

/// Faces within breadth-first distance `k` (face adjacency over shared edges) of face `seed`.
pub fn face_ball(m: &IndexedMesh, seed: u32, k: usize) -> Vec<bool> {
    use std::collections::HashMap;
    let mut edge: HashMap<(u32, u32), Vec<u32>> = HashMap::new();
    for (f, t) in m.tris.iter().enumerate() {
        for c in 0..3 {
            let (a, b) = (t[c], t[(c + 1) % 3]);
            edge.entry((a.min(b), a.max(b))).or_default().push(f as u32);
        }
    }
    let mut dist = vec![usize::MAX; m.tris.len()];
    let mut frontier = vec![seed];
    dist[seed as usize] = 0;
    let mut d = 0;
    while !frontier.is_empty() && d < k {
        d += 1;
        let mut next = Vec::new();
        for f in frontier {
            let t = m.tris[f as usize];
            for c in 0..3 {
                let (a, b) = (t[c], t[(c + 1) % 3]);
                for &g in &edge[&(a.min(b), a.max(b))] {
                    if dist[g as usize] == usize::MAX {
                        dist[g as usize] = d;
                        next.push(g);
                    }
                }
            }
        }
        frontier = next;
    }
    dist.iter().map(|&x| x != usize::MAX).collect()
}
