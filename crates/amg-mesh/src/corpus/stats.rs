//! Independent topology counts for corpus cross-checks. This is not the S07 analysis (MESH-02):
//! it is a few dozen lines that the manifest flags and the generator's ground truth can be
//! verified against before MESH-02 exists, and that MESH-02's report must agree with.

use std::collections::HashMap;

use crate::IndexedMesh;

/// Topology counts of an indexed mesh (exact vertex identity, no tolerance).
#[derive(Clone, Debug, PartialEq)]
pub struct QuickStats {
    /// Face count (all faces).
    pub faces: usize,
    /// Faces with a repeated vertex index or area <= 1e-12 * bbox_diagonal^2.
    pub degenerate_faces: usize,
    /// Faces whose sorted vertex triple already occurs earlier.
    pub duplicate_faces: usize,
    /// Edges used by exactly one (non-degenerate) face.
    pub boundary_edges: usize,
    /// Edges used by more than two faces.
    pub non_manifold_edges: usize,
    /// Edges used by two faces in the same direction.
    pub inconsistent_edges: usize,
    /// Connected components over non-degenerate faces.
    pub components: usize,
    /// Signed volume of the whole mesh (divergence theorem).
    pub signed_volume: f64,
    /// Vertices whose faces form more than one fan.
    pub non_manifold_vertices: usize,
    /// Boundary loop lengths, sorted; `None` when a boundary vertex has more than one outgoing
    /// boundary edge (loops are not simple).
    pub boundary_loops: Option<Vec<usize>>,
}

fn find(p: &mut [u32], mut x: u32) -> u32 {
    while p[x as usize] != x {
        let g = p[p[x as usize] as usize];
        p[x as usize] = g;
        x = g;
    }
    x
}

/// Computes [`QuickStats`].
pub fn quick_stats(m: &IndexedMesh) -> QuickStats {
    let diag = {
        let bb = m.aabb();
        (bb.max - bb.min).length()
    };
    let area_eps = 1e-12 * diag * diag;
    let mut degenerate = 0usize;
    let mut live: Vec<[u32; 3]> = Vec::with_capacity(m.tris.len());
    let mut vol = 0.0;
    for t in &m.tris {
        let [a, b, c] = t.map(|i| m.positions[i as usize]);
        vol += a.dot(b.cross(c)) / 6.0;
        let idx_degen = t[0] == t[1] || t[1] == t[2] || t[0] == t[2];
        let area = 0.5 * (b - a).cross(c - a).length();
        if idx_degen || area <= area_eps {
            degenerate += 1;
        }
        if !idx_degen {
            live.push(*t);
        }
    }
    let mut keys: Vec<[u32; 3]> = m
        .tris
        .iter()
        .map(|t| {
            let mut s = *t;
            s.sort_unstable();
            s
        })
        .collect();
    keys.sort_unstable();
    let duplicate_faces = keys.windows(2).filter(|w| w[0] == w[1]).count();

    // (undirected key, forward?, from, to)
    let mut edges: Vec<(u64, bool, u32, u32)> = Vec::with_capacity(live.len() * 3);
    for t in &live {
        for k in 0..3 {
            let (u, v) = (t[k], t[(k + 1) % 3]);
            let (lo, hi) = (u.min(v), u.max(v));
            edges.push(((u64::from(lo) << 32) | u64::from(hi), u < v, u, v));
        }
    }
    edges.sort_unstable_by_key(|e| e.0);
    let (mut boundary, mut nm, mut inconsistent) = (0, 0, 0);
    let mut out_edge: HashMap<u32, Vec<u32>> = HashMap::new();
    let mut i = 0;
    while i < edges.len() {
        let mut j = i;
        while j < edges.len() && edges[j].0 == edges[i].0 {
            j += 1;
        }
        match j - i {
            1 => {
                boundary += 1;
                out_edge.entry(edges[i].2).or_default().push(edges[i].3);
            }
            2 => {
                if edges[i].1 == edges[i + 1].1 {
                    inconsistent += 1;
                }
            }
            _ => nm += 1,
        }
        i = j;
    }
    let boundary_loops = if out_edge.values().any(|v| v.len() > 1) {
        None
    } else {
        let mut next: HashMap<u32, u32> = out_edge.iter().map(|(k, v)| (*k, v[0])).collect();
        let mut starts: Vec<u32> = next.keys().copied().collect();
        starts.sort_unstable();
        let mut loops = Vec::new();
        for s in starts {
            if !next.contains_key(&s) {
                continue;
            }
            let (mut n, mut cur) = (0usize, s);
            while let Some(nx) = next.remove(&cur) {
                n += 1;
                cur = nx;
            }
            loops.push(n);
        }
        loops.sort_unstable();
        Some(loops)
    };

    // Components over live faces.
    let nv = m.positions.len();
    let mut parent: Vec<u32> = (0..nv as u32).collect();
    let mut used = vec![false; nv];
    for t in &live {
        for &v in t {
            used[v as usize] = true;
        }
        let (a, b) = (find(&mut parent, t[0]), find(&mut parent, t[1]));
        parent[b as usize] = a;
        let (a, c) = (find(&mut parent, t[0]), find(&mut parent, t[2]));
        parent[c as usize] = a;
    }
    let mut roots = std::collections::BTreeSet::new();
    for v in 0..nv as u32 {
        if used[v as usize] {
            roots.insert(find(&mut parent, v));
        }
    }

    // Non-manifold vertices: faces around a vertex joined through a shared neighbour vertex.
    let mut fan: HashMap<u32, Vec<[u32; 2]>> = HashMap::new();
    for t in &live {
        for k in 0..3 {
            fan.entry(t[k])
                .or_default()
                .push([t[(k + 1) % 3], t[(k + 2) % 3]]);
        }
    }
    let mut nm_verts = 0;
    for (_, faces) in fan {
        if faces.len() < 2 {
            continue;
        }
        let mut p: Vec<u32> = (0..faces.len() as u32).collect();
        let mut link: HashMap<u32, u32> = HashMap::new();
        for (fi, f) in faces.iter().enumerate() {
            for &o in f {
                match link.get(&o) {
                    Some(&g) => {
                        let (a, b) = (find(&mut p, fi as u32), find(&mut p, g));
                        p[a as usize] = b;
                    }
                    None => {
                        link.insert(o, fi as u32);
                    }
                }
            }
        }
        let first = find(&mut p, 0);
        if (1..faces.len() as u32).any(|f| find(&mut p, f) != first) {
            nm_verts += 1;
        }
    }
    QuickStats {
        faces: m.tris.len(),
        degenerate_faces: degenerate,
        duplicate_faces,
        boundary_edges: boundary,
        non_manifold_edges: nm,
        inconsistent_edges: inconsistent,
        components: roots.len(),
        signed_volume: vol,
        non_manifold_vertices: nm_verts,
        boundary_loops,
    }
}
