//! Bilateral normal filtering (Zheng, Fu, Au & Tai 2011) followed by a vertex update that
//! fits vertices to the filtered face normals (Sun et al. 2007 / Zheng et al. 2011).

use amg_core::{Point3, Vec3};
use rayon::prelude::*;

use super::adjacency::Adjacency;

/// Per-face geometry.
struct FaceGeom {
    normal: Vec<Vec3>,
    area: Vec<f64>,
    centroid: Vec<Point3>,
}

fn face_geom(pos: &[Point3], tris: &[[u32; 3]]) -> FaceGeom {
    let rows: Vec<(Vec3, f64, Point3)> = tris
        .par_iter()
        .map(|t| {
            let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
            let n = (b - a).cross(c - a);
            let len = n.length();
            let unit = if len > 1e-300 { n / len } else { Vec3::ZERO };
            (unit, 0.5 * len, (a + b + c) / 3.0)
        })
        .collect();
    FaceGeom {
        normal: rows.iter().map(|r| r.0).collect(),
        area: rows.iter().map(|r| r.1).collect(),
        centroid: rows.iter().map(|r| r.2).collect(),
    }
}

/// Calls `f` for every face that shares a vertex with `fi` (each once, `fi` excluded).
#[inline]
fn for_each_neighbor_face(tris: &[[u32; 3]], adj: &Adjacency, fi: usize, mut f: impl FnMut(usize)) {
    let t = tris[fi];
    for (k, &v) in t.iter().enumerate() {
        for &g in adj.faces_of(v as usize) {
            let g = g as usize;
            if g == fi {
                continue;
            }
            // Skip faces already visited through an earlier corner of `fi`.
            let u = tris[g];
            if t[..k].iter().any(|w| u.contains(w)) {
                continue;
            }
            f(g);
        }
    }
}

/// One outer round: filter face normals, then move vertices.
pub(super) fn round(
    pos: &mut Vec<Point3>,
    tris: &[[u32; 3]],
    adj: &Adjacency,
    movable: &[bool],
    sigma_r: f64,
    normal_iterations: u32,
    vertex_iterations: u32,
) {
    let g = face_geom(pos, tris);
    // Spatial sigma: mean centroid distance to neighbouring faces.
    let (sum, cnt) = (0..tris.len())
        .into_par_iter()
        .map(|i| {
            let (mut s, mut c) = (0.0, 0u64);
            for_each_neighbor_face(tris, adj, i, |j| {
                s += (g.centroid[i] - g.centroid[j]).length();
                c += 1;
            });
            (s, c)
        })
        .reduce(|| (0.0, 0), |a, b| (a.0 + b.0, a.1 + b.1));
    let sigma_s = if cnt > 0 && sum > 0.0 {
        sum / cnt as f64
    } else {
        1.0
    };
    let inv_s = -0.5 / (sigma_s * sigma_s);
    let inv_r = -0.5 / (sigma_r * sigma_r);

    let mut normals = g.normal.clone();
    for _ in 0..normal_iterations {
        let next: Vec<Vec3> = (0..tris.len())
            .into_par_iter()
            .map(|i| {
                let ni = normals[i];
                if ni == Vec3::ZERO {
                    return ni;
                }
                let mut acc = g.area[i] * ni;
                for_each_neighbor_face(tris, adj, i, |j| {
                    let ds = (g.centroid[i] - g.centroid[j]).length_squared();
                    let dr = (ni - normals[j]).length_squared();
                    let w = g.area[j] * (inv_s * ds + inv_r * dr).exp();
                    acc += w * normals[j];
                });
                let len = acc.length();
                if len > 1e-300 { acc / len } else { ni }
            })
            .collect();
        normals = next;
    }

    for it in 0..vertex_iterations {
        let fresh = (it > 0).then(|| face_geom(pos, tris));
        let (cur, area) = match &fresh {
            Some(f) => (&f.centroid, &f.area),
            None => (&g.centroid, &g.area),
        };
        let next: Vec<Point3> = (0..pos.len())
            .into_par_iter()
            .map(|v| {
                let p = pos[v];
                if !movable[v] {
                    return p;
                }
                let mut d = Vec3::ZERO;
                let mut wsum = 0.0;
                for &f in adj.faces_of(v) {
                    let f = f as usize;
                    let n = normals[f];
                    d += area[f] * n * n.dot(cur[f] - p);
                    wsum += area[f];
                }
                if wsum > 1e-300 { p + d / wsum } else { p }
            })
            .collect();
        *pos = next;
    }
}
