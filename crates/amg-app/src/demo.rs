//! The RND-01 demo mesh (a displaced icosphere) shown until the scene adapter (UI-20) feeds
//! real bodies. Generated and uploaded on a job, never on the UI thread.

use std::collections::HashMap;

use amg_core::{Point3, Vec3};
use amg_render::MeshData;

/// An icosahedron subdivided so the sphere has at least `min_tris` triangles (`20 f²`),
/// displaced radially by a few sine waves; radius about 100 mm. Same shape as the RND-01
/// `spin` example.
pub fn displaced_icosphere(min_tris: usize) -> MeshData {
    const T: f64 = 1.618_033_988_749_895;
    const CORNERS: [[f64; 3]; 12] = [
        [-1.0, T, 0.0],
        [1.0, T, 0.0],
        [-1.0, -T, 0.0],
        [1.0, -T, 0.0],
        [0.0, -1.0, T],
        [0.0, 1.0, T],
        [0.0, -1.0, -T],
        [0.0, 1.0, -T],
        [T, 0.0, -1.0],
        [T, 0.0, 1.0],
        [-T, 0.0, -1.0],
        [-T, 0.0, 1.0],
    ];
    const FACES: [[u32; 3]; 20] = [
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];

    let f = ((min_tris as f64 / 20.0).sqrt().ceil() as u32).max(1);
    let fu = f as usize;
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(10 * fu * fu + 2);
    let mut indices: Vec<u32> = Vec::with_capacity(20 * fu * fu * 3);
    // Points on icosahedron edges and corners are shared between faces; interior ones are not.
    let mut shared: HashMap<(u32, u32, u32), u32> = HashMap::new();
    let corner = |i: u32| Vec3::from_array(CORNERS[i as usize]);

    let mut row: Vec<Vec<u32>> = Vec::with_capacity(fu + 1);
    for [a, b, c] in FACES {
        row.clear();
        for i in 0..=f {
            let mut ids = Vec::with_capacity(i as usize + 1);
            for j in 0..=i {
                let (wa, wb, wc) = (f - i, i - j, j);
                let key = match (wa > 0, wb > 0, wc > 0) {
                    (true, false, false) => Some((a, a, f)),
                    (false, true, false) => Some((b, b, f)),
                    (false, false, true) => Some((c, c, f)),
                    (true, true, false) => Some(edge_key(a, wa, b, wb)),
                    (false, true, true) => Some(edge_key(b, wb, c, wc)),
                    (true, false, true) => Some(edge_key(a, wa, c, wc)),
                    _ => None,
                };
                let new_point = |positions: &mut Vec<[f32; 3]>| {
                    let dir = (corner(a) * f64::from(wa)
                        + corner(b) * f64::from(wb)
                        + corner(c) * f64::from(wc))
                    .normalize();
                    positions.push((dir * displacement(dir)).as_vec3().to_array());
                    (positions.len() - 1) as u32
                };
                ids.push(match key {
                    Some(key) => *shared
                        .entry(key)
                        .or_insert_with(|| new_point(&mut positions)),
                    None => new_point(&mut positions),
                });
            }
            row.push(ids);
        }
        for i in 0..fu {
            for j in 0..=i {
                indices.extend([row[i][j], row[i + 1][j], row[i + 1][j + 1]]);
                if j < i {
                    indices.extend([row[i][j], row[i + 1][j + 1], row[i][j + 1]]);
                }
            }
        }
    }
    let normals = vertex_normals(&positions, &indices);
    MeshData {
        origin: Point3::ZERO,
        positions,
        normals,
        indices,
        closed: true,
    }
}

fn edge_key(a: u32, wa: u32, b: u32, wb: u32) -> (u32, u32, u32) {
    if a < b { (a, b, wa) } else { (b, a, wb) }
}

fn displacement(d: Vec3) -> f64 {
    100.0
        * (1.0
            + 0.05 * (11.0 * d.x).sin() * (9.0 * d.y).sin()
            + 0.03 * (23.0 * d.z + 7.0 * d.x).sin()
            + 0.015 * (47.0 * d.y + 31.0 * d.z).sin())
}

/// Area-weighted vertex normals.
fn vertex_normals(positions: &[[f32; 3]], indices: &[u32]) -> Vec<[f32; 3]> {
    let mut sum = vec![Vec3::ZERO; positions.len()];
    let p = |i: u32| Vec3::from_array(positions[i as usize].map(f64::from));
    for t in indices.chunks_exact(3) {
        let n = (p(t[1]) - p(t[0])).cross(p(t[2]) - p(t[0]));
        for &v in t {
            sum[v as usize] += n;
        }
    }
    sum.iter()
        .map(|n| n.normalize_or_zero().as_vec3().to_array())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn icosphere_has_at_least_the_requested_triangles_and_valid_indices() {
        let m = displaced_icosphere(5_000);
        let tris = m.indices.len() / 3;
        assert!(tris >= 5_000, "{tris}");
        assert_eq!(m.normals.len(), m.positions.len());
        assert!(m.indices.iter().all(|&i| (i as usize) < m.positions.len()));
        // Closed: Euler characteristic of a sphere, V - E + F = 2 with E = 3F/2.
        let v = m.positions.len() as i64;
        let f = tris as i64;
        assert_eq!(v - 3 * f / 2 + f, 2);
    }
}
