//! Reference meshes for tests: an axis-aligned cube and icospheres.

use std::collections::HashMap;

use amg_core::{Point3, Vec3};

use crate::TriSoup;

/// An owned indexed triangle mesh, mm. [`TriMesh::soup`] borrows it as a [`TriSoup`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TriMesh {
    /// Vertex positions, mm.
    pub positions: Vec<Point3>,
    /// Triangles, counter-clockwise seen from outside.
    pub triangles: Vec<[u32; 3]>,
}

impl TriMesh {
    /// Borrowed view.
    pub fn soup(&self) -> TriSoup<'_> {
        TriSoup {
            positions: &self.positions,
            triangles: &self.triangles,
        }
    }
}

/// Axis-aligned cube with corner `min` and edge length `size`: 8 shared vertices and
/// 12 outward-facing triangles.
pub fn cube(min: Point3, size: f64) -> TriMesh {
    let positions = (0..8u32)
        .map(|i| {
            let bit = |b: u32| f64::from((i >> b) & 1);
            min + size * Vec3::new(bit(0), bit(1), bit(2))
        })
        .collect();
    // Vertex i has x = bit 0, y = bit 1, z = bit 2.
    let triangles = vec![
        [0, 2, 3],
        [0, 3, 1], // z = 0
        [4, 5, 7],
        [4, 7, 6], // z = 1
        [0, 1, 5],
        [0, 5, 4], // y = 0
        [2, 6, 7],
        [2, 7, 3], // y = 1
        [0, 4, 6],
        [0, 6, 2], // x = 0
        [1, 3, 7],
        [1, 7, 5], // x = 1
    ];
    TriMesh {
        positions,
        triangles,
    }
}

/// Icosphere: an icosahedron whose triangles are split into four `subdivisions` times, with
/// every vertex projected onto the sphere. It has `20 * 4^subdivisions` triangles
/// (subdivision 8 gives 1 310 720) and `10 * 4^subdivisions + 2` vertices.
///
/// # Panics
/// If `subdivisions > 12` (the vertex count would overflow `u32`).
pub fn icosphere(center: Point3, radius: f64, subdivisions: u32) -> TriMesh {
    assert!(
        subdivisions <= 12,
        "icosphere subdivision {subdivisions} too fine"
    );
    let t = (1.0 + 5f64.sqrt()) / 2.0;
    let mut unit: Vec<Vec3> = [
        (-1.0, t, 0.0),
        (1.0, t, 0.0),
        (-1.0, -t, 0.0),
        (1.0, -t, 0.0),
        (0.0, -1.0, t),
        (0.0, 1.0, t),
        (0.0, -1.0, -t),
        (0.0, 1.0, -t),
        (t, 0.0, -1.0),
        (t, 0.0, 1.0),
        (-t, 0.0, -1.0),
        (-t, 0.0, 1.0),
    ]
    .iter()
    .map(|&(x, y, z)| Vec3::new(x, y, z).normalize())
    .collect();
    let mut tris: Vec<[u32; 3]> = vec![
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
    for _ in 0..subdivisions {
        let mut mid: HashMap<(u32, u32), u32> = HashMap::with_capacity(tris.len() * 3 / 2);
        let mut next = Vec::with_capacity(tris.len() * 4);
        let mut midpoint = |a: u32, b: u32, unit: &mut Vec<Vec3>| -> u32 {
            let key = if a < b { (a, b) } else { (b, a) };
            *mid.entry(key).or_insert_with(|| {
                let p = (unit[a as usize] + unit[b as usize]).normalize();
                unit.push(p);
                u32::try_from(unit.len() - 1).expect("vertex count fits u32")
            })
        };
        for &[a, b, c] in &tris {
            let ab = midpoint(a, b, &mut unit);
            let bc = midpoint(b, c, &mut unit);
            let ca = midpoint(c, a, &mut unit);
            next.extend_from_slice(&[[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
        }
        tris = next;
    }
    TriMesh {
        positions: unit.into_iter().map(|u| center + radius * u).collect(),
        triangles: tris,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{metrics, signed_volume};

    #[test]
    fn icosphere_counts_and_orientation() {
        for k in 0..=3 {
            let s = icosphere(Point3::new(1.0, 2.0, 3.0), 2.0, k);
            assert_eq!(s.triangles.len(), 20 * 4usize.pow(k));
            assert_eq!(s.positions.len(), 10 * 4usize.pow(k) + 2);
            for p in &s.positions {
                let d = (*p - Point3::new(1.0, 2.0, 3.0)).length();
                assert!((d - 2.0).abs() < 1e-12);
            }
            assert!(signed_volume(s.soup()) > 0.0, "outward winding");
        }
    }

    #[test]
    fn cube_is_closed_and_outward() {
        let c = cube(Point3::splat(-1.0), 2.0);
        let m = metrics(c.soup());
        assert_eq!((m.boundary_edges, m.euler), (0, 2));
        assert!(m.volume > 0.0);
    }
}
