//! Scalar and topological metrics of a triangle soup.

use amg_core::{Aabb3, Point3};
use rayon::slice::ParallelSliceMut;

use crate::TriSoup;

/// Metrics compared by golden tests.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshMetrics {
    /// Signed enclosed volume, mm³ (positive for outward-facing triangles).
    pub volume: f64,
    /// Surface area, mm².
    pub area: f64,
    /// Bounding box of the referenced vertices.
    pub bbox: Aabb3,
    /// Number of distinct vertex indices referenced by triangles.
    pub vertices: usize,
    /// Number of triangles.
    pub triangles: usize,
    /// Undirected edges used by exactly one triangle. Self-loop "edges" of degenerate
    /// triangles with a repeated index (`[a, a, b]`) are not edges and are never counted.
    pub boundary_edges: usize,
    /// Undirected edges used by more than two triangles.
    pub non_manifold_edges: usize,
    /// Euler characteristic `V - E + F` (2 for a closed genus-0 surface).
    pub euler: i64,
}

/// Computes every [`MeshMetrics`] field.
///
/// # Panics
/// If a triangle index is out of range.
pub fn metrics(m: TriSoup<'_>) -> MeshMetrics {
    let mut used = vec![false; m.positions.len()];
    let mut bbox = Aabb3::EMPTY;
    for tri in m.triangles {
        for &i in tri {
            let i = i as usize;
            if !used[i] {
                used[i] = true;
                bbox.include_point(m.positions[i]);
            }
        }
    }
    let vertices = used.iter().filter(|&&u| u).count();

    let mut edges: Vec<u64> = Vec::with_capacity(m.triangles.len() * 3);
    for &[a, b, c] in m.triangles {
        for (p, q) in [(a, b), (b, c), (c, a)] {
            if p == q {
                continue; // degenerate triangle: not an edge
            }
            let (lo, hi) = if p < q { (p, q) } else { (q, p) };
            edges.push((u64::from(lo) << 32) | u64::from(hi));
        }
    }
    edges.par_sort_unstable();
    let (mut unique, mut boundary, mut non_manifold) = (0usize, 0usize, 0usize);
    for run in edges.chunk_by(|a, b| a == b) {
        unique += 1;
        match run.len() {
            1 => boundary += 1,
            2 => {}
            _ => non_manifold += 1,
        }
    }

    let euler = to_i64(vertices) - to_i64(unique) + to_i64(m.triangles.len());
    MeshMetrics {
        volume: signed_volume_about(m, center(&bbox)),
        area: area(m),
        bbox,
        vertices,
        triangles: m.triangles.len(),
        boundary_edges: boundary,
        non_manifold_edges: non_manifold,
        euler,
    }
}

/// Signed volume by the divergence theorem, mm³. Positive when triangles wind
/// counter-clockwise seen from outside. Only meaningful for closed soups.
///
/// # Panics
/// If a triangle index is out of range.
pub fn signed_volume(m: TriSoup<'_>) -> f64 {
    let bbox = Aabb3::from_points(
        m.triangles
            .iter()
            .flatten()
            .map(|&i| m.positions[i as usize]),
    );
    signed_volume_about(m, center(&bbox))
}

/// Total triangle area, mm².
///
/// # Panics
/// If a triangle index is out of range.
pub fn area(m: TriSoup<'_>) -> f64 {
    (0..m.triangles.len())
        .map(|t| {
            let [a, b, c] = m.corners(t);
            0.5 * (b - a).cross(c - a).length()
        })
        .sum()
}

/// Volume with every vertex shifted by `-o`. The result does not depend on `o` for a closed
/// soup, but a nearby origin keeps the triple products small, which avoids cancellation.
fn signed_volume_about(m: TriSoup<'_>, o: Point3) -> f64 {
    let six: f64 = (0..m.triangles.len())
        .map(|t| {
            let [a, b, c] = m.corners(t);
            (a - o).dot((b - o).cross(c - o))
        })
        .sum();
    six / 6.0
}

fn center(b: &Aabb3) -> Point3 {
    if b.is_empty() {
        Point3::ZERO
    } else {
        b.center()
    }
}

fn to_i64(n: usize) -> i64 {
    i64::try_from(n).expect("count fits in i64")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes;

    #[test]
    fn cube_metrics() {
        let cube = shapes::cube(Point3::ZERO, 10.0);
        let m = metrics(cube.soup());
        assert!((m.volume - 1000.0).abs() <= 1e-9, "volume {}", m.volume);
        assert!((m.area - 600.0).abs() <= 1e-9, "area {}", m.area);
        assert_eq!(m.bbox, Aabb3::new(Point3::ZERO, Point3::splat(10.0)));
        assert_eq!(m.vertices, 8);
        assert_eq!(m.triangles, 12);
        assert_eq!(m.boundary_edges, 0);
        assert_eq!(m.non_manifold_edges, 0);
        assert_eq!(m.euler, 2);
    }

    #[test]
    fn cube_far_from_origin_keeps_precision() {
        let cube = shapes::cube(Point3::new(1.0e5, -2.0e5, 3.0e5), 10.0);
        let v = signed_volume(cube.soup());
        assert!((v - 1000.0).abs() <= 1e-9, "volume {v}");
    }

    #[test]
    fn flipped_cube_has_negative_volume() {
        let mut cube = shapes::cube(Point3::ZERO, 10.0);
        for t in &mut cube.triangles {
            t.swap(1, 2);
        }
        let v = signed_volume(cube.soup());
        assert!((v + 1000.0).abs() <= 1e-9, "volume {v}");
    }

    #[test]
    fn open_and_non_manifold_counts() {
        // One triangle: 3 boundary edges, V - E + F = 3 - 3 + 1 = 1.
        let pos = [Point3::ZERO, Point3::X, Point3::Y, Point3::Z, Point3::NEG_Z];
        let one = TriSoup {
            positions: &pos,
            triangles: &[[0, 1, 2]],
        };
        let m = metrics(one);
        assert_eq!((m.boundary_edges, m.non_manifold_edges, m.euler), (3, 0, 1));
        assert_eq!(m.vertices, 3, "unreferenced vertices are not counted");
        assert!((m.area - 0.5).abs() <= 1e-15);

        // Three fins on edge 0-1: that edge is non-manifold, the 6 others are boundary.
        let fins = TriSoup {
            positions: &pos,
            triangles: &[[0, 1, 2], [1, 0, 3], [0, 1, 4]],
        };
        let m = metrics(fins);
        assert_eq!((m.boundary_edges, m.non_manifold_edges), (6, 1));
        assert_eq!(m.euler, 5 - 7 + 3);
    }

    #[test]
    fn degenerate_triangles_add_no_boundary_edges() {
        // A closed cube plus a sliver [0, 0, 1] on an existing edge: the self-loop is ignored,
        // and 0-1 gains two more uses, so it is non-manifold rather than "boundary".
        let mut cube = shapes::cube(Point3::ZERO, 10.0);
        cube.triangles.push([0, 0, 1]);
        let m = metrics(cube.soup());
        assert_eq!(m.boundary_edges, 0);
        assert!((m.volume - 1000.0).abs() <= 1e-9 && (m.area - 600.0).abs() <= 1e-9);
        // A fully collapsed [3, 3, 3] adds no edge at all.
        let mut cube = shapes::cube(Point3::ZERO, 10.0);
        cube.triangles.push([3, 3, 3]);
        let m = metrics(cube.soup());
        assert_eq!((m.boundary_edges, m.non_manifold_edges), (0, 0));
        assert_eq!(m.euler, 8 - 18 + 13);
    }

    #[test]
    fn empty_soup() {
        let m = metrics(TriSoup {
            positions: &[],
            triangles: &[],
        });
        assert!(m.bbox.is_empty());
        assert_eq!((m.vertices, m.triangles, m.euler), (0, 0, 0));
        assert_eq!(m.volume.abs().to_bits(), 0, "{}", m.volume);
        assert_eq!(
            area(TriSoup {
                positions: &[],
                triangles: &[]
            })
            .abs()
            .to_bits(),
            0
        );
    }

    #[test]
    fn sphere_volume_converges_quadratically() {
        // Each icosphere subdivision quadruples the triangle count; the volume error must at
        // least halve (for a smooth surface it drops ~4x).
        let r = 10.0;
        let exact = 4.0 / 3.0 * std::f64::consts::PI * r * r * r;
        let errs: Vec<f64> = (2..=6)
            .map(|k| {
                let s = shapes::icosphere(Point3::ZERO, r, k);
                (signed_volume(s.soup()) - exact).abs()
            })
            .collect();
        for w in errs.windows(2) {
            let ratio = w[0] / w[1];
            assert!(ratio >= 2.0, "error ratio {ratio} in {errs:?}");
        }
        assert!(errs[errs.len() - 1] / exact < 1e-3, "{errs:?}");
    }

    #[test]
    fn sphere_topology_is_closed_genus_zero() {
        let s = shapes::icosphere(Point3::ZERO, 1.0, 3);
        let m = metrics(s.soup());
        assert_eq!((m.boundary_edges, m.non_manifold_edges, m.euler), (0, 0, 2));
        assert_eq!(m.triangles, 20 * 4usize.pow(3));
    }
}
