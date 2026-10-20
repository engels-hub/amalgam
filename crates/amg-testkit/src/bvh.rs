//! Internal bounding volume hierarchy over the triangles of a [`TriSoup`], answering
//! closest-point queries for [`crate::hausdorff`].
//!
//! Median split on the longest centroid axis, leaves of at most [`LEAF`] triangles. Nodes are
//! stored depth first; an inner node's left child is the next node and its right child sits
//! `right` nodes further on. Because that offset is relative, subtrees built in parallel are
//! simply concatenated.

use amg_core::{Point3, Vec3};

use crate::TriSoup;

const LEAF: usize = 4;
/// Below this many triangles a subtree is built on the current thread.
const PAR_THRESHOLD: usize = 1 << 14;

#[derive(Clone, Copy, Debug)]
struct Node {
    min: Point3,
    max: Point3,
    /// Leaf: first triangle in `Bvh::tris`. Inner: offset from this node to its right child.
    a: u32,
    /// Leaf: triangle count (> 0). Inner: 0.
    count: u32,
}

#[derive(Debug)]
pub(crate) struct Bvh<'a> {
    positions: &'a [Point3],
    /// Triangles reordered so every leaf is a contiguous range.
    tris: Vec<[u32; 3]>,
    nodes: Vec<Node>,
}

impl<'a> Bvh<'a> {
    /// Builds the tree. Panics if a triangle index is out of range.
    pub(crate) fn new(m: TriSoup<'a>) -> Bvh<'a> {
        let mut items: Vec<Item> = (0..m.triangles.len())
            .map(|t| {
                let [a, b, c] = m.corners(t);
                Item {
                    tri: m.triangles[t],
                    min: a.min(b).min(c),
                    max: a.max(b).max(c),
                    centroid: (a + b + c) / 3.0,
                }
            })
            .collect();
        let nodes = if items.is_empty() {
            Vec::new()
        } else {
            build(&mut items, 0)
        };
        Bvh {
            positions: m.positions,
            tris: items.into_iter().map(|i| i.tri).collect(),
            nodes,
        }
    }

    /// Squared distance from `p` to the nearest triangle; `+inf` for an empty soup.
    pub(crate) fn closest_dist2(&self, p: Point3) -> f64 {
        let mut best = f64::INFINITY;
        if self.nodes.is_empty() {
            return best;
        }
        let mut stack: Vec<usize> = Vec::with_capacity(64);
        let mut node = 0usize;
        loop {
            let n = &self.nodes[node];
            if n.count > 0 {
                let start = n.a as usize;
                for tri in &self.tris[start..start + n.count as usize] {
                    let [a, b, c] = tri.map(|i| self.positions[i as usize]);
                    best = best.min(point_triangle_dist2(p, a, b, c));
                }
            } else {
                let (l, r) = (node + 1, node + n.a as usize);
                let (dl, dr) = (self.box_dist2(l, p), self.box_dist2(r, p));
                let (near, dn, far, df) = if dl <= dr {
                    (l, dl, r, dr)
                } else {
                    (r, dr, l, dl)
                };
                if df < best {
                    stack.push(far);
                }
                if dn < best {
                    node = near;
                    continue;
                }
            }
            // Pop until a node that can still beat `best`.
            loop {
                match stack.pop() {
                    None => return best,
                    Some(s) if self.box_dist2(s, p) < best => {
                        node = s;
                        break;
                    }
                    Some(_) => {}
                }
            }
        }
    }

    fn box_dist2(&self, node: usize, p: Point3) -> f64 {
        let n = &self.nodes[node];
        let d = (n.min - p).max(Vec3::ZERO).max(p - n.max);
        d.length_squared()
    }
}

#[derive(Clone, Copy, Debug)]
struct Item {
    tri: [u32; 3],
    min: Point3,
    max: Point3,
    centroid: Point3,
}

/// Builds the subtree over `items`, whose first element is triangle `offset` overall.
fn build(items: &mut [Item], offset: usize) -> Vec<Node> {
    let (mut min, mut max) = (Point3::INFINITY, Point3::NEG_INFINITY);
    let (mut cmin, mut cmax) = (Point3::INFINITY, Point3::NEG_INFINITY);
    for it in items.iter() {
        min = min.min(it.min);
        max = max.max(it.max);
        cmin = cmin.min(it.centroid);
        cmax = cmax.max(it.centroid);
    }
    let extent = cmax - cmin;
    let n = items.len();
    if n <= LEAF || extent.max_element() <= 0.0 {
        return vec![Node {
            min,
            max,
            a: to_u32(offset),
            count: to_u32(n),
        }];
    }
    let axis = if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    };
    let mid = n / 2;
    items.select_nth_unstable_by(mid, |a, b| a.centroid[axis].total_cmp(&b.centroid[axis]));
    let (lo, hi) = items.split_at_mut(mid);
    let (left, right) = if n >= PAR_THRESHOLD {
        rayon::join(|| build(lo, offset), || build(hi, offset + mid))
    } else {
        (build(lo, offset), build(hi, offset + mid))
    };
    let mut nodes = Vec::with_capacity(1 + left.len() + right.len());
    nodes.push(Node {
        min,
        max,
        a: to_u32(1 + left.len()),
        count: 0,
    });
    nodes.extend(left);
    nodes.extend(right);
    nodes
}

fn to_u32(n: usize) -> u32 {
    u32::try_from(n).expect("BVH sizes fit in u32")
}

/// Squared distance from `p` to triangle `abc` (Ericson, *Real-Time Collision Detection*,
/// §5.1.5). Degenerate triangles fall back to the nearest edge or vertex.
pub(crate) fn point_triangle_dist2(p: Point3, a: Point3, b: Point3, c: Point3) -> f64 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.length_squared();
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return bp.length_squared();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (p - (a + v * ab)).length_squared();
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return cp.length_squared();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (p - (a + w * ac)).length_squared();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + w * (c - b))).length_squared();
    }
    let sum = va + vb + vc;
    if sum <= 0.0 || !sum.is_finite() {
        // Degenerate (zero-area) triangle: nearest of the three edges.
        return seg_dist2(p, a, b)
            .min(seg_dist2(p, b, c))
            .min(seg_dist2(p, c, a));
    }
    let denom = 1.0 / sum;
    let v = vb * denom;
    let w = vc * denom;
    (p - (a + ab * v + ac * w)).length_squared()
}

fn seg_dist2(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let len2 = ab.length_squared();
    let t = if len2 > 0.0 {
        ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + t * ab)).length_squared()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes;

    fn brute(m: TriSoup<'_>, p: Point3) -> f64 {
        (0..m.triangles.len())
            .map(|t| {
                let [a, b, c] = m.corners(t);
                point_triangle_dist2(p, a, b, c)
            })
            .fold(f64::INFINITY, f64::min)
    }

    #[test]
    fn point_triangle_regions() {
        let (a, b, c) = (Point3::ZERO, Point3::X, Point3::Y);
        let d = |p: Point3| point_triangle_dist2(p, a, b, c);
        assert!((d(Point3::new(0.2, 0.2, 3.0)) - 9.0).abs() < 1e-12, "face");
        assert!(
            (d(Point3::new(-1.0, -1.0, 0.0)) - 2.0).abs() < 1e-12,
            "vertex a"
        );
        assert!(
            (d(Point3::new(2.0, 0.0, 0.0)) - 1.0).abs() < 1e-12,
            "vertex b"
        );
        assert!(
            (d(Point3::new(0.0, 3.0, 0.0)) - 4.0).abs() < 1e-12,
            "vertex c"
        );
        assert!(
            (d(Point3::new(0.5, -2.0, 0.0)) - 4.0).abs() < 1e-12,
            "edge ab"
        );
        assert!(
            (d(Point3::new(-2.0, 0.5, 0.0)) - 4.0).abs() < 1e-12,
            "edge ca"
        );
        assert!(
            (d(Point3::new(1.0, 1.0, 0.0)) - 0.5).abs() < 1e-12,
            "edge bc"
        );
        // Degenerate: all three on a line.
        let dd = point_triangle_dist2(Point3::new(0.5, 1.0, 0.0), a, b, Point3::new(2.0, 0.0, 0.0));
        assert!((dd - 1.0).abs() < 1e-12);
    }

    #[test]
    fn bvh_matches_brute_force() {
        let s = shapes::icosphere(Point3::new(1.0, -2.0, 0.5), 3.0, 3);
        let bvh = Bvh::new(s.soup());
        let mut rng = crate::hausdorff::SplitMix64::new(7);
        for _ in 0..300 {
            let p = Point3::new(rng.unit(), rng.unit(), rng.unit()) * 12.0 - Vec3::splat(6.0);
            let (fast, slow) = (bvh.closest_dist2(p), brute(s.soup(), p));
            assert!(
                (fast - slow).abs() <= 1e-12 * (1.0 + slow),
                "{p}: {fast} vs {slow}"
            );
        }
    }

    #[test]
    fn empty_bvh_is_infinitely_far() {
        let bvh = Bvh::new(TriSoup {
            positions: &[],
            triangles: &[],
        });
        assert!(bvh.closest_dist2(Point3::ZERO).is_infinite());
    }
}
