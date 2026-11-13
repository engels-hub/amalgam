//! CPU ray casts over the adapter's display meshes: `CpuPickService` (tests) and, later,
//! UI-04's select-through (`PickMode::AllAlongRay`, which an id buffer cannot serve).

use amg_core::intersect::{ray_aabb, ray_triangle};
use amg_core::{Aabb3, Dir3, Point3, Ray3, Transform, Triangle3, Vec3};

use super::tess::DisplayMesh;

const LEAF: usize = 4;

#[derive(Debug)]
enum Kind {
    Leaf { start: u32, end: u32 },
    Inner { left: u32, right: u32 },
}

#[derive(Debug)]
struct Node {
    bounds: Aabb3,
    kind: Kind,
}

/// Median-split bounding volume hierarchy over a display mesh's triangles (mesh frame, with
/// `origin` applied). Built lazily on the first ray cast of a mesh.
#[derive(Debug)]
pub(crate) struct Bvh {
    nodes: Vec<Node>,
    tris: Vec<u32>,
}

fn vertex(m: &DisplayMesh, i: u32) -> Point3 {
    let p = m.data.positions[i as usize];
    m.data.origin + Vec3::new(f64::from(p[0]), f64::from(p[1]), f64::from(p[2]))
}

fn triangle(m: &DisplayMesh, t: u32) -> Option<Triangle3> {
    let i = t as usize * 3;
    let idx = m.data.indices.get(i..i + 3)?;
    if idx.iter().any(|&v| v as usize >= m.data.positions.len()) {
        return None;
    }
    Some(Triangle3 {
        a: vertex(m, idx[0]),
        b: vertex(m, idx[1]),
        c: vertex(m, idx[2]),
    })
}

impl Bvh {
    pub(crate) fn build(m: &DisplayMesh) -> Bvh {
        let n = m.triangle_count();
        let mut items: Vec<(u32, Aabb3, Point3)> = (0..n)
            .filter_map(|t| {
                let tri = triangle(m, t)?;
                let b = Aabb3::from_points([tri.a, tri.b, tri.c]);
                Some((t, b, b.center()))
            })
            .collect();
        let mut bvh = Bvh {
            nodes: Vec::with_capacity(items.len() / LEAF * 2 + 1),
            tris: Vec::with_capacity(items.len()),
        };
        if !items.is_empty() {
            bvh.split(&mut items);
        }
        bvh
    }

    fn split(&mut self, items: &mut [(u32, Aabb3, Point3)]) -> u32 {
        let bounds = items
            .iter()
            .fold(Aabb3::EMPTY, |acc, (_, b, _)| acc.union(b));
        let id = u32::try_from(self.nodes.len()).unwrap_or(u32::MAX);
        if items.len() <= LEAF {
            let start = u32::try_from(self.tris.len()).unwrap_or(u32::MAX);
            self.tris.extend(items.iter().map(|(t, _, _)| *t));
            let end = u32::try_from(self.tris.len()).unwrap_or(u32::MAX);
            self.nodes.push(Node {
                bounds,
                kind: Kind::Leaf { start, end },
            });
            return id;
        }
        let size = bounds.size();
        let axis = if size.x >= size.y && size.x >= size.z {
            0
        } else if size.y >= size.z {
            1
        } else {
            2
        };
        let mid = items.len() / 2;
        items.select_nth_unstable_by(mid, |a, b| a.2[axis].total_cmp(&b.2[axis]));
        self.nodes.push(Node {
            bounds,
            kind: Kind::Leaf { start: 0, end: 0 },
        });
        let (lo, hi) = items.split_at_mut(mid);
        let left = self.split(lo);
        let right = self.split(hi);
        self.nodes[id as usize].kind = Kind::Inner { left, right };
        id
    }

    /// The nearest triangle hit by `ray` (mesh frame): `(triangle, t)`.
    pub(crate) fn nearest(&self, m: &DisplayMesh, ray: &Ray3) -> Option<(u32, f64)> {
        let mut best: Option<(u32, f64)> = None;
        let mut stack: Vec<u32> = Vec::with_capacity(64);
        if !self.nodes.is_empty() {
            stack.push(0);
        }
        while let Some(i) = stack.pop() {
            let node = &self.nodes[i as usize];
            let Some((t0, _)) = ray_aabb(ray, &node.bounds) else {
                continue;
            };
            if best.is_some_and(|(_, bt)| t0 > bt) {
                continue;
            }
            match node.kind {
                Kind::Leaf { start, end } => {
                    for &t in &self.tris[start as usize..end as usize] {
                        let Some(tri) = triangle(m, t) else { continue };
                        if let Some(h) = ray_triangle(ray, &tri)
                            && best.is_none_or(|(_, bt)| h.t < bt)
                        {
                            best = Some((t, h.t));
                        }
                    }
                }
                Kind::Inner { left, right } => {
                    stack.push(right);
                    stack.push(left);
                }
            }
        }
        best
    }
}

/// `ray` (world) in the frame of an instance with world transform `world`. `None` for a
/// singular transform.
pub(crate) fn to_local(ray: &Ray3, world: &Transform) -> Option<Ray3> {
    if world.matrix3.determinant().abs() < 1e-300 {
        return None;
    }
    let inv = world.inverse();
    let origin = inv.transform_point3(ray.origin);
    let d = inv.transform_vector3(ray.dir.get());
    let len = d.length();
    if !len.is_finite() || len == 0.0 {
        return None;
    }
    Some(Ray3 {
        origin,
        dir: Dir3::new_unchecked(d / len),
    })
}
