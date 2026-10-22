//! Spatial queries on triangle meshes (MESH-03): [`Bvh`] and [`MeshSpatial`].
//!
//! - [`Bvh`]: binned-SAH hierarchy, 32-byte nodes with outward-rounded f32 bounds.
//! - [`MeshSpatial`]: watertight ray cast, closest point, box query, fast and exact
//!   generalized winding number, signed distance.
//!
//! Every query is a pure function of the mesh and the arguments, so results are identical for
//! any rayon thread count.

mod bvh;
mod pairs;
mod tri;
mod winding;

use std::sync::{Arc, OnceLock};

use amg_core::{Aabb3, Point3, Ray3, Vec3};
use amg_jobs::JobCtx;
use rayon::prelude::*;

pub use bvh::{Bvh, BvhNode, BvhNodeKind, BvhOpts};
pub use tri::TriFeature;

use bvh::STACK;
use tri::{RayPre, closest_on_tri, solid_angle_over_4pi};
use winding::WindingTree;

use crate::FaceId;
use crate::error::MeshResult;
use crate::indexed::IndexedMesh;

/// Default accuracy parameter of the fast winding number.
pub const DEFAULT_WINDING_BETA: f64 = 2.0;

/// A ray–mesh intersection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    /// Distance along the (unit) ray direction, mm.
    pub t: f64,
    /// Face that was hit.
    pub face: FaceId,
    /// Barycentric weights of the face's three corners at the hit.
    pub bary: Vec3,
}

/// Closest point on a mesh.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosestPoint {
    /// The point on the surface.
    pub point: Point3,
    /// Face holding it (lowest id on ties).
    pub face: FaceId,
    /// Barycentric weights of the face's three corners.
    pub bary: Vec3,
    /// Squared distance from the query point, mm².
    pub dist_sq: f64,
    /// Vertex / edge / interior of `face` the point lies on.
    pub feature: TriFeature,
}

/// Ize (2013) padding factor `1 + 2γ₃` applied to the far slab distance.
const SLAB_PAD: f64 = 1.0 + 2.0 * (3.0 * f64::EPSILON * 0.5) / (1.0 - 3.0 * f64::EPSILON * 0.5);

/// Mesh plus acceleration structures.
#[derive(Debug)]
pub struct MeshSpatial {
    mesh: Arc<IndexedMesh>,
    bvh: Bvh,
    winding: OnceLock<WindingTree>,
}

impl MeshSpatial {
    /// Builds the BVH over `mesh`'s faces. The winding-number tree is built lazily on the
    /// first winding query (or by [`MeshSpatial::prepare_winding`]). An empty mesh is allowed
    /// and answers every query with `None` / 0.
    ///
    /// # Errors
    /// [`crate::MeshError::InvalidOption`] for bad options or non-finite positions,
    /// [`crate::MeshError::Cancelled`].
    pub fn build(mesh: Arc<IndexedMesh>, opts: &BvhOpts, ctx: &JobCtx) -> MeshResult<Self> {
        let m: &IndexedMesh = &mesh;
        let bounds: Vec<Aabb3> = m
            .tris
            .par_iter()
            .map(|t| Aabb3::from_points(t.map(|i| m.positions[i as usize])))
            .collect();
        ctx.checkpoint()?;
        let bvh = Bvh::build(&bounds, opts, ctx)?;
        Ok(Self {
            mesh,
            bvh,
            winding: OnceLock::new(),
        })
    }

    /// The mesh.
    pub fn mesh(&self) -> &Arc<IndexedMesh> {
        &self.mesh
    }

    /// The hierarchy (primitive `i` is face `i`).
    pub fn bvh(&self) -> &Bvh {
        &self.bvh
    }

    #[inline]
    fn corners(&self, prim: u32) -> [Point3; 3] {
        self.mesh.tris[prim as usize].map(|i| self.mesh.positions[i as usize])
    }

    /// Nearest hit with `0 <= t <= t_max` (`t` in mm, `ray.dir` is a unit vector). Both
    /// faces are hit. A ray through a shared edge may report either adjacent face; the lowest
    /// face id wins exact `t` ties.
    #[allow(
        clippy::float_cmp,
        reason = "exact ties are broken by face id for determinism"
    )]
    pub fn ray_cast(&self, ray: &Ray3, t_max: f64) -> Option<RayHit> {
        if self.bvh.nodes.is_empty() {
            return None;
        }
        let dir = ray.dir.get();
        let pre = RayPre::new(ray.origin, dir);
        let inv = inv_dir(dir);
        let mut best_t = t_max;
        let mut best: Option<RayHit> = None;
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let i = stack[sp];
            let n = &self.bvh.nodes[i as usize];
            if slab(n.min64(), n.max64(), ray.origin, inv, best_t).is_none() {
                continue;
            }
            if n.is_leaf() {
                let f = n.left_or_first as usize;
                for &prim in &self.bvh.prim_indices[f..f + n.count as usize] {
                    let [a, b, c] = self.corners(prim);
                    if let Some((t, bary)) = pre.hit(a, b, c) {
                        let better =
                            t < best_t || (t == best_t && best.is_none_or(|h| prim < h.face.0));
                        if better {
                            best_t = t;
                            best = Some(RayHit {
                                t,
                                face: FaceId(prim),
                                bary,
                            });
                        }
                    }
                }
            } else {
                let (l, r) = (i + 1, n.left_or_first);
                let tl = self.child_entry(l, ray.origin, inv, best_t);
                let tr = self.child_entry(r, ray.origin, inv, best_t);
                match (tl, tr) {
                    (Some(a), Some(b)) => {
                        // Push the farther child first so the nearer pops first.
                        if a <= b {
                            stack[sp] = r;
                            stack[sp + 1] = l;
                        } else {
                            stack[sp] = l;
                            stack[sp + 1] = r;
                        }
                        sp += 2;
                    }
                    (Some(_), None) => {
                        stack[sp] = l;
                        sp += 1;
                    }
                    (None, Some(_)) => {
                        stack[sp] = r;
                        sp += 1;
                    }
                    (None, None) => {}
                }
            }
        }
        best
    }

    #[inline]
    fn child_entry(&self, i: u32, o: Point3, inv: Vec3, t_max: f64) -> Option<f64> {
        let n = &self.bvh.nodes[i as usize];
        slab(n.min64(), n.max64(), o, inv, t_max)
    }

    /// Appends all hits with `0 <= t <= t_max` to `out`, sorted by `(t, face)`. A ray through
    /// a shared edge or vertex can report each adjacent face.
    pub fn ray_cast_all(&self, ray: &Ray3, t_max: f64, out: &mut Vec<RayHit>) {
        if self.bvh.nodes.is_empty() {
            return;
        }
        let start = out.len();
        let dir = ray.dir.get();
        let pre = RayPre::new(ray.origin, dir);
        let inv = inv_dir(dir);
        let mut stack = [0u32; STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let i = stack[sp];
            let n = &self.bvh.nodes[i as usize];
            if slab(n.min64(), n.max64(), ray.origin, inv, t_max).is_none() {
                continue;
            }
            if n.is_leaf() {
                let f = n.left_or_first as usize;
                for &prim in &self.bvh.prim_indices[f..f + n.count as usize] {
                    let [a, b, c] = self.corners(prim);
                    if let Some((t, bary)) = pre.hit(a, b, c)
                        && t <= t_max
                    {
                        out.push(RayHit {
                            t,
                            face: FaceId(prim),
                            bary,
                        });
                    }
                }
            } else {
                stack[sp] = n.left_or_first;
                stack[sp + 1] = i + 1;
                sp += 2;
            }
        }
        out[start..].sort_by(|a, b| a.t.total_cmp(&b.t).then(a.face.cmp(&b.face)));
    }

    /// Closest surface point to `p` within `max_dist` (inclusive), or `None`.
    #[allow(
        clippy::float_cmp,
        reason = "exact ties are broken by face id for determinism"
    )]
    pub fn closest_point(&self, p: Point3, max_dist: f64) -> Option<ClosestPoint> {
        if self.bvh.nodes.is_empty() || p.is_nan() {
            return None;
        }
        let mut best_d2 = max_dist * max_dist;
        let mut best: Option<ClosestPoint> = None;
        let mut stack = [(0u32, 0.0f64); STACK];
        let mut sp = 1;
        while sp > 0 {
            sp -= 1;
            let (i, d2) = stack[sp];
            if d2 > best_d2 {
                continue;
            }
            let n = &self.bvh.nodes[i as usize];
            if n.is_leaf() {
                let f = n.left_or_first as usize;
                for &prim in &self.bvh.prim_indices[f..f + n.count as usize] {
                    let [a, b, c] = self.corners(prim);
                    let (q, bary, feature) = closest_on_tri(p, a, b, c);
                    let d = (q - p).length_squared();
                    if d < best_d2 || (d == best_d2 && best.is_none_or(|h| prim < h.face.0)) {
                        best_d2 = d;
                        best = Some(ClosestPoint {
                            point: q,
                            face: FaceId(prim),
                            bary,
                            dist_sq: d,
                            feature,
                        });
                    }
                }
            } else {
                let (l, r) = (i + 1, n.left_or_first);
                let dl = box_dist_sq(&self.bvh.nodes[l as usize], p);
                let dr = box_dist_sq(&self.bvh.nodes[r as usize], p);
                let (near, far) = if dl <= dr {
                    ((l, dl), (r, dr))
                } else {
                    ((r, dr), (l, dl))
                };
                if far.1 <= best_d2 {
                    stack[sp] = far;
                    sp += 1;
                }
                if near.1 <= best_d2 {
                    stack[sp] = near;
                    sp += 1;
                }
            }
        }
        best
    }

    /// Appends the faces whose bounding box overlaps `b` (inclusive) to `out`, in ascending
    /// face order.
    pub fn faces_in_aabb(&self, b: &Aabb3, out: &mut Vec<FaceId>) {
        let start = out.len();
        self.bvh.query_aabb(b, &mut |prim| {
            let [p, q, r] = self.corners(prim);
            let lo = p.min(q).min(r);
            let hi = p.max(q).max(r);
            if !(lo.cmpgt(b.max).any() || hi.cmplt(b.min).any()) {
                out.push(FaceId(prim));
            }
        });
        out[start..].sort_unstable();
    }

    /// Builds the winding-number tree now (cancellable); a no-op when it already exists.
    ///
    /// # Errors
    /// [`crate::MeshError::Cancelled`].
    pub fn prepare_winding(&self, ctx: &JobCtx) -> MeshResult<()> {
        if self.winding.get().is_some() {
            return Ok(());
        }
        let tree = WindingTree::build(&self.bvh, &self.mesh, ctx)?;
        let _ = self.winding.set(tree);
        Ok(())
    }

    fn winding_tree(&self) -> &WindingTree {
        self.winding.get_or_init(|| {
            // A detached context is never cancelled, so this cannot fail.
            WindingTree::build(&self.bvh, &self.mesh, &JobCtx::detached())
                .unwrap_or_else(|_| unreachable!("detached context cannot be cancelled"))
        })
    }

    /// Fast generalized winding number (Barill et al. 2018, order-2 dipoles, β = 2): about 1
    /// inside a closed outward-oriented surface, 0 outside, fractional near open surfaces.
    /// Accurate to ~0.01 for points at least one mean edge length off the surface.
    pub fn winding_number(&self, p: Point3) -> f64 {
        self.winding_number_with_beta(p, DEFAULT_WINDING_BETA)
    }

    /// Fast winding number with accuracy parameter `beta` (larger is slower and more exact).
    pub fn winding_number_with_beta(&self, p: Point3, beta: f64) -> f64 {
        self.winding_tree().eval(&self.bvh, &self.mesh, p, beta)
    }

    /// Fast winding numbers for many points, in parallel; `out[i]` is for `pts[i]`.
    pub fn winding_numbers(&self, pts: &[Point3], beta: f64) -> Vec<f64> {
        self.winding_tree()
            .eval_batch(&self.bvh, &self.mesh, pts, beta)
    }

    /// Exact winding number: O(n) sum of solid angles (reference for the fast version).
    pub fn winding_number_exact(&self, p: Point3) -> f64 {
        (0..self.mesh.tris.len() as u32)
            .map(|f| {
                let [a, b, c] = self.corners(f);
                solid_angle_over_4pi(p, a, b, c)
            })
            .sum()
    }

    /// True if the fast winding number exceeds 0.5.
    pub fn is_inside(&self, p: Point3) -> bool {
        self.winding_number(p) > 0.5
    }

    /// Signed distance: `|d|` from the closest point, negative inside (winding number > 0.5)
    /// and positive outside; `None` when the surface is farther than `max_dist`.
    pub fn signed_distance(&self, p: Point3, max_dist: f64) -> Option<f64> {
        let c = self.closest_point(p, max_dist)?;
        let d = c.dist_sq.sqrt();
        Some(if self.is_inside(p) { -d } else { d })
    }
}

/// `1/d` per axis; zero components become a huge finite value so `0 * inv` is never NaN.
#[inline]
fn inv_dir(d: Vec3) -> Vec3 {
    let f = |x: f64| if x == 0.0 { f64::MAX } else { 1.0 / x };
    Vec3::new(f(d.x), f(d.y), f(d.z))
}

/// Slab test (Ize 2013 padding); returns the entry distance if the ray overlaps the box
/// within `[0, t_max]`.
#[inline]
fn slab(lo: Point3, hi: Point3, o: Point3, inv: Vec3, t_max: f64) -> Option<f64> {
    let mut t0 = 0.0f64;
    let mut t1 = t_max;
    for a in 0..3 {
        let ta = (lo[a] - o[a]) * inv[a];
        let tb = (hi[a] - o[a]) * inv[a];
        let (near, far) = if ta < tb { (ta, tb) } else { (tb, ta) };
        t0 = t0.max(near);
        t1 = t1.min(far * SLAB_PAD);
    }
    (t0 <= t1).then_some(t0)
}

#[inline]
fn box_dist_sq(n: &bvh::RawNode, p: Point3) -> f64 {
    let d = (n.min64() - p).max(p - n.max64()).max(Point3::ZERO);
    d.length_squared()
}
