//! Self-intersection detection (MESH-19): BVH self-traversal plus exact predicates.
//!
//! [`self_intersections`] returns every pair of faces `[a, b]` (`a < b`) of a mesh that
//! intersect. The broad phase is [`crate::Bvh::par_overlapping_pairs`] over the exact f64 face
//! boxes; the narrow phase is an exact closed triangle-triangle test on Shewchuk's adaptive
//! predicates (the `robust` crate), so no result depends on a tolerance.
//!
//! Semantics (vertices are shared *by index*, so weld first for a meaningful answer):
//! - faces with no shared vertex intersect when the closed triangles share any point (touching
//!   counts, so a T-junction is reported);
//! - faces sharing one vertex intersect when they meet anywhere but that vertex;
//! - faces sharing an edge intersect when they are coplanar and overlap (a fold-over);
//! - faces with all three vertices in common (duplicates) intersect;
//! - exactly degenerate (zero-area) faces are skipped; MESH-02 reports them separately.

mod exact;
mod repair;

pub use repair::{
    RemoveSelfIntersectionsOpts, RemoveSelfIntersectionsReport, remove_self_intersections,
    remove_self_intersections_with,
};

use amg_core::Aabb3;
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::FaceId;
use crate::error::MeshResult;
use crate::indexed::IndexedMesh;
use crate::spatial::MeshSpatial;
use exact::{DEGENERATE, Tri, drop_axis, pair_intersects};

/// Options for [`self_intersections`].
#[derive(Clone, Debug, Default)]
pub struct SelfIntersectOpts {
    /// Stop early: return a deterministic prefix of at most this many pairs (the pairs found
    /// first in BVH leaf order, then sorted). `None` finds all.
    pub max_pairs: Option<usize>,
}

fn tri_of(m: &IndexedMesh, f: u32, drop: u8) -> Tri {
    let t = m.tris[f as usize];
    Tri {
        v: t.map(|i| m.positions[i as usize]),
        drop,
    }
}

/// Exact test for a single pair of faces (same semantics as [`self_intersections`]).
/// Degenerate faces never intersect anything.
///
/// # Panics
/// If a face id is out of range.
pub fn faces_intersect(m: &IndexedMesh, a: FaceId, b: FaceId) -> bool {
    if a == b {
        return false;
    }
    let ta = m.tris[a.0 as usize].map(|i| m.positions[i as usize]);
    let tb = m.tris[b.0 as usize].map(|i| m.positions[i as usize]);
    let (da, db) = (
        drop_axis(ta[0], ta[1], ta[2]),
        drop_axis(tb[0], tb[1], tb[2]),
    );
    if da == DEGENERATE || db == DEGENERATE {
        return false;
    }
    pair_intersects(
        &tri_of(m, a.0, da),
        m.tris[a.0 as usize],
        &tri_of(m, b.0, db),
        m.tris[b.0 as usize],
    )
}

/// All intersecting face pairs `[a, b]` with `a < b`, sorted ascending. See the module docs
/// for what counts as an intersection.
///
/// Parallel and deterministic: the result is identical for any rayon thread count.
///
/// # Errors
/// [`crate::MeshError::Cancelled`].
pub fn self_intersections(
    s: &MeshSpatial,
    opts: &SelfIntersectOpts,
    ctx: &JobCtx,
) -> MeshResult<Vec<[FaceId; 2]>> {
    let m: &IndexedMesh = s.mesh();
    let n = m.tris.len();
    let drops: Vec<u8> = m
        .tris
        .par_iter()
        .map(|t| {
            let p = t.map(|i| m.positions[i as usize]);
            drop_axis(p[0], p[1], p[2])
        })
        .collect();
    ctx.checkpoint()?;
    // Exact boxes (the BVH keeps only outward-rounded f32 boxes). Degenerate faces get an
    // empty-in-effect box far from everything by simply being filtered in the narrow phase.
    let bounds: Vec<Aabb3> = (0..n)
        .into_par_iter()
        .map(|f| Aabb3::from_points(m.tris[f].map(|i| m.positions[i as usize])))
        .collect();
    ctx.checkpoint()?;
    let mut pairs = s.bvh().par_overlapping_pairs(
        &bounds,
        opts.max_pairs,
        ctx,
        |a: u32, b: u32, out: &mut Vec<[FaceId; 2]>| {
            let (da, db) = (drops[a as usize], drops[b as usize]);
            if da == DEGENERATE || db == DEGENERATE {
                return;
            }
            if pair_intersects(
                &tri_of(m, a, da),
                m.tris[a as usize],
                &tri_of(m, b, db),
                m.tris[b as usize],
            ) {
                out.push([FaceId(a), FaceId(b)]);
            }
        },
    )?;
    pairs.sort_unstable();
    Ok(pairs)
}
