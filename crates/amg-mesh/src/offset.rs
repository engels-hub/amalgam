//! Quick shell (MESH-10, part 1): vertex-normal offset plus a stitched rim.
//!
//! [`offset_shell_normal`] copies the surface, moves one or both copies along the
//! angle-weighted vertex normals and closes every boundary loop with a rim of quads. It is
//! fast (linear, mostly parallel) but not robust: where the thickness exceeds the local
//! radius of curvature, or at concave creases, the offset surface folds over or crosses the
//! other side. [`ShellReport::self_intersecting_pairs`] says how often (the exact MESH-19
//! count on the output). The robust alternative is `amg_features::mesh::shell` (SDF based).
//!
//! # Geometry
//! - Normal: angle-weighted (Thürmer–Wüthrich) vertex normal `n`.
//! - Distance: `d · n / c` with `c = min over incident faces of (n · n_face)`, clamped to
//!   `[1/MAX_STRETCH, 1]`. So a flat region moves by exactly `d`, and the faces at a convex
//!   or concave crease of a box move by `d` along their own normals (cube corner:
//!   `c = 1/√3`) instead of `d · cos`, which keeps the wall thickness at prismatic features.
//! - [`ShellSide::Inside`]: the outer copy is the input, the inner one moves by `-t`.
//!   [`ShellSide::Outside`]: the inner copy is the input, the outer one moves by `+t`.
//!   [`ShellSide::Both`]: `±t/2`.
//! - The inner copy is reversed, so the result is consistently oriented (normals point away
//!   from the material). A closed input gives two closed shells (a hollow body); an open input
//!   (or one with `open_faces` removed) gives one shell whose boundary loops are joined by the
//!   rim (`b, a, a'` / `b, a', b'` for a boundary half-edge `a → b`).
//!
//! # Groups and attributes
//! The copy that coincides with the input (the outer one for `Inside` and `Both`, the inner
//! one for `Outside`) keeps the input's face groups; the offset copy is one new group
//! `GroupOrigin::Shell { side }` with `side` its direction (`Inside` / `Outside`), and the rim
//! is one new group `GroupOrigin::Shell { side: Both }`. Vertex attributes (normals, colours,
//! corner UVs, 3MF properties) are not carried over.

use amg_core::{Point3, Vec3};
use amg_jobs::JobCtx;
use rayon::prelude::*;
use std::sync::Arc;

use crate::error::{MeshError, MeshResult};
use crate::groups::{FaceGroupInfo, GroupOrigin, ShellSide};
use crate::indexed::IndexedMesh;
use crate::selfint::{SelfIntersectOpts, self_intersections};
use crate::spatial::{BvhOpts, MeshSpatial};
use crate::{FaceId, INVALID};

/// Largest factor the per-vertex offset is stretched by to keep face planes at the requested
/// distance (a cube corner needs √3; sharper cones are clamped).
pub const MAX_STRETCH: f64 = 2.0;

/// Options for [`offset_shell_normal`].
#[derive(Clone, Debug)]
pub struct NormalOffsetOpts {
    /// Wall thickness, mm (finite, > 0).
    pub thickness: f64,
    /// Which side of the input the wall goes to.
    pub side: ShellSide,
    /// Faces removed before offsetting, to make an open shell (out-of-range ids are an error).
    pub open_faces: Vec<FaceId>,
}

/// Outcome of [`offset_shell_normal`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ShellReport {
    /// Intersecting face pairs of the output (MESH-19 exact count; touching counts).
    pub self_intersecting_pairs: usize,
}

/// Quick normal-offset shell of `m`. See the module docs.
///
/// # Errors
/// [`MeshError::Empty`] when no face is left after removing `open_faces`,
/// [`MeshError::InvalidOption`] for a non-finite or non-positive thickness or an out-of-range
/// face id, [`MeshError::TooLarge`] when the output exceeds the `u32` index range,
/// [`MeshError::Cancelled`].
pub fn offset_shell_normal(
    m: &IndexedMesh,
    opts: &NormalOffsetOpts,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, ShellReport)> {
    let out = offset_shell_normal_unchecked(m, opts, &ctx.sub(0.0, 0.3))?;
    let s = MeshSpatial::build(Arc::new(out), &BvhOpts::default(), &ctx.sub(0.3, 0.6))?;
    let pairs = self_intersections(&s, &SelfIntersectOpts::default(), &ctx.sub(0.6, 1.0))?;
    let arc = Arc::clone(s.mesh());
    drop(s);
    ctx.progress(1.0);
    Ok((
        Arc::unwrap_or_clone(arc),
        ShellReport {
            self_intersecting_pairs: pairs.len(),
        },
    ))
}

/// [`offset_shell_normal`] without the self-intersection count: the same mesh, built in
/// about a quarter of the time (the MESH-19 count over the doubled output dominates).
///
/// # Errors
/// As [`offset_shell_normal`].
pub fn offset_shell_normal_unchecked(
    m: &IndexedMesh,
    opts: &NormalOffsetOpts,
    ctx: &JobCtx,
) -> MeshResult<IndexedMesh> {
    let t = opts.thickness;
    if !(t.is_finite() && t > 0.0) {
        return Err(MeshError::InvalidOption {
            name: "thickness",
            reason: format!("must be finite and > 0, got {t}"),
        });
    }
    let base = prepare(m, &opts.open_faces)?;
    ctx.checkpoint()?;
    let (nv, nf) = (base.positions.len(), base.tris.len());
    let out_v = 2 * nv;
    if out_v >= INVALID as usize || 2 * nf + 2 * 3 * nf >= INVALID as usize {
        return Err(MeshError::TooLarge { count: out_v });
    }

    let dirs = offset_directions(&base);
    ctx.checkpoint()?;
    ctx.progress(0.3);
    let (d_out, d_in) = match opts.side {
        ShellSide::Inside => (0.0, -t),
        ShellSide::Outside => (t, 0.0),
        ShellSide::Both => (0.5 * t, -0.5 * t),
    };
    let moved = |d: f64| -> Vec<Point3> {
        if d == 0.0 {
            return base.positions.clone();
        }
        base.positions
            .par_iter()
            .zip(dirs.par_iter())
            .map(|(p, n)| *p + *n * d)
            .collect()
    };
    let mut positions = moved(d_out);
    positions.extend(moved(d_in));
    ctx.checkpoint()?;

    let boundary = boundary_half_edges(&base.tris);
    ctx.progress(0.6);
    let nvu = nv as u32;
    let mut tris = Vec::with_capacity(2 * nf + 2 * boundary.len());
    tris.extend_from_slice(&base.tris);
    tris.extend(
        base.tris
            .iter()
            .map(|&[a, b, c]| [a + nvu, c + nvu, b + nvu]),
    );
    for &(a, b) in &boundary {
        tris.push([b, a, a + nvu]);
        tris.push([b, a + nvu, b + nvu]);
    }

    // Groups: the copy on the input keeps the input groups.
    let mut groups = base.groups.clone();
    let offset_side = match opts.side {
        ShellSide::Outside => ShellSide::Outside,
        ShellSide::Inside | ShellSide::Both => ShellSide::Inside,
    };
    let g_off = groups.try_alloc(FaceGroupInfo::new(GroupOrigin::Shell { side: offset_side }))?;
    let g_rim = if boundary.is_empty() {
        None
    } else {
        Some(groups.try_alloc(FaceGroupInfo::new(GroupOrigin::Shell {
            side: ShellSide::Both,
        }))?)
    };
    let input: Vec<_> = base.groups.per_face().to_vec();
    let offset_copy = vec![g_off; nf];
    let (outer, inner) = match opts.side {
        ShellSide::Outside => (&offset_copy, &input),
        ShellSide::Inside | ShellSide::Both => (&input, &offset_copy),
    };
    groups.retain(&vec![false; nf]);
    for &g in outer.iter().chain(inner.iter()) {
        groups.push(g);
    }
    if let Some(g) = g_rim {
        for _ in 0..2 * boundary.len() {
            groups.push(g);
        }
    }
    let mut out = IndexedMesh::new(positions, tris)?;
    out.groups = groups;
    ctx.checkpoint()?;
    ctx.progress(1.0);
    Ok(out)
}

/// The input without `open` faces, unreferenced vertices and attributes.
fn prepare(m: &IndexedMesh, open: &[FaceId]) -> MeshResult<IndexedMesh> {
    let nf = m.tris.len();
    let mut keep = vec![true; nf];
    for f in open {
        match keep.get_mut(f.0 as usize) {
            Some(k) => *k = false,
            None => {
                return Err(MeshError::InvalidOption {
                    name: "open_faces",
                    reason: format!("face {} out of range (mesh has {nf} faces)", f.0),
                });
            }
        }
    }
    let mut b = IndexedMesh::new(m.positions.clone(), m.tris.clone())?;
    b.groups = m.groups.clone();
    b.groups.ensure_len(nf);
    if !open.is_empty() {
        b.groups.retain(&keep);
        let mut i = 0;
        b.tris.retain(|_| {
            i += 1;
            keep[i - 1]
        });
    }
    if b.tris.is_empty() {
        return Err(MeshError::Empty);
    }
    b.compact();
    Ok(b)
}

/// Per vertex: the angle-weighted unit normal divided by the clamped minimum cosine to the
/// incident face normals (see the module docs). Zero where the normal cancels.
fn offset_directions(m: &IndexedMesh) -> Vec<Vec3> {
    let nv = m.positions.len();
    let face_n: Vec<Vec3> = m
        .tris
        .par_iter()
        .map(|t| {
            let [a, b, c] = t.map(|i| m.positions[i as usize]);
            (b - a).cross(c - a).normalize_or_zero()
        })
        .collect();
    let mut acc = vec![Vec3::ZERO; nv];
    for (t, &n) in m.tris.iter().zip(&face_n) {
        let p = t.map(|i| m.positions[i as usize]);
        for k in 0..3 {
            let ang = (p[(k + 1) % 3] - p[k]).angle_between(p[(k + 2) % 3] - p[k]);
            if ang.is_finite() {
                acc[t[k] as usize] += n * ang;
            }
        }
    }
    acc.par_iter_mut().for_each(|n| *n = n.normalize_or_zero());
    let mut min_cos = vec![1.0f64; nv];
    for (t, &n) in m.tris.iter().zip(&face_n) {
        if n == Vec3::ZERO {
            continue;
        }
        for &i in t {
            let c = acc[i as usize].dot(n);
            let s = &mut min_cos[i as usize];
            *s = s.min(c);
        }
    }
    acc.par_iter_mut()
        .zip(min_cos.par_iter())
        .for_each(|(n, &c)| *n /= c.clamp(1.0 / MAX_STRETCH, 1.0));
    acc
}

/// Directed edges `a → b` of `tris` whose reverse `b → a` is used less often than they are
/// (boundary half-edges), in face order. Degenerate edges (`a == b`) are ignored.
fn boundary_half_edges(tris: &[[u32; 3]]) -> Vec<(u32, u32)> {
    let key = |a: u32, b: u32| (u64::from(a) << 32) | u64::from(b);
    let mut all: Vec<u64> = tris
        .iter()
        .flat_map(|t| [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])])
        .filter(|(a, b)| a != b)
        .map(|(a, b)| key(a, b))
        .collect();
    all.par_sort_unstable();
    let count = |k: u64| all.partition_point(|&x| x <= k) - all.partition_point(|&x| x < k);
    // Fast path: every edge paired once (closed, manifold).
    let unpaired: Vec<bool> = tris
        .par_iter()
        .flat_map_iter(|t| [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])])
        .map(|(a, b)| a != b && all.binary_search(&key(b, a)).is_err())
        .collect();
    if !unpaired.contains(&true) && all.windows(2).all(|w| w[0] != w[1]) {
        return Vec::new();
    }
    // General case: an edge used `k` times with its reverse used `r < k` times contributes
    // `k - r` boundary half-edges (the first ones in face order).
    let mut used = std::collections::HashMap::new();
    let mut out = Vec::new();
    for t in tris {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            if a == b {
                continue;
            }
            let (k, r) = (count(key(a, b)), count(key(b, a)));
            if r >= k {
                continue;
            }
            let u = used.entry(key(a, b)).or_insert(0usize);
            if *u < k - r {
                *u += 1;
                out.push((a, b));
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boundary_of_closed_tetra_is_empty() {
        let t = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
        assert!(boundary_half_edges(&t).is_empty());
    }

    #[test]
    fn boundary_of_single_triangle() {
        assert_eq!(
            boundary_half_edges(&[[0, 1, 2]]),
            vec![(0, 1), (1, 2), (2, 0)]
        );
    }
}
