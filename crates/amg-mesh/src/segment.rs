//! Face-group segmentation (MESH-12): region growing by normal deviation and curvature, then
//! merging of small regions.
//!
//! 1. Face adjacency from the shared edges (an edge with exactly two faces; boundary and
//!    non-manifold edges join nothing).
//! 2. A per-face curvature estimate: the mean over the neighbours of `angle between normals /
//!    distance between centroids` (1/mm). It orders the seeds, flattest first, ties by face id.
//! 3. Region growing from the seeds in that order. A neighbour joins when the angle between its
//!    normal and the normal of the face it is reached from is within `angle_threshold` and, if
//!    `curvature_threshold` is set, its curvature is within that of the seed.
//! 4. Regions below `min_group_faces` are merged, smallest first, into the neighbour region they
//!    share the longest border with.
//!
//! The result does not depend on the thread count: parallel passes only compute per-face values
//! and sort with total orders. Groups are numbered by their smallest face.

use std::collections::{BTreeMap, BTreeSet};

use amg_core::Vec3;
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::error::{MeshError, MeshResult};
use crate::groups::{FaceGroupId, FaceGroupInfo, GroupOrigin};
use crate::{FaceId, IndexedMesh};

const NONE: u32 = u32::MAX;
const CHUNK: usize = 1 << 14;

/// Options of [`segment_face_groups`].
#[derive(Clone, Debug, PartialEq)]
pub struct SegmentOpts {
    /// Largest angle between the normals of two adjacent faces of one region, radians
    /// (in `(0, pi)`).
    pub angle_threshold: f64,
    /// Largest difference between the curvature (1/mm) of a face and of the seed of its region;
    /// `None`: curvature only orders the seeds.
    pub curvature_threshold: Option<f64>,
    /// Regions with fewer faces are merged into a neighbour (`0` or `1`: no merging).
    pub min_group_faces: usize,
}

impl Default for SegmentOpts {
    fn default() -> Self {
        SegmentOpts {
            angle_threshold: 30f64.to_radians(),
            curvature_threshold: None,
            min_group_faces: 16,
        }
    }
}

/// Segments `m` into face groups and stores them in `m.groups`: one new group per region
/// (origin [`GroupOrigin::Segmentation`], numbered by the region's smallest face), every face
/// assigned. Earlier groups stay in the table, now without faces. Returns the number of groups.
///
/// # Errors
/// `InvalidOption` for a threshold out of range, `TooLarge` when the group ids run out,
/// `Cancelled` when `ctx` is cancelled (the mesh is then unchanged).
pub fn segment_face_groups(
    m: &mut IndexedMesh,
    opts: &SegmentOpts,
    ctx: &JobCtx,
) -> MeshResult<usize> {
    let labels = segment_labels(m, opts, ctx)?;
    let count = labels.iter().max().map_or(0, |&l| l as usize + 1);
    if (m.groups.remaining_ids() as usize) < count {
        return Err(MeshError::TooLarge {
            count: m.groups.next_id() as usize + count,
        });
    }
    let ids: Vec<FaceGroupId> = (0..count)
        .map(|_| {
            m.groups
                .alloc(FaceGroupInfo::new(GroupOrigin::Segmentation))
        })
        .collect();
    for (f, &l) in labels.iter().enumerate() {
        m.groups.set(FaceId(f as u32), ids[l as usize]);
    }
    ctx.progress(1.0);
    Ok(count)
}

/// Region label per face (`0..count`, numbered by smallest face).
fn segment_labels(m: &IndexedMesh, opts: &SegmentOpts, ctx: &JobCtx) -> MeshResult<Vec<u32>> {
    if !(opts.angle_threshold.is_finite()
        && opts.angle_threshold > 0.0
        && opts.angle_threshold < std::f64::consts::PI)
    {
        return Err(MeshError::InvalidOption {
            name: "angle_threshold",
            reason: format!("must be in (0, pi), got {}", opts.angle_threshold),
        });
    }
    if let Some(c) = opts.curvature_threshold
        && !(c.is_finite() && c >= 0.0)
    {
        return Err(MeshError::InvalidOption {
            name: "curvature_threshold",
            reason: format!("must be finite and not negative, got {c}"),
        });
    }
    let n = m.tris.len();
    if n == 0 {
        return Ok(Vec::new());
    }
    if n >= NONE as usize / 3 {
        return Err(MeshError::TooLarge { count: n });
    }

    let (normals, centroids): (Vec<Vec3>, Vec<Vec3>) = m
        .tris
        .par_iter()
        .map(|t| {
            let [a, b, c] = t.map(|i| m.positions[i as usize]);
            ((b - a).cross(c - a).normalize_or_zero(), (a + b + c) / 3.0)
        })
        .unzip();
    ctx.checkpoint()?;
    ctx.progress(0.1);
    let nbr = adjacency(&m.tris);
    ctx.checkpoint()?;
    ctx.progress(0.3);

    let curvature: Vec<f64> = (0..n)
        .into_par_iter()
        .with_min_len(CHUNK)
        .map(|f| {
            if normals[f] == Vec3::ZERO {
                return f64::INFINITY;
            }
            let (mut sum, mut k) = (0.0, 0u32);
            for &g in &nbr[f] {
                let g = g as usize;
                if g == NONE as usize || normals[g] == Vec3::ZERO {
                    continue;
                }
                let d = centroids[f].distance(centroids[g]);
                if d > 0.0 {
                    sum += normals[f].dot(normals[g]).clamp(-1.0, 1.0).acos() / d;
                    k += 1;
                }
            }
            if k == 0 { 0.0 } else { sum / f64::from(k) }
        })
        .collect();
    let mut seeds: Vec<u32> = (0..n as u32).collect();
    seeds.par_sort_unstable_by(|&a, &b| {
        curvature[a as usize]
            .total_cmp(&curvature[b as usize])
            .then(a.cmp(&b))
    });
    ctx.checkpoint()?;
    ctx.progress(0.4);

    // Region growing.
    let cos_t = opts.angle_threshold.cos();
    let mut label = vec![NONE; n];
    let mut regions = 0u32;
    let mut stack: Vec<u32> = Vec::new();
    for (si, &seed) in seeds.iter().enumerate() {
        if si % CHUNK == 0 {
            ctx.checkpoint()?;
        }
        if label[seed as usize] != NONE {
            continue;
        }
        let r = regions;
        regions += 1;
        let seed_curv = curvature[seed as usize];
        label[seed as usize] = r;
        stack.push(seed);
        while let Some(f) = stack.pop() {
            let nf = normals[f as usize];
            for &g in &nbr[f as usize] {
                if g == NONE || label[g as usize] != NONE {
                    continue;
                }
                if nf.dot(normals[g as usize]) < cos_t {
                    continue;
                }
                if let Some(t) = opts.curvature_threshold
                    && (curvature[g as usize] - seed_curv).abs() > t
                {
                    continue;
                }
                label[g as usize] = r;
                stack.push(g);
            }
        }
    }
    ctx.checkpoint()?;
    ctx.progress(0.7);

    let label = merge_small(&nbr, label, regions as usize, opts.min_group_faces);
    ctx.checkpoint()?;
    Ok(canonical(&label))
}

/// Neighbour face across each edge (`NONE`: boundary, non-manifold edge, degenerate edge).
fn adjacency(tris: &[[u32; 3]]) -> Vec<[u32; 3]> {
    let mut edges: Vec<(u64, u32)> = tris
        .par_iter()
        .enumerate()
        .flat_map_iter(|(f, t)| {
            (0..3).filter_map(move |c| {
                let (a, b) = (t[c], t[(c + 1) % 3]);
                (a != b).then(|| {
                    let (lo, hi) = (a.min(b), a.max(b));
                    ((u64::from(lo) << 32) | u64::from(hi), (f * 3 + c) as u32)
                })
            })
        })
        .collect();
    edges.par_sort_unstable();
    let mut nbr = vec![[NONE; 3]; tris.len()];
    let mut i = 0;
    while i < edges.len() {
        let mut j = i + 1;
        while j < edges.len() && edges[j].0 == edges[i].0 {
            j += 1;
        }
        if j - i == 2 {
            let (fa, ca) = ((edges[i].1 / 3) as usize, (edges[i].1 % 3) as usize);
            let (fb, cb) = ((edges[i + 1].1 / 3) as usize, (edges[i + 1].1 % 3) as usize);
            if fa != fb {
                nbr[fa][ca] = fb as u32;
                nbr[fb][cb] = fa as u32;
            }
        }
        i = j;
    }
    nbr
}

/// Merges regions below `min_faces` into the neighbour with the longest shared border,
/// smallest first (ties by region id); regions without a neighbour stay.
fn merge_small(nbr: &[[u32; 3]], label: Vec<u32>, regions: usize, min_faces: usize) -> Vec<u32> {
    if min_faces <= 1 || regions <= 1 {
        return label;
    }
    let mut size = vec![0usize; regions];
    for &l in &label {
        size[l as usize] += 1;
    }
    if size.iter().all(|&s| s >= min_faces) {
        return label;
    }
    let mut adj: Vec<BTreeMap<u32, u32>> = vec![BTreeMap::new(); regions];
    for (f, ns) in nbr.iter().enumerate() {
        for &g in ns {
            if g != NONE && (g as usize) > f && label[g as usize] != label[f] {
                let (a, b) = (label[f], label[g as usize]);
                *adj[a as usize].entry(b).or_insert(0) += 1;
                *adj[b as usize].entry(a).or_insert(0) += 1;
            }
        }
    }
    let mut parent: Vec<u32> = (0..regions as u32).collect();
    let mut small: BTreeSet<(usize, u32)> = (0..regions)
        .filter(|&r| size[r] < min_faces)
        .map(|r| (size[r], r as u32))
        .collect();
    while let Some((s, r)) = small.pop_first() {
        let ru = r as usize;
        debug_assert_eq!(s, size[ru]);
        // Longest border, ties to the lower id.
        let Some((&q, _)) = adj[ru].iter().max_by_key(|&(&x, &c)| (c, u32::MAX - x)) else {
            continue;
        };
        let qu = q as usize;
        small.remove(&(size[qu], q));
        let moved = std::mem::take(&mut adj[ru]);
        adj[qu].remove(&r);
        for (x, c) in moved {
            if x == q {
                continue;
            }
            adj[x as usize].remove(&r);
            *adj[x as usize].entry(q).or_insert(0) += c;
            *adj[qu].entry(x).or_insert(0) += c;
        }
        size[qu] += size[ru];
        size[ru] = 0;
        parent[ru] = q;
        if size[qu] < min_faces {
            small.insert((size[qu], q));
        }
    }
    label
        .into_iter()
        .map(|l| {
            let mut r = l;
            while parent[r as usize] != r {
                r = parent[r as usize];
            }
            r
        })
        .collect()
}

/// Renumbers labels `0..count` in order of the smallest face.
fn canonical(label: &[u32]) -> Vec<u32> {
    let mut map: BTreeMap<u32, u32> = BTreeMap::new();
    label
        .iter()
        .map(|&l| {
            let next = map.len() as u32;
            *map.entry(l).or_insert(next)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use amg_core::Point3;

    use super::*;

    /// Open two-quad "roof": a flat strip folded by `fold` radians along x = 1.
    fn roof(fold: f64, n: u32) -> IndexedMesh {
        let mut pos = Vec::new();
        let mut tris = Vec::new();
        for side in 0..2u32 {
            let base = pos.len() as u32;
            for i in 0..=n {
                for j in 0..=n {
                    let (u, v) = (f64::from(i) / f64::from(n), f64::from(j) / f64::from(n));
                    pos.push(if side == 0 {
                        Point3::new(u, v, 0.0)
                    } else {
                        Point3::new(1.0 + u * fold.cos(), v, u * fold.sin())
                    });
                }
            }
            let id = |i: u32, j: u32| base + i * (n + 1) + j;
            for i in 0..n {
                for j in 0..n {
                    tris.push([id(i, j), id(i + 1, j), id(i + 1, j + 1)]);
                    tris.push([id(i, j), id(i + 1, j + 1), id(i, j + 1)]);
                }
            }
        }
        let mut m = IndexedMesh::new(pos, tris).unwrap();
        m.weld(crate::WeldOpts { tolerance: 1e-9 }).unwrap();
        m
    }

    #[test]
    fn fold_splits_and_flat_does_not() {
        let ctx = JobCtx::detached();
        let mut sharp = roof(60f64.to_radians(), 8);
        assert_eq!(
            segment_face_groups(&mut sharp, &SegmentOpts::default(), &ctx).unwrap(),
            2
        );
        let mut flat = roof(10f64.to_radians(), 8);
        assert_eq!(
            segment_face_groups(&mut flat, &SegmentOpts::default(), &ctx).unwrap(),
            1
        );
        assert!(sharp.groups.ids().any(|g| matches!(
            sharp.groups.info(g).unwrap().origin,
            GroupOrigin::Segmentation
        )));
    }

    #[test]
    fn small_regions_are_merged() {
        let ctx = JobCtx::detached();
        let mut m = roof(60f64.to_radians(), 8);
        let o = SegmentOpts {
            min_group_faces: 1000,
            ..SegmentOpts::default()
        };
        // Both halves are below the limit: the smaller (ties: lower id) merges into the other.
        assert_eq!(segment_face_groups(&mut m, &o, &ctx).unwrap(), 1);
    }

    #[test]
    fn invalid_options_and_empty_mesh() {
        let ctx = JobCtx::detached();
        let mut m = roof(1.0, 2);
        for bad in [0.0, -1.0, f64::NAN, 4.0] {
            let o = SegmentOpts {
                angle_threshold: bad,
                ..SegmentOpts::default()
            };
            assert!(matches!(
                segment_face_groups(&mut m, &o, &ctx),
                Err(MeshError::InvalidOption { .. })
            ));
        }
        let mut e = IndexedMesh::new(Vec::new(), Vec::new()).unwrap();
        assert_eq!(
            segment_face_groups(&mut e, &SegmentOpts::default(), &ctx).unwrap(),
            0
        );
    }

    #[test]
    fn cancelled_leaves_the_mesh_unchanged() {
        let token = amg_jobs::CancelToken::new();
        token.cancel();
        let ctx = JobCtx::with_token(token);
        let mut m = roof(1.0, 4);
        let before = m.content_hash();
        assert!(matches!(
            segment_face_groups(&mut m, &SegmentOpts::default(), &ctx),
            Err(MeshError::Cancelled)
        ));
        assert_eq!(m.content_hash(), before);
    }
}
