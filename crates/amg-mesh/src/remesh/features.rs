//! Constrained-edge classification and snapping to the input's feature curves.

use amg_core::{Aabb3, Point3};
use amg_jobs::JobCtx;
use rayon::prelude::*;

use super::{EdgeMap, RemeshOpts, key};
use crate::{Bvh, BvhOpts, FaceId, HalfEdgeMesh, MeshResult};

/// Edge lies on a curve of the input; vertices on it are snapped back to that curve.
pub(super) const HARD: u8 = 1;
/// Boundary edge of a mesh whose boundary may move; constrained for sliding only.
pub(super) const SOFT: u8 = 2;

/// Classifies the edges of `he` (an unedited conversion of the input). Returns the constrained
/// edges and the segments of the HARD ones, both in ascending half-edge order.
pub(super) fn classify(he: &HalfEdgeMesh, opts: &RemeshOpts) -> (EdgeMap, Vec<[Point3; 2]>) {
    let cos_feature = opts.feature_angle.map(f64::cos);
    let normal = |f: FaceId| {
        let [a, b, c] = he.face_vertices(f).map(|v| he.position(v));
        (b - a).cross(c - a).normalize_or_zero()
    };
    let found: Vec<(u64, u8, [Point3; 2])> = (0..he.face_slots() as u32)
        .into_par_iter()
        .flat_map_iter(|f| (0..3).map(move |k| (FaceId(f), k)))
        .filter_map(|(f, k)| {
            let h = he.halfedge(f, k);
            let kind = match he.twin(h) {
                None => {
                    if opts.preserve_boundary {
                        HARD
                    } else {
                        SOFT
                    }
                }
                Some(t) if t.0 > h.0 => {
                    let g = he.face(t);
                    let mut hard = opts.preserve_group_borders
                        && he.groups().group_of(f) != he.groups().group_of(g);
                    if let Some(c) = cos_feature {
                        let (nf, ng) = (normal(f), normal(g));
                        hard |= nf != Point3::ZERO && ng != Point3::ZERO && nf.dot(ng) < c;
                    }
                    if hard { HARD } else { return None }
                }
                Some(_) => return None,
            };
            let (a, b) = (he.origin(h), he.target(h));
            Some((key(a.0, b.0), kind, [he.position(a), he.position(b)]))
        })
        .collect();
    let mut cons = EdgeMap::default();
    let mut segs = Vec::new();
    for (k, kind, seg) in found {
        cons.insert(k, kind);
        if kind == HARD {
            segs.push(seg);
        }
    }
    (cons, segs)
}

/// Closest point on a set of segments.
pub(super) struct Snapper {
    segs: Vec<[Point3; 2]>,
    bvh: Bvh,
}

fn closest_on_segment(p: Point3, a: Point3, b: Point3) -> Point3 {
    let d = b - a;
    let l2 = d.length_squared();
    if l2 <= 0.0 {
        return a;
    }
    a + d * ((p - a).dot(d) / l2).clamp(0.0, 1.0)
}

impl Snapper {
    pub(super) fn build(segs: &[[Point3; 2]], ctx: &JobCtx) -> MeshResult<Self> {
        let bounds: Vec<Aabb3> = segs.iter().map(|s| Aabb3::new(s[0], s[1])).collect();
        Ok(Snapper {
            segs: segs.to_vec(),
            bvh: Bvh::build(&bounds, &BvhOpts::default(), ctx)?,
        })
    }

    /// Closest point to `p` on the segments within `radius`; lowest segment index wins ties.
    pub(super) fn snap(&self, p: Point3, radius: f64) -> Option<Point3> {
        if self.segs.is_empty() {
            return None;
        }
        let r = Point3::splat(radius);
        let mut best: Option<(f64, u32, Point3)> = None;
        self.bvh.query_aabb(&Aabb3::new(p - r, p + r), &mut |prim| {
            let [a, b] = self.segs[prim as usize];
            let q = closest_on_segment(p, a, b);
            let d = (q - p).length_squared();
            if d <= radius * radius && best.is_none_or(|(bd, bp, _)| (d, prim) < (bd, bp)) {
                best = Some((d, prim, q));
            }
        });
        best.map(|(_, _, q)| q)
    }
}
