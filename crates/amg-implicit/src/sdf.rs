//! Signed distance fields backed by data: [`MeshSdf`] (IMP-03). Sampled grids (IMP-12) follow.
//!
//! Design note `docs/notes/implicit-modeling.md` §5.

use std::sync::Arc;

use amg_core::{Aabb3, Point3};
use amg_jobs::JobCtx;
use amg_mesh::iso::ScalarField;
use amg_mesh::{ClosestPoint, FaceGroupId, FaceId, IndexedMesh, MeshSpatial, TriFeature};
use rayon::prelude::*;

/// Accuracy parameter of the first, cheap winding-number estimate on closed meshes (about 4×
/// faster than β = 2 for points near the surface).
pub const FAST_BETA: f64 = 1.0;
/// The cheap estimate decides the sign when it is within this of an integer.
pub const FAST_MARGIN: f64 = 0.2;

/// Exact narrow-band signed distance to a triangle mesh.
///
/// - `|d|` is the exact distance to the closest point on the mesh (MESH-03 BVH query), clamped
///   to `band`.
/// - The sign is negative inside, where the fast generalized winding number (MESH-03)
///   exceeds 0.5 (see [`MeshSdf::is_inside`]). That is robust to small holes and
///   self-overlaps.
/// - Outside the band the field is `±band`, so on a closed mesh it stays 1-Lipschitz and
///   [`ScalarField::may_cross`] can reject boxes that are farther than the band.
///
/// Every query is a pure function of the mesh, the band and the point, so results are
/// bit-identical for any rayon thread count. CPU only; IMP-12 bakes it into a sampled grid
/// for the GPU.
#[derive(Debug)]
pub struct MeshSdf {
    spatial: Arc<MeshSpatial>,
    band: f64,
    /// The mesh's boundary is empty as a chain (every undirected edge is used equally often
    /// in both directions). Then the winding number is integer-valued off the surface and the
    /// sign can only change across the surface.
    closed: bool,
}

impl MeshSdf {
    /// A mesh SDF over `spatial` with narrow band `band` (mm). A negative or NaN band is
    /// treated as 0; `f64::INFINITY` gives the unclamped signed distance.
    ///
    /// Builds the winding-number tree now if `spatial` has none yet (with a detached,
    /// uncancellable context: call [`MeshSpatial::prepare_winding`] first to make that part
    /// cancellable) and checks whether the mesh is closed, both O(n). Building eagerly also
    /// keeps the lazy tree initialisation out of parallel query loops.
    pub fn new(spatial: Arc<MeshSpatial>, band: f64) -> Self {
        // `+ 0.0` turns a -0.0 into +0.0 so equal bands hash equally.
        let band = band.max(0.0) + 0.0;
        if spatial.prepare_winding(&JobCtx::detached()).is_err() {
            // A detached context is never cancelled; MeshSpatial also builds lazily as a
            // fallback, so nothing is lost if this ever happened.
        }
        let closed = boundary_is_empty(spatial.mesh());
        Self {
            spatial,
            band,
            closed,
        }
    }

    /// The narrow band, mm.
    pub fn band(&self) -> f64 {
        self.band
    }

    /// The spatial index the field queries.
    pub fn spatial(&self) -> &Arc<MeshSpatial> {
        &self.spatial
    }

    /// True if the mesh boundary vanishes (closed and consistently oriented as an edge chain,
    /// by vertex index). Only then does [`ScalarField::may_cross`] prune.
    pub fn is_closed(&self) -> bool {
        self.closed
    }

    /// Signed distance at `p`: negative inside (winding number > 0.5), clamped to `±band`
    /// outside the band. NaN for a NaN point.
    pub fn signed(&self, p: Point3) -> f64 {
        if p.is_nan() {
            return f64::NAN;
        }
        let d = self.unsigned(p);
        if self.is_inside(p) { -d } else { d }
    }

    /// Inside test: winding number > 0.5.
    ///
    /// On a closed mesh the winding number is an integer off the surface, so a cheap
    /// estimate (β = [`FAST_BETA`]) decides when it is within [`FAST_MARGIN`] of an integer;
    /// otherwise, and always on open meshes (fractional winding numbers), the MESH-03
    /// default β = 2 decides.
    pub fn is_inside(&self, p: Point3) -> bool {
        if self.closed {
            let w = self.spatial.winding_number_with_beta(p, FAST_BETA);
            let r = w.round();
            if (w - r).abs() <= FAST_MARGIN {
                return r > 0.5;
            }
        }
        self.spatial.is_inside(p)
    }

    /// Unsigned distance to the surface, clamped to `band`. NaN for a NaN point.
    pub fn unsigned(&self, p: Point3) -> f64 {
        if p.is_nan() {
            return f64::NAN;
        }
        self.spatial
            .closest_point(p, self.band)
            .map_or(self.band, |c| self.distance(p, &c).min(self.band))
    }

    /// Distance to the closest point `c` found by MESH-03. For an interior (`Face`) closest
    /// point it is recomputed as the distance to the face's plane, with the normal taken from
    /// the two shortest edges: MESH-03's Gram-determinant formula loses up to ~1e-9 mm on long
    /// thin faces (seen on corpus model thingi10k-99899), the plane distance does not. Edge
    /// and vertex distances are well conditioned and kept.
    fn distance(&self, p: Point3, c: &ClosestPoint) -> f64 {
        let d = c.dist_sq.sqrt();
        if c.feature != TriFeature::Face {
            return d;
        }
        let [a, b, cc] = self.spatial.mesh().triangle(c.face);
        let (la, lb, lc) = (
            (cc - b).length_squared(),
            (a - cc).length_squared(),
            (b - a).length_squared(),
        );
        // Corner opposite the longest edge, then its two edges in counter-clockwise order.
        let (o, x, y) = if la >= lb && la >= lc {
            (a, b, cc)
        } else if lb >= lc {
            (b, cc, a)
        } else {
            (cc, a, b)
        };
        let n = (x - o).cross(y - o);
        let nl = n.length();
        let plane = ((p - o).dot(n) / nl).abs();
        if plane.is_finite() { plane } else { d }
    }

    /// Face group of the closest triangle (lowest face id on ties), or `None` when the
    /// surface is farther than `band` (or `p` is NaN).
    pub fn closest_group(&self, p: Point3) -> Option<FaceGroupId> {
        let c = self.spatial.closest_point(p, self.band)?;
        Some(self.spatial.mesh().groups.group_of(FaceId(c.face.0)))
    }

    /// Hash of the mesh content ([`IndexedMesh::content_hash`]) and the band. Stable across
    /// runs and builds.
    pub fn content_hash(&self) -> u64 {
        let h = self.spatial.mesh().content_hash();
        mix(h ^ mix(self.band.to_bits() ^ 0x4d65_7368_5364_6631)) // "MeshSdf1"
    }
}

impl ScalarField for MeshSdf {
    fn value(&self, p: Point3) -> f64 {
        self.signed(p)
    }

    /// Parallel batch evaluation; each value equals [`MeshSdf::signed`]. Large batches are
    /// evaluated in Morton order (for cache locality in the BVH) and scattered back, which
    /// changes no value.
    fn values(&self, ps: &[Point3], out: &mut [f64]) {
        let n = ps.len().min(out.len());
        if n < MORTON_MIN {
            ps[..n]
                .par_iter()
                .zip(out[..n].par_iter_mut())
                .with_min_len(64)
                .for_each(|(p, o)| *o = self.signed(*p));
            return;
        }
        let order = morton_order(&ps[..n]);
        let vals: Vec<f64> = order
            .par_iter()
            .with_min_len(64)
            .map(|&i| self.signed(ps[i as usize]))
            .collect();
        for (&i, v) in order.iter().zip(vals) {
            out[i as usize] = v;
        }
    }

    /// On a closed mesh the box cannot cross when either
    /// - the surface is farther than `half_diagonal + |iso|` from the box centre (then every
    ///   point of the box has `|d| > |iso|`, all on one side of the surface), or
    /// - `|f(centre) − iso| > half_diagonal` (the field is 1-Lipschitz).
    ///
    /// Levels beyond `±band` are never reached.
    /// On a mesh that is not closed the sign can jump away from the surface (where the
    /// winding number passes 0.5), so every box with `|iso| < band` may cross.
    fn may_cross(&self, b: &Aabb3, iso: f64) -> bool {
        let a = iso.abs();
        if a > self.band {
            return false; // the field lies in [-band, band]
        }
        // `a == band` (reached outside the band) or NaN iso.
        if a.is_nan() || a >= self.band || !self.closed {
            return true; // iso == ±band is reached outside the band; NaN iso; open mesh
        }
        if !(b.min.is_finite() && b.max.is_finite()) || b.min.cmpgt(b.max).any() {
            return true;
        }
        let c = (b.min + b.max) * 0.5;
        let r = (b.max - b.min).length() * 0.5;
        // Slack covers rounding in the centre, the diagonal and the BVH distance.
        let reach = (r + a) * (1.0 + 1e-9) + 1e-9;
        if self.spatial.closest_point(c, reach).is_none() {
            return false;
        }
        let slack = r * (1.0 + 1e-9) + 1e-9;
        (self.signed(c) - iso).abs() <= slack
    }
}

/// True if every undirected edge `{a, b}` (`a != b`) is used as often as `a → b` as `b → a`.
fn boundary_is_empty(m: &IndexedMesh) -> bool {
    let key = |a: u32, b: u32| (u64::from(a) << 32) | u64::from(b);
    let mut fwd = Vec::with_capacity(m.tris.len() * 3 / 2 + 1);
    let mut back = Vec::with_capacity(m.tris.len() * 3 / 2 + 1);
    for t in &m.tris {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            if a < b {
                fwd.push(key(a, b));
            } else if b < a {
                back.push(key(b, a));
            }
        }
    }
    if fwd.len() != back.len() {
        return false;
    }
    fwd.par_sort_unstable();
    back.par_sort_unstable();
    fwd == back
}

/// Batches at least this large are evaluated in Morton order.
const MORTON_MIN: usize = 4096;

/// Indices of `ps` sorted by a 63-bit Morton code over the finite points' bounding box
/// (non-finite points get key 0); ties by index, so the order is deterministic.
fn morton_order(ps: &[Point3]) -> Vec<u32> {
    let mut lo = Point3::splat(f64::INFINITY);
    let mut hi = Point3::splat(f64::NEG_INFINITY);
    for p in ps.iter().filter(|p| p.is_finite()) {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    let ext = (hi - lo).max_element();
    let scale = if ext > 0.0 {
        ((1u64 << 21) - 1) as f64 / ext
    } else {
        0.0
    };
    let spread = |v: u64| {
        let mut x = v & 0x1f_ffff;
        x = (x | x << 32) & 0x1f_0000_0000_ffff;
        x = (x | x << 16) & 0x1f_0000_ff00_00ff;
        x = (x | x << 8) & 0x100f_00f0_0f00_f00f;
        x = (x | x << 4) & 0x10c3_0c30_c30c_30c3;
        (x | x << 2) & 0x1249_2492_4924_9249
    };
    let mut keys: Vec<(u64, u32)> = ps
        .par_iter()
        .enumerate()
        .map(|(i, p)| {
            let k = if p.is_finite() {
                let q = (*p - lo) * scale;
                // `as` saturates; q is in [0, 2^21 - 1] up to rounding.
                spread(q.x as u64) | spread(q.y as u64) << 1 | spread(q.z as u64) << 2
            } else {
                0
            };
            (k, i as u32)
        })
        .collect();
    keys.par_sort_unstable();
    keys.into_iter().map(|(_, i)| i).collect()
}

/// SplitMix64 finaliser.
fn mix(mut z: u64) -> u64 {
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tet(flip: bool) -> IndexedMesh {
        let p = vec![Point3::ZERO, Point3::X, Point3::Y, Point3::Z];
        let mut t = vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
        if flip {
            t[0].swap(1, 2);
        }
        IndexedMesh::new(p, t).expect("tet")
    }

    #[test]
    fn boundary_check() {
        assert!(boundary_is_empty(&tet(false)));
        assert!(!boundary_is_empty(&tet(true)));
        assert!(boundary_is_empty(&IndexedMesh::default()));
        let mut open = tet(false);
        open.tris.pop();
        assert!(!boundary_is_empty(&open));
        // Two copies of the same closed surface (an overlapping shell) are still closed.
        let mut two = tet(false);
        let t = two.tris.clone();
        two.tris.extend(t);
        assert!(boundary_is_empty(&two));
    }
}
