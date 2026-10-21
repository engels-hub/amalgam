//! Profile (region) detection (SK-06).
//!
//! [`detect_profiles`] turns the non-construction curves of a sketch into the bounded faces of
//! their planar arrangement:
//!
//! 1. **Intersect.** AABB sweep-and-prune, then analytic line/arc/circle intersection with
//!    tolerance `weld_tol` (end points touching a curve, tangent contacts and overlaps
//!    included; see [`geom::intersect`](crate::geom::intersect)).
//! 2. **Split and weld.** Curves are split at every hit; hit and end points closer than
//!    `weld_tol` are welded into one vertex (grid hash + union–find). Coincident pieces of
//!    overlapping curves are merged into one edge that carries all source ids.
//! 3. **Graph.** Bridges and dangling trees are removed (reported as `open_chains`). Around
//!    each vertex the outgoing half-edges are sorted by tangent angle, ties (tangent contacts)
//!    broken by signed curvature; `next` is the clockwise neighbour of the twin.
//! 4. **Faces.** Cycles with the face on their left are traced; per connected component the
//!    most negative cycle is its outer boundary, the others are bounded faces (CCW). Each
//!    component's outer boundary becomes a hole (CW) of the smallest enclosing face of another
//!    component. Every bounded face is one [`Profile`]; an island inside a hole is its own
//!    profile.
//!
//! Areas are exact for lines and arcs (polygon through the vertices plus circular-segment
//! terms). Output is deterministic: vertices are keyed by [`VertexKey`], loops start at their
//! smallest edge key, profiles are sorted by [`ProfileId`].

use amg_core::{Aabb2, Point2};
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;

use crate::geom::Curve2;
use crate::ids::EntityId;
use crate::model::{End, Sketch};

mod arrange;
mod hash;
#[cfg(test)]
mod tests;

/// Stable profile id: a 64-bit hash of the profile's sorted boundary edge keys (outer loop and
/// holes). Dimension edits that keep the arrangement topology keep it.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ProfileId(pub u64);

/// Persistent key of an arrangement vertex. A welded vertex uses the smallest key of the
/// points welded into it (end points before intersections before seams).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum VertexKey {
    /// End point of a line or arc.
    Endpoint(EntityId, End),
    /// The `ordinal`-th intersection of curves `a < b`, counted by parameter along `a`.
    Intersection {
        /// Lower entity id.
        a: EntityId,
        /// Higher entity id.
        b: EntityId,
        /// Index of the hit along `a`.
        ordinal: u16,
    },
    /// Parameter 0 of a closed curve that nothing splits.
    Seam(EntityId),
}

/// One boundary edge of a profile loop.
#[derive(Clone, Debug)]
pub struct ProfileEdge {
    /// Source entity (the smallest id when overlapping curves were merged). This is the
    /// `geom::Tag` SOL-02/03 put on the B-rep edge.
    pub source: EntityId,
    /// Other overlapping, coincident sources merged into this edge (sorted).
    pub also: SmallVec<[EntityId; 1]>,
    /// Trimmed piece, oriented along the loop. Lines are snapped to the welded vertices;
    /// circle and arc pieces are `Curve2::Arc` (an unsplit circle is an arc of `±2π`).
    pub curve: Curve2,
    /// Vertex at the start of `curve`.
    pub start: VertexKey,
    /// Vertex at the end of `curve`.
    pub end: VertexKey,
}

/// A closed loop of edges: outer loops are CCW, holes CW.
#[derive(Clone, Debug, Default)]
pub struct Loop {
    /// Edges in order; each edge's end is the next edge's start.
    pub edges: Vec<ProfileEdge>,
}

impl Loop {
    /// Signed area (positive for CCW), mm².
    pub fn signed_area(&self) -> f64 {
        arrange::loop_area_moment(self.edges.iter().map(|e| &e.curve)).0
    }

    /// True if `p` lies inside the loop (crossing parity; boundary points are unspecified).
    pub fn contains(&self, p: Point2) -> bool {
        arrange::loop_contains(self.edges.iter().map(|e| &e.curve), p)
    }

    /// Bounding box of the loop.
    pub fn bbox(&self) -> Aabb2 {
        self.edges
            .iter()
            .fold(Aabb2::EMPTY, |b, e| b.union(&e.curve.bbox()))
    }
}

/// Data for matching a profile across topology changes (SK-15, S01 persistent naming).
#[derive(Clone, Debug, PartialEq)]
pub struct ProfileFingerprint {
    /// Net area (outer minus holes), mm².
    pub area: f64,
    /// Centroid of the net region.
    pub centroid: Point2,
    /// Bounding box of the outer loop.
    pub bbox: Aabb2,
    /// Every source entity of every edge (sorted, unique).
    pub sources: Vec<EntityId>,
}

/// One bounded face of the arrangement.
#[derive(Clone, Debug)]
pub struct Profile {
    /// Stable id.
    pub id: ProfileId,
    /// Outer boundary, CCW.
    pub outer: Loop,
    /// Holes, CW, sorted by their smallest edge key.
    pub holes: Vec<Loop>,
    /// Matching data.
    pub fingerprint: ProfileFingerprint,
}

impl Profile {
    /// True if `p` is inside the outer loop and outside every hole.
    pub fn contains(&self, p: Point2) -> bool {
        self.outer.contains(p) && !self.holes.iter().any(|h| h.contains(p))
    }
}

/// Problems found while detecting profiles ("sketch doctor").
#[derive(Clone, Debug, PartialEq)]
pub enum ProfileIssue {
    /// A free end point lies within `(weld_tol, gap_warn_tol]` of another end point.
    NearGap {
        /// Vertex of the smaller key.
        a: VertexKey,
        /// Vertex of the larger key.
        b: VertexKey,
        /// Distance, mm.
        distance: f64,
    },
    /// Curves `a < b` share a stretch of positive length (merged into common edges).
    Overlap {
        /// Lower entity id.
        a: EntityId,
        /// Higher entity id.
        b: EntityId,
    },
    /// A curve intersects itself (splines, SK-08).
    SelfIntersection(EntityId),
    /// A curve no longer than `weld_tol` (skipped).
    ZeroLength(EntityId),
}

/// Options for [`detect_profiles`].
#[derive(Clone, Debug)]
pub struct ProfileOptions {
    /// Points closer than this are one vertex, mm. Default 1e-6.
    pub weld_tol: f64,
    /// Free end points closer than this (but farther than `weld_tol`) are reported as
    /// [`ProfileIssue::NearGap`], mm. Default 1e-2. Gaps are never healed.
    pub gap_warn_tol: f64,
    /// Include projected geometry. Default true.
    pub include_projected: bool,
}

impl Default for ProfileOptions {
    fn default() -> Self {
        ProfileOptions {
            weld_tol: 1e-6,
            gap_warn_tol: 1e-2,
            include_projected: true,
        }
    }
}

/// Profiles of one sketch.
#[derive(Clone, Debug, Default)]
pub struct ProfileSet {
    /// Bounded faces, sorted by id.
    pub profiles: Vec<Profile>,
    /// Curves (or pieces of them) that bound no region: dangling chains and bridges between
    /// loops. Each entry is one connected group's source ids (sorted, unique); groups are
    /// sorted.
    pub open_chains: Vec<Vec<EntityId>>,
    /// Issues in a deterministic order: zero-length curves, overlaps, near gaps.
    pub issues: Vec<ProfileIssue>,
}

impl ProfileSet {
    /// The profile with id `id`.
    pub fn get(&self, id: ProfileId) -> Option<&Profile> {
        self.profiles
            .binary_search_by_key(&id, |p| p.id)
            .ok()
            .map(|i| &self.profiles[i])
    }

    /// The profile containing `p` (profiles are disjoint faces, so this is the innermost
    /// region at `p`); `None` outside every profile.
    pub fn at_point(&self, p: Point2) -> Option<ProfileId> {
        self.profiles
            .iter()
            .filter(|pr| {
                let b = &pr.fingerprint.bbox;
                p.cmpge(b.min).all() && p.cmple(b.max).all()
            })
            .find(|pr| pr.contains(p))
            .map(|pr| pr.id)
    }
}

/// Detects the profiles (bounded regions) of `sketch`'s non-construction curves.
///
/// Never fails: degenerate input is skipped and reported in [`ProfileSet::issues`].
pub fn detect_profiles(sketch: &Sketch, opts: &ProfileOptions) -> ProfileSet {
    arrange::detect(sketch, opts).0
}
