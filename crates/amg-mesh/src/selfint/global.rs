//! Global self-intersection removal (MESH-29): cut the surface along its self-intersection
//! curves, then drop the patches that lie inside another part of the solid.
//!
//! This is the answer for *overlapping solids* (several closed shells, or a shell folded into
//! itself), where [`super::remove_self_intersections_with`] has to delete whole sheets and
//! refill the hole, which is off by a fraction of an edge (a coarse model has edges of several
//! percent of its diagonal). Here nothing is invented: the output faces are pieces of the input
//! faces, so the output lies on the input surface up to the rounding of the new vertices and
//! the perturbation of step 0 (1e-9 of the diagonal).
//!
//! 0. **General position.** Every vertex is moved by a deterministic pseudo-random offset of at
//!    most [`GlobalSelfIntersectionOpts::perturbation`] of the bounding-box diagonal. CAD
//!    exports are full of exact contacts (a vertex on a face, coplanar faces, T-junctions)
//!    where "intersecting" has no clean answer; after the move they are either clear gaps or
//!    clear crossings, so every pair below is transversal. Most of the corpus models that
//!    MESH-21 cannot repair are of that kind.
//! 1. **Cut.** Detect the intersecting pairs (MESH-19). Two faces cross in a segment whose ends
//!    are crossings of an edge with the other face (found with Shewchuk's exact predicates,
//!    the point itself in f64). A crossing is computed once per `(edge, face)` and its vertex
//!    is shared by every face around the edge, so the cut mesh has no cracks. Faces that share
//!    one vertex cut in a segment that starts at it. Where three cut faces meet, the three
//!    segments cross in one shared vertex (three-plane intersection).
//! 2. **Re-triangulate.** Every face with a segment or a crossing on an edge gets a constrained
//!    Delaunay triangulation (`spade`) in its plane with the segments as constraints. A face
//!    whose constraints cannot be inserted keeps only its edge subdivision (a fan), which keeps
//!    the mesh closed but leaves its pairs.
//! 3. **Classify.** The cut mesh splits into patches at the edges of the cut curves. The
//!    *generalized winding number* of the **input** (Jacobson et al. 2013, evaluated with the
//!    fast hierarchical version of [`crate::MeshSpatial`]) is sampled a little off both sides
//!    of every face. A patch is an inner sheet when, by area, it is mostly inside the solid on
//!    both sides, or when the winding number does not jump by one across it (another sheet
//!    lies within the sampling offset: a coincident or inverted layer). Only patches that touch
//!    a cut curve are candidates.
//! 4. **Accept.** Three versions are tried in turn (drop all inner patches, drop only the
//!    large ones, drop nothing). The first with fewer pairs that opens no new loop and keeps a
//!    watertight input watertight wins; where no patch can be classified (open surfaces) the
//!    sheets stay and touch along the curve, which is intersection-free but not manifold. The
//!    few pairs that remain (failed triangulations) go through the local refill of MESH-21.
//!
//! The mesh-arrangement approach of Zhou et al. 2016 (exact rational arithmetic, cell
//! labelling by flood fill) removes the need for the perturbation and for the triangulation
//! fallbacks; this module keeps its two ideas (cut at exact crossings, label by winding number)
//! with f64 coordinates and a bounded perturbation instead.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::sync::Arc;

use amg_core::Point3;
use amg_jobs::JobCtx;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

use super::exact::{DEGENERATE, drop_axis, o2, o3};
use super::{
    RemoveSelfIntersectionsOpts, RemoveSelfIntersectionsReport, SelfIntersectOpts,
    remove_self_intersections_with, self_intersections,
};
use crate::analysis::{AnalysisReport, AnalyzeOpts, analyze};
use crate::holefill::HoleFillOpts;
use crate::indexed::IndexedMesh;
use crate::spatial::{BvhOpts, DEFAULT_WINDING_BETA, MeshSpatial};
use crate::{FaceId, MeshResult};

/// Options for [`remove_self_intersections_global`].
#[derive(Clone, Debug)]
pub struct GlobalSelfIntersectionOpts {
    /// How far off a face the winding number is sampled, as a fraction of the square root of
    /// the face area (the probe sits at the face centre, along its normal). Default 0.01.
    pub probe_offset: f64,
    /// A face is "inside" on one side when `|winding number|` exceeds this. Default 0.5.
    pub winding_threshold: f64,
    /// Accuracy parameter of the fast winding number. Default [`DEFAULT_WINDING_BETA`].
    pub winding_beta: f64,
    /// Every vertex is moved by a deterministic pseudo-random offset of up to this fraction of
    /// the bounding-box diagonal before the cut, which puts exact contacts (a vertex on a face,
    /// coplanar faces, T-junctions) into general position; the output keeps the moved
    /// positions, so the output lies within that distance of the input surface (plus the
    /// rounding of the new vertices). `0` disables it: exact contacts are then left to the
    /// clean-up. Default `1e-9`.
    pub perturbation: f64,
    /// Accept a cut that leaves sheets touching along the intersection curve (non-manifold
    /// edges) where no patch could be classified as an inner sheet (open surfaces). Such a
    /// result is free of intersecting pairs but not manifold; a watertight input still never
    /// becomes open. Default `true`.
    pub allow_seams: bool,
    /// Inner patches smaller than this fraction of the total area are only dropped in the
    /// first candidate (the one that drops everything); the second candidate leaves them.
    /// Default `1e-6`.
    pub min_drop_area: f64,
    /// The clean-up only runs while at most this many pairs remain; a mesh that still has more
    /// intersections than that is a mass overlap that the local refill does not repair (and
    /// can take minutes to try). Default 100.
    pub cleanup_max_pairs: usize,
    /// Local refill (MESH-21) applied to the pairs that survive the cut-and-drop (degenerate
    /// pairs, triple points). `None` skips it.
    pub cleanup: Option<RemoveSelfIntersectionsOpts>,
}

impl Default for GlobalSelfIntersectionOpts {
    fn default() -> Self {
        Self {
            probe_offset: 0.01,
            winding_threshold: 0.5,
            winding_beta: DEFAULT_WINDING_BETA,
            perturbation: 1e-9,
            allow_seams: true,
            min_drop_area: 1e-6,
            cleanup_max_pairs: 100,
            // The clean-up only has to mend what the cut left: small, local patches.
            cleanup: Some(RemoveSelfIntersectionsOpts {
                max_rounds: 4,
                max_removed_area: 0.05,
                fill: HoleFillOpts {
                    max_boundary_edges: 300,
                    ..RemoveSelfIntersectionsOpts::default().fill
                },
                ..RemoveSelfIntersectionsOpts::default()
            }),
        }
    }
}

/// Topology counts of a mesh, as far as the acceptance test looks at them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GlobalHealth {
    /// Open boundary loops.
    pub boundary_loops: usize,
    /// Edges with a single face.
    pub boundary_edges: usize,
    /// Edges with more than two faces.
    pub non_manifold_edges: usize,
    /// Non-manifold vertices.
    pub non_manifold_vertices: usize,
    /// Edges whose two faces disagree on orientation.
    pub orientation_conflicts: usize,
    /// Closed, manifold and oriented.
    pub watertight: bool,
}

impl GlobalHealth {
    fn of(r: &AnalysisReport) -> Self {
        Self {
            boundary_loops: r.boundary_loops.len(),
            boundary_edges: r.boundary_edges,
            non_manifold_edges: r.non_manifold_edges.len(),
            non_manifold_vertices: r.non_manifold_vertices.len(),
            orientation_conflicts: r.orientation_conflicts,
            watertight: r.is_watertight(),
        }
    }

    /// Nothing is worse than in `before`, and a watertight mesh stays watertight. With
    /// `seams` the non-manifold edges and vertices may grow: where two sheets cross and
    /// neither encloses the other (open surfaces) the cut leaves them touching along the
    /// intersection curve.
    fn no_worse_than(&self, before: &Self, seams: bool) -> bool {
        self.boundary_loops <= before.boundary_loops
            && self.orientation_conflicts <= before.orientation_conflicts
            && (seams
                || (self.non_manifold_edges <= before.non_manifold_edges
                    && self.non_manifold_vertices <= before.non_manifold_vertices))
            && (!before.watertight || self.watertight)
    }
}

/// What [`remove_self_intersections_global`] did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GlobalSelfIntersectionReport {
    /// Intersecting pairs in the input.
    pub pairs_before: usize,
    /// Pairs that were cut exactly (transversal, no shared vertex).
    pub pairs_cut: usize,
    /// Pairs of faces sharing an edge (coplanar fold-overs) or all vertices; not cut.
    pub pairs_shared_edge: usize,
    /// Pairs where a vertex or edge lies exactly on the other face (or they are coplanar); not cut.
    pub pairs_degenerate: usize,
    /// Pairs with an unexpected number of edge crossings; not cut.
    pub pairs_other: usize,
    /// Faces re-triangulated.
    pub faces_cut: usize,
    /// Of those, faces that fell back to an edge-only split (crossing segments or a failed
    /// triangulation); their pairs stay.
    pub faces_fallback: usize,
    /// Patches the cut surface was split into.
    pub patches: usize,
    /// Patches dropped as inner sheets.
    pub patches_dropped: usize,
    /// Faces of the dropped patches.
    pub faces_dropped: usize,
    /// Area of the dropped patches.
    pub area_dropped: f64,
    /// Pairs left right after the drop, before the optional clean-up.
    pub pairs_after_drop: usize,
    /// Topology of the input.
    pub health_before: GlobalHealth,
    /// Topology right after the drop (`None` when nothing was cut).
    pub health_after_drop: Option<GlobalHealth>,
    /// Which version of the cut was accepted ([`Candidate::None`] if none was).
    pub candidate: Candidate,
    /// Whether the cut-and-drop mesh was accepted (fewer pairs, nothing broken). When it is
    /// not, the input is returned unchanged.
    pub accepted: bool,
    /// Report of the optional local clean-up.
    pub cleanup: Option<RemoveSelfIntersectionsReport>,
    /// Intersecting pairs left in the result.
    pub pairs_after: usize,
}

/// Which version of the cut mesh was returned.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Candidate {
    /// No version of the cut was acceptable (or there was nothing to cut).
    #[default]
    None,
    /// The perturbation alone resolved every contact.
    Perturbed,
    /// All inner patches dropped.
    Drop,
    /// Only the large inner patches dropped.
    DropLarge,
    /// The surface is cut along the curves but nothing is dropped (sheets touch along them).
    CutOnly,
}

/// Result of intersecting an edge with a face.
#[derive(Clone, Copy, Debug)]
enum Cross {
    Miss,
    /// Touching exactly (vertex or edge on the other face's boundary, coplanar).
    Degenerate,
    /// The edge passes through the interior of the face at this new vertex.
    Hit(u32),
}

#[inline]
fn canon(a: u32, b: u32) -> (u32, u32) {
    if a < b { (a, b) } else { (b, a) }
}

/// Is `p` (in the plane of `t`) inside the closed triangle `t`? Exact in the projection.
fn in_tri(p: Point3, t: [Point3; 3], drop: u8) -> bool {
    let orient = o2(t[0], t[1], t[2], drop);
    let s1 = o2(t[0], t[1], p, drop) * orient;
    let s2 = o2(t[1], t[2], p, drop) * orient;
    let s3 = o2(t[2], t[0], p, drop) * orient;
    s1 >= 0 && s2 >= 0 && s3 >= 0
}

struct Cutter<'a> {
    m: &'a IndexedMesh,
    /// New vertices (ids start at `m.positions.len()`).
    new_pos: Vec<Point3>,
    cache: HashMap<(u32, u32, u32), Cross>,
    /// Crossings on each undirected edge: `(parameter from the smaller vertex, vertex id)`.
    edge_pts: BTreeMap<(u32, u32), Vec<(f64, u32)>>,
}

impl Cutter<'_> {
    fn p(&self, v: u32) -> Point3 {
        let n = self.m.positions.len() as u32;
        if v < n {
            self.m.positions[v as usize]
        } else {
            self.new_pos[(v - n) as usize]
        }
    }

    fn add_point(&mut self, x: Point3) -> u32 {
        let id = self.m.positions.len() as u32 + self.new_pos.len() as u32;
        self.new_pos.push(x);
        id
    }

    fn tri(&self, f: u32) -> [Point3; 3] {
        self.m.tris[f as usize].map(|i| self.m.positions[i as usize])
    }

    /// The edge `a < b` against face `g` (cached, so every face around the edge sees the same
    /// answer and the same vertex).
    fn cross(&mut self, a: u32, b: u32, g: u32) -> Cross {
        if let Some(&c) = self.cache.get(&(a, b, g)) {
            return c;
        }
        let c = self.compute(a, b, g);
        self.cache.insert((a, b, g), c);
        c
    }

    fn compute(&mut self, a: u32, b: u32, g: u32) -> Cross {
        let t = self.tri(g);
        let drop = drop_axis(t[0], t[1], t[2]);
        if drop == DEGENERATE {
            return Cross::Miss;
        }
        let (pa, pb) = (self.m.positions[a as usize], self.m.positions[b as usize]);
        let (sa, sb) = (o3(t[0], t[1], t[2], pa), o3(t[0], t[1], t[2], pb));
        if sa == 0 || sb == 0 {
            if sa == sb {
                return Cross::Degenerate; // the edge lies in the plane of g
            }
            let on = if sa == 0 { pa } else { pb };
            return if in_tri(on, t, drop) {
                Cross::Degenerate
            } else {
                Cross::Miss
            };
        }
        if sa == sb {
            return Cross::Miss;
        }
        let s = [
            o3(pa, pb, t[0], t[1]),
            o3(pa, pb, t[1], t[2]),
            o3(pa, pb, t[2], t[0]),
        ];
        let (pos, neg) = (s.iter().any(|&x| x > 0), s.iter().any(|&x| x < 0));
        if pos && neg {
            return Cross::Miss;
        }
        if s.contains(&0) {
            return Cross::Degenerate;
        }
        let n = (t[1] - t[0]).cross(t[2] - t[0]);
        let (da, db) = (n.dot(pa - t[0]), n.dot(pb - t[0]));
        let u = da / (da - db);
        if !(u > 0.0 && u < 1.0) {
            return Cross::Degenerate;
        }
        let x = pa + (pb - pa) * u;
        let id = self.add_point(x);
        self.edge_pts.entry((a, b)).or_default().push((u, id));
        Cross::Hit(id)
    }
}

/// One cut face: the triangles replacing it, and whether the cut fell back to a fan.
struct Retri {
    tris: Vec<[u32; 3]>,
    /// Edges of the triangulation that are cut curves (not face edges).
    cuts: Vec<(u32, u32)>,
    fallback: bool,
}

/// Projects `p` by dropping axis `drop` (same convention as the exact predicates).
fn proj(p: Point3, drop: u8) -> (f64, f64) {
    match drop {
        0 => (p.y, p.z),
        1 => (p.z, p.x),
        _ => (p.x, p.y),
    }
}

/// Constrained Delaunay triangulation of the face polygon `chain` (the corners with the
/// crossings on its edges, in face order) with the points `interior` and the constraint
/// segments `segs`. `None` when the points degenerate in 2D or the segments cross.
fn cdt_face(
    chain: &[u32],
    interior: &[u32],
    segs: &[[u32; 2]],
    pos: &dyn Fn(u32) -> Point3,
    drop: u8,
    flip: bool,
) -> Option<Retri> {
    let ids: Vec<u32> = chain.iter().chain(interior).copied().collect();
    let pts: Vec<(f64, f64)> = ids.iter().map(|&v| proj(pos(v), drop)).collect();
    let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f64>>::new();
    let mut handles = Vec::with_capacity(ids.len());
    for &(x, y) in &pts {
        handles.push(cdt.insert(Point2::new(x, y)).ok()?);
    }
    let mut seen: Vec<usize> = handles.iter().map(|h| h.index()).collect();
    seen.sort_unstable();
    seen.dedup();
    if seen.len() != ids.len() {
        {
            return None;
        }
    }
    let n = chain.len();
    for i in 0..n {
        let (a, b) = (handles[i], handles[(i + 1) % n]);
        if !cdt.can_add_constraint(a, b) {
            {
                return None;
            }
        }
        cdt.add_constraint(a, b);
    }
    let local_of = |v: u32| ids.iter().position(|&x| x == v);
    for s in segs {
        let (ia, ib) = (local_of(s[0])?, local_of(s[1])?);
        if ia == ib {
            {
                return None;
            }
        }
        let (a, b) = (handles[ia], handles[ib]);
        if cdt.exists_constraint(a, b) {
            continue;
        }
        if !cdt.can_add_constraint(a, b) {
            {
                return None;
            }
        }
        cdt.add_constraint(a, b);
    }
    let mut local = vec![u32::MAX; cdt.num_vertices()];
    for (i, h) in handles.iter().enumerate() {
        local[h.index()] = i as u32;
    }
    // Faces reachable from the outer face without crossing a constraint lie outside the
    // polygon (slivers between the polygon and the hull when the edge points are a rounding
    // error off the straight edge).
    let mut outside = vec![false; cdt.num_all_faces()];
    let mut stack = Vec::new();
    for e in cdt.convex_hull() {
        if cdt.is_constraint_edge(e.fix().as_undirected()) {
            continue;
        }
        for side in [e.face(), e.rev().face()] {
            if let Some(inner) = side.as_inner() {
                let k = inner.fix().index();
                if !outside[k] {
                    outside[k] = true;
                    stack.push(inner.fix());
                }
            }
        }
    }
    while let Some(fh) = stack.pop() {
        for e in cdt.face(fh).adjacent_edges() {
            if cdt.is_constraint_edge(e.fix().as_undirected()) {
                continue;
            }
            if let Some(nb) = e.rev().face().as_inner() {
                let k = nb.fix().index();
                if !outside[k] {
                    outside[k] = true;
                    stack.push(nb.fix());
                }
            }
        }
    }
    let mut tris = Vec::new();
    for f in cdt.inner_faces() {
        if outside[f.fix().index()] {
            continue;
        }
        let vs = f.vertices();
        let t = [
            local[vs[0].fix().index()],
            local[vs[1].fix().index()],
            local[vs[2].fix().index()],
        ];
        if t.contains(&u32::MAX) {
            {
                return None;
            } // a Steiner vertex from a split constraint
        }
        let mut t = t.map(|i| ids[i as usize]);
        // Spade returns counter-clockwise faces in the projection.
        if flip {
            t.swap(1, 2);
        }
        tris.push(t);
    }
    if tris.len() != n - 2 + 2 * interior.len() {
        return None;
    }
    // The constrained edges that are not part of the face boundary are the cut curves.
    let on_chain: BTreeSet<(u32, u32)> = (0..n)
        .map(|i| canon(chain[i], chain[(i + 1) % n]))
        .collect();
    let mut cuts = Vec::new();
    for e in cdt.undirected_edges() {
        if !cdt.is_constraint_edge(e.fix()) {
            continue;
        }
        let [a, b] = e.vertices();
        let (la, lb) = (local[a.fix().index()], local[b.fix().index()]);
        if la == u32::MAX || lb == u32::MAX {
            continue;
        }
        let key = canon(ids[la as usize], ids[lb as usize]);
        if !on_chain.contains(&key) {
            cuts.push(key);
        }
    }
    Some(Retri {
        tris,
        cuts,
        fallback: false,
    })
}

fn fan(chain: &[u32]) -> Vec<[u32; 3]> {
    (1..chain.len() - 1)
        .map(|i| [chain[0], chain[i], chain[i + 1]])
        .collect()
}

/// Union-find with path halving.
struct Dsu(Vec<u32>);

impl Dsu {
    fn find(&mut self, mut x: u32) -> u32 {
        while self.0[x as usize] != x {
            let p = self.0[self.0[x as usize] as usize];
            self.0[x as usize] = p;
            x = p;
        }
        x
    }

    fn union(&mut self, a: u32, b: u32) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.0[a.max(b) as usize] = a.min(b);
        }
    }
}

fn pairs_of(m: &IndexedMesh, ctx: &JobCtx) -> MeshResult<(MeshSpatial, Vec<[FaceId; 2]>)> {
    let s = MeshSpatial::build(Arc::new(m.clone()), &BvhOpts::default(), ctx)?;
    let p = self_intersections(&s, &SelfIntersectOpts::default(), ctx)?;
    Ok((s, p))
}

/// splitmix64.
fn mix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// `m` with every vertex moved by a deterministic offset of at most `amp` per axis.
fn perturbed(m: &IndexedMesh, amp: f64) -> IndexedMesh {
    let unit = |h: u64| (h >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0;
    let mut out = m.clone();
    for (i, p) in out.positions.iter_mut().enumerate() {
        let h = mix(i as u64);
        let (h2, h3) = (mix(h), mix(h ^ 0xA5A5));
        *p += Point3::new(unit(h), unit(h2), unit(h3)) * amp;
    }
    out
}

/// Where three cut faces `f`, `g`, `h` meet in one point, the polylines of their three pairwise
/// intersections cross in that point. It becomes one vertex shared by all three faces and
/// splits those polylines, so no face is asked to cross two of its own constraints.
fn split_at_triple_points(
    cutter: &mut Cutter<'_>,
    pair_segs: &mut BTreeMap<(u32, u32), Vec<u32>>,
    face_pairs: &BTreeMap<u32, Vec<(u32, u32)>>,
    interior: &mut BTreeMap<u32, BTreeSet<u32>>,
) {
    let mut done: BTreeSet<[u32; 3]> = BTreeSet::new();
    for (&f, keys) in face_pairs {
        let tf = cutter.tri(f);
        let drop = drop_axis(tf[0], tf[1], tf[2]);
        if drop == DEGENERATE {
            continue;
        }
        let partner = |k: &(u32, u32)| if k.0 == f { k.1 } else { k.0 };
        for i in 0..keys.len() {
            for j in i + 1..keys.len() {
                let (g, h) = (partner(&keys[i]), partner(&keys[j]));
                let mut tri_key = [f, g, h];
                tri_key.sort_unstable();
                if g == h || done.contains(&tri_key) {
                    continue;
                }
                let gh = (g.min(h), g.max(h));
                if !pair_segs.contains_key(&gh) {
                    continue;
                }
                let (p, q) = ends(cutter, &pair_segs[&keys[i]]);
                let (a, b) = ends(cutter, &pair_segs[&keys[j]]);
                let cross = o2(p, q, a, drop) * o2(p, q, b, drop) < 0
                    && o2(a, b, p, drop) * o2(a, b, q, drop) < 0;
                if !cross {
                    continue;
                }
                let (tg, th) = (cutter.tri(g), cutter.tri(h));
                let Some(x) = three_planes([tf, tg, th]) else {
                    continue;
                };
                // Parameter of `x` along each of the three polylines; all must be interior.
                let param = |poly: &[u32]| {
                    let (a, b) = ends(cutter, poly);
                    let ab = b - a;
                    (x - a).dot(ab) / ab.length_squared()
                };
                let polys = [keys[i], keys[j], gh];
                if !polys
                    .iter()
                    .all(|k| matches!(param(&pair_segs[k]), t if t > 0.0 && t < 1.0))
                {
                    continue;
                }
                done.insert(tri_key);
                let id = cutter.add_point(x);
                for k in polys {
                    let poly = &pair_segs[&k];
                    let (a, b) = ends(cutter, poly);
                    let ab = b - a;
                    let tx = (x - a).dot(ab);
                    let at = poly
                        .iter()
                        .position(|&v| (cutter.p(v) - a).dot(ab) > tx)
                        .unwrap_or(poly.len() - 1);
                    if let Some(poly) = pair_segs.get_mut(&k) {
                        poly.insert(at, id);
                    }
                }
                for face in tri_key {
                    interior.entry(face).or_default().insert(id);
                }
            }
        }
    }
}

fn ends(c: &Cutter<'_>, poly: &[u32]) -> (Point3, Point3) {
    (c.p(poly[0]), c.p(poly[poly.len() - 1]))
}

/// The point where the planes of three triangles meet, or `None` for (nearly) parallel planes.
fn three_planes(t: [[Point3; 3]; 3]) -> Option<Point3> {
    let n = t.map(|v| (v[1] - v[0]).cross(v[2] - v[0]));
    let d = [n[0].dot(t[0][0]), n[1].dot(t[1][0]), n[2].dot(t[2][0])];
    let det = n[0].dot(n[1].cross(n[2]));
    if det.abs() <= 1e-12 * n[0].length() * n[1].length() * n[2].length() {
        return None;
    }
    Some((n[1].cross(n[2]) * d[0] + n[2].cross(n[0]) * d[1] + n[0].cross(n[1]) * d[2]) / det)
}

/// The cut-and-drop stage. `None` when there is nothing to cut.
fn cut_and_drop(
    m: &IndexedMesh,
    spatial: &MeshSpatial,
    pairs: &[[FaceId; 2]],
    opts: &GlobalSelfIntersectionOpts,
    rep: &mut GlobalSelfIntersectionReport,
    ctx: &JobCtx,
) -> MeshResult<Option<Vec<(Candidate, IndexedMesh)>>> {
    let mut cutter = Cutter {
        m,
        new_pos: Vec::new(),
        cache: HashMap::new(),
        edge_pts: BTreeMap::new(),
    };
    // Polyline of the intersection of each cut pair of faces (smaller id first), and the
    // pairs each face is part of.
    let mut pair_segs: BTreeMap<(u32, u32), Vec<u32>> = BTreeMap::new();
    let mut face_pairs: BTreeMap<u32, Vec<(u32, u32)>> = BTreeMap::new();
    let mut interior: BTreeMap<u32, BTreeSet<u32>> = BTreeMap::new();
    for pair in pairs {
        let (f, g) = (pair[0].0, pair[1].0);
        let (tf, tg) = (m.tris[f as usize], m.tris[g as usize]);
        let mut shared = [0u32; 3];
        let mut ns = 0;
        for &x in &tf {
            if tg.contains(&x) {
                shared[ns] = x;
                ns += 1;
            }
        }
        if ns >= 2 {
            // A fold-over (shared edge) or a duplicate face: coplanar, not cut.
            rep.pairs_shared_edge += 1;
            continue;
        }
        let mut hits: Vec<(u32, u32)> = Vec::new(); // (vertex, face whose interior it lies in)
        let mut degenerate = false;
        for (src, dst) in [(tf, g), (tg, f)] {
            for k in 0..3 {
                let (a0, b0) = (src[k], src[(k + 1) % 3]);
                if ns == 1 && (a0 == shared[0] || b0 == shared[0]) {
                    continue; // touches the other face at the shared vertex
                }
                let (a, b) = canon(a0, b0);
                match cutter.cross(a, b, dst) {
                    Cross::Hit(v) => hits.push((v, dst)),
                    Cross::Degenerate => degenerate = true,
                    Cross::Miss => {}
                }
            }
        }
        // Two crossings, or one when the faces share a vertex (the segment starts there).
        if degenerate {
            rep.pairs_degenerate += 1;
            continue;
        }
        if hits.len() != 2 - ns {
            rep.pairs_other += 1;
            continue;
        }
        let seg = if ns == 1 {
            [shared[0], hits[0].0]
        } else {
            [hits[0].0, hits[1].0]
        };
        rep.pairs_cut += 1;
        let key = (f.min(g), f.max(g));
        pair_segs.insert(key, seg.to_vec());
        face_pairs.entry(f).or_default().push(key);
        face_pairs.entry(g).or_default().push(key);
        for (v, face) in hits {
            interior.entry(face).or_default().insert(v);
        }
    }
    ctx.checkpoint()?;
    if cutter.edge_pts.is_empty() && pair_segs.is_empty() {
        return Ok(None);
    }
    split_at_triple_points(&mut cutter, &mut pair_segs, &face_pairs, &mut interior);
    for list in cutter.edge_pts.values_mut() {
        list.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    }

    // Faces to re-triangulate: around every edge with a crossing, and the ones with segments.
    let mut cut: BTreeSet<u32> = face_pairs.keys().copied().collect();
    for (f, t) in m.tris.iter().enumerate() {
        if (0..3).any(|k| cutter.edge_pts.contains_key(&canon(t[k], t[(k + 1) % 3]))) {
            cut.insert(f as u32);
        }
    }
    rep.faces_cut = cut.len();

    let mut replaced: BTreeMap<u32, Retri> = BTreeMap::new();
    for &f in &cut {
        let t = m.tris[f as usize];
        let mut chain = Vec::new();
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            chain.push(a);
            if let Some(list) = cutter.edge_pts.get(&canon(a, b)) {
                if a < b {
                    chain.extend(list.iter().map(|x| x.1));
                } else {
                    chain.extend(list.iter().rev().map(|x| x.1));
                }
            }
        }
        let inner: Vec<u32> = interior
            .get(&f)
            .map(|s| s.iter().copied().collect())
            .unwrap_or_default();
        let fs: Vec<[u32; 2]> = face_pairs
            .get(&f)
            .into_iter()
            .flatten()
            .flat_map(|k| pair_segs[k].windows(2).map(|w| [w[0], w[1]]))
            .collect();
        let tp = cutter.tri(f);
        let drop = drop_axis(tp[0], tp[1], tp[2]);
        let nrm = (tp[1] - tp[0]).cross(tp[2] - tp[0]);
        let flip = nrm[drop.min(2) as usize] < 0.0;
        let pos = |v: u32| cutter.p(v);
        let tris = if drop == DEGENERATE {
            None
        } else {
            cdt_face(&chain, &inner, &fs, &pos, drop, flip)
        };
        let r = tris.unwrap_or_else(|| Retri {
            tris: fan(&chain),
            cuts: Vec::new(),
            fallback: true,
        });
        if r.fallback {
            rep.faces_fallback += 1;
        }
        replaced.insert(f, r);
    }
    ctx.checkpoint()?;

    // Edges that separate the two sides of a cut curve in one sheet, whatever the other sheet
    // does there.
    let barrier: BTreeSet<u64> = replaced
        .values()
        .flat_map(|r| r.cuts.iter())
        .map(|&(a, b)| (u64::from(a) << 32) | u64::from(b))
        .collect();

    // The cut mesh.
    let mut tris: Vec<[u32; 3]> = Vec::with_capacity(m.tris.len() + replaced.len() * 2);
    let mut parent: Vec<u32> = Vec::with_capacity(tris.capacity());
    for (f, t) in m.tris.iter().enumerate() {
        match replaced.get(&(f as u32)) {
            Some(r) => {
                for &x in &r.tris {
                    tris.push(x);
                    parent.push(f as u32);
                }
            }
            None => {
                tris.push(*t);
                parent.push(f as u32);
            }
        }
    }
    let mut positions = m.positions.clone();
    positions.extend_from_slice(&cutter.new_pos);
    let pos_of = |v: u32| positions[v as usize];

    // Patches: faces connected through edges shared by exactly two faces.
    let mut edges: Vec<(u64, u32)> = Vec::with_capacity(tris.len() * 3);
    for (f, t) in tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = canon(t[k], t[(k + 1) % 3]);
            edges.push(((u64::from(a) << 32) | u64::from(b), f as u32));
        }
    }
    edges.sort_unstable();
    let mut dsu = Dsu((0..tris.len() as u32).collect());
    let mut seam = vec![false; tris.len()];
    // Faces around every edge with more than two faces (the cut curves).
    let mut seam_groups: Vec<Vec<u32>> = Vec::new();
    let mut i = 0;
    while i < edges.len() {
        let mut j = i;
        while j < edges.len() && edges[j].0 == edges[i].0 {
            j += 1;
        }
        let cut_curve = barrier.contains(&edges[i].0);
        match j - i {
            2 if !cut_curve => dsu.union(edges[i].1, edges[i + 1].1),
            2.. => {
                for e in &edges[i..j] {
                    seam[e.1 as usize] = true;
                }
                seam_groups.push(edges[i..j].iter().map(|e| e.1).collect());
            }
            _ => {}
        }
        i = j;
    }
    drop(edges);
    ctx.checkpoint()?;

    // Winding numbers of the input a little off both sides of every face.
    let mut probes: Vec<Point3> = Vec::with_capacity(tris.len() * 2);
    let mut area = vec![0.0f64; tris.len()];
    for (f, t) in tris.iter().enumerate() {
        let [a, b, c] = t.map(pos_of);
        let pf = m.triangle(FaceId(parent[f]));
        let n = (pf[1] - pf[0]).cross(pf[2] - pf[0]).normalize_or_zero();
        let a2 = (b - a).cross(c - a).length() * 0.5;
        area[f] = a2;
        let d = n * (opts.probe_offset * a2.sqrt());
        let ctr = (a + b + c) / 3.0;
        probes.push(ctr + d);
        probes.push(ctr - d);
    }
    let w = spatial.winding_numbers(&probes, opts.winding_beta);
    ctx.checkpoint()?;
    let thr = opts.winding_threshold;
    let sense = if spatial.mesh().signed_volume() < 0.0 {
        -1.0
    } else {
        1.0
    };
    let mut patch_area: HashMap<u32, f64> = HashMap::new();
    let mut patch_buried: HashMap<u32, f64> = HashMap::new();
    let mut patch_seam: BTreeSet<u32> = BTreeSet::new();
    for f in 0..tris.len() {
        let root = dsu.find(f as u32);
        *patch_area.entry(root).or_default() += area[f];
        // Inner: inside the solid on both sides, or a sheet the winding number does not see as
        // a boundary (another sheet within the probe offset, a coincident or inverted layer):
        // across one face the winding number changes by exactly one in the direction of the
        // surface orientation.
        let (wp, wm) = (w[2 * f], w[2 * f + 1]);
        let jump = (wm - wp) * sense;
        if (wp.abs() > thr && wm.abs() > thr) || (jump - 1.0).abs() > 0.5 {
            *patch_buried.entry(root).or_default() += area[f];
        }
        if seam[f] {
            patch_seam.insert(root);
        }
    }
    rep.patches = patch_area.len();
    let total_area: f64 = patch_area.values().sum();
    // Patches the winding number calls inner. Candidate 1 drops all of them, candidate 2 only
    // the large ones (the tiny ones are numerical slivers around the cut that change nothing
    // geometrically but can perturb the topology), candidate 3 drops none.
    let inner_roots = |min_area: f64| -> BTreeSet<u32> {
        let mut set = BTreeSet::new();
        for (&root, &a) in &patch_area {
            let b = patch_buried.get(&root).copied().unwrap_or(0.0);
            if patch_seam.contains(&root) && b > 0.5 * a && a >= min_area {
                set.insert(root);
            }
        }
        set
    };
    let mut out = Vec::new();
    let all = inner_roots(0.0);
    let large = inner_roots(opts.min_drop_area * total_area);
    rep.patches_dropped = all.len();
    for (kind, mut dropped) in [
        (Candidate::Drop, all),
        (Candidate::DropLarge, large),
        (Candidate::CutOnly, BTreeSet::new()),
    ] {
        // Dropping must not leave an edge with a single face (a new hole): where it would,
        // the largest dropped patch around that edge stays.
        loop {
            let mut changed = false;
            for g in &seam_groups {
                let roots: Vec<u32> = g.iter().map(|&f| dsu.find(f)).collect();
                let alive = roots.iter().filter(|r| !dropped.contains(r)).count();
                if alive == 1 {
                    let back = roots
                        .iter()
                        .filter(|r| dropped.contains(r))
                        .max_by(|a, b| patch_area[*a].total_cmp(&patch_area[*b]).then((*b).cmp(*a)))
                        .copied();
                    if let Some(r) = back {
                        dropped.remove(&r);
                        changed = true;
                    }
                }
            }
            if !changed {
                break;
            }
        }
        let mut keep_tris: Vec<[u32; 3]> = Vec::with_capacity(tris.len());
        let mut faces_dropped = 0;
        for (f, t) in tris.iter().enumerate() {
            if dropped.contains(&dsu.find(f as u32)) {
                faces_dropped += 1;
            } else {
                keep_tris.push(*t);
            }
        }
        if kind == Candidate::Drop {
            rep.patches_dropped = dropped.len();
            rep.faces_dropped = faces_dropped;
            rep.area_dropped = dropped.iter().map(|r| patch_area[r]).sum();
        }
        // Compact: only the vertices that are still used.
        let mut map = vec![u32::MAX; positions.len()];
        let mut out_pos = Vec::new();
        for t in &mut keep_tris {
            for v in t.iter_mut() {
                if map[*v as usize] == u32::MAX {
                    map[*v as usize] = out_pos.len() as u32;
                    out_pos.push(positions[*v as usize]);
                }
                *v = map[*v as usize];
            }
        }
        out.push((kind, IndexedMesh::new(out_pos, keep_tris)?));
    }
    Ok(Some(out))
}

/// Removes self-intersections of overlapping solids by cutting the surface along the
/// intersection curves and dropping the inner sheets (see the module docs).
///
/// Faces share vertices by index, so weld the mesh and orient it (outward) first. The result
/// opens no new loop, and a watertight input stays watertight; when the cut-and-drop would
/// break that, or does not reduce the intersections, the input is returned unchanged
/// ([`GlobalSelfIntersectionReport::accepted`] is false; [`remove_self_intersections_auto`]
/// then keeps the local result). Vertices move by up to
/// [`GlobalSelfIntersectionOpts::perturbation`]; attributes and face groups are not carried
/// over (the face set changes completely). An input without intersections is returned
/// unchanged.
///
/// # Errors
/// [`crate::MeshError::Cancelled`], or the errors of [`IndexedMesh::validate`].
pub fn remove_self_intersections_global(
    m: &IndexedMesh,
    opts: &GlobalSelfIntersectionOpts,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, GlobalSelfIntersectionReport)> {
    m.validate()?;
    let (spatial0, pairs0) = pairs_of(m, ctx)?;
    let mut rep = GlobalSelfIntersectionReport {
        pairs_before: pairs0.len(),
        ..Default::default()
    };
    if pairs0.is_empty() {
        rep.accepted = true;
        return Ok((m.clone(), rep));
    }
    rep.health_before = GlobalHealth::of(&analyze(m, &AnalyzeOpts::default(), ctx)?);
    let mut cur = m.clone();
    let mut cur_pairs = pairs0.len();
    let work = if opts.perturbation > 0.0 {
        let b = m.aabb();
        perturbed(m, opts.perturbation * (b.max - b.min).length())
    } else {
        m.clone()
    };
    let (spatial, pairs) = if opts.perturbation > 0.0 {
        pairs_of(&work, ctx)?
    } else {
        (spatial0, pairs0)
    };
    let cands: Vec<(Candidate, IndexedMesh)> = if pairs.is_empty() {
        // The perturbation alone resolved every contact.
        vec![(Candidate::Perturbed, work.clone())]
    } else {
        cut_and_drop(&work, &spatial, &pairs, opts, &mut rep, ctx)?.unwrap_or_default()
    };
    for (kind, cand) in cands {
        let after = GlobalHealth::of(&analyze(&cand, &AnalyzeOpts::default(), ctx)?);
        let cand_pairs = pairs_of(&cand, ctx)?.1.len();
        if rep.health_after_drop.is_none() {
            rep.pairs_after_drop = cand_pairs;
            rep.health_after_drop = Some(after);
        }
        if cand_pairs < cur_pairs && after.no_worse_than(&rep.health_before, opts.allow_seams) {
            rep.accepted = true;
            rep.candidate = kind;
            rep.pairs_after_drop = cand_pairs;
            rep.health_after_drop = Some(after);
            cur = cand;
            cur_pairs = cand_pairs;
            break;
        }
    }
    if rep.accepted
        && cur_pairs > 0
        && cur_pairs <= opts.cleanup_max_pairs
        && let Some(c) = &opts.cleanup
    {
        let (out, crep) = remove_self_intersections_with(&cur, c, ctx)?;
        cur_pairs = crep.pairs_after;
        cur = out;
        rep.cleanup = Some(crep);
    }
    rep.pairs_after = cur_pairs;
    Ok((cur, rep))
}

/// Options for [`remove_self_intersections_auto`].
#[derive(Clone, Debug)]
pub struct AutoSelfIntersectionOpts {
    /// The local refill, tried first.
    pub local: RemoveSelfIntersectionsOpts,
    /// The global removal, tried when the local one leaves pairs or strays too far.
    pub global: GlobalSelfIntersectionOpts,
    /// The local result is final only if the bound on its distance to the input
    /// ([`RemoveSelfIntersectionsReport::deviation`]) is at most this fraction of the
    /// bounding-box diagonal. `None`: a local result without remaining pairs is always final.
    /// Default `Some(0.01)`.
    pub max_deviation: Option<f64>,
}

impl Default for AutoSelfIntersectionOpts {
    fn default() -> Self {
        Self {
            local: RemoveSelfIntersectionsOpts::default(),
            global: GlobalSelfIntersectionOpts::default(),
            max_deviation: Some(0.01),
        }
    }
}

/// Which method produced the result of [`remove_self_intersections_auto`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SelfIntersectionMethod {
    /// The input had no intersections.
    Unchanged,
    /// The local refill of MESH-21.
    Local,
    /// The cut-and-drop of MESH-29.
    Global,
}

/// What [`remove_self_intersections_auto`] did.
#[derive(Clone, Debug, PartialEq)]
pub struct AutoSelfIntersectionReport {
    /// The method whose mesh was returned.
    pub method: SelfIntersectionMethod,
    /// Intersecting pairs in the input.
    pub pairs_before: usize,
    /// Intersecting pairs in the result.
    pub pairs_after: usize,
    /// Report of the local refill.
    pub local: RemoveSelfIntersectionsReport,
    /// Report of the global removal; `None` when the local result was good enough.
    pub global: Option<GlobalSelfIntersectionReport>,
}

/// Removes self-intersections: the local refill of MESH-21 first, and when it leaves pairs or
/// strays further from the input than [`AutoSelfIntersectionOpts::max_deviation`], the global
/// cut-and-drop of MESH-29 on the same input. The result with fewer remaining pairs wins; at
/// equal pairs the one with the smaller distance bound. This is the entry point for the repair
/// pipeline (MESH-04).
///
/// # Errors
/// [`crate::MeshError::Cancelled`], or the errors of [`IndexedMesh::validate`].
pub fn remove_self_intersections_auto(
    m: &IndexedMesh,
    opts: &AutoSelfIntersectionOpts,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, AutoSelfIntersectionReport)> {
    let (local, lrep) = remove_self_intersections_with(m, &opts.local, ctx)?;
    let mut rep = AutoSelfIntersectionReport {
        method: if lrep.pairs_before == 0 {
            SelfIntersectionMethod::Unchanged
        } else {
            SelfIntersectionMethod::Local
        },
        pairs_before: lrep.pairs_before,
        pairs_after: lrep.pairs_after,
        local: lrep,
        global: None,
    };
    if rep.pairs_before == 0 {
        return Ok((local, rep));
    }
    let b = m.aabb();
    let budget = opts.max_deviation.map(|d| d * (b.max - b.min).length());
    if rep.local.pairs_after == 0 && budget.is_none_or(|b| rep.local.deviation <= b) {
        return Ok((local, rep));
    }
    let (global, grep) = remove_self_intersections_global(m, &opts.global, ctx)?;
    let gdev = grep.cleanup.as_ref().map_or(0.0, |c| c.deviation);
    let pick_global = if grep.pairs_after == rep.local.pairs_after {
        grep.accepted && gdev < rep.local.deviation
    } else {
        grep.pairs_after < rep.local.pairs_after
    };
    if pick_global {
        rep.method = SelfIntersectionMethod::Global;
        rep.pairs_after = grep.pairs_after;
    }
    rep.global = Some(grep);
    Ok((if pick_global { global } else { local }, rep))
}
