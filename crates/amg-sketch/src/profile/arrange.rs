//! The arrangement pipeline behind [`detect_profiles`](super::detect_profiles).

use std::collections::{BTreeSet, HashMap};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

use amg_core::{Aabb2, Point2, Segment2, Tolerance, Vec2};
use smallvec::SmallVec;

use super::hash::{EdgeKey, Fnv};
use super::{
    Loop, Profile, ProfileEdge, ProfileFingerprint, ProfileId, ProfileIssue, ProfileOptions,
    ProfileSet, VertexKey,
};
use crate::geom::{BSpline2, Curve2, intersect_detailed};
use crate::ids::EntityId;
use crate::model::{End, EntityOrigin, Sketch};

/// Arrangement counts after bridge removal (for tests: Euler `V − E + F = 1 + C`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Stats {
    /// Vertices with at least one edge.
    pub vertices: usize,
    /// Edges.
    pub edges: usize,
    /// Connected components.
    pub components: usize,
    /// Traced cycles (bounded faces + one outer boundary per component).
    pub cycles: usize,
}

/// One input curve.
struct Src {
    id: EntityId,
    curve: Curve2,
    /// Upper end of the parameter range.
    span: f64,
    /// Length per parameter unit.
    scale: f64,
    bbox: Aabb2,
}

/// A point on a curve that becomes (part of) a vertex.
struct PRec {
    src: u32,
    t: f64,
    pos: Point2,
    key: VertexKey,
}

/// A piece of one source between two vertices (after merging overlaps: an edge).
struct Edge {
    src: u32,
    also: SmallVec<[EntityId; 1]>,
    t0: f64,
    t1: f64,
    v0: u32,
    v1: u32,
}

struct UnionFind(Vec<u32>);

impl UnionFind {
    fn new(n: usize) -> UnionFind {
        UnionFind((0..n).map(idx).collect())
    }
    fn find(&mut self, mut i: u32) -> u32 {
        while self.0[i as usize] != i {
            let g = self.0[self.0[i as usize] as usize];
            self.0[i as usize] = g;
            i = g;
        }
        i
    }
    fn union(&mut self, a: u32, b: u32) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            let (lo, hi) = if a < b { (a, b) } else { (b, a) };
            self.0[hi as usize] = lo;
        }
    }
}

/// `usize` → `u32` index. Sketch sizes are far below `u32::MAX`.
fn idx(i: usize) -> u32 {
    u32::try_from(i).unwrap_or(u32::MAX)
}

fn end_pos(c: &Curve2, end: End) -> Point2 {
    match (c, end) {
        (Curve2::Line(s), End::Start) => s.a,
        (Curve2::Line(s), End::End) => s.b,
        (_, End::Start) => c.eval(0.0),
        (_, End::End) => c.eval(c.param_range().1),
    }
}

pub(crate) fn detect(sketch: &Sketch, opts: &ProfileOptions) -> (ProfileSet, Stats) {
    let weld = if opts.weld_tol.is_finite() && opts.weld_tol > 0.0 {
        opts.weld_tol
    } else {
        ProfileOptions::default().weld_tol
    };
    let gap = if opts.gap_warn_tol.is_finite() {
        opts.gap_warn_tol
    } else {
        0.0
    };
    let tol = Tolerance {
        linear: weld,
        angular: Tolerance::DEFAULT.angular,
    };
    let mut issues = Vec::new();

    // ---- 1. Sources --------------------------------------------------------------------
    let mut srcs: Vec<Src> = Vec::new();
    for (id, e) in sketch.entities() {
        if !e.kind.is_curve()
            || e.construction
            || (!opts.include_projected && matches!(e.origin, EntityOrigin::Projected(_)))
        {
            continue;
        }
        let Some(curve) = sketch.curve(id) else {
            continue;
        };
        let len = curve.length();
        let bbox = curve.bbox();
        if !(len.is_finite() && bbox.min.is_finite() && bbox.max.is_finite()) {
            continue;
        }
        if len <= weld || curve.round().is_some_and(|r| r.r <= weld) {
            issues.push(ProfileIssue::ZeroLength(id));
            continue;
        }
        let span = curve.param_range().1;
        srcs.push(Src {
            id,
            curve,
            span,
            scale: len / span,
            bbox,
        });
    }

    // ---- 2. Intersections ----------------------------------------------------------------
    let mut recs: Vec<PRec> = Vec::new();
    let mut joins: Vec<(u32, u32)> = Vec::new();
    let mut has_hit = vec![false; srcs.len()];
    for (i, j) in broad_phase(&srcs, weld) {
        let (a, b) = (&srcs[i as usize], &srcs[j as usize]);
        let x = intersect_detailed(&a.curve, &b.curve, &tol);
        if x.overlap {
            issues.push(ProfileIssue::Overlap { a: a.id, b: b.id });
        }
        for (k, h) in x.hits.iter().enumerate() {
            let key = VertexKey::Intersection {
                a: a.id,
                b: b.id,
                ordinal: u16::try_from(k).unwrap_or(u16::MAX),
            };
            let r = idx(recs.len());
            recs.push(PRec {
                src: i,
                t: h.ta.clamp(0.0, a.span),
                pos: h.point,
                key,
            });
            recs.push(PRec {
                src: j,
                t: h.tb.clamp(0.0, b.span),
                pos: h.point,
                key,
            });
            joins.push((r, r + 1));
            has_hit[i as usize] = true;
            has_hit[j as usize] = true;
        }
    }
    // Splines may cross themselves: the crossing is a vertex, the loop between the two
    // parameters an edge.
    for (i, s) in srcs.iter().enumerate() {
        let Curve2::BSpline(b) = &s.curve else {
            continue;
        };
        let hits = b.self_intersections(weld);
        if hits.is_empty() {
            continue;
        }
        issues.push(ProfileIssue::SelfIntersection(s.id));
        for (k, &(t1, t2, pos)) in hits.iter().enumerate() {
            let key = VertexKey::Intersection {
                a: s.id,
                b: s.id,
                ordinal: u16::try_from(k).unwrap_or(u16::MAX),
            };
            let r = idx(recs.len());
            recs.push(PRec {
                src: idx(i),
                t: t1,
                pos,
                key,
            });
            recs.push(PRec {
                src: idx(i),
                t: t2,
                pos,
                key,
            });
            joins.push((r, r + 1));
            has_hit[i] = true;
        }
    }
    for (i, s) in srcs.iter().enumerate() {
        if matches!(s.curve, Curve2::Circle(_)) {
            if !has_hit[i] {
                recs.push(PRec {
                    src: idx(i),
                    t: 0.0,
                    pos: s.curve.eval(0.0),
                    key: VertexKey::Seam(s.id),
                });
            }
        } else {
            for (end, t) in [(End::Start, 0.0), (End::End, s.span)] {
                recs.push(PRec {
                    src: idx(i),
                    t,
                    pos: end_pos(&s.curve, end),
                    key: VertexKey::Endpoint(s.id, end),
                });
            }
        }
    }

    // ---- 3. Weld -------------------------------------------------------------------------
    let (vert_of, vpos, vkey) = weld_points(&recs, &joins, weld);

    // ---- 4. Split ------------------------------------------------------------------------
    let mut per_src: Vec<Vec<(f64, u32)>> = vec![Vec::new(); srcs.len()];
    for (r, &v) in recs.iter().zip(&vert_of) {
        per_src[r.src as usize].push((r.t, v));
    }
    let tiny_loop = 4.0 * weld;
    let mut pieces: Vec<Edge> = Vec::new();
    for (i, list) in per_src.iter_mut().enumerate() {
        list.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        list.dedup();
        let s = &srcs[i];
        let mut push = |t0: f64, v0: u32, t1: f64, v1: u32| {
            if v0 == v1 && (t1 - t0) * s.scale <= tiny_loop {
                return;
            }
            pieces.push(Edge {
                src: idx(i),
                also: SmallVec::new(),
                t0,
                t1,
                v0,
                v1,
            });
        };
        for w in list.windows(2) {
            push(w[0].0, w[0].1, w[1].0, w[1].1);
        }
        if matches!(s.curve, Curve2::Circle(_))
            && let (Some(&(tf, vf)), Some(&(tl, vl))) = (list.first(), list.last())
        {
            push(tl, vl, tf + TAU, vf);
        }
    }

    // ---- 5. Merge coincident pieces of overlapping curves -------------------------------
    let edges = merge_pieces(pieces, &srcs, 4.0 * weld);
    let nv = vpos.len();
    let mut deg = vec![0u32; nv];
    for e in &edges {
        deg[e.v0 as usize] += 1;
        deg[e.v1 as usize] += 1;
    }
    issues.extend(near_gaps(&edges, &vpos, &vkey, &deg, weld, gap));

    // ---- 6. Bridges and dangling trees → open chains -------------------------------------
    let bridge = find_bridges(&edges, nv);
    let open_chains = open_chains(&edges, &bridge, &srcs, nv);
    let kept: Vec<u32> = (0..edges.len()).filter(|&e| !bridge[e]).map(idx).collect();

    // ---- 7. Half-edges and rotation system -----------------------------------------------
    // Half-edge 2k runs v0 → v1 of edge kept[k]; 2k + 1 runs back.
    let nh = kept.len() * 2;
    let mut origin = vec![0u32; nh];
    let mut dirs = vec![(Vec2::ZERO, 0.0f64); nh];
    let mut min_len = vec![f64::INFINITY; nv];
    for (k, &e) in kept.iter().enumerate() {
        let e = &edges[e as usize];
        let c = &srcs[e.src as usize].curve;
        let s = &srcs[e.src as usize];
        origin[2 * k] = e.v0;
        origin[2 * k + 1] = e.v1;
        // Curvature at each end (it varies along an ellipse).
        dirs[2 * k] = (c.derivative(e.t0), c.curvature_at(e.t0));
        dirs[2 * k + 1] = (-c.derivative(e.t1), -c.curvature_at(e.t1));
        let len = (e.t1 - e.t0) * s.scale;
        for v in [e.v0, e.v1] {
            min_len[v as usize] = min_len[v as usize].min(len);
        }
    }
    let mut rot: Vec<Vec<u32>> = vec![Vec::new(); nv];
    for (h, &v) in origin.iter().enumerate() {
        rot[v as usize].push(idx(h));
    }
    let mut pos_in_rot = vec![0u32; nh];
    for (v, hs) in rot.iter_mut().enumerate() {
        // Direction of the chord to the point at arc length δ along each half-edge: the
        // tangent angle plus half the angle turned, so tangent contacts are ordered by
        // curvature. δ is a few weld tolerances (the scale below which a crossing would have
        // been welded into this vertex) and at most a quarter of the shortest edge here.
        let delta = (8.0 * weld).min(0.25 * min_len[v]);
        let key = |h: u32| {
            let (d, kappa) = dirs[h as usize];
            (d.y.atan2(d.x) + 0.5 * kappa * delta).rem_euclid(TAU)
        };
        hs.sort_by(|&a, &b| key(a).total_cmp(&key(b)).then(a.cmp(&b)));
        for (i, &h) in hs.iter().enumerate() {
            pos_in_rot[h as usize] = idx(i);
        }
    }
    let next = |h: u32| -> u32 {
        let t = h ^ 1;
        let hs = &rot[origin[t as usize] as usize];
        let p = pos_in_rot[t as usize] as usize;
        hs[(p + hs.len() - 1) % hs.len()]
    };

    // ---- 8. Cycles -----------------------------------------------------------------------
    let mut uf = UnionFind::new(nv);
    for &e in &kept {
        let e = &edges[e as usize];
        uf.union(e.v0, e.v1);
    }
    let mut seen = vec![false; nh];
    let mut cycles: Vec<Cycle> = Vec::new();
    for h0 in 0..nh {
        if seen[h0] {
            continue;
        }
        let mut hs = Vec::new();
        let mut h = idx(h0);
        while !seen[h as usize] && hs.len() <= nh {
            seen[h as usize] = true;
            hs.push(h);
            h = next(h);
        }
        let comp = uf.find(origin[h0]);
        cycles.push(Cycle::new(
            &hs, comp, &kept, &edges, &srcs, &origin, &vpos, &vkey,
        ));
    }
    let mut comps: BTreeSet<u32> = BTreeSet::new();
    let mut used = vec![false; nv];
    for &v in &origin {
        used[v as usize] = true;
        comps.insert(uf.find(v));
    }
    let stats = Stats {
        vertices: used.iter().filter(|&&u| u).count(),
        edges: kept.len(),
        components: comps.len(),
        cycles: cycles.len(),
    };

    // ---- 9. Faces, holes, profiles -------------------------------------------------------
    // Per component the most negative cycle bounds the unbounded side; the others are faces.
    let mut outer_of: HashMap<u32, usize> = HashMap::new();
    for (i, c) in cycles.iter().enumerate() {
        let e = outer_of.entry(c.comp).or_insert(i);
        if c.area < cycles[*e].area {
            *e = i;
        }
    }
    let mut is_face = vec![false; cycles.len()];
    for (i, c) in cycles.iter().enumerate() {
        is_face[i] = outer_of[&c.comp] != i && c.area > 0.5 * weld * c.perimeter;
    }
    let faces: Vec<usize> = (0..cycles.len()).filter(|&i| is_face[i]).collect();
    let mut holes_of: Vec<Vec<usize>> = vec![Vec::new(); cycles.len()];
    let mut outers: Vec<usize> = outer_of.values().copied().collect();
    outers.sort_unstable();
    for o in outers {
        let oc = &cycles[o];
        let p = vpos[origin[oc.hs[0] as usize] as usize];
        let host = faces
            .iter()
            .copied()
            .filter(|&f| {
                let fc = &cycles[f];
                fc.comp != oc.comp
                    && p.cmpge(fc.bbox.min).all()
                    && p.cmple(fc.bbox.max).all()
                    && loop_contains(fc.edges.iter().map(|e| &e.curve), p)
            })
            .min_by(|&a, &b| cycles[a].area.total_cmp(&cycles[b].area).then(a.cmp(&b)));
        if let Some(f) = host {
            holes_of[f].push(o);
        }
    }

    let mut profiles = Vec::with_capacity(faces.len());
    for f in faces {
        let fc = &cycles[f];
        let mut keys: Vec<EdgeKey> = fc.keys.clone();
        let mut area = fc.area;
        let mut moment = fc.moment;
        let mut outer_sources: Vec<EntityId> = fc.sources();
        outer_sources.sort_unstable();
        outer_sources.dedup();
        let mut sources: Vec<EntityId> = outer_sources.clone();
        let mut holes: Vec<(EdgeKey, Loop)> = Vec::new();
        for &h in &holes_of[f] {
            let hc = &cycles[h];
            keys.extend_from_slice(&hc.keys);
            area += hc.area;
            moment += hc.moment;
            sources.extend(hc.sources());
            holes.push(hc.to_loop());
        }
        holes.sort_by_key(|h| h.0);
        keys.sort_unstable();
        let mut hasher = Fnv::new();
        for k in &keys {
            hasher.edge(k);
        }
        sources.sort_unstable();
        sources.dedup();
        let centroid = if area != 0.0 {
            moment / area
        } else {
            fc.bbox.center()
        };
        profiles.push(Profile {
            id: ProfileId(hasher.finish()),
            outer: fc.to_loop().1,
            holes: holes.into_iter().map(|h| h.1).collect(),
            fingerprint: ProfileFingerprint {
                area,
                centroid,
                bbox: fc.bbox,
                sources,
                outer_sources,
            },
        });
    }
    profiles.sort_by_key(|p| p.id);

    (
        ProfileSet {
            profiles,
            open_chains,
            issues,
        },
        stats,
    )
}

/// Index pairs `i < j` of sources whose padded boxes overlap, sorted.
fn broad_phase(srcs: &[Src], pad: f64) -> Vec<(u32, u32)> {
    let boxes: Vec<Aabb2> = srcs.iter().map(|s| s.bbox.expanded(pad)).collect();
    let mut order: Vec<u32> = (0..srcs.len()).map(idx).collect();
    order.sort_by(|&a, &b| {
        boxes[a as usize]
            .min
            .x
            .total_cmp(&boxes[b as usize].min.x)
            .then(a.cmp(&b))
    });
    let mut active: Vec<u32> = Vec::new();
    let mut pairs = Vec::new();
    for &i in &order {
        let bi = boxes[i as usize];
        active.retain(|&j| boxes[j as usize].max.x >= bi.min.x);
        for &j in &active {
            let bj = boxes[j as usize];
            if bj.min.y <= bi.max.y && bi.min.y <= bj.max.y {
                pairs.push((i.min(j), i.max(j)));
            }
        }
        active.push(i);
    }
    pairs.sort_unstable();
    pairs
}

/// Welds points closer than `weld` (and the explicit `joins`). Returns the vertex of each
/// record and per vertex its position and key (the smallest member key). Vertices are
/// numbered in key order.
fn weld_points(
    recs: &[PRec],
    joins: &[(u32, u32)],
    weld: f64,
) -> (Vec<u32>, Vec<Point2>, Vec<VertexKey>) {
    let mut uf = UnionFind::new(recs.len());
    for &(a, b) in joins {
        uf.union(a, b);
    }
    let inv = 1.0 / weld;
    #[allow(
        clippy::cast_possible_truncation,
        reason = "grid cell index; saturating casts only merge far-away cells"
    )]
    let cell = |p: Point2| ((p.x * inv).floor() as i64, (p.y * inv).floor() as i64);
    let mut grid: HashMap<(i64, i64), SmallVec<[u32; 4]>> = HashMap::with_capacity(recs.len());
    let dup = 1e-3 * weld;
    for (i, r) in recs.iter().enumerate() {
        let (cx, cy) = cell(r.pos);
        let mut duplicate = false;
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(list) = grid.get(&(cx + dx, cy + dy)) {
                    for &j in list {
                        let d = recs[j as usize].pos.distance(r.pos);
                        if d <= weld {
                            uf.union(idx(i), j);
                            duplicate |= d <= dup;
                        }
                    }
                }
            }
        }
        // Points (almost) on top of a stored one add nothing for later queries; skipping them
        // keeps clusters (many curves through one point) linear.
        if !duplicate {
            grid.entry((cx, cy)).or_default().push(idx(i));
        }
    }
    let mut best: Vec<Option<u32>> = vec![None; recs.len()];
    for i in 0..recs.len() {
        let r = uf.find(idx(i)) as usize;
        if best[r].is_none_or(|b| recs[i].key < recs[b as usize].key) {
            best[r] = Some(idx(i));
        }
    }
    let mut roots: Vec<(VertexKey, u32)> = best
        .iter()
        .enumerate()
        .filter_map(|(r, b)| b.map(|b| (recs[b as usize].key, idx(r))))
        .collect();
    roots.sort_unstable();
    // Position: an intersection point if the vertex has one (it lies on both curves, while a
    // touching end point may be up to `weld` off the curve it touches), else the key's point.
    let mut at: Vec<Option<u32>> = vec![None; recs.len()];
    for i in 0..recs.len() {
        if matches!(recs[i].key, VertexKey::Intersection { .. }) {
            let r = uf.find(idx(i)) as usize;
            if at[r].is_none_or(|b| recs[i].key < recs[b as usize].key) {
                at[r] = Some(idx(i));
            }
        }
    }
    let mut vid = vec![0u32; recs.len()];
    let mut vpos = Vec::with_capacity(roots.len());
    let mut vkey = Vec::with_capacity(roots.len());
    for (v, &(key, r)) in roots.iter().enumerate() {
        vid[r as usize] = idx(v);
        vkey.push(key);
        let src = at[r as usize]
            .or(best[r as usize])
            .map_or(0, |b| b as usize);
        vpos.push(recs[src].pos);
    }
    let vert_of = (0..recs.len())
        .map(|i| vid[uf.find(idx(i)) as usize])
        .collect();
    (vert_of, vpos, vkey)
}

/// Merges pieces that join the same two vertices along the same path (midpoints within
/// `tol`): the coincident spans of overlapping curves. The edge keeps the smallest source.
fn merge_pieces(mut pieces: Vec<Edge>, srcs: &[Src], tol: f64) -> Vec<Edge> {
    let ends = |e: &Edge| (e.v0.min(e.v1), e.v0.max(e.v1));
    pieces.sort_by(|a, b| {
        ends(a)
            .cmp(&ends(b))
            .then(a.src.cmp(&b.src))
            .then(a.t0.total_cmp(&b.t0))
    });
    let mid = |e: &Edge| srcs[e.src as usize].curve.eval(0.5 * (e.t0 + e.t1));
    let mut out: Vec<Edge> = Vec::with_capacity(pieces.len());
    let mut group_start = 0;
    for p in pieces {
        if out.last().is_none_or(|l| ends(l) != ends(&p)) {
            group_start = out.len();
        }
        let m = mid(&p);
        let id = srcs[p.src as usize].id;
        if let Some(e) = out[group_start..]
            .iter_mut()
            .find(|e| mid(e).distance(m) <= tol)
        {
            if srcs[e.src as usize].id != id && !e.also.contains(&id) {
                e.also.push(id);
            }
            continue;
        }
        out.push(p);
    }
    out
}

/// End-point vertices of degree 1 within `(weld, gap]` of another end-point vertex they are
/// not joined to by an edge.
fn near_gaps(
    edges: &[Edge],
    vpos: &[Point2],
    vkey: &[VertexKey],
    deg: &[u32],
    weld: f64,
    gap: f64,
) -> Vec<ProfileIssue> {
    if gap <= weld {
        return Vec::new();
    }
    let is_end = |v: usize| matches!(vkey[v], VertexKey::Endpoint(..));
    let adjacent: BTreeSet<(u32, u32)> = edges
        .iter()
        .map(|e| (e.v0.min(e.v1), e.v0.max(e.v1)))
        .collect();
    let inv = 1.0 / gap;
    #[allow(
        clippy::cast_possible_truncation,
        reason = "grid cell index; saturating casts only merge far-away cells"
    )]
    let cell = |p: Point2| ((p.x * inv).floor() as i64, (p.y * inv).floor() as i64);
    let mut grid: HashMap<(i64, i64), SmallVec<[u32; 2]>> = HashMap::new();
    for v in (0..vpos.len()).filter(|&v| is_end(v)) {
        grid.entry(cell(vpos[v])).or_default().push(idx(v));
    }
    let mut found: BTreeSet<(u32, u32)> = BTreeSet::new();
    for u in (0..vpos.len()).filter(|&v| is_end(v) && deg[v] == 1) {
        let (cx, cy) = cell(vpos[u]);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for &w in grid.get(&(cx + dx, cy + dy)).into_iter().flatten() {
                    let pair = (idx(u).min(w), idx(u).max(w));
                    let d = vpos[u].distance(vpos[w as usize]);
                    if w as usize != u && d > weld && d <= gap && !adjacent.contains(&pair) {
                        found.insert(pair);
                    }
                }
            }
        }
    }
    found
        .into_iter()
        .map(|(a, b)| ProfileIssue::NearGap {
            a: vkey[a as usize],
            b: vkey[b as usize],
            distance: vpos[a as usize].distance(vpos[b as usize]),
        })
        .collect()
}

/// Bridges of the multigraph (iterative Tarjan). Self-loops and parallel edges are never
/// bridges.
fn find_bridges(edges: &[Edge], nv: usize) -> Vec<bool> {
    let mut adj: Vec<Vec<(u32, u32)>> = vec![Vec::new(); nv];
    for (i, e) in edges.iter().enumerate() {
        if e.v0 != e.v1 {
            adj[e.v0 as usize].push((e.v1, idx(i)));
            adj[e.v1 as usize].push((e.v0, idx(i)));
        }
    }
    const UNSEEN: u32 = u32::MAX;
    let mut disc = vec![UNSEEN; nv];
    let mut low = vec![0u32; nv];
    let mut bridge = vec![false; edges.len()];
    let mut timer = 0u32;
    let mut stack: Vec<(u32, u32, usize)> = Vec::new();
    for root in 0..nv {
        if disc[root] != UNSEEN || adj[root].is_empty() {
            continue;
        }
        disc[root] = timer;
        low[root] = timer;
        timer += 1;
        stack.push((idx(root), UNSEEN, 0));
        while let Some(top) = stack.last_mut() {
            let (v, parent_edge) = (top.0 as usize, top.1);
            if top.2 < adj[v].len() {
                let (w, e) = adj[v][top.2];
                top.2 += 1;
                if e == parent_edge {
                    continue;
                }
                let w = w as usize;
                if disc[w] == UNSEEN {
                    disc[w] = timer;
                    low[w] = timer;
                    timer += 1;
                    stack.push((idx(w), e, 0));
                } else {
                    low[v] = low[v].min(disc[w]);
                }
            } else {
                stack.pop();
                if let Some(&(p, _, _)) = stack.last() {
                    let p = p as usize;
                    low[p] = low[p].min(low[v]);
                    if low[v] > disc[p] {
                        bridge[parent_edge as usize] = true;
                    }
                }
            }
        }
    }
    bridge
}

/// Connected groups of bridge edges as sorted source-id lists.
fn open_chains(edges: &[Edge], bridge: &[bool], srcs: &[Src], nv: usize) -> Vec<Vec<EntityId>> {
    let mut uf = UnionFind::new(nv);
    for (e, _) in edges.iter().zip(bridge).filter(|(_, b)| **b) {
        uf.union(e.v0, e.v1);
    }
    let mut groups: HashMap<u32, Vec<EntityId>> = HashMap::new();
    for (e, _) in edges.iter().zip(bridge).filter(|(_, b)| **b) {
        let g = groups.entry(uf.find(e.v0)).or_default();
        g.push(srcs[e.src as usize].id);
        g.extend_from_slice(&e.also);
    }
    let mut out: Vec<Vec<EntityId>> = groups
        .into_values()
        .map(|mut g| {
            g.sort_unstable();
            g.dedup();
            g
        })
        .collect();
    out.sort();
    out
}

/// One traced face cycle.
struct Cycle {
    hs: Vec<u32>,
    comp: u32,
    edges: Vec<ProfileEdge>,
    keys: Vec<EdgeKey>,
    area: f64,
    /// First moment of area about the sketch origin.
    moment: Vec2,
    perimeter: f64,
    bbox: Aabb2,
}

impl Cycle {
    #[allow(
        clippy::too_many_arguments,
        reason = "internal helper over the pipeline's arrays"
    )]
    fn new(
        hs: &[u32],
        comp: u32,
        kept: &[u32],
        edges: &[Edge],
        srcs: &[Src],
        origin: &[u32],
        vpos: &[Point2],
        vkey: &[VertexKey],
    ) -> Cycle {
        let mut out = Vec::with_capacity(hs.len());
        let mut keys = Vec::with_capacity(hs.len());
        let mut perimeter = 0.0;
        for &h in hs {
            let e = &edges[kept[(h / 2) as usize] as usize];
            let s = &srcs[e.src as usize];
            let forward = h % 2 == 0;
            let (a, b) = (origin[h as usize], origin[(h ^ 1) as usize]);
            let curve = match s.curve {
                Curve2::Line(_) => Curve2::Line(Segment2 {
                    a: vpos[a as usize],
                    b: vpos[b as usize],
                }),
                _ if forward => s.curve.trimmed(e.t0, e.t1),
                _ => s.curve.trimmed(e.t1, e.t0),
            };
            perimeter += (e.t1 - e.t0) * s.scale;
            let edge = ProfileEdge {
                source: s.id,
                also: e.also.clone(),
                curve,
                start: vkey[a as usize],
                end: vkey[b as usize],
            };
            keys.push((edge.source, edge.start, edge.end, forward));
            out.push(edge);
        }
        let curves: Vec<&Curve2> = out.iter().map(|e| &e.curve).collect();
        let starts: Vec<Point2> = hs
            .iter()
            .map(|&h| vpos[origin[h as usize] as usize])
            .collect();
        let (area, moment) = area_moment(&curves, &starts);
        let bbox = out
            .iter()
            .fold(Aabb2::EMPTY, |b, e| b.union(&e.curve.bbox()));
        Cycle {
            hs: hs.to_vec(),
            comp,
            edges: out,
            keys,
            area,
            moment,
            perimeter,
            bbox,
        }
    }

    fn sources(&self) -> Vec<EntityId> {
        self.edges
            .iter()
            .flat_map(|e| std::iter::once(e.source).chain(e.also.iter().copied()))
            .collect()
    }

    /// The loop rotated to start at its smallest edge key, and that key.
    fn to_loop(&self) -> (EdgeKey, Loop) {
        let start = (0..self.keys.len())
            .min_by(|&a, &b| self.keys[a].cmp(&self.keys[b]))
            .unwrap_or(0);
        let mut edges = self.edges.clone();
        edges.rotate_left(start);
        let key = self.keys.get(start).copied().unwrap_or((
            EntityId(0),
            VertexKey::Seam(EntityId(0)),
            VertexKey::Seam(EntityId(0)),
            true,
        ));
        (key, Loop { edges })
    }
}

fn dir(a: f64) -> Vec2 {
    Vec2::new(a.cos(), a.sin())
}

fn start_point(c: &Curve2) -> Point2 {
    match c {
        Curve2::Line(s) => s.a,
        _ => c.eval(0.0),
    }
}

fn end_point(c: &Curve2) -> Point2 {
    match c {
        Curve2::Line(s) => s.b,
        _ => c.eval(c.param_range().1),
    }
}

/// Signed area and first moment (about the sketch origin) of a closed loop of curves:
/// polygon through the start points plus the circular segment of every arc.
pub(crate) fn loop_area_moment<'a>(curves: impl Iterator<Item = &'a Curve2>) -> (f64, Vec2) {
    let curves: Vec<&Curve2> = curves.collect();
    let starts: Vec<Point2> = curves.iter().map(|c| start_point(c)).collect();
    area_moment(&curves, &starts)
}

/// [`loop_area_moment`] with the polygon running through `starts` (the welded vertices), so
/// arcs whose end points are off their vertex by up to the weld tolerance add no spikes.
fn area_moment(curves: &[&Curve2], starts: &[Point2]) -> (f64, Vec2) {
    let Some(&o) = starts.first() else {
        return (0.0, Vec2::ZERO);
    };
    let n = curves.len();
    let mut area = 0.0;
    let mut m = Vec2::ZERO;
    for i in 0..n {
        let q0 = starts[i] - o;
        let q1 = starts[(i + 1) % n] - o;
        let cr = q0.perp_dot(q1);
        area += 0.5 * cr;
        m += (q0 + q1) * (cr / 6.0);
        let arc = match curves[i] {
            Curve2::Arc(a) => Some((a.center, a.radius, a.start_angle, a.sweep)),
            Curve2::Circle(c) => Some((c.center, c.radius, 0.0, TAU)),
            Curve2::Line(_) => None,
            Curve2::BSpline(b) => {
                // Region between the spline and its chord (exact for polynomial splines,
                // Gauss–Legendre otherwise).
                let (seg, mom) = b.segment_area_moment();
                area += seg;
                m += mom - o * seg;
                continue;
            }
            Curve2::Ellipse(e) => {
                // Segment between the chord and the arc: the affine image of a unit-circle
                // segment (centre `c`, axes `m` and `w`, area scale `a·b`).
                let sweep = e.sweep();
                let alpha = sweep.abs();
                let (a, b) = (e.semi_major(), e.minor_radius);
                let seg = 0.5 * a * b * (sweep - sweep.sin());
                area += seg;
                let mid = e.t0 + 0.5 * sweep;
                let axis = e.major * mid.cos() + e.minor() * mid.sin();
                m += (e.center - o) * seg
                    + axis * (sweep.signum() * (2.0 / 3.0) * a * b * (0.5 * alpha).sin().powi(3));
                continue;
            }
        };
        if let Some((c, r, s, sweep)) = arc {
            let alpha = sweep.abs();
            let ab = 0.5 * r * r * (sweep - sweep.sin());
            area += ab;
            let sgn = sweep.signum();
            m += (c - o) * ab
                + dir(s + 0.5 * sweep)
                    * (sgn * (2.0 / 3.0) * r.powi(3) * (0.5 * alpha).sin().powi(3));
        }
    }
    (area, m + o * area)
}

/// Toggles `inside` for every crossing of the ray from `p` to `+x` with the elliptical arc `e`
/// that runs from `q_start` to `q_end` (the loop's vertices, which the arc ends may miss by up
/// to the weld tolerance). The arc is split at the extrema of `y(θ)`, where it is monotone, and
/// each monotone piece that straddles the ray's height is solved by bisection.
fn ellipse_crossings(
    e: &crate::geom::EllipseArc2,
    q_start: Point2,
    q_end: Point2,
    p: Point2,
    inside: &mut bool,
) {
    let (lo, hi) = (e.t0.min(e.t1), e.t0.max(e.t1));
    let base = e.minor().y.atan2(e.major.y);
    let mut cuts: SmallVec<[f64; 4]> = SmallVec::new();
    let mut phi = ((lo - base) / PI).ceil() * PI + base;
    while phi < hi && cuts.len() < 4 {
        if phi > lo {
            cuts.push(phi);
        }
        phi += PI;
    }
    if e.t1 < e.t0 {
        cuts.reverse();
    }
    let y_of = |th: f64| e.point_at(th).y;
    let (mut q0, mut a0) = (q_start, e.t0);
    for a1 in cuts.into_iter().chain(std::iter::once(e.t1)) {
        let q1 = if a1.to_bits() == e.t1.to_bits() {
            q_end
        } else {
            e.point_at(a1)
        };
        if (q0.y > p.y) != (q1.y > p.y) {
            let above = q0.y > p.y;
            let (mut l, mut h) = (a0, a1);
            for _ in 0..80 {
                let mid = 0.5 * (l + h);
                if (y_of(mid) > p.y) == above {
                    l = mid;
                } else {
                    h = mid;
                }
            }
            if e.point_at(0.5 * (l + h)).x > p.x {
                *inside = !*inside;
            }
        }
        q0 = q1;
        a0 = a1;
    }
}

/// Toggles `inside` for every crossing of the ray from `p` to `+x` with the spline `b` that runs
/// from `q_start` to `q_end` (the loop's vertices). The spline is cut where `y` has an extremum;
/// every piece that straddles the ray's height is solved by bisection.
fn spline_crossings(b: &BSpline2, q_start: Point2, q_end: Point2, p: Point2, inside: &mut bool) {
    let cuts = b.monotone_y();
    let y_of = |t: f64| b.eval(t).y;
    let last = cuts.len() - 1;
    let (mut q0, mut a0) = (q_start, cuts[0]);
    for (k, &a1) in cuts.iter().enumerate().skip(1) {
        let q1 = if k == last { q_end } else { b.eval(a1) };
        if (q0.y > p.y) != (q1.y > p.y) {
            let above = q0.y > p.y;
            let (mut l, mut h) = (a0, a1);
            for _ in 0..80 {
                let mid = 0.5 * (l + h);
                if (y_of(mid) > p.y) == above {
                    l = mid;
                } else {
                    h = mid;
                }
            }
            if b.eval(0.5 * (l + h)).x > p.x {
                *inside = !*inside;
            }
        }
        q0 = q1;
        a0 = a1;
    }
}

/// Crossing parity of a horizontal ray from `p` to `+x` with a closed loop. Each edge starts at
/// the previous edge's end point, so shared vertices are counted consistently (half-open
/// rule).
pub(crate) fn loop_contains<'a>(curves: impl Iterator<Item = &'a Curve2>, p: Point2) -> bool {
    let curves: Vec<&Curve2> = curves.collect();
    let Some(last) = curves.last() else {
        return false;
    };
    let mut inside = false;
    let mut prev = end_point(last);
    let toggle_line = |q0: Point2, q1: Point2, inside: &mut bool| {
        if (q0.y > p.y) != (q1.y > p.y) {
            let x = q0.x + (p.y - q0.y) * (q1.x - q0.x) / (q1.y - q0.y);
            if x > p.x {
                *inside = !*inside;
            }
        }
    };
    for c in curves {
        let end = end_point(c);
        let arc = match c {
            Curve2::Arc(a) => Some((a.center, a.radius, a.start_angle, a.sweep)),
            Curve2::Circle(k) => Some((k.center, k.radius, 0.0, TAU)),
            Curve2::Line(_) => None,
            Curve2::Ellipse(e) => {
                ellipse_crossings(e, prev, end, p, &mut inside);
                prev = end;
                continue;
            }
            Curve2::BSpline(b) => {
                spline_crossings(b, prev, end, p, &mut inside);
                prev = end;
                continue;
            }
        };
        match arc {
            None => toggle_line(prev, end, &mut inside),
            Some((ctr, r, s, sweep)) => {
                // Split into y-monotone pieces at the top/bottom points (θ = π/2 + kπ).
                let (lo, hi) = (s.min(s + sweep), s.max(s + sweep));
                let mut cuts: SmallVec<[f64; 4]> = SmallVec::new();
                let mut phi = ((lo - FRAC_PI_2) / PI).ceil() * PI + FRAC_PI_2;
                while phi < hi && cuts.len() < 4 {
                    if phi > lo {
                        cuts.push(phi);
                    }
                    phi += PI;
                }
                if sweep < 0.0 {
                    cuts.reverse();
                }
                let mut q0 = prev;
                let mut a0 = s;
                for a1 in cuts.into_iter().chain(std::iter::once(s + sweep)) {
                    #[allow(clippy::float_cmp, reason = "the last cut is exactly `s + sweep`")]
                    let q1 = if a1 == s + sweep {
                        end
                    } else {
                        ctr + dir(a1) * r
                    };
                    if (q0.y > p.y) != (q1.y > p.y) {
                        let mid = 0.5 * (a0 + a1);
                        let w = (r * r - (p.y - ctr.y).powi(2)).max(0.0).sqrt();
                        let x = ctr.x + mid.cos().signum() * w;
                        if x > p.x {
                            inside = !inside;
                        }
                    }
                    q0 = q1;
                    a0 = a1;
                }
            }
        }
        prev = end;
    }
    inside
}
