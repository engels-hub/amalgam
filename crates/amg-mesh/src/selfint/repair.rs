//! Self-intersection removal (MESH-21): delete the faces involved plus a k-ring, refill the
//! holes (MESH-20), re-detect (MESH-19), and repeat with a wider ring.
//!
//! One round on mesh `M` with intersecting pairs `P`:
//! 1. The faces of `P` are grown by `k` vertex rings and deleted.
//! 2. Vertices where the remaining faces would form more than one fan (pinch points) and
//!    faces left with no neighbour are deleted as well, so every cut is a set of simple loops.
//! 3. Only the loops of that cut are filled ([`crate::HoleFillOpts`]); holes the input already
//!    had are left alone (unless the cut ate into them, which merges the loops).
//! 4. The candidate is accepted only if it has fewer intersecting pairs and no more open
//!    loops, non-manifold elements or orientation conflicts than the current mesh, and it is
//!    watertight whenever the current mesh is. A rejected round is retried from the current
//!    mesh with a wider ring.
//!
//! The result is the best accepted mesh, so a watertight input is never returned open, and the
//! intersections that could not be removed are counted.

use std::sync::Arc;

use amg_jobs::JobCtx;

use crate::analysis::{AnalysisReport, AnalyzeOpts, analyze};
use crate::halfedge::{HalfEdgeMesh, NonManifoldPolicy};
use crate::holefill::{Fairing, HoleFillOpts, fill_holes_where};
use crate::indexed::{IndexedMesh, WeldOpts};
use crate::spatial::{BvhOpts, MeshSpatial};
use crate::{FaceId, MeshError, MeshResult};

use super::{SelfIntersectOpts, self_intersections};

/// Options for [`remove_self_intersections_with`].
#[derive(Clone, Debug)]
pub struct RemoveSelfIntersectionsOpts {
    /// Delete / refill rounds (a round that finds no better mesh retries with a wider ring).
    /// Default 12.
    pub max_rounds: u32,
    /// Vertex rings deleted around the intersecting faces in round 0; round `r` uses
    /// `first_ring + r` (capped at `max_ring`). Default 1.
    pub first_ring: u32,
    /// Cap of the ring width. Default 6.
    pub max_ring: u32,
    /// Budget for how far the refilled patches may lie from the surface they replace, as a
    /// fraction of the input's bounding-box diagonal (the sum over accepted rounds of the
    /// largest distance of a new face to the previous surface, an upper bound of the
    /// one-sided Hausdorff distance output -> input). A round that would exceed it is
    /// rejected and its intersections are reported as remaining. `None` (the default) = no
    /// limit: on the corpus a 1 % budget leaves most models unrepaired, see the S07 spec.
    pub max_deviation: Option<f64>,
    /// Alternative budget in mean edge lengths of the input; the larger of the two budgets
    /// applies. Hole filling cannot know a surface better than its sampling: on a coarse model
    /// (edges of several percent of the diagonal) a patch is off by a fraction of an edge, far
    /// more than 1 % of the diagonal. Default 0.5.
    pub max_deviation_edges: f64,
    /// A round may delete at most this fraction of the surface area; a larger cut is not a
    /// local repair. Default 0.9.
    pub max_removed_area: f64,
    /// How the cut holes are filled. When a round with these options finds no better mesh it is
    /// tried once more with plain triangulation (no refinement, no fairing), which makes
    /// patches that stay closer to the cut and intersect less.
    pub fill: HoleFillOpts,
}

impl Default for RemoveSelfIntersectionsOpts {
    fn default() -> Self {
        Self {
            max_rounds: 12,
            first_ring: 1,
            max_ring: 6,
            max_deviation: None,
            max_deviation_edges: 0.5,
            max_removed_area: 0.9,
            fill: HoleFillOpts {
                fairing: Fairing::None,
                ..HoleFillOpts::default()
            },
        }
    }
}

/// What [`remove_self_intersections_with`] did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RemoveSelfIntersectionsReport {
    /// Intersecting pairs in the input.
    pub pairs_before: usize,
    /// Intersecting pairs left in the result.
    pub pairs_after: usize,
    /// Rounds run.
    pub rounds: u32,
    /// Rounds whose candidate was accepted.
    pub accepted_rounds: u32,
    /// Faces deleted by accepted rounds.
    pub faces_removed: usize,
    /// Faces added by hole filling in accepted rounds.
    pub faces_added: usize,
    /// Rounds rejected because they would have exceeded the deviation budget (or deleted too
    /// much).
    pub rejected_for_deviation: u32,
    /// Rounds rejected because the candidate had no fewer intersections, or more open loops,
    /// non-manifold elements or orientation conflicts, or lost watertightness.
    pub rejected_not_better: u32,
    /// Rounds whose cut mesh could not be converted to half-edges.
    pub build_failed: u32,
    /// Distance budget spent by accepted rounds, mm (bound on the one-sided Hausdorff distance
    /// output -> input).
    pub deviation: f64,
}

/// One mesh with everything the acceptance test needs.
struct State {
    spatial: MeshSpatial,
    mesh: Arc<IndexedMesh>,
    pairs: Vec<[FaceId; 2]>,
    report: AnalysisReport,
}

/// `cap`: stop counting intersections at this many (a candidate only has to beat the current
/// count, so a mesh that is no better need not be enumerated in full).
fn evaluate(mesh: IndexedMesh, cap: Option<usize>, ctx: &JobCtx) -> MeshResult<State> {
    let report = analyze(&mesh, &AnalyzeOpts::default(), ctx)?;
    let mesh = Arc::new(mesh);
    let spatial = MeshSpatial::build(Arc::clone(&mesh), &BvhOpts::default(), ctx)?;
    let pairs = self_intersections(&spatial, &SelfIntersectOpts { max_pairs: cap }, ctx)?;
    Ok(State {
        spatial,
        mesh,
        pairs,
        report,
    })
}

/// Is `cand` an improvement on `cur` that does not break anything?
fn acceptable(cand: &State, cur: &State) -> bool {
    let (a, b) = (&cand.report, &cur.report);
    cand.pairs.len() < cur.pairs.len()
        && a.boundary_loops.len() <= b.boundary_loops.len()
        && a.non_manifold_edges.len() <= b.non_manifold_edges.len()
        && a.non_manifold_vertices.len() <= b.non_manifold_vertices.len()
        && a.orientation_conflicts <= b.orientation_conflicts
        && (!b.is_watertight() || a.is_watertight())
}

/// Vertex -> incident faces, CSR.
struct VertexFaces {
    offs: Vec<u32>,
    faces: Vec<u32>,
}

impl VertexFaces {
    fn new(m: &IndexedMesh) -> Self {
        let mut offs = vec![0u32; m.positions.len() + 1];
        for t in &m.tris {
            for &v in t {
                offs[v as usize + 1] += 1;
            }
        }
        for i in 0..m.positions.len() {
            offs[i + 1] += offs[i];
        }
        let mut fill = offs.clone();
        let mut faces = vec![0u32; m.tris.len() * 3];
        for (f, t) in m.tris.iter().enumerate() {
            for &v in t {
                faces[fill[v as usize] as usize] = f as u32;
                fill[v as usize] += 1;
            }
        }
        Self { offs, faces }
    }

    fn of(&self, v: u32) -> &[u32] {
        &self.faces[self.offs[v as usize] as usize..self.offs[v as usize + 1] as usize]
    }
}

/// Faces to delete: the pair faces grown by `ring` vertex rings, then closed under the pinch
/// and isolated-face rules. Returns `(deleted, touched)` where `touched` marks every vertex of
/// a deleted face.
fn faces_to_delete(
    m: &IndexedMesh,
    vf: &VertexFaces,
    pairs: &[[FaceId; 2]],
    ring: u32,
) -> (Vec<bool>, Vec<bool>) {
    let mut del = vec![false; m.tris.len()];
    let mut touched = vec![false; m.positions.len()];
    let mut frontier: Vec<u32> = Vec::new();
    for p in pairs {
        for f in p {
            if !del[f.0 as usize] {
                del[f.0 as usize] = true;
                frontier.push(f.0);
            }
        }
    }
    let mut all: Vec<u32> = frontier.clone();
    for _ in 0..ring {
        let mut next = Vec::new();
        for &f in &frontier {
            for &v in &m.tris[f as usize] {
                if touched[v as usize] {
                    continue;
                }
                touched[v as usize] = true;
                for &g in vf.of(v) {
                    if !del[g as usize] {
                        del[g as usize] = true;
                        next.push(g);
                    }
                }
            }
        }
        all.extend(&next);
        frontier = next;
    }
    for &f in &all {
        for &v in &m.tris[f as usize] {
            touched[v as usize] = true;
        }
    }
    close_cut(m, vf, &mut del, &mut touched, &all);
    (del, touched)
}

fn find(p: &mut [usize], mut x: usize) -> usize {
    while p[x] != x {
        p[x] = p[p[x]];
        x = p[x];
    }
    x
}

/// Deletes faces until no vertex of the cut is a pinch point (remaining faces forming more
/// than one fan) and no remaining face is left with all three edges on the cut.
fn close_cut(
    m: &IndexedMesh,
    vf: &VertexFaces,
    del: &mut [bool],
    touched: &mut [bool],
    newly: &[u32],
) {
    let mut work: Vec<u32> = Vec::new();
    for &f in newly {
        work.extend(m.tris[f as usize]);
    }
    let mut fan_nb: Vec<(u32, usize)> = Vec::new();
    while let Some(v) = work.pop() {
        let live: Vec<u32> = vf
            .of(v)
            .iter()
            .copied()
            .filter(|&f| !del[f as usize])
            .collect();
        if live.is_empty() {
            continue;
        }
        // Fans at v: faces connected through an edge at v.
        let mut parent: Vec<usize> = (0..live.len()).collect();
        fan_nb.clear();
        for (i, &f) in live.iter().enumerate() {
            for &w in &m.tris[f as usize] {
                if w == v {
                    continue;
                }
                if let Some(&(_, j)) = fan_nb.iter().find(|(x, _)| *x == w) {
                    let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                    parent[a] = b;
                } else {
                    fan_nb.push((w, i));
                }
            }
        }
        let root = find(&mut parent, 0);
        let mut kill = (1..live.len()).any(|i| find(&mut parent, i) != root);
        // A lone face of the cut with no live neighbour on any edge would be refilled as its
        // own mirror image.
        if !kill && live.len() == 1 && touched[v as usize] {
            let f0 = live[0];
            let t = m.tris[f0 as usize];
            kill = (0..3).all(|k| {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                !vf.of(a)
                    .iter()
                    .any(|&g| g != f0 && !del[g as usize] && m.tris[g as usize].contains(&b))
            });
        }
        if kill {
            for f in live {
                del[f as usize] = true;
                for &w in &m.tris[f as usize] {
                    touched[w as usize] = true;
                    work.push(w);
                }
            }
        }
    }
}

/// `m` without the faces flagged in `del`, vertex array unchanged.
fn without_faces(m: &IndexedMesh, del: &[bool]) -> IndexedMesh {
    let keep: Vec<bool> = del.iter().map(|d| !d).collect();
    let mut attrs = m.attrs.clone();
    attrs.retain(&vec![true; m.positions.len()], &keep);
    let mut groups = m.groups.clone();
    groups.retain(&keep);
    IndexedMesh {
        positions: m.positions.clone(),
        tris: m
            .tris
            .iter()
            .zip(&keep)
            .filter(|(_, k)| **k)
            .map(|(t, _)| *t)
            .collect(),
        attrs,
        groups,
    }
}

/// A built candidate.
struct Candidate {
    mesh: IndexedMesh,
    /// Faces of the current mesh that were deleted.
    deleted: Vec<u32>,
    /// Faces added by filling (they are the last `added` faces of `mesh`).
    added: usize,
}

/// One delete / refill round, or `None` when the cut mesh cannot be turned into a half-edge
/// mesh.
fn attempt(
    cur: &State,
    ring: u32,
    fill: &HoleFillOpts,
    ctx: &JobCtx,
) -> MeshResult<Option<Candidate>> {
    let m: &IndexedMesh = &cur.mesh;
    let vf = VertexFaces::new(m);
    let (del, touched) = faces_to_delete(m, &vf, &cur.pairs, ring);
    ctx.checkpoint()?;
    let cut = without_faces(m, &del);
    let (mut he, conv) = match HalfEdgeMesh::from_indexed(&cut, NonManifoldPolicy::SplitVertices) {
        Ok(x) => x,
        Err(MeshError::Cancelled) => return Err(MeshError::Cancelled),
        Err(_) => return Ok(None),
    };
    let rep = fill_holes_where(&mut he, fill, ctx, &|h, lp| {
        lp.iter().any(|&e| {
            touched
                .get(h.origin(e).0 as usize)
                .copied()
                .unwrap_or(false)
        })
    })?;
    let mut out = he.to_indexed();
    if conv.split_vertices > 0 || conv.cut_edges > 0 {
        // The input had non-manifold places elsewhere; the conversion split them, so merge
        // the bit-equal copies again to leave those places as they were.
        out.weld(WeldOpts::EXACT)?;
    }
    let deleted = (0..del.len() as u32).filter(|&f| del[f as usize]).collect();
    Ok(Some(Candidate {
        mesh: out,
        deleted,
        added: rep.new_faces,
    }))
}

/// Largest distance from the faces filled in by `c` to the surface of `cur` (they are the
/// last `c.added` faces of `cand`), or `None` when it exceeds `limit` or the round deletes
/// more than `max_removed_area` of the surface. The faces that were deleted are not
/// measured: removing the part of a surface that lies inside another part is the point of
/// the repair.
fn deviation(
    cur: &State,
    cand: &State,
    deleted: &[u32],
    added: usize,
    limit: f64,
    max_removed_area: f64,
) -> Option<f64> {
    let m: &IndexedMesh = &cur.mesh;
    let n: &IndexedMesh = &cand.mesh;
    let removed: f64 = deleted.iter().map(|&f| m.face_area(FaceId(f))).sum();
    if removed > max_removed_area * cur.report.area {
        return None;
    }
    let mut worst = 0.0f64;
    for t in n.tris.iter().skip(n.tris.len().saturating_sub(added)) {
        let t = t.map(|i| n.positions[i as usize]);
        let centre = (t[0] + t[1] + t[2]) / 3.0;
        for p in [t[0], t[1], t[2], centre] {
            let h = cur.spatial.closest_point(p, limit)?;
            worst = worst.max(h.dist_sq.sqrt());
        }
    }
    Some(worst)
}

/// Mean edge length over the faces' edges (0 for an empty mesh).
fn mean_edge(m: &IndexedMesh) -> f64 {
    let (mut sum, mut n) = (0.0, 0usize);
    for t in &m.tris {
        for k in 0..3 {
            sum += m.positions[t[k] as usize].distance(m.positions[t[(k + 1) % 3] as usize]);
            n += 1;
        }
    }
    if n == 0 { 0.0 } else { sum / n as f64 }
}

/// Removes self-intersections by local deletion and refilling (see the module docs).
///
/// Faces share vertices by index, so weld the mesh first. The result keeps the input's
/// watertightness and has at most as many open loops and non-manifold elements; it has
/// no intersecting pairs unless [`RemoveSelfIntersectionsReport::pairs_after`] says otherwise.
/// An input without intersections is returned unchanged.
///
/// # Errors
/// [`MeshError::Cancelled`], or the errors of [`IndexedMesh::validate`].
pub fn remove_self_intersections_with(
    m: &IndexedMesh,
    opts: &RemoveSelfIntersectionsOpts,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, RemoveSelfIntersectionsReport)> {
    m.validate()?;
    let mut cur = evaluate(m.clone(), None, ctx)?;
    let mut rep = RemoveSelfIntersectionsReport {
        pairs_before: cur.pairs.len(),
        ..Default::default()
    };
    let limit = opts.max_deviation.map(|r| {
        let b = m.aabb();
        (r * (b.max - b.min).length()).max(opts.max_deviation_edges * mean_edge(m))
    });
    let mut ring = opts.first_ring.max(1);
    for round in 0..opts.max_rounds {
        if cur.pairs.is_empty() {
            break;
        }
        ctx.checkpoint()?;
        ctx.progress(round as f32 / opts.max_rounds as f32);
        rep.rounds += 1;
        let mut accepted = false;
        let r = ring.min(opts.max_ring.max(1));
        let plain = HoleFillOpts {
            refine: false,
            fairing: Fairing::None,
            ..opts.fill.clone()
        };
        let variants: &[&HoleFillOpts] = if opts.fill.refine || opts.fill.fairing != Fairing::None {
            &[&opts.fill, &plain]
        } else {
            &[&opts.fill]
        };
        for fill in variants {
            let Some(c) = attempt(&cur, r, fill, ctx)? else {
                rep.build_failed += 1;
                break;
            };
            let Candidate {
                mesh,
                deleted,
                added,
            } = c;
            let cand = evaluate(mesh, Some(cur.pairs.len()), ctx)?;
            if !acceptable(&cand, &cur) {
                rep.rejected_not_better += 1;
                continue;
            }
            let room = limit.map_or(f64::INFINITY, |l| l - rep.deviation);
            match deviation(&cur, &cand, &deleted, added, room, opts.max_removed_area) {
                Some(d) => {
                    rep.deviation += d;
                    rep.accepted_rounds += 1;
                    rep.faces_removed += deleted.len();
                    rep.faces_added += added;
                    cur = cand;
                    accepted = true;
                    break;
                }
                None => rep.rejected_for_deviation += 1,
            }
        }
        // After progress the leftovers are local again: start narrow. After a miss, widen.
        ring = if accepted {
            opts.first_ring.max(1)
        } else {
            ring + 1
        };
    }
    ctx.progress(1.0);
    rep.pairs_after = cur.pairs.len();
    let mesh = Arc::try_unwrap(cur.mesh).unwrap_or_else(|a| (*a).clone());
    Ok((mesh, rep))
}

/// [`remove_self_intersections_with`] with default options and `max_rounds` rounds; returns the
/// repaired mesh and the number of intersecting pairs that remain.
///
/// # Errors
/// As [`remove_self_intersections_with`].
pub fn remove_self_intersections(
    m: &IndexedMesh,
    max_rounds: u32,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, usize)> {
    let opts = RemoveSelfIntersectionsOpts {
        max_rounds,
        ..Default::default()
    };
    let (out, rep) = remove_self_intersections_with(m, &opts, ctx)?;
    Ok((out, rep.pairs_after))
}
