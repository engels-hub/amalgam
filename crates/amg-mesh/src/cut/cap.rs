//! Section loops and CDT caps.
//!
//! The in-plane boundary of a cut half is a set of directed edges (the half's own winding,
//! material on the left seen from the plane's +normal side). [`trace`] chains them into
//! polylines; [`triangulate`] caps them with a constrained Delaunay triangulation without
//! tracing: every cap triangle is flood-filled from the left face of a reversed boundary edge,
//! so nested loops (holes, islands) and loops that touch at a vertex need no special case.

use std::collections::BTreeMap;

use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

/// A chain of vertex ids; `closed` chains do not repeat their first id.
#[derive(Clone, Debug)]
pub(super) struct Chain {
    pub ids: Vec<u32>,
    pub closed: bool,
}

/// Chains directed edges into polylines. Open chains (unbalanced vertices) come first, then
/// closed loops, each starting at its smallest remaining edge. At a vertex with several unused
/// outgoing edges (a pinch) the edge that turns most tightly keeps the loop around one
/// material region.
pub(super) fn trace(edges: &[(u32, u32)], uv: &dyn Fn(u32) -> (f64, f64)) -> Vec<Chain> {
    let mut e: Vec<(u32, u32)> = edges.to_vec();
    e.sort_unstable();
    e.dedup();
    let mut used = vec![false; e.len()];
    let mut balance: BTreeMap<u32, i64> = BTreeMap::new();
    for &(a, b) in &e {
        *balance.entry(a).or_default() += 1;
        *balance.entry(b).or_default() -= 1;
    }
    let first_out = |v: u32| e.partition_point(|x| x.0 < v);
    let mut out = Vec::new();

    let follow = |start: usize, used: &mut [bool], open_if_stuck: bool| -> Chain {
        let mut ids = vec![e[start].0];
        let mut cur = start;
        used[cur] = true;
        let origin = e[start].0;
        loop {
            let (u, v) = e[cur];
            if v == origin && !open_if_stuck {
                return Chain { ids, closed: true };
            }
            ids.push(v);
            let cands: Vec<usize> = (first_out(v)..e.len())
                .take_while(|&k| e[k].0 == v)
                .filter(|&k| !used[k])
                .collect();
            let next = match cands.len() {
                0 => return Chain { ids, closed: false },
                1 => cands[0],
                _ => {
                    let (ux, uy) = uv(u);
                    let (vx, vy) = uv(v);
                    let back = (uy - vy).atan2(ux - vx);
                    let mut best = (f64::INFINITY, cands[0]);
                    for &k in &cands {
                        let (wx, wy) = uv(e[k].1);
                        let dir = (wy - vy).atan2(wx - vx);
                        let mut cw = (back - dir).rem_euclid(std::f64::consts::TAU);
                        if cw < 1e-12 {
                            cw = std::f64::consts::TAU;
                        }
                        if cw < best.0 {
                            best = (cw, k);
                        }
                    }
                    best.1
                }
            };
            used[next] = true;
            cur = next;
        }
    };

    let starts: Vec<(u32, i64)> = balance
        .iter()
        .filter(|&(_, &b)| b > 0)
        .map(|(&v, &b)| (v, b))
        .collect();
    for (v, b) in starts {
        for _ in 0..b {
            if let Some(k) = (first_out(v)..e.len())
                .take_while(|&k| e[k].0 == v)
                .find(|&k| !used[k])
            {
                out.push(follow(k, &mut used, true));
            }
        }
    }
    for k in 0..e.len() {
        if !used[k] {
            out.push(follow(k, &mut used, false));
        }
    }
    out
}

/// Flushes values spade rejects (non-zero below 2^-142).
fn flush(x: f64) -> f64 {
    if x.abs() < 1e-30 { 0.0 } else { x }
}

/// CDT cap of the loops formed by `edges` (directed as the cut half's own boundary; the cap
/// uses them reversed). `uv` gives the 2D coordinates, mirrored so that the cap's outward
/// normal faces the viewer: the returned triangles are counter-clockwise in those coordinates.
///
/// `None` when the 2D problem is inconsistent: coincident points, crossing or split
/// constraints, or an orientation conflict between the loops.
pub(super) fn triangulate(
    edges: &[(u32, u32)],
    uv: &dyn Fn(u32) -> (f64, f64),
) -> Option<Vec<[u32; 3]>> {
    triangulate_checked(edges, uv).ok()
}

/// [`triangulate`] with the reason for a failure.
fn triangulate_checked(
    edges: &[(u32, u32)],
    uv: &dyn Fn(u32) -> (f64, f64),
) -> Result<Vec<[u32; 3]>, &'static str> {
    let mut ids: Vec<u32> = edges.iter().flat_map(|e| [e.0, e.1]).collect();
    ids.sort_unstable();
    ids.dedup();
    let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f64>>::new();
    let mut handles = Vec::with_capacity(ids.len());
    for &id in &ids {
        let (x, y) = uv(id);
        handles.push(
            cdt.insert(Point2::new(flush(x), flush(y)))
                .map_err(|_| "insert")?,
        );
    }
    let mut seen: Vec<usize> = handles.iter().map(|h| h.index()).collect();
    seen.sort_unstable();
    seen.dedup();
    if seen.len() != ids.len() {
        return Err("dup points");
    }
    let handle = |id: u32| ids.binary_search(&id).ok().map(|i| handles[i]);
    let mut cap_edges = Vec::with_capacity(edges.len());
    for &(u, v) in edges {
        let (a, b) = (handle(v).ok_or("h")?, handle(u).ok_or("h")?);
        if a == b {
            return Err("self edge");
        }
        if !cdt.can_add_constraint(a, b) {
            return Err("cannot add constraint");
        }
        cdt.add_constraint(a, b);
        cap_edges.push((a, b));
    }
    let mut inside = vec![false; cdt.num_all_faces()];
    let mut stack = Vec::new();
    for &(a, b) in &cap_edges {
        let de = cdt.get_edge_from_neighbors(a, b).ok_or("no edge")?;
        if !de.is_constraint_edge() {
            return Err("split"); // split by a vertex on the segment
        }
        let f = de.face().fix();
        f.as_inner().ok_or("outer left")?;
        if !inside[f.index()] {
            inside[f.index()] = true;
            stack.push(f);
        }
    }
    while let Some(f) = stack.pop() {
        let Some(inner) = f.as_inner() else { continue };
        for de in cdt.face(inner).adjacent_edges() {
            if de.is_constraint_edge() {
                continue;
            }
            let nb = de.rev().face().fix();
            if nb.as_inner().is_some() && !inside[nb.index()] {
                inside[nb.index()] = true;
                stack.push(nb);
            }
        }
    }
    // Both sides of a boundary edge must agree with its direction.
    for &(a, b) in &cap_edges {
        let de = cdt.get_edge_from_neighbors(a, b).ok_or("no edge2")?;
        let right = de.rev().face().fix();
        if right.as_inner().is_some() && inside[right.index()] {
            return Err("right inside");
        }
    }
    let mut id_of = vec![u32::MAX; cdt.num_vertices()];
    for (h, &id) in handles.iter().zip(&ids) {
        id_of[h.index()] = id;
    }
    let mut tris = Vec::new();
    for f in cdt.inner_faces() {
        if !inside[f.fix().index()] {
            continue;
        }
        let vs = f.vertices();
        let mut t = [0u32; 3];
        for (k, v) in vs.iter().enumerate() {
            t[k] = *id_of.get(v.fix().index()).ok_or("idof")?;
        }
        tris.push(t);
    }
    Ok(tris)
}
