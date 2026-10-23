//! Hole filling (MESH-20): Liepa 2003, "Filling holes in meshes".
//!
//! Per boundary loop:
//! 1. **Triangulate** the loop polygon: a constrained Delaunay triangulation when the loop is
//!    planar (`spade`; the usual cap of a plane cut), otherwise the minimum-weight dynamic
//!    program over (largest dihedral angle, total area) for loops of up to
//!    [`HoleFillOpts::max_dp_edges`] edges, and ear clipping in the best-fit plane beyond that.
//!    A fan from a new centre vertex is the last resort, so every loop can be filled.
//! 2. **Refine** (optional): split triangles at their centroid and flip edges until the patch
//!    edge length follows the surrounding mesh ([`HoleFillOpts::density_factor`]).
//! 3. **Fair** the new vertices (harmonic or bi-harmonic, see [`Fairing`]). A planar loop stays
//!    a flat cap: bi-harmonic fairing would bulge it to match the walls' tangents.
//!
//! Chords that already exist in the surrounding mesh are never created, so the result has no
//! non-manifold edge. All new faces get one new face group of origin `Fill`.
//!
//! The patch is built on its own and only then added to the [`HalfEdgeMesh`]; the pass is
//! sequential and uses no hash-order-dependent decisions, so the output does not depend on the
//! rayon thread count.

mod fair;
mod refine;
mod triangulate;

use std::collections::{HashMap, HashSet};

use amg_core::{Point3, Vec3};
use amg_jobs::JobCtx;

use crate::{
    AnalyzeOpts, FaceGroupId, FaceGroupInfo, GroupOrigin, HalfEdgeId, HalfEdgeMesh, IndexedMesh,
    MeshError, MeshResult, NonManifoldPolicy, VertexId, analyze,
};
use fair::Ring;
use refine::Patch;

/// How the refined vertices of a patch are placed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fairing {
    /// Leave the patch vertices where triangulation and refinement put them.
    None,
    /// Minimise the umbrella operator on the new vertices (a membrane: continuous with the
    /// surrounding surface, flatter than it).
    Harmonic,
    /// Minimise the umbrella operator on the new vertices and on the loop (a thin plate:
    /// tangent-continuous with the surrounding surface).
    Biharmonic,
}

/// Options for [`fill_holes`].
#[derive(Clone, Debug)]
pub struct HoleFillOpts {
    /// Loops with more edges are left open and counted in
    /// [`HoleFillReport::skipped_too_large`]. Default 2000.
    pub max_boundary_edges: usize,
    /// Loops up to this many edges use the O(n^3) dynamic program, longer ones ear clipping.
    /// Default 250.
    pub max_dp_edges: usize,
    /// Add interior vertices so the patch matches the surrounding edge length. Fairing only
    /// moves such new vertices, so without refinement it has nothing to do.
    pub refine: bool,
    /// Liepa's alpha: larger gives a denser patch. Default sqrt 2.
    pub density_factor: f64,
    /// Fairing of the refined vertices (not applied to planar loops, whose cap stays flat). Large patches (over 60 000 new vertices) are downgraded
    /// to harmonic, and over 600 000 are left unfaired.
    pub fairing: Fairing,
}

impl Default for HoleFillOpts {
    fn default() -> Self {
        HoleFillOpts {
            max_boundary_edges: 2000,
            max_dp_edges: 250,
            refine: true,
            density_factor: std::f64::consts::SQRT_2,
            fairing: Fairing::Biharmonic,
        }
    }
}

/// What [`fill_holes`] did.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct HoleFillReport {
    /// Loops filled.
    pub filled: usize,
    /// Loops left open because they exceed [`HoleFillOpts::max_boundary_edges`].
    pub skipped_too_large: usize,
    /// Loops the half-edge structure refused (rolled back, mesh unchanged for that loop). Not
    /// expected; counted so a caller can report it.
    pub failed: usize,
    /// Faces added.
    pub new_faces: usize,
    /// Face group of the added faces (`None` if nothing was filled).
    pub new_group: Option<FaceGroupId>,
}

fn validate(opts: &HoleFillOpts) -> MeshResult<()> {
    if !(opts.density_factor > 0.0 && opts.density_factor.is_finite()) {
        return Err(MeshError::InvalidOption {
            name: "density_factor",
            reason: format!("must be positive and finite, got {}", opts.density_factor),
        });
    }
    Ok(())
}

/// Fills the boundary loops of `m` (every loop with at most
/// [`HoleFillOpts::max_boundary_edges`] edges). The result is manifold where it was, and every
/// filled loop becomes interior. Cancellable between loops and inside long steps.
pub fn fill_holes(
    m: &mut HalfEdgeMesh,
    opts: &HoleFillOpts,
    ctx: &JobCtx,
) -> MeshResult<HoleFillReport> {
    fill_holes_where(m, opts, ctx, &|_, _| true)
}

/// [`fill_holes`] restricted to the loops for which `pick` returns true (MESH-21 fills only
/// the holes it cut itself and leaves pre-existing ones alone).
pub(crate) fn fill_holes_where(
    m: &mut HalfEdgeMesh,
    opts: &HoleFillOpts,
    ctx: &JobCtx,
    pick: &dyn Fn(&HalfEdgeMesh, &[HalfEdgeId]) -> bool,
) -> MeshResult<HoleFillReport> {
    validate(opts)?;
    let loops: Vec<Vec<HalfEdgeId>> = m
        .boundary_loops()
        .into_iter()
        .filter(|l| pick(m, l))
        .collect();
    let mut report = HoleFillReport::default();
    let count = loops.len().max(1) as f32;
    for (i, lp) in loops.iter().enumerate() {
        ctx.checkpoint()?;
        ctx.progress(i as f32 / count);
        if lp.len() > opts.max_boundary_edges {
            report.skipped_too_large += 1;
            continue;
        }
        if lp.len() < 3 {
            report.failed += 1;
            continue;
        }
        match fill_loop(m, lp, opts, &mut report.new_group, ctx)? {
            Some(faces) => {
                report.filled += 1;
                report.new_faces += faces;
            }
            None => report.failed += 1,
        }
    }
    ctx.progress(1.0);
    Ok(report)
}

/// [`fill_holes`] on an [`IndexedMesh`]. Non-manifold vertices are split first (as
/// [`HalfEdgeMesh::from_indexed`] does with [`NonManifoldPolicy::SplitVertices`]); a mesh
/// without boundary loops is returned unchanged.
pub fn fill_holes_indexed(
    m: &IndexedMesh,
    opts: &HoleFillOpts,
    ctx: &JobCtx,
) -> MeshResult<(IndexedMesh, HoleFillReport)> {
    validate(opts)?;
    let rep = analyze(m, &AnalyzeOpts::default(), ctx)?;
    if rep.boundary_loops.is_empty() {
        return Ok((m.clone(), HoleFillReport::default()));
    }
    let (mut he, _) = HalfEdgeMesh::from_indexed(m, NonManifoldPolicy::SplitVertices)?;
    let report = fill_holes(&mut he, opts, ctx)?;
    Ok((he.to_indexed(), report))
}

fn unit_or_zero(v: Vec3) -> Vec3 {
    let l = v.length();
    if l > 0.0 && l.is_finite() {
        v / l
    } else {
        Vec3::ZERO
    }
}

/// Fills one loop; `Ok(None)` when the half-edge structure rejected the patch.
fn fill_loop(
    he: &mut HalfEdgeMesh,
    lp: &[HalfEdgeId],
    opts: &HoleFillOpts,
    group: &mut Option<FaceGroupId>,
    ctx: &JobCtx,
) -> MeshResult<Option<usize>> {
    let n = lp.len();
    // Polygon order: reverse of the loop, so new triangles run against the boundary edges.
    let q: Vec<VertexId> = (0..n).map(|k| he.origin(lp[n - 1 - k])).collect();
    let pos: Vec<Point3> = q.iter().map(|&v| he.position(v)).collect();
    let outer: Vec<Vec3> = (0..n)
        .map(|k| {
            let h = lp[(n + n - 2 - k) % n];
            let [a, b, c] = he.face_vertices(he.face(h)).map(|v| he.position(v));
            unit_or_zero((b - a).cross(c - a))
        })
        .collect();
    let local: HashMap<u32, usize> = q.iter().enumerate().map(|(i, v)| (v.0, i)).collect();
    let mut sigma = Vec::with_capacity(n);
    let mut rings = vec![Ring::default(); n];
    let mut existing: HashSet<(u32, u32)> = HashSet::new();
    for (i, &v) in q.iter().enumerate() {
        let nbs = he.neighbors(v);
        let mut sum = 0.0;
        for w in &nbs {
            let pw = he.position(*w);
            sum += (pw - pos[i]).length();
            match local.get(&w.0) {
                Some(&j) => {
                    rings[i].loop_nb.push(j as u32);
                    existing.insert((i.min(j) as u32, i.max(j) as u32));
                }
                None => rings[i].fixed.push(pw),
            }
        }
        sigma.push(if nbs.is_empty() {
            0.0
        } else {
            sum / nbs.len() as f64
        });
    }
    let forbidden = |a: usize, b: usize| existing.contains(&(a.min(b) as u32, a.max(b) as u32));

    let (mut patch, planar) = triangulate_loop(&pos, &outer, &sigma, &forbidden, opts, ctx)?;
    if opts.refine {
        let cap = (4 * n * n + 1000).min(2_000_000);
        refine::refine(&mut patch, opts.density_factor, cap, &forbidden, ctx)?;
    }
    ctx.checkpoint()?;
    if !planar {
        fair::fair(&mut patch, &rings, opts.fairing);
    }
    commit(he, &q, &patch, group)
}

/// Picks the triangulation method and falls back down the chain until one is valid.
fn triangulate_loop(
    pos: &[Point3],
    outer: &[Vec3],
    sigma: &[f64],
    forbidden: &dyn Fn(usize, usize) -> bool,
    opts: &HoleFillOpts,
    ctx: &JobCtx,
) -> MeshResult<(Patch, bool)> {
    let n = pos.len();
    let mut planar = false;
    let valid = |t: &Vec<[u32; 3]>| triangulate::is_valid_patch(t, n, 0);
    let mut tris: Option<Vec<[u32; 3]>> = None;
    if n == 3 {
        tris = Some(vec![[0, 1, 2]]);
    }
    if tris.is_none()
        && let Some(frame) = triangulate::planar_frame(pos, 1e-5)
    {
        tris = triangulate::cdt_triangulate(pos, frame, forbidden).filter(valid);
        planar = tris.is_some();
    }
    if tris.is_none() && n <= opts.max_dp_edges {
        tris = triangulate::dp_triangulate(pos, outer, forbidden, ctx)?.filter(valid);
    }
    if tris.is_none() {
        let nrm = triangulate::newell(pos);
        if nrm.length() > 0.0 {
            let c = pos.iter().copied().sum::<Point3>() / n as f64;
            tris = triangulate::ear_clip(pos, (c, nrm.normalize()), forbidden).filter(valid);
        }
    }
    let mut patch = Patch {
        pos: pos.to_vec(),
        tris: Vec::new(),
        sigma: sigma.to_vec(),
        n_loop: n,
    };
    patch.tris = match tris {
        Some(t) => t,
        None => {
            patch
                .pos
                .push(pos.iter().copied().sum::<Point3>() / n as f64);
            patch.sigma.push(sigma.iter().sum::<f64>() / n as f64);
            triangulate::centroid_fan(n)
        }
    };
    Ok((patch, planar))
}

/// Adds the patch to the mesh. Triangles are added from the old boundary inwards, a deferred
/// one retried once its neighbours exist. On failure everything is rolled back.
fn commit(
    he: &mut HalfEdgeMesh,
    q: &[VertexId],
    patch: &Patch,
    group: &mut Option<FaceGroupId>,
) -> MeshResult<Option<usize>> {
    let n = patch.n_loop;
    let g = match *group {
        Some(g) => g,
        None => {
            let g = he
                .groups_mut()
                .try_alloc(FaceGroupInfo::new(GroupOrigin::Fill))?;
            *group = Some(g);
            g
        }
    };
    let mut ids: Vec<VertexId> = q.to_vec();
    for p in &patch.pos[n..] {
        ids.push(he.add_vertex(*p));
    }
    let new_verts = ids[n..].to_vec();
    // Breadth-first order from the loop edges.
    let order = peel_order(&patch.tris, n, patch.pos.len());
    let mut pending = order;
    let mut added = Vec::with_capacity(pending.len());
    loop {
        let before = pending.len();
        let mut deferred = Vec::new();
        for &t in &pending {
            let v = patch.tris[t];
            match he.add_face(v.map(|i| ids[i as usize]), g) {
                Ok(f) => added.push(f),
                Err(_) => deferred.push(t),
            }
        }
        pending = deferred;
        if pending.is_empty() {
            return Ok(Some(added.len()));
        }
        if pending.len() == before {
            break;
        }
    }
    for f in added.into_iter().rev() {
        he.delete_face(f);
    }
    for v in new_verts {
        he.delete_vertex(v);
    }
    Ok(None)
}

/// An order in which the triangles can be added to the half-edge mesh: the patch disc is peeled
/// from the loop inwards, each time removing a triangle that leaves a disc (one boundary edge
/// and an apex off the boundary, or two or three boundary edges). Adding in that order keeps
/// the unfilled rest a disc, so every `add_face` attaches to a boundary edge without creating a
/// second fan at a vertex. (Breadth-first order is not enough: fans can meet in the wrong order
/// and deadlock.)
fn peel_order(tris: &[[u32; 3]], n: usize, vertices: usize) -> Vec<usize> {
    let mut edge: HashMap<(u32, u32), usize> = HashMap::with_capacity(tris.len() * 3);
    for (t, v) in tris.iter().enumerate() {
        for k in 0..3 {
            edge.insert((v[k], v[(k + 1) % 3]), t);
        }
    }
    let mut alive = vec![true; tris.len()];
    // Boundary edges of the remaining disc incident to each vertex.
    let mut bcount = vec![0u32; vertices];
    for i in 0..n {
        bcount[i] += 1;
        bcount[(i + 1) % n] += 1;
    }
    let is_boundary = |alive: &[bool], x: u32, y: u32| match edge.get(&(y, x)) {
        Some(&s) => !alive[s],
        None => true,
    };
    let eligible = |alive: &[bool], bcount: &[u32], t: usize| {
        let v = tris[t];
        let b: Vec<bool> = (0..3)
            .map(|k| is_boundary(alive, v[k], v[(k + 1) % 3]))
            .collect();
        match b.iter().filter(|x| **x).count() {
            0 => false,
            1 => {
                let k = b.iter().position(|x| *x).unwrap_or(0);
                bcount[v[(k + 2) % 3] as usize] == 0
            }
            _ => true,
        }
    };
    let mut stack: Vec<usize> = (0..tris.len())
        .rev()
        .filter(|&t| {
            let v = tris[t];
            (0..3).any(|k| is_boundary(&alive, v[k], v[(k + 1) % 3]))
        })
        .collect();
    let mut order = Vec::with_capacity(tris.len());
    while let Some(t) = stack.pop() {
        if !alive[t] || !eligible(&alive, &bcount, t) {
            continue;
        }
        let v = tris[t];
        alive[t] = false;
        order.push(t);
        for k in 0..3 {
            let (x, y) = (v[k], v[(k + 1) % 3]);
            match edge.get(&(y, x)) {
                Some(&s) if alive[s] => {
                    // The edge becomes a boundary edge.
                    bcount[x as usize] += 1;
                    bcount[y as usize] += 1;
                    stack.push(s);
                }
                _ => {
                    bcount[x as usize] -= 1;
                    bcount[y as usize] -= 1;
                }
            }
        }
    }
    // Anything left (not expected for a valid disc) still gets a turn.
    for (t, a) in alive.iter().enumerate() {
        if *a {
            order.push(t);
        }
    }
    order
}
