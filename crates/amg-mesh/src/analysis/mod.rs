//! Mesh analysis (MESH-02): shells, boundary loops, non-manifold edges and vertices,
//! orientation, degenerate and duplicate faces, area and volume.
//!
//! # Algorithm
//! One pass over the faces counts, per vertex, the incident live faces (a face with a repeated
//! vertex index is *index-degenerate*; it is reported but takes no part in the topology) and
//! unions the face's vertices in a lock-free union-find (shells). A second pass fills a CSR
//! vertex to corner table. The third pass visits every vertex in parallel: it classifies each
//! edge `(v, w)` with `v < w` by its incident-face count and direction (boundary, manifold,
//! non-manifold, orientation conflict), decides whether the vertex's faces form a single fan,
//! and finds duplicate faces at their smallest vertex. A last pass gathers the shell
//! statistics. Every reduction runs over fixed chunks, so results do not depend on the number
//! of threads.
//!
//! Shells are connected components over *vertices* (faces that only touch at a vertex belong
//! to the same shell), numbered by their smallest vertex index. This is what the corpus
//! manifest's `multiple_shells` flag and the synthetic ground truth use.

mod unwelded;

use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

use amg_core::Aabb3;
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::error::{MeshError, MeshResult};
use crate::indexed::IndexedMesh;
use crate::{BvhOpts, FaceId, INVALID, MeshSpatial, SelfIntersectOpts, self_intersections};

/// Faces per work item. Fixed: results never depend on the thread count.
const FACE_CHUNK: usize = 1 << 14;
/// Vertices per work item.
const VERT_CHUNK: usize = 1 << 12;

/// Options for [`analyze`].
#[derive(Clone, Debug)]
pub struct AnalyzeOpts {
    /// Faces with an area at or below this are degenerate, mm².
    pub degenerate_area: f64,
    /// Faces with an area at or below `degenerate_relative * diagonal²` are degenerate, where
    /// `diagonal` is the bounding-box diagonal. The effective threshold is the larger of the two.
    pub degenerate_relative: f64,
    /// Detect self-intersections with [`crate::self_intersections`] (MESH-19). This builds a
    /// [`crate::MeshSpatial`] over a copy of the mesh; callers that already hold one should call
    /// `self_intersections` on it directly instead.
    pub self_intersections: bool,
    /// Merge distance, relative to the bounding-box diagonal, for finding seams that only
    /// welding would close
    /// ([`AnalysisReport::unwelded_boundary_edges`]). `0` disables the check.
    pub unwelded_tolerance: f64,
}

impl Default for AnalyzeOpts {
    fn default() -> Self {
        Self {
            degenerate_area: 1e-12,
            degenerate_relative: 1e-12,
            self_intersections: false,
            unwelded_tolerance: 1e-4,
        }
    }
}

/// One shell (connected component).
#[derive(Clone, Debug, PartialEq)]
pub struct ShellInfo {
    /// Faces in the shell.
    pub faces: usize,
    /// Surface area, mm².
    pub area: f64,
    /// Signed volume, mm³ (meaningful if `closed` and oriented).
    pub signed_volume: f64,
    /// No boundary edge and no non-manifold edge.
    pub closed: bool,
    /// Bounding box of the shell's vertices.
    pub aabb: Aabb3,
}

/// Result of [`analyze`].
#[derive(Clone, Debug, PartialEq)]
pub struct AnalysisReport {
    /// Vertex count.
    pub vertices: usize,
    /// Face count (including degenerate faces).
    pub faces: usize,
    /// Vertices that no face uses.
    pub unreferenced_vertices: usize,
    /// Shells, numbered by their smallest vertex index.
    pub shells: Vec<ShellInfo>,
    /// Shell of every face; [`INVALID`] for faces with a repeated vertex index.
    pub shell_of_face: Vec<u32>,
    /// Boundary loops as vertex sequences (each loop follows its faces' winding). Sorted by
    /// the smallest vertex. A pinched boundary is split greedily; an unclosable chain is
    /// returned as it is.
    pub boundary_loops: Vec<Vec<u32>>,
    /// Number of edges that belong to exactly one face.
    pub boundary_edges: usize,
    /// Loops in `boundary_loops` that do not close (only on non-manifold or inconsistently
    /// oriented input); such a chain lists `edges + 1` vertices.
    pub unclosed_boundary_loops: usize,
    /// Edges `[lo, hi]` with more than two faces, sorted.
    pub non_manifold_edges: Vec<[u32; 2]>,
    /// Vertices whose faces form more than one fan, sorted.
    pub non_manifold_vertices: Vec<u32>,
    /// Edges with exactly two faces that traverse it in the same direction.
    pub orientation_conflicts: usize,
    /// Faces with a repeated vertex index or an area below the threshold, sorted.
    pub degenerate_faces: Vec<FaceId>,
    /// Faces whose vertex set equals that of an earlier face, sorted.
    pub duplicate_faces: Vec<FaceId>,
    /// Pairs of intersecting faces; `None` unless requested (MESH-19).
    pub self_intersecting_pairs: Option<Vec<[FaceId; 2]>>,
    /// Boundary edges whose endpoints lie within `unwelded_tolerance * diagonal` of another
    /// edge's endpoints (and that are longer than that distance): they would pair up
    /// if the mesh were welded. Non-zero marks an unwelded seam.
    pub unwelded_boundary_edges: usize,
    /// Total area, mm².
    pub area: f64,
    /// Total signed volume; `Some` only if the mesh is watertight.
    pub volume: Option<f64>,
    /// Bounding box of all vertices.
    pub aabb: Aabb3,
    /// Has faces and no boundary edge.
    pub is_closed: bool,
    /// No non-manifold edge or vertex.
    pub is_manifold: bool,
    /// No orientation conflict.
    pub is_oriented: bool,
    /// Watertight with negative volume (faces point inward).
    pub is_inverted: bool,
}

impl AnalysisReport {
    /// Closed, manifold and consistently oriented.
    pub fn is_watertight(&self) -> bool {
        self.is_closed && self.is_manifold && self.is_oriented
    }
}

/// Per-shell accumulator.
#[derive(Clone, Copy)]
struct Acc {
    faces: usize,
    area: f64,
    vol6: f64,
    aabb: Aabb3,
}

impl Acc {
    const ZERO: Acc = Acc {
        faces: 0,
        area: 0.0,
        vol6: 0.0,
        aabb: Aabb3::EMPTY,
    };
    fn merge(&mut self, o: &Acc) {
        self.faces += o.faces;
        self.area += o.area;
        self.vol6 += o.vol6;
        self.aabb = self.aabb.union(&o.aabb);
    }
}

#[derive(Default)]
struct IdHasher(u64);
impl Hasher for IdHasher {
    fn finish(&self) -> u64 {
        self.0
    }
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = (self.0 ^ u64::from(b)).wrapping_mul(0x100_0000_01b3);
        }
    }
    fn write_u32(&mut self, x: u32) {
        self.0 = u64::from(x).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }
}
type IdMap = HashMap<u32, usize, BuildHasherDefault<IdHasher>>;

#[inline]
fn is_index_degenerate(t: &[u32; 3]) -> bool {
    t[0] == t[1] || t[1] == t[2] || t[0] == t[2]
}

fn find(parent: &[AtomicU32], mut x: u32) -> u32 {
    loop {
        let p = parent[x as usize].load(Ordering::Relaxed);
        if p == x {
            return x;
        }
        let g = parent[p as usize].load(Ordering::Relaxed);
        // Path halving; losing the race only skips a shortcut.
        let _ =
            parent[x as usize].compare_exchange_weak(p, g, Ordering::Relaxed, Ordering::Relaxed);
        x = g;
    }
}

/// Links the larger root under the smaller, so the root is always the smallest member.
fn union(parent: &[AtomicU32], a: u32, b: u32) {
    let (mut a, mut b) = (find(parent, a), find(parent, b));
    while a != b {
        if a < b {
            std::mem::swap(&mut a, &mut b);
        }
        // a > b: hang a below b.
        match parent[a as usize].compare_exchange(a, b, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => return,
            Err(_) => {
                a = find(parent, a);
                b = find(parent, b);
            }
        }
    }
}

fn cancelled(ctx: &JobCtx) -> MeshResult<()> {
    ctx.checkpoint().map_err(|_| MeshError::Cancelled)
}

/// Output of the per-vertex pass for one chunk of vertices.
#[derive(Default)]
struct VertOut {
    /// Directed boundary edges `(from, to, face)`.
    boundary: Vec<(u32, u32, u32)>,
    nm_edges: Vec<([u32; 2], u32)>,
    nm_verts: Vec<u32>,
    dups: Vec<u32>,
    conflicts: usize,
}

/// Reusable buffers of the per-vertex pass.
#[derive(Default)]
struct Scratch {
    link: Vec<(u32, u32, u32)>,
    edges: Vec<(u32, u32, bool)>,
    keys: Vec<(u32, u32, u32)>,
    labels: Vec<u32>,
    dsu: Vec<u32>,
}

fn dsu_find(d: &mut [u32], mut x: u32) -> u32 {
    while d[x as usize] != x {
        d[x as usize] = d[d[x as usize] as usize];
        x = d[x as usize];
    }
    x
}

/// True if the faces in `s.link` (pairs of neighbour vertices) form one fan.
fn single_fan(s: &mut Scratch) -> bool {
    if s.link.len() < 2 {
        return true;
    }
    s.labels.clear();
    for &(a, b, _) in &s.link {
        s.labels.push(a);
        s.labels.push(b);
    }
    s.labels.sort_unstable();
    s.labels.dedup();
    s.dsu.clear();
    s.dsu.extend(0..s.labels.len() as u32);
    let mut comps = s.labels.len();
    for i in 0..s.link.len() {
        let (a, b, _) = s.link[i];
        let (Ok(ia), Ok(ib)) = (s.labels.binary_search(&a), s.labels.binary_search(&b)) else {
            continue;
        };
        let (ra, rb) = (
            dsu_find(&mut s.dsu, ia as u32),
            dsu_find(&mut s.dsu, ib as u32),
        );
        if ra != rb {
            s.dsu[ra as usize] = rb;
            comps -= 1;
        }
    }
    comps == 1
}

fn process_vertex(
    m: &IndexedMesh,
    v: u32,
    corners: &[AtomicU32],
    s: &mut Scratch,
    out: &mut VertOut,
) {
    s.link.clear();
    s.edges.clear();
    s.keys.clear();
    for c in corners {
        let c = c.load(Ordering::Relaxed);
        let (f, k) = (c / 3, (c % 3) as usize);
        let t = &m.tris[f as usize];
        let (a, b) = (t[(k + 1) % 3], t[(k + 2) % 3]);
        s.link.push((a, b, f));
        // The face runs v -> a -> b -> v: edge {v,a} leaves v, edge {v,b} enters v.
        if a > v {
            s.edges.push((a, f, true));
        }
        if b > v {
            s.edges.push((b, f, false));
        }
        if a > v && b > v {
            s.keys.push((a.min(b), a.max(b), f));
        }
    }
    if !single_fan(s) {
        out.nm_verts.push(v);
    }
    s.edges.sort_unstable();
    let mut i = 0;
    while i < s.edges.len() {
        let w = s.edges[i].0;
        let mut j = i + 1;
        while j < s.edges.len() && s.edges[j].0 == w {
            j += 1;
        }
        match j - i {
            1 => {
                let (_, f, from_lo) = s.edges[i];
                out.boundary
                    .push(if from_lo { (v, w, f) } else { (w, v, f) });
            }
            2 => {
                if s.edges[i].2 == s.edges[i + 1].2 {
                    out.conflicts += 1;
                }
            }
            _ => out.nm_edges.push(([v, w], s.edges[i].1)),
        }
        i = j;
    }
    s.keys.sort_unstable();
    let mut i = 0;
    while i < s.keys.len() {
        let mut j = i + 1;
        while j < s.keys.len() && s.keys[j].0 == s.keys[i].0 && s.keys[j].1 == s.keys[i].1 {
            out.dups.push(s.keys[j].2);
            j += 1;
        }
        i = j;
    }
}

/// Analyses `m`. See the module documentation for what is reported and how.
///
/// # Errors
/// [`MeshError::IndexOutOfRange`] for a face that references a missing vertex,
/// [`MeshError::TooLarge`] beyond `u32` corner indices, and [`MeshError::Cancelled`].
pub fn analyze(m: &IndexedMesh, opts: &AnalyzeOpts, ctx: &JobCtx) -> MeshResult<AnalysisReport> {
    let (nv, nf) = (m.positions.len(), m.tris.len());
    if nf.checked_mul(3).is_none_or(|n| n > u32::MAX as usize) || nv > u32::MAX as usize {
        return Err(MeshError::TooLarge { count: nf.max(nv) });
    }
    cancelled(ctx)?;
    let aabb = m.aabb();
    let diag = if nv == 0 { 0.0 } else { aabb.diagonal() };
    let area_eps = opts
        .degenerate_area
        .max(opts.degenerate_relative * diag * diag);

    // Pass 1: range check, degenerate faces, per-vertex live-face counts, vertex unions.
    let counts: Vec<AtomicU32> = (0..nv).into_par_iter().map(|_| AtomicU32::new(0)).collect();
    let parent: Vec<AtomicU32> = (0..nv as u32).into_par_iter().map(AtomicU32::new).collect();
    let p1: Vec<(Vec<u32>, Vec<u32>)> = m
        .tris
        .par_chunks(FACE_CHUNK)
        .enumerate()
        .map(|(ci, chunk)| -> MeshResult<(Vec<u32>, Vec<u32>)> {
            let (mut degen, mut idx_verts) = (Vec::new(), Vec::new());
            for (i, t) in chunk.iter().enumerate() {
                let f = (ci * FACE_CHUNK + i) as u32;
                for &x in t {
                    if x as usize >= nv {
                        return Err(MeshError::IndexOutOfRange { index: x, len: nv });
                    }
                }
                if is_index_degenerate(t) {
                    degen.push(f);
                    idx_verts.extend_from_slice(t);
                    continue;
                }
                let [a, b, c] = t.map(|x| m.positions[x as usize]);
                if 0.5 * (b - a).cross(c - a).length() <= area_eps {
                    degen.push(f);
                }
                for &x in t {
                    counts[x as usize].fetch_add(1, Ordering::Relaxed);
                }
                union(&parent, t[0], t[1]);
                union(&parent, t[0], t[2]);
            }
            Ok((degen, idx_verts))
        })
        .collect::<MeshResult<_>>()?;
    let degenerate_faces: Vec<FaceId> = p1
        .iter()
        .flat_map(|(d, _)| d.iter().map(|&f| FaceId(f)))
        .collect();
    let mut idx_verts: Vec<u32> = p1.into_iter().flat_map(|(_, v)| v).collect();
    idx_verts.sort_unstable();
    idx_verts.dedup();
    // Collapsed faces (repeated index) are not in the topology tables; their duplicates are
    // found here. A collapsed face never equals a live face's vertex set.
    let mut collapsed: Vec<([u32; 3], u32)> = degenerate_faces
        .iter()
        .filter(|f| is_index_degenerate(&m.tris[f.0 as usize]))
        .map(|f| {
            let mut k = m.tris[f.0 as usize];
            k.sort_unstable();
            (k, f.0)
        })
        .collect();
    collapsed.sort_unstable();
    let collapsed_dups: Vec<u32> = collapsed
        .windows(2)
        .filter(|w| w[0].0 == w[1].0)
        .map(|w| w[1].1)
        .collect();
    cancelled(ctx)?;

    // Pass 2: CSR vertex -> corners. `counts` becomes the fill cursor.
    let mut offsets: Vec<u32> = Vec::with_capacity(nv + 1);
    let mut total = 0u32;
    for c in &counts {
        offsets.push(total);
        total += c.load(Ordering::Relaxed);
    }
    offsets.push(total);
    counts
        .par_iter()
        .zip(offsets.par_iter())
        .for_each(|(c, &o)| c.store(o, Ordering::Relaxed));
    let corners: Vec<AtomicU32> = (0..total as usize)
        .into_par_iter()
        .map(|_| AtomicU32::new(0))
        .collect();
    m.tris
        .par_chunks(FACE_CHUNK)
        .enumerate()
        .for_each(|(ci, chunk)| {
            for (i, t) in chunk.iter().enumerate() {
                if is_index_degenerate(t) {
                    continue;
                }
                let f = (ci * FACE_CHUNK + i) as u32;
                for (k, &x) in t.iter().enumerate() {
                    let slot = counts[x as usize].fetch_add(1, Ordering::Relaxed);
                    corners[slot as usize].store(3 * f + k as u32, Ordering::Relaxed);
                }
            }
        });
    drop(counts);
    cancelled(ctx)?;

    // Pass 3: per-vertex topology.
    let outs: Vec<VertOut> = (0..nv.div_ceil(VERT_CHUNK))
        .into_par_iter()
        .map(|ci| {
            let mut out = VertOut::default();
            let mut s = Scratch::default();
            let lo = ci * VERT_CHUNK;
            for v in lo..(lo + VERT_CHUNK).min(nv) {
                let (a, b) = (offsets[v] as usize, offsets[v + 1] as usize);
                if a < b {
                    process_vertex(m, v as u32, &corners[a..b], &mut s, &mut out);
                }
            }
            out
        })
        .collect();
    drop(corners);
    cancelled(ctx)?;

    let mut boundary: Vec<(u32, u32, u32)> = Vec::new();
    let mut nm_edges_f: Vec<([u32; 2], u32)> = Vec::new();
    let mut non_manifold_vertices = Vec::new();
    let mut dups: Vec<u32> = collapsed_dups;
    let mut orientation_conflicts = 0usize;
    for o in outs {
        if boundary.is_empty() {
            boundary = o.boundary;
        } else {
            boundary.extend(o.boundary);
        }
        nm_edges_f.extend(o.nm_edges);
        non_manifold_vertices.extend(o.nm_verts);
        dups.extend(o.dups);
        orientation_conflicts += o.conflicts;
    }
    dups.sort_unstable();
    dups.dedup();
    let duplicate_faces: Vec<FaceId> = dups.into_iter().map(FaceId).collect();
    let non_manifold_edges: Vec<[u32; 2]> = nm_edges_f.iter().map(|e| e.0).collect();

    // Shell numbering: roots are the smallest vertex of each component.
    (0..nv).into_par_iter().for_each(|v| {
        let r = find(&parent, v as u32);
        parent[v].store(r, Ordering::Relaxed);
    });
    let mut shell_id = vec![INVALID; nv];
    let mut n_shells = 0u32;
    for v in 0..nv {
        if offsets[v] < offsets[v + 1] && parent[v].load(Ordering::Relaxed) == v as u32 {
            shell_id[v] = n_shells;
            n_shells += 1;
        }
    }
    let unreferenced_vertices = (0..nv)
        .into_par_iter()
        .filter(|&v| offsets[v] == offsets[v + 1] && idx_verts.binary_search(&(v as u32)).is_err())
        .count();

    // Pass 4: shell of every face, per-shell statistics.
    let origin = m.positions.first().copied().unwrap_or_default();
    let mut shell_of_face = vec![INVALID; nf];
    let chunk_accs: Vec<Vec<(u32, Acc)>> = shell_of_face
        .par_chunks_mut(FACE_CHUNK)
        .zip(m.tris.par_chunks(FACE_CHUNK))
        .map(|(out, tris)| {
            let mut local: Vec<(u32, Acc)> = Vec::new();
            let mut index: Option<IdMap> = None;
            let mut last = usize::MAX;
            for (o, t) in out.iter_mut().zip(tris) {
                if is_index_degenerate(t) {
                    continue;
                }
                let s = shell_id[parent[t[0] as usize].load(Ordering::Relaxed) as usize];
                *o = s;
                let slot = if last != usize::MAX && local[last].0 == s {
                    last
                } else if let Some(map) = index.as_mut() {
                    *map.entry(s).or_insert_with(|| {
                        local.push((s, Acc::ZERO));
                        local.len() - 1
                    })
                } else if let Some(p) = local.iter().position(|e| e.0 == s) {
                    p
                } else {
                    local.push((s, Acc::ZERO));
                    if local.len() > 8 {
                        index = Some(local.iter().enumerate().map(|(i, e)| (e.0, i)).collect());
                    }
                    local.len() - 1
                };
                last = slot;
                let [a, b, c] = t.map(|x| m.positions[x as usize]);
                let acc = &mut local[slot].1;
                acc.faces += 1;
                acc.area += 0.5 * (b - a).cross(c - a).length();
                acc.vol6 += (a - origin).dot((b - origin).cross(c - origin));
                acc.aabb.include_point(a);
                acc.aabb.include_point(b);
                acc.aabb.include_point(c);
            }
            local
        })
        .collect();
    let mut accs = vec![Acc::ZERO; n_shells as usize];
    for chunk in &chunk_accs {
        for (s, a) in chunk {
            accs[*s as usize].merge(a);
        }
    }
    drop(chunk_accs);
    let mut closed = vec![true; n_shells as usize];
    for &(_, _, f) in &boundary {
        closed[shell_of_face[f as usize] as usize] = false;
    }
    for &(_, f) in &nm_edges_f {
        closed[shell_of_face[f as usize] as usize] = false;
    }
    let shells: Vec<ShellInfo> = accs
        .iter()
        .zip(&closed)
        .map(|(a, &closed)| ShellInfo {
            faces: a.faces,
            area: a.area,
            signed_volume: a.vol6 / 6.0,
            closed,
            aabb: a.aabb,
        })
        .collect();
    let area: f64 = shells.iter().map(|s| s.area).sum();
    let total_volume: f64 = shells.iter().map(|s| s.signed_volume).sum();
    cancelled(ctx)?;

    // Boundary loops and unwelded seams (both only touch boundary edges).
    let unwelded_boundary_edges = if opts.unwelded_tolerance > 0.0 && !boundary.is_empty() {
        unwelded::count(m, &boundary, opts.unwelded_tolerance * diag)
    } else {
        0
    };
    let boundary_edges = boundary.len();
    let (boundary_loops, unclosed_boundary_loops) = trace_loops(&mut boundary);

    let is_closed = !shells.is_empty() && boundary_loops.is_empty() && closed.iter().all(|&c| c);
    let is_manifold = non_manifold_edges.is_empty() && non_manifold_vertices.is_empty();
    let is_oriented = orientation_conflicts == 0;
    let watertight = is_closed && is_manifold && is_oriented;
    // Pass 1 has checked every index, so the spatial index can be built safely.
    let self_intersecting_pairs = if opts.self_intersections {
        let s = MeshSpatial::build(Arc::new(m.clone()), &BvhOpts::default(), ctx)?;
        Some(self_intersections(&s, &SelfIntersectOpts::default(), ctx)?)
    } else {
        None
    };

    Ok(AnalysisReport {
        vertices: nv,
        faces: nf,
        unreferenced_vertices,
        shells,
        shell_of_face,
        boundary_loops,
        boundary_edges,
        unclosed_boundary_loops,
        non_manifold_edges,
        non_manifold_vertices,
        orientation_conflicts,
        degenerate_faces,
        duplicate_faces,
        self_intersecting_pairs,
        unwelded_boundary_edges,
        area,
        volume: watertight.then_some(total_volume),
        aabb,
        is_closed,
        is_manifold,
        is_oriented,
        is_inverted: watertight && total_volume < 0.0,
    })
}

/// Chains directed boundary edges into vertex loops.
fn trace_loops(boundary: &mut [(u32, u32, u32)]) -> (Vec<Vec<u32>>, usize) {
    boundary.par_sort_unstable();
    let mut used = vec![false; boundary.len()];
    let mut loops = Vec::new();
    let mut unclosed = 0;
    for start in 0..boundary.len() {
        if used[start] {
            continue;
        }
        let u0 = boundary[start].0;
        let mut lp = vec![u0];
        let mut cur = start;
        loop {
            used[cur] = true;
            let v = boundary[cur].1;
            if v == u0 {
                break;
            }
            lp.push(v);
            let first = boundary.partition_point(|e| e.0 < v);
            match (first..boundary.len())
                .take_while(|&i| boundary[i].0 == v)
                .find(|&i| !used[i])
            {
                Some(n) => cur = n,
                None => {
                    unclosed += 1;
                    break;
                }
            }
        }
        loops.push(lp);
    }
    (loops, unclosed)
}

#[cfg(test)]
mod tests;
