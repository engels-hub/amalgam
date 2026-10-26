//! Plane cut (MESH-09): exact plane-side classification, edge splitting, CDT caps, section
//! polylines, and the small mesh edits that go with them ([`separate_shells`],
//! [`erase_and_fill`], [`reverse_normals`]).
//!
//! # Plane cut
//! 1. Every vertex gets an exact side (`-1`, `0`, `+1`) from the sign of `n · (p − o)` (see
//!    `side.rs`); `snap_tolerance > 0` counts vertices within that distance as on the plane.
//!    Topology is decided from these signs only, so rounding of the new vertices never changes
//!    it and both halves share them: they are closed and manifold whenever the input is.
//! 2. A triangle with vertices strictly on both sides is clipped; each crossed edge gets one
//!    new vertex (shared by both neighbours and both halves). A vertex on the plane is its own
//!    cut vertex: no duplicate is created.
//! 3. Triangles that only touch the plane go whole to the side they lie on. A triangle lying
//!    in the plane goes to the negative half when its normal points to `+n` (the solid is
//!    below it) and to the positive half otherwise, so the halves never overlap.
//! 4. The in-plane boundary edges of each half are capped with a constrained Delaunay
//!    triangulation ([`cap`]) in one new [`GroupOrigin::Cap`] group shared by both halves.
//!    A half whose boundary does not close (open input) or cannot be triangulated stays open
//!    and is counted in [`PlaneCutResult::open_loops`].
//!
//! Attribute policy: vertex normals and colours are interpolated at new vertices, corner UVs
//! inside the parent face, 3MF properties are inherited (new corners take the property index
//! of the nearest parent corner); cap faces have zero UVs and no property. Output face order:
//! unchanged faces in input order, then the pieces of cut faces, then the cap.

mod cap;
mod misc;
mod side;

use std::sync::atomic::{AtomicBool, Ordering};

use amg_core::{Plane, Point2, Point3};
use amg_jobs::JobCtx;
use rayon::prelude::*;

use crate::attrs::TriProperty;
use crate::error::{MeshError, MeshResult};
use crate::groups::{FaceGroupInfo, FaceGroups, GroupOrigin};
use crate::{FaceGroupId, INVALID, IndexedMesh};
use side::PlaneSide;

pub use misc::{erase_and_fill, reverse_normals, separate_shells};

/// Which halves [`plane_cut`] returns.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeepSide {
    /// Only the side `plane.normal` points to.
    Positive,
    /// Only the other side.
    Negative,
    /// Both.
    Both,
}

/// Options for [`plane_cut`].
#[derive(Clone, Debug)]
pub struct PlaneCutOpts {
    /// The cutting plane; the positive side is where its normal points.
    pub plane: Plane,
    /// Which halves to build.
    pub keep: KeepSide,
    /// Close each half with a CDT cap in a new [`GroupOrigin::Cap`] group.
    pub cap: bool,
    /// Vertices within this distance (mm) of the plane count as on it; `0` classifies
    /// exactly.
    pub snap_tolerance: f64,
}

/// Result of [`plane_cut`].
#[derive(Clone, Debug)]
pub struct PlaneCutResult {
    /// The part on the positive side (`None` if empty or not requested).
    pub positive: Option<IndexedMesh>,
    /// The part on the negative side (`None` if empty or not requested).
    pub negative: Option<IndexedMesh>,
    /// The cap group, identical in both halves (`None` if no cap face was made).
    pub cap_group: Option<FaceGroupId>,
    /// Section loops left uncapped: chains that do not close (open input) plus all loops of a
    /// half whose cap could not be triangulated. `0` when `cap` is off.
    pub open_loops: usize,
}

/// A polyline in 3D.
#[derive(Clone, Debug, PartialEq)]
pub struct Polyline3 {
    /// Points; a closed polyline does not repeat its first point.
    pub points: Vec<Point3>,
    /// Whether the last point connects back to the first.
    pub closed: bool,
}

/// Cut points closer than this (times the bounding-box diagonal) to an end of their edge are
/// that end.
const ALIAS_RELATIVE: f64 = 1e-12;
const KIND_POS: u8 = 0;
const KIND_NEG: u8 = 1;
const KIND_CROSS: u8 = 2;
/// Faces per work item. Fixed: results never depend on the thread count.
const CHUNK: usize = 1 << 14;

/// A vertex of a cut face piece: a mesh vertex (input or edge-cut) and its barycentric
/// weights in the parent face.
#[derive(Clone, Copy)]
struct CornerVert {
    id: u32,
    w: [f64; 3],
}

#[derive(Clone, Copy)]
struct Piece {
    tri: [CornerVert; 3],
    face: u32,
}

/// One cut edge (strictly opposite endpoint sides).
struct XEdge {
    key: u64,
    /// Parameter along `lo → hi`.
    t: f64,
    pos: Point3,
    /// The endpoint the cut point coincides with (to within rounding), or [`INVALID`]. Such a
    /// cut point is not a vertex of its own: using the endpoint avoids two loop vertices at
    /// the same position, which no triangulation could tell apart.
    alias: u32,
}

fn edge_key(a: u32, b: u32) -> u64 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    (u64::from(lo) << 32) | u64::from(hi)
}

fn unit(i: usize) -> [f64; 3] {
    let mut w = [0.0; 3];
    w[i] = 1.0;
    w
}

/// Classification of a mesh against a plane.
struct Cut<'a> {
    m: &'a IndexedMesh,
    plane: Plane,
    n_orig: u32,
    sign: Vec<i8>,
    kind: Vec<u8>,
    /// Faces with vertices strictly on both sides, ascending.
    crossing: Vec<u32>,
    /// Uncut faces with at least two vertices on the plane, ascending.
    plane_faces: Vec<u32>,
    /// Cut edges sorted by key; edge `k` becomes vertex `n_orig + k`.
    edges: Vec<XEdge>,
    /// Input vertices that stand in for a coincident cut point (ascending): they count as on
    /// the plane although their side is not zero.
    promoted: Vec<u32>,
}

impl<'a> Cut<'a> {
    fn new(m: &'a IndexedMesh, plane: &Plane, tol: f64, ctx: &JobCtx) -> MeshResult<Self> {
        if m.groups.per_face().len() != m.tris.len() {
            return Err(MeshError::AttributeLength {
                name: "groups",
                got: m.groups.per_face().len(),
                expected: m.tris.len(),
            });
        }
        let n = plane.normal.get();
        let o = plane.origin;
        if !(n.is_finite() && o.is_finite()) {
            return Err(MeshError::InvalidOption {
                name: "plane",
                reason: "origin and normal must be finite".into(),
            });
        }
        if !(tol.is_finite() && tol >= 0.0) {
            return Err(MeshError::InvalidOption {
                name: "snap_tolerance",
                reason: format!("must be finite and non-negative, got {tol}"),
            });
        }
        let ps = PlaneSide::new(o, n, tol);
        let n_orig = u32::try_from(m.positions.len()).map_err(|_| MeshError::TooLarge {
            count: m.positions.len(),
        })?;
        let sign: Vec<i8> = m.positions.par_iter().map(|&p| ps.side(p).0).collect();
        ctx.checkpoint()?;
        ctx.progress(0.1);

        let mut kind = vec![KIND_POS; m.tris.len()];
        let lists: Vec<Vec<u32>> = m
            .tris
            .par_chunks(CHUNK)
            .zip(kind.par_chunks_mut(CHUNK))
            .enumerate()
            .map(|(ci, (tris, kinds))| {
                let mut cross = Vec::new();
                for (i, (t, k)) in tris.iter().zip(kinds.iter_mut()).enumerate() {
                    let s = t.map(|v| sign[v as usize]);
                    let f = (ci * CHUNK + i) as u32;
                    let pos = s.iter().any(|&x| x > 0);
                    let neg = s.iter().any(|&x| x < 0);
                    *k = if pos && neg {
                        cross.push(f);
                        KIND_CROSS
                    } else if pos {
                        KIND_POS
                    } else if neg {
                        KIND_NEG
                    } else {
                        let [a, b, c] = t.map(|v| m.positions[v as usize]);
                        if (b - a).cross(c - a).dot(n) > 0.0 {
                            KIND_NEG
                        } else {
                            KIND_POS
                        }
                    };
                }
                cross
            })
            .collect();
        let crossing: Vec<u32> = lists.into_iter().flatten().collect();
        ctx.checkpoint()?;
        ctx.progress(0.2);

        let sign_ref = &sign;
        let mut keys: Vec<u64> = crossing
            .par_iter()
            .flat_map_iter(|&f| {
                let t = m.tris[f as usize];
                (0..3).filter_map(move |i| {
                    let j = (i + 1) % 3;
                    let (a, b) = (t[i], t[j]);
                    (sign_ref[a as usize] * sign_ref[b as usize] < 0).then(|| edge_key(a, b))
                })
            })
            .collect();
        keys.par_sort_unstable();
        keys.dedup();
        if (n_orig as usize) + keys.len() >= INVALID as usize {
            return Err(MeshError::TooLarge {
                count: n_orig as usize + keys.len(),
            });
        }
        let merge = ALIAS_RELATIVE * m.aabb().diagonal().max(o.length());
        let edges: Vec<XEdge> = keys
            .par_iter()
            .map(|&key| {
                let (lo, hi) = ((key >> 32) as usize, (key & 0xffff_ffff) as usize);
                let (pa, pb) = (m.positions[lo], m.positions[hi]);
                let (da, db) = (ps.side(pa).1, ps.side(pb).1);
                let t = (da / (da - db)).clamp(0.0, 1.0);
                let t = if t.is_nan() { 0.5 } else { t };
                let pos = pa + (pb - pa) * t;
                let (near_a, near_b) = (pos.distance(pa), pos.distance(pb));
                let alias = if near_a.min(near_b) > merge {
                    INVALID
                } else if near_a <= near_b {
                    lo as u32
                } else {
                    hi as u32
                };
                XEdge { key, t, pos, alias }
            })
            .collect();
        let mut promoted: Vec<u32> = edges
            .iter()
            .map(|e| e.alias)
            .filter(|&a| a != INVALID)
            .collect();
        promoted.sort_unstable();
        promoted.dedup();
        // Uncut faces with at least two vertices on the plane (exactly, or promoted).
        let mut on_plane: Vec<bool> = sign.iter().map(|&x| x == 0).collect();
        for &p in &promoted {
            on_plane[p as usize] = true;
        }
        let plane_faces: Vec<u32> = (0..m.tris.len() as u32)
            .into_par_iter()
            .filter(|&f| {
                kind[f as usize] != KIND_CROSS
                    && m.tris[f as usize]
                        .iter()
                        .filter(|&&v| on_plane[v as usize])
                        .count()
                        >= 2
            })
            .collect();
        ctx.progress(0.3);
        Ok(Cut {
            m,
            plane: *plane,
            n_orig,
            sign,
            kind,
            crossing,
            plane_faces,
            edges,
            promoted,
        })
    }

    fn pos(&self, id: u32) -> Point3 {
        if id < self.n_orig {
            self.m.positions[id as usize]
        } else {
            self.edges[(id - self.n_orig) as usize].pos
        }
    }

    fn on_plane(&self, id: u32) -> bool {
        id >= self.n_orig || self.sign[id as usize] == 0 || self.promoted.binary_search(&id).is_ok()
    }

    /// The cut vertex on edge `t[i] → t[j]`.
    fn cross_vertex(&self, t: [u32; 3], i: usize, j: usize) -> CornerVert {
        let (a, b) = (t[i], t[j]);
        let key = edge_key(a, b);
        let k = self
            .edges
            .binary_search_by_key(&key, |e| e.key)
            .unwrap_or(0);
        let tt = self.edges.get(k).map_or(0.5, |e| e.t);
        if let Some(e) = self.edges.get(k)
            && e.alias != INVALID
        {
            let corner = if e.alias == a { i } else { j };
            return CornerVert {
                id: e.alias,
                w: unit(corner),
            };
        }
        let mut w = [0.0; 3];
        if a < b {
            w[i] = 1.0 - tt;
            w[j] = tt;
        } else {
            w[j] = 1.0 - tt;
            w[i] = tt;
        }
        CornerVert {
            id: self.n_orig + k as u32,
            w,
        }
    }

    /// Appends the pieces of crossing face `f` on `side` (`1` positive, `-1` negative).
    fn pieces_into(&self, f: u32, side: i8, out: &mut Vec<Piece>) {
        let t = self.m.tris[f as usize];
        let s = t.map(|v| self.sign[v as usize]);
        let mut poly = [CornerVert { id: 0, w: [0.0; 3] }; 6];
        let mut len = 0;
        for i in 0..3 {
            let j = (i + 1) % 3;
            if (side > 0 && s[i] >= 0) || (side < 0 && s[i] <= 0) {
                poly[len] = CornerVert {
                    id: t[i],
                    w: unit(i),
                };
                len += 1;
            }
            if s[i] * s[j] < 0 {
                poly[len] = self.cross_vertex(t, i, j);
                len += 1;
            }
        }
        let mut push = |tri: [CornerVert; 3]| {
            let [a, b, c] = tri.map(|v| v.id);
            if a != b && b != c && a != c {
                out.push(Piece { tri, face: f });
            }
        };
        match len {
            3 => push([poly[0], poly[1], poly[2]]),
            4 => {
                let p = |k: usize| self.pos(poly[k].id);
                let (d02, d13) = (p(0).distance_squared(p(2)), p(1).distance_squared(p(3)));
                let (a, b) = if d02 <= d13 {
                    ([0, 1, 2], [0, 2, 3])
                } else {
                    ([0, 1, 3], [1, 2, 3])
                };
                for tri in [a, b] {
                    push(tri.map(|k| poly[k]));
                }
            }
            _ => {}
        }
    }

    fn pieces(&self, side: i8) -> Vec<Piece> {
        let parts: Vec<Vec<Piece>> = self
            .crossing
            .par_chunks(1024)
            .map(|chunk| {
                let mut out = Vec::with_capacity(chunk.len() * 3 / 2);
                for &f in chunk {
                    self.pieces_into(f, side, &mut out);
                }
                out
            })
            .collect();
        parts.into_iter().flatten().collect()
    }

    fn want_kind(side: i8) -> u8 {
        if side > 0 { KIND_POS } else { KIND_NEG }
    }

    /// Directed in-plane edges used by exactly one face of the half (its own winding).
    /// `Err(())` when an in-plane edge is used inconsistently (non-manifold input).
    fn plane_boundary(&self, side: i8, pieces: &[Piece]) -> Result<Vec<(u32, u32)>, ()> {
        let want = Self::want_kind(side);
        let mut raw: Vec<(u64, u8)> = Vec::new();
        let mut push = |a: u32, b: u32| {
            if a != b && self.on_plane(a) && self.on_plane(b) {
                raw.push((edge_key(a, b), u8::from(a < b)));
            }
        };
        for p in pieces {
            for k in 0..3 {
                push(p.tri[k].id, p.tri[(k + 1) % 3].id);
            }
        }
        for &f in &self.plane_faces {
            if self.kind[f as usize] == want {
                let t = self.m.tris[f as usize];
                for k in 0..3 {
                    push(t[k], t[(k + 1) % 3]);
                }
            }
        }
        raw.sort_unstable();
        let mut out = Vec::new();
        let mut i = 0;
        while i < raw.len() {
            let mut j = i;
            while j < raw.len() && raw[j].0 == raw[i].0 {
                j += 1;
            }
            let (lo, hi) = ((raw[i].0 >> 32) as u32, (raw[i].0 & 0xffff_ffff) as u32);
            match j - i {
                1 => out.push(if raw[i].1 == 1 { (lo, hi) } else { (hi, lo) }),
                2 if raw[i].1 != raw[i + 1].1 => {}
                _ => return Err(()),
            }
            i = j;
        }
        Ok(out)
    }

    fn local(&self, id: u32) -> (f64, f64) {
        let p = self.plane.to_local(self.pos(id));
        (p.x, p.y)
    }

    /// Builds the half on `side`, with its cap when `cap` is set. Returns the mesh (`None`
    /// when empty), its cap group, and the number of uncapped loops.
    fn half(
        &self,
        side: i8,
        cap: bool,
    ) -> MeshResult<(Option<IndexedMesh>, Option<FaceGroupId>, usize)> {
        let m = self.m;
        let want = Self::want_kind(side);
        let pieces = self.pieces(side);
        let mut cap_tris: Vec<[u32; 3]> = Vec::new();
        let mut open = 0;
        if cap {
            match self.plane_boundary(side, &pieces) {
                Err(()) => open += 1,
                Ok(edges) if edges.is_empty() => {}
                Ok(edges) => {
                    let chains = cap::trace(&edges, &|v| self.local(v));
                    let unclosed = chains.iter().filter(|c| !c.closed).count();
                    if unclosed > 0 {
                        open += unclosed;
                    } else {
                        let mirror = if side > 0 { -1.0 } else { 1.0 };
                        let uv = |v: u32| {
                            let (u, w) = self.local(v);
                            (u, mirror * w)
                        };
                        match cap::triangulate(&edges, &uv) {
                            Some(t) => cap_tris = t,
                            None => open += chains.len(),
                        }
                    }
                }
            }
        }

        let ids: Vec<u32> = (0..m.tris.len() as u32)
            .into_par_iter()
            .filter(|&f| self.kind[f as usize] == want)
            .collect();
        let n_total = self.n_orig as usize + self.edges.len();
        let used: Vec<AtomicBool> = (0..n_total)
            .into_par_iter()
            .map(|_| AtomicBool::new(false))
            .collect();
        let mark = |v: u32| {
            let u = &used[v as usize];
            if !u.load(Ordering::Relaxed) {
                u.store(true, Ordering::Relaxed);
            }
        };
        ids.par_iter().for_each(|&f| {
            for v in m.tris[f as usize] {
                mark(v);
            }
        });
        for p in &pieces {
            for c in &p.tri {
                mark(c.id);
            }
        }
        for t in &cap_tris {
            for &v in t {
                mark(v);
            }
        }
        let face_total = ids.len() + pieces.len() + cap_tris.len();
        if face_total == 0 {
            return Ok((None, None, open));
        }
        let mut map = vec![INVALID; n_total];
        let mut kept: Vec<u32> = Vec::new();
        for (v, u) in used.iter().enumerate() {
            if u.load(Ordering::Relaxed) {
                map[v] = kept.len() as u32;
                kept.push(v as u32);
            }
        }
        let positions: Vec<Point3> = kept.par_iter().map(|&v| self.pos(v)).collect();
        let mut tris: Vec<[u32; 3]> = Vec::with_capacity(face_total);
        tris.par_extend(
            ids.par_iter()
                .map(|&f| m.tris[f as usize].map(|v| map[v as usize])),
        );
        tris.extend(pieces.iter().map(|p| p.tri.map(|c| map[c.id as usize])));
        tris.extend(cap_tris.iter().map(|t| t.map(|v| map[v as usize])));

        // Face groups: the input table, plus one cap group when a cap exists.
        let (_, info, next) = m.groups.raw();
        let mut table = FaceGroups::from_raw(Vec::new(), info.clone(), next);
        let cap_id = if cap_tris.is_empty() {
            None
        } else {
            Some(table.try_alloc(FaceGroupInfo::new(GroupOrigin::Cap))?)
        };
        let (_, info, next) = table.raw();
        let src = m.groups.per_face();
        let mut per_face: Vec<FaceGroupId> = Vec::with_capacity(face_total);
        per_face.par_extend(ids.par_iter().map(|&f| src[f as usize]));
        per_face.extend(pieces.iter().map(|p| src[p.face as usize]));
        if let Some(c) = cap_id {
            per_face.extend(std::iter::repeat_n(c, cap_tris.len()));
        }
        let groups = FaceGroups::from_raw(per_face, info.clone(), next);

        let mut out = IndexedMesh {
            positions,
            tris,
            attrs: crate::MeshAttrs::default(),
            groups,
        };
        self.gather_attrs(&mut out, &kept, &ids, &pieces, cap_tris.len());
        Ok((Some(out), cap_id, open))
    }

    fn gather_attrs(
        &self,
        out: &mut IndexedMesh,
        kept: &[u32],
        ids: &[u32],
        pieces: &[Piece],
        cap_faces: usize,
    ) {
        let a = &self.m.attrs;
        let edge_ends = |id: u32| {
            let e = &self.edges[(id - self.n_orig) as usize];
            ((e.key >> 32) as usize, (e.key & 0xffff_ffff) as usize, e.t)
        };
        if let Some(ns) = &a.vertex_normals {
            out.attrs.vertex_normals = Some(
                kept.par_iter()
                    .map(|&v| {
                        if v < self.n_orig {
                            ns[v as usize]
                        } else {
                            let (lo, hi, t) = edge_ends(v);
                            let l = ns[lo].lerp(ns[hi], t);
                            l.try_normalize().unwrap_or(l)
                        }
                    })
                    .collect(),
            );
        }
        if let Some(cs) = &a.vertex_colors {
            out.attrs.vertex_colors = Some(
                kept.par_iter()
                    .map(|&v| {
                        if v < self.n_orig {
                            cs[v as usize]
                        } else {
                            let (lo, hi, t) = edge_ends(v);
                            cs[lo].lerp(cs[hi], t)
                        }
                    })
                    .collect(),
            );
        }
        if let Some(uvs) = &a.corner_uvs {
            let mut v: Vec<[Point2; 3]> = Vec::with_capacity(out.tris.len());
            v.extend(ids.iter().map(|&f| uvs[f as usize]));
            v.extend(pieces.iter().map(|p| {
                let src = uvs[p.face as usize];
                p.tri
                    .map(|c| src[0] * c.w[0] + src[1] * c.w[1] + src[2] * c.w[2])
            }));
            v.extend(std::iter::repeat_n([Point2::ZERO; 3], cap_faces));
            out.attrs.corner_uvs = Some(v);
        }
        if let Some(tp) = &a.tri_properties {
            let mut v: Vec<TriProperty> = Vec::with_capacity(out.tris.len());
            v.extend(ids.iter().map(|&f| tp[f as usize]));
            v.extend(pieces.iter().map(|p| {
                let src = tp[p.face as usize];
                let mut index = [INVALID; 3];
                for (k, c) in p.tri.iter().enumerate() {
                    let near = (0..3)
                        .max_by(|&x, &y| c.w[x].total_cmp(&c.w[y]))
                        .unwrap_or(0);
                    index[k] = src.index[near];
                }
                TriProperty {
                    group: src.group,
                    index,
                }
            }));
            v.extend(std::iter::repeat_n(TriProperty::NONE, cap_faces));
            out.attrs.tri_properties = Some(v);
        }
    }
}

/// Cuts `m` with a plane (see the module docs for the exact semantics).
///
/// `m` should be a closed, manifold, oriented mesh for the halves to be closed; an open or
/// non-manifold input is cut all the same, the affected section loops stay uncapped and are
/// counted in [`PlaneCutResult::open_loops`]. Cancellable between stages.
///
/// # Errors
/// [`MeshError::Empty`] for a mesh without faces, [`MeshError::InvalidOption`] for a
/// non-finite plane or tolerance, [`MeshError::Cancelled`].
pub fn plane_cut(m: &IndexedMesh, opts: &PlaneCutOpts, ctx: &JobCtx) -> MeshResult<PlaneCutResult> {
    if m.tris.is_empty() {
        return Err(MeshError::Empty);
    }
    let cut = Cut::new(m, &opts.plane, opts.snap_tolerance, ctx)?;
    let (want_p, want_n) = (
        opts.keep != KeepSide::Negative,
        opts.keep != KeepSide::Positive,
    );
    let (p, n) = rayon::join(
        || want_p.then(|| cut.half(1, opts.cap)),
        || want_n.then(|| cut.half(-1, opts.cap)),
    );
    ctx.checkpoint()?;
    ctx.progress(1.0);
    let mut res = PlaneCutResult {
        positive: None,
        negative: None,
        cap_group: None,
        open_loops: 0,
    };
    for (slot, half) in [(&mut res.positive, p), (&mut res.negative, n)] {
        if let Some(h) = half {
            let (mesh, cap_id, open) = h?;
            res.cap_group = res.cap_group.or(cap_id);
            res.open_loops = res.open_loops.max(open);
            *slot = mesh;
        }
    }
    Ok(res)
}

/// The cross-section of `m` with `plane` as polylines.
///
/// For a closed, oriented mesh the loops are closed and oriented with material on the left
/// seen from the plane's `+normal` side. Loops touching at a vertex are kept apart; chains of
/// an open mesh come back with `closed == false`. `snap_tolerance` is as in
/// [`PlaneCutOpts`]. A face lying in the plane contributes through the same convention as the
/// cut (it belongs to the half its normal faces away from), so a plane coinciding with a face
/// of a solid gives the outline of that face only where the solid continues beyond it.
/// Open chains first, then closed loops, in a deterministic order.
pub fn section(m: &IndexedMesh, plane: &Plane, snap_tolerance: f64) -> Vec<Polyline3> {
    if m.tris.is_empty() {
        return Vec::new();
    }
    let Ok(cut) = Cut::new(m, plane, snap_tolerance, &JobCtx::detached()) else {
        return Vec::new();
    };
    let pieces = cut.pieces(1);
    let Ok(edges) = cut.plane_boundary(1, &pieces) else {
        return Vec::new();
    };
    cap::trace(&edges, &|v| cut.local(v))
        .into_iter()
        .map(|c| Polyline3 {
            points: c.ids.iter().map(|&v| cut.pos(v)).collect(),
            closed: c.closed,
        })
        .collect()
}
