//! Operands lowered to primitive sets (note §2.1) and the exact minimum distance between two
//! sets by best-first simultaneous BVH traversal (note §2.2).
//!
//! All work happens in the local frame of operand A; B's primitives are mapped by
//! `T_ab = A.to_world⁻¹ · B.to_world` (exactly `t_ab.transform_point3` on each corner). Every
//! primitive pair whose distance can equal the minimum is evaluated, and the winner is the
//! lexicographic minimum of `(distance², a_prim, b_prim)`, so the result does not depend on
//! the traversal order or the thread count.

use std::cmp::{Ordering, Reverse};
use std::collections::BinaryHeap;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering as AtomicOrdering};

use amg_core::{Aabb3, Point3, Transform};
use amg_jobs::JobCtx;
use amg_mesh::{Bvh, BvhNodeKind, BvhOpts, FaceId, IndexedMesh};
use rayon::prelude::*;

use super::prims::{Prim, PrimDistance, PrimFeature, prim_distance};
use super::{MeshElement, MeshOperand, Operand, WitnessFeature};
use crate::inspect::InspectError;
use crate::inspect::cache::{Derived, derived, node_min_prims};

/// Sets with more primitives than this get their own BVH.
const FLAT_MAX: usize = 64;
/// Both sets above this size: the root's grandchild pairs run in parallel (note §2.2).
const PARALLEL_MIN: usize = 200_000;
/// Cancellation is checked every this many node pairs.
const CANCEL_EVERY: u32 = 4096;

/// What a list primitive's corners are in the mesh.
#[derive(Clone, Copy, Debug)]
enum Meta {
    /// A point with a fixed witness feature.
    Point(WitnessFeature),
    /// A segment between two mesh vertices.
    Seg(u32, u32),
}

enum Src {
    /// Mesh triangles: all faces, or the sorted subset `faces`.
    Mesh {
        mesh: Arc<IndexedMesh>,
        faces: Option<Vec<u32>>,
    },
    /// Explicit points and segments.
    List { prims: Vec<Prim>, meta: Vec<Meta> },
}

enum Tree {
    /// One leaf holding every primitive.
    Flat(Aabb3),
    /// A BVH over the set (slot → set index) and each node's smallest set index.
    Own(Bvh, Vec<u32>),
    /// The body's cached BVH (prim = face id).
    Body(Arc<Derived>),
}

#[derive(Clone, Copy)]
enum Node {
    Inner(u32, u32),
    Leaf(u32, u32),
}

/// An operand as a primitive set in its own local frame.
pub(crate) struct Lowered {
    pub(crate) to_world: Transform,
    src: Src,
    tree: Tree,
    /// The body's derived data when the operand is a whole body.
    pub(crate) body: Option<Arc<Derived>>,
}

impl Lowered {
    pub(crate) fn len(&self) -> usize {
        match &self.src {
            Src::Mesh { mesh, faces } => faces.as_ref().map_or(mesh.face_count(), Vec::len),
            Src::List { prims, .. } => prims.len(),
        }
    }

    fn prim(&self, i: u32) -> Prim {
        match &self.src {
            Src::Mesh { mesh, faces } => {
                let f = faces.as_ref().map_or(i, |f| f[i as usize]);
                Prim::Tri(mesh.triangle(FaceId(f)))
            }
            Src::List { prims, .. } => prims[i as usize],
        }
    }

    /// The mesh feature of primitive `i` holding a witness on `feat`.
    fn witness_feature(&self, i: u32, feat: PrimFeature) -> WitnessFeature {
        match &self.src {
            Src::Mesh { mesh, faces } => {
                let f = faces.as_ref().map_or(i, |f| f[i as usize]);
                let v = mesh.tris[f as usize];
                match feat {
                    PrimFeature::Corner(k) => WitnessFeature::Vertex(v[usize::from(k)]),
                    PrimFeature::Edge(k, l) => edge(v[usize::from(k)], v[usize::from(l)]),
                    PrimFeature::Interior => WitnessFeature::Face(FaceId(f)),
                }
            }
            Src::List { meta, .. } => match (meta[i as usize], feat) {
                (Meta::Point(w), _) => w,
                (Meta::Seg(a, _), PrimFeature::Corner(0)) => WitnessFeature::Vertex(a),
                (Meta::Seg(_, b), PrimFeature::Corner(_)) => WitnessFeature::Vertex(b),
                (Meta::Seg(a, b), _) => edge(a, b),
            },
        }
    }

    fn root(&self) -> Option<u32> {
        match &self.tree {
            Tree::Flat(_) => (self.len() > 0).then_some(0),
            Tree::Own(b, _) => b.root(),
            Tree::Body(d) => d.spatial.bvh().root(),
        }
    }

    fn node(&self, i: u32) -> (Aabb3, Node) {
        let bvh = match &self.tree {
            Tree::Flat(b) => return (*b, Node::Leaf(0, self.len() as u32)),
            Tree::Own(b, _) => b,
            Tree::Body(d) => d.spatial.bvh(),
        };
        let n = bvh.node(i);
        let kind = match n.kind {
            BvhNodeKind::Inner { left, right } => Node::Inner(left, right),
            BvhNodeKind::Leaf { first, count } => Node::Leaf(first, count),
        };
        (n.bounds, kind)
    }

    /// Smallest primitive index under node `i`.
    fn min_prim(&self, i: u32) -> u32 {
        match &self.tree {
            Tree::Flat(_) => 0,
            Tree::Own(_, m) => m[i as usize],
            Tree::Body(d) => d.min_prims()[i as usize],
        }
    }

    fn slot(&self, s: u32) -> u32 {
        match &self.tree {
            Tree::Flat(_) => s,
            Tree::Own(b, _) => b.prim(s),
            Tree::Body(d) => d.spatial.bvh().prim(s),
        }
    }
}

fn edge(a: u32, b: u32) -> WitnessFeature {
    WitnessFeature::Edge(a.min(b), a.max(b))
}

/// `|det R − 1| ≤ 1e-9` and `RᵀR = I` within 1e-9 (note §2.1).
pub(crate) fn is_rigid(t: &Transform) -> bool {
    let m = t.matrix3;
    if !m.is_finite() || !t.translation.is_finite() || (m.determinant() - 1.0).abs() > 1e-9 {
        return false;
    }
    let g = m.transpose() * m;
    (0..3).all(|i| (0..3).all(|j| (g.col(i)[j] - f64::from(u8::from(i == j))).abs() <= 1e-9))
}

/// Checks `op` and returns the mesh's derived data when the element is the whole body.
pub(crate) fn check_mesh(op: &MeshOperand) -> Result<(), InspectError> {
    if is_rigid(&op.to_world) {
        Ok(())
    } else {
        Err(InspectError::NonRigidTransform)
    }
}

fn bad(what: impl std::fmt::Display) -> InspectError {
    InspectError::BadElement(what.to_string())
}

/// Validated vertex index.
pub(crate) fn vertex(m: &IndexedMesh, v: u32) -> Result<Point3, InspectError> {
    m.positions
        .get(v as usize)
        .copied()
        .ok_or_else(|| bad(format_args!("vertex {v}")))
}

/// The sorted, deduplicated faces of a faces / group element.
pub(crate) fn element_faces(
    m: &IndexedMesh,
    element: &MeshElement,
) -> Result<Option<Vec<u32>>, InspectError> {
    let faces: Vec<u32> = match element {
        MeshElement::FaceGroup(g) => {
            if m.groups.info(*g).is_none() {
                return Err(bad(format_args!("face group {}", g.0)));
            }
            m.groups.faces_of(*g).map(|f| f.0).collect()
        }
        MeshElement::Faces(ids) => {
            let mut f: Vec<u32> = ids.iter().map(|f| f.0).collect();
            f.sort_unstable();
            f.dedup();
            if let Some(&last) = f.last()
                && last as usize >= m.face_count()
            {
                return Err(bad(format_args!("face {last}")));
            }
            f
        }
        _ => return Ok(None),
    };
    if faces.is_empty() {
        return Err(bad("an empty face set"));
    }
    Ok(Some(faces))
}

/// The segments of a polyline element (closing segment included when `closed`).
pub(crate) fn polyline_segments(
    m: &IndexedMesh,
    verts: &[u32],
    closed: bool,
) -> Result<Vec<(u32, u32)>, InspectError> {
    if verts.len() < 2 {
        return Err(bad("a polyline with fewer than 2 vertices"));
    }
    for &v in verts {
        vertex(m, v)?;
    }
    let mut segs: Vec<(u32, u32)> = verts.windows(2).map(|w| (w[0], w[1])).collect();
    if closed && verts.len() > 2 {
        segs.push((verts[verts.len() - 1], verts[0]));
    }
    Ok(segs)
}

/// Lowers `op` to a primitive set (note §2.1).
pub(crate) fn lower(op: &Operand, ctx: &JobCtx) -> Result<Lowered, InspectError> {
    let (to_world, src, body) = match op {
        Operand::Point(p) => (
            Transform::IDENTITY,
            Src::List {
                prims: vec![Prim::Point(*p)],
                meta: vec![Meta::Point(WitnessFeature::Point)],
            },
            None,
        ),
        Operand::Mesh(mo) => {
            check_mesh(mo)?;
            let mesh = mo.body.mesh();
            let m: &IndexedMesh = mesh;
            let list = |prims, meta| Src::List { prims, meta };
            match &mo.element {
                MeshElement::Body => {
                    if m.face_count() == 0 {
                        return Err(InspectError::Degenerate("the body has no faces"));
                    }
                    let d = derived(&mo.body, ctx)?;
                    (
                        mo.to_world,
                        Src::Mesh {
                            mesh: Arc::clone(mesh),
                            faces: None,
                        },
                        Some(d),
                    )
                }
                e @ (MeshElement::FaceGroup(_) | MeshElement::Faces(_)) => (
                    mo.to_world,
                    Src::Mesh {
                        mesh: Arc::clone(mesh),
                        faces: element_faces(m, e)?,
                    },
                    None,
                ),
                MeshElement::Edge { a, b } => {
                    let (pa, pb) = (vertex(m, *a)?, vertex(m, *b)?);
                    if a == b {
                        return Err(bad(format_args!("edge {a}–{b}")));
                    }
                    (
                        mo.to_world,
                        list(vec![Prim::Seg([pa, pb])], vec![Meta::Seg(*a, *b)]),
                        None,
                    )
                }
                MeshElement::Polyline { verts, closed } => {
                    let segs = polyline_segments(m, verts, *closed)?;
                    let prims = segs
                        .iter()
                        .map(|&(a, b)| {
                            Prim::Seg([m.positions[a as usize], m.positions[b as usize]])
                        })
                        .collect();
                    let meta = segs.iter().map(|&(a, b)| Meta::Seg(a, b)).collect();
                    (mo.to_world, list(prims, meta), None)
                }
                MeshElement::Vertex(v) => (
                    mo.to_world,
                    list(
                        vec![Prim::Point(vertex(m, *v)?)],
                        vec![Meta::Point(WitnessFeature::Vertex(*v))],
                    ),
                    None,
                ),
                MeshElement::SurfacePoint { face, bary } => {
                    let p = surface_point(m, *face, bary)?;
                    (
                        mo.to_world,
                        list(
                            vec![Prim::Point(p)],
                            vec![Meta::Point(WitnessFeature::Face(*face))],
                        ),
                        None,
                    )
                }
            }
        }
    };
    let mut low = Lowered {
        to_world,
        src,
        tree: Tree::Flat(Aabb3::EMPTY),
        body: body.clone(),
    };
    low.tree = if let Some(d) = body {
        Tree::Body(d)
    } else {
        let bounds: Vec<Aabb3> = (0..low.len() as u32)
            .map(|i| {
                let (lo, hi) = low.prim(i).bounds();
                Aabb3 { min: lo, max: hi }
            })
            .collect();
        if bounds.len() > FLAT_MAX {
            let bvh = Bvh::build(&bounds, &BvhOpts::default(), ctx)?;
            let mins = node_min_prims(&bvh);
            Tree::Own(bvh, mins)
        } else {
            Tree::Flat(bounds.iter().fold(Aabb3::EMPTY, |acc, b| acc.union(b)))
        }
    };
    Ok(low)
}

/// Point of `face` at barycentric weights `bary`.
pub(crate) fn surface_point(
    m: &IndexedMesh,
    face: FaceId,
    bary: &[f64; 3],
) -> Result<Point3, InspectError> {
    if face.0 as usize >= m.face_count() {
        return Err(bad(format_args!("face {}", face.0)));
    }
    if !bary.iter().all(|w| w.is_finite()) {
        return Err(InspectError::Degenerate("non-finite barycentric weights"));
    }
    let [a, b, c] = m.triangle(face);
    Ok(a * bary[0] + b * bary[1] + c * bary[2])
}

/// The closest pair found, in A's local frame.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Closest {
    pub(crate) dist_sq: f64,
    pub(crate) ia: u32,
    pub(crate) ib: u32,
    /// Points in A's frame.
    pub(crate) pa: Point3,
    pub(crate) pb: Point3,
    pub(crate) fa: WitnessFeature,
    pub(crate) fb: WitnessFeature,
}

fn key_less(a: (f64, u32, u32), b: (f64, u32, u32)) -> bool {
    match a.0.total_cmp(&b.0) {
        Ordering::Less => true,
        Ordering::Greater => false,
        Ordering::Equal => (a.1, a.2) < (b.1, b.2),
    }
}

#[derive(Clone, Copy, PartialEq)]
struct Entry {
    lb2: f64,
    /// Smallest primitive ids under the two nodes (tie pruning at distance 0).
    ma: u32,
    mb: u32,
    na: u32,
    nb: u32,
}

impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Entry {
    fn cmp(&self, o: &Self) -> Ordering {
        self.lb2
            .total_cmp(&o.lb2)
            .then(self.ma.cmp(&o.ma))
            .then(self.mb.cmp(&o.mb))
            .then(self.na.cmp(&o.na))
            .then(self.nb.cmp(&o.nb))
    }
}

/// Once a zero-distance pair is known, only pairs with smaller ids can still win: a node pair
/// whose smallest ids `(ma, mb)` cannot beat `(best.ia, best.ib)` is skipped.
fn dominated(best: Option<&Closest>, ma: u32, mb: u32) -> bool {
    best.is_some_and(|b| b.dist_sq <= 0.0 && (ma > b.ia || (ma == b.ia && mb >= b.ib)))
}

fn dominated_pair(best: Option<&Closest>, ia: u32, ib: u32) -> bool {
    best.is_some_and(|b| b.dist_sq <= 0.0 && (ia, ib) >= (b.ia, b.ib))
}

/// Lower bound on the distance between `t` (a triangle) and `o`: if every corner of `o` lies
/// strictly on one side of `t`'s plane, the smallest corner distance to that plane; else 0.
fn plane_gap(t: &Prim, o: &Prim) -> f64 {
    let Prim::Tri([a, b, c]) = t else {
        return 0.0;
    };
    let n = (*b - *a).cross(*c - *a);
    let len = n.length();
    if len.is_nan() || len <= 0.0 {
        return 0.0;
    }
    let n = n / len;
    let pts: &[Point3] = match o {
        Prim::Point(p) => std::slice::from_ref(p),
        Prim::Seg(s) => s,
        Prim::Tri(t) => t,
    };
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in pts {
        let d = n.dot(*p - *a);
        lo = lo.min(d);
        hi = hi.max(d);
    }
    if lo > 0.0 {
        lo
    } else if hi < 0.0 {
        -hi
    } else {
        0.0
    }
}

/// Gap between two boxes (0 if they overlap).
fn box_gap(a: &Aabb3, b: &Aabb3) -> f64 {
    let d = (a.min - b.max).max(b.min - a.max).max(Point3::ZERO);
    d.length()
}

struct Traversal<'a> {
    a: &'a Lowered,
    b: &'a Lowered,
    t_ab: Transform,
    /// Absolute slack subtracted from every lower bound (rounding of the transformed boxes).
    eps: f64,
    /// Slack of the triangle-plane lower bound.
    eps_plane: f64,
    shared: &'a AtomicU64,
    ctx: &'a JobCtx,
}

impl Traversal<'_> {
    fn lower_bound_sq(&self, ba: &Aabb3, bb: &Aabb3) -> f64 {
        let g = (box_gap(ba, &bb.transformed(&self.t_ab)) - self.eps).max(0.0);
        g * g
    }

    fn entry(&self, na: u32, nb: u32) -> Entry {
        let (ba, _) = self.a.node(na);
        let (bb, _) = self.b.node(nb);
        Entry {
            lb2: self.lower_bound_sq(&ba, &bb),
            ma: self.a.min_prim(na),
            mb: self.b.min_prim(nb),
            na,
            nb,
        }
    }

    fn bound(&self, best: Option<&Closest>) -> f64 {
        let s = f64::from_bits(self.shared.load(AtomicOrdering::Relaxed));
        best.map_or(s, |b| b.dist_sq.min(s))
    }

    /// Best-first traversal from `start`; returns the task's best pair.
    fn run(&self, start: Entry) -> Result<Option<Closest>, InspectError> {
        let mut heap = BinaryHeap::new();
        heap.push(Reverse(start));
        let mut best: Option<Closest> = None;
        let mut buf_b: Vec<(u32, Prim, Aabb3)> = Vec::new();
        let mut pops = 0u32;
        while let Some(Reverse(e)) = heap.pop() {
            pops = pops.wrapping_add(1);
            if pops.is_multiple_of(CANCEL_EVERY) {
                self.ctx.checkpoint()?;
            }
            let u2 = self.bound(best.as_ref());
            if e.lb2 > u2 {
                break;
            }
            if dominated(best.as_ref(), e.ma, e.mb) {
                continue;
            }
            let (ba, ka) = self.a.node(e.na);
            let (bb, kb) = self.b.node(e.nb);
            match (ka, kb) {
                (Node::Leaf(fa, ca), Node::Leaf(fb, cb)) => {
                    self.leaf_pair((fa, ca), (fb, cb), &mut best, &mut buf_b);
                }
                _ => {
                    let split_a = match (ka, kb) {
                        (Node::Inner(..), Node::Leaf(..)) => true,
                        (Node::Leaf(..), _) => false,
                        _ => ba.diagonal() >= bb.diagonal(),
                    };
                    let kids: [Entry; 2] = match (split_a, ka, kb) {
                        (true, Node::Inner(l, r), _) => [self.entry(l, e.nb), self.entry(r, e.nb)],
                        (false, _, Node::Inner(l, r)) => [self.entry(e.na, l), self.entry(e.na, r)],
                        _ => continue,
                    };
                    let u2 = self.bound(best.as_ref());
                    for k in kids {
                        if k.lb2 <= u2 && !dominated(best.as_ref(), k.ma, k.mb) {
                            heap.push(Reverse(k));
                        }
                    }
                }
            }
        }
        Ok(best)
    }

    fn leaf_pair(
        &self,
        (fa, ca): (u32, u32),
        (fb, cb): (u32, u32),
        best: &mut Option<Closest>,
        buf_b: &mut Vec<(u32, Prim, Aabb3)>,
    ) {
        buf_b.clear();
        for s in fb..fb + cb {
            let ib = self.b.slot(s);
            let p = self.b.prim(ib).transformed(&self.t_ab);
            let (lo, hi) = p.bounds();
            buf_b.push((ib, p, Aabb3 { min: lo, max: hi }));
        }
        for s in fa..fa + ca {
            let ia = self.a.slot(s);
            let pa = self.a.prim(ia);
            let (lo, hi) = pa.bounds();
            let box_a = Aabb3 { min: lo, max: hi };
            for (ib, pb, box_b) in buf_b.iter() {
                let u2 = self.bound(best.as_ref());
                let g = (box_gap(&box_a, box_b) - self.eps).max(0.0);
                if g * g > u2 || dominated_pair(best.as_ref(), ia, *ib) {
                    continue;
                }
                let lp = (plane_gap(&pa, pb).max(plane_gap(pb, &pa)) - self.eps_plane).max(0.0);
                if lp * lp > u2 {
                    continue;
                }
                let d: PrimDistance = prim_distance(&pa, pb);
                let better = best
                    .as_ref()
                    .is_none_or(|c| key_less((d.dist_sq, ia, *ib), (c.dist_sq, c.ia, c.ib)));
                if better {
                    *best = Some(Closest {
                        dist_sq: d.dist_sq,
                        ia,
                        ib: *ib,
                        pa: d.a,
                        pb: d.b,
                        fa: self.a.witness_feature(ia, d.fa),
                        fb: self.b.witness_feature(*ib, d.fb),
                    });
                    self.shared
                        .fetch_min(d.dist_sq.to_bits(), AtomicOrdering::Relaxed);
                }
            }
        }
    }
}

/// Exact closest pair of two lowered operands, in A's frame. `None` if either set is empty.
pub(crate) fn closest(
    a: &Lowered,
    b: &Lowered,
    ctx: &JobCtx,
) -> Result<Option<Closest>, InspectError> {
    let (Some(ra), Some(rb)) = (a.root(), b.root()) else {
        return Ok(None);
    };
    let t_ab = a.to_world.inverse() * b.to_world;
    let (box_a, _) = a.node(ra);
    let (box_b, _) = b.node(rb);
    let box_b = box_b.transformed(&t_ab);
    let scale = [box_a.min, box_a.max, box_b.min, box_b.max]
        .iter()
        .map(|p| p.abs().max_element())
        .fold(1.0f64, f64::max);
    let shared = AtomicU64::new(f64::INFINITY.to_bits());
    let tr = Traversal {
        a,
        b,
        t_ab,
        eps: 1e-12 * scale,
        eps_plane: 1e-10 * scale,
        shared: &shared,
        ctx,
    };
    let start = tr.entry(ra, rb);
    let results: Vec<Option<Closest>> = if a.len() > PARALLEL_MIN && b.len() > PARALLEL_MIN {
        // The root's grandchild pairs (≤ 4 × 4) run in parallel with a shared upper bound.
        let mut tasks = vec![(ra, rb)];
        for _ in 0..2 {
            tasks = tasks
                .into_iter()
                .flat_map(|(na, nb)| {
                    let ca = children(a, na);
                    let cb = children(b, nb);
                    ca.into_iter()
                        .flat_map(move |x| cb.clone().into_iter().map(move |y| (x, y)))
                        .collect::<Vec<_>>()
                })
                .collect();
        }
        tasks
            .par_iter()
            .map(|&(na, nb)| tr.run(tr.entry(na, nb)))
            .collect::<Result<_, _>>()?
    } else {
        vec![tr.run(start)?]
    };
    Ok(results.into_iter().flatten().reduce(|x, y| {
        if key_less((y.dist_sq, y.ia, y.ib), (x.dist_sq, x.ia, x.ib)) {
            y
        } else {
            x
        }
    }))
}

fn children(s: &Lowered, n: u32) -> Vec<u32> {
    match s.node(n).1 {
        Node::Inner(l, r) => vec![l, r],
        Node::Leaf(..) => vec![n],
    }
}
