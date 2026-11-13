//! Section caps on meshes (INS-02, S05 §3, note `inspection-geometry.md` §3).
//!
//! A section plane **removes the half-space its normal points into** (`signed_distance > 0`);
//! several planes keep the intersection of their negative half-spaces. For each body and plane
//! [`section_mesh`]:
//! 1. maps the plane into body-local coordinates and calls [`amg_mesh::section`] for the section
//!    polylines (closed loops of a closed, oriented mesh come back with material on the left
//!    seen from `+normal`, S07 MESH-09; a mirroring `to_world` reverses them);
//! 2. inserts the closed-loop vertices into a `spade` constrained Delaunay triangulation and
//!    classifies its triangles by winding number:
//!    - loops that do not cross (the normal case) become constraint edges as they are, and the
//!      winding number is propagated from the outer face across every edge by the signed count
//!      of loop segments on it: exact, with no geometric inside test;
//!    - crossing loops (overlapping shells) are inserted with `add_constraint_and_split`, and each
//!      region bounded by constraint edges is evaluated once, at the incentre of its fattest
//!      triangle, against the oriented loops;
//! 3. keeps a triangle for `w != 0` ([`FillRule::NonZero`]: overlapping shells give their union)
//!    or odd `w` ([`FillRule::EvenOdd`], for meshes that are not closed and oriented);
//! 4. clips the kept triangles and the section curves by the other planes' kept half-spaces
//!    (Sutherland–Hodgman in the plane frame, fan triangulation);
//! 5. returns world-space cap triangles with plane-local `uv` (mm), the closed outlines, the
//!    open curves (never capped) and [`SectionStats`].
//!
//! Open section curves of broken meshes are reported in [`Cap::open`] and
//! [`SectionStats::open_loops`]; only closed loops bound caps.

#[cfg(test)]
mod tests;

use std::collections::HashMap;

use amg_core::{Aabb2, Area, Dir3, Length, Plane, Point2, Point3, Quat, Tolerance, Transform};
use amg_jobs::JobCtx;
use amg_mesh::{MeshBody, Polyline3};
use rayon::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, Triangulation};

use super::InspectError;

/// Most section planes that combine (owner Q2, RND-06).
pub const MAX_SECTION_PLANES: usize = 6;

/// World-space section planes, at most [`MAX_SECTION_PLANES`]. Each plane removes the
/// half-space its normal points into (`signed_distance > 0`).
#[derive(Clone, Debug, PartialEq)]
pub struct SectionPlanes(pub Vec<Plane>);

/// Which points of the plane are inside the section loops.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FillRule {
    /// [`FillRule::NonZero`] if the mesh is closed and consistently oriented, else
    /// [`FillRule::EvenOdd`].
    Auto,
    /// Winding number `!= 0` (the union of overlapping closed shells).
    NonZero,
    /// Odd winding number (loops taken as unoriented).
    EvenOdd,
}

/// Options for [`section_mesh`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapOpts {
    /// Mesh vertices within this distance (mm, body frame) of a plane count as on it, as in
    /// [`amg_mesh::PlaneCutOpts::snap_tolerance`].
    pub snap_tolerance: f64,
    /// Inside test of the cap.
    pub fill_rule: FillRule,
}

impl Default for CapOpts {
    /// `Tolerance::mesh().linear` (1e-5 mm) and [`FillRule::Auto`].
    fn default() -> Self {
        CapOpts {
            snap_tolerance: Tolerance::mesh().linear,
            fill_rule: FillRule::Auto,
        }
    }
}

/// The cap of one body on one plane.
#[derive(Clone, Debug, PartialEq)]
pub struct Cap {
    /// Index of the plane in [`SectionPlanes`].
    pub plane_index: u8,
    /// Cap vertices, world. Vertices of the section loops are the section points themselves
    /// (shared bit for bit with [`amg_mesh::plane_cut`] for an identity `to_world`); crossing and
    /// clip vertices lie on the plane.
    pub positions: Vec<Point3>,
    /// Plane-local coordinates of `positions` in mm (`Plane::to_local`), for world-anchored
    /// hatching.
    pub uv: Vec<Point2>,
    /// Counter-clockwise seen from the removed side (`+normal`), i.e. facing the viewer.
    pub tris: Vec<[u32; 3]>,
    /// Closed section curves after clipping by the other planes, world. An unclipped loop is
    /// `closed`; the pieces of a clipped loop are open polylines.
    pub outlines: Vec<Polyline3>,
    /// Open section curves (meshes that are not closed), clipped the same way; never capped.
    pub open: Vec<Polyline3>,
}

/// Statistics of one plane over one body.
#[derive(Clone, Debug, PartialEq)]
pub struct SectionStats {
    /// Index of the plane in [`SectionPlanes`].
    pub plane_index: u8,
    /// Σ cap triangle areas.
    pub area: Area,
    /// Σ lengths of [`Cap::outlines`] (closed loops include their closing segment).
    pub perimeter: Length,
    /// Closed loops with a part inside the other planes' kept half-spaces.
    pub loops: u32,
    /// Open curves with a part inside the other planes' kept half-spaces.
    pub open_loops: u32,
    /// Bounding box of the cap and the curves in the plane frame (`Aabb2::EMPTY` if nothing).
    pub extent: Aabb2,
}

/// Caps and statistics of one body: one [`Cap`] and one [`SectionStats`] per plane, in plane
/// order (empty when the plane misses the body).
#[derive(Clone, Debug, PartialEq)]
pub struct BodySection {
    /// Per plane.
    pub caps: Vec<Cap>,
    /// Per plane.
    pub stats: Vec<SectionStats>,
}

/// Section caps of `body` placed by `to_world` for `planes` (see the module docs).
///
/// `to_world` may be any invertible affine transform (a mirror reverses the loops; caps stay
/// facing `+normal`). The closed-and-oriented flag for [`FillRule::Auto`] is cached per mesh.
/// Planes are processed in parallel; the result does not depend on the thread count.
///
/// # Errors
/// [`InspectError::TooManyPlanes`] for more than [`MAX_SECTION_PLANES`] planes,
/// [`InspectError::Degenerate`] for a non-finite plane, a singular or non-finite `to_world`,
/// a negative or non-finite snap tolerance, or loop coordinates the triangulation rejects,
/// [`InspectError::Cancelled`].
pub fn section_mesh(
    body: &MeshBody,
    to_world: &Transform,
    planes: &SectionPlanes,
    opts: &CapOpts,
    ctx: &JobCtx,
) -> Result<BodySection, InspectError> {
    let planes = &planes.0;
    if planes.len() > MAX_SECTION_PLANES {
        return Err(InspectError::TooManyPlanes(planes.len()));
    }
    if !(opts.snap_tolerance.is_finite() && opts.snap_tolerance >= 0.0) {
        return Err(InspectError::Degenerate("snap tolerance"));
    }
    if planes.iter().any(|p| !plane_finite(p)) {
        return Err(InspectError::Degenerate("non-finite section plane"));
    }
    let det = to_world.matrix3.determinant();
    let m = to_world.matrix3;
    let scale = m.x_axis.length() * m.y_axis.length() * m.z_axis.length();
    if !det.is_finite() || !to_world.translation.is_finite() || det.abs() <= 1e-12 * scale {
        return Err(InspectError::Degenerate("singular to_world transform"));
    }
    if planes.is_empty() {
        return Ok(BodySection {
            caps: Vec::new(),
            stats: Vec::new(),
        });
    }
    let rule = match opts.fill_rule {
        FillRule::Auto => {
            if super::cache::oriented(body, ctx)? {
                FillRule::NonZero
            } else {
                FillRule::EvenOdd
            }
        }
        r => r,
    };
    ctx.checkpoint()?;
    let inv = to_world.inverse();
    // Body-frame snap distance: the local length that maps to `snap_tolerance` on average.
    let snap_local = opts.snap_tolerance / det.abs().cbrt();
    let mirrored = det < 0.0;
    let mesh = body.mesh();
    let results: Vec<Result<(Cap, SectionStats), InspectError>> = (0..planes.len())
        .into_par_iter()
        .map(|i| {
            ctx.checkpoint()?;
            let plane = &planes[i];
            let local = local_plane(plane, to_world, &inv)?;
            let mut curves = amg_mesh::section(mesh, &local, snap_local);
            for c in &mut curves {
                for p in &mut c.points {
                    *p = to_world.transform_point3(*p);
                }
                if mirrored {
                    c.points.reverse();
                }
            }
            ctx.checkpoint()?;
            let others: Vec<HalfPlane> = planes
                .iter()
                .enumerate()
                .filter(|&(j, _)| j != i)
                .map(|(_, q)| HalfPlane::new(plane, q))
                .collect();
            let others3: Vec<&Plane> = planes
                .iter()
                .enumerate()
                .filter(|&(j, _)| j != i)
                .map(|(_, q)| q)
                .collect();
            build_cap(i as u8, plane, &curves, rule, &others, &others3)
        })
        .collect();
    ctx.checkpoint()?;
    ctx.progress(1.0);
    let mut out = BodySection {
        caps: Vec::with_capacity(planes.len()),
        stats: Vec::with_capacity(planes.len()),
    };
    for r in results {
        let (c, s) = r?;
        out.caps.push(c);
        out.stats.push(s);
    }
    Ok(out)
}

/// The Phase 1 section plane (S05 §3, `SectionAnalysis`): `base` moved by `offset` mm along its
/// normal, rotated about the moved origin by `tilt[0]` rad about the base `x` axis and then by
/// `tilt[1]` rad about the base `y` axis, and finally [`Plane::flipped`] if `flip`.
///
/// The frame stays right-handed (`normal = x × y`); with zero tilt the result is `base`
/// translated (and flipped), with `x` kept.
pub fn section_plane(base: &Plane, offset: f64, tilt: [f64; 2], flip: bool) -> Plane {
    let origin = base.origin + base.normal.get() * offset;
    let rot =
        Quat::from_axis_angle(base.y.get(), tilt[1]) * Quat::from_axis_angle(base.x.get(), tilt[0]);
    let unit = |v: Point3| Dir3::new_unchecked(v.normalize());
    let x = unit(rot * base.x.get());
    let y = unit(rot * base.y.get());
    let normal = unit(x.get().cross(y.get()));
    let p = Plane {
        origin,
        x,
        y,
        normal,
    };
    if flip { p.flipped() } else { p }
}

fn plane_finite(p: &Plane) -> bool {
    p.origin.is_finite()
        && p.x.get().is_finite()
        && p.y.get().is_finite()
        && p.normal.get().is_finite()
        && (p.normal.get().length() - 1.0).abs() < 1e-6
}

/// The world plane in body-local coordinates (same positive side).
fn local_plane(
    plane: &Plane,
    to_world: &Transform,
    inv: &Transform,
) -> Result<Plane, InspectError> {
    // An identity `to_world` keeps the plane bit for bit, so the section points equal those of
    // `amg_mesh::plane_cut` with the same plane.
    if *to_world == Transform::IDENTITY {
        return Ok(*plane);
    }
    // n·(M x + t − o) = (Mᵀ n)·x + …, so the local normal is Mᵀ n (direction only).
    let n = to_world.matrix3.transpose() * plane.normal.get();
    let len = n.length();
    if !(len.is_finite() && len > 0.0) {
        return Err(InspectError::Degenerate("section plane in the body frame"));
    }
    Ok(Plane::from_normal(
        inv.transform_point3(plane.origin),
        Dir3::new_unchecked(n / len),
    ))
}

/// The kept side `s(uv) ≤ 0` of another plane `q`, in the frame of `plane`.
#[derive(Clone, Copy, Debug)]
struct HalfPlane {
    d0: f64,
    a: f64,
    b: f64,
}

impl HalfPlane {
    fn new(plane: &Plane, q: &Plane) -> HalfPlane {
        let n = q.normal.get();
        HalfPlane {
            d0: q.signed_distance(plane.origin),
            a: plane.x.get().dot(n),
            b: plane.y.get().dot(n),
        }
    }

    fn eval(&self, p: Point2) -> f64 {
        self.d0 + p.x * self.a + p.y * self.b
    }
}

/// Flushes values spade rejects (non-zero below 2^-142).
fn flush(x: f64) -> f64 {
    if x.abs() < 1e-30 { 0.0 } else { x }
}

type Cdt = ConstrainedDelaunayTriangulation<spade::Point2<f64>>;

fn spade_err(_: spade::InsertionError) -> InspectError {
    InspectError::Degenerate("section loop coordinates rejected by the triangulation")
}

/// Cap of one plane from its world-space section curves.
fn build_cap(
    index: u8,
    plane: &Plane,
    curves: &[Polyline3],
    rule: FillRule,
    others: &[HalfPlane],
    others3: &[&Plane],
) -> Result<(Cap, SectionStats), InspectError> {
    let closed: Vec<&Polyline3> = curves
        .iter()
        .filter(|c| c.closed && c.points.len() >= 3)
        .collect();
    let mut cap = Cap {
        plane_index: index,
        positions: Vec::new(),
        uv: Vec::new(),
        tris: Vec::new(),
        outlines: Vec::new(),
        open: Vec::new(),
    };
    let mut stats = SectionStats {
        plane_index: index,
        area: Area::ZERO,
        perimeter: Length::ZERO,
        loops: 0,
        open_loops: 0,
        extent: Aabb2::EMPTY,
    };
    let mut extent = Aabb2::EMPTY;
    let mut grow = |p: Point2| {
        extent.min = extent.min.min(p);
        extent.max = extent.max.max(p);
    };

    // ---- CDT of the closed loops ----
    let loops_uv: Vec<Vec<Point2>> = closed
        .iter()
        .map(|c| c.points.iter().map(|&p| plane.to_local(p)).collect())
        .collect();
    let (cdt_world, cdt_uv, tris) = if closed.is_empty() {
        (Vec::new(), Vec::new(), Vec::new())
    } else {
        triangulate(&closed, &loops_uv, rule, plane)?
    };

    // ---- clip by the other planes ----
    let mut verts = Verts::default();
    let mut remap: Vec<u32> = vec![u32::MAX; cdt_uv.len()];
    let mut vid = |verts: &mut Verts, k: usize| -> u32 {
        if remap[k] == u32::MAX {
            remap[k] = verts.push(cdt_world[k], cdt_uv[k]);
        }
        remap[k]
    };
    let mut area2 = 0.0;
    for t in tris {
        let pts = t.map(|k| cdt_uv[k as usize]);
        let inside = others.iter().all(|h| pts.iter().all(|&p| h.eval(p) <= 0.0));
        if inside {
            let ids = t.map(|k| vid(&mut verts, k as usize));
            area2 += cross2(pts[1] - pts[0], pts[2] - pts[0]);
            cap.tris.push(ids);
            continue;
        }
        // Polygon of (uv, existing id or MAX).
        let mut poly: Vec<(Point2, Option<usize>)> =
            (0..3).map(|j| (pts[j], Some(t[j] as usize))).collect();
        for h in others {
            poly = clip_poly(&poly, h);
            if poly.len() < 3 {
                break;
            }
        }
        if poly.len() < 3 {
            continue;
        }
        let ids: Vec<u32> = poly
            .iter()
            .map(|&(p, k)| match k {
                Some(k) => vid(&mut verts, k),
                None => verts.push_clip(plane, p),
            })
            .collect();
        for j in 1..ids.len() - 1 {
            let (a, b, c) = (poly[0].0, poly[j].0, poly[j + 1].0);
            let a2 = cross2(b - a, c - a);
            if a2 <= 0.0 || ids[0] == ids[j] || ids[j] == ids[j + 1] || ids[0] == ids[j + 1] {
                continue;
            }
            area2 += a2;
            cap.tris.push([ids[0], ids[j], ids[j + 1]]);
        }
    }
    for &p in &verts.uv {
        grow(p);
    }
    cap.positions = verts.world;
    cap.uv = verts.uv;
    stats.area = Area::from_canonical(0.5 * area2);

    // ---- outlines and open curves ----
    let mut perimeter = 0.0;
    for c in curves {
        if c.points.len() < 2 {
            continue;
        }
        let pieces = clip_polyline(c, others3);
        if pieces.is_empty() {
            continue;
        }
        for piece in &pieces {
            for &p in &piece.points {
                grow(plane.to_local(p));
            }
        }
        if c.closed {
            stats.loops += 1;
            for piece in &pieces {
                perimeter += polyline_length(piece);
            }
            cap.outlines.extend(pieces);
        } else {
            stats.open_loops += 1;
            cap.open.extend(pieces);
        }
    }
    stats.perimeter = Length::mm(perimeter);
    stats.extent = extent;
    Ok((cap, stats))
}

/// CDT vertices (world, uv) and kept triangles (CCW in uv).
type Kept = (Vec<Point3>, Vec<Point2>, Vec<[u32; 3]>);

/// Vertex handles of each closed loop.
type LoopHandles = Vec<Vec<spade::handles::FixedVertexHandle>>;

/// A CDT holding the loop vertices (equal positions share a vertex), their world positions
/// and each loop's vertex handles.
fn insert_points(
    closed: &[&Polyline3],
    loops_uv: &[Vec<Point2>],
) -> Result<(Cdt, Vec<Point3>, LoopHandles), InspectError> {
    let mut cdt = Cdt::new();
    let mut world: Vec<Point3> = Vec::new();
    let mut handles: LoopHandles = Vec::with_capacity(closed.len());
    for (c, uv) in closed.iter().zip(loops_uv) {
        let mut hs = Vec::with_capacity(uv.len());
        for (&p, &q) in c.points.iter().zip(uv) {
            let h = cdt
                .insert(spade::Point2::new(flush(q.x), flush(q.y)))
                .map_err(spade_err)?;
            if h.index() == world.len() {
                world.push(p);
            }
            hs.push(h);
        }
        handles.push(hs);
    }
    Ok((cdt, world, handles))
}

fn segments(handles: &LoopHandles) -> impl Iterator<Item = (FixedVertex, FixedVertex)> + '_ {
    handles.iter().flat_map(|hs| {
        (0..hs.len())
            .map(move |k| (hs[k], hs[(k + 1) % hs.len()]))
            .filter(|(a, b)| a != b)
    })
}

type FixedVertex = spade::handles::FixedVertexHandle;

fn keep(rule: FillRule, w: i64) -> bool {
    match rule {
        FillRule::EvenOdd => w % 2 != 0,
        _ => w != 0,
    }
}

/// Exact path for loops that do not cross: every segment becomes one constraint edge, and the
/// winding number is carried from the outer face (0) across edges by the signed number of
/// loop segments on each edge (material on the left: crossing a segment from its right to its
/// left adds one). No geometric inside test, so slivers and touching loops are classified
/// exactly. `None` when a segment crosses another or passes through a vertex.
fn triangulate_exact(
    cdt: &mut Cdt,
    handles: &LoopHandles,
    rule: FillRule,
) -> Option<Vec<[u32; 3]>> {
    for (a, b) in segments(handles) {
        if !cdt.can_add_constraint(a, b) {
            return None;
        }
        cdt.add_constraint(a, b);
    }
    // Signed coverage per undirected edge, positive along lo → hi.
    let mut cov: HashMap<(usize, usize), i64> = HashMap::new();
    for (a, b) in segments(handles) {
        let e = cdt.get_edge_from_neighbors(a, b)?;
        if !e.is_constraint_edge() {
            return None;
        }
        let (i, j) = (a.index(), b.index());
        *cov.entry((i.min(j), i.max(j))).or_default() += if i < j { 1 } else { -1 };
    }
    let along = |u: usize, v: usize| -> i64 {
        let c = cov.get(&(u.min(v), u.max(v))).copied().unwrap_or(0);
        if u < v { c } else { -c }
    };
    let mut w: Vec<Option<i64>> = vec![None; cdt.num_all_faces()];
    let mut stack = Vec::new();
    // Hull edges have the outer face on their left; the inner face is on their right.
    for e in cdt.directed_edges() {
        if !e.face().is_outer() {
            continue;
        }
        if let Some(r) = e.rev().face().as_inner() {
            let ri = r.fix().index();
            if w[ri].is_none() {
                w[ri] = Some(-along(e.from().fix().index(), e.to().fix().index()));
                stack.push(r.fix());
            }
        }
    }
    while let Some(f) = stack.pop() {
        let wf = w[f.index()]?;
        for de in cdt.face(f).adjacent_edges() {
            if let Some(r) = de.rev().face().as_inner() {
                let ri = r.fix().index();
                if w[ri].is_none() {
                    w[ri] = Some(wf - along(de.from().fix().index(), de.to().fix().index()));
                    stack.push(r.fix());
                }
            }
        }
    }
    let mut tris = Vec::new();
    for f in cdt.inner_faces() {
        if w[f.fix().index()].is_some_and(|x| keep(rule, x)) {
            tris.push(f.vertices().map(|v| v.fix().index() as u32));
        }
    }
    Some(tris)
}

/// CDT of the closed loops and the triangles inside them under `rule`: the exact path of
/// [`triangulate_exact`] when no loops cross, else split constraints and one winding-number
/// probe per constraint-bounded region.
fn triangulate(
    closed: &[&Polyline3],
    loops_uv: &[Vec<Point2>],
    rule: FillRule,
    plane: &Plane,
) -> Result<Kept, InspectError> {
    let (mut cdt, world, handles) = insert_points(closed, loops_uv)?;
    if let Some(tris) = triangulate_exact(&mut cdt, &handles, rule) {
        let uv: Vec<Point2> = (0..cdt.num_vertices())
            .map(|i| {
                let p = cdt.vertex(FixedVertex::from_index(i)).position();
                Point2::new(p.x, p.y)
            })
            .collect();
        return Ok((world, uv, tris));
    }
    let (mut cdt, mut world, handles) = insert_points(closed, loops_uv)?;
    for (a, b) in segments(&handles) {
        cdt.add_constraint_and_split(a, b, |p| p);
    }
    // Vertices created by splits lie on the plane.
    let uv_of = |i: usize| {
        let p = cdt
            .vertex(spade::handles::FixedVertexHandle::from_index(i))
            .position();
        Point2::new(p.x, p.y)
    };
    let n_v = cdt.num_vertices();
    let uv_all: Vec<Point2> = (0..n_v).map(uv_of).collect();
    for p in &uv_all[world.len()..] {
        world.push(plane.to_world(*p));
    }

    // Regions: inner faces joined across non-constraint edges.
    let n_f = cdt.num_all_faces();
    let mut region = vec![u32::MAX; n_f];
    let mut reps: Vec<(f64, Point2)> = Vec::new(); // (inradius, incentre) per region
    let mut stack = Vec::new();
    for f in cdt.inner_faces() {
        let fi = f.fix().index();
        if region[fi] != u32::MAX {
            continue;
        }
        let r = reps.len() as u32;
        reps.push((-1.0, Point2::ZERO));
        region[fi] = r;
        stack.push(f.fix());
        while let Some(g) = stack.pop() {
            let face = cdt.face(g);
            let [a, b, c] = face.vertices().map(|v| {
                let p = v.position();
                Point2::new(p.x, p.y)
            });
            let (la, lb, lc) = ((b - c).length(), (c - a).length(), (a - b).length());
            let per = la + lb + lc;
            let inr = if per > 0.0 {
                cross2(b - a, c - a).abs() / per
            } else {
                0.0
            };
            let rep = &mut reps[r as usize];
            if inr > rep.0 {
                *rep = (inr, (a * la + b * lb + c * lc) / per);
            }
            for de in face.adjacent_edges() {
                if de.is_constraint_edge() {
                    continue;
                }
                if let Some(nb) = de.rev().face().as_inner() {
                    let ni = nb.fix().index();
                    if region[ni] == u32::MAX {
                        region[ni] = r;
                        stack.push(nb.fix());
                    }
                }
            }
        }
    }
    let inside: Vec<bool> = reps
        .iter()
        .map(|&(inr, p)| {
            if inr <= 0.0 {
                return false;
            }
            keep(rule, winding(p, loops_uv))
        })
        .collect();
    let mut tris = Vec::new();
    for f in cdt.inner_faces() {
        let r = region[f.fix().index()];
        if r != u32::MAX && inside[r as usize] {
            tris.push(f.vertices().map(|v| v.fix().index() as u32));
        }
    }
    Ok((world, uv_all, tris))
}

/// Winding number of `p` with respect to closed polygons (Sunday's crossing rule).
fn winding(p: Point2, loops: &[Vec<Point2>]) -> i64 {
    let mut w = 0i64;
    for l in loops {
        let n = l.len();
        for k in 0..n {
            let (a, b) = (l[k], l[(k + 1) % n]);
            if a.y <= p.y {
                if b.y > p.y && cross2(b - a, p - a) > 0.0 {
                    w += 1;
                }
            } else if b.y <= p.y && cross2(b - a, p - a) < 0.0 {
                w -= 1;
            }
        }
    }
    w
}

fn cross2(a: Point2, b: Point2) -> f64 {
    a.x * b.y - a.y * b.x
}

/// Sutherland–Hodgman step against one half-plane. The cut point of an edge is computed from
/// its endpoints in a canonical order, so the two triangles sharing an edge get the same bits.
fn clip_poly(poly: &[(Point2, Option<usize>)], h: &HalfPlane) -> Vec<(Point2, Option<usize>)> {
    let mut out = Vec::with_capacity(poly.len() + 1);
    let n = poly.len();
    for k in 0..n {
        let cur = poly[k];
        let next = poly[(k + 1) % n];
        let (sc, sn) = (h.eval(cur.0), h.eval(next.0));
        if sc <= 0.0 {
            out.push(cur);
        }
        if (sc <= 0.0) != (sn <= 0.0) {
            out.push((cut_point(cur.0, next.0, h), None));
        }
    }
    out
}

fn cut_point(p: Point2, q: Point2, h: &HalfPlane) -> Point2 {
    let (p, q) = match p.x.total_cmp(&q.x).then(p.y.total_cmp(&q.y)) {
        std::cmp::Ordering::Greater => (q, p),
        _ => (p, q),
    };
    let (sp, sq) = (h.eval(p), h.eval(q));
    let t = sp / (sp - sq);
    let x = p + (q - p) * t;
    // Pull the point onto the kept side if rounding left it a hair outside.
    let s = h.eval(x);
    if s > 0.0 {
        let g = Point2::new(h.a, h.b);
        let gg = g.length_squared();
        if gg > 0.0 {
            return x - g * (s / gg);
        }
    }
    x
}

/// Output vertices of a cap, with clip vertices deduplicated by their uv bits.
#[derive(Default)]
struct Verts {
    world: Vec<Point3>,
    uv: Vec<Point2>,
    clip: HashMap<(u64, u64), u32>,
}

impl Verts {
    fn push(&mut self, w: Point3, uv: Point2) -> u32 {
        self.world.push(w);
        self.uv.push(uv);
        (self.world.len() - 1) as u32
    }

    fn push_clip(&mut self, plane: &Plane, uv: Point2) -> u32 {
        let key = (uv.x.to_bits(), uv.y.to_bits());
        if let Some(&i) = self.clip.get(&key) {
            return i;
        }
        let i = self.push(plane.to_world(uv), uv);
        self.clip.insert(key, i);
        i
    }
}

/// Clips a world polyline to the kept half-spaces of `planes` (`signed_distance ≤ 0`).
/// Returns the kept pieces; an untouched closed polyline comes back whole and closed.
fn clip_polyline(c: &Polyline3, planes: &[&Plane]) -> Vec<Polyline3> {
    let pts = &c.points;
    let inside = |p: Point3| planes.iter().all(|q| q.signed_distance(p) <= 0.0);
    if pts.iter().all(|&p| inside(p)) {
        return vec![c.clone()];
    }
    let n = pts.len();
    let segs = if c.closed { n } else { n - 1 };
    let mut pieces: Vec<Vec<Point3>> = Vec::new();
    let mut cur: Vec<Point3> = Vec::new();
    for k in 0..segs {
        let (p, q) = (pts[k], pts[(k + 1) % n]);
        let (mut t0, mut t1) = (0.0f64, 1.0f64);
        let mut out = false;
        for pl in planes {
            let (sp, sq) = (pl.signed_distance(p), pl.signed_distance(q));
            if sp > 0.0 && sq > 0.0 {
                out = true;
                break;
            }
            if sp > 0.0 {
                t0 = t0.max(sp / (sp - sq));
            } else if sq > 0.0 {
                t1 = t1.min(sp / (sp - sq));
            }
        }
        if out || t0 > t1 {
            if cur.len() >= 2 {
                pieces.push(std::mem::take(&mut cur));
            }
            cur.clear();
            continue;
        }
        let a = if t0 <= 0.0 { p } else { p + (q - p) * t0 };
        let b = if t1 >= 1.0 { q } else { p + (q - p) * t1 };
        if cur.is_empty() || t0 > 0.0 {
            if cur.len() >= 2 {
                pieces.push(std::mem::take(&mut cur));
            }
            cur.clear();
            cur.push(a);
        }
        cur.push(b);
        if t1 < 1.0 {
            pieces.push(std::mem::take(&mut cur));
        }
    }
    if cur.len() >= 2 {
        pieces.push(cur);
    }
    // A closed loop's last piece runs into its first one through point 0.
    if c.closed && pieces.len() >= 2 {
        let first_starts_at_0 = pieces[0].first() == Some(&pts[0]);
        let last_ends_at_0 = pieces.last().and_then(|l| l.last()) == Some(&pts[0]);
        if first_starts_at_0 && last_ends_at_0 {
            let first = pieces.remove(0);
            if let Some(last) = pieces.last_mut() {
                last.extend_from_slice(&first[1..]);
            }
        }
    }
    pieces
        .into_iter()
        .filter(|p| p.len() >= 2)
        .map(|points| Polyline3 {
            points,
            closed: false,
        })
        .collect()
}

fn polyline_length(p: &Polyline3) -> f64 {
    let mut l: f64 = p.points.windows(2).map(|w| w[0].distance(w[1])).sum();
    if p.closed && p.points.len() >= 2 {
        l += p.points[p.points.len() - 1].distance(p.points[0]);
    }
    l
}
