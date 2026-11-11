//! Placement of a body on a plate (S16 AM-03, design note `am-build-prep.md` §4).
//!
//! AM-01 defined the stored data types ([`Placement`], [`ZMode`]); this module adds the
//! algorithms: the effective transform, face-down rotation, fit check, hull overlap and the
//! display geometry of the build volume.
//!
//! The stored translation is never kept: [`effective_transform`] recomputes it from the mesh
//! every time, so an item stays on the plate and at the same `xy` when an upstream feature
//! changes the body.

use std::f64::consts::PI;

use amg_core::glam::{DAffine3, DMat3, DQuat, DVec2, DVec3};
use amg_core::{Aabb3, Transform};
use amg_jobs::JobCtx;
use amg_mesh::{FaceGroups, FaceId, IndexedMesh, MeshAttrs, Polyline3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::AmError;
use crate::machine::{BedShape, BuildVolume};

/// Linear tolerance of the fit check, mm (`Tolerance::DEFAULT.linear`).
pub const FIT_TOLERANCE: f64 = 1e-6;
/// Vertices per rayon work item. Fixed: results never depend on the thread count.
const CHUNK: usize = 1 << 14;
/// Below this vertex count the passes run serially.
const PAR_MIN: usize = 1 << 15;
/// Thickness of the display plate slab, mm (top face at z = 0).
pub const PLATE_THICKNESS: f64 = 2.0;
/// Thickness of the display exclude slabs, mm (bottom face at z = 0).
pub const EXCLUDE_THICKNESS: f64 = 0.2;
/// Normal spread (rad) up to which a face group counts as planar for [`pick_plane_normal`].
const PLANAR_SPREAD: f64 = 1.0 * PI / 180.0;

/// Vertical placement mode.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ZMode {
    /// Lowest point at `elevation` above the plate (0 = on the plate).
    OnPlate {
        /// Height of the lowest point above the plate, mm.
        elevation: f64,
    },
    /// Fixed plate-frame z offset, mm.
    Fixed {
        /// The offset.
        z: f64,
    },
}

/// Where and how a body sits on the plate.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Placement {
    /// Unit rotation, body-local → plate orientation.
    pub rotation: DQuat,
    /// Plate position of the rotated footprint-AABB centre, mm.
    pub xy: DVec2,
    /// Vertical mode.
    pub z: ZMode,
}

impl Placement {
    /// On the plate (elevation 0) with `rotation` at `xy`.
    pub fn on_plate(rotation: DQuat, xy: DVec2) -> Self {
        Placement {
            rotation,
            xy,
            z: ZMode::OnPlate { elevation: 0.0 },
        }
    }

    /// Re-normalises the rotation (call after composing rotations).
    pub fn normalized(self) -> Self {
        Placement {
            rotation: self.rotation.normalize(),
            ..self
        }
    }
}

// ------------------------------------------------------------------------ transform

/// Min and max of `m·v + t` over `positions`; ordered reductions, so identical for any thread
/// count.
fn bounds(positions: &[DVec3], m: &DMat3, t: DVec3) -> (DVec3, DVec3) {
    let fold = |chunk: &[DVec3]| {
        let mut lo = DVec3::INFINITY;
        let mut hi = DVec3::NEG_INFINITY;
        for &p in chunk {
            let q = *m * p + t;
            lo = lo.min(q);
            hi = hi.max(q);
        }
        (lo, hi)
    };
    if positions.len() < PAR_MIN {
        return fold(positions);
    }
    positions.par_chunks(CHUNK).map(fold).reduce(
        || (DVec3::INFINITY, DVec3::NEG_INFINITY),
        |a, b| (a.0.min(b.0), a.1.max(b.1)),
    )
}

/// Body-local → plate transform: `translate(t) · R` with `t.xy = p.xy − centre_xy(AABB(R·V))`
/// and `t.z = elevation − min_z(R·V)` (or the fixed `z`). One parallel pass over the vertices;
/// deterministic. An empty mesh gets `t = (xy, elevation | z)`.
pub fn effective_transform(m: &IndexedMesh, p: &Placement) -> Transform {
    let rot = DMat3::from_quat(p.rotation.normalize());
    let (lo, hi) = bounds(&m.positions, &rot, DVec3::ZERO);
    let (cx, cy, min_z) = if lo.x <= hi.x {
        ((lo.x + hi.x) * 0.5, (lo.y + hi.y) * 0.5, lo.z)
    } else {
        (0.0, 0.0, 0.0)
    };
    let tz = match p.z {
        ZMode::OnPlate { elevation } => elevation - min_z,
        ZMode::Fixed { z } => z,
    };
    DAffine3::from_mat3_translation(rot, DVec3::new(p.xy.x - cx, p.xy.y - cy, tz))
}

/// Rotation that puts body-local normal `n_local` facing −Z, composed with `current` by
/// shortest arc (keeps unrelated yaw). The anti-parallel case (the face points up) is a 180°
/// turn about plate X. A zero or non-finite normal returns `current`.
pub fn face_down_rotation(current: DQuat, n_local: DVec3) -> DQuat {
    let n = current * n_local;
    let len = n.length();
    if !(len > 0.0 && len.is_finite()) {
        return current;
    }
    let mut v = n / len;
    // Exact half turn about X, used first when the arc would be ill-conditioned.
    let mut acc = DQuat::IDENTITY;
    if v.z > 0.999 {
        acc = DQuat::from_xyzw(1.0, 0.0, 0.0, 0.0);
        v = DVec3::new(v.x, -v.y, -v.z);
    }
    let c = v.cross(DVec3::NEG_Z);
    let w = 1.0 + v.dot(DVec3::NEG_Z);
    let arc = DQuat::from_xyzw(c.x, c.y, c.z, w).normalize();
    (arc * acc * current).normalize()
}

/// Spins the item about the build axis; the anchor (`xy`, `z`) is unchanged.
pub fn rotate_about_build_axis(p: &Placement, angle: f64) -> Placement {
    Placement {
        rotation: (DQuat::from_rotation_z(angle) * p.rotation).normalize(),
        ..*p
    }
}

/// Moves the item so the footprint-AABB centre is at the centre of the bed's bounding
/// rectangle.
pub fn centre_on_plate(p: &Placement, v: &BuildVolume) -> Placement {
    Placement {
        xy: v.centre(),
        ..*p
    }
}

/// Normal used for "place on plate" when the user picks triangle `f`: the area-weighted mean
/// normal of the triangle's face group if the group has a fitted plane or a normal spread
/// ≤ 1°, else the triangle's own normal. A degenerate triangle gives `DVec3::ZERO`; an
/// out-of-range `f` gives `DVec3::Z`.
pub fn pick_plane_normal(m: &IndexedMesh, f: FaceId) -> DVec3 {
    if f.0 as usize >= m.face_count() {
        return DVec3::Z;
    }
    let own = m.face_normal(f);
    let g = m.groups.group_of(f);
    let fitted_plane = m
        .groups
        .info(g)
        .is_some_and(|i| matches!(i.primitive, Some(amg_mesh::Primitive::Plane { .. })));
    let mut sum = DVec3::ZERO;
    for fi in m.groups.faces_of(g) {
        sum += m.face_normal(fi) * m.face_area(fi);
    }
    let mean = sum.normalize_or_zero();
    if mean == DVec3::ZERO {
        return own;
    }
    if fitted_plane {
        return mean;
    }
    let cos_max = PLANAR_SPREAD.cos();
    let planar = m.groups.faces_of(g).all(|fi| {
        let n = m.face_normal(fi);
        n == DVec3::ZERO || n.dot(mean) >= cos_max
    });
    if planar { mean } else { own }
}

// ---------------------------------------------------------------------------- hull

fn cross2(o: DVec2, a: DVec2, b: DVec2) -> f64 {
    (a - o).perp_dot(b - o)
}

/// Andrew's monotone chain; CCW, no repeated first point, collinear points dropped.
fn monotone_chain(mut pts: Vec<DVec2>) -> Vec<DVec2> {
    pts.sort_unstable_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }
    let mut h: Vec<DVec2> = Vec::with_capacity(pts.len().min(1024));
    for &p in &pts {
        while h.len() >= 2 && cross2(h[h.len() - 2], h[h.len() - 1], p) <= 0.0 {
            h.pop();
        }
        h.push(p);
    }
    let lower = h.len() + 1;
    for &p in pts.iter().rev().skip(1) {
        while h.len() >= lower && cross2(h[h.len() - 2], h[h.len() - 1], p) <= 0.0 {
            h.pop();
        }
        h.push(p);
    }
    h.pop();
    h
}

/// Plate-frame xy of `p` under the affine `t`, from the first two matrix rows only.
#[derive(Clone, Copy)]
struct XyMap {
    rx: DVec3,
    ry: DVec3,
    t: DVec2,
}

impl XyMap {
    fn new(t: &Transform) -> Self {
        let m = t.matrix3;
        XyMap {
            rx: DVec3::new(m.x_axis.x, m.y_axis.x, m.z_axis.x),
            ry: DVec3::new(m.x_axis.y, m.y_axis.y, m.z_axis.y),
            t: t.translation.truncate(),
        }
    }

    fn apply(&self, p: DVec3) -> DVec2 {
        DVec2::new(self.rx.dot(p) + self.t.x, self.ry.dot(p) + self.t.y)
    }
}

/// Directions of the Akl–Toussaint pre-filter: ±x, ±y, ±(x+y), ±(x−y).
fn key(k: usize, p: DVec2) -> f64 {
    match k {
        0 => p.x,
        1 => -p.x,
        2 => p.y,
        3 => -p.y,
        4 => p.x + p.y,
        5 => -p.x - p.y,
        6 => p.x - p.y,
        _ => p.y - p.x,
    }
}

type Extremes = [(f64, DVec2); 8];

const NO_EXTREMES: Extremes = [(f64::NEG_INFINITY, DVec2::ZERO); 8];

fn extremes_of(chunk: &[DVec3], map: &XyMap) -> Extremes {
    let mut e = NO_EXTREMES;
    for &v in chunk {
        let p = map.apply(v);
        if !(p.x.is_finite() && p.y.is_finite()) {
            continue;
        }
        for (k, slot) in e.iter_mut().enumerate() {
            let kv = key(k, p);
            if kv > slot.0 {
                *slot = (kv, p);
            }
        }
    }
    e
}

fn merge_extremes(mut a: Extremes, b: Extremes) -> Extremes {
    for (x, y) in a.iter_mut().zip(b) {
        if y.0 > x.0 {
            *x = y;
        }
    }
    a
}

/// Convex hull (CCW) of the transformed vertices projected to the plate plane.
///
/// Points strictly inside the octagon spanned by the eight extreme points are dropped in
/// parallel before the sort, so the sort only sees the points near the rim.
pub fn footprint_hull(m: &IndexedMesh, t: &Transform) -> Vec<DVec2> {
    let map = XyMap::new(t);
    let pos = &m.positions;
    let ext = if pos.len() < PAR_MIN {
        extremes_of(pos, &map)
    } else {
        pos.par_chunks(CHUNK)
            .map(|c| extremes_of(c, &map))
            .reduce(|| NO_EXTREMES, merge_extremes)
    };
    if ext[0].0 == f64::NEG_INFINITY {
        return Vec::new(); // no finite vertex
    }
    let octagon = monotone_chain(ext.iter().map(|e| e.1).collect());
    let scale = (ext[0].0 + ext[1].0).max(ext[2].0 + ext[3].0).max(1.0);
    let margin = 1e-9 * scale * scale;
    let keep = |p: DVec2| -> bool {
        if !(p.x.is_finite() && p.y.is_finite()) {
            return false;
        }
        if octagon.len() < 3 {
            return true;
        }
        (0..octagon.len()).any(|i| {
            let a = octagon[i];
            let b = octagon[(i + 1) % octagon.len()];
            cross2(a, b, p) <= margin
        })
    };
    let survivors: Vec<DVec2> = if pos.len() < PAR_MIN {
        pos.iter()
            .map(|&v| map.apply(v))
            .filter(|&p| keep(p))
            .collect()
    } else {
        let parts: Vec<Vec<DVec2>> = pos
            .par_chunks(CHUNK)
            .map(|c| {
                c.iter()
                    .map(|&v| map.apply(v))
                    .filter(|&p| keep(p))
                    .collect()
            })
            .collect();
        parts.concat()
    };
    monotone_chain(survivors)
}

// ----------------------------------------------------------------------------- fit

/// Result of [`check_fit`].
#[derive(Clone, Debug, PartialEq)]
pub struct FitReport {
    /// Height, footprint and exclude checks all pass (within [`FIT_TOLERANCE`]).
    pub fits: bool,
    /// mm above `volume.height` (0 if ok).
    pub height_excess: f64,
    /// mm the footprint hull leaves the bed (0 if ok).
    pub max_outside_xy: f64,
    /// The footprint hull overlaps a keep-out polygon.
    pub hits_exclude: bool,
    /// Footprint hull, CCW, plate frame.
    pub footprint_hull: Vec<DVec2>,
    /// Bounds of the transformed vertices, plate frame (`Aabb3::EMPTY` for an empty mesh).
    pub aabb: Aabb3,
}

fn point_in_polygon(p: DVec2, poly: &[DVec2]) -> bool {
    let mut inside = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}

fn dist_to_segment(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let len2 = ab.length_squared();
    let t = if len2 > 0.0 {
        ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + ab * t)).length()
}

fn dist_to_boundary(p: DVec2, poly: &[DVec2]) -> f64 {
    let mut best = f64::INFINITY;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        best = best.min(dist_to_segment(p, poly[j], poly[i]));
        j = i;
    }
    best
}

/// Parameter `t` on segment `a→b` where it meets segment `c→d`, if they cross or touch.
fn segment_param(a: DVec2, b: DVec2, c: DVec2, d: DVec2) -> Option<f64> {
    let r = b - a;
    let s = d - c;
    let den = r.perp_dot(s);
    if den == 0.0 {
        return None;
    }
    let t = (c - a).perp_dot(s) / den;
    let u = (c - a).perp_dot(r) / den;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some(t)
}

/// Edges `(i, i+1)` of a hull of `n` vertices (a two-point hull has one edge).
fn hull_edge_count(n: usize) -> usize {
    match n {
        0 | 1 => 0,
        2 => 1,
        _ => n,
    }
}

/// How far the footprint hull leaves the bed, mm.
fn outside_bed(v: &BuildVolume, poly: &[DVec2], hull: &[DVec2]) -> f64 {
    if hull.is_empty() {
        return 0.0;
    }
    if let BedShape::Circle { diameter } = v.bed {
        // Convex: the hull is inside iff its vertices are.
        let (c, r) = (DVec2::splat(diameter * 0.5), diameter * 0.5);
        return hull
            .iter()
            .map(|p| ((*p - c).length() - r).max(0.0))
            .fold(0.0, f64::max);
    }
    if poly.len() < 3 {
        return f64::INFINITY;
    }
    let outside = |p: DVec2| -> f64 {
        if point_in_polygon(p, poly) {
            0.0
        } else {
            dist_to_boundary(p, poly)
        }
    };
    let mut worst = hull.iter().map(|p| outside(*p)).fold(0.0, f64::max);
    // Hull edges can cross a notch of a non-convex bed while both ends are inside.
    if !matches!(v.bed, BedShape::Rect { .. }) {
        for i in 0..hull_edge_count(hull.len()) {
            let (a, b) = (hull[i], hull[(i + 1) % hull.len()]);
            let mut ts = vec![0.0, 1.0];
            let mut j = poly.len() - 1;
            for k in 0..poly.len() {
                if let Some(t) = segment_param(a, b, poly[j], poly[k]) {
                    ts.push(t);
                }
                j = k;
            }
            ts.sort_unstable_by(f64::total_cmp);
            for w in ts.windows(2) {
                let (t0, t1) = (w[0], w[1]);
                if t1 - t0 < 1e-12 {
                    continue;
                }
                let f = |t: f64| outside(a + (b - a) * t);
                if f(0.5 * (t0 + t1)) <= 0.0 {
                    continue;
                }
                worst = worst.max(maximise(&f, t0, t1));
            }
        }
    }
    worst
}

/// Maximum of `f` on `[lo, hi]`: 33 samples, then golden-section refinement of the best bracket.
fn maximise(f: &dyn Fn(f64) -> f64, lo: f64, hi: f64) -> f64 {
    const N: usize = 32;
    let step = (hi - lo) / N as f64;
    let (mut bi, mut bv) = (0usize, f64::NEG_INFINITY);
    for i in 0..=N {
        let v = f(lo + step * i as f64);
        if v > bv {
            (bi, bv) = (i, v);
        }
    }
    let mut a = lo + step * bi.saturating_sub(1) as f64;
    let mut b = (lo + step * (bi + 1) as f64).min(hi);
    let g = (5f64.sqrt() - 1.0) * 0.5;
    let (mut c, mut d) = (b - g * (b - a), a + g * (b - a));
    let (mut fc, mut fd) = (f(c), f(d));
    for _ in 0..60 {
        if fc > fd {
            b = d;
            (d, fd) = (c, fc);
            c = b - g * (b - a);
            fc = f(c);
        } else {
            a = c;
            (c, fc) = (d, fd);
            d = a + g * (b - a);
            fd = f(d);
        }
        bv = bv.max(fc).max(fd);
    }
    bv
}

/// True if the hull overlaps the keep-out polygon `e` by more than the tolerance.
fn hull_hits_polygon(hull: &[DVec2], e: &[DVec2]) -> bool {
    if e.len() < 3 || hull.is_empty() {
        return false;
    }
    let tol = FIT_TOLERANCE;
    if hull
        .iter()
        .any(|p| point_in_polygon(*p, e) && dist_to_boundary(*p, e) > tol)
    {
        return true;
    }
    if hull.len() >= 3 {
        let inside_hull = |q: DVec2| {
            (0..hull.len()).all(|i| {
                let (a, b) = (hull[i], hull[(i + 1) % hull.len()]);
                cross2(a, b, q) / (b - a).length() > tol
            })
        };
        if e.iter().any(|q| inside_hull(*q)) {
            return true;
        }
    }
    // Proper crossings: each segment's ends are more than `tol` on opposite sides of the other.
    let side = |a: DVec2, b: DVec2, p: DVec2| cross2(a, b, p) / (b - a).length().max(1e-300);
    let opposite = |s1: f64, s2: f64| (s1 > tol && s2 < -tol) || (s1 < -tol && s2 > tol);
    for i in 0..hull_edge_count(hull.len()) {
        let (a, b) = (hull[i], hull[(i + 1) % hull.len()]);
        let mut j = e.len() - 1;
        for k in 0..e.len() {
            let (c, d) = (e[j], e[k]);
            j = k;
            if opposite(side(a, b, c), side(a, b, d)) && opposite(side(c, d, a), side(c, d, b)) {
                return true;
            }
        }
    }
    false
}

/// Fit of the item with transform `t` in the build volume: height, footprint hull vs the bed
/// polygon (non-convex beds handled, including hull edges that cross a notch) and the
/// keep-out areas. `max_outside_xy` is the largest distance from a hull point to the bed;
/// along hull edges that cross a notch it is found by sampling plus golden-section search.
///
/// # Errors
/// [`AmError::Cancelled`] if `ctx` is cancelled between passes.
pub fn check_fit(
    m: &IndexedMesh,
    t: &Transform,
    v: &BuildVolume,
    ctx: &JobCtx,
) -> Result<FitReport, AmError> {
    ctx.checkpoint()?;
    let (lo, hi) = bounds(&m.positions, &t.matrix3, t.translation);
    let aabb = if lo.x <= hi.x {
        Aabb3 { min: lo, max: hi }
    } else {
        Aabb3::EMPTY
    };
    ctx.checkpoint()?;
    let hull = footprint_hull(m, t);
    ctx.checkpoint()?;
    let excess = if aabb.is_empty() {
        0.0
    } else {
        (hi.z - v.height).max(0.0)
    };
    let height_excess = if excess <= FIT_TOLERANCE { 0.0 } else { excess };
    let poly = v.bed_polygon();
    let outside = outside_bed(v, &poly, &hull);
    let max_outside_xy = if outside <= FIT_TOLERANCE {
        0.0
    } else {
        outside
    };
    let hits_exclude = v.exclude.iter().any(|e| hull_hits_polygon(&hull, e));
    Ok(FitReport {
        fits: height_excess == 0.0 && max_outside_xy == 0.0 && !hits_exclude,
        height_excess,
        max_outside_xy,
        hits_exclude,
        footprint_hull: hull,
        aabb,
    })
}

// ------------------------------------------------------------------------- overlap

/// Distance between two disjoint convex polygons (vertex-to-edge, both ways).
fn polygon_distance(a: &[DVec2], b: &[DVec2]) -> f64 {
    let one_way = |p: &[DVec2], q: &[DVec2]| {
        let mut best = f64::INFINITY;
        for &x in p {
            let mut j = q.len() - 1;
            for i in 0..q.len() {
                best = best.min(dist_to_segment(x, q[j], q[i]));
                j = i;
            }
        }
        best
    };
    one_way(a, b).min(one_way(b, a))
}

/// SAT on CCW convex hulls: true if the interiors intersect (deeper than `-gap` when
/// `gap < 0`) or the hulls are closer than `gap`. Touching hulls do not overlap at `gap = 0`.
/// Hulls with fewer than three vertices are compared by distance only.
pub fn hulls_overlap(a: &[DVec2], b: &[DVec2], gap: f64) -> bool {
    if a.is_empty() || b.is_empty() {
        return false;
    }
    if a.len() < 3 || b.len() < 3 {
        return gap > 0.0 && polygon_distance(a, b) < gap;
    }
    let range = |poly: &[DVec2], n: DVec2| {
        poly.iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                let d = n.dot(*p);
                (lo.min(d), hi.max(d))
            })
    };
    let mut sep = f64::NEG_INFINITY;
    for poly in [a, b] {
        for i in 0..poly.len() {
            let e = poly[(i + 1) % poly.len()] - poly[i];
            if e.length_squared() == 0.0 {
                continue;
            }
            let n = DVec2::new(e.y, -e.x).normalize();
            let (alo, ahi) = range(a, n);
            let (blo, bhi) = range(b, n);
            sep = sep.max((blo - ahi).max(alo - bhi));
        }
    }
    if sep == f64::NEG_INFINITY {
        return false;
    }
    if sep < 0.0 {
        return sep < gap;
    }
    if sep >= gap {
        return false;
    }
    polygon_distance(a, b) < gap
}

// ------------------------------------------------------------------------ geometry

/// Display geometry of a build volume (handed to the renderer as non-pickable helpers).
#[derive(Clone, Debug)]
pub struct BuildVolumeGeometry {
    /// Thin slab ([`PLATE_THICKNESS`]), top face at z = 0.
    pub plate: IndexedMesh,
    /// Grid lines on z = 0 every `grid_step` mm, at multiples of the step, clipped to the bed.
    pub grid: Vec<Polyline3>,
    /// Volume edges: bed outline at z = 0 and z = height plus vertical edges at the corners
    /// (four at the quadrant points for a circular bed).
    pub outline: Vec<Polyline3>,
    /// One thin slab ([`EXCLUDE_THICKNESS`], bottom at z = 0) per keep-out polygon.
    pub exclude: Vec<IndexedMesh>,
}

fn signed_area(poly: &[DVec2]) -> f64 {
    let mut s = 0.0;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        s += poly[j].perp_dot(poly[i]);
        j = i;
    }
    s * 0.5
}

/// Ear clipping of a simple CCW polygon; indices into `poly`.
fn triangulate(poly: &[DVec2]) -> Vec<[u32; 3]> {
    let mut idx: Vec<usize> = (0..poly.len()).collect();
    let mut out = Vec::with_capacity(poly.len().saturating_sub(2));
    let in_tri = |p: DVec2, a: DVec2, b: DVec2, c: DVec2| {
        cross2(a, b, p) >= 0.0 && cross2(b, c, p) >= 0.0 && cross2(c, a, p) >= 0.0
    };
    while idx.len() > 3 {
        let n = idx.len();
        let ear = (0..n).find(|&i| {
            let (ia, ib, ic) = (idx[(i + n - 1) % n], idx[i], idx[(i + 1) % n]);
            let (a, b, c) = (poly[ia], poly[ib], poly[ic]);
            cross2(a, b, c) > 1e-12
                && !idx.iter().any(|&j| {
                    let p = poly[j];
                    j != ia
                        && j != ib
                        && j != ic
                        && p != a
                        && p != b
                        && p != c
                        && in_tri(p, a, b, c)
                })
        });
        let i = ear.unwrap_or(0); // degenerate input: clip anyway to terminate
        let (ia, ib, ic) = (idx[(i + n - 1) % n], idx[i], idx[(i + 1) % n]);
        out.push([ia as u32, ib as u32, ic as u32]);
        idx.remove(i);
    }
    if idx.len() == 3 {
        out.push([idx[0] as u32, idx[1] as u32, idx[2] as u32]);
    }
    out
}

fn empty_mesh() -> IndexedMesh {
    IndexedMesh {
        positions: Vec::new(),
        tris: Vec::new(),
        attrs: MeshAttrs::default(),
        groups: FaceGroups::new(0),
    }
}

/// Closed prism of `poly` between `z0` and `z1`; empty for a degenerate polygon.
fn slab(poly: &[DVec2], z0: f64, z1: f64) -> IndexedMesh {
    if poly.len() < 3 || signed_area(poly) == 0.0 {
        return empty_mesh();
    }
    let mut ring: Vec<DVec2> = poly.to_vec();
    if signed_area(&ring) < 0.0 {
        ring.reverse();
    }
    let n = ring.len() as u32;
    let mut positions: Vec<DVec3> = ring.iter().map(|p| p.extend(z0)).collect();
    positions.extend(ring.iter().map(|p| p.extend(z1)));
    let mut tris = Vec::new();
    for [a, b, c] in triangulate(&ring) {
        tris.push([n + a, n + b, n + c]);
        tris.push([a, c, b]);
    }
    for i in 0..n {
        let j = (i + 1) % n;
        tris.push([i, j, n + j]);
        tris.push([i, n + j, n + i]);
    }
    let faces = tris.len();
    IndexedMesh {
        positions,
        tris,
        attrs: MeshAttrs::default(),
        groups: FaceGroups::new(faces),
    }
}

type Axis = fn(DVec2) -> f64;

/// Chords of the bed along the line `x = c` (`y = c` when `vertical` is false), as ranges of
/// the other coordinate.
fn bed_chords(v: &BuildVolume, poly: &[DVec2], c: f64, vertical: bool) -> Vec<(f64, f64)> {
    let (u, w): (Axis, Axis) = if vertical {
        (|p| p.x, |p| p.y)
    } else {
        (|p| p.y, |p| p.x)
    };
    let mut hits: Vec<f64> = Vec::new();
    if let BedShape::Circle { diameter } = v.bed {
        let r = diameter * 0.5;
        let d = c - r;
        if d.abs() <= r {
            let h = (r * r - d * d).sqrt();
            hits = vec![r - h, r + h];
        }
    } else {
        // The crossing rule is half-open, so a line on the far boundary finds nothing; retry
        // just inside it.
        let nudge = 1e-9 * (1.0 + c.abs());
        for cc in [c, c - nudge, c + nudge] {
            let mut j = poly.len() - 1;
            for i in 0..poly.len() {
                let (a, b) = (poly[j], poly[i]);
                j = i;
                if (u(a) > cc) != (u(b) > cc) {
                    hits.push(w(a) + (cc - u(a)) / (u(b) - u(a)) * (w(b) - w(a)));
                }
            }
            if !hits.is_empty() {
                break;
            }
        }
        hits.sort_unstable_by(f64::total_cmp);
    }
    hits.chunks_exact(2).map(|p| (p[0], p[1])).collect()
}

/// Plate slab, grid, volume edges and keep-out slabs for display.
/// `grid_step` ≤ 0 or non-finite gives no grid.
pub fn build_volume_geometry(v: &BuildVolume, grid_step: f64) -> BuildVolumeGeometry {
    const MAX_LINES: f64 = 100_000.0;
    let poly = v.bed_polygon();
    let plate = slab(&poly, -PLATE_THICKNESS, 0.0);
    let exclude = v
        .exclude
        .iter()
        .map(|e| slab(e, 0.0, EXCLUDE_THICKNESS))
        .collect();

    let (lo, hi) = v.bed_rect();
    let mut grid = Vec::new();
    if grid_step.is_finite()
        && grid_step > 0.0
        && poly.len() >= 3
        && ((hi.x - lo.x) + (hi.y - lo.y)) / grid_step <= MAX_LINES
    {
        for vertical in [true, false] {
            let (a, b) = if vertical { (lo.x, hi.x) } else { (lo.y, hi.y) };
            let mut k = (a / grid_step).ceil();
            while k * grid_step <= b {
                let c = k * grid_step;
                for (s, e) in bed_chords(v, &poly, c, vertical) {
                    let pt = |w: f64| {
                        if vertical {
                            DVec3::new(c, w, 0.0)
                        } else {
                            DVec3::new(w, c, 0.0)
                        }
                    };
                    grid.push(Polyline3 {
                        points: vec![pt(s), pt(e)],
                        closed: false,
                    });
                }
                k += 1.0;
            }
        }
    }

    let mut outline = Vec::new();
    if poly.len() >= 3 {
        for z in [0.0, v.height] {
            outline.push(Polyline3 {
                points: poly.iter().map(|p| p.extend(z)).collect(),
                closed: true,
            });
        }
        let corners: Vec<DVec2> = match v.bed {
            BedShape::Circle { diameter } => {
                let r = diameter * 0.5;
                vec![
                    DVec2::new(0.0, r),
                    DVec2::new(r, 0.0),
                    DVec2::new(diameter, r),
                    DVec2::new(r, diameter),
                ]
            }
            _ => poly.clone(),
        };
        for p in corners {
            outline.push(Polyline3 {
                points: vec![p.extend(0.0), p.extend(v.height)],
                closed: false,
            });
        }
    }
    BuildVolumeGeometry {
        plate,
        grid,
        outline,
        exclude,
    }
}
