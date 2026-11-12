//! Overhang analysis (S16 AM-02): per-face classes, metrics, islands, stability and the
//! support-volume estimate. Definitions are in `docs/notes/am-build-prep.md` §§1–3.
//!
//! - [`classify_faces`] is the O(F) pass used for coarse live updates (no rays).
//! - [`analyze_overhang`] adds the metrics, islands, stability and, when
//!   [`OverhangOpts::support`] is set, the support estimate: the column method on closed
//!   meshes (note §3.1) and the per-face centroid rays on open ones (note §3.2).
//! - [`shader_params`] gives the uniforms the S10 overhang shader needs; the shared golden file
//!   `tests/golden/am/overhang_classes.toml` checks that the shader and [`classify_face`]
//!   agree.
//!
//! Every result is a pure function of (mesh, transform, options): sums are taken per fixed
//! chunk and added in order, so they are bit-identical for any rayon thread count.
//!
//! Comparisons at a threshold use tiny guards so that a face exactly at the critical angle (up
//! to rounding) is self-supporting, as the strict rule `c > cos α` intends: `c` must exceed
//! `cos α` by more than [`ANGLE_GUARD`], and a contact face may lie up to [`LINEAR_GUARD`]
//! above the tolerance.

use std::f64::consts::FRAC_PI_2;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use amg_core::glam::{DVec2, DVec3};
use amg_core::{Dir3, Ray3, Transform};
use amg_jobs::JobCtx;
use amg_mesh::{FaceId, IndexedMesh, MeshSpatial, RayHit, VertexId};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::AmError;
use crate::machine::Machine;
use crate::place::{dist_to_boundary, monotone_chain, point_in_polygon, signed_area};

/// Guard on the cosine test: a face is `Overhang` only if `c > cos α + ANGLE_GUARD`.
pub const ANGLE_GUARD: f64 = 1e-12;
/// Linear guard, mm: contact slab, island heights and coincident ray hits.
pub const LINEAR_GUARD: f64 = 1e-9;
/// |c| at or below this counts as a vertical wall for the ray walk (neither entry nor exit).
const FACING_EPS: f64 = 1e-12;
/// Faces per parallel chunk; sums are taken per chunk and then added in order.
const CHUNK: usize = 8192;
/// Offset of the centroid-fallback ray origin below its face, mm.
const RAY_OFFSET: f64 = 1e-6;

/// User-facing analysis settings stored in a print setup.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AnalysisSettings {
    /// Critical overhang angle α, rad.
    pub critical_angle: f64,
    /// Contact tolerance, mm; `None` → [`Machine::default_contact_tolerance`].
    pub contact_tolerance: Option<f64>,
    /// Support-estimate cell size, mm.
    pub support_cell: f64,
    /// Maximum support-estimate ray count.
    pub max_rays: u32,
}

impl AnalysisSettings {
    /// Defaults for `m`: its critical angle, automatic tolerance, 0.5 mm cells, 262 144 rays.
    pub fn for_machine(m: &Machine) -> Self {
        AnalysisSettings {
            critical_angle: m.default_critical_angle,
            contact_tolerance: None,
            support_cell: 0.5,
            max_rays: 262_144,
        }
    }
}

/// Class of one face (note §1.2), evaluated in the order Degenerate, Contact, Overhang.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum FaceClass {
    /// Needs no support: upward faces, walls and self-supporting slopes.
    Supported = 0,
    /// Downward face steeper than the critical angle (`c > cos α`).
    Overhang = 1,
    /// Downward face inside the contact slab above the item's lowest point.
    Contact = 2,
    /// Zero-area face (no normal).
    Degenerate = 3,
}

/// Resolved options of one analysis.
#[derive(Clone, Debug)]
pub struct OverhangOpts {
    /// Critical angle α, rad, in (0, π/2].
    pub critical_angle: f64,
    /// Contact slab height above the item's lowest point, mm (resolved, ≥ 0).
    pub contact_tolerance: f64,
    /// `None` skips the rays (classification and area metrics only).
    pub support: Option<SupportOpts>,
}

impl OverhangOpts {
    /// Options from a print setup's settings for machine `m` (tolerance resolved), with the
    /// support estimate on.
    pub fn from_settings(s: &AnalysisSettings, m: &Machine) -> Self {
        OverhangOpts {
            critical_angle: s.critical_angle,
            contact_tolerance: s
                .contact_tolerance
                .unwrap_or_else(|| m.default_contact_tolerance()),
            support: Some(SupportOpts {
                cell: s.support_cell,
                max_rays: s.max_rays,
            }),
        }
    }

    fn validate(&self) -> Result<(), AmError> {
        let bad = |s: &str| Err(AmError::InvalidOption(s.to_owned()));
        if !(self.critical_angle > 0.0 && self.critical_angle <= FRAC_PI_2) {
            return bad("critical_angle must be in (0, π/2]");
        }
        if !(self.contact_tolerance >= 0.0 && self.contact_tolerance.is_finite()) {
            return bad("contact_tolerance must be finite and ≥ 0");
        }
        if let Some(s) = &self.support {
            if !(s.cell > 0.0 && s.cell.is_finite()) {
                return bad("support cell must be finite and > 0");
            }
            if s.max_rays == 0 {
                return bad("support max_rays must be ≥ 1");
            }
        }
        Ok(())
    }
}

/// Support-estimate resolution.
#[derive(Clone, Copy, Debug)]
pub struct SupportOpts {
    /// Requested grid cell, mm; grows so that the ray count stays ≤ `max_rays`.
    pub cell: f64,
    /// Ray budget.
    pub max_rays: u32,
}

/// Support-volume estimate (note §3).
#[derive(Clone, Debug, PartialEq)]
pub struct SupportEstimate {
    /// Total support volume, mm³.
    pub volume: f64,
    /// Part standing on the plate, mm³.
    pub on_plate_volume: f64,
    /// Part standing on the model, mm³.
    pub on_part_volume: f64,
    /// Columns: area of the cells with any counted gap. Face centroids: projected area of the
    /// counted faces. mm².
    pub footprint_area: f64,
    /// Cell actually used (≥ requested), mm.
    pub cell: f64,
    /// Rays cast.
    pub rays: u32,
    /// How it was computed.
    pub method: SupportMethod,
}

/// Support-estimate method.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SupportMethod {
    /// Exact per grid column (closed meshes).
    Columns,
    /// One ray per overhang face (sub)sample (open meshes); approximate.
    FaceCentroids,
}

/// Metrics of one item at one transform (note §2).
#[derive(Clone, Debug, PartialEq)]
pub struct OverhangMetrics {
    /// Σ area of `Overhang` faces, mm².
    pub overhang_area: f64,
    /// Σ area · c over `Overhang` faces (shadow on the plate), mm².
    pub overhang_projected_area: f64,
    /// Σ area · c over `Contact` faces, mm².
    pub contact_area: f64,
    /// Area of the convex hull of all vertices projected to the plate, mm².
    pub footprint_area: f64,
    /// z extent of the part (excl. elevation), mm.
    pub height: f64,
    /// Number of island vertices.
    pub island_count: usize,
    /// Centroid over the contact hull; `None` without contact or for an open mesh.
    pub stable: Option<bool>,
    /// Signed distance of the centroid to the contact hull boundary, mm (negative = tips over).
    pub com_margin: Option<f64>,
    /// Support estimate, if requested.
    pub support: Option<SupportEstimate>,
    /// The mesh is not closed and oriented; the support estimate (if any) is approximate.
    pub approximate: bool,
}

/// Full result of [`analyze_overhang`].
#[derive(Clone, Debug)]
pub struct OverhangReport {
    /// The transform analysed (body-local → plate).
    pub transform: Transform,
    /// Class per face; `len == faces`.
    pub face_class: Vec<FaceClass>,
    /// Down angle ψ per face, rad in [0, π] (π/2 for degenerate faces).
    pub down_angle: Vec<f64>,
    /// Island vertices (note §1.2), sorted.
    pub islands: Vec<VertexId>,
    /// Lowest plate z of the item, mm (for [`shader_params`]).
    pub z_min: f64,
    /// Metrics.
    pub metrics: OverhangMetrics,
}

/// Uniforms for the S10 overhang shader, in body-local space (f32 conversion happens in
/// amg-ui). A face is contact if `dot(contact_plane.0, p) + contact_plane.1 ≤
/// contact_tolerance` at its highest corner `p` (the plate height above the item's minimum).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OverhangShaderParams {
    /// Build direction `d_local = R⁻¹·Z`, unit.
    pub build_dir_local: DVec3,
    /// cos α.
    pub cos_critical: f64,
    /// Plane `(n, w)` with `n·p + w` = height of body-local `p` above the item's minimum.
    pub contact_plane: (DVec3, f64),
    /// Contact slab height, mm.
    pub contact_tolerance: f64,
}

// ------------------------------------------------------------------------ classification

/// Plate height of body-local points: `z = row·p + tz`, plus the unit build direction.
#[derive(Clone, Copy)]
struct Frame {
    row: DVec3,
    tz: f64,
    d: DVec3,
}

impl Frame {
    fn new(t: &Transform) -> Self {
        let m = t.matrix3;
        let row = DVec3::new(m.x_axis.z, m.y_axis.z, m.z_axis.z);
        Frame {
            row,
            tz: t.translation.z,
            d: row.normalize_or_zero(),
        }
    }

    #[inline]
    fn z(&self, p: DVec3) -> f64 {
        self.row.dot(p) + self.tz
    }
}

/// Class of one face from its unit normal (body-local, `ZERO` if degenerate), the unit build
/// direction in the same frame, `cos α`, the face's highest point above the item's lowest
/// point and the contact tolerance. This is the rule the S10 shader must reproduce.
pub fn classify_face(
    normal: DVec3,
    build_dir: DVec3,
    cos_critical: f64,
    height: f64,
    contact_tolerance: f64,
) -> FaceClass {
    if normal.length_squared() <= 0.0 || normal.is_nan() {
        return FaceClass::Degenerate;
    }
    let c = -normal.dot(build_dir);
    if c > 0.0 && height <= contact_tolerance + LINEAR_GUARD {
        FaceClass::Contact
    } else if c > cos_critical + ANGLE_GUARD {
        FaceClass::Overhang
    } else {
        FaceClass::Supported
    }
}

/// Down angle ψ of a unit normal: `acos(clamp(−n·d))`; π/2 for a zero normal.
fn down_angle_of(normal: DVec3, d: DVec3) -> f64 {
    if normal.length_squared() <= 0.0 || normal.is_nan() {
        return FRAC_PI_2;
    }
    (-normal.dot(d)).clamp(-1.0, 1.0).acos()
}

/// Unit normal as `IndexedMesh::face_normal` computes it, from the corner indices.
#[inline]
fn normal_of(m: &IndexedMesh, t: &[u32; 3]) -> (DVec3, f64) {
    let [a, b, c] = t.map(|i| m.positions[i as usize]);
    let raw = (b - a).cross(c - a);
    (raw.normalize_or_zero(), 0.5 * raw.length())
}

struct Classified {
    class: Vec<FaceClass>,
    psi: Vec<f64>,
    z: Vec<f64>,
    z_min: f64,
    z_max: f64,
}

fn min_max(z: &[f64]) -> (f64, f64) {
    let fold = |c: &[f64]| {
        c.iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &v| {
                (lo.min(v), hi.max(v))
            })
    };
    let (lo, hi) = z.par_chunks(CHUNK).map(fold).reduce(
        || (f64::INFINITY, f64::NEG_INFINITY),
        |a, b| (a.0.min(b.0), a.1.max(b.1)),
    );
    if lo <= hi { (lo, hi) } else { (0.0, 0.0) }
}

fn classify_inner(
    m: &IndexedMesh,
    t: &Transform,
    opts: &OverhangOpts,
    ctx: &JobCtx,
) -> Result<Classified, AmError> {
    opts.validate()?;
    let fr = Frame::new(t);
    let z: Vec<f64> = m.positions.par_iter().map(|&p| fr.z(p)).collect();
    let (z_min, z_max) = min_max(&z);
    ctx.checkpoint()?;
    let cos_a = opts.critical_angle.cos();
    let tol = opts.contact_tolerance;
    let chunks: Vec<(Vec<FaceClass>, Vec<f64>)> = m
        .tris
        .par_chunks(CHUNK)
        .map(|chunk| {
            ctx.checkpoint()?;
            let mut cls = Vec::with_capacity(chunk.len());
            let mut psi = Vec::with_capacity(chunk.len());
            for tri in chunk {
                let (n, _) = normal_of(m, tri);
                let h = tri
                    .iter()
                    .map(|&i| z[i as usize])
                    .fold(f64::NEG_INFINITY, f64::max)
                    - z_min;
                cls.push(classify_face(n, fr.d, cos_a, h, tol));
                psi.push(down_angle_of(n, fr.d));
            }
            Ok((cls, psi))
        })
        .collect::<Result<_, amg_core::Cancelled>>()?;
    let mut class = Vec::with_capacity(m.tris.len());
    let mut psi = Vec::with_capacity(m.tris.len());
    for (c, p) in chunks {
        class.extend_from_slice(&c);
        psi.extend_from_slice(&p);
    }
    Ok(Classified {
        class,
        psi,
        z,
        z_min,
        z_max,
    })
}

/// Classification only: O(F), parallel, no rays. Used for coarse live updates. Returns the
/// class and the down angle ψ (rad) per face.
///
/// # Errors
/// [`AmError::InvalidOption`] for out-of-range options, [`AmError::Cancelled`].
pub fn classify_faces(
    m: &IndexedMesh,
    t: &Transform,
    opts: &OverhangOpts,
    ctx: &JobCtx,
) -> Result<(Vec<FaceClass>, Vec<f64>), AmError> {
    let c = classify_inner(m, t, opts, ctx)?;
    Ok((c.class, c.psi))
}

// ------------------------------------------------------------------------- severity

/// Severity of one face from its down angle: `clamp((cos ψ − cos α)/(1 − cos α), 0, 1)`
/// (0 = fine, 1 = horizontal ceiling).
pub fn severity(down_angle: f64, critical_angle: f64) -> f64 {
    let ca = critical_angle.cos();
    let den = 1.0 - ca;
    if den <= 0.0 || den.is_nan() {
        return 0.0;
    }
    ((down_angle.cos() - ca) / den).clamp(0.0, 1.0)
}

/// Area-weighted per-vertex severity ∈ [0, 1] for RND-08 per-vertex overlays: the mean of the
/// incident faces' [`severity`], weighted by face area. Only `Overhang` faces contribute a
/// non-zero severity (contact faces are 0). Unreferenced vertices get 0.
pub fn vertex_severity(m: &IndexedMesh, r: &OverhangReport, critical_angle: f64) -> Vec<f64> {
    let nv = m.positions.len();
    let mut num = vec![0.0; nv];
    let mut den = vec![0.0; nv];
    for (f, tri) in m.tris.iter().enumerate() {
        let (_, area) = normal_of(m, tri);
        let s = match r.face_class.get(f) {
            Some(FaceClass::Overhang) => severity(r.down_angle[f], critical_angle),
            _ => 0.0,
        };
        for &v in tri {
            num[v as usize] += area * s;
            den[v as usize] += area;
        }
    }
    num.iter()
        .zip(&den)
        .map(|(&n, &d)| {
            if d > 0.0 {
                (n / d).clamp(0.0, 1.0)
            } else {
                0.0
            }
        })
        .collect()
}

/// Uniforms for the S10 overhang shader, in body-local space. `z_min` is the item's lowest
/// plate z ([`OverhangReport::z_min`], or the elevation for an on-plate placement).
pub fn shader_params(t: &Transform, z_min: f64, opts: &OverhangOpts) -> OverhangShaderParams {
    let fr = Frame::new(t);
    let len = fr.row.length();
    let (n, w) = if len > 0.0 {
        (fr.row / len, (fr.tz - z_min) / len)
    } else {
        (DVec3::ZERO, 0.0)
    };
    OverhangShaderParams {
        build_dir_local: n,
        cos_critical: opts.critical_angle.cos(),
        contact_plane: (n, w),
        contact_tolerance: opts.contact_tolerance,
    }
}

// ------------------------------------------------------------------------- analysis

/// Full analysis; `s` is the body's `MeshSpatial` (body-local, MESH-03). Rays run in
/// body-local space, so rotating an item never rebuilds the BVH.
///
/// # Errors
/// [`AmError::InvalidOption`] for out-of-range options, [`AmError::Cancelled`].
pub fn analyze_overhang(
    s: &MeshSpatial,
    t: &Transform,
    opts: &OverhangOpts,
    ctx: &JobCtx,
) -> Result<OverhangReport, AmError> {
    let m: &IndexedMesh = s.mesh();
    let cl = classify_inner(m, t, opts, ctx)?;
    ctx.progress(0.1);
    let fr = Frame::new(t);
    let sums = face_sums(m, &cl.class, fr.d, ctx)?;
    let closed = sums.vol6 > 0.0 && closed_oriented(m);
    ctx.checkpoint()?;
    let islands = islands(m, &cl.z, cl.z_min, opts.contact_tolerance);
    let hull = plate_hull(m, t);
    let footprint_area = if hull.len() >= 3 {
        signed_area(&hull).abs()
    } else {
        0.0
    };
    ctx.progress(0.2);

    let (stable, com_margin) = if closed && cl.z_min <= opts.contact_tolerance + LINEAR_GUARD {
        let centroid = t.transform_point3(sums.centroid(m)).truncate();
        let tol = opts.contact_tolerance + LINEAR_GUARD;
        let pts: Vec<DVec2> =
            cl.z.par_iter()
                .enumerate()
                .filter(|&(_, &z)| z - cl.z_min <= tol)
                .map(|(i, _)| t.transform_point3(m.positions[i]).truncate())
                .collect();
        stability(centroid, pts)
    } else {
        (None, None)
    };
    ctx.checkpoint()?;

    let support = match &opts.support {
        None => None,
        Some(so) if closed => Some(support_columns(s, t, &fr, &cl, so, ctx)?),
        Some(so) => Some(support_centroids(s, &fr, &cl, so, ctx)?),
    };
    ctx.progress(1.0);

    Ok(OverhangReport {
        transform: *t,
        metrics: OverhangMetrics {
            overhang_area: sums.overhang_area,
            overhang_projected_area: sums.overhang_projected,
            contact_area: sums.contact_area,
            footprint_area,
            height: cl.z_max - cl.z_min,
            island_count: islands.len(),
            stable,
            com_margin,
            support,
            approximate: !closed,
        },
        face_class: cl.class,
        down_angle: cl.psi,
        islands,
        z_min: cl.z_min,
    })
}

/// Per-face sums: areas and the volume moments (about the first vertex).
#[derive(Clone, Copy, Default)]
struct Sums {
    overhang_area: f64,
    overhang_projected: f64,
    contact_area: f64,
    vol6: f64,
    moment: DVec3,
}

impl Sums {
    fn add(mut self, o: Sums) -> Sums {
        self.overhang_area += o.overhang_area;
        self.overhang_projected += o.overhang_projected;
        self.contact_area += o.contact_area;
        self.vol6 += o.vol6;
        self.moment += o.moment;
        self
    }

    /// Volume centroid, body-local (meaningful for closed meshes with `vol6 > 0`).
    fn centroid(&self, m: &IndexedMesh) -> DVec3 {
        let o = m.positions.first().copied().unwrap_or(DVec3::ZERO);
        if self.vol6 > 0.0 {
            o + self.moment / (4.0 * self.vol6)
        } else {
            o
        }
    }
}

fn face_sums(
    m: &IndexedMesh,
    class: &[FaceClass],
    d: DVec3,
    ctx: &JobCtx,
) -> Result<Sums, AmError> {
    let o = m.positions.first().copied().unwrap_or(DVec3::ZERO);
    let parts: Vec<Sums> = m
        .tris
        .par_chunks(CHUNK)
        .enumerate()
        .map(|(ci, chunk)| {
            ctx.checkpoint()?;
            let mut s = Sums::default();
            for (k, tri) in chunk.iter().enumerate() {
                let f = ci * CHUNK + k;
                let (n, area) = normal_of(m, tri);
                let c = -n.dot(d);
                match class[f] {
                    FaceClass::Overhang => {
                        s.overhang_area += area;
                        s.overhang_projected += area * c;
                    }
                    FaceClass::Contact => s.contact_area += area * c,
                    FaceClass::Supported | FaceClass::Degenerate => {}
                }
                let [a, b, cc] = tri.map(|i| m.positions[i as usize] - o);
                let v6 = a.dot(b.cross(cc));
                s.vol6 += v6;
                s.moment += v6 * (a + b + cc);
            }
            Ok(s)
        })
        .collect::<Result<_, amg_core::Cancelled>>()?;
    Ok(parts.into_iter().fold(Sums::default(), Sums::add))
}

/// Closed and consistently oriented: every directed edge is unique and has its opposite.
/// Counting sort of the 3·F edges by lower vertex, then a per-vertex check.
/// (Non-manifold vertices are not checked; they do not affect the column walk.)
fn closed_oriented(m: &IndexedMesh) -> bool {
    if m.tris.is_empty() || m.positions.len() >= 1 << 31 || 3 * m.tris.len() >= u32::MAX as usize {
        return !m.tris.is_empty() && closed_oriented_wide(m);
    }
    // Bucket every edge by its lower vertex (counting sort) with the value `max << 1 | (a > b)`.
    // Closed and consistently oriented ⇔ in every bucket each `max` occurs exactly twice, once
    // in each direction (so directed edges are unique and each has its opposite).
    let nv = m.positions.len();
    let ends = |t: &[u32; 3], k: usize| (t[k], t[(k + 1) % 3]);
    let count: Vec<AtomicU32> = (0..nv).into_par_iter().map(|_| AtomicU32::new(0)).collect();
    m.tris.par_iter().for_each(|t| {
        for k in 0..3 {
            let (a, b) = ends(t, k);
            count[a.min(b) as usize].fetch_add(1, Ordering::Relaxed);
        }
    });
    let mut offs = Vec::with_capacity(nv + 1);
    offs.push(0u32);
    for c in &count {
        offs.push(offs[offs.len() - 1] + c.load(Ordering::Relaxed));
    }
    let cursor: Vec<AtomicU32> = offs[..nv].par_iter().map(|&o| AtomicU32::new(o)).collect();
    let vals: Vec<AtomicU32> = (0..3 * m.tris.len())
        .into_par_iter()
        .map(|_| AtomicU32::new(0))
        .collect();
    m.tris.par_iter().for_each(|t| {
        for k in 0..3 {
            let (a, b) = ends(t, k);
            let slot = cursor[a.min(b) as usize].fetch_add(1, Ordering::Relaxed);
            vals[slot as usize].store((a.max(b) << 1) | u32::from(a > b), Ordering::Relaxed);
        }
    });
    (0..nv).into_par_iter().all(|v| {
        let (lo, hi) = (offs[v] as usize, offs[v + 1] as usize);
        let (mut stack, mut heap) = ([0u32; 32], Vec::new());
        let e: &mut [u32] = if hi - lo <= stack.len() {
            &mut stack[..hi - lo]
        } else {
            heap.resize(hi - lo, 0);
            &mut heap
        };
        for (d, x) in e.iter_mut().zip(&vals[lo..hi]) {
            *d = x.load(Ordering::Relaxed);
        }
        e.sort_unstable();
        e.len().is_multiple_of(2)
            && e.chunks(2).enumerate().all(|(i, w)| {
                w[0] >> 1 != v as u32
                    && w[0] & 1 == 0
                    && w[1] == w[0] | 1
                    && e.get(2 * i + 2).is_none_or(|&n| n >> 1 != w[0] >> 1)
            })
    })
}

/// [`closed_oriented`] for meshes with ≥ 2³¹ vertices (directed keys + binary search).
fn closed_oriented_wide(m: &IndexedMesh) -> bool {
    let mut e: Vec<u64> = m
        .tris
        .par_iter()
        .flat_map_iter(|t| (0..3).map(move |k| (u64::from(t[k]) << 32) | u64::from(t[(k + 1) % 3])))
        .collect();
    e.par_sort_unstable();
    if e.par_windows(2).any(|w| w[0] == w[1]) {
        return false;
    }
    e.par_iter().all(|&k| {
        let (a, b) = (k >> 32, k & 0xFFFF_FFFF);
        a != b && e.binary_search(&((b << 32) | a)).is_ok()
    })
}

/// Convex hull of the vertices' plate xy: per-chunk hulls (parallel), then the hull of their
/// union. Same point set as `place::footprint_hull`, without its sequential final pass over
/// every vertex outside the pre-filter octagon.
fn plate_hull(m: &IndexedMesh, t: &Transform) -> Vec<DVec2> {
    let parts: Vec<Vec<DVec2>> = m
        .positions
        .par_chunks(CHUNK)
        .map(|c| {
            monotone_chain(
                c.iter()
                    .map(|&p| t.transform_point3(p).truncate())
                    .filter(|q| q.is_finite())
                    .collect(),
            )
        })
        .collect();
    monotone_chain(parts.concat())
}

/// Island vertices (note §1.2): plateaus of vertices tied within [`LINEAR_GUARD`] along
/// face edges with no lower neighbour, reported by their lowest vertex index, more than the
/// contact tolerance above the item's minimum.
fn islands(m: &IndexedMesh, z: &[f64], z_min: f64, tol: f64) -> Vec<VertexId> {
    let nv = z.len();
    let lower: Vec<AtomicBool> = (0..nv)
        .into_par_iter()
        .map(|_| AtomicBool::new(false))
        .collect();
    let used: Vec<AtomicBool> = (0..nv)
        .into_par_iter()
        .map(|_| AtomicBool::new(false))
        .collect();
    let ties: Vec<Vec<(u32, u32)>> = m
        .tris
        .par_chunks(CHUNK)
        .map(|chunk| {
            let mut ties = Vec::new();
            for t in chunk {
                for k in 0..3 {
                    let (a, b) = (t[k], t[(k + 1) % 3]);
                    used[a as usize].store(true, Ordering::Relaxed);
                    let (za, zb) = (z[a as usize], z[b as usize]);
                    if zb < za - LINEAR_GUARD {
                        lower[a as usize].store(true, Ordering::Relaxed);
                    } else if za < zb - LINEAR_GUARD {
                        lower[b as usize].store(true, Ordering::Relaxed);
                    } else if a != b {
                        ties.push((a.min(b), a.max(b)));
                    }
                }
            }
            ties
        })
        .collect();
    // Union-find over tied edges; the root of a plateau is its lowest vertex index.
    let mut parent: Vec<u32> = (0..nv as u32).collect();
    fn find(p: &mut [u32], mut v: u32) -> u32 {
        while p[v as usize] != v {
            let g = p[p[v as usize] as usize];
            p[v as usize] = g;
            v = g;
        }
        v
    }
    for &(a, b) in ties.iter().flatten() {
        let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
        if ra != rb {
            let (lo, hi) = (ra.min(rb), ra.max(rb));
            parent[hi as usize] = lo;
        }
    }
    let mut plateau_lower = vec![false; nv];
    for v in 0..nv as u32 {
        if lower[v as usize].load(Ordering::Relaxed) {
            let r = find(&mut parent, v);
            plateau_lower[r as usize] = true;
        }
    }
    let mut out = Vec::new();
    for v in 0..nv as u32 {
        let i = v as usize;
        if used[i].load(Ordering::Relaxed)
            && find(&mut parent, v) == v
            && !plateau_lower[i]
            && z[i] - z_min > tol + LINEAR_GUARD
        {
            out.push(VertexId(v));
        }
    }
    out
}

/// Stability of the centroid's plate xy over the hull of the contact points.
fn stability(centroid: DVec2, pts: Vec<DVec2>) -> (Option<bool>, Option<f64>) {
    let hull = monotone_chain(pts);
    match hull.len() {
        0 => (None, None),
        1 => (Some(false), Some(-(centroid - hull[0]).length())),
        2 => (
            Some(false),
            Some(-crate::place::dist_to_segment(centroid, hull[0], hull[1])),
        ),
        _ => {
            let dist = dist_to_boundary(centroid, &hull);
            let margin = if point_in_polygon(centroid, &hull) {
                dist
            } else {
                -dist
            };
            (Some(margin >= 0.0), Some(margin))
        }
    }
}

// ------------------------------------------------------------------------- support

/// Cell size ≥ `cell` such that the grid over `ext` has at most `max_rays` cells.
fn grid(ext: DVec2, cell: f64, max_rays: u32) -> (f64, usize, usize) {
    let max = max_rays.max(1) as usize;
    let mut s = cell.max((ext.x * ext.y / max as f64).sqrt());
    if !(s > 0.0 && s.is_finite()) {
        s = cell;
    }
    loop {
        let nx = ((ext.x / s).ceil() as usize).max(1);
        let ny = ((ext.y / s).ceil() as usize).max(1);
        if nx.saturating_mul(ny) <= max {
            return (s, nx, ny);
        }
        s *= 1.0 + 1.0 / 256.0;
    }
}

/// Plate-xy bounds of the transformed vertices.
fn xy_bounds(m: &IndexedMesh, t: &Transform) -> (DVec2, DVec2) {
    let (lo, hi) = m
        .positions
        .par_chunks(CHUNK)
        .map(|c| {
            c.iter()
                .fold((DVec2::INFINITY, DVec2::NEG_INFINITY), |(lo, hi), &p| {
                    let q = t.transform_point3(p).truncate();
                    (lo.min(q), hi.max(q))
                })
        })
        .reduce(
            || (DVec2::INFINITY, DVec2::NEG_INFINITY),
            |a, b| (a.0.min(b.0), a.1.max(b.1)),
        );
    if lo.x <= hi.x {
        (lo, hi)
    } else {
        (DVec2::ZERO, DVec2::ZERO)
    }
}

/// Signed facing of face `f` against the ray direction `d` (> 0 = entry).
#[inline]
fn facing(m: &IndexedMesh, f: FaceId, d: DVec3) -> f64 {
    -m.face_normal(f).dot(d)
}

/// Up-facing (exit) plate heights of one column's hits, ascending. Coincident hits are
/// grouped; a group with both entry and exit faces (a grazing ray) is skipped, and exits at
/// or below the plate are ignored.
fn column_exits(hits: &[RayHit], z0: f64, m: &IndexedMesh, d: DVec3, out: &mut Vec<f64>) {
    let mut i = 0;
    while i < hits.len() {
        let t0 = hits[i].t;
        let (mut entry, mut exit) = (false, false);
        let mut j = i;
        while j < hits.len() && hits[j].t - t0 <= LINEAR_GUARD {
            let c = facing(m, hits[j].face, d);
            if c > FACING_EPS {
                entry = true;
            } else if c < -FACING_EPS {
                exit = true;
            }
            j += 1;
        }
        let z = z0 + t0;
        if exit && !entry && z > LINEAR_GUARD {
            out.push(z);
        }
        i = j;
    }
}

/// Sutherland–Hodgman clip of a convex polygon to `p[axis] ≥ v` (`keep_ge`) or `≤ v`.
fn clip(poly: &[DVec2], axis: usize, v: f64, keep_ge: bool, out: &mut Vec<DVec2>) {
    out.clear();
    let inside = |p: DVec2| if keep_ge { p[axis] >= v } else { p[axis] <= v };
    for k in 0..poly.len() {
        let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
        let (ia, ib) = (inside(a), inside(b));
        if ia {
            out.push(a);
        }
        if ia != ib {
            let t = (v - a[axis]) / (b[axis] - a[axis]);
            out.push(a + (b - a) * t);
        }
    }
}

/// Area (≥ 0) and centroid of a simple polygon of either orientation.
fn area_centroid(p: &[DVec2]) -> (f64, DVec2) {
    let o = p[0];
    let (mut a2, mut c) = (0.0, DVec2::ZERO);
    for k in 1..p.len() - 1 {
        let (u, v) = (p[k] - o, p[k + 1] - o);
        let cr = u.perp_dot(v);
        a2 += cr;
        c += cr * (u + v);
    }
    if a2.abs() < f64::MIN_POSITIVE {
        return (0.0, o);
    }
    (0.5 * a2.abs(), o + c / (3.0 * a2))
}

/// Column support estimate (note §3.1, refined in AM-02). One ray per grid cell centre
/// along +d records the up-facing (exit) surfaces of that column. In a closed, oriented mesh
/// every downward face is an entry, so the gap under an `Overhang` face at (x, y) is its height
/// minus the highest exit below it (or the plate). That gap is integrated exactly over the
/// part of the face inside each cell (clipped area × height at the clipped centroid, exact for
/// a planar face over a constant base); only the base is sampled per cell. Pure centre
/// sampling would quantise the overhang boundary itself, which does not converge smoothly.
fn support_columns(
    s: &MeshSpatial,
    t: &Transform,
    fr: &Frame,
    cl: &Classified,
    so: &SupportOpts,
    ctx: &JobCtx,
) -> Result<SupportEstimate, AmError> {
    let m: &IndexedMesh = s.mesh();
    let (lo, hi) = xy_bounds(m, t);
    let (cell, nx, ny) = grid(hi - lo, so.cell, so.max_rays);
    let inv = t.inverse();
    let dir = Dir3::new_unchecked(fr.d);
    let z0 = cl.z_min.min(0.0) - 1.0;
    let t_max = cl.z_max - z0 + 1.0;

    // Exit heights per cell (CSR, row-major).
    let rows: Vec<(Vec<u32>, Vec<f64>)> = (0..ny)
        .into_par_iter()
        .map(|j| {
            ctx.checkpoint()?;
            let mut hits = Vec::new();
            let (mut counts, mut zs) = (Vec::with_capacity(nx), Vec::new());
            let y = lo.y + (j as f64 + 0.5) * cell;
            for i in 0..nx {
                let x = lo.x + (i as f64 + 0.5) * cell;
                let origin = inv.transform_point3(DVec3::new(x, y, z0));
                hits.clear();
                s.ray_cast_all(&Ray3 { origin, dir }, t_max, &mut hits);
                let before = zs.len();
                column_exits(&hits, z0, m, fr.d, &mut zs);
                counts.push(u32::try_from(zs.len() - before).unwrap_or(u32::MAX));
            }
            Ok((counts, zs))
        })
        .collect::<Result<_, amg_core::Cancelled>>()?;
    let mut offs = Vec::with_capacity(nx * ny + 1);
    offs.push(0usize);
    let mut exits = Vec::new();
    for (counts, zs) in rows {
        for c in counts {
            let last = offs[offs.len() - 1];
            offs.push(last + c as usize);
        }
        exits.extend_from_slice(&zs);
    }
    ctx.progress(0.7);

    let faces: Vec<u32> = (0..cl.class.len() as u32)
        .into_par_iter()
        .filter(|&f| cl.class[f as usize] == FaceClass::Overhang)
        .collect();
    let parts: Vec<(f64, f64, Vec<u32>)> = faces
        .par_chunks(1024)
        .map(|chunk| {
            ctx.checkpoint()?;
            let (mut vol, mut part, mut cells) = (0.0, 0.0, Vec::new());
            let (mut buf_a, mut buf_b) = (Vec::with_capacity(8), Vec::with_capacity(8));
            for &f in chunk {
                let p = m.tris[f as usize].map(|i| t.transform_point3(m.positions[i as usize]));
                let q = p.map(|v| v.truncate());
                let (e1, e2) = (q[1] - q[0], q[2] - q[0]);
                let det = e1.perp_dot(e2);
                if det.abs() < f64::MIN_POSITIVE {
                    continue;
                }
                let (dz1, dz2) = (p[1].z - p[0].z, p[2].z - p[0].z);
                let a = (dz1 * e2.y - dz2 * e1.y) / det;
                let b = (e1.x * dz2 - e2.x * dz1) / det;
                let c = p[0].z - a * q[0].x - b * q[0].y;
                let bmin = q[0].min(q[1]).min(q[2]);
                let bmax = q[0].max(q[1]).max(q[2]);
                let idx = |v: f64, lo: f64, n: usize| {
                    (((v - lo) / cell).floor().max(0.0) as usize).min(n - 1)
                };
                let (i0, i1) = (idx(bmin.x, lo.x, nx), idx(bmax.x, lo.x, nx));
                let (j0, j1) = (idx(bmin.y, lo.y, ny), idx(bmax.y, lo.y, ny));
                for j in j0..=j1 {
                    let y0 = lo.y + j as f64 * cell;
                    for i in i0..=i1 {
                        let x0 = lo.x + i as f64 * cell;
                        buf_a.clear();
                        buf_a.extend_from_slice(&q);
                        clip(&buf_a, 0, x0, true, &mut buf_b);
                        clip(&buf_b, 0, x0 + cell, false, &mut buf_a);
                        clip(&buf_a, 1, y0, true, &mut buf_b);
                        clip(&buf_b, 1, y0 + cell, false, &mut buf_a);
                        if buf_a.len() < 3 {
                            continue;
                        }
                        let (area, cen) = area_centroid(&buf_a);
                        if area <= 0.0 {
                            continue;
                        }
                        let zf = a * cen.x + b * cen.y + c;
                        let k = j * nx + i;
                        let base = exits[offs[k]..offs[k + 1]]
                            .iter()
                            .rev()
                            .find(|&&z| z < zf - LINEAR_GUARD)
                            .copied();
                        let gap = zf - base.unwrap_or(0.0);
                        if gap > 0.0 {
                            vol += area * gap;
                            if base.is_some() {
                                part += area * gap;
                            }
                            cells.push(k as u32);
                        }
                    }
                }
            }
            Ok((vol, part, cells))
        })
        .collect::<Result<_, amg_core::Cancelled>>()?;
    let (mut vol, mut part) = (0.0, 0.0);
    let mut counted = vec![false; nx * ny];
    for (v, p, cells) in parts {
        vol += v;
        part += p;
        for k in cells {
            counted[k as usize] = true;
        }
    }
    let n_cells = counted.iter().filter(|&&b| b).count();
    Ok(SupportEstimate {
        volume: vol,
        on_plate_volume: vol - part,
        on_part_volume: part,
        footprint_area: n_cells as f64 * cell * cell,
        cell,
        rays: u32::try_from(nx * ny).unwrap_or(u32::MAX),
        method: SupportMethod::Columns,
    })
}

/// Subdivisions per edge of a face of `area` at cell `s` (faces larger than `4 s²` are
/// subsampled on a barycentric grid of `k²` sub-triangles).
fn subdivisions(area: f64, s: f64) -> usize {
    if area > 4.0 * s * s {
        ((area / (s * s)).sqrt().ceil() as usize).max(1)
    } else {
        1
    }
}

fn support_centroids(
    s: &MeshSpatial,
    fr: &Frame,
    cl: &Classified,
    so: &SupportOpts,
    ctx: &JobCtx,
) -> Result<SupportEstimate, AmError> {
    let m: &IndexedMesh = s.mesh();
    let faces: Vec<(usize, f64)> = cl
        .class
        .iter()
        .enumerate()
        .filter(|(_, c)| **c == FaceClass::Overhang)
        .map(|(f, _)| (f, normal_of(m, &m.tris[f]).1))
        .collect();
    let count = |cell: f64| -> usize {
        faces
            .iter()
            .map(|&(_, a)| {
                let k = subdivisions(a, cell);
                k * k
            })
            .sum()
    };
    let mut cell = so.cell;
    while count(cell) > so.max_rays as usize && faces.len() < count(cell) {
        cell *= 1.25;
    }
    let down = Dir3::new_unchecked(-fr.d);
    let parts: Vec<(f64, f64, f64, u64)> = faces
        .par_chunks(1024)
        .map(|chunk| {
            ctx.checkpoint()?;
            let (mut vol, mut part, mut foot, mut rays) = (0.0, 0.0, 0.0, 0u64);
            for &(f, area) in chunk {
                let tri = m.tris[f];
                let [a, b, c] = tri.map(|i| m.positions[i as usize]);
                let cos = facing(m, FaceId(f as u32), fr.d);
                foot += area * cos;
                let k = subdivisions(area, cell);
                let w = area * cos / (k * k) as f64;
                let kf = k as f64;
                let mut sample = |u: f64, v: f64| {
                    let p = a + (b - a) * (u / kf) + (c - a) * (v / kf);
                    let origin = p - fr.d * RAY_OFFSET;
                    rays += 1;
                    match s.ray_cast(&Ray3 { origin, dir: down }, f64::INFINITY) {
                        Some(h) => {
                            let g = (h.t + RAY_OFFSET) * w;
                            vol += g;
                            part += g;
                        }
                        None => vol += fr.z(p).max(0.0) * w,
                    }
                };
                for i in 0..k {
                    for j in 0..k - i {
                        let (fi, fj) = (i as f64, j as f64);
                        sample(fi + 1.0 / 3.0, fj + 1.0 / 3.0);
                        if i + j + 2 <= k {
                            sample(fi + 2.0 / 3.0, fj + 2.0 / 3.0);
                        }
                    }
                }
            }
            Ok((vol, part, foot, rays))
        })
        .collect::<Result<_, amg_core::Cancelled>>()?;
    let (mut vol, mut part, mut foot, mut rays) = (0.0, 0.0, 0.0, 0u64);
    for (v, p, f, r) in parts {
        vol += v;
        part += p;
        foot += f;
        rays += r;
    }
    Ok(SupportEstimate {
        volume: vol,
        on_plate_volume: vol - part,
        on_part_volume: part,
        footprint_area: foot,
        cell,
        rays: u32::try_from(rays).unwrap_or(u32::MAX),
        method: SupportMethod::FaceCentroids,
    })
}

#[cfg(test)]
#[allow(clippy::expect_used, reason = "test")]
mod tests {
    use super::*;

    fn mesh(tris: Vec<[u32; 3]>) -> IndexedMesh {
        let pos = vec![DVec3::ZERO, DVec3::X, DVec3::Y, DVec3::Z];
        IndexedMesh::new(pos, tris).expect("mesh")
    }

    /// The counting-sort check agrees with the directed-key reference on closed, open,
    /// mis-oriented and doubled meshes.
    #[test]
    fn closed_oriented_matches_reference() {
        let tet = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
        let mut flipped = tet.clone();
        flipped[2] = [1, 3, 2];
        let mut doubled = tet.clone();
        doubled.extend_from_slice(&tet);
        let cases = [
            (tet.clone(), true),
            (tet[..3].to_vec(), false),
            (flipped, false),
            (doubled, false),
            (tet.iter().map(|t| [t[0], t[2], t[1]]).collect(), true),
        ];
        for (tris, want) in cases {
            let m = mesh(tris);
            assert_eq!(closed_oriented(&m), want);
            assert_eq!(closed_oriented_wide(&m), want);
        }
    }
}
